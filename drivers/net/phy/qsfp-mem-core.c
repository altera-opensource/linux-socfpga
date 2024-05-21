// SPDX-License-Identifier: GPL

/* Intel(R) Memory based QSFP driver.
 *
 * Copyright (C) 2020,2024 Intel Corporation. All rights reserved.
 */

#include <linux/bitfield.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/i2c.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/regmap.h>
#include <linux/uaccess.h>
#include <linux/phy/qsfp-mem.h>

#define CONF_OFF	0x20
#define CONF_RST_MOD	BIT(0)
#define CONF_RST_CON	BIT(1)
#define CONF_MOD_SEL	BIT(2)
#define CONF_LOW_POW	BIT(3)
#define CONF_POLL_EN	BIT(4)

#define STAT_OFF	0x28
#define MODPRSL         BIT(0)
#define DELAY_REG       0x38
#define DELAY_VALUE       0xffffff

#define I2C_TX_FIFO     0x40
#define I2C_TX_FIFO_START   BIT(9)
#define I2C_TX_FIFO_STOP    BIT(8)
#define I2C_TX_FIFO_WRITE   (0)

#define I2C_CTRL        0x48
#define I2C_CTRL_EN	BIT(0)
#define I2C_CTRL_BSP	BIT(1)
#define I2C_CTRL_FIFO  GENMASK(3, 2)
#define I2C_CTRL_FIFO_NOT_FULL 3

#define I2C_ISER	0x4c
#define I2C_ISER_TXRDY	BIT(0)
#define I2C_ISER_RXRDY	BIT(1)

#define I2C_ISR             0x50
#define I2C_ISR_NACK_DET    BIT(2)
#define I2C_ISR_ARBLOST_DET BIT(3)
#define I2C_ISR_RX_OVER     BIT(4)
#define I2C_ISR_CLEAR_FLAGS (I2C_ISR_NACK_DET | I2C_ISR_ARBLOST_DET | I2C_ISR_RX_OVER)

#define I2C_STATUS	    0x54
#define I2C_STATUS_CORE	    BIT(0) /* 0 = idle */
#define I2C_TX_FIFO_LVL     0x58

#define I2C_SCL_LOW	0x60
#define COUNT_PERIOD_LOW 170
#define I2C_SCL_HIGH	0x64
#define COUNT_PERIOD_HIGH 80
#define I2C_SDA_HOLD	0x68
#define COUNT_PERIOD_HOLD 60

#define QSFP_CONTROLLER_VER 0x80
#define QSFP_I2C_INIT_DONE  0x90

#define QSFP_SHADOW_CSRS_BASE_OFF	0x100
#define QSFP_SHADOW_CSRS_BASE_END	0x3f8

#define DELAY_US 1000

#define QSFP_CHECK_TIME 500
#define QSFP_CHK_RDY_CNT 1000

#define I2C_QFSP_ADDR       0x50

#define I2C_MAX_TIMEOUT     100

static const struct regmap_range qsfp_mem_regmap_range[] = {
	regmap_reg_range(CONF_OFF, STAT_OFF),
	regmap_reg_range(QSFP_SHADOW_CSRS_BASE_OFF, QSFP_SHADOW_CSRS_BASE_END),
};

static const struct regmap_access_table qsfp_mem_access_table = {
	.yes_ranges	= qsfp_mem_regmap_range,
	.n_yes_ranges	= ARRAY_SIZE(qsfp_mem_regmap_range),
};

static void qsfp_init_i2c(struct qsfp *qsfp)
{
	writel(I2C_ISER_TXRDY | I2C_ISER_RXRDY, qsfp->base + I2C_ISER);
	writel(COUNT_PERIOD_LOW, qsfp->base + I2C_SCL_LOW);
	writel(COUNT_PERIOD_HIGH, qsfp->base + I2C_SCL_HIGH);
	writel(COUNT_PERIOD_HOLD, qsfp->base + I2C_SDA_HOLD);

	writel(FIELD_PREP(I2C_CTRL_FIFO, I2C_CTRL_FIFO_NOT_FULL) |
			I2C_CTRL_EN | I2C_CTRL_BSP, qsfp->base + I2C_CTRL);
}

static int qsfp_ctrl_version_read(struct qsfp *qsfp)
{
	u32 ver;

	ver = readl(qsfp->base + QSFP_CONTROLLER_VER);
	dev_info(qsfp->dev, "QSFP I2C CONTROLLER VERSION: 0x%x \n", ver);

	return ver;
}

static bool qsfp_is_i2c_init_done(struct qsfp *qsfp)
{
	u64 init_done;

	init_done = readq(qsfp->base + QSFP_I2C_INIT_DONE);
	return (init_done & 1) ? true : false;
}

static const struct regmap_config mmio_cfg = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.fast_io = true,
	.rd_table = &qsfp_mem_access_table,
	.max_register = QSFP_SHADOW_CSRS_BASE_END,
};

static int i2c_txcmp(struct qsfp *qsfp)
{
	u32 fifo_lvl;

	return readl_poll_timeout(qsfp->base + I2C_TX_FIFO_LVL, fifo_lvl, !fifo_lvl,
				  10, I2C_MAX_TIMEOUT);
}

static int i2c_send(struct qsfp *qsfp, int data)
{
	int ret = i2c_txcmp(qsfp);

	if (ret)
		return ret;

	writel(data, qsfp->base + I2C_TX_FIFO);
	return 0;
}

static int send_qsfp_cmd_page0(struct qsfp *qsfp)
{
	int st, ret;

	i2c_send(qsfp, I2C_TX_FIFO_START | (I2C_QFSP_ADDR << 1) | I2C_TX_FIFO_WRITE);
	i2c_send(qsfp, 0x7f);
	i2c_send(qsfp, I2C_TX_FIFO_STOP);

	ret = i2c_txcmp(qsfp);
	if (ret)
		return ret;

	/* Check status */
	st = readl(qsfp->base + I2C_ISR);
	dev_dbg(qsfp->dev, "QSFP I2C ISR = 0x%02X STAT = 0x%x\n", st,
		readl(qsfp->base + I2C_STATUS));
	if (st & I2C_ISR_CLEAR_FLAGS) {
		writel(I2C_ISR_CLEAR_FLAGS, qsfp->base + I2C_ISR);
		return 1;
	}

	return 0;
}

static int qsfp_init(struct qsfp *qsfp)
{
	int cnt;

	/* Reset QSFP Module and QSFP Controller	*/
	writeq(CONF_RST_MOD | CONF_RST_CON | CONF_MOD_SEL, qsfp->base + CONF_OFF);

	udelay(DELAY_US);

	writeq(CONF_MOD_SEL, qsfp->base + CONF_OFF);

	/*TODO: Version check to be coded in later release */
	qsfp_ctrl_version_read(qsfp);

	if (qsfp_is_i2c_init_done(qsfp))
		goto i2c_init_done;

	/* Initialize Intel FPGA Avalon I2C (Master) Core */
	qsfp_init_i2c(qsfp);

	writel(I2C_ISR_CLEAR_FLAGS, qsfp->base + I2C_ISR);

	writeq(DELAY_VALUE, qsfp->base + DELAY_REG);

	/* Check QSFP Module Ready */
	for (cnt = 0; cnt < QSFP_CHK_RDY_CNT; cnt++) {
		/* try to send a command to change page 0 */
		if (!send_qsfp_cmd_page0(qsfp)) {
			dev_info(qsfp->dev, "QSFP module ready after waiting for %dms", cnt);
			break;
		}

		udelay(DELAY_US);
	}

	if (cnt >= QSFP_CHK_RDY_CNT) {
		dev_err(qsfp->dev, "QSFP I2C check ready timeout error");
		return -ETIMEDOUT;
	}

i2c_init_done:
	/* Enable Polling mode */
	writeq(CONF_POLL_EN | CONF_MOD_SEL, qsfp->base + CONF_OFF);
	return 0;
}

int check_qsfp_plugin(struct qsfp *qsfp)
{
	u64 status;

	status = readq(qsfp->base + STAT_OFF);

	return (!(status & MODPRSL));
}
EXPORT_SYMBOL_GPL(check_qsfp_plugin);

static ssize_t qsfp_connected_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct qsfp *qsfp = dev_get_drvdata(dev);
	u32 plugin;

	mutex_lock(&qsfp->lock);
	plugin = check_qsfp_plugin(qsfp) && (qsfp->init == QSFP_INIT_DONE);
	mutex_unlock(&qsfp->lock);

	return sysfs_emit(buf, "%u\n", plugin);
}
static DEVICE_ATTR_RO(qsfp_connected);

static struct attribute *qsfp_mem_attrs[] = {
	&dev_attr_qsfp_connected.attr,
	NULL,
};

static const struct attribute_group qsfp_mem_group = {
	.attrs = qsfp_mem_attrs,
};

const struct attribute_group *qsfp_mem_groups[] = {
	&qsfp_mem_group,
	NULL,
};
EXPORT_SYMBOL_GPL(qsfp_mem_groups);

static void qsfp_check_hotplug(struct work_struct *work)
{
	struct delayed_work *dwork;
	struct qsfp *qsfp;
	u64 status;

	dwork = to_delayed_work(work);
	qsfp = container_of(dwork, struct qsfp, dwork);

	mutex_lock(&qsfp->lock);

	status = readq(qsfp->base + STAT_OFF);
	dev_dbg(qsfp->dev, "qsfp status 0x%llx\n", status);

	if (check_qsfp_plugin(qsfp) && qsfp->init == QSFP_INIT_RESET) {
		dev_info(qsfp->dev, "detected QSFP plugin\n");
		if (!qsfp_init(qsfp))
			WRITE_ONCE(qsfp->init, QSFP_INIT_DONE);
	} else if (!check_qsfp_plugin(qsfp) &&
		   qsfp->init == QSFP_INIT_DONE) {
		dev_info(qsfp->dev, "detected QSFP unplugin\n");
		WRITE_ONCE(qsfp->init, QSFP_INIT_RESET);
	}
	mutex_unlock(&qsfp->lock);

	schedule_delayed_work(&qsfp->dwork, msecs_to_jiffies(QSFP_CHECK_TIME));
}

int qsfp_init_work(struct qsfp *qsfp)
{
	qsfp->init = QSFP_INIT_RESET;
	INIT_DELAYED_WORK(&qsfp->dwork, qsfp_check_hotplug);
	qsfp_check_hotplug(&qsfp->dwork.work);
	return 0;
}
EXPORT_SYMBOL_GPL(qsfp_init_work);

int qsfp_register_regmap(struct qsfp *qsfp)
{
	struct device *dev = qsfp->dev;

	qsfp->regmap = devm_regmap_init_mmio(dev, qsfp->base, &mmio_cfg);
	if (IS_ERR(qsfp->regmap))
		dev_err(dev, "Failed to create qsfp regmap\n");

	return PTR_ERR_OR_ZERO(qsfp->regmap);
}
EXPORT_SYMBOL_GPL(qsfp_register_regmap);

void qsfp_remove_device(struct qsfp *qsfp)
{
	writeq(CONF_MOD_SEL, qsfp->base + CONF_OFF);
	cancel_delayed_work_sync(&qsfp->dwork);
}
EXPORT_SYMBOL_GPL(qsfp_remove_device);
MODULE_LICENSE("GPL");
