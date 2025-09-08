// SPDX-License-Identifier: GPL-2.0

/* Intel(R) Memory based SFP driver.
 *
 * Copyright (C) 2025 Intel Corporation. All rights reserved.
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
#include <linux/phy/sfp-mem.h>
#include <linux/sfp.h>
#include <linux/ethtool.h>

#define CONF_OFF	0x20
#define CONF_RST_CON	BIT(1)
#define CONF_LOW_POW	BIT(3)
#define CONF_POLL_EN	BIT(4)
#define CONF_A0PAGE_UPD BIT(5)
#define CONF_PAGE_SEL	GENMASK(7, 6)

#define STAT_OFF	0x28
#define MODPRSL             BIT(0)
#define A0_UPD_RDY_TO_START BIT(34)
#define A0_UPD_IN_PROG	    BIT(35)
#define A0_UPD_COMPLETE	    BIT(36)
#define A2_UPD_IN_PROG	    BIT(37)
#define A2_UPD_COMPLETE	    BIT(38)
#define A0_UPD_ERROR        BIT(39)
#define A2_UPD_ERROR	    BIT(40)

#define DELAY_REG       0x38
#define DELAY_VALUE       0xffffff

#define I2C_TX_FIFO     0x40
#define I2C_TX_FIFO_START   BIT(9)
#define I2C_TX_FIFO_STOP    BIT(8)
#define I2C_TX_FIFO_WRITE   (0)
#define I2C_TX_FIFO_READ    (1)

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

#define SFP_CONTROLLER_VER 0x80
#define SFP_I2C_INIT_DONE  0x90

#define SFP_SINGLE_I2C_MASTER_VER 0x0200
#define DELAY_US 1000

#define SFP_CHECK_TIME 500
#define SFP_CHK_RDY_CNT 1000

#define I2C_SFP_ADDR       0x50

#define I2C_MAX_TIMEOUT     100

#define A0_START_ADDR   0x800
#define ADDR_MODE	 0x5C
#define MULTI_PAGE_SEL   0x40
#define A0_END_ADDR     0x880

#define MODE_SEL	BIT(2)
#define PAGE_SEL	BIT(11)
#define A2_START_ADDR	0x100
#define A2_END_ADDR	0x700

static const struct regmap_range sfp_mem_regmap_range[] = {
	regmap_reg_range(CONF_OFF, SFP_CONTROLLER_VER),
	regmap_reg_range(A0_START_ADDR, A0_END_ADDR),
	regmap_reg_range(A2_START_ADDR, A2_END_ADDR),
};

static const struct regmap_access_table sfp_mem_access_table = {
	.yes_ranges	= sfp_mem_regmap_range,
	.n_yes_ranges	= ARRAY_SIZE(sfp_mem_regmap_range),
};

static void sfp_init_i2c(struct sfp *sfp)
{
	writel(I2C_ISER_TXRDY | I2C_ISER_RXRDY, sfp->base + I2C_ISER);
	writel(COUNT_PERIOD_LOW, sfp->base + I2C_SCL_LOW);
	writel(COUNT_PERIOD_HIGH, sfp->base + I2C_SCL_HIGH);
	writel(COUNT_PERIOD_HOLD, sfp->base + I2C_SDA_HOLD);

	writel(FIELD_PREP(I2C_CTRL_FIFO, I2C_CTRL_FIFO_NOT_FULL) |
			I2C_CTRL_EN | I2C_CTRL_BSP, sfp->base + I2C_CTRL);
}

static const struct regmap_config mmio_cfg = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.fast_io = true,
	.rd_table = &sfp_mem_access_table,
	.max_register = A0_END_ADDR,
};

static bool sfp_init(struct sfp *sfp)
{
	/* Reset SFP Module */
	writel(CONF_RST_CON, sfp->base + CONF_OFF);

	udelay(DELAY_US);

	/* Initialize Intel FPGA Avalon I2C (Master) Core */
	sfp_init_i2c(sfp);

	writel(I2C_ISR_CLEAR_FLAGS, sfp->base + I2C_ISR);

	writel(DELAY_VALUE, sfp->base + DELAY_REG);

	return true;
}

bool check_sfp_plugin(struct sfp *sfp)
{
	u64 status;

	status = readq(sfp->base + STAT_OFF);

	return (status & MODPRSL);
}
EXPORT_SYMBOL_GPL(check_sfp_plugin);

static ssize_t sfp_connected_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sfp *sfp = dev_get_drvdata(dev);
	bool plugin;

	mutex_lock(&sfp->lock);
	plugin = check_sfp_plugin(sfp) && (sfp->state == SFP_INIT_DONE);
	mutex_unlock(&sfp->lock);

	return sysfs_emit(buf, "%u\n", plugin);
}
static DEVICE_ATTR_RO(sfp_connected);

static struct attribute *sfp_mem_attrs[] = {
	&dev_attr_sfp_connected.attr,
	NULL,
};

static const struct attribute_group sfp_mem_group = {
	.attrs = sfp_mem_attrs,
};

const struct attribute_group *sfp_mem_groups[] = {
	&sfp_mem_group,
	NULL,
};
EXPORT_SYMBOL_GPL(sfp_mem_groups);

static void sfp_check_hotplug(struct work_struct *work)
{
	struct delayed_work *dwork;
	struct sfp *sfp;
	int poll_timeout = 0;
	bool is_sfp_pluggedin = false;
	u64 conf_off = 0;
	u64 stat_off = 0;
	u64 page_sel = 0;
	u64 sfp_sel  = 0;

	dwork = to_delayed_work(work);
	sfp = container_of(dwork, struct sfp, dwork);

	mutex_lock(&sfp->lock);

	conf_off = readq(sfp->base + CONF_OFF);
	stat_off = readq(sfp->base + STAT_OFF);

	is_sfp_pluggedin = check_sfp_plugin(sfp);

	if ((!is_sfp_pluggedin) && (sfp->state != SFP_DETECT)) {
		dev_info_ratelimited(sfp->dev, "detected SFP plug out\n");
		WRITE_ONCE(sfp->state, SFP_DETECT);
	}

	/* in case of error observed then we need to take defensive action */
	if (stat_off & A2_UPD_ERROR)
		WRITE_ONCE(sfp->state, SFP_A2_UPDATE_ERROR);

	if (stat_off & A0_UPD_ERROR)
		WRITE_ONCE(sfp->state, SFP_A0_UPDATE_ERROR);

	switch (sfp->state) {
	case SFP_DETECT:
		if (is_sfp_pluggedin) {
			dev_info_ratelimited(sfp->dev, "detected SFP plugin\n");
			WRITE_ONCE(sfp->state, SFP_INIT_RESET);
		}
		break;

	case SFP_INIT_RESET:
		if (sfp_init(sfp))
			WRITE_ONCE(sfp->state, SFP_INIT_DONE);
		sfp->tolerance_count = IP_RESPONSE_TOLERANCE_LIMIT;
		break;

	case SFP_INIT_DONE:
		if (!(stat_off & A0_UPD_RDY_TO_START))
			dev_warn_ratelimited(sfp->dev, "SFP FSM should had been in A0 ready state\n");
		else {
			/* Driver is ready and so is the RTL now start the A0 update */
			writel(CONF_A0PAGE_UPD, sfp->base + CONF_OFF);

			poll_timeout = readq_poll_timeout(sfp->base + STAT_OFF, stat_off,
							  ((stat_off & A0_UPD_COMPLETE) ||
							  (stat_off & A0_UPD_IN_PROG)),
							  10, I2C_MAX_TIMEOUT);
			if (!poll_timeout) {
				WRITE_ONCE(sfp->state, SFP_A0PAGE_UPDATE_INPROG);
				sfp->tolerance_count = IP_RESPONSE_TOLERANCE_LIMIT;
				break;
			} else {
				dev_warn_ratelimited(sfp->dev,
						     "SFP FSM state change to SFP_A0PAGE_UPDATE_INPROG unexpected delay\n");
			}

			if (--sfp->tolerance_count == IP_IRRESPONSIVE) {
				WRITE_ONCE(sfp->state, SFP_INIT_RESET);
				break;
			}
		}

		break;

	case SFP_A0PAGE_UPDATE_INPROG:
		#define SFP_PAGE_BIT BIT(6)
		#define SFP_MODE_BIT BIT(7)

		if (stat_off & A0_UPD_COMPLETE) {
			page_sel = readl(sfp->base + A0_START_ADDR + MULTI_PAGE_SEL) & PAGE_SEL ? SFP_PAGE_BIT : 0;
			sfp_sel = readl(sfp->base + A0_START_ADDR + ADDR_MODE) & MODE_SEL ? SFP_MODE_BIT : 0;
			sfp_sel |= page_sel;

			/* write the bits to update the multi page info and the addr mode
			 * supported info to the design
			 */
			writeq(sfp_sel, sfp->base + CONF_OFF);

			WRITE_ONCE(sfp->state, SFP_A0PAGE_UPDATE_COMPLETE);
		}
		break;

	case SFP_A0PAGE_UPDATE_COMPLETE:

		writeq(CONF_POLL_EN, sfp->base + CONF_OFF);
		WRITE_ONCE(sfp->state, SFP_A2PAGE_UPDATE_INPROG);
		sfp->tolerance_count = IP_RESPONSE_TOLERANCE_LIMIT;

		break;

	case SFP_A2PAGE_UPDATE_INPROG:
		if (stat_off & A2_UPD_COMPLETE) {
			WRITE_ONCE(sfp->state, SFP_A2PAGE_UPDATE_COMPLETE);
		} else {
			if (--sfp->tolerance_count == IP_IRRESPONSIVE)
				WRITE_ONCE(sfp->state, SFP_INIT_RESET);
		}
		break;

	case SFP_A0_UPDATE_ERROR:
		dev_err(sfp->dev, "SFP: A0 Page error observed, restarting A0 page operation\n");
		WRITE_ONCE(sfp->state, SFP_INIT_RESET);
		break;

	case SFP_A2_UPDATE_ERROR:
		dev_err(sfp->dev, "SFP: A2 Page error observed, restarting A2 page operation\n");
		WRITE_ONCE(sfp->state, SFP_A0PAGE_UPDATE_COMPLETE);
		break;

	case SFP_A2PAGE_UPDATE_COMPLETE:
	default:
		break;
	}

	mutex_unlock(&sfp->lock);

	schedule_delayed_work(&sfp->dwork, msecs_to_jiffies(SFP_CHECK_TIME));
}

int sfp_register_regmap(struct sfp *sfp)
{
	struct device *dev = sfp->dev;

	sfp->regmap = devm_regmap_init_mmio(dev, sfp->base, &mmio_cfg);
	if (IS_ERR(sfp->regmap))
		dev_err(dev, "Failed to create sfp regmap\n");

	return PTR_ERR_OR_ZERO(sfp->regmap);
}
EXPORT_SYMBOL_GPL(sfp_register_regmap);

void sfp_remove_device(struct sfp *sfp)
{
	cancel_delayed_work_sync(&sfp->dwork);
}
EXPORT_SYMBOL_GPL(sfp_remove_device);

/* copy the A0, A2 page content */
static void sfp_page_copy(struct sfp *sfp)
{
	u32 *page;

	page = (u32 *)sfp->a0_page.a0_page;
	for (u16 update_eeprom = 0, pg_byte = 0;
	    update_eeprom < sizeof(sfp->a0_page); pg_byte += 1, update_eeprom += 4)
		page[pg_byte] =
			readl(sfp->base + A0_START_ADDR + update_eeprom);

	page = (u32 *)sfp->a2_page.a2_page;
	for (u16 update_eeprom = 0, pg_byte = 0;
	    update_eeprom < sizeof(sfp->a2_page); pg_byte += 1, update_eeprom += 4)
		page[pg_byte] =
			readl(sfp->base + A2_START_ADDR + update_eeprom);
}

static int sfp_module_info(struct sfp *sfp, struct ethtool_modinfo *modinfo)
{
	/* At least A0 page update is completed */
	if (!(sfp->state >= SFP_A0PAGE_UPDATE_COMPLETE))
		return -EIO;

	sfp_page_copy(sfp);

	if ((sfp->a0_page.a0.ext.sff8472_compliance) &&
	    (!((sfp->a0_page.a0.ext.diagmon) & SFP_DIAGMON_ADDRMODE))) {
		modinfo->type = ETH_MODULE_SFF_8472;
		//modinfo->eeprom_len = ETH_MODULE_SFF_8472_LEN;
		modinfo->eeprom_len = A0_EEPROM_SIZE + A2_EEPROM_SIZE;
	} else {
		modinfo->type = ETH_MODULE_SFF_8079;
		modinfo->eeprom_len = ETH_MODULE_SFF_8079_LEN;
	}

	return 0;
}

static int sfp_module_eeprom_calc(struct sfp *sfp,
				  u16 offset,
				  u16 len_dump,
				  u8 *data)
{
	u16 from_offset = offset;
	u16 til_offset  = offset + len_dump;

	if ((len_dump == 0) || (til_offset > sizeof(sfp->a0_page) + sizeof(sfp->a2_page)))
		return -EINVAL;

	/* offset is within A0 page size */
	if (til_offset <= ETH_MODULE_SFF_8079_LEN) {
		/* copy from the offset the desired length */
		memcpy(data, (u8 *)sfp->a0_page.a0_page + from_offset, len_dump);
	}

	/* offset requested is on A2 page range */
	else if ((from_offset >= ETH_MODULE_SFF_8079_LEN) && (til_offset <= A2_EEPROM_SIZE)) {
		memcpy(data,
		       (u8 *)sfp->a2_page.a2_page + from_offset,
		       len_dump);
	} else {
		/* requested dump covers both A0 and A2 */
		u16 len;

		len = ETH_MODULE_SFF_8079_LEN - from_offset;
		memcpy(data,
		       (u8 *)sfp->a0_page.a0_page + from_offset,
			len);

		memcpy(data + len + 1, (u8 *)sfp->a2_page.a2_page, len_dump - len);
	}

	return 0;
}

static int sfp_module_eeprom(struct sfp *sfp,
			     struct ethtool_eeprom *ee,
			     u8 *data)
{
	sfp_page_copy(sfp);

	return sfp_module_eeprom_calc(sfp, ee->offset, ee->len, data);
}

static int sfp_module_eeprom_by_page(struct sfp *sfp,
				     const struct ethtool_module_eeprom *page_data,
				     struct netlink_ext_ack *extack)
{
	int ret = 0;
	u16 abs_offset;

	if ((page_data->page == 0) && (page_data->length == 1))
		return 0;

	abs_offset = (page_data->page * ETH_MODULE_EEPROM_PAGE_LEN) + page_data->offset;

	if (abs_offset > ETH_MODULE_EEPROM_PAGE_LEN)
		return -EINVAL;

	ret = sfp_module_eeprom_calc(sfp, page_data->offset,
				     page_data->length, page_data->data);

	return ret;
}

static void unused_func(struct sfp *sfp)
{
	return;
}

static const struct sfp_socket_ops sfp_module_ops = {
	.start = unused_func,
	.stop =  unused_func,
	.attach =  unused_func,
	.module_info = sfp_module_info,
	.module_eeprom = sfp_module_eeprom,
	.module_eeprom_by_page = sfp_module_eeprom_by_page,
};

int sfp_init_work(struct sfp *sfp)
{
	sfp->state = SFP_DETECT;

	sfp->sfp_bus = sfp_register_socket(sfp->dev, sfp, &sfp_module_ops);
	if (!sfp->sfp_bus)
		return -ENOMEM;

	INIT_DELAYED_WORK(&sfp->dwork, sfp_check_hotplug);
	schedule_delayed_work(&sfp->dwork, msecs_to_jiffies(SFP_CHECK_TIME));
	return 0;
}
EXPORT_SYMBOL_GPL(sfp_init_work);

MODULE_LICENSE("GPL");
