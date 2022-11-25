// SPDX-License-Identifier: GPL-2.0

/* Intel(R) Memory based QSFP driver.
 *
 * Copyright (C) 2020 Intel Corporation. All rights reserved.
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
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/processor.h>
#include <linux/slab.h>

#define CONF_OFF	0x20
#define CONF_RST_MOD	BIT(0)
#define CONF_RST_CON	BIT(1)
#define CONF_MOD_SEL	BIT(2)
#define CONF_LOW_POW	BIT(3)
#define CONF_POLL_EN	BIT(4)

#define STAT_OFF	0x28

#define I2C_CTRL        0x48
#define I2C_CTRL_EN	BIT(0)
#define I2C_CTRL_BSP	BIT(1)
#define I2C_CTRL_FIFO  GENMASK(3, 2)
#define I2C_CTRL_FIFO_NOT_FULL 3

#define I2C_ISER	0x4c
#define I2C_ISER_TXRDY	BIT(0)
#define I2C_ISER_RXRDY	BIT(1)
#define I2C_SCL_LOW	0x60
#define COUNT_PERIOD_LOW 0x82
#define I2C_SCL_HIGH	0x64
#define COUNT_PERIOD_HIGH 0x3c
#define I2C_SDA_HOLD	0x68
#define COUNT_PERIOD_HOLD 0x28

#define QSFP_SHADOW_CSRS_BASE_OFF	0x100
#define QSFP_SHADOW_CSRS_BASE_END	0x3f8

#define DELAY_US 1000

struct qsfp {
	void __iomem *base;
	struct regmap *regmap;
};

static const struct regmap_range qsfp_mem_regmap_range[] = {
	regmap_reg_range(CONF_OFF, STAT_OFF),
	regmap_reg_range(QSFP_SHADOW_CSRS_BASE_OFF, QSFP_SHADOW_CSRS_BASE_END),
};

static const struct regmap_access_table qsfp_mem_access_table = {
	.yes_ranges	= qsfp_mem_regmap_range,
	.n_yes_ranges	= ARRAY_SIZE(qsfp_mem_regmap_range),
};

static void qsfp_init_i2c(struct device *dev, struct qsfp *qsfp)
{
	writel(I2C_ISER_TXRDY | I2C_ISER_RXRDY, qsfp->base + I2C_ISER);
    udelay(DELAY_US);
	writel(COUNT_PERIOD_LOW, qsfp->base + I2C_SCL_LOW);
	writel(COUNT_PERIOD_HIGH, qsfp->base + I2C_SCL_HIGH);
	writel(COUNT_PERIOD_HOLD, qsfp->base + I2C_SDA_HOLD);
	writel(FIELD_PREP(I2C_CTRL_FIFO, I2C_CTRL_FIFO_NOT_FULL) |
	       I2C_CTRL_EN | I2C_CTRL_BSP, qsfp->base + I2C_CTRL);
}

static const struct regmap_config mmio_cfg = {
	.reg_bits = 64,
	.reg_stride = 8,
	.val_bits = 64,
	.fast_io = true,
	.rd_table = &qsfp_mem_access_table,
	.max_register = QSFP_SHADOW_CSRS_BASE_END,
};

#define INTEL_QSFP_MEM_CONTROLLER_NAME "qsfp-mem-ctrl"

static int qsfp_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct resource *region = NULL;
    struct resource *qsfpConfig = NULL;
	struct qsfp *qsfp = NULL;

    printk("%s:%d \r\n", __FUNCTION__,__LINE__);
	qsfp = devm_kzalloc(dev, sizeof(*qsfp), GFP_KERNEL);
	if (!qsfp)
		return -ENOMEM;

	platform_set_drvdata(pdev, qsfp);

    /* QSFP Mem address space */
    qsfpConfig = platform_get_resource_byname(pdev, IORESOURCE_MEM, INTEL_QSFP_MEM_CONTROLLER_NAME);
	if (!qsfpConfig) {
		dev_err(dev, "resource %s not defined\n", INTEL_QSFP_MEM_CONTROLLER_NAME);
		return -ENODEV;
	}

	region = devm_request_mem_region(dev, qsfpConfig->start,
					 resource_size(qsfpConfig), dev_name(dev));
	if (!region) {
		dev_err(dev, "unable to request %s\n", INTEL_QSFP_MEM_CONTROLLER_NAME);
		return -EBUSY;
	}

	qsfp->base = devm_ioremap(dev, region->start, resource_size(region));
	if (!(qsfp->base)) {
		dev_err(dev, "ioremap of %s failed!", INTEL_QSFP_MEM_CONTROLLER_NAME);
		return -ENOMEM;
	}

#if 1
    writeq(CONF_RST_MOD | CONF_RST_CON | CONF_MOD_SEL,
            qsfp->base + CONF_OFF);
    udelay(DELAY_US);
    writeq(CONF_MOD_SEL, qsfp->base + CONF_OFF);
    udelay(DELAY_US);

    qsfp_init_i2c(dev, qsfp);

    writeq(CONF_POLL_EN | CONF_MOD_SEL, qsfp->base + CONF_OFF);
    udelay(DELAY_US);

#else
    printk("%s:%d Base: 0x%p Data: 0x%x 0x%lx \r\n", __FUNCTION__,__LINE__, (qsfp->base + CONF_OFF), 
            (*(u32*)(qsfp->base + CONF_OFF)), (CONF_RST_MOD | CONF_RST_CON | CONF_MOD_SEL)); 
	writel(CONF_RST_MOD | CONF_RST_CON | CONF_MOD_SEL,
	       qsfp->base + CONF_OFF);
    printk("%s:%d After Write - Base: 0x%p Data: 0x%lx\r\n", __FUNCTION__,__LINE__, (qsfp->base + CONF_OFF),
               readl(qsfp->base + CONF_OFF));
	udelay(DELAY_US);
    printk("%s:%d Base: 0x%p Data: 0x%x 0x%lx \r\n", __FUNCTION__,__LINE__, (qsfp->base + CONF_OFF),
            (*(u32*)(qsfp->base + CONF_OFF)), (CONF_MOD_SEL));
	writel(CONF_MOD_SEL, qsfp->base + CONF_OFF);
    printk("%s:%d After Write - Base: 0x%p Data: 0x%lx\r\n", __FUNCTION__,__LINE__, (qsfp->base + CONF_OFF),
               readl(qsfp->base + CONF_OFF));
	udelay(DELAY_US);

	qsfp_init_i2c(dev, qsfp);

    printk("%s:%d Base: 0x%p Data: 0x%x 0x%lx \r\n", __FUNCTION__,__LINE__, (qsfp->base + CONF_OFF),
               (*(u32*)(qsfp->base + CONF_OFF)), (CONF_POLL_EN | CONF_MOD_SEL));
	writel(CONF_POLL_EN | CONF_MOD_SEL, qsfp->base + CONF_OFF);
    printk("%s:%d After Write - Base: 0x%p Data: 0x%lx\r\n", __FUNCTION__,__LINE__, (qsfp->base + CONF_OFF),
               readl(qsfp->base + CONF_OFF));
	udelay(DELAY_US);
#endif

	qsfp->regmap = devm_regmap_init_mmio(dev, qsfp->base, &mmio_cfg);
	if (IS_ERR(qsfp->regmap))
		dev_err(dev, "Failed to create qsfp regmap\n");

    printk("%s:%d Probe done. Everything OK\r\n", __FUNCTION__,__LINE__);
	return PTR_ERR_OR_ZERO(qsfp->regmap);
}

static int qsfp_remove(struct platform_device *pdev)
{
	struct qsfp *qsfp = platform_get_drvdata(pdev);

	writeq(CONF_MOD_SEL, qsfp->base + CONF_OFF);

    return 0;
}

#define FME_FEATURE_ID_QSFP 0x13

static const struct of_device_id intel_fpga_qsfp_mem_ids[] = {
    { .compatible = "intel,qsfp-mem",
        .data = NULL, },
    {},
};
MODULE_DEVICE_TABLE(of, intel_fpga_qsfp_mem_ids);

static struct platform_driver qsfp_driver = {
    .probe		= qsfp_probe,
    .remove		= qsfp_remove,
    .suspend	= NULL,
    .resume		= NULL,
    .driver		= {
        .name	= "intel,qsfp-mem",
        .owner	= THIS_MODULE,
        .of_match_table = intel_fpga_qsfp_mem_ids,
    },
};

module_platform_driver(qsfp_driver);

MODULE_DESCRIPTION("Intel(R) Memory based QSFP driver");
MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("GPL v2");
