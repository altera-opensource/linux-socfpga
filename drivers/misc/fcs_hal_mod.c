// SPDX-License-Identifier: GPL-2.0-or-later OR MIT
/*
 * Copyright (C) 2024 Intel Corporation
 */

#include <linux/of_platform.h>
#include <misc/socfpga_fcs_hal.h>
#include <linux/platform_device.h>
#include <linux/of.h>

struct socfpga_fcs_priv *priv;
EXPORT_SYMBOL(priv);

static int fcs_hal_driver_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	priv = devm_kzalloc(dev, sizeof(struct socfpga_fcs_priv), GFP_KERNEL);
	if (!priv) {
		dev_err(dev, "Failed to allocate memory for priv\n");
		return -ENOMEM;
	}

	ret = hal_fcs_init(dev);
	if (ret) {
		dev_err(dev, "Failed to initialize HAL FCS\n");
		return ret;
	}

	return 0;
}

static const struct of_device_id fcs_hal_of_match[] = {
	{.compatible = "intel,agilex5-soc-fcs-hal",
	 .data = NULL,
	},
	{},
};

static struct platform_driver fcs_hal_driver = {
	.probe = fcs_hal_driver_probe,
	.driver = {
		.name = "socfpga-fcs-hal",
		.of_match_table = of_match_ptr(fcs_hal_of_match),
	},
};

MODULE_DEVICE_TABLE(of, fcs_hal_of_match);

static int __init fcs_hal_init(void)
{
	struct device_node *fw_np;
	struct device_node *np;
	int ret;

	fw_np = of_find_node_by_name(NULL, "svc");
	if (!fw_np)
		return -ENODEV;

	of_node_get(fw_np);
	np = of_find_matching_node(fw_np, fcs_hal_of_match);
	if (!np) {
		of_node_put(fw_np);
		return -ENODEV;
	}

	of_node_put(np);
	ret = of_platform_populate(fw_np, fcs_hal_of_match, NULL, NULL);
	of_node_put(fw_np);
	if (ret)
		return ret;

	return platform_driver_register(&fcs_hal_driver);
}

static void __exit fcs_hal_exit(void)
{
	hal_fcs_cleanup();

	return platform_driver_unregister(&fcs_hal_driver);
}

module_init(fcs_hal_init);
module_exit(fcs_hal_exit);

MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("Altera FGPA Crypto Services HAL Driver");
MODULE_AUTHOR("Sagar Khadgi, Santosh Male, Balsundar Ponnusamy");
