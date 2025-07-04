// SPDX-License-Identifier: GPL-2.0

/* Intel(R) Memory based SFP driver for platform devices.
 *
 * Copyright (C) 2025 Intel Corporation. All rights reserved.
 */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/phy/sfp-mem.h>
#include <linux/processor.h>
#include <linux/slab.h>
#include <linux/platform_device.h>

#define INTEL_SFP_MEM_CONTROLLER_NAME "sfp-mem-ctrl"

struct platform_device;

static int sfp_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *region = NULL;
	struct resource *sfpconfig = NULL;
	struct sfp *sfp = NULL;
	int ret;

	sfp = devm_kzalloc(dev, sizeof(*sfp), GFP_KERNEL);
	if (!sfp)
		return -ENOMEM;

	sfp->dev = dev;
	mutex_init(&sfp->lock);
	platform_set_drvdata(pdev, sfp);

	/* SFP Mem address space */
	sfpconfig = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						  INTEL_SFP_MEM_CONTROLLER_NAME);
	if (!sfpconfig) {
		dev_err(dev, "resource %s not defined\n", INTEL_SFP_MEM_CONTROLLER_NAME);
		return -ENODEV;
	}

	region = devm_request_mem_region(dev, sfpconfig->start,
					 resource_size(sfpconfig), dev_name(dev));
	if (!region) {
		dev_err(dev, "unable to request %s\n", INTEL_SFP_MEM_CONTROLLER_NAME);
		return -EBUSY;
	}
	sfp->base = devm_ioremap(dev, region->start, resource_size(region));
	if (!(sfp->base)) {
		dev_err(dev, "ioremap of %s failed!", INTEL_SFP_MEM_CONTROLLER_NAME);
		return -ENOMEM;
	}

	ret = sfp_init_work(sfp);
	if (ret) {
		dev_err_probe(dev, ret,
			      "Failed to initialize delayed work to read SFP\n");
		goto exit;
	}

	ret = sfp_register_regmap(sfp);
	if (ret)
		goto cancel_work;

	return 0;

cancel_work:
	sfp_remove_device(sfp);
exit:
	mutex_destroy(&sfp->lock);
	return ret;
}

static void sfp_platform_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sfp *sfp = dev_get_drvdata(dev);

	sfp_remove_device(sfp);
	mutex_destroy(&sfp->lock);
}

static const struct of_device_id intel_fpga_sfp_mem_ids[] = {
	{ .compatible = "altera,sfp-mem", .data = NULL, },
	{},
};

MODULE_DEVICE_TABLE(of, intel_fpga_sfp_mem_ids);

static struct platform_driver sfp_driver = {
	.probe      = sfp_platform_probe,
	.remove     = sfp_platform_remove,
	.suspend    = NULL,
	.resume     = NULL,
	.driver     = {
		.name   = "sfp-mem",
		.dev_groups = sfp_mem_groups,
		.owner  = THIS_MODULE,
		.of_match_table = intel_fpga_sfp_mem_ids,
	},
};

module_platform_driver(sfp_driver);
MODULE_DESCRIPTION("Altera(R) Memory based SFP Platform driver");
MODULE_AUTHOR("Altera Corporation");
MODULE_LICENSE("GPL");
