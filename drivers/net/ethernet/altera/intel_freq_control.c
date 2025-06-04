// SPDX-License-Identifier: GPL-2.0
/* Intel FPGA Clock Cleaner Frequency Adjustment Driver
 * Copyright (C) 2015-2016 Altera Corporation. All rights reserved.
 * Copyright (C) 2017-2023 Intel Corporation. All rights reserved.
 *
 * Author(s):
 *	Markos Papadonikolakis <markos.papadonikolakis@intel.com>
 *	Lubana Badakar <lubana.badakar@intel.com>
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include "intel_freq_control.h"
#include "dpll/intel_freq_ctrl_common_spi.h"
#include "dpll/intel_freq_ctrl_zl30793_spi.h"
#include "dpll/intel_freq_ctrl_zl30733_i2c.h"
#include "dpll/intel_freq_ctrl_common_i2c.h"
#include "dpll/intel_freq_ctrl_zl30733_debugfs.h"

struct platform_device;

void schedule_pll_lock_check(struct intel_freq_control_private *priv)
{
	priv->pll_lock_check_ctr = 0;
	priv->intf_ops->reset_pll_state(priv);
	schedule_delayed_work(&priv->pll_lock_dwork,
			      msecs_to_jiffies(ZL30793_LOCK_CHECK_INTERVAL_IN_MS));
}

static void intel_fpga_frequency_control(struct freq_work *fw)
{
	flush_workqueue(fw->workqueue);
	queue_work(fw->workqueue, &fw->w);
}

static struct ptp_freq_ctrl_info intel_frequency_control_ops = {
	.freqctrl = intel_fpga_frequency_control,
};

static int intel_frequency_control_open(struct intel_freq_control_private *priv)
{
	int ret = 0;

	priv->queued_work.workqueue =
		alloc_ordered_workqueue("freq_contrl_queue", WQ_HIGHPRI);

	if (!priv->queued_work.workqueue) {
		ret = -ENOMEM;
		goto err;
	}

	if (!priv->intf_ops->clock_cleaner) {
		pr_info("Cannot identify [ %s ] clock cleaner\n",
			priv->clockcleaner_info.clock_name);

		destroy_workqueue(priv->queued_work.workqueue);

		ret = -ENODEV;
		goto err;
	}

	if (priv->intf_ops->clock_check(priv) != FREQ_CTRL_ERROR_SUCCESS) {
		ret = -ENODEV;
		goto err;
	}

	priv->freqctrl_ops = intel_frequency_control_ops;

	pr_info("Clock cleaner [ %s ] identified\n",
		priv->clockcleaner_info.clock_name);

	return 0;

err:
	return ret;
}

static void intel_frequency_control_close(struct intel_freq_control_private *priv)
{
	flush_workqueue(priv->queued_work.workqueue);
	destroy_workqueue(priv->queued_work.workqueue);
	cancel_delayed_work_sync(&priv->pll_lock_dwork);
}

static int intel_fpga_fs_probe(struct platform_device *pdev)
{
	int ret = -ENODEV;
	const struct xtile_intf_ops *op_ptr;
	struct intel_freq_control_private *priv = NULL;

	priv = devm_kzalloc(&pdev->dev,
			    sizeof(struct intel_freq_control_private), GFP_KERNEL);

	ret = of_property_read_string(pdev->dev.of_node, "dpll-name",
				      &priv->clockcleaner_info.clock_name);
	if (ret < 0) {
		dev_err(&pdev->dev, "missing dpll-name label\n");
		goto clk_cleaner_err;
	}

	ret = of_property_read_string(pdev->dev.of_node, "interface",
				      &priv->clockcleaner_info.interface);
	if (ret < 0) {
		dev_err(&pdev->dev, "missing interface label\n");
		goto clk_cleaner_err;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "bus-num",
				   &priv->clockcleaner_info.bus_num);
	if (ret < 0) {
		dev_err(&pdev->dev, "missing bus-num label\n");
		goto clk_cleaner_err;
	}

	dev_info(&pdev->dev,
		 "ptp-clockcleaner dpll-name :%s interface: %s\n",
		priv->clockcleaner_info.clock_name,
		priv->clockcleaner_info.interface);

	if (!strcmp(priv->clockcleaner_info.interface, "i2c")) {
		ret = of_property_read_u32(pdev->dev.of_node, "bus-address",
					   &priv->clockcleaner_info.bus_address);
		if (ret < 0) {
			dev_err(&pdev->dev, "missing bus-address value\n");
			goto clk_cleaner_err;
		}

		dev_info(&pdev->dev,
			 "bus-num: %0X bus-address: %0X\n",
			priv->clockcleaner_info.bus_num,
			priv->clockcleaner_info.bus_address);

	} else if (!strcmp(priv->clockcleaner_info.interface, "spi")) {
		ret = of_property_read_u32(pdev->dev.of_node, "chipselect",
					   &priv->clockcleaner_info.chip_select);
		if (ret < 0) {
			dev_err(&pdev->dev, "missing chipselect value\n");
			goto clk_cleaner_err;
		}

		dev_info(&pdev->dev,
			 "chip-select : %0X\n",
			priv->clockcleaner_info.chip_select);
	}

	op_ptr = of_device_get_match_data(&pdev->dev);

	if (!op_ptr) {
		dev_err(&pdev->dev, "No matching data field found\n");
		goto clk_cleaner_err;
	}

	priv->intf_ops = (struct xtile_intf_ops *)op_ptr;

	if (priv->intf_ops->client_validator(&priv->clockcleaner_info) !=
			FREQ_CTRL_ERROR_SUCCESS) {
		dev_err(&pdev->dev, "client validation failed\n");

		ret = -EPROBE_DEFER;
		goto clk_cleaner_err;
	}

	ret = intel_frequency_control_open(priv);

	if (ret == 0) {
		priv->queued_work.scaled_ppm = 0;
		INIT_WORK(&priv->queued_work.w, priv->intf_ops->clock_cleaner);

		dev_set_drvdata(&pdev->dev, priv);
	} else {
		dev_err(&pdev->dev, "Device not detected which is unexpected, quitting");
	}
clk_cleaner_err:
	return ret;
}

static void intel_fpga_fs_remove(struct platform_device *pdev)
{
	struct intel_freq_control_private *priv =
		dev_get_drvdata(&pdev->dev);

	if (priv->intf_ops->shutdown_handler)
		priv->intf_ops->shutdown_handler(priv->pll_dbg);

	intel_frequency_control_close(priv);
}

static const struct xtile_intf_ops zl_spi_data = {
	.client_validator = determine_spi_client,
	.clock_cleaner = intel_freq_control_zl30793,
	.clock_check = spi_dev_check_zl30793_clock,
	.reset_pll_state = reset_dpll_mode,
	.shutdown_handler = zl30733_dbgfs_remove,
};

static const struct xtile_intf_ops zl_i2c_data = {
	.client_validator = determine_i2c_client,
	.clock_cleaner = intel_freq_control_zl30733,
	.clock_check = i2c_dev_check_zl30733_clock,
};

static const struct of_device_id intel_fpga_fs_ids[] = {
	{.compatible = "intel, freq-steering-zl-spi",
	 .data = &zl_spi_data,
	},
	{.compatible = "intel, freq-steering-zl-i2c",
	 .data = &zl_i2c_data,
	},
	{},
};

MODULE_DEVICE_TABLE(of, intel_fpga_fs_ids);

static struct platform_driver intel_fpga_fs_driver = {
	.probe          = intel_fpga_fs_probe,
	.remove         = intel_fpga_fs_remove,
	.suspend        = NULL,
	.resume         = NULL,
	.driver         = {
		.name = "freq-steer",
		.owner  = THIS_MODULE,
		.of_match_table = intel_fpga_fs_ids,
	},
};

module_platform_driver(intel_fpga_fs_driver);

MODULE_DESCRIPTION("Intel FPGA Frequency steering");
MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("GPL");
