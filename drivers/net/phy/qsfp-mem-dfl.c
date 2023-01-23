// SPDX-License-Identifier: GPL-2.0

/* Intel(R) Memory based QSFP driver For DFL based devices.
 *
 * Copyright (C) 2022 Intel Corporation. All rights reserved.
 */
#include <linux/dfl.h>
#include <linux/module.h>
#include <linux/phy/qsfp-mem.h>

static int qsfp_dfl_probe(struct dfl_device *dfl_dev)
{
	struct device *dev = &dfl_dev->dev;
	struct qsfp *qsfp;
	int ret;

	qsfp = devm_kzalloc(dev, sizeof(*qsfp), GFP_KERNEL);
	if (!qsfp)
		return -ENOMEM;

	qsfp->base = devm_ioremap_resource(dev, &dfl_dev->mmio_res);
	if (!qsfp->base)
		return -ENOMEM;

	qsfp->dev = dev;
	mutex_init(&qsfp->lock);

	dev_set_drvdata(dev, qsfp);

	ret = qsfp_init_work(qsfp);
	if (ret) {
		dev_err_probe(dev, ret,
			      "Failed to initialize delayed work to read QSFP\n");
		goto exit;
	}

	ret = qsfp_register_regmap(qsfp);
	if (ret)
		goto cancel_work;

	return 0;

cancel_work:
	qsfp_remove_device(qsfp);
exit:
	mutex_destroy(&qsfp->lock);
	return ret;
}

static void qsfp_dfl_remove(struct dfl_device *dfl_dev)
{
	struct device *dev = &dfl_dev->dev;
	struct qsfp *qsfp = dev_get_drvdata(dev);

	qsfp_remove_device(qsfp);
	mutex_destroy(&qsfp->lock);
}

#define FME_FEATURE_ID_QSFP 0x13

static const struct dfl_device_id qsfp_ids[] = {
	{ FME_ID, FME_FEATURE_ID_QSFP },
	{ }
};

static struct dfl_driver qsfp_driver = {
	.drv = {
		.name = "qsfp-mem",
		.dev_groups = qsfp_mem_groups,
	},
	.id_table = qsfp_ids,
	.probe = qsfp_dfl_probe,
	.remove = qsfp_dfl_remove,
};

module_dfl_driver(qsfp_driver);
MODULE_DEVICE_TABLE(dfl, qsfp_ids);
MODULE_DESCRIPTION("Intel(R) Memory based QSFP DFL driver");
MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("GPL");
