/* SPDX-License-Identifier: GPL-2.0-or-later */

/* Intel(R) Memory based core QSFP driver header.
 *
 * Copyright (C) 2022 Intel Corporation. All rights reserved.
 */

#ifndef __LINUX_QSFP_MEM_H
#define __LINUX_QSFP_MEM_H

#include <linux/bitfield.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/i2c.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/regmap.h>
#include <linux/uaccess.h>

enum qsfp_init_status {
	QSFP_INIT_RESET = 0,
	QSFP_INIT_DONE,
};

/**
 * struct qsfp - device private data structure
 * @base: base address of the device.
 * @regmap: regmap for device.
 * @dwork: work struct for checking qsfp plugin status.
 * @dev: point to device.
 * @init: qsfp init status.
 * @lock: lock for qsfp initial function and status.
 */
struct qsfp {
	void __iomem *base;
	struct regmap *regmap;
	struct delayed_work dwork;
	struct device *dev;
	enum qsfp_init_status init;
	struct mutex lock;
};

int qsfp_init_work(struct qsfp *qsfp);
int qsfp_register_regmap(struct qsfp *qsfp);
void qsfp_remove_device(struct qsfp *qsfp);
int check_qsfp_plugin(struct qsfp *qsfp);
extern const struct attribute_group *qsfp_mem_groups[];

#endif //__LINUX_QSFP_MEM_H
