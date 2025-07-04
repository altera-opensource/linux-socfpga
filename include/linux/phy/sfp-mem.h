/* SPDX-License-Identifier: GPL-2.0-or-later */

/* Intel(R) Memory based core SFP driver header.
 *
 * Copyright (C) 2025 Intel Corporation. All rights reserved.
 */

#ifndef __LINUX_SFP_MEM_H
#define __LINUX_SFP_MEM_H

#include <linux/bitfield.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/i2c.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/regmap.h>
#include <linux/uaccess.h>

enum sfp_init_status {
	SFP_INIT_RESET = 0,
	SFP_INIT_DONE,
	SFP_A0PAGE_UPDATE_INPROG,
	SFP_A0PAGE_UPDATE_COMPLETE,
	SFP_A2PAGE_UPDATE_INPROG,
	SFP_A2PAGE_UPDATE_COMPLETE,
	SFP_A0_UPDATE_ERROR,
	SFP_A2_UPDATE_ERROR,
};

/**
 * struct sfp - device private data structure
 * @base: base address of the device.
 * @regmap: regmap for device.
 * @dwork: work struct for checking sfp plugin status.
 * @dev: point to device.
 * @init: sfp init status.
 * @lock: lock for sfp initial function and status.
 */
struct sfp {
	void __iomem *base;
	struct regmap *regmap;
	struct delayed_work dwork;
	struct device *dev;
	enum sfp_init_status init;
	struct mutex lock;
	u32 tolerance_count;
};

int sfp_init_work(struct sfp *qsfp);
int sfp_register_regmap(struct sfp *qsfp);
void sfp_remove_device(struct sfp *qsfp);
bool check_sfp_plugin(struct sfp *qsfp);
extern const struct attribute_group *sfp_mem_groups[];

#define IP_IRRESPONSIVE		 0
#define IP_RESPONSE_TOLERANCE_LIMIT  100

#endif //__LINUX_SFP_MEM_H
