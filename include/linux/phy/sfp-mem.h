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
#include <linux/sfp.h>

#define IP_IRRESPONSIVE		 0
#define IP_RESPONSE_TOLERANCE_LIMIT  100

#define A0_EEPROM_SIZE (ETH_MODULE_SFF_8472_LEN/2)
#define IMPLEMENTED_A2PAGES 4 /* changing this macro enables to dump A2 multiple pages */
#define A2_EEPROM_SIZE (ETH_MODULE_SFF_8472_LEN/2) * IMPLEMENTED_A2PAGES

enum sfp_init_status {
	SFP_DETECT = 0,
	SFP_INIT_RESET,
	SFP_INIT_DONE,
	SFP_A0PAGE_UPDATE_INPROG,
	SFP_A0PAGE_UPDATE_COMPLETE,
	SFP_A2PAGE_UPDATE_INPROG,
	SFP_A2PAGE_UPDATE_COMPLETE,
	SFP_A0_UPDATE_ERROR,
	SFP_A2_UPDATE_ERROR,
};

union sfp_a2_page {
	u8 a2_page[A2_EEPROM_SIZE];
};

union sfp_a0_page {
	u8 a0_page[A0_EEPROM_SIZE];
	struct sfp_eeprom_id a0;
};

/**
 * struct sfp - device private data structure
 * @base: base address of the device.
 * @regmap: regmap for device.
 * @dwork: work struct for checking sfp plugin status.
 * @dev: point to device.
 * @state: sfp status.
 * @lock: lock for sfp initial function and status.
 */
struct sfp {
	void __iomem *base;
	struct regmap *regmap;
	struct delayed_work dwork;
	struct device *dev;
	enum sfp_init_status state;
	struct mutex lock;
	u32 tolerance_count;
	struct sfp_bus *sfp_bus;
	union  sfp_a0_page   a0_page;
	union  sfp_a2_page   a2_page;
};

int sfp_init_work(struct sfp *qsfp);
int sfp_register_regmap(struct sfp *qsfp);
void sfp_remove_device(struct sfp *qsfp);
bool check_sfp_plugin(struct sfp *qsfp);
extern const struct attribute_group *sfp_mem_groups[];

struct sfp_socket_ops {
        void (*attach)(struct sfp *sfp);
        void (*detach)(struct sfp *sfp);
        void (*start)(struct sfp *sfp);
        void (*stop)(struct sfp *sfp);
        void (*set_signal_rate)(struct sfp *sfp, unsigned int rate_kbd);
        int (*module_info)(struct sfp *sfp, struct ethtool_modinfo *modinfo);
        int (*module_eeprom)(struct sfp *sfp, struct ethtool_eeprom *ee,
                             u8 *data);
        int (*module_eeprom_by_page)(struct sfp *sfp,
                                     const struct ethtool_module_eeprom *page,
                                     struct netlink_ext_ack *extack);
};

struct sfp_bus *sfp_register_socket(struct device *dev, struct sfp *sfp,
                                    const struct sfp_socket_ops *ops);

#endif //__LINUX_SFP_MEM_H
