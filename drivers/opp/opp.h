/*
 * Generic OPP Interface
 *
 * Copyright (C) 2009-2010 Texas Instruments Incorporated.
 *	Nishanth Menon
 *	Romit Dasgupta
 *	Kevin Hilman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __DRIVER_OPP_H__
#define __DRIVER_OPP_H__

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/limits.h>
#include <linux/pm_opp.h>
#include <linux/notifier.h>

struct clk;
struct regulator;

/* Lock to allow exclusive modification to the device and opp lists */
extern struct mutex opp_table_lock;

extern struct list_head opp_tables;

/*
 * Internal data structure organization with the OPP layer library is as
 * follows:
 * opp_tables (root)
 *	|- device 1 (represents voltage domain 1)
 *	|	|- opp 1 (availability, freq, voltage)
 *	|	|- opp 2 ..
 *	...	...
 *	|	`- opp n ..
 *	|- device 2 (represents the next voltage domain)
 *	...
 *	`- device m (represents mth voltage domain)
 * device 1, 2.. are represented by opp_table structure while each opp
 * is represented by the opp structure.
 */

/**
 * struct dev_pm_opp - Generic OPP description structure
 * @node:	opp table node. The nodes are maintained throughout the lifetime
 *		of boot. It is expected only an optimal set of OPPs are
 *		added to the library by the SoC framework.
 *		IMPORTANT: the opp nodes should be maintained in increasing
 *		order.
 * @kref:	for reference count of the OPP.
 * @available:	true/false - marks if this OPP as available or not
 * @dynamic:	not-created from static DT entries.
 * @turbo:	true if turbo (boost) OPP
 * @suspend:	true if suspend OPP
 * @pstate: Device's power domain's performance state.
 * @rate:	Frequency in hertz
 * @supplies:	Power supplies voltage/current values
 * @clock_latency_ns: Latency (in nanoseconds) of switching to this OPP's
 *		frequency from any other OPP's frequency.
 * @opp_table:	points back to the opp_table struct this opp belongs to
 * @np:		OPP's device node.
 * @dentry:	debugfs dentry pointer (per opp)
 *
 * This structure stores the OPP information for a given device.
 */
struct dev_pm_opp {
	struct list_head node;
	struct kref kref;

	bool available;
	bool dynamic;
	bool turbo;
	bool suspend;
	unsigned int pstate;
	unsigned long rate;

	struct dev_pm_opp_supply *supplies;

	unsigned long clock_latency_ns;

	struct opp_table *opp_table;

	struct device_node *np;

#ifdef CONFIG_DEBUG_FS
	struct dentry *dentry;
#endif
};

/**
 * struct opp_device - devices managed by 'struct opp_table'
 * @node:	list node
 * @dev:	device to which the struct object belongs
 * @dentry:	debugfs dentry pointer (per device)
 *
 * This is an internal data structure maintaining the devices that are managed
 * by 'struct opp_table'.
 */
struct opp_device {
	struct list_head node;
	const struct device *dev;

#ifdef CONFIG_DEBUG_FS
	struct dentry *dentry;
#endif
};

enum opp_table_access {
	OPP_TABLE_ACCESS_UNKNOWN = 0,
	OPP_TABLE_ACCESS_EXCLUSIVE = 1,
	OPP_TABLE_ACCESS_SHARED = 2,
};

/**
 * struct opp_table - Device opp structure
 * @node:	table node - contains the devices with OPPs that
 *		have been registered. Nodes once added are not modified in this
 *		table.
 * @head:	notifier head to notify the OPP availability changes.
 * @dev_list:	list of devices that share these OPPs
 * @opp_list:	table of opps
 * @kref:	for reference count of the table.
 * @lock:	mutex protecting the opp_list.
 * @np:		struct device_node pointer for opp's DT node.
 * @clock_latency_ns_max: Max clock latency in nanoseconds.
 * @shared_opp: OPP is shared between multiple devices.
 * @suspend_opp: Pointer to OPP to be used during device suspend.
 * @supported_hw: Array of version number to support.
 * @supported_hw_count: Number of elements in supported_hw array.
 * @prop_name: A name to postfix to many DT properties, while parsing them.
 * @clk: Device's clock handle
 * @regulators: Supply regulators
 * @regulator_count: Number of power supply regulators
 * @genpd_performance_state: Device's power domain support performance state.
 * @set_opp: Platform specific set_opp callback
 * @set_opp_data: Data to be passed to set_opp callback
 * @get_pstate: Platform specific get_pstate callback
 * @dentry:	debugfs dentry pointer of the real device directory (not links).
 * @dentry_name: Name of the real dentry.
 *
 * @voltage_tolerance_v1: In percentage, for v1 bindings only.
 *
 * This is an internal data structure maintaining the link to opps attached to
 * a device. This structure is not meant to be shared to users as it is
 * meant for book keeping and private to OPP library.
 */
struct opp_table {
	struct list_head node;

	struct blocking_notifier_head head;
	struct list_head dev_list;
	struct list_head opp_list;
	struct kref kref;
	struct mutex lock;

	struct device_node *np;
	unsigned long clock_latency_ns_max;

	/* For backward compatibility with v1 bindings */
	unsigned int voltage_tolerance_v1;

	enum opp_table_access shared_opp;
	struct dev_pm_opp *suspend_opp;

	unsigned int *supported_hw;
	unsigned int supported_hw_count;
	const char *prop_name;
	struct clk *clk;
	struct regulator **regulators;
	unsigned int regulator_count;
	bool genpd_performance_state;

	int (*set_opp)(struct dev_pm_set_opp_data *data);
	struct dev_pm_set_opp_data *set_opp_data;
	int (*get_pstate)(struct device *dev, unsigned long rate);

#ifdef CONFIG_DEBUG_FS
	struct dentry *dentry;
	char dentry_name[NAME_MAX];
#endif
};

/* Routines internal to opp core */
void _get_opp_table_kref(struct opp_table *opp_table);
struct opp_table *_find_opp_table(struct device *dev);
struct opp_device *_add_opp_dev(const struct device *dev, struct opp_table *opp_table);
void _dev_pm_opp_remove_table(struct opp_table *opp_table, struct device *dev, bool remove_all);
void _dev_pm_opp_find_and_remove_table(struct device *dev, bool remove_all);
struct dev_pm_opp *_opp_allocate(struct opp_table *opp_table);
void _opp_free(struct dev_pm_opp *opp);
int _opp_add(struct device *dev, struct dev_pm_opp *new_opp, struct opp_table *opp_table);
int _opp_add_v1(struct opp_table *opp_table, struct device *dev, unsigned long freq, long u_volt, bool dynamic);
void _dev_pm_opp_cpumask_remove_table(const struct cpumask *cpumask, bool of);
struct opp_table *_add_opp_table(struct device *dev);

#ifdef CONFIG_OF
void _of_init_opp_table(struct opp_table *opp_table, struct device *dev);
#else
static inline void _of_init_opp_table(struct opp_table *opp_table, struct device *dev) {}
#endif

#ifdef CONFIG_DEBUG_FS
void opp_debug_remove_one(struct dev_pm_opp *opp);
int opp_debug_create_one(struct dev_pm_opp *opp, struct opp_table *opp_table);
int opp_debug_register(struct opp_device *opp_dev, struct opp_table *opp_table);
void opp_debug_unregister(struct opp_device *opp_dev, struct opp_table *opp_table);
#else
static inline void opp_debug_remove_one(struct dev_pm_opp *opp) {}

static inline int opp_debug_create_one(struct dev_pm_opp *opp,
				       struct opp_table *opp_table)
{ return 0; }
static inline int opp_debug_register(struct opp_device *opp_dev,
				     struct opp_table *opp_table)
{ return 0; }

static inline void opp_debug_unregister(struct opp_device *opp_dev,
					struct opp_table *opp_table)
{ }
#endif		/* DEBUG_FS */

#endif		/* __DRIVER_OPP_H__ */
