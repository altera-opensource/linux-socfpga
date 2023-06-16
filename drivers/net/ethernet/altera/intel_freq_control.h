// SPDX-License-Identifier: GPL
/* Intel FPGA Clock Cleaner Frequency Adjustment Driver
 * Copyright (C) 2015-2016 Altera Corporation. All rights reserved.
 * Copyright (C) 2017-2023 Intel Corporation. All rights reserved.
 *
 * Author(s):
 *	Markos Papadonikolakis <markos.papadonikolakis@intel.com>
 *	Lubana Badakar <lubana.badakar@intel.com>
 */

#ifndef HAVE_INTEL_FREQ_CONTROL_H
#define HAVE_INTEL_FREQ_CONTROL_H

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/math64.h>
#include <linux/spi/spi.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>

struct clock_cleaner {
	const char *clock_name;		/* Eg. si5518 or lmk05028 */
	const char *interface;		/* Eg. spi or i2c*/
	u32 bus_num;			/* spi or i2c bus */
	union {
		u32 bus_address;	/* bus address for i2c*/
		u32 chip_select;	/* chip select for spi*/
	};
};

struct intel_freq_control_private;

struct freq_work;
struct ptp_freq_ctrl_info {
	void (*freqctrl)(struct freq_work *);
};

struct freq_work {
	long scaled_ppm;
	struct work_struct w;
	struct workqueue_struct *workqueue;
};

struct intf_type {
	struct i2c_client *i2c_cli;
	struct spi_device *spi_dev;
};

struct xtile_intf_ops {
	int (*client_validator)(struct clock_cleaner *);
	void (*clock_cleaner)(struct work_struct *);
	int (*clock_check)(struct spi_device *, struct intel_freq_control_private*);
	int (*zl30733_clock_check)(struct intel_freq_control_private*);
};

struct intel_freq_control_private {
	u32 step_size;
	struct freq_work queued_work;
	struct intf_type fc_acc_type;
	struct xtile_intf_ops  *intf_ops;
	struct clock_cleaner clockcleaner_info;
	struct ptp_freq_ctrl_info freqctrl_ops;
};

#define FREQ_CTRL_ERROR_SUCCESS 0
#define FREQ_CTRL_ERROR_FAIL    1

#endif