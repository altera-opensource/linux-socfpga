/* SPDX-License-Identifier: GPL-2.0 */
/* Altera FPGA Clock Cleaner Frequency Adjustment Driver
 * Copyright (C) 2015-2016 Altera Corporation. All rights reserved.
 * Copyright (C) 2017-2023 Altera Corporation. All rights reserved.
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

 #define FREQ_CTRL_ERROR_SUCCESS 0
 #define FREQ_CTRL_ERROR_FAIL    1

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
	void (*freqctrl)(struct freq_work *fw);
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

struct zarlink_pll_dbg;
struct xtile_intf_ops {
	int (*client_validator)(struct clock_cleaner *cc);
	void (*clock_cleaner)(struct work_struct *ws);
	int (*clock_check)(struct intel_freq_control_private *fq);
	int (*reset_pll_state)(struct intel_freq_control_private *priv);
	void (*shutdown_handler)(struct zarlink_pll_dbg *d);
};

struct intel_freq_control_private {
	u32 step_size;
	struct freq_work queued_work;
	struct intf_type fc_acc_type;
	struct xtile_intf_ops  *intf_ops;
	struct clock_cleaner clockcleaner_info;
	struct ptp_freq_ctrl_info freqctrl_ops;
	struct delayed_work pll_lock_dwork;
	int pll_lock_check_ctr;
	struct zarlink_pll_dbg *pll_dbg;
};

void schedule_pll_lock_check(struct intel_freq_control_private *priv);
 #endif
