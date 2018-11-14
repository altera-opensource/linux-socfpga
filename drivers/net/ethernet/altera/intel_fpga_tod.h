/* SPDX-License-Identifier: GPL-2.0 */
/* Altera PTP Hardware Clock (PHC) Linux driver
 * Copyright (C) 2015-2016 Altera Corporation. All rights reserved.
 * Copyright (C) 2017-2020 Intel Corporation. All rights reserved.
 *
 * Author(s):
 *	Dalon Westergreen <dalon.westergreen@intel.com>
 */

#ifndef __INTEL_FPGA_TOD_H__
#define __INTEL_FPGA_TOD_H__

#include <linux/debugfs.h>
#include <linux/netdevice.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>

/* Altera Time-of-Day (ToD) clock register space. */
struct intel_fpga_tod {
	u32 seconds_msb;
	u32 seconds_lsb;
	u32 nanosec;
	u32 reserved1[0x1];
	u32 period;
	u32 adjust_period;
	u32 adjust_count;
	u32 drift_adjust;
	u32 drift_adjust_rate;
};

#define tod_csroffs(a)	(offsetof(struct intel_fpga_tod, a))

struct intel_fpga_tod_private {
	struct net_device *dev;

	struct ptp_clock_info ptp_clock_ops;
	struct ptp_clock *ptp_clock;

	/* Time-of-Day (ToD) Clock address space */
	struct intel_fpga_tod __iomem *tod_ctrl;
	struct clk *tod_clk;

	/* ToD clock registers protection */
	spinlock_t tod_lock;
};

int intel_fpga_tod_init(struct intel_fpga_tod_private *priv);
void intel_fpga_tod_uinit(struct intel_fpga_tod_private *priv);
int intel_fpga_tod_register(struct intel_fpga_tod_private *priv,
			    struct device *device);
void intel_fpga_tod_unregister(struct intel_fpga_tod_private *priv);
int intel_fpga_tod_probe(struct platform_device *pdev,
			 struct intel_fpga_tod_private *priv);

#endif /* __INTEL_FPGA_TOD_H__ */
