// SPDX-License-Identifier: GPL-2.0
/* Intel FPGA ToD PTP Hardware Clock (PHC) Linux driver
 * Copyright (C) 2015-2016 Altera Corporation. All rights reserved.
 * Copyright (C) 2017-2020 Intel Corporation. All rights reserved.
 *
 * Author(s):
 *	Dalon Westergreen <dalon.westergreen@intel.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gcd.h>
#include <linux/module.h>
#include <linux/math64.h>
#include <linux/net_tstamp.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include "altera_utils.h"
#include "intel_fpga_tod.h"

#define NOMINAL_PPB			1000000000ULL
#define TOD_PERIOD_MAX			0xfffff
#define TOD_PERIOD_MIN			0
#define TOD_DRIFT_ADJUST_FNS_MAX	0xffff
#define TOD_DRIFT_ADJUST_RATE_MAX	0xffff
#define TOD_ADJUST_COUNT_MAX		0xfffff
#define TOD_ADJUST_MS_MAX		(((((TOD_PERIOD_MAX) >> 16) + 1) * \
					  ((TOD_ADJUST_COUNT_MAX) + 1)) /  \
					 1000000UL)

/* A fine ToD HW clock offset adjustment.
 * To perform the fine offset adjustment the AdjustPeriod register is used
 * to replace the Period register for AdjustCount clock cycles in hardware.
 */
static int fine_adjust_tod_clock(struct intel_fpga_tod_private *priv,
				 u32 adjust_period, u32 adjust_count)
{
	int limit;

	csrwr32(adjust_period, priv->tod_ctrl, tod_csroffs(adjust_period));
	csrwr32(adjust_count, priv->tod_ctrl, tod_csroffs(adjust_count));

	/* Wait for present offset adjustment update to complete */
	limit = TOD_ADJUST_MS_MAX;
	while (limit--) {
		if (!csrrd32(priv->tod_ctrl, tod_csroffs(adjust_count)))
			break;
		mdelay(1);
	}
	if (limit < 0)
		return -EBUSY;

	return 0;
}

/* A coarse ToD HW clock offset adjustment.
 * The coarse time adjustment performs by adding or subtracting the delta value
 * from the current ToD HW clock time.
 */
static int coarse_adjust_tod_clock(struct intel_fpga_tod_private *priv,
				   s64 delta)
{
	u32 seconds_msb, seconds_lsb, nanosec;
	u64 seconds, now;

	if (delta == 0)
		goto out;

	/* Get current time */
	nanosec = csrrd32(priv->tod_ctrl, tod_csroffs(nanosec));
	seconds_lsb = csrrd32(priv->tod_ctrl, tod_csroffs(seconds_lsb));
	seconds_msb = csrrd32(priv->tod_ctrl, tod_csroffs(seconds_msb));

	/* Calculate new time */
	seconds = (((u64)(seconds_msb & 0x0000ffff)) << 32) | seconds_lsb;
	now = seconds * NSEC_PER_SEC + nanosec + delta;

	seconds = div_u64_rem(now, NSEC_PER_SEC, &nanosec);
	seconds_msb = upper_32_bits(seconds) & 0x0000ffff;
	seconds_lsb = lower_32_bits(seconds);

	/* Set corrected time */
	csrwr32(seconds_msb, priv->tod_ctrl, tod_csroffs(seconds_msb));
	csrwr32(seconds_lsb, priv->tod_ctrl, tod_csroffs(seconds_lsb));
	csrwr32(nanosec, priv->tod_ctrl, tod_csroffs(nanosec));

out:
	return 0;
}

static int intel_fpga_tod_adjust_fine(struct ptp_clock_info *ptp,
				      long scaled_ppm)
{
	struct intel_fpga_tod_private *priv =
		container_of(ptp, struct intel_fpga_tod_private, ptp_clock_ops);
	u32 tod_period, tod_rem, tod_drift_adjust_fns, tod_drift_adjust_rate;
	unsigned long flags;
	unsigned long rate;
	int ret = 0;
	s64 ppb;
	u64 new_ppb;

	rate = clk_get_rate(priv->tod_clk);

	/* From scaled_ppm_to_ppb */
	ppb = 1 + scaled_ppm;
	ppb *= 125;
	ppb >>= 13;

	new_ppb = (s32)ppb + NOMINAL_PPB;

	tod_period = div_u64_rem(new_ppb << 16, rate, &tod_rem);
	if (tod_period > TOD_PERIOD_MAX) {
		ret = -ERANGE;
		goto out;
	}

	/* The drift of ToD adjusted periodically by adding a drift_adjust_fns
	 * correction value every drift_adjust_rate count of clock cycles.
	 */
	tod_drift_adjust_fns = tod_rem / gcd(tod_rem, rate);
	tod_drift_adjust_rate = rate / gcd(tod_rem, rate);

	while ((tod_drift_adjust_fns > TOD_DRIFT_ADJUST_FNS_MAX) |
		(tod_drift_adjust_rate > TOD_DRIFT_ADJUST_RATE_MAX)) {
		tod_drift_adjust_fns = tod_drift_adjust_fns >> 1;
		tod_drift_adjust_rate = tod_drift_adjust_rate >> 1;
	}

	if (tod_drift_adjust_fns == 0)
		tod_drift_adjust_rate = 0;

	spin_lock_irqsave(&priv->tod_lock, flags);
	csrwr32(tod_period, priv->tod_ctrl, tod_csroffs(period));
	csrwr32(0, priv->tod_ctrl, tod_csroffs(adjust_period));
	csrwr32(0, priv->tod_ctrl, tod_csroffs(adjust_count));
	csrwr32(tod_drift_adjust_fns, priv->tod_ctrl,
		tod_csroffs(drift_adjust));
	csrwr32(tod_drift_adjust_rate, priv->tod_ctrl,
		tod_csroffs(drift_adjust_rate));
	spin_unlock_irqrestore(&priv->tod_lock, flags);

out:
	return ret;
}

static int intel_fpga_tod_adjust_time(struct ptp_clock_info *ptp, s64 delta)
{
	struct intel_fpga_tod_private *priv =
		container_of(ptp, struct intel_fpga_tod_private, ptp_clock_ops);
	unsigned long flags;
	u32 period, diff, rem, rem_period, adj_period;
	u64 count;
	int neg_adj = 0, ret = 0;

	if (delta < 0) {
		neg_adj = 1;
		delta = -delta;
	}

	spin_lock_irqsave(&priv->tod_lock, flags);

	/* Get the maximum possible value of the Period register offset
	 * adjustment in nanoseconds scale. This depends on the current
	 * Period register setting and the maximum and minimum possible
	 * values of the Period register.
	 */
	period = csrrd32(priv->tod_ctrl, tod_csroffs(period));

	if (neg_adj)
		diff = (period - TOD_PERIOD_MIN) >> 16;
	else
		diff = (TOD_PERIOD_MAX - period) >> 16;

	/* Find the number of cycles required for the
	 * time adjustment
	 */
	count = div_u64_rem(delta, diff, &rem);

	if (neg_adj) {
		adj_period = period - (diff << 16);
		rem_period = period - (rem << 16);
	} else {
		adj_period = period + (diff << 16);
		rem_period = period + (rem << 16);
	}

	/* If count is larger than the maximum count,
	 * just set the time.
	 */
	if (count > TOD_ADJUST_COUNT_MAX) {
		if (neg_adj)
			delta = -delta;

		/* Perform the coarse time offset adjustment */
		ret = coarse_adjust_tod_clock(priv, delta);
	} else {
		/* Adjust the period for count cycles to adjust
		 * the time.
		 */
		if (count)
			ret = fine_adjust_tod_clock(priv, adj_period, count);

		/* If there is a remainder, adjust the period for an
		 * additional cycle
		 */
		if (rem)
			ret = fine_adjust_tod_clock(priv, rem_period, 1);
	}

	spin_unlock_irqrestore(&priv->tod_lock, flags);

	return ret;
}

static int intel_fpga_tod_get_time(struct ptp_clock_info *ptp,
				   struct timespec64 *ts)
{
	struct intel_fpga_tod_private *priv =
		container_of(ptp, struct intel_fpga_tod_private, ptp_clock_ops);
	u32 seconds_msb, seconds_lsb, nanosec;
	unsigned long flags;
	u64 seconds;

	spin_lock_irqsave(&priv->tod_lock, flags);
	nanosec = csrrd32(priv->tod_ctrl, tod_csroffs(nanosec));
	seconds_lsb = csrrd32(priv->tod_ctrl, tod_csroffs(seconds_lsb));
	seconds_msb = csrrd32(priv->tod_ctrl, tod_csroffs(seconds_msb));
	spin_unlock_irqrestore(&priv->tod_lock, flags);

	seconds = (((u64)(seconds_msb & 0x0000ffff)) << 32) | seconds_lsb;

	ts->tv_nsec = nanosec;
	ts->tv_sec = (__kernel_old_time_t)seconds;

	return 0;
}

static int intel_fpga_tod_set_time(struct ptp_clock_info *ptp,
				   const struct timespec64 *ts)
{
	struct intel_fpga_tod_private *priv =
		container_of(ptp, struct intel_fpga_tod_private, ptp_clock_ops);
	u32 seconds_msb = upper_32_bits(ts->tv_sec) & 0x0000ffff;
	u32 seconds_lsb = lower_32_bits(ts->tv_sec);
	u32 nanosec = lower_32_bits(ts->tv_nsec);
	unsigned long flags;

	spin_lock_irqsave(&priv->tod_lock, flags);
	csrwr32(seconds_msb, priv->tod_ctrl, tod_csroffs(seconds_msb));
	csrwr32(seconds_lsb, priv->tod_ctrl, tod_csroffs(seconds_lsb));
	csrwr32(nanosec, priv->tod_ctrl, tod_csroffs(nanosec));
	spin_unlock_irqrestore(&priv->tod_lock, flags);

	return 0;
}

static int intel_fpga_tod_enable_feature(struct ptp_clock_info *ptp,
					 struct ptp_clock_request *request,
					 int on)
{
	return -EOPNOTSUPP;
}

static struct ptp_clock_info intel_fpga_tod_clock_ops = {
	.owner = THIS_MODULE,
	.name = "intel_fpga_tod",
	.max_adj = 500000000,
	.n_alarm = 0,
	.n_ext_ts = 0,
	.n_per_out = 0,
	.pps = 0,
	.adjfine = intel_fpga_tod_adjust_fine,
	.adjtime = intel_fpga_tod_adjust_time,
	.gettime64 = intel_fpga_tod_get_time,
	.settime64 = intel_fpga_tod_set_time,
	.enable = intel_fpga_tod_enable_feature,
};

/* Register the PTP clock driver to kernel */
int intel_fpga_tod_register(struct intel_fpga_tod_private *priv,
			    struct device *device)
{
	int ret = 0;
	struct timespec64 ts = { 0, 0 };

	priv->ptp_clock_ops = intel_fpga_tod_clock_ops;

	priv->ptp_clock = ptp_clock_register(&priv->ptp_clock_ops, device);
	if (IS_ERR(priv->ptp_clock)) {
		priv->ptp_clock = NULL;
		ret = -ENODEV;
	}

	if (priv->tod_clk)
		ret = clk_prepare_enable(priv->tod_clk);

	/* Initialize the hardware clock to zero */
	intel_fpga_tod_set_time(&priv->ptp_clock_ops, &ts);

	return ret;
}

/* Remove/unregister the ptp clock driver from the kernel */
void intel_fpga_tod_unregister(struct intel_fpga_tod_private *priv)
{
	if (priv->ptp_clock) {
		ptp_clock_unregister(priv->ptp_clock);
		priv->ptp_clock = NULL;
	}

	if (priv->tod_clk)
		clk_disable_unprepare(priv->tod_clk);
}

/* Common PTP probe function */
int intel_fpga_tod_probe(struct platform_device *pdev,
			 struct intel_fpga_tod_private *priv)
{
	struct resource *ptp_res;
	int ret = -ENODEV;

	priv->dev = (struct net_device *)platform_get_drvdata(pdev);

	/* Time-of-Day (ToD) Clock address space */
	ret = request_and_map(pdev, "tod_ctrl", &ptp_res,
			      (void __iomem **)&priv->tod_ctrl);
	if (ret)
		goto err;

	dev_info(&pdev->dev, "\tTOD Ctrl at 0x%08lx\n",
		 (unsigned long)ptp_res->start);

	/* Time-of-Day (ToD) Clock period clock */
	priv->tod_clk = devm_clk_get(&pdev->dev, "tod_clk");
	if (IS_ERR(priv->tod_clk)) {
		dev_err(&pdev->dev, "cannot obtain ToD period clock\n");
		ret = -ENXIO;
		goto err;
	}

	spin_lock_init(&priv->tod_lock);
err:
	return ret;
}

MODULE_LICENSE("GPL");
