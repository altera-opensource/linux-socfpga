#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gcd.h>
#include <linux/module.h>
#include <linux/math64.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/interrupt.h>
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
	struct intel_freq_control_private *freq_priv = priv->ptp_freq_priv;
	u32 tod_period, tod_rem, tod_drift_adjust_fns;
        u32 tod_drift_adjust_rate, gcd_out;
	s64 ppb;
	s64 new_ppb;
	int ret = 0;
	unsigned long flags;
	unsigned long rate;



	/* If there is frequency steering hardware present then use the same */
	if ( (priv->ptp_clockcleaner_enable) && (priv->ptp_freq_priv) &&
			(priv->ptp_freq_priv->freqctrl_ops.freqctrl) ){
		
		if (scaled_ppm) {
			priv->ptp_freq_priv->queued_work.scaled_ppm = scaled_ppm;
			freq_priv->freqctrl_ops.freqctrl(&priv->ptp_freq_priv->queued_work);
		}
		ret = 0;
		goto out;
	}

	rate = clk_get_rate(priv->tod_clk);
	if (!rate) {
		ret = -ENODEV;
		goto out;
	}

	ppb = scaled_ppm_to_ppb(scaled_ppm);
	new_ppb = ppb + NOMINAL_PPB;

	tod_period = div_u64_rem(new_ppb << 16, rate, &tod_rem);
	if (tod_period > TOD_PERIOD_MAX) {
		ret = -ERANGE;
		goto out;
	}

	/* The drift of ToD adjusted periodically by adding a drift_adjust_fns
	 * correction value every drift_adjust_rate count of clock cycles.
	 */
	gcd_out = gcd(tod_rem, rate);

	tod_drift_adjust_fns = tod_rem / gcd_out;
	tod_drift_adjust_rate = rate / gcd_out;

	while ((tod_drift_adjust_fns > TOD_DRIFT_ADJUST_FNS_MAX) ||
	       (tod_drift_adjust_rate > TOD_DRIFT_ADJUST_RATE_MAX)) {
		tod_drift_adjust_fns >>= 1;
		tod_drift_adjust_rate >>= 1;
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

static int intel_fpga_tod_extts_configure(struct ptp_clock_request *rq)
{
	/* Reject requests with unsupported flags */
	if (rq->extts.flags & ~(PTP_ENABLE_FEATURE |
				PTP_RISING_EDGE |
				PTP_FALLING_EDGE |
				PTP_STRICT_FLAGS))
		return -EOPNOTSUPP;

	/* Reject requests to enable time stamping on both edges. */
	if ((rq->extts.flags & PTP_STRICT_FLAGS) &&
	    (rq->extts.flags & PTP_ENABLE_FEATURE) &&
	    (rq->extts.flags & PTP_EXTTS_EDGES) == PTP_EXTTS_EDGES)
		return -EOPNOTSUPP;

	return 0;
}


static int intel_fpga_tod_enable_feature(struct ptp_clock_info *ptp,
					 struct ptp_clock_request *request,
					 int feature_on)
{
	int ret;

	
	switch (request->type) {
	case PTP_CLK_REQ_EXTTS:

		ret = intel_fpga_tod_extts_configure(request);
		break;

	default:
			return -EOPNOTSUPP;
	}

	
	return ret;
}

/* Note the interrupt is level triggered */
static irqreturn_t intel_fpga_pps_isr(int irq, void *data)
{
	struct intel_fpga_tod_private *priv = (typeof(priv))data;
	struct ptp_clock_event event;
	unsigned long flags;
	u32 nanosec, seconds_lsb, seconds_msb;
	u64 seconds;
	spin_lock_irqsave(&priv->tod_lock, flags);

	nanosec = csrrd32(priv->pps_ctrl, pps_csroffs(nanosec));
	seconds_lsb = csrrd32(priv->pps_ctrl, pps_csroffs(seconds_lsb));
	seconds_msb = csrrd32(priv->pps_ctrl, pps_csroffs(seconds_msb));
	spin_unlock_irqrestore(&priv->tod_lock, flags);
	/* Calculate new time */
	seconds = (((u64)(seconds_msb & 0x0000ffff)) << 32) | seconds_lsb;

	/* queue the ptp event to be triggered */	
	event.type = PTP_CLOCK_EXTTS;
	event.index = ptp_clock_index(priv->ptp_clock);
	event.timestamp = seconds * NSEC_PER_SEC + nanosec;
	ptp_clock_event(priv->ptp_clock, &event);

	return IRQ_HANDLED;
}


static struct ptp_clock_info intel_fpga_tod_clock_ops = {
	.owner = THIS_MODULE,
	.name = "intel_fpga_tod",
	.max_adj = 500000000,
	.n_alarm = 0,
	.n_ext_ts = 2,
	.n_per_out = 0,
	.pps = 0,
	.adjfine = intel_fpga_tod_adjust_fine,
	.adjtime = intel_fpga_tod_adjust_time,
	.gettime64 = intel_fpga_tod_get_time,
	.settime64 = intel_fpga_tod_set_time,
	.enable = intel_fpga_tod_enable_feature,
};

/* Register the PTP clock driver to kernel */
static int intel_fpga_tod_register(struct intel_fpga_tod_private *priv,
				   struct device *device)
{
	int ret = 0;
	struct timespec64 ts = { 0, 0 };
	unsigned long flags;
	u32 tod_period, tod_rem, tod_drift_adjust_fns, tod_drift_adjust_rate;
	unsigned long rate;

	priv->ptp_clock_ops = intel_fpga_tod_clock_ops;

	priv->ptp_clock = ptp_clock_register(&priv->ptp_clock_ops, device);

	if (IS_ERR(priv->ptp_clock)) {
		dev_err_probe(device, PTR_ERR(priv->ptp_clock), "cannot obtain ToD period clock\n");
		priv->ptp_clock = NULL;
		ret = -ENODEV;
		goto err;
	}

	if (priv->tod_clk)
		ret = clk_prepare_enable(priv->tod_clk);

	/* Initialize the hardware clock to zero */
	intel_fpga_tod_set_time(&priv->ptp_clock_ops, &ts);
	rate = clk_get_rate(priv->tod_clk);
	if(!rate )
		return 0;

	tod_period = div_u64_rem( NOMINAL_PPB << 16, rate, &tod_rem);
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

err:
	return ret;
}

/* Remove/unregister the ptp clock driver from the kernel */
static int intel_fpga_tod_unregister(struct platform_device *pdev)
{
	struct intel_fpga_tod_private *priv = dev_get_drvdata(&pdev->dev);

	if (priv->ptp_clock) {
		ptp_clock_unregister(priv->ptp_clock);
		priv->ptp_clock = NULL;
	}

	if (priv->tod_clk)
		clk_disable_unprepare(priv->tod_clk);



	return 0;
}

/* Common PTP probe function */
static int intel_fpga_tod_probe(struct platform_device *pdev)
{
	u32 pps_irq;
	bool pps_support = false;
	int ret = -ENODEV;
	struct resource *ptp_res;
	struct resource *pps_res;
	struct device_node *dev_fc;
	struct platform_device *pdev_fc;
	struct intel_fpga_tod_private *priv;
	struct device *dev = &pdev->dev;

	priv = devm_kzalloc(dev, sizeof(struct intel_fpga_tod_private), GFP_KERNEL);
	if (!priv) {
		dev_err_probe(dev,
			      PTR_ERR(priv),
			      "Could not allocate memory for ToD\n");
		ret = -ENOMEM;
		goto err;
	}

	/* Time-of-Day (ToD) Clock address space */
	ret = request_and_map(pdev, "tod_ctrl", &ptp_res,
			      (void __iomem **)&priv->tod_ctrl);
	if (ret)
		goto err;

	dev_info(&pdev->dev, "\tTOD Ctrl at 0x%08lx\n",
		 (unsigned long)ptp_res->start);

	priv->dev = dev;

	ret = request_and_map(pdev, "pps_ctrl", &pps_res,
			      (void __iomem **)&priv->pps_ctrl);

	if (!ret) {
		dev_info(&pdev->dev, "\tPPS Ctrl at 0x%08lx\n",
			 (unsigned long)pps_res->start);
	} else {
		dev_info(&pdev->dev, "\tPPS Ctrl unmapped\n");
		priv->pps_ctrl = NULL;
	}

	/* Time-of-Day (ToD) Clock period clock */
	priv->tod_clk = devm_clk_get(&pdev->dev, "tod_clock");
	if (IS_ERR(priv->tod_clk)) {
		ret = -ENXIO;
		dev_err_probe(&pdev->dev, PTR_ERR(priv->tod_clk),
			      "cannot obtain ToD period clock\n");
		goto err;
	}

	pps_irq = platform_get_irq_byname(pdev, "pps_irq");

	if ((pps_irq > 0) && priv->pps_ctrl) {
		/* pps interrupt is level triggered */
		ret = devm_request_irq(&pdev->dev, pps_irq, intel_fpga_pps_isr,
				       IRQF_TRIGGER_HIGH, "pps_ip", priv);

		if (ret)
			dev_err(&pdev->dev, "could not register pps irq\n");
		else
			pps_support = true;
	}

	if (!pps_support)
		dev_err(&pdev->dev, "pps ctrl not supported");

	priv->ptp_clockcleaner_enable =
	    of_property_read_bool(pdev->dev.of_node,
				  "altr,has-ptp-clockcleaner");

	priv->ptp_freq_priv = NULL;

	/* There is a frequency steering hardware present in the system */
	if (priv->ptp_clockcleaner_enable) {

		/* Get the Freq steering node device from the device tree node */
		dev_fc = of_parse_phandle(pdev->dev.of_node,
					 "clock-cleaner", 0);

		if (dev_fc) {
			pdev_fc = of_find_device_by_node(dev_fc);
			if (!pdev_fc) {
				dev_err(&pdev->dev, "clock cleaner hw details not found\n");
				of_node_put(dev_fc);

				goto no_clock_cleaner;
			}
			else {
				priv->ptp_freq_priv =
					dev_get_drvdata(&pdev_fc->dev);
					if(!priv->ptp_freq_priv)
					{
						dev_err(&pdev->dev, "Frequency steering not available\n");
						ret = -EPROBE_DEFER;
						goto err;
					}
						
			}
		}
	}

no_clock_cleaner:
	ret = intel_fpga_tod_register(priv, dev);
	if (ret)
		goto err;

	spin_lock_init(&priv->tod_lock);
	dev_set_drvdata(dev, priv);
err:
	return ret;
}

static const struct of_device_id intel_fpga_tod_ids[] = {
		{.compatible = "intel, tod",},
		{ }
};

MODULE_DEVICE_TABLE(of, intel_fpga_tod_ids);

static struct platform_driver intel_fpga_tod_driver = {
	.probe		= intel_fpga_tod_probe,
	.remove		= intel_fpga_tod_unregister,
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name = "tod",
		.owner	= THIS_MODULE,
		.of_match_table = intel_fpga_tod_ids,
	},
};

module_platform_driver(intel_fpga_tod_driver);
MODULE_DESCRIPTION("Intel FPGA ToD driver");
MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("GPL");
