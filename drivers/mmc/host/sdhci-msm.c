/*
 * drivers/mmc/host/sdhci-msm.c - Qualcomm SDHCI Platform driver
 *
 * Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/mmc/mmc.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/iopoll.h>

#include "sdhci-pltfm.h"

#define CORE_MCI_VERSION		0x50
#define CORE_VERSION_MAJOR_SHIFT	28
#define CORE_VERSION_MAJOR_MASK		(0xf << CORE_VERSION_MAJOR_SHIFT)
#define CORE_VERSION_MINOR_MASK		0xff

#define CORE_MCI_GENERICS		0x70
#define SWITCHABLE_SIGNALING_VOLTAGE	BIT(29)

#define CORE_HC_MODE		0x78
#define HC_MODE_EN		0x1
#define CORE_POWER		0x0
#define CORE_SW_RST		BIT(7)
#define FF_CLK_SW_RST_DIS	BIT(13)

#define CORE_PWRCTL_STATUS	0xdc
#define CORE_PWRCTL_MASK	0xe0
#define CORE_PWRCTL_CLEAR	0xe4
#define CORE_PWRCTL_CTL		0xe8
#define CORE_PWRCTL_BUS_OFF	BIT(0)
#define CORE_PWRCTL_BUS_ON	BIT(1)
#define CORE_PWRCTL_IO_LOW	BIT(2)
#define CORE_PWRCTL_IO_HIGH	BIT(3)
#define CORE_PWRCTL_BUS_SUCCESS BIT(0)
#define CORE_PWRCTL_IO_SUCCESS	BIT(2)
#define REQ_BUS_OFF		BIT(0)
#define REQ_BUS_ON		BIT(1)
#define REQ_IO_LOW		BIT(2)
#define REQ_IO_HIGH		BIT(3)
#define INT_MASK		0xf
#define MAX_PHASES		16
#define CORE_DLL_LOCK		BIT(7)
#define CORE_DDR_DLL_LOCK	BIT(11)
#define CORE_DLL_EN		BIT(16)
#define CORE_CDR_EN		BIT(17)
#define CORE_CK_OUT_EN		BIT(18)
#define CORE_CDR_EXT_EN		BIT(19)
#define CORE_DLL_PDN		BIT(29)
#define CORE_DLL_RST		BIT(30)
#define CORE_DLL_CONFIG		0x100
#define CORE_CMD_DAT_TRACK_SEL	BIT(0)
#define CORE_DLL_STATUS		0x108

#define CORE_DLL_CONFIG_2	0x1b4
#define CORE_DDR_CAL_EN		BIT(0)
#define CORE_FLL_CYCLE_CNT	BIT(18)
#define CORE_DLL_CLOCK_DISABLE	BIT(21)

#define CORE_VENDOR_SPEC	0x10c
#define CORE_VENDOR_SPEC_POR_VAL	0xa1c
#define CORE_CLK_PWRSAVE	BIT(1)
#define CORE_HC_MCLK_SEL_DFLT	(2 << 8)
#define CORE_HC_MCLK_SEL_HS400	(3 << 8)
#define CORE_HC_MCLK_SEL_MASK	(3 << 8)
#define CORE_HC_SELECT_IN_EN	BIT(18)
#define CORE_HC_SELECT_IN_HS400	(6 << 19)
#define CORE_HC_SELECT_IN_MASK	(7 << 19)

#define CORE_CSR_CDC_CTLR_CFG0		0x130
#define CORE_SW_TRIG_FULL_CALIB		BIT(16)
#define CORE_HW_AUTOCAL_ENA		BIT(17)

#define CORE_CSR_CDC_CTLR_CFG1		0x134
#define CORE_CSR_CDC_CAL_TIMER_CFG0	0x138
#define CORE_TIMER_ENA			BIT(16)

#define CORE_CSR_CDC_CAL_TIMER_CFG1	0x13C
#define CORE_CSR_CDC_REFCOUNT_CFG	0x140
#define CORE_CSR_CDC_COARSE_CAL_CFG	0x144
#define CORE_CDC_OFFSET_CFG		0x14C
#define CORE_CSR_CDC_DELAY_CFG		0x150
#define CORE_CDC_SLAVE_DDA_CFG		0x160
#define CORE_CSR_CDC_STATUS0		0x164
#define CORE_CALIBRATION_DONE		BIT(0)

#define CORE_CDC_ERROR_CODE_MASK	0x7000000

#define CORE_CSR_CDC_GEN_CFG		0x178
#define CORE_CDC_SWITCH_BYPASS_OFF	BIT(0)
#define CORE_CDC_SWITCH_RC_EN		BIT(1)

#define CORE_DDR_200_CFG		0x184
#define CORE_CDC_T4_DLY_SEL		BIT(0)
#define CORE_CMDIN_RCLK_EN		BIT(1)
#define CORE_START_CDC_TRAFFIC		BIT(6)
#define CORE_VENDOR_SPEC3	0x1b0
#define CORE_PWRSAVE_DLL	BIT(3)

#define CORE_DDR_CONFIG		0x1b8
#define DDR_CONFIG_POR_VAL	0x80040853

#define CORE_VENDOR_SPEC_CAPABILITIES0	0x11c

#define INVALID_TUNING_PHASE	-1
#define SDHCI_MSM_MIN_CLOCK	400000
#define CORE_FREQ_100MHZ	(100 * 1000 * 1000)

#define CDR_SELEXT_SHIFT	20
#define CDR_SELEXT_MASK		(0xf << CDR_SELEXT_SHIFT)
#define CMUX_SHIFT_PHASE_SHIFT	24
#define CMUX_SHIFT_PHASE_MASK	(7 << CMUX_SHIFT_PHASE_SHIFT)

#define MSM_MMC_AUTOSUSPEND_DELAY_MS	50

/* Timeout value to avoid infinite waiting for pwr_irq */
#define MSM_PWR_IRQ_TIMEOUT_MS 5000

struct sdhci_msm_host {
	struct platform_device *pdev;
	void __iomem *core_mem;	/* MSM SDCC mapped address */
	int pwr_irq;		/* power irq */
	struct clk *bus_clk;	/* SDHC bus voter clock */
	struct clk *xo_clk;	/* TCXO clk needed for FLL feature of cm_dll*/
	struct clk_bulk_data bulk_clks[4]; /* core, iface, cal, sleep clocks */
	unsigned long clk_rate;
	struct mmc_host *mmc;
	bool use_14lpp_dll_reset;
	bool tuning_done;
	bool calibration_done;
	u8 saved_tuning_phase;
	bool use_cdclp533;
	u32 curr_pwr_state;
	u32 curr_io_level;
	wait_queue_head_t pwr_irq_wait;
	bool pwr_irq_flag;
};

static unsigned int msm_get_clock_rate_for_bus_mode(struct sdhci_host *host,
						    unsigned int clock)
{
	struct mmc_ios ios = host->mmc->ios;
	/*
	 * The SDHC requires internal clock frequency to be double the
	 * actual clock that will be set for DDR mode. The controller
	 * uses the faster clock(100/400MHz) for some of its parts and
	 * send the actual required clock (50/200MHz) to the card.
	 */
	if (ios.timing == MMC_TIMING_UHS_DDR50 ||
	    ios.timing == MMC_TIMING_MMC_DDR52 ||
	    ios.timing == MMC_TIMING_MMC_HS400 ||
	    host->flags & SDHCI_HS400_TUNING)
		clock *= 2;
	return clock;
}

static void msm_set_clock_rate_for_bus_mode(struct sdhci_host *host,
					    unsigned int clock)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	struct mmc_ios curr_ios = host->mmc->ios;
	struct clk *core_clk = msm_host->bulk_clks[0].clk;
	int rc;

	clock = msm_get_clock_rate_for_bus_mode(host, clock);
	rc = clk_set_rate(core_clk, clock);
	if (rc) {
		pr_err("%s: Failed to set clock at rate %u at timing %d\n",
		       mmc_hostname(host->mmc), clock,
		       curr_ios.timing);
		return;
	}
	msm_host->clk_rate = clock;
	pr_debug("%s: Setting clock at rate %lu at timing %d\n",
		 mmc_hostname(host->mmc), clk_get_rate(core_clk),
		 curr_ios.timing);
}

/* Platform specific tuning */
static inline int msm_dll_poll_ck_out_en(struct sdhci_host *host, u8 poll)
{
	u32 wait_cnt = 50;
	u8 ck_out_en;
	struct mmc_host *mmc = host->mmc;

	/* Poll for CK_OUT_EN bit.  max. poll time = 50us */
	ck_out_en = !!(readl_relaxed(host->ioaddr + CORE_DLL_CONFIG) &
			CORE_CK_OUT_EN);

	while (ck_out_en != poll) {
		if (--wait_cnt == 0) {
			dev_err(mmc_dev(mmc), "%s: CK_OUT_EN bit is not %d\n",
			       mmc_hostname(mmc), poll);
			return -ETIMEDOUT;
		}
		udelay(1);

		ck_out_en = !!(readl_relaxed(host->ioaddr + CORE_DLL_CONFIG) &
				CORE_CK_OUT_EN);
	}

	return 0;
}

static int msm_config_cm_dll_phase(struct sdhci_host *host, u8 phase)
{
	int rc;
	static const u8 grey_coded_phase_table[] = {
		0x0, 0x1, 0x3, 0x2, 0x6, 0x7, 0x5, 0x4,
		0xc, 0xd, 0xf, 0xe, 0xa, 0xb, 0x9, 0x8
	};
	unsigned long flags;
	u32 config;
	struct mmc_host *mmc = host->mmc;

	if (phase > 0xf)
		return -EINVAL;

	spin_lock_irqsave(&host->lock, flags);

	config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG);
	config &= ~(CORE_CDR_EN | CORE_CK_OUT_EN);
	config |= (CORE_CDR_EXT_EN | CORE_DLL_EN);
	writel_relaxed(config, host->ioaddr + CORE_DLL_CONFIG);

	/* Wait until CK_OUT_EN bit of DLL_CONFIG register becomes '0' */
	rc = msm_dll_poll_ck_out_en(host, 0);
	if (rc)
		goto err_out;

	/*
	 * Write the selected DLL clock output phase (0 ... 15)
	 * to CDR_SELEXT bit field of DLL_CONFIG register.
	 */
	config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG);
	config &= ~CDR_SELEXT_MASK;
	config |= grey_coded_phase_table[phase] << CDR_SELEXT_SHIFT;
	writel_relaxed(config, host->ioaddr + CORE_DLL_CONFIG);

	config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG);
	config |= CORE_CK_OUT_EN;
	writel_relaxed(config, host->ioaddr + CORE_DLL_CONFIG);

	/* Wait until CK_OUT_EN bit of DLL_CONFIG register becomes '1' */
	rc = msm_dll_poll_ck_out_en(host, 1);
	if (rc)
		goto err_out;

	config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG);
	config |= CORE_CDR_EN;
	config &= ~CORE_CDR_EXT_EN;
	writel_relaxed(config, host->ioaddr + CORE_DLL_CONFIG);
	goto out;

err_out:
	dev_err(mmc_dev(mmc), "%s: Failed to set DLL phase: %d\n",
	       mmc_hostname(mmc), phase);
out:
	spin_unlock_irqrestore(&host->lock, flags);
	return rc;
}

/*
 * Find out the greatest range of consecuitive selected
 * DLL clock output phases that can be used as sampling
 * setting for SD3.0 UHS-I card read operation (in SDR104
 * timing mode) or for eMMC4.5 card read operation (in
 * HS400/HS200 timing mode).
 * Select the 3/4 of the range and configure the DLL with the
 * selected DLL clock output phase.
 */

static int msm_find_most_appropriate_phase(struct sdhci_host *host,
					   u8 *phase_table, u8 total_phases)
{
	int ret;
	u8 ranges[MAX_PHASES][MAX_PHASES] = { {0}, {0} };
	u8 phases_per_row[MAX_PHASES] = { 0 };
	int row_index = 0, col_index = 0, selected_row_index = 0, curr_max = 0;
	int i, cnt, phase_0_raw_index = 0, phase_15_raw_index = 0;
	bool phase_0_found = false, phase_15_found = false;
	struct mmc_host *mmc = host->mmc;

	if (!total_phases || (total_phases > MAX_PHASES)) {
		dev_err(mmc_dev(mmc), "%s: Invalid argument: total_phases=%d\n",
		       mmc_hostname(mmc), total_phases);
		return -EINVAL;
	}

	for (cnt = 0; cnt < total_phases; cnt++) {
		ranges[row_index][col_index] = phase_table[cnt];
		phases_per_row[row_index] += 1;
		col_index++;

		if ((cnt + 1) == total_phases) {
			continue;
		/* check if next phase in phase_table is consecutive or not */
		} else if ((phase_table[cnt] + 1) != phase_table[cnt + 1]) {
			row_index++;
			col_index = 0;
		}
	}

	if (row_index >= MAX_PHASES)
		return -EINVAL;

	/* Check if phase-0 is present in first valid window? */
	if (!ranges[0][0]) {
		phase_0_found = true;
		phase_0_raw_index = 0;
		/* Check if cycle exist between 2 valid windows */
		for (cnt = 1; cnt <= row_index; cnt++) {
			if (phases_per_row[cnt]) {
				for (i = 0; i < phases_per_row[cnt]; i++) {
					if (ranges[cnt][i] == 15) {
						phase_15_found = true;
						phase_15_raw_index = cnt;
						break;
					}
				}
			}
		}
	}

	/* If 2 valid windows form cycle then merge them as single window */
	if (phase_0_found && phase_15_found) {
		/* number of phases in raw where phase 0 is present */
		u8 phases_0 = phases_per_row[phase_0_raw_index];
		/* number of phases in raw where phase 15 is present */
		u8 phases_15 = phases_per_row[phase_15_raw_index];

		if (phases_0 + phases_15 >= MAX_PHASES)
			/*
			 * If there are more than 1 phase windows then total
			 * number of phases in both the windows should not be
			 * more than or equal to MAX_PHASES.
			 */
			return -EINVAL;

		/* Merge 2 cyclic windows */
		i = phases_15;
		for (cnt = 0; cnt < phases_0; cnt++) {
			ranges[phase_15_raw_index][i] =
			    ranges[phase_0_raw_index][cnt];
			if (++i >= MAX_PHASES)
				break;
		}

		phases_per_row[phase_0_raw_index] = 0;
		phases_per_row[phase_15_raw_index] = phases_15 + phases_0;
	}

	for (cnt = 0; cnt <= row_index; cnt++) {
		if (phases_per_row[cnt] > curr_max) {
			curr_max = phases_per_row[cnt];
			selected_row_index = cnt;
		}
	}

	i = (curr_max * 3) / 4;
	if (i)
		i--;

	ret = ranges[selected_row_index][i];

	if (ret >= MAX_PHASES) {
		ret = -EINVAL;
		dev_err(mmc_dev(mmc), "%s: Invalid phase selected=%d\n",
		       mmc_hostname(mmc), ret);
	}

	return ret;
}

static inline void msm_cm_dll_set_freq(struct sdhci_host *host)
{
	u32 mclk_freq = 0, config;

	/* Program the MCLK value to MCLK_FREQ bit field */
	if (host->clock <= 112000000)
		mclk_freq = 0;
	else if (host->clock <= 125000000)
		mclk_freq = 1;
	else if (host->clock <= 137000000)
		mclk_freq = 2;
	else if (host->clock <= 150000000)
		mclk_freq = 3;
	else if (host->clock <= 162000000)
		mclk_freq = 4;
	else if (host->clock <= 175000000)
		mclk_freq = 5;
	else if (host->clock <= 187000000)
		mclk_freq = 6;
	else if (host->clock <= 200000000)
		mclk_freq = 7;

	config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG);
	config &= ~CMUX_SHIFT_PHASE_MASK;
	config |= mclk_freq << CMUX_SHIFT_PHASE_SHIFT;
	writel_relaxed(config, host->ioaddr + CORE_DLL_CONFIG);
}

/* Initialize the DLL (Programmable Delay Line) */
static int msm_init_cm_dll(struct sdhci_host *host)
{
	struct mmc_host *mmc = host->mmc;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	int wait_cnt = 50;
	unsigned long flags;
	u32 config;

	spin_lock_irqsave(&host->lock, flags);

	/*
	 * Make sure that clock is always enabled when DLL
	 * tuning is in progress. Keeping PWRSAVE ON may
	 * turn off the clock.
	 */
	config = readl_relaxed(host->ioaddr + CORE_VENDOR_SPEC);
	config &= ~CORE_CLK_PWRSAVE;
	writel_relaxed(config, host->ioaddr + CORE_VENDOR_SPEC);

	if (msm_host->use_14lpp_dll_reset) {
		config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG);
		config &= ~CORE_CK_OUT_EN;
		writel_relaxed(config, host->ioaddr + CORE_DLL_CONFIG);

		config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG_2);
		config |= CORE_DLL_CLOCK_DISABLE;
		writel_relaxed(config, host->ioaddr + CORE_DLL_CONFIG_2);
	}

	config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG);
	config |= CORE_DLL_RST;
	writel_relaxed(config, host->ioaddr + CORE_DLL_CONFIG);

	config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG);
	config |= CORE_DLL_PDN;
	writel_relaxed(config, host->ioaddr + CORE_DLL_CONFIG);
	msm_cm_dll_set_freq(host);

	if (msm_host->use_14lpp_dll_reset &&
	    !IS_ERR_OR_NULL(msm_host->xo_clk)) {
		u32 mclk_freq = 0;

		config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG_2);
		config &= CORE_FLL_CYCLE_CNT;
		if (config)
			mclk_freq = DIV_ROUND_CLOSEST_ULL((host->clock * 8),
					clk_get_rate(msm_host->xo_clk));
		else
			mclk_freq = DIV_ROUND_CLOSEST_ULL((host->clock * 4),
					clk_get_rate(msm_host->xo_clk));

		config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG_2);
		config &= ~(0xFF << 10);
		config |= mclk_freq << 10;

		writel_relaxed(config, host->ioaddr + CORE_DLL_CONFIG_2);
		/* wait for 5us before enabling DLL clock */
		udelay(5);
	}

	config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG);
	config &= ~CORE_DLL_RST;
	writel_relaxed(config, host->ioaddr + CORE_DLL_CONFIG);

	config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG);
	config &= ~CORE_DLL_PDN;
	writel_relaxed(config, host->ioaddr + CORE_DLL_CONFIG);

	if (msm_host->use_14lpp_dll_reset) {
		msm_cm_dll_set_freq(host);
		config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG_2);
		config &= ~CORE_DLL_CLOCK_DISABLE;
		writel_relaxed(config, host->ioaddr + CORE_DLL_CONFIG_2);
	}

	config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG);
	config |= CORE_DLL_EN;
	writel_relaxed(config, host->ioaddr + CORE_DLL_CONFIG);

	config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG);
	config |= CORE_CK_OUT_EN;
	writel_relaxed(config, host->ioaddr + CORE_DLL_CONFIG);

	/* Wait until DLL_LOCK bit of DLL_STATUS register becomes '1' */
	while (!(readl_relaxed(host->ioaddr + CORE_DLL_STATUS) &
		 CORE_DLL_LOCK)) {
		/* max. wait for 50us sec for LOCK bit to be set */
		if (--wait_cnt == 0) {
			dev_err(mmc_dev(mmc), "%s: DLL failed to LOCK\n",
			       mmc_hostname(mmc));
			spin_unlock_irqrestore(&host->lock, flags);
			return -ETIMEDOUT;
		}
		udelay(1);
	}

	spin_unlock_irqrestore(&host->lock, flags);
	return 0;
}

static void msm_hc_select_default(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	u32 config;

	if (!msm_host->use_cdclp533) {
		config = readl_relaxed(host->ioaddr +
				CORE_VENDOR_SPEC3);
		config &= ~CORE_PWRSAVE_DLL;
		writel_relaxed(config, host->ioaddr +
				CORE_VENDOR_SPEC3);
	}

	config = readl_relaxed(host->ioaddr + CORE_VENDOR_SPEC);
	config &= ~CORE_HC_MCLK_SEL_MASK;
	config |= CORE_HC_MCLK_SEL_DFLT;
	writel_relaxed(config, host->ioaddr + CORE_VENDOR_SPEC);

	/*
	 * Disable HC_SELECT_IN to be able to use the UHS mode select
	 * configuration from Host Control2 register for all other
	 * modes.
	 * Write 0 to HC_SELECT_IN and HC_SELECT_IN_EN field
	 * in VENDOR_SPEC_FUNC
	 */
	config = readl_relaxed(host->ioaddr + CORE_VENDOR_SPEC);
	config &= ~CORE_HC_SELECT_IN_EN;
	config &= ~CORE_HC_SELECT_IN_MASK;
	writel_relaxed(config, host->ioaddr + CORE_VENDOR_SPEC);

	/*
	 * Make sure above writes impacting free running MCLK are completed
	 * before changing the clk_rate at GCC.
	 */
	wmb();
}

static void msm_hc_select_hs400(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	struct mmc_ios ios = host->mmc->ios;
	u32 config, dll_lock;
	int rc;

	/* Select the divided clock (free running MCLK/2) */
	config = readl_relaxed(host->ioaddr + CORE_VENDOR_SPEC);
	config &= ~CORE_HC_MCLK_SEL_MASK;
	config |= CORE_HC_MCLK_SEL_HS400;

	writel_relaxed(config, host->ioaddr + CORE_VENDOR_SPEC);
	/*
	 * Select HS400 mode using the HC_SELECT_IN from VENDOR SPEC
	 * register
	 */
	if ((msm_host->tuning_done || ios.enhanced_strobe) &&
	    !msm_host->calibration_done) {
		config = readl_relaxed(host->ioaddr + CORE_VENDOR_SPEC);
		config |= CORE_HC_SELECT_IN_HS400;
		config |= CORE_HC_SELECT_IN_EN;
		writel_relaxed(config, host->ioaddr + CORE_VENDOR_SPEC);
	}
	if (!msm_host->clk_rate && !msm_host->use_cdclp533) {
		/*
		 * Poll on DLL_LOCK or DDR_DLL_LOCK bits in
		 * CORE_DLL_STATUS to be set.  This should get set
		 * within 15 us at 200 MHz.
		 */
		rc = readl_relaxed_poll_timeout(host->ioaddr +
						CORE_DLL_STATUS,
						dll_lock,
						(dll_lock &
						(CORE_DLL_LOCK |
						CORE_DDR_DLL_LOCK)), 10,
						1000);
		if (rc == -ETIMEDOUT)
			pr_err("%s: Unable to get DLL_LOCK/DDR_DLL_LOCK, dll_status: 0x%08x\n",
			       mmc_hostname(host->mmc), dll_lock);
	}
	/*
	 * Make sure above writes impacting free running MCLK are completed
	 * before changing the clk_rate at GCC.
	 */
	wmb();
}

/*
 * sdhci_msm_hc_select_mode :- In general all timing modes are
 * controlled via UHS mode select in Host Control2 register.
 * eMMC specific HS200/HS400 doesn't have their respective modes
 * defined here, hence we use these values.
 *
 * HS200 - SDR104 (Since they both are equivalent in functionality)
 * HS400 - This involves multiple configurations
 *		Initially SDR104 - when tuning is required as HS200
 *		Then when switching to DDR @ 400MHz (HS400) we use
 *		the vendor specific HC_SELECT_IN to control the mode.
 *
 * In addition to controlling the modes we also need to select the
 * correct input clock for DLL depending on the mode.
 *
 * HS400 - divided clock (free running MCLK/2)
 * All other modes - default (free running MCLK)
 */
static void sdhci_msm_hc_select_mode(struct sdhci_host *host)
{
	struct mmc_ios ios = host->mmc->ios;

	if (ios.timing == MMC_TIMING_MMC_HS400 ||
	    host->flags & SDHCI_HS400_TUNING)
		msm_hc_select_hs400(host);
	else
		msm_hc_select_default(host);
}

static int sdhci_msm_cdclp533_calibration(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	u32 config, calib_done;
	int ret;

	pr_debug("%s: %s: Enter\n", mmc_hostname(host->mmc), __func__);

	/*
	 * Retuning in HS400 (DDR mode) will fail, just reset the
	 * tuning block and restore the saved tuning phase.
	 */
	ret = msm_init_cm_dll(host);
	if (ret)
		goto out;

	/* Set the selected phase in delay line hw block */
	ret = msm_config_cm_dll_phase(host, msm_host->saved_tuning_phase);
	if (ret)
		goto out;

	config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG);
	config |= CORE_CMD_DAT_TRACK_SEL;
	writel_relaxed(config, host->ioaddr + CORE_DLL_CONFIG);

	config = readl_relaxed(host->ioaddr + CORE_DDR_200_CFG);
	config &= ~CORE_CDC_T4_DLY_SEL;
	writel_relaxed(config, host->ioaddr + CORE_DDR_200_CFG);

	config = readl_relaxed(host->ioaddr + CORE_CSR_CDC_GEN_CFG);
	config &= ~CORE_CDC_SWITCH_BYPASS_OFF;
	writel_relaxed(config, host->ioaddr + CORE_CSR_CDC_GEN_CFG);

	config = readl_relaxed(host->ioaddr + CORE_CSR_CDC_GEN_CFG);
	config |= CORE_CDC_SWITCH_RC_EN;
	writel_relaxed(config, host->ioaddr + CORE_CSR_CDC_GEN_CFG);

	config = readl_relaxed(host->ioaddr + CORE_DDR_200_CFG);
	config &= ~CORE_START_CDC_TRAFFIC;
	writel_relaxed(config, host->ioaddr + CORE_DDR_200_CFG);

	/* Perform CDC Register Initialization Sequence */

	writel_relaxed(0x11800EC, host->ioaddr + CORE_CSR_CDC_CTLR_CFG0);
	writel_relaxed(0x3011111, host->ioaddr + CORE_CSR_CDC_CTLR_CFG1);
	writel_relaxed(0x1201000, host->ioaddr + CORE_CSR_CDC_CAL_TIMER_CFG0);
	writel_relaxed(0x4, host->ioaddr + CORE_CSR_CDC_CAL_TIMER_CFG1);
	writel_relaxed(0xCB732020, host->ioaddr + CORE_CSR_CDC_REFCOUNT_CFG);
	writel_relaxed(0xB19, host->ioaddr + CORE_CSR_CDC_COARSE_CAL_CFG);
	writel_relaxed(0x4E2, host->ioaddr + CORE_CSR_CDC_DELAY_CFG);
	writel_relaxed(0x0, host->ioaddr + CORE_CDC_OFFSET_CFG);
	writel_relaxed(0x16334, host->ioaddr + CORE_CDC_SLAVE_DDA_CFG);

	/* CDC HW Calibration */

	config = readl_relaxed(host->ioaddr + CORE_CSR_CDC_CTLR_CFG0);
	config |= CORE_SW_TRIG_FULL_CALIB;
	writel_relaxed(config, host->ioaddr + CORE_CSR_CDC_CTLR_CFG0);

	config = readl_relaxed(host->ioaddr + CORE_CSR_CDC_CTLR_CFG0);
	config &= ~CORE_SW_TRIG_FULL_CALIB;
	writel_relaxed(config, host->ioaddr + CORE_CSR_CDC_CTLR_CFG0);

	config = readl_relaxed(host->ioaddr + CORE_CSR_CDC_CTLR_CFG0);
	config |= CORE_HW_AUTOCAL_ENA;
	writel_relaxed(config, host->ioaddr + CORE_CSR_CDC_CTLR_CFG0);

	config = readl_relaxed(host->ioaddr + CORE_CSR_CDC_CAL_TIMER_CFG0);
	config |= CORE_TIMER_ENA;
	writel_relaxed(config, host->ioaddr + CORE_CSR_CDC_CAL_TIMER_CFG0);

	ret = readl_relaxed_poll_timeout(host->ioaddr + CORE_CSR_CDC_STATUS0,
					 calib_done,
					 (calib_done & CORE_CALIBRATION_DONE),
					 1, 50);

	if (ret == -ETIMEDOUT) {
		pr_err("%s: %s: CDC calibration was not completed\n",
		       mmc_hostname(host->mmc), __func__);
		goto out;
	}

	ret = readl_relaxed(host->ioaddr + CORE_CSR_CDC_STATUS0)
			& CORE_CDC_ERROR_CODE_MASK;
	if (ret) {
		pr_err("%s: %s: CDC error code %d\n",
		       mmc_hostname(host->mmc), __func__, ret);
		ret = -EINVAL;
		goto out;
	}

	config = readl_relaxed(host->ioaddr + CORE_DDR_200_CFG);
	config |= CORE_START_CDC_TRAFFIC;
	writel_relaxed(config, host->ioaddr + CORE_DDR_200_CFG);
out:
	pr_debug("%s: %s: Exit, ret %d\n", mmc_hostname(host->mmc),
		 __func__, ret);
	return ret;
}

static int sdhci_msm_cm_dll_sdc4_calibration(struct sdhci_host *host)
{
	struct mmc_host *mmc = host->mmc;
	u32 dll_status, config;
	int ret;

	pr_debug("%s: %s: Enter\n", mmc_hostname(host->mmc), __func__);

	/*
	 * Currently the CORE_DDR_CONFIG register defaults to desired
	 * configuration on reset. Currently reprogramming the power on
	 * reset (POR) value in case it might have been modified by
	 * bootloaders. In the future, if this changes, then the desired
	 * values will need to be programmed appropriately.
	 */
	writel_relaxed(DDR_CONFIG_POR_VAL, host->ioaddr + CORE_DDR_CONFIG);

	if (mmc->ios.enhanced_strobe) {
		config = readl_relaxed(host->ioaddr + CORE_DDR_200_CFG);
		config |= CORE_CMDIN_RCLK_EN;
		writel_relaxed(config, host->ioaddr + CORE_DDR_200_CFG);
	}

	config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG_2);
	config |= CORE_DDR_CAL_EN;
	writel_relaxed(config, host->ioaddr + CORE_DLL_CONFIG_2);

	ret = readl_relaxed_poll_timeout(host->ioaddr + CORE_DLL_STATUS,
					 dll_status,
					 (dll_status & CORE_DDR_DLL_LOCK),
					 10, 1000);

	if (ret == -ETIMEDOUT) {
		pr_err("%s: %s: CM_DLL_SDC4 calibration was not completed\n",
		       mmc_hostname(host->mmc), __func__);
		goto out;
	}

	config = readl_relaxed(host->ioaddr + CORE_VENDOR_SPEC3);
	config |= CORE_PWRSAVE_DLL;
	writel_relaxed(config, host->ioaddr + CORE_VENDOR_SPEC3);

	/*
	 * Drain writebuffer to ensure above DLL calibration
	 * and PWRSAVE DLL is enabled.
	 */
	wmb();
out:
	pr_debug("%s: %s: Exit, ret %d\n", mmc_hostname(host->mmc),
		 __func__, ret);
	return ret;
}

static int sdhci_msm_hs400_dll_calibration(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	struct mmc_host *mmc = host->mmc;
	int ret;
	u32 config;

	pr_debug("%s: %s: Enter\n", mmc_hostname(host->mmc), __func__);

	/*
	 * Retuning in HS400 (DDR mode) will fail, just reset the
	 * tuning block and restore the saved tuning phase.
	 */
	ret = msm_init_cm_dll(host);
	if (ret)
		goto out;

	if (!mmc->ios.enhanced_strobe) {
		/* Set the selected phase in delay line hw block */
		ret = msm_config_cm_dll_phase(host,
					      msm_host->saved_tuning_phase);
		if (ret)
			goto out;
		config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG);
		config |= CORE_CMD_DAT_TRACK_SEL;
		writel_relaxed(config, host->ioaddr + CORE_DLL_CONFIG);
	}

	if (msm_host->use_cdclp533)
		ret = sdhci_msm_cdclp533_calibration(host);
	else
		ret = sdhci_msm_cm_dll_sdc4_calibration(host);
out:
	pr_debug("%s: %s: Exit, ret %d\n", mmc_hostname(host->mmc),
		 __func__, ret);
	return ret;
}

static int sdhci_msm_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct sdhci_host *host = mmc_priv(mmc);
	int tuning_seq_cnt = 3;
	u8 phase, tuned_phases[16], tuned_phase_cnt = 0;
	int rc;
	struct mmc_ios ios = host->mmc->ios;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);

	/*
	 * Tuning is required for SDR104, HS200 and HS400 cards and
	 * if clock frequency is greater than 100MHz in these modes.
	 */
	if (host->clock <= CORE_FREQ_100MHZ ||
	    !(ios.timing == MMC_TIMING_MMC_HS400 ||
	    ios.timing == MMC_TIMING_MMC_HS200 ||
	    ios.timing == MMC_TIMING_UHS_SDR104))
		return 0;

	/*
	 * For HS400 tuning in HS200 timing requires:
	 * - select MCLK/2 in VENDOR_SPEC
	 * - program MCLK to 400MHz (or nearest supported) in GCC
	 */
	if (host->flags & SDHCI_HS400_TUNING) {
		sdhci_msm_hc_select_mode(host);
		msm_set_clock_rate_for_bus_mode(host, ios.clock);
		host->flags &= ~SDHCI_HS400_TUNING;
	}

retry:
	/* First of all reset the tuning block */
	rc = msm_init_cm_dll(host);
	if (rc)
		return rc;

	phase = 0;
	do {
		/* Set the phase in delay line hw block */
		rc = msm_config_cm_dll_phase(host, phase);
		if (rc)
			return rc;

		msm_host->saved_tuning_phase = phase;
		rc = mmc_send_tuning(mmc, opcode, NULL);
		if (!rc) {
			/* Tuning is successful at this tuning point */
			tuned_phases[tuned_phase_cnt++] = phase;
			dev_dbg(mmc_dev(mmc), "%s: Found good phase = %d\n",
				 mmc_hostname(mmc), phase);
		}
	} while (++phase < ARRAY_SIZE(tuned_phases));

	if (tuned_phase_cnt) {
		rc = msm_find_most_appropriate_phase(host, tuned_phases,
						     tuned_phase_cnt);
		if (rc < 0)
			return rc;
		else
			phase = rc;

		/*
		 * Finally set the selected phase in delay
		 * line hw block.
		 */
		rc = msm_config_cm_dll_phase(host, phase);
		if (rc)
			return rc;
		dev_dbg(mmc_dev(mmc), "%s: Setting the tuning phase to %d\n",
			 mmc_hostname(mmc), phase);
	} else {
		if (--tuning_seq_cnt)
			goto retry;
		/* Tuning failed */
		dev_dbg(mmc_dev(mmc), "%s: No tuning point found\n",
		       mmc_hostname(mmc));
		rc = -EIO;
	}

	if (!rc)
		msm_host->tuning_done = true;
	return rc;
}

/*
 * sdhci_msm_hs400 - Calibrate the DLL for HS400 bus speed mode operation.
 * This needs to be done for both tuning and enhanced_strobe mode.
 * DLL operation is only needed for clock > 100MHz. For clock <= 100MHz
 * fixed feedback clock is used.
 */
static void sdhci_msm_hs400(struct sdhci_host *host, struct mmc_ios *ios)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	int ret;

	if (host->clock > CORE_FREQ_100MHZ &&
	    (msm_host->tuning_done || ios->enhanced_strobe) &&
	    !msm_host->calibration_done) {
		ret = sdhci_msm_hs400_dll_calibration(host);
		if (!ret)
			msm_host->calibration_done = true;
		else
			pr_err("%s: Failed to calibrate DLL for hs400 mode (%d)\n",
			       mmc_hostname(host->mmc), ret);
	}
}

static void sdhci_msm_set_uhs_signaling(struct sdhci_host *host,
					unsigned int uhs)
{
	struct mmc_host *mmc = host->mmc;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	u16 ctrl_2;
	u32 config;

	ctrl_2 = sdhci_readw(host, SDHCI_HOST_CONTROL2);
	/* Select Bus Speed Mode for host */
	ctrl_2 &= ~SDHCI_CTRL_UHS_MASK;
	switch (uhs) {
	case MMC_TIMING_UHS_SDR12:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR12;
		break;
	case MMC_TIMING_UHS_SDR25:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR25;
		break;
	case MMC_TIMING_UHS_SDR50:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR50;
		break;
	case MMC_TIMING_MMC_HS400:
	case MMC_TIMING_MMC_HS200:
	case MMC_TIMING_UHS_SDR104:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR104;
		break;
	case MMC_TIMING_UHS_DDR50:
	case MMC_TIMING_MMC_DDR52:
		ctrl_2 |= SDHCI_CTRL_UHS_DDR50;
		break;
	}

	/*
	 * When clock frequency is less than 100MHz, the feedback clock must be
	 * provided and DLL must not be used so that tuning can be skipped. To
	 * provide feedback clock, the mode selection can be any value less
	 * than 3'b011 in bits [2:0] of HOST CONTROL2 register.
	 */
	if (host->clock <= CORE_FREQ_100MHZ) {
		if (uhs == MMC_TIMING_MMC_HS400 ||
		    uhs == MMC_TIMING_MMC_HS200 ||
		    uhs == MMC_TIMING_UHS_SDR104)
			ctrl_2 &= ~SDHCI_CTRL_UHS_MASK;
		/*
		 * DLL is not required for clock <= 100MHz
		 * Thus, make sure DLL it is disabled when not required
		 */
		config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG);
		config |= CORE_DLL_RST;
		writel_relaxed(config, host->ioaddr + CORE_DLL_CONFIG);

		config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG);
		config |= CORE_DLL_PDN;
		writel_relaxed(config, host->ioaddr + CORE_DLL_CONFIG);

		/*
		 * The DLL needs to be restored and CDCLP533 recalibrated
		 * when the clock frequency is set back to 400MHz.
		 */
		msm_host->calibration_done = false;
	}

	dev_dbg(mmc_dev(mmc), "%s: clock=%u uhs=%u ctrl_2=0x%x\n",
		mmc_hostname(host->mmc), host->clock, uhs, ctrl_2);
	sdhci_writew(host, ctrl_2, SDHCI_HOST_CONTROL2);

	if (mmc->ios.timing == MMC_TIMING_MMC_HS400)
		sdhci_msm_hs400(host, &mmc->ios);
}

static inline void sdhci_msm_init_pwr_irq_wait(struct sdhci_msm_host *msm_host)
{
	init_waitqueue_head(&msm_host->pwr_irq_wait);
}

static inline void sdhci_msm_complete_pwr_irq_wait(
		struct sdhci_msm_host *msm_host)
{
	wake_up(&msm_host->pwr_irq_wait);
}

/*
 * sdhci_msm_check_power_status API should be called when registers writes
 * which can toggle sdhci IO bus ON/OFF or change IO lines HIGH/LOW happens.
 * To what state the register writes will change the IO lines should be passed
 * as the argument req_type. This API will check whether the IO line's state
 * is already the expected state and will wait for power irq only if
 * power irq is expected to be trigerred based on the current IO line state
 * and expected IO line state.
 */
static void sdhci_msm_check_power_status(struct sdhci_host *host, u32 req_type)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	bool done = false;
	u32 val;

	pr_debug("%s: %s: request %d curr_pwr_state %x curr_io_level %x\n",
			mmc_hostname(host->mmc), __func__, req_type,
			msm_host->curr_pwr_state, msm_host->curr_io_level);

	/*
	 * The power interrupt will not be generated for signal voltage
	 * switches if SWITCHABLE_SIGNALING_VOLTAGE in MCI_GENERICS is not set.
	 */
	val = readl(msm_host->core_mem + CORE_MCI_GENERICS);
	if ((req_type & REQ_IO_HIGH || req_type & REQ_IO_LOW) &&
	    !(val & SWITCHABLE_SIGNALING_VOLTAGE)) {
		return;
	}

	/*
	 * The IRQ for request type IO High/LOW will be generated when -
	 * there is a state change in 1.8V enable bit (bit 3) of
	 * SDHCI_HOST_CONTROL2 register. The reset state of that bit is 0
	 * which indicates 3.3V IO voltage. So, when MMC core layer tries
	 * to set it to 3.3V before card detection happens, the
	 * IRQ doesn't get triggered as there is no state change in this bit.
	 * The driver already handles this case by changing the IO voltage
	 * level to high as part of controller power up sequence. Hence, check
	 * for host->pwr to handle a case where IO voltage high request is
	 * issued even before controller power up.
	 */
	if ((req_type & REQ_IO_HIGH) && !host->pwr) {
		pr_debug("%s: do not wait for power IRQ that never comes, req_type: %d\n",
				mmc_hostname(host->mmc), req_type);
		return;
	}
	if ((req_type & msm_host->curr_pwr_state) ||
			(req_type & msm_host->curr_io_level))
		done = true;
	/*
	 * This is needed here to handle cases where register writes will
	 * not change the current bus state or io level of the controller.
	 * In this case, no power irq will be triggerred and we should
	 * not wait.
	 */
	if (!done) {
		if (!wait_event_timeout(msm_host->pwr_irq_wait,
				msm_host->pwr_irq_flag,
				msecs_to_jiffies(MSM_PWR_IRQ_TIMEOUT_MS)))
			dev_warn(&msm_host->pdev->dev,
				 "%s: pwr_irq for req: (%d) timed out\n",
				 mmc_hostname(host->mmc), req_type);
	}
	pr_debug("%s: %s: request %d done\n", mmc_hostname(host->mmc),
			__func__, req_type);
}

static void sdhci_msm_dump_pwr_ctrl_regs(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);

	pr_err("%s: PWRCTL_STATUS: 0x%08x | PWRCTL_MASK: 0x%08x | PWRCTL_CTL: 0x%08x\n",
			mmc_hostname(host->mmc),
			readl_relaxed(msm_host->core_mem + CORE_PWRCTL_STATUS),
			readl_relaxed(msm_host->core_mem + CORE_PWRCTL_MASK),
			readl_relaxed(msm_host->core_mem + CORE_PWRCTL_CTL));
}

static void sdhci_msm_handle_pwr_irq(struct sdhci_host *host, int irq)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	u32 irq_status, irq_ack = 0;
	int retry = 10;
	int pwr_state = 0, io_level = 0;


	irq_status = readl_relaxed(msm_host->core_mem + CORE_PWRCTL_STATUS);
	irq_status &= INT_MASK;

	writel_relaxed(irq_status, msm_host->core_mem + CORE_PWRCTL_CLEAR);

	/*
	 * There is a rare HW scenario where the first clear pulse could be
	 * lost when actual reset and clear/read of status register is
	 * happening at a time. Hence, retry for at least 10 times to make
	 * sure status register is cleared. Otherwise, this will result in
	 * a spurious power IRQ resulting in system instability.
	 */
	while (irq_status & readl_relaxed(msm_host->core_mem +
				CORE_PWRCTL_STATUS)) {
		if (retry == 0) {
			pr_err("%s: Timedout clearing (0x%x) pwrctl status register\n",
					mmc_hostname(host->mmc), irq_status);
			sdhci_msm_dump_pwr_ctrl_regs(host);
			WARN_ON(1);
			break;
		}
		writel_relaxed(irq_status,
				msm_host->core_mem + CORE_PWRCTL_CLEAR);
		retry--;
		udelay(10);
	}

	/* Handle BUS ON/OFF*/
	if (irq_status & CORE_PWRCTL_BUS_ON) {
		pwr_state = REQ_BUS_ON;
		io_level = REQ_IO_HIGH;
		irq_ack |= CORE_PWRCTL_BUS_SUCCESS;
	}
	if (irq_status & CORE_PWRCTL_BUS_OFF) {
		pwr_state = REQ_BUS_OFF;
		io_level = REQ_IO_LOW;
		irq_ack |= CORE_PWRCTL_BUS_SUCCESS;
	}
	/* Handle IO LOW/HIGH */
	if (irq_status & CORE_PWRCTL_IO_LOW) {
		io_level = REQ_IO_LOW;
		irq_ack |= CORE_PWRCTL_IO_SUCCESS;
	}
	if (irq_status & CORE_PWRCTL_IO_HIGH) {
		io_level = REQ_IO_HIGH;
		irq_ack |= CORE_PWRCTL_IO_SUCCESS;
	}

	/*
	 * The driver has to acknowledge the interrupt, switch voltages and
	 * report back if it succeded or not to this register. The voltage
	 * switches are handled by the sdhci core, so just report success.
	 */
	writel_relaxed(irq_ack, msm_host->core_mem + CORE_PWRCTL_CTL);

	if (pwr_state)
		msm_host->curr_pwr_state = pwr_state;
	if (io_level)
		msm_host->curr_io_level = io_level;

	pr_debug("%s: %s: Handled IRQ(%d), irq_status=0x%x, ack=0x%x\n",
		mmc_hostname(msm_host->mmc), __func__, irq, irq_status,
		irq_ack);
}

static irqreturn_t sdhci_msm_pwr_irq(int irq, void *data)
{
	struct sdhci_host *host = (struct sdhci_host *)data;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);

	sdhci_msm_handle_pwr_irq(host, irq);
	msm_host->pwr_irq_flag = 1;
	sdhci_msm_complete_pwr_irq_wait(msm_host);


	return IRQ_HANDLED;
}

static unsigned int sdhci_msm_get_max_clock(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	struct clk *core_clk = msm_host->bulk_clks[0].clk;

	return clk_round_rate(core_clk, ULONG_MAX);
}

static unsigned int sdhci_msm_get_min_clock(struct sdhci_host *host)
{
	return SDHCI_MSM_MIN_CLOCK;
}

/**
 * __sdhci_msm_set_clock - sdhci_msm clock control.
 *
 * Description:
 * MSM controller does not use internal divider and
 * instead directly control the GCC clock as per
 * HW recommendation.
 **/
static void __sdhci_msm_set_clock(struct sdhci_host *host, unsigned int clock)
{
	u16 clk;
	/*
	 * Keep actual_clock as zero -
	 * - since there is no divider used so no need of having actual_clock.
	 * - MSM controller uses SDCLK for data timeout calculation. If
	 *   actual_clock is zero, host->clock is taken for calculation.
	 */
	host->mmc->actual_clock = 0;

	sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);

	if (clock == 0)
		return;

	/*
	 * MSM controller do not use clock divider.
	 * Thus read SDHCI_CLOCK_CONTROL and only enable
	 * clock with no divider value programmed.
	 */
	clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	sdhci_enable_clk(host, clk);
}

/* sdhci_msm_set_clock - Called with (host->lock) spinlock held. */
static void sdhci_msm_set_clock(struct sdhci_host *host, unsigned int clock)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);

	if (!clock) {
		msm_host->clk_rate = clock;
		goto out;
	}

	sdhci_msm_hc_select_mode(host);

	msm_set_clock_rate_for_bus_mode(host, clock);
out:
	__sdhci_msm_set_clock(host, clock);
}

/*
 * Platform specific register write functions. This is so that, if any
 * register write needs to be followed up by platform specific actions,
 * they can be added here. These functions can go to sleep when writes
 * to certain registers are done.
 * These functions are relying on sdhci_set_ios not using spinlock.
 */
static int __sdhci_msm_check_write(struct sdhci_host *host, u16 val, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	u32 req_type = 0;

	switch (reg) {
	case SDHCI_HOST_CONTROL2:
		req_type = (val & SDHCI_CTRL_VDD_180) ? REQ_IO_LOW :
			REQ_IO_HIGH;
		break;
	case SDHCI_SOFTWARE_RESET:
		if (host->pwr && (val & SDHCI_RESET_ALL))
			req_type = REQ_BUS_OFF;
		break;
	case SDHCI_POWER_CONTROL:
		req_type = !val ? REQ_BUS_OFF : REQ_BUS_ON;
		break;
	}

	if (req_type) {
		msm_host->pwr_irq_flag = 0;
		/*
		 * Since this register write may trigger a power irq, ensure
		 * all previous register writes are complete by this point.
		 */
		mb();
	}
	return req_type;
}

/* This function may sleep*/
static void sdhci_msm_writew(struct sdhci_host *host, u16 val, int reg)
{
	u32 req_type = 0;

	req_type = __sdhci_msm_check_write(host, val, reg);
	writew_relaxed(val, host->ioaddr + reg);

	if (req_type)
		sdhci_msm_check_power_status(host, req_type);
}

/* This function may sleep*/
static void sdhci_msm_writeb(struct sdhci_host *host, u8 val, int reg)
{
	u32 req_type = 0;

	req_type = __sdhci_msm_check_write(host, val, reg);

	writeb_relaxed(val, host->ioaddr + reg);

	if (req_type)
		sdhci_msm_check_power_status(host, req_type);
}

static const struct of_device_id sdhci_msm_dt_match[] = {
	{ .compatible = "qcom,sdhci-msm-v4" },
	{},
};

MODULE_DEVICE_TABLE(of, sdhci_msm_dt_match);

static const struct sdhci_ops sdhci_msm_ops = {
	.reset = sdhci_reset,
	.set_clock = sdhci_msm_set_clock,
	.get_min_clock = sdhci_msm_get_min_clock,
	.get_max_clock = sdhci_msm_get_max_clock,
	.set_bus_width = sdhci_set_bus_width,
	.set_uhs_signaling = sdhci_msm_set_uhs_signaling,
	.write_w = sdhci_msm_writew,
	.write_b = sdhci_msm_writeb,
};

static const struct sdhci_pltfm_data sdhci_msm_pdata = {
	.quirks = SDHCI_QUIRK_BROKEN_CARD_DETECTION |
		  SDHCI_QUIRK_NO_CARD_NO_RESET |
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.quirks2 = SDHCI_QUIRK2_PRESET_VALUE_BROKEN,
	.ops = &sdhci_msm_ops,
};

static int sdhci_msm_probe(struct platform_device *pdev)
{
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_msm_host *msm_host;
	struct resource *core_memres;
	struct clk *clk;
	int ret;
	u16 host_version, core_minor;
	u32 core_version, config;
	u8 core_major;

	host = sdhci_pltfm_init(pdev, &sdhci_msm_pdata, sizeof(*msm_host));
	if (IS_ERR(host))
		return PTR_ERR(host);

	host->sdma_boundary = 0;
	pltfm_host = sdhci_priv(host);
	msm_host = sdhci_pltfm_priv(pltfm_host);
	msm_host->mmc = host->mmc;
	msm_host->pdev = pdev;

	ret = mmc_of_parse(host->mmc);
	if (ret)
		goto pltfm_free;

	sdhci_get_of_property(pdev);

	msm_host->saved_tuning_phase = INVALID_TUNING_PHASE;

	/* Setup SDCC bus voter clock. */
	msm_host->bus_clk = devm_clk_get(&pdev->dev, "bus");
	if (!IS_ERR(msm_host->bus_clk)) {
		/* Vote for max. clk rate for max. performance */
		ret = clk_set_rate(msm_host->bus_clk, INT_MAX);
		if (ret)
			goto pltfm_free;
		ret = clk_prepare_enable(msm_host->bus_clk);
		if (ret)
			goto pltfm_free;
	}

	/* Setup main peripheral bus clock */
	clk = devm_clk_get(&pdev->dev, "iface");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_err(&pdev->dev, "Peripheral clk setup failed (%d)\n", ret);
		goto bus_clk_disable;
	}
	msm_host->bulk_clks[1].clk = clk;

	/* Setup SDC MMC clock */
	clk = devm_clk_get(&pdev->dev, "core");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_err(&pdev->dev, "SDC MMC clk setup failed (%d)\n", ret);
		goto bus_clk_disable;
	}
	msm_host->bulk_clks[0].clk = clk;

	/* Vote for maximum clock rate for maximum performance */
	ret = clk_set_rate(clk, INT_MAX);
	if (ret)
		dev_warn(&pdev->dev, "core clock boost failed\n");

	clk = devm_clk_get(&pdev->dev, "cal");
	if (IS_ERR(clk))
		clk = NULL;
	msm_host->bulk_clks[2].clk = clk;

	clk = devm_clk_get(&pdev->dev, "sleep");
	if (IS_ERR(clk))
		clk = NULL;
	msm_host->bulk_clks[3].clk = clk;

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(msm_host->bulk_clks),
				      msm_host->bulk_clks);
	if (ret)
		goto bus_clk_disable;

	/*
	 * xo clock is needed for FLL feature of cm_dll.
	 * In case if xo clock is not mentioned in DT, warn and proceed.
	 */
	msm_host->xo_clk = devm_clk_get(&pdev->dev, "xo");
	if (IS_ERR(msm_host->xo_clk)) {
		ret = PTR_ERR(msm_host->xo_clk);
		dev_warn(&pdev->dev, "TCXO clk not present (%d)\n", ret);
	}

	core_memres = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	msm_host->core_mem = devm_ioremap_resource(&pdev->dev, core_memres);

	if (IS_ERR(msm_host->core_mem)) {
		dev_err(&pdev->dev, "Failed to remap registers\n");
		ret = PTR_ERR(msm_host->core_mem);
		goto clk_disable;
	}

	/* Reset the vendor spec register to power on reset state */
	writel_relaxed(CORE_VENDOR_SPEC_POR_VAL,
		       host->ioaddr + CORE_VENDOR_SPEC);

	/* Set HC_MODE_EN bit in HC_MODE register */
	writel_relaxed(HC_MODE_EN, (msm_host->core_mem + CORE_HC_MODE));

	config = readl_relaxed(msm_host->core_mem + CORE_HC_MODE);
	config |= FF_CLK_SW_RST_DIS;
	writel_relaxed(config, msm_host->core_mem + CORE_HC_MODE);

	host_version = readw_relaxed((host->ioaddr + SDHCI_HOST_VERSION));
	dev_dbg(&pdev->dev, "Host Version: 0x%x Vendor Version 0x%x\n",
		host_version, ((host_version & SDHCI_VENDOR_VER_MASK) >>
			       SDHCI_VENDOR_VER_SHIFT));

	core_version = readl_relaxed(msm_host->core_mem + CORE_MCI_VERSION);
	core_major = (core_version & CORE_VERSION_MAJOR_MASK) >>
		      CORE_VERSION_MAJOR_SHIFT;
	core_minor = core_version & CORE_VERSION_MINOR_MASK;
	dev_dbg(&pdev->dev, "MCI Version: 0x%08x, major: 0x%04x, minor: 0x%02x\n",
		core_version, core_major, core_minor);

	if (core_major == 1 && core_minor >= 0x42)
		msm_host->use_14lpp_dll_reset = true;

	/*
	 * SDCC 5 controller with major version 1, minor version 0x34 and later
	 * with HS 400 mode support will use CM DLL instead of CDC LP 533 DLL.
	 */
	if (core_major == 1 && core_minor < 0x34)
		msm_host->use_cdclp533 = true;

	/*
	 * Support for some capabilities is not advertised by newer
	 * controller versions and must be explicitly enabled.
	 */
	if (core_major >= 1 && core_minor != 0x11 && core_minor != 0x12) {
		config = readl_relaxed(host->ioaddr + SDHCI_CAPABILITIES);
		config |= SDHCI_CAN_VDD_300 | SDHCI_CAN_DO_8BIT;
		writel_relaxed(config, host->ioaddr +
			       CORE_VENDOR_SPEC_CAPABILITIES0);
	}

	/*
	 * Power on reset state may trigger power irq if previous status of
	 * PWRCTL was either BUS_ON or IO_HIGH_V. So before enabling pwr irq
	 * interrupt in GIC, any pending power irq interrupt should be
	 * acknowledged. Otherwise power irq interrupt handler would be
	 * fired prematurely.
	 */
	sdhci_msm_handle_pwr_irq(host, 0);

	/*
	 * Ensure that above writes are propogated before interrupt enablement
	 * in GIC.
	 */
	mb();

	/* Setup IRQ for handling power/voltage tasks with PMIC */
	msm_host->pwr_irq = platform_get_irq_byname(pdev, "pwr_irq");
	if (msm_host->pwr_irq < 0) {
		dev_err(&pdev->dev, "Get pwr_irq failed (%d)\n",
			msm_host->pwr_irq);
		ret = msm_host->pwr_irq;
		goto clk_disable;
	}

	sdhci_msm_init_pwr_irq_wait(msm_host);
	/* Enable pwr irq interrupts */
	writel_relaxed(INT_MASK, msm_host->core_mem + CORE_PWRCTL_MASK);

	ret = devm_request_threaded_irq(&pdev->dev, msm_host->pwr_irq, NULL,
					sdhci_msm_pwr_irq, IRQF_ONESHOT,
					dev_name(&pdev->dev), host);
	if (ret) {
		dev_err(&pdev->dev, "Request IRQ failed (%d)\n", ret);
		goto clk_disable;
	}

	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev,
					 MSM_MMC_AUTOSUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(&pdev->dev);

	host->mmc_host_ops.execute_tuning = sdhci_msm_execute_tuning;
	ret = sdhci_add_host(host);
	if (ret)
		goto pm_runtime_disable;

	pm_runtime_mark_last_busy(&pdev->dev);
	pm_runtime_put_autosuspend(&pdev->dev);

	return 0;

pm_runtime_disable:
	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);
clk_disable:
	clk_bulk_disable_unprepare(ARRAY_SIZE(msm_host->bulk_clks),
				   msm_host->bulk_clks);
bus_clk_disable:
	if (!IS_ERR(msm_host->bus_clk))
		clk_disable_unprepare(msm_host->bus_clk);
pltfm_free:
	sdhci_pltfm_free(pdev);
	return ret;
}

static int sdhci_msm_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	int dead = (readl_relaxed(host->ioaddr + SDHCI_INT_STATUS) ==
		    0xffffffff);

	sdhci_remove_host(host, dead);

	pm_runtime_get_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);

	clk_bulk_disable_unprepare(ARRAY_SIZE(msm_host->bulk_clks),
				   msm_host->bulk_clks);
	if (!IS_ERR(msm_host->bus_clk))
		clk_disable_unprepare(msm_host->bus_clk);
	sdhci_pltfm_free(pdev);
	return 0;
}

#ifdef CONFIG_PM
static int sdhci_msm_runtime_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);

	clk_bulk_disable_unprepare(ARRAY_SIZE(msm_host->bulk_clks),
				   msm_host->bulk_clks);

	return 0;
}

static int sdhci_msm_runtime_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);

	return clk_bulk_prepare_enable(ARRAY_SIZE(msm_host->bulk_clks),
				       msm_host->bulk_clks);
}
#endif

static const struct dev_pm_ops sdhci_msm_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(sdhci_msm_runtime_suspend,
			   sdhci_msm_runtime_resume,
			   NULL)
};

static struct platform_driver sdhci_msm_driver = {
	.probe = sdhci_msm_probe,
	.remove = sdhci_msm_remove,
	.driver = {
		   .name = "sdhci_msm",
		   .of_match_table = sdhci_msm_dt_match,
		   .pm = &sdhci_msm_pm_ops,
	},
};

module_platform_driver(sdhci_msm_driver);

MODULE_DESCRIPTION("Qualcomm Secure Digital Host Controller Interface driver");
MODULE_LICENSE("GPL v2");
