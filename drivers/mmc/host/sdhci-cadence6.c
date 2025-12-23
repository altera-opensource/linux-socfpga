// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * PHY and host controller support for Cadence SD6HC SDHCI
 *
 * This file provides comprehensive support for Cadence's sixth-generation
 * SDHCI controller (SD6HC), handling both low-level PHY operations and
 * high-level host controller programming. The implementation includes PHY
 * initialization and timing calculations, DLL (Delay-Locked Loop)
 * management and configuration, host controller register programming for
 * IO delay compensation and signal timing optimization.
 *
 * Copyright (C) 2025 Altera Corporation
 *   Author: Tanmay Kathpalia <tanmay.kathpalia@altera.com>
 */

#include "sdhci-cadence.h"

/* IO Delay Information */
#define SDHCI_CDNS_HRS07		0x1c
#define   SDHCI_CDNS_HRS07_RW_COMPENSATE	GENMASK(20, 16)
#define   SDHCI_CDNS_HRS07_IDELAY_VAL		GENMASK(4, 0)

/* PHY Control and Status */
#define SDHCI_CDNS_HRS09		0x24
#define   SDHCI_CDNS_HRS09_RDDATA_EN		BIT(16)
#define   SDHCI_CDNS_HRS09_RDCMD_EN		BIT(15)
#define   SDHCI_CDNS_HRS09_EXTENDED_WR_MODE	BIT(3)
#define   SDHCI_CDNS_HRS09_EXTENDED_RD_MODE	BIT(2)
#define   SDHCI_CDNS_HRS09_PHY_INIT_COMPLETE	BIT(1)
#define   SDHCI_CDNS_HRS09_PHY_SW_RESET		BIT(0)

/* SDCLK start point adjustment */
#define SDHCI_CDNS_HRS10		0x28
#define   SDHCI_CDNS_HRS10_HCSDCLKADJ		GENMASK(19, 16)

/* eMMC Control */
#define SDHCI_CDNS_HRS11		0x2c
#define   SDHCI_CDNS_HRS11_EMMC_RST		BIT(0) /* eMMC reset */

/* CMD/DAT output delay */
#define SDHCI_CDNS_HRS16		0x40
#define   SDHCI_CDNS_HRS16_WRDATA1_SDCLK_DLY	GENMASK(31, 28)
#define   SDHCI_CDNS_HRS16_WRDATA0_SDCLK_DLY	GENMASK(27, 24)
#define   SDHCI_CDNS_HRS16_WRCMD1_SDCLK_DLY	GENMASK(23, 20)
#define   SDHCI_CDNS_HRS16_WRCMD0_SDCLK_DLY	GENMASK(19, 16)
#define   SDHCI_CDNS_HRS16_WRDATA1_DLY		GENMASK(15, 12)
#define   SDHCI_CDNS_HRS16_WRDATA0_DLY		GENMASK(11, 8)
#define   SDHCI_CDNS_HRS16_WRCMD1_DLY		GENMASK(7, 4)
#define   SDHCI_CDNS_HRS16_WRCMD0_DLY		GENMASK(3, 0)

/* PHY Special Function Registers */
/* DQ timing */
#define SDHCI_CDNS6_PHY_DQ_TIMING_REG			0x2000
#define   SDHCI_CDNS6_PHY_DQ_TIMING_IO_MASK_ALWAYS_ON		BIT(31)
#define   SDHCI_CDNS6_PHY_DQ_TIMING_IO_MASK_END		GENMASK(29, 27)
#define   SDHCI_CDNS6_PHY_DQ_TIMING_IO_MASK_START		GENMASK(26, 24)
#define   SDHCI_CDNS6_PHY_DQ_TIMING_DATA_SELECT_OE_END		GENMASK(2, 0)

/* DQS timing */
#define SDHCI_CDNS6_PHY_DQS_TIMING_REG			0x2004
#define   SDHCI_CDNS6_PHY_DQS_TIMING_USE_EXT_LPBK_DQS		BIT(22)
#define   SDHCI_CDNS6_PHY_DQS_TIMING_USE_LPBK_DQS		BIT(21)
#define   SDHCI_CDNS6_PHY_DQS_TIMING_USE_PHONY_DQS		BIT(20)
#define   SDHCI_CDNS6_PHY_DQS_TIMING_USE_PHONY_DQS_CMD		BIT(19)

/* Gate and loopback control */
#define SDHCI_CDNS6_PHY_GATE_LPBK_CTRL_REG		0x2008
#define   SDHCI_CDNS6_PHY_GATE_LPBK_CTRL_SYNC_METHOD		BIT(31)
#define   SDHCI_CDNS6_PHY_GATE_LPBK_CTRL_SW_HALF_CYCLE_SHIFT	BIT(28)
#define   SDHCI_CDNS6_PHY_GATE_LPBK_CTRL_RD_DEL_SEL			GENMASK(24, 19)
#define   SDHCI_CDNS6_PHY_GATE_LPBK_CTRL_UNDERRUN_SUPPRESS		BIT(18)
#define   SDHCI_CDNS6_PHY_GATE_LPBK_CTRL_GATE_CFG_ALWAYS_ON		BIT(6)

/* Master DLL logic */
#define SDHCI_CDNS6_PHY_DLL_MASTER_CTRL_REG		0x200c
#define   SDHCI_CDNS6_PHY_DLL_MASTER_CTRL_BYPASS_MODE		BIT(23)
#define   SDHCI_CDNS6_PHY_DLL_MASTER_CTRL_PHASE_DETECT_SEL		GENMASK(22, 20)
#define   SDHCI_CDNS6_PHY_DLL_MASTER_CTRL_DLL_LOCK_NUM		GENMASK(18, 16)
#define   SDHCI_CDNS6_PHY_DLL_MASTER_CTRL_DLL_START_POINT		GENMASK(7, 0)

/* Slave DLL logic */
#define SDHCI_CDNS6_PHY_DLL_SLAVE_CTRL_REG		0x2010
#define   SDHCI_CDNS6_PHY_DLL_SLAVE_CTRL_READ_DQS_CMD_DELAY	GENMASK(31, 24)
#define   SDHCI_CDNS6_PHY_DLL_SLAVE_CTRL_CLK_WRDQS_DELAY		GENMASK(23, 16)
#define   SDHCI_CDNS6_PHY_DLL_SLAVE_CTRL_CLK_WR_DELAY		GENMASK(15, 8)
#define   SDHCI_CDNS6_PHY_DLL_SLAVE_CTRL_READ_DQS_DELAY		GENMASK(7, 0)

/* Global control settings */
#define SDHCI_CDNS6_PHY_CTRL_REG				0x2080
#define   SDHCI_CDNS6_PHY_CTRL_PHONY_DQS_TIMING		GENMASK(9, 4)

/* Default PHY settings */
#define SDHCI_CDNS6_PHY_DEFAULT_IOCELL_DELAY	2500
#define SDHCI_CDNS6_PHY_DEFAULT_DELAY_ELEMENT	24
#define SDHCI_CDNS6_PHY_DEFAULT_RD_DEL_SEL	52
#define SDHCI_CDNS6_PHY_DEFAULT_DLL_START	4

/* Scale tuning tap (0..39) to 8-bit PHY DLL delay field (0..255) */
#define SDHCI_CDNS6_PHY_DLL_FIELD_SIZE		256

struct sdhci_cdns6_phy {
	/*
	 * Mode-specific timing constraints (in picoseconds)
	 * These define valid output/input windows per SD/eMMC spec
	 */
	u32 t_cmd_output_min;
	u32 t_cmd_output_max;
	u32 t_dat_output_min;
	u32 t_dat_output_max;
	u32 t_cmd_input_min;
	u32 t_cmd_input_max;
	u32 t_dat_input_min;
	u32 t_dat_input_max;

	/*
	 * PHY delay configuration (in picoseconds)
	 * Derived from clock period and board-level IO cell delays
	 */
	u32 phy_sdclk_delay;
	u32 phy_cmd_o_delay;
	u32 phy_dat_o_delay;
	u32 iocell_input_delay;
	u32 iocell_output_delay;
	u32 delay_element_org;	/* Original delay element from DT */
	u32 delay_element;	/* Current delay element (may be doubled) */

	/* PHY_DLL_SLAVE_CTRL register fields */
	u32 cp_read_dqs_cmd_delay;
	u32 cp_read_dqs_delay;
	u32 cp_clk_wr_delay;
	u32 cp_clk_wrdqs_delay;

	/* PHY_DLL_MASTER_CTRL register fields */
	u32 cp_dll_bypass_mode;
	u32 cp_dll_start_point;

	/* PHY_GATE_LPBK_CTRL register fields */
	u32 cp_gate_cfg_always_on;
	u32 cp_sync_method;
	u32 cp_rd_del_sel;
	u32 cp_sw_half_cycle_shift;
	u32 cp_underrun_suppress;

	/* PHY_DQ_TIMING register fields */
	u32 cp_io_mask_always_on;
	u32 cp_io_mask_end;
	u32 cp_io_mask_start;
	u32 cp_data_select_oe_end;

	/* PHY_DQS_TIMING register fields */
	u32 cp_use_ext_lpbk_dqs;
	u32 cp_use_lpbk_dqs;
	u8 cp_use_phony_dqs;
	u8 cp_use_phony_dqs_cmd;

	/* HRS09 register fields - PHY control and Status */
	u8 sdhc_extended_rd_mode;
	u8 sdhc_extended_wr_mode;
	u32 sdhc_rdcmd_en;
	u32 sdhc_rddata_en;

	/* HRS10 register - SDCLK start point adjustment */
	u32 sdhc_hcsdclkadj;

	/* HRS07 register - IO delay Information */
	u32 sdhc_idelay_val;
	u32 sdhc_rw_compensate;

	/* HRS16 register fields - CMD/DAT output delay control */
	u32 sdhc_wrcmd0_dly;
	u32 sdhc_wrcmd0_sdclk_dly;
	u32 sdhc_wrcmd1_dly;
	u32 sdhc_wrcmd1_sdclk_dly;
	u32 sdhc_wrdata0_dly;
	u32 sdhc_wrdata0_sdclk_dly;
	u32 sdhc_wrdata1_dly;
	u32 sdhc_wrdata1_sdclk_dly;

	/*
	 * DLL calculation intermediate values
	 * Used during PHY timing calculations
	 */
	u32 t_sdmclk_calc;	/* Calculated SDMCLK period for DLL */
	u32 dll_max_value;	/* Max DLL delay value (127/255/256) */

	/* Tuning value for HS200/HS400 modes */
	u32 hs200_tune_val;

	/* Clock periods (in picoseconds) */
	u32 t_sdmclk;		/* Master clock period */
	u32 t_sdclk;		/* SD card clock period */

	/* Current operating state */
	bool strobe_cmd;	/* Enhanced strobe for CMD line */
	int mode;		/* Current MMC_TIMING_* mode */
};

static void init_ds(struct sdhci_cdns6_phy *phy, u32 t_sdclk)
{
	phy->t_cmd_output_min = 5000;
	phy->t_cmd_output_max = t_sdclk - 5000;
	phy->t_dat_output_min = 5000;
	phy->t_dat_output_max = t_sdclk - 5000;
	phy->t_cmd_input_min = t_sdclk / 2 + 14000;
	phy->t_cmd_input_max = t_sdclk + t_sdclk / 2;
	phy->t_dat_input_min = t_sdclk / 2 + 14000;
	phy->t_dat_input_max = t_sdclk + t_sdclk / 2;
}

static void init_hs(struct sdhci_cdns6_phy *phy, u32 t_sdclk)
{
	phy->t_cmd_output_min = 2000;
	phy->t_cmd_output_max = t_sdclk - 6000;
	phy->t_dat_output_min = 2000;
	phy->t_dat_output_max = t_sdclk - 6000;
	phy->t_cmd_input_min = 14000;
	phy->t_cmd_input_max = t_sdclk + 2500;
	phy->t_dat_input_min = 14000;
	phy->t_dat_input_max = t_sdclk + 2500;
}

static void init_uhs_sdr12(struct sdhci_cdns6_phy *phy, u32 t_sdclk)
{
	phy->t_cmd_output_min = 800;
	phy->t_cmd_output_max = t_sdclk - 3000;
	phy->t_dat_output_min = 800;
	phy->t_dat_output_max = t_sdclk - 3000;
	phy->t_cmd_input_min = 14000;
	phy->t_cmd_input_max = t_sdclk + 1500;
	phy->t_dat_input_min = 14000;
	phy->t_dat_input_max = t_sdclk + 1500;
}

static void init_uhs_sdr25(struct sdhci_cdns6_phy *phy, u32 t_sdclk)
{
	phy->t_cmd_output_min = 800;
	phy->t_cmd_output_max = t_sdclk - 3000;
	phy->t_dat_output_min = 800;
	phy->t_dat_output_max = t_sdclk - 3000;
	phy->t_cmd_input_min = 14000;
	phy->t_cmd_input_max = t_sdclk + 1500;
	phy->t_dat_input_min = 14000;
	phy->t_dat_input_max = t_sdclk + 1500;
}

static void init_uhs_sdr50(struct sdhci_cdns6_phy *phy, u32 t_sdclk)
{
	phy->t_cmd_output_min = 800;
	phy->t_cmd_output_max = t_sdclk - 3000;
	phy->t_dat_output_min = 800;
	phy->t_dat_output_max = t_sdclk - 3000;
	phy->t_cmd_input_min = 7500;
	phy->t_cmd_input_max = t_sdclk + 1500;
	phy->t_dat_input_min = 7500;
	phy->t_dat_input_max = t_sdclk + 1500;
}

static void init_uhs_sdr104(struct sdhci_cdns6_phy *phy, u32 t_sdclk)
{
	phy->t_cmd_output_min = 800;
	phy->t_cmd_output_max = t_sdclk - 1400;
	phy->t_dat_output_min = 800;
	phy->t_dat_output_max = t_sdclk - 1400;
	phy->t_cmd_input_min = 1000;
	phy->t_cmd_input_max = t_sdclk + 1000;
	phy->t_dat_input_min = 1000;
	phy->t_dat_input_max = t_sdclk + 1000;
}

static void init_uhs_ddr50(struct sdhci_cdns6_phy *phy, u32 t_sdclk)
{
	phy->t_cmd_output_min = 800;
	phy->t_cmd_output_max = t_sdclk - 3000;
	phy->t_dat_output_min = 800;
	phy->t_dat_output_max = t_sdclk - 3000;
	phy->t_cmd_input_min = 13700;
	phy->t_cmd_input_max = t_sdclk + 1500;
	phy->t_dat_input_min = 7000;
	phy->t_dat_input_max = t_sdclk + 1500;
}

static void init_emmc_sdr(struct sdhci_cdns6_phy *phy, u32 t_sdclk)
{
	phy->t_cmd_output_min = 3000;
	phy->t_cmd_output_max = t_sdclk - 3000;
	phy->t_dat_output_min = 3000;
	phy->t_dat_output_max = t_sdclk - 3000;
	phy->t_cmd_input_min = 13700;
	phy->t_cmd_input_max = t_sdclk + 2500;
	phy->t_dat_input_min = 13700;
	phy->t_dat_input_max = t_sdclk + 2500;
}

static void init_emmc_ddr(struct sdhci_cdns6_phy *phy, u32 t_sdclk)
{
	phy->t_cmd_output_min = 3000;
	phy->t_cmd_output_max = t_sdclk - 3000;
	phy->t_dat_output_min = 2500;
	phy->t_dat_output_max = t_sdclk - 2500;
	phy->t_cmd_input_min = 13700;
	phy->t_cmd_input_max = t_sdclk + 2500;
	phy->t_dat_input_min = 7000;
	phy->t_dat_input_max = t_sdclk + 1500;
}

static void init_emmc_hs200(struct sdhci_cdns6_phy *phy, u32 t_sdclk)
{
	phy->t_cmd_output_min = 800;
	phy->t_cmd_output_max = t_sdclk - 1400;
	phy->t_dat_output_min = 800;
	phy->t_dat_output_max = t_sdclk - 1400;
	phy->t_cmd_input_min = 1000;
	phy->t_cmd_input_max = t_sdclk + 1000;
	phy->t_dat_input_min = 1000;
	phy->t_dat_input_max = t_sdclk + 1000;
}

/* HS400 and HS400ES */
static void init_emmc_hs400(struct sdhci_cdns6_phy *phy, u32 t_sdclk)
{
	phy->t_cmd_output_min = 800;
	phy->t_cmd_output_max = t_sdclk - 1400;
	phy->t_dat_output_min = 400;
	phy->t_dat_output_max = t_sdclk - 400;
	phy->t_cmd_input_min = 1000;
	phy->t_cmd_input_max = t_sdclk + 1000;
	phy->t_dat_input_min = 1000;
	phy->t_dat_input_max = t_sdclk + 1000;
}

static void (*init_timings[])(struct sdhci_cdns6_phy *, u32) = {
	&init_ds, &init_emmc_sdr, &init_hs, &init_uhs_sdr12, &init_uhs_sdr25, &init_uhs_sdr50,
	&init_uhs_sdr104, &init_uhs_ddr50, &init_emmc_ddr, &init_emmc_hs200, &init_emmc_hs400
};

static unsigned int sdhci_cdns6_read_phy_reg(struct sdhci_cdns_priv *priv,
					     const u32 address)
{
	writel(address, priv->hrs_addr + SDHCI_CDNS_HRS04);
	return readl(priv->hrs_addr + SDHCI_CDNS_HRS05);
}

static void sdhci_cdns6_write_phy_reg(struct sdhci_cdns_priv *priv,
				      const u32 address, const u32 value)
{
	writel(address, priv->hrs_addr + SDHCI_CDNS_HRS04);
	writel(value, priv->hrs_addr + SDHCI_CDNS_HRS05);
}

static int sdhci_cdns6_phy_lock_dll(struct sdhci_cdns6_phy *phy)
{
	u32 delay_element = phy->delay_element_org;
	u32 delay_elements_in_sdmclk;

	delay_elements_in_sdmclk = DIV_ROUND_UP(phy->t_sdmclk, delay_element);
	if (delay_elements_in_sdmclk > 256) {
		delay_element *= 2;
		delay_elements_in_sdmclk = DIV_ROUND_UP(phy->t_sdmclk,
							delay_element);

		if (delay_elements_in_sdmclk > 256)
			return -EINVAL;

		phy->dll_max_value = 127;
	} else {
		phy->dll_max_value = 255;
	}

	phy->t_sdmclk_calc = delay_element * delay_elements_in_sdmclk;
	phy->delay_element = delay_element;
	phy->cp_dll_bypass_mode = 0;

	return 0;
}

static void sdhci_cdns6_phy_dll_bypass(struct sdhci_cdns6_phy *phy)
{
	phy->dll_max_value = 256;
	phy->cp_dll_bypass_mode = 1;
}

static void sdhci_cdns6_phy_configure_dll(struct sdhci_cdns6_phy *phy)
{
	if (phy->sdhc_extended_wr_mode == 0) {
		if (sdhci_cdns6_phy_lock_dll(phy) == 0)
			return;
	}
	sdhci_cdns6_phy_dll_bypass(phy);
}

static void sdhci_cdns6_phy_calc_out(struct sdhci_cdns6_phy *phy,
					bool cmd_not_dat)
{
	u32 wr0_dly = 0, wr1_dly = 0, output_min, output_max, phy_o_delay,
	    clk_wr_delay = 0, wr0_sdclk_dly = 0, wr1_sdclk_dly = 0;
	bool ddr = (phy->mode == MMC_TIMING_UHS_DDR50) ||
			(phy->mode == MMC_TIMING_MMC_DDR52) ||
			(phy->mode == MMC_TIMING_MMC_HS400);
	bool data_ddr = ddr && !cmd_not_dat;
	int t;

	if (cmd_not_dat) {
		output_min = phy->t_cmd_output_min;
		output_max = phy->t_cmd_output_max;
		phy_o_delay = phy->phy_cmd_o_delay;
	} else {
		output_min = phy->t_dat_output_min;
		output_max = phy->t_dat_output_max;
		phy_o_delay = phy->phy_dat_o_delay;
	}

	if (data_ddr) {
		wr0_sdclk_dly = 1;
		wr1_sdclk_dly = 1;
	}

	t = phy_o_delay - phy->phy_sdclk_delay - output_min;
	if (t < 0 && phy->sdhc_extended_wr_mode == 1) {
		u32 n_half_cycle = DIV_ROUND_UP(-t * 2, phy->t_sdmclk);

		wr0_dly = (n_half_cycle + 1) / 2;
		if (data_ddr)
			wr1_dly = (n_half_cycle + 1) / 2;
		else
			wr1_dly = (n_half_cycle + 1) % 2 + wr0_dly - 1;
	}

	if (phy->sdhc_extended_wr_mode == 0) {
		u32 out_hold, out_setup, out_hold_margin;
		u32 n;

		if (!data_ddr)
			wr0_dly = 1;

		out_setup = output_max;
		out_hold = output_min;
		out_hold_margin = DIV_ROUND_UP(out_setup - out_hold, 4);
		out_hold += out_hold_margin;

		if (phy->cp_dll_bypass_mode == 0)
			n = DIV_ROUND_UP(256 * out_hold, phy->t_sdmclk_calc);
		else
			n = DIV_ROUND_UP(out_hold, phy->delay_element) - 1;

		if (n <= phy->dll_max_value)
			clk_wr_delay = n;
		else
			/* clk_wr_delay is an 8-bit field, clamp to max value of 255 */
			clk_wr_delay = 255;
	} else {
		/*  sdhc_extended_wr_mode = 1 => PHY IO cell work in SDR mode */
		clk_wr_delay = 0;
	}

	if (cmd_not_dat) {
		phy->sdhc_wrcmd0_dly = wr0_dly;
		phy->sdhc_wrcmd1_dly = wr1_dly;
		phy->cp_clk_wrdqs_delay = clk_wr_delay;
		phy->sdhc_wrcmd0_sdclk_dly = wr0_sdclk_dly;
		phy->sdhc_wrcmd1_sdclk_dly = wr1_sdclk_dly;
	} else {
		phy->sdhc_wrdata0_dly = wr0_dly;
		phy->sdhc_wrdata1_dly = wr1_dly;
		phy->cp_clk_wr_delay = clk_wr_delay;
		phy->sdhc_wrdata0_sdclk_dly = wr0_sdclk_dly;
		phy->sdhc_wrdata1_sdclk_dly = wr1_sdclk_dly;
	}
}

static void sdhci_cdns6_phy_calc_cmd_out(struct sdhci_cdns6_phy *phy)
{
	sdhci_cdns6_phy_calc_out(phy, true);
}

static void sdhci_cdns6_phy_calc_cmd_in(struct sdhci_cdns6_phy *phy)
{
	phy->cp_io_mask_end =
		((phy->iocell_output_delay + phy->iocell_input_delay) * 2)
		/ phy->t_sdmclk;

	/* cp_io_mask_end is a 3-bit field, clamp to max value of 7 */
	if (phy->cp_io_mask_end >= 8)
		phy->cp_io_mask_end = 7;

	if (phy->strobe_cmd && phy->cp_io_mask_end > 0)
		phy->cp_io_mask_end--;

	if (phy->strobe_cmd) {
		phy->cp_use_phony_dqs_cmd = 0;
		phy->cp_read_dqs_cmd_delay = 64;
	} else {
		phy->cp_use_phony_dqs_cmd = 1;
		phy->cp_read_dqs_cmd_delay = 0;
	}

	if ((phy->mode == MMC_TIMING_MMC_HS400 && !phy->strobe_cmd) ||
	    phy->mode == MMC_TIMING_MMC_HS200)
		phy->cp_read_dqs_cmd_delay =
			phy->hs200_tune_val;
}

static void sdhci_cdns6_phy_calc_dat_in(struct sdhci_cdns6_phy *phy)
{
	u32 hcsdclkadj = 0;
	bool strobe_dat = (phy->mode == MMC_TIMING_MMC_HS400);

	if (strobe_dat) {
		phy->cp_use_phony_dqs = 0;
		phy->cp_read_dqs_delay = 64;
	} else {
		phy->cp_use_phony_dqs = 1;
		phy->cp_read_dqs_delay = 0;
	}

	if (phy->mode == MMC_TIMING_MMC_HS200)
		phy->cp_read_dqs_delay =
			phy->hs200_tune_val;

	if (strobe_dat) {
		/* dqs loopback input via IO cell */
		hcsdclkadj += phy->iocell_input_delay;
		/* dfi_dqs_in: mem_dqs -> clean_dqs_mod; delay of hic_dll_dqs_nand2 */
		hcsdclkadj += phy->delay_element / 2;
		/* delay line */
		hcsdclkadj += phy->t_sdclk / 2;
		/* PHY FIFO write pointer */
		hcsdclkadj += phy->t_sdclk / 2 + phy->delay_element;
		/* 1st synchronizer */
		hcsdclkadj += DIV_ROUND_UP(hcsdclkadj, phy->t_sdmclk)
			* phy->t_sdmclk - hcsdclkadj;
		/*
		 * 2nd synchronizer + PHY FIFO read pointer + PHY rddata
		 * + PHY rddata registered, + FIFO 1st ciu_en
		 */
		hcsdclkadj += 5 * phy->t_sdmclk;
		/* FIFO 2nd ciu_en */
		hcsdclkadj += phy->t_sdclk;

		hcsdclkadj /= phy->t_sdclk;
	} else {
		u32 n;

		/* rebar PHY delay */
		hcsdclkadj += 2 * phy->t_sdmclk;
		/* rebar output via IO cell */
		hcsdclkadj += phy->iocell_output_delay;
		/* dqs loopback input via IO cell */
		hcsdclkadj += phy->iocell_input_delay;
		/* dfi_dqs_in: mem_dqs -> clean_dqs_mod delay of hic_dll_dqs_nand2 */
		hcsdclkadj += phy->delay_element / 2;
		/* dll: one delay element between SIGI_0 and SIGO_0 */
		hcsdclkadj += phy->delay_element;
		/* dfi_dqs_in: mem_dqs_delayed -> clk_dqs delay of hic_dll_dqs_nand2 */
		hcsdclkadj += phy->delay_element / 2;
		/* deskew DLL: clk_dqs -> clk_dqN: one delay element */
		hcsdclkadj += phy->delay_element;

		if (phy->t_sdclk == phy->t_sdmclk)
			n = (hcsdclkadj - 2 * phy->t_sdmclk) / phy->t_sdclk;
		else
			n = hcsdclkadj / phy->t_sdclk;

		/* phase shift within one t_sdclk clock cycle caused by rebar - lbk dqs delay */
		hcsdclkadj = hcsdclkadj % phy->t_sdclk;
		/* PHY FIFO write pointer */
		hcsdclkadj += phy->t_sdclk / 2;
		/* 1st synchronizer */
		hcsdclkadj += DIV_ROUND_UP(hcsdclkadj, phy->t_sdmclk)
			* phy->t_sdmclk - hcsdclkadj;
		/*
		 * 2nd synchronizer + PHY FIFO read pointer + PHY rddata
		 * + PHY rddata registered
		 */
		hcsdclkadj += 4 * phy->t_sdmclk;

		if ((phy->t_sdclk / phy->t_sdmclk) > 1) {
			u32 tmp1, tmp2;

			tmp1 = hcsdclkadj;
			tmp2 = (hcsdclkadj / phy->t_sdclk) * phy->t_sdclk
				+ phy->t_sdclk - phy->t_sdmclk;
			if (tmp1 == tmp2)
				tmp2 += phy->t_sdclk;

			/* FIFO aligns to clock cycle before ciu_en */
			hcsdclkadj += tmp2 - tmp1;
		}

		/* FIFO 1st ciu_en */
		hcsdclkadj += phy->t_sdmclk;
		/* FIFO 2nd ciu_en */
		hcsdclkadj += phy->t_sdclk;

		hcsdclkadj /= phy->t_sdclk;

		hcsdclkadj += n;

		if ((phy->t_sdclk / phy->t_sdmclk) >= 2) {
			if (phy->mode == MMC_TIMING_UHS_DDR50 ||
			    phy->mode == MMC_TIMING_MMC_DDR52)
				hcsdclkadj -= 2;
			else
				hcsdclkadj -= 1;
		} else if ((phy->t_sdclk / phy->t_sdmclk) == 1) {
			hcsdclkadj += 2;
		}

		if (phy->mode == MMC_TIMING_UHS_SDR104 || phy->mode == MMC_TIMING_MMC_HS200)
			hcsdclkadj -= 1;
	}

	/* hcsdclkadj is a 4-bit field, clamp to max value of 15 */
	if (hcsdclkadj > 15)
		hcsdclkadj = 15;

	phy->sdhc_hcsdclkadj = hcsdclkadj;
}

static void sdhci_cdns6_phy_calc_dat_out(struct sdhci_cdns6_phy *phy)
{
	sdhci_cdns6_phy_calc_out(phy, false);
}

static void sdhci_cdns6_phy_calc_io(struct sdhci_cdns6_phy *phy)
{
	u32 rw_compensate;

	rw_compensate = ((phy->iocell_input_delay + phy->iocell_output_delay)
		/ phy->t_sdmclk) + phy->sdhc_wrdata0_dly + 5 + 3;

	phy->sdhc_idelay_val = (2 * phy->iocell_input_delay)
		/ phy->t_sdmclk;

	phy->cp_io_mask_start = 0;
	if (phy->t_sdclk == phy->t_sdmclk && rw_compensate > 10)
		phy->cp_io_mask_start = 2 * (rw_compensate - 10);

	if (phy->mode == MMC_TIMING_UHS_SDR104)
		phy->cp_io_mask_start++;

	if (phy->t_sdclk == phy->t_sdmclk && phy->mode == MMC_TIMING_UHS_SDR50)
		phy->cp_io_mask_start++;

	phy->sdhc_rw_compensate = rw_compensate;
}

static void sdhci_cdns6_phy_calc_settings(struct sdhci_cdns6_phy *phy)
{
	sdhci_cdns6_phy_calc_cmd_out(phy);
	sdhci_cdns6_phy_calc_cmd_in(phy);
	sdhci_cdns6_phy_calc_dat_out(phy);
	sdhci_cdns6_phy_calc_dat_in(phy);
	sdhci_cdns6_phy_calc_io(phy);
}

static int sdhci_cdns6_dll_reset(struct sdhci_cdns_priv *priv, bool reset)
{
	u32 reg;
	int ret = 0;

	reg = readl(priv->hrs_addr + SDHCI_CDNS_HRS09);
	if (reset)
		reg &= ~SDHCI_CDNS_HRS09_PHY_SW_RESET;
	else
		reg |= SDHCI_CDNS_HRS09_PHY_SW_RESET;

	writel(reg, priv->hrs_addr + SDHCI_CDNS_HRS09);

	/* After reset, wait until HRS09.PHY_INIT_COMPLETE is set to 1 within 3000us*/
	if (!reset) {
		ret = readl_poll_timeout(priv->hrs_addr + SDHCI_CDNS_HRS09, reg,
					(reg & SDHCI_CDNS_HRS09_PHY_INIT_COMPLETE), 0, 3000);
	}

	return ret;
}

int sdhci_cdns6_phy_init(struct sdhci_cdns_priv *priv)
{
	int ret;
	u32 reg;
	struct sdhci_cdns6_phy *phy = priv->phy;

	sdhci_cdns6_dll_reset(priv, true);

	reg = sdhci_cdns6_read_phy_reg(priv, SDHCI_CDNS6_PHY_DQS_TIMING_REG);
	reg &= ~SDHCI_CDNS6_PHY_DQS_TIMING_USE_EXT_LPBK_DQS;
	reg &= ~SDHCI_CDNS6_PHY_DQS_TIMING_USE_LPBK_DQS;
	reg &= ~SDHCI_CDNS6_PHY_DQS_TIMING_USE_PHONY_DQS;
	reg &= ~SDHCI_CDNS6_PHY_DQS_TIMING_USE_PHONY_DQS_CMD;
	reg |= FIELD_PREP(SDHCI_CDNS6_PHY_DQS_TIMING_USE_EXT_LPBK_DQS,
			phy->cp_use_ext_lpbk_dqs);
	reg |= FIELD_PREP(SDHCI_CDNS6_PHY_DQS_TIMING_USE_LPBK_DQS,
			phy->cp_use_lpbk_dqs);
	reg |= FIELD_PREP(SDHCI_CDNS6_PHY_DQS_TIMING_USE_PHONY_DQS,
			  phy->cp_use_phony_dqs);
	reg |= FIELD_PREP(SDHCI_CDNS6_PHY_DQS_TIMING_USE_PHONY_DQS_CMD,
			  phy->cp_use_phony_dqs_cmd);
	sdhci_cdns6_write_phy_reg(priv, SDHCI_CDNS6_PHY_DQS_TIMING_REG, reg);

	reg = sdhci_cdns6_read_phy_reg(priv, SDHCI_CDNS6_PHY_GATE_LPBK_CTRL_REG);
	reg &= ~SDHCI_CDNS6_PHY_GATE_LPBK_CTRL_SYNC_METHOD;
	reg &= ~SDHCI_CDNS6_PHY_GATE_LPBK_CTRL_SW_HALF_CYCLE_SHIFT;
	reg &= ~SDHCI_CDNS6_PHY_GATE_LPBK_CTRL_RD_DEL_SEL;
	reg &= ~SDHCI_CDNS6_PHY_GATE_LPBK_CTRL_UNDERRUN_SUPPRESS;
	reg &= ~SDHCI_CDNS6_PHY_GATE_LPBK_CTRL_GATE_CFG_ALWAYS_ON;
	reg |= FIELD_PREP(SDHCI_CDNS6_PHY_GATE_LPBK_CTRL_SYNC_METHOD,
			phy->cp_sync_method);
	reg |= FIELD_PREP(SDHCI_CDNS6_PHY_GATE_LPBK_CTRL_SW_HALF_CYCLE_SHIFT,
			phy->cp_sw_half_cycle_shift);
	reg |= FIELD_PREP(SDHCI_CDNS6_PHY_GATE_LPBK_CTRL_RD_DEL_SEL,
			phy->cp_rd_del_sel);
	reg |= FIELD_PREP(SDHCI_CDNS6_PHY_GATE_LPBK_CTRL_UNDERRUN_SUPPRESS,
			phy->cp_underrun_suppress);
	reg |= FIELD_PREP(SDHCI_CDNS6_PHY_GATE_LPBK_CTRL_GATE_CFG_ALWAYS_ON,
			phy->cp_gate_cfg_always_on);
	sdhci_cdns6_write_phy_reg(priv, SDHCI_CDNS6_PHY_GATE_LPBK_CTRL_REG, reg);

	reg = FIELD_PREP(SDHCI_CDNS6_PHY_DLL_MASTER_CTRL_BYPASS_MODE,
			 phy->cp_dll_bypass_mode);
	reg |= FIELD_PREP(SDHCI_CDNS6_PHY_DLL_MASTER_CTRL_PHASE_DETECT_SEL, 2);
	reg |= FIELD_PREP(SDHCI_CDNS6_PHY_DLL_MASTER_CTRL_DLL_LOCK_NUM, 0);
	reg |= FIELD_PREP(SDHCI_CDNS6_PHY_DLL_MASTER_CTRL_DLL_START_POINT,
			phy->cp_dll_start_point);
	sdhci_cdns6_write_phy_reg(priv, SDHCI_CDNS6_PHY_DLL_MASTER_CTRL_REG, reg);

	reg = FIELD_PREP(SDHCI_CDNS6_PHY_DLL_SLAVE_CTRL_READ_DQS_CMD_DELAY,
			 phy->cp_read_dqs_cmd_delay);
	reg |= FIELD_PREP(SDHCI_CDNS6_PHY_DLL_SLAVE_CTRL_CLK_WRDQS_DELAY,
			  phy->cp_clk_wrdqs_delay);
	reg |= FIELD_PREP(SDHCI_CDNS6_PHY_DLL_SLAVE_CTRL_CLK_WR_DELAY,
			  phy->cp_clk_wr_delay);
	reg |= FIELD_PREP(SDHCI_CDNS6_PHY_DLL_SLAVE_CTRL_READ_DQS_DELAY,
			  phy->cp_read_dqs_delay);
	sdhci_cdns6_write_phy_reg(priv, SDHCI_CDNS6_PHY_DLL_SLAVE_CTRL_REG, reg);

	reg = sdhci_cdns6_read_phy_reg(priv, SDHCI_CDNS6_PHY_CTRL_REG);
	reg &= ~SDHCI_CDNS6_PHY_CTRL_PHONY_DQS_TIMING;
	sdhci_cdns6_write_phy_reg(priv, SDHCI_CDNS6_PHY_CTRL_REG, reg);

	smp_wmb(); /* drain writebuffer */

	ret = sdhci_cdns6_dll_reset(priv, false);
	if (ret)
		return ret;

	reg = sdhci_cdns6_read_phy_reg(priv, SDHCI_CDNS6_PHY_DQ_TIMING_REG);
	reg &= ~SDHCI_CDNS6_PHY_DQ_TIMING_IO_MASK_ALWAYS_ON;
	reg &= ~SDHCI_CDNS6_PHY_DQ_TIMING_IO_MASK_END;
	reg &= ~SDHCI_CDNS6_PHY_DQ_TIMING_IO_MASK_START;
	reg &= ~SDHCI_CDNS6_PHY_DQ_TIMING_DATA_SELECT_OE_END;
	reg |= FIELD_PREP(SDHCI_CDNS6_PHY_DQ_TIMING_IO_MASK_ALWAYS_ON,
			phy->cp_io_mask_always_on);
	reg |= FIELD_PREP(SDHCI_CDNS6_PHY_DQ_TIMING_IO_MASK_END,
			  phy->cp_io_mask_end);
	reg |= FIELD_PREP(SDHCI_CDNS6_PHY_DQ_TIMING_IO_MASK_START,
			  phy->cp_io_mask_start);
	reg |= FIELD_PREP(SDHCI_CDNS6_PHY_DQ_TIMING_DATA_SELECT_OE_END,
			phy->cp_data_select_oe_end);
	sdhci_cdns6_write_phy_reg(priv, SDHCI_CDNS6_PHY_DQ_TIMING_REG, reg);

	smp_wmb(); /* drain writebuffer */

	reg = readl(priv->hrs_addr + SDHCI_CDNS_HRS09);
	if (phy->sdhc_extended_wr_mode)
		reg |= SDHCI_CDNS_HRS09_EXTENDED_WR_MODE;
	else
		reg &= ~SDHCI_CDNS_HRS09_EXTENDED_WR_MODE;

	if (phy->sdhc_extended_rd_mode)
		reg |= SDHCI_CDNS_HRS09_EXTENDED_RD_MODE;
	else
		reg &= ~SDHCI_CDNS_HRS09_EXTENDED_RD_MODE;

	if (phy->sdhc_rddata_en)
		reg |= SDHCI_CDNS_HRS09_RDDATA_EN;
	else
		reg &= ~SDHCI_CDNS_HRS09_RDDATA_EN;

	if (phy->sdhc_rdcmd_en)
		reg |= SDHCI_CDNS_HRS09_RDCMD_EN;
	else
		reg &= ~SDHCI_CDNS_HRS09_RDCMD_EN;

	writel(reg, priv->hrs_addr + SDHCI_CDNS_HRS09);

	reg = FIELD_PREP(SDHCI_CDNS_HRS10_HCSDCLKADJ, phy->sdhc_hcsdclkadj);
	writel(reg, priv->hrs_addr + SDHCI_CDNS_HRS10);

	reg = FIELD_PREP(SDHCI_CDNS_HRS16_WRDATA1_SDCLK_DLY,
				phy->sdhc_wrdata1_sdclk_dly);
	reg |= FIELD_PREP(SDHCI_CDNS_HRS16_WRDATA0_SDCLK_DLY,
				phy->sdhc_wrdata0_sdclk_dly);
	reg |= FIELD_PREP(SDHCI_CDNS_HRS16_WRCMD1_SDCLK_DLY,
				phy->sdhc_wrcmd1_sdclk_dly);
	reg |= FIELD_PREP(SDHCI_CDNS_HRS16_WRCMD0_SDCLK_DLY,
				phy->sdhc_wrcmd0_sdclk_dly);
	reg |= FIELD_PREP(SDHCI_CDNS_HRS16_WRDATA1_DLY,
				phy->sdhc_wrdata1_dly);
	reg |= FIELD_PREP(SDHCI_CDNS_HRS16_WRDATA0_DLY,
				phy->sdhc_wrdata0_dly);
	reg |= FIELD_PREP(SDHCI_CDNS_HRS16_WRCMD1_DLY,
				phy->sdhc_wrcmd1_dly);
	reg |= FIELD_PREP(SDHCI_CDNS_HRS16_WRCMD0_DLY,
				phy->sdhc_wrcmd0_dly);
	writel(reg, priv->hrs_addr + SDHCI_CDNS_HRS16);

	reg = FIELD_PREP(SDHCI_CDNS_HRS07_RW_COMPENSATE,
			 phy->sdhc_rw_compensate);
	reg |= FIELD_PREP(SDHCI_CDNS_HRS07_IDELAY_VAL,
			 phy->sdhc_idelay_val);
	writel(reg, priv->hrs_addr + SDHCI_CDNS_HRS07);

	smp_wmb(); /* drain writebuffer */

	/* Allow 5ms for clock and PHY signals to stabilize after configuration */
	mdelay(5);

	return 0;
}

int sdhci_cdns6_set_tune_val(struct sdhci_host *host,
				       unsigned int val)
{
	struct sdhci_cdns_priv *priv = sdhci_cdns_get_priv(host);
	struct sdhci_cdns6_phy *phy = priv->phy;
	u32 tuneval;

	tuneval = (val * SDHCI_CDNS6_PHY_DLL_FIELD_SIZE) /
		  SDHCI_CDNS_MAX_TUNING_LOOP;

	phy->hs200_tune_val = tuneval;
	phy->cp_read_dqs_cmd_delay = tuneval;
	phy->cp_read_dqs_delay = tuneval;

	return sdhci_cdns6_phy_init(priv);
}

static int sdhci_cdns6_phy_update_timings(struct sdhci_host *host)
{
	struct sdhci_cdns_priv *priv = sdhci_cdns_get_priv(host);
	struct sdhci_cdns6_phy *phy = priv->phy;
	u32 t_sdmclk = phy->t_sdmclk;

	/* Validate mode is within supported range */
	if (phy->mode < 0 || phy->mode >= ARRAY_SIZE(init_timings))
		return -EINVAL;

	/* initialize input */
	init_timings[phy->mode](phy, phy->t_sdclk);

	phy->strobe_cmd = false;

	if (priv->enhanced_strobe)
		phy->strobe_cmd = true;

	phy->phy_sdclk_delay = 2 * t_sdmclk;

	/*
	 * CMD and DAT output delays are currently identical, but kept separate
	 * to allow independent tuning for specific modes (e.g., HS400) or
	 * board-specific optimizations in the future.
	 */
	phy->phy_cmd_o_delay = 2 * t_sdmclk + t_sdmclk / 2;
	phy->phy_dat_o_delay = 2 * t_sdmclk + t_sdmclk / 2;

	if (phy->t_sdclk == phy->t_sdmclk) {
		phy->sdhc_extended_wr_mode = 0;
		phy->sdhc_extended_rd_mode = 0;
	} else {
		phy->sdhc_extended_wr_mode = 1;
		phy->sdhc_extended_rd_mode = 1;
	}

	phy->cp_gate_cfg_always_on = 1;

	sdhci_cdns6_phy_configure_dll(phy);

	sdhci_cdns6_phy_calc_settings(phy);

	return 0;
}

int sdhci_cdns6_phy_probe(struct platform_device *pdev,
				    struct sdhci_cdns_priv *priv)
{
	struct device *dev = &pdev->dev;
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_cdns6_phy *phy;
	unsigned long val;
	int ret;

	phy = devm_kzalloc(dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	val = clk_get_rate(pltfm_host->clk);
	phy->t_sdmclk = DIV_ROUND_DOWN_ULL(1000000000000ULL, val);

	ret = of_property_read_u32(dev->of_node, "cdns,iocell-input-delay",
				   &phy->iocell_input_delay);
	if (ret)
		phy->iocell_input_delay = SDHCI_CDNS6_PHY_DEFAULT_IOCELL_DELAY;

	ret = of_property_read_u32(dev->of_node, "cdns,iocell-output-delay",
				   &phy->iocell_output_delay);
	if (ret)
		phy->iocell_output_delay = SDHCI_CDNS6_PHY_DEFAULT_IOCELL_DELAY;

	ret = of_property_read_u32(dev->of_node, "cdns,delay-element",
				   &phy->delay_element);
	if (ret)
		phy->delay_element = SDHCI_CDNS6_PHY_DEFAULT_DELAY_ELEMENT;

	phy->delay_element_org = phy->delay_element;

	priv->phy = phy;

	/* default settings */
	phy->sdhc_rdcmd_en = 1;
	phy->sdhc_rddata_en = 1;
	phy->cp_use_ext_lpbk_dqs = 1;
	phy->cp_use_lpbk_dqs = 1;
	phy->cp_sync_method = 1;
	phy->cp_rd_del_sel = SDHCI_CDNS6_PHY_DEFAULT_RD_DEL_SEL;
	phy->cp_dll_start_point = SDHCI_CDNS6_PHY_DEFAULT_DLL_START;
	phy->cp_data_select_oe_end = 1;
	phy->cp_io_mask_always_on = 0;
	phy->cp_underrun_suppress = 1;

	return 0;
}

void sdhci_cdns6_set_uhs_signaling(struct sdhci_host *host, unsigned int timing)
{
	struct sdhci_cdns_priv *priv = sdhci_cdns_get_priv(host);
	struct sdhci_cdns6_phy *phy = priv->phy;

	phy->t_sdclk = DIV_ROUND_DOWN_ULL(1000000000000ULL, host->mmc->ios.clock);
	phy->mode = timing;

	if (sdhci_cdns6_phy_update_timings(host))
		dev_dbg(mmc_dev(host->mmc), "%s: update timings failed\n", __func__);

	if (sdhci_cdns6_phy_init(priv))
		dev_dbg(mmc_dev(host->mmc), "%s: phy init failed\n", __func__);
}

void sdhci_cdns6_hw_reset(struct sdhci_host *host)
{
	struct sdhci_cdns_priv *priv = sdhci_cdns_get_priv(host);
	void __iomem *reg;

	reg = priv->hrs_addr + SDHCI_CDNS_HRS11;
	writel(SDHCI_CDNS_HRS11_EMMC_RST, reg);
	udelay(9);
	writel(!SDHCI_CDNS_HRS11_EMMC_RST, reg);
	usleep_range(300, 1000);
}
