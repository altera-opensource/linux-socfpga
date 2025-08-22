// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2025 Altera Corporation
 *   Author: Tanmay Kathpalia <tanmay.kathpalia@altera.com>
 */

#ifndef _MMC_HOST_SDHCI_CADENCE_H
#define _MMC_HOST_SDHCI_CADENCE_H

#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/mmc/mmc.h>
#include "sdhci-pltfm.h"

/* HRS - Host Register Set (specific to Cadence) */
#define SDHCI_CDNS_HRS04		0x10		/* PHY access port */
#define SDHCI_CDNS_HRS05		0x14		/* PHY access port data */

/*
 * The tuned val register is 6 bit-wide, but not the whole of the range is
 * available. The range 0-42 seems to be available (then 43 wraps around to 0)
 * but I am not quite sure if it is official.  Use only 0 to 39 for safety.
 */
#define SDHCI_CDNS_MAX_TUNING_LOOP	40

/**
 * struct sdhci_cdns_drv_data - Cadence SDHCI driver-specific data
 * @init: Pointer to SoC-specific initialization function.
 * @pltfm_data: Platform data.
 *
 * Used for passing implementation or SoC-specific hooks and parameters
 * to the Cadence SDHCI controller driver.
 */
struct sdhci_cdns_drv_data {
	int (*init)(struct platform_device *pdev);
	const struct sdhci_pltfm_data pltfm_data;
};

/**
 * struct sdhci_cdns4_phy_param - PHY parameter/address pair
 * @addr: PHY register address.
 * @data: Value to write to the PHY register.
 *
 * Used for passing a list of PHY configuration parameters to the
 * Cadence SDHCI V4 controller.
 */
struct sdhci_cdns4_phy_param {
	u8 addr;
	u8 data;
};

/**
 * struct sdhci_cdns_priv - Cadence SDHCI private controller data
 * @hrs_addr: Base address of Cadence Host Register Set (HRS) registers.
 * @ctl_addr: Base address for write control registers.
 *            Used only for "amd,pensando-elba-sd4hc" compatible controllers
 *            to enable byte-lane writes.
 * @wrlock: Spinlock for protecting register writes (Elba only).
 * @enhanced_strobe: Flag indicating if Enhanced Strobe (HS400ES) is enabled.
 * @priv_writel: Optional SoC-specific write function for register access.
 *               Used for Elba to ensure correct byte-lane enable.
 * @rst_hw: Hardware reset control for the controller.
 * @ciu_clk: Card Interface Unit (CIU) clock handle.
 *           Used only for V6 (SDHCI spec >= 4.20) controllers.
 * @nr_phy_params: Number of PHY parameter entries parsed from DT (V4 only).
 * @phy_params: Array of PHY parameter/address pairs for PHY initialization (V4 only).
 */
struct sdhci_cdns_priv {
	void __iomem *hrs_addr;
	void __iomem *ctl_addr; /* write control */
	spinlock_t wrlock; /* write lock */
	bool enhanced_strobe;
	void (*priv_writel)(struct sdhci_cdns_priv *priv, u32 val,
			    void __iomem *reg);
	struct reset_control *rst_hw;
	struct clk *ciu_clk; /* Card Interface Unit clock */
	unsigned int nr_phy_params;
	struct sdhci_cdns4_phy_param phy_params[];
};

/*
 * sdhci_cdns_priv - Helper to retrieve Cadence private data from sdhci_host
 * @host: Pointer to struct sdhci_host.
 *
 * Returns: Pointer to struct sdhci_cdns_priv.
 */
static inline void *sdhci_cdns_priv(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	return sdhci_pltfm_priv(pltfm_host);
}

/**
 * sdhci_cdns6_phy_adj - Program PHY registers for a specific timing mode.
 * @host: Pointer to struct sdhci_host.
 * @timing: MMC timing mode (MMC_TIMING_*).
 *
 * Returns 0 on success or a negative error code.
 */
int sdhci_cdns6_phy_adj(struct sdhci_host *host, unsigned char timing);

/**
 * sdhci_cdns6_set_tune_val - Set the PHY tuning value.
 * @host: Pointer to struct sdhci_host.
 * @val: Tuning value to program.
 *
 * Returns 0 on success or a negative error code.
 */
int sdhci_cdns6_set_tune_val(struct sdhci_host *host, unsigned int val);

/**
 * sdhci_cdns6_phy_probe - Initialize the Cadence PHY using device tree.
 * @host: Pointer to struct sdhci_host.
 *
 * Returns 0 on success or a negative error code.
 */
int sdhci_cdns6_phy_probe(struct sdhci_host *host);
/**
 * sdhci_cdns6_hw_reset - Perform hardware reset of the Cadence SDHCI controller.
 * @host: Pointer to struct sdhci_host.
 */
void sdhci_cdns6_hw_reset(struct sdhci_host *host);

#endif /* _MMC_HOST_SDHCI_CADENCE_H */
