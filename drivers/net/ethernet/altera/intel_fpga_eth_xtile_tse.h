/* SPDX-License-Identifier: GPL-2.0 */
/* Altera Triple-Speed Ethernet (TSE) variant for the Intel FPGA xtile
 * pluggable Ethernet driver framework.
 *
 * This header defines the variant-private state (intel_fpga_tse_private)
 * stored at priv->intel_fpga_tile_private, and exports the tile ops callbacks
 * consumed by intel_fpga_eth_main.c via the xtile_spec_ops table.
 *
 * Reference hardware: small Triple-Speed Ethernet IP with 1000BASE-X/SGMII PCS
 * connected through SGMII to an external 10/100/1000 copper PHY
 *
 * Copyright (C) 2026 Altera Corporation.
 *
 * Contributors:
 *   Mahesh Vaidya
 *   Krishna Kumar S R
 *   Preetam Narayan
 *   Lubana.Badakar
 */

#ifndef __INTEL_FPGA_ETH_TSE_H__
#define __INTEL_FPGA_ETH_TSE_H__

#include <linux/netdevice.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <linux/platform_device.h>

#include "intel_fpga_eth_main.h"
#include "altera_tse.h"		/* For struct altera_tse_mac, tse_csroffs(), MAC_CMDCFG_* */

#define INTEL_FPGA_TSE_RESOURCE_NAME	"intel_fpga_tse"

/* TSE-specific private data, allocated per device and stored in
 * priv->intel_fpga_tile_private.
 */
struct intel_fpga_tse_private {
	/* MAC register window (memory-mapped Avalon-MM slave for MAC CSRs).
	 * Layout described by struct altera_tse_mac (altera_tse.h).
	 */
	struct altera_tse_mac __iomem	*mac_dev;

	/* PCS register window. Either:
	 *  - A dedicated AVMM slave passed via the "pcs" reg in DT, or
	 *  - Aliased to mac_dev + tse_csroffs(mdio_phy0) (PCS occupies MDIO Space 0
	 *    within the MAC CSR space; cf. TSE User Guide Section 5.2).
	 */
	void __iomem			*pcs_base;
	u32				pcs_reg_width;	/* 4 or 2 */

	/* MAC IP revision (megacore_revision register) */
	u32				revision;

	/* Local MDIO bus for the external PHY. The PCS is NOT on this bus; it
	 * lives on a separate mdio-regmap shim bus over pcs_base (see the pcs
	 * field below).
	 */
	struct mii_bus			*mdio;
	int				phy_addr;	/* -1 => auto-poll */

	/* Phylink PCS handle for the SGMII/1000BASE-X PCS. The Altera TSE PCS is
	 * a memory-mapped instance of the Lynx PCS, so it is driven by the Lynx
	 * PCS driver: a regmap over the PCS window is exposed as an MDIO bus via
	 * mdio-regmap, and lynx_pcs_create_mdiodev() returns this handle.
	 */
	struct phylink_pcs		*pcs;

	/* Multicast hash filter present (DT: altr,has-hash-multicast-filter).
	 * Selects which ndo_set_rx_mode implementation is installed at probe.
	 */
	bool				hash_filter;

	/* PHY-resolved link state, latched by phylink mac_link_up/down callbacks.
	 * Used by tile.link_fault_status to feed the xtile framework's link
	 * monitor (eth_monitor_link_status) so tile.start/tile.stop are sequenced
	 * correctly with the data path.
	 */
	bool				link_up;
};

/* Tile-ops callbacks exported to intel_fpga_eth_main.c */

bool intel_fpga_tse_check_dts_param(intel_fpga_xtile_eth_private *priv);
int  intel_fpga_tse_remove(struct platform_device *pdev);

int  intel_fpga_tse_init(intel_fpga_xtile_eth_private *priv);
int  intel_fpga_tse_uninit(intel_fpga_xtile_eth_private *priv);

int  intel_fpga_tse_start(intel_fpga_xtile_eth_private *priv);
int  intel_fpga_tse_stop(intel_fpga_xtile_eth_private *priv);

int  intel_fpga_tse_reset(intel_fpga_xtile_eth_private *priv,
			  bool tx, bool rx, bool sys);
int  intel_fpga_tse_deassert_reset(intel_fpga_xtile_eth_private *priv);

void intel_fpga_tse_update_mac_addr(intel_fpga_xtile_eth_private *priv);
bool intel_fpga_tse_get_link_fault_status(intel_fpga_xtile_eth_private *priv);

void intel_fpga_tse_dispatch_set_rx_mode(struct net_device *dev);

/* Optional alternate ndo_set_rx_mode implementations (kept exported so the
 * core may install one of them at probe time if the variant wants hash-filter
 * behaviour vs. plain promiscuous-on-multicast).
 */
void intel_fpga_tse_set_rx_mode(struct net_device *dev);
void intel_fpga_tse_set_rx_mode_hashfilter(struct net_device *dev);

/* Ethtool ops registration */
void intel_fpga_tse_set_ethtool_ops(struct net_device *netdev);

#endif /* __INTEL_FPGA_ETH_TSE_H__ */
