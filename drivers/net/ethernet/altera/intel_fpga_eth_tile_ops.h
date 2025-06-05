/* SPDX-License-Identifier: GPL-2.0 */
/* Altera FPGA ethernet callback interface
 * Copyright (C) 2023, 2025 Altera Corporation. All rights reserved
 *
 * Contributors:
 *   Preetam Narayan
 *
 */

#ifndef __XTILE_SPEC_OPS__
#define __XTILE_SPEC_OPS__

struct xtile_spec_ops {
	const struct altera_dmaops *dma_ops;

	struct {
		int (*reset)(intel_fpga_xtile_eth_private *priv, bool tx, bool rx, bool sys);
		int (*deassert_reset)(intel_fpga_xtile_eth_private *priv);
		int (*init)(intel_fpga_xtile_eth_private *priv);
		int (*uninit)(intel_fpga_xtile_eth_private *priv);
		int (*start)(intel_fpga_xtile_eth_private *priv);
		int (*stop)(intel_fpga_xtile_eth_private *priv);
		int (*run_check)(intel_fpga_xtile_eth_private *priv);
		bool (*link_fault_status)(intel_fpga_xtile_eth_private *priv);
		int (*check_counter_complete)(intel_fpga_xtile_eth_private *priv, u32 regbank,
					      size_t offs, u8 bit_mask, bool set_bit,
					       int align);
		void (*update_mac_addr)(intel_fpga_xtile_eth_private *priv);
		void (*net_stats)(struct net_device *netdev, struct rtnl_link_stats64 *priv);
		void (*reg_ethtool_ops)(struct net_device *netdev);
		bool (*check_dts_param)(intel_fpga_xtile_eth_private *priv);
	} tile;

	bool (*link_check)(intel_fpga_xtile_eth_private *priv);
};

#endif
