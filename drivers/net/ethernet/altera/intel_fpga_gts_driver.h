/* SPDX-License-Identifier: GPL-2.0 */
/* Altera FPGA GTS specific driver header
 * Copyright (C) 2022, 2025 Altera Corporation. All rights reserved
 *
 * Contributors:
 *   Preetam Narayan
 */

 #ifndef __INTEL_FPGA_GTS_DRIVER_H__
 #define __INTEL_FPGA_GTS_DRIVER_H__

void gts_enable_mac(intel_fpga_xtile_eth_private *priv);
void gts_disable_mac(intel_fpga_xtile_eth_private *priv);
int gts_ehip_reset(intel_fpga_xtile_eth_private *priv,
		   bool tx_reset, bool rx_reset, bool sys_reset);
int gts_ehip_deassert_reset(intel_fpga_xtile_eth_private *priv);
int gts_init(intel_fpga_xtile_eth_private *priv);
int gts_uninit(intel_fpga_xtile_eth_private *priv);
int gts_start(intel_fpga_xtile_eth_private *priv);
int gts_stop(intel_fpga_xtile_eth_private *priv);
int gts_run_check(intel_fpga_xtile_eth_private *priv);
void gts_update_mac_addr(intel_fpga_xtile_eth_private *priv);
bool gts_get_link_fault_status(intel_fpga_xtile_eth_private *priv);
void gts_get_stats64(struct net_device *dev,
		     struct rtnl_link_stats64 *storage);
int gts_check_counter_complete(intel_fpga_xtile_eth_private *priv, u32 regbank,
			       size_t offs, u8 bit_mask, bool set_bit, int align);
bool gts_check_dts_param(intel_fpga_xtile_eth_private *priv);
void intel_fpga_gts_set_ethtool_ops(struct net_device *dev);
 #endif
