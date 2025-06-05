/* SPDX-License-Identifier: GPL-2.0 */
/* Altera FPGA Etile specific driver header
 * Copyright (C) 2022, 2025 Altera Corporation. All rights reserved
 *
 * Contributors:
 *   Preetam Narayan
 */

#ifndef __INTEL_FPGA_ETILE_DRIVER_H__
#define __INTEL_FPGA_ETILE_DRIVER_H__

int etile_ehip_reset(intel_fpga_xtile_eth_private *priv,
		     bool tx_reset, bool rx_reset, bool sys_reset);
int etile_ehip_deassert_reset(intel_fpga_xtile_eth_private *priv);
int etile_init(intel_fpga_xtile_eth_private *priv);
int etile_uninit(intel_fpga_xtile_eth_private *priv);
int etile_start(intel_fpga_xtile_eth_private *priv);
int etile_stop(intel_fpga_xtile_eth_private *priv);
int etile_run_check(intel_fpga_xtile_eth_private *priv);
void etile_update_mac_addr(intel_fpga_xtile_eth_private *priv);
bool etile_get_link_fault_status(intel_fpga_xtile_eth_private *priv);
void intel_fpga_etile_set_ethtool_ops(struct net_device *dev);
void etile_get_stats64(struct net_device *dev,
		       struct rtnl_link_stats64 *storage);
int etile_check_counter_complete(intel_fpga_xtile_eth_private *priv, u32 regbank,
				 size_t offs, u8 bit_mask, bool set_bit, int align);
#endif
