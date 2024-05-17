/* SPDX-License-Identifier: GPL-2.0 */
/* Intel FPGA Ftile specific driver header
 * Copyright (C) 2022-2024 Intel Corporation. All rights reserved
 *
 * Contributors:
 *   Preetam Narayan
 */

#ifndef __INTEL_FPGA_FTILE_DRIVER_H__
#define __INTEL_FPGA_FTILE_DRIVER_H__

int ftile_ehip_reset(intel_fpga_xtile_eth_private *priv,
		     bool tx_reset, bool rx_reset, bool sys_reset);
int ftile_ehip_deassert_reset(intel_fpga_xtile_eth_private *priv);
int ftile_init(intel_fpga_xtile_eth_private *priv);
int ftile_uninit(intel_fpga_xtile_eth_private *priv);
int ftile_start(intel_fpga_xtile_eth_private *priv);
int ftile_stop(intel_fpga_xtile_eth_private *priv);
int ftile_run_check(intel_fpga_xtile_eth_private *priv);
void ftile_update_mac_addr(intel_fpga_xtile_eth_private *priv);
bool ftile_get_link_fault_status(intel_fpga_xtile_eth_private *priv);
void intel_fpga_ftile_set_ethtool_ops(struct net_device *dev);
void ftile_get_stats64(struct net_device *dev,
		       struct rtnl_link_stats64 *storage);
int ftile_check_counter_complete(intel_fpga_xtile_eth_private *priv, u32 regbank,
				 size_t offs, u8 bit_mask, bool set_bit, int align);
bool ftile_check_dts_param(intel_fpga_xtile_eth_private *priv);
#endif
