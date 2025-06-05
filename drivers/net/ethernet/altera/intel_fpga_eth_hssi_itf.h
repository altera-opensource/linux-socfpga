/* SPDX-License-Identifier: GPL-2.0 */
/* Altera FPGA HSSI Interface API
 * Copyright (C) 2022 Altera Corporation. All rights reserved
 *
 * Contributors:
 *   Preetam Narayan
 */

#ifndef __INTEL_FPGA_ETH_HSSI_ITF_H__
#define __INTEL_FPGA_ETH_HSSI_ITF_H__

#include <linux/kernel.h>
#include <linux/platform_device.h>

#include "intel_fpga_hssiss.h"

#define INTEL_FPGA_RET_SUCCESS 0

u32  hssi_csrrd32(struct platform_device *pdev,
		  enum hssiss_tile_regbank regbank,
		  u32 chan,
		  u32 offset);

void hssi_csrwr32(struct platform_device *pdev,
		  enum hssiss_tile_regbank regbank,
		  u32 chan,
		  u32 offset,
		  u32 reg_value);

u32  hssi_csrrd32_ba(struct platform_device *pdev,
		     enum hssiss_tile_regbank regbank,
		  u32 chan,
		  u32 offset);

void hssi_csrwr32_ba(struct platform_device *pdev,
		     enum hssiss_tile_regbank regbank,
		  u32 chan,
		  u32 offset,
		  u32 reg_value);

u8   hssi_csrrd8(struct platform_device *pdev,
		 enum hssiss_tile_regbank regbank,
		 u32 chan,
		 u32 offset);

int  hssi_csrrd8_errcheck(struct platform_device *pdev,
			  enum hssiss_tile_regbank regbank,
			  u32 chan,
			  u32 offset,
			  u8 *reg_value);

void hssi_csrwr8(struct platform_device *pdev,
		 enum hssiss_tile_regbank regbank,
		 u32 chan,
		 u32 offset,
		 u8 reg_value);

void hssi_reset_mac_stats(struct platform_device *pdev,
			  u32 port,
			  bool tx_rst,
			  bool rx_rst);

int hssi_en_serial_loopback(struct platform_device *pdev,
			    enum hssiss_loopback_type type,
			    u32 port);

int hssi_dis_serial_loopback(struct platform_device *pdev,
			     enum hssiss_loopback_type type,
			     u32 port);

bool hssi_ethport_is_stable(struct platform_device *pdev,
			    u32 port, bool logging);

u64  hssi_read_mac_stats64(struct platform_device *pdev,
			   u32 port,
			   enum hssiss_mac_stat_counter_type stat_type);

u64  hssi_read_mac_stats64_atomic(struct platform_device *pdev,
				  u32 port,
			   enum hssiss_mac_stat_counter_type stat_type);

void hssi_disable_hotplug(struct platform_device *pdev);

void hssi_enable_hotplug(struct platform_device *pdev);

void hssi_set_bit_ba(struct platform_device *pdev,
		     enum hssiss_tile_regbank regbank,
		  u32 chan,
		  u32 offset,
		  u32 bit_mask);

void hssi_set_bit(struct platform_device *pdev,
		  enum hssiss_tile_regbank regbank,
		  u32 chan,
		  u32 offset,
		  u32 bit_mask);

void hssi_clear_bit_ba(struct platform_device *pdev,
		       enum hssiss_tile_regbank regbank,
		    u32 chan,
		    u32 offset,
		    u32 bit_mask);

void hssi_clear_bit(struct platform_device *pdev,
		    enum hssiss_tile_regbank regbank,
		    u32 chan,
		    u32 offset,
		    u32 bit_mask);

bool hssi_bit_is_set_ba(struct platform_device *pdev,
			enum hssiss_tile_regbank regbank,
		     u32 chan,
		     u32 offset,
		     u32 bit_mask);

bool hssi_bit_is_set(struct platform_device *pdev,
		     enum hssiss_tile_regbank regbank,
		     u32 chan,
		     u32 offset,
		     u32 bit_mask);

bool hssi_bit_is_clear_ba(struct platform_device *pdev,
			  enum hssiss_tile_regbank regbank,
		       u32 chan,
		       u32 offset,
		       u32 bit_mask);

bool hssi_bit_is_clear(struct platform_device *pdev,
		       enum hssiss_tile_regbank regbank,
		       u32 chan,
		       u32 offset,
		       u32 bit_mask);

int hssi_lock_mac_stats(struct platform_device *pdev, u32 port);
int hssi_unlock_mac_stats(struct platform_device *pdev, u32 port);
void hssi_reset_port(struct platform_device *pdev, u32 port);
int hssi_set_mtu(struct platform_device *pdev, u32 cmd, void* mtu_data);
int hssi_get_mtu(struct platform_device *pdev, u32 cmd, void* mtu_data);
#endif
