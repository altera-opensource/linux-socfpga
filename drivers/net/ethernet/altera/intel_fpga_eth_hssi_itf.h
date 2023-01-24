/* SPDX-License-Identifier: GPL-2.0 */
/* Intel FPGA HSSI Interface API
 * Copyright (C) 2022 Intel Corporation. All rights reserved
 *
 * Contributors:
 *   Preetam Narayan
 */

#ifndef __INTEL_FPGA_ETH_HSSI_ITF_H__
#define __INTEL_FPGA_ETH_HSSI_ITF_H__

#include <linux/kernel.h>
#include <linux/platform_device.h>

#include "intel_fpga_hssiss.h"

enum access_type {
  BYTE_ACCESS,
  WORD_ACCESS
};


#define INTEL_FPGA_RET_SUCCESS 0

u32  hssi_csrrd32(struct platform_device *pdev,
		  enum hssiss_tile_reg_type regbank,
		  u32 chan,
		  u32 offset);

u32  hssi_csrrd32_atomic(struct platform_device *pdev,
			 enum hssiss_tile_reg_type regbank,
			 u32 chan,
			 u32 offset);

int  hssi_csrrd32_errcheck(struct platform_device *pdev,
			   enum hssiss_tile_reg_type regbank,
			   u32 chan,
			   u32 offset,
			   bool atomocity,
			   u32 *ret_value);

void hssi_csrwr32(struct platform_device *pdev,
		  enum hssiss_tile_reg_type regbank,
		  u32 chan,
		  u32 offset,
		  u32 reg_value);

void hssi_csrwr32_atomic(struct platform_device *pdev,
		         enum hssiss_tile_reg_type regbank,
		  	 u32 chan,
		  	 u32 offset,
		  	 u32 reg_value);

u8   hssi_csrrd8(struct platform_device *pdev,
		 enum hssiss_tile_reg_type regbank,
		 u32 chan,
		 u32 offset);

u8   hssi_csrrd8_atomic(struct platform_device *pdev,
			enum hssiss_tile_reg_type regbank,
			u32 chan,
			u32 offset);

int  hssi_csrrd8_errcheck(struct platform_device *pdev,
			  enum hssiss_tile_reg_type regbank,
			  u32 chan,
			  u32 offset,
			  bool atomic,
			  u8 *reg_value);

void hssi_csrwr8(struct platform_device *pdev,
		 enum hssiss_tile_reg_type regbank,
		 u32 chan,
		 u32 offset,
		 u8 reg_value);

void hssi_set_bit(struct platform_device *pdev,
		  enum hssiss_tile_reg_type regbank,
		  u32 chan,
		  u32 offset,
		  u32 bit_mask);

void hssi_clear_bit(struct platform_device *pdev,
		    enum hssiss_tile_reg_type regbank,
		    u32 chan,
		    u32 offset,
		    u32 bit_mask);

void hssi_set_bit_atomic(struct platform_device *pdev,
                  	  enum tile_reg_type regbank,
                  	  u32 chan,
                  	  u32 offset,
                  	  u32 bit_mask);

void hssi_clear_bit_atomic(struct platform_device *pdev,
                    	   enum tile_reg_type regbank,
                    	   u32 chan,
                    	   u32 offset,
                    	   u32 bit_mask);

bool hssi_bit_is_set(struct platform_device *pdev,
		     enum hssiss_tile_reg_type regbank,
		     u32 chan,
		     u32 offset,
		     u32 bit_mask);

bool hssi_bit_is_clear(struct platform_device *pdev,
		       enum hssiss_tile_reg_type regbank,
		       u32 chan,
		       u32 offset,
		       u32 bit_mask);

void hssi_reset_mac_stats(struct platform_device *pdev,
			  u32 chan,
			  bool tx_rst,
			  bool rx_rst);

int hssi_en_serial_loopback(struct platform_device *pdev,
			     u32 chan);

int hssi_dis_serial_loopback(struct platform_device *pdev,
			      u32 chan);

bool hssi_ethport_is_stable(struct platform_device *pdev,
			    u32 chan, bool);

u64  hssi_read_mac_stats64(struct platform_device *pdev,
			   u32 chan,
			   enum hssiss_mac_stat_counter_type stat_type);

void hssi_disable_hotplug(struct platform_device *pdev);

void hssi_enable_hotplug(struct platform_device *pdev);
#endif 
