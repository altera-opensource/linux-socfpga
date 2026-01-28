// SPDX-License-Identifier: GPL-2.0
/* Altera FPGA HSSI glue logic driver interface
 * Copyright (C) 2024, 2025 Altera Corporation. All rights reserved
 *
 * Contributors:
 *   Preetam Narayan
 *
*/

#ifndef __INTEL_FPGA_HSSIGL_DRIVER_H__
#define __INTEL_FPGA_HSSIGL_DRIVER_H__

#define ERR_CNT_RST 6

struct intel_fpga_userspace_reg {
        u32 control_reg;
        u32 error_reg;
        u32 status_reg;
        u32 fifo_status_reg;
        u32 p0_packet_err_count;
        u32 p1_packet_err_count;
        u32 error_ctrl_reg;
};
#define eth_userspace_csroffs(a) (offsetof(struct intel_fpga_userspace_reg, a))

int hssigldrv_probe_init(struct platform_device *pdev);
int hssigldrv_get_set_csr(struct platform_device *pdev, u32 cmd,
			  void *csr_data,
			  bool rd);
hssi_eth_port_sts hssigldrv_get_ethport_status(struct platform_device *pdev,
					       int port);
int hssigldrv_reset_mac_stat(struct platform_device *pdev,
			     enum hssiss_salcmd cmd,
			     void *data);
int hssigldrv_read_mac_stats(struct platform_device *pdev,
			     enum hssiss_salcmd cmd,
			     void *data);
int hssigldrv_get_mtu(struct platform_device *pdev,
		      enum hssiss_salcmd cmd,
		      void *data);
int hssigldrv_set_mtu(struct platform_device *pdev,
		      enum hssiss_salcmd cmd,
		      void *data);
int hssigldrv_enable_disable_loopback(struct platform_device *pdev,
				      u32 cmdid,
				      void *lb_data);
int hssigldrv_lock_mac_stats(struct platform_device *pdev,
			     int port);
int hssigldrv_unlock_mac_stats(struct platform_device *pdev,
			       int port);
void hssigldrv_reset_port(struct platform_device *pdev,
			  int port);
void hssigldrv_err_cnt_reset(struct platform_device *pdev, int port);
u32 hssigldrv_err_cnt_read(struct platform_device *pdev, u32 addr_offs);
void hssigldrv_err_cnt_enable(struct platform_device *pdev, int port, bool enable);

 #endif
