// SPDX-License-Identifier: GPL-2.0
/* Intel FPGA HSSI callback interface
 * Copyright (C) 2024 Intel Corporation. All rights reserved
 *
 * Contributors:
 *   Preetam Narayan
 *
 */
#ifndef __HSSI_TILE_OPS_H__
#define __HSSI_TILE_OPS_H__

struct hssi_tile_ops {
	void (*probe_init)(struct platform_device* pdev);
	int (*read_mac_stat)(struct platform_device *pdev, u32 cmd,
			     void *priv_data, bool atomic);
	int (*get_set_csr)(struct platform_device *pdev, enum hssiss_salcmd cmd, void *data,
			bool get, bool atomic);
	int (*enable_disable_loopback)(struct platform_device *pdev, enum hssiss_salcmd cmd,
                            void *data, bool atomic);
	int (*get_fw_version)(struct platform_device *pdev, enum hssiss_salcmd cmd,
                   void *data, bool atomic);
	int (*get_ncsi_link_status)(struct platform_device *pdev, enum hssiss_salcmd cmd,
                     	     void *data, bool atomic);
	int (*reset_mac_stat)(struct platform_device *pdev, enum hssiss_salcmd cmd,
                   void *data, bool atomic);
	int (*get_set_dr_profile)(struct platform_device *pdev, enum hssiss_salcmd cmd,
                       void *data, bool get, bool atomic);
	int (*test_nios)(struct platform_device *pdev, enum hssiss_salcmd cmd, bool atomic);
	int (*get_mtu)(struct platform_device *pdev, u32 cmd,
                          void *priv_data, bool atomic);
	hssi_eth_port_sts (*get_ethport_status)(struct platform_device *pdev, int port);
	int (*set_ethport_status)(struct platform_device *pdev, int port, u32 data);
	hssi_eth_port_attr (*get_ethport_attr)(struct platform_device *pdev, int port);
	void (*hotplug_enable)(struct platform_device *pdev, bool enable);
	int (*perform_cold_rst)(struct platform_device *pdev);
	int (*hotplug_disable_status)(struct platform_device *pdev);
};

struct hssi_spec_ops {
	enum hssiss_hip_type tile_type;
	struct hssi_tile_ops *tile;
};

#endif
