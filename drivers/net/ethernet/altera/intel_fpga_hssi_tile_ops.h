/* SPDX-License-Identifier: GPL-2.0 */
/* Altera FPGA HSSI callback interface
 * Copyright (C) 2024, 2025 Altera Corporation. All rights reserved
 *
 * Contributors:
 *   Preetam Narayan
 *
 */
#ifndef __HSSI_TILE_OPS_H__
#define __HSSI_TILE_OPS_H__

struct hssi_gen_ops {
	int (*probe_init)(struct platform_device *pdev);
	int (*read_mac_stat)(struct platform_device *pdev, u32 cmd,
			     void *priv_data);
	int (*get_set_csr)(struct platform_device *pdev, enum hssiss_salcmd cmd, void *data,
			   bool get);
	int (*enable_disable_loopback)(struct platform_device *pdev, enum hssiss_salcmd cmd,
				       void *data);
	int (*get_fw_version)(struct platform_device *pdev, enum hssiss_salcmd cmd,
			      void *data);
	int (*get_ncsi_link_status)(struct platform_device *pdev, enum hssiss_salcmd cmd,
				    void *data);
	int (*reset_mac_stat)(struct platform_device *pdev, enum hssiss_salcmd cmd,
			      void *data);
	int (*get_set_dr_profile)(struct platform_device *pdev, enum hssiss_salcmd cmd,
				  void *data, bool get);
	int (*test_nios)(struct platform_device *pdev, enum hssiss_salcmd cmd);
	int (*get_mtu)(struct platform_device *pdev, enum hssiss_salcmd cmd,
		       void *priv_data);
	int (*set_mtu)(struct platform_device *pdev, enum hssiss_salcmd, void *data);
	hssi_eth_port_sts(*get_ethport_status)(struct platform_device *pdev, int port);
	int (*set_ethport_status)(struct platform_device *pdev, int port, u32 data);

	hssi_eth_port_attr(*get_ethport_attr)(struct platform_device *pdev, int port);
	void (*hotplug_enable)(struct platform_device *pdev, bool enable);
	int (*perform_cold_rst)(struct platform_device *pdev);
	int (*hotplug_disable_status)(struct platform_device *pdev);
	int (*lock_mac_stats)(struct platform_device *pdev, int port);
	int (*unlock_mac_stats)(struct platform_device *pdev, int port);
	void (*reset_port)(struct platform_device *pdev, int port);
};

struct hssi_dev_ops {
	void (*probe_init)(struct platform_device *pdev);
	int (*read_mac_stat)(struct platform_device *pdev, struct read_mac_stat_data *priv_data);
	int (*get_mtu)(struct platform_device *pdev, void *data);
	int (*set_mtu)(struct platform_device *pdev, void *data);
	u32 (*get_addr_offset)(struct platform_device *pdev, u8 chan,
			       enum hssiss_tile_regbank regbank, u32 offs);
	hssi_eth_port_sts(*get_ethport_status)(struct platform_device *pdev, int port);
	int (*reset_mac_stat)(struct platform_device *pdev, int port);
	int (*enable_loopback)(struct platform_device *pdev,
			       enum hssiss_loopback_type, int port);
	int (*disable_loopback)(struct platform_device *pdev,
				enum hssiss_loopback_type type, int port);
	int (*freeze_mac_stats)(struct platform_device *pdev, int port);
	int (*defreeze_mac_stats)(struct platform_device *pdev, int port);
	void (*reset_port)(struct platform_device *pdev, int port);
};

struct hssi_spec_ops {
	struct hssi_gen_ops *ops;
	struct hssi_dev_ops *dev_ops;
};

#endif
