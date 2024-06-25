/* SPDX-License-Identifier: GPL-2.0 */
/* Intel FPGA HSSI SS interface driver
 * Copyright (C) 2022, 2024 Intel Corporation. All rights reserved
 *
 * Contributors:
 *   Subhransu S. Prusty
 *   Preetam Narayan
 */

#ifndef __INTEL_FPGA_HSSISS_H__
#define __INTEL_FPGA_HSSISS_H__

#include <linux/io.h>

enum hssiss_salcmd {
	SAL_NOP,
	SAL_GET_HSSI_PROFILE,
	SAL_SET_HSSI_PROFILE,
	SAL_READ_MAC_STAT,
	SAL_GET_MTU,
	SAL_SET_CSR,
	SAL_GET_CSR,
	SAL_ENABLE_LOOPBACK,
	SAL_DISABLE_LOOPBACK,
	SAL_RESET_MAC_STAT,
	SAL_RSVD,
	SAL_NCSI_GET_LINK_STS,
	SAL_FW_VERSION,
};

enum hssiss_mac_stat_counter_type {
	MACSTAT_TX_PACKETS,
	MACSTAT_RX_PACKETS,
	MACSTAT_RX_CRC_ERRORS,
	MACSTAT_RX_ALIGN_ERRORS,
	MACSTAT_TX_BYTES,
	MACSTAT_RX_BYTES,
	MACSTAT_TX_PAUSE,
	MACSTAT_RX_PAUSE,
	MACSTAT_RX_ERRORS,
	MACSTAT_TX_ERRORS,
	MACSTAT_RX_UNICAST,
	MACSTAT_RX_MULTICAST,
	MACSTAT_RX_BROADCAST,
	MACSTAT_TX_DISCARDS,
	MACSTAT_TX_UNICAST,
	MACSTAT_TX_MULTICAST,
	MACSTAT_TX_BROADCAST,
	MACSTAT_ETHER_DROPS,
	MACSTAT_RX_TOTAL_BYTES,
	MACSTAT_RX_TOTAL_PACKETS,
	MACSTAT_RX_UNDERSIZE,
	MACSTAT_RX_OVERSIZE,
	MACSTAT_RX_64_BYTES,
	MACSTAT_RX_65_127_BYTES,
	MACSTAT_RX_128_255_BYTES,
	MACSTAT_RX_256_511_BYTES,
	MACSTAT_RX_512_1023_BYTES,
	MACSTAT_RX_1024_1518_BYTES,
	MACSTAT_RX_GTE_1519_BYTES,
	MACSTAT_RX_JABBERS,
	MACSTAT_RX_RUNTS,
};

struct hssiss_salcmd_to_name {
	enum hssiss_salcmd cmd;
	u32 cmdid;
	char name[80];
};

/* data for get/set DR profile */
struct get_set_dr_data {
	u32 dr_grp;
	u32 profile;
	unsigned int port;
};

/* data for reset_mac_stat */
struct reset_mac_stat_data {
	u32 port;
	bool tx;
	bool rx;
};

/* data for read_mac_stat */
struct read_mac_stat_data {
	u32 port_data;
	enum hssiss_mac_stat_counter_type type;
	bool lsb;
};

union ncsi_link_status_data {
	struct {
		u32 link_flag:1;
		u32 speed_duplex:4;
		u32 auto_neg_flag:1;
		u32 auto_neg_complete:1;
		u32 parallel_det_flag;
		u32 res:1;
		u32 link_partner_advert_speed_duplex_100TFD:1;
		u32 link_partner_advert_speed_duplex_100THD:1;
		u32 link_partner_advert_speed_duplex_100T4:1;
		u32 link_partner_advert_speed_duplex_100TXFD:1;
		u32 link_partner_advert_speed_duplex_100TXHD:1;
		u32 link_partner_advert_speed_duplex_10TFD:1;
		u32 link_partner_advert_speed_duplex_10THD:1;
		u32 tx_flow_ctrl:1;
		u32 rx_flow_ctrl:1;
		u32 link_partner_advert_flow_ctrl:2;
		u32 serdes_link:1;
		u32 oem_link_speed_valid:1;
	} part;
	u32 full;		/* used for both port number and return data */
};

typedef union eth_port_status {
	struct {
		u32 o_ehip_ready:1;
		u32 o_rx_hi_ber:1;
		u32 o_cdr_lock:1;
		u32 rx_am_lock:1;
		u32 rx_block_lock:1;
		u32 link_fault_gen_en:1;
		u32 unidirectional_en:1;
		u32 local_fault_status:1;
		u32 remote_fault_status:1;
		u32 unidirectional_force_remote_faul:1;
		u32 unidirectional_remote_fault_dis:1;
		u32 pcs_eccstatus:2;
		u32 mac_eccstatus:2;
		u32 set_10:1;
		u32 set_1000:1;
		u32 ena_10:1;
		u32 eth_mode:1;
		u32 load_recipe_error:1;
		u32 ical_pcal_errors:1;
		u32 tx_lanes_stable:1;
		u32 rx_pcs_ready:1;
		u32 tx_pll_locked:1;
		u32 ptp_tx_pll_locked:1;
		u32 reserved:5;
	} part;
	u32 full;
} hssi_eth_port_sts;

typedef union eth_port_attr {
	struct {
		u32 profile:6;
		u32 rdy_latency:4;
		u32 data_bus_width:3;
		u32 low_speed_eth:2;
		u32 dr_en:1;
		u32 sub_profile:5;
		u32 rsfec_en:1;
		u32 anlt_en:1;
		u32 ptp_en:1;
		u32 reserved:8;
	} part;
	u32 full;
} hssi_eth_port_attr;

enum hssiss_hip_type {
	HSSISS_ETILE = 1,
	HSSISS_FTILE = 2,
	HSSISS_TILE_INDEPENDENT = 3,
};

enum hssi_port_profile {
	HSSI_PORT_PROFILE_10GBE = 20,
	HSSI_PORT_PROFILE_25GBE,
	HSSI_PORT_PROFILE_40GCAUI_4,
	HSSI_PORT_PROFILE_50GAUI_2,
	HSSI_PORT_PROFILE_50GAUI_1,
	HSSI_PORT_PROFILE_100GAUI_1,
	HSSI_PORT_PROFILE_100GAUI_2,
	HSSI_PORT_PROFILE_100GCAUI_4,
	HSSI_PORT_PROFILE_200GAUI_2,
	HSSI_PORT_PROFILE_200GAUI_4,
	HSSI_PORT_PROFILE_200GAUI_8,
	HSSI_PORT_PROFILE_400GAUI_4,
	HSSI_PORT_PROFILE_400GAUI_8,
};

enum hssiss_tile_regbank {
	HSSI_ETH_RECONFIG,
	HSSI_RSFEC,
	HSSI_PHY_XCVR_PMACAP,
	HSSI_PHY_XCVR_PMAAVMM,
	HSSI_SOFTIP,
	HSSI_PTP_PACKET_CLASSIFIER,
	HSSI_RSVD,
};

/* data for get/set csr
 * @offs: To hold address offset,
 */
struct get_set_csr_data {
	u32 offs;
	u32 data;	/* wr for set_csr, read for get_csr */
	unsigned int ch;
	enum hssiss_tile_regbank reg_type;
	bool word;
};

struct get_mtu_data {
	unsigned int port;
	u16 max_tx_frame_size;
	u16 max_rx_frame_size;
};

union hssiss_feature_list {
	struct {
		u32 axi4_support:1;
		u32 num_ports:5;
		u32 port_enable_mask:16;
		u32 res:12;
	} part;
	u32 full;
};

union hssiss_cmd_sts {
	struct {
		u32 rdcmd:1;
		u32 wrcmd:1;
		u32 ack:1;
		u32 busy:1;
		u32 err:1;
		u32 regoffset:2;		//Applicable only on e-tile
		u32 rsvd:25;
	} part;
	u32 full;
};

/* misc bits in ctrl_addr:
 *   for get_csr, set_csr
 *	address bits[23:8]
 *   for read_MAC_statistic
 *	[20:16] - Counters
 *	[30:21] - Reserved
 *	[31:31] - LSB
 *   for reset_MAC_statistic
 *	[16:16] - TX
 *	[17:17] - RX
 *	[31:18] - Reserved
 */
union hssiss_ctrl_addr {
	struct {
		u32 salcmd:8;
		u32 port_addr:4;
		u32 ch_addr:4;
		u32 misc:16;
	} part;
	u32 full;
};

struct hssiss_csr_v5_only {
	u32 dfh_lo;				//0x0
	u32 dfh_hi;				//0x4
	u32 feature_guid_lo_lsb;		//0x8
	u32 feature_guid_lo_msb;		//0xc
	u32 feature_guid_hi_lsb;		//0x10
	u32 feature_guid_hi_msb;		//0x14
	u32 feature_csr_addr_lsb;		//0x18
	u32 feature_csr_addr_msb;		//0x1c
	u32 feature_csr_size_lsb;		//0x20
	u32 feature_csr_size_msb;		//0x24
};

#define feature_offs(x) (offsetof(struct hssiss_csr_v5_only, x))

struct cold_reset_register {
	u32 ofs;
	u32 rst_bit;
	u32 rst_ack;
};

struct hssiss_sysfs_data {
	u32 regdata;
};

struct hssiss_dbg;
struct hssiss_private {
	struct device *dev;

	/* HSSI SS CSR address space */
	void __iomem *sscsr;

	struct hssi_spec_ops *spec_ops;
	/* private data */
	unsigned int dfh_feature_rev;
	unsigned int ver; /* 1: etile, 2: ftile */
	unsigned int csr_addroff;
	union hssiss_feature_list feature_list;

	struct mutex sal_mutex;
	struct mutex coldrst_mutex;
	atomic_t coldrst_inprogress; /* checks if the cold reset has been performed */
	spinlock_t sal_spinlock; /* spinlock used for the sal command access */
	struct hssiss_sysfs_data sysfs;
	struct cold_reset_register cold_rst_reg;
	int hssi_err_wa;
#ifdef CONFIG_DEBUG_FS
	struct hssiss_dbg *dbgfs;
#endif
};

int hssiss_execute_sal_cmd(struct platform_device *pdev,
			   enum hssiss_salcmd cmd, void *data);
int hssiss_execute_sal_cmd_atomic(struct platform_device *pdev,
				  enum hssiss_salcmd cmd, void *data);
void hssiss_hotplug_enable(struct platform_device *pdev, bool enable);
hssi_eth_port_attr hssiss_get_ethport_attr(struct platform_device *pdev, int port);
hssi_eth_port_sts hssiss_get_ethport_status(struct platform_device *pdev, int port);
int hssidrv_set_ethport_status(struct platform_device *pdev, int port, u32 data);
enum hssiss_hip_type hssiss_get_hip_type(struct platform_device *pdev);
hssi_eth_port_sts get_ethport_status(struct platform_device *pdev, int port);
int set_ethport_status(struct platform_device *pdev, int port, u32 data);
int hssiss_cold_rst(struct platform_device *pdev);

#ifdef CONFIG_DEBUG_FS
struct hssiss_dbg *hssiss_dbgfs_init(struct platform_device *pdev);
void hssiss_dbgfs_remove(struct hssiss_dbg *d);
#endif /* CONFIG_DEBUG_FS */

#endif /* __INTEL_FPGA_HSSISS_H__ */

