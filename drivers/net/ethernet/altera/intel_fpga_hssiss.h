
/* SPDX-License-Identifier: GPL-2.0 */
/* Intel FPGA HSSI SS driver
 * Copyright (C) 2022 Intel Corporation. All rights reserved
 *
 * Contributors:
 *   Subhransu S. Prusty
 */

#ifndef __INTEL_FPGA_HSSISS_H__
#define __INTEL_FPGA_HSSISS_H__

#include <linux/io.h>
/* Registers and macros */

/*
 * csr_offset = value@csr_addr_offset + offset
 * for
 *    v0: csr addr offset = 0
 *    v5: read from feature_csr_addr register
 *
 * inter_attrib_port = 0x10 + X * 4 + CSR_ADDROFF
 * eth_port_sts = 0x68 + X (0x00 .. 0x0F)*4 + CSR_ADDROFF
 */
#define HSSISS_CSR_VER				0x8
#define HSSISS_CSR_COMMON_FEATURE_LIST		0xc
#define HSSISS_CSR_INTER_ATTRIB_PORT		0x10
#define HSSISS_CSR_CMDSTS			0x50
#define HSSISS_CSR_CTRLADDR			0x54
#define HSSISS_CSR_RD_DATA			0x58
#define HSSISS_CSR_WR_DATA			0x5C
#define HSSISS_CSR_GMII_TX_LATENCY		0x60
#define HSSISS_CSR_GMII_RX_LATENCY		0x64
#define HSSISS_CSR_ETH_PORT_STS			0x68
#define HSSISS_CSR_TSE_CTRL			0xa8
#define HSSISS_CSR_DBG_CTRL			0xb0
#define HSSISS_CSR_HOTPLUG_DBG_CTRL		0xb4
#define HSSISS_CSR_HOTPLUG_DBG_STS		0xb8

/* DFH */
#define HSSISS_DFHLO_DFHV0_FEA_REV_MASK		GENMASK(15, 12)
#define HSSISS_DFHLO_DFHV0_FEA_REV_SHIFT	12

/* Command status bits */
#define HSSI_SAL_CMDSTS_RD		BIT(0)
#define HSSI_SAL_CMDSTS_WR		BIT(1)
#define HSSI_SAL_CMDSTS_ACK		BIT(2)
#define HSSI_SAL_CMDSTS_BUSY		BIT(3)
#define HSSI_SAL_CMDSTS_ERR		BIT(4)
#define HSSI_SAL_CMDSTS_REG_OFFS_MASK	GENMASK(6,5)
#define HSSI_SAL_CMDSTS_REG_OFFS_SHIFT	5

/* Control address bits */
#define HSSI_SAL_CTRLADDR_SALCMD		0xFF
#define HSSI_SAL_CTRLADDR_PORT_SHIFT		8
#define HSSI_SAL_CTRLADDR_COUNTER_SHIFT		16
#define HSSI_SAL_CTRLADDR_LSB_SHIFT		31
#define HSSI_SAL_CTRLADDR_ADDRBITS_MASK		0xFFFFF
#define HSSI_SAL_CTRLADDR_ADDRBITS_SHIFT	8
#define HSSI_SAL_CTRLADDR_TX			BIT(16)
#define HSSI_SAL_CTRLADDR_RX			BIT(17)

/* Hotplug dbg ctrl and status */
#define HSSI_HOTPLUG_DBG_STS_DISABLE_SHIFT	4

/* Feature CSR v5 only */
#define HSSISS_FEATURE_CSR_ADDR_MASK		GENMASK(31, 1)
#define HSSISS_FEATURE_CSR_ADDR_SHIFT		1

/* Bit index and mask */
#define HSSI_SAL_RESET_MAC_STAT_TX	BIT(16)
#define HSSI_SAL_RESET_MAC_STAT_RX	BIT(17)

#define DR_GRP_INDEX	4
#define HSSI_DR_GRP_MASK	GENMASK(6, 4)
#define HSSI_DR_PROFILE_MASK	GENMASK(3, 0)

/*
 * Bestcase: 100ns, max: 10ms, driver interval: 10us
 * For DR and enable/disable loopback SAL sequences, the whole operation might
 * take more than 10ms and timeout doesn't apply for these sequences, instead
 * polling method is implemeted where a polling counter is used to poll the DR
 * status and it will exit error when the polling counter expires.
 * <TODO>
 */
#define FW_ACK_POLL_INTERVAL_US		10
#define FW_ACK_POLL_TIMEOUT_US		10000

/* CSR read/write macros */
#define csrrd32_withoffset(base, csroff, offs) csrrd32(base, offs + csroff)
#define csrwr32_withoffset(val, base, csroff, offs) \
		csrwr32(val, base, offs + csroff)

#define MASK(idx, nr) (((1 << nr) - 1) << ((idx + 1) - nr))
#define test_reg_bits(val, idx, numbits) (val & MASK(idx, numbits))
#define clear_reg_bits(val, idx, numbits) (val & ~(MASK(idx, numbits)))

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

enum tile_reg_type {
	HSSI_ETH_RECONFIG,
	HSSI_RSFEC,
	HSSI_PHY_XCVR_PMACAP,
	HSSI_PHY_XCVR_PMAAVMM,
	HSSI_SOFTIP,
	HSSI_PTP_PACKET_CLASSIFIER,
	HSSI_RSVD,
};

/*
 * data for get/set csr
 * @offs: To hold address offset,
 */
struct get_set_csr_data {
	u32 offs;
	u32 data;	/* wr for set_csr, read for get_csr */
	unsigned int ch;
	enum tile_reg_type reg_type;
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

/*
 * misc bits in ctrl_addr:
 *   for get_csr, set_csr
 *   	address bits[23:8]
 *   for read_MAC_statistic
 *   	[20:16] - Counters
 *   	[30:21] - Reserved
 *   	[31:31] - LSB
 *   for reset_MAC_statistic
 *   	[16:16] - TX
 *   	[17:17] - RX
 *   	[31:18] - Reserved
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

struct hssiss_csr_common {
	u32 version;				//0x8 + csr_offset
	union hssiss_feature_list feature_list;	//0xc + csr_offset
	u32 inter_attrib_port;			//0x10 + csr_offset
	union hssiss_cmd_sts cmd;		//0x50 + csr_offset
	union hssiss_ctrl_addr ctrl;		//ctrl and addr:
						//0x54 + csr_offset
	u32 rd_data;				//0x58 + csr_offset
	u32 wr_data;				//0x5c + csr_offset
	u32 gmii_tx_latency;			//0x60 + csr_offset
	u32 gmii_rx_latency;			//0x64 + csr_offset
	u32 eth_port_sts;			//0x68 + csr_offset
	u32 tse_ctrl;				//0xa8 + csr_offset
	u32 dbg_ctrl;				//0xb0 + csr_offset
	u32 hotplug_dbg_ctrl;			//0xb4 + csr_offset
	u32 hotplug_dbg_sts;			//0xb8 + csr_offset
};
#define csr_offs(x) (offsetof(struct hssiss_csr_common, x))

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

	/* private data */
	unsigned int dfh_feature_rev;
	unsigned int csr_addroff;
	union hssiss_feature_list feature_list;

	struct mutex sal_mutex;
	struct mutex coldrst_mutex;
	atomic_t coldrst_inprogress;
	spinlock_t sal_spinlock;
	struct hssiss_sysfs_data sysfs;
	struct cold_reset_register cold_rst_reg;
#ifdef CONFIG_DEBUG_FS
	struct hssiss_dbg *dbgfs;
#endif
};

int hssiss_execute_sal_cmd(struct platform_device *pdev,
		enum hssiss_salcmd cmd, void *data);
int hssiss_execute_sal_cmd_atomic(struct platform_device *pdev,
		enum hssiss_salcmd cmd, void *data);
hssi_eth_port_sts hssiss_get_ethport_status(struct platform_device *pdev, int port);
int hssi_cold_rst(struct platform_device *pdev);

#ifdef CONFIG_DEBUG_FS
struct hssiss_dbg *hssiss_dbgfs_init(struct platform_device *pdev);
void hssiss_dbgfs_remove(struct hssiss_dbg *d);
#endif /* CONFIG_DEBUG_FS */
#endif /* __INTEL_FPGA_HSSISS_H__ */

