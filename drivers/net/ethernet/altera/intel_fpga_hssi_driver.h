/* SPDX-License-Identifier: GPL-2.0 */
/* Altera FPGA HSSI SS driver
 * Copyright (C) 2022, 2025 Altera Corporation. All rights reserved
 *
 * Contributors:
 *   Subhransu S. Prusty
 *   Preetam Narayan
 */
 #ifndef __INTEL_FPGA_HSSISS_DRV_H__
 #define __INTEL_FPGA_HSSISS_DRV_H__

 #define HSSISS_FTILE 2

/* Registers and macros */

/* csr_offset = value@csr_addr_offset + offset
 * for
 *    v0: csr addr offset = 0
 *    v5: read from feature_csr_addr register
 *
 * eth_port_sts = 0x68 + X (0x00 .. 0x0F)*4 + CSR_ADDROFF
 */
 #define HSSISS_CSR_VER                          0x8
 #define HSSISS_CSR_COMMON_FEATURE_LIST          0xc
/*Port attr For E-tile: 0x10 + x * 4 + CSR_ADDROFF */
 #define HSSISS_CSR_INTER_ATTRIB_PORT            0x10
 #define HSSISS_CSR_CMDSTS                       0x50
 #define HSSISS_CSR_CTRLADDR                     0x54
 #define HSSISS_CSR_RD_DATA                      0x58
 #define HSSISS_CSR_WR_DATA                      0x5C
 #define HSSISS_CSR_GMII_TX_LATENCY              0x60
 #define HSSISS_CSR_GMII_RX_LATENCY              0x64
 #define HSSISS_CSR_ETH_PORT_STS                 0x68
 #define HSSISS_CSR_TSE_CTRL                     0xa8
 #define HSSISS_CSR_DBG_CTRL                     0xb0
 #define HSSISS_CSR_HOTPLUG_DBG_CTRL             0xb4
 #define HSSISS_CSR_HOTPLUG_DBG_STS              0xb8
 #define HSSISS_CSR_GENERAL_STATUS               0xbc

/*F-tile specific */
 #define HSSISS_CSR_INTER_ATTRIB_PORT_FHT        0x300 /* + x * 4 */
 #define HSSISS_CSR_ETH_PORT_STS_FHT             0x200 /* + x * 4 */

/* Ftile only: PFC offset + x * 4, read returns 0 if port doesn't exit */
 #define HSSISS_CSR_PFC_CTRL                     0x400
 #define HSSISS_CSR_PFC_ERR_STS                  0x450
 #define HSSISS_CSR_PFC_RX_PARSE_CFG             0x4A0

/* Ftile only: ANLT: ANLTx = ANLT_BASE + x * RANGE */
 #define HSSISS_CSR_ANLT_BASE                    0x10000
 #define HSSISS_CSR_ANLT_RANGE                   0x400
// e.g. csrrd32(base, 0x10000 + 0*(0x400) + 0x2C0)
/* FTile : Auto-negotiation */
#define HSSISS_CSR_ANLT_SEQ_CFG			0x2C0
#define HSSISS_CSR_ANLT_SEQ_STATUS		0x2C4
#define HSSISS_CSR_AN_CFG_1			0x300
#define HSSISS_CSR_AN_CFG_2                     0x304
#define HSSISS_CSR_AN_STATUS			0x308
#define HSSISS_CSR_AN_CFG_3                     0x30C
#define HSSISS_CSR_AN_CFG_4                     0x310
#define HSSISS_CSR_AN_CFG_5                     0x314
#define HSSISS_CSR_AN_CFG_6                     0x318
#define HSSISS_CSR_AN_STATUS_NUM 		6
#define HSSISS_CSR_AN_STATUS_1                  0x31C // LP base page lower bits
#define HSSISS_CSR_AN_STATUS_2                  0x320 // LP base page upper bits
#define HSSISS_CSR_AN_STATUS_3                  0x324 // LP next page lower bits
#define HSSISS_CSR_AN_STATUS_4                  0x328 // LP next page upper bits
#define HSSISS_CSR_AN_CFG_8                     0x330
#define HSSISS_CSR_AN_STATUS_6                  0x338 // LP Consortium NP Technology ability
/* FTile : Link Training */
#define HSSISS_CSR_LT_CFG_1			0x340
#define HSSISS_CSR_LT_CFG_2                     0x344
#define HSSISS_CSR_LT_STATUS_1			0x348
/* KR IP(ANLT IP) debug registers */
#define HSSISS_CSR_KR_DEBUG_0			0x3C0
#define HSSISS_CSR_KR_DEBUG_1                   0x3C4
#define HSSISS_CSR_KR_DEBUG_2                   0x3C8
#define HSSISS_CSR_KR_DEBUG_3                   0x3CC
#define HSSISS_CSR_KR_DEBUG_4                   0x3D0
#define HSSISS_CSR_KR_DEBUG_5                   0x3D4
#define HSSISS_CSR_KR_DEBUG_6                   0x3D8
#define HSSISS_CSR_KR_DEBUG_7                   0x3DC
#define HSSISS_CSR_KR_DEBUG_8                   0x3E0
#define HSSISS_CSR_KR_DEBUG_9                   0x3E4
#define HSSISS_CSR_KR_DEBUG_10                  0x3E8
#define HSSISS_CSR_KR_DEBUG_11                  0x3EC
#define HSSISS_CSR_KR_DEBUG_12                  0x3F0
#define HSSISS_CSR_KR_DEBUG_13                  0x3F4
#define HSSISS_CSR_KR_DEBUG_14                  0x3F8
#define HSSISS_CSR_KR_DEBUG_15                  0x3FC

#define HSSISS_CSR_AN_PAGE_RCVD			BIT(1)
#define HSSISS_CSR_AN_COMPLETE			BIT(2)
#define HSSISS_CSR_AN_ADV_REMOTE_FAULT		BIT(3)
#define HSSISS_CSR_PHY_AN_ABILITY		BIT(5)
#define HSSISS_CSR_AN_STATUS_BIT		BIT(6)
#define HSSISS_CSR_AN_LP_ABILITY		BIT(7)
#define HSSISS_CSR_BASER_FEC_NEGOTIATED		BIT(8)
#define HSSISS_CSR_AN_FAILURE			BIT(9)
#define HSSISS_CSR_CONSORTIUM_NEXT_PAGE_RCVD	BIT(10)
#define HSSISS_CSR_NEG_FAILURE			BIT(11)
#define HSSISS_CSR_IEEE_NEG_PORT_10G_KR		BIT(12)
#define HSSISS_CSR_IEEE_NEG_PORT_40G_KR4	BIT(13)
#define HSSISS_CSR_IEEE_NEG_PORT_40G_CR4	BIT(14)
#define HSSISS_CSR_IEEE_NEG_PORT_100G_KR4	BIT(15)
#define HSSISS_CSR_IEEE_NEG_PORT_100G_CR4	BIT(16)
#define HSSISS_CSR_IEEE_NEG_PORT_25G_KR_CR_S	BIT(17)
#define HSSISS_CSR_IEEE_NEG_PORT_25G_KR_CR	BIT(18)
#define HSSISS_CSR_IEEE_NEG_PORT_50G_KR_CR	BIT(19)
#define HSSISS_CSR_IEEE_NEG_PORT_100G_KR2_CR2	BIT(20)
#define HSSISS_CSR_IEEE_NEG_PORT_200G_KR4_CR4	BIT(21)
#define HSSISS_CSR_IEEE_NEG_PORT_100G_KR_CR	BIT(22)
#define HSSISS_CSR_IEEE_NEG_PORT_200G_KR2_CR2	BIT(23)
#define HSSISS_CSR_IEEE_NEG_PORT_MASK		GENMASK(23,12)
#define HSSISS_CSR_IEEE_NEG_PORT_MASK_START	12
#define HSSISS_CSR_CONS_NEG_PORT_25G_KR1	BIT(24)
#define HSSISS_CSR_CONS_NEG_PORT_25G_CR1	BIT(25)
#define HSSISS_CSR_CONS_NEG_PORT_50G_KR2	BIT(26)
#define HSSISS_CSR_CONS_NEG_PORT_50G_CR2	BIT(27)
#define HSSISS_CSR_CONS_NEG_PORT_400_KR8_CR8	BIT(28)
#define HSSISS_CSR_CONS_NEG_PORT_MASK		GENMASK(28,24)
#define HSSISS_CSR_CONS_NEG_PORT_MASK_START	24
#define HSSISS_CSR_CONS_NEG_PORT_400_KR4_CR4	BIT(29)
#define HSSISS_IEEE_EXT_NEG_PORT_MASK_START	29
#define HSSISS_CSR_RS_FEC_NEGOTIATED		BIT(30)
#define HSSISS_CSR_LL_FEC_NEGOTIATED		BIT(31)
#define HSSISS_CSR_FEC_MODES_MASK		GENMASK(31,30)
#define HSSISS_CSR_FEC_MODES_MASK_START		30

#define HSSISS_CSR_ANLT_SEQ_RESET_SEQ		BIT(0)
#define HSSISS_CSR_ANLT_SEQ_DISABLE_AN_TIMER	BIT(1)
#define HSSISS_CSR_ANLT_SEQ_DISABLE_LF_TIMER	BIT(2)
#define HSSISS_CSR_ANLT_SEQ_LT_FAILURE_RES	BIT(12)
#define HSSISS_CSR_ANLT_SEQ_LINK_FAIL_HIGH_BER	BIT(13)
#define HSSISS_CSR_ANLT_SEQ_SKIP_LT_ON_AN_TIMEOUT	BIT(14)
#define HSSISS_CSR_ANLT_SEQ_KR_PAUSE		BIT(31)

#define HSSISS_CSR_AN_ENABLE_AN				BIT(0)
#define HSSISS_CSR_AN_AN_BASE_PAGES_CTL			BIT(1)
#define HSSISS_CSR_AN_NEXT_PAGES_CTL			BIT(2)
#define HSSISS_CSR_AN_LOCAL_DEV_REMOTE_FAULT		BIT(3)
#define HSSISS_CSR_AN_FORCE_TX_NONCE_VALUE		BIT(4)
#define HSSISS_CSR_AN_OVERRIDE_AN_PARAMETERS_EN		BIT(5)
#define HSSISS_CSR_AN_OVERRIDE_AN_CHAN_ENABLE		BIT(6)
#define HSSISS_CSR_AN_IGNORE_NONCE_FIELD		BIT(7)
#define HSSISS_CSR_AN_EN_CONSORTIUM_NEXT_PAGE_SEND	BIT(8)
#define HSSISS_CSR_AN_EN_CONSORTIUM_NEXT_PAGE_RECV	BIT(9)
#define HSSISS_CSR_AN_IGNORE_CONS_NP_TECH_ABILITY_CODE	BIT(11)

#define HSSISS_CSR_LT_ENABLE_LINK_TRAINING	BIT(0)
#define HSSISS_CSR_LT_DIS_MAX_WAIT_TIMER	BIT(1)
#define HSSISS_CSR_LT_DIS_ADAPTATION		BIT(4)

#define HSSISS_CSR_ANLT_IEEE_CAP_MASK		0x00fff000
#define HSSISS_CSR_ANLT_IEEE_CAP_TRAILING_ZEROS		12

//an_status_1 bit
#define HSSISS_CSR_AN_STATUS_1_LP_SELECTOR_MASK 	GENMASK(4,0)
#define HSSISS_CSR_AN_STATUS_1_LP_ECHO_NONCE_MASK	GENMASK(9,5)
#define HSSISS_CSR_AN_STATUS_1_LP_PAUSE_SUPP		BIT(10)
#define HSSISS_CSR_AN_STATUS_1_LP_ASYM_PAUSE_SUPP	BIT(11)
#define HSSISS_CSR_AN_STATUS_1_LP_PAUSE_MASK		GENMASK(12,10)
#define HSSISS_CSR_AN_STATUS_1_LP_RF_STATUS		BIT(13)
#define HSSISS_CSR_AN_STATUS_1_LP_ACK_STATUS		BIT(14)
#define HSSISS_CSR_AN_STATUS_1_LP_NEXT_PAGE		BIT(15)
//an_status_2 bit
#define HSSISS_CSR_AN_STATUS_2_LP_TX_NONCE_MASK		GENMASK(4,0)
#define HSSISS_AN2_LP_TECH_ABL_MASK	GENMASK(26,5)
#define HSSISS_AN2_LP_TECH_ABL_START	5
#define HSSISS_CSR_AN_STATUS_2_LP_FEC_LLFEC		BIT(27)
#define HSSISS_CSR_AN_STATUS_2_LP_FEC_RSFEC		BIT(28)
#define HSSISS_CSR_AN_STATUS_2_LP_FEC_BASER		BIT(29)
#define HSSISS_CSR_AN_STATUS_2_LP_FEC_MASK		GENMASK(31,27)
//an_status_3 bit
#define HSSISS_CSR_AN_STATUS_3_LP_MSG_MASK		GENMASK(10,0)
#define HSSISS_CSR_AN_STATUS_3_LP_TOGGLE		BIT(11)
#define HSSISS_CSR_AN_STATUS_3_LP_ACK2			BIT(12)
#define HSSISS_CSR_AN_STATUS_3_LP_MP			BIT(13)
#define HSSISS_CSR_AN_STATUS_3_LP_ACK			BIT(14)
#define HSSISS_CSR_AN_STATUS_3_LP_NXT_PAGE		BIT(15)
//an_status_4 bit
#define HSSISS_CSR_AN_STATUS_4_BASE_OFFSET		16
#define HSSISS_CSR_AN_STATUS_4_25GBASE_KR1		BIT(20 - HSSISS_CSR_AN_STATUS_4_BASE_OFFSET)
#define HSSISS_CSR_AN_STATUS_4_25GBASE_CR1		BIT(21 - HSSISS_CSR_AN_STATUS_4_BASE_OFFSET)
#define HSSISS_AN4_NPG_ABL_MASK_25G  GENMASK(5,4)
#define HSSISS_AN4_NPG_ABL_MASK_25G_START	4
#define HSSISS_CSR_AN_STATUS_4_50GBASE_KR2		BIT(24 - HSSISS_CSR_AN_STATUS_4_BASE_OFFSET)
#define HSSISS_CSR_AN_STATUS_4_50GBASE_CR2              BIT(25 - HSSISS_CSR_AN_STATUS_4_BASE_OFFSET)
#define HSSISS_AN4_NPG_ABL_MASK_50G  GENMASK(9,8)
#define HSSISS_AN4_NPG_ABL_MASK_50G_START	8
#define HSSISS_AN4_400GBASE_KR8_CR8		BIT(34 - HSSISS_CSR_AN_STATUS_4_BASE_OFFSET)
#define HSSISS_AN4_NPG_ABL_MASK	(HSSISS_AN4_NPG_ABL_MASK_25G |\
							 HSSISS_AN4_NPG_ABL_MASK_50G |\
							 HSSISS_AN4_400GBASE_KR8_CR8)
#define HSSISS_CSR_AN_STATUS_4_LF1_LL_RSFEC_ABILITY	BIT(37 - HSSISS_CSR_AN_STATUS_4_BASE_OFFSET)
#define HSSISS_CSR_AN_STATUS_4_LF2_LL_RSFEC_ABILITY     BIT(38 - HSSISS_CSR_AN_STATUS_4_BASE_OFFSET)
#define HSSISS_CSR_AN_STATUS_4_LF3_LL_RSFEC_ABILITY     BIT(39 - HSSISS_CSR_AN_STATUS_4_BASE_OFFSET)
#define HSSISS_AN4_F1_FEC_CTL_ADV_RSFEC BIT(40 - HSSISS_CSR_AN_STATUS_4_BASE_OFFSET)
#define HSSISS_AN4_F2_FEC_CTL_ADV_BASERFEC BIT(41 - HSSISS_CSR_AN_STATUS_4_BASE_OFFSET)
#define HSSISS_CSR_AN_STATUS_4_F3_FEC_CONTROL_REQ_RSFEC BIT(42 - HSSISS_CSR_AN_STATUS_4_BASE_OFFSET)
#define HSSISS_CSR_AN_STATUS_4_F4_FEC_CONTROL_REQ_BASERFEC BIT(43 - HSSISS_CSR_AN_STATUS_4_BASE_OFFSET)
#define HSSISS_AN4_LFR_LL_RSFEC_REQ	BIT(44 - HSSISS_CSR_AN_STATUS_4_BASE_OFFSET)
#define HSSISS_CSR_AN_STATUS_4_LP_NEXT_PAGE		GENMASK(31,0)

//an_status_6 bit
#define HSSISS_CSR_AN_STATUS_6_LP_CONSORTIUM_NXT_PAGE_EXT	GENMASK(8,0)
#define HSSISS_CSR_AN_STATUS_6_LP_CONSORTIUM_NXT_PAGE_RSVD	GENMASK(12,9)
#define HSSISS_AN6_LP_CONS_NPG_ABL_MASK_25G	GENMASK(14,13)
#define HSSISS_AN6_LP_CONS_NPG_ABL_MASK_25G_START	13
#define HSSISS_AN_LP_NPG_ABL_MASK_25G_NUM_BITS		2
#define HSSISS_AN_LP_NPG_ABL_MASK_50G_NUM_BITS		2
#define HSSISS_AN6_LP_CONS_NPG_ABL_MASK_50G	GENMASK(18,17)
#define HSSISS_AN6_LP_CONS_NPG_ABL_MASK_50G_START	17
#define HSSISS_AN6_LP_CONS_NPG_ABL_MASK (HSSISS_AN6_LP_CONS_NPG_ABL_MASK_25G |\
					  HSSISS_AN6_LP_CONS_NPG_ABL_MASK_50G)
#define HSSISS_CSR_AN_STATUS_6_LP_CONSORTIUM_NXT_PAGE_ABILITY_RSVD	GENMASK(16,15)

/* Ftile only: PTP tile adaptor */
 #define HSSISS_CSR_PTP_ASYMMETRY_BASE           0x20000
 #define HSSISS_CSR_PTP_ASYMMETRY_RANGE          0x20000
 #define HSSISS_CSR_PTP_PEER_TO_PEER_MPD         0x40000
 #define HSSISS_CSR_PTP_PEER_TO_PEER_MPD_RANGE   0x20000

/* DFH */
 #define HSSISS_DFHLO_DFHV0_FEA_REV_MASK         GENMASK(15, 12)
 #define HSSISS_DFHLO_DFHV0_FEA_REV_SHIFT        12

/* Command status bits */
 #define HSSI_SAL_CMDSTS_RD              BIT(0)
 #define HSSI_SAL_CMDSTS_WR              BIT(1)
 #define HSSI_SAL_CMDSTS_ACK             BIT(2)
 #define HSSI_SAL_CMDSTS_BUSY            BIT(3)
 #define HSSI_SAL_CMDSTS_ERR             BIT(4)
 #define HSSI_SAL_CMDSTS_REG_OFFS_MASK   GENMASK(6, 5)
 #define HSSI_SAL_CMDSTS_REG_OFFS_SHIFT  5

/* Control address bits */
 #define HSSI_SAL_CTRLADDR_SALCMD                0xFF
 #define HSSI_SAL_CTRLADDR_PORT_SHIFT            8
 #define HSSI_SAL_CTRLADDR_COUNTER_SHIFT         16
 #define HSSI_SAL_CTRLADDR_LSB_SHIFT             31
 #define HSSI_SAL_CTRLADDR_ADDRBITS_MASK         0xFFFFF
 #define HSSI_SAL_CTRLADDR_ADDRBITS_SHIFT        8
 #define HSSI_SAL_CTRLADDR_TX                    BIT(16)
 #define HSSI_SAL_CTRLADDR_RX                    BIT(17)

/* Hotplug dbg ctrl and status */
 #define HSSI_HOTPLUG_DBG_STS_DISABLE_SHIFT      4

/* Feature CSR v5 only */
 #define HSSISS_FEATURE_CSR_ADDR_MASK            GENMASK(31, 1)
 #define HSSISS_FEATURE_CSR_ADDR_SHIFT           1

/* Bit index and mask */
 #define HSSI_SAL_RESET_MAC_STAT_TX      BIT(16)
 #define HSSI_SAL_RESET_MAC_STAT_RX      BIT(17)

 #define DR_GRP_INDEX    4
 #define HSSISS_VER_CSR_ADDR_MASK                GENMASK(31, 16)
 #define HSSISS_VER_CSR_ADDR_SHIFT               16

 /* GET HSSI PROFILE Register details */
 #define DYN_RCFG_DR_TRIGGER_REG 			0
 #define DYN_RCFG_DR_TX_FULLY_OUT_RESET_REG		2
 #define DYN_RCFG_DR_TX_SRC_ALARM_REG 			4
 #define DYN_RCFG_DR_RX_FULLY_OUT_RESET_REG		6
 #define DYN_RCFG_DR_RX_SRC_ALARM_REG 			8
 #define DYN_RCFG_LOCAL_ERROR_STAT_CTRL_REG 	    0x200
 #define DYN_RCFG_LOCAL_RX_SRC_ALARM_REG	       10

 #define FIRMWARE_ERROR		 GENMASK(23, 16)
 #define NEXT_PROFILE_LO_IND     BIT(15)
 #define NEXT_PROFILE_HI_MASK    GENMASK(30, 16)
 #define NEXT_PROFILE_LO_MASK    GENMASK(15, 0)
 #define NEXT_PROFILE_HI_IND     BIT(31)
 #define NEXT_PROFILE_MAX_ID     GENMASK(14, 0)  /* max valid profile index (0x7FFF) */
 #define NEXT_PROFILE_ENABLE     BIT(15)         /* enable bit for two-register profile scheme */
 #define READY_FOR_DR            BIT(1)

 #define DYN_RCFG_DR_NEXT_PROFILE_0_REG 0x4
 #define DYN_RCFG_DR_NEXT_PROFILE_1_REG (DYN_RCFG_DR_NEXT_PROFILE_0_REG + 4)

/* Bestcase: 100ns, max: 10ms, driver interval: 10us
 * For DR and enable/disable loopback SAL sequences, the whole operation might
 * take more than 10ms and timeout doesn't apply for these sequences, instead
 * polling method is implemeted where a polling counter is used to poll the DR
 * status and it will exit error when the polling counter expires.
 * <TODO>
 */
 #define FW_ACK_POLL_INTERVAL_US         10
 #define FW_ACK_POLL_TIMEOUT_US          10000

/* CSR read/write macros */
 #define csrrd32_withoffset(base, csroff, offs) csrrd32(base, (offs) + (csroff))
 #define csrwr32_withoffset(val, base, csroff, offs) \
		csrwr32(val, base, (offs) + (csroff))

 #define MASK(idx, nr) (((1 << (nr)) - 1) << (((idx) + 1) - (nr)))
 #define test_reg_bits(val, idx, numbits) ((val) & MASK(idx, numbits))
 #define clear_reg_bits(val, idx, numbits) ((val) & ~(MASK(idx, numbits)))
 #define update_bit(val, pos, bit) ( ((val) & ~(1 << (pos))) | ((bit) << (pos)))

int hssidrv_cold_rst(struct platform_device *pdev);
void hssidrv_hotplug_enable(struct platform_device *pdev, bool enable);

int hssidrv_probe_init(struct platform_device *pdev);
int hssidrv_get_fw_version(struct platform_device *pdev, u32 cmd, void *priv_data);
int hssidrv_ncsi_link_status(struct platform_device *pdev, u32 cmd, void *priv_data);
int hssidrv_read_mac_stat(struct platform_device *pdev, u32 cmd, void *priv_data);
int hssidrv_get_mtu(struct platform_device *pdev, u32 cmd, void *priv_data);
int hssidrv_set_mtu(struct platform_device *pdev, u32 cmd, void *priv_data);
int hssidrv_reset_mac_stat(struct platform_device *pdev, u32 cmd, void *priv_data);
int hssidrv_get_dr_profile(struct platform_device *pdev, u32 cmd, void *dr_data);
int hssidrv_set_dr_profile(struct platform_device *pdev, u32 cmd, void *dr_data);
int hssidrv_test_nios(struct platform_device *pdev, u32 cmd);
int hssidrv_get_set_csr(struct platform_device *pdev, u32 cmd, void *csr_data,
			bool rd);
int hssidrv_enable_disable_loopback(struct platform_device *pdev, u32 cmdid,
				    void *data);
hssi_eth_port_attr hssidrv_get_ethport_attr(struct platform_device *pdev, int port);
int hssidrv_set_ethport_status(struct platform_device *pdev, int port, u32 data);
hssi_eth_port_sts hssidrv_get_ethport_status(struct platform_device *pdev, int port);
int hssidrv_hotplug_disable_status(struct platform_device *pdev);
int hssidrv_cold_rst(struct platform_device *pdev);
int hssidrv_anlt_update(struct platform_device *pdev, int port, bool enable_anlt);
u32 hssidrv_anlt_get_status(struct platform_device *pdev, int port);
u32 hssidrv_anlt_get_cfg(struct platform_device *pdev, int port);
u32 hssidrv_anlt_get_ext_status(struct platform_device *pdev, int port, int *an_status);

 #ifdef CONFIG_DEBUG_FS
struct hssiss_dbg *hssiss_dbgfs_init(struct platform_device *pdev);
void hssiss_dbgfs_remove(struct hssiss_dbg *d);
 #endif /* CONFIG_DEBUG_FS */

 #endif
