/* SPDX-License-Identifier: GPL-2.0 */
/* Intel FPGA HSSI SS driver
 * Copyright (C) 2022, 2024 Intel Corporation. All rights reserved
 *
 * Contributors:
 *   Subhransu S. Prusty
 *   Preetam Narayan
 */
#ifndef __INTEL_FPGA_HSSISS_DRV_H__
#define __INTEL_FPGA_HSSISS_DRV_H__

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
#define HSSI_DR_GRP_MASK        GENMASK(6, 4)
#define HSSI_DR_PROFILE_MASK    GENMASK(3, 0)
#define HSSISS_VER_CSR_ADDR_MASK                GENMASK(31, 16)
#define HSSISS_VER_CSR_ADDR_SHIFT               16

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
#define csrrd32_withoffset(base, csroff, offs) csrrd32(base, offs + csroff)
#define csrwr32_withoffset(val, base, csroff, offs) \
		csrwr32(val, base, offs + csroff)

#define MASK(idx, nr) (((1 << nr) - 1) << ((idx + 1) - nr))
#define test_reg_bits(val, idx, numbits) (val & MASK(idx, numbits))
#define clear_reg_bits(val, idx, numbits) (val & ~(MASK(idx, numbits)))

int hssidrv_cold_rst(struct platform_device *pdev);
void hssidrv_hotplug_enable(struct platform_device *pdev, bool enable);
void hssidrv_probe_init(struct platform_device *pdev);
int hssidrv_get_fw_version(struct platform_device *pdev, u32 cmd, void *priv_data, bool atomic);
int hssidrv_ncsi_link_status(struct platform_device *pdev, u32 cmd, void *priv_data, bool atomic);
int hssidrv_read_mac_stat(struct platform_device *pdev, u32 cmd, void *priv_data, bool atomic);
int hssidrv_get_mtu(struct platform_device *pdev, u32 cmd, void *priv_data, bool atomic);
int hssidrv_reset_mac_stat(struct platform_device *pdev, u32 cmd, void *priv_data, bool atomic);
int hssidrv_get_set_dr_profile(struct platform_device *pdev, u32 cmd, void *dr_data,
			       bool rd, bool atomic);
int hssidrv_test_nios(struct platform_device *pdev, u32 cmd, bool atomic);
int hssidrv_get_set_csr(struct platform_device *pdev, u32 cmd, void *csr_data,
			bool rd, bool atomic);
int hssidrv_enable_disable_loopback(struct platform_device *pdev, u32 cmdid,
				    void *data, bool atomic);
hssi_eth_port_attr hssidrv_get_ethport_attr(struct platform_device *pdev, int port);
int hssidrv_set_ethport_status(struct platform_device *pdev, int port, u32 data);
hssi_eth_port_sts hssidrv_get_ethport_status(struct platform_device *pdev, int port);
int hssidrv_hotplug_disable_status(struct platform_device *pdev);
int hssidrv_cold_rst(struct platform_device *pdev);

#ifdef CONFIG_DEBUG_FS
struct hssiss_dbg *hssiss_dbgfs_init(struct platform_device *pdev);
void hssiss_dbgfs_remove(struct hssiss_dbg *d);
#endif /* CONFIG_DEBUG_FS */

#endif
