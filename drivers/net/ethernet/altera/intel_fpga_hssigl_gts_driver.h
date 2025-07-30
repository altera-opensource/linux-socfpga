// SPDX-License-Identifier: GPL-2.0
/* Altera FPGA HSSI glue logic GTS driver interface
 * Copyright (C) 2024, 2025 Altera Corporation. All rights reserved
 *
 * Contributors:
 *   Preetam Narayan
 *
 */
 #ifndef __INTEL_FPGA_HSSIGL_GTS_DRIVER_H__
 #define __INTEL_FPGA_HSSIGL_GTS_DRIVER_H__

/* *_BASE_START address is relative offset from the sscsr register as
 * selected by the DTS */
 #define  ANLT_BASE_START       0x010000
 #define  ANLT_BASE_LEN	        (0x001000 - 1)
 #define  DR_BASE_START         0x011000
 #define  DR_BASE_LEN           0x000080
 #define  SOFTIP_BASE_START     0x000100
 #define  SOFTIP_BASE_LEN       0x00004F
 #define  SOFTIP_PTP_START      0x000800
 #define  SOFTIP_PTP_LEN        0x000118
 #define  HARDIP_PLD_START      0x020000
 #define  HARDIP_PLD_LEN        0x0003FF
 #define  HARDIP_PTP_START      0x040000
 #define  HARDIP_PTP_LEN	0x0007FF
 #define  HARDIP_EMAC_START     0x050000
 #define  HARDIP_EMAC_LEN       0x000FFF
 #define  HARDIP_PCS_FEC_START  0x051000
 #define  HARDIP_PCS_FEC_LEN    (0x2F0000 - 1)
 #define  HARDIP_XCVR_PMA_START 0x080000
 #define  HARDIP_XCVR_PMA_LEN   0x001FFF
 #define  HARDIP_PMA_START      0x090000
 #define  HARDIP_PMA_LEN	0x0D0000

 #define  USER_CSR_START        0x000000
 #define  USER_CSR_LEN	        0x001000

 #define CHANNEL_OFFSET	       0x200000
 #define DELAY_READY_WAIT      0x000400

 #define BIT_VALUE(x, n)       (((x) & BIT(n)) ? 1 : 0)
 #define RX_PCS_READY_STATUS(x) BIT_VALUE(x, 3)
 #define TX_LANE_STABLE_STATUS(x) BIT_VALUE(x, 2)

 #define U64_FROM_32(high, low) ((((u64)(high)) << 32) | (u32)(low))

struct eth_sub_system {
	u32 anlt_base_start;
	u32 anlt_base_len;

	u32 dr_base_start;
	u32 dr_base_len;

	u32 softip_base_start;
	u32 softip_base_len;

	u32 softip_ptp_start;
	u32 softip_ptp_len;

	u32 hardip_pld_start;
	u32 hardip_pld_len;

	u32 hardip_ptp_start;
	u32 hardip_ptp_len;

	u32 hardip_emac_start;
	u32 hardip_emac_len;

	u32 hardip_pcs_fec_start;
	u32 hardip_pcs_fec_len;

	u32 hardip_xcvr_pma_start;
	u32 hardip_xcvr_pma_len;

	u32 hardip_pma_start;
	u32 hardip_pma_len;

	u32 user_csr_start;
	u32 user_csr_len;
};

 #endif
