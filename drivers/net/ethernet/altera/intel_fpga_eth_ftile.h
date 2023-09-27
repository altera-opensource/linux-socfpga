/* SPDX-License-Identifier: GPL-2.0 */
/* Intel FPGA F-tile Ethernet MAC driver
 * Copyright (C) 2020-2022 Intel Corporation. All rights reserved.
 *
 * Contributors:
 *   Roman Bulgakov
 *   Yu Ying Choo
 *   Joyce Ooi
 *   Arzu Ozdogan-tackin
 *   Alexis Rodriguez
 *
 * Original driver contributed by GlobalLogic.
 *
 * This file is ported from intel_fpga_etile.h with updates
 * to match Intel FPGA F-tile register definitions
 *
 */

#ifndef __INTEL_FPGA_FTILE_ETH_H__
#define __INTEL_FPGA_FTILE_ETH_H__

#define INTEL_FPGA_FTILE_ETH_RESOURCE_NAME "intel_fpga_ftile"

#include "intel_fpga_eth_main.h"

//#include "intel_freq_control.h"
#define INTEL_FPGA_BYTE_ALIGN	8
#define INTEL_FPGA_WORD_ALIGN	32

#define MOD_PARAM_PERM  0644


/* Poll for PTP RX READY bit, If any disturbance on RX PCS, this can go low
 * So that time we need to rerun the PTP TX RX USER FLOW again.
 */
#define PTP_TX_RX_USER_FLOW_POLL_TIMEOUT            500

#define INTEL_FPGA_FTILE_SW_RESET_WATCHDOG_CNTR		1000000

#define INTEL_FPGA_FTILE_UI_VALUE_10G			0x018D3019 // 10G-1
#define INTEL_FPGA_FTILE_UI_VALUE_25G			0x009EE00A // 25G-1, 50G-2 (No-FEC), 100G-4 (KR-FEC), 200G-8
#define INTEL_FPGA_FTILE_UI_VALUE_50G			0x004F7005 // 50G-1, 100G-2, 200G-4, 400G-8
#define INTEL_FPGA_FTILE_UI_VALUE_100G			0x0027B802 // 100G-1, 200G-2, 400G-4

#define INTEL_FPGA_TX_PMA_DELAY_25G_UX			79   // 10G-1, 25G-1, 50G-2 (No-FEC + KP-FEC), 100G-4 (KR-FEC + KP-FEC), 200G-8 (KP-FEC) FGT
#define INTEL_FPGA_TX_PMA_DELAY_25G_BK			288  // 10G-1, 25G-1, 50G-2 (No-FEC + KP-FEC), 100G-4 (KR-FEC + KP-FEC), 200G-8 (KP-FEC) FHT
#define INTEL_FPGA_RX_PMA_DELAY_25G_UX			88   // 10G-1, 25G-1, 50G-2 (No-FEC + KP-FEC), 100G-4 (KR-FEC + KP-FEC), 200G-8 (KP-FEC) FGT
#define INTEL_FPGA_RX_PMA_DELAY_25G_BK			250  // 10G-1, 25G-1, 50G-2 (No-FEC + KP-FEC), 100G-4 (KR-FEC + KP-FEC), 200G-8 (KP-FEC) FHT
#define INTEL_FPGA_TX_PMA_DELAY_50G_UX			158  // 50G-1 (KP-FEC), 100G-2 (LL-FEC), 200G-4 (LL-FEC), 400G8 (LL-FEC) FGT
#define INTEL_FPGA_TX_PMA_DELAY_50G_BK			576  // 50G-1 (KP-FEC), 100G-2 (LL-FEC), 200G-4 (LL-FEC), 400G8 (LL-FEC) FHT
#define INTEL_FPGA_RX_PMA_DELAY_50G_UX			175  // 50G-1 (KP-FEC), 100G-2 (LL-FEC), 200G-4 (LL-FEC), 400G8 (LL-FEC) FGT
#define INTEL_FPGA_RX_PMA_DELAY_50G_BK			500  // 50G-1 (KP-FEC), 100G-2 (LL-FEC), 200G-4 (LL-FEC), 400G8 (LL-FEC) FHT
#define INTEL_FPGA_TX_PMA_DELAY_100G_BK			1058 // 100G-1 (KP-FEC), 200G-2 (KP-FEC), 400G-4 (KP-FEC) FHT
#define INTEL_FPGA_RX_PMA_DELAY_100G_BK			996  // 100G-1 (KP-FEC), 200G-2 (KP-FEC), 400G-4 (KP-FEC) FHT

enum intel_fpga_ftile_eth_rate {
	INTEL_FPGA_FTILE_ETH_RATE_10G_25G  = 0, // 10G / 25G
	INTEL_FPGA_FTILE_ETH_RATE_50G      = 1, // 50G
	INTEL_FPGA_FTILE_ETH_RATE_40G_100G = 2, // 40G / 100G
	INTEL_FPGA_FTILE_ETH_RATE_200G     = 3, // 200G
	INTEL_FPGA_FTILE_ETH_RATE_400G     = 4  // 400G
};

/* Flow Control defines */
#define FLOW_OFF	0
#define FLOW_RX		1
#define FLOW_TX		2
#define FLOW_ON		(FLOW_TX | FLOW_RX)

/* Ethernet Reconfiguration Interface
 * Auto Negotiation and Link Training
 * Bit Definitions
 */
/* 0x02C0: ANLT Sequencer Config */
#define ETH_ANLT_SEQ_CONF_RESET_SEQ				BIT(0)
#define ETH_ANLT_SEQ_CONF_DISABLE_AN_TIMER			BIT(1)
#define ETH_ANLT_SEQ_CONF_DISABLE_LF_TIMER			BIT(2)
#define ETH_ANLT_SEQ_CONF_SEQ_FORCE_MODE_NONE			(0 << 4)
#define ETH_ANLT_SEQ_CONF_SEQ_FORCE_MODE_10G_R1			(1 << 4)
#define ETH_ANLT_SEQ_CONF_SEQ_FORCE_MODE_25G_R1			(2 << 4)
#define ETH_ANLT_SEQ_CONF_SEQ_FORCE_MODE_50G_R2			(3 << 4)
#define ETH_ANLT_SEQ_CONF_SEQ_FORCE_MODE_100G_R4		(4 << 4)
#define ETH_ANLT_SEQ_CONF_SEQ_FORCE_MODE_40G_R4			(5 << 4)
#define ETH_ANLT_SEQ_CONF_SEQ_FORCE_MODE_100G_R2		(6 << 4)
#define ETH_ANLT_SEQ_CONF_SEQ_FORCE_MODE_50G_R1			(7 << 4)
#define ETH_ANLT_SEQ_CONF_SEQ_FORCE_MODE_200G_R4		(8 << 4)
#define ETH_ANLT_SEQ_CONF_SEQ_FORCE_MODE_400G_R8		(9 << 4)
#define ETH_ANLT_SEQ_CONF_SEQ_FORCE_MODE_100G_R1		(10 << 4)
#define ETH_ANLT_SEQ_CONF_SEQ_FORCE_MODE_200G_R2		(11 << 4)
#define ETH_ANLT_SEQ_CONF_SEQ_FORCE_MODE_400G_R4		(12 << 4)
#define ETH_ANLT_SEQ_CONF_SEQ_FORCE_MODE_RESERVED_13		(13 << 4)
#define ETH_ANLT_SEQ_CONF_SEQ_FORCE_MODE_RESERVED_14		(14 << 4)
#define ETH_ANLT_SEQ_CONF_SEQ_FORCE_MODE_NO_RECONFIG		(15 << 4)
#define ETH_ANLT_SEQ_CONF_LINK_FAILURE_RESP			BIT(12)
#define ETH_ANLT_SEQ_CONF_LINK_FAIL_IF_HIBER			BIT(13)
#define ETH_ANLT_SEQ_CONF_SKIP_LINK_ON_TO			BIT(14)

/* 0x02C4: ANLT Sequencer Status */
#define ETH_ANLT_SEQ_STAT_LINK_READY				BIT(0)
#define ETH_ANLT_SEQ_STAT_AUTO_NEG_TIMEOUT			BIT(1)
#define ETH_ANLT_SEQ_STAT_LINK_TR_TIMEOUT			BIT(2)
#define ETH_ANLT_SEQ_STAT_RECONF_MODE_AN			BIT(8)
#define ETH_ANLT_SEQ_STAT_RECONF_MODE_LT			BIT(9)
#define ETH_ANLT_SEQ_STAT_RECONF_MODE_10G_DATA			BIT(10)
#define ETH_ANLT_SEQ_STAT_RECONF_MODE_25G_DATA			BIT(11)
#define ETH_ANLT_SEQ_STAT_RECONF_MODE_50G_R2			BIT(12)
#define ETH_ANLT_SEQ_STAT_RECONF_MODE_100G_R4			BIT(13)
#define ETH_ANLT_SEQ_STAT_RECONF_MODE_40G_R4			BIT(14)
#define ETH_ANLT_SEQ_STAT_RECONF_MODE_100G_R2			BIT(15)
#define ETH_ANLT_SEQ_STAT_RECONF_MODE_50G_R1			BIT(16)
#define ETH_ANLT_SEQ_STAT_RECONF_MODE_200G_R4			BIT(17)
#define ETH_ANLT_SEQ_STAT_RECONF_MODE_400G_R8			BIT(18)
#define ETH_ANLT_SEQ_STAT_RECONF_MODE_100G_R1			BIT(19)
#define ETH_ANLT_SEQ_STAT_RECONF_MODE_200G_R2			BIT(20)
#define ETH_ANLT_SEQ_STAT_RECONF_MODE_400G_R4			BIT(21)

/* 0x0300: Auto Negotiation Config Register 1 */
#define ETH_AUTO_NEG_CONF_1_EN					BIT(0)
#define ETH_AUTO_NEG_CONF_1_EN_AN_BASE_PAGE			BIT(1)
#define ETH_AUTO_NEG_CONF_1_EN_AN_NEXT_PAGE			BIT(2)
#define ETH_AUTO_NEG_CONF_1_FORCE_LOCAL_DEV_REM_FAULT		BIT(3)
#define ETH_AUTO_NEG_CONF_1_OVERRIDE_AN_CHAN_EN			BIT(6)
#define ETH_AUTO_NEG_CONF_1_IGNORE_NONCE_FIELD			BIT(7)
#define ETH_AUTO_NEG_CONF_1_EN_CONSORT_NEXT_PAGE_SEND		BIT(8)
#define ETH_AUTO_NEG_CONF_1_EN_CONSORT_NEXT_PAGE_RECV		BIT(9)
#define ETH_AUTO_NEG_CONF_1_IGNORE_CONSORT_NEXT_PAGE_CODE	BIT(11)
#define ETH_AUTO_NEG_CONF_1_CONSORT_OUI				(0xFFFF << 16)

/* 0x0304: Auto Negotiation Config Register 2 */
#define ETH_AUTO_NEG_CONF_2_RESET_AN				BIT(0)
#define ETH_AUTO_NEG_CONF_2_AN_NEXT_PAGE			BIT(8)
#define ETH_AUTO_NEG_CONF_2_CONST_OUI_UPPER			(0xFF << 16)

/* 0x0308: Auto Negotiation Status Register */
#define ETH_AUTO_NEG_STATUS_AN_PAGE_RECEIVED			BIT(1)
#define ETH_AUTO_NEG_STATUS_AN_COMPLETE				BIT(2)
#define ETH_AUTO_NEG_STATUS_AN_ADV_REM_FAULT			BIT(3)
#define ETH_AUTO_NEG_STATUS_PHY_AN_ABILITY			BIT(5)
#define ETH_AUTO_NEG_STATUS_AN_STATUS				BIT(6)
#define ETH_AUTO_NEG_STATUS_AN_LP_ABILITY			BIT(7)
#define ETH_AUTO_NEG_STATUS_BASE_R_FEC_NEGOTIATED_EN		BIT(8)
#define ETH_AUTO_NEG_STATUS_CONSORT_NEXT_PAGE_RECV		BIT(10)
#define ETH_AUTO_NEG_STATUS_AN_FAILURE				BIT(11)
#define ETH_AUTO_NEG_STATUS_IEEE_10GBASE_KR			BIT(12)
#define ETH_AUTO_NEG_STATUS_IEEE_40GBASE_KR4			BIT(13)
#define ETH_AUTO_NEG_STATUS_IEEE_40GBASE_CR4			BIT(14)
#define ETH_AUTO_NEG_STATUS_IEEE_100GBASE_KR4			BIT(15)
#define ETH_AUTO_NEG_STATUS_IEEE_100GBASE_CR4			BIT(16)
#define ETH_AUTO_NEG_STATUS_IEEE_25GBASE_KRS_CRS		BIT(17)
#define ETH_AUTO_NEG_STATUS_IEEE_25GBASE_KR_CR			BIT(18)
#define ETH_AUTO_NEG_STATUS_IEEE_50GBASE_KR_CR			BIT(19)
#define ETH_AUTO_NEG_STATUS_IEEE_100GBASE_KR2_CR2		BIT(20)
#define ETH_AUTO_NEG_STATUS_IEEE_200GBASE_KR4_CR4		BIT(21)
#define ETH_AUTO_NEG_STATUS_IEEE_100GBASE_KR_CR			BIT(22)
#define ETH_AUTO_NEG_STATUS_IEEE_200GBASE_KR2_CR2		BIT(23)
#define ETH_AUTO_NEG_STATUS_CONST_25GBASE_KR1			BIT(24)
#define ETH_AUTO_NEG_STATUS_CONST_25GBASE_CR1			BIT(25)
#define ETH_AUTO_NEG_STATUS_CONST_50GBASE_KR2			BIT(26)
#define ETH_AUTO_NEG_STATUS_CONST_50GBASE_CR2			BIT(27)
#define ETH_AUTO_NEG_STATUS_CONST_400GBASE_KR8_CR8		BIT(28)
#define ETH_AUTO_NEG_STATUS_CONST_400GBASE_KR4_CR4		BIT(29)
#define ETH_AUTO_NEG_STATUS_RS_FEC_NEGOTIATED			BIT(30)
#define ETH_AUTO_NEG_STATUS_LL_FEC_NEGOTIATED			BIT(31)

/* 0x030C: Auto Negotiation Config Register 3 */
#define ETH_AUTO_NEG_CONF_3_USER_BASE_PAGE_LOW_SELECTOR			0x1F
#define ETH_AUTO_NEG_CONF_3_USER_BASE_PAGE_LOW_ECHOED_NONE		(0x1F << 5)
#define ETH_AUTO_NEG_CONF_3_USER_BASE_PAGE_LOW_PAUSE_BITS		(7 << 10)
#define ETH_AUTO_NEG_CONF_3_USER_BASE_PAGE_LOW_REMOTE_FAULT		BIT(13)
#define ETH_AUTO_NEG_CONF_3_USER_BASE_PAGE_LOW_ACK			BIT(14)
#define ETH_AUTO_NEG_CONF_3_USER_BASE_PAGE_LOW_NEXT_PG			BIT(15)

/* 0x0314: Auto Negotiation Config Register 5 */
#define ETH_AUTO_NEG_CONF_5_MSG_CODE				0x7FF
#define ETH_AUTO_NEG_CONF_5_TOGGLE				BIT(11)
#define ETH_AUTO_NEG_CONF_5_ACK2				BIT(12)
#define ETH_AUTO_NEG_CONF_5_MP					BIT(13)
#define ETH_AUTO_NEG_CONF_5_ACK					BIT(14)
#define ETH_AUTO_NEG_CONF_5_NEXT_PAGE				BIT(15)

/* 0x0318: Auto Negotiation Config Register 6 */
#define ETH_AUTO_NEG_CONF_6_USER_NEXT_PAGE_HIGH			0xFFFFFFFF

/* 0x031C: Auto Negotiation Status Register 1 */
#define ETH_AUTO_NEG_STAT_1_LP_BASE_PG_LO_SEL			0x1F
#define ETH_AUTO_NEG_STAT_1_LP_BASE_PG_LO_ECHO			(0x1F << 5)
#define ETH_AUTO_NEG_STAT_1_LP_BASE_PG_LO_PAUSE			(0x7 << 10)
#define ETH_AUTO_NEG_STAT_1_LP_BASE_PG_LO_RF			BIT(13)
#define ETH_AUTO_NEG_STAT_1_LP_BASE_PG_LO_ACK			BIT(14)
#define ETH_AUTO_NEG_STAT_1_LP_BASE_PG_LO_NP			BIT(15)

/* 0x0320: Auto Negotiation Status Register 2 */
#define ETH_AUTO_NEG_STAT_2_LP_BASE_PG_HI_TX_NONCE		0x1F
#define ETH_AUTO_NEG_STAT_2_LP_BASE_PG_HI_TECH_AB		0x3FFFFFE0
#define ETH_AUTO_NEG_STAT_2_LP_BASE_PG_HI_FEC			(0x3 << 30)

/* 0x0324: Auto Negotiation Status Register 3 */
#define ETH_AUTO_NEG_STAT_3_LP_NEXT_PG_LO_MSG			0x7FF
#define ETH_AUTO_NEG_STAT_3_LP_NEXT_PG_LO_TOGGLE		BIT(11)
#define ETH_AUTO_NEG_STAT_3_LP_NEXT_PG_LO_ACK2			BIT(12)
#define ETH_AUTO_NEG_STAT_3_LP_NEXT_PG_LO_MP			BIT(13)
#define ETH_AUTO_NEG_STAT_3_LP_NEXT_PG_LO_ACK			BIT(14)
#define ETH_AUTO_NEG_STAT_3_LP_NEXT_PG_LO_NEXT_PAGE		BIT(15)

/* 0x0328: Auto Negotiation Status Register 4 */
#define ETH_AUTO_NEG_STAT_4_LP_NEXT_PG_HI			0xFFFFFFFF

/* 0x0330: AN Channel Override */
#define ETH_OR_AN_CH						0x3
#define ETH_OR_RSFEC_ABILITY					BIT(8)
#define ETH_OR_25G_RSFEC_ABILITY				BIT(9)
#define ETH_OR_RSFEC_REQ					BIT(10)
#define ETH_OR_25G_RSFEC_REQ					BIT(11)
#define ETH_OR_IEEE_PORT_ABILITY_10GBASE_KR			BIT(12)
#define ETH_OR_IEEE_PORT_ABILITY_40GBASE_KR4			BIT(13)
#define ETH_OR_IEEE_PORT_ABILITY_40GBASE_CR4			BIT(14)
#define ETH_OR_IEEE_PORT_ABILITY_100GBASE_KR4			BIT(15)
#define ETH_OR_IEEE_PORT_ABILITY_100GBASE_CR4			BIT(16)
#define ETH_OR_IEEE_PORT_ABILITY_25GBASE_KRS_CRS		BIT(17)
#define ETH_OR_IEEE_PORT_ABILITY_25GBASE_KR_CR			BIT(18)
#define ETH_OR_IEEE_PORT_ABILITY_50GBASE_KR_CR			BIT(19)
#define ETH_OR_IEEE_PORT_ABILITY_100GBASE_KR2_CR2		BIT(20)
#define ETH_OR_IEEE_PORT_ABILITY_200GBASE_KR4_CR4		BIT(21)
#define ETH_OR_IEEE_PORT_ABILITY_100GBASE_KR_CR			BIT(22)
#define ETH_OR_IEEE_PORT_ABILITY_200GBASE_KR2_CR2		BIT(23)
#define ETH_OR_CONST_PORT_ABILITY_25GBASE_KR1			BIT(24)
#define ETH_OR_CONST_PORT_ABILITY_25GBASE_CR1			BIT(25)
#define ETH_OR_CONST_PORT_ABILITY_50GBASE_KR2			BIT(26)
#define ETH_OR_CONST_PORT_ABILITY_50GBASE_CR2			BIT(27)
#define ETH_OR_CONST_PORT_ABILITY_400GBASE_KR8_CR8		BIT(28)

/* 0x0338: Consortium Next Page Link Partner Status */
#define ETH_LP_CONSORT_NEXT_PG_TECH_25GBASE_KR1_AB		BIT(13)
#define ETH_LP_CONSORT_NEXT_PG_TECH_25GBASE_CR1_AB		BIT(14)
#define ETH_LP_CONSORT_NEXT_PG_TECH_50GBASE_KR2_AB		BIT(17)
#define ETH_LP_CONSORT_NEXT_PG_TECH_50GBASE_CR2_AB		BIT(18)

/* 0x0340: Link training Config 1 */
#define ETH_LNK_CONF_LT_EN					BIT(0)
#define ETH_LNK_CONF_LT_MAX_WAIT_TMR_DIS			BIT(1)
#define ETH_LNK_CONF_LT_MAX_WAIT_TMR_SCALE_NONE			(0 << 2)
#define ETH_LNK_CONF_LT_MAX_WAIT_TMR_SCALE_1E3			(1 << 2)
#define ETH_LNK_CONF_LT_MAX_WAIT_TMR_SCALE_1E4			(2 << 2)
#define ETH_LNK_CONF_LT_MAX_WAIT_TMR_SCALE_1E6			(3 << 2)
#define ETH_LNK_CONF_LT_ADAPT_DIS				BIT(4)

/* 0x0344: Link training Config 2 */
#define ETH_LNK_CONF_LT_RESTART_LN(x)				BIT(x) // x:0-7

/* 0x0348: Link training Status 1 */
#define ETH_LNK_TR_STAT_LN(x)					BIT(0+4*(x)) // x:0-7
#define ETH_LNK_TR_STAT_FRM_LN0					BIT(1+4*(x)) // x:0-7
#define ETH_LNK_TR_STAT_STARTUP_LN0				BIT(2+4*(x)) // x:0-7
#define ETH_LNK_TR_STAT_FAIL_LN0				BIT(3+4*(x)) // x:0-7

/* 0x0100: Device and IP variant */
#define ETH_GUI_OPTION_DEVICE_NAME				3
#define ETH_GUI_OPTION_DEVICE_NAME_INTEL_AGILEX			(1 << 0)
#define ETH_GUI_OPTION_TILE_NAME				(3 << 2)
#define ETH_GUI_OPTION_TILE_NAME_F_TILE				(3 << 2)
#define ETH_GUI_OPTION_ETH_RATE					(7 << 5)
#define ETH_GUI_OPTION_ETH_RATE_SHIFT				5
#define ETH_GUI_OPTION_ETH_RATE_10G				(0 << 5)
#define ETH_GUI_OPTION_ETH_RATE_25G				(1 << 5)
#define ETH_GUI_OPTION_ETH_RATE_40G				(2 << 5)
#define ETH_GUI_OPTION_ETH_RATE_50G				(3 << 5)
#define ETH_GUI_OPTION_ETH_RATE_100G				(4 << 5)
#define ETH_GUI_OPTION_ETH_RATE_200G				(5 << 5)
#define ETH_GUI_OPTION_ETH_RATE_400G				(6 << 5)
#define ETH_GUI_OPTION_ANLT_ENABLE				BIT(8)
#define ETH_GUI_OPTION_MODULATION_TYPE				BIT(9)
#define ETH_GUI_OPTION_RSFEC_TYPE				(7 << 10)
#define ETH_GUI_OPTION_RSFEC_TYPE_NONE				(0 << 10)
#define ETH_GUI_OPTION_RSFEC_TYPE_FIRECODE			(1 << 10)
#define ETH_GUI_OPTION_RSFEC_TYPE_RS_528_514			(2 << 10)
#define ETH_GUI_OPTION_RSFEC_TYPE_RS_544_514			(3 << 10)
#define ETH_GUI_OPTION_RSFEC_TYPE_RS_272_258			(4 << 10)
#define ETH_GUI_OPTION_PTP_ENABLE				BIT(13)
#define ETH_GUI_OPTION_FLOW_CTRL_MODE				(7 << 14)
#define ETH_GUI_OPTION_FLOW_CTRL_MODE_DISABLED			(0 << 14)
#define ETH_GUI_OPTION_FLOW_CTRL_MODE_SFC			(1 << 14)
#define ETH_GUI_OPTION_FLOW_CTRL_MODE_SFC_NO_XOFF		(2 << 14)
#define ETH_GUI_OPTION_FLOW_CTRL_MODE_PFC			(3 << 14)
#define ETH_GUI_OPTION_FLOW_CTRL_MODE_PFC_NO_XOFF		(4 << 14)
#define ETH_GUI_OPTION_FLOW_CTRL_MODE_SFC_PFC			(5 << 14)
#define ETH_GUI_OPTION_FLOW_CTRL_MODE_SFC_PFC_NO_XOFF		(6 << 14)
#define ETH_GUI_OPTION_CLIENT_INTF				(7 << 17)
#define ETH_GUI_OPTION_CLIENT_INTF_MAC_SEG			(0 << 17)
#define ETH_GUI_OPTION_CLIENT_INTF_MAC_AVALON			(1 << 17)
#define ETH_GUI_OPTION_CLIENT_INTF_PCS				(2 << 17)
#define ETH_GUI_OPTION_CLIENT_INTF_OTN				(3 << 17)
#define ETH_GUI_OPTION_CLIENT_INTF_FLEXE			(4 << 17)
#define ETH_GUI_OPTION_XCVR_TYPE				BIT(20)
#define ETH_GUI_OPTION_NUM_LANES				(0xF << 21)
#define ETH_GUI_OPTION_NUM_LANES_SHIFT				21
#define ETH_GUI_OPTION_NUM_LANES_1				(1 << 21)
#define ETH_GUI_OPTION_NUM_LANES_2				(2 << 21)
#define ETH_GUI_OPTION_NUM_LANES_4				(4 << 21)
#define ETH_GUI_OPTION_NUM_LANES_8				(8 << 21)

/* 0x0108: IP Soft Reset Register eth_reset */
#define ETH_EIO_SYS_RST						BIT(0)
#define ETH_SOFT_TX_RST						BIT(1)
#define ETH_SOFT_RX_RST						BIT(2)
#define ETH_TX_CLEAR_ALARM					(0xFF << 16)
#define ETH_RX_CLEAR_ALARM					(0xFF << 24)

/* 0x010C: IP Reset Status Register eth_reset_status */
#define ETH_RST_ACK_N						BIT(0)
#define ETH_TX_RST_ACK_N					BIT(1)
#define ETH_RX_RST_ACK_N					BIT(2)
#define ETH_TX_LANE_CUR_STATE					(7 << 8)
#define ETH_RX_LANE_CUR_STATE					(7 << 12)
#define ETH_TX_ALARM						(0xFF << 16)
#define ETH_RX_ALARM						(0xFF << 24)

/* 0x0118: PHY PCS status */
#define ETH_PHY_PCS_STATUS_DSKEW_STATUS				BIT(0)
#define ETH_PHY_PCS_STATUS_DSKEW_CHNG				BIT(1)
#define ETH_PHY_PCS_STATUS_TX_LANES_STABLE			BIT(2)
#define ETH_PHY_PCS_STATUS_RX_PCS_READY				BIT(3)
#define ETH_PHY_PCS_STATUS_KR_MODE				BIT(4) // AN/LT mode
#define ETH_PHY_PCS_STATUS_KR_FEC_MODE				BIT(5) // AN/LT FEC mode

/* 0x011C: PHY PCS control */
#define ETH_PHY_PCS_CONTROL_CLR_DSKEW_CHNG			BIT(0)

/* 0x0120: Link Fault Status */
#define ETH_LINK_FAULT_STATUS_LFAULT				BIT(0)
#define ETH_LINK_FAULT_STATUS_RFAULT				BIT(1)

/* 0x0148: RX MAC Adapter Control  */
#define ETH_RX_MAC_ADAPT_DROPPED_CTRL_CLEAR			BIT(0)
#define ETH_RX_MAC_ADAPT_DROPPED_CTRL_SNAPSHOT			BIT(1)

/* 0x080 PHY frame erors detected */
#define ETH_PHY_FRM_ERROR					0xFFFFF

/* 0x084: PHY RX PCS status */
#define ETH_PHY_RX_PCS_ALIGNED					BIT(0)
#define ETH_PHY_HI_BER						BIT(1)

/* 0x078: PHY pcs error injection */
#define ETH_PHY_INJ_ERROR					0x000FFFFF

/* 0x088: PHY alignment marker lock */
#define ETH_PHY_AM_LOCK						BIT(0)

/* 0x08C: PHY RX PCS deskew status */
#define ETH_PHY_RX_PCS_LANES_DESKEWED				BIT(0)

/* 0x090: PHY BER count */
#define ETH_PHY_BER_CNT						0xffffffff

/* 0x094: PHY virtual lane 0 */
#define ETH_PHY_VLANE_0_vlan0					0x1F
#define ETH_PHY_VLANE_0_vlan1					(0x1F << 5)
#define ETH_PHY_VLANE_0_vlan2					(0x1F << 10)
#define ETH_PHY_VLANE_0_vlan3					(0x1F << 15)
#define ETH_PHY_VLANE_0_vlan4					(0x1F << 20)
#define ETH_PHY_VLANE_0_vlan5					(0x1F << 25)

/* 0x098: PHY virtual lane 1 */
#define ETH_PHY_VLANE_1_vlan6					0x1F
#define ETH_PHY_VLANE_1_vlan7					(0x1F << 5)
#define ETH_PHY_VLANE_1_vlan8					(0x1F << 10)
#define ETH_PHY_VLANE_1_vlan9					(0x1F << 15)
#define ETH_PHY_VLANE_1_vlan10					(0x1F << 20)
#define ETH_PHY_VLANE_1_vlan11					(0x1F << 25)

/* 0x09C: PHY virtual lane 2 */
#define ETH_PHY_VLANE_2_vlan12					0x1F
#define ETH_PHY_VLANE_2_vlan13					(0x1F << 5)
#define ETH_PHY_VLANE_2_vlan14					(0x1F << 10)
#define ETH_PHY_VLANE_2_vlan15					(0x1F << 15)
#define ETH_PHY_VLANE_2_vlan16					(0x1F << 20)
#define ETH_PHY_VLANE_2_vlan17					(0x1F << 25)

/* 0x0A0: PHY virtual lane 3 */
#define ETH_PHY_VLANE_3_vlan18					0x1F
#define ETH_PHY_VLANE_3_vlan19					(0X1F << 5)

/* 0x000: MAC Config control register */
#define ETH_MAC_CONFIG_CTRL_RX_SHADOW_REQ			BIT(1)
#define ETH_MAC_CONFIG_CTRL_TX_SHADOW_REQ			BIT(0)

/* 0x010: PHY TX PLD Configuration */
#define ETH_PHY_TX_PLD_CONF_MAC					0
#define ETH_PHY_TX_PLD_CONF_MAC_PTP				1
#define ETH_PHY_TX_PLD_CONF_PCS_MII				2
#define ETH_PHY_TX_PLD_CONF_OTN					3
#define ETH_PHY_TX_PLD_CONF_PCS66				4
#define ETH_PHY_TX_PLD_CONF_PMA_DIR				5
#define ETH_PHY_TX_PLD_CONF_PTP_ONLY				6
#define ETH_PHY_TX_PLD_CONF_FEC_DIR				7

/* 0x014: PHY RX PLD Configuration */
#define ETH_PHY_RX_PLD_CONF_EHIP_MODE_MASK			3
#define ETH_PHY_RX_PLD_CONF_EHIP_MODE_SHIFT			0
#define ETH_PHY_RX_PLD_CONF_EHIP_MODE_MAC			0
#define ETH_PHY_RX_PLD_CONF_EHIP_MODE_MAC_PTP			1
#define ETH_PHY_RX_PLD_CONF_EHIP_MODE_PCS_MII			2
#define ETH_PHY_RX_PLD_CONF_EHIP_MODE_OTN			3
#define ETH_PHY_RX_PLD_CONF_EHIP_MODE_PCS66			4
#define ETH_PHY_RX_PLD_CONF_EHIP_MODE_PMA_DIR			5
#define ETH_PHY_RX_PLD_CONF_EHIP_MODE_FEC_DIR			7
#define ETH_PHY_RX_PLD_CONF_RXPLDMUX_PCS			BIT(5)
#define ETH_PHY_RX_PLD_CONF_RXPLDMUX_PCS66			BIT(6)
#define ETH_PHY_RX_PLD_CONF_RXPLDMUX_SEL_MASK			(3 << 8)
#define ETH_PHY_RX_PLD_CONF_RXPLDMUX_SEL_SHIFT			8
#define ETH_PHY_RX_PLD_CONF_RXPLDMUX_SEL_RX_MAC			0
#define ETH_PHY_RX_PLD_CONF_RXPLDMUX_SEL_RX_PCS			1
#define ETH_PHY_RX_PLD_CONF_RXPLDMUX_SEL_RX_FEC			2
#define ETH_PHY_RX_PLD_CONF_RXPLDMUX_SEL_RX_PMA			3
#define ETH_PHY_RX_PLD_CONF_RXPLDMUX_SEL_TX_FEC			4
#define ETH_PHY_RX_PLD_CONF_RXPLDMUX_SEL_TX_PCS			5
#define ETH_PHY_RX_PLD_CONF_RXPLDMUX_SEL_TX_MAC			6
#define ETH_PHY_RX_PLD_CONF_RXPLDMUX_SEL_TX_EMIB		7

/* 0x1C0: PHY TX PLD status */
#define ETH_PHY_TX_PLD_STAT_EVAL_DONE				BIT(0)
#define ETH_PHY_TX_PLD_STAT_DESKW				(0x7 << 1)
#define ETH_PHY_TX_PLD_STAT_MONITOR_ERR				(0x3F << 8)
#define ETH_PHY_TX_PLD_STAT_DSK_ACT_CHAN			(0x3F << 16)
#define ETH_PHY_TX_PLD_STAT_FIFO_UNDERFLOW			BIT(22)
#define ETH_PHY_TX_PLD_STAT_FIFO_EMPTY				BIT(23)
#define ETH_PHY_TX_PLD_STAT_FIFO_OVERFLOW			BIT(24)

/* 0x048: PHY EHIP PCS Mode Configuration */
#define ETH_PHY_SCLR_FRAME_ERROR				BIT(12)

/* 0x04C: PHY RX PCS */
#define ETH_PHY_RX_PCS_AM_INTVL					0x3FFF
#define ETH_PHY_RX_PCS_RX_PCS_MAX_SKEW				(0x3F << 14)
#define ETH_PHY_RX_PCS_USE_HIBER_MON				BIT(20)

/* 0x0A4 - 0x0F0: PHY BIP counter 0 - 19 */
#define ETH_PHY_BIP_CNT_x					0x0000FFFF

/* 0x070: PHY hiber checks */
#define ETH_PHY_HIBER_CHECKS					0x001FFFFF

/* 0x074: PHY hiber frame error */
#define ETH_PHY_HIBER_FRM_ERR					0x7F

/* 0x0F4: PHY error block count */
#define ETH_PHY_ERR_BLOCK_COUNT					0xffffffff

/* 0x0FC - 0x108: PHY BIP deskew dept 0 - 3 */
#define ETH_PHY_DES_DEPT_0					0x3F
#define ETH_PHY_DES_DEPT_1					(0x3F << 6)
#define ETH_PHY_DES_DEPT_2					(0x3F << 12)
#define ETH_PHY_DES_DEPT_3					(0x3F << 18)
#define ETH_PHY_DES_DEPT_4					(0x3F << 24)

/* 0x10C: PHY RX PCS test error count */
#define ETH_PHY_RX_PCS_TEST_ERR_CNT				0xFFFFFFFF

/* 0x110: PHY RX PCS bitslip count */
#define ETH_PHY_RX_PCS_BITSLIP_CNT				0x0000007F
#define ETH_PHY_RX_PCS_DLPULSE_ALIGNED				BIT(7)

/* 0x114 - 0x160: ptp_vl_data_lsb */
#define ETH_PHY_PTP_LSB_GBSTATE_MASK				(7 << 0)
#define ETH_PHY_PTP_LSB_GBSTATE_SHIFT				0
#define ETH_PHY_PTP_LSB_AL_BLK_PHASE_MASK			(3 << 3)
#define ETH_PHY_PTP_LSB_AL_BLK_PHASE_SHIFT			3
#define ETH_PHY_PTP_LSB_AL_POS_MASK				(0x1F << 5)
#define ETH_PHY_PTP_LSB_AL_POS_SHIFT				5
#define ETH_PHY_PTP_LSB_AM_COUNT_MASK				(0x3FFF << 10)
#define ETH_PHY_PTP_LSB_AM_COUNT_SHIFT				10
#define ETH_PHY_PTP_LSB_LOCAL_VL_MASK				(0x1F << 24)
#define ETH_PHY_PTP_LSB_LOCAL_VL_SHIFT				24
#define ETH_PHY_PTP_LSB_SPARE_MASK				(7 << 29)
#define ETH_PHY_PTP_LSB_SPARE_SHIFT				29

/* 0x164 - 0x1B0: ptp_vl_data_msb */
#define ETH_PHY_PTP_MSB_REMOTE_VL_NUM_MASK			(0x1F << 0)
#define ETH_PHY_PTP_MSB_REMOTE_VL_NUM_SHIFT			0
#define ETH_PHY_PTP_MSB_LOCAL_PL_MASK				(3 << 5)
#define ETH_PHY_PTP_MSB_LOCAL_PL_SHIFT				5
#define ETH_PHY_PTP_MSB_LOCAL_VL_MASK				(0x1F << 8)
#define ETH_PHY_PTP_MSB_LOCAL_VL_SHIFT				8
#define ETH_PHY_PTP_MSB_BLK_ALIGN_OCC_MASK			(7 << 13)
#define ETH_PHY_PTP_MSB_BLK_ALIGN_OCC_SHIFT			13
#define ETH_PHY_PTP_MSB_AM_DETECT_OCC_MASK			(7 << 16)
#define ETH_PHY_PTP_MSB_AM_DETECT_OCC_SHIFT			16
#define ETH_PHY_PTP_MSB_GB66TO110_OCC_MASK			(7 << 19)
#define ETH_PHY_PTP_MSB_GB66TO110_OCC_SHIFT			19
#define ETH_PHY_PTP_MSB_GB33TO66_OCC_MASK			(3 << 22)
#define ETH_PHY_PTP_MSB_GB33TO66_OCC_SHIFT			22
#define ETH_PHY_PTP_MSB_AL_POS_50G_MASK				(0x7F << 24)
#define ETH_PHY_PTP_MSB_AL_POS_50G_SHIFT			24

/* 0x1B4: PTP Last AM Lane */
#define ETH_PHY_PTP_LAL_COUNT					0x1F

/* 0x200: TX MAC link fault configuiration  */
#define ETH_TX_MAC_LF_EN					BIT(0)
#define ETH_TX_MAC_UNIDIR_EN					BIT(1)
#define ETH_TX_MAC_DISABLE_RF					BIT(2)
#define ETH_TX_MAC_FORCE_RF					BIT(3)

/* 0x204: TX MAC IPG col rem  */
#define ETH_TX_MAC_IPG_COL_REM					0xFFFF

/* 0x208: TX MAC frame size  */
#define ETH_TX_MAC_MAX_TX					0xFFFF

/* 0x20C: TX MAC configuration  */
#define ETH_TX_MAC_ENABLE_S_ADDR_EN				BIT(3)
#define ETH_TX_MAC_DISABLE_TXMAC				BIT(2)
#define ETH_TX_MAC_DISABLE_TXVLAN				BIT(1)

/* 0x210: TX MAC EHIP configuration  */
#define ETH_TX_MAC_EHIP_CONF_EN_PP				BIT(0)
#define ETH_TX_MAC_EHIP_CONF_IPG				(0x3 << 1)
#define ETH_TX_MAC_EHIP_CONF_TX_PLEN_EN				BIT(4)
#define ETH_TX_MAC_EHIP_CONF_TXCRC_PREAMBLE			BIT(9)

/* 0x214: TX MAC source address lower bytes  */
#define ETH_TX_MAC_LOW_BYTES					0xffffffff

/* 0x218: TX MAC source address higher bytes  */
#define ETH_TX_MAC_HIGH_BYTES					0x0000ffff

/* 0x21C: RX MAC frame size  */
#define ETH_RX_MAC_FRAME_SIZE					0x0000ffff

/* 0x220: RX MAC CRC forwarding */
#define ETH_RX_MAC_CRC_FORWARD					BIT(0)

/* 0x224: RX MAC configuration */
#define ETH_RX_MAC_EN_PLEN					BIT(0)
#define ETH_RX_MAC_RXVLAN_DISABLE				BIT(1)
#define ETH_RX_MAC_EN_CHECK_SFD					BIT(3)
#define ETH_RX_MAC_EN_STR_PREAMBLE				BIT(4)
#define ETH_RX_MAC_ENFORCE_MAX_RX				BIT(7)
#define ETH_RX_MAC_REMOVE_RX_PAD				BIT(8)

/* 0x228: RX MAC feature configuration */
#define ETH_RX_MAC_EN_PP					BIT(0)
#define ETH_RX_MAC_RXCRC_PREAMBLE				BIT(1)
#define ETH_RX_MAC_MAC_LB_MODE_MASK				(3 << 2)
#define ETH_RX_MAC_MAC_LB_MODE_SHIFT				2

/* Ethernet Reconfiguration Interface
 * Pause and Priority - Based Flow Control
 * Bit Definitions
 */
/* 0x22C Enable TX Pause Ports */
#define ETH_EN_PFC_PORT_FOR_PFC					0xFF
#define ETH_EN_PFC_PORT_FOR_PAUSE				BIT(8)

/* 0x230 TX Pause Request */
#define ETH_TX_PAUSE_REQ_FOR_PFC				0xFF
#define ETH_TX_PAUSE_REQ_FOR_PAUSE				BIT(8)

/* 0x234 Enable Automatic TX Pause Retransmission */
#define ETH_TX_EN_HOLDOFF_FOR_PFC				0xFF
#define ETH_TX_EN_HOLDOFF_FOR_PAUSE				BIT(8)

/* 0x238 Retransmit Holdoff Quanta Fields */
#define ETH_TX_RETRANSMIT_HOLDOFF_QUANTA			0xFFFF

/* 0x23C Retransmit Pause Quanta */
#define ETH_RETRANSMIT_PAUSE_QUANTA				0xFFFF

/* 0x240 Enable TX XOFF */
#define ETH_TX_XOFF						0x7

/* 0x244 Enable Uniform Holdoff */
#define ETH_EN_UNIFORM_HOLDOFF					BIT(0)

/* 0x248 Set Uniform Holdoff */
#define ETH_SET_UNIFORM_HOLDOFF					0xFFFF

/* 0x24C Lower 4 Bytes of the Dest Addr for Flow Control Fields */
#define ETH_TX_DEST_ADDR_FLOW_CTRL_LO				0xFFFFFFFF

/* 0x250 Higher 2 Bytes of the Dest Addr for Flow Control Fields */
#define ETH_TX_DEST_ADDR_FLOW_CTRL_HI				0xFFFF

/* 0x254 Lower 4 Bytes of the Src Addr for Flow Control Fields */
#define ETH_TX_SRC_ADDR_FLOW_CTRL_LO				0xFFFFFFFF

/* 0x258 Higher 2 Bytes of the Src Addr for Flow Control Fields */
#define ETH_TX_SRC_ADDR_FLOW_CTRL_HI				0xFFFF

/* 0x25C TX Flow Control Feature Configuration */
#define ETH_TX_EN_STD_FLOW_CTRL					BIT(0)
#define ETH_TX_EN_PRIORITY_FLOW_CTRL				BIT(1)

/* 0x260 Enable RX Pause Frame Processing Fields */
#define ETH_EN_RX_PAUSE(queue)					BIT(queue)

/* 0x264 RX Forward Flow Control Frames */
#define ETH_RX_PAUSE_FWD					BIT(0)

/* 0x268 Lower 4 bytes of Dest. Addr. for RX Pause Frames */
#define ETH_RX_DEST_ADDR_FLOW_CTRL_LO				0xFFFFFFFF

/* 0x26C Higher 2 bytes of the Dest Addr for RX Pause Frames */
#define ETH_RX_DEST_ADDR_FLOW_CTRL_HI				0xFFFF

/* 0x270 RX Flow Control Feature Configuration */
#define ETH_RX_EN_STD_FLOW_CTRL					BIT(0)
#define ETH_RX_EN_PRIORITY_FLOW_CTRL				BIT(1)

/* 0x284 Pause Quanta 0 */
#define ETH_PAUSE_QUANTA_0					0xFFFF

/* 0x288 Pause Quanta 1 */
#define ETH_PAUSE_QUANTA_1					0xFFFF

/* 0x28C Pause Quanta 2 */
#define ETH_PAUSE_QUANTA_2					0xFFFF

/* 0x290 Pause Quanta 3 */
#define ETH_PAUSE_QUANTA_3					0xFFFF

/* 0x294 Pause Quanta 4 */
#define ETH_PAUSE_QUANTA_4					0xFFFF

/* 0x298 Pause Quanta 5 */
#define ETH_PAUSE_QUANTA_5					0xFFFF

/* 0x29C Pause Quanta 6 */
#define ETH_PAUSE_QUANTA_6					0xFFFF

/* 0x2A0 Pause Quanta 7 */
#define ETH_PAUSE_QUANTA_7					0xFFFF

/* 0x2A4 PFC Holdoff Quanta 0 */
#define ETH_PFC_HOLDOFF_QUANTA_0				0xFFFF

/* 0x2A8 PFC Holdoff Quanta 1 */
#define ETH_PFC_HOLDOFF_QUANTA_1				0xFFFF

/* 0x2AC PFC Holdoff Quanta 2 */
#define ETH_PFC_HOLDOFF_QUANTA_2				0xFFFF

/* 0x2B0 PFC Holdoff Quanta 3 */
#define ETH_PFC_HOLDOFF_QUANTA_3				0xFFFF

/* 0x2B4 PFC Holdoff Quanta 4 */
#define ETH_PFC_HOLDOFF_QUANTA_4				0xFFFF

/* 0x2B8 PFC Holdoff Quanta 5 */
#define ETH_PFC_HOLDOFF_QUANTA_5				0xFFFF

/* 0x2BC PFC Holdoff Quanta 6 */
#define ETH_PFC_HOLDOFF_QUANTA_6				0xFFFF

/* 0x2C0 PFC Holdoff Quanta 7 */
#define ETH_PFC_HOLDOFF_QUANTA_7				0xFFFF

/* Ethernet Reconfiguration Interface
 * TX Statistics Counter
 * Bit Definitions
 */
/* 0x274: Configuration of TX Statistics Counters */
#define ETH_TX_CNTR_CFG_RST_ALL					BIT(0)
#define ETH_TX_CNTR_CFG_RST_PARITY_ERR				BIT(1)

/* Ethernet Reconfiguration Interface
 * RX Statistics Counter
 * Bit Definitions
 */
/* 0x278: Configuration of RX Statistics Counters */
#define ETH_RX_CNTR_CFG_RST_ALL					BIT(0)
#define ETH_RX_CNTR_CFG_RST_PARITY_ERR				BIT(1)

/* 0x946: Status of RX Statistics Counters */
#define ETH_RX_CNTR_STAT_PARITY_ERR				BIT(0)
#define ETH_RX_CNTR_STAT_CSR					BIT(1)

/* Ethernet Reconfiguration Interface
 * 1588 PTP
 * Bit Definitions
 */
/* 0x2E0: TX 1588 PTP Extra Latency */
#define ETH_TX_PTP_EXTRA_LATENCY_FRAC_NS			0xFFFF
#define ETH_TX_PTP_EXTRA_LATENCY_NS				(0x7FFF << 16)
#define ETH_TX_PTP_EXTRA_LATENCY_SIGN				BIT(31)

/* 0x2E4: TX 1588 PTP UI */
#define ETH_TX_UI_FRAC_NS					0xFFFFFF
#define ETH_TX_UI_NS						(0xFF << 24)

/* 0x2EC: TX PTP XCVR physical lane number */
#define ETH_TX_PTP_PHY_LANE_NUM					0x3F

/* 0x2F0: TX PTP Asyncronous Pulse Filter */
#define ETH_TX_PTP_AP_FILTER					0xFFFF

/* 0x344: RX 1588 PTP Extra Latency */
#define ETH_RX_PTP_EXTRA_LATENCY_FRAC_NS			0xFFFF
#define ETH_RX_PTP_EXTRA_LATENCY_NS				(0x7FFF << 16)
#define ETH_RX_PTP_EXTRA_LATENCY_SIGN				BIT(31)

/* 0x348: RX 1588 PTP UI */
#define ETH_RX_UI_FRAC_NS					0xFFFFFF
#define ETH_RX_UI_NS						(0xFF << 24)

/* 0x350: RX PTP PMA Physical Lane Number */
#define ETH_RX_PTP_PHY_LANE_NUM					0x3F

/* 0x354: RX PTP Asyncronous Pulse Filter */
#define ETH_RX_PTP_AP_FILTER					0xFFFF

/* 0x3A8 - 0x3F4: RX PTP VL to PL mapping */
#define ETH_RX_PTP_VL_TO_PL					0x1F

/* 0x3F8: RX n-timestamp counter */
#define ETH_RX_PKT_N_TS_RX_CTR					0xFFFFF

/* 0xA58: TX PTP Correction field overflow status */
#define ETH_TX_PTP_CF_OVERFLOW					BIT(1)

/* 0x080C: PTP reference lane */
#define ETH_PTP_TX_REF_LANE_MASK				(3 << 0)
#define ETH_PTP_TX_REF_LANE_SHIFT				0
#define ETH_PTP_RX_REF_LANE_MASK				(3 << 3)
#define ETH_PTP_RX_REF_LANE_SHIFT				3

/* 0x0810: PTP TX user configuration status */
#define ETH_PTP_TX_EHIP_PREAMPLE_PASSTHROUGH			BIT(0)

/* 0x0814: PTP TX user configuration status */
#define ETH_PTP_TX_USER_CFG_DONE				BIT(0)

/* 0x0818: PTP RX user configuration status */
#define ETH_PTP_RX_USER_CFG_DONE				BIT(0)
#define ETH_PTP_RX_FEC_CW_POS_CFG_DONE				BIT(1)

/* 0x081C: Time Value Control*/
#define ETH_TX_TAM_SNAPSHOT					BIT(0)
#define ETH_RX_TAM_SNAPSHOT					BIT(1)

/* 0x0820: TX TAM Lower */
#define ETH_TX_TAM_LO_FRAC_NS					0xFFFF
#define ETH_TX_TAM_LO_NS					(0xFFFF << 16)

/* 0x0824: TX TAM Upper */
#define ETH_TX_TAM_HI_NS					0xFFFF

/* 0x0824: TX TAM Count / valid */
#define ETH_TX_TAM_CNT_SHIFT					16
#define ETH_TX_TAM_CNT_MASK					(0x7FFF << 16)
#define ETH_TX_TAM_VALID					BIT(31)

/* 0x0828: RX TAM Lower */
#define ETH_RX_TAM_LO_FRAC_NS					0xFFFF
#define ETH_RX_TAM_LO_NS					(0xFFFF << 16)

/* 0x082C: RX TAM Upper */
#define ETH_RX_TAM_HI_NS					0xFFFF

/* 0x082C: RX TAM Count / valid */
#define ETH_RX_TAM_CNT_SHIFT					16
#define ETH_RX_TAM_CNT_MASK					(0x7FFF << 16)
#define ETH_RX_TAM_VALID					BIT(31)

/* 0x0830: PTP Status */
#define ETH_TX_PTP_OFFSET_DATA_VALID				BIT(0)
#define ETH_RX_PTP_OFFSET_DATA_VALID				BIT(1)
#define ETH_TX_PTP_READY					BIT(2)
#define ETH_RX_PTP_READY					BIT(3)

/* 0x0840: PTP Status2 (internal use) */
#define ETH_TX_CALC_DATA_TIME_VALID				BIT(1)
#define ETH_TX_CALC_DATA_WIREDELAY_VALID			BIT(2)
#define ETH_RX_CALC_DATA_OFFSET_VALID				BIT(3)
#define ETH_RX_CALC_DATA_TIME_VALID				BIT(4)
#define ETH_RX_CALC_DATA_WIREDELAY_VALID			BIT(5)
#define ETH_RX_VL_OFFSET_DATA_READY				BIT(6)

/* RSFEC Reconfiguration Interface
 * TX & RX RSFEC
 * Bit Definitions
 */
/* 0x600C0: RS-FEC Settings */
#define RSFEC_E25G_S0_TOP_FEC_DATA_MUX				BIT(0)
#define RSFEC_E25G_S0_TOP_FEC_LPBK_EN				BIT(1)
#define RSFEC_E25G_S0_TOP_FEC_BYPASS_EN				BIT(2)

/* 0x600C8: RS-FEC per lane configuration */
#define RSFEC_E25G_S0_LANE_CFG0_INDIC_BYP			BIT(8)
#define RSFEC_E25G_S0_LANE_CFG0_CORR_BYP			BIT(9)

/* 0x6138: RS-FEC Error Injection Mode */
#define RSFEC_E25G_S0_ERR_INJ_TX_RATE				0xFF
#define RSFEC_E25G_S0_ERR_INJ_TX_PAT				(0xFF << 8)

/* 0x6148: RSFEC Status per TX Lane */
#define RSFEC_TX_STATUS_LANE_HDR_INV				BIT(0)
#define RSFEC_TX_STATUS_LANE_BLK_INV				BIT(1)
#define RSFEC_TX_STATUS_LANE_RESYNC				BIT(2)
#define RSFEC_TX_STATUS_LANE_PACE_INV				BIT(3)
#define RSFEC_TX_STATUS_LANE_AM_SF_TX_IN			(3 << 4)

/* 0x614C: RSFEC per TX Lane Hold Status*/
#define RSFEC_TX_HOLD_STATUS_LANE_HDR_INV			BIT(0)
#define RSFEC_TX_HOLD_STATUS_LANE_BLK_INV			BIT(1)
#define RSFEC_TX_HOLD_STATUS_LANE_RESYNC			BIT(2)
#define RSFEC_TX_HOLD_STATUS_LANE_PACE_INV			BIT(3)
#define RSFEC_TX_HOLD_STATUS_LANE_AM_SF_TX_IN			(3 << 4)

/* 0x6154: RSFEC Status per RX Lane */
#define RSFEC_RX_STATUS_LANE_SIG_FAIL				BIT(0)
#define RSFEC_RX_STATUS_LANE_NOT_LOCKED				BIT(1)
#define RSFEC_RX_STATUS_LANE_FEC_3BAD				BIT(2)
#define RSFEC_RX_STATUS_LANE_AM_5BAD				BIT(3)
#define RSFEC_RX_STATUS_LANE_HI_SER				BIT(4)
#define RSFEC_RX_STATUS_LANE_CORR_CW				BIT(5)
#define RSFEC_RX_STATUS_LANE_UNCORR_CW				BIT(6)
#define RSFEC_RX_STATUS_LANE_CORR_CW_BINS_LO			(0xFF << 8)
#define RSFEC_RX_STATUS_LANE_CORR_CW_BINS_HI			(0xFF << 16)
#define RSFEC_RX_STATUS_LANE_DEGR_SER				BIT(24)
#define RSFEC_RX_STATUS_LANE_AM_SF_RX_IN			(7 << 25)

/* 0x6158: RSFEC per RX Lane Hold Status */
#define RSFEC_RX_HOLD_STATUS_LANE_SIG_FAIL			BIT(0)
#define RSFEC_RX_HOLD_STATUS_LANE_NOT_LOCKED			BIT(1)
#define RSFEC_RX_HOLD_STATUS_LANE_FEC_3BAD			BIT(2)
#define RSFEC_RX_HOLD_STATUS_LANE_AM_5BAD			BIT(3)
#define RSFEC_RX_HOLD_STATUS_LANE_HI_SER			BIT(4)
#define RSFEC_RX_HOLD_STATUS_LANE_CORR_CW			BIT(5)
#define RSFEC_RX_HOLD_STATUS_LANE_UNCORR_CW			BIT(6)
#define RSFEC_RX_HOLD_STATUS_LANE_CORR_CW_BINS_LO		(0xFF << 8)
#define RSFEC_RX_HOLD_STATUS_LANE_CORR_CW_BINS_HI		(0xFF << 16)
#define RSFEC_RX_HOLD_STATUS_LANE_DEGR_SER			BIT(24)
#define RSFEC_RX_HOLD_STATUS_LANE_AM_SF_RX_IN			(7 << 25)

/* 0x6160: RS-FEC per Aggregate Rx Status */
#define RSFEC_AGGR_RX_STATUS_NOT_ALIGN				BIT(0)
#define RSFEC_AGGR_RX_STATUS_NOT_DESKEW				BIT(1)
#define RSFEC_AGGR_RX_STATUS_LAST_LANE				(0xF << 2)

/* 0x6164: RS-FEC per Aggregate Rx Hold */
#define RSFEC_AGGR_RX_HOLD_NOT_ALIGN				BIT(0)
#define RSFEC_AGGR_RX_HOLD_NOT_DESKEW				BIT(1)

/* 0x616C: RS-FEC FEC Lane Mapping RX */
#define RSFEC_LN_MAPPING_RX_FEC_LANE				0xF

/* 0x6170: RS-FEC FEC Lane Skew RX */
#define RSFEC_LN_SKEW_RX_SKEW					0xFF

/* 0x6174: RS-FEC Codeword Bit Position on RX */
#define RSFEC_CW_POS_RX_num					0x7FFF

/* 0x6178: RSFEC SRAM ECC Status Hold */
#define RSFEC_SRAM_ECC_HOLD_STATUS_SBE				0xFF
#define RSFEC_SRAM_ECC_HOLD_STATUS_MBE				(0xFF << 8)

/* 0x617C: RSFEC per Lane Error Injection Status */
#define RSFEC_ERR_VAL_TX_INJ0S					0xFF
#define RSFEC_ERR_VAL_TX_INJ1S					(0xFF << 8)

/* 0x61D0: RSFEC number of corrected CWs having 0, 1..15 errored symbols */
#define RSFEC_CORR_CWBIN_CNT_0_STAT				0xFF
#define RSFEC_CORR_CWBIN_CNT_1_STAT				(0xFF << 8)
#define RSFEC_CORR_CWBIN_CNT_2_STAT				(0xFF << 16)
#define RSFEC_CORR_CWBIN_CNT_3_STAT				(0xFF << 24)

/* 0x61D4: RSFEC number of corrected CWs having 0, 1..15 errored symbols */
#define RSFEC_CORR_CWBIN_CNT_4_STAT				0xFF
#define RSFEC_CORR_CWBIN_CNT_5_STAT				(0xFF << 8)
#define RSFEC_CORR_CWBIN_CNT_6_STAT				(0xFF << 16)
#define RSFEC_CORR_CWBIN_CNT_7_STAT				(0xFF << 24)

/* 0x61D8: RSFEC number of corrected CWs having 0, 1..15 errored symbols */
#define RSFEC_CORR_CWBIN_CNT_8_STAT				0xFF
#define RSFEC_CORR_CWBIN_CNT_9_STAT				(0xFF << 8)
#define RSFEC_CORR_CWBIN_CNT_10_STAT				(0xFF << 16)
#define RSFEC_CORR_CWBIN_CNT_11_STAT				(0xFF << 24)

/* 0x61DC: RSFEC number of corrected CWs having 0, 1..15 errored symbols */
#define RSFEC_CORR_CWBIN_CNT_12_STAT				0xFF
#define RSFEC_CORR_CWBIN_CNT_13_STAT				(0xFF << 8)
#define RSFEC_CORR_CWBIN_CNT_14_STAT				(0xFF << 16)
#define RSFEC_CORR_CWBIN_CNT_15_STAT				(0xFF << 24)

/* 0x61E0: RSFEC Extra config and debug register for fec_clock */
#define RSFEC_DEBUG_CFG_SHADOW_REQ				BIT(0)
#define RSFEC_DEBUG_CFG_SHADOW_CLEAR				BIT(1)

/* Transceiver Reconfiguration Interface
 * PMA/FEC
 * Bit Definitions
 */
/* 0x800: XCVR PMA Device and IP variant */
#define XCVR_PMA_LINE_RATE					0xFFF
#define XCVR_PMA_TYPE						BIT(12)
#define XCVR_PMA_TYPE_FGT					(0 << 12)
#define XCVR_PMA_TYPE_FHT					(1 << 12)
#define XCVR_PMA_MODULATION_TYPE				BIT(13)
#define XCVR_PMA_MODULATION_TYPE_NRZ				(0 << 13)
#define XCVR_PMA_MODULATION_TYPE_PAM4				(1 << 13)
#define XCVR_PMA_MODE						0xC000
#define XCVR_PMA_MODE_RX_SIMPLEX				(1 << 14)
#define XCVR_PMA_MODE_TX_SIMPLEX				(2 << 14)
#define XCVR_PMA_MODE_DUPLEX					(3 << 14)
#define XCVR_PMA_NUM_LANES					0x1F0000
#define XCVR_PMA_NUM_LANES_SHIFT				16
#define XCVR_PMA_FEC_ENABLE					BIT(21)
#define XCVR_PMA_NUM_EMIBS					0x1F000000
#define XCVR_PMA_NUM_EMIBS_SHIFT				24

/* 0x808: XCVR PMA PHY Soft Reset register */
#define XCVR_PMA_PHY_RESET_SOFT_TX_RST				BIT(0)
#define XCVR_PMA_PHY_RESET_SOFT_RX_RST				BIT(1)
#define XCVR_PMA_PHY_RESET_TX_RST_OVR				BIT(4)
#define XCVR_PMA_PHY_RESET_RX_RST_OVR				BIT(5)

/* 0x80C: XCVR PMA PHY Reset status register */
#define XCVR_PMA_PHY_RESET_STATUS_TX_RST_ACK_N			BIT(0)
#define XCVR_PMA_PHY_RESET_STATUS_RX_RST_ACK_N			BIT(1)
#define XCVR_PMA_PHY_RESET_STATUS_TX_READY			BIT(4)
#define XCVR_PMA_PHY_RESET_STATUS_RX_READY			BIT(5)
#define XCVR_PMA_PHY_RESET_STATUS_AVMM_READY			BIT(7)

/* 0x810: XCVR PMA PHY TX PLL locked register */
#define XCVR_PMA_PHY_TX_PLL_LOCKED				0xFFFF

/* 0x814: XCVR PMA PHY RX CDR PLL locked register */
#define XCVR_PMA_PHY_RX_CDR_LOCKED2REF				0x0000FFFF
#define XCVR_PMA_PHY_RX_CDR_LOCKED2DATA				0xFFFF0000

/* 0x818: XCVR PMA source control register */
#define XCVR_PMA_SRC_CTRL_RX_IGNORE_LOCKED2DATA			BIT(0)

/* 0x4003C: XCVR Serdes FHT provide RX ready status */
#define XCVR_SRDS_FHT_RXSRDS_RDY				BIT(0)

/* 0x40740: XCVR Serdes FGT LC medium post divider */
#define XCVR_SRDS_CFG_SYNTHLCMEDPCS_POSTDIV_P5_LOCOVR_MUXD0	BIT(31)

/* 0x40744: XCVR Serdes FGT LC medium post divider */
#define XCVR_SRDS_CFG_SYNTHLCMEDPCS_POSTDIV_LOCOVR_MUXD0_SHIFT	7
#define XCVR_SRDS_CFG_SYNTHLCMEDPCS_POSTDIV_LOCOVR_MUXD0_MASK	0x3F80

/* 0x40840: XCVR Serdes FGT LC slow post divider */
#define XCVR_SRDS_CFG_SYNTHLCSLOWPCS_POSTDIV_P5_LOCOVR_MUXD0	BIT(30)

/* 0x40844: XCVR Serdes FGT LC slow post divider */
#define XCVR_SRDS_CFG_SYNTHLCSLOWPCS_POSTDIV_LOCOVR_MUXD0_SHIFT	7
#define XCVR_SRDS_CFG_SYNTHLCSLOWPCS_POSTDIV_LOCOVR_MUXD0_MASK	0x3F80

/* 0x40994: XCVR Serdes FGT LC fast post divider */
#define XCVR_SRDS_CFG_SYNTHLCFASPCS_POSTDIV_P5_LOCOVR_MUXD0	BIT(30)

/* 0x40998: XCVR Serdes FGT LC fast post divider */
#define XCVR_SRDS_CFG_SYNTHLCFASTPCS_POSTDIV_LOCOVR_MUXD0_SHIFT	21
#define XCVR_SRDS_CFG_SYNTHLCFASTPCS_POSTDIV_LOCOVR_MUXD0_MASK	(0x7F << 21)

/* 0x41428: XCVR Serdes FGT RX/TX polarity */
#define XCVR_SRDS_CFG_POLARITY_RX				BIT(6)
#define XCVR_SRDS_CFG_POLARITY_TX				BIT(7)

/* 0x415B0: XCVR Serdes FGT RX bitflip */
#define XCVR_SRDS_CFG_RXPAM_BITORDER_SHIFT			24
#define XCVR_SRDS_CFG_RXPAM_BITORDER_MASK			(7 << 24)
#define XCVR_SRDS_CFG_RXPCS_POSTDIV_P5_MUXD0			BIT(31)

/* 0x41768: XCVR Serdes FGT TX bitflip */
#define XCVR_SRDS_CFG_TXPAM_BITORDER_SHIFT			28
#define XCVR_SRDS_CFG_TXPAM_BITORDER_MASK			(7 << 28)

/* 0x41BB8: XCVR Serdes FGT RX EQ High Freq Boost */
#define XCVR_SRDS_CFG_RXEQSET_STATIC_HIFREQAGCRES_SHIFT		19
#define XCVR_SRDS_CFG_RXEQSET_STATIC_HIFREQAGCRES_MASK		(0x3F << 19)
#define XCVR_SRDS_CFG_RXEQSET_STATIC_HIFREQVGAGAIN_SHIFT	25
#define XCVR_SRDS_CFG_RXEQSET_STATIC_HIFREQVGAGAIN_MASK		(0x7F << 25)

/* 0x42000: XCVR Serdes FHT provide RX lock to reference status */
#define XCVR_SRDS_CFG_CDR_LOCK2REF				BIT(1)

/* 0x428DC: XCVR Serdes FHT control RX BER Counter */
#define XCVR_SRDS_CFG_DFT_BER_CNT_EN				BIT(0)
#define XCVR_SRDS_CFG_DFT_BER_CNT_MODE_SHIFT			1
#define XCVR_SRDS_CFG_DFT_BER_CNT_MODE_MASK			(0x3 << 1)

/* 0x428E0: XCVR Serdes FHT trigger RX BER Counter */
#define XCVR_SRDS_CFG_DFT_BER_CLEAR				BIT(0)
#define XCVR_SRDS_CFG_DFT_BER_START				BIT(1)
#define XCVR_SRDS_CFG_DFT_BER_STOP				BIT(2)

/* 0x428F4: XCVR Serdes FHT provide RX pattern lock status */
#define XCVR_SRDS_DFT_BER_RX_LOCK				BIT(1)

/* 0x42930: XCVR Serdes FHT control PRBS verifier (RX) */
#define XCVR_SRDS_CFG_DFT_RX_PRBS_COMMON_EN			BIT(0)
#define XCVR_SRDS_CFG_DFT_RX_PRBS_SEL_PRBS7			(0 << 1)
#define XCVR_SRDS_CFG_DFT_RX_PRBS_SEL_PRBS9			(1 << 1)
#define XCVR_SRDS_CFG_DFT_RX_PRBS_SEL_PRBS11			(2 << 1)
#define XCVR_SRDS_CFG_DFT_RX_PRBS_SEL_PRBS23			(3 << 1)
#define XCVR_SRDS_CFG_DFT_RX_PRBS_SEL_PRBS31			(4 << 1)
#define XCVR_SRDS_CFG_DFT_RX_PRBS_SEL_PRBS58			(5 << 1)
#define XCVR_SRDS_CFG_DFT_RX_PRBS_SEL_PRBS13			(6 << 1)
#define XCVR_SRDS_CFG_DFT_RX_PRBS_SEL_PRBS15			(8 << 1)
#define XCVR_SRDS_CFG_DFT_RX_DATA_SEL_SHIFT			5
#define XCVR_SRDS_CFG_DFT_RX_DATA_SEL_MASK			(3 << 5)

/* 0x42934: XCVR Serdes FHT control PRBS generator (TX) */
#define XCVR_SRDS_CFG_DFT_TX_PRBS_EN				BIT(0)
#define XCVR_SRDS_CFG_DFT_TX_PRBS_MODE_PRBS7			(0 << 1)
#define XCVR_SRDS_CFG_DFT_TX_PRBS_MODE_PRBS9			(1 << 1)
#define XCVR_SRDS_CFG_DFT_TX_PRBS_MODE_PRBS11			(2 << 1)
#define XCVR_SRDS_CFG_DFT_TX_PRBS_MODE_PRBS23			(3 << 1)
#define XCVR_SRDS_CFG_DFT_TX_PRBS_MODE_PRBS31			(4 << 1)
#define XCVR_SRDS_CFG_DFT_TX_PRBS_MODE_PRBS58			(5 << 1)
#define XCVR_SRDS_CFG_DFT_TX_PRBS_MODE_PRBS13			(6 << 1)
#define XCVR_SRDS_CFG_DFT_TX_PRBS_MODE_PRBS15			(8 << 1)

/* 0x4293C: XCVR Serdes FHT control PRBS generator pattern (TX) */
#define XCVR_SRDS_CFG_DFT_TX_PRBS_INIT				BIT(0)

/* 0x4312C: XCVR Serdes FHT register to specify user clock divide ratio */
#define XCVR_SRDS_GLB_DIV33_FRAC0P5_FW				BIT(1)
#define XCVR_SRDS_GLB_DIV33_FRAC0P25_FW				BIT(2)
#define XCVR_SRDS_GLB_DIV33_RATIO_FW_SHIFT			3
#define XCVR_SRDS_GLB_DIV33_RATIO_FW_MASK			0xF8

/* 0x44000: XCVR Serdes FGT LC Slow M Counter */
#define XCVR_SRDS_CFG_PLLLCSLOW_O_FBDIV_INTGR_SHIFT		0
#define XCVR_SRDS_CFG_PLLLCSLOW_O_FBDIV_INTGR_MASK		0x1FF
#define XCVR_SRDS_CFG_PLLLCSLOW_O_FBDIV_FRAC_SHIFT		9
#define XCVR_SRDS_CFG_PLLLCSLOW_O_FBDIV_FRAC_MASK		0x7FFFFE00

/* 0x44100: XCVR Serdes FGT LC Medium M Counter */
#define XCVR_SRDS_CFG_PLLLCMED_O_FBDIV_INTGR_SHIFT		0
#define XCVR_SRDS_CFG_PLLLCMED_O_FBDIV_INTGR_MASK		0x1FF
#define XCVR_SRDS_CFG_PLLLCMED_O_FBDIV_FRAC_SHIFT		9
#define XCVR_SRDS_CFG_PLLLCMED_O_FBDIV_FRAC_MASK		0x7FFFFE00

/* 0x44200: XCVR Serdes FGT LC Fast M Counter */
#define XCVR_SRDS_CFG_PLLLCFAST_O_FBDIV_INTGR_SHIFT		0
#define XCVR_SRDS_CFG_PLLLCFAST_O_FBDIV_INTGR_MASK		0x1FF
#define XCVR_SRDS_CFG_PLLLCFAST_O_FBDIV_FRAC_SHIFT		9
#define XCVR_SRDS_CFG_PLLLCFAST_O_FBDIV_FRAC_MASK		0x7FFFFE00

/* 0x44524: XCVR Serdes FGT K Strobe */
#define XCVR_SRDS_CFG_FGT_K_STROBE				BIT(31)

/* 0x44800: XCVR Serdes FHT fractional feedback divider ratio */
#define XCVR_SRDS_ANA_PLL_FBDIC_FRAC_SHIFT			9
#define XCVR_SRDS_ANA_PLL_FBDIC_FRAC_MASK			0x7FFFFC00
#define XCVR_SRDS_ANA_PLL_FRACNEN				BIT(31)

/* 0x4485C: XCVR Serdes FHT register to specify user clock divide ratio */
#define XCVR_SRDS_CFG_PCS3334_DIVSEL_MASK			(3 << 5)
#define XCVR_SRDS_CFG_PCS3334_DIVSEL_DIV33			(0 << 5)
#define XCVR_SRDS_CFG_PCS3334_DIVSEL_DIV34			(1 << 5)
#define XCVR_SRDS_CFG_PCS3334_DIVSEL_DIV66			(2 << 5)
#define XCVR_SRDS_CFG_PCS3334_DIVSEL_DIV68			(3 << 5)

/* 0x45080: XCVR Serdes FHT register to set TX Equalizer settings */
#define XCVR_SRDS_CSR_TXFFE_COEFF_LOAD				BIT(0)
#define XCVR_SRDS_CSR_TXFFE_COEFF_M2_SHIFT			2
#define XCVR_SRDS_CSR_TXFFE_COEFF_M2_MASK			0x000000FC
#define XCVR_SRDS_CSR_TXFFE_COEFF_M1_SHIFT			8
#define XCVR_SRDS_CSR_TXFFE_COEFF_M1_MASK			0x00003F00
#define XCVR_SRDS_CSR_TXFFE_COEFF_0_SHIFT			14
#define XCVR_SRDS_CSR_TXFFE_COEFF_0_MASK			0x001FC000
#define XCVR_SRDS_CSR_TXFFE_COEFF_P1_SHIFT			21
#define XCVR_SRDS_CSR_TXFFE_COEFF_P1_MASK			0x07E00000

/* 0x45084: XCVR Serdes FHT register to set TX Equalizer settings */
#define XCVR_SRDS_CSR_TXFFE_COEFF_P2_SHIFT			0
#define XCVR_SRDS_CSR_TXFFE_COEFF_P2_MASK			0x0000003F
#define XCVR_SRDS_CSR_TXFFE_COEFF_P3_SHIFT			6
#define XCVR_SRDS_CSR_TXFFE_COEFF_P3_MASK			0x00000FC0
#define XCVR_SRDS_CSR_TXFFE_COEFF_P4_SHIFT			12
#define XCVR_SRDS_CSR_TXFFE_COEFF_P4_MASK			0x0003F000
#define XCVR_SRDS_CSR_TXFFE_COEFF_M3_SHIFT			18
#define XCVR_SRDS_CSR_TXFFE_COEFF_M3_MASK			0x00FC0000

/* 0x45800: XCVR Serdes FHT control RX loopback and polarity inversion */
#define XCVR_SRDS_CFG_LANE_DIG_TX2RX_LOOPBACK_EN		BIT(0)
#define XCVR_SRDS_CFG_RX_PROBE_BUS_SEL_SHIFT			11
#define XCVR_SRDS_CFG_RX_PROBE_BUS_SEL_MASK			(0xF << 11)
#define XCVR_SRDS_CFG_RX_INV					BIT(18)

/* 0x45804: XCVR Serdes FHT register to control TX PRBS generator and polarity inversion */
#define XCVR_SRDS_CFG_TX_BUS_TAKE_DFT				BIT(0)
#define XCVR_SRDS_CFG_LANE_DIG_RX2TX_LOOPBACK_EN		BIT(19)
#define XCVR_SRDS_CFG_TX_INV					BIT(20)

/* 0x45808: XCVR Serdes FHT TX Error inject control */
#define XCVR_SRDS_CFG_TX_ERR_INJ_EN				BIT(0)
#define XCVR_SRDS_CFG_TX_ERR_INJ_MASK_CFG_SHIFT			6
#define XCVR_SRDS_CFG_TX_ERR_INJ_MASK_CFG_MASK			0x003FFFC0

/* 0x4585C: XCVR Serdes FHT control RX reconverge */
#define XCVR_SRDS_RX_RECONVERGE					BIT(14)

/* 0x47808: XCVR Serdes FGT TX precode */
#define XCVR_SRDS_CFG_ICTL_PCS_TXPAM_PRECODE_EN_A		BIT(2)
#define XCVR_SRDS_CFG_ICTL_PCS_TXPAM_GRAY_EN_A			BIT(3)

/* 0x47810: XCVR Serdes FGT RX graycode */
#define XCVR_SRDS_CFG_ICTL_PCS_RXPAM_GRAY_EN_A			BIT(2)
#define XCVR_SRDS_CFG_ICTL_PCS_RXPAM_PRECODE_EN_A		BIT(3)

/* 0x4781C: XCVR Serdes loopback */
#define XCVR_SRDS_LOOPBACK_INTERNAL				BIT(1)
#define XCVR_SRDS_LOOPBACK_EXTERNAL				BIT(2)

/* 0x47830: XCVR Serdes FGT TX Equalization */
#define XCVR_SRDS_CFG_ICTL_PCS_TXDRV_LEVNM1_SHIFT		0
#define XCVR_SRDS_CFG_ICTL_PCS_TXDRV_LEVNM1_MASK		0x1F
#define XCVR_SRDS_CFG_ICTL_PCS_TXDRV_LEVNP1_SHIFT		5
#define XCVR_SRDS_CFG_ICTL_PCS_TXDRV_LEVNP1_MASK		(0x1F << 5)
#define XCVR_SRDS_CFG_ICTL_PCS_TXDRV_LEVN_SHIFT			10
#define XCVR_SRDS_CFG_ICTL_PCS_TXDRV_LEVN_MASK			(0x3F << 10)
#define XCVR_SRDS_CFG_ICTL_PCS_TXDRV_LEVNM2_SHIFT		16
#define XCVR_SRDS_CFG_ICTL_PCS_TXDRV_LEVNM2_MASK		(7 << 16)

/* 0x60000: XCVR Serdes FHT enable PRBS */
#define XCVR_SRDS_CAR_TX_CLK_SRC_SEL				BIT(2)

/* 0x62000: XCVR Serdes lane CAR clock enable */
#define XCVR_SRDS_SHIM_LANE_CAR_CLKEN				BIT(16)

/* 0x62004: XCVR Serdes lane CAR software reset */
#define XCVR_SRDS_SHIM_LANE_CAR_SWRST				BIT(12)

/* 0x62008: XCVR Serdes lane fast lock mode in GPON */
#define XCVR_SRDS_SHIM_LANE_CAR_PWRCTRL_CDRLOCK2DATAEN_SRC_SEL	0x06000
#define XCVR_SRDS_SHIM_LANE_CAR_PWRCTRL_CDRLOCK2DATAEN		BIT(15)
#define XCVR_SRDS_SHIM_LANE_CAR_PWRCTRL_CDRLOCK2DATA_SRC_SEL	0x30000

/* 0x9003C: XCVR Serdes FGT Serial Loopback, PRBS select and BER test control */
#define XCVR_SRDS_CFG_LINK_MNG_CPI_CMD				0x0000FFFF
#define XCVR_SRDS_CFG_LINK_MNG_CPI_DATA				0xFFFF0000

/* 0x90040: XCVR Serdes FGT Serial Loopback, PRBS select and BER test control */
#define XCVR_SRDS_CFG_PHY_CPI_CMD				0x0000FFFF
#define XCVR_SRDS_CFG_PHY_CPI_DATA				0xFFFF0000

/* 0xF0004: XCVR Serdes FHT register to select user clock */
#define XCVR_SRDS_CFG_USER_RX_CLK1_SEL_LANEX_SHIFT		0
#define XCVR_SRDS_CFG_USER_RX_CLK1_SEL_LANEX_MASK		0x0000000F
#define XCVR_SRDS_CFG_USER_RX_CLK2_SEL_LANEX_SHIFT		4
#define XCVR_SRDS_CFG_USER_RX_CLK2_SEL_LANEX_MASK		0x000000F0
#define XCVR_SRDS_CFG_TX2RX_BYPASS_FHT_LANEX_SHIFT		8
#define XCVR_SRDS_CFG_TX2RX_BYPASS_FHT_LANEX_MASK		0x00000F00
#define XCVR_SRDS_CFG_USER_TX_CLK1_SEL_LANEX_SHIFT		16
#define XCVR_SRDS_CFG_USER_TX_CLK1_SEL_LANEX_MASK		0x000F0000
#define XCVR_SRDS_CFG_USER_TX_CLK2_SEL_LANEX_SHIFT		20
#define XCVR_SRDS_CFG_USER_TX_CLK2_SEL_LANEX_MASK		0x00F00000

/* 0xF0010 / 0xF0018 / 0xF0020 / 0xF0028: XCVR FGT Deterministic Latency control */
#define XCVR_FGT_Q_DL_CTRL_RX_LAT_CNTRVAL_ASYNC			0x3FFFF
#define XCVR_FGT_Q_DL_CTRL_RX_LAT_CNTR_ADDER			0x1C0000

#define XCVR_FHT_Q_DL_CTRL_RX_LAT_CNTRVAL_ASYNC			0x3FFFF

/* Ethernet Reconfiguration Interface Register Base Addresses
 * Word Offset	Register Type
 * 0x0000 - 0x00FC  F-Tile AIB Config
 * 0x0100 - 0x0FFC  Soft CSRs
 * 0x1000 - 0x1FFC  EHIP Registers (PHY, MAC, PTP), 10G/25G
 * 0x2000 - 0x2FFC  EHIP Registers (PHY, MAC, PTP), 50G
 * 0x3000 - 0x3FFC  EHIP Registers (PHY, MAC, PTP), 40G/100G
 * 0x4000 - 0x4FFC  EHIP Registers (PHY, MAC, PTP), 200G
 * 0x5000 - 0x5FFC  EHIP Registers (PHY, MAC, PTP), 400G
 * 0x6000 - 0x9FFC  FEC/XCVRIF Registers
 *
 * struct Definitions
 */

// F-Tile AIB Config : 0x0000 - 0x00FC
struct intel_fpga_ftile_eth_aib_config {
	u32 reserved[256/4];						// 0x0000 - 0x00FC
};

// F-Tile Soft CSRs : 0x0100 - 0x0FFC
struct intel_fpga_ftile_eth_soft_csr {
	u32 gui_option;							// 0x0100
	u32 qhip_scratch;						// 0x0104
	u32 eth_reset;							// 0x0108
	u32 eth_reset_status;						// 0x010C
	u32 phy_tx_pll_locked;						// 0x0110
	u32 phy_eiofreq_locked;						// 0x0114
	u32 pcs_status;							// 0x0118
	u32 pcs_control;						// 0x011C
	u32 link_fault_status;						// 0x0120
	u32 reserved_0124;						// 0x0124
	u32 clk_tx_khz;							// 0x0128
	u32 clk_rx_khz;							// 0x012C
	u32 clk_pll_khz;						// 0x0130
	u32 clk_tx_div_khz;						// 0x0134
	u32 clk_rec_div64_khz;						// 0x0138
	u32 clk_rec_div_khz;						// 0x013C
	u32 rxmac_adapt_dropped_31_0;					// 0x0140
	u32 rxmac_adapt_dropped_63_32;					// 0x0144
	u32 rxmac_adapt_dropped_control;				// 0x0148
	u32 reserved_014C[93];						// 0x014C - 0x02BC
	u32 anlt_sequencer_config;					// 0x02C0
	u32 anlt_sequencer_status;					// 0x02C4
	u32 reserved_02C8[14];						// 0x02C8 - 0x02FC
	u32 auto_neg_conf_1;						// 0x0300
	u32 auto_neg_conf_2;						// 0x0304
	u32 auto_neg_stat;						// 0x0308
	u32 auto_neg_conf_3;						// 0x030C
	u32 reserved_0310;						// 0x0310
	u32 auto_neg_conf_5;						// 0x0314
	u32 auto_neg_conf_6;						// 0x0318
	u32 auto_neg_stat_1;						// 0x031C
	u32 auto_neg_stat_2;						// 0x0320
	u32 auto_neg_stat_3;						// 0x0324
	u32 auto_neg_stat_4;						// 0x0328
	u32 reserved_032C;						// 0x032C
	u32 auto_neg_an_channel_override;				// 0x0330
	u32 reserved_0334;						// 0x0334
	u32 auto_neg_const_next_page_lp_stat;				// 0x0338
	u32 reserved_033C;						// 0x033C
	u32 link_train_conf_1;						// 0x0340
	u32 link_train_conf_2;						// 0x0344
	u32 link_train_stat_1;						// 0x0348
	u32 reserved_034C[301];						// 0x034C - 0x07FC
	u32 ptp_tx_tam_adjust;						// 0x0800
	u32 ptp_rx_tam_adjust;						// 0x0804
	u32 reserved_0808;						// 0x0808
	u32 ptp_ref_lane;						// 0x080C
	u32 ptp_dr_cfg;							// 0x0810
	u32 ptp_tx_user_cfg_status;					// 0x0814
	u32 ptp_rx_user_cfg_status;					// 0x0818
	u32 ptp_uim_tam_snapshot;					// 0x081C
	u32 ptp_tx_uim_tam_info0;					// 0x0820
	u32 ptp_tx_uim_tam_info1;					// 0x0824
	u32 ptp_rx_uim_tam_info0;					// 0x0828
	u32 ptp_rx_uim_tam_info1;					// 0x082C
	u32 ptp_status;							// 0x0830
	u32 reserved_0834[3];						// 0x0834 - 0x083C
	u32 ptp_status2;						// 0x0840
	u32 reserved_0844[43];						// 0x0844 - 0x08EC
	u32 ptp_tx_lane_calc_data_constdelay;				// 0x08F0
	u32 ptp_rx_lane_calc_data_constdelay;				// 0x08F4
	u32 reserved_08F8[2];						// 0x08F8 - 0x08FC
	u32 ptp_tx_lane0_calc_data_offset;				// 0x0900
	u32 ptp_rx_lane0_calc_data_offset;				// 0x0904
	u32 ptp_tx_lane0_calc_data_time;				// 0x0908
	u32 ptp_rx_lane0_calc_data_time;				// 0x090C
	u32 ptp_tx_lane0_calc_data_wiredelay;				// 0x0910
	u32 ptp_rx_lane0_calc_data_wiredelay;				// 0x0914
	u32 reserved_0918[2];						// 0x0918 - 0x091C
	u32 ptp_tx_lane1_calc_data_offset;				// 0x0920
	u32 ptp_rx_lane1_calc_data_offset;				// 0x0924
	u32 ptp_tx_lane1_calc_data_time;				// 0x0928
	u32 ptp_rx_lane1_calc_data_time;				// 0x092C
	u32 ptp_tx_lane1_calc_data_wiredelay;				// 0x0930
	u32 ptp_rx_lane1_calc_data_wiredelay;				// 0x0934
	u32 reserved_0938[2];						// 0x0938 - 0x093C
	u32 ptp_tx_lane2_calc_data_offset;				// 0x0940
	u32 ptp_rx_lane2_calc_data_offset;				// 0x0944
	u32 ptp_tx_lane2_calc_data_time;				// 0x0948
	u32 ptp_rx_lane2_calc_data_time;				// 0x094C
	u32 ptp_tx_lane2_calc_data_wiredelay;				// 0x0950
	u32 ptp_rx_lane2_calc_data_wiredelay;				// 0x0954
	u32 reserved_0958[2];						// 0x0958 - 0x095C
	u32 ptp_tx_lane3_calc_data_offset;				// 0x0960
	u32 ptp_rx_lane3_calc_data_offset;				// 0x0964
	u32 ptp_tx_lane3_calc_data_time;				// 0x0968
	u32 ptp_rx_lane3_calc_data_time;				// 0x096C
	u32 ptp_tx_lane3_calc_data_wiredelay;				// 0x0970
	u32 ptp_rx_lane3_calc_data_wiredelay;				// 0x0974
	u32 reserved_0978[2];						// 0x0978 - 0x097C
	u32 ptp_tx_lane4_calc_data_offset;				// 0x0980
	u32 ptp_rx_lane4_calc_data_offset;				// 0x0984
	u32 ptp_tx_lane4_calc_data_time;				// 0x0988
	u32 ptp_rx_lane4_calc_data_time;				// 0x098C
	u32 ptp_tx_lane4_calc_data_wiredelay;				// 0x0990
	u32 ptp_rx_lane4_calc_data_wiredelay;				// 0x0994
	u32 reserved_0998[2];						// 0x0998 - 0x099C
	u32 ptp_tx_lane5_calc_data_offset;				// 0x09A0
	u32 ptp_rx_lane5_calc_data_offset;				// 0x09A4
	u32 ptp_tx_lane5_calc_data_time;				// 0x09A8
	u32 ptp_rx_lane5_calc_data_time;				// 0x09AC
	u32 ptp_tx_lane5_calc_data_wiredelay;				// 0x09B0
	u32 ptp_rx_lane5_calc_data_wiredelay;				// 0x09B4
	u32 reserved_09B8[2];						// 0x09B8 - 0x09BC
	u32 ptp_tx_lane6_calc_data_offset;				// 0x09C0
	u32 ptp_rx_lane6_calc_data_offset;				// 0x09C4
	u32 ptp_tx_lane6_calc_data_time;				// 0x09C8
	u32 ptp_rx_lane6_calc_data_time;				// 0x09CC
	u32 ptp_tx_lane6_calc_data_wiredelay;				// 0x09D0
	u32 ptp_rx_lane6_calc_data_wiredelay;				// 0x09D4
	u32 reserved_09D8[2];						// 0x09D8 - 0x09DC
	u32 ptp_tx_lane7_calc_data_offset;				// 0x09E0
	u32 ptp_rx_lane7_calc_data_offset;				// 0x09E4
	u32 ptp_tx_lane7_calc_data_time;				// 0x09E8
	u32 ptp_rx_lane7_calc_data_time;				// 0x09EC
	u32 ptp_tx_lane7_calc_data_wiredelay;				// 0x09F0
	u32 ptp_rx_lane7_calc_data_wiredelay;				// 0x09F4
	u32 reserved_09F8[386];						// 0x09F8 - 0x0FFC
};

/* EHIP PHY registers
 * 0x000 - 0x07C : PCS Config
 * 0x080 - 0x1FC : PCS Status
 */
struct intel_fpga_ftile_eth_phy {
	// 0x000 - 0x07C : PCS Config
	u32 config_ctrl;				// 0x000
	u32 reserved_004[2];				// 0x004 - 0x008
	u32 ehip_reset_and_debug;			// 0x00C
	u32 phy_tx_pld_conf;				// 0x010
	u32 phy_rx_pld_conf;				// 0x014
	u32 reserved_018[12];				// 0x018 - 0x044
	u32 phy_ehip_pcs_modes;				// 0x048
	u32 phy_rx_pcs_conf;				// 0x04C
	u32 phy_tx_am_enc[4];				// 0x050 - 0x05C
	u32 phy_rx_am_enc[4];				// 0x060 - 0x06C
	u32 phy_timer_window_hiber_check;		// 0x070
	u32 phy_hiber_frm_err;				// 0x074
	u32 phy_pcs_err_inject;				// 0x078
	u32 reserved_07C;				// 0x07C
	// 0x080 - 0x1FC : PCS Status
	u32 phy_frm_err_detect;				// 0x080
	u32 phy_pcs_stat_anlt;				// 0x084
	u32 phy_am_lock;				// 0x088
	u32 phy_lanes_deskewed;				// 0x08C
	u32 phy_ber_cnt;				// 0x090
	u32 phy_pcs_virtual_ln_0;			// 0x094
	u32 phy_pcs_virtual_ln_1;			// 0x098
	u32 phy_pcs_virtual_ln_2;			// 0x09C
	u32 phy_pcs_virtual_ln_3;			// 0x0A0
	u32 phy_bip_cnt[20];				// 0x0A4 - 0x0F0
	u32 phy_err_block_cnt;				// 0x0F4
	u32 reserved_0F8;				// 0x0F8
	u32 phy_deskew_dept[4];				// 0x0FC - 0x108
	u32 phy_rx_pcs_test_err_cnt;			// 0x10C
	u32 phy_rx_bitslip_cnt;				// 0x110
	u32 ptp_vl_data_lsb[20];			// 0x114 - 0x160
	u32 ptp_vl_data_msb[20];			// 0x164 - 0x1B0
	u32 ptp_lal;					// 0x1B4
	u32 reserved_1B8[2];				// 0x1B8 - 0x1BC
	u32 phy_tx_pld_stat;				// 0x1C0
	u32 reserved_1C4[15];				// 0x1C4 - 0x1FC
};

/* EHIP MAC/PTP registers
 * 0x200 - 0x7FC : MAC/PTP Config
 * 0x800 - 0xFFC : MAC/PTP Statistics
 */
struct intel_fpga_ftile_eth_mac_ptp {
	// 0x200 - 0x7FC : MAC/PTP Config
	u32 tx_mac_link_fault;				// 0x200
	u32 tx_mac_ipg_col_rem;				// 0x204
	u32 tx_mac_max_frm_size;			// 0x208
	u32 tx_mac_conf;				// 0x20C
	u32 tx_mac_ehip_conf;				// 0x210
	u32 tx_mac_source_addr_lower_bytes;		// 0x214
	u32 tx_mac_source_addr_higher_bytes;		// 0x218
	u32 rx_mac_max_frm_size;			// 0x21C
	u32 rx_mac_frwd_rx_crc;				// 0x220
	u32 rx_mac_conf;				// 0x224
	u32 rx_mac_ehip_conf;				// 0x228
	u32 enable_tx_pause_ports;			// 0x22C
	u32 tx_pause_request;				// 0x230
	u32 enable_automatic_tx_pause_retransmission;	// 0x234
	u32 retransmit_holdoff_quanta;			// 0x238
	u32 retransmit_pause_quanta;			// 0x23C
	u32 enable_tx_xoff;				// 0x240
	u32 enable_uniform_holdoff;			// 0x244
	u32 set_uniform_holdoff;			// 0x248
	u32 flow_control_fields_lsb;			// 0x24C
	u32 flow_control_fields_msb;			// 0x250
	u32 flow_control_frames_lsb;			// 0x254
	u32 flow_control_frames_msb;			// 0x258
	u32 tx_flow_control_feature_cfg;		// 0x25C
	u32 enable_rx_pause_frame_processing_fields;	// 0x260
	u32 forward_flow_control_frames;		// 0x264
	u32 rx_pause_frames_lsb;			// 0x268
	u32 rx_pause_frames_msb;			// 0x26C
	u32 rx_flow_control_feature_cfg;		// 0x270
	u32 tx_cntr_config;				// 0x274
	u32 rx_cntr_config;				// 0x278
	u32 reserved_27C[2];				// 0x27C - 0x280
	u32 pause_quanta_0;				// 0x284
	u32 pause_quanta_1;				// 0x288
	u32 pause_quanta_2;				// 0x28C
	u32 pause_quanta_3;				// 0x290
	u32 pause_quanta_4;				// 0x294
	u32 pause_quanta_5;				// 0x298
	u32 pause_quanta_6;				// 0x29C
	u32 pause_quanta_7;				// 0x2A0
	u32 pfc_holdoff_quanta_0;			// 0x2A4
	u32 pfc_holdoff_quanta_1;			// 0x2A8
	u32 pfc_holdoff_quanta_2;			// 0x2AC
	u32 pfc_holdoff_quanta_3;			// 0x2B0
	u32 pfc_holdoff_quanta_4;			// 0x2B4
	u32 pfc_holdoff_quanta_5;			// 0x2B8
	u32 pfc_holdoff_quanta_6;			// 0x2BC
	u32 pfc_holdoff_quanta_7;			// 0x2C0
	u32 reserved_2C4[7];				// 0x2C4 - 0x2DC
	u32 tx_ptp_extra_latency;			// 0x2E0
	u32 tx_ptp_ui;					// 0x2E4
	u32 reserved_2E8;				// 0x2E8
	u32 tx_ptp_phy_lane_num;			// 0x2EC
	u32 tx_ptp_ap_filter;				// 0x2F0
	u32 tx_ptp_vl_offset[20];			// 0x2F4 - 0x340
	u32 rx_ptp_extra_latency;			// 0x344
	u32 rx_ptp_ui;					// 0x348
	u32 reserved_34C;				// 0x34C
	u32 rx_ptp_phy_lane_num;			// 0x350
	u32 rx_ptp_ap_filter;				// 0x354
	u32 rx_ptp_vl_offset[20];			// 0x358 - 0x3A4
	u32 rx_ptp_vl_to_pl[20];			// 0x3A8 - 0x3F4
	u32 rx_pkt_n_ts_rx_ctr;				// 0x3F8
	u32 reserved_3FC[257];				// 0x3FC - 0x7FC
	// 0x800 - 0xFFC : MAC/PTP Statistics
	u32 tx_fragments_lsb;				// 0x800
	u32 reserved_804;				// 0x804
	u32 tx_jabbers_lsb;				// 0x808
	u32 reserved_80C;				// 0x80C
	u32 reserved_810;				// 0x810
	u32 reserved_814;				// 0x814
	u32 tx_fcserr_lsb;				// 0x818
	u32 reserved_81C;				// 0x81C
	u32 tx_mcast_data_err_lsb;			// 0x820
	u32 reserved_824;				// 0x824
	u32 tx_bcast_data_err_lsb;			// 0x828
	u32 reserved_82C;				// 0x82C
	u32 tx_ucast_data_err_lsb;			// 0x830
	u32 reserved_834;				// 0x834
	u32 tx_mcast_ctrl_err_lsb;			// 0x838
	u32 reserved_83C;				// 0x83C
	u32 tx_bcast_ctrl_err_lsb;			// 0x840
	u32 reserved_844;				// 0x844
	u32 tx_ucast_ctrl_err_lsb;			// 0x848
	u32 reserved_84C;				// 0x84C
	u32 tx_pause_err_lsb;				// 0x850
	u32 reserved_854;				// 0x854
	u32 tx_64b_lsb;					// 0x858
	u32 tx_64b_msb;					// 0x85C
	u32 tx_65to127b_lsb;				// 0x860
	u32 tx_65to127b_msb;				// 0x864
	u32 tx_128to255b_lsb;				// 0x868
	u32 tx_128to255b_msb;				// 0x86C
	u32 tx_256to511b_lsb;				// 0x870
	u32 tx_256to511b_msb;				// 0x874
	u32 tx_512to1023b_lsb;				// 0x878
	u32 tx_512to1023b_msb;				// 0x87C
	u32 tx_1024to1518b_lsb;				// 0x880
	u32 tx_1024to1518b_msb;				// 0x884
	u32 tx_1519tomaxb_lsb;				// 0x888
	u32 tx_1519tomaxb_msb;				// 0x88C
	u32 tx_oversize_lsb;				// 0x890
	u32 reserved_894;				// 0x894
	u32 tx_mcast_data_ok_lsb;			// 0x898
	u32 tx_mcast_data_ok_msb;			// 0x89C
	u32 tx_bcast_data_ok_lsb;			// 0x8A0
	u32 tx_bcast_data_ok_msb;			// 0x8A4
	u32 tx_ucast_data_ok_lsb;			// 0x8A8
	u32 tx_ucast_data_ok_msb;			// 0x8AC
	u32 tx_mcast_ctrl_ok_lsb;			// 0x8B0
	u32 tx_mcast_ctrl_ok_msb;			// 0x8B4
	u32 tx_bcast_ctrl_ok_lsb;			// 0x8B8
	u32 tx_bcast_ctrl_ok_msb;			// 0x8BC
	u32 tx_ucast_ctrl_ok_lsb;			// 0x8C0
	u32 tx_ucast_ctrl_ok_msb;			// 0x8C4
	u32 tx_pause_lsb;				// 0x8C8
	u32 tx_pause_msb;				// 0x8CC
	u32 tx_rnt_lsb;					// 0x8D0
	u32 reserved_8D4;				// 0x8D4
	u32 tx_st_lsb;					// 0x8D8
	u32 tx_st_msb;					// 0x8DC
	u32 tx_lenerr_lsb;				// 0x8E0
	u32 reserved_8E4;				// 0x8E4
	u32 tx_pfc_err_lsb;				// 0x8E8
	u32 reserved_8EC;				// 0x8EC
	u32 tx_pfc_lsb;					// 0x8F0
	u32 tx_pfc_msb;					// 0x8F4
	u32 tx_payload_octetsok_lsb;			// 0x8F8
	u32 tx_payload_octetsok_msb;			// 0x8FC
	u32 tx_frame_octetsok_lsb;			// 0x900
	u32 tx_frame_octetsok_msb;			// 0x904
	u32 tx_malformed_ctrl_lsb;			// 0x908
	u32 reserved_90C;				// 0x90C
	u32 tx_dropped_ctrl_lsb;			// 0x910
	u32 reserved_914;				// 0x914
	u32 tx_badlt_ctrl_lsb;				// 0x918
	u32 reserved_91C;				// 0x91C
	u32 tx_total_ptp_pkts;				// 0x920
	u32 tx_total_1step_ptp_pkts;			// 0x924
	u32 tx_total_2step_ptp_pkts;			// 0x928
	u32 tx_total_v1_ptp_pkts;			// 0x92C
	u32 tx_total_v2_ptp_pkts;			// 0x930
	u32 rx_fragments_lsb;				// 0x934
	u32 reserved_938;				// 0x938
	u32 rx_jabbers_lsb;				// 0x93C
	u32 reserved_940;				// 0x940
	u32 reserved_944;				// 0x944
	u32 reserved_948;				// 0x948
	u32 rx_fcserr_lsb;				// 0x94C
	u32 reserved_950;				// 0x950
	u32 rx_mcast_data_err_lsb;			// 0x954
	u32 reserved_958;				// 0x958
	u32 rx_bcast_data_err_lsb;			// 0x95C
	u32 reserved_960;				// 0x960
	u32 rx_ucast_data_err_lsb;			// 0x964
	u32 reserved;					// 0x968
	u32 rx_mcast_ctrl_err_lsb;			// 0x96C
	u32 reserved_970;				// 0x970
	u32 rx_bcast_ctrl_err_lsb;			// 0x974
	u32 reserved_978;				// 0x978
	u32 rx_ucast_ctrl_err_lsb;			// 0x97C
	u32 reserved_980;				// 0x980
	u32 rx_pause_err_lsb;				// 0x984
	u32 reserved_988;				// 0x988
	u32 rx_64b_lsb;					// 0x98C
	u32 rx_64b_msb;					// 0x990
	u32 rx_65to127b_lsb;				// 0x994
	u32 rx_65to127b_msb;				// 0x998
	u32 rx_128to255b_lsb;				// 0x99C
	u32 rx_128to255b_msb;				// 0x9A0
	u32 rx_256to511b_lsb;				// 0x9A4
	u32 rx_256to511b_msb;				// 0x9A8
	u32 rx_512to1023b_lsb;				// 0x9AC
	u32 rx_512to1023b_msb;				// 0x9B0
	u32 rx_1024to1518b_lsb;				// 0x9B4
	u32 rx_1024to1518b_msb;				// 0x9B8
	u32 rx_1519tomaxb_lsb;				// 0x9BC
	u32 rx_1519tomaxb_msb;				// 0x9C0
	u32 rx_oversize_lsb;				// 0x9C4
	u32 reserved_9C8;				// 0x9C8
	u32 rx_mcast_data_ok_lsb;			// 0x9CC
	u32 rx_mcast_data_ok_msb;			// 0x9D0
	u32 rx_bcast_data_ok_lsb;			// 0x9D4
	u32 rx_bcast_data_ok_msb;			// 0x9D8
	u32 rx_ucast_data_ok_lsb;			// 0x9DC
	u32 rx_ucast_data_ok_msb;			// 0x9E0
	u32 rx_mcast_ctrl_ok_lsb;			// 0x9E4
	u32 rx_mcast_ctrl_ok_msb;			// 0x9E8
	u32 rx_bcast_ctrl_ok_lsb;			// 0x9EC
	u32 rx_bcast_ctrl_ok_msb;			// 0x9F0
	u32 rx_ucast_ctrl_ok_lsb;			// 0x9F4
	u32 rx_ucast_ctrl_ok_msb;			// 0x9F8
	u32 rx_pause_lsb;				// 0x9FC
	u32 rx_pause_msb;				// 0xA00
	u32 rx_rnt_lsb;					// 0xA04
	u32 reserved_A08;				// 0xA08
	u32 rx_st_lsb;					// 0xA0C
	u32 rx_st_msb;					// 0xA10
	u32 rx_lenerr_lsb;				// 0xA14
	u32 reserved_A18;				// 0xA18
	u32 rx_pfc_err_lsb;				// 0xA1C
	u32 reserved_A20;				// 0xA20
	u32 rx_pfc_lsb;					// 0xA24
	u32 rx_pfc_msb;					// 0xA28
	u32 rx_payload_octetsok_lsb;			// 0xA2C
	u32 rx_payload_octetsok_msb;			// 0xA30
	u32 rx_frame_octetsok_lsb;			// 0xA34
	u32 rx_frame_octetsok_msb;			// 0xA38
	u32 rx_malformed_lsb;				// 0xA3C
	u32 reserved_A40;				// 0xA40
	u32 rx_dropped_lsb;				// 0xA44
	u32 reserved_A48;				// 0xA48
	u32 rx_badlt_lsb;				// 0xA4C
	u32 reserved_A50;				// 0xA50
	u32 rx_total_ptp_ts;				// 0xA54
	u32 tx_ptp_cf_overflow;				// 0xA58
	u32 tx_ptp_tam_lo_pl_0;				// 0xA5C
	u32 tx_ptp_tam_med_pl_0;			// 0xA60
	u32 tx_ptp_tam_hi_pl_0;				// 0xA64
	u32 tx_ptp_tam_adj_pl_0;			// 0xA68
	u32 tx_ptp_tam_lo_pl_1;				// 0xA6C
	u32 tx_ptp_tam_med_pl_1;			// 0xA70
	u32 tx_ptp_tam_hi_pl_1;				// 0xA74
	u32 tx_ptp_tam_adj_pl_1;			// 0xA78
	u32 tx_ptp_tam_lo_pl_2;				// 0xA7C
	u32 tx_ptp_tam_med_pl_2;			// 0xA80
	u32 tx_ptp_tam_hi_pl_2;				// 0xA84
	u32 tx_ptp_tam_adj_pl_2;			// 0xA88
	u32 tx_ptp_tam_lo_pl_3;				// 0xA8C
	u32 tx_ptp_tam_med_pl_3;			// 0xA90
	u32 tx_ptp_tam_hi_pl_3;				// 0xA94
	u32 tx_ptp_tam_adj_pl_3;			// 0xA98
	u32 tx_ptp_tam_lo_pl_4;				// 0xA9C
	u32 tx_ptp_tam_med_pl_4;			// 0xAA0
	u32 tx_ptp_tam_hi_pl_4;				// 0xAA4
	u32 tx_ptp_tam_adj_pl_4;			// 0xAA8
	u32 tx_ptp_tam_lo_pl_5;				// 0xAAC
	u32 tx_ptp_tam_med_pl_5;			// 0xAB0
	u32 tx_ptp_tam_hi_pl_5;				// 0xAB4
	u32 tx_ptp_tam_adj_pl_5;			// 0xAB8
	u32 tx_ptp_tam_lo_pl_6;				// 0xABC
	u32 tx_ptp_tam_med_pl_6;			// 0xAC0
	u32 tx_ptp_tam_hi_pl_6;				// 0xAC4
	u32 tx_ptp_tam_adj_pl_6;			// 0xAC8
	u32 tx_ptp_tam_lo_pl_7;				// 0xACC
	u32 tx_ptp_tam_med_pl_7;			// 0xAD0
	u32 tx_ptp_tam_hi_pl_7;				// 0xAD4
	u32 tx_ptp_tam_adj_pl_7;			// 0xAD8
	u32 tx_ptp_tam_lo_pl_8;				// 0xADC
	u32 tx_ptp_tam_med_pl_8;			// 0xAE0
	u32 tx_ptp_tam_hi_pl_8;				// 0xAE4
	u32 tx_ptp_tam_adj_pl_8;			// 0xAE8
	u32 tx_ptp_tam_lo_pl_9;				// 0xAEC
	u32 tx_ptp_tam_med_pl_9;			// 0xAF0
	u32 tx_ptp_tam_hi_pl_9;				// 0xAF4
	u32 tx_ptp_tam_adj_pl_9;			// 0xAF8
	u32 tx_ptp_tam_lo_pl_10;			// 0xAFC
	u32 tx_ptp_tam_med_pl_10;			// 0xB00
	u32 tx_ptp_tam_hi_pl_10;			// 0xB04
	u32 tx_ptp_tam_adj_pl_10;			// 0xB08
	u32 tx_ptp_tam_lo_pl_11;			// 0xB0C
	u32 tx_ptp_tam_med_pl_11;			// 0xB10
	u32 tx_ptp_tam_hi_pl_11;			// 0xB14
	u32 tx_ptp_tam_adj_pl_11;			// 0xB18
	u32 tx_ptp_tam_lo_pl_12;			// 0xB1C
	u32 tx_ptp_tam_med_pl_12;			// 0xB20
	u32 tx_ptp_tam_hi_pl_12;			// 0xB24
	u32 tx_ptp_tam_adj_pl_12;			// 0xB28
	u32 tx_ptp_tam_lo_pl_13;			// 0xB2C
	u32 tx_ptp_tam_med_pl_13;			// 0xB30
	u32 tx_ptp_tam_hi_pl_13;			// 0xB34
	u32 tx_ptp_tam_adj_pl_13;			// 0xB38
	u32 tx_ptp_tam_lo_pl_14;			// 0xB3C
	u32 tx_ptp_tam_med_pl_14;			// 0xB40
	u32 tx_ptp_tam_hi_pl_14;			// 0xB44
	u32 tx_ptp_tam_adj_pl_14;			// 0xB48
	u32 tx_ptp_tam_lo_pl_15;			// 0xB4C
	u32 tx_ptp_tam_med_pl_15;			// 0xB50
	u32 tx_ptp_tam_hi_pl_15;			// 0xB54
	u32 tx_ptp_tam_adj_pl_15;			// 0xB58
	u32 tx_ts_ss_lo;				// 0xB5C
	u32 tx_ts_ss_mid;				// 0xB60
	u32 tx_ts_ss_hi;				// 0xB64
	u32 reserved_B68;				// 0xB68
	u32 rx_ptp_tam_lo_pl_0;				// 0xB6C
	u32 rx_ptp_tam_med_pl_0;			// 0xB70
	u32 rx_ptp_tam_hi_pl_0;				// 0xB74
	u32 rx_ptp_tam_adj_pl_0;			// 0xB78
	u32 rx_ptp_tam_lo_pl_1;				// 0xB7C
	u32 rx_ptp_tam_med_pl_1;			// 0xB80
	u32 rx_ptp_tam_hi_pl_1;				// 0xB84
	u32 rx_ptp_tam_adj_pl_1;			// 0xB88
	u32 rx_ptp_tam_lo_pl_2;				// 0xB8C
	u32 rx_ptp_tam_med_pl_2;			// 0xB90
	u32 rx_ptp_tam_hi_pl_2;				// 0xB94
	u32 rx_ptp_tam_adj_pl_2;			// 0xB98
	u32 rx_ptp_tam_lo_pl_3;				// 0xB9C
	u32 rx_ptp_tam_med_pl_3;			// 0xBA0
	u32 rx_ptp_tam_hi_pl_3;				// 0xBA4
	u32 rx_ptp_tam_adj_pl_3;			// 0xBA8
	u32 rx_ptp_tam_lo_pl_4;				// 0xBAC
	u32 rx_ptp_tam_med_pl_4;			// 0xBB0
	u32 rx_ptp_tam_hi_pl_4;				// 0xBB4
	u32 rx_ptp_tam_adj_pl_4;			// 0xBB8
	u32 rx_ptp_tam_lo_pl_5;				// 0xBBC
	u32 rx_ptp_tam_med_pl_5;			// 0xBC0
	u32 rx_ptp_tam_hi_pl_5;				// 0xBC4
	u32 rx_ptp_tam_adj_pl_5;			// 0xBC8
	u32 rx_ptp_tam_lo_pl_6;				// 0xBCC
	u32 rx_ptp_tam_med_pl_6;			// 0xBD0
	u32 rx_ptp_tam_hi_pl_6;				// 0xBD4
	u32 rx_ptp_tam_adj_pl_6;			// 0xBD8
	u32 rx_ptp_tam_lo_pl_7;				// 0xBDC
	u32 rx_ptp_tam_med_pl_7;			// 0xBE0
	u32 rx_ptp_tam_hi_pl_7;				// 0xBE4
	u32 rx_ptp_tam_adj_pl_7;			// 0xBE8
	u32 rx_ptp_tam_lo_pl_8;				// 0xBEC
	u32 rx_ptp_tam_med_pl_8;			// 0xBF0
	u32 rx_ptp_tam_hi_pl_8;				// 0xBF4
	u32 rx_ptp_tam_adj_pl_8;			// 0xBF8
	u32 rx_ptp_tam_lo_pl_9;				// 0xBFC
	u32 rx_ptp_tam_med_pl_9;			// 0xC00
	u32 rx_ptp_tam_hi_pl_9;				// 0xC04
	u32 rx_ptp_tam_adj_pl_9;			// 0xC08
	u32 rx_ptp_tam_lo_pl_10;			// 0xC0C
	u32 rx_ptp_tam_med_pl_10;			// 0xC10
	u32 rx_ptp_tam_hi_pl_10;			// 0xC14
	u32 rx_ptp_tam_adj_pl_10;			// 0xC18
	u32 rx_ptp_tam_lo_pl_11;			// 0xC1C
	u32 rx_ptp_tam_med_pl_11;			// 0xC20
	u32 rx_ptp_tam_hi_pl_11;			// 0xC24
	u32 rx_ptp_tam_adj_pl_11;			// 0xC28
	u32 rx_ptp_tam_lo_pl_12;			// 0xC2C
	u32 rx_ptp_tam_med_pl_12;			// 0xC30
	u32 rx_ptp_tam_hi_pl_12;			// 0xC34
	u32 rx_ptp_tam_adj_pl_12;			// 0xC38
	u32 rx_ptp_tam_lo_pl_13;			// 0xC3C
	u32 rx_ptp_tam_med_pl_13;			// 0xC40
	u32 rx_ptp_tam_hi_pl_13;			// 0xC44
	u32 rx_ptp_tam_adj_pl_13;			// 0xC48
	u32 rx_ptp_tam_lo_pl_14;			// 0xC4C
	u32 rx_ptp_tam_med_pl_14;			// 0xC50
	u32 rx_ptp_tam_hi_pl_14;			// 0xC54
	u32 rx_ptp_tam_adj_pl_14;			// 0xC58
	u32 rx_ptp_tam_lo_pl_15;			// 0xC5C
	u32 rx_ptp_tam_med_pl_15;			// 0xC60
	u32 rx_ptp_tam_hi_pl_15;			// 0xC64
	u32 rx_ptp_tam_adj_pl_15;			// 0xC68
	u32 rx_ts_ss_lo;				// 0xC6C
	u32 rx_ts_ss_mid;				// 0xC70
	u32 rx_ts_ss_hi;				// 0xC74
	u32 reserved_C78[226];				// 0xC78 - 0xFFC
};

/* F-Tile EHIP registers - address offset depends on Eth speed:
 * 10G/25G:            0x1000
 * 50G:                0x2000
 * 40G/100G:           0x3000
 * 200G:               0x4000
 * 400G:               0x5000
 */
struct intel_fpga_ftile_eth_ehip {
	struct intel_fpga_ftile_eth_phy     phy;			// 0x000 - 0x1FC
	struct intel_fpga_ftile_eth_mac_ptp mac_ptp;			// 0x200 - 0xFFC
};

/* F-Tile RS-FEC / XCVRIF Register Base Addresses
 * 0x6000 - 0x9FFC
 * 25GE:  Lane Segment0: 0x6000
 * 50GE:  Lane Segment0: 0x6200
 *        Lane Segment1: 0x6400
 * 100GE: Lane Segment0: 0x6600
 *        Lane Segment1: 0x6800
 *        Lane Segment2: 0x6A00
 *        Lane Segment3: 0x6C00
 * 200GE: Lane Segment0: 0x6E00
 *        ...
 *        Lane Segment7: 0x7C00
 * 400GE: Lane Segment0: 0x7E00
 *        ...
 *        Lane Segment15: 0x9C00
 */
struct intel_fpga_ftile_rsfec_xcvr {
	u32 reserved_0000[48];				// 0x0000 - 0x00BC
	u32 e25g_s0_rsfec_top;				// 0x00C0
	u32 reserved_00C4;				// 0x00C4
	u32 e25g_s0_rsfec_lane_cfg0;			// 0x00C8
	u32 reserved_00CC[27];				// 0x00CC - 0x0134
	u32 e25g_s0_rsfec_err_inj_tx;			// 0x0138
	u32 reserved_013C[3];				// 0x013C - 0x0144
	u32 rsfec_lane_tx_stat;				// 0x0148
	u32 rsfec_lane_tx_hold;				// 0x014C
	u32 reserved_0150;				// 0x0150
	u32 rsfec_lane_rx_stat;				// 0x0154
	u32 rsfec_lane_rx_hold;				// 0x0158
	u32 reserved_015C;				// 0x015C
	u32 rsfec_aggr_rx_stat;				// 0x0160
	u32 rsfec_aggr_rx_hold;				// 0x0164
	u32 reserved_0168;				// 0x0168
	u32 rsfec_ln_mapping_rx;			// 0x016C
	u32 rsfec_ln_skew_rx;				// 0x0170
	u32 rsfec_cw_pos_rx;				// 0x0174
	u32 rsfec_core_ecc_hold;			// 0x0178
	u32 rsfec_err_val_tx;				// 0x017C
	u32 reserved_0180;				// 0x0180
	u32 rsfec_corr_cw_cnt_lsb;			// 0x0184
	u32 rsfec_corr_cw_cnt_msb;			// 0x0188
	u32 rsfec_uncorr_cw_cnt_lsb;			// 0x018C
	u32 rsfec_uncorr_cw_cnt_msb;			// 0x0190
	u32 rsfec_corr_syms_cnt_lsb;			// 0x0194
	u32 rsfec_corr_syms_cnt_msb;			// 0x0198
	u32 rsfec_corr_0s_cnt_lsb;			// 0x019C
	u32 rsfec_corr_0s_cnt_msb;			// 0x01A0
	u32 rsfec_corr_1s_cnt_lsb;			// 0x01A4
	u32 rsfec_corr_1s_cnt_msb;			// 0x01A8
	u32 reserved_01AC[9];				// 0x01AC - 0x01CC
	u32 rsfec_corr_cwbin_cnt_0_3;			// 0x01D0
	u32 rsfec_corr_cwbin_cnt_4_7;			// 0x01D4
	u32 rsfec_corr_cwbin_cnt_8_11;			// 0x01D8
	u32 rsfec_corr_cwbin_cnt_12_15;			// 0x01DC
	u32 rsfec_debug_cfg;				// 0x01E0
	u32 reserved_01E4[7];				// 0x01E4 - 01FC
};

struct intel_fpga_ftile_ethernet {
	struct intel_fpga_ftile_eth_aib_config aib_config;		// 0x0000 - 0x00FC
	struct intel_fpga_ftile_eth_soft_csr   soft_csr;		// 0x0100 - 0x0FFC
	struct intel_fpga_ftile_eth_ehip       ehip[5];			// 0x1000 - 0x5FFC
	struct intel_fpga_ftile_rsfec_xcvr     rsfec_xcvr[32];		// 0x6000 - 0x9FFC
};

#define eth_csroffs(a)	(offsetof(struct intel_fpga_ftile_ethernet, a))
#define eth_soft_csroffs(a) \
	(offsetof(struct intel_fpga_ftile_ethernet, soft_csr.a))
#define eth_phy_csroffs(s, a)						\
	((offsetof(struct intel_fpga_ftile_ethernet, ehip[s].phy.a)) & 0xffff)
#define eth_mac_ptp_csroffs(s, a)					\
	((offsetof(struct intel_fpga_ftile_ethernet, ehip[s].mac_ptp.a)) & 0xffff)
#define eth_rsfec_csroffs(s, ln, a)					\
	(offsetof(struct intel_fpga_ftile_ethernet, rsfec_xcvr[(1 << (s)) - 1 + (ln)].a))

/* F-Tile Transceiver Reconfiguration Interface Register Base Addresses FGT/FHT per lane
 * add 0x8000 for each lane (0-3)
 */
struct intel_fpga_ftile_xcvr_fgt_fht {
	u32 reserved_40000[15];				// 0x40000 - 0x40038
	u32 srds_lane_car_ready;			// 0x4003C
	u32 reserved_40040[448];			// 0x40040 - 0x4073C
	u32 srds_ip_synth_med_reg_16;			// 0x40740
	u32 srds_ip_synth_med_reg_17;			// 0x40744
	u32 reserved_40748[62];				// 0x40748 - 0x4083C
	u32 srds_ip_synth_slow_reg_16;			// 0x40840
	u32 srds_ip_synth_slow_reg_17;			// 0x40844
	u32 reserved_40848[83];				// 0x40848 - 0x40990
	u32 srds_ip_synth_fast_reg_37;			// 0x40994
	u32 srds_ip_synth_fast_reg_38;			// 0x40998
	u32 reserved_4099C[675];			// 0x4099C - 0x41424
	u32 srds_ip_lane_reg_11;			// 0x41428
	u32 reserved_4142C[97];				// 0x4142C - 0x415AC
	u32 srds_ip_lane_rxeq_reg_109;			// 0x415B0
	u32 srds_ip_lane_rxeq_reg_110;			// 0x415B4
	u32 reserved_415B8[108];			// 0x415B8 - 0x41764
	u32 srds_ip_lane_reg_219;			// 0x41768
	u32 reserved_4176C[106];			// 0x4176C - 0x41910
	u32 srds_ip_lane_rxeq_reg_5;			// 0x41914
	u32 reserved_41918[168];			// 0x41918 - 0x41BB4
	u32 srds_ip_lane_rxeq_reg_174;			// 0x41BB8
	u32 reserved_41BBC[273];			// 0x41BBC - 0x41FFC
	u32 srds_cdr_enable;				// 0x42000
	u32 reserved_42004[566];			// 0x42004 - 0x428D8
	u32 srds_dft_ctrl_rx_ber_cnt_ctrl;		// 0x428DC
	u32 srds_dft_ctrl_rx_ber_cnt_trig;		// 0x428E0
	u32 srds_dft_ctrl_rx_ber_cnt_mask_0_31;		// 0x428E4
	u32 srds_dft_ctrl_rx_ber_cnt_mask_32_63;	// 0x428E8
	u32 srds_dft_ctrl_rx_ber_cnt_limit_lsb;		// 0x428EC
	u32 srds_dft_ctrl_rx_ber_cnt_limit_msb;		// 0x428F0
	u32 srds_dft_ctrl_rx_ber_cnt_status;		// 0x428F4
	u32 reserved_428F8[14];				// 0x428F8 - 0x4292C
	u32 srds_dft_ctrl_rx_prbs_common_ctrl;		// 0x42930
	u32 srds_dft_ctrl_tx_prbs_gen_ctrl;		// 0x42934
	u32 reserved_42938;				// 0x42938
	u32 srds_dft_ctrl_tx_prbs_gen_init;		// 0x4293C
	u32 reserved_42940[10];				// 0x42940 - 0x42964
	u32 srds_dft_ctrl_rx_ber_cnt_mask_64_95;	// 0x42968
	u32 srds_dft_ctrl_rx_ber_cnt_mask_96_127;	// 0x4296C
	u32 reserved_42970[495];			// 0x42970 - 0x43128
	u32 srds_lane_ana_adc_reg11;			// 0x4312C
	u32 reserved_43130[948];			// 0x43130 - 0x43FFC
	u32 srds_ip_plllcslow_div;			// 0x44000
	u32 reserved_44004[63];				// 0x44004 - 0x440FC
	u32 srds_ip_plllcmed_div;			// 0x44100
	u32 reserved_44104[63];				// 0x44104 - 0x441FC
	u32 srds_ip_plllcfast_div;			// 0x44200
	u32 reserved_44204[200];			// 0x44204 - 0x44520
	u32 srds_ip_pll_cfg_loader_reg_71;		// 0x44524
	u32 reserved_44528[182];			// 0x44528 - 0x447FC
	u32 srds_lane_ana_pll_div;			// 0x44800
	u32 reserved_44804[22];				// 0x44804 - 0x44858
	u32 srds_lane_ana_pll_control1;			// 0x4485C
	u32 reserved_44860[520];			// 0x44860 - 0x4507C
	u32 srds_lane_ana_tx_ffe1;			// 0x45080
	u32 srds_lane_ana_tx_ffe2;			// 0x45084
	u32 reserved_45088[478];			// 0x45088 - 0x457FC
	u32 srds_lane_rx_ctrl;				// 0x45800
	u32 srds_lane_tx_ctrl;				// 0x45804
	u32 srds_lane_ctrl_tx_err_inj_ctrl;		// 0x45808
	u32 srds_lane_ctrl_tx_err_inj_trig;		// 0x4580C
	u32 srds_lane_ctrl_tx_err_inj_load;		// 0x45810
	u32 reserved_45814[18];				// 0x45814 - 0x45858
	u32 srds_lane_ctrl_mode;			// 0x4585C
	u32 reserved_45860[2026];			// 0x45860 - 0x47804
	u32 srds_ip_if_tx;				// 0x47808
	u32 reserved_4780C;				// 0x4780C
	u32 srds_ip_if_rx;				// 0x47810
	u32 reserved_47814[2];				// 0x47814 - 0x47818
	u32 srds_ip_if_debug;				// 0x4781C
	u32 reserved_47820[4];				// 0x47820 - 0x4782C
	u32 srds_ip_if_tx1;				// 0x47830
	u32 reserved_47834[499];			// 0x47834 - 0x47FFC
};

/* F-Tile Transceiver Reconfiguration Interface Register Base Addresses
 * FHT/FGT PMA lane offset is 0x40000 for lane 0, 0x48000 for lane 1, 0x50000 for lane 2 etc.
 * i.e. add 0x8000 x lane_num (0-3)
 */
struct intel_fpga_ftile_xcvr {
	u32 reserved_000[512];				// 0x000 - 0x7FC
	u32 gui_option;					// 0x800
	u32 phy_scratch;				// 0x804
	u32 phy_reset;					// 0x808
	u32 phy_reset_status;				// 0x80C
	u32 phy_tx_pll_locked;				// 0x810
	u32 phy_rx_cdr_locked;				// 0x814
	u32 src_ctrl;					// 0x818
	u32 reserved_81C[65017];			// 0x81C - 0x3FFFC
	struct intel_fpga_ftile_xcvr_fgt_fht fgt_fht[4];// 0x40000 - 5FFFC
	u32 srds_shim_wrap_car_clkmux;			// 0x60000
	u32 reserved_60004[2047];			// 0x60004 - 61FFC
	u32 srds_shim_lane_car_clken;			// 0x62000
	u32 srds_shim_lane_car_swrst;			// 0x62004
	u32 srds_shim_lane_car_pwrctrl;			// 0x62008
	u32 reserved_6200C[47116];			// 0x6200C - 0x90038
	u32 scmng_pm_link_mng_side_cpi;			// 0x9003C
	u32 scmng_pm_phy_side_cpi;			// 0x90040
	u32 reserved_90044[98288];			// 0x90044 - 0xF0000
	u32 fht_config;					// 0xF0004
	u32 reserved_F0008[2];				// 0xF0008 - 0xF000C
	u32 fgt_q_dl_ctrl_a_l0;				// 0xF0010
	u32 reserved_f0014;				// 0xF0014
	u32 fgt_q_dl_ctrl_a_l1;				// 0xF0018
	u32 reserved_f001c;				// 0xF001C
	u32 fgt_q_dl_ctrl_a_l2;				// 0xF0020 FHT lane 0
	u32 fht_q_dl_ctrl_a_l1;				// 0xF0024 FHT lane 1
	u32 fgt_q_dl_ctrl_a_l3;				// 0xF0028 FHT lane 2
	u32 fht_q_dl_ctrl_a_l3;				// 0xF002C FHT lane 3
	u32 fht_preserve_ctrl;				// 0xF0030
	u32 reserved_f0034[16370];			// 0xF0034 - 0xFFFF8
	union {
		u32 fgt_quad_lane_number;		// 0xFFFFC
		u32 fht_cur_lane;			// 0xFFFFC
	} u;
};

#define eth_pma_avmm_csroffs(a, xcvr_num)		\
	((xcvr_num) * 0x100000 + offsetof(struct intel_fpga_ftile_xcvr, a))

/* Function prototypes */
void ftile_ui_adjustments(struct work_struct *work);
void ftile_ui_adjustments_init_worker(intel_fpga_xtile_eth_private *priv);
void ui_adjustments_cancel_worker(intel_fpga_xtile_eth_private *priv);

#endif
