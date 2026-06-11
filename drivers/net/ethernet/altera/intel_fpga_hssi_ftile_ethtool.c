// SPDX-License-Identifier: GPL-2.0
/* Ethtool support for Altera FPGA F-tile Ethernet MAC driver
 * Copyright (C) 2019-2022 Altera Corporation. All rights reserved
 *
 * Contributors:
 *   Roman Bulgakov
 *   Yu Ying Choo
 *   Dalon Westergreen
 *   Joyce Ooi
 *
 * Original driver contributed by GlobalLogic.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/kernel.h>
#include <linux/net_tstamp.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <linux/qsfp.h>
#include "altera_eth_dma.h"
#include "altera_fpga_anlt.h"
#include "intel_fpga_eth_ftile.h"
#include "intel_fpga_eth_hssi_itf.h"
#include "intel_fpga_eth_main.h"
#include "intel_fpga_ftile_driver.h"
#include "intel_fpga_hssiss.h"

#define DR_COMPLETE_TIMEOUT_US	1500000ULL
#define DR_POLL_INTERVAL_US	100

#define FTILE_STATS_LEN		ARRAY_SIZE(stat_gstrings)
#define FTILE_NUM_REGS		559

/*
 * Private flags exposed via ethtool --show-priv-flags.
 *
 * Bits [8:0] are named boolean flags displayed by ethtool --show-priv-flags:
 *   dr_supported         - DR profiles are present in DTS
 *   active_profile_valid - the active profile index is valid
 *   fec_baser            - Base-R / KR FEC is active on the current profile
 *   fec_rs               - RS-FEC is active on the current profile
 *   active_speed_*       - mutually exclusive; one set per active speed tier
 *
 * Bits [31:16] carry the raw active profile index as an unsigned integer
 * (use FTILE_PRIV_FLAGS_PROFILE_IDX_MASK/SHIFT to extract it).  These bits
 * are not part of the named-flag count so ethtool does not display them as
 * on/off toggles, but the value is accessible from the returned u32.
 */
enum ftile_priv_flags_bits {
	FTILE_PRIV_FLAG_BIT_DR_SUPPORTED = 0,
	FTILE_PRIV_FLAG_BIT_ACTIVE_PROFILE_VALID,
	FTILE_PRIV_FLAG_BIT_FEC_BASER,
	FTILE_PRIV_FLAG_BIT_FEC_RS,
	FTILE_PRIV_FLAG_BIT_SPEED_10G,
	FTILE_PRIV_FLAG_BIT_SPEED_25G,
	FTILE_PRIV_FLAG_BIT_SPEED_50G,
	FTILE_PRIV_FLAG_BIT_SPEED_100G,
	FTILE_PRIV_FLAG_BIT_SPEED_200G,
	FTILE_PRIV_FLAG_BIT_SPEED_400G,
	FTILE_PRIV_FLAG_BIT_PTP_ENABLED,
	FTILE_PRIV_FLAG_BIT_PTP_CLOCKCLEANER_ENABLED,
	FTILE_PRIV_FLAGS_COUNT,
};

#define FTILE_PRIV_FLAG(bit)			BIT(FTILE_PRIV_FLAG_BIT_##bit)

/* Active profile index packed into bits [31:16] of the priv-flags u32. */
#define FTILE_PRIV_FLAGS_PROFILE_IDX_SHIFT	16
#define FTILE_PRIV_FLAGS_PROFILE_IDX_MASK	GENMASK(31, 16)

static const char ftile_priv_flags_strings[][ETH_GSTRING_LEN] = {
	[FTILE_PRIV_FLAG_BIT_DR_SUPPORTED]         = "dr_supported",
	[FTILE_PRIV_FLAG_BIT_ACTIVE_PROFILE_VALID] = "active_profile_valid",
	[FTILE_PRIV_FLAG_BIT_FEC_BASER]            = "fec_baser",
	[FTILE_PRIV_FLAG_BIT_FEC_RS]               = "fec_rs",
	[FTILE_PRIV_FLAG_BIT_SPEED_10G]            = "active_speed_10g",
	[FTILE_PRIV_FLAG_BIT_SPEED_25G]            = "active_speed_25g",
	[FTILE_PRIV_FLAG_BIT_SPEED_50G]            = "active_speed_50g",
	[FTILE_PRIV_FLAG_BIT_SPEED_100G]           = "active_speed_100g",
	[FTILE_PRIV_FLAG_BIT_SPEED_200G]           = "active_speed_200g",
	[FTILE_PRIV_FLAG_BIT_SPEED_400G]           = "active_speed_400g",
	[FTILE_PRIV_FLAG_BIT_PTP_ENABLED]                  = "ptp_enabled",
	[FTILE_PRIV_FLAG_BIT_PTP_CLOCKCLEANER_ENABLED]     = "ptp_clockcleaner_enable",
};

#define FTILE_PRIV_FLAGS_LEN	ARRAY_SIZE(ftile_priv_flags_strings)

static const char stat_gstrings[][ETH_GSTRING_LEN] = {
	"tx_packets",
	"rx_packets",
	"rx_crc_errors",
	"rx_align_errors",
	"tx_bytes",
	"rx_bytes",
	"tx_pause",
	"rx_pause",
	"rx_errors",
	"tx_errors",
	"rx_unicast",
	"rx_multicast",
	"rx_broadcast",
	"tx_discards",
	"tx_unicast",
	"tx_multicast",
	"tx_broadcast",
	"ether_drops",
	"rx_total_bytes",
	"rx_total_packets",
	"rx_undersize",
	"rx_oversize",
	"rx_64_bytes",
	"rx_65_127_bytes",
	"rx_128_255_bytes",
	"rx_256_511_bytes",
	"rx_512_1023_bytes",
	"rx_1024_1518_bytes",
	"rx_gte_1519_bytes",
	"rx_jabbers",
	"rx_runts",
};

static void ftile_get_drvinfo(struct net_device *dev,
			      struct ethtool_drvinfo *info)
{
	strscpy(info->driver, "intel_fpga_ftile", ETH_GSTRING_LEN);
	strscpy(info->version, "v1.0", ETH_GSTRING_LEN);
	strscpy(info->bus_info, "platform", ETH_GSTRING_LEN);
}

/* Fill in a buffer with the strings which correspond to the
 * stats
 */
static void ftile_gstrings(struct net_device *dev, u32 stringset, u8 *buf)
{
	switch (stringset) {
	case ETH_SS_STATS:
		memcpy(buf, stat_gstrings, sizeof(stat_gstrings));
		break;
	case ETH_SS_PRIV_FLAGS:
		memcpy(buf, ftile_priv_flags_strings,
		       sizeof(ftile_priv_flags_strings));
		break;
	}
}

static void ftile_fill_stats(struct net_device *dev, struct ethtool_stats *dummy,
			     u64 *buf)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	struct platform_device *pdev  = priv->pdev_hssi;
	u32 hssi_port = priv->hssi_port;

	/* Tx packets */
	buf[0] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_PACKETS);

	/* Rx packets */
	buf[1] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_PACKETS);

	/* Rx CRC error packets */
	buf[2] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_CRC_ERRORS);

	/* Rx align error packets */
	buf[3] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_ALIGN_ERRORS);

	/* Tx bytes */
	buf[4] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_BYTES);

	/* Rx bytes */
	buf[5] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_BYTES);

	/* Tx pause bytes */
	buf[6] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_PAUSE);

	/* Rx pause bytes */
	buf[7] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_PAUSE);

	/* Rx error bytes */
	buf[8] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_ERRORS);

	/* Tx error bytes */
	buf[9] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_ERRORS);

	/* Rx unicast bytes */
	buf[10] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_UNICAST);

	/* Rx multicast bytes */
	buf[11] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_MULTICAST);

	/* Rx broadcast bytes */
	buf[12] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_BROADCAST);

	/* Tx discards bytes */
	buf[13] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_DISCARDS);

	/* Tx unicast bytes */
	buf[14] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_UNICAST);

	/* Tx multicast bytes */
	buf[15] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_MULTICAST);

	/* Tx broadcast bytes */
	buf[16] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_BROADCAST);

	/* Rx Ethernet drops */
	buf[17] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_ETHER_DROPS);

	/* Rx total bytes*/
	buf[18] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_TOTAL_BYTES);

	/* Rx total packets*/
	buf[19] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_TOTAL_PACKETS);

	/* Rx undersize*/
	buf[20] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_UNDERSIZE);

	/* Rx oversize*/
	buf[21] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_OVERSIZE);

	/* Rx 64 bytes*/
	buf[22] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_64_BYTES);

	/* Rx 65-127 bytes*/
	buf[23] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_65_127_BYTES);

	/* Rx 128-255 bytes*/
	buf[24] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_128_255_BYTES);

	/* Rx 256-511 bytes*/
	buf[25] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_256_511_BYTES);

	/* Rx 512-1023 bytes*/
	buf[26] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_512_1023_BYTES);

	/* Rx 1024-1518 bytes*/
	buf[27] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_1024_1518_BYTES);

	/* Rx > 10519 bytes*/
	buf[28] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_GTE_1519_BYTES);

	/* Rx jabber bytes*/
	buf[29] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_JABBERS);

	/* Rx fragments*/
	buf[30] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_RUNTS);
}

static int ftile_sset_count(struct net_device *dev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return FTILE_STATS_LEN;
	case ETH_SS_PRIV_FLAGS:
		return FTILE_PRIV_FLAGS_LEN;
	default:
		return -EOPNOTSUPP;
	}
}

/**
 * ftile_get_priv_flags - report read-only DR/profile state as priv flags.
 * @dev: network device
 *
 * Populates a u32 consumed by ethtool --show-priv-flags with:
 *
 *   dr_supported         - DR profiles are present in DTS
 *   active_profile_valid - the active profile index is valid
 *   fec_on               - FEC is enabled on the active profile
 *   active_speed_*       - one flag set to indicate the active profile speed
 *
 * Bits [31:16] carry the raw active profile index as an unsigned integer
 * (FTILE_PRIV_FLAGS_PROFILE_IDX_MASK).  ethtool does not display these bits
 * as named toggles, but the value is readable from the returned u32.
 *
 * This function is read-only for most flags; set_priv_flags supports
 * toggling ptp_clockcleaner_enable.
 */
static u32 ftile_get_priv_flags(struct net_device *dev)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	struct platform_device *pdev = priv->pdev_hssi;
	u32 flags = 0;
	int speed, fec;

	if (priv->dr_supported)
		flags |= FTILE_PRIV_FLAG(DR_SUPPORTED);

	if (hssi_active_profile_valid(pdev)) {
		flags |= FTILE_PRIV_FLAG(ACTIVE_PROFILE_VALID);

		fec = hssi_get_active_fec_mode(pdev);
		switch (fec) {
		case FTILE_FEC_BASER:
			flags |= FTILE_PRIV_FLAG(FEC_BASER);
			break;
		case FTILE_FEC_RS:
			flags |= FTILE_PRIV_FLAG(FEC_RS);
			break;
		default:
			break;
		}

		speed = hssi_get_active_profile_speed(pdev);
		switch (speed) {
		case SPEED_10000:
			flags |= FTILE_PRIV_FLAG(SPEED_10G);
			break;
		case SPEED_25000:
			flags |= FTILE_PRIV_FLAG(SPEED_25G);
			break;
		case SPEED_50000:
			flags |= FTILE_PRIV_FLAG(SPEED_50G);
			break;
		case SPEED_100000:
			flags |= FTILE_PRIV_FLAG(SPEED_100G);
			break;
		case SPEED_200000:
			flags |= FTILE_PRIV_FLAG(SPEED_200G);
			break;
		case SPEED_400000:
			flags |= FTILE_PRIV_FLAG(SPEED_400G);
			break;
		default:
			break;
		}

		flags |= FIELD_PREP(FTILE_PRIV_FLAGS_PROFILE_IDX_MASK,
				    hssi_active_profile_idx(pdev));
	}

	if (priv->ptp_enable)
		flags |= FTILE_PRIV_FLAG(PTP_ENABLED);

	if (priv->ptp_enable && priv->ptp_priv->ptp_clockcleaner_enable)
		flags |= FTILE_PRIV_FLAG(PTP_CLOCKCLEANER_ENABLED);

	return flags;
}

static u32 ftile_get_msglevel(struct net_device *dev)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	return priv->msg_enable;
}

static void ftile_set_msglevel(struct net_device *dev, u32 data)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	priv->msg_enable = data;
}

static int ftile_reglen(struct net_device *dev)
{
	return FTILE_NUM_REGS * sizeof(u32);
}

static void ftile_get_regs(struct net_device *dev, struct ethtool_regs *regs,
			   void *regbuf)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	struct platform_device *pdev  = priv->pdev_hssi;
	u32 chan = priv->tile_chan;

	u32 *buf = regbuf, i;

	/* Set version to a known value, so ethtool knows
	 * how to do any special formatting of this data.
	 * This version number will need to change if and
	 * when this register table is changed.
	 *
	 * version[31:0] = 1: Dump the 10GbE MAC IP Registers
	 *      Upper bits are all 0 by default
	 *
	 * Upper 16-bits will indicate feature presence for
	 * Ethtool register decoding in future version.
	 */

	regs->version = 1;

	/* F-Tile Soft CSRs : 0x0100 - 0x0FFC */
	buf[0] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				 eth_soft_csroffs(gui_option));			// 0x0100
	buf[1] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				 eth_soft_csroffs(qhip_scratch));		// 0x0104
	buf[2] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				 eth_soft_csroffs(eth_reset));			// 0x0108
	buf[3] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				 eth_soft_csroffs(eth_reset_status));		// 0x010C
	buf[4] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				 eth_soft_csroffs(phy_tx_pll_locked));		// 0x0110
	buf[5] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				 eth_soft_csroffs(phy_eiofreq_locked));		// 0x0114
	buf[6] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				 eth_soft_csroffs(pcs_status));			// 0x0118
	buf[7] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				 eth_soft_csroffs(pcs_control));		// 0x011C
	buf[8] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				 eth_soft_csroffs(link_fault_status));		// 0x0120
	buf[9] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				 eth_soft_csroffs(clk_tx_khz));			// 0x0128
	buf[10] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(clk_rx_khz));		// 0x012C
	buf[11] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(clk_pll_khz));		// 0x0130
	buf[12] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(clk_tx_div_khz));		// 0x0134
	buf[13] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(clk_rec_div64_khz));		// 0x0138
	buf[14] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(clk_rec_div_khz));		// 0x013C
	buf[15] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(rxmac_adapt_dropped_31_0));	// 0x0140
	buf[16] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(rxmac_adapt_dropped_63_32));	// 0x0144
	buf[17] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(rxmac_adapt_dropped_control));// 0x0148
	buf[18] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(profile_sel));		// 0x0200
	buf[19] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(fec_mode));			// 0x0204
	buf[20] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(sel_25g_10g));		// 0x0208
	buf[21] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(preamble_passthrough));	// 0x020C
	buf[22] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(anlt_sequencer_config));	// 0x02C0
	buf[23] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(anlt_sequencer_status));	// 0x02C4
	buf[24] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(auto_neg_conf_1));		// 0x0300
	buf[25] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(auto_neg_conf_2));		// 0x0304
	buf[26] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(auto_neg_stat));		// 0x0308
	buf[27] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(auto_neg_conf_3));		// 0x030C
	buf[28] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(auto_neg_conf_5));		// 0x0314
	buf[29] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(auto_neg_conf_6));		// 0x0318
	buf[30] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(auto_neg_stat_1));		// 0x031C
	buf[31] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(auto_neg_stat_2));		// 0x0320
	buf[32] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(auto_neg_stat_3));		// 0x0324
	buf[33] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(auto_neg_stat_4));		// 0x0328
	buf[34] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(auto_neg_an_channel_override));// 0x0330
	buf[35] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(auto_neg_const_next_page_lp_stat));// 0x0338
	buf[36] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(link_train_conf_1));		// 0x0340
	buf[37] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(link_train_conf_2));		// 0x0344
	buf[38] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(link_train_stat_1));		// 0x0348
	buf[39] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_tam_adjust));		// 0x0800
	buf[40] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_tam_adjust));		// 0x0804
	buf[41] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_ref_lane));		// 0x080C
	buf[42] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_dr_cfg));		// 0x0810
	buf[43] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_user_cfg_status));	// 0x0814
	buf[44] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_user_cfg_status));	// 0x0818
	buf[45] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_uim_tam_snapshot));	// 0x081C
	buf[46] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_uim_tam_info0));	// 0x0820
	buf[47] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_uim_tam_info1));	// 0x0824
	buf[48] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_uim_tam_info0));	// 0x0828
	buf[49] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_uim_tam_info1));	// 0x082C
	buf[50] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_status));		// 0x0830
	buf[51] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_status2));		// 0x0840
	buf[52] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_lane_calc_data_constdelay));	// 0x08F0
	buf[53] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_lane_calc_data_constdelay));	// 0x08F4
	buf[54] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_lane0_calc_data_offset));	// 0x0900
	buf[55] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_lane0_calc_data_offset));	// 0x0904
	buf[56] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_lane0_calc_data_time));	// 0x0908
	buf[57] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_lane0_calc_data_time));	// 0x090C
	buf[58] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_lane0_calc_data_wiredelay));	// 0x0910
	buf[59] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_lane0_calc_data_wiredelay));	// 0x0914
	buf[60] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_lane1_calc_data_offset));	// 0x0920
	buf[61] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_lane1_calc_data_offset));	// 0x0924
	buf[62] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_lane1_calc_data_time));	// 0x0928
	buf[63] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_lane1_calc_data_time));	// 0x092C
	buf[64] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_lane1_calc_data_wiredelay));	// 0x0930
	buf[65] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_lane1_calc_data_wiredelay));	// 0x0934
	buf[66] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_lane2_calc_data_offset));	// 0x0940
	buf[67] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_lane2_calc_data_offset));	// 0x0944
	buf[68] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_lane2_calc_data_time));	// 0x0948
	buf[69] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_lane2_calc_data_time));	// 0x094C
	buf[70] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_lane2_calc_data_wiredelay));	// 0x0950
	buf[71] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_lane2_calc_data_wiredelay));	// 0x0954
	buf[72] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_lane3_calc_data_offset));	// 0x0960
	buf[73] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_lane3_calc_data_offset));	// 0x0964
	buf[74] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_lane3_calc_data_time));	// 0x0968
	buf[75] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_lane3_calc_data_time));	// 0x096C
	buf[76] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_lane3_calc_data_wiredelay));	// 0x0970
	buf[77] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_lane3_calc_data_wiredelay));	// 0x0974
	buf[78] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_lane4_calc_data_offset));	// 0x0980
	buf[79] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_lane4_calc_data_offset));	// 0x0984
	buf[80] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_lane4_calc_data_time));	// 0x0988
	buf[81] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_lane4_calc_data_time));	// 0x098C
	buf[82] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_lane4_calc_data_wiredelay));	// 0x0990
	buf[83] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_lane4_calc_data_wiredelay));	// 0x0994
	buf[84] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_lane5_calc_data_offset));	// 0x09A0
	buf[85] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_lane5_calc_data_offset));	// 0x09A4
	buf[86] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_lane5_calc_data_time));	// 0x09A8
	buf[87] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_lane5_calc_data_time));	// 0x09AC
	buf[88] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_lane5_calc_data_wiredelay));	// 0x09B0
	buf[89] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_lane5_calc_data_wiredelay));	// 0x09B4
	buf[90] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_lane6_calc_data_offset));	// 0x09C0
	buf[91] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_lane6_calc_data_offset));	// 0x09C4
	buf[92] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_lane6_calc_data_time));	// 0x09C8
	buf[93] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_lane6_calc_data_time));	// 0x09CC
	buf[94] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_lane6_calc_data_wiredelay));	// 0x09D0
	buf[95] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_lane6_calc_data_wiredelay));	// 0x09D4
	buf[96] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_lane7_calc_data_offset));	// 0x09E0
	buf[97] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_lane7_calc_data_offset));	// 0x09E4
	buf[98] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_tx_lane7_calc_data_time));	// 0x09E8
	buf[99] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_soft_csroffs(ptp_rx_lane7_calc_data_time));	// 0x09EC
	buf[100] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_soft_csroffs(ptp_tx_lane7_calc_data_wiredelay));	// 0x09F0
	buf[101] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_soft_csroffs(ptp_rx_lane7_calc_data_wiredelay));	// 0x09F4

	/* F-tile EHIP PHY registers: */
	/* 0x000 - 0x07C : PCS Config */
	buf[102] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, config_ctrl));	// 0x000
	buf[103] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, ehip_reset_and_debug));// 0x00C
	buf[104] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_tx_pld_conf));	// 0x010
	buf[105] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_rx_pld_conf));	// 0x014
	buf[106] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_ehip_pcs_modes));// 0x048
	buf[107] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_rx_pcs_conf));	// 0x04C
	buf[108] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_tx_am_enc[0]));	// 0x050
	buf[109] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_tx_am_enc[1]));	// 0x054
	buf[110] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_tx_am_enc[2]));	// 0x058
	buf[111] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_tx_am_enc[3]));	// 0x05C
	buf[112] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_rx_am_enc[0]));	// 0x060
	buf[113] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_rx_am_enc[1]));	// 0x064
	buf[114] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_rx_am_enc[2]));	// 0x068
	buf[115] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_rx_am_enc[3]));	// 0x06C
	buf[116] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_timer_window_hiber_check));
	buf[117] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_hiber_frm_err));	// 0x074
	buf[118] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_pcs_err_inject));// 0x078
	// 0x080 - 0x1FC : PCS Status
	buf[119] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_frm_err_detect));// 0x080
	buf[120] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_pcs_stat_anlt));	// 0x084
	buf[121] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_am_lock));	// 0x088
	buf[122] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_lanes_deskewed));// 0x08C
	buf[123] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_ber_cnt));	// 0x090
	buf[124] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_pcs_virtual_ln_0));// 0x094
	buf[125] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_pcs_virtual_ln_1));// 0x098
	buf[126] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_pcs_virtual_ln_2));// 0x09C
	buf[127] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_pcs_virtual_ln_3));// 0x0A0
	// 0x0A4 - 0x0F0
	for (i = 0; i < 20; ++i)
		buf[124 + i] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
					       eth_phy_csroffs(priv->eth_rate, phy_bip_cnt[i]));
	buf[148] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_err_block_cnt));	// 0x0F4
	buf[149] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_deskew_dept[0]));// 0x0FC
	buf[150] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_deskew_dept[1]));// 0x100
	buf[151] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_deskew_dept[2]));// 0x104
	buf[152] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_deskew_dept[3]));// 0x108
	buf[153] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_rx_pcs_test_err_cnt));
	// 0x114 - 0x160
	for (i = 0; i < 20; ++i)
		buf[150 + i] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
					       eth_phy_csroffs(priv->eth_rate,
							       ptp_vl_data_lsb[i]));
	for (i = 0; i < 20; ++i)
		buf[170 + i] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
					       eth_phy_csroffs(priv->eth_rate,
							       ptp_vl_data_msb[i]));// 0x164 - 0x1B0
	buf[194] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, ptp_lal));	// 0x1B4
	buf[195] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(priv->eth_rate, phy_tx_pld_stat));	// 0x1C0

	/* F-tile EHIP MAC/PTP registers: */
	/* 0x200 - 0x7FC : MAC/PTP Config */
	buf[196] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_mac_link_fault));		// 0x200
	buf[197] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_mac_ipg_col_rem));		// 0x204
	buf[198] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_mac_max_frm_size));		// 0x208
	buf[199] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_mac_conf));			// 0x20C
	buf[200] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_mac_ehip_conf));		// 0x210
	buf[201] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_mac_source_addr_lower_bytes));// 0x214
	buf[202] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_mac_source_addr_higher_bytes));// 0x218
	buf[203] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_mac_max_frm_size));		// 0x21C
	buf[204] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_mac_frwd_rx_crc));		// 0x220
	buf[205] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_mac_conf));			// 0x224
	buf[206] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_mac_ehip_conf));		// 0x228
	buf[207] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       enable_tx_pause_ports));		// 0x22C
	buf[208] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_pause_request));		// 0x230
	buf[209] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,// 0x234
						       enable_automatic_tx_pause_retransmission));
	buf[210] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       retransmit_holdoff_quanta));	// 0x238
	buf[211] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       retransmit_pause_quanta));	// 0x23C
	buf[212] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       enable_tx_xoff));		// 0x240
	buf[213] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       enable_uniform_holdoff));	// 0x244
	buf[214] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       set_uniform_holdoff));		// 0x248
	buf[215] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       flow_control_fields_lsb));	// 0x24C
	buf[216] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       flow_control_fields_msb));	// 0x250
	buf[217] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       flow_control_frames_lsb));	// 0x254
	buf[218] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       flow_control_frames_msb));	// 0x258
	buf[219] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_flow_control_feature_cfg));	// 0x25C
	buf[220] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,// 0x260
						       enable_rx_pause_frame_processing_fields));
	buf[221] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       forward_flow_control_frames));	// 0x264
	buf[222] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_pause_frames_lsb));		// 0x268
	buf[223] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_pause_frames_msb));		// 0x26C
	buf[224] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_flow_control_feature_cfg));	// 0x270
	buf[225] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_cntr_config));		// 0x274
	buf[226] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_cntr_config));		// 0x278
	buf[227] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       pause_quanta_0));		// 0x284
	buf[228] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       pause_quanta_1));		// 0x288
	buf[229] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       pause_quanta_2));		// 0x28C
	buf[230] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       pause_quanta_3));		// 0x290
	buf[231] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       pause_quanta_4));		// 0x294
	buf[232] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       pause_quanta_5));		// 0x298
	buf[233] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       pause_quanta_6));		// 0x29C
	buf[234] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       pause_quanta_7));		// 0x2A0
	buf[235] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       pfc_holdoff_quanta_0));		// 0x2A4
	buf[236] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       pfc_holdoff_quanta_1));		// 0x2A8
	buf[237] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       pfc_holdoff_quanta_2));		// 0x2AC
	buf[238] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       pfc_holdoff_quanta_3));		// 0x2B0
	buf[239] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       pfc_holdoff_quanta_4));		// 0x2B4
	buf[240] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       pfc_holdoff_quanta_5));		// 0x2B8
	buf[241] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       pfc_holdoff_quanta_6));		// 0x2BC
	buf[242] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       pfc_holdoff_quanta_7));		// 0x2C0
	buf[243] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_extra_latency));		// 0x2E0
	buf[244] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_ui));			// 0x2E4
	buf[245] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_phy_lane_num));		// 0x2EC
	buf[246] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_ap_filter));		// 0x2F0
	for (i = 0; i < 20; ++i)
		buf[243 + i] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
					       eth_mac_ptp_csroffs(priv->eth_rate,// 0x2F4 - 0x340
								   tx_ptp_vl_offset[i]));
	buf[267] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_extra_latency));		// 0x344
	buf[268] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_ui));			// 0x348
	buf[269] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_phy_lane_num));		// 0x350
	buf[270] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_ap_filter));		// 0x354
	for (i = 0; i < 20; ++i)
		buf[267 + i] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
					       eth_mac_ptp_csroffs(priv->eth_rate,// 0x358 - 0x3A4
								   rx_ptp_vl_offset[i]));
	for (i = 0; i < 20; ++i)
		buf[287 + i] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
					       eth_mac_ptp_csroffs(priv->eth_rate,// 0x3A8 - 0x3F4
								   rx_ptp_vl_to_pl[i]));
	buf[311] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_pkt_n_ts_rx_ctr));		// 0x3F8
	/* 0x800 - 0xFFC : MAC/PTP Statistics */
	buf[312] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_fragments_lsb));		// 0x800
	buf[313] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_jabbers_lsb));		// 0x808
	buf[314] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate, tx_fcserr_lsb));	// 0x818
	buf[315] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_mcast_data_err_lsb));		// 0x820
	buf[316] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_bcast_data_err_lsb));		// 0x828
	buf[317] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ucast_data_err_lsb));		// 0x830
	buf[318] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_mcast_ctrl_err_lsb));		// 0x838
	buf[319] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_bcast_ctrl_err_lsb));		// 0x840
	buf[320] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ucast_ctrl_err_lsb));		// 0x848
	buf[321] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_pause_err_lsb));		// 0x850
	buf[322] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_64b_lsb));			// 0x858
	buf[323] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_64b_msb));			// 0x85C
	buf[324] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_65to127b_lsb));		// 0x860
	buf[325] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_65to127b_msb));		// 0x864
	buf[326] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_128to255b_lsb));		// 0x868
	buf[327] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_128to255b_msb));		// 0x86C
	buf[328] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_256to511b_lsb));		// 0x870
	buf[329] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_256to511b_msb));		// 0x874
	buf[330] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_512to1023b_lsb));		// 0x878
	buf[331] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_512to1023b_msb));		// 0x87C
	buf[332] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_1024to1518b_lsb));		// 0x880
	buf[333] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_1024to1518b_msb));		// 0x884
	buf[334] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_1519tomaxb_lsb));		// 0x888
	buf[335] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_1519tomaxb_msb));		// 0x88C
	buf[336] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_oversize_lsb));		// 0x890
	buf[337] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_mcast_data_ok_lsb));		// 0x898
	buf[338] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_mcast_data_ok_msb));		// 0x89C
	buf[339] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_bcast_data_ok_lsb));		// 0x8A0
	buf[340] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_bcast_data_ok_msb));		// 0x8A4
	buf[341] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ucast_data_ok_lsb));		// 0x8A8
	buf[342] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ucast_data_ok_msb));		// 0x8AC
	buf[343] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_mcast_ctrl_ok_lsb));		// 0x8B0
	buf[344] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_mcast_ctrl_ok_msb));		// 0x8B4
	buf[345] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_bcast_ctrl_ok_lsb));		// 0x8B8
	buf[346] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_bcast_ctrl_ok_msb));		// 0x8BC
	buf[347] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ucast_ctrl_ok_lsb));		// 0x8C0
	buf[348] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ucast_ctrl_ok_msb));		// 0x8C4
	buf[349] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_pause_lsb));			// 0x8C8
	buf[350] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_pause_msb));			// 0x8CC
	buf[351] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_rnt_lsb));			// 0x8D0
	buf[352] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_st_lsb));			// 0x8D8
	buf[353] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_st_msb));			// 0x8DC
	buf[354] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_lenerr_lsb));			// 0x8E0
	buf[355] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_pfc_err_lsb));		// 0x8E8
	buf[356] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_pfc_lsb));			// 0x8F0
	buf[357] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_pfc_msb));			// 0x8F4
	buf[358] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_payload_octetsok_lsb));	// 0x8F8
	buf[359] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_payload_octetsok_msb));	// 0x8FC
	buf[360] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_frame_octetsok_lsb));		// 0x900
	buf[361] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_frame_octetsok_msb));		// 0x904
	buf[362] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_malformed_ctrl_lsb));		// 0x908
	buf[363] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_dropped_ctrl_lsb));		// 0x910
	buf[364] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_badlt_ctrl_lsb));		// 0x918
	buf[365] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_total_ptp_pkts));		// 0x920
	buf[366] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_total_1step_ptp_pkts));	// 0x924
	buf[367] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_total_2step_ptp_pkts));	// 0x928
	buf[368] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_total_v1_ptp_pkts));		// 0x92C
	buf[369] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_total_v2_ptp_pkts));		// 0x930
	buf[370] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_fragments_lsb));		// 0x934
	buf[371] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_jabbers_lsb));		// 0x93C
	buf[372] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_fcserr_lsb));			// 0x94C
	buf[373] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_mcast_data_err_lsb));		// 0x954
	buf[374] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_bcast_data_err_lsb));		// 0x95C
	buf[375] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ucast_data_err_lsb));		// 0x964
	buf[376] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_mcast_ctrl_err_lsb));		// 0x96C
	buf[377] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_bcast_ctrl_err_lsb));		// 0x974
	buf[378] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ucast_ctrl_err_lsb));		// 0x97C
	buf[379] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_pause_err_lsb));		// 0x984
	buf[380] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_64b_lsb));			// 0x98C
	buf[381] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_64b_msb));			// 0x990
	buf[382] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_65to127b_lsb));		// 0x994
	buf[383] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_65to127b_msb));		// 0x998
	buf[384] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_128to255b_lsb));		// 0x99C
	buf[385] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_128to255b_msb));		// 0x9A0
	buf[386] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_256to511b_lsb));		// 0x9A4
	buf[387] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_256to511b_msb));		// 0x9A8
	buf[388] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_512to1023b_lsb));		// 0x9AC
	buf[389] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_512to1023b_msb));		// 0x9B0
	buf[390] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_1024to1518b_lsb));		// 0x9B4
	buf[391] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_1024to1518b_msb));		// 0x9B8
	buf[392] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_1519tomaxb_lsb));		// 0x9BC
	buf[393] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_1519tomaxb_msb));		// 0x9C0
	buf[394] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_oversize_lsb));		// 0x9C4
	buf[395] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_mcast_data_ok_lsb));		// 0x9CC
	buf[396] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_mcast_data_ok_msb));		// 0x9D0
	buf[397] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_bcast_data_ok_lsb));		// 0x9D4
	buf[398] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_bcast_data_ok_msb));		// 0x9D8
	buf[399] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ucast_data_ok_lsb));		// 0x9DC
	buf[400] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ucast_data_ok_msb));		// 0x9E0
	buf[401] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_mcast_ctrl_ok_lsb));		// 0x9E4
	buf[402] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_mcast_ctrl_ok_msb));		// 0x9E8
	buf[403] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_bcast_ctrl_ok_lsb));		// 0x9EC
	buf[404] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_bcast_ctrl_ok_msb));		// 0x9F0
	buf[405] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ucast_ctrl_ok_lsb));		// 0x9F4
	buf[406] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ucast_ctrl_ok_msb));		// 0x9F8
	buf[407] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_pause_lsb));			// 0x9FC
	buf[408] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_pause_msb));			// 0xA00
	buf[409] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_rnt_lsb));			// 0xA04
	buf[410] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_st_lsb));			// 0xA0C
	buf[411] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_st_msb));			// 0xA10
	buf[412] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_lenerr_lsb));			// 0xA14
	buf[413] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_pfc_err_lsb));		// 0xA1C
	buf[414] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_pfc_lsb));			// 0xA24
	buf[415] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_pfc_msb));			// 0xA28
	buf[416] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_payload_octetsok_lsb));	// 0xA2C
	buf[417] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_payload_octetsok_msb));	// 0xA30
	buf[418] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_frame_octetsok_lsb));		// 0xA34
	buf[419] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_frame_octetsok_msb));		// 0xA38
	buf[420] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_malformed_lsb));		// 0xA3C
	buf[421] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_dropped_lsb));		// 0xA44
	buf[422] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_badlt_lsb));			// 0xA4C
	buf[423] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_total_ptp_ts));		// 0xA54
	buf[424] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_cf_overflow));		// 0xA58
	buf[425] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_lo_pl_0));		// 0xA5C
	buf[426] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_med_pl_0));		// 0xA60
	buf[427] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_hi_pl_0));		// 0xA64
	buf[428] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_adj_pl_0));		// 0xA68
	buf[429] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_lo_pl_1));		// 0xA6C
	buf[430] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_med_pl_1));		// 0xA70
	buf[431] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_hi_pl_1));		// 0xA74
	buf[432] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_adj_pl_1));		// 0xA78
	buf[433] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_lo_pl_2));		// 0xA7C
	buf[434] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_med_pl_2));		// 0xA80
	buf[435] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_hi_pl_2));		// 0xA84
	buf[436] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_adj_pl_2));		// 0xA88
	buf[437] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_lo_pl_3));		// 0xA8C
	buf[438] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_med_pl_3));		// 0xA90
	buf[439] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_hi_pl_3));		// 0xA94
	buf[440] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_adj_pl_3));		// 0xA98
	buf[441] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_lo_pl_4));		// 0xA9C
	buf[442] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_med_pl_4));		// 0xAA0
	buf[443] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_hi_pl_4));		// 0xAA4
	buf[444] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_adj_pl_4));		// 0xAA8
	buf[445] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_lo_pl_5));		// 0xAAC
	buf[446] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_med_pl_5));		// 0xAB0
	buf[447] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_hi_pl_5));		// 0xAB4
	buf[448] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_adj_pl_5));		// 0xAB8
	buf[449] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_lo_pl_6));		// 0xABC
	buf[450] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_med_pl_6));		// 0xAC0
	buf[451] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_hi_pl_6));		// 0xAC4
	buf[452] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_adj_pl_6));		// 0xAC8
	buf[453] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_lo_pl_7));		// 0xACC
	buf[454] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_med_pl_7));		// 0xAD0
	buf[455] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_hi_pl_7));		// 0xAD4
	buf[456] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_adj_pl_7));		// 0xAD8
	buf[457] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_lo_pl_8));		// 0xADC
	buf[458] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_med_pl_8));		// 0xAE0
	buf[459] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_hi_pl_8));		// 0xAE4
	buf[460] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_adj_pl_8));		// 0xAE8
	buf[461] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_lo_pl_9));		// 0xAEC
	buf[462] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_med_pl_9));		// 0xAF0
	buf[463] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_hi_pl_9));		// 0xAF4
	buf[464] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_adj_pl_9));		// 0xAF8
	buf[465] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_lo_pl_10));		// 0xAFC
	buf[466] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_med_pl_10));		// 0xB00
	buf[467] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_hi_pl_10));		// 0xB04
	buf[468] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_adj_pl_10));		// 0xB08
	buf[469] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_lo_pl_11));		// 0xB0C
	buf[470] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_med_pl_11));		// 0xB10
	buf[471] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_hi_pl_11));		// 0xB14
	buf[472] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_adj_pl_11));		// 0xB18
	buf[473] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_lo_pl_12));		// 0xB1C
	buf[474] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_med_pl_12));		// 0xB20
	buf[475] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_hi_pl_12));		// 0xB24
	buf[476] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_adj_pl_12));		// 0xB28
	buf[477] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_lo_pl_13));		// 0xB2C
	buf[478] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_med_pl_13));		// 0xB30
	buf[479] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_hi_pl_13));		// 0xB34
	buf[480] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_adj_pl_13));		// 0xB38
	buf[481] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_lo_pl_14));		// 0xB3C
	buf[482] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_med_pl_14));		// 0xB40
	buf[483] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_hi_pl_14));		// 0xB44
	buf[484] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_adj_pl_14));		// 0xB48
	buf[485] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_lo_pl_15));		// 0xB4C
	buf[486] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_med_pl_15));		// 0xB50
	buf[487] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_hi_pl_15));		// 0xB54
	buf[488] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ptp_tam_adj_pl_15));		// 0xB58
	buf[489] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ts_ss_lo));			// 0xB5C
	buf[490] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ts_ss_mid));			// 0xB60
	buf[491] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       tx_ts_ss_hi));			// 0xB64
	buf[492] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_lo_pl_0));		// 0xB6C
	buf[493] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_med_pl_0));		// 0xB70
	buf[494] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_hi_pl_0));		// 0xB74
	buf[495] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_adj_pl_0));		// 0xB78
	buf[496] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_lo_pl_1));		// 0xB7C
	buf[497] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_med_pl_1));		// 0xB80
	buf[498] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_hi_pl_1));		// 0xB84
	buf[499] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_adj_pl_1));		// 0xB88
	buf[500] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_lo_pl_2));		// 0xB8C
	buf[501] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_med_pl_2));		// 0xB90
	buf[502] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_hi_pl_2));		// 0xB94
	buf[503] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_adj_pl_2));		// 0xB98
	buf[504] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_lo_pl_3));		// 0xB9C
	buf[505] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_med_pl_3));		// 0xBA0
	buf[506] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_hi_pl_3));		// 0xBA4
	buf[507] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_adj_pl_3));		// 0xBA8
	buf[508] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_lo_pl_4));		// 0xBAC
	buf[509] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_med_pl_4));		// 0xBB0
	buf[510] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_hi_pl_4));		// 0xBB4
	buf[511] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_adj_pl_4));		// 0xBB8
	buf[512] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_lo_pl_5));		// 0xBBC
	buf[513] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_med_pl_5));		// 0xBC0
	buf[514] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_hi_pl_5));		// 0xBC4
	buf[515] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_adj_pl_5));		// 0xBC8
	buf[516] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_lo_pl_6));		// 0xBCC
	buf[517] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_med_pl_6));		// 0xBD0
	buf[518] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_hi_pl_6));		// 0xBD4
	buf[519] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_adj_pl_6));		// 0xBD8
	buf[520] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_lo_pl_7));		// 0xBDC
	buf[521] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_med_pl_7));		// 0xBE0
	buf[522] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_hi_pl_7));		// 0xBE4
	buf[523] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_adj_pl_7));		// 0xBE8
	buf[524] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_lo_pl_8));		// 0xBEC
	buf[525] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_med_pl_8));		// 0xBF0
	buf[526] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_hi_pl_8));		// 0xBF4
	buf[527] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_adj_pl_8));		// 0xBF8
	buf[528] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_lo_pl_9));		// 0xBFC
	buf[529] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_med_pl_9));		// 0xC00
	buf[530] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_hi_pl_9));		// 0xC04
	buf[531] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_adj_pl_9));		// 0xC08
	buf[532] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_lo_pl_10));		// 0xC0C
	buf[533] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_med_pl_10));		// 0xC10
	buf[534] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_hi_pl_10));		// 0xC14
	buf[535] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_adj_pl_10));		// 0xC18
	buf[536] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_lo_pl_11));		// 0xC1C
	buf[537] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_med_pl_11));		// 0xC20
	buf[538] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_hi_pl_11));		// 0xC24
	buf[539] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_adj_pl_11));		// 0xC28
	buf[540] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_lo_pl_12));		// 0xC2C
	buf[541] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_med_pl_12));		// 0xC30
	buf[542] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_hi_pl_12));		// 0xC34
	buf[543] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_adj_pl_12));		// 0xC38
	buf[544] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_lo_pl_13));		// 0xC3C
	buf[545] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_med_pl_13));		// 0xC40
	buf[546] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_hi_pl_13));		// 0xC44
	buf[547] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_adj_pl_13));		// 0xC48
	buf[548] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_lo_pl_14));		// 0xC4C
	buf[549] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_med_pl_14));		// 0xC50
	buf[550] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_hi_pl_14));		// 0xC54
	buf[551] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_adj_pl_14));		// 0xC58
	buf[552] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_lo_pl_15));		// 0xC5C
	buf[553] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_med_pl_15));		// 0xC60
	buf[554] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_hi_pl_15));		// 0xC64
	buf[555] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ptp_tam_adj_pl_15));		// 0xC68
	buf[556] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ts_ss_lo));			// 0xC6C
	buf[557] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ts_ss_mid));			// 0xC70
	buf[558] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_mac_ptp_csroffs(priv->eth_rate,
						       rx_ts_ss_hi));			// 0xC74

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
}

static void ftile_get_pauseparam(struct net_device *dev,
				 struct ethtool_pauseparam *pauseparam)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	pauseparam->rx_pause = 0;
	pauseparam->tx_pause = 0;
	pauseparam->autoneg = 0;

	if (priv->flow_ctrl & FLOW_RX)
		pauseparam->rx_pause = 1;
	if (priv->flow_ctrl & FLOW_TX)
		pauseparam->tx_pause = 1;
}

static int ftile_set_pauseparam(struct net_device *dev,
				struct ethtool_pauseparam *pauseparam)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	int new_pause = FLOW_OFF;
	struct platform_device *pdev  = priv->pdev_hssi;
	u32 chan = priv->tile_chan;

	/*
	 * Do not hold mac_cfg_lock (a spinlock) here. The CSR access path
	 * through hssi_set_bit_ba/hssi_csrwr32_ba calls hssidrv_sal_execute()
	 * which acquires sal_mutex (a sleepable mutex) and uses
	 * read_poll_timeout(), both of which may sleep. Taking a spinlock
	 * around sleepable operations causes a "scheduling while atomic" BUG.
	 *
	 * Serialization is already provided by:
	 *   - the RTNL lock held by the ethtool core for all ethtool callbacks
	 *   - sal_mutex inside hssidrv_sal_execute() for CSR access
	 */

	if (pauseparam->autoneg != 0)
		return -EINVAL;

	if (pauseparam->rx_pause) {
		new_pause |= FLOW_RX;
		hssi_set_bit_ba(pdev, HSSI_ETH_RECONFIG, chan,
				eth_mac_ptp_csroffs(priv->eth_rate, rx_flow_control_feature_cfg),
				ETH_RX_EN_STD_FLOW_CTRL);
	} else {
		hssi_clear_bit_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_mac_ptp_csroffs(priv->eth_rate, rx_flow_control_feature_cfg),
				  ETH_RX_EN_STD_FLOW_CTRL);
	}

	if (pauseparam->tx_pause) {
		new_pause |= FLOW_TX;
		hssi_set_bit_ba(pdev, HSSI_ETH_RECONFIG, chan,
				eth_mac_ptp_csroffs(priv->eth_rate, tx_flow_control_feature_cfg),
				ETH_TX_EN_STD_FLOW_CTRL);
	} else {
		hssi_clear_bit_ba(pdev, HSSI_ETH_RECONFIG, chan,
				  eth_mac_ptp_csroffs(priv->eth_rate, tx_flow_control_feature_cfg),
				  ETH_TX_EN_STD_FLOW_CTRL);
	}

	hssi_csrwr32_ba(pdev, HSSI_ETH_RECONFIG, chan,
			eth_mac_ptp_csroffs(priv->eth_rate, pause_quanta_0), priv->pause);
	priv->flow_ctrl = new_pause;

	return 0;
}

static int ftile_get_ts_info(struct net_device *dev,
			     struct kernel_ethtool_ts_info *info)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	if (!priv->ptp_enable)
		return ethtool_op_get_ts_info(dev, info);

	info->so_timestamping = SOF_TIMESTAMPING_TX_HARDWARE |
				SOF_TIMESTAMPING_RX_HARDWARE |
				SOF_TIMESTAMPING_RAW_HARDWARE |
				SOF_TIMESTAMPING_TX_SOFTWARE |
				SOF_TIMESTAMPING_RX_SOFTWARE |
				SOF_TIMESTAMPING_SOFTWARE;

	if (priv->ptp_priv->ptp_clock)
		info->phc_index = ptp_clock_index(priv->ptp_priv->ptp_clock);
	else
		info->phc_index = -1;

	info->tx_types = (1 << HWTSTAMP_TX_OFF) |
			 (1 << HWTSTAMP_TX_ON) |
			 (1 << HWTSTAMP_TX_ONESTEP_SYNC);

	info->rx_filters = (1 << HWTSTAMP_FILTER_NONE) |
			   (1 << HWTSTAMP_FILTER_ALL);

	return 0;
}

static u32 ftile_get_active_fec_mode(intel_fpga_xtile_eth_private *priv)
{
	int ret;

	ret = hssi_get_active_fec_mode(priv->pdev_hssi);

	if (ret == -EINVAL) {
		if (!strcasecmp(priv->fec_type, "rs-fec"))
			return FTILE_FEC_RS;
		if (!strcasecmp(priv->fec_type, "kr-fec") ||
		    !strcasecmp(priv->fec_type, "base-r"))
			return FTILE_FEC_BASER;
		return FTILE_FEC_NONE;
	}

	return ret;
}

/**
 * ftile_update_active_profile - update the active DR profile and sync FEC state
 * @dev:         network device
 * @profile_idx: profile index to mark as active
 * @fec:         FEC mode associated with this profile (0 = no-FEC, 1 = Base-R, 2 = RS-FEC)
 *
 * Wrapper around hssi_update_active_profile() that also keeps priv->fec_type
 * in sync so that software FEC queries reflect the newly active profile.
 */
static void ftile_update_active_profile(struct net_device *dev,
					u32 profile_idx, u32 fec)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	hssi_update_active_profile(priv->pdev_hssi, profile_idx);

	priv->fec_type = hssi_fec_type_str(fec);
}

/**
 * ftile_switch_profile - look up and activate a DR profile by speed + FEC
 * @dev:        network device
 * @speed_mbps: requested link speed
 * @fec:        requested FEC mode (0 = no-FEC, 1 = Base-R, 2 = RS-FEC)
 *
 * This function performs a fully generic F-Tile Dynamic Reconfiguration
 * (DR) profile switch. It supports switching from any currently active
 * profile to any other profile without relying on profile ordering or
 * adjacency.
 *
 * IMPORTANT DESIGN NOTES:
 *
 * 1) Profile IDs are opaque values assigned by Quartus.
 *    They are NOT contiguous, NOT ordered, and MUST NOT be derived
 *    arithmetically (e.g. profile_id - 1 is invalid).
 *
 * 2) The DR engine is stateless. Hardware does not remember the previous
 *    profile, nor does it provide an automatic rollback mechanism.
 *    Therefore, software must track the active profile explicitly.
 *
 * 3) Dynamic reconfiguration is NOT atomic.
 *    Once a DR trigger is issued, partial reconfiguration may occur.
 *    Rollback, if required, must be done explicitly using a second DR
 *    operation or startup profile recovery.
 *
 * RETURN VALUE:
 *  - 0 on success
 *  - -ENOENT if no DR profile matches (speed, fec)
 *  - negative errno on hardware / DR failure
 */
static int ftile_switch_profile(struct net_device *dev,
				u32 speed_mbps, u32 fec)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	struct get_set_dr_data dr_data;
	u32 target_profile;
	u32 prev_profile;
	u32 rel_port = priv->hssi_rel_port;
	int ret;

	/* ------------------------------------------------------------
	 * Step 1: Resolve requested (speed, fec) to a Quartus profile ID
	 *
	 * This lookup is table-driven (DTS/IP metadata). If the profile
	 * does not exist, we fail early before touching hardware.
	 * ------------------------------------------------------------
	 */
	ret = hssi_find_dr_profile(priv->pdev_hssi, speed_mbps, fec, rel_port, &target_profile);
	if (ret)
		return ret; /* -ENOENT is expected and handled by caller */

	if (WARN_ON_ONCE(target_profile >= hssi_num_dr_profiles(priv->pdev_hssi)))
		return -EINVAL;

	/* Step 2: Capture current profile index for rollback (always valid
	 * here because dr_supported guarantees probe-time initialisation).
	 */
	prev_profile = hssi_active_profile_idx(priv->pdev_hssi);

	/* Early exit: target profile already active, nothing to do. */
	if (target_profile == prev_profile) {
		netdev_info(dev,
			    "profile %u already active (speed=%u fec=%u), no switch needed\n",
			    target_profile, speed_mbps, fec);
		return 0;
	}

	/* ------------------------------------------------------------
	 * Step 3: Ensure DR engine is ready for a new trigger
	 *
	 * Bit[1] of the DR trigger register indicates "Ready for
	 * New Trigger". Issuing a trigger when not ready can corrupt
	 * the reconfiguration state.
	 * ------------------------------------------------------------
	 */
	dr_data.addr_offs = DYN_RCFG_DR_TRIGGER_REG;
	ret = hssi_get_dr_profile(priv->pdev_hssi, &dr_data);
	if (ret)
		return ret;

	if (!(dr_data.val & READY_FOR_DR))
		return -EBUSY;

	netdev_dbg(dev, "Step 3: addr_offs: %x Val: 0x%x expected: 0x%lx\n",
		   dr_data.addr_offs, dr_data.val, READY_FOR_DR);

	hssi_invalidate_active_profile(priv->pdev_hssi);

	/* ------------------------------------------------------------
	 * Step 4: Disable the currently active profile
	 *
	 * Write the active profile number to its register (slot = profile
	 * index) with bit 16 cleared. Only two profile registers (0 and 1)
	 * are used; the slot is the profile index itself.
	 * ------------------------------------------------------------
	 */
	dr_data.addr_offs = DYN_RCFG_DR_NEXT_PROFILE_0_REG;
	dr_data.val = hssi_get_dr_profile_hw_id(priv->pdev_hssi, prev_profile);

	ret = hssi_set_dr_profile(priv->pdev_hssi, &dr_data);
	if (ret)
		goto dr_failed;

	netdev_dbg(dev, "Step 4: Disable active profile addr_offs: 0x%x Val: 0x%x\n",
		   dr_data.addr_offs, dr_data.val);

	/* ------------------------------------------------------------
	 * Step 5: Enable the target profile
	 *
	 * Write the new profile number to its register with bit 16 set.
	 * ------------------------------------------------------------
	 */
	dr_data.addr_offs = DYN_RCFG_DR_NEXT_PROFILE_1_REG;
	dr_data.val = hssi_get_dr_profile_hw_id(priv->pdev_hssi, target_profile) |
		      NEXT_PROFILE_ENABLE;

	ret = hssi_set_dr_profile(priv->pdev_hssi, &dr_data);
	if (ret)
		goto dr_failed;

	netdev_dbg(dev, "Step 5: Enable target profile addr_offs: 0x%x Val: 0x%x\n",
		   dr_data.addr_offs, dr_data.val);

	/* ------------------------------------------------------------
	 * Step 6: Trigger reconfiguration
	 *
	 * Writing BIT(0) causes the DR Nios controller to execute the
	 * programmed profile sequence.
	 * ------------------------------------------------------------
	 */
	dr_data.addr_offs = DYN_RCFG_DR_TRIGGER_REG;
	dr_data.val       = BIT(0);

	ret = hssi_set_dr_profile(priv->pdev_hssi, &dr_data);
	netdev_dbg(dev, "Step 6: Trigger reconfig addr_offs: %x Val: 0x%x\n",
		   dr_data.addr_offs, dr_data.val);
	if (ret)
		goto dr_failed;

	/* ------------------------------------------------------------
	 * Step 7: Wait for DR reconfiguration to complete
	 *
	 * After the trigger, the DR Nios firmware clears READY_FOR_DR
	 * while it reconfigures the transceiver. We poll until the bit
	 * is set again (= done) or the timeout expires.
	 *
	 * Poll interval: 100 µs  (fine-grained, DR usually < 500 ms)
	 * Timeout:       1.5 s   (covers worst-case cold reconfiguration)
	 * ------------------------------------------------------------
	 */
	{
		ktime_t dr_start = ktime_get();
		s64 elapsed_us;

		while (true) {
			dr_data.addr_offs = DYN_RCFG_DR_TRIGGER_REG;
			dr_data.val       = BIT(0);
			ret = hssi_get_dr_profile(priv->pdev_hssi, &dr_data);
			if (ret)
				goto dr_failed;

			if (dr_data.val & READY_FOR_DR) {
				elapsed_us = ktime_to_us(ktime_sub(ktime_get(), dr_start));
				netdev_info(dev,
					    "DR complete in %lld us (profile=%u speed=%u fec=%u)\n",
					    elapsed_us, target_profile,
					    speed_mbps, fec);
				break;
			}

			elapsed_us = ktime_to_us(ktime_sub(ktime_get(), dr_start));
			if (elapsed_us >= DR_COMPLETE_TIMEOUT_US) {
				netdev_err(dev,
					   "DR timed out after %lld us (profile=%u speed=%u fec=%u)\n",
					   elapsed_us, target_profile,
					   speed_mbps, fec);
				ret = -EBUSY;
				goto dr_failed;
			}

			usleep_range(DR_POLL_INTERVAL_US,
				     DR_POLL_INTERVAL_US + 50);
		}
	}

	dr_data.addr_offs = DYN_RCFG_DR_TX_SRC_ALARM_REG;
	ret = hssi_get_dr_profile(priv->pdev_hssi, &dr_data);
	if (ret)
		goto dr_failed;

	if (dr_data.val & BIT(priv->hssi_port)) {
		ret = -EBUSY;
		goto dr_failed;
	}
	netdev_dbg(dev, "Step 7: Wait TX SRC Alarm addr_offs: %x Val: 0x%x\n",
		   dr_data.addr_offs, dr_data.val);

	dr_data.addr_offs = DYN_RCFG_DR_RX_SRC_ALARM_REG;
	ret = hssi_get_dr_profile(priv->pdev_hssi, &dr_data);
	if (ret)
		goto dr_failed;

	if (dr_data.val & BIT(priv->hssi_port)) {
		ret = -EBUSY;
		goto dr_failed;
	}

	netdev_dbg(dev, " Step 7: Wait RX SRC Alarm addr_offs: %x Val: 0x%x\n",
		   dr_data.addr_offs, dr_data.val);
	dr_data.addr_offs = DYN_RCFG_LOCAL_ERROR_STAT_CTRL_REG;
	ret = hssi_get_dr_profile(priv->pdev_hssi, &dr_data);
	if (ret)
		goto dr_failed;

	if (dr_data.val & FIRMWARE_ERROR) {
		ret = -EBUSY;
		goto dr_failed;
	}

	netdev_dbg(dev, " Step 7: Wait LOCAL_ERROR_STAT_CTRL_REG  addr_offs: %x Val: 0x%x\n",
		   dr_data.addr_offs, dr_data.val);
	/* ------------------------------------------------------------
	 * Step 8: Update software-visible active profile
	 *
	 * At this point the DR engine has accepted the request.
	 * Functional validation (link up, PCS alignment, etc.) may
	 * still fail later and should be handled at a higher layer.
	 * ------------------------------------------------------------
	 */
	ftile_update_active_profile(dev, target_profile, fec);

	netdev_info(dev,
		    "switch: speed=%u Mbps fec=%u profile_id=%u\n",
		    speed_mbps, fec, target_profile);

	return 0;

dr_failed:
	/*
	 * The trigger write failed; the Nios firmware likely never
	 * started reconfiguring.  Restore software state to the
	 * previously active profile.
	 */
	netdev_warn(dev,
		    "DR failed for profile %u; NIOS might not be in proper state\n",
		    target_profile);
	return ret;
}

static void ftile_apply_mrip_speed_settings(struct net_device *dev,
					    u32 speed, u8 lane)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	u32 port_speed;

	switch (speed) {
	case SPEED_25000:

		hssi_csrwr32_ba(priv->pdev_hssi, HSSI_ETH_RECONFIG, priv->hssi_port,
				eth_soft_csroffs(profile_sel), 0x0);

		port_speed = hssi_csrrd32_ba(priv->pdev_hssi, HSSI_ETH_RECONFIG, priv->hssi_port,
					     eth_soft_csroffs(sel_25g_10g));

		hssi_csrwr32_ba(priv->pdev_hssi, HSSI_ETH_RECONFIG, priv->hssi_port,
				eth_soft_csroffs(sel_25g_10g), port_speed & ~(0x1 << lane));

		/* clear lane bit for 25G */
		ftile_pio_speed_set(priv, lane, SPEED_25000);
		break;

	case SPEED_10000:

		hssi_csrwr32_ba(priv->pdev_hssi, HSSI_ETH_RECONFIG, priv->hssi_port,
				eth_soft_csroffs(profile_sel), 0x0);

		port_speed = hssi_csrrd32_ba(priv->pdev_hssi, HSSI_ETH_RECONFIG, priv->hssi_port,
					     eth_soft_csroffs(sel_25g_10g));

		hssi_csrwr32_ba(priv->pdev_hssi, HSSI_ETH_RECONFIG, priv->hssi_port,
				eth_soft_csroffs(sel_25g_10g), port_speed | (BIT(0) << lane));

		/* set lane bit for 10G */
		ftile_pio_speed_set(priv, lane, SPEED_10000);
		break;
	default:
		break;
	}
}

static void ftile_apply_mrip_fec_settings(struct net_device *dev,
					  u32 requested_fec, u8 port)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	u32 shift = port * 3;
	u32 cur;

	/*
	 * The FTILE_FEC_* enum values match the MRIP hardware fec_mode
	 * register encoding directly:
	 *   FTILE_FEC_NONE  (0) -> 0x0  no FEC
	 *   FTILE_FEC_BASER (1) -> 0x1  Base-R / KR FEC
	 *   FTILE_FEC_RS    (2) -> 0x2  RS-FEC CL-91
	 *   FTILE_FEC_RS    (3) -> 0x3  RS-FEC CL-134
	 */
	cur = hssi_csrrd32_ba(priv->pdev_hssi, HSSI_ETH_RECONFIG, priv->hssi_port,
			      eth_soft_csroffs(fec_mode));

	cur = (cur & ~(GENMASK(2, 0) << shift)) | ((requested_fec & GENMASK(2, 0)) << shift);

	hssi_csrwr32_ba(priv->pdev_hssi, HSSI_ETH_RECONFIG, priv->hssi_port,
			eth_soft_csroffs(fec_mode), cur);
}

/* Set link ksettings (autoneg and forced speed/duplex) */
static int ftile_set_link_ksettings(struct net_device *dev,
				    const struct ethtool_link_ksettings *cmd)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	struct platform_device *pdev  = priv->pdev_hssi;
	u32 hssi_port = priv->hssi_port;
	u8  lane      = priv->hssi_rel_port;
	int ret = 0;

	if (netif_running(dev)) {
		netdev_err(dev, "interface must be down to change speed or autoneg mode\n");
		return -EINVAL;
	}

	ftile_pio_datapath_reset(priv, lane, true);

	if (cmd->base.autoneg == AUTONEG_ENABLE) {
		/*
		 * Autoneg requested: enable ANLT and leave the hardware to
		 * negotiate speed and FEC.  Profile DR for autoneg startup
		 * will be added here in a future patch.
		 */
		if (!priv->anlt) {
			ret = -EOPNOTSUPP;
			goto out_release_reset;
		}

		priv->autoneg = true;
		ret = hssi_anlt_enable(pdev, hssi_port);
		if (ret)
			priv->autoneg = false;
		
		goto out_release_reset;

	} else if (priv->anlt) {
		priv->autoneg = false;
		ret = hssi_anlt_disable(pdev, hssi_port);
		if (ret)
			priv->autoneg = true;
			
		goto out_release_reset;
	}

	/*
	 * Step 1: if a specific speed was requested, look up the matching
	 * profile and trigger reconfiguration.
	 * The current FEC mode is preserved where possible; if no profile
	 * exists for the requested speed + current FEC, no-fec is tried as
	 * a fallback.  Use ethtool --set-fec to change FEC mode explicitly.
	 */
	if (cmd->base.speed != SPEED_UNKNOWN) {
		u32 current_fec = ftile_get_active_fec_mode(priv);
		u32 prev_speed  = priv->link_speed;

		if (!priv->dr_supported) {
			netdev_err(dev, "speed change is not supported on this interface\n");
			ret = -EOPNOTSUPP;
			goto out_release_reset;
		}

		/*
		 * Configure MRIP registers for the target speed and current
		 * FEC before issuing the DR profile switch. The hardware
		 * firmware requires MRIP to reflect the new configuration
		 * prior to the trigger.
		 */
		ftile_apply_mrip_speed_settings(dev, cmd->base.speed, lane);
		ftile_apply_mrip_fec_settings(dev, current_fec, lane);

		ret = ftile_switch_profile(dev, cmd->base.speed, current_fec);
		if (ret == -ENOENT && current_fec != FTILE_FEC_NONE) {
			netdev_warn(dev,
				    "no profile for speed %u fec=%u, retrying with no-fec\n",
				    cmd->base.speed, current_fec);
			/* Update MRIP FEC to no-fec before the fallback switch. */
			ftile_apply_mrip_fec_settings(dev, FTILE_FEC_NONE, lane);
			ret = ftile_switch_profile(dev, cmd->base.speed,
						   FTILE_FEC_NONE);
		}
		if (ret) {
			/* Rollback MRIP to the previously active configuration. */
			ftile_apply_mrip_speed_settings(dev, prev_speed, lane);
			ftile_apply_mrip_fec_settings(dev, current_fec, lane);
			ret = (ret == -ENOENT) ? -EOPNOTSUPP : ret;
			goto out_release_reset;
		}

		priv->link_speed = cmd->base.speed;
	}

out_release_reset:
	ftile_pio_datapath_reset(priv, lane, false);
	return ret;
}

static int ftile_get_fecparam(struct net_device *dev,
			      struct ethtool_fecparam *fec)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	u32 mode = ftile_get_active_fec_mode(priv);
	u32 ethtool_mode;

	switch (mode) {
	case FTILE_FEC_RS:
		ethtool_mode = ETHTOOL_FEC_RS;
		break;
	case FTILE_FEC_BASER:
		ethtool_mode = ETHTOOL_FEC_BASER;
		break;
	default:
		ethtool_mode = ETHTOOL_FEC_OFF;
		break;
	}

	fec->fec        = ethtool_mode;
	fec->active_fec = ethtool_mode;

	return 0;
}

static int ftile_set_fecparam(struct net_device *dev,
			      struct ethtool_fecparam *fec)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	u32 requested_fec;
	u8  rel_port;
	int ret;

	if (!priv->dr_supported) {
		netdev_err(dev, "FEC mode change is not supported on this interface\n");
		return -EOPNOTSUPP;
	}

	if (netif_running(dev)) {
		netdev_err(dev, "interface must be down to change FEC mode\n");
		return -EBUSY;
	}

	rel_port = priv->hssi_rel_port;

	ftile_pio_datapath_reset(priv, rel_port, true);

	switch (fec->fec) {
	case ETHTOOL_FEC_OFF:
		requested_fec = FTILE_FEC_NONE;
		break;
	case ETHTOOL_FEC_RS:
		requested_fec = FTILE_FEC_RS;
		break;
	case ETHTOOL_FEC_BASER:
		requested_fec = FTILE_FEC_BASER;
		break;
	default:
		netdev_err(dev,
			   "unsupported FEC mode 0x%x; supported: off, rs, baser\n",
			   fec->fec);
		ret = -EINVAL;
		goto out_release_reset;
	}

	/* If already in the requested FEC mode there is nothing to do. */
	if (ftile_get_active_fec_mode(priv) == requested_fec) {
		ret = 0;
		goto out_release_reset;
	}

	/*
	 * Configure MRIP registers for the requested FEC (at the
	 * current speed) before issuing the DR profile switch. The
	 * hardware firmware requires MRIP to reflect the new
	 * configuration prior to the trigger.
	 */
	ftile_apply_mrip_speed_settings(dev, priv->link_speed, rel_port);
	ftile_apply_mrip_fec_settings(dev, requested_fec, rel_port);

	ret = ftile_switch_profile(dev, priv->link_speed, requested_fec);
	if (ret) {
		/* Rollback MRIP to the previously active FEC. */
		ftile_apply_mrip_speed_settings(dev, priv->link_speed, rel_port);
		ftile_apply_mrip_fec_settings(dev, ftile_get_active_fec_mode(priv), rel_port);
		if (ret == -ENOENT) {
			netdev_err(dev,
				   "requested FEC mode is not supported at the current speed (%u Mbps)\n",
				   priv->link_speed);
			ret = -EOPNOTSUPP;
		}
	}

out_release_reset:
	ftile_pio_datapath_reset(priv, rel_port, false);
	return ret;
}

/* Get link ksettings (phy address, speed) for ethtools */
static int ftile_get_link_ksettings(struct net_device *dev,
				    struct ethtool_link_ksettings *cmd)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	struct platform_device *pdev;
	u32 an_status = 0, an_sts_e[6], an_cfg;
	u32 an_tec_abl, an_con_abl, temp;

	phylink_ethtool_ksettings_get(priv->phylink, cmd);

	ethtool_link_ksettings_zero_link_mode(cmd, supported);
	ethtool_link_ksettings_zero_link_mode(cmd, advertising);
	ethtool_link_ksettings_zero_link_mode(cmd, lp_advertising);

	ethtool_link_ksettings_del_link_mode(cmd, supported, MII);
	ethtool_link_ksettings_add_link_mode(cmd, supported, FIBRE);
	ethtool_link_ksettings_add_link_mode(cmd, supported, TP);
	cmd->base.port = PORT_FIBRE;

	ethtool_link_ksettings_del_link_mode(cmd, supported, Pause);
	if ((priv->flow_ctrl & FLOW_TX) && (priv->flow_ctrl & FLOW_RX))
		ethtool_link_ksettings_add_link_mode(cmd, supported, Pause);
	if ((priv->flow_ctrl & FLOW_TX) || (priv->flow_ctrl & FLOW_RX))
		ethtool_link_ksettings_add_link_mode(cmd, supported, Asym_Pause);

	pdev = priv->pdev_hssi;
	if (priv->anlt)
		an_status = hssiss_anlt_get_status(pdev, priv->hssi_port);

	if (priv->anlt && an_status & HSSISS_CSR_PHY_AN_ABILITY) {
		ethtool_link_ksettings_add_link_mode(cmd, supported, Autoneg);

		an_cfg = hssiss_anlt_get_cfg(pdev, priv->hssi_port);
		if (!(an_cfg & HSSISS_CSR_AN_ENABLE_AN)) {
			ethtool_link_ksettings_zero_link_mode(cmd, advertising);
			ethtool_link_ksettings_zero_link_mode(cmd, supported);
		}

		if (an_status & HSSISS_CSR_AN_LP_ABILITY) {
			ethtool_link_ksettings_add_link_mode(cmd, lp_advertising, Autoneg);
			if (hssiss_anlt_get_ext_status(pdev, priv->hssi_port, an_sts_e) == 0) {
				if (an_sts_e[0] & HSSISS_CSR_AN_STATUS_1_LP_PAUSE_MASK) {
					if (an_sts_e[0] & HSSISS_CSR_AN_STATUS_1_LP_PAUSE_SUPP)
						ethtool_link_ksettings_add_link_mode(cmd,
										     lp_advertising,
										     Pause);
					if (an_sts_e[0] & HSSISS_CSR_AN_STATUS_1_LP_ASYM_PAUSE_SUPP)
						ethtool_link_ksettings_add_link_mode(cmd,
										     lp_advertising,
										     Asym_Pause);
				}
				if (an_sts_e[1] & HSSISS_AN2_LP_TECH_ABL_MASK) {
					an_tec_abl = an_sts_e[1] &
						     HSSISS_AN2_LP_TECH_ABL_MASK;
					an_tec_abl = altera_rm_trail0(an_tec_abl,
								      HSSISS_AN2_LP_TECH_ABL_START);
					set_ethtool_linkmodes(cmd->link_modes.lp_advertising,
							      AN_IEEE_ABILITY_TYPE, an_tec_abl);
				}
				if (an_sts_e[1] & HSSISS_CSR_AN_STATUS_2_LP_FEC_MASK) {
					if (an_sts_e[1] & HSSISS_CSR_AN_STATUS_2_LP_FEC_LLFEC)
						ethtool_link_ksettings_add_link_mode(cmd,
										     lp_advertising,
										     FEC_LLRS);
					if (an_sts_e[1] & HSSISS_CSR_AN_STATUS_2_LP_FEC_RSFEC)
						ethtool_link_ksettings_add_link_mode(cmd,
										     lp_advertising,
										     FEC_RS);
					if (an_sts_e[1] & HSSISS_CSR_AN_STATUS_2_LP_FEC_BASER)
						ethtool_link_ksettings_add_link_mode(cmd,
										     lp_advertising,
										     FEC_BASER);
				}
				if (an_status & HSSISS_CSR_CONSORTIUM_NEXT_PAGE_RCVD) {
					if (an_sts_e[5] &
						HSSISS_AN6_LP_CONS_NPG_ABL_MASK) {
						an_con_abl = an_sts_e[5] &
								HSSISS_AN6_LP_CONS_NPG_ABL_MASK_25G;
						an_con_abl = altera_rm_trail0(an_con_abl,
									      HSSISS_AN6_LP_CONS_NPG_ABL_MASK_25G_START);
						temp = an_sts_e[5] &
							HSSISS_AN6_LP_CONS_NPG_ABL_MASK_50G;
						temp = altera_rm_trail0(temp,
									HSSISS_AN6_LP_CONS_NPG_ABL_MASK_50G_START);
						temp = temp <<
						       HSSISS_AN_LP_NPG_ABL_MASK_25G_NUM_BITS;
						an_con_abl |= temp;
						set_ethtool_linkmodes(cmd->link_modes.lp_advertising,
								      AN_CONSORTIUM_ABILITY_TYPE,
								      an_con_abl);
					}
				}
				if (an_sts_e[3] & HSSISS_CSR_AN_STATUS_4_LP_NEXT_PAGE) {
					if (an_sts_e[3] & HSSISS_AN4_NPG_ABL_MASK) {
						an_con_abl = an_sts_e[3] &
							     HSSISS_AN4_NPG_ABL_MASK_25G;
						an_con_abl = altera_rm_trail0(an_con_abl,
									      HSSISS_AN4_NPG_ABL_MASK_25G_START);
						temp = an_sts_e[3] & HSSISS_AN4_NPG_ABL_MASK_50G;
						temp = altera_rm_trail0(temp,
									HSSISS_AN4_NPG_ABL_MASK_50G_START);
						temp = temp <<
						       HSSISS_AN_LP_NPG_ABL_MASK_25G_NUM_BITS;
						an_con_abl |= temp;
						temp = 0;
						temp = an_sts_e[3] & HSSISS_AN4_400GBASE_KR8_CR8;
						temp = altera_rm_trail0(temp,
									HSSISS_AN4_400GBASE_KR8_CR8);
						temp = temp <<
							(HSSISS_AN_LP_NPG_ABL_MASK_25G_NUM_BITS +
							HSSISS_AN_LP_NPG_ABL_MASK_50G_NUM_BITS);
						an_con_abl |= temp;
						set_ethtool_linkmodes(cmd->link_modes.lp_advertising,
								      AN_CONSORTIUM_ABILITY_TYPE,
								      an_con_abl);
					}
					if (an_sts_e[3] & HSSISS_AN4_F1_FEC_CTL_ADV_RSFEC)
						ethtool_link_ksettings_add_link_mode(cmd,
										     lp_advertising,
										     FEC_RS);
					if (an_sts_e[3] & HSSISS_AN4_F2_FEC_CTL_ADV_BASERFEC)
						ethtool_link_ksettings_add_link_mode(cmd,
										     lp_advertising,
										     FEC_BASER);
					if (an_sts_e[3] & HSSISS_AN4_LFR_LL_RSFEC_REQ)
						ethtool_link_ksettings_add_link_mode(cmd,
										     lp_advertising,
										     FEC_LLRS);
				}
			}
		} else {
			ethtool_link_ksettings_zero_link_mode(cmd, lp_advertising);
		}

		if (an_status & HSSISS_CSR_AN_COMPLETE) {
			ethtool_link_ksettings_add_link_mode(cmd, advertising, Autoneg);
			if (an_status & HSSISS_CSR_IEEE_NEG_PORT_MASK) {
				an_tec_abl = an_status & HSSISS_CSR_IEEE_NEG_PORT_MASK;
				an_tec_abl = altera_rm_trail0(an_tec_abl,
							      HSSISS_CSR_IEEE_NEG_PORT_MASK_START);
				set_ethtool_linkmodes(cmd->link_modes.supported,
						      AN_IEEE_NEG_ABILITY_TYPE, an_tec_abl);
				set_ethtool_linkmodes(cmd->link_modes.advertising,
						      AN_IEEE_NEG_ABILITY_TYPE, an_tec_abl);
			}
			if (an_status & HSSISS_CSR_CONS_NEG_PORT_MASK) {
				an_con_abl = an_status & HSSISS_CSR_CONS_NEG_PORT_MASK;
				an_con_abl = altera_rm_trail0(an_con_abl,
							      HSSISS_CSR_CONS_NEG_PORT_MASK_START);
				set_ethtool_linkmodes(cmd->link_modes.supported,
						      AN_CONSORTIUM_ABILITY_TYPE, an_con_abl);
				set_ethtool_linkmodes(cmd->link_modes.advertising,
						      AN_CONSORTIUM_ABILITY_TYPE, an_con_abl);
			}
			if (an_status & HSSISS_CSR_CONS_NEG_PORT_400_KR4_CR4) {
				an_tec_abl = an_status & HSSISS_CSR_CONS_NEG_PORT_400_KR4_CR4;
				an_tec_abl = altera_rm_trail0(an_tec_abl,
							      HSSISS_IEEE_EXT_NEG_PORT_MASK_START);
				set_ethtool_linkmodes(cmd->link_modes.supported,
						      AN_IEEE_NEG_ABILITY_TYPE_EXT, an_tec_abl);
				set_ethtool_linkmodes(cmd->link_modes.advertising,
						      AN_IEEE_NEG_ABILITY_TYPE_EXT, an_tec_abl);
			}
			if (an_status & HSSISS_CSR_FEC_MODES_MASK) {
				if (an_status & HSSISS_CSR_RS_FEC_NEGOTIATED) {
					ethtool_link_ksettings_add_link_mode(cmd, supported,
									     FEC_RS);
					ethtool_link_ksettings_add_link_mode(cmd, advertising,
									     FEC_RS);
				}
				if (an_status & HSSISS_CSR_LL_FEC_NEGOTIATED) {
					ethtool_link_ksettings_add_link_mode(cmd, supported,
									     FEC_LLRS);
					ethtool_link_ksettings_add_link_mode(cmd, advertising,
									     FEC_LLRS);
				}
			}
			cmd->base.speed = priv->link_speed;
			cmd->base.duplex = priv->duplex;
			cmd->base.autoneg = AUTONEG_ENABLE;
		}
	} else {
		cmd->base.autoneg = AUTONEG_DISABLE;
		cmd->base.speed = priv->link_speed;
		cmd->base.duplex = priv->duplex;
	}

	return 0;
}

static int ftile_get_module_info(struct net_device *dev, struct ethtool_modinfo *info)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	if (!priv)
		return -ENODEV;

	if (!priv->phylink || !priv->dev || !priv->dev->qsfp_bus)
		return -ENODEV;

	return qsfp_get_module_info(priv->dev->qsfp_bus, info);
}

static int ftile_get_module_eeprom(struct net_device *dev, struct ethtool_eeprom *eeprom, u8 *data)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	if (!priv)
		return -ENODEV;

	if (!priv->phylink || !priv->dev || !priv->dev->qsfp_bus)
		return -ENODEV;

	return qsfp_get_module_eeprom(priv->dev->qsfp_bus, eeprom, data);
}

static const struct ethtool_ops ftile_ethtool_ops = {
	.get_drvinfo = ftile_get_drvinfo,
	.get_regs_len = ftile_reglen,
	.get_regs = ftile_get_regs,
	.get_link = ethtool_op_get_link,
	.get_strings = ftile_gstrings,
	.get_sset_count = ftile_sset_count,
	.get_ethtool_stats = ftile_fill_stats,
	.get_msglevel = ftile_get_msglevel,
	.set_msglevel = ftile_set_msglevel,
	.get_pauseparam = ftile_get_pauseparam,
	.set_pauseparam = ftile_set_pauseparam,
	.get_ts_info = ftile_get_ts_info,
	.get_link_ksettings = ftile_get_link_ksettings,
	.set_link_ksettings = ftile_set_link_ksettings,
	.get_fecparam = ftile_get_fecparam,
	.set_fecparam = ftile_set_fecparam,
	.get_priv_flags = ftile_get_priv_flags,
	.get_module_info = ftile_get_module_info,
	.get_module_eeprom = ftile_get_module_eeprom,
};

void intel_fpga_ftile_set_ethtool_ops(struct net_device *netdev)
{
	netdev->ethtool_ops = &ftile_ethtool_ops;
}
