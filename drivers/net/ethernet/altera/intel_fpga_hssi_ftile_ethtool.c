// SPDX-License-Identifier: GPL-2.0
/* Ethtool support for Intel FPGA F-tile Ethernet MAC driver
 * Copyright (C) 2019-2022 Intel Corporation. All rights reserved
 *
 * Contributors:
 *   Roman Bulgakov
 *   Yu Ying Choo
 *   Dalon Westergreen
 *   Joyce Ooi
 *
 * Original driver contributed by GlobalLogic.
 */

#include <linux/ethtool.h>
#include <linux/kernel.h>
#include <linux/net_tstamp.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/phylink.h>


#include "altera_eth_dma.h"
#include "intel_fpga_eth_main.h"
#include "intel_fpga_eth_ftile.h"
#include "intel_fpga_eth_hssi_itf.h"

#define FTILE_STATS_LEN	ARRAY_SIZE(stat_gstrings)
#define FTILE_NUM_REGS	555


static char const stat_gstrings[][ETH_GSTRING_LEN] = {
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
	memcpy(buf, stat_gstrings, FTILE_STATS_LEN * ETH_GSTRING_LEN);
}

static void ftile_fill_stats(struct net_device *dev, struct ethtool_stats *dummy,
			     u64 *buf)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	struct platform_device *pdev  = priv->pdev_hssi;
	u32 hssi_port = priv->hssi_port;

	/* Tx packets */
	buf[0] = hssi_read_mac_stats64(pdev, hssi_port , MACSTAT_TX_PACKETS);

	/* Rx packets */
	buf[1] = hssi_read_mac_stats64(pdev,hssi_port, MACSTAT_RX_PACKETS);

	/* Rx CRC error packets */
	buf[2] = hssi_read_mac_stats64(pdev,hssi_port, MACSTAT_RX_CRC_ERRORS);

	/* Rx align error packets */
	buf[3] = hssi_read_mac_stats64(pdev,hssi_port, MACSTAT_RX_ALIGN_ERRORS);

	/* Tx bytes */
	buf[4] = hssi_read_mac_stats64(pdev,hssi_port, MACSTAT_TX_BYTES);

	/* Rx bytes */
	buf[5] = hssi_read_mac_stats64(pdev,hssi_port, MACSTAT_RX_BYTES);

	/* Tx pause bytes */
	buf[6] = hssi_read_mac_stats64(pdev,hssi_port, MACSTAT_TX_PAUSE);

	/* Rx pause bytes */
	buf[7] = hssi_read_mac_stats64(pdev,hssi_port, MACSTAT_RX_PAUSE);

	/* Rx error bytes */
	buf[8] = hssi_read_mac_stats64(pdev,hssi_port, MACSTAT_RX_ERRORS);

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
	buf[25] = hssi_read_mac_stats64(pdev,hssi_port, MACSTAT_RX_256_511_BYTES);

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
	default:
		return -EOPNOTSUPP;
	}
}

static u32 ftile_get_msglevel(struct net_device *dev)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	return priv->msg_enable;
}

static void ftile_set_msglevel(struct net_device *dev, uint32_t data)
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
	buf[0] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(gui_option));				// 0x0100
	buf[1] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(qhip_scratch));			// 0x0104
	buf[2] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(eth_reset));				// 0x0108
	buf[3] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(eth_reset_status));			// 0x010C
	buf[4] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(phy_tx_pll_locked));			// 0x0110
	buf[5] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(phy_eiofreq_locked));			// 0x0114
	buf[6] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(pcs_status));				// 0x0118
	buf[7] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(pcs_control));				// 0x011C
	buf[8] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(link_fault_status));			// 0x0120
	buf[9] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(clk_tx_khz));				// 0x0128
	buf[10] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(clk_rx_khz));				// 0x012C
	buf[11] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(clk_pll_khz));			// 0x0130
	buf[12] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(clk_tx_div_khz));			// 0x0134
	buf[13] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(clk_rec_div64_khz));			// 0x0138
	buf[14] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(clk_rec_div_khz));			// 0x013C
	buf[15] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(rxmac_adapt_dropped_31_0));		// 0x0140
	buf[16] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(rxmac_adapt_dropped_63_32));		// 0x0144
	buf[17] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(rxmac_adapt_dropped_control));	// 0x0148
	buf[18] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(anlt_sequencer_config));		// 0x02C0
	buf[19] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(anlt_sequencer_status));		// 0x02C4
	buf[20] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(auto_neg_conf_1));			// 0x0300
	buf[21] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(auto_neg_conf_2));			// 0x0304
	buf[22] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(auto_neg_stat));			// 0x0308
	buf[23] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(auto_neg_conf_3));			// 0x030C
	buf[24] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(auto_neg_conf_5));			// 0x0314
	buf[25] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(auto_neg_conf_6));			// 0x0318
	buf[26] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(auto_neg_stat_1));			// 0x031C
	buf[27] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(auto_neg_stat_2));			// 0x0320
	buf[28] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(auto_neg_stat_3));			// 0x0324
	buf[29] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(auto_neg_stat_4));			// 0x0328
	buf[30] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(auto_neg_an_channel_override));	// 0x0330
	buf[31] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(auto_neg_const_next_page_lp_stat));	// 0x0338
	buf[32] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(link_train_conf_1));			// 0x0340
	buf[33] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(link_train_conf_2));			// 0x0344
	buf[34] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(link_train_stat_1));			// 0x0348
	buf[35] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_tam_adjust));			// 0x0800
	buf[36] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_tam_adjust));			// 0x0804
	buf[37] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_ref_lane));			// 0x080C
	buf[38] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_dr_cfg));				// 0x0810
	buf[39] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_user_cfg_status));		// 0x0814
	buf[40] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_user_cfg_status));		// 0x0818
	buf[41] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_uim_tam_snapshot));		// 0x081C
	buf[42] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_uim_tam_info0));		// 0x0820
	buf[43] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_uim_tam_info1));		// 0x0824
	buf[44] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_uim_tam_info0));		// 0x0828
	buf[45] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_uim_tam_info1));		// 0x082C
	buf[46] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_status));				// 0x0830
	buf[47] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_status2));			// 0x0840
	buf[48] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane_calc_data_constdelay));	// 0x08F0
	buf[49] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane_calc_data_constdelay));	// 0x08F4
	buf[50] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane0_calc_data_offset));	// 0x0900
	buf[51] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane0_calc_data_offset));	// 0x0904
	buf[52] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane0_calc_data_time));	// 0x0908
	buf[53] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane0_calc_data_time));	// 0x090C
	buf[54] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane0_calc_data_wiredelay));	// 0x0910
	buf[55] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane0_calc_data_wiredelay));	// 0x0914
	buf[56] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane1_calc_data_offset));	// 0x0920
	buf[57] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane1_calc_data_offset));	// 0x0924
	buf[58] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane1_calc_data_time));	// 0x0928
	buf[59] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane1_calc_data_time));	// 0x092C
	buf[60] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane1_calc_data_wiredelay));	// 0x0930
	buf[61] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane1_calc_data_wiredelay));	// 0x0934
	buf[62] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane2_calc_data_offset));	// 0x0940
	buf[63] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane2_calc_data_offset));	// 0x0944
	buf[64] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane2_calc_data_time));	// 0x0948
	buf[65] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane2_calc_data_time));	// 0x094C
	buf[66] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane2_calc_data_wiredelay));	// 0x0950
	buf[67] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane2_calc_data_wiredelay));	// 0x0954
	buf[68] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane3_calc_data_offset));	// 0x0960
	buf[69] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane3_calc_data_offset));	// 0x0964
	buf[70] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane3_calc_data_time));	// 0x0968
	buf[71] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane3_calc_data_time));	// 0x096C
	buf[72] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane3_calc_data_wiredelay));	// 0x0970
	buf[73] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane3_calc_data_wiredelay));	// 0x0974
	buf[74] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane4_calc_data_offset));	// 0x0980
	buf[75] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane4_calc_data_offset));	// 0x0984
	buf[76] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane4_calc_data_time));	// 0x0988
	buf[77] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane4_calc_data_time));	// 0x098C
	buf[78] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane4_calc_data_wiredelay));	// 0x0990
	buf[79] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane4_calc_data_wiredelay));	// 0x0994
	buf[80] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane5_calc_data_offset));	// 0x09A0
	buf[81] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane5_calc_data_offset));	// 0x09A4
	buf[82] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane5_calc_data_time));	// 0x09A8
	buf[83] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane5_calc_data_time));	// 0x09AC
	buf[84] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane5_calc_data_wiredelay));	// 0x09B0
	buf[85] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane5_calc_data_wiredelay));	// 0x09B4
	buf[86] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane6_calc_data_offset));	// 0x09C0
	buf[87] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane6_calc_data_offset));	// 0x09C4
	buf[88] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane6_calc_data_time));	// 0x09C8
	buf[89] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane6_calc_data_time));	// 0x09CC
	buf[90] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane6_calc_data_wiredelay));	// 0x09D0
	buf[91] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane6_calc_data_wiredelay));	// 0x09D4
	buf[92] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane7_calc_data_offset));	// 0x09E0
	buf[93] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane7_calc_data_offset));	// 0x09E4
	buf[94] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane7_calc_data_time));	// 0x09E8
	buf[95] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane7_calc_data_time));	// 0x09EC
	buf[96] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_tx_lane7_calc_data_wiredelay));	// 0x09F0
	buf[97] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_rx_lane7_calc_data_wiredelay));	// 0x09F4

	/* F-tile EHIP PHY registers: */
	/* 0x000 - 0x07C : PCS Config */
	buf[98] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, config_ctrl));				// 0x000
	buf[99] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, ehip_reset_and_debug));		// 0x00C
	buf[100] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_tx_pld_conf));			// 0x010
	buf[101] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_rx_pld_conf));			// 0x014
	buf[102] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_ehip_pcs_modes));			// 0x048
	buf[103] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_rx_pcs_conf));			// 0x04C
	buf[104] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_tx_am_enc[0]));			// 0x050
	buf[105] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_tx_am_enc[1]));			// 0x054
	buf[106] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_tx_am_enc[2]));			// 0x058
	buf[107] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_tx_am_enc[3]));			// 0x05C
	buf[108] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_rx_am_enc[0]));			// 0x060
	buf[109] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_rx_am_enc[1]));			// 0x064
	buf[110] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_rx_am_enc[2]));			// 0x068
	buf[111] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_rx_am_enc[3]));			// 0x06C
	buf[112] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_timer_window_hiber_check));	// 0x070
	buf[113] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_hiber_frm_err));			// 0x074
	buf[114] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_pcs_err_inject));			// 0x078
	// 0x080 - 0x1FC : PCS Status
	buf[115] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_frm_err_detect));			// 0x080
	buf[116] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_pcs_stat_anlt));			// 0x084
	buf[117] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_am_lock));			// 0x088
	buf[118] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_lanes_deskewed));			// 0x08C
	buf[119] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_ber_cnt));			// 0x090
	buf[120] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_pcs_virtual_ln_0));		// 0x094
	buf[121] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_pcs_virtual_ln_1));		// 0x098
	buf[122] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_pcs_virtual_ln_2));		// 0x09C
	buf[123] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_pcs_virtual_ln_3));		// 0x0A0
	for (i = 0; i < 20; ++i)
		buf[124+i] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_bip_cnt[i]));		// 0x0A4 - 0x0F0
	buf[144] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_err_block_cnt));			// 0x0F4
	buf[145] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_deskew_dept[0]));			// 0x0FC
	buf[146] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_deskew_dept[1]));			// 0x100
	buf[147] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_deskew_dept[2]));			// 0x104
	buf[148] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_deskew_dept[3]));			// 0x108
	buf[149] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_rx_pcs_test_err_cnt));		// 0x10C
	for (i = 0; i < 20; ++i)
		buf[150+i] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, ptp_vl_data_lsb[i]));	// 0x114 - 0x160
	for (i = 0; i < 20; ++i)
		buf[170+i] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, ptp_vl_data_msb[i]));	// 0x164 - 0x1B0
	buf[190] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, ptp_lal));				// 0x1B4
	buf[191] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(0, phy_tx_pld_stat));			// 0x1C0

	/* F-tile EHIP MAC/PTP registers: */
	/* 0x200 - 0x7FC : MAC/PTP Config */
	buf[192] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_mac_link_fault));				// 0x200
	buf[193] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_mac_ipg_col_rem));				// 0x204
	buf[194] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_mac_max_frm_size));				// 0x208
	buf[195] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_mac_conf));					// 0x20C
	buf[196] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_mac_ehip_conf));				// 0x210
	buf[197] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_mac_source_addr_lower_bytes));			// 0x214
	buf[198] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_mac_source_addr_higher_bytes));		// 0x218
	buf[199] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_mac_max_frm_size));				// 0x21C
	buf[200] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_mac_frwd_rx_crc));				// 0x220
	buf[201] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_mac_conf));					// 0x224
	buf[202] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_mac_ehip_conf));				// 0x228
	buf[203] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, enable_tx_pause_ports));				// 0x22C
	buf[204] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_pause_request));				// 0x230
	buf[205] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, enable_automatic_tx_pause_retransmission));	// 0x234
	buf[206] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, retransmit_holdoff_quanta));			// 0x238
	buf[207] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, retransmit_pause_quanta));			// 0x23C
	buf[208] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, enable_tx_xoff));					// 0x240
	buf[209] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, enable_uniform_holdoff));				// 0x244
	buf[210] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, set_uniform_holdoff));				// 0x248
	buf[211] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, flow_control_fields_lsb));			// 0x24C
	buf[212] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, flow_control_fields_msb));			// 0x250
	buf[213] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, flow_control_frames_lsb));			// 0x254
	buf[214] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, flow_control_frames_msb));			// 0x258
	buf[215] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_flow_control_feature_cfg));			// 0x25C
	buf[216] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, enable_rx_pause_frame_processing_fields));	// 0x260
	buf[217] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, forward_flow_control_frames));			// 0x264
	buf[218] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_pause_frames_lsb));				// 0x268
	buf[219] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_pause_frames_msb));				// 0x26C
	buf[220] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_flow_control_feature_cfg));			// 0x270
	buf[221] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_cntr_config));					// 0x274
	buf[222] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_cntr_config));					// 0x278
	buf[223] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, pause_quanta_0));					// 0x284
	buf[224] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, pause_quanta_1));					// 0x288
	buf[225] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, pause_quanta_2));					// 0x28C
	buf[226] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, pause_quanta_3));					// 0x290
	buf[227] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, pause_quanta_4));					// 0x294
	buf[228] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, pause_quanta_5));					// 0x298
	buf[229] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, pause_quanta_6));					// 0x29C
	buf[230] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, pause_quanta_7));					// 0x2A0
	buf[231] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, pfc_holdoff_quanta_0));				// 0x2A4
	buf[232] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, pfc_holdoff_quanta_1));				// 0x2A8
	buf[233] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, pfc_holdoff_quanta_2));				// 0x2AC
	buf[234] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, pfc_holdoff_quanta_3));				// 0x2B0
	buf[235] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, pfc_holdoff_quanta_4));				// 0x2B4
	buf[236] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, pfc_holdoff_quanta_5));				// 0x2B8
	buf[237] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, pfc_holdoff_quanta_6));				// 0x2BC
	buf[238] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, pfc_holdoff_quanta_7));				// 0x2C0
	buf[239] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_extra_latency));				// 0x2E0
	buf[240] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_ui));					// 0x2E4
	buf[241] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_phy_lane_num));				// 0x2EC
	buf[242] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_ap_filter));				// 0x2F0
	for (i = 0; i < 20; ++i)
		buf[243+i] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_vl_offset[i]));			// 0x2F4 - 0x340
	buf[263] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_extra_latency));				// 0x344
	buf[264] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_ui));					// 0x348
	buf[265] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_phy_lane_num));				// 0x350
	buf[266] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_ap_filter));				// 0x354
	for (i = 0; i < 20; ++i)
		buf[267+i] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_vl_offset[i]));			// 0x358 - 0x3A4
	for (i = 0; i < 20; ++i)
		buf[287+i] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_vl_to_pl[i]));			// 0x3A8 - 0x3F4
	buf[307] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_pkt_n_ts_rx_ctr));				// 0x3F8
	/* 0x800 - 0xFFC : MAC/PTP Statistics */
	buf[308] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_fragments_lsb));				// 0x800
	buf[309] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_jabbers_lsb));					// 0x808
	buf[310] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_fcserr_lsb));					// 0x818
	buf[311] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_mcast_data_err_lsb));				// 0x820
	buf[312] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_bcast_data_err_lsb));				// 0x828
	buf[313] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ucast_data_err_lsb));				// 0x830
	buf[314] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_mcast_ctrl_err_lsb));				// 0x838
	buf[315] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_bcast_ctrl_err_lsb));				// 0x840
	buf[316] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ucast_ctrl_err_lsb));				// 0x848
	buf[317] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_pause_err_lsb));				// 0x850
	buf[318] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_64b_lsb));					// 0x858
	buf[319] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_64b_msb));					// 0x85C
	buf[320] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_65to127b_lsb));				// 0x860
	buf[321] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_65to127b_msb));				// 0x864
	buf[322] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_128to255b_lsb));				// 0x868
	buf[323] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_128to255b_msb));				// 0x86C
	buf[324] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_256to511b_lsb));				// 0x870
	buf[325] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_256to511b_msb));				// 0x874
	buf[326] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_512to1023b_lsb));				// 0x878
	buf[327] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_512to1023b_msb));				// 0x87C
	buf[328] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_1024to1518b_lsb));				// 0x880
	buf[329] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_1024to1518b_msb));				// 0x884
	buf[330] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_1519tomaxb_lsb));				// 0x888
	buf[331] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_1519tomaxb_msb));				// 0x88C
	buf[332] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_oversize_lsb));				// 0x890
	buf[333] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_mcast_data_ok_lsb));				// 0x898
	buf[334] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_mcast_data_ok_msb));				// 0x89C
	buf[335] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_bcast_data_ok_lsb));				// 0x8A0
	buf[336] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_bcast_data_ok_msb));				// 0x8A4
	buf[337] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ucast_data_ok_lsb));				// 0x8A8
	buf[338] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ucast_data_ok_msb));				// 0x8AC
	buf[339] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_mcast_ctrl_ok_lsb));				// 0x8B0
	buf[340] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_mcast_ctrl_ok_msb));				// 0x8B4
	buf[341] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_bcast_ctrl_ok_lsb));				// 0x8B8
	buf[342] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_bcast_ctrl_ok_msb));				// 0x8BC
	buf[343] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ucast_ctrl_ok_lsb));				// 0x8C0
	buf[344] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ucast_ctrl_ok_msb));				// 0x8C4
	buf[345] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_pause_lsb));					// 0x8C8
	buf[346] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_pause_msb));					// 0x8CC
	buf[347] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_rnt_lsb));					// 0x8D0
	buf[348] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_st_lsb));					// 0x8D8
	buf[349] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_st_msb));					// 0x8DC
	buf[350] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_lenerr_lsb));					// 0x8E0
	buf[351] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_pfc_err_lsb));					// 0x8E8
	buf[352] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_pfc_lsb));					// 0x8F0
	buf[353] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_pfc_msb));					// 0x8F4
	buf[354] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_payload_octetsok_lsb));			// 0x8F8
	buf[355] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_payload_octetsok_msb));			// 0x8FC
	buf[356] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_frame_octetsok_lsb));				// 0x900
	buf[357] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_frame_octetsok_msb));				// 0x904
	buf[358] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_malformed_ctrl_lsb));				// 0x908
	buf[359] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_dropped_ctrl_lsb));				// 0x910
	buf[360] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_badlt_ctrl_lsb));				// 0x918
	buf[361] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_total_ptp_pkts));				// 0x920
	buf[362] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_total_1step_ptp_pkts));			// 0x924
	buf[363] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_total_2step_ptp_pkts));			// 0x928
	buf[364] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_total_v1_ptp_pkts));				// 0x92C
	buf[365] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_total_v2_ptp_pkts));				// 0x930
	buf[366] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_fragments_lsb));				// 0x934
	buf[367] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_jabbers_lsb));					// 0x93C
	buf[368] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_fcserr_lsb));					// 0x94C
	buf[369] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_mcast_data_err_lsb));				// 0x954
	buf[370] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_bcast_data_err_lsb));				// 0x95C
	buf[371] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ucast_data_err_lsb));				// 0x964
	buf[372] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_mcast_ctrl_err_lsb));				// 0x96C
	buf[373] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_bcast_ctrl_err_lsb));				// 0x974
	buf[374] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ucast_ctrl_err_lsb));				// 0x97C
	buf[375] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_pause_err_lsb));				// 0x984
	buf[376] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_64b_lsb));					// 0x98C
	buf[377] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_64b_msb));					// 0x990
	buf[378] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_65to127b_lsb));				// 0x994
	buf[379] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_65to127b_msb));				// 0x998
	buf[380] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_128to255b_lsb));				// 0x99C
	buf[381] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_128to255b_msb));				// 0x9A0
	buf[382] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_256to511b_lsb));				// 0x9A4
	buf[383] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_256to511b_msb));				// 0x9A8
	buf[384] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_512to1023b_lsb));				// 0x9AC
	buf[385] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_512to1023b_msb));				// 0x9B0
	buf[386] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_1024to1518b_lsb));				// 0x9B4
	buf[387] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_1024to1518b_msb));				// 0x9B8
	buf[388] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_1519tomaxb_lsb));				// 0x9BC
	buf[389] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_1519tomaxb_msb));				// 0x9C0
	buf[390] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_oversize_lsb));				// 0x9C4
	buf[391] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_mcast_data_ok_lsb));				// 0x9CC
	buf[392] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_mcast_data_ok_msb));				// 0x9D0
	buf[393] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_bcast_data_ok_lsb));				// 0x9D4
	buf[394] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_bcast_data_ok_msb));				// 0x9D8
	buf[395] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ucast_data_ok_lsb));				// 0x9DC
	buf[396] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ucast_data_ok_msb));				// 0x9E0
	buf[397] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_mcast_ctrl_ok_lsb));				// 0x9E4
	buf[398] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_mcast_ctrl_ok_msb));				// 0x9E8
	buf[399] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_bcast_ctrl_ok_lsb));				// 0x9EC
	buf[400] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_bcast_ctrl_ok_msb));				// 0x9F0
	buf[401] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ucast_ctrl_ok_lsb));				// 0x9F4
	buf[402] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ucast_ctrl_ok_msb));				// 0x9F8
	buf[403] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_pause_lsb));					// 0x9FC
	buf[404] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_pause_msb));					// 0xA00
	buf[405] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_rnt_lsb));					// 0xA04
	buf[406] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_st_lsb));					// 0xA0C
	buf[407] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_st_msb));					// 0xA10
	buf[408] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_lenerr_lsb));					// 0xA14
	buf[409] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_pfc_err_lsb));					// 0xA1C
	buf[410] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_pfc_lsb));					// 0xA24
	buf[411] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_pfc_msb));					// 0xA28
	buf[412] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_payload_octetsok_lsb));			// 0xA2C
	buf[413] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_payload_octetsok_msb));			// 0xA30
	buf[414] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_frame_octetsok_lsb));				// 0xA34
	buf[415] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_frame_octetsok_msb));				// 0xA38
	buf[416] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_malformed_lsb));				// 0xA3C
	buf[417] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_dropped_lsb));					// 0xA44
	buf[418] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_badlt_lsb));					// 0xA4C
	buf[419] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_total_ptp_ts));				// 0xA54
	buf[420] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_cf_overflow));				// 0xA58
	buf[421] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_lo_pl_0));				// 0xA5C
	buf[422] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_med_pl_0));				// 0xA60
	buf[423] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_hi_pl_0));				// 0xA64
	buf[424] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_adj_pl_0));				// 0xA68
	buf[425] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_lo_pl_1));				// 0xA6C
	buf[426] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_med_pl_1));				// 0xA70
	buf[427] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_hi_pl_1));				// 0xA74
	buf[428] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_adj_pl_1));				// 0xA78
	buf[429] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_lo_pl_2));				// 0xA7C
	buf[430] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_med_pl_2));				// 0xA80
	buf[431] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_hi_pl_2));				// 0xA84
	buf[432] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_adj_pl_2));				// 0xA88
	buf[433] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_lo_pl_3));				// 0xA8C
	buf[434] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_med_pl_3));				// 0xA90
	buf[435] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_hi_pl_3));				// 0xA94
	buf[436] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_adj_pl_3));				// 0xA98
	buf[437] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_lo_pl_4));				// 0xA9C
	buf[438] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_med_pl_4));				// 0xAA0
	buf[439] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_hi_pl_4));				// 0xAA4
	buf[440] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_adj_pl_4));				// 0xAA8
	buf[441] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_lo_pl_5));				// 0xAAC
	buf[442] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_med_pl_5));				// 0xAB0
	buf[443] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_hi_pl_5));				// 0xAB4
	buf[444] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_adj_pl_5));				// 0xAB8
	buf[445] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_lo_pl_6));				// 0xABC
	buf[446] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_med_pl_6));				// 0xAC0
	buf[447] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_hi_pl_6));				// 0xAC4
	buf[448] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_adj_pl_6));				// 0xAC8
	buf[449] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_lo_pl_7));				// 0xACC
	buf[450] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_med_pl_7));				// 0xAD0
	buf[451] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_hi_pl_7));				// 0xAD4
	buf[452] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_adj_pl_7));				// 0xAD8
	buf[453] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_lo_pl_8));				// 0xADC
	buf[454] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_med_pl_8));				// 0xAE0
	buf[455] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_hi_pl_8));				// 0xAE4
	buf[456] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_adj_pl_8));				// 0xAE8
	buf[457] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_lo_pl_9));				// 0xAEC
	buf[458] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_med_pl_9));				// 0xAF0
	buf[459] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_hi_pl_9));				// 0xAF4
	buf[460] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_adj_pl_9));				// 0xAF8
	buf[461] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_lo_pl_10));				// 0xAFC
	buf[462] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_med_pl_10));				// 0xB00
	buf[463] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_hi_pl_10));				// 0xB04
	buf[464] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_adj_pl_10));				// 0xB08
	buf[465] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_lo_pl_11));				// 0xB0C
	buf[466] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_med_pl_11));				// 0xB10
	buf[467] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_hi_pl_11));				// 0xB14
	buf[468] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_adj_pl_11));				// 0xB18
	buf[469] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_lo_pl_12));				// 0xB1C
	buf[470] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_med_pl_12));				// 0xB20
	buf[471] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_hi_pl_12));				// 0xB24
	buf[472] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_adj_pl_12));				// 0xB28
	buf[473] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_lo_pl_13));				// 0xB2C
	buf[474] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_med_pl_13));				// 0xB30
	buf[475] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_hi_pl_13));				// 0xB34
	buf[476] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_adj_pl_13));				// 0xB38
	buf[477] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_lo_pl_14));				// 0xB3C
	buf[478] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_med_pl_14));				// 0xB40
	buf[479] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_hi_pl_14));				// 0xB44
	buf[480] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_adj_pl_14));				// 0xB48
	buf[481] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_lo_pl_15));				// 0xB4C
	buf[482] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_med_pl_15));				// 0xB50
	buf[483] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_hi_pl_15));				// 0xB54
	buf[484] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ptp_tam_adj_pl_15));				// 0xB58
	buf[485] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ts_ss_lo));					// 0xB5C
	buf[486] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ts_ss_mid));					// 0xB60
	buf[487] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, tx_ts_ss_hi));					// 0xB64
	buf[488] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_lo_pl_0));				// 0xB6C
	buf[489] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_med_pl_0));				// 0xB70
	buf[490] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_hi_pl_0));				// 0xB74
	buf[491] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_adj_pl_0));				// 0xB78
	buf[492] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_lo_pl_1));				// 0xB7C
	buf[493] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_med_pl_1));				// 0xB80
	buf[494] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_hi_pl_1));				// 0xB84
	buf[495] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_adj_pl_1));				// 0xB88
	buf[496] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_lo_pl_2));				// 0xB8C
	buf[497] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_med_pl_2));				// 0xB90
	buf[498] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_hi_pl_2));				// 0xB94
	buf[499] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_adj_pl_2));				// 0xB98
	buf[500] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_lo_pl_3));				// 0xB9C
	buf[501] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_med_pl_3));				// 0xBA0
	buf[502] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_hi_pl_3));				// 0xBA4
	buf[503] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_adj_pl_3));				// 0xBA8
	buf[504] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_lo_pl_4));				// 0xBAC
	buf[505] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_med_pl_4));				// 0xBB0
	buf[506] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_hi_pl_4));				// 0xBB4
	buf[507] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_adj_pl_4));				// 0xBB8
	buf[508] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_lo_pl_5));				// 0xBBC
	buf[509] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_med_pl_5));				// 0xBC0
	buf[510] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_hi_pl_5));				// 0xBC4
	buf[511] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_adj_pl_5));				// 0xBC8
	buf[512] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_lo_pl_6));				// 0xBCC
	buf[513] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_med_pl_6));				// 0xBD0
	buf[514] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_hi_pl_6));				// 0xBD4
	buf[515] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_adj_pl_6));				// 0xBD8
	buf[516] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_lo_pl_7));				// 0xBDC
	buf[517] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_med_pl_7));				// 0xBE0
	buf[518] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_hi_pl_7));				// 0xBE4
	buf[519] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_adj_pl_7));				// 0xBE8
	buf[520] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_lo_pl_8));				// 0xBEC
	buf[521] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_med_pl_8));				// 0xBF0
	buf[522] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_hi_pl_8));				// 0xBF4
	buf[523] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_adj_pl_8));				// 0xBF8
	buf[524] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_lo_pl_9));				// 0xBFC
	buf[525] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_med_pl_9));				// 0xC00
	buf[526] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_hi_pl_9));				// 0xC04
	buf[527] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_adj_pl_9));				// 0xC08
	buf[528] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_lo_pl_10));				// 0xC0C
	buf[529] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_med_pl_10));				// 0xC10
	buf[530] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_hi_pl_10));				// 0xC14
	buf[531] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_adj_pl_10));				// 0xC18
	buf[532] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_lo_pl_11));				// 0xC1C
	buf[533] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_med_pl_11));				// 0xC20
	buf[534] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_hi_pl_11));				// 0xC24
	buf[535] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_adj_pl_11));				// 0xC28
	buf[536] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_lo_pl_12));				// 0xC2C
	buf[537] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_med_pl_12));				// 0xC30
	buf[538] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_hi_pl_12));				// 0xC34
	buf[539] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_adj_pl_12));				// 0xC38
	buf[540] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_lo_pl_13));				// 0xC3C
	buf[541] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_med_pl_13));				// 0xC40
	buf[542] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_hi_pl_13));				// 0xC44
	buf[543] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_adj_pl_13));				// 0xC48
	buf[544] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_lo_pl_14));				// 0xC4C
	buf[545] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_med_pl_14));				// 0xC50
	buf[546] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_hi_pl_14));				// 0xC54
	buf[547] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_adj_pl_14));				// 0xC58
	buf[548] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_lo_pl_15));				// 0xC5C
	buf[549] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_med_pl_15));				// 0xC60
	buf[550] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_hi_pl_15));				// 0xC64
	buf[551] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ptp_tam_adj_pl_15));				// 0xC68
	buf[552] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ts_ss_lo));					// 0xC6C
	buf[553] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ts_ss_mid));					// 0xC70
	buf[554] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(0, rx_ts_ss_hi));					// 0xC74

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

	pauseparam->rx_pause = 0;
	pauseparam->tx_pause = 0;
	pauseparam->autoneg = 0;

//	if (priv->flow_ctrl & FLOW_RX)
		pauseparam->rx_pause = 1;
//	if (priv->flow_ctrl & FLOW_TX)
		pauseparam->tx_pause = 1;
}

static int ftile_set_pauseparam(struct net_device *dev,
				struct ethtool_pauseparam *pauseparam)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	int new_pause = FLOW_OFF;
	int ret = 0;
	//struct intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
        struct platform_device *pdev  = priv->pdev_hssi;
        u32 chan = priv->tile_chan;


	spin_lock(&priv->mac_cfg_lock);

	if (pauseparam->autoneg != 0) {
		ret = -EINVAL;
		goto out;
	}

	if (pauseparam->rx_pause) {
		new_pause |= FLOW_RX;
		hssi_set_bit(pdev, HSSI_ETH_RECONFIG, chan,
			    eth_mac_ptp_csroffs(0, rx_flow_control_feature_cfg),
			    ETH_RX_EN_STD_FLOW_CTRL,true);
	} else {
		hssi_clear_bit(pdev, HSSI_ETH_RECONFIG, chan,
			      eth_mac_ptp_csroffs(0, rx_flow_control_feature_cfg),
			      ETH_RX_EN_STD_FLOW_CTRL,true);
	}

	if (pauseparam->tx_pause) {
		new_pause |= FLOW_TX;
		hssi_set_bit(pdev, HSSI_ETH_RECONFIG, chan,
			    eth_mac_ptp_csroffs(0, tx_flow_control_feature_cfg),
			    ETH_TX_EN_STD_FLOW_CTRL,true);
	} else {
		hssi_clear_bit(pdev, HSSI_ETH_RECONFIG, chan,
			      eth_mac_ptp_csroffs(0, tx_flow_control_feature_cfg),
			      ETH_TX_EN_STD_FLOW_CTRL,true);
	}

	hssi_csrwr32_ba(pdev,HSSI_ETH_RECONFIG,chan,
		eth_mac_ptp_csroffs(0, pause_quanta_0),priv->pause);
	priv->flow_ctrl = new_pause;
out:
	spin_unlock(&priv->mac_cfg_lock);
	return ret;
}

static int ftile_get_ts_info(struct net_device *dev,
			     struct ethtool_ts_info *info)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	info->so_timestamping = SOF_TIMESTAMPING_TX_HARDWARE |
				SOF_TIMESTAMPING_RX_HARDWARE |
				SOF_TIMESTAMPING_RAW_HARDWARE;

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

/* Set link ksettings (phy address, speed) for ethtools */
static int ftile_set_link_ksettings(struct net_device *dev,
				    const struct ethtool_link_ksettings *cmd)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	//if (!priv || !priv->phylink)
	if(!priv)
		return -ENODEV;

	return phylink_ethtool_ksettings_set(priv->phylink, cmd);
}

/* Get link ksettings (phy address, speed) for ethtools */
static int ftile_get_link_ksettings(struct net_device *dev,
				    struct ethtool_link_ksettings *cmd)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	//if (!priv || !priv->phylink)
	if(!priv)
		return -ENODEV;

	return phylink_ethtool_ksettings_get(priv->phylink, cmd);
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
};


void intel_fpga_ftile_set_ethtool_ops(struct net_device *netdev)
{
	netdev->ethtool_ops = &ftile_ethtool_ops;
}


