// SPDX-License-Identifier: GPL-2.0
/* Ethtool support for Intel FPGA E-tile Ethernet MAC driver
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
#include "intel_fpga_eth_etile.h"
#include "intel_fpga_eth_hssi_itf.h"

#define ETILE_STATS_LEN	ARRAY_SIZE(stat_gstrings)
#define ETILE_NUM_REGS	294

#define ARRAY_LEN(arr) ((int)((int)sizeof(arr) / (int)sizeof(arr[0])))

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

static void etile_get_drvinfo(struct net_device *dev,
			      struct ethtool_drvinfo *info)
{
	strscpy(info->driver, "intel_fpga_hssi_etile", ETH_GSTRING_LEN);
	strscpy(info->version, "v1.0", ETH_GSTRING_LEN);
	strscpy(info->bus_info, "platform", ETH_GSTRING_LEN);
}

/* Fill in a buffer with the strings which correspond to the
 * stats
 */
static void etile_gstrings(struct net_device *dev, u32 stringset, u8 *buf)
{
	switch (stringset) {

	case ETH_SS_STATS:
		memcpy(buf, stat_gstrings, ETILE_STATS_LEN * ETH_GSTRING_LEN);
		return;
	default:
		return;
	}
}

static void etile_fill_stats(struct net_device *dev, struct ethtool_stats *dummy,
			     u64 *buf)
{
	struct intel_fpga_etile_eth_private *priv = netdev_priv(dev);

	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->chan;

	/* Tx packets */
	buf[0] = hssi_read_mac_stats64(pdev, chan, MACSTAT_TX_PACKETS);

	/* Rx packets */
	buf[1] = hssi_read_mac_stats64(pdev, chan, MACSTAT_RX_PACKETS);

	/* Rx CRC error packets */
	buf[2] = hssi_read_mac_stats64(pdev, chan, MACSTAT_RX_CRC_ERRORS);

	/* Rx align error packets */
	buf[3] = hssi_read_mac_stats64(pdev, chan, MACSTAT_RX_ALIGN_ERRORS);

	/* Tx bytes */
	buf[4] = hssi_read_mac_stats64(pdev, chan, MACSTAT_TX_BYTES);

	/* Rx bytes */
	buf[5] = hssi_read_mac_stats64(pdev, chan, MACSTAT_RX_BYTES);

	/* Tx pause bytes */
	buf[6] = hssi_read_mac_stats64(pdev, chan, MACSTAT_TX_PAUSE);

	/* Rx pause bytes */
	buf[7] = hssi_read_mac_stats64(pdev, chan, MACSTAT_RX_PAUSE);

	/* Rx error bytes */
	buf[8] = hssi_read_mac_stats64(pdev, chan, MACSTAT_RX_ERRORS);

	/* Tx error bytes */
	buf[9] = hssi_read_mac_stats64(pdev, chan, MACSTAT_TX_ERRORS);

	/* Rx unicast bytes */
	buf[10] = hssi_read_mac_stats64(pdev, chan, MACSTAT_RX_UNICAST);

	/* Rx multicast bytes */
	buf[11] = hssi_read_mac_stats64(pdev, chan, MACSTAT_RX_MULTICAST);

	/* Rx broadcast bytes */
	buf[12] = hssi_read_mac_stats64(pdev, chan, MACSTAT_RX_BROADCAST);

	/* Tx discards bytes */
	buf[13] = hssi_read_mac_stats64(pdev, chan, MACSTAT_TX_DISCARDS);

	/* Tx unicast bytes */
	buf[14] = hssi_read_mac_stats64(pdev, chan, MACSTAT_TX_UNICAST);

	/* Tx multicast bytes */
	buf[15] = hssi_read_mac_stats64(pdev, chan, MACSTAT_TX_MULTICAST);

	/* Tx broadcast bytes */
	buf[16] = hssi_read_mac_stats64(pdev, chan, MACSTAT_TX_BROADCAST);

	/* Rx Ethernet drops */
	buf[17] = hssi_read_mac_stats64(pdev, chan, MACSTAT_ETHER_DROPS);

	/* Rx total bytes*/
	buf[18] = hssi_read_mac_stats64(pdev, chan, MACSTAT_RX_TOTAL_BYTES);

	/* Rx total packets*/
	buf[19] = hssi_read_mac_stats64(pdev, chan, MACSTAT_RX_TOTAL_PACKETS);

	/* Rx undersize*/
	buf[20] = hssi_read_mac_stats64(pdev, chan, MACSTAT_RX_UNDERSIZE);

	/* Rx oversize*/
	buf[21] = hssi_read_mac_stats64(pdev, chan, MACSTAT_RX_OVERSIZE);

	/* Rx 64 bytes*/
	buf[22] = hssi_read_mac_stats64(pdev, chan, MACSTAT_RX_64_BYTES);

	/* Rx 65-127 bytes*/
	buf[23] = hssi_read_mac_stats64(pdev, chan, MACSTAT_RX_65_127_BYTES);

	/* Rx 128-255 bytes*/
	buf[24] = hssi_read_mac_stats64(pdev, chan, MACSTAT_RX_128_255_BYTES);

	/* Rx 256-511 bytes*/
	buf[25] = hssi_read_mac_stats64(pdev, chan, MACSTAT_RX_256_511_BYTES);

	/* Rx 512-1023 bytes*/
	buf[26] = hssi_read_mac_stats64(pdev, chan, MACSTAT_RX_512_1023_BYTES);

	/* Rx 1024-1518 bytes*/
	buf[27] = hssi_read_mac_stats64(pdev, chan, MACSTAT_RX_1024_1518_BYTES);

	/* Rx > 10519 bytes*/
	buf[28] = hssi_read_mac_stats64(pdev, chan, MACSTAT_RX_GTE_1519_BYTES);

	/* Rx jabber bytes*/
	buf[29] = hssi_read_mac_stats64(pdev, chan, MACSTAT_RX_JABBERS);

	/* Rx fragments*/
	buf[30] = hssi_read_mac_stats64(pdev, chan, MACSTAT_RX_RUNTS);
}

static int etile_sset_count(struct net_device *dev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return ETILE_STATS_LEN;
	default:
		return -EOPNOTSUPP;
	}
}

static u32 etile_get_msglevel(struct net_device *dev)
{
	struct intel_fpga_etile_eth_private *priv = netdev_priv(dev);

	return priv->msg_enable;
}


static void etile_set_msglevel(struct net_device *dev, uint32_t data)
{
	struct intel_fpga_etile_eth_private *priv = netdev_priv(dev);

	priv->msg_enable = data;
}

static int etile_reglen(struct net_device *dev)
{
	return ETILE_NUM_REGS * sizeof(u32);
}

static void etile_get_regs(struct net_device *dev, struct ethtool_regs *regs,
			   void *regbuf)
{
	struct intel_fpga_etile_eth_private *priv = netdev_priv(dev);

	struct platform_device *pdev  = priv->pdev_hssi;
	u32 chan = priv->chan;

	u32 *buf = regbuf;

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
	/* Auto Negotiation and Link Training Registers */
	buf[0] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_auto_neg_link_csroffs(anlt_sequencer_config));
	buf[1] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_auto_neg_link_csroffs(anlt_sequencer_status));
	buf[2] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_auto_neg_link_csroffs(auto_neg_conf_1));
	buf[3] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_auto_neg_link_csroffs(auto_neg_conf_2));
	buf[4] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_auto_neg_link_csroffs(auto_neg_stat));
	buf[5] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_auto_neg_link_csroffs(auto_neg_conf_3));
	buf[6] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_auto_neg_link_csroffs(auto_neg_conf_4));
	buf[7] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_auto_neg_link_csroffs(auto_neg_conf_5));
	buf[8] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_auto_neg_link_csroffs(auto_neg_conf_6));
	buf[9] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_auto_neg_link_csroffs(auto_neg_stat_1));
	buf[10] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_auto_neg_link_csroffs(auto_neg_stat_2));
	buf[11] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_auto_neg_link_csroffs(auto_neg_stat_3));
	buf[12] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_auto_neg_link_csroffs(auto_neg_stat_4));
	buf[13] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_auto_neg_link_csroffs(auto_neg_stat_5));
	buf[14] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_auto_neg_link_csroffs(auto_neg_an_channel_override));
	buf[15] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan,
			  eth_auto_neg_link_csroffs(auto_neg_const_next_page_override));
	buf[16] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan,
			  eth_auto_neg_link_csroffs(auto_neg_const_next_page_lp_stat));
	buf[17] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_auto_neg_link_csroffs(link_train_conf_1));
	buf[18] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_auto_neg_link_csroffs(link_train_stat_1));
	buf[19] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_auto_neg_link_csroffs(link_train_conf_lane_0));
	buf[20] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_auto_neg_link_csroffs(link_train_conf_lane_1));
	buf[21] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_auto_neg_link_csroffs(link_train_conf_lane_2));
	buf[22] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_auto_neg_link_csroffs(link_train_conf_lane_3));

	/* PHY registers */
	buf[23] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_rev_id));
	buf[24] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_scratch));
	buf[25] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_loopback));
	buf[26] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_config));
	buf[27] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_cdr_pll_locked));
	buf[28] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_tx_datapath_ready));
	buf[29] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_frm_err_detect));
	buf[30] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_clr_frm_err));
	buf[31] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_pcs_stat_anlt));
	buf[32] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_pcs_err_inject));
	buf[33] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_am_lock));
	buf[34] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_dskew_chng));
	buf[35] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_ber_cnt));
	buf[36] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_aib_transfer_ready_stat));
	buf[37] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_soft_rc_reset_stat));
	buf[38] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_pcs_virtual_ln_0));
	buf[39] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_pcs_virtual_ln_1));
	buf[40] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_pcs_virtual_ln_2));
	buf[41] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_pcs_virtual_ln_3));
	buf[42] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_recovered_clk_freq));
	buf[43] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_tx_clk_freq));
	buf[44] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_tx_pld_conf));
	buf[45] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_tx_pld_stat));
	buf[46] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_dynamic_deskew_buf_stat));
	buf[47] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_rx_pld_conf));
	buf[48] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_rx_pcs_conf));
	buf[49] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_bip_cnt_0));
	buf[50] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_bip_cnt_1));
	buf[51] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_bip_cnt_2));
	buf[52] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_bip_cnt_3));
	buf[53] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_bip_cnt_4));
	buf[54] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_bip_cnt_5));
	buf[55] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_bip_cnt_6));
	buf[56] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_bip_cnt_7));
	buf[57] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_bip_cnt_8));
	buf[58] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_bip_cnt_9));
	buf[59] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_bip_cnt_10));
	buf[60] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_bip_cnt_11));
	buf[61] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_bip_cnt_12));
	buf[62] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_bip_cnt_13));
	buf[63] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_bip_cnt_14));
	buf[64] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_bip_cnt_15));
	buf[65] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_bip_cnt_16));
	buf[66] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_bip_cnt_17));
	buf[67] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_bip_cnt_18));
	buf[68] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_bip_cnt_19));
	buf[69] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_timer_window_hiber_check));
	buf[70] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_hiber_frm_err));
	buf[71] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_err_block_cnt));
	buf[72] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_deskew_dept_0));
	buf[73] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_deskew_dept_1));
	buf[74] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_deskew_dept_2));
	buf[75] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_deskew_dept_3));
	buf[76] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_phy_csroffs(phy_rx_pcs_test_err_cnt));

	/* TX MAC Registers */
	buf[77] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_mac_csroffs(tx_mac_rev_id));
	buf[78] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_mac_csroffs(tx_mac_scratch));
	buf[79] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_mac_csroffs(tx_mac_link_fault));
	buf[80] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_mac_csroffs(tx_mac_ipg_col_rem));
	buf[81] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_mac_csroffs(tx_mac_max_frm_size));
	buf[82] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_mac_csroffs(tx_mac_conf));
	buf[83] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_mac_csroffs(tx_mac_ehip_conf));
	buf[84] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_mac_csroffs(tx_mac_source_addr_lower_bytes));
	buf[85] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_mac_csroffs(tx_mac_source_addr_higher_bytes));

	/* RX MAC Registers */
	buf[86] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_mac_csroffs(rx_mac_rev_id));
	buf[87] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_mac_csroffs(rx_mac_scratch));
	buf[88] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_mac_csroffs(rx_mac_max_frm_size));
	buf[89] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_mac_csroffs(rx_mac_frwd_rx_crc));
	buf[90] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_mac_csroffs(rx_max_link_fault));
	buf[91] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_mac_csroffs(rx_mac_conf));
	buf[92] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_mac_csroffs(rx_mac_ehip_conf));

	/* Pause and Priority Registers */
	buf[93] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(txsfc_module_revision_id));
	buf[94] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(txsfc_scratch_register));
	buf[95] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(enable_tx_pause_ports));
	buf[96] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(tx_pause_request));
	buf[97] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan,
			  eth_pause_and_priority_csroffs(enable_automatic_tx_pause_retransmission));
	buf[98] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(retransmit_holdoff_quanta));
	buf[99] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(retransmit_pause_quanta));
	buf[100] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(enable_tx_xoff));
	buf[101] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(enable_uniform_holdoff));
	buf[102] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(set_uniform_holdoff));
	buf[103] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(flow_control_fields_lsb));
	buf[104] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(flow_control_fields_msb));
	buf[105] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(flow_control_frames_lsb));
	buf[106] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(flow_control_frames_msb));
	buf[107] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan,
			   eth_pause_and_priority_csroffs(tx_flow_control_feature_cfg));
	buf[108] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(pause_quanta_0));
	buf[109] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(pause_quanta_1));
	buf[110] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(pause_quanta_2));
	buf[111] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(pause_quanta_3));
	buf[112] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(pause_quanta_4));
	buf[113] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(pause_quanta_5));
	buf[114] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(pause_quanta_6));
	buf[115] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(pause_quanta_7));
	buf[116] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(pfc_holdoff_quanta_0));
	buf[117] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(pfc_holdoff_quanta_1));
	buf[118] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(pfc_holdoff_quanta_2));
	buf[119] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(pfc_holdoff_quanta_3));
	buf[120] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(pfc_holdoff_quanta_4));
	buf[121] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(pfc_holdoff_quanta_5));
	buf[122] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(pfc_holdoff_quanta_6));
	buf[123] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(pfc_holdoff_quanta_7));
	buf[124] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(rxsfc_module_revision_id));
	buf[125] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(rxsfc_scratch_register));
	buf[126] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan,
			   eth_pause_and_priority_csroffs(enable_rx_pause_frame_processing_fields));
	buf[127] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan,
			   eth_pause_and_priority_csroffs(forward_flow_control_frames));
	buf[128] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(rx_pause_frames_lsb));
	buf[129] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_pause_and_priority_csroffs(rx_pause_frames_msb));
	buf[130] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan,
			   eth_pause_and_priority_csroffs(rx_flow_control_feature_cfg));

	/* TX Statistics Counter Registers */
	buf[131] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_fragments_lsb));
	buf[132] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_fragments_msb));
	buf[133] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_jabbers_lsb));
	buf[134] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_jabbers_msb));
	buf[135] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_fcserr_lsb));
	buf[136] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_fcserr_msb));
	buf[137] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_crcerr_okpkt_lsb));
	buf[138] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_crcerr_okpkt_msb));
	buf[139] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_mcast_data_err_lsb));
	buf[140] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_mcast_data_err_msb));
	buf[141] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_bcast_data_err_lsb));
	buf[142] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_bcast_data_err_msb));
	buf[143] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_ucast_data_err_lsb));
	buf[144] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_ucast_data_err_msb));
	buf[145] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_mcast_ctrl_err_lsb));
	buf[146] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_mcast_ctrl_err_msb));
	buf[147] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_bcast_ctrl_err_lsb));
	buf[148] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_bcast_ctrl_err_msb));
	buf[149] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_ucast_ctrl_err_lsb));
	buf[150] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_ucast_ctrl_err_msb));
	buf[151] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_pause_err_lsb));
	buf[152] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_pause_err_msb));
	buf[153] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_64b_lsb));
	buf[154] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_64b_msb));
	buf[155] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_65to127b_lsb));
	buf[156] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_65to127b_msb));
	buf[157] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_128to255b_lsb));
	buf[158] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_128to255b_msb));
	buf[159] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_256to511b_lsb));
	buf[160] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_256to511b_msb));
	buf[161] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_512to1023b_lsb));
	buf[162] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_512to1023b_msb));
	buf[163] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_1024to1518b_lsb));
	buf[164] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_1024to1518b_msb));
	buf[165] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_1519tomaxb_lsb));
	buf[166] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_1519tomaxb_msb));
	buf[167] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_oversize_lsb));
	buf[168] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_oversize_msb));
	buf[169] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_mcast_data_ok_lsb));
	buf[170] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_mcast_data_ok_msb));
	buf[171] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_bcast_data_ok_lsb));
	buf[172] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_bcast_data_ok_msb));
	buf[173] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_ucast_data_ok_lsb));
	buf[174] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_ucast_data_ok_msb));
	buf[175] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_mcast_ctrl_ok_lsb));
	buf[176] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_mcast_ctrl_ok_msb));
	buf[177] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_bcast_ctrl_ok_lsb));
	buf[178] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_bcast_ctrl_ok_msb));
	buf[179] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_ucast_ctrl_ok_lsb));
	buf[180] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_ucast_ctrl_ok_msb));
	buf[181] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_pause_lsb));
	buf[182] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_pause_msb));
	buf[183] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_rnt_lsb));
	buf[184] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_rnt_msb));
	buf[185] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_st_lsb));
	buf[186] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_st_msb));
	buf[187] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_lenerr_lsb));
	buf[188] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_lenerr_msb));
	buf[189] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_pfc_err_lsb));
	buf[190] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_pfc_err_msb));
	buf[191] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_pfc_lsb));
	buf[192] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_pfc_msb));
	buf[193] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_stat_revid));
	buf[194] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_stat_scratch));
	buf[195] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_cntr_config));
	buf[196] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_cntr_status));
	buf[197] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_payload_octetsok_lsb));
	buf[198] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_payload_octetsok_msb));
	buf[199] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_frame_octetsok_lsb));
	buf[200] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_frame_octetsok_msb));
	buf[201] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_malformed_ctrl_lsb));
	buf[202] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_malformed_ctrl_msb));
	buf[203] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_dropped_ctrl_lsb));
	buf[204] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_dropped_ctrl_msb));
	buf[205] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_badlt_ctrl_lsb));
	buf[206] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_tx_stats_csroffs(tx_badlt_ctrl_msb));

	/* RX Statistics Counter Registers */
	buf[207] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_fragments_lsb));
	buf[208] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_fragments_msb));
	buf[209] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_jabbers_lsb));
	buf[210] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_jabbers_msb));
	buf[211] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_fcserr_lsb));
	buf[212] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_fcserr_msb));
	buf[213] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_crcerr_okpkt_lsb));
	buf[214] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_crcerr_okpkt_msb));
	buf[215] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_mcast_data_err_lsb));
	buf[216] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_mcast_data_err_msb));
	buf[217] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_bcast_data_err_lsb));
	buf[218] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_bcast_data_err_msb));
	buf[219] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_ucast_data_err_lsb));
	buf[220] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_ucast_data_err_msb));
	buf[221] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_mcast_ctrl_err_lsb));
	buf[222] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_mcast_ctrl_err_msb));
	buf[223] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_bcast_ctrl_err_lsb));
	buf[224] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_bcast_ctrl_err_msb));
	buf[225] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_ucast_ctrl_err_lsb));
	buf[226] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_ucast_ctrl_err_msb));
	buf[227] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_pause_err_lsb));
	buf[228] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_pause_err_msb));
	buf[229] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_64b_lsb));
	buf[230] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_64b_msb));
	buf[231] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_65to127b_lsb));
	buf[232] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_65to127b_msb));
	buf[233] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_128to255b_lsb));
	buf[234] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_128to255b_msb));
	buf[235] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_256to511b_lsb));
	buf[236] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_256to511b_msb));
	buf[237] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_512to1023b_lsb));
	buf[238] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_512to1023b_msb));
	buf[239] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_1024to1518b_lsb));
	buf[240] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_1024to1518b_msb));
	buf[241] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_1519tomaxb_lsb));
	buf[242] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_1519tomaxb_msb));
	buf[243] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_oversize_lsb));
	buf[244] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_oversize_msb));
	buf[245] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_mcast_data_ok_lsb));
	buf[246] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_mcast_data_ok_msb));
	buf[247] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_bcast_data_ok_lsb));
	buf[248] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_bcast_data_ok_msb));
	buf[249] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_ucast_data_ok_lsb));
	buf[250] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_ucast_data_ok_msb));
	buf[251] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_mcast_ctrl_ok_lsb));
	buf[252] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_mcast_ctrl_ok_msb));
	buf[253] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_bcast_ctrl_ok_lsb));
	buf[254] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_bcast_ctrl_ok_msb));
	buf[255] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_ucast_ctrl_ok_lsb));
	buf[256] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_ucast_ctrl_ok_msb));
	buf[257] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_pause_lsb));
	buf[258] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_pause_msb));
	buf[259] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_rnt_lsb));
	buf[260] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_rnt_msb));
	buf[261] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_st_lsb));
	buf[262] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_st_msb));
	buf[263] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_lenerr_lsb));
	buf[264] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_lenerr_msb));
	buf[265] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_pfc_err_lsb));
	buf[266] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_pfc_err_msb));
	buf[267] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_pfc_lsb));
	buf[268] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_pfc_msb));
	buf[269] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_stat_revid));
	buf[270] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_stat_scratch));
	buf[271] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_cntr_config));
	buf[272] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_cntr_status));
	buf[273] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_payload_octetsok_lsb));
	buf[274] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_payload_octetsok_msb));
	buf[275] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_frame_octetsok_lsb));
	buf[276] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_rx_stats_csroffs(rx_frame_octetsok_msb));

	/* PTP Registers */
	buf[277] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(txptp_revid));
	buf[278] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(txptp_scratch));
	buf[279] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(tx_ptp_clk_period));
	buf[280] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(tx_ptp_extra_latency));
	buf[281] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(ptp_debug));
	buf[282] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(rxptp_revid));
	buf[283] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(rxptp_scratch));
	buf[284] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(rx_ptp_extra_latency));
	buf[285] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(tx_ui_reg));
	buf[286] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(rx_ui_reg));
	buf[287] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(tam_snapshot));
	buf[288] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(tx_tam_l));
	buf[289] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(tx_tam_h));
	buf[290] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(tx_count));
	buf[291] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(rx_tam_l));
	buf[292] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(rx_tam_h));
	buf[293] = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(rx_count));
}

static void etile_get_pauseparam(struct net_device *dev,
				 struct ethtool_pauseparam *pauseparam)
{
	struct intel_fpga_etile_eth_private *priv = netdev_priv(dev);

	pauseparam->rx_pause = 0;
	pauseparam->tx_pause = 0;
	pauseparam->autoneg = 0;

	if (priv->flow_ctrl & FLOW_RX)
		pauseparam->rx_pause = 1;
	if (priv->flow_ctrl & FLOW_TX)
		pauseparam->tx_pause = 1;
}

static int etile_set_pauseparam(struct net_device *dev,
				struct ethtool_pauseparam *pauseparam)
{
	struct intel_fpga_etile_eth_private *priv = netdev_priv(dev);
	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->chan;

	int new_pause = FLOW_OFF;
	int ret = 0;

	spin_lock(&priv->mac_cfg_lock);

	if (pauseparam->autoneg != 0) {
		ret = -EINVAL;
		goto out;
	}

	if (pauseparam->rx_pause) {
		new_pause |= FLOW_RX;
		hssi_set_bit(pdev, HSSI_ETH_RECONFIG, chan,
			    eth_pause_and_priority_csroffs(rx_flow_control_feature_cfg),
			    ETH_RX_EN_STD_FLOW_CTRL);
	} else {
		hssi_clear_bit(pdev, HSSI_ETH_RECONFIG, chan,
			      eth_pause_and_priority_csroffs(rx_flow_control_feature_cfg),
			      ETH_RX_EN_STD_FLOW_CTRL);
	}

	if (pauseparam->tx_pause) {
		new_pause |= FLOW_TX;
		hssi_set_bit(pdev, HSSI_ETH_RECONFIG, chan,
			     eth_pause_and_priority_csroffs(tx_flow_control_feature_cfg),
			     ETH_TX_EN_STD_FLOW_CTRL);
	} else {
		hssi_clear_bit(pdev, HSSI_ETH_RECONFIG, chan,
			      eth_pause_and_priority_csroffs(tx_flow_control_feature_cfg),
			      ETH_TX_EN_STD_FLOW_CTRL);
	}

	hssi_csrwr32(pdev, HSSI_ETH_RECONFIG, chan,
		eth_pause_and_priority_csroffs(pause_quanta_0), false, priv->pause);
	priv->flow_ctrl = new_pause;
out:
	spin_unlock(&priv->mac_cfg_lock);
	return ret;
}

static int etile_get_ts_info(struct net_device *dev,
			     struct ethtool_ts_info *info)
{

#if 0
	struct intel_fpga_etile_eth_private *priv = netdev_priv(dev);

	if (priv->ptp_priv.ptp_clock)
		info->phc_index = ptp_clock_index(priv->ptp_priv.ptp_clock);
	else
#endif
		info->phc_index = -1;
	info->so_timestamping = SOF_TIMESTAMPING_TX_HARDWARE |
				SOF_TIMESTAMPING_RX_HARDWARE |
				SOF_TIMESTAMPING_RAW_HARDWARE;

	info->tx_types = (1 << HWTSTAMP_TX_OFF) |
			 (1 << HWTSTAMP_TX_ON) |
			 (1 << HWTSTAMP_TX_ONESTEP_SYNC);

	info->rx_filters = (1 << HWTSTAMP_FILTER_NONE) |
			   (1 << HWTSTAMP_FILTER_ALL);

	return 0;
}

/* Set link ksettings (phy address, speed) for ethtools */
static int etile_set_link_ksettings(struct net_device *dev,
				    const struct ethtool_link_ksettings *cmd)
{
	struct intel_fpga_etile_eth_private *priv = netdev_priv(dev);

	if (!priv)
		return -ENODEV;

	return phylink_ethtool_ksettings_set(priv->phylink, cmd);
}

/* Get link ksettings (phy address, speed) for ethtools */
static int etile_get_link_ksettings(struct net_device *dev,
				    struct ethtool_link_ksettings *cmd)
{
	struct intel_fpga_etile_eth_private *priv = netdev_priv(dev);

	if (!priv)
		return -ENODEV;

	return phylink_ethtool_ksettings_get(priv->phylink, cmd);
}

static const struct ethtool_ops xtile_ethtool_ops = {
	.get_drvinfo = etile_get_drvinfo,
	.get_regs_len = etile_reglen,
	.get_regs = etile_get_regs,
	.get_link = ethtool_op_get_link,
	.get_strings = etile_gstrings,
	.get_sset_count = etile_sset_count,
	.get_ethtool_stats = etile_fill_stats,
	.get_msglevel = etile_get_msglevel,
	.set_msglevel = etile_set_msglevel,
	.get_pauseparam = etile_get_pauseparam,
	.set_pauseparam = etile_set_pauseparam,
	.get_ts_info = etile_get_ts_info,
	.get_link_ksettings = etile_get_link_ksettings,
	.set_link_ksettings = etile_set_link_ksettings,
};

void intel_fpga_xtile_set_ethtool_ops(struct net_device *netdev)
{
	netdev->ethtool_ops = &xtile_ethtool_ops;
}


