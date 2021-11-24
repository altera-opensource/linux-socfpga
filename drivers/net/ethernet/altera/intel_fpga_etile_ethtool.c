// SPDX-License-Identifier: GPL-2.0
/* Ethtool support for Intel FPGA E-tile Ethernet MAC driver
 * Copyright (C) 2019-2021 Intel Corporation. All rights reserved
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
#include "intel_fpga_etile.h"
#include "altera_utils.h"

#define ETILE_STATS_LEN	ARRAY_SIZE(stat_gstrings)
#define ETILE_NUM_REGS	294

static char const stat_gstrings[][ETH_GSTRING_LEN] = {
	"tx_fragments",
	"tx_jabbers",
	"tx_fcs_errors",
	"tx_crc_errors",
	"tx_errored_multicast",
	"tx_errored_broadcast",
	"tx_errored_unicast",
	"tx_errored_mulitcast_ctrl_frames",
	"tx_errored_broadcast_ctrl_frames",
	"tx_errored_unicast_ctrl_frames",
	"tx_pause_errors",
	"tx_64byte_frames",
	"tx_65to127bytes_frames",
	"tx_128to255bytes_frames",
	"tx_256to511bytes_frames",
	"tx_512to1023bytes_frames",
	"tx_1024to1518bytes_frames",
	"tx_1519tomax_frames",
	"tx_oversize_frames",
	"tx_multicast_frames",
	"tx_broadcast_frames",
	"tx_unicast_frames",
	"tx_multicast_ctrl_frames",
	"tx_broadcast_ctrl_frames",
	"tx_unicast_ctrl_frames",
	"tx_pause_frames",
	"tx_runt_packets",
	"tx_frame_starts",
	"tx_length_errored_frames",
	"tx_prc_errored_frames",
	"tx_prc_frames",
	"tx_payload_bytes",
	"tx_bytes",
	"tx_errors",
	"tx_dropped",
	"tx_bad_length_type_frames",
	"rx_fragments",
	"rx_jabbers",
	"rx_fcs_errors",
	"rx_crc_errors",
	"rx_errored_multicast",
	"rx_errored_broadcast",
	"rx_errored_unicast",
	"rx_errored_mulitcast_ctrl_frames",
	"rx_errored_broadcast_ctrl_frames",
	"rx_errored_unicast_ctrl_frames",
	"rx_pause_errors",
	"rx_64byte_frames",
	"rx_65to127bytes_frames",
	"rx_128to255bytes_frames",
	"rx_256to511bytes_frames",
	"rx_512to1023bytes_frames",
	"rx_1024to1518bytes_frames",
	"rx_1519tomax_frames",
	"rx_oversize_frames",
	"rx_multicast_frames",
	"rx_broadcast_frames",
	"rx_unicast_frames",
	"rx_multicast_ctrl_frames",
	"rx_broadcast_ctrl_frames",
	"rx_unicast_ctrl_frames",
	"rx_pause_frames",
	"rx_runt_packets",
	"rx_frame_starts",
	"rx_length_errored_frames",
	"rx_prc_errored_frames",
	"rx_prc_frames",
	"rx_payload_bytes",
	"rx_bytes"
};

static void etile_get_drvinfo(struct net_device *dev,
			      struct ethtool_drvinfo *info)
{
	strscpy(info->driver, "intel_fpga_etile", ETH_GSTRING_LEN);
	strscpy(info->version, "v1.0", ETH_GSTRING_LEN);
	strscpy(info->bus_info, "platform", ETH_GSTRING_LEN);
}

/* Fill in a buffer with the strings which correspond to the
 * stats
 */
static void etile_gstrings(struct net_device *dev, u32 stringset, u8 *buf)
{
	memcpy(buf, stat_gstrings, ETILE_STATS_LEN * ETH_GSTRING_LEN);
}

static void etile_fill_stats(struct net_device *dev, struct ethtool_stats *dummy,
			     u64 *buf)
{
	struct intel_fpga_etile_eth_private *priv = netdev_priv(dev);
	u32 lsb;
	u32 msb;

	/* TX Fragments */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_fragments_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_fragments_msb));
	buf[0] = ((u64)msb << 32) | lsb;

	/* TX Jabbers */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_jabbers_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_jabbers_msb));
	buf[1] = ((u64)msb << 32) | lsb;

	/* TX FCS errors */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_fcserr_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_fcserr_msb));
	buf[2] = ((u64)msb << 32) | lsb;

	/* TX CRC errors */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_crcerr_okpkt_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_crcerr_okpkt_msb));
	buf[3] = ((u64)msb << 32) | lsb;

	/* TX errored multicast */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_mcast_data_err_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_mcast_data_err_msb));
	buf[4] = ((u64)msb << 32) | lsb;

	/* TX errored broadcast */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_bcast_data_err_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_bcast_data_err_msb));
	buf[5] = ((u64)msb << 32) | lsb;

	/* TX errored unicast */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_ucast_data_err_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_ucast_data_err_msb));
	buf[6] = ((u64)msb << 32) | lsb;

	/* TX errored multicast ctrl frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_mcast_ctrl_err_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_mcast_ctrl_err_msb));
	buf[7] = ((u64)msb << 32) | lsb;

	/* TX errored broadcast ctrl frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_bcast_ctrl_err_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_bcast_ctrl_err_msb));
	buf[8] = ((u64)msb << 32) | lsb;

	/* TX errored unicast ctrl frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_ucast_ctrl_err_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_ucast_ctrl_err_msb));
	buf[9] = ((u64)msb << 32) | lsb;

	/* TX pause errors */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_pause_err_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_pause_err_msb));
	buf[10] = ((u64)msb << 32) | lsb;

	/* TX 64-byte frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_64b_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_64b_msb));
	buf[11] = ((u64)msb << 32) | lsb;

	/* TX 65to127-byte frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_65to127b_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_65to127b_msb));
	buf[12] = ((u64)msb << 32) | lsb;

	/* TX 128to255-byte frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_128to255b_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_128to255b_msb));
	buf[13] = ((u64)msb << 32) | lsb;

	/* TX 256to511-byte frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_256to511b_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_256to511b_msb));
	buf[14] = ((u64)msb << 32) | lsb;

	/* TX 512to1023-byte frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_512to1023b_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_512to1023b_msb));
	buf[15] = ((u64)msb << 32) | lsb;

	/* TX 1024to1518-byte frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_1024to1518b_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_1024to1518b_msb));
	buf[16] = ((u64)msb << 32) | lsb;

	/* TX 1519toMAX-byte frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_1519tomaxb_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_1519tomaxb_msb));
	buf[17] = ((u64)msb << 32) | lsb;

	/* TX oversize frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_oversize_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_oversize_msb));
	buf[18] = ((u64)msb << 32) | lsb;

	/* TX multicast frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_mcast_data_ok_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_mcast_data_ok_msb));
	buf[19] = ((u64)msb << 32) | lsb;

	/* TX broadcast frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_bcast_data_ok_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_bcast_data_ok_msb));
	buf[20] = ((u64)msb << 32) | lsb;

	/* TX unicast frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_ucast_data_ok_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_ucast_data_ok_msb));
	buf[21] = ((u64)msb << 32) | lsb;

	/* TX multicast ctrl frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_mcast_ctrl_ok_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_mcast_ctrl_ok_msb));
	buf[22] = ((u64)msb << 32) | lsb;

	/* TX broadcast ctrl frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_bcast_ctrl_ok_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_bcast_ctrl_ok_msb));
	buf[23] = ((u64)msb << 32) | lsb;

	/* TX unicast ctrl frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_ucast_ctrl_ok_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_ucast_ctrl_ok_msb));
	buf[24] = ((u64)msb << 32) | lsb;

	/* TX pause frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_pause_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_pause_msb));
	buf[25] = ((u64)msb << 32) | lsb;

	/* TX runt packets */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_rnt_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_rnt_msb));
	buf[26] = ((u64)msb << 32) | lsb;

	/* TX frame starts */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_st_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_st_msb));
	buf[27] = ((u64)msb << 32) | lsb;

	/* TX length-errored frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_lenerr_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_lenerr_msb));
	buf[28] = ((u64)msb << 32) | lsb;

	/* TX PRC-errored frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_pfc_err_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_pfc_err_msb));
	buf[29] = ((u64)msb << 32) | lsb;

	/* TX PFC frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_pfc_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_pfc_msb));
	buf[30] = ((u64)msb << 32) | lsb;

	/* TX payload bytes in frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_payload_octetsok_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_payload_octetsok_msb));
	buf[31] = ((u64)msb << 32) | lsb;

	/* TX bytes in frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_frame_octetsok_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_frame_octetsok_msb));
	buf[32] = ((u64)msb << 32) | lsb;

	/* TX malformed frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_malformed_ctrl_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_malformed_ctrl_msb));
	buf[33] = ((u64)msb << 32) | lsb;

	/* TX dropped frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_dropped_ctrl_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_dropped_ctrl_msb));
	buf[34] = ((u64)msb << 32) | lsb;

	/* TX bad-length/type frames */
	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_badlt_ctrl_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_badlt_ctrl_msb));
	buf[35] = ((u64)msb << 32) | lsb;

	/* RX Fragments */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_fragments_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_fragments_msb));
	buf[36] = ((u64)msb << 32) | lsb;

	/* RX Jabbers */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_jabbers_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_jabbers_msb));
	buf[37] = ((u64)msb << 32) | lsb;

	/* RX FCS errors */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_fcserr_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_fcserr_msb));
	buf[38] = ((u64)msb << 32) | lsb;

	/* RX CRC errors */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_crcerr_okpkt_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_crcerr_okpkt_msb));
	buf[39] = ((u64)msb << 32) | lsb;

	/* RX errored multicast */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_mcast_data_err_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_mcast_data_err_msb));
	buf[40] = ((u64)msb << 32) | lsb;

	/* RX errored broadcast */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_bcast_data_err_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_bcast_data_err_msb));
	buf[41] = ((u64)msb << 32) | lsb;

	/* RX errored unicast */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_ucast_data_err_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_ucast_data_err_msb));
	buf[42] = ((u64)msb << 32) | lsb;

	/* RX errored multicast ctrl frames */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_mcast_ctrl_err_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_mcast_ctrl_err_msb));
	buf[43] = ((u64)msb << 32) | lsb;

	/* RX errored broadcast ctrl frames */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_bcast_ctrl_err_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_bcast_ctrl_err_msb));
	buf[44] = ((u64)msb << 32) | lsb;

	/* RX errored unicast ctrl frames */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_ucast_ctrl_err_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_ucast_ctrl_err_msb));
	buf[45] = ((u64)msb << 32) | lsb;

	/* RX pause errors */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_pause_err_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_pause_err_msb));
	buf[46] = ((u64)msb << 32) | lsb;

	/* RX 64-byte frames */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_64b_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_64b_msb));
	buf[47] = ((u64)msb << 32) | lsb;

	/* RX 65to127-byte frames */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_65to127b_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_65to127b_msb));
	buf[48] = ((u64)msb << 32) | lsb;

	/* RX 128to255-byte frames */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_128to255b_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_128to255b_msb));
	buf[49] = ((u64)msb << 32) | lsb;

	/* RX 256to511-byte frames */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_256to511b_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_256to511b_msb));
	buf[50] = ((u64)msb << 32) | lsb;

	/* RX 512to1023-byte frames */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_512to1023b_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_512to1023b_msb));
	buf[51] = ((u64)msb << 32) | lsb;

	/* RX 1024to1518-byte frames */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_1024to1518b_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_1024to1518b_msb));
	buf[52] = ((u64)msb << 32) | lsb;

	/* RX 1519toMAX-byte frames */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_1519tomaxb_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_1519tomaxb_msb));
	buf[53] = ((u64)msb << 32) | lsb;

	/* RX oversize frames */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_oversize_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_oversize_msb));
	buf[54] = ((u64)msb << 32) | lsb;

	/* RX multicast frames */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_mcast_data_ok_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_mcast_data_ok_msb));
	buf[55] = ((u64)msb << 32) | lsb;

	/* RX broadcast frames */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_bcast_data_ok_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_bcast_data_ok_msb));
	buf[56] = ((u64)msb << 32) | lsb;

	/* RX unicast frames */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_ucast_data_ok_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_ucast_data_ok_msb));
	buf[57] = ((u64)msb << 32) | lsb;

	/* RX multicast ctrl frames */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_mcast_ctrl_ok_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_mcast_ctrl_ok_msb));
	buf[58] = ((u64)msb << 32) | lsb;

	/* RX broadcast ctrl frames */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_bcast_ctrl_ok_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_bcast_ctrl_ok_msb));
	buf[59] = ((u64)msb << 32) | lsb;

	/* RX unicast ctrl frames */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_ucast_ctrl_ok_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_ucast_ctrl_ok_msb));
	buf[60] = ((u64)msb << 32) | lsb;

	/* RX pause frames */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_pause_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_pause_msb));
	buf[61] = ((u64)msb << 32) | lsb;

	/* RX runt frames */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_rnt_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_rnt_msb));
	buf[62] = ((u64)msb << 32) | lsb;

	/* RX frame starts */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_st_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_st_msb));
	buf[63] = ((u64)msb << 32) | lsb;

	/* RX length-errored frames */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_lenerr_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_lenerr_msb));
	buf[64] = ((u64)msb << 32) | lsb;

	/* RX PRC-errored frames */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_pfc_err_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_pfc_err_msb));
	buf[65] = ((u64)msb << 32) | lsb;

	/* RX PRC frames */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_pfc_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_pfc_msb));
	buf[66] = ((u64)msb << 32) | lsb;

	/* RX payload bytes in frames */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_payload_octetsok_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_payload_octetsok_msb));
	buf[67] = ((u64)msb << 32) | lsb;

	/* RX bytes in frames */
	lsb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_frame_octetsok_lsb));
	msb = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_frame_octetsok_msb));
	buf[68] = ((u64)msb << 32) | lsb;
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
	buf[0] = csrrd32(priv->mac_dev, eth_auto_neg_link_csroffs(anlt_sequencer_config));
	buf[1] = csrrd32(priv->mac_dev, eth_auto_neg_link_csroffs(anlt_sequencer_status));
	buf[2] = csrrd32(priv->mac_dev, eth_auto_neg_link_csroffs(auto_neg_conf_1));
	buf[3] = csrrd32(priv->mac_dev, eth_auto_neg_link_csroffs(auto_neg_conf_2));
	buf[4] = csrrd32(priv->mac_dev, eth_auto_neg_link_csroffs(auto_neg_stat));
	buf[5] = csrrd32(priv->mac_dev, eth_auto_neg_link_csroffs(auto_neg_conf_3));
	buf[6] = csrrd32(priv->mac_dev, eth_auto_neg_link_csroffs(auto_neg_conf_4));
	buf[7] = csrrd32(priv->mac_dev, eth_auto_neg_link_csroffs(auto_neg_conf_5));
	buf[8] = csrrd32(priv->mac_dev, eth_auto_neg_link_csroffs(auto_neg_conf_6));
	buf[9] = csrrd32(priv->mac_dev, eth_auto_neg_link_csroffs(auto_neg_stat_1));
	buf[10] = csrrd32(priv->mac_dev, eth_auto_neg_link_csroffs(auto_neg_stat_2));
	buf[11] = csrrd32(priv->mac_dev, eth_auto_neg_link_csroffs(auto_neg_stat_3));
	buf[12] = csrrd32(priv->mac_dev, eth_auto_neg_link_csroffs(auto_neg_stat_4));
	buf[13] = csrrd32(priv->mac_dev, eth_auto_neg_link_csroffs(auto_neg_stat_5));
	buf[14] = csrrd32(priv->mac_dev, eth_auto_neg_link_csroffs(auto_neg_an_channel_override));
	buf[15] = csrrd32(priv->mac_dev,
			  eth_auto_neg_link_csroffs(auto_neg_const_next_page_override));
	buf[16] = csrrd32(priv->mac_dev,
			  eth_auto_neg_link_csroffs(auto_neg_const_next_page_lp_stat));
	buf[17] = csrrd32(priv->mac_dev, eth_auto_neg_link_csroffs(link_train_conf_1));
	buf[18] = csrrd32(priv->mac_dev, eth_auto_neg_link_csroffs(link_train_stat_1));
	buf[19] = csrrd32(priv->mac_dev, eth_auto_neg_link_csroffs(link_train_conf_lane_0));
	buf[20] = csrrd32(priv->mac_dev, eth_auto_neg_link_csroffs(link_train_conf_lane_1));
	buf[21] = csrrd32(priv->mac_dev, eth_auto_neg_link_csroffs(link_train_conf_lane_2));
	buf[22] = csrrd32(priv->mac_dev, eth_auto_neg_link_csroffs(link_train_conf_lane_3));

	/* PHY registers */
	buf[23] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_rev_id));
	buf[24] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_scratch));
	buf[25] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_loopback));
	buf[26] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_config));
	buf[27] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_cdr_pll_locked));
	buf[28] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_tx_datapath_ready));
	buf[29] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_frm_err_detect));
	buf[30] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_clr_frm_err));
	buf[31] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_pcs_stat_anlt));
	buf[32] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_pcs_err_inject));
	buf[33] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_am_lock));
	buf[34] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_dskew_chng));
	buf[35] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_ber_cnt));
	buf[36] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_aib_transfer_ready_stat));
	buf[37] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_soft_rc_reset_stat));
	buf[38] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_pcs_virtual_ln_0));
	buf[39] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_pcs_virtual_ln_1));
	buf[40] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_pcs_virtual_ln_2));
	buf[41] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_pcs_virtual_ln_3));
	buf[42] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_recovered_clk_freq));
	buf[43] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_tx_clk_freq));
	buf[44] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_tx_pld_conf));
	buf[45] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_tx_pld_stat));
	buf[46] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_dynamic_deskew_buf_stat));
	buf[47] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_rx_pld_conf));
	buf[48] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_rx_pcs_conf));
	buf[49] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_bip_cnt_0));
	buf[50] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_bip_cnt_1));
	buf[51] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_bip_cnt_2));
	buf[52] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_bip_cnt_3));
	buf[53] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_bip_cnt_4));
	buf[54] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_bip_cnt_5));
	buf[55] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_bip_cnt_6));
	buf[56] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_bip_cnt_7));
	buf[57] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_bip_cnt_8));
	buf[58] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_bip_cnt_9));
	buf[59] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_bip_cnt_10));
	buf[60] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_bip_cnt_11));
	buf[61] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_bip_cnt_12));
	buf[62] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_bip_cnt_13));
	buf[63] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_bip_cnt_14));
	buf[64] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_bip_cnt_15));
	buf[65] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_bip_cnt_16));
	buf[66] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_bip_cnt_17));
	buf[67] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_bip_cnt_18));
	buf[68] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_bip_cnt_19));
	buf[69] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_timer_window_hiber_check));
	buf[70] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_hiber_frm_err));
	buf[71] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_err_block_cnt));
	buf[72] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_deskew_dept_0));
	buf[73] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_deskew_dept_1));
	buf[74] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_deskew_dept_2));
	buf[75] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_deskew_dept_3));
	buf[76] = csrrd32(priv->mac_dev, eth_phy_csroffs(phy_rx_pcs_test_err_cnt));

	/* TX MAC Registers */
	buf[77] = csrrd32(priv->mac_dev, eth_tx_mac_csroffs(tx_mac_rev_id));
	buf[78] = csrrd32(priv->mac_dev, eth_tx_mac_csroffs(tx_mac_scratch));
	buf[79] = csrrd32(priv->mac_dev, eth_tx_mac_csroffs(tx_mac_link_fault));
	buf[80] = csrrd32(priv->mac_dev, eth_tx_mac_csroffs(tx_mac_ipg_col_rem));
	buf[81] = csrrd32(priv->mac_dev, eth_tx_mac_csroffs(tx_mac_max_frm_size));
	buf[82] = csrrd32(priv->mac_dev, eth_tx_mac_csroffs(tx_mac_conf));
	buf[83] = csrrd32(priv->mac_dev, eth_tx_mac_csroffs(tx_mac_ehip_conf));
	buf[84] = csrrd32(priv->mac_dev, eth_tx_mac_csroffs(tx_mac_source_addr_lower_bytes));
	buf[85] = csrrd32(priv->mac_dev, eth_tx_mac_csroffs(tx_mac_source_addr_higher_bytes));

	/* RX MAC Registers */
	buf[86] = csrrd32(priv->mac_dev, eth_rx_mac_csroffs(rx_mac_rev_id));
	buf[87] = csrrd32(priv->mac_dev, eth_rx_mac_csroffs(rx_mac_scratch));
	buf[88] = csrrd32(priv->mac_dev, eth_rx_mac_csroffs(rx_mac_max_frm_size));
	buf[89] = csrrd32(priv->mac_dev, eth_rx_mac_csroffs(rx_mac_frwd_rx_crc));
	buf[90] = csrrd32(priv->mac_dev, eth_rx_mac_csroffs(rx_max_link_fault));
	buf[91] = csrrd32(priv->mac_dev, eth_rx_mac_csroffs(rx_mac_conf));
	buf[92] = csrrd32(priv->mac_dev, eth_rx_mac_csroffs(rx_mac_ehip_conf));

	/* Pause and Priority Registers */
	buf[93] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(txsfc_module_revision_id));
	buf[94] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(txsfc_scratch_register));
	buf[95] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(enable_tx_pause_ports));
	buf[96] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(tx_pause_request));
	buf[97] = csrrd32(priv->mac_dev,
			  eth_pause_and_priority_csroffs(enable_automatic_tx_pause_retransmission));
	buf[98] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(retransmit_holdoff_quanta));
	buf[99] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(retransmit_pause_quanta));
	buf[100] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(enable_tx_xoff));
	buf[101] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(enable_uniform_holdoff));
	buf[102] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(set_uniform_holdoff));
	buf[103] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(flow_control_fields_lsb));
	buf[104] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(flow_control_fields_msb));
	buf[105] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(flow_control_frames_lsb));
	buf[106] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(flow_control_frames_msb));
	buf[107] = csrrd32(priv->mac_dev,
			   eth_pause_and_priority_csroffs(tx_flow_control_feature_cfg));
	buf[108] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(pause_quanta_0));
	buf[109] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(pause_quanta_1));
	buf[110] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(pause_quanta_2));
	buf[111] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(pause_quanta_3));
	buf[112] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(pause_quanta_4));
	buf[113] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(pause_quanta_5));
	buf[114] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(pause_quanta_6));
	buf[115] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(pause_quanta_7));
	buf[116] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(pfc_holdoff_quanta_0));
	buf[117] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(pfc_holdoff_quanta_1));
	buf[118] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(pfc_holdoff_quanta_2));
	buf[119] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(pfc_holdoff_quanta_3));
	buf[120] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(pfc_holdoff_quanta_4));
	buf[121] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(pfc_holdoff_quanta_5));
	buf[122] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(pfc_holdoff_quanta_6));
	buf[123] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(pfc_holdoff_quanta_7));
	buf[124] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(rxsfc_module_revision_id));
	buf[125] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(rxsfc_scratch_register));
	buf[126] = csrrd32(priv->mac_dev,
			   eth_pause_and_priority_csroffs(enable_rx_pause_frame_processing_fields));
	buf[127] = csrrd32(priv->mac_dev,
			   eth_pause_and_priority_csroffs(forward_flow_control_frames));
	buf[128] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(rx_pause_frames_lsb));
	buf[129] = csrrd32(priv->mac_dev, eth_pause_and_priority_csroffs(rx_pause_frames_msb));
	buf[130] = csrrd32(priv->mac_dev,
			   eth_pause_and_priority_csroffs(rx_flow_control_feature_cfg));

	/* TX Statistics Counter Registers */
	buf[131] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_fragments_lsb));
	buf[132] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_fragments_msb));
	buf[133] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_jabbers_lsb));
	buf[134] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_jabbers_msb));
	buf[135] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_fcserr_lsb));
	buf[136] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_fcserr_msb));
	buf[137] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_crcerr_okpkt_lsb));
	buf[138] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_crcerr_okpkt_msb));
	buf[139] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_mcast_data_err_lsb));
	buf[140] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_mcast_data_err_msb));
	buf[141] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_bcast_data_err_lsb));
	buf[142] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_bcast_data_err_msb));
	buf[143] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_ucast_data_err_lsb));
	buf[144] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_ucast_data_err_msb));
	buf[145] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_mcast_ctrl_err_lsb));
	buf[146] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_mcast_ctrl_err_msb));
	buf[147] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_bcast_ctrl_err_lsb));
	buf[148] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_bcast_ctrl_err_msb));
	buf[149] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_ucast_ctrl_err_lsb));
	buf[150] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_ucast_ctrl_err_msb));
	buf[151] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_pause_err_lsb));
	buf[152] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_pause_err_msb));
	buf[153] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_64b_lsb));
	buf[154] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_64b_msb));
	buf[155] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_65to127b_lsb));
	buf[156] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_65to127b_msb));
	buf[157] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_128to255b_lsb));
	buf[158] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_128to255b_msb));
	buf[159] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_256to511b_lsb));
	buf[160] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_256to511b_msb));
	buf[161] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_512to1023b_lsb));
	buf[162] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_512to1023b_msb));
	buf[163] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_1024to1518b_lsb));
	buf[164] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_1024to1518b_msb));
	buf[165] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_1519tomaxb_lsb));
	buf[166] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_1519tomaxb_msb));
	buf[167] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_oversize_lsb));
	buf[168] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_oversize_msb));
	buf[169] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_mcast_data_ok_lsb));
	buf[170] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_mcast_data_ok_msb));
	buf[171] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_bcast_data_ok_lsb));
	buf[172] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_bcast_data_ok_msb));
	buf[173] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_ucast_data_ok_lsb));
	buf[174] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_ucast_data_ok_msb));
	buf[175] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_mcast_ctrl_ok_lsb));
	buf[176] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_mcast_ctrl_ok_msb));
	buf[177] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_bcast_ctrl_ok_lsb));
	buf[178] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_bcast_ctrl_ok_msb));
	buf[179] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_ucast_ctrl_ok_lsb));
	buf[180] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_ucast_ctrl_ok_msb));
	buf[181] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_pause_lsb));
	buf[182] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_pause_msb));
	buf[183] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_rnt_lsb));
	buf[184] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_rnt_msb));
	buf[185] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_st_lsb));
	buf[186] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_st_msb));
	buf[187] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_lenerr_lsb));
	buf[188] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_lenerr_msb));
	buf[189] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_pfc_err_lsb));
	buf[190] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_pfc_err_msb));
	buf[191] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_pfc_lsb));
	buf[192] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_pfc_msb));
	buf[193] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_stat_revid));
	buf[194] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_stat_scratch));
	buf[195] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_cntr_config));
	buf[196] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_cntr_status));
	buf[197] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_payload_octetsok_lsb));
	buf[198] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_payload_octetsok_msb));
	buf[199] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_frame_octetsok_lsb));
	buf[200] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_frame_octetsok_msb));
	buf[201] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_malformed_ctrl_lsb));
	buf[202] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_malformed_ctrl_msb));
	buf[203] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_dropped_ctrl_lsb));
	buf[204] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_dropped_ctrl_msb));
	buf[205] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_badlt_ctrl_lsb));
	buf[206] = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_badlt_ctrl_msb));

	/* RX Statistics Counter Registers */
	buf[207] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_fragments_lsb));
	buf[208] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_fragments_msb));
	buf[209] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_jabbers_lsb));
	buf[210] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_jabbers_msb));
	buf[211] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_fcserr_lsb));
	buf[212] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_fcserr_msb));
	buf[213] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_crcerr_okpkt_lsb));
	buf[214] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_crcerr_okpkt_msb));
	buf[215] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_mcast_data_err_lsb));
	buf[216] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_mcast_data_err_msb));
	buf[217] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_bcast_data_err_lsb));
	buf[218] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_bcast_data_err_msb));
	buf[219] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_ucast_data_err_lsb));
	buf[220] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_ucast_data_err_msb));
	buf[221] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_mcast_ctrl_err_lsb));
	buf[222] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_mcast_ctrl_err_msb));
	buf[223] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_bcast_ctrl_err_lsb));
	buf[224] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_bcast_ctrl_err_msb));
	buf[225] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_ucast_ctrl_err_lsb));
	buf[226] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_ucast_ctrl_err_msb));
	buf[227] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_pause_err_lsb));
	buf[228] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_pause_err_msb));
	buf[229] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_64b_lsb));
	buf[230] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_64b_msb));
	buf[231] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_65to127b_lsb));
	buf[232] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_65to127b_msb));
	buf[233] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_128to255b_lsb));
	buf[234] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_128to255b_msb));
	buf[235] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_256to511b_lsb));
	buf[236] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_256to511b_msb));
	buf[237] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_512to1023b_lsb));
	buf[238] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_512to1023b_msb));
	buf[239] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_1024to1518b_lsb));
	buf[240] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_1024to1518b_msb));
	buf[241] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_1519tomaxb_lsb));
	buf[242] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_1519tomaxb_msb));
	buf[243] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_oversize_lsb));
	buf[244] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_oversize_msb));
	buf[245] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_mcast_data_ok_lsb));
	buf[246] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_mcast_data_ok_msb));
	buf[247] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_bcast_data_ok_lsb));
	buf[248] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_bcast_data_ok_msb));
	buf[249] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_ucast_data_ok_lsb));
	buf[250] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_ucast_data_ok_msb));
	buf[251] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_mcast_ctrl_ok_lsb));
	buf[252] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_mcast_ctrl_ok_msb));
	buf[253] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_bcast_ctrl_ok_lsb));
	buf[254] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_bcast_ctrl_ok_msb));
	buf[255] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_ucast_ctrl_ok_lsb));
	buf[256] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_ucast_ctrl_ok_msb));
	buf[257] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_pause_lsb));
	buf[258] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_pause_msb));
	buf[259] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_rnt_lsb));
	buf[260] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_rnt_msb));
	buf[261] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_st_lsb));
	buf[262] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_st_msb));
	buf[263] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_lenerr_lsb));
	buf[264] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_lenerr_msb));
	buf[265] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_pfc_err_lsb));
	buf[266] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_pfc_err_msb));
	buf[267] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_pfc_lsb));
	buf[268] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_pfc_msb));
	buf[269] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_stat_revid));
	buf[270] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_stat_scratch));
	buf[271] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_cntr_config));
	buf[272] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_cntr_status));
	buf[273] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_payload_octetsok_lsb));
	buf[274] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_payload_octetsok_msb));
	buf[275] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_frame_octetsok_lsb));
	buf[276] = csrrd32(priv->mac_dev, eth_rx_stats_csroffs(rx_frame_octetsok_msb));

	/* PTP Registers */
	buf[277] = csrrd32(priv->mac_dev, eth_ptp_csroffs(txptp_revid));
	buf[278] = csrrd32(priv->mac_dev, eth_ptp_csroffs(txptp_scratch));
	buf[279] = csrrd32(priv->mac_dev, eth_ptp_csroffs(tx_ptp_clk_period));
	buf[280] = csrrd32(priv->mac_dev, eth_ptp_csroffs(tx_ptp_extra_latency));
	buf[281] = csrrd32(priv->mac_dev, eth_ptp_csroffs(ptp_debug));
	buf[282] = csrrd32(priv->mac_dev, eth_ptp_csroffs(rxptp_revid));
	buf[283] = csrrd32(priv->mac_dev, eth_ptp_csroffs(rxptp_scratch));
	buf[284] = csrrd32(priv->mac_dev, eth_ptp_csroffs(rx_ptp_extra_latency));
	buf[285] = csrrd32(priv->mac_dev, eth_ptp_csroffs(tx_ui_reg));
	buf[286] = csrrd32(priv->mac_dev, eth_ptp_csroffs(rx_ui_reg));
	buf[287] = csrrd32(priv->mac_dev, eth_ptp_csroffs(tam_snapshot));
	buf[288] = csrrd32(priv->mac_dev, eth_ptp_csroffs(tx_tam_l));
	buf[289] = csrrd32(priv->mac_dev, eth_ptp_csroffs(tx_tam_h));
	buf[290] = csrrd32(priv->mac_dev, eth_ptp_csroffs(tx_count));
	buf[291] = csrrd32(priv->mac_dev, eth_ptp_csroffs(rx_tam_l));
	buf[292] = csrrd32(priv->mac_dev, eth_ptp_csroffs(rx_tam_h));
	buf[293] = csrrd32(priv->mac_dev, eth_ptp_csroffs(rx_count));
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
	int new_pause = FLOW_OFF;
	int ret = 0;

	spin_lock(&priv->mac_cfg_lock);

	if (pauseparam->autoneg != 0) {
		ret = -EINVAL;
		goto out;
	}

	if (pauseparam->rx_pause) {
		new_pause |= FLOW_RX;
		tse_set_bit(priv->mac_dev,
			    eth_pause_and_priority_csroffs(rx_flow_control_feature_cfg),
			    ETH_RX_EN_STD_FLOW_CTRL);
	} else {
		tse_clear_bit(priv->mac_dev,
			      eth_pause_and_priority_csroffs(rx_flow_control_feature_cfg),
			      ETH_RX_EN_STD_FLOW_CTRL);
	}

	if (pauseparam->tx_pause) {
		new_pause |= FLOW_TX;
		tse_set_bit(priv->mac_dev,
			    eth_pause_and_priority_csroffs(tx_flow_control_feature_cfg),
			    ETH_TX_EN_STD_FLOW_CTRL);
	} else {
		tse_clear_bit(priv->mac_dev,
			      eth_pause_and_priority_csroffs(tx_flow_control_feature_cfg),
			      ETH_TX_EN_STD_FLOW_CTRL);
	}

	csrwr32(priv->pause, priv->mac_dev,
		eth_pause_and_priority_csroffs(pause_quanta_0));
	priv->flow_ctrl = new_pause;
out:
	spin_unlock(&priv->mac_cfg_lock);
	return ret;
}

static int etile_get_ts_info(struct net_device *dev,
			     struct ethtool_ts_info *info)
{
	struct intel_fpga_etile_eth_private *priv = netdev_priv(dev);

	info->so_timestamping = SOF_TIMESTAMPING_TX_HARDWARE |
				SOF_TIMESTAMPING_RX_HARDWARE |
				SOF_TIMESTAMPING_RAW_HARDWARE;

	if (priv->ptp_priv.ptp_clock)
		info->phc_index = ptp_clock_index(priv->ptp_priv.ptp_clock);
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

static const struct ethtool_ops etile_ethtool_ops = {
	.get_drvinfo = etile_get_drvinfo,
	.get_regs_len = etile_reglen,
	.get_regs = etile_get_regs,
	.get_link = ethtool_op_get_link,
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

void intel_fpga_etile_set_ethtool_ops(struct net_device *netdev)
{
	netdev->ethtool_ops = &etile_ethtool_ops;
}
