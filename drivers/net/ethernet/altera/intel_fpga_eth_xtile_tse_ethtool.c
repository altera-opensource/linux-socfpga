// SPDX-License-Identifier: GPL-2.0
/* Ethtool ops for the Altera TSE variant of the Intel FPGA xtile driver.
 *
 * This file is a near-direct port of drivers/net/ethernet/altera/altera_tse_ethtool.c
 * adapted to:
 *   - access MAC registers through priv->intel_fpga_tile_private->mac_dev
 *     instead of priv->mac_dev;
 *   - delegate link_ksettings to phylink instead of legacy phylib;
 *   - register itself via the tile.reg_ethtool_ops xtile_spec_ops hook.
 *
 * Counter set (31 entries) is preserved unchanged so existing ethtool consumers
 * (collectd, prometheus exporters, custom scripts) continue to work.
 *
 * Copyright (C) 2026 Altera Corporation.
 *
 * Contributors:
 *   Mahesh Vaidya
 *   Krishna Kumar S R
 *   Preetam Narayan
 *   Lubana.Badakar
 */

#include <linux/ethtool.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/net_tstamp.h>
#include <linux/phy.h>
#include <linux/phylink.h>

#include "altera_eth_dma.h"
#include "altera_tse.h"
#include "altera_utils.h"
#include "intel_fpga_eth_main.h"
#include "intel_fpga_eth_xtile_tse.h"

#define TSE_STATS_LEN	31
#define TSE_NUM_REGS	128

/* Match altera_tse_ethtool.c verbatim so user-space tooling that parses these
 * names continues to work.
 */
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

static inline struct intel_fpga_tse_private *
tse_eth_tilepriv(struct net_device *dev)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	return (struct intel_fpga_tse_private *)priv->intel_fpga_tile_private;
}

static void tse_eth_get_drvinfo(struct net_device *dev,
				struct ethtool_drvinfo *info)
{
	struct intel_fpga_tse_private *tp = tse_eth_tilepriv(dev);
	u32 rev = ioread32(&tp->mac_dev->megacore_revision);

	strscpy(info->driver, "intel_fpga_tse", sizeof(info->driver));
	snprintf(info->fw_version, ETHTOOL_FWVERS_LEN, "v%d.%d",
		 rev & 0xffff, (rev >> 16) & 0xffff);
	strscpy(info->bus_info, "platform", sizeof(info->bus_info));
}

static void tse_eth_gstrings(struct net_device *dev, u32 stringset, u8 *buf)
{
	if (stringset != ETH_SS_STATS)
		return;
	memcpy(buf, stat_gstrings, TSE_STATS_LEN * ETH_GSTRING_LEN);
}

/* Fill the 31-counter ethtool stats buffer from MAC CSRs.
 *
 * The MAC's IEEE 802.3 / RFC 2819 (RMON) counters are 32-bit, except for
 * octets_transmitted_ok, octets_received_ok and ether_stats_octets which
 * have 32-bit MSB extensions to form 64-bit counters.
 *
 * Reading order matches what's been in production with altera_tse_ethtool.c
 * for years; keep it stable.
 */
static void tse_eth_fill_stats(struct net_device *dev,
			       struct ethtool_stats *dummy, u64 *buf)
{
	struct intel_fpga_tse_private *tp = tse_eth_tilepriv(dev);
	int i = 0;
	u64 ext;

	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(frames_transmitted_ok));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(frames_received_ok));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(frames_check_sequence_errors));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(alignment_errors));

	/* Extended aOctetsTransmittedOK counter (64-bit) */
	ext  = (u64)csrrd32(tp->mac_dev, tse_csroffs(msb_octets_transmitted_ok)) << 32;
	ext |= csrrd32(tp->mac_dev, tse_csroffs(octets_transmitted_ok));
	buf[i++] = ext;

	/* Extended aOctetsReceivedOK counter (64-bit) */
	ext  = (u64)csrrd32(tp->mac_dev, tse_csroffs(msb_octets_received_ok)) << 32;
	ext |= csrrd32(tp->mac_dev, tse_csroffs(octets_received_ok));
	buf[i++] = ext;

	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(tx_pause_mac_ctrl_frames));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(rx_pause_mac_ctrl_frames));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(if_in_errors));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(if_out_errors));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(if_in_ucast_pkts));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(if_in_multicast_pkts));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(if_in_broadcast_pkts));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(if_out_discards));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(if_out_ucast_pkts));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(if_out_multicast_pkts));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(if_out_broadcast_pkts));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(ether_stats_drop_events));

	/* Extended etherStatsOctets counter (64-bit) */
	ext  = (u64)csrrd32(tp->mac_dev, tse_csroffs(msb_ether_stats_octets)) << 32;
	ext |= csrrd32(tp->mac_dev, tse_csroffs(ether_stats_octets));
	buf[i++] = ext;

	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(ether_stats_pkts));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(ether_stats_undersize_pkts));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(ether_stats_oversize_pkts));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(ether_stats_pkts_64_octets));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(ether_stats_pkts_65to127_octets));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(ether_stats_pkts_128to255_octets));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(ether_stats_pkts_256to511_octets));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(ether_stats_pkts_512to1023_octets));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(ether_stats_pkts_1024to1518_octets));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(ether_stats_pkts_1519tox_octets));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(ether_stats_jabbers));
	buf[i++] = csrrd32(tp->mac_dev, tse_csroffs(ether_stats_fragments));

	WARN_ON(i != TSE_STATS_LEN);
}

static int tse_eth_sset_count(struct net_device *dev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return TSE_STATS_LEN;
	default:
		return -EOPNOTSUPP;
	}
}

static u32 tse_eth_get_msglevel(struct net_device *dev)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	return priv->msg_enable;
}

static void tse_eth_set_msglevel(struct net_device *dev, u32 data)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	priv->msg_enable = data;
}

static int tse_eth_reglen(struct net_device *dev)
{
	return TSE_NUM_REGS * sizeof(u32);
}

/* Dump the first 128 dwords of the MAC CSR space (covers all documented
 * MAC registers, command_config / mac_addr_* / FIFO / stats / hash table).
 * Version=1 keeps wire compatibility with the original altera_tse driver.
 */
static void tse_eth_get_regs(struct net_device *dev, struct ethtool_regs *regs,
			     void *regbuf)
{
	struct intel_fpga_tse_private *tp = tse_eth_tilepriv(dev);
	u32 *buf = regbuf;
	int i;

	regs->version = 1;
	for (i = 0; i < TSE_NUM_REGS; i++)
		buf[i] = csrrd32(tp->mac_dev, i * 4);
}

/* link_ksettings: phylink-based — the variant owns its phylink instance, so
 * delegate. (The unpluggable driver uses phy_ethtool_*; we do not because the
 * authoritative state lives in phylink.)
 */
static int tse_eth_get_link_ksettings(struct net_device *dev,
				      struct ethtool_link_ksettings *cmd)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	if (!priv->phylink)
		return -EOPNOTSUPP;
	return phylink_ethtool_ksettings_get(priv->phylink, cmd);
}

static int tse_eth_set_link_ksettings(struct net_device *dev,
				      const struct ethtool_link_ksettings *cmd)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	if (!priv->phylink)
		return -EOPNOTSUPP;
	return phylink_ethtool_ksettings_set(priv->phylink, cmd);
}

static int tse_eth_nway_reset(struct net_device *dev)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	if (!priv->phylink)
		return -EOPNOTSUPP;
	return phylink_ethtool_nway_reset(priv->phylink);
}

static void tse_eth_get_pauseparam(struct net_device *dev,
				   struct ethtool_pauseparam *pause)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	if (priv->phylink)
		phylink_ethtool_get_pauseparam(priv->phylink, pause);
}

static int tse_eth_set_pauseparam(struct net_device *dev,
				  struct ethtool_pauseparam *pause)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	if (!priv->phylink)
		return -EOPNOTSUPP;
	return phylink_ethtool_set_pauseparam(priv->phylink, pause);
}

static const struct ethtool_ops intel_fpga_tse_ethtool_ops = {
	.get_drvinfo		= tse_eth_get_drvinfo,
	.get_regs_len		= tse_eth_reglen,
	.get_regs		= tse_eth_get_regs,
	.get_link		= ethtool_op_get_link,
	.get_strings		= tse_eth_gstrings,
	.get_sset_count		= tse_eth_sset_count,
	.get_ethtool_stats	= tse_eth_fill_stats,
	.get_msglevel		= tse_eth_get_msglevel,
	.set_msglevel		= tse_eth_set_msglevel,
	.get_link_ksettings	= tse_eth_get_link_ksettings,
	.set_link_ksettings	= tse_eth_set_link_ksettings,
	.nway_reset		= tse_eth_nway_reset,
	.get_pauseparam		= tse_eth_get_pauseparam,
	.set_pauseparam		= tse_eth_set_pauseparam,
};

void intel_fpga_tse_set_ethtool_ops(struct net_device *netdev)
{
	netdev->ethtool_ops = &intel_fpga_tse_ethtool_ops;
}
