// SPDX-License-Identifier: GPL-2.0
/* Ethtool support for Altera FPGA GTS Ethernet MAC driver
 * Copyright (C) 2024, 2025 Altera Corporation. All rights reserved
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
#include <linux/sfp.h>
#include <linux/phy/sfp-mem.h>
#include <linux/phylink.h>
#include "altera_eth_dma.h"
#include "intel_fpga_eth_main.h"
#include "intel_fpga_eth_gts.h"
#include "intel_fpga_gts_driver.h"
#include "intel_fpga_eth_hssi_itf.h"

#define GTS_STATS_LEN	ARRAY_SIZE(stat_gstrings)
#define GTS_NUM_REGS	555

static const char stat_gstrings[][ETH_GSTRING_LEN] = {
	"tx_packets",
	"tx_total_sop",
	"tx_total_packets",
	"tx_unicast",
	"tx_multicast",
	"tx_broadcast",
	"tx_bytes",
	"tx_total_bytes",
	"tx_64_bytes",
	"tx_65_127_bytes",
	"tx_128_255_bytes",
	"tx_256_511_bytes",
	"tx_512_1023_bytes",
	"tx_1024_1518_bytes",
	"tx_gte_1519_bytes",
	"tx_jabbers",
	"tx_runts",
	"tx_pause",
	"tx_undersize",
	"tx_oversize",
	"tx_crc_errors",
	"tx_ether_drops",
	"tx_align_errors",
	"tx_errors",
	"rx_packets",
	"rx_total_sop",
	"rx_total_packets",
	"rx_unicast",
	"rx_multicast",
	"rx_broadcast",
	"rx_bytes",
	"rx_total_bytes",
	"rx_64_bytes",
	"rx_65_127_bytes",
	"rx_128_255_bytes",
	"rx_256_511_bytes",
	"rx_512_1023_bytes",
	"rx_1024_1518_bytes",
	"rx_gte_1519_bytes",
	"rx_jabbers",
	"rx_runts",
	"rx_pause",
	"rx_undersize",
	"rx_oversize",
	"rx_crc_errors",
	"rx_ether_drops",
	"rx_align_errors",
	"rx_errors",
};

static void gts_get_drvinfo(struct net_device *dev,
			    struct ethtool_drvinfo *info)
{
	strscpy(info->driver, "intel_fpga_gts", ETH_GSTRING_LEN);
	strscpy(info->version, "v1.0", ETH_GSTRING_LEN);
	strscpy(info->bus_info, "platform", ETH_GSTRING_LEN);
}

/* Fill in a buffer with the strings which correspond to the
 * stats
 */
static void gts_gstrings(struct net_device *dev, u32 stringset, u8 *buf)
{
	memcpy(buf, stat_gstrings, GTS_STATS_LEN * ETH_GSTRING_LEN);
}

static int gts_get_eeprom_len(struct net_device *dev)
{
	return A0_EEPROM_SIZE;
}

static int gts_get_module_info (struct net_device *dev,
				struct ethtool_modinfo *info)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	if (!priv)
		return -ENODEV;

	if (!priv->phylink || !priv->dev || !priv->dev->sfp_bus) {
		return -ENODEV;
	}

	return sfp_get_module_info(priv->dev->sfp_bus, info);
}

static int gts_get_module_eeprom(struct net_device *dev,
				 struct ethtool_eeprom *eeprom, u8 *data)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	if (!priv)
		return -ENODEV;

	if (!priv->phylink || !priv->dev || !priv->dev->sfp_bus) {
		return -ENODEV;
	}

	return sfp_get_module_eeprom(priv->dev->sfp_bus, eeprom, data);
}

static int gts_get_module_eeprom_by_page(struct net_device *dev,
                                         const struct ethtool_module_eeprom *page,
                                         struct netlink_ext_ack *extack)
{
        intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	if (!priv)
                return -ENODEV;

        if (!priv->phylink || !priv->dev || !priv->dev->sfp_bus) {
                return -ENODEV;
        }
    	return sfp_get_module_eeprom_by_page(priv->dev->sfp_bus, page, extack);
}

static void gts_fill_stats(struct net_device *dev,
			   struct ethtool_stats *dummy,
			   u64 *buf)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	struct platform_device *pdev  = priv->pdev_hssi;
	u32 hssi_port = priv->hssi_port;
	u8 count = 0;

	hssi_lock_mac_stats(pdev, hssi_port);

	/* Tx packets */
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_PACKETS);

        /* Tx SOP count*/
        buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_SOP_COUNT);

	/* Tx total packets*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_TOTAL_PACKETS);

	/* Tx unicast bytes */
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_UNICAST);

	/* Tx multicast bytes */
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_MULTICAST);

	/* Tx broadcast bytes */
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_BROADCAST);

	/* Tx bytes */
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_BYTES);

	/* Tx total bytes*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_TOTAL_BYTES);

	/* Tx 64 bytes*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_64_BYTES);

	/* Tx 65-127 bytes*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_65_127_BYTES);

	/* Tx 128-255 bytes*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_128_255_BYTES);

	/* Tx 256-511 bytes*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_256_511_BYTES);

	/* Tx 512-1023 bytes*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_512_1023_BYTES);

	/* Tx 1024-1518 bytes*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_1024_1518_BYTES);

	/* Tx > 10519 bytes*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_GTE_1519_BYTES);

	/* Tx jabber bytes*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_JABBERS);

	/* Tx fragments*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_RUNTS);

	/* Tx pause bytes */
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_PAUSE);
	
	/* Tx undersize*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_UNDERSIZE);

	/* Tx oversize*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_OVERSIZE);

	/* Tx CRC error packets */
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_CRC_ERRORS);
	
	/* Tx Ethernet drops */
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_ETHER_DROPS);

	/* Tx align error packets */
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_ALIGN_ERRORS);
	
	/* Tx error bytes */
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_ERRORS);

	/* Rx packets */
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_PACKETS);

        /* Rx SOP count*/
        buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_SOP_COUNT);

	/* Rx total packets*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_TOTAL_PACKETS);

	/* Rx unicast bytes */
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_UNICAST);

	/* Rx multicast bytes */
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_MULTICAST);

	/* Rx broadcast bytes */
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_BROADCAST);

	/* Rx bytes */
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_BYTES);

	/* Rx total bytes*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_TOTAL_BYTES);

	/* Rx 64 bytes*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_64_BYTES);

	/* Rx 65-127 bytes*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_65_127_BYTES);

	/* Rx 128-255 bytes*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_128_255_BYTES);

	/* Rx 256-511 bytes*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_256_511_BYTES);

	/* Rx 512-1023 bytes*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_512_1023_BYTES);

	/* Rx 1024-1518 bytes*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_1024_1518_BYTES);

	/* Rx > 10519 bytes*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_GTE_1519_BYTES);

	/* Rx jabber bytes*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_JABBERS);

	/* Rx fragments*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_RUNTS);

	/* Rx pause bytes */
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_PAUSE);

	/* Rx undersize*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_UNDERSIZE);

	/* Rx oversize*/
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_OVERSIZE);

	/* Rx CRC error packets */
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_CRC_ERRORS);

	/* Rx Ethernet drops */
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_ETHER_DROPS);

	/* Rx align error packets */
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_ALIGN_ERRORS);

	/* Rx error bytes */
	buf[count++] = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_ERRORS);

	hssi_unlock_mac_stats(pdev, hssi_port);
}

static int gts_sset_count(struct net_device *dev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return GTS_STATS_LEN;
	default:
		return -EOPNOTSUPP;
	}
}

static u32 gts_get_msglevel(struct net_device *dev)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	return priv->msg_enable;
}

static void gts_set_msglevel(struct net_device *dev, uint32_t data)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	priv->msg_enable = data;
}

static int gts_reglen(struct net_device *dev)
{
	return GTS_NUM_REGS * sizeof(u32);
}

#define FILLER_BYTES(in) buf[buf_index++] = 0;

#define FILLER_HARDIP_EMAC(in) FILLER_BYTES(hardip_xcvr_pma->(in))

static void gts_get_regs(struct net_device *dev, struct ethtool_regs *regs,
			 void *regbuf)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	struct intel_fpga_gts_hardip_xcvr_pma *hardip_xcvr_pma;
	struct platform_device *pdev  = priv->pdev_hssi;
	u32 hssi_port = priv->hssi_port;
	u32 *buf = regbuf;
	u32 buf_index = 0;

	hardip_xcvr_pma = NULL;

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

	hssi_lock_mac_stats(pdev, hssi_port);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(link_fault_config));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(ipg_col_rem));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(max_tx_size_config));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(txmac_control));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(txmac_ehip_cfg));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(txmac_saddrl));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(txmac_saddrh));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(max_rx_size_config));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(mac_crc_config));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(rxmac_control));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(rxmac_ehip_cfg));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(tx_pause_en));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(tx_pause_request));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(retransmit_xoff_holdoff_en));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(retransmit_xoff_holdoff_quanta));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(tx_pause_quanta));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(tx_xof_en_tx_pause_qnumber));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cfg_retransmit_holdoff_en));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cfg_retransmit_holdoff_quanta));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(tx_pfc_daddrl));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(tx_pfc_daddrh));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(tx_pfc_saddrl));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(tx_pfc_saddrh));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(txsfc_ehip_cfg));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(rx_pause_enable));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(rx_pause_fwd));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(rx_pause_daddrl));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(rx_pause_daddrh));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(rxsfc_ehip_cfg));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_config));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_config));

	FILLER_HARDIP_EMAC(res1);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(pfc_pause_quanta_0));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(pfc_pause_quanta_1));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(pfc_pause_quanta_2));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(pfc_pause_quanta_3));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(pfc_pause_quanta_4));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(pfc_pause_quanta_5));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(pfc_pause_quanta_6));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(pfc_pause_quanta_7));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(pfc_holdoff_quanta_0));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(pfc_holdoff_quanta_1));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(pfc_holdoff_quanta_2));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(pfc_holdoff_quanta_3));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(pfc_holdoff_quanta_4));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(pfc_holdoff_quanta_5));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(pfc_holdoff_quanta_6));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(pfc_pause_quanta_7));

	FILLER_HARDIP_EMAC(res2);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(tx_ptp_extra_latency));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(tx_ptp_ui));

	FILLER_HARDIP_EMAC(res3);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(tx_ptp_phy_lane_num));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(tx_ptp_ap_filter));

	FILLER_HARDIP_EMAC(res4);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(rx_ptp_extra_latency));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(rx_ptp_ui));

	FILLER_HARDIP_EMAC(res5);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(rx_ptp_phy_lane_num));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(rx_ptp_ap_filter));

	FILLER_HARDIP_EMAC(res6);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(rx_pkt_n_ts_rx_ctr));

	FILLER_HARDIP_EMAC(res7);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_fragments_lo));

	FILLER_HARDIP_EMAC(res8);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_jabbers_lo));

	FILLER_HARDIP_EMAC(res9);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_fcs_lo));

	FILLER_HARDIP_EMAC(res10);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_fcs_err_okpkt_lo));

	FILLER_HARDIP_EMAC(res11);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_mcast_data_err_lo));

	FILLER_HARDIP_EMAC(res12);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_bcast_data_err_lo));

	FILLER_HARDIP_EMAC(res13);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_ucast_data_err_lo));

	FILLER_HARDIP_EMAC(res14);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_mcast_ctrl_err_lo));

	FILLER_HARDIP_EMAC(res15);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_bcast_ctrl_err_lo));

	FILLER_HARDIP_EMAC(res16);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_ucast_ctrl_err_lo));

	FILLER_HARDIP_EMAC(res17);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_pause_err_lo));

	FILLER_HARDIP_EMAC(res18);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_64b_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_64b_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_65to127b_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_65to127b_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_128to255b_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_128to255b_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_256to511b_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_256to511b_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_512to1023b_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_512to1023b_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_1024to1518b_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_1024to1518b_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_1519tomaxb_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_1519tomaxb_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_oversize_lo));

	FILLER_HARDIP_EMAC(res19);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_mcast_data_ok_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_mcast_data_ok_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_bcast_data_ok_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_bcast_data_ok_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_ucast_data_ok_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_ucast_data_ok_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_mcast_ctrl_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_mcast_ctrl_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_bcast_ctrl_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_bcast_ctrl_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_ucast_ctrl_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_ucast_ctrl_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_pause_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_pause_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_runt_lo));

	FILLER_HARDIP_EMAC(res20);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_st_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_st_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_lenerr_lo));
	FILLER_HARDIP_EMAC(res21);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_pfc_err_lo));
	FILLER_HARDIP_EMAC(res22);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_pfc_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_pfc_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_payloadoctetsok_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_payloadoctetsok_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_octetsok_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_octetsok_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_malformed_lo));

	FILLER_HARDIP_EMAC(res23);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_dropped_lo));
	FILLER_HARDIP_EMAC(res24);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_badlt_lo));

	FILLER_HARDIP_EMAC(res25);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_total_ptp_pkts));

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_total_1step_ptp_pkts));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_total_2step_ptp_pkts));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_total_v1_ptp_pkts));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_tx_total_v2_ptp_pkts));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_fragments_lo));

	FILLER_HARDIP_EMAC(res26);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_jabbers_lo));
	FILLER_HARDIP_EMAC(res27);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_fcs_lo));
	FILLER_HARDIP_EMAC(res28);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_fcs_err_okpkt_lo));
	FILLER_HARDIP_EMAC(res29);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_mcast_data_err_lo));
	FILLER_HARDIP_EMAC(res30);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_bcast_data_err_lo));
	FILLER_HARDIP_EMAC(res31);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_ucast_data_err_lo));
	FILLER_HARDIP_EMAC(res32);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_mcast_ctrl_err_lo));
	FILLER_HARDIP_EMAC(res33);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_bcast_ctrl_err_lo));
	FILLER_HARDIP_EMAC(res34);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_ucast_ctrl_err_lo));
	FILLER_HARDIP_EMAC(res35);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_pause_err_lo));
	FILLER_HARDIP_EMAC(res36);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_64b_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_64b_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_65to127b_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_65to127b_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_128to255b_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_128to255b_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_256to511b_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_256to511b_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_512to1023b_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_512to1023b_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_1024to1518b_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_1024to1518b_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_1519tomaxb_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_1519tomaxb_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_oversize_lo));

	FILLER_HARDIP_EMAC(res37);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_mcast_data_ok_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_mcast_data_ok_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_bcast_data_ok_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_bcast_data_ok_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_ucast_data_ok_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_ucast_data_ok_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_mcast_ctrl_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_mcast_ctrl_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_bcast_ctrl_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_bcast_ctrl_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_ucast_ctrl_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_ucast_ctrl_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_pause_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_pause_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_runt_lo));

	FILLER_HARDIP_EMAC(res38);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_st_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_st_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_lenerr_lo));

	FILLER_HARDIP_EMAC(res39);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_pfc_err_lo));
	FILLER_HARDIP_EMAC(res40);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_pfc_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_pfc_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_payloadoctetsok_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_payloadoctetsok_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_octetsok_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_octetsok_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_malformed_lo));

	FILLER_HARDIP_EMAC(res41);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_dropped_lo));
	FILLER_HARDIP_EMAC(res42);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_badlt_lo));
	FILLER_HARDIP_EMAC(res43);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(cntr_rx_total_ptp_ts));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(tx_ptp_cf_overflow));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(tx_ptp_tam_lo_pl_0));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(tx_ptp_tam_med_pl_0));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(tx_ptp_tam_hi_pl_0));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(tx_ptp_tam_adj_pl_0));

	FILLER_HARDIP_EMAC(res44);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(tx_ts_ss_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(tx_ts_ss_mid));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(tx_ts_ss_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(tx_vl_ss));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(rx_ptp_tam_lo_pl_0));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(rx_ptp_tam_med_pl_0));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(rx_ptp_tam_hi_pl_0));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(rx_ptp_tam_adj_pl_0));

	FILLER_HARDIP_EMAC(res45);

	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(rx_ts_ss_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(rx_ts_ss_mid));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, hssi_port,
					eth_hardip_emac_csroffs(rx_ts_ss_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_XCVR_PMA_HARDIP, hssi_port,
					eth_hardip_xcvr_pma_csroffs(sm_xcvrif_debug1));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_XCVR_PMA_HARDIP, hssi_port,
					eth_hardip_xcvr_pma_csroffs(sm_xcvrif_reg_9));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_XCVR_PMA_HARDIP, hssi_port,
					eth_hardip_xcvr_pma_csroffs(xcvrif_stat_0));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_XCVR_PMA_HARDIP, hssi_port,
					eth_hardip_xcvr_pma_csroffs(xcvrif_stat_hold_1));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_XCVR_PMA_HARDIP, hssi_port,
					eth_hardip_xcvr_pma_csroffs(xcvrif_stat_3));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_XCVR_PMA_HARDIP, hssi_port,
					eth_hardip_xcvr_pma_csroffs(xcvrif_stat_hold_4));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(SRDS_IP_SYNTH_MED_reg_16));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(SRDS_IP_SYNTH_MED_reg_17));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(SRDS_IP_SYNTH_SLOW_reg_16));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(SRDS_IP_SYNTH_SLOW_reg_17));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(SRDS_IP_SYNTH_FAST_reg_37));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(SRDS_IP_SYNTH_FAST_reg_38));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(SRDS_IP_LANE_reg_7));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(SRDS_IP_LANE_reg_9));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(SRDS_IP_LANE_reg_11));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(SRDS_IP_LANE_reg_110));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(SRDS_IP_LANE_reg_213));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(SRDS_IP_LANE_RXEQ_reg_5));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(SRDS_IP_LANE_RXEQ_reg_174));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(SRDS_IP_PLLLCSLOW_DIV0));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(SRDS_IP_PLLLCSLOW_FRAC_LOCK0));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(SRDS_IP_PLLLCMED_DIV0));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(SRDS_IP_PLLLCMED_FRAC_LOCK0));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(SRDS_IP_PLLLCFAST_DIV0));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(SRDS_IP_PLLLCFAST_FRAC_LOCK0));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(SRDS_IP_IF_debug));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(SRDS_IP_IF_TX1));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(SCMNG_PM_LINK_MNG_SIDE_CPI_REGS));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(SCMNG_PM_PHY_SIDE_CPI_REGS));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(GTS_Physical_LANE_Number));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, hssi_port,
					eth_hardip_pma_csroffs(pre_pma_lblk));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_BASE_SOFTIP, hssi_port,
					eth_soft_csroffs(gui_option));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_BASE_SOFTIP, hssi_port,
					eth_soft_csroffs(qhip_scratch));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_BASE_SOFTIP, hssi_port,
					eth_soft_csroffs(eth_reset));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_BASE_SOFTIP, hssi_port,
					eth_soft_csroffs(eth_reset_status));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_BASE_SOFTIP, hssi_port,
					eth_soft_csroffs(phy_tx_pll_locked));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_BASE_SOFTIP, hssi_port,
					eth_soft_csroffs(phy_eiofreq_locked));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_BASE_SOFTIP, hssi_port,
					eth_soft_csroffs(pcs_status));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_BASE_SOFTIP, hssi_port,
					eth_soft_csroffs(pcs_control));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_BASE_SOFTIP, hssi_port,
					eth_soft_csroffs(link_fault_status));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_BASE_SOFTIP, hssi_port,
					eth_soft_csroffs(clk_tx_khz));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_BASE_SOFTIP, hssi_port,
					eth_soft_csroffs(clk_rx_khz));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_BASE_SOFTIP, hssi_port,
					eth_soft_csroffs(clk_pll_khz));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_BASE_SOFTIP, hssi_port,
					eth_soft_csroffs(clk_tx_div_khz));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_BASE_SOFTIP, hssi_port,
					eth_soft_csroffs(clk_rec_div64_khz));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_BASE_SOFTIP, hssi_port,
					eth_soft_csroffs(clk_rec_div_khz));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_BASE_SOFTIP, hssi_port,
					eth_soft_csroffs(status_signals));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, hssi_port,
					eth_softip_ptp_csroffs(ptp_tx_tam_adjust));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, hssi_port,
					eth_softip_ptp_csroffs(ptp_rx_tam_adjust));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, hssi_port,
					eth_softip_ptp_csroffs(ptp_ref_lane));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, hssi_port,
					eth_softip_ptp_csroffs(ptp_dr_cfg));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, hssi_port,
					eth_softip_ptp_csroffs(ptp_tx_user_cfg_status));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, hssi_port,
					eth_softip_ptp_csroffs(ptp_rx_user_cfg_status));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, hssi_port,
					eth_softip_ptp_csroffs(ptp_uim_tam_snapshot));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, hssi_port,
					eth_softip_ptp_csroffs(ptp_tx_uim_tam_info0));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, hssi_port,
					eth_softip_ptp_csroffs(ptp_tx_uim_tam_info1));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, hssi_port,
					eth_softip_ptp_csroffs(ptp_rx_uim_tam_info0));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, hssi_port,
					eth_softip_ptp_csroffs(ptp_rx_uim_tam_info1));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, hssi_port,
					eth_softip_ptp_csroffs(ptp_status));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, hssi_port,
					eth_softip_ptp_csroffs(ptp_status2));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, hssi_port,
					eth_softip_ptp_csroffs(ptp_tx_lane_calc_data_constdelay));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, hssi_port,
					eth_softip_ptp_csroffs(ptp_rx_lane_calc_data_constdelay));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, hssi_port,
					eth_softip_ptp_csroffs(ptp_tx_lane0_calc_data_offset));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, hssi_port,
					eth_softip_ptp_csroffs(ptp_rx_lane0_calc_data_offset));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, hssi_port,
					eth_softip_ptp_csroffs(ptp_tx_lane0_calc_data_time));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, hssi_port,
					eth_softip_ptp_csroffs(ptp_rx_lane0_calc_data_time));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, hssi_port,
					eth_softip_ptp_csroffs(ptp_tx_lane0_calc_data_wiredelay));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, hssi_port,
					eth_softip_ptp_csroffs(ptp_rx_lane0_calc_data_wiredelay));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(config_ctrl));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(tx_pld_conf));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(phy_ehip_pcs_modes));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(xus_timer_window));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(ber_invalid_count));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(err_inj));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(phy_frame_error));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(phy_rxpcs_status));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(am_lock));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(ber_count));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(err_block_cnt));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_tx_top));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_lane_cfg0));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_err_inj_tx));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_lane_tx_stat));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_lane_tx_hold));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_lane_rx_stat));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_lane_rx_hold));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_cw_pos_rx));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_err_val_tx));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_corr_cw_cnt_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_corr_cw_cnt_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_uncorr_cw_cnt_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_uncorr_cw_cnt_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_corr_syms_cnt_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_corr_syms_cnt_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_corr_0s_cnt_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_corr_0s_cnt_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_corr_1s_cnt_lo));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_corr_1s_cnt_hi));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_corr_cwbin_cnt_0_1));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_corr_cwbin_cnt_2_3));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_corr_cwbin_cnt_4_5));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_corr_cwbin_cnt_6_7));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_corr_cwbin_cnt_8_9));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_corr_cwbin_cnt_10_11));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, hssi_port,
					eth_hardip_pcsfec_csroffs(rsfec_debug_cfg));
#if 0
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(ptp_clk_mux));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p0));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p1));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p2));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p3));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p4));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p5));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p6));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p7));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p8));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p9));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p10));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p11));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p12));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p13));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p14));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p15));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p16));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p17));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p18));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p19));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p20));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p21));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p22));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p23));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p24));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p25));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p26));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p27));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p28));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p29));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p30));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p31));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p32));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p33));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p34));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p35));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p36));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p37));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p38));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p39));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p40));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p41));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p42));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p43));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p44));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p45));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p46));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p47));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p48));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p49));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p50));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p51));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p52));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p53));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p54));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p55));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p56));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p57));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p58));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p59));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p60));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p61));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p62));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p63));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p64));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p65));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p66));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p67));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p68));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p69));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p70));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p71));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p72));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p73));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p74));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p75));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p76));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p77));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p78));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p79));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p80));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p80));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p81));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p82));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p83));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p84));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p85));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p86));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p87));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p88));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p89));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p90));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p91));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p92));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p93));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p94));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p95));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p96));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p97));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p98));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p99));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p100));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p101));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p102));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p103));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p104));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p105));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p106));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p107));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p108));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p109));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p110));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p111));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p112));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p113));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p114));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p115));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p116));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p117));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p118));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p119));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p120));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p121));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p122));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p123));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p124));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p125));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p126));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_p2p127));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm0));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm1));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm2));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm3));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm4));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm5));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm6));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm7));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm8));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm9));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm10));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm11));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm12));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm13));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm14));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm15));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm16));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm17));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm18));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm19));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm20));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm21));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm22));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm23));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm24));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm25));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm26));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm27));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm28));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm29));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm30));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm31));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm32));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm33));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm34));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm35));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm36));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm37));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm38));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm39));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm40));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm41));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm42));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm43));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm44));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm45));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm46));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm47));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm48));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm49));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm50));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm51));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm52));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm53));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm54));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm55));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm56));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm57));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm58));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm59));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm60));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm61));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm62));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm63));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm64));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm65));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm66));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm67));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm68));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm69));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm70));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm71));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm72));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm73));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm74));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm75));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm76));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm77));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm78));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm79));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm80));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm81));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm82));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm83));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm84));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm85));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm86));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm87));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm88));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm89));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm90));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm91));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm92));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm93));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm94));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm95));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm96));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm97));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm98));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm99));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm100));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm101));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm102));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm103));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm104));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm105));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm106));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm107));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm108));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm109));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm110));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm111));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm112));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm113));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm114));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm115));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm116));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm117));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm118));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm119));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm120));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm121));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm122));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm123));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm124));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm125));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm126));
	buf[buf_index++] = hssi_csrrd32(pdev, HSSI_PTP_HARDIP, hssi_port,
					eth_hardip_ptp_csroffs(cfg_tx_ptp0_asm127));
#endif
	hssi_unlock_mac_stats(pdev, hssi_port);
}

static void gts_get_pauseparam(struct net_device *dev,
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

static int gts_set_pauseparam(struct net_device *dev,
			      struct ethtool_pauseparam *pauseparam)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	int new_pause = FLOW_OFF;
	int ret = 0;
	struct platform_device *pdev  = priv->pdev_hssi;
	u32 hssi_port = priv->hssi_port;

	spin_lock(&priv->mac_cfg_lock);

	if (pauseparam->autoneg != 0) {
		ret = -EINVAL;
		goto out;
	}

	if (pauseparam->rx_pause) {
		new_pause |= FLOW_RX;
		hssi_set_bit(pdev, HSSI_EMAC_HARDIP, hssi_port,
			     eth_hardip_emac_csroffs(rxsfc_ehip_cfg),
			     ETH_RX_EN_STD_FLOW_CTRL);
	} else {
		hssi_clear_bit(pdev, HSSI_EMAC_HARDIP, hssi_port,
			       eth_hardip_emac_csroffs(rxsfc_ehip_cfg),
			       ETH_RX_EN_STD_FLOW_CTRL);
	}

	if (pauseparam->tx_pause) {
		new_pause |= FLOW_TX;
		hssi_set_bit(pdev, HSSI_EMAC_HARDIP, hssi_port,
			     eth_hardip_emac_csroffs(txsfc_ehip_cfg),
				ETH_TX_EN_STD_FLOW_CTRL);
	} else {
		hssi_clear_bit(pdev, HSSI_EMAC_HARDIP, hssi_port,
			       eth_hardip_emac_csroffs(txsfc_ehip_cfg),
			       ETH_TX_EN_STD_FLOW_CTRL);
	}

	hssi_csrwr32(pdev, HSSI_EMAC_HARDIP, hssi_port,
		     eth_hardip_emac_csroffs(pfc_pause_quanta_0), priv->pause);
	priv->flow_ctrl = new_pause;
out:
	spin_unlock(&priv->mac_cfg_lock);
	return ret;
}

static int gts_get_ts_info(struct net_device *dev,
			   struct kernel_ethtool_ts_info *info)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	return -EOPNOTSUPP;

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
static int gts_set_link_ksettings(struct net_device *dev,
				  const struct ethtool_link_ksettings *cmd)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	//if (!priv || !priv->phylink)
	if (!priv)
		return -ENODEV;

	return phylink_ethtool_ksettings_set(priv->phylink, cmd);
}

/* Get link ksettings (phy address, speed) for ethtools */
static int gts_get_link_ksettings(struct net_device *dev,
				  struct ethtool_link_ksettings *cmd)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	if (!priv)
		return -ENODEV;

	return phylink_ethtool_ksettings_get(priv->phylink, cmd);
}

static const struct ethtool_ops gts_ethtool_ops = {
	.get_drvinfo = gts_get_drvinfo,
	.get_regs_len = gts_reglen,
	.get_regs = gts_get_regs,
	.get_link = ethtool_op_get_link,
	.get_strings = gts_gstrings,
	.get_sset_count = gts_sset_count,
	.get_ethtool_stats = gts_fill_stats,
	.get_msglevel = gts_get_msglevel,
	.set_msglevel = gts_set_msglevel,
	.get_pauseparam = gts_get_pauseparam,
	.set_pauseparam = gts_set_pauseparam,
	.get_ts_info = gts_get_ts_info,
	.get_link_ksettings = gts_get_link_ksettings,
	.set_link_ksettings = gts_set_link_ksettings,
	.get_module_info = gts_get_module_info,
	.get_eeprom = gts_get_module_eeprom,
	.get_eeprom_len = gts_get_eeprom_len,
	.get_module_eeprom = gts_get_module_eeprom,
	.get_module_eeprom_by_page = gts_get_module_eeprom_by_page,

};

void intel_fpga_gts_set_ethtool_ops(struct net_device *netdev)
{
	netdev->ethtool_ops = &gts_ethtool_ops;
}

