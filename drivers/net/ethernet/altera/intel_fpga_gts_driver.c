// SPDX-License-Identifier: GPL-2.0
/* Intel FPGA E-tile Ethernet MAC driver
 * Copyright (C) 2022, 2024 Intel Corporation. All rights reserved
 *
 * Contributors:
 *   Preetam Narayan
 *
 * Original driver contributed by GlobalLogic.
 */
#include <linux/delay.h>
#include <linux/phylink.h>
#include "intel_fpga_eth_main.h"
#include "intel_fpga_eth_hssi_itf.h"
#include "intel_fpga_hssi_driver.h"
#include "intel_fpga_gts_driver.h"
#include "intel_fpga_eth_gts.h"

#define GTS_EHIP_RESET_POLL_INTERVAL      5 /* in us */

static int gts_wait_reset_ack(struct platform_device *pdev, u32 chan,
                                u32 rst_ack_mask, u32 maskval)
{
        unsigned long timeout, start;
        u32 val;

        start = jiffies;
        timeout = start + usecs_to_jiffies(FW_ACK_POLL_TIMEOUT_US);
        do {
                udelay(GTS_EHIP_RESET_POLL_INTERVAL);
                val = hssi_csrrd32(pdev, HSSI_BASE_SOFTIP, chan,
                                   eth_soft_csroffs(eth_reset_status));

                if ((val & rst_ack_mask) == maskval)
                        return 0;

        } while (time_before(jiffies, timeout));

        return -ETIME;
}

int gts_ehip_reset(intel_fpga_xtile_eth_private *priv,
                     bool tx, bool rx, bool sys)
{
        struct platform_device *pdev = priv->pdev_hssi;
        u32 chan = priv->tile_chan;
        u32 rst_ack_mask = ETH_SOFT_TX_RST | ETH_SOFT_RX_RST | ETH_EIO_SYS_RST;
        u32 maskval = rst_ack_mask;
        u32 val;

        val = hssi_csrrd32(pdev, HSSI_BASE_SOFTIP, chan,
                           eth_soft_csroffs(eth_reset));
        
	if (tx) {
                val |= ETH_SOFT_TX_RST;
                maskval &= ~ETH_SOFT_TX_RST;
        }

	if (rx) {
                val |= ETH_SOFT_RX_RST;
                maskval &= ~ETH_SOFT_RX_RST;
        }

	if (sys) {
                val |= ETH_EIO_SYS_RST;
                maskval = 0;
        }

        hssi_csrwr32(pdev, HSSI_BASE_SOFTIP, chan,
                        eth_soft_csroffs(eth_reset), val);

        return gts_wait_reset_ack(pdev, chan, rst_ack_mask, maskval);
}

int gts_ehip_deassert_reset(intel_fpga_xtile_eth_private *priv)
{
        struct platform_device *pdev = priv->pdev_hssi;
        u32 chan = priv->tile_chan;
        u32 rst_ack_mask = ETH_SOFT_TX_RST | ETH_SOFT_RX_RST | ETH_EIO_SYS_RST;
        u32 maskval = 0;
        u32 val;

        val = hssi_csrrd32(pdev, HSSI_BASE_SOFTIP, chan,
                           eth_soft_csroffs(eth_reset));

        if (val & ETH_SOFT_TX_RST) {
                val &= ~ETH_SOFT_TX_RST;
                maskval |= ETH_SOFT_TX_RST;
        }

        if (val & ETH_SOFT_RX_RST) {
                val &= ~ETH_SOFT_RX_RST;
                maskval |= ETH_SOFT_RX_RST;
        }

        if (val & ETH_EIO_SYS_RST) {
                val &= ~ETH_EIO_SYS_RST;
                maskval = rst_ack_mask;
        }

        hssi_csrwr32(pdev, HSSI_BASE_SOFTIP, chan,
                     eth_soft_csroffs(eth_reset), val);

        return gts_wait_reset_ack(pdev, chan, rst_ack_mask, maskval);
}

void gts_get_stats64(struct net_device *dev,
                       struct rtnl_link_stats64 *storage)
{
        intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
        struct platform_device *pdev = priv->pdev_hssi;
        u32 hssi_port = priv->hssi_port;

        /* rx stats */
        storage->rx_bytes = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_BYTES);

        storage->multicast = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_MULTICAST);;

        storage->collisions = 0;

        storage->rx_length_errors = 
		hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_UNDERSIZE) + 
		hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_OVERSIZE);

        storage->rx_over_errors = 0;

        storage->rx_crc_errors = 
		hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_CRC_ERRORS);;
        storage->rx_fifo_errors = 0;
        storage->rx_missed_errors = 0;
        //IP UG does not have total RX packets, total RX bad packets, total RX dropped packets
        storage->rx_packets = priv->dev->stats.rx_packets;
        storage->rx_errors = 
		hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_ERRORS);;
        storage->rx_dropped = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_ETHER_DROPS);;
        /* also count the packets dropped by this network driver */
        storage->rx_dropped += dev->stats.rx_dropped;

        /* tx stats */
        storage->tx_bytes = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_BYTES);

        storage->tx_errors = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_ERRORS);;

        storage->tx_dropped = 0;

        storage->tx_aborted_errors = 0;
        storage->tx_fifo_errors = 0;
        storage->tx_heartbeat_errors = 0;
        storage->tx_window_errors = 0;
        storage->rx_compressed = 0;
        storage->tx_compressed = 0;
        storage->tx_packets = priv->dev->stats.tx_packets;
}

void gts_update_mac_addr(intel_fpga_xtile_eth_private *priv)
{
        u32 msb;
        u32 lsb;
        u32 chan = priv->hssi_port;
        struct platform_device *pdev = priv->pdev_hssi;
        const u8 *addr = priv->dev->dev_addr;

        lsb = (addr[2] << 24) | (addr[3] << 16) | (addr[4] << 8) | addr[5];
        msb = ((addr[0] << 8) | addr[1]) & 0xffff;

        /* Set MAC address */
        hssi_csrwr32(pdev, HSSI_EMAC_HARDIP, chan,
                     eth_hardip_emac_csroffs(txmac_saddrl), lsb);
        hssi_csrwr32(pdev, HSSI_EMAC_HARDIP, chan,
                     eth_hardip_emac_csroffs(txmac_saddrh), msb);

        /* Enable Source address insertion */
        hssi_set_bit(pdev, HSSI_EMAC_HARDIP, chan,
                     eth_hardip_emac_csroffs(txmac_control), ETH_TX_MAC_ENABLE_S_ADDR_EN);

        netdev_info(priv->dev, "Device MAC address %pM\n", priv->dev->dev_addr);
}

void gts_enable_mac(intel_fpga_xtile_eth_private *priv)
{
        struct platform_device *pdev = priv->pdev_hssi;
        u32 chan = priv->hssi_port;

        /* Enable Tx MAC datapath */
        hssi_clear_bit(pdev, HSSI_EMAC_HARDIP, chan,
                       eth_hardip_emac_csroffs(txmac_control),
                       ETH_TX_MAC_DISABLE_TXMAC);

        hssi_clear_bit(pdev, HSSI_EMAC_HARDIP, chan,
                       eth_hardip_emac_csroffs(mac_crc_config),
                       ETH_RX_MAC_CRC_FORWARD);
}

void gts_disable_mac(intel_fpga_xtile_eth_private *priv)
{
        struct platform_device *pdev = priv->pdev_hssi;
        u32 chan = priv->hssi_port;

        /* Disable Tx MAC datapath */
        hssi_set_bit(pdev, HSSI_EMAC_HARDIP, chan,
                     eth_hardip_emac_csroffs(txmac_control),
                     ETH_TX_MAC_DISABLE_TXMAC);

        hssi_clear_bit(pdev, HSSI_EMAC_HARDIP, chan,
                     eth_hardip_emac_csroffs(txmac_control),
		     ETH_TX_MAC_ENABLE_S_ADDR_EN);

        netif_warn(priv, drv, priv->dev, "Tx and Rx datapath stop done\n");
}

int gts_init(intel_fpga_xtile_eth_private *priv)
{
        /* Set/Config source MAC address */
        gts_update_mac_addr(priv);

        return 0;
}

int gts_start(intel_fpga_xtile_eth_private *priv)
{
        /* Enable MAC datapath */
        gts_enable_mac(priv);

        return 0;
}

int gts_stop(intel_fpga_xtile_eth_private *priv)
{
        /* Disable Ftile MAC datapath */
        gts_disable_mac(priv);

        return 0;
}

int gts_uninit(intel_fpga_xtile_eth_private *priv)
{
        /* Just to make sure Ftile feature are disabled */
        return gts_stop(priv);
}

int gts_run_check(intel_fpga_xtile_eth_private *priv)
{
        return 0;
}

/* WA : Remove check for local_fault and remote_fault from etile driver once fixed in HSSI IP */
static bool gts_check_local_remote_fault_status(intel_fpga_xtile_eth_private *priv)
{
        bool curr_link_state = true;

        u32 rx_mac_link_fault = hssi_csrrd32(priv->pdev_hssi,
                                             HSSI_BASE_SOFTIP,
                                             priv->hssi_port,
                                             eth_soft_csroffs(link_fault_status));

        if ((rx_mac_link_fault & ETH_RX_MAC_REMOTE_FAULT) || 
	    (rx_mac_link_fault & ETH_RX_MAC_LOCAL_FAULT))
                curr_link_state = false;

        return curr_link_state;
}

bool gts_get_link_fault_status(intel_fpga_xtile_eth_private *priv)
{
        return  hssi_ethport_is_stable(priv->pdev_hssi, priv->hssi_port, false) &&
                gts_check_local_remote_fault_status(priv);
}

bool gts_check_dts_param(intel_fpga_xtile_eth_private *priv)
{
        struct platform_device *pdev;
        struct device_node *np;
        int ret;

        pdev = to_platform_device(priv->device);
        np = pdev->dev.of_node;

        if (of_property_read_u16(np, "pma_type",
                                 &priv->pma_type)) {
                dev_warn(&pdev->dev, "cannot obtain pma type defaulting to be FGT\n");
                priv->pma_type = 0;
        }

        if (priv->ptp_enable) {
                /* PTP Timestamp Accuracy mode */
                ret  = of_property_read_string(pdev->dev.of_node, "ptp_accu_mode",
                                               &priv->ptp_accu_mode);
                if (ret < 0)
                        priv->ptp_accu_mode = "Basic";

                if (strcasecmp(priv->ptp_accu_mode, "Advanced") == 0) {
                        /* Tx Routing adjustment delay */
                        if (of_property_read_u32(np, "ptp_tx_routing_adj",
                                                 &priv->ptp_tx_routing_adj)) {
                                priv->ptp_tx_routing_adj = 0;
                        }

                        /* Rx Routing adjustment delay */
                        if (of_property_read_u32(np, "ptp_rx_routing_adj",
                                                 &priv->ptp_rx_routing_adj)) {
                                priv->ptp_rx_routing_adj = 0;
                        }
                }
        }

        return true;
}
