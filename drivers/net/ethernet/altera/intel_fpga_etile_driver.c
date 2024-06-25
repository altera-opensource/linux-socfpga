// SPDX-License-Identifier: GPL-2.0
/* Intel FPGA E-tile Ethernet MAC driver
 * Copyright (C) 2022, 2024 Intel Corporation. All rights reserved
 *
 * Contributors:
 *   Preetam Narayan
 *
 */

#include <linux/phylink.h>
#include "intel_fpga_eth_etile.h"
#include "intel_fpga_eth_hssi_itf.h"
#include "intel_fpga_hssi_driver.h"
#include <linux/interrupt.h>

#define ETILE_EHIP_RESET_POLL_INTERVAL  5 /* in us */

/* WA : Remove check for local_fault and remote_fault from etile driver once fixed in HSSI IP */
static bool etile_check_local_remote_fault_status(intel_fpga_xtile_eth_private *priv)
{
	bool curr_link_state = true;

	u32 rx_mac_link_fault = hssi_csrrd32(priv->pdev_hssi,
					     HSSI_ETH_RECONFIG,
					     priv->tile_chan,
					     eth_rx_mac_csroffs(link_fault_status));

	if ((rx_mac_link_fault & ETH_RX_MAC_REMOTE_FAULT) ||
	    (rx_mac_link_fault & ETH_RX_MAC_LOCAL_FAULT)) {
		curr_link_state = false;
	}

	return curr_link_state;
}

bool etile_get_link_fault_status(intel_fpga_xtile_eth_private *priv)
{
	return  hssi_ethport_is_stable(priv->pdev_hssi, priv->hssi_port, false) &&
		etile_check_local_remote_fault_status(priv);
}

void etile_enable_mac(intel_fpga_xtile_eth_private *priv)
{
	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;

	/* Enable Tx MAC datapath */
	hssi_clear_bit(pdev, HSSI_ETH_RECONFIG, chan,
		       eth_tx_mac_csroffs(tx_mac_conf),
		       ETH_TX_MAC_DISABLE_TXVMAC);

	hssi_clear_bit(pdev, HSSI_ETH_RECONFIG, chan,
		       eth_rx_mac_csroffs(rx_mac_frwd_rx_crc),
		       ETH_RX_MAC_CRC_FORWARD);
}

void etile_disable_mac(intel_fpga_xtile_eth_private *priv)
{
	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;

	/* Disable Tx MAC datapath */
	hssi_set_bit_ba(pdev, HSSI_ETH_RECONFIG, chan,
			eth_tx_mac_csroffs(tx_mac_conf),
			ETH_TX_MAC_DISABLE_TXVMAC);

	hssi_clear_bit(pdev, HSSI_ETH_RECONFIG, chan,
		       eth_tx_mac_csroffs(tx_mac_conf),
		       ETH_TX_MAC_DISABLE_S_ADDR_EN);

	netif_warn(priv, drv, priv->dev, "Tx and Rx datapath stop done\n");
}

void etile_update_mac_addr(intel_fpga_xtile_eth_private *priv)
{
	u32 msb;
	u32 lsb;
	u32 chan = priv->tile_chan;
	const u8 *addr = priv->dev->dev_addr;

	struct platform_device *pdev = priv->pdev_hssi;

	lsb = (addr[2] << 24) | (addr[3] << 16) | (addr[4] << 8) | addr[5];
	msb = ((addr[0] << 8) | addr[1]) & 0xffff;

	/* Set MAC address */
	hssi_csrwr32(pdev, HSSI_ETH_RECONFIG, chan,
		     eth_tx_mac_csroffs(tx_mac_source_addr_lower_bytes), lsb);
	hssi_csrwr32(pdev, HSSI_ETH_RECONFIG, chan,
		     eth_tx_mac_csroffs(tx_mac_source_addr_higher_bytes), msb);

	hssi_clear_bit(pdev, HSSI_ETH_RECONFIG, chan,
		       eth_tx_mac_csroffs(tx_mac_conf), ETH_TX_MAC_DISABLE_S_ADDR_EN);
}

static void etile_enable_mac_flow_ctrl(intel_fpga_xtile_eth_private *priv)
{
	u32 reg;
	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;

	/* Rx MAC flow control */
	if ((priv->flow_ctrl & FLOW_RX)) {
		hssi_set_bit(pdev, HSSI_ETH_RECONFIG, chan,
			     eth_pause_and_priority_csroffs(rx_flow_control_feature_cfg),
			     ETH_RX_EN_STD_FLOW_CTRL);

		reg = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_pause_and_priority_csroffs(rx_flow_control_feature_cfg));

		if (netif_msg_ifup(priv))
			netdev_info(priv->dev, "E-tile rx_flow_ctrl: 0x%08x\n", reg);
	}

	/* Tx MAC flow control */
	if ((priv->flow_ctrl & FLOW_TX)) {
		hssi_set_bit(pdev, HSSI_ETH_RECONFIG, chan,
			     eth_pause_and_priority_csroffs(tx_flow_control_feature_cfg),
			     ETH_TX_EN_PRIORITY_FLOW_CTRL);

		reg = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_pause_and_priority_csroffs(tx_flow_control_feature_cfg));

		if (netif_msg_ifup(priv))
			netdev_info(priv->dev, "E-tile tx_flow_ctrl: 0x%08x\n", reg);
	}

	/* Set pfc pause quanta */
	hssi_csrwr32(pdev, HSSI_ETH_RECONFIG, chan,
		     eth_pause_and_priority_csroffs(pause_quanta_0), priv->pause);

	reg = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan,
			   eth_pause_and_priority_csroffs(pause_quanta_0));

	if (netif_msg_ifup(priv))
		netdev_info(priv->dev, "E-tile: pause_quanta0: 0x%08x\n", reg);
}

static void etile_disable_mac_flow_ctrl(intel_fpga_xtile_eth_private *priv)
{
	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;

	/* Disable Rx MAC flow control */
	if ((priv->flow_ctrl & FLOW_RX)) {
		hssi_clear_bit(pdev, HSSI_ETH_RECONFIG, chan,
			       eth_pause_and_priority_csroffs(rx_flow_control_feature_cfg),
			       ETH_RX_EN_STD_FLOW_CTRL);
	}

	/* Disable Tx MAC flow control */
	if ((priv->flow_ctrl & FLOW_TX)) {
		hssi_clear_bit(pdev, HSSI_ETH_RECONFIG, chan,
			       eth_pause_and_priority_csroffs(tx_flow_control_feature_cfg),
			       ETH_TX_EN_PRIORITY_FLOW_CTRL);
	}
}

int etile_check_counter_complete(intel_fpga_xtile_eth_private *priv, u32 regbank,
				 size_t offs, u8 bit_mask, bool set_bit, int align)
{
	int counter;
	u32 chan = priv->tile_chan;
	struct platform_device *pdev = priv->pdev_hssi;
	(void)align;
	counter = 0;

	while (counter++ < INTEL_FPGA_XTILE_SW_RESET_WATCHDOG_CNTR) {
		if (set_bit) {
			if (hssi_bit_is_set(pdev, regbank, chan,
					    offs, bit_mask))
				break;
		} else {
			if (hssi_bit_is_clear(pdev, regbank, chan,
					      offs, bit_mask))
				break;
		}
		udelay(1);
	}

	if (counter >= INTEL_FPGA_XTILE_SW_RESET_WATCHDOG_CNTR) {
		if (set_bit) {
			if (hssi_bit_is_clear(pdev, regbank, chan,
					      offs, bit_mask))
				return -EINVAL;
		} else {
			if (hssi_bit_is_set(pdev, regbank, chan,
					    offs, bit_mask))
				return -EINVAL;
		}
	}

	return 0;
}

static int eth_etile_tx_rx_user_flow(intel_fpga_xtile_eth_private *priv)
{
	u32 ui_value;
	u32 chan = priv->tile_chan;

	u32 tx_pma_delay_ns   = 0;
	u32 tx_extra_latency  = 0;
	u32 rx_fec_cw_pos     = 0;
	u32 rx_spulse_offset  = 0;
	u32 rx_pma_delay_ns   = 0;
	u32 rx_extra_latency  = 0;
	u8 rx_bitslip_cnt     = 0;
	u8 rx_fec_cw_pos_b0   = 0;
	u8 rx_fec_cw_pos_b8   = 0;
	const char *kr_fec    = "kr-fec";
	struct platform_device *pdev = priv->pdev_hssi;

	switch (priv->phy_iface) {
	case PHY_INTERFACE_MODE_10GKR:
	case PHY_INTERFACE_MODE_10GBASER:
		ui_value = INTEL_FPGA_ETILE_UI_VALUE_10G;
		break;

	case PHY_INTERFACE_MODE_25GKR:
		ui_value = INTEL_FPGA_ETILE_UI_VALUE_25G;
		break;

	default:
		return -ENODEV;
	}

	/*  Step 1 Calculate TX extra latency */
	/* Convert unit of TX PMA delay from UI to nanoseconds */
	tx_pma_delay_ns = INTEL_FPGA_TX_PMA_DELAY * ui_value;

	/* Get Tx external PHY delay from vendor and add in device tree
	 * and total up all extra latency together
	 */
	tx_extra_latency = (tx_pma_delay_ns + priv->tx_external_phy_delay_ns) >> 8;

	/* Step 2 Write TX extra latency*/
	hssi_csrwr32(pdev, HSSI_ETH_RECONFIG, chan,
		     eth_ptp_csroffs(tx_ptp_extra_latency), tx_extra_latency);

	// TX PTP is up
	// Adjust TX UI

	/* Check for 25G FEC variants */
	if ((priv->link_speed == SPEED_25000) &&
	    !strcasecmp(kr_fec, priv->fec_type)) {
		/*  Step 2a Read RX FEC codeword position */
		switch (priv->rsfec_cw_pos_rx) {
		case 0:
			rx_fec_cw_pos_b0 = hssi_csrrd8(pdev, HSSI_RSFEC, chan,
						       eth_rsfec_csroffs(rsfec_cw_pos_rx_0_b0));
			rx_fec_cw_pos_b8 = hssi_csrrd8(pdev, HSSI_RSFEC, chan,
						       eth_rsfec_csroffs(rsfec_cw_pos_rx_0_b8));
			break;
		case 1:
			rx_fec_cw_pos_b0 = hssi_csrrd8(pdev, HSSI_RSFEC, chan,
						       eth_rsfec_csroffs(rsfec_cw_pos_rx_1_b0));
			rx_fec_cw_pos_b8 = hssi_csrrd8(pdev, HSSI_RSFEC, chan,
						       eth_rsfec_csroffs(rsfec_cw_pos_rx_1_b8));
			break;

		case 2:
			rx_fec_cw_pos_b0 = hssi_csrrd8(pdev, HSSI_RSFEC, chan,
						       eth_rsfec_csroffs(rsfec_cw_pos_rx_1_b0));
			rx_fec_cw_pos_b8 = hssi_csrrd8(pdev, HSSI_RSFEC, chan,
						       eth_rsfec_csroffs(rsfec_cw_pos_rx_1_b8));
			break;
		case 3:
		default:
			rx_fec_cw_pos_b0 = hssi_csrrd8(pdev, HSSI_RSFEC, chan,
						       eth_rsfec_csroffs(rsfec_cw_pos_rx_3_b0));
			rx_fec_cw_pos_b8 = hssi_csrrd8(pdev, HSSI_RSFEC, chan,
						       eth_rsfec_csroffs(rsfec_cw_pos_rx_3_b8));
			break;
		}

		rx_fec_cw_pos = (rx_fec_cw_pos_b8 << 8) | rx_fec_cw_pos_b0;

		/* Step 3 Determine sync pulse (Alignment Marker)
		 * offsets with reference to async pulse
		 */
		rx_spulse_offset = (rx_fec_cw_pos * ui_value);

		netdev_info(priv->dev, "Rx FEC lane:%d codeword pos:%d ui value:0x%x\n",
			    priv->rsfec_cw_pos_rx, rx_fec_cw_pos, ui_value);

		/* Step 4 Calculate RX Extra latency and total up extra latency together */
		rx_pma_delay_ns = (INTEL_FPGA_RX_PMA_DELAY * ui_value);
		rx_extra_latency = ((rx_pma_delay_ns + priv->rx_external_phy_delay_ns -
				    rx_spulse_offset) >> 8) | 0x80000000;
	} else {
		/*  Step 2b Read bitslip count from IP */
		rx_bitslip_cnt = hssi_csrrd8(pdev,
					     HSSI_PHY_XCVR_PMAAVMM,
					     chan,
					     eth_pma_avmm_csroffs(reg_028));

		/* Step 3 Determine sync pulse (Alignment Marker)
		 * offsets with reference to async pulse
		 */
		rx_spulse_offset = (rx_bitslip_cnt * ui_value);

		if (rx_bitslip_cnt > 62) {
			rx_spulse_offset = (rx_bitslip_cnt - 66) * ui_value;
			if (rx_bitslip_cnt > 62 && rx_bitslip_cnt <= 66) {
				netdev_warn(priv->dev,
					    "rx_blitslip_cnt value :%d is incorrect!\n",
					    rx_bitslip_cnt);
			}
		}
		netdev_info(priv->dev, "Rx bitslip cnt:%d ui value:%x\n",
			    rx_bitslip_cnt, ui_value);

		/* Step 4 Calculate RX Extra latency and total up extra latency together */
		rx_pma_delay_ns = (INTEL_FPGA_RX_PMA_DELAY * ui_value);
		rx_extra_latency = ((rx_pma_delay_ns + rx_spulse_offset +
			priv->rx_external_phy_delay_ns) >> 8) | 0x80000000;
	}

	/* Step 5 Write RX extra Latency */
	hssi_csrwr32(pdev, HSSI_ETH_RECONFIG,
		     chan, eth_ptp_csroffs(rx_ptp_extra_latency),
		     rx_extra_latency);

	netdev_info(priv->dev, "tx_extra_latency:0x%x , rx_extra_latency:0x%x\n",
		    tx_extra_latency, rx_extra_latency);

	return 0;
}

static int etile_wait_reset_ack(struct platform_device *pdev, u32 chan,
				u32 rst_ack_mask, u32 maskval)
{
	unsigned long timeout, start;
	u32 val;

	start = jiffies;
	timeout = start + usecs_to_jiffies(FW_ACK_POLL_TIMEOUT_US);
	do {
		udelay(ETILE_EHIP_RESET_POLL_INTERVAL);
		val = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan,
				   eth_phy_csroffs(phy_config));

		if ((val & rst_ack_mask) == maskval)
			return 0;

	} while (time_before(jiffies, timeout));

	return -ETIME;
}

int etile_ehip_reset(intel_fpga_xtile_eth_private *priv,
		     bool tx, bool rx, bool sys)
{
	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;
	u32 rst_ack_mask = ETH_PHY_CONF_SOFT_TXP_RESET |
			   ETH_PHY_CONF_SOFT_RXP_RESET |
			   ETH_PHY_CONF_IO_SYS_RESET;
	u32 maskval = rst_ack_mask;
	u32 val;

	val = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan,
			   eth_phy_csroffs(phy_config));
	/* Trigger RX reset
	 * 1.   EHIP CSR Write, Offset = 0x310, value = 0x4
	 * Trigger TX reset
	 * 1.   EHIP CSR Write, Offset = 0x310, value = 0x2
	 * Trigger sys reset
	 * 1.   EHIP CSR Write, Offset = 0x310, value = 0x1
	 */
	if (tx) {
		val |= ETH_PHY_CONF_SOFT_TXP_RESET;
		maskval &= ~ETH_PHY_CONF_SOFT_TXP_RESET;
	}
	if (rx) {
		val |= ETH_PHY_CONF_SOFT_RXP_RESET;
		maskval &= ~ETH_PHY_CONF_SOFT_RXP_RESET;
	}

	if (sys) {
		val |= ETH_PHY_CONF_IO_SYS_RESET;
		maskval = 0;
}

	hssi_csrwr32(pdev, HSSI_ETH_RECONFIG, chan,
		     eth_phy_csroffs(phy_config), val);

	return etile_wait_reset_ack(pdev, chan, rst_ack_mask, maskval);
}

int etile_ehip_deassert_reset(intel_fpga_xtile_eth_private *priv)
{
	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;
	u32 rst_ack_mask = ETH_PHY_CONF_SOFT_TXP_RESET |
			   ETH_PHY_CONF_SOFT_RXP_RESET |
			   ETH_PHY_CONF_IO_SYS_RESET;
	u32 maskval = 0;
	u32 val;

	val = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan,
			   eth_phy_csroffs(phy_config));

	if (val & ETH_PHY_CONF_SOFT_TXP_RESET) {
		val &= ~ETH_PHY_CONF_SOFT_TXP_RESET;
		maskval |= ETH_PHY_CONF_SOFT_TXP_RESET;
	}

	if (val & ETH_PHY_CONF_SOFT_RXP_RESET) {
		val &= ~ETH_PHY_CONF_SOFT_RXP_RESET;
		maskval |= ETH_PHY_CONF_SOFT_RXP_RESET;
	}

	if (val & ETH_PHY_CONF_IO_SYS_RESET) {
		val &= ~ETH_PHY_CONF_IO_SYS_RESET;
		maskval = rst_ack_mask;
	}

	hssi_csrwr32(pdev, HSSI_ETH_RECONFIG, chan,
		     eth_phy_csroffs(phy_config), val);

	return etile_wait_reset_ack(pdev, chan, rst_ack_mask, maskval);
}

void etile_get_stats64(struct net_device *dev,
		       struct rtnl_link_stats64 *storage)
{
	/* a. All the blocking calls are avoided and only driver
	 * gathered statistics are populated. This is to avoid
	 * long spin locks
	 * b. Run time statistics can be dumped via ethtool
	 */

	storage->multicast  = 0;
	storage->collisions = 0;

	/* rx stats */
	storage->rx_crc_errors    = 0;
	storage->rx_over_errors   = 0;
	storage->rx_fifo_errors   = 0;
	storage->rx_missed_errors = 0;
	storage->rx_length_errors = 0;
	storage->rx_bytes   = dev->stats.rx_bytes;
	storage->rx_packets = dev->stats.rx_packets;
	storage->rx_dropped = dev->stats.rx_dropped;
	storage->rx_errors  = storage->rx_length_errors +
		storage->rx_crc_errors;

	/* tx stats */
	storage->tx_errors	         = 0;
	storage->tx_dropped          = 0;
	storage->rx_compressed	     = 0;
	storage->tx_compressed	     = 0;
	storage->tx_fifo_errors      = 0;
	storage->tx_window_errors    = 0;
	storage->tx_aborted_errors   = 0;
	storage->tx_heartbeat_errors = 0;
	storage->tx_bytes = dev->stats.tx_bytes;
	storage->tx_packets = dev->stats.tx_packets;
}

int etile_init(intel_fpga_xtile_eth_private *priv)
{
	/* Set/Config source MAC address */
	etile_update_mac_addr(priv);

	return 0;
}

int etile_start(intel_fpga_xtile_eth_private *priv)
{
	int ret;

	/* Enable E-tile MAC datapath */
	etile_enable_mac(priv);

	/* Enable flow ctrl */
	etile_enable_mac_flow_ctrl(priv);

	/* Enable PTP feature */
	if (priv->ptp_enable) {
		ret = eth_etile_tx_rx_user_flow(priv);
		if (ret < 0)
			goto ptp_error;

		/* Start UI thread */
		etile_ui_adjustments_init_worker(priv);
	}

	return 0;

ptp_error:
	etile_disable_mac(priv);
	etile_disable_mac_flow_ctrl(priv);
	return ret;
}

int etile_stop(intel_fpga_xtile_eth_private *priv)
{
	/* Disable etile MAC datapath */
	etile_disable_mac(priv);

	/* Stop UI thread */
	if (priv->ptp_enable)
		etile_ui_adjustments_cancel_worker(priv);

	/* Disable etile MAC flow ctrl */
	etile_disable_mac_flow_ctrl(priv);

	return 0;
}

int etile_uninit(intel_fpga_xtile_eth_private *priv)
{
	/* Just to make sure etile feature are disabled */
	return etile_stop(priv);
}
