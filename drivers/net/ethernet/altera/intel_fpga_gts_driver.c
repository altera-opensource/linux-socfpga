// SPDX-License-Identifier: GPL-2.0
/* Altera FPGA E-tile Ethernet MAC driver
 * Copyright (C) 2022, 2025 Altera Corporation. All rights reserved
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

static void gts_enable_mac_flow_ctrl(intel_fpga_xtile_eth_private *priv)
{
	u32 reg;
	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;

	/* Rx MAC flow control */
	if (priv->flow_ctrl & FLOW_RX) {
		hssi_set_bit(pdev, HSSI_EMAC_HARDIP, chan,
			     eth_hardip_emac_csroffs(rxsfc_ehip_cfg),
			     ETH_RX_EN_STD_FLOW_CTRL);

		reg = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, chan,
				   eth_hardip_emac_csroffs(rxsfc_ehip_cfg));

		if (netif_msg_ifup(priv))
			netdev_info(priv->dev, "GTS rx_flow_ctrl: 0x%08x\n", reg);
	}

	/* Tx MAC flow control */
	if (priv->flow_ctrl & FLOW_TX) {
		hssi_set_bit(pdev, HSSI_EMAC_HARDIP, chan,
			     eth_hardip_emac_csroffs(txsfc_ehip_cfg),
			     ETH_TX_EN_PRIORITY_FLOW_CTRL);

		reg = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, chan,
				   eth_hardip_emac_csroffs(txsfc_ehip_cfg));

		if (netif_msg_ifup(priv))
			netdev_info(priv->dev, "GTS tx_flow_ctrl: 0x%08x\n", reg);
	}

	/* Set pfc pause quanta */
	if (priv->flow_ctrl & FLOW_TX) {
		hssi_csrwr32(pdev, HSSI_EMAC_HARDIP, chan,
			     eth_hardip_emac_csroffs(pfc_pause_quanta_0), priv->pause);

		reg = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, chan,
				   eth_hardip_emac_csroffs(pfc_pause_quanta_0));

		if (netif_msg_ifup(priv))
			netdev_info(priv->dev, "GTS: pause_quanta0: 0x%08x\n", reg);
	}
}

static void gts_disable_mac_flow_ctrl(intel_fpga_xtile_eth_private *priv)
{
	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;

	/* Disable Rx MAC flow control */
	if ((priv->flow_ctrl & FLOW_RX)) {
		hssi_clear_bit(pdev, HSSI_EMAC_HARDIP, chan,
			       eth_hardip_emac_csroffs(rxsfc_ehip_cfg),
			       ETH_RX_EN_STD_FLOW_CTRL);
	}

	/* Disable Tx MAC flow control */
	if ((priv->flow_ctrl & FLOW_TX)) {
		hssi_clear_bit(pdev, HSSI_EMAC_HARDIP, chan,
			       eth_hardip_emac_csroffs(txsfc_ehip_cfg),
			       ETH_TX_EN_PRIORITY_FLOW_CTRL);
	}
}

void gts_get_stats64(struct net_device *dev,
		     struct rtnl_link_stats64 *storage)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	struct platform_device *pdev = priv->pdev_hssi;
	u32 hssi_port = priv->hssi_port;

	/* rx stats */
	storage->rx_bytes = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_BYTES);

	storage->multicast = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_MULTICAST);

	storage->collisions = 0;

	storage->rx_length_errors =
		hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_UNDERSIZE) +
		hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_OVERSIZE);

	storage->rx_over_errors = 0;

	storage->rx_crc_errors =
		hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_CRC_ERRORS);
	storage->rx_fifo_errors = 0;
	storage->rx_missed_errors = 0;
	//IP UG does not have total RX packets, total RX bad packets, total RX dropped packets
	storage->rx_packets = priv->dev->stats.rx_packets;
	storage->rx_errors =
		hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_RX_ERRORS);
	storage->rx_dropped = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_ETHER_DROPS);
	/* also count the packets dropped by this network driver */
	storage->rx_dropped += dev->stats.rx_dropped;

	/* tx stats */
	storage->tx_bytes = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_BYTES);

	storage->tx_errors = hssi_read_mac_stats64(pdev, hssi_port, MACSTAT_TX_ERRORS);

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

static bool gts_is_fec_type(intel_fpga_xtile_eth_private *priv)
{
	return !!strcasecmp(priv->fec_type, "no-fec");
}

static int gts_rx_user_flow(intel_fpga_xtile_eth_private *priv)
{
	u32 regval = 0;
	u32  rx_const_delay = 0;
	u32 rx_xcvr_if_pulse_adj = 0;
	u32 rx_apulse_offset = 0;
	bool rx_const_is_neg = false;
	bool rx_apulse_is_neg = false;
	bool rx_spulse_is_neg = false;
	u32 rx_spulse_offset = 0;
	u32 rx_apulse_wdelay = 0;
	u32 rx_apulse_time = 0;
	u32 rx_tam_adjust = 0;
	s32 rx_tam_adjust_2c = 0;
	u32 rx_pma_delay_ns = 0;
	u32 rx_extra_latency = 0;
	u32 rx_ui_value = 0;
	u32 pma_delay = 0;
	u8  rx_bitslip_cnt;
	u32 chan = priv->hssi_port;
	struct platform_device *pdev = priv->pdev_hssi;

	switch (priv->phy_iface) {
	case PHY_INTERFACE_MODE_10GKR:
	case PHY_INTERFACE_MODE_10GBASER:
		rx_ui_value = INTEL_FPGA_GTS_UI_VALUE_10G;
		pma_delay = INTEL_FPGA_RX_PMA_DELAY_10G;
		break;
	case PHY_INTERFACE_MODE_25GKR:
	case PHY_INTERFACE_MODE_25GBASER:
		rx_ui_value = INTEL_FPGA_GTS_UI_VALUE_25G;
		pma_delay = INTEL_FPGA_RX_PMA_DELAY_25G;
		break;
	default:
		BUG_ON(priv->phy_iface);
	}

	/* Step 1: After power up, reset or link down, wait until RX PCS is fully aligned */
	if (xtile_check_counter_complete(priv, HSSI_PCS_FEC_HARDIP,
					 eth_hardip_pcsfec_csroffs(phy_rxpcs_status),
					 ETH_PHY_RX_PCS_ALIGNED, true,
					 INTEL_FPGA_WORD_ALIGN)) {
		netdev_err(priv->dev, "MAC Rx datapath not ready (PHY_RX_PCS_ALIGNED=0)\n");
		return -EINVAL;
	}

	/* Step 2: its a fec type */
	if (gts_is_fec_type(priv)) {
		/* Step 2a: Write value of 0x0 for pulse adjustment into IP*/
		regval = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, chan,
				      eth_hardip_pma_csroffs(cfg_rx_lat_bit_for_async));
		regval &= ~RX_LAT_ASYNC_MASK;
		hssi_csrwr32(pdev, HSSI_PMA_HARDIP, chan,
			     eth_hardip_pma_csroffs(cfg_rx_lat_bit_for_async), regval);

		/* Step 2b: Read RX FEC codeword position and FEC channel mapping for each PMA */
		regval = hssi_csrrd32(pdev, HSSI_PCS_FEC_HARDIP, chan,
				      eth_hardip_pcsfec_csroffs(rsfec_cw_pos_rx));
		rx_xcvr_if_pulse_adj = regval & RSFEC_CW_POS_MASK;

		/* Step 2d: Write the pulse adjustments into the IP */
		regval = hssi_csrrd32(pdev, HSSI_PMA_HARDIP, chan,
				      eth_hardip_pma_csroffs(cfg_rx_lat_bit_for_async));
		regval &= ~RX_LAT_ASYNC_MASK;
		regval |= RX_LAT_ASYNC_MASK & rx_xcvr_if_pulse_adj;
		hssi_csrwr32(pdev, HSSI_PMA_HARDIP, chan,
			     eth_hardip_pma_csroffs(cfg_rx_lat_bit_for_async), regval);

		/* Step 2e: Notify soft PTP that pulse adjustments have been configured */
		  regval = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, chan,
					eth_softip_ptp_csroffs(ptp_rx_user_cfg_status));
		  regval |= ETH_PTP_RX_FEC_CW_POS_DONE;

		  hssi_csrwr32(pdev, HSSI_PTP_SOFTIP, chan,
			       eth_softip_ptp_csroffs(ptp_rx_user_cfg_status), regval);
	}

	/* Step 3: Wait until RX raw offset data are ready */
	if (xtile_check_counter_complete(priv, HSSI_PTP_SOFTIP,
					 eth_softip_ptp_csroffs(ptp_status),
					 ETH_RX_PTP_OFFSET_DATA_VALID, true,
					 INTEL_FPGA_WORD_ALIGN)) {
		netdev_err(priv->dev, "PTP Rx calculation data invalid\n");
		return -EINVAL;
	}

	/* Step 4: Read RX raw offset data from IP */
	regval = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, chan,
			      eth_softip_ptp_csroffs(ptp_rx_lane_calc_data_constdelay));

	rx_const_is_neg = (regval & BIT(31)) ? true : false;
	rx_const_delay = rx_const_is_neg ? (regval & ~BIT(31)) : regval;

	regval = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, chan,
			      eth_softip_ptp_csroffs(ptp_rx_lane0_calc_data_offset));

	rx_apulse_is_neg = (regval & BIT(31)) ? true : false;
	rx_apulse_offset = rx_apulse_is_neg ? (regval & ~BIT(31)) : regval;

	regval = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, chan,
			      eth_softip_ptp_csroffs(ptp_rx_lane0_calc_data_wiredelay));
	rx_apulse_wdelay = regval & GENMASK(19, 0);

	regval = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, chan,
			      eth_softip_ptp_csroffs(ptp_rx_lane0_calc_data_time));
	rx_apulse_time = regval & GENMASK(27, 0);

	/* Step 4a: 10GE/25GE no FEC variants */
	if (!gts_is_fec_type(priv)) {
		u8 rx_dlpulse_cnt;
		u64 bslip_p_dlpulse;

		regval = hssi_csrrd32(pdev, HSSI_EMAC_HARDIP, chan,
				      eth_hardip_emac_csroffs(phy_rx_bitslip_cnt));
		rx_bitslip_cnt = regval & GENMASK(7, 0);
		rx_dlpulse_cnt = (regval & ETH_PHY_RX_PCS_DLPULSE_ALIGNED) ? 33 : 0;

		bslip_p_dlpulse = (u64)(rx_bitslip_cnt + rx_dlpulse_cnt) * rx_ui_value;
		rx_spulse_offset = (bslip_p_dlpulse & GENMASK(34, 12)) >> 12;
		rx_spulse_is_neg = false;
	}

	/* Step 5: Determine synchronous pulse AM offsets with reference to asynchronous pulse */
	if (gts_is_fec_type(priv)) {
		/* Step 5a: For FEC variant */
		rx_spulse_offset = ((rx_xcvr_if_pulse_adj & GENMASK(4, 0)) * rx_ui_value);
		rx_spulse_offset >>= (28 - 16);
		rx_spulse_is_neg = false;
	}

	/* Step 6: Calculate Rx offsets */
	/* Step 6a: Calculate Rx TAM adjust for FEC/non-FEC variant */
	rx_tam_adjust = (rx_const_is_neg ? -rx_const_delay : rx_const_delay) +
			(rx_apulse_is_neg ? -rx_apulse_offset : rx_apulse_offset) +
			(rx_spulse_is_neg ? -rx_spulse_offset : rx_spulse_offset) -
			rx_apulse_wdelay;

	/* Convert TAM adjust to a 32-bit 2's complement number */
	rx_tam_adjust_2c = rx_tam_adjust;

	/* Step 6b: Calculate RX extra latency */
	/* The UI format differs from the format of other variables.
	 * UI uses the {4-bit ns, 28-bit fractional ns} format.
	 * Other variables defined in this flow use the
	 * {N-bit ns, 16-bit fractional ns} format, where N is the largest number
	 * to store the calculation's maximum value.
	 * If you use UI format in your calculation, you must convert
	 * your result to a 16-bit fractional ns format
	 */
	rx_pma_delay_ns = (rx_ui_value * pma_delay) >> (28 - 16);

	/* RX extra latency is a positive adjustment and
	 * to indicate the positive adjustment, set the most-significant register bit to 1.
	 * total up all extra latency together
	 */
	rx_extra_latency = rx_pma_delay_ns + priv->rx_external_phy_delay_ns;
	rx_extra_latency |= BIT(31);

	/* Step 7: Write the calculated RX offsets to IP */
	/* Step 7a: Write RX extra latency */
	hssi_csrwr32(pdev, HSSI_EMAC_HARDIP, chan,
		     eth_hardip_emac_csroffs(rx_ptp_extra_latency), rx_extra_latency);

	/* Step 7b: Write Rx TAM adjust */
	hssi_csrwr32(pdev, HSSI_PTP_SOFTIP, chan,
		     eth_softip_ptp_csroffs(ptp_rx_tam_adjust), rx_tam_adjust_2c);

	/* Step 8: Notify soft PTP that user flow configuration is completed */
	regval = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, chan,
			      eth_softip_ptp_csroffs(ptp_rx_user_cfg_status));
	regval |= ETH_PTP_RX_USER_CFG_DONE;
	hssi_csrwr32(pdev, HSSI_PTP_SOFTIP, chan,
		     eth_softip_ptp_csroffs(ptp_rx_user_cfg_status), regval);

	/* Step 9: Programming 0PPM UI value */
	hssi_csrwr32(pdev, HSSI_EMAC_HARDIP, chan,
		     eth_hardip_emac_csroffs(rx_ptp_ui), rx_ui_value);

	/* Step 10: Wait until RX PTP is ready */
	if (xtile_check_counter_complete(priv, HSSI_PTP_SOFTIP,
					 eth_softip_ptp_csroffs(ptp_status),
					 ETH_RX_PTP_READY, true,
					 INTEL_FPGA_WORD_ALIGN)) {
		netdev_err(priv->dev, "MAC Rx PTP not ready\n");
		return -EINVAL;
	}

	dev_info(priv->device,
		 "INFO: ETH_RX_PTP_READY - rx_extra_latency:0x%08x rx_tam_adjust:%i\n",
		 rx_extra_latency, (int32_t)rx_tam_adjust);

	return 0;
}

// PTP Tx user flow
static int gts_tx_user_flow(intel_fpga_xtile_eth_private *priv)
{
	u32 regval = 0;
	u32  tx_const_delay = 0;
	bool tx_const_is_neg = false;
	u32 tx_apulse_offset = 0;
	bool tx_apulse_is_neg = false;
	u32 tx_apulse_wdelay = 0;
	u32 tx_tam_adjust = 0;
	s32 tx_tam_adjust_2c = 0;
	u32 tx_pma_delay_ns = 0;
	u32 tx_extra_latency = 0;
	u32 tx_ui_value = 0;
	u32 pma_delay = 0;
	u32 chan = priv->hssi_port;
	struct platform_device *pdev = priv->pdev_hssi;

	/* Step 1: After power up or reset, wait until TX raw offset data are ready */
	if (xtile_check_counter_complete(priv, HSSI_PTP_SOFTIP,
					 eth_softip_ptp_csroffs(ptp_status),
					 ETH_TX_PTP_OFFSET_DATA_VALID, true,
					 INTEL_FPGA_WORD_ALIGN)) {
		netdev_err(priv->dev, "PTP Tx calculation data invalid\n");
		return -EINVAL;
	}

	/* Step 2: Read TX raw offset data from IP */
	regval = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, chan,
			      eth_softip_ptp_csroffs(ptp_tx_lane_calc_data_constdelay));

	tx_const_is_neg = (regval & BIT(31)) ? true : false;
	tx_const_delay = tx_const_is_neg ? (regval & ~BIT(31)) : regval;

	regval = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, chan,
			      eth_softip_ptp_csroffs(ptp_tx_lane0_calc_data_offset));

	tx_apulse_is_neg = (regval & BIT(31)) ? true : false;
	tx_apulse_offset = tx_apulse_is_neg ? (regval & ~BIT(31)) : regval;

	regval = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, chan,
			      eth_softip_ptp_csroffs(ptp_tx_lane0_calc_data_wiredelay));
	tx_apulse_wdelay = regval & GENMASK(19, 0);

	/* Step 3: Calculate TAM offsets */

	/* Step 3a: Calculate TAM adjust */
	tx_tam_adjust = (tx_const_is_neg ? -tx_const_delay : tx_const_delay) +
			(tx_apulse_is_neg ? -tx_apulse_offset : tx_apulse_offset) -
			tx_apulse_wdelay;

	/* Convert TAM adjust to a 32-bit 2's complement number */
	tx_tam_adjust_2c = tx_tam_adjust;

	/* Step 3b: Calculate TX extra latency */
	/* The UI format differs from the format of other variables.
	 * UI uses the {4-bit ns, 28-bit fractional ns} format.
	 * Other variables defined in this flow use the
	 * {N-bit ns, 16-bit fractional ns} format, where N is the largest number
	 * to store the calculation's maximum value.
	 * If you use UI format in your calculation, you must convert
	 * your result to a 16-bit fractional ns format
	 */
	switch (priv->phy_iface) {
	case PHY_INTERFACE_MODE_10GKR:
	case PHY_INTERFACE_MODE_10GBASER:
		tx_ui_value = INTEL_FPGA_GTS_UI_VALUE_10G;
		pma_delay = INTEL_FPGA_TX_PMA_DELAY_10G;
		break;
	case PHY_INTERFACE_MODE_25GKR:
	case PHY_INTERFACE_MODE_25GBASER:
		tx_ui_value = INTEL_FPGA_GTS_UI_VALUE_25G;
		pma_delay = INTEL_FPGA_TX_PMA_DELAY_25G;
		break;
	default:
		BUG_ON(priv->phy_iface);
	}

	tx_pma_delay_ns = (tx_ui_value * pma_delay) >> (28 - 16);

	/* TX extra latency is a positive adjustment and
	 * to indicate the positive adjustment, set the most-significant register bit to 0.
	 * total up all extra latency together
	 */
	tx_extra_latency = tx_pma_delay_ns + priv->tx_external_phy_delay_ns;
	tx_extra_latency &= ~BIT(31);

	/* Step 4: Write the calculated TX offsets to IP */
	/* Step 4a: Write TX extra latency */
	hssi_csrwr32(pdev, HSSI_EMAC_HARDIP, chan,
		     eth_hardip_emac_csroffs(tx_ptp_extra_latency), tx_extra_latency);

	/* Step 4b: Write Tx TAM adjust */
	hssi_csrwr32(pdev, HSSI_PTP_SOFTIP, chan,
		     eth_softip_ptp_csroffs(ptp_tx_tam_adjust), tx_tam_adjust_2c);

	/* Step 5: Programming 0PPM UI value */
	hssi_csrwr32(pdev, HSSI_EMAC_HARDIP, chan,
		     eth_hardip_emac_csroffs(tx_ptp_ui), tx_ui_value);

	/* Step 6: Notify soft PTP that user flow configuration is completed */
	regval = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, chan,
			      eth_softip_ptp_csroffs(ptp_tx_user_cfg_status));
	regval |= ETH_PTP_TX_USER_CFG_DONE;
	hssi_csrwr32(pdev, HSSI_PTP_SOFTIP, chan,
		     eth_softip_ptp_csroffs(ptp_tx_user_cfg_status), regval);

	/* Step 7: Wait until TX PTP is ready */
	if (xtile_check_counter_complete(priv, HSSI_PTP_SOFTIP,
					 eth_softip_ptp_csroffs(ptp_status),
					 ETH_TX_PTP_READY, true,
					 INTEL_FPGA_WORD_ALIGN)) {
		netdev_err(priv->dev, "MAC Tx PTP not ready\n");
		return -EINVAL;
	}

	dev_info(priv->device,
		 "INFO: ETH_TX_PTP_READY - tx_extra_latency:0x%08x tx_tam_adjust:%i\n",
		 tx_extra_latency, (int32_t)tx_tam_adjust);

	return 0;
}

static int gts_tx_rx_user_flow(intel_fpga_xtile_eth_private *priv)
{
	int ret = 0;

	ret = gts_tx_user_flow(priv);
	if (!ret)
		ret = gts_rx_user_flow(priv);

	return ret;
}

int gts_init(intel_fpga_xtile_eth_private *priv)
{
	/* Set/Config source MAC address */
	gts_update_mac_addr(priv);

	/* Enable MAC datapath */
	gts_enable_mac(priv);

	/* Enable flow ctrl */
	gts_enable_mac_flow_ctrl(priv);

	return 0;
}

int gts_start(intel_fpga_xtile_eth_private *priv)
{
	int ret = 0;

	/* Enable PTP feature */
	if (priv->ptp_enable) {
		ret = gts_tx_rx_user_flow(priv);
		if (ret < 0)
			goto ptp_error;

		/* Start UI thread */
		gts_ui_adjustments_init_worker(priv);
	}

	hssi_errpkt_cnt_reset(priv->pdev_hssi, priv->tile_chan);

	return 0;
ptp_error:
	return -1;
}

int gts_stop(intel_fpga_xtile_eth_private *priv)
{
	/* Stop UI thread */
	if (priv->ptp_enable)
		gts_ui_adjustments_cancel_worker(priv);

	return 0;
}

int gts_uninit(intel_fpga_xtile_eth_private *priv)
{
	/* Disable GTS MAC datapath */
	gts_disable_mac(priv);

	/* Disable GTS MAC datapath */
	gts_disable_mac_flow_ctrl(priv);

	/* Just to make sure GTS feature are disabled */
	return gts_stop(priv);
}

static bool gts_ptp_rx_ready_bit_is_set(intel_fpga_xtile_eth_private *priv)
{
	bool is_set = true;

	if (priv->ptp_enable) {
		// Check PTP RX ready bit set or not set,
		// If not, we need rerun ptp tx rx user flow again
		is_set = hssi_bit_is_set_ba(priv->pdev_hssi, HSSI_PTP_SOFTIP, priv->tile_chan,
					    eth_softip_ptp_csroffs(ptp_status), ETH_RX_PTP_READY);
	}

	return is_set;
}

int gts_run_check(intel_fpga_xtile_eth_private *priv)
{
	int ret;

	/* Check ptp rx ready bit is toggled,
	 * if yes, stop UI and rerun ptp tx rx user flow
	 */
	if (!gts_ptp_rx_ready_bit_is_set(priv)) {
		gts_ui_adjustments_cancel_worker(priv);

		ret = gts_tx_rx_user_flow(priv);
		if (ret)
			return ret;

		gts_ui_adjustments_init_worker(priv);
	}

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

int gts_check_counter_complete(intel_fpga_xtile_eth_private *priv, u32 regbank,
			       size_t offs, u8 bit_mask, bool set_bit, int align)
{
	u32 chan = priv->tile_chan;
	struct platform_device *pdev = priv->pdev_hssi;
	(void)align;

	if (set_bit) {
		if (hssi_bit_is_clear(pdev, regbank, chan,
				      offs, bit_mask))
			return -EINVAL;
	} else {
		if (hssi_bit_is_set(pdev, regbank, chan,
				    offs, bit_mask))
			return -EINVAL;
	}

	return 0;
}

