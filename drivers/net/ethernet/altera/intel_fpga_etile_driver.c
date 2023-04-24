// SPDX-License-Identifier: GPL
/* Intel FPGA E-tile Ethernet MAC driver
 * Copyright (C) 2022,2023 Intel Corporation. All rights reserved
 *
 * Contributors:
 *   Preetam Narayan
 *
 */

#include <linux/phylink.h>
#include "intel_fpga_eth_etile.h"
#include "intel_fpga_eth_hssi_itf.h"
#include <linux/interrupt.h>

static void etile_set_mac(intel_fpga_xtile_eth_private *priv,
			  bool enable)
{
	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;

	if (enable) {
		/* Enable Tx datapath */
		hssi_clear_bit(pdev, HSSI_ETH_RECONFIG, chan,
			      eth_tx_mac_csroffs(tx_mac_conf),
			      ETH_TX_MAC_DISABLE_TXVMAC,
			      false);

		hssi_clear_bit(pdev, HSSI_ETH_RECONFIG, chan,
			       eth_rx_mac_csroffs(rx_mac_frwd_rx_crc),
			       ETH_RX_MAC_CRC_FORWARD,
			       false);
	} else {
		/* Disable Tx datapath */
		hssi_set_bit(pdev, HSSI_ETH_RECONFIG, chan,
			    eth_tx_mac_csroffs(tx_mac_conf),
			    ETH_TX_MAC_DISABLE_TXVMAC,
			    false);

		hssi_clear_bit(pdev, HSSI_ETH_RECONFIG, chan,
			      eth_tx_mac_csroffs(tx_mac_conf),
			      ETH_TX_MAC_DISABLE_S_ADDR_EN,
			      false);

		netif_warn(priv, drv, priv->dev, "Tx and Rx datapath stop done\n");
	}
}

void etile_update_mac_addr(intel_fpga_xtile_eth_private *priv)
{
	u32 msb;
	u32 lsb;
	u32 chan = priv->tile_chan;
	u8 *addr = priv->dev->dev_addr;

	struct platform_device *pdev = priv->pdev_hssi;

	lsb = (addr[2] << 24) | (addr[3] << 16) | (addr[4] << 8) | addr[5];
	msb = ((addr[0] << 8) | addr[1]) & 0xffff;

	/* Set MAC address */
	hssi_csrwr32(pdev, HSSI_ETH_RECONFIG, chan,
		     eth_tx_mac_csroffs(tx_mac_source_addr_lower_bytes), lsb);
	hssi_csrwr32(pdev, HSSI_ETH_RECONFIG, chan,
		     eth_tx_mac_csroffs(tx_mac_source_addr_higher_bytes), msb);

	hssi_set_bit(pdev, HSSI_ETH_RECONFIG, chan,
		     eth_tx_mac_csroffs(tx_mac_conf), ETH_TX_MAC_DISABLE_S_ADDR_EN,false);
}

static void etile_set_mac_flow_ctrl(intel_fpga_xtile_eth_private *priv)
{
	u32 reg;
	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;

	if (priv->flow_ctrl & FLOW_RX)

		hssi_set_bit(pdev, HSSI_ETH_RECONFIG, chan,
			    eth_pause_and_priority_csroffs(rx_flow_control_feature_cfg),
			    ETH_RX_EN_STD_FLOW_CTRL,false);
	else
		hssi_clear_bit(pdev, HSSI_ETH_RECONFIG, chan,
			      eth_pause_and_priority_csroffs(rx_flow_control_feature_cfg),
			      ETH_RX_EN_STD_FLOW_CTRL,false);

	reg = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan,
		      eth_pause_and_priority_csroffs(rx_flow_control_feature_cfg));

	if (netif_msg_ifup(priv))
		netdev_info(priv->dev, "E-tile rx_flow_ctrl: 0x%08x\n", reg);

	if (priv->flow_ctrl & FLOW_TX) {

		hssi_set_bit(pdev, HSSI_ETH_RECONFIG, chan,
			    eth_pause_and_priority_csroffs(tx_flow_control_feature_cfg),
			    ETH_TX_EN_PRIORITY_FLOW_CTRL,false);
	} else {
		hssi_clear_bit(pdev, HSSI_ETH_RECONFIG, chan,
			      eth_pause_and_priority_csroffs(tx_flow_control_feature_cfg),
			      ETH_TX_EN_PRIORITY_FLOW_CTRL,false);
	}

	reg = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan,
			   eth_pause_and_priority_csroffs(tx_flow_control_feature_cfg));

	if (netif_msg_ifup(priv))
		netdev_info(priv->dev, "E-tile tx_flow_ctrl: 0x%08x\n", reg);

	hssi_csrwr32(pdev, HSSI_ETH_RECONFIG, chan,
		eth_pause_and_priority_csroffs(pause_quanta_0), priv->pause);

	reg = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan,
		      eth_pause_and_priority_csroffs(pause_quanta_0));

	if (netif_msg_ifup(priv))
		netdev_info(priv->dev, "E-tile: pause_quanta0: 0x%08x\n", reg);
}

static int eth_etile_tx_rx_user_flow(intel_fpga_xtile_eth_private *priv)
{
	int ret;
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
	if (priv->link_speed == SPEED_25000 &&
	    (!strcasecmp(kr_fec, priv->fec_type))) {
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
		rx_bitslip_cnt = hssi_csrrd8(pdev, HSSI_PHY_XCVR_PMAAVMM, chan, eth_pma_avmm_csroffs(reg_028));

		/* Step 3 Determine sync pulse (Alignment Marker)
		 * offsets with reference to async pulse
		 */
		if (rx_bitslip_cnt > 62) {
			rx_spulse_offset = (rx_bitslip_cnt - 66) * ui_value;
			if (rx_bitslip_cnt > 62 && rx_bitslip_cnt <= 66) {
				netdev_warn(priv->dev,
					    "rx_blitslip_cnt value :%d is incorrect!\n",
					    rx_bitslip_cnt);
			}
		}
		rx_spulse_offset = (rx_bitslip_cnt * ui_value);

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

#if 0
	/* Adjust UI value */
	timer_setup(&priv->fec_timer, ui_adjustments, 0);
	ret = mod_timer(&priv->fec_timer, jiffies + msecs_to_jiffies(5000));
	if (ret)
		netdev_err(priv->dev, "Timer failed to start UI adjustment\n");
#endif
	return 0;
}

int etile_ehip_reset(intel_fpga_xtile_eth_private *priv,
			bool tx, bool rx, bool sys)
{
	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;
	u32 val;
	int ret;

	val = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan,
			eth_phy_csroffs(phy_config));

	if (tx)
		val |= ETH_PHY_CONF_SOFT_TXP_RESET;
	if (rx)
		val |= ETH_PHY_CONF_SOFT_RXP_RESET;
	if (sys)
		val |= ETH_PHY_CONF_IO_SYS_RESET;

	hssi_csrwr32(pdev, HSSI_ETH_RECONFIG, chan,
			eth_phy_csroffs(phy_config), val);
	udelay(3);

	return 0;
}

int etile_ehip_deassert_reset(intel_fpga_xtile_eth_private *priv) {

	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;
	u32 val;

	val = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan,
			eth_phy_csroffs(phy_config));

	if (val & ETH_PHY_CONF_SOFT_TXP_RESET)
		val &= ~ETH_PHY_CONF_SOFT_TXP_RESET;
	if (val & ETH_PHY_CONF_SOFT_RXP_RESET)
		val &= ~ETH_PHY_CONF_SOFT_RXP_RESET;
	if (val & ETH_PHY_CONF_IO_SYS_RESET)
		val &= ~ETH_PHY_CONF_IO_SYS_RESET;

	hssi_csrwr32(pdev, HSSI_ETH_RECONFIG,chan,
			eth_phy_csroffs(phy_config), val);

	udelay(3);

	return 0;
}


int etile_init_mac(intel_fpga_xtile_eth_private *priv)
{
        int ret;

        /* Enable in E-tile Tx datapath */
        etile_set_mac(priv, true);
        etile_update_mac_addr(priv);

        ret = eth_etile_tx_rx_user_flow(priv);
        if (ret < 0) {
                netdev_err(priv->dev, "Tx & Rx user flow failed\n");
                return ret;
        }

        etile_set_mac_flow_ctrl(priv);

        return 0;
}
