// SPDX-License-Identifier: GPL-2.0
/* Intel FPGA E-tile Ethernet MAC driver
 * Copyright (C) 2022 Intel Corporation. All rights reserved
 *
 * Contributors:
 *   Preetam Narayan
 *
 * Original driver contributed by GlobalLogic.
 */

#include <linux/phylink.h>
#include "intel_fpga_eth_ftile.h"
#include "intel_fpga_eth_hssi_itf.h"
#include <linux/interrupt.h>

#define FTILE_EHIP_RESET_TO		10000 /* in us */
#define FTILE_EHIP_RESET_POLL_INTERVAL	5 /* in us */

static int ftile_wait_reset_ack(struct platform_device *pdev, u32 chan,
				u32 rst_ack_mask, u32 maskval)
{
	unsigned long timeout, start;
	u32 val;

	start = jiffies;
	timeout = start + usecs_to_jiffies(FW_ACK_POLL_TIMEOUT_US);
	do {
		udelay(FTILE_EHIP_RESET_POLL_INTERVAL);
		val = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				      eth_soft_csroffs(eth_reset_status));

		if ((val & rst_ack_mask) == maskval)
			return 0;

	} while (time_before(jiffies, timeout));

	return -ETIME;
}

int ftile_ehip_reset(intel_fpga_xtile_eth_private *priv,
		     bool tx, bool rx, bool sys)
{
	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;
	u32 rst_ack_mask = ETH_SOFT_TX_RST | ETH_SOFT_RX_RST | ETH_EIO_SYS_RST;
	u32 maskval = rst_ack_mask;
	u32 val;

	val = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
			      eth_soft_csroffs(eth_reset));
	/* Trigger RX reset
	 * 1.   EHIP CSR Write, Offset = 0x310, value = 0x4
	 * Trigger TX reset
	 * 1.   EHIP CSR Write, Offset = 0x310, value = 0x2
	 * Trigger sys reset
	 * 1.   EHIP CSR Write, Offset = 0x310, value = 0x1
	 */
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

	hssi_csrwr32_ba(pdev, HSSI_ETH_RECONFIG, chan,
			eth_soft_csroffs(eth_reset), val);

	return ftile_wait_reset_ack(pdev, chan, rst_ack_mask, maskval);
}

int ftile_ehip_deassert_reset(intel_fpga_xtile_eth_private *priv)
{
	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;
	u32 rst_ack_mask = ETH_SOFT_TX_RST | ETH_SOFT_RX_RST | ETH_EIO_SYS_RST;
	u32 maskval = 0;
	u32 val;

	val = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
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

	hssi_csrwr32_ba(pdev, HSSI_ETH_RECONFIG, chan,
			eth_soft_csroffs(eth_reset), val);

	return ftile_wait_reset_ack(pdev, chan, rst_ack_mask, maskval);
}

void ftile_enable_mac(intel_fpga_xtile_eth_private *priv)
{
	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;

	/* Enable Tx MAC datapath */
	hssi_clear_bit(pdev, HSSI_ETH_RECONFIG, chan,
		       eth_mac_ptp_csroffs(0, tx_mac_conf),
		       ETH_TX_MAC_DISABLE_TXMAC,
		       true);

	hssi_clear_bit(pdev, HSSI_ETH_RECONFIG, chan,
		       eth_mac_ptp_csroffs(0, rx_mac_frwd_rx_crc),
		       ETH_RX_MAC_CRC_FORWARD,
		       true);
}

void ftile_disable_mac(intel_fpga_xtile_eth_private *priv)
{
	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;

	/* Disable Tx MAC datapath */
	hssi_set_bit(pdev, HSSI_ETH_RECONFIG, chan,
		     eth_mac_ptp_csroffs(0, tx_mac_conf),
		     ETH_TX_MAC_DISABLE_TXMAC,
		     true);

	hssi_clear_bit(pdev, HSSI_ETH_RECONFIG, chan,
		       eth_mac_ptp_csroffs(0, tx_mac_conf),
		       ETH_TX_MAC_ENABLE_S_ADDR_EN,
		       true);
}

void ftile_update_mac_addr(intel_fpga_xtile_eth_private *priv)
{
	u32 msb;
	u32 lsb;
	u32 chan = priv->tile_chan;
	struct platform_device *pdev = priv->pdev_hssi;
	u8 *addr = priv->dev->dev_addr;

	lsb = (addr[2] << 24) | (addr[3] << 16) | (addr[4] << 8) | addr[5];
	msb = ((addr[0] << 8) | addr[1]) & 0xffff;

	/* Set MAC address */
	hssi_csrwr32_ba(pdev, HSSI_ETH_RECONFIG, chan,
			eth_mac_ptp_csroffs(0, tx_mac_source_addr_lower_bytes), lsb);
	hssi_csrwr32_ba(pdev, HSSI_ETH_RECONFIG, chan,
			eth_mac_ptp_csroffs(0, tx_mac_source_addr_higher_bytes), msb);

	/* Enable Source address insertion */
	hssi_set_bit(pdev, HSSI_ETH_RECONFIG, chan,
		     eth_mac_ptp_csroffs(0, tx_mac_conf), ETH_TX_MAC_ENABLE_S_ADDR_EN, true);

	netdev_info(priv->dev, "Device MAC address %pM\n", priv->dev->dev_addr);
}

static void ftile_enable_mac_flow_ctrl(intel_fpga_xtile_eth_private *priv)
{
	u32 reg;
	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;

	/* Rx MAC flow control */
	if ((priv->flow_ctrl & FLOW_RX)) {
		hssi_set_bit(pdev, HSSI_ETH_RECONFIG, chan,
			     eth_mac_ptp_csroffs(0, rx_flow_control_feature_cfg),
			     ETH_RX_EN_STD_FLOW_CTRL, true);

		reg = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				      eth_mac_ptp_csroffs(0, rx_flow_control_feature_cfg));

		if (netif_msg_ifup(priv))
			netdev_info(priv->dev, "F-tile rx_flow_ctrl: 0x%08x\n", reg);
	}

	/* Tx MAC flow control */
	if ((priv->flow_ctrl & FLOW_TX)) {
		hssi_set_bit(pdev, HSSI_ETH_RECONFIG, chan,
			     eth_mac_ptp_csroffs(0, tx_flow_control_feature_cfg),
			     ETH_TX_EN_PRIORITY_FLOW_CTRL, true);

		reg = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				      eth_mac_ptp_csroffs(0, tx_flow_control_feature_cfg));

		if (netif_msg_ifup(priv))
			netdev_info(priv->dev, "F-tile tx_flow_ctrl: 0x%08x\n", reg);
	}

	/* Set pfc pause quanta */
	if (priv->flow_ctrl & FLOW_TX) {
		hssi_csrwr32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				eth_mac_ptp_csroffs(0, pause_quanta_0), priv->pause);

		reg = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				      eth_mac_ptp_csroffs(0, pause_quanta_0));

		if (netif_msg_ifup(priv))
			netdev_info(priv->dev, "F-tile: pause_quanta0: 0x%08x\n", reg);
	}
}

static void ftile_disable_mac_flow_ctrl(intel_fpga_xtile_eth_private *priv)
{
	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;

	/* Disable Rx MAC flow control */
	if ((priv->flow_ctrl & FLOW_RX)) {
		hssi_clear_bit(pdev, HSSI_ETH_RECONFIG, chan,
			       eth_mac_ptp_csroffs(0, rx_flow_control_feature_cfg),
			       ETH_RX_EN_STD_FLOW_CTRL, true);
	}

	/* Disable Tx MAC flow control */
	if ((priv->flow_ctrl & FLOW_TX)) {
		hssi_clear_bit(pdev, HSSI_ETH_RECONFIG, chan,
			       eth_mac_ptp_csroffs(0, tx_flow_control_feature_cfg),
			       ETH_TX_EN_PRIORITY_FLOW_CTRL, true);
	}
}

static u32 get_gb_66_110_occupancy(const u32 speed, const u32 rvld_lsb, const u32 rvld_msb)
{
	const u8 occ     = (rvld_msb & ETH_PHY_PTP_MSB_GB66TO110_OCC_MASK) >>
			   ETH_PHY_PTP_MSB_GB66TO110_OCC_SHIFT;
	const u8 occ_2_1 = occ >> 1;
	const u8 occ_2   = occ >> 2;
	const u8 gbs     = (rvld_lsb & ETH_PHY_PTP_LSB_GBSTATE_MASK) >>
			    ETH_PHY_PTP_LSB_GBSTATE_SHIFT;

	if (speed == SPEED_50000) {
		if (occ_2_1 == 0) {
			return 0;
		} else if (occ_2_1 == 1) {
			return 132;
		} else if (occ_2_1 == 2) {
			return 66;
		} else if (occ_2_1 == 3) {
			pr_alert("%s Illegal case for GB 66_110\n", __func__);
			return 0;
		}
	} else if (speed == SPEED_100000) {
		if (gbs == 0) {
			if (occ_2 == 1)
				return 110;
			else
				return 0;
		} else if (gbs == 1) {
			return 66;
		} else if (gbs == 2) {
			if (occ_2 == 1)
				return 22 + 110;
			else
				return 22;
		} else if (gbs == 3) {
			return 88;
		} else if (gbs == 4) {
			if (occ_2 == 1)
				return 44 + 110;
			else
				return 44;
		}
	}
	return 0;
}

static u32 get_am_detect_occupancy(const u32 speed, const u32 rvld_lsb, const u32 rvld_msb)
{
	const u8 occ   = (rvld_msb & ETH_PHY_PTP_MSB_AM_DETECT_OCC_MASK) >>
			  ETH_PHY_PTP_MSB_AM_DETECT_OCC_SHIFT;
	const u8 occ_0 = occ & 1;

	if (speed == SPEED_50000) {
		if (occ_0 == 0)
			return 0;
		else if (occ_0 == 1)
			return 66;
	} else if (speed == SPEED_100000) {
		if (occ == 0)
			return 44;
		else if (occ == 1)
			return 66;
		else if (occ == 2)
			return 22;
		else if (occ == 3)
			return 44;
		else if (occ == 4)
			return 0;
		else if (occ == 5)
			return 22;
	}
	return 0;
}

static u32 get_blk_align_occupancy(const u32 speed, const u32 rvld_lsb, const u32 rvld_msb)
{
	const u8 blk_align = (rvld_msb & ETH_PHY_PTP_MSB_BLK_ALIGN_OCC_MASK) >>
			      ETH_PHY_PTP_MSB_BLK_ALIGN_OCC_SHIFT;
	const u8 al_pos    = (rvld_msb & ETH_PHY_PTP_MSB_AL_POS_50G_MASK) >>
			      ETH_PHY_PTP_MSB_AL_POS_50G_SHIFT;
	const u8 occ_1_0   = blk_align & 3;

	if (speed == SPEED_50000) {
		if (occ_1_0 == 0)
			return 65 - al_pos;
		else if (occ_1_0 == 1)
			return 66 + (65 - al_pos);
		else if (occ_1_0 == 2)
			return 66 + (65 - al_pos);
		else if (occ_1_0 == 3)
			return 66 + (65 - al_pos) + 66;
	} else if (speed == SPEED_100000) {
		if (occ_1_0 == 0)
			return 21 - al_pos;
		else if (occ_1_0 == 1)
			return 22 + (21 - al_pos);
		else if (occ_1_0 == 2)
			return 22 + (21 - al_pos);
		else if (occ_1_0 == 3)
			return 22 + (21 - al_pos) + 22;
	}
	return 0;
}

static u32 get_gb_33_66_occupancy(const u32 speed, const u32 rvld_lsb, const u32 rvld_msb)
{
	const u8 occ = (rvld_msb & ETH_PHY_PTP_MSB_GB33TO66_OCC_MASK) >>
			ETH_PHY_PTP_MSB_GB33TO66_OCC_SHIFT;

	if (occ > 1) /* nfor values 0 & 1 return 0 , for 2&3 return 33 */
		return 33;

	return 0;
}

// PTP Tx and Rx user flow
static int eth_ftile_tx_rx_user_flow(intel_fpga_xtile_eth_private *priv)
{
	const u8 xcvr_top_lane = 15;
	u32 regval, speed, n;
	u32 tx_const_delay;
	u8  tx_const_delay_sign, num_pl, num_vl, num_fl, pl_fl_map, fl, vl, apl;
	u32 tx_apulse_offset[8];
	u8  tx_apulse_offset_sign[8];
	u32 tx_apulse_wdelay[8];
	u32 tx_apulse_time[8], tx_apulse_time_max = 0;
	u32 tx_am_actual_time[8], tx_am_actual_time_max = 0;
	u8  tx_ref_pl = 0, pl;
	u8  xcvr_sel = 0 /* hw_xcvr_sel */, init_pl;
	u32 tx_extra_latency;
	u32 tx_vl_offset[20] = { 0 };
	u32 tx_tam_adjust, tx_tam_adjust_sim;
	u32 tx_pma_delay_ns;
	u32 rx_const_delay;
	u32 rx_const_delay_sign;
	u32 rx_apulse_offset[8];
	u8  rx_apulse_offset_sign[8];
	u32 rx_apulse_wdelay[8];
	u32 rx_apulse_time[8], rx_apulse_time_max = 0;
	u8  rx_bitslip_cnt, rx_dlpulse_alignment;
	u8  rx_ref_pl, rx_ref_fl, rx_ref_vl;
	u32 rx_spulse_offset[20];
	u8  rx_spulse_offset_sign[20];
	u32 rvld_lsb[20], rvld_msb[20], local_vl, final_offs;
	u32 rx_spulse_post_am[20];
	u32 am_interval;
	u32 rx_am_actual_time[20], rx_am_actual_time_max;
	u32 rx_tam_adjust, rx_tam_adjust_sim;
	u32 tx_pma_delay_ui, rx_pma_delay_ui;
	u32 rx_pma_delay_ns;
	u32 rx_extra_latency;
	u32 rx_vl_offset[20];
	u32 rx_fec_cw_pos[16];
	u32 rx_xcvr_if_pulse_adj[16];
	u32 ui_value;
	u32 tx_routing_adj, rx_routing_adj;
	u8 tx_routing_adj_sign, rx_routing_adj_sign;

	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;
	u16 pma_type = priv->pma_type;
	u8 eth_rate = priv->eth_rate;

	speed = priv->link_speed;

	// TBD add other PHY modes
	switch (priv->phy_iface) {
	case PHY_INTERFACE_MODE_10GKR:
	case PHY_INTERFACE_MODE_10GBASER:
		ui_value = INTEL_FPGA_FTILE_UI_VALUE_10G;
		if ((pma_type) == XCVR_PMA_TYPE_FGT) {
			tx_pma_delay_ui = INTEL_FPGA_TX_PMA_DELAY_25G_UX;
			rx_pma_delay_ui = INTEL_FPGA_RX_PMA_DELAY_25G_UX;
		} else { // XCVR_PMA_TYPE_FHT
			tx_pma_delay_ui = INTEL_FPGA_TX_PMA_DELAY_25G_BK;
			rx_pma_delay_ui = INTEL_FPGA_RX_PMA_DELAY_25G_BK;
		}
		break;
	case PHY_INTERFACE_MODE_25GKR:
	case PHY_INTERFACE_MODE_25GBASER:
		ui_value = INTEL_FPGA_FTILE_UI_VALUE_25G;
		if ((pma_type) == XCVR_PMA_TYPE_FGT) {
			tx_pma_delay_ui = INTEL_FPGA_TX_PMA_DELAY_25G_UX;
			rx_pma_delay_ui = INTEL_FPGA_RX_PMA_DELAY_25G_UX;
		} else { // XCVR_PMA_TYPE_FHT
			tx_pma_delay_ui = INTEL_FPGA_TX_PMA_DELAY_25G_BK;
			rx_pma_delay_ui = INTEL_FPGA_RX_PMA_DELAY_25G_BK;
		}
		break;
	default:
		netdev_err(priv->dev, "Unsupported PHY mode: %s\n", phy_modes(priv->phy_iface));
		return -ENODEV;
	}

	num_fl = speed / SPEED_25000; // FL
	// VL value:
	if (speed <= SPEED_25000) {
		num_vl = 1;
		num_fl = 1;
	} else if (speed == SPEED_50000) {
		num_vl = 4;
	} else if (speed == SPEED_100000) {
		num_vl = 20;
	} else if (speed == SPEED_200000) {
		num_vl = 8;
	} else if (speed == SPEED_400000) {
		num_vl = 16;
	} else {
		netdev_err(priv->dev, "Unsupported speed: %u\n", speed);
		return -ENODEV;
	}
	num_pl = priv->pma_lanes_used;      // PL
	dev_info(priv->device, "DBG: %s speed=%u num_vl=%u num_fl=%u num_pl=%u\n", __func__, speed,
		 num_vl, num_fl, num_pl);

	/* TX User Flow */
	/* Step 1 After power up or reset, wait until TX raw offset data are ready */
	if (xtile_check_counter_complete(priv, HSSI_ETH_RECONFIG, eth_soft_csroffs(ptp_status),
					 ETH_TX_PTP_OFFSET_DATA_VALID, true,
					 INTEL_FPGA_WORD_ALIGN)) {
		netdev_err(priv->dev, "MAC Tx datapath not ready\n");
		return -EINVAL;
	}

	/* Step 2 Read TX raw offset data from IP */
	regval = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				 eth_soft_csroffs(ptp_tx_lane_calc_data_constdelay));
	tx_const_delay      = regval & 0x7FFFFFFF;
	tx_const_delay_sign = regval >> 31;
	n = eth_soft_csroffs(ptp_tx_lane1_calc_data_offset)
	    - eth_soft_csroffs(ptp_tx_lane0_calc_data_offset);
	for (pl = 0; pl < num_pl; pl++) {
		regval = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
					 eth_soft_csroffs(ptp_tx_lane0_calc_data_offset) + n * pl);
		tx_apulse_offset[pl]      = regval & 0x7FFFFFFF;
		tx_apulse_offset_sign[pl] = regval >> 31;
		tx_apulse_wdelay[pl]      = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
							    eth_soft_csroffs
							    (ptp_tx_lane0_calc_data_wiredelay)
							    + n * pl) & 0xFFFFF;
		tx_apulse_time[pl]        = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
							    eth_soft_csroffs
							    (ptp_tx_lane0_calc_data_time)
							    + n * pl) & 0x0FFFFFFF;
		if (tx_apulse_time[pl] > tx_apulse_time_max)
			tx_apulse_time_max = tx_apulse_time[pl];
	}

	/* Step 3 Determine TX reference lane */
	if (num_pl > 1) {
		/* Step 3a Detect rollover of async pulse time */
		for (pl = 0; pl < num_pl; pl++) {
			if (tx_apulse_time_max - tx_apulse_time[pl] > 0x01F40000) { // 500 ns
				/* if diff is above 500 ns,
				 * we assume rollover (normally diff is below 10 ns)
				 */
				if ((tx_apulse_time_max >> 24) == 0xF)
					// natural rollover bit 27 to 28, so add bit 28
					tx_apulse_time[pl] += 0x10000000;
				else
					/* TOD 1 billion rollover 48'3B9A_CA00_0000,
					 * so add 0xA000000
					 */
					tx_apulse_time[pl] += 0x0A000000;
			}
		}
		/* Step 3b Calculate actual time of TX Alignment Marker at
		 * TX PMA parallel data interface
		 */
		for (pl = 0; pl < num_pl; pl++) {
			tx_am_actual_time[pl] = (tx_apulse_time[pl] +
						 (tx_apulse_offset_sign[pl] ?
						 -tx_apulse_offset[pl] : tx_apulse_offset[pl])
						 - tx_apulse_wdelay[pl]);
			/* Step 3c Determine TX reference lane */
			/* TX reference lane is the TX physical lane where
			 * tx_am_actual_time is the largest for all physical lanes.
			 */
			if (tx_am_actual_time[pl] > tx_am_actual_time_max) {
				tx_am_actual_time_max = tx_am_actual_time[pl];
				tx_ref_pl = pl;
			}
		}
	}

	/* Step 4 Calculate TX offsets */
	/* Step 4a Calculate TX TAM adjust */
	tx_tam_adjust_sim = ((tx_const_delay_sign ? -tx_const_delay : tx_const_delay) +
			     (tx_apulse_offset_sign[tx_ref_pl] ?
			     -tx_apulse_offset[tx_ref_pl] : tx_apulse_offset[tx_ref_pl]) -
			     tx_apulse_wdelay[tx_ref_pl]);

	// tx_tam_adjust is a 32-bit two's complement number.
	tx_tam_adjust = tx_tam_adjust_sim;

	/* hardware run with advanced accuracy mode:
	 * Note: See Section 5.3.4 on how to obtain tx_routing_adj_sign and
	 * tx_routing_adj information. (routing delays for specific image,
	 * generated via steps outlined in Section 5.3.4 of "Towards a F-Tile.." doc.)
	 * Others:
	 */
	if (strcasecmp(priv->ptp_accu_mode, "Advanced") == 0) {
		tx_routing_adj = priv->ptp_tx_routing_adj;
		tx_routing_adj_sign = tx_routing_adj >> 31;
		tx_tam_adjust = tx_tam_adjust_sim +
				(tx_routing_adj_sign ? -tx_routing_adj : tx_routing_adj);
	}

	/* Step 4b Calculate TX extra latency */
	/* Convert unit of TX PMA delay from UI to nanoseconds
	 * Note: Format of UI is {4-bit ns, 28-bit frac ns},
	 * while other variables are {N-bit ns, 16-bit frac ns},
	 * where N is the largest number to store max value from the calculation.
	 * Result of the multiplication involving UI must be converted to
	 * 16-bit frac ns format.
	 */
	tx_pma_delay_ns = ((u64)tx_pma_delay_ui * ui_value) >> 12;
	// Total up all extra latency together
	tx_extra_latency = tx_pma_delay_ns + priv->tx_external_phy_delay_ns;
	/* TX extra latency is a positive adjustment,
	 * set most-significant bit of the register to 0 to indicate positive.
	 */
	tx_extra_latency &= 0x7FFFFFFF;

	/* Step 4c - must be skipped for 10G/25G */
	if (speed > SPEED_25000) {
		// Calculate TX virtual lane offsets
		/* Using VL0 as reference virtual lane,
		 * assign TX virtual lane offset values according
		 * to virtual lane order as described in section 5.2.3.
		 * Note: Format of UI is {4-bit ns, 28-bit frac ns},
		 * while other variables are {N-bit ns, 16-bit frac ns},
		 * where N is the largest number to store max value from the calculation.
		 * Result of the multiplication
		 * involving UI must be converted to 16-bit frac ns format.
		 */
		if (!strcasecmp(priv->fec_type, "kp-fec") ||
		    !strcasecmp(priv->fec_type, "ll-fec")) {
			// KP-FEC/LL-FEC variants:
			for (vl = 0; vl < num_vl; vl++)
				tx_vl_offset[vl] = ((u64)(vl -
						    (vl % num_pl)) / num_pl * 68 * ui_value) >>
						    12;
		} else if (!strcasecmp(priv->fec_type, "kr-fec")) {
			// KR-FEC variants:
			for (vl = 0; vl < num_vl; vl++)
				tx_vl_offset[vl] = ((u64)(vl -
						    (vl % num_pl)) / num_pl * 66 * ui_value) >>
						    12;
		} else if (!strcasecmp(priv->fec_type, "no-fec")) {
			// No FEC variants:
			for (vl = 0; vl < num_vl; vl++)
				tx_vl_offset[vl] = ((u64)(vl -
						    (vl % num_pl)) / num_pl * 1 * ui_value) >>
						    12;
		}
	}

	/* Step 5 Write the determined TX reference lane into IP */
	regval = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_ref_lane));
	regval = (regval & ~ETH_PTP_TX_REF_LANE_MASK) | (tx_ref_pl << ETH_PTP_TX_REF_LANE_SHIFT);
	hssi_csrwr32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_soft_csroffs(ptp_ref_lane), regval);

	/* Step 6 Write the calculated TX offsets to IP */

	/* Step 6a Write TX virtual lane offsets - must be skipped for 10G/25G */
	if (speed > SPEED_25000) {
		for (vl = 0; vl < num_vl; vl++) {
			hssi_csrwr32_ba(pdev, HSSI_ETH_RECONFIG, chan,
					eth_mac_ptp_csroffs(eth_rate, tx_ptp_vl_offset[vl]),
					tx_vl_offset[vl]);
		}
	}

	/* Step 6b Write TX extra latency */
	hssi_csrwr32_ba(pdev, HSSI_ETH_RECONFIG, chan,
			eth_mac_ptp_csroffs(eth_rate, tx_ptp_extra_latency), tx_extra_latency);

	/* Step 6c Write TX TAM adjust */
	hssi_csrwr32_ba(pdev, HSSI_ETH_RECONFIG, chan,
			eth_soft_csroffs(ptp_tx_tam_adjust), tx_tam_adjust);

	/* Step 7 UI value measurement */
	// Perform step 1 to 7 as specified in Table 90.
	/* Note: For simulation or hardware run with 0ppm setup,
	 * user is advised to skip measurement and program 0ppm UI value as stated in Table 89.
	 */
	/* Program 0ppm UI value here and
	 * later start a timer to do UI value measurements / adjustments
	 */
	hssi_csrwr32_ba(pdev, HSSI_ETH_RECONFIG, chan,
			eth_mac_ptp_csroffs(eth_rate, tx_ptp_ui), ui_value);

	/* Step 8 Notify soft PTP that user flow configuration is completed */
	regval = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				 eth_soft_csroffs(ptp_tx_user_cfg_status));
	regval |= ETH_PTP_TX_USER_CFG_DONE;
	hssi_csrwr32_ba(pdev, HSSI_ETH_RECONFIG, chan,
			eth_soft_csroffs(ptp_tx_user_cfg_status), regval);

	/* Step 9 Wait until TX PTP is ready */
	if (xtile_check_counter_complete(priv, HSSI_ETH_RECONFIG,
					 eth_soft_csroffs(ptp_status),
					 ETH_TX_PTP_READY, true,
					 INTEL_FPGA_WORD_ALIGN)) {
		netdev_err(priv->dev, "MAC Tx PTP not ready\n");
		return -EINVAL;
	}
	dev_info(priv->device,
		 "DBG: %s ETH_TX_PTP_READY - tx_ref_pl:%u tx_extra_latency:0x%08x tx_tam_adjust:%i\n",
		 __func__, tx_ref_pl, tx_extra_latency, (int32_t)tx_tam_adjust);

	/* Step 10 TX PTP is up and running */
	/* Step 10a Adjust TX UI value */
	/* User should perform TX UI adjustment of hard PTP IP from time to time to
	 * prevent time counter drift from golden time-of-day in the system.
	 * Done via timer_setup at end of this function
	 */

	/* RX User Flow */
	// Check all FEC lanes are locked
	if (strcasecmp(priv->fec_type, "no-fec") != 0) {
		for (fl = 0; fl < num_fl; fl++) {
			if (xtile_check_counter_complete(priv, HSSI_ETH_RECONFIG,
							 eth_rsfec_csroffs(eth_rate, fl,
									   rsfec_lane_rx_stat),
							 RSFEC_RX_STATUS_LANE_NOT_LOCKED, false,
							 INTEL_FPGA_WORD_ALIGN)) {
				netdev_err(priv->dev,
					   "MAC Rx datapath not ready &RS FEC lane is not locked\n");
				return -EINVAL;
			}
		}
	}

	/* Step 1 After power up, reset or link down, wait until RX PCS is fully aligned */
	// 10G-100G No FEC variants, 25G FEC variants:
	if (!strcasecmp(priv->fec_type, "no-fec") || speed == SPEED_25000) {
		if (xtile_check_counter_complete(priv, HSSI_ETH_RECONFIG,
						 eth_phy_csroffs(eth_rate, phy_pcs_stat_anlt),
						 ETH_PHY_RX_PCS_ALIGNED, true,
						 INTEL_FPGA_WORD_ALIGN)) {
			netdev_err(priv->dev, "MAC Rx datapath not ready (PHY_RX_PCS_ALIGNED=0)\n");
			return -EINVAL;
		}
	} else {
		// 50G-400G FEC variants:
		for (fl = 0; fl < num_fl; fl++)
			if (xtile_check_counter_complete(priv, HSSI_ETH_RECONFIG,
							 eth_rsfec_csroffs(eth_rate, fl,
									   rsfec_aggr_rx_stat),
							 RSFEC_AGGR_RX_STATUS_NOT_ALIGN, false,
							 INTEL_FPGA_WORD_ALIGN)) {
				netdev_err(priv->dev,
					   "MAC Rx datapath not ready(RSFEC_AGGR_RX_STATUS_NOT_ALIGN, fl=%u)\n",
					    fl);
				return -EINVAL;
			}
	}

	// init_pl value obtained like this as per tcl scripts:
	regval = hssi_csrrd32_ba(pdev, HSSI_PHY_XCVR_PMACAP, chan,
				 eth_pma_avmm_csroffs(u.fht_cur_lane, 0));
	init_pl = xcvr_top_lane - regval;

	/* Step 2 [FEC variant only] */
	if (strcasecmp(priv->fec_type, "no-fec") != 0) {
		// Configure RX FEC codeword position into transceiver
		// Step 2a Read RX FEC codeword position and FEC lane mapping for each PMA lane
		// Determine mapping from PMA lane to FEC lane:
		pl_fl_map = num_fl / num_pl;

		for (fl = 0; fl < num_fl; fl++)
			rx_fec_cw_pos[fl] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
							    eth_rsfec_csroffs(eth_rate, fl,
									      rsfec_cw_pos_rx)) &
							    RSFEC_CW_POS_RX_num;

		// Step 2b Calculate pulse adjustments
		for (fl = 0; fl < num_fl; fl++)
			rx_xcvr_if_pulse_adj[fl] = rx_fec_cw_pos[fl];
		// Step 2c Write the pulse adjustments into IP
		/* For multiple FEC lanes that interleave within single transceiver,
		 * we select adjustment value from FEC lane with lowest index
		 */
		if ((pma_type) == XCVR_PMA_TYPE_FGT) {
			/* FGT transceiver:
			 *  Note: There are 4 FGT quads with 4 apl lanes each.
			 * User must ensure register of all active quad lanes are programmed.
			 * Please refer to AVMM2 or global AVMM User Guide on
			 * how to access different FGT quad.
			 */
			n = eth_pma_avmm_csroffs(fgt_q_dl_ctrl_a_l1, 0) -
			    eth_pma_avmm_csroffs(fgt_q_dl_ctrl_a_l0, 0);
			for (pl = 0; pl < num_pl; pl++) {
				apl = (3 - ((pl + init_pl) % 4));
				regval = hssi_csrrd32_ba(pdev, HSSI_PHY_XCVR_PMACAP, chan,
							 eth_pma_avmm_csroffs(fgt_q_dl_ctrl_a_l0,
									      xcvr_sel)
							 + apl * n);
				regval &= ~XCVR_FGT_Q_DL_CTRL_RX_LAT_CNTRVAL_ASYNC;
				regval |= rx_xcvr_if_pulse_adj[pl * pl_fl_map] &
					  XCVR_FGT_Q_DL_CTRL_RX_LAT_CNTRVAL_ASYNC;
				hssi_csrwr32_ba(pdev, HSSI_PHY_XCVR_PMACAP, chan,
						eth_pma_avmm_csroffs(fgt_q_dl_ctrl_a_l0,
								     xcvr_sel) +
						apl * n, regval);
				/* tcl script has this xcvr_sel 8,9,A,B ==> quad3,
				 * xcvr_sel C,D,E,F ==> quad2, etc
				 */
				xcvr_sel++;
			}
		} else {
			// FHT transceiver:
			n = 4;
			for (pl = 0; pl < num_pl; pl++) {
				apl = (3 - ((pl + init_pl) % 4));
				regval = hssi_csrrd32_ba(pdev, HSSI_PHY_XCVR_PMACAP, chan,
							 eth_pma_avmm_csroffs(fgt_q_dl_ctrl_a_l2,
									      xcvr_sel) +
							 apl * n);
				regval &= ~XCVR_FHT_Q_DL_CTRL_RX_LAT_CNTRVAL_ASYNC;
				regval |= rx_xcvr_if_pulse_adj[pl * pl_fl_map] &
					  XCVR_FHT_Q_DL_CTRL_RX_LAT_CNTRVAL_ASYNC;
				hssi_csrwr32_ba(pdev, HSSI_PHY_XCVR_PMACAP, chan,
						eth_pma_avmm_csroffs(fgt_q_dl_ctrl_a_l2,
								     xcvr_sel) +
						apl * n, regval);
			}
			/* Note: Both apl and pl are used in this step,
			 * see Table 84 for clarification.
			 */
		}

		// Step 2d Notify soft PTP that pulse adjustments have been configured
		regval = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
					 eth_soft_csroffs(ptp_rx_user_cfg_status));
		regval |= ETH_PTP_RX_FEC_CW_POS_CFG_DONE;
		hssi_csrwr32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				eth_soft_csroffs(ptp_rx_user_cfg_status), regval);
	}

	/* Step 3 Wait until RX raw offset data are ready */
	if (xtile_check_counter_complete(priv, HSSI_ETH_RECONFIG,
					 eth_soft_csroffs(ptp_status),
					 ETH_RX_PTP_OFFSET_DATA_VALID, true,
					 INTEL_FPGA_WORD_ALIGN)) {
		netdev_err(priv->dev, "MAC Rx datapath not ready (RX_PTP_OFFSET_DATA_VALID=0)\n");
		return -EINVAL;
	}

	/* Step 4 Read RX raw offset data from IP */
	regval = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				 eth_soft_csroffs(ptp_rx_lane_calc_data_constdelay));
	rx_const_delay      = regval & 0x7FFFFFFF;
	rx_const_delay_sign = regval >> 31;
	n = eth_soft_csroffs(ptp_rx_lane1_calc_data_offset) -
	    eth_soft_csroffs(ptp_rx_lane0_calc_data_offset);
	for (pl = 0; pl < num_pl; pl++) {
		regval = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
					 eth_soft_csroffs(ptp_rx_lane0_calc_data_offset) + n * pl);
		rx_apulse_offset[pl]      = regval & 0x7FFFFFFF;
		rx_apulse_offset_sign[pl] = regval >> 31;
		rx_apulse_wdelay[pl]      = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
							    eth_soft_csroffs
							    (ptp_rx_lane0_calc_data_wiredelay)
							    + n * pl) & 0xFFFFF;
		rx_apulse_time[pl]        = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
							    eth_soft_csroffs
							    (ptp_rx_lane0_calc_data_time)
							    + n * pl) & 0x0FFFFFFF;
		if (rx_apulse_time[pl] > rx_apulse_time_max)
			rx_apulse_time_max = rx_apulse_time[pl];
	}
	if (speed <= SPEED_25000 && strcasecmp(priv->fec_type, "no-fec") == 0) {
		// 10G/25G No FEC variants:
		regval = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
					 eth_phy_csroffs(eth_rate, phy_rx_bitslip_cnt));
		rx_bitslip_cnt       = regval & ETH_PHY_RX_PCS_BITSLIP_CNT;
		rx_dlpulse_alignment = (regval & ETH_PHY_RX_PCS_DLPULSE_ALIGNED) ? 33 : 0;
	}

	/* Step 5 Determine RX reference lane */
	/* Step 5a Determine sync pulse (Alignment Marker) offsets with reference to async pulse */
	/* Note: Format of UI is {4-bit ns, 28-bit frac ns},
	 * while other variables are {N-bit ns, 16-bit frac ns},
	 * where N is the largest number to store max value from the calculation.
	 * Result of the multiplication
	 * involving UI must be converted to 16-bit frac ns format.
	 */
	if (strcasecmp(priv->fec_type, "no-fec") != 0) {
		// FEC variants:
		for (fl = 0; fl < num_fl; fl++) {
			if ((rx_xcvr_if_pulse_adj[fl] +
			    (rx_xcvr_if_pulse_adj[fl - (fl % pl_fl_map)] & 0x1f)) >
			    rx_xcvr_if_pulse_adj[fl - (fl % pl_fl_map)]) {
				rx_spulse_offset[fl] = ((u64)(rx_xcvr_if_pulse_adj[fl] -
							rx_xcvr_if_pulse_adj[fl -
							(fl % pl_fl_map)] +
							(rx_xcvr_if_pulse_adj[fl -
							(fl % pl_fl_map)] & 0x1f))
							* ui_value * pl_fl_map) >> 12;
				rx_spulse_offset_sign[fl] = 0;
			} else {
				rx_spulse_offset[fl] = ((u64)(rx_xcvr_if_pulse_adj[fl -
							(fl % pl_fl_map)] -
							rx_xcvr_if_pulse_adj[fl] -
							(rx_xcvr_if_pulse_adj[fl -
							(fl % pl_fl_map)] & 0x1f))
							* ui_value * pl_fl_map) >> 12;
				rx_spulse_offset_sign[fl] = 1;
			}
		}
	} else if (speed == SPEED_50000 || speed == SPEED_100000) {
		// 50G/100G No FEC variants:
		if (speed == SPEED_50000) // 50G-2
			am_interval = 32768 * 66;
		else // 100G-4:
			am_interval = 81920 * 66;
		// read vl data and determine final_offs, the bit distance from
		// PCS internal Alignment Marker to sync pulse of each virtual lane:
		/* (see https://www.intel.com/content/www/us/en/docs/programmable
		 *  /683023/22-1/rx-virtual-lane-offset-calculation-for.html)
		 */
		for (vl = 0; vl < num_vl; vl++) {
			rvld_lsb[vl] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
						       eth_phy_csroffs(eth_rate,
								       ptp_vl_data_lsb[vl]));
			rvld_msb[vl] = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
						       eth_phy_csroffs(eth_rate,
								       ptp_vl_data_msb[vl]));
			local_vl = (rvld_lsb[vl] & ETH_PHY_PTP_LSB_LOCAL_VL_MASK) >>
				   ETH_PHY_PTP_LSB_LOCAL_VL_SHIFT;
			if (speed == SPEED_50000) {
				final_offs = (get_gb_33_66_occupancy(speed, rvld_lsb[vl],
								     rvld_msb[vl]) +
					      get_gb_66_110_occupancy(speed, rvld_lsb[vl],
								      rvld_msb[vl]) +
					      2 * get_blk_align_occupancy(speed, rvld_lsb[vl],
									  rvld_msb[vl]) +
					      2 * get_am_detect_occupancy(speed, rvld_lsb[vl],
									  rvld_msb[vl]) +
					      2 * 66 * (((rvld_lsb[vl] &
							ETH_PHY_PTP_LSB_AM_COUNT_MASK) >>
							ETH_PHY_PTP_LSB_AM_COUNT_SHIFT) |
							(((rvld_lsb[vl] &
							 ETH_PHY_PTP_LSB_SPARE_MASK) >>
							 ETH_PHY_PTP_LSB_SPARE_SHIFT) << 2)));
				if (local_vl == 1 || local_vl == 3)
					final_offs -= 1;
			} else { // speed == 100
				final_offs = (get_gb_33_66_occupancy(speed, rvld_lsb[vl],
								     rvld_msb[vl]) +
					      get_gb_66_110_occupancy(speed, rvld_lsb[vl],
								      rvld_msb[vl]) +
					      5 * get_blk_align_occupancy(speed, rvld_lsb[vl],
									  rvld_msb[vl]) +
					      5 * get_am_detect_occupancy(speed, rvld_lsb[vl],
									  rvld_msb[vl]) +
					      5 * 66 * (((rvld_lsb[vl] &
							ETH_PHY_PTP_LSB_AM_COUNT_MASK) >>
							ETH_PHY_PTP_LSB_AM_COUNT_SHIFT) |
							(((rvld_lsb[vl] &
							 ETH_PHY_PTP_LSB_SPARE_MASK) >>
							 ETH_PHY_PTP_LSB_SPARE_SHIFT) << 2)));
				if (local_vl == 1 || local_vl == 6 || local_vl == 11 ||
				    local_vl == 16)
					final_offs -= 1;
				else if (local_vl == 2 || local_vl == 7 || local_vl == 12 ||
					 local_vl == 17)
					final_offs -= 2;
				else if (local_vl == 3 || local_vl == 8 || local_vl == 13 ||
					 local_vl == 18)
					final_offs -= 3;
				else if (local_vl == 4 || local_vl == 9 || local_vl == 14 ||
					 local_vl == 19)
					final_offs -= 4;
				if (final_offs > am_interval)
					final_offs -= am_interval;
			}
			rx_spulse_post_am[local_vl]     = final_offs;
			/* Note: Format of UI is {4-bit ns, 28-bit frac ns},
			 * while other variables are {N-bit ns, 16-bit frac ns},
			 * where N is the largest number to store max value from the calculation.
			 * Result of the multiplication
			 * involving UI must be converted to 16-bit frac ns format.
			 */
			rx_spulse_offset[local_vl]      = ((u64)(am_interval -
							   rx_spulse_post_am[local_vl]) *
							   ui_value) >> 12;
			rx_spulse_offset_sign[local_vl] = 0;
		}
	} else if (speed == SPEED_10000 || speed == SPEED_25000) {
		// 10G/25G No FEC variants:
		/* Note: Format of UI is {4-bit ns, 28-bit frac ns},
		 * while other variables are {N-bit ns, 16-bit frac ns},
		 * where N is the largest number to store max value from the calculation.
		 * Result of the multiplication
		 * involving UI must be converted to 16-bit frac ns format.
		 */
		rx_spulse_offset[0]      = ((u64)(rx_bitslip_cnt + rx_dlpulse_alignment) *
					    ui_value) >> 12;
		rx_spulse_offset_sign[0] = 0;
	} else {
	}

	if (speed == SPEED_10000 || speed == SPEED_25000) {
		// Step 5b, 5c and 5d could be skipped for single lane 10G and 25G:
		rx_ref_pl = 0;
		rx_ref_fl = 0;
		rx_ref_vl = 0;
	} else { // multi PMA lane variants
		/* Step 5b Detect rollover of async pulse time */
		/* rx_apulse_time[pl] represents async pulse time of
		 * each physical lane in 28 bit format, where bit [27:16] is ns
		 * while bit [15:0] is fns. There are 2 types of rollover possibly happening:
		 * 1. Natural rollover from bit 27 to bit 28 when the value is reaching 28'hFFF_FFFF
		 * Bit[27:24] before rollover are all 1s'
		 * 2. Billion rollover when TOD is reaching 1 billion ns or 48'3B9A_CA00_0000
		 * Bit[27:24] before rollover is 4'h9
		 */

		// Given rx_apulse_time_max is largest rx_apulse_time out of all physical lanes
		for (pl = 0; pl < num_pl; pl++) {
			if (rx_apulse_time_max - rx_apulse_time[pl] > 0x01F40000) { // 500 ns
				// if diff is above 500 ns, we assume rollover
				// (normally diff is below 10 ns)
				if ((rx_apulse_time_max >> 24) == 0xF)
					// natural rollover bit 27 to 28, so add bit 28
					rx_apulse_time[pl] += 0x10000000;
				else
					// TOD 1 billion rollover 48'3B9A_CA00_0000,
					// so add 0xA000000
					rx_apulse_time[pl] += 0x0A000000;
			}
		}

		/* Step 5c Calculate actual time of RX Alignment Marker
		 * at RX PMA parallel data interface
		 */
		rx_am_actual_time_max = 0;
		if (strcasecmp(priv->fec_type, "no-fec") != 0) {
			// FEC variants:
			for (fl = 0; fl < num_fl; fl++) {
				pl = (fl - (fl % pl_fl_map)) / pl_fl_map;
				rx_am_actual_time[fl] = ((rx_apulse_time[pl])
							 + (rx_apulse_offset_sign[pl] ?
							 -rx_apulse_offset[pl] :
							 rx_apulse_offset[pl])
							 - (rx_apulse_wdelay[pl])
							 + (rx_spulse_offset_sign[fl] ?
							 -rx_spulse_offset[fl] :
							 rx_spulse_offset[fl]));
				if (rx_am_actual_time[fl] >= rx_am_actual_time_max) {
					rx_am_actual_time_max = rx_am_actual_time[fl];
					rx_ref_fl = fl;
				}
			}
		} else {
			// No FEC variants:
			for (vl = 0; vl < num_vl; vl++) {
				regval = rvld_msb[vl];
				pl = (regval & ETH_PHY_PTP_MSB_LOCAL_PL_MASK) >>
				      ETH_PHY_PTP_MSB_LOCAL_PL_SHIFT;
				rx_am_actual_time[vl] = ((rx_apulse_time[pl])
							 + (rx_apulse_offset_sign[pl] ?
							 -rx_apulse_offset[pl] :
							 rx_apulse_offset[pl])
							 - (rx_apulse_wdelay[pl])
							 + (rx_spulse_offset_sign[vl] ?
							 -rx_spulse_offset[vl] :
							 rx_spulse_offset[vl]));
				if (rx_am_actual_time[vl] >= rx_am_actual_time_max) {
					rx_am_actual_time_max = rx_am_actual_time[vl];
					rx_ref_vl = vl;
				}
			}
		}

		/* Step 5d Determine RX reference lane */
		/* RX reference lane, rx_ref_pl is the RX physical lane associated to
		 * RX FEC/virtual lane
		 * which its rx_am_actual_time is the largest among all FEC/virtual lanes.
		 */
		if (strcasecmp(priv->fec_type, "no-fec") != 0) {
			// FEC variants:
			// rx_am_actual_time_max is maximum value, found for
			// rx_am_actual_time[rx_ref_fl];
			rx_ref_pl = (rx_ref_fl - (rx_ref_fl % pl_fl_map)) / pl_fl_map;
		} else {
			// No FEC variants:
			// rx_am_actual_time_max is maximum value,
			// found for rx_am_actual_time[rx_ref_vl]
			regval = rvld_msb[rx_ref_vl];
			rx_ref_pl = (regval & ETH_PHY_PTP_MSB_LOCAL_PL_MASK) >>
				    ETH_PHY_PTP_MSB_LOCAL_PL_SHIFT;
		}
	}

	/* Step 6 Calculate RX offsets */
	/* Step 6a Calculate RX TAM adjust */
	if (strcasecmp(priv->fec_type, "no-fec") != 0) {
		// FEC variants:
		rx_tam_adjust_sim = ((rx_const_delay_sign ? -rx_const_delay : rx_const_delay)
				     + (rx_apulse_offset_sign[rx_ref_pl] ?
				     -rx_apulse_offset[rx_ref_pl] : rx_apulse_offset[rx_ref_pl])
				     - (rx_apulse_wdelay[rx_ref_pl])
				     + (rx_spulse_offset_sign[rx_ref_fl] ?
				     -rx_spulse_offset[rx_ref_fl] : rx_spulse_offset[rx_ref_fl]));
	} else {
		// No FEC variants:
		rx_tam_adjust_sim = ((rx_const_delay_sign ? -rx_const_delay : rx_const_delay)
				     + (rx_apulse_offset_sign[rx_ref_pl] ?
				     -rx_apulse_offset[rx_ref_pl] : rx_apulse_offset[rx_ref_pl])
				     - (rx_apulse_wdelay[rx_ref_pl])
				     + (rx_spulse_offset_sign[rx_ref_vl] ?
				     -rx_spulse_offset[rx_ref_vl] : rx_spulse_offset[rx_ref_vl]));
	}

	rx_tam_adjust = rx_tam_adjust_sim; // rx_tam_adjust is a 32-bit two's complement number.

	/* Hardware run with advanced accuracy mode:
	 * Note: See Section 10.3.5 on how to obtain rx_routing_adj_sign and
	 * rx_routing_adj information. (routing delays for specific image,
	 * generated via steps outlined in Section 10.3.5 of doc.)
	 * Others:
	 */
	if (strcasecmp(priv->ptp_accu_mode, "Advanced") == 0) {
		rx_routing_adj = priv->ptp_rx_routing_adj;
		rx_routing_adj_sign = rx_routing_adj >> 31;
		rx_tam_adjust = (rx_tam_adjust_sim) + (rx_routing_adj_sign ?
							-rx_routing_adj : rx_routing_adj);
	}

	/* Step 6b Calculate RX extra latency */
	// Convert unit of RX PMA delay from UI to nanoseconds
	/* Note: Format of UI is {4-bit ns, 28-bit frac ns},
	 * while other variables are {N-bit ns, 16-bit frac ns},
	 * where N is the largest number to store max value from the calculation.
	 * Result of the multiplication
	 * involving UI must be converted to 16-bit frac ns format.
	 */
	rx_pma_delay_ns	= ((u64)rx_pma_delay_ui * ui_value) >> 12;
	// Total up all extra latency together
	rx_extra_latency = rx_pma_delay_ns + priv->rx_external_phy_delay_ns;
	// RX extra latency is a negative adjustment,
	// set most-significant bit of the register to 1 to indicate negative.
	rx_extra_latency |= 0x80000000;

	/* Step 6c Calculate RX virtual lane offsets - must be skipped for 10G/25G. */
	if (speed > SPEED_25000) {
		// Using determined reference virtual lane,
		// assign RX virtual lane offset values as described in section 10.2.3.
		/* Note: Format of UI is {4-bit ns, 28-bit frac ns},
		 * while other variables are {N-bit ns, 16-bit frac ns},
		 * where N is the largest number to store max value from the calculation.
		 * Result of the multiplication
		 * involving UI must be converted to 16-bit frac ns format.
		 */
		if (!strcasecmp(priv->fec_type, "kp-fec") ||
		    !strcasecmp(priv->fec_type, "ll-fec")) {
			// KP-FEC/LL-FEC variants:
			for (vl = 0; vl < num_vl; vl++)
				rx_vl_offset[vl] = ((u64)(vl -
						    (vl % num_pl)) / num_pl * 68 *
						    ui_value) >> 12;
		} else if (!strcasecmp(priv->fec_type, "kr-fec")) {
			// KR-FEC variants:
			for (vl = 0; vl < num_vl; vl++)
				rx_vl_offset[vl] = ((u64)(vl -
						    (vl % num_pl)) / num_pl * 66 *
						    ui_value) >> 12;
		} else if (!strcasecmp(priv->fec_type, "no-fec")) {
			// No FEC 100G variants:
			if (speed == SPEED_100000) {
				for (vl = 0; vl < num_vl; vl++)
					rx_vl_offset[vl] = ((u64)2 * ui_value) >> 12;
			} else if (speed == SPEED_50000) {
				// No FEC 50G variants:
				for (vl = 0; vl < num_vl; vl++)
					rx_vl_offset[vl] = ui_value >> 13; // 0.5 * UI
			}
		}
	}

	/* Step 7 Write the determined RX reference lane into IP -
	 * this step must be skipped for 10G/25G.
	 */
	if (speed > SPEED_25000) {
		regval = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
					 eth_soft_csroffs(ptp_ref_lane));
		regval = (regval & ~ETH_PTP_RX_REF_LANE_MASK) |
			 (rx_ref_pl << ETH_PTP_RX_REF_LANE_SHIFT);
		hssi_csrwr32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				eth_soft_csroffs(ptp_ref_lane), regval);
	}

	/* Step 8 Write the calculated RX offsets to IP */
	/* Step 8a Write RX virtual lane offsets - skipped for 10G/25G. */
	if (speed > SPEED_25000) {
		for (vl = 0; vl < num_vl; vl++) {
			hssi_csrwr32_ba(pdev, HSSI_ETH_RECONFIG, chan,
					eth_mac_ptp_csroffs(eth_rate, rx_ptp_vl_offset[vl]),
							    rx_vl_offset[vl]);
		}
	}

	/* Step 8b Write RX extra latency */
	hssi_csrwr32_ba(pdev, HSSI_ETH_RECONFIG, chan,
			eth_mac_ptp_csroffs(eth_rate, rx_ptp_extra_latency), rx_extra_latency);

	/* Step 8c Write RX TAM adjust */
	hssi_csrwr32_ba(pdev, HSSI_ETH_RECONFIG, chan,
			eth_soft_csroffs(ptp_rx_tam_adjust), rx_tam_adjust);

	/* Step 9 Continue UI value measurement */
	// Program 0ppm UI value here and
	// later start a timer to do UI value measurements / adjustments
	hssi_csrwr32_ba(pdev, HSSI_ETH_RECONFIG, chan, eth_mac_ptp_csroffs(eth_rate,
									   rx_ptp_ui),
			ui_value);

	/* Step 10 Notify soft PTP that user flow configuration is completed */
	regval = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
				 eth_soft_csroffs(ptp_rx_user_cfg_status));
	regval |= ETH_PTP_RX_USER_CFG_DONE;
	hssi_csrwr32_ba(pdev, HSSI_ETH_RECONFIG, chan,
			eth_soft_csroffs(ptp_rx_user_cfg_status), regval);

	/* Step 11 Wait until RX PTP is ready */
	if (xtile_check_counter_complete(priv, HSSI_ETH_RECONFIG,
					 eth_soft_csroffs(ptp_status),
					 ETH_RX_PTP_READY, true,
					 INTEL_FPGA_WORD_ALIGN)) {
		netdev_err(priv->dev, "MAC Rx PTP not ready\n");
		return -EINVAL;
	}
	dev_info(priv->device,
		 "DBG: %s ETH_RX_PTP_READY - rx_ref_pl:%u rx_extra_latency:0x%08x rx_tam_adjust:%i\n",
		 __func__, rx_ref_pl, rx_extra_latency, (int32_t)rx_tam_adjust);

	/* Step 12 RX PTP is up and running */

	return 0;
}

void ftile_pma_digital_reset(intel_fpga_xtile_eth_private *priv,
			     bool tx_reset,
			     bool rx_reset)
{
	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;

	/* Trigger RX digital reset
	 * 1.   EHIP CSR Write, Offset = 0x310, value = 0x4
	 * Trigger TX digital reset
	 * 1.   EHIP CSR Write, Offset = 0x310, value = 0x2
	 */
	if (rx_reset)
		hssi_csrwr32_ba(pdev, HSSI_ETH_RECONFIG,
				chan, eth_soft_csroffs(eth_reset), ETH_SOFT_RX_RST);
	if (tx_reset)

		hssi_csrwr32_ba(pdev, HSSI_ETH_RECONFIG,
				chan, eth_soft_csroffs(eth_reset), ETH_SOFT_TX_RST);
}

void ftile_get_stats64(struct net_device *dev,
		       struct rtnl_link_stats64 *storage)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;
	u32 lsb;
	u32 msb;

	/* rx stats */
	lsb = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
			      eth_mac_ptp_csroffs(0, rx_frame_octetsok_lsb));
	msb = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
			      eth_mac_ptp_csroffs(0, rx_frame_octetsok_msb));
	storage->rx_bytes = ((u64)msb << 32) | lsb;

	lsb = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
			      eth_mac_ptp_csroffs(0, rx_mcast_data_ok_lsb));
	msb = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
			      eth_mac_ptp_csroffs(0, rx_mcast_data_ok_msb));
	storage->multicast = ((u64)msb << 32) | lsb;

	storage->collisions = 0;

	lsb = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
			      eth_mac_ptp_csroffs(0, rx_lenerr_lsb));
	msb = 0;
	storage->rx_length_errors = ((u64)msb << 32) | lsb;

	storage->rx_over_errors = 0;

	storage->rx_crc_errors = 0;

	storage->rx_fifo_errors = 0;
	storage->rx_missed_errors = 0;
	//IP UG does not have total RX packets, total RX bad packets, total RX dropped packets
	storage->rx_packets = priv->dev->stats.rx_packets;
	storage->rx_errors = 0;
	storage->rx_dropped = 0;
	/* also count the packets dropped by this network driver */
	storage->rx_dropped += dev->stats.rx_dropped;

	/* tx stats */
	lsb = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
			      eth_mac_ptp_csroffs(0, tx_frame_octetsok_lsb));
	msb = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
			      eth_mac_ptp_csroffs(0, tx_frame_octetsok_msb));
	storage->tx_bytes = ((u64)msb << 32) | lsb;

	lsb = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
			      eth_mac_ptp_csroffs(0, tx_malformed_ctrl_lsb));
	msb = 0;
	storage->tx_errors = ((u64)msb << 32) | lsb;

	lsb = hssi_csrrd32_ba(pdev, HSSI_ETH_RECONFIG, chan,
			      eth_mac_ptp_csroffs(0, tx_dropped_ctrl_lsb));
	msb = 0;
	storage->tx_dropped = ((u64)msb << 32) | lsb;

	storage->tx_aborted_errors = 0;
	storage->tx_fifo_errors = 0;
	storage->tx_heartbeat_errors = 0;
	storage->tx_window_errors = 0;
	storage->rx_compressed = 0;
	storage->tx_compressed = 0;
	//IP UG does not have total TX packets
	storage->tx_packets = priv->dev->stats.tx_packets;
}

static bool ftile_ptp_rx_ready_bit_is_set(intel_fpga_xtile_eth_private *priv)
{
	bool is_set = true;

	if (priv->ptp_enable) {
		// Check PTP RX ready bit set or not set,
		// If not, we need rerun ptp tx rx user flow again
		is_set = hssi_bit_is_set(priv->pdev_hssi, HSSI_ETH_RECONFIG, priv->tile_chan,
					 eth_soft_csroffs(ptp_status), ETH_RX_PTP_READY, true);
	}

	return is_set;
}

static void ftile_convert_eth_speed_to_eth_rate(intel_fpga_xtile_eth_private *priv)
{
	u32 eth_speed = priv->link_speed;

	switch (eth_speed) {
	case SPEED_10000:
	case SPEED_25000:
		priv->eth_rate = INTEL_FPGA_FTILE_ETH_RATE_10G_25G;
		break;
	case SPEED_50000:
		priv->eth_rate = INTEL_FPGA_FTILE_ETH_RATE_50G;
		break;
	case SPEED_40000:
	case SPEED_100000:
		priv->eth_rate = INTEL_FPGA_FTILE_ETH_RATE_40G_100G;
		break;
	case SPEED_200000:
		priv->eth_rate = INTEL_FPGA_FTILE_ETH_RATE_200G;
		break;
	case SPEED_400000:
		priv->eth_rate = INTEL_FPGA_FTILE_ETH_RATE_400G;
		break;
	default:
		pr_err("invalid eth speed %d, Failed to convert to eth_rate\n", eth_speed);
		break;
	}
}

int ftile_init(intel_fpga_xtile_eth_private *priv)
{
	/* Get eth_rate */
	ftile_convert_eth_speed_to_eth_rate(priv);

	/* Set/Config source MAC address */
	ftile_update_mac_addr(priv);

	return 0;
}

int ftile_start(intel_fpga_xtile_eth_private *priv)
{
	int ret;

	/* Enable F-tile MAC datapath */
	ftile_enable_mac(priv);

	/* Enable flow ctrl */
	ftile_enable_mac_flow_ctrl(priv);

	/* Enable PTP feature */
	if (priv->ptp_enable) {
		ret = eth_ftile_tx_rx_user_flow(priv);
		if (ret < 0)
			goto ptp_error;

		/* Start UI thread */
		ftile_ui_adjustments_init_worker(priv);
	}

	return 0;

ptp_error:
	ftile_disable_mac(priv);
	ftile_disable_mac_flow_ctrl(priv);
        return ret;
}

int ftile_stop(intel_fpga_xtile_eth_private *priv)
{
	/* Disable Ftile MAC datapath */
	ftile_disable_mac(priv);

	/* Disable Ftile MAC flow ctrl */
	ftile_disable_mac_flow_ctrl(priv);

	/* Stop UI thread */
        ui_adjustments_cancel_worker(priv);

	return 0;
}

int ftile_uninit(intel_fpga_xtile_eth_private *priv)
{
	/* Just to make sure Ftile feature are disabled */
	return ftile_stop(priv);
}

int ftile_run_check(intel_fpga_xtile_eth_private *priv)
{
	int ret;

	/* Check ptp rx ready bit is toggled,
	 * if yes, stop UI and rerun ptp tx rx user flow
	 */
	if (ftile_ptp_rx_ready_bit_is_set(priv)) {
		//do nothing
	} else {
		ui_adjustments_cancel_worker(priv);

		if ((ret = eth_ftile_tx_rx_user_flow(priv)))
			return ret;
		else
			ftile_ui_adjustments_init_worker(priv);
	}

	return 0;
}

bool ftile_get_link_fault_status(intel_fpga_xtile_eth_private *priv)
{
	int link_fault;

	link_fault = hssi_csrrd32_ba(priv->pdev_hssi, HSSI_ETH_RECONFIG, priv->tile_chan,
				     eth_soft_csroffs(link_fault_status));

	/* Check for remote link fault */
	if (link_fault & 0x2)
		return true;
	else
		return false;
}
