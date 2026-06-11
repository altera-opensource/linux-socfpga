// SPDX-License-Identifier: GPL-2.0
/* Altera FPGA GTS Forward Error Correction (FEC) Linux driver
 * Copyright (C) 2025-2026 Altera Corporation. All rights reserved.
 *
 * Contributors:
 *   Preetam Narayan
 */

#include <linux/bitops.h>
#include <linux/if_vlan.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/phylink.h>
#include "intel_fpga_eth_main.h"
#include "altera_eth_dma.h"
#include "intel_fpga_hssigl_gts_driver.h"
#include "intel_fpga_eth_gts.h"
#include "intel_fpga_eth_hssi_itf.h"
#include "intel_fpga_gts_driver.h"

#define MAX_COUNT_OFFSET		64000

#define GTS_10G_RX_TX_MIN_UI		0x18CC73E
#define GTS_10G_RX_TX_MAX_UI		0x18D98F5

#define GTS_25G_RX_TX_MIN_UI		0x9EDC00
#define GTS_25G_RX_TX_MAX_UI		0x9EE420

/* write protected read for the ui enable status */
static inline bool wpr_get_uienable_status(intel_fpga_xtile_eth_private *priv)
{
	bool value = false;

	read_lock(&priv->wr_lock);
	value = priv->ui_enable;
	read_unlock(&priv->wr_lock);

	return value;
}

/* read protected write for the ui enable status */
static inline void rpw_set_uienable_status(bool value,
					   intel_fpga_xtile_eth_private *priv) {
	write_lock(&priv->wr_lock);
	priv->ui_enable = value;
	write_unlock(&priv->wr_lock);
}

static void get_min_max_ui(intel_fpga_xtile_eth_private *priv, u64 *min_ui, u64 *max_ui)
{
	if (!min_ui || !max_ui) {
		dev_warn(priv->device, "%s: Invalid params\n", __func__);
		return;
	}

	switch (priv->link_speed) {
	case SPEED_10000:
		*min_ui = GTS_10G_RX_TX_MIN_UI;
		*max_ui = GTS_10G_RX_TX_MAX_UI;
		break;
	case SPEED_25000:
		*min_ui = GTS_25G_RX_TX_MIN_UI;
		*max_ui = GTS_25G_RX_TX_MAX_UI;
		break;
	default:
		dev_warn(priv->device, "%s: Eth link speed  unknown\n",
			 __func__);
		*min_ui = 0;
		*max_ui = 0;
		break;
	}
}

static void ui_adjustments_worker_handle(struct timer_list *t)
{
	intel_fpga_xtile_eth_private *priv = from_timer(priv, t, fec_timer);

	schedule_work(&priv->ui_worker);
}

void gts_ui_adjustments_cancel_worker(intel_fpga_xtile_eth_private *priv)
{
	/* if the ui adjustment timer is already cancelled and we request
	 * cancel again, case should be avoided
	 */
	if (!wpr_get_uienable_status(priv))
		return;

	/* we cancel the timer so that it doesn't schedule new
	 * worker thread execution
	 */
	rpw_set_uienable_status(false, priv);
	del_timer_sync(&priv->fec_timer);
	cancel_work_sync(&priv->ui_worker);
}

#define MIN_UI_REF_TIME 5300
static bool gts_ui_tx_tam_values(intel_fpga_xtile_eth_private *priv,
				 u64 *tam_initial,
				 u32 *tam_count_initial)
{
	bool ret = false;
	s8 snapshot_trial = 5;
	u64 tx_tam_initial;
	u32 tx_tam_count_initial;
	u32 tx_tam_l_initial, tx_tam_h_initial, ptp_tx_uim_tam_info1;
	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;

	/* Step1: Request snapshot of initial TX TAM */
	hssi_set_bit(pdev, HSSI_PTP_SOFTIP, chan,
		     eth_softip_ptp_csroffs(ptp_uim_tam_snapshot),
		     ETH_TX_TAM_SNAPSHOT);

	/* Snapshot should be set before proceeding to TAM calculation */
	do {
		udelay(1);

		ptp_tx_uim_tam_info1 = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, chan,
						    eth_softip_ptp_csroffs(ptp_tx_uim_tam_info1));

	} while (--snapshot_trial && !(ptp_tx_uim_tam_info1 & ETH_TX_TAM_VALID));

	if (!(ptp_tx_uim_tam_info1 & ETH_TX_TAM_VALID)) {
		dev_err(priv->device, "Tx snapshot capture failed\n");

		goto failed;
	}

	/* Step2: Read snapshoted initial TAM and counter values */
	tx_tam_l_initial = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, chan,
					eth_softip_ptp_csroffs(ptp_tx_uim_tam_info0));

	tx_tam_h_initial = ptp_tx_uim_tam_info1 & ETH_TX_TAM_HI_NS;
	tx_tam_count_initial = (ptp_tx_uim_tam_info1 & ETH_TX_TAM_CNT_MASK) >> ETH_TX_TAM_CNT_SHIFT;

	tx_tam_initial = ((u64)tx_tam_h_initial << 32) | tx_tam_l_initial;

	*tam_initial = tx_tam_initial;
	*tam_count_initial = tx_tam_count_initial;

	ret = true;

failed:
	hssi_clear_bit(pdev, HSSI_PTP_SOFTIP, chan,
		       eth_softip_ptp_csroffs(ptp_uim_tam_snapshot),
		       ETH_TX_TAM_SNAPSHOT);

	return ret;
}

static bool gts_ui_rx_tam_values(intel_fpga_xtile_eth_private *priv,
				 u64 *tam_initial,
				u32 *tam_count_initial)
{
	bool ret = false;
	s8 snapshot_trial = 5;
	u64 rx_tam_initial;
	u32 rx_tam_count_initial;
	u32 rx_tam_l_initial, rx_tam_h_initial, ptp_rx_uim_tam_info1;
	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;

	/* Step1: Request snapshot of initial TX TAM */
	hssi_set_bit(pdev, HSSI_PTP_SOFTIP, chan,
		     eth_softip_ptp_csroffs(ptp_uim_tam_snapshot),
		     ETH_RX_TAM_SNAPSHOT);

	/* Snapshot should be set before proceeding to TAM calculation */
	do {
		udelay(1);

		ptp_rx_uim_tam_info1 = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, chan,
						    eth_softip_ptp_csroffs(ptp_rx_uim_tam_info1));

	} while (--snapshot_trial && !(ptp_rx_uim_tam_info1 & ETH_RX_TAM_VALID));

	if (!(ptp_rx_uim_tam_info1 & ETH_RX_TAM_VALID)) {
		dev_err(priv->device, "Rx snapshot capture failed\n");

		goto failed;
	}

	/* Step2: Read snapshoted initial TAM and counter values */
	rx_tam_l_initial = hssi_csrrd32(pdev, HSSI_PTP_SOFTIP, chan,
					eth_softip_ptp_csroffs(ptp_rx_uim_tam_info0));

	rx_tam_h_initial = ptp_rx_uim_tam_info1 & ETH_RX_TAM_HI_NS;
	rx_tam_count_initial = (ptp_rx_uim_tam_info1 & ETH_RX_TAM_CNT_MASK) >> ETH_RX_TAM_CNT_SHIFT;

	rx_tam_initial = ((u64)rx_tam_h_initial << 32) | rx_tam_l_initial;

	*tam_initial = rx_tam_initial;
	*tam_count_initial = rx_tam_count_initial;

	ret = true;

failed:
	hssi_clear_bit(pdev, HSSI_PTP_SOFTIP, chan,
		       eth_softip_ptp_csroffs(ptp_uim_tam_snapshot),
		       ETH_RX_TAM_SNAPSHOT);

	return ret;
}

static bool calculate_tx_ui(intel_fpga_xtile_eth_private *priv,
			    u64 tx_tam_initial, u64 tx_tam_nth,
			    u32 tx_tam_count_initial, u32 tx_tam_count_nth)

{
	u64 tx_tam_delta;
	u32 tx_tam_interval = 0;
	u64 tx_ui, min_ui, max_ui;
	u32 tx_tam_count;
	u32 ui_value;
	u16 num_pl = priv->pma_lanes_used;
	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;

	if ((priv->link_speed == SPEED_25000) || (priv->link_speed == SPEED_10000))
		tx_tam_interval = 5406720;

	/* Calculate time elapsed */
	if (tx_tam_nth <= tx_tam_initial)
		// 10^9 ns = 0x3B9ACA000000
		tx_tam_delta = tx_tam_nth + 0x3B9ACA000000UL - tx_tam_initial;
	else
		tx_tam_delta = tx_tam_nth - tx_tam_initial;

	dev_dbg(priv->device, "tx_tam_initial:0x%llx tx_tam_nth:0x%llx tx_tam_delta:0x%llx\n",
		tx_tam_initial, tx_tam_nth, tx_tam_delta);

	// TBD add other PHY modes and ui_value for those...
	switch (priv->phy_iface) {
	case PHY_INTERFACE_MODE_10GKR:
	case PHY_INTERFACE_MODE_10GBASER:
		ui_value = INTEL_FPGA_GTS_UI_VALUE_10G;
		break;
	case PHY_INTERFACE_MODE_25GKR:
		ui_value = INTEL_FPGA_GTS_UI_VALUE_25G;
		break;
	default:
		ui_value = 0; //invalid value
	}

	dev_dbg(priv->device,
		"tx_tam_count_initial:0x%08x tx_tam_count_nth:0x%08x tx_tam_count:0x%08x\n",
		tx_tam_count_initial, tx_tam_count_nth, tx_tam_count);

	/* Calculate TAM count value */
	if (tx_tam_count_nth <= tx_tam_count_initial)
		tx_tam_count = (tx_tam_count_nth + (1 << 15)) - tx_tam_count_initial;
	else
		tx_tam_count = tx_tam_count_nth - tx_tam_count_initial;

	// Make sure the format is {4-bit nanoseconds, 28-bit fractional nanoseconds}
	tx_ui = (tx_tam_delta << 12) / (((u64)tx_tam_count * tx_tam_interval) / num_pl);

	get_min_max_ui(priv, &min_ui, &max_ui);

	// check new tx_ui against min./max. ui_value
	if (tx_ui > max_ui || tx_ui < min_ui) {
		dev_warn(priv->device, "TX UI value (0x%llX) is not within (0x%llx) to (0x%llx) range\n",
			 tx_ui, min_ui, max_ui);

		return false;
	}

	hssi_csrwr32(pdev, HSSI_EMAC_HARDIP,
		     chan, eth_hardip_emac_csroffs(tx_ptp_ui),
		     tx_ui);

	return true;
}

static void calculate_rx_ui(intel_fpga_xtile_eth_private *priv,
			    u64 rx_tam_initial, u64 rx_tam_nth,
			   u32 rx_tam_count_initial, u32 rx_tam_count_nth)
{
	u16 num_pl = priv->pma_lanes_used;
	u64 rx_tam_delta;
	u32 rx_tam_interval = 0;
	u64 rx_ui, min_ui, max_ui;
	u32 ui_value;
	u32 rx_tam_count;
	struct platform_device *pdev = priv->pdev_hssi;
	u32 chan = priv->tile_chan;

	rx_tam_interval = 168960;
	if ((priv->link_speed == SPEED_25000) && !strcasecmp(priv->fec_type, "kr-fec"))
		rx_tam_interval = 5406720;

	/* Calculate time elapsed */
	if (rx_tam_nth <= rx_tam_initial)
		// 10^9 ns = 0x3B9ACA000000
		rx_tam_delta = rx_tam_nth + 0x3B9ACA000000UL - rx_tam_initial;
	else
		rx_tam_delta = rx_tam_nth - rx_tam_initial;

	dev_dbg(priv->device, "rx_tam_initial:0x%llx rx_tam_nth:0x%llx rx_tam_delta:0x%llx\n",
		rx_tam_initial, rx_tam_nth, rx_tam_delta);

	// TBD add other PHY modes and ui_value for those...
	switch (priv->phy_iface) {
	case PHY_INTERFACE_MODE_10GKR:
	case PHY_INTERFACE_MODE_10GBASER:
		ui_value = INTEL_FPGA_GTS_UI_VALUE_10G;
		break;
	case PHY_INTERFACE_MODE_25GKR:
		ui_value = INTEL_FPGA_GTS_UI_VALUE_25G;
		break;
	default:
		ui_value = 0; //invalid value
	}

	if (rx_tam_count_nth <= rx_tam_count_initial)
		rx_tam_count = (rx_tam_count_nth + (1 << 15)) - rx_tam_count_initial;
	else
		rx_tam_count = rx_tam_count_nth - rx_tam_count_initial;

	dev_dbg(priv->device,
		"rx_tam_count_initial:0x%08x rx_tam_count_nth:0x%08x rx_tam_count:0x%08x\n",
		rx_tam_count_initial, rx_tam_count_nth, rx_tam_count);

	// Make sure the format is {4-bit nanoseconds, 28-bit fractional nanoseconds}
	rx_ui = (rx_tam_delta << 12) / (((u64)rx_tam_count * rx_tam_interval) / num_pl);

	get_min_max_ui(priv, &min_ui, &max_ui);

	// check new rx_ui against min./max. ui_value
	if (rx_ui > max_ui || rx_ui < min_ui) {
		dev_warn(priv->device, "RX UI value (0x%llX) is not within (0x%llx) to (0x%llx) range\n",
			 rx_ui, min_ui, max_ui);

		return;
	}

	hssi_csrwr32(pdev, HSSI_EMAC_HARDIP,
		     chan, eth_hardip_emac_csroffs(rx_ptp_ui),
		     rx_ui);
}

/* Calculate Unit Interval Adjustments */
void gts_ui_adjustments(struct work_struct *work)
{
	intel_fpga_xtile_eth_private *priv = container_of(work, intel_fpga_xtile_eth_private,
							  ui_worker);
	u64 start_jiffies;
	u64 tx_tam_initial, rx_tam_initial;
	u32 tx_tam_count_initial, rx_tam_count_initial;
	u32 tx_tam_count_nth, rx_tam_count_nth;
	u64 tx_tam_nth, rx_tam_nth;
	bool  tx_tam_valid, rx_tam_valid;

	if (priv->ui_adjust_interval == 0)
		goto ui_restart;

	/* to avoid race condition where the timer is deleted and we are scheduled */
	if (!wpr_get_uienable_status(priv))
		return;

	start_jiffies = get_jiffies_64();

	tx_tam_valid = gts_ui_tx_tam_values(priv, &tx_tam_initial, &tx_tam_count_initial);

	rx_tam_valid = gts_ui_rx_tam_values(priv, &rx_tam_initial, &rx_tam_count_initial);

	if (!rx_tam_valid || !tx_tam_valid) {
		dev_warn(priv->device, "%s: Initial rx_tam_valid=%u tx_tam_valid=%u\n", __func__,
			 rx_tam_valid, tx_tam_valid);

		goto ui_restart;
	}

	/* Wait for a few TAM interval */
	udelay(MIN_UI_REF_TIME);

	/* Read snapshotted of Nth TX TAM and counter values */
	tx_tam_valid = gts_ui_tx_tam_values(priv, &tx_tam_nth, &tx_tam_count_nth);

	/* Read snapshotted of Nth RX TAM and counter values */
	rx_tam_valid = gts_ui_rx_tam_values(priv, &rx_tam_nth, &rx_tam_count_nth);

	if ((get_jiffies_64() - start_jiffies) > HZ) {
		dev_warn(priv->device, "%s: 1st to Nth snapshot takes more than 1 second\n",
			 __func__);
		goto ui_restart;

	} else if (!rx_tam_valid || !tx_tam_valid) {
		dev_warn(priv->device, "%s: Nth rx_tam_valid=%u tx_tam_valid=%u\n", __func__,
			 rx_tam_valid, tx_tam_valid);

		goto ui_restart;
	}

	if (calculate_tx_ui(priv, tx_tam_initial, tx_tam_nth,
			    tx_tam_count_initial, tx_tam_count_nth)) {
		calculate_rx_ui(priv, rx_tam_initial, rx_tam_nth,
				rx_tam_count_initial, rx_tam_count_nth);
	}

ui_restart:

	/* to avoid race condition where the timer is deleted and we are scheduled */
	if (!wpr_get_uienable_status(priv))
		return;

	if (priv->ui_adjust_interval == 0)
		mod_timer(&priv->fec_timer, jiffies + msecs_to_jiffies(1000));
	else
		mod_timer(&priv->fec_timer, jiffies + msecs_to_jiffies(priv->ui_adjust_interval));
}

void gts_ui_adjustments_init_worker(intel_fpga_xtile_eth_private *priv)
{
	int ret;

	rpw_set_uienable_status(true, priv);
	INIT_WORK(&priv->ui_worker, gts_ui_adjustments);
	timer_setup(&priv->fec_timer, ui_adjustments_worker_handle, 0);

	if (priv->ui_adjust_interval == 0)
		ret = mod_timer(&priv->fec_timer, jiffies + msecs_to_jiffies(250));
	else
		ret = mod_timer(&priv->fec_timer,
				jiffies + msecs_to_jiffies(priv->ui_adjust_interval));

	if (ret)
		netdev_err(priv->dev, "Timer failed to start UI adjustment\n");
}

MODULE_LICENSE("GPL");
