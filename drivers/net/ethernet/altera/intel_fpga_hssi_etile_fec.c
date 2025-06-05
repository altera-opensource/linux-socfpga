// SPDX-License-Identifier: GPL-2.0
/* Altera FPGA E-tile Forward Error Correction (FEC) Linux driver
 * Copyright (C) 2020-2022 Altera Corporation. All rights reserved.
 *
 * Contributors:
 *   Joyce Ooi
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

#include "altera_eth_dma.h"
#include "intel_fpga_eth_etile.h"
#include "intel_fpga_eth_hssi_itf.h"

#define MAX_COUNT_OFFSET		64000

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

static void etile_ui_adjustments_worker_handle(struct timer_list *t) //timer handler
{
	intel_fpga_xtile_eth_private *priv = from_timer(priv, t, fec_timer);

	schedule_work(&priv->ui_worker);
}

void etile_ui_adjustments_cancel_worker(intel_fpga_xtile_eth_private *priv)
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

/* Calculate Unit Interval Adjustments */
static void etile_ui_adjustments(struct work_struct *work)
{
	intel_fpga_xtile_eth_private *priv;
	struct platform_device *pdev;
	u32 chan;
	u32 tx_tam_l_initial, tx_tam_h_initial, tx_tam_count_initial;
	u32 rx_tam_l_initial, rx_tam_h_initial, rx_tam_count_initial;
	u32 tx_tam_l_nth, tx_tam_h_nth, tx_tam_count_nth;
	u32 rx_tam_l_nth, rx_tam_h_nth, rx_tam_count_nth;
	u64 tx_tam_initial, rx_tam_initial, tx_tam_nth, rx_tam_nth;
	u32 tx_tam_interval = 0, rx_tam_interval = 0;
	u32 tx_tam_count_est = 0, rx_tam_count_est = 0;
	u32 ui_value = 0, tx_tam_count = 0, rx_tam_count = 0;
	u64 tx_tam_delta, rx_tam_delta;
	u64 tx_ui = 0, rx_ui = 0;
	u64 start_jiffies;
	u32 ui_value_16bit_fns = 0;

	priv = container_of(work, intel_fpga_xtile_eth_private, ui_worker);
	pdev = priv->pdev_hssi;
	chan = priv->tile_chan;

	if (priv->ui_adjust_interval == 0)
		goto ui_restart;

	/* to avoid race condition where the timer is deleted and we are scheduled */
	if (!wpr_get_uienable_status(priv))
		return;

	start_jiffies = get_jiffies_64();

	/* Set tam_snapshot to 1 to take the first snapshot of the Time of
	 * Alignment marker (TAM)
	 */
	hssi_set_bit(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(tam_snapshot),
		     ETH_TAM_SNAPSHOT);

	/* Read snapshotted initial TX TAM and counter values */
	tx_tam_l_initial = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(tx_tam_l));
	tx_tam_h_initial = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(tx_tam_h));
	tx_tam_initial = ((u64)tx_tam_h_initial << 32) | tx_tam_l_initial;
	tx_tam_count_initial = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG,
					    chan, eth_ptp_csroffs(tx_count));

	/* Read snapshotted initial RX TAM and counter values */
	rx_tam_l_initial = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(rx_tam_l));
	rx_tam_h_initial = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(rx_tam_h));
	rx_tam_initial = ((u64)rx_tam_h_initial << 32) | rx_tam_l_initial;
	rx_tam_count_initial = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG,
					    chan, eth_ptp_csroffs(rx_count));

	/* Clear snapshot */
	hssi_clear_bit(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(tam_snapshot),
		       ETH_TAM_SNAPSHOT);

	/* Wait for a few TAM interval */
	udelay(5300);

	/* Request snapshot of Nth TX TAM and RX TAM */
	hssi_set_bit(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(tam_snapshot),
		     ETH_TAM_SNAPSHOT);

	/* Read snapshotted of Nth TX TAM and counter values */
	tx_tam_l_nth = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(tx_tam_l));
	tx_tam_h_nth = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(tx_tam_h));
	tx_tam_nth = ((u64)tx_tam_h_nth << 32) | tx_tam_l_nth;
	tx_tam_count_nth = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(tx_count));

	/* Read snapshotted of Nth RX TAM and counter values */
	rx_tam_l_nth = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(rx_tam_l));
	rx_tam_h_nth = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(rx_tam_h));
	rx_tam_nth = ((u64)rx_tam_h_nth << 32) | rx_tam_l_nth;
	rx_tam_count_nth = hssi_csrrd32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(rx_count));

	/* Clear snapshot */
	hssi_clear_bit(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(tam_snapshot),
		       ETH_TAM_SNAPSHOT);

	if ((get_jiffies_64() - start_jiffies) > HZ) {
		netdev_warn(priv->dev,
			    "%s:1st to Nth snapshot takes more than 1 second\n",
			    __func__);
		goto ui_restart;
	}

	/* Calculate new UI value */
	/* Reference Time (TAM) interval = AM interval * Unit interval of serial bit
	 * AM interval for No FEC for 10/25GbE: TX = 5406720, RX = 6336
	 * AM interval for KR-FEC for 25GbE: TX = 5406720, RX = 5406720
	 * Unit interval of serial bit = 0.0387878 nanoseconds
	 */
	if (!strcasecmp(priv->fec_type, "kr-fec")) {
		tx_tam_interval = 5406720;
		rx_tam_interval = 5406720;
	} else if (!strcasecmp(priv->fec_type, "no-fec")) {
		tx_tam_interval = 5406720;
		rx_tam_interval = 6336;
	}

	/* Calculate time elapsed */
	if (tx_tam_nth <= tx_tam_initial)
		tx_tam_delta = (tx_tam_nth + (int_pow(10, 9) << 16)) - tx_tam_initial;
	else
		tx_tam_delta = tx_tam_nth - tx_tam_initial;

	if (rx_tam_nth <= rx_tam_initial)
		rx_tam_delta = (rx_tam_nth + (int_pow(10, 9) << 16)) - rx_tam_initial;
	else
		rx_tam_delta = rx_tam_nth - rx_tam_initial;

	switch (priv->phy_iface) {
	case PHY_INTERFACE_MODE_10GKR:
	case PHY_INTERFACE_MODE_10GBASER:
		ui_value = INTEL_FPGA_ETILE_UI_VALUE_10G;
		ui_value_16bit_fns = ui_value >> 8;
		break;
	case PHY_INTERFACE_MODE_25GKR:
		ui_value = INTEL_FPGA_ETILE_UI_VALUE_25G;
		ui_value_16bit_fns = ui_value >> 8;
		break;
	default:
		ui_value = 0; //invalid value
	}

	/* Calculate estimated count value */
	if (ui_value > 0) {
		if (tx_tam_interval > 0)
			tx_tam_count_est = tx_tam_delta /
			(tx_tam_interval * ui_value_16bit_fns);

		if (rx_tam_interval > 0)
			rx_tam_count_est = rx_tam_delta /
			(rx_tam_interval * ui_value_16bit_fns);
	}

	/* if estimated count value is more than 64000 (max count value with
	 * offset), discard the snapshot and repeat steps
	 */
	if (tx_tam_count_est > MAX_COUNT_OFFSET) {
		if (unlikely(netif_msg_hw(priv))) {
			netdev_warn(priv->dev,
				    "Est count exceeded:tx_tam_count_est: %u = tx_tam_delta:%llu / (tx_tam_interval:%u * ui_value_16bit_fns:0x%x)\n",
				    tx_tam_count_est, tx_tam_delta, tx_tam_interval,
				    ui_value_16bit_fns);
			netdev_warn(priv->dev, "tx_tam_nth: %llu, tx_tam_initial: %llu\n",
				    tx_tam_nth, tx_tam_initial);
		}
		goto ui_restart;
	}

	if (rx_tam_count_est > MAX_COUNT_OFFSET) {
		if (unlikely(netif_msg_hw(priv))) {
			netdev_warn(priv->dev,
				    "Est count exceeded:rx_tam_count_est: %u = rx_tam_delta:%llu / (rx_tam_interval:%u * ui_value_16bit_fns:0x%x)\n",
				    rx_tam_count_est, rx_tam_delta, rx_tam_interval,
				    ui_value_16bit_fns);
			netdev_warn(priv->dev, "rx_tam_nth: %llu, rx_tam_initial: %llu\n",
				    rx_tam_nth, rx_tam_initial);
		}
		goto ui_restart;
	}

	/* Calculate TAM count value */
	if (tx_tam_count_nth <= tx_tam_count_initial)
		tx_tam_count = (tx_tam_count_nth + int_pow(2, 16)) - tx_tam_count_initial;
	else
		tx_tam_count = tx_tam_count_nth - tx_tam_count_initial;

	if (rx_tam_count_nth <= rx_tam_count_initial)
		rx_tam_count = (rx_tam_count_nth + int_pow(2, 16)) - rx_tam_count_initial;
	else
		rx_tam_count = rx_tam_count_nth - rx_tam_count_initial;

	/* Calculate UI value */
	if (tx_tam_count > 0 && tx_tam_interval > 0)
		tx_ui = (tx_tam_delta * int_pow(2, 8)) / (tx_tam_count * tx_tam_interval);

	if (rx_tam_count > 0 && rx_tam_interval > 0)
		rx_ui = (rx_tam_delta * int_pow(2, 8)) / (rx_tam_count * rx_tam_interval);

	/* UI Adjustment for 25G kr-fec */
	if (priv->link_speed == SPEED_25000) {
		if (tx_ui > 0x9EE42 || tx_ui < 0x9EDC0) {
			if (unlikely(netif_msg_hw(priv))) {
				netdev_warn(priv->dev,
					    "%s: TX UI value(0x%llx) is not within 0x9EDC0 to 0x9EE42 range\n",
					    __func__, tx_ui);
			}
			goto ui_restart;
		}
		if (rx_ui > 0x9EE42 || rx_ui < 0x9EDC0) {
			if (unlikely(netif_msg_hw(priv))) {
				netdev_warn(priv->dev,
					    "%s: RX UI value(0x%llx) is not within 0x9EDC0 to 0x9EE42 range\n",
					    __func__, rx_ui);
			}
			goto ui_restart;
		}
	} else {
		if (tx_ui > 0x18D3A4 || tx_ui < 0x18D25F) {
			if (unlikely(netif_msg_hw(priv))) {
				netdev_warn(priv->dev,
					    "%s: TX UI value (0x%llx) is not within 0x18D25F to 0x18D3A4 range\n",
					    __func__, tx_ui);
			}
			goto ui_restart;
		}
		if (rx_ui > 0x18D3A4 || rx_ui < 0x18D25F) {
			if (unlikely(netif_msg_hw(priv))) {
				netdev_warn(priv->dev,
					    "%s: RX UI value (0x%llx) is not within 0x18D25F to 0x18D3A4 range\n",
					    __func__, rx_ui);
			}
			goto ui_restart;
		}
	}

	hssi_csrwr32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(tx_ui_reg), tx_ui);
	hssi_csrwr32(pdev, HSSI_ETH_RECONFIG, chan, eth_ptp_csroffs(rx_ui_reg), rx_ui);

ui_restart:

	/* to avoid race condition where the timer is deleted and we are scheduled */
	if (!wpr_get_uienable_status(priv))
		return;

	if (priv->ui_adjust_interval == 0)
		mod_timer(&priv->fec_timer, jiffies + msecs_to_jiffies(20000));
	else
		mod_timer(&priv->fec_timer, jiffies + msecs_to_jiffies(priv->ui_adjust_interval));
}

void etile_ui_adjustments_init_worker(intel_fpga_xtile_eth_private *priv)
{
	int ret;

	rpw_set_uienable_status(true, priv);
	INIT_WORK(&priv->ui_worker, etile_ui_adjustments);
	timer_setup(&priv->fec_timer, etile_ui_adjustments_worker_handle, 0);
	ret = mod_timer(&priv->fec_timer, jiffies + msecs_to_jiffies(5000));
	if (ret)
		netdev_err(priv->dev, "Timer failed to start UI adjustment\n");
}

MODULE_LICENSE("GPL");
