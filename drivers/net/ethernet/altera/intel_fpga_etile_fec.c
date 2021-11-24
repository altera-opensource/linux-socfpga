// SPDX-License-Identifier: GPL-2.0
/* Intel FPGA E-tile Forward Error Correction (FEC) Linux driver
 * Copyright (C) 2020-2021 Intel Corporation. All rights reserved.
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
#include "altera_utils.h"
#include "intel_fpga_etile.h"

#define MAX_COUNT_OFFSET		64000

/* Init FEC */
int fec_init(struct platform_device *pdev, struct intel_fpga_etile_eth_private *priv)
{
	int ret;

	/* get FEC type from device tree */
	ret  = of_property_read_string(pdev->dev.of_node, "fec-type",
				       &priv->fec_type);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot obtain fec-type\n");
		return ret;
	}
	dev_info(&pdev->dev, "\tFEC type is %s\n", priv->fec_type);

	/* get FEC channel from device tree */
	if (of_property_read_u32(pdev->dev.of_node, "fec-channel",
				 &priv->fec_channel)) {
		dev_err(&pdev->dev, "cannot obtain fec-channel\n");
		return -ENXIO;
	}
	dev_info(&pdev->dev, "\tfec-channel is 0x%x\n", priv->fec_channel);

	return 0;
}

/* Calculate Unit Interval Adjustments */
void ui_adjustments(struct timer_list *t)
{
	struct intel_fpga_etile_eth_private *priv = from_timer(priv, t, fec_timer);
	u32 tx_tam_l_initial, tx_tam_h_initial, tx_tam_count_initial;
	u32 rx_tam_l_initial, rx_tam_h_initial, rx_tam_count_initial;
	u32 tx_tam_l_nth, tx_tam_h_nth, tx_tam_count_nth;
	u32 rx_tam_l_nth, rx_tam_h_nth, rx_tam_count_nth;
	u64 tx_tam_initial, rx_tam_initial, tx_tam_nth, rx_tam_nth;
	u32 tx_tam_interval = 0, rx_tam_interval = 0;
	u32 tx_tam_count_est, rx_tam_count_est, ui_value, tx_tam_count, rx_tam_count;
	u64 tx_tam_delta, rx_tam_delta;
	u64 tx_ui, rx_ui;
	u64 start_jiffies;

	start_jiffies = get_jiffies_64();
	/* Set tam_snapshot to 1 to take the first snapshot of the Time of
	 * Alignment marker (TAM)
	 */
	tse_set_bit(priv->mac_dev, eth_ptp_csroffs(tam_snapshot),
		    ETH_TAM_SNAPSHOT);

	/* Read snapshotted initial TX TAM and counter values */
	tx_tam_l_initial = csrrd32(priv->mac_dev, eth_ptp_csroffs(tx_tam_l));
	tx_tam_h_initial = csrrd32(priv->mac_dev, eth_ptp_csroffs(tx_tam_h));
	tx_tam_initial = ((u64)tx_tam_h_initial << 32) | tx_tam_l_initial;
	tx_tam_count_initial = csrrd32(priv->mac_dev, eth_ptp_csroffs(tx_count));

	/* Read snapshotted initial RX TAM and counter values */
	rx_tam_l_initial = csrrd32(priv->mac_dev, eth_ptp_csroffs(rx_tam_l));
	rx_tam_h_initial = csrrd32(priv->mac_dev, eth_ptp_csroffs(rx_tam_h));
	rx_tam_initial = ((u64)rx_tam_h_initial << 32) | rx_tam_l_initial;
	rx_tam_count_initial = csrrd32(priv->mac_dev, eth_ptp_csroffs(rx_count));

	/* Clear snapshot */
	tse_clear_bit(priv->mac_dev, eth_ptp_csroffs(tam_snapshot),
		      ETH_TAM_SNAPSHOT);

	/* Wait for a few TAM interval */
	udelay(210);

	/* Request snapshot of Nth TX TAM and RX TAM */
	tse_set_bit(priv->mac_dev, eth_ptp_csroffs(tam_snapshot),
		    ETH_TAM_SNAPSHOT);

	/* Read snapshotted of Nth TX TAM and counter values */
	tx_tam_l_nth = csrrd32(priv->mac_dev, eth_ptp_csroffs(tx_tam_l));
	tx_tam_h_nth = csrrd32(priv->mac_dev, eth_ptp_csroffs(tx_tam_h));
	tx_tam_nth = ((u64)tx_tam_h_nth << 32) | tx_tam_l_nth;
	tx_tam_count_nth = csrrd32(priv->mac_dev, eth_ptp_csroffs(tx_count));

	/* Read snapshotted of Nth RX TAM and counter values */
	rx_tam_l_nth = csrrd32(priv->mac_dev, eth_ptp_csroffs(rx_tam_l));
	rx_tam_h_nth = csrrd32(priv->mac_dev, eth_ptp_csroffs(rx_tam_h));
	rx_tam_nth = ((u64)rx_tam_h_nth << 32) | rx_tam_l_nth;
	rx_tam_count_nth = csrrd32(priv->mac_dev, eth_ptp_csroffs(rx_count));

	/* Clear snapshot */
	tse_clear_bit(priv->mac_dev, eth_ptp_csroffs(tam_snapshot),
		      ETH_TAM_SNAPSHOT);
	if ((get_jiffies_64() - start_jiffies) > HZ) {
		printk(KERN_ALERT "%s: 1st to Nth snapshot takes more than 1 second\n"
		, __func__);
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
		break;
	case PHY_INTERFACE_MODE_25GKR:
		ui_value = INTEL_FPGA_ETILE_UI_VALUE_25G;
		break;
	default:
		ui_value = 0; //invalid value
	}

	/* Calculate estimated count value */
	if (ui_value > 0) {
		if (tx_tam_interval > 0)
			tx_tam_count_est = tx_tam_delta / (tx_tam_interval * ui_value);

		if (rx_tam_interval > 0)
			rx_tam_count_est = rx_tam_delta / (rx_tam_interval * ui_value);
	}

	/* if estimated count value is more than 64000 (max count value with
	 * offset), discard the snapshot and repeat steps
	 */
	if (tx_tam_count_est > MAX_COUNT_OFFSET ||
		rx_tam_count_est > MAX_COUNT_OFFSET) {
		printk(KERN_ALERT "%s: estimated count value (tx: %d, rx: %d)"
			"is more than %d\n", __func__, tx_tam_count_est,
			rx_tam_count_est, MAX_COUNT_OFFSET);
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

	if (tx_ui > 0x9EE42 || tx_ui < 0x9EDC0) {
		printk(KERN_ALERT "%s: TX UI value (0x%llx) is not within "
		"0x9EDC0 to 0x9EE42 range\n",__func__, tx_ui);
		goto ui_restart;
	}
	if (rx_ui > 0x9EE42 || rx_ui < 0x9EDC0) {
		printk(KERN_ALERT "%s: RX UI value (0x%llx) is not within "
		"0x9EDC0 to 0x9EE42 range\n",__func__, rx_ui);
		goto ui_restart;
	}

	csrwr32(tx_ui, priv->mac_dev, eth_ptp_csroffs(tx_ui_reg));
	csrwr32(rx_ui, priv->mac_dev, eth_ptp_csroffs(rx_ui_reg));

ui_restart:
	mod_timer(&priv->fec_timer, jiffies + msecs_to_jiffies(1000));
}

MODULE_LICENSE("GPL");
