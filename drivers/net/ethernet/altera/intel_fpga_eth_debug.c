// SPDX-License-Identifier: GPL-2.0
/* Altera FPGA Network debug interface API
 * Copyright (C) 2022, 2025 Altera Corporation. All rights reserved
 *
   Contributors:
   Preetam Narayan
 *
 */

 #include "altera_eth_dma.h"
 #include "altera_msgdma.h"
 #include "altera_msgdmahw.h"
 #include "altera_utils.h"
 #include "altera_msgdma_prefetcher.h"
 #include "altera_msgdmahw_prefetcher.h"
 #include "altera_sgdma.h"
 #include "intel_fpga_eth_main.h"
 #include "intel_fpga_eth_tile_ops.h"
 #include "intel_fpga_eth_hssi_itf.h"
 #include "intel_freq_control.h"
 #include <linux/string.h>

static void xtile_prefetcher_reg_dump_tx(struct altera_dma_private *priv)
{
	s32 ret;

	netdev_info(priv->dev, "TX PREFETCHER:\n");

	ret = csrrd32(priv->tx_pref_csr, msgdma_pref_csroffs(control));
	netdev_info(priv->dev, "\tCONTROL	     | 0x%x\n", ret);

	ret = csrrd32(priv->tx_pref_csr, msgdma_pref_csroffs(status));
	netdev_info(priv->dev, "\tSTATUS	     | 0x%x\n", ret);

	ret = csrrd32(priv->tx_pref_csr, msgdma_pref_csroffs(next_desc_lo));
	netdev_info(priv->dev, "\tNEXT DESC LOW  | 0x%x\n", ret);
}

static void xtile_prefetcher_reg_dump_rx(struct altera_dma_private *priv)
{
	s32 ret;

	netdev_info(priv->dev, "RX PREFETCHER:\n");

	ret = csrrd32(priv->rx_pref_csr, msgdma_pref_csroffs(control));
	netdev_info(priv->dev, "\tCONTROL	    | 0x%x\n", ret);

	ret = csrrd32(priv->rx_pref_csr, msgdma_pref_csroffs(status));
	netdev_info(priv->dev, "\tSTATUS	    | 0x%x\n", ret);

	ret = csrrd32(priv->rx_pref_csr, msgdma_pref_csroffs(next_desc_lo));
	netdev_info(priv->dev, "\tNEXT DESC LOW  | 0x%x\n", ret);
}

static void xtile_dispatcher_reg_dump_tx(struct altera_dma_private *priv)
{
	s32 ret;

	netdev_info(priv->dev, "Tx DISPATCHER:\n");

	ret = csrrd32(priv->tx_dma_csr, msgdma_csroffs(status));
	netdev_info(priv->dev, "\tSTATUS   | 0x%x\n", ret);

	ret = csrrd32(priv->tx_dma_csr, msgdma_csroffs(control));
	netdev_info(priv->dev, "\tCONTROL  | 0x%x\n", ret);
}

static void xtile_dispatcher_reg_dump_rx(struct altera_dma_private *priv)
{
	s32 ret;

	netdev_info(priv->dev, "Rx DISPATCHER:\n");

	ret = csrrd32(priv->rx_dma_csr, msgdma_csroffs(status));
	netdev_info(priv->dev, "\tSTATUS   | 0x%x\n", ret);

	ret = csrrd32(priv->rx_dma_csr, msgdma_csroffs(control));
	netdev_info(priv->dev, "\tCONTROL  | 0x%x\n", ret);
}

static void xtile_fifo_fill_level_tx(struct altera_dma_private *priv)
{
	s32 ret;

	netdev_info(priv->dev, "Tx FILL LEVEL:\n");

	ret = csrrd32(priv->tx_dma_csr, msgdma_csroffs(rw_fill_level));

	netdev_info(priv->dev, "\tWR FILL LEVEL  | 0x%x\n", MSGDMA_CSR_WR_FILL_LEVEL_GET(ret));
	netdev_info(priv->dev, "\tRD FILL LEVEL  | 0x%x\n", MSGDMA_CSR_RD_FILL_LEVEL_GET(ret));

	ret = MSGDMA_CSR_RESP_FILL_LEVEL_GET(csrrd32(priv->tx_dma_csr,
						     msgdma_csroffs(resp_fill_level)));

	netdev_info(priv->dev, "\tRSP FILL LEVEL | 0x%x\n", ret);
}

static void xtile_fifo_fill_level_rx(struct altera_dma_private *priv)
{
	s32 ret;

	netdev_info(priv->dev, "Rx FILL LEVEL:\n");

	ret = csrrd32(priv->rx_dma_csr, msgdma_csroffs(rw_fill_level));

	netdev_info(priv->dev, "\tWR FILL LEVEL  | 0x%x\n",
		    MSGDMA_CSR_WR_FILL_LEVEL_GET(ret));
	netdev_info(priv->dev, "\tRD FILL LEVEL  | 0x%x\n",
		    MSGDMA_CSR_RD_FILL_LEVEL_GET(ret));

	ret = MSGDMA_CSR_RESP_FILL_LEVEL_GET(csrrd32(priv->rx_dma_csr,
						     msgdma_csroffs(resp_fill_level)));

	netdev_info(priv->dev, "\tRSP FILL LEVEL | 0x%x\n", ret);
}

static void xtile_process_seq_no_tx(struct altera_dma_private *priv)
{
	netdev_info(priv->dev, "Tx PROD/CONS     | 0x%x/0x%x (Diff: %d)\n",
		    priv->tx_prod, priv->tx_cons, (priv->tx_prod - priv->tx_cons));
}

static void xtile_process_seq_no_rx(struct altera_dma_private *priv)
{
	netdev_info(priv->dev, "Rx PROD/CONS     | 0x%x/0x%x (Diff: %d)\n",
		    priv->rx_prod, priv->rx_cons, (priv->rx_prod - priv->rx_cons));
}

static void xtile_seq_no_dump_tx(struct altera_dma_private *priv)
{
	s32 ret;

	ret = csrrd32(priv->tx_dma_csr, msgdma_csroffs(rw_seq_num));
	netdev_info(priv->dev, "Tx READ SEQ NO   | 0x%x\n", (ret & 0x0000fffff));
	netdev_info(priv->dev, "Tx WRITE SEQ NO  | 0x%x\n", (ret & 0xffff0000) >> 16);
}

static void xtile_seq_no_dump_rx(struct altera_dma_private *priv)
{
	s32 ret;

	ret = csrrd32(priv->rx_dma_csr, msgdma_csroffs(rw_seq_num));
	netdev_info(priv->dev, "Rx READ SEQ NO   | 0x%x\n", (ret & 0x0000fffff));
	netdev_info(priv->dev, "Rx WRITE SEQ NO  | 0x%x\n", (ret & 0xffff0000) >> 16);
}

static void xtile_comp_version_rx(struct altera_dma_private *priv)
{
	s32 ret;

	ret = csrrd32(priv->rx_dma_csr, msgdma_csroffs(pad[2]));
	netdev_info(priv->dev, "Rx COMP TYPE_VERSION 0x%x\n", ret);
}

static void xtile_comp_version_tx(struct altera_dma_private *priv)
{
	s32 ret;

	ret = csrrd32(priv->tx_dma_csr, msgdma_csroffs(pad[2]));
	netdev_info(priv->dev, "Tx COMP TYPE_VERSION 0x%x\n", ret);
}

static void xtile_unprocess_desc_tx(struct altera_dma_private *priv)
{
	u32 index;
	u32 sw_owned = 0;
	u32 hw_owned = 0;
	u32 desc_ringsize = priv->tx_ring_size * 2;

	for (index = 0; index < desc_ringsize; index++) {
		if (priv->pref_txdesc[index].desc_control &
				MSGDMA_PREF_DESC_CTL_OWNED_BY_HW) {
			hw_owned++;
		} else {
			sw_owned++;
		}
	}

	netdev_info(priv->dev, "SW Owned: %d HW Owned: %d\n", sw_owned, hw_owned);
}

static void xtile_dma_regs(struct altera_dma_private *priv)
{
	xtile_comp_version_tx(priv);
	xtile_seq_no_dump_tx(priv);
	xtile_process_seq_no_tx(priv);
	xtile_fifo_fill_level_tx(priv);
	xtile_dispatcher_reg_dump_tx(priv);
	xtile_prefetcher_reg_dump_tx(priv);

	netdev_info(priv->dev, "<==========================================>\n");

	xtile_comp_version_rx(priv);
	xtile_seq_no_dump_rx(priv);
	xtile_process_seq_no_rx(priv);
	xtile_dispatcher_reg_dump_rx(priv);
	xtile_prefetcher_reg_dump_rx(priv);
	xtile_fifo_fill_level_rx(priv);
}

static ssize_t msgdma_reg_dump_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct net_device *ndev = platform_get_drvdata(pdev);
	intel_fpga_xtile_eth_private *priv = netdev_priv(ndev);
	int queue;

	for (queue = 0; queue < priv->num_channels; queue++)
		xtile_dma_regs(&priv->dma_info[queue].dma_priv);

	return sprintf(buf, "%x", 1);
}

static ssize_t msgdma_tx_irq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct net_device *ndev = platform_get_drvdata(pdev);
	intel_fpga_xtile_eth_private *priv = netdev_priv(ndev);
	bool intr_state;
	int queue;

	for (queue = 0; queue < priv->num_channels; queue++) {
		intr_state = priv->spec_ops->dma_ops->is_txirq_set(&priv->dma_info[queue].dma_priv);

		xtile_dispatcher_reg_dump_tx(&priv->dma_info[queue].dma_priv);
		xtile_prefetcher_reg_dump_tx(&priv->dma_info[queue].dma_priv);

		netdev_info(priv->dev,
			    "TX Intr state - %x. Rx- %d Tx - %d Napi state: %d Cntr: %lld %lld %lld %lld",
			    intr_state, priv->dma_info[queue].rx_irq_enabled,
				priv->dma_info[queue].tx_irq_enabled,
				priv->dma_info[queue].napi_state,
			    priv->dma_info[queue].irq_rx_enable_cntr,
				priv->dma_info[queue].irq_tx_enable_cntr,
			    priv->dma_info[queue].irq_rx_disable_cntr,
				priv->dma_info[queue].irq_tx_disable_cntr);

		intr_state = priv->spec_ops->dma_ops->is_rxirq_set(&priv->dma_info[queue].dma_priv);
		xtile_dispatcher_reg_dump_rx(&priv->dma_info[queue].dma_priv);
		xtile_prefetcher_reg_dump_rx(&priv->dma_info[queue].dma_priv);

		sprintf(buf, "RX Intr state: %x", intr_state);
		netdev_info(priv->dev, "%s", buf);
	}
	return 1;
}

static ssize_t msgdma_tx_irq_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t len)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct net_device *ndev = platform_get_drvdata(pdev);
	intel_fpga_xtile_eth_private *priv = netdev_priv(ndev);
	unsigned long flags;

	int values[2];
	int i = 0;
	char *token;
	char *input_copy;
	char *delimiter = " ";
	char *saveptr;

	// Make a copy of the input string because strsep modifies the input
	input_copy = kstrdup(buf, GFP_KERNEL);

	if (!input_copy) {
		netdev_info(priv->dev, "Failed to allocate memory for input copy\n");
		return 0;
	}

	// Initialize saveptr to the copied input string
	saveptr = input_copy;

	// Tokenize the string using ',' as the delimiter
	while ((token = strsep(&saveptr, delimiter)) != NULL) {
		// Process each token as a number (assuming they are numeric strings)
		if (kstrtoint(token, 10, &values[i]) == 0) {
			// Successfully converted token to an integer
		} else {
			// Failed to convert token to an integer
			netdev_info(priv->dev, "Invalid number: %s\n", token);
		}
		i++;
	}

	kfree(input_copy);  // Free the allocated memory

	if (values[1] == 0) {
		priv->spec_ops->dma_ops->disable_txirq(&priv->dma_info[values[0]].dma_priv);
		priv->spec_ops->dma_ops->clear_txirq(&priv->dma_info[values[0]].dma_priv);
		disable_irq(priv->dma_info[values[0]].tx_irq);
		netdev_info(priv->dev, "IRQ Disabled\n");
	} else if (values[1] == 1) {
		enable_irq(priv->dma_info[values[0]].tx_irq);
		spin_lock_irqsave(&priv->dma_info[values[0]].rxdma_irq_lock, flags);
		priv->spec_ops->dma_ops->clear_txirq(&priv->dma_info[values[0]].dma_priv);
		priv->spec_ops->dma_ops->enable_txirq(&priv->dma_info[values[0]].dma_priv);
		spin_unlock_irqrestore(&priv->dma_info[values[0]].rxdma_irq_lock, flags);
		netdev_info(priv->dev, "IRQ Enabled\n");
	}
	return 1;
}

static ssize_t msgdma_tx_desc_dump_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct net_device *ndev = platform_get_drvdata(pdev);
	intel_fpga_xtile_eth_private *priv = netdev_priv(ndev);
	int queue;

	for (queue = 0; queue < priv->num_channels; queue++)
		xtile_unprocess_desc_tx(&priv->dma_info[queue].dma_priv);

	return sprintf(buf, "%x", 1);
}

static ssize_t link_state_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct net_device *ndev = platform_get_drvdata(pdev);
	intel_fpga_xtile_eth_private *priv = netdev_priv(ndev);

	netdev_info(priv->dev,
		    "Cable is %s\n", priv->cable_unplugged ?
		    "not connected" : "connected");

	hssi_ethport_is_stable(priv->pdev_hssi, priv->hssi_port, true);

	return sprintf(buf, "%x", 1);
}

static ssize_t ui_interval_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct net_device *ndev = platform_get_drvdata(pdev);
	intel_fpga_xtile_eth_private *priv = netdev_priv(ndev);

	return sprintf(buf, "%u\n", priv->ui_adjust_interval);
}

static ssize_t ui_interval_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct net_device *ndev = platform_get_drvdata(pdev);
	intel_fpga_xtile_eth_private *priv = netdev_priv(ndev);
	int value;
	int ret;

	ret = kstrtouint(buf, 10, &value);
	if (ret < 0)
		return ret;

	priv->ui_adjust_interval = value;
	return len;
}

static ssize_t en_dis_sec_ip_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t len)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct net_device *ndev = platform_get_drvdata(pdev);
	intel_fpga_xtile_eth_private *priv = netdev_priv(ndev);
	int value;
	int ret;

	ret = kstrtouint(buf, 10, &value);
	if (ret < 0)
		return ret;

	if (priv->ptp_priv->pps_ctrl) {
		csrwr32(value, priv->ptp_priv->pps_ctrl, pps_csroffs(pps_ctrl));
		schedule_pll_lock_check(priv->ptp_priv->ptp_freq_priv);
	} else {
		netdev_info(priv->dev, "pps ctrl is not supported");
	}

	return len;
}

static ssize_t eth_poll_monitoring_interval_show(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct net_device *ndev = platform_get_drvdata(pdev);
	intel_fpga_xtile_eth_private *priv = netdev_priv(ndev);

	return sprintf(buf, "%u\n", priv->monitor_poll_interval);
}

static ssize_t eth_poll_monitoring_interval_store(struct device *dev,
						  struct device_attribute *attr,
						  const char *buf, size_t len)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct net_device *ndev = platform_get_drvdata(pdev);
	intel_fpga_xtile_eth_private *priv = netdev_priv(ndev);
	int value;
	int ret;

	ret = kstrtouint(buf, 10, &value);
	if (ret < 0)
		return ret;

	priv->monitor_poll_interval = value;

	return len;
}

static DEVICE_ATTR(msgdma_reg_dump, 0644, msgdma_reg_dump_show, NULL);
static DEVICE_ATTR(msgdma_tx_desc_dump, 0644, msgdma_tx_desc_dump_show, NULL);
static DEVICE_ATTR_RW(msgdma_tx_irq);
static DEVICE_ATTR(link_state, 0644, link_state_show, NULL);
static DEVICE_ATTR_RW(ui_interval);
static DEVICE_ATTR(en_dis_sec_ip, 0644, NULL, en_dis_sec_ip_store);
static DEVICE_ATTR(eth_poll_monitoring_interval, 0644,
		   eth_poll_monitoring_interval_show,
		   eth_poll_monitoring_interval_store);

static struct attribute *msgdma_sysfs_attrs[] = {
	&dev_attr_msgdma_reg_dump.attr,
	&dev_attr_msgdma_tx_desc_dump.attr,
	&dev_attr_msgdma_tx_irq.attr,
	&dev_attr_link_state.attr,
	&dev_attr_ui_interval.attr,
	&dev_attr_en_dis_sec_ip.attr,
	&dev_attr_eth_poll_monitoring_interval.attr,
	NULL
};

static const struct attribute_group msgdma_attr_group = {
	.attrs = msgdma_sysfs_attrs,
};

const struct attribute_group *msgdma_attr_groups[] = {
	&msgdma_attr_group,
	NULL
};

