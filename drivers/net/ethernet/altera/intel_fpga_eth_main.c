// SPDX-License-Identifier: GPL-2.0
/* Altera FPGA Ethernet MAC driver
 * Copyright (C) 2022,2025 Altera Corporation. All rights reserved
 *
 * Contributors:
 *	Preetam Narayan
 *
 */

#include <linux/etherdevice.h>
#include <linux/if_ether.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/if_vlan.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/net_tstamp.h>
#include <linux/netdevice.h>
#include <linux/of_device.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/phylink.h>
#include <linux/skbuff.h>
#include "intel_fpga_eth_main.h"
#include "intel_fpga_ftile_driver.h"
#include "intel_fpga_etile_driver.h"
#include "intel_fpga_gts_driver.h"
#include "intel_fpga_eth_hssi_itf.h"
#include "intel_fpga_eth_tile_ops.h"
#include <linux/sched.h>

/* Module parameters */
static int debug = -1;

module_param(debug, int, MOD_PARAM_PERM);
MODULE_PARM_DESC(debug,
		 "Message Level (-1: default, 0: no output, 16: all)");

static const u32 default_msg_level =  NETIF_MSG_PROBE  |
				      NETIF_MSG_LINK   |
				      NETIF_MSG_IFUP   |
				      NETIF_MSG_RX_ERR |
				      NETIF_MSG_TX_ERR |
				      NETIF_MSG_IFDOWN;

static int flow_ctrl = FLOW_TX | FLOW_RX;

module_param(flow_ctrl, int, MOD_PARAM_PERM);
MODULE_PARM_DESC(flow_ctrl,
		 "Flow control (0: off, 1: rx, 2: tx, 3: on)");

static int pause = MAC_PAUSEFRAME_QUANTA;
module_param(pause, int, MOD_PARAM_PERM);
MODULE_PARM_DESC(pause, "Flow Control Pause Time");

extern const struct attribute_group *msgdma_attr_groups[];

#define RX_DESCRIPTORS 512
static int dma_rx_num = RX_DESCRIPTORS;
module_param(dma_rx_num, int, 0644);
MODULE_PARM_DESC(dma_rx_num, "Number of descriptors in the RX list");

#define TX_DESCRIPTORS 512
static int dma_tx_num = TX_DESCRIPTORS;
module_param(dma_tx_num, int, 0644);
MODULE_PARM_DESC(dma_tx_num, "Number of descriptors in the TX list");

/* Make sure DMA buffer size is larger than the max frame size
 * plus some alignment offset and a VLAN header. If the max frame size is
 * 1518, a VLAN header would be additional 4 bytes and additional
 * headroom for alignment is 2 bytes, 2048 is just fine.
 */
#define INTEL_FPGA_RXDMABUFFER_SIZE	2048
#define INTEL_FPGA_COAL_TIMER(x)	(jiffies + usecs_to_jiffies(x))

/* Allow network stack to resume queueing packets after we've
 * finished transmitting at least 1/4 of the packets in the queue.
 */
#define ETH_TX_THRESH(x, i)	((x)->dma_info[(i)].dma_priv.tx_ring_size / 4)
#define ETH_TX_THRESH_DMA(x)	((x)->dma_priv.tx_ring_size / 4)

#define TXQUEUESTOP_THRESHOLD	2

static const struct of_device_id intel_fpga_xtile_ll_ids[];

/* Enable/Disable the Tx interrupt if it is not already enabled to avoid interrupt stacking */
static inline void xtile_modify_cpu_txintr_state(struct intel_fpga_xtile_eth_private *priv,
						 bool enable, int queue)
{
	bool lc_txirq = false;
	unsigned long flags;

	spin_lock_irqsave(&priv->dma_info[queue].rxdma_irq_lock, flags);

	if (priv->dma_info[queue].tx_irq_enabled != enable) {
		priv->dma_info[queue].tx_irq_enabled = enable;
		lc_txirq = true;
	}

	spin_unlock_irqrestore(&priv->dma_info[queue].rxdma_irq_lock, flags);

	if (lc_txirq) {
		if (enable) {
			enable_irq(priv->dma_info[queue].tx_irq);
			priv->dma_info[queue].irq_tx_enable_cntr++;
		} else {
			disable_irq(priv->dma_info[queue].tx_irq);
			priv->dma_info[queue].irq_tx_disable_cntr++;
		}
	}
}

/* Enable/Disable the Rx interrupt to avoid interrupt disable stacking */
static inline void xtile_modify_cpu_rxintr_state(struct intel_fpga_xtile_eth_private *priv,
						 bool enable, int queue)
{
	bool lc_rxirq = false;
	unsigned long flags;

	spin_lock_irqsave(&priv->dma_info[queue].rxdma_irq_lock, flags);

	if (priv->dma_info[queue].rx_irq_enabled != enable) {
		priv->dma_info[queue].rx_irq_enabled = enable;
		lc_rxirq = true;
	}

	spin_unlock_irqrestore(&priv->dma_info[queue].rxdma_irq_lock, flags);

	if (lc_rxirq) {
		if (enable) {
			enable_irq(priv->dma_info[queue].rx_irq);
			priv->dma_info[queue].irq_rx_enable_cntr++;
		} else {
			disable_irq(priv->dma_info[queue].rx_irq);
			priv->dma_info[queue].irq_rx_disable_cntr++;
		}
	}
}

/* Wrapper API to be used for the CPU interrupt management to avoid interrupt disable stacking */
static inline void xtile_modify_cpu_intr_state(struct intel_fpga_xtile_eth_private *priv,
					       bool enable, int queue)
{
	xtile_modify_cpu_txintr_state(priv, enable, queue);
	xtile_modify_cpu_rxintr_state(priv, enable, queue);
}

static inline void xtile_modify_cpu_enable_intr(struct intel_fpga_xtile_eth_private *priv,
						int queue)
{
	xtile_modify_cpu_intr_state(priv, true, queue);
}

static inline void xtile_modify_cpu_disable_intr(struct intel_fpga_xtile_eth_private *priv,
						 int queue)
{
	xtile_modify_cpu_intr_state(priv, false, queue);
}

/* Enable/Disable the Tx interrupt if it is not already enabled to avoid interrupt stacking */
static inline void xtile_modify_cpu_txintr_state_per_dma(struct intel_xtile_msgdma_info *dma,
							 bool enable)
{
	bool lc_txirq = false;
	unsigned long flags;

	spin_lock_irqsave(&dma->rxdma_irq_lock, flags);

	if (dma->tx_irq_enabled != enable) {
		dma->tx_irq_enabled = enable;
		lc_txirq = true;
	}

	spin_unlock_irqrestore(&dma->rxdma_irq_lock, flags);

	if (lc_txirq) {
		if (enable) {
			enable_irq(dma->tx_irq);
			dma->irq_tx_enable_cntr++;
		} else {
			disable_irq(dma->tx_irq);
			dma->irq_tx_disable_cntr++;
		}
	}
}

/* Enable/Disable the Rx interrupt to avoid interrupt disable stacking */
static inline void xtile_modify_cpu_rxintr_state_per_dma(struct intel_xtile_msgdma_info *dma,
							 bool enable)
{
	bool lc_rxirq = false;
	unsigned long flags;

	spin_lock_irqsave(&dma->rxdma_irq_lock, flags);

	if (dma->rx_irq_enabled != enable) {
		dma->rx_irq_enabled = enable;
		lc_rxirq = true;
	}

	spin_unlock_irqrestore(&dma->rxdma_irq_lock, flags);

	if (lc_rxirq) {
		if (enable) {
			enable_irq(dma->rx_irq);
			dma->irq_rx_enable_cntr++;
		} else {
			disable_irq(dma->rx_irq);
			dma->irq_rx_disable_cntr++;
		}
	}
}

/* Wrapper API to be used for the CPU interrupt management to avoid interrupt disable stacking */
static inline void xtile_modify_cpu_intr_state_per_dma(struct intel_xtile_msgdma_info *dma,
						       bool enable)
{
	xtile_modify_cpu_txintr_state_per_dma(dma, enable);
	xtile_modify_cpu_rxintr_state_per_dma(dma, enable);
}

static inline void xtile_modify_cpu_enable_intr_per_dma(struct intel_xtile_msgdma_info *dma)
{
	xtile_modify_cpu_intr_state_per_dma(dma, true);
}

static inline void xtile_modify_cpu_disable_intr_per_dma(struct intel_xtile_msgdma_info *dma)
{
	xtile_modify_cpu_intr_state_per_dma(dma, false);
}

static inline void xtile_txdmaintr_modify(struct intel_fpga_xtile_eth_private *priv,
					  bool enable, int queue)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->dma_info[queue].rxdma_irq_lock, flags);
	priv->spec_ops->dma_ops->clear_txirq(&priv->dma_info[queue].dma_priv);

	if (enable)
		priv->spec_ops->dma_ops->enable_txirq(&priv->dma_info[queue].dma_priv);
	else
		priv->spec_ops->dma_ops->disable_txirq(&priv->dma_info[queue].dma_priv);
	spin_unlock_irqrestore(&priv->dma_info[queue].rxdma_irq_lock, flags);
}

static inline void xtile_rxdmaintr_modify(struct intel_fpga_xtile_eth_private *priv,
					  bool enable, int queue)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->dma_info[queue].rxdma_irq_lock, flags);
	priv->spec_ops->dma_ops->clear_rxirq(&priv->dma_info[queue].dma_priv);
	if (enable)
		priv->spec_ops->dma_ops->enable_rxirq(&priv->dma_info[queue].dma_priv);
	else
		priv->spec_ops->dma_ops->disable_rxirq(&priv->dma_info[queue].dma_priv);
	spin_unlock_irqrestore(&priv->dma_info[queue].rxdma_irq_lock, flags);
}

static inline void xtile_dmaintr_enable(struct intel_fpga_xtile_eth_private *priv,
					int queue)
{
	xtile_txdmaintr_modify(priv, true, queue);
	xtile_rxdmaintr_modify(priv, true, queue);
}

static inline void xtile_dmaintr_disable(struct intel_fpga_xtile_eth_private *priv,
					 int queue)
{
	xtile_txdmaintr_modify(priv, false, queue);
	xtile_rxdmaintr_modify(priv, false, queue);
}

static int xtile_fec_init(struct platform_device *pdev, struct intel_fpga_xtile_eth_private *priv)
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
	if (of_property_read_u32(pdev->dev.of_node, "fec-cw-pos-rx",
				 &priv->rsfec_cw_pos_rx)) {
		dev_err(&pdev->dev, "cannot obtain fec codeword bit position!\n");
		return -ENXIO;
	}
	dev_info(&pdev->dev, "\trsfec rx codeword bit position is 0x%x\n",
		 priv->rsfec_cw_pos_rx);

	/* get UI adjust interval from device tree */
	if (of_property_read_u32(pdev->dev.of_node, "ui-adj-interval",
				 &priv->ui_adjust_interval)) {
		dev_err(&pdev->dev, "cannot obtain UI adjust interval value!\n");
		priv->ui_adjust_interval =  1000;
	}
	dev_info(&pdev->dev, "\tui_adjust interval gap is %d\n",
		 priv->ui_adjust_interval);

	return 0;
}

static inline u32 xtile_tx_avail(struct intel_fpga_xtile_eth_private *priv, int queue)
{
	return priv->dma_info[queue].dma_priv.tx_cons + priv->dma_info[queue].dma_priv.tx_ring_size
		- priv->dma_info[queue].dma_priv.tx_prod - 1;
}

static inline u32 xtile_tx_avail_per_dma(struct intel_xtile_msgdma_info *dma)
{
	return dma->dma_priv.tx_cons + dma->dma_priv.tx_ring_size
		- dma->dma_priv.tx_prod - 1;
}

static int xtile_init_rx_buffer(struct intel_fpga_xtile_eth_private *priv,
				struct altera_dma_buffer *rxbuffer,
				int len)
{
       rxbuffer->skb = netdev_alloc_skb(priv->dev, len);
       skb_reserve(rxbuffer->skb, SKB_DMA_REALIGN);

       if (!rxbuffer->skb)
                return -ENOMEM;

	rxbuffer->dma_addr = dma_map_single(priv->device,
					    rxbuffer->skb->data,
					    len, DMA_FROM_DEVICE);

	if (dma_mapping_error(priv->device, rxbuffer->dma_addr)) {
		netdev_err(priv->dev,
			   "%s: DMA mapping error\n", __func__);

		dev_kfree_skb_any(rxbuffer->skb);

		return -EINVAL;
	}

	rxbuffer->len = len;

	return 0;
}

static void xtile_free_rx_buffer(struct intel_fpga_xtile_eth_private *priv,
				 struct altera_dma_buffer *rxbuffer)
{
	struct sk_buff *skb = rxbuffer->skb;
	dma_addr_t dma_addr = rxbuffer->dma_addr;

	if (skb) {
		if (dma_addr)
			dma_unmap_single(priv->device, dma_addr,
					 rxbuffer->len,
					 DMA_FROM_DEVICE);
		
		dev_consume_skb_any(skb);
		rxbuffer->skb = NULL;
		rxbuffer->dma_addr = 0;
	}
}

/* Unmap and free Tx buffer resources
 */
static void xtile_free_tx_buffer(struct intel_fpga_xtile_eth_private *priv,
				 struct altera_dma_buffer *buffer)
{
	if (buffer->dma_addr) {
		if (buffer->mapped_as_page)
			dma_unmap_page(priv->device, buffer->dma_addr,
				       buffer->len, DMA_TO_DEVICE);
		else
			dma_unmap_single(priv->device, buffer->dma_addr,
					 buffer->len, DMA_TO_DEVICE);
		buffer->dma_addr = 0;
	}
	if (buffer->skb) {
		dev_consume_skb_any(buffer->skb);
		buffer->skb = NULL;
	}
}

static int xtile_alloc_init_skbufs(struct intel_fpga_xtile_eth_private *priv, int queue)
{
	unsigned int rx_descs = priv->dma_info[queue].dma_priv.rx_ring_size;
	unsigned int tx_descs = priv->dma_info[queue].dma_priv.tx_ring_size;
	int ret = -ENOMEM;
	int i;

	/* Create Rx ring buffer */
	priv->dma_info[queue].dma_priv.rx_ring = kcalloc(rx_descs,
							 sizeof(struct altera_dma_buffer),
							 GFP_KERNEL);
	if (!priv->dma_info[queue].dma_priv.rx_ring)
		goto err_rx_ring;

	/* Create Tx ring buffer */
	priv->dma_info[queue].dma_priv.tx_ring = kcalloc(tx_descs,
							 sizeof(struct altera_dma_buffer),
							 GFP_KERNEL);
	if (!priv->dma_info[queue].dma_priv.tx_ring)
		goto err_tx_ring;

	priv->dma_info[queue].dma_priv.tx_cons = 0;
	priv->dma_info[queue].dma_priv.tx_prod = 0;

	/* Init Rx FIFO */
	csrwr32(priv->dma_info[queue].rx_fifo_almost_full, priv->dma_info[queue].rx_fifo,
		rx_fifo_csroffs(almost_full_threshold));

	csrwr32(priv->dma_info[queue].rx_fifo_almost_empty, priv->dma_info[queue].rx_fifo,
		rx_fifo_csroffs(almost_empty_threshold));

	/* Init Rx ring */
	for (i = 0; i < rx_descs; i++) {
		ret = xtile_init_rx_buffer(priv, &priv->dma_info[queue].dma_priv.rx_ring[i],
					   priv->dma_info[queue].dma_priv.rx_dma_buf_sz);
		if (ret)
			goto err_init_rx_buffers;
	}

	priv->dma_info[queue].dma_priv.rx_cons = 0;
	priv->dma_info[queue].dma_priv.rx_prod = 0;

	return 0;

err_init_rx_buffers:
	while (--i >= 0)
		xtile_free_rx_buffer(priv, &priv->dma_info[queue].dma_priv.rx_ring[i]);

	kfree(priv->dma_info[queue].dma_priv.tx_ring);
err_tx_ring:
	kfree(priv->dma_info[queue].dma_priv.rx_ring);
err_rx_ring:
	return ret;
}

static void xtile_free_skbufs(struct net_device *dev)
{
	struct intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	int queue = 0;
	unsigned int rx_descs = 0;
	unsigned int tx_descs = 0;
	int i;

	for (queue = 0; queue < priv->num_channels; queue++) {
		rx_descs = priv->dma_info[queue].dma_priv.rx_ring_size;
		tx_descs = priv->dma_info[queue].dma_priv.tx_ring_size;

		/* Release the DMA TX/RX socket buffers */
		for (i = 0; i < rx_descs; i++) {
			if (priv->dma_info[queue].dma_priv.rx_ring)
				xtile_free_rx_buffer(priv,
						     &priv->dma_info[queue].dma_priv.rx_ring[i]);
		}

		for (i = 0; i < tx_descs; i++) {
			if (priv->dma_info[queue].dma_priv.tx_ring)
				xtile_free_tx_buffer(priv,
						     &priv->dma_info[queue].dma_priv.tx_ring[i]);
		}

		kfree(priv->dma_info[queue].dma_priv.tx_ring);
		kfree(priv->dma_info[queue].dma_priv.rx_ring);
	}
}

/* Reallocate the skb for the reception process
 */
static inline void xtile_rx_refill(struct intel_xtile_msgdma_info *dma)
{
	struct intel_fpga_xtile_eth_private *priv = dma->priv;
	unsigned int rxsize = dma->dma_priv.rx_ring_size;
	unsigned int entry;
	int ret;

	for (; dma->dma_priv.rx_cons - dma->dma_priv.rx_prod > 0;
			dma->dma_priv.rx_prod++) {
		entry = dma->dma_priv.rx_prod % rxsize;
		if (likely(!dma->dma_priv.rx_ring[entry].skb)) {
			ret = xtile_init_rx_buffer(priv,
						   &dma->dma_priv.rx_ring[entry],
						   dma->dma_priv.rx_dma_buf_sz);
			if (unlikely(ret != 0))
				break;
			priv->spec_ops->dma_ops->add_rx_desc(&dma->dma_priv,
					&dma->dma_priv.rx_ring[entry]);
		}
	}
}

/* Pull out the VLAN tag and fix up the packet
 */
static inline void xtile_rx_vlan(struct net_device *dev, struct sk_buff *skb)
{
	struct ethhdr *eth_hdr;
	u16 vid;

	if ((dev->features & NETIF_F_HW_VLAN_CTAG_RX) &&
	    !__vlan_get_tag(skb, &vid)) {
		eth_hdr = (struct ethhdr *)skb->data;
		memmove(skb->data + VLAN_HLEN, eth_hdr, ETH_ALEN * 2);
		skb_pull(skb, VLAN_HLEN);
		__vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q), vid);
	}
}

/* Receive a packet: retrieve and pass over to upper levels
 */
static int xtile_rx(struct intel_xtile_msgdma_info *dma, int limit)
{
	struct intel_fpga_xtile_eth_private *priv = dma->priv;
	unsigned int count = 0;
	unsigned int next_entry;
	struct sk_buff *skb;
	unsigned int entry =
		dma->dma_priv.rx_cons % dma->dma_priv.rx_ring_size;
	u32 rxstatus;
	u16 pktlength;
	u16 pktstatus;

	while ((count < limit) &&
	       ((rxstatus =
		  priv->spec_ops->dma_ops->get_rx_status(&dma->dma_priv)) != 0)) {
		pktstatus = rxstatus >> 16;
		pktlength = rxstatus & 0xffff;

		skb = dma->dma_priv.rx_ring[entry].skb;
		if (unlikely(!skb)) {
			netdev_err(priv->dev,
				   "%s: Inconsistent Rx descriptor chain\n",
				   __func__);
			priv->dev->stats.rx_dropped++;
			break;
		}

		count++;
		next_entry = (++dma->dma_priv.rx_cons)
			      % dma->dma_priv.rx_ring_size;

		dma->dma_priv.rx_ring[entry].skb = NULL;
		skb_put(skb, pktlength);

		/* make cache consistent with receive packet buffer */
		dma_sync_single_for_cpu(priv->device,
					dma->dma_priv.rx_ring[entry].dma_addr,
					dma->dma_priv.rx_ring[entry].len,
					DMA_FROM_DEVICE);

		dma_unmap_single(priv->device,
				 dma->dma_priv.rx_ring[entry].dma_addr,
				 dma->dma_priv.rx_ring[entry].len,
				 DMA_FROM_DEVICE);

		if (unlikely(netif_msg_pktdata(priv))) {
			netdev_info(priv->dev, "frame received %d bytes\n",
				    pktlength);

			print_hex_dump(KERN_ERR, "data: ", DUMP_PREFIX_OFFSET,
				       16, 1, skb->data, pktlength, true);
		}

		xtile_rx_vlan(priv->dev, skb);
		skb->protocol = eth_type_trans(skb, priv->dev);
		skb_checksum_none_assert(skb);
		napi_gro_receive(&dma->napi, skb);
		priv->dev->stats.rx_packets++;
		priv->dev->stats.rx_bytes += pktlength;
		entry = next_entry;
		xtile_rx_refill(dma);
	}

	return count;
}

/* Reclaim resources after transmission completes
 */
static int xtile_tx_complete(struct intel_xtile_msgdma_info *dma)
{
	struct intel_fpga_xtile_eth_private *priv =
		(struct intel_fpga_xtile_eth_private *)dma->priv;
	unsigned int txsize = dma->dma_priv.tx_ring_size;
	u32 ready;
	unsigned int entry;
	struct altera_dma_buffer *tx_buff;
	int txcomplete = 0;
	struct netdev_queue *txq;

	spin_lock(&dma->tx_lock);
	ready = priv->spec_ops->dma_ops->tx_completions(&dma->dma_priv);

	/* Free sent buffers */
	while (ready && (dma->dma_priv.tx_cons != dma->dma_priv.tx_prod)) {
		entry = dma->dma_priv.tx_cons % txsize;
		tx_buff = &dma->dma_priv.tx_ring[entry];

		if (likely(tx_buff->skb))
			priv->dev->stats.tx_packets++;

		if (netif_msg_tx_done(priv))
			netdev_info(priv->dev, "%s: curr %d, dirty %d\n",
				    __func__, dma->dma_priv.tx_prod,
				    dma->dma_priv.tx_cons);

		xtile_free_tx_buffer(priv, tx_buff);
		dma->dma_priv.tx_cons++;

		txcomplete++;
		ready--;
	}

	txq = netdev_get_tx_queue(priv->dev, dma->queue);
	if (unlikely(netif_tx_queue_stopped(txq) &&
		     xtile_tx_avail_per_dma(dma) > ETH_TX_THRESH_DMA(dma))) {
		netif_tx_lock(priv->dev);
		if (netif_msg_tx_done(priv))
			netdev_info(priv->dev, "restart transmit");
		netif_tx_wake_queue(txq);
		netif_tx_unlock(priv->dev);
	}

	spin_unlock(&dma->tx_lock);

	return txcomplete;
}

static u16 xtile_select_queue(struct net_device *dev, struct sk_buff *skb,
			      struct net_device *sb_dev)
{
	int traffic_class = 0;

	if (dev->real_num_tx_queues == 1)
		return 0;

	traffic_class = skb->priority;
	// Assuming you have a one-to-one mapping between traffic classes and queues
	if (traffic_class < dev->real_num_tx_queues)
		return traffic_class;
	else
		return (dev->real_num_tx_queues - 1);
}

/* NAPI polling function
 * Ref: from napi_schedule to poll call = 20us
 */
static int xtile_poll(struct napi_struct *napi, int budget)
{
	struct intel_xtile_msgdma_info *dma = container_of(napi,
							   struct intel_xtile_msgdma_info, napi);
	struct intel_fpga_xtile_eth_private *priv =
		(struct intel_fpga_xtile_eth_private *)dma->priv;
	int rxcomplete, txcomplete, min_run;
	int credits = budget;
	unsigned long flags;
	bool irq_state;

	if (credits == 0)
		/* as per the spec only process only Tx */
		credits = NAPI_POLL_WEIGHT;

	/* txcomplete should be ideally the no. of units of work
	 * used up from the budget but it is impacting the performance
	 *  Handling reserved for future and all packets in ring ready
	 *  for processing are handled
	 */

	priv->spec_ops->dma_ops->clear_txirq(&dma->dma_priv);
	txcomplete = xtile_tx_complete(dma);

	/* Handle case where we are called by netpoll with a budget of 0 */
	if (unlikely(budget <= 0))
		goto repoll;

	priv->spec_ops->dma_ops->clear_rxirq(&dma->dma_priv);
	rxcomplete = xtile_rx(dma, credits);

	min_run = min_t(int, txcomplete + rxcomplete, budget);

	spin_lock_irqsave(&dma->rxdma_irq_lock, flags);
	irq_state = priv->spec_ops->dma_ops->is_txirq_set(&dma->dma_priv) ||
		priv->spec_ops->dma_ops->is_rxirq_set(&dma->dma_priv);
	spin_unlock_irqrestore(&dma->rxdma_irq_lock, flags);

	if (irq_state) {
		/* In case the tx/rx interrupt is raised,
		 * we clear it and request to repoll
		 */
		priv->spec_ops->dma_ops->clear_rxirq(&dma->dma_priv);
		priv->spec_ops->dma_ops->clear_txirq(&dma->dma_priv);
		goto repoll;
	}

	/* This case implies that there is possibility of some packets
	 * unprocessed in the Rx DMA buffer
	 */
	if (min_run == credits)
		goto repoll;

	/* enable the interrupt to CPU only if the napi complete done */
	if (napi_complete_done(napi, min_run))
		xtile_modify_cpu_enable_intr_per_dma(dma);
	else
		netdev_dbg(priv->dev, "napi complete failed");

	/* Amount of Rx work done is returned */
	if (unlikely(netif_msg_intr(priv)))
		netdev_err(priv->dev, "TX/RX complete: %d/%d %d/%d\n",
			   txcomplete, rxcomplete, min_run, budget);

	return min_run;
repoll:
	return budget;
}

static int intel_get_queue_num_from_isr(struct intel_fpga_xtile_eth_private *priv, int irq)
{
	int i = 0;

	for (i = 0; i < priv->num_channels; i++) {
		if (priv->dma_info[i].tx_irq == irq ||
		    priv->dma_info[i].rx_irq == irq)
			return i;
	}
	return MAX_DMA_CHANNELS;
}

/* DMA TX & RX FIFO interrupt routing
 * Exec: 6 us
 */
static irqreturn_t intel_fpga_xtile_isr(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct intel_fpga_xtile_eth_private *priv;
	int queue = 0;

	if (unlikely(!dev)) {
		pr_err("%s: invalid dev_id\n", __func__);
		return IRQ_NONE;
	}

	priv = netdev_priv(dev);

	//get dma queue number from the irq number
	queue = intel_get_queue_num_from_isr(priv, irq);

	if (unlikely(netif_msg_intr(priv)))
		netdev_info(dev, "Q: %d %s interrupt %d\n", queue,
			    ((irq == priv->dma_info[queue].rx_irq) ? "RX" : "TX"), irq);

	spin_lock(&priv->dma_info[queue].rxdma_irq_lock);

	if (likely(napi_schedule_prep(&priv->dma_info[queue].napi))) {
		if (priv->dma_info[queue].rx_irq_enabled) {
			priv->dma_info[queue].rx_irq_enabled = false;
			disable_irq_nosync(priv->dma_info[queue].rx_irq);
			priv->dma_info[queue].irq_rx_disable_cntr++;
		}
		if (priv->dma_info[queue].tx_irq_enabled) {
			priv->dma_info[queue].tx_irq_enabled = false;
			disable_irq_nosync(priv->dma_info[queue].tx_irq);
			priv->dma_info[queue].irq_tx_disable_cntr++;
		}
		__napi_schedule(&priv->dma_info[queue].napi);
	}

	priv->spec_ops->dma_ops->clear_rxirq(&priv->dma_info[queue].dma_priv);
	priv->spec_ops->dma_ops->clear_txirq(&priv->dma_info[queue].dma_priv);

	spin_unlock(&priv->dma_info[queue].rxdma_irq_lock);
	return IRQ_HANDLED;
}

int xtile_check_counter_complete(struct intel_fpga_xtile_eth_private *priv, u32 regbank,
				 size_t offs, u8 bit_mask, bool set_bit,
				 int align)
{
	int counter;
	u32 chan = priv->tile_chan;
	struct platform_device *pdev = priv->pdev_hssi;

	counter = 0;
	switch (align) {
	case 8: /* byte aligned */
		while (counter++ < INTEL_FPGA_XTILE_CNTR_CHECK) {
			if (set_bit) {
				if (hssi_csrrd8(pdev, regbank, chan, offs) & bit_mask)
					break;
			} else {
				if ((hssi_csrrd8(pdev, regbank, chan, offs) & bit_mask) == 0)
					break;
			}
		}
		if (counter >= INTEL_FPGA_XTILE_CNTR_CHECK) {
			if (set_bit) {
				if ((hssi_csrrd8(pdev, regbank, chan, offs) & bit_mask) == 0)
					return -EINVAL;
			} else {
				if (hssi_csrrd8(pdev, regbank, chan, offs) & bit_mask)
					return -EINVAL;
			}
		}
		break;
	default:
	/* default is word aligned */
		while (counter++ < INTEL_FPGA_XTILE_CNTR_CHECK) {
			if (priv->spec_ops->tile.check_counter_complete(priv, regbank, offs,
									bit_mask, set_bit,
									align) == 0)
				break;
			usleep_range(100, 200);
		}
		if (counter >= INTEL_FPGA_XTILE_CNTR_CHECK)
			return -EINVAL;
		break;
	}
	return 0;
}

static void xtile_clear_mac_statistics(struct platform_device *pdev, u32 port)
{
	bool is_tx_reset = true;
	bool is_rx_reset = true;

	/* Clear all statistics counters for the receive and transmit path */
	hssi_reset_mac_stats(pdev, port, is_tx_reset, is_rx_reset);
}

static bool xtile_get_link_status(struct intel_fpga_xtile_eth_private *priv)
{
	bool curr_link_state = priv->spec_ops->tile.link_fault_status &&
			       priv->spec_ops->tile.link_fault_status(priv);

	/* if the link is down then we reload the stability check count to be
	 * used in the next link up case
	 */
	if (curr_link_state) {
		if (--priv->link_stability_check > 0)
			curr_link_state = false;
	} else {
		priv->link_stability_check = PRELOAD_LINK_STABILITY_COUNT;
	}

	return curr_link_state;
}

static const char *phylink_pause_to_str(int pause)
{
	switch (pause & MLO_PAUSE_TXRX_MASK) {
	case MLO_PAUSE_TX | MLO_PAUSE_RX:
		return "rx/tx";
	case MLO_PAUSE_TX:
		return "tx";
	case MLO_PAUSE_RX:
		return "rx";
	default:
		return "off";
	}
}

static void eth_link_up(struct intel_fpga_xtile_eth_private *priv)
{
	int queue = 0;
	struct netdev_queue *txq;

	if (priv->num_channels == 0)
		return;

	/* In case there is issue and packet forwarded to DMA engine
	 * but are not being consumed by it then the DMA ring wouldn't
	 * be freed resulting in the depeletion of the ring buffers.
	 * When the buffer threshold has reached we shouldn't wake the
	 * queue back again
	 */
	for (queue = 0; queue < priv->num_channels; queue++) {
		txq = netdev_get_tx_queue(priv->dev, queue);
		if (netif_tx_queue_stopped(txq)) {
			/* In case the queue is full then when poll runs it should
			 * release some buffers
			 */
			/*FIXME*/
			if (!(xtile_tx_avail(priv, queue) <= TXQUEUESTOP_THRESHOLD))
				netif_tx_wake_queue(txq);
		} else {
		/* queue is started if the port is up for the first time */
			netif_tx_wake_queue(txq);
		}
	}

	/* Make sure carrier is on */
	if (!(netif_carrier_ok(priv->dev))) {
		netif_carrier_on(priv->dev);
		netdev_info(priv->dev, "Link is Up - %s/%s - flow control %s\n",
			    phy_speed_to_str(priv->link_speed),
			    phy_duplex_to_str(priv->duplex),
			    phylink_pause_to_str(priv->flow_ctrl));
	}

	/* My L1 is declared UP now we should be receiving data so start
	 * data transfer by enabling the interrupts
	 */
	for (queue = 0; queue < priv->num_channels; queue++) {
		xtile_modify_cpu_enable_intr(priv, queue);
		/* case to handle where the prods have filled up the ring we need to
		 * clear the ring so that the packet processing can happen.
		 * For that we are forcing the napi schedule to take place
		 */
		if (napi_schedule_prep(&priv->dma_info[queue].napi)) {
			if (netif_msg_hw(priv)) {
				netdev_err(priv->dev,
					   " __napi_schedule invoked in non ISR context: state(0x%lx)\n",
					   (&priv->dma_info[queue].napi)->state);
			}
			__napi_schedule(&priv->dma_info[queue].napi);
		} else {
			if (netif_msg_hw(priv)) {
				netdev_err(priv->dev,
					   "NAPI prep failed: State: 0x%lx\n",
					   (&priv->dma_info[queue].napi)->state);
			}
		}

		xtile_dmaintr_enable(priv, queue);

		/* Enable the Tx send */
		priv->dma_info[queue].napi_state = NAPI_ENABLED_TXREADY;
	}
}

static void eth_link_down(struct intel_fpga_xtile_eth_private *priv)
{
	int queue = 0;
	struct netdev_queue *txq;

	if (priv->num_channels == 0)
		return;

	for (queue = 0; queue < priv->num_channels; queue++) {
		txq = netdev_get_tx_queue(priv->dev, queue);
		priv->dma_info[queue].napi_state = NAPI_ENABLED_TXBLOCKED;

		/* stop the queue to avoid packet pumped to Tx DMA */
		if (!netif_tx_queue_stopped(txq))
			netif_tx_stop_queue(txq);

		/* stop the interrupts so that xtile poll doesn't get scheduled */
		xtile_dmaintr_disable(priv, queue);
	}
	/* Declare the L1 is down */
	if (netif_carrier_ok(priv->dev)) {
		netif_carrier_off(priv->dev);
		netdev_info(priv->dev, "Link is Down\n");
	}
}

/* write protected read for the monitor link status */
static inline bool wpr_get_monitor_link_status(struct intel_fpga_xtile_eth_private *priv)
{
	bool value = false;

	read_lock(&priv->wr_lock);
	value = priv->monitor_thread_enable;
	read_unlock(&priv->wr_lock);

	return value;
}

/* read protected write for the monitor link status */
static inline void rpw_set_monitor_link_status(bool value,
					       struct intel_fpga_xtile_eth_private *priv)
{
	write_lock(&priv->wr_lock);
	priv->monitor_thread_enable = value;
	write_unlock(&priv->wr_lock);
}

static void eth_monitor_link_status(struct work_struct *work)
{
	bool link = false;
	int tile_error = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct intel_fpga_xtile_eth_private *priv = container_of(dwork,
							  struct intel_fpga_xtile_eth_private,
							  dwork);

	/* NOTE: Monitoring thread can be cancelled asynchronously
	 * if the monitor thread is disabled then we have no reason to continue
	 */
	if (!wpr_get_monitor_link_status(priv))
		return;

	/* Get link status */
	if (xtile_get_link_status(priv))
		link = true;

	/* Forcly change state to STOP after RUN,
	 * once link is detected as low
	 */
	if (priv->link_state == ETH_LINK_STATE_RUN && !link)
		priv->link_state = ETH_LINK_STATE_STOP;

	switch (priv->link_state) {
	case ETH_LINK_STATE_RESET:
		priv->link_state = ETH_LINK_STATE_START;
		break;
	case ETH_LINK_STATE_START:
		if (link) {
			/* Start Tile functionality */
			if (priv->spec_ops->tile.start)
				tile_error = priv->spec_ops->tile.start(priv);

			if (!tile_error) {
				/* Start napi, netif queue, enable interrupts and phy */
				eth_link_up(priv);
				priv->link_state = ETH_LINK_STATE_RUN;
			}
		}
		break;
	case ETH_LINK_STATE_STOP:
		/* Stop Tile functionality */
		if (priv->spec_ops->tile.stop)
			tile_error = priv->spec_ops->tile.stop(priv);

		if (!tile_error) {
			/* Stop napi, netif queue, enable interrupts and phy */
			eth_link_down(priv);

			priv->link_state = ETH_LINK_STATE_RESET;
		}
		break;
	case ETH_LINK_STATE_RUN:
		if (priv->spec_ops->tile.run_check)
			tile_error = priv->spec_ops->tile.run_check(priv);

		if (tile_error)
			priv->link_state = ETH_LINK_STATE_STOP;
		break;
	}

	if (wpr_get_monitor_link_status(priv))
		schedule_delayed_work(&priv->dwork, msecs_to_jiffies(priv->monitor_poll_interval));
}

#define PRELOAD_LINK_STABILITY_COUNT 10
static void start_link_monitoring_thread(struct intel_fpga_xtile_eth_private *priv)
{
	rpw_set_monitor_link_status(true, priv);

	priv->link_stability_check = PRELOAD_LINK_STABILITY_COUNT;
	priv->link_state = ETH_LINK_STATE_RESET;
	INIT_DELAYED_WORK(&priv->dwork, eth_monitor_link_status);
	eth_monitor_link_status(&priv->dwork.work);
}

static void stop_link_monitoring_thread(struct intel_fpga_xtile_eth_private *priv)
{
	rpw_set_monitor_link_status(false, priv);
	priv->link_state = ETH_LINK_STATE_STOP;

	cancel_delayed_work_sync(&priv->dwork);
}

/* Open and initialize the interface */
static int xtile_open(struct net_device *dev)
{
	struct intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	struct platform_device *pdev = priv->pdev_hssi;
	u32 hssi_port = priv->hssi_port;
	unsigned long flags;
	int ret = 0;
	int i, queue;

	/* deassert reset:
	 * emib interface, mac, pcs, fec, pma, stat for both tx and rx
	 * different for etile aand  ftile
	 */
	if (priv->spec_ops->tile.deassert_reset)
		priv->spec_ops->tile.deassert_reset(priv);

	/* Create and initialize the TX/RX descriptors chains. */
	for (queue = 0; queue < priv->num_channels; queue++) {
		priv->dma_info[queue].dma_priv.rx_ring_size = dma_rx_num;
		priv->dma_info[queue].dma_priv.tx_ring_size = dma_tx_num;

		/* Allocate Tx and Rx descriptor ring and initialize respectively */
		ret = priv->spec_ops->dma_ops->init_dma(&priv->dma_info[queue].dma_priv);
		if (ret) {
			netdev_err(dev, "Cannot initialize DMA\n");
			goto phy_error;
		}
	}

	if (netif_msg_ifup(priv))
		netdev_info(dev, "device MAC address %pM\n",
			    dev->dev_addr);

	/* clear the MAC layer statistics to start afresh */
	xtile_clear_mac_statistics(pdev, hssi_port);
   
   	/* we need to clear the dev stats so that the ifconfig on interface shouldn't show old data */
        memset(&dev->stats, 0, sizeof(dev->stats));

	for (queue = 0; queue < priv->num_channels; queue++) {
		priv->spec_ops->dma_ops->quiese_pref(&priv->dma_info[queue].dma_priv);

		/* Reset the mSGDMA engine and configure the mSGDMA register to           */
		/* provide information of the Tx and Rx DMA ring start address in memory  */
		priv->spec_ops->dma_ops->reset_dma(&priv->dma_info[queue].dma_priv);

		priv->dma_info[queue].dma_priv.rx_ring = 0;
		priv->dma_info[queue].dma_priv.tx_ring = 0;

		ret = xtile_alloc_init_skbufs(priv, queue);
		if (ret) {
			netdev_err(dev, "DMA descriptors initialization failed\n");
			goto alloc_skbuf_error;
		}

		/* Disable DMA interrupts */
		xtile_dmaintr_disable(priv, queue);
	}

	/* Setup RX descriptor chain */
	for (queue = 0; queue < priv->num_channels; queue++) {
		spin_lock_irqsave(&priv->dma_info[queue].rxdma_irq_lock, flags);
		for (i = 0; i < priv->dma_info[queue].dma_priv.rx_ring_size; i++)
			priv->spec_ops->dma_ops->add_rx_desc(&priv->dma_info[queue].dma_priv,
					&priv->dma_info[queue].dma_priv.rx_ring[i]);
		spin_unlock_irqrestore(&priv->dma_info[queue].rxdma_irq_lock, flags);
	}

	for (queue = 0; queue < priv->num_channels; queue++) {
		xtile_modify_cpu_enable_intr(priv, queue);
		priv->dma_info[queue].irq_rx_enable_cntr = 0;
		priv->dma_info[queue].irq_tx_enable_cntr = 0;
		priv->dma_info[queue].irq_rx_disable_cntr = 0;
		priv->dma_info[queue].irq_tx_disable_cntr = 0;

		if (priv->spec_ops->dma_ops->start_txdma)
			priv->spec_ops->dma_ops->start_txdma(&priv->dma_info[queue].dma_priv);

		priv->spec_ops->dma_ops->start_rxdma(&priv->dma_info[queue].dma_priv);
	}

	/* Pre link, tile initialization */
	if (priv->spec_ops->tile.init) {
		ret  = priv->spec_ops->tile.init(priv);
		if (ret)
			goto tile_init_error;
	}

	/* Tx queue might be enabled by default we need to wait for the link to be up
	 * for the tx transmission to start
	 */
	netif_tx_stop_all_queues(dev);

	/* Enable NAPI so that driver is ready to poll when there is napi_schedule call */
	if (priv->num_channels != 0) {
		for (queue = 0; queue < priv->num_channels; queue++) {
			napi_enable(&priv->dma_info[queue].napi);
			priv->dma_info[queue].napi_state = NAPI_ENABLED_TXBLOCKED;
		}
	}

	if (!wpr_get_monitor_link_status(priv)) {
		start_link_monitoring_thread(priv);
		netdev_info(dev, "Ethernet link monitoring thread started");
	} else {
		netdev_err(dev, "Ethernet link monitoring thread already running");
	}

	return 0;

tile_init_error:
	for (queue = 0; queue < priv->num_channels; queue++)
		xtile_dmaintr_disable(priv, queue);
alloc_skbuf_error:
	/* Deallocate SKBuffer */
	xtile_free_skbufs(dev);
phy_error:
	return ret;
}

/* Stop MAC interface and put the device in an inactive state
 */
static int xtile_shutdown(struct net_device *dev)
{
	int queue = 0;
	struct intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	stop_link_monitoring_thread(priv);

	if (priv->spec_ops->tile.uninit)
		priv->spec_ops->tile.uninit(priv);

	eth_link_down(priv);

	/* Disable CPU interrupts.DMA intrs are already disabled in eth_link_down */
	for (queue = 0; queue < priv->num_channels; queue++) {
		xtile_modify_cpu_disable_intr(priv, queue);

		if (priv->dma_info[queue].napi_state != NAPI_DISABLED) {
			napi_synchronize(&priv->dma_info[queue].napi);
			napi_disable(&priv->dma_info[queue].napi);
			priv->dma_info[queue].napi_state = NAPI_DISABLED;
		}
	}

	/* we are ensuring we get the lock so that any pending activity */
	/* if any is done */
	for (queue = 0; queue < priv->num_channels; queue++) {
		spin_lock(&priv->dma_info[queue].tx_lock);
		spin_unlock(&priv->dma_info[queue].tx_lock);
	}

	spin_lock(&priv->mac_cfg_lock);
	spin_unlock(&priv->mac_cfg_lock);

	for (queue = 0; queue < priv->num_channels; queue++) {
		priv->spec_ops->dma_ops->quiese_pref(&priv->dma_info[queue].dma_priv);
		priv->spec_ops->dma_ops->reset_dma(&priv->dma_info[queue].dma_priv);
	}
	/* Clear the allocate skbuffers */
	xtile_free_skbufs(dev);

	for (queue = 0; queue < priv->num_channels; queue++)
		priv->spec_ops->dma_ops->uninit_dma(&priv->dma_info[queue].dma_priv);

	/* reset: emib interface, mac, pcs, fec, pma, stat for both tx and rx */
	if (priv->spec_ops->tile.reset)
		priv->spec_ops->tile.reset(priv, false, false, true);

	for (queue = 0; queue < priv->num_channels; queue++) {
		/* Initialize back the producer and consumers for better debug */
		priv->dma_info[queue].dma_priv.tx_cons = 0;
		priv->dma_info[queue].dma_priv.rx_cons = 0;
		priv->dma_info[queue].dma_priv.tx_prod = 0;
		priv->dma_info[queue].dma_priv.rx_prod = 0;
	}
	return 0;
}

static int xtile_change_mac(struct net_device *dev, void *inet_ds)
{
	struct sockaddr *addr = inet_ds;
	struct intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	dev_addr_set(dev, addr->sa_data);

	if (priv->spec_ops->tile.update_mac_addr)
		priv->spec_ops->tile.update_mac_addr(priv);

	return 0;
}

/* Transmit a packet (called by the kernel). Dispatches
 * either the SGDMA method for transmitting or the
 * MSGDMA method, assumes no scatter/gather support,
 * implying an assumption that there's only one
 * physically contiguous fragment starting at
 * skb->data, for length of skb_headlen(skb).
 *
 * Exec: xmit time taken 10us
 */
static int xtile_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	unsigned int entry;
	int queue = 0;
	dma_addr_t dma_addr;
	struct altera_dma_buffer *buffer = NULL;
	int nfrags = skb_shinfo(skb)->nr_frags;
	unsigned int nopaged_len = skb_headlen(skb);
	struct intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	unsigned int txsize = 0;
	struct netdev_queue *txq;

	if (priv->num_channels == 0)
		return NETDEV_TX_BUSY;

	queue = skb_get_queue_mapping(skb);
	if (queue >= MAX_DMA_CHANNELS) {
		netdev_err(dev, "SKB Queue is wrong: %d %d %p", queue, MAX_DMA_CHANNELS, skb);
		queue = 0;
	}

	txsize = priv->dma_info[queue].dma_priv.tx_ring_size;
	txq = netdev_get_tx_queue(priv->dev, queue);

	// pad with dummy bytes, DMA irq will stop otherwise
	if (nopaged_len < 60)
		nopaged_len = 60;

	if (netif_tx_queue_stopped(txq) ||
	    priv->dma_info[queue].napi_state != NAPI_ENABLED_TXREADY)
		return NETDEV_TX_BUSY;

	spin_lock_bh(&priv->dma_info[queue].tx_lock);

	/* at start up or in the case the queue is stopped (i.e. L1 down)
	 * we do not send the packet down the DMA
	 * If the napi is not started then we wouldn't get txcompletion for the
	 * packet sent to DMA so better to not send and ignore the packet
	 * NOTE: netdev at least transmits one packet even if queue is stopped
	 * hence an additional check on napi_state to avoid the packet leak to DMA
	 */
	if (unlikely(xtile_tx_avail(priv, queue) < nfrags + 1)) {
		/* This is a hard error, log it. */
		netdev_err(priv->dev,
			   "Tx list full when queue awake\n");
		goto err;
	}

	if (unlikely(netif_msg_pktdata(priv))) {
		netdev_info(dev, "sending skb of len=%d\n", skb->len);

		print_hex_dump(KERN_ERR, "data: ", DUMP_PREFIX_OFFSET,
			       16, 1, skb->data, skb->len, true);
	}

	skb_reserve(skb, SKB_DMA_REALIGN);

	/* Map the first skb fragment */
	entry = priv->dma_info[queue].dma_priv.tx_prod % txsize;
	buffer = &priv->dma_info[queue].dma_priv.tx_ring[entry];

	/* buffer is created prior just to keep the spin lock section short */
	dma_addr = dma_map_single(priv->device, skb->data,
				  nopaged_len,
			DMA_TO_DEVICE);

	/* Ref: https://www.kernel.org/doc/html/latest/core-api/dma-api-howto.html */
	if (dma_mapping_error(priv->device, dma_addr)) {
		netdev_err(priv->dev, "DMA mapping error\n");

		dma_unmap_single(priv->device, dma_addr,
				 nopaged_len,
				DMA_TO_DEVICE);

		dev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);

		goto allgood;
	}

	buffer->skb = skb;
	buffer->dma_addr = dma_addr;
	buffer->len = nopaged_len;

	/* Push data out of the cache hierarchy into main memory */
	dma_sync_single_for_device(priv->device, buffer->dma_addr,
				   buffer->len, DMA_TO_DEVICE);

	/* Provide a hardware time stamp if requested.  */
	if (unlikely((skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) &&
		     priv->dma_info[queue].dma_priv.hwts_tx_en))
		/* declare that device is doing timestamping */
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;

	/* Provide a software time stamp if requested and hardware timestamping
	 * is not possible (SKBTX_IN_PROGRESS not set).
	 */
	if (!priv->dma_info[queue].dma_priv.hwts_tx_en)
		skb_tx_timestamp(skb);

	if (unlikely(NETDEV_TX_BUSY ==
		     priv->spec_ops->dma_ops->tx_buffer(&priv->dma_info[queue].dma_priv, buffer))) {
		/* In order to avoid skb to be freed
		 * Ref: https://www.kernel.org/doc/html/latest/networking/driver.html
		 */
		buffer->skb = NULL;
		xtile_free_tx_buffer(priv, buffer);
		goto err;
	} else {
		priv->dma_info[queue].dma_priv.tx_prod++;
		dev->stats.tx_bytes += nopaged_len;
	}

	if (unlikely(xtile_tx_avail(priv, queue) <= TXQUEUESTOP_THRESHOLD)) {
		if (netif_msg_hw(priv))
			netdev_info(priv->dev, " stopped transmitting packets\n");
		netif_tx_stop_queue(txq);
	}

allgood:
	spin_unlock_bh(&priv->dma_info[queue].tx_lock);
	return NETDEV_TX_OK;

err:
	spin_unlock_bh(&priv->dma_info[queue].tx_lock);
	if (!netif_tx_queue_stopped(txq))
		netif_tx_stop_queue(txq);

	dev->stats.tx_errors++;
	netdev_err(priv->dev, "xmit NETDEV_TX_BUSY\n");

	return NETDEV_TX_BUSY;
}

/* Control hardware timestamping.
 * This function configures the MAC to enable/disable both outgoing(TX)
 * and incoming(RX) packets time stamping based on user input.
 */
static int xtile_set_hwtstamp_config(struct net_device *dev, struct ifreq *ifr)
{
	int ret = 0;
	int queue = 0;
	struct intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	struct hwtstamp_config config;

	if (copy_from_user(&config, ifr->ifr_data,
			   sizeof(struct hwtstamp_config)))
		return -EFAULT;

	if (unlikely(netif_msg_drv(priv))) {
		netif_info(priv, drv, dev,
			   "%s config flags:0x%x, tx_type:0x%x, rx_filter:0x%x\n",
			   __func__, config.flags, config.tx_type, config.rx_filter);
	}

	/* reserved for future extensions */
	if (config.flags)
		return -EINVAL;

	switch (config.tx_type) {
	case HWTSTAMP_TX_ON:
		for (queue = 0; queue < priv->num_channels; queue++)
			priv->dma_info[queue].dma_priv.hwts_tx_en = 1;
		break;
	default:
		return -ERANGE;
	}

	switch (config.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		for (queue = 0; queue < priv->num_channels; queue++)
			priv->dma_info[queue].dma_priv.hwts_rx_en = 0;
		config.rx_filter = HWTSTAMP_FILTER_NONE;
		break;
	default:
		for (queue = 0; queue < priv->num_channels; queue++)
			priv->dma_info[queue].dma_priv.hwts_rx_en = 1;
		config.rx_filter = HWTSTAMP_FILTER_ALL;
		break;
	}

	if (copy_to_user(ifr->ifr_data, &config,
			 sizeof(struct hwtstamp_config)))
		return -EFAULT;
	return ret;
}

/* Set or clear the multicast filter for this adaptor
 */
static void xtile_set_rx_mode(struct net_device *dev)
{
	/* Not Supported */
}

/* Change the MTU
 */
static int xtile_change_mtu(struct net_device *dev, int new_mtu)
{
	struct intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	unsigned int max_mtu = priv->dev->max_mtu;
	unsigned int min_mtu = priv->dev->min_mtu;

	if (netif_running(dev)) {
		netdev_err(dev, "must be stopped to change its MTU\n");
		return -EBUSY;
	}

	if (new_mtu < min_mtu || new_mtu > max_mtu) {
		netdev_err(dev, "invalid MTU, max MTU is: %u\n", max_mtu);
		return -EINVAL;
	}

	dev->mtu = new_mtu;
	netdev_update_features(dev);

	return 0;
}

/* Entry point for the ioctl.
 */
static int xtile_do_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	int ret = 0;

	if (!netif_running(dev))
		return -EINVAL;

	switch (cmd) {
	case SIOCSHWTSTAMP:
		ret = xtile_set_hwtstamp_config(dev, ifr);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return ret;
}

static void drv_get_stats64(struct net_device *dev,
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

static void xtile_get_stats64(struct net_device *dev,
			      struct rtnl_link_stats64 *storage)
{
	struct intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	if (priv->spec_ops->tile.net_stats)
		priv->spec_ops->tile.net_stats(dev, storage);
	else
		drv_get_stats64(dev, storage);
}

static const struct net_device_ops intel_fpga_xtile_netdev_ops = {
	.ndo_open		= xtile_open,
	.ndo_stop		= xtile_shutdown,
	.ndo_start_xmit		= xtile_start_xmit,
	.ndo_set_mac_address	= xtile_change_mac,
	.ndo_set_rx_mode	= xtile_set_rx_mode,
	.ndo_change_mtu		= xtile_change_mtu,
	.ndo_eth_ioctl		= xtile_do_ioctl,
	.ndo_validate_addr      = eth_validate_addr,
	.ndo_get_stats64	= xtile_get_stats64,
	.ndo_select_queue	= xtile_select_queue
};

static int intel_fpga_xtile_validate(struct phylink_pcs *pcs,
				     unsigned long *supported,
				      const struct phylink_link_state *state_validate)
{
	struct phylink_link_state *state = (struct phylink_link_state *)state_validate;
	intel_fpga_xtile_eth_private *priv =
		container_of(pcs, intel_fpga_xtile_eth_private, pcs);

	__ETHTOOL_DECLARE_LINK_MODE_MASK(mac_supported) = { 0, };
	__ETHTOOL_DECLARE_LINK_MODE_MASK(mask) = { 0, };

	if (!priv)
		return -EINVAL;

	if (state->interface != PHY_INTERFACE_MODE_NA &&
	    state->interface != PHY_INTERFACE_MODE_10GKR &&
	    state->interface != PHY_INTERFACE_MODE_10GBASER &&
	    state->interface != PHY_INTERFACE_MODE_25GKR) {
		bitmap_zero(supported, __ETHTOOL_LINK_MODE_MASK_NBITS);
		return 0;
	}

	if (priv->autoneg) {
		phylink_set(mask, Autoneg);
		phylink_set(mac_supported, Autoneg);
	} else {
		phylink_clear(mask, Autoneg);
		phylink_clear(mac_supported, Autoneg);
	}

	phylink_set(mask, Pause);
	phylink_set(mac_supported, Pause);
	phylink_set(mask, Asym_Pause);
	phylink_set(mac_supported, Asym_Pause);
	phylink_set_port_modes(mask);
	phylink_set_port_modes(mac_supported);

	switch (state->interface) {
	case PHY_INTERFACE_MODE_10GKR:
	case PHY_INTERFACE_MODE_10GBASER:
		phylink_set(mask, 10000baseT_Full);
		phylink_set(mask, 10000baseCR_Full);
		phylink_set(mask, 10000baseSR_Full);
		phylink_set(mask, 10000baseLR_Full);
		phylink_set(mask, 10000baseLRM_Full);
		phylink_set(mask, 10000baseER_Full);
		phylink_set(mask, 10000baseKR_Full);
		phylink_set(mac_supported, 10000baseT_Full);
		phylink_set(mac_supported, 10000baseCR_Full);
		phylink_set(mac_supported, 10000baseSR_Full);
		phylink_set(mac_supported, 10000baseLR_Full);
		phylink_set(mac_supported, 10000baseLRM_Full);
		phylink_set(mac_supported, 10000baseER_Full);
		phylink_set(mac_supported, 10000baseKR_Full);
		state->speed = SPEED_10000;
		break;
	case PHY_INTERFACE_MODE_25GKR:
		phylink_set(mask, 25000baseCR_Full);
		phylink_set(mask, 25000baseKR_Full);
		phylink_set(mask, 25000baseSR_Full);
		phylink_set(mac_supported, 25000baseCR_Full);
		phylink_set(mac_supported, 25000baseKR_Full);
		phylink_set(mac_supported, 25000baseSR_Full);
		state->speed = SPEED_25000;
	default:
		break;
	}

	bitmap_and(supported, supported, mask, __ETHTOOL_LINK_MODE_MASK_NBITS);
	bitmap_and(state->advertising, state->advertising, mask,
		   __ETHTOOL_LINK_MODE_MASK_NBITS);
	bitmap_and(supported, supported, mac_supported,
		   __ETHTOOL_LINK_MODE_MASK_NBITS);
	bitmap_and(state->advertising, state->advertising, mac_supported,
		   __ETHTOOL_LINK_MODE_MASK_NBITS);

	return 0;
}

static void intel_fpga_xtile_mac_pcs_get_state(struct phylink_pcs *pcs,
					       struct phylink_link_state *state)
{
	/* fixed speed for now */
	intel_fpga_xtile_eth_private *priv =
		container_of(pcs, intel_fpga_xtile_eth_private, pcs);

	if (!priv)
		return;

	state->speed = priv->link_speed;
	state->duplex = DUPLEX_FULL;
	state->link = 1;
}

static void intel_fpga_xtile_mac_an_restart(struct phylink_pcs *config)
{
	/* Not Supported */
}

static struct phylink_pcs *intel_fpga_xtile_mac_select_pcs(struct phylink_config *config,
							   phy_interface_t iface)
{
	intel_fpga_xtile_eth_private *priv =
		netdev_priv(to_net_dev(config->dev));

	if (!priv)
		return NULL;

	return &priv->pcs;
}

static void intel_fpga_xtile_get_pcs_fixed_state(struct phylink_config *config,
						 struct phylink_link_state *state)
{
	intel_fpga_xtile_eth_private *priv =
		netdev_priv(to_net_dev(config->dev));

	if (!priv)
		return;

	state->speed = priv->link_speed;
	state->duplex = DUPLEX_FULL;
	state->an_complete = AUTONEG_ENABLE;
	if (priv->autoneg)
		state->an_complete = AUTONEG_DISABLE;
}

static void intel_fpga_xtile_mac_config(struct phylink_config *config,
					unsigned int mode,
					const struct phylink_link_state *state)
{
	/* Not Supported */
}

static void intel_fpga_xtile_mac_link_down(struct phylink_config *config,
					   unsigned int mode,
					   phy_interface_t interface)
{
	struct intel_fpga_xtile_eth_private *priv =
			netdev_priv(to_net_dev(config->dev));

	phylink_mac_change(priv->phylink, false);
}

static void intel_fpga_xtile_mac_link_up(struct phylink_config *config,
					 struct phy_device *phy,
					 unsigned int mode,
					 phy_interface_t interface, int speed,
					 int duplex, bool tx_pause,
					 bool rx_pause)
{
	struct intel_fpga_xtile_eth_private *priv =
			netdev_priv(to_net_dev(config->dev));

	phylink_mac_change(priv->phylink, true);
}

static const struct phylink_pcs_ops intel_fpga_xtile_pcs_ops = {
	.pcs_get_state = intel_fpga_xtile_mac_pcs_get_state,
	.pcs_an_restart = intel_fpga_xtile_mac_an_restart,
	.pcs_validate = intel_fpga_xtile_validate,
};

static const struct phylink_mac_ops intel_fpga_xtile_phylink_ops = {
	.mac_select_pcs = intel_fpga_xtile_mac_select_pcs,
	.mac_config = intel_fpga_xtile_mac_config,
	.mac_link_down = intel_fpga_xtile_mac_link_down,
	.mac_link_up = intel_fpga_xtile_mac_link_up,
};

/* Probe MAC device */
static int intel_fpga_xtile_probe(struct platform_device *pdev)
{
	int ret = -ENODEV;
	struct device_node *np, *dmanp;
	struct net_device *ndev;
	struct device_node *dev_hssi;
	u8 macaddr[ETH_ALEN];
	struct fwnode_handle *fixed_node;
	struct platform_device *pdev_hssi;
	const struct xtile_spec_ops *op_ptr;
	struct intel_fpga_xtile_eth_private *priv;
	struct device_node *dev_tod;
	struct platform_device *pdev_tod;
	char dma_nodename[6];
	int queue = 0;
	const char *if_name = NULL;
	char irq_name[12];
	struct set_mtu_data mtu;

	np = pdev->dev.of_node;

	ndev = alloc_etherdev_mq(sizeof(struct intel_fpga_xtile_eth_private), MAX_DMA_CHANNELS);
	if (!ndev) {
		dev_err(&pdev->dev, "Could not allocate network device\n");
		return -ENODEV;
	}

	SET_NETDEV_DEV(ndev, &pdev->dev);

	priv = netdev_priv(ndev);

	priv->dev	      = ndev;
	priv->flow_ctrl	      = flow_ctrl;
	priv->pause	      = pause;
	priv->device          = &pdev->dev;
	priv->msg_enable      = netif_msg_init(debug, default_msg_level);

	priv->phylink_config.dev = &priv->dev->dev;
	priv->phylink_config.type = PHYLINK_NETDEV;
	priv->phylink_config.get_fixed_state = intel_fpga_xtile_get_pcs_fixed_state;

	priv->pcs.ops = &intel_fpga_xtile_pcs_ops;
	priv->pcs.neg_mode = true;
	priv->pcs.poll = true;

	op_ptr = of_device_get_match_data(&pdev->dev);

	if (!op_ptr) {
		dev_err(&pdev->dev, "No matching data field found\n");
		ret = -ENODEV;
		goto err_free_netdev;
	}
	if (of_property_read_u32(np, "num_channels", &priv->num_channels)) {
		dev_err(&pdev->dev, "Cannot get number of dma channels. Defaulting to 1.\n");
		priv->num_channels = 1;
	}

	priv->dma_info = kcalloc(priv->num_channels, sizeof(struct intel_xtile_msgdma_info),
				 GFP_KERNEL);
	if (priv->dma_info == 0) {
		dev_err(&pdev->dev, "Cannot allocate memory for DMA channels\n");
		ret = -ENODEV;
		goto err_free_netdev;
	}
	for (queue = 0; queue < priv->num_channels; queue++) {
		priv->dma_info[queue].queue = queue;
		priv->dma_info[queue].priv = priv;
		priv->dma_info[queue].dma_priv.dev = ndev;
		priv->dma_info[queue].dma_priv.device = &pdev->dev;
		priv->dma_info[queue].dma_priv.msg_enable = netif_msg_init(debug,
									   default_msg_level);
	}
	/* Get the HSSI node device from the device tree node */
	dev_hssi = of_parse_phandle(pdev->dev.of_node, "hssiss", 0);
	if (!dev_hssi)
		return -ENOENT;

	pdev_hssi = of_find_device_by_node(dev_hssi);
	if (!pdev_hssi) {
		of_node_put(dev_hssi);
		return -ENODEV;
	}
	priv->pdev_hssi = pdev_hssi;

	/* Get the HSSI node device from the device tree node */
	/* get hssi port no from device tree */
	if (of_property_read_u32(np, "hssi_port",
				 &priv->hssi_port)) {
		dev_err(&pdev->dev, "cannot obtain hssi port info\n");
		ret = -ENXIO;
		goto err_free_netdev;
	}

	if (of_property_read_u32(np, "tile_chan",
				 &priv->tile_chan)) {
		dev_err(&pdev->dev, "cannot obtain tile channel info\n");
		ret = -ENXIO;
		goto err_free_netdev;
	}

	if (of_property_read_u32(np, "monitor_poll_interval",
				 &priv->monitor_poll_interval)) {
		dev_err(&pdev->dev, "cannot obtain monitor poll interval\n");
		ret = -ENXIO;
		goto err_free_netdev;
	}

	priv->spec_ops = (struct xtile_spec_ops *)op_ptr;

	/* PTP is only supported with a modified MSGDMA */
	priv->ptp_enable = of_property_read_bool(pdev->dev.of_node,
						 "altr,has-ptp");
	if (priv->ptp_enable &&
	    priv->spec_ops->dma_ops->altera_dtype != ALTERA_DTYPE_MSGDMA_PREF) {
		dev_err(&pdev->dev, "PTP requires modified dma\n");
		ret = -ENODEV;
		goto err_free_netdev;
	}

	if (priv->spec_ops->tile.check_dts_param) {
		if (!priv->spec_ops->tile.check_dts_param(priv)) {
			ret = -ENXIO;
			goto err_free_netdev;
		}
	}

	priv->ptp_clockcleaner_enable = of_property_read_bool(pdev->dev.of_node,
							      "altr,has-ptp-clockcleaner");
	/* ptp clock cleaner is not applicable for Ethernet only design */
	if (priv->ptp_clockcleaner_enable && !priv->ptp_enable) {
		dev_err(&pdev->dev, "Hardware Clock Frequency adjustment requires PTP\n");
		ret = -ENODEV;
		goto err_free_netdev;
	}

	if (priv->num_channels != 0) {
		netif_set_real_num_tx_queues(ndev, priv->num_channels);
		queue = 0;
		dmanp = NULL;
		while ((dmanp = of_get_next_child(np, dmanp))) {
			memset(dma_nodename, 0, sizeof(dma_nodename));
			snprintf(dma_nodename, sizeof(dma_nodename), "dma_%d", queue);
			if (!of_node_name_eq(dmanp, dma_nodename))
				continue;
			/* mSGDMA Tx IRQ */
			memset(irq_name, 0, sizeof(irq_name));
			snprintf(irq_name, sizeof(irq_name), "dma%d_tx_irq", queue);
			priv->dma_info[queue].tx_irq = platform_get_irq_byname(pdev, irq_name);
			if (priv->dma_info[queue].tx_irq == -ENXIO) {
				dev_err(&pdev->dev, "cannot obtain Tx IRQ for DMA %d\n", queue);
				ret = -ENXIO;
				goto err_free_netdev;
			}

			/* Register TX interrupt */
			ret = devm_request_irq(priv->device, priv->dma_info[queue].tx_irq,
					       intel_fpga_xtile_isr,
					       IRQF_SHARED, ndev->name, ndev);
			if (ret) {
				dev_err(&pdev->dev, "Unable to register TX interrupt %d\n",
					priv->dma_info[queue].tx_irq);
				goto err_free_netdev;
			}
			disable_irq(priv->dma_info[queue].tx_irq);
			priv->dma_info[queue].tx_irq_enabled = false;

			snprintf(irq_name, sizeof(irq_name), "dma%d_rx_irq", queue);
			priv->dma_info[queue].rx_irq = platform_get_irq_byname(pdev, irq_name);
			if (priv->dma_info[queue].rx_irq == -ENXIO) {
				dev_err(&pdev->dev, "cannot obtain Rx IRQ for DMA %d\n", queue);
				ret = -ENXIO;
				goto err_free_netdev;
			}

			/* Register RX interrupt */
			ret = devm_request_irq(priv->device, priv->dma_info[queue].rx_irq,
					       intel_fpga_xtile_isr,
					       IRQF_SHARED, ndev->name, ndev);
			if (ret) {
				dev_err(&pdev->dev, "Unable to register RX interrupt %d\n",
					priv->dma_info[queue].rx_irq);
				goto err_free_netdev;
			}
			disable_irq(priv->dma_info[queue].rx_irq);
			priv->dma_info[queue].rx_irq_enabled = false;

			/* Map DMA */
			ret = altera_eth_dma_node_probe(pdev, dmanp, &priv->dma_info[queue],
							priv->spec_ops->dma_ops->altera_dtype);
			if (ret) {
				dev_err(&pdev->dev, "cannot map DMA\n");
				goto err_free_netdev;
			}

			/* Rx Fifo */
			ret = request_and_map_node(pdev, dmanp, "rx_fifo",
						   (void __iomem **)(&priv->dma_info[queue].rx_fifo));
			if (ret)
				goto err_free_netdev;

			/* Tx Fifo */
			ret = request_and_map_node(pdev, dmanp, "tx_fifo",
						   (void __iomem **)(&priv->dma_info[queue].tx_fifo));
			if (ret)
				goto err_free_netdev;

			if (of_property_read_u32(dmanp,
						 "rx-fifo-almost-full",
						 &priv->dma_info[queue].rx_fifo_almost_full)) {
				dev_err(&pdev->dev, "cannot obtain rx-fifo-almost-full\n");
				priv->dma_info[queue].rx_fifo_almost_full = 0x4000;
			}

			if (of_property_read_u32(dmanp,
						 "rx-fifo-almost-empty",
						 &priv->dma_info[queue].rx_fifo_almost_empty)) {
				dev_err(&pdev->dev, "cannot obtain rx-fifo-almost-empty\n");
				priv->dma_info[queue].rx_fifo_almost_empty = 0x3000;
			}

			/* The DMA buffer size already accounts for an alignment bias
			 * to avoid unaligned access exceptions for the NIOS processor,
			 */
			priv->dma_info[queue].dma_priv.rx_dma_buf_sz = INTEL_FPGA_RXDMABUFFER_SIZE;
			queue++;
		}
		if (dma_set_mask_and_coherent(priv->device,
					      DMA_BIT_MASK(priv->spec_ops->dma_ops->dmamask))) {
			if (dma_set_mask_and_coherent(priv->device,
						      DMA_BIT_MASK(32))) {
				goto err_free_netdev;
			}
		}
	}

	priv->dev->min_mtu = ETH_ZLEN + ETH_FCS_LEN;

	/* Max MTU is 1500, ETH_DATA_LEN */
	priv->dev->max_mtu = VLAN_ETH_FRAME_LEN + ETH_FCS_LEN;

	/* Get the max mtu from the device tree. Note that the
	 * "max-frame-size" parameter is actually max mtu. Definition
	 * in the ePAPR v1.1 spec and usage differ, so go with usage.
	 */
	if (of_property_read_u32(pdev->dev.of_node, "max-frame-size",
				 &priv->dev->max_mtu)) {
		dev_warn(&pdev->dev, "Not able to get max-frame-size. Defaulting max_mtu to %d\n",
			 priv->dev->max_mtu);
	} else {
                mtu.port = priv->hssi_port;
                mtu.max_tx_frame_size = mtu.max_rx_frame_size = priv->dev->max_mtu;
                hssiss_set_mtu(priv->pdev_hssi, SAL_SET_MTU, &mtu);
        }

	/* The DMA buffer size already accounts for an alignment bias
	 * to avoid unaligned access exceptions for the NIOS processor,
	 */
	priv->dma_priv.rx_dma_buf_sz = INTEL_FPGA_RXDMABUFFER_SIZE;

	/* Get MAC PMA digital delays from device tree */
	if (of_property_read_u32(np, "altr,tx-pma-delay-ns",
				 &priv->tx_pma_delay_ns)) {
		dev_warn(&pdev->dev, "cannot obtain Tx PMA delay ns\n");
		priv->tx_pma_delay_ns = 0;
	}

	if (of_property_read_u32(np, "altr,rx-pma-delay-ns",
				 &priv->rx_pma_delay_ns)) {
		dev_warn(&pdev->dev, "cannot obtain Rx PMA delay\n");
		priv->rx_pma_delay_ns = 0;
	}

	if (of_property_read_u32(np, "altr,tx-pma-delay-fns",
				 &priv->tx_pma_delay_fns)) {
		dev_warn(&pdev->dev, "cannot obtain Tx PMA delay fns\n");
		priv->tx_pma_delay_fns = 0;
	}

	if (of_property_read_u32(np, "altr,rx-pma-delay-fns",
				 &priv->rx_pma_delay_fns)) {
		dev_warn(&pdev->dev, "cannot obtain Rx PMA delay\n");
		priv->rx_pma_delay_fns = 0;
	}

	if (of_property_read_u32(np, "altr,tx-external-phy-delay-ns",
				 &priv->tx_external_phy_delay_ns)) {
		dev_warn(&pdev->dev, "cannot obtain Tx phy delay ns\n");
		priv->tx_external_phy_delay_ns = 0;
	}

	if (of_property_read_u32(np, "altr,rx-external-phy-delay-ns",
				 &priv->rx_external_phy_delay_ns)) {
		dev_warn(&pdev->dev, "cannot obtain Rx phy delay ns\n");
		priv->rx_external_phy_delay_ns = 0;
	}

	if (of_get_mac_address(pdev->dev.of_node, macaddr)) {
		dev_info(&pdev->dev, "cannot obtain MAC address using random HW address\n");
		eth_hw_addr_random(ndev);
	} else {
		dev_addr_set(ndev, macaddr);
	}

	/* initialize netdev */
	ndev->netdev_ops = &intel_fpga_xtile_netdev_ops;

	priv->spec_ops->tile.reg_ethtool_ops(ndev);

	ndev->mem_start = 0;
	ndev->mem_end   = 0;

	/* Scatter/gather IO is not supported,
	 * so it is turned off
	 */
	ndev->hw_features &= ~NETIF_F_SG;
	ndev->features |= ndev->hw_features | NETIF_F_HIGHDMA;

	/* VLAN offloading of tagging, stripping and filtering is not
	 * supported by hardware, but driver will accommodate the
	 * extra 4-byte VLAN tag for processing by upper layers
	 */
	ndev->features |= NETIF_F_HW_VLAN_CTAG_RX;

	for (queue = 0; queue < priv->num_channels; queue++) {
		/* setup NAPI interface */
		netif_napi_add(ndev, &priv->dma_info[queue].napi, xtile_poll);

		/* tracks the current napi state whether enabled or disabled */
		priv->dma_info[queue].napi_state = NAPI_DISABLED;

		spin_lock_init(&priv->dma_info[queue].tx_lock);
		spin_lock_init(&priv->dma_info[queue].rxdma_irq_lock);
	}
	spin_lock_init(&priv->mac_cfg_lock);

	rwlock_init(&priv->wr_lock);

	/* check if phy-mode is present */
	ret = of_get_phy_mode(np, &priv->phy_iface);
	if (ret) {
		dev_err(&pdev->dev, "incorrect phy-mode\n");
		goto err_free_netdev;
	}

	if (priv->ptp_enable) {
		dev_tod  = of_parse_phandle(pdev->dev.of_node, "tod", 0);
		pdev_tod = of_find_device_by_node(dev_tod);
		if (pdev_tod)
			priv->ptp_priv = dev_get_drvdata(&pdev_tod->dev);
		if (!pdev_tod || !priv->ptp_priv) {
			dev_err(&pdev->dev, "PTP clock not available, retry!\n");
			ret = -EPROBE_DEFER;
			goto err_free_netdev;
		}
		dev_info(&pdev->dev, "\tPTP Clock: %s\n", priv->ptp_priv->ptp_clock_ops.name);
	}

	__set_bit(PHY_INTERFACE_MODE_10GBASER,
		  priv->phylink_config.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_25GBASER,
		  priv->phylink_config.supported_interfaces);

	/* create phylink */
	priv->phylink = phylink_create(&priv->phylink_config, pdev->dev.fwnode,
				       priv->phy_iface, &intel_fpga_xtile_phylink_ops);
	if (IS_ERR(priv->phylink)) {
		dev_err(&pdev->dev, "failed to create phylink\n");
		ret = PTR_ERR(priv->phylink);
		goto err_free_netdev;
	}

	priv->autoneg = true;

       fixed_node = fwnode_get_named_child_node(pdev->dev.fwnode, "fixed-link");
       if (fixed_node) {
                fwnode_property_read_u32(fixed_node, "speed", &priv->link_speed);
		/* read the fixed link properties*/
		priv->duplex = DUPLEX_FULL;
		priv->autoneg = false;

		dev_info(&pdev->dev, "\tfixed link speed:%d full duplex:%d\n",
			 priv->link_speed, priv->duplex);

		fwnode_handle_put(fixed_node);
	} else {
		dev_err(&pdev->dev, "fixed link property undefined\n");
		ret = -ENODEV;
		goto err_free_netdev;
	}

        ret  = of_property_read_string(pdev->dev.of_node, "if_name",
                                       &if_name);

        if (if_name) {
                memset(&ndev->name, 0, 16);
                memcpy(ndev->name, if_name, strlen(if_name));
        }

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register ethernet device\n");
		goto err_register_netdev;
	}

	platform_set_drvdata(pdev, ndev);

	ret = xtile_fec_init(pdev, priv);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to init FEC\n");
		ret = -ENXIO;
		goto err_init_fec;
	}

	/* Default, Need to change this */
	priv->pma_lanes_used = 1;

	/* make the carrier off by default */
	netif_carrier_off(ndev);

	return 0;

err_init_fec:
	unregister_netdev(ndev);
err_register_netdev:
	for (queue = 0; queue < priv->num_channels; queue++)
		netif_napi_del(&priv->dma_info[queue].napi);
err_free_netdev:
	kfree(priv->dma_info);
	free_netdev(ndev);
	return ret;
}

/* Remove MAC device */
static void intel_fpga_xtile_remove(struct platform_device *pdev)
{
	struct intel_fpga_xtile_eth_private *priv;
	struct net_device *ndev;

	ndev = platform_get_drvdata(pdev);
	priv = netdev_priv(ndev);

	/* perform the proper cleaning up */
	xtile_shutdown(ndev);
	kfree(priv->dma_info);
	platform_set_drvdata(pdev, NULL);
	unregister_netdev(ndev);
	free_netdev(ndev);
}

static const struct altera_dmaops altera_dtype_prefetcher = {
	.altera_dtype   = ALTERA_DTYPE_MSGDMA_PREF,
	.dmamask        = 64,
	.quiese_pref    = msgdma_pref_quiese,
	.reset_dma      = msgdma_pref_reset,
	.is_txirq_set   = msgdma_pref_is_txirq,
	.is_rxirq_set   = msgdma_pref_is_rxirq,
	.enable_txirq   = msgdma_pref_enable_txirq,
	.enable_rxirq   = msgdma_pref_enable_rxirq,
	.disable_txirq  = msgdma_pref_disable_txirq,
	.disable_rxirq  = msgdma_pref_disable_rxirq,
	.clear_txirq    = msgdma_pref_clear_txirq,
	.clear_rxirq    = msgdma_pref_clear_rxirq,
	.tx_buffer      = msgdma_pref_tx_buffer,
	.tx_completions = msgdma_pref_tx_completions,
	.add_rx_desc    = msgdma_pref_add_rx_desc,
	.get_rx_status  = msgdma_pref_rx_status,
	.init_dma       = msgdma_pref_initialize,
	.uninit_dma     = msgdma_pref_uninitialize,
	.start_rxdma    = msgdma_pref_start_rxdma,
	.start_txdma    = msgdma_pref_start_txdma,
};

static const struct xtile_spec_ops etile_data = {
	.dma_ops   = &altera_dtype_prefetcher,
	.tile = {
		.reset            = etile_ehip_reset,
		.deassert_reset   = etile_ehip_deassert_reset,
		.init             = etile_init,
		.uninit           = etile_uninit,
		.start            = etile_start,
		.stop             = etile_stop,
		.update_mac_addr  = etile_update_mac_addr,
		.link_fault_status = etile_get_link_fault_status,
		.reg_ethtool_ops  =
			intel_fpga_etile_set_ethtool_ops,
		.check_counter_complete =
			etile_check_counter_complete,
	},
};

static const struct xtile_spec_ops ftile_data = {
	.dma_ops   = &altera_dtype_prefetcher,
	.tile = {
		.reset            = ftile_ehip_reset,
		.deassert_reset   = ftile_ehip_deassert_reset,
		.init             = ftile_init,
		.uninit           = ftile_uninit,
		.start            = ftile_start,
		.stop             = ftile_stop,
		.run_check        = ftile_run_check,
		.update_mac_addr  = ftile_update_mac_addr,
		.link_fault_status = ftile_get_link_fault_status,
		.reg_ethtool_ops  =
			intel_fpga_ftile_set_ethtool_ops,
		.check_counter_complete =
			ftile_check_counter_complete,
		.check_dts_param = ftile_check_dts_param,
	},
};

static const struct xtile_spec_ops gts_data = {
        .dma_ops   = &altera_dtype_prefetcher,
        .tile = {
                .reset            = gts_ehip_reset,
                .deassert_reset   = gts_ehip_deassert_reset,
                .init             = gts_init,
                .uninit           = gts_uninit,
                .start            = gts_start,
                .stop             = gts_stop,
                .run_check        = gts_run_check,
                .update_mac_addr  = gts_update_mac_addr,
                .link_fault_status = gts_get_link_fault_status,
                .reg_ethtool_ops  =
                        intel_fpga_gts_set_ethtool_ops,
                .check_dts_param = gts_check_dts_param,
        },
};

static const struct of_device_id intel_fpga_xtile_ll_ids[] = {
	{.compatible = "altr,hssi-etile-1.0",
	 .data = &etile_data,
	},
	{.compatible = "altr,hssi-ftile-1.0",
	 .data = &ftile_data,
	},
        {.compatible = "altr,msgdma-gts-1.0",
         .data = &gts_data,
        },
};

MODULE_DEVICE_TABLE(of, intel_fpga_xtile_ll_ids);

static struct platform_driver intel_fpga_xtile_driver = {
	.probe		= intel_fpga_xtile_probe,
	.remove		= intel_fpga_xtile_remove,
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name	= INTEL_FPGA_XTILE_ETH_RESOURCE_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = intel_fpga_xtile_ll_ids,
#ifdef CONFIG_DEBUG_FS
		.dev_groups = msgdma_attr_groups,
#endif
		},
};

module_platform_driver(intel_fpga_xtile_driver);

MODULE_AUTHOR("Altera Corporation");
MODULE_DESCRIPTION("Altera HSSI based MAC driver");
MODULE_LICENSE("GPL v2");
