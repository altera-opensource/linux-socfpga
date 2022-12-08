// SPDX-License-Identifier: GPL-2.0
/* Intel FPGA Ethernet MAC driver
 * Copyright (C) 2020-2022 Intel Corporation. All rights reserved
 *
 * Contributors:
 *   Roman Bulgakov
 *   Yu Ying Choov
 *   Dalon Westergreen
 *   Joyce Ooi
 *   Arzu Ozdogan-tackin
 *
 * Original driver contributed by GlobalLogic.
 */

#include <linux/etherdevice.h>
#include <linux/if_ether.h>
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
#include <linux/ptp_classify.h>
#include <linux/skbuff.h>

#include "altera_utils.h"
#include "intel_fpga_tod.h"
#include "intel_fpga_eth_main.h"
#include "intel_fpga_eth_hssi_itf.h"

/* Module parameters */
static int debug = -1;
module_param(debug, int, MOD_PARAM_PERM);
MODULE_PARM_DESC(debug, "Message Level (-1: default, 0: no output, 16: all)");

static const u32 default_msg_level =  NETIF_MSG_DRV    |
				      NETIF_MSG_PROBE  |
				      NETIF_MSG_LINK   |
				      NETIF_MSG_IFUP   |
				      NETIF_MSG_IFDOWN;

static int flow_ctrl = FLOW_OFF;

module_param(flow_ctrl, int, MOD_PARAM_PERM);
MODULE_PARM_DESC(flow_ctrl, "Flow control (0: off, 1: rx, 2: tx, 3: on)");

static int pause = MAC_PAUSEFRAME_QUANTA;
module_param(pause, int, MOD_PARAM_PERM);
MODULE_PARM_DESC(pause, "Flow Control Pause Time");


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
#define ETH_TX_THRESH(x)	((x)->dma_priv.tx_ring_size / 4)

#define TXQUEUESTOP_THRESHOLD	2

static const struct of_device_id intel_fpga_xtile_ll_ids[];

static inline u32 xtile_tx_avail(struct intel_fpga_etile_eth_private *priv)
{
	return priv->dma_priv.tx_cons + priv->dma_priv.tx_ring_size
		- priv->dma_priv.tx_prod - 1;
}

static int xtile_init_rx_buffer(struct intel_fpga_etile_eth_private *priv,
				struct altera_dma_buffer *rxbuffer, int len)
{
	rxbuffer->skb = netdev_alloc_skb_ip_align(priv->dev, len);
	if (!rxbuffer->skb)
		return -ENOMEM;

	rxbuffer->dma_addr = dma_map_single(priv->device,
					    rxbuffer->skb->data,
					    len, DMA_FROM_DEVICE);

	if (dma_mapping_error(priv->device, rxbuffer->dma_addr)) {

		netdev_err(priv->dev, "%s: DMA mapping error\n", __func__);
		dev_kfree_skb_any(rxbuffer->skb);

		return -EINVAL;
	}
	rxbuffer->dma_addr &= (dma_addr_t)~3;
	rxbuffer->len = len;

	return 0;
}

static void xtile_free_rx_buffer(struct intel_fpga_etile_eth_private *priv,
				 struct altera_dma_buffer *rxbuffer)
{
	struct sk_buff *skb = rxbuffer->skb;
	dma_addr_t dma_addr = rxbuffer->dma_addr;

	if (skb) {
		if (dma_addr)
			dma_unmap_single(priv->device, dma_addr,
					 rxbuffer->len,
					 DMA_FROM_DEVICE);
		dev_kfree_skb_any(skb);
		rxbuffer->skb = NULL;
		rxbuffer->dma_addr = 0;
	}
}

/* Unmap and free Tx buffer resources
 */
static void xtile_free_tx_buffer(struct intel_fpga_etile_eth_private *priv,
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
		dev_kfree_skb_any(buffer->skb);
		buffer->skb = NULL;
	}
}

static int xtile_alloc_init_skbufs(struct intel_fpga_etile_eth_private *priv)
{
	unsigned int rx_descs = priv->dma_priv.rx_ring_size;
	unsigned int tx_descs = priv->dma_priv.tx_ring_size;
	int ret = -ENOMEM;
	int i;

	/* Create Rx ring buffer */
	priv->dma_priv.rx_ring = kcalloc(rx_descs,
					 sizeof(struct altera_dma_private),
					 GFP_KERNEL);
	if (!priv->dma_priv.rx_ring)
		goto err_rx_ring;

	/* Create Tx ring buffer */
	priv->dma_priv.tx_ring = kcalloc(tx_descs,
					 sizeof(struct altera_dma_private),
					 GFP_KERNEL);
	if (!priv->dma_priv.tx_ring)
		goto err_tx_ring;

	priv->dma_priv.tx_cons = 0;
	priv->dma_priv.tx_prod = 0;

	/* Init Rx FIFO */
	csrwr32(priv->rx_fifo_almost_full, priv->rx_fifo,
		rx_fifo_csroffs(almost_full_threshold));

	csrwr32(priv->rx_fifo_almost_empty, priv->rx_fifo,
		rx_fifo_csroffs(almost_empty_threshold));

	/* Init Rx ring */
	for (i = 0; i < rx_descs; i++) {
		ret = xtile_init_rx_buffer(priv, &priv->dma_priv.rx_ring[i],
					   priv->dma_priv.rx_dma_buf_sz);
		if (ret)
			goto err_init_rx_buffers;
	}

	priv->dma_priv.rx_cons = 0;
	priv->dma_priv.rx_prod = 0;

	return 0;

err_init_rx_buffers:
	while (--i >= 0)
		xtile_free_rx_buffer(priv, &priv->dma_priv.rx_ring[i]);

	kfree(priv->dma_priv.tx_ring);
err_tx_ring:
	kfree(priv->dma_priv.rx_ring);
err_rx_ring:
	return ret;
}

static void xtile_free_skbufs(struct net_device *dev)
{
	struct intel_fpga_etile_eth_private *priv = netdev_priv(dev);
	unsigned int rx_descs = priv->dma_priv.rx_ring_size;
	unsigned int tx_descs = priv->dma_priv.tx_ring_size;
	int i;

	/* Release the DMA TX/RX socket buffers */
	for (i = 0; i < rx_descs; i++)
		xtile_free_rx_buffer(priv, &priv->dma_priv.rx_ring[i]);

	for (i = 0; i < tx_descs; i++)
		xtile_free_tx_buffer(priv, &priv->dma_priv.tx_ring[i]);
}

/* Reallocate the skb for the reception process
 */
static inline void xtile_rx_refill(struct intel_fpga_etile_eth_private *priv)
{
	unsigned int rxsize = priv->dma_priv.rx_ring_size;
	unsigned int entry;
	int ret;

	for (; priv->dma_priv.rx_cons - priv->dma_priv.rx_prod > 0;
			priv->dma_priv.rx_prod++) {
		entry = priv->dma_priv.rx_prod % rxsize;
		if (likely(!priv->dma_priv.rx_ring[entry].skb)) {
			ret = xtile_init_rx_buffer(priv,
						   &priv->dma_priv.rx_ring[entry],
						   priv->dma_priv.rx_dma_buf_sz);
			if (unlikely(ret != 0))
				break;
			priv->dmaops->add_rx_desc(&priv->dma_priv,
					&priv->dma_priv.rx_ring[entry]);
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
static int xtile_rx(struct intel_fpga_etile_eth_private *priv, int limit)
{
	unsigned int count = 0;
	unsigned int next_entry;
	struct sk_buff *skb;
	unsigned int entry =
		priv->dma_priv.rx_cons % priv->dma_priv.rx_ring_size;
	u32 rxstatus;
	u16 pktlength;
	u16 pktstatus;

	while ((count < limit) &&
	       ((rxstatus = priv->dmaops->get_rx_status(&priv->dma_priv))
		!= 0)) {
		pktstatus = rxstatus >> 16;
		pktlength = rxstatus & 0xffff;

		/* DMA transfer from TSE starts with 2 additional bytes for
		 * IP payload alignment. Status returned by get_rx_status()
		 * contains DMA transfer length. Packet is 2 bytes shorter.
		 */
		//pktlength -= 2;

		count++;
		next_entry = (++priv->dma_priv.rx_cons)
			      % priv->dma_priv.rx_ring_size;

		skb = priv->dma_priv.rx_ring[entry].skb;
		if (unlikely(!skb)) {
			netdev_err(priv->dev,
				   "%s: Inconsistent Rx descriptor chain\n",
				   __func__);
			priv->dev->stats.rx_dropped++;
			break;
		}
		priv->dma_priv.rx_ring[entry].skb = NULL;
		skb_put(skb, pktlength);

		/* make cache consistent with receive packet buffer */
		dma_sync_single_for_cpu(priv->device,
					priv->dma_priv.rx_ring[entry].dma_addr,
					priv->dma_priv.rx_ring[entry].len,
					DMA_FROM_DEVICE);

		dma_unmap_single(priv->device,
				 priv->dma_priv.rx_ring[entry].dma_addr,
				 priv->dma_priv.rx_ring[entry].len,
				 DMA_FROM_DEVICE);

		if (netif_msg_pktdata(priv)) {
			netdev_info(priv->dev, "frame received %d bytes\n",
				    pktlength);
			print_hex_dump(KERN_ERR, "data: ", DUMP_PREFIX_OFFSET,
				       16, 1, skb->data, pktlength, true);
		}

		xtile_rx_vlan(priv->dev, skb);
		skb->protocol = eth_type_trans(skb, priv->dev);
		skb_checksum_none_assert(skb);
		napi_gro_receive(&priv->napi, skb);
		priv->dev->stats.rx_packets++;
		priv->dev->stats.rx_bytes += pktlength;
		entry = next_entry;
		xtile_rx_refill(priv);
	}

	return count;
}

/* Reclaim resources after transmission completes
 */
static int xtile_tx_complete(struct intel_fpga_etile_eth_private *priv)
{
	unsigned int txsize = priv->dma_priv.tx_ring_size;
	u32 ready;
	unsigned int entry;
	struct altera_dma_buffer *tx_buff;
	int txcomplete = 0;

	spin_lock(&priv->tx_lock);
	ready = priv->dmaops->tx_completions(&priv->dma_priv);

	/* Free sent buffers */
	while (ready && (priv->dma_priv.tx_cons != priv->dma_priv.tx_prod)) {
		entry = priv->dma_priv.tx_cons % txsize;
		tx_buff = &priv->dma_priv.tx_ring[entry];

		if (likely(tx_buff->skb))
			priv->dev->stats.tx_packets++;

		if (netif_msg_tx_done(priv))
			netdev_info(priv->dev, "%s: curr %d, dirty %d\n",
				    __func__, priv->dma_priv.tx_prod,
				    priv->dma_priv.tx_cons);

		xtile_free_tx_buffer(priv, tx_buff);
		priv->dma_priv.tx_cons++;

		txcomplete++;
		ready--;
	}

	if (unlikely(netif_queue_stopped(priv->dev) &&
		     xtile_tx_avail(priv) > ETH_TX_THRESH(priv))) {
		netif_tx_lock(priv->dev);
		if (netif_msg_tx_done(priv))
			netdev_info(priv->dev, "%s: restart transmit\n",
				    __func__);
		netif_wake_queue(priv->dev);
		netif_tx_unlock(priv->dev);
	}

	spin_unlock(&priv->tx_lock);

	return txcomplete;
}

/* NAPI polling function */
static int xtile_poll(struct napi_struct *napi, int budget)
{
	struct intel_fpga_etile_eth_private *priv =
			container_of(napi, struct intel_fpga_etile_eth_private, napi);
	int rxcomplete = 0;
	int txcomplete = 0;

	unsigned long flags;

	spin_lock_irqsave(&priv->rxdma_irq_lock, flags);
	disable_irq(priv->tx_irq);
	priv->dmaops->enable_txirq(&priv->dma_priv);
	spin_unlock_irqrestore(&priv->rxdma_irq_lock, flags);

	txcomplete = xtile_tx_complete(priv);

	spin_lock_irqsave(&priv->rxdma_irq_lock, flags);
	disable_irq(priv->rx_irq);
	priv->dmaops->enable_rxirq(&priv->dma_priv);
	spin_unlock_irqrestore(&priv->rxdma_irq_lock, flags);

	rxcomplete = xtile_rx(priv, budget);

	netdev_dbg(priv->dev,
		   "TX/RX complete: %d/%d of budget %d\n",
		   txcomplete, rxcomplete, budget);

	spin_lock_irqsave(&priv->rxdma_irq_lock, flags);

	if (rxcomplete >= budget || !napi_complete_done(napi, rxcomplete)) {
		priv->dmaops->disable_txirq(&priv->dma_priv);
		priv->dmaops->disable_rxirq(&priv->dma_priv);
		priv->dmaops->clear_txirq(&priv->dma_priv);
		priv->dmaops->clear_rxirq(&priv->dma_priv);
	}

	enable_irq(priv->tx_irq);
	enable_irq(priv->rx_irq);

	spin_unlock_irqrestore(&priv->rxdma_irq_lock, flags);


	return rxcomplete;
}

/* DMA TX & RX FIFO interrupt routing
 */
static irqreturn_t intel_fpga_xtile_isr(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct intel_fpga_etile_eth_private *priv;

	if (unlikely(!dev)) {
		pr_err("%s: invalid dev_id\n", __func__);
		return IRQ_NONE;
	}
	priv = netdev_priv(dev);

	if (unlikely(irq != priv->tx_irq && irq != priv->rx_irq)) {
		pr_err("%s: invalid irq\n", __func__);
		return IRQ_NONE;
	}


	if (unlikely(netif_msg_intr(priv)))
	   netdev_info(dev, "%s interrupt\n", (irq == priv->rx_irq) ? "RX" : "TX");

	spin_lock(&priv->rxdma_irq_lock);

	if (likely(napi_schedule_prep(&priv->napi))) {
		priv->dmaops->disable_rxirq(&priv->dma_priv);
		priv->dmaops->disable_txirq(&priv->dma_priv);
		__napi_schedule(&priv->napi);
	}

	priv->dmaops->clear_rxirq(&priv->dma_priv);
	priv->dmaops->clear_txirq(&priv->dma_priv);

	spin_unlock(&priv->rxdma_irq_lock);


	return IRQ_HANDLED;
}

int xtile_check_counter_complete(struct intel_fpga_etile_eth_private *priv, u32 regbank,
				 size_t offs, u8 bit_mask, bool set_bit,
				 int align)
{
	int counter;
	u32 chan = priv->chan;
	struct platform_device *pdev = priv->pdev_hssi;

	counter = 0;
	switch (align) {
	case 8: /* byte aligned */
		while (counter++ < INTEL_FPGA_XTILE_SW_RESET_WATCHDOG_CNTR) {
			if (set_bit) {
				if (hssi_csrrd8(pdev, regbank, chan, offs) & bit_mask)
					break;
			} else {
				if ((hssi_csrrd8(pdev, regbank, chan, offs) & bit_mask) == 0)
					break;
			}
		}
		if (counter >= INTEL_FPGA_XTILE_SW_RESET_WATCHDOG_CNTR) {
			if (set_bit) {
				if ((hssi_csrrd8(pdev, regbank, chan, offs) & bit_mask) == 0)
					return -EINVAL;
			} else {
				if (hssi_csrrd8(pdev, regbank, chan, offs) & bit_mask)
					return -EINVAL;
			}
		}
		break;
	default: /* default is word aligned */
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
		break;
	}
	return 0;
}

static void xtile_clear_mac_statistics(struct platform_device *pdev, u32 chan)
{
	bool is_tx_reset = true;
	bool is_rx_reset = true;

	/* Clear all statistics counters for the receive and transmit path */
	hssi_reset_mac_stats(pdev, chan, is_tx_reset, is_rx_reset);
}

/* Open and initialize the interface
 */
static int xtile_open(struct net_device *dev)
{
	struct intel_fpga_etile_eth_private *priv = netdev_priv(dev);
	struct platform_device *pdev = priv->pdev_hssi;
	bool eth_portstatus = false;
	u32 chan = priv->chan;
	unsigned long flags;
	int ret = 0;
	int i;

	/* Check Ethernet Port X status, if the port is not stable then  */
	/* no use to continue further					 */
	eth_portstatus = hssi_ethport_is_stable(pdev, chan);
	if (eth_portstatus == false) {
		ret = -EREMOTEIO;
		goto phy_error;
	}

	/* Create and initialize the TX/RX descriptors chains. */
	priv->dma_priv.rx_ring_size = dma_rx_num;
	priv->dma_priv.tx_ring_size = dma_tx_num;

	/* Allocate Tx and Rx descriptor ring and initialize respectively */
	ret = priv->dmaops->init_dma(&priv->dma_priv);
	if (ret) {
		netdev_err(dev, "Cannot initialize DMA\n");
		goto phy_error;
	}

	/*  Initialize the MAC layer */
	ret = init_mac(priv);

	if (ret)
	{
	  netdev_dbg(dev, "Tx datapath config (error: %d)\n", ret);
	  goto alloc_skbuf_error;
	}

	if (netif_msg_ifup(priv))
		netdev_info(dev, "device MAC address %pM\n",
			    dev->dev_addr);

	/* clear the MAC layer statistics to start afresh */
	xtile_clear_mac_statistics(pdev, chan);

	/* Reset the mSGDMA engine and configure the mSGDMA register to           */
	/* provide information of the Tx and Rx DMA ring start address in memory  */
	priv->dmaops->reset_dma(&priv->dma_priv);

	ret = xtile_alloc_init_skbufs(priv);
	if (ret) {
		netdev_err(dev, "DMA descriptors initialization failed\n");
		goto alloc_skbuf_error;
	}

	/* Register RX interrupt */
	ret = devm_request_irq(priv->device, priv->rx_irq, intel_fpga_xtile_isr,
			       IRQF_SHARED, dev->name, dev);
	if (ret) {
		netdev_err(dev, "Unable to register RX interrupt %d\n",
			   priv->rx_irq);
		goto init_error;
	}

	/* Register TX interrupt */
	ret = devm_request_irq(priv->device, priv->tx_irq, intel_fpga_xtile_isr,
			       IRQF_SHARED, dev->name, dev);
	if (ret)
	{
		netdev_err(dev, "Unable to register TX interrupt %d\n",
			   priv->tx_irq);
		goto init_error;
	}

	/* Enable DMA interrupts */
	spin_lock_irqsave(&priv->rxdma_irq_lock, flags);
	priv->dmaops->enable_rxirq(&priv->dma_priv);
	priv->dmaops->enable_txirq(&priv->dma_priv);

	/* Setup RX descriptor chain */
	for (i = 0; i < priv->dma_priv.rx_ring_size; i++)
		priv->dmaops->add_rx_desc(&priv->dma_priv,
					  &priv->dma_priv.rx_ring[i]);

	spin_unlock_irqrestore(&priv->rxdma_irq_lock, flags);
	napi_enable(&priv->napi);
	netif_start_queue(dev);

	if (priv->dmaops->start_txdma)
	   priv->dmaops->start_txdma(&priv->dma_priv);

	priv->dmaops->start_rxdma(&priv->dma_priv);

	if (priv->phylink)
		phylink_start(priv->phylink);

	return 0;

init_error:
	xtile_free_skbufs(dev);
	xtile_free_skbufs(dev);
phy_error:

alloc_skbuf_error:
	return ret;
}

/* Stop TSE MAC interface and put the device in an inactive state
 */
static int xtile_shutdown(struct net_device *dev)
{
	struct intel_fpga_etile_eth_private *priv = netdev_priv(dev);
	unsigned long flags;


	/* Stop the PHY */
	if (priv->phylink)
		phylink_stop(priv->phylink);

	netif_stop_queue(dev);
	napi_disable(&priv->napi);

	/* Disable DMA interrupts */
	spin_lock_irqsave(&priv->rxdma_irq_lock, flags);
	priv->dmaops->disable_rxirq(&priv->dma_priv);
	priv->dmaops->disable_txirq(&priv->dma_priv);
	spin_unlock_irqrestore(&priv->rxdma_irq_lock, flags);

	/* Unregister RX interrupt */
	devm_free_irq(priv->device, priv->rx_irq, dev);

	/* Unregister TX interrupt */
	devm_free_irq(priv->device, priv->tx_irq, dev);

	/* disable and reset the MAC, empties fifo */
	/* Trigger RX digital reset */
	pma_reset(priv, false, true);

	priv->dmaops->reset_dma(&priv->dma_priv);
	xtile_free_skbufs(dev);

	priv->dmaops->uninit_dma(&priv->dma_priv);
	del_timer_sync(&priv->fec_timer);

	return 0;
}

/* Transmit a packet (called by the kernel). Dispatches
 * either the SGDMA method for transmitting or the
 * MSGDMA method, assumes no scatter/gather support,
 * implying an assumption that there's only one
 * physically contiguous fragment starting at
 * skb->data, for length of skb_headlen(skb).
 */

static int xtile_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	enum netdev_tx ret = NETDEV_TX_OK;
	struct intel_fpga_etile_eth_private *priv = netdev_priv(dev);
	unsigned int txsize = priv->dma_priv.tx_ring_size;
	unsigned int entry;
	struct altera_dma_buffer *buffer = NULL;
	int nfrags = skb_shinfo(skb)->nr_frags;
	unsigned int nopaged_len = skb_headlen(skb);
	dma_addr_t dma_addr;

	if (nopaged_len < 60)
	  nopaged_len = 60; // pad with dummy bytes, DMA irq will stop otherwise

	spin_lock_bh(&priv->tx_lock);

	if (unlikely(xtile_tx_avail(priv) < nfrags + 1)) {
		if (!netif_queue_stopped(dev)) {
			netif_stop_queue(dev);
			/* This is a hard error, log it. */
			netdev_err(priv->dev,
				   "%s: Tx list full when queue awake\n",
				   __func__);
		}
		ret = NETDEV_TX_BUSY;
		netdev_info(priv->dev, "xmit NETDEV_TX_BUSY\n");
		goto out;
	}

	if (unlikely(netif_msg_tx_queued(priv))) {
		netdev_info(dev, "sending 0x%p, len=%d\n", skb, skb->len);
		if (netif_msg_pktdata(priv))
			print_hex_dump(KERN_ERR, "data: ", DUMP_PREFIX_OFFSET,
				       16, 1, skb->data, skb->len, true);
	}

	/* Map the first skb fragment */
	entry = priv->dma_priv.tx_prod % txsize;
	buffer = &priv->dma_priv.tx_ring[entry];

	dma_addr = dma_map_single(priv->device, skb->data, nopaged_len,
				  DMA_TO_DEVICE);
	if (dma_mapping_error(priv->device, dma_addr)) {
		netdev_err(priv->dev, "%s: DMA mapping error\n", __func__);
		dev_kfree_skb_any(skb);
		ret = -EINVAL;
		goto out;
	}

	buffer->skb = skb;
	buffer->dma_addr = dma_addr;
	buffer->len = nopaged_len;

	/* Push data out of the cache hierarchy into main memory */
	dma_sync_single_for_device(priv->device, buffer->dma_addr,
				   buffer->len, DMA_TO_DEVICE);

	priv->dmaops->tx_buffer(&priv->dma_priv, buffer);

	/* Provide a hardware time stamp if requested.
	 */
	if (unlikely((skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) &&
		     priv->dma_priv.hwts_tx_en))
		/* declare that device is doing timestamping */
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;

	/* Provide a software time stamp if requested and hardware timestamping
	 * is not possible (SKBTX_IN_PROGRESS not set).
	 */
	if (!priv->dma_priv.hwts_tx_en)
		skb_tx_timestamp(skb);

	priv->dma_priv.tx_prod++;
	dev->stats.tx_bytes += skb->len;

	if (unlikely(xtile_tx_avail(priv) <= TXQUEUESTOP_THRESHOLD)) {
		if (netif_msg_hw(priv))
			netdev_info(priv->dev, "%s: stop transmitted packets\n",
				    __func__);
		netif_stop_queue(dev);
	}

out:
	spin_unlock_bh(&priv->tx_lock);

	return ret;
}

/* Control hardware timestamping.
 * This function configures the MAC to enable/disable both outgoing(TX)
 * and incoming(RX) packets time stamping based on user input.
 */
static int xtile_set_hwtstamp_config(struct net_device *dev, struct ifreq *ifr)
{
	int ret = 0;
	struct intel_fpga_etile_eth_private *priv = netdev_priv(dev);
	struct hwtstamp_config config;

	if (copy_from_user(&config, ifr->ifr_data,
			   sizeof(struct hwtstamp_config)))
		return -EFAULT;

	netif_info(priv, drv, dev,
		   "%s config flags:0x%x, tx_type:0x%x, rx_filter:0x%x\n",
		   __func__, config.flags, config.tx_type, config.rx_filter);

	/* reserved for future extensions */
	if (config.flags)
		return -EINVAL;

	switch (config.tx_type) {
	case HWTSTAMP_TX_ON:
		priv->dma_priv.hwts_tx_en = 1;
		break;
	default:
		return -ERANGE;
	}

	switch (config.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		priv->dma_priv.hwts_rx_en = 0;
		config.rx_filter = HWTSTAMP_FILTER_NONE;
		break;
	default:
		priv->dma_priv.hwts_rx_en = 1;
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
	struct intel_fpga_etile_eth_private *priv = netdev_priv(dev);
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

static const struct net_device_ops intel_fpga_xtile_netdev_ops = {
	.ndo_open		= xtile_open,
	.ndo_stop		= xtile_shutdown,
	.ndo_start_xmit		= xtile_start_xmit,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_set_rx_mode	= xtile_set_rx_mode,
	.ndo_change_mtu		= xtile_change_mtu,
	.ndo_do_ioctl		= xtile_do_ioctl,
	.ndo_get_stats64	= xtile_get_stats64
};

static void intel_fpga_xtile_validate(struct phylink_config *config,
				      unsigned long *supported,
				      struct phylink_link_state *state)
{
	__ETHTOOL_DECLARE_LINK_MODE_MASK(mac_supported) = { 0, };
	__ETHTOOL_DECLARE_LINK_MODE_MASK(mask) = { 0, };

	if (state->interface != PHY_INTERFACE_MODE_NA &&
	    state->interface != PHY_INTERFACE_MODE_10GKR &&
	    state->interface != PHY_INTERFACE_MODE_10GBASER &&
	    state->interface != PHY_INTERFACE_MODE_25GKR) {
		bitmap_zero(supported, __ETHTOOL_LINK_MODE_MASK_NBITS);
		return;
	}

	/* Allow all the expected bits */
	phylink_set(mask, Autoneg);
	phylink_set(mask, Pause);
	phylink_set(mask, Asym_Pause);
	phylink_set_port_modes(mask);
	phylink_set(mac_supported, Autoneg);
	phylink_set(mac_supported, Pause);
	phylink_set(mac_supported, Asym_Pause);
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
}

static void intel_fpga_xtile_mac_pcs_get_state(struct phylink_config *config,
					       struct phylink_link_state *state)
{
	/* fixed speed for now */
	state->speed = SPEED_10000;
	state->duplex = DUPLEX_FULL;
	state->link = 1;

}

static void intel_fpga_xtile_mac_an_restart(struct phylink_config *config)
{
	/* Not Supported */
}

static void intel_fpga_xtile_get_pcs_fixed_state(struct phylink_config *config,
						 struct phylink_link_state *state)
{
	struct intel_fpga_etile_eth_private *priv =
		netdev_priv(to_net_dev(config->dev));

	if (!priv)
		return;

	state->speed = priv->link_speed;
	state->duplex = DUPLEX_FULL;
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
	struct intel_fpga_etile_eth_private *priv =
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
	struct intel_fpga_etile_eth_private *priv =
			netdev_priv(to_net_dev(config->dev));

	phylink_mac_change(priv->phylink, true);
}

static const struct phylink_mac_ops intel_fpga_xtile_phylink_ops = {
	.validate = intel_fpga_xtile_validate,
	.mac_pcs_get_state = intel_fpga_xtile_mac_pcs_get_state,
	.mac_an_restart = intel_fpga_xtile_mac_an_restart,
	.mac_config = intel_fpga_xtile_mac_config,
	.mac_link_down = intel_fpga_xtile_mac_link_down,
	.mac_link_up = intel_fpga_xtile_mac_link_up,
};

/* Probe MAC device */
static int intel_fpga_xtile_probe(struct platform_device *pdev)
{
	int ret = -ENODEV;
	struct device_node *np;
	struct net_device *ndev;
	struct resource *rx_fifo;
	struct device_node *dev_hssi;
	const unsigned char *macaddr;
	struct fwnode_handle *fixed_node;
	struct platform_device *pdev_hssi;
	const struct of_device_id *of_id = NULL;
	struct intel_fpga_etile_eth_private *priv;


	/* Get the HSSI node device from the device tree node */
	dev_hssi = of_parse_phandle(pdev->dev.of_node, "hssiss", 0);
	if (!dev_hssi) {
		return -ENOENT;
	}

	pdev_hssi = of_find_device_by_node(dev_hssi);
	if (!pdev) {
		of_node_put(dev_hssi);
		return -ENODEV;
	}

	np = pdev->dev.of_node;

	ndev = alloc_etherdev(sizeof(struct intel_fpga_etile_eth_private));
	if (!ndev) {
		dev_err(&pdev->dev, "Could not allocate network device\n");
		return -ENODEV;
	}

	SET_NETDEV_DEV(ndev, &pdev->dev);

	priv = netdev_priv(ndev);

	priv->dev	      = ndev;
	priv->flow_ctrl	      = flow_ctrl;
	priv->pause	      = pause;
	priv->pdev_hssi       = pdev_hssi;
	priv->device          = &pdev->dev;
	priv->dma_priv.dev    = ndev;
	priv->ptp_priv.dev    = ndev;
	priv->dma_priv.device = &pdev->dev;
	priv->msg_enable      = netif_msg_init(debug, default_msg_level);
	priv->dma_priv.msg_enable = netif_msg_init(debug, default_msg_level);

	priv->phylink_config.dev = &priv->dev->dev;
	priv->phylink_config.type = PHYLINK_NETDEV;
	priv->phylink_config.get_fixed_state = intel_fpga_xtile_get_pcs_fixed_state;

	of_id = of_match_device(intel_fpga_xtile_ll_ids, &pdev->dev);

	/* get hssi channel no from device tree */
	if (of_property_read_u32(np, "chan",
				 &priv->chan)) {

		dev_err(&pdev->dev, "cannot obtain chan\n");
		ret = -ENXIO;
		goto err_free_netdev;
	}
	if (of_id)
		priv->dmaops = (struct altera_dmaops *)of_id->data;

	/* PTP is only supported with a modified MSGDMA */
	priv->ptp_enable = of_property_read_bool(pdev->dev.of_node,
						 "altr,has-ptp");
	if (priv->ptp_enable &&
	    priv->dmaops->altera_dtype != ALTERA_DTYPE_MSGDMA_PREF) {
		dev_err(&pdev->dev, "PTP requires modified dma\n");
		ret = -ENODEV;
		goto err_free_netdev;
	}

	/* mSGDMA Tx IRQ */
	priv->tx_irq = platform_get_irq_byname(pdev, "tx_irq");
	if (priv->tx_irq == -ENXIO) {
		dev_err(&pdev->dev, "cannot obtain Tx IRQ\n");
		ret = -ENXIO;
		goto err_free_netdev;
	}

	/* mSGDMA Rx IRQ */
	priv->rx_irq = platform_get_irq_byname(pdev, "rx_irq");
	if (priv->rx_irq == -ENXIO) {
		dev_err(&pdev->dev, "cannot obtain Rx IRQ\n");
		ret = -ENXIO;
		goto err_free_netdev;
	}

	/* Map DMA */
	ret = altera_eth_dma_probe(pdev, &priv->dma_priv,
				   priv->dmaops->altera_dtype);
	if (ret) {
		dev_err(&pdev->dev, "cannot map DMA\n");
		goto err_free_netdev;
	}
	/* Rx Fifo */
	ret = request_and_map(pdev, "rx_fifo", &rx_fifo,
			      (void __iomem **)&priv->rx_fifo);
	if (ret)
		goto err_free_netdev;

	if (netif_msg_probe(priv))
		dev_info(&pdev->dev, "\tRX FIFO  at 0x%08lx\n",
			 (unsigned long)rx_fifo->start);

#if 0
	/* we only support ptp with the msgdma */
	if (priv->ptp_enable) {
		/* MAP PTP */
		ret = intel_fpga_tod_probe(pdev, &priv->ptp_priv);
		if (ret) {
			dev_err(&pdev->dev, "cannot map PTP\n");
			goto err_free_netdev;
		}
	}
#endif
	if (!dma_set_mask(priv->device, DMA_BIT_MASK(priv->dmaops->dmamask)))
		dma_set_coherent_mask(priv->device,
				      DMA_BIT_MASK(priv->dmaops->dmamask));
	else if (!dma_set_mask(priv->device, DMA_BIT_MASK(32)))
		dma_set_coherent_mask(priv->device, DMA_BIT_MASK(32));
	else
		goto err_free_netdev;

	/* get FIFO depths from device tree */
	if (of_property_read_u32(pdev->dev.of_node, "rx-fifo-depth",
				 &priv->rx_fifo_depth)) {
		dev_err(&pdev->dev, "cannot obtain rx-fifo-depth\n");
		ret = -ENXIO;
		goto err_free_netdev;
	}

	if (of_property_read_u32(pdev->dev.of_node, "tx-fifo-depth",
				 &priv->tx_fifo_depth)) {
		dev_err(&pdev->dev, "cannot obtain tx-fifo-depth\n");
		ret = -ENXIO;
		goto err_free_netdev;
	}

	if (of_property_read_u32(pdev->dev.of_node, "rx-fifo-almost-full",
				 &priv->rx_fifo_almost_full)) {
		dev_err(&pdev->dev, "cannot obtain rx-fifo-almost-full\n");
		priv->rx_fifo_almost_full = 0x4000;
	}

	if (of_property_read_u32(pdev->dev.of_node, "rx-fifo-almost-empty",
				 &priv->rx_fifo_almost_empty)) {
		dev_err(&pdev->dev, "cannot obtain rx-fifo-almost-empty\n");
		priv->rx_fifo_almost_empty = 0x3000;
	}

	priv->dev->min_mtu = ETH_ZLEN + ETH_FCS_LEN;

	/* Max MTU is 1500, ETH_DATA_LEN */
	priv->dev->max_mtu = ETH_DATA_LEN;

	/* Get the max mtu from the device tree. Note that the
	 * "max-frame-size" parameter is actually max mtu. Definition
	 * in the ePAPR v1.1 spec and usage differ, so go with usage.
	 */
	of_property_read_u32(pdev->dev.of_node, "max-frame-size",
			     &priv->dev->max_mtu);

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

	/* get default MAC address from device tree */
	macaddr = of_get_mac_address(pdev->dev.of_node);
	if (!IS_ERR(macaddr))
		ether_addr_copy(ndev->dev_addr, macaddr);
	else
		eth_hw_addr_random(ndev);

	/* initialize netdev */
	ndev->netdev_ops = &intel_fpga_xtile_netdev_ops;
	intel_fpga_xtile_set_ethtool_ops(ndev);

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

	/* setup NAPI interface */
	netif_napi_add(ndev, &priv->napi, xtile_poll, NAPI_POLL_WEIGHT);

	spin_lock_init(&priv->tx_lock);
	spin_lock_init(&priv->rxdma_irq_lock);
	spin_lock_init(&priv->ptp_priv.tod_lock);
	spin_lock_init(&priv->mac_cfg_lock);

	/* check if phy-mode is present */
	ret = of_get_phy_mode(np, &priv->phy_iface);
	if (ret) {
		dev_err(&pdev->dev, "incorrect phy-mode\n");
		goto err_free_netdev;
	}

	/* create phylink */
	priv->phylink = phylink_create(&priv->phylink_config, pdev->dev.fwnode,
				       priv->phy_iface, &intel_fpga_xtile_phylink_ops);
	if (IS_ERR(priv->phylink)) {
		dev_err(&pdev->dev, "failed to create phylink\n");
		ret = PTR_ERR(priv->phylink);
		goto err_free_netdev;
	}

	/* read the fixed link properties*/
	fixed_node = fwnode_get_named_child_node(pdev->dev.fwnode, "fixed-link");
	if (fixed_node) {
		fwnode_property_read_u32(fixed_node, "speed", &priv->link_speed);
		priv->duplex = DUPLEX_FULL;

		dev_info(&pdev->dev, "\tfixed link speed:%d full duplex:%d\n",
			 priv->link_speed, priv->duplex);
	} else {
		dev_err(&pdev->dev, "fixed link property undefined\n");
		goto err_free_netdev;
	}

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register ethernet device\n");
		goto err_register_netdev;
	}

	platform_set_drvdata(pdev, ndev);

#if 0
	if (priv->ptp_enable) {
		ret = intel_fpga_tod_register(&priv->ptp_priv, priv->device);
		if (ret) {
			dev_err(&pdev->dev, "Unable to register PTP clock\n");
			ret = -ENXIO;
			goto err_init_phy;
		}
	}

#endif
	ret = fec_init(pdev, priv);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to init FEC\n");
		ret = -ENXIO;
		goto err_init_phy;
	}

	return 0;
err_init_phy:
	unregister_netdev(ndev);
err_register_netdev:
	netif_napi_del(&priv->napi);
err_free_netdev:
	free_netdev(ndev);
	return ret;
}

/* Remove MAC device */
static int intel_fpga_xtile_remove(struct platform_device *pdev)
{
	struct platform_device *pdev_hssi;
	struct intel_fpga_etile_eth_private *priv;
	struct net_device *ndev;

	/* Get the HSSI node device from the node */
	struct device_node *dev_hssi = of_parse_phandle(pdev->dev.of_node, "hssiss", 0);

	if (!dev_hssi)
	{
	  return -ENOENT;
	}

	pdev_hssi = of_find_device_by_node(dev_hssi);
	if (!pdev)
	{
	  of_node_put(dev_hssi);
	  return -ENODEV;
	}

	ndev = platform_get_drvdata(pdev);
	priv = netdev_priv(ndev);

#if 0
	if (priv->ptp_enable)
		intel_fpga_tod_unregister(&priv->ptp_priv);
#endif

	platform_set_drvdata(pdev, NULL);
	unregister_netdev(ndev);
	free_netdev(ndev);
	return 0;
}

static const struct altera_dmaops altera_dtype_prefetcher = {
	.altera_dtype   = ALTERA_DTYPE_MSGDMA_PREF,
	.dmamask        = 64,
	.reset_dma      = msgdma_pref_reset,
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

static const struct of_device_id intel_fpga_xtile_ll_ids[] = {
	{ .compatible = "altr,hssi-etile-1.0",
	  .data = &altera_dtype_prefetcher,
	},
	{},
};

MODULE_DEVICE_TABLE(of, intel_fpga_xtile_ll_ids);

static struct platform_driver intel_fpga_xtile_driver = {
	.probe		= intel_fpga_xtile_probe,
	.remove		= intel_fpga_xtile_remove,
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name	= INTEL_FPGA_ETILE_ETH_RESOURCE_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = intel_fpga_xtile_ll_ids,
		},
};

module_platform_driver(intel_fpga_xtile_driver);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("Altera HSSI based MAC driver");
MODULE_LICENSE("GPL v2");
