// SPDX-License-Identifier: GPL-2.0
/* Intel FPGA E-tile Ethernet MAC driver
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

#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/if_ether.h>
#include <linux/if_vlan.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/net_tstamp.h>
#include <linux/netdevice.h>
#include <linux/of_device.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/phylink.h>
#include <linux/ptp_classify.h>
#include <linux/sfp.h>
#include <linux/skbuff.h>
#include <asm/cacheflush.h>

#include "altera_eth_dma.h"
#include "altera_msgdma.h"
#include "altera_msgdma_prefetcher.h"
#include "altera_sgdma.h"
#include "altera_utils.h"
#include "intel_fpga_tod.h"
#include "intel_fpga_etile.h"

/* Module parameters */
static int debug = -1;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Message Level (-1: default, 0: no output, 16: all)");

static const u32 default_msg_level = (NETIF_MSG_DRV | NETIF_MSG_PROBE |
					NETIF_MSG_LINK | NETIF_MSG_IFUP |
					NETIF_MSG_IFDOWN);

#define RX_DESCRIPTORS 512
static int dma_rx_num = RX_DESCRIPTORS;
module_param(dma_rx_num, int, 0644);
MODULE_PARM_DESC(dma_rx_num, "Number of descriptors in the RX list");

#define TX_DESCRIPTORS 512
static int dma_tx_num = TX_DESCRIPTORS;
module_param(dma_tx_num, int, 0644);
MODULE_PARM_DESC(dma_tx_num, "Number of descriptors in the TX list");

static int flow_ctrl = FLOW_OFF;
module_param(flow_ctrl, int, 0644);
MODULE_PARM_DESC(flow_ctrl, "Flow control (0: off, 1: rx, 2: tx, 3: on)");

static int pause = MAC_PAUSEFRAME_QUANTA;
module_param(pause, int, 0644);
MODULE_PARM_DESC(pause, "Flow Control Pause Time");

#define INTEL_FPGA_BYTE_ALIGN	8
#define INTEL_FPGA_WORD_ALIGN	32

#define INTEL_FPGA_COAL_TIMER(x)	(jiffies + usecs_to_jiffies(x))

static const struct of_device_id intel_fpga_etile_ll_ids[];

/* Allow network stack to resume queueing packets after we've
 * finished transmitting at least 1/4 of the packets in the queue.
 */
#define ETH_TX_THRESH(x)	((x)->dma_priv.tx_ring_size / 4)

#define TXQUEUESTOP_THRESHOLD	2

static inline u32 etile_tx_avail(struct intel_fpga_etile_eth_private *priv)
{
	return priv->dma_priv.tx_cons + priv->dma_priv.tx_ring_size
		- priv->dma_priv.tx_prod - 1;
}

static int etile_init_rx_buffer(struct intel_fpga_etile_eth_private *priv,
				struct altera_dma_buffer *rxbuffer, int len)
{
	rxbuffer->skb = netdev_alloc_skb_ip_align(priv->dev, len);
	if (!rxbuffer->skb)
		return -ENOMEM;

	rxbuffer->dma_addr = dma_map_single(priv->device, rxbuffer->skb->data,
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

static void etile_free_rx_buffer(struct intel_fpga_etile_eth_private *priv,
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
static void etile_free_tx_buffer(struct intel_fpga_etile_eth_private *priv,
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

static int etile_alloc_init_skbufs(struct intel_fpga_etile_eth_private *priv)
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
		ret = etile_init_rx_buffer(priv, &priv->dma_priv.rx_ring[i],
					   priv->dma_priv.rx_dma_buf_sz);
		if (ret)
			goto err_init_rx_buffers;
	}

	priv->dma_priv.rx_cons = 0;
	priv->dma_priv.rx_prod = 0;

	return 0;
err_init_rx_buffers:
	while (--i >= 0)
		etile_free_rx_buffer(priv, &priv->dma_priv.rx_ring[i]);
	kfree(priv->dma_priv.tx_ring);
err_tx_ring:
	kfree(priv->dma_priv.rx_ring);
err_rx_ring:
	return ret;
}

static void etile_free_skbufs(struct net_device *dev)
{
	struct intel_fpga_etile_eth_private *priv = netdev_priv(dev);
	unsigned int rx_descs = priv->dma_priv.rx_ring_size;
	unsigned int tx_descs = priv->dma_priv.tx_ring_size;
	int i;

	/* Release the DMA TX/RX socket buffers */
	for (i = 0; i < rx_descs; i++)
		etile_free_rx_buffer(priv, &priv->dma_priv.rx_ring[i]);
	for (i = 0; i < tx_descs; i++)
		etile_free_tx_buffer(priv, &priv->dma_priv.tx_ring[i]);
}

/* Reallocate the skb for the reception process
 */
static inline void etile_rx_refill(struct intel_fpga_etile_eth_private *priv)
{
	unsigned int rxsize = priv->dma_priv.rx_ring_size;
	unsigned int entry;
	int ret;

	for (; priv->dma_priv.rx_cons - priv->dma_priv.rx_prod > 0;
			priv->dma_priv.rx_prod++) {
		entry = priv->dma_priv.rx_prod % rxsize;
		if (likely(!priv->dma_priv.rx_ring[entry].skb)) {
			ret = etile_init_rx_buffer(priv,
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
static inline void etile_rx_vlan(struct net_device *dev, struct sk_buff *skb)
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
static int etile_rx(struct intel_fpga_etile_eth_private *priv, int limit)
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

		if ((pktstatus & 0xff) || pktlength == 0)
			netdev_err(priv->dev,
				   "RCV pktstatus %08X pktlength %08X\n",
				   pktstatus, pktlength);

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

		etile_rx_vlan(priv->dev, skb);
		skb->protocol = eth_type_trans(skb, priv->dev);
		skb_checksum_none_assert(skb);
		napi_gro_receive(&priv->napi, skb);
		priv->dev->stats.rx_packets++;
		priv->dev->stats.rx_bytes += pktlength;
		entry = next_entry;
		etile_rx_refill(priv);
	}

	return count;
}

/* Reclaim resources after transmission completes
 */
int etile_tx_complete(struct intel_fpga_etile_eth_private *priv)
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

		etile_free_tx_buffer(priv, tx_buff);
		priv->dma_priv.tx_cons++;

		txcomplete++;
		ready--;
	}

	if (unlikely(netif_queue_stopped(priv->dev) &&
		     etile_tx_avail(priv) > ETH_TX_THRESH(priv))) {
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

/* NAPI polling function
 */
int etile_poll(struct napi_struct *napi, int budget)
{
	struct intel_fpga_etile_eth_private *priv =
			container_of(napi, struct intel_fpga_etile_eth_private, napi);
	int rxcomplete = 0;
	unsigned long flags;

	etile_tx_complete(priv);

	rxcomplete = etile_rx(priv, budget);

	if (rxcomplete < budget) {
		napi_complete_done(napi, rxcomplete);
		netdev_dbg(priv->dev,
			   "NAPI Complete, did %d packets with budget %d\n",
			   rxcomplete, budget);
		spin_lock_irqsave(&priv->rxdma_irq_lock, flags);
		priv->dmaops->enable_rxirq(&priv->dma_priv);
		priv->dmaops->enable_txirq(&priv->dma_priv);
		spin_unlock_irqrestore(&priv->rxdma_irq_lock, flags);
	}

	return rxcomplete;
}

/* DMA TX & RX FIFO interrupt routing
 */
irqreturn_t intel_fpga_etile_isr(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct intel_fpga_etile_eth_private *priv;

	if (unlikely(!dev)) {
		pr_err("%s: invalid dev pointer\n", __func__);
		return IRQ_NONE;
	}
	priv = netdev_priv(dev);

	if (unlikely(netif_msg_intr(priv)))
		netdev_info(dev, "Got TX/RX Interrupt");

	spin_lock(&priv->rxdma_irq_lock);
	/* reset IRQs */
	priv->dmaops->clear_rxirq(&priv->dma_priv);
	priv->dmaops->clear_txirq(&priv->dma_priv);
	spin_unlock(&priv->rxdma_irq_lock);

	if (likely(napi_schedule_prep(&priv->napi))) {
		spin_lock(&priv->rxdma_irq_lock);
		priv->dmaops->disable_rxirq(&priv->dma_priv);
		priv->dmaops->disable_txirq(&priv->dma_priv);
		spin_unlock(&priv->rxdma_irq_lock);
		__napi_schedule(&priv->napi);
	}

	return IRQ_HANDLED;
}

/* Transmit a packet (called by the kernel). Dispatches
 * either the SGDMA method for transmitting or the
 * MSGDMA method, assumes no scatter/gather support,
 * implying an assumption that there's only one
 * physically contiguous fragment starting at
 * skb->data, for length of skb_headlen(skb).
 */
int etile_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct intel_fpga_etile_eth_private *priv = netdev_priv(dev);
	unsigned int txsize = priv->dma_priv.tx_ring_size;
	unsigned int entry;
	struct altera_dma_buffer *buffer = NULL;
	int nfrags = skb_shinfo(skb)->nr_frags;
	unsigned int nopaged_len = skb_headlen(skb);
	enum netdev_tx ret = NETDEV_TX_OK;
	dma_addr_t dma_addr;

	spin_lock_bh(&priv->tx_lock);

	if (unlikely(etile_tx_avail(priv) < nfrags + 1)) {
		if (!netif_queue_stopped(dev)) {
			netif_stop_queue(dev);
			/* This is a hard error, log it. */
			netdev_err(priv->dev,
				   "%s: Tx list full when queue awake\n",
				   __func__);
		}
		ret = NETDEV_TX_BUSY;
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

	if (unlikely(etile_tx_avail(priv) <= TXQUEUESTOP_THRESHOLD)) {
		if (netif_msg_hw(priv))
			netdev_info(priv->dev, "%s: stop transmitted packets\n",
				    __func__);
		netif_stop_queue(dev);
	}

out:
	spin_unlock_bh(&priv->tx_lock);

	return ret;
}

static int check_counter_complete(void __iomem *ioaddr,
				  size_t offs, u32 bit_mask, bool set_bit,
				  int align)
{
	int counter;

	counter = 0;
	switch (align) {
	case 8: /* byte aligned */
		while (counter++ < INTEL_FPGA_ETILE_SW_RESET_WATCHDOG_CNTR) {
			if (set_bit) {
				if (csrrd8(ioaddr, offs) & bit_mask)
					break;
			} else {
				if ((csrrd8(ioaddr, offs) & bit_mask) == 0)
					break;
			}
			udelay(1);
		}
		if (counter >= INTEL_FPGA_ETILE_SW_RESET_WATCHDOG_CNTR) {
			if (set_bit) {
				if ((csrrd8(ioaddr, offs) & bit_mask) == 0)
					return -EINVAL;
			} else {
				if (csrrd8(ioaddr, offs) & bit_mask)
					return -EINVAL;
			}
		}
		break;
	default: /* default is word aligned */
		while (counter++ < INTEL_FPGA_ETILE_SW_RESET_WATCHDOG_CNTR) {
			if (set_bit) {
				if (tse_bit_is_set(ioaddr,
						   offs, bit_mask))
					break;
			} else {
				if (tse_bit_is_clear(ioaddr,
						     offs, bit_mask))
					break;
			}
			udelay(1);
		}
		if (counter >= INTEL_FPGA_ETILE_SW_RESET_WATCHDOG_CNTR) {
			if (set_bit) {
				if (tse_bit_is_clear(ioaddr,
						     offs, bit_mask))
					return -EINVAL;
			} else {
				if (tse_bit_is_set(ioaddr,
						   offs, bit_mask))
					return -EINVAL;
			}
		}
		break;
	}
	return 0;
}

static void etile_set_mac(struct intel_fpga_etile_eth_private *priv, bool enable)
{
	if (enable) {
		/* Enable Rx and Tx datapath */
		tse_clear_bit(priv->mac_dev,
			      eth_tx_mac_csroffs(tx_mac_conf),
			      ETH_TX_MAC_DISABLE_TXVMAC);
		tse_clear_bit(priv->mac_dev, eth_rx_mac_csroffs(rx_mac_frwd_rx_crc),
			      ETH_RX_MAC_CRC_FORWARD);
	} else {
		/* Disable Rx and Tx datapath */
		tse_set_bit(priv->mac_dev,
			    eth_tx_mac_csroffs(tx_mac_conf),
			    ETH_TX_MAC_DISABLE_TXVMAC);
		tse_clear_bit(priv->mac_dev,
			      eth_tx_mac_csroffs(tx_mac_conf),
			      ETH_TX_MAC_DISABLE_S_ADDR_EN);
		netif_warn(priv, drv, priv->dev, "Stop done\n");
	}
}

/* Change the MTU
 */
static int etile_change_mtu(struct net_device *dev, int new_mtu)
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

static void etile_update_mac_addr(struct intel_fpga_etile_eth_private *priv, u8 *addr)
{
	u32 msb;
	u32 lsb;

	lsb = (addr[2] << 24) | (addr[3] << 16) | (addr[4] << 8) | addr[5];
	msb = ((addr[0] << 8) | addr[1]) & 0xffff;
	/* Set primary MAC address */
	csrwr32(lsb, priv->mac_dev, eth_tx_mac_csroffs(tx_mac_source_addr_lower_bytes));
	csrwr32(msb, priv->mac_dev, eth_tx_mac_csroffs(tx_mac_source_addr_higher_bytes));

	tse_set_bit(priv->mac_dev, eth_tx_mac_csroffs(tx_mac_conf), ETH_TX_MAC_DISABLE_S_ADDR_EN);
}

static void etile_set_mac_flow_ctrl(struct intel_fpga_etile_eth_private *priv)
{
	u32 reg;

	if (priv->flow_ctrl & FLOW_RX)
		tse_set_bit(priv->mac_dev,
			    eth_pause_and_priority_csroffs(rx_flow_control_feature_cfg),
			    ETH_RX_EN_STD_FLOW_CTRL);
	else
		tse_clear_bit(priv->mac_dev,
			      eth_pause_and_priority_csroffs(rx_flow_control_feature_cfg),
			      ETH_RX_EN_STD_FLOW_CTRL);

	reg = csrrd32(priv->mac_dev,
		      eth_pause_and_priority_csroffs(rx_flow_control_feature_cfg));
	if (netif_msg_ifup(priv))
		netdev_info(priv->dev, "E-tile rx_flow_ctrl: 0x%08x\n", reg);

	if (priv->flow_ctrl & FLOW_TX) {
		tse_set_bit(priv->mac_dev,
			    eth_pause_and_priority_csroffs(tx_flow_control_feature_cfg),
			    ETH_TX_EN_PRIORITY_FLOW_CTRL);
	} else {
		tse_clear_bit(priv->mac_dev,
			      eth_pause_and_priority_csroffs(tx_flow_control_feature_cfg),
			      ETH_TX_EN_PRIORITY_FLOW_CTRL);
	}

	reg = csrrd32(priv->mac_dev,
		      eth_pause_and_priority_csroffs(tx_flow_control_feature_cfg));
	if (netif_msg_ifup(priv))
		netdev_info(priv->dev, "E-tile tx_flow_ctrl: 0x%08x\n", reg);

	csrwr32(priv->pause, priv->mac_dev,
		eth_pause_and_priority_csroffs(pause_quanta_0));

	reg = csrrd32(priv->mac_dev,
		      eth_pause_and_priority_csroffs(pause_quanta_0));
	if (netif_msg_ifup(priv))
		netdev_info(priv->dev, "E-tile: pause_quanta0: 0x%08x\n", reg);
}

static void etile_clear_mac_statistics(struct intel_fpga_etile_eth_private *priv)
{
	/* Clear all statistics counters for the receive and transmit path */
	tse_set_bit(priv->mac_dev, eth_tx_stats_csroffs(tx_cntr_config),
		    ETH_TX_CNTR_CFG_RST_ALL);
	tse_clear_bit(priv->mac_dev, eth_tx_stats_csroffs(tx_cntr_config),
		      ETH_TX_CNTR_CFG_RST_ALL);
	tse_set_bit(priv->mac_dev, eth_rx_stats_csroffs(rx_cntr_config),
		    ETH_RX_CNTR_CFG_RST_ALL);
	tse_clear_bit(priv->mac_dev, eth_rx_stats_csroffs(rx_cntr_config),
		      ETH_RX_CNTR_CFG_RST_ALL);
}

/* Set or clear the multicast filter for this adaptor
 */
static void etile_set_rx_mode(struct net_device *dev)
{
	/* Not Supported */
}

static int eth_etile_tx_rx_user_flow(struct intel_fpga_etile_eth_private *priv)
{
	u32 tx_pma_delay_ns = 0;
	u32 tx_extra_latency = 0;
	u32 rx_fec_cw_pos = 0;
	u32 rx_spulse_offset = 0;
	u32 rx_pma_delay_ns = 0;
	u32 rx_extra_latency = 0;
	u32 ui_value;
	int ret;

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

	/* TX User Flow */
	/* Step 1 After power up or reset, wait until TX data path is up */
	if (check_counter_complete(priv->mac_dev,
				   eth_phy_csroffs(phy_tx_datapath_ready),
				   ETH_PHY_TX_PCS_READY, true,
				   INTEL_FPGA_WORD_ALIGN)) {
		netdev_err(priv->dev, "MAC Tx datapath not ready\n");
		return -EINVAL;
	}

	/*  Step 2 Calculate TX extra latency */
	/* Convert unit of TX PMA delay from UI to nanoseconds */
	tx_pma_delay_ns = INTEL_FPGA_TX_PMA_DELAY_25G * ui_value;

	/* Get Tx external PHY delay from vendor and add in device tree
	 * and total up all extra latency together
	 */
	tx_extra_latency = (tx_pma_delay_ns + priv->tx_external_phy_delay_ns) >> 8;

	/* Step 3 Write TX extra latency*/
	csrwr32(tx_extra_latency, priv->mac_dev, eth_ptp_csroffs(tx_ptp_extra_latency));
	// wait until TX PTP is ready -> o_sl_tx_ptp_ready = 1'b1
	// TX PTP is up
	// Adjust TX UI

	/* RX User Flow */
	/* Step 1 After power up or reset, wait until RX data path is up */
	if (check_counter_complete(priv->mac_dev,
				   eth_phy_csroffs(phy_pcs_stat_anlt),
				   ETH_PHY_RX_PCS_ALIGNED, true,
				   INTEL_FPGA_WORD_ALIGN)) {
		netdev_err(priv->dev, "MAC Rx datapath not ready\n");
		return -EINVAL;
	}

	/*  Step 2 Read RX FEC codeword position */
	rx_fec_cw_pos = csrrd32(priv->rsfec, eth_rsfec_csroffs(rsfec_cw_pos_rx_0));

	/* Step 3 Determine sync pulse (Alignment Marker) offsets with reference to async pulse */
	rx_spulse_offset = (rx_fec_cw_pos * ui_value);

	/* Step 4 Calculate RX Extra latency and total up extra latency together */
	rx_pma_delay_ns = (INTEL_FPGA_RX_PMA_DELAY_25G * ui_value);
	rx_extra_latency = ((rx_pma_delay_ns + rx_spulse_offset +
			     priv->rx_external_phy_delay_ns) >> 8) | 0x80000000;

	/* Step 5 Write RX extra Latency */
	csrwr32(rx_extra_latency, priv->mac_dev, eth_ptp_csroffs(rx_ptp_extra_latency));

	netdev_info(priv->dev, "tx_extra_latency:0x%x , rx_extra_latency:0x%x\n",
		    tx_extra_latency, rx_extra_latency);

	/* Adjust UI value */
	timer_setup(&priv->fec_timer, ui_adjustments, 0);
	ret = mod_timer(&priv->fec_timer, jiffies + msecs_to_jiffies(1000));
	if (ret)
		netdev_err(priv->dev, "Timer failed to start UI adjustment\n");

	return 0;
}

static int init_rst_mac(struct intel_fpga_etile_eth_private *priv)
{
	int ret;

	/* start the mac */
	etile_set_mac(priv, true);
	etile_update_mac_addr(priv, priv->dev->dev_addr);

	/* Step 1 - Trigger TX and RX digital reset
	 *	1.	EHIP CSR Write, Offset = 0x310, value = 0x6
	 */
	csrwr32(0x6, priv->mac_dev, eth_phy_csroffs(phy_config));

	/* Step 2 - Trigger PMA analog reset
	 *	1.	PMA AVMM Write, Offset = 0x200, value = 0x0
	 *	2.	PMA AVMM Write, Offset = 0x201, value = 0x0
	 *	3.	PMA AVMM Write, Offset = 0x202, value = 0x0
	 *	4.	PMA AVMM Write, Ofset = 0x203, value = 0x81
	 *	5.	PMA AVMM Read, Offset = 0x207, expected value = 0x80
	 *	6.	PMA AVMM Read, Offset = 0x204, expected value = 0x0 (channel #)
	 */
	csrwr8(0x0, priv->xcvr, eth_pma_avmm_csroffs(reg_200));
	csrwr8(0x0, priv->xcvr, eth_pma_avmm_csroffs(reg_201));
	csrwr8(0x0, priv->xcvr, eth_pma_avmm_csroffs(reg_202));
	csrwr8(0x81, priv->xcvr, eth_pma_avmm_csroffs(reg_203));

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_207),
				   XCVR_PMA_AVMM_207_LAST_OP_ON_200_203_SUCCESS,
				   true, INTEL_FPGA_BYTE_ALIGN)) {
		netdev_err(priv->dev, "Analog PMA reset failed, abort\n");
		return -EINVAL;
	}

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_204),
				   XCVR_PMA_AVMM_204_RET_PHYS_CHANNEL_NUMBER,
				   false, INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev, "Cannot read channel number\n");

	/* Step 3 - Reload PMA settings
	 *	1.	PMA AVMM Write, Offset = 0x91[0], value = 0x1
	 *	2.	PMA AVMM Read, Offset = 0x8B, expected values bit [2] and [3] = ‘11’
	 */
	csrwr8(0x1, priv->xcvr, eth_pma_avmm_csroffs(reg_091));
	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_08B),
				   XCVR_PMA_AVMM_08B_PMA_RELOAD_SUCCESS,
				   true, INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev, "Reload PMA settings failed\n");

	/* Step 4 - De-assert TX digital reset
	 *	EHIP CSR Write, Offset = 0x310, value = 0x4
	 */
	csrwr32(0x4, priv->mac_dev, eth_phy_csroffs(phy_config));

	/* Step 5 - Ignore*/

	/* Step 6a - Enable Internal Loopback
	 *	1.	PMA AVMM Write, Offset = 0x84, value = 0x1
	 *	2.	PMA AVMM Write, Offset = 0x85, value = 0x1
	 *	3.	PMA AVMM Write, Offset = 0x86, value = 0x8
	 *	4.	PMA AVMM Write, Offset = 0x87, value = 0x0
	 *	5.	PMA AVMM Write, Offset = 0x90, value = 0x1
	 *	6.	PMA AVMM Read, Offset = 0x8A[7], expected value = 1
	 *	7.	PMA AVMM Read, Offset = 0x8B[0], expected value = 0
	 *	8.	PMA AVMM Read, Offset = 0x88, expected value = 0x8 (same as 0x86 in step #3)
	 *	9.	PMA AVMM Read, Offset = 0x89, expected value = 0x0 (same as 0x87 in step #4)
	 *	10.	PMA AVMM Write, Offset = 0x8A[7], value = 1
	 */
	csrwr8(0x1, priv->xcvr, eth_pma_avmm_csroffs(reg_084));
	csrwr8(0x1, priv->xcvr, eth_pma_avmm_csroffs(reg_085));
	csrwr8(0x8, priv->xcvr, eth_pma_avmm_csroffs(reg_086));
	csrwr8(0x0, priv->xcvr, eth_pma_avmm_csroffs(reg_087));
	csrwr8(0x1, priv->xcvr, eth_pma_avmm_csroffs(reg_090));

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_08A),
				   XCVR_PMA_AVMM_08A_PMA_ATTR_SENT_SUCCESS,
				   true, INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Internal loopback: PMA attribute sent failed\n");

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_08B),
				   XCVR_PMA_AVMM_08B_PMA_FINISH_ATTR,
				   false, INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Internal loopback: PMA attribute not returned\n");

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_088),
				   XCVR_PMA_AVMM_088_PMA_INTERNAL_LOOPBACK,
				   true, INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Internal loopback: PMA low byte failed\n");

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_089),
				   XCVR_PMA_AVMM_089_CORE_PMA_ATTR_CODE_RET_VAL_HI,
				   false, INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Internal loopback: PMA high byte failed\n");

	csrwr8(0x80, priv->xcvr, eth_pma_avmm_csroffs(reg_08A));

	/* Step 6b - Initial Adaptation
	 *	1.	PMA AVMM Write, Offset = 0x84, value = 0x1
	 *	2.	PMA AVMM Write, Offset = 0x85, value = 0x0
	 *	3.	PMA AVMM Write, Offset = 0x86, value = 0xA
	 *	4.	PMA AVMM Write, Offset = 0x87, value = 0x0
	 *	5.	PMA AVMM Write, Offset = 0x90, value = 0x1
	 *	6.	PMA AVMM Read, Offset = 0x8A[7], expected value = 1
	 *	7.	PMA AVMM Read, Offset = 0x8B[0], expected value = 0
	 *	8.	PMA AVMM Read, Offset = 0x88, expected value = 0xA (same as 0x86 in step #3)
	 *	9.	PMA AVMM Read, Offset = 0x89, expected value = 0x0 (same as 0x87 in step #4)
	 *	10.	PMA AVMM Write, Offset = 0x8A[7], value = 1
	 */
	csrwr8(0x1, priv->xcvr, eth_pma_avmm_csroffs(reg_084));
	csrwr8(0x0, priv->xcvr, eth_pma_avmm_csroffs(reg_085));
	csrwr8(0xA, priv->xcvr, eth_pma_avmm_csroffs(reg_086));
	csrwr8(0x0, priv->xcvr, eth_pma_avmm_csroffs(reg_087));
	csrwr8(0x1, priv->xcvr, eth_pma_avmm_csroffs(reg_090));

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_08A),
				   XCVR_PMA_AVMM_08A_PMA_ATTR_SENT_SUCCESS,
				   true, INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Initial Adaptation: PMA attribute sent failed\n");

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_08B),
				   XCVR_PMA_AVMM_08B_PMA_FINISH_ATTR, false,
				   INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Initial Adaptation: PMA attribute not returned\n");

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_088),
				   XCVR_PMA_AVMM_088_PMA_RECEIVER_TUNING_CTRL,
				   true, INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Initial Adaptation: PMA low byte failed\n");

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_089),
				   XCVR_PMA_AVMM_089_CORE_PMA_ATTR_CODE_RET_VAL_HI,
				   false, INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Initial Adaptation: PMA high byte failed\n");

	csrwr8(0x80, priv->xcvr, eth_pma_avmm_csroffs(reg_08A));

	/* Step 6c - Verify Initial Adaptation Status
	 *	1.	PMA AVMM Write, Offset = 0x84, value = 0x0
	 *	2.	PMA AVMM Write, Offset = 0x85, value = 0xB
	 *	3.	PMA AVMM Write, Offset = 0x86, value = 0x26
	 *	4.	PMA AVMM Write, Offset = 0x87, value = 0x1
	 *	5.	PMA AVMM Write, Offset = 0x90, value = 0x1
	 *	6.	PMA AVMM Read, Offset = 0x8A[7], expected value = 1
	 *	7.	PMA AVMM Read, Offset = 0x8B[0], expected value = 0
	 *	8.	PMA AVMM Read, Offset = 0x88[0], expected value = 0
	 *	9.	PMA AVMM Read, Offset = 0x89, expected value = 0x0
	 *	10.	PMA AVMM Write, Offset = 0x8A[7], value = 1
	 */
	csrwr8(0x0, priv->xcvr, eth_pma_avmm_csroffs(reg_084));
	csrwr8(0xB, priv->xcvr, eth_pma_avmm_csroffs(reg_085));
	csrwr8(0x26, priv->xcvr, eth_pma_avmm_csroffs(reg_086));
	csrwr8(0x1, priv->xcvr, eth_pma_avmm_csroffs(reg_087));
	csrwr8(0x1, priv->xcvr, eth_pma_avmm_csroffs(reg_090));

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_08A),
				   XCVR_PMA_AVMM_08A_PMA_ATTR_SENT_SUCCESS,
				   true, INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Initial Adaptation Status: PMA sent failed\n");

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_08B),
				   XCVR_PMA_AVMM_08B_PMA_FINISH_ATTR, false,
				   INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Initial Adaptation Status: PMA not returned\n");

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_088),
				   XCVR_PMA_AVMM_088_PMA_READ_RECEIVER_TUNING,
				   false, INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Initial Adaptation Status: PMA low byte failed");

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_089),
				   XCVR_PMA_AVMM_089_CORE_PMA_ATTR_CODE_RET_VAL_HI,
				   false, INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Initial Adaptation Status: PMA high byte failed\n");

	csrwr8(0x80, priv->xcvr, eth_pma_avmm_csroffs(reg_08A));

	/* Step 7 - Disable internal serial loopback
	 *	1.	PMA AVMM Write, Offset = 0x84, value = 0x0
	 *	2.	PMA AVMM Write, Offset = 0x85, value = 0x1
	 *	3.	PMA AVMM Write, Offset = 0x86, value = 0x8
	 *	4.	PMA AVMM Write, Offset = 0x87, value = 0x0
	 *	5.	PMA AVMM Write, Offset = 0x90, value = 0x1
	 *	6.	PMA AVMM Read, Offset = 0x8A[7], expected value = 1
	 *	7.	PMA AVMM Read, Offset = 0x8B[0], expected value = 0
	 *	8.	PMA AVMM Read, Offset = 0x88, expected value = 0x8 (same as 0x86 in step #3)
	 *	9.	PMA AVMM Read, Offset = 0x89, expected value = 0x0 (same as 0x87 in step #4)
	 *	10.	PMA AVMM Write, Offset = 0x8A[7], value = 1
	 */
	csrwr8(0x0, priv->xcvr, eth_pma_avmm_csroffs(reg_084));
	csrwr8(0x1, priv->xcvr, eth_pma_avmm_csroffs(reg_085));
	csrwr8(0x8, priv->xcvr, eth_pma_avmm_csroffs(reg_086));
	csrwr8(0x0, priv->xcvr, eth_pma_avmm_csroffs(reg_087));
	csrwr8(0x1, priv->xcvr, eth_pma_avmm_csroffs(reg_090));

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_08A),
				   XCVR_PMA_AVMM_08A_PMA_ATTR_SENT_SUCCESS,
				   true, INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Disable loopback: PMA attribute sent failed\n");

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_08B),
				   XCVR_PMA_AVMM_08B_PMA_FINISH_ATTR, false,
				   INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Disable loopback: PMA attribute not returned\n");

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_088),
				   XCVR_PMA_AVMM_088_PMA_INTERNAL_LOOPBACK,
				   true, INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Disable loopback: PMA low byte failed\n");

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_089),
				   XCVR_PMA_AVMM_089_CORE_PMA_ATTR_CODE_RET_VAL_HI,
				   false, INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Disable loopback: PMA high byte failed\n");

	csrwr8(0x80, priv->xcvr, eth_pma_avmm_csroffs(reg_08A));

	/* Step 8 - Wait for valid data traffic on RX and then proceed to the next step.
	 */

	/* Step 9 - Run initial adaptation.
	 *	Verify that the initial adaptation status is complete using interrupt code 0x0126
	 *	and data 0x0B00 (skip this step if using internal serial loopback).
	 *	Same as step 6b and 6c
	 * Step 9.6b - Initial Adaptation
	 *	1.	PMA AVMM Write, Offset = 0x84, value = 0x1
	 *	2.	PMA AVMM Write, Offset = 0x85, value = 0x0
	 *	3.	PMA AVMM Write, Offset = 0x86, value = 0xA
	 *	4.	PMA AVMM Write, Offset = 0x87, value = 0x0
	 *	5.	PMA AVMM Write, Offset = 0x90, value = 0x1
	 *	6.	PMA AVMM Read, Offset = 0x8A[7], expected value = 1
	 *	7.	PMA AVMM Read, Offset = 0x8B[0], expected value = 0
	 *	8.	PMA AVMM Read, Offset = 0x88, expected value = 0xA (same as 0x86 in step #3)
	 *	9.	PMA AVMM Read, Offset = 0x89, expected value = 0x0 (same as 0x87 in step #4)
	 *	10.	PMA AVMM Write, Offset = 0x8A[7], value = 1
	 */
	csrwr8(0x1, priv->xcvr, eth_pma_avmm_csroffs(reg_084));
	csrwr8(0x0, priv->xcvr, eth_pma_avmm_csroffs(reg_085));
	csrwr8(0xA, priv->xcvr, eth_pma_avmm_csroffs(reg_086));
	csrwr8(0x0, priv->xcvr, eth_pma_avmm_csroffs(reg_087));
	csrwr8(0x1, priv->xcvr, eth_pma_avmm_csroffs(reg_090));

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_08A),
				   XCVR_PMA_AVMM_08A_PMA_ATTR_SENT_SUCCESS,
				   true, INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Initial Adaptation repeat: PMA sent failed\n");

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_08B),
				   XCVR_PMA_AVMM_08B_PMA_FINISH_ATTR, false,
				   INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Initial Adaptation repeat: PMA not returned\n");

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_088),
				   XCVR_PMA_AVMM_088_PMA_RECEIVER_TUNING_CTRL,
				   true, INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Initial Adaptation repeat: PMA low byte failed\n");

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_089),
				   XCVR_PMA_AVMM_089_CORE_PMA_ATTR_CODE_RET_VAL_HI,
				   false, INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Initial Adaptation repeat: PMA high byte failed\n");

	csrwr8(0x80, priv->xcvr, eth_pma_avmm_csroffs(reg_08A));

	/* Step 9.6c - Verify Initial Adaptation Status
	 *	1.	PMA AVMM Write, Offset = 0x84, value = 0x0
	 *	2.	PMA AVMM Write, Offset = 0x85, value = 0xB
	 *	3.	PMA AVMM Write, Offset = 0x86, value = 0x26
	 *	4.	PMA AVMM Write, Offset = 0x87, value = 0x1
	 *	5.	PMA AVMM Write, Offset = 0x90, value = 0x1
	 *	6.	PMA AVMM Read, Offset = 0x8A[7], expected value = 1
	 *	7.	PMA AVMM Read, Offset = 0x8B[0], expected value = 0
	 *	8.	PMA AVMM Read, Offset = 0x88[0], expected value = 0
	 *	9.	PMA AVMM Read, Offset = 0x89, expected value = 0x0
	 *	10.	PMA AVMM Write, Offset = 0x8A[7], value = 1
	 */
	csrwr8(0x0, priv->xcvr, eth_pma_avmm_csroffs(reg_084));
	csrwr8(0xB, priv->xcvr, eth_pma_avmm_csroffs(reg_085));
	csrwr8(0x26, priv->xcvr, eth_pma_avmm_csroffs(reg_086));
	csrwr8(0x1, priv->xcvr, eth_pma_avmm_csroffs(reg_087));
	csrwr8(0x1, priv->xcvr, eth_pma_avmm_csroffs(reg_090));

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_08A),
				   XCVR_PMA_AVMM_08A_PMA_ATTR_SENT_SUCCESS,
				   true, INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Initial Adaptation Status repeat: PMA sent failed\n");

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_08B),
				   XCVR_PMA_AVMM_08B_PMA_FINISH_ATTR, false,
				   INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Initial Adaptation Status repeat: PMA not returned\n");

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_088),
				   XCVR_PMA_AVMM_088_PMA_READ_RECEIVER_TUNING,
				   false, INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Initial Adaptation Status repeat: PMA low byte failed");

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_089),
				   XCVR_PMA_AVMM_089_CORE_PMA_ATTR_CODE_RET_VAL_HI,
				   false, INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Initial Adaptation Status repeat: PMA high byte failed\n");

	csrwr8(0x80, priv->xcvr, eth_pma_avmm_csroffs(reg_08A));

	/* Step 10 - Run continuous adaptation
	 *	*During the continuous adaptation, the link partner must keep sending the data.
	 *	If link goes down, the entire sequence must be repeated.
	 *	1.	PMA AVMM Write, Offset = 0x84, value = 0x6
	 *	2.	PMA AVMM Write, Offset = 0x85, value = 0x0
	 *	3.	PMA AVMM Write, Offset = 0x86, value = 0xA
	 *	4.	PMA AVMM Write, Offset = 0x87, value = 0x0
	 *	5.	PMA AVMM Write, Offset = 0x90, value = 0x1
	 *	6.	PMA AVMM Read, Offset = 0x8A[7], expected value = 1
	 *	7.	PMA AVMM Read, Offset = 0x8B[0], expected value = 0
	 *	8.	PMA AVMM Read, Offset = 0x88, expected value = 0xA (same as 0x86 in step #3)
	 *	9.	PMA AVMM Read, Offset = 0x89, expected value = 0x0 (same as 0x87 in step #4)
	 *	10.	PMA AVMM Write, Offset = 0x8A[7], value = 1
	 */
	csrwr8(0x6, priv->xcvr, eth_pma_avmm_csroffs(reg_084));
	csrwr8(0x0, priv->xcvr, eth_pma_avmm_csroffs(reg_085));
	csrwr8(0xA, priv->xcvr, eth_pma_avmm_csroffs(reg_086));
	csrwr8(0x0, priv->xcvr, eth_pma_avmm_csroffs(reg_087));
	csrwr8(0x1, priv->xcvr, eth_pma_avmm_csroffs(reg_090));

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_08A),
				   XCVR_PMA_AVMM_08A_PMA_ATTR_SENT_SUCCESS,
				   true, INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Continuous Adaption: PMA failed to send\n");

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_08B),
				   XCVR_PMA_AVMM_08B_PMA_FINISH_ATTR, false,
				   INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Continuous Adaption: PMA not returned\n");

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_088),
				   XCVR_PMA_AVMM_088_PMA_RECEIVER_TUNING_CTRL,
				   true, INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Continuous Adaption: PMA low byte failed\n");

	if (check_counter_complete(priv->xcvr, eth_pma_avmm_csroffs(reg_089),
				   XCVR_PMA_AVMM_089_CORE_PMA_ATTR_CODE_RET_VAL_HI,
				   false, INTEL_FPGA_BYTE_ALIGN))
		netdev_warn(priv->dev,
			    "Continuous Adaption: PMA high byte failed\n");

	csrwr8(0x80, priv->xcvr, eth_pma_avmm_csroffs(reg_08A));

	/* Step 11 - De-assert RX digital reset
	 *	EHIP CSR Write, Offset = 0x310, value = 0x0
	 */
	csrwr32(0x0, priv->mac_dev, eth_phy_csroffs(phy_config));

	/* Step 12 - Verify RX PCS Status
	 *	EHIP CSR Read, Offset = 0x326, expected value = 0x1
	 */
	if (check_counter_complete(priv->mac_dev,
				   eth_phy_csroffs(phy_pcs_stat_anlt),
				   ETH_PHY_RX_PCS_ALIGNED, true,
				   INTEL_FPGA_WORD_ALIGN)) {
		netdev_err(priv->dev, "RX PCS is not aligned\n");
		return -EINVAL;
	}

	/* Step 13 - IP Ready */
	/* if the link goes down anytime, this whole process above needs to be repeated */
	ret = eth_etile_tx_rx_user_flow(priv);
	if (ret < 0) {
		netdev_err(priv->dev, "Tx & Rx user flow failed\n");
		return ret;
	}

	etile_clear_mac_statistics(priv);
	etile_set_mac_flow_ctrl(priv);

	return 0;
}

/* Control hardware timestamping.
 * This function configures the MAC to enable/disable both outgoing(TX)
 * and incoming(RX) packets time stamping based on user input.
 */
static int etile_set_hwtstamp_config(struct net_device *dev, struct ifreq *ifr)
{
	struct intel_fpga_etile_eth_private *priv = netdev_priv(dev);
	struct hwtstamp_config config;
	int ret = 0;

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

/* Entry point for the ioctl.
 */
static int etile_do_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	int ret = 0;

	if (!netif_running(dev))
		return -EINVAL;

	switch (cmd) {
	case SIOCSHWTSTAMP:
		ret = etile_set_hwtstamp_config(dev, ifr);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return ret;
}

/* Open and initialize the interface
 */
static int etile_open(struct net_device *dev)
{
	struct intel_fpga_etile_eth_private *priv = netdev_priv(dev);
	int ret = 0;
	int i;
	unsigned long flags;

	/* Create and initialize the TX/RX descriptors chains. */
	priv->dma_priv.rx_ring_size = dma_rx_num;
	priv->dma_priv.tx_ring_size = dma_tx_num;
	/* Reset and configure E-tile MAC and probe associated PHY */
	ret = priv->dmaops->init_dma(&priv->dma_priv);
	if (ret) {
		netdev_err(dev, "Cannot initialize DMA\n");
		goto phy_error;
	}

	if (netif_msg_ifup(priv))
		netdev_info(dev, "device MAC address %pM\n",
			    dev->dev_addr);

	spin_lock(&priv->mac_cfg_lock);

	/*  E-tile reset */
	ret = init_rst_mac(priv);
	spin_unlock(&priv->mac_cfg_lock);
	if (ret) {
		netdev_dbg(dev, "Cannot reset MAC core (error: %d)\n", ret);
		goto alloc_skbuf_error;
	}

	priv->dmaops->reset_dma(&priv->dma_priv);

	ret = etile_alloc_init_skbufs(priv);
	if (ret) {
		netdev_err(dev, "DMA descriptors initialization failed\n");
		goto alloc_skbuf_error;
	}

	/* Register RX interrupt */
	ret = devm_request_irq(priv->device, priv->rx_irq, intel_fpga_etile_isr,
			       IRQF_SHARED, dev->name, dev);
	if (ret) {
		netdev_err(dev, "Unable to register RX interrupt %d\n",
			   priv->rx_irq);
		goto init_error;
	}

	/* Register TX interrupt */
	ret = devm_request_irq(priv->device, priv->tx_irq, intel_fpga_etile_isr,
			       IRQF_SHARED, dev->name, dev);
	if (ret) {
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

	netdev_warn(dev, "start rxdma\n");
	priv->dmaops->start_rxdma(&priv->dma_priv);

	if (priv->dmaops->start_txdma)
		priv->dmaops->start_txdma(&priv->dma_priv);

	return 0;

init_error:
	etile_free_skbufs(dev);
alloc_skbuf_error:
phy_error:
	return ret;
}

/* Stop TSE MAC interface and put the device in an inactive state
 */
static int etile_shutdown(struct net_device *dev)
{
	struct intel_fpga_etile_eth_private *priv = netdev_priv(dev);
	unsigned long flags;

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
	spin_lock(&priv->mac_cfg_lock);
	spin_lock(&priv->tx_lock);

	priv->dmaops->reset_dma(&priv->dma_priv);
	etile_free_skbufs(dev);

	spin_unlock(&priv->tx_lock);
	spin_unlock(&priv->mac_cfg_lock);
	priv->dmaops->uninit_dma(&priv->dma_priv);
	del_timer_sync(&priv->fec_timer);

	return 0;
}

static void etile_get_stats64(struct net_device *dev,
			      struct rtnl_link_stats64 *storage)
{
	struct intel_fpga_etile_eth_private *priv = netdev_priv(dev);
	u32 lsb;
	u32 msb;

	/* rx stats */
	lsb = csrrd32(priv->mac_dev,
		      eth_rx_stats_csroffs(rx_frame_octetsok_lsb));
	msb = csrrd32(priv->mac_dev,
		      eth_rx_stats_csroffs(rx_frame_octetsok_msb));
	storage->rx_bytes = ((u64)msb << 32) | lsb;

	lsb = csrrd32(priv->mac_dev,
		      eth_rx_stats_csroffs(rx_mcast_data_ok_lsb));
	msb = csrrd32(priv->mac_dev,
		      eth_rx_stats_csroffs(rx_mcast_data_ok_msb));
	storage->multicast = ((u64)msb << 32) | lsb;

	storage->collisions = 0;

	lsb = csrrd32(priv->mac_dev,
		      eth_rx_stats_csroffs(rx_lenerr_lsb));
	msb = csrrd32(priv->mac_dev,
		      eth_rx_stats_csroffs(rx_lenerr_msb));
	storage->rx_length_errors = ((u64)msb << 32) | lsb;

	storage->rx_over_errors = 0;

	lsb = csrrd32(priv->mac_dev,
		      eth_rx_stats_csroffs(rx_crcerr_okpkt_lsb));
	msb = csrrd32(priv->mac_dev,
		      eth_rx_stats_csroffs(rx_crcerr_okpkt_msb));
	storage->rx_crc_errors = ((u64)msb << 32) | lsb;

	storage->rx_fifo_errors = 0;
	storage->rx_missed_errors = 0;
	//IP UG does not have total RX packets, total RX bad packets, total RX dropped packets
	storage->rx_packets = 0;
	storage->rx_errors = 0;
	storage->rx_dropped = 0;
	/* also count the packets dropped by this network driver */
	storage->rx_dropped += dev->stats.rx_dropped;

	/* tx stats */
	lsb = csrrd32(priv->mac_dev,
		      eth_tx_stats_csroffs(tx_frame_octetsok_lsb));
	msb = csrrd32(priv->mac_dev,
		      eth_tx_stats_csroffs(tx_frame_octetsok_msb));
	storage->tx_bytes = ((u64)msb << 32) | lsb;

	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_malformed_ctrl_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_malformed_ctrl_msb));
	storage->tx_errors = ((u64)msb << 32) | lsb;

	lsb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_dropped_ctrl_lsb));
	msb = csrrd32(priv->mac_dev, eth_tx_stats_csroffs(tx_dropped_ctrl_msb));
	storage->tx_dropped = ((u64)msb << 32) | lsb;

	storage->tx_aborted_errors = 0;
	storage->tx_fifo_errors = 0;
	storage->tx_heartbeat_errors = 0;
	storage->tx_window_errors = 0;
	storage->rx_compressed = 0;
	storage->tx_compressed = 0;
	//IP UG does not have total TX packets
	storage->tx_packets = 0;
}

static const struct net_device_ops intel_fpga_etile_netdev_ops = {
	.ndo_open		= etile_open,
	.ndo_stop		= etile_shutdown,
	.ndo_start_xmit		= etile_start_xmit,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_set_rx_mode	= etile_set_rx_mode,
	.ndo_change_mtu		= etile_change_mtu,
	.ndo_do_ioctl		= etile_do_ioctl,
	.ndo_get_stats64	= etile_get_stats64
};

static void intel_fpga_etile_validate(struct phylink_config *config,
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

static void intel_fpga_etile_mac_pcs_get_state(struct phylink_config *config,
					       struct phylink_link_state *state)
{
	/* fixed speed for now */
	state->speed = SPEED_25000;
	state->duplex = DUPLEX_FULL;
	state->link = 1;
}

static void intel_fpga_etile_mac_an_restart(struct phylink_config *config)
{
	/* Not Supported */
}

static void intel_fpga_etile_mac_config(struct phylink_config *config,
					unsigned int mode,
					const struct phylink_link_state *state)
{
	struct intel_fpga_etile_eth_private *priv =
			netdev_priv(to_net_dev(config->dev));
	int ret;

	/* init mac */
	ret = init_rst_mac(priv);
	if (ret)
		netdev_err(priv->dev, "Cannot reset MAC core (error: %d)\n", ret);
}

static void intel_fpga_etile_mac_link_down(struct phylink_config *config,
					   unsigned int mode,
					   phy_interface_t interface)
{
	struct intel_fpga_etile_eth_private *priv =
			netdev_priv(to_net_dev(config->dev));

	phylink_mac_change(priv->phylink, false);
}

static void intel_fpga_etile_mac_link_up(struct phylink_config *config,
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

static const struct phylink_mac_ops intel_fpga_etile_phylink_ops = {
	.validate = intel_fpga_etile_validate,
	.mac_pcs_get_state = intel_fpga_etile_mac_pcs_get_state,
	.mac_an_restart = intel_fpga_etile_mac_an_restart,
	.mac_config = intel_fpga_etile_mac_config,
	.mac_link_down = intel_fpga_etile_mac_link_down,
	.mac_link_up = intel_fpga_etile_mac_link_up,
};

/* Probe Altera E-tile MAC device
 */
static int intel_fpga_etile_probe(struct platform_device *pdev)
{
	struct net_device *ndev;
	int ret = -ENODEV;
	struct resource *eth_reconfig;
	struct resource *rx_fifo;
	struct resource *xcvr;
	struct resource *rsfec;
	struct intel_fpga_etile_eth_private *priv;
	struct device_node *np = pdev->dev.of_node;
	const unsigned char *macaddr;
	const struct of_device_id *of_id = NULL;

	ndev = alloc_etherdev(sizeof(struct intel_fpga_etile_eth_private));
	if (!ndev) {
		dev_err(&pdev->dev, "Could not allocate network device\n");
		return -ENODEV;
	}

	SET_NETDEV_DEV(ndev, &pdev->dev);

	priv = netdev_priv(ndev);
	priv->device = &pdev->dev;
	priv->dma_priv.device = &pdev->dev;
	priv->dev = ndev;
	priv->dma_priv.dev = ndev;
	priv->ptp_priv.dev = ndev;
	priv->msg_enable = netif_msg_init(debug, default_msg_level);
	priv->dma_priv.msg_enable = netif_msg_init(debug, default_msg_level);
	priv->pause = pause;
	priv->flow_ctrl = flow_ctrl;

	of_id = of_match_device(intel_fpga_etile_ll_ids, &pdev->dev);
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

	/* MAC address space */
	ret = request_and_map(pdev, "eth_reconfig", &eth_reconfig,
			      (void __iomem **)&priv->mac_dev);
	if (ret)
		goto err_free_netdev;

	if (netif_msg_probe(priv))
		dev_info(&pdev->dev, "\tEth Reconfig  at 0x%08lx\n",
			 (unsigned long)eth_reconfig->start);

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

	/* XCVR address space */
	ret = request_and_map(pdev, "xcvr", &xcvr,
			      (void __iomem **)&priv->xcvr);
	if (ret)
		goto err_free_netdev;

	if (netif_msg_probe(priv))
		dev_info(&pdev->dev, "\tXCVR  at 0x%08lx\n",
			 (unsigned long)xcvr->start);

	/* RS-FEC address space */
	ret = request_and_map(pdev, "rsfec", &rsfec,
			      (void __iomem **)&priv->rsfec);
	if (ret)
		goto err_free_netdev;

	if (netif_msg_probe(priv))
		dev_info(&pdev->dev, "\tRS-FEC  at 0x%08lx\n",
			 (unsigned long)rsfec->start);

	/* we only support ptp with the msgdma */
	if (priv->ptp_enable) {
		/* MAP PTP */
		ret = intel_fpga_tod_probe(pdev, &priv->ptp_priv);
		if (ret) {
			dev_err(&pdev->dev, "cannot map PTP\n");
			goto err_free_netdev;
		}
	}

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

	/* Set hash filter to not set for now until the
	 * multicast filter receive issue is debugged
	 */
	priv->hash_filter = 0;

	/* get supplemental address settings for this instance */
	priv->added_unicast =
		of_property_read_bool(pdev->dev.of_node,
				      "altr,has-supplementary-unicast");

	priv->dev->min_mtu = ETH_ZLEN + ETH_FCS_LEN;
	/* Max MTU is 1500, ETH_DATA_LEN */
	priv->dev->max_mtu = ETH_DATA_LEN;

	/* Get the max mtu from the device tree. Note that the
	 * "max-frame-size" parameter is actually max mtu. Definition
	 * in the ePAPR v1.1 spec and usage differ, so go with usage.
	 */
	of_property_read_u32(pdev->dev.of_node, "max-frame-size",
			     &priv->dev->max_mtu);

	/* Make sure DMA buffer size is larger than the max frame size
	 * plus some alignment offset and a VLAN header. If the max frame size is
	 * 1518, a VLAN header would be additional 4 bytes and additional
	 * headroom for alignment is 2 bytes, 2048 is just fine.
	 */
	if (of_property_read_u32(pdev->dev.of_node, "altr,rx-dma-buffer-size",
				 &priv->dma_priv.rx_dma_buf_sz)) {
		dev_warn(&pdev->dev, "cannot obtain Rx dma buffer size\n");
		priv->dma_priv.rx_dma_buf_sz = 2048;
	}

	/* Get MAC PMA digital delays from device tree */
	if (of_property_read_u32(pdev->dev.of_node, "altr,tx-pma-delay-ns",
				 &priv->tx_pma_delay_ns)) {
		dev_warn(&pdev->dev, "cannot obtain Tx PMA delay ns\n");
		priv->tx_pma_delay_ns = 0;
	}

	if (of_property_read_u32(pdev->dev.of_node, "altr,rx-pma-delay-ns",
				 &priv->rx_pma_delay_ns)) {
		dev_warn(&pdev->dev, "cannot obtain Rx PMA delay\n");
		priv->rx_pma_delay_ns = 0;
	}

	if (of_property_read_u32(pdev->dev.of_node, "altr,tx-pma-delay-fns",
				 &priv->tx_pma_delay_fns)) {
		dev_warn(&pdev->dev, "cannot obtain Tx PMA delay fns\n");
		priv->tx_pma_delay_fns = 0;
	}

	if (of_property_read_u32(pdev->dev.of_node, "altr,rx-pma-delay-fns",
				 &priv->rx_pma_delay_fns)) {
		dev_warn(&pdev->dev, "cannot obtain Rx PMA delay\n");
		priv->rx_pma_delay_fns = 0;
	}

	if (of_property_read_u32(pdev->dev.of_node,
				 "altr,tx-external-phy-delay-ns",
				 &priv->tx_external_phy_delay_ns)) {
		dev_warn(&pdev->dev, "cannot obtain Tx phy delay ns\n");
		priv->tx_external_phy_delay_ns = 0;
	}

	if (of_property_read_u32(pdev->dev.of_node,
				 "altr,rx-external-phy-delay-ns",
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
	ndev->mem_start = eth_reconfig->start;
	ndev->mem_end = eth_reconfig->end;
	ndev->netdev_ops = &intel_fpga_etile_netdev_ops;
	intel_fpga_etile_set_ethtool_ops(ndev);

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
	netif_napi_add(ndev, &priv->napi, etile_poll, NAPI_POLL_WEIGHT);

	spin_lock_init(&priv->mac_cfg_lock);
	spin_lock_init(&priv->tx_lock);
	spin_lock_init(&priv->rxdma_irq_lock);
	spin_lock_init(&priv->ptp_priv.tod_lock);

	/* check if phy-mode is present */
	ret = of_get_phy_mode(np, &priv->phy_iface);
	if (ret) {
		dev_err(&pdev->dev, "incorrect phy-mode\n");
		goto err_free_netdev;
	}

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register E-tile ethernet device\n");
		goto err_register_netdev;
	}

	platform_set_drvdata(pdev, ndev);

	if (priv->ptp_enable) {
		ret = intel_fpga_tod_register(&priv->ptp_priv, priv->device);
		if (ret) {
			dev_err(&pdev->dev, "Unable to register PTP clock\n");
			ret = -ENXIO;
			goto err_init_phy;
		}
	}

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

/* Remove Altera E-tile MAC device
 */
static int intel_fpga_etile_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct intel_fpga_etile_eth_private *priv = netdev_priv(ndev);

	if (priv->ptp_enable)
		intel_fpga_tod_unregister(&priv->ptp_priv);

	platform_set_drvdata(pdev, NULL);
	unregister_netdev(ndev);
	free_netdev(ndev);
	return 0;
}

static const struct altera_dmaops altera_dtype_prefetcher = {
	.altera_dtype = ALTERA_DTYPE_MSGDMA_PREF,
	.dmamask = 64,
	.reset_dma = msgdma_pref_reset,
	.enable_txirq = msgdma_pref_enable_txirq,
	.enable_rxirq = msgdma_pref_enable_rxirq,
	.disable_txirq = msgdma_pref_disable_txirq,
	.disable_rxirq = msgdma_pref_disable_rxirq,
	.clear_txirq = msgdma_pref_clear_txirq,
	.clear_rxirq = msgdma_pref_clear_rxirq,
	.tx_buffer = msgdma_pref_tx_buffer,
	.tx_completions = msgdma_pref_tx_completions,
	.add_rx_desc = msgdma_pref_add_rx_desc,
	.get_rx_status = msgdma_pref_rx_status,
	.init_dma = msgdma_pref_initialize,
	.uninit_dma = msgdma_pref_uninitialize,
	.start_rxdma = msgdma_pref_start_rxdma,
	.start_txdma = msgdma_pref_start_txdma,
};

static const struct of_device_id intel_fpga_etile_ll_ids[] = {
	{ .compatible = "altr,etile-msgdma-2.0",
		.data = &altera_dtype_prefetcher, },
	{},
};
MODULE_DEVICE_TABLE(of, intel_fpga_etile_ll_ids);

static struct platform_driver intel_fpga_etile_driver = {
	.probe		= intel_fpga_etile_probe,
	.remove		= intel_fpga_etile_remove,
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name	= INTEL_FPGA_ETILE_ETH_RESOURCE_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = intel_fpga_etile_ll_ids,
	},
};

module_platform_driver(intel_fpga_etile_driver);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("Altera E-tile MAC driver");
MODULE_LICENSE("GPL v2");
