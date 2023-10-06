// SPDX-License-Identifier: GPL
/* Intel FPGA Ethernet MAC driver
 * Copyright (C) 2022,2023 Intel Corporation. All rights reserved
 *
 * Contributors:
 * 	Preetam Narayan
 *
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
#include <linux/skbuff.h>

#include "intel_fpga_eth_main.h"
#include "intel_fpga_eth_hssi_itf.h"
#include "intel_fpga_eth_tile_ops.h"
/* Module parameters */
static int debug = -1;

module_param(debug, int, MOD_PARAM_PERM);
MODULE_PARM_DESC(debug,
		 "Message Level (-1: default, 0: no output, 16: all)");

static const u32 default_msg_level =  NETIF_MSG_DRV    |
				      NETIF_MSG_PROBE  |
				      NETIF_MSG_LINK   |
				      NETIF_MSG_IFUP   |
				      NETIF_MSG_RX_ERR |
				      NETIF_MSG_TX_ERR |
				      NETIF_MSG_IFDOWN;

static int flow_ctrl = FLOW_OFF;

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
#define ETH_TX_THRESH(x)	((x)->dma_priv.tx_ring_size / 4)

#define TXQUEUESTOP_THRESHOLD	2

static const struct of_device_id intel_fpga_xtile_ll_ids[];

static inline void xtile_modify_cpu_txintr_state(intel_fpga_xtile_eth_private *priv, bool enable)
{
	bool lc_txirq = false;
	unsigned long flags;

	spin_lock_irqsave(&priv->rxdma_irq_lock, flags);

	if (priv->tx_irq_enabled != enable) {
		priv->tx_irq_enabled = enable;
		lc_txirq = true;
	}

	spin_unlock_irqrestore(&priv->rxdma_irq_lock, flags);

	if (lc_txirq) {
		if (enable) {
			enable_irq(priv->tx_irq);
			priv->irq_tx_enable_cntr++;
		} else {
			disable_irq(priv->tx_irq);
			priv->irq_tx_disable_cntr++;
		}
	}
}


/* Enable/Disable the Rx interrupt to avoid interrupt disable stacking */
static inline void xtile_modify_cpu_rxintr_state(intel_fpga_xtile_eth_private *priv, bool enable)
{
	bool lc_rxirq = false;
	unsigned long flags;

	spin_lock_irqsave(&priv->rxdma_irq_lock, flags);

	if (priv->rx_irq_enabled != enable) {
		priv->rx_irq_enabled = enable;
		lc_rxirq = true;
	}

	spin_unlock_irqrestore(&priv->rxdma_irq_lock, flags);

	if (lc_rxirq) {
		if (enable) {
			enable_irq(priv->rx_irq);
			priv->irq_rx_enable_cntr++;
		} else {
			disable_irq(priv->rx_irq);
			priv->irq_rx_disable_cntr++;
		}
	}
}

static inline void xtile_enable_cpu_interrupts(intel_fpga_xtile_eth_private *priv)
{
	xtile_modify_cpu_txintr_state(priv, true);
	xtile_modify_cpu_rxintr_state(priv, true);
}

static inline void xtile_disable_cpu_interrupts(intel_fpga_xtile_eth_private *priv)
{
	xtile_modify_cpu_txintr_state(priv, false);
	xtile_modify_cpu_rxintr_state(priv, false);
}

static inline void xtile_enable_disable_dma_interrupts(intel_fpga_xtile_eth_private *priv,
						       bool enable)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->rxdma_irq_lock, flags);

	priv->spec_ops->dma_ops->clear_txirq(&priv->dma_priv);
	priv->spec_ops->dma_ops->clear_rxirq(&priv->dma_priv);

	if (enable) {
		priv->spec_ops->dma_ops->enable_txirq(&priv->dma_priv);
		priv->spec_ops->dma_ops->enable_rxirq(&priv->dma_priv);
	} else {
		priv->spec_ops->dma_ops->disable_txirq(&priv->dma_priv);
		priv->spec_ops->dma_ops->disable_rxirq(&priv->dma_priv);
	}

	spin_unlock_irqrestore(&priv->rxdma_irq_lock, flags);
}

#define PORT_STS_TIMEOUT		20000	/* in us*/
#define PORT_STS_STABILITY_COUNT	10 /* in us*/
static int check_link_stable(intel_fpga_xtile_eth_private *priv)
{
	struct platform_device *pdev = priv->pdev_hssi;
	u32 hssi_port = priv->hssi_port;
	unsigned long timeout, start;
	u32 counter = 0;
	bool eth_portstatus;

	start = jiffies;
	timeout = start + usecs_to_jiffies(PORT_STS_TIMEOUT);
	do
	{
		udelay(1);
		eth_portstatus = hssi_ethport_is_stable(pdev, hssi_port, false);
		if (!eth_portstatus)
			counter = 0;
		else
			counter++;

		if (counter == PORT_STS_STABILITY_COUNT)
			return 0;

	} while(time_before(jiffies, timeout));

	return -ETIME;
}

static int xtile_fec_init(struct platform_device *pdev, intel_fpga_xtile_eth_private *priv)
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

	return 0;
}

static inline u32 xtile_tx_avail(intel_fpga_xtile_eth_private *priv)
{
	return priv->dma_priv.tx_cons + priv->dma_priv.tx_ring_size
		- priv->dma_priv.tx_prod - 1;
}

static int xtile_init_rx_buffer(intel_fpga_xtile_eth_private *priv,
				struct altera_dma_buffer *rxbuffer,
				int len)
{
	rxbuffer->skb = netdev_alloc_skb_ip_align(priv->dev, len);
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

	/* align the address on 4 byte boundary */
	rxbuffer->dma_addr &= (dma_addr_t)~3;
	rxbuffer->len = len;

	return 0;
}

static void xtile_free_rx_buffer(intel_fpga_xtile_eth_private *priv,
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
static void xtile_free_tx_buffer(intel_fpga_xtile_eth_private *priv,
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

static int xtile_alloc_init_skbufs(intel_fpga_xtile_eth_private *priv)
{
	unsigned int rx_descs = priv->dma_priv.rx_ring_size;
	unsigned int tx_descs = priv->dma_priv.tx_ring_size;
	int ret = -ENOMEM;
	int i;

	/* Create Rx ring buffer */
	priv->dma_priv.rx_ring = kcalloc(rx_descs,
					 sizeof(struct altera_dma_buffer),
					 GFP_KERNEL);
	if (!priv->dma_priv.rx_ring)
		goto err_rx_ring;

	/* Create Tx ring buffer */
	priv->dma_priv.tx_ring = kcalloc(tx_descs,
					 sizeof(struct altera_dma_buffer),
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
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	unsigned int rx_descs = priv->dma_priv.rx_ring_size;
	unsigned int tx_descs = priv->dma_priv.tx_ring_size;
	int i;

	/* Release the DMA TX/RX socket buffers */
	for (i = 0; i < rx_descs; i++)
		xtile_free_rx_buffer(priv, &priv->dma_priv.rx_ring[i]);

	for (i = 0; i < tx_descs; i++)
		xtile_free_tx_buffer(priv, &priv->dma_priv.tx_ring[i]);

	kfree(priv->dma_priv.tx_ring);
	kfree(priv->dma_priv.rx_ring);
}

/* Reallocate the skb for the reception process
 */
static inline void xtile_rx_refill(intel_fpga_xtile_eth_private *priv)
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
			priv->spec_ops->dma_ops->add_rx_desc(&priv->dma_priv,
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
static int xtile_rx(intel_fpga_xtile_eth_private *priv, int limit)
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
	        ((rxstatus =
		  priv->spec_ops->dma_ops->get_rx_status(&priv->dma_priv)) != 0) ) {

		pktstatus = rxstatus >> 16;
		pktlength = rxstatus & 0xffff;

		skb = priv->dma_priv.rx_ring[entry].skb;
		if (unlikely(!skb)) {
			netdev_err(priv->dev,
				   "%s: Inconsistent Rx descriptor chain\n",
				   __func__);
			priv->dev->stats.rx_dropped++;
			break;
		}

		count++;
		next_entry = (++priv->dma_priv.rx_cons)
			      % priv->dma_priv.rx_ring_size;

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

		if (unlikely(netif_msg_pktdata(priv))) {
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
static int xtile_tx_complete(intel_fpga_xtile_eth_private *priv)
{
	unsigned int txsize = priv->dma_priv.tx_ring_size;
	u32 ready;
	unsigned int entry;
	struct altera_dma_buffer *tx_buff;
	int txcomplete = 0;

	spin_lock(&priv->tx_lock);
	ready = priv->spec_ops->dma_ops->tx_completions(&priv->dma_priv);

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
		priv->netque_state = true;
		netif_tx_unlock(priv->dev);
	}

	spin_unlock(&priv->tx_lock);

	return txcomplete;
}

/* NAPI polling function
 * Ref: from napi_schedule to poll call = 20us
*/
static int xtile_poll(struct napi_struct *napi, int budget)
{
	intel_fpga_xtile_eth_private *priv =
		container_of(napi, intel_fpga_xtile_eth_private, napi);
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

	priv->spec_ops->dma_ops->clear_txirq(&priv->dma_priv);
	txcomplete = xtile_tx_complete(priv);

	/* Handle case where we are called by netpoll with a budget of 0 */
	if (unlikely(budget <= 0))
		goto repoll;

	priv->spec_ops->dma_ops->clear_rxirq(&priv->dma_priv);
	rxcomplete = xtile_rx(priv, credits);

	min_run = min_t(int, txcomplete + rxcomplete, budget);

	spin_lock_irqsave(&priv->rxdma_irq_lock, flags);

	irq_state = priv->spec_ops->dma_ops->is_txirq_set(&priv->dma_priv) ||
		priv->spec_ops->dma_ops->is_rxirq_set(&priv->dma_priv);

	spin_unlock_irqrestore(&priv->rxdma_irq_lock, flags);

	if (irq_state) {
		priv->spec_ops->dma_ops->clear_txirq(&priv->dma_priv);
		priv->spec_ops->dma_ops->clear_rxirq(&priv->dma_priv);
		/* request to repoll */
		goto repoll;
	}

	/* This case implies that there is possibility of some packets
	 * unprocessed in the Rx DMA buffer
	 */
	if (min_run == credits)
		goto repoll;

	/* enable the interrupt to CPU only if the napi complete done */
	if (napi_complete_done(napi, min_run)) {
		xtile_enable_cpu_interrupts(priv);
	} else {
		netdev_dbg(priv->dev, "napi complete failed");
	}

	/* Amount of Rx work done is returned */
	if (unlikely(netif_msg_intr(priv)))
		netdev_info(priv->dev, "TX/RX complete: %d/%d %d/%d\n",
				txcomplete, rxcomplete, min_run, budget);

	return min_run;

repoll:
	return budget;
}

/* DMA TX & RX FIFO interrupt routing
 * Exec: 6 us
 */
static irqreturn_t intel_fpga_xtile_isr(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	intel_fpga_xtile_eth_private *priv;

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
		netdev_info(dev, "%s interrupt\n",
			(irq == priv->rx_irq) ? "RX" : "TX");

	spin_lock(&priv->rxdma_irq_lock);

	if (likely(napi_schedule_prep(&priv->napi))) {
		if (priv->rx_irq_enabled == true) {
			priv->rx_irq_enabled = false;
			disable_irq_nosync(priv->rx_irq);
			priv->irq_rx_disable_cntr++;
		}
		if (priv->tx_irq_enabled == true) {
			priv->tx_irq_enabled = false;
			disable_irq_nosync(priv->tx_irq);
			priv->irq_tx_disable_cntr++;
		}
		__napi_schedule(&priv->napi);
	}

	priv->spec_ops->dma_ops->clear_rxirq(&priv->dma_priv);
	priv->spec_ops->dma_ops->clear_txirq(&priv->dma_priv);

	spin_unlock(&priv->rxdma_irq_lock);

	return IRQ_HANDLED;
}

int xtile_check_counter_complete(intel_fpga_xtile_eth_private *priv, u32 regbank,
				 size_t offs, u8 bit_mask, bool set_bit,
				 int align)
{
	int counter;
	u32 chan = priv->tile_chan;
	struct platform_device *pdev = priv->pdev_hssi;
	struct hssiss_private *priv_hssi = platform_get_drvdata(pdev);

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

	if(priv_hssi->ver == HSSISS_ETILE)
	{
		while (counter++ < INTEL_FPGA_XTILE_SW_RESET_WATCHDOG_CNTR) {
			if (set_bit) {
				if (hssi_bit_is_set(pdev, regbank, chan,
						   offs, bit_mask,false))
					break;
			} else {
				if (hssi_bit_is_clear(pdev, regbank, chan,
						     offs, bit_mask,false))
					break;
			}
			udelay(1);
		}
		if (counter >= INTEL_FPGA_XTILE_SW_RESET_WATCHDOG_CNTR) {
			if (set_bit) {
				if (hssi_bit_is_clear(pdev, regbank, chan,
						     offs, bit_mask,false))
					return -EINVAL;
			} else {
				if (hssi_bit_is_set(pdev, regbank, chan,
						   offs, bit_mask,false))
					return -EINVAL;
			}
		}
	}
	else
	{
		 while (counter++ < INTEL_FPGA_XTILE_SW_RESET_WATCHDOG_CNTR) {
                        if (set_bit) {
                                if (hssi_bit_is_set(pdev, regbank, chan,
                                                   offs, bit_mask,true))
                                        break;
                        } else {
                                if (hssi_bit_is_clear(pdev, regbank, chan,
                                                     offs, bit_mask,true))
                                        break;
                        }
                        udelay(1);
                }
                if (counter >= INTEL_FPGA_XTILE_SW_RESET_WATCHDOG_CNTR) {
                        if (set_bit) {
                                if (hssi_bit_is_clear(pdev, regbank, chan,
                                                     offs, bit_mask,true))
                                        return -EINVAL;
                        } else {
                                if (hssi_bit_is_set(pdev, regbank, chan,
                                                   offs, bit_mask,true))
                                        return -EINVAL;
                        }
                }

	}
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

static bool xtile_get_link_status(intel_fpga_xtile_eth_private *priv)
{
	if ((!(check_link_stable(priv))) && (!((priv->spec_ops->tile.link_fault_status) &&
					     (priv->spec_ops->tile.link_fault_status(priv)))))
		return true;
	else
		return false;

}

static void eth_link_up(intel_fpga_xtile_eth_private *priv)
{
        /* In case there is issue and packet forwarded to DMA engine
         * but are not being consumed by it then the DMA ring wouldn't
         * be freed resulting in the depeletion of the ring buffers.
         * When the buffer threshold has reached we shouldn't wake the
         * queue back again
         */
        if (netif_queue_stopped(priv->dev)) {
                if (!(xtile_tx_avail(priv) <= TXQUEUESTOP_THRESHOLD)) {
                        netif_wake_queue(priv->dev);
			priv->netque_state = true;
		}
	} else {
		netif_start_queue(priv->dev);
		priv->netque_state = true;
	}

	/* Make sure carrier is on */
	if (!(netif_carrier_ok(priv->dev)))
		netif_carrier_on(priv->dev);

	/* Enable cpu interrupts */
	xtile_enable_cpu_interrupts(priv);

	/* case to handle where the prods have filled up the ring we need to
	 * clear the ring so that the packet processing can happen.
	 * For that we are forcing the napi schedule to take place
	 */
	if (napi_schedule_prep(&priv->napi)) {
		if (netif_msg_hw(priv)) {
			netdev_err(priv->dev,
				   " __napi_schedule invoked in non ISR context\n");

		}
		__napi_schedule(&priv->napi);
	} else {
		if (netif_msg_hw(priv)) {
			netdev_err(priv->dev,
				   "NAPI prep failed\n");
		}
	}

	/* Enable dma interrupts */
	//xtile_enable_disable_dma_interrupts(priv, true);

	netdev_info(priv->dev, "Link is UP\n");
}

static void eth_link_down(intel_fpga_xtile_eth_private *priv)
{
        if (!netif_queue_stopped(priv->dev)) {
                netif_tx_disable(priv->dev);
		priv->netque_state = false;
	}

	/* Disable DMA interrupts */
	//xtile_enable_disable_dma_interrupts(priv, false);

	if (netif_carrier_ok(priv->dev))
		netif_carrier_off(priv->dev);

	netdev_info(priv->dev, "Link is Down\n");
}

/* make sure this function only called from xtile open */
static void xtile_start_phy(intel_fpga_xtile_eth_private *priv)
{
	rtnl_lock();
	if (priv->phylink)
		phylink_start(priv->phylink);
	rtnl_unlock();
}

/* make sure this function only called from xtile shutdown */
static void xtile_stop_phy(intel_fpga_xtile_eth_private *priv, bool rtnl_lock_available)
{
	if (priv->phylink) {
		if (rtnl_lock_available) {
			phylink_stop(priv->phylink);
		} else {
			if (rtnl_trylock()) {
				phylink_stop(priv->phylink);
				rtnl_unlock();
			} else {
				if (netif_msg_ifdown(priv))
					netdev_info(priv->dev, "Link is Down\n");
			}
		}
	}
}

/* write protected read for the monitor link status */
static inline bool wpr_get_monitor_link_status(intel_fpga_xtile_eth_private *priv) {
	bool value = false;

	read_lock(&priv->wr_lock);
	value = priv->monitor_thread_enable;
	read_unlock(&priv->wr_lock);

	return value;
}

/* read protected write for the monitor link status */
static inline void rpw_set_monitor_link_status(bool value,
					       intel_fpga_xtile_eth_private *priv) {
	write_lock(&priv->wr_lock);
	priv->monitor_thread_enable = value;
	write_unlock(&priv->wr_lock);
}

#define LINK_STS_POLL_TIMEOUT		10	/* 10 ms*/
static void eth_monitor_link_status(struct work_struct *work)
{
	bool link = false;
	int tile_error = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	intel_fpga_xtile_eth_private *priv = container_of(dwork,
							  intel_fpga_xtile_eth_private,
							  dwork);

	/* NOTE: Monitoring thread can be cancelled asynchronously
	 * if the monitor thread is disabled then we have no reason to continue
	 */
	if (!wpr_get_monitor_link_status(priv))
		return;

	/* Get link status */
	if (priv->spec_ops->link_check)
		if (priv->spec_ops->link_check(priv))
			link = true;

	/* Forcly change state to STOP after RUN,
	 * once link is detected as low
	 */
	if ((ETH_LINK_STATE_RUN == priv->link_state) && !link)
		priv->link_state = ETH_LINK_STATE_STOP;

	switch(priv->link_state) {
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
				//xtile_start_phy(priv);

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
			//xtile_stop_phy(priv, false);
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

	/* Error handling */
	if (!tile_error) {
		// change the poll timeout if required
	}

	if (wpr_get_monitor_link_status(priv))
		schedule_delayed_work(&priv->dwork, msecs_to_jiffies(LINK_STS_POLL_TIMEOUT));
}

static void start_link_monitoring_thread(intel_fpga_xtile_eth_private *priv)
{
	rpw_set_monitor_link_status(true, priv);

	priv->link_state = ETH_LINK_STATE_START;
	INIT_DELAYED_WORK(&priv->dwork, eth_monitor_link_status);
	eth_monitor_link_status(&priv->dwork.work);
}

static void stop_link_monitoring_thread(intel_fpga_xtile_eth_private *priv)
{
	rpw_set_monitor_link_status(false, priv);
	priv->link_state = ETH_LINK_STATE_STOP;

	cancel_delayed_work_sync(&priv->dwork);
}

/* Open and initialize the interface */
static int xtile_open(struct net_device *dev)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	struct platform_device *pdev = priv->pdev_hssi;
	u32 hssi_port = priv->hssi_port;
	unsigned long flags;
	int ret = 0;
	int i;

	/*
	 * deassert reset:
	 * emib interface, mac, pcs, fec, pma, stat for both tx and rx
	 * different for etile aand  ftile
	 */
	if (priv->spec_ops->tile.deassert_reset)
		priv->spec_ops->tile.deassert_reset(priv);

	priv->spec_ops->dma_ops->quiese_pref(&priv->dma_priv);

        /* Reset the mSGDMA engine and configure the mSGDMA register to           */
        /* provide information of the Tx and Rx DMA ring start address in memory  */
        priv->spec_ops->dma_ops->reset_dma(&priv->dma_priv);

	/* Create and initialize the TX/RX descriptors chains. */
	priv->dma_priv.rx_ring_size = dma_rx_num;
	priv->dma_priv.tx_ring_size = dma_tx_num;

	/* Allocate Tx and Rx descriptor ring and initialize respectively */
	ret = priv->spec_ops->dma_ops->init_dma(&priv->dma_priv);
	if (ret) {
		netdev_err(dev, "Cannot initialize DMA\n");
		goto dma_error;
	}

	/* Allocate SKBuffers */
	ret = xtile_alloc_init_skbufs(priv);
	if (ret) {
		netdev_err(dev, "DMA descriptors initialization failed\n");
		goto dma_error;
	}

	/* Safe start, enable interrupts */
	xtile_enable_disable_dma_interrupts(priv, true);

	/* Acquire spin lock */
	spin_lock_irqsave(&priv->rxdma_irq_lock, flags);

	/* Setup RX descriptor chain */
	for (i = 0; i < priv->dma_priv.rx_ring_size; i++)
		priv->spec_ops->dma_ops->add_rx_desc(&priv->dma_priv,
					  &priv->dma_priv.rx_ring[i]);

	/* Release spin lock */
	spin_unlock_irqrestore(&priv->rxdma_irq_lock, flags);

	/* Start/Prepare DMA */
	if (priv->spec_ops->dma_ops->start_txdma)
		priv->spec_ops->dma_ops->start_txdma(&priv->dma_priv);

	if (priv->spec_ops->dma_ops->start_rxdma)
		priv->spec_ops->dma_ops->start_rxdma(&priv->dma_priv);

	/* Pre link, tile initialization */
	if (priv->spec_ops->tile.init)
		if ((ret = (priv->spec_ops->tile.init(priv))))
			goto tile_init_error;

	/* clear the MAC layer statistics to start afresh */
        xtile_clear_mac_statistics(pdev, hssi_port);

	/* Enable CPU interrupts and reset counters */
	xtile_enable_cpu_interrupts(priv);
	priv->irq_rx_enable_cntr = 0;
	priv->irq_tx_enable_cntr = 0;
	priv->irq_rx_disable_cntr = 0;
	priv->irq_tx_disable_cntr = 0;

	/* Tx queue might be enabled by default we need to wait for the link to be up
	 * for the tx transmission to start
	 */
	netif_tx_disable(dev);
	priv->netque_state = false;

	/* Enable NAPI so that driver is ready to poll when there is napi_schedule call */
	if (!priv->napi_state) {
		napi_enable(&priv->napi);
		priv->napi_state = true;
	}

	if (!wpr_get_monitor_link_status(priv)) {
		start_link_monitoring_thread(priv);
		netdev_info(dev, "Ethernet link monitoring thread started");
	} else {
		netdev_err(dev, "Ethernet link monitoring thread already running");
	}

	return 0;

tile_init_error:
	xtile_enable_disable_dma_interrupts(priv, false);
	/* Deallocate SKBuffer */
	xtile_free_skbufs(dev);
dma_error:
	return ret;
}

/* Stop MAC interface and put the device in an inactive state
 */
static int xtile_shutdown(struct net_device *dev)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	stop_link_monitoring_thread(priv);

	if (priv->spec_ops->tile.uninit)
		priv->spec_ops->tile.uninit(priv);

	/* Stop the phylink */
	//xtile_stop_phy(priv, true);
	eth_link_down(priv);

	/* Disable DMA interrupts */
	xtile_enable_disable_dma_interrupts(priv, false);

	/* Disable CPU interrupts */
	xtile_disable_cpu_interrupts(priv);

	/* stop napi */
	if (priv->napi_state) {
		napi_synchronize(&priv->napi);
		napi_disable(&priv->napi);
		priv->napi_state = false;
	}

        /* we are ensuring we get the lock so that any pending activity */
        /* if any is done */
        spin_lock(&priv->tx_lock);
        spin_unlock(&priv->tx_lock);

        spin_lock(&priv->mac_cfg_lock);
        spin_unlock(&priv->mac_cfg_lock);

	priv->spec_ops->dma_ops->quiese_pref(&priv->dma_priv);

        /* Reset the mSGDMA engine */
	priv->spec_ops->dma_ops->reset_dma(&priv->dma_priv);

	/* Clear the allocate skbuffers */
	xtile_free_skbufs(dev);

	/* Uninitialize mSGDMA */
        priv->spec_ops->dma_ops->uninit_dma(&priv->dma_priv);

	/* reset: emib interface, mac, pcs, fec, pma, stat for both tx and rx */
	if (priv->spec_ops->tile.reset)
		priv->spec_ops->tile.reset(priv, false, false, true);

	priv->dma_priv.tx_cons = 0;
	priv->dma_priv.rx_cons = 0;
	priv->dma_priv.tx_prod = 0;
	priv->dma_priv.rx_prod = 0;
	return 0;
}

static int xtile_change_mac(struct net_device *dev, void *inet_ds) {

	struct sockaddr *addr = inet_ds;
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

	if (!is_valid_ether_addr(addr->sa_data))
                return -EADDRNOTAVAIL;

	memcpy(dev->dev_addr, addr->sa_data, ETH_ALEN);

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
	dma_addr_t dma_addr;
	struct altera_dma_buffer *buffer = NULL;
	int nfrags = skb_shinfo(skb)->nr_frags;
	unsigned int nopaged_len = skb_headlen(skb);
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	unsigned int txsize = priv->dma_priv.tx_ring_size;

	if (netif_queue_stopped(dev) || (priv->netque_state == false))
		return NETDEV_TX_BUSY;

	// pad with dummy bytes, DMA irq will stop otherwise
	if (nopaged_len < 60)
		nopaged_len = 60;

	spin_lock_bh(&priv->tx_lock);

	if (unlikely(xtile_tx_avail(priv) < nfrags + 1)) {
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

	/* Map the first skb fragment */
	entry = priv->dma_priv.tx_prod % txsize;
	buffer = &priv->dma_priv.tx_ring[entry];

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
		     priv->dma_priv.hwts_tx_en))
		/* declare that device is doing timestamping */
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;

	/* Provide a software time stamp if requested and hardware timestamping
	 * is not possible (SKBTX_IN_PROGRESS not set).
	 */
	if (!priv->dma_priv.hwts_tx_en)
		skb_tx_timestamp(skb);

	if (unlikely( NETDEV_TX_BUSY ==
		      priv->spec_ops->dma_ops->tx_buffer(&priv->dma_priv, buffer)) ) {

		/* In order to avoid skb to be freed
		 * Ref: https://www.kernel.org/doc/html/latest/networking/driver.html
		 */
		buffer->skb = NULL;
		xtile_free_tx_buffer(priv, buffer);
		goto err;
	}
	else {
		priv->dma_priv.tx_prod++;
		dev->stats.tx_bytes += nopaged_len;
	}

	if (unlikely(xtile_tx_avail(priv) <= TXQUEUESTOP_THRESHOLD)) {
		if (netif_msg_hw(priv))
			netdev_info(priv->dev, " stopped transmitting packets\n");
		netif_stop_queue(dev);
		priv->netque_state = false;
	}

allgood:
	spin_unlock_bh(&priv->tx_lock);
	return NETDEV_TX_OK;

err:
	spin_unlock_bh(&priv->tx_lock);
	if (!netif_queue_stopped(dev)) {
		netif_tx_disable(dev);
		priv->netque_state = false;
	}

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
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
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
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
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
	storage->tx_errors 	         = 0;
	storage->tx_dropped          = 0;
	storage->rx_compressed 	     = 0;
	storage->tx_compressed 	     = 0;
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
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);

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
	.ndo_get_stats64	= xtile_get_stats64
};

static void intel_fpga_xtile_validate(struct phylink_config *config,
				      unsigned long *supported,
				      struct phylink_link_state *state)
{
        intel_fpga_xtile_eth_private *priv =
                netdev_priv(to_net_dev(config->dev));


	__ETHTOOL_DECLARE_LINK_MODE_MASK(mac_supported) = { 0, };
	__ETHTOOL_DECLARE_LINK_MODE_MASK(mask) = { 0, };

	if (!priv)
                return;

	if (state->interface != PHY_INTERFACE_MODE_NA &&
	    state->interface != PHY_INTERFACE_MODE_10GKR &&
	    state->interface != PHY_INTERFACE_MODE_10GBASER &&
	    state->interface != PHY_INTERFACE_MODE_25GKR) {
		bitmap_zero(supported, __ETHTOOL_LINK_MODE_MASK_NBITS);
		return;
	}

        if (priv->autoneg == true)
        {
                phylink_set(mask, Autoneg);
                phylink_set(mac_supported, Autoneg);
        }
        else
        {
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
}

static void intel_fpga_xtile_mac_pcs_get_state(struct phylink_config *config,
					       struct phylink_link_state *state)
{
	/* fixed speed for now */
        intel_fpga_xtile_eth_private *priv =
                netdev_priv(to_net_dev(config->dev));

        if (!priv)
                return;

        state->speed = priv->link_speed;
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
	intel_fpga_xtile_eth_private *priv =
		netdev_priv(to_net_dev(config->dev));

	if (!priv)
		return;

	state->speed = priv->link_speed;
	state->duplex = DUPLEX_FULL;
	if (priv->autoneg == false)
		state->an_enabled = AUTONEG_DISABLE;
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
	intel_fpga_xtile_eth_private *priv =
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
	intel_fpga_xtile_eth_private *priv =
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
	struct resource *tx_fifo;
	struct device_node *dev_hssi;
	u8 macaddr[ETH_ALEN];
	struct fwnode_handle *fixed_node;
	struct platform_device *pdev_hssi;
	const struct xtile_spec_ops *op_ptr;
	intel_fpga_xtile_eth_private *priv;
	struct device_node *dev_tod;
	struct platform_device *pdev_tod;

	np = pdev->dev.of_node;

	ndev = alloc_etherdev(sizeof(intel_fpga_xtile_eth_private));
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
	priv->dma_priv.dev    = ndev;
	priv->dma_priv.device = &pdev->dev;
	priv->msg_enable      = netif_msg_init(debug, default_msg_level);
	priv->dma_priv.msg_enable = netif_msg_init(debug, default_msg_level);

	priv->phylink_config.dev = &priv->dev->dev;
	priv->phylink_config.type = PHYLINK_NETDEV;
	priv->phylink_config.get_fixed_state = intel_fpga_xtile_get_pcs_fixed_state;

	op_ptr = of_device_get_match_data(&pdev->dev);

	if (op_ptr == NULL) {
		dev_err(&pdev->dev, "No matching data field found\n");
		ret = -ENODEV;
		goto err_free_netdev;
	}

	/* Get the HSSI node device from the device tree node */
	dev_hssi = of_parse_phandle(pdev->dev.of_node, "hssiss", 0);
	if (!dev_hssi) {
		return -ENOENT;
	}

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

	if (of_property_read_u16(np, "pma_type",
                                 &priv->pma_type)) {

		dev_warn(&pdev->dev, "cannot obtain pma type defaulting to be FGT \n");
		priv->pma_type = 0;
	}
	priv->spec_ops = (struct xtile_spec_ops *) op_ptr;

	/* PTP is only supported with a modified MSGDMA */
	priv->ptp_enable = of_property_read_bool(pdev->dev.of_node,
						 "altr,has-ptp");
	if (priv->ptp_enable &&
	    priv->spec_ops->dma_ops->altera_dtype != ALTERA_DTYPE_MSGDMA_PREF) {
		dev_err(&pdev->dev, "PTP requires modified dma\n");
		ret = -ENODEV;
		goto err_free_netdev;
	}


	if (priv->ptp_enable)
	{
		/* PTP Timestamp Accuracy mode */
		ret  = of_property_read_string(pdev->dev.of_node, "ptp_accu_mode",
				       &priv->ptp_accu_mode);
		if (ret < 0) {
			priv->ptp_accu_mode = "Basic";
		}

		if (strcasecmp(priv->ptp_accu_mode, "Advanced") == 0)
		{
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
	priv->ptp_clockcleaner_enable = of_property_read_bool(pdev->dev.of_node,
							      "altr,has-ptp-clockcleaner");
	if (!priv->ptp_enable && !priv->ptp_clockcleaner_enable ) {
		dev_err(&pdev->dev, "Hardware Clock Frequency adjustment requires PTP\n");
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

	/* Register TX interrupt */
        ret = devm_request_irq(priv->device, priv->tx_irq, intel_fpga_xtile_isr,
                               IRQF_SHARED, ndev->name, ndev);
        if (ret)
        {
                dev_err(&pdev->dev, "Unable to register TX interrupt %d\n",
                           priv->tx_irq);
                goto err_free_netdev;
        }
	disable_irq(priv->tx_irq);
	priv->tx_irq_enabled = false;

	/* mSGDMA Rx IRQ */
	priv->rx_irq = platform_get_irq_byname(pdev, "rx_irq");
	if (priv->rx_irq == -ENXIO) {
		dev_err(&pdev->dev, "cannot obtain Rx IRQ\n");
		ret = -ENXIO;
		goto err_free_netdev;
	}

	/* Register RX interrupt */
	ret = devm_request_irq(priv->device, priv->rx_irq, intel_fpga_xtile_isr,
			       IRQF_SHARED, ndev->name, ndev);
	if (ret) {
		dev_err(&pdev->dev, "Unable to register RX interrupt %d\n",
			   priv->rx_irq);
		goto err_free_netdev;
	}
	disable_irq(priv->rx_irq);
	priv->rx_irq_enabled = false;

	/* Map DMA */
	ret = altera_eth_dma_probe(pdev, &priv->dma_priv,
				   priv->spec_ops->dma_ops->altera_dtype);
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

        /* Tx Fifo */
        ret = request_and_map(pdev, "tx_fifo", &tx_fifo,
                              (void __iomem **)&priv->tx_fifo);
        if (ret)
                goto err_free_netdev;

        if (netif_msg_probe(priv))
                dev_info(&pdev->dev, "\tTX FIFO  at 0x%08lx\n",
	 			(unsigned long)tx_fifo->start);

	if (dma_set_mask_and_coherent(priv->device,
				      DMA_BIT_MASK(priv->spec_ops->dma_ops->dmamask))) {
		if (dma_set_mask_and_coherent(priv->device,
					      DMA_BIT_MASK(32))) {
			goto err_free_netdev;
		}
	}

	if (of_property_read_u32(pdev->dev.of_node,
				"rx-fifo-almost-full",
				 &priv->rx_fifo_almost_full)) {
		dev_err(&pdev->dev, "cannot obtain rx-fifo-almost-full\n");
		priv->rx_fifo_almost_full = 0x4000;
	}

	if (of_property_read_u32(pdev->dev.of_node,
				"rx-fifo-almost-empty",
				 &priv->rx_fifo_almost_empty)) {
		dev_err(&pdev->dev, "cannot obtain rx-fifo-almost-empty\n");
		priv->rx_fifo_almost_empty = 0x3000;
	}

	priv->dev->min_mtu = ETH_ZLEN + ETH_FCS_LEN;

	/* Max MTU is 1500, ETH_DATA_LEN */
	priv->dev->max_mtu = VLAN_ETH_FRAME_LEN + ETH_FCS_LEN;

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

	if(of_get_mac_address(pdev->dev.of_node,macaddr)){
		dev_info(&pdev->dev, "cannot obtain MAC address using random HW addresss\n");
		eth_hw_addr_random(ndev);

	}
	else{
		ether_addr_copy(ndev->dev_addr, macaddr);
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

	/* setup NAPI interface */
	netif_napi_add(ndev, &priv->napi, xtile_poll, NAPI_POLL_WEIGHT);

	/* tracks the current napi state whether enabled or disabled */
	priv->napi_state = false;
	priv->netque_state = false;

	spin_lock_init(&priv->tx_lock);
	spin_lock_init(&priv->rxdma_irq_lock);
	spin_lock_init(&priv->mac_cfg_lock);

	rwlock_init(&priv->wr_lock);

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

	priv->autoneg = true;

	/* read the fixed link properties*/
	fixed_node = fwnode_get_named_child_node(pdev->dev.fwnode, "fixed-link");
	if (fixed_node) {
		fwnode_property_read_u32(fixed_node, "speed", &priv->link_speed);
		priv->duplex = DUPLEX_FULL;
		priv->autoneg = false;

		dev_info(&pdev->dev, "\tfixed link speed:%d full duplex:%d\n",
			 priv->link_speed, priv->duplex);
	} else {
		dev_err(&pdev->dev, "fixed link property undefined\n");
		goto err_free_netdev;
	}
	if (priv->ptp_enable) {
		dev_tod  = of_parse_phandle(pdev->dev.of_node, "tod", 0);
		pdev_tod = of_find_device_by_node(dev_tod);
		if(pdev_tod)
			priv->ptp_priv = dev_get_drvdata(&pdev_tod->dev);
		if (!pdev_tod || !priv->ptp_priv) {
			dev_err(&pdev->dev, "PTP clock not available\n");
			ret = -EPROBE_DEFER;
			goto err_free_netdev;
		}
		dev_info(&pdev->dev, "\tPTP Clock: %s\n", priv->ptp_priv->ptp_clock_ops.name);
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
	netif_napi_del(&priv->napi);
err_free_netdev:
	free_netdev(ndev);
	return ret;
}

/* Remove MAC device */
static int intel_fpga_xtile_remove(struct platform_device *pdev)
{
	intel_fpga_xtile_eth_private *priv;
	struct net_device *ndev;

	ndev = platform_get_drvdata(pdev);
	priv = netdev_priv(ndev);

	/* perform the proper cleaning up */
	xtile_shutdown(ndev);

	/* Unregister RX interrupt */
	devm_free_irq(priv->device, priv->rx_irq, ndev);

	/* Unregister TX interrupt */
	devm_free_irq(priv->device, priv->tx_irq, ndev);

	platform_set_drvdata(pdev, NULL);
	unregister_netdev(ndev);
	free_netdev(ndev);

	return 0;
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
	},
	.link_check       = xtile_get_link_status,
};

static const struct of_device_id intel_fpga_xtile_ll_ids[] = {
	{.compatible = "altr,hssi-etile-1.0",
	 .data = &etile_data,
	},
	{.compatible = "altr,hssi-ftile-1.0",
         .data = &ftile_data,
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

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("Altera HSSI based MAC driver");
MODULE_LICENSE("GPL v2");
