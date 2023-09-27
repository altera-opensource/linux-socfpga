// SPDX-License-Identifier: GPL-2.0
/* MSGDMA Prefetcher driver for Altera ethernet devices
 *
 * Copyright (C) 2020 Intel Corporation. All rights reserved.
 * Author(s):
 *   Dalon Westergreen <dalon.westergreen@intel.com>
 */

#include <linux/list.h>
#include <linux/netdevice.h>
#include <linux/net_tstamp.h>
#include "altera_eth_dma.h"
#include "altera_msgdma.h"
#include "altera_msgdmahw.h"
#include "altera_msgdma_prefetcher.h"
#include "altera_msgdmahw_prefetcher.h"
#include "altera_utils.h"

int msgdma_pref_initialize(struct altera_dma_private *priv)
{
	int i;
	struct msgdma_pref_extended_desc *rx_descs;
	struct msgdma_pref_extended_desc *tx_descs;
	dma_addr_t rx_descsphys;
	dma_addr_t tx_descsphys;

	priv->pref_rxdescphys = (dma_addr_t)0;
	priv->pref_txdescphys = (dma_addr_t)0;

	/* we need to allocate more pref descriptors than ringsize to
	 * prevent all of the descriptors being owned by hw.  To do this
	 * we just allocate twice ring_size descriptors.
	 * rx_ring_size = priv->rx_ring_size * 2
	 * tx_ring_size = priv->tx_ring_size * 2
	 */

	/* The prefetcher requires the descriptors to be aligned to the
	 * descriptor read/write master's data width which worst case is
	 * 512 bits.  Currently we DO NOT CHECK THIS and only support 32-bit
	 * prefetcher masters.
	 */

	/* allocate memory for rx descriptors */
	priv->pref_rxdesc =
		dma_alloc_coherent(priv->device,
				   sizeof(struct msgdma_pref_extended_desc)
				   * priv->rx_ring_size * 2,
				   &priv->pref_rxdescphys, GFP_KERNEL);

	if (!priv->pref_rxdesc)
		goto err_rx;

	/* allocate memory for tx descriptors */
	priv->pref_txdesc =
		dma_alloc_coherent(priv->device,
				   sizeof(struct msgdma_pref_extended_desc)
				   * priv->tx_ring_size * 2,
				   &priv->pref_txdescphys, GFP_KERNEL);

	if (!priv->pref_txdesc)
		goto err_tx;

	/* setup base descriptor ring for tx & rx */
	rx_descs = (struct msgdma_pref_extended_desc *)priv->pref_rxdesc;
	tx_descs = (struct msgdma_pref_extended_desc *)priv->pref_txdesc;
	tx_descsphys = priv->pref_txdescphys;
	rx_descsphys = priv->pref_rxdescphys;

	/* setup RX descriptors */
	priv->pref_rx_prod = 0;
	for (i = 0; i < priv->rx_ring_size * 2; i++) {
		rx_descsphys = priv->pref_rxdescphys +
			(((i + 1) % (priv->rx_ring_size * 2)) *
			sizeof(struct msgdma_pref_extended_desc));
		rx_descs[i].next_desc_lo = lower_32_bits(rx_descsphys);
		rx_descs[i].next_desc_hi = upper_32_bits(rx_descsphys);
		rx_descs[i].stride = MSGDMA_DESC_RX_STRIDE;
		/* burst set to 0 so it defaults to max configured */
		/* set seq number to desc number */
		rx_descs[i].burst_seq_num = i;
	}

	/* setup TX descriptors */
	for (i = 0; i < priv->tx_ring_size * 2; i++) {
		tx_descsphys = priv->pref_txdescphys +
			(((i + 1) % (priv->tx_ring_size * 2)) *
			sizeof(struct msgdma_pref_extended_desc));
		tx_descs[i].next_desc_lo = lower_32_bits(tx_descsphys);
		tx_descs[i].next_desc_hi = upper_32_bits(tx_descsphys);
		tx_descs[i].stride = MSGDMA_DESC_TX_STRIDE;
		/* burst set to 0 so it defaults to max configured */
		/* set seq number to desc number */
		tx_descs[i].burst_seq_num = i;
	}

	if (netif_msg_ifup(priv))
		netdev_info(priv->dev, "%s: RX Desc mem at 0x%llx\n", __func__,
			    priv->pref_rxdescphys);

	if (netif_msg_ifup(priv))
		netdev_info(priv->dev, "%s: TX Desc mem at 0x%llx\n", __func__,
			    priv->pref_txdescphys);

	return 0;

err_tx:
	dma_free_coherent(priv->device,
			  sizeof(struct msgdma_pref_extended_desc)
			  * priv->rx_ring_size * 2,
			  priv->pref_rxdesc, priv->pref_rxdescphys);
err_rx:
	return -ENOMEM;
}

void msgdma_pref_uninitialize(struct altera_dma_private *priv)
{
	if (priv->pref_rxdesc)
		dma_free_coherent(priv->device,
				  sizeof(struct msgdma_pref_extended_desc)
				  * priv->rx_ring_size * 2,
				  priv->pref_rxdesc, priv->pref_rxdescphys);

	if (priv->pref_txdesc)
		dma_free_coherent(priv->device,
				  sizeof(struct msgdma_pref_extended_desc)
				  * priv->tx_ring_size * 2,
				  priv->pref_txdesc, priv->pref_txdescphys);
}

bool msgdma_pref_is_txirq(struct altera_dma_private *priv)
{
       return tse_bit_is_set(priv->tx_pref_csr,
		       msgdma_pref_csroffs(status), MSGDMA_PREF_STAT_IRQ);
}

bool msgdma_pref_is_rxirq(struct altera_dma_private *priv)
{
       return tse_bit_is_set(priv->rx_pref_csr,
		       msgdma_pref_csroffs(status), MSGDMA_PREF_STAT_IRQ);
}

void msgdma_pref_enable_txirq(struct altera_dma_private *priv)
{
	tse_set_bit(priv->tx_pref_csr, msgdma_pref_csroffs(control),
		    MSGDMA_PREF_CTL_GLOBAL_INTR);
}

void msgdma_pref_disable_txirq(struct altera_dma_private *priv)
{
	tse_clear_bit(priv->tx_pref_csr, msgdma_pref_csroffs(control),
		      MSGDMA_PREF_CTL_GLOBAL_INTR);
}

void msgdma_pref_clear_txirq(struct altera_dma_private *priv)
{
	csrwr32(MSGDMA_PREF_STAT_IRQ, priv->tx_pref_csr,
		msgdma_pref_csroffs(status));
}

void msgdma_pref_enable_rxirq(struct altera_dma_private *priv)
{
	tse_set_bit(priv->rx_pref_csr, msgdma_pref_csroffs(control),
		    MSGDMA_PREF_CTL_GLOBAL_INTR);
}

void msgdma_pref_disable_rxirq(struct altera_dma_private *priv)
{
	tse_clear_bit(priv->rx_pref_csr, msgdma_pref_csroffs(control),
		      MSGDMA_PREF_CTL_GLOBAL_INTR);
}

void msgdma_pref_clear_rxirq(struct altera_dma_private *priv)
{
	csrwr32(MSGDMA_PREF_STAT_IRQ, priv->rx_pref_csr,
		msgdma_pref_csroffs(status));
}

static u64 timestamp_to_ns(struct msgdma_pref_extended_desc *desc)
{
	u64 ns = 0;
	u64 second;
	u32 tmp;

	tmp = desc->timestamp_96b[0] >> 16;
	tmp |= (desc->timestamp_96b[1] << 16);

	second = desc->timestamp_96b[2];
	second <<= 16;
	second |= ((desc->timestamp_96b[1] & 0xffff0000) >> 16);

	ns = second * NSEC_PER_SEC + tmp;

	return ns;
}

/* Setup TX descriptor
 *   -> this should never be called when a descriptor isn't available
 */

netdev_tx_t msgdma_pref_tx_buffer(struct altera_dma_private *priv,
				  struct altera_dma_buffer *buffer)
{
	u32 desc_entry = priv->tx_prod % (priv->tx_ring_size * 2);
	struct msgdma_pref_extended_desc *tx_descs = priv->pref_txdesc;

	/* if for some reason the descriptor is still owned by hardware */
	if (unlikely(tx_descs[desc_entry].desc_control
		     & MSGDMA_PREF_DESC_CTL_OWNED_BY_HW)) {
		if (!netif_queue_stopped(priv->dev))
			netif_stop_queue(priv->dev);
		return NETDEV_TX_BUSY;
	}

	/* write descriptor entries */
	tx_descs[desc_entry].len = buffer->len;
	tx_descs[desc_entry].read_addr_lo = lower_32_bits(buffer->dma_addr);
	tx_descs[desc_entry].read_addr_hi = upper_32_bits(buffer->dma_addr);

	/*
	 * Ensure that the high and low address bits of the descriptor are
	 * written prior to the go bit being set.
	 */
	dma_wmb();

	/* set the control bits and set owned by hw */
	tx_descs[desc_entry].desc_control = (MSGDMA_DESC_CTL_TX_SINGLE
			| MSGDMA_PREF_DESC_CTL_OWNED_BY_HW);

	if (netif_msg_tx_queued(priv))
		netdev_info(priv->dev, "%s: cons: %d prod: %d",
			    __func__, priv->tx_cons, priv->tx_prod);

	return NETDEV_TX_OK;
}

u32 msgdma_pref_tx_completions(struct altera_dma_private *priv)
{
	u32 control;
	u32 ready = 0;
	u32 cons = priv->tx_cons;
	u32 desc_ringsize = priv->tx_ring_size * 2;
	u32 ringsize = priv->tx_ring_size;
	u64 ns = 0;
	struct msgdma_pref_extended_desc *cur;
	struct altera_dma_buffer *tx_buff;
	struct skb_shared_hwtstamps shhwtstamp;
	int i;

	if (netif_msg_tx_done(priv))
		for (i = 0; i < desc_ringsize; i++)
			netdev_info(priv->dev, "%s: desc: %d control 0x%x\n",
				    __func__, i,
				    priv->pref_txdesc[i].desc_control);

	cur = &priv->pref_txdesc[cons % desc_ringsize];
	control = cur->desc_control;
	tx_buff = &priv->tx_ring[cons % ringsize];

	while (!(control & MSGDMA_PREF_DESC_CTL_OWNED_BY_HW) &&
	       (priv->tx_prod != (cons + ready)) && control) {
		if (skb_shinfo(tx_buff->skb)->tx_flags & SKBTX_IN_PROGRESS) {
			/* Timestamping is enabled, pass timestamp back */
			ns = timestamp_to_ns(cur);
			memset(&shhwtstamp, 0,
			       sizeof(struct skb_shared_hwtstamps));
			shhwtstamp.hwtstamp = ns_to_ktime(ns);
			skb_tstamp_tx(tx_buff->skb, &shhwtstamp);
		}

		if (netif_msg_tx_done(priv))
			netdev_info(priv->dev, "%s: cur: %d ts: %lld ns\n",
				    __func__,
				    ((cons + ready) % desc_ringsize), ns);

		/* clear data */
		cur->desc_control = 0;
		cur->timestamp_96b[0] = 0;
		cur->timestamp_96b[1] = 0;
		cur->timestamp_96b[2] = 0;

		ready++;
		cur = &priv->pref_txdesc[(cons + ready) % desc_ringsize];
		tx_buff = &priv->tx_ring[(cons + ready) % ringsize];
		control = cur->desc_control;
	}

	return ready;
}

static void msgdma_confirm_fill_levels(struct altera_dma_private *priv)
{
	int counter;
	int ret;

	/* Look at the prefill level and wait for it to become 0 */

	/* 1. Tx Read write fill level */
	counter = 0;
	while (counter++ < ALTERA_TSE_SW_RESET_WATCHDOG_CNTR) {
		ret = csrrd32(priv->tx_dma_csr, msgdma_csroffs(rw_fill_level));
		ret &= MSGDMA_CSR_FILL_LEVEL_VALID;

		if (ret == 0)
			break;

		udelay(1);
	}

	if (counter >= ALTERA_TSE_SW_RESET_WATCHDOG_CNTR)
		netdev_err(priv->dev,
			   "Tx DMA RW Fill level never cleared! 0x%X\n", ret);

	/* 2. TX response fill level */
	counter = 0;
	while (counter++ < ALTERA_TSE_SW_RESET_WATCHDOG_CNTR) {
		ret = MSGDMA_CSR_RESP_FILL_LEVEL_GET(csrrd32(priv->tx_dma_csr,
						     msgdma_csroffs(resp_fill_level)));
		if (ret == 0)
			break;

		udelay(1);
	}

	if (counter >= ALTERA_TSE_SW_RESET_WATCHDOG_CNTR)
		netdev_err(priv->dev,
			   "Rx DMA RW Fill level never cleared! 0x%X\n", ret);

	/* 3. RX response fill level */
	counter = 0;
	while (counter++ < ALTERA_TSE_SW_RESET_WATCHDOG_CNTR) {
		ret = MSGDMA_CSR_RESP_FILL_LEVEL_GET(csrrd32(priv->rx_dma_csr,
						     msgdma_csroffs(resp_fill_level)));
		if (ret == 0)
			break;

		udelay(1);
	}

	if (counter >= ALTERA_TSE_SW_RESET_WATCHDOG_CNTR)
		netdev_err(priv->dev,
			   "Rx DMA RW Fill level never cleared! 0x%X\n", ret);

	return;
}

void msgdma_pref_quiese(struct altera_dma_private *priv) {

	int counter;

	/* Stop the dispatcher from processing any descriptor */
	tse_set_bit(priv->tx_dma_csr, msgdma_csroffs(control),MSGDMA_CSR_CTL_STOP_DESCS);
	tse_set_bit(priv->rx_dma_csr, msgdma_csroffs(control),MSGDMA_CSR_CTL_STOP_DESCS);

        /* wait for the stop */
	counter = 0;
	while (counter++ < ALTERA_TSE_SW_RESET_WATCHDOG_CNTR) {
		if (tse_bit_is_set(priv->tx_dma_csr,
				   msgdma_csroffs(status),
				   MSGDMA_CSR_STAT_STOPPED))
			break;

		udelay(1);
        }

	if (counter >= ALTERA_TSE_SW_RESET_WATCHDOG_CNTR)
		netdev_err(priv->dev,
			   "TX DMA stop bit not set");

	counter = 0;
	while (counter++ < ALTERA_TSE_SW_RESET_WATCHDOG_CNTR) {
		if (tse_bit_is_set(priv->rx_dma_csr,
				   msgdma_csroffs(status),
				   MSGDMA_CSR_STAT_STOPPED))
			break;

            udelay(1);
	}
}

void msgdma_pref_reset(struct altera_dma_private *priv)
{
	int counter;

	/* turn off polling */
	tse_clear_bit(priv->rx_pref_csr, msgdma_pref_csroffs(control),
		      MSGDMA_PREF_CTL_DESC_POLL_EN);
	tse_clear_bit(priv->tx_pref_csr, msgdma_pref_csroffs(control),
		      MSGDMA_PREF_CTL_DESC_POLL_EN);

	msgdma_pref_quiese(priv);

	//Look at the prefill level and wait for it to become 0
	msgdma_confirm_fill_levels(priv);

	/* Reset the RX Prefetcher */
	csrwr32(MSGDMA_PREF_STAT_IRQ, priv->rx_pref_csr,
		msgdma_pref_csroffs(status));
	csrwr32(MSGDMA_PREF_CTL_RESET, priv->rx_pref_csr,
		msgdma_pref_csroffs(control));

	counter = 0;
	while (counter++ < ALTERA_TSE_SW_RESET_WATCHDOG_CNTR) {
		if (tse_bit_is_clear(priv->rx_pref_csr,
				     msgdma_pref_csroffs(control),
				     MSGDMA_PREF_CTL_RESET))
			break;
		udelay(1);
	}

	if (counter >= ALTERA_TSE_SW_RESET_WATCHDOG_CNTR)
		netif_warn(priv, drv, priv->dev,
			   "TSE Rx Prefetcher reset bit never cleared!\n");

	/* Reset the TX Prefetcher */
	csrwr32(MSGDMA_PREF_STAT_IRQ, priv->tx_pref_csr,
		msgdma_pref_csroffs(status));
	csrwr32(MSGDMA_PREF_CTL_RESET, priv->tx_pref_csr,
		msgdma_pref_csroffs(control));

	counter = 0;
	while (counter++ < ALTERA_TSE_SW_RESET_WATCHDOG_CNTR) {
		if (tse_bit_is_clear(priv->tx_pref_csr,
				     msgdma_pref_csroffs(control),
				     MSGDMA_PREF_CTL_RESET))
			break;
		udelay(1);
	}

	if (counter >= ALTERA_TSE_SW_RESET_WATCHDOG_CNTR)
		netif_warn(priv, drv, priv->dev,
			   "TSE Tx Prefetcher reset bit never cleared!\n");

	/* clear all status bits */
	csrwr32(MSGDMA_PREF_STAT_IRQ, priv->tx_pref_csr,
		msgdma_pref_csroffs(status));

	/* Reset mSGDMA dispatchers*/
	msgdma_reset(priv);
}

/* Setup the RX and TX prefetchers to poll the descriptor chain */
void msgdma_pref_start_rxdma(struct altera_dma_private *priv)
{
	csrwr32(priv->rx_poll_freq, priv->rx_pref_csr,
		msgdma_pref_csroffs(desc_poll_freq));
	csrwr32(lower_32_bits(priv->pref_rxdescphys), priv->rx_pref_csr,
		msgdma_pref_csroffs(next_desc_lo));
	csrwr32(upper_32_bits(priv->pref_rxdescphys), priv->rx_pref_csr,
		msgdma_pref_csroffs(next_desc_hi));
	tse_set_bit(priv->rx_pref_csr, msgdma_pref_csroffs(control),
		    MSGDMA_PREF_CTL_DESC_POLL_EN | MSGDMA_PREF_CTL_RUN);
}

void msgdma_pref_start_txdma(struct altera_dma_private *priv)
{
	csrwr32(priv->tx_poll_freq, priv->tx_pref_csr,
		msgdma_pref_csroffs(desc_poll_freq));
	csrwr32(lower_32_bits(priv->pref_txdescphys), priv->tx_pref_csr,
		msgdma_pref_csroffs(next_desc_lo));
	csrwr32(upper_32_bits(priv->pref_txdescphys), priv->tx_pref_csr,
		msgdma_pref_csroffs(next_desc_hi));
	tse_set_bit(priv->tx_pref_csr, msgdma_pref_csroffs(control),
		    MSGDMA_PREF_CTL_DESC_POLL_EN | MSGDMA_PREF_CTL_RUN);
}

/* Add MSGDMA Prefetcher Descriptor to descriptor list
 *   -> This should never be called when a descriptor isn't available
 */
void msgdma_pref_add_rx_desc(struct altera_dma_private *priv,
			     struct altera_dma_buffer *rxbuffer)
{
	struct msgdma_pref_extended_desc *rx_descs = priv->pref_rxdesc;
	u32 desc_entry = priv->pref_rx_prod % (priv->rx_ring_size * 2);

	/* write descriptor entries */
	rx_descs[desc_entry].len = priv->rx_dma_buf_sz;
	rx_descs[desc_entry].write_addr_lo = lower_32_bits(rxbuffer->dma_addr);
	rx_descs[desc_entry].write_addr_hi = upper_32_bits(rxbuffer->dma_addr);

	/* set the control bits and set owned by hw */
	rx_descs[desc_entry].desc_control = (MSGDMA_DESC_CTL_END_ON_EOP
			| MSGDMA_DESC_CTL_END_ON_LEN
			| MSGDMA_DESC_CTL_TR_COMP_IRQ
			| MSGDMA_DESC_CTL_EARLY_IRQ
			| MSGDMA_DESC_CTL_TR_ERR_IRQ
			| MSGDMA_DESC_CTL_GO
			| MSGDMA_PREF_DESC_CTL_OWNED_BY_HW);

	/* we need to keep a separate one for rx as RX_DESCRIPTORS are
	 * pre-configured at startup
	 */
	priv->pref_rx_prod++;

	if (netif_msg_rx_status(priv)) {
		netdev_info(priv->dev, "%s: desc: %d buf: %d control 0x%x\n",
			    __func__, desc_entry,
			    priv->rx_prod % priv->rx_ring_size,
			    priv->pref_rxdesc[desc_entry].desc_control);
	}
}

u32 msgdma_pref_rx_status(struct altera_dma_private *priv)
{
	u32 rxstatus = 0;
	u32 pktlength;
	u32 pktstatus;
	u64 ns = 0;
	u32 entry = priv->rx_cons % priv->rx_ring_size;
	u32 desc_entry = priv->rx_prod % (priv->rx_ring_size * 2);
	struct msgdma_pref_extended_desc *rx_descs = priv->pref_rxdesc;
	struct skb_shared_hwtstamps *shhwtstamp = NULL;
	struct altera_dma_buffer *rx_buff = priv->rx_ring;

	/* if the current entry is not owned by hardware, process it */
	if (!(rx_descs[desc_entry].desc_control
	      & MSGDMA_PREF_DESC_CTL_OWNED_BY_HW) &&
	      rx_descs[desc_entry].desc_control) {
		pktlength = rx_descs[desc_entry].bytes_transferred;
		pktstatus = rx_descs[desc_entry].desc_status;
		rxstatus = pktstatus;
		rxstatus = rxstatus << 16;
		rxstatus |= (pktlength & 0xffff);

		/* get the timestamp */
		if (priv->hwts_rx_en) {
			ns = timestamp_to_ns(&rx_descs[desc_entry]);
			shhwtstamp = skb_hwtstamps(rx_buff[entry].skb);
			memset(shhwtstamp, 0,
			       sizeof(struct skb_shared_hwtstamps));
			shhwtstamp->hwtstamp = ns_to_ktime(ns);
		}

		/* clear data */
		rx_descs[desc_entry].desc_control = 0;
		rx_descs[desc_entry].timestamp_96b[0] = 0;
		rx_descs[desc_entry].timestamp_96b[1] = 0;
		rx_descs[desc_entry].timestamp_96b[2] = 0;

		if (netif_msg_rx_status(priv))
			netdev_info(priv->dev, "%s: desc: %d buf: %d ts: %lld ns",
				    __func__, desc_entry, entry, ns);
	}
	return rxstatus;
}
