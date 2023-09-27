/* SPDX-License-Identifier: GPL-2.0 */
/* DMA support for Intel FPGA Quad-Speed Ethernet MAC driver
 * Copyright (C) 2019 Intel Corporation. All rights reserved
 *
 * Contributors:
 *   Dalon Westergreen
 *   Thomas Chou
 *   Ian Abbott
 *   Yuriy Kozlov
 *   Tobias Klauser
 *   Andriy Smolskyy
 *   Roman Bulgakov
 *   Dmytro Mytarchuk
 *   Matthew Gerlach
 *   Joyce Ooi
 */

#ifndef __ALTERA_ETH_DMA_H__
#define __ALTERA_ETH_DMA_H__

#include <linux/bitops.h>
#include <linux/list.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>

#define ALTERA_TSE_SW_RESET_WATCHDOG_CNTR	10000

struct altera_dma_private {
	struct net_device *dev;
	struct device *device;

	/* mSGDMA Rx Dispatcher address space */
	void __iomem *rx_dma_csr;
	void __iomem *rx_dma_desc;
	void __iomem *rx_dma_resp;

	/* mSGDMA Tx Dispatcher address space */
	void __iomem *tx_dma_csr;
	void __iomem *tx_dma_desc;
	void __iomem *tx_dma_resp;

	/* mSGDMA Rx Prefecher address space */
	void __iomem *rx_pref_csr;
	struct msgdma_pref_extended_desc *pref_rxdesc;
	dma_addr_t pref_rxdescphys;
	u32 pref_rx_prod;

	/* mSGDMA Tx Prefecher address space */
	void __iomem *tx_pref_csr;
	struct msgdma_pref_extended_desc *pref_txdesc;
	dma_addr_t pref_txdescphys;
	u32 rx_poll_freq;
	u32 tx_poll_freq;

	/* Rx buffers queue */
	struct altera_dma_buffer *rx_ring;
	u32 rx_cons;
	u32 rx_prod;
	u32 rx_ring_size;
	u32 rx_dma_buf_sz;

	/* Tx ring buffer */
	struct altera_dma_buffer *tx_ring;
	u32 tx_prod;
	u32 tx_cons;
	u32 tx_ring_size;

	/* Descriptor memory info for managing SGDMA */
	u32 txdescmem;
	u32 rxdescmem;
	dma_addr_t rxdescmem_busaddr;
	dma_addr_t txdescmem_busaddr;
	u32 txctrlreg;
	u32 rxctrlreg;
	dma_addr_t rxdescphys;
	dma_addr_t txdescphys;

	struct list_head txlisthd;
	struct list_head rxlisthd;

	int hwts_tx_en;
	int hwts_rx_en;

	/* ethtool msglvl option */
	u32 msg_enable;
};

/* Wrapper around a pointer to a socket buffer,
 * so a DMA handle can be stored along with the buffer
 */
struct altera_dma_buffer {
	struct list_head lh;
	struct sk_buff *skb;
	dma_addr_t dma_addr;
	u32 len;
	int mapped_as_page;
};

enum altera_dma_type {
	ALTERA_DTYPE_SGDMA = 1,
	ALTERA_DTYPE_MSGDMA = 2,
	ALTERA_DTYPE_MSGDMA_PTP = 3,
	ALTERA_DTYPE_MSGDMA_PREF = 4,
};

struct altera_dma_resp {
	u32 status;
	u32 external_resp[4];
};

/* standard DMA interface for SGDMA and MSGDMA */
struct altera_dmaops {
	enum altera_dma_type altera_dtype;
	int dmamask;
	void (*quiese_pref)(struct altera_dma_private *priv);
	void (*reset_dma)(struct altera_dma_private *priv);
	bool (*is_txirq_set)(struct altera_dma_private *priv);
	bool (*is_rxirq_set)(struct altera_dma_private *priv);
	void (*enable_txirq)(struct altera_dma_private *priv);
	void (*enable_rxirq)(struct altera_dma_private *priv);
	void (*disable_txirq)(struct altera_dma_private *priv);
	void (*disable_rxirq)(struct altera_dma_private *priv);
	void (*clear_txirq)(struct altera_dma_private *priv);
	void (*clear_rxirq)(struct altera_dma_private *priv);
	int (*tx_buffer)(struct altera_dma_private *priv,
			 struct altera_dma_buffer *buffer);
	u32 (*tx_completions)(struct altera_dma_private *priv);
	void (*add_rx_desc)(struct altera_dma_private *priv,
			    struct altera_dma_buffer *buffer);
	u32 (*get_rx_status)(struct altera_dma_private *priv);
	int (*init_dma)(struct altera_dma_private *priv);
	void (*uninit_dma)(struct altera_dma_private *priv);
	void (*start_rxdma)(struct altera_dma_private *priv);
	void (*start_txdma)(struct altera_dma_private *priv);
};

int altera_eth_dma_probe(struct platform_device *pdev,
			 struct altera_dma_private *priv,
			 enum altera_dma_type type);

#endif /* __ALTERA_ETH_DMA_H__ */
