// SPDX-License-Identifier: GPL-2.0
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

#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/module.h>

#include "altera_eth_dma.h"
#include "altera_utils.h"

/* Probe DMA
 */
int altera_eth_dma_probe(struct platform_device *pdev,
			 struct altera_dma_private *priv,
			 enum altera_dma_type type)
{
	int ret = -ENODEV;
	struct resource *dma_res;
	void __iomem *descmap;

	/* xSGDMA Rx Dispatcher address space */
	ret = request_and_map(pdev, "rx_csr", &dma_res,
			      &priv->rx_dma_csr);
	if (ret)
		goto err;

	if (netif_msg_probe(priv))
		dev_info(&pdev->dev, "\tDMA RX CSR at 0x%08lx\n",
			 (unsigned long)dma_res->start);

	/* mSGDMA Tx Dispatcher address space */
	ret = request_and_map(pdev, "tx_csr", &dma_res,
			      &priv->tx_dma_csr);
	if (ret)
		goto err;

	if (netif_msg_probe(priv))
		dev_info(&pdev->dev, "\tDMA TX CSR at 0x%08lx\n",
			 (unsigned long)dma_res->start);

	switch (type) {
	case ALTERA_DTYPE_SGDMA:
		/* Get the mapped address to the SGDMA descriptor memory */
		ret = request_and_map(pdev, "s1", &dma_res, &descmap);
		if (ret)
			break;

		if (netif_msg_probe(priv))
			dev_info(&pdev->dev, "\tDMA Desc Mem at 0x%08lx\n",
				 (unsigned long)dma_res->start);

		/* Start of that memory is for transmit descriptors */
		priv->tx_dma_desc = descmap;

		/* First half is for tx descriptors, other half for tx */
		priv->txdescmem = resource_size(dma_res) / 2;

		priv->txdescmem_busaddr = (dma_addr_t)dma_res->start;

		priv->rx_dma_desc = (void __iomem *)((uintptr_t)(descmap +
						     priv->txdescmem));
		priv->rxdescmem = resource_size(dma_res) / 2;
		priv->rxdescmem_busaddr = dma_res->start;
		priv->rxdescmem_busaddr += priv->txdescmem;

		if (upper_32_bits(priv->rxdescmem_busaddr))
			ret = -EINVAL;

		if (upper_32_bits(priv->txdescmem_busaddr))
			ret = -EINVAL;
		break;
	case ALTERA_DTYPE_MSGDMA:
		ret = request_and_map(pdev, "rx_resp", &dma_res,
				      &priv->rx_dma_resp);
		if (ret)
			break;
		if (netif_msg_probe(priv))
			dev_info(&pdev->dev, "\tRX Resp Slave at 0x%08lx\n",
				 (unsigned long)dma_res->start);

		ret = request_and_map(pdev, "tx_desc", &dma_res,
				      &priv->tx_dma_desc);
		if (ret)
			break;
		if (netif_msg_probe(priv))
			dev_info(&pdev->dev, "\tTX Desc Slave at 0x%08lx\n",
				 (unsigned long)dma_res->start);

		priv->txdescmem = resource_size(dma_res);
		priv->txdescmem_busaddr = dma_res->start;

		ret = request_and_map(pdev, "rx_desc", &dma_res,
				      &priv->rx_dma_desc);
		if (ret)
			break;
		if (netif_msg_probe(priv))
			dev_info(&pdev->dev, "\tRX Desc Slave at 0x%08lx\n",
				 (unsigned long)dma_res->start);

		priv->rxdescmem = resource_size(dma_res);
		priv->rxdescmem_busaddr = dma_res->start;
		break;
	case ALTERA_DTYPE_MSGDMA_PTP:
		ret = request_and_map(pdev, "rx_desc", &dma_res,
				      &priv->rx_dma_desc);
		if (ret)
			break;
		if (netif_msg_probe(priv))
			dev_info(&pdev->dev, "\tRX Desc Slave at 0x%08lx\n",
				 (unsigned long)dma_res->start);

		priv->rxdescmem = resource_size(dma_res);
		priv->rxdescmem_busaddr = dma_res->start;

		ret = request_and_map(pdev, "rx_resp", &dma_res,
				      &priv->rx_dma_resp);
		if (ret)
			break;
		if (netif_msg_probe(priv))
			dev_info(&pdev->dev, "\tRX Resp Slave at 0x%08lx\n",
				 (unsigned long)dma_res->start);

		ret = request_and_map(pdev, "tx_desc", &dma_res,
				      &priv->tx_dma_desc);
		if (ret)
			break;
		if (netif_msg_probe(priv))
			dev_info(&pdev->dev, "\tTX Desc Slave at 0x%08lx\n",
				 (unsigned long)dma_res->start);

		priv->txdescmem = resource_size(dma_res);
		priv->txdescmem_busaddr = dma_res->start;

		ret = request_and_map(pdev, "tx_resp", &dma_res,
				      &priv->tx_dma_resp);
		if (ret)
			break;
		if (netif_msg_probe(priv))
			dev_info(&pdev->dev, "\tTX Resp Slave at 0x%08lx\n",
				 (unsigned long)dma_res->start);
		break;
	case ALTERA_DTYPE_MSGDMA_PREF:
		/* mSGDMA Rx Prefetcher address space */
		ret = request_and_map(pdev, "rx_pref", &dma_res,
				      &priv->rx_pref_csr);
		if (ret)
			break;

		/* mSGDMA Tx Prefetcher address space */
		ret = request_and_map(pdev, "tx_pref", &dma_res,
				      &priv->tx_pref_csr);
		if (ret)
			break;

		/* get prefetcher rx poll frequency from device tree */
		if (of_property_read_u32(pdev->dev.of_node,
					 "rx-poll-freq",
					 &priv->rx_poll_freq)) {
			dev_info(&pdev->dev, "Defaulting RX Poll Frequency to 128\n");
			priv->rx_poll_freq = 128;
		}

		/* get prefetcher rx poll frequency from device tree */
		if (of_property_read_u32(pdev->dev.of_node,
					 "tx-poll-freq",
					 &priv->tx_poll_freq)) {
			dev_info(&pdev->dev, "Defaulting TX Poll Frequency to 128\n");
			priv->tx_poll_freq = 128;
		}
		break;
	default:
		ret = -ENODEV;
		break;
	}
err:
	return ret;
};
EXPORT_SYMBOL_GPL(altera_eth_dma_probe);
MODULE_LICENSE("GPL");
