/* Altera TSE SGDMA and MSGDMA Linux driver
 * Copyright (C) 2014 Altera Corporation. All rights reserved
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __ALTERA_SGDMA_H__
#define __ALTERA_SGDMA_H__

void sgdma_reset(struct altera_dma_private *priv);
void sgdma_enable_txirq(struct altera_dma_private *priv);
void sgdma_enable_rxirq(struct altera_dma_private *priv);
void sgdma_disable_rxirq(struct altera_dma_private *priv);
void sgdma_disable_txirq(struct altera_dma_private *priv);
void sgdma_clear_rxirq(struct altera_dma_private *priv);
void sgdma_clear_txirq(struct altera_dma_private *priv);
netdev_tx_t sgdma_tx_buffer(struct altera_dma_private *priv,
			    struct altera_dma_buffer *buffer);
u32 sgdma_tx_completions(struct altera_dma_private *priv);
void sgdma_add_rx_desc(struct altera_dma_private *priv,
		       struct altera_dma_buffer *buffer);
void sgdma_status(struct altera_dma_private *priv);
u32 sgdma_rx_status(struct altera_dma_private *priv);
int sgdma_initialize(struct altera_dma_private *priv);
void sgdma_uninitialize(struct altera_dma_private *priv);
void sgdma_start_rxdma(struct altera_dma_private *priv);

#endif /*  __ALTERA_SGDMA_H__ */
