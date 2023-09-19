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

#ifndef __ALTERA_MSGDMA_H__
#define __ALTERA_MSGDMA_H__

void msgdma_reset(struct altera_dma_private *priv);
void msgdma_enable_txirq(struct altera_dma_private *priv);
void msgdma_enable_rxirq(struct altera_dma_private *priv);
void msgdma_disable_rxirq(struct altera_dma_private *priv);
void msgdma_disable_txirq(struct altera_dma_private *priv);
void msgdma_clear_rxirq(struct altera_dma_private *priv);
void msgdma_clear_txirq(struct altera_dma_private *priv);
u32 msgdma_tx_completions(struct altera_dma_private *priv);
void msgdma_add_rx_desc(struct altera_dma_private *priv,
			struct altera_dma_buffer *buffer);
netdev_tx_t msgdma_tx_buffer(struct altera_dma_private *priv,
			     struct altera_dma_buffer *buffer);
u32 msgdma_rx_status(struct altera_dma_private *priv);
int msgdma_initialize(struct altera_dma_private *priv);
void msgdma_uninitialize(struct altera_dma_private *priv);
void msgdma_start_rxdma(struct altera_dma_private *priv);

#endif /*  __ALTERA_MSGDMA_H__ */
