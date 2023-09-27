/* SPDX-License-Identifier: GPL-2.0 */
/* MSGDMA Prefetcher driver for Altera ethernet devices
 *
 * Copyright (C) 2020 Intel Corporation.
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
 */

#ifndef __ALTERA_PREF_MSGDMA_H__
#define __ALTERA_PREF_MSGDMA_H__

void msgdma_pref_quiese(struct altera_dma_private *priv);
void msgdma_pref_reset(struct altera_dma_private *priv);
bool msgdma_pref_is_txirq(struct altera_dma_private *priv);
bool msgdma_pref_is_rxirq(struct altera_dma_private *priv);
void msgdma_pref_enable_txirq(struct altera_dma_private *priv);
void msgdma_pref_enable_rxirq(struct altera_dma_private *priv);
void msgdma_pref_disable_rxirq(struct altera_dma_private *priv);
void msgdma_pref_disable_txirq(struct altera_dma_private *priv);
void msgdma_pref_clear_rxirq(struct altera_dma_private *priv);
void msgdma_pref_clear_txirq(struct altera_dma_private *priv);
u32 msgdma_pref_tx_completions(struct altera_dma_private *priv);
void msgdma_pref_add_rx_desc(struct altera_dma_private *priv,
			     struct altera_dma_buffer *buffer);
int msgdma_pref_tx_buffer(struct altera_dma_private *priv,
			  struct altera_dma_buffer *buffer);
u32 msgdma_pref_rx_status(struct altera_dma_private *priv);
int msgdma_pref_initialize(struct altera_dma_private *priv);
void msgdma_pref_uninitialize(struct altera_dma_private *priv);
void msgdma_pref_start_rxdma(struct altera_dma_private *priv);
void msgdma_pref_start_txdma(struct altera_dma_private *priv);

#endif /*  __ALTERA_PREF_MSGDMA_H__ */
