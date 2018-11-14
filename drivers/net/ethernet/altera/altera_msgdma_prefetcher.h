/* SPDX-License-Identifier: GPL-2.0 */
/* MSGDMA Prefetcher driver for Altera ethernet devices
 *
 * Copyright (C) 2020 Intel Corporation. All rights reserved.
 * Author(s):
 *   Dalon Westergreen <dalon.westergreen@intel.com>
 */

#ifndef __ALTERA_PREF_MSGDMA_H__
#define __ALTERA_PREF_MSGDMA_H__

void msgdma_pref_reset(struct altera_tse_private *priv);
void msgdma_pref_enable_txirq(struct altera_tse_private *priv);
void msgdma_pref_enable_rxirq(struct altera_tse_private *priv);
void msgdma_pref_disable_rxirq(struct altera_tse_private *priv);
void msgdma_pref_disable_txirq(struct altera_tse_private *priv);
void msgdma_pref_clear_rxirq(struct altera_tse_private *priv);
void msgdma_pref_clear_txirq(struct altera_tse_private *priv);
u32 msgdma_pref_tx_completions(struct altera_tse_private *priv);
void msgdma_pref_add_rx_desc(struct altera_tse_private *priv,
			     struct tse_buffer *buffer);
netdev_tx_t msgdma_pref_tx_buffer(struct altera_tse_private *priv,
				  struct tse_buffer *buffer);
u32 msgdma_pref_rx_status(struct altera_tse_private *priv);
int msgdma_pref_initialize(struct altera_tse_private *priv);
void msgdma_pref_uninitialize(struct altera_tse_private *priv);
void msgdma_pref_start_rxdma(struct altera_tse_private *priv);
void msgdma_pref_start_txdma(struct altera_tse_private *priv);

#endif /*  __ALTERA_PREF_MSGDMA_H__ */
