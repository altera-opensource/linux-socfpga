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

#ifndef __ALTERA_MSGDMAHW_PREFETCHER_H__
#define __ALTERA_MSGDMAHW_PREFETCHER_H__

/* mSGDMA prefetcher extended prefectcher descriptor format
 */
struct msgdma_pref_extended_desc {
	/* data buffer source address low bits */
	u32 read_addr_lo;
	/* data buffer destination address low bits */
	u32 write_addr_lo;
	/* the number of bytes to transfer */
	u32 len;
	/* next descriptor low address */
	u32 next_desc_lo;
	/* number of bytes transferred */
	u32 bytes_transferred;
	u32 desc_status;
	u32 reserved_18;
	/* bit 31:24 write burst */
	/* bit 23:16 read burst */
	/* bit 15:0  sequence number */
	u32 burst_seq_num;
	/* bit 31:16 write stride */
	/* bit 15:0  read stride */
	u32 stride;
	/* data buffer source address high bits */
	u32 read_addr_hi;
	/* data buffer destination address high bits */
	u32 write_addr_hi;
	/* next descriptor high address */
	u32 next_desc_hi;
	/* prefetcher mod now writes these reserved bits*/
	/* Response bits [191:160] */
	u32 timestamp_96b[3];
	/* desc_control */
	u32 desc_control;
};

/* mSGDMA Prefetcher Descriptor Status bits */
#define MSGDMA_PREF_DESC_STAT_STOPPED_ON_EARLY		BIT(8)
#define MSGDMA_PREF_DESC_STAT_MASK			0xFF

/* mSGDMA Prefetcher Descriptor Control bits */
/* bit 31 and bits 29-0 are the same as the normal dispatcher ctl flags */
#define MSGDMA_PREF_DESC_CTL_OWNED_BY_HW		BIT(30)

/* mSGDMA Prefetcher CSR */
struct msgdma_prefetcher_csr {
	u32 control;
	u32 next_desc_lo;
	u32 next_desc_hi;
	u32 desc_poll_freq;
	u32 status;
};

/* mSGDMA Prefetcher Control */
#define MSGDMA_PREF_CTL_PARK				BIT(4)
#define MSGDMA_PREF_CTL_GLOBAL_INTR			BIT(3)
#define MSGDMA_PREF_CTL_RESET				BIT(2)
#define MSGDMA_PREF_CTL_DESC_POLL_EN			BIT(1)
#define MSGDMA_PREF_CTL_RUN				BIT(0)

#define MSGDMA_PREF_POLL_FREQ_MASK			0xFFFF

/* mSGDMA Prefetcher Status */
#define MSGDMA_PREF_STAT_IRQ				BIT(0)

#define msgdma_pref_csroffs(a) (offsetof(struct msgdma_prefetcher_csr, a))
#define msgdma_pref_descroffs(a) (offsetof(struct msgdma_pref_extended_desc, a))

#endif /* __ALTERA_MSGDMAHW_PREFETCHER_H__*/
