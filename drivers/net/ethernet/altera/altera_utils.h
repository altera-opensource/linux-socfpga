/* SPDX-License-Identifier: GPL-2.0-only */
/* Altera TSE SGDMA and MSGDMA Linux driver
 * Copyright (C) 2014 Altera Corporation. All rights reserved
 */

#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/io.h>

#ifndef __ALTERA_UTILS_H__
#define __ALTERA_UTILS_H__

void tse_set_bit(void __iomem *ioaddr, size_t offs, u32 bit_mask);
void tse_clear_bit(void __iomem *ioaddr, size_t offs, u32 bit_mask);
int tse_bit_is_set(void __iomem *ioaddr, size_t offs, u32 bit_mask);
int tse_bit_is_clear(void __iomem *ioaddr, size_t offs, u32 bit_mask);
int request_and_map(struct platform_device *pdev, const char *name,
		    struct resource **res, void __iomem **ptr);

static inline
u32 csrrd32(void __iomem *mac, size_t offs)
{
	void __iomem *paddr = (void __iomem *)((uintptr_t)mac + offs);

	return readl(paddr);
}

static inline
u16 csrrd16(void __iomem *mac, size_t offs)
{
	void __iomem *paddr = (void __iomem *)((uintptr_t)mac + offs);

	return readw(paddr);
}

static inline
u8 csrrd8(void __iomem *mac, size_t offs)
{
	void __iomem *paddr = (void __iomem *)((uintptr_t)mac + offs);

	return readb(paddr);
}

static inline
void csrwr32(u32 val, void __iomem *mac, size_t offs)
{
	void __iomem *paddr = (void __iomem *)((uintptr_t)mac + offs);

	writel(val, paddr);
}

static inline
void csrwr16(u16 val, void __iomem *mac, size_t offs)
{
	void __iomem *paddr = (void __iomem *)((uintptr_t)mac + offs);

	writew(val, paddr);
}

static inline
void csrwr8(u8 val, void __iomem *mac, size_t offs)
{
	void __iomem *paddr = (void __iomem *)((uintptr_t)mac + offs);

	writeb(val, paddr);
}
#endif /* __ALTERA_UTILS_H__*/
