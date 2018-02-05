/*
 * Copyright (C) 2016 Felix Fietkau <nbd@nbd.name>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "mt76.h"
#include "trace.h"

static u32 mt76_mmio_rr(struct mt76_dev *dev, u32 offset)
{
	u32 val;

	val = ioread32(dev->regs + offset);
	trace_reg_rr(dev, offset, val);

	return val;
}

static void mt76_mmio_wr(struct mt76_dev *dev, u32 offset, u32 val)
{
	trace_reg_wr(dev, offset, val);
	iowrite32(val, dev->regs + offset);
}

static u32 mt76_mmio_rmw(struct mt76_dev *dev, u32 offset, u32 mask, u32 val)
{
	val |= mt76_mmio_rr(dev, offset) & ~mask;
	mt76_mmio_wr(dev, offset, val);
	return val;
}

static void mt76_mmio_copy(struct mt76_dev *dev, u32 offset, const void *data,
			   int len)
{
	__iowrite32_copy(dev->regs + offset, data, len >> 2);
}

void mt76_mmio_init(struct mt76_dev *dev, void __iomem *regs)
{
	static const struct mt76_bus_ops mt76_mmio_ops = {
		.rr = mt76_mmio_rr,
		.rmw = mt76_mmio_rmw,
		.wr = mt76_mmio_wr,
		.copy = mt76_mmio_copy,
	};

	dev->bus = &mt76_mmio_ops;
	dev->regs = regs;
}
EXPORT_SYMBOL_GPL(mt76_mmio_init);
