/*
 * DMA coherent memory allocation.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * Copyright (C) 2002 - 2005 Tensilica Inc.
 * Copyright (C) 2015 Cadence Design Systems Inc.
 *
 * Based on version for i386.
 *
 * Chris Zankel <chris@zankel.net>
 * Joe Taylor <joe@tensilica.com, joetylr@yahoo.com>
 */

#include <linux/dma-contiguous.h>
#include <linux/gfp.h>
#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/string.h>
#include <linux/types.h>
#include <asm/cacheflush.h>
#include <asm/io.h>

static void do_cache_op(dma_addr_t dma_handle, size_t size,
			void (*fn)(unsigned long, unsigned long))
{
	unsigned long off = dma_handle & (PAGE_SIZE - 1);
	unsigned long pfn = PFN_DOWN(dma_handle);
	struct page *page = pfn_to_page(pfn);

	if (!PageHighMem(page))
		fn((unsigned long)bus_to_virt(dma_handle), size);
	else
		while (size > 0) {
			size_t sz = min_t(size_t, size, PAGE_SIZE - off);
			void *vaddr = kmap_atomic(page);

			fn((unsigned long)vaddr + off, sz);
			kunmap_atomic(vaddr);
			off = 0;
			++page;
			size -= sz;
		}
}

static void xtensa_sync_single_for_cpu(struct device *dev,
				       dma_addr_t dma_handle, size_t size,
				       enum dma_data_direction dir)
{
	switch (dir) {
	case DMA_BIDIRECTIONAL:
	case DMA_FROM_DEVICE:
		do_cache_op(dma_handle, size, __invalidate_dcache_range);
		break;

	case DMA_NONE:
		BUG();
		break;

	default:
		break;
	}
}

static void xtensa_sync_single_for_device(struct device *dev,
					  dma_addr_t dma_handle, size_t size,
					  enum dma_data_direction dir)
{
	switch (dir) {
	case DMA_BIDIRECTIONAL:
	case DMA_TO_DEVICE:
		if (XCHAL_DCACHE_IS_WRITEBACK)
			do_cache_op(dma_handle, size, __flush_dcache_range);
		break;

	case DMA_NONE:
		BUG();
		break;

	default:
		break;
	}
}

static void xtensa_sync_sg_for_cpu(struct device *dev,
				   struct scatterlist *sg, int nents,
				   enum dma_data_direction dir)
{
	struct scatterlist *s;
	int i;

	for_each_sg(sg, s, nents, i) {
		xtensa_sync_single_for_cpu(dev, sg_dma_address(s),
					   sg_dma_len(s), dir);
	}
}

static void xtensa_sync_sg_for_device(struct device *dev,
				      struct scatterlist *sg, int nents,
				      enum dma_data_direction dir)
{
	struct scatterlist *s;
	int i;

	for_each_sg(sg, s, nents, i) {
		xtensa_sync_single_for_device(dev, sg_dma_address(s),
					      sg_dma_len(s), dir);
	}
}

/*
 * Note: We assume that the full memory space is always mapped to 'kseg'
 *	 Otherwise we have to use page attributes (not implemented).
 */

static void *xtensa_dma_alloc(struct device *dev, size_t size,
			      dma_addr_t *handle, gfp_t flag,
			      unsigned long attrs)
{
	unsigned long ret;
	unsigned long uncached = 0;
	unsigned long count = PAGE_ALIGN(size) >> PAGE_SHIFT;
	struct page *page = NULL;

	/* ignore region speicifiers */

	flag &= ~(__GFP_DMA | __GFP_HIGHMEM);

	if (dev == NULL || (dev->coherent_dma_mask < 0xffffffff))
		flag |= GFP_DMA;

	if (gfpflags_allow_blocking(flag))
		page = dma_alloc_from_contiguous(dev, count, get_order(size),
						 flag);

	if (!page)
		page = alloc_pages(flag, get_order(size));

	if (!page)
		return NULL;

	ret = (unsigned long)page_address(page);

	/* We currently don't support coherent memory outside KSEG */

	BUG_ON(ret < XCHAL_KSEG_CACHED_VADDR ||
	       ret > XCHAL_KSEG_CACHED_VADDR + XCHAL_KSEG_SIZE - 1);

	uncached = ret + XCHAL_KSEG_BYPASS_VADDR - XCHAL_KSEG_CACHED_VADDR;
	*handle = virt_to_bus((void *)ret);
	__invalidate_dcache_range(ret, size);

	return (void *)uncached;
}

static void xtensa_dma_free(struct device *dev, size_t size, void *vaddr,
			    dma_addr_t dma_handle, unsigned long attrs)
{
	unsigned long addr = (unsigned long)vaddr +
		XCHAL_KSEG_CACHED_VADDR - XCHAL_KSEG_BYPASS_VADDR;
	struct page *page = virt_to_page(addr);
	unsigned long count = PAGE_ALIGN(size) >> PAGE_SHIFT;

	BUG_ON(addr < XCHAL_KSEG_CACHED_VADDR ||
	       addr > XCHAL_KSEG_CACHED_VADDR + XCHAL_KSEG_SIZE - 1);

	if (!dma_release_from_contiguous(dev, page, count))
		__free_pages(page, get_order(size));
}

static dma_addr_t xtensa_map_page(struct device *dev, struct page *page,
				  unsigned long offset, size_t size,
				  enum dma_data_direction dir,
				  unsigned long attrs)
{
	dma_addr_t dma_handle = page_to_phys(page) + offset;

	if (!(attrs & DMA_ATTR_SKIP_CPU_SYNC))
		xtensa_sync_single_for_device(dev, dma_handle, size, dir);

	return dma_handle;
}

static void xtensa_unmap_page(struct device *dev, dma_addr_t dma_handle,
			      size_t size, enum dma_data_direction dir,
			      unsigned long attrs)
{
	if (!(attrs & DMA_ATTR_SKIP_CPU_SYNC))
		xtensa_sync_single_for_cpu(dev, dma_handle, size, dir);
}

static int xtensa_map_sg(struct device *dev, struct scatterlist *sg,
			 int nents, enum dma_data_direction dir,
			 unsigned long attrs)
{
	struct scatterlist *s;
	int i;

	for_each_sg(sg, s, nents, i) {
		s->dma_address = xtensa_map_page(dev, sg_page(s), s->offset,
						 s->length, dir, attrs);
	}
	return nents;
}

static void xtensa_unmap_sg(struct device *dev,
			    struct scatterlist *sg, int nents,
			    enum dma_data_direction dir,
			    unsigned long attrs)
{
	struct scatterlist *s;
	int i;

	for_each_sg(sg, s, nents, i) {
		xtensa_unmap_page(dev, sg_dma_address(s),
				  sg_dma_len(s), dir, attrs);
	}
}

int xtensa_dma_mapping_error(struct device *dev, dma_addr_t dma_addr)
{
	return 0;
}

const struct dma_map_ops xtensa_dma_map_ops = {
	.alloc = xtensa_dma_alloc,
	.free = xtensa_dma_free,
	.map_page = xtensa_map_page,
	.unmap_page = xtensa_unmap_page,
	.map_sg = xtensa_map_sg,
	.unmap_sg = xtensa_unmap_sg,
	.sync_single_for_cpu = xtensa_sync_single_for_cpu,
	.sync_single_for_device = xtensa_sync_single_for_device,
	.sync_sg_for_cpu = xtensa_sync_sg_for_cpu,
	.sync_sg_for_device = xtensa_sync_sg_for_device,
	.mapping_error = xtensa_dma_mapping_error,
};
EXPORT_SYMBOL(xtensa_dma_map_ops);

#define PREALLOC_DMA_DEBUG_ENTRIES (1 << 16)

static int __init xtensa_dma_init(void)
{
	dma_debug_init(PREALLOC_DMA_DEBUG_ENTRIES);
	return 0;
}
fs_initcall(xtensa_dma_init);
