// SPDX-License-Identifier: GPL-2.0
/* bounce buffer handling for block devices
 *
 * - Split from highmem.c
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/mm.h>
#include <linux/export.h>
#include <linux/swap.h>
#include <linux/gfp.h>
#include <linux/bio.h>
#include <linux/pagemap.h>
#include <linux/mempool.h>
#include <linux/blkdev.h>
#include <linux/backing-dev.h>
#include <linux/init.h>
#include <linux/hash.h>
#include <linux/highmem.h>
#include <linux/bootmem.h>
#include <linux/printk.h>
#include <asm/tlbflush.h>

#include <trace/events/block.h>
#include "blk.h"

#define POOL_SIZE	64
#define ISA_POOL_SIZE	16

static struct bio_set *bounce_bio_set, *bounce_bio_split;
static mempool_t *page_pool, *isa_page_pool;

#if defined(CONFIG_HIGHMEM) || defined(CONFIG_NEED_BOUNCE_POOL)
static __init int init_emergency_pool(void)
{
#if defined(CONFIG_HIGHMEM) && !defined(CONFIG_MEMORY_HOTPLUG)
	if (max_pfn <= max_low_pfn)
		return 0;
#endif

	page_pool = mempool_create_page_pool(POOL_SIZE, 0);
	BUG_ON(!page_pool);
	pr_info("pool size: %d pages\n", POOL_SIZE);

	bounce_bio_set = bioset_create(BIO_POOL_SIZE, 0, BIOSET_NEED_BVECS);
	BUG_ON(!bounce_bio_set);
	if (bioset_integrity_create(bounce_bio_set, BIO_POOL_SIZE))
		BUG_ON(1);

	bounce_bio_split = bioset_create(BIO_POOL_SIZE, 0, 0);
	BUG_ON(!bounce_bio_split);

	return 0;
}

__initcall(init_emergency_pool);
#endif

#ifdef CONFIG_HIGHMEM
/*
 * highmem version, map in to vec
 */
static void bounce_copy_vec(struct bio_vec *to, unsigned char *vfrom)
{
	unsigned long flags;
	unsigned char *vto;

	local_irq_save(flags);
	vto = kmap_atomic(to->bv_page);
	memcpy(vto + to->bv_offset, vfrom, to->bv_len);
	kunmap_atomic(vto);
	local_irq_restore(flags);
}

#else /* CONFIG_HIGHMEM */

#define bounce_copy_vec(to, vfrom)	\
	memcpy(page_address((to)->bv_page) + (to)->bv_offset, vfrom, (to)->bv_len)

#endif /* CONFIG_HIGHMEM */

/*
 * allocate pages in the DMA region for the ISA pool
 */
static void *mempool_alloc_pages_isa(gfp_t gfp_mask, void *data)
{
	return mempool_alloc_pages(gfp_mask | GFP_DMA, data);
}

/*
 * gets called "every" time someone init's a queue with BLK_BOUNCE_ISA
 * as the max address, so check if the pool has already been created.
 */
int init_emergency_isa_pool(void)
{
	if (isa_page_pool)
		return 0;

	isa_page_pool = mempool_create(ISA_POOL_SIZE, mempool_alloc_pages_isa,
				       mempool_free_pages, (void *) 0);
	BUG_ON(!isa_page_pool);

	pr_info("isa pool size: %d pages\n", ISA_POOL_SIZE);
	return 0;
}

/*
 * Simple bounce buffer support for highmem pages. Depending on the
 * queue gfp mask set, *to may or may not be a highmem page. kmap it
 * always, it will do the Right Thing
 */
static void copy_to_high_bio_irq(struct bio *to, struct bio *from)
{
	unsigned char *vfrom;
	struct bio_vec tovec, fromvec;
	struct bvec_iter iter;
	/*
	 * The bio of @from is created by bounce, so we can iterate
	 * its bvec from start to end, but the @from->bi_iter can't be
	 * trusted because it might be changed by splitting.
	 */
	struct bvec_iter from_iter = BVEC_ITER_ALL_INIT;

	bio_for_each_segment(tovec, to, iter) {
		fromvec = bio_iter_iovec(from, from_iter);
		if (tovec.bv_page != fromvec.bv_page) {
			/*
			 * fromvec->bv_offset and fromvec->bv_len might have
			 * been modified by the block layer, so use the original
			 * copy, bounce_copy_vec already uses tovec->bv_len
			 */
			vfrom = page_address(fromvec.bv_page) +
				tovec.bv_offset;

			bounce_copy_vec(&tovec, vfrom);
			flush_dcache_page(tovec.bv_page);
		}
		bio_advance_iter(from, &from_iter, tovec.bv_len);
	}
}

static void bounce_end_io(struct bio *bio, mempool_t *pool)
{
	struct bio *bio_orig = bio->bi_private;
	struct bio_vec *bvec, orig_vec;
	int i;
	struct bvec_iter orig_iter = bio_orig->bi_iter;

	/*
	 * free up bounce indirect pages used
	 */
	bio_for_each_segment_all(bvec, bio, i) {
		orig_vec = bio_iter_iovec(bio_orig, orig_iter);
		if (bvec->bv_page != orig_vec.bv_page) {
			dec_zone_page_state(bvec->bv_page, NR_BOUNCE);
			mempool_free(bvec->bv_page, pool);
		}
		bio_advance_iter(bio_orig, &orig_iter, orig_vec.bv_len);
	}

	bio_orig->bi_status = bio->bi_status;
	bio_endio(bio_orig);
	bio_put(bio);
}

static void bounce_end_io_write(struct bio *bio)
{
	bounce_end_io(bio, page_pool);
}

static void bounce_end_io_write_isa(struct bio *bio)
{

	bounce_end_io(bio, isa_page_pool);
}

static void __bounce_end_io_read(struct bio *bio, mempool_t *pool)
{
	struct bio *bio_orig = bio->bi_private;

	if (!bio->bi_status)
		copy_to_high_bio_irq(bio_orig, bio);

	bounce_end_io(bio, pool);
}

static void bounce_end_io_read(struct bio *bio)
{
	__bounce_end_io_read(bio, page_pool);
}

static void bounce_end_io_read_isa(struct bio *bio)
{
	__bounce_end_io_read(bio, isa_page_pool);
}

static void __blk_queue_bounce(struct request_queue *q, struct bio **bio_orig,
			       mempool_t *pool)
{
	struct bio *bio;
	int rw = bio_data_dir(*bio_orig);
	struct bio_vec *to, from;
	struct bvec_iter iter;
	unsigned i = 0;
	bool bounce = false;
	int sectors = 0;
	bool passthrough = bio_is_passthrough(*bio_orig);

	bio_for_each_segment(from, *bio_orig, iter) {
		if (i++ < BIO_MAX_PAGES)
			sectors += from.bv_len >> 9;
		if (page_to_pfn(from.bv_page) > q->limits.bounce_pfn)
			bounce = true;
	}
	if (!bounce)
		return;

	if (!passthrough && sectors < bio_sectors(*bio_orig)) {
		bio = bio_split(*bio_orig, sectors, GFP_NOIO, bounce_bio_split);
		bio_chain(bio, *bio_orig);
		generic_make_request(*bio_orig);
		*bio_orig = bio;
	}
	bio = bio_clone_bioset(*bio_orig, GFP_NOIO, passthrough ? NULL :
			bounce_bio_set);

	bio_for_each_segment_all(to, bio, i) {
		struct page *page = to->bv_page;

		if (page_to_pfn(page) <= q->limits.bounce_pfn)
			continue;

		to->bv_page = mempool_alloc(pool, q->bounce_gfp);
		inc_zone_page_state(to->bv_page, NR_BOUNCE);

		if (rw == WRITE) {
			char *vto, *vfrom;

			flush_dcache_page(page);

			vto = page_address(to->bv_page) + to->bv_offset;
			vfrom = kmap_atomic(page) + to->bv_offset;
			memcpy(vto, vfrom, to->bv_len);
			kunmap_atomic(vfrom);
		}
	}

	trace_block_bio_bounce(q, *bio_orig);

	bio->bi_flags |= (1 << BIO_BOUNCED);

	if (pool == page_pool) {
		bio->bi_end_io = bounce_end_io_write;
		if (rw == READ)
			bio->bi_end_io = bounce_end_io_read;
	} else {
		bio->bi_end_io = bounce_end_io_write_isa;
		if (rw == READ)
			bio->bi_end_io = bounce_end_io_read_isa;
	}

	bio->bi_private = *bio_orig;
	*bio_orig = bio;
}

void blk_queue_bounce(struct request_queue *q, struct bio **bio_orig)
{
	mempool_t *pool;

	/*
	 * Data-less bio, nothing to bounce
	 */
	if (!bio_has_data(*bio_orig))
		return;

	/*
	 * for non-isa bounce case, just check if the bounce pfn is equal
	 * to or bigger than the highest pfn in the system -- in that case,
	 * don't waste time iterating over bio segments
	 */
	if (!(q->bounce_gfp & GFP_DMA)) {
		if (q->limits.bounce_pfn >= blk_max_pfn)
			return;
		pool = page_pool;
	} else {
		BUG_ON(!isa_page_pool);
		pool = isa_page_pool;
	}

	/*
	 * slow path
	 */
	__blk_queue_bounce(q, bio_orig, pool);
}
