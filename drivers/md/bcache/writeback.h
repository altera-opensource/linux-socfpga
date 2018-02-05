/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _BCACHE_WRITEBACK_H
#define _BCACHE_WRITEBACK_H

#define CUTOFF_WRITEBACK	40
#define CUTOFF_WRITEBACK_SYNC	70

#define MAX_WRITEBACKS_IN_PASS  5
#define MAX_WRITESIZE_IN_PASS   5000	/* *512b */

/*
 * 14 (16384ths) is chosen here as something that each backing device
 * should be a reasonable fraction of the share, and not to blow up
 * until individual backing devices are a petabyte.
 */
#define WRITEBACK_SHARE_SHIFT   14

static inline uint64_t bcache_dev_sectors_dirty(struct bcache_device *d)
{
	uint64_t i, ret = 0;

	for (i = 0; i < d->nr_stripes; i++)
		ret += atomic_read(d->stripe_sectors_dirty + i);

	return ret;
}

static inline uint64_t  bcache_flash_devs_sectors_dirty(struct cache_set *c)
{
	uint64_t i, ret = 0;

	mutex_lock(&bch_register_lock);

	for (i = 0; i < c->devices_max_used; i++) {
		struct bcache_device *d = c->devices[i];

		if (!d || !UUID_FLASH_ONLY(&c->uuids[i]))
			continue;
	   ret += bcache_dev_sectors_dirty(d);
	}

	mutex_unlock(&bch_register_lock);

	return ret;
}

static inline unsigned offset_to_stripe(struct bcache_device *d,
					uint64_t offset)
{
	do_div(offset, d->stripe_size);
	return offset;
}

static inline bool bcache_dev_stripe_dirty(struct cached_dev *dc,
					   uint64_t offset,
					   unsigned nr_sectors)
{
	unsigned stripe = offset_to_stripe(&dc->disk, offset);

	while (1) {
		if (atomic_read(dc->disk.stripe_sectors_dirty + stripe))
			return true;

		if (nr_sectors <= dc->disk.stripe_size)
			return false;

		nr_sectors -= dc->disk.stripe_size;
		stripe++;
	}
}

static inline bool should_writeback(struct cached_dev *dc, struct bio *bio,
				    unsigned cache_mode, bool would_skip)
{
	unsigned in_use = dc->disk.c->gc_stats.in_use;

	if (cache_mode != CACHE_MODE_WRITEBACK ||
	    test_bit(BCACHE_DEV_DETACHING, &dc->disk.flags) ||
	    in_use > CUTOFF_WRITEBACK_SYNC)
		return false;

	if (dc->partial_stripes_expensive &&
	    bcache_dev_stripe_dirty(dc, bio->bi_iter.bi_sector,
				    bio_sectors(bio)))
		return true;

	if (would_skip)
		return false;

	return (op_is_sync(bio->bi_opf) ||
		bio->bi_opf & (REQ_META|REQ_PRIO) ||
		in_use <= CUTOFF_WRITEBACK);
}

static inline void bch_writeback_queue(struct cached_dev *dc)
{
	if (!IS_ERR_OR_NULL(dc->writeback_thread))
		wake_up_process(dc->writeback_thread);
}

static inline void bch_writeback_add(struct cached_dev *dc)
{
	if (!atomic_read(&dc->has_dirty) &&
	    !atomic_xchg(&dc->has_dirty, 1)) {
		refcount_inc(&dc->count);

		if (BDEV_STATE(&dc->sb) != BDEV_STATE_DIRTY) {
			SET_BDEV_STATE(&dc->sb, BDEV_STATE_DIRTY);
			/* XXX: should do this synchronously */
			bch_write_bdev_super(dc, NULL);
		}

		bch_writeback_queue(dc);
	}
}

void bcache_dev_sectors_dirty_add(struct cache_set *, unsigned, uint64_t, int);

void bch_sectors_dirty_init(struct bcache_device *);
void bch_cached_dev_writeback_init(struct cached_dev *);
int bch_cached_dev_writeback_start(struct cached_dev *);

#endif
