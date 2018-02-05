// SPDX-License-Identifier: GPL-2.0
/*
 * background writeback - scan btree for dirty data and write it to the backing
 * device
 *
 * Copyright 2010, 2011 Kent Overstreet <kent.overstreet@gmail.com>
 * Copyright 2012 Google, Inc.
 */

#include "bcache.h"
#include "btree.h"
#include "debug.h"
#include "writeback.h"

#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/sched/clock.h>
#include <trace/events/bcache.h>

/* Rate limiting */
static uint64_t __calc_target_rate(struct cached_dev *dc)
{
	struct cache_set *c = dc->disk.c;

	/*
	 * This is the size of the cache, minus the amount used for
	 * flash-only devices
	 */
	uint64_t cache_sectors = c->nbuckets * c->sb.bucket_size -
				bcache_flash_devs_sectors_dirty(c);

	/*
	 * Unfortunately there is no control of global dirty data.  If the
	 * user states that they want 10% dirty data in the cache, and has,
	 * e.g., 5 backing volumes of equal size, we try and ensure each
	 * backing volume uses about 2% of the cache for dirty data.
	 */
	uint32_t bdev_share =
		div64_u64(bdev_sectors(dc->bdev) << WRITEBACK_SHARE_SHIFT,
				c->cached_dev_sectors);

	uint64_t cache_dirty_target =
		div_u64(cache_sectors * dc->writeback_percent, 100);

	/* Ensure each backing dev gets at least one dirty share */
	if (bdev_share < 1)
		bdev_share = 1;

	return (cache_dirty_target * bdev_share) >> WRITEBACK_SHARE_SHIFT;
}

static void __update_writeback_rate(struct cached_dev *dc)
{
	/*
	 * PI controller:
	 * Figures out the amount that should be written per second.
	 *
	 * First, the error (number of sectors that are dirty beyond our
	 * target) is calculated.  The error is accumulated (numerically
	 * integrated).
	 *
	 * Then, the proportional value and integral value are scaled
	 * based on configured values.  These are stored as inverses to
	 * avoid fixed point math and to make configuration easy-- e.g.
	 * the default value of 40 for writeback_rate_p_term_inverse
	 * attempts to write at a rate that would retire all the dirty
	 * blocks in 40 seconds.
	 *
	 * The writeback_rate_i_inverse value of 10000 means that 1/10000th
	 * of the error is accumulated in the integral term per second.
	 * This acts as a slow, long-term average that is not subject to
	 * variations in usage like the p term.
	 */
	int64_t target = __calc_target_rate(dc);
	int64_t dirty = bcache_dev_sectors_dirty(&dc->disk);
	int64_t error = dirty - target;
	int64_t proportional_scaled =
		div_s64(error, dc->writeback_rate_p_term_inverse);
	int64_t integral_scaled;
	uint32_t new_rate;

	if ((error < 0 && dc->writeback_rate_integral > 0) ||
	    (error > 0 && time_before64(local_clock(),
			 dc->writeback_rate.next + NSEC_PER_MSEC))) {
		/*
		 * Only decrease the integral term if it's more than
		 * zero.  Only increase the integral term if the device
		 * is keeping up.  (Don't wind up the integral
		 * ineffectively in either case).
		 *
		 * It's necessary to scale this by
		 * writeback_rate_update_seconds to keep the integral
		 * term dimensioned properly.
		 */
		dc->writeback_rate_integral += error *
			dc->writeback_rate_update_seconds;
	}

	integral_scaled = div_s64(dc->writeback_rate_integral,
			dc->writeback_rate_i_term_inverse);

	new_rate = clamp_t(int32_t, (proportional_scaled + integral_scaled),
			dc->writeback_rate_minimum, NSEC_PER_SEC);

	dc->writeback_rate_proportional = proportional_scaled;
	dc->writeback_rate_integral_scaled = integral_scaled;
	dc->writeback_rate_change = new_rate - dc->writeback_rate.rate;
	dc->writeback_rate.rate = new_rate;
	dc->writeback_rate_target = target;
}

static void update_writeback_rate(struct work_struct *work)
{
	struct cached_dev *dc = container_of(to_delayed_work(work),
					     struct cached_dev,
					     writeback_rate_update);

	down_read(&dc->writeback_lock);

	if (atomic_read(&dc->has_dirty) &&
	    dc->writeback_percent)
		__update_writeback_rate(dc);

	up_read(&dc->writeback_lock);

	schedule_delayed_work(&dc->writeback_rate_update,
			      dc->writeback_rate_update_seconds * HZ);
}

static unsigned writeback_delay(struct cached_dev *dc, unsigned sectors)
{
	if (test_bit(BCACHE_DEV_DETACHING, &dc->disk.flags) ||
	    !dc->writeback_percent)
		return 0;

	return bch_next_delay(&dc->writeback_rate, sectors);
}

struct dirty_io {
	struct closure		cl;
	struct cached_dev	*dc;
	uint16_t		sequence;
	struct bio		bio;
};

static void dirty_init(struct keybuf_key *w)
{
	struct dirty_io *io = w->private;
	struct bio *bio = &io->bio;

	bio_init(bio, bio->bi_inline_vecs,
		 DIV_ROUND_UP(KEY_SIZE(&w->key), PAGE_SECTORS));
	if (!io->dc->writeback_percent)
		bio_set_prio(bio, IOPRIO_PRIO_VALUE(IOPRIO_CLASS_IDLE, 0));

	bio->bi_iter.bi_size	= KEY_SIZE(&w->key) << 9;
	bio->bi_private		= w;
	bch_bio_map(bio, NULL);
}

static void dirty_io_destructor(struct closure *cl)
{
	struct dirty_io *io = container_of(cl, struct dirty_io, cl);
	kfree(io);
}

static void write_dirty_finish(struct closure *cl)
{
	struct dirty_io *io = container_of(cl, struct dirty_io, cl);
	struct keybuf_key *w = io->bio.bi_private;
	struct cached_dev *dc = io->dc;

	bio_free_pages(&io->bio);

	/* This is kind of a dumb way of signalling errors. */
	if (KEY_DIRTY(&w->key)) {
		int ret;
		unsigned i;
		struct keylist keys;

		bch_keylist_init(&keys);

		bkey_copy(keys.top, &w->key);
		SET_KEY_DIRTY(keys.top, false);
		bch_keylist_push(&keys);

		for (i = 0; i < KEY_PTRS(&w->key); i++)
			atomic_inc(&PTR_BUCKET(dc->disk.c, &w->key, i)->pin);

		ret = bch_btree_insert(dc->disk.c, &keys, NULL, &w->key);

		if (ret)
			trace_bcache_writeback_collision(&w->key);

		atomic_long_inc(ret
				? &dc->disk.c->writeback_keys_failed
				: &dc->disk.c->writeback_keys_done);
	}

	bch_keybuf_del(&dc->writeback_keys, w);
	up(&dc->in_flight);

	closure_return_with_destructor(cl, dirty_io_destructor);
}

static void dirty_endio(struct bio *bio)
{
	struct keybuf_key *w = bio->bi_private;
	struct dirty_io *io = w->private;

	if (bio->bi_status)
		SET_KEY_DIRTY(&w->key, false);

	closure_put(&io->cl);
}

static void write_dirty(struct closure *cl)
{
	struct dirty_io *io = container_of(cl, struct dirty_io, cl);
	struct keybuf_key *w = io->bio.bi_private;
	struct cached_dev *dc = io->dc;

	uint16_t next_sequence;

	if (atomic_read(&dc->writeback_sequence_next) != io->sequence) {
		/* Not our turn to write; wait for a write to complete */
		closure_wait(&dc->writeback_ordering_wait, cl);

		if (atomic_read(&dc->writeback_sequence_next) == io->sequence) {
			/*
			 * Edge case-- it happened in indeterminate order
			 * relative to when we were added to wait list..
			 */
			closure_wake_up(&dc->writeback_ordering_wait);
		}

		continue_at(cl, write_dirty, io->dc->writeback_write_wq);
		return;
	}

	next_sequence = io->sequence + 1;

	/*
	 * IO errors are signalled using the dirty bit on the key.
	 * If we failed to read, we should not attempt to write to the
	 * backing device.  Instead, immediately go to write_dirty_finish
	 * to clean up.
	 */
	if (KEY_DIRTY(&w->key)) {
		dirty_init(w);
		bio_set_op_attrs(&io->bio, REQ_OP_WRITE, 0);
		io->bio.bi_iter.bi_sector = KEY_START(&w->key);
		bio_set_dev(&io->bio, io->dc->bdev);
		io->bio.bi_end_io	= dirty_endio;

		closure_bio_submit(&io->bio, cl);
	}

	atomic_set(&dc->writeback_sequence_next, next_sequence);
	closure_wake_up(&dc->writeback_ordering_wait);

	continue_at(cl, write_dirty_finish, io->dc->writeback_write_wq);
}

static void read_dirty_endio(struct bio *bio)
{
	struct keybuf_key *w = bio->bi_private;
	struct dirty_io *io = w->private;

	/* is_read = 1 */
	bch_count_io_errors(PTR_CACHE(io->dc->disk.c, &w->key, 0),
			    bio->bi_status, 1,
			    "reading dirty data from cache");

	dirty_endio(bio);
}

static void read_dirty_submit(struct closure *cl)
{
	struct dirty_io *io = container_of(cl, struct dirty_io, cl);

	closure_bio_submit(&io->bio, cl);

	continue_at(cl, write_dirty, io->dc->writeback_write_wq);
}

static void read_dirty(struct cached_dev *dc)
{
	unsigned delay = 0;
	struct keybuf_key *next, *keys[MAX_WRITEBACKS_IN_PASS], *w;
	size_t size;
	int nk, i;
	struct dirty_io *io;
	struct closure cl;
	uint16_t sequence = 0;

	BUG_ON(!llist_empty(&dc->writeback_ordering_wait.list));
	atomic_set(&dc->writeback_sequence_next, sequence);
	closure_init_stack(&cl);

	/*
	 * XXX: if we error, background writeback just spins. Should use some
	 * mempools.
	 */

	next = bch_keybuf_next(&dc->writeback_keys);

	while (!kthread_should_stop() && next) {
		size = 0;
		nk = 0;

		do {
			BUG_ON(ptr_stale(dc->disk.c, &next->key, 0));

			/*
			 * Don't combine too many operations, even if they
			 * are all small.
			 */
			if (nk >= MAX_WRITEBACKS_IN_PASS)
				break;

			/*
			 * If the current operation is very large, don't
			 * further combine operations.
			 */
			if (size >= MAX_WRITESIZE_IN_PASS)
				break;

			/*
			 * Operations are only eligible to be combined
			 * if they are contiguous.
			 *
			 * TODO: add a heuristic willing to fire a
			 * certain amount of non-contiguous IO per pass,
			 * so that we can benefit from backing device
			 * command queueing.
			 */
			if ((nk != 0) && bkey_cmp(&keys[nk-1]->key,
						&START_KEY(&next->key)))
				break;

			size += KEY_SIZE(&next->key);
			keys[nk++] = next;
		} while ((next = bch_keybuf_next(&dc->writeback_keys)));

		/* Now we have gathered a set of 1..5 keys to write back. */
		for (i = 0; i < nk; i++) {
			w = keys[i];

			io = kzalloc(sizeof(struct dirty_io) +
				     sizeof(struct bio_vec) *
				     DIV_ROUND_UP(KEY_SIZE(&w->key), PAGE_SECTORS),
				     GFP_KERNEL);
			if (!io)
				goto err;

			w->private	= io;
			io->dc		= dc;
			io->sequence    = sequence++;

			dirty_init(w);
			bio_set_op_attrs(&io->bio, REQ_OP_READ, 0);
			io->bio.bi_iter.bi_sector = PTR_OFFSET(&w->key, 0);
			bio_set_dev(&io->bio,
				    PTR_CACHE(dc->disk.c, &w->key, 0)->bdev);
			io->bio.bi_end_io	= read_dirty_endio;

			if (bch_bio_alloc_pages(&io->bio, GFP_KERNEL))
				goto err_free;

			trace_bcache_writeback(&w->key);

			down(&dc->in_flight);

			/* We've acquired a semaphore for the maximum
			 * simultaneous number of writebacks; from here
			 * everything happens asynchronously.
			 */
			closure_call(&io->cl, read_dirty_submit, NULL, &cl);
		}

		delay = writeback_delay(dc, size);

		/* If the control system would wait for at least half a
		 * second, and there's been no reqs hitting the backing disk
		 * for awhile: use an alternate mode where we have at most
		 * one contiguous set of writebacks in flight at a time.  If
		 * someone wants to do IO it will be quick, as it will only
		 * have to contend with one operation in flight, and we'll
		 * be round-tripping data to the backing disk as quickly as
		 * it can accept it.
		 */
		if (delay >= HZ / 2) {
			/* 3 means at least 1.5 seconds, up to 7.5 if we
			 * have slowed way down.
			 */
			if (atomic_inc_return(&dc->backing_idle) >= 3) {
				/* Wait for current I/Os to finish */
				closure_sync(&cl);
				/* And immediately launch a new set. */
				delay = 0;
			}
		}

		while (!kthread_should_stop() && delay) {
			schedule_timeout_interruptible(delay);
			delay = writeback_delay(dc, 0);
		}
	}

	if (0) {
err_free:
		kfree(w->private);
err:
		bch_keybuf_del(&dc->writeback_keys, w);
	}

	/*
	 * Wait for outstanding writeback IOs to finish (and keybuf slots to be
	 * freed) before refilling again
	 */
	closure_sync(&cl);
}

/* Scan for dirty data */

void bcache_dev_sectors_dirty_add(struct cache_set *c, unsigned inode,
				  uint64_t offset, int nr_sectors)
{
	struct bcache_device *d = c->devices[inode];
	unsigned stripe_offset, stripe, sectors_dirty;

	if (!d)
		return;

	stripe = offset_to_stripe(d, offset);
	stripe_offset = offset & (d->stripe_size - 1);

	while (nr_sectors) {
		int s = min_t(unsigned, abs(nr_sectors),
			      d->stripe_size - stripe_offset);

		if (nr_sectors < 0)
			s = -s;

		if (stripe >= d->nr_stripes)
			return;

		sectors_dirty = atomic_add_return(s,
					d->stripe_sectors_dirty + stripe);
		if (sectors_dirty == d->stripe_size)
			set_bit(stripe, d->full_dirty_stripes);
		else
			clear_bit(stripe, d->full_dirty_stripes);

		nr_sectors -= s;
		stripe_offset = 0;
		stripe++;
	}
}

static bool dirty_pred(struct keybuf *buf, struct bkey *k)
{
	struct cached_dev *dc = container_of(buf, struct cached_dev, writeback_keys);

	BUG_ON(KEY_INODE(k) != dc->disk.id);

	return KEY_DIRTY(k);
}

static void refill_full_stripes(struct cached_dev *dc)
{
	struct keybuf *buf = &dc->writeback_keys;
	unsigned start_stripe, stripe, next_stripe;
	bool wrapped = false;

	stripe = offset_to_stripe(&dc->disk, KEY_OFFSET(&buf->last_scanned));

	if (stripe >= dc->disk.nr_stripes)
		stripe = 0;

	start_stripe = stripe;

	while (1) {
		stripe = find_next_bit(dc->disk.full_dirty_stripes,
				       dc->disk.nr_stripes, stripe);

		if (stripe == dc->disk.nr_stripes)
			goto next;

		next_stripe = find_next_zero_bit(dc->disk.full_dirty_stripes,
						 dc->disk.nr_stripes, stripe);

		buf->last_scanned = KEY(dc->disk.id,
					stripe * dc->disk.stripe_size, 0);

		bch_refill_keybuf(dc->disk.c, buf,
				  &KEY(dc->disk.id,
				       next_stripe * dc->disk.stripe_size, 0),
				  dirty_pred);

		if (array_freelist_empty(&buf->freelist))
			return;

		stripe = next_stripe;
next:
		if (wrapped && stripe > start_stripe)
			return;

		if (stripe == dc->disk.nr_stripes) {
			stripe = 0;
			wrapped = true;
		}
	}
}

/*
 * Returns true if we scanned the entire disk
 */
static bool refill_dirty(struct cached_dev *dc)
{
	struct keybuf *buf = &dc->writeback_keys;
	struct bkey start = KEY(dc->disk.id, 0, 0);
	struct bkey end = KEY(dc->disk.id, MAX_KEY_OFFSET, 0);
	struct bkey start_pos;

	/*
	 * make sure keybuf pos is inside the range for this disk - at bringup
	 * we might not be attached yet so this disk's inode nr isn't
	 * initialized then
	 */
	if (bkey_cmp(&buf->last_scanned, &start) < 0 ||
	    bkey_cmp(&buf->last_scanned, &end) > 0)
		buf->last_scanned = start;

	if (dc->partial_stripes_expensive) {
		refill_full_stripes(dc);
		if (array_freelist_empty(&buf->freelist))
			return false;
	}

	start_pos = buf->last_scanned;
	bch_refill_keybuf(dc->disk.c, buf, &end, dirty_pred);

	if (bkey_cmp(&buf->last_scanned, &end) < 0)
		return false;

	/*
	 * If we get to the end start scanning again from the beginning, and
	 * only scan up to where we initially started scanning from:
	 */
	buf->last_scanned = start;
	bch_refill_keybuf(dc->disk.c, buf, &start_pos, dirty_pred);

	return bkey_cmp(&buf->last_scanned, &start_pos) >= 0;
}

static int bch_writeback_thread(void *arg)
{
	struct cached_dev *dc = arg;
	bool searched_full_index;

	bch_ratelimit_reset(&dc->writeback_rate);

	while (!kthread_should_stop()) {
		down_write(&dc->writeback_lock);
		if (!atomic_read(&dc->has_dirty) ||
		    (!test_bit(BCACHE_DEV_DETACHING, &dc->disk.flags) &&
		     !dc->writeback_running)) {
			up_write(&dc->writeback_lock);
			set_current_state(TASK_INTERRUPTIBLE);

			if (kthread_should_stop())
				return 0;

			schedule();
			continue;
		}

		searched_full_index = refill_dirty(dc);

		if (searched_full_index &&
		    RB_EMPTY_ROOT(&dc->writeback_keys.keys)) {
			atomic_set(&dc->has_dirty, 0);
			cached_dev_put(dc);
			SET_BDEV_STATE(&dc->sb, BDEV_STATE_CLEAN);
			bch_write_bdev_super(dc, NULL);
		}

		up_write(&dc->writeback_lock);

		read_dirty(dc);

		if (searched_full_index) {
			unsigned delay = dc->writeback_delay * HZ;

			while (delay &&
			       !kthread_should_stop() &&
			       !test_bit(BCACHE_DEV_DETACHING, &dc->disk.flags))
				delay = schedule_timeout_interruptible(delay);

			bch_ratelimit_reset(&dc->writeback_rate);
		}
	}

	return 0;
}

/* Init */

struct sectors_dirty_init {
	struct btree_op	op;
	unsigned	inode;
};

static int sectors_dirty_init_fn(struct btree_op *_op, struct btree *b,
				 struct bkey *k)
{
	struct sectors_dirty_init *op = container_of(_op,
						struct sectors_dirty_init, op);
	if (KEY_INODE(k) > op->inode)
		return MAP_DONE;

	if (KEY_DIRTY(k))
		bcache_dev_sectors_dirty_add(b->c, KEY_INODE(k),
					     KEY_START(k), KEY_SIZE(k));

	return MAP_CONTINUE;
}

void bch_sectors_dirty_init(struct bcache_device *d)
{
	struct sectors_dirty_init op;

	bch_btree_op_init(&op.op, -1);
	op.inode = d->id;

	bch_btree_map_keys(&op.op, d->c, &KEY(op.inode, 0, 0),
			   sectors_dirty_init_fn, 0);
}

void bch_cached_dev_writeback_init(struct cached_dev *dc)
{
	sema_init(&dc->in_flight, 64);
	init_rwsem(&dc->writeback_lock);
	bch_keybuf_init(&dc->writeback_keys);

	dc->writeback_metadata		= true;
	dc->writeback_running		= true;
	dc->writeback_percent		= 10;
	dc->writeback_delay		= 30;
	dc->writeback_rate.rate		= 1024;
	dc->writeback_rate_minimum	= 8;

	dc->writeback_rate_update_seconds = 5;
	dc->writeback_rate_p_term_inverse = 40;
	dc->writeback_rate_i_term_inverse = 10000;

	INIT_DELAYED_WORK(&dc->writeback_rate_update, update_writeback_rate);
}

int bch_cached_dev_writeback_start(struct cached_dev *dc)
{
	dc->writeback_write_wq = alloc_workqueue("bcache_writeback_wq",
						WQ_MEM_RECLAIM, 0);
	if (!dc->writeback_write_wq)
		return -ENOMEM;

	dc->writeback_thread = kthread_create(bch_writeback_thread, dc,
					      "bcache_writeback");
	if (IS_ERR(dc->writeback_thread))
		return PTR_ERR(dc->writeback_thread);

	schedule_delayed_work(&dc->writeback_rate_update,
			      dc->writeback_rate_update_seconds * HZ);

	bch_writeback_queue(dc);

	return 0;
}
