/*
 * Copyright (C) 2001 Momchil Velikov
 * Portions Copyright (C) 2001 Christoph Hellwig
 * Copyright (C) 2006 Nick Piggin
 * Copyright (C) 2012 Konstantin Khlebnikov
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2, or (at
 * your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#ifndef _LINUX_RADIX_TREE_H
#define _LINUX_RADIX_TREE_H

#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/preempt.h>
#include <linux/rcupdate.h>
#include <linux/spinlock.h>
#include <linux/types.h>

/*
 * The bottom two bits of the slot determine how the remaining bits in the
 * slot are interpreted:
 *
 * 00 - data pointer
 * 01 - internal entry
 * 10 - exceptional entry
 * 11 - this bit combination is currently unused/reserved
 *
 * The internal entry may be a pointer to the next level in the tree, a
 * sibling entry, or an indicator that the entry in this slot has been moved
 * to another location in the tree and the lookup should be restarted.  While
 * NULL fits the 'data pointer' pattern, it means that there is no entry in
 * the tree for this index (no matter what level of the tree it is found at).
 * This means that you cannot store NULL in the tree as a value for the index.
 */
#define RADIX_TREE_ENTRY_MASK		3UL
#define RADIX_TREE_INTERNAL_NODE	1UL

/*
 * Most users of the radix tree store pointers but shmem/tmpfs stores swap
 * entries in the same tree.  They are marked as exceptional entries to
 * distinguish them from pointers to struct page.
 * EXCEPTIONAL_ENTRY tests the bit, EXCEPTIONAL_SHIFT shifts content past it.
 */
#define RADIX_TREE_EXCEPTIONAL_ENTRY	2
#define RADIX_TREE_EXCEPTIONAL_SHIFT	2

static inline bool radix_tree_is_internal_node(void *ptr)
{
	return ((unsigned long)ptr & RADIX_TREE_ENTRY_MASK) ==
				RADIX_TREE_INTERNAL_NODE;
}

/*** radix-tree API starts here ***/

#define RADIX_TREE_MAX_TAGS 3

#ifndef RADIX_TREE_MAP_SHIFT
#define RADIX_TREE_MAP_SHIFT	(CONFIG_BASE_SMALL ? 4 : 6)
#endif

#define RADIX_TREE_MAP_SIZE	(1UL << RADIX_TREE_MAP_SHIFT)
#define RADIX_TREE_MAP_MASK	(RADIX_TREE_MAP_SIZE-1)

#define RADIX_TREE_TAG_LONGS	\
	((RADIX_TREE_MAP_SIZE + BITS_PER_LONG - 1) / BITS_PER_LONG)

#define RADIX_TREE_INDEX_BITS  (8 /* CHAR_BIT */ * sizeof(unsigned long))
#define RADIX_TREE_MAX_PATH (DIV_ROUND_UP(RADIX_TREE_INDEX_BITS, \
					  RADIX_TREE_MAP_SHIFT))

/*
 * @count is the count of every non-NULL element in the ->slots array
 * whether that is an exceptional entry, a retry entry, a user pointer,
 * a sibling entry or a pointer to the next level of the tree.
 * @exceptional is the count of every element in ->slots which is
 * either radix_tree_exceptional_entry() or is a sibling entry for an
 * exceptional entry.
 */
struct radix_tree_node {
	unsigned char	shift;		/* Bits remaining in each slot */
	unsigned char	offset;		/* Slot offset in parent */
	unsigned char	count;		/* Total entry count */
	unsigned char	exceptional;	/* Exceptional entry count */
	struct radix_tree_node *parent;		/* Used when ascending tree */
	struct radix_tree_root *root;		/* The tree we belong to */
	union {
		struct list_head private_list;	/* For tree user */
		struct rcu_head	rcu_head;	/* Used when freeing node */
	};
	void __rcu	*slots[RADIX_TREE_MAP_SIZE];
	unsigned long	tags[RADIX_TREE_MAX_TAGS][RADIX_TREE_TAG_LONGS];
};

/* The top bits of gfp_mask are used to store the root tags and the IDR flag */
#define ROOT_IS_IDR	((__force gfp_t)(1 << __GFP_BITS_SHIFT))
#define ROOT_TAG_SHIFT	(__GFP_BITS_SHIFT + 1)

struct radix_tree_root {
	gfp_t			gfp_mask;
	struct radix_tree_node	__rcu *rnode;
};

#define RADIX_TREE_INIT(mask)	{					\
	.gfp_mask = (mask),						\
	.rnode = NULL,							\
}

#define RADIX_TREE(name, mask) \
	struct radix_tree_root name = RADIX_TREE_INIT(mask)

#define INIT_RADIX_TREE(root, mask)					\
do {									\
	(root)->gfp_mask = (mask);					\
	(root)->rnode = NULL;						\
} while (0)

static inline bool radix_tree_empty(const struct radix_tree_root *root)
{
	return root->rnode == NULL;
}

/**
 * struct radix_tree_iter - radix tree iterator state
 *
 * @index:	index of current slot
 * @next_index:	one beyond the last index for this chunk
 * @tags:	bit-mask for tag-iterating
 * @node:	node that contains current slot
 * @shift:	shift for the node that holds our slots
 *
 * This radix tree iterator works in terms of "chunks" of slots.  A chunk is a
 * subinterval of slots contained within one radix tree leaf node.  It is
 * described by a pointer to its first slot and a struct radix_tree_iter
 * which holds the chunk's position in the tree and its size.  For tagged
 * iteration radix_tree_iter also holds the slots' bit-mask for one chosen
 * radix tree tag.
 */
struct radix_tree_iter {
	unsigned long	index;
	unsigned long	next_index;
	unsigned long	tags;
	struct radix_tree_node *node;
#ifdef CONFIG_RADIX_TREE_MULTIORDER
	unsigned int	shift;
#endif
};

static inline unsigned int iter_shift(const struct radix_tree_iter *iter)
{
#ifdef CONFIG_RADIX_TREE_MULTIORDER
	return iter->shift;
#else
	return 0;
#endif
}

/**
 * Radix-tree synchronization
 *
 * The radix-tree API requires that users provide all synchronisation (with
 * specific exceptions, noted below).
 *
 * Synchronization of access to the data items being stored in the tree, and
 * management of their lifetimes must be completely managed by API users.
 *
 * For API usage, in general,
 * - any function _modifying_ the tree or tags (inserting or deleting
 *   items, setting or clearing tags) must exclude other modifications, and
 *   exclude any functions reading the tree.
 * - any function _reading_ the tree or tags (looking up items or tags,
 *   gang lookups) must exclude modifications to the tree, but may occur
 *   concurrently with other readers.
 *
 * The notable exceptions to this rule are the following functions:
 * __radix_tree_lookup
 * radix_tree_lookup
 * radix_tree_lookup_slot
 * radix_tree_tag_get
 * radix_tree_gang_lookup
 * radix_tree_gang_lookup_slot
 * radix_tree_gang_lookup_tag
 * radix_tree_gang_lookup_tag_slot
 * radix_tree_tagged
 *
 * The first 8 functions are able to be called locklessly, using RCU. The
 * caller must ensure calls to these functions are made within rcu_read_lock()
 * regions. Other readers (lock-free or otherwise) and modifications may be
 * running concurrently.
 *
 * It is still required that the caller manage the synchronization and lifetimes
 * of the items. So if RCU lock-free lookups are used, typically this would mean
 * that the items have their own locks, or are amenable to lock-free access; and
 * that the items are freed by RCU (or only freed after having been deleted from
 * the radix tree *and* a synchronize_rcu() grace period).
 *
 * (Note, rcu_assign_pointer and rcu_dereference are not needed to control
 * access to data items when inserting into or looking up from the radix tree)
 *
 * Note that the value returned by radix_tree_tag_get() may not be relied upon
 * if only the RCU read lock is held.  Functions to set/clear tags and to
 * delete nodes running concurrently with it may affect its result such that
 * two consecutive reads in the same locked section may return different
 * values.  If reliability is required, modification functions must also be
 * excluded from concurrency.
 *
 * radix_tree_tagged is able to be called without locking or RCU.
 */

/**
 * radix_tree_deref_slot - dereference a slot
 * @slot: slot pointer, returned by radix_tree_lookup_slot
 *
 * For use with radix_tree_lookup_slot().  Caller must hold tree at least read
 * locked across slot lookup and dereference. Not required if write lock is
 * held (ie. items cannot be concurrently inserted).
 *
 * radix_tree_deref_retry must be used to confirm validity of the pointer if
 * only the read lock is held.
 *
 * Return: entry stored in that slot.
 */
static inline void *radix_tree_deref_slot(void __rcu **slot)
{
	return rcu_dereference(*slot);
}

/**
 * radix_tree_deref_slot_protected - dereference a slot with tree lock held
 * @slot: slot pointer, returned by radix_tree_lookup_slot
 *
 * Similar to radix_tree_deref_slot.  The caller does not hold the RCU read
 * lock but it must hold the tree lock to prevent parallel updates.
 *
 * Return: entry stored in that slot.
 */
static inline void *radix_tree_deref_slot_protected(void __rcu **slot,
							spinlock_t *treelock)
{
	return rcu_dereference_protected(*slot, lockdep_is_held(treelock));
}

/**
 * radix_tree_deref_retry	- check radix_tree_deref_slot
 * @arg:	pointer returned by radix_tree_deref_slot
 * Returns:	0 if retry is not required, otherwise retry is required
 *
 * radix_tree_deref_retry must be used with radix_tree_deref_slot.
 */
static inline int radix_tree_deref_retry(void *arg)
{
	return unlikely(radix_tree_is_internal_node(arg));
}

/**
 * radix_tree_exceptional_entry	- radix_tree_deref_slot gave exceptional entry?
 * @arg:	value returned by radix_tree_deref_slot
 * Returns:	0 if well-aligned pointer, non-0 if exceptional entry.
 */
static inline int radix_tree_exceptional_entry(void *arg)
{
	/* Not unlikely because radix_tree_exception often tested first */
	return (unsigned long)arg & RADIX_TREE_EXCEPTIONAL_ENTRY;
}

/**
 * radix_tree_exception	- radix_tree_deref_slot returned either exception?
 * @arg:	value returned by radix_tree_deref_slot
 * Returns:	0 if well-aligned pointer, non-0 if either kind of exception.
 */
static inline int radix_tree_exception(void *arg)
{
	return unlikely((unsigned long)arg & RADIX_TREE_ENTRY_MASK);
}

int __radix_tree_create(struct radix_tree_root *, unsigned long index,
			unsigned order, struct radix_tree_node **nodep,
			void __rcu ***slotp);
int __radix_tree_insert(struct radix_tree_root *, unsigned long index,
			unsigned order, void *);
static inline int radix_tree_insert(struct radix_tree_root *root,
			unsigned long index, void *entry)
{
	return __radix_tree_insert(root, index, 0, entry);
}
void *__radix_tree_lookup(const struct radix_tree_root *, unsigned long index,
			  struct radix_tree_node **nodep, void __rcu ***slotp);
void *radix_tree_lookup(const struct radix_tree_root *, unsigned long);
void __rcu **radix_tree_lookup_slot(const struct radix_tree_root *,
					unsigned long index);
typedef void (*radix_tree_update_node_t)(struct radix_tree_node *);
void __radix_tree_replace(struct radix_tree_root *, struct radix_tree_node *,
			  void __rcu **slot, void *entry,
			  radix_tree_update_node_t update_node);
void radix_tree_iter_replace(struct radix_tree_root *,
		const struct radix_tree_iter *, void __rcu **slot, void *entry);
void radix_tree_replace_slot(struct radix_tree_root *,
			     void __rcu **slot, void *entry);
void __radix_tree_delete_node(struct radix_tree_root *,
			      struct radix_tree_node *,
			      radix_tree_update_node_t update_node);
void radix_tree_iter_delete(struct radix_tree_root *,
			struct radix_tree_iter *iter, void __rcu **slot);
void *radix_tree_delete_item(struct radix_tree_root *, unsigned long, void *);
void *radix_tree_delete(struct radix_tree_root *, unsigned long);
void radix_tree_clear_tags(struct radix_tree_root *, struct radix_tree_node *,
			   void __rcu **slot);
unsigned int radix_tree_gang_lookup(const struct radix_tree_root *,
			void **results, unsigned long first_index,
			unsigned int max_items);
unsigned int radix_tree_gang_lookup_slot(const struct radix_tree_root *,
			void __rcu ***results, unsigned long *indices,
			unsigned long first_index, unsigned int max_items);
int radix_tree_preload(gfp_t gfp_mask);
int radix_tree_maybe_preload(gfp_t gfp_mask);
int radix_tree_maybe_preload_order(gfp_t gfp_mask, int order);
void radix_tree_init(void);
void *radix_tree_tag_set(struct radix_tree_root *,
			unsigned long index, unsigned int tag);
void *radix_tree_tag_clear(struct radix_tree_root *,
			unsigned long index, unsigned int tag);
int radix_tree_tag_get(const struct radix_tree_root *,
			unsigned long index, unsigned int tag);
void radix_tree_iter_tag_set(struct radix_tree_root *,
		const struct radix_tree_iter *iter, unsigned int tag);
void radix_tree_iter_tag_clear(struct radix_tree_root *,
		const struct radix_tree_iter *iter, unsigned int tag);
unsigned int radix_tree_gang_lookup_tag(const struct radix_tree_root *,
		void **results, unsigned long first_index,
		unsigned int max_items, unsigned int tag);
unsigned int radix_tree_gang_lookup_tag_slot(const struct radix_tree_root *,
		void __rcu ***results, unsigned long first_index,
		unsigned int max_items, unsigned int tag);
int radix_tree_tagged(const struct radix_tree_root *, unsigned int tag);

static inline void radix_tree_preload_end(void)
{
	preempt_enable();
}

int radix_tree_split_preload(unsigned old_order, unsigned new_order, gfp_t);
int radix_tree_split(struct radix_tree_root *, unsigned long index,
			unsigned new_order);
int radix_tree_join(struct radix_tree_root *, unsigned long index,
			unsigned new_order, void *);

void __rcu **idr_get_free_cmn(struct radix_tree_root *root,
			      struct radix_tree_iter *iter, gfp_t gfp,
			      unsigned long max);
static inline void __rcu **idr_get_free(struct radix_tree_root *root,
					struct radix_tree_iter *iter,
					gfp_t gfp,
					int end)
{
	return idr_get_free_cmn(root, iter, gfp, end > 0 ? end - 1 : INT_MAX);
}

static inline void __rcu **idr_get_free_ext(struct radix_tree_root *root,
					    struct radix_tree_iter *iter,
					    gfp_t gfp,
					    unsigned long end)
{
	return idr_get_free_cmn(root, iter, gfp, end - 1);
}

enum {
	RADIX_TREE_ITER_TAG_MASK = 0x0f,	/* tag index in lower nybble */
	RADIX_TREE_ITER_TAGGED   = 0x10,	/* lookup tagged slots */
	RADIX_TREE_ITER_CONTIG   = 0x20,	/* stop at first hole */
};

/**
 * radix_tree_iter_init - initialize radix tree iterator
 *
 * @iter:	pointer to iterator state
 * @start:	iteration starting index
 * Returns:	NULL
 */
static __always_inline void __rcu **
radix_tree_iter_init(struct radix_tree_iter *iter, unsigned long start)
{
	/*
	 * Leave iter->tags uninitialized. radix_tree_next_chunk() will fill it
	 * in the case of a successful tagged chunk lookup.  If the lookup was
	 * unsuccessful or non-tagged then nobody cares about ->tags.
	 *
	 * Set index to zero to bypass next_index overflow protection.
	 * See the comment in radix_tree_next_chunk() for details.
	 */
	iter->index = 0;
	iter->next_index = start;
	return NULL;
}

/**
 * radix_tree_next_chunk - find next chunk of slots for iteration
 *
 * @root:	radix tree root
 * @iter:	iterator state
 * @flags:	RADIX_TREE_ITER_* flags and tag index
 * Returns:	pointer to chunk first slot, or NULL if there no more left
 *
 * This function looks up the next chunk in the radix tree starting from
 * @iter->next_index.  It returns a pointer to the chunk's first slot.
 * Also it fills @iter with data about chunk: position in the tree (index),
 * its end (next_index), and constructs a bit mask for tagged iterating (tags).
 */
void __rcu **radix_tree_next_chunk(const struct radix_tree_root *,
			     struct radix_tree_iter *iter, unsigned flags);

/**
 * radix_tree_iter_lookup - look up an index in the radix tree
 * @root: radix tree root
 * @iter: iterator state
 * @index: key to look up
 *
 * If @index is present in the radix tree, this function returns the slot
 * containing it and updates @iter to describe the entry.  If @index is not
 * present, it returns NULL.
 */
static inline void __rcu **
radix_tree_iter_lookup(const struct radix_tree_root *root,
			struct radix_tree_iter *iter, unsigned long index)
{
	radix_tree_iter_init(iter, index);
	return radix_tree_next_chunk(root, iter, RADIX_TREE_ITER_CONTIG);
}

/**
 * radix_tree_iter_find - find a present entry
 * @root: radix tree root
 * @iter: iterator state
 * @index: start location
 *
 * This function returns the slot containing the entry with the lowest index
 * which is at least @index.  If @index is larger than any present entry, this
 * function returns NULL.  The @iter is updated to describe the entry found.
 */
static inline void __rcu **
radix_tree_iter_find(const struct radix_tree_root *root,
			struct radix_tree_iter *iter, unsigned long index)
{
	radix_tree_iter_init(iter, index);
	return radix_tree_next_chunk(root, iter, 0);
}

/**
 * radix_tree_iter_retry - retry this chunk of the iteration
 * @iter:	iterator state
 *
 * If we iterate over a tree protected only by the RCU lock, a race
 * against deletion or creation may result in seeing a slot for which
 * radix_tree_deref_retry() returns true.  If so, call this function
 * and continue the iteration.
 */
static inline __must_check
void __rcu **radix_tree_iter_retry(struct radix_tree_iter *iter)
{
	iter->next_index = iter->index;
	iter->tags = 0;
	return NULL;
}

static inline unsigned long
__radix_tree_iter_add(struct radix_tree_iter *iter, unsigned long slots)
{
	return iter->index + (slots << iter_shift(iter));
}

/**
 * radix_tree_iter_resume - resume iterating when the chunk may be invalid
 * @slot: pointer to current slot
 * @iter: iterator state
 * Returns: New slot pointer
 *
 * If the iterator needs to release then reacquire a lock, the chunk may
 * have been invalidated by an insertion or deletion.  Call this function
 * before releasing the lock to continue the iteration from the next index.
 */
void __rcu **__must_check radix_tree_iter_resume(void __rcu **slot,
					struct radix_tree_iter *iter);

/**
 * radix_tree_chunk_size - get current chunk size
 *
 * @iter:	pointer to radix tree iterator
 * Returns:	current chunk size
 */
static __always_inline long
radix_tree_chunk_size(struct radix_tree_iter *iter)
{
	return (iter->next_index - iter->index) >> iter_shift(iter);
}

#ifdef CONFIG_RADIX_TREE_MULTIORDER
void __rcu **__radix_tree_next_slot(void __rcu **slot,
				struct radix_tree_iter *iter, unsigned flags);
#else
/* Can't happen without sibling entries, but the compiler can't tell that */
static inline void __rcu **__radix_tree_next_slot(void __rcu **slot,
				struct radix_tree_iter *iter, unsigned flags)
{
	return slot;
}
#endif

/**
 * radix_tree_next_slot - find next slot in chunk
 *
 * @slot:	pointer to current slot
 * @iter:	pointer to interator state
 * @flags:	RADIX_TREE_ITER_*, should be constant
 * Returns:	pointer to next slot, or NULL if there no more left
 *
 * This function updates @iter->index in the case of a successful lookup.
 * For tagged lookup it also eats @iter->tags.
 *
 * There are several cases where 'slot' can be passed in as NULL to this
 * function.  These cases result from the use of radix_tree_iter_resume() or
 * radix_tree_iter_retry().  In these cases we don't end up dereferencing
 * 'slot' because either:
 * a) we are doing tagged iteration and iter->tags has been set to 0, or
 * b) we are doing non-tagged iteration, and iter->index and iter->next_index
 *    have been set up so that radix_tree_chunk_size() returns 1 or 0.
 */
static __always_inline void __rcu **radix_tree_next_slot(void __rcu **slot,
				struct radix_tree_iter *iter, unsigned flags)
{
	if (flags & RADIX_TREE_ITER_TAGGED) {
		iter->tags >>= 1;
		if (unlikely(!iter->tags))
			return NULL;
		if (likely(iter->tags & 1ul)) {
			iter->index = __radix_tree_iter_add(iter, 1);
			slot++;
			goto found;
		}
		if (!(flags & RADIX_TREE_ITER_CONTIG)) {
			unsigned offset = __ffs(iter->tags);

			iter->tags >>= offset++;
			iter->index = __radix_tree_iter_add(iter, offset);
			slot += offset;
			goto found;
		}
	} else {
		long count = radix_tree_chunk_size(iter);

		while (--count > 0) {
			slot++;
			iter->index = __radix_tree_iter_add(iter, 1);

			if (likely(*slot))
				goto found;
			if (flags & RADIX_TREE_ITER_CONTIG) {
				/* forbid switching to the next chunk */
				iter->next_index = 0;
				break;
			}
		}
	}
	return NULL;

 found:
	if (unlikely(radix_tree_is_internal_node(rcu_dereference_raw(*slot))))
		return __radix_tree_next_slot(slot, iter, flags);
	return slot;
}

/**
 * radix_tree_for_each_slot - iterate over non-empty slots
 *
 * @slot:	the void** variable for pointer to slot
 * @root:	the struct radix_tree_root pointer
 * @iter:	the struct radix_tree_iter pointer
 * @start:	iteration starting index
 *
 * @slot points to radix tree slot, @iter->index contains its index.
 */
#define radix_tree_for_each_slot(slot, root, iter, start)		\
	for (slot = radix_tree_iter_init(iter, start) ;			\
	     slot || (slot = radix_tree_next_chunk(root, iter, 0)) ;	\
	     slot = radix_tree_next_slot(slot, iter, 0))

/**
 * radix_tree_for_each_contig - iterate over contiguous slots
 *
 * @slot:	the void** variable for pointer to slot
 * @root:	the struct radix_tree_root pointer
 * @iter:	the struct radix_tree_iter pointer
 * @start:	iteration starting index
 *
 * @slot points to radix tree slot, @iter->index contains its index.
 */
#define radix_tree_for_each_contig(slot, root, iter, start)		\
	for (slot = radix_tree_iter_init(iter, start) ;			\
	     slot || (slot = radix_tree_next_chunk(root, iter,		\
				RADIX_TREE_ITER_CONTIG)) ;		\
	     slot = radix_tree_next_slot(slot, iter,			\
				RADIX_TREE_ITER_CONTIG))

/**
 * radix_tree_for_each_tagged - iterate over tagged slots
 *
 * @slot:	the void** variable for pointer to slot
 * @root:	the struct radix_tree_root pointer
 * @iter:	the struct radix_tree_iter pointer
 * @start:	iteration starting index
 * @tag:	tag index
 *
 * @slot points to radix tree slot, @iter->index contains its index.
 */
#define radix_tree_for_each_tagged(slot, root, iter, start, tag)	\
	for (slot = radix_tree_iter_init(iter, start) ;			\
	     slot || (slot = radix_tree_next_chunk(root, iter,		\
			      RADIX_TREE_ITER_TAGGED | tag)) ;		\
	     slot = radix_tree_next_slot(slot, iter,			\
				RADIX_TREE_ITER_TAGGED | tag))

#endif /* _LINUX_RADIX_TREE_H */
