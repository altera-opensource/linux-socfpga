/*
 * Copyright (C) 2016 Oracle.  All Rights Reserved.
 *
 * Author: Darrick J. Wong <darrick.wong@oracle.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it would be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write the Free Software Foundation,
 * Inc.,  51 Franklin St, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#ifndef __XFS_REFCOUNT_H__
#define __XFS_REFCOUNT_H__

extern int xfs_refcount_lookup_le(struct xfs_btree_cur *cur,
		xfs_agblock_t bno, int *stat);
extern int xfs_refcount_lookup_ge(struct xfs_btree_cur *cur,
		xfs_agblock_t bno, int *stat);
extern int xfs_refcount_get_rec(struct xfs_btree_cur *cur,
		struct xfs_refcount_irec *irec, int *stat);

enum xfs_refcount_intent_type {
	XFS_REFCOUNT_INCREASE = 1,
	XFS_REFCOUNT_DECREASE,
	XFS_REFCOUNT_ALLOC_COW,
	XFS_REFCOUNT_FREE_COW,
};

struct xfs_refcount_intent {
	struct list_head			ri_list;
	enum xfs_refcount_intent_type		ri_type;
	xfs_fsblock_t				ri_startblock;
	xfs_extlen_t				ri_blockcount;
};

extern int xfs_refcount_increase_extent(struct xfs_mount *mp,
		struct xfs_defer_ops *dfops, struct xfs_bmbt_irec *irec);
extern int xfs_refcount_decrease_extent(struct xfs_mount *mp,
		struct xfs_defer_ops *dfops, struct xfs_bmbt_irec *irec);

extern void xfs_refcount_finish_one_cleanup(struct xfs_trans *tp,
		struct xfs_btree_cur *rcur, int error);
extern int xfs_refcount_finish_one(struct xfs_trans *tp,
		struct xfs_defer_ops *dfops, enum xfs_refcount_intent_type type,
		xfs_fsblock_t startblock, xfs_extlen_t blockcount,
		xfs_fsblock_t *new_fsb, xfs_extlen_t *new_len,
		struct xfs_btree_cur **pcur);

extern int xfs_refcount_find_shared(struct xfs_btree_cur *cur,
		xfs_agblock_t agbno, xfs_extlen_t aglen, xfs_agblock_t *fbno,
		xfs_extlen_t *flen, bool find_end_of_shared);

extern int xfs_refcount_alloc_cow_extent(struct xfs_mount *mp,
		struct xfs_defer_ops *dfops, xfs_fsblock_t fsb,
		xfs_extlen_t len);
extern int xfs_refcount_free_cow_extent(struct xfs_mount *mp,
		struct xfs_defer_ops *dfops, xfs_fsblock_t fsb,
		xfs_extlen_t len);
extern int xfs_refcount_recover_cow_leftovers(struct xfs_mount *mp,
		xfs_agnumber_t agno);

/*
 * While we're adjusting the refcounts records of an extent, we have
 * to keep an eye on the number of extents we're dirtying -- run too
 * many in a single transaction and we'll exceed the transaction's
 * reservation and crash the fs.  Each record adds 12 bytes to the
 * log (plus any key updates) so we'll conservatively assume 32 bytes
 * per record.  We must also leave space for btree splits on both ends
 * of the range and space for the CUD and a new CUI.
 */
#define XFS_REFCOUNT_ITEM_OVERHEAD	32

static inline xfs_fileoff_t xfs_refcount_max_unmap(int log_res)
{
	return (log_res * 3 / 4) / XFS_REFCOUNT_ITEM_OVERHEAD;
}

extern int xfs_refcount_has_record(struct xfs_btree_cur *cur,
		xfs_agblock_t bno, xfs_extlen_t len, bool *exists);

#endif	/* __XFS_REFCOUNT_H__ */
