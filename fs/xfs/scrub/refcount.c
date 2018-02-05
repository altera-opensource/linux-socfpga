/*
 * Copyright (C) 2017 Oracle.  All Rights Reserved.
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
#include "xfs.h"
#include "xfs_fs.h"
#include "xfs_shared.h"
#include "xfs_format.h"
#include "xfs_trans_resv.h"
#include "xfs_mount.h"
#include "xfs_defer.h"
#include "xfs_btree.h"
#include "xfs_bit.h"
#include "xfs_log_format.h"
#include "xfs_trans.h"
#include "xfs_sb.h"
#include "xfs_alloc.h"
#include "xfs_rmap.h"
#include "xfs_refcount.h"
#include "scrub/xfs_scrub.h"
#include "scrub/scrub.h"
#include "scrub/common.h"
#include "scrub/btree.h"
#include "scrub/trace.h"

/*
 * Set us up to scrub reference count btrees.
 */
int
xfs_scrub_setup_ag_refcountbt(
	struct xfs_scrub_context	*sc,
	struct xfs_inode		*ip)
{
	return xfs_scrub_setup_ag_btree(sc, ip, false);
}

/* Reference count btree scrubber. */

/*
 * Confirming Reference Counts via Reverse Mappings
 *
 * We want to count the reverse mappings overlapping a refcount record
 * (bno, len, refcount), allowing for the possibility that some of the
 * overlap may come from smaller adjoining reverse mappings, while some
 * comes from single extents which overlap the range entirely.  The
 * outer loop is as follows:
 *
 * 1. For all reverse mappings overlapping the refcount extent,
 *    a. If a given rmap completely overlaps, mark it as seen.
 *    b. Otherwise, record the fragment (in agbno order) for later
 *       processing.
 *
 * Once we've seen all the rmaps, we know that for all blocks in the
 * refcount record we want to find $refcount owners and we've already
 * visited $seen extents that overlap all the blocks.  Therefore, we
 * need to find ($refcount - $seen) owners for every block in the
 * extent; call that quantity $target_nr.  Proceed as follows:
 *
 * 2. Pull the first $target_nr fragments from the list; all of them
 *    should start at or before the start of the extent.
 *    Call this subset of fragments the working set.
 * 3. Until there are no more unprocessed fragments,
 *    a. Find the shortest fragments in the set and remove them.
 *    b. Note the block number of the end of these fragments.
 *    c. Pull the same number of fragments from the list.  All of these
 *       fragments should start at the block number recorded in the
 *       previous step.
 *    d. Put those fragments in the set.
 * 4. Check that there are $target_nr fragments remaining in the list,
 *    and that they all end at or beyond the end of the refcount extent.
 *
 * If the refcount is correct, all the check conditions in the algorithm
 * should always hold true.  If not, the refcount is incorrect.
 */
struct xfs_scrub_refcnt_frag {
	struct list_head		list;
	struct xfs_rmap_irec		rm;
};

struct xfs_scrub_refcnt_check {
	struct xfs_scrub_context	*sc;
	struct list_head		fragments;

	/* refcount extent we're examining */
	xfs_agblock_t			bno;
	xfs_extlen_t			len;
	xfs_nlink_t			refcount;

	/* number of owners seen */
	xfs_nlink_t			seen;
};

/*
 * Decide if the given rmap is large enough that we can redeem it
 * towards refcount verification now, or if it's a fragment, in
 * which case we'll hang onto it in the hopes that we'll later
 * discover that we've collected exactly the correct number of
 * fragments as the refcountbt says we should have.
 */
STATIC int
xfs_scrub_refcountbt_rmap_check(
	struct xfs_btree_cur		*cur,
	struct xfs_rmap_irec		*rec,
	void				*priv)
{
	struct xfs_scrub_refcnt_check	*refchk = priv;
	struct xfs_scrub_refcnt_frag	*frag;
	xfs_agblock_t			rm_last;
	xfs_agblock_t			rc_last;
	int				error = 0;

	if (xfs_scrub_should_terminate(refchk->sc, &error))
		return error;

	rm_last = rec->rm_startblock + rec->rm_blockcount - 1;
	rc_last = refchk->bno + refchk->len - 1;

	/* Confirm that a single-owner refc extent is a CoW stage. */
	if (refchk->refcount == 1 && rec->rm_owner != XFS_RMAP_OWN_COW) {
		xfs_scrub_btree_xref_set_corrupt(refchk->sc, cur, 0);
		return 0;
	}

	if (rec->rm_startblock <= refchk->bno && rm_last >= rc_last) {
		/*
		 * The rmap overlaps the refcount record, so we can confirm
		 * one refcount owner seen.
		 */
		refchk->seen++;
	} else {
		/*
		 * This rmap covers only part of the refcount record, so
		 * save the fragment for later processing.  If the rmapbt
		 * is healthy each rmap_irec we see will be in agbno order
		 * so we don't need insertion sort here.
		 */
		frag = kmem_alloc(sizeof(struct xfs_scrub_refcnt_frag),
				KM_MAYFAIL | KM_NOFS);
		if (!frag)
			return -ENOMEM;
		memcpy(&frag->rm, rec, sizeof(frag->rm));
		list_add_tail(&frag->list, &refchk->fragments);
	}

	return 0;
}

/*
 * Given a bunch of rmap fragments, iterate through them, keeping
 * a running tally of the refcount.  If this ever deviates from
 * what we expect (which is the refcountbt's refcount minus the
 * number of extents that totally covered the refcountbt extent),
 * we have a refcountbt error.
 */
STATIC void
xfs_scrub_refcountbt_process_rmap_fragments(
	struct xfs_scrub_refcnt_check	*refchk)
{
	struct list_head		worklist;
	struct xfs_scrub_refcnt_frag	*frag;
	struct xfs_scrub_refcnt_frag	*n;
	xfs_agblock_t			bno;
	xfs_agblock_t			rbno;
	xfs_agblock_t			next_rbno;
	xfs_nlink_t			nr;
	xfs_nlink_t			target_nr;

	target_nr = refchk->refcount - refchk->seen;
	if (target_nr == 0)
		return;

	/*
	 * There are (refchk->rc.rc_refcount - refchk->nr refcount)
	 * references we haven't found yet.  Pull that many off the
	 * fragment list and figure out where the smallest rmap ends
	 * (and therefore the next rmap should start).  All the rmaps
	 * we pull off should start at or before the beginning of the
	 * refcount record's range.
	 */
	INIT_LIST_HEAD(&worklist);
	rbno = NULLAGBLOCK;
	nr = 1;

	/* Make sure the fragments actually /are/ in agbno order. */
	bno = 0;
	list_for_each_entry(frag, &refchk->fragments, list) {
		if (frag->rm.rm_startblock < bno)
			goto done;
		bno = frag->rm.rm_startblock;
	}

	/*
	 * Find all the rmaps that start at or before the refc extent,
	 * and put them on the worklist.
	 */
	list_for_each_entry_safe(frag, n, &refchk->fragments, list) {
		if (frag->rm.rm_startblock > refchk->bno)
			goto done;
		bno = frag->rm.rm_startblock + frag->rm.rm_blockcount;
		if (bno < rbno)
			rbno = bno;
		list_move_tail(&frag->list, &worklist);
		if (nr == target_nr)
			break;
		nr++;
	}

	/*
	 * We should have found exactly $target_nr rmap fragments starting
	 * at or before the refcount extent.
	 */
	if (nr != target_nr)
		goto done;

	while (!list_empty(&refchk->fragments)) {
		/* Discard any fragments ending at rbno from the worklist. */
		nr = 0;
		next_rbno = NULLAGBLOCK;
		list_for_each_entry_safe(frag, n, &worklist, list) {
			bno = frag->rm.rm_startblock + frag->rm.rm_blockcount;
			if (bno != rbno) {
				if (bno < next_rbno)
					next_rbno = bno;
				continue;
			}
			list_del(&frag->list);
			kmem_free(frag);
			nr++;
		}

		/* Try to add nr rmaps starting at rbno to the worklist. */
		list_for_each_entry_safe(frag, n, &refchk->fragments, list) {
			bno = frag->rm.rm_startblock + frag->rm.rm_blockcount;
			if (frag->rm.rm_startblock != rbno)
				goto done;
			list_move_tail(&frag->list, &worklist);
			if (next_rbno > bno)
				next_rbno = bno;
			nr--;
			if (nr == 0)
				break;
		}

		/*
		 * If we get here and nr > 0, this means that we added fewer
		 * items to the worklist than we discarded because the fragment
		 * list ran out of items.  Therefore, we cannot maintain the
		 * required refcount.  Something is wrong, so we're done.
		 */
		if (nr)
			goto done;

		rbno = next_rbno;
	}

	/*
	 * Make sure the last extent we processed ends at or beyond
	 * the end of the refcount extent.
	 */
	if (rbno < refchk->bno + refchk->len)
		goto done;

	/* Actually record us having seen the remaining refcount. */
	refchk->seen = refchk->refcount;
done:
	/* Delete fragments and work list. */
	list_for_each_entry_safe(frag, n, &worklist, list) {
		list_del(&frag->list);
		kmem_free(frag);
	}
	list_for_each_entry_safe(frag, n, &refchk->fragments, list) {
		list_del(&frag->list);
		kmem_free(frag);
	}
}

/* Use the rmap entries covering this extent to verify the refcount. */
STATIC void
xfs_scrub_refcountbt_xref_rmap(
	struct xfs_scrub_context	*sc,
	xfs_agblock_t			bno,
	xfs_extlen_t			len,
	xfs_nlink_t			refcount)
{
	struct xfs_scrub_refcnt_check	refchk = {
		.sc = sc,
		.bno = bno,
		.len = len,
		.refcount = refcount,
		.seen = 0,
	};
	struct xfs_rmap_irec		low;
	struct xfs_rmap_irec		high;
	struct xfs_scrub_refcnt_frag	*frag;
	struct xfs_scrub_refcnt_frag	*n;
	int				error;

	if (!sc->sa.rmap_cur)
		return;

	/* Cross-reference with the rmapbt to confirm the refcount. */
	memset(&low, 0, sizeof(low));
	low.rm_startblock = bno;
	memset(&high, 0xFF, sizeof(high));
	high.rm_startblock = bno + len - 1;

	INIT_LIST_HEAD(&refchk.fragments);
	error = xfs_rmap_query_range(sc->sa.rmap_cur, &low, &high,
			&xfs_scrub_refcountbt_rmap_check, &refchk);
	if (!xfs_scrub_should_check_xref(sc, &error, &sc->sa.rmap_cur))
		goto out_free;

	xfs_scrub_refcountbt_process_rmap_fragments(&refchk);
	if (refcount != refchk.seen)
		xfs_scrub_btree_xref_set_corrupt(sc, sc->sa.rmap_cur, 0);

out_free:
	list_for_each_entry_safe(frag, n, &refchk.fragments, list) {
		list_del(&frag->list);
		kmem_free(frag);
	}
}

/* Cross-reference with the other btrees. */
STATIC void
xfs_scrub_refcountbt_xref(
	struct xfs_scrub_context	*sc,
	xfs_agblock_t			agbno,
	xfs_extlen_t			len,
	xfs_nlink_t			refcount)
{
	if (sc->sm->sm_flags & XFS_SCRUB_OFLAG_CORRUPT)
		return;

	xfs_scrub_xref_is_used_space(sc, agbno, len);
	xfs_scrub_xref_is_not_inode_chunk(sc, agbno, len);
	xfs_scrub_refcountbt_xref_rmap(sc, agbno, len, refcount);
}

/* Scrub a refcountbt record. */
STATIC int
xfs_scrub_refcountbt_rec(
	struct xfs_scrub_btree		*bs,
	union xfs_btree_rec		*rec)
{
	struct xfs_mount		*mp = bs->cur->bc_mp;
	xfs_agblock_t			*cow_blocks = bs->private;
	xfs_agnumber_t			agno = bs->cur->bc_private.a.agno;
	xfs_agblock_t			bno;
	xfs_extlen_t			len;
	xfs_nlink_t			refcount;
	bool				has_cowflag;
	int				error = 0;

	bno = be32_to_cpu(rec->refc.rc_startblock);
	len = be32_to_cpu(rec->refc.rc_blockcount);
	refcount = be32_to_cpu(rec->refc.rc_refcount);

	/* Only CoW records can have refcount == 1. */
	has_cowflag = (bno & XFS_REFC_COW_START);
	if ((refcount == 1 && !has_cowflag) || (refcount != 1 && has_cowflag))
		xfs_scrub_btree_set_corrupt(bs->sc, bs->cur, 0);
	if (has_cowflag)
		(*cow_blocks) += len;

	/* Check the extent. */
	bno &= ~XFS_REFC_COW_START;
	if (bno + len <= bno ||
	    !xfs_verify_agbno(mp, agno, bno) ||
	    !xfs_verify_agbno(mp, agno, bno + len - 1))
		xfs_scrub_btree_set_corrupt(bs->sc, bs->cur, 0);

	if (refcount == 0)
		xfs_scrub_btree_set_corrupt(bs->sc, bs->cur, 0);

	xfs_scrub_refcountbt_xref(bs->sc, bno, len, refcount);

	return error;
}

/* Make sure we have as many refc blocks as the rmap says. */
STATIC void
xfs_scrub_refcount_xref_rmap(
	struct xfs_scrub_context	*sc,
	struct xfs_owner_info		*oinfo,
	xfs_filblks_t			cow_blocks)
{
	xfs_extlen_t			refcbt_blocks = 0;
	xfs_filblks_t			blocks;
	int				error;

	if (!sc->sa.rmap_cur)
		return;

	/* Check that we saw as many refcbt blocks as the rmap knows about. */
	error = xfs_btree_count_blocks(sc->sa.refc_cur, &refcbt_blocks);
	if (!xfs_scrub_btree_process_error(sc, sc->sa.refc_cur, 0, &error))
		return;
	error = xfs_scrub_count_rmap_ownedby_ag(sc, sc->sa.rmap_cur, oinfo,
			&blocks);
	if (!xfs_scrub_should_check_xref(sc, &error, &sc->sa.rmap_cur))
		return;
	if (blocks != refcbt_blocks)
		xfs_scrub_btree_xref_set_corrupt(sc, sc->sa.rmap_cur, 0);

	/* Check that we saw as many cow blocks as the rmap knows about. */
	xfs_rmap_ag_owner(oinfo, XFS_RMAP_OWN_COW);
	error = xfs_scrub_count_rmap_ownedby_ag(sc, sc->sa.rmap_cur, oinfo,
			&blocks);
	if (!xfs_scrub_should_check_xref(sc, &error, &sc->sa.rmap_cur))
		return;
	if (blocks != cow_blocks)
		xfs_scrub_btree_xref_set_corrupt(sc, sc->sa.rmap_cur, 0);
}

/* Scrub the refcount btree for some AG. */
int
xfs_scrub_refcountbt(
	struct xfs_scrub_context	*sc)
{
	struct xfs_owner_info		oinfo;
	xfs_agblock_t			cow_blocks = 0;
	int				error;

	xfs_rmap_ag_owner(&oinfo, XFS_RMAP_OWN_REFC);
	error = xfs_scrub_btree(sc, sc->sa.refc_cur, xfs_scrub_refcountbt_rec,
			&oinfo, &cow_blocks);
	if (error)
		return error;

	xfs_scrub_refcount_xref_rmap(sc, &oinfo, cow_blocks);

	return 0;
}

/* xref check that a cow staging extent is marked in the refcountbt. */
void
xfs_scrub_xref_is_cow_staging(
	struct xfs_scrub_context	*sc,
	xfs_agblock_t			agbno,
	xfs_extlen_t			len)
{
	struct xfs_refcount_irec	rc;
	bool				has_cowflag;
	int				has_refcount;
	int				error;

	if (!sc->sa.refc_cur)
		return;

	/* Find the CoW staging extent. */
	error = xfs_refcount_lookup_le(sc->sa.refc_cur,
			agbno + XFS_REFC_COW_START, &has_refcount);
	if (!xfs_scrub_should_check_xref(sc, &error, &sc->sa.refc_cur))
		return;
	if (!has_refcount) {
		xfs_scrub_btree_xref_set_corrupt(sc, sc->sa.refc_cur, 0);
		return;
	}

	error = xfs_refcount_get_rec(sc->sa.refc_cur, &rc, &has_refcount);
	if (!xfs_scrub_should_check_xref(sc, &error, &sc->sa.refc_cur))
		return;
	if (!has_refcount) {
		xfs_scrub_btree_xref_set_corrupt(sc, sc->sa.refc_cur, 0);
		return;
	}

	/* CoW flag must be set, refcount must be 1. */
	has_cowflag = (rc.rc_startblock & XFS_REFC_COW_START);
	if (!has_cowflag || rc.rc_refcount != 1)
		xfs_scrub_btree_xref_set_corrupt(sc, sc->sa.refc_cur, 0);

	/* Must be at least as long as what was passed in */
	if (rc.rc_blockcount < len)
		xfs_scrub_btree_xref_set_corrupt(sc, sc->sa.refc_cur, 0);
}

/*
 * xref check that the extent is not shared.  Only file data blocks
 * can have multiple owners.
 */
void
xfs_scrub_xref_is_not_shared(
	struct xfs_scrub_context	*sc,
	xfs_agblock_t			agbno,
	xfs_extlen_t			len)
{
	bool				shared;
	int				error;

	if (!sc->sa.refc_cur)
		return;

	error = xfs_refcount_has_record(sc->sa.refc_cur, agbno, len, &shared);
	if (!xfs_scrub_should_check_xref(sc, &error, &sc->sa.refc_cur))
		return;
	if (shared)
		xfs_scrub_btree_xref_set_corrupt(sc, sc->sa.refc_cur, 0);
}
