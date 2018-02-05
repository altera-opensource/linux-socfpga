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
#include "xfs_ialloc.h"
#include "xfs_rmap.h"
#include "xfs_refcount.h"
#include "scrub/xfs_scrub.h"
#include "scrub/scrub.h"
#include "scrub/common.h"
#include "scrub/btree.h"
#include "scrub/trace.h"

/*
 * Set us up to scrub reverse mapping btrees.
 */
int
xfs_scrub_setup_ag_rmapbt(
	struct xfs_scrub_context	*sc,
	struct xfs_inode		*ip)
{
	return xfs_scrub_setup_ag_btree(sc, ip, false);
}

/* Reverse-mapping scrubber. */

/* Cross-reference a rmap against the refcount btree. */
STATIC void
xfs_scrub_rmapbt_xref_refc(
	struct xfs_scrub_context	*sc,
	struct xfs_rmap_irec		*irec)
{
	xfs_agblock_t			fbno;
	xfs_extlen_t			flen;
	bool				non_inode;
	bool				is_bmbt;
	bool				is_attr;
	bool				is_unwritten;
	int				error;

	if (!sc->sa.refc_cur)
		return;

	non_inode = XFS_RMAP_NON_INODE_OWNER(irec->rm_owner);
	is_bmbt = irec->rm_flags & XFS_RMAP_BMBT_BLOCK;
	is_attr = irec->rm_flags & XFS_RMAP_ATTR_FORK;
	is_unwritten = irec->rm_flags & XFS_RMAP_UNWRITTEN;

	/* If this is shared, must be a data fork extent. */
	error = xfs_refcount_find_shared(sc->sa.refc_cur, irec->rm_startblock,
			irec->rm_blockcount, &fbno, &flen, false);
	if (!xfs_scrub_should_check_xref(sc, &error, &sc->sa.refc_cur))
		return;
	if (flen != 0 && (non_inode || is_attr || is_bmbt || is_unwritten))
		xfs_scrub_btree_xref_set_corrupt(sc, sc->sa.refc_cur, 0);
}

/* Cross-reference with the other btrees. */
STATIC void
xfs_scrub_rmapbt_xref(
	struct xfs_scrub_context	*sc,
	struct xfs_rmap_irec		*irec)
{
	xfs_agblock_t			agbno = irec->rm_startblock;
	xfs_extlen_t			len = irec->rm_blockcount;

	if (sc->sm->sm_flags & XFS_SCRUB_OFLAG_CORRUPT)
		return;

	xfs_scrub_xref_is_used_space(sc, agbno, len);
	if (irec->rm_owner == XFS_RMAP_OWN_INODES)
		xfs_scrub_xref_is_inode_chunk(sc, agbno, len);
	else
		xfs_scrub_xref_is_not_inode_chunk(sc, agbno, len);
	if (irec->rm_owner == XFS_RMAP_OWN_COW)
		xfs_scrub_xref_is_cow_staging(sc, irec->rm_startblock,
				irec->rm_blockcount);
	else
		xfs_scrub_rmapbt_xref_refc(sc, irec);
}

/* Scrub an rmapbt record. */
STATIC int
xfs_scrub_rmapbt_rec(
	struct xfs_scrub_btree		*bs,
	union xfs_btree_rec		*rec)
{
	struct xfs_mount		*mp = bs->cur->bc_mp;
	struct xfs_rmap_irec		irec;
	xfs_agnumber_t			agno = bs->cur->bc_private.a.agno;
	bool				non_inode;
	bool				is_unwritten;
	bool				is_bmbt;
	bool				is_attr;
	int				error;

	error = xfs_rmap_btrec_to_irec(rec, &irec);
	if (!xfs_scrub_btree_process_error(bs->sc, bs->cur, 0, &error))
		goto out;

	/* Check extent. */
	if (irec.rm_startblock + irec.rm_blockcount <= irec.rm_startblock)
		xfs_scrub_btree_set_corrupt(bs->sc, bs->cur, 0);

	if (irec.rm_owner == XFS_RMAP_OWN_FS) {
		/*
		 * xfs_verify_agbno returns false for static fs metadata.
		 * Since that only exists at the start of the AG, validate
		 * that by hand.
		 */
		if (irec.rm_startblock != 0 ||
		    irec.rm_blockcount != XFS_AGFL_BLOCK(mp) + 1)
			xfs_scrub_btree_set_corrupt(bs->sc, bs->cur, 0);
	} else {
		/*
		 * Otherwise we must point somewhere past the static metadata
		 * but before the end of the FS.  Run the regular check.
		 */
		if (!xfs_verify_agbno(mp, agno, irec.rm_startblock) ||
		    !xfs_verify_agbno(mp, agno, irec.rm_startblock +
				irec.rm_blockcount - 1))
			xfs_scrub_btree_set_corrupt(bs->sc, bs->cur, 0);
	}

	/* Check flags. */
	non_inode = XFS_RMAP_NON_INODE_OWNER(irec.rm_owner);
	is_bmbt = irec.rm_flags & XFS_RMAP_BMBT_BLOCK;
	is_attr = irec.rm_flags & XFS_RMAP_ATTR_FORK;
	is_unwritten = irec.rm_flags & XFS_RMAP_UNWRITTEN;

	if (is_bmbt && irec.rm_offset != 0)
		xfs_scrub_btree_set_corrupt(bs->sc, bs->cur, 0);

	if (non_inode && irec.rm_offset != 0)
		xfs_scrub_btree_set_corrupt(bs->sc, bs->cur, 0);

	if (is_unwritten && (is_bmbt || non_inode || is_attr))
		xfs_scrub_btree_set_corrupt(bs->sc, bs->cur, 0);

	if (non_inode && (is_bmbt || is_unwritten || is_attr))
		xfs_scrub_btree_set_corrupt(bs->sc, bs->cur, 0);

	if (!non_inode) {
		if (!xfs_verify_ino(mp, irec.rm_owner))
			xfs_scrub_btree_set_corrupt(bs->sc, bs->cur, 0);
	} else {
		/* Non-inode owner within the magic values? */
		if (irec.rm_owner <= XFS_RMAP_OWN_MIN ||
		    irec.rm_owner > XFS_RMAP_OWN_FS)
			xfs_scrub_btree_set_corrupt(bs->sc, bs->cur, 0);
	}

	xfs_scrub_rmapbt_xref(bs->sc, &irec);
out:
	return error;
}

/* Scrub the rmap btree for some AG. */
int
xfs_scrub_rmapbt(
	struct xfs_scrub_context	*sc)
{
	struct xfs_owner_info		oinfo;

	xfs_rmap_ag_owner(&oinfo, XFS_RMAP_OWN_AG);
	return xfs_scrub_btree(sc, sc->sa.rmap_cur, xfs_scrub_rmapbt_rec,
			&oinfo, NULL);
}

/* xref check that the extent is owned by a given owner */
static inline void
xfs_scrub_xref_check_owner(
	struct xfs_scrub_context	*sc,
	xfs_agblock_t			bno,
	xfs_extlen_t			len,
	struct xfs_owner_info		*oinfo,
	bool				should_have_rmap)
{
	bool				has_rmap;
	int				error;

	if (!sc->sa.rmap_cur)
		return;

	error = xfs_rmap_record_exists(sc->sa.rmap_cur, bno, len, oinfo,
			&has_rmap);
	if (!xfs_scrub_should_check_xref(sc, &error, &sc->sa.rmap_cur))
		return;
	if (has_rmap != should_have_rmap)
		xfs_scrub_btree_xref_set_corrupt(sc, sc->sa.rmap_cur, 0);
}

/* xref check that the extent is owned by a given owner */
void
xfs_scrub_xref_is_owned_by(
	struct xfs_scrub_context	*sc,
	xfs_agblock_t			bno,
	xfs_extlen_t			len,
	struct xfs_owner_info		*oinfo)
{
	xfs_scrub_xref_check_owner(sc, bno, len, oinfo, true);
}

/* xref check that the extent is not owned by a given owner */
void
xfs_scrub_xref_is_not_owned_by(
	struct xfs_scrub_context	*sc,
	xfs_agblock_t			bno,
	xfs_extlen_t			len,
	struct xfs_owner_info		*oinfo)
{
	xfs_scrub_xref_check_owner(sc, bno, len, oinfo, false);
}

/* xref check that the extent has no reverse mapping at all */
void
xfs_scrub_xref_has_no_owner(
	struct xfs_scrub_context	*sc,
	xfs_agblock_t			bno,
	xfs_extlen_t			len)
{
	bool				has_rmap;
	int				error;

	if (!sc->sa.rmap_cur)
		return;

	error = xfs_rmap_has_record(sc->sa.rmap_cur, bno, len, &has_rmap);
	if (!xfs_scrub_should_check_xref(sc, &error, &sc->sa.rmap_cur))
		return;
	if (has_rmap)
		xfs_scrub_btree_xref_set_corrupt(sc, sc->sa.rmap_cur, 0);
}
