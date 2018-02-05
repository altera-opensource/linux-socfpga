// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2015, 2017 Oracle.  All rights reserved.
 * Copyright (c) 2003-2007 Network Appliance, Inc. All rights reserved.
 */

/* Lightweight memory registration using Fast Registration Work
 * Requests (FRWR).
 *
 * FRWR features ordered asynchronous registration and deregistration
 * of arbitrarily sized memory regions. This is the fastest and safest
 * but most complex memory registration mode.
 */

/* Normal operation
 *
 * A Memory Region is prepared for RDMA READ or WRITE using a FAST_REG
 * Work Request (frwr_op_map). When the RDMA operation is finished, this
 * Memory Region is invalidated using a LOCAL_INV Work Request
 * (frwr_op_unmap_sync).
 *
 * Typically these Work Requests are not signaled, and neither are RDMA
 * SEND Work Requests (with the exception of signaling occasionally to
 * prevent provider work queue overflows). This greatly reduces HCA
 * interrupt workload.
 *
 * As an optimization, frwr_op_unmap marks MRs INVALID before the
 * LOCAL_INV WR is posted. If posting succeeds, the MR is placed on
 * rb_mrs immediately so that no work (like managing a linked list
 * under a spinlock) is needed in the completion upcall.
 *
 * But this means that frwr_op_map() can occasionally encounter an MR
 * that is INVALID but the LOCAL_INV WR has not completed. Work Queue
 * ordering prevents a subsequent FAST_REG WR from executing against
 * that MR while it is still being invalidated.
 */

/* Transport recovery
 *
 * ->op_map and the transport connect worker cannot run at the same
 * time, but ->op_unmap can fire while the transport connect worker
 * is running. Thus MR recovery is handled in ->op_map, to guarantee
 * that recovered MRs are owned by a sending RPC, and not one where
 * ->op_unmap could fire at the same time transport reconnect is
 * being done.
 *
 * When the underlying transport disconnects, MRs are left in one of
 * four states:
 *
 * INVALID:	The MR was not in use before the QP entered ERROR state.
 *
 * VALID:	The MR was registered before the QP entered ERROR state.
 *
 * FLUSHED_FR:	The MR was being registered when the QP entered ERROR
 *		state, and the pending WR was flushed.
 *
 * FLUSHED_LI:	The MR was being invalidated when the QP entered ERROR
 *		state, and the pending WR was flushed.
 *
 * When frwr_op_map encounters FLUSHED and VALID MRs, they are recovered
 * with ib_dereg_mr and then are re-initialized. Because MR recovery
 * allocates fresh resources, it is deferred to a workqueue, and the
 * recovered MRs are placed back on the rb_mrs list when recovery is
 * complete. frwr_op_map allocates another MR for the current RPC while
 * the broken MR is reset.
 *
 * To ensure that frwr_op_map doesn't encounter an MR that is marked
 * INVALID but that is about to be flushed due to a previous transport
 * disconnect, the transport connect worker attempts to drain all
 * pending send queue WRs before the transport is reconnected.
 */

#include <linux/sunrpc/rpc_rdma.h>

#include "xprt_rdma.h"

#if IS_ENABLED(CONFIG_SUNRPC_DEBUG)
# define RPCDBG_FACILITY	RPCDBG_TRANS
#endif

bool
frwr_is_supported(struct rpcrdma_ia *ia)
{
	struct ib_device_attr *attrs = &ia->ri_device->attrs;

	if (!(attrs->device_cap_flags & IB_DEVICE_MEM_MGT_EXTENSIONS))
		goto out_not_supported;
	if (attrs->max_fast_reg_page_list_len == 0)
		goto out_not_supported;
	return true;

out_not_supported:
	pr_info("rpcrdma: 'frwr' mode is not supported by device %s\n",
		ia->ri_device->name);
	return false;
}

static int
frwr_op_init_mr(struct rpcrdma_ia *ia, struct rpcrdma_mr *mr)
{
	unsigned int depth = ia->ri_max_frwr_depth;
	struct rpcrdma_frwr *frwr = &mr->frwr;
	int rc;

	frwr->fr_mr = ib_alloc_mr(ia->ri_pd, ia->ri_mrtype, depth);
	if (IS_ERR(frwr->fr_mr))
		goto out_mr_err;

	mr->mr_sg = kcalloc(depth, sizeof(*mr->mr_sg), GFP_KERNEL);
	if (!mr->mr_sg)
		goto out_list_err;

	sg_init_table(mr->mr_sg, depth);
	init_completion(&frwr->fr_linv_done);
	return 0;

out_mr_err:
	rc = PTR_ERR(frwr->fr_mr);
	dprintk("RPC:       %s: ib_alloc_mr status %i\n",
		__func__, rc);
	return rc;

out_list_err:
	rc = -ENOMEM;
	dprintk("RPC:       %s: sg allocation failure\n",
		__func__);
	ib_dereg_mr(frwr->fr_mr);
	return rc;
}

static void
frwr_op_release_mr(struct rpcrdma_mr *mr)
{
	int rc;

	/* Ensure MR is not on any rl_registered list */
	if (!list_empty(&mr->mr_list))
		list_del(&mr->mr_list);

	rc = ib_dereg_mr(mr->frwr.fr_mr);
	if (rc)
		pr_err("rpcrdma: final ib_dereg_mr for %p returned %i\n",
		       mr, rc);
	kfree(mr->mr_sg);
	kfree(mr);
}

static int
__frwr_mr_reset(struct rpcrdma_ia *ia, struct rpcrdma_mr *mr)
{
	struct rpcrdma_frwr *frwr = &mr->frwr;
	int rc;

	rc = ib_dereg_mr(frwr->fr_mr);
	if (rc) {
		pr_warn("rpcrdma: ib_dereg_mr status %d, frwr %p orphaned\n",
			rc, mr);
		return rc;
	}

	frwr->fr_mr = ib_alloc_mr(ia->ri_pd, ia->ri_mrtype,
				  ia->ri_max_frwr_depth);
	if (IS_ERR(frwr->fr_mr)) {
		pr_warn("rpcrdma: ib_alloc_mr status %ld, frwr %p orphaned\n",
			PTR_ERR(frwr->fr_mr), mr);
		return PTR_ERR(frwr->fr_mr);
	}

	dprintk("RPC:       %s: recovered FRWR %p\n", __func__, frwr);
	frwr->fr_state = FRWR_IS_INVALID;
	return 0;
}

/* Reset of a single FRWR. Generate a fresh rkey by replacing the MR.
 */
static void
frwr_op_recover_mr(struct rpcrdma_mr *mr)
{
	enum rpcrdma_frwr_state state = mr->frwr.fr_state;
	struct rpcrdma_xprt *r_xprt = mr->mr_xprt;
	struct rpcrdma_ia *ia = &r_xprt->rx_ia;
	int rc;

	rc = __frwr_mr_reset(ia, mr);
	if (state != FRWR_FLUSHED_LI) {
		trace_xprtrdma_dma_unmap(mr);
		ib_dma_unmap_sg(ia->ri_device,
				mr->mr_sg, mr->mr_nents, mr->mr_dir);
	}
	if (rc)
		goto out_release;

	rpcrdma_mr_put(mr);
	r_xprt->rx_stats.mrs_recovered++;
	return;

out_release:
	pr_err("rpcrdma: FRWR reset failed %d, %p release\n", rc, mr);
	r_xprt->rx_stats.mrs_orphaned++;

	spin_lock(&r_xprt->rx_buf.rb_mrlock);
	list_del(&mr->mr_all);
	spin_unlock(&r_xprt->rx_buf.rb_mrlock);

	frwr_op_release_mr(mr);
}

static int
frwr_op_open(struct rpcrdma_ia *ia, struct rpcrdma_ep *ep,
	     struct rpcrdma_create_data_internal *cdata)
{
	struct ib_device_attr *attrs = &ia->ri_device->attrs;
	int depth, delta;

	ia->ri_mrtype = IB_MR_TYPE_MEM_REG;
	if (attrs->device_cap_flags & IB_DEVICE_SG_GAPS_REG)
		ia->ri_mrtype = IB_MR_TYPE_SG_GAPS;

	ia->ri_max_frwr_depth =
			min_t(unsigned int, RPCRDMA_MAX_DATA_SEGS,
			      attrs->max_fast_reg_page_list_len);
	dprintk("RPC:       %s: device's max FR page list len = %u\n",
		__func__, ia->ri_max_frwr_depth);

	/* Add room for frwr register and invalidate WRs.
	 * 1. FRWR reg WR for head
	 * 2. FRWR invalidate WR for head
	 * 3. N FRWR reg WRs for pagelist
	 * 4. N FRWR invalidate WRs for pagelist
	 * 5. FRWR reg WR for tail
	 * 6. FRWR invalidate WR for tail
	 * 7. The RDMA_SEND WR
	 */
	depth = 7;

	/* Calculate N if the device max FRWR depth is smaller than
	 * RPCRDMA_MAX_DATA_SEGS.
	 */
	if (ia->ri_max_frwr_depth < RPCRDMA_MAX_DATA_SEGS) {
		delta = RPCRDMA_MAX_DATA_SEGS - ia->ri_max_frwr_depth;
		do {
			depth += 2; /* FRWR reg + invalidate */
			delta -= ia->ri_max_frwr_depth;
		} while (delta > 0);
	}

	ep->rep_attr.cap.max_send_wr *= depth;
	if (ep->rep_attr.cap.max_send_wr > attrs->max_qp_wr) {
		cdata->max_requests = attrs->max_qp_wr / depth;
		if (!cdata->max_requests)
			return -EINVAL;
		ep->rep_attr.cap.max_send_wr = cdata->max_requests *
					       depth;
	}

	ia->ri_max_segs = max_t(unsigned int, 1, RPCRDMA_MAX_DATA_SEGS /
				ia->ri_max_frwr_depth);
	return 0;
}

/* FRWR mode conveys a list of pages per chunk segment. The
 * maximum length of that list is the FRWR page list depth.
 */
static size_t
frwr_op_maxpages(struct rpcrdma_xprt *r_xprt)
{
	struct rpcrdma_ia *ia = &r_xprt->rx_ia;

	return min_t(unsigned int, RPCRDMA_MAX_DATA_SEGS,
		     RPCRDMA_MAX_HDR_SEGS * ia->ri_max_frwr_depth);
}

static void
__frwr_sendcompletion_flush(struct ib_wc *wc, const char *wr)
{
	if (wc->status != IB_WC_WR_FLUSH_ERR)
		pr_err("rpcrdma: %s: %s (%u/0x%x)\n",
		       wr, ib_wc_status_msg(wc->status),
		       wc->status, wc->vendor_err);
}

/**
 * frwr_wc_fastreg - Invoked by RDMA provider for a flushed FastReg WC
 * @cq:	completion queue (ignored)
 * @wc:	completed WR
 *
 */
static void
frwr_wc_fastreg(struct ib_cq *cq, struct ib_wc *wc)
{
	struct ib_cqe *cqe = wc->wr_cqe;
	struct rpcrdma_frwr *frwr =
			container_of(cqe, struct rpcrdma_frwr, fr_cqe);

	/* WARNING: Only wr_cqe and status are reliable at this point */
	if (wc->status != IB_WC_SUCCESS) {
		frwr->fr_state = FRWR_FLUSHED_FR;
		__frwr_sendcompletion_flush(wc, "fastreg");
	}
	trace_xprtrdma_wc_fastreg(wc, frwr);
}

/**
 * frwr_wc_localinv - Invoked by RDMA provider for a flushed LocalInv WC
 * @cq:	completion queue (ignored)
 * @wc:	completed WR
 *
 */
static void
frwr_wc_localinv(struct ib_cq *cq, struct ib_wc *wc)
{
	struct ib_cqe *cqe = wc->wr_cqe;
	struct rpcrdma_frwr *frwr = container_of(cqe, struct rpcrdma_frwr,
						 fr_cqe);

	/* WARNING: Only wr_cqe and status are reliable at this point */
	if (wc->status != IB_WC_SUCCESS) {
		frwr->fr_state = FRWR_FLUSHED_LI;
		__frwr_sendcompletion_flush(wc, "localinv");
	}
	trace_xprtrdma_wc_li(wc, frwr);
}

/**
 * frwr_wc_localinv_wake - Invoked by RDMA provider for a signaled LocalInv WC
 * @cq:	completion queue (ignored)
 * @wc:	completed WR
 *
 * Awaken anyone waiting for an MR to finish being fenced.
 */
static void
frwr_wc_localinv_wake(struct ib_cq *cq, struct ib_wc *wc)
{
	struct ib_cqe *cqe = wc->wr_cqe;
	struct rpcrdma_frwr *frwr = container_of(cqe, struct rpcrdma_frwr,
						 fr_cqe);

	/* WARNING: Only wr_cqe and status are reliable at this point */
	if (wc->status != IB_WC_SUCCESS) {
		frwr->fr_state = FRWR_FLUSHED_LI;
		__frwr_sendcompletion_flush(wc, "localinv");
	}
	complete(&frwr->fr_linv_done);
	trace_xprtrdma_wc_li_wake(wc, frwr);
}

/* Post a REG_MR Work Request to register a memory region
 * for remote access via RDMA READ or RDMA WRITE.
 */
static struct rpcrdma_mr_seg *
frwr_op_map(struct rpcrdma_xprt *r_xprt, struct rpcrdma_mr_seg *seg,
	    int nsegs, bool writing, struct rpcrdma_mr **out)
{
	struct rpcrdma_ia *ia = &r_xprt->rx_ia;
	bool holes_ok = ia->ri_mrtype == IB_MR_TYPE_SG_GAPS;
	struct rpcrdma_frwr *frwr;
	struct rpcrdma_mr *mr;
	struct ib_mr *ibmr;
	struct ib_reg_wr *reg_wr;
	struct ib_send_wr *bad_wr;
	int rc, i, n;
	u8 key;

	mr = NULL;
	do {
		if (mr)
			rpcrdma_mr_defer_recovery(mr);
		mr = rpcrdma_mr_get(r_xprt);
		if (!mr)
			return ERR_PTR(-ENOBUFS);
	} while (mr->frwr.fr_state != FRWR_IS_INVALID);
	frwr = &mr->frwr;
	frwr->fr_state = FRWR_IS_VALID;

	if (nsegs > ia->ri_max_frwr_depth)
		nsegs = ia->ri_max_frwr_depth;
	for (i = 0; i < nsegs;) {
		if (seg->mr_page)
			sg_set_page(&mr->mr_sg[i],
				    seg->mr_page,
				    seg->mr_len,
				    offset_in_page(seg->mr_offset));
		else
			sg_set_buf(&mr->mr_sg[i], seg->mr_offset,
				   seg->mr_len);

		++seg;
		++i;
		if (holes_ok)
			continue;
		if ((i < nsegs && offset_in_page(seg->mr_offset)) ||
		    offset_in_page((seg-1)->mr_offset + (seg-1)->mr_len))
			break;
	}
	mr->mr_dir = rpcrdma_data_dir(writing);

	mr->mr_nents = ib_dma_map_sg(ia->ri_device, mr->mr_sg, i, mr->mr_dir);
	if (!mr->mr_nents)
		goto out_dmamap_err;

	ibmr = frwr->fr_mr;
	n = ib_map_mr_sg(ibmr, mr->mr_sg, mr->mr_nents, NULL, PAGE_SIZE);
	if (unlikely(n != mr->mr_nents))
		goto out_mapmr_err;

	key = (u8)(ibmr->rkey & 0x000000FF);
	ib_update_fast_reg_key(ibmr, ++key);

	reg_wr = &frwr->fr_regwr;
	reg_wr->wr.next = NULL;
	reg_wr->wr.opcode = IB_WR_REG_MR;
	frwr->fr_cqe.done = frwr_wc_fastreg;
	reg_wr->wr.wr_cqe = &frwr->fr_cqe;
	reg_wr->wr.num_sge = 0;
	reg_wr->wr.send_flags = 0;
	reg_wr->mr = ibmr;
	reg_wr->key = ibmr->rkey;
	reg_wr->access = writing ?
			 IB_ACCESS_REMOTE_WRITE | IB_ACCESS_LOCAL_WRITE :
			 IB_ACCESS_REMOTE_READ;

	rc = ib_post_send(ia->ri_id->qp, &reg_wr->wr, &bad_wr);
	if (rc)
		goto out_senderr;

	mr->mr_handle = ibmr->rkey;
	mr->mr_length = ibmr->length;
	mr->mr_offset = ibmr->iova;

	*out = mr;
	return seg;

out_dmamap_err:
	pr_err("rpcrdma: failed to DMA map sg %p sg_nents %d\n",
	       mr->mr_sg, i);
	frwr->fr_state = FRWR_IS_INVALID;
	rpcrdma_mr_put(mr);
	return ERR_PTR(-EIO);

out_mapmr_err:
	pr_err("rpcrdma: failed to map mr %p (%d/%d)\n",
	       frwr->fr_mr, n, mr->mr_nents);
	rpcrdma_mr_defer_recovery(mr);
	return ERR_PTR(-EIO);

out_senderr:
	pr_err("rpcrdma: FRWR registration ib_post_send returned %i\n", rc);
	rpcrdma_mr_defer_recovery(mr);
	return ERR_PTR(-ENOTCONN);
}

/* Handle a remotely invalidated mr on the @mrs list
 */
static void
frwr_op_reminv(struct rpcrdma_rep *rep, struct list_head *mrs)
{
	struct rpcrdma_mr *mr;

	list_for_each_entry(mr, mrs, mr_list)
		if (mr->mr_handle == rep->rr_inv_rkey) {
			list_del(&mr->mr_list);
			trace_xprtrdma_remoteinv(mr);
			mr->frwr.fr_state = FRWR_IS_INVALID;
			rpcrdma_mr_unmap_and_put(mr);
			break;	/* only one invalidated MR per RPC */
		}
}

/* Invalidate all memory regions that were registered for "req".
 *
 * Sleeps until it is safe for the host CPU to access the
 * previously mapped memory regions.
 *
 * Caller ensures that @mrs is not empty before the call. This
 * function empties the list.
 */
static void
frwr_op_unmap_sync(struct rpcrdma_xprt *r_xprt, struct list_head *mrs)
{
	struct ib_send_wr *first, **prev, *last, *bad_wr;
	struct rpcrdma_ia *ia = &r_xprt->rx_ia;
	struct rpcrdma_frwr *frwr;
	struct rpcrdma_mr *mr;
	int count, rc;

	/* ORDER: Invalidate all of the MRs first
	 *
	 * Chain the LOCAL_INV Work Requests and post them with
	 * a single ib_post_send() call.
	 */
	frwr = NULL;
	count = 0;
	prev = &first;
	list_for_each_entry(mr, mrs, mr_list) {
		mr->frwr.fr_state = FRWR_IS_INVALID;

		frwr = &mr->frwr;
		trace_xprtrdma_localinv(mr);

		frwr->fr_cqe.done = frwr_wc_localinv;
		last = &frwr->fr_invwr;
		memset(last, 0, sizeof(*last));
		last->wr_cqe = &frwr->fr_cqe;
		last->opcode = IB_WR_LOCAL_INV;
		last->ex.invalidate_rkey = mr->mr_handle;
		count++;

		*prev = last;
		prev = &last->next;
	}
	if (!frwr)
		goto unmap;

	/* Strong send queue ordering guarantees that when the
	 * last WR in the chain completes, all WRs in the chain
	 * are complete.
	 */
	last->send_flags = IB_SEND_SIGNALED;
	frwr->fr_cqe.done = frwr_wc_localinv_wake;
	reinit_completion(&frwr->fr_linv_done);

	/* Transport disconnect drains the receive CQ before it
	 * replaces the QP. The RPC reply handler won't call us
	 * unless ri_id->qp is a valid pointer.
	 */
	r_xprt->rx_stats.local_inv_needed++;
	bad_wr = NULL;
	rc = ib_post_send(ia->ri_id->qp, first, &bad_wr);
	if (bad_wr != first)
		wait_for_completion(&frwr->fr_linv_done);
	if (rc)
		goto reset_mrs;

	/* ORDER: Now DMA unmap all of the MRs, and return
	 * them to the free MR list.
	 */
unmap:
	while (!list_empty(mrs)) {
		mr = rpcrdma_mr_pop(mrs);
		rpcrdma_mr_unmap_and_put(mr);
	}
	return;

reset_mrs:
	pr_err("rpcrdma: FRWR invalidate ib_post_send returned %i\n", rc);

	/* Find and reset the MRs in the LOCAL_INV WRs that did not
	 * get posted.
	 */
	while (bad_wr) {
		frwr = container_of(bad_wr, struct rpcrdma_frwr,
				    fr_invwr);
		mr = container_of(frwr, struct rpcrdma_mr, frwr);

		__frwr_mr_reset(ia, mr);

		bad_wr = bad_wr->next;
	}
	goto unmap;
}

const struct rpcrdma_memreg_ops rpcrdma_frwr_memreg_ops = {
	.ro_map				= frwr_op_map,
	.ro_reminv			= frwr_op_reminv,
	.ro_unmap_sync			= frwr_op_unmap_sync,
	.ro_recover_mr			= frwr_op_recover_mr,
	.ro_open			= frwr_op_open,
	.ro_maxpages			= frwr_op_maxpages,
	.ro_init_mr			= frwr_op_init_mr,
	.ro_release_mr			= frwr_op_release_mr,
	.ro_displayname			= "frwr",
	.ro_send_w_inv_ok		= RPCRDMA_CMP_F_SND_W_INV_OK,
};
