/*
 *  BSG helper library
 *
 *  Copyright (C) 2008   James Smart, Emulex Corporation
 *  Copyright (C) 2011   Red Hat, Inc.  All rights reserved.
 *  Copyright (C) 2011   Mike Christie
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/slab.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/scatterlist.h>
#include <linux/bsg-lib.h>
#include <linux/export.h>
#include <scsi/scsi_cmnd.h>

/**
 * bsg_teardown_job - routine to teardown a bsg job
 * @kref: kref inside bsg_job that is to be torn down
 */
static void bsg_teardown_job(struct kref *kref)
{
	struct bsg_job *job = container_of(kref, struct bsg_job, kref);
	struct request *rq = job->req;

	put_device(job->dev);	/* release reference for the request */

	kfree(job->request_payload.sg_list);
	kfree(job->reply_payload.sg_list);

	blk_end_request_all(rq, BLK_STS_OK);
}

void bsg_job_put(struct bsg_job *job)
{
	kref_put(&job->kref, bsg_teardown_job);
}
EXPORT_SYMBOL_GPL(bsg_job_put);

int bsg_job_get(struct bsg_job *job)
{
	return kref_get_unless_zero(&job->kref);
}
EXPORT_SYMBOL_GPL(bsg_job_get);

/**
 * bsg_job_done - completion routine for bsg requests
 * @job: bsg_job that is complete
 * @result: job reply result
 * @reply_payload_rcv_len: length of payload recvd
 *
 * The LLD should call this when the bsg job has completed.
 */
void bsg_job_done(struct bsg_job *job, int result,
		  unsigned int reply_payload_rcv_len)
{
	struct request *req = job->req;
	struct request *rsp = req->next_rq;
	struct scsi_request *rq = scsi_req(req);
	int err;

	err = scsi_req(job->req)->result = result;
	if (err < 0)
		/* we're only returning the result field in the reply */
		rq->sense_len = sizeof(u32);
	else
		rq->sense_len = job->reply_len;
	/* we assume all request payload was transferred, residual == 0 */
	rq->resid_len = 0;

	if (rsp) {
		WARN_ON(reply_payload_rcv_len > scsi_req(rsp)->resid_len);

		/* set reply (bidi) residual */
		scsi_req(rsp)->resid_len -=
			min(reply_payload_rcv_len, scsi_req(rsp)->resid_len);
	}
	blk_complete_request(req);
}
EXPORT_SYMBOL_GPL(bsg_job_done);

/**
 * bsg_softirq_done - softirq done routine for destroying the bsg requests
 * @rq: BSG request that holds the job to be destroyed
 */
static void bsg_softirq_done(struct request *rq)
{
	struct bsg_job *job = blk_mq_rq_to_pdu(rq);

	bsg_job_put(job);
}

static int bsg_map_buffer(struct bsg_buffer *buf, struct request *req)
{
	size_t sz = (sizeof(struct scatterlist) * req->nr_phys_segments);

	BUG_ON(!req->nr_phys_segments);

	buf->sg_list = kzalloc(sz, GFP_KERNEL);
	if (!buf->sg_list)
		return -ENOMEM;
	sg_init_table(buf->sg_list, req->nr_phys_segments);
	scsi_req(req)->resid_len = blk_rq_bytes(req);
	buf->sg_cnt = blk_rq_map_sg(req->q, req, buf->sg_list);
	buf->payload_len = blk_rq_bytes(req);
	return 0;
}

/**
 * bsg_prepare_job - create the bsg_job structure for the bsg request
 * @dev: device that is being sent the bsg request
 * @req: BSG request that needs a job structure
 */
static int bsg_prepare_job(struct device *dev, struct request *req)
{
	struct request *rsp = req->next_rq;
	struct scsi_request *rq = scsi_req(req);
	struct bsg_job *job = blk_mq_rq_to_pdu(req);
	int ret;

	job->request = rq->cmd;
	job->request_len = rq->cmd_len;

	if (req->bio) {
		ret = bsg_map_buffer(&job->request_payload, req);
		if (ret)
			goto failjob_rls_job;
	}
	if (rsp && rsp->bio) {
		ret = bsg_map_buffer(&job->reply_payload, rsp);
		if (ret)
			goto failjob_rls_rqst_payload;
	}
	job->dev = dev;
	/* take a reference for the request */
	get_device(job->dev);
	kref_init(&job->kref);
	return 0;

failjob_rls_rqst_payload:
	kfree(job->request_payload.sg_list);
failjob_rls_job:
	return -ENOMEM;
}

/**
 * bsg_request_fn - generic handler for bsg requests
 * @q: request queue to manage
 *
 * On error the create_bsg_job function should return a -Exyz error value
 * that will be set to ->result.
 *
 * Drivers/subsys should pass this to the queue init function.
 */
static void bsg_request_fn(struct request_queue *q)
	__releases(q->queue_lock)
	__acquires(q->queue_lock)
{
	struct device *dev = q->queuedata;
	struct request *req;
	int ret;

	if (!get_device(dev))
		return;

	while (1) {
		req = blk_fetch_request(q);
		if (!req)
			break;
		spin_unlock_irq(q->queue_lock);

		ret = bsg_prepare_job(dev, req);
		if (ret) {
			scsi_req(req)->result = ret;
			blk_end_request_all(req, BLK_STS_OK);
			spin_lock_irq(q->queue_lock);
			continue;
		}

		ret = q->bsg_job_fn(blk_mq_rq_to_pdu(req));
		spin_lock_irq(q->queue_lock);
		if (ret)
			break;
	}

	spin_unlock_irq(q->queue_lock);
	put_device(dev);
	spin_lock_irq(q->queue_lock);
}

static int bsg_init_rq(struct request_queue *q, struct request *req, gfp_t gfp)
{
	struct bsg_job *job = blk_mq_rq_to_pdu(req);
	struct scsi_request *sreq = &job->sreq;

	/* called right after the request is allocated for the request_queue */

	sreq->sense = kzalloc(SCSI_SENSE_BUFFERSIZE, gfp);
	if (!sreq->sense)
		return -ENOMEM;

	return 0;
}

static void bsg_initialize_rq(struct request *req)
{
	struct bsg_job *job = blk_mq_rq_to_pdu(req);
	struct scsi_request *sreq = &job->sreq;
	void *sense = sreq->sense;

	/* called right before the request is given to the request_queue user */

	memset(job, 0, sizeof(*job));

	scsi_req_init(sreq);

	sreq->sense = sense;
	sreq->sense_len = SCSI_SENSE_BUFFERSIZE;

	job->req = req;
	job->reply = sense;
	job->reply_len = sreq->sense_len;
	job->dd_data = job + 1;
}

static void bsg_exit_rq(struct request_queue *q, struct request *req)
{
	struct bsg_job *job = blk_mq_rq_to_pdu(req);
	struct scsi_request *sreq = &job->sreq;

	kfree(sreq->sense);
}

/**
 * bsg_setup_queue - Create and add the bsg hooks so we can receive requests
 * @dev: device to attach bsg device to
 * @name: device to give bsg device
 * @job_fn: bsg job handler
 * @dd_job_size: size of LLD data needed for each job
 * @release: @dev release function
 */
struct request_queue *bsg_setup_queue(struct device *dev, const char *name,
		bsg_job_fn *job_fn, int dd_job_size,
		void (*release)(struct device *))
{
	struct request_queue *q;
	int ret;

	q = blk_alloc_queue(GFP_KERNEL);
	if (!q)
		return ERR_PTR(-ENOMEM);
	q->cmd_size = sizeof(struct bsg_job) + dd_job_size;
	q->init_rq_fn = bsg_init_rq;
	q->exit_rq_fn = bsg_exit_rq;
	q->initialize_rq_fn = bsg_initialize_rq;
	q->request_fn = bsg_request_fn;

	ret = blk_init_allocated_queue(q);
	if (ret)
		goto out_cleanup_queue;

	q->queuedata = dev;
	q->bsg_job_fn = job_fn;
	queue_flag_set_unlocked(QUEUE_FLAG_BIDI, q);
	queue_flag_set_unlocked(QUEUE_FLAG_SCSI_PASSTHROUGH, q);
	blk_queue_softirq_done(q, bsg_softirq_done);
	blk_queue_rq_timeout(q, BLK_DEFAULT_SG_TIMEOUT);

	ret = bsg_register_queue(q, dev, name, release);
	if (ret) {
		printk(KERN_ERR "%s: bsg interface failed to "
		       "initialize - register queue\n", dev->kobj.name);
		goto out_cleanup_queue;
	}

	return q;
out_cleanup_queue:
	blk_cleanup_queue(q);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(bsg_setup_queue);
