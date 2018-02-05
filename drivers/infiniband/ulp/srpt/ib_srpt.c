/*
 * Copyright (c) 2006 - 2009 Mellanox Technology Inc.  All rights reserved.
 * Copyright (C) 2008 - 2011 Bart Van Assche <bvanassche@acm.org>.
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * COPYING in the main directory of this source tree, or the
 * OpenIB.org BSD license below:
 *
 *     Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *      - Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *
 *      - Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/kthread.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include <rdma/ib_cache.h>
#include <scsi/scsi_proto.h>
#include <scsi/scsi_tcq.h>
#include <target/target_core_base.h>
#include <target/target_core_fabric.h>
#include "ib_srpt.h"

/* Name of this kernel module. */
#define DRV_NAME		"ib_srpt"
#define DRV_VERSION		"2.0.0"
#define DRV_RELDATE		"2011-02-14"

#define SRPT_ID_STRING	"Linux SRP target"

#undef pr_fmt
#define pr_fmt(fmt) DRV_NAME " " fmt

MODULE_AUTHOR("Vu Pham and Bart Van Assche");
MODULE_DESCRIPTION("InfiniBand SCSI RDMA Protocol target "
		   "v" DRV_VERSION " (" DRV_RELDATE ")");
MODULE_LICENSE("Dual BSD/GPL");

/*
 * Global Variables
 */

static u64 srpt_service_guid;
static DEFINE_SPINLOCK(srpt_dev_lock);	/* Protects srpt_dev_list. */
static LIST_HEAD(srpt_dev_list);	/* List of srpt_device structures. */

static unsigned srp_max_req_size = DEFAULT_MAX_REQ_SIZE;
module_param(srp_max_req_size, int, 0444);
MODULE_PARM_DESC(srp_max_req_size,
		 "Maximum size of SRP request messages in bytes.");

static int srpt_srq_size = DEFAULT_SRPT_SRQ_SIZE;
module_param(srpt_srq_size, int, 0444);
MODULE_PARM_DESC(srpt_srq_size,
		 "Shared receive queue (SRQ) size.");

static int srpt_get_u64_x(char *buffer, const struct kernel_param *kp)
{
	return sprintf(buffer, "0x%016llx", *(u64 *)kp->arg);
}
module_param_call(srpt_service_guid, NULL, srpt_get_u64_x, &srpt_service_guid,
		  0444);
MODULE_PARM_DESC(srpt_service_guid,
		 "Using this value for ioc_guid, id_ext, and cm_listen_id"
		 " instead of using the node_guid of the first HCA.");

static struct ib_client srpt_client;
static void srpt_release_cmd(struct se_cmd *se_cmd);
static void srpt_free_ch(struct kref *kref);
static int srpt_queue_status(struct se_cmd *cmd);
static void srpt_recv_done(struct ib_cq *cq, struct ib_wc *wc);
static void srpt_send_done(struct ib_cq *cq, struct ib_wc *wc);
static void srpt_process_wait_list(struct srpt_rdma_ch *ch);

/*
 * The only allowed channel state changes are those that change the channel
 * state into a state with a higher numerical value. Hence the new > prev test.
 */
static bool srpt_set_ch_state(struct srpt_rdma_ch *ch, enum rdma_ch_state new)
{
	unsigned long flags;
	enum rdma_ch_state prev;
	bool changed = false;

	spin_lock_irqsave(&ch->spinlock, flags);
	prev = ch->state;
	if (new > prev) {
		ch->state = new;
		changed = true;
	}
	spin_unlock_irqrestore(&ch->spinlock, flags);

	return changed;
}

/**
 * srpt_event_handler - asynchronous IB event callback function
 * @handler: IB event handler registered by ib_register_event_handler().
 * @event: Description of the event that occurred.
 *
 * Callback function called by the InfiniBand core when an asynchronous IB
 * event occurs. This callback may occur in interrupt context. See also
 * section 11.5.2, Set Asynchronous Event Handler in the InfiniBand
 * Architecture Specification.
 */
static void srpt_event_handler(struct ib_event_handler *handler,
			       struct ib_event *event)
{
	struct srpt_device *sdev;
	struct srpt_port *sport;
	u8 port_num;

	sdev = ib_get_client_data(event->device, &srpt_client);
	if (!sdev || sdev->device != event->device)
		return;

	pr_debug("ASYNC event= %d on device= %s\n", event->event,
		 sdev->device->name);

	switch (event->event) {
	case IB_EVENT_PORT_ERR:
		port_num = event->element.port_num - 1;
		if (port_num < sdev->device->phys_port_cnt) {
			sport = &sdev->port[port_num];
			sport->lid = 0;
			sport->sm_lid = 0;
		} else {
			WARN(true, "event %d: port_num %d out of range 1..%d\n",
			     event->event, port_num + 1,
			     sdev->device->phys_port_cnt);
		}
		break;
	case IB_EVENT_PORT_ACTIVE:
	case IB_EVENT_LID_CHANGE:
	case IB_EVENT_PKEY_CHANGE:
	case IB_EVENT_SM_CHANGE:
	case IB_EVENT_CLIENT_REREGISTER:
	case IB_EVENT_GID_CHANGE:
		/* Refresh port data asynchronously. */
		port_num = event->element.port_num - 1;
		if (port_num < sdev->device->phys_port_cnt) {
			sport = &sdev->port[port_num];
			if (!sport->lid && !sport->sm_lid)
				schedule_work(&sport->work);
		} else {
			WARN(true, "event %d: port_num %d out of range 1..%d\n",
			     event->event, port_num + 1,
			     sdev->device->phys_port_cnt);
		}
		break;
	default:
		pr_err("received unrecognized IB event %d\n", event->event);
		break;
	}
}

/**
 * srpt_srq_event - SRQ event callback function
 * @event: Description of the event that occurred.
 * @ctx: Context pointer specified at SRQ creation time.
 */
static void srpt_srq_event(struct ib_event *event, void *ctx)
{
	pr_debug("SRQ event %d\n", event->event);
}

static const char *get_ch_state_name(enum rdma_ch_state s)
{
	switch (s) {
	case CH_CONNECTING:
		return "connecting";
	case CH_LIVE:
		return "live";
	case CH_DISCONNECTING:
		return "disconnecting";
	case CH_DRAINING:
		return "draining";
	case CH_DISCONNECTED:
		return "disconnected";
	}
	return "???";
}

/**
 * srpt_qp_event - QP event callback function
 * @event: Description of the event that occurred.
 * @ch: SRPT RDMA channel.
 */
static void srpt_qp_event(struct ib_event *event, struct srpt_rdma_ch *ch)
{
	pr_debug("QP event %d on ch=%p sess_name=%s state=%d\n",
		 event->event, ch, ch->sess_name, ch->state);

	switch (event->event) {
	case IB_EVENT_COMM_EST:
		ib_cm_notify(ch->ib_cm.cm_id, event->event);
		break;
	case IB_EVENT_QP_LAST_WQE_REACHED:
		pr_debug("%s-%d, state %s: received Last WQE event.\n",
			 ch->sess_name, ch->qp->qp_num,
			 get_ch_state_name(ch->state));
		break;
	default:
		pr_err("received unrecognized IB QP event %d\n", event->event);
		break;
	}
}

/**
 * srpt_set_ioc - initialize a IOUnitInfo structure
 * @c_list: controller list.
 * @slot: one-based slot number.
 * @value: four-bit value.
 *
 * Copies the lowest four bits of value in element slot of the array of four
 * bit elements called c_list (controller list). The index slot is one-based.
 */
static void srpt_set_ioc(u8 *c_list, u32 slot, u8 value)
{
	u16 id;
	u8 tmp;

	id = (slot - 1) / 2;
	if (slot & 0x1) {
		tmp = c_list[id] & 0xf;
		c_list[id] = (value << 4) | tmp;
	} else {
		tmp = c_list[id] & 0xf0;
		c_list[id] = (value & 0xf) | tmp;
	}
}

/**
 * srpt_get_class_port_info - copy ClassPortInfo to a management datagram
 * @mad: Datagram that will be sent as response to DM_ATTR_CLASS_PORT_INFO.
 *
 * See also section 16.3.3.1 ClassPortInfo in the InfiniBand Architecture
 * Specification.
 */
static void srpt_get_class_port_info(struct ib_dm_mad *mad)
{
	struct ib_class_port_info *cif;

	cif = (struct ib_class_port_info *)mad->data;
	memset(cif, 0, sizeof(*cif));
	cif->base_version = 1;
	cif->class_version = 1;

	ib_set_cpi_resp_time(cif, 20);
	mad->mad_hdr.status = 0;
}

/**
 * srpt_get_iou - write IOUnitInfo to a management datagram
 * @mad: Datagram that will be sent as response to DM_ATTR_IOU_INFO.
 *
 * See also section 16.3.3.3 IOUnitInfo in the InfiniBand Architecture
 * Specification. See also section B.7, table B.6 in the SRP r16a document.
 */
static void srpt_get_iou(struct ib_dm_mad *mad)
{
	struct ib_dm_iou_info *ioui;
	u8 slot;
	int i;

	ioui = (struct ib_dm_iou_info *)mad->data;
	ioui->change_id = cpu_to_be16(1);
	ioui->max_controllers = 16;

	/* set present for slot 1 and empty for the rest */
	srpt_set_ioc(ioui->controller_list, 1, 1);
	for (i = 1, slot = 2; i < 16; i++, slot++)
		srpt_set_ioc(ioui->controller_list, slot, 0);

	mad->mad_hdr.status = 0;
}

/**
 * srpt_get_ioc - write IOControllerprofile to a management datagram
 * @sport: HCA port through which the MAD has been received.
 * @slot: Slot number specified in DM_ATTR_IOC_PROFILE query.
 * @mad: Datagram that will be sent as response to DM_ATTR_IOC_PROFILE.
 *
 * See also section 16.3.3.4 IOControllerProfile in the InfiniBand
 * Architecture Specification. See also section B.7, table B.7 in the SRP
 * r16a document.
 */
static void srpt_get_ioc(struct srpt_port *sport, u32 slot,
			 struct ib_dm_mad *mad)
{
	struct srpt_device *sdev = sport->sdev;
	struct ib_dm_ioc_profile *iocp;
	int send_queue_depth;

	iocp = (struct ib_dm_ioc_profile *)mad->data;

	if (!slot || slot > 16) {
		mad->mad_hdr.status
			= cpu_to_be16(DM_MAD_STATUS_INVALID_FIELD);
		return;
	}

	if (slot > 2) {
		mad->mad_hdr.status
			= cpu_to_be16(DM_MAD_STATUS_NO_IOC);
		return;
	}

	if (sdev->use_srq)
		send_queue_depth = sdev->srq_size;
	else
		send_queue_depth = min(MAX_SRPT_RQ_SIZE,
				       sdev->device->attrs.max_qp_wr);

	memset(iocp, 0, sizeof(*iocp));
	strcpy(iocp->id_string, SRPT_ID_STRING);
	iocp->guid = cpu_to_be64(srpt_service_guid);
	iocp->vendor_id = cpu_to_be32(sdev->device->attrs.vendor_id);
	iocp->device_id = cpu_to_be32(sdev->device->attrs.vendor_part_id);
	iocp->device_version = cpu_to_be16(sdev->device->attrs.hw_ver);
	iocp->subsys_vendor_id = cpu_to_be32(sdev->device->attrs.vendor_id);
	iocp->subsys_device_id = 0x0;
	iocp->io_class = cpu_to_be16(SRP_REV16A_IB_IO_CLASS);
	iocp->io_subclass = cpu_to_be16(SRP_IO_SUBCLASS);
	iocp->protocol = cpu_to_be16(SRP_PROTOCOL);
	iocp->protocol_version = cpu_to_be16(SRP_PROTOCOL_VERSION);
	iocp->send_queue_depth = cpu_to_be16(send_queue_depth);
	iocp->rdma_read_depth = 4;
	iocp->send_size = cpu_to_be32(srp_max_req_size);
	iocp->rdma_size = cpu_to_be32(min(sport->port_attrib.srp_max_rdma_size,
					  1U << 24));
	iocp->num_svc_entries = 1;
	iocp->op_cap_mask = SRP_SEND_TO_IOC | SRP_SEND_FROM_IOC |
		SRP_RDMA_READ_FROM_IOC | SRP_RDMA_WRITE_FROM_IOC;

	mad->mad_hdr.status = 0;
}

/**
 * srpt_get_svc_entries - write ServiceEntries to a management datagram
 * @ioc_guid: I/O controller GUID to use in reply.
 * @slot: I/O controller number.
 * @hi: End of the range of service entries to be specified in the reply.
 * @lo: Start of the range of service entries to be specified in the reply..
 * @mad: Datagram that will be sent as response to DM_ATTR_SVC_ENTRIES.
 *
 * See also section 16.3.3.5 ServiceEntries in the InfiniBand Architecture
 * Specification. See also section B.7, table B.8 in the SRP r16a document.
 */
static void srpt_get_svc_entries(u64 ioc_guid,
				 u16 slot, u8 hi, u8 lo, struct ib_dm_mad *mad)
{
	struct ib_dm_svc_entries *svc_entries;

	WARN_ON(!ioc_guid);

	if (!slot || slot > 16) {
		mad->mad_hdr.status
			= cpu_to_be16(DM_MAD_STATUS_INVALID_FIELD);
		return;
	}

	if (slot > 2 || lo > hi || hi > 1) {
		mad->mad_hdr.status
			= cpu_to_be16(DM_MAD_STATUS_NO_IOC);
		return;
	}

	svc_entries = (struct ib_dm_svc_entries *)mad->data;
	memset(svc_entries, 0, sizeof(*svc_entries));
	svc_entries->service_entries[0].id = cpu_to_be64(ioc_guid);
	snprintf(svc_entries->service_entries[0].name,
		 sizeof(svc_entries->service_entries[0].name),
		 "%s%016llx",
		 SRP_SERVICE_NAME_PREFIX,
		 ioc_guid);

	mad->mad_hdr.status = 0;
}

/**
 * srpt_mgmt_method_get - process a received management datagram
 * @sp:      HCA port through which the MAD has been received.
 * @rq_mad:  received MAD.
 * @rsp_mad: response MAD.
 */
static void srpt_mgmt_method_get(struct srpt_port *sp, struct ib_mad *rq_mad,
				 struct ib_dm_mad *rsp_mad)
{
	u16 attr_id;
	u32 slot;
	u8 hi, lo;

	attr_id = be16_to_cpu(rq_mad->mad_hdr.attr_id);
	switch (attr_id) {
	case DM_ATTR_CLASS_PORT_INFO:
		srpt_get_class_port_info(rsp_mad);
		break;
	case DM_ATTR_IOU_INFO:
		srpt_get_iou(rsp_mad);
		break;
	case DM_ATTR_IOC_PROFILE:
		slot = be32_to_cpu(rq_mad->mad_hdr.attr_mod);
		srpt_get_ioc(sp, slot, rsp_mad);
		break;
	case DM_ATTR_SVC_ENTRIES:
		slot = be32_to_cpu(rq_mad->mad_hdr.attr_mod);
		hi = (u8) ((slot >> 8) & 0xff);
		lo = (u8) (slot & 0xff);
		slot = (u16) ((slot >> 16) & 0xffff);
		srpt_get_svc_entries(srpt_service_guid,
				     slot, hi, lo, rsp_mad);
		break;
	default:
		rsp_mad->mad_hdr.status =
		    cpu_to_be16(DM_MAD_STATUS_UNSUP_METHOD_ATTR);
		break;
	}
}

/**
 * srpt_mad_send_handler - MAD send completion callback
 * @mad_agent: Return value of ib_register_mad_agent().
 * @mad_wc: Work completion reporting that the MAD has been sent.
 */
static void srpt_mad_send_handler(struct ib_mad_agent *mad_agent,
				  struct ib_mad_send_wc *mad_wc)
{
	rdma_destroy_ah(mad_wc->send_buf->ah);
	ib_free_send_mad(mad_wc->send_buf);
}

/**
 * srpt_mad_recv_handler - MAD reception callback function
 * @mad_agent: Return value of ib_register_mad_agent().
 * @send_buf: Not used.
 * @mad_wc: Work completion reporting that a MAD has been received.
 */
static void srpt_mad_recv_handler(struct ib_mad_agent *mad_agent,
				  struct ib_mad_send_buf *send_buf,
				  struct ib_mad_recv_wc *mad_wc)
{
	struct srpt_port *sport = (struct srpt_port *)mad_agent->context;
	struct ib_ah *ah;
	struct ib_mad_send_buf *rsp;
	struct ib_dm_mad *dm_mad;

	if (!mad_wc || !mad_wc->recv_buf.mad)
		return;

	ah = ib_create_ah_from_wc(mad_agent->qp->pd, mad_wc->wc,
				  mad_wc->recv_buf.grh, mad_agent->port_num);
	if (IS_ERR(ah))
		goto err;

	BUILD_BUG_ON(offsetof(struct ib_dm_mad, data) != IB_MGMT_DEVICE_HDR);

	rsp = ib_create_send_mad(mad_agent, mad_wc->wc->src_qp,
				 mad_wc->wc->pkey_index, 0,
				 IB_MGMT_DEVICE_HDR, IB_MGMT_DEVICE_DATA,
				 GFP_KERNEL,
				 IB_MGMT_BASE_VERSION);
	if (IS_ERR(rsp))
		goto err_rsp;

	rsp->ah = ah;

	dm_mad = rsp->mad;
	memcpy(dm_mad, mad_wc->recv_buf.mad, sizeof(*dm_mad));
	dm_mad->mad_hdr.method = IB_MGMT_METHOD_GET_RESP;
	dm_mad->mad_hdr.status = 0;

	switch (mad_wc->recv_buf.mad->mad_hdr.method) {
	case IB_MGMT_METHOD_GET:
		srpt_mgmt_method_get(sport, mad_wc->recv_buf.mad, dm_mad);
		break;
	case IB_MGMT_METHOD_SET:
		dm_mad->mad_hdr.status =
		    cpu_to_be16(DM_MAD_STATUS_UNSUP_METHOD_ATTR);
		break;
	default:
		dm_mad->mad_hdr.status =
		    cpu_to_be16(DM_MAD_STATUS_UNSUP_METHOD);
		break;
	}

	if (!ib_post_send_mad(rsp, NULL)) {
		ib_free_recv_mad(mad_wc);
		/* will destroy_ah & free_send_mad in send completion */
		return;
	}

	ib_free_send_mad(rsp);

err_rsp:
	rdma_destroy_ah(ah);
err:
	ib_free_recv_mad(mad_wc);
}

static int srpt_format_guid(char *buf, unsigned int size, const __be64 *guid)
{
	const __be16 *g = (const __be16 *)guid;

	return snprintf(buf, size, "%04x:%04x:%04x:%04x",
			be16_to_cpu(g[0]), be16_to_cpu(g[1]),
			be16_to_cpu(g[2]), be16_to_cpu(g[3]));
}

/**
 * srpt_refresh_port - configure a HCA port
 * @sport: SRPT HCA port.
 *
 * Enable InfiniBand management datagram processing, update the cached sm_lid,
 * lid and gid values, and register a callback function for processing MADs
 * on the specified port.
 *
 * Note: It is safe to call this function more than once for the same port.
 */
static int srpt_refresh_port(struct srpt_port *sport)
{
	struct ib_mad_reg_req reg_req;
	struct ib_port_modify port_modify;
	struct ib_port_attr port_attr;
	int ret;

	memset(&port_modify, 0, sizeof(port_modify));
	port_modify.set_port_cap_mask = IB_PORT_DEVICE_MGMT_SUP;
	port_modify.clr_port_cap_mask = 0;

	ret = ib_modify_port(sport->sdev->device, sport->port, 0, &port_modify);
	if (ret)
		goto err_mod_port;

	ret = ib_query_port(sport->sdev->device, sport->port, &port_attr);
	if (ret)
		goto err_query_port;

	sport->sm_lid = port_attr.sm_lid;
	sport->lid = port_attr.lid;

	ret = ib_query_gid(sport->sdev->device, sport->port, 0, &sport->gid,
			   NULL);
	if (ret)
		goto err_query_port;

	sport->port_guid_wwn.priv = sport;
	srpt_format_guid(sport->port_guid, sizeof(sport->port_guid),
			 &sport->gid.global.interface_id);
	sport->port_gid_wwn.priv = sport;
	snprintf(sport->port_gid, sizeof(sport->port_gid),
		 "0x%016llx%016llx",
		 be64_to_cpu(sport->gid.global.subnet_prefix),
		 be64_to_cpu(sport->gid.global.interface_id));

	if (!sport->mad_agent) {
		memset(&reg_req, 0, sizeof(reg_req));
		reg_req.mgmt_class = IB_MGMT_CLASS_DEVICE_MGMT;
		reg_req.mgmt_class_version = IB_MGMT_BASE_VERSION;
		set_bit(IB_MGMT_METHOD_GET, reg_req.method_mask);
		set_bit(IB_MGMT_METHOD_SET, reg_req.method_mask);

		sport->mad_agent = ib_register_mad_agent(sport->sdev->device,
							 sport->port,
							 IB_QPT_GSI,
							 &reg_req, 0,
							 srpt_mad_send_handler,
							 srpt_mad_recv_handler,
							 sport, 0);
		if (IS_ERR(sport->mad_agent)) {
			ret = PTR_ERR(sport->mad_agent);
			sport->mad_agent = NULL;
			goto err_query_port;
		}
	}

	return 0;

err_query_port:

	port_modify.set_port_cap_mask = 0;
	port_modify.clr_port_cap_mask = IB_PORT_DEVICE_MGMT_SUP;
	ib_modify_port(sport->sdev->device, sport->port, 0, &port_modify);

err_mod_port:

	return ret;
}

/**
 * srpt_unregister_mad_agent - unregister MAD callback functions
 * @sdev: SRPT HCA pointer.
 *
 * Note: It is safe to call this function more than once for the same device.
 */
static void srpt_unregister_mad_agent(struct srpt_device *sdev)
{
	struct ib_port_modify port_modify = {
		.clr_port_cap_mask = IB_PORT_DEVICE_MGMT_SUP,
	};
	struct srpt_port *sport;
	int i;

	for (i = 1; i <= sdev->device->phys_port_cnt; i++) {
		sport = &sdev->port[i - 1];
		WARN_ON(sport->port != i);
		if (ib_modify_port(sdev->device, i, 0, &port_modify) < 0)
			pr_err("disabling MAD processing failed.\n");
		if (sport->mad_agent) {
			ib_unregister_mad_agent(sport->mad_agent);
			sport->mad_agent = NULL;
		}
	}
}

/**
 * srpt_alloc_ioctx - allocate a SRPT I/O context structure
 * @sdev: SRPT HCA pointer.
 * @ioctx_size: I/O context size.
 * @dma_size: Size of I/O context DMA buffer.
 * @dir: DMA data direction.
 */
static struct srpt_ioctx *srpt_alloc_ioctx(struct srpt_device *sdev,
					   int ioctx_size, int dma_size,
					   enum dma_data_direction dir)
{
	struct srpt_ioctx *ioctx;

	ioctx = kmalloc(ioctx_size, GFP_KERNEL);
	if (!ioctx)
		goto err;

	ioctx->buf = kmalloc(dma_size, GFP_KERNEL);
	if (!ioctx->buf)
		goto err_free_ioctx;

	ioctx->dma = ib_dma_map_single(sdev->device, ioctx->buf, dma_size, dir);
	if (ib_dma_mapping_error(sdev->device, ioctx->dma))
		goto err_free_buf;

	return ioctx;

err_free_buf:
	kfree(ioctx->buf);
err_free_ioctx:
	kfree(ioctx);
err:
	return NULL;
}

/**
 * srpt_free_ioctx - free a SRPT I/O context structure
 * @sdev: SRPT HCA pointer.
 * @ioctx: I/O context pointer.
 * @dma_size: Size of I/O context DMA buffer.
 * @dir: DMA data direction.
 */
static void srpt_free_ioctx(struct srpt_device *sdev, struct srpt_ioctx *ioctx,
			    int dma_size, enum dma_data_direction dir)
{
	if (!ioctx)
		return;

	ib_dma_unmap_single(sdev->device, ioctx->dma, dma_size, dir);
	kfree(ioctx->buf);
	kfree(ioctx);
}

/**
 * srpt_alloc_ioctx_ring - allocate a ring of SRPT I/O context structures
 * @sdev:       Device to allocate the I/O context ring for.
 * @ring_size:  Number of elements in the I/O context ring.
 * @ioctx_size: I/O context size.
 * @dma_size:   DMA buffer size.
 * @dir:        DMA data direction.
 */
static struct srpt_ioctx **srpt_alloc_ioctx_ring(struct srpt_device *sdev,
				int ring_size, int ioctx_size,
				int dma_size, enum dma_data_direction dir)
{
	struct srpt_ioctx **ring;
	int i;

	WARN_ON(ioctx_size != sizeof(struct srpt_recv_ioctx)
		&& ioctx_size != sizeof(struct srpt_send_ioctx));

	ring = kmalloc(ring_size * sizeof(ring[0]), GFP_KERNEL);
	if (!ring)
		goto out;
	for (i = 0; i < ring_size; ++i) {
		ring[i] = srpt_alloc_ioctx(sdev, ioctx_size, dma_size, dir);
		if (!ring[i])
			goto err;
		ring[i]->index = i;
	}
	goto out;

err:
	while (--i >= 0)
		srpt_free_ioctx(sdev, ring[i], dma_size, dir);
	kfree(ring);
	ring = NULL;
out:
	return ring;
}

/**
 * srpt_free_ioctx_ring - free the ring of SRPT I/O context structures
 * @ioctx_ring: I/O context ring to be freed.
 * @sdev: SRPT HCA pointer.
 * @ring_size: Number of ring elements.
 * @dma_size: Size of I/O context DMA buffer.
 * @dir: DMA data direction.
 */
static void srpt_free_ioctx_ring(struct srpt_ioctx **ioctx_ring,
				 struct srpt_device *sdev, int ring_size,
				 int dma_size, enum dma_data_direction dir)
{
	int i;

	if (!ioctx_ring)
		return;

	for (i = 0; i < ring_size; ++i)
		srpt_free_ioctx(sdev, ioctx_ring[i], dma_size, dir);
	kfree(ioctx_ring);
}

/**
 * srpt_set_cmd_state - set the state of a SCSI command
 * @ioctx: Send I/O context.
 * @new: New I/O context state.
 *
 * Does not modify the state of aborted commands. Returns the previous command
 * state.
 */
static enum srpt_command_state srpt_set_cmd_state(struct srpt_send_ioctx *ioctx,
						  enum srpt_command_state new)
{
	enum srpt_command_state previous;

	previous = ioctx->state;
	if (previous != SRPT_STATE_DONE)
		ioctx->state = new;

	return previous;
}

/**
 * srpt_test_and_set_cmd_state - test and set the state of a command
 * @ioctx: Send I/O context.
 * @old: Current I/O context state.
 * @new: New I/O context state.
 *
 * Returns true if and only if the previous command state was equal to 'old'.
 */
static bool srpt_test_and_set_cmd_state(struct srpt_send_ioctx *ioctx,
					enum srpt_command_state old,
					enum srpt_command_state new)
{
	enum srpt_command_state previous;

	WARN_ON(!ioctx);
	WARN_ON(old == SRPT_STATE_DONE);
	WARN_ON(new == SRPT_STATE_NEW);

	previous = ioctx->state;
	if (previous == old)
		ioctx->state = new;

	return previous == old;
}

/**
 * srpt_post_recv - post an IB receive request
 * @sdev: SRPT HCA pointer.
 * @ch: SRPT RDMA channel.
 * @ioctx: Receive I/O context pointer.
 */
static int srpt_post_recv(struct srpt_device *sdev, struct srpt_rdma_ch *ch,
			  struct srpt_recv_ioctx *ioctx)
{
	struct ib_sge list;
	struct ib_recv_wr wr, *bad_wr;

	BUG_ON(!sdev);
	list.addr = ioctx->ioctx.dma;
	list.length = srp_max_req_size;
	list.lkey = sdev->lkey;

	ioctx->ioctx.cqe.done = srpt_recv_done;
	wr.wr_cqe = &ioctx->ioctx.cqe;
	wr.next = NULL;
	wr.sg_list = &list;
	wr.num_sge = 1;

	if (sdev->use_srq)
		return ib_post_srq_recv(sdev->srq, &wr, &bad_wr);
	else
		return ib_post_recv(ch->qp, &wr, &bad_wr);
}

/**
 * srpt_zerolength_write - perform a zero-length RDMA write
 * @ch: SRPT RDMA channel.
 *
 * A quote from the InfiniBand specification: C9-88: For an HCA responder
 * using Reliable Connection service, for each zero-length RDMA READ or WRITE
 * request, the R_Key shall not be validated, even if the request includes
 * Immediate data.
 */
static int srpt_zerolength_write(struct srpt_rdma_ch *ch)
{
	struct ib_send_wr wr, *bad_wr;

	pr_debug("%s-%d: queued zerolength write\n", ch->sess_name,
		 ch->qp->qp_num);

	memset(&wr, 0, sizeof(wr));
	wr.opcode = IB_WR_RDMA_WRITE;
	wr.wr_cqe = &ch->zw_cqe;
	wr.send_flags = IB_SEND_SIGNALED;
	return ib_post_send(ch->qp, &wr, &bad_wr);
}

static void srpt_zerolength_write_done(struct ib_cq *cq, struct ib_wc *wc)
{
	struct srpt_rdma_ch *ch = cq->cq_context;

	pr_debug("%s-%d wc->status %d\n", ch->sess_name, ch->qp->qp_num,
		 wc->status);

	if (wc->status == IB_WC_SUCCESS) {
		srpt_process_wait_list(ch);
	} else {
		if (srpt_set_ch_state(ch, CH_DISCONNECTED))
			schedule_work(&ch->release_work);
		else
			pr_debug("%s-%d: already disconnected.\n",
				 ch->sess_name, ch->qp->qp_num);
	}
}

static int srpt_alloc_rw_ctxs(struct srpt_send_ioctx *ioctx,
		struct srp_direct_buf *db, int nbufs, struct scatterlist **sg,
		unsigned *sg_cnt)
{
	enum dma_data_direction dir = target_reverse_dma_direction(&ioctx->cmd);
	struct srpt_rdma_ch *ch = ioctx->ch;
	struct scatterlist *prev = NULL;
	unsigned prev_nents;
	int ret, i;

	if (nbufs == 1) {
		ioctx->rw_ctxs = &ioctx->s_rw_ctx;
	} else {
		ioctx->rw_ctxs = kmalloc_array(nbufs, sizeof(*ioctx->rw_ctxs),
			GFP_KERNEL);
		if (!ioctx->rw_ctxs)
			return -ENOMEM;
	}

	for (i = ioctx->n_rw_ctx; i < nbufs; i++, db++) {
		struct srpt_rw_ctx *ctx = &ioctx->rw_ctxs[i];
		u64 remote_addr = be64_to_cpu(db->va);
		u32 size = be32_to_cpu(db->len);
		u32 rkey = be32_to_cpu(db->key);

		ret = target_alloc_sgl(&ctx->sg, &ctx->nents, size, false,
				i < nbufs - 1);
		if (ret)
			goto unwind;

		ret = rdma_rw_ctx_init(&ctx->rw, ch->qp, ch->sport->port,
				ctx->sg, ctx->nents, 0, remote_addr, rkey, dir);
		if (ret < 0) {
			target_free_sgl(ctx->sg, ctx->nents);
			goto unwind;
		}

		ioctx->n_rdma += ret;
		ioctx->n_rw_ctx++;

		if (prev) {
			sg_unmark_end(&prev[prev_nents - 1]);
			sg_chain(prev, prev_nents + 1, ctx->sg);
		} else {
			*sg = ctx->sg;
		}

		prev = ctx->sg;
		prev_nents = ctx->nents;

		*sg_cnt += ctx->nents;
	}

	return 0;

unwind:
	while (--i >= 0) {
		struct srpt_rw_ctx *ctx = &ioctx->rw_ctxs[i];

		rdma_rw_ctx_destroy(&ctx->rw, ch->qp, ch->sport->port,
				ctx->sg, ctx->nents, dir);
		target_free_sgl(ctx->sg, ctx->nents);
	}
	if (ioctx->rw_ctxs != &ioctx->s_rw_ctx)
		kfree(ioctx->rw_ctxs);
	return ret;
}

static void srpt_free_rw_ctxs(struct srpt_rdma_ch *ch,
				    struct srpt_send_ioctx *ioctx)
{
	enum dma_data_direction dir = target_reverse_dma_direction(&ioctx->cmd);
	int i;

	for (i = 0; i < ioctx->n_rw_ctx; i++) {
		struct srpt_rw_ctx *ctx = &ioctx->rw_ctxs[i];

		rdma_rw_ctx_destroy(&ctx->rw, ch->qp, ch->sport->port,
				ctx->sg, ctx->nents, dir);
		target_free_sgl(ctx->sg, ctx->nents);
	}

	if (ioctx->rw_ctxs != &ioctx->s_rw_ctx)
		kfree(ioctx->rw_ctxs);
}

static inline void *srpt_get_desc_buf(struct srp_cmd *srp_cmd)
{
	/*
	 * The pointer computations below will only be compiled correctly
	 * if srp_cmd::add_data is declared as s8*, u8*, s8[] or u8[], so check
	 * whether srp_cmd::add_data has been declared as a byte pointer.
	 */
	BUILD_BUG_ON(!__same_type(srp_cmd->add_data[0], (s8)0) &&
		     !__same_type(srp_cmd->add_data[0], (u8)0));

	/*
	 * According to the SRP spec, the lower two bits of the 'ADDITIONAL
	 * CDB LENGTH' field are reserved and the size in bytes of this field
	 * is four times the value specified in bits 3..7. Hence the "& ~3".
	 */
	return srp_cmd->add_data + (srp_cmd->add_cdb_len & ~3);
}

/**
 * srpt_get_desc_tbl - parse the data descriptors of a SRP_CMD request
 * @ioctx: Pointer to the I/O context associated with the request.
 * @srp_cmd: Pointer to the SRP_CMD request data.
 * @dir: Pointer to the variable to which the transfer direction will be
 *   written.
 * @sg: [out] scatterlist allocated for the parsed SRP_CMD.
 * @sg_cnt: [out] length of @sg.
 * @data_len: Pointer to the variable to which the total data length of all
 *   descriptors in the SRP_CMD request will be written.
 *
 * This function initializes ioctx->nrbuf and ioctx->r_bufs.
 *
 * Returns -EINVAL when the SRP_CMD request contains inconsistent descriptors;
 * -ENOMEM when memory allocation fails and zero upon success.
 */
static int srpt_get_desc_tbl(struct srpt_send_ioctx *ioctx,
		struct srp_cmd *srp_cmd, enum dma_data_direction *dir,
		struct scatterlist **sg, unsigned *sg_cnt, u64 *data_len)
{
	BUG_ON(!dir);
	BUG_ON(!data_len);

	/*
	 * The lower four bits of the buffer format field contain the DATA-IN
	 * buffer descriptor format, and the highest four bits contain the
	 * DATA-OUT buffer descriptor format.
	 */
	if (srp_cmd->buf_fmt & 0xf)
		/* DATA-IN: transfer data from target to initiator (read). */
		*dir = DMA_FROM_DEVICE;
	else if (srp_cmd->buf_fmt >> 4)
		/* DATA-OUT: transfer data from initiator to target (write). */
		*dir = DMA_TO_DEVICE;
	else
		*dir = DMA_NONE;

	/* initialize data_direction early as srpt_alloc_rw_ctxs needs it */
	ioctx->cmd.data_direction = *dir;

	if (((srp_cmd->buf_fmt & 0xf) == SRP_DATA_DESC_DIRECT) ||
	    ((srp_cmd->buf_fmt >> 4) == SRP_DATA_DESC_DIRECT)) {
	    	struct srp_direct_buf *db = srpt_get_desc_buf(srp_cmd);

		*data_len = be32_to_cpu(db->len);
		return srpt_alloc_rw_ctxs(ioctx, db, 1, sg, sg_cnt);
	} else if (((srp_cmd->buf_fmt & 0xf) == SRP_DATA_DESC_INDIRECT) ||
		   ((srp_cmd->buf_fmt >> 4) == SRP_DATA_DESC_INDIRECT)) {
		struct srp_indirect_buf *idb = srpt_get_desc_buf(srp_cmd);
		int nbufs = be32_to_cpu(idb->table_desc.len) /
				sizeof(struct srp_direct_buf);

		if (nbufs >
		    (srp_cmd->data_out_desc_cnt + srp_cmd->data_in_desc_cnt)) {
			pr_err("received unsupported SRP_CMD request"
			       " type (%u out + %u in != %u / %zu)\n",
			       srp_cmd->data_out_desc_cnt,
			       srp_cmd->data_in_desc_cnt,
			       be32_to_cpu(idb->table_desc.len),
			       sizeof(struct srp_direct_buf));
			return -EINVAL;
		}

		*data_len = be32_to_cpu(idb->len);
		return srpt_alloc_rw_ctxs(ioctx, idb->desc_list, nbufs,
				sg, sg_cnt);
	} else {
		*data_len = 0;
		return 0;
	}
}

/**
 * srpt_init_ch_qp - initialize queue pair attributes
 * @ch: SRPT RDMA channel.
 * @qp: Queue pair pointer.
 *
 * Initialized the attributes of queue pair 'qp' by allowing local write,
 * remote read and remote write. Also transitions 'qp' to state IB_QPS_INIT.
 */
static int srpt_init_ch_qp(struct srpt_rdma_ch *ch, struct ib_qp *qp)
{
	struct ib_qp_attr *attr;
	int ret;

	attr = kzalloc(sizeof(*attr), GFP_KERNEL);
	if (!attr)
		return -ENOMEM;

	attr->qp_state = IB_QPS_INIT;
	attr->qp_access_flags = IB_ACCESS_LOCAL_WRITE;
	attr->port_num = ch->sport->port;

	ret = ib_find_cached_pkey(ch->sport->sdev->device, ch->sport->port,
				  ch->pkey, &attr->pkey_index);
	if (ret < 0)
		pr_err("Translating pkey %#x failed (%d) - using index 0\n",
		       ch->pkey, ret);

	ret = ib_modify_qp(qp, attr,
			   IB_QP_STATE | IB_QP_ACCESS_FLAGS | IB_QP_PORT |
			   IB_QP_PKEY_INDEX);

	kfree(attr);
	return ret;
}

/**
 * srpt_ch_qp_rtr - change the state of a channel to 'ready to receive' (RTR)
 * @ch: channel of the queue pair.
 * @qp: queue pair to change the state of.
 *
 * Returns zero upon success and a negative value upon failure.
 *
 * Note: currently a struct ib_qp_attr takes 136 bytes on a 64-bit system.
 * If this structure ever becomes larger, it might be necessary to allocate
 * it dynamically instead of on the stack.
 */
static int srpt_ch_qp_rtr(struct srpt_rdma_ch *ch, struct ib_qp *qp)
{
	struct ib_qp_attr qp_attr;
	int attr_mask;
	int ret;

	qp_attr.qp_state = IB_QPS_RTR;
	ret = ib_cm_init_qp_attr(ch->ib_cm.cm_id, &qp_attr, &attr_mask);
	if (ret)
		goto out;

	qp_attr.max_dest_rd_atomic = 4;

	ret = ib_modify_qp(qp, &qp_attr, attr_mask);

out:
	return ret;
}

/**
 * srpt_ch_qp_rts - change the state of a channel to 'ready to send' (RTS)
 * @ch: channel of the queue pair.
 * @qp: queue pair to change the state of.
 *
 * Returns zero upon success and a negative value upon failure.
 *
 * Note: currently a struct ib_qp_attr takes 136 bytes on a 64-bit system.
 * If this structure ever becomes larger, it might be necessary to allocate
 * it dynamically instead of on the stack.
 */
static int srpt_ch_qp_rts(struct srpt_rdma_ch *ch, struct ib_qp *qp)
{
	struct ib_qp_attr qp_attr;
	int attr_mask;
	int ret;

	qp_attr.qp_state = IB_QPS_RTS;
	ret = ib_cm_init_qp_attr(ch->ib_cm.cm_id, &qp_attr, &attr_mask);
	if (ret)
		goto out;

	qp_attr.max_rd_atomic = 4;

	ret = ib_modify_qp(qp, &qp_attr, attr_mask);

out:
	return ret;
}

/**
 * srpt_ch_qp_err - set the channel queue pair state to 'error'
 * @ch: SRPT RDMA channel.
 */
static int srpt_ch_qp_err(struct srpt_rdma_ch *ch)
{
	struct ib_qp_attr qp_attr;

	qp_attr.qp_state = IB_QPS_ERR;
	return ib_modify_qp(ch->qp, &qp_attr, IB_QP_STATE);
}

/**
 * srpt_get_send_ioctx - obtain an I/O context for sending to the initiator
 * @ch: SRPT RDMA channel.
 */
static struct srpt_send_ioctx *srpt_get_send_ioctx(struct srpt_rdma_ch *ch)
{
	struct srpt_send_ioctx *ioctx;
	unsigned long flags;

	BUG_ON(!ch);

	ioctx = NULL;
	spin_lock_irqsave(&ch->spinlock, flags);
	if (!list_empty(&ch->free_list)) {
		ioctx = list_first_entry(&ch->free_list,
					 struct srpt_send_ioctx, free_list);
		list_del(&ioctx->free_list);
	}
	spin_unlock_irqrestore(&ch->spinlock, flags);

	if (!ioctx)
		return ioctx;

	BUG_ON(ioctx->ch != ch);
	ioctx->state = SRPT_STATE_NEW;
	ioctx->n_rdma = 0;
	ioctx->n_rw_ctx = 0;
	ioctx->queue_status_only = false;
	/*
	 * transport_init_se_cmd() does not initialize all fields, so do it
	 * here.
	 */
	memset(&ioctx->cmd, 0, sizeof(ioctx->cmd));
	memset(&ioctx->sense_data, 0, sizeof(ioctx->sense_data));

	return ioctx;
}

/**
 * srpt_abort_cmd - abort a SCSI command
 * @ioctx:   I/O context associated with the SCSI command.
 */
static int srpt_abort_cmd(struct srpt_send_ioctx *ioctx)
{
	enum srpt_command_state state;

	BUG_ON(!ioctx);

	/*
	 * If the command is in a state where the target core is waiting for
	 * the ib_srpt driver, change the state to the next state.
	 */

	state = ioctx->state;
	switch (state) {
	case SRPT_STATE_NEED_DATA:
		ioctx->state = SRPT_STATE_DATA_IN;
		break;
	case SRPT_STATE_CMD_RSP_SENT:
	case SRPT_STATE_MGMT_RSP_SENT:
		ioctx->state = SRPT_STATE_DONE;
		break;
	default:
		WARN_ONCE(true, "%s: unexpected I/O context state %d\n",
			  __func__, state);
		break;
	}

	pr_debug("Aborting cmd with state %d -> %d and tag %lld\n", state,
		 ioctx->state, ioctx->cmd.tag);

	switch (state) {
	case SRPT_STATE_NEW:
	case SRPT_STATE_DATA_IN:
	case SRPT_STATE_MGMT:
	case SRPT_STATE_DONE:
		/*
		 * Do nothing - defer abort processing until
		 * srpt_queue_response() is invoked.
		 */
		break;
	case SRPT_STATE_NEED_DATA:
		pr_debug("tag %#llx: RDMA read error\n", ioctx->cmd.tag);
		transport_generic_request_failure(&ioctx->cmd,
					TCM_CHECK_CONDITION_ABORT_CMD);
		break;
	case SRPT_STATE_CMD_RSP_SENT:
		/*
		 * SRP_RSP sending failed or the SRP_RSP send completion has
		 * not been received in time.
		 */
		transport_generic_free_cmd(&ioctx->cmd, 0);
		break;
	case SRPT_STATE_MGMT_RSP_SENT:
		transport_generic_free_cmd(&ioctx->cmd, 0);
		break;
	default:
		WARN(1, "Unexpected command state (%d)", state);
		break;
	}

	return state;
}

/**
 * srpt_rdma_read_done - RDMA read completion callback
 * @cq: Completion queue.
 * @wc: Work completion.
 *
 * XXX: what is now target_execute_cmd used to be asynchronous, and unmapping
 * the data that has been transferred via IB RDMA had to be postponed until the
 * check_stop_free() callback.  None of this is necessary anymore and needs to
 * be cleaned up.
 */
static void srpt_rdma_read_done(struct ib_cq *cq, struct ib_wc *wc)
{
	struct srpt_rdma_ch *ch = cq->cq_context;
	struct srpt_send_ioctx *ioctx =
		container_of(wc->wr_cqe, struct srpt_send_ioctx, rdma_cqe);

	WARN_ON(ioctx->n_rdma <= 0);
	atomic_add(ioctx->n_rdma, &ch->sq_wr_avail);
	ioctx->n_rdma = 0;

	if (unlikely(wc->status != IB_WC_SUCCESS)) {
		pr_info("RDMA_READ for ioctx 0x%p failed with status %d\n",
			ioctx, wc->status);
		srpt_abort_cmd(ioctx);
		return;
	}

	if (srpt_test_and_set_cmd_state(ioctx, SRPT_STATE_NEED_DATA,
					SRPT_STATE_DATA_IN))
		target_execute_cmd(&ioctx->cmd);
	else
		pr_err("%s[%d]: wrong state = %d\n", __func__,
		       __LINE__, ioctx->state);
}

/**
 * srpt_build_cmd_rsp - build a SRP_RSP response
 * @ch: RDMA channel through which the request has been received.
 * @ioctx: I/O context associated with the SRP_CMD request. The response will
 *   be built in the buffer ioctx->buf points at and hence this function will
 *   overwrite the request data.
 * @tag: tag of the request for which this response is being generated.
 * @status: value for the STATUS field of the SRP_RSP information unit.
 *
 * Returns the size in bytes of the SRP_RSP response.
 *
 * An SRP_RSP response contains a SCSI status or service response. See also
 * section 6.9 in the SRP r16a document for the format of an SRP_RSP
 * response. See also SPC-2 for more information about sense data.
 */
static int srpt_build_cmd_rsp(struct srpt_rdma_ch *ch,
			      struct srpt_send_ioctx *ioctx, u64 tag,
			      int status)
{
	struct srp_rsp *srp_rsp;
	const u8 *sense_data;
	int sense_data_len, max_sense_len;

	/*
	 * The lowest bit of all SAM-3 status codes is zero (see also
	 * paragraph 5.3 in SAM-3).
	 */
	WARN_ON(status & 1);

	srp_rsp = ioctx->ioctx.buf;
	BUG_ON(!srp_rsp);

	sense_data = ioctx->sense_data;
	sense_data_len = ioctx->cmd.scsi_sense_length;
	WARN_ON(sense_data_len > sizeof(ioctx->sense_data));

	memset(srp_rsp, 0, sizeof(*srp_rsp));
	srp_rsp->opcode = SRP_RSP;
	srp_rsp->req_lim_delta =
		cpu_to_be32(1 + atomic_xchg(&ch->req_lim_delta, 0));
	srp_rsp->tag = tag;
	srp_rsp->status = status;

	if (sense_data_len) {
		BUILD_BUG_ON(MIN_MAX_RSP_SIZE <= sizeof(*srp_rsp));
		max_sense_len = ch->max_ti_iu_len - sizeof(*srp_rsp);
		if (sense_data_len > max_sense_len) {
			pr_warn("truncated sense data from %d to %d"
				" bytes\n", sense_data_len, max_sense_len);
			sense_data_len = max_sense_len;
		}

		srp_rsp->flags |= SRP_RSP_FLAG_SNSVALID;
		srp_rsp->sense_data_len = cpu_to_be32(sense_data_len);
		memcpy(srp_rsp + 1, sense_data, sense_data_len);
	}

	return sizeof(*srp_rsp) + sense_data_len;
}

/**
 * srpt_build_tskmgmt_rsp - build a task management response
 * @ch:       RDMA channel through which the request has been received.
 * @ioctx:    I/O context in which the SRP_RSP response will be built.
 * @rsp_code: RSP_CODE that will be stored in the response.
 * @tag:      Tag of the request for which this response is being generated.
 *
 * Returns the size in bytes of the SRP_RSP response.
 *
 * An SRP_RSP response contains a SCSI status or service response. See also
 * section 6.9 in the SRP r16a document for the format of an SRP_RSP
 * response.
 */
static int srpt_build_tskmgmt_rsp(struct srpt_rdma_ch *ch,
				  struct srpt_send_ioctx *ioctx,
				  u8 rsp_code, u64 tag)
{
	struct srp_rsp *srp_rsp;
	int resp_data_len;
	int resp_len;

	resp_data_len = 4;
	resp_len = sizeof(*srp_rsp) + resp_data_len;

	srp_rsp = ioctx->ioctx.buf;
	BUG_ON(!srp_rsp);
	memset(srp_rsp, 0, sizeof(*srp_rsp));

	srp_rsp->opcode = SRP_RSP;
	srp_rsp->req_lim_delta =
		cpu_to_be32(1 + atomic_xchg(&ch->req_lim_delta, 0));
	srp_rsp->tag = tag;

	srp_rsp->flags |= SRP_RSP_FLAG_RSPVALID;
	srp_rsp->resp_data_len = cpu_to_be32(resp_data_len);
	srp_rsp->data[3] = rsp_code;

	return resp_len;
}

static int srpt_check_stop_free(struct se_cmd *cmd)
{
	struct srpt_send_ioctx *ioctx = container_of(cmd,
				struct srpt_send_ioctx, cmd);

	return target_put_sess_cmd(&ioctx->cmd);
}

/**
 * srpt_handle_cmd - process a SRP_CMD information unit
 * @ch: SRPT RDMA channel.
 * @recv_ioctx: Receive I/O context.
 * @send_ioctx: Send I/O context.
 */
static void srpt_handle_cmd(struct srpt_rdma_ch *ch,
			    struct srpt_recv_ioctx *recv_ioctx,
			    struct srpt_send_ioctx *send_ioctx)
{
	struct se_cmd *cmd;
	struct srp_cmd *srp_cmd;
	struct scatterlist *sg = NULL;
	unsigned sg_cnt = 0;
	u64 data_len;
	enum dma_data_direction dir;
	int rc;

	BUG_ON(!send_ioctx);

	srp_cmd = recv_ioctx->ioctx.buf;
	cmd = &send_ioctx->cmd;
	cmd->tag = srp_cmd->tag;

	switch (srp_cmd->task_attr) {
	case SRP_CMD_SIMPLE_Q:
		cmd->sam_task_attr = TCM_SIMPLE_TAG;
		break;
	case SRP_CMD_ORDERED_Q:
	default:
		cmd->sam_task_attr = TCM_ORDERED_TAG;
		break;
	case SRP_CMD_HEAD_OF_Q:
		cmd->sam_task_attr = TCM_HEAD_TAG;
		break;
	case SRP_CMD_ACA:
		cmd->sam_task_attr = TCM_ACA_TAG;
		break;
	}

	rc = srpt_get_desc_tbl(send_ioctx, srp_cmd, &dir, &sg, &sg_cnt,
			&data_len);
	if (rc) {
		if (rc != -EAGAIN) {
			pr_err("0x%llx: parsing SRP descriptor table failed.\n",
			       srp_cmd->tag);
		}
		goto release_ioctx;
	}

	rc = target_submit_cmd_map_sgls(cmd, ch->sess, srp_cmd->cdb,
			       &send_ioctx->sense_data[0],
			       scsilun_to_int(&srp_cmd->lun), data_len,
			       TCM_SIMPLE_TAG, dir, TARGET_SCF_ACK_KREF,
			       sg, sg_cnt, NULL, 0, NULL, 0);
	if (rc != 0) {
		pr_debug("target_submit_cmd() returned %d for tag %#llx\n", rc,
			 srp_cmd->tag);
		goto release_ioctx;
	}
	return;

release_ioctx:
	send_ioctx->state = SRPT_STATE_DONE;
	srpt_release_cmd(cmd);
}

static int srp_tmr_to_tcm(int fn)
{
	switch (fn) {
	case SRP_TSK_ABORT_TASK:
		return TMR_ABORT_TASK;
	case SRP_TSK_ABORT_TASK_SET:
		return TMR_ABORT_TASK_SET;
	case SRP_TSK_CLEAR_TASK_SET:
		return TMR_CLEAR_TASK_SET;
	case SRP_TSK_LUN_RESET:
		return TMR_LUN_RESET;
	case SRP_TSK_CLEAR_ACA:
		return TMR_CLEAR_ACA;
	default:
		return -1;
	}
}

/**
 * srpt_handle_tsk_mgmt - process a SRP_TSK_MGMT information unit
 * @ch: SRPT RDMA channel.
 * @recv_ioctx: Receive I/O context.
 * @send_ioctx: Send I/O context.
 *
 * Returns 0 if and only if the request will be processed by the target core.
 *
 * For more information about SRP_TSK_MGMT information units, see also section
 * 6.7 in the SRP r16a document.
 */
static void srpt_handle_tsk_mgmt(struct srpt_rdma_ch *ch,
				 struct srpt_recv_ioctx *recv_ioctx,
				 struct srpt_send_ioctx *send_ioctx)
{
	struct srp_tsk_mgmt *srp_tsk;
	struct se_cmd *cmd;
	struct se_session *sess = ch->sess;
	int tcm_tmr;
	int rc;

	BUG_ON(!send_ioctx);

	srp_tsk = recv_ioctx->ioctx.buf;
	cmd = &send_ioctx->cmd;

	pr_debug("recv tsk_mgmt fn %d for task_tag %lld and cmd tag %lld ch %p sess %p\n",
		 srp_tsk->tsk_mgmt_func, srp_tsk->task_tag, srp_tsk->tag, ch,
		 ch->sess);

	srpt_set_cmd_state(send_ioctx, SRPT_STATE_MGMT);
	send_ioctx->cmd.tag = srp_tsk->tag;
	tcm_tmr = srp_tmr_to_tcm(srp_tsk->tsk_mgmt_func);
	rc = target_submit_tmr(&send_ioctx->cmd, sess, NULL,
			       scsilun_to_int(&srp_tsk->lun), srp_tsk, tcm_tmr,
			       GFP_KERNEL, srp_tsk->task_tag,
			       TARGET_SCF_ACK_KREF);
	if (rc != 0) {
		send_ioctx->cmd.se_tmr_req->response = TMR_FUNCTION_REJECTED;
		goto fail;
	}
	return;
fail:
	transport_send_check_condition_and_sense(cmd, 0, 0); // XXX:
}

/**
 * srpt_handle_new_iu - process a newly received information unit
 * @ch:    RDMA channel through which the information unit has been received.
 * @recv_ioctx: Receive I/O context associated with the information unit.
 */
static bool
srpt_handle_new_iu(struct srpt_rdma_ch *ch, struct srpt_recv_ioctx *recv_ioctx)
{
	struct srpt_send_ioctx *send_ioctx = NULL;
	struct srp_cmd *srp_cmd;
	bool res = false;
	u8 opcode;

	BUG_ON(!ch);
	BUG_ON(!recv_ioctx);

	if (unlikely(ch->state == CH_CONNECTING))
		goto push;

	ib_dma_sync_single_for_cpu(ch->sport->sdev->device,
				   recv_ioctx->ioctx.dma, srp_max_req_size,
				   DMA_FROM_DEVICE);

	srp_cmd = recv_ioctx->ioctx.buf;
	opcode = srp_cmd->opcode;
	if (opcode == SRP_CMD || opcode == SRP_TSK_MGMT) {
		send_ioctx = srpt_get_send_ioctx(ch);
		if (unlikely(!send_ioctx))
			goto push;
	}

	if (!list_empty(&recv_ioctx->wait_list)) {
		WARN_ON_ONCE(!ch->processing_wait_list);
		list_del_init(&recv_ioctx->wait_list);
	}

	switch (opcode) {
	case SRP_CMD:
		srpt_handle_cmd(ch, recv_ioctx, send_ioctx);
		break;
	case SRP_TSK_MGMT:
		srpt_handle_tsk_mgmt(ch, recv_ioctx, send_ioctx);
		break;
	case SRP_I_LOGOUT:
		pr_err("Not yet implemented: SRP_I_LOGOUT\n");
		break;
	case SRP_CRED_RSP:
		pr_debug("received SRP_CRED_RSP\n");
		break;
	case SRP_AER_RSP:
		pr_debug("received SRP_AER_RSP\n");
		break;
	case SRP_RSP:
		pr_err("Received SRP_RSP\n");
		break;
	default:
		pr_err("received IU with unknown opcode 0x%x\n", opcode);
		break;
	}

	srpt_post_recv(ch->sport->sdev, ch, recv_ioctx);
	res = true;

out:
	return res;

push:
	if (list_empty(&recv_ioctx->wait_list)) {
		WARN_ON_ONCE(ch->processing_wait_list);
		list_add_tail(&recv_ioctx->wait_list, &ch->cmd_wait_list);
	}
	goto out;
}

static void srpt_recv_done(struct ib_cq *cq, struct ib_wc *wc)
{
	struct srpt_rdma_ch *ch = cq->cq_context;
	struct srpt_recv_ioctx *ioctx =
		container_of(wc->wr_cqe, struct srpt_recv_ioctx, ioctx.cqe);

	if (wc->status == IB_WC_SUCCESS) {
		int req_lim;

		req_lim = atomic_dec_return(&ch->req_lim);
		if (unlikely(req_lim < 0))
			pr_err("req_lim = %d < 0\n", req_lim);
		srpt_handle_new_iu(ch, ioctx);
	} else {
		pr_info_ratelimited("receiving failed for ioctx %p with status %d\n",
				    ioctx, wc->status);
	}
}

/*
 * This function must be called from the context in which RDMA completions are
 * processed because it accesses the wait list without protection against
 * access from other threads.
 */
static void srpt_process_wait_list(struct srpt_rdma_ch *ch)
{
	struct srpt_recv_ioctx *recv_ioctx, *tmp;

	WARN_ON_ONCE(ch->state == CH_CONNECTING);

	if (list_empty(&ch->cmd_wait_list))
		return;

	WARN_ON_ONCE(ch->processing_wait_list);
	ch->processing_wait_list = true;
	list_for_each_entry_safe(recv_ioctx, tmp, &ch->cmd_wait_list,
				 wait_list) {
		if (!srpt_handle_new_iu(ch, recv_ioctx))
			break;
	}
	ch->processing_wait_list = false;
}

/**
 * srpt_send_done - send completion callback
 * @cq: Completion queue.
 * @wc: Work completion.
 *
 * Note: Although this has not yet been observed during tests, at least in
 * theory it is possible that the srpt_get_send_ioctx() call invoked by
 * srpt_handle_new_iu() fails. This is possible because the req_lim_delta
 * value in each response is set to one, and it is possible that this response
 * makes the initiator send a new request before the send completion for that
 * response has been processed. This could e.g. happen if the call to
 * srpt_put_send_iotcx() is delayed because of a higher priority interrupt or
 * if IB retransmission causes generation of the send completion to be
 * delayed. Incoming information units for which srpt_get_send_ioctx() fails
 * are queued on cmd_wait_list. The code below processes these delayed
 * requests one at a time.
 */
static void srpt_send_done(struct ib_cq *cq, struct ib_wc *wc)
{
	struct srpt_rdma_ch *ch = cq->cq_context;
	struct srpt_send_ioctx *ioctx =
		container_of(wc->wr_cqe, struct srpt_send_ioctx, ioctx.cqe);
	enum srpt_command_state state;

	state = srpt_set_cmd_state(ioctx, SRPT_STATE_DONE);

	WARN_ON(state != SRPT_STATE_CMD_RSP_SENT &&
		state != SRPT_STATE_MGMT_RSP_SENT);

	atomic_add(1 + ioctx->n_rdma, &ch->sq_wr_avail);

	if (wc->status != IB_WC_SUCCESS)
		pr_info("sending response for ioctx 0x%p failed"
			" with status %d\n", ioctx, wc->status);

	if (state != SRPT_STATE_DONE) {
		transport_generic_free_cmd(&ioctx->cmd, 0);
	} else {
		pr_err("IB completion has been received too late for"
		       " wr_id = %u.\n", ioctx->ioctx.index);
	}

	srpt_process_wait_list(ch);
}

/**
 * srpt_create_ch_ib - create receive and send completion queues
 * @ch: SRPT RDMA channel.
 */
static int srpt_create_ch_ib(struct srpt_rdma_ch *ch)
{
	struct ib_qp_init_attr *qp_init;
	struct srpt_port *sport = ch->sport;
	struct srpt_device *sdev = sport->sdev;
	const struct ib_device_attr *attrs = &sdev->device->attrs;
	int sq_size = sport->port_attrib.srp_sq_size;
	int i, ret;

	WARN_ON(ch->rq_size < 1);

	ret = -ENOMEM;
	qp_init = kzalloc(sizeof(*qp_init), GFP_KERNEL);
	if (!qp_init)
		goto out;

retry:
	ch->cq = ib_alloc_cq(sdev->device, ch, ch->rq_size + sq_size,
			0 /* XXX: spread CQs */, IB_POLL_WORKQUEUE);
	if (IS_ERR(ch->cq)) {
		ret = PTR_ERR(ch->cq);
		pr_err("failed to create CQ cqe= %d ret= %d\n",
		       ch->rq_size + sq_size, ret);
		goto out;
	}

	qp_init->qp_context = (void *)ch;
	qp_init->event_handler
		= (void(*)(struct ib_event *, void*))srpt_qp_event;
	qp_init->send_cq = ch->cq;
	qp_init->recv_cq = ch->cq;
	qp_init->sq_sig_type = IB_SIGNAL_REQ_WR;
	qp_init->qp_type = IB_QPT_RC;
	/*
	 * We divide up our send queue size into half SEND WRs to send the
	 * completions, and half R/W contexts to actually do the RDMA
	 * READ/WRITE transfers.  Note that we need to allocate CQ slots for
	 * both both, as RDMA contexts will also post completions for the
	 * RDMA READ case.
	 */
	qp_init->cap.max_send_wr = min(sq_size / 2, attrs->max_qp_wr);
	qp_init->cap.max_rdma_ctxs = sq_size / 2;
	qp_init->cap.max_send_sge = min(attrs->max_sge, SRPT_MAX_SG_PER_WQE);
	qp_init->port_num = ch->sport->port;
	if (sdev->use_srq) {
		qp_init->srq = sdev->srq;
	} else {
		qp_init->cap.max_recv_wr = ch->rq_size;
		qp_init->cap.max_recv_sge = qp_init->cap.max_send_sge;
	}

	ch->qp = ib_create_qp(sdev->pd, qp_init);
	if (IS_ERR(ch->qp)) {
		ret = PTR_ERR(ch->qp);
		if (ret == -ENOMEM) {
			sq_size /= 2;
			if (sq_size >= MIN_SRPT_SQ_SIZE) {
				ib_destroy_cq(ch->cq);
				goto retry;
			}
		}
		pr_err("failed to create_qp ret= %d\n", ret);
		goto err_destroy_cq;
	}

	atomic_set(&ch->sq_wr_avail, qp_init->cap.max_send_wr);

	pr_debug("%s: max_cqe= %d max_sge= %d sq_size = %d ch= %p\n",
		 __func__, ch->cq->cqe, qp_init->cap.max_send_sge,
		 qp_init->cap.max_send_wr, ch);

	ret = srpt_init_ch_qp(ch, ch->qp);
	if (ret)
		goto err_destroy_qp;

	if (!sdev->use_srq)
		for (i = 0; i < ch->rq_size; i++)
			srpt_post_recv(sdev, ch, ch->ioctx_recv_ring[i]);

out:
	kfree(qp_init);
	return ret;

err_destroy_qp:
	ib_destroy_qp(ch->qp);
err_destroy_cq:
	ib_free_cq(ch->cq);
	goto out;
}

static void srpt_destroy_ch_ib(struct srpt_rdma_ch *ch)
{
	ib_destroy_qp(ch->qp);
	ib_free_cq(ch->cq);
}

/**
 * srpt_close_ch - close a RDMA channel
 * @ch: SRPT RDMA channel.
 *
 * Make sure all resources associated with the channel will be deallocated at
 * an appropriate time.
 *
 * Returns true if and only if the channel state has been modified into
 * CH_DRAINING.
 */
static bool srpt_close_ch(struct srpt_rdma_ch *ch)
{
	int ret;

	if (!srpt_set_ch_state(ch, CH_DRAINING)) {
		pr_debug("%s-%d: already closed\n", ch->sess_name,
			 ch->qp->qp_num);
		return false;
	}

	kref_get(&ch->kref);

	ret = srpt_ch_qp_err(ch);
	if (ret < 0)
		pr_err("%s-%d: changing queue pair into error state failed: %d\n",
		       ch->sess_name, ch->qp->qp_num, ret);

	ret = srpt_zerolength_write(ch);
	if (ret < 0) {
		pr_err("%s-%d: queuing zero-length write failed: %d\n",
		       ch->sess_name, ch->qp->qp_num, ret);
		if (srpt_set_ch_state(ch, CH_DISCONNECTED))
			schedule_work(&ch->release_work);
		else
			WARN_ON_ONCE(true);
	}

	kref_put(&ch->kref, srpt_free_ch);

	return true;
}

/*
 * Change the channel state into CH_DISCONNECTING. If a channel has not yet
 * reached the connected state, close it. If a channel is in the connected
 * state, send a DREQ. If a DREQ has been received, send a DREP. Note: it is
 * the responsibility of the caller to ensure that this function is not
 * invoked concurrently with the code that accepts a connection. This means
 * that this function must either be invoked from inside a CM callback
 * function or that it must be invoked with the srpt_port.mutex held.
 */
static int srpt_disconnect_ch(struct srpt_rdma_ch *ch)
{
	int ret;

	if (!srpt_set_ch_state(ch, CH_DISCONNECTING))
		return -ENOTCONN;

	ret = ib_send_cm_dreq(ch->ib_cm.cm_id, NULL, 0);
	if (ret < 0)
		ret = ib_send_cm_drep(ch->ib_cm.cm_id, NULL, 0);

	if (ret < 0 && srpt_close_ch(ch))
		ret = 0;

	return ret;
}

static bool srpt_ch_closed(struct srpt_port *sport, struct srpt_rdma_ch *ch)
{
	struct srpt_nexus *nexus;
	struct srpt_rdma_ch *ch2;
	bool res = true;

	rcu_read_lock();
	list_for_each_entry(nexus, &sport->nexus_list, entry) {
		list_for_each_entry(ch2, &nexus->ch_list, list) {
			if (ch2 == ch) {
				res = false;
				goto done;
			}
		}
	}
done:
	rcu_read_unlock();

	return res;
}

/* Send DREQ and wait for DREP. */
static void srpt_disconnect_ch_sync(struct srpt_rdma_ch *ch)
{
	struct srpt_port *sport = ch->sport;

	pr_debug("ch %s-%d state %d\n", ch->sess_name, ch->qp->qp_num,
		 ch->state);

	mutex_lock(&sport->mutex);
	srpt_disconnect_ch(ch);
	mutex_unlock(&sport->mutex);

	while (wait_event_timeout(sport->ch_releaseQ, srpt_ch_closed(sport, ch),
				  5 * HZ) == 0)
		pr_info("%s(%s-%d state %d): still waiting ...\n", __func__,
			ch->sess_name, ch->qp->qp_num, ch->state);

}

static void __srpt_close_all_ch(struct srpt_port *sport)
{
	struct srpt_nexus *nexus;
	struct srpt_rdma_ch *ch;

	lockdep_assert_held(&sport->mutex);

	list_for_each_entry(nexus, &sport->nexus_list, entry) {
		list_for_each_entry(ch, &nexus->ch_list, list) {
			if (srpt_disconnect_ch(ch) >= 0)
				pr_info("Closing channel %s-%d because target %s_%d has been disabled\n",
					ch->sess_name, ch->qp->qp_num,
					sport->sdev->device->name, sport->port);
			srpt_close_ch(ch);
		}
	}
}

/*
 * Look up (i_port_id, t_port_id) in sport->nexus_list. Create an entry if
 * it does not yet exist.
 */
static struct srpt_nexus *srpt_get_nexus(struct srpt_port *sport,
					 const u8 i_port_id[16],
					 const u8 t_port_id[16])
{
	struct srpt_nexus *nexus = NULL, *tmp_nexus = NULL, *n;

	for (;;) {
		mutex_lock(&sport->mutex);
		list_for_each_entry(n, &sport->nexus_list, entry) {
			if (memcmp(n->i_port_id, i_port_id, 16) == 0 &&
			    memcmp(n->t_port_id, t_port_id, 16) == 0) {
				nexus = n;
				break;
			}
		}
		if (!nexus && tmp_nexus) {
			list_add_tail_rcu(&tmp_nexus->entry,
					  &sport->nexus_list);
			swap(nexus, tmp_nexus);
		}
		mutex_unlock(&sport->mutex);

		if (nexus)
			break;
		tmp_nexus = kzalloc(sizeof(*nexus), GFP_KERNEL);
		if (!tmp_nexus) {
			nexus = ERR_PTR(-ENOMEM);
			break;
		}
		INIT_LIST_HEAD(&tmp_nexus->ch_list);
		memcpy(tmp_nexus->i_port_id, i_port_id, 16);
		memcpy(tmp_nexus->t_port_id, t_port_id, 16);
	}

	kfree(tmp_nexus);

	return nexus;
}

static void srpt_set_enabled(struct srpt_port *sport, bool enabled)
	__must_hold(&sport->mutex)
{
	lockdep_assert_held(&sport->mutex);

	if (sport->enabled == enabled)
		return;
	sport->enabled = enabled;
	if (!enabled)
		__srpt_close_all_ch(sport);
}

static void srpt_free_ch(struct kref *kref)
{
	struct srpt_rdma_ch *ch = container_of(kref, struct srpt_rdma_ch, kref);

	kfree_rcu(ch, rcu);
}

static void srpt_release_channel_work(struct work_struct *w)
{
	struct srpt_rdma_ch *ch;
	struct srpt_device *sdev;
	struct srpt_port *sport;
	struct se_session *se_sess;

	ch = container_of(w, struct srpt_rdma_ch, release_work);
	pr_debug("%s-%d\n", ch->sess_name, ch->qp->qp_num);

	sdev = ch->sport->sdev;
	BUG_ON(!sdev);

	se_sess = ch->sess;
	BUG_ON(!se_sess);

	target_sess_cmd_list_set_waiting(se_sess);
	target_wait_for_sess_cmds(se_sess);

	transport_deregister_session_configfs(se_sess);
	transport_deregister_session(se_sess);
	ch->sess = NULL;

	ib_destroy_cm_id(ch->ib_cm.cm_id);

	srpt_destroy_ch_ib(ch);

	srpt_free_ioctx_ring((struct srpt_ioctx **)ch->ioctx_ring,
			     ch->sport->sdev, ch->rq_size,
			     ch->max_rsp_size, DMA_TO_DEVICE);

	srpt_free_ioctx_ring((struct srpt_ioctx **)ch->ioctx_recv_ring,
			     sdev, ch->rq_size,
			     srp_max_req_size, DMA_FROM_DEVICE);

	sport = ch->sport;
	mutex_lock(&sport->mutex);
	list_del_rcu(&ch->list);
	mutex_unlock(&sport->mutex);

	wake_up(&sport->ch_releaseQ);

	kref_put(&ch->kref, srpt_free_ch);
}

/**
 * srpt_cm_req_recv - process the event IB_CM_REQ_RECEIVED
 * @cm_id: IB/CM connection identifier.
 * @port_num: Port through which the IB/CM REQ message was received.
 * @pkey: P_Key of the incoming connection.
 * @req: SRP login request.
 * @src_addr: GID of the port that submitted the login request.
 *
 * Ownership of the cm_id is transferred to the target session if this
 * functions returns zero. Otherwise the caller remains the owner of cm_id.
 */
static int srpt_cm_req_recv(struct ib_cm_id *cm_id,
			    u8 port_num, __be16 pkey,
			    const struct srp_login_req *req,
			    const char *src_addr)
{
	struct srpt_device *sdev = cm_id->context;
	struct srpt_port *sport = &sdev->port[port_num - 1];
	struct srpt_nexus *nexus;
	struct srp_login_rsp *rsp = NULL;
	struct srp_login_rej *rej = NULL;
	struct ib_cm_rep_param *rep_param = NULL;
	struct srpt_rdma_ch *ch;
	char i_port_id[36];
	u32 it_iu_len;
	int i, ret;

	WARN_ON_ONCE(irqs_disabled());

	if (WARN_ON(!sdev || !req))
		return -EINVAL;

	it_iu_len = be32_to_cpu(req->req_it_iu_len);

	pr_info("Received SRP_LOGIN_REQ with i_port_id %pI6, t_port_id %pI6 and it_iu_len %d on port %d (guid=%pI6); pkey %#04x\n",
		req->initiator_port_id, req->target_port_id, it_iu_len,
		port_num, &sport->gid, be16_to_cpu(pkey));

	nexus = srpt_get_nexus(sport, req->initiator_port_id,
			       req->target_port_id);
	if (IS_ERR(nexus)) {
		ret = PTR_ERR(nexus);
		goto out;
	}

	ret = -ENOMEM;
	rsp = kzalloc(sizeof(*rsp), GFP_KERNEL);
	rej = kzalloc(sizeof(*rej), GFP_KERNEL);
	rep_param = kzalloc(sizeof(*rep_param), GFP_KERNEL);
	if (!rsp || !rej || !rep_param)
		goto out;

	ret = -EINVAL;
	if (it_iu_len > srp_max_req_size || it_iu_len < 64) {
		rej->reason = cpu_to_be32(
				SRP_LOGIN_REJ_REQ_IT_IU_LENGTH_TOO_LARGE);
		pr_err("rejected SRP_LOGIN_REQ because its length (%d bytes) is out of range (%d .. %d)\n",
		       it_iu_len, 64, srp_max_req_size);
		goto reject;
	}

	if (!sport->enabled) {
		rej->reason = cpu_to_be32(SRP_LOGIN_REJ_INSUFFICIENT_RESOURCES);
		pr_info("rejected SRP_LOGIN_REQ because target port %s_%d has not yet been enabled\n",
			sport->sdev->device->name, port_num);
		goto reject;
	}

	if (*(__be64 *)req->target_port_id != cpu_to_be64(srpt_service_guid)
	    || *(__be64 *)(req->target_port_id + 8) !=
	       cpu_to_be64(srpt_service_guid)) {
		rej->reason = cpu_to_be32(
				SRP_LOGIN_REJ_UNABLE_ASSOCIATE_CHANNEL);
		pr_err("rejected SRP_LOGIN_REQ because it has an invalid target port identifier.\n");
		goto reject;
	}

	ret = -ENOMEM;
	ch = kzalloc(sizeof(*ch), GFP_KERNEL);
	if (!ch) {
		rej->reason = cpu_to_be32(SRP_LOGIN_REJ_INSUFFICIENT_RESOURCES);
		pr_err("rejected SRP_LOGIN_REQ because out of memory.\n");
		goto reject;
	}

	kref_init(&ch->kref);
	ch->pkey = be16_to_cpu(pkey);
	ch->nexus = nexus;
	ch->zw_cqe.done = srpt_zerolength_write_done;
	INIT_WORK(&ch->release_work, srpt_release_channel_work);
	ch->sport = sport;
	ch->ib_cm.cm_id = cm_id;
	cm_id->context = ch;
	/*
	 * ch->rq_size should be at least as large as the initiator queue
	 * depth to avoid that the initiator driver has to report QUEUE_FULL
	 * to the SCSI mid-layer.
	 */
	ch->rq_size = min(MAX_SRPT_RQ_SIZE, sdev->device->attrs.max_qp_wr);
	spin_lock_init(&ch->spinlock);
	ch->state = CH_CONNECTING;
	INIT_LIST_HEAD(&ch->cmd_wait_list);
	ch->max_rsp_size = ch->sport->port_attrib.srp_max_rsp_size;

	ch->ioctx_ring = (struct srpt_send_ioctx **)
		srpt_alloc_ioctx_ring(ch->sport->sdev, ch->rq_size,
				      sizeof(*ch->ioctx_ring[0]),
				      ch->max_rsp_size, DMA_TO_DEVICE);
	if (!ch->ioctx_ring) {
		pr_err("rejected SRP_LOGIN_REQ because creating a new QP SQ ring failed.\n");
		rej->reason = cpu_to_be32(SRP_LOGIN_REJ_INSUFFICIENT_RESOURCES);
		goto free_ch;
	}

	INIT_LIST_HEAD(&ch->free_list);
	for (i = 0; i < ch->rq_size; i++) {
		ch->ioctx_ring[i]->ch = ch;
		list_add_tail(&ch->ioctx_ring[i]->free_list, &ch->free_list);
	}
	if (!sdev->use_srq) {
		ch->ioctx_recv_ring = (struct srpt_recv_ioctx **)
			srpt_alloc_ioctx_ring(ch->sport->sdev, ch->rq_size,
					      sizeof(*ch->ioctx_recv_ring[0]),
					      srp_max_req_size,
					      DMA_FROM_DEVICE);
		if (!ch->ioctx_recv_ring) {
			pr_err("rejected SRP_LOGIN_REQ because creating a new QP RQ ring failed.\n");
			rej->reason =
			    cpu_to_be32(SRP_LOGIN_REJ_INSUFFICIENT_RESOURCES);
			goto free_ring;
		}
		for (i = 0; i < ch->rq_size; i++)
			INIT_LIST_HEAD(&ch->ioctx_recv_ring[i]->wait_list);
	}

	ret = srpt_create_ch_ib(ch);
	if (ret) {
		rej->reason = cpu_to_be32(SRP_LOGIN_REJ_INSUFFICIENT_RESOURCES);
		pr_err("rejected SRP_LOGIN_REQ because creating a new RDMA channel failed.\n");
		goto free_recv_ring;
	}

	strlcpy(ch->sess_name, src_addr, sizeof(ch->sess_name));
	snprintf(i_port_id, sizeof(i_port_id), "0x%016llx%016llx",
			be64_to_cpu(*(__be64 *)nexus->i_port_id),
			be64_to_cpu(*(__be64 *)(nexus->i_port_id + 8)));

	pr_debug("registering session %s\n", ch->sess_name);

	if (sport->port_guid_tpg.se_tpg_wwn)
		ch->sess = target_alloc_session(&sport->port_guid_tpg, 0, 0,
						TARGET_PROT_NORMAL,
						ch->sess_name, ch, NULL);
	if (sport->port_gid_tpg.se_tpg_wwn && IS_ERR_OR_NULL(ch->sess))
		ch->sess = target_alloc_session(&sport->port_gid_tpg, 0, 0,
					TARGET_PROT_NORMAL, i_port_id, ch,
					NULL);
	/* Retry without leading "0x" */
	if (sport->port_gid_tpg.se_tpg_wwn && IS_ERR_OR_NULL(ch->sess))
		ch->sess = target_alloc_session(&sport->port_gid_tpg, 0, 0,
						TARGET_PROT_NORMAL,
						i_port_id + 2, ch, NULL);
	if (IS_ERR_OR_NULL(ch->sess)) {
		ret = PTR_ERR(ch->sess);
		pr_info("Rejected login for initiator %s: ret = %d.\n",
			ch->sess_name, ret);
		rej->reason = cpu_to_be32(ret == -ENOMEM ?
				SRP_LOGIN_REJ_INSUFFICIENT_RESOURCES :
				SRP_LOGIN_REJ_CHANNEL_LIMIT_REACHED);
		goto reject;
	}

	mutex_lock(&sport->mutex);

	if ((req->req_flags & SRP_MTCH_ACTION) == SRP_MULTICHAN_SINGLE) {
		struct srpt_rdma_ch *ch2;

		rsp->rsp_flags = SRP_LOGIN_RSP_MULTICHAN_NO_CHAN;

		list_for_each_entry(ch2, &nexus->ch_list, list) {
			if (srpt_disconnect_ch(ch2) < 0)
				continue;
			pr_info("Relogin - closed existing channel %s\n",
				ch2->sess_name);
			rsp->rsp_flags = SRP_LOGIN_RSP_MULTICHAN_TERMINATED;
		}
	} else {
		rsp->rsp_flags = SRP_LOGIN_RSP_MULTICHAN_MAINTAINED;
	}

	list_add_tail_rcu(&ch->list, &nexus->ch_list);

	if (!sport->enabled) {
		rej->reason = cpu_to_be32(
				SRP_LOGIN_REJ_INSUFFICIENT_RESOURCES);
		pr_info("rejected SRP_LOGIN_REQ because target %s_%d is not enabled\n",
			sdev->device->name, port_num);
		mutex_unlock(&sport->mutex);
		goto reject;
	}

	mutex_unlock(&sport->mutex);

	ret = srpt_ch_qp_rtr(ch, ch->qp);
	if (ret) {
		rej->reason = cpu_to_be32(SRP_LOGIN_REJ_INSUFFICIENT_RESOURCES);
		pr_err("rejected SRP_LOGIN_REQ because enabling RTR failed (error code = %d)\n",
		       ret);
		goto destroy_ib;
	}

	pr_debug("Establish connection sess=%p name=%s ch=%p\n", ch->sess,
		 ch->sess_name, ch);

	/* create srp_login_response */
	rsp->opcode = SRP_LOGIN_RSP;
	rsp->tag = req->tag;
	rsp->max_it_iu_len = req->req_it_iu_len;
	rsp->max_ti_iu_len = req->req_it_iu_len;
	ch->max_ti_iu_len = it_iu_len;
	rsp->buf_fmt = cpu_to_be16(SRP_BUF_FORMAT_DIRECT |
				   SRP_BUF_FORMAT_INDIRECT);
	rsp->req_lim_delta = cpu_to_be32(ch->rq_size);
	atomic_set(&ch->req_lim, ch->rq_size);
	atomic_set(&ch->req_lim_delta, 0);

	/* create cm reply */
	rep_param->qp_num = ch->qp->qp_num;
	rep_param->private_data = (void *)rsp;
	rep_param->private_data_len = sizeof(*rsp);
	rep_param->rnr_retry_count = 7;
	rep_param->flow_control = 1;
	rep_param->failover_accepted = 0;
	rep_param->srq = 1;
	rep_param->responder_resources = 4;
	rep_param->initiator_depth = 4;

	/*
	 * Hold the sport mutex while accepting a connection to avoid that
	 * srpt_disconnect_ch() is invoked concurrently with this code.
	 */
	mutex_lock(&sport->mutex);
	if (sport->enabled && ch->state == CH_CONNECTING)
		ret = ib_send_cm_rep(cm_id, rep_param);
	else
		ret = -EINVAL;
	mutex_unlock(&sport->mutex);

	switch (ret) {
	case 0:
		break;
	case -EINVAL:
		goto reject;
	default:
		rej->reason = cpu_to_be32(SRP_LOGIN_REJ_INSUFFICIENT_RESOURCES);
		pr_err("sending SRP_LOGIN_REQ response failed (error code = %d)\n",
		       ret);
		goto reject;
	}

	goto out;

destroy_ib:
	srpt_destroy_ch_ib(ch);

free_recv_ring:
	srpt_free_ioctx_ring((struct srpt_ioctx **)ch->ioctx_recv_ring,
			     ch->sport->sdev, ch->rq_size,
			     srp_max_req_size, DMA_FROM_DEVICE);

free_ring:
	srpt_free_ioctx_ring((struct srpt_ioctx **)ch->ioctx_ring,
			     ch->sport->sdev, ch->rq_size,
			     ch->max_rsp_size, DMA_TO_DEVICE);
free_ch:
	cm_id->context = NULL;
	kfree(ch);
	ch = NULL;

	WARN_ON_ONCE(ret == 0);

reject:
	pr_info("Rejecting login with reason %#x\n", be32_to_cpu(rej->reason));
	rej->opcode = SRP_LOGIN_REJ;
	rej->tag = req->tag;
	rej->buf_fmt = cpu_to_be16(SRP_BUF_FORMAT_DIRECT |
				   SRP_BUF_FORMAT_INDIRECT);

	ib_send_cm_rej(cm_id, IB_CM_REJ_CONSUMER_DEFINED, NULL, 0,
			     (void *)rej, sizeof(*rej));

out:
	kfree(rep_param);
	kfree(rsp);
	kfree(rej);

	return ret;
}

static int srpt_ib_cm_req_recv(struct ib_cm_id *cm_id,
			       struct ib_cm_req_event_param *param,
			       void *private_data)
{
	char sguid[40];

	srpt_format_guid(sguid, sizeof(sguid),
			 &param->primary_path->dgid.global.interface_id);

	return srpt_cm_req_recv(cm_id, param->port, param->primary_path->pkey,
				private_data, sguid);
}

static void srpt_cm_rej_recv(struct srpt_rdma_ch *ch,
			     enum ib_cm_rej_reason reason,
			     const u8 *private_data,
			     u8 private_data_len)
{
	char *priv = NULL;
	int i;

	if (private_data_len && (priv = kmalloc(private_data_len * 3 + 1,
						GFP_KERNEL))) {
		for (i = 0; i < private_data_len; i++)
			sprintf(priv + 3 * i, " %02x", private_data[i]);
	}
	pr_info("Received CM REJ for ch %s-%d; reason %d%s%s.\n",
		ch->sess_name, ch->qp->qp_num, reason, private_data_len ?
		"; private data" : "", priv ? priv : " (?)");
	kfree(priv);
}

/**
 * srpt_cm_rtu_recv - process an IB_CM_RTU_RECEIVED or USER_ESTABLISHED event
 * @ch: SRPT RDMA channel.
 *
 * An IB_CM_RTU_RECEIVED message indicates that the connection is established
 * and that the recipient may begin transmitting (RTU = ready to use).
 */
static void srpt_cm_rtu_recv(struct srpt_rdma_ch *ch)
{
	int ret;

	ret = srpt_ch_qp_rts(ch, ch->qp);
	if (ret < 0) {
		pr_err("%s-%d: QP transition to RTS failed\n", ch->sess_name,
		       ch->qp->qp_num);
		srpt_close_ch(ch);
		return;
	}

	/*
	 * Note: calling srpt_close_ch() if the transition to the LIVE state
	 * fails is not necessary since that means that that function has
	 * already been invoked from another thread.
	 */
	if (!srpt_set_ch_state(ch, CH_LIVE)) {
		pr_err("%s-%d: channel transition to LIVE state failed\n",
		       ch->sess_name, ch->qp->qp_num);
		return;
	}

	/* Trigger wait list processing. */
	ret = srpt_zerolength_write(ch);
	WARN_ONCE(ret < 0, "%d\n", ret);
}

/**
 * srpt_cm_handler - IB connection manager callback function
 * @cm_id: IB/CM connection identifier.
 * @event: IB/CM event.
 *
 * A non-zero return value will cause the caller destroy the CM ID.
 *
 * Note: srpt_cm_handler() must only return a non-zero value when transferring
 * ownership of the cm_id to a channel by srpt_cm_req_recv() failed. Returning
 * a non-zero value in any other case will trigger a race with the
 * ib_destroy_cm_id() call in srpt_release_channel().
 */
static int srpt_cm_handler(struct ib_cm_id *cm_id, struct ib_cm_event *event)
{
	struct srpt_rdma_ch *ch = cm_id->context;
	int ret;

	ret = 0;
	switch (event->event) {
	case IB_CM_REQ_RECEIVED:
		ret = srpt_ib_cm_req_recv(cm_id, &event->param.req_rcvd,
					  event->private_data);
		break;
	case IB_CM_REJ_RECEIVED:
		srpt_cm_rej_recv(ch, event->param.rej_rcvd.reason,
				 event->private_data,
				 IB_CM_REJ_PRIVATE_DATA_SIZE);
		break;
	case IB_CM_RTU_RECEIVED:
	case IB_CM_USER_ESTABLISHED:
		srpt_cm_rtu_recv(ch);
		break;
	case IB_CM_DREQ_RECEIVED:
		srpt_disconnect_ch(ch);
		break;
	case IB_CM_DREP_RECEIVED:
		pr_info("Received CM DREP message for ch %s-%d.\n",
			ch->sess_name, ch->qp->qp_num);
		srpt_close_ch(ch);
		break;
	case IB_CM_TIMEWAIT_EXIT:
		pr_info("Received CM TimeWait exit for ch %s-%d.\n",
			ch->sess_name, ch->qp->qp_num);
		srpt_close_ch(ch);
		break;
	case IB_CM_REP_ERROR:
		pr_info("Received CM REP error for ch %s-%d.\n", ch->sess_name,
			ch->qp->qp_num);
		break;
	case IB_CM_DREQ_ERROR:
		pr_info("Received CM DREQ ERROR event.\n");
		break;
	case IB_CM_MRA_RECEIVED:
		pr_info("Received CM MRA event\n");
		break;
	default:
		pr_err("received unrecognized CM event %d\n", event->event);
		break;
	}

	return ret;
}

static int srpt_write_pending_status(struct se_cmd *se_cmd)
{
	struct srpt_send_ioctx *ioctx;

	ioctx = container_of(se_cmd, struct srpt_send_ioctx, cmd);
	return ioctx->state == SRPT_STATE_NEED_DATA;
}

/*
 * srpt_write_pending - Start data transfer from initiator to target (write).
 */
static int srpt_write_pending(struct se_cmd *se_cmd)
{
	struct srpt_send_ioctx *ioctx =
		container_of(se_cmd, struct srpt_send_ioctx, cmd);
	struct srpt_rdma_ch *ch = ioctx->ch;
	struct ib_send_wr *first_wr = NULL, *bad_wr;
	struct ib_cqe *cqe = &ioctx->rdma_cqe;
	enum srpt_command_state new_state;
	int ret, i;

	new_state = srpt_set_cmd_state(ioctx, SRPT_STATE_NEED_DATA);
	WARN_ON(new_state == SRPT_STATE_DONE);

	if (atomic_sub_return(ioctx->n_rdma, &ch->sq_wr_avail) < 0) {
		pr_warn("%s: IB send queue full (needed %d)\n",
				__func__, ioctx->n_rdma);
		ret = -ENOMEM;
		goto out_undo;
	}

	cqe->done = srpt_rdma_read_done;
	for (i = ioctx->n_rw_ctx - 1; i >= 0; i--) {
		struct srpt_rw_ctx *ctx = &ioctx->rw_ctxs[i];

		first_wr = rdma_rw_ctx_wrs(&ctx->rw, ch->qp, ch->sport->port,
				cqe, first_wr);
		cqe = NULL;
	}

	ret = ib_post_send(ch->qp, first_wr, &bad_wr);
	if (ret) {
		pr_err("%s: ib_post_send() returned %d for %d (avail: %d)\n",
			 __func__, ret, ioctx->n_rdma,
			 atomic_read(&ch->sq_wr_avail));
		goto out_undo;
	}

	return 0;
out_undo:
	atomic_add(ioctx->n_rdma, &ch->sq_wr_avail);
	return ret;
}

static u8 tcm_to_srp_tsk_mgmt_status(const int tcm_mgmt_status)
{
	switch (tcm_mgmt_status) {
	case TMR_FUNCTION_COMPLETE:
		return SRP_TSK_MGMT_SUCCESS;
	case TMR_FUNCTION_REJECTED:
		return SRP_TSK_MGMT_FUNC_NOT_SUPP;
	}
	return SRP_TSK_MGMT_FAILED;
}

/**
 * srpt_queue_response - transmit the response to a SCSI command
 * @cmd: SCSI target command.
 *
 * Callback function called by the TCM core. Must not block since it can be
 * invoked on the context of the IB completion handler.
 */
static void srpt_queue_response(struct se_cmd *cmd)
{
	struct srpt_send_ioctx *ioctx =
		container_of(cmd, struct srpt_send_ioctx, cmd);
	struct srpt_rdma_ch *ch = ioctx->ch;
	struct srpt_device *sdev = ch->sport->sdev;
	struct ib_send_wr send_wr, *first_wr = &send_wr, *bad_wr;
	struct ib_sge sge;
	enum srpt_command_state state;
	int resp_len, ret, i;
	u8 srp_tm_status;

	BUG_ON(!ch);

	state = ioctx->state;
	switch (state) {
	case SRPT_STATE_NEW:
	case SRPT_STATE_DATA_IN:
		ioctx->state = SRPT_STATE_CMD_RSP_SENT;
		break;
	case SRPT_STATE_MGMT:
		ioctx->state = SRPT_STATE_MGMT_RSP_SENT;
		break;
	default:
		WARN(true, "ch %p; cmd %d: unexpected command state %d\n",
			ch, ioctx->ioctx.index, ioctx->state);
		break;
	}

	if (unlikely(WARN_ON_ONCE(state == SRPT_STATE_CMD_RSP_SENT)))
		return;

	/* For read commands, transfer the data to the initiator. */
	if (ioctx->cmd.data_direction == DMA_FROM_DEVICE &&
	    ioctx->cmd.data_length &&
	    !ioctx->queue_status_only) {
		for (i = ioctx->n_rw_ctx - 1; i >= 0; i--) {
			struct srpt_rw_ctx *ctx = &ioctx->rw_ctxs[i];

			first_wr = rdma_rw_ctx_wrs(&ctx->rw, ch->qp,
					ch->sport->port, NULL, first_wr);
		}
	}

	if (state != SRPT_STATE_MGMT)
		resp_len = srpt_build_cmd_rsp(ch, ioctx, ioctx->cmd.tag,
					      cmd->scsi_status);
	else {
		srp_tm_status
			= tcm_to_srp_tsk_mgmt_status(cmd->se_tmr_req->response);
		resp_len = srpt_build_tskmgmt_rsp(ch, ioctx, srp_tm_status,
						 ioctx->cmd.tag);
	}

	atomic_inc(&ch->req_lim);

	if (unlikely(atomic_sub_return(1 + ioctx->n_rdma,
			&ch->sq_wr_avail) < 0)) {
		pr_warn("%s: IB send queue full (needed %d)\n",
				__func__, ioctx->n_rdma);
		ret = -ENOMEM;
		goto out;
	}

	ib_dma_sync_single_for_device(sdev->device, ioctx->ioctx.dma, resp_len,
				      DMA_TO_DEVICE);

	sge.addr = ioctx->ioctx.dma;
	sge.length = resp_len;
	sge.lkey = sdev->lkey;

	ioctx->ioctx.cqe.done = srpt_send_done;
	send_wr.next = NULL;
	send_wr.wr_cqe = &ioctx->ioctx.cqe;
	send_wr.sg_list = &sge;
	send_wr.num_sge = 1;
	send_wr.opcode = IB_WR_SEND;
	send_wr.send_flags = IB_SEND_SIGNALED;

	ret = ib_post_send(ch->qp, first_wr, &bad_wr);
	if (ret < 0) {
		pr_err("%s: sending cmd response failed for tag %llu (%d)\n",
			__func__, ioctx->cmd.tag, ret);
		goto out;
	}

	return;

out:
	atomic_add(1 + ioctx->n_rdma, &ch->sq_wr_avail);
	atomic_dec(&ch->req_lim);
	srpt_set_cmd_state(ioctx, SRPT_STATE_DONE);
	target_put_sess_cmd(&ioctx->cmd);
}

static int srpt_queue_data_in(struct se_cmd *cmd)
{
	srpt_queue_response(cmd);
	return 0;
}

static void srpt_queue_tm_rsp(struct se_cmd *cmd)
{
	srpt_queue_response(cmd);
}

static void srpt_aborted_task(struct se_cmd *cmd)
{
}

static int srpt_queue_status(struct se_cmd *cmd)
{
	struct srpt_send_ioctx *ioctx;

	ioctx = container_of(cmd, struct srpt_send_ioctx, cmd);
	BUG_ON(ioctx->sense_data != cmd->sense_buffer);
	if (cmd->se_cmd_flags &
	    (SCF_TRANSPORT_TASK_SENSE | SCF_EMULATED_TASK_SENSE))
		WARN_ON(cmd->scsi_status != SAM_STAT_CHECK_CONDITION);
	ioctx->queue_status_only = true;
	srpt_queue_response(cmd);
	return 0;
}

static void srpt_refresh_port_work(struct work_struct *work)
{
	struct srpt_port *sport = container_of(work, struct srpt_port, work);

	srpt_refresh_port(sport);
}

static bool srpt_ch_list_empty(struct srpt_port *sport)
{
	struct srpt_nexus *nexus;
	bool res = true;

	rcu_read_lock();
	list_for_each_entry(nexus, &sport->nexus_list, entry)
		if (!list_empty(&nexus->ch_list))
			res = false;
	rcu_read_unlock();

	return res;
}

/**
 * srpt_release_sport - disable login and wait for associated channels
 * @sport: SRPT HCA port.
 */
static int srpt_release_sport(struct srpt_port *sport)
{
	struct srpt_nexus *nexus, *next_n;
	struct srpt_rdma_ch *ch;

	WARN_ON_ONCE(irqs_disabled());

	mutex_lock(&sport->mutex);
	srpt_set_enabled(sport, false);
	mutex_unlock(&sport->mutex);

	while (wait_event_timeout(sport->ch_releaseQ,
				  srpt_ch_list_empty(sport), 5 * HZ) <= 0) {
		pr_info("%s_%d: waiting for session unregistration ...\n",
			sport->sdev->device->name, sport->port);
		rcu_read_lock();
		list_for_each_entry(nexus, &sport->nexus_list, entry) {
			list_for_each_entry(ch, &nexus->ch_list, list) {
				pr_info("%s-%d: state %s\n",
					ch->sess_name, ch->qp->qp_num,
					get_ch_state_name(ch->state));
			}
		}
		rcu_read_unlock();
	}

	mutex_lock(&sport->mutex);
	list_for_each_entry_safe(nexus, next_n, &sport->nexus_list, entry) {
		list_del(&nexus->entry);
		kfree_rcu(nexus, rcu);
	}
	mutex_unlock(&sport->mutex);

	return 0;
}

static struct se_wwn *__srpt_lookup_wwn(const char *name)
{
	struct ib_device *dev;
	struct srpt_device *sdev;
	struct srpt_port *sport;
	int i;

	list_for_each_entry(sdev, &srpt_dev_list, list) {
		dev = sdev->device;
		if (!dev)
			continue;

		for (i = 0; i < dev->phys_port_cnt; i++) {
			sport = &sdev->port[i];

			if (strcmp(sport->port_guid, name) == 0)
				return &sport->port_guid_wwn;
			if (strcmp(sport->port_gid, name) == 0)
				return &sport->port_gid_wwn;
		}
	}

	return NULL;
}

static struct se_wwn *srpt_lookup_wwn(const char *name)
{
	struct se_wwn *wwn;

	spin_lock(&srpt_dev_lock);
	wwn = __srpt_lookup_wwn(name);
	spin_unlock(&srpt_dev_lock);

	return wwn;
}

static void srpt_free_srq(struct srpt_device *sdev)
{
	if (!sdev->srq)
		return;

	ib_destroy_srq(sdev->srq);
	srpt_free_ioctx_ring((struct srpt_ioctx **)sdev->ioctx_ring, sdev,
			     sdev->srq_size, srp_max_req_size, DMA_FROM_DEVICE);
	sdev->srq = NULL;
}

static int srpt_alloc_srq(struct srpt_device *sdev)
{
	struct ib_srq_init_attr srq_attr = {
		.event_handler = srpt_srq_event,
		.srq_context = (void *)sdev,
		.attr.max_wr = sdev->srq_size,
		.attr.max_sge = 1,
		.srq_type = IB_SRQT_BASIC,
	};
	struct ib_device *device = sdev->device;
	struct ib_srq *srq;
	int i;

	WARN_ON_ONCE(sdev->srq);
	srq = ib_create_srq(sdev->pd, &srq_attr);
	if (IS_ERR(srq)) {
		pr_debug("ib_create_srq() failed: %ld\n", PTR_ERR(srq));
		return PTR_ERR(srq);
	}

	pr_debug("create SRQ #wr= %d max_allow=%d dev= %s\n", sdev->srq_size,
		 sdev->device->attrs.max_srq_wr, device->name);

	sdev->ioctx_ring = (struct srpt_recv_ioctx **)
		srpt_alloc_ioctx_ring(sdev, sdev->srq_size,
				      sizeof(*sdev->ioctx_ring[0]),
				      srp_max_req_size, DMA_FROM_DEVICE);
	if (!sdev->ioctx_ring) {
		ib_destroy_srq(srq);
		return -ENOMEM;
	}

	sdev->use_srq = true;
	sdev->srq = srq;

	for (i = 0; i < sdev->srq_size; ++i) {
		INIT_LIST_HEAD(&sdev->ioctx_ring[i]->wait_list);
		srpt_post_recv(sdev, NULL, sdev->ioctx_ring[i]);
	}

	return 0;
}

static int srpt_use_srq(struct srpt_device *sdev, bool use_srq)
{
	struct ib_device *device = sdev->device;
	int ret = 0;

	if (!use_srq) {
		srpt_free_srq(sdev);
		sdev->use_srq = false;
	} else if (use_srq && !sdev->srq) {
		ret = srpt_alloc_srq(sdev);
	}
	pr_debug("%s(%s): use_srq = %d; ret = %d\n", __func__, device->name,
		 sdev->use_srq, ret);
	return ret;
}

/**
 * srpt_add_one - InfiniBand device addition callback function
 * @device: Describes a HCA.
 */
static void srpt_add_one(struct ib_device *device)
{
	struct srpt_device *sdev;
	struct srpt_port *sport;
	int i;

	pr_debug("device = %p\n", device);

	sdev = kzalloc(sizeof(*sdev), GFP_KERNEL);
	if (!sdev)
		goto err;

	sdev->device = device;
	mutex_init(&sdev->sdev_mutex);

	sdev->pd = ib_alloc_pd(device, 0);
	if (IS_ERR(sdev->pd))
		goto free_dev;

	sdev->lkey = sdev->pd->local_dma_lkey;

	sdev->srq_size = min(srpt_srq_size, sdev->device->attrs.max_srq_wr);

	srpt_use_srq(sdev, sdev->port[0].port_attrib.use_srq);

	if (!srpt_service_guid)
		srpt_service_guid = be64_to_cpu(device->node_guid);

	sdev->cm_id = ib_create_cm_id(device, srpt_cm_handler, sdev);
	if (IS_ERR(sdev->cm_id))
		goto err_ring;

	/* print out target login information */
	pr_debug("Target login info: id_ext=%016llx,ioc_guid=%016llx,"
		 "pkey=ffff,service_id=%016llx\n", srpt_service_guid,
		 srpt_service_guid, srpt_service_guid);

	/*
	 * We do not have a consistent service_id (ie. also id_ext of target_id)
	 * to identify this target. We currently use the guid of the first HCA
	 * in the system as service_id; therefore, the target_id will change
	 * if this HCA is gone bad and replaced by different HCA
	 */
	if (ib_cm_listen(sdev->cm_id, cpu_to_be64(srpt_service_guid), 0))
		goto err_cm;

	INIT_IB_EVENT_HANDLER(&sdev->event_handler, sdev->device,
			      srpt_event_handler);
	ib_register_event_handler(&sdev->event_handler);

	WARN_ON(sdev->device->phys_port_cnt > ARRAY_SIZE(sdev->port));

	for (i = 1; i <= sdev->device->phys_port_cnt; i++) {
		sport = &sdev->port[i - 1];
		INIT_LIST_HEAD(&sport->nexus_list);
		init_waitqueue_head(&sport->ch_releaseQ);
		mutex_init(&sport->mutex);
		sport->sdev = sdev;
		sport->port = i;
		sport->port_attrib.srp_max_rdma_size = DEFAULT_MAX_RDMA_SIZE;
		sport->port_attrib.srp_max_rsp_size = DEFAULT_MAX_RSP_SIZE;
		sport->port_attrib.srp_sq_size = DEF_SRPT_SQ_SIZE;
		sport->port_attrib.use_srq = false;
		INIT_WORK(&sport->work, srpt_refresh_port_work);

		if (srpt_refresh_port(sport)) {
			pr_err("MAD registration failed for %s-%d.\n",
			       sdev->device->name, i);
			goto err_event;
		}
	}

	spin_lock(&srpt_dev_lock);
	list_add_tail(&sdev->list, &srpt_dev_list);
	spin_unlock(&srpt_dev_lock);

out:
	ib_set_client_data(device, &srpt_client, sdev);
	pr_debug("added %s.\n", device->name);
	return;

err_event:
	ib_unregister_event_handler(&sdev->event_handler);
err_cm:
	ib_destroy_cm_id(sdev->cm_id);
err_ring:
	srpt_free_srq(sdev);
	ib_dealloc_pd(sdev->pd);
free_dev:
	kfree(sdev);
err:
	sdev = NULL;
	pr_info("%s(%s) failed.\n", __func__, device->name);
	goto out;
}

/**
 * srpt_remove_one - InfiniBand device removal callback function
 * @device: Describes a HCA.
 * @client_data: The value passed as the third argument to ib_set_client_data().
 */
static void srpt_remove_one(struct ib_device *device, void *client_data)
{
	struct srpt_device *sdev = client_data;
	int i;

	if (!sdev) {
		pr_info("%s(%s): nothing to do.\n", __func__, device->name);
		return;
	}

	srpt_unregister_mad_agent(sdev);

	ib_unregister_event_handler(&sdev->event_handler);

	/* Cancel any work queued by the just unregistered IB event handler. */
	for (i = 0; i < sdev->device->phys_port_cnt; i++)
		cancel_work_sync(&sdev->port[i].work);

	ib_destroy_cm_id(sdev->cm_id);

	/*
	 * Unregistering a target must happen after destroying sdev->cm_id
	 * such that no new SRP_LOGIN_REQ information units can arrive while
	 * destroying the target.
	 */
	spin_lock(&srpt_dev_lock);
	list_del(&sdev->list);
	spin_unlock(&srpt_dev_lock);

	for (i = 0; i < sdev->device->phys_port_cnt; i++)
		srpt_release_sport(&sdev->port[i]);

	srpt_free_srq(sdev);

	ib_dealloc_pd(sdev->pd);

	kfree(sdev);
}

static struct ib_client srpt_client = {
	.name = DRV_NAME,
	.add = srpt_add_one,
	.remove = srpt_remove_one
};

static int srpt_check_true(struct se_portal_group *se_tpg)
{
	return 1;
}

static int srpt_check_false(struct se_portal_group *se_tpg)
{
	return 0;
}

static char *srpt_get_fabric_name(void)
{
	return "srpt";
}

static struct srpt_port *srpt_tpg_to_sport(struct se_portal_group *tpg)
{
	return tpg->se_tpg_wwn->priv;
}

static char *srpt_get_fabric_wwn(struct se_portal_group *tpg)
{
	struct srpt_port *sport = srpt_tpg_to_sport(tpg);

	WARN_ON_ONCE(tpg != &sport->port_guid_tpg &&
		     tpg != &sport->port_gid_tpg);
	return tpg == &sport->port_guid_tpg ? sport->port_guid :
		sport->port_gid;
}

static u16 srpt_get_tag(struct se_portal_group *tpg)
{
	return 1;
}

static u32 srpt_tpg_get_inst_index(struct se_portal_group *se_tpg)
{
	return 1;
}

static void srpt_release_cmd(struct se_cmd *se_cmd)
{
	struct srpt_send_ioctx *ioctx = container_of(se_cmd,
				struct srpt_send_ioctx, cmd);
	struct srpt_rdma_ch *ch = ioctx->ch;
	unsigned long flags;

	WARN_ON_ONCE(ioctx->state != SRPT_STATE_DONE &&
		     !(ioctx->cmd.transport_state & CMD_T_ABORTED));

	if (ioctx->n_rw_ctx) {
		srpt_free_rw_ctxs(ch, ioctx);
		ioctx->n_rw_ctx = 0;
	}

	spin_lock_irqsave(&ch->spinlock, flags);
	list_add(&ioctx->free_list, &ch->free_list);
	spin_unlock_irqrestore(&ch->spinlock, flags);
}

/**
 * srpt_close_session - forcibly close a session
 * @se_sess: SCSI target session.
 *
 * Callback function invoked by the TCM core to clean up sessions associated
 * with a node ACL when the user invokes
 * rmdir /sys/kernel/config/target/$driver/$port/$tpg/acls/$i_port_id
 */
static void srpt_close_session(struct se_session *se_sess)
{
	struct srpt_rdma_ch *ch = se_sess->fabric_sess_ptr;

	srpt_disconnect_ch_sync(ch);
}

/**
 * srpt_sess_get_index - return the value of scsiAttIntrPortIndex (SCSI-MIB)
 * @se_sess: SCSI target session.
 *
 * A quote from RFC 4455 (SCSI-MIB) about this MIB object:
 * This object represents an arbitrary integer used to uniquely identify a
 * particular attached remote initiator port to a particular SCSI target port
 * within a particular SCSI target device within a particular SCSI instance.
 */
static u32 srpt_sess_get_index(struct se_session *se_sess)
{
	return 0;
}

static void srpt_set_default_node_attrs(struct se_node_acl *nacl)
{
}

/* Note: only used from inside debug printk's by the TCM core. */
static int srpt_get_tcm_cmd_state(struct se_cmd *se_cmd)
{
	struct srpt_send_ioctx *ioctx;

	ioctx = container_of(se_cmd, struct srpt_send_ioctx, cmd);
	return ioctx->state;
}

static int srpt_parse_guid(u64 *guid, const char *name)
{
	u16 w[4];
	int ret = -EINVAL;

	if (sscanf(name, "%hx:%hx:%hx:%hx", &w[0], &w[1], &w[2], &w[3]) != 4)
		goto out;
	*guid = get_unaligned_be64(w);
	ret = 0;
out:
	return ret;
}

/**
 * srpt_parse_i_port_id - parse an initiator port ID
 * @name: ASCII representation of a 128-bit initiator port ID.
 * @i_port_id: Binary 128-bit port ID.
 */
static int srpt_parse_i_port_id(u8 i_port_id[16], const char *name)
{
	const char *p;
	unsigned len, count, leading_zero_bytes;
	int ret;

	p = name;
	if (strncasecmp(p, "0x", 2) == 0)
		p += 2;
	ret = -EINVAL;
	len = strlen(p);
	if (len % 2)
		goto out;
	count = min(len / 2, 16U);
	leading_zero_bytes = 16 - count;
	memset(i_port_id, 0, leading_zero_bytes);
	ret = hex2bin(i_port_id + leading_zero_bytes, p, count);
	if (ret < 0)
		pr_debug("hex2bin failed for srpt_parse_i_port_id: %d\n", ret);
out:
	return ret;
}

/*
 * configfs callback function invoked for
 * mkdir /sys/kernel/config/target/$driver/$port/$tpg/acls/$i_port_id
 */
static int srpt_init_nodeacl(struct se_node_acl *se_nacl, const char *name)
{
	u64 guid;
	u8 i_port_id[16];
	int ret;

	ret = srpt_parse_guid(&guid, name);
	if (ret < 0)
		ret = srpt_parse_i_port_id(i_port_id, name);
	if (ret < 0)
		pr_err("invalid initiator port ID %s\n", name);
	return ret;
}

static ssize_t srpt_tpg_attrib_srp_max_rdma_size_show(struct config_item *item,
		char *page)
{
	struct se_portal_group *se_tpg = attrib_to_tpg(item);
	struct srpt_port *sport = srpt_tpg_to_sport(se_tpg);

	return sprintf(page, "%u\n", sport->port_attrib.srp_max_rdma_size);
}

static ssize_t srpt_tpg_attrib_srp_max_rdma_size_store(struct config_item *item,
		const char *page, size_t count)
{
	struct se_portal_group *se_tpg = attrib_to_tpg(item);
	struct srpt_port *sport = srpt_tpg_to_sport(se_tpg);
	unsigned long val;
	int ret;

	ret = kstrtoul(page, 0, &val);
	if (ret < 0) {
		pr_err("kstrtoul() failed with ret: %d\n", ret);
		return -EINVAL;
	}
	if (val > MAX_SRPT_RDMA_SIZE) {
		pr_err("val: %lu exceeds MAX_SRPT_RDMA_SIZE: %d\n", val,
			MAX_SRPT_RDMA_SIZE);
		return -EINVAL;
	}
	if (val < DEFAULT_MAX_RDMA_SIZE) {
		pr_err("val: %lu smaller than DEFAULT_MAX_RDMA_SIZE: %d\n",
			val, DEFAULT_MAX_RDMA_SIZE);
		return -EINVAL;
	}
	sport->port_attrib.srp_max_rdma_size = val;

	return count;
}

static ssize_t srpt_tpg_attrib_srp_max_rsp_size_show(struct config_item *item,
		char *page)
{
	struct se_portal_group *se_tpg = attrib_to_tpg(item);
	struct srpt_port *sport = srpt_tpg_to_sport(se_tpg);

	return sprintf(page, "%u\n", sport->port_attrib.srp_max_rsp_size);
}

static ssize_t srpt_tpg_attrib_srp_max_rsp_size_store(struct config_item *item,
		const char *page, size_t count)
{
	struct se_portal_group *se_tpg = attrib_to_tpg(item);
	struct srpt_port *sport = srpt_tpg_to_sport(se_tpg);
	unsigned long val;
	int ret;

	ret = kstrtoul(page, 0, &val);
	if (ret < 0) {
		pr_err("kstrtoul() failed with ret: %d\n", ret);
		return -EINVAL;
	}
	if (val > MAX_SRPT_RSP_SIZE) {
		pr_err("val: %lu exceeds MAX_SRPT_RSP_SIZE: %d\n", val,
			MAX_SRPT_RSP_SIZE);
		return -EINVAL;
	}
	if (val < MIN_MAX_RSP_SIZE) {
		pr_err("val: %lu smaller than MIN_MAX_RSP_SIZE: %d\n", val,
			MIN_MAX_RSP_SIZE);
		return -EINVAL;
	}
	sport->port_attrib.srp_max_rsp_size = val;

	return count;
}

static ssize_t srpt_tpg_attrib_srp_sq_size_show(struct config_item *item,
		char *page)
{
	struct se_portal_group *se_tpg = attrib_to_tpg(item);
	struct srpt_port *sport = srpt_tpg_to_sport(se_tpg);

	return sprintf(page, "%u\n", sport->port_attrib.srp_sq_size);
}

static ssize_t srpt_tpg_attrib_srp_sq_size_store(struct config_item *item,
		const char *page, size_t count)
{
	struct se_portal_group *se_tpg = attrib_to_tpg(item);
	struct srpt_port *sport = srpt_tpg_to_sport(se_tpg);
	unsigned long val;
	int ret;

	ret = kstrtoul(page, 0, &val);
	if (ret < 0) {
		pr_err("kstrtoul() failed with ret: %d\n", ret);
		return -EINVAL;
	}
	if (val > MAX_SRPT_SRQ_SIZE) {
		pr_err("val: %lu exceeds MAX_SRPT_SRQ_SIZE: %d\n", val,
			MAX_SRPT_SRQ_SIZE);
		return -EINVAL;
	}
	if (val < MIN_SRPT_SRQ_SIZE) {
		pr_err("val: %lu smaller than MIN_SRPT_SRQ_SIZE: %d\n", val,
			MIN_SRPT_SRQ_SIZE);
		return -EINVAL;
	}
	sport->port_attrib.srp_sq_size = val;

	return count;
}

static ssize_t srpt_tpg_attrib_use_srq_show(struct config_item *item,
					    char *page)
{
	struct se_portal_group *se_tpg = attrib_to_tpg(item);
	struct srpt_port *sport = srpt_tpg_to_sport(se_tpg);

	return sprintf(page, "%d\n", sport->port_attrib.use_srq);
}

static ssize_t srpt_tpg_attrib_use_srq_store(struct config_item *item,
					     const char *page, size_t count)
{
	struct se_portal_group *se_tpg = attrib_to_tpg(item);
	struct srpt_port *sport = srpt_tpg_to_sport(se_tpg);
	struct srpt_device *sdev = sport->sdev;
	unsigned long val;
	bool enabled;
	int ret;

	ret = kstrtoul(page, 0, &val);
	if (ret < 0)
		return ret;
	if (val != !!val)
		return -EINVAL;

	ret = mutex_lock_interruptible(&sdev->sdev_mutex);
	if (ret < 0)
		return ret;
	ret = mutex_lock_interruptible(&sport->mutex);
	if (ret < 0)
		goto unlock_sdev;
	enabled = sport->enabled;
	/* Log out all initiator systems before changing 'use_srq'. */
	srpt_set_enabled(sport, false);
	sport->port_attrib.use_srq = val;
	srpt_use_srq(sdev, sport->port_attrib.use_srq);
	srpt_set_enabled(sport, enabled);
	ret = count;
	mutex_unlock(&sport->mutex);
unlock_sdev:
	mutex_unlock(&sdev->sdev_mutex);

	return ret;
}

CONFIGFS_ATTR(srpt_tpg_attrib_,  srp_max_rdma_size);
CONFIGFS_ATTR(srpt_tpg_attrib_,  srp_max_rsp_size);
CONFIGFS_ATTR(srpt_tpg_attrib_,  srp_sq_size);
CONFIGFS_ATTR(srpt_tpg_attrib_,  use_srq);

static struct configfs_attribute *srpt_tpg_attrib_attrs[] = {
	&srpt_tpg_attrib_attr_srp_max_rdma_size,
	&srpt_tpg_attrib_attr_srp_max_rsp_size,
	&srpt_tpg_attrib_attr_srp_sq_size,
	&srpt_tpg_attrib_attr_use_srq,
	NULL,
};

static ssize_t srpt_tpg_enable_show(struct config_item *item, char *page)
{
	struct se_portal_group *se_tpg = to_tpg(item);
	struct srpt_port *sport = srpt_tpg_to_sport(se_tpg);

	return snprintf(page, PAGE_SIZE, "%d\n", (sport->enabled) ? 1: 0);
}

static ssize_t srpt_tpg_enable_store(struct config_item *item,
		const char *page, size_t count)
{
	struct se_portal_group *se_tpg = to_tpg(item);
	struct srpt_port *sport = srpt_tpg_to_sport(se_tpg);
	unsigned long tmp;
        int ret;

	ret = kstrtoul(page, 0, &tmp);
	if (ret < 0) {
		pr_err("Unable to extract srpt_tpg_store_enable\n");
		return -EINVAL;
	}

	if ((tmp != 0) && (tmp != 1)) {
		pr_err("Illegal value for srpt_tpg_store_enable: %lu\n", tmp);
		return -EINVAL;
	}

	mutex_lock(&sport->mutex);
	srpt_set_enabled(sport, tmp);
	mutex_unlock(&sport->mutex);

	return count;
}

CONFIGFS_ATTR(srpt_tpg_, enable);

static struct configfs_attribute *srpt_tpg_attrs[] = {
	&srpt_tpg_attr_enable,
	NULL,
};

/**
 * srpt_make_tpg - configfs callback invoked for mkdir /sys/kernel/config/target/$driver/$port/$tpg
 * @wwn: Corresponds to $driver/$port.
 * @group: Not used.
 * @name: $tpg.
 */
static struct se_portal_group *srpt_make_tpg(struct se_wwn *wwn,
					     struct config_group *group,
					     const char *name)
{
	struct srpt_port *sport = wwn->priv;
	static struct se_portal_group *tpg;
	int res;

	WARN_ON_ONCE(wwn != &sport->port_guid_wwn &&
		     wwn != &sport->port_gid_wwn);
	tpg = wwn == &sport->port_guid_wwn ? &sport->port_guid_tpg :
		&sport->port_gid_tpg;
	res = core_tpg_register(wwn, tpg, SCSI_PROTOCOL_SRP);
	if (res)
		return ERR_PTR(res);

	return tpg;
}

/**
 * srpt_drop_tpg - configfs callback invoked for rmdir /sys/kernel/config/target/$driver/$port/$tpg
 * @tpg: Target portal group to deregister.
 */
static void srpt_drop_tpg(struct se_portal_group *tpg)
{
	struct srpt_port *sport = srpt_tpg_to_sport(tpg);

	sport->enabled = false;
	core_tpg_deregister(tpg);
}

/**
 * srpt_make_tport - configfs callback invoked for mkdir /sys/kernel/config/target/$driver/$port
 * @tf: Not used.
 * @group: Not used.
 * @name: $port.
 */
static struct se_wwn *srpt_make_tport(struct target_fabric_configfs *tf,
				      struct config_group *group,
				      const char *name)
{
	return srpt_lookup_wwn(name) ? : ERR_PTR(-EINVAL);
}

/**
 * srpt_drop_tport - configfs callback invoked for rmdir /sys/kernel/config/target/$driver/$port
 * @wwn: $port.
 */
static void srpt_drop_tport(struct se_wwn *wwn)
{
}

static ssize_t srpt_wwn_version_show(struct config_item *item, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s\n", DRV_VERSION);
}

CONFIGFS_ATTR_RO(srpt_wwn_, version);

static struct configfs_attribute *srpt_wwn_attrs[] = {
	&srpt_wwn_attr_version,
	NULL,
};

static const struct target_core_fabric_ops srpt_template = {
	.module				= THIS_MODULE,
	.name				= "srpt",
	.get_fabric_name		= srpt_get_fabric_name,
	.tpg_get_wwn			= srpt_get_fabric_wwn,
	.tpg_get_tag			= srpt_get_tag,
	.tpg_check_demo_mode		= srpt_check_false,
	.tpg_check_demo_mode_cache	= srpt_check_true,
	.tpg_check_demo_mode_write_protect = srpt_check_true,
	.tpg_check_prod_mode_write_protect = srpt_check_false,
	.tpg_get_inst_index		= srpt_tpg_get_inst_index,
	.release_cmd			= srpt_release_cmd,
	.check_stop_free		= srpt_check_stop_free,
	.close_session			= srpt_close_session,
	.sess_get_index			= srpt_sess_get_index,
	.sess_get_initiator_sid		= NULL,
	.write_pending			= srpt_write_pending,
	.write_pending_status		= srpt_write_pending_status,
	.set_default_node_attributes	= srpt_set_default_node_attrs,
	.get_cmd_state			= srpt_get_tcm_cmd_state,
	.queue_data_in			= srpt_queue_data_in,
	.queue_status			= srpt_queue_status,
	.queue_tm_rsp			= srpt_queue_tm_rsp,
	.aborted_task			= srpt_aborted_task,
	/*
	 * Setup function pointers for generic logic in
	 * target_core_fabric_configfs.c
	 */
	.fabric_make_wwn		= srpt_make_tport,
	.fabric_drop_wwn		= srpt_drop_tport,
	.fabric_make_tpg		= srpt_make_tpg,
	.fabric_drop_tpg		= srpt_drop_tpg,
	.fabric_init_nodeacl		= srpt_init_nodeacl,

	.tfc_wwn_attrs			= srpt_wwn_attrs,
	.tfc_tpg_base_attrs		= srpt_tpg_attrs,
	.tfc_tpg_attrib_attrs		= srpt_tpg_attrib_attrs,
};

/**
 * srpt_init_module - kernel module initialization
 *
 * Note: Since ib_register_client() registers callback functions, and since at
 * least one of these callback functions (srpt_add_one()) calls target core
 * functions, this driver must be registered with the target core before
 * ib_register_client() is called.
 */
static int __init srpt_init_module(void)
{
	int ret;

	ret = -EINVAL;
	if (srp_max_req_size < MIN_MAX_REQ_SIZE) {
		pr_err("invalid value %d for kernel module parameter"
		       " srp_max_req_size -- must be at least %d.\n",
		       srp_max_req_size, MIN_MAX_REQ_SIZE);
		goto out;
	}

	if (srpt_srq_size < MIN_SRPT_SRQ_SIZE
	    || srpt_srq_size > MAX_SRPT_SRQ_SIZE) {
		pr_err("invalid value %d for kernel module parameter"
		       " srpt_srq_size -- must be in the range [%d..%d].\n",
		       srpt_srq_size, MIN_SRPT_SRQ_SIZE, MAX_SRPT_SRQ_SIZE);
		goto out;
	}

	ret = target_register_template(&srpt_template);
	if (ret)
		goto out;

	ret = ib_register_client(&srpt_client);
	if (ret) {
		pr_err("couldn't register IB client\n");
		goto out_unregister_target;
	}

	return 0;

out_unregister_target:
	target_unregister_template(&srpt_template);
out:
	return ret;
}

static void __exit srpt_cleanup_module(void)
{
	ib_unregister_client(&srpt_client);
	target_unregister_template(&srpt_template);
}

module_init(srpt_init_module);
module_exit(srpt_cleanup_module);
