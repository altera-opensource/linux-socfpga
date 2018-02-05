/*
 * NVM Express device driver tracepoints
 * Copyright (c) 2018 Johannes Thumshirn, SUSE Linux GmbH
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM nvme

#if !defined(_TRACE_NVME_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_NVME_H

#include <linux/nvme.h>
#include <linux/tracepoint.h>
#include <linux/trace_seq.h>

#include "nvme.h"

#define nvme_admin_opcode_name(opcode)	{ opcode, #opcode }
#define show_admin_opcode_name(val)					\
	__print_symbolic(val,						\
		nvme_admin_opcode_name(nvme_admin_delete_sq),		\
		nvme_admin_opcode_name(nvme_admin_create_sq),		\
		nvme_admin_opcode_name(nvme_admin_get_log_page),	\
		nvme_admin_opcode_name(nvme_admin_delete_cq),		\
		nvme_admin_opcode_name(nvme_admin_create_cq),		\
		nvme_admin_opcode_name(nvme_admin_identify),		\
		nvme_admin_opcode_name(nvme_admin_abort_cmd),		\
		nvme_admin_opcode_name(nvme_admin_set_features),	\
		nvme_admin_opcode_name(nvme_admin_get_features),	\
		nvme_admin_opcode_name(nvme_admin_async_event),		\
		nvme_admin_opcode_name(nvme_admin_ns_mgmt),		\
		nvme_admin_opcode_name(nvme_admin_activate_fw),		\
		nvme_admin_opcode_name(nvme_admin_download_fw),		\
		nvme_admin_opcode_name(nvme_admin_ns_attach),		\
		nvme_admin_opcode_name(nvme_admin_keep_alive),		\
		nvme_admin_opcode_name(nvme_admin_directive_send),	\
		nvme_admin_opcode_name(nvme_admin_directive_recv),	\
		nvme_admin_opcode_name(nvme_admin_dbbuf),		\
		nvme_admin_opcode_name(nvme_admin_format_nvm),		\
		nvme_admin_opcode_name(nvme_admin_security_send),	\
		nvme_admin_opcode_name(nvme_admin_security_recv),	\
		nvme_admin_opcode_name(nvme_admin_sanitize_nvm))

const char *nvme_trace_parse_admin_cmd(struct trace_seq *p, u8 opcode,
				       u8 *cdw10);
#define __parse_nvme_admin_cmd(opcode, cdw10) \
	nvme_trace_parse_admin_cmd(p, opcode, cdw10)

#define nvme_opcode_name(opcode)	{ opcode, #opcode }
#define show_opcode_name(val)					\
	__print_symbolic(val,					\
		nvme_opcode_name(nvme_cmd_flush),		\
		nvme_opcode_name(nvme_cmd_write),		\
		nvme_opcode_name(nvme_cmd_read),		\
		nvme_opcode_name(nvme_cmd_write_uncor),		\
		nvme_opcode_name(nvme_cmd_compare),		\
		nvme_opcode_name(nvme_cmd_write_zeroes),	\
		nvme_opcode_name(nvme_cmd_dsm),			\
		nvme_opcode_name(nvme_cmd_resv_register),	\
		nvme_opcode_name(nvme_cmd_resv_report),		\
		nvme_opcode_name(nvme_cmd_resv_acquire),	\
		nvme_opcode_name(nvme_cmd_resv_release))

const char *nvme_trace_parse_nvm_cmd(struct trace_seq *p, u8 opcode,
				     u8 *cdw10);
#define __parse_nvme_cmd(opcode, cdw10) \
	nvme_trace_parse_nvm_cmd(p, opcode, cdw10)

TRACE_EVENT(nvme_setup_admin_cmd,
	    TP_PROTO(struct nvme_command *cmd),
	    TP_ARGS(cmd),
	    TP_STRUCT__entry(
		    __field(u8, opcode)
		    __field(u8, flags)
		    __field(u16, cid)
		    __field(u64, metadata)
		    __array(u8, cdw10, 24)
	    ),
	    TP_fast_assign(
		    __entry->opcode = cmd->common.opcode;
		    __entry->flags = cmd->common.flags;
		    __entry->cid = cmd->common.command_id;
		    __entry->metadata = le64_to_cpu(cmd->common.metadata);
		    memcpy(__entry->cdw10, cmd->common.cdw10,
			   sizeof(__entry->cdw10));
	    ),
	    TP_printk(" cmdid=%u, flags=0x%x, meta=0x%llx, cmd=(%s %s)",
		      __entry->cid, __entry->flags, __entry->metadata,
		      show_admin_opcode_name(__entry->opcode),
		      __parse_nvme_admin_cmd(__entry->opcode, __entry->cdw10))
);


TRACE_EVENT(nvme_setup_nvm_cmd,
	    TP_PROTO(int qid, struct nvme_command *cmd),
	    TP_ARGS(qid, cmd),
	    TP_STRUCT__entry(
		    __field(int, qid)
		    __field(u8, opcode)
		    __field(u8, flags)
		    __field(u16, cid)
		    __field(u32, nsid)
		    __field(u64, metadata)
		    __array(u8, cdw10, 24)
	    ),
	    TP_fast_assign(
		    __entry->qid = qid;
		    __entry->opcode = cmd->common.opcode;
		    __entry->flags = cmd->common.flags;
		    __entry->cid = cmd->common.command_id;
		    __entry->nsid = le32_to_cpu(cmd->common.nsid);
		    __entry->metadata = le64_to_cpu(cmd->common.metadata);
		    memcpy(__entry->cdw10, cmd->common.cdw10,
			   sizeof(__entry->cdw10));
	    ),
	    TP_printk("qid=%d, nsid=%u, cmdid=%u, flags=0x%x, meta=0x%llx, cmd=(%s %s)",
		      __entry->qid, __entry->nsid, __entry->cid,
		      __entry->flags, __entry->metadata,
		      show_opcode_name(__entry->opcode),
		      __parse_nvme_cmd(__entry->opcode, __entry->cdw10))
);

TRACE_EVENT(nvme_complete_rq,
	    TP_PROTO(struct request *req),
	    TP_ARGS(req),
	    TP_STRUCT__entry(
		    __field(int, qid)
		    __field(int, cid)
		    __field(u64, result)
		    __field(u8, retries)
		    __field(u8, flags)
		    __field(u16, status)
	    ),
	    TP_fast_assign(
		    __entry->qid = req->q->id;
		    __entry->cid = req->tag;
		    __entry->result = le64_to_cpu(nvme_req(req)->result.u64);
		    __entry->retries = nvme_req(req)->retries;
		    __entry->flags = nvme_req(req)->flags;
		    __entry->status = nvme_req(req)->status;
	    ),
	    TP_printk("cmdid=%u, qid=%d, res=%llu, retries=%u, flags=0x%x, status=%u",
		      __entry->cid, __entry->qid, __entry->result,
		      __entry->retries, __entry->flags, __entry->status)

);

#endif /* _TRACE_NVME_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace

/* This part must be outside protection */
#include <trace/define_trace.h>
