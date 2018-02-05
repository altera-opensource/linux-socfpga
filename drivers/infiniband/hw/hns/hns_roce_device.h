/*
 * Copyright (c) 2016 Hisilicon Limited.
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
 */

#ifndef _HNS_ROCE_DEVICE_H
#define _HNS_ROCE_DEVICE_H

#include <rdma/ib_verbs.h>

#define DRV_NAME "hns_roce"

#define HNS_ROCE_HW_VER1	('h' << 24 | 'i' << 16 | '0' << 8 | '6')

#define MAC_ADDR_OCTET_NUM			6
#define HNS_ROCE_MAX_MSG_LEN			0x80000000

#define HNS_ROCE_ALOGN_UP(a, b) ((((a) + (b) - 1) / (b)) * (b))

#define HNS_ROCE_IB_MIN_SQ_STRIDE		6

#define HNS_ROCE_BA_SIZE			(32 * 4096)

/* Hardware specification only for v1 engine */
#define HNS_ROCE_MIN_CQE_NUM			0x40
#define HNS_ROCE_MIN_WQE_NUM			0x20

/* Hardware specification only for v1 engine */
#define HNS_ROCE_MAX_INNER_MTPT_NUM		0x7
#define HNS_ROCE_MAX_MTPT_PBL_NUM		0x100000

#define HNS_ROCE_EACH_FREE_CQ_WAIT_MSECS	20
#define HNS_ROCE_MAX_FREE_CQ_WAIT_CNT	\
	(5000 / HNS_ROCE_EACH_FREE_CQ_WAIT_MSECS)
#define HNS_ROCE_CQE_WCMD_EMPTY_BIT		0x2
#define HNS_ROCE_MIN_CQE_CNT			16

#define HNS_ROCE_MAX_IRQ_NUM			128

#define EQ_ENABLE				1
#define EQ_DISABLE				0

#define HNS_ROCE_CEQ				0
#define HNS_ROCE_AEQ				1

#define HNS_ROCE_CEQ_ENTRY_SIZE			0x4
#define HNS_ROCE_AEQ_ENTRY_SIZE			0x10

/* 4G/4K = 1M */
#define HNS_ROCE_SL_SHIFT			28
#define HNS_ROCE_TCLASS_SHIFT			20
#define HNS_ROCE_FLOW_LABLE_MASK		0xfffff

#define HNS_ROCE_MAX_PORTS			6
#define HNS_ROCE_MAX_GID_NUM			16
#define HNS_ROCE_GID_SIZE			16

#define HNS_ROCE_HOP_NUM_0			0xff

#define BITMAP_NO_RR				0
#define BITMAP_RR				1

#define MR_TYPE_MR				0x00
#define MR_TYPE_DMA				0x03

#define PKEY_ID					0xffff
#define GUID_LEN				8
#define NODE_DESC_SIZE				64
#define DB_REG_OFFSET				0x1000

#define SERV_TYPE_RC				0
#define SERV_TYPE_RD				1
#define SERV_TYPE_UC				2
#define SERV_TYPE_UD				3

#define PAGES_SHIFT_8				8
#define PAGES_SHIFT_16				16
#define PAGES_SHIFT_24				24
#define PAGES_SHIFT_32				32

enum hns_roce_qp_state {
	HNS_ROCE_QP_STATE_RST,
	HNS_ROCE_QP_STATE_INIT,
	HNS_ROCE_QP_STATE_RTR,
	HNS_ROCE_QP_STATE_RTS,
	HNS_ROCE_QP_STATE_SQD,
	HNS_ROCE_QP_STATE_ERR,
	HNS_ROCE_QP_NUM_STATE,
};

enum hns_roce_event {
	HNS_ROCE_EVENT_TYPE_PATH_MIG                  = 0x01,
	HNS_ROCE_EVENT_TYPE_PATH_MIG_FAILED           = 0x02,
	HNS_ROCE_EVENT_TYPE_COMM_EST                  = 0x03,
	HNS_ROCE_EVENT_TYPE_SQ_DRAINED                = 0x04,
	HNS_ROCE_EVENT_TYPE_WQ_CATAS_ERROR            = 0x05,
	HNS_ROCE_EVENT_TYPE_INV_REQ_LOCAL_WQ_ERROR    = 0x06,
	HNS_ROCE_EVENT_TYPE_LOCAL_WQ_ACCESS_ERROR     = 0x07,
	HNS_ROCE_EVENT_TYPE_SRQ_LIMIT_REACH           = 0x08,
	HNS_ROCE_EVENT_TYPE_SRQ_LAST_WQE_REACH        = 0x09,
	HNS_ROCE_EVENT_TYPE_SRQ_CATAS_ERROR           = 0x0a,
	HNS_ROCE_EVENT_TYPE_CQ_ACCESS_ERROR           = 0x0b,
	HNS_ROCE_EVENT_TYPE_CQ_OVERFLOW               = 0x0c,
	HNS_ROCE_EVENT_TYPE_CQ_ID_INVALID             = 0x0d,
	HNS_ROCE_EVENT_TYPE_PORT_CHANGE               = 0x0f,
	/* 0x10 and 0x11 is unused in currently application case */
	HNS_ROCE_EVENT_TYPE_DB_OVERFLOW               = 0x12,
	HNS_ROCE_EVENT_TYPE_MB                        = 0x13,
	HNS_ROCE_EVENT_TYPE_CEQ_OVERFLOW              = 0x14,
	HNS_ROCE_EVENT_TYPE_FLR			      = 0x15,
};

/* Local Work Queue Catastrophic Error,SUBTYPE 0x5 */
enum {
	HNS_ROCE_LWQCE_QPC_ERROR		= 1,
	HNS_ROCE_LWQCE_MTU_ERROR		= 2,
	HNS_ROCE_LWQCE_WQE_BA_ADDR_ERROR	= 3,
	HNS_ROCE_LWQCE_WQE_ADDR_ERROR		= 4,
	HNS_ROCE_LWQCE_SQ_WQE_SHIFT_ERROR	= 5,
	HNS_ROCE_LWQCE_SL_ERROR			= 6,
	HNS_ROCE_LWQCE_PORT_ERROR		= 7,
};

/* Local Access Violation Work Queue Error,SUBTYPE 0x7 */
enum {
	HNS_ROCE_LAVWQE_R_KEY_VIOLATION		= 1,
	HNS_ROCE_LAVWQE_LENGTH_ERROR		= 2,
	HNS_ROCE_LAVWQE_VA_ERROR		= 3,
	HNS_ROCE_LAVWQE_PD_ERROR		= 4,
	HNS_ROCE_LAVWQE_RW_ACC_ERROR		= 5,
	HNS_ROCE_LAVWQE_KEY_STATE_ERROR		= 6,
	HNS_ROCE_LAVWQE_MR_OPERATION_ERROR	= 7,
};

/* DOORBELL overflow subtype */
enum {
	HNS_ROCE_DB_SUBTYPE_SDB_OVF		= 1,
	HNS_ROCE_DB_SUBTYPE_SDB_ALM_OVF		= 2,
	HNS_ROCE_DB_SUBTYPE_ODB_OVF		= 3,
	HNS_ROCE_DB_SUBTYPE_ODB_ALM_OVF		= 4,
	HNS_ROCE_DB_SUBTYPE_SDB_ALM_EMP		= 5,
	HNS_ROCE_DB_SUBTYPE_ODB_ALM_EMP		= 6,
};

enum {
	/* RQ&SRQ related operations */
	HNS_ROCE_OPCODE_SEND_DATA_RECEIVE	= 0x06,
	HNS_ROCE_OPCODE_RDMA_WITH_IMM_RECEIVE	= 0x07,
};

enum {
	HNS_ROCE_CAP_FLAG_REREG_MR		= BIT(0),
	HNS_ROCE_CAP_FLAG_ROCE_V1_V2		= BIT(1),
	HNS_ROCE_CAP_FLAG_RQ_INLINE		= BIT(2)
};

enum hns_roce_mtt_type {
	MTT_TYPE_WQE,
	MTT_TYPE_CQE,
};

#define HNS_ROCE_CMD_SUCCESS			1

#define HNS_ROCE_PORT_DOWN			0
#define HNS_ROCE_PORT_UP			1

#define HNS_ROCE_MTT_ENTRY_PER_SEG		8

#define PAGE_ADDR_SHIFT				12

struct hns_roce_uar {
	u64		pfn;
	unsigned long	index;
};

struct hns_roce_ucontext {
	struct ib_ucontext	ibucontext;
	struct hns_roce_uar	uar;
};

struct hns_roce_pd {
	struct ib_pd		ibpd;
	unsigned long		pdn;
};

struct hns_roce_bitmap {
	/* Bitmap Traversal last a bit which is 1 */
	unsigned long		last;
	unsigned long		top;
	unsigned long		max;
	unsigned long		reserved_top;
	unsigned long		mask;
	spinlock_t		lock;
	unsigned long		*table;
};

/* Order bitmap length -- bit num compute formula: 1 << (max_order - order) */
/* Order = 0: bitmap is biggest, order = max bitmap is least (only a bit) */
/* Every bit repesent to a partner free/used status in bitmap */
/*
 * Initial, bits of other bitmap are all 0 except that a bit of max_order is 1
 * Bit = 1 represent to idle and available; bit = 0: not available
 */
struct hns_roce_buddy {
	/* Members point to every order level bitmap */
	unsigned long **bits;
	/* Represent to avail bits of the order level bitmap */
	u32            *num_free;
	int             max_order;
	spinlock_t      lock;
};

/* For Hardware Entry Memory */
struct hns_roce_hem_table {
	/* HEM type: 0 = qpc, 1 = mtt, 2 = cqc, 3 = srq, 4 = other */
	u32		type;
	/* HEM array elment num */
	unsigned long	num_hem;
	/* HEM entry record obj total num */
	unsigned long	num_obj;
	/*Single obj size */
	unsigned long	obj_size;
	unsigned long	table_chunk_size;
	int		lowmem;
	struct mutex	mutex;
	struct hns_roce_hem **hem;
	u64		**bt_l1;
	dma_addr_t	*bt_l1_dma_addr;
	u64		**bt_l0;
	dma_addr_t	*bt_l0_dma_addr;
};

struct hns_roce_mtt {
	unsigned long		first_seg;
	int			order;
	int			page_shift;
	enum hns_roce_mtt_type	mtt_type;
};

/* Only support 4K page size for mr register */
#define MR_SIZE_4K 0

struct hns_roce_mr {
	struct ib_mr		ibmr;
	struct ib_umem		*umem;
	u64			iova; /* MR's virtual orignal addr */
	u64			size; /* Address range of MR */
	u32			key; /* Key of MR */
	u32			pd;   /* PD num of MR */
	u32			access;/* Access permission of MR */
	int			enabled; /* MR's active status */
	int			type;	/* MR's register type */
	u64			*pbl_buf;/* MR's PBL space */
	dma_addr_t		pbl_dma_addr;	/* MR's PBL space PA */
	u32			pbl_size;/* PA number in the PBL */
	u64			pbl_ba;/* page table address */
	u32			l0_chunk_last_num;/* L0 last number */
	u32			l1_chunk_last_num;/* L1 last number */
	u64			**pbl_bt_l2;/* PBL BT L2 */
	u64			**pbl_bt_l1;/* PBL BT L1 */
	u64			*pbl_bt_l0;/* PBL BT L0 */
	dma_addr_t		*pbl_l2_dma_addr;/* PBL BT L2 dma addr */
	dma_addr_t		*pbl_l1_dma_addr;/* PBL BT L1 dma addr */
	dma_addr_t		pbl_l0_dma_addr;/* PBL BT L0 dma addr */
	u32			pbl_ba_pg_sz;/* BT chunk page size */
	u32			pbl_buf_pg_sz;/* buf chunk page size */
	u32			pbl_hop_num;/* multi-hop number */
};

struct hns_roce_mr_table {
	struct hns_roce_bitmap		mtpt_bitmap;
	struct hns_roce_buddy		mtt_buddy;
	struct hns_roce_hem_table	mtt_table;
	struct hns_roce_hem_table	mtpt_table;
	struct hns_roce_buddy		mtt_cqe_buddy;
	struct hns_roce_hem_table	mtt_cqe_table;
};

struct hns_roce_wq {
	u64		*wrid;     /* Work request ID */
	spinlock_t	lock;
	int		wqe_cnt;  /* WQE num */
	u32		max_post;
	int		max_gs;
	int		offset;
	int		wqe_shift;/* WQE size */
	u32		head;
	u32		tail;
	void __iomem	*db_reg_l;
};

struct hns_roce_sge {
	int		sge_cnt;  /* SGE num */
	int		offset;
	int		sge_shift;/* SGE size */
};

struct hns_roce_buf_list {
	void		*buf;
	dma_addr_t	map;
};

struct hns_roce_buf {
	struct hns_roce_buf_list	direct;
	struct hns_roce_buf_list	*page_list;
	int				nbufs;
	u32				npages;
	int				page_shift;
};

struct hns_roce_cq_buf {
	struct hns_roce_buf hr_buf;
	struct hns_roce_mtt hr_mtt;
};

struct hns_roce_cq {
	struct ib_cq			ib_cq;
	struct hns_roce_cq_buf		hr_buf;
	spinlock_t			lock;
	struct ib_umem			*umem;
	void (*comp)(struct hns_roce_cq *);
	void (*event)(struct hns_roce_cq *, enum hns_roce_event);

	struct hns_roce_uar		*uar;
	u32				cq_depth;
	u32				cons_index;
	void __iomem			*cq_db_l;
	u16				*tptr_addr;
	int				arm_sn;
	unsigned long			cqn;
	u32				vector;
	atomic_t			refcount;
	struct completion		free;
};

struct hns_roce_srq {
	struct ib_srq		ibsrq;
	int			srqn;
};

struct hns_roce_uar_table {
	struct hns_roce_bitmap bitmap;
};

struct hns_roce_qp_table {
	struct hns_roce_bitmap		bitmap;
	spinlock_t			lock;
	struct hns_roce_hem_table	qp_table;
	struct hns_roce_hem_table	irrl_table;
	struct hns_roce_hem_table	trrl_table;
};

struct hns_roce_cq_table {
	struct hns_roce_bitmap		bitmap;
	spinlock_t			lock;
	struct radix_tree_root		tree;
	struct hns_roce_hem_table	table;
};

struct hns_roce_raq_table {
	struct hns_roce_buf_list	*e_raq_buf;
};

struct hns_roce_av {
	__le32      port_pd;
	u8          gid_index;
	u8          stat_rate;
	u8          hop_limit;
	__le32      sl_tclass_flowlabel;
	u8          dgid[HNS_ROCE_GID_SIZE];
	u8          mac[6];
	__le16      vlan;
};

struct hns_roce_ah {
	struct ib_ah		ibah;
	struct hns_roce_av	av;
};

struct hns_roce_cmd_context {
	struct completion	done;
	int			result;
	int			next;
	u64			out_param;
	u16			token;
};

struct hns_roce_cmdq {
	struct dma_pool		*pool;
	struct mutex		hcr_mutex;
	struct semaphore	poll_sem;
	/*
	 * Event mode: cmd register mutex protection,
	 * ensure to not exceed max_cmds and user use limit region
	 */
	struct semaphore	event_sem;
	int			max_cmds;
	spinlock_t		context_lock;
	int			free_head;
	struct hns_roce_cmd_context *context;
	/*
	 * Result of get integer part
	 * which max_comds compute according a power of 2
	 */
	u16			token_mask;
	/*
	 * Process whether use event mode, init default non-zero
	 * After the event queue of cmd event ready,
	 * can switch into event mode
	 * close device, switch into poll mode(non event mode)
	 */
	u8			use_events;
	u8			toggle;
};

struct hns_roce_cmd_mailbox {
	void		       *buf;
	dma_addr_t		dma;
};

struct hns_roce_dev;

struct hns_roce_rinl_sge {
	void			*addr;
	u32			len;
};

struct hns_roce_rinl_wqe {
	struct hns_roce_rinl_sge *sg_list;
	u32			 sge_cnt;
};

struct hns_roce_rinl_buf {
	struct hns_roce_rinl_wqe *wqe_list;
	u32			 wqe_cnt;
};

struct hns_roce_qp {
	struct ib_qp		ibqp;
	struct hns_roce_buf	hr_buf;
	struct hns_roce_wq	rq;
	__le64			doorbell_qpn;
	__le32			sq_signal_bits;
	u32			sq_next_wqe;
	int			sq_max_wqes_per_wr;
	int			sq_spare_wqes;
	struct hns_roce_wq	sq;

	struct ib_umem		*umem;
	struct hns_roce_mtt	mtt;
	u32			buff_size;
	struct mutex		mutex;
	u8			port;
	u8			phy_port;
	u8			sl;
	u8			resp_depth;
	u8			state;
	u32			access_flags;
	u32                     atomic_rd_en;
	u32			pkey_index;
	u32			qkey;
	void			(*event)(struct hns_roce_qp *,
					 enum hns_roce_event);
	unsigned long		qpn;

	atomic_t		refcount;
	struct completion	free;

	struct hns_roce_sge	sge;
	u32			next_sge;

	struct hns_roce_rinl_buf rq_inl_buf;
};

struct hns_roce_sqp {
	struct hns_roce_qp	hr_qp;
};

struct hns_roce_ib_iboe {
	spinlock_t		lock;
	struct net_device      *netdevs[HNS_ROCE_MAX_PORTS];
	struct notifier_block	nb;
	u8			phy_port[HNS_ROCE_MAX_PORTS];
};

enum {
	HNS_ROCE_EQ_STAT_INVALID  = 0,
	HNS_ROCE_EQ_STAT_VALID    = 2,
};

struct hns_roce_ceqe {
	u32			comp;
};

struct hns_roce_aeqe {
	u32 asyn;
	union {
		struct {
			u32 qp;
			u32 rsv0;
			u32 rsv1;
		} qp_event;

		struct {
			u32 cq;
			u32 rsv0;
			u32 rsv1;
		} cq_event;

		struct {
			u32 ceqe;
			u32 rsv0;
			u32 rsv1;
		} ce_event;

		struct {
			__le64  out_param;
			__le16  token;
			u8	status;
			u8	rsv0;
		} __packed cmd;
	 } event;
};

struct hns_roce_eq {
	struct hns_roce_dev		*hr_dev;
	void __iomem			*doorbell;

	int				type_flag;/* Aeq:1 ceq:0 */
	int				eqn;
	u32				entries;
	int				log_entries;
	int				eqe_size;
	int				irq;
	int				log_page_size;
	int				cons_index;
	struct hns_roce_buf_list	*buf_list;
	int				over_ignore;
	int				coalesce;
	int				arm_st;
	u64				eqe_ba;
	int				eqe_ba_pg_sz;
	int				eqe_buf_pg_sz;
	int				hop_num;
	u64				*bt_l0;	/* Base address table for L0 */
	u64				**bt_l1; /* Base address table for L1 */
	u64				**buf;
	dma_addr_t			l0_dma;
	dma_addr_t			*l1_dma;
	dma_addr_t			*buf_dma;
	u32				l0_last_num; /* L0 last chunk num */
	u32				l1_last_num; /* L1 last chunk num */
	int				eq_max_cnt;
	int				eq_period;
	int				shift;
	dma_addr_t			cur_eqe_ba;
	dma_addr_t			nxt_eqe_ba;
};

struct hns_roce_eq_table {
	struct hns_roce_eq	*eq;
	void __iomem		**eqc_base; /* only for hw v1 */
};

struct hns_roce_caps {
	u8		num_ports;
	int		gid_table_len[HNS_ROCE_MAX_PORTS];
	int		pkey_table_len[HNS_ROCE_MAX_PORTS];
	int		local_ca_ack_delay;
	int		num_uars;
	u32		phy_num_uars;
	u32		max_sq_sg;	/* 2 */
	u32		max_sq_inline;	/* 32 */
	u32		max_rq_sg;	/* 2 */
	int		num_qps;	/* 256k */
	u32		max_wqes;	/* 16k */
	u32		max_sq_desc_sz;	/* 64 */
	u32		max_rq_desc_sz;	/* 64 */
	u32		max_srq_desc_sz;
	int		max_qp_init_rdma;
	int		max_qp_dest_rdma;
	int		num_cqs;
	int		max_cqes;
	int		min_cqes;
	u32		min_wqes;
	int		reserved_cqs;
	int		num_aeq_vectors;	/* 1 */
	int		num_comp_vectors;
	int		num_other_vectors;
	int		num_mtpts;
	u32		num_mtt_segs;
	u32		num_cqe_segs;
	int		reserved_mrws;
	int		reserved_uars;
	int		num_pds;
	int		reserved_pds;
	u32		mtt_entry_sz;
	u32		cq_entry_sz;
	u32		page_size_cap;
	u32		reserved_lkey;
	int		mtpt_entry_sz;
	int		qpc_entry_sz;
	int		irrl_entry_sz;
	int		trrl_entry_sz;
	int		cqc_entry_sz;
	u32		pbl_ba_pg_sz;
	u32		pbl_buf_pg_sz;
	u32		pbl_hop_num;
	int		aeqe_depth;
	int		ceqe_depth;
	enum ib_mtu	max_mtu;
	u32		qpc_bt_num;
	u32		srqc_bt_num;
	u32		cqc_bt_num;
	u32		mpt_bt_num;
	u32		qpc_ba_pg_sz;
	u32		qpc_buf_pg_sz;
	u32		qpc_hop_num;
	u32		srqc_ba_pg_sz;
	u32		srqc_buf_pg_sz;
	u32		srqc_hop_num;
	u32		cqc_ba_pg_sz;
	u32		cqc_buf_pg_sz;
	u32		cqc_hop_num;
	u32		mpt_ba_pg_sz;
	u32		mpt_buf_pg_sz;
	u32		mpt_hop_num;
	u32		mtt_ba_pg_sz;
	u32		mtt_buf_pg_sz;
	u32		mtt_hop_num;
	u32		cqe_ba_pg_sz;
	u32		cqe_buf_pg_sz;
	u32		cqe_hop_num;
	u32		eqe_ba_pg_sz;
	u32		eqe_buf_pg_sz;
	u32		eqe_hop_num;
	u32		chunk_sz;	/* chunk size in non multihop mode*/
	u64		flags;
};

struct hns_roce_hw {
	int (*reset)(struct hns_roce_dev *hr_dev, bool enable);
	int (*cmq_init)(struct hns_roce_dev *hr_dev);
	void (*cmq_exit)(struct hns_roce_dev *hr_dev);
	int (*hw_profile)(struct hns_roce_dev *hr_dev);
	int (*hw_init)(struct hns_roce_dev *hr_dev);
	void (*hw_exit)(struct hns_roce_dev *hr_dev);
	int (*post_mbox)(struct hns_roce_dev *hr_dev, u64 in_param,
			 u64 out_param, u32 in_modifier, u8 op_modifier, u16 op,
			 u16 token, int event);
	int (*chk_mbox)(struct hns_roce_dev *hr_dev, unsigned long timeout);
	int (*set_gid)(struct hns_roce_dev *hr_dev, u8 port, int gid_index,
		       union ib_gid *gid, const struct ib_gid_attr *attr);
	int (*set_mac)(struct hns_roce_dev *hr_dev, u8 phy_port, u8 *addr);
	void (*set_mtu)(struct hns_roce_dev *hr_dev, u8 phy_port,
			enum ib_mtu mtu);
	int (*write_mtpt)(void *mb_buf, struct hns_roce_mr *mr,
			  unsigned long mtpt_idx);
	int (*rereg_write_mtpt)(struct hns_roce_dev *hr_dev,
				struct hns_roce_mr *mr, int flags, u32 pdn,
				int mr_access_flags, u64 iova, u64 size,
				void *mb_buf);
	void (*write_cqc)(struct hns_roce_dev *hr_dev,
			  struct hns_roce_cq *hr_cq, void *mb_buf, u64 *mtts,
			  dma_addr_t dma_handle, int nent, u32 vector);
	int (*set_hem)(struct hns_roce_dev *hr_dev,
		       struct hns_roce_hem_table *table, int obj, int step_idx);
	int (*clear_hem)(struct hns_roce_dev *hr_dev,
			 struct hns_roce_hem_table *table, int obj,
			 int step_idx);
	int (*query_qp)(struct ib_qp *ibqp, struct ib_qp_attr *qp_attr,
			int qp_attr_mask, struct ib_qp_init_attr *qp_init_attr);
	int (*modify_qp)(struct ib_qp *ibqp, const struct ib_qp_attr *attr,
			 int attr_mask, enum ib_qp_state cur_state,
			 enum ib_qp_state new_state);
	int (*destroy_qp)(struct ib_qp *ibqp);
	int (*post_send)(struct ib_qp *ibqp, struct ib_send_wr *wr,
			 struct ib_send_wr **bad_wr);
	int (*post_recv)(struct ib_qp *qp, struct ib_recv_wr *recv_wr,
			 struct ib_recv_wr **bad_recv_wr);
	int (*req_notify_cq)(struct ib_cq *ibcq, enum ib_cq_notify_flags flags);
	int (*poll_cq)(struct ib_cq *ibcq, int num_entries, struct ib_wc *wc);
	int (*dereg_mr)(struct hns_roce_dev *hr_dev, struct hns_roce_mr *mr);
	int (*destroy_cq)(struct ib_cq *ibcq);
	int (*modify_cq)(struct ib_cq *cq, u16 cq_count, u16 cq_period);
	int (*init_eq)(struct hns_roce_dev *hr_dev);
	void (*cleanup_eq)(struct hns_roce_dev *hr_dev);
};

struct hns_roce_dev {
	struct ib_device	ib_dev;
	struct platform_device  *pdev;
	struct pci_dev		*pci_dev;
	struct device		*dev;
	struct hns_roce_uar     priv_uar;
	const char		*irq_names[HNS_ROCE_MAX_IRQ_NUM];
	spinlock_t		sm_lock;
	spinlock_t		bt_cmd_lock;
	struct hns_roce_ib_iboe iboe;

	int			irq[HNS_ROCE_MAX_IRQ_NUM];
	u8 __iomem		*reg_base;
	struct hns_roce_caps	caps;
	struct radix_tree_root  qp_table_tree;

	unsigned char	dev_addr[HNS_ROCE_MAX_PORTS][MAC_ADDR_OCTET_NUM];
	u64			sys_image_guid;
	u32                     vendor_id;
	u32                     vendor_part_id;
	u32                     hw_rev;
	void __iomem            *priv_addr;

	struct hns_roce_cmdq	cmd;
	struct hns_roce_bitmap    pd_bitmap;
	struct hns_roce_uar_table uar_table;
	struct hns_roce_mr_table  mr_table;
	struct hns_roce_cq_table  cq_table;
	struct hns_roce_qp_table  qp_table;
	struct hns_roce_eq_table  eq_table;

	int			cmd_mod;
	int			loop_idc;
	u32			sdb_offset;
	u32			odb_offset;
	dma_addr_t		tptr_dma_addr; /*only for hw v1*/
	u32			tptr_size; /*only for hw v1*/
	const struct hns_roce_hw *hw;
	void			*priv;
};

static inline struct hns_roce_dev *to_hr_dev(struct ib_device *ib_dev)
{
	return container_of(ib_dev, struct hns_roce_dev, ib_dev);
}

static inline struct hns_roce_ucontext
			*to_hr_ucontext(struct ib_ucontext *ibucontext)
{
	return container_of(ibucontext, struct hns_roce_ucontext, ibucontext);
}

static inline struct hns_roce_pd *to_hr_pd(struct ib_pd *ibpd)
{
	return container_of(ibpd, struct hns_roce_pd, ibpd);
}

static inline struct hns_roce_ah *to_hr_ah(struct ib_ah *ibah)
{
	return container_of(ibah, struct hns_roce_ah, ibah);
}

static inline struct hns_roce_mr *to_hr_mr(struct ib_mr *ibmr)
{
	return container_of(ibmr, struct hns_roce_mr, ibmr);
}

static inline struct hns_roce_qp *to_hr_qp(struct ib_qp *ibqp)
{
	return container_of(ibqp, struct hns_roce_qp, ibqp);
}

static inline struct hns_roce_cq *to_hr_cq(struct ib_cq *ib_cq)
{
	return container_of(ib_cq, struct hns_roce_cq, ib_cq);
}

static inline struct hns_roce_srq *to_hr_srq(struct ib_srq *ibsrq)
{
	return container_of(ibsrq, struct hns_roce_srq, ibsrq);
}

static inline struct hns_roce_sqp *hr_to_hr_sqp(struct hns_roce_qp *hr_qp)
{
	return container_of(hr_qp, struct hns_roce_sqp, hr_qp);
}

static inline void hns_roce_write64_k(__be32 val[2], void __iomem *dest)
{
	__raw_writeq(*(u64 *) val, dest);
}

static inline struct hns_roce_qp
	*__hns_roce_qp_lookup(struct hns_roce_dev *hr_dev, u32 qpn)
{
	return radix_tree_lookup(&hr_dev->qp_table_tree,
				 qpn & (hr_dev->caps.num_qps - 1));
}

static inline void *hns_roce_buf_offset(struct hns_roce_buf *buf, int offset)
{
	u32 page_size = 1 << buf->page_shift;

	if (buf->nbufs == 1)
		return (char *)(buf->direct.buf) + offset;
	else
		return (char *)(buf->page_list[offset >> buf->page_shift].buf) +
		       (offset & (page_size - 1));
}

int hns_roce_init_uar_table(struct hns_roce_dev *dev);
int hns_roce_uar_alloc(struct hns_roce_dev *dev, struct hns_roce_uar *uar);
void hns_roce_uar_free(struct hns_roce_dev *dev, struct hns_roce_uar *uar);
void hns_roce_cleanup_uar_table(struct hns_roce_dev *dev);

int hns_roce_cmd_init(struct hns_roce_dev *hr_dev);
void hns_roce_cmd_cleanup(struct hns_roce_dev *hr_dev);
void hns_roce_cmd_event(struct hns_roce_dev *hr_dev, u16 token, u8 status,
			u64 out_param);
int hns_roce_cmd_use_events(struct hns_roce_dev *hr_dev);
void hns_roce_cmd_use_polling(struct hns_roce_dev *hr_dev);

int hns_roce_mtt_init(struct hns_roce_dev *hr_dev, int npages, int page_shift,
		      struct hns_roce_mtt *mtt);
void hns_roce_mtt_cleanup(struct hns_roce_dev *hr_dev,
			  struct hns_roce_mtt *mtt);
int hns_roce_buf_write_mtt(struct hns_roce_dev *hr_dev,
			   struct hns_roce_mtt *mtt, struct hns_roce_buf *buf);

int hns_roce_init_pd_table(struct hns_roce_dev *hr_dev);
int hns_roce_init_mr_table(struct hns_roce_dev *hr_dev);
int hns_roce_init_eq_table(struct hns_roce_dev *hr_dev);
int hns_roce_init_cq_table(struct hns_roce_dev *hr_dev);
int hns_roce_init_qp_table(struct hns_roce_dev *hr_dev);

void hns_roce_cleanup_pd_table(struct hns_roce_dev *hr_dev);
void hns_roce_cleanup_mr_table(struct hns_roce_dev *hr_dev);
void hns_roce_cleanup_eq_table(struct hns_roce_dev *hr_dev);
void hns_roce_cleanup_cq_table(struct hns_roce_dev *hr_dev);
void hns_roce_cleanup_qp_table(struct hns_roce_dev *hr_dev);

int hns_roce_bitmap_alloc(struct hns_roce_bitmap *bitmap, unsigned long *obj);
void hns_roce_bitmap_free(struct hns_roce_bitmap *bitmap, unsigned long obj,
			 int rr);
int hns_roce_bitmap_init(struct hns_roce_bitmap *bitmap, u32 num, u32 mask,
			 u32 reserved_bot, u32 resetrved_top);
void hns_roce_bitmap_cleanup(struct hns_roce_bitmap *bitmap);
void hns_roce_cleanup_bitmap(struct hns_roce_dev *hr_dev);
int hns_roce_bitmap_alloc_range(struct hns_roce_bitmap *bitmap, int cnt,
				int align, unsigned long *obj);
void hns_roce_bitmap_free_range(struct hns_roce_bitmap *bitmap,
				unsigned long obj, int cnt,
				int rr);

struct ib_ah *hns_roce_create_ah(struct ib_pd *pd,
				 struct rdma_ah_attr *ah_attr,
				 struct ib_udata *udata);
int hns_roce_query_ah(struct ib_ah *ibah, struct rdma_ah_attr *ah_attr);
int hns_roce_destroy_ah(struct ib_ah *ah);

struct ib_pd *hns_roce_alloc_pd(struct ib_device *ib_dev,
				struct ib_ucontext *context,
				struct ib_udata *udata);
int hns_roce_dealloc_pd(struct ib_pd *pd);

struct ib_mr *hns_roce_get_dma_mr(struct ib_pd *pd, int acc);
struct ib_mr *hns_roce_reg_user_mr(struct ib_pd *pd, u64 start, u64 length,
				   u64 virt_addr, int access_flags,
				   struct ib_udata *udata);
int hns_roce_rereg_user_mr(struct ib_mr *mr, int flags, u64 start, u64 length,
			   u64 virt_addr, int mr_access_flags, struct ib_pd *pd,
			   struct ib_udata *udata);
int hns_roce_dereg_mr(struct ib_mr *ibmr);
int hns_roce_hw2sw_mpt(struct hns_roce_dev *hr_dev,
		       struct hns_roce_cmd_mailbox *mailbox,
		       unsigned long mpt_index);
unsigned long key_to_hw_index(u32 key);

void hns_roce_buf_free(struct hns_roce_dev *hr_dev, u32 size,
		       struct hns_roce_buf *buf);
int hns_roce_buf_alloc(struct hns_roce_dev *hr_dev, u32 size, u32 max_direct,
		       struct hns_roce_buf *buf, u32 page_shift);

int hns_roce_ib_umem_write_mtt(struct hns_roce_dev *hr_dev,
			       struct hns_roce_mtt *mtt, struct ib_umem *umem);

struct ib_qp *hns_roce_create_qp(struct ib_pd *ib_pd,
				 struct ib_qp_init_attr *init_attr,
				 struct ib_udata *udata);
int hns_roce_modify_qp(struct ib_qp *ibqp, struct ib_qp_attr *attr,
		       int attr_mask, struct ib_udata *udata);
void *get_recv_wqe(struct hns_roce_qp *hr_qp, int n);
void *get_send_wqe(struct hns_roce_qp *hr_qp, int n);
void *get_send_extend_sge(struct hns_roce_qp *hr_qp, int n);
bool hns_roce_wq_overflow(struct hns_roce_wq *hr_wq, int nreq,
			  struct ib_cq *ib_cq);
enum hns_roce_qp_state to_hns_roce_state(enum ib_qp_state state);
void hns_roce_lock_cqs(struct hns_roce_cq *send_cq,
		       struct hns_roce_cq *recv_cq);
void hns_roce_unlock_cqs(struct hns_roce_cq *send_cq,
			 struct hns_roce_cq *recv_cq);
void hns_roce_qp_remove(struct hns_roce_dev *hr_dev, struct hns_roce_qp *hr_qp);
void hns_roce_qp_free(struct hns_roce_dev *hr_dev, struct hns_roce_qp *hr_qp);
void hns_roce_release_range_qp(struct hns_roce_dev *hr_dev, int base_qpn,
			       int cnt);
__be32 send_ieth(struct ib_send_wr *wr);
int to_hr_qp_type(int qp_type);

struct ib_cq *hns_roce_ib_create_cq(struct ib_device *ib_dev,
				    const struct ib_cq_init_attr *attr,
				    struct ib_ucontext *context,
				    struct ib_udata *udata);

int hns_roce_ib_destroy_cq(struct ib_cq *ib_cq);
void hns_roce_free_cq(struct hns_roce_dev *hr_dev, struct hns_roce_cq *hr_cq);

void hns_roce_cq_completion(struct hns_roce_dev *hr_dev, u32 cqn);
void hns_roce_cq_event(struct hns_roce_dev *hr_dev, u32 cqn, int event_type);
void hns_roce_qp_event(struct hns_roce_dev *hr_dev, u32 qpn, int event_type);
int hns_get_gid_index(struct hns_roce_dev *hr_dev, u8 port, int gid_index);
int hns_roce_init(struct hns_roce_dev *hr_dev);
void hns_roce_exit(struct hns_roce_dev *hr_dev);

#endif /* _HNS_ROCE_DEVICE_H */
