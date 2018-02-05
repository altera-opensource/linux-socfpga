/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Shared Memory Communications over RDMA (SMC-R) and RoCE
 *
 *  Definitions for SMC Connections, Link Groups and Links
 *
 *  Copyright IBM Corp. 2016
 *
 *  Author(s):  Ursula Braun <ubraun@linux.vnet.ibm.com>
 */

#ifndef _SMC_CORE_H
#define _SMC_CORE_H

#include <linux/atomic.h>
#include <rdma/ib_verbs.h>

#include "smc.h"
#include "smc_ib.h"

#define SMC_RMBS_PER_LGR_MAX	255	/* max. # of RMBs per link group */

struct smc_lgr_list {			/* list of link group definition */
	struct list_head	list;
	spinlock_t		lock;	/* protects list of link groups */
};

extern struct smc_lgr_list	smc_lgr_list; /* list of link groups */

enum smc_lgr_role {		/* possible roles of a link group */
	SMC_CLNT,	/* client */
	SMC_SERV	/* server */
};

#define SMC_WR_BUF_SIZE		48	/* size of work request buffer */

struct smc_wr_buf {
	u8	raw[SMC_WR_BUF_SIZE];
};

#define SMC_WR_REG_MR_WAIT_TIME	(5 * HZ)/* wait time for ib_wr_reg_mr result */

enum smc_wr_reg_state {
	POSTED,		/* ib_wr_reg_mr request posted */
	CONFIRMED,	/* ib_wr_reg_mr response: successful */
	FAILED		/* ib_wr_reg_mr response: failure */
};

struct smc_link {
	struct smc_ib_device	*smcibdev;	/* ib-device */
	u8			ibport;		/* port - values 1 | 2 */
	struct ib_pd		*roce_pd;	/* IB protection domain,
						 * unique for every RoCE QP
						 */
	struct ib_qp		*roce_qp;	/* IB queue pair */
	struct ib_qp_attr	qp_attr;	/* IB queue pair attributes */

	struct smc_wr_buf	*wr_tx_bufs;	/* WR send payload buffers */
	struct ib_send_wr	*wr_tx_ibs;	/* WR send meta data */
	struct ib_sge		*wr_tx_sges;	/* WR send gather meta data */
	struct smc_wr_tx_pend	*wr_tx_pends;	/* WR send waiting for CQE */
	/* above four vectors have wr_tx_cnt elements and use the same index */
	dma_addr_t		wr_tx_dma_addr;	/* DMA address of wr_tx_bufs */
	atomic_long_t		wr_tx_id;	/* seq # of last sent WR */
	unsigned long		*wr_tx_mask;	/* bit mask of used indexes */
	u32			wr_tx_cnt;	/* number of WR send buffers */
	wait_queue_head_t	wr_tx_wait;	/* wait for free WR send buf */

	struct smc_wr_buf	*wr_rx_bufs;	/* WR recv payload buffers */
	struct ib_recv_wr	*wr_rx_ibs;	/* WR recv meta data */
	struct ib_sge		*wr_rx_sges;	/* WR recv scatter meta data */
	/* above three vectors have wr_rx_cnt elements and use the same index */
	dma_addr_t		wr_rx_dma_addr;	/* DMA address of wr_rx_bufs */
	u64			wr_rx_id;	/* seq # of last recv WR */
	u32			wr_rx_cnt;	/* number of WR recv buffers */

	struct ib_reg_wr	wr_reg;		/* WR register memory region */
	wait_queue_head_t	wr_reg_wait;	/* wait for wr_reg result */
	enum smc_wr_reg_state	wr_reg_state;	/* state of wr_reg request */

	union ib_gid		gid;		/* gid matching used vlan id */
	u32			peer_qpn;	/* QP number of peer */
	enum ib_mtu		path_mtu;	/* used mtu */
	enum ib_mtu		peer_mtu;	/* mtu size of peer */
	u32			psn_initial;	/* QP tx initial packet seqno */
	u32			peer_psn;	/* QP rx initial packet seqno */
	u8			peer_mac[ETH_ALEN];	/* = gid[8:10||13:15] */
	u8			peer_gid[sizeof(union ib_gid)];	/* gid of peer*/
	u8			link_id;	/* unique # within link group */
	struct completion	llc_confirm;	/* wait for rx of conf link */
	struct completion	llc_confirm_resp; /* wait 4 rx of cnf lnk rsp */
};

/* For now we just allow one parallel link per link group. The SMC protocol
 * allows more (up to 8).
 */
#define SMC_LINKS_PER_LGR_MAX	1
#define SMC_SINGLE_LINK		0

#define SMC_FIRST_CONTACT	1		/* first contact to a peer */
#define SMC_REUSE_CONTACT	0		/* follow-on contact to a peer*/

/* tx/rx buffer list element for sndbufs list and rmbs list of a lgr */
struct smc_buf_desc {
	struct list_head	list;
	void			*cpu_addr;	/* virtual address of buffer */
	struct sg_table		sgt[SMC_LINKS_PER_LGR_MAX];/* virtual buffer */
	struct ib_mr		*mr_rx[SMC_LINKS_PER_LGR_MAX];
						/* for rmb only: memory region
						 * incl. rkey provided to peer
						 */
	u32			order;		/* allocation order */
	u32			used;		/* currently used / unused */
	bool			reused;		/* new created / reused */
};

struct smc_rtoken {				/* address/key of remote RMB */
	u64			dma_addr;
	u32			rkey;
};

#define SMC_LGR_ID_SIZE		4

struct smc_link_group {
	struct list_head	list;
	enum smc_lgr_role	role;		/* client or server */
	__be32			daddr;		/* destination ip address */
	struct smc_link		lnk[SMC_LINKS_PER_LGR_MAX];	/* smc link */
	char			peer_systemid[SMC_SYSTEMID_LEN];
						/* unique system_id of peer */
	struct rb_root		conns_all;	/* connection tree */
	rwlock_t		conns_lock;	/* protects conns_all */
	unsigned int		conns_num;	/* current # of connections */
	unsigned short		vlan_id;	/* vlan id of link group */

	struct list_head	sndbufs[SMC_RMBE_SIZES];/* tx buffers */
	rwlock_t		sndbufs_lock;	/* protects tx buffers */
	struct list_head	rmbs[SMC_RMBE_SIZES];	/* rx buffers */
	rwlock_t		rmbs_lock;	/* protects rx buffers */
	struct smc_rtoken	rtokens[SMC_RMBS_PER_LGR_MAX]
				       [SMC_LINKS_PER_LGR_MAX];
						/* remote addr/key pairs */
	unsigned long		rtokens_used_mask[BITS_TO_LONGS(
							SMC_RMBS_PER_LGR_MAX)];
						/* used rtoken elements */

	u8			id[SMC_LGR_ID_SIZE];	/* unique lgr id */
	struct delayed_work	free_work;	/* delayed freeing of an lgr */
	bool			sync_err;	/* lgr no longer fits to peer */
};

/* Find the connection associated with the given alert token in the link group.
 * To use rbtrees we have to implement our own search core.
 * Requires @conns_lock
 * @token	alert token to search for
 * @lgr		 link group to search in
 * Returns connection associated with token if found, NULL otherwise.
 */
static inline struct smc_connection *smc_lgr_find_conn(
	u32 token, struct smc_link_group *lgr)
{
	struct smc_connection *res = NULL;
	struct rb_node *node;

	node = lgr->conns_all.rb_node;
	while (node) {
		struct smc_connection *cur = rb_entry(node,
					struct smc_connection, alert_node);

		if (cur->alert_token_local > token) {
			node = node->rb_left;
		} else {
			if (cur->alert_token_local < token) {
				node = node->rb_right;
			} else {
				res = cur;
				break;
			}
		}
	}

	return res;
}

struct smc_sock;
struct smc_clc_msg_accept_confirm;

void smc_lgr_free(struct smc_link_group *lgr);
void smc_lgr_terminate(struct smc_link_group *lgr);
int smc_buf_create(struct smc_sock *smc);
int smc_rmb_rtoken_handling(struct smc_connection *conn,
			    struct smc_clc_msg_accept_confirm *clc);
void smc_sndbuf_sync_sg_for_cpu(struct smc_connection *conn);
void smc_sndbuf_sync_sg_for_device(struct smc_connection *conn);
void smc_rmb_sync_sg_for_cpu(struct smc_connection *conn);
void smc_rmb_sync_sg_for_device(struct smc_connection *conn);
#endif
