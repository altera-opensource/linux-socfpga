/*
 * Copyright (C) 2016-2017 Netronome Systems, Inc.
 *
 * This software is dual licensed under the GNU General License Version 2,
 * June 1991 as shown in the file COPYING in the top-level directory of this
 * source tree or the BSD 2-Clause License provided below.  You have the
 * option to license this software under the complete terms of either license.
 *
 * The BSD 2-Clause License:
 *
 *     Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *      1. Redistributions of source code must retain the above
 *         copyright notice, this list of conditions and the following
 *         disclaimer.
 *
 *      2. Redistributions in binary form must reproduce the above
 *         copyright notice, this list of conditions and the following
 *         disclaimer in the documentation and/or other materials
 *         provided with the distribution.
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

#ifndef __NFP_BPF_H__
#define __NFP_BPF_H__ 1

#include <linux/bitfield.h>
#include <linux/bpf.h>
#include <linux/bpf_verifier.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/skbuff.h>
#include <linux/types.h>
#include <linux/wait.h>

#include "../nfp_asm.h"
#include "fw.h"

/* For relocation logic use up-most byte of branch instruction as scratch
 * area.  Remember to clear this before sending instructions to HW!
 */
#define OP_RELO_TYPE	0xff00000000000000ULL

enum nfp_relo_type {
	RELO_NONE = 0,
	/* standard internal jumps */
	RELO_BR_REL,
	/* internal jumps to parts of the outro */
	RELO_BR_GO_OUT,
	RELO_BR_GO_ABORT,
	/* external jumps to fixed addresses */
	RELO_BR_NEXT_PKT,
	RELO_BR_HELPER,
	/* immediate relocation against load address */
	RELO_IMMED_REL,
};

/* To make absolute relocated branches (branches other than RELO_BR_REL)
 * distinguishable in user space dumps from normal jumps, add a large offset
 * to them.
 */
#define BR_OFF_RELO		15000

enum static_regs {
	STATIC_REG_IMM		= 21, /* Bank AB */
	STATIC_REG_STACK	= 22, /* Bank A */
	STATIC_REG_PKT_LEN	= 22, /* Bank B */
};

enum pkt_vec {
	PKT_VEC_PKT_LEN		= 0,
	PKT_VEC_PKT_PTR		= 2,
};

#define pv_len(np)	reg_lm(1, PKT_VEC_PKT_LEN)
#define pv_ctm_ptr(np)	reg_lm(1, PKT_VEC_PKT_PTR)

#define stack_reg(np)	reg_a(STATIC_REG_STACK)
#define stack_imm(np)	imm_b(np)
#define plen_reg(np)	reg_b(STATIC_REG_PKT_LEN)
#define pptr_reg(np)	pv_ctm_ptr(np)
#define imm_a(np)	reg_a(STATIC_REG_IMM)
#define imm_b(np)	reg_b(STATIC_REG_IMM)
#define imm_both(np)	reg_both(STATIC_REG_IMM)

#define NFP_BPF_ABI_FLAGS	reg_imm(0)
#define   NFP_BPF_ABI_FLAG_MARK	1

/**
 * struct nfp_app_bpf - bpf app priv structure
 * @app:		backpointer to the app
 *
 * @tag_allocator:	bitmap of control message tags in use
 * @tag_alloc_next:	next tag bit to allocate
 * @tag_alloc_last:	next tag bit to be freed
 *
 * @cmsg_replies:	received cmsg replies waiting to be consumed
 * @cmsg_wq:		work queue for waiting for cmsg replies
 *
 * @map_list:		list of offloaded maps
 * @maps_in_use:	number of currently offloaded maps
 * @map_elems_in_use:	number of elements allocated to offloaded maps
 *
 * @adjust_head:	adjust head capability
 * @flags:		extra flags for adjust head
 * @off_min:		minimal packet offset within buffer required
 * @off_max:		maximum packet offset within buffer required
 * @guaranteed_sub:	amount of negative adjustment guaranteed possible
 * @guaranteed_add:	amount of positive adjustment guaranteed possible
 *
 * @maps:		map capability
 * @types:		supported map types
 * @max_maps:		max number of maps supported
 * @max_elems:		max number of entries in each map
 * @max_key_sz:		max size of map key
 * @max_val_sz:		max size of map value
 * @max_elem_sz:	max size of map entry (key + value)
 *
 * @helpers:		helper addressess for various calls
 * @map_lookup:		map lookup helper address
 */
struct nfp_app_bpf {
	struct nfp_app *app;

	DECLARE_BITMAP(tag_allocator, U16_MAX + 1);
	u16 tag_alloc_next;
	u16 tag_alloc_last;

	struct sk_buff_head cmsg_replies;
	struct wait_queue_head cmsg_wq;

	struct list_head map_list;
	unsigned int maps_in_use;
	unsigned int map_elems_in_use;

	struct nfp_bpf_cap_adjust_head {
		u32 flags;
		int off_min;
		int off_max;
		int guaranteed_sub;
		int guaranteed_add;
	} adjust_head;

	struct {
		u32 types;
		u32 max_maps;
		u32 max_elems;
		u32 max_key_sz;
		u32 max_val_sz;
		u32 max_elem_sz;
	} maps;

	struct {
		u32 map_lookup;
	} helpers;
};

/**
 * struct nfp_bpf_map - private per-map data attached to BPF maps for offload
 * @offmap:	pointer to the offloaded BPF map
 * @bpf:	back pointer to bpf app private structure
 * @tid:	table id identifying map on datapath
 * @l:		link on the nfp_app_bpf->map_list list
 */
struct nfp_bpf_map {
	struct bpf_offloaded_map *offmap;
	struct nfp_app_bpf *bpf;
	u32 tid;
	struct list_head l;
};

struct nfp_prog;
struct nfp_insn_meta;
typedef int (*instr_cb_t)(struct nfp_prog *, struct nfp_insn_meta *);

#define nfp_prog_first_meta(nfp_prog)					\
	list_first_entry(&(nfp_prog)->insns, struct nfp_insn_meta, l)
#define nfp_prog_last_meta(nfp_prog)					\
	list_last_entry(&(nfp_prog)->insns, struct nfp_insn_meta, l)
#define nfp_meta_next(meta)	list_next_entry(meta, l)
#define nfp_meta_prev(meta)	list_prev_entry(meta, l)

#define FLAG_INSN_IS_JUMP_DST	BIT(0)

/**
 * struct nfp_insn_meta - BPF instruction wrapper
 * @insn: BPF instruction
 * @ptr: pointer type for memory operations
 * @ldst_gather_len: memcpy length gathered from load/store sequence
 * @paired_st: the paired store insn at the head of the sequence
 * @ptr_not_const: pointer is not always constant
 * @jmp_dst: destination info for jump instructions
 * @func_id: function id for call instructions
 * @arg1: arg1 for call instructions
 * @arg2: arg2 for call instructions
 * @arg2_var_off: arg2 changes stack offset on different paths
 * @off: index of first generated machine instruction (in nfp_prog.prog)
 * @n: eBPF instruction number
 * @flags: eBPF instruction extra optimization flags
 * @skip: skip this instruction (optimized out)
 * @double_cb: callback for second part of the instruction
 * @l: link on nfp_prog->insns list
 */
struct nfp_insn_meta {
	struct bpf_insn insn;
	union {
		struct {
			struct bpf_reg_state ptr;
			struct bpf_insn *paired_st;
			s16 ldst_gather_len;
			bool ptr_not_const;
		};
		struct nfp_insn_meta *jmp_dst;
		struct {
			u32 func_id;
			struct bpf_reg_state arg1;
			struct bpf_reg_state arg2;
			bool arg2_var_off;
		};
	};
	unsigned int off;
	unsigned short n;
	unsigned short flags;
	bool skip;
	instr_cb_t double_cb;

	struct list_head l;
};

#define BPF_SIZE_MASK	0x18

static inline u8 mbpf_class(const struct nfp_insn_meta *meta)
{
	return BPF_CLASS(meta->insn.code);
}

static inline u8 mbpf_src(const struct nfp_insn_meta *meta)
{
	return BPF_SRC(meta->insn.code);
}

static inline u8 mbpf_op(const struct nfp_insn_meta *meta)
{
	return BPF_OP(meta->insn.code);
}

static inline u8 mbpf_mode(const struct nfp_insn_meta *meta)
{
	return BPF_MODE(meta->insn.code);
}

static inline bool is_mbpf_load(const struct nfp_insn_meta *meta)
{
	return (meta->insn.code & ~BPF_SIZE_MASK) == (BPF_LDX | BPF_MEM);
}

static inline bool is_mbpf_store(const struct nfp_insn_meta *meta)
{
	return (meta->insn.code & ~BPF_SIZE_MASK) == (BPF_STX | BPF_MEM);
}

/**
 * struct nfp_prog - nfp BPF program
 * @bpf: backpointer to the bpf app priv structure
 * @prog: machine code
 * @prog_len: number of valid instructions in @prog array
 * @__prog_alloc_len: alloc size of @prog array
 * @verifier_meta: temporary storage for verifier's insn meta
 * @type: BPF program type
 * @last_bpf_off: address of the last instruction translated from BPF
 * @tgt_out: jump target for normal exit
 * @tgt_abort: jump target for abort (e.g. access outside of packet buffer)
 * @n_translated: number of successfully translated instructions (for errors)
 * @error: error code if something went wrong
 * @stack_depth: max stack depth from the verifier
 * @adjust_head_location: if program has single adjust head call - the insn no.
 * @insns: list of BPF instruction wrappers (struct nfp_insn_meta)
 */
struct nfp_prog {
	struct nfp_app_bpf *bpf;

	u64 *prog;
	unsigned int prog_len;
	unsigned int __prog_alloc_len;

	struct nfp_insn_meta *verifier_meta;

	enum bpf_prog_type type;

	unsigned int last_bpf_off;
	unsigned int tgt_out;
	unsigned int tgt_abort;

	unsigned int n_translated;
	int error;

	unsigned int stack_depth;
	unsigned int adjust_head_location;

	struct list_head insns;
};

/**
 * struct nfp_bpf_vnic - per-vNIC BPF priv structure
 * @tc_prog:	currently loaded cls_bpf program
 * @start_off:	address of the first instruction in the memory
 * @tgt_done:	jump target to get the next packet
 */
struct nfp_bpf_vnic {
	struct bpf_prog *tc_prog;
	unsigned int start_off;
	unsigned int tgt_done;
};

void nfp_bpf_jit_prepare(struct nfp_prog *nfp_prog, unsigned int cnt);
int nfp_bpf_jit(struct nfp_prog *prog);
bool nfp_bpf_supported_opcode(u8 code);

extern const struct bpf_prog_offload_ops nfp_bpf_analyzer_ops;

struct netdev_bpf;
struct nfp_app;
struct nfp_net;

int nfp_ndo_bpf(struct nfp_app *app, struct nfp_net *nn,
		struct netdev_bpf *bpf);
int nfp_net_bpf_offload(struct nfp_net *nn, struct bpf_prog *prog,
			bool old_prog, struct netlink_ext_ack *extack);

struct nfp_insn_meta *
nfp_bpf_goto_meta(struct nfp_prog *nfp_prog, struct nfp_insn_meta *meta,
		  unsigned int insn_idx, unsigned int n_insns);

void *nfp_bpf_relo_for_vnic(struct nfp_prog *nfp_prog, struct nfp_bpf_vnic *bv);

long long int
nfp_bpf_ctrl_alloc_map(struct nfp_app_bpf *bpf, struct bpf_map *map);
void
nfp_bpf_ctrl_free_map(struct nfp_app_bpf *bpf, struct nfp_bpf_map *nfp_map);
int nfp_bpf_ctrl_getfirst_entry(struct bpf_offloaded_map *offmap,
				void *next_key);
int nfp_bpf_ctrl_update_entry(struct bpf_offloaded_map *offmap,
			      void *key, void *value, u64 flags);
int nfp_bpf_ctrl_del_entry(struct bpf_offloaded_map *offmap, void *key);
int nfp_bpf_ctrl_lookup_entry(struct bpf_offloaded_map *offmap,
			      void *key, void *value);
int nfp_bpf_ctrl_getnext_entry(struct bpf_offloaded_map *offmap,
			       void *key, void *next_key);

void nfp_bpf_ctrl_msg_rx(struct nfp_app *app, struct sk_buff *skb);
#endif
