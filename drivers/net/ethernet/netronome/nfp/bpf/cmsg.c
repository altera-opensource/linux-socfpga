/*
 * Copyright (C) 2017 Netronome Systems, Inc.
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

#include <linux/bpf.h>
#include <linux/bitops.h>
#include <linux/bug.h>
#include <linux/jiffies.h>
#include <linux/skbuff.h>
#include <linux/wait.h>

#include "../nfp_app.h"
#include "../nfp_net.h"
#include "fw.h"
#include "main.h"

#define cmsg_warn(bpf, msg...)	nn_dp_warn(&(bpf)->app->ctrl->dp, msg)

#define NFP_BPF_TAG_ALLOC_SPAN	(U16_MAX / 4)

static bool nfp_bpf_all_tags_busy(struct nfp_app_bpf *bpf)
{
	u16 used_tags;

	used_tags = bpf->tag_alloc_next - bpf->tag_alloc_last;

	return used_tags > NFP_BPF_TAG_ALLOC_SPAN;
}

static int nfp_bpf_alloc_tag(struct nfp_app_bpf *bpf)
{
	/* All FW communication for BPF is request-reply.  To make sure we
	 * don't reuse the message ID too early after timeout - limit the
	 * number of requests in flight.
	 */
	if (nfp_bpf_all_tags_busy(bpf)) {
		cmsg_warn(bpf, "all FW request contexts busy!\n");
		return -EAGAIN;
	}

	WARN_ON(__test_and_set_bit(bpf->tag_alloc_next, bpf->tag_allocator));
	return bpf->tag_alloc_next++;
}

static void nfp_bpf_free_tag(struct nfp_app_bpf *bpf, u16 tag)
{
	WARN_ON(!__test_and_clear_bit(tag, bpf->tag_allocator));

	while (!test_bit(bpf->tag_alloc_last, bpf->tag_allocator) &&
	       bpf->tag_alloc_last != bpf->tag_alloc_next)
		bpf->tag_alloc_last++;
}

static struct sk_buff *
nfp_bpf_cmsg_alloc(struct nfp_app_bpf *bpf, unsigned int size)
{
	struct sk_buff *skb;

	skb = nfp_app_ctrl_msg_alloc(bpf->app, size, GFP_KERNEL);
	skb_put(skb, size);

	return skb;
}

static struct sk_buff *
nfp_bpf_cmsg_map_req_alloc(struct nfp_app_bpf *bpf, unsigned int n)
{
	unsigned int size;

	size = sizeof(struct cmsg_req_map_op);
	size += sizeof(struct cmsg_key_value_pair) * n;

	return nfp_bpf_cmsg_alloc(bpf, size);
}

static unsigned int nfp_bpf_cmsg_get_tag(struct sk_buff *skb)
{
	struct cmsg_hdr *hdr;

	hdr = (struct cmsg_hdr *)skb->data;

	return be16_to_cpu(hdr->tag);
}

static struct sk_buff *__nfp_bpf_reply(struct nfp_app_bpf *bpf, u16 tag)
{
	unsigned int msg_tag;
	struct sk_buff *skb;

	skb_queue_walk(&bpf->cmsg_replies, skb) {
		msg_tag = nfp_bpf_cmsg_get_tag(skb);
		if (msg_tag == tag) {
			nfp_bpf_free_tag(bpf, tag);
			__skb_unlink(skb, &bpf->cmsg_replies);
			return skb;
		}
	}

	return NULL;
}

static struct sk_buff *nfp_bpf_reply(struct nfp_app_bpf *bpf, u16 tag)
{
	struct sk_buff *skb;

	nfp_ctrl_lock(bpf->app->ctrl);
	skb = __nfp_bpf_reply(bpf, tag);
	nfp_ctrl_unlock(bpf->app->ctrl);

	return skb;
}

static struct sk_buff *nfp_bpf_reply_drop_tag(struct nfp_app_bpf *bpf, u16 tag)
{
	struct sk_buff *skb;

	nfp_ctrl_lock(bpf->app->ctrl);
	skb = __nfp_bpf_reply(bpf, tag);
	if (!skb)
		nfp_bpf_free_tag(bpf, tag);
	nfp_ctrl_unlock(bpf->app->ctrl);

	return skb;
}

static struct sk_buff *
nfp_bpf_cmsg_wait_reply(struct nfp_app_bpf *bpf, enum nfp_bpf_cmsg_type type,
			int tag)
{
	struct sk_buff *skb;
	int i, err;

	for (i = 0; i < 50; i++) {
		udelay(4);
		skb = nfp_bpf_reply(bpf, tag);
		if (skb)
			return skb;
	}

	err = wait_event_interruptible_timeout(bpf->cmsg_wq,
					       skb = nfp_bpf_reply(bpf, tag),
					       msecs_to_jiffies(5000));
	/* We didn't get a response - try last time and atomically drop
	 * the tag even if no response is matched.
	 */
	if (!skb)
		skb = nfp_bpf_reply_drop_tag(bpf, tag);
	if (err < 0) {
		cmsg_warn(bpf, "%s waiting for response to 0x%02x: %d\n",
			  err == ERESTARTSYS ? "interrupted" : "error",
			  type, err);
		return ERR_PTR(err);
	}
	if (!skb) {
		cmsg_warn(bpf, "timeout waiting for response to 0x%02x\n",
			  type);
		return ERR_PTR(-ETIMEDOUT);
	}

	return skb;
}

static struct sk_buff *
nfp_bpf_cmsg_communicate(struct nfp_app_bpf *bpf, struct sk_buff *skb,
			 enum nfp_bpf_cmsg_type type, unsigned int reply_size)
{
	struct cmsg_hdr *hdr;
	int tag;

	nfp_ctrl_lock(bpf->app->ctrl);
	tag = nfp_bpf_alloc_tag(bpf);
	if (tag < 0) {
		nfp_ctrl_unlock(bpf->app->ctrl);
		dev_kfree_skb_any(skb);
		return ERR_PTR(tag);
	}

	hdr = (void *)skb->data;
	hdr->ver = CMSG_MAP_ABI_VERSION;
	hdr->type = type;
	hdr->tag = cpu_to_be16(tag);

	__nfp_app_ctrl_tx(bpf->app, skb);

	nfp_ctrl_unlock(bpf->app->ctrl);

	skb = nfp_bpf_cmsg_wait_reply(bpf, type, tag);
	if (IS_ERR(skb))
		return skb;

	hdr = (struct cmsg_hdr *)skb->data;
	/* 0 reply_size means caller will do the validation */
	if (reply_size && skb->len != reply_size) {
		cmsg_warn(bpf, "cmsg drop - wrong size %d != %d!\n",
			  skb->len, reply_size);
		goto err_free;
	}
	if (hdr->type != __CMSG_REPLY(type)) {
		cmsg_warn(bpf, "cmsg drop - wrong type 0x%02x != 0x%02lx!\n",
			  hdr->type, __CMSG_REPLY(type));
		goto err_free;
	}

	return skb;
err_free:
	dev_kfree_skb_any(skb);
	return ERR_PTR(-EIO);
}

static int
nfp_bpf_ctrl_rc_to_errno(struct nfp_app_bpf *bpf,
			 struct cmsg_reply_map_simple *reply)
{
	static const int res_table[] = {
		[CMSG_RC_SUCCESS]	= 0,
		[CMSG_RC_ERR_MAP_FD]	= -EBADFD,
		[CMSG_RC_ERR_MAP_NOENT]	= -ENOENT,
		[CMSG_RC_ERR_MAP_ERR]	= -EINVAL,
		[CMSG_RC_ERR_MAP_PARSE]	= -EIO,
		[CMSG_RC_ERR_MAP_EXIST]	= -EEXIST,
		[CMSG_RC_ERR_MAP_NOMEM]	= -ENOMEM,
		[CMSG_RC_ERR_MAP_E2BIG]	= -E2BIG,
	};
	u32 rc;

	rc = be32_to_cpu(reply->rc);
	if (rc >= ARRAY_SIZE(res_table)) {
		cmsg_warn(bpf, "FW responded with invalid status: %u\n", rc);
		return -EIO;
	}

	return res_table[rc];
}

long long int
nfp_bpf_ctrl_alloc_map(struct nfp_app_bpf *bpf, struct bpf_map *map)
{
	struct cmsg_reply_map_alloc_tbl *reply;
	struct cmsg_req_map_alloc_tbl *req;
	struct sk_buff *skb;
	u32 tid;
	int err;

	skb = nfp_bpf_cmsg_alloc(bpf, sizeof(*req));
	if (!skb)
		return -ENOMEM;

	req = (void *)skb->data;
	req->key_size = cpu_to_be32(map->key_size);
	req->value_size = cpu_to_be32(map->value_size);
	req->max_entries = cpu_to_be32(map->max_entries);
	req->map_type = cpu_to_be32(map->map_type);
	req->map_flags = 0;

	skb = nfp_bpf_cmsg_communicate(bpf, skb, CMSG_TYPE_MAP_ALLOC,
				       sizeof(*reply));
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	reply = (void *)skb->data;
	err = nfp_bpf_ctrl_rc_to_errno(bpf, &reply->reply_hdr);
	if (err)
		goto err_free;

	tid = be32_to_cpu(reply->tid);
	dev_consume_skb_any(skb);

	return tid;
err_free:
	dev_kfree_skb_any(skb);
	return err;
}

void nfp_bpf_ctrl_free_map(struct nfp_app_bpf *bpf, struct nfp_bpf_map *nfp_map)
{
	struct cmsg_reply_map_free_tbl *reply;
	struct cmsg_req_map_free_tbl *req;
	struct sk_buff *skb;
	int err;

	skb = nfp_bpf_cmsg_alloc(bpf, sizeof(*req));
	if (!skb) {
		cmsg_warn(bpf, "leaking map - failed to allocate msg\n");
		return;
	}

	req = (void *)skb->data;
	req->tid = cpu_to_be32(nfp_map->tid);

	skb = nfp_bpf_cmsg_communicate(bpf, skb, CMSG_TYPE_MAP_FREE,
				       sizeof(*reply));
	if (IS_ERR(skb)) {
		cmsg_warn(bpf, "leaking map - I/O error\n");
		return;
	}

	reply = (void *)skb->data;
	err = nfp_bpf_ctrl_rc_to_errno(bpf, &reply->reply_hdr);
	if (err)
		cmsg_warn(bpf, "leaking map - FW responded with: %d\n", err);

	dev_consume_skb_any(skb);
}

static int
nfp_bpf_ctrl_entry_op(struct bpf_offloaded_map *offmap,
		      enum nfp_bpf_cmsg_type op,
		      u8 *key, u8 *value, u64 flags, u8 *out_key, u8 *out_value)
{
	struct nfp_bpf_map *nfp_map = offmap->dev_priv;
	struct nfp_app_bpf *bpf = nfp_map->bpf;
	struct bpf_map *map = &offmap->map;
	struct cmsg_reply_map_op *reply;
	struct cmsg_req_map_op *req;
	struct sk_buff *skb;
	int err;

	/* FW messages have no space for more than 32 bits of flags */
	if (flags >> 32)
		return -EOPNOTSUPP;

	skb = nfp_bpf_cmsg_map_req_alloc(bpf, 1);
	if (!skb)
		return -ENOMEM;

	req = (void *)skb->data;
	req->tid = cpu_to_be32(nfp_map->tid);
	req->count = cpu_to_be32(1);
	req->flags = cpu_to_be32(flags);

	/* Copy inputs */
	if (key)
		memcpy(&req->elem[0].key, key, map->key_size);
	if (value)
		memcpy(&req->elem[0].value, value, map->value_size);

	skb = nfp_bpf_cmsg_communicate(bpf, skb, op,
				       sizeof(*reply) + sizeof(*reply->elem));
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	reply = (void *)skb->data;
	err = nfp_bpf_ctrl_rc_to_errno(bpf, &reply->reply_hdr);
	if (err)
		goto err_free;

	/* Copy outputs */
	if (out_key)
		memcpy(out_key, &reply->elem[0].key, map->key_size);
	if (out_value)
		memcpy(out_value, &reply->elem[0].value, map->value_size);

	dev_consume_skb_any(skb);

	return 0;
err_free:
	dev_kfree_skb_any(skb);
	return err;
}

int nfp_bpf_ctrl_update_entry(struct bpf_offloaded_map *offmap,
			      void *key, void *value, u64 flags)
{
	return nfp_bpf_ctrl_entry_op(offmap, CMSG_TYPE_MAP_UPDATE,
				     key, value, flags, NULL, NULL);
}

int nfp_bpf_ctrl_del_entry(struct bpf_offloaded_map *offmap, void *key)
{
	return nfp_bpf_ctrl_entry_op(offmap, CMSG_TYPE_MAP_DELETE,
				     key, NULL, 0, NULL, NULL);
}

int nfp_bpf_ctrl_lookup_entry(struct bpf_offloaded_map *offmap,
			      void *key, void *value)
{
	return nfp_bpf_ctrl_entry_op(offmap, CMSG_TYPE_MAP_LOOKUP,
				     key, NULL, 0, NULL, value);
}

int nfp_bpf_ctrl_getfirst_entry(struct bpf_offloaded_map *offmap,
				void *next_key)
{
	return nfp_bpf_ctrl_entry_op(offmap, CMSG_TYPE_MAP_GETFIRST,
				     NULL, NULL, 0, next_key, NULL);
}

int nfp_bpf_ctrl_getnext_entry(struct bpf_offloaded_map *offmap,
			       void *key, void *next_key)
{
	return nfp_bpf_ctrl_entry_op(offmap, CMSG_TYPE_MAP_GETNEXT,
				     key, NULL, 0, next_key, NULL);
}

void nfp_bpf_ctrl_msg_rx(struct nfp_app *app, struct sk_buff *skb)
{
	struct nfp_app_bpf *bpf = app->priv;
	unsigned int tag;

	if (unlikely(skb->len < sizeof(struct cmsg_reply_map_simple))) {
		cmsg_warn(bpf, "cmsg drop - too short %d!\n", skb->len);
		goto err_free;
	}

	nfp_ctrl_lock(bpf->app->ctrl);

	tag = nfp_bpf_cmsg_get_tag(skb);
	if (unlikely(!test_bit(tag, bpf->tag_allocator))) {
		cmsg_warn(bpf, "cmsg drop - no one is waiting for tag %u!\n",
			  tag);
		goto err_unlock;
	}

	__skb_queue_tail(&bpf->cmsg_replies, skb);
	wake_up_interruptible_all(&bpf->cmsg_wq);

	nfp_ctrl_unlock(bpf->app->ctrl);

	return;
err_unlock:
	nfp_ctrl_unlock(bpf->app->ctrl);
err_free:
	dev_kfree_skb_any(skb);
}
