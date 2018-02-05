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

#include <net/pkt_cls.h>

#include "../nfpcore/nfp_cpp.h"
#include "../nfpcore/nfp_nffw.h"
#include "../nfp_app.h"
#include "../nfp_main.h"
#include "../nfp_net.h"
#include "../nfp_port.h"
#include "fw.h"
#include "main.h"

static bool nfp_net_ebpf_capable(struct nfp_net *nn)
{
#ifdef __LITTLE_ENDIAN
	if (nn->cap & NFP_NET_CFG_CTRL_BPF &&
	    nn_readb(nn, NFP_NET_CFG_BPF_ABI) == NFP_NET_BPF_ABI)
		return true;
#endif
	return false;
}

static int
nfp_bpf_xdp_offload(struct nfp_app *app, struct nfp_net *nn,
		    struct bpf_prog *prog, struct netlink_ext_ack *extack)
{
	bool running, xdp_running;
	int ret;

	if (!nfp_net_ebpf_capable(nn))
		return -EINVAL;

	running = nn->dp.ctrl & NFP_NET_CFG_CTRL_BPF;
	xdp_running = running && nn->dp.bpf_offload_xdp;

	if (!prog && !xdp_running)
		return 0;
	if (prog && running && !xdp_running)
		return -EBUSY;

	ret = nfp_net_bpf_offload(nn, prog, running, extack);
	/* Stop offload if replace not possible */
	if (ret && prog)
		nfp_bpf_xdp_offload(app, nn, NULL, extack);

	nn->dp.bpf_offload_xdp = prog && !ret;
	return ret;
}

static const char *nfp_bpf_extra_cap(struct nfp_app *app, struct nfp_net *nn)
{
	return nfp_net_ebpf_capable(nn) ? "BPF" : "";
}

static int
nfp_bpf_vnic_alloc(struct nfp_app *app, struct nfp_net *nn, unsigned int id)
{
	struct nfp_bpf_vnic *bv;
	int err;

	bv = kzalloc(sizeof(*bv), GFP_KERNEL);
	if (!bv)
		return -ENOMEM;
	nn->app_priv = bv;

	err = nfp_app_nic_vnic_alloc(app, nn, id);
	if (err)
		goto err_free_priv;

	bv->start_off = nn_readw(nn, NFP_NET_CFG_BPF_START);
	bv->tgt_done = nn_readw(nn, NFP_NET_CFG_BPF_DONE);

	return 0;
err_free_priv:
	kfree(nn->app_priv);
	return err;
}

static void nfp_bpf_vnic_free(struct nfp_app *app, struct nfp_net *nn)
{
	struct nfp_bpf_vnic *bv = nn->app_priv;

	WARN_ON(bv->tc_prog);
	kfree(bv);
}

static int nfp_bpf_setup_tc_block_cb(enum tc_setup_type type,
				     void *type_data, void *cb_priv)
{
	struct tc_cls_bpf_offload *cls_bpf = type_data;
	struct nfp_net *nn = cb_priv;
	struct bpf_prog *oldprog;
	struct nfp_bpf_vnic *bv;
	int err;

	if (type != TC_SETUP_CLSBPF) {
		NL_SET_ERR_MSG_MOD(cls_bpf->common.extack,
				   "only offload of BPF classifiers supported");
		return -EOPNOTSUPP;
	}
	if (!tc_cls_can_offload_and_chain0(nn->dp.netdev, &cls_bpf->common))
		return -EOPNOTSUPP;
	if (!nfp_net_ebpf_capable(nn)) {
		NL_SET_ERR_MSG_MOD(cls_bpf->common.extack,
				   "NFP firmware does not support eBPF offload");
		return -EOPNOTSUPP;
	}
	if (cls_bpf->common.protocol != htons(ETH_P_ALL)) {
		NL_SET_ERR_MSG_MOD(cls_bpf->common.extack,
				   "only ETH_P_ALL supported as filter protocol");
		return -EOPNOTSUPP;
	}

	/* Only support TC direct action */
	if (!cls_bpf->exts_integrated ||
	    tcf_exts_has_actions(cls_bpf->exts)) {
		NL_SET_ERR_MSG_MOD(cls_bpf->common.extack,
				   "only direct action with no legacy actions supported");
		return -EOPNOTSUPP;
	}

	if (cls_bpf->command != TC_CLSBPF_OFFLOAD)
		return -EOPNOTSUPP;

	bv = nn->app_priv;
	oldprog = cls_bpf->oldprog;

	/* Don't remove if oldprog doesn't match driver's state */
	if (bv->tc_prog != oldprog) {
		oldprog = NULL;
		if (!cls_bpf->prog)
			return 0;
	}

	err = nfp_net_bpf_offload(nn, cls_bpf->prog, oldprog,
				  cls_bpf->common.extack);
	if (err)
		return err;

	bv->tc_prog = cls_bpf->prog;
	return 0;
}

static int nfp_bpf_setup_tc_block(struct net_device *netdev,
				  struct tc_block_offload *f)
{
	struct nfp_net *nn = netdev_priv(netdev);

	if (f->binder_type != TCF_BLOCK_BINDER_TYPE_CLSACT_INGRESS)
		return -EOPNOTSUPP;

	switch (f->command) {
	case TC_BLOCK_BIND:
		return tcf_block_cb_register(f->block,
					     nfp_bpf_setup_tc_block_cb,
					     nn, nn);
	case TC_BLOCK_UNBIND:
		tcf_block_cb_unregister(f->block,
					nfp_bpf_setup_tc_block_cb,
					nn);
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int nfp_bpf_setup_tc(struct nfp_app *app, struct net_device *netdev,
			    enum tc_setup_type type, void *type_data)
{
	switch (type) {
	case TC_SETUP_BLOCK:
		return nfp_bpf_setup_tc_block(netdev, type_data);
	default:
		return -EOPNOTSUPP;
	}
}

static bool nfp_bpf_tc_busy(struct nfp_app *app, struct nfp_net *nn)
{
	struct nfp_bpf_vnic *bv = nn->app_priv;

	return !!bv->tc_prog;
}

static int
nfp_bpf_change_mtu(struct nfp_app *app, struct net_device *netdev, int new_mtu)
{
	struct nfp_net *nn = netdev_priv(netdev);
	unsigned int max_mtu;

	if (~nn->dp.ctrl & NFP_NET_CFG_CTRL_BPF)
		return 0;

	max_mtu = nn_readb(nn, NFP_NET_CFG_BPF_INL_MTU) * 64 - 32;
	if (new_mtu > max_mtu) {
		nn_info(nn, "BPF offload active, MTU over %u not supported\n",
			max_mtu);
		return -EBUSY;
	}
	return 0;
}

static int
nfp_bpf_parse_cap_adjust_head(struct nfp_app_bpf *bpf, void __iomem *value,
			      u32 length)
{
	struct nfp_bpf_cap_tlv_adjust_head __iomem *cap = value;
	struct nfp_cpp *cpp = bpf->app->pf->cpp;

	if (length < sizeof(*cap)) {
		nfp_err(cpp, "truncated adjust_head TLV: %d\n", length);
		return -EINVAL;
	}

	bpf->adjust_head.flags = readl(&cap->flags);
	bpf->adjust_head.off_min = readl(&cap->off_min);
	bpf->adjust_head.off_max = readl(&cap->off_max);
	bpf->adjust_head.guaranteed_sub = readl(&cap->guaranteed_sub);
	bpf->adjust_head.guaranteed_add = readl(&cap->guaranteed_add);

	if (bpf->adjust_head.off_min > bpf->adjust_head.off_max) {
		nfp_err(cpp, "invalid adjust_head TLV: min > max\n");
		return -EINVAL;
	}
	if (!FIELD_FIT(UR_REG_IMM_MAX, bpf->adjust_head.off_min) ||
	    !FIELD_FIT(UR_REG_IMM_MAX, bpf->adjust_head.off_max)) {
		nfp_warn(cpp, "disabling adjust_head - driver expects min/max to fit in as immediates\n");
		memset(&bpf->adjust_head, 0, sizeof(bpf->adjust_head));
		return 0;
	}

	return 0;
}

static int
nfp_bpf_parse_cap_func(struct nfp_app_bpf *bpf, void __iomem *value, u32 length)
{
	struct nfp_bpf_cap_tlv_func __iomem *cap = value;

	if (length < sizeof(*cap)) {
		nfp_err(bpf->app->cpp, "truncated function TLV: %d\n", length);
		return -EINVAL;
	}

	switch (readl(&cap->func_id)) {
	case BPF_FUNC_map_lookup_elem:
		bpf->helpers.map_lookup = readl(&cap->func_addr);
		break;
	}

	return 0;
}

static int
nfp_bpf_parse_cap_maps(struct nfp_app_bpf *bpf, void __iomem *value, u32 length)
{
	struct nfp_bpf_cap_tlv_maps __iomem *cap = value;

	if (length < sizeof(*cap)) {
		nfp_err(bpf->app->cpp, "truncated maps TLV: %d\n", length);
		return -EINVAL;
	}

	bpf->maps.types = readl(&cap->types);
	bpf->maps.max_maps = readl(&cap->max_maps);
	bpf->maps.max_elems = readl(&cap->max_elems);
	bpf->maps.max_key_sz = readl(&cap->max_key_sz);
	bpf->maps.max_val_sz = readl(&cap->max_val_sz);
	bpf->maps.max_elem_sz = readl(&cap->max_elem_sz);

	return 0;
}

static int nfp_bpf_parse_capabilities(struct nfp_app *app)
{
	struct nfp_cpp *cpp = app->pf->cpp;
	struct nfp_cpp_area *area;
	u8 __iomem *mem, *start;

	mem = nfp_rtsym_map(app->pf->rtbl, "_abi_bpf_capabilities", "bpf.cap",
			    8, &area);
	if (IS_ERR(mem))
		return PTR_ERR(mem) == -ENOENT ? 0 : PTR_ERR(mem);

	start = mem;
	while (mem - start + 8 < nfp_cpp_area_size(area)) {
		u8 __iomem *value;
		u32 type, length;

		type = readl(mem);
		length = readl(mem + 4);
		value = mem + 8;

		mem += 8 + length;
		if (mem - start > nfp_cpp_area_size(area))
			goto err_release_free;

		switch (type) {
		case NFP_BPF_CAP_TYPE_FUNC:
			if (nfp_bpf_parse_cap_func(app->priv, value, length))
				goto err_release_free;
			break;
		case NFP_BPF_CAP_TYPE_ADJUST_HEAD:
			if (nfp_bpf_parse_cap_adjust_head(app->priv, value,
							  length))
				goto err_release_free;
			break;
		case NFP_BPF_CAP_TYPE_MAPS:
			if (nfp_bpf_parse_cap_maps(app->priv, value, length))
				goto err_release_free;
			break;
		default:
			nfp_dbg(cpp, "unknown BPF capability: %d\n", type);
			break;
		}
	}
	if (mem - start != nfp_cpp_area_size(area)) {
		nfp_err(cpp, "BPF capabilities left after parsing, parsed:%zd total length:%zu\n",
			mem - start, nfp_cpp_area_size(area));
		goto err_release_free;
	}

	nfp_cpp_area_release_free(area);

	return 0;

err_release_free:
	nfp_err(cpp, "invalid BPF capabilities at offset:%zd\n", mem - start);
	nfp_cpp_area_release_free(area);
	return -EINVAL;
}

static int nfp_bpf_init(struct nfp_app *app)
{
	struct nfp_app_bpf *bpf;
	int err;

	bpf = kzalloc(sizeof(*bpf), GFP_KERNEL);
	if (!bpf)
		return -ENOMEM;
	bpf->app = app;
	app->priv = bpf;

	skb_queue_head_init(&bpf->cmsg_replies);
	init_waitqueue_head(&bpf->cmsg_wq);
	INIT_LIST_HEAD(&bpf->map_list);

	err = nfp_bpf_parse_capabilities(app);
	if (err)
		goto err_free_bpf;

	return 0;

err_free_bpf:
	kfree(bpf);
	return err;
}

static void nfp_bpf_clean(struct nfp_app *app)
{
	struct nfp_app_bpf *bpf = app->priv;

	WARN_ON(!skb_queue_empty(&bpf->cmsg_replies));
	WARN_ON(!list_empty(&bpf->map_list));
	WARN_ON(bpf->maps_in_use || bpf->map_elems_in_use);
	kfree(bpf);
}

const struct nfp_app_type app_bpf = {
	.id		= NFP_APP_BPF_NIC,
	.name		= "ebpf",

	.ctrl_cap_mask	= 0,

	.init		= nfp_bpf_init,
	.clean		= nfp_bpf_clean,

	.change_mtu	= nfp_bpf_change_mtu,

	.extra_cap	= nfp_bpf_extra_cap,

	.vnic_alloc	= nfp_bpf_vnic_alloc,
	.vnic_free	= nfp_bpf_vnic_free,

	.ctrl_msg_rx	= nfp_bpf_ctrl_msg_rx,

	.setup_tc	= nfp_bpf_setup_tc,
	.tc_busy	= nfp_bpf_tc_busy,
	.bpf		= nfp_ndo_bpf,
	.xdp_offload	= nfp_bpf_xdp_offload,
};
