#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/netfilter.h>
#include <linux/rhashtable.h>
#include <linux/netdevice.h>
#include <net/netfilter/nf_flow_table.h>
#include <net/netfilter/nf_conntrack.h>
#include <net/netfilter/nf_conntrack_core.h>
#include <net/netfilter/nf_conntrack_tuple.h>

struct flow_offload_entry {
	struct flow_offload	flow;
	struct nf_conn		*ct;
	struct rcu_head		rcu_head;
};

struct flow_offload *
flow_offload_alloc(struct nf_conn *ct, struct nf_flow_route *route)
{
	struct flow_offload_entry *entry;
	struct flow_offload *flow;

	if (unlikely(nf_ct_is_dying(ct) ||
	    !atomic_inc_not_zero(&ct->ct_general.use)))
		return NULL;

	entry = kzalloc(sizeof(*entry), GFP_ATOMIC);
	if (!entry)
		goto err_ct_refcnt;

	flow = &entry->flow;

	if (!dst_hold_safe(route->tuple[FLOW_OFFLOAD_DIR_ORIGINAL].dst))
		goto err_dst_cache_original;

	if (!dst_hold_safe(route->tuple[FLOW_OFFLOAD_DIR_REPLY].dst))
		goto err_dst_cache_reply;

	entry->ct = ct;

	switch (ct->tuplehash[IP_CT_DIR_ORIGINAL].tuple.src.l3num) {
	case NFPROTO_IPV4:
		flow->tuplehash[FLOW_OFFLOAD_DIR_ORIGINAL].tuple.src_v4 =
			ct->tuplehash[IP_CT_DIR_ORIGINAL].tuple.src.u3.in;
		flow->tuplehash[FLOW_OFFLOAD_DIR_ORIGINAL].tuple.dst_v4 =
			ct->tuplehash[IP_CT_DIR_ORIGINAL].tuple.dst.u3.in;
		flow->tuplehash[FLOW_OFFLOAD_DIR_REPLY].tuple.src_v4 =
			ct->tuplehash[IP_CT_DIR_REPLY].tuple.src.u3.in;
		flow->tuplehash[FLOW_OFFLOAD_DIR_REPLY].tuple.dst_v4 =
			ct->tuplehash[IP_CT_DIR_REPLY].tuple.dst.u3.in;
		break;
	case NFPROTO_IPV6:
		flow->tuplehash[FLOW_OFFLOAD_DIR_ORIGINAL].tuple.src_v6 =
			ct->tuplehash[IP_CT_DIR_ORIGINAL].tuple.src.u3.in6;
		flow->tuplehash[FLOW_OFFLOAD_DIR_ORIGINAL].tuple.dst_v6 =
			ct->tuplehash[IP_CT_DIR_ORIGINAL].tuple.dst.u3.in6;
		flow->tuplehash[FLOW_OFFLOAD_DIR_REPLY].tuple.src_v6 =
			ct->tuplehash[IP_CT_DIR_REPLY].tuple.src.u3.in6;
		flow->tuplehash[FLOW_OFFLOAD_DIR_REPLY].tuple.dst_v6 =
			ct->tuplehash[IP_CT_DIR_REPLY].tuple.dst.u3.in6;
		break;
	}

	flow->tuplehash[FLOW_OFFLOAD_DIR_ORIGINAL].tuple.l3proto =
		ct->tuplehash[IP_CT_DIR_ORIGINAL].tuple.src.l3num;
	flow->tuplehash[FLOW_OFFLOAD_DIR_ORIGINAL].tuple.l4proto =
		ct->tuplehash[IP_CT_DIR_ORIGINAL].tuple.dst.protonum;
	flow->tuplehash[FLOW_OFFLOAD_DIR_REPLY].tuple.l3proto =
		ct->tuplehash[IP_CT_DIR_ORIGINAL].tuple.src.l3num;
	flow->tuplehash[FLOW_OFFLOAD_DIR_REPLY].tuple.l4proto =
		ct->tuplehash[IP_CT_DIR_ORIGINAL].tuple.dst.protonum;

	flow->tuplehash[FLOW_OFFLOAD_DIR_ORIGINAL].tuple.dst_cache =
		  route->tuple[FLOW_OFFLOAD_DIR_ORIGINAL].dst;
	flow->tuplehash[FLOW_OFFLOAD_DIR_REPLY].tuple.dst_cache =
		  route->tuple[FLOW_OFFLOAD_DIR_REPLY].dst;

	flow->tuplehash[FLOW_OFFLOAD_DIR_ORIGINAL].tuple.src_port =
		ct->tuplehash[IP_CT_DIR_ORIGINAL].tuple.src.u.tcp.port;
	flow->tuplehash[FLOW_OFFLOAD_DIR_ORIGINAL].tuple.dst_port =
		ct->tuplehash[IP_CT_DIR_ORIGINAL].tuple.dst.u.tcp.port;
	flow->tuplehash[FLOW_OFFLOAD_DIR_REPLY].tuple.src_port =
		ct->tuplehash[IP_CT_DIR_REPLY].tuple.src.u.tcp.port;
	flow->tuplehash[FLOW_OFFLOAD_DIR_REPLY].tuple.dst_port =
		ct->tuplehash[IP_CT_DIR_REPLY].tuple.dst.u.tcp.port;

	flow->tuplehash[FLOW_OFFLOAD_DIR_ORIGINAL].tuple.dir =
						FLOW_OFFLOAD_DIR_ORIGINAL;
	flow->tuplehash[FLOW_OFFLOAD_DIR_REPLY].tuple.dir =
						FLOW_OFFLOAD_DIR_REPLY;

	flow->tuplehash[FLOW_OFFLOAD_DIR_ORIGINAL].tuple.iifidx =
		route->tuple[FLOW_OFFLOAD_DIR_ORIGINAL].ifindex;
	flow->tuplehash[FLOW_OFFLOAD_DIR_ORIGINAL].tuple.oifidx =
		route->tuple[FLOW_OFFLOAD_DIR_REPLY].ifindex;
	flow->tuplehash[FLOW_OFFLOAD_DIR_REPLY].tuple.iifidx =
		route->tuple[FLOW_OFFLOAD_DIR_REPLY].ifindex;
	flow->tuplehash[FLOW_OFFLOAD_DIR_REPLY].tuple.oifidx =
		route->tuple[FLOW_OFFLOAD_DIR_ORIGINAL].ifindex;

	if (ct->status & IPS_SRC_NAT)
		flow->flags |= FLOW_OFFLOAD_SNAT;
	else if (ct->status & IPS_DST_NAT)
		flow->flags |= FLOW_OFFLOAD_DNAT;

	return flow;

err_dst_cache_reply:
	dst_release(route->tuple[FLOW_OFFLOAD_DIR_ORIGINAL].dst);
err_dst_cache_original:
	kfree(entry);
err_ct_refcnt:
	nf_ct_put(ct);

	return NULL;
}
EXPORT_SYMBOL_GPL(flow_offload_alloc);

void flow_offload_free(struct flow_offload *flow)
{
	struct flow_offload_entry *e;

	dst_release(flow->tuplehash[FLOW_OFFLOAD_DIR_ORIGINAL].tuple.dst_cache);
	dst_release(flow->tuplehash[FLOW_OFFLOAD_DIR_REPLY].tuple.dst_cache);
	e = container_of(flow, struct flow_offload_entry, flow);
	kfree(e);
}
EXPORT_SYMBOL_GPL(flow_offload_free);

void flow_offload_dead(struct flow_offload *flow)
{
	flow->flags |= FLOW_OFFLOAD_DYING;
}
EXPORT_SYMBOL_GPL(flow_offload_dead);

int flow_offload_add(struct nf_flowtable *flow_table, struct flow_offload *flow)
{
	flow->timeout = (u32)jiffies;

	rhashtable_insert_fast(&flow_table->rhashtable,
			       &flow->tuplehash[FLOW_OFFLOAD_DIR_ORIGINAL].node,
			       *flow_table->type->params);
	rhashtable_insert_fast(&flow_table->rhashtable,
			       &flow->tuplehash[FLOW_OFFLOAD_DIR_REPLY].node,
			       *flow_table->type->params);
	return 0;
}
EXPORT_SYMBOL_GPL(flow_offload_add);

void flow_offload_del(struct nf_flowtable *flow_table,
		      struct flow_offload *flow)
{
	struct flow_offload_entry *e;

	rhashtable_remove_fast(&flow_table->rhashtable,
			       &flow->tuplehash[FLOW_OFFLOAD_DIR_ORIGINAL].node,
			       *flow_table->type->params);
	rhashtable_remove_fast(&flow_table->rhashtable,
			       &flow->tuplehash[FLOW_OFFLOAD_DIR_REPLY].node,
			       *flow_table->type->params);

	e = container_of(flow, struct flow_offload_entry, flow);
	kfree_rcu(e, rcu_head);
}
EXPORT_SYMBOL_GPL(flow_offload_del);

struct flow_offload_tuple_rhash *
flow_offload_lookup(struct nf_flowtable *flow_table,
		    struct flow_offload_tuple *tuple)
{
	return rhashtable_lookup_fast(&flow_table->rhashtable, tuple,
				      *flow_table->type->params);
}
EXPORT_SYMBOL_GPL(flow_offload_lookup);

static void nf_flow_release_ct(const struct flow_offload *flow)
{
	struct flow_offload_entry *e;

	e = container_of(flow, struct flow_offload_entry, flow);
	nf_ct_delete(e->ct, 0, 0);
	nf_ct_put(e->ct);
}

int nf_flow_table_iterate(struct nf_flowtable *flow_table,
			  void (*iter)(struct flow_offload *flow, void *data),
			  void *data)
{
	struct flow_offload_tuple_rhash *tuplehash;
	struct rhashtable_iter hti;
	struct flow_offload *flow;
	int err;

	err = rhashtable_walk_init(&flow_table->rhashtable, &hti, GFP_KERNEL);
	if (err)
		return err;

	rhashtable_walk_start(&hti);

	while ((tuplehash = rhashtable_walk_next(&hti))) {
		if (IS_ERR(tuplehash)) {
			err = PTR_ERR(tuplehash);
			if (err != -EAGAIN)
				goto out;

			continue;
		}
		if (tuplehash->tuple.dir)
			continue;

		flow = container_of(tuplehash, struct flow_offload, tuplehash[0]);

		iter(flow, data);
	}
out:
	rhashtable_walk_stop(&hti);
	rhashtable_walk_exit(&hti);

	return err;
}
EXPORT_SYMBOL_GPL(nf_flow_table_iterate);

static inline bool nf_flow_has_expired(const struct flow_offload *flow)
{
	return (__s32)(flow->timeout - (u32)jiffies) <= 0;
}

static inline bool nf_flow_is_dying(const struct flow_offload *flow)
{
	return flow->flags & FLOW_OFFLOAD_DYING;
}

void nf_flow_offload_work_gc(struct work_struct *work)
{
	struct flow_offload_tuple_rhash *tuplehash;
	struct nf_flowtable *flow_table;
	struct rhashtable_iter hti;
	struct flow_offload *flow;
	int err;

	flow_table = container_of(work, struct nf_flowtable, gc_work.work);

	err = rhashtable_walk_init(&flow_table->rhashtable, &hti, GFP_KERNEL);
	if (err)
		goto schedule;

	rhashtable_walk_start(&hti);

	while ((tuplehash = rhashtable_walk_next(&hti))) {
		if (IS_ERR(tuplehash)) {
			err = PTR_ERR(tuplehash);
			if (err != -EAGAIN)
				goto out;

			continue;
		}
		if (tuplehash->tuple.dir)
			continue;

		flow = container_of(tuplehash, struct flow_offload, tuplehash[0]);

		if (nf_flow_has_expired(flow) ||
		    nf_flow_is_dying(flow)) {
			flow_offload_del(flow_table, flow);
			nf_flow_release_ct(flow);
		}
	}
out:
	rhashtable_walk_stop(&hti);
	rhashtable_walk_exit(&hti);
schedule:
	queue_delayed_work(system_power_efficient_wq, &flow_table->gc_work, HZ);
}
EXPORT_SYMBOL_GPL(nf_flow_offload_work_gc);

static u32 flow_offload_hash(const void *data, u32 len, u32 seed)
{
	const struct flow_offload_tuple *tuple = data;

	return jhash(tuple, offsetof(struct flow_offload_tuple, dir), seed);
}

static u32 flow_offload_hash_obj(const void *data, u32 len, u32 seed)
{
	const struct flow_offload_tuple_rhash *tuplehash = data;

	return jhash(&tuplehash->tuple, offsetof(struct flow_offload_tuple, dir), seed);
}

static int flow_offload_hash_cmp(struct rhashtable_compare_arg *arg,
					const void *ptr)
{
	const struct flow_offload_tuple *tuple = arg->key;
	const struct flow_offload_tuple_rhash *x = ptr;

	if (memcmp(&x->tuple, tuple, offsetof(struct flow_offload_tuple, dir)))
		return 1;

	return 0;
}

const struct rhashtable_params nf_flow_offload_rhash_params = {
	.head_offset		= offsetof(struct flow_offload_tuple_rhash, node),
	.hashfn			= flow_offload_hash,
	.obj_hashfn		= flow_offload_hash_obj,
	.obj_cmpfn		= flow_offload_hash_cmp,
	.automatic_shrinking	= true,
};
EXPORT_SYMBOL_GPL(nf_flow_offload_rhash_params);

static int nf_flow_nat_port_tcp(struct sk_buff *skb, unsigned int thoff,
				__be16 port, __be16 new_port)
{
	struct tcphdr *tcph;

	if (!pskb_may_pull(skb, thoff + sizeof(*tcph)) ||
	    skb_try_make_writable(skb, thoff + sizeof(*tcph)))
		return -1;

	tcph = (void *)(skb_network_header(skb) + thoff);
	inet_proto_csum_replace2(&tcph->check, skb, port, new_port, true);

	return 0;
}

static int nf_flow_nat_port_udp(struct sk_buff *skb, unsigned int thoff,
				__be16 port, __be16 new_port)
{
	struct udphdr *udph;

	if (!pskb_may_pull(skb, thoff + sizeof(*udph)) ||
	    skb_try_make_writable(skb, thoff + sizeof(*udph)))
		return -1;

	udph = (void *)(skb_network_header(skb) + thoff);
	if (udph->check || skb->ip_summed == CHECKSUM_PARTIAL) {
		inet_proto_csum_replace2(&udph->check, skb, port,
					 new_port, true);
		if (!udph->check)
			udph->check = CSUM_MANGLED_0;
	}

	return 0;
}

static int nf_flow_nat_port(struct sk_buff *skb, unsigned int thoff,
			    u8 protocol, __be16 port, __be16 new_port)
{
	switch (protocol) {
	case IPPROTO_TCP:
		if (nf_flow_nat_port_tcp(skb, thoff, port, new_port) < 0)
			return NF_DROP;
		break;
	case IPPROTO_UDP:
		if (nf_flow_nat_port_udp(skb, thoff, port, new_port) < 0)
			return NF_DROP;
		break;
	}

	return 0;
}

int nf_flow_snat_port(const struct flow_offload *flow,
		      struct sk_buff *skb, unsigned int thoff,
		      u8 protocol, enum flow_offload_tuple_dir dir)
{
	struct flow_ports *hdr;
	__be16 port, new_port;

	if (!pskb_may_pull(skb, thoff + sizeof(*hdr)) ||
	    skb_try_make_writable(skb, thoff + sizeof(*hdr)))
		return -1;

	hdr = (void *)(skb_network_header(skb) + thoff);

	switch (dir) {
	case FLOW_OFFLOAD_DIR_ORIGINAL:
		port = hdr->source;
		new_port = flow->tuplehash[FLOW_OFFLOAD_DIR_REPLY].tuple.dst_port;
		hdr->source = new_port;
		break;
	case FLOW_OFFLOAD_DIR_REPLY:
		port = hdr->dest;
		new_port = flow->tuplehash[FLOW_OFFLOAD_DIR_ORIGINAL].tuple.src_port;
		hdr->dest = new_port;
		break;
	default:
		return -1;
	}

	return nf_flow_nat_port(skb, thoff, protocol, port, new_port);
}
EXPORT_SYMBOL_GPL(nf_flow_snat_port);

int nf_flow_dnat_port(const struct flow_offload *flow,
		      struct sk_buff *skb, unsigned int thoff,
		      u8 protocol, enum flow_offload_tuple_dir dir)
{
	struct flow_ports *hdr;
	__be16 port, new_port;

	if (!pskb_may_pull(skb, thoff + sizeof(*hdr)) ||
	    skb_try_make_writable(skb, thoff + sizeof(*hdr)))
		return -1;

	hdr = (void *)(skb_network_header(skb) + thoff);

	switch (dir) {
	case FLOW_OFFLOAD_DIR_ORIGINAL:
		port = hdr->dest;
		new_port = flow->tuplehash[FLOW_OFFLOAD_DIR_REPLY].tuple.src_port;
		hdr->dest = new_port;
		break;
	case FLOW_OFFLOAD_DIR_REPLY:
		port = hdr->source;
		new_port = flow->tuplehash[FLOW_OFFLOAD_DIR_ORIGINAL].tuple.dst_port;
		hdr->source = new_port;
		break;
	default:
		return -1;
	}

	return nf_flow_nat_port(skb, thoff, protocol, port, new_port);
}
EXPORT_SYMBOL_GPL(nf_flow_dnat_port);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Pablo Neira Ayuso <pablo@netfilter.org>");
