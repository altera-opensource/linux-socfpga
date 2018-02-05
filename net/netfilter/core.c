/* netfilter.c: look after the filters for various protocols.
 * Heavily influenced by the old firewall.c by David Bonn and Alan Cox.
 *
 * Thanks to Rob `CmdrTaco' Malda for not influencing this code in any
 * way.
 *
 * This code is GPL.
 */
#include <linux/kernel.h>
#include <linux/netfilter.h>
#include <net/protocol.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/wait.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/if.h>
#include <linux/netdevice.h>
#include <linux/netfilter_ipv6.h>
#include <linux/inetdevice.h>
#include <linux/proc_fs.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/rcupdate.h>
#include <net/net_namespace.h>
#include <net/sock.h>

#include "nf_internals.h"

const struct nf_ipv6_ops __rcu *nf_ipv6_ops __read_mostly;
EXPORT_SYMBOL_GPL(nf_ipv6_ops);

DEFINE_PER_CPU(bool, nf_skb_duplicated);
EXPORT_SYMBOL_GPL(nf_skb_duplicated);

#ifdef HAVE_JUMP_LABEL
struct static_key nf_hooks_needed[NFPROTO_NUMPROTO][NF_MAX_HOOKS];
EXPORT_SYMBOL(nf_hooks_needed);
#endif

static DEFINE_MUTEX(nf_hook_mutex);

/* max hooks per family/hooknum */
#define MAX_HOOK_COUNT		1024

#define nf_entry_dereference(e) \
	rcu_dereference_protected(e, lockdep_is_held(&nf_hook_mutex))

static struct nf_hook_entries *allocate_hook_entries_size(u16 num)
{
	struct nf_hook_entries *e;
	size_t alloc = sizeof(*e) +
		       sizeof(struct nf_hook_entry) * num +
		       sizeof(struct nf_hook_ops *) * num +
		       sizeof(struct nf_hook_entries_rcu_head);

	if (num == 0)
		return NULL;

	e = kvzalloc(alloc, GFP_KERNEL);
	if (e)
		e->num_hook_entries = num;
	return e;
}

static void __nf_hook_entries_free(struct rcu_head *h)
{
	struct nf_hook_entries_rcu_head *head;

	head = container_of(h, struct nf_hook_entries_rcu_head, head);
	kvfree(head->allocation);
}

static void nf_hook_entries_free(struct nf_hook_entries *e)
{
	struct nf_hook_entries_rcu_head *head;
	struct nf_hook_ops **ops;
	unsigned int num;

	if (!e)
		return;

	num = e->num_hook_entries;
	ops = nf_hook_entries_get_hook_ops(e);
	head = (void *)&ops[num];
	head->allocation = e;
	call_rcu(&head->head, __nf_hook_entries_free);
}

static unsigned int accept_all(void *priv,
			       struct sk_buff *skb,
			       const struct nf_hook_state *state)
{
	return NF_ACCEPT; /* ACCEPT makes nf_hook_slow call next hook */
}

static const struct nf_hook_ops dummy_ops = {
	.hook = accept_all,
	.priority = INT_MIN,
};

static struct nf_hook_entries *
nf_hook_entries_grow(const struct nf_hook_entries *old,
		     const struct nf_hook_ops *reg)
{
	unsigned int i, alloc_entries, nhooks, old_entries;
	struct nf_hook_ops **orig_ops = NULL;
	struct nf_hook_ops **new_ops;
	struct nf_hook_entries *new;
	bool inserted = false;

	alloc_entries = 1;
	old_entries = old ? old->num_hook_entries : 0;

	if (old) {
		orig_ops = nf_hook_entries_get_hook_ops(old);

		for (i = 0; i < old_entries; i++) {
			if (orig_ops[i] != &dummy_ops)
				alloc_entries++;
		}
	}

	if (alloc_entries > MAX_HOOK_COUNT)
		return ERR_PTR(-E2BIG);

	new = allocate_hook_entries_size(alloc_entries);
	if (!new)
		return ERR_PTR(-ENOMEM);

	new_ops = nf_hook_entries_get_hook_ops(new);

	i = 0;
	nhooks = 0;
	while (i < old_entries) {
		if (orig_ops[i] == &dummy_ops) {
			++i;
			continue;
		}

		if (reg->nat_hook && orig_ops[i]->nat_hook) {
			kvfree(new);
			return ERR_PTR(-EBUSY);
		}

		if (inserted || reg->priority > orig_ops[i]->priority) {
			new_ops[nhooks] = (void *)orig_ops[i];
			new->hooks[nhooks] = old->hooks[i];
			i++;
		} else {
			new_ops[nhooks] = (void *)reg;
			new->hooks[nhooks].hook = reg->hook;
			new->hooks[nhooks].priv = reg->priv;
			inserted = true;
		}
		nhooks++;
	}

	if (!inserted) {
		new_ops[nhooks] = (void *)reg;
		new->hooks[nhooks].hook = reg->hook;
		new->hooks[nhooks].priv = reg->priv;
	}

	return new;
}

static void hooks_validate(const struct nf_hook_entries *hooks)
{
#ifdef CONFIG_DEBUG_KERNEL
	struct nf_hook_ops **orig_ops;
	int prio = INT_MIN;
	size_t i = 0;

	orig_ops = nf_hook_entries_get_hook_ops(hooks);

	for (i = 0; i < hooks->num_hook_entries; i++) {
		if (orig_ops[i] == &dummy_ops)
			continue;

		WARN_ON(orig_ops[i]->priority < prio);

		if (orig_ops[i]->priority > prio)
			prio = orig_ops[i]->priority;
	}
#endif
}

/*
 * __nf_hook_entries_try_shrink - try to shrink hook array
 *
 * @pp -- location of hook blob
 *
 * Hook unregistration must always succeed, so to-be-removed hooks
 * are replaced by a dummy one that will just move to next hook.
 *
 * This counts the current dummy hooks, attempts to allocate new blob,
 * copies the live hooks, then replaces and discards old one.
 *
 * return values:
 *
 * Returns address to free, or NULL.
 */
static void *__nf_hook_entries_try_shrink(struct nf_hook_entries __rcu **pp)
{
	struct nf_hook_entries *old, *new = NULL;
	unsigned int i, j, skip = 0, hook_entries;
	struct nf_hook_ops **orig_ops;
	struct nf_hook_ops **new_ops;

	old = nf_entry_dereference(*pp);
	if (WARN_ON_ONCE(!old))
		return NULL;

	orig_ops = nf_hook_entries_get_hook_ops(old);
	for (i = 0; i < old->num_hook_entries; i++) {
		if (orig_ops[i] == &dummy_ops)
			skip++;
	}

	/* if skip == hook_entries all hooks have been removed */
	hook_entries = old->num_hook_entries;
	if (skip == hook_entries)
		goto out_assign;

	if (skip == 0)
		return NULL;

	hook_entries -= skip;
	new = allocate_hook_entries_size(hook_entries);
	if (!new)
		return NULL;

	new_ops = nf_hook_entries_get_hook_ops(new);
	for (i = 0, j = 0; i < old->num_hook_entries; i++) {
		if (orig_ops[i] == &dummy_ops)
			continue;
		new->hooks[j] = old->hooks[i];
		new_ops[j] = (void *)orig_ops[i];
		j++;
	}
	hooks_validate(new);
out_assign:
	rcu_assign_pointer(*pp, new);
	return old;
}

static struct nf_hook_entries __rcu **
nf_hook_entry_head(struct net *net, int pf, unsigned int hooknum,
		   struct net_device *dev)
{
	switch (pf) {
	case NFPROTO_NETDEV:
		break;
#ifdef CONFIG_NETFILTER_FAMILY_ARP
	case NFPROTO_ARP:
		if (WARN_ON_ONCE(ARRAY_SIZE(net->nf.hooks_arp) <= hooknum))
			return NULL;
		return net->nf.hooks_arp + hooknum;
#endif
#ifdef CONFIG_NETFILTER_FAMILY_BRIDGE
	case NFPROTO_BRIDGE:
		if (WARN_ON_ONCE(ARRAY_SIZE(net->nf.hooks_bridge) <= hooknum))
			return NULL;
		return net->nf.hooks_bridge + hooknum;
#endif
	case NFPROTO_IPV4:
		if (WARN_ON_ONCE(ARRAY_SIZE(net->nf.hooks_ipv4) <= hooknum))
			return NULL;
		return net->nf.hooks_ipv4 + hooknum;
	case NFPROTO_IPV6:
		if (WARN_ON_ONCE(ARRAY_SIZE(net->nf.hooks_ipv6) <= hooknum))
			return NULL;
		return net->nf.hooks_ipv6 + hooknum;
#if IS_ENABLED(CONFIG_DECNET)
	case NFPROTO_DECNET:
		if (WARN_ON_ONCE(ARRAY_SIZE(net->nf.hooks_decnet) <= hooknum))
			return NULL;
		return net->nf.hooks_decnet + hooknum;
#endif
	default:
		WARN_ON_ONCE(1);
		return NULL;
	}

#ifdef CONFIG_NETFILTER_INGRESS
	if (hooknum == NF_NETDEV_INGRESS) {
		if (dev && dev_net(dev) == net)
			return &dev->nf_hooks_ingress;
	}
#endif
	WARN_ON_ONCE(1);
	return NULL;
}

static int __nf_register_net_hook(struct net *net, int pf,
				  const struct nf_hook_ops *reg)
{
	struct nf_hook_entries *p, *new_hooks;
	struct nf_hook_entries __rcu **pp;

	if (pf == NFPROTO_NETDEV) {
#ifndef CONFIG_NETFILTER_INGRESS
		if (reg->hooknum == NF_NETDEV_INGRESS)
			return -EOPNOTSUPP;
#endif
		if (reg->hooknum != NF_NETDEV_INGRESS ||
		    !reg->dev || dev_net(reg->dev) != net)
			return -EINVAL;
	}

	pp = nf_hook_entry_head(net, pf, reg->hooknum, reg->dev);
	if (!pp)
		return -EINVAL;

	mutex_lock(&nf_hook_mutex);

	p = nf_entry_dereference(*pp);
	new_hooks = nf_hook_entries_grow(p, reg);

	if (!IS_ERR(new_hooks))
		rcu_assign_pointer(*pp, new_hooks);

	mutex_unlock(&nf_hook_mutex);
	if (IS_ERR(new_hooks))
		return PTR_ERR(new_hooks);

	hooks_validate(new_hooks);
#ifdef CONFIG_NETFILTER_INGRESS
	if (pf == NFPROTO_NETDEV && reg->hooknum == NF_NETDEV_INGRESS)
		net_inc_ingress_queue();
#endif
#ifdef HAVE_JUMP_LABEL
	static_key_slow_inc(&nf_hooks_needed[pf][reg->hooknum]);
#endif
	BUG_ON(p == new_hooks);
	nf_hook_entries_free(p);
	return 0;
}

/*
 * nf_remove_net_hook - remove a hook from blob
 *
 * @oldp: current address of hook blob
 * @unreg: hook to unregister
 *
 * This cannot fail, hook unregistration must always succeed.
 * Therefore replace the to-be-removed hook with a dummy hook.
 */
static void nf_remove_net_hook(struct nf_hook_entries *old,
			       const struct nf_hook_ops *unreg, int pf)
{
	struct nf_hook_ops **orig_ops;
	bool found = false;
	unsigned int i;

	orig_ops = nf_hook_entries_get_hook_ops(old);
	for (i = 0; i < old->num_hook_entries; i++) {
		if (orig_ops[i] != unreg)
			continue;
		WRITE_ONCE(old->hooks[i].hook, accept_all);
		WRITE_ONCE(orig_ops[i], &dummy_ops);
		found = true;
		break;
	}

	if (found) {
#ifdef CONFIG_NETFILTER_INGRESS
		if (pf == NFPROTO_NETDEV && unreg->hooknum == NF_NETDEV_INGRESS)
			net_dec_ingress_queue();
#endif
#ifdef HAVE_JUMP_LABEL
		static_key_slow_dec(&nf_hooks_needed[pf][unreg->hooknum]);
#endif
	} else {
		WARN_ONCE(1, "hook not found, pf %d num %d", pf, unreg->hooknum);
	}
}

static void __nf_unregister_net_hook(struct net *net, int pf,
				     const struct nf_hook_ops *reg)
{
	struct nf_hook_entries __rcu **pp;
	struct nf_hook_entries *p;

	pp = nf_hook_entry_head(net, pf, reg->hooknum, reg->dev);
	if (!pp)
		return;

	mutex_lock(&nf_hook_mutex);

	p = nf_entry_dereference(*pp);
	if (WARN_ON_ONCE(!p)) {
		mutex_unlock(&nf_hook_mutex);
		return;
	}

	nf_remove_net_hook(p, reg, pf);

	p = __nf_hook_entries_try_shrink(pp);
	mutex_unlock(&nf_hook_mutex);
	if (!p)
		return;

	nf_queue_nf_hook_drop(net);
	nf_hook_entries_free(p);
}

void nf_unregister_net_hook(struct net *net, const struct nf_hook_ops *reg)
{
	if (reg->pf == NFPROTO_INET) {
		__nf_unregister_net_hook(net, NFPROTO_IPV4, reg);
		__nf_unregister_net_hook(net, NFPROTO_IPV6, reg);
	} else {
		__nf_unregister_net_hook(net, reg->pf, reg);
	}
}
EXPORT_SYMBOL(nf_unregister_net_hook);

int nf_register_net_hook(struct net *net, const struct nf_hook_ops *reg)
{
	int err;

	if (reg->pf == NFPROTO_INET) {
		err = __nf_register_net_hook(net, NFPROTO_IPV4, reg);
		if (err < 0)
			return err;

		err = __nf_register_net_hook(net, NFPROTO_IPV6, reg);
		if (err < 0) {
			__nf_unregister_net_hook(net, NFPROTO_IPV4, reg);
			return err;
		}
	} else {
		err = __nf_register_net_hook(net, reg->pf, reg);
		if (err < 0)
			return err;
	}

	return 0;
}
EXPORT_SYMBOL(nf_register_net_hook);

int nf_register_net_hooks(struct net *net, const struct nf_hook_ops *reg,
			  unsigned int n)
{
	unsigned int i;
	int err = 0;

	for (i = 0; i < n; i++) {
		err = nf_register_net_hook(net, &reg[i]);
		if (err)
			goto err;
	}
	return err;

err:
	if (i > 0)
		nf_unregister_net_hooks(net, reg, i);
	return err;
}
EXPORT_SYMBOL(nf_register_net_hooks);

void nf_unregister_net_hooks(struct net *net, const struct nf_hook_ops *reg,
			     unsigned int hookcount)
{
	unsigned int i;

	for (i = 0; i < hookcount; i++)
		nf_unregister_net_hook(net, &reg[i]);
}
EXPORT_SYMBOL(nf_unregister_net_hooks);

/* Returns 1 if okfn() needs to be executed by the caller,
 * -EPERM for NF_DROP, 0 otherwise.  Caller must hold rcu_read_lock. */
int nf_hook_slow(struct sk_buff *skb, struct nf_hook_state *state,
		 const struct nf_hook_entries *e, unsigned int s)
{
	unsigned int verdict;
	int ret;

	for (; s < e->num_hook_entries; s++) {
		verdict = nf_hook_entry_hookfn(&e->hooks[s], skb, state);
		switch (verdict & NF_VERDICT_MASK) {
		case NF_ACCEPT:
			break;
		case NF_DROP:
			kfree_skb(skb);
			ret = NF_DROP_GETERR(verdict);
			if (ret == 0)
				ret = -EPERM;
			return ret;
		case NF_QUEUE:
			ret = nf_queue(skb, state, e, s, verdict);
			if (ret == 1)
				continue;
			return ret;
		default:
			/* Implicit handling for NF_STOLEN, as well as any other
			 * non conventional verdicts.
			 */
			return 0;
		}
	}

	return 1;
}
EXPORT_SYMBOL(nf_hook_slow);


int skb_make_writable(struct sk_buff *skb, unsigned int writable_len)
{
	if (writable_len > skb->len)
		return 0;

	/* Not exclusive use of packet?  Must copy. */
	if (!skb_cloned(skb)) {
		if (writable_len <= skb_headlen(skb))
			return 1;
	} else if (skb_clone_writable(skb, writable_len))
		return 1;

	if (writable_len <= skb_headlen(skb))
		writable_len = 0;
	else
		writable_len -= skb_headlen(skb);

	return !!__pskb_pull_tail(skb, writable_len);
}
EXPORT_SYMBOL(skb_make_writable);

/* This needs to be compiled in any case to avoid dependencies between the
 * nfnetlink_queue code and nf_conntrack.
 */
struct nfnl_ct_hook __rcu *nfnl_ct_hook __read_mostly;
EXPORT_SYMBOL_GPL(nfnl_ct_hook);

#if IS_ENABLED(CONFIG_NF_CONNTRACK)
/* This does not belong here, but locally generated errors need it if connection
   tracking in use: without this, connection may not be in hash table, and hence
   manufactured ICMP or RST packets will not be associated with it. */
void (*ip_ct_attach)(struct sk_buff *, const struct sk_buff *)
		__rcu __read_mostly;
EXPORT_SYMBOL(ip_ct_attach);

void nf_ct_attach(struct sk_buff *new, const struct sk_buff *skb)
{
	void (*attach)(struct sk_buff *, const struct sk_buff *);

	if (skb->_nfct) {
		rcu_read_lock();
		attach = rcu_dereference(ip_ct_attach);
		if (attach)
			attach(new, skb);
		rcu_read_unlock();
	}
}
EXPORT_SYMBOL(nf_ct_attach);

void (*nf_ct_destroy)(struct nf_conntrack *) __rcu __read_mostly;
EXPORT_SYMBOL(nf_ct_destroy);

void nf_conntrack_destroy(struct nf_conntrack *nfct)
{
	void (*destroy)(struct nf_conntrack *);

	rcu_read_lock();
	destroy = rcu_dereference(nf_ct_destroy);
	BUG_ON(destroy == NULL);
	destroy(nfct);
	rcu_read_unlock();
}
EXPORT_SYMBOL(nf_conntrack_destroy);

/* Built-in default zone used e.g. by modules. */
const struct nf_conntrack_zone nf_ct_zone_dflt = {
	.id	= NF_CT_DEFAULT_ZONE_ID,
	.dir	= NF_CT_DEFAULT_ZONE_DIR,
};
EXPORT_SYMBOL_GPL(nf_ct_zone_dflt);
#endif /* CONFIG_NF_CONNTRACK */

#ifdef CONFIG_NF_NAT_NEEDED
void (*nf_nat_decode_session_hook)(struct sk_buff *, struct flowi *);
EXPORT_SYMBOL(nf_nat_decode_session_hook);
#endif

static void __net_init __netfilter_net_init(struct nf_hook_entries **e, int max)
{
	int h;

	for (h = 0; h < max; h++)
		RCU_INIT_POINTER(e[h], NULL);
}

static int __net_init netfilter_net_init(struct net *net)
{
	__netfilter_net_init(net->nf.hooks_ipv4, ARRAY_SIZE(net->nf.hooks_ipv4));
	__netfilter_net_init(net->nf.hooks_ipv6, ARRAY_SIZE(net->nf.hooks_ipv6));
#ifdef CONFIG_NETFILTER_FAMILY_ARP
	__netfilter_net_init(net->nf.hooks_arp, ARRAY_SIZE(net->nf.hooks_arp));
#endif
#ifdef CONFIG_NETFILTER_FAMILY_BRIDGE
	__netfilter_net_init(net->nf.hooks_bridge, ARRAY_SIZE(net->nf.hooks_bridge));
#endif
#if IS_ENABLED(CONFIG_DECNET)
	__netfilter_net_init(net->nf.hooks_decnet, ARRAY_SIZE(net->nf.hooks_decnet));
#endif

#ifdef CONFIG_PROC_FS
	net->nf.proc_netfilter = proc_net_mkdir(net, "netfilter",
						net->proc_net);
	if (!net->nf.proc_netfilter) {
		if (!net_eq(net, &init_net))
			pr_err("cannot create netfilter proc entry");

		return -ENOMEM;
	}
#endif

	return 0;
}

static void __net_exit netfilter_net_exit(struct net *net)
{
	remove_proc_entry("netfilter", net->proc_net);
}

static struct pernet_operations netfilter_net_ops = {
	.init = netfilter_net_init,
	.exit = netfilter_net_exit,
};

int __init netfilter_init(void)
{
	int ret;

	ret = register_pernet_subsys(&netfilter_net_ops);
	if (ret < 0)
		goto err;

	ret = netfilter_log_init();
	if (ret < 0)
		goto err_pernet;

	return 0;
err_pernet:
	unregister_pernet_subsys(&netfilter_net_ops);
err:
	return ret;
}
