/*
 * Generic pidhash and scalable, time-bounded PID allocator
 *
 * (C) 2002-2003 Nadia Yvette Chambers, IBM
 * (C) 2004 Nadia Yvette Chambers, Oracle
 * (C) 2002-2004 Ingo Molnar, Red Hat
 *
 * pid-structures are backing objects for tasks sharing a given ID to chain
 * against. There is very little to them aside from hashing them and
 * parking tasks using given ID's on a list.
 *
 * The hash is always changed with the tasklist_lock write-acquired,
 * and the hash is only accessed with the tasklist_lock at least
 * read-acquired, so there's no additional SMP locking needed here.
 *
 * We have a list of bitmap pages, which bitmaps represent the PID space.
 * Allocating and freeing PIDs is completely lockless. The worst-case
 * allocation scenario when all but one out of 1 million PIDs possible are
 * allocated already: the scanning of 32 list entries and at most PAGE_SIZE
 * bytes. The typical fastpath is a single successful setbit. Freeing is O(1).
 *
 * Pid namespaces:
 *    (C) 2007 Pavel Emelyanov <xemul@openvz.org>, OpenVZ, SWsoft Inc.
 *    (C) 2007 Sukadev Bhattiprolu <sukadev@us.ibm.com>, IBM
 *     Many thanks to Oleg Nesterov for comments and help
 *
 */

#include <linux/mm.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/rculist.h>
#include <linux/bootmem.h>
#include <linux/hash.h>
#include <linux/pid_namespace.h>
#include <linux/init_task.h>
#include <linux/syscalls.h>
#include <linux/proc_ns.h>
#include <linux/proc_fs.h>
#include <linux/sched/task.h>
#include <linux/idr.h>

struct pid init_struct_pid = {
	.count 		= ATOMIC_INIT(1),
	.tasks		= {
		{ .first = NULL },
		{ .first = NULL },
		{ .first = NULL },
	},
	.level		= 0,
	.numbers	= { {
		.nr		= 0,
		.ns		= &init_pid_ns,
	}, }
};

int pid_max = PID_MAX_DEFAULT;

#define RESERVED_PIDS		300

int pid_max_min = RESERVED_PIDS + 1;
int pid_max_max = PID_MAX_LIMIT;

/*
 * PID-map pages start out as NULL, they get allocated upon
 * first use and are never deallocated. This way a low pid_max
 * value does not cause lots of bitmaps to be allocated, but
 * the scheme scales to up to 4 million PIDs, runtime.
 */
struct pid_namespace init_pid_ns = {
	.kref = KREF_INIT(2),
	.idr = IDR_INIT,
	.pid_allocated = PIDNS_ADDING,
	.level = 0,
	.child_reaper = &init_task,
	.user_ns = &init_user_ns,
	.ns.inum = PROC_PID_INIT_INO,
#ifdef CONFIG_PID_NS
	.ns.ops = &pidns_operations,
#endif
};
EXPORT_SYMBOL_GPL(init_pid_ns);

/*
 * Note: disable interrupts while the pidmap_lock is held as an
 * interrupt might come in and do read_lock(&tasklist_lock).
 *
 * If we don't disable interrupts there is a nasty deadlock between
 * detach_pid()->free_pid() and another cpu that does
 * spin_lock(&pidmap_lock) followed by an interrupt routine that does
 * read_lock(&tasklist_lock);
 *
 * After we clean up the tasklist_lock and know there are no
 * irq handlers that take it we can leave the interrupts enabled.
 * For now it is easier to be safe than to prove it can't happen.
 */

static  __cacheline_aligned_in_smp DEFINE_SPINLOCK(pidmap_lock);

void put_pid(struct pid *pid)
{
	struct pid_namespace *ns;

	if (!pid)
		return;

	ns = pid->numbers[pid->level].ns;
	if ((atomic_read(&pid->count) == 1) ||
	     atomic_dec_and_test(&pid->count)) {
		kmem_cache_free(ns->pid_cachep, pid);
		put_pid_ns(ns);
	}
}
EXPORT_SYMBOL_GPL(put_pid);

static void delayed_put_pid(struct rcu_head *rhp)
{
	struct pid *pid = container_of(rhp, struct pid, rcu);
	put_pid(pid);
}

void free_pid(struct pid *pid)
{
	/* We can be called with write_lock_irq(&tasklist_lock) held */
	int i;
	unsigned long flags;

	spin_lock_irqsave(&pidmap_lock, flags);
	for (i = 0; i <= pid->level; i++) {
		struct upid *upid = pid->numbers + i;
		struct pid_namespace *ns = upid->ns;
		switch (--ns->pid_allocated) {
		case 2:
		case 1:
			/* When all that is left in the pid namespace
			 * is the reaper wake up the reaper.  The reaper
			 * may be sleeping in zap_pid_ns_processes().
			 */
			wake_up_process(ns->child_reaper);
			break;
		case PIDNS_ADDING:
			/* Handle a fork failure of the first process */
			WARN_ON(ns->child_reaper);
			ns->pid_allocated = 0;
			/* fall through */
		case 0:
			schedule_work(&ns->proc_work);
			break;
		}

		idr_remove(&ns->idr, upid->nr);
	}
	spin_unlock_irqrestore(&pidmap_lock, flags);

	call_rcu(&pid->rcu, delayed_put_pid);
}

struct pid *alloc_pid(struct pid_namespace *ns)
{
	struct pid *pid;
	enum pid_type type;
	int i, nr;
	struct pid_namespace *tmp;
	struct upid *upid;
	int retval = -ENOMEM;

	pid = kmem_cache_alloc(ns->pid_cachep, GFP_KERNEL);
	if (!pid)
		return ERR_PTR(retval);

	tmp = ns;
	pid->level = ns->level;

	for (i = ns->level; i >= 0; i--) {
		int pid_min = 1;

		idr_preload(GFP_KERNEL);
		spin_lock_irq(&pidmap_lock);

		/*
		 * init really needs pid 1, but after reaching the maximum
		 * wrap back to RESERVED_PIDS
		 */
		if (idr_get_cursor(&tmp->idr) > RESERVED_PIDS)
			pid_min = RESERVED_PIDS;

		/*
		 * Store a null pointer so find_pid_ns does not find
		 * a partially initialized PID (see below).
		 */
		nr = idr_alloc_cyclic(&tmp->idr, NULL, pid_min,
				      pid_max, GFP_ATOMIC);
		spin_unlock_irq(&pidmap_lock);
		idr_preload_end();

		if (nr < 0) {
			retval = nr;
			goto out_free;
		}

		pid->numbers[i].nr = nr;
		pid->numbers[i].ns = tmp;
		tmp = tmp->parent;
	}

	if (unlikely(is_child_reaper(pid))) {
		if (pid_ns_prepare_proc(ns))
			goto out_free;
	}

	get_pid_ns(ns);
	atomic_set(&pid->count, 1);
	for (type = 0; type < PIDTYPE_MAX; ++type)
		INIT_HLIST_HEAD(&pid->tasks[type]);

	upid = pid->numbers + ns->level;
	spin_lock_irq(&pidmap_lock);
	if (!(ns->pid_allocated & PIDNS_ADDING))
		goto out_unlock;
	for ( ; upid >= pid->numbers; --upid) {
		/* Make the PID visible to find_pid_ns. */
		idr_replace(&upid->ns->idr, pid, upid->nr);
		upid->ns->pid_allocated++;
	}
	spin_unlock_irq(&pidmap_lock);

	return pid;

out_unlock:
	spin_unlock_irq(&pidmap_lock);
	put_pid_ns(ns);

out_free:
	spin_lock_irq(&pidmap_lock);
	while (++i <= ns->level)
		idr_remove(&ns->idr, (pid->numbers + i)->nr);

	/* On failure to allocate the first pid, reset the state */
	if (ns->pid_allocated == PIDNS_ADDING)
		idr_set_cursor(&ns->idr, 0);

	spin_unlock_irq(&pidmap_lock);

	kmem_cache_free(ns->pid_cachep, pid);
	return ERR_PTR(retval);
}

void disable_pid_allocation(struct pid_namespace *ns)
{
	spin_lock_irq(&pidmap_lock);
	ns->pid_allocated &= ~PIDNS_ADDING;
	spin_unlock_irq(&pidmap_lock);
}

struct pid *find_pid_ns(int nr, struct pid_namespace *ns)
{
	return idr_find(&ns->idr, nr);
}
EXPORT_SYMBOL_GPL(find_pid_ns);

struct pid *find_vpid(int nr)
{
	return find_pid_ns(nr, task_active_pid_ns(current));
}
EXPORT_SYMBOL_GPL(find_vpid);

/*
 * attach_pid() must be called with the tasklist_lock write-held.
 */
void attach_pid(struct task_struct *task, enum pid_type type)
{
	struct pid_link *link = &task->pids[type];
	hlist_add_head_rcu(&link->node, &link->pid->tasks[type]);
}

static void __change_pid(struct task_struct *task, enum pid_type type,
			struct pid *new)
{
	struct pid_link *link;
	struct pid *pid;
	int tmp;

	link = &task->pids[type];
	pid = link->pid;

	hlist_del_rcu(&link->node);
	link->pid = new;

	for (tmp = PIDTYPE_MAX; --tmp >= 0; )
		if (!hlist_empty(&pid->tasks[tmp]))
			return;

	free_pid(pid);
}

void detach_pid(struct task_struct *task, enum pid_type type)
{
	__change_pid(task, type, NULL);
}

void change_pid(struct task_struct *task, enum pid_type type,
		struct pid *pid)
{
	__change_pid(task, type, pid);
	attach_pid(task, type);
}

/* transfer_pid is an optimization of attach_pid(new), detach_pid(old) */
void transfer_pid(struct task_struct *old, struct task_struct *new,
			   enum pid_type type)
{
	new->pids[type].pid = old->pids[type].pid;
	hlist_replace_rcu(&old->pids[type].node, &new->pids[type].node);
}

struct task_struct *pid_task(struct pid *pid, enum pid_type type)
{
	struct task_struct *result = NULL;
	if (pid) {
		struct hlist_node *first;
		first = rcu_dereference_check(hlist_first_rcu(&pid->tasks[type]),
					      lockdep_tasklist_lock_is_held());
		if (first)
			result = hlist_entry(first, struct task_struct, pids[(type)].node);
	}
	return result;
}
EXPORT_SYMBOL(pid_task);

/*
 * Must be called under rcu_read_lock().
 */
struct task_struct *find_task_by_pid_ns(pid_t nr, struct pid_namespace *ns)
{
	RCU_LOCKDEP_WARN(!rcu_read_lock_held(),
			 "find_task_by_pid_ns() needs rcu_read_lock() protection");
	return pid_task(find_pid_ns(nr, ns), PIDTYPE_PID);
}

struct task_struct *find_task_by_vpid(pid_t vnr)
{
	return find_task_by_pid_ns(vnr, task_active_pid_ns(current));
}

struct pid *get_task_pid(struct task_struct *task, enum pid_type type)
{
	struct pid *pid;
	rcu_read_lock();
	if (type != PIDTYPE_PID)
		task = task->group_leader;
	pid = get_pid(rcu_dereference(task->pids[type].pid));
	rcu_read_unlock();
	return pid;
}
EXPORT_SYMBOL_GPL(get_task_pid);

struct task_struct *get_pid_task(struct pid *pid, enum pid_type type)
{
	struct task_struct *result;
	rcu_read_lock();
	result = pid_task(pid, type);
	if (result)
		get_task_struct(result);
	rcu_read_unlock();
	return result;
}
EXPORT_SYMBOL_GPL(get_pid_task);

struct pid *find_get_pid(pid_t nr)
{
	struct pid *pid;

	rcu_read_lock();
	pid = get_pid(find_vpid(nr));
	rcu_read_unlock();

	return pid;
}
EXPORT_SYMBOL_GPL(find_get_pid);

pid_t pid_nr_ns(struct pid *pid, struct pid_namespace *ns)
{
	struct upid *upid;
	pid_t nr = 0;

	if (pid && ns->level <= pid->level) {
		upid = &pid->numbers[ns->level];
		if (upid->ns == ns)
			nr = upid->nr;
	}
	return nr;
}
EXPORT_SYMBOL_GPL(pid_nr_ns);

pid_t pid_vnr(struct pid *pid)
{
	return pid_nr_ns(pid, task_active_pid_ns(current));
}
EXPORT_SYMBOL_GPL(pid_vnr);

pid_t __task_pid_nr_ns(struct task_struct *task, enum pid_type type,
			struct pid_namespace *ns)
{
	pid_t nr = 0;

	rcu_read_lock();
	if (!ns)
		ns = task_active_pid_ns(current);
	if (likely(pid_alive(task))) {
		if (type != PIDTYPE_PID) {
			if (type == __PIDTYPE_TGID)
				type = PIDTYPE_PID;

			task = task->group_leader;
		}
		nr = pid_nr_ns(rcu_dereference(task->pids[type].pid), ns);
	}
	rcu_read_unlock();

	return nr;
}
EXPORT_SYMBOL(__task_pid_nr_ns);

struct pid_namespace *task_active_pid_ns(struct task_struct *tsk)
{
	return ns_of_pid(task_pid(tsk));
}
EXPORT_SYMBOL_GPL(task_active_pid_ns);

/*
 * Used by proc to find the first pid that is greater than or equal to nr.
 *
 * If there is a pid at nr this function is exactly the same as find_pid_ns.
 */
struct pid *find_ge_pid(int nr, struct pid_namespace *ns)
{
	return idr_get_next(&ns->idr, &nr);
}

void __init pid_idr_init(void)
{
	/* Verify no one has done anything silly: */
	BUILD_BUG_ON(PID_MAX_LIMIT >= PIDNS_ADDING);

	/* bump default and minimum pid_max based on number of cpus */
	pid_max = min(pid_max_max, max_t(int, pid_max,
				PIDS_PER_CPU_DEFAULT * num_possible_cpus()));
	pid_max_min = max_t(int, pid_max_min,
				PIDS_PER_CPU_MIN * num_possible_cpus());
	pr_info("pid_max: default: %u minimum: %u\n", pid_max, pid_max_min);

	idr_init(&init_pid_ns.idr);

	init_pid_ns.pid_cachep = KMEM_CACHE(pid,
			SLAB_HWCACHE_ALIGN | SLAB_PANIC | SLAB_ACCOUNT);
}
