// SPDX-License-Identifier: GPL-2.0
/*
 * GPL HEADER START
 *
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 only,
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License version 2 for more details (a copy is included
 * in the LICENSE file that accompanied this code).
 *
 * GPL HEADER END
 */
/*
 * Copyright (c) 2010, Oracle and/or its affiliates. All rights reserved.
 *
 * Copyright (c) 2012, 2015 Intel Corporation.
 */
/*
 * This file is part of Lustre, http://www.lustre.org/
 * Lustre is a trademark of Sun Microsystems, Inc.
 *
 * Author: liang@whamcloud.com
 */

#define DEBUG_SUBSYSTEM S_LNET

#include <linux/cpu.h>
#include <linux/sched.h>
#include <linux/libcfs/libcfs.h>

#ifdef CONFIG_SMP

/**
 * modparam for setting number of partitions
 *
 *  0 : estimate best value based on cores or NUMA nodes
 *  1 : disable multiple partitions
 * >1 : specify number of partitions
 */
static int	cpu_npartitions;
module_param(cpu_npartitions, int, 0444);
MODULE_PARM_DESC(cpu_npartitions, "# of CPU partitions");

/**
 * modparam for setting CPU partitions patterns:
 *
 * i.e: "0[0,1,2,3] 1[4,5,6,7]", number before bracket is CPU partition ID,
 *      number in bracket is processor ID (core or HT)
 *
 * i.e: "N 0[0,1] 1[2,3]" the first character 'N' means numbers in bracket
 *       are NUMA node ID, number before bracket is CPU partition ID.
 *
 * i.e: "N", shortcut expression to create CPT from NUMA & CPU topology
 *
 * NB: If user specified cpu_pattern, cpu_npartitions will be ignored
 */
static char	*cpu_pattern = "N";
module_param(cpu_pattern, charp, 0444);
MODULE_PARM_DESC(cpu_pattern, "CPU partitions pattern");

struct cfs_cpt_data {
	/* serialize hotplug etc */
	spinlock_t		cpt_lock;
	/* reserved for hotplug */
	unsigned long		cpt_version;
	/* mutex to protect cpt_cpumask */
	struct mutex		cpt_mutex;
	/* scratch buffer for set/unset_node */
	cpumask_var_t		cpt_cpumask;
};

static struct cfs_cpt_data	cpt_data;

static void
cfs_node_to_cpumask(int node, cpumask_t *mask)
{
	const cpumask_t *tmp = cpumask_of_node(node);

	if (tmp)
		cpumask_copy(mask, tmp);
	else
		cpumask_clear(mask);
}

void
cfs_cpt_table_free(struct cfs_cpt_table *cptab)
{
	int i;

	kvfree(cptab->ctb_cpu2cpt);

	for (i = 0; cptab->ctb_parts && i < cptab->ctb_nparts; i++) {
		struct cfs_cpu_partition *part = &cptab->ctb_parts[i];

		kfree(part->cpt_nodemask);
		free_cpumask_var(part->cpt_cpumask);
	}

	kvfree(cptab->ctb_parts);

	kfree(cptab->ctb_nodemask);
	free_cpumask_var(cptab->ctb_cpumask);

	kfree(cptab);
}
EXPORT_SYMBOL(cfs_cpt_table_free);

struct cfs_cpt_table *
cfs_cpt_table_alloc(unsigned int ncpt)
{
	struct cfs_cpt_table *cptab;
	int i;

	cptab = kzalloc(sizeof(*cptab), GFP_NOFS);
	if (!cptab)
		return NULL;

	cptab->ctb_nparts = ncpt;

	cptab->ctb_nodemask = kzalloc(sizeof(*cptab->ctb_nodemask),
				      GFP_NOFS);
	if (!zalloc_cpumask_var(&cptab->ctb_cpumask, GFP_NOFS) ||
	    !cptab->ctb_nodemask)
		goto failed;

	cptab->ctb_cpu2cpt = kvmalloc_array(num_possible_cpus(),
					    sizeof(cptab->ctb_cpu2cpt[0]),
					    GFP_KERNEL);
	if (!cptab->ctb_cpu2cpt)
		goto failed;

	memset(cptab->ctb_cpu2cpt, -1,
	       num_possible_cpus() * sizeof(cptab->ctb_cpu2cpt[0]));

	cptab->ctb_parts = kvmalloc_array(ncpt, sizeof(cptab->ctb_parts[0]),
					  GFP_KERNEL);
	if (!cptab->ctb_parts)
		goto failed;

	for (i = 0; i < ncpt; i++) {
		struct cfs_cpu_partition *part = &cptab->ctb_parts[i];

		part->cpt_nodemask = kzalloc(sizeof(*part->cpt_nodemask),
					     GFP_NOFS);
		if (!zalloc_cpumask_var(&part->cpt_cpumask, GFP_NOFS) ||
		    !part->cpt_nodemask)
			goto failed;
	}

	spin_lock(&cpt_data.cpt_lock);
	/* Reserved for hotplug */
	cptab->ctb_version = cpt_data.cpt_version;
	spin_unlock(&cpt_data.cpt_lock);

	return cptab;

 failed:
	cfs_cpt_table_free(cptab);
	return NULL;
}
EXPORT_SYMBOL(cfs_cpt_table_alloc);

int
cfs_cpt_table_print(struct cfs_cpt_table *cptab, char *buf, int len)
{
	char *tmp = buf;
	int rc = 0;
	int i;
	int j;

	for (i = 0; i < cptab->ctb_nparts; i++) {
		if (len > 0) {
			rc = snprintf(tmp, len, "%d\t: ", i);
			len -= rc;
		}

		if (len <= 0) {
			rc = -EFBIG;
			goto out;
		}

		tmp += rc;
		for_each_cpu(j, cptab->ctb_parts[i].cpt_cpumask) {
			rc = snprintf(tmp, len, "%d ", j);
			len -= rc;
			if (len <= 0) {
				rc = -EFBIG;
				goto out;
			}
			tmp += rc;
		}

		*tmp = '\n';
		tmp++;
		len--;
	}

 out:
	if (rc < 0)
		return rc;

	return tmp - buf;
}
EXPORT_SYMBOL(cfs_cpt_table_print);

int
cfs_cpt_number(struct cfs_cpt_table *cptab)
{
	return cptab->ctb_nparts;
}
EXPORT_SYMBOL(cfs_cpt_number);

int
cfs_cpt_weight(struct cfs_cpt_table *cptab, int cpt)
{
	LASSERT(cpt == CFS_CPT_ANY || (cpt >= 0 && cpt < cptab->ctb_nparts));

	return cpt == CFS_CPT_ANY ?
	       cpumask_weight(cptab->ctb_cpumask) :
	       cpumask_weight(cptab->ctb_parts[cpt].cpt_cpumask);
}
EXPORT_SYMBOL(cfs_cpt_weight);

int
cfs_cpt_online(struct cfs_cpt_table *cptab, int cpt)
{
	LASSERT(cpt == CFS_CPT_ANY || (cpt >= 0 && cpt < cptab->ctb_nparts));

	return cpt == CFS_CPT_ANY ?
	       cpumask_any_and(cptab->ctb_cpumask,
			       cpu_online_mask) < nr_cpu_ids :
	       cpumask_any_and(cptab->ctb_parts[cpt].cpt_cpumask,
			       cpu_online_mask) < nr_cpu_ids;
}
EXPORT_SYMBOL(cfs_cpt_online);

cpumask_var_t *
cfs_cpt_cpumask(struct cfs_cpt_table *cptab, int cpt)
{
	LASSERT(cpt == CFS_CPT_ANY || (cpt >= 0 && cpt < cptab->ctb_nparts));

	return cpt == CFS_CPT_ANY ?
	       &cptab->ctb_cpumask : &cptab->ctb_parts[cpt].cpt_cpumask;
}
EXPORT_SYMBOL(cfs_cpt_cpumask);

nodemask_t *
cfs_cpt_nodemask(struct cfs_cpt_table *cptab, int cpt)
{
	LASSERT(cpt == CFS_CPT_ANY || (cpt >= 0 && cpt < cptab->ctb_nparts));

	return cpt == CFS_CPT_ANY ?
	       cptab->ctb_nodemask : cptab->ctb_parts[cpt].cpt_nodemask;
}
EXPORT_SYMBOL(cfs_cpt_nodemask);

int
cfs_cpt_set_cpu(struct cfs_cpt_table *cptab, int cpt, int cpu)
{
	int node;

	LASSERT(cpt >= 0 && cpt < cptab->ctb_nparts);

	if (cpu < 0 || cpu >= nr_cpu_ids || !cpu_online(cpu)) {
		CDEBUG(D_INFO, "CPU %d is invalid or it's offline\n", cpu);
		return 0;
	}

	if (cptab->ctb_cpu2cpt[cpu] != -1) {
		CDEBUG(D_INFO, "CPU %d is already in partition %d\n",
		       cpu, cptab->ctb_cpu2cpt[cpu]);
		return 0;
	}

	cptab->ctb_cpu2cpt[cpu] = cpt;

	LASSERT(!cpumask_test_cpu(cpu, cptab->ctb_cpumask));
	LASSERT(!cpumask_test_cpu(cpu, cptab->ctb_parts[cpt].cpt_cpumask));

	cpumask_set_cpu(cpu, cptab->ctb_cpumask);
	cpumask_set_cpu(cpu, cptab->ctb_parts[cpt].cpt_cpumask);

	node = cpu_to_node(cpu);

	/* first CPU of @node in this CPT table */
	if (!node_isset(node, *cptab->ctb_nodemask))
		node_set(node, *cptab->ctb_nodemask);

	/* first CPU of @node in this partition */
	if (!node_isset(node, *cptab->ctb_parts[cpt].cpt_nodemask))
		node_set(node, *cptab->ctb_parts[cpt].cpt_nodemask);

	return 1;
}
EXPORT_SYMBOL(cfs_cpt_set_cpu);

void
cfs_cpt_unset_cpu(struct cfs_cpt_table *cptab, int cpt, int cpu)
{
	int node;
	int i;

	LASSERT(cpt == CFS_CPT_ANY || (cpt >= 0 && cpt < cptab->ctb_nparts));

	if (cpu < 0 || cpu >= nr_cpu_ids) {
		CDEBUG(D_INFO, "Invalid CPU id %d\n", cpu);
		return;
	}

	if (cpt == CFS_CPT_ANY) {
		/* caller doesn't know the partition ID */
		cpt = cptab->ctb_cpu2cpt[cpu];
		if (cpt < 0) { /* not set in this CPT-table */
			CDEBUG(D_INFO, "Try to unset cpu %d which is not in CPT-table %p\n",
			       cpt, cptab);
			return;
		}

	} else if (cpt != cptab->ctb_cpu2cpt[cpu]) {
		CDEBUG(D_INFO,
		       "CPU %d is not in cpu-partition %d\n", cpu, cpt);
		return;
	}

	LASSERT(cpumask_test_cpu(cpu, cptab->ctb_parts[cpt].cpt_cpumask));
	LASSERT(cpumask_test_cpu(cpu, cptab->ctb_cpumask));

	cpumask_clear_cpu(cpu, cptab->ctb_parts[cpt].cpt_cpumask);
	cpumask_clear_cpu(cpu, cptab->ctb_cpumask);
	cptab->ctb_cpu2cpt[cpu] = -1;

	node = cpu_to_node(cpu);

	LASSERT(node_isset(node, *cptab->ctb_parts[cpt].cpt_nodemask));
	LASSERT(node_isset(node, *cptab->ctb_nodemask));

	for_each_cpu(i, cptab->ctb_parts[cpt].cpt_cpumask) {
		/* this CPT has other CPU belonging to this node? */
		if (cpu_to_node(i) == node)
			break;
	}

	if (i >= nr_cpu_ids)
		node_clear(node, *cptab->ctb_parts[cpt].cpt_nodemask);

	for_each_cpu(i, cptab->ctb_cpumask) {
		/* this CPT-table has other CPU belonging to this node? */
		if (cpu_to_node(i) == node)
			break;
	}

	if (i >= nr_cpu_ids)
		node_clear(node, *cptab->ctb_nodemask);
}
EXPORT_SYMBOL(cfs_cpt_unset_cpu);

int
cfs_cpt_set_cpumask(struct cfs_cpt_table *cptab, int cpt, cpumask_t *mask)
{
	int i;

	if (!cpumask_weight(mask) ||
	    cpumask_any_and(mask, cpu_online_mask) >= nr_cpu_ids) {
		CDEBUG(D_INFO, "No online CPU is found in the CPU mask for CPU partition %d\n",
		       cpt);
		return 0;
	}

	for_each_cpu(i, mask) {
		if (!cfs_cpt_set_cpu(cptab, cpt, i))
			return 0;
	}

	return 1;
}
EXPORT_SYMBOL(cfs_cpt_set_cpumask);

void
cfs_cpt_unset_cpumask(struct cfs_cpt_table *cptab, int cpt, cpumask_t *mask)
{
	int i;

	for_each_cpu(i, mask)
		cfs_cpt_unset_cpu(cptab, cpt, i);
}
EXPORT_SYMBOL(cfs_cpt_unset_cpumask);

int
cfs_cpt_set_node(struct cfs_cpt_table *cptab, int cpt, int node)
{
	int rc;

	if (node < 0 || node >= MAX_NUMNODES) {
		CDEBUG(D_INFO,
		       "Invalid NUMA id %d for CPU partition %d\n", node, cpt);
		return 0;
	}

	mutex_lock(&cpt_data.cpt_mutex);

	cfs_node_to_cpumask(node, cpt_data.cpt_cpumask);

	rc = cfs_cpt_set_cpumask(cptab, cpt, cpt_data.cpt_cpumask);

	mutex_unlock(&cpt_data.cpt_mutex);

	return rc;
}
EXPORT_SYMBOL(cfs_cpt_set_node);

void
cfs_cpt_unset_node(struct cfs_cpt_table *cptab, int cpt, int node)
{
	if (node < 0 || node >= MAX_NUMNODES) {
		CDEBUG(D_INFO,
		       "Invalid NUMA id %d for CPU partition %d\n", node, cpt);
		return;
	}

	mutex_lock(&cpt_data.cpt_mutex);

	cfs_node_to_cpumask(node, cpt_data.cpt_cpumask);

	cfs_cpt_unset_cpumask(cptab, cpt, cpt_data.cpt_cpumask);

	mutex_unlock(&cpt_data.cpt_mutex);
}
EXPORT_SYMBOL(cfs_cpt_unset_node);

int
cfs_cpt_set_nodemask(struct cfs_cpt_table *cptab, int cpt, nodemask_t *mask)
{
	int i;

	for_each_node_mask(i, *mask) {
		if (!cfs_cpt_set_node(cptab, cpt, i))
			return 0;
	}

	return 1;
}
EXPORT_SYMBOL(cfs_cpt_set_nodemask);

void
cfs_cpt_unset_nodemask(struct cfs_cpt_table *cptab, int cpt, nodemask_t *mask)
{
	int i;

	for_each_node_mask(i, *mask)
		cfs_cpt_unset_node(cptab, cpt, i);
}
EXPORT_SYMBOL(cfs_cpt_unset_nodemask);

void
cfs_cpt_clear(struct cfs_cpt_table *cptab, int cpt)
{
	int last;
	int i;

	if (cpt == CFS_CPT_ANY) {
		last = cptab->ctb_nparts - 1;
		cpt = 0;
	} else {
		last = cpt;
	}

	for (; cpt <= last; cpt++) {
		for_each_cpu(i, cptab->ctb_parts[cpt].cpt_cpumask)
			cfs_cpt_unset_cpu(cptab, cpt, i);
	}
}
EXPORT_SYMBOL(cfs_cpt_clear);

int
cfs_cpt_spread_node(struct cfs_cpt_table *cptab, int cpt)
{
	nodemask_t *mask;
	int weight;
	int rotor;
	int node;

	/* convert CPU partition ID to HW node id */

	if (cpt < 0 || cpt >= cptab->ctb_nparts) {
		mask = cptab->ctb_nodemask;
		rotor = cptab->ctb_spread_rotor++;
	} else {
		mask = cptab->ctb_parts[cpt].cpt_nodemask;
		rotor = cptab->ctb_parts[cpt].cpt_spread_rotor++;
	}

	weight = nodes_weight(*mask);
	LASSERT(weight > 0);

	rotor %= weight;

	for_each_node_mask(node, *mask) {
		if (!rotor--)
			return node;
	}

	LBUG();
	return 0;
}
EXPORT_SYMBOL(cfs_cpt_spread_node);

int
cfs_cpt_current(struct cfs_cpt_table *cptab, int remap)
{
	int cpu;
	int cpt;

	preempt_disable();
	cpu = smp_processor_id();
	cpt = cptab->ctb_cpu2cpt[cpu];

	if (cpt < 0 && remap) {
		/* don't return negative value for safety of upper layer,
		 * instead we shadow the unknown cpu to a valid partition ID
		 */
		cpt = cpu % cptab->ctb_nparts;
	}
	preempt_enable();
	return cpt;
}
EXPORT_SYMBOL(cfs_cpt_current);

int
cfs_cpt_of_cpu(struct cfs_cpt_table *cptab, int cpu)
{
	LASSERT(cpu >= 0 && cpu < nr_cpu_ids);

	return cptab->ctb_cpu2cpt[cpu];
}
EXPORT_SYMBOL(cfs_cpt_of_cpu);

int
cfs_cpt_bind(struct cfs_cpt_table *cptab, int cpt)
{
	cpumask_var_t *cpumask;
	nodemask_t *nodemask;
	int rc;
	int i;

	LASSERT(cpt == CFS_CPT_ANY || (cpt >= 0 && cpt < cptab->ctb_nparts));

	if (cpt == CFS_CPT_ANY) {
		cpumask = &cptab->ctb_cpumask;
		nodemask = cptab->ctb_nodemask;
	} else {
		cpumask = &cptab->ctb_parts[cpt].cpt_cpumask;
		nodemask = cptab->ctb_parts[cpt].cpt_nodemask;
	}

	if (cpumask_any_and(*cpumask, cpu_online_mask) >= nr_cpu_ids) {
		CERROR("No online CPU found in CPU partition %d, did someone do CPU hotplug on system? You might need to reload Lustre modules to keep system working well.\n",
		       cpt);
		return -EINVAL;
	}

	for_each_online_cpu(i) {
		if (cpumask_test_cpu(i, *cpumask))
			continue;

		rc = set_cpus_allowed_ptr(current, *cpumask);
		set_mems_allowed(*nodemask);
		if (!rc)
			schedule(); /* switch to allowed CPU */

		return rc;
	}

	/* don't need to set affinity because all online CPUs are covered */
	return 0;
}
EXPORT_SYMBOL(cfs_cpt_bind);

/**
 * Choose max to \a number CPUs from \a node and set them in \a cpt.
 * We always prefer to choose CPU in the same core/socket.
 */
static int
cfs_cpt_choose_ncpus(struct cfs_cpt_table *cptab, int cpt,
		     cpumask_t *node, int number)
{
	cpumask_var_t socket;
	cpumask_var_t core;
	int rc = 0;
	int cpu;

	LASSERT(number > 0);

	if (number >= cpumask_weight(node)) {
		while (!cpumask_empty(node)) {
			cpu = cpumask_first(node);

			rc = cfs_cpt_set_cpu(cptab, cpt, cpu);
			if (!rc)
				return -EINVAL;
			cpumask_clear_cpu(cpu, node);
		}
		return 0;
	}

	/*
	 * Allocate scratch buffers
	 * As we cannot initialize a cpumask_var_t, we need
	 * to alloc both before we can risk trying to free either
	 */
	if (!zalloc_cpumask_var(&socket, GFP_NOFS))
		rc = -ENOMEM;
	if (!zalloc_cpumask_var(&core, GFP_NOFS))
		rc = -ENOMEM;
	if (rc)
		goto out;

	while (!cpumask_empty(node)) {
		cpu = cpumask_first(node);

		/* get cpumask for cores in the same socket */
		cpumask_copy(socket, topology_core_cpumask(cpu));
		cpumask_and(socket, socket, node);

		LASSERT(!cpumask_empty(socket));

		while (!cpumask_empty(socket)) {
			int i;

			/* get cpumask for hts in the same core */
			cpumask_copy(core, topology_sibling_cpumask(cpu));
			cpumask_and(core, core, node);

			LASSERT(!cpumask_empty(core));

			for_each_cpu(i, core) {
				cpumask_clear_cpu(i, socket);
				cpumask_clear_cpu(i, node);

				rc = cfs_cpt_set_cpu(cptab, cpt, i);
				if (!rc) {
					rc = -EINVAL;
					goto out;
				}

				if (!--number)
					goto out;
			}
			cpu = cpumask_first(socket);
		}
	}

out:
	free_cpumask_var(socket);
	free_cpumask_var(core);
	return rc;
}

#define CPT_WEIGHT_MIN  4u

static unsigned int
cfs_cpt_num_estimate(void)
{
	unsigned int nnode = num_online_nodes();
	unsigned int ncpu = num_online_cpus();
	unsigned int ncpt;

	if (ncpu <= CPT_WEIGHT_MIN) {
		ncpt = 1;
		goto out;
	}

	/* generate reasonable number of CPU partitions based on total number
	 * of CPUs, Preferred N should be power2 and match this condition:
	 * 2 * (N - 1)^2 < NCPUS <= 2 * N^2
	 */
	for (ncpt = 2; ncpu > 2 * ncpt * ncpt; ncpt <<= 1)
		;

	if (ncpt <= nnode) { /* fat numa system */
		while (nnode > ncpt)
			nnode >>= 1;

	} else { /* ncpt > nnode */
		while ((nnode << 1) <= ncpt)
			nnode <<= 1;
	}

	ncpt = nnode;

out:
#if (BITS_PER_LONG == 32)
	/* config many CPU partitions on 32-bit system could consume
	 * too much memory
	 */
	ncpt = min(2U, ncpt);
#endif
	while (ncpu % ncpt)
		ncpt--; /* worst case is 1 */

	return ncpt;
}

static struct cfs_cpt_table *
cfs_cpt_table_create(int ncpt)
{
	struct cfs_cpt_table *cptab = NULL;
	cpumask_var_t mask;
	int cpt = 0;
	int num;
	int rc;
	int i;

	rc = cfs_cpt_num_estimate();
	if (ncpt <= 0)
		ncpt = rc;

	if (ncpt > num_online_cpus() || ncpt > 4 * rc) {
		CWARN("CPU partition number %d is larger than suggested value (%d), your system may have performance issue or run out of memory while under pressure\n",
		      ncpt, rc);
	}

	if (num_online_cpus() % ncpt) {
		CERROR("CPU number %d is not multiple of cpu_npartition %d, please try different cpu_npartitions value or set pattern string by cpu_pattern=STRING\n",
		       (int)num_online_cpus(), ncpt);
		goto failed;
	}

	cptab = cfs_cpt_table_alloc(ncpt);
	if (!cptab) {
		CERROR("Failed to allocate CPU map(%d)\n", ncpt);
		goto failed;
	}

	num = num_online_cpus() / ncpt;
	if (!num) {
		CERROR("CPU changed while setting CPU partition\n");
		goto failed;
	}

	if (!zalloc_cpumask_var(&mask, GFP_NOFS)){
		CERROR("Failed to allocate scratch cpumask\n");
		goto failed;
	}

	for_each_online_node(i) {
		cfs_node_to_cpumask(i, mask);

		while (!cpumask_empty(mask)) {
			struct cfs_cpu_partition *part;
			int n;

			/*
			 * Each emulated NUMA node has all allowed CPUs in
			 * the mask.
			 * End loop when all partitions have assigned CPUs.
			 */
			if (cpt == ncpt)
				break;

			part = &cptab->ctb_parts[cpt];

			n = num - cpumask_weight(part->cpt_cpumask);
			LASSERT(n > 0);

			rc = cfs_cpt_choose_ncpus(cptab, cpt, mask, n);
			if (rc < 0)
				goto failed_mask;

			LASSERT(num >= cpumask_weight(part->cpt_cpumask));
			if (num == cpumask_weight(part->cpt_cpumask))
				cpt++;
		}
	}

	if (cpt != ncpt ||
	    num != cpumask_weight(cptab->ctb_parts[ncpt - 1].cpt_cpumask)) {
		CERROR("Expect %d(%d) CPU partitions but got %d(%d), CPU hotplug/unplug while setting?\n",
		       cptab->ctb_nparts, num, cpt,
		       cpumask_weight(cptab->ctb_parts[ncpt - 1].cpt_cpumask));
		goto failed_mask;
	}

	free_cpumask_var(mask);

	return cptab;

 failed_mask:
	free_cpumask_var(mask);
 failed:
	CERROR("Failed to setup CPU-partition-table with %d CPU-partitions, online HW nodes: %d, HW cpus: %d.\n",
	       ncpt, num_online_nodes(), num_online_cpus());

	if (cptab)
		cfs_cpt_table_free(cptab);

	return NULL;
}

static struct cfs_cpt_table *
cfs_cpt_table_create_pattern(char *pattern)
{
	struct cfs_cpt_table *cptab;
	char *str;
	int node = 0;
	int high;
	int ncpt = 0;
	int cpt;
	int rc;
	int c;
	int i;

	str = strim(pattern);
	if (*str == 'n' || *str == 'N') {
		pattern = str + 1;
		if (*pattern != '\0') {
			node = 1;
		} else { /* shortcut to create CPT from NUMA & CPU topology */
			node = -1;
			ncpt = num_online_nodes();
		}
	}

	if (!ncpt) { /* scanning bracket which is mark of partition */
		for (str = pattern;; str++, ncpt++) {
			str = strchr(str, '[');
			if (!str)
				break;
		}
	}

	if (!ncpt ||
	    (node && ncpt > num_online_nodes()) ||
	    (!node && ncpt > num_online_cpus())) {
		CERROR("Invalid pattern %s, or too many partitions %d\n",
		       pattern, ncpt);
		return NULL;
	}

	cptab = cfs_cpt_table_alloc(ncpt);
	if (!cptab) {
		CERROR("Failed to allocate cpu partition table\n");
		return NULL;
	}

	if (node < 0) { /* shortcut to create CPT from NUMA & CPU topology */
		cpt = 0;

		for_each_online_node(i) {
			if (cpt >= ncpt) {
				CERROR("CPU changed while setting CPU partition table, %d/%d\n",
				       cpt, ncpt);
				goto failed;
			}

			rc = cfs_cpt_set_node(cptab, cpt++, i);
			if (!rc)
				goto failed;
		}
		return cptab;
	}

	high = node ? MAX_NUMNODES - 1 : nr_cpu_ids - 1;

	for (str = strim(pattern), c = 0;; c++) {
		struct cfs_range_expr *range;
		struct cfs_expr_list *el;
		char *bracket = strchr(str, '[');
		int n;

		if (!bracket) {
			if (*str) {
				CERROR("Invalid pattern %s\n", str);
				goto failed;
			}
			if (c != ncpt) {
				CERROR("expect %d partitions but found %d\n",
				       ncpt, c);
				goto failed;
			}
			break;
		}

		if (sscanf(str, "%d%n", &cpt, &n) < 1) {
			CERROR("Invalid cpu pattern %s\n", str);
			goto failed;
		}

		if (cpt < 0 || cpt >= ncpt) {
			CERROR("Invalid partition id %d, total partitions %d\n",
			       cpt, ncpt);
			goto failed;
		}

		if (cfs_cpt_weight(cptab, cpt)) {
			CERROR("Partition %d has already been set.\n", cpt);
			goto failed;
		}

		str = strim(str + n);
		if (str != bracket) {
			CERROR("Invalid pattern %s\n", str);
			goto failed;
		}

		bracket = strchr(str, ']');
		if (!bracket) {
			CERROR("missing right bracket for cpt %d, %s\n",
			       cpt, str);
			goto failed;
		}

		if (cfs_expr_list_parse(str, (bracket - str) + 1,
					0, high, &el)) {
			CERROR("Can't parse number range: %s\n", str);
			goto failed;
		}

		list_for_each_entry(range, &el->el_exprs, re_link) {
			for (i = range->re_lo; i <= range->re_hi; i++) {
				if ((i - range->re_lo) % range->re_stride)
					continue;

				rc = node ? cfs_cpt_set_node(cptab, cpt, i) :
					    cfs_cpt_set_cpu(cptab, cpt, i);
				if (!rc) {
					cfs_expr_list_free(el);
					goto failed;
				}
			}
		}

		cfs_expr_list_free(el);

		if (!cfs_cpt_online(cptab, cpt)) {
			CERROR("No online CPU is found on partition %d\n", cpt);
			goto failed;
		}

		str = strim(bracket + 1);
	}

	return cptab;

 failed:
	cfs_cpt_table_free(cptab);
	return NULL;
}

#ifdef CONFIG_HOTPLUG_CPU
static enum cpuhp_state lustre_cpu_online;

static void cfs_cpu_incr_cpt_version(void)
{
	spin_lock(&cpt_data.cpt_lock);
	cpt_data.cpt_version++;
	spin_unlock(&cpt_data.cpt_lock);
}

static int cfs_cpu_online(unsigned int cpu)
{
	cfs_cpu_incr_cpt_version();
	return 0;
}

static int cfs_cpu_dead(unsigned int cpu)
{
	bool warn;

	cfs_cpu_incr_cpt_version();

	mutex_lock(&cpt_data.cpt_mutex);
	/* if all HTs in a core are offline, it may break affinity */
	cpumask_copy(cpt_data.cpt_cpumask, topology_sibling_cpumask(cpu));
	warn = cpumask_any_and(cpt_data.cpt_cpumask,
			       cpu_online_mask) >= nr_cpu_ids;
	mutex_unlock(&cpt_data.cpt_mutex);
	CDEBUG(warn ? D_WARNING : D_INFO,
	       "Lustre: can't support CPU plug-out well now, performance and stability could be impacted [CPU %u]\n",
	       cpu);
	return 0;
}
#endif

void
cfs_cpu_fini(void)
{
	if (cfs_cpt_table)
		cfs_cpt_table_free(cfs_cpt_table);

#ifdef CONFIG_HOTPLUG_CPU
	if (lustre_cpu_online > 0)
		cpuhp_remove_state_nocalls(lustre_cpu_online);
	cpuhp_remove_state_nocalls(CPUHP_LUSTRE_CFS_DEAD);
#endif
	free_cpumask_var(cpt_data.cpt_cpumask);
}

int
cfs_cpu_init(void)
{
	int ret = 0;

	LASSERT(!cfs_cpt_table);

	memset(&cpt_data, 0, sizeof(cpt_data));

	if (!zalloc_cpumask_var(&cpt_data.cpt_cpumask, GFP_NOFS)) {
		CERROR("Failed to allocate scratch buffer\n");
		return -1;
	}

	spin_lock_init(&cpt_data.cpt_lock);
	mutex_init(&cpt_data.cpt_mutex);

#ifdef CONFIG_HOTPLUG_CPU
	ret = cpuhp_setup_state_nocalls(CPUHP_LUSTRE_CFS_DEAD,
					"staging/lustre/cfe:dead", NULL,
					cfs_cpu_dead);
	if (ret < 0)
		goto failed;
	ret = cpuhp_setup_state_nocalls(CPUHP_AP_ONLINE_DYN,
					"staging/lustre/cfe:online",
					cfs_cpu_online, NULL);
	if (ret < 0)
		goto failed;
	lustre_cpu_online = ret;
#endif
	ret = -EINVAL;

	if (*cpu_pattern) {
		char *cpu_pattern_dup = kstrdup(cpu_pattern, GFP_KERNEL);

		if (!cpu_pattern_dup) {
			CERROR("Failed to duplicate cpu_pattern\n");
			goto failed;
		}

		cfs_cpt_table = cfs_cpt_table_create_pattern(cpu_pattern_dup);
		kfree(cpu_pattern_dup);
		if (!cfs_cpt_table) {
			CERROR("Failed to create cptab from pattern %s\n",
			       cpu_pattern);
			goto failed;
		}

	} else {
		cfs_cpt_table = cfs_cpt_table_create(cpu_npartitions);
		if (!cfs_cpt_table) {
			CERROR("Failed to create ptable with npartitions %d\n",
			       cpu_npartitions);
			goto failed;
		}
	}

	spin_lock(&cpt_data.cpt_lock);
	if (cfs_cpt_table->ctb_version != cpt_data.cpt_version) {
		spin_unlock(&cpt_data.cpt_lock);
		CERROR("CPU hotplug/unplug during setup\n");
		goto failed;
	}
	spin_unlock(&cpt_data.cpt_lock);

	LCONSOLE(0, "HW nodes: %d, HW CPU cores: %d, npartitions: %d\n",
		 num_online_nodes(), num_online_cpus(),
		 cfs_cpt_number(cfs_cpt_table));
	return 0;

 failed:
	cfs_cpu_fini();
	return ret;
}

#endif
