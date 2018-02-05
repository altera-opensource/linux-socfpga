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
 * You should have received a copy of the GNU General Public License
 * version 2 along with this program; If not, see
 * http://www.gnu.org/licenses/gpl-2.0.html
 *
 * GPL HEADER END
 */
/*
 * Copyright (c) 2008, 2010, Oracle and/or its affiliates. All rights reserved.
 * Use is subject to license terms.
 *
 * Copyright (c) 2011, 2015, Intel Corporation.
 */
/*
 * This file is part of Lustre, http://www.lustre.org/
 * Lustre is a trademark of Sun Microsystems, Inc.
 */

#ifndef __LIBCFS_LIBCFS_H__
#define __LIBCFS_LIBCFS_H__

#include <linux/gfp.h>
#include <linux/list.h>

#include <uapi/linux/lnet/libcfs_ioctl.h>
#include <linux/libcfs/linux/libcfs.h>
#include <linux/libcfs/libcfs_debug.h>
#include <linux/libcfs/libcfs_private.h>
#include <linux/libcfs/libcfs_cpu.h>
#include <linux/libcfs/libcfs_prim.h>
#include <linux/libcfs/libcfs_time.h>
#include <linux/libcfs/libcfs_string.h>
#include <linux/libcfs/libcfs_hash.h>
#include <linux/libcfs/libcfs_fail.h>
#include <linux/libcfs/curproc.h>

#define LIBCFS_VERSION "0.7.0"

#define LOWEST_BIT_SET(x)       ((x) & ~((x) - 1))

/*
 * Lustre Error Checksum: calculates checksum
 * of Hex number by XORing each bit.
 */
#define LERRCHKSUM(hexnum) (((hexnum) & 0xf) ^ ((hexnum) >> 4 & 0xf) ^ \
			   ((hexnum) >> 8 & 0xf))

/* need both kernel and user-land acceptor */
#define LNET_ACCEPTOR_MIN_RESERVED_PORT    512
#define LNET_ACCEPTOR_MAX_RESERVED_PORT    1023

/*
 * Defined by platform
 */
sigset_t cfs_block_allsigs(void);
sigset_t cfs_block_sigs(unsigned long sigs);
sigset_t cfs_block_sigsinv(unsigned long sigs);
void cfs_restore_sigs(sigset_t sigset);
void cfs_clear_sigpending(void);

struct libcfs_ioctl_handler {
	struct list_head item;
	int (*handle_ioctl)(unsigned int cmd, struct libcfs_ioctl_hdr *hdr);
};

#define DECLARE_IOCTL_HANDLER(ident, func)			\
	struct libcfs_ioctl_handler ident = {			\
		.item		= LIST_HEAD_INIT(ident.item),	\
		.handle_ioctl	= func				\
	}

int libcfs_register_ioctl(struct libcfs_ioctl_handler *hand);
int libcfs_deregister_ioctl(struct libcfs_ioctl_handler *hand);

int libcfs_ioctl_getdata(struct libcfs_ioctl_hdr **hdr_pp,
			 const struct libcfs_ioctl_hdr __user *uparam);
int libcfs_ioctl_data_adjust(struct libcfs_ioctl_data *data);
int libcfs_ioctl(unsigned long cmd, void __user *arg);

/* container_of depends on "likely" which is defined in libcfs_private.h */
static inline void *__container_of(void *ptr, unsigned long shift)
{
	if (IS_ERR_OR_NULL(ptr))
		return ptr;
	return (char *)ptr - shift;
}

#define container_of0(ptr, type, member) \
	((type *)__container_of((void *)(ptr), offsetof(type, member)))

#define _LIBCFS_H

void *libcfs_kvzalloc(size_t size, gfp_t flags);
void *libcfs_kvzalloc_cpt(struct cfs_cpt_table *cptab, int cpt, size_t size,
			  gfp_t flags);

extern struct miscdevice libcfs_dev;
/**
 * The path of debug log dump upcall script.
 */
extern char lnet_debug_log_upcall[1024];

extern struct workqueue_struct *cfs_rehash_wq;

struct lnet_debugfs_symlink_def {
	char *name;
	char *target;
};

void lustre_insert_debugfs(struct ctl_table *table,
			   const struct lnet_debugfs_symlink_def *symlinks);
int lprocfs_call_handler(void *data, int write, loff_t *ppos,
			 void __user *buffer, size_t *lenp,
			 int (*handler)(void *data, int write, loff_t pos,
					void __user *buffer, int len));

#endif /* _LIBCFS_H */
