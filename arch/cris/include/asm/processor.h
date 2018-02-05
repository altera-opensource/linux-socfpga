/* SPDX-License-Identifier: GPL-2.0 */
/*
 * include/asm-cris/processor.h
 *
 * Copyright (C) 2000, 2001 Axis Communications AB
 *
 * Authors:         Bjorn Wesen        Initial version
 *
 */

#ifndef __ASM_CRIS_PROCESSOR_H
#define __ASM_CRIS_PROCESSOR_H

#include <asm/page.h>
#include <asm/ptrace.h>
#include <arch/processor.h>
#include <arch/system.h>

struct task_struct;

#define STACK_TOP	TASK_SIZE
#define STACK_TOP_MAX	STACK_TOP

/* This decides where the kernel will search for a free chunk of vm
 * space during mmap's.
 */
#define TASK_UNMAPPED_BASE      (PAGE_ALIGN(TASK_SIZE / 3))

/*
 * At user->kernel entry, the pt_regs struct is stacked on the top of the kernel-stack.
 * This macro allows us to find those regs for a task.
 * Notice that subsequent pt_regs stackings, like recursive interrupts occurring while
 * we're in the kernel, won't affect this - only the first user->kernel transition
 * registers are reached by this.
 */

#define user_regs(thread_info) (((struct pt_regs *)((unsigned long)(thread_info) + THREAD_SIZE)) - 1)

/*
 * Dito but for the currently running task
 */

#define task_pt_regs(task) user_regs(task_thread_info(task))

unsigned long get_wchan(struct task_struct *p);

#define KSTK_ESP(tsk)   ((tsk) == current ? rdusp() : (tsk)->thread.usp)

/* Free all resources held by a thread. */
static inline void release_thread(struct task_struct *dead_task)
{
        /* Nothing needs to be done.  */
}

#define cpu_relax()     barrier()

void default_idle(void);

#endif /* __ASM_CRIS_PROCESSOR_H */
