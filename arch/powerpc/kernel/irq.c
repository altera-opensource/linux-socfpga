/*
 *  Derived from arch/i386/kernel/irq.c
 *    Copyright (C) 1992 Linus Torvalds
 *  Adapted from arch/i386 by Gary Thomas
 *    Copyright (C) 1995-1996 Gary Thomas (gdt@linuxppc.org)
 *  Updated and modified by Cort Dougan <cort@fsmlabs.com>
 *    Copyright (C) 1996-2001 Cort Dougan
 *  Adapted for Power Macintosh by Paul Mackerras
 *    Copyright (C) 1996 Paul Mackerras (paulus@cs.anu.edu.au)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * This file contains the code used by various IRQ handling routines:
 * asking for different IRQ's should be done through these routines
 * instead of just grabbing them. Thus setups with different IRQ numbers
 * shouldn't result in any weird surprises, and installing new handlers
 * should be easier.
 *
 * The MPC8xx has an interrupt mask in the SIU.  If a bit is set, the
 * interrupt is _enabled_.  As expected, IRQ0 is bit 0 in the 32-bit
 * mask register (of which only 16 are defined), hence the weird shifting
 * and complement of the cached_irq_mask.  I want to be able to stuff
 * this right into the SIU SMASK register.
 * Many of the prep/chrp functions are conditional compiled on CONFIG_PPC_8xx
 * to reduce code space and undefined function references.
 */

#undef DEBUG

#include <linux/export.h>
#include <linux/threads.h>
#include <linux/kernel_stat.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/timex.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/seq_file.h>
#include <linux/cpumask.h>
#include <linux/profile.h>
#include <linux/bitops.h>
#include <linux/list.h>
#include <linux/radix-tree.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/debugfs.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#include <linux/uaccess.h>
#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/irq.h>
#include <asm/cache.h>
#include <asm/prom.h>
#include <asm/ptrace.h>
#include <asm/machdep.h>
#include <asm/udbg.h>
#include <asm/smp.h>
#include <asm/livepatch.h>
#include <asm/asm-prototypes.h>
#include <asm/hw_irq.h>

#ifdef CONFIG_PPC64
#include <asm/paca.h>
#include <asm/firmware.h>
#include <asm/lv1call.h>
#endif
#define CREATE_TRACE_POINTS
#include <asm/trace.h>
#include <asm/cpu_has_feature.h>

DEFINE_PER_CPU_SHARED_ALIGNED(irq_cpustat_t, irq_stat);
EXPORT_PER_CPU_SYMBOL(irq_stat);

int __irq_offset_value;

#ifdef CONFIG_PPC32
EXPORT_SYMBOL(__irq_offset_value);
atomic_t ppc_n_lost_interrupts;

#ifdef CONFIG_TAU_INT
extern int tau_initialized;
extern int tau_interrupts(int);
#endif
#endif /* CONFIG_PPC32 */

#ifdef CONFIG_PPC64

int distribute_irqs = 1;

static inline notrace unsigned long get_irq_happened(void)
{
	unsigned long happened;

	__asm__ __volatile__("lbz %0,%1(13)"
	: "=r" (happened) : "i" (offsetof(struct paca_struct, irq_happened)));

	return happened;
}

static inline notrace int decrementer_check_overflow(void)
{
 	u64 now = get_tb_or_rtc();
	u64 *next_tb = this_cpu_ptr(&decrementers_next_tb);
 
	return now >= *next_tb;
}

/* This is called whenever we are re-enabling interrupts
 * and returns either 0 (nothing to do) or 500/900/280/a00/e80 if
 * there's an EE, DEC or DBELL to generate.
 *
 * This is called in two contexts: From arch_local_irq_restore()
 * before soft-enabling interrupts, and from the exception exit
 * path when returning from an interrupt from a soft-disabled to
 * a soft enabled context. In both case we have interrupts hard
 * disabled.
 *
 * We take care of only clearing the bits we handled in the
 * PACA irq_happened field since we can only re-emit one at a
 * time and we don't want to "lose" one.
 */
notrace unsigned int __check_irq_replay(void)
{
	/*
	 * We use local_paca rather than get_paca() to avoid all
	 * the debug_smp_processor_id() business in this low level
	 * function
	 */
	unsigned char happened = local_paca->irq_happened;

	/*
	 * We are responding to the next interrupt, so interrupt-off
	 * latencies should be reset here.
	 */
	trace_hardirqs_on();
	trace_hardirqs_off();

	if (happened & PACA_IRQ_HARD_DIS) {
		/* Clear bit 0 which we wouldn't clear otherwise */
		local_paca->irq_happened &= ~PACA_IRQ_HARD_DIS;

		/*
		 * We may have missed a decrementer interrupt if hard disabled.
		 * Check the decrementer register in case we had a rollover
		 * while hard disabled.
		 */
		if (!(happened & PACA_IRQ_DEC)) {
			if (decrementer_check_overflow()) {
				local_paca->irq_happened |= PACA_IRQ_DEC;
				happened |= PACA_IRQ_DEC;
			}
		}
	}

	/*
	 * Force the delivery of pending soft-disabled interrupts on PS3.
	 * Any HV call will have this side effect.
	 */
	if (firmware_has_feature(FW_FEATURE_PS3_LV1)) {
		u64 tmp, tmp2;
		lv1_get_version_info(&tmp, &tmp2);
	}

	/*
	 * Check if an hypervisor Maintenance interrupt happened.
	 * This is a higher priority interrupt than the others, so
	 * replay it first.
	 */
	if (happened & PACA_IRQ_HMI) {
		local_paca->irq_happened &= ~PACA_IRQ_HMI;
		return 0xe60;
	}

	if (happened & PACA_IRQ_DEC) {
		local_paca->irq_happened &= ~PACA_IRQ_DEC;
		return 0x900;
	}

	if (happened & PACA_IRQ_PMI) {
		local_paca->irq_happened &= ~PACA_IRQ_PMI;
		return 0xf00;
	}

	if (happened & PACA_IRQ_EE) {
		local_paca->irq_happened &= ~PACA_IRQ_EE;
		return 0x500;
	}

#ifdef CONFIG_PPC_BOOK3E
	/*
	 * Check if an EPR external interrupt happened this bit is typically
	 * set if we need to handle another "edge" interrupt from within the
	 * MPIC "EPR" handler.
	 */
	if (happened & PACA_IRQ_EE_EDGE) {
		local_paca->irq_happened &= ~PACA_IRQ_EE_EDGE;
		return 0x500;
	}

	if (happened & PACA_IRQ_DBELL) {
		local_paca->irq_happened &= ~PACA_IRQ_DBELL;
		return 0x280;
	}
#else
	if (happened & PACA_IRQ_DBELL) {
		local_paca->irq_happened &= ~PACA_IRQ_DBELL;
		return 0xa00;
	}
#endif /* CONFIG_PPC_BOOK3E */

	/* There should be nothing left ! */
	BUG_ON(local_paca->irq_happened != 0);

	return 0;
}

notrace void arch_local_irq_restore(unsigned long mask)
{
	unsigned char irq_happened;
	unsigned int replay;

	/* Write the new soft-enabled value */
	irq_soft_mask_set(mask);
	if (mask)
		return;

	/*
	 * From this point onward, we can take interrupts, preempt,
	 * etc... unless we got hard-disabled. We check if an event
	 * happened. If none happened, we know we can just return.
	 *
	 * We may have preempted before the check below, in which case
	 * we are checking the "new" CPU instead of the old one. This
	 * is only a problem if an event happened on the "old" CPU.
	 *
	 * External interrupt events will have caused interrupts to
	 * be hard-disabled, so there is no problem, we
	 * cannot have preempted.
	 */
	irq_happened = get_irq_happened();
	if (!irq_happened)
		return;

	/*
	 * We need to hard disable to get a trusted value from
	 * __check_irq_replay(). We also need to soft-disable
	 * again to avoid warnings in there due to the use of
	 * per-cpu variables.
	 *
	 * We know that if the value in irq_happened is exactly 0x01
	 * then we are already hard disabled (there are other less
	 * common cases that we'll ignore for now), so we skip the
	 * (expensive) mtmsrd.
	 */
	if (unlikely(irq_happened != PACA_IRQ_HARD_DIS))
		__hard_irq_disable();
#ifdef CONFIG_PPC_IRQ_SOFT_MASK_DEBUG
	else {
		/*
		 * We should already be hard disabled here. We had bugs
		 * where that wasn't the case so let's dbl check it and
		 * warn if we are wrong. Only do that when IRQ tracing
		 * is enabled as mfmsr() can be costly.
		 */
		if (WARN_ON(mfmsr() & MSR_EE))
			__hard_irq_disable();
	}
#endif

	irq_soft_mask_set(IRQS_ALL_DISABLED);
	trace_hardirqs_off();

	/*
	 * Check if anything needs to be re-emitted. We haven't
	 * soft-enabled yet to avoid warnings in decrementer_check_overflow
	 * accessing per-cpu variables
	 */
	replay = __check_irq_replay();

	/* We can soft-enable now */
	trace_hardirqs_on();
	irq_soft_mask_set(IRQS_ENABLED);

	/*
	 * And replay if we have to. This will return with interrupts
	 * hard-enabled.
	 */
	if (replay) {
		__replay_interrupt(replay);
		return;
	}

	/* Finally, let's ensure we are hard enabled */
	__hard_irq_enable();
}
EXPORT_SYMBOL(arch_local_irq_restore);

/*
 * This is specifically called by assembly code to re-enable interrupts
 * if they are currently disabled. This is typically called before
 * schedule() or do_signal() when returning to userspace. We do it
 * in C to avoid the burden of dealing with lockdep etc...
 *
 * NOTE: This is called with interrupts hard disabled but not marked
 * as such in paca->irq_happened, so we need to resync this.
 */
void notrace restore_interrupts(void)
{
	if (irqs_disabled()) {
		local_paca->irq_happened |= PACA_IRQ_HARD_DIS;
		local_irq_enable();
	} else
		__hard_irq_enable();
}

/*
 * This is a helper to use when about to go into idle low-power
 * when the latter has the side effect of re-enabling interrupts
 * (such as calling H_CEDE under pHyp).
 *
 * You call this function with interrupts soft-disabled (this is
 * already the case when ppc_md.power_save is called). The function
 * will return whether to enter power save or just return.
 *
 * In the former case, it will have notified lockdep of interrupts
 * being re-enabled and generally sanitized the lazy irq state,
 * and in the latter case it will leave with interrupts hard
 * disabled and marked as such, so the local_irq_enable() call
 * in arch_cpu_idle() will properly re-enable everything.
 */
bool prep_irq_for_idle(void)
{
	/*
	 * First we need to hard disable to ensure no interrupt
	 * occurs before we effectively enter the low power state
	 */
	__hard_irq_disable();
	local_paca->irq_happened |= PACA_IRQ_HARD_DIS;

	/*
	 * If anything happened while we were soft-disabled,
	 * we return now and do not enter the low power state.
	 */
	if (lazy_irq_pending())
		return false;

	/* Tell lockdep we are about to re-enable */
	trace_hardirqs_on();

	/*
	 * Mark interrupts as soft-enabled and clear the
	 * PACA_IRQ_HARD_DIS from the pending mask since we
	 * are about to hard enable as well as a side effect
	 * of entering the low power state.
	 */
	local_paca->irq_happened &= ~PACA_IRQ_HARD_DIS;
	irq_soft_mask_set(IRQS_ENABLED);

	/* Tell the caller to enter the low power state */
	return true;
}

#ifdef CONFIG_PPC_BOOK3S
/*
 * This is for idle sequences that return with IRQs off, but the
 * idle state itself wakes on interrupt. Tell the irq tracer that
 * IRQs are enabled for the duration of idle so it does not get long
 * off times. Must be paired with fini_irq_for_idle_irqsoff.
 */
bool prep_irq_for_idle_irqsoff(void)
{
	WARN_ON(!irqs_disabled());

	/*
	 * First we need to hard disable to ensure no interrupt
	 * occurs before we effectively enter the low power state
	 */
	__hard_irq_disable();
	local_paca->irq_happened |= PACA_IRQ_HARD_DIS;

	/*
	 * If anything happened while we were soft-disabled,
	 * we return now and do not enter the low power state.
	 */
	if (lazy_irq_pending())
		return false;

	/* Tell lockdep we are about to re-enable */
	trace_hardirqs_on();

	return true;
}

/*
 * Take the SRR1 wakeup reason, index into this table to find the
 * appropriate irq_happened bit.
 *
 * Sytem reset exceptions taken in idle state also come through here,
 * but they are NMI interrupts so do not need to wait for IRQs to be
 * restored, and should be taken as early as practical. These are marked
 * with 0xff in the table. The Power ISA specifies 0100b as the system
 * reset interrupt reason.
 */
#define IRQ_SYSTEM_RESET	0xff

static const u8 srr1_to_lazyirq[0x10] = {
	0, 0, 0,
	PACA_IRQ_DBELL,
	IRQ_SYSTEM_RESET,
	PACA_IRQ_DBELL,
	PACA_IRQ_DEC,
	0,
	PACA_IRQ_EE,
	PACA_IRQ_EE,
	PACA_IRQ_HMI,
	0, 0, 0, 0, 0 };

void replay_system_reset(void)
{
	struct pt_regs regs;

	ppc_save_regs(&regs);
	regs.trap = 0x100;
	get_paca()->in_nmi = 1;
	system_reset_exception(&regs);
	get_paca()->in_nmi = 0;
}
EXPORT_SYMBOL_GPL(replay_system_reset);

void irq_set_pending_from_srr1(unsigned long srr1)
{
	unsigned int idx = (srr1 & SRR1_WAKEMASK_P8) >> 18;
	u8 reason = srr1_to_lazyirq[idx];

	/*
	 * Take the system reset now, which is immediately after registers
	 * are restored from idle. It's an NMI, so interrupts need not be
	 * re-enabled before it is taken.
	 */
	if (unlikely(reason == IRQ_SYSTEM_RESET)) {
		replay_system_reset();
		return;
	}

	/*
	 * The 0 index (SRR1[42:45]=b0000) must always evaluate to 0,
	 * so this can be called unconditionally with the SRR1 wake
	 * reason as returned by the idle code, which uses 0 to mean no
	 * interrupt.
	 *
	 * If a future CPU was to designate this as an interrupt reason,
	 * then a new index for no interrupt must be assigned.
	 */
	local_paca->irq_happened |= reason;
}
#endif /* CONFIG_PPC_BOOK3S */

/*
 * Force a replay of the external interrupt handler on this CPU.
 */
void force_external_irq_replay(void)
{
	/*
	 * This must only be called with interrupts soft-disabled,
	 * the replay will happen when re-enabling.
	 */
	WARN_ON(!arch_irqs_disabled());

	/* Indicate in the PACA that we have an interrupt to replay */
	local_paca->irq_happened |= PACA_IRQ_EE;
}

#endif /* CONFIG_PPC64 */

int arch_show_interrupts(struct seq_file *p, int prec)
{
	int j;

#if defined(CONFIG_PPC32) && defined(CONFIG_TAU_INT)
	if (tau_initialized) {
		seq_printf(p, "%*s: ", prec, "TAU");
		for_each_online_cpu(j)
			seq_printf(p, "%10u ", tau_interrupts(j));
		seq_puts(p, "  PowerPC             Thermal Assist (cpu temp)\n");
	}
#endif /* CONFIG_PPC32 && CONFIG_TAU_INT */

	seq_printf(p, "%*s: ", prec, "LOC");
	for_each_online_cpu(j)
		seq_printf(p, "%10u ", per_cpu(irq_stat, j).timer_irqs_event);
        seq_printf(p, "  Local timer interrupts for timer event device\n");

	seq_printf(p, "%*s: ", prec, "LOC");
	for_each_online_cpu(j)
		seq_printf(p, "%10u ", per_cpu(irq_stat, j).timer_irqs_others);
        seq_printf(p, "  Local timer interrupts for others\n");

	seq_printf(p, "%*s: ", prec, "SPU");
	for_each_online_cpu(j)
		seq_printf(p, "%10u ", per_cpu(irq_stat, j).spurious_irqs);
	seq_printf(p, "  Spurious interrupts\n");

	seq_printf(p, "%*s: ", prec, "PMI");
	for_each_online_cpu(j)
		seq_printf(p, "%10u ", per_cpu(irq_stat, j).pmu_irqs);
	seq_printf(p, "  Performance monitoring interrupts\n");

	seq_printf(p, "%*s: ", prec, "MCE");
	for_each_online_cpu(j)
		seq_printf(p, "%10u ", per_cpu(irq_stat, j).mce_exceptions);
	seq_printf(p, "  Machine check exceptions\n");

	if (cpu_has_feature(CPU_FTR_HVMODE)) {
		seq_printf(p, "%*s: ", prec, "HMI");
		for_each_online_cpu(j)
			seq_printf(p, "%10u ",
					per_cpu(irq_stat, j).hmi_exceptions);
		seq_printf(p, "  Hypervisor Maintenance Interrupts\n");
	}

	seq_printf(p, "%*s: ", prec, "NMI");
	for_each_online_cpu(j)
		seq_printf(p, "%10u ", per_cpu(irq_stat, j).sreset_irqs);
	seq_printf(p, "  System Reset interrupts\n");

#ifdef CONFIG_PPC_WATCHDOG
	seq_printf(p, "%*s: ", prec, "WDG");
	for_each_online_cpu(j)
		seq_printf(p, "%10u ", per_cpu(irq_stat, j).soft_nmi_irqs);
	seq_printf(p, "  Watchdog soft-NMI interrupts\n");
#endif

#ifdef CONFIG_PPC_DOORBELL
	if (cpu_has_feature(CPU_FTR_DBELL)) {
		seq_printf(p, "%*s: ", prec, "DBL");
		for_each_online_cpu(j)
			seq_printf(p, "%10u ", per_cpu(irq_stat, j).doorbell_irqs);
		seq_printf(p, "  Doorbell interrupts\n");
	}
#endif

	return 0;
}

/*
 * /proc/stat helpers
 */
u64 arch_irq_stat_cpu(unsigned int cpu)
{
	u64 sum = per_cpu(irq_stat, cpu).timer_irqs_event;

	sum += per_cpu(irq_stat, cpu).pmu_irqs;
	sum += per_cpu(irq_stat, cpu).mce_exceptions;
	sum += per_cpu(irq_stat, cpu).spurious_irqs;
	sum += per_cpu(irq_stat, cpu).timer_irqs_others;
	sum += per_cpu(irq_stat, cpu).hmi_exceptions;
	sum += per_cpu(irq_stat, cpu).sreset_irqs;
#ifdef CONFIG_PPC_WATCHDOG
	sum += per_cpu(irq_stat, cpu).soft_nmi_irqs;
#endif
#ifdef CONFIG_PPC_DOORBELL
	sum += per_cpu(irq_stat, cpu).doorbell_irqs;
#endif

	return sum;
}

static inline void check_stack_overflow(void)
{
#ifdef CONFIG_DEBUG_STACKOVERFLOW
	long sp;

	sp = current_stack_pointer() & (THREAD_SIZE-1);

	/* check for stack overflow: is there less than 2KB free? */
	if (unlikely(sp < (sizeof(struct thread_info) + 2048))) {
		pr_err("do_IRQ: stack overflow: %ld\n",
			sp - sizeof(struct thread_info));
		dump_stack();
	}
#endif
}

void __do_irq(struct pt_regs *regs)
{
	unsigned int irq;

	irq_enter();

	trace_irq_entry(regs);

	check_stack_overflow();

	/*
	 * Query the platform PIC for the interrupt & ack it.
	 *
	 * This will typically lower the interrupt line to the CPU
	 */
	irq = ppc_md.get_irq();

	/* We can hard enable interrupts now to allow perf interrupts */
	may_hard_irq_enable();

	/* And finally process it */
	if (unlikely(!irq))
		__this_cpu_inc(irq_stat.spurious_irqs);
	else
		generic_handle_irq(irq);

	trace_irq_exit(regs);

	irq_exit();
}

void do_IRQ(struct pt_regs *regs)
{
	struct pt_regs *old_regs = set_irq_regs(regs);
	struct thread_info *curtp, *irqtp, *sirqtp;

	/* Switch to the irq stack to handle this */
	curtp = current_thread_info();
	irqtp = hardirq_ctx[raw_smp_processor_id()];
	sirqtp = softirq_ctx[raw_smp_processor_id()];

	/* Already there ? */
	if (unlikely(curtp == irqtp || curtp == sirqtp)) {
		__do_irq(regs);
		set_irq_regs(old_regs);
		return;
	}

	/* Prepare the thread_info in the irq stack */
	irqtp->task = curtp->task;
	irqtp->flags = 0;

	/* Copy the preempt_count so that the [soft]irq checks work. */
	irqtp->preempt_count = curtp->preempt_count;

	/* Switch stack and call */
	call_do_irq(regs, irqtp);

	/* Restore stack limit */
	irqtp->task = NULL;

	/* Copy back updates to the thread_info */
	if (irqtp->flags)
		set_bits(irqtp->flags, &curtp->flags);

	set_irq_regs(old_regs);
}

void __init init_IRQ(void)
{
	if (ppc_md.init_IRQ)
		ppc_md.init_IRQ();

	exc_lvl_ctx_init();

	irq_ctx_init();
}

#if defined(CONFIG_BOOKE) || defined(CONFIG_40x)
struct thread_info   *critirq_ctx[NR_CPUS] __read_mostly;
struct thread_info    *dbgirq_ctx[NR_CPUS] __read_mostly;
struct thread_info *mcheckirq_ctx[NR_CPUS] __read_mostly;

void exc_lvl_ctx_init(void)
{
	struct thread_info *tp;
	int i, cpu_nr;

	for_each_possible_cpu(i) {
#ifdef CONFIG_PPC64
		cpu_nr = i;
#else
#ifdef CONFIG_SMP
		cpu_nr = get_hard_smp_processor_id(i);
#else
		cpu_nr = 0;
#endif
#endif

		memset((void *)critirq_ctx[cpu_nr], 0, THREAD_SIZE);
		tp = critirq_ctx[cpu_nr];
		tp->cpu = cpu_nr;
		tp->preempt_count = 0;

#ifdef CONFIG_BOOKE
		memset((void *)dbgirq_ctx[cpu_nr], 0, THREAD_SIZE);
		tp = dbgirq_ctx[cpu_nr];
		tp->cpu = cpu_nr;
		tp->preempt_count = 0;

		memset((void *)mcheckirq_ctx[cpu_nr], 0, THREAD_SIZE);
		tp = mcheckirq_ctx[cpu_nr];
		tp->cpu = cpu_nr;
		tp->preempt_count = HARDIRQ_OFFSET;
#endif
	}
}
#endif

struct thread_info *softirq_ctx[NR_CPUS] __read_mostly;
struct thread_info *hardirq_ctx[NR_CPUS] __read_mostly;

void irq_ctx_init(void)
{
	struct thread_info *tp;
	int i;

	for_each_possible_cpu(i) {
		memset((void *)softirq_ctx[i], 0, THREAD_SIZE);
		tp = softirq_ctx[i];
		tp->cpu = i;
		klp_init_thread_info(tp);

		memset((void *)hardirq_ctx[i], 0, THREAD_SIZE);
		tp = hardirq_ctx[i];
		tp->cpu = i;
		klp_init_thread_info(tp);
	}
}

void do_softirq_own_stack(void)
{
	struct thread_info *curtp, *irqtp;

	curtp = current_thread_info();
	irqtp = softirq_ctx[smp_processor_id()];
	irqtp->task = curtp->task;
	irqtp->flags = 0;
	call_do_softirq(irqtp);
	irqtp->task = NULL;

	/* Set any flag that may have been set on the
	 * alternate stack
	 */
	if (irqtp->flags)
		set_bits(irqtp->flags, &curtp->flags);
}

irq_hw_number_t virq_to_hw(unsigned int virq)
{
	struct irq_data *irq_data = irq_get_irq_data(virq);
	return WARN_ON(!irq_data) ? 0 : irq_data->hwirq;
}
EXPORT_SYMBOL_GPL(virq_to_hw);

#ifdef CONFIG_SMP
int irq_choose_cpu(const struct cpumask *mask)
{
	int cpuid;

	if (cpumask_equal(mask, cpu_online_mask)) {
		static int irq_rover;
		static DEFINE_RAW_SPINLOCK(irq_rover_lock);
		unsigned long flags;

		/* Round-robin distribution... */
do_round_robin:
		raw_spin_lock_irqsave(&irq_rover_lock, flags);

		irq_rover = cpumask_next(irq_rover, cpu_online_mask);
		if (irq_rover >= nr_cpu_ids)
			irq_rover = cpumask_first(cpu_online_mask);

		cpuid = irq_rover;

		raw_spin_unlock_irqrestore(&irq_rover_lock, flags);
	} else {
		cpuid = cpumask_first_and(mask, cpu_online_mask);
		if (cpuid >= nr_cpu_ids)
			goto do_round_robin;
	}

	return get_hard_smp_processor_id(cpuid);
}
#else
int irq_choose_cpu(const struct cpumask *mask)
{
	return hard_smp_processor_id();
}
#endif

int arch_early_irq_init(void)
{
	return 0;
}

#ifdef CONFIG_PPC64
static int __init setup_noirqdistrib(char *str)
{
	distribute_irqs = 0;
	return 1;
}

__setup("noirqdistrib", setup_noirqdistrib);
#endif /* CONFIG_PPC64 */
