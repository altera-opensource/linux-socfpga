/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_SCHED_NOHZ_H
#define _LINUX_SCHED_NOHZ_H

/*
 * This is the interface between the scheduler and nohz/dynticks:
 */

#if defined(CONFIG_SMP) && defined(CONFIG_NO_HZ_COMMON)
extern void cpu_load_update_nohz_start(void);
extern void cpu_load_update_nohz_stop(void);
#else
static inline void cpu_load_update_nohz_start(void) { }
static inline void cpu_load_update_nohz_stop(void) { }
#endif

#if defined(CONFIG_SMP) && defined(CONFIG_NO_HZ_COMMON)
extern void nohz_balance_enter_idle(int cpu);
extern void set_cpu_sd_state_idle(void);
extern int get_nohz_timer_target(void);
#else
static inline void nohz_balance_enter_idle(int cpu) { }
static inline void set_cpu_sd_state_idle(void) { }
#endif

#ifdef CONFIG_NO_HZ_COMMON
void calc_load_nohz_start(void);
void calc_load_nohz_stop(void);
#else
static inline void calc_load_nohz_start(void) { }
static inline void calc_load_nohz_stop(void) { }
#endif /* CONFIG_NO_HZ_COMMON */

#if defined(CONFIG_NO_HZ_COMMON) && defined(CONFIG_SMP)
extern void wake_up_nohz_cpu(int cpu);
#else
static inline void wake_up_nohz_cpu(int cpu) { }
#endif

#ifdef CONFIG_NO_HZ_FULL
extern u64 scheduler_tick_max_deferment(void);
#endif

#endif /* _LINUX_SCHED_NOHZ_H */
