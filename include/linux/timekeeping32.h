#ifndef _LINUX_TIMEKEEPING32_H
#define _LINUX_TIMEKEEPING32_H
/*
 * These interfaces are all based on the old timespec type
 * and should get replaced with the timespec64 based versions
 * over time so we can remove the file here.
 */

extern void do_gettimeofday(struct timeval *tv);
unsigned long get_seconds(void);

/* does not take xtime_lock */
struct timespec __current_kernel_time(void);

static inline struct timespec current_kernel_time(void)
{
	struct timespec64 now = current_kernel_time64();

	return timespec64_to_timespec(now);
}

#if BITS_PER_LONG == 64
/**
 * Deprecated. Use do_settimeofday64().
 */
static inline int do_settimeofday(const struct timespec *ts)
{
	return do_settimeofday64(ts);
}

static inline int __getnstimeofday(struct timespec *ts)
{
	return __getnstimeofday64(ts);
}

static inline void getnstimeofday(struct timespec *ts)
{
	getnstimeofday64(ts);
}

static inline void ktime_get_ts(struct timespec *ts)
{
	ktime_get_ts64(ts);
}

static inline void ktime_get_real_ts(struct timespec *ts)
{
	getnstimeofday64(ts);
}

static inline void getrawmonotonic(struct timespec *ts)
{
	getrawmonotonic64(ts);
}

static inline struct timespec get_monotonic_coarse(void)
{
	return get_monotonic_coarse64();
}

static inline void getboottime(struct timespec *ts)
{
	return getboottime64(ts);
}
#else
/**
 * Deprecated. Use do_settimeofday64().
 */
static inline int do_settimeofday(const struct timespec *ts)
{
	struct timespec64 ts64;

	ts64 = timespec_to_timespec64(*ts);
	return do_settimeofday64(&ts64);
}

static inline int __getnstimeofday(struct timespec *ts)
{
	struct timespec64 ts64;
	int ret = __getnstimeofday64(&ts64);

	*ts = timespec64_to_timespec(ts64);
	return ret;
}

static inline void getnstimeofday(struct timespec *ts)
{
	struct timespec64 ts64;

	getnstimeofday64(&ts64);
	*ts = timespec64_to_timespec(ts64);
}

static inline void ktime_get_ts(struct timespec *ts)
{
	struct timespec64 ts64;

	ktime_get_ts64(&ts64);
	*ts = timespec64_to_timespec(ts64);
}

static inline void ktime_get_real_ts(struct timespec *ts)
{
	struct timespec64 ts64;

	getnstimeofday64(&ts64);
	*ts = timespec64_to_timespec(ts64);
}

static inline void getrawmonotonic(struct timespec *ts)
{
	struct timespec64 ts64;

	getrawmonotonic64(&ts64);
	*ts = timespec64_to_timespec(ts64);
}

static inline struct timespec get_monotonic_coarse(void)
{
	return timespec64_to_timespec(get_monotonic_coarse64());
}

static inline void getboottime(struct timespec *ts)
{
	struct timespec64 ts64;

	getboottime64(&ts64);
	*ts = timespec64_to_timespec(ts64);
}
#endif

/*
 * Timespec interfaces utilizing the ktime based ones
 */
static inline void get_monotonic_boottime(struct timespec *ts)
{
	*ts = ktime_to_timespec(ktime_get_boottime());
}

static inline void timekeeping_clocktai(struct timespec *ts)
{
	*ts = ktime_to_timespec(ktime_get_clocktai());
}

/*
 * Persistent clock related interfaces
 */
extern void read_persistent_clock(struct timespec *ts);
extern int update_persistent_clock(struct timespec now);

#endif
