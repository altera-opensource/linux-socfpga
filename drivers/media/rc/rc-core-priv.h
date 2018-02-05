/*
 * Remote Controller core raw events header
 *
 * Copyright (C) 2010 by Mauro Carvalho Chehab
 *
 * This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#ifndef _RC_CORE_PRIV
#define _RC_CORE_PRIV

/* Define the max number of pulse/space transitions to buffer */
#define	MAX_IR_EVENT_SIZE	512

#include <linux/slab.h>
#include <media/rc-core.h>

struct ir_raw_handler {
	struct list_head list;

	u64 protocols; /* which are handled by this handler */
	int (*decode)(struct rc_dev *dev, struct ir_raw_event event);
	int (*encode)(enum rc_proto protocol, u32 scancode,
		      struct ir_raw_event *events, unsigned int max);

	/* These two should only be used by the lirc decoder */
	int (*raw_register)(struct rc_dev *dev);
	int (*raw_unregister)(struct rc_dev *dev);
};

struct ir_raw_event_ctrl {
	struct list_head		list;		/* to keep track of raw clients */
	struct task_struct		*thread;
	/* fifo for the pulse/space durations */
	DECLARE_KFIFO(kfifo, struct ir_raw_event, MAX_IR_EVENT_SIZE);
	ktime_t				last_event;	/* when last event occurred */
	struct rc_dev			*dev;		/* pointer to the parent rc_dev */
	/* edge driver */
	struct timer_list edge_handle;

	/* raw decoder state follows */
	struct ir_raw_event prev_ev;
	struct ir_raw_event this_ev;
	struct nec_dec {
		int state;
		unsigned count;
		u32 bits;
		bool is_nec_x;
		bool necx_repeat;
	} nec;
	struct rc5_dec {
		int state;
		u32 bits;
		unsigned count;
		bool is_rc5x;
	} rc5;
	struct rc6_dec {
		int state;
		u8 header;
		u32 body;
		bool toggle;
		unsigned count;
		unsigned wanted_bits;
	} rc6;
	struct sony_dec {
		int state;
		u32 bits;
		unsigned count;
	} sony;
	struct jvc_dec {
		int state;
		u16 bits;
		u16 old_bits;
		unsigned count;
		bool first;
		bool toggle;
	} jvc;
	struct sanyo_dec {
		int state;
		unsigned count;
		u64 bits;
	} sanyo;
	struct sharp_dec {
		int state;
		unsigned count;
		u32 bits;
		unsigned int pulse_len;
	} sharp;
	struct mce_kbd_dec {
		struct input_dev *idev;
		struct timer_list rx_timeout;
		char name[64];
		char phys[64];
		int state;
		u8 header;
		u32 body;
		unsigned count;
		unsigned wanted_bits;
	} mce_kbd;
	struct lirc_codec {
		struct rc_dev *dev;
		struct lirc_dev *ldev;
		int carrier_low;

		ktime_t gap_start;
		u64 gap_duration;
		bool gap;
		bool send_timeout_reports;

	} lirc;
	struct xmp_dec {
		int state;
		unsigned count;
		u32 durations[16];
	} xmp;
};

/* macros for IR decoders */
static inline bool geq_margin(unsigned d1, unsigned d2, unsigned margin)
{
	return d1 > (d2 - margin);
}

static inline bool eq_margin(unsigned d1, unsigned d2, unsigned margin)
{
	return ((d1 > (d2 - margin)) && (d1 < (d2 + margin)));
}

static inline bool is_transition(struct ir_raw_event *x, struct ir_raw_event *y)
{
	return x->pulse != y->pulse;
}

static inline void decrease_duration(struct ir_raw_event *ev, unsigned duration)
{
	if (duration > ev->duration)
		ev->duration = 0;
	else
		ev->duration -= duration;
}

/* Returns true if event is normal pulse/space event */
static inline bool is_timing_event(struct ir_raw_event ev)
{
	return !ev.carrier_report && !ev.reset;
}

#define TO_US(duration)			DIV_ROUND_CLOSEST((duration), 1000)
#define TO_STR(is_pulse)		((is_pulse) ? "pulse" : "space")

/* functions for IR encoders */

static inline void init_ir_raw_event_duration(struct ir_raw_event *ev,
					      unsigned int pulse,
					      u32 duration)
{
	init_ir_raw_event(ev);
	ev->duration = duration;
	ev->pulse = pulse;
}

/**
 * struct ir_raw_timings_manchester - Manchester coding timings
 * @leader:		duration of leader pulse (if any) 0 if continuing
 *			existing signal (see @pulse_space_start)
 * @pulse_space_start:	1 for starting with pulse (0 for starting with space)
 * @clock:		duration of each pulse/space in ns
 * @invert:		if set clock logic is inverted
 *			(0 = space + pulse, 1 = pulse + space)
 * @trailer_space:	duration of trailer space in ns
 */
struct ir_raw_timings_manchester {
	unsigned int leader;
	unsigned int pulse_space_start:1;
	unsigned int clock;
	unsigned int invert:1;
	unsigned int trailer_space;
};

int ir_raw_gen_manchester(struct ir_raw_event **ev, unsigned int max,
			  const struct ir_raw_timings_manchester *timings,
			  unsigned int n, u64 data);

/**
 * ir_raw_gen_pulse_space() - generate pulse and space raw events.
 * @ev:			Pointer to pointer to next free raw event.
 *			Will be incremented for each raw event written.
 * @max:		Pointer to number of raw events available in buffer.
 *			Will be decremented for each raw event written.
 * @pulse_width:	Width of pulse in ns.
 * @space_width:	Width of space in ns.
 *
 * Returns:	0 on success.
 *		-ENOBUFS if there isn't enough buffer space to write both raw
 *		events. In this case @max events will have been written.
 */
static inline int ir_raw_gen_pulse_space(struct ir_raw_event **ev,
					 unsigned int *max,
					 unsigned int pulse_width,
					 unsigned int space_width)
{
	if (!*max)
		return -ENOBUFS;
	init_ir_raw_event_duration((*ev)++, 1, pulse_width);
	if (!--*max)
		return -ENOBUFS;
	init_ir_raw_event_duration((*ev)++, 0, space_width);
	--*max;
	return 0;
}

/**
 * struct ir_raw_timings_pd - pulse-distance modulation timings
 * @header_pulse:	duration of header pulse in ns (0 for none)
 * @header_space:	duration of header space in ns
 * @bit_pulse:		duration of bit pulse in ns
 * @bit_space:		duration of bit space (for logic 0 and 1) in ns
 * @trailer_pulse:	duration of trailer pulse in ns
 * @trailer_space:	duration of trailer space in ns
 * @msb_first:		1 if most significant bit is sent first
 */
struct ir_raw_timings_pd {
	unsigned int header_pulse;
	unsigned int header_space;
	unsigned int bit_pulse;
	unsigned int bit_space[2];
	unsigned int trailer_pulse;
	unsigned int trailer_space;
	unsigned int msb_first:1;
};

int ir_raw_gen_pd(struct ir_raw_event **ev, unsigned int max,
		  const struct ir_raw_timings_pd *timings,
		  unsigned int n, u64 data);

/**
 * struct ir_raw_timings_pl - pulse-length modulation timings
 * @header_pulse:	duration of header pulse in ns (0 for none)
 * @bit_space:		duration of bit space in ns
 * @bit_pulse:		duration of bit pulse (for logic 0 and 1) in ns
 * @trailer_space:	duration of trailer space in ns
 * @msb_first:		1 if most significant bit is sent first
 */
struct ir_raw_timings_pl {
	unsigned int header_pulse;
	unsigned int bit_space;
	unsigned int bit_pulse[2];
	unsigned int trailer_space;
	unsigned int msb_first:1;
};

int ir_raw_gen_pl(struct ir_raw_event **ev, unsigned int max,
		  const struct ir_raw_timings_pl *timings,
		  unsigned int n, u64 data);

/*
 * Routines from rc-raw.c to be used internally and by decoders
 */
u64 ir_raw_get_allowed_protocols(void);
int ir_raw_event_prepare(struct rc_dev *dev);
int ir_raw_event_register(struct rc_dev *dev);
void ir_raw_event_free(struct rc_dev *dev);
void ir_raw_event_unregister(struct rc_dev *dev);
int ir_raw_handler_register(struct ir_raw_handler *ir_raw_handler);
void ir_raw_handler_unregister(struct ir_raw_handler *ir_raw_handler);
void ir_raw_init(void);

/*
 * Decoder initialization code
 *
 * Those load logic are called during ir-core init, and automatically
 * loads the compiled decoders for their usage with IR raw events
 */

#endif /* _RC_CORE_PRIV */
