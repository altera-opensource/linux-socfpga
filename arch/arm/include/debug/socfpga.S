/*
 *  Copyright (C) 1994-1999 Russell King
 *  Moved from linux/arch/arm/kernel/debug.S by Ben Dooks
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define UART_SHIFT 2
		.macro	addruart, rp, rv, tmp
		mov	\rp, #CONFIG_SOCFPGA_DEBUG_LL_UART_OFFSET
		orr	\rp, \rp, #0x00c00000
		orr	\rv, \rp, #0xfe000000	@ virtual base
		orr	\rp, \rp, #0xff000000	@ physical base
		.endm

#include "8250_32.S"

