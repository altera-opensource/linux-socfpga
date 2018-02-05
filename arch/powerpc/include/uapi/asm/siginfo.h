/* SPDX-License-Identifier: GPL-2.0+ WITH Linux-syscall-note */
#ifndef _ASM_POWERPC_SIGINFO_H
#define _ASM_POWERPC_SIGINFO_H

/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#ifdef __powerpc64__
#    define __ARCH_SI_PREAMBLE_SIZE	(4 * sizeof(int))
#endif

#include <asm-generic/siginfo.h>

/*
 * SIGFPE si_codes
 */
#ifdef __KERNEL__
#define FPE_FIXME	0	/* Broken dup of SI_USER */
#endif /* __KERNEL__ */

/*
 * SIGTRAP si_codes
 */
#ifdef __KERNEL__
#define TRAP_FIXME	0	/* Broken dup of SI_USER */
#endif /* __KERNEL__ */


#endif	/* _ASM_POWERPC_SIGINFO_H */
