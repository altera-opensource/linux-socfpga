/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Copyright (C) 2012 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef _UAPI__ASM_HWCAP_H
#define _UAPI__ASM_HWCAP_H

/*
 * HWCAP flags - for elf_hwcap (in kernel) and AT_HWCAP
 */
#define HWCAP_FP		(1 << 0)
#define HWCAP_ASIMD		(1 << 1)
#define HWCAP_EVTSTRM		(1 << 2)
#define HWCAP_AES		(1 << 3)
#define HWCAP_PMULL		(1 << 4)
#define HWCAP_SHA1		(1 << 5)
#define HWCAP_SHA2		(1 << 6)
#define HWCAP_CRC32		(1 << 7)
#define HWCAP_ATOMICS		(1 << 8)
#define HWCAP_FPHP		(1 << 9)
#define HWCAP_ASIMDHP		(1 << 10)
#define HWCAP_CPUID		(1 << 11)
#define HWCAP_ASIMDRDM		(1 << 12)
#define HWCAP_JSCVT		(1 << 13)
#define HWCAP_FCMA		(1 << 14)
#define HWCAP_LRCPC		(1 << 15)
#define HWCAP_DCPOP		(1 << 16)
#define HWCAP_SHA3		(1 << 17)
#define HWCAP_SM3		(1 << 18)
#define HWCAP_SM4		(1 << 19)
#define HWCAP_ASIMDDP		(1 << 20)
#define HWCAP_SHA512		(1 << 21)
#define HWCAP_SVE		(1 << 22)
#define HWCAP_ASIMDFHM		(1 << 23)

#endif /* _UAPI__ASM_HWCAP_H */
