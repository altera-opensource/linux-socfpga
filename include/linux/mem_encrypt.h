/*
 * AMD Memory Encryption Support
 *
 * Copyright (C) 2016 Advanced Micro Devices, Inc.
 *
 * Author: Tom Lendacky <thomas.lendacky@amd.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MEM_ENCRYPT_H__
#define __MEM_ENCRYPT_H__

#ifndef __ASSEMBLY__

#ifdef CONFIG_ARCH_HAS_MEM_ENCRYPT

#include <asm/mem_encrypt.h>

#else	/* !CONFIG_ARCH_HAS_MEM_ENCRYPT */

#define sme_me_mask	0ULL

static inline bool sme_active(void) { return false; }
static inline bool sev_active(void) { return false; }

#endif	/* CONFIG_ARCH_HAS_MEM_ENCRYPT */

static inline bool mem_encrypt_active(void)
{
	return sme_me_mask;
}

static inline u64 sme_get_me_mask(void)
{
	return sme_me_mask;
}

#ifdef CONFIG_AMD_MEM_ENCRYPT
/*
 * The __sme_set() and __sme_clr() macros are useful for adding or removing
 * the encryption mask from a value (e.g. when dealing with pagetable
 * entries).
 */
#define __sme_set(x)		((x) | sme_me_mask)
#define __sme_clr(x)		((x) & ~sme_me_mask)
#else
#define __sme_set(x)		(x)
#define __sme_clr(x)		(x)
#endif

#endif	/* __ASSEMBLY__ */

#endif	/* __MEM_ENCRYPT_H__ */
