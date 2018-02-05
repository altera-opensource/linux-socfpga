/*
 * Copyright (C) 2009 Chen Liqin <liqin.chen@sunplusct.com>
 * Copyright (C) 2012 Regents of the University of California
 *
 *   This program is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License
 *   as published by the Free Software Foundation, version 2.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 */

#ifndef _ASM_RISCV_PGALLOC_H
#define _ASM_RISCV_PGALLOC_H

#include <linux/mm.h>
#include <asm/tlb.h>

static inline void pmd_populate_kernel(struct mm_struct *mm,
	pmd_t *pmd, pte_t *pte)
{
	unsigned long pfn = virt_to_pfn(pte);

	set_pmd(pmd, __pmd((pfn << _PAGE_PFN_SHIFT) | _PAGE_TABLE));
}

static inline void pmd_populate(struct mm_struct *mm,
	pmd_t *pmd, pgtable_t pte)
{
	unsigned long pfn = virt_to_pfn(page_address(pte));

	set_pmd(pmd, __pmd((pfn << _PAGE_PFN_SHIFT) | _PAGE_TABLE));
}

#ifndef __PAGETABLE_PMD_FOLDED
static inline void pud_populate(struct mm_struct *mm, pud_t *pud, pmd_t *pmd)
{
	unsigned long pfn = virt_to_pfn(pmd);

	set_pud(pud, __pud((pfn << _PAGE_PFN_SHIFT) | _PAGE_TABLE));
}
#endif /* __PAGETABLE_PMD_FOLDED */

#define pmd_pgtable(pmd)	pmd_page(pmd)

static inline pgd_t *pgd_alloc(struct mm_struct *mm)
{
	pgd_t *pgd;

	pgd = (pgd_t *)__get_free_page(GFP_KERNEL);
	if (likely(pgd != NULL)) {
		memset(pgd, 0, USER_PTRS_PER_PGD * sizeof(pgd_t));
		/* Copy kernel mappings */
		memcpy(pgd + USER_PTRS_PER_PGD,
			init_mm.pgd + USER_PTRS_PER_PGD,
			(PTRS_PER_PGD - USER_PTRS_PER_PGD) * sizeof(pgd_t));
	}
	return pgd;
}

static inline void pgd_free(struct mm_struct *mm, pgd_t *pgd)
{
	free_page((unsigned long)pgd);
}

#ifndef __PAGETABLE_PMD_FOLDED

static inline pmd_t *pmd_alloc_one(struct mm_struct *mm, unsigned long addr)
{
	return (pmd_t *)__get_free_page(
		GFP_KERNEL | __GFP_RETRY_MAYFAIL | __GFP_ZERO);
}

static inline void pmd_free(struct mm_struct *mm, pmd_t *pmd)
{
	free_page((unsigned long)pmd);
}

#define __pmd_free_tlb(tlb, pmd, addr)  pmd_free((tlb)->mm, pmd)

#endif /* __PAGETABLE_PMD_FOLDED */

static inline pte_t *pte_alloc_one_kernel(struct mm_struct *mm,
	unsigned long address)
{
	return (pte_t *)__get_free_page(
		GFP_KERNEL | __GFP_RETRY_MAYFAIL | __GFP_ZERO);
}

static inline struct page *pte_alloc_one(struct mm_struct *mm,
	unsigned long address)
{
	struct page *pte;

	pte = alloc_page(GFP_KERNEL | __GFP_RETRY_MAYFAIL | __GFP_ZERO);
	if (likely(pte != NULL))
		pgtable_page_ctor(pte);
	return pte;
}

static inline void pte_free_kernel(struct mm_struct *mm, pte_t *pte)
{
	free_page((unsigned long)pte);
}

static inline void pte_free(struct mm_struct *mm, pgtable_t pte)
{
	pgtable_page_dtor(pte);
	__free_page(pte);
}

#define __pte_free_tlb(tlb, pte, buf)   \
do {                                    \
	pgtable_page_dtor(pte);         \
	tlb_remove_page((tlb), pte);    \
} while (0)

static inline void check_pgt_cache(void)
{
}

#endif /* _ASM_RISCV_PGALLOC_H */
