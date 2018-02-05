/*
 * Debug helper to dump the current kernel pagetables of the system
 * so that we can see what the various memory ranges are set to.
 *
 * (C) Copyright 2008 Intel Corporation
 *
 * Author: Arjan van de Ven <arjan@linux.intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/debugfs.h>
#include <linux/kasan.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/seq_file.h>

#include <asm/pgtable.h>

/*
 * The dumper groups pagetable entries of the same type into one, and for
 * that it needs to keep some state when walking, and flush this state
 * when a "break" in the continuity is found.
 */
struct pg_state {
	int level;
	pgprot_t current_prot;
	unsigned long start_address;
	unsigned long current_address;
	const struct addr_marker *marker;
	unsigned long lines;
	bool to_dmesg;
	bool check_wx;
	unsigned long wx_pages;
};

struct addr_marker {
	unsigned long start_address;
	const char *name;
	unsigned long max_lines;
};

/* Address space markers hints */

#ifdef CONFIG_X86_64

enum address_markers_idx {
	USER_SPACE_NR = 0,
	KERNEL_SPACE_NR,
	LOW_KERNEL_NR,
#if defined(CONFIG_MODIFY_LDT_SYSCALL) && defined(CONFIG_X86_5LEVEL)
	LDT_NR,
#endif
	VMALLOC_START_NR,
	VMEMMAP_START_NR,
#ifdef CONFIG_KASAN
	KASAN_SHADOW_START_NR,
	KASAN_SHADOW_END_NR,
#endif
	CPU_ENTRY_AREA_NR,
#if defined(CONFIG_MODIFY_LDT_SYSCALL) && !defined(CONFIG_X86_5LEVEL)
	LDT_NR,
#endif
#ifdef CONFIG_X86_ESPFIX64
	ESPFIX_START_NR,
#endif
#ifdef CONFIG_EFI
	EFI_END_NR,
#endif
	HIGH_KERNEL_NR,
	MODULES_VADDR_NR,
	MODULES_END_NR,
	FIXADDR_START_NR,
	END_OF_SPACE_NR,
};

static struct addr_marker address_markers[] = {
	[USER_SPACE_NR]		= { 0,			"User Space" },
	[KERNEL_SPACE_NR]	= { (1UL << 63),	"Kernel Space" },
	[LOW_KERNEL_NR]		= { 0UL,		"Low Kernel Mapping" },
	[VMALLOC_START_NR]	= { 0UL,		"vmalloc() Area" },
	[VMEMMAP_START_NR]	= { 0UL,		"Vmemmap" },
#ifdef CONFIG_KASAN
	[KASAN_SHADOW_START_NR]	= { KASAN_SHADOW_START,	"KASAN shadow" },
	[KASAN_SHADOW_END_NR]	= { KASAN_SHADOW_END,	"KASAN shadow end" },
#endif
#ifdef CONFIG_MODIFY_LDT_SYSCALL
	[LDT_NR]		= { LDT_BASE_ADDR,	"LDT remap" },
#endif
	[CPU_ENTRY_AREA_NR]	= { CPU_ENTRY_AREA_BASE,"CPU entry Area" },
#ifdef CONFIG_X86_ESPFIX64
	[ESPFIX_START_NR]	= { ESPFIX_BASE_ADDR,	"ESPfix Area", 16 },
#endif
#ifdef CONFIG_EFI
	[EFI_END_NR]		= { EFI_VA_END,		"EFI Runtime Services" },
#endif
	[HIGH_KERNEL_NR]	= { __START_KERNEL_map,	"High Kernel Mapping" },
	[MODULES_VADDR_NR]	= { MODULES_VADDR,	"Modules" },
	[MODULES_END_NR]	= { MODULES_END,	"End Modules" },
	[FIXADDR_START_NR]	= { FIXADDR_START,	"Fixmap Area" },
	[END_OF_SPACE_NR]	= { -1,			NULL }
};

#else /* CONFIG_X86_64 */

enum address_markers_idx {
	USER_SPACE_NR = 0,
	KERNEL_SPACE_NR,
	VMALLOC_START_NR,
	VMALLOC_END_NR,
#ifdef CONFIG_HIGHMEM
	PKMAP_BASE_NR,
#endif
	CPU_ENTRY_AREA_NR,
	FIXADDR_START_NR,
	END_OF_SPACE_NR,
};

static struct addr_marker address_markers[] = {
	[USER_SPACE_NR]		= { 0,			"User Space" },
	[KERNEL_SPACE_NR]	= { PAGE_OFFSET,	"Kernel Mapping" },
	[VMALLOC_START_NR]	= { 0UL,		"vmalloc() Area" },
	[VMALLOC_END_NR]	= { 0UL,		"vmalloc() End" },
#ifdef CONFIG_HIGHMEM
	[PKMAP_BASE_NR]		= { 0UL,		"Persistent kmap() Area" },
#endif
	[CPU_ENTRY_AREA_NR]	= { 0UL,		"CPU entry area" },
	[FIXADDR_START_NR]	= { 0UL,		"Fixmap area" },
	[END_OF_SPACE_NR]	= { -1,			NULL }
};

#endif /* !CONFIG_X86_64 */

/* Multipliers for offsets within the PTEs */
#define PTE_LEVEL_MULT (PAGE_SIZE)
#define PMD_LEVEL_MULT (PTRS_PER_PTE * PTE_LEVEL_MULT)
#define PUD_LEVEL_MULT (PTRS_PER_PMD * PMD_LEVEL_MULT)
#define P4D_LEVEL_MULT (PTRS_PER_PUD * PUD_LEVEL_MULT)
#define PGD_LEVEL_MULT (PTRS_PER_P4D * P4D_LEVEL_MULT)

#define pt_dump_seq_printf(m, to_dmesg, fmt, args...)		\
({								\
	if (to_dmesg)					\
		printk(KERN_INFO fmt, ##args);			\
	else							\
		if (m)						\
			seq_printf(m, fmt, ##args);		\
})

#define pt_dump_cont_printf(m, to_dmesg, fmt, args...)		\
({								\
	if (to_dmesg)					\
		printk(KERN_CONT fmt, ##args);			\
	else							\
		if (m)						\
			seq_printf(m, fmt, ##args);		\
})

/*
 * Print a readable form of a pgprot_t to the seq_file
 */
static void printk_prot(struct seq_file *m, pgprot_t prot, int level, bool dmsg)
{
	pgprotval_t pr = pgprot_val(prot);
	static const char * const level_name[] =
		{ "cr3", "pgd", "p4d", "pud", "pmd", "pte" };

	if (!(pr & _PAGE_PRESENT)) {
		/* Not present */
		pt_dump_cont_printf(m, dmsg, "                              ");
	} else {
		if (pr & _PAGE_USER)
			pt_dump_cont_printf(m, dmsg, "USR ");
		else
			pt_dump_cont_printf(m, dmsg, "    ");
		if (pr & _PAGE_RW)
			pt_dump_cont_printf(m, dmsg, "RW ");
		else
			pt_dump_cont_printf(m, dmsg, "ro ");
		if (pr & _PAGE_PWT)
			pt_dump_cont_printf(m, dmsg, "PWT ");
		else
			pt_dump_cont_printf(m, dmsg, "    ");
		if (pr & _PAGE_PCD)
			pt_dump_cont_printf(m, dmsg, "PCD ");
		else
			pt_dump_cont_printf(m, dmsg, "    ");

		/* Bit 7 has a different meaning on level 3 vs 4 */
		if (level <= 4 && pr & _PAGE_PSE)
			pt_dump_cont_printf(m, dmsg, "PSE ");
		else
			pt_dump_cont_printf(m, dmsg, "    ");
		if ((level == 5 && pr & _PAGE_PAT) ||
		    ((level == 4 || level == 3) && pr & _PAGE_PAT_LARGE))
			pt_dump_cont_printf(m, dmsg, "PAT ");
		else
			pt_dump_cont_printf(m, dmsg, "    ");
		if (pr & _PAGE_GLOBAL)
			pt_dump_cont_printf(m, dmsg, "GLB ");
		else
			pt_dump_cont_printf(m, dmsg, "    ");
		if (pr & _PAGE_NX)
			pt_dump_cont_printf(m, dmsg, "NX ");
		else
			pt_dump_cont_printf(m, dmsg, "x  ");
	}
	pt_dump_cont_printf(m, dmsg, "%s\n", level_name[level]);
}

/*
 * On 64 bits, sign-extend the 48 bit address to 64 bit
 */
static unsigned long normalize_addr(unsigned long u)
{
	int shift;
	if (!IS_ENABLED(CONFIG_X86_64))
		return u;

	shift = 64 - (__VIRTUAL_MASK_SHIFT + 1);
	return (signed long)(u << shift) >> shift;
}

/*
 * This function gets called on a break in a continuous series
 * of PTE entries; the next one is different so we need to
 * print what we collected so far.
 */
static void note_page(struct seq_file *m, struct pg_state *st,
		      pgprot_t new_prot, int level)
{
	pgprotval_t prot, cur;
	static const char units[] = "BKMGTPE";

	/*
	 * If we have a "break" in the series, we need to flush the state that
	 * we have now. "break" is either changing perms, levels or
	 * address space marker.
	 */
	prot = pgprot_val(new_prot);
	cur = pgprot_val(st->current_prot);

	if (!st->level) {
		/* First entry */
		st->current_prot = new_prot;
		st->level = level;
		st->marker = address_markers;
		st->lines = 0;
		pt_dump_seq_printf(m, st->to_dmesg, "---[ %s ]---\n",
				   st->marker->name);
	} else if (prot != cur || level != st->level ||
		   st->current_address >= st->marker[1].start_address) {
		const char *unit = units;
		unsigned long delta;
		int width = sizeof(unsigned long) * 2;
		pgprotval_t pr = pgprot_val(st->current_prot);

		if (st->check_wx && (pr & _PAGE_RW) && !(pr & _PAGE_NX)) {
			WARN_ONCE(1,
				  "x86/mm: Found insecure W+X mapping at address %p/%pS\n",
				  (void *)st->start_address,
				  (void *)st->start_address);
			st->wx_pages += (st->current_address -
					 st->start_address) / PAGE_SIZE;
		}

		/*
		 * Now print the actual finished series
		 */
		if (!st->marker->max_lines ||
		    st->lines < st->marker->max_lines) {
			pt_dump_seq_printf(m, st->to_dmesg,
					   "0x%0*lx-0x%0*lx   ",
					   width, st->start_address,
					   width, st->current_address);

			delta = st->current_address - st->start_address;
			while (!(delta & 1023) && unit[1]) {
				delta >>= 10;
				unit++;
			}
			pt_dump_cont_printf(m, st->to_dmesg, "%9lu%c ",
					    delta, *unit);
			printk_prot(m, st->current_prot, st->level,
				    st->to_dmesg);
		}
		st->lines++;

		/*
		 * We print markers for special areas of address space,
		 * such as the start of vmalloc space etc.
		 * This helps in the interpretation.
		 */
		if (st->current_address >= st->marker[1].start_address) {
			if (st->marker->max_lines &&
			    st->lines > st->marker->max_lines) {
				unsigned long nskip =
					st->lines - st->marker->max_lines;
				pt_dump_seq_printf(m, st->to_dmesg,
						   "... %lu entr%s skipped ... \n",
						   nskip,
						   nskip == 1 ? "y" : "ies");
			}
			st->marker++;
			st->lines = 0;
			pt_dump_seq_printf(m, st->to_dmesg, "---[ %s ]---\n",
					   st->marker->name);
		}

		st->start_address = st->current_address;
		st->current_prot = new_prot;
		st->level = level;
	}
}

static void walk_pte_level(struct seq_file *m, struct pg_state *st, pmd_t addr, unsigned long P)
{
	int i;
	pte_t *start;
	pgprotval_t prot;

	start = (pte_t *)pmd_page_vaddr(addr);
	for (i = 0; i < PTRS_PER_PTE; i++) {
		prot = pte_flags(*start);
		st->current_address = normalize_addr(P + i * PTE_LEVEL_MULT);
		note_page(m, st, __pgprot(prot), 5);
		start++;
	}
}
#ifdef CONFIG_KASAN

/*
 * This is an optimization for KASAN=y case. Since all kasan page tables
 * eventually point to the kasan_zero_page we could call note_page()
 * right away without walking through lower level page tables. This saves
 * us dozens of seconds (minutes for 5-level config) while checking for
 * W+X mapping or reading kernel_page_tables debugfs file.
 */
static inline bool kasan_page_table(struct seq_file *m, struct pg_state *st,
				void *pt)
{
	if (__pa(pt) == __pa(kasan_zero_pmd) ||
#ifdef CONFIG_X86_5LEVEL
	    __pa(pt) == __pa(kasan_zero_p4d) ||
#endif
	    __pa(pt) == __pa(kasan_zero_pud)) {
		pgprotval_t prot = pte_flags(kasan_zero_pte[0]);
		note_page(m, st, __pgprot(prot), 5);
		return true;
	}
	return false;
}
#else
static inline bool kasan_page_table(struct seq_file *m, struct pg_state *st,
				void *pt)
{
	return false;
}
#endif

#if PTRS_PER_PMD > 1

static void walk_pmd_level(struct seq_file *m, struct pg_state *st, pud_t addr, unsigned long P)
{
	int i;
	pmd_t *start, *pmd_start;
	pgprotval_t prot;

	pmd_start = start = (pmd_t *)pud_page_vaddr(addr);
	for (i = 0; i < PTRS_PER_PMD; i++) {
		st->current_address = normalize_addr(P + i * PMD_LEVEL_MULT);
		if (!pmd_none(*start)) {
			if (pmd_large(*start) || !pmd_present(*start)) {
				prot = pmd_flags(*start);
				note_page(m, st, __pgprot(prot), 4);
			} else if (!kasan_page_table(m, st, pmd_start)) {
				walk_pte_level(m, st, *start,
					       P + i * PMD_LEVEL_MULT);
			}
		} else
			note_page(m, st, __pgprot(0), 4);
		start++;
	}
}

#else
#define walk_pmd_level(m,s,a,p) walk_pte_level(m,s,__pmd(pud_val(a)),p)
#define pud_large(a) pmd_large(__pmd(pud_val(a)))
#define pud_none(a)  pmd_none(__pmd(pud_val(a)))
#endif

#if PTRS_PER_PUD > 1

static void walk_pud_level(struct seq_file *m, struct pg_state *st, p4d_t addr, unsigned long P)
{
	int i;
	pud_t *start, *pud_start;
	pgprotval_t prot;
	pud_t *prev_pud = NULL;

	pud_start = start = (pud_t *)p4d_page_vaddr(addr);

	for (i = 0; i < PTRS_PER_PUD; i++) {
		st->current_address = normalize_addr(P + i * PUD_LEVEL_MULT);
		if (!pud_none(*start)) {
			if (pud_large(*start) || !pud_present(*start)) {
				prot = pud_flags(*start);
				note_page(m, st, __pgprot(prot), 3);
			} else if (!kasan_page_table(m, st, pud_start)) {
				walk_pmd_level(m, st, *start,
					       P + i * PUD_LEVEL_MULT);
			}
		} else
			note_page(m, st, __pgprot(0), 3);

		prev_pud = start;
		start++;
	}
}

#else
#define walk_pud_level(m,s,a,p) walk_pmd_level(m,s,__pud(p4d_val(a)),p)
#define p4d_large(a) pud_large(__pud(p4d_val(a)))
#define p4d_none(a)  pud_none(__pud(p4d_val(a)))
#endif

#if PTRS_PER_P4D > 1

static void walk_p4d_level(struct seq_file *m, struct pg_state *st, pgd_t addr, unsigned long P)
{
	int i;
	p4d_t *start, *p4d_start;
	pgprotval_t prot;

	p4d_start = start = (p4d_t *)pgd_page_vaddr(addr);

	for (i = 0; i < PTRS_PER_P4D; i++) {
		st->current_address = normalize_addr(P + i * P4D_LEVEL_MULT);
		if (!p4d_none(*start)) {
			if (p4d_large(*start) || !p4d_present(*start)) {
				prot = p4d_flags(*start);
				note_page(m, st, __pgprot(prot), 2);
			} else if (!kasan_page_table(m, st, p4d_start)) {
				walk_pud_level(m, st, *start,
					       P + i * P4D_LEVEL_MULT);
			}
		} else
			note_page(m, st, __pgprot(0), 2);

		start++;
	}
}

#else
#define walk_p4d_level(m,s,a,p) walk_pud_level(m,s,__p4d(pgd_val(a)),p)
#define pgd_large(a) p4d_large(__p4d(pgd_val(a)))
#define pgd_none(a)  p4d_none(__p4d(pgd_val(a)))
#endif

static inline bool is_hypervisor_range(int idx)
{
#ifdef CONFIG_X86_64
	/*
	 * ffff800000000000 - ffff87ffffffffff is reserved for
	 * the hypervisor.
	 */
	return	(idx >= pgd_index(__PAGE_OFFSET) - 16) &&
		(idx <  pgd_index(__PAGE_OFFSET));
#else
	return false;
#endif
}

static void ptdump_walk_pgd_level_core(struct seq_file *m, pgd_t *pgd,
				       bool checkwx, bool dmesg)
{
#ifdef CONFIG_X86_64
	pgd_t *start = (pgd_t *) &init_top_pgt;
#else
	pgd_t *start = swapper_pg_dir;
#endif
	pgprotval_t prot;
	int i;
	struct pg_state st = {};

	if (pgd) {
		start = pgd;
		st.to_dmesg = dmesg;
	}

	st.check_wx = checkwx;
	if (checkwx)
		st.wx_pages = 0;

	for (i = 0; i < PTRS_PER_PGD; i++) {
		st.current_address = normalize_addr(i * PGD_LEVEL_MULT);
		if (!pgd_none(*start) && !is_hypervisor_range(i)) {
			if (pgd_large(*start) || !pgd_present(*start)) {
				prot = pgd_flags(*start);
				note_page(m, &st, __pgprot(prot), 1);
			} else {
				walk_p4d_level(m, &st, *start,
					       i * PGD_LEVEL_MULT);
			}
		} else
			note_page(m, &st, __pgprot(0), 1);

		cond_resched();
		start++;
	}

	/* Flush out the last page */
	st.current_address = normalize_addr(PTRS_PER_PGD*PGD_LEVEL_MULT);
	note_page(m, &st, __pgprot(0), 0);
	if (!checkwx)
		return;
	if (st.wx_pages)
		pr_info("x86/mm: Checked W+X mappings: FAILED, %lu W+X pages found.\n",
			st.wx_pages);
	else
		pr_info("x86/mm: Checked W+X mappings: passed, no W+X pages found.\n");
}

void ptdump_walk_pgd_level(struct seq_file *m, pgd_t *pgd)
{
	ptdump_walk_pgd_level_core(m, pgd, false, true);
}

void ptdump_walk_pgd_level_debugfs(struct seq_file *m, pgd_t *pgd, bool user)
{
#ifdef CONFIG_PAGE_TABLE_ISOLATION
	if (user && static_cpu_has(X86_FEATURE_PTI))
		pgd = kernel_to_user_pgdp(pgd);
#endif
	ptdump_walk_pgd_level_core(m, pgd, false, false);
}
EXPORT_SYMBOL_GPL(ptdump_walk_pgd_level_debugfs);

static void ptdump_walk_user_pgd_level_checkwx(void)
{
#ifdef CONFIG_PAGE_TABLE_ISOLATION
	pgd_t *pgd = (pgd_t *) &init_top_pgt;

	if (!static_cpu_has(X86_FEATURE_PTI))
		return;

	pr_info("x86/mm: Checking user space page tables\n");
	pgd = kernel_to_user_pgdp(pgd);
	ptdump_walk_pgd_level_core(NULL, pgd, true, false);
#endif
}

void ptdump_walk_pgd_level_checkwx(void)
{
	ptdump_walk_pgd_level_core(NULL, NULL, true, false);
	ptdump_walk_user_pgd_level_checkwx();
}

static int __init pt_dump_init(void)
{
	/*
	 * Various markers are not compile-time constants, so assign them
	 * here.
	 */
#ifdef CONFIG_X86_64
	address_markers[LOW_KERNEL_NR].start_address = PAGE_OFFSET;
	address_markers[VMALLOC_START_NR].start_address = VMALLOC_START;
	address_markers[VMEMMAP_START_NR].start_address = VMEMMAP_START;
#endif
#ifdef CONFIG_X86_32
	address_markers[VMALLOC_START_NR].start_address = VMALLOC_START;
	address_markers[VMALLOC_END_NR].start_address = VMALLOC_END;
# ifdef CONFIG_HIGHMEM
	address_markers[PKMAP_BASE_NR].start_address = PKMAP_BASE;
# endif
	address_markers[FIXADDR_START_NR].start_address = FIXADDR_START;
	address_markers[CPU_ENTRY_AREA_NR].start_address = CPU_ENTRY_AREA_BASE;
#endif
	return 0;
}
__initcall(pt_dump_init);
