/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  Copyright (C) 2017 Zihao Yu
 */

#include <linux/elf.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/moduleloader.h>

static int apply_r_riscv_64_rela(struct module *me, u32 *location, Elf_Addr v)
{
	*(u64 *)location = v;
	return 0;
}

static int apply_r_riscv_branch_rela(struct module *me, u32 *location,
				     Elf_Addr v)
{
	s64 offset = (void *)v - (void *)location;
	u32 imm12 = (offset & 0x1000) << (31 - 12);
	u32 imm11 = (offset & 0x800) >> (11 - 7);
	u32 imm10_5 = (offset & 0x7e0) << (30 - 10);
	u32 imm4_1 = (offset & 0x1e) << (11 - 4);

	*location = (*location & 0x1fff07f) | imm12 | imm11 | imm10_5 | imm4_1;
	return 0;
}

static int apply_r_riscv_jal_rela(struct module *me, u32 *location,
				  Elf_Addr v)
{
	s64 offset = (void *)v - (void *)location;
	u32 imm20 = (offset & 0x100000) << (31 - 20);
	u32 imm19_12 = (offset & 0xff000);
	u32 imm11 = (offset & 0x800) << (20 - 11);
	u32 imm10_1 = (offset & 0x7fe) << (30 - 10);

	*location = (*location & 0xfff) | imm20 | imm19_12 | imm11 | imm10_1;
	return 0;
}

static int apply_r_riscv_pcrel_hi20_rela(struct module *me, u32 *location,
					 Elf_Addr v)
{
	s64 offset = (void *)v - (void *)location;
	s32 hi20;

	if (offset != (s32)offset) {
		pr_err(
		  "%s: target %016llx can not be addressed by the 32-bit offset from PC = %p\n",
		  me->name, v, location);
		return -EINVAL;
	}

	hi20 = (offset + 0x800) & 0xfffff000;
	*location = (*location & 0xfff) | hi20;
	return 0;
}

static int apply_r_riscv_pcrel_lo12_i_rela(struct module *me, u32 *location,
					   Elf_Addr v)
{
	/*
	 * v is the lo12 value to fill. It is calculated before calling this
	 * handler.
	 */
	*location = (*location & 0xfffff) | ((v & 0xfff) << 20);
	return 0;
}

static int apply_r_riscv_pcrel_lo12_s_rela(struct module *me, u32 *location,
					   Elf_Addr v)
{
	/*
	 * v is the lo12 value to fill. It is calculated before calling this
	 * handler.
	 */
	u32 imm11_5 = (v & 0xfe0) << (31 - 11);
	u32 imm4_0 = (v & 0x1f) << (11 - 4);

	*location = (*location & 0x1fff07f) | imm11_5 | imm4_0;
	return 0;
}

static int apply_r_riscv_call_plt_rela(struct module *me, u32 *location,
				       Elf_Addr v)
{
	s64 offset = (void *)v - (void *)location;
	s32 fill_v = offset;
	u32 hi20, lo12;

	if (offset != fill_v) {
		pr_err(
		  "%s: target %016llx can not be addressed by the 32-bit offset from PC = %p\n",
		  me->name, v, location);
		return -EINVAL;
	}

	hi20 = (offset + 0x800) & 0xfffff000;
	lo12 = (offset - hi20) & 0xfff;
	*location = (*location & 0xfff) | hi20;
	*(location + 1) = (*(location + 1) & 0xfffff) | (lo12 << 20);
	return 0;
}

static int apply_r_riscv_relax_rela(struct module *me, u32 *location,
				    Elf_Addr v)
{
	return 0;
}

static int (*reloc_handlers_rela[]) (struct module *me, u32 *location,
				Elf_Addr v) = {
	[R_RISCV_64]			= apply_r_riscv_64_rela,
	[R_RISCV_BRANCH]		= apply_r_riscv_branch_rela,
	[R_RISCV_JAL]			= apply_r_riscv_jal_rela,
	[R_RISCV_PCREL_HI20]		= apply_r_riscv_pcrel_hi20_rela,
	[R_RISCV_PCREL_LO12_I]		= apply_r_riscv_pcrel_lo12_i_rela,
	[R_RISCV_PCREL_LO12_S]		= apply_r_riscv_pcrel_lo12_s_rela,
	[R_RISCV_CALL_PLT]		= apply_r_riscv_call_plt_rela,
	[R_RISCV_RELAX]			= apply_r_riscv_relax_rela,
};

int apply_relocate_add(Elf_Shdr *sechdrs, const char *strtab,
		       unsigned int symindex, unsigned int relsec,
		       struct module *me)
{
	Elf_Rela *rel = (void *) sechdrs[relsec].sh_addr;
	int (*handler)(struct module *me, u32 *location, Elf_Addr v);
	Elf_Sym *sym;
	u32 *location;
	unsigned int i, type;
	Elf_Addr v;
	int res;

	pr_debug("Applying relocate section %u to %u\n", relsec,
	       sechdrs[relsec].sh_info);

	for (i = 0; i < sechdrs[relsec].sh_size / sizeof(*rel); i++) {
		/* This is where to make the change */
		location = (void *)sechdrs[sechdrs[relsec].sh_info].sh_addr
			+ rel[i].r_offset;
		/* This is the symbol it is referring to */
		sym = (Elf_Sym *)sechdrs[symindex].sh_addr
			+ ELF_RISCV_R_SYM(rel[i].r_info);
		if (IS_ERR_VALUE(sym->st_value)) {
			/* Ignore unresolved weak symbol */
			if (ELF_ST_BIND(sym->st_info) == STB_WEAK)
				continue;
			pr_warning("%s: Unknown symbol %s\n",
				   me->name, strtab + sym->st_name);
			return -ENOENT;
		}

		type = ELF_RISCV_R_TYPE(rel[i].r_info);

		if (type < ARRAY_SIZE(reloc_handlers_rela))
			handler = reloc_handlers_rela[type];
		else
			handler = NULL;

		if (!handler) {
			pr_err("%s: Unknown relocation type %u\n",
			       me->name, type);
			return -EINVAL;
		}

		v = sym->st_value + rel[i].r_addend;

		if (type == R_RISCV_PCREL_LO12_I || type == R_RISCV_PCREL_LO12_S) {
			unsigned int j;

			for (j = 0; j < sechdrs[relsec].sh_size / sizeof(*rel); j++) {
				u64 hi20_loc =
					sechdrs[sechdrs[relsec].sh_info].sh_addr
					+ rel[j].r_offset;
				/* Find the corresponding HI20 PC-relative relocation entry */
				if (hi20_loc == sym->st_value) {
					Elf_Sym *hi20_sym =
						(Elf_Sym *)sechdrs[symindex].sh_addr
						+ ELF_RISCV_R_SYM(rel[j].r_info);
					u64 hi20_sym_val =
						hi20_sym->st_value
						+ rel[j].r_addend;
					/* Calculate lo12 */
					s64 offset = hi20_sym_val - hi20_loc;
					s32 hi20 = (offset + 0x800) & 0xfffff000;
					s32 lo12 = offset - hi20;
					v = lo12;
					break;
				}
			}
			if (j == sechdrs[relsec].sh_size / sizeof(*rel)) {
				pr_err(
				  "%s: Can not find HI20 PC-relative relocation information\n",
				  me->name);
				return -EINVAL;
			}
		}

		res = handler(me, location, v);
		if (res)
			return res;
	}

	return 0;
}
