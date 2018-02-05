// SPDX-License-Identifier: GPL-2.0
static struct ins x86__instructions[] = {
	{ .name = "add",	.ops = &mov_ops,  },
	{ .name = "addl",	.ops = &mov_ops,  },
	{ .name = "addq",	.ops = &mov_ops,  },
	{ .name = "addw",	.ops = &mov_ops,  },
	{ .name = "and",	.ops = &mov_ops,  },
	{ .name = "bts",	.ops = &mov_ops,  },
	{ .name = "call",	.ops = &call_ops, },
	{ .name = "callq",	.ops = &call_ops, },
	{ .name = "cmp",	.ops = &mov_ops,  },
	{ .name = "cmpb",	.ops = &mov_ops,  },
	{ .name = "cmpl",	.ops = &mov_ops,  },
	{ .name = "cmpq",	.ops = &mov_ops,  },
	{ .name = "cmpw",	.ops = &mov_ops,  },
	{ .name = "cmpxch",	.ops = &mov_ops,  },
	{ .name = "dec",	.ops = &dec_ops,  },
	{ .name = "decl",	.ops = &dec_ops,  },
	{ .name = "imul",	.ops = &mov_ops,  },
	{ .name = "inc",	.ops = &dec_ops,  },
	{ .name = "incl",	.ops = &dec_ops,  },
	{ .name = "ja",		.ops = &jump_ops, },
	{ .name = "jae",	.ops = &jump_ops, },
	{ .name = "jb",		.ops = &jump_ops, },
	{ .name = "jbe",	.ops = &jump_ops, },
	{ .name = "jc",		.ops = &jump_ops, },
	{ .name = "jcxz",	.ops = &jump_ops, },
	{ .name = "je",		.ops = &jump_ops, },
	{ .name = "jecxz",	.ops = &jump_ops, },
	{ .name = "jg",		.ops = &jump_ops, },
	{ .name = "jge",	.ops = &jump_ops, },
	{ .name = "jl",		.ops = &jump_ops, },
	{ .name = "jle",	.ops = &jump_ops, },
	{ .name = "jmp",	.ops = &jump_ops, },
	{ .name = "jmpq",	.ops = &jump_ops, },
	{ .name = "jna",	.ops = &jump_ops, },
	{ .name = "jnae",	.ops = &jump_ops, },
	{ .name = "jnb",	.ops = &jump_ops, },
	{ .name = "jnbe",	.ops = &jump_ops, },
	{ .name = "jnc",	.ops = &jump_ops, },
	{ .name = "jne",	.ops = &jump_ops, },
	{ .name = "jng",	.ops = &jump_ops, },
	{ .name = "jnge",	.ops = &jump_ops, },
	{ .name = "jnl",	.ops = &jump_ops, },
	{ .name = "jnle",	.ops = &jump_ops, },
	{ .name = "jno",	.ops = &jump_ops, },
	{ .name = "jnp",	.ops = &jump_ops, },
	{ .name = "jns",	.ops = &jump_ops, },
	{ .name = "jnz",	.ops = &jump_ops, },
	{ .name = "jo",		.ops = &jump_ops, },
	{ .name = "jp",		.ops = &jump_ops, },
	{ .name = "jpe",	.ops = &jump_ops, },
	{ .name = "jpo",	.ops = &jump_ops, },
	{ .name = "jrcxz",	.ops = &jump_ops, },
	{ .name = "js",		.ops = &jump_ops, },
	{ .name = "jz",		.ops = &jump_ops, },
	{ .name = "lea",	.ops = &mov_ops,  },
	{ .name = "lock",	.ops = &lock_ops, },
	{ .name = "mov",	.ops = &mov_ops,  },
	{ .name = "movb",	.ops = &mov_ops,  },
	{ .name = "movdqa",	.ops = &mov_ops,  },
	{ .name = "movl",	.ops = &mov_ops,  },
	{ .name = "movq",	.ops = &mov_ops,  },
	{ .name = "movslq",	.ops = &mov_ops,  },
	{ .name = "movzbl",	.ops = &mov_ops,  },
	{ .name = "movzwl",	.ops = &mov_ops,  },
	{ .name = "nop",	.ops = &nop_ops,  },
	{ .name = "nopl",	.ops = &nop_ops,  },
	{ .name = "nopw",	.ops = &nop_ops,  },
	{ .name = "or",		.ops = &mov_ops,  },
	{ .name = "orl",	.ops = &mov_ops,  },
	{ .name = "test",	.ops = &mov_ops,  },
	{ .name = "testb",	.ops = &mov_ops,  },
	{ .name = "testl",	.ops = &mov_ops,  },
	{ .name = "xadd",	.ops = &mov_ops,  },
	{ .name = "xbeginl",	.ops = &jump_ops, },
	{ .name = "xbeginq",	.ops = &jump_ops, },
	{ .name = "retq",	.ops = &ret_ops,  },
};

static bool x86__ins_is_fused(struct arch *arch, const char *ins1,
			      const char *ins2)
{
	if (arch->family != 6 || arch->model < 0x1e || strstr(ins2, "jmp"))
		return false;

	if (arch->model == 0x1e) {
		/* Nehalem */
		if ((strstr(ins1, "cmp") && !strstr(ins1, "xchg")) ||
		     strstr(ins1, "test")) {
			return true;
		}
	} else {
		/* Newer platform */
		if ((strstr(ins1, "cmp") && !strstr(ins1, "xchg")) ||
		     strstr(ins1, "test") ||
		     strstr(ins1, "add") ||
		     strstr(ins1, "sub") ||
		     strstr(ins1, "and") ||
		     strstr(ins1, "inc") ||
		     strstr(ins1, "dec")) {
			return true;
		}
	}

	return false;
}

static int x86__cpuid_parse(struct arch *arch, char *cpuid)
{
	unsigned int family, model, stepping;
	int ret;

	/*
	 * cpuid = "GenuineIntel,family,model,stepping"
	 */
	ret = sscanf(cpuid, "%*[^,],%u,%u,%u", &family, &model, &stepping);
	if (ret == 3) {
		arch->family = family;
		arch->model = model;
		return 0;
	}

	return -1;
}

static int x86__annotate_init(struct arch *arch, char *cpuid)
{
	int err = 0;

	if (arch->initialized)
		return 0;

	if (cpuid)
		err = x86__cpuid_parse(arch, cpuid);

	arch->initialized = true;
	return err;
}
