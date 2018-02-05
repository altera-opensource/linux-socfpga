// SPDX-License-Identifier: GPL-2.0+
/*
 * PowerPC Memory Protection Keys management
 *
 * Copyright 2017, Ram Pai, IBM Corporation.
 */

#include <asm/mman.h>
#include <asm/setup.h>
#include <linux/pkeys.h>
#include <linux/of_device.h>

DEFINE_STATIC_KEY_TRUE(pkey_disabled);
bool pkey_execute_disable_supported;
int  pkeys_total;		/* Total pkeys as per device tree */
bool pkeys_devtree_defined;	/* pkey property exported by device tree */
u32  initial_allocation_mask;	/* Bits set for reserved keys */
u64  pkey_amr_uamor_mask;	/* Bits in AMR/UMOR not to be touched */
u64  pkey_iamr_mask;		/* Bits in AMR not to be touched */

#define AMR_BITS_PER_PKEY 2
#define AMR_RD_BIT 0x1UL
#define AMR_WR_BIT 0x2UL
#define IAMR_EX_BIT 0x1UL
#define PKEY_REG_BITS (sizeof(u64)*8)
#define pkeyshift(pkey) (PKEY_REG_BITS - ((pkey+1) * AMR_BITS_PER_PKEY))

static void scan_pkey_feature(void)
{
	u32 vals[2];
	struct device_node *cpu;

	cpu = of_find_node_by_type(NULL, "cpu");
	if (!cpu)
		return;

	if (of_property_read_u32_array(cpu,
			"ibm,processor-storage-keys", vals, 2))
		return;

	/*
	 * Since any pkey can be used for data or execute, we will just treat
	 * all keys as equal and track them as one entity.
	 */
	pkeys_total = be32_to_cpu(vals[0]);
	pkeys_devtree_defined = true;
}

static inline bool pkey_mmu_enabled(void)
{
	if (firmware_has_feature(FW_FEATURE_LPAR))
		return pkeys_total;
	else
		return cpu_has_feature(CPU_FTR_PKEY);
}

int pkey_initialize(void)
{
	int os_reserved, i;

	/*
	 * We define PKEY_DISABLE_EXECUTE in addition to the arch-neutral
	 * generic defines for PKEY_DISABLE_ACCESS and PKEY_DISABLE_WRITE.
	 * Ensure that the bits a distinct.
	 */
	BUILD_BUG_ON(PKEY_DISABLE_EXECUTE &
		     (PKEY_DISABLE_ACCESS | PKEY_DISABLE_WRITE));

	/*
	 * pkey_to_vmflag_bits() assumes that the pkey bits are contiguous
	 * in the vmaflag. Make sure that is really the case.
	 */
	BUILD_BUG_ON(__builtin_clzl(ARCH_VM_PKEY_FLAGS >> VM_PKEY_SHIFT) +
		     __builtin_popcountl(ARCH_VM_PKEY_FLAGS >> VM_PKEY_SHIFT)
				!= (sizeof(u64) * BITS_PER_BYTE));

	/* scan the device tree for pkey feature */
	scan_pkey_feature();

	/*
	 * Let's assume 32 pkeys on P8 bare metal, if its not defined by device
	 * tree. We make this exception since skiboot forgot to expose this
	 * property on power8.
	 */
	if (!pkeys_devtree_defined && !firmware_has_feature(FW_FEATURE_LPAR) &&
			cpu_has_feature(CPU_FTRS_POWER8))
		pkeys_total = 32;

	/*
	 * Adjust the upper limit, based on the number of bits supported by
	 * arch-neutral code.
	 */
	pkeys_total = min_t(int, pkeys_total,
			(ARCH_VM_PKEY_FLAGS >> VM_PKEY_SHIFT));

	if (!pkey_mmu_enabled() || radix_enabled() || !pkeys_total)
		static_branch_enable(&pkey_disabled);
	else
		static_branch_disable(&pkey_disabled);

	if (static_branch_likely(&pkey_disabled))
		return 0;

	/*
	 * The device tree cannot be relied to indicate support for
	 * execute_disable support. Instead we use a PVR check.
	 */
	if (pvr_version_is(PVR_POWER7) || pvr_version_is(PVR_POWER7p))
		pkey_execute_disable_supported = false;
	else
		pkey_execute_disable_supported = true;

#ifdef CONFIG_PPC_4K_PAGES
	/*
	 * The OS can manage only 8 pkeys due to its inability to represent them
	 * in the Linux 4K PTE.
	 */
	os_reserved = pkeys_total - 8;
#else
	os_reserved = 0;
#endif
	/*
	 * Bits are in LE format. NOTE: 1, 0 are reserved.
	 * key 0 is the default key, which allows read/write/execute.
	 * key 1 is recommended not to be used. PowerISA(3.0) page 1015,
	 * programming note.
	 */
	initial_allocation_mask = ~0x0;

	/* register mask is in BE format */
	pkey_amr_uamor_mask = ~0x0ul;
	pkey_iamr_mask = ~0x0ul;

	for (i = 2; i < (pkeys_total - os_reserved); i++) {
		initial_allocation_mask &= ~(0x1 << i);
		pkey_amr_uamor_mask &= ~(0x3ul << pkeyshift(i));
		pkey_iamr_mask &= ~(0x1ul << pkeyshift(i));
	}
	return 0;
}

arch_initcall(pkey_initialize);

void pkey_mm_init(struct mm_struct *mm)
{
	if (static_branch_likely(&pkey_disabled))
		return;
	mm_pkey_allocation_map(mm) = initial_allocation_mask;
	/* -1 means unallocated or invalid */
	mm->context.execute_only_pkey = -1;
}

static inline u64 read_amr(void)
{
	return mfspr(SPRN_AMR);
}

static inline void write_amr(u64 value)
{
	mtspr(SPRN_AMR, value);
}

static inline u64 read_iamr(void)
{
	if (!likely(pkey_execute_disable_supported))
		return 0x0UL;

	return mfspr(SPRN_IAMR);
}

static inline void write_iamr(u64 value)
{
	if (!likely(pkey_execute_disable_supported))
		return;

	mtspr(SPRN_IAMR, value);
}

static inline u64 read_uamor(void)
{
	return mfspr(SPRN_UAMOR);
}

static inline void write_uamor(u64 value)
{
	mtspr(SPRN_UAMOR, value);
}

static bool is_pkey_enabled(int pkey)
{
	u64 uamor = read_uamor();
	u64 pkey_bits = 0x3ul << pkeyshift(pkey);
	u64 uamor_pkey_bits = (uamor & pkey_bits);

	/*
	 * Both the bits in UAMOR corresponding to the key should be set or
	 * reset.
	 */
	WARN_ON(uamor_pkey_bits && (uamor_pkey_bits != pkey_bits));
	return !!(uamor_pkey_bits);
}

static inline void init_amr(int pkey, u8 init_bits)
{
	u64 new_amr_bits = (((u64)init_bits & 0x3UL) << pkeyshift(pkey));
	u64 old_amr = read_amr() & ~((u64)(0x3ul) << pkeyshift(pkey));

	write_amr(old_amr | new_amr_bits);
}

static inline void init_iamr(int pkey, u8 init_bits)
{
	u64 new_iamr_bits = (((u64)init_bits & 0x1UL) << pkeyshift(pkey));
	u64 old_iamr = read_iamr() & ~((u64)(0x1ul) << pkeyshift(pkey));

	write_iamr(old_iamr | new_iamr_bits);
}

static void pkey_status_change(int pkey, bool enable)
{
	u64 old_uamor;

	/* Reset the AMR and IAMR bits for this key */
	init_amr(pkey, 0x0);
	init_iamr(pkey, 0x0);

	/* Enable/disable key */
	old_uamor = read_uamor();
	if (enable)
		old_uamor |= (0x3ul << pkeyshift(pkey));
	else
		old_uamor &= ~(0x3ul << pkeyshift(pkey));
	write_uamor(old_uamor);
}

void __arch_activate_pkey(int pkey)
{
	pkey_status_change(pkey, true);
}

void __arch_deactivate_pkey(int pkey)
{
	pkey_status_change(pkey, false);
}

/*
 * Set the access rights in AMR IAMR and UAMOR registers for @pkey to that
 * specified in @init_val.
 */
int __arch_set_user_pkey_access(struct task_struct *tsk, int pkey,
				unsigned long init_val)
{
	u64 new_amr_bits = 0x0ul;
	u64 new_iamr_bits = 0x0ul;

	if (!is_pkey_enabled(pkey))
		return -EINVAL;

	if (init_val & PKEY_DISABLE_EXECUTE) {
		if (!pkey_execute_disable_supported)
			return -EINVAL;
		new_iamr_bits |= IAMR_EX_BIT;
	}
	init_iamr(pkey, new_iamr_bits);

	/* Set the bits we need in AMR: */
	if (init_val & PKEY_DISABLE_ACCESS)
		new_amr_bits |= AMR_RD_BIT | AMR_WR_BIT;
	else if (init_val & PKEY_DISABLE_WRITE)
		new_amr_bits |= AMR_WR_BIT;

	init_amr(pkey, new_amr_bits);
	return 0;
}

void thread_pkey_regs_save(struct thread_struct *thread)
{
	if (static_branch_likely(&pkey_disabled))
		return;

	/*
	 * TODO: Skip saving registers if @thread hasn't used any keys yet.
	 */
	thread->amr = read_amr();
	thread->iamr = read_iamr();
	thread->uamor = read_uamor();
}

void thread_pkey_regs_restore(struct thread_struct *new_thread,
			      struct thread_struct *old_thread)
{
	if (static_branch_likely(&pkey_disabled))
		return;

	/*
	 * TODO: Just set UAMOR to zero if @new_thread hasn't used any keys yet.
	 */
	if (old_thread->amr != new_thread->amr)
		write_amr(new_thread->amr);
	if (old_thread->iamr != new_thread->iamr)
		write_iamr(new_thread->iamr);
	if (old_thread->uamor != new_thread->uamor)
		write_uamor(new_thread->uamor);
}

void thread_pkey_regs_init(struct thread_struct *thread)
{
	if (static_branch_likely(&pkey_disabled))
		return;

	write_amr(read_amr() & pkey_amr_uamor_mask);
	write_iamr(read_iamr() & pkey_iamr_mask);
	write_uamor(read_uamor() & pkey_amr_uamor_mask);
}

static inline bool pkey_allows_readwrite(int pkey)
{
	int pkey_shift = pkeyshift(pkey);

	if (!is_pkey_enabled(pkey))
		return true;

	return !(read_amr() & ((AMR_RD_BIT|AMR_WR_BIT) << pkey_shift));
}

int __execute_only_pkey(struct mm_struct *mm)
{
	bool need_to_set_mm_pkey = false;
	int execute_only_pkey = mm->context.execute_only_pkey;
	int ret;

	/* Do we need to assign a pkey for mm's execute-only maps? */
	if (execute_only_pkey == -1) {
		/* Go allocate one to use, which might fail */
		execute_only_pkey = mm_pkey_alloc(mm);
		if (execute_only_pkey < 0)
			return -1;
		need_to_set_mm_pkey = true;
	}

	/*
	 * We do not want to go through the relatively costly dance to set AMR
	 * if we do not need to. Check it first and assume that if the
	 * execute-only pkey is readwrite-disabled than we do not have to set it
	 * ourselves.
	 */
	if (!need_to_set_mm_pkey && !pkey_allows_readwrite(execute_only_pkey))
		return execute_only_pkey;

	/*
	 * Set up AMR so that it denies access for everything other than
	 * execution.
	 */
	ret = __arch_set_user_pkey_access(current, execute_only_pkey,
					  PKEY_DISABLE_ACCESS |
					  PKEY_DISABLE_WRITE);
	/*
	 * If the AMR-set operation failed somehow, just return 0 and
	 * effectively disable execute-only support.
	 */
	if (ret) {
		mm_pkey_free(mm, execute_only_pkey);
		return -1;
	}

	/* We got one, store it and use it from here on out */
	if (need_to_set_mm_pkey)
		mm->context.execute_only_pkey = execute_only_pkey;
	return execute_only_pkey;
}

static inline bool vma_is_pkey_exec_only(struct vm_area_struct *vma)
{
	/* Do this check first since the vm_flags should be hot */
	if ((vma->vm_flags & (VM_READ | VM_WRITE | VM_EXEC)) != VM_EXEC)
		return false;

	return (vma_pkey(vma) == vma->vm_mm->context.execute_only_pkey);
}

/*
 * This should only be called for *plain* mprotect calls.
 */
int __arch_override_mprotect_pkey(struct vm_area_struct *vma, int prot,
				  int pkey)
{
	/*
	 * If the currently associated pkey is execute-only, but the requested
	 * protection requires read or write, move it back to the default pkey.
	 */
	if (vma_is_pkey_exec_only(vma) && (prot & (PROT_READ | PROT_WRITE)))
		return 0;

	/*
	 * The requested protection is execute-only. Hence let's use an
	 * execute-only pkey.
	 */
	if (prot == PROT_EXEC) {
		pkey = execute_only_pkey(vma->vm_mm);
		if (pkey > 0)
			return pkey;
	}

	/* Nothing to override. */
	return vma_pkey(vma);
}

static bool pkey_access_permitted(int pkey, bool write, bool execute)
{
	int pkey_shift;
	u64 amr;

	if (!pkey)
		return true;

	if (!is_pkey_enabled(pkey))
		return true;

	pkey_shift = pkeyshift(pkey);
	if (execute && !(read_iamr() & (IAMR_EX_BIT << pkey_shift)))
		return true;

	amr = read_amr(); /* Delay reading amr until absolutely needed */
	return ((!write && !(amr & (AMR_RD_BIT << pkey_shift))) ||
		(write &&  !(amr & (AMR_WR_BIT << pkey_shift))));
}

bool arch_pte_access_permitted(u64 pte, bool write, bool execute)
{
	if (static_branch_likely(&pkey_disabled))
		return true;

	return pkey_access_permitted(pte_to_pkey_bits(pte), write, execute);
}

/*
 * We only want to enforce protection keys on the current thread because we
 * effectively have no access to AMR/IAMR for other threads or any way to tell
 * which AMR/IAMR in a threaded process we could use.
 *
 * So do not enforce things if the VMA is not from the current mm, or if we are
 * in a kernel thread.
 */
static inline bool vma_is_foreign(struct vm_area_struct *vma)
{
	if (!current->mm)
		return true;

	/* if it is not our ->mm, it has to be foreign */
	if (current->mm != vma->vm_mm)
		return true;

	return false;
}

bool arch_vma_access_permitted(struct vm_area_struct *vma, bool write,
			       bool execute, bool foreign)
{
	if (static_branch_likely(&pkey_disabled))
		return true;
	/*
	 * Do not enforce our key-permissions on a foreign vma.
	 */
	if (foreign || vma_is_foreign(vma))
		return true;

	return pkey_access_permitted(vma_pkey(vma), write, execute);
}
