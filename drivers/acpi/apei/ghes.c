/*
 * APEI Generic Hardware Error Source support
 *
 * Generic Hardware Error Source provides a way to report platform
 * hardware errors (such as that from chipset). It works in so called
 * "Firmware First" mode, that is, hardware errors are reported to
 * firmware firstly, then reported to Linux by firmware. This way,
 * some non-standard hardware error registers or non-standard hardware
 * link can be checked by firmware to produce more hardware error
 * information for Linux.
 *
 * For more information about Generic Hardware Error Source, please
 * refer to ACPI Specification version 4.0, section 17.3.2.6
 *
 * Copyright 2010,2011 Intel Corp.
 *   Author: Huang Ying <ying.huang@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/acpi.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/cper.h>
#include <linux/kdebug.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/ratelimit.h>
#include <linux/vmalloc.h>
#include <linux/irq_work.h>
#include <linux/llist.h>
#include <linux/genalloc.h>
#include <linux/pci.h>
#include <linux/aer.h>
#include <linux/nmi.h>
#include <linux/sched/clock.h>
#include <linux/uuid.h>
#include <linux/ras.h>

#include <acpi/actbl1.h>
#include <acpi/ghes.h>
#include <acpi/apei.h>
#include <asm/fixmap.h>
#include <asm/tlbflush.h>
#include <ras/ras_event.h>

#include "apei-internal.h"

#define GHES_PFX	"GHES: "

#define GHES_ESTATUS_MAX_SIZE		65536
#define GHES_ESOURCE_PREALLOC_MAX_SIZE	65536

#define GHES_ESTATUS_POOL_MIN_ALLOC_ORDER 3

/* This is just an estimation for memory pool allocation */
#define GHES_ESTATUS_CACHE_AVG_SIZE	512

#define GHES_ESTATUS_CACHES_SIZE	4

#define GHES_ESTATUS_IN_CACHE_MAX_NSEC	10000000000ULL
/* Prevent too many caches are allocated because of RCU */
#define GHES_ESTATUS_CACHE_ALLOCED_MAX	(GHES_ESTATUS_CACHES_SIZE * 3 / 2)

#define GHES_ESTATUS_CACHE_LEN(estatus_len)			\
	(sizeof(struct ghes_estatus_cache) + (estatus_len))
#define GHES_ESTATUS_FROM_CACHE(estatus_cache)			\
	((struct acpi_hest_generic_status *)				\
	 ((struct ghes_estatus_cache *)(estatus_cache) + 1))

#define GHES_ESTATUS_NODE_LEN(estatus_len)			\
	(sizeof(struct ghes_estatus_node) + (estatus_len))
#define GHES_ESTATUS_FROM_NODE(estatus_node)			\
	((struct acpi_hest_generic_status *)				\
	 ((struct ghes_estatus_node *)(estatus_node) + 1))

static inline bool is_hest_type_generic_v2(struct ghes *ghes)
{
	return ghes->generic->header.type == ACPI_HEST_TYPE_GENERIC_ERROR_V2;
}

/*
 * This driver isn't really modular, however for the time being,
 * continuing to use module_param is the easiest way to remain
 * compatible with existing boot arg use cases.
 */
bool ghes_disable;
module_param_named(disable, ghes_disable, bool, 0);

/*
 * All error sources notified with HED (Hardware Error Device) share a
 * single notifier callback, so they need to be linked and checked one
 * by one. This holds true for NMI too.
 *
 * RCU is used for these lists, so ghes_list_mutex is only used for
 * list changing, not for traversing.
 */
static LIST_HEAD(ghes_hed);
static DEFINE_MUTEX(ghes_list_mutex);

/*
 * Because the memory area used to transfer hardware error information
 * from BIOS to Linux can be determined only in NMI, IRQ or timer
 * handler, but general ioremap can not be used in atomic context, so
 * the fixmap is used instead.
 *
 * These 2 spinlocks are used to prevent the fixmap entries from being used
 * simultaneously.
 */
static DEFINE_RAW_SPINLOCK(ghes_ioremap_lock_nmi);
static DEFINE_SPINLOCK(ghes_ioremap_lock_irq);

static struct gen_pool *ghes_estatus_pool;
static unsigned long ghes_estatus_pool_size_request;

static struct ghes_estatus_cache *ghes_estatus_caches[GHES_ESTATUS_CACHES_SIZE];
static atomic_t ghes_estatus_cache_alloced;

static int ghes_panic_timeout __read_mostly = 30;

static void __iomem *ghes_ioremap_pfn_nmi(u64 pfn)
{
	phys_addr_t paddr;
	pgprot_t prot;

	paddr = pfn << PAGE_SHIFT;
	prot = arch_apei_get_mem_attribute(paddr);
	__set_fixmap(FIX_APEI_GHES_NMI, paddr, prot);

	return (void __iomem *) fix_to_virt(FIX_APEI_GHES_NMI);
}

static void __iomem *ghes_ioremap_pfn_irq(u64 pfn)
{
	phys_addr_t paddr;
	pgprot_t prot;

	paddr = pfn << PAGE_SHIFT;
	prot = arch_apei_get_mem_attribute(paddr);
	__set_fixmap(FIX_APEI_GHES_IRQ, paddr, prot);

	return (void __iomem *) fix_to_virt(FIX_APEI_GHES_IRQ);
}

static void ghes_iounmap_nmi(void)
{
	clear_fixmap(FIX_APEI_GHES_NMI);
}

static void ghes_iounmap_irq(void)
{
	clear_fixmap(FIX_APEI_GHES_IRQ);
}

static int ghes_estatus_pool_init(void)
{
	ghes_estatus_pool = gen_pool_create(GHES_ESTATUS_POOL_MIN_ALLOC_ORDER, -1);
	if (!ghes_estatus_pool)
		return -ENOMEM;
	return 0;
}

static void ghes_estatus_pool_free_chunk_page(struct gen_pool *pool,
					      struct gen_pool_chunk *chunk,
					      void *data)
{
	free_page(chunk->start_addr);
}

static void ghes_estatus_pool_exit(void)
{
	gen_pool_for_each_chunk(ghes_estatus_pool,
				ghes_estatus_pool_free_chunk_page, NULL);
	gen_pool_destroy(ghes_estatus_pool);
}

static int ghes_estatus_pool_expand(unsigned long len)
{
	unsigned long i, pages, size, addr;
	int ret;

	ghes_estatus_pool_size_request += PAGE_ALIGN(len);
	size = gen_pool_size(ghes_estatus_pool);
	if (size >= ghes_estatus_pool_size_request)
		return 0;
	pages = (ghes_estatus_pool_size_request - size) / PAGE_SIZE;
	for (i = 0; i < pages; i++) {
		addr = __get_free_page(GFP_KERNEL);
		if (!addr)
			return -ENOMEM;
		ret = gen_pool_add(ghes_estatus_pool, addr, PAGE_SIZE, -1);
		if (ret)
			return ret;
	}

	return 0;
}

static int map_gen_v2(struct ghes *ghes)
{
	return apei_map_generic_address(&ghes->generic_v2->read_ack_register);
}

static void unmap_gen_v2(struct ghes *ghes)
{
	apei_unmap_generic_address(&ghes->generic_v2->read_ack_register);
}

static struct ghes *ghes_new(struct acpi_hest_generic *generic)
{
	struct ghes *ghes;
	unsigned int error_block_length;
	int rc;

	ghes = kzalloc(sizeof(*ghes), GFP_KERNEL);
	if (!ghes)
		return ERR_PTR(-ENOMEM);

	ghes->generic = generic;
	if (is_hest_type_generic_v2(ghes)) {
		rc = map_gen_v2(ghes);
		if (rc)
			goto err_free;
	}

	rc = apei_map_generic_address(&generic->error_status_address);
	if (rc)
		goto err_unmap_read_ack_addr;
	error_block_length = generic->error_block_length;
	if (error_block_length > GHES_ESTATUS_MAX_SIZE) {
		pr_warning(FW_WARN GHES_PFX
			   "Error status block length is too long: %u for "
			   "generic hardware error source: %d.\n",
			   error_block_length, generic->header.source_id);
		error_block_length = GHES_ESTATUS_MAX_SIZE;
	}
	ghes->estatus = kmalloc(error_block_length, GFP_KERNEL);
	if (!ghes->estatus) {
		rc = -ENOMEM;
		goto err_unmap_status_addr;
	}

	return ghes;

err_unmap_status_addr:
	apei_unmap_generic_address(&generic->error_status_address);
err_unmap_read_ack_addr:
	if (is_hest_type_generic_v2(ghes))
		unmap_gen_v2(ghes);
err_free:
	kfree(ghes);
	return ERR_PTR(rc);
}

static void ghes_fini(struct ghes *ghes)
{
	kfree(ghes->estatus);
	apei_unmap_generic_address(&ghes->generic->error_status_address);
	if (is_hest_type_generic_v2(ghes))
		unmap_gen_v2(ghes);
}

static inline int ghes_severity(int severity)
{
	switch (severity) {
	case CPER_SEV_INFORMATIONAL:
		return GHES_SEV_NO;
	case CPER_SEV_CORRECTED:
		return GHES_SEV_CORRECTED;
	case CPER_SEV_RECOVERABLE:
		return GHES_SEV_RECOVERABLE;
	case CPER_SEV_FATAL:
		return GHES_SEV_PANIC;
	default:
		/* Unknown, go panic */
		return GHES_SEV_PANIC;
	}
}

static void ghes_copy_tofrom_phys(void *buffer, u64 paddr, u32 len,
				  int from_phys)
{
	void __iomem *vaddr;
	unsigned long flags = 0;
	int in_nmi = in_nmi();
	u64 offset;
	u32 trunk;

	while (len > 0) {
		offset = paddr - (paddr & PAGE_MASK);
		if (in_nmi) {
			raw_spin_lock(&ghes_ioremap_lock_nmi);
			vaddr = ghes_ioremap_pfn_nmi(paddr >> PAGE_SHIFT);
		} else {
			spin_lock_irqsave(&ghes_ioremap_lock_irq, flags);
			vaddr = ghes_ioremap_pfn_irq(paddr >> PAGE_SHIFT);
		}
		trunk = PAGE_SIZE - offset;
		trunk = min(trunk, len);
		if (from_phys)
			memcpy_fromio(buffer, vaddr + offset, trunk);
		else
			memcpy_toio(vaddr + offset, buffer, trunk);
		len -= trunk;
		paddr += trunk;
		buffer += trunk;
		if (in_nmi) {
			ghes_iounmap_nmi();
			raw_spin_unlock(&ghes_ioremap_lock_nmi);
		} else {
			ghes_iounmap_irq();
			spin_unlock_irqrestore(&ghes_ioremap_lock_irq, flags);
		}
	}
}

static int ghes_read_estatus(struct ghes *ghes, int silent)
{
	struct acpi_hest_generic *g = ghes->generic;
	u64 buf_paddr;
	u32 len;
	int rc;

	rc = apei_read(&buf_paddr, &g->error_status_address);
	if (rc) {
		if (!silent && printk_ratelimit())
			pr_warning(FW_WARN GHES_PFX
"Failed to read error status block address for hardware error source: %d.\n",
				   g->header.source_id);
		return -EIO;
	}
	if (!buf_paddr)
		return -ENOENT;

	ghes_copy_tofrom_phys(ghes->estatus, buf_paddr,
			      sizeof(*ghes->estatus), 1);
	if (!ghes->estatus->block_status)
		return -ENOENT;

	ghes->buffer_paddr = buf_paddr;
	ghes->flags |= GHES_TO_CLEAR;

	rc = -EIO;
	len = cper_estatus_len(ghes->estatus);
	if (len < sizeof(*ghes->estatus))
		goto err_read_block;
	if (len > ghes->generic->error_block_length)
		goto err_read_block;
	if (cper_estatus_check_header(ghes->estatus))
		goto err_read_block;
	ghes_copy_tofrom_phys(ghes->estatus + 1,
			      buf_paddr + sizeof(*ghes->estatus),
			      len - sizeof(*ghes->estatus), 1);
	if (cper_estatus_check(ghes->estatus))
		goto err_read_block;
	rc = 0;

err_read_block:
	if (rc && !silent && printk_ratelimit())
		pr_warning(FW_WARN GHES_PFX
			   "Failed to read error status block!\n");
	return rc;
}

static void ghes_clear_estatus(struct ghes *ghes)
{
	ghes->estatus->block_status = 0;
	if (!(ghes->flags & GHES_TO_CLEAR))
		return;
	ghes_copy_tofrom_phys(ghes->estatus, ghes->buffer_paddr,
			      sizeof(ghes->estatus->block_status), 0);
	ghes->flags &= ~GHES_TO_CLEAR;
}

static void ghes_handle_memory_failure(struct acpi_hest_generic_data *gdata, int sev)
{
#ifdef CONFIG_ACPI_APEI_MEMORY_FAILURE
	unsigned long pfn;
	int flags = -1;
	int sec_sev = ghes_severity(gdata->error_severity);
	struct cper_sec_mem_err *mem_err = acpi_hest_get_payload(gdata);

	if (!(mem_err->validation_bits & CPER_MEM_VALID_PA))
		return;

	pfn = mem_err->physical_addr >> PAGE_SHIFT;
	if (!pfn_valid(pfn)) {
		pr_warn_ratelimited(FW_WARN GHES_PFX
		"Invalid address in generic error data: %#llx\n",
		mem_err->physical_addr);
		return;
	}

	/* iff following two events can be handled properly by now */
	if (sec_sev == GHES_SEV_CORRECTED &&
	    (gdata->flags & CPER_SEC_ERROR_THRESHOLD_EXCEEDED))
		flags = MF_SOFT_OFFLINE;
	if (sev == GHES_SEV_RECOVERABLE && sec_sev == GHES_SEV_RECOVERABLE)
		flags = 0;

	if (flags != -1)
		memory_failure_queue(pfn, flags);
#endif
}

/*
 * PCIe AER errors need to be sent to the AER driver for reporting and
 * recovery. The GHES severities map to the following AER severities and
 * require the following handling:
 *
 * GHES_SEV_CORRECTABLE -> AER_CORRECTABLE
 *     These need to be reported by the AER driver but no recovery is
 *     necessary.
 * GHES_SEV_RECOVERABLE -> AER_NONFATAL
 * GHES_SEV_RECOVERABLE && CPER_SEC_RESET -> AER_FATAL
 *     These both need to be reported and recovered from by the AER driver.
 * GHES_SEV_PANIC does not make it to this handling since the kernel must
 *     panic.
 */
static void ghes_handle_aer(struct acpi_hest_generic_data *gdata)
{
#ifdef CONFIG_ACPI_APEI_PCIEAER
	struct cper_sec_pcie *pcie_err = acpi_hest_get_payload(gdata);

	if (pcie_err->validation_bits & CPER_PCIE_VALID_DEVICE_ID &&
	    pcie_err->validation_bits & CPER_PCIE_VALID_AER_INFO) {
		unsigned int devfn;
		int aer_severity;

		devfn = PCI_DEVFN(pcie_err->device_id.device,
				  pcie_err->device_id.function);
		aer_severity = cper_severity_to_aer(gdata->error_severity);

		/*
		 * If firmware reset the component to contain
		 * the error, we must reinitialize it before
		 * use, so treat it as a fatal AER error.
		 */
		if (gdata->flags & CPER_SEC_RESET)
			aer_severity = AER_FATAL;

		aer_recover_queue(pcie_err->device_id.segment,
				  pcie_err->device_id.bus,
				  devfn, aer_severity,
				  (struct aer_capability_regs *)
				  pcie_err->aer_info);
	}
#endif
}

static void ghes_do_proc(struct ghes *ghes,
			 const struct acpi_hest_generic_status *estatus)
{
	int sev, sec_sev;
	struct acpi_hest_generic_data *gdata;
	guid_t *sec_type;
	guid_t *fru_id = &NULL_UUID_LE;
	char *fru_text = "";

	sev = ghes_severity(estatus->error_severity);
	apei_estatus_for_each_section(estatus, gdata) {
		sec_type = (guid_t *)gdata->section_type;
		sec_sev = ghes_severity(gdata->error_severity);
		if (gdata->validation_bits & CPER_SEC_VALID_FRU_ID)
			fru_id = (guid_t *)gdata->fru_id;

		if (gdata->validation_bits & CPER_SEC_VALID_FRU_TEXT)
			fru_text = gdata->fru_text;

		if (guid_equal(sec_type, &CPER_SEC_PLATFORM_MEM)) {
			struct cper_sec_mem_err *mem_err = acpi_hest_get_payload(gdata);

			ghes_edac_report_mem_error(ghes, sev, mem_err);

			arch_apei_report_mem_error(sev, mem_err);
			ghes_handle_memory_failure(gdata, sev);
		}
		else if (guid_equal(sec_type, &CPER_SEC_PCIE)) {
			ghes_handle_aer(gdata);
		}
		else if (guid_equal(sec_type, &CPER_SEC_PROC_ARM)) {
			struct cper_sec_proc_arm *err = acpi_hest_get_payload(gdata);

			log_arm_hw_error(err);
		} else {
			void *err = acpi_hest_get_payload(gdata);

			log_non_standard_event(sec_type, fru_id, fru_text,
					       sec_sev, err,
					       gdata->error_data_length);
		}
	}
}

static void __ghes_print_estatus(const char *pfx,
				 const struct acpi_hest_generic *generic,
				 const struct acpi_hest_generic_status *estatus)
{
	static atomic_t seqno;
	unsigned int curr_seqno;
	char pfx_seq[64];

	if (pfx == NULL) {
		if (ghes_severity(estatus->error_severity) <=
		    GHES_SEV_CORRECTED)
			pfx = KERN_WARNING;
		else
			pfx = KERN_ERR;
	}
	curr_seqno = atomic_inc_return(&seqno);
	snprintf(pfx_seq, sizeof(pfx_seq), "%s{%u}" HW_ERR, pfx, curr_seqno);
	printk("%s""Hardware error from APEI Generic Hardware Error Source: %d\n",
	       pfx_seq, generic->header.source_id);
	cper_estatus_print(pfx_seq, estatus);
}

static int ghes_print_estatus(const char *pfx,
			      const struct acpi_hest_generic *generic,
			      const struct acpi_hest_generic_status *estatus)
{
	/* Not more than 2 messages every 5 seconds */
	static DEFINE_RATELIMIT_STATE(ratelimit_corrected, 5*HZ, 2);
	static DEFINE_RATELIMIT_STATE(ratelimit_uncorrected, 5*HZ, 2);
	struct ratelimit_state *ratelimit;

	if (ghes_severity(estatus->error_severity) <= GHES_SEV_CORRECTED)
		ratelimit = &ratelimit_corrected;
	else
		ratelimit = &ratelimit_uncorrected;
	if (__ratelimit(ratelimit)) {
		__ghes_print_estatus(pfx, generic, estatus);
		return 1;
	}
	return 0;
}

/*
 * GHES error status reporting throttle, to report more kinds of
 * errors, instead of just most frequently occurred errors.
 */
static int ghes_estatus_cached(struct acpi_hest_generic_status *estatus)
{
	u32 len;
	int i, cached = 0;
	unsigned long long now;
	struct ghes_estatus_cache *cache;
	struct acpi_hest_generic_status *cache_estatus;

	len = cper_estatus_len(estatus);
	rcu_read_lock();
	for (i = 0; i < GHES_ESTATUS_CACHES_SIZE; i++) {
		cache = rcu_dereference(ghes_estatus_caches[i]);
		if (cache == NULL)
			continue;
		if (len != cache->estatus_len)
			continue;
		cache_estatus = GHES_ESTATUS_FROM_CACHE(cache);
		if (memcmp(estatus, cache_estatus, len))
			continue;
		atomic_inc(&cache->count);
		now = sched_clock();
		if (now - cache->time_in < GHES_ESTATUS_IN_CACHE_MAX_NSEC)
			cached = 1;
		break;
	}
	rcu_read_unlock();
	return cached;
}

static struct ghes_estatus_cache *ghes_estatus_cache_alloc(
	struct acpi_hest_generic *generic,
	struct acpi_hest_generic_status *estatus)
{
	int alloced;
	u32 len, cache_len;
	struct ghes_estatus_cache *cache;
	struct acpi_hest_generic_status *cache_estatus;

	alloced = atomic_add_return(1, &ghes_estatus_cache_alloced);
	if (alloced > GHES_ESTATUS_CACHE_ALLOCED_MAX) {
		atomic_dec(&ghes_estatus_cache_alloced);
		return NULL;
	}
	len = cper_estatus_len(estatus);
	cache_len = GHES_ESTATUS_CACHE_LEN(len);
	cache = (void *)gen_pool_alloc(ghes_estatus_pool, cache_len);
	if (!cache) {
		atomic_dec(&ghes_estatus_cache_alloced);
		return NULL;
	}
	cache_estatus = GHES_ESTATUS_FROM_CACHE(cache);
	memcpy(cache_estatus, estatus, len);
	cache->estatus_len = len;
	atomic_set(&cache->count, 0);
	cache->generic = generic;
	cache->time_in = sched_clock();
	return cache;
}

static void ghes_estatus_cache_free(struct ghes_estatus_cache *cache)
{
	u32 len;

	len = cper_estatus_len(GHES_ESTATUS_FROM_CACHE(cache));
	len = GHES_ESTATUS_CACHE_LEN(len);
	gen_pool_free(ghes_estatus_pool, (unsigned long)cache, len);
	atomic_dec(&ghes_estatus_cache_alloced);
}

static void ghes_estatus_cache_rcu_free(struct rcu_head *head)
{
	struct ghes_estatus_cache *cache;

	cache = container_of(head, struct ghes_estatus_cache, rcu);
	ghes_estatus_cache_free(cache);
}

static void ghes_estatus_cache_add(
	struct acpi_hest_generic *generic,
	struct acpi_hest_generic_status *estatus)
{
	int i, slot = -1, count;
	unsigned long long now, duration, period, max_period = 0;
	struct ghes_estatus_cache *cache, *slot_cache = NULL, *new_cache;

	new_cache = ghes_estatus_cache_alloc(generic, estatus);
	if (new_cache == NULL)
		return;
	rcu_read_lock();
	now = sched_clock();
	for (i = 0; i < GHES_ESTATUS_CACHES_SIZE; i++) {
		cache = rcu_dereference(ghes_estatus_caches[i]);
		if (cache == NULL) {
			slot = i;
			slot_cache = NULL;
			break;
		}
		duration = now - cache->time_in;
		if (duration >= GHES_ESTATUS_IN_CACHE_MAX_NSEC) {
			slot = i;
			slot_cache = cache;
			break;
		}
		count = atomic_read(&cache->count);
		period = duration;
		do_div(period, (count + 1));
		if (period > max_period) {
			max_period = period;
			slot = i;
			slot_cache = cache;
		}
	}
	/* new_cache must be put into array after its contents are written */
	smp_wmb();
	if (slot != -1 && cmpxchg(ghes_estatus_caches + slot,
				  slot_cache, new_cache) == slot_cache) {
		if (slot_cache)
			call_rcu(&slot_cache->rcu, ghes_estatus_cache_rcu_free);
	} else
		ghes_estatus_cache_free(new_cache);
	rcu_read_unlock();
}

static int ghes_ack_error(struct acpi_hest_generic_v2 *gv2)
{
	int rc;
	u64 val = 0;

	rc = apei_read(&val, &gv2->read_ack_register);
	if (rc)
		return rc;

	val &= gv2->read_ack_preserve << gv2->read_ack_register.bit_offset;
	val |= gv2->read_ack_write    << gv2->read_ack_register.bit_offset;

	return apei_write(val, &gv2->read_ack_register);
}

static void __ghes_panic(struct ghes *ghes)
{
	__ghes_print_estatus(KERN_EMERG, ghes->generic, ghes->estatus);

	/* reboot to log the error! */
	if (!panic_timeout)
		panic_timeout = ghes_panic_timeout;
	panic("Fatal hardware error!");
}

static int ghes_proc(struct ghes *ghes)
{
	int rc;

	rc = ghes_read_estatus(ghes, 0);
	if (rc)
		goto out;

	if (ghes_severity(ghes->estatus->error_severity) >= GHES_SEV_PANIC) {
		__ghes_panic(ghes);
	}

	if (!ghes_estatus_cached(ghes->estatus)) {
		if (ghes_print_estatus(NULL, ghes->generic, ghes->estatus))
			ghes_estatus_cache_add(ghes->generic, ghes->estatus);
	}
	ghes_do_proc(ghes, ghes->estatus);

out:
	ghes_clear_estatus(ghes);

	if (rc == -ENOENT)
		return rc;

	/*
	 * GHESv2 type HEST entries introduce support for error acknowledgment,
	 * so only acknowledge the error if this support is present.
	 */
	if (is_hest_type_generic_v2(ghes))
		return ghes_ack_error(ghes->generic_v2);

	return rc;
}

static void ghes_add_timer(struct ghes *ghes)
{
	struct acpi_hest_generic *g = ghes->generic;
	unsigned long expire;

	if (!g->notify.poll_interval) {
		pr_warning(FW_WARN GHES_PFX "Poll interval is 0 for generic hardware error source: %d, disabled.\n",
			   g->header.source_id);
		return;
	}
	expire = jiffies + msecs_to_jiffies(g->notify.poll_interval);
	ghes->timer.expires = round_jiffies_relative(expire);
	add_timer(&ghes->timer);
}

static void ghes_poll_func(struct timer_list *t)
{
	struct ghes *ghes = from_timer(ghes, t, timer);

	ghes_proc(ghes);
	if (!(ghes->flags & GHES_EXITING))
		ghes_add_timer(ghes);
}

static irqreturn_t ghes_irq_func(int irq, void *data)
{
	struct ghes *ghes = data;
	int rc;

	rc = ghes_proc(ghes);
	if (rc)
		return IRQ_NONE;

	return IRQ_HANDLED;
}

static int ghes_notify_hed(struct notifier_block *this, unsigned long event,
			   void *data)
{
	struct ghes *ghes;
	int ret = NOTIFY_DONE;

	rcu_read_lock();
	list_for_each_entry_rcu(ghes, &ghes_hed, list) {
		if (!ghes_proc(ghes))
			ret = NOTIFY_OK;
	}
	rcu_read_unlock();

	return ret;
}

static struct notifier_block ghes_notifier_hed = {
	.notifier_call = ghes_notify_hed,
};

#ifdef CONFIG_ACPI_APEI_SEA
static LIST_HEAD(ghes_sea);

/*
 * Return 0 only if one of the SEA error sources successfully reported an error
 * record sent from the firmware.
 */
int ghes_notify_sea(void)
{
	struct ghes *ghes;
	int ret = -ENOENT;

	rcu_read_lock();
	list_for_each_entry_rcu(ghes, &ghes_sea, list) {
		if (!ghes_proc(ghes))
			ret = 0;
	}
	rcu_read_unlock();
	return ret;
}

static void ghes_sea_add(struct ghes *ghes)
{
	mutex_lock(&ghes_list_mutex);
	list_add_rcu(&ghes->list, &ghes_sea);
	mutex_unlock(&ghes_list_mutex);
}

static void ghes_sea_remove(struct ghes *ghes)
{
	mutex_lock(&ghes_list_mutex);
	list_del_rcu(&ghes->list);
	mutex_unlock(&ghes_list_mutex);
	synchronize_rcu();
}
#else /* CONFIG_ACPI_APEI_SEA */
static inline void ghes_sea_add(struct ghes *ghes) { }
static inline void ghes_sea_remove(struct ghes *ghes) { }
#endif /* CONFIG_ACPI_APEI_SEA */

#ifdef CONFIG_HAVE_ACPI_APEI_NMI
/*
 * printk is not safe in NMI context.  So in NMI handler, we allocate
 * required memory from lock-less memory allocator
 * (ghes_estatus_pool), save estatus into it, put them into lock-less
 * list (ghes_estatus_llist), then delay printk into IRQ context via
 * irq_work (ghes_proc_irq_work).  ghes_estatus_size_request record
 * required pool size by all NMI error source.
 */
static struct llist_head ghes_estatus_llist;
static struct irq_work ghes_proc_irq_work;

/*
 * NMI may be triggered on any CPU, so ghes_in_nmi is used for
 * having only one concurrent reader.
 */
static atomic_t ghes_in_nmi = ATOMIC_INIT(0);

static LIST_HEAD(ghes_nmi);

static void ghes_proc_in_irq(struct irq_work *irq_work)
{
	struct llist_node *llnode, *next;
	struct ghes_estatus_node *estatus_node;
	struct acpi_hest_generic *generic;
	struct acpi_hest_generic_status *estatus;
	u32 len, node_len;

	llnode = llist_del_all(&ghes_estatus_llist);
	/*
	 * Because the time order of estatus in list is reversed,
	 * revert it back to proper order.
	 */
	llnode = llist_reverse_order(llnode);
	while (llnode) {
		next = llnode->next;
		estatus_node = llist_entry(llnode, struct ghes_estatus_node,
					   llnode);
		estatus = GHES_ESTATUS_FROM_NODE(estatus_node);
		len = cper_estatus_len(estatus);
		node_len = GHES_ESTATUS_NODE_LEN(len);
		ghes_do_proc(estatus_node->ghes, estatus);
		if (!ghes_estatus_cached(estatus)) {
			generic = estatus_node->generic;
			if (ghes_print_estatus(NULL, generic, estatus))
				ghes_estatus_cache_add(generic, estatus);
		}
		gen_pool_free(ghes_estatus_pool, (unsigned long)estatus_node,
			      node_len);
		llnode = next;
	}
}

static void ghes_print_queued_estatus(void)
{
	struct llist_node *llnode;
	struct ghes_estatus_node *estatus_node;
	struct acpi_hest_generic *generic;
	struct acpi_hest_generic_status *estatus;

	llnode = llist_del_all(&ghes_estatus_llist);
	/*
	 * Because the time order of estatus in list is reversed,
	 * revert it back to proper order.
	 */
	llnode = llist_reverse_order(llnode);
	while (llnode) {
		estatus_node = llist_entry(llnode, struct ghes_estatus_node,
					   llnode);
		estatus = GHES_ESTATUS_FROM_NODE(estatus_node);
		generic = estatus_node->generic;
		ghes_print_estatus(NULL, generic, estatus);
		llnode = llnode->next;
	}
}

/* Save estatus for further processing in IRQ context */
static void __process_error(struct ghes *ghes)
{
#ifdef CONFIG_ARCH_HAVE_NMI_SAFE_CMPXCHG
	u32 len, node_len;
	struct ghes_estatus_node *estatus_node;
	struct acpi_hest_generic_status *estatus;

	if (ghes_estatus_cached(ghes->estatus))
		return;

	len = cper_estatus_len(ghes->estatus);
	node_len = GHES_ESTATUS_NODE_LEN(len);

	estatus_node = (void *)gen_pool_alloc(ghes_estatus_pool, node_len);
	if (!estatus_node)
		return;

	estatus_node->ghes = ghes;
	estatus_node->generic = ghes->generic;
	estatus = GHES_ESTATUS_FROM_NODE(estatus_node);
	memcpy(estatus, ghes->estatus, len);
	llist_add(&estatus_node->llnode, &ghes_estatus_llist);
#endif
}

static int ghes_notify_nmi(unsigned int cmd, struct pt_regs *regs)
{
	struct ghes *ghes;
	int sev, ret = NMI_DONE;

	if (!atomic_add_unless(&ghes_in_nmi, 1, 1))
		return ret;

	list_for_each_entry_rcu(ghes, &ghes_nmi, list) {
		if (ghes_read_estatus(ghes, 1)) {
			ghes_clear_estatus(ghes);
			continue;
		} else {
			ret = NMI_HANDLED;
		}

		sev = ghes_severity(ghes->estatus->error_severity);
		if (sev >= GHES_SEV_PANIC) {
			oops_begin();
			ghes_print_queued_estatus();
			__ghes_panic(ghes);
		}

		if (!(ghes->flags & GHES_TO_CLEAR))
			continue;

		__process_error(ghes);
		ghes_clear_estatus(ghes);
	}

#ifdef CONFIG_ARCH_HAVE_NMI_SAFE_CMPXCHG
	if (ret == NMI_HANDLED)
		irq_work_queue(&ghes_proc_irq_work);
#endif
	atomic_dec(&ghes_in_nmi);
	return ret;
}

static unsigned long ghes_esource_prealloc_size(
	const struct acpi_hest_generic *generic)
{
	unsigned long block_length, prealloc_records, prealloc_size;

	block_length = min_t(unsigned long, generic->error_block_length,
			     GHES_ESTATUS_MAX_SIZE);
	prealloc_records = max_t(unsigned long,
				 generic->records_to_preallocate, 1);
	prealloc_size = min_t(unsigned long, block_length * prealloc_records,
			      GHES_ESOURCE_PREALLOC_MAX_SIZE);

	return prealloc_size;
}

static void ghes_estatus_pool_shrink(unsigned long len)
{
	ghes_estatus_pool_size_request -= PAGE_ALIGN(len);
}

static void ghes_nmi_add(struct ghes *ghes)
{
	unsigned long len;

	len = ghes_esource_prealloc_size(ghes->generic);
	ghes_estatus_pool_expand(len);
	mutex_lock(&ghes_list_mutex);
	if (list_empty(&ghes_nmi))
		register_nmi_handler(NMI_LOCAL, ghes_notify_nmi, 0, "ghes");
	list_add_rcu(&ghes->list, &ghes_nmi);
	mutex_unlock(&ghes_list_mutex);
}

static void ghes_nmi_remove(struct ghes *ghes)
{
	unsigned long len;

	mutex_lock(&ghes_list_mutex);
	list_del_rcu(&ghes->list);
	if (list_empty(&ghes_nmi))
		unregister_nmi_handler(NMI_LOCAL, "ghes");
	mutex_unlock(&ghes_list_mutex);
	/*
	 * To synchronize with NMI handler, ghes can only be
	 * freed after NMI handler finishes.
	 */
	synchronize_rcu();
	len = ghes_esource_prealloc_size(ghes->generic);
	ghes_estatus_pool_shrink(len);
}

static void ghes_nmi_init_cxt(void)
{
	init_irq_work(&ghes_proc_irq_work, ghes_proc_in_irq);
}
#else /* CONFIG_HAVE_ACPI_APEI_NMI */
static inline void ghes_nmi_add(struct ghes *ghes) { }
static inline void ghes_nmi_remove(struct ghes *ghes) { }
static inline void ghes_nmi_init_cxt(void) { }
#endif /* CONFIG_HAVE_ACPI_APEI_NMI */

static int ghes_probe(struct platform_device *ghes_dev)
{
	struct acpi_hest_generic *generic;
	struct ghes *ghes = NULL;

	int rc = -EINVAL;

	generic = *(struct acpi_hest_generic **)ghes_dev->dev.platform_data;
	if (!generic->enabled)
		return -ENODEV;

	switch (generic->notify.type) {
	case ACPI_HEST_NOTIFY_POLLED:
	case ACPI_HEST_NOTIFY_EXTERNAL:
	case ACPI_HEST_NOTIFY_SCI:
	case ACPI_HEST_NOTIFY_GSIV:
	case ACPI_HEST_NOTIFY_GPIO:
		break;

	case ACPI_HEST_NOTIFY_SEA:
		if (!IS_ENABLED(CONFIG_ACPI_APEI_SEA)) {
			pr_warn(GHES_PFX "Generic hardware error source: %d notified via SEA is not supported\n",
				generic->header.source_id);
			rc = -ENOTSUPP;
			goto err;
		}
		break;
	case ACPI_HEST_NOTIFY_NMI:
		if (!IS_ENABLED(CONFIG_HAVE_ACPI_APEI_NMI)) {
			pr_warn(GHES_PFX "Generic hardware error source: %d notified via NMI interrupt is not supported!\n",
				generic->header.source_id);
			goto err;
		}
		break;
	case ACPI_HEST_NOTIFY_LOCAL:
		pr_warning(GHES_PFX "Generic hardware error source: %d notified via local interrupt is not supported!\n",
			   generic->header.source_id);
		goto err;
	default:
		pr_warning(FW_WARN GHES_PFX "Unknown notification type: %u for generic hardware error source: %d\n",
			   generic->notify.type, generic->header.source_id);
		goto err;
	}

	rc = -EIO;
	if (generic->error_block_length <
	    sizeof(struct acpi_hest_generic_status)) {
		pr_warning(FW_BUG GHES_PFX "Invalid error block length: %u for generic hardware error source: %d\n",
			   generic->error_block_length,
			   generic->header.source_id);
		goto err;
	}
	ghes = ghes_new(generic);
	if (IS_ERR(ghes)) {
		rc = PTR_ERR(ghes);
		ghes = NULL;
		goto err;
	}

	rc = ghes_edac_register(ghes, &ghes_dev->dev);
	if (rc < 0)
		goto err;

	switch (generic->notify.type) {
	case ACPI_HEST_NOTIFY_POLLED:
		timer_setup(&ghes->timer, ghes_poll_func, TIMER_DEFERRABLE);
		ghes_add_timer(ghes);
		break;
	case ACPI_HEST_NOTIFY_EXTERNAL:
		/* External interrupt vector is GSI */
		rc = acpi_gsi_to_irq(generic->notify.vector, &ghes->irq);
		if (rc) {
			pr_err(GHES_PFX "Failed to map GSI to IRQ for generic hardware error source: %d\n",
			       generic->header.source_id);
			goto err_edac_unreg;
		}
		rc = request_irq(ghes->irq, ghes_irq_func, IRQF_SHARED,
				 "GHES IRQ", ghes);
		if (rc) {
			pr_err(GHES_PFX "Failed to register IRQ for generic hardware error source: %d\n",
			       generic->header.source_id);
			goto err_edac_unreg;
		}
		break;

	case ACPI_HEST_NOTIFY_SCI:
	case ACPI_HEST_NOTIFY_GSIV:
	case ACPI_HEST_NOTIFY_GPIO:
		mutex_lock(&ghes_list_mutex);
		if (list_empty(&ghes_hed))
			register_acpi_hed_notifier(&ghes_notifier_hed);
		list_add_rcu(&ghes->list, &ghes_hed);
		mutex_unlock(&ghes_list_mutex);
		break;

	case ACPI_HEST_NOTIFY_SEA:
		ghes_sea_add(ghes);
		break;
	case ACPI_HEST_NOTIFY_NMI:
		ghes_nmi_add(ghes);
		break;
	default:
		BUG();
	}
	platform_set_drvdata(ghes_dev, ghes);

	/* Handle any pending errors right away */
	ghes_proc(ghes);

	return 0;
err_edac_unreg:
	ghes_edac_unregister(ghes);
err:
	if (ghes) {
		ghes_fini(ghes);
		kfree(ghes);
	}
	return rc;
}

static int ghes_remove(struct platform_device *ghes_dev)
{
	struct ghes *ghes;
	struct acpi_hest_generic *generic;

	ghes = platform_get_drvdata(ghes_dev);
	generic = ghes->generic;

	ghes->flags |= GHES_EXITING;
	switch (generic->notify.type) {
	case ACPI_HEST_NOTIFY_POLLED:
		del_timer_sync(&ghes->timer);
		break;
	case ACPI_HEST_NOTIFY_EXTERNAL:
		free_irq(ghes->irq, ghes);
		break;

	case ACPI_HEST_NOTIFY_SCI:
	case ACPI_HEST_NOTIFY_GSIV:
	case ACPI_HEST_NOTIFY_GPIO:
		mutex_lock(&ghes_list_mutex);
		list_del_rcu(&ghes->list);
		if (list_empty(&ghes_hed))
			unregister_acpi_hed_notifier(&ghes_notifier_hed);
		mutex_unlock(&ghes_list_mutex);
		synchronize_rcu();
		break;

	case ACPI_HEST_NOTIFY_SEA:
		ghes_sea_remove(ghes);
		break;
	case ACPI_HEST_NOTIFY_NMI:
		ghes_nmi_remove(ghes);
		break;
	default:
		BUG();
		break;
	}

	ghes_fini(ghes);

	ghes_edac_unregister(ghes);

	kfree(ghes);

	platform_set_drvdata(ghes_dev, NULL);

	return 0;
}

static struct platform_driver ghes_platform_driver = {
	.driver		= {
		.name	= "GHES",
	},
	.probe		= ghes_probe,
	.remove		= ghes_remove,
};

static int __init ghes_init(void)
{
	int rc;

	if (acpi_disabled)
		return -ENODEV;

	switch (hest_disable) {
	case HEST_NOT_FOUND:
		return -ENODEV;
	case HEST_DISABLED:
		pr_info(GHES_PFX "HEST is not enabled!\n");
		return -EINVAL;
	default:
		break;
	}

	if (ghes_disable) {
		pr_info(GHES_PFX "GHES is not enabled!\n");
		return -EINVAL;
	}

	ghes_nmi_init_cxt();

	rc = ghes_estatus_pool_init();
	if (rc)
		goto err;

	rc = ghes_estatus_pool_expand(GHES_ESTATUS_CACHE_AVG_SIZE *
				      GHES_ESTATUS_CACHE_ALLOCED_MAX);
	if (rc)
		goto err_pool_exit;

	rc = platform_driver_register(&ghes_platform_driver);
	if (rc)
		goto err_pool_exit;

	rc = apei_osc_setup();
	if (rc == 0 && osc_sb_apei_support_acked)
		pr_info(GHES_PFX "APEI firmware first mode is enabled by APEI bit and WHEA _OSC.\n");
	else if (rc == 0 && !osc_sb_apei_support_acked)
		pr_info(GHES_PFX "APEI firmware first mode is enabled by WHEA _OSC.\n");
	else if (rc && osc_sb_apei_support_acked)
		pr_info(GHES_PFX "APEI firmware first mode is enabled by APEI bit.\n");
	else
		pr_info(GHES_PFX "Failed to enable APEI firmware first mode.\n");

	return 0;
err_pool_exit:
	ghes_estatus_pool_exit();
err:
	return rc;
}
device_initcall(ghes_init);
