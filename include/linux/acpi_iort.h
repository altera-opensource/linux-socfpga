/*
 * Copyright (C) 2016, Semihalf
 *	Author: Tomasz Nowicki <tn@semihalf.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 * Place - Suite 330, Boston, MA 02111-1307 USA.
 */

#ifndef __ACPI_IORT_H__
#define __ACPI_IORT_H__

#include <linux/acpi.h>
#include <linux/fwnode.h>
#include <linux/irqdomain.h>

#define IORT_IRQ_MASK(irq)		(irq & 0xffffffffULL)
#define IORT_IRQ_TRIGGER_MASK(irq)	((irq >> 32) & 0xffffffffULL)

int iort_register_domain_token(int trans_id, struct fwnode_handle *fw_node);
void iort_deregister_domain_token(int trans_id);
struct fwnode_handle *iort_find_domain_token(int trans_id);
#ifdef CONFIG_ACPI_IORT
void acpi_iort_init(void);
u32 iort_msi_map_rid(struct device *dev, u32 req_id);
struct irq_domain *iort_get_device_domain(struct device *dev, u32 req_id);
void acpi_configure_pmsi_domain(struct device *dev);
int iort_pmsi_get_dev_id(struct device *dev, u32 *dev_id);
/* IOMMU interface */
void iort_dma_setup(struct device *dev, u64 *dma_addr, u64 *size);
const struct iommu_ops *iort_iommu_configure(struct device *dev);
#else
static inline void acpi_iort_init(void) { }
static inline u32 iort_msi_map_rid(struct device *dev, u32 req_id)
{ return req_id; }
static inline struct irq_domain *iort_get_device_domain(struct device *dev,
							u32 req_id)
{ return NULL; }
static inline void acpi_configure_pmsi_domain(struct device *dev) { }
/* IOMMU interface */
static inline void iort_dma_setup(struct device *dev, u64 *dma_addr,
				  u64 *size) { }
static inline const struct iommu_ops *iort_iommu_configure(
				      struct device *dev)
{ return NULL; }
#endif

#endif /* __ACPI_IORT_H__ */
