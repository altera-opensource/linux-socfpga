// SPDX-License-Identifier: GPL-2.0-only
/* Altera TSE SGDMA and MSGDMA Linux driver
 * Copyright (C) 2014 Altera Corporation. All rights reserved
 */

#include "altera_eth_dma.h"
#include "altera_tse.h"
#include "altera_utils.h"

void tse_set_bit(void __iomem *ioaddr, size_t offs, u32 bit_mask)
{
	u32 value = csrrd32(ioaddr, offs);
	value |= bit_mask;
	csrwr32(value, ioaddr, offs);
}
EXPORT_SYMBOL(tse_set_bit);

void tse_clear_bit(void __iomem *ioaddr, size_t offs, u32 bit_mask)
{
	u32 value = csrrd32(ioaddr, offs);
	value &= ~bit_mask;
	csrwr32(value, ioaddr, offs);
}
EXPORT_SYMBOL(tse_clear_bit);

int tse_bit_is_set(void __iomem *ioaddr, size_t offs, u32 bit_mask)
{
	u32 value = csrrd32(ioaddr, offs);
	return (value & bit_mask) ? 1 : 0;
}
EXPORT_SYMBOL(tse_bit_is_set);

int tse_bit_is_clear(void __iomem *ioaddr, size_t offs, u32 bit_mask)
{
	u32 value = csrrd32(ioaddr, offs);
	return (value & bit_mask) ? 0 : 1;
}
EXPORT_SYMBOL(tse_bit_is_clear);

int request_and_map(struct platform_device *pdev, const char *name,
		    struct resource **res, void __iomem **ptr)
{
	struct resource *region;
	struct device *device = &pdev->dev;

	*res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!*res) {
		dev_err(device, "resource %s not defined\n", name);
		return -ENODEV;
	}

	region = devm_request_mem_region(device, (*res)->start,
					 resource_size(*res), dev_name(device));
	if (!region) {
		dev_err(device, "unable to request %s\n", name);
		return -EBUSY;
	}

	*ptr = devm_ioremap(device, region->start,
			    resource_size(region));
	if (!*ptr) {
		dev_err(device, "ioremap of %s failed!", name);
		return -ENOMEM;
	}

	return 0;
}
EXPORT_SYMBOL(request_and_map);
