// SPDX-License-Identifier: GPL-2.0-only
/* Altera TSE SGDMA and MSGDMA Linux driver
 * Copyright (C) 2014 Altera Corporation. All rights reserved
 */

#include "altera_eth_dma.h"
#include "altera_tse.h"
#include "altera_utils.h"
#include "linux/of_address.h"

void tse_set_bit(void __iomem *ioaddr, size_t offs, u32 bit_mask)
{
	u32 value = csrrd32(ioaddr, offs);
	value |= bit_mask;
	csrwr32(value, ioaddr, offs);
}

void tse_clear_bit(void __iomem *ioaddr, size_t offs, u32 bit_mask)
{
	u32 value = csrrd32(ioaddr, offs);
	value &= ~bit_mask;
	csrwr32(value, ioaddr, offs);
}

int tse_bit_is_set(void __iomem *ioaddr, size_t offs, u32 bit_mask)
{
	u32 value = csrrd32(ioaddr, offs);
	return (value & bit_mask) ? 1 : 0;
}

int tse_bit_is_clear(void __iomem *ioaddr, size_t offs, u32 bit_mask)
{
	u32 value = csrrd32(ioaddr, offs);
	return (value & bit_mask) ? 0 : 1;
}

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

int request_and_map_node(struct platform_device *pdev, struct device_node *dmanp,
			 const char *name, void __iomem **ptr)
{
	int ret = 0;
	struct resource *region;
	struct device *device = &pdev->dev;
	struct resource res={0};

	// Find the property by name
	int index = of_property_match_string(dmanp, "reg-names", name);

	if (index < 0) {
		// Handle error, register name not found
		pr_err("Register name not found: %s\n", name);
	} else {
		// Read the resource from the "reg" property
		ret = of_address_to_resource(dmanp, index, &res);
		if (ret != 0) {
			// Handle error, failed to read the resource
			pr_err("Failed to read resource %s from device node - Index: %d\n",
			       name, index);
		}
	}

	region = devm_request_mem_region(device, res.start,
					 resource_size(&res), dev_name(device));
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

	dev_info(&pdev->dev, "\tDMA %s  at 0x%08lx 0x%llx 0x%llx\n", name,
		 (unsigned long)res.start, (unsigned long long)res.end,
		 (unsigned long long)resource_size(&res));

	return 0;
}
