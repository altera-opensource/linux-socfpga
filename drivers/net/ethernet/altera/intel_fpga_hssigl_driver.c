// SPDX-License-Identifier: GPL-2.0
/* Intel FPGA HSSI generic glue logic driver
 * Copyright (C) 2024 Intel Corporation. All rights reserved
 *
 * Contributors:
 *   Preetam Narayan
 *
 */
#define DEBUG

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include "altera_utils.h"
#include "intel_fpga_hssiss.h"
#include "intel_fpga_hssi_tile_ops.h"
#include "intel_fpga_hssigl_driver.h"

int hssigldrv_probe_init(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *usrcsr;
	struct device_node *dev_tr;
	struct platform_device *pdev_tr;
	struct hssiss_private *priv = platform_get_drvdata(pdev);
        
	/* USR CSR address space */
        ret = request_and_map(pdev, "usr_csr", &usrcsr,          
                              (void __iomem **)&priv->usrcsr);
        if (ret)
                dev_warn(&pdev->dev, "No user space resource mapped");

	dev_info(&pdev->dev, "User CSR starts at 0x%08lx\n",
				(unsigned long)usrcsr->start);

	dev_tr = of_parse_phandle(pdev->dev.of_node, "tr-type", 0);
	if (!dev_tr)
		return -ENOENT;
	
	pdev_tr = of_find_device_by_node(dev_tr);
        if (!dev_tr) {
                of_node_put(dev_tr);
                return -ENODEV;
        }

	priv->spec_ops->dev_ops = platform_get_drvdata(pdev_tr);
	BUG_ON(!priv->spec_ops->dev_ops);
	
	priv->spec_ops->dev_ops->probe_init(pdev);
	return ret; 
}

static void hssigldrv_rdexecute(struct platform_device *pdev,
			 	u32 offset, u8 acc_type, u32 *val)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	void __iomem *base = priv->sscsr;

	switch(acc_type) {
	case WORD_ACCESS:
		*val = csrrd32(base, offset);
		break;
	case BYTE_ACCESS:
		*val = csrrd8(base, offset);
		break;
	default:
		dev_err(&pdev->dev, "Unsupported read access type\n");
	}
}

static void hssigldrv_wrexecute(struct platform_device *pdev,
			 	u32 offset, u8 acc_type, u32 val)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	void __iomem *base = priv->sscsr;

	switch(acc_type) {
	case WORD_ACCESS:
		csrwr32(val, base, offset);
		break;
	case BYTE_ACCESS:
		csrwr8(val, base, offset);
		break;
	default:
		dev_err(&pdev->dev, "Unsupported write access type\n");
	}
}

int hssigldrv_get_set_csr(struct platform_device *pdev, u32 cmd,
			  void *csr_data,
			  bool rd)
{
	u32 addr_offset = 0;
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	struct get_set_csr_data *data = (struct get_set_csr_data *)csr_data;

	addr_offset = priv->spec_ops->dev_ops->get_addr_offset(pdev, 
							       data->ch,
							       data->reg_type,
							       data->offs);
	switch(cmd) {
	case SAL_GET_CSR:
		hssigldrv_rdexecute(pdev, addr_offset, data->word,
				    &data->data);
		break;
	case SAL_SET_CSR:
		hssigldrv_wrexecute(pdev, addr_offset, data->word,
				    data->data);
		break;
	default:
		dev_err(&pdev->dev, "Bad command type other than get/set csr\n");
	}
	
	return 0;
}

hssi_eth_port_sts hssigldrv_get_ethport_status(struct platform_device *pdev,
					       int port)
{
        hssi_eth_port_sts port_sts;
        struct hssiss_private *priv = platform_get_drvdata(pdev);
        
	port_sts = priv->spec_ops->dev_ops->get_ethport_status(pdev, port);

	return port_sts;
}

int hssigldrv_get_mtu(struct platform_device *pdev, enum hssiss_salcmd cmd,
	    	      void *priv_data)
{
	(void)cmd;
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	priv->spec_ops->dev_ops->get_mtu(pdev, priv_data);

	return 0;
}

int hssigldrv_set_mtu(struct platform_device *pdev, enum hssiss_salcmd cmd,
                      void *priv_data)
{
	(void)cmd;
        struct hssiss_private *priv = platform_get_drvdata(pdev);

        priv->spec_ops->dev_ops->set_mtu(pdev, priv_data);

        return 0;
}

int hssigldrv_lock_mac_stats(struct platform_device *pdev,
			     int port)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	priv->spec_ops->dev_ops->freeze_mac_stats(pdev, port);

	return 0;
}


int hssigldrv_unlock_mac_stats(struct platform_device *pdev,
			       int port)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	priv->spec_ops->dev_ops->defreeze_mac_stats(pdev, port);

	return 0;
}

int hssigldrv_read_mac_stats(struct platform_device *pdev,
			     enum hssiss_salcmd cmd, void *data)

{
	(void)cmd;
	u64 val = 0;
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	val = priv->spec_ops->dev_ops->read_mac_stat(pdev, (struct read_mac_stat_data*)data);
	
	return val;	
}

int hssigldrv_reset_mac_stat(struct platform_device *pdev, enum hssiss_salcmd cmd,
                             void *data)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	(void)cmd;
	
	priv->spec_ops->dev_ops->reset_mac_stat(pdev,
						((struct reset_mac_stat_data *)data)->port);

	return 0;
}

int hssigldrv_enable_disable_loopback(struct platform_device *pdev, u32 cmdid,
				      void *lb_data)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	struct set_loopback_data *data = (struct set_loopback_data *)lb_data;
	int ret;

	if (cmdid == SAL_ENABLE_LOOPBACK)
		 ret = priv->spec_ops->dev_ops->enable_loopback(pdev, data->type, data->port);
	else
		ret = priv->spec_ops->dev_ops->disable_loopback(pdev, data->type, data->port);

	return ret;
}

void hssigldrv_reset_port(struct platform_device *pdev, int port)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	priv->spec_ops->dev_ops->reset_port(pdev, port);
}
