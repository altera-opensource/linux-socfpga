// SPDX-License-Identifier: GPL-2.0
/* Altera FPGA HSSI platform driver
 * Copyright (C) 2022, 2025 Altera Corporation. All rights reserved
 *
 * Contributors:
 *   Subhransu S. Prusty
 *   Preetam Narayan
 *
 */
 #define DEBUG

 #include <linux/kernel.h>
 #include <linux/module.h>
 #include <linux/delay.h>

 #include <linux/mod_devicetable.h>
 #include <linux/of.h>
 #include <linux/of_device.h>
 #include <linux/platform_device.h>

 #include "altera_utils.h"
 #include "intel_fpga_hssiss.h"
 #include "intel_fpga_hssi_driver.h"
 #include "intel_fpga_hssigl_driver.h"
 #include "intel_fpga_hssi_tile_ops.h"

 #define INTEL_FPGA_HSSISS_NAME "intel_fpga_hssiss"

static struct hssiss_salcmd_to_name salcmd_name[] = {
	{SAL_NOP, 0x0, "SAL_NOP"},
	{SAL_GET_HSSI_PROFILE, 0x1, "SAL_GET_HSSI_PROFILE"},
	{SAL_SET_HSSI_PROFILE, 0x2, "SAL_SET_HSSI_PROFILE"},
	{SAL_READ_MAC_STAT, 0x3, "SAL_READ_MAC_STAT"},
	{SAL_GET_MTU, 0x4, "SAL_GET_MTU"},
	{SAL_SET_CSR, 0x5, "SAL_SET_CSR"},
	{SAL_GET_CSR, 0x6, "SAL_GET_CSR"},
	{SAL_ENABLE_LOOPBACK, 0x7, "SAL_ENABLE_LOOPBACK"},
	{SAL_DISABLE_LOOPBACK, 0x8, "SAL_DISABLE_LOOPBACK"},
	{SAL_RESET_MAC_STAT, 0x9, "SAL_RESET_MAC_STAT"},
	{SAL_SET_MTU, 0xA, "SAL_SET_MTU"},
	{SAL_NCSI_GET_LINK_STS, 0xB, "SAL_NCSI_GET_LINK_STS"},
	{SAL_FW_VERSION, 0xFF, "SAL_FW_VERSION"},
};

static int hssiss_get_set_csr(struct platform_device *pdev, u32 cmd, void *csr_data,
			      bool rd)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	return priv->spec_ops->ops->get_set_csr(pdev, cmd, csr_data, rd);
}

static int hssiss_get_fw_version(struct platform_device *pdev, u32 cmd,
				 void *priv_data)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	return priv->spec_ops->ops->get_fw_version(pdev, cmd, priv_data);
}

static int hssiss_ncsi_link_status(struct platform_device *pdev, u32 cmd,
				   void *priv_data)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	return priv->spec_ops->ops->get_ncsi_link_status(pdev, cmd, priv_data);
}

static int hssiss_get_mtu(struct platform_device *pdev, u32 cmd,
			  void *priv_data)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	return priv->spec_ops->ops->get_mtu(pdev, cmd, priv_data);
}

int hssiss_set_mtu(struct platform_device *pdev, u32 cmd,
		   void *priv_data)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	if (priv->spec_ops->ops->set_mtu)
		return priv->spec_ops->ops->set_mtu(pdev, cmd, priv_data);

	return -EOPNOTSUPP;
}

static int hssiss_read_mac_stat(struct platform_device *pdev, u32 cmd,
				void *priv_data)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	return priv->spec_ops->ops->read_mac_stat(pdev, cmd, priv_data);
}

static int hssiss_get_set_dr_profile(struct platform_device *pdev, u32 cmd, void *dr_data,
				     bool rd)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	return priv->spec_ops->ops->get_set_dr_profile(pdev, cmd, dr_data, rd);
}

hssi_eth_port_sts hssiss_get_ethport_status(struct platform_device *pdev, int port)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	return priv->spec_ops->ops->get_ethport_status(pdev, port);
}

int hssiss_set_ethport_status(struct platform_device *pdev, int port, u32 data)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	return priv->spec_ops->ops->set_ethport_status(pdev, port, data);
}

hssi_eth_port_attr hssiss_get_ethport_attr(struct platform_device *pdev, int port)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	return priv->spec_ops->ops->get_ethport_attr(pdev, port);
}

void hssiss_hotplug_enable(struct platform_device *pdev, bool enable)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	return priv->spec_ops->ops->hotplug_enable(pdev, enable);
}

int hssiss_hotplug_disable_status(struct platform_device *pdev)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	return priv->spec_ops->ops->hotplug_disable_status(pdev);
}

int hssiss_cold_rst(struct platform_device *pdev)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	return priv->spec_ops->ops->perform_cold_rst(pdev);
}

static int hssiss_reset_mac_stat(struct platform_device *pdev, u32 cmd,
				 void *priv_data)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	return priv->spec_ops->ops->reset_mac_stat(pdev, cmd, priv_data);
}

static int hssiss_test_nios(struct platform_device *pdev, u32 cmd)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	return priv->spec_ops->ops->test_nios(pdev, cmd);
}

int hssiss_enable_disable_loopback(struct platform_device *pdev, u32 cmdid,
				   void *data)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	return priv->spec_ops->ops->enable_disable_loopback(pdev, cmdid, data);
}

int hssiss_lock_stats(struct platform_device *pdev, int port)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	return priv->spec_ops->ops->lock_mac_stats(pdev, port);
}

int hssiss_unlock_stats(struct platform_device *pdev, int port)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	return priv->spec_ops->ops->unlock_mac_stats(pdev, port);
}

void hssiss_reset_port(struct platform_device *pdev, int port)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	if (priv->spec_ops->ops->reset_port)
		return priv->spec_ops->ops->reset_port(pdev, port);
}

int hssiss_anlt_update(struct platform_device *pdev, int port, bool enable_anlt)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	if (priv->spec_ops->ops->anlt_update)
		return priv->spec_ops->ops->anlt_update(pdev, port, enable_anlt);
	return 0;
}

u32 hssiss_anlt_get_status(struct platform_device *pdev, int port)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	if (priv->spec_ops->ops->anlt_get_status)
		return priv->spec_ops->ops->anlt_get_status(pdev, port);
	return 0;
}

u32 hssiss_anlt_get_cfg(struct platform_device *pdev, int port)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	if (priv->spec_ops->ops->anlt_get_cfg)
		return priv->spec_ops->ops->anlt_get_cfg(pdev, port);
	return 0;
}

u32 hssiss_anlt_get_ext_status(struct platform_device *pdev, int port, int *an_status)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	if (priv->spec_ops->ops->anlt_get_ext_status)
		return priv->spec_ops->ops->anlt_get_ext_status(pdev, port, an_status);
	return 0;
}

u32 hssiss_usrspace_pkterr_cnt(struct platform_device *pdev, u32 addr_offs)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	if (priv->spec_ops->ops->usrspace_pkterr_cnt)
		return priv->spec_ops->ops->usrspace_pkterr_cnt(pdev, addr_offs);

	return ~0;
}

void hssiss_usrspace_pkterr_logic_en(struct platform_device *pdev, u32 addr_offs, bool enable)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	if (priv->spec_ops->ops->usrspace_pkterr_logic_en)
		priv->spec_ops->ops->usrspace_pkterr_logic_en(pdev, addr_offs, enable);
}

void hssiss_usrspace_pkterr_cnt_rst(struct platform_device *pdev, int port)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	if (priv->spec_ops->ops->usrspace_pkterr_cnt_rst)
		priv->spec_ops->ops->usrspace_pkterr_cnt_rst(pdev, port);
}

static int execute_sal_cmd(struct platform_device *pdev,
			   enum hssiss_salcmd cmd, void *data)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	int ret = 0;

	if (atomic_read(&priv->coldrst_inprogress))
		return -EBUSY;

	switch (cmd) {
	case SAL_NOP:
		ret = hssiss_test_nios(pdev, salcmd_name[cmd].cmdid);
		break;

	case SAL_GET_HSSI_PROFILE:
		ret = hssiss_get_set_dr_profile(pdev, salcmd_name[cmd].cmdid, data, true);
		break;

	case SAL_SET_HSSI_PROFILE:
		ret = hssiss_get_set_dr_profile(pdev, salcmd_name[cmd].cmdid, data, false);
		break;

	case SAL_RSVD:
		break;

	case SAL_READ_MAC_STAT:
		ret = hssiss_read_mac_stat(pdev, salcmd_name[cmd].cmdid, data);
		break;

	case SAL_GET_MTU:
		ret = hssiss_get_mtu(pdev, salcmd_name[cmd].cmdid, data);
		break;

	case SAL_RESET_MAC_STAT:
		ret = hssiss_reset_mac_stat(pdev, salcmd_name[cmd].cmdid, data);
		break;

	case SAL_NCSI_GET_LINK_STS:
		ret = hssiss_ncsi_link_status(pdev, salcmd_name[cmd].cmdid, data);
		break;

	case SAL_FW_VERSION:
		ret = hssiss_get_fw_version(pdev, salcmd_name[cmd].cmdid, data);
		break;

	case SAL_SET_CSR:
		ret = hssiss_get_set_csr(pdev, salcmd_name[cmd].cmdid, data, false);
		break;

	case SAL_GET_CSR:
		ret = hssiss_get_set_csr(pdev, salcmd_name[cmd].cmdid, data, true);
		break;

	case SAL_DISABLE_LOOPBACK:
	case SAL_ENABLE_LOOPBACK:
		ret = hssiss_enable_disable_loopback(pdev, salcmd_name[cmd].cmdid, data);
		break;
	default:
		dev_err(&pdev->dev, "Invalid command, cmd: %x\n", cmd);
		return -EINVAL;
	};

	return ret;
}

int hssiss_execute_sal_cmd(struct platform_device *pdev,
			   enum hssiss_salcmd cmd, void *data)
{
	return execute_sal_cmd(pdev, cmd, data);
}

static ssize_t hssi_hotplug_disable_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);

	return sprintf(buf, "%u\n", hssiss_hotplug_disable_status(pdev));
}

static ssize_t hssi_hotplug_disable_store(struct device *dev,
					  struct device_attribute *attr,
					    const char *buf, size_t len)
{
	int ret;
	int disable;
	struct platform_device *pdev = to_platform_device(dev);

	ret = kstrtouint(buf, 10, &disable);
	if (ret)
		return -EINVAL;

	hssiss_hotplug_enable(pdev, (disable ? false : true));

	return len;
}

static ssize_t hssi_err_wa_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", priv->hssi_err_wa);
}

static ssize_t hssi_err_wa_store(struct device *dev,
				 struct device_attribute *attr, const char *buf, size_t len)
{
	int ret;
	struct platform_device *pdev = to_platform_device(dev);
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	ret = kstrtouint(buf, 10, &priv->hssi_err_wa);
	if (ret)
		return -EINVAL;

	return len;
}

static DEVICE_ATTR(hssi_hotplug_disable, 0644, hssi_hotplug_disable_show,
		   hssi_hotplug_disable_store);
static DEVICE_ATTR_RW(hssi_err_wa);

static struct attribute *hssiss_sysfs_attrs[] = {
	&dev_attr_hssi_hotplug_disable.attr,
	&dev_attr_hssi_err_wa.attr,
	NULL
};

static const struct attribute_group hssiss_attr_group = {
	.attrs = hssiss_sysfs_attrs,
};

const struct attribute_group *hssiss_attr_groups[] = {
	&hssiss_attr_group,
	NULL
};

static struct hssi_gen_ops hssi_gen_ops = {
	.get_set_csr = hssidrv_get_set_csr,
	.get_fw_version = hssidrv_get_fw_version,
	.get_ncsi_link_status = hssidrv_ncsi_link_status,
	.get_mtu = hssidrv_get_mtu,
	.set_mtu = hssidrv_set_mtu,
	.read_mac_stat = hssidrv_read_mac_stat,
	.get_set_dr_profile = hssidrv_get_set_dr_profile,
	.get_ethport_status = hssidrv_get_ethport_status,
	.set_ethport_status = hssidrv_set_ethport_status,
	.get_ethport_attr = hssidrv_get_ethport_attr,
	.enable_disable_loopback = hssidrv_enable_disable_loopback,
	.hotplug_enable = hssidrv_hotplug_enable,
	.perform_cold_rst = hssidrv_cold_rst,
	.hotplug_disable_status = hssidrv_hotplug_disable_status,
	.reset_mac_stat = hssidrv_reset_mac_stat,
	.test_nios = hssidrv_test_nios,
	.probe_init = hssidrv_probe_init,
	.anlt_get_status = hssidrv_anlt_get_status,
	.anlt_get_cfg = hssidrv_anlt_get_cfg,
	.anlt_get_ext_status = hssidrv_anlt_get_ext_status,
	.anlt_update = hssidrv_anlt_update,
};

struct hssi_spec_ops hssi_xtile_data = {
	.ops = &hssi_gen_ops,
};

static struct hssi_gen_ops nonhssi_gen_ops = {
	.probe_init = hssigldrv_probe_init,
	.get_set_csr = hssigldrv_get_set_csr,
	.get_ethport_status = hssigldrv_get_ethport_status,
	.reset_mac_stat = hssigldrv_reset_mac_stat,
	.read_mac_stat = hssigldrv_read_mac_stats,
	.get_mtu = hssigldrv_get_mtu,
	.set_mtu = hssigldrv_set_mtu,
	.lock_mac_stats = hssigldrv_lock_mac_stats,
	.unlock_mac_stats = hssigldrv_unlock_mac_stats,
	.enable_disable_loopback = hssigldrv_enable_disable_loopback,
	.reset_port = hssigldrv_reset_port,
	.usrspace_pkterr_cnt = hssigldrv_err_cnt_read,
	.usrspace_pkterr_cnt_rst = hssigldrv_err_cnt_reset,
	.usrspace_pkterr_logic_en = hssigldrv_err_cnt_enable,
};

struct hssi_spec_ops nonhssi_xtile_data = {
	.ops = &nonhssi_gen_ops,
};

static const struct of_device_id hssiss_ids[] = {
	{ .compatible = "intel, hssiss-1.0",
	  .data = &hssi_xtile_data},
	{ .compatible = "intel, hssiss-2.0",
	  .data = &nonhssi_xtile_data},
	{},
};

MODULE_DEVICE_TABLE(of, hssiss_ids);

static int hssiss_probe(struct platform_device *pdev)
{
	struct hssiss_private *priv;
	struct resource *sscsr;
	const struct of_device_id *of_id = NULL;
	const struct hssi_spec_ops *op_ptr;
	struct device_node *np = pdev->dev.of_node;
	struct fwnode_handle *cold_rst;
	const char *rm;
	int ret = 0;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	of_id = of_match_device(hssiss_ids, &pdev->dev);
	if (!of_id)
		return -ENODEV;

	op_ptr = of_device_get_match_data(&pdev->dev);
	if (!op_ptr) {
		dev_err(&pdev->dev, "No matching data field found\n");
		return -ENODEV;
	}

	priv->spec_ops = (struct hssi_spec_ops *)op_ptr;

	priv->dev = &pdev->dev;

	/* HSSI SS CSR address space */
	ret = request_and_map(pdev, "sscsr", &sscsr,		/* TODO */
			      (void __iomem **)&priv->sscsr);
	if (ret)
		return -EIO;

	ret = of_property_read_string(np, "reset-mode", &rm);
	if (ret == 0) {
		if (!strcasecmp(rm, "reg")) {
			cold_rst = fwnode_get_named_child_node(pdev->dev.fwnode, "cold-reset");
			if (cold_rst) {
				fwnode_property_read_u32(cold_rst, "ofs",
							 &priv->cold_rst_reg.ofs);
				fwnode_property_read_u32(cold_rst, "rst-bit",
							 &priv->cold_rst_reg.rst_bit);
				fwnode_property_read_u32(cold_rst, "rst-ack",
							 &priv->cold_rst_reg.rst_ack);
			}
		}
	}

	platform_set_drvdata(pdev, priv);

	ret = priv->spec_ops->ops->probe_init(pdev);

 #ifdef CONFIG_DEBUG_FS
	priv->dbgfs = hssiss_dbgfs_init(pdev);
	if (!priv->dbgfs)
		dev_warn(&pdev->dev, "Error creating dbgfs");
 #endif
	dev_info(&pdev->dev, "Probe done\n");

	return ret;
}

static void hssiss_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);
}

static struct platform_driver hssiss_driver = {
	.probe		= hssiss_probe,
	.remove		= hssiss_remove,
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name	= INTEL_FPGA_HSSISS_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = hssiss_ids,
		.dev_groups = hssiss_attr_groups,
	},
};

module_platform_driver(hssiss_driver);

MODULE_AUTHOR("Altera Corporation");
MODULE_DESCRIPTION("Altera HSSI SS interface driver");
MODULE_LICENSE("GPL v2");
