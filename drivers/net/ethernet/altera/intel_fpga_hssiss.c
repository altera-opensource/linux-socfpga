// SPDX-License-Identifier: GPL-2.0
/* Intel FPGA HSSI SS driver
 * Copyright (C) 2022 Intel Corporation. All rights reserved
 *
	 * Contributors:
	 *   Subhransu S. Prusty
 *
 */
#define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>

#include <linux/of_device.h>
#include <linux/platform_device.h>

#include "altera_utils.h"
#include "intel_fpga_hssiss.h"

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
	{SAL_RSVD, 0xA, "SAL_RSVD"},
	{SAL_NCSI_GET_LINK_STS, 0xB, "SAL_NCSI_GET_LINK_STS"},
	{SAL_FW_VERSION, 0xFF, "SAL_FW_VERSION"},
};

#define ADDR_OFFSET_INCR 0x200000
static u32 etile_addrmap[] =
	{0x0200000, 0x0204000, 0x0240000, 0x0250000, 0x0260000, 0x0261000, 0x0262000};
static u32 ftile_addrmap[] =
	{0x0200000, 0, 0x0300000, 0, 0, 0x0261000, 0};

static int read_poll_timeout(void __iomem *base,
		unsigned int csr_addroff, u32 offs, u32 sel, bool atomic)
{
	u32 val;
	unsigned long timeout, start;

	start = jiffies;
	timeout = start + usecs_to_jiffies(FW_ACK_POLL_TIMEOUT_US);
	do {
		if (atomic)
			udelay(2 * FW_ACK_POLL_INTERVAL_US);
		else
			usleep_range(FW_ACK_POLL_INTERVAL_US, 2 * FW_ACK_POLL_INTERVAL_US);
		val = csrrd32_withoffset(base, csr_addroff, offs);
		if ((val & sel) == sel)
			return val;

	} while(time_before(jiffies, timeout));

	return -ETIME;
}

static int hssiss_mailbox_reg_set(void __iomem *base,
		unsigned int csr_addroff, u32 offs, u32 setval, bool atomic)
{
	u32 val;
	unsigned long timeout, start;

	start = jiffies;
	timeout = start + usecs_to_jiffies(FW_ACK_POLL_TIMEOUT_US);
	csrwr32_withoffset(setval, base, csr_addroff, offs);
	do {
		if (atomic)
			udelay(2 * FW_ACK_POLL_INTERVAL_US);
		else
			usleep_range(FW_ACK_POLL_INTERVAL_US, 2 * FW_ACK_POLL_INTERVAL_US);

		val = csrrd32_withoffset(base, csr_addroff, offs);
		if (val == setval)
			return 0;
	} while(time_before(jiffies, timeout));

	return -ETIME;
}

static int hssiss_sal_execute(struct platform_device *pdev, u32 ctrl_addr,
			u32 cmd_sts, u32 *val, bool atomic)
{
	int ret;
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	unsigned int csr_addroff = priv->csr_addroff;
	void __iomem *base = priv->sscsr;

	if (atomic)
		spin_lock(&priv->sal_spinlock);
	else
		mutex_lock(&priv->sal_mutex);

	if ((cmd_sts & HSSI_SAL_CMDSTS_WR) && val) {
		ret = hssiss_mailbox_reg_set(base, csr_addroff,
					HSSISS_CSR_WR_DATA, *val, atomic);
		if (ret < 0)
			goto unlock;
	}

	csrwr32_withoffset(ctrl_addr, base, csr_addroff, HSSISS_CSR_CTRLADDR);
	csrwr32_withoffset(cmd_sts, base, csr_addroff, HSSISS_CSR_CMDSTS);
	ret = read_poll_timeout(base, csr_addroff,
			HSSISS_CSR_CMDSTS, HSSI_SAL_CMDSTS_ACK, atomic);

	/*
	 * WA: f-tile loopback enable sets the error bit.
	 * Ignore for now if both ack and error set.
	 */
	if (priv->hssi_err_wa && (ret & HSSI_SAL_CMDSTS_ACK) &&
			(ret & HSSI_SAL_CMDSTS_ERR)) {
		ret = 0;
		goto unlock;
	}

	if (ret > 0) {

		if (ret & HSSI_SAL_CMDSTS_BUSY) {
			dev_err(&pdev->dev, "FW hung. Reset required. ret: %x\n", ret);
			ret = -EBUSY;
		} else if (ret & HSSI_SAL_CMDSTS_ERR) {
			dev_err(&pdev->dev, "Command execution error. ret: %x\n", ret);
			ret = -EINVAL;
		} else
			ret = 0;
	}

	if (!ret && (cmd_sts & HSSI_SAL_CMDSTS_RD) && val) {
		*val = csrrd32_withoffset(priv->sscsr,
				priv->csr_addroff, HSSISS_CSR_RD_DATA);
	}

unlock:
	if (atomic)
		spin_unlock(&priv->sal_spinlock);
	else
		mutex_unlock(&priv->sal_mutex);

	return ret;
}

static int enable_disable_loopback(struct platform_device *pdev, u32 cmdid,
				void *data, bool atomic)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	unsigned int port = (*(unsigned int *)data);
	u32 ctrl_addr = 0;
	u32 cmd_sts = 0;

	if ((port > priv->feature_list.part.num_ports) ||
			(!test_reg_bits(priv->feature_list.part.port_enable_mask, port, 1)))
		return -EIO;

	ctrl_addr |= cmdid;
	ctrl_addr |= port << HSSI_SAL_CTRLADDR_PORT_SHIFT;
	cmd_sts |= HSSI_SAL_CMDSTS_WR;

	return hssiss_sal_execute(pdev, ctrl_addr, cmd_sts, NULL, atomic);
}

/*
 * Calculate ctrl address field for get/set csr.
 */
static u32 hssiss_make_get_set_csr_addr(u32 base, u32 offs, bool word)
{
	if (word)
		return ((base + (offs * 4))/4); /* registers at word offset */
	else
		return ((base + offs)/4);	/* registers at byte offset */
}

static int get_set_csr(struct platform_device *pdev, u32 cmd, void *csr_data,
			bool rd, bool atomic)
{
	u32 ctrl_addr = 0;
	u32 cmd_sts = 0;
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	int ret;
	u32 addr;
	struct get_set_csr_data *data = (struct get_set_csr_data *)csr_data;
	u32 base;

	if (priv->ver == HSSISS_FTILE) {
		base = (data->ch * ADDR_OFFSET_INCR) +
				ftile_addrmap[data->reg_type];
	} else {
		base = (data->ch * ADDR_OFFSET_INCR) +
				etile_addrmap[data->reg_type];
	}

	addr = hssiss_make_get_set_csr_addr(base, data->offs, data->word);

	ctrl_addr |= addr << HSSI_SAL_CTRLADDR_ADDRBITS_SHIFT;
	ctrl_addr |= cmd;
	cmd_sts |= (data->offs % 4) << HSSI_SAL_CMDSTS_REG_OFFS_SHIFT;

	cmd_sts |= rd ? HSSI_SAL_CMDSTS_RD : HSSI_SAL_CMDSTS_WR;

	ret = hssiss_sal_execute(pdev, ctrl_addr, cmd_sts, &data->data, atomic);

	return ret;
}

static int test_nios(struct platform_device *pdev, u32 cmd, bool atomic)
{
	u32 ctrl_addr = 0;
	u32 cmd_sts = 0;

	ctrl_addr |= cmd;
	cmd_sts |= HSSI_SAL_CMDSTS_WR;

	dev_info(&pdev->dev, "ctrl_addr: %x, cmd_sts: %x\n", ctrl_addr, cmd_sts);

	return hssiss_sal_execute(pdev, ctrl_addr, cmd_sts, NULL, atomic);
}

static int get_set_dr_profile(struct platform_device *pdev, u32 cmd, void *dr_data,
				bool rd, bool atomic)
{
	int ret;
	u32 ctrl_addr = 0;
	u32 cmd_sts = 0;
	struct get_set_dr_data *data = (struct get_set_dr_data *)dr_data;
	u32 val = 0;

	ctrl_addr |= data->port << HSSI_SAL_CTRLADDR_PORT_SHIFT;
	ctrl_addr |= cmd;

	if (rd) {
		cmd_sts |= HSSI_SAL_CMDSTS_RD;
	} else {
		cmd_sts |= HSSI_SAL_CMDSTS_WR;
		val |= data->profile & HSSI_DR_PROFILE_MASK;
		val |= (data->dr_grp << DR_GRP_INDEX) & HSSI_DR_GRP_MASK;
	}

	ret = hssiss_sal_execute(pdev, ctrl_addr, cmd_sts, &val, atomic);

	if (rd && (ret == 0)) {
		data->dr_grp = (val & HSSI_DR_GRP_MASK) >> DR_GRP_INDEX;
		data->profile = val & HSSI_DR_PROFILE_MASK;
	}

	return ret;
}

static int reset_mac_stat(struct platform_device *pdev, u32 cmd,
			void *priv_data, bool atomic)
{
	int ret;
	u32 ctrl_addr = 0;
	u32 cmd_sts = 0;
	struct reset_mac_stat_data *data = (struct reset_mac_stat_data *)priv_data;

	ctrl_addr |= data->port << HSSI_SAL_CTRLADDR_PORT_SHIFT;

	if (data->tx)
		ctrl_addr |= HSSI_SAL_RESET_MAC_STAT_TX;

	if (data->rx)
		ctrl_addr |= HSSI_SAL_RESET_MAC_STAT_RX;

	ctrl_addr |= cmd;
	cmd_sts |= HSSI_SAL_CMDSTS_WR;

	ret = hssiss_sal_execute(pdev, ctrl_addr, cmd_sts, NULL, atomic);

	return ret;
}

static int get_mtu(struct platform_device *pdev, u32 cmd,
		void *priv_data, bool atomic)
{
	int ret;
	u32 ctrl_addr = 0;
	u32 cmd_sts = 0;
	u32 val;
	struct get_mtu_data *data = (struct get_mtu_data*)priv_data;

	ctrl_addr |= data->port << HSSI_SAL_CTRLADDR_PORT_SHIFT;
	ctrl_addr |= cmd;
	cmd_sts |= HSSI_SAL_CMDSTS_RD;

	ret = hssiss_sal_execute(pdev, ctrl_addr, cmd_sts, &val, atomic);
	if (ret == 0) {
		data->max_tx_frame_size = val & GENMASK(31,16) >> 16;
		data->max_rx_frame_size = val & GENMASK(15,0);
	}

	return ret;
}

static int read_mac_stat(struct platform_device *pdev, u32 cmd,
			void *priv_data, bool atomic)
{
	int ret;
	u32 ctrl_addr = 0;
	u32 cmd_sts = 0;
	struct read_mac_stat_data *data =
		(struct read_mac_stat_data *)priv_data;

	ctrl_addr |= data->port_data << HSSI_SAL_CTRLADDR_PORT_SHIFT;
	ctrl_addr |= cmd;
	ctrl_addr |= data->type << HSSI_SAL_CTRLADDR_COUNTER_SHIFT;
	ctrl_addr |= data->lsb << HSSI_SAL_CTRLADDR_LSB_SHIFT;
	cmd_sts |= HSSI_SAL_CMDSTS_RD;

	ret = hssiss_sal_execute(pdev, ctrl_addr, cmd_sts, &data->port_data, atomic);

	return ret;
}

static int ncsi_link_status(struct platform_device *pdev, u32 cmd,
			void *priv_data, bool atomic)
{
	int ret;
	u32 ctrl_addr = 0;
	u32 cmd_sts = 0;
	union ncsi_link_status_data *data =
		(union ncsi_link_status_data *)priv_data;

	ctrl_addr |= data->full << HSSI_SAL_CTRLADDR_PORT_SHIFT;
	ctrl_addr |= cmd;
	cmd_sts |= HSSI_SAL_CMDSTS_RD;

	ret = hssiss_sal_execute(pdev, ctrl_addr, cmd_sts, &data->full, atomic);

	return ret;
}

static int get_fw_version(struct platform_device *pdev, u32 cmd,
			void *priv_data, bool atomic)
{
	int ret;
	u32 ctrl_addr = 0;
	u32 cmd_sts = 0;
	u32 *data =(u32 *)priv_data;

	ctrl_addr |= cmd;
	cmd_sts |= HSSI_SAL_CMDSTS_RD;

	ret = hssiss_sal_execute(pdev, ctrl_addr, cmd_sts, data, atomic);

	return ret;
}

static int execute_sal_cmd(struct platform_device *pdev,
		enum hssiss_salcmd cmd, void *data, bool atomic)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	int ret = 0;

	if (atomic_read(&priv->coldrst_inprogress))
		return -EBUSY;

	switch(cmd) {
	case SAL_NOP:
		ret = test_nios(pdev, salcmd_name[cmd].cmdid, atomic);
		break;

	case SAL_GET_HSSI_PROFILE:
		ret = get_set_dr_profile(pdev, salcmd_name[cmd].cmdid, data, true, atomic);
		break;

	case SAL_SET_HSSI_PROFILE:
		ret = get_set_dr_profile(pdev, salcmd_name[cmd].cmdid, data, false, atomic);
		break;

	case SAL_RSVD:
		break;

	case SAL_READ_MAC_STAT:
		ret = read_mac_stat(pdev, salcmd_name[cmd].cmdid, data, atomic);
		break;

	case SAL_GET_MTU:
		ret = get_mtu(pdev, salcmd_name[cmd].cmdid, data, atomic);
		break;

	case SAL_RESET_MAC_STAT:
		ret = reset_mac_stat(pdev, salcmd_name[cmd].cmdid, data, atomic);
		break;

	case SAL_NCSI_GET_LINK_STS:
		ret = ncsi_link_status(pdev, salcmd_name[cmd].cmdid, data, atomic);
		break;

	case SAL_FW_VERSION:
		ret = get_fw_version(pdev, salcmd_name[cmd].cmdid, data, atomic);
		break;

	case SAL_SET_CSR:
		ret = get_set_csr(pdev, salcmd_name[cmd].cmdid, data, false, atomic);
		break;

	case SAL_GET_CSR:
		ret = get_set_csr(pdev, salcmd_name[cmd].cmdid, data, true, atomic);
		break;

	case SAL_DISABLE_LOOPBACK:
	case SAL_ENABLE_LOOPBACK:
		ret = enable_disable_loopback(pdev, salcmd_name[cmd].cmdid, data, atomic);
		break;
	default:
		dev_err(&pdev->dev, "Invalid command, cmd: %x\n", cmd);
		return -EINVAL;
	};

	return ret;
}

int hssiss_execute_sal_cmd_atomic(struct platform_device *pdev,
		enum hssiss_salcmd cmd, void *data)
{
	return execute_sal_cmd(pdev, cmd, data, true);
}

int hssiss_execute_sal_cmd(struct platform_device *pdev,
		enum hssiss_salcmd cmd, void *data)
{
	return execute_sal_cmd(pdev, cmd, data, false);
}

hssi_eth_port_sts hssiss_get_ethport_status(struct platform_device *pdev, int port)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	hssi_eth_port_sts port_sts;

	if (priv->ver == HSSISS_FTILE) {
		port_sts.full = csrrd32(priv->sscsr,
				(HSSISS_CSR_ETH_PORT_STS_FTILE + port * 4));
	} else {
		port_sts.full = csrrd32_withoffset(priv->sscsr,
				priv->csr_addroff,
				(HSSISS_CSR_ETH_PORT_STS + port * 4));
	}
	return port_sts;
}

/* Enable/disable hotplug */
void hssiss_hotplug_enable(struct platform_device *pdev, bool enable)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	u32 val;

	val = csrrd32_withoffset(priv->sscsr, priv->csr_addroff,
				HSSISS_CSR_HOTPLUG_DBG_CTRL);

	if (enable)
		val &= ~0x1;
	else
		val |= 0x1;

	csrwr32_withoffset(val, priv->sscsr, priv->csr_addroff,
				HSSISS_CSR_HOTPLUG_DBG_CTRL);
}

int hssiss_cold_rst(struct platform_device *pdev)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	void __iomem *base = priv->sscsr;
	unsigned int csr_addroff = priv->csr_addroff;
	struct cold_reset_register *cold_rst  = &priv->cold_rst_reg;

	if (!mutex_trylock(&priv->coldrst_mutex))
		return -EBUSY;

	atomic_set(&priv->coldrst_inprogress, 1);
	csrwr32_withoffset((1 << cold_rst->rst_bit),
			base, csr_addroff, cold_rst->ofs);

	read_poll_timeout(base, csr_addroff,
			cold_rst->ofs, (1 << cold_rst->rst_ack), false);
	atomic_set(&priv->coldrst_inprogress, 0);
	mutex_unlock(&priv->coldrst_mutex);

	return 0;
}

enum hssiss_hip_type hssiss_get_hip_type(struct platform_device *pdev)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	if (!priv)
		return -EINVAL;

	return priv->ver;
}

/* Utility functions for hssi driver */
static unsigned int get_dfh_feature_rev(void __iomem *addr)
{
	u32 val;

	val = csrrd32(addr, feature_offs(dfh_lo));

	return ((val & HSSISS_DFHLO_DFHV0_FEA_REV_MASK) >> HSSISS_DFHLO_DFHV0_FEA_REV_SHIFT);
}

static unsigned int get_csr_addroff(void __iomem *base,
			unsigned int feature_rev)
{
	u32 val;

	if (feature_rev == 0x0 || feature_rev == 0x1) {
		return 0;
	} else { //feature_rev == 0x2
		val = csrrd32(base, feature_offs(feature_csr_addr_lsb));
		return (((val & HSSISS_FEATURE_CSR_ADDR_MASK) >>
				HSSISS_FEATURE_CSR_ADDR_SHIFT) - 0x8);
	}
}

static ssize_t hssiss_hotplug_disable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	u32 val;

	val = csrrd32_withoffset(priv->sscsr,
				priv->csr_addroff, HSSISS_CSR_HOTPLUG_DBG_STS);

	return sprintf(buf, "%u\n", ((val >> HSSI_HOTPLUG_DBG_STS_DISABLE_SHIFT) & 1));
}

static ssize_t hssiss_hotplug_disable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	struct platform_device *pdev = to_platform_device(dev);
	int disable;

	sscanf(buf, "%d", &disable);

	hssiss_hotplug_enable(pdev, (disable? false:true));

	return len;
}

static ssize_t hssiss_err_wa_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", priv->hssi_err_wa);
}

static ssize_t hssiss_err_wa_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	sscanf(buf, "%d", &(priv->hssi_err_wa));

	return len;
}

static DEVICE_ATTR(hssi_hotplug_disable, 0644, hssiss_hotplug_disable_show, hssiss_hotplug_disable_store);
static DEVICE_ATTR(hssi_err_wa, 0644, hssiss_err_wa_show, hssiss_err_wa_store);

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

static const struct of_device_id hssiss_ids[] = {
	{ .compatible = "intel, hssiss-1.0"},
	{},
};
MODULE_DEVICE_TABLE(of, hssiss_ids);

static int hssiss_probe(struct platform_device *pdev)
{
	struct hssiss_private *priv;
	struct resource *sscsr;
	const struct of_device_id *of_id = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct fwnode_handle *cold_rst;
	unsigned int version;
	const char *rm;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	of_id = of_match_device(hssiss_ids, &pdev->dev);
	if (!of_id)
		return -ENODEV;

	priv->dev = &pdev->dev;

	/* HSSI SS CSR address space */
	ret = request_and_map(pdev, "sscsr", &sscsr,		/* TODO */
			      (void __iomem **)&priv->sscsr);
	if (ret)
		return -EIO;

	mutex_init(&priv->sal_mutex);
	mutex_init(&priv->coldrst_mutex);
	spin_lock_init(&priv->sal_spinlock);

	ret = of_property_read_string(np, "reset-mode", &rm);
	if (ret == 0) {
		if (!strcasecmp(rm, "reg")) {
			cold_rst = fwnode_get_named_child_node(pdev->dev.fwnode, "cold-reset");
			if (cold_rst) {
				fwnode_property_read_u32(cold_rst, "ofs", &priv->cold_rst_reg.ofs);
				fwnode_property_read_u32(cold_rst, "rst-bit", &priv->cold_rst_reg.rst_bit);
				fwnode_property_read_u32(cold_rst, "rst-ack", &priv->cold_rst_reg.rst_ack);

			}
		}
	}

	priv->dfh_feature_rev = get_dfh_feature_rev(priv->sscsr);
	priv->csr_addroff = get_csr_addroff(priv->sscsr, priv->dfh_feature_rev);
	dev_info(&pdev->dev, "csr_addr offset: %x, dfh_feature_rev: %x\n",
			priv->csr_addroff, priv->dfh_feature_rev);

	priv->feature_list.full =
		csrrd32_withoffset(priv->sscsr,
			priv->csr_addroff, HSSISS_CSR_COMMON_FEATURE_LIST);
	version = csrrd32_withoffset(priv->sscsr, priv->csr_addroff, HSSISS_CSR_VER);
	priv->ver = (version & HSSISS_VER_CSR_ADDR_MASK) >>
				HSSISS_VER_CSR_ADDR_SHIFT;

	platform_set_drvdata(pdev, priv);

#ifdef CONFIG_DEBUG_FS
	priv->dbgfs = hssiss_dbgfs_init(pdev);
	if (!priv->dbgfs)
		dev_warn(&pdev->dev, "Error creating dbgfs");
#endif
	dev_info(&pdev->dev, "Probe done\n");

	return 0;
}

static int hssiss_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);

	/* TODO: REMOVE dbgfs */

	return 0;
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

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("Intel HSSI SS driver");
MODULE_LICENSE("GPL v2");
