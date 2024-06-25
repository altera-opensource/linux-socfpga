// SPDX-License-Identifier: GPL-2.0
/* Intel FPGA HSSI SS driver
 * Copyright (C) 2022, 2024 Intel Corporation. All rights reserved
 *
 * Contributors:
 *   Subhransu S. Prusty
 *   Preetam Narayan
 *
 */
#define DEBUG

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include "altera_utils.h"
#include "intel_fpga_hssiss.h"
#include "intel_fpga_hssi_driver.h"

#define ADDR_OFFSET_INCR 0x200000
static u32 etile_addrmap[] = {
	0x0200000, 0x0204000, 0x0240000, 0x0250000, 0x0260000, 0x0261000, 0x0262000};
static u32 ftile_addrmap[] = {
	0x0200000, 0, 0x0300000, 0, 0, 0x0261000, 0};

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

	} while (time_before(jiffies, timeout));

	return -ETIME;
}

static int hssidrv_mailbox_reg_set(void __iomem *base,
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
	} while (time_before(jiffies, timeout));

	return -ETIME;
}

static int hssidrv_sal_execute(struct platform_device *pdev, u32 ctrl_addr,
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
		ret = hssidrv_mailbox_reg_set(base, csr_addroff,
					      HSSISS_CSR_WR_DATA, *val, atomic);
		if (ret < 0)
			goto unlock;
	}

	csrwr32_withoffset(ctrl_addr, base, csr_addroff, HSSISS_CSR_CTRLADDR);
	csrwr32_withoffset(cmd_sts, base, csr_addroff, HSSISS_CSR_CMDSTS);
	ret = read_poll_timeout(base, csr_addroff,
				HSSISS_CSR_CMDSTS, HSSI_SAL_CMDSTS_ACK, atomic);

	/* WA: f-tile loopback enable sets the error bit */
	/* Ignore for now if both ack and error set.     */
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
		} else {
			ret = 0;
		}
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

int hssidrv_enable_disable_loopback(struct platform_device *pdev, u32 cmdid,
				    void *data, bool atomic)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	unsigned int port = (*(unsigned int *)data);
	u32 ctrl_addr = 0;
	u32 cmd_sts = 0;

	if (!test_reg_bits(priv->feature_list.part.port_enable_mask, port, 1))
		return -EIO;

	ctrl_addr |= cmdid;
	ctrl_addr |= port << HSSI_SAL_CTRLADDR_PORT_SHIFT;
	cmd_sts |= HSSI_SAL_CMDSTS_WR;

	return hssidrv_sal_execute(pdev, ctrl_addr, cmd_sts, NULL, atomic);
}

/* Calculate ctrl address field for get/set csr */
static u32 hssidrv_make_get_set_csr_addr(u32 base, u32 offs, bool word)
{
	if (word)
		return ((base + (offs * 4)) / 4); /* registers at word offset */
	else
		return ((base + offs) / 4);	/* registers at byte offset */
}

int hssidrv_get_set_csr(struct platform_device *pdev, u32 cmd, void *csr_data,
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

	addr = hssidrv_make_get_set_csr_addr(base, data->offs, data->word);

	ctrl_addr |= addr << HSSI_SAL_CTRLADDR_ADDRBITS_SHIFT;
	ctrl_addr |= cmd;
	cmd_sts |= (data->offs % 4) << HSSI_SAL_CMDSTS_REG_OFFS_SHIFT;

	cmd_sts |= rd ? HSSI_SAL_CMDSTS_RD : HSSI_SAL_CMDSTS_WR;

	ret = hssidrv_sal_execute(pdev, ctrl_addr, cmd_sts, &data->data, atomic);

	return ret;
}

int hssidrv_test_nios(struct platform_device *pdev, u32 cmd, bool atomic)
{
	u32 ctrl_addr = 0;
	u32 cmd_sts = 0;

	ctrl_addr |= cmd;
	cmd_sts |= HSSI_SAL_CMDSTS_WR;

	dev_info(&pdev->dev, "ctrl_addr: %x, cmd_sts: %x\n", ctrl_addr, cmd_sts);

	return hssidrv_sal_execute(pdev, ctrl_addr, cmd_sts, NULL, atomic);
}

int hssidrv_get_set_dr_profile(struct platform_device *pdev, u32 cmd, void *dr_data,
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

	ret = hssidrv_sal_execute(pdev, ctrl_addr, cmd_sts, &val, atomic);

	if (rd && (ret == 0)) {
		data->dr_grp = (val & HSSI_DR_GRP_MASK) >> DR_GRP_INDEX;
		data->profile = val & HSSI_DR_PROFILE_MASK;
	}

	return ret;
}

int hssidrv_reset_mac_stat(struct platform_device *pdev, u32 cmd,
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

	ret = hssidrv_sal_execute(pdev, ctrl_addr, cmd_sts, NULL, atomic);

	return ret;
}

int hssidrv_get_mtu(struct platform_device *pdev, u32 cmd,
		    void *priv_data, bool atomic)
{
	int ret;
	u32 ctrl_addr = 0;
	u32 cmd_sts = 0;
	u32 val;
	struct get_mtu_data *data = (struct get_mtu_data *)priv_data;

	ctrl_addr |= data->port << HSSI_SAL_CTRLADDR_PORT_SHIFT;
	ctrl_addr |= cmd;
	cmd_sts |= HSSI_SAL_CMDSTS_RD;

	ret = hssidrv_sal_execute(pdev, ctrl_addr, cmd_sts, &val, atomic);
	if (ret == 0) {
		data->max_tx_frame_size = val & GENMASK(31, 16) >> 16;
		data->max_rx_frame_size = val & GENMASK(15, 0);
	}

	return ret;
}

int hssidrv_read_mac_stat(struct platform_device *pdev, u32 cmd,
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

	ret = hssidrv_sal_execute(pdev, ctrl_addr, cmd_sts, &data->port_data, atomic);

	return ret;
}

int hssidrv_ncsi_link_status(struct platform_device *pdev, u32 cmd,
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

	ret = hssidrv_sal_execute(pdev, ctrl_addr, cmd_sts, &data->full, atomic);

	return ret;
}

int hssidrv_get_fw_version(struct platform_device *pdev, u32 cmd,
			   void *priv_data, bool atomic)
{
	int ret;
	u32 ctrl_addr = 0;
	u32 cmd_sts = 0;
	u32 *data = (u32 *)priv_data;

	ctrl_addr |= cmd;
	cmd_sts |= HSSI_SAL_CMDSTS_RD;

	ret = hssidrv_sal_execute(pdev, ctrl_addr, cmd_sts, data, atomic);

	return ret;
}

hssi_eth_port_sts hssidrv_get_ethport_status(struct platform_device *pdev, int port)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	hssi_eth_port_sts port_sts;

	port_sts.full = 0;

	/* E-tile and FGT in F-tile */
	if (port >= 0 && port < 16) {
		port_sts.full = csrrd32_withoffset(priv->sscsr,
						   priv->csr_addroff,
				(HSSISS_CSR_ETH_PORT_STS + port * 4));
		return port_sts;
	}

	/* For F-tile FHT only */
	if (priv->ver == HSSISS_FTILE && (port >= 16 && port < 20)) {
		port_sts.full = csrrd32(priv->sscsr,
					(HSSISS_CSR_ETH_PORT_STS_FHT + port * 4));
	}

	return port_sts;
}

int hssidrv_set_ethport_status(struct platform_device *pdev, int port, u32 data)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	/* E-tile and FGT in F-tile */
	if (port >= 0 && port < 16) {
		csrwr32_withoffset(data, priv->sscsr,
				   priv->csr_addroff,
				   (HSSISS_CSR_ETH_PORT_STS + port * 4));
	}

	/* For F-tile FHT only */
	if (priv->ver == HSSISS_FTILE && (port >= 16 && port < 20)) {
		csrwr32_withoffset(data, priv->sscsr,
				   priv->csr_addroff,
				   (HSSISS_CSR_ETH_PORT_STS_FHT + port * 4));
	}

	return 0;
}

hssi_eth_port_attr hssidrv_get_ethport_attr(struct platform_device *pdev, int port)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	hssi_eth_port_attr port_attr;

	port_attr.full = 0;
	/* E-tile and FGT in F-tile */
	if (port >= 0 && port < 16) {
		port_attr.full = csrrd32_withoffset(priv->sscsr,
						    priv->csr_addroff,
				(HSSISS_CSR_INTER_ATTRIB_PORT + port * 4));
		return port_attr;
	}

	/* For F-tile FHT only */
	if (priv->ver == HSSISS_FTILE && (port >= 16 && port < 20)) {
		port_attr.full = csrrd32(priv->sscsr,
					 (HSSISS_CSR_INTER_ATTRIB_PORT_FHT + port * 4));
	}

	return port_attr;
}

/* Enable/disable hotplug */
void hssidrv_hotplug_enable(struct platform_device *pdev, bool enable)
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

/* Hotplug status */
int hssidrv_hotplug_disable_status(struct platform_device *pdev)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	u32 val;

	val = csrrd32_withoffset(priv->sscsr,
				 priv->csr_addroff, HSSISS_CSR_HOTPLUG_DBG_STS);
	return ((val >> HSSI_HOTPLUG_DBG_STS_DISABLE_SHIFT) & 1);
}

int hssidrv_cold_rst(struct platform_device *pdev)
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

void hssidrv_probe_init(struct platform_device *pdev)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	unsigned int version;

	mutex_init(&priv->sal_mutex);
	mutex_init(&priv->coldrst_mutex);
	spin_lock_init(&priv->sal_spinlock);

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
}
