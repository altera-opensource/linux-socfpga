// SPDX-License-Identifier: GPL-2.0
/* Altera FPGA HSSI SS debugfs
 * Copyright (C) 2022, 2025 Altera Corporation. All rights reserved
 *
 * Contributors:
 *   Subhransu S. Prusty
 *   Preetam Narayan
 *
 */
 #include <linux/slab.h>
 #include <linux/platform_device.h>
 #include <linux/debugfs.h>
 #include "altera_utils.h"
 #include "intel_fpga_hssiss.h"
 #include "intel_fpga_hssi_driver.h"
 #include "altera_fpga_anlt.h"

struct hssiss_dbg_read_data {
	u32 dr_grp; /* get_hss_profile */
	u32 profile; /* get_hss_profile */
	u32 data; /* port_data for mac_stat, data for link_status, fw_version and csr */
	u32 max_tx_frame_size;
	u32 max_rx_frame_size;
};

struct anlt_dbg_read_data {
	int port;
	u32 hssiss_csr_anlt_seq_cfg;
	u32 hssiss_csr_an_cfg_1;
	u32 hssiss_csr_an_cfg_2;
	u32 hssiss_csr_an_cfg_3;
	u32 hssiss_csr_an_cfg_4;
	u32 hssiss_csr_an_cfg_5;
	u32 hssiss_csr_an_cfg_6;
	u32 hssiss_csr_an_cfg_8;
	u32 hssiss_csr_lt_cfg_1;
	u32 hssiss_csr_lt_cfg_2;
	u32 hssiss_csr_anlt_seq_status;
	u32 hssiss_csr_an_status;
	u32 hssiss_csr_an_status_1;
	u32 hssiss_csr_an_status_2;
	u32 hssiss_csr_an_status_3;
	u32 hssiss_csr_an_status_4;
	u32 hssiss_csr_an_status_6;
	u32 hssiss_csr_lt_status_1;
	u32 hssiss_csr_kr_debug_0;
	u32 hssiss_csr_kr_debug_1;
	u32 hssiss_csr_kr_debug_2;
	u32 hssiss_csr_kr_debug_3;
	u32 hssiss_csr_kr_debug_4;
	u32 hssiss_csr_kr_debug_5;
	u32 hssiss_csr_kr_debug_6;
	u32 hssiss_csr_kr_debug_7;
	u32 hssiss_csr_kr_debug_8;
	u32 hssiss_csr_kr_debug_9;
	u32 hssiss_csr_kr_debug_10;
	u32 hssiss_csr_kr_debug_11;
	u32 hssiss_csr_kr_debug_12;
	u32 hssiss_csr_kr_debug_13;
	u32 hssiss_csr_kr_debug_14;
	u32 hssiss_csr_kr_debug_15;
};

struct hssiss_dbg {
	struct platform_device *pdev;
	struct dentry *dbgfs;
	enum hssiss_salcmd sal_cmd;
	struct hssiss_dbg_read_data read;
	struct anlt_dbg_read_data anlt_data;
};

/*
 * hssiss_dbgfs_csr_read() - hssiss debugfs-node csr read callback
 */
static ssize_t hssiss_dbgfs_csr_read(struct file *filep, char __user *ubuf,
				     size_t count, loff_t *offp)
{
	struct hssiss_dbg *d = filep->private_data;
	char buf[10];
	int size;

	size = snprintf(buf, sizeof(buf), "%x\n", d->read.data);

	return simple_read_from_buffer(ubuf, count, offp, buf, size);
}

/*
 * hssiss_dbgfs_csr_write() - hssiss debugfs-node csr write callback
 * for read:
 *	echo "ch type offset word" > hssi_reg
 * for write:
 *	echo "ch type offset word data" > hssi_reg
 *
 * word: 1 for word read/write, 0 for byte read/write
 */
static ssize_t hssiss_dbgfs_csr_write(struct file *filep, const char __user *ubuf,
				      size_t count, loff_t *offp)
{
	struct hssiss_dbg *d = filep->private_data;
	struct platform_device *pdev = d->pdev;
	struct get_set_csr_data data;
	char *buf;
	int word, type, ch, ret;
	u32 offset, val;

	/* Copy data from User-space */
	buf = kmalloc(count + 1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = simple_write_to_buffer(buf, count, offp, ubuf, count);
	if (ret < 0)
		goto free_buf;
	buf[count] = 0;

	/* Parse the values */
	ret = sscanf(buf, "%d %d %x %d %x", &ch, &type, &offset, &word, &val);

	if (ret < 4) {
		ret = -EINVAL;
		goto free_buf;
	}

	data.ch = ch;
	data.reg_type = type;
	data.offs = offset;
	data.word = word ? true : false;
	data.data = val;

	if (ret == 4) {
		ret = hssiss_execute_sal_cmd(pdev, SAL_GET_CSR, &data);
		if (ret == 0)
			d->read.data = data.data;
	} else {
		ret = hssiss_execute_sal_cmd(pdev, SAL_SET_CSR, &data);
	}

free_buf:
	kfree(buf);
	return (ret < 0 ? ret : count);
}

/*
 * hssiss_dbgfs_sal_read() - hssiss debugfs-node SAL read callback
 * Note: Except get/set csr. Use get/set csr dbgfs to read csr registers.
 */
static ssize_t hssiss_dbgfs_sal_read(struct file *filep, char __user *ubuf,
				     size_t count, loff_t *offp)
{
	struct hssiss_dbg *d = filep->private_data;
	char buf[100];
	int size;

	switch (d->sal_cmd) {
	case SAL_GET_HSSI_PROFILE:
		size = scnprintf(buf, sizeof(buf),
				 "dr_grp: %x profile: %x",
				d->read.dr_grp, d->read.profile);
		break;
	case SAL_READ_MAC_STAT:
		size = scnprintf(buf, sizeof(buf), "%x", d->read.data);
		break;
	case SAL_GET_MTU:
		size = scnprintf(buf, sizeof(buf),
				 "max_tx_frame_size: %x max_rx_frame_size:%x",
				d->read.max_tx_frame_size,
				d->read.max_rx_frame_size);
		break;
	case SAL_NCSI_GET_LINK_STS:
		size = scnprintf(buf, sizeof(buf), "%x", d->read.data);
		break;
	case SAL_FW_VERSION:
		size = scnprintf(buf, sizeof(buf), "%x", d->read.data);
		break;
	default:
		size = scnprintf(buf, sizeof(buf), "No command in progress\n");
		break;
	}

	return simple_read_from_buffer(ubuf, count, offp, buf, size);
}

/*
 * hssiss_dbgfs_sal_write() - hssiss debugfs-node sal write callback
 * Note: Except get/set csr. Use get/set csr dbgfs to read csr registers.
 */
static ssize_t hssiss_dbgfs_sal_write(struct file *filep, const char __user *ubuf,
				      size_t count, loff_t *offp)
{
	struct hssiss_dbg *d = filep->private_data;
	struct platform_device *pdev = d->pdev;
	char *buf;
	u32 cmd;
	int ret;

	/* Copy data from User-space */
	buf = kmalloc(count + 1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = simple_write_to_buffer(buf, count, offp, ubuf, count);
	if (ret < 0)
		goto free_buf;
	buf[count] = 0;

	/* Parse SAL command */
	//ret = sscanf(buf, "%x", &cmd);
	ret = kstrtouint(buf, 16, &cmd);
	if (!ret) {
		ret = -EINVAL;
		goto free_buf;
	}

	d->sal_cmd = cmd;

	/* Parse and prepare data for command */
	switch (cmd) {
	case SAL_GET_HSSI_PROFILE:
	case SAL_SET_HSSI_PROFILE:
	{
		struct get_set_dr_data data;

		ret = sscanf(buf, "%x %x %x %u", &cmd, &data.dr_grp, &data.profile, &data.port);
		if (ret != 4) {
			ret = -EINVAL;
			goto free_buf;
		}

		ret = hssiss_execute_sal_cmd(pdev, cmd, &data);

		d->read.dr_grp = data.dr_grp;
		d->read.profile = data.profile;

		break;
	}
	case SAL_READ_MAC_STAT:
	{
		struct read_mac_stat_data data;
		int lsb;

		ret = sscanf(buf, "%x %x %x %d", &cmd, &data.port_data, &data.type, &lsb);
		if (ret != 4) {
			ret = -EINVAL;
			goto free_buf;
		}

		data.lsb = (lsb) ? true : false;

		ret = hssiss_execute_sal_cmd(pdev, cmd, &data);

		d->read.data = data.port_data;

		break;
	}
	case SAL_GET_MTU:
	{
		struct get_mtu_data data;

		ret = sscanf(buf, "%x %u %hu %hu",
			     &cmd, &data.port, &data.max_tx_frame_size,
				&data.max_rx_frame_size);
		if (ret != 4) {
			ret = -EINVAL;
			goto free_buf;
		}

		ret = hssiss_execute_sal_cmd(pdev, cmd, &data);
		d->read.max_tx_frame_size = data.max_tx_frame_size;
		d->read.max_rx_frame_size = data.max_rx_frame_size;

		break;
	}
	case SAL_RESET_MAC_STAT:
	{
		struct reset_mac_stat_data data;
		int tx, rx;

		ret = sscanf(buf, "%x %u %d %d", &cmd, &data.port, &tx, &rx);
		if (ret != 4) {
			ret = -EINVAL;
			goto free_buf;
		}

		data.tx = tx ? true : false;
		data.rx = rx ? true : false;

		ret = hssiss_execute_sal_cmd(pdev, cmd, &data);

		break;
	}
	case SAL_NCSI_GET_LINK_STS:
	{
		union ncsi_link_status_data data;

		ret = sscanf(buf, "%x %x", &cmd, &data.full);
		if (ret != 2) {
			ret = -EINVAL;
			goto free_buf;
		}

		ret = hssiss_execute_sal_cmd(pdev, cmd, &data);
		d->read.data = data.full;

		break;
	}
	case SAL_FW_VERSION:
	{
		u32 data;

		ret = hssiss_execute_sal_cmd(pdev, cmd, &data);
		d->read.data = data;

		break;
	}
	case SAL_DISABLE_LOOPBACK:
	case SAL_ENABLE_LOOPBACK:
	{
		u32 data = 0;

		ret = sscanf(buf, "%x %x", &cmd, &data);
		if (ret != 2) {
			ret = -EINVAL;
			goto free_buf;
		}
		ret = hssiss_execute_sal_cmd(pdev, cmd, &data);
		break;
	}
	default:
		ret = -EINVAL;
		break;
	}

free_buf:
	kfree(buf);
	return (ret < 0 ? ret : count);
}

 #define BUF_SIZE	PAGE_SIZE
/*
 * hssiss_dbgfs_readme_read() - hssiss debugfs-node readme read callback
 */
static ssize_t hssiss_dbgfs_readme_read(struct file *filep, char __user *ubuf,
					size_t count, loff_t *offp)
{
	char *buf;
	int ret;

	buf = kzalloc(BUF_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = scnprintf(buf, BUF_SIZE, "get_csr: to read byte data:\n");
	ret += scnprintf(buf + ret, BUF_SIZE - ret,
			 "\techo \"ch type offset 0\" > hssi_reg\n");
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "\tcat hssi_reg\n");
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "get_csr: to read word data:\n");
	ret += scnprintf(buf + ret, BUF_SIZE - ret,
			 "\techo \"ch type offset 1\" > hssi_reg\n");
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "\tcat hssi_reg\n");
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "set_csr: to write byte data:\n");
	ret += scnprintf(buf + ret, BUF_SIZE - ret,
			 "\techo \"ch type offset 0 data\" > hssi_reg\n");
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "set_csr: to write word data:\n");
	ret += scnprintf(buf + ret, BUF_SIZE - ret,
			 "\techo \"ch type offset 1 data\" > hssi_reg\n");
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "Execute sal command:\n");
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "\techo \"cmd x y z\" > sal\n");
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "\tcmd: SAL command\n");
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "\tx, y, z: SAL command specific data\n");
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "\tcat sal\n");
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "Execute direct SAL command:\n");
	ret += scnprintf(buf + ret, BUF_SIZE - ret,
			 "\techo <ctrladdr reg_data> > ctrladdr\n");
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "\tfor write: echo <wr reg_data> > wr\n");
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "\techo <cmdsts reg_data> > cmdsts\n");
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "\tto check ack or err: cat cmdsts\n");
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "\tto read data: cat rd\n");
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "Execute direct register access:\n");
	ret += scnprintf(buf + ret, BUF_SIZE - ret,
			 "\tfor wr: echo <baseaddr offset direct val> > direct_reg\n");
	ret += scnprintf(buf + ret, BUF_SIZE - ret,
			 "\tfor rd: echo <baseaddr offset direct> > direct_reg\n");
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "\tcat direct_reg\n");

	ret = simple_read_from_buffer(ubuf, count, offp, buf, ret);

	kfree(buf);
	return ret;
}

/*
 * hssiss_dbgfs_dumpcsr_read() - hssiss debugfs-node dumpcsr read callback
 */
static ssize_t hssiss_dbgfs_dumpcsr_read(struct file *filep, char __user *ubuf,
					 size_t count, loff_t *offp)
{
	struct hssiss_dbg *d = filep->private_data;
	struct platform_device *pdev = d->pdev;
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	unsigned int csr_addroff = priv->csr_addroff;
	void __iomem *base = priv->sscsr;
	char *buf;
	int ret;
	int i;
	u32 val;

	buf = kzalloc(BUF_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = scnprintf(buf, BUF_SIZE, "Dumping device feature registers\n");
	for (i = 0; i < 10; i++)
		ret += scnprintf(buf + ret, BUF_SIZE - ret, "\t%x: %x\n",
				(i * 4), csrrd32(base, (i * 4)));

	ret += scnprintf(buf + ret, BUF_SIZE - ret, "Dumping other CSR registers\n");

	val = csrrd32_withoffset(base, csr_addroff, HSSISS_CSR_VER);
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_VER: %x\n", val);

	val = csrrd32_withoffset(base, csr_addroff, HSSISS_CSR_COMMON_FEATURE_LIST);
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_COMMON_FEATURE_LIST: %x\n", val);

	ret += scnprintf(buf + ret, BUF_SIZE - ret, "Dumping port attributes\n");
	for (i = 0; i < 15; i++) { /* E-tile and FGT in F-tile */
		val = csrrd32_withoffset(base, csr_addroff,
					 HSSISS_CSR_INTER_ATTRIB_PORT + (i * 4));
		ret += scnprintf(buf + ret, BUF_SIZE - ret, "\t%x: %x\n", i, val);
	}

	if (priv->ver == HSSISS_FTILE) { /* For F-tile FHT only */
		for (i = 16; i < 20; i++) {
			val = csrrd32(base, HSSISS_CSR_INTER_ATTRIB_PORT_FHT + (i * 4));
			ret += scnprintf(buf + ret, BUF_SIZE - ret, "\t%x: %x\n", i, val);
		}
	}

	val = csrrd32_withoffset(base, csr_addroff, HSSISS_CSR_CMDSTS);
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_CMDSTS: %x\n", val);

	val = csrrd32_withoffset(base, csr_addroff, HSSISS_CSR_CTRLADDR);
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_CTRLADDR: %x\n", val);

	val = csrrd32_withoffset(base, csr_addroff, HSSISS_CSR_RD_DATA);
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_RD_DATA: %x\n", val);

	val = csrrd32_withoffset(base, csr_addroff, HSSISS_CSR_WR_DATA);
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_WR_DATA: %x\n", val);

	val = csrrd32_withoffset(base, csr_addroff, HSSISS_CSR_GMII_TX_LATENCY);
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_GMII_TX_LATENCY: %x\n", val);

	val = csrrd32_withoffset(base, csr_addroff, HSSISS_CSR_GMII_RX_LATENCY);
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_GMII_RX_LATENCY: %x\n", val);

	ret += scnprintf(buf + ret, BUF_SIZE - ret, "Dumping port status\n");

	for (i = 0; i < 15; i++) { /* E-tile and FGT in F-tile */
		val = csrrd32_withoffset(base, csr_addroff,
					 HSSISS_CSR_ETH_PORT_STS + (i * 4));
		ret += scnprintf(buf + ret, BUF_SIZE - ret, "\t%x: %x\n", i, val);
	}

	if (priv->ver == HSSISS_FTILE) { /* For F-tile FHT only */
		for (i = 16; i < 20; i++) {
			val = csrrd32(base, HSSISS_CSR_ETH_PORT_STS_FHT + (i * 4));
			ret += scnprintf(buf + ret, BUF_SIZE - ret, "\t%x: %x\n", i, val);
		}
	}

	val = csrrd32_withoffset(base, csr_addroff, HSSISS_CSR_TSE_CTRL);
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_TSE_CTRL: %x\n", val);

	val = csrrd32_withoffset(base, csr_addroff, HSSISS_CSR_DBG_CTRL);
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_DBG_CTRL: %x\n", val);

	val = csrrd32_withoffset(base, csr_addroff, HSSISS_CSR_HOTPLUG_DBG_CTRL);
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_HOTPLUG_DBG_CTRL: %x\n", val);

	val = csrrd32_withoffset(base, csr_addroff, HSSISS_CSR_HOTPLUG_DBG_STS);
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_HOTPLUG_DBG_STS: %x\n", val);

	ret = simple_read_from_buffer(ubuf, count, offp, buf, ret);

	kfree(buf);
	return ret;
}

static ssize_t hssiss_dbgfs_ctrladdr_read(struct file *filep, char __user *ubuf,
					  size_t count, loff_t *offp)
{
	struct hssiss_dbg *d = filep->private_data;
	struct platform_device *pdev = d->pdev;
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	char buf[10];
	u32 val;
	int size;

	val = csrrd32_withoffset(priv->sscsr, priv->csr_addroff,
				 HSSISS_CSR_CTRLADDR);

	size = snprintf(buf, sizeof(buf), "%x\n", val);

	return simple_read_from_buffer(ubuf, count, offp, buf, size);
}

static ssize_t hssiss_dbgfs_ctrladdr_write(struct file *filep, const char __user *ubuf,
					   size_t count, loff_t *offp)
{
	struct hssiss_dbg *d = filep->private_data;
	struct platform_device *pdev = d->pdev;
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	u32 val;
	char *buf;
	int ret;

	/* Copy data from User-space */
	buf = kmalloc(count + 1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = simple_write_to_buffer(buf, count, offp, ubuf, count);
	if (ret < 0) {
		kfree(buf);
		return -EIO;
	}
	buf[count] = 0;

	/* Parse the values */
	//ret = sscanf(buf, "%x", &val);
	ret = kstrtouint(buf, 16, &val);
	kfree(buf);
	if (ret < 1)
		return -EINVAL;

	csrwr32_withoffset(val, priv->sscsr, priv->csr_addroff,
			   HSSISS_CSR_CTRLADDR);

	return count;
}

static ssize_t hssiss_dbgfs_cmdsts_read(struct file *filep, char __user *ubuf,
					size_t count, loff_t *offp)
{
	struct hssiss_dbg *d = filep->private_data;
	struct platform_device *pdev = d->pdev;
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	char buf[10];
	u32 val;
	int size;

	val = csrrd32_withoffset(priv->sscsr, priv->csr_addroff,
				 HSSISS_CSR_CMDSTS);

	size = snprintf(buf, sizeof(buf), "%x\n", val);

	return simple_read_from_buffer(ubuf, count, offp, buf, size);
}

static ssize_t hssiss_dbgfs_cmdsts_write(struct file *filep, const char __user *ubuf,
					 size_t count, loff_t *offp)
{
	struct hssiss_dbg *d = filep->private_data;
	struct platform_device *pdev = d->pdev;
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	u32 val;
	char *buf;
	int ret;

	/* Copy data from User-space */
	buf = kmalloc(count + 1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = simple_write_to_buffer(buf, count, offp, ubuf, count);
	if (ret < 0) {
		kfree(buf);
		return -EIO;
	}
	buf[count] = 0;

	/* Parse the values */
	//ret = sscanf(buf, "%x", &val);
	ret = kstrtouint(buf, 16, &val);
	kfree(buf);
	if (ret < 1)
		return -EINVAL;

	csrwr32_withoffset(val, priv->sscsr, priv->csr_addroff,
			   HSSISS_CSR_CMDSTS);

	return count;
}

static ssize_t hssiss_dbgfs_wr_read(struct file *filep, char __user *ubuf,
				    size_t count, loff_t *offp)
{
	struct hssiss_dbg *d = filep->private_data;
	struct platform_device *pdev = d->pdev;
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	char buf[10];
	u32 val;
	int size;

	val = csrrd32_withoffset(priv->sscsr, priv->csr_addroff,
				 HSSISS_CSR_WR_DATA);

	size = snprintf(buf, sizeof(buf), "%x\n", val);

	return simple_read_from_buffer(ubuf, count, offp, buf, size);
}

static ssize_t hssiss_dbgfs_wr_write(struct file *filep, const char __user *ubuf,
				     size_t count, loff_t *offp)
{
	struct hssiss_dbg *d = filep->private_data;
	struct platform_device *pdev = d->pdev;
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	u32 val;
	char *buf;
	int ret;

	/* Copy data from User-space */
	buf = kmalloc(count + 1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = simple_write_to_buffer(buf, count, offp, ubuf, count);
	if (ret < 0) {
		kfree(buf);
		return -EIO;
	}
	buf[count] = 0;

	/* Parse the values */
	//ret = sscanf(buf, "%x", &val);
	ret = kstrtouint(buf, 16, &val);
	kfree(buf);
	if (ret < 1)
		return -EINVAL;

	csrwr32_withoffset(val, priv->sscsr, priv->csr_addroff,
			   HSSISS_CSR_WR_DATA);

	return count;
}

static ssize_t hssiss_dbgfs_rd_read(struct file *filep, char __user *ubuf,
				    size_t count, loff_t *offp)
{
	struct hssiss_dbg *d = filep->private_data;
	struct platform_device *pdev = d->pdev;
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	char buf[10];
	u32 val;
	int size;

	val = csrrd32_withoffset(priv->sscsr, priv->csr_addroff,
				 HSSISS_CSR_RD_DATA);

	size = snprintf(buf, sizeof(buf), "%x\n", val);

	return simple_read_from_buffer(ubuf, count, offp, buf, size);
}

static ssize_t hssiss_dbgfs_rd_write(struct file *filep, const char __user *ubuf,
				     size_t count, loff_t *offp)
{
	struct hssiss_dbg *d = filep->private_data;
	struct platform_device *pdev = d->pdev;
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	u32 val;
	char *buf;
	int ret;

	/* Copy data from User-space */
	buf = kmalloc(count + 1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = simple_write_to_buffer(buf, count, offp, ubuf, count);
	if (ret < 0) {
		kfree(buf);
		return -EIO;
	}
	buf[count] = 0;

	/* Parse the values */
	//ret = sscanf(buf, "%x", &val);
	ret = kstrtouint(buf, 16, &val);
	kfree(buf);
	if (ret < 1)
		return -EINVAL;

	csrwr32_withoffset(val, priv->sscsr, priv->csr_addroff,
			   HSSISS_CSR_RD_DATA);

	return count;
}

static ssize_t hssiss_dbgfs_direct_reg_read(struct file *filep, char __user *ubuf,
					    size_t count, loff_t *offp)
{
	struct hssiss_dbg *d = filep->private_data;
	char buf[10];
	int size;

	size = snprintf(buf, sizeof(buf), "%x\n", d->read.data);

	return simple_read_from_buffer(ubuf, count, offp, buf, size);
}

static ssize_t hssiss_dbgfs_direct_reg_write(struct file *filep, const char __user *ubuf,
					     size_t count, loff_t *offp)
{
	struct hssiss_dbg *d = filep->private_data;
	struct platform_device *pdev = d->pdev;
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	u32 base, offset, val;
	char *buf;
	int ret, direct;

	/* Copy data from User-space */
	buf = kmalloc(count + 1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = simple_write_to_buffer(buf, count, offp, ubuf, count);
	if (ret < 0) {
		kfree(buf);
		return -EIO;
	}
	buf[count] = 0;

	/* Parse the values */
	ret = sscanf(buf, "%x %x %d %x", &base, &offset, &direct, &val);
	kfree(buf);
	if (ret < 3)
		return -EINVAL;

	if (ret == 4) {
		if (direct) {
			csrwr32_withoffset(val, priv->sscsr + base,
					   0, offset);
		} else {
			csrwr32_withoffset(val, priv->sscsr + base,
					   priv->csr_addroff, offset);
		}
	} else {
		if (direct) {
			d->read.data = csrrd32_withoffset(priv->sscsr + base,
							  0, offset);
		} else {
			d->read.data = csrrd32_withoffset(priv->sscsr + base,
							  priv->csr_addroff, offset);
		}
	}

	return count;
}

static ssize_t anlt_dbgfs_dump_by_port_read(struct file *filep, char __user *ubuf,
					    size_t count, loff_t *offp)
{
	struct hssiss_dbg *d = filep->private_data;
	char *buf;
	int ret;
	u32 val;

	buf = kzalloc(BUF_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = scnprintf(buf, BUF_SIZE, "Dumping AN/LT registers for port : %d\n\n",
			d->anlt_data.port);

	ret += scnprintf(buf + ret, BUF_SIZE - ret, "ANLT sequencer config registers:\n");

	val = d->anlt_data.hssiss_csr_anlt_seq_cfg;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_ANLT_SEQ_CFG: %x\n\n", val);

	ret += scnprintf(buf + ret, BUF_SIZE - ret, "AN config registers:\n");
	val = d->anlt_data.hssiss_csr_an_cfg_1;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_AN_CFG_1: %x\n", val);
	val = d->anlt_data.hssiss_csr_an_cfg_2;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_AN_CFG_2: %x\n", val);
	val = d->anlt_data.hssiss_csr_an_cfg_3;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_AN_CFG_3: %x\n", val);
	val = d->anlt_data.hssiss_csr_an_cfg_4;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_AN_CFG_4: %x\n", val);
	val = d->anlt_data.hssiss_csr_an_cfg_5;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_AN_CFG_5: %x\n", val);
	val = d->anlt_data.hssiss_csr_an_cfg_6;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_AN_CFG_6: %x\n", val);
	val = d->anlt_data.hssiss_csr_an_cfg_8;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_AN_CFG_8: %x\n\n", val);

	ret += scnprintf(buf + ret, BUF_SIZE - ret, "LT config registers:\n");
	val = d->anlt_data.hssiss_csr_lt_cfg_1;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_LT_CFG_1: %x\n", val);
	val = d->anlt_data.hssiss_csr_lt_cfg_2;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_LT_CFG_2: %x\n\n", val);

	/* ANLT Sequencer (Complete State machine) */
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "ANLT sequencer status registers:\n");
	val = d->anlt_data.hssiss_csr_anlt_seq_status;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_ANLT_SEQ_STATUS: %x\n\n", val);
	/* Autonegotiation registers */
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "AN status registers:\n");
	val = d->anlt_data.hssiss_csr_an_status;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_AN_STATUS: %x\n", val);
	val = d->anlt_data.hssiss_csr_an_status_1;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_AN_STATUS_1: %x\n", val);
	val = d->anlt_data.hssiss_csr_an_status_2;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_AN_STATUS_2: %x\n", val);
	val = d->anlt_data.hssiss_csr_an_status_3;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_AN_STATUS_3: %x\n", val);
	val = d->anlt_data.hssiss_csr_an_status_4;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_AN_STATUS_4: %x\n", val);
	val = d->anlt_data.hssiss_csr_an_status_6;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_AN_STATUS_6: %x\n\n", val);
	/* Link Training registers */
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "LT status registers:\n");
	val = d->anlt_data.hssiss_csr_lt_status_1;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_LT_STATUS_1: %x\n\n", val);
	/* KR_DEBUG registers */
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "KR_DEBUG registers:\n");
	val = d->anlt_data.hssiss_csr_kr_debug_0;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_KR_DEBUG_0: %x\n", val);
	val = d->anlt_data.hssiss_csr_kr_debug_1;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_KR_DEBUG_1: %x\n", val);
	val = d->anlt_data.hssiss_csr_kr_debug_2;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_KR_DEBUG_2: %x\n", val);
	val = d->anlt_data.hssiss_csr_kr_debug_3;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_KR_DEBUG_3: %x\n", val);
	val = d->anlt_data.hssiss_csr_kr_debug_4;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_KR_DEBUG_4: %x\n", val);
	val = d->anlt_data.hssiss_csr_kr_debug_5;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_KR_DEBUG_5: %x\n", val);
	val = d->anlt_data.hssiss_csr_kr_debug_6;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_KR_DEBUG_6: %x\n", val);
	val = d->anlt_data.hssiss_csr_kr_debug_7;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_KR_DEBUG_7: %x\n", val);
	val = d->anlt_data.hssiss_csr_kr_debug_8;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_KR_DEBUG_8: %x\n", val);
	val = d->anlt_data.hssiss_csr_kr_debug_9;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_KR_DEBUG_9: %x\n", val);
	val = d->anlt_data.hssiss_csr_kr_debug_10;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_KR_DEBUG_10: %x\n", val);
	val = d->anlt_data.hssiss_csr_kr_debug_11;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_KR_DEBUG_11: %x\n", val);
	val = d->anlt_data.hssiss_csr_kr_debug_12;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_KR_DEBUG_12: %x\n", val);
	val = d->anlt_data.hssiss_csr_kr_debug_13;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_KR_DEBUG_13: %x\n", val);
	val = d->anlt_data.hssiss_csr_kr_debug_14;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_KR_DEBUG_14: %x\n", val);
	val = d->anlt_data.hssiss_csr_kr_debug_15;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_KR_DEBUG_15: %x\n", val);

	ret = simple_read_from_buffer(ubuf, count, offp, buf, ret);

	kfree(buf);
	return ret;
}

static ssize_t anlt_dbgfs_dump_by_port_write(struct file *filep, const char __user *ubuf,
					     size_t count, loff_t *offp)
{
	struct hssiss_dbg *d = filep->private_data;
	struct platform_device *pdev = d->pdev;
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	void __iomem *base = priv->sscsr;
	char *buf;
	int ret, port;
	u32 anlt_base;
	/* Copy data from User-space */
	buf = kmalloc(count + 1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = simple_write_to_buffer(buf, count, offp, ubuf, count);
	if (ret < 0) {
		kfree(buf);
		return -EIO;
	}
	buf[count] = 0;

	/* Parse the values */
	//ret = sscanf(buf, "%d", &port);
	ret = kstrtouint(buf, 10, &port);
	kfree(buf);
	if (ret < 1)
		return -EINVAL;
	anlt_base = HSSISS_CSR_ANLT_BASE + (port * HSSISS_CSR_ANLT_RANGE);
	d->anlt_data.port = port;
	// Config registers
	d->anlt_data.hssiss_csr_anlt_seq_cfg = csrrd32(base, anlt_base + HSSISS_CSR_ANLT_SEQ_CFG);

	d->anlt_data.hssiss_csr_an_cfg_1 = csrrd32(base, anlt_base + HSSISS_CSR_AN_CFG_1);
	d->anlt_data.hssiss_csr_an_cfg_2 = csrrd32(base, anlt_base + HSSISS_CSR_AN_CFG_2);
	d->anlt_data.hssiss_csr_an_cfg_3 = csrrd32(base, anlt_base + HSSISS_CSR_AN_CFG_3);
	d->anlt_data.hssiss_csr_an_cfg_4 = csrrd32(base, anlt_base + HSSISS_CSR_AN_CFG_4);
	d->anlt_data.hssiss_csr_an_cfg_5 = csrrd32(base, anlt_base + HSSISS_CSR_AN_CFG_5);
	d->anlt_data.hssiss_csr_an_cfg_6 = csrrd32(base, anlt_base + HSSISS_CSR_AN_CFG_6);
	d->anlt_data.hssiss_csr_an_cfg_8 = csrrd32(base, anlt_base + HSSISS_CSR_AN_CFG_8);

	d->anlt_data.hssiss_csr_lt_cfg_1 = csrrd32(base, anlt_base + HSSISS_CSR_LT_CFG_1);
	d->anlt_data.hssiss_csr_lt_cfg_2 = csrrd32(base, anlt_base + HSSISS_CSR_LT_CFG_2);

	//Status registers
	d->anlt_data.hssiss_csr_anlt_seq_status = csrrd32(base,
							  anlt_base + HSSISS_CSR_ANLT_SEQ_STATUS);
	d->anlt_data.hssiss_csr_an_status = csrrd32(base, anlt_base + HSSISS_CSR_AN_STATUS);

	d->anlt_data.hssiss_csr_an_status_1 = csrrd32(base, anlt_base + HSSISS_CSR_AN_STATUS_1);
	d->anlt_data.hssiss_csr_an_status_2 = csrrd32(base, anlt_base + HSSISS_CSR_AN_STATUS_2);
	d->anlt_data.hssiss_csr_an_status_3 = csrrd32(base, anlt_base + HSSISS_CSR_AN_STATUS_3);
	d->anlt_data.hssiss_csr_an_status_4 = csrrd32(base, anlt_base + HSSISS_CSR_AN_STATUS_4);
	d->anlt_data.hssiss_csr_an_status_6 = csrrd32(base, anlt_base + HSSISS_CSR_AN_STATUS_6);
	d->anlt_data.hssiss_csr_lt_status_1 = csrrd32(base, anlt_base + HSSISS_CSR_LT_STATUS_1);

	d->anlt_data.hssiss_csr_kr_debug_0 = csrrd32(base, anlt_base + HSSISS_CSR_KR_DEBUG_0);
	d->anlt_data.hssiss_csr_kr_debug_1 = csrrd32(base, anlt_base + HSSISS_CSR_KR_DEBUG_1);
	d->anlt_data.hssiss_csr_kr_debug_2 = csrrd32(base, anlt_base + HSSISS_CSR_KR_DEBUG_2);
	d->anlt_data.hssiss_csr_kr_debug_3 = csrrd32(base, anlt_base + HSSISS_CSR_KR_DEBUG_3);
	d->anlt_data.hssiss_csr_kr_debug_4 = csrrd32(base, anlt_base + HSSISS_CSR_KR_DEBUG_4);
	d->anlt_data.hssiss_csr_kr_debug_5 = csrrd32(base, anlt_base + HSSISS_CSR_KR_DEBUG_5);
	d->anlt_data.hssiss_csr_kr_debug_6 = csrrd32(base, anlt_base + HSSISS_CSR_KR_DEBUG_6);
	d->anlt_data.hssiss_csr_kr_debug_7 = csrrd32(base, anlt_base + HSSISS_CSR_KR_DEBUG_7);
	d->anlt_data.hssiss_csr_kr_debug_8 = csrrd32(base, anlt_base + HSSISS_CSR_KR_DEBUG_8);
	d->anlt_data.hssiss_csr_kr_debug_9 = csrrd32(base, anlt_base + HSSISS_CSR_KR_DEBUG_9);
	d->anlt_data.hssiss_csr_kr_debug_10 = csrrd32(base, anlt_base + HSSISS_CSR_KR_DEBUG_10);
	d->anlt_data.hssiss_csr_kr_debug_11 = csrrd32(base, anlt_base + HSSISS_CSR_KR_DEBUG_11);
	d->anlt_data.hssiss_csr_kr_debug_12 = csrrd32(base, anlt_base + HSSISS_CSR_KR_DEBUG_12);
	d->anlt_data.hssiss_csr_kr_debug_13 = csrrd32(base, anlt_base + HSSISS_CSR_KR_DEBUG_13);
	d->anlt_data.hssiss_csr_kr_debug_14 = csrrd32(base, anlt_base + HSSISS_CSR_KR_DEBUG_14);
	d->anlt_data.hssiss_csr_kr_debug_15 = csrrd32(base, anlt_base + HSSISS_CSR_KR_DEBUG_15);

	return count;
}

static ssize_t en_dis_anlt_read(struct file *filep, char __user *ubuf,
				size_t count, loff_t *offp)
{
	struct hssiss_dbg *d = filep->private_data;
	char *buf;
	int ret;
	u32 val;

	buf = kzalloc(BUF_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = scnprintf(buf, BUF_SIZE, "Dumping AN/LT config registers for port : %d\n\n",
			d->anlt_data.port);
	/* ANLT Sequencer (Complete State machine) */
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "ANLT sequencer config registers:\n");

	val = d->anlt_data.hssiss_csr_an_cfg_1;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_AN_CFG_1: %x\n\n", val);
	ret = simple_read_from_buffer(ubuf, count, offp, buf, ret);

	val = d->anlt_data.hssiss_csr_lt_cfg_1;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_LT_CFG_1: %x\n\n", val);
	ret = simple_read_from_buffer(ubuf, count, offp, buf, ret);

	val = d->anlt_data.hssiss_csr_anlt_seq_cfg;
	ret += scnprintf(buf + ret, BUF_SIZE - ret, "HSSISS_CSR_ANLT_SEQ_CFG: %x\n\n", val);
	ret = simple_read_from_buffer(ubuf, count, offp, buf, ret);

	kfree(buf);
	return ret;
}

static ssize_t en_dis_anlt_write(struct file *filep, const char __user *ubuf,
				 size_t count, loff_t *offp)
{
	struct hssiss_dbg *d = filep->private_data;
	struct platform_device *pdev = d->pdev;
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	void __iomem *base = priv->sscsr;
	char *buf;
	int ret, port, bit;
	u32 anlt_base, val, an_cfg_1, lt_cfg_1, anlt_seq_cfg;
	/* Copy data from User-space */
	buf = kmalloc(count + 1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = simple_write_to_buffer(buf, count, offp, ubuf, count);
	if (ret < 0) {
		kfree(buf);
		return -EIO;
	}
	buf[count] = 0;

	/* Parse the values */
	ret = sscanf(buf, "%d %d", &port, &bit);
	kfree(buf);
	if (ret < 1)
		return -EINVAL;

	anlt_base = HSSISS_CSR_ANLT_BASE + (port * HSSISS_CSR_ANLT_RANGE);
	d->anlt_data.port = port;
	an_cfg_1 = csrrd32(base, anlt_base + HSSISS_CSR_AN_CFG_1);
	dev_dbg(&pdev->dev, "prev: an_cfg_1: %x\n", an_cfg_1);
	val = update_bit(an_cfg_1, 0, bit);
	csrwr32(val, base, anlt_base + HSSISS_CSR_AN_CFG_1);
	d->anlt_data.hssiss_csr_an_cfg_1 = csrrd32(base, anlt_base + HSSISS_CSR_AN_CFG_1);

	lt_cfg_1 = csrrd32(base, anlt_base + HSSISS_CSR_LT_CFG_1);
	dev_dbg(&pdev->dev, "prev: lt_cfg_1: %x\n", lt_cfg_1);
	val = update_bit(lt_cfg_1, 0, bit);
	csrwr32(val, base, anlt_base + HSSISS_CSR_LT_CFG_1);
	d->anlt_data.hssiss_csr_lt_cfg_1 = csrrd32(base, anlt_base + HSSISS_CSR_LT_CFG_1);

	anlt_seq_cfg = csrrd32(base, anlt_base + HSSISS_CSR_ANLT_SEQ_CFG);
	dev_dbg(&pdev->dev, "prev: anlt_seq_cfg: %x\n", anlt_seq_cfg);
	val = update_bit(anlt_seq_cfg, 0, 1);
	csrwr32(val, base, anlt_base + HSSISS_CSR_ANLT_SEQ_CFG);
	d->anlt_data.hssiss_csr_anlt_seq_cfg = csrrd32(base, anlt_base + HSSISS_CSR_ANLT_SEQ_CFG);
	return count;
}

static const struct file_operations ctrladdr_dbgfs_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = hssiss_dbgfs_ctrladdr_write,
	.read = hssiss_dbgfs_ctrladdr_read
};

static const struct file_operations cmdsts_dbgfs_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = hssiss_dbgfs_cmdsts_write,
	.read = hssiss_dbgfs_cmdsts_read
};

static const struct file_operations csr_dbgfs_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = hssiss_dbgfs_csr_write,
	.read = hssiss_dbgfs_csr_read
};

static const struct file_operations wr_dbgfs_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = hssiss_dbgfs_wr_write,
	.read = hssiss_dbgfs_wr_read
};

static const struct file_operations rd_dbgfs_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = hssiss_dbgfs_rd_write,
	.read = hssiss_dbgfs_rd_read
};

static const struct file_operations sal_dbgfs_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = hssiss_dbgfs_sal_write,
	.read = hssiss_dbgfs_sal_read
};

static const struct file_operations readme_dbgfs_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = hssiss_dbgfs_readme_read
};

static const struct file_operations dumpcsr_dbgfs_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = hssiss_dbgfs_dumpcsr_read
};

static const struct file_operations direct_reg_dbgfs_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = hssiss_dbgfs_direct_reg_write,
	.read = hssiss_dbgfs_direct_reg_read
};

/* ANLT debug ops */

static const struct file_operations anlt_status_dump_by_port_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = anlt_dbgfs_dump_by_port_write,
	.read = anlt_dbgfs_dump_by_port_read
};

static const struct file_operations en_dis_anlt_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = en_dis_anlt_write,
	.read = en_dis_anlt_read
};

struct hssiss_dbg *hssiss_dbgfs_init(struct platform_device *pdev)
{
	struct hssiss_dbg *d;
	char *hssidev_name;

	struct dentry *anlt_dbgfs;

	d = devm_kzalloc(&pdev->dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return NULL;

	d->pdev = pdev;

	hssidev_name = kzalloc(strlen(pdev->name) + 5, GFP_KERNEL);
	if (!hssidev_name)
		return NULL;

	sprintf(hssidev_name, "%s_dbg", pdev->name);
	d->dbgfs = debugfs_create_dir(hssidev_name, NULL);

	debugfs_create_file("csr", 0644, d->dbgfs, d, &csr_dbgfs_ops);
	debugfs_create_file("ctrladdr", 0644, d->dbgfs, d, &ctrladdr_dbgfs_ops);
	debugfs_create_file("cmdsts", 0644, d->dbgfs, d, &cmdsts_dbgfs_ops);
	debugfs_create_file("wr", 0644, d->dbgfs, d, &wr_dbgfs_ops);
	debugfs_create_file("rd", 0644, d->dbgfs, d, &rd_dbgfs_ops);
	debugfs_create_file("sal", 0644, d->dbgfs, d, &sal_dbgfs_ops);
	debugfs_create_file("dumpcsr", 0444, d->dbgfs, d, &dumpcsr_dbgfs_ops);
	debugfs_create_file("readme", 0444, d->dbgfs, d, &readme_dbgfs_ops);
	debugfs_create_file("direct_reg", 0644, d->dbgfs, d, &direct_reg_dbgfs_ops);

	anlt_dbgfs = debugfs_create_dir("anlt_dbg", d->dbgfs);
	debugfs_create_file("anlt_dump_status_by_port", 0644, anlt_dbgfs, d,
			    &anlt_status_dump_by_port_ops);
	debugfs_create_file("en_dis_anlt", 0644, anlt_dbgfs, d, &en_dis_anlt_ops);
	return d;
}

void hssiss_dbgfs_remove(struct hssiss_dbg *d)
{
	debugfs_remove_recursive(d->dbgfs);
}
