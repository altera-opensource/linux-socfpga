// SPDX-License-Identifier: GPL-2.0

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/iopoll.h>

#define ALTERA_QUADSPI_RESOURCE_NAME			"altera_quadspi"

/* max possible slots for serial flash chip in the QUADSPI controller */
#define MAX_NUM_FLASH_CHIP				3
#define EPCS					1
#define NON_EPCS					2

#define WRITE_CHECK					1
#define ERASE_CHECK					0

#define NOR_OP_RDID					0x9F
#define NOR_OP_RDSR					0x05

/* Define max times to check status register before we give up. */
#define QUADSPI_MAX_TIME_OUT_USEC			40000

/* defines for status register */
#define QUADSPI_SR_REG					0x0
#define QUADSPI_SR_WIP_MASK				0x00000001
#define QUADSPI_SR_WIP					0x1
#define QUADSPI_SR_WEL					0x2
#define QUADSPI_SR_BP0					0x4
#define QUADSPI_SR_BP1					0x8
#define QUADSPI_SR_BP2					0x10
#define QUADSPI_SR_BP3					0x40
#define QUADSPI_SR_TB					0x20
#define QUADSPI_SR_MASK				0x0000000F

/* defines for device id register */
#define QUADSPI_SID_REG				0x4
#define QUADSPI_RDID_REG				0x8
#define QUADSPI_ID_MASK				0x000000FF

/*
 * QUADSPI_MEM_OP register offset
 *
 * The QUADSPI_MEM_OP register is used to do memory protect and erase operations
 *
 */
#define QUADSPI_MEM_OP_REG				0xC

#define QUADSPI_MEM_OP_CMD_MASK			0x00000003
#define QUADSPI_MEM_OP_BULK_ERASE_CMD			0x00000001
#define QUADSPI_MEM_OP_SECTOR_ERASE_CMD		0x00000002
#define QUADSPI_MEM_OP_SECTOR_PROTECT_CMD		0x00000003
#define QUADSPI_MEM_OP_SECTOR_VALUE_MASK		0x0003FF00
#define QUADSPI_MEM_OP_SECTOR_PROTECT_VALUE_MASK	0x00001F00
#define QUADSPI_MEM_OP_SECTOR_PROTECT_SHIFT		8
/*
 * QUADSPI_ISR register offset
 *
 * The QUADSPI_ISR register is used to determine whether an invalid write or
 * erase operation trigerred an interrupt
 *
 */
#define QUADSPI_ISR_REG				0x10

#define QUADSPI_ISR_ILLEGAL_ERASE_MASK			0x00000001
#define QUADSPI_ISR_ILLEGAL_WRITE_MASK			0x00000002

/*
 * QUADSPI_IMR register offset
 *
 * The QUADSPI_IMR register is used to mask the invalid erase or the invalid
 * write interrupts.
 *
 */
#define QUADSPI_IMR_REG				0x14
#define QUADSPI_IMR_ILLEGAL_ERASE_MASK			0x00000001

#define QUADSPI_IMR_ILLEGAL_WRITE_MASK			0x00000002

#define QUADSPI_CHIP_SELECT_REG			0x18
#define QUADSPI_CHIP_SELECT_MASK			0x00000007
#define QUADSPI_CHIP_SELECT_0				0x00000001
#define QUADSPI_CHIP_SELECT_1				0x00000002
#define QUADSPI_CHIP_SELECT_2				0x00000004

struct altera_quadspi {
	u32 flash_id;
	void __iomem *csr_base;
	void __iomem *data_base;
	u32 num_flashes;
	struct device *dev;
	struct mutex lock;	/* device lock */
	struct altera_quadspi_flash *flash[MAX_NUM_FLASH_CHIP];
	struct device_node *np[MAX_NUM_FLASH_CHIP];
};

struct altera_quadspi_flash {
	struct mtd_info mtd;
	struct mutex lock;	/* flash lock */
};

struct flash_device {
	char *name;
	u32 flash_id;
	u32 device_id;
	u32 sector_size;
	u64 size;
};

#define FLASH_ID(_n, _flash_id, _id, _ssize, _size)	\
{					\
	.name = (_n),			\
	.flash_id = (_flash_id),	\
	.device_id = (_id),		\
	.sector_size = (_ssize),	\
	.size = (_size),		\
}

static struct flash_device flash_devices[] = {
	FLASH_ID("epcs16-nonjedec",    EPCS,     0x14, 0x10000, 0x200000),
	FLASH_ID("epcs64-nonjedec",    EPCS,     0x16, 0x10000, 0x800000),
	FLASH_ID("epcs128-nonjedec",   EPCS,     0x18, 0x40000, 0x1000000),

	FLASH_ID("epcq16-nonjedec",    NON_EPCS, 0x15, 0x10000, 0x200000),
	FLASH_ID("epcq32-nonjedec",    NON_EPCS, 0x16, 0x10000, 0x400000),
	FLASH_ID("epcq64-nonjedec",    NON_EPCS, 0x17, 0x10000, 0x800000),
	FLASH_ID("epcq128-nonjedec",   NON_EPCS, 0x18, 0x10000, 0x1000000),
	FLASH_ID("epcq256-nonjedec",   NON_EPCS, 0x19, 0x10000, 0x2000000),
	FLASH_ID("epcq512-nonjedec",   NON_EPCS, 0x20, 0x10000, 0x4000000),
	FLASH_ID("epcq1024-nonjedec",  NON_EPCS, 0x21, 0x10000, 0x8000000),

	FLASH_ID("epcql256-nonjedec",  NON_EPCS, 0x19, 0x10000, 0x2000000),
	FLASH_ID("epcql512-nonjedec",  NON_EPCS, 0x20, 0x10000, 0x4000000),
	FLASH_ID("epcql1024-nonjedec", NON_EPCS, 0x21, 0x10000, 0x8000000),

	FLASH_ID("n25q016",           NON_EPCS, 0x15, 0x10000, 0x200000),
	FLASH_ID("n25q032",           NON_EPCS, 0x16, 0x10000, 0x400000),
	FLASH_ID("n25q064",           NON_EPCS, 0x17, 0x10000, 0x800000),
	FLASH_ID("n25q128a13",        NON_EPCS, 0x18, 0x10000, 0x1000000),
	FLASH_ID("n25q256a",          NON_EPCS, 0x19, 0x10000, 0x2000000),
	FLASH_ID("n25q256a11",        NON_EPCS, 0x19, 0x10000, 0x2000000),
	FLASH_ID("n25q512a",          NON_EPCS, 0x20, 0x10000, 0x4000000),
	FLASH_ID("n25q512ax3",        NON_EPCS, 0x20, 0x10000, 0x4000000),
	FLASH_ID("mt25ql512",         NON_EPCS, 0x20, 0x10000, 0x4000000),
	FLASH_ID("n25q00a11",         NON_EPCS, 0x21, 0x10000, 0x8000000),

};

static inline struct altera_quadspi_flash *get_flash_data(struct mtd_info *mtd)
{
	return container_of(mtd, struct altera_quadspi_flash, mtd);
}

static int altera_quadspi_read_reg(struct altera_quadspi *q, u8 opcode, u8 *val)
{
	u32 data = 0;

	switch (opcode) {
	case NOR_OP_RDSR:
		data = readl(q->csr_base + QUADSPI_SR_REG);
		*val = (u8)data & QUADSPI_SR_MASK;
		break;
	case NOR_OP_RDID:
		if (q->flash_id == EPCS)
			data = readl(q->csr_base + QUADSPI_SID_REG);
		else
			data = readl(q->csr_base + QUADSPI_RDID_REG);

		*val = (u8)data & QUADSPI_ID_MASK;
		break;
	default:
		*val = 0;
		break;
	}
	return 0;
}

static int altera_quadspi_write_erase_check(struct altera_quadspi *q,
					    bool write_erase)
{
	u32 val;
	u32 mask;

	if (write_erase)
		mask = QUADSPI_ISR_ILLEGAL_WRITE_MASK;
	else
		mask = QUADSPI_ISR_ILLEGAL_ERASE_MASK;

	val = readl(q->csr_base + QUADSPI_ISR_REG);
	if (val & mask) {
		dev_err(q->dev,
			"write/erase failed, sector might be protected\n");
		/* clear this status for next use */
		writel(val, q->csr_base + QUADSPI_ISR_REG);
		return -EIO;
	}
	return 0;
}

static int altera_quadspi_addr_to_sector(struct mtd_info *mtd, u64 offset)
{
	if (mtd->erasesize_shift)
		return offset >> mtd->erasesize_shift;
	do_div(offset, mtd->erasesize);
	return offset;
}

static int altera_quadspi_erase_chip(struct mtd_info *mtd)
{
	u32 val;
	struct altera_quadspi *q = mtd->priv;

	/* erase chip. */
	val = QUADSPI_MEM_OP_BULK_ERASE_CMD;
	writel(val, q->csr_base + QUADSPI_MEM_OP_REG);

	 /* check whether write triggered a illegal erase interrupt */
	return altera_quadspi_write_erase_check(q, ERASE_CHECK);
}

static int altera_quadspi_erase_sector(struct mtd_info *mtd,
				       u64 addr_offset)
{
	struct altera_quadspi *q = mtd->priv;
	u32 val;
	int sector_value;

	sector_value = altera_quadspi_addr_to_sector(mtd, addr_offset);

	/* sanity check that block_offset is a valid sector number */
	if (sector_value < 0)
		return -EINVAL;

	/* sector value should occupy bits 17:8 */
	val = (sector_value << 8) & QUADSPI_MEM_OP_SECTOR_VALUE_MASK;

	/* sector erase commands occupies lower 2 bits */
	val |= QUADSPI_MEM_OP_SECTOR_ERASE_CMD;

	/* write sector erase command to QUADSPI_MEM_OP register */
	writel(val, q->csr_base + QUADSPI_MEM_OP_REG);

	 /* check whether write triggered a illegal erase interrupt */
	return altera_quadspi_write_erase_check(q, ERASE_CHECK);
}

static int altera_quadspi_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct altera_quadspi *q = mtd->priv;
	struct altera_quadspi_flash *flash = get_flash_data(mtd);
	u32 addr, len;
	u32 rem;
	int ret = 0;
	void __iomem *reg_addr;
	u8 status;

	dev_dbg(q->dev, "Erase at 0x%llx, len %lld\n", (long long)instr->addr,
		(long long)instr->len);

	div_u64_rem(instr->len, mtd->erasesize, &rem);
	if (rem)
		return -EINVAL;

	addr = instr->addr;
	len = instr->len;

	if (q->flash_id == EPCS)
		reg_addr = q->csr_base + QUADSPI_SID_REG;
       else
		reg_addr = q->csr_base + QUADSPI_RDID_REG;
	mutex_lock(&flash->lock);

	/* erase whole chip */
	if (len == mtd->size) {
		if (altera_quadspi_erase_chip(mtd)) {
			ret = -EIO;
			goto erase_err;
		}
		ret = readl_poll_timeout(reg_addr, status,
				(!(status & QUADSPI_SR_WIP) || (status < 0)),
				jiffies_to_usecs(cond_resched()),
				QUADSPI_MAX_TIME_OUT_USEC);
		if (ret)
			goto erase_err;
	/* "sector"-at-a-time erase */
	} else {
		while (len) {
			if (altera_quadspi_erase_sector(mtd, addr)) {
				ret = -EIO;
				goto erase_err;
			}

			addr += mtd->erasesize;
			len -= mtd->erasesize;
			ret = readl_poll_timeout(reg_addr, status,
				(!(status & QUADSPI_SR_WIP) || (status < 0)),
				jiffies_to_usecs(cond_resched()),
				QUADSPI_MAX_TIME_OUT_USEC);
			if (ret)
				goto erase_err;
		}
	}
	mutex_unlock(&flash->lock);

	instr->state = MTD_ERASE_DONE;
	mtd_erase_callback(instr);
	return ret;

erase_err:
	mutex_unlock(&flash->lock);
	instr->state = MTD_ERASE_FAILED;
	return ret;
}

static int altera_quadspi_read(struct mtd_info *mtd, loff_t from, size_t len,
			       size_t *retlen, u8 *buf)
{
	struct altera_quadspi_flash *flash = get_flash_data(mtd);
	struct altera_quadspi *q = mtd->priv;

	mutex_lock(&flash->lock);

	memcpy_fromio(buf, q->data_base + from, len);
	*retlen = len;

	mutex_unlock(&flash->lock);
	return 0;
}

static int altera_quadspi_write(struct mtd_info *mtd, loff_t to, size_t len,
				size_t *retlen, const u8 *buf)
{
	struct altera_quadspi_flash *flash = get_flash_data(mtd);
	struct altera_quadspi *q = mtd->priv;
	int ret = 0;
	void __iomem *reg_addr;
	u8 status;

	mutex_lock(&flash->lock);

	if (q->flash_id == EPCS)
		reg_addr = q->csr_base + QUADSPI_SID_REG;
	else
		reg_addr = q->csr_base + QUADSPI_RDID_REG;

	/* wait until finished previous write command */
	ret = readl_poll_timeout(reg_addr, status,
				(!(status & QUADSPI_SR_WIP) || (status < 0)),
				jiffies_to_usecs(cond_resched()),
				QUADSPI_MAX_TIME_OUT_USEC);
	if (ret)
		goto err;

	memcpy_toio(q->data_base + to, buf, len);
	*retlen += len;

	 /* check whether write triggered a illegal write interrupt */
	ret = altera_quadspi_write_erase_check(q, WRITE_CHECK);

err:
	mutex_unlock(&flash->lock);
	return ret;
}

static int altera_quadspi_lock(struct mtd_info *mtd, loff_t ofs, u64 len)
{
	struct altera_quadspi_flash *flash = get_flash_data(mtd);
	struct altera_quadspi *q = mtd->priv;
	u32 offset = ofs;
	u32 sector_start, sector_end;
	u64 num_sectors;
	u32 mem_op;
	u32 sr_bp;
	u32 sr_tb;

	sector_start = offset;
	sector_end = altera_quadspi_addr_to_sector(mtd, offset + len);
	num_sectors = mtd->size;
	do_div(num_sectors, mtd->erasesize);

	mutex_lock(&flash->lock);

	/* Refer to block protection tables in flash datasheet */
	if (sector_start >= num_sectors / 2) {
		sr_bp = fls(num_sectors - 1 - sector_start) + 1;
		sr_tb = 0;
	} else if ((sector_end < num_sectors / 2) &&
		  (q->flash_id != EPCS)) {
		sr_bp = fls(sector_end) + 1;
		sr_tb = 1;
	} else {
		sr_bp = 16;
		sr_tb = 0;
	}

	mem_op = (sr_tb << 12) | (sr_bp << 8);
	mem_op &= QUADSPI_MEM_OP_SECTOR_PROTECT_VALUE_MASK;
	mem_op |= QUADSPI_MEM_OP_SECTOR_PROTECT_CMD;
	writel(mem_op, q->csr_base + QUADSPI_MEM_OP_REG);

	mutex_unlock(&flash->lock);
	return 0;
}

static int altera_quadspi_unlock(struct mtd_info *mtd, loff_t ofs, u64 len)
{
	struct altera_quadspi_flash *flash = get_flash_data(mtd);
	struct altera_quadspi *q = mtd->priv;
	u32 mem_op;

	mutex_lock(&flash->lock);

	dev_dbg(q->dev, "Unlock all protected area\n");
	mem_op = 0;
	mem_op &= QUADSPI_MEM_OP_SECTOR_PROTECT_VALUE_MASK;
	mem_op |= QUADSPI_MEM_OP_SECTOR_PROTECT_CMD;
	writel(mem_op, q->csr_base + QUADSPI_MEM_OP_REG);

	mutex_unlock(&flash->lock);
	return 0;
}

static void altera_quadspi_chip_select(struct altera_quadspi *q, u32 bank)
{
	u32 val = 0;

	switch (bank) {
	case 0:
		val = QUADSPI_CHIP_SELECT_0;
		break;
	case 1:
		val = QUADSPI_CHIP_SELECT_1;
		break;
	case 2:
		val = QUADSPI_CHIP_SELECT_2;
		break;
	default:
		dev_err(q->dev, "invalid bank\n");
		return;
	}
	writel(val, q->csr_base + QUADSPI_CHIP_SELECT_REG);
}

static int altera_quadspi_probe_config_dt(struct platform_device *pdev,
					  struct device_node *np,
					  struct altera_quadspi *q)
{
	struct device_node *pp = NULL;
	struct resource *quadspi_res;
	int i = 0;

	quadspi_res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   "avl_csr");
	q->csr_base = devm_ioremap_resource(&pdev->dev, quadspi_res);
	if (IS_ERR(q->csr_base)) {
		dev_err(&pdev->dev, "%s: ERROR: failed to map csr base\n",
			__func__);
		return PTR_ERR(q->csr_base);
	}

	quadspi_res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   "avl_mem");
	q->data_base = devm_ioremap_resource(&pdev->dev, quadspi_res);
	if (IS_ERR(q->data_base)) {
		dev_err(&pdev->dev, "%s: ERROR: failed to map data base\n",
			__func__);
		return PTR_ERR(q->data_base);
	}

	/* Fill structs for each subnode (flash device) */
	for_each_available_child_of_node(np, pp) {
		/* Read bank id from DT */
		q->np[i] = pp;
		i++;
	}
	q->num_flashes = i;
	return 0;
}

static int altera_quadspi_scan(struct altera_quadspi *q, const char *name)
{
	int index;
	int ret = 0;
	u8 id = 0;

	ret = altera_quadspi_read_reg(q, NOR_OP_RDID, &id);
	if (ret)
		return -EINVAL;
	for (index = 0; index < ARRAY_SIZE(flash_devices); index++) {
		if (flash_devices[index].device_id == id &&
		    strcmp(name, flash_devices[index].name) == 0) {
			q->flash_id = flash_devices[index].flash_id;
			return index;
		}
	}
	/* Memory chip is not listed and not supported */
	return -EINVAL;
}

static int altera_quadspi_setup_banks(struct platform_device *pdev,
				      u32 bank, struct device_node *np)
{
	struct altera_quadspi *q = platform_get_drvdata(pdev);
	struct mtd_part_parser_data ppdata = {};
	struct altera_quadspi_flash *flash;
	int id;
	char modalias[40];

	altera_quadspi_chip_select(q, bank);

	if (bank > q->num_flashes - 1)
		return -EINVAL;

	flash = devm_kzalloc(q->dev, sizeof(*flash), GFP_KERNEL);
	if (!flash)
		return -ENOMEM;

	mutex_init(&flash->lock);

	/* get compatible string for each flash node */
	if (of_modalias_node(np, modalias, sizeof(modalias)) < 0)
		return -EINVAL;
	id = altera_quadspi_scan(q, modalias);
	if (id < 0)
		return id;

	q->flash[bank] = flash;

	/* make sure sector size is non-zero value */
	if (flash_devices[id].sector_size == 0) {
		dev_err(q->dev, "invalid sector size\n");
		return -EINVAL;
	}
	/* mtd framework */
	flash->mtd.priv = q;
	flash->mtd.name = flash_devices[id].name;
	mtd_set_of_node(&flash->mtd, np);
	flash->mtd.type = MTD_NORFLASH;
	flash->mtd.writesize = 1;
	flash->mtd.flags = MTD_CAP_NORFLASH;
	flash->mtd.size = flash_devices[id].size;
	flash->mtd.erasesize = flash_devices[id].sector_size;
	flash->mtd._erase = altera_quadspi_erase;
	flash->mtd._read = altera_quadspi_read;
	flash->mtd._write = altera_quadspi_write;
	flash->mtd._lock = altera_quadspi_lock;
	flash->mtd._unlock = altera_quadspi_unlock;

	dev_dbg(q->dev, "mtd .name=%s .size=0x%llx (%lluM)\n",
		flash->mtd.name, (long long)flash->mtd.size,
		(long long)(flash->mtd.size >> 20));

	dev_dbg(q->dev, ".erasesize = 0x%x(%uK)\n",
		flash->mtd.erasesize, flash->mtd.erasesize >> 10);

	return mtd_device_parse_register(&flash->mtd, NULL, &ppdata, NULL, 0);
}

static int altera_quadspi_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct altera_quadspi *q;
	int ret = 0;
	int i;

	if (!np) {
		dev_err(dev, "no device found\n");
		return -ENODEV;
	}

	q = devm_kzalloc(dev, sizeof(*q), GFP_KERNEL);
	if (!q)
		return -ENOMEM;

	ret = altera_quadspi_probe_config_dt(pdev, np, q);
	if (ret) {
		dev_err(dev, "probe fail\n");
		return -ENODEV;
	}

	mutex_init(&q->lock);

	q->dev = dev;

	/* check number of flashes */
	if (q->num_flashes > MAX_NUM_FLASH_CHIP) {
		dev_err(dev, "exceeding max number of flashes\n");
		q->num_flashes = MAX_NUM_FLASH_CHIP;
	}

	platform_set_drvdata(pdev, q);

	/* loop for each serial flash which is connected to quadspi */
	for (i = 0; i < q->num_flashes; i++) {
		ret = altera_quadspi_setup_banks(pdev, i, q->np[i]);
		if (ret) {
			dev_err(dev, "bank setup failed\n");
			return ret;
		}
	}

	return 0;
}

static int altera_quadspi_remove(struct platform_device *pdev)
{
	struct altera_quadspi *q = platform_get_drvdata(pdev);
	struct altera_quadspi_flash *flash;
	int i;
	int ret = 0;

	/* clean up for all nor flash */
	for (i = 0; i < q->num_flashes; i++) {
		flash = q->flash[i];
		if (!flash)
			continue;

		/* clean up mtd stuff */
		ret = mtd_device_unregister(&flash->mtd);
		if (ret) {
			dev_err(&pdev->dev, "error removing mtd\n");
			return ret;
		}
	}

	return 0;
}

static const struct of_device_id altera_quadspi_id_table[] = {
	{ .compatible = "altr,quadspi-1.0" },
	{}
};

MODULE_DEVICE_TABLE(of, altera_quadspi_id_table);

static struct platform_driver altera_quadspi_driver = {
	.driver = {
		.name = ALTERA_QUADSPI_RESOURCE_NAME,
		.bus = &platform_bus_type,
		.of_match_table = altera_quadspi_id_table,
	},
	.probe = altera_quadspi_probe,
	.remove = altera_quadspi_remove,
};

module_platform_driver(altera_quadspi_driver);

MODULE_AUTHOR("Viet Nga Dao <vndao@altera.com>");
MODULE_DESCRIPTION("Altera QuadSPI Driver");
MODULE_LICENSE("GPL v2");
