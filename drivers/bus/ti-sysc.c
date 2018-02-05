/*
 * ti-sysc.c - Texas Instruments sysc interconnect target driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/io.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_data/ti-sysc.h>

#include <dt-bindings/bus/ti-sysc.h>

enum sysc_registers {
	SYSC_REVISION,
	SYSC_SYSCONFIG,
	SYSC_SYSSTATUS,
	SYSC_MAX_REGS,
};

static const char * const reg_names[] = { "rev", "sysc", "syss", };

enum sysc_clocks {
	SYSC_FCK,
	SYSC_ICK,
	SYSC_MAX_CLOCKS,
};

static const char * const clock_names[] = { "fck", "ick", };

#define SYSC_IDLEMODE_MASK		3
#define SYSC_CLOCKACTIVITY_MASK		3

/**
 * struct sysc - TI sysc interconnect target module registers and capabilities
 * @dev: struct device pointer
 * @module_pa: physical address of the interconnect target module
 * @module_size: size of the interconnect target module
 * @module_va: virtual address of the interconnect target module
 * @offsets: register offsets from module base
 * @clocks: clocks used by the interconnect target module
 * @legacy_mode: configured for legacy mode if set
 * @cap: interconnect target module capabilities
 * @cfg: interconnect target module configuration
 * @name: name if available
 * @revision: interconnect target module revision
 */
struct sysc {
	struct device *dev;
	u64 module_pa;
	u32 module_size;
	void __iomem *module_va;
	int offsets[SYSC_MAX_REGS];
	struct clk *clocks[SYSC_MAX_CLOCKS];
	const char *legacy_mode;
	const struct sysc_capabilities *cap;
	struct sysc_config cfg;
	const char *name;
	u32 revision;
};

static u32 sysc_read(struct sysc *ddata, int offset)
{
	if (ddata->cfg.quirks & SYSC_QUIRK_16BIT) {
		u32 val;

		val = readw_relaxed(ddata->module_va + offset);
		val |= (readw_relaxed(ddata->module_va + offset + 4) << 16);

		return val;
	}

	return readl_relaxed(ddata->module_va + offset);
}

static u32 sysc_read_revision(struct sysc *ddata)
{
	int offset = ddata->offsets[SYSC_REVISION];

	if (offset < 0)
		return 0;

	return sysc_read(ddata, offset);
}

static int sysc_get_one_clock(struct sysc *ddata,
			      enum sysc_clocks index)
{
	const char *name;
	int error;

	switch (index) {
	case SYSC_FCK:
		break;
	case SYSC_ICK:
		break;
	default:
		return -EINVAL;
	}
	name = clock_names[index];

	ddata->clocks[index] = devm_clk_get(ddata->dev, name);
	if (IS_ERR(ddata->clocks[index])) {
		if (PTR_ERR(ddata->clocks[index]) == -ENOENT)
			return 0;

		dev_err(ddata->dev, "clock get error for %s: %li\n",
			name, PTR_ERR(ddata->clocks[index]));

		return PTR_ERR(ddata->clocks[index]);
	}

	error = clk_prepare(ddata->clocks[index]);
	if (error) {
		dev_err(ddata->dev, "clock prepare error for %s: %i\n",
			name, error);

		return error;
	}

	return 0;
}

static int sysc_get_clocks(struct sysc *ddata)
{
	int i, error;

	if (ddata->legacy_mode)
		return 0;

	for (i = 0; i < SYSC_MAX_CLOCKS; i++) {
		error = sysc_get_one_clock(ddata, i);
		if (error && error != -ENOENT)
			return error;
	}

	return 0;
}

/**
 * sysc_parse_and_check_child_range - parses module IO region from ranges
 * @ddata: device driver data
 *
 * In general we only need rev, syss, and sysc registers and not the whole
 * module range. But we do want the offsets for these registers from the
 * module base. This allows us to check them against the legacy hwmod
 * platform data. Let's also check the ranges are configured properly.
 */
static int sysc_parse_and_check_child_range(struct sysc *ddata)
{
	struct device_node *np = ddata->dev->of_node;
	const __be32 *ranges;
	u32 nr_addr, nr_size;
	int len, error;

	ranges = of_get_property(np, "ranges", &len);
	if (!ranges) {
		dev_err(ddata->dev, "missing ranges for %pOF\n", np);

		return -ENOENT;
	}

	len /= sizeof(*ranges);

	if (len < 3) {
		dev_err(ddata->dev, "incomplete ranges for %pOF\n", np);

		return -EINVAL;
	}

	error = of_property_read_u32(np, "#address-cells", &nr_addr);
	if (error)
		return -ENOENT;

	error = of_property_read_u32(np, "#size-cells", &nr_size);
	if (error)
		return -ENOENT;

	if (nr_addr != 1 || nr_size != 1) {
		dev_err(ddata->dev, "invalid ranges for %pOF\n", np);

		return -EINVAL;
	}

	ranges++;
	ddata->module_pa = of_translate_address(np, ranges++);
	ddata->module_size = be32_to_cpup(ranges);

	dev_dbg(ddata->dev, "interconnect target 0x%llx size 0x%x for %pOF\n",
		ddata->module_pa, ddata->module_size, np);

	return 0;
}

/**
 * sysc_check_one_child - check child configuration
 * @ddata: device driver data
 * @np: child device node
 *
 * Let's avoid messy situations where we have new interconnect target
 * node but children have "ti,hwmods". These belong to the interconnect
 * target node and are managed by this driver.
 */
static int sysc_check_one_child(struct sysc *ddata,
				struct device_node *np)
{
	const char *name;

	name = of_get_property(np, "ti,hwmods", NULL);
	if (name)
		dev_warn(ddata->dev, "really a child ti,hwmods property?");

	return 0;
}

static int sysc_check_children(struct sysc *ddata)
{
	struct device_node *child;
	int error;

	for_each_child_of_node(ddata->dev->of_node, child) {
		error = sysc_check_one_child(ddata, child);
		if (error)
			return error;
	}

	return 0;
}

/*
 * So far only I2C uses 16-bit read access with clockactivity with revision
 * in two registers with stride of 4. We can detect this based on the rev
 * register size to configure things far enough to be able to properly read
 * the revision register.
 */
static void sysc_check_quirk_16bit(struct sysc *ddata, struct resource *res)
{
	if (resource_size(res) == 8) {
		dev_dbg(ddata->dev,
			"enabling 16-bit and clockactivity quirks\n");
		ddata->cfg.quirks |= SYSC_QUIRK_16BIT | SYSC_QUIRK_USE_CLOCKACT;
	}
}

/**
 * sysc_parse_one - parses the interconnect target module registers
 * @ddata: device driver data
 * @reg: register to parse
 */
static int sysc_parse_one(struct sysc *ddata, enum sysc_registers reg)
{
	struct resource *res;
	const char *name;

	switch (reg) {
	case SYSC_REVISION:
	case SYSC_SYSCONFIG:
	case SYSC_SYSSTATUS:
		name = reg_names[reg];
		break;
	default:
		return -EINVAL;
	}

	res = platform_get_resource_byname(to_platform_device(ddata->dev),
					   IORESOURCE_MEM, name);
	if (!res) {
		dev_dbg(ddata->dev, "has no %s register\n", name);
		ddata->offsets[reg] = -ENODEV;

		return 0;
	}

	ddata->offsets[reg] = res->start - ddata->module_pa;
	if (reg == SYSC_REVISION)
		sysc_check_quirk_16bit(ddata, res);

	return 0;
}

static int sysc_parse_registers(struct sysc *ddata)
{
	int i, error;

	for (i = 0; i < SYSC_MAX_REGS; i++) {
		error = sysc_parse_one(ddata, i);
		if (error)
			return error;
	}

	return 0;
}

/**
 * sysc_check_registers - check for misconfigured register overlaps
 * @ddata: device driver data
 */
static int sysc_check_registers(struct sysc *ddata)
{
	int i, j, nr_regs = 0, nr_matches = 0;

	for (i = 0; i < SYSC_MAX_REGS; i++) {
		if (ddata->offsets[i] < 0)
			continue;

		if (ddata->offsets[i] > (ddata->module_size - 4)) {
			dev_err(ddata->dev, "register outside module range");

				return -EINVAL;
		}

		for (j = 0; j < SYSC_MAX_REGS; j++) {
			if (ddata->offsets[j] < 0)
				continue;

			if (ddata->offsets[i] == ddata->offsets[j])
				nr_matches++;
		}
		nr_regs++;
	}

	if (nr_regs < 1) {
		dev_err(ddata->dev, "missing registers\n");

		return -EINVAL;
	}

	if (nr_matches > nr_regs) {
		dev_err(ddata->dev, "overlapping registers: (%i/%i)",
			nr_regs, nr_matches);

		return -EINVAL;
	}

	return 0;
}

/**
 * syc_ioremap - ioremap register space for the interconnect target module
 * @ddata: deviec driver data
 *
 * Note that the interconnect target module registers can be anywhere
 * within the first child device address space. For example, SGX has
 * them at offset 0x1fc00 in the 32MB module address space. We just
 * what we need around the interconnect target module registers.
 */
static int sysc_ioremap(struct sysc *ddata)
{
	u32 size = 0;

	if (ddata->offsets[SYSC_SYSSTATUS] >= 0)
		size = ddata->offsets[SYSC_SYSSTATUS];
	else if (ddata->offsets[SYSC_SYSCONFIG] >= 0)
		size = ddata->offsets[SYSC_SYSCONFIG];
	else if (ddata->offsets[SYSC_REVISION] >= 0)
		size = ddata->offsets[SYSC_REVISION];
	else
		return -EINVAL;

	size &= 0xfff00;
	size += SZ_256;

	ddata->module_va = devm_ioremap(ddata->dev,
					ddata->module_pa,
					size);
	if (!ddata->module_va)
		return -EIO;

	return 0;
}

/**
 * sysc_map_and_check_registers - ioremap and check device registers
 * @ddata: device driver data
 */
static int sysc_map_and_check_registers(struct sysc *ddata)
{
	int error;

	error = sysc_parse_and_check_child_range(ddata);
	if (error)
		return error;

	error = sysc_check_children(ddata);
	if (error)
		return error;

	error = sysc_parse_registers(ddata);
	if (error)
		return error;

	error = sysc_ioremap(ddata);
	if (error)
		return error;

	error = sysc_check_registers(ddata);
	if (error)
		return error;

	return 0;
}

/**
 * sysc_show_rev - read and show interconnect target module revision
 * @bufp: buffer to print the information to
 * @ddata: device driver data
 */
static int sysc_show_rev(char *bufp, struct sysc *ddata)
{
	int len;

	if (ddata->offsets[SYSC_REVISION] < 0)
		return sprintf(bufp, ":NA");

	len = sprintf(bufp, ":%08x", ddata->revision);

	return len;
}

static int sysc_show_reg(struct sysc *ddata,
			 char *bufp, enum sysc_registers reg)
{
	if (ddata->offsets[reg] < 0)
		return sprintf(bufp, ":NA");

	return sprintf(bufp, ":%x", ddata->offsets[reg]);
}

/**
 * sysc_show_registers - show information about interconnect target module
 * @ddata: device driver data
 */
static void sysc_show_registers(struct sysc *ddata)
{
	char buf[128];
	char *bufp = buf;
	int i;

	for (i = 0; i < SYSC_MAX_REGS; i++)
		bufp += sysc_show_reg(ddata, bufp, i);

	bufp += sysc_show_rev(bufp, ddata);

	dev_dbg(ddata->dev, "%llx:%x%s\n",
		ddata->module_pa, ddata->module_size,
		buf);
}

static int __maybe_unused sysc_runtime_suspend(struct device *dev)
{
	struct sysc *ddata;
	int i;

	ddata = dev_get_drvdata(dev);

	if (ddata->legacy_mode)
		return 0;

	for (i = 0; i < SYSC_MAX_CLOCKS; i++) {
		if (IS_ERR_OR_NULL(ddata->clocks[i]))
			continue;
		clk_disable(ddata->clocks[i]);
	}

	return 0;
}

static int __maybe_unused sysc_runtime_resume(struct device *dev)
{
	struct sysc *ddata;
	int i, error;

	ddata = dev_get_drvdata(dev);

	if (ddata->legacy_mode)
		return 0;

	for (i = 0; i < SYSC_MAX_CLOCKS; i++) {
		if (IS_ERR_OR_NULL(ddata->clocks[i]))
			continue;
		error = clk_enable(ddata->clocks[i]);
		if (error)
			return error;
	}

	return 0;
}

static const struct dev_pm_ops sysc_pm_ops = {
	SET_RUNTIME_PM_OPS(sysc_runtime_suspend,
			   sysc_runtime_resume,
			   NULL)
};

/* At this point the module is configured enough to read the revision */
static int sysc_init_module(struct sysc *ddata)
{
	int error;

	error = pm_runtime_get_sync(ddata->dev);
	if (error < 0) {
		pm_runtime_put_noidle(ddata->dev);

		return 0;
	}
	ddata->revision = sysc_read_revision(ddata);
	pm_runtime_put_sync(ddata->dev);

	return 0;
}

static int sysc_init_sysc_mask(struct sysc *ddata)
{
	struct device_node *np = ddata->dev->of_node;
	int error;
	u32 val;

	error = of_property_read_u32(np, "ti,sysc-mask", &val);
	if (error)
		return 0;

	if (val)
		ddata->cfg.sysc_val = val & ddata->cap->sysc_mask;
	else
		ddata->cfg.sysc_val = ddata->cap->sysc_mask;

	return 0;
}

static int sysc_init_idlemode(struct sysc *ddata, u8 *idlemodes,
			      const char *name)
{
	struct device_node *np = ddata->dev->of_node;
	struct property *prop;
	const __be32 *p;
	u32 val;

	of_property_for_each_u32(np, name, prop, p, val) {
		if (val >= SYSC_NR_IDLEMODES) {
			dev_err(ddata->dev, "invalid idlemode: %i\n", val);
			return -EINVAL;
		}
		*idlemodes |=  (1 << val);
	}

	return 0;
}

static int sysc_init_idlemodes(struct sysc *ddata)
{
	int error;

	error = sysc_init_idlemode(ddata, &ddata->cfg.midlemodes,
				   "ti,sysc-midle");
	if (error)
		return error;

	error = sysc_init_idlemode(ddata, &ddata->cfg.sidlemodes,
				   "ti,sysc-sidle");
	if (error)
		return error;

	return 0;
}

/*
 * Only some devices on omap4 and later have SYSCONFIG reset done
 * bit. We can detect this if there is no SYSSTATUS at all, or the
 * SYSTATUS bit 0 is not used. Note that some SYSSTATUS registers
 * have multiple bits for the child devices like OHCI and EHCI.
 * Depends on SYSC being parsed first.
 */
static int sysc_init_syss_mask(struct sysc *ddata)
{
	struct device_node *np = ddata->dev->of_node;
	int error;
	u32 val;

	error = of_property_read_u32(np, "ti,syss-mask", &val);
	if (error) {
		if ((ddata->cap->type == TI_SYSC_OMAP4 ||
		     ddata->cap->type == TI_SYSC_OMAP4_TIMER) &&
		    (ddata->cfg.sysc_val & SYSC_OMAP4_SOFTRESET))
			ddata->cfg.quirks |= SYSC_QUIRK_RESET_STATUS;

		return 0;
	}

	if (!(val & 1) && (ddata->cfg.sysc_val & SYSC_OMAP4_SOFTRESET))
		ddata->cfg.quirks |= SYSC_QUIRK_RESET_STATUS;

	ddata->cfg.syss_mask = val;

	return 0;
}

/* Device tree configured quirks */
struct sysc_dts_quirk {
	const char *name;
	u32 mask;
};

static const struct sysc_dts_quirk sysc_dts_quirks[] = {
	{ .name = "ti,no-idle-on-init",
	  .mask = SYSC_QUIRK_NO_IDLE_ON_INIT, },
	{ .name = "ti,no-reset-on-init",
	  .mask = SYSC_QUIRK_NO_RESET_ON_INIT, },
};

static int sysc_init_dts_quirks(struct sysc *ddata)
{
	struct device_node *np = ddata->dev->of_node;
	const struct property *prop;
	int i, len, error;
	u32 val;

	ddata->legacy_mode = of_get_property(np, "ti,hwmods", NULL);

	for (i = 0; i < ARRAY_SIZE(sysc_dts_quirks); i++) {
		prop = of_get_property(np, sysc_dts_quirks[i].name, &len);
		if (!prop)
			break;

		ddata->cfg.quirks |= sysc_dts_quirks[i].mask;
	}

	error = of_property_read_u32(np, "ti,sysc-delay-us", &val);
	if (!error) {
		if (val > 255) {
			dev_warn(ddata->dev, "bad ti,sysc-delay-us: %i\n",
				 val);
		}

		ddata->cfg.srst_udelay = (u8)val;
	}

	return 0;
}

static void sysc_unprepare(struct sysc *ddata)
{
	int i;

	for (i = 0; i < SYSC_MAX_CLOCKS; i++) {
		if (!IS_ERR_OR_NULL(ddata->clocks[i]))
			clk_unprepare(ddata->clocks[i]);
	}
}

/*
 * Common sysc register bits found on omap2, also known as type1
 */
static const struct sysc_regbits sysc_regbits_omap2 = {
	.dmadisable_shift = -ENODEV,
	.midle_shift = 12,
	.sidle_shift = 3,
	.clkact_shift = 8,
	.emufree_shift = 5,
	.enwkup_shift = 2,
	.srst_shift = 1,
	.autoidle_shift = 0,
};

static const struct sysc_capabilities sysc_omap2 = {
	.type = TI_SYSC_OMAP2,
	.sysc_mask = SYSC_OMAP2_CLOCKACTIVITY | SYSC_OMAP2_EMUFREE |
		     SYSC_OMAP2_ENAWAKEUP | SYSC_OMAP2_SOFTRESET |
		     SYSC_OMAP2_AUTOIDLE,
	.regbits = &sysc_regbits_omap2,
};

/* All omap2 and 3 timers, and timers 1, 2 & 10 on omap 4 and 5 */
static const struct sysc_capabilities sysc_omap2_timer = {
	.type = TI_SYSC_OMAP2_TIMER,
	.sysc_mask = SYSC_OMAP2_CLOCKACTIVITY | SYSC_OMAP2_EMUFREE |
		     SYSC_OMAP2_ENAWAKEUP | SYSC_OMAP2_SOFTRESET |
		     SYSC_OMAP2_AUTOIDLE,
	.regbits = &sysc_regbits_omap2,
	.mod_quirks = SYSC_QUIRK_USE_CLOCKACT,
};

/*
 * SHAM2 (SHA1/MD5) sysc found on omap3, a variant of sysc_regbits_omap2
 * with different sidle position
 */
static const struct sysc_regbits sysc_regbits_omap3_sham = {
	.dmadisable_shift = -ENODEV,
	.midle_shift = -ENODEV,
	.sidle_shift = 4,
	.clkact_shift = -ENODEV,
	.enwkup_shift = -ENODEV,
	.srst_shift = 1,
	.autoidle_shift = 0,
	.emufree_shift = -ENODEV,
};

static const struct sysc_capabilities sysc_omap3_sham = {
	.type = TI_SYSC_OMAP3_SHAM,
	.sysc_mask = SYSC_OMAP2_SOFTRESET | SYSC_OMAP2_AUTOIDLE,
	.regbits = &sysc_regbits_omap3_sham,
};

/*
 * AES register bits found on omap3 and later, a variant of
 * sysc_regbits_omap2 with different sidle position
 */
static const struct sysc_regbits sysc_regbits_omap3_aes = {
	.dmadisable_shift = -ENODEV,
	.midle_shift = -ENODEV,
	.sidle_shift = 6,
	.clkact_shift = -ENODEV,
	.enwkup_shift = -ENODEV,
	.srst_shift = 1,
	.autoidle_shift = 0,
	.emufree_shift = -ENODEV,
};

static const struct sysc_capabilities sysc_omap3_aes = {
	.type = TI_SYSC_OMAP3_AES,
	.sysc_mask = SYSC_OMAP2_SOFTRESET | SYSC_OMAP2_AUTOIDLE,
	.regbits = &sysc_regbits_omap3_aes,
};

/*
 * Common sysc register bits found on omap4, also known as type2
 */
static const struct sysc_regbits sysc_regbits_omap4 = {
	.dmadisable_shift = 16,
	.midle_shift = 4,
	.sidle_shift = 2,
	.clkact_shift = -ENODEV,
	.enwkup_shift = -ENODEV,
	.emufree_shift = 1,
	.srst_shift = 0,
	.autoidle_shift = -ENODEV,
};

static const struct sysc_capabilities sysc_omap4 = {
	.type = TI_SYSC_OMAP4,
	.sysc_mask = SYSC_OMAP4_DMADISABLE | SYSC_OMAP4_FREEEMU |
		     SYSC_OMAP4_SOFTRESET,
	.regbits = &sysc_regbits_omap4,
};

static const struct sysc_capabilities sysc_omap4_timer = {
	.type = TI_SYSC_OMAP4_TIMER,
	.sysc_mask = SYSC_OMAP4_DMADISABLE | SYSC_OMAP4_FREEEMU |
		     SYSC_OMAP4_SOFTRESET,
	.regbits = &sysc_regbits_omap4,
};

/*
 * Common sysc register bits found on omap4, also known as type3
 */
static const struct sysc_regbits sysc_regbits_omap4_simple = {
	.dmadisable_shift = -ENODEV,
	.midle_shift = 2,
	.sidle_shift = 0,
	.clkact_shift = -ENODEV,
	.enwkup_shift = -ENODEV,
	.srst_shift = -ENODEV,
	.emufree_shift = -ENODEV,
	.autoidle_shift = -ENODEV,
};

static const struct sysc_capabilities sysc_omap4_simple = {
	.type = TI_SYSC_OMAP4_SIMPLE,
	.regbits = &sysc_regbits_omap4_simple,
};

/*
 * SmartReflex sysc found on omap34xx
 */
static const struct sysc_regbits sysc_regbits_omap34xx_sr = {
	.dmadisable_shift = -ENODEV,
	.midle_shift = -ENODEV,
	.sidle_shift = -ENODEV,
	.clkact_shift = 20,
	.enwkup_shift = -ENODEV,
	.srst_shift = -ENODEV,
	.emufree_shift = -ENODEV,
	.autoidle_shift = -ENODEV,
};

static const struct sysc_capabilities sysc_34xx_sr = {
	.type = TI_SYSC_OMAP34XX_SR,
	.sysc_mask = SYSC_OMAP2_CLOCKACTIVITY,
	.regbits = &sysc_regbits_omap34xx_sr,
	.mod_quirks = SYSC_QUIRK_USE_CLOCKACT | SYSC_QUIRK_UNCACHED,
};

/*
 * SmartReflex sysc found on omap36xx and later
 */
static const struct sysc_regbits sysc_regbits_omap36xx_sr = {
	.dmadisable_shift = -ENODEV,
	.midle_shift = -ENODEV,
	.sidle_shift = 24,
	.clkact_shift = -ENODEV,
	.enwkup_shift = 26,
	.srst_shift = -ENODEV,
	.emufree_shift = -ENODEV,
	.autoidle_shift = -ENODEV,
};

static const struct sysc_capabilities sysc_36xx_sr = {
	.type = TI_SYSC_OMAP36XX_SR,
	.sysc_mask = SYSC_OMAP3_SR_ENAWAKEUP,
	.regbits = &sysc_regbits_omap36xx_sr,
	.mod_quirks = SYSC_QUIRK_UNCACHED,
};

static const struct sysc_capabilities sysc_omap4_sr = {
	.type = TI_SYSC_OMAP4_SR,
	.regbits = &sysc_regbits_omap36xx_sr,
};

/*
 * McASP register bits found on omap4 and later
 */
static const struct sysc_regbits sysc_regbits_omap4_mcasp = {
	.dmadisable_shift = -ENODEV,
	.midle_shift = -ENODEV,
	.sidle_shift = 0,
	.clkact_shift = -ENODEV,
	.enwkup_shift = -ENODEV,
	.srst_shift = -ENODEV,
	.emufree_shift = -ENODEV,
	.autoidle_shift = -ENODEV,
};

static const struct sysc_capabilities sysc_omap4_mcasp = {
	.type = TI_SYSC_OMAP4_MCASP,
	.regbits = &sysc_regbits_omap4_mcasp,
};

/*
 * FS USB host found on omap4 and later
 */
static const struct sysc_regbits sysc_regbits_omap4_usb_host_fs = {
	.dmadisable_shift = -ENODEV,
	.midle_shift = -ENODEV,
	.sidle_shift = 24,
	.clkact_shift = -ENODEV,
	.enwkup_shift = 26,
	.srst_shift = -ENODEV,
	.emufree_shift = -ENODEV,
	.autoidle_shift = -ENODEV,
};

static const struct sysc_capabilities sysc_omap4_usb_host_fs = {
	.type = TI_SYSC_OMAP4_USB_HOST_FS,
	.sysc_mask = SYSC_OMAP2_ENAWAKEUP,
	.regbits = &sysc_regbits_omap4_usb_host_fs,
};

static int sysc_init_match(struct sysc *ddata)
{
	const struct sysc_capabilities *cap;

	cap = of_device_get_match_data(ddata->dev);
	if (!cap)
		return -EINVAL;

	ddata->cap = cap;
	if (ddata->cap)
		ddata->cfg.quirks |= ddata->cap->mod_quirks;

	return 0;
}

static int sysc_probe(struct platform_device *pdev)
{
	struct sysc *ddata;
	int error;

	ddata = devm_kzalloc(&pdev->dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	ddata->dev = &pdev->dev;
	platform_set_drvdata(pdev, ddata);

	error = sysc_init_match(ddata);
	if (error)
		return error;

	error = sysc_init_dts_quirks(ddata);
	if (error)
		goto unprepare;

	error = sysc_get_clocks(ddata);
	if (error)
		return error;

	error = sysc_map_and_check_registers(ddata);
	if (error)
		goto unprepare;

	error = sysc_init_sysc_mask(ddata);
	if (error)
		goto unprepare;

	error = sysc_init_idlemodes(ddata);
	if (error)
		goto unprepare;

	error = sysc_init_syss_mask(ddata);
	if (error)
		goto unprepare;

	pm_runtime_enable(ddata->dev);

	error = sysc_init_module(ddata);
	if (error)
		goto unprepare;

	error = pm_runtime_get_sync(ddata->dev);
	if (error < 0) {
		pm_runtime_put_noidle(ddata->dev);
		pm_runtime_disable(ddata->dev);
		goto unprepare;
	}

	pm_runtime_use_autosuspend(ddata->dev);

	sysc_show_registers(ddata);

	error = of_platform_populate(ddata->dev->of_node,
				     NULL, NULL, ddata->dev);
	if (error)
		goto err;

	pm_runtime_mark_last_busy(ddata->dev);
	pm_runtime_put_autosuspend(ddata->dev);

	return 0;

err:
	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
unprepare:
	sysc_unprepare(ddata);

	return error;
}

static int sysc_remove(struct platform_device *pdev)
{
	struct sysc *ddata = platform_get_drvdata(pdev);
	int error;

	error = pm_runtime_get_sync(ddata->dev);
	if (error < 0) {
		pm_runtime_put_noidle(ddata->dev);
		pm_runtime_disable(ddata->dev);
		goto unprepare;
	}

	of_platform_depopulate(&pdev->dev);

	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

unprepare:
	sysc_unprepare(ddata);

	return 0;
}

static const struct of_device_id sysc_match[] = {
	{ .compatible = "ti,sysc-omap2", .data = &sysc_omap2, },
	{ .compatible = "ti,sysc-omap2-timer", .data = &sysc_omap2_timer, },
	{ .compatible = "ti,sysc-omap4", .data = &sysc_omap4, },
	{ .compatible = "ti,sysc-omap4-timer", .data = &sysc_omap4_timer, },
	{ .compatible = "ti,sysc-omap4-simple", .data = &sysc_omap4_simple, },
	{ .compatible = "ti,sysc-omap3430-sr", .data = &sysc_34xx_sr, },
	{ .compatible = "ti,sysc-omap3630-sr", .data = &sysc_36xx_sr, },
	{ .compatible = "ti,sysc-omap4-sr", .data = &sysc_omap4_sr, },
	{ .compatible = "ti,sysc-omap3-sham", .data = &sysc_omap3_sham, },
	{ .compatible = "ti,sysc-omap-aes", .data = &sysc_omap3_aes, },
	{ .compatible = "ti,sysc-mcasp", .data = &sysc_omap4_mcasp, },
	{ .compatible = "ti,sysc-usb-host-fs",
	  .data = &sysc_omap4_usb_host_fs, },
	{  },
};
MODULE_DEVICE_TABLE(of, sysc_match);

static struct platform_driver sysc_driver = {
	.probe		= sysc_probe,
	.remove		= sysc_remove,
	.driver         = {
		.name   = "ti-sysc",
		.of_match_table	= sysc_match,
		.pm = &sysc_pm_ops,
	},
};
module_platform_driver(sysc_driver);

MODULE_DESCRIPTION("TI sysc interconnect target driver");
MODULE_LICENSE("GPL v2");
