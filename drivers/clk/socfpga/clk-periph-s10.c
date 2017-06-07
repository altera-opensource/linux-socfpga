/*
 * Copyright (C) 2017, Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/slab.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of.h>

#include "clk.h"

#define CLK_MGR_FREE_SHIFT		16
#define CLK_MGR_FREE_MASK		0x7

#define SWCTRLBTCLKSEL_MASK		0x200
#define SWCTRLBTCLKSEL_SHIFT		9
#define SWCTRLBTCLKEN_MASK		0x100
#define SWCTRLBTCLKSEN_SHIFT		8
#define SOCFPGA_BOOT_CLK		"boot_clk"

#define to_socfpga_periph_clk(p) container_of(p, struct socfpga_periph_clk, hw.hw)

static unsigned long clk_periclk_recalc_rate(struct clk_hw *hwclk,
					     unsigned long parent_rate)
{
	struct socfpga_periph_clk *socfpgaclk = to_socfpga_periph_clk(hwclk);
	unsigned long div = 1;
	u32 val;

	if (socfpgaclk->fixed_div) {
		div = socfpgaclk->fixed_div;
	} else if (socfpgaclk->div_reg) {
		val = readl(socfpgaclk->div_reg) >> socfpgaclk->shift;
		val &= GENMASK(socfpgaclk->width - 1, 0);
		parent_rate /= val;
	} else {
		if (streq(hwclk->init->name, SOCFPGA_BOOT_CLK)) {
			if ((readl(socfpgaclk->hw.reg) & SWCTRLBTCLKEN_MASK) >>
			     SWCTRLBTCLKSEN_SHIFT) {
				div = ((readl(socfpgaclk->hw.reg) &
					SWCTRLBTCLKSEL_MASK) >>
					SWCTRLBTCLKSEL_SHIFT);
				div += 1;
			}
		} else if (!socfpgaclk->bypass_reg)
			div = ((readl(socfpgaclk->hw.reg) & 0x7ff) + 1);
	}

	return parent_rate / div;
}

static u8 clk_periclk_get_parent(struct clk_hw *hwclk)
{
	struct socfpga_periph_clk *socfpgaclk = to_socfpga_periph_clk(hwclk);
	u32 clk_src, mask;
	u8 parent;

	if (socfpgaclk->bypass_reg) {
		mask = (0x1 << socfpgaclk->bypass_shift);
		parent = ((readl(socfpgaclk->bypass_reg) & mask) >>
			   socfpgaclk->bypass_shift);
	} else {
		clk_src = readl(socfpgaclk->hw.reg);
		parent = (clk_src >> CLK_MGR_FREE_SHIFT) &
			CLK_MGR_FREE_MASK;
	}
	return parent;
}

static const struct clk_ops periclk_ops = {
	.recalc_rate = clk_periclk_recalc_rate,
	.get_parent = clk_periclk_get_parent,
};

static __init void __socfpga_periph_init(struct device_node *node,
					 const struct clk_ops *ops)
{
	u32 reg;
	struct clk *clk;
	struct socfpga_periph_clk *periph_clk;
	const char *clk_name = node->name;
	const char *parent_names[SOCFPGA_MAX_PARENTS];
	struct clk_init_data init;
	int rc;
	u32 fixed_div;
	u32 div_reg[3];
	u32 bypass_reg[2];

	of_property_read_u32(node, "reg", &reg);

	periph_clk = kzalloc(sizeof(*periph_clk), GFP_KERNEL);
	if (WARN_ON(!periph_clk))
		return;

	periph_clk->hw.reg = clk_mgr_s10_base_addr + reg;

	rc = of_property_read_u32_array(node, "div-reg", div_reg, 3);
	if (!rc) {
		periph_clk->div_reg = clk_mgr_s10_base_addr + div_reg[0];
		periph_clk->shift = div_reg[1];
		periph_clk->width = div_reg[2];
	} else {
		periph_clk->div_reg = NULL;
	}

	rc = of_property_read_u32_array(node, "bypass-reg", bypass_reg, 2);
	if (!rc) {
		periph_clk->bypass_reg = clk_mgr_s10_base_addr + bypass_reg[0];
		periph_clk->bypass_shift = bypass_reg[1];
	} else {
		periph_clk->bypass_reg = NULL;
	}

	rc = of_property_read_u32(node, "fixed-divider", &fixed_div);
	if (rc)
		periph_clk->fixed_div = 0;
	else
		periph_clk->fixed_div = fixed_div;

	of_property_read_string(node, "clock-output-names", &clk_name);

	init.name = clk_name;
	init.ops = ops;
	init.flags = 0;

	init.num_parents = of_clk_parent_fill(node, parent_names, SOCFPGA_MAX_PARENTS);
	init.parent_names = parent_names;

	periph_clk->hw.hw.init = &init;

	clk = clk_register(NULL, &periph_clk->hw.hw);
	if (WARN_ON(IS_ERR(clk))) {
		kfree(periph_clk);
		return;
	}
	rc = of_clk_add_provider(node, of_clk_src_simple_get, clk);
	if (rc < 0) {
		pr_err("Could not register clock provider for node:%s\n",
		       clk_name);
		goto err_clk;
	}

	return;

err_clk:
	clk_unregister(clk);
}

void __init socfpga_s10_periph_init(struct device_node *node)
{
	__socfpga_periph_init(node, &periclk_ops);
}
