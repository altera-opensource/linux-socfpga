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
#include <linux/of_address.h>

#include "clk.h"

/* Clock Manager offsets */
#define CLK_MGR_PLL_CLK_SRC_SHIFT	16
#define CLK_MGR_PLL_CLK_SRC_MASK	0x3

/* PLL Clock enable bits */
#define SOCFPGA_PLL_POWER		0
#define SOCFPGA_PLL_RESET_MASK		0x2
#define SOCFPGA_PLL_REFDIV_MASK		0x00003F00
#define SOCFPGA_PLL_REFDIV_SHIFT	8
#define SOCFPGA_PLL_MDIV_MASK		0xFF000000
#define SOCFPGA_PLL_MDIV_SHIFT		24
#define SOCFGPA_MAX_PARENTS	5

#define to_socfpga_clk(p) container_of(p, struct socfpga_pll, hw.hw)

void __iomem *clk_mgr_s10_base_addr;

static unsigned long clk_pll_recalc_rate(struct clk_hw *hwclk,
					 unsigned long parent_rate)
{
	struct socfpga_pll *socfpgaclk = to_socfpga_clk(hwclk);
	unsigned long mdiv;
	unsigned long refdiv;
	unsigned long reg;
	unsigned long long vco_freq;

	/* read VCO1 reg for numerator and denominator */
	reg = readl(socfpgaclk->hw.reg);
	refdiv = (reg & SOCFPGA_PLL_REFDIV_MASK) >> SOCFPGA_PLL_REFDIV_SHIFT;
	vco_freq = (unsigned long long)parent_rate / refdiv;

	/* Read mdiv and fdiv from the fdbck register */
	reg = readl(socfpgaclk->hw.reg + 0x4);
	mdiv = (reg & SOCFPGA_PLL_MDIV_MASK) >> SOCFPGA_PLL_MDIV_SHIFT;
	vco_freq = (unsigned long long)parent_rate * (mdiv + 6);

	return (unsigned long)vco_freq;
}

static u8 clk_pll_get_parent(struct clk_hw *hwclk)
{
	struct socfpga_pll *socfpgaclk = to_socfpga_clk(hwclk);
	u32 pll_src;

	pll_src = readl(socfpgaclk->hw.reg);

	return (pll_src >> CLK_MGR_PLL_CLK_SRC_SHIFT) &
		CLK_MGR_PLL_CLK_SRC_MASK;
}

static int clk_pll_prepare(struct clk_hw *hwclk)
{
	struct socfpga_pll *socfpgaclk = to_socfpga_clk(hwclk);
	u32 reg;

	/* Bring PLL out of reset */
	reg = readl(socfpgaclk->hw.reg);
	reg |= SOCFPGA_PLL_RESET_MASK;
	writel(reg, socfpgaclk->hw.reg);

	return 0;
}

static struct clk_ops clk_pll_ops = {
	.recalc_rate = clk_pll_recalc_rate,
	.get_parent = clk_pll_get_parent,
	.prepare = clk_pll_prepare,
};

static struct clk * __init __socfpga_pll_init(struct device_node *node,
					      const struct clk_ops *ops)
{
	u32 reg, reg2;
	struct clk *clk;
	struct socfpga_pll *pll_clk;
	const char *clk_name = node->name;
	const char *parent_names[SOCFGPA_MAX_PARENTS];
	struct clk_init_data init;
	struct device_node *clkmgr_np;
	int rc;
	int i = 0;

	of_property_read_u32(node, "reg", &reg);

	pll_clk = kzalloc(sizeof(*pll_clk), GFP_KERNEL);
	if (WARN_ON(!pll_clk))
		return NULL;

	clkmgr_np = of_find_compatible_node(NULL, NULL, "altr,clk-mgr");
	clk_mgr_s10_base_addr = of_iomap(clkmgr_np, 0);
	BUG_ON(!clk_mgr_s10_base_addr);
	pll_clk->hw.reg = clk_mgr_s10_base_addr + reg;

	of_property_read_string(node, "clock-output-names", &clk_name);

	init.name = clk_name;
	init.ops = ops;
	init.flags = 0;

	while (i < SOCFGPA_MAX_PARENTS && (parent_names[i] =
			of_clk_get_parent_name(node, i)) != NULL)
		i++;
	init.num_parents = i;
	init.parent_names = parent_names;
	pll_clk->hw.hw.init = &init;

	pll_clk->hw.bit_idx = SOCFPGA_PLL_POWER;
	clk_pll_ops.enable = clk_gate_ops.enable;
	clk_pll_ops.disable = clk_gate_ops.disable;

	clk = clk_register(NULL, &pll_clk->hw.hw);
	if (WARN_ON(IS_ERR(clk))) {
		kfree(pll_clk);
		return NULL;
	}
	rc = of_clk_add_provider(node, of_clk_src_simple_get, clk);
	return clk;
}

void __init socfpga_s10_pll_init(struct device_node *node)
{
	__socfpga_pll_init(node, &clk_pll_ops);
}
