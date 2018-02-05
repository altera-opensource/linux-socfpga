/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __QCOM_CLK_ALPHA_PLL_H__
#define __QCOM_CLK_ALPHA_PLL_H__

#include <linux/clk-provider.h>
#include "clk-regmap.h"

/* Alpha PLL types */
enum {
	CLK_ALPHA_PLL_TYPE_DEFAULT,
	CLK_ALPHA_PLL_TYPE_HUAYRA,
	CLK_ALPHA_PLL_TYPE_BRAMMO,
	CLK_ALPHA_PLL_TYPE_MAX,
};

enum {
	PLL_OFF_L_VAL,
	PLL_OFF_ALPHA_VAL,
	PLL_OFF_ALPHA_VAL_U,
	PLL_OFF_USER_CTL,
	PLL_OFF_USER_CTL_U,
	PLL_OFF_CONFIG_CTL,
	PLL_OFF_CONFIG_CTL_U,
	PLL_OFF_TEST_CTL,
	PLL_OFF_TEST_CTL_U,
	PLL_OFF_STATUS,
	PLL_OFF_MAX_REGS
};

extern const u8 clk_alpha_pll_regs[CLK_ALPHA_PLL_TYPE_MAX][PLL_OFF_MAX_REGS];

struct pll_vco {
	unsigned long min_freq;
	unsigned long max_freq;
	u32 val;
};

/**
 * struct clk_alpha_pll - phase locked loop (PLL)
 * @offset: base address of registers
 * @vco_table: array of VCO settings
 * @regs: alpha pll register map (see @clk_alpha_pll_regs)
 * @clkr: regmap clock handle
 */
struct clk_alpha_pll {
	u32 offset;
	const u8 *regs;

	const struct pll_vco *vco_table;
	size_t num_vco;
#define SUPPORTS_OFFLINE_REQ	BIT(0)
#define SUPPORTS_FSM_MODE	BIT(2)
#define SUPPORTS_DYNAMIC_UPDATE	BIT(3)
	u8 flags;

	struct clk_regmap clkr;
};

/**
 * struct clk_alpha_pll_postdiv - phase locked loop (PLL) post-divider
 * @offset: base address of registers
 * @regs: alpha pll register map (see @clk_alpha_pll_regs)
 * @width: width of post-divider
 * @clkr: regmap clock handle
 */
struct clk_alpha_pll_postdiv {
	u32 offset;
	u8 width;
	const u8 *regs;

	struct clk_regmap clkr;
};

struct alpha_pll_config {
	u32 l;
	u32 alpha;
	u32 alpha_hi;
	u32 config_ctl_val;
	u32 config_ctl_hi_val;
	u32 main_output_mask;
	u32 aux_output_mask;
	u32 aux2_output_mask;
	u32 early_output_mask;
	u32 alpha_en_mask;
	u32 alpha_mode_mask;
	u32 pre_div_val;
	u32 pre_div_mask;
	u32 post_div_val;
	u32 post_div_mask;
	u32 vco_val;
	u32 vco_mask;
};

extern const struct clk_ops clk_alpha_pll_ops;
extern const struct clk_ops clk_alpha_pll_hwfsm_ops;
extern const struct clk_ops clk_alpha_pll_postdiv_ops;
extern const struct clk_ops clk_alpha_pll_huayra_ops;
extern const struct clk_ops clk_alpha_pll_postdiv_ro_ops;

void clk_alpha_pll_configure(struct clk_alpha_pll *pll, struct regmap *regmap,
			     const struct alpha_pll_config *config);

#endif
