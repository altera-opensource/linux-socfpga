// SPDX-License-Identifier: GPL-2.0
/*
 * This file is part of STM32 ADC driver
 *
 * Copyright (C) 2016, STMicroelectronics - All Rights Reserved
 * Author: Fabrice Gasnier <fabrice.gasnier@st.com>.
 *
 * Inspired from: fsl-imx25-tsadc
 *
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdesc.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include "stm32-adc-core.h"

/* STM32F4 - common registers for all ADC instances: 1, 2 & 3 */
#define STM32F4_ADC_CSR			(STM32_ADCX_COMN_OFFSET + 0x00)
#define STM32F4_ADC_CCR			(STM32_ADCX_COMN_OFFSET + 0x04)

/* STM32F4_ADC_CSR - bit fields */
#define STM32F4_EOC3			BIT(17)
#define STM32F4_EOC2			BIT(9)
#define STM32F4_EOC1			BIT(1)

/* STM32F4_ADC_CCR - bit fields */
#define STM32F4_ADC_ADCPRE_SHIFT	16
#define STM32F4_ADC_ADCPRE_MASK		GENMASK(17, 16)

/* STM32 F4 maximum analog clock rate (from datasheet) */
#define STM32F4_ADC_MAX_CLK_RATE	36000000

/* STM32H7 - common registers for all ADC instances */
#define STM32H7_ADC_CSR			(STM32_ADCX_COMN_OFFSET + 0x00)
#define STM32H7_ADC_CCR			(STM32_ADCX_COMN_OFFSET + 0x08)

/* STM32H7_ADC_CSR - bit fields */
#define STM32H7_EOC_SLV			BIT(18)
#define STM32H7_EOC_MST			BIT(2)

/* STM32H7_ADC_CCR - bit fields */
#define STM32H7_PRESC_SHIFT		18
#define STM32H7_PRESC_MASK		GENMASK(21, 18)
#define STM32H7_CKMODE_SHIFT		16
#define STM32H7_CKMODE_MASK		GENMASK(17, 16)

/* STM32 H7 maximum analog clock rate (from datasheet) */
#define STM32H7_ADC_MAX_CLK_RATE	36000000

/**
 * stm32_adc_common_regs - stm32 common registers, compatible dependent data
 * @csr:	common status register offset
 * @eoc1:	adc1 end of conversion flag in @csr
 * @eoc2:	adc2 end of conversion flag in @csr
 * @eoc3:	adc3 end of conversion flag in @csr
 */
struct stm32_adc_common_regs {
	u32 csr;
	u32 eoc1_msk;
	u32 eoc2_msk;
	u32 eoc3_msk;
};

struct stm32_adc_priv;

/**
 * stm32_adc_priv_cfg - stm32 core compatible configuration data
 * @regs:	common registers for all instances
 * @clk_sel:	clock selection routine
 */
struct stm32_adc_priv_cfg {
	const struct stm32_adc_common_regs *regs;
	int (*clk_sel)(struct platform_device *, struct stm32_adc_priv *);
};

/**
 * struct stm32_adc_priv - stm32 ADC core private data
 * @irq:		irq for ADC block
 * @domain:		irq domain reference
 * @aclk:		clock reference for the analog circuitry
 * @bclk:		bus clock common for all ADCs, depends on part used
 * @vref:		regulator reference
 * @cfg:		compatible configuration data
 * @common:		common data for all ADC instances
 */
struct stm32_adc_priv {
	int				irq;
	struct irq_domain		*domain;
	struct clk			*aclk;
	struct clk			*bclk;
	struct regulator		*vref;
	const struct stm32_adc_priv_cfg	*cfg;
	struct stm32_adc_common		common;
};

static struct stm32_adc_priv *to_stm32_adc_priv(struct stm32_adc_common *com)
{
	return container_of(com, struct stm32_adc_priv, common);
}

/* STM32F4 ADC internal common clock prescaler division ratios */
static int stm32f4_pclk_div[] = {2, 4, 6, 8};

/**
 * stm32f4_adc_clk_sel() - Select stm32f4 ADC common clock prescaler
 * @priv: stm32 ADC core private data
 * Select clock prescaler used for analog conversions, before using ADC.
 */
static int stm32f4_adc_clk_sel(struct platform_device *pdev,
			       struct stm32_adc_priv *priv)
{
	unsigned long rate;
	u32 val;
	int i;

	/* stm32f4 has one clk input for analog (mandatory), enforce it here */
	if (!priv->aclk) {
		dev_err(&pdev->dev, "No 'adc' clock found\n");
		return -ENOENT;
	}

	rate = clk_get_rate(priv->aclk);
	if (!rate) {
		dev_err(&pdev->dev, "Invalid clock rate: 0\n");
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(stm32f4_pclk_div); i++) {
		if ((rate / stm32f4_pclk_div[i]) <= STM32F4_ADC_MAX_CLK_RATE)
			break;
	}
	if (i >= ARRAY_SIZE(stm32f4_pclk_div)) {
		dev_err(&pdev->dev, "adc clk selection failed\n");
		return -EINVAL;
	}

	priv->common.rate = rate / stm32f4_pclk_div[i];
	val = readl_relaxed(priv->common.base + STM32F4_ADC_CCR);
	val &= ~STM32F4_ADC_ADCPRE_MASK;
	val |= i << STM32F4_ADC_ADCPRE_SHIFT;
	writel_relaxed(val, priv->common.base + STM32F4_ADC_CCR);

	dev_dbg(&pdev->dev, "Using analog clock source at %ld kHz\n",
		priv->common.rate / 1000);

	return 0;
}

/**
 * struct stm32h7_adc_ck_spec - specification for stm32h7 adc clock
 * @ckmode: ADC clock mode, Async or sync with prescaler.
 * @presc: prescaler bitfield for async clock mode
 * @div: prescaler division ratio
 */
struct stm32h7_adc_ck_spec {
	u32 ckmode;
	u32 presc;
	int div;
};

static const struct stm32h7_adc_ck_spec stm32h7_adc_ckmodes_spec[] = {
	/* 00: CK_ADC[1..3]: Asynchronous clock modes */
	{ 0, 0, 1 },
	{ 0, 1, 2 },
	{ 0, 2, 4 },
	{ 0, 3, 6 },
	{ 0, 4, 8 },
	{ 0, 5, 10 },
	{ 0, 6, 12 },
	{ 0, 7, 16 },
	{ 0, 8, 32 },
	{ 0, 9, 64 },
	{ 0, 10, 128 },
	{ 0, 11, 256 },
	/* HCLK used: Synchronous clock modes (1, 2 or 4 prescaler) */
	{ 1, 0, 1 },
	{ 2, 0, 2 },
	{ 3, 0, 4 },
};

static int stm32h7_adc_clk_sel(struct platform_device *pdev,
			       struct stm32_adc_priv *priv)
{
	u32 ckmode, presc, val;
	unsigned long rate;
	int i, div;

	/* stm32h7 bus clock is common for all ADC instances (mandatory) */
	if (!priv->bclk) {
		dev_err(&pdev->dev, "No 'bus' clock found\n");
		return -ENOENT;
	}

	/*
	 * stm32h7 can use either 'bus' or 'adc' clock for analog circuitry.
	 * So, choice is to have bus clock mandatory and adc clock optional.
	 * If optional 'adc' clock has been found, then try to use it first.
	 */
	if (priv->aclk) {
		/*
		 * Asynchronous clock modes (e.g. ckmode == 0)
		 * From spec: PLL output musn't exceed max rate
		 */
		rate = clk_get_rate(priv->aclk);
		if (!rate) {
			dev_err(&pdev->dev, "Invalid adc clock rate: 0\n");
			return -EINVAL;
		}

		for (i = 0; i < ARRAY_SIZE(stm32h7_adc_ckmodes_spec); i++) {
			ckmode = stm32h7_adc_ckmodes_spec[i].ckmode;
			presc = stm32h7_adc_ckmodes_spec[i].presc;
			div = stm32h7_adc_ckmodes_spec[i].div;

			if (ckmode)
				continue;

			if ((rate / div) <= STM32H7_ADC_MAX_CLK_RATE)
				goto out;
		}
	}

	/* Synchronous clock modes (e.g. ckmode is 1, 2 or 3) */
	rate = clk_get_rate(priv->bclk);
	if (!rate) {
		dev_err(&pdev->dev, "Invalid bus clock rate: 0\n");
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(stm32h7_adc_ckmodes_spec); i++) {
		ckmode = stm32h7_adc_ckmodes_spec[i].ckmode;
		presc = stm32h7_adc_ckmodes_spec[i].presc;
		div = stm32h7_adc_ckmodes_spec[i].div;

		if (!ckmode)
			continue;

		if ((rate / div) <= STM32H7_ADC_MAX_CLK_RATE)
			goto out;
	}

	dev_err(&pdev->dev, "adc clk selection failed\n");
	return -EINVAL;

out:
	/* rate used later by each ADC instance to control BOOST mode */
	priv->common.rate = rate / div;

	/* Set common clock mode and prescaler */
	val = readl_relaxed(priv->common.base + STM32H7_ADC_CCR);
	val &= ~(STM32H7_CKMODE_MASK | STM32H7_PRESC_MASK);
	val |= ckmode << STM32H7_CKMODE_SHIFT;
	val |= presc << STM32H7_PRESC_SHIFT;
	writel_relaxed(val, priv->common.base + STM32H7_ADC_CCR);

	dev_dbg(&pdev->dev, "Using %s clock/%d source at %ld kHz\n",
		ckmode ? "bus" : "adc", div, priv->common.rate / 1000);

	return 0;
}

/* STM32F4 common registers definitions */
static const struct stm32_adc_common_regs stm32f4_adc_common_regs = {
	.csr = STM32F4_ADC_CSR,
	.eoc1_msk = STM32F4_EOC1,
	.eoc2_msk = STM32F4_EOC2,
	.eoc3_msk = STM32F4_EOC3,
};

/* STM32H7 common registers definitions */
static const struct stm32_adc_common_regs stm32h7_adc_common_regs = {
	.csr = STM32H7_ADC_CSR,
	.eoc1_msk = STM32H7_EOC_MST,
	.eoc2_msk = STM32H7_EOC_SLV,
};

/* ADC common interrupt for all instances */
static void stm32_adc_irq_handler(struct irq_desc *desc)
{
	struct stm32_adc_priv *priv = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 status;

	chained_irq_enter(chip, desc);
	status = readl_relaxed(priv->common.base + priv->cfg->regs->csr);

	if (status & priv->cfg->regs->eoc1_msk)
		generic_handle_irq(irq_find_mapping(priv->domain, 0));

	if (status & priv->cfg->regs->eoc2_msk)
		generic_handle_irq(irq_find_mapping(priv->domain, 1));

	if (status & priv->cfg->regs->eoc3_msk)
		generic_handle_irq(irq_find_mapping(priv->domain, 2));

	chained_irq_exit(chip, desc);
};

static int stm32_adc_domain_map(struct irq_domain *d, unsigned int irq,
				irq_hw_number_t hwirq)
{
	irq_set_chip_data(irq, d->host_data);
	irq_set_chip_and_handler(irq, &dummy_irq_chip, handle_level_irq);

	return 0;
}

static void stm32_adc_domain_unmap(struct irq_domain *d, unsigned int irq)
{
	irq_set_chip_and_handler(irq, NULL, NULL);
	irq_set_chip_data(irq, NULL);
}

static const struct irq_domain_ops stm32_adc_domain_ops = {
	.map = stm32_adc_domain_map,
	.unmap  = stm32_adc_domain_unmap,
	.xlate = irq_domain_xlate_onecell,
};

static int stm32_adc_irq_probe(struct platform_device *pdev,
			       struct stm32_adc_priv *priv)
{
	struct device_node *np = pdev->dev.of_node;

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0) {
		dev_err(&pdev->dev, "failed to get irq\n");
		return priv->irq;
	}

	priv->domain = irq_domain_add_simple(np, STM32_ADC_MAX_ADCS, 0,
					     &stm32_adc_domain_ops,
					     priv);
	if (!priv->domain) {
		dev_err(&pdev->dev, "Failed to add irq domain\n");
		return -ENOMEM;
	}

	irq_set_chained_handler(priv->irq, stm32_adc_irq_handler);
	irq_set_handler_data(priv->irq, priv);

	return 0;
}

static void stm32_adc_irq_remove(struct platform_device *pdev,
				 struct stm32_adc_priv *priv)
{
	int hwirq;

	for (hwirq = 0; hwirq < STM32_ADC_MAX_ADCS; hwirq++)
		irq_dispose_mapping(irq_find_mapping(priv->domain, hwirq));
	irq_domain_remove(priv->domain);
	irq_set_chained_handler(priv->irq, NULL);
}

static int stm32_adc_probe(struct platform_device *pdev)
{
	struct stm32_adc_priv *priv;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	int ret;

	if (!pdev->dev.of_node)
		return -ENODEV;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->cfg = (const struct stm32_adc_priv_cfg *)
		of_match_device(dev->driver->of_match_table, dev)->data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->common.base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->common.base))
		return PTR_ERR(priv->common.base);
	priv->common.phys_base = res->start;

	priv->vref = devm_regulator_get(&pdev->dev, "vref");
	if (IS_ERR(priv->vref)) {
		ret = PTR_ERR(priv->vref);
		dev_err(&pdev->dev, "vref get failed, %d\n", ret);
		return ret;
	}

	ret = regulator_enable(priv->vref);
	if (ret < 0) {
		dev_err(&pdev->dev, "vref enable failed\n");
		return ret;
	}

	ret = regulator_get_voltage(priv->vref);
	if (ret < 0) {
		dev_err(&pdev->dev, "vref get voltage failed, %d\n", ret);
		goto err_regulator_disable;
	}
	priv->common.vref_mv = ret / 1000;
	dev_dbg(&pdev->dev, "vref+=%dmV\n", priv->common.vref_mv);

	priv->aclk = devm_clk_get(&pdev->dev, "adc");
	if (IS_ERR(priv->aclk)) {
		ret = PTR_ERR(priv->aclk);
		if (ret == -ENOENT) {
			priv->aclk = NULL;
		} else {
			dev_err(&pdev->dev, "Can't get 'adc' clock\n");
			goto err_regulator_disable;
		}
	}

	if (priv->aclk) {
		ret = clk_prepare_enable(priv->aclk);
		if (ret < 0) {
			dev_err(&pdev->dev, "adc clk enable failed\n");
			goto err_regulator_disable;
		}
	}

	priv->bclk = devm_clk_get(&pdev->dev, "bus");
	if (IS_ERR(priv->bclk)) {
		ret = PTR_ERR(priv->bclk);
		if (ret == -ENOENT) {
			priv->bclk = NULL;
		} else {
			dev_err(&pdev->dev, "Can't get 'bus' clock\n");
			goto err_aclk_disable;
		}
	}

	if (priv->bclk) {
		ret = clk_prepare_enable(priv->bclk);
		if (ret < 0) {
			dev_err(&pdev->dev, "adc clk enable failed\n");
			goto err_aclk_disable;
		}
	}

	ret = priv->cfg->clk_sel(pdev, priv);
	if (ret < 0)
		goto err_bclk_disable;

	ret = stm32_adc_irq_probe(pdev, priv);
	if (ret < 0)
		goto err_bclk_disable;

	platform_set_drvdata(pdev, &priv->common);

	ret = of_platform_populate(np, NULL, NULL, &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to populate DT children\n");
		goto err_irq_remove;
	}

	return 0;

err_irq_remove:
	stm32_adc_irq_remove(pdev, priv);

err_bclk_disable:
	if (priv->bclk)
		clk_disable_unprepare(priv->bclk);

err_aclk_disable:
	if (priv->aclk)
		clk_disable_unprepare(priv->aclk);

err_regulator_disable:
	regulator_disable(priv->vref);

	return ret;
}

static int stm32_adc_remove(struct platform_device *pdev)
{
	struct stm32_adc_common *common = platform_get_drvdata(pdev);
	struct stm32_adc_priv *priv = to_stm32_adc_priv(common);

	of_platform_depopulate(&pdev->dev);
	stm32_adc_irq_remove(pdev, priv);
	if (priv->bclk)
		clk_disable_unprepare(priv->bclk);
	if (priv->aclk)
		clk_disable_unprepare(priv->aclk);
	regulator_disable(priv->vref);

	return 0;
}

static const struct stm32_adc_priv_cfg stm32f4_adc_priv_cfg = {
	.regs = &stm32f4_adc_common_regs,
	.clk_sel = stm32f4_adc_clk_sel,
};

static const struct stm32_adc_priv_cfg stm32h7_adc_priv_cfg = {
	.regs = &stm32h7_adc_common_regs,
	.clk_sel = stm32h7_adc_clk_sel,
};

static const struct of_device_id stm32_adc_of_match[] = {
	{
		.compatible = "st,stm32f4-adc-core",
		.data = (void *)&stm32f4_adc_priv_cfg
	}, {
		.compatible = "st,stm32h7-adc-core",
		.data = (void *)&stm32h7_adc_priv_cfg
	}, {
	},
};
MODULE_DEVICE_TABLE(of, stm32_adc_of_match);

static struct platform_driver stm32_adc_driver = {
	.probe = stm32_adc_probe,
	.remove = stm32_adc_remove,
	.driver = {
		.name = "stm32-adc-core",
		.of_match_table = stm32_adc_of_match,
	},
};
module_platform_driver(stm32_adc_driver);

MODULE_AUTHOR("Fabrice Gasnier <fabrice.gasnier@st.com>");
MODULE_DESCRIPTION("STMicroelectronics STM32 ADC core driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:stm32-adc-core");
