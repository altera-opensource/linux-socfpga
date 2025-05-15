// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025, Altera Corporation
 */

#include <linux/of_platform.h>
#include <crypto/internal/rng.h>
#include <misc/socfpga_fcs_hal.h>
#include <linux/platform_device.h>
#include <linux/of.h>

static int socfpga_rng_generate(struct crypto_rng *tfm, const u8 *src,
				unsigned int slen, u8 *rng,
				unsigned int rng_len)
{
	struct fcs_cmd_context *k_ctx;
	int ret = 0;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		return -EFAULT;
	}

	k_ctx->rng.rng = rng;
	k_ctx->rng.rng_len = rng_len;
	ret = hal_random_number(k_ctx);
	if (ret)
		pr_err("Failed to generate random number\n");

	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static struct rng_alg socfpga_rng_alg = {
	.generate	= socfpga_rng_generate,
	.base		= {
		.cra_name		= "socfpga_rng",
		.cra_driver_name	= "socfpga_rng",
		.cra_flags		= CRYPTO_ALG_TYPE_RNG,
		.cra_priority		= 300,
		.cra_blocksize		= 4,
		.cra_ctxsize		= 0,
		.cra_module		= THIS_MODULE,
		.cra_init		= NULL,
	},
};

static int socfpga_crypto_register(struct device *dev)
{
	int ret;

	ret = crypto_register_rng(&socfpga_rng_alg);
	if (ret) {
		pr_err("socfpga_rng: registration failed\n");
		return ret;
	}

	pr_info("Crypto Random number Algorithms Registered Successfully\n");

	return ret;
}

static void socfpga_crypto_unregister(void)
{
	crypto_unregister_rng(&socfpga_rng_alg);
}

static int socfpga_crypto_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;

	if (!hal_fcs_is_ready()) {
		pr_err(" FCS HAL driver is not ready");
		return -EPROBE_DEFER;
	}

	/* Algorithms Registration */
	ret = socfpga_crypto_register(dev);
	if (ret) {
		pr_err("SOCFPGA Crypto - algrithms register failed.\n");
		return ret;
	}

	pr_info("%s is successfully completed", __func__);

	return 0;
}

static const struct of_device_id socfpga_crypto_of_match[] = {
	{ .compatible = "intel,agilex5-soc-fcs-crypto" },
	{ .compatible = "intel,agilex-soc-fcs-crypto" },
	{},
};

static struct platform_driver socfpga_crypto_driver = {
	.probe = socfpga_crypto_probe,
	.driver = {
		.name = "socfpga-crypto",
		.of_match_table = of_match_ptr(socfpga_crypto_of_match),
	},
};
MODULE_DEVICE_TABLE(of, socfpga_crypto_of_match);

static int __init socfpga_crypto_init(void)
{
	struct device_node *svc_np;
	struct device_node *np;
	int ret;

	svc_np = of_find_node_by_name(NULL, "firmware");
	if (!svc_np)
		return -ENODEV;

	of_node_get(svc_np);
	np = of_find_matching_node(svc_np, socfpga_crypto_of_match);
	if (!np) {
		of_node_put(svc_np);
		return -ENODEV;
	}

	ret = of_platform_populate(svc_np, socfpga_crypto_of_match, NULL, NULL);
	of_node_put(svc_np);
	if (ret)
		return ret;

	return platform_driver_register(&socfpga_crypto_driver);
}

/*
 * This function is called when the module is unloaded.
 */
static void __exit socfpga_crypto_exit(void)
{
	socfpga_crypto_unregister();

	return platform_driver_unregister(&socfpga_crypto_driver);
}

module_init(socfpga_crypto_init);
module_exit(socfpga_crypto_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Altera socfpga crypto driver");
MODULE_AUTHOR("Santosh Male, Sagar Khadgi, Balsundar Ponnusamy");
