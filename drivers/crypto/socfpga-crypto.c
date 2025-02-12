// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025, Altera Corporation
 */

#include <linux/of_platform.h>
#include <linux/crypto.h>
#include <crypto/algapi.h>
#include <crypto/aes.h>
#include <crypto/gcm.h>
#include <crypto/sha2.h>
#include <crypto/hash.h>
#include <crypto/internal/aead.h>
#include <crypto/internal/skcipher.h>
#include <crypto/internal/hash.h>
#include <crypto/internal/rng.h>
#include <crypto/rng.h>
#include <misc/socfpga_fcs_hal.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#define SHA256_DIGEST_SIZE_ID 0
#define SHA384_DIGEST_SIZE_ID 1
#define SHA512_DIGEST_SIZE_ID 2

struct socfpga_sha_ctx {
	char *intermediate_data;
	unsigned int intermediate_len;
	unsigned int sha_init_completed;
	unsigned int is_hmac;
	unsigned int request_size;
	unsigned int send_size;
};

struct socfpga_sha_reqctx {
	struct scatterlist *sg;
};

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

static int socfpga_sha_init(struct ahash_request *req)
{
	struct fcs_cmd_context *k_ctx;
	int ret = 0;
	u32 digest_len = 0;
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct socfpga_sha_ctx *ctx = crypto_ahash_ctx(tfm);

	if (ctx->sha_init_completed == 1)
		return 0;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		return -EFAULT;
	}

	ctx->request_size = k_ctx->dgst.src_len;
	ctx->send_size = 0;

	switch (crypto_ahash_digestsize(tfm)) {
	case SHA256_DIGEST_SIZE:
		k_ctx->dgst.sha_digest_sz = SHA256_DIGEST_SIZE_ID;
		break;
	case SHA384_DIGEST_SIZE:
		k_ctx->dgst.sha_digest_sz = SHA384_DIGEST_SIZE_ID;
		break;
	case SHA512_DIGEST_SIZE:
		k_ctx->dgst.sha_digest_sz = SHA512_DIGEST_SIZE_ID;
		break;
	default:
		pr_err("Invalid digest size: %u\n",
		       crypto_ahash_digestsize(tfm));
		hal_release_fcs_cmd_ctx(k_ctx);
		return -EINVAL;
	}

	k_ctx->dgst.src = (char *)req->src;
	k_ctx->dgst.src_len = 0;
	k_ctx->dgst.digest = req->result;
	if (ctx->is_hmac)
		k_ctx->dgst.sha_op_mode = 2;
	else
		k_ctx->dgst.sha_op_mode = 1;

	k_ctx->dgst.stage = FCS_DIGEST_STAGE_INIT;
	digest_len = k_ctx->dgst.sha_digest_sz;
	k_ctx->dgst.digest_len = &digest_len;

	ret = hal_digest(k_ctx);
	if (ret)
		pr_err("Failed to perform digest init operation\n");

	hal_release_fcs_cmd_ctx(k_ctx);

	ctx->sha_init_completed = 1;
	return ret;
}

/*
 * Define the algorithm's update function
 */
static int socfpga_sha_update(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct socfpga_sha_ctx *ctx = crypto_ahash_ctx(tfm);
	struct fcs_cmd_context *k_ctx;
	int ret = 0;
	unsigned int remaining_bytes;
	static u8 pending_intermediate_processing;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		return -EFAULT;
	}

	remaining_bytes = ctx->request_size - ctx->send_size;
	ctx->intermediate_len = req->nbytes;
	ctx->intermediate_data = k_ctx->dgst.src + ctx->send_size;

	if ((remaining_bytes - req->nbytes) > 8) {
		if (ctx->send_size + req->nbytes >= SZ_4M) {
			k_ctx->dgst.stage = FCS_DIGEST_STAGE_UPDATE;
			ret = hal_digest(k_ctx);
			if (ret)
				pr_err("Failed to perform digest update operation\n");
			pending_intermediate_processing = 0;
			ctx->request_size -= ctx->send_size;
			ctx->send_size = 0;
			k_ctx->dgst.src_len = 0;
			ctx->intermediate_data = k_ctx->dgst.src;
		}

		if (sg_copy_to_buffer(req->src, sg_nents(req->src),
				      ctx->intermediate_data,
				      req->nbytes) != req->nbytes) {
			pr_err("Failed to copy data from scatterlist\n");
			hal_release_fcs_cmd_ctx(k_ctx);
			return -EFAULT;
		}

		k_ctx->dgst.src_len += req->nbytes;
		ctx->send_size += req->nbytes;
		pending_intermediate_processing = 1;

	} else {
		if (ctx->send_size + req->nbytes >= SZ_4M) {
			k_ctx->dgst.stage = FCS_DIGEST_STAGE_UPDATE;
			k_ctx->dgst.src_len = ctx->send_size;
			ret = hal_digest(k_ctx);
			if (ret)
				pr_err("Failed to perform digest update operation\n");
			pending_intermediate_processing = 0;
			ctx->request_size -= ctx->send_size;
			ctx->send_size = 0;
			k_ctx->dgst.src_len = 0;
			ctx->intermediate_data = k_ctx->dgst.src;
		}

		if (sg_copy_to_buffer(req->src, sg_nents(req->src),
				      ctx->intermediate_data,
				      req->nbytes) != req->nbytes) {
			pr_err("Failed to copy data from scatterlist\n");
			hal_release_fcs_cmd_ctx(k_ctx);
			return -EFAULT;
		}

		ctx->send_size += req->nbytes;
		k_ctx->dgst.src_len += req->nbytes;
		if (pending_intermediate_processing == 1)
			ctx->intermediate_data = k_ctx->dgst.src;
	}
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static int socfpga_sha_final(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct socfpga_sha_ctx *ctx = crypto_ahash_ctx(tfm);
	struct fcs_cmd_context *k_ctx;
	int ret;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		return -EFAULT;
	}

	k_ctx->dgst.src = ctx->intermediate_data;
	k_ctx->dgst.digest = req->result;
	k_ctx->dgst.stage = FCS_DIGEST_STAGE_FINAL;

	ret = hal_digest(k_ctx);
	if (ret)
		pr_err("Failed to perform digest final operation\n");

	hal_destroy_fcs_cmd_ctx(k_ctx);
	hal_release_fcs_cmd_ctx(k_ctx);
	ctx->sha_init_completed = 0;

	return ret;
}

static int socfpga_hmac_init_tfm(struct crypto_ahash *hash)
{
	struct socfpga_sha_ctx *ctx = crypto_ahash_ctx(hash);

	ctx->is_hmac = 1;

	return 0;
}

static int socfpga_sha_init_tfm(struct crypto_ahash *hash)
{
	struct socfpga_sha_ctx *ctx = crypto_ahash_ctx(hash);

	ctx->is_hmac = 0;

	return 0;
}

/* Define the ahash algorithm structures for HMAC-SHA-256, HMAC-SHA-384,
 * and HMAC-SHA-512
 */
static struct ahash_alg socfpga_hmac_algs[] = {
	{
		.init       = socfpga_sha_init,
		.update     = socfpga_sha_update,
		.final      = socfpga_sha_final,
		.init_tfm   = socfpga_hmac_init_tfm,
		.halg = {
			.base = {
				.cra_name        = "socfpga-hmac-sha256",
				.cra_driver_name = "socfpga-hmac-sha256",
				.cra_priority    = 300,
				.cra_flags       = CRYPTO_ALG_ASYNC,
				.cra_blocksize   = SHA256_BLOCK_SIZE,
				.cra_ctxsize     = sizeof(struct socfpga_sha_ctx),
				.cra_module      = THIS_MODULE,
			},
			.digestsize    = SHA256_DIGEST_SIZE,
			.statesize     = sizeof(struct socfpga_sha_reqctx),
		}
	},
	{
		.init       = socfpga_sha_init,
		.update     = socfpga_sha_update,
		.final      = socfpga_sha_final,
		.init_tfm   = socfpga_hmac_init_tfm,
		.halg = {
			.base = {
				.cra_name        = "socfpga-hmac-sha384",
				.cra_driver_name = "socfpga-hmac-sha384",
				.cra_priority    = 300,
				.cra_flags       = CRYPTO_ALG_ASYNC,
				.cra_blocksize   = SHA384_BLOCK_SIZE,
				.cra_ctxsize     = sizeof(struct socfpga_sha_ctx),
				.cra_module      = THIS_MODULE,
			},
			.digestsize    = SHA384_DIGEST_SIZE,
			.statesize     = sizeof(struct socfpga_sha_reqctx),
		}
	},
	{
		.init       = socfpga_sha_init,
		.update     = socfpga_sha_update,
		.final      = socfpga_sha_final,
		.init_tfm   = socfpga_hmac_init_tfm,
		.halg = {
			.base = {
				.cra_name        = "socfpga-hmac-sha512",
				.cra_driver_name = "socfpga-hmac-sha512",
				.cra_priority    = 300,
				.cra_flags       = CRYPTO_ALG_ASYNC,
				.cra_blocksize   = SHA512_BLOCK_SIZE,
				.cra_ctxsize     = sizeof(struct socfpga_sha_ctx),
				.cra_module      = THIS_MODULE,
			},
			.digestsize    = SHA512_DIGEST_SIZE,
			.statesize     = sizeof(struct socfpga_sha_reqctx),
		}
	}
};

/* Define the ahash algorithm structures for SHA-256, SHA-384, and SHA-512 */
static struct ahash_alg socfpga_sha_algs[] = {
	{
		.init       = socfpga_sha_init,
		.update     = socfpga_sha_update,
		.final      = socfpga_sha_final,
		.init_tfm   = socfpga_sha_init_tfm,
		.halg = {
			.base = {
				.cra_name        = "socfpga-sha256",
				.cra_driver_name = "socfpga-sha256",
				.cra_priority    = 300,
				.cra_flags       = CRYPTO_ALG_ASYNC,
				.cra_blocksize   = SHA256_BLOCK_SIZE,
				.cra_ctxsize     = sizeof(struct socfpga_sha_ctx),
				.cra_module      = THIS_MODULE,
			},
			.digestsize    = SHA256_DIGEST_SIZE,
			.statesize     = sizeof(struct socfpga_sha_reqctx),
		}
	},
	{
		.init       = socfpga_sha_init,
		.update     = socfpga_sha_update,
		.final      = socfpga_sha_final,
		.init_tfm   = socfpga_sha_init_tfm,
		.halg = {
			.base = {
				.cra_name        = "socfpga-sha384",
				.cra_driver_name = "socfpga-sha384",
				.cra_priority    = 300,
				.cra_flags       = CRYPTO_ALG_ASYNC,
				.cra_blocksize   = SHA384_BLOCK_SIZE,
				.cra_ctxsize     = sizeof(struct socfpga_sha_ctx),
				.cra_module      = THIS_MODULE,
			},
			.digestsize    = SHA384_DIGEST_SIZE,
			.statesize     = sizeof(struct socfpga_sha_reqctx),
		}
	},
	{
		.init       = socfpga_sha_init,
		.update     = socfpga_sha_update,
		.final      = socfpga_sha_final,
		.init_tfm   = socfpga_sha_init_tfm,
		.halg = {
			.base = {
				.cra_name        = "socfpga-sha512",
				.cra_driver_name = "socfpga-sha512",
				.cra_priority    = 300,
				.cra_flags       = CRYPTO_ALG_ASYNC,
				.cra_blocksize   = SHA512_BLOCK_SIZE,
				.cra_ctxsize     = sizeof(struct socfpga_sha_ctx),
				.cra_module      = THIS_MODULE,
			},
			.digestsize    = SHA512_DIGEST_SIZE,
			.statesize     = sizeof(struct socfpga_sha_reqctx),
		}
	},
};

static int socfpga_crypto_register(struct device *dev)
{
	int ret;
	int i;

	ret = crypto_register_rng(&socfpga_rng_alg);
	if (ret) {
		pr_err("socfpga_rng: registration failed\n");
		return ret;
	}

	pr_info("Crypto Random number Algorithms Registered Successfully\n");

	/* Register the SHA-256, SHA-384, and SHA-512 algorithms */
	for (i = 0; i < ARRAY_SIZE(socfpga_sha_algs); i++) {
		ret = crypto_register_ahash(&socfpga_sha_algs[i]);
		if (ret) {
			pr_err("Failed to register socfpga %s algorithm: %d\n",
			       socfpga_sha_algs[i].halg.base.cra_name, ret);
			goto unregister_md_algs;
		}
	}

	/* Register the HMAC-SHA-256, HMAC-SHA-384, and HMAC-SHA-512 algorithms */
	for (i = 0; i < ARRAY_SIZE(socfpga_hmac_algs); i++) {
		ret = crypto_register_ahash(&socfpga_hmac_algs[i]);
		if (ret) {
			pr_err("Failed to register socfpga %s algorithm: %d\n",
			       socfpga_hmac_algs[i].halg.base.cra_name, ret);
			goto unregister_hmac_algs;
		}
	}

	pr_info("SHA and HMAC algorithms registered successfully\n");

	return 0;

unregister_hmac_algs:
	while (i--)
		crypto_unregister_ahash(&socfpga_hmac_algs[i]);
	i = ARRAY_SIZE(socfpga_sha_algs);
unregister_md_algs:
	while (i--)
		crypto_unregister_ahash(&socfpga_sha_algs[i]);
	return ret;
}

static void socfpga_crypto_unregister(void)
{
	int i;
	/*
	 * Unregister the RNG algorithm
	 */
	crypto_unregister_rng(&socfpga_rng_alg);

	/*
	 * Unregister the SHA-256, SHA-384, and SHA-512 algorithms
	 */
	for (i = 0; i < ARRAY_SIZE(socfpga_sha_algs); i++)
		crypto_unregister_ahash(&socfpga_sha_algs[i]);

	/*
	 * Unregister the HMAC-SHA-256, HMAC-SHA-384, and HMAC-SHA-512 algorithms
	 */
	for (i = 0; i < ARRAY_SIZE(socfpga_hmac_algs); i++)
		crypto_unregister_ahash(&socfpga_hmac_algs[i]);

	pr_info("SoC FPGA SHA and HMAC algorithms unregistered successfully\n");
}

static int socfpga_crypto_probe(struct platform_device *pdev)
{
	int ret;
	/* device node pointer */
	struct device_node *np = pdev->dev.of_node;
	struct device_node *fcs_hal_np;
	struct device *dev = &pdev->dev;

	fcs_hal_np = of_parse_phandle(np, "dependent-on", 0);
	if (!fcs_hal_np) {
		pr_err("Failed to find HAL Driver device\n");
		return -ENODEV;
	}

	if (of_device_is_compatible(fcs_hal_np, "intel,agilex5-soc-fcs-hal") &&
	    !of_device_is_available(fcs_hal_np)) {
		pr_err("FCS HAL is not available.\n");
		of_node_put(fcs_hal_np);
		return -ENODEV;
	}

	/* Algorithms Registration */
	ret = socfpga_crypto_register(dev);
	if (ret) {
		pr_err("SOCFPGA Crypto - algrithms register failed.\n");
		return ret;
	}

	return 0;
}

static const struct of_device_id socfpga_crypto_of_match[] = {
	{ .compatible = "intel,agilex5-soc-fcs-crypto" },
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

	svc_np = of_find_node_by_name(NULL, "svc");
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
