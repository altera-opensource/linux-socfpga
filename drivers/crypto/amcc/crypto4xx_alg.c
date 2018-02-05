/**
 * AMCC SoC PPC4xx Crypto Driver
 *
 * Copyright (c) 2008 Applied Micro Circuits Corporation.
 * All rights reserved. James Hsiao <jhsiao@amcc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This file implements the Linux crypto algorithms.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/spinlock_types.h>
#include <linux/scatterlist.h>
#include <linux/crypto.h>
#include <linux/hash.h>
#include <crypto/internal/hash.h>
#include <linux/dma-mapping.h>
#include <crypto/algapi.h>
#include <crypto/aead.h>
#include <crypto/aes.h>
#include <crypto/gcm.h>
#include <crypto/sha.h>
#include <crypto/ctr.h>
#include "crypto4xx_reg_def.h"
#include "crypto4xx_core.h"
#include "crypto4xx_sa.h"

static void set_dynamic_sa_command_0(struct dynamic_sa_ctl *sa, u32 save_h,
				     u32 save_iv, u32 ld_h, u32 ld_iv,
				     u32 hdr_proc, u32 h, u32 c, u32 pad_type,
				     u32 op_grp, u32 op, u32 dir)
{
	sa->sa_command_0.w = 0;
	sa->sa_command_0.bf.save_hash_state = save_h;
	sa->sa_command_0.bf.save_iv = save_iv;
	sa->sa_command_0.bf.load_hash_state = ld_h;
	sa->sa_command_0.bf.load_iv = ld_iv;
	sa->sa_command_0.bf.hdr_proc = hdr_proc;
	sa->sa_command_0.bf.hash_alg = h;
	sa->sa_command_0.bf.cipher_alg = c;
	sa->sa_command_0.bf.pad_type = pad_type & 3;
	sa->sa_command_0.bf.extend_pad = pad_type >> 2;
	sa->sa_command_0.bf.op_group = op_grp;
	sa->sa_command_0.bf.opcode = op;
	sa->sa_command_0.bf.dir = dir;
}

static void set_dynamic_sa_command_1(struct dynamic_sa_ctl *sa, u32 cm,
				     u32 hmac_mc, u32 cfb, u32 esn,
				     u32 sn_mask, u32 mute, u32 cp_pad,
				     u32 cp_pay, u32 cp_hdr)
{
	sa->sa_command_1.w = 0;
	sa->sa_command_1.bf.crypto_mode31 = (cm & 4) >> 2;
	sa->sa_command_1.bf.crypto_mode9_8 = cm & 3;
	sa->sa_command_1.bf.feedback_mode = cfb,
	sa->sa_command_1.bf.sa_rev = 1;
	sa->sa_command_1.bf.hmac_muting = hmac_mc;
	sa->sa_command_1.bf.extended_seq_num = esn;
	sa->sa_command_1.bf.seq_num_mask = sn_mask;
	sa->sa_command_1.bf.mutable_bit_proc = mute;
	sa->sa_command_1.bf.copy_pad = cp_pad;
	sa->sa_command_1.bf.copy_payload = cp_pay;
	sa->sa_command_1.bf.copy_hdr = cp_hdr;
}

int crypto4xx_encrypt(struct ablkcipher_request *req)
{
	struct crypto4xx_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	unsigned int ivlen = crypto_ablkcipher_ivsize(
		crypto_ablkcipher_reqtfm(req));
	__le32 iv[ivlen];

	if (ivlen)
		crypto4xx_memcpy_to_le32(iv, req->info, ivlen);

	return crypto4xx_build_pd(&req->base, ctx, req->src, req->dst,
		req->nbytes, iv, ivlen, ctx->sa_out, ctx->sa_len, 0);
}

int crypto4xx_decrypt(struct ablkcipher_request *req)
{
	struct crypto4xx_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	unsigned int ivlen = crypto_ablkcipher_ivsize(
		crypto_ablkcipher_reqtfm(req));
	__le32 iv[ivlen];

	if (ivlen)
		crypto4xx_memcpy_to_le32(iv, req->info, ivlen);

	return crypto4xx_build_pd(&req->base, ctx, req->src, req->dst,
		req->nbytes, iv, ivlen, ctx->sa_in, ctx->sa_len, 0);
}

/**
 * AES Functions
 */
static int crypto4xx_setkey_aes(struct crypto_ablkcipher *cipher,
				const u8 *key,
				unsigned int keylen,
				unsigned char cm,
				u8 fb)
{
	struct crypto_tfm *tfm = crypto_ablkcipher_tfm(cipher);
	struct crypto4xx_ctx *ctx = crypto_tfm_ctx(tfm);
	struct dynamic_sa_ctl *sa;
	int    rc;

	if (keylen != AES_KEYSIZE_256 &&
		keylen != AES_KEYSIZE_192 && keylen != AES_KEYSIZE_128) {
		crypto_ablkcipher_set_flags(cipher,
				CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}

	/* Create SA */
	if (ctx->sa_in || ctx->sa_out)
		crypto4xx_free_sa(ctx);

	rc = crypto4xx_alloc_sa(ctx, SA_AES128_LEN + (keylen-16) / 4);
	if (rc)
		return rc;

	/* Setup SA */
	sa = ctx->sa_in;

	set_dynamic_sa_command_0(sa, SA_NOT_SAVE_HASH, SA_NOT_SAVE_IV,
				 SA_LOAD_HASH_FROM_SA, SA_LOAD_IV_FROM_STATE,
				 SA_NO_HEADER_PROC, SA_HASH_ALG_NULL,
				 SA_CIPHER_ALG_AES, SA_PAD_TYPE_ZERO,
				 SA_OP_GROUP_BASIC, SA_OPCODE_DECRYPT,
				 DIR_INBOUND);

	set_dynamic_sa_command_1(sa, cm, SA_HASH_MODE_HASH,
				 fb, SA_EXTENDED_SN_OFF,
				 SA_SEQ_MASK_OFF, SA_MC_ENABLE,
				 SA_NOT_COPY_PAD, SA_NOT_COPY_PAYLOAD,
				 SA_NOT_COPY_HDR);
	crypto4xx_memcpy_to_le32(get_dynamic_sa_key_field(sa),
				 key, keylen);
	sa->sa_contents.w = SA_AES_CONTENTS | (keylen << 2);
	sa->sa_command_1.bf.key_len = keylen >> 3;

	memcpy(ctx->sa_out, ctx->sa_in, ctx->sa_len * 4);
	sa = ctx->sa_out;
	sa->sa_command_0.bf.dir = DIR_OUTBOUND;

	return 0;
}

int crypto4xx_setkey_aes_cbc(struct crypto_ablkcipher *cipher,
			     const u8 *key, unsigned int keylen)
{
	return crypto4xx_setkey_aes(cipher, key, keylen, CRYPTO_MODE_CBC,
				    CRYPTO_FEEDBACK_MODE_NO_FB);
}

int crypto4xx_setkey_aes_cfb(struct crypto_ablkcipher *cipher,
			     const u8 *key, unsigned int keylen)
{
	return crypto4xx_setkey_aes(cipher, key, keylen, CRYPTO_MODE_CFB,
				    CRYPTO_FEEDBACK_MODE_128BIT_CFB);
}

int crypto4xx_setkey_aes_ecb(struct crypto_ablkcipher *cipher,
			     const u8 *key, unsigned int keylen)
{
	return crypto4xx_setkey_aes(cipher, key, keylen, CRYPTO_MODE_ECB,
				    CRYPTO_FEEDBACK_MODE_NO_FB);
}

int crypto4xx_setkey_aes_ofb(struct crypto_ablkcipher *cipher,
			     const u8 *key, unsigned int keylen)
{
	return crypto4xx_setkey_aes(cipher, key, keylen, CRYPTO_MODE_OFB,
				    CRYPTO_FEEDBACK_MODE_64BIT_OFB);
}

int crypto4xx_setkey_rfc3686(struct crypto_ablkcipher *cipher,
			     const u8 *key, unsigned int keylen)
{
	struct crypto_tfm *tfm = crypto_ablkcipher_tfm(cipher);
	struct crypto4xx_ctx *ctx = crypto_tfm_ctx(tfm);
	int rc;

	rc = crypto4xx_setkey_aes(cipher, key, keylen - CTR_RFC3686_NONCE_SIZE,
		CRYPTO_MODE_CTR, CRYPTO_FEEDBACK_MODE_NO_FB);
	if (rc)
		return rc;

	ctx->iv_nonce = cpu_to_le32p((u32 *)&key[keylen -
						 CTR_RFC3686_NONCE_SIZE]);

	return 0;
}

int crypto4xx_rfc3686_encrypt(struct ablkcipher_request *req)
{
	struct crypto4xx_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	__le32 iv[AES_IV_SIZE / 4] = {
		ctx->iv_nonce,
		cpu_to_le32p((u32 *) req->info),
		cpu_to_le32p((u32 *) (req->info + 4)),
		cpu_to_le32(1) };

	return crypto4xx_build_pd(&req->base, ctx, req->src, req->dst,
				  req->nbytes, iv, AES_IV_SIZE,
				  ctx->sa_out, ctx->sa_len, 0);
}

int crypto4xx_rfc3686_decrypt(struct ablkcipher_request *req)
{
	struct crypto4xx_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	__le32 iv[AES_IV_SIZE / 4] = {
		ctx->iv_nonce,
		cpu_to_le32p((u32 *) req->info),
		cpu_to_le32p((u32 *) (req->info + 4)),
		cpu_to_le32(1) };

	return crypto4xx_build_pd(&req->base, ctx, req->src, req->dst,
				  req->nbytes, iv, AES_IV_SIZE,
				  ctx->sa_out, ctx->sa_len, 0);
}

static inline bool crypto4xx_aead_need_fallback(struct aead_request *req,
						bool is_ccm, bool decrypt)
{
	struct crypto_aead *aead = crypto_aead_reqtfm(req);

	/* authsize has to be a multiple of 4 */
	if (aead->authsize & 3)
		return true;

	/*
	 * hardware does not handle cases where cryptlen
	 * is less than a block
	 */
	if (req->cryptlen < AES_BLOCK_SIZE)
		return true;

	/* assoc len needs to be a multiple of 4 */
	if (req->assoclen & 0x3)
		return true;

	/* CCM supports only counter field length of 2 and 4 bytes */
	if (is_ccm && !(req->iv[0] == 1 || req->iv[0] == 3))
		return true;

	return false;
}

static int crypto4xx_aead_fallback(struct aead_request *req,
	struct crypto4xx_ctx *ctx, bool do_decrypt)
{
	char aead_req_data[sizeof(struct aead_request) +
			   crypto_aead_reqsize(ctx->sw_cipher.aead)]
		__aligned(__alignof__(struct aead_request));

	struct aead_request *subreq = (void *) aead_req_data;

	memset(subreq, 0, sizeof(aead_req_data));

	aead_request_set_tfm(subreq, ctx->sw_cipher.aead);
	aead_request_set_callback(subreq, req->base.flags,
				  req->base.complete, req->base.data);
	aead_request_set_crypt(subreq, req->src, req->dst, req->cryptlen,
			       req->iv);
	aead_request_set_ad(subreq, req->assoclen);
	return do_decrypt ? crypto_aead_decrypt(subreq) :
			    crypto_aead_encrypt(subreq);
}

static int crypto4xx_setup_fallback(struct crypto4xx_ctx *ctx,
				    struct crypto_aead *cipher,
				    const u8 *key,
				    unsigned int keylen)
{
	int rc;

	crypto_aead_clear_flags(ctx->sw_cipher.aead, CRYPTO_TFM_REQ_MASK);
	crypto_aead_set_flags(ctx->sw_cipher.aead,
		crypto_aead_get_flags(cipher) & CRYPTO_TFM_REQ_MASK);
	rc = crypto_aead_setkey(ctx->sw_cipher.aead, key, keylen);
	crypto_aead_clear_flags(cipher, CRYPTO_TFM_RES_MASK);
	crypto_aead_set_flags(cipher,
		crypto_aead_get_flags(ctx->sw_cipher.aead) &
			CRYPTO_TFM_RES_MASK);

	return rc;
}

/**
 * AES-CCM Functions
 */

int crypto4xx_setkey_aes_ccm(struct crypto_aead *cipher, const u8 *key,
			     unsigned int keylen)
{
	struct crypto_tfm *tfm = crypto_aead_tfm(cipher);
	struct crypto4xx_ctx *ctx = crypto_tfm_ctx(tfm);
	struct dynamic_sa_ctl *sa;
	int rc = 0;

	rc = crypto4xx_setup_fallback(ctx, cipher, key, keylen);
	if (rc)
		return rc;

	if (ctx->sa_in || ctx->sa_out)
		crypto4xx_free_sa(ctx);

	rc = crypto4xx_alloc_sa(ctx, SA_AES128_CCM_LEN + (keylen - 16) / 4);
	if (rc)
		return rc;

	/* Setup SA */
	sa = (struct dynamic_sa_ctl *) ctx->sa_in;
	sa->sa_contents.w = SA_AES_CCM_CONTENTS | (keylen << 2);

	set_dynamic_sa_command_0(sa, SA_SAVE_HASH, SA_NOT_SAVE_IV,
				 SA_LOAD_HASH_FROM_SA, SA_LOAD_IV_FROM_STATE,
				 SA_NO_HEADER_PROC, SA_HASH_ALG_CBC_MAC,
				 SA_CIPHER_ALG_AES,
				 SA_PAD_TYPE_ZERO, SA_OP_GROUP_BASIC,
				 SA_OPCODE_HASH_DECRYPT, DIR_INBOUND);

	set_dynamic_sa_command_1(sa, CRYPTO_MODE_CTR, SA_HASH_MODE_HASH,
				 CRYPTO_FEEDBACK_MODE_NO_FB, SA_EXTENDED_SN_OFF,
				 SA_SEQ_MASK_OFF, SA_MC_ENABLE,
				 SA_NOT_COPY_PAD, SA_COPY_PAYLOAD,
				 SA_NOT_COPY_HDR);

	sa->sa_command_1.bf.key_len = keylen >> 3;

	crypto4xx_memcpy_to_le32(get_dynamic_sa_key_field(sa), key, keylen);

	memcpy(ctx->sa_out, ctx->sa_in, ctx->sa_len * 4);
	sa = (struct dynamic_sa_ctl *) ctx->sa_out;

	set_dynamic_sa_command_0(sa, SA_SAVE_HASH, SA_NOT_SAVE_IV,
				 SA_LOAD_HASH_FROM_SA, SA_LOAD_IV_FROM_STATE,
				 SA_NO_HEADER_PROC, SA_HASH_ALG_CBC_MAC,
				 SA_CIPHER_ALG_AES,
				 SA_PAD_TYPE_ZERO, SA_OP_GROUP_BASIC,
				 SA_OPCODE_ENCRYPT_HASH, DIR_OUTBOUND);

	set_dynamic_sa_command_1(sa, CRYPTO_MODE_CTR, SA_HASH_MODE_HASH,
				 CRYPTO_FEEDBACK_MODE_NO_FB, SA_EXTENDED_SN_OFF,
				 SA_SEQ_MASK_OFF, SA_MC_ENABLE,
				 SA_COPY_PAD, SA_COPY_PAYLOAD,
				 SA_NOT_COPY_HDR);

	sa->sa_command_1.bf.key_len = keylen >> 3;
	return 0;
}

static int crypto4xx_crypt_aes_ccm(struct aead_request *req, bool decrypt)
{
	struct crypto4xx_ctx *ctx  = crypto_tfm_ctx(req->base.tfm);
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	unsigned int len = req->cryptlen;
	__le32 iv[16];
	u32 tmp_sa[ctx->sa_len * 4];
	struct dynamic_sa_ctl *sa = (struct dynamic_sa_ctl *)tmp_sa;

	if (crypto4xx_aead_need_fallback(req, true, decrypt))
		return crypto4xx_aead_fallback(req, ctx, decrypt);

	if (decrypt)
		len -= crypto_aead_authsize(aead);

	memcpy(tmp_sa, decrypt ? ctx->sa_in : ctx->sa_out, sizeof(tmp_sa));
	sa->sa_command_0.bf.digest_len = crypto_aead_authsize(aead) >> 2;

	if (req->iv[0] == 1) {
		/* CRYPTO_MODE_AES_ICM */
		sa->sa_command_1.bf.crypto_mode9_8 = 1;
	}

	iv[3] = cpu_to_le32(0);
	crypto4xx_memcpy_to_le32(iv, req->iv, 16 - (req->iv[0] + 1));

	return crypto4xx_build_pd(&req->base, ctx, req->src, req->dst,
				  len, iv, sizeof(iv),
				  sa, ctx->sa_len, req->assoclen);
}

int crypto4xx_encrypt_aes_ccm(struct aead_request *req)
{
	return crypto4xx_crypt_aes_ccm(req, false);
}

int crypto4xx_decrypt_aes_ccm(struct aead_request *req)
{
	return crypto4xx_crypt_aes_ccm(req, true);
}

int crypto4xx_setauthsize_aead(struct crypto_aead *cipher,
			       unsigned int authsize)
{
	struct crypto_tfm *tfm = crypto_aead_tfm(cipher);
	struct crypto4xx_ctx *ctx = crypto_tfm_ctx(tfm);

	return crypto_aead_setauthsize(ctx->sw_cipher.aead, authsize);
}

/**
 * AES-GCM Functions
 */

static int crypto4xx_aes_gcm_validate_keylen(unsigned int keylen)
{
	switch (keylen) {
	case 16:
	case 24:
	case 32:
		return 0;
	default:
		return -EINVAL;
	}
}

static int crypto4xx_compute_gcm_hash_key_sw(__le32 *hash_start, const u8 *key,
					     unsigned int keylen)
{
	struct crypto_cipher *aes_tfm = NULL;
	uint8_t src[16] = { 0 };
	int rc = 0;

	aes_tfm = crypto_alloc_cipher("aes", 0, CRYPTO_ALG_ASYNC |
				      CRYPTO_ALG_NEED_FALLBACK);
	if (IS_ERR(aes_tfm)) {
		rc = PTR_ERR(aes_tfm);
		pr_warn("could not load aes cipher driver: %d\n", rc);
		return rc;
	}

	rc = crypto_cipher_setkey(aes_tfm, key, keylen);
	if (rc) {
		pr_err("setkey() failed: %d\n", rc);
		goto out;
	}

	crypto_cipher_encrypt_one(aes_tfm, src, src);
	crypto4xx_memcpy_to_le32(hash_start, src, 16);
out:
	crypto_free_cipher(aes_tfm);
	return rc;
}

int crypto4xx_setkey_aes_gcm(struct crypto_aead *cipher,
			     const u8 *key, unsigned int keylen)
{
	struct crypto_tfm *tfm = crypto_aead_tfm(cipher);
	struct crypto4xx_ctx *ctx = crypto_tfm_ctx(tfm);
	struct dynamic_sa_ctl *sa;
	int    rc = 0;

	if (crypto4xx_aes_gcm_validate_keylen(keylen) != 0) {
		crypto_aead_set_flags(cipher, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}

	rc = crypto4xx_setup_fallback(ctx, cipher, key, keylen);
	if (rc)
		return rc;

	if (ctx->sa_in || ctx->sa_out)
		crypto4xx_free_sa(ctx);

	rc = crypto4xx_alloc_sa(ctx, SA_AES128_GCM_LEN + (keylen - 16) / 4);
	if (rc)
		return rc;

	sa  = (struct dynamic_sa_ctl *) ctx->sa_in;

	sa->sa_contents.w = SA_AES_GCM_CONTENTS | (keylen << 2);
	set_dynamic_sa_command_0(sa, SA_SAVE_HASH, SA_NOT_SAVE_IV,
				 SA_LOAD_HASH_FROM_SA, SA_LOAD_IV_FROM_STATE,
				 SA_NO_HEADER_PROC, SA_HASH_ALG_GHASH,
				 SA_CIPHER_ALG_AES, SA_PAD_TYPE_ZERO,
				 SA_OP_GROUP_BASIC, SA_OPCODE_HASH_DECRYPT,
				 DIR_INBOUND);
	set_dynamic_sa_command_1(sa, CRYPTO_MODE_CTR, SA_HASH_MODE_HASH,
				 CRYPTO_FEEDBACK_MODE_NO_FB, SA_EXTENDED_SN_OFF,
				 SA_SEQ_MASK_ON, SA_MC_DISABLE,
				 SA_NOT_COPY_PAD, SA_COPY_PAYLOAD,
				 SA_NOT_COPY_HDR);

	sa->sa_command_1.bf.key_len = keylen >> 3;

	crypto4xx_memcpy_to_le32(get_dynamic_sa_key_field(sa),
				 key, keylen);

	rc = crypto4xx_compute_gcm_hash_key_sw(get_dynamic_sa_inner_digest(sa),
		key, keylen);
	if (rc) {
		pr_err("GCM hash key setting failed = %d\n", rc);
		goto err;
	}

	memcpy(ctx->sa_out, ctx->sa_in, ctx->sa_len * 4);
	sa = (struct dynamic_sa_ctl *) ctx->sa_out;
	sa->sa_command_0.bf.dir = DIR_OUTBOUND;
	sa->sa_command_0.bf.opcode = SA_OPCODE_ENCRYPT_HASH;

	return 0;
err:
	crypto4xx_free_sa(ctx);
	return rc;
}

static inline int crypto4xx_crypt_aes_gcm(struct aead_request *req,
					  bool decrypt)
{
	struct crypto4xx_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	unsigned int len = req->cryptlen;
	__le32 iv[4];

	if (crypto4xx_aead_need_fallback(req, false, decrypt))
		return crypto4xx_aead_fallback(req, ctx, decrypt);

	crypto4xx_memcpy_to_le32(iv, req->iv, GCM_AES_IV_SIZE);
	iv[3] = cpu_to_le32(1);

	if (decrypt)
		len -= crypto_aead_authsize(crypto_aead_reqtfm(req));

	return crypto4xx_build_pd(&req->base, ctx, req->src, req->dst,
				  len, iv, sizeof(iv),
				  decrypt ? ctx->sa_in : ctx->sa_out,
				  ctx->sa_len, req->assoclen);
}

int crypto4xx_encrypt_aes_gcm(struct aead_request *req)
{
	return crypto4xx_crypt_aes_gcm(req, false);
}

int crypto4xx_decrypt_aes_gcm(struct aead_request *req)
{
	return crypto4xx_crypt_aes_gcm(req, true);
}

/**
 * HASH SHA1 Functions
 */
static int crypto4xx_hash_alg_init(struct crypto_tfm *tfm,
				   unsigned int sa_len,
				   unsigned char ha,
				   unsigned char hm)
{
	struct crypto_alg *alg = tfm->__crt_alg;
	struct crypto4xx_alg *my_alg;
	struct crypto4xx_ctx *ctx = crypto_tfm_ctx(tfm);
	struct dynamic_sa_hash160 *sa;
	int rc;

	my_alg = container_of(__crypto_ahash_alg(alg), struct crypto4xx_alg,
			      alg.u.hash);
	ctx->dev   = my_alg->dev;

	/* Create SA */
	if (ctx->sa_in || ctx->sa_out)
		crypto4xx_free_sa(ctx);

	rc = crypto4xx_alloc_sa(ctx, sa_len);
	if (rc)
		return rc;

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct crypto4xx_ctx));
	sa = (struct dynamic_sa_hash160 *)ctx->sa_in;
	set_dynamic_sa_command_0(&sa->ctrl, SA_SAVE_HASH, SA_NOT_SAVE_IV,
				 SA_NOT_LOAD_HASH, SA_LOAD_IV_FROM_SA,
				 SA_NO_HEADER_PROC, ha, SA_CIPHER_ALG_NULL,
				 SA_PAD_TYPE_ZERO, SA_OP_GROUP_BASIC,
				 SA_OPCODE_HASH, DIR_INBOUND);
	set_dynamic_sa_command_1(&sa->ctrl, 0, SA_HASH_MODE_HASH,
				 CRYPTO_FEEDBACK_MODE_NO_FB, SA_EXTENDED_SN_OFF,
				 SA_SEQ_MASK_OFF, SA_MC_ENABLE,
				 SA_NOT_COPY_PAD, SA_NOT_COPY_PAYLOAD,
				 SA_NOT_COPY_HDR);
	/* Need to zero hash digest in SA */
	memset(sa->inner_digest, 0, sizeof(sa->inner_digest));
	memset(sa->outer_digest, 0, sizeof(sa->outer_digest));

	return 0;
}

int crypto4xx_hash_init(struct ahash_request *req)
{
	struct crypto4xx_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	int ds;
	struct dynamic_sa_ctl *sa;

	sa = ctx->sa_in;
	ds = crypto_ahash_digestsize(
			__crypto_ahash_cast(req->base.tfm));
	sa->sa_command_0.bf.digest_len = ds >> 2;
	sa->sa_command_0.bf.load_hash_state = SA_LOAD_HASH_FROM_SA;

	return 0;
}

int crypto4xx_hash_update(struct ahash_request *req)
{
	struct crypto_ahash *ahash = crypto_ahash_reqtfm(req);
	struct crypto4xx_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct scatterlist dst;
	unsigned int ds = crypto_ahash_digestsize(ahash);

	sg_init_one(&dst, req->result, ds);

	return crypto4xx_build_pd(&req->base, ctx, req->src, &dst,
				  req->nbytes, NULL, 0, ctx->sa_in,
				  ctx->sa_len, 0);
}

int crypto4xx_hash_final(struct ahash_request *req)
{
	return 0;
}

int crypto4xx_hash_digest(struct ahash_request *req)
{
	struct crypto_ahash *ahash = crypto_ahash_reqtfm(req);
	struct crypto4xx_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct scatterlist dst;
	unsigned int ds = crypto_ahash_digestsize(ahash);

	sg_init_one(&dst, req->result, ds);

	return crypto4xx_build_pd(&req->base, ctx, req->src, &dst,
				  req->nbytes, NULL, 0, ctx->sa_in,
				  ctx->sa_len, 0);
}

/**
 * SHA1 Algorithm
 */
int crypto4xx_sha1_alg_init(struct crypto_tfm *tfm)
{
	return crypto4xx_hash_alg_init(tfm, SA_HASH160_LEN, SA_HASH_ALG_SHA1,
				       SA_HASH_MODE_HASH);
}
