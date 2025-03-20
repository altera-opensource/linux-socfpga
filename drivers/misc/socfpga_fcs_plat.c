// SPDX-License-Identifier: GPL-2.0-or-later OR MIT
/*
 * Copyright (C) 2024 Intel Corporation
 */

#ifndef __SOCFPA_HAL_LL_H
#define __SOCFPA_HAL_LL_H

#include <misc/socfpga_fcs_hal.h>
#include "socfpga_fcs_plat.h"

#include <linux/firmware/intel/stratix10-svc-client.h>
#include <linux/delay.h>

#define INVALID_STATUS		0xFFFFFFFF
#define INVALID_ID		0xFFFFFFFF

#define DIGEST_PARAM_SZ		4
#define DIGEST_SZ_OFFSET	4
#define CRYPTO_ECC_PARAM_SZ	4

#define MSG_RETRY		3
#define RETRY_SLEEP_MS		1
#define TIMEOUT			1000

#define FCS_SVC_CLIENT_NAME "socfpga-fcs"

FCS_HAL_VOID fcs_plat_reinit_completion(FCS_HAL_COMPLETION  *completion)
{
	reinit_completion(completion);
}

static void fcs_atf_version_callback(struct stratix10_svc_client *client,
				     struct stratix10_svc_cb_data *data)
{
	struct socfpga_fcs_priv *priv = client->priv;

	priv->status = data->status;
	if (data->status == BIT(SVC_STATUS_OK)) {
		priv->status = 0;
		priv->atf_version[0] = *((FCS_HAL_UINT *)data->kaddr1);
		priv->atf_version[1] = *((FCS_HAL_UINT *)data->kaddr2);
		priv->atf_version[2] = *((FCS_HAL_UINT *)data->kaddr3);
	} else if (data->status == BIT(SVC_STATUS_ERROR)) {
		priv->status = *((FCS_HAL_UINT *)data->kaddr1);
		dev_err(client->dev, "mbox_error=0x%x\n", priv->status);
	}

	complete(&priv->completion);
}

FCS_HAL_INT fcs_plat_dma_addr_map(struct socfpga_fcs_priv *priv,
				  FCS_HAL_DMA_ADDR *dma_handle,
				  FCS_HAL_VOID *buf, FCS_HAL_SIZE size,
				  FCS_HAL_UINT direction)
{
	struct device *dev = priv->client.dev;

	*dma_handle = dma_map_single(dev, buf, size, direction);
	if (dma_mapping_error(dev, *dma_handle)) {
		dev_err(dev, "DMA mapping failed\n");
		return -EFAULT;
	}

	return 0;
}

FCS_HAL_VOID fcs_plat_dma_addr_unmap(struct socfpga_fcs_priv *priv,
				     FCS_HAL_DMA_ADDR *dma_handle,
				     FCS_HAL_SIZE size, FCS_HAL_UINT direction)
{
	struct device *dev = priv->client.dev;

	dma_unmap_single(dev, *dma_handle, size, direction);
}

FCS_HAL_INT fcs_plat_copy_to_user(FCS_HAL_VOID *dst, FCS_HAL_VOID *src, FCS_HAL_SIZE size)
{
	if (access_ok(dst, size)) {
		if (copy_to_user(dst, src, size)) {
			pr_err("Failed to copy data to user-space\n");
			return -EFAULT;
		}
	} else {
		fcs_plat_memcpy(dst, src, size);
	}

	return 0;
}

FCS_HAL_INT fcs_plat_copy_from_user(FCS_HAL_VOID *dst, FCS_HAL_VOID *src, FCS_HAL_SIZE size)
{
	if (access_ok(src, size)) {
		if (copy_from_user(dst, src, size)) {
			pr_err("Failed to copy data from user-space\n");
			return -EFAULT;
		}
	} else {
		fcs_plat_memcpy(dst, src, size);
	}

	return 0;
}

FCS_HAL_VOID fcs_plat_memset(FCS_HAL_VOID *dst, FCS_HAL_U8 val, FCS_HAL_SIZE size)
{
	memset(dst, val, size);
}

FCS_HAL_VOID fcs_plat_memcpy(FCS_HAL_VOID *dst, FCS_HAL_VOID *src, FCS_HAL_SIZE size)
{
	memcpy(dst, src, size);
}

FCS_HAL_INT fcs_plat_wait_for_completion(FCS_HAL_COMPLETION *completion, FCS_HAL_ULONG timeout)
{
	return wait_for_completion_timeout(completion, timeout);
}

FCS_HAL_VOID fcs_plat_mutex_lock(struct socfpga_fcs_priv *priv)
{
	mutex_lock(&priv->lock);
}

FCS_HAL_VOID fcs_plat_mutex_unlock(struct socfpga_fcs_priv *priv)
{
	mutex_unlock(&priv->lock);
}

FCS_HAL_VOID *fcs_plat_alloc_mem(FCS_HAL_SIZE size)
{
	return kmalloc(size, GFP_KERNEL);
}

FCS_HAL_VOID fcs_plat_free_mem(FCS_HAL_VOID *ptr)
{
	kfree(ptr);
}

FCS_HAL_BOOL fcs_plat_uuid_compare(FCS_HAL_UUID *uuid1, FCS_HAL_UUID *uuid2)
{
	return uuid_equal(uuid1, uuid2);
}

FCS_HAL_VOID fcs_plat_uuid_copy(FCS_HAL_UUID *dst, FCS_HAL_UUID *src)
{
	uuid_copy(dst, src);
}

FCS_HAL_VOID fcs_plat_uuid_generate(struct socfpga_fcs_priv *priv)
{
	uuid_gen(&priv->uuid_id);
}

FCS_HAL_VOID fcs_plat_uuid_clear(struct socfpga_fcs_priv *priv)
{
	memset(&priv->uuid_id, 0, sizeof(FCS_HAL_UUID));
	memset(&priv->session_id, 0, sizeof(FCS_HAL_U32));
}

FCS_HAL_VOID fcs_plat_free_svc_memory(struct socfpga_fcs_priv *priv,
				      void *buf1, void *buf2, void *buf3)
{
	if (buf1)
		stratix10_svc_free_memory(priv->chan, buf1);

	if (buf2)
		stratix10_svc_free_memory(priv->chan, buf2);

	if (buf3)
		stratix10_svc_free_memory(priv->chan, buf3);
}

static FCS_HAL_VOID *plat_sip_svc_allocate_memory(struct socfpga_fcs_priv *priv,
						  size_t size)
{
	return stratix10_svc_allocate_memory(priv->chan, size);
}

static FCS_HAL_VOID plat_sip_svc_free_memory(struct socfpga_fcs_priv *priv,
					     void *buf)
{
	stratix10_svc_free_memory(priv->chan, buf);
}

static FCS_HAL_VOID plat_sip_svc_task_done(struct socfpga_fcs_priv *priv)
{
	stratix10_svc_done(priv->chan);
}

static void soc64_async_callback(void *ptr)
{
	if (ptr)
		complete(ptr);
}

static FCS_HAL_INT plat_sip_svc_send_request(struct socfpga_fcs_priv *priv,
					     enum fcs_command_code command,
					     FCS_HAL_ULONG timeout)
{
	FCS_HAL_BOOL no_async_poll = false;
	FCS_HAL_INT ret = 0;
	int status, index;
	void *handle = NULL;
	struct stratix10_svc_cb_data data;
	struct completion completion;
	struct fcs_cmd_context *k_ctx = &priv->k_ctx;
	FCS_SVC_CLIENT_MSG *msg =
		kzalloc(sizeof(FCS_SVC_CLIENT_MSG), GFP_KERNEL);

	if (!msg) {
		pr_err("failed to allocate memory for svc client message ret: %d\n",
		       ret);
		return -ENOMEM;
	}

	priv->status = 0;
	priv->resp = 0;

	switch (command) {
	case FCS_DEV_CRYPTO_OPEN_SESSION:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_OPEN_SESSION\n");
		msg->command = COMMAND_FCS_CRYPTO_OPEN_SESSION;
		break;

	case FCS_DEV_CRYPTO_CLOSE_SESSION:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_CLOSE_SESSION with session_id: 0x%x\n",
			 priv->session_id);
		msg->arg[0] = priv->session_id;
		msg->command = COMMAND_FCS_CRYPTO_CLOSE_SESSION;
		break;

	case FCS_DEV_ATF_VERSION:
		pr_debug("Sending command: COMMAND_SMC_ATF_BUILD_VER\n");
		msg->command = COMMAND_SMC_ATF_BUILD_VER;
		priv->client.receive_cb = fcs_atf_version_callback;
		break;

	case FCS_DEV_RANDOM_NUMBER_GEN:
		pr_debug("Sending command: COMMAND_FCS_RANDOM_NUMBER_GEN_EXT with session_id: 0x%x, context_id: 0x%x\n",
			 priv->session_id, k_ctx->rng.context_id);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->rng.context_id;
		msg->command = COMMAND_FCS_RANDOM_NUMBER_GEN_EXT;
		msg->payload_output = k_ctx->rng.rng;
		msg->payload_length_output = k_ctx->rng.rng_len;
		break;

	case FCS_DEV_CRYPTO_IMPORT_KEY:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_IMPORT_KEY\n");
		pr_debug("Key: %*ph\n", k_ctx->import_key.key_len,
			 k_ctx->import_key.key);
		msg->payload = k_ctx->import_key.key;
		msg->payload_length = k_ctx->import_key.key_len;
		msg->command = COMMAND_FCS_CRYPTO_IMPORT_KEY;
		break;

	case FCS_DEV_CRYPTO_EXPORT_KEY:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_EXPORT_KEY with session_id: 0x%x, key_id: 0x%x\n",
			 priv->session_id, k_ctx->export_key.key_id);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->export_key.key_id;
		msg->payload_output = k_ctx->export_key.key;
		msg->payload_length_output = *k_ctx->export_key.key_len;
		msg->command = COMMAND_FCS_CRYPTO_EXPORT_KEY;
		break;

	case FCS_DEV_CRYPTO_REMOVE_KEY:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_REMOVE_KEY with session_id: 0x%x, key_id: 0x%x\n",
			 priv->session_id, k_ctx->remove_key.key_id);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->remove_key.key_id;
		msg->command = COMMAND_FCS_CRYPTO_REMOVE_KEY;
		break;

	case FCS_DEV_CRYPTO_GET_KEY_INFO:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_GET_KEY_INFO with session_id: 0x%x, key_id: 0x%x\n",
			 priv->session_id, k_ctx->key_info.key_id);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->key_info.key_id;
		msg->payload_output = k_ctx->key_info.info;
		msg->payload_length_output = *k_ctx->key_info.info_len;
		msg->command = COMMAND_FCS_CRYPTO_GET_KEY_INFO;
		break;

	case FCS_DEV_CRYPTO_CREATE_KEY:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_CREATE_KEY\n");
		pr_debug("Key: %*ph\n", k_ctx->create_key.key_len,
			 k_ctx->create_key.key);
		msg->payload = k_ctx->create_key.key;
		msg->payload_length = k_ctx->create_key.key_len;
		msg->command = COMMAND_FCS_CRYPTO_CREATE_KEY;
		break;

	case FCS_DEV_CRYPTO_HKDF_REQUEST:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_HKDF_REQUEST with session_id: 0x%x, steptype: 0x%x, macmode: 0x%x, key_id: 0x%x\n",
			 priv->session_id, k_ctx->hkdf_req.step_type,
			 k_ctx->hkdf_req.mac_mode, k_ctx->hkdf_req.key_id);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->hkdf_req.step_type;
		msg->arg[2] = k_ctx->hkdf_req.mac_mode;
		msg->arg[3] = k_ctx->hkdf_req.key_id;
		msg->arg[4] = k_ctx->hkdf_req.output_key_obj_len;
		msg->payload = k_ctx->hkdf_req.ikm;
		msg->command = COMMAND_FCS_CRYPTO_HKDF_REQUEST;
		break;

	case FCS_DEV_GET_PROVISION_DATA:
		pr_debug("Sending command: COMMAND_FCS_GET_PROVISION_DATA\n");
		msg->payload_output = k_ctx->prov_data.data;
		msg->payload_length_output = *k_ctx->prov_data.data_len;
		msg->command = COMMAND_FCS_GET_PROVISION_DATA;
		break;

	case FCS_DEV_COUNTER_SET:
		pr_debug("Sending command: COMMAND_FCS_SEND_CERTIFICATE\n");
		msg->payload = k_ctx->ctr_set.ccert;
		msg->payload_length = k_ctx->ctr_set.ccert_len;
		msg->command = COMMAND_FCS_SEND_CERTIFICATE;
		break;

	case FCS_DEV_COUNTER_SET_POLL_SERVICE:
		pr_debug("Sending command: COMMAND_POLL_SERVICE_STATUS\n");
		msg->payload = k_ctx->ctr_set.status;
		msg->payload_length = *k_ctx->ctr_set.status_len;
		msg->command = COMMAND_POLL_SERVICE_STATUS;
		break;

	case FCS_DEV_COUNTER_SET_PREAUTHORIZED:
		pr_debug("Sending command: COMMAND_FCS_COUNTER_SET_PREAUTHORIZED with ctr_type: 0x%x, ctr_val: 0x%x, test: 0x%x\n",
			 k_ctx->ctr_set_preauth.ctr_type,
			 k_ctx->ctr_set_preauth.ctr_val,
			 k_ctx->ctr_set_preauth.test);
		msg->arg[0] = k_ctx->ctr_set_preauth.ctr_type;
		msg->arg[1] = k_ctx->ctr_set_preauth.ctr_val;
		msg->arg[2] = k_ctx->ctr_set_preauth.test;
		msg->command = COMMAND_FCS_COUNTER_SET_PREAUTHORIZED;
		break;

	case FCS_DEV_CRYPTO_GET_DIGEST_INIT:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_GET_DIGEST_INIT with session_id: 0x%x, context_id: 0x%x, key_id: 0x%x,sha_op_mode: 0x%x, sha_digest_sz: 0x%x\n",
			 priv->session_id, k_ctx->dgst.context_id,
			 k_ctx->dgst.key_id, k_ctx->dgst.sha_op_mode,
			 k_ctx->dgst.sha_digest_sz);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->dgst.context_id;
		msg->arg[2] = k_ctx->dgst.key_id;
		msg->arg[3] = DIGEST_PARAM_SZ;
		msg->arg[4] = k_ctx->dgst.sha_op_mode |
			      (k_ctx->dgst.sha_digest_sz << DIGEST_SZ_OFFSET);
		msg->command = COMMAND_FCS_CRYPTO_GET_DIGEST_INIT;
		no_async_poll = true;
		break;

	case FCS_DEV_CRYPTO_GET_DIGEST_UPDATE:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_GET_DIGEST_UPDATE with session_id: 0x%x, context_id: 0x%x, src_len: 0x%x\n",
			 priv->session_id, k_ctx->dgst.context_id,
			 k_ctx->dgst.src_len);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->dgst.context_id;
		msg->payload = k_ctx->dgst.src;
		msg->payload_length = k_ctx->dgst.src_len;
		msg->payload_output = k_ctx->dgst.digest;
		msg->payload_length_output = *k_ctx->dgst.digest_len;
		msg->command = COMMAND_FCS_CRYPTO_GET_DIGEST_UPDATE;
		break;

	case FCS_DEV_CRYPTO_GET_DIGEST_FINAL:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_GET_DIGEST_FINALIZE with session_id: 0x%x, context_id: 0x%x, src_len: 0x%x\n",
			 priv->session_id, k_ctx->dgst.context_id,
			 k_ctx->dgst.src_len);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->dgst.context_id;
		msg->payload = k_ctx->dgst.src;
		msg->payload_length = k_ctx->dgst.src_len;
		msg->payload_output = k_ctx->dgst.digest;
		msg->payload_length_output = *k_ctx->dgst.digest_len;
		msg->command = COMMAND_FCS_CRYPTO_GET_DIGEST_FINALIZE;
		break;

	case FCS_DEV_CRYPTO_MAC_VERIFY_INIT:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_MAC_VERIFY_INIT with session_id: 0x%x, context_id: 0x%x, key_id: 0x%x, sha_op_mode: 0x%x, sha_digest_sz: 0x%x\n",
			 priv->session_id, k_ctx->mac_verify.context_id,
			 k_ctx->mac_verify.key_id, k_ctx->mac_verify.sha_op_mode,
			 k_ctx->mac_verify.sha_digest_sz);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->mac_verify.context_id;
		msg->arg[2] = k_ctx->mac_verify.key_id;
		msg->arg[3] = DIGEST_PARAM_SZ;
		msg->arg[4] =
			k_ctx->mac_verify.sha_op_mode |
			(k_ctx->mac_verify.sha_digest_sz << DIGEST_SZ_OFFSET);
		msg->command = COMMAND_FCS_CRYPTO_MAC_VERIFY_INIT;
		no_async_poll = true;
		break;

	case FCS_DEV_CRYPTO_MAC_VERIFY_UPDATE:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_MAC_VERIFY_UPDATE with session_id: 0x%x, context_id: 0x%x, user_data_size: 0x%x\n",
			 priv->session_id, k_ctx->mac_verify.context_id,
			 k_ctx->mac_verify.user_data_size);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->mac_verify.context_id;
		msg->arg[2] = k_ctx->mac_verify.user_data_size;
		msg->payload = k_ctx->mac_verify.src;
		msg->payload_length = k_ctx->mac_verify.src_size;
		msg->payload_output = k_ctx->mac_verify.dst;
		msg->payload_length_output = *k_ctx->mac_verify.dst_size;
		msg->command = COMMAND_FCS_CRYPTO_MAC_VERIFY_UPDATE;
		break;

	case FCS_DEV_CRYPTO_MAC_VERIFY_FINAL:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_MAC_VERIFY_FINALIZE with session_id: 0x%x, context_id: 0x%x, user_data_size: 0x%x\n",
			 priv->session_id, k_ctx->mac_verify.context_id,
			 k_ctx->mac_verify.user_data_size);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->mac_verify.context_id;
		msg->arg[2] = k_ctx->mac_verify.user_data_size;
		msg->payload = k_ctx->mac_verify.src;
		msg->payload_length = k_ctx->mac_verify.src_size;
		msg->payload_output = k_ctx->mac_verify.dst;
		msg->payload_length_output = *k_ctx->mac_verify.dst_size;
		msg->command = COMMAND_FCS_CRYPTO_MAC_VERIFY_FINALIZE;
		break;

	case FCS_DEV_CRYPTO_AES_CRYPT_INIT:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_AES_CRYPT_INIT with session_id: 0x%x, cid: 0x%x, kid: 0x%x\n",
			 priv->session_id, k_ctx->aes.cid, k_ctx->aes.kid);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->aes.cid;
		msg->arg[2] = k_ctx->aes.kid;
		msg->payload = k_ctx->aes.input;
		msg->payload_length = k_ctx->aes.ip_len;
		msg->payload_output = NULL;
		msg->payload_length_output = 0;
		msg->command = COMMAND_FCS_CRYPTO_AES_CRYPT_INIT;
		no_async_poll = true;
		break;

	case FCS_DEV_CRYPTO_AES_CRYPT_UPDATE:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_AES_CRYPT_UPDATE with session_id:0x%x, cid: 0x%x, kid: 0x%x src = %p, dst = %p, src_size = %d dst_size= %d\n",
			 priv->session_id, k_ctx->aes.cid, k_ctx->aes.kid, k_ctx->aes.input,
			 k_ctx->aes.output, k_ctx->aes.ip_len, *k_ctx->aes.op_len);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->aes.cid;

		if (k_ctx->aes.mode == FCS_AES_BLOCK_MODE_GCM ||
		    k_ctx->aes.mode == FCS_AES_BLOCK_MODE_GHASH)
			msg->arg[2] = k_ctx->aes.input_pad;
		else
			msg->arg[2] = 0;

		msg->payload = k_ctx->aes.input;
		msg->payload_length = k_ctx->aes.ip_len;
		msg->payload_output = k_ctx->aes.output;
		msg->payload_length_output = *k_ctx->aes.op_len;
		msg->command = COMMAND_FCS_CRYPTO_AES_CRYPT_UPDATE;
		break;

	case FCS_DEV_CRYPTO_AES_CRYPT_FINAL:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_AES_CRYPT_FINALIZE with session_id: 0x%x, cid: 0x%x, kid: 0x%x src = %p, dst = %p, src_size = %d dst_size= %d\n",
			 priv->session_id, k_ctx->aes.cid, k_ctx->aes.kid, k_ctx->aes.input,
			 k_ctx->aes.output, k_ctx->aes.ip_len, *k_ctx->aes.op_len);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->aes.cid;

		if (k_ctx->aes.mode == FCS_AES_BLOCK_MODE_GCM ||
		    k_ctx->aes.mode == FCS_AES_BLOCK_MODE_GHASH)
			msg->arg[2] = k_ctx->aes.input_pad;
		else
			msg->arg[2] = 0;

		msg->payload = k_ctx->aes.input;
		msg->payload_length = k_ctx->aes.ip_len;
		msg->payload_output = k_ctx->aes.output;
		msg->payload_length_output = *k_ctx->aes.op_len;
		msg->command = COMMAND_FCS_CRYPTO_AES_CRYPT_FINALIZE;
		break;

	case FCS_DEV_CRYPTO_ECDH_REQUEST_INIT:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_ECDH_REQUEST_INIT with session_id: 0x%x, cid: 0x%x, kid: 0x%x, ecc_curve: 0x%x\n",
			 priv->session_id, k_ctx->ecdh_req.cid,
			 k_ctx->ecdh_req.kid, k_ctx->ecdh_req.ecc_curve);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->ecdh_req.cid;
		msg->arg[2] = k_ctx->ecdh_req.kid;
		msg->arg[3] = CRYPTO_ECC_PARAM_SZ;
		msg->arg[4] = k_ctx->ecdh_req.ecc_curve & FCS_ECC_CURVE_MASK;
		msg->command = COMMAND_FCS_CRYPTO_ECDH_REQUEST_INIT;
		no_async_poll = true;
		break;

	case FCS_DEV_CRYPTO_ECDH_REQUEST_FINALIZE:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_ECDH_REQUEST_FINALIZE with session_id: 0x%x, cid: 0x%x, kid: 0x%x, pubkey_len: 0x%x\n",
			 priv->session_id, k_ctx->ecdh_req.cid,
			 k_ctx->ecdh_req.kid, k_ctx->ecdh_req.pubkey_len);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->ecdh_req.cid;
		msg->arg[2] = k_ctx->ecdh_req.kid;
		msg->payload = k_ctx->ecdh_req.pubkey;
		msg->payload_length = k_ctx->ecdh_req.pubkey_len;
		msg->payload_output = k_ctx->ecdh_req.sh_secret;
		msg->payload_length_output = *k_ctx->ecdh_req.sh_secret_len;
		msg->command = COMMAND_FCS_CRYPTO_ECDH_REQUEST_FINALIZE;
		break;

	case FCS_DEV_CHIP_ID:
		pr_debug("Sending command: COMMAND_FCS_GET_CHIP_ID\n");
		msg->command = COMMAND_FCS_GET_CHIP_ID;
		break;

	case FCS_DEV_ATTESTATION_GET_CERTIFICATE:
		pr_debug("Sending command: COMMAND_FCS_ATTESTATION_CERTIFICATE with cert_request: 0x%x\n",
			 k_ctx->attestation_cert.cert_request);
		msg->payload = NULL;
		msg->payload_length = 0;
		msg->payload_output = k_ctx->attestation_cert.cert;
		msg->payload_length_output = *k_ctx->attestation_cert.cert_size;
		msg->arg[0] = k_ctx->attestation_cert.cert_request;
		msg->command = COMMAND_FCS_ATTESTATION_CERTIFICATE;
		break;

	case FCS_DEV_ATTESTATION_CERTIFICATE_RELOAD:
		pr_debug("Sending command: COMMAND_FCS_ATTESTATION_CERTIFICATE_RELOAD with cert_request: 0x%x\n",
			 k_ctx->attestation_cert_reload.cert_request & 0xff);
		msg->arg[0] = k_ctx->attestation_cert_reload.cert_request &
			      0xff;
		msg->command = COMMAND_FCS_ATTESTATION_CERTIFICATE_RELOAD;
		break;

	case FCS_DEV_MCTP_REQUEST:
		pr_debug("Sending command: COMMAND_FCS_MCTP_SEND with mctp_req_len: 0x%x\n",
			 k_ctx->mctp.mctp_req_len);
		msg->command = COMMAND_FCS_MCTP_SEND;
		msg->payload = k_ctx->mctp.mctp_req;
		msg->payload_length = k_ctx->mctp.mctp_req_len;
		msg->payload_output = k_ctx->mctp.mctp_resp;
		msg->payload_length_output = *k_ctx->mctp.mctp_resp_len;
		break;

	case FCS_DEV_GET_IDCODE:
		pr_debug("Sending command: COMMAND_GET_IDCODE\n");
		msg->command = COMMAND_GET_IDCODE;
		break;

	case FCS_DEV_GET_DEVICE_IDENTITY:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_GET_DEVICE_IDENTITY\n");
		msg->command = COMMAND_FCS_CRYPTO_GET_DEVICE_IDENTITY;
		msg->payload_output = k_ctx->device_identity.identity;
		msg->payload_length_output =
			*k_ctx->device_identity.identity_len;
		break;

	case FCS_DEV_QSPI_OPEN:
		pr_debug("Sending command: COMMAND_QSPI_OPEN\n");
		msg->command = COMMAND_QSPI_OPEN;
		break;

	case FCS_DEV_QSPI_CLOSE:
		pr_debug("Sending command: COMMAND_QSPI_CLOSE\n");
		msg->command = COMMAND_QSPI_CLOSE;
		break;

	case FCS_DEV_QSPI_CS:
		pr_debug("Sending command: COMMAND_QSPI_SET_CS with chipsel: 0x%x\n",
			 k_ctx->qspi_cs.chipsel);
		msg->command = COMMAND_QSPI_SET_CS;
		msg->arg[0] = (k_ctx->qspi_cs.chipsel >> 28) & 0xF;
		msg->arg[1] = (k_ctx->qspi_cs.chipsel >> 27) & 0x1;
		msg->arg[2] = (k_ctx->qspi_cs.chipsel >> 26) & 0x1;
		break;

	case FCS_DEV_QSPI_READ:
		pr_debug("Sending command: COMMAND_QSPI_READ with qspi_addr: 0x%x, qspi_len: 0x%x\n",
			 k_ctx->qspi_read.qspi_addr, k_ctx->qspi_read.qspi_len);
		msg->command = COMMAND_QSPI_READ;
		msg->arg[0] = k_ctx->qspi_read.qspi_addr;
		msg->payload_output = k_ctx->qspi_read.qspi_data;
		msg->payload_length_output = k_ctx->qspi_read.qspi_len * 4;
		break;

	case FCS_DEV_QSPI_WRITE:
		pr_debug("Sending command: COMMAND_QSPI_WRITE with qspi_data_len: 0x%x\n",
			 *k_ctx->qspi_write.qspi_data_len);
		msg->command = COMMAND_QSPI_WRITE;
		msg->payload = k_ctx->qspi_write.qspi_data;
		msg->payload_length = *k_ctx->qspi_write.qspi_data_len;
		break;

	case FCS_DEV_QSPI_ERASE:
		pr_debug("Sending command: COMMAND_QSPI_ERASE with qspi_addr: 0x%x, len: 0x%x\n",
			 k_ctx->qspi_erase.qspi_addr, k_ctx->qspi_erase.len);
		msg->command = COMMAND_QSPI_ERASE;
		msg->arg[0] = k_ctx->qspi_erase.qspi_addr;
		msg->arg[1] = k_ctx->qspi_erase.len * 4;
		break;

	case FCS_DEV_SDOS_DATA_EXT:
		pr_debug("Sending command: COMMAND_FCS_SDOS_DATA_EXT with session_id: 0x%x, context_id: 0x%x, op_mode: 0x%x, own: 0x%llx\n",
			 priv->session_id, k_ctx->sdos.context_id,
			 k_ctx->sdos.op_mode, k_ctx->sdos.own);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->sdos.context_id;
		msg->arg[2] = k_ctx->sdos.op_mode;
		msg->arg[3] = k_ctx->sdos.own;
		msg->payload = k_ctx->sdos.src;
		msg->payload_length = k_ctx->sdos.src_size;
		msg->payload_output = k_ctx->sdos.dst;
		msg->payload_length_output = *k_ctx->sdos.dst_size;
		msg->command = COMMAND_FCS_SDOS_DATA_EXT;
		break;

	case FCS_DEV_CRYPTO_ECDSA_GET_PUBLIC_KEY_INIT:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_ECDSA_GET_PUBLIC_KEY_INIT with session_id: 0x%x, context_id: 0x%x, key_id: 0x%x,ecc_curve: 0x%x\n",
			 priv->session_id, k_ctx->ecdsa_pub_key.context_id,
			 k_ctx->ecdsa_pub_key.key_id,
			 k_ctx->ecdsa_pub_key.ecc_curve & FCS_ECC_CURVE_MASK);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->ecdsa_pub_key.context_id;
		msg->arg[2] = k_ctx->ecdsa_pub_key.key_id;
		msg->arg[3] = CRYPTO_ECC_PARAM_SZ;
		msg->arg[4] = k_ctx->ecdsa_pub_key.ecc_curve &
			      FCS_ECC_CURVE_MASK;
		msg->command = COMMAND_FCS_CRYPTO_ECDSA_GET_PUBLIC_KEY_INIT;
		no_async_poll = true;
		break;

	case FCS_DEV_CRYPTO_ECDSA_GET_PUBLIC_KEY_FINALIZE:
		pr_debug("Sending command: ECDSA_GET_PUBLIC_KEY_FINALIZE with session_id: 0x%x, context_id: 0x%x\n",
			 priv->session_id, k_ctx->ecdsa_pub_key.context_id);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->ecdsa_pub_key.context_id;
		msg->payload = NULL;
		msg->payload_length = 0;
		msg->payload_output = k_ctx->ecdsa_pub_key.pubkey;
		msg->payload_length_output = *k_ctx->ecdsa_pub_key.pubkey_len;
		msg->command = COMMAND_FCS_CRYPTO_ECDSA_GET_PUBLIC_KEY_FINALIZE;
		break;

	case FCS_DEV_CRYPTO_ECDSA_HASH_SIGNING_INIT:
		pr_debug("Sending command: ECDSA_HASH_SIGNING_INIT with session_id: 0x%x, context_id: 0x%x, key_id: 0x%x, ecc_curve: 0x%x\n",
			 priv->session_id, k_ctx->ecdsa_hash_sign.context_id,
			 k_ctx->ecdsa_hash_sign.key_id,
			 k_ctx->ecdsa_hash_sign.ecc_curve & FCS_ECC_CURVE_MASK);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->ecdsa_hash_sign.context_id;
		msg->arg[2] = k_ctx->ecdsa_hash_sign.key_id;
		msg->arg[3] = CRYPTO_ECC_PARAM_SZ;
		msg->arg[4] = k_ctx->ecdsa_hash_sign.ecc_curve &
			      FCS_ECC_CURVE_MASK;
		msg->command = COMMAND_FCS_CRYPTO_ECDSA_HASH_SIGNING_INIT;
		no_async_poll = true;
		break;

	case FCS_DEV_CRYPTO_ECDSA_HASH_SIGNING_FINALIZE:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_ECDSA_HASH_SIGNING_FINALIZE with session_id: 0x%x, context_id: 0x%x, src_len: 0x%x\n",
			 priv->session_id, k_ctx->ecdsa_hash_sign.context_id,
			 k_ctx->ecdsa_hash_sign.src_len);

		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->ecdsa_hash_sign.context_id;
		msg->payload = k_ctx->ecdsa_hash_sign.src;
		msg->payload_length = k_ctx->ecdsa_hash_sign.src_len;
		msg->payload_output = k_ctx->ecdsa_hash_sign.dst;
		msg->payload_length_output = *k_ctx->ecdsa_hash_sign.dst_len;
		msg->command = COMMAND_FCS_CRYPTO_ECDSA_HASH_SIGNING_FINALIZE;
		break;

	case FCS_DEV_CRYPTO_ECDSA_HASH_VERIFY_INIT:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_ECDSA_HASH_VERIFY_INIT with session_id: 0x%x, context_id: 0x%x, key_id: 0x%x, ecc_curve: 0x%x\n",
			 priv->session_id, k_ctx->ecdsa_hash_verify.context_id,
			 k_ctx->ecdsa_hash_verify.key_id,
			 k_ctx->ecdsa_hash_verify.ecc_curve &
			 FCS_ECC_CURVE_MASK);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->ecdsa_hash_verify.context_id;
		msg->arg[2] = k_ctx->ecdsa_hash_verify.key_id;
		msg->arg[3] = CRYPTO_ECC_PARAM_SZ;
		msg->arg[4] = k_ctx->ecdsa_hash_verify.ecc_curve &
			      FCS_ECC_CURVE_MASK;
		msg->command = COMMAND_FCS_CRYPTO_ECDSA_HASH_VERIFY_INIT;
		no_async_poll = true;
		break;

	case FCS_DEV_CRYPTO_ECDSA_HASH_VERIFY_FINALIZE:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_ECDSA_HASH_VERIFY_FINALIZE with session_id: 0x%x, context_id: 0x%x, src_len: 0x%x\n",
			 priv->session_id, k_ctx->ecdsa_hash_verify.context_id,
			 k_ctx->ecdsa_hash_verify.src_len);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->ecdsa_hash_verify.context_id;
		msg->payload = k_ctx->ecdsa_hash_verify.src;
		msg->payload_length = k_ctx->ecdsa_hash_verify.src_len;
		msg->payload_output = k_ctx->ecdsa_hash_verify.dst;
		msg->payload_length_output = *k_ctx->ecdsa_hash_verify.dst_len;
		msg->command = COMMAND_FCS_CRYPTO_ECDSA_HASH_VERIFY_FINALIZE;
		break;

	case FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_SIGNING_INIT:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_INIT with session_id: 0x%x, context_id: 0x%x, key_id: 0x%x, ecc_curve: 0x%x\n",
			 priv->session_id,
			 k_ctx->ecdsa_sha2_data_sign.context_id,
			 k_ctx->ecdsa_sha2_data_sign.key_id,
			 k_ctx->ecdsa_sha2_data_sign.ecc_curve &
			 FCS_ECC_CURVE_MASK);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->ecdsa_sha2_data_sign.context_id;
		msg->arg[2] = k_ctx->ecdsa_sha2_data_sign.key_id;
		msg->arg[3] = CRYPTO_ECC_PARAM_SZ;
		msg->arg[4] = k_ctx->ecdsa_sha2_data_sign.ecc_curve &
			      FCS_ECC_CURVE_MASK;
		msg->command = COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_INIT;
		no_async_poll = true;
		break;

	case FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_SIGNING_UPDATE:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_UPDATE with session_id: 0x%x, context_id: 0x%x, src_len: 0x%x\n",
			 priv->session_id,
			 k_ctx->ecdsa_sha2_data_sign.context_id,
			 k_ctx->ecdsa_sha2_data_sign.src_len);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->ecdsa_sha2_data_sign.context_id;
		msg->payload = k_ctx->ecdsa_sha2_data_sign.src;
		msg->payload_length = k_ctx->ecdsa_sha2_data_sign.src_len;
		msg->payload_output = k_ctx->ecdsa_sha2_data_sign.dst;
		msg->payload_length_output =
			*k_ctx->ecdsa_sha2_data_sign.dst_len;
		msg->command =
			COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_UPDATE;
		break;

	case FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_SIGNING_FINALIZE:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_FINALIZE with session_id: 0x%x, context_id: 0x%x, src_len: 0x%x\n",
			 priv->session_id,
			 k_ctx->ecdsa_sha2_data_sign.context_id,
			 k_ctx->ecdsa_sha2_data_sign.src_len);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->ecdsa_sha2_data_sign.context_id;
		msg->payload = k_ctx->ecdsa_sha2_data_sign.src;
		msg->payload_length = k_ctx->ecdsa_sha2_data_sign.src_len;
		msg->payload_output = k_ctx->ecdsa_sha2_data_sign.dst;
		msg->payload_length_output =
			*k_ctx->ecdsa_sha2_data_sign.dst_len;
		msg->command =
			COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_FINALIZE;
		break;

	case FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_VERIFY_INIT:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_INIT with session_id: 0x%x, context_id: 0x%x, key_id: 0x%x, ecc_curve: 0x%x\n",
			 priv->session_id,
			 k_ctx->ecdsa_sha2_data_verify.context_id,
			 k_ctx->ecdsa_sha2_data_verify.key_id,
			 k_ctx->ecdsa_sha2_data_verify.ecc_curve &
			 FCS_ECC_CURVE_MASK);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->ecdsa_sha2_data_verify.context_id;
		msg->arg[2] = k_ctx->ecdsa_sha2_data_verify.key_id;
		msg->arg[3] = CRYPTO_ECC_PARAM_SZ;
		msg->arg[4] = k_ctx->ecdsa_sha2_data_verify.ecc_curve &
			      FCS_ECC_CURVE_MASK;
		msg->command = COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_INIT;
		no_async_poll = true;
		break;

	case FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_VERIFY_FINALIZE:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_FINALIZE with session_id: 0x%x, context_id: 0x%x, user_data_sz: 0x%x\n",
			priv->session_id,
			k_ctx->ecdsa_sha2_data_verify.context_id,
			k_ctx->ecdsa_sha2_data_verify.user_data_sz);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->ecdsa_sha2_data_verify.context_id;
		msg->arg[2] = k_ctx->ecdsa_sha2_data_verify.user_data_sz;
		msg->payload = k_ctx->ecdsa_sha2_data_verify.src;
		msg->payload_length = k_ctx->ecdsa_sha2_data_verify.src_len;
		msg->payload_output = k_ctx->ecdsa_sha2_data_verify.dst;
		msg->payload_length_output =
			*k_ctx->ecdsa_sha2_data_verify.dst_len;
		msg->command = COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_FINALIZE;
		break;

	case FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_VERIFY_UPDATE:
		pr_debug("Sending command: COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_UPDATE with session_id: 0x%x, context_id: 0x%x, user_data_sz: 0x%x\n",
			priv->session_id,
			k_ctx->ecdsa_sha2_data_verify.context_id,
			k_ctx->ecdsa_sha2_data_verify.user_data_sz);
		msg->arg[0] = priv->session_id;
		msg->arg[1] = k_ctx->ecdsa_sha2_data_verify.context_id;
		msg->arg[2] = k_ctx->ecdsa_sha2_data_verify.user_data_sz;
		msg->payload = k_ctx->ecdsa_sha2_data_verify.src;
		msg->payload_length = k_ctx->ecdsa_sha2_data_verify.src_len;
		msg->payload_output = k_ctx->ecdsa_sha2_data_verify.dst;
		msg->payload_length_output =
			*k_ctx->ecdsa_sha2_data_verify.dst_len;
		msg->command = COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_UPDATE;
		break;

	case FCS_DEV_HPS_IMG_VALIDATE_REQUEST:
		pr_debug("Sending command: COMMAND_FCS_SEND_CERTIFICATE with vab_cert_len: 0x%x\n",
			k_ctx->hps_img_validate.vab_cert_len);
		msg->payload = k_ctx->hps_img_validate.vab_cert;
		msg->payload_length = k_ctx->hps_img_validate.vab_cert_len;
		msg->command = COMMAND_FCS_SEND_CERTIFICATE;
		break;

	default:
		pr_err("Unknown command: 0x%x\n", command);
		ret = -EINVAL;
		break;
	}

	if (command == FCS_DEV_ATF_VERSION) {
		reinit_completion(&priv->completion);

		ret = stratix10_svc_send(priv->chan, msg);
		if (ret) {
			pr_err("failed to send message to service channel\n");
			goto fun_ret;
		}

		if (!wait_for_completion_timeout(&priv->completion, timeout)) {
			pr_err("svc timeout to get completed status\n");
			ret = -ETIMEDOUT;
		}
fun_ret:
		kfree(msg);
		return ret;
	}

	init_completion(&completion);

	for (index = 0; index < MSG_RETRY; index++) {
		status = stratix10_svc_async_send(priv->chan, msg, &handle,
						  soc64_async_callback,
						  &completion);
		if (status == 0)
			break;
		msleep(RETRY_SLEEP_MS);
	}

	if (!handle || status != 0) {
		pr_err("Failed to send async message\n");
		return -ETIMEDOUT;
	}

	if (!no_async_poll) {
		ret = wait_for_completion_io_timeout(&completion, (TIMEOUT));
		if (ret > 0)
			pr_debug("Received async interrupt\n");
		else
			pr_err("timeout occurred while waiting for async message\n");

		ret = stratix10_svc_async_poll(priv->chan, handle, &data);
		if (ret) {
			pr_err("Failed to poll async message\n");
			goto out;
		}

		priv->status = data.status;

		if (data.kaddr1) {
			if (command == FCS_DEV_CHIP_ID) {
				priv->chip_id_lo =
					*((FCS_HAL_UINT *)data.kaddr1);
				priv->chip_id_hi =
					*((FCS_HAL_UINT *)data.kaddr2);
			} else {
				priv->resp = *((FCS_HAL_U32 *)data.kaddr1);
			}
		} else {
			priv->resp = 0;
		}
	}

out:
	stratix10_svc_async_done(priv->chan, handle);
	kfree(msg);

	return ret;
}

FCS_HAL_INT fcs_plat_init(FCS_HAL_DEV *dev, struct socfpga_fcs_priv *priv)
{
	mutex_init(&priv->lock);
	FCS_HAL_S32 ret = 0;

	priv->plat_data =
		kmalloc(sizeof(struct socfpga_fcs_service_ops), GFP_KERNEL);
	if (!priv->plat_data) {
		pr_err("Failed to allocate memory for priv->plat_data\n");
		return -ENOMEM;
	}

	priv->dev = dev;
	priv->client.dev = dev;
	priv->client.receive_cb = NULL;
	priv->client.priv = priv;

	priv->chan = stratix10_svc_request_channel_byname(&priv->client,
							  SVC_CLIENT_FCS);
	if (IS_ERR(priv->chan)) {
		pr_err("couldn't get service channel %s\n", SVC_CLIENT_FCS);
		return -ENODEV;
	}

	ret = stratix10_svc_add_async_client(priv->chan, true);
	if (ret) {
		pr_err("Failed to add async client\n");
		return ret;
	}

	init_completion(&priv->completion);

	priv->plat_data->svc_send_request = plat_sip_svc_send_request;
	priv->plat_data->svc_alloc_memory = plat_sip_svc_allocate_memory;
	priv->plat_data->svc_free_memory = plat_sip_svc_free_memory;
	priv->plat_data->svc_task_done = plat_sip_svc_task_done;

	return 0;
}

FCS_HAL_VOID fcs_plat_cleanup(struct socfpga_fcs_priv *priv)
{
	stratix10_svc_free_channel(priv->chan);
}

#endif /* __SOCFPA_HAL_LL_H */
