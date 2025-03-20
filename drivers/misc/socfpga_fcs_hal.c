// SPDX-License-Identifier: GPL-2.0-or-later OR MIT
/*
 * Copyright (C) 2025 Altera Corporation
 */

#include <misc/socfpga_fcs_hal.h>
#include "socfpga_fcs_plat.h"

#define RANDOM_NUMBER_EXT_HDR_SIZE		12
#define FCS_CRYPTO_KEY_HEADER_SIZE		12
#define CERTIFICATE_RSP_MAX_SZ			4096
#define MBOX_SEND_RSP_MAX_SZ			4096
#define QSPI_READ_LEN_MAX			4096
#define MCTP_MAX_LEN				4096
#define DEVICE_IDENTITY_MAX_LEN			4096
#define QSPI_GET_INFO_LEN			36

#define HKDF_REQ_SZ_MAX				4096

#define DIGEST_CMD_MAX_SZ			SZ_4M
#define CRYPTO_DIGEST_MAX_SZ			SZ_4M
#define MAC_CMD_MAX_SZ				SZ_4M
#define FCS_ECC_PUBKEY_LEN			SZ_4M
#define FCS_ECDSA_HASH_SIGN_MAX_LEN		SZ_4M
#define FCS_ECDSA_HSHA2_DATA_SIGN_BLOCK_SZ	SZ_4M
#define FCS_ECDSA_SHA2_DATA_VERIFY_BLOCK_SZ	SZ_4M
#define FCS_ECDSA_DATA_SIGN_VERIFY_MAX_LEN	SZ_4M

#define FCS_ECDSA_SHA2_DATA_VERIFY_RSP_SZ	SZ_4M

#define WORDS_TO_BYTES_SIZE			4 /* 4 bytes in a word */

#define CRYPTO_SERVICE_MIN_DATA_SIZE		8

#define OWNER_ID_OFFSET				12
#define OWNER_ID_SIZE				8

#define RESPONSE_HEADER_SIZE			12

/*SDM required minimum 8 bytes of data for crypto service*/
#define DIGEST_SERVICE_MIN_DATA_SIZE	8
#define FCS_AES_IV_SZ			16
#define FCS_POLL_STATUS_LEN		4
#define FCS_AES_REQUEST_TIMEOUT		(10 * FCS_REQUEST_TIMEOUT)
#define FCS_CRYPTO_BLOCK_SZ		(4 * 1024 * 1024)
#define FCS_AES_CRYPT_BLOCK_SZ		FCS_CRYPTO_BLOCK_SZ
#define AES_PARAMS_CRYPT_OFFSET		1
#define AES_PARAMS_TAG_LEN_OFFSET	2
#define AES_PARAMS_IV_TYPE_OFFSET	4
#define AES_PARAMS_AAD_LEN_OFFSET	8
#define FCS_AES_PARAMS_ECB_SZ		12
#define GCM_TAG_LEN			16
#define GCM_AAD_ALIGN			16
#define GCM_DATA_ALIGN			16
#define NON_GCM_DATA_ALIGN		32
#define FCS_STATUS_LEN			4
#define FCS_ECDSA_CRYPTO_BLOCK_SZ	FCS_CRYPTO_BLOCK_SZ

/* HKDF input payload size with 1st and 2nd input */
#define HKDF_INPUT_DATA_SIZE		80

FCS_HAL_INT hal_session_close(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	struct fcs_cmd_context ctx;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	ret = fcs_plat_uuid_compare(&priv->uuid_id,
				    &k_ctx->close_session.suuid);
	if (!ret) {
		ret = -EINVAL;
		LOG_ERR("Session UUID Mismatch ret: %d\n", ret);
		return ret;
	}

	ret = priv->plat_data->svc_send_request(
		priv, FCS_DEV_CRYPTO_CLOSE_SESSION, FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_CRYPTO_CLOSE_SESSION, ret);
		return ret;
	}

	fcs_plat_uuid_clear(priv);

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to close session ret: %d\n",
			ret);
	}

	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mail box status code to user ret: %d\n",
			ret);
	}

	priv->plat_data->svc_task_done(priv);

	return ret;
}
EXPORT_SYMBOL(hal_session_close);

FCS_HAL_INT hal_session_open(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;

	ret = priv->plat_data->svc_send_request(
		priv, FCS_DEV_CRYPTO_OPEN_SESSION, FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_CRYPTO_OPEN_SESSION, ret);
		return ret;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to open session ret: %d\n", ret);
		goto copy_mbox_status;
	}

	fcs_plat_uuid_generate(priv);

	memcpy(&priv->session_id, &priv->resp, sizeof(priv->session_id));

	ret = fcs_plat_copy_to_user(k_ctx->open_session.suuid, &priv->uuid_id,
				    sizeof(FCS_HAL_UUID));
	if (ret) {
		LOG_ERR("Failed to copy session ID to user suuid addr: %p ret: %d\n",
			k_ctx->open_session.suuid, ret);
		goto copy_mbox_status;
	}

copy_mbox_status:
	ret = fcs_plat_copy_to_user(k_ctx->error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mail box status code to user ret: %d\n",
			ret);
	}
	priv->plat_data->svc_task_done(priv);

	return ret;
}
EXPORT_SYMBOL(hal_session_open);

FCS_HAL_VOID hal_get_atf_version(FCS_HAL_U32 *version)
{
	fcs_plat_memcpy(version, priv->atf_version, sizeof(priv->atf_version));
}
EXPORT_SYMBOL(hal_get_atf_version);

FCS_HAL_INT hal_import_key(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	FCS_HAL_UINT sbuf_size;
	FCS_HAL_VOID *s_buf = NULL;
	FCS_HAL_DMA_ADDR fcs_dma_handle_src;
	struct fcs_cmd_context ctx;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	ret = fcs_plat_uuid_compare(&priv->uuid_id, &k_ctx->import_key.suuid);
	if (!ret) {
		ret = -EINVAL;
		LOG_ERR("Session UUID Mismatch ret: %d\n", ret);
		return ret;
	}

	sbuf_size = k_ctx->import_key.key_len + FCS_CRYPTO_KEY_HEADER_SIZE;

	s_buf = priv->plat_data->svc_alloc_memory(priv, sbuf_size);
	if (IS_ERR(s_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for source buffer ret: %d\n",
			ret);
		return ret;
	}

	/* Copy the session ID into the source buffer */
	fcs_plat_memcpy(s_buf, &priv->session_id, sizeof(FCS_HAL_U32));

	/* Copy the key data from user space to the source buffer */
	ret = fcs_plat_copy_from_user(s_buf + FCS_CRYPTO_KEY_HEADER_SIZE,
				      k_ctx->import_key.key,
				      k_ctx->import_key.key_len);
	if (ret) {
		LOG_ERR("Failed to copy data from user to kernel source buffer ret: %d\n",
			ret);
		goto free_mem;
	}

	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_src, s_buf, sbuf_size,
				    FCS_DMA_TO_DEVICE);
	if (ret) {
		LOG_ERR("Failed perform dma address map for the kernel source buffer ret: %d\n",
			ret);
		goto free_mem;
	}

	k_ctx->import_key.key = s_buf;
	k_ctx->import_key.key_len = sbuf_size;

	ret = priv->plat_data->svc_send_request(priv, FCS_DEV_CRYPTO_IMPORT_KEY,
						FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_CRYPTO_IMPORT_KEY, ret);
		ret = -EFAULT;
		goto unmap;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to import key ret: %d\n", ret);
		goto copy_mbox_status;
	}

	ret = fcs_plat_copy_to_user(ctx.import_key.status, &priv->resp, 1);
	if (ret) {
		LOG_ERR("Failed to copy import key response data to user ret: %d\n",
			ret);
		goto copy_mbox_status;
	}

copy_mbox_status:
	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mbox error code to user ret: %d\n",
			ret);
		ret = -EFAULT;
	}
	priv->plat_data->svc_task_done(priv);
unmap:
	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_src, sbuf_size,
				FCS_DMA_TO_DEVICE);
free_mem:
	priv->plat_data->svc_free_memory(priv, s_buf);

	return ret;
}
EXPORT_SYMBOL(hal_import_key);

FCS_HAL_INT hal_export_key(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	FCS_HAL_VOID *d_buf = NULL;
	FCS_HAL_UINT key_len = CRYPTO_EXPORTED_KEY_OBJECT_MAX_SZ;
	FCS_HAL_DMA_ADDR fcs_dma_handle_dst;
	struct fcs_cmd_context ctx;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	ret = fcs_plat_uuid_compare(&priv->uuid_id, &k_ctx->export_key.suuid);
	if (!ret) {
		ret = -EINVAL;
		LOG_ERR("Session UUID Mismatch ret: %d\n", ret);
		return ret;
	}

	d_buf = priv->plat_data->svc_alloc_memory(priv,
						  key_len + FCS_STATUS_LEN);
	if (IS_ERR(d_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for key object ret: %d\n",
			ret);
		return ret;
	}

	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_dst, d_buf,
				    key_len + FCS_STATUS_LEN,
				    FCS_DMA_FROM_DEVICE);
	if (ret)
		goto free_dest;

	k_ctx->export_key.key = d_buf;
	k_ctx->export_key.key_len = &key_len;

	ret = priv->plat_data->svc_send_request(priv, FCS_DEV_CRYPTO_EXPORT_KEY,
						FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_CRYPTO_EXPORT_KEY, ret);
		goto unmap;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to export key ret: %d\n", ret);
		goto copy_mbox_status;
	}

	ret = fcs_plat_copy_to_user(ctx.export_key.key, d_buf + FCS_STATUS_LEN,
				    priv->resp - FCS_STATUS_LEN);
	if (ret) {
		LOG_ERR("Failed to copy key to user ret: %d\n", ret);
		goto copy_mbox_status;
	}

	ret = fcs_plat_copy_to_user(ctx.export_key.key_len, &priv->resp,
				    sizeof(priv->resp));
	if (ret) {
		LOG_ERR("Failed to copy key length to user ret: %d\n", ret);
		goto copy_mbox_status;
	}

copy_mbox_status:
	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mail box status code to user ret: %d\n",
			ret);
	}
	priv->plat_data->svc_task_done(priv);
unmap:
	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst,
				CRYPTO_EXPORTED_KEY_OBJECT_MAX_SZ,
				FCS_DMA_FROM_DEVICE);
free_dest:
	priv->plat_data->svc_free_memory(priv, d_buf);

	return ret;
}
EXPORT_SYMBOL(hal_export_key);

FCS_HAL_INT hal_remove_key(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	struct fcs_cmd_context ctx;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	ret = fcs_plat_uuid_compare(&priv->uuid_id, &k_ctx->remove_key.suuid);
	if (!ret) {
		ret = -EINVAL;
		LOG_ERR("Session UUID Mismatch ret: %d\n", ret);
		return ret;
	}

	ret = priv->plat_data->svc_send_request(priv, FCS_DEV_CRYPTO_REMOVE_KEY,
						FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_CRYPTO_REMOVE_KEY, ret);
		return ret;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to remove key ret: %d\n", ret);
	}

	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mail box status code to user ret: %d\n",
			ret);
	}

	priv->plat_data->svc_task_done(priv);

	return ret;
}
EXPORT_SYMBOL(hal_remove_key);

FCS_HAL_INT hal_get_key_info(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	FCS_HAL_UINT info_len = CRYPTO_KEY_INFO_MAX_SZ;
	FCS_HAL_VOID *d_buf = NULL;
	FCS_HAL_DMA_ADDR fcs_dma_handle_dst;
	struct fcs_cmd_context ctx;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	ret = fcs_plat_uuid_compare(&priv->uuid_id, &k_ctx->key_info.suuid);
	if (!ret) {
		ret = -EINVAL;
		LOG_ERR("session UUID Mismatch ret: %d\n", ret);
		return ret;
	}

	/* Allocate memory for the key info kernel buffer */
	d_buf = priv->plat_data->svc_alloc_memory(priv, CRYPTO_KEY_INFO_MAX_SZ);
	if (IS_ERR(d_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for key info kernel buffer ret: %d\n",
			ret);
		return ret;
	}

	/* Map the destination buffer for DMA */
	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_dst, d_buf,
				    CRYPTO_KEY_INFO_MAX_SZ,
				    FCS_DMA_FROM_DEVICE);
	if (ret) {
		LOG_ERR("Failed perform dma address map for the key info kernel buffer ret: %d\n",
			ret);
		goto free_dest;
	}

	k_ctx->key_info.info = d_buf;
	k_ctx->key_info.info_len = &info_len;

	/* Send the request to get key info */
	ret = priv->plat_data->svc_send_request(
		priv, FCS_DEV_CRYPTO_GET_KEY_INFO, FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_CRYPTO_GET_KEY_INFO, ret);
		goto unmap;
	}

	/* Check if there was a mailbox error during key info retrieval */
	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to get key info ret: %d\n", ret);
		goto copy_mbox_status;
	}

	/* Copy the key info from kernel space to user space */
	ret = fcs_plat_copy_to_user(ctx.key_info.info, d_buf, priv->resp);
	if (ret) {
		LOG_ERR("Failed to copy key info to user ret: %d\n", ret);
		goto copy_mbox_status;
	}

	/* Copy the key info length from kernel space to user space */
	ret = fcs_plat_copy_to_user(ctx.key_info.info_len, &priv->resp,
				    sizeof(priv->resp));
	if (ret) {
		LOG_ERR("Failed to copy key info length to user ret: %d\n",
			ret);
		goto copy_mbox_status;
	}

copy_mbox_status:
	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
	}
	priv->plat_data->svc_task_done(priv);
unmap:
	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst,
				CRYPTO_KEY_INFO_MAX_SZ, FCS_DMA_FROM_DEVICE);
free_dest:
	priv->plat_data->svc_free_memory(priv, d_buf);

	return ret;
}
EXPORT_SYMBOL(hal_get_key_info);

FCS_HAL_INT hal_create_key(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	FCS_HAL_VOID *s_buf = NULL;
	FCS_HAL_DMA_ADDR fcs_dma_handle_src;
	struct fcs_cmd_context ctx;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	/* Compare the session UUIDs to check for a match */
	ret = fcs_plat_uuid_compare(&priv->uuid_id, &k_ctx->create_key.suuid);
	if (!ret) {
		ret = -EINVAL;
		LOG_ERR("session UUID Mismatch ret: %d\n", ret);
		return ret;
	}

	/* Calculate the total key length including the header */
	k_ctx->create_key.key_len =
		FCS_CRYPTO_KEY_HEADER_SIZE + k_ctx->create_key.key_len;

	/* Allocate memory for the key object */
	s_buf = priv->plat_data->svc_alloc_memory(priv,
						  k_ctx->create_key.key_len);
	if (IS_ERR(s_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for key object ret: %d\n",
			ret);
		return ret;
	}

	/* Copy the session ID into the source buffer */
	fcs_plat_memcpy(s_buf, &priv->session_id, sizeof(FCS_HAL_U32));

	/* Copy the key object data from user space to the source buffer */
	ret = fcs_plat_copy_from_user(s_buf + FCS_CRYPTO_KEY_HEADER_SIZE,
				      k_ctx->create_key.key,
				      k_ctx->create_key.key_len);
	if (ret) {
		LOG_ERR("Failed to copy data from user to kernel source buffer ret: %d\n",
			ret);
		goto free_mem;
	}

	/* Map the source buffer for DMA */
	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_src, s_buf,
				    ctx.create_key.key_len, FCS_DMA_TO_DEVICE);
	if (ret) {
		LOG_ERR("Failed perform dma address map for the key object ret: %d\n",
			ret);
		goto free_mem;
	}

	k_ctx->create_key.key = s_buf;

	/* Send the request to create key */
	ret = priv->plat_data->svc_send_request(priv, FCS_DEV_CRYPTO_CREATE_KEY,
						FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_CRYPTO_CREATE_KEY, ret);
		goto unmap;
	}

	/* Check if there was a mailbox error during key creation */
	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to create key ret: %d\n", ret);
		goto copy_mbox_status;
	}

	ret = fcs_plat_copy_to_user(ctx.create_key.status, &priv->resp, 1);
	if (ret) {
		LOG_ERR("Failed to copy create key status to user ret: %d\n",
			ret);
		goto copy_mbox_status;
	}

copy_mbox_status:
	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
	}
	priv->plat_data->svc_task_done(priv);

unmap:
	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_src,
				ctx.create_key.key_len, FCS_DMA_TO_DEVICE);

free_mem:
	priv->plat_data->svc_free_memory(priv, s_buf);

	return ret;
}
EXPORT_SYMBOL(hal_create_key);

FCS_HAL_INT hal_get_provision_data(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	FCS_HAL_U32 data_len = CRYPTO_PROVISION_DATA_MAX_SZ;
	FCS_HAL_VOID *d_buf = NULL;
	FCS_HAL_DMA_ADDR fcs_dma_handle_dst;
	struct fcs_cmd_context ctx;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	d_buf = priv->plat_data->svc_alloc_memory(priv, data_len);
	if (IS_ERR(d_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for provision data ret: %d\n",
			ret);
		return ret;
	}

	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_dst, d_buf, data_len,
				    FCS_DMA_FROM_DEVICE);
	if (ret)
		goto free_dest;

	k_ctx->prov_data.data = d_buf;
	k_ctx->prov_data.data_len = &data_len;

	ret = priv->plat_data->svc_send_request(
		priv, FCS_DEV_GET_PROVISION_DATA, FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_GET_PROVISION_DATA, ret);
		goto unmap;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, get provision data request Failed ret: %d\n",
			ret);
		goto copy_mbox_status;
	}

	ret = fcs_plat_copy_to_user(ctx.prov_data.data, d_buf, priv->resp);
	if (ret) {
		LOG_ERR("Failed to copy provision data to user ret: %d\n", ret);
		goto copy_mbox_status;
	}

	ret = fcs_plat_copy_to_user(ctx.prov_data.data_len, &priv->resp,
				    sizeof(priv->resp));
	if (ret) {
		LOG_ERR("Failed to copy provision data length to user ret: %d\n",
			ret);
		goto copy_mbox_status;
	}

copy_mbox_status:
	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret)
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);

	priv->plat_data->svc_task_done(priv);
unmap:
	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst,
				CRYPTO_PROVISION_DATA_MAX_SZ,
				FCS_DMA_FROM_DEVICE);
free_dest:
	priv->plat_data->svc_free_memory(priv, d_buf);

	return ret;
}
EXPORT_SYMBOL(hal_get_provision_data);

FCS_HAL_INT hal_counter_set(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT tsz, datasz;
	FCS_HAL_INT ret = 0;
	FCS_HAL_VOID *s_buf = NULL;
	FCS_HAL_DMA_ADDR fcs_dma_handle_src;
	struct fcs_cmd_context ctx;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	/* Allocate memory for certificate + test word */
	tsz = sizeof(FCS_HAL_U32);
	datasz = ctx.ctr_set.ccert_len + tsz;

	s_buf = priv->plat_data->svc_alloc_memory(priv, datasz);
	if (IS_ERR(s_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for counter set kernel buffer ret: %d\n",
			ret);
		return ret;
	}

	ret = fcs_plat_copy_from_user(s_buf, &k_ctx->ctr_set.cache, tsz);
	if (ret) {
		LOG_ERR("Failed to copy cache to kernel buffer ret: %d\n", ret);
		goto free_mem;
	}

	ret = fcs_plat_copy_from_user(s_buf + tsz, k_ctx->ctr_set.ccert,
				      k_ctx->ctr_set.ccert_len);
	if (ret) {
		LOG_ERR("Failed to copy certificate to kernel buffer ret: %d\n",
			ret);
		goto free_mem;
	}

	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_src, s_buf, datasz,
				    FCS_DMA_TO_DEVICE);
	if (ret) {
		LOG_ERR("Failed to map dma address for kernel buffer ret: %d\n",
			ret);
		goto free_mem;
	}

	k_ctx->ctr_set.ccert = s_buf;
	k_ctx->ctr_set.ccert_len = datasz;

	ret = priv->plat_data->svc_send_request(priv, FCS_DEV_COUNTER_SET,
						FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_COUNTER_SET, ret);
		goto unmap;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to set counter ret: %d\n", ret);
		goto copy_mbox_status;
	}

	ret = fcs_plat_copy_to_user(ctx.ctr_set.status, &priv->resp, 1);
	if (ret) {
		LOG_ERR("Failed to copy counter set status to user ret: %d\n",
			ret);
		goto copy_mbox_status;
	}

copy_mbox_status:
	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
	}
	priv->plat_data->svc_task_done(priv);
unmap:
	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_src, datasz,
				FCS_DMA_TO_DEVICE);
free_mem:
	priv->plat_data->svc_free_memory(priv, s_buf);

	return ret;
}
EXPORT_SYMBOL(hal_counter_set);

FCS_HAL_INT hal_counter_set_preauth(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	struct fcs_cmd_context ctx;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	ret = priv->plat_data->svc_send_request(
		priv, FCS_DEV_COUNTER_SET_PREAUTHORIZED, FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_COUNTER_SET_PREAUTHORIZED, ret);
		ret = -EFAULT;
		goto unmap;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to set counter preauthorized ret: %d\n",
			ret);
		goto copy_mbox_status;
	}

copy_mbox_status:
	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
	}
	priv->plat_data->svc_task_done(priv);

unmap:
	return ret;
}
EXPORT_SYMBOL(hal_counter_set_preauth);

FCS_HAL_INT hal_random_number(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	FCS_HAL_VOID *s_buf = NULL;
	FCS_HAL_DMA_ADDR fcs_dma_handle_src;
	struct fcs_cmd_context ctx;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	/* Compare the session UUIDs to check for a match. Here suuid is set
	 * through hal_store_context
	 */
	ret = fcs_plat_uuid_compare(&priv->uuid_id, &k_ctx->rng.suuid);
	if (!ret) {
		ret = -EINVAL;
		LOG_ERR("session UUID Mismatch ret: %d\n", ret);
		return ret;
	}

	k_ctx->rng.rng_len = ctx.rng.rng_len;

	s_buf = priv->plat_data->svc_alloc_memory(
		priv, k_ctx->rng.rng_len + RANDOM_NUMBER_EXT_HDR_SIZE);
	if (IS_ERR(s_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for rng kernel source buffer ret: %d\n",
			ret);
		return ret;
	}

	k_ctx->rng.rng = s_buf;

	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_src, k_ctx->rng.rng,
				    k_ctx->rng.rng_len +
					    RANDOM_NUMBER_EXT_HDR_SIZE,
				    FCS_DMA_TO_DEVICE);
	if (ret) {
		LOG_ERR("Failed to map dma address for buffer. ret: %d\n", ret);
		goto free_mem;
	}

	ret = priv->plat_data->svc_send_request(priv, FCS_DEV_RANDOM_NUMBER_GEN,
						FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_RANDOM_NUMBER_GEN, ret);
		goto unmap;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, generate random number request failed ret: %d\n",
			ret);
		goto copy_mbox_status;
	}

	ret = fcs_plat_copy_to_user(ctx.rng.rng,
				    k_ctx->rng.rng + RANDOM_NUMBER_EXT_HDR_SIZE,
				    ctx.rng.rng_len);
	if (ret)
		LOG_ERR("Failed to copy random number to user ret: %d\n", ret);

copy_mbox_status:
	ret = fcs_plat_copy_to_user(k_ctx->error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mail box status code to user ret: %d\n",
			ret);
	}
	priv->plat_data->svc_task_done(priv);
unmap:
	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_src, k_ctx->rng.rng_len,
				FCS_DMA_TO_DEVICE);
free_mem:
	priv->plat_data->svc_free_memory(priv, s_buf);

	return ret;
}
EXPORT_SYMBOL(hal_random_number);

FCS_HAL_INT hal_hkdf_request(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	FCS_HAL_VOID *s_buf = NULL, *src_ptr = NULL;
	FCS_HAL_DMA_ADDR fcs_dma_handle_src;
	struct fcs_cmd_context ctx;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	/* Compare the session UUIDs to check for a match. Here suuid is set
	 * through hal_store_context
	 */
	ret = fcs_plat_uuid_compare(&priv->uuid_id, &k_ctx->hkdf_req.suuid);
	if (!ret) {
		ret = -EINVAL;
		LOG_ERR("session UUID Mismatch ret: %d\n", ret);
		return ret;
	}

	/* Allocate memory for the shared secret kernel buffer */
	s_buf = priv->plat_data->svc_alloc_memory(priv, FCS_KDK_MAX_SZ);
	if (IS_ERR(s_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for HKDF kernel buffer ret: %d\n",
			ret);
		return ret;
	}

	src_ptr = s_buf;

	fcs_plat_memset(src_ptr, 0, HKDF_REQ_SZ_MAX);

	ret = fcs_plat_copy_from_user(src_ptr, &ctx.hkdf_req.ikm_len,
				      sizeof(ctx.hkdf_req.ikm_len));
	if (ret) {
		LOG_ERR("Failed to copy HKDF salt length from user to kernel buffer ret: %d\n",
			ret);
		goto free_mem;
	}
	src_ptr += sizeof(ctx.hkdf_req.ikm_len);

	ret = fcs_plat_copy_from_user(src_ptr, ctx.hkdf_req.ikm,
				      ctx.hkdf_req.ikm_len);
	if (ret) {
		LOG_ERR("Failed to copy HKDF info from user to kernel buffer ret: %d\n",
			ret);
		goto free_mem;
	}
	src_ptr += HKDF_INPUT_DATA_SIZE;

	ret = fcs_plat_copy_from_user(src_ptr, &ctx.hkdf_req.info_len,
				      sizeof(ctx.hkdf_req.info_len));
	if (ret) {
		LOG_ERR("Failed to copy HKDF salt from user to kernel buffer ret: %d\n",
			ret);
		goto free_mem;
	}
	src_ptr += sizeof(ctx.hkdf_req.info_len);

	ret = fcs_plat_copy_from_user(src_ptr, ctx.hkdf_req.info,
					ctx.hkdf_req.info_len);
	if (ret) {
		LOG_ERR("Failed to copy HKDF salt from user to kernel buffer ret: %d\n",
			ret);
		goto free_mem;
	}
	src_ptr += HKDF_INPUT_DATA_SIZE;

	ret = fcs_plat_copy_from_user(src_ptr,
				      ctx.hkdf_req.output_key_obj,
				      ctx.hkdf_req.output_key_obj_len);
	if (ret) {
		LOG_ERR("Failed to copy HKDF output key obj from user ret: %d\n",
			ret);
		goto free_mem;
	}

	/* Map buffer for DMA */
	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_src, s_buf,
				    FCS_KDK_MAX_SZ, FCS_DMA_TO_DEVICE);
	if (ret) {
		LOG_ERR("Failed perform dma address map for the HKDF kernel buffer ret: %d\n",
			ret);
		goto free_mem;
	}

	k_ctx->hkdf_req.ikm = s_buf;

	ret = priv->plat_data->svc_send_request(priv,
						FCS_DEV_CRYPTO_HKDF_REQUEST,
						FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_CRYPTO_HKDF_REQUEST, ret);
		goto unmap;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to perform HKDF ret: %d\n", ret);
	}

	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
	}

	ret = fcs_plat_copy_to_user(ctx.hkdf_req.hkdf_resp, &priv->resp,
				    sizeof(priv->resp));
	if (ret)
		LOG_ERR("Failed to copy HKDF status to user ret: %d\n", ret);

	priv->plat_data->svc_task_done(priv);
unmap:
	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_src, FCS_KDK_MAX_SZ,
				FCS_DMA_TO_DEVICE);
free_mem:
	priv->plat_data->svc_free_memory(priv, s_buf);

	return ret;
}
EXPORT_SYMBOL(hal_hkdf_request);

static FCS_HAL_INT hal_digest_init(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	struct fcs_cmd_context ctx;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	ret = priv->plat_data->svc_send_request(
		priv, FCS_DEV_CRYPTO_GET_DIGEST_INIT, FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_CRYPTO_GET_DIGEST_INIT, ret);
		return ret;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to initialize digest ret: %d\n",
			ret);
	}

	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
	}
	priv->plat_data->svc_task_done(priv);
	return ret;
}

static FCS_HAL_INT hal_digest_update(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	FCS_HAL_VOID *d_buf = NULL;
	FCS_HAL_DMA_ADDR fcs_dma_handle_src, fcs_dma_handle_dst;
	struct fcs_cmd_context ctx;
	FCS_HAL_U32 ldigest_len = DIGEST_CMD_MAX_SZ;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	k_ctx->dgst.digest_len = &ldigest_len;

	/* Map the input data kernel buffer for DMA */
	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_src, k_ctx->dgst.src,
				    DIGEST_CMD_MAX_SZ, FCS_DMA_TO_DEVICE);
	if (ret) {
		LOG_ERR("Failed to perform dma address map for the input buffer. ret: %d\n",
			ret);
		return ret;
	}

	d_buf = priv->plat_data->svc_alloc_memory(priv, DIGEST_CMD_MAX_SZ);
	if (IS_ERR(d_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for digest output kernel buffer. ret: %d\n",
			ret);
		goto unmap_src;
	}
	k_ctx->dgst.digest = d_buf;

	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_dst, d_buf,
				    DIGEST_CMD_MAX_SZ, FCS_DMA_FROM_DEVICE);
	if (ret) {
		LOG_ERR("Failed perform dma address map for the digest output data kernel buffer ret: %d\n",
			ret);
		goto free_dest;
	}

	/* Send the request to perform digest */
	ret = priv->plat_data->svc_send_request(
		priv, FCS_DEV_CRYPTO_GET_DIGEST_UPDATE,
		10 * FCS_REQUEST_TIMEOUT);

	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst, DIGEST_CMD_MAX_SZ,
				FCS_DMA_FROM_DEVICE);

	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_CRYPTO_GET_DIGEST_UPDATE, ret);
		goto free_dest;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to perform digest ret: %d\n",
			ret);
		goto copy_mbox_status;
	}

copy_mbox_status:
	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
	}
	priv->plat_data->svc_task_done(priv);

free_dest:
	priv->plat_data->svc_free_memory(priv, d_buf);
unmap_src:
	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_src, DIGEST_CMD_MAX_SZ,
				FCS_DMA_TO_DEVICE);
	return ret;
}

static FCS_HAL_INT hal_digest_final(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	FCS_HAL_VOID *d_buf = NULL;
	FCS_HAL_DMA_ADDR fcs_dma_handle_src, fcs_dma_handle_dst;
	struct fcs_cmd_context ctx;
	FCS_HAL_U32 ldigest_len = DIGEST_CMD_MAX_SZ;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	/* Map the input data kernel buffer for DMA */
	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_src, k_ctx->dgst.src,
				    k_ctx->dgst.src_len, FCS_DMA_TO_DEVICE);
	if (ret) {
		LOG_ERR("Failed perform dma address map for the digest input data kernel buffer ret: %d\n",
			ret);
		goto return_fun;
	}

	d_buf = priv->plat_data->svc_alloc_memory(priv, DIGEST_CMD_MAX_SZ);
	if (IS_ERR(d_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for digest output kernel buffer ret: %d\n",
			ret);
		goto return_fun;
	}

	/* Map the digest output kernel buffer for DMA */
	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_dst, d_buf,
				    DIGEST_CMD_MAX_SZ, FCS_DMA_FROM_DEVICE);
	if (ret) {
		LOG_ERR("Failed perform dma address map for the digest output data kernel buffer ret: %d\n",
			ret);
		goto unmap_src;
	}

	k_ctx->dgst.digest = d_buf;
	k_ctx->dgst.digest_len = &ldigest_len;

	/* Send the request to finalize digest */
	ret = priv->plat_data->svc_send_request(priv,
						FCS_DEV_CRYPTO_GET_DIGEST_FINAL,
						10 * FCS_REQUEST_TIMEOUT);

	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst, CRYPTO_DIGEST_MAX_SZ,
				FCS_DMA_FROM_DEVICE);

	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_CRYPTO_GET_DIGEST_FINAL, ret);
		goto unmap_src;
	}
	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to finalize digest ret: %d\n",
			ret);
		goto copy_mbox_status;
	}

	priv->resp -= RESPONSE_HEADER_SIZE;

	/* Copy the digest output from kernel space to user space */
	ret = fcs_plat_copy_to_user(ctx.dgst.digest,
				    k_ctx->dgst.digest + RESPONSE_HEADER_SIZE,
				    priv->resp);
	if (ret) {
		LOG_ERR("Failed to copy digest output to user ret: %d\n", ret);
		goto copy_mbox_status;
	}

	/* Copy the digest output length from kernel space to user space */
	ret = fcs_plat_copy_to_user(ctx.dgst.digest_len, &priv->resp,
				    sizeof(priv->resp));
	if (ret) {
		LOG_ERR("Failed to copy digest output length to user ret: %d\n",
			ret);
	}

copy_mbox_status:
	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
	}

unmap_src:
	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_src, k_ctx->dgst.src_len,
				FCS_DMA_TO_DEVICE);

return_fun:
	priv->plat_data->svc_free_memory(priv, k_ctx->dgst.src);
	priv->plat_data->svc_free_memory(priv, d_buf);
	priv->plat_data->svc_task_done(priv);

	return ret;
}

FCS_HAL_INT hal_digest(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	struct fcs_cmd_context ctx;
	FCS_HAL_VOID *s_buf = NULL;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	if (k_ctx->dgst.stage == FCS_DIGEST_STAGE_INIT) {
		/* Compare the session UUIDs to check for a match */
		ret = fcs_plat_uuid_compare(&priv->uuid_id, &k_ctx->dgst.suuid);
		if (!ret) {
			ret = -EINVAL;
			LOG_ERR("session UUID Mismatch ret: %d\n", ret);
			return ret;
		}
		ret = hal_digest_init(k_ctx);
		if (ret) {
			LOG_ERR("Failed to initialize get digest command ret: %d\n",
				ret);
			return ret;
		}

		s_buf = priv->plat_data->svc_alloc_memory(priv,
							  DIGEST_CMD_MAX_SZ);
		if (IS_ERR(s_buf)) {
			ret = -ENOMEM;
			LOG_ERR("Failed to allocate memory for digest input data kernel buffer ret: %d\n",
				ret);
			return ret;
		}

		k_ctx->dgst.src = s_buf;
		return ret;
	}

	if (k_ctx->dgst.stage == FCS_DIGEST_STAGE_UPDATE) {
		ret = hal_digest_update(k_ctx);
		if (ret) {
			LOG_ERR("Failed to update get digest command ret: %d\n",
				ret);
		}
		return ret;
	}

	if (k_ctx->dgst.stage == FCS_DIGEST_STAGE_FINAL) {
		ret = hal_digest_final(k_ctx);
		if (ret) {
			LOG_ERR("Failed to finalize get digest command ret: %d\n",
				ret);
			return ret;
		}
	}

	return ret;
}
EXPORT_SYMBOL(hal_digest);

FCS_HAL_INT hal_mac_verify(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	FCS_HAL_VOID *s_buf = NULL, *d_buf = NULL;
	FCS_HAL_DMA_ADDR fcs_dma_handle_src, fcs_dma_handle_dst;
	struct fcs_cmd_context ctx;
	FCS_HAL_VOID *input_buffer;
	FCS_HAL_U32 remaining_size;
	FCS_HAL_U32 sign_size;
	FCS_HAL_U32 data_size;
	FCS_HAL_U32 ud_sz, out_sz = 32;
	FCS_HAL_U32 update_stage;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	/* Compare the session UUIDs to check for a match. Here suuid is set
	 * through hal_store_context
	 */
	ret = fcs_plat_uuid_compare(&priv->uuid_id, &k_ctx->rng.suuid);
	if (!ret) {
		ret = -EINVAL;
		LOG_ERR("session UUID Mismatch ret: %d\n", ret);
		return ret;
	}

	ret = priv->plat_data->svc_send_request(
		priv, FCS_DEV_CRYPTO_MAC_VERIFY_INIT, FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_CRYPTO_MAC_VERIFY_INIT, ret);
		return ret;
	}
	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to initialize digest ret: %d\n",
			ret);
		return ret;
	}

	input_buffer = k_ctx->mac_verify.src;
	remaining_size = k_ctx->mac_verify.src_size;
	sign_size =
		k_ctx->mac_verify.src_size - k_ctx->mac_verify.user_data_size;

	/* Allocate memory for the input data kernel buffer */
	s_buf = priv->plat_data->svc_alloc_memory(priv, MAC_CMD_MAX_SZ);
	if (IS_ERR(s_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for mac input data kernel buffer ret: %d\n",
			ret);
		return ret;
	}

	d_buf = priv->plat_data->svc_alloc_memory(priv, MAC_CMD_MAX_SZ);
	if (IS_ERR(d_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for mac output kernel buffer ret: %d\n",
			ret);
		goto free_s_buf;
	}

	while (remaining_size > 0) {
		if (remaining_size > MAC_CMD_MAX_SZ) {
			/* Finalize stage require minimum 8bytes data size */
			if ((remaining_size - MAC_CMD_MAX_SZ) >=
			    (DIGEST_SERVICE_MIN_DATA_SIZE + sign_size)) {
				data_size = CRYPTO_DIGEST_MAX_SZ;
				ud_sz = CRYPTO_DIGEST_MAX_SZ;
				LOG_DBG("Update full. data_size=%d, ud_sz=%d\n",
					data_size, ud_sz);
			} else {
				/* Partial stage */
				data_size = remaining_size -
					    DIGEST_SERVICE_MIN_DATA_SIZE -
					    sign_size;
				ud_sz = remaining_size -
					DIGEST_SERVICE_MIN_DATA_SIZE -
					sign_size;
				LOG_DBG("Update partial. data_size=%d, ud_sz=%d\n",
					data_size, ud_sz);
			}
			update_stage = 1;
		} else {
			data_size = remaining_size;
			ud_sz = remaining_size - sign_size;
			LOG_ERR("Finalize. data_size=%d, ud_sz=%d\n", data_size,
				ud_sz);
			update_stage = 0;
		}

		/* Copy the user space input data to the input data kernel buffer */
		ret = fcs_plat_copy_from_user(s_buf, input_buffer, data_size);
		if (ret) {
			LOG_ERR("Failed to copy input data from user to kernel buffer ret: %d\n",
				ret);
			goto free_dest;
		}

		ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_src, s_buf,
					    MAC_CMD_MAX_SZ, FCS_DMA_TO_DEVICE);
		if (ret) {
			LOG_ERR("Failed to perform dma address for the counter set kernel buffer ret: %d\n",
				ret);
			goto free_dest;
		}

		ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_dst, d_buf,
					    MAC_CMD_MAX_SZ,
					    FCS_DMA_FROM_DEVICE);
		if (ret) {
			fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_src,
						MAC_CMD_MAX_SZ,
						FCS_DMA_TO_DEVICE);
			goto free_dest;
		}

		k_ctx->mac_verify.src = s_buf;
		k_ctx->mac_verify.src_size = data_size;
		k_ctx->mac_verify.dst = d_buf;
		k_ctx->mac_verify.dst_size = &out_sz;
		k_ctx->mac_verify.user_data_size = ud_sz;

		if (update_stage == 1) {
			ret = priv->plat_data->svc_send_request(
				priv, FCS_DEV_CRYPTO_MAC_VERIFY_UPDATE,
				100 * FCS_REQUEST_TIMEOUT);

			fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst,
						MAC_CMD_MAX_SZ,
						FCS_DMA_FROM_DEVICE);
			fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_src,
						MAC_CMD_MAX_SZ,
						FCS_DMA_TO_DEVICE);

			if (ret) {
				LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
					FCS_DEV_CRYPTO_MAC_VERIFY_UPDATE, ret);
				goto free_dest;
			}
			update_stage = 0;
		} else {
			/* Finalize stage */
			ret = priv->plat_data->svc_send_request(
				priv, FCS_DEV_CRYPTO_MAC_VERIFY_FINAL,
				100 * FCS_REQUEST_TIMEOUT);

			fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst,
						MAC_CMD_MAX_SZ,
						FCS_DMA_FROM_DEVICE);
			fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_src,
						MAC_CMD_MAX_SZ,
						FCS_DMA_TO_DEVICE);
			if (ret) {
				LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
					FCS_DEV_CRYPTO_MAC_VERIFY_FINAL, ret);
				goto free_dest;
			}
		}

		if (priv->status) {
			ret = -EIO;
			LOG_ERR("Mailbox error, Failed to Update digest verify ret: %d\n",
				ret);
			goto copy_mbox_status;
		}

		remaining_size -= data_size;
		if (remaining_size == 0) {
			priv->resp -= RESPONSE_HEADER_SIZE;

			ret = fcs_plat_copy_to_user(
				ctx.mac_verify.dst,
				d_buf + RESPONSE_HEADER_SIZE, priv->resp);
			if (ret) {
				LOG_ERR("Failed to copy MAC verify data to user ret: %d\n",
					ret);
				goto free_dest;
			}
			ret = fcs_plat_copy_to_user(ctx.mac_verify.dst_size,
						    &priv->resp,
						    sizeof(priv->resp));
			if (ret) {
				LOG_ERR("Failed to copy MAC verify data size to user ret: %d\n",
					ret);
				goto free_dest;
			}
		} else {
			input_buffer += data_size;
		}
	}

copy_mbox_status:
	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
	}

	priv->plat_data->svc_task_done(priv);

free_dest:
	priv->plat_data->svc_free_memory(priv, d_buf);
free_s_buf:
	priv->plat_data->svc_free_memory(priv, s_buf);

	return ret;
}
EXPORT_SYMBOL(hal_mac_verify);

static FCS_HAL_INT hal_aes_crypt_init(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	FCS_HAL_CHAR *aes_parms = NULL;
	FCS_HAL_UINT aes_parms_len = FCS_AES_PARAMS_ECB_SZ + FCS_AES_IV_SZ;
	FCS_HAL_DMA_ADDR fcs_dma_handle_aesparms;

	aes_parms = priv->plat_data->svc_alloc_memory(priv, aes_parms_len);
	if (IS_ERR(aes_parms)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for AES parameters ret: %d\n",
			ret);
		return ret;
	}

	fcs_plat_memset(aes_parms, 0, aes_parms_len);
	fcs_plat_memcpy(aes_parms, &k_ctx->aes.mode, 1);
	fcs_plat_memcpy(aes_parms + AES_PARAMS_CRYPT_OFFSET, &k_ctx->aes.crypt, 1);
	fcs_plat_memcpy(aes_parms + AES_PARAMS_TAG_LEN_OFFSET, &k_ctx->aes.tag_len, 2);
	fcs_plat_memcpy(aes_parms + AES_PARAMS_IV_TYPE_OFFSET, &k_ctx->aes.iv_source, 1);
	fcs_plat_memcpy(aes_parms + AES_PARAMS_AAD_LEN_OFFSET, &k_ctx->aes.aad_len, 4);

	LOG_DBG("AES init: mode: %d, ENC/DEC: %d, tag_len: %d iv_src: %d aad_len: %d\n",
		k_ctx->aes.mode, k_ctx->aes.crypt, k_ctx->aes.tag_len,
		k_ctx->aes.iv_source, k_ctx->aes.aad_len);

	k_ctx->aes.ip_len = FCS_AES_PARAMS_ECB_SZ;
	if (k_ctx->aes.mode != FCS_AES_BLOCK_MODE_ECB) {
		ret = fcs_plat_copy_from_user(aes_parms + FCS_AES_PARAMS_ECB_SZ,
					      k_ctx->aes.iv, FCS_AES_IV_SZ);
		if (ret) {
			LOG_ERR("Failed to copy iv from user to kernel buffer ret: %d\n",
				ret);
			goto free_mem;
		}
		k_ctx->aes.ip_len += FCS_AES_IV_SZ;
	}
	k_ctx->aes.input = aes_parms;

	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_aesparms, aes_parms,
				    aes_parms_len, FCS_DMA_TO_DEVICE);
	if (ret) {
		LOG_ERR("Failed perform dma address map for the AES poll service buffer ret: %d\n",
			ret);
		goto free_mem;
	}

	ret = priv->plat_data->svc_send_request(
		priv, FCS_DEV_CRYPTO_AES_CRYPT_INIT, FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_CRYPTO_AES_CRYPT_INIT, ret);
		goto unmap;
	}

	priv->plat_data->svc_task_done(priv);
unmap:
	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_aesparms, aes_parms_len,
				FCS_DMA_TO_DEVICE);
free_mem:
	priv->plat_data->svc_free_memory(priv, aes_parms);

	return ret;
}

static FCS_HAL_INT hal_aes_crypt_update_final(FCS_HAL_CHAR *ip_ptr, FCS_HAL_UINT src_len,
					      FCS_HAL_CHAR *aad, FCS_HAL_UINT aad_size,
					      FCS_HAL_CHAR *tag, FCS_HAL_UINT src_tag_len,
					      FCS_HAL_UINT dst_tag_len, FCS_HAL_CHAR *op_ptr,
					      FCS_HAL_UINT mode,
					      struct fcs_cmd_context *const k_ctx,
					      FCS_HAL_INT command)
{
	FCS_HAL_INT ret = 0, pad1 = 0, pad2 = 0, s_buf_size = 0, d_buf_size = 0;
	FCS_HAL_CHAR *s_buf = NULL, *s_buf_wr_ptr = NULL, *d_buf = NULL;
	FCS_HAL_DMA_ADDR fcs_dma_handle_src, fcs_dma_handle_dst;

	if (mode == FCS_AES_BLOCK_MODE_GCM ||
	    mode == FCS_AES_BLOCK_MODE_GHASH) {
		pad1 = (aad_size % GCM_AAD_ALIGN) ?
			(GCM_AAD_ALIGN - (aad_size % GCM_AAD_ALIGN)) : 0;
		pad2 = (src_len % GCM_DATA_ALIGN) ?
			(GCM_DATA_ALIGN - (src_len % GCM_DATA_ALIGN)) : 0;

		s_buf_size = aad_size + pad1 + src_len + pad2 + src_tag_len;
		d_buf_size = src_len + pad2 + dst_tag_len;

		if (s_buf_size > FCS_AES_CRYPT_BLOCK_SZ ||
		    d_buf_size > FCS_AES_CRYPT_BLOCK_SZ) {
			LOG_ERR("Invalid size request. Maximum buffer size supported is %d bytes\n",
				FCS_AES_CRYPT_BLOCK_SZ);
			return -EINVAL;
		}

		LOG_DBG("AES GCM: aadlen:%d, pad1:%d, srcln:%d, pad2:%d, srctag:%d, dsttag:%d\n",
			aad_size, pad1, src_len, pad2, src_tag_len,
			dst_tag_len);
	} else {
		pad2 = (src_len % NON_GCM_DATA_ALIGN) ?
			       (NON_GCM_DATA_ALIGN - (aad_size % NON_GCM_DATA_ALIGN)) :
			       0;
		s_buf_size = src_len + pad2;
		d_buf_size = src_len + pad2;

		if (s_buf_size > FCS_AES_CRYPT_BLOCK_SZ) {
			LOG_ERR("Invalid size request. Maximum buffer size supported is %d bytes\n",
				FCS_AES_CRYPT_BLOCK_SZ);
			return -EINVAL;
		}
		aad_size = 0;
		src_tag_len = 0;
		dst_tag_len = 0;
	}

	s_buf = priv->plat_data->svc_alloc_memory(priv, FCS_AES_CRYPT_BLOCK_SZ);
	if (IS_ERR(s_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for AES source buffer ret: %d\n", ret);
		return ret;
	}
	s_buf_wr_ptr = s_buf;

	LOG_DBG("AES Update/final s_buf = %p, s_buf_size = %d\n", s_buf, s_buf_size);

	if (aad_size) {
		if (aad) {
			LOG_ERR("AES Update/final copy AAD at %p aad_size = %d\n",
				s_buf, aad_size);

			ret = fcs_plat_copy_from_user(s_buf, aad, aad_size);
			if (ret) {
				LOG_ERR("Failed to copy AAD data to svc buffer ret: %d\n",
					ret);
				goto free_src;
			}

			fcs_plat_memset(s_buf + aad_size, 0, pad1);
		} else {
			LOG_ERR("Invalid AAD data buffer address %d\n", ret);
			ret = -EINVAL;
			goto free_src;
		}

		aad_size += pad1;
		s_buf_wr_ptr = s_buf_wr_ptr + aad_size;
	}

	LOG_DBG("AES Update/final copy Data at %p  data_size = %d\n",
		s_buf_wr_ptr, src_len);

	ret = fcs_plat_copy_from_user(s_buf_wr_ptr, ip_ptr, src_len);
	if (ret) {
		LOG_ERR("Failed to copy AES data to svc buffer ret: %d\n", ret);
		goto free_src;
	}

	s_buf_wr_ptr = s_buf_wr_ptr + src_len;
	fcs_plat_memset(s_buf_wr_ptr, 0, pad2);
	s_buf_wr_ptr = s_buf_wr_ptr + pad2;

	if (src_tag_len) {
		if (tag) {
			LOG_DBG("AES Update/final Tag value at %p  tag_size = %d\n",
				s_buf_wr_ptr, src_tag_len);
			ret = fcs_plat_copy_from_user(s_buf_wr_ptr, tag,
						      src_tag_len);
			if (ret) {
				LOG_ERR("Failed to copy Tag data to svc buffer ret: %d\n",
					ret);
				goto free_src;
			}
		} else {
			LOG_ERR("Invalid TAG data buffer address %d\n", ret);
			ret = -EINVAL;
			goto free_src;
		}
	}

	d_buf = priv->plat_data->svc_alloc_memory(priv, FCS_AES_CRYPT_BLOCK_SZ);
	if (IS_ERR(d_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for AES destination buffer ret: %d\n",
			ret);
		goto free_src;
	}

	/* Map the source buffer for DMA */
	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_src, s_buf,
				    FCS_AES_CRYPT_BLOCK_SZ, FCS_DMA_TO_DEVICE);
	if (ret) {
		LOG_ERR("Failed perform dma address map for the AES source buffer ret: %d\n",
			ret);
		goto free_dst;
	}

	/* Map the destination buffer for DMA */
	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_dst, d_buf,
				    FCS_AES_CRYPT_BLOCK_SZ,
				    FCS_DMA_FROM_DEVICE);
	if (ret) {
		LOG_ERR("Failed perform dma address map for the AES destination buffer ret: %d\n",
			ret);
		fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_src,
					FCS_AES_CRYPT_BLOCK_SZ, FCS_DMA_TO_DEVICE);
		goto free_dst;
	}

	k_ctx->aes.ip_len = s_buf_size;
	*k_ctx->aes.op_len = d_buf_size;
	k_ctx->aes.input = s_buf;
	k_ctx->aes.output = d_buf;
	k_ctx->aes.input_pad = pad2;

	/* Send the AES crypt request */
	ret = priv->plat_data->svc_send_request(priv, command,
				FCS_AES_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n", command, ret);
		fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst,
					FCS_AES_CRYPT_BLOCK_SZ, FCS_DMA_FROM_DEVICE);
		fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_src,
					FCS_AES_CRYPT_BLOCK_SZ, FCS_DMA_TO_DEVICE);
		goto free_dst;
	}

	/* Copy the mailbox status code to the user */
	ret = fcs_plat_copy_to_user(k_ctx->error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
		fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst,
					FCS_AES_CRYPT_BLOCK_SZ, FCS_DMA_FROM_DEVICE);
		fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_src,
					FCS_AES_CRYPT_BLOCK_SZ, FCS_DMA_TO_DEVICE);
		goto task_done;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to perform AES crypt Mbox status: 0x%x\n",
			priv->status);
		fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst,
					FCS_AES_CRYPT_BLOCK_SZ, FCS_DMA_FROM_DEVICE);
		fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_src,
					FCS_AES_CRYPT_BLOCK_SZ, FCS_DMA_TO_DEVICE);

		goto task_done;
	}

	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst,
				FCS_AES_CRYPT_BLOCK_SZ, FCS_DMA_FROM_DEVICE);
	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_src,
				FCS_AES_CRYPT_BLOCK_SZ, FCS_DMA_TO_DEVICE);

	if (mode != FCS_AES_BLOCK_MODE_GHASH) {
		LOG_DBG("AES copy Data to destination buffer %p  data_size = %d\n",
			op_ptr, src_len + pad2);
		/* Copy the destination buffer to the user space */
		ret = fcs_plat_copy_to_user(op_ptr, d_buf, src_len + pad2);
		if (ret)
			LOG_ERR("Failed to copy AES data from kernel to user buffer ret: %d\n",
				ret);
	}

	if (dst_tag_len) {
		if (tag) {
			LOG_DBG("AES copy tag value to Tag buffer %p  data_size = %d\n",
				tag, dst_tag_len);
			ret = fcs_plat_copy_to_user(tag, d_buf + src_len + pad2,
						    dst_tag_len);
			if (ret) {
				LOG_ERR("Failed to copy TAG value to tag buffer ret: %d\n",
					ret);
				goto task_done;
			}
		} else {
			LOG_ERR("Invalid TAG data buffer address %d\n", ret);
			goto task_done;
		}
	}

	LOG_DBG("AES Update/final Success\n");

task_done:
	priv->plat_data->svc_task_done(priv);
free_dst:
	priv->plat_data->svc_free_memory(priv, d_buf);
free_src:
	priv->plat_data->svc_free_memory(priv, s_buf);
	return ret;
}

FCS_HAL_INT hal_aes_crypt(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	struct fcs_cmd_context ctx;
	FCS_HAL_CHAR *ip_ptr = NULL, *op_ptr = NULL;
	FCS_HAL_UINT ip_len = 0, op_len = 0, src_len = 0;
	FCS_HAL_UINT total_op_len = 0;
	FCS_HAL_UINT pad1 = 0, aad_size = 0, src_tag_len = 0, dst_tag_len = 0;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	/* Compare the session UUIDs to check for a match. Here suuid is set
	 * through hal_store_context
	 */
	ret = fcs_plat_uuid_compare(&priv->uuid_id, &ctx.aes.suuid);
	if (!ret) {
		ret = -EINVAL;
		LOG_ERR("session UUID Mismatch ret: %d\n", ret);
		return ret;
	}

	/* Initialize the AES crypt */
	ret = hal_aes_crypt_init(k_ctx);
	if (ret) {
		LOG_ERR("Failed to perform AES crypt init ret: %d\n", ret);
		return ret;
	}

	/* Calculate AAD data padding length. AAD data shall be 16 bytes aligned.
	 * Applicable for only GCM for other modes aad_len will be 0 hence pad1 will be 0
	 */
	aad_size = k_ctx->aes.aad_len;
	pad1 = (aad_size % GCM_AAD_ALIGN) ?
		       GCM_AAD_ALIGN - (k_ctx->aes.aad_len % GCM_AAD_ALIGN) : 0;

	ip_len = (ctx.aes.ip_len + aad_size + pad1);
	k_ctx->aes.op_len = &op_len;
	ip_ptr = ctx.aes.input;
	op_ptr = ctx.aes.output;
	k_ctx->aes.input_pad = 0;

	while (ip_len > FCS_AES_CRYPT_BLOCK_SZ) {
		src_len = FCS_AES_CRYPT_BLOCK_SZ - (aad_size + pad1);

		ret = hal_aes_crypt_update_final(ip_ptr, src_len,
						 k_ctx->aes.aad, aad_size,
						 NULL, src_tag_len,  dst_tag_len,
						 op_ptr, k_ctx->aes.mode, k_ctx,
						 FCS_DEV_CRYPTO_AES_CRYPT_UPDATE);
		if (ret) {
			LOG_ERR("Failed to perform AES crypt update ret: %d\n",
				ret);
			return ret;
		}

		ip_ptr += src_len;
		op_ptr += src_len;
		ip_len -= (src_len + aad_size + pad1);
		total_op_len += src_len;
		aad_size = 0;
		pad1 = 0;
	}

	if (k_ctx->aes.mode == FCS_AES_BLOCK_MODE_GCM ||
	    k_ctx->aes.mode == FCS_AES_BLOCK_MODE_GHASH) {
		if (ip_len > (FCS_AES_CRYPT_BLOCK_SZ - GCM_TAG_LEN)) {
			src_len = FCS_AES_CRYPT_BLOCK_SZ - GCM_TAG_LEN -
				  (aad_size + pad1);

			ret = hal_aes_crypt_update_final(ip_ptr, src_len,
							 k_ctx->aes.aad, aad_size,
							 NULL, src_tag_len,  dst_tag_len,
							 op_ptr, k_ctx->aes.mode, k_ctx,
							 FCS_DEV_CRYPTO_AES_CRYPT_UPDATE);
			if (ret) {
				LOG_ERR("Failed to perform AES crypt update ret: %d\n",
					ret);
				return ret;
			}

			ip_ptr += src_len;
			op_ptr += src_len;
			ip_len -= (src_len + aad_size + pad1);
			total_op_len += src_len;
			aad_size = 0;
			pad1 = 0;
		}

		if (k_ctx->aes.crypt == FCS_AES_ENCRYPT) {
			src_tag_len = 0;
			dst_tag_len = GCM_TAG_LEN;
		} else {
			src_tag_len = GCM_TAG_LEN;
			dst_tag_len = 0;
		}
	}

	if (ip_len) {
		src_len = ip_len - (aad_size + pad1);

		ret = hal_aes_crypt_update_final(ip_ptr, src_len,
						 k_ctx->aes.aad, aad_size,
						 k_ctx->aes.tag, src_tag_len, dst_tag_len,
						 op_ptr, k_ctx->aes.mode, k_ctx,
						 FCS_DEV_CRYPTO_AES_CRYPT_FINAL);
		if (ret) {
			LOG_ERR("Failed to perform AES crypt update ret: %d\n",
				ret);
			return ret;
		}

		if (k_ctx->aes.mode != FCS_AES_BLOCK_MODE_GHASH)
			total_op_len += src_len;
		else
			total_op_len = 0;
	}

	/* Copy the destination buffer to the user space */
	ret = fcs_plat_copy_to_user(ctx.aes.op_len, &total_op_len,
				    sizeof(ctx.aes.op_len));
	if (ret) {
		LOG_ERR("Failed to copy AES data from kernel to user buffer ret: %d\n",
			ret);
	}

	return ret;
}
EXPORT_SYMBOL(hal_aes_crypt);

FCS_HAL_INT hal_ecdh_req(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	FCS_HAL_VOID *s_buf = NULL, *d_buf = NULL;
	FCS_HAL_DMA_ADDR fcs_dma_handle_src, fcs_dma_handle_dst;
	struct fcs_cmd_context ctx;
	FCS_HAL_UINT d_buf_len = 0;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	if ((ctx.ecdh_req.ecc_curve == FCS_ECC_CURVE_NIST_P256 &&
	     ctx.ecdh_req.pubkey_len != FCS_ECDH_P256_PUBKEY_LEN) ||
	    (ctx.ecdh_req.ecc_curve == FCS_ECC_CURVE_NIST_P384 &&
	     ctx.ecdh_req.pubkey_len != FCS_ECDH_P384_PUBKEY_LEN) ||
	    (ctx.ecdh_req.ecc_curve == FCS_ECC_CURVE_BRAINPOOL_P256 &&
	     ctx.ecdh_req.pubkey_len != FCS_ECDH_BP256_PUBKEY_LEN) ||
	    (ctx.ecdh_req.ecc_curve == FCS_ECC_CURVE_BRAINPOOL_P384 &&
	     ctx.ecdh_req.pubkey_len != FCS_ECDH_BP384_PUBKEY_LEN)) {
		ret = -EINVAL;
		LOG_ERR("Invalid shared secret length ret: %d\n", ret);
		return ret;
	}

	/* Compare the session UUIDs to check for a match. Here suuid is set
	 * through hal_store_context
	 */
	ret = fcs_plat_uuid_compare(&priv->uuid_id, &ctx.ecdh_req.suuid);
	if (!ret) {
		ret = -EINVAL;
		LOG_ERR("session UUID Mismatch ret: %d\n", ret);
		return ret;
	}

	/* Allocate memory for the source buffer */
	s_buf = priv->plat_data->svc_alloc_memory(priv,
						  ctx.ecdh_req.pubkey_len);
	if (IS_ERR(s_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for ECDH source buffer ret: %d\n",
			ret);
		return ret;
	}

	/* Copy the user space source data to the source buffer */
	ret = fcs_plat_copy_from_user(s_buf, ctx.ecdh_req.pubkey,
				      ctx.ecdh_req.pubkey_len);
	if (ret) {
		LOG_ERR("Failed to copy ECDH data from user to kernel buffer ret: %d\n",
			ret);
		goto free_src;
	}

	/* Map the source buffer for DMA */
	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_src, s_buf,
				    ctx.ecdh_req.pubkey_len, FCS_DMA_TO_DEVICE);
	if (ret) {
		LOG_ERR("Failed to map dma address for the ECDH source buffer ret: %d\n",
			ret);
		goto free_src;
	}

	/* 1 byte for format indicator + pk len bytes for X coordinate + pk len
	 * bytes for Y coordinate
	 */
	d_buf_len = ctx.ecdh_req.pubkey_len >> 1;
	/* Allocate memory for the destination buffer */
	d_buf = priv->plat_data->svc_alloc_memory(priv, d_buf_len + 12);
	if (IS_ERR(d_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for ECDH destination buffer ret: %d\n",
			ret);
		goto unmap_src;
	}

	/* Map the destination buffer for DMA */
	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_dst, d_buf, d_buf_len,
				    FCS_DMA_FROM_DEVICE);
	if (ret) {
		LOG_ERR("Failed to map dma address for the ECDH destination buffer ret: %d\n",
			ret);
		goto free_dst;
	}

	k_ctx->ecdh_req.pubkey = s_buf;
	k_ctx->ecdh_req.sh_secret = d_buf;
	k_ctx->ecdh_req.sh_secret_len = &d_buf_len;

	ret = priv->plat_data->svc_send_request(
		priv, FCS_DEV_CRYPTO_ECDH_REQUEST_INIT, FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_CRYPTO_ECDH_REQUEST_INIT, ret);
		goto unmap_dst;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to perform ECDH ret: %d\n", ret);
		goto copy_mbox_status;
	}

	ret = priv->plat_data->svc_send_request(
		priv, FCS_DEV_CRYPTO_ECDH_REQUEST_FINALIZE,
		FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_CRYPTO_ECDH_REQUEST_FINALIZE, ret);
		goto unmap_dst;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to perform ECDH ret: %d\n", ret);
		goto copy_mbox_status;
	}

	priv->resp -= RESPONSE_HEADER_SIZE;

	ret = fcs_plat_copy_to_user(ctx.ecdh_req.sh_secret,
				    d_buf + RESPONSE_HEADER_SIZE, priv->resp);
	if (ret)
		LOG_ERR("Failed to copy ECDH data to user ret: %d\n", ret);

	ret = fcs_plat_copy_to_user(ctx.ecdh_req.sh_secret_len, &priv->resp,
				    sizeof(priv->resp));
	if (ret) {
		LOG_ERR("Failed to copy ECDH data length to user ret: %d\n",
			ret);
	}
copy_mbox_status:
	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
	}
	priv->plat_data->svc_task_done(priv);

unmap_dst:
	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst, d_buf_len,
				FCS_DMA_FROM_DEVICE);
free_dst:
	priv->plat_data->svc_free_memory(priv, d_buf);
unmap_src:
	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_src,
				ctx.ecdh_req.pubkey_len, FCS_DMA_TO_DEVICE);
free_src:
	priv->plat_data->svc_free_memory(priv, s_buf);

	return ret;
}
EXPORT_SYMBOL(hal_ecdh_req);

FCS_HAL_INT hal_get_chip_id(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	struct fcs_cmd_context ctx;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	ret = priv->plat_data->svc_send_request(priv, FCS_DEV_CHIP_ID,
						FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n", FCS_DEV_CHIP_ID,
			ret);
		return ret;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, get chip ID request failed ret: %d\n",
			ret);
		goto copy_mbox_status;
	}

	ret = fcs_plat_copy_to_user(ctx.chip_id.chip_id_lo, &priv->chip_id_lo,
				    sizeof(priv->chip_id_lo));
	if (ret) {
		LOG_ERR("Failed to copy chip ID to user ret: %d\n", ret);
		goto copy_mbox_status;
	}

	ret = fcs_plat_copy_to_user(ctx.chip_id.chip_id_hi, &priv->chip_id_hi,
				    sizeof(priv->chip_id_hi));
	if (ret)
		LOG_ERR("Failed to copy chip ID to user ret: %d\n", ret);

copy_mbox_status:
	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
	}

	priv->plat_data->svc_task_done(priv);

	return ret;
}
EXPORT_SYMBOL(hal_get_chip_id);

FCS_HAL_INT hal_attestation_get_certificate(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	FCS_HAL_U32 cert_len = CERTIFICATE_RSP_MAX_SZ;
	FCS_HAL_VOID *d_buf = NULL;
	FCS_HAL_DMA_ADDR fcs_dma_handle_dst;
	struct fcs_cmd_context ctx;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	d_buf = priv->plat_data->svc_alloc_memory(priv, cert_len);
	if (IS_ERR(d_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for certificate kernel buffer ret: %d\n",
			ret);
		return ret;
	}

	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_dst, d_buf, cert_len,
				    FCS_DMA_FROM_DEVICE);
	if (ret)
		goto free_dest;

	k_ctx->attestation_cert.cert = d_buf;
	k_ctx->attestation_cert.cert_size = &cert_len;

	ret = priv->plat_data->svc_send_request(
		priv, FCS_DEV_ATTESTATION_GET_CERTIFICATE,
		10 * FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_ATTESTATION_GET_CERTIFICATE, ret);
		goto unmap;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, get attestation certificate request failed ret: %d\n",
			ret);
		goto copy_mbox_status;
	}

	ret = fcs_plat_copy_to_user(ctx.attestation_cert.cert, d_buf,
				    priv->resp);
	if (ret) {
		LOG_ERR("Failed to copy attestation certificate to user ret: %d\n",
			ret);
		goto copy_mbox_status;
	}

	ret = fcs_plat_copy_to_user(ctx.attestation_cert.cert_size, &priv->resp,
				    sizeof(priv->resp));
	if (ret) {
		LOG_ERR("Failed to copy attestation certificate length to user ret: %d\n",
			ret);
	}

copy_mbox_status:
	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
	}
	priv->plat_data->svc_task_done(priv);
unmap:
	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst,
				CERTIFICATE_RSP_MAX_SZ, FCS_DMA_FROM_DEVICE);
free_dest:
	priv->plat_data->svc_free_memory(priv, d_buf);

	return ret;
}
EXPORT_SYMBOL(hal_attestation_get_certificate);

FCS_HAL_INT
hal_attestation_certificate_reload(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	struct fcs_cmd_context ctx;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	ret = priv->plat_data->svc_send_request(
		priv, FCS_DEV_ATTESTATION_CERTIFICATE_RELOAD,
		FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_ATTESTATION_CERTIFICATE_RELOAD, ret);
		return ret;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, attestation certificate reload request failed ret: %d\n",
			ret);
	}

	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
	}

	priv->plat_data->svc_task_done(priv);

	return ret;
}
EXPORT_SYMBOL(hal_attestation_certificate_reload);

FCS_HAL_INT hal_mctp_request(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	FCS_HAL_UINT mctp_len = MCTP_MAX_LEN;
	FCS_HAL_VOID *s_buf = NULL, *d_buf = NULL;
	FCS_HAL_DMA_ADDR fcs_dma_handle_src, fcs_dma_handle_dst;
	struct fcs_cmd_context ctx;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	if (ctx.mctp.mctp_req_len > MCTP_MAX_LEN) {
		LOG_ERR("MCTP data length %d is Invalid, must be less than %d\n",
			ctx.mctp.mctp_req_len, MCTP_MAX_LEN);
		return -EINVAL;
	}

	s_buf = priv->plat_data->svc_alloc_memory(priv, ctx.mctp.mctp_req_len);
	if (IS_ERR(s_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for source buffer ret: %d\n",
			ret);
		return ret;
	}

	ret = fcs_plat_copy_from_user(s_buf, ctx.mctp.mctp_req,
				      ctx.mctp.mctp_req_len);
	if (ret) {
		LOG_ERR("Failed to copy data from user to kernel buffer ret: %d\n",
			ret);
		goto free_mem;
	}

	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_src, s_buf,
				    ctx.mctp.mctp_req_len, FCS_DMA_TO_DEVICE);
	if (ret) {
		LOG_ERR("Failed to map dma address for source buffer ret: %d\n",
			ret);
		goto free_mem;
	}

	d_buf = priv->plat_data->svc_alloc_memory(priv, mctp_len);
	if (IS_ERR(d_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for destination buffer ret: %d\n",
			ret);
		goto unmap;
	}

	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_dst, d_buf, mctp_len,
				    FCS_DMA_FROM_DEVICE);
	if (ret) {
		LOG_ERR("Failed to map dma address for destination buffer ret: %d\n",
			ret);
		goto free_dest;
	}

	k_ctx->mctp.mctp_req = s_buf;
	k_ctx->mctp.mctp_resp = d_buf;
	k_ctx->mctp.mctp_resp_len = &mctp_len;

	ret = priv->plat_data->svc_send_request(priv, FCS_DEV_MCTP_REQUEST,
						10 * FCS_REQUEST_TIMEOUT);

	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst, MCTP_MAX_LEN,
				FCS_DMA_FROM_DEVICE);

	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_MCTP_REQUEST, ret);
		goto unmap;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, MCTP request failed ret: %d\n", ret);
		goto copy_mbox_status;
	}

	ret = fcs_plat_copy_to_user(ctx.mctp.mctp_resp, d_buf, priv->resp);
	if (ret) {
		LOG_ERR("Failed to copy MCTP response to user ret: %d\n", ret);
		goto copy_mbox_status;
	}

	ret = fcs_plat_copy_to_user(ctx.mctp.mctp_resp_len, &priv->resp,
				    sizeof(priv->resp));
	if (ret) {
		LOG_ERR("Failed to copy MCTP response size to user ret: %d\n",
			ret);
	}

copy_mbox_status:
	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
	}

	priv->plat_data->svc_task_done(priv);

free_dest:
	priv->plat_data->svc_free_memory(priv, d_buf);

unmap:
	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_src,
				ctx.mctp.mctp_req_len, FCS_DMA_TO_DEVICE);

free_mem:
	priv->plat_data->svc_free_memory(priv, s_buf);

	return ret;
}
EXPORT_SYMBOL(hal_mctp_request);

FCS_HAL_INT hal_jtag_idcode(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_U32 ret;
	struct fcs_cmd_context ctx;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	ret = priv->plat_data->svc_send_request(priv, FCS_DEV_GET_IDCODE,
						FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_GET_IDCODE, ret);
		return ret;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to get JTAG IDCODE ret: %d\n",
			ret);
		goto copy_mbox_status;
	}

	ret = fcs_plat_copy_to_user(ctx.jtag_id.jtag_idcode, &priv->resp,
				    sizeof(priv->resp));
	if (ret)
		LOG_ERR("Failed to copy JTAG IDCODE to user ret: %d\n", ret);

copy_mbox_status:
	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
	}

	priv->plat_data->svc_task_done(priv);

	return ret;
}
EXPORT_SYMBOL(hal_jtag_idcode);

FCS_HAL_INT hal_get_device_identity(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_U32 ret;
	struct fcs_cmd_context ctx;
	FCS_HAL_VOID *d_buf = NULL;
	FCS_HAL_DMA_ADDR fcs_dma_handle_dst;
	FCS_HAL_U32 devid_len = DEVICE_IDENTITY_MAX_LEN;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	d_buf = priv->plat_data->svc_alloc_memory(priv, devid_len);
	if (IS_ERR(d_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for Device Identity kernel buffer ret: %d\n",
			ret);
		return ret;
	}

	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_dst, d_buf, devid_len,
				    FCS_DMA_FROM_DEVICE);
	if (ret)
		goto free_dest;

	k_ctx->device_identity.identity = d_buf;
	k_ctx->device_identity.identity_len = &devid_len;

	ret = priv->plat_data->svc_send_request(
		priv, FCS_DEV_GET_DEVICE_IDENTITY, FCS_REQUEST_TIMEOUT);

	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst,
				DEVICE_IDENTITY_MAX_LEN, FCS_DMA_FROM_DEVICE);

	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_GET_DEVICE_IDENTITY, ret);
		goto free_dest;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to get Device Identity ret: %d\n",
			ret);
		goto copy_mbox_status;
	}

	ret = fcs_plat_copy_to_user(ctx.device_identity.identity, d_buf,
				    priv->resp);
	if (ret) {
		LOG_ERR("Failed to copy Device Identity to user ret: %d\n",
			ret);
		goto copy_mbox_status;
	}

	ret = fcs_plat_copy_to_user(ctx.device_identity.identity_len,
				    &priv->resp, sizeof(priv->resp));
	if (ret) {
		LOG_ERR("Failed to copy Device Identity length to user ret: %d\n",
			ret);
	}

copy_mbox_status:
	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
	}

	priv->plat_data->svc_task_done(priv);

free_dest:
	priv->plat_data->svc_free_memory(priv, d_buf);

	return 0;
}
EXPORT_SYMBOL(hal_get_device_identity);

FCS_HAL_INT hal_qspi_open(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	struct fcs_cmd_context ctx;

	if (k_ctx)
		fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	ret = priv->plat_data->svc_send_request(priv, FCS_DEV_QSPI_OPEN,
						FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n", FCS_DEV_QSPI_OPEN,
			ret);
		return ret;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to open QSPI ret: %d\n", ret);
	}

	if (k_ctx) {
		ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
					    sizeof(priv->status));
		if (ret) {
			LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
				ret);
		}
	}

	priv->plat_data->svc_task_done(priv);

	return 0;
}
EXPORT_SYMBOL(hal_qspi_open);

FCS_HAL_INT hal_qspi_close(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	struct fcs_cmd_context ctx;

	if (k_ctx)
		fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	ret = priv->plat_data->svc_send_request(priv, FCS_DEV_QSPI_CLOSE,
						FCS_REQUEST_TIMEOUT);

	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_QSPI_CLOSE, ret);
		return ret;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to close QSPI ret: %d\n", ret);
	}

	if (k_ctx) {
		ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
					    sizeof(priv->status));
		if (ret) {
			LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
				ret);
		}
	}

	return 0;
}
EXPORT_SYMBOL(hal_qspi_close);

FCS_HAL_INT hal_qspi_cs(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	struct fcs_cmd_context ctx;

	if (k_ctx)
		fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	ret = priv->plat_data->svc_send_request(priv, FCS_DEV_QSPI_CS,
						FCS_REQUEST_TIMEOUT);

	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n", FCS_DEV_QSPI_CS,
			ret);
		return ret;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to perform QSPI CS ret: %d\n",
			ret);
	}

	if (k_ctx) {
		ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
					    sizeof(priv->status));
	}

	return 0;
}
EXPORT_SYMBOL(hal_qspi_cs);

FCS_HAL_INT hal_qspi_read(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	struct fcs_cmd_context ctx;
	FCS_HAL_VOID *d_buf = NULL;
	FCS_HAL_U32 resp_len = QSPI_READ_LEN_MAX;
	FCS_HAL_DMA_ADDR fcs_dma_handle_dst;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	d_buf = priv->plat_data->svc_alloc_memory(priv, resp_len);
	if (IS_ERR(d_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for QSPI read kernel buffer ret: %d\n",
			ret);
		return ret;
	}

	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_dst, d_buf, resp_len,
				    FCS_DMA_FROM_DEVICE);
	if (ret)
		goto free_dest;

	k_ctx->qspi_read.qspi_data = d_buf;
	k_ctx->qspi_read.qspi_data_len = &resp_len;

	ret = priv->plat_data->svc_send_request(priv, FCS_DEV_QSPI_READ,
						FCS_REQUEST_TIMEOUT);

	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst, QSPI_READ_LEN_MAX,
				FCS_DMA_FROM_DEVICE);

	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n", FCS_DEV_QSPI_READ,
			ret);
		goto free_dest;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to read QSPI ret: %d\n", ret);
		goto copy_mbox_status;
	}

	/* requested size and response is not matching */
	if (ctx.qspi_read.qspi_len != priv->resp / 4) {
		LOG_ERR("QSPI read req and resp size is not matching resp_size:0x%x\n",
			priv->resp);
		ret = -EFAULT;
		goto copy_mbox_status;
	}

	ret = fcs_plat_copy_to_user(ctx.qspi_read.qspi_data, d_buf, priv->resp);
	if (ret) {
		LOG_ERR("Failed to copy QSPI read data to user ret: %d\n", ret);
		goto copy_mbox_status;
	}

copy_mbox_status:
	if (fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				  sizeof(priv->status))) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
	}

	priv->plat_data->svc_task_done(priv);

free_dest:
	priv->plat_data->svc_free_memory(priv, d_buf);

	return 0;
}
EXPORT_SYMBOL(hal_qspi_read);

FCS_HAL_INT hal_qspi_write(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	struct fcs_cmd_context ctx;
	FCS_HAL_VOID *s_buf = NULL;
	FCS_HAL_DMA_ADDR fcs_dma_handle_src;
	FCS_HAL_U32 s_buf_sz = 0;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	/* s_buf_sz--> (number of words * 4) + 4 bytes for qspi addr + 4 bytes qspi write len */
	s_buf_sz = ctx.qspi_write.qspi_len * WORDS_TO_BYTES_SIZE + 8;

	s_buf = priv->plat_data->svc_alloc_memory(priv, s_buf_sz);
	if (IS_ERR(s_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for QSPI write kernel buffer ret: %d\n",
			ret);
		return ret;
	}

	ret = fcs_plat_copy_from_user(s_buf, &ctx.qspi_write.qspi_addr, 4);
	if (ret) {
		LOG_ERR("Failed to copy QSPI write address from user to kernel buffer ret: %d\n",
			ret);
		goto free_src_mem;
	}

	ret = fcs_plat_copy_from_user(s_buf + 4, &ctx.qspi_write.qspi_len, 4);
	if (ret) {
		LOG_ERR("Failed to copy QSPI write length from user to kernel buffer ret: %d\n",
			ret);
		goto free_src_mem;
	}

	ret = fcs_plat_copy_from_user(s_buf + 8, ctx.qspi_write.qspi_data,
				      ctx.qspi_write.qspi_len * 4);
	if (ret) {
		LOG_ERR("Failed to copy data from user to kernel buffer ret: %d\n",
			ret);
		goto free_src_mem;
	}

	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_src, s_buf, s_buf_sz,
				    FCS_DMA_TO_DEVICE);
	if (ret) {
		LOG_ERR("Failed to map dma address for the QSPI write ret: %d\n",
			ret);
		goto free_src_mem;
	}

	k_ctx->qspi_write.qspi_data = s_buf;
	k_ctx->qspi_write.qspi_data_len = &s_buf_sz;

	ret = priv->plat_data->svc_send_request(priv, FCS_DEV_QSPI_WRITE,
						FCS_REQUEST_TIMEOUT);

	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_QSPI_WRITE, ret);
		goto unmap;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to write QSPI ret: %d\n", ret);
	}

	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
	}

	priv->plat_data->svc_task_done(priv);
unmap:
	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_src, s_buf_sz,
				FCS_DMA_TO_DEVICE);
free_src_mem:
	priv->plat_data->svc_free_memory(priv, s_buf);

	return 0;
}
EXPORT_SYMBOL(hal_qspi_write);

FCS_HAL_INT hal_qspi_erase(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	struct fcs_cmd_context ctx;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	ret = priv->plat_data->svc_send_request(priv, FCS_DEV_QSPI_ERASE,
						FCS_REQUEST_TIMEOUT);

	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_QSPI_ERASE, ret);
		return ret;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to erase QSPI ret: %d\n", ret);
	}

	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
	}

	priv->plat_data->svc_task_done(priv);

	return 0;
}
EXPORT_SYMBOL(hal_qspi_erase);

FCS_HAL_INT hal_sdos_crypt(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_DMA_ADDR fcs_dma_handle_src, fcs_dma_handle_dst;
	FCS_HAL_VOID *s_buf = NULL, *d_buf = NULL;
	struct fcs_cmd_context ctx;
	FCS_HAL_U32 output_size;
	FCS_HAL_U64 owner_id;
	FCS_HAL_INT ret = 0;
	FCS_HAL_CHAR *temp;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	if (ctx.sdos.op_mode)
		output_size = SDOS_ENCRYPTED_MAX_SZ;
	else
		output_size = SDOS_DECRYPTED_MAX_SZ;

	/* Compare the session UUIDs to check for a match. Here suuid is set
	 * through hal_store_context
	 */
	ret = fcs_plat_uuid_compare(&priv->uuid_id, &k_ctx->rng.suuid);
	if (!ret) {
		LOG_ERR("Session UUID Mismatch ret: %d\n", ret);
		ret = -EINVAL;
		return ret;
	}

	s_buf = priv->plat_data->svc_alloc_memory(priv, k_ctx->sdos.src_size);
	if (IS_ERR(s_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for SDOS input data kernel buffer ret: %d\n",
			ret);
		return ret;
	}

	k_ctx->sdos.dst_size = &output_size;

	d_buf = priv->plat_data->svc_alloc_memory(priv, *k_ctx->sdos.dst_size);
	if (IS_ERR(d_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for SDOS output kernel buffer ret: %d\n",
			ret);
		goto free_sbuf;
	}

	/* Copy the user space input data to the input data kernel buffer */
	ret = fcs_plat_copy_from_user(s_buf, k_ctx->sdos.src,
				      k_ctx->sdos.src_size);
	if (ret) {
		LOG_ERR("Failed to copy SDOS data from user to kernel buffer ret: %d\n",
			ret);
		goto free_dbuf;
	}

	/* Get Owner ID from buf */
	temp = (uint8_t *)s_buf;
	memcpy(&owner_id, temp + OWNER_ID_OFFSET, OWNER_ID_SIZE);

	k_ctx->sdos.own = owner_id;

	/* Map the input data kernel buffer for DMA */
	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_src, s_buf,
				    k_ctx->sdos.src_size, FCS_DMA_TO_DEVICE);
	if (ret) {
		LOG_ERR("Failed to map dma address for the digest input data kernel buffer ret: %d\n",
			ret);
		goto free_dbuf;
	}

	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_dst, d_buf,
				    *k_ctx->sdos.dst_size, FCS_DMA_FROM_DEVICE);
	if (ret) {
		LOG_ERR("Failed to perform dma address map for output buffer ret: %d\n",
			ret);
		goto unmap_sbuf;
	}

	k_ctx->sdos.src = s_buf;
	k_ctx->sdos.dst = d_buf;

	ret = priv->plat_data->svc_send_request(priv, FCS_DEV_SDOS_DATA_EXT,
						FCS_REQUEST_TIMEOUT);
	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst,
				*k_ctx->sdos.dst_size, FCS_DMA_FROM_DEVICE);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_SDOS_DATA_EXT, ret);
		goto unmap_sbuf;
	}
	if (priv->status) {
		LOG_ERR("Mailbox error, Failed to perform SDOS operation ret: %d priv->status = %d\n",
			ret, priv->status);
		ret = -EIO;
		goto copy_mbox_status;
	}

	/* Copy the encrypted/decrypted output from kernel space to user space */
	ret = fcs_plat_copy_to_user(ctx.sdos.dst, d_buf, priv->resp);
	if (ret) {
		LOG_ERR("Failed to copy encrypted output to user ret: %d\n",
			ret);
		goto copy_mbox_status;
	}

	/* Copy the encrypted output length from kernel space to user space */
	ret = fcs_plat_copy_to_user(ctx.sdos.dst_size, &priv->resp,
				    sizeof(priv->resp));
	if (ret) {
		LOG_ERR("Failed to copy encrypted output length to user ret: %d\n",
			ret);
	}

copy_mbox_status:
	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
	}
	priv->plat_data->svc_task_done(priv);
	goto unmap_sbuf;
unmap_sbuf:
	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_src, k_ctx->sdos.src_size,
				FCS_DMA_TO_DEVICE);
free_dbuf:
	priv->plat_data->svc_free_memory(priv, d_buf);
free_sbuf:
	priv->plat_data->svc_free_memory(priv, s_buf);

	return ret;
}
EXPORT_SYMBOL(hal_sdos_crypt);

FCS_HAL_INT hal_ecdsa_get_pubkey(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	FCS_HAL_UINT pubkey_len = FCS_ECC_PUBKEY_LEN;
	FCS_HAL_VOID *d_buf = NULL;
	FCS_HAL_DMA_ADDR fcs_dma_handle_dst;
	struct fcs_cmd_context ctx;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	/* Compare the session UUIDs to check for a match. Here suuid is set
	 * through hal_store_context
	 */
	ret = fcs_plat_uuid_compare(&priv->uuid_id,
				    &k_ctx->ecdsa_pub_key.suuid);
	if (!ret) {
		ret = -EINVAL;
		LOG_ERR("session UUID Mismatch while requesting pubkey ret: %d\n",
			ret);
		return ret;
	}

	ret = priv->plat_data->svc_send_request(
		priv, FCS_DEV_CRYPTO_ECDSA_GET_PUBLIC_KEY_INIT,
		FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_CRYPTO_ECDSA_GET_PUBLIC_KEY_INIT, ret);
		return ret;
	}

	if (priv->status) {
		ret = -EIO;
		priv->plat_data->svc_task_done(priv);
		LOG_ERR("Failed to get public key with mbox status:0x%X\n",
			priv->status);
		return ret;
	}

	d_buf = priv->plat_data->svc_alloc_memory(priv, pubkey_len);
	if (IS_ERR(d_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for ECDSA public key kernel buffer ret: %d\n",
			ret);
		return ret;
	}

	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_dst, d_buf,
				    pubkey_len, FCS_DMA_FROM_DEVICE);
	if (ret)
		goto free_dest;

	k_ctx->ecdsa_pub_key.pubkey = d_buf;
	k_ctx->ecdsa_pub_key.pubkey_len = &pubkey_len;

	ret = priv->plat_data->svc_send_request(
		priv, FCS_DEV_CRYPTO_ECDSA_GET_PUBLIC_KEY_FINALIZE,
		10 * FCS_REQUEST_TIMEOUT);

	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst, FCS_ECC_PUBKEY_LEN,
				FCS_DMA_FROM_DEVICE);

	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_CRYPTO_ECDSA_GET_PUBLIC_KEY_FINALIZE, ret);
		goto free_dest;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, get ECDSA public key request failed ret: %d\n",
			ret);
		goto copy_mbox_status;
	}

	priv->resp -= RESPONSE_HEADER_SIZE;

	ret = fcs_plat_copy_to_user(ctx.ecdsa_pub_key.pubkey,
				    d_buf + RESPONSE_HEADER_SIZE, priv->resp);
	if (ret) {
		LOG_ERR("Failed to copy ECDSA public key to user ret: %d\n",
			ret);
		goto copy_mbox_status;
	}

	ret = fcs_plat_copy_to_user(ctx.ecdsa_pub_key.pubkey_len, &priv->resp,
				    sizeof(priv->resp));
	if (ret) {
		LOG_ERR("Failed to copy ECDSA public key length to user ret: %d\n",
			ret);
	}

copy_mbox_status:
	if (fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				  sizeof(priv->status))) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
	}

	priv->plat_data->svc_task_done(priv);

free_dest:
	priv->plat_data->svc_free_memory(priv, d_buf);

	return ret;
}
EXPORT_SYMBOL(hal_ecdsa_get_pubkey);

FCS_HAL_INT hal_ecdsa_hash_sign(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	FCS_HAL_VOID *s_buf = NULL, *d_buf = NULL;
	FCS_HAL_DMA_ADDR fcs_dma_handle_src, fcs_dma_handle_dst;
	FCS_HAL_UINT hash_len = FCS_ECDSA_HASH_SIGN_MAX_LEN;
	struct fcs_cmd_context ctx;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	/* Compare the session UUIDs to check for a match. Here suuid is set
	 * through hal_store_context
	 */
	ret = fcs_plat_uuid_compare(&priv->uuid_id,
				    &k_ctx->ecdsa_hash_sign.suuid);
	if (!ret) {
		ret = -EINVAL;
		LOG_ERR("session UUID Mismatch ret: %d\n", ret);
		return ret;
	}

	ret = priv->plat_data->svc_send_request(
		priv, FCS_DEV_CRYPTO_ECDSA_HASH_SIGNING_INIT,
		FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_CRYPTO_ECDSA_HASH_SIGNING_INIT, ret);
		return ret;
	}

	if (priv->status) {
		ret = -EIO;
		priv->plat_data->svc_task_done(priv);
		LOG_ERR("ECDSA Hash sign initialization failed mbox status:0x%X\n",
			priv->status);
		return ret;
	}

	s_buf = priv->plat_data->svc_alloc_memory(priv,
						  ctx.ecdsa_hash_sign.src_len);
	if (IS_ERR(s_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for ECDSA hash sign ret: %d\n",
			ret);
		return ret;
	}

	/* Copy the user space input data to the input data kernel buffer */
	ret = fcs_plat_copy_from_user(s_buf, ctx.ecdsa_hash_sign.src,
				      ctx.ecdsa_hash_sign.src_len);
	if (ret) {
		LOG_ERR("Failed to copy ECDSA sign data from user to kernel buffer ret: %d\n",
			ret);
		goto free_sbuf;
	}

	/* Map the input data kernel buffer for DMA */
	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_src, s_buf,
				    ctx.ecdsa_hash_sign.src_len,
				    FCS_DMA_TO_DEVICE);
	if (ret) {
		LOG_ERR("ECDSA hash sign for src buf failed to map dma address ret: %d\n",
			ret);
		goto free_sbuf;
	}

	d_buf = priv->plat_data->svc_alloc_memory(priv, hash_len);
	if (IS_ERR(d_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for ECDSA hash sign dst buffer ret: %d\n",
			ret);
		goto unmap;
	}

	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_dst, d_buf, hash_len,
				    FCS_DMA_FROM_DEVICE);
	if (ret) {
		LOG_ERR("ECDSA hash sign for dst buf failed to map dma address ret: %d\n",
			ret);
		goto free_dbuf;
	}

	k_ctx->ecdsa_hash_sign.src = s_buf;
	k_ctx->ecdsa_hash_sign.dst = d_buf;
	k_ctx->ecdsa_hash_sign.dst_len = &hash_len;

	ret = priv->plat_data->svc_send_request(
		priv, FCS_DEV_CRYPTO_ECDSA_HASH_SIGNING_FINALIZE,
		10 * FCS_REQUEST_TIMEOUT);

	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst,
				FCS_ECDSA_HASH_SIGN_MAX_LEN,
				FCS_DMA_FROM_DEVICE);

	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_CRYPTO_ECDSA_HASH_SIGNING_FINALIZE, ret);
		goto free_dbuf;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("ECDSA Hash sign initialization failed mbox status:0x%X\n",
			priv->status);
		goto copy_mbox_status;
	}

	priv->resp -= RESPONSE_HEADER_SIZE;

	ret = fcs_plat_copy_to_user(ctx.ecdsa_hash_sign.dst_len, &priv->resp,
				    priv->resp);
	if (ret) {
		LOG_ERR("Failed to copy ECDSA sign data length to user ret: %d\n",
			ret);
	}

	ret = fcs_plat_copy_to_user(ctx.ecdsa_hash_sign.dst,
				    d_buf + RESPONSE_HEADER_SIZE, priv->resp);
	if (ret) {
		LOG_ERR("Failed to copy ECDSA sign data to user ret: %d\n",
			ret);
	}

copy_mbox_status:
	if (fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				  sizeof(priv->status))) {
		LOG_ERR("Failed to copy mailbox status code to user\n");
	}
	priv->plat_data->svc_task_done(priv);

free_dbuf:
	priv->plat_data->svc_free_memory(priv, d_buf);
unmap:
	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_src,
				k_ctx->ecdsa_hash_sign.src_len,
				FCS_DMA_TO_DEVICE);
free_sbuf:
	priv->plat_data->svc_free_memory(priv, s_buf);

	return ret;
}
EXPORT_SYMBOL(hal_ecdsa_hash_sign);

FCS_HAL_INT hal_ecdsa_hash_verify(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	FCS_HAL_VOID *s_buf = NULL, *d_buf = NULL;
	FCS_HAL_U32 total_sz;
	FCS_HAL_DMA_ADDR fcs_dma_handle_src, fcs_dma_handle_dst;
	FCS_HAL_UINT hash_len = FCS_ECDSA_HASH_SIGN_MAX_LEN;
	struct fcs_cmd_context ctx;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	/* Compare the session UUIDs to check for a match. Here suuid is set
	 * through hal_store_context
	 */
	ret = fcs_plat_uuid_compare(&priv->uuid_id,
				    &k_ctx->ecdsa_hash_verify.suuid);
	if (!ret) {
		ret = -EINVAL;
		LOG_ERR("session UUID Mismatch ret: %d\n", ret);
		return ret;
	}

	ret = priv->plat_data->svc_send_request(
		priv, FCS_DEV_CRYPTO_ECDSA_HASH_VERIFY_INIT,
		FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_CRYPTO_ECDSA_HASH_VERIFY_INIT, ret);
		return ret;
	}

	if (priv->status) {
		ret = -EIO;
		priv->plat_data->svc_task_done(priv);
		LOG_ERR("Failed to initialize ECDSA verify, mbox status:0x%X\n",
			priv->status);
		return ret;
	}

	total_sz = ctx.ecdsa_hash_verify.src_len +
		   ctx.ecdsa_hash_verify.signature_len +
		   ctx.ecdsa_hash_verify.pubkey_len;

	s_buf = priv->plat_data->svc_alloc_memory(priv, total_sz);
	if (IS_ERR(s_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for ECDSA verify src buffer ret: %d\n",
			ret);
		return ret;
	}

	/* Copy the user space input data to the input data kernel buffer */
	ret = fcs_plat_copy_from_user(s_buf, ctx.ecdsa_hash_verify.src,
				      ctx.ecdsa_hash_verify.src_len);
	if (ret) {
		LOG_ERR("Failed to copy ECDSA verify user data from user to sbuf ret: %d\n",
			ret);
		goto free_sbuf;
	}

	/* Copy the user space signature data to the input data kernel buffer */
	ret = fcs_plat_copy_from_user(s_buf + ctx.ecdsa_hash_verify.src_len,
				      ctx.ecdsa_hash_verify.signature,
				      ctx.ecdsa_hash_verify.signature_len);
	if (ret) {
		LOG_ERR("Failed to copy ECDSA verify signature from user to sbuf ret: %d\n",
			ret);
		goto free_sbuf;
	}

	if (ctx.ecdsa_hash_verify.key_id == 0) {
		/* Copy the user space public key data to the input data kernel buffer */
		ret = fcs_plat_copy_from_user(
			s_buf + ctx.ecdsa_hash_verify.src_len +
				ctx.ecdsa_hash_verify.signature_len,
			ctx.ecdsa_hash_verify.pubkey,
			ctx.ecdsa_hash_verify.pubkey_len);
		if (ret) {
			LOG_ERR("ECDSA verify: copy from user failed for public key ret:%d\n",
				ret);
			goto free_sbuf;
		}
	}

	/* Map the input data kernel buffer for DMA */
	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_src, s_buf, total_sz,
				    FCS_DMA_TO_DEVICE);
	if (ret) {
		LOG_ERR("Failed to map dma address for the ECDSA verify src buffer ret: %d\n",
			ret);
		goto free_sbuf;
	}

	d_buf = priv->plat_data->svc_alloc_memory(priv, hash_len);
	if (IS_ERR(d_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for ECDSA verify dst buffer ret: %d\n",
			ret);
		goto unmap;
	}

	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_dst, d_buf, hash_len,
				    FCS_DMA_FROM_DEVICE);
	if (ret) {
		LOG_ERR("Failed to map dma address for the ECDSA verify dst buffer ret: %d\n",
			ret);
		goto free_dbuf;
	}

	k_ctx->ecdsa_hash_verify.src = s_buf;
	k_ctx->ecdsa_hash_verify.src_len = total_sz;
	k_ctx->ecdsa_hash_verify.dst = d_buf;
	k_ctx->ecdsa_hash_verify.dst_len = &hash_len;

	ret = priv->plat_data->svc_send_request(
		priv, FCS_DEV_CRYPTO_ECDSA_HASH_VERIFY_FINALIZE,
		10 * FCS_REQUEST_TIMEOUT);

	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst,
				FCS_ECDSA_HASH_SIGN_MAX_LEN,
				FCS_DMA_FROM_DEVICE);

	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_CRYPTO_ECDSA_HASH_VERIFY_FINALIZE, ret);
		goto free_dbuf;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Failed to perform ECDSA verify mbox status:0x%X\n",
			priv->status);
		goto copy_mbox_status;
	}

	priv->resp -= RESPONSE_HEADER_SIZE;

	ret = fcs_plat_copy_to_user(ctx.ecdsa_hash_verify.dst_len, &priv->resp,
				    priv->resp);
	if (ret) {
		LOG_ERR("Failed to copy ECDSA verify data length to user ret: %d\n",
			ret);
	}

	ret = fcs_plat_copy_to_user(ctx.ecdsa_hash_verify.dst,
				    d_buf + RESPONSE_HEADER_SIZE, priv->resp);
	if (ret) {
		LOG_ERR("Failed to copy ECDSA verify data to user ret: %d\n",
			ret);
	}

copy_mbox_status:
	if (copy_to_user(ctx.error_code_addr, &priv->status,
			 sizeof(priv->status))) {
		LOG_ERR("Failed to copy mailbox status code to user\n");
	}

	priv->plat_data->svc_task_done(priv);

free_dbuf:
	priv->plat_data->svc_free_memory(priv, d_buf);
unmap:
	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_src, total_sz,
				FCS_DMA_TO_DEVICE);
free_sbuf:
	priv->plat_data->svc_free_memory(priv, s_buf);

	return ret;
}
EXPORT_SYMBOL(hal_ecdsa_hash_verify);

static FCS_HAL_INT
hal_ecdsa_sha2data_sign_upfinal(FCS_HAL_VOID *src, FCS_HAL_U32 src_len,
				FCS_HAL_VOID *dst, FCS_HAL_U32 dst_len,
				struct fcs_cmd_context *const k_ctx,
				FCS_HAL_U32 command)
{
	FCS_HAL_INT ret = 0;
	FCS_HAL_DMA_ADDR fcs_dma_handle_src;

	k_ctx->ecdsa_sha2_data_sign.src_len = src_len;

	ret = fcs_plat_copy_from_user(k_ctx->ecdsa_sha2_data_sign.src, src,
				      src_len);
	if (ret) {
		LOG_ERR("Failed to copy ECDSA sign data from user to src buffer ret: %d\n",
			ret);
		return ret;
	}

	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_src,
				    k_ctx->ecdsa_sha2_data_sign.src, src_len,
				    FCS_DMA_TO_DEVICE);
	if (ret) {
		LOG_ERR("Failed to map dma address for the ECDSA sign src buf ret: %d\n",
			ret);
		return ret;
	}

	ret = priv->plat_data->svc_send_request(priv, command,
						10 * FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n", command, ret);
		goto unmap;
	}

	ret = fcs_plat_copy_to_user(k_ctx->error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
		goto unmap;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Failed to perform ECDSA sha2 data sign mbox status: 0x%x\n",
			priv->status);
	}

	priv->plat_data->svc_task_done(priv);

unmap:
	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_src, src_len,
				FCS_DMA_TO_DEVICE);

	return ret;
}

static FCS_HAL_INT
hal_ecdsa_sha2_data_sign_init(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;

	ret = priv->plat_data->svc_send_request(
		priv, FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_SIGNING_INIT,
		FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_SIGNING_INIT, ret);
		return ret;
	}

	if (priv->status) {
		ret = -EIO;
		priv->plat_data->svc_task_done(priv);
		LOG_ERR("Failed to initialize ECDSA sign ret: %d\n", ret);
		return ret;
	}

	return ret;
}

FCS_HAL_INT hal_ecdsa_sha2_data_sign(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	FCS_HAL_VOID *s_buf = NULL, *d_buf = NULL;
	FCS_HAL_VOID *ip_ptr = NULL;
	FCS_HAL_U32 s_buf_sz, d_buf_sz;
	FCS_HAL_U32 remaining_sz;
	FCS_HAL_DMA_ADDR fcs_dma_handle_dst;
	struct fcs_cmd_context ctx;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	/* Compare the session UUIDs to check for a match.
	 * Here suuid is set through hal_store_context
	 */
	ret = fcs_plat_uuid_compare(&priv->uuid_id,
				    &k_ctx->ecdsa_sha2_data_sign.suuid);
	if (!ret) {
		ret = -EINVAL;
		LOG_ERR("session UUID Mismatch while performing sha2 data sign ret: %d\n",
			ret);
		return ret;
	}

	ret = hal_ecdsa_sha2_data_sign_init(k_ctx);
	if (ret) {
		LOG_ERR("Failed to initialize ECDSA sign ret: %d\n", ret);
		return ret;
	}

	s_buf_sz = (ctx.ecdsa_sha2_data_sign.src_len >
		    FCS_ECDSA_HSHA2_DATA_SIGN_BLOCK_SZ) ?
			   FCS_ECDSA_HSHA2_DATA_SIGN_BLOCK_SZ :
			   ctx.ecdsa_sha2_data_sign.src_len;

	s_buf = priv->plat_data->svc_alloc_memory(priv, s_buf_sz);
	if (IS_ERR(s_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for ECDSA sign src buffer ret: %d\n",
			ret);
		return ret;
	}

	d_buf_sz = FCS_ECDSA_HASH_SIGN_MAX_LEN;

	d_buf = priv->plat_data->svc_alloc_memory(priv, d_buf_sz);
	if (IS_ERR(d_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for ECDSA sign dst buffer ret: %d\n",
			ret);
		goto free_sbuf;
	}

	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_dst, d_buf, d_buf_sz,
				    FCS_DMA_FROM_DEVICE);
	if (ret) {
		LOG_ERR("Failed to map dma address for the ECDSA sign dst buffer ret: %d\n",
			ret);
		goto free_mem;
	}

	remaining_sz = ctx.ecdsa_sha2_data_sign.src_len;
	ip_ptr = ctx.ecdsa_sha2_data_sign.src;

	k_ctx->ecdsa_sha2_data_sign.src = s_buf;
	k_ctx->ecdsa_sha2_data_sign.dst = d_buf;
	k_ctx->ecdsa_sha2_data_sign.dst_len = &d_buf_sz;

	/**
	 * Perform the update and final stage of ECDSA SHA-2 data signing.
	 *
	 * This function processes the input data in blocks of size
	 * FCS_ECDSA_HSHA2_DATA_SIGN_BLOCK_SZ. For each block, it calls
	 * hal_ecdsa_sha2data_sign_upfinal to perform the cryptographic update.
	 *
	 * if the remaining_sz is less than FCS_ECDSA_HSHA2_DATA_SIGN_BLOCK_SZ,
	 * the final block is processed by sending the final command to the SDM.
	 */
	while (remaining_sz > FCS_ECDSA_HSHA2_DATA_SIGN_BLOCK_SZ) {
		ret = hal_ecdsa_sha2data_sign_upfinal(
			ip_ptr, s_buf_sz, d_buf, d_buf_sz, k_ctx,
			FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_SIGNING_UPDATE);
		if (ret) {
			LOG_ERR("Failed to perform SHA2 data sign update ret: %d\n",
				ret);
			fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst,
						FCS_ECDSA_HASH_SIGN_MAX_LEN,
						FCS_DMA_FROM_DEVICE);
			goto copy_mbox_status;
		}
		remaining_sz -= FCS_ECDSA_HSHA2_DATA_SIGN_BLOCK_SZ;
		ip_ptr += FCS_ECDSA_HSHA2_DATA_SIGN_BLOCK_SZ;
	}

	ret = hal_ecdsa_sha2data_sign_upfinal(
		ip_ptr, remaining_sz, d_buf, d_buf_sz, k_ctx,
		FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_SIGNING_FINALIZE);
	if (ret) {
		LOG_ERR("Failed to perform ECDSA SHA2 Data Signing final ret: %d\n",
			ret);
			fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst,
						FCS_ECDSA_HASH_SIGN_MAX_LEN,
						FCS_DMA_FROM_DEVICE);
		goto copy_mbox_status;
	}

	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst,
				FCS_ECDSA_HASH_SIGN_MAX_LEN,
				FCS_DMA_FROM_DEVICE);

	priv->resp -= RESPONSE_HEADER_SIZE;

	ret = fcs_plat_copy_to_user(ctx.ecdsa_sha2_data_sign.dst_len,
				    &priv->resp, sizeof(priv->resp));
	if (ret) {
		LOG_ERR("Failed to copy ECDSA sign data length to user ret: %d\n",
			ret);
		goto copy_mbox_status;
	}

	ret = fcs_plat_copy_to_user(ctx.ecdsa_sha2_data_sign.dst,
				    d_buf + RESPONSE_HEADER_SIZE, priv->resp);
	if (ret) {
		LOG_ERR("Failed to copy ECDSA sign data to user ret: %d\n",
			ret);
	}

copy_mbox_status:
	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
	}

	priv->plat_data->svc_task_done(priv);

free_mem:
	priv->plat_data->svc_free_memory(priv, d_buf);
free_sbuf:
	priv->plat_data->svc_free_memory(priv, s_buf);

	return ret;
}
EXPORT_SYMBOL(hal_ecdsa_sha2_data_sign);

static FCS_HAL_INT hal_ecdsa_sha2data_verify_upfinal(
	FCS_HAL_VOID *ip_ptr, FCS_HAL_U32 ip_len, FCS_HAL_CHAR *signature,
	FCS_HAL_U32 signature_len, FCS_HAL_CHAR *pubkey, FCS_HAL_U32 pubkey_len,
	FCS_HAL_VOID *dst, FCS_HAL_U32 dst_len,
	struct fcs_cmd_context *const k_ctx, FCS_HAL_U32 command)
{
	FCS_HAL_INT ret = 0;
	FCS_HAL_DMA_ADDR fcs_dma_handle_src;
	FCS_HAL_U32 copy_sz = ip_len;

	ret = fcs_plat_copy_from_user(k_ctx->ecdsa_sha2_data_verify.src, ip_ptr,
				      ip_len);
	if (ret) {
		LOG_ERR("Failed to copy ECDSA sign data from user to kernel buffer ret: %d\n",
			ret);
		return ret;
	}

	if (command == FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_VERIFY_FINALIZE) {
		copy_sz += signature_len + pubkey_len;

		ret = fcs_plat_copy_from_user(
			k_ctx->ecdsa_sha2_data_verify.src + ip_len, signature,
			signature_len);
		if (ret) {
			LOG_ERR("ECDSA sha2verify: signature from user failed ret: %d\n",
				ret);
			return ret;
		}

		if (k_ctx->ecdsa_sha2_data_verify.key_id == 0) {
			ret = fcs_plat_copy_from_user(
				k_ctx->ecdsa_sha2_data_verify.src + ip_len +
					signature_len,
				pubkey, pubkey_len);
			if (ret) {
				LOG_ERR("ECDSA sha2verify: pubkey from user failed ret: %d\n",
					ret);
				return ret;
			}
		}
	}

	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_src,
				    k_ctx->ecdsa_sha2_data_verify.src, copy_sz,
				    FCS_DMA_TO_DEVICE);
	if (ret) {
		LOG_ERR("Failed to map(src) dma address for the ECDSA sha2verify ret: %d\n",
			ret);
		return ret;
	}

	ret = priv->plat_data->svc_send_request(priv, command,
						10 * FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n", command, ret);
		goto unmap;
	}

	ret = fcs_plat_copy_to_user(k_ctx->error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
		goto unmap;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Failed to perform ECDSA sha2 data verify mbox status:%x\n",
			priv->status);
	}

	priv->plat_data->svc_task_done(priv);

unmap:
	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_src, copy_sz,
				FCS_DMA_TO_DEVICE);

	return ret;
}

static FCS_HAL_INT
hal_ecdsa_sha2_data_verify_init(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;

	ret = priv->plat_data->svc_send_request(
		priv, FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_VERIFY_INIT,
		FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_VERIFY_INIT, ret);
		return ret;
	}

	if (priv->status) {
		ret = -EIO;
		priv->plat_data->svc_task_done(priv);
		LOG_ERR("Mailbox error, Failed to initialize ECDSA verify ret: %d\n",
			ret);
		return ret;
	}

	return ret;
}

FCS_HAL_INT hal_ecdsa_sha2_data_verify(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	FCS_HAL_VOID *s_buf = NULL, *d_buf = NULL;
	FCS_HAL_U32 s_buf_sz, d_buf_sz = FCS_ECDSA_SHA2_DATA_VERIFY_RSP_SZ;
	FCS_HAL_U32 remaining_sz;
	FCS_HAL_VOID *ip_ptr = NULL;
	FCS_HAL_DMA_ADDR fcs_dma_handle_dst;
	FCS_HAL_U32 command;
	struct fcs_cmd_context ctx;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	/* Compare the session UUIDs to check for a match. Here suuid is set
	 * through hal_store_context
	 */
	ret = fcs_plat_uuid_compare(&priv->uuid_id,
				    &k_ctx->ecdsa_sha2_data_verify.suuid);
	if (!ret) {
		ret = -EINVAL;
		LOG_ERR("session UUID Mismatch in sha2 data verify request ret: %d\n",
			ret);
		return ret;
	}

	ret = hal_ecdsa_sha2_data_verify_init(k_ctx);
	if (ret) {
		LOG_ERR("Failed to initialize ECDSA verify ret: %d\n", ret);
		return ret;
	}

	remaining_sz = ctx.ecdsa_sha2_data_verify.src_len +
		       ctx.ecdsa_sha2_data_verify.signature_len +
		       ctx.ecdsa_sha2_data_verify.pubkey_len;

	s_buf_sz = (remaining_sz > FCS_ECDSA_SHA2_DATA_VERIFY_BLOCK_SZ) ?
			   FCS_ECDSA_SHA2_DATA_VERIFY_BLOCK_SZ :
			   remaining_sz;

	s_buf = priv->plat_data->svc_alloc_memory(priv, s_buf_sz);
	if (IS_ERR(s_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for ECDSA sha2verify src buffer ret: %d\n",
			ret);
		return ret;
	}

	d_buf = priv->plat_data->svc_alloc_memory(priv, d_buf_sz);
	if (IS_ERR(d_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for ECDSA sha2verify dst buffer ret: %d\n",
			ret);
		goto free_sbuf;
	}

	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_dst, d_buf, d_buf_sz,
				    FCS_DMA_FROM_DEVICE);
	if (ret) {
		LOG_ERR("Failed to map(dbuf) dma address for the ECDSA sha2verify ret: %d\n",
			ret);
		goto free_mem;
	}

	k_ctx->ecdsa_sha2_data_verify.src = s_buf;
	k_ctx->ecdsa_sha2_data_verify.dst = d_buf;
	k_ctx->ecdsa_sha2_data_verify.dst_len = &d_buf_sz;

	ip_ptr = ctx.ecdsa_sha2_data_verify.src;

	/**
	 * Perform the update and final stage of ECDSA SHA-2 data verification.
	 *
	 * This function processes the input data in blocks of size
	 * FCS_ECDSA_SHA2_DATA_VERIFY_BLOCK_SZ. For each block, it calls
	 * hal_ecdsa_sha2data_verify_upfinal to perform the cryptographic update.
	 *
	 * if the remaining_sz is less than FCS_ECDSA_SHA2_DATA_VERIFY_BLOCK_SZ,
	 * the final block is processed by sending the final command to the SDM.
	 */

	/* Final stage requires minimum 8-bytes of source buffer to be sent */

	while (remaining_sz > 0) {
		if (remaining_sz > FCS_ECDSA_SHA2_DATA_VERIFY_BLOCK_SZ) {
			if ((remaining_sz -
			     FCS_ECDSA_SHA2_DATA_VERIFY_BLOCK_SZ) >=
			    (CRYPTO_SERVICE_MIN_DATA_SIZE +
			     ctx.ecdsa_sha2_data_verify.signature_len +
			     ctx.ecdsa_sha2_data_verify.pubkey_len)) {
				k_ctx->ecdsa_sha2_data_verify.src_len =
					FCS_ECDSA_SHA2_DATA_VERIFY_BLOCK_SZ;
				k_ctx->ecdsa_sha2_data_verify.user_data_sz =
					FCS_ECDSA_SHA2_DATA_VERIFY_BLOCK_SZ;
			} else {
				k_ctx->ecdsa_sha2_data_verify.src_len =
					remaining_sz -
					CRYPTO_SERVICE_MIN_DATA_SIZE -
					ctx.ecdsa_sha2_data_verify
						.signature_len -
					ctx.ecdsa_sha2_data_verify.pubkey_len;
				k_ctx->ecdsa_sha2_data_verify.user_data_sz =
					remaining_sz -
					CRYPTO_SERVICE_MIN_DATA_SIZE -
					ctx.ecdsa_sha2_data_verify
						.signature_len -
					ctx.ecdsa_sha2_data_verify.pubkey_len;
			}

			command = FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_VERIFY_UPDATE;
		} else {
			k_ctx->ecdsa_sha2_data_verify.src_len = remaining_sz;
			k_ctx->ecdsa_sha2_data_verify.user_data_sz =
				remaining_sz -
				ctx.ecdsa_sha2_data_verify.signature_len -
				ctx.ecdsa_sha2_data_verify.pubkey_len;
			command =
				FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_VERIFY_FINALIZE;
		}

		ret = hal_ecdsa_sha2data_verify_upfinal(
			ip_ptr, k_ctx->ecdsa_sha2_data_verify.user_data_sz,
			ctx.ecdsa_sha2_data_verify.signature,
			ctx.ecdsa_sha2_data_verify.signature_len,
			ctx.ecdsa_sha2_data_verify.pubkey,
			ctx.ecdsa_sha2_data_verify.pubkey_len, d_buf, d_buf_sz,
			k_ctx, command);
		if (ret) {
			LOG_ERR("Failed to perform SHA2 Data verify final ret: %d\n",
				ret);
			fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst,
						FCS_ECDSA_SHA2_DATA_VERIFY_RSP_SZ,
						FCS_DMA_FROM_DEVICE);
			goto free_mem;
		}

		ip_ptr += k_ctx->ecdsa_sha2_data_verify.src_len;
		remaining_sz -= k_ctx->ecdsa_sha2_data_verify.src_len;
	}

	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_dst,
				FCS_ECDSA_SHA2_DATA_VERIFY_RSP_SZ,
				FCS_DMA_FROM_DEVICE);

	priv->resp -= RESPONSE_HEADER_SIZE;

	ret = fcs_plat_copy_to_user(ctx.ecdsa_sha2_data_verify.dst,
				    d_buf + RESPONSE_HEADER_SIZE, priv->resp);
	if (ret) {
		LOG_ERR("Failed to copy ECDSA verify data to user ret: %d\n",
			ret);
		goto free_mem;
	}

	ret = fcs_plat_copy_to_user(ctx.ecdsa_sha2_data_verify.dst_len,
				    &priv->resp, sizeof(priv->resp));
	if (ret) {
		LOG_ERR("Failed to copy ECDSA verify data length to user ret: %d\n",
			ret);
	}

	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
	}

	priv->plat_data->svc_task_done(priv);

free_mem:
	priv->plat_data->svc_free_memory(priv, d_buf);
free_sbuf:
	priv->plat_data->svc_free_memory(priv, s_buf);

	return ret;
}
EXPORT_SYMBOL(hal_ecdsa_sha2_data_verify);

FCS_HAL_INT hal_hps_img_validate(struct fcs_cmd_context *const k_ctx)
{
	FCS_HAL_INT ret = 0;
	FCS_HAL_VOID *s_buf = NULL;
	FCS_HAL_DMA_ADDR fcs_dma_handle_src;
	struct fcs_cmd_context ctx;
	FCS_HAL_UINT s_buf_len = 0, tsz = 0;

	fcs_plat_memcpy(&ctx, k_ctx, sizeof(struct fcs_cmd_context));

	tsz = sizeof(ctx.hps_img_validate.test);
	s_buf_len = ctx.hps_img_validate.vab_cert_len + tsz;

	/* Allocate memory for the source buffer */
	s_buf = priv->plat_data->svc_alloc_memory(priv, s_buf_len);
	if (IS_ERR(s_buf)) {
		ret = -ENOMEM;
		LOG_ERR("Failed to allocate memory for HPS image buffer ret: %d\n",
			ret);
		return ret;
	}

	fcs_plat_memcpy(s_buf, &ctx.hps_img_validate.test, tsz);

	/* Copy the user space source data to the source buffer */
	ret = fcs_plat_copy_from_user(s_buf + tsz,
				      ctx.hps_img_validate.vab_cert,
				      ctx.hps_img_validate.vab_cert_len);
	if (ret) {
		LOG_ERR("Failed to copy HPS image validat from user to kernel buffer ret: %d\n",
			ret);
		goto free_mem;
	}

	/* Map the source buffer for DMA */
	ret = fcs_plat_dma_addr_map(priv, &fcs_dma_handle_src, s_buf, s_buf_len,
				    FCS_DMA_TO_DEVICE);
	if (ret) {
		LOG_ERR("Failed perform dma address map for the HPS image buffer ret: %d\n",
			ret);
		goto free_mem;
	}

	k_ctx->hps_img_validate.vab_cert = s_buf;
	k_ctx->hps_img_validate.vab_cert_len = s_buf_len;

	ret = priv->plat_data->svc_send_request(
		priv, FCS_DEV_HPS_IMG_VALIDATE_REQUEST, FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_HPS_IMG_VALIDATE_REQUEST, ret);
		goto unmap;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, failed to perform HPS image validation ret: %d\n",
			ret);
		goto copy_mbox_status;
	}

	ret = fcs_plat_copy_to_user(ctx.hps_img_validate.resp, &priv->resp,
				    sizeof(FCS_HAL_U32));
	if (ret) {
		LOG_ERR("Failed to copy Image validation response to user ret: %d\n",
			ret);
		goto copy_mbox_status;
	}

copy_mbox_status:
	ret = fcs_plat_copy_to_user(ctx.error_code_addr, &priv->status,
				    sizeof(priv->status));
	if (ret) {
		LOG_ERR("Failed to copy mailbox status code to user ret: %d\n",
			ret);
	}

	priv->plat_data->svc_task_done(priv);

unmap:
	fcs_plat_dma_addr_unmap(priv, &fcs_dma_handle_src, s_buf_len,
				FCS_DMA_TO_DEVICE);

free_mem:
	priv->plat_data->svc_free_memory(priv, s_buf);

	return ret;
}
EXPORT_SYMBOL(hal_hps_img_validate);

struct fcs_cmd_context *hal_get_fcs_cmd_ctx(void)
{
	fcs_plat_mutex_lock(priv);
	return &priv->k_ctx;
}
EXPORT_SYMBOL(hal_get_fcs_cmd_ctx);

FCS_HAL_VOID hal_destroy_fcs_cmd_ctx(struct fcs_cmd_context *const k_ctx)
{
	fcs_plat_memset(k_ctx, 0, sizeof(struct fcs_cmd_context));
}
EXPORT_SYMBOL(hal_destroy_fcs_cmd_ctx);

FCS_HAL_VOID hal_release_fcs_cmd_ctx(struct fcs_cmd_context *const k_ctx)
{
	fcs_plat_mutex_unlock(priv);
}
EXPORT_SYMBOL(hal_release_fcs_cmd_ctx);

static FCS_HAL_INT hal_read_version_from_atf(FCS_HAL_VOID)
{
	FCS_HAL_INT ret = 0;

	ret = priv->plat_data->svc_send_request(priv, FCS_DEV_ATF_VERSION,
						FCS_REQUEST_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to send the cmd=%d,ret=%d\n",
			FCS_DEV_ATF_VERSION, ret);
		return ret;
	}

	if (priv->status) {
		ret = -EIO;
		LOG_ERR("Mailbox error, Failed to read ATF version ret: %d\n",
			ret);
	}

	priv->plat_data->svc_task_done(priv);

	return ret;
}

FCS_HAL_INT hal_fcs_init(FCS_HAL_DEV *dev)
{
	FCS_HAL_INT ret;

	ret = fcs_plat_init(dev, priv);
	if (ret) {
		LOG_ERR("Failed to initialize platform data ret: %d\n", ret);
		return ret;
	}

	hal_read_version_from_atf();

	return ret;
}

FCS_HAL_VOID hal_fcs_cleanup(void)
{
	fcs_plat_cleanup(priv);
}
