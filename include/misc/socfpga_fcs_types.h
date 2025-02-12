/* SPDX-License-Identifier: GPL-2.0-or-later OR MIT */
/*
 * Copyright (C) 2025 Altera
 */

#ifndef SOCFPGA_FCS_TYPES_H
#define SOCFPGA_FCS_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <linux/completion.h>
#include <linux/firmware/intel/stratix10-svc-client.h>
#include <linux/dma-mapping.h>

#define LOG_ERR(fmt, ...)		pr_err(fmt, ##__VA_ARGS__)
#define LOG_DBG(fmt, ...)		pr_debug(fmt, ##__VA_ARGS__)
#define LOG_INF(fmt, ...)		pr_info(fmt, ##__VA_ARGS__)
#define LOG_WRN(fmt, ...)		pr_warn(fmt, ##__VA_ARGS__)

#define FCS_REQUEST_TIMEOUT	(msecs_to_jiffies(SVC_FCS_REQUEST_TIMEOUT_MS))
#define FCS_COMPLETED_TIMEOUT	(msecs_to_jiffies(SVC_COMPLETED_TIMEOUT_MS))

#define FCS_DMA_FROM_DEVICE		DMA_FROM_DEVICE
#define FCS_DMA_TO_DEVICE		DMA_TO_DEVICE

#define FCS_AES_BLOCK_MODE_ECB		0
#define FCS_AES_BLOCK_MODE_CBC		1
#define FCS_AES_BLOCK_MODE_CTR		2
#define FCS_AES_BLOCK_MODE_GCM		3
#define FCS_AES_BLOCK_MODE_GHASH	4
#define FCS_MAX_AES_CRYPT_MODE		5
#define FCS_AES_GCM_TAG_SIZE		3
#define FCS_AES_IV_SOURCE_EXTERNAL	0
#define FCS_AES_IV_SOURCE_INTERNAL	1
#define FCS_AES_ENCRYPT			0
#define FCS_AES_DECRYPT			1

#define FCS_ECC_CURVE_NIST_P256		1
#define FCS_ECC_CURVE_NIST_P384		2
#define FCS_ECC_CURVE_BRAINPOOL_P256	3
#define FCS_ECC_CURVE_BRAINPOOL_P384	4

#define FCS_ECC_CURVE_MASK		0xF

#define FCS_ECDH_P256_PUBKEY_LEN	64
#define FCS_ECDH_P384_PUBKEY_LEN	96
#define FCS_ECDH_BP256_PUBKEY_LEN	64
#define FCS_ECDH_BP384_PUBKEY_LEN	96
#define FCS_ECDH_P256_SECRET_LEN	32
#define FCS_ECDH_P384_SECRET_LEN	48
#define FCS_ECDH_BP256_SECRET_LEN	32
#define FCS_ECDH_BP384_SECRET_LEN	48

/** unsigned 64 bit*/
typedef u64 FCS_HAL_U64;
/** unsigned 32 bit*/
typedef u32 FCS_HAL_U32;
/** unsigned 16 bit*/
typedef u16 FCS_HAL_U16;
/** unsigned 8 bit*/
typedef u8 FCS_HAL_U8;

/** signed 64 bit*/
typedef s64 FCS_HAL_S64;
/** signed 32 bit*/
typedef s32 FCS_HAL_S32;
/** unsigned 16 bit*/
typedef s16 FCS_HAL_S16;
/** unsigned 8 bit*/
typedef s8 FCS_HAL_S8;

/** void type*/
typedef void FCS_HAL_VOID;
/** character data type*/
typedef char FCS_HAL_CHAR;
/** boolean data type*/
typedef bool FCS_HAL_BOOL;

/** integer data type*/
typedef int FCS_HAL_INT;
/** integer data type*/
typedef unsigned int FCS_HAL_UINT;
/** data type to denote offset */
typedef off_t FCS_HAL_OFFSET;
/** data type to denote size*/
typedef size_t FCS_HAL_SIZE;

/** integer data type uuid for session ids*/
typedef uuid_t FCS_HAL_UUID;

/** Unsigned long */
typedef unsigned long FCS_HAL_ULONG;

// TODO:  which data type
typedef int FCS_HAL_ERROR;

typedef struct completion FCS_HAL_COMPLETION;
typedef struct mutex FCS_HAL_MUTEX;

typedef struct device FCS_HAL_DEV;

typedef dma_addr_t FCS_HAL_DMA_ADDR;

typedef struct stratix10_svc_client_msg FCS_SVC_CLIENT_MSG;
typedef struct stratix10_svc_client FCS_SVC_CLIENT;
typedef struct stratix10_svc_cb_data FCS_SVC_CB_DATA;
typedef struct stratix10_svc_chan FCS_HAL_CHAN;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SOCFPGA_FCS_TYPES_H */
