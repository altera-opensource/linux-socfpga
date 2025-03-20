/* SPDX-License-Identifier: GPL-2.0-or-later OR MIT */
/*
 * Copyright (C) 2024 Altera
 */

/**
 *
 * @file socfpga_fcs_hal.h
 * @brief contains API interfaces description to be called by upper layer.
 */
#ifndef SPCFPGA_FCS_HAL_H
#define SPCFPGA_FCS_HAL_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "socfpga_fcs_types.h"

#define	MAX_SESSION				1
#define	CRYPTO_EXPORTED_KEY_OBJECT_MAX_SZ	364
#define	CRYPTO_KEY_INFO_MAX_SZ			144
#define	CRYPTO_CREATE_KEY_STATUS_MAX_SZ		1
#define	CRYPTO_PROVISION_DATA_MAX_SZ		1024
#define	FCS_KDK_MAX_SZ				384
#define	FCS_DIGEST_STAGE_INIT			0
#define	FCS_DIGEST_STAGE_UPDATE			1
#define	FCS_DIGEST_STAGE_FINAL			2

#define SDOS_HEADER_SZ		40
#define SDOS_HMAC_SZ		48
#define SDOS_MAGIC_WORD		0xACBDBDED
#define SDOS_HEADER_PADDING	0x01020304
#define SDOS_PLAINDATA_MIN_SZ	32
#define SDOS_PLAINDATA_MAX_SZ	32672
#define SDOS_DECRYPTED_MIN_SZ	(SDOS_PLAINDATA_MIN_SZ + SDOS_HEADER_SZ)
#define SDOS_DECRYPTED_MAX_SZ	(SDOS_PLAINDATA_MAX_SZ + SDOS_HEADER_SZ)
#define SDOS_ENCRYPTED_MIN_SZ	(SDOS_PLAINDATA_MIN_SZ + SDOS_HEADER_SZ + SDOS_HMAC_SZ)
#define SDOS_ENCRYPTED_MAX_SZ	(SDOS_PLAINDATA_MAX_SZ + SDOS_HEADER_SZ + SDOS_HMAC_SZ)

extern struct socfpga_fcs_priv *priv;

#pragma pack(push, 1)
struct fcs_cmd_context {
	/* Error status variable address */
	FCS_HAL_INT *error_code_addr;
	union {
		struct {
			/* Session id */
			FCS_HAL_CHAR *suuid;
			FCS_HAL_UINT *suuid_len;
		} open_session;

		struct {
			/* Session id */
			FCS_HAL_UUID suuid;
		} close_session;

		struct {
			/* Session id */
			FCS_HAL_UUID suuid;
			FCS_HAL_CHAR *key;
			FCS_HAL_UINT key_len;
			FCS_HAL_CHAR *status;
			FCS_HAL_UINT *status_len;
		} import_key;

		struct {
			/* Session id */
			FCS_HAL_UUID suuid;
			FCS_HAL_U32 key_id;
			FCS_HAL_CHAR *key;
			FCS_HAL_UINT *key_len;
		} export_key;

		struct {
			/* Session id */
			FCS_HAL_UUID suuid;
			FCS_HAL_U32 key_id;
		} remove_key;

		struct {
			/* Session id */
			FCS_HAL_UUID suuid;
			/* random number size */
			FCS_HAL_U32 key_id;
			FCS_HAL_CHAR *info;
			FCS_HAL_UINT *info_len;
		} key_info;

		struct {
			/* Session id */
			FCS_HAL_UUID suuid;
			FCS_HAL_CHAR *key;
			FCS_HAL_UINT key_len;
			FCS_HAL_CHAR *status;
			FCS_HAL_UINT *status_len;
		} create_key;

		struct {
			/* Session id */
			FCS_HAL_UUID suuid;
			FCS_HAL_U32 key_id;
			FCS_HAL_U32 step_type;
			FCS_HAL_U32 mac_mode;
			FCS_HAL_CHAR *ikm;
			FCS_HAL_U32 ikm_len;
			FCS_HAL_CHAR *info;
			FCS_HAL_U32 info_len;
			FCS_HAL_CHAR *output_key_obj;
			FCS_HAL_U32 output_key_obj_len;
			FCS_HAL_U32 *hkdf_resp;
		} hkdf_req;

		struct {
			FCS_HAL_CHAR *data;
			FCS_HAL_U32 *data_len;
		} prov_data;

		struct {
			FCS_HAL_U32 cache;
			FCS_HAL_CHAR *ccert;
			FCS_HAL_U32 ccert_len;
			FCS_HAL_CHAR *status;
			FCS_HAL_UINT *status_len;
		} ctr_set;

		struct {
			FCS_HAL_U32 ctr_type;
			FCS_HAL_U32 ctr_val;
			FCS_HAL_INT test;
		} ctr_set_preauth;

		struct {
			/* Session id */
			FCS_HAL_UUID suuid;
			/* context id */
			FCS_HAL_U32 context_id;
			FCS_HAL_CHAR *rng;
			FCS_HAL_U32 rng_len;
		} rng;

		struct {
			/* Session id */
			FCS_HAL_UUID suuid;
			/* context id */
			FCS_HAL_U32 context_id;
			FCS_HAL_U32 key_id;
			FCS_HAL_U32 sha_op_mode;
			FCS_HAL_U32 sha_digest_sz;
			FCS_HAL_CHAR *src;
			FCS_HAL_U32 src_len;
			FCS_HAL_CHAR *digest;
			FCS_HAL_U32 *digest_len;
			FCS_HAL_UINT stage;
		} dgst;

		struct {
			/* Session id */
			FCS_HAL_UUID suuid;
			/* context id */
			FCS_HAL_U32 context_id;
			FCS_HAL_U32 key_id;
			FCS_HAL_U32 sha_op_mode;
			FCS_HAL_U32 sha_digest_sz;
			FCS_HAL_CHAR *src;
			FCS_HAL_U32 src_size;
			FCS_HAL_CHAR *dst;
			FCS_HAL_U32 *dst_size;
			FCS_HAL_U32 user_data_size;
		} mac_verify;

		struct {
			FCS_HAL_UUID suuid;
			FCS_HAL_UINT cid; /* Context ID */
			FCS_HAL_UINT kid; /* Key ID */
			FCS_HAL_U8 mode; /* ECB/CBS/CTR */
			FCS_HAL_U8 crypt; /* Encrypt/Decrypt */
			FCS_HAL_U32 aad_len; /* AAD Length */
			FCS_HAL_U16 tag_len; /* Tag length */
			FCS_HAL_U8 iv_source; /* IV source External/Internal */
			FCS_HAL_CHAR *iv; /* IV */
			FCS_HAL_CHAR *aad; /* AAD */
			FCS_HAL_CHAR *tag; /* Tag */
			FCS_HAL_CHAR *input; /* Input data */
			FCS_HAL_UINT ip_len; /* Input Length */
			FCS_HAL_CHAR *output; /* Output data */
			FCS_HAL_UINT *op_len; /* Output Length */
			FCS_HAL_UINT input_pad; /* Source data padding (only GCM mode) */
		} aes;

		struct {
			FCS_HAL_UUID suuid;
			FCS_HAL_U32 kid;
			FCS_HAL_U32 cid;
			FCS_HAL_U32 ecc_curve;
			FCS_HAL_CHAR *pubkey;
			FCS_HAL_U32 pubkey_len;
			FCS_HAL_CHAR *sh_secret;
			FCS_HAL_U32 *sh_secret_len;
		} ecdh_req;

		struct {
			FCS_HAL_U32 *chip_id_lo;
			FCS_HAL_U32 *chip_id_hi;
		} chip_id;

		struct {
			FCS_HAL_INT cert_request;
			FCS_HAL_CHAR *cert;
			FCS_HAL_INT *cert_size;
		} attestation_cert;

		struct {
			FCS_HAL_INT cert_request;
		} attestation_cert_reload;

		struct {
			FCS_HAL_U32 mbox_cmd;
			FCS_HAL_U8 urgent;
			FCS_HAL_VOID *cmd_data;
			FCS_HAL_U32 cmd_data_sz;
			FCS_HAL_VOID *resp_data;
			FCS_HAL_U32 *resp_data_sz;
		} mbox;

		struct {
			FCS_HAL_CHAR *mctp_req;
			FCS_HAL_U32 mctp_req_len;
			FCS_HAL_CHAR *mctp_resp;
			FCS_HAL_U32 *mctp_resp_len;
		} mctp;

		struct {
			FCS_HAL_U32 *jtag_idcode;
		} jtag_id;

		struct {
			FCS_HAL_CHAR *identity;
			FCS_HAL_U32 *identity_len;
		} device_identity;

		struct {
			FCS_HAL_U32 chipsel;
		} qspi_cs;

		struct {
			FCS_HAL_U32 qspi_addr;
			FCS_HAL_U32 qspi_len;
			FCS_HAL_CHAR *qspi_data;
			FCS_HAL_U32 *qspi_data_len;
		} qspi_read, qspi_write;

		struct {
			FCS_HAL_U32 qspi_addr;
			FCS_HAL_U32 len;
		} qspi_erase;

		struct {
			FCS_HAL_VOID *qspi_info;
			FCS_HAL_U32 qspi_info_len;
		} qspi_dev_info;

		struct {
			/* Session id */
			FCS_HAL_UUID suuid;
			/* context id */
			FCS_HAL_U32 context_id;
			FCS_HAL_U32 op_mode;
			FCS_HAL_CHAR *src;
			FCS_HAL_U32 src_size;
			FCS_HAL_CHAR *dst;
			FCS_HAL_U32 *dst_size;
			FCS_HAL_U16 id;
			FCS_HAL_U64 own;
			FCS_HAL_INT pad;
		} sdos;

		struct {
			FCS_HAL_UUID suuid;
			FCS_HAL_U32 context_id;
			FCS_HAL_U32 key_id;
			FCS_HAL_U32 ecc_curve;
			FCS_HAL_CHAR *pubkey;
			FCS_HAL_U32 *pubkey_len;
		} ecdsa_pub_key;

		struct {
			FCS_HAL_UUID suuid;
			FCS_HAL_U32 context_id;
			FCS_HAL_U32 key_id;
			FCS_HAL_U32 ecc_curve;
			FCS_HAL_CHAR *src;
			FCS_HAL_U32 src_len;
			FCS_HAL_CHAR *dst;
			FCS_HAL_U32 *dst_len;
		} ecdsa_hash_sign;

		struct {
			FCS_HAL_UUID suuid;
			FCS_HAL_U32 context_id;
			FCS_HAL_U32 key_id;
			FCS_HAL_U32 ecc_curve;
			FCS_HAL_CHAR *src;
			FCS_HAL_U32 src_len;
			FCS_HAL_CHAR *signature;
			FCS_HAL_U32 signature_len;
			FCS_HAL_CHAR *pubkey;
			FCS_HAL_U32 pubkey_len;
			FCS_HAL_CHAR *dst;
			FCS_HAL_U32 *dst_len;
		} ecdsa_hash_verify;

		struct {
			FCS_HAL_UUID suuid;
			FCS_HAL_U32 context_id;
			FCS_HAL_U32 key_id;
			FCS_HAL_U32 ecc_curve;
			FCS_HAL_CHAR *src;
			FCS_HAL_U32 src_len;
			FCS_HAL_CHAR *dst;
			FCS_HAL_U32 *dst_len;
		} ecdsa_sha2_data_sign;

		struct {
			FCS_HAL_UUID suuid;
			FCS_HAL_U32 context_id;
			FCS_HAL_U32 key_id;
			FCS_HAL_U32 ecc_curve;
			FCS_HAL_CHAR *signature;
			FCS_HAL_U32 signature_len;
			FCS_HAL_CHAR *pubkey;
			FCS_HAL_U32 pubkey_len;
			FCS_HAL_U32 user_data_sz;
			FCS_HAL_CHAR *src;
			FCS_HAL_U32 src_len;
			FCS_HAL_CHAR *dst;
			FCS_HAL_U32 *dst_len;
		} ecdsa_sha2_data_verify;

		/* This command sends the certificate to the device requesting validation
		 * of an HPS image
		 */
		struct {
			FCS_HAL_CHAR *vab_cert;
			FCS_HAL_U32 vab_cert_len;
			FCS_HAL_U32 test;
			FCS_HAL_U32 *resp;
		} hps_img_validate;
	};
};

#pragma pack(pop)

/**
 * @brief data struct of message which stands for the communication
 *  format with ATF when talk with OS dependent layer API
 */
struct socfpga_fcs_priv {
	/** Communication channel */
	FCS_HAL_CHAN *chan;
	/** plat data */
	struct socfpga_fcs_service_ops *plat_data;
	/* command context */
	struct fcs_cmd_context k_ctx;
	/** cli structure */
	FCS_SVC_CLIENT client;
	/** Completion status */
	FCS_HAL_COMPLETION completion;
	/** Mutex lock */
	FCS_HAL_MUTEX lock;
	/** status */
	FCS_HAL_INT status;
	/** response */
	FCS_HAL_U32 resp;
	/** Size */
	FCS_HAL_U32 resp_size;
	/** chip ID */
	FCS_HAL_U32 chip_id_lo;
	FCS_HAL_U32 chip_id_hi;
	/* Session ID */
	FCS_HAL_U32 session_id;
	/** UUID */
	FCS_HAL_UUID uuid_id;
	/** Client ID */
	FCS_HAL_U32 client_id;
	/** Hardware RNG */
	FCS_HAL_VOID *hwrng;
	/** device to issue command */
	FCS_HAL_DEV *dev;
	/** ATF version */
	FCS_HAL_U32 atf_version[3];
	/** Reserved */
	FCS_HAL_VOID *preserved;
};

enum fcs_command_code {
	FCS_DEV_COMMAND_NONE = 0,
	FCS_DEV_CERTIFICATE,
	FCS_DEV_HPS_IMG_VALIDATE_REQUEST,
	FCS_DEV_HPS_IMG_VALIDATE_POLL_SERVICE,
	FCS_DEV_COUNTER_SET,
	FCS_DEV_COUNTER_SET_POLL_SERVICE,
	FCS_DEV_COUNTER_SET_PREAUTHORIZED,
	FCS_DEV_GET_PROVISION_DATA,
	FCS_DEV_GET_PROVISION_DATA_POLL_SERVICE,
	FCS_DEV_DATA_ENCRYPTION,
	FCS_DEV_DATA_DECRYPTION,
	FCS_DEV_PSGSIGMA_TEARDOWN,
	FCS_DEV_CHIP_ID,
	FCS_DEV_ATTESTATION_SUBKEY,
	FCS_DEV_ATTESTATION_MEASUREMENT,
	FCS_DEV_ATTESTATION_GET_CERTIFICATE,
	FCS_DEV_ATTESTATION_CERTIFICATE_RELOAD,
	FCS_DEV_GET_ROM_PATCH_SHA384,
	FCS_DEV_CRYPTO_OPEN_SESSION,
	FCS_DEV_CRYPTO_CLOSE_SESSION,
	FCS_DEV_CRYPTO_IMPORT_KEY,
	FCS_DEV_IMPORT_KEY_POLL_SERVICE,
	FCS_DEV_CRYPTO_EXPORT_KEY,
	FCS_DEV_CRYPTO_REMOVE_KEY,
	FCS_DEV_CRYPTO_GET_KEY_INFO,
	FCS_DEV_CRYPTO_CREATE_KEY,
	FCS_DEV_CRYPTO_CREATE_KEY_POLL_SERVICE,
	FCS_DEV_CRYPTO_AES_CRYPT,
	FCS_DEV_CRYPTO_GET_DIGEST_INIT,
	FCS_DEV_CRYPTO_GET_DIGEST_UPDATE,
	FCS_DEV_CRYPTO_GET_DIGEST_FINAL,
	FCS_DEV_CRYPTO_MAC_VERIFY_INIT,
	FCS_DEV_CRYPTO_MAC_VERIFY_UPDATE,
	FCS_DEV_CRYPTO_MAC_VERIFY_FINAL,
	FCS_DEV_CRYPTO_AES_CRYPT_INIT,
	FCS_DEV_CRYPTO_AES_CRYPT_UPDATE,
	FCS_DEV_CRYPTO_AES_CRYPT_FINAL,
	FCS_DEV_CRYPTO_AES_CRYPT_POLL_SERVICE,
	FCS_DEV_CRYPTO_GET_DIGEST,
	FCS_DEV_CRYPTO_MAC_VERIFY,
	FCS_DEV_CRYPTO_ECDSA_HASH_SIGNING,
	FCS_DEV_CRYPTO_ECDSA_HASH_SIGNING_INIT,
	FCS_DEV_CRYPTO_ECDSA_HASH_SIGNING_FINALIZE,
	FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_SIGNING,
	FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_SIGNING_INIT,
	FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_SIGNING_UPDATE,
	FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_SIGNING_FINALIZE,
	FCS_DEV_CRYPTO_ECDSA_HASH_VERIFY,
	FCS_DEV_CRYPTO_ECDSA_HASH_VERIFY_INIT,
	FCS_DEV_CRYPTO_ECDSA_HASH_VERIFY_FINALIZE,
	FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_VERIFY,
	FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_VERIFY_INIT,
	FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_VERIFY_UPDATE,
	FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_VERIFY_FINALIZE,
	FCS_DEV_CRYPTO_ECDSA_GET_PUBLIC_KEY_INIT,
	FCS_DEV_CRYPTO_ECDSA_GET_PUBLIC_KEY_FINALIZE,
	FCS_DEV_CRYPTO_ECDSA_GET_PUBLIC_KEY,
	FCS_DEV_CRYPTO_ECDH_REQUEST_INIT,
	FCS_DEV_CRYPTO_ECDH_REQUEST_FINALIZE,
	FCS_DEV_CRYPTO_HKDF_REQUEST,
	FCS_DEV_RANDOM_NUMBER_GEN,
	FCS_DEV_RNG_ASYNC_POLL_SERVICE,
	FCS_DEV_SDOS_DATA_EXT,
	FCS_DEV_CRYPTO_AES_CRYPT_SMMU,
	FCS_DEV_CRYPTO_GET_DIGEST_SMMU,
	FCS_DEV_CRYPTO_MAC_VERIFY_SMMU,
	FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_SIGNING_SMMU,
	FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_VERIFY_SMMU,
	FCS_DEV_CHECK_SMMU_ENABLED,
	FCS_DEV_MCTP_REQUEST,
	FCS_DEV_GET_IDCODE,
	FCS_DEV_GET_DEVICE_IDENTITY,
	FCS_DEV_QSPI_OPEN,
	FCS_DEV_QSPI_CLOSE,
	FCS_DEV_QSPI_CS,
	FCS_DEV_QSPI_READ,
	FCS_DEV_QSPI_WRITE,
	FCS_DEV_QSPI_ERASE,
	FCS_DEV_ATF_VERSION
};

/**
 * @brief Gets the FCS command context.
 *
 * This function gets the FCS command context.
 *
 * @return Returns a pointer to the FCS command context.
 */
struct fcs_cmd_context *hal_get_fcs_cmd_ctx(void);

/**
 * @brief Destroys the FCS command context.
 *
 * This function creates the FCS command context.
 *
 * @return Returns a pointer to the FCS command context.
 */
FCS_HAL_VOID hal_destroy_fcs_cmd_ctx(struct fcs_cmd_context *const k_ctx);

/**
 * @brief Releases the FCS command context.
 *
 * This function releases the FCS command context.
 *
 * @param k_ctx A pointer to the command context structure.
 */
FCS_HAL_VOID hal_release_fcs_cmd_ctx(struct fcs_cmd_context *const k_ctx);

/**
 * @brief Initializes the FCS HAL.
 *
 * This function initializes the FCS HAL and performs any necessary setup.
 *
 * @return Returns an FCS_HAL_INT value indicating the status of the initialization.
 */
FCS_HAL_INT hal_fcs_init(struct device *dev);

/**
 * @brief Cleans up the FCS HAL.
 *
 * This function cleans up the FCS HAL and performs any necessary cleanup.
 *
 * @return Returns an FCS_HAL_INT value indicating the status of the cleanup.
 */

FCS_HAL_VOID hal_fcs_cleanup(void);

/**
 * @brief Requests SDM to open a session.
 *
 * This function is used to request the SDM (System Device Manager) to open a session.
 *
 * @param ctx A pointer to the command context structure.
 * @return Returns 0 if the session is opened successfully, otherwise returns an error code.
 */
FCS_HAL_INT hal_session_open(struct fcs_cmd_context *const ctx);

/**
 * @brief Requests to SDM for closing a given opened session.
 *
 * This function is used to request the SDM (System Device Manager) to close a
 * previously opened session.
 *
 * @param ctx A pointer to the command context structure.
 * @return Returns 0 if the session is closed successfully, otherwise returns an error code.
 */
FCS_HAL_INT hal_session_close(struct fcs_cmd_context *const ctx);

/**
 * @brief Requests to get CHIP ID from SDM.
 *
 * This function is used to request the SDM (System Device Manager) to get the
 * CHIP ID.
 *
 * @param ctx A pointer to the command context structure.
 * @return Returns 0 if the CHIP ID is retrieved successfully, otherwise returns an error code.
 */
FCS_HAL_INT hal_get_chip_id(struct fcs_cmd_context *const ctx);
/**
 * @brief Requests to generate a random number from SDM.
 *
 * @param ctx A pointer to the command context structure.
 * @return 0 if the random number is generated successfully, otherwise an error code.
 */
FCS_HAL_INT hal_random_number(struct fcs_cmd_context *const ctx);

/**
 * @brief Stores the context information.
 *
 * @param ctx A pointer to the command context structure.
 * @return 0 if the context is stored successfully, otherwise an error code.
 */
FCS_HAL_INT hal_store_context(struct fcs_cmd_context *const ctx);

/**
 * Imports a key into the device.
 *
 * @param ctx A pointer to the command context structure.
 * @return An integer indicating the success or failure of the key import operation.
 */
FCS_HAL_INT hal_import_key(struct fcs_cmd_context *const ctx);

/**
 * @brief Retrieves the version of the ATF (Arm Trusted Firmware).
 *
 * This function fetches the current version of the ATF and stores it in the
 * provided version pointer.
 *
 * @param[out] version Pointer to a variable where the ATF version will be stored.
 *
 * @return void
 */
FCS_HAL_VOID hal_get_atf_version(FCS_HAL_U32 *version);

/**
 * Exports a key from the device.
 *
 * @param ctx A pointer to the command context structure.
 * @return An integer indicating the success or failure of the key export operation.
 */
FCS_HAL_INT hal_export_key(struct fcs_cmd_context *const ctx);

/**
 * Removes an imported key.
 *
 * @param ctx A pointer to the command context structure.
 * @return An integer indicating the success or failure of the key removal operation.
 */
FCS_HAL_INT hal_remove_key(struct fcs_cmd_context *const ctx);

/**
 * Gets the key information of imported key from the device.
 *
 * @param ctx A pointer to the command context structure.
 * @return An integer indicating the success or failure of the key information retrieval operation.
 */
FCS_HAL_INT hal_get_key_info(struct fcs_cmd_context *const ctx);

/**
 * Creates a key in the device.
 *
 * @param ctx A pointer to the command context structure.
 * @return An integer indicating the success or failure of the key creation operation.
 */
FCS_HAL_INT hal_create_key(struct fcs_cmd_context *const ctx);

/**
 * Requests the SDM to perform HKDF operation.
 *
 * @param ctx A pointer to the command context structure.
 * @return An integer indicating the success or failure of the HKDF operation.
 */
FCS_HAL_INT hal_hkdf_request(struct fcs_cmd_context *const ctx);

/**
 * Requests the SDM to get the provision data.
 *
 * @param ctx A pointer to the command context structure.
 * @return An integer indicating the success or failure of the provision data retrieval operation.
 */
FCS_HAL_INT hal_get_provision_data(struct fcs_cmd_context *const ctx);

/**
 * Sets the counter value.
 *
 * @param ctx A pointer to the command context structure.
 * @return An integer indicating the success or failure of the counter value setting operation.
 */
FCS_HAL_INT hal_counter_set(struct fcs_cmd_context *const ctx);

/**
 * Sets the preauthorized counter value.
 *
 * @param ctx A pointer to the command context structure.
 * @return An integer indicating the success or failure of the operation.
 */
FCS_HAL_INT hal_counter_set_preauth(struct fcs_cmd_context *const ctx);

/**
 * @brief Computes the digest for the given command context.
 *
 * This function calculates the digest based on the provided command context.
 *
 * @param k_ctx Pointer to the command context structure.
 * @return FCS_HAL_INT Result of the digest computation.
 */
FCS_HAL_INT hal_digest(struct fcs_cmd_context *const k_ctx);

/**
 * hal_digest_free_resource - Frees resources associated with the digest operation.
 * @k_ctx: Pointer to the FCS command context structure.
 *
 * This function releases any resources that were allocated for the digest operation
 * in the given FCS command context.
 *
 * @param k_ctx Pointer to the command context structure.
 *
 * Return:
 * None
 */
FCS_HAL_VOID hal_digest_free_resource(struct fcs_cmd_context *const k_ctx);

/**
 * @brief Verifies the MAC for the given command context.
 *
 * This function verifies the MAC (Message Authentication Code) based on the
 * provided command context.
 *
 * @param k_ctx Pointer to the command context structure.
 * @return FCS_HAL_INT Result of the MAC verification.
 */
FCS_HAL_INT hal_mac_verify(struct fcs_cmd_context *const k_ctx);

/**
 * Requests the SDM to perform AES encryption/decryption operation.
 *
 * @param ctx A pointer to the command context structure.
 * @return An integer indicating the success or failure of the AES encryption/decryption operation.
 */
FCS_HAL_INT hal_aes_crypt(struct fcs_cmd_context *const ctx);

/**
 * Requests the SDM to perform AES encryption/decryption operation.
 *
 * @param ctx A pointer to the command context structure.
 * @return An integer indicating the success or failure of the AES encryption/decryption operation.
 */
FCS_HAL_INT hal_ecdh_req(struct fcs_cmd_context *const ctx);

/**
 * Gets the chip ID.
 *
 * @param ctx A pointer to the command context structure.
 * @return An integer indicating the success or failure of the chip ID retrieval operation.
 */
FCS_HAL_INT hal_get_chip_id(struct fcs_cmd_context *const k_ctx);

/**
 * Retrieves the attestation certificate.
 *
 * @param ctx A pointer to the command context structure.
 * @return An integer indicating the success or failure of the attestation
 * certificate retrieval operation.
 */
FCS_HAL_INT hal_attestation_get_certificate(struct fcs_cmd_context *const ctx);

/**
 * Reloads the attestation certificate.
 *
 * @param ctx A pointer to the command context structure.
 * @return An integer indicating the success or failure of the attestation
 * certificate reload operation.
 */
FCS_HAL_INT hal_attestation_certificate_reload(struct fcs_cmd_context *const ctx);

/**
 * Requests the SDM to perform an MCTP operation.
 *
 * @param ctx A pointer to the command context structure.
 * @return An integer indicating the success or failure of the MCTP operation.
 */
FCS_HAL_INT hal_mctp_request(struct fcs_cmd_context *const k_ctx);

/**
 * requests access to the qspi interface
 *
 * @param ctx A pointer to the command context structure.
 * @return An integer indicating the success or failure to get access to qspi interface.
 */
FCS_HAL_INT hal_qspi_open(struct fcs_cmd_context *const ctx);

/**
 * requests to close the qspi interface
 *
 * @param ctx A pointer to the command context structure.
 * @return An integer indicating the success or failure to close the qspi interface.
 */
FCS_HAL_INT hal_qspi_close(struct fcs_cmd_context *const ctx);

/**
 * requests to select the qspi device
 *
 * @param ctx A pointer to the command context structure.
 * @return An integer indicating the success or failure to select the qspi device.
 */
FCS_HAL_INT hal_qspi_cs(struct fcs_cmd_context *const ctx);

/**
 * requests to read from the qspi device
 *
 * @param ctx A pointer to the command context structure.
 * @return An integer indicating the success or failure to read from the qspi device.
 */
FCS_HAL_INT hal_qspi_read(struct fcs_cmd_context *const ctx);

/**
 * requests to write to the qspi device
 *
 * @param ctx A pointer to the command context structure.
 * @return An integer indicating the success or failure to write to the qspi device.
 */
FCS_HAL_INT hal_qspi_write(struct fcs_cmd_context *const ctx);

/**
 * requests to erase the qspi device
 *
 * @param ctx A pointer to the command context structure.
 * @return An integer indicating the success or failure to erase the qspi device.
 */
FCS_HAL_INT hal_qspi_erase(struct fcs_cmd_context *const ctx);

/**
 * requests to get the jtag idcode
 *
 * @param ctx A pointer to the command context structure.
 * @return An integer indicating the success or failure to get the jtag idcode.
 */
FCS_HAL_INT hal_jtag_idcode(struct fcs_cmd_context *const ctx);

/**
 * requests to get the device identity
 *
 * @param ctx A pointer to the command context structure.
 * @return An integer indicating the success or failure to get the device identity.
 */
FCS_HAL_INT hal_get_device_identity(struct fcs_cmd_context *const ctx);

/**
 * @brief Data Encrypts/Decrypt  using SDOS (Secure Data Object Storage).
 *
 * This function encrypts/decrypts data based on the provided command context using SDOS.
 *
 * @param k_ctx Pointer to the command context structure.
 * @return FCS_HAL_INT Result of the encryption operation.
 */
FCS_HAL_INT hal_sdos_crypt(struct fcs_cmd_context *const k_ctx);

/**
 * @brief Retrieves the ECDSA public key.
 *
 * This function retrieves the ECDSA public key from the given command context.
 *
 * @param ctx Pointer to the command context structure.
 * @return FCS_HAL_INT Status code indicating the result of the operation.
 */
FCS_HAL_INT hal_ecdsa_get_pubkey(struct fcs_cmd_context *const ctx);

/**
 * @brief Signs the hash using ECDSA.
 *
 * This function signs the hash using ECDSA based on the provided command context.
 *
 * @param ctx Pointer to the command context structure.
 * @return FCS_HAL_INT Result of the hash signing operation.
 */
FCS_HAL_INT hal_ecdsa_hash_sign(struct fcs_cmd_context *const ctx);

/**
 * @brief Verifies the hash using ECDSA.
 *
 * This function verifies the hash using ECDSA based on the provided command context.
 *
 * @param ctx Pointer to the command context structure.
 * @return FCS_HAL_INT Result of the hash verification operation.
 */
FCS_HAL_INT hal_ecdsa_hash_verify(struct fcs_cmd_context *const ctx);

/**
 * @brief Signs the data using ECDSA.
 *
 * This function signs the data using ECDSA based on the provided command context.
 *
 * @param ctx Pointer to the command context structure.
 * @return FCS_HAL_INT Result of the data signing operation.
 */
FCS_HAL_INT hal_ecdsa_sha2_data_sign(struct fcs_cmd_context *const ctx);

/**
 * @brief Verifies the data using ECDSA.
 *
 * This function verifies the data using ECDSA based on the provided command context.
 *
 * @param ctx Pointer to the command context structure.
 * @return FCS_HAL_INT Result of the data verification operation.
 */
FCS_HAL_INT hal_ecdsa_sha2_data_verify(struct fcs_cmd_context *const ctx);

/**
 * Requests the SDM to validate an HPS image.
 *
 * @param ctx A pointer to the command context structure.
 * @return An integer indicating the success or failure of the HPS image validation operation.
 */

FCS_HAL_INT hal_hps_img_validate(struct fcs_cmd_context *const ctx);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SPCFPGA_FCS_HAL_H */
