/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2017-2024, Intel Corporation
 */

#ifndef __STRATIX10_SVC_CLIENT_H
#define __STRATIX10_SVC_CLIENT_H

/*
 * Service layer driver supports client names
 *
 * fpga: for FPGA configuration
 * rsu: for remote status update
 * hwmon: for hardware monitoring (volatge and temperature)
 */
#define SVC_CLIENT_FPGA			"fpga"
#define SVC_CLIENT_RSU			"rsu"
#define SVC_CLIENT_FCS			"fcs"
#define SVC_CLIENT_HWMON		"hwmon"

/**
 * Status of the sent command, in bit number
 *
 * SVC_STATUS_OK:
 * Secure firmware accepts the request issued by one of service clients.
 *
 * SVC_STATUS_BUFFER_SUBMITTED:
 * Service client successfully submits data buffer to secure firmware.
 *
 * SVC_STATUS_BUFFER_DONE:
 * Secure firmware completes data process, ready to accept the
 * next WRITE transaction.
 *
 * SVC_STATUS_COMPLETED:
 * Secure firmware completes service request successfully. In case of
 * FPGA configuration, FPGA should be in user mode.
 *
 * SVC_COMMAND_STATUS_BUSY:
 * Service request is still in process.
 *
 * SVC_COMMAND_STATUS_ERROR:
 * Error encountered during the process of the service request.
 *
 * SVC_STATUS_NO_SUPPORT:
 * Secure firmware doesn't support requested features such as RSU retry
 * or RSU notify.
 */
#define SVC_STATUS_OK			0
#define SVC_STATUS_BUFFER_SUBMITTED	1
#define SVC_STATUS_BUFFER_DONE		2
#define SVC_STATUS_COMPLETED		3
#define SVC_STATUS_BUSY			4
#define SVC_STATUS_ERROR		5
#define SVC_STATUS_NO_SUPPORT		6
#define SVC_STATUS_INVALID_PARAM	7
#define SVC_STATUS_NO_RESPONSE		8
/**
 * Flag bit for COMMAND_RECONFIG
 *
 * COMMAND_RECONFIG_FLAG_PARTIAL:
 * Set to FPGA configuration type (full or partial).
 *
 * COMMAND_AUTHENTICATE_BITSTREAM:
 * Set for bitstream authentication, which makes sure a signed bitstream
 * has valid signatures before committing it to device.
 */
#define COMMAND_RECONFIG_FLAG_PARTIAL	0
#define COMMAND_AUTHENTICATE_BITSTREAM	1

/*
 * Timeout settings for service clients:
 * timeout value used in Stratix10 FPGA manager driver.
 * timeout value used in RSU driver
 */
#define SVC_RECONFIG_REQUEST_TIMEOUT_MS         5000
#define SVC_RECONFIG_BUFFER_TIMEOUT_MS          5000
#define SVC_RSU_REQUEST_TIMEOUT_MS              2000
#define SVC_FCS_REQUEST_TIMEOUT_MS		2000
#define SVC_COMPLETED_TIMEOUT_MS		30000
#define SVC_HWMON_REQUEST_TIMEOUT_MS		2000

struct stratix10_svc_chan;

/**
 * enum stratix10_svc_command_code - supported service commands
 *
 * @COMMAND_NOOP: do 'dummy' request for integration/debug/trouble-shooting
 *
 * @COMMAND_RECONFIG: ask for FPGA configuration preparation, return status
 * is SVC_STATUS_OK
 *
 * @COMMAND_RECONFIG_DATA_SUBMIT: submit buffer(s) of bit-stream data for the
 * FPGA configuration, return status is SVC_STATUS_SUBMITTED or SVC_STATUS_ERROR
 *
 * @COMMAND_RECONFIG_DATA_CLAIM: check the status of the configuration, return
 * status is SVC_STATUS_COMPLETED, or SVC_STATUS_BUSY, or SVC_STATUS_ERROR
 *
 * @COMMAND_RECONFIG_STATUS: check the status of the configuration, return
 * status is SVC_STATUS_COMPLETED, or SVC_STATUS_BUSY, or SVC_STATUS_ERROR
 *
 * @COMMAND_RSU_STATUS: request remote system update boot log, return status
 * is log data or SVC_STATUS_RSU_ERROR
 *
 * @COMMAND_RSU_UPDATE: set the offset of the bitstream to boot after reboot,
 * return status is SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_RSU_NOTIFY: report the status of hard processor system
 * software to firmware, return status is SVC_STATUS_OK or
 * SVC_STATUS_ERROR
 *
 * @COMMAND_RSU_RETRY: query firmware for the current image's retry counter,
 * return status is SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_RSU_MAX_RETRY: query firmware for the max retry value,
 * return status is SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_RSU_DCMF_VERSION: query firmware for the DCMF version, return status
 * is SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_POLL_SERVICE_STATUS: poll if the service request is complete,
 * return statis is SVC_STATUS_OK, SVC_STATUS_ERROR or SVC_STATUS_BUSY
 *
 * @COMMAND_FIRMWARE_VERSION: query running firmware version, return status
 * is SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_SMC_SVC_VERSION: Non-mailbox SMC SVC API Version,
 * return status is SVC_STATUS_OK
 *
 * @COMMAND_MBOX_SEND_CMD: send generic mailbox command, return status is
 * SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_RSU_DCMF_STATUS: query firmware for the DCMF status
 * return status is SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_RSU_GET_DEVICE_INFO: query firmware for QSPI device info
 * return status is SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_REQUEST_SERVICE: request validation of image from firmware,
 * return status is SVC_STATUS_OK, SVC_STATUS_INVALID_PARAM
 *
 * @COMMAND_FCS_SEND_CERTIFICATE: send a certificate, return status is
 * SVC_STATUS_OK, SVC_STATUS_INVALID_PARAM, SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_GET_PROVISION_DATA: read the provisioning data, return status is
 * SVC_STATUS_OK, SVC_STATUS_INVALID_PARAM, SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_DATA_ENCRYPTION: encrypt the data, return status is
 * SVC_STATUS_OK, SVC_STATUS_INVALID_PARAM, SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_DATA_DECRYPTION: decrypt the data, return status is
 * SVC_STATUS_OK, SVC_STATUS_INVALID_PARAM, SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_RANDOM_NUMBER_GEN: generate a random number, return status
 * is SVC_STATUS_OK, or SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_COUNTER_SET_PREAUTHORIZED: update the counter value for
 * the selected counter without the signed certificate, return status is
 * SVC_STATUS_OK, or SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_PSGSIGMA_TEARDOWN: tear down all previous black key
 * provision sessions and delete keys assicated with those sessions,
 * return status is SVC_STATUS_SUBMITTED or SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_GET_CHIP_ID: get the device's chip ID, return status is
 * SVC_STATUS_SUBMITTED or SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_ATTESTATION_SUBKEY: get device's attestation subkey,
 * return status is SVC_STATUS_SUBMITTED or SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_ATTESTATION_MEASUREMENTS: to get device's attestation
 * measurements, return status is SVC_STATUS_SUBMITTED or SVC_STATUS_ERROR
 *
 * @COMMAND_POLL_SERVICE_STATUS: poll if the service request is complete,
 * return statis is SVC_STATUS_OK, SVC_STATUS_ERROR or SVC_STATUS_BUSY
 *
 * @COMMAND_FCS_ATTESTATION_CERTIFICATE: get FPGA attestation certificate,
 * return status is SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_ATTESTATION_CERTIFICATE_RELOAD: reload FPGA attestation
 * certificate, return status is SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_GET_ROM_PATCH_SHA384: read the ROM patch SHA384 value,
 * return status is SVC_STATUS_OK, or SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_CRYPTO_OPEN_SESSION: open the crypto service session(s),
 * return status is SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_CRYPTO_CLOSE_SESSION: close the crypto service session(s),
 * return status is SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_CRYPTO_IMPORT_KEY: import the crypto service key object,
 * return status is SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_CRYPTO_EXPORT_KEY: export the crypto service key object,
 * return status is SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_CRYPTO_REMOVE_KEY: remove the crypto service key object
 * from the device, return status is SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_CRYPTO_GET_KEY_INFO: get the crypto service key object
 * info, return status is SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_CRYPTO_CREATE_KEY: create the crypto service key object,
 * return status is SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_CRYPTO_AES_CRYPT: sends request to encrypt or decrypt a
 * data block, return status is SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_CRYPTO_GET_DIGEST (INIT and FINALIZE): request the SHA-2
 * hash digest on a data block, return status is SVC_STATUS_OK or
 * SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_CRYPTO_MAC_VERIFY (INIT and FINALIZE): check the integrity
 * and authenticity of a blob, return status is SVC_STATUS_OK or
 * SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_CRYPTO_ECDSA_HASH_SIGNING (INIT and FINALIZE): send
 * digital signature signing request on a data blob, return status is
 * SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING (INIT and FINALIZE): send
 * SHA2 digital signature signing request on a data blob, return status is
 * SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_CRYPTO_ECDSA_HASH_VERIFY (INIT and FINALIZE): send
 * digital signature verify request with precalculated hash, return status is
 * SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY (INIT and FINALIZE): send digital
 * signature verify request, return status is SVC_STATUS_OK or
 * SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_CRYPTO_ECDSA_GET_PUBLIC_KEY (INIT and FINALIZE): send the
 * request to get the public key, return status is SVC_STATUS_OK or
 * SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_CRYPTO_ECDH_REQUEST (INIT and FINALIZE): send the request
 * on generating a share secret on Diffie-Hellman key exchange, return
 * status is SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_CRYPTO_HKDF_REQUEST: performs HKDF extract or expand with
 * input key and data, the output key will be placed in key vault, return
 * status is SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_RANDOM_NUMBER_GEN_EXT: extend random number generation,
 * return status is SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_SDOS_DATA_EXT: extend SDOS data encryption & decryption,
 * return status is SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_SMC_SVC_VERSION: Non-mailbox SMC SVC API Version,
 * return status is SVC_STATUS_OK
 *
 * @COMMAND_FCS_CRYPTO_GET_DEVICE_IDENTITY: send the request to get the
 * device identity, return status is SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_FCS_MCTP_SEND: send the MCTP message, return status is
 * SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_MBOX_SEND_CMD: send generic mailbox command, return status is
 * SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_GET_IDCODE: get the device's IDCODE, return status is
 * SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_QSPI_OPEN: open the QSPI proxy, return status is
 * SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_QSPI_CLOSE: close the QSPI proxy, return status is
 * SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_QSPI_SET_CS: set the QSPI proxy chip select, return status is
 * SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_QSPI_READ: read from the QSPI proxy, return status is
 * SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_QSPI_WRITE: write to the QSPI proxy, return status is
 * SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @COMMAND_QSPI_ERASE: erase the QSPI proxy, return status is
 * SVC_STATUS_OK or SVC_STATUS_ERROR
 *
 * @
 *
 */
enum stratix10_svc_command_code {
	/* for FPGA */
	COMMAND_NOOP = 0,
	COMMAND_RECONFIG,
	COMMAND_RECONFIG_DATA_SUBMIT,
	COMMAND_RECONFIG_DATA_CLAIM,
	COMMAND_RECONFIG_STATUS,
	/* for RSU */
	COMMAND_RSU_STATUS = 10,
	COMMAND_RSU_UPDATE,
	COMMAND_RSU_NOTIFY,
	COMMAND_RSU_RETRY,
	COMMAND_RSU_MAX_RETRY,
	COMMAND_RSU_DCMF_VERSION,
	COMMAND_RSU_DCMF_STATUS,
	COMMAND_RSU_GET_DEVICE_INFO,
	/* for FCS */
	COMMAND_FCS_REQUEST_SERVICE = 20,
	COMMAND_FCS_SEND_CERTIFICATE,
	COMMAND_FCS_GET_PROVISION_DATA,
	COMMAND_FCS_DATA_ENCRYPTION,
	COMMAND_FCS_DATA_DECRYPTION,
	COMMAND_FCS_RANDOM_NUMBER_GEN,
	COMMAND_FCS_COUNTER_SET_PREAUTHORIZED,
	COMMAND_FCS_GET_ROM_PATCH_SHA384,
	/* for Attestation */
	COMMAND_FCS_PSGSIGMA_TEARDOWN = 30,
	COMMAND_FCS_GET_CHIP_ID,
	COMMAND_FCS_ATTESTATION_SUBKEY,
	COMMAND_FCS_ATTESTATION_MEASUREMENTS,
	COMMAND_FCS_ATTESTATION_CERTIFICATE,
	COMMAND_FCS_ATTESTATION_CERTIFICATE_RELOAD,
	/* for general status poll */
	COMMAND_POLL_SERVICE_STATUS = 40,
	COMMAND_FIRMWARE_VERSION,
	COMMAND_POLL_SERVICE_STATUS_ASYNC,
	/* for crypto service */
	COMMAND_FCS_CRYPTO_OPEN_SESSION = 50,
	COMMAND_FCS_CRYPTO_CLOSE_SESSION,
	COMMAND_FCS_CRYPTO_IMPORT_KEY,
	COMMAND_FCS_CRYPTO_EXPORT_KEY,
	COMMAND_FCS_CRYPTO_REMOVE_KEY,
	COMMAND_FCS_CRYPTO_GET_KEY_INFO,
	COMMAND_FCS_CRYPTO_CREATE_KEY,
	COMMAND_FCS_CRYPTO_AES_CRYPT_INIT,
	COMMAND_FCS_CRYPTO_AES_CRYPT_UPDATE,
	COMMAND_FCS_CRYPTO_AES_CRYPT_FINALIZE,
	COMMAND_FCS_CRYPTO_GET_DIGEST_INIT,
	COMMAND_FCS_CRYPTO_GET_DIGEST_UPDATE,
	COMMAND_FCS_CRYPTO_GET_DIGEST_FINALIZE,
	COMMAND_FCS_CRYPTO_MAC_VERIFY_INIT,
	COMMAND_FCS_CRYPTO_MAC_VERIFY_UPDATE,
	COMMAND_FCS_CRYPTO_MAC_VERIFY_FINALIZE,
	COMMAND_FCS_CRYPTO_ECDSA_HASH_SIGNING_INIT,
	COMMAND_FCS_CRYPTO_ECDSA_HASH_SIGNING_FINALIZE,
	COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_INIT,
	COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_UPDATE,
	COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_FINALIZE,
	COMMAND_FCS_CRYPTO_ECDSA_HASH_VERIFY_INIT,
	COMMAND_FCS_CRYPTO_ECDSA_HASH_VERIFY_FINALIZE,
	COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_INIT,
	COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_UPDATE,
	COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_FINALIZE,
	COMMAND_FCS_CRYPTO_ECDSA_GET_PUBLIC_KEY_INIT,
	COMMAND_FCS_CRYPTO_ECDSA_GET_PUBLIC_KEY_FINALIZE,
	COMMAND_FCS_CRYPTO_ECDH_REQUEST_INIT,
	COMMAND_FCS_CRYPTO_ECDH_REQUEST_FINALIZE,
	COMMAND_FCS_CRYPTO_HKDF_REQUEST,
	COMMAND_FCS_RANDOM_NUMBER_GEN_EXT,
	COMMAND_FCS_SDOS_DATA_EXT,
	COMMAND_WRITE_TO_SECURE_REG,
	COMMAND_READ_SECURE_REG,
	COMMAND_FCS_CRYPTO_AES_CRYPT_UPDATE_SMMU,
	COMMAND_FCS_CRYPTO_AES_CRYPT_FINALIZE_SMMU,
	COMMAND_FCS_CRYPTO_GET_DIGEST_UPDATE_SMMU,
	COMMAND_FCS_CRYPTO_GET_DIGEST_FINALIZE_SMMU,
	COMMAND_FCS_CRYPTO_MAC_VERIFY_UPDATE_SMMU,
	COMMAND_FCS_CRYPTO_MAC_VERIFY_FINALIZE_SMMU,
	COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_UPDATE_SMMU,
	COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_FINALIZE_SMMU,
	COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_UPDATE_SMMU,
	COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_FINALIZE_SMMU,
	COMMAND_FCS_CRYPTO_GET_DEVICE_IDENTITY,
	COMMAND_FCS_MCTP_SEND,
	/* for generic mailbox send command */
	COMMAND_MBOX_SEND_CMD = 100,
	/* Non-mailbox SMC Call */
	COMMAND_SMC_SVC_VERSION = 200,
	/* for HWMON */
	COMMAND_HWMON_READTEMP,
	COMMAND_HWMON_READVOLT,
	/*for Device identity*/
	COMMAND_GET_IDCODE,
	/*for QSPI proxy commands via SDM*/
	COMMAND_QSPI_OPEN,
	COMMAND_QSPI_CLOSE,
	COMMAND_QSPI_SET_CS,
	COMMAND_QSPI_READ,
	COMMAND_QSPI_WRITE,
	COMMAND_QSPI_ERASE
};

/**
 * struct stratix10_svc_client_msg - message sent by client to service
 * @payload: starting address of data need be processed
 * @payload_length: to be processed data size in bytes
 * @payload_output: starting address of processed data
 * @payload_length_output: processed data size in bytes
 * @command: service command
 * @arg: args to be passed via registers and not physically mapped buffers
 */
struct stratix10_svc_client_msg {
	void *payload;
	size_t payload_length;
	void *payload_output;
	size_t payload_length_output;
	enum stratix10_svc_command_code command;
	u64 arg[6];
};

/**
 * struct stratix10_svc_command_config_type - config type
 * @flags: flag bit for the type of FPGA configuration
 */
struct stratix10_svc_command_config_type {
	u32 flags;
};

/**
 * struct stratix10_svc_cb_data - callback data structure from service layer
 * @status: the status of sent command
 * @kaddr1: address of 1st completed data block
 * @kaddr2: address of 2nd completed data block
 * @kaddr3: address of 3rd completed data block
 * @kaddr4: address of 4th completed data block
 */
struct stratix10_svc_cb_data {
	u32 status;
	void *kaddr1;
	void *kaddr2;
	void *kaddr3;
	void *kaddr4;
};

/**
 * struct stratix10_svc_client - service client structure
 * @dev: the client device
 * @receive_cb: callback to provide service client the received data
 * @priv: client private data
 */
struct stratix10_svc_client {
	struct device *dev;
	void (*receive_cb)(struct stratix10_svc_client *client,
			   struct stratix10_svc_cb_data *cb_data);
	void *priv;
};

/**
 * stratix10_svc_request_channel_byname() - request service channel
 * @client: identity of the client requesting the channel
 * @name: supporting client name defined above
 *
 * Return: a pointer to channel assigned to the client on success,
 * or ERR_PTR() on error.
 */
struct stratix10_svc_chan
*stratix10_svc_request_channel_byname(struct stratix10_svc_client *client,
	const char *name);

/**
 * stratix10_svc_free_channel() - free service channel.
 * @chan: service channel to be freed
 */
void stratix10_svc_free_channel(struct stratix10_svc_chan *chan);

/**
 * stratix10_svc_allocate_memory() - allocate the momory
 * @chan: service channel assigned to the client
 * @size: number of bytes client requests
 *
 * Service layer allocates the requested number of bytes from the memory
 * pool for the client.
 *
 * Return: the starting address of allocated memory on success, or
 * ERR_PTR() on error.
 */
void *stratix10_svc_allocate_memory(struct stratix10_svc_chan *chan,
				    size_t size);

/**
 * stratix10_svc_free_memory() - free allocated memory
 * @chan: service channel assigned to the client
 * @kaddr: starting address of memory to be free back to pool
 */
void stratix10_svc_free_memory(struct stratix10_svc_chan *chan, void *kaddr);

/**
 * stratix10_svc_send() - send a message to the remote
 * @chan: service channel assigned to the client
 * @msg: message data to be sent, in the format of
 * struct stratix10_svc_client_msg
 *
 * Return: 0 for success, -ENOMEM or -ENOBUFS on error.
 */
int stratix10_svc_send(struct stratix10_svc_chan *chan, void *msg);

/**
 * stratix10_svc_done() - complete service request
 * @chan: service channel assigned to the client
 *
 * This function is used by service client to inform service layer that
 * client's service requests are completed, or there is an error in the
 * request process.
 */
void stratix10_svc_done(struct stratix10_svc_chan *chan);

/**
 * @typedef async_callback_t
 * @brief A type definition for an asynchronous callback function.
 *
 * This type defines a function pointer for an asynchronous callback.
 * The callback function takes a single argument, which is a pointer to
 * user-defined data.
 *
 * @param cb_arg A pointer to user-defined data passed to the callback function.
 */
typedef void (*async_callback_t)(void *cb_arg);

/**
 * stratix10_svc_add_async_client - Add an asynchronous client to a Stratix 10
 *                                  service channel.
 * @chan: Pointer to the Stratix 10 service channel structure.
 * @use_unique_clientid: Boolean flag indicating whether to use a unique client ID.
 *
 * This function registers an asynchronous client with the specified Stratix 10
 * service channel. If the use_unique_clientid flag is set to true, a unique client
 * ID will be assigned to the client.
 *
 * Return: 0 on success, or a negative error code on failure:
 *         -EINVAL if the channel is NULL or the async controller is not initialized.
 *         -EALREADY if the async channel is already allocated.
 *         -ENOMEM if memory allocation fails.
 *         Other negative values if ID allocation fails
 */
int stratix10_svc_add_async_client(struct stratix10_svc_chan *chan, bool use_unique_clientid);

/**
 * stratix10_svc_remove_async_client - Remove an asynchronous client from the Stratix 10
 *                                     service channel.
 * @chan: Pointer to the Stratix 10 service channel structure.
 *
 * This function removes an asynchronous client from the specified Stratix 10 service channel.
 * It is typically used to clean up and release resources associated with the client.
 *
 * Return: 0 on success, -EINVAL if the channel or asynchronous channel is invalid.
 */
int stratix10_svc_remove_async_client(struct stratix10_svc_chan *chan);

/**
 * stratix10_svc_async_send - Send an asynchronous message to the SDM mailbox
 *                            in EL3 secure firmware.
 * @chan: Pointer to the service channel structure.
 * @msg: Pointer to the message to be sent.
 * @handler: Pointer to the handler object used by caller to track the transaction.
 * @cb: Callback function to be called upon completion.
 * @cb_arg: Argument to be passed to the callback function.
 *
 * This function sends a message asynchronously to the SDM mailbox in EL3 secure firmware.
 * and registers a callback function to be invoked when the operation completes.
 *
 * Return: 0 on success,and negative error codes on failure.
 */
int stratix10_svc_async_send(struct stratix10_svc_chan *chan, void *msg, void **handler,
			     async_callback_t cb, void *cb_arg);

/**
 * stratix10_svc_async_poll - Polls the status of an asynchronous service request.
 * @chan: Pointer to the service channel structure.
 * @tx_handle: Handle to the transaction being polled.
 * @data: Pointer to the callback data structure to be filled with the result.
 *
 * This function checks the status of an asynchronous service request
 * and fills the provided callback data structure with the result.
 *
 * Return: 0 on success, -EINVAL if any input parameter is invalid or if the
 *         async controller is not initialized, -EAGAIN if the transaction is
 *         still in progress, or other negative error codes on failure.
 */
int stratix10_svc_async_poll(struct stratix10_svc_chan *chan, void *tx_handle,
			     struct stratix10_svc_cb_data *data);

/**
 * stratix10_svc_async_done - Complete an asynchronous transaction
 * @chan: Pointer to the service channel structure
 * @tx_handle: Pointer to the transaction handle
 *
 * This function completes an asynchronous transaction by removing the
 * transaction from the hash table and deallocating the associated resources.
 *
 * Return: 0 on success, -EINVAL on invalid input or errors.
 */
int stratix10_svc_async_done(struct stratix10_svc_chan *chan, void *tx_handle);

#endif

