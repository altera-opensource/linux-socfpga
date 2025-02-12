/* SPDX-License-Identifier: GPL-2.0-or-later OR MIT */
/*
 * Copyright (C) 2025 Altera
 */

#ifndef SOCFPGA_FCS_PLAT_H_
#define SOCFPGA_FCS_PLAT_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define FCS_STATUS_OK			SVC_STATUS_OK
#define FCS_STATUS_BUFFER_SUBMITTED	SVC_STATUS_BUFFER_SUBMITTED
#define FCS_STATUS_BUFFER_DONE		SVC_STATUS_BUFFER_DONE
#define FCS_STATUS_COMPLETED		SVC_STATUS_COMPLETED
#define FCS_STATUS_BUSY			SVC_STATUS_BUSY
#define FCS_STATUS_ERROR		SVC_STATUS_ERROR
#define FCS_STATUS_NO_SUPPORT		SVC_STATUS_NO_SUPPORT
#define FCS_STATUS_INVALID_PARAM	SVC_STATUS_INVALID_PARAM
#define FCS_STATUS_NO_RESPONSE		SVC_STATUS_NO_RESPONSE
#define FCS_ASYNC_POLL_SERVICE		0x00004F4E

/**
 * struct socfpga_fcs_service_ops - Service operations for SoCFPGA FCS
 * @svc_send_request: Function pointer for sending a request
 * @svc_alloc_memory: Function pointer for allocating memory
 * @svc_free_memory: Function pointer for freeing allocated memory
 * @svc_task_done: Function pointer for marking a task as done
 *
 * This structure defines the service operations for the SoCFPGA FCS (FPGA
 * Crypto Service). Each member is a function pointer to the respective
 * operation required for handling FCS services.
 */
struct socfpga_fcs_service_ops {
	FCS_HAL_INT(*svc_send_request)
	(struct socfpga_fcs_priv *priv, enum fcs_command_code command,
	 FCS_HAL_ULONG timeout);
	FCS_HAL_VOID *(*svc_alloc_memory)(struct socfpga_fcs_priv *priv,
					  size_t size);
	FCS_HAL_VOID (*svc_free_memory)
	(struct socfpga_fcs_priv *priv, void *buf);
	FCS_HAL_VOID (*svc_task_done)(struct socfpga_fcs_priv *priv);
};

/**
 * @brief Function to complete the platform operation.
 *
 * This function is used to complete the platform operation by providing the
 * completion structure.
 *
 * @param completion Pointer to the completion structure.
 */
FCS_HAL_VOID fcs_plat_complete(FCS_HAL_COMPLETION *completion);

/**
 * @brief Reinitializes the given completion structure.
 *
 * This function reinitializes the completion structure pointed to by the
 * `completion` parameter. It is typically used to reset the state of the
 * completion structure so that it can be reused.
 *
 * @param completion Pointer to the completion structure to be reinitialized.
 *
 * @return FCS_HAL_VOID
 */
FCS_HAL_VOID fcs_plat_reinit_completion(FCS_HAL_COMPLETION *completion);

FCS_HAL_INT fcs_plat_dma_addr_map(struct socfpga_fcs_priv *priv,
				  FCS_HAL_DMA_ADDR *dma_handle,
				  FCS_HAL_VOID *buf, FCS_HAL_SIZE size,
				  FCS_HAL_UINT direction);
FCS_HAL_VOID fcs_plat_dma_addr_unmap(struct socfpga_fcs_priv *priv,
				     FCS_HAL_DMA_ADDR *dma_handle,
				     FCS_HAL_SIZE size, FCS_HAL_UINT direction);
FCS_HAL_INT fcs_plat_wait_for_completion(FCS_HAL_COMPLETION *completion,
					 FCS_HAL_ULONG timeout);
FCS_HAL_VOID fcs_plat_mutex_lock(struct socfpga_fcs_priv *priv);
FCS_HAL_VOID fcs_plat_mutex_unlock(struct socfpga_fcs_priv *priv);
FCS_HAL_VOID *fcs_plat_alloc_mem(FCS_HAL_SIZE size);
FCS_HAL_VOID fcs_plat_free_mem(FCS_HAL_VOID *ptr);
FCS_HAL_BOOL fcs_plat_uuid_compare(FCS_HAL_UUID *uuid1, FCS_HAL_UUID *uuid2);
FCS_HAL_VOID fcs_plat_uuid_copy(FCS_HAL_UUID *dst, FCS_HAL_UUID *src);
FCS_HAL_VOID fcs_plat_uuid_generate(struct socfpga_fcs_priv *priv);
FCS_HAL_VOID fcs_plat_free_svc_memory(struct socfpga_fcs_priv *priv, void *buf1,
				      void *buf2, void *buf3);
FCS_HAL_INT fcs_plat_init(struct device *dev, struct socfpga_fcs_priv *priv);
FCS_HAL_VOID fcs_plat_cleanup(struct socfpga_fcs_priv *priv);
FCS_HAL_VOID fcs_plat_uuid_clear(struct socfpga_fcs_priv *priv);
FCS_HAL_INT fcs_plat_copy_to_user(FCS_HAL_VOID *dst, FCS_HAL_VOID *src,
				  FCS_HAL_SIZE size);
FCS_HAL_INT fcs_plat_copy_from_user(FCS_HAL_VOID *dst, FCS_HAL_VOID *src,
				    FCS_HAL_SIZE size);
FCS_HAL_VOID fcs_plat_memcpy(FCS_HAL_VOID *dst, FCS_HAL_VOID *src,
			     FCS_HAL_SIZE size);
FCS_HAL_VOID fcs_plat_memset(FCS_HAL_VOID *dst, FCS_HAL_U8 val,
			     FCS_HAL_SIZE size);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
