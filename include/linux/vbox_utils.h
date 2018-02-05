/* SPDX-License-Identifier: (GPL-2.0 OR CDDL-1.0) */
/* Copyright (C) 2006-2016 Oracle Corporation */

#ifndef __VBOX_UTILS_H__
#define __VBOX_UTILS_H__

#include <linux/printk.h>
#include <linux/vbox_vmmdev_types.h>

struct vbg_dev;

/**
 * vboxguest logging functions, these log both to the backdoor and call
 * the equivalent kernel pr_foo function.
 */
__printf(1, 2) void vbg_info(const char *fmt, ...);
__printf(1, 2) void vbg_warn(const char *fmt, ...);
__printf(1, 2) void vbg_err(const char *fmt, ...);

/* Only use backdoor logging for non-dynamic debug builds */
#if defined(DEBUG) && !defined(CONFIG_DYNAMIC_DEBUG)
__printf(1, 2) void vbg_debug(const char *fmt, ...);
#else
#define vbg_debug pr_debug
#endif

/**
 * Allocate memory for generic request and initialize the request header.
 *
 * Return: the allocated memory
 * @len:		Size of memory block required for the request.
 * @req_type:		The generic request type.
 */
void *vbg_req_alloc(size_t len, enum vmmdev_request_type req_type);

/**
 * Perform a generic request.
 *
 * Return: VBox status code
 * @gdev:		The Guest extension device.
 * @req:		Pointer to the request structure.
 */
int vbg_req_perform(struct vbg_dev *gdev, void *req);

int vbg_hgcm_connect(struct vbg_dev *gdev,
		     struct vmmdev_hgcm_service_location *loc,
		     u32 *client_id, int *vbox_status);

int vbg_hgcm_disconnect(struct vbg_dev *gdev, u32 client_id, int *vbox_status);

int vbg_hgcm_call(struct vbg_dev *gdev, u32 client_id, u32 function,
		  u32 timeout_ms, struct vmmdev_hgcm_function_parameter *parms,
		  u32 parm_count, int *vbox_status);

int vbg_hgcm_call32(
	struct vbg_dev *gdev, u32 client_id, u32 function, u32 timeout_ms,
	struct vmmdev_hgcm_function_parameter32 *parm32, u32 parm_count,
	int *vbox_status);

/**
 * Convert a VirtualBox status code to a standard Linux kernel return value.
 * Return: 0 or negative errno value.
 * @rc:			VirtualBox status code to convert.
 */
int vbg_status_code_to_errno(int rc);

/**
 * Helper for the vboxsf driver to get a reference to the guest device.
 * Return: a pointer to the gdev; or a ERR_PTR value on error.
 */
struct vbg_dev *vbg_get_gdev(void);

/**
 * Helper for the vboxsf driver to put a guest device reference.
 * @gdev:		Reference returned by vbg_get_gdev to put.
 */
void vbg_put_gdev(struct vbg_dev *gdev);

#endif
