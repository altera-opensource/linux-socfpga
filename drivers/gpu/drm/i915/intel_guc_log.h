/*
 * Copyright © 2014-2017 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */

#ifndef _INTEL_GUC_LOG_H_
#define _INTEL_GUC_LOG_H_

#include <linux/workqueue.h>

#include "intel_guc_fwif.h"

struct drm_i915_private;
struct intel_guc;

struct intel_guc_log {
	u32 flags;
	struct i915_vma *vma;
	/* The runtime stuff gets created only when GuC logging gets enabled */
	struct {
		void *buf_addr;
		struct workqueue_struct *flush_wq;
		struct work_struct flush_work;
		struct rchan *relay_chan;
	} runtime;
	/* logging related stats */
	u32 capture_miss_count;
	u32 flush_interrupt_count;
	u32 prev_overflow_count[GUC_MAX_LOG_BUFFER];
	u32 total_overflow_count[GUC_MAX_LOG_BUFFER];
	u32 flush_count[GUC_MAX_LOG_BUFFER];
};

int intel_guc_log_create(struct intel_guc *guc);
void intel_guc_log_destroy(struct intel_guc *guc);
int i915_guc_log_control(struct drm_i915_private *dev_priv, u64 control_val);
void i915_guc_log_register(struct drm_i915_private *dev_priv);
void i915_guc_log_unregister(struct drm_i915_private *dev_priv);

#endif
