/*
 * Copyright © 2016 Intel Corporation
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

#ifndef __I915_GEM_H__
#define __I915_GEM_H__

#include <linux/bug.h>

#ifdef CONFIG_DRM_I915_DEBUG_GEM
#define GEM_BUG_ON(condition) do { if (unlikely((condition))) {	\
		printk(KERN_ERR "GEM_BUG_ON(%s)\n", __stringify(condition)); \
		BUG(); \
		} \
	} while(0)
#define GEM_WARN_ON(expr) WARN_ON(expr)

#define GEM_DEBUG_DECL(var) var
#define GEM_DEBUG_EXEC(expr) expr
#define GEM_DEBUG_BUG_ON(expr) GEM_BUG_ON(expr)

#else
#define GEM_BUG_ON(expr) BUILD_BUG_ON_INVALID(expr)
#define GEM_WARN_ON(expr) (BUILD_BUG_ON_INVALID(expr), 0)

#define GEM_DEBUG_DECL(var)
#define GEM_DEBUG_EXEC(expr) do { } while (0)
#define GEM_DEBUG_BUG_ON(expr)
#endif

#if IS_ENABLED(CONFIG_DRM_I915_TRACE_GEM)
#define GEM_TRACE(...) trace_printk(__VA_ARGS__)
#else
#define GEM_TRACE(...) do { } while (0)
#endif

#define I915_NUM_ENGINES 5

#endif /* __I915_GEM_H__ */
