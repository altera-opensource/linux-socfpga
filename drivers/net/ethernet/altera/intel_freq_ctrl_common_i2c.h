/* SPDX-License-Identifier: GPL */
/* Copyright (C) 2023 Intel Corporation. All rights reserved.
 *
 * Author(s):
 */

#ifndef HAVE_INTEL_FREQ_CONTROL_COMMON_I2C_H
#define HAVE_INTEL_FREQ_CONTROL_COMMON_I2C_H

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

struct clock_cleaner;
int determine_i2c_client(struct clock_cleaner *clockcleaner_info);

#endif

