// SPDX-License-Identifier: GPL
 /* Copyright (C) 2023 Intel Corporation. All rights reserved.
 *
 * Author(s):
 *	Lubana Badakar <lubana.badakar@altera.com>
 */

#ifndef __ALTERA_FREQ_CONTROL_SI5518_I2C_H__
#define __ALTERA_FREQ_CONTROL_SI5518_I2C_H__

#include <linux/of_platform.h>

void intel_freq_control_i2c_si5518(struct work_struct *work);
int i2c_dev_check_si5518_clock(struct intel_freq_control_private *);
bool si5518_clock_pre_modify_check(struct intel_freq_control_private *fq,
				   long scaled_ppm);
void i2c_dev_si5518_init(struct intel_freq_control_private *priv);

#endif
