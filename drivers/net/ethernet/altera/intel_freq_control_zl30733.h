/* SPDX-License-Identifier: GPL-2.0 */
/* Intel FPGA Clock Cleaner Frequency Adjustment Driver
 * Copyright (C) 2023 Intel Corporation. All rights reserved.
 *
 * Author(s):
 *	Lubana Badakar <lubana.badakar@intel.com>
 */

#ifndef HAVE_INTEL_FREQUENCY_CONTROL_ZL30733_H
#define HAVE_INTEL_FREQUENCY_CONTROL_ZL30733_H

int zl30733_sysfs_configure(struct i2c_client *i2c_cli);
u8 i2c_zl30733_write_byte_data(const struct i2c_client *client, u16 reg, u8 data[], u8 data_len);
u8 i2c_zl30733_read_byte_data(const struct i2c_client *client, u16 reg, u8 *buf, u8 data_len);

#endif //HAVE_INTEL_FREQUENCY_CONTROL_ZL30793_H

