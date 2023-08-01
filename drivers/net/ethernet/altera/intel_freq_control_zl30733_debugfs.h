/* SPDX-License-Identifier: GPL-2.0 */
/* Intel FPGA Clock Cleaner Frequency Adjustment Driver
 * Copyright (C) 2023 Intel Corporation. All rights reserved.
 *
 * contributor(s):
 *	Inian Pavel <inian.pavel.sakthi@intel.com>
 */

#ifndef HAVE_INTEL_FREQUENCY_CONTROL_ZL30733_DEBUGFS_H
#define HAVE_INTEL_FREQUENCY_CONTROL_ZL30733_DEBUGFS_H

struct zarlink_pll_dbg *zl30733_dbgfs_init(struct i2c_client *i2c_cli);

#endif //HAVE_INTEL_FREQUENCY_CONTROL_ZL30793_DEBUGFS_H
