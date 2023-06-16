// SPDX-License-Identifier: GPL
 /* Copyright (C) 2023 Intel Corporation. All rights reserved.
 *
 * Author(s):
 */

#ifndef HAVE_INTEL_FREQ_CONTROL_ZL30793_SPI_H
#define HAVE_INTEL_FREQ_CONTROL_ZL30793_SPI_H

#include <linux/of_platform.h>
#include "intel_freq_control_zl30793.h"

/* bit 15:8  :0x0001 bit  7:0  :0x0002 */
#define ZL30793_PAGE0_REG_GENERAL_ID_0          0x0001
/* bit 2:0 state [0: FREERUN (or NCO mode)      1: HOLDOVER
 * 2: FAST_LOCK         3: ACQUIRING            4: LOCK]
 */
#define ZL30793_PAGE2_REG_DPLL_STATE_OFFSET_0   0x0120
/* bit 2:0 mode */
#define ZL30793_PAGE4_REG_DPLL_MODE_REFSEL_0    0x0210
/* bit 4:ignore sync    bit 3:nco_hybrid_en     bit 2:nco_auto_read
 * bit 1:tod_step_reset bit 0:tie_clear
 */
#define ZL30793_PAGE4_REG_DPLL_CTRL_0           0x0211
/* bit 47:40 :0x300  bit 39:32 :0x301  bit 31:24 :0x302
 * bit 23:16 :0x303  bit 15:8  :0x304  bit  7:0  :0x305
 */
#define ZL30793_PAGE6_REG_DPLL_DF_OFFSET_0_0    0x0300
/* 7:0  :0x306, bit 4 read_sem */
#define ZL30793_PAGE6_REG_DPLL_DF_CTRL_0        0x0306

#define PLL_SPI_PAGE_REG                        0x7f
#define PLL_SPI_PAGE(addr)                      (((addr) & 0xf80) >> 7)
#define PLL_SPI_READ(addr)                      (((addr) | 0x80) << 8)
#define PLL_SPI_WRITE(addr)                     (((addr) & 0x7f) << 8)
#define ZL30793_ADDR_ADDR(addr)                 ((addr) & 0x7f)

#define ZL30793_REG_READ_INTERVAL               (10)
#define ZL30793_DPLL_DF_CTRL_SEM_GET(data)      (((data) & 0x10) >> 4)

void intel_freq_control_zl30793(struct work_struct *work);
int spi_dev_check_zl30793_clock(struct spi_device *spi,
		struct intel_freq_control_private *); 

#endif