/* SPDX-License-Identifier: GPL-2.0 */
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
#define PLL_MODE_AUTOMATIC			0x3
#define SET_DPLL_MODE_AUTOMATIC(x)              ((x & 0xF8) | PLL_MODE_AUTOMATIC) 
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
#define ZL30793_PAGE8_REG_GP_OUT_WIDTH_1        0x0436
#define ZL30793_PAGE2_REG_DPLL_MON_STATUS_0        0x0118
#define ZL30793_DPLL_IS_LOCKED(data)      ((data) & 0x01)


#define PLL_SPI_PAGE_REG                        0x7f
#define ZL30793_PLL_STATE_MASK			0x7
#define PLL_SPI_PAGE(addr)                      (((addr) & 0xf80) >> 7)
#define PLL_SPI_READ(addr)                      (((addr) | 0x80) << 8)
#define PLL_SPI_WRITE(addr)                     (((addr) & 0x7f) << 8)
#define ZL30793_ADDR_ADDR(addr)                 ((addr) & 0x7f)

#define ZL30793_REG_READ_INTERVAL               (10)
#define ZL30793_DPLL_DF_CTRL_SEM_GET(data)      (((data) & 0x10) >> 4)

#define ZL30793_LOCK_CHECK_INTERVAL_IN_MS	(500)
#define ZL30793_MAX_PLL_LOCK_CHECK_COUNTER      (500)

void intel_freq_control_zl30793(struct work_struct *work);
int spi_dev_check_zl30793_clock(struct intel_freq_control_private *priv);
int reset_dpll_mode(struct intel_freq_control_private *priv);
void zl30733_dbgfs_remove(struct zarlink_pll_dbg *d);
#endif
