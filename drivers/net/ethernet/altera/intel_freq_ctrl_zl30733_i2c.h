/* SPDX-License-Identifier: GPL */
/* Intel zarlink i2c driver
 * Copyright (C) 2023 Intel Corporation. All rights reserved
 *
 * Contributors:
 *
 *	Inian Pavel Sakthi <inian.pavel.sakthi@intel.com>
 */

#ifndef HAVE_INTEL_FREQ_CONTROL_ZL30733_I2C_H
#define HAVE_INTEL_FREQ_CONTROL_ZL30733_I2C_H
// Register numbers and other defines for ZL30733
#define ZL30733_ADDR_TO_PAGE(addr)      (((addr) & 0xf80) >> 7)
#define ZL30733_ADDR_ADDR(addr)         ((addr) & 0x7f)
#define ZL30733_ID_VALUE                0x0e95 // ZL30733
#define ZL30733_REG_ID_0                0x0001
#define ZL30733_REG_ID_1                0x0002
#define ZL30733_REG_PAGE_SEL            0x007f
#define ZL30733_REG_DPLL_MON_STATUS(x)  (0x0110 + (x)) // x=0-7
#define ZL30733_REG_DPLL_MODE_REFSEL_0  0x0284
#define ZL30733_REG_DPLL_CTRL_0         0x0285

#define ZL30733_REG_DPLL_DF_OFFSET_0_0  0x0300 // bit 47:40
#define ZL30733_REG_DPLL_DF_OFFSET_0_1  0x0301 // bit 39:32
#define ZL30733_REG_DPLL_DF_OFFSET_0_2  0x0302 // bit 31:24
#define ZL30733_REG_DPLL_DF_OFFSET_0_3  0x0303 // bit 23:16
#define ZL30733_REG_DPLL_DF_OFFSET_0_4  0x0304 // bit 15:8
#define ZL30733_REG_DPLL_DF_OFFSET_0_5  0x0305 // bit  7:0

/* bit 15:8  :0x0001 bit  7:0  :0x0002 */
#define ZL30733_PAGE0_REG_GENERAL_ID_0          0x0001
/* bit 2:0 state [0: FREERUN (or NCO mode)      1: HOLDOVER
 * 2: FAST_LOCK         3: ACQUIRING            4: LOCK]
 */
#define ZL30733_PAGE2_REG_DPLL_STATE_OFFSET_0   0x0120

#define ZL30733_PAGE2_REG_DPLL_STATE_OFFSET_1   0x0121
/* bit 2:0 mode */
#define ZL30733_PAGE4_REG_DPLL_MODE_REFSEL_0    0x0284

#define ZL30733_PAGE4_REG_DPLL_MODE_REFSEL_1    0x0288

/* bit 4:ignore sync    bit 3:nco_hybrid_en     bit 2:nco_auto_read
 * bit 1:tod_step_reset bit 0:tie_clear
 */
#define ZL30733_PAGE4_REG_DPLL_CTRL_0           0x0285
#define ZL30733_PAGE4_REG_DPLL_CTRL_1           0x0289
/* bit 47:40 :0x300  bit 39:32 :0x301  bit 31:24 :0x302
 * bit 23:16 :0x303  bit 15:8  :0x304  bit  7:0  :0x305
 */
#define ZL30733_PAGE6_REG_DPLL_DF_OFFSET_0_0    0x0300
/* 7:0  :0x306, bit 4 read_sem */
#define ZL30733_PAGE5_REG_DPLL_DF_READ_0        0x02A8

#define ZL30733_PAGE5_REG_DPLL_DF_READ_1        0x02A9

#define ZL30733_REG_DPLL_MODE_REFSEL_0		0x0284

#define ZL30733_REG_READ_INTERVAL               (10)
#define ZL30733_DPLL_DF_CTRL_SEM_GET(data)      (((data) & 0x10) >> 4)
#define ZL30733_DPLL_NCO_MODE                   0x04
#define ZL30733_DPLL_MODE_MASK                   0x07
#define ZL30733_DPLL_CHECK_NCO_MODE(x)  (((x) & ZL30733_DPLL_MODE_MASK) == ZL30733_DPLL_NCO_MODE)
#define ZL30733_DPLL_SET_NCO_MODE(x)      (((x) & 0xf8) | ZL30733_DPLL_NCO_MODE)

int i2c_dev_check_zl30733_clock(struct intel_freq_control_private *priv);

void intel_freq_control_zl30733(struct work_struct *work);

u8 i2c_zl30733_write_byte_data(const struct i2c_client *client, u16 reg, u8 data[], u8 data_len);
u8 i2c_zl30733_read_byte_data(const struct i2c_client *client, u16 reg, u8 *buf, u8 data_len);

#endif
