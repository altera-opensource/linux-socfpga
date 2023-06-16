// SPDX-License-Identifier: GPL
 /* Copyright (C) 2023 Intel Corporation. All rights reserved.
 *
 * Author(s):
 */

#ifndef HAVE_INTEL_FREQ_CONTROL_LMK5028_I2C_H
#define HAVE_INTEL_FREQ_CONTROL_LMK5028_I2C_H

/* Register numbers and other defines for LMK0502*/
/*Post-dividers P2 and P1 for DPLL2*/
#define LMK05028_REG_PLL2_P2_P1         0x0062
/* REF feedback pre-divider PR for DPLL2 */
#define LMK05028_REG_PLL2_REF_PR        0x01f5
/* REF1 R divider for DPLL2, high order byte */
#define LMK05028_REG_PLL2_REF1_RH       0x01bf
/* REF FB denominator DEN for DPLL2, high order byte */
#define LMK05028_REG_PLL2_REF_DENH      0x01ff
/* Frequency step word for DPLL2, high order byte */
#define LMK05028_REG_PLL2_FDEVH         0x024c
/* Frequency incr/decr register control for DPLL2 */
#define LMK05028_REG_PLL2_FSTEP         0x0251
/* Expected fIN of REF1 for PLL2 in kHz */
#define LMK05028_PLL2_FIN1              390625

/* VCO for AGX E-Tile & EXT TI LMK05028 EVB Clocking scheme */
#define LMK05028_PLL2_FVCO              5625000 // 5625000khz

void intel_freq_control_lmk05028(struct work_struct *work);

#endif