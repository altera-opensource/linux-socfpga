// SPDX-License-Identifier: GPL-2.0
/* Intel FPGA Clock Cleaner Frequency Adjustment Driver
 * Copyright (C) 2023 Intel Corporation. All rights reserved.
 *
 * Author(s):
 *	Lubana Badakar <lubana.badakar@intel.com>
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/spi/spi.h>
#include "intel_freq_control_zl30733.h"

/* Register Map Page 0, General */
u16 zl30733_reg_page_0[] = {
0x0000,// info 0x22 R
0x0001,//:0x0002 id see register definition R
0x0002,//:0x0002 id see register definition R
0x0003, // revision
0x0005,//:0x0006 fw_ver Contact Microchip R
0x0006,//:0x0006 fw_ver Contact Microchip R
0x0007,//:0x000A custom_config_ver 0xFFFFFFFF R/W
0x0008,//:0x000A custom_config_ver 0xFFFFFFFF R/W
0x0009,//:0x000A custom_config_ver 0xFFFFFFFF R/W
0x000A,//:0x000A custom_config_ver 0xFFFFFFFF R/W
0x000B,//:0x000E central_freq_offset 0x046AAAAB R/W
0x000C,//:0x000E central_freq_offset 0x046AAAAB R/W
0x000D,//:0x000E central_freq_offset 0x046AAAAB R/W
0x000E,//:0x000E central_freq_offset 0x046AAAAB R/W
0x0018, // 0x0018 reset status
0x0019,//:0x001A gpio_at_startup 0x0000 R
0x001A,//:0x001A gpio_at_startup 0x0000 R
0x0021, // x0 amp select
0x0022, // x0 osci select
0x0023,// xo osc0 select
0x0025, // xo tst ctrl
0x0026, // x0 config
0x0029,// sys apll source config
0x002A, // sys apll primary div int
0x002B,
0x002C, // sys apll primary div frac
0x002D,
0x002E,
0x002F,
0x0030, // sys apll secondry div
0x0032, // master clk status
0x0033,// master clock config ready
0x003E, // i2c device address
0x0046,//:0x002A master_clk_ofst 0x0000000000 R/W
0x0047,//:0x002A master_clk_ofst 0x0000000000 R/W
0x0048,//:0x002A master_clk_ofst 0x0000000000 R/W
0x0049,//:0x002A master_clk_ofst 0x0000000000 R/W
0x004A,//:0x002A master_clk_ofst 0x0000000000 R/W
0x002C,// osci_ctrl 0x02 R/W
0x007E,// uport 0x00 R/W
0x007F,// page sel
};

//Register Map Page 2, Status 0x100
u16 zl30733_reg_page_2[] = {
0x100, //split_xo_sel_status
0x0102,// ref_mon_status_0P 0x00 R
0x0103,// ref_mon_status_0N 0x00 R
0x0104,// ref_mon_status_1P 0x00 R
0x0105,// ref_mon_status_1N 0x00
0x0106,// ref_mon_status_2P 0x00 R
0x0107,// ref_mon_status_2N 0x00 R
0x0109,// ref_mon_status_3P 0x00 R
0x0109,// ref_mon_status_3N 0x00 R
0x010A,// ref_mon_status_4P 0x00 R
0x010B,// ref_mon_status_4N 0x00 R
0x0110,// dpll_mon_status_0 0x00 R
0x0111,// dpll_mon_status_1 0x00 R
0x0112,// dpll_mon_status_2 0x00 R
0x0113,// dpll_mon_status_3 0x00 R
0x0114,// dpll_mon_status_4 0x00 R
0x0115,// dpll_mon_status_5 0x00 R
0x0116,// dpll_mon_status_6 0x00 R
0x0117,// dpll_mon_status_7 0x00 R
0x0127,// dpll ns roll over status
0x0130,// dpll_state_refsel__status_0 0x00 R
0x0131,// dpll_state_refsel__status_1 0x00 R
0x0132,// dpll_state_refsel_status_2 0x00 R
0x0133,// dpll_state_refsel_status_3 0x00 R
0x0134,// dpll_state_refsel_status_4 0x00 R
0x0135,// dpll_state_refsel_status_5 0x00 R
0x0136,// dpll_state_refsel_status_6 0x00 R
0x0137,// dpll_state_refsel_status_7 0x00 R
0x0140,// gp_in_status 0x00 R
0x0141,// gp_in_status_7_0 0x00 R
0x0142,// gp_in_status_9_8 0x00 R
0x0144,// ref freq 0p 0x00 R
0x0145,// ref freq 0p 0x00 R
0x0146,// ref freq 0p 0x00 R
0x0147,// ref freq 0p 0x00 R
0x0148,// ref freq 0n 0x00 R
0x0149,// ref freq 0n 0x00 R
0x014A,// ref freq 0n 0x00 R
0x014b,// ref freq 0n 0x00 R

0x014c,// ref freq 1p 0x00 R
0x014d,// ref freq 1p 0x00 R
0x014e,// ref freq 1p 0x00 R
0x014f,// ref freq 1p 0x00 R
0x0150,// ref freq 1n 0x00 R
0x0151,// ref freq 1n 0x00 R
0x0152,// ref freq 1n 0x00 R
0x0153,// ref freq 1n 0x00 R

0x0154,// ref freq 2p 0x00 R
0x0155,// ref freq 2p 0x00 R
0x0156,// ref freq 2p 0x00 R
0x0157,// ref freq 2p 0x00 R
0x0158,// ref freq 2n 0x00 R
0x0159,// ref freq 2n 0x00 R
0x015a,// ref freq 2n 0x00 R
0x015b,// ref freq 2n 0x00 R

0x015c,// ref freq 3p 0x00 R
0x015d,// ref freq 3p 0x00 R
0x015e,// ref freq 3p 0x00 R
0x015f,// ref freq 3p 0x00 R
0x0160,// ref freq 3n 0x00 R
0x0161,// ref freq 3n 0x00 R
0x0162,// ref freq 3n 0x00 R
0x0163,// ref freq 3n 0x00 R

0x0164,// ref freq 4p 0x00 R
0x0165,// ref freq 4p 0x00 R
0x0166,// ref freq 4p 0x00 R
0x0167,// ref freq 4p 0x00 R
0x0168,// ref freq 4n 0x00 R
0x0169,// ref freq 4n 0x00 R
0x016a,// ref freq 4n 0x00 R
0x016b,// ref freq 4n 0x00 R

0x017E,// uport 0x00 R/W
0x017F,// page_sel 0x00 R/W
};

//Register Map Page 3, sticky
u16 zl30733_reg_page_3[] = {
0x0180,// sticky_lock 0x00 R/W
0x0182, // ref_mon_th_sticky_0p
0x0183,// ref_mon_th_sticky_0n
0x0184, // ref_mon_th_sticky_1p
0x0185,//ref_mon_th_sticky_1n
0x0186,// ref_mon_th_sticky_2p
0x0187,// ref_mon_th_sticky_2n
0x0188,//ref_mon_th_sticky_3p
0x0189,//ref_mon_th_sticky_3n
0x018A,//ref_mon_th_sticky_4p
0x018B,//ref_mon_th_sticky_4p
0x018C,//ref_dpll_freq_sticky_3_0 0x00 S
0x018D,//ref_dpll_freq_sticky_4  0x00 S
0x018E,//dpll_meas_ref_sticky_3_0
0x018F,//dpll_meas_ref_sticky_4
0x0190,//dpll_mon_th_sticky_0 0x00 S
0x0191,//dpll_mon_th_sticky_1 0x00 S
0x0192,//dpll_mon_th_sticky_2 0x00 S
0x0193,//dpll_mon_th_sticky_3 0x00 S
0x0194,//dpll_mon_th_sticky_4 0x00 S
0x0195,//dpll_mon_th_sticky_5 0x00 S
0x0196,//dpll_mon_th_sticky_6 0x00 S
0x0197,//dpll_mon_th_sticky_7 0x00 S
0x01a7,//dpll ns rollover sticky
0x01B2,//ref_mon_tl_sticky_0P 0x00 S
0x01B3,//ref_mon_tl_sticky_0N 0x00 S
0x01B4,//ref_mon_tl_sticky_1P 0x00 S
0x01B5,//ref_mon_tl_sticky_1N 0x00 S
0x01B6,//ref_mon_tl_sticky_2P 0x00 S
0x01B7,//ref_mon_tl_sticky_2N 0x00 S
0x01B8,//ref_mon_tl_sticky_3P 0x00 S
0x01B9,//ref_mon_tl_sticky_3N 0x00 S
0x01BA,//ref_mon_tl_sticky_4P 0x00 S
0x01BB,//ref_mon_tl_sticky_4N 0x00 S

0x01C0,//dpll_mon_tl_sticky_0 0x00 S
0x01C1,//dpll_mon_tl_sticky_1 0x00 S
0x01C2,//dpll_mon_tl_sticky_2 0x00 S
0x01C3,//dpll_mon_tl_sticky_3 0x00 S
0x01C4,//dpll_mon_tl_sticky_4 0x00 S
0x01C5,//dpll_mon_tl_sticky_5 0x00 S
0x01C6,//dpll_mon_tl_sticky_6 0x00 S
0x01C7,//dpll_mon_tl_sticky_7 0x00 S
0x01E0,//DPLL_FAST_LOCK_PAHSE_sticky 0x00 S
0x01E1,//dpll_fast_lock_frequency_sticky 0x00 S
0x01E4,//dpll_tie_wr_sticky 0x00 S
0x01E8,//ref_irq_ative_3-0 0x00 S
0x01E9,//ref_irq_ative_4 0x00 S
0x01Eb,//dpll_irq_ative
0x01EC,//synth_irq_ative
0x01ED,//output irq_active_7_0 0x00 S
0x01EE,//output irq_active_9_8 0x00 S
0x01EF,//dpll_ns_rollover_irq_active
0x01FE,//uport
0x01FF,//pagesel
};

//Register Map Page 4, Control
u16 zl30733_reg_page_4[] = {
0x0200,// ref_los_3_0 0x00 R/W
0x0201,// ref_los_4 0x00 R/W
0x020f,// ref_phase_err_read-request 0x00 R/W
0x021c,// ref_freq_meas_ctrl 0x00 R/W
0x021d,// ref_freq_meas_mask_3_0 0x00 R/W
0x021e,// ref_freq_meas_mask_4 0x00 R/W
0x021f,// dpll_meas_ref_freq_ctrl 0x00 R/W
//ref phase 0p
0x0220,
0x0221,
0x0222,
0x0223,
0x0224,
0x0225,
//ref phase 0n
0x0226,
0x0227,
0x0228,
0x0229,
0x022a,
0x022b,
//ref phase 1p
0x022c,
0x022d,
0x022e,
0x022f,
0x0230,
0x0231,
//ref phase 1n
0x0232,
0x0233,
0x0234,
0x0235,
0x0236,
0x0237,
//ref phase 2p
0x0238,
0x0239,
0x023a,
0x023b,
0x023c,
0x023d,
//ref phase 2n
0x023e,
0x023f,
0x0240,
0x0241,
0x0242,
0x0243,
//ref phase 3p
0x0244,
0x0245,
0x0246,
0x0247,
0x0248,
0x0249,
//ref phase 3n
0x024a,
0x024b,
0x024c,
0x024d,
0x024e,
0x024f,

//ref phase 4p
0x0250,
0x0251,
0x0252,
0x0253,
0x0254,
0x0255,
//ref phase 4n
0x0256,
0x0257,
0x0258,
0x0259,
0x025a,
0x025b,
//dpll phase 0
0x025c,
0x025d,
0x025e,
0x025f,
0x0260,
0x0261,
//dpll phase 1
0x0262,
0x0263,
0x0264,
0x0265,
0x0266,
0x0267,
//dpll phase 2
0x0268,
0x0269,
0x026a,
0x026b,
0x026c,
0x026d,
//dpll phase 3
0x026e,
0x026f,
0x0270,
0x0271,
0x0272,
0x0273,
//dpll phase 4
0x0274,
0x0275,
0x0276,
0x0277,
0x0278,
0x0279,
0x027e, // uport
0x027f,//pagesel
};

u16 zl30733_reg_page_5[] = {
0x0280,// split xO_ref 0x00 R/W
0x0281,// split xO_modectrl 0x00 R/W
0x0283,// dpll_enable 0x00 R/W
0x0284,// dpll_mode refsel0 0x00 R/W
0x0285,// dpll_ctrl 0 0x00 R/W
0x0286,// dpll_cmd 0 0x00 R/W
0x0288,// dpll_mode refsel1 0x00 R/W
0x0289,// dpll_ctrl 1 0x00 R/W
0x028a,// dpll_cmd 1 0x00 R/W
0x028c,// dpll_mode refsel2 0x00 R/W
0x028d,// dpll_ctrl 2 0x00 R/W
0x028e,// dpll_cmd 2 0x00 R/W
0x0290,// dpll_mode refsel3 0x00 R/W
0x0291,// dpll_ctrl 3 0x00 R/W
0x0292,// dpll_cmd 3 0x00 R/W
0x0294,// dpll_mode refsel4 0x00 R/W
0x0295,// dpll_ctrl 4 0x00 R/W
0x0296,// dpll_cmd 4 0x00 R/W
0x0298,// dpll_mode refsel5 0x00 R/W
0x0299,// dpll_ctrl 5 0x00 R/W
0x029a,// dpll_cmd 5 0x00 R/W
0x029c,// dpll_mode refsel6 0x00 R/W
0x029d,// dpll_ctrl 6 0x00 R/W
0x029e,// dpll_cmd 6 0x00 R/W
0x02a0,// dpll_mode refsel7 0x00 R/W
0x02a1,// dpll_ctrl 70x00 R/W
0x02a2,// dpll_cmd 7 0x00 R/W
0x02a4,// ext fb ctrl 0x00 R/W
0x02a5,// ext fb sel 0x00 R/W
0x02a6,// dpll_df_read_all_mask 0x00 R/W
0x02a7,// dpll_df_read_all 0x00 R/W
0x02a8,// dpll_df_read_0 0x00 R/W
0x02a9,// dpll_df_read_1 0x00 R/W
0x02aa,// dpll_df_read_2 0x00 R/W
0x02ab,// dpll_df_read_3 0x00 R/W
0x02ac,// dpll_df_read_4 0x00 R/W
0x02ad,// dpll_df_read_5 0x00 R/W
0x02ae,// dpll_df_read_6 0x00 R/W
0x02af,// dpll_df_read_7 0x00 R/W
0x02b0,// dpll ti ctrl 0x00 R/W
0x02b1,// dpll tie mask 0x00 R/W
0x02b8,// dpll tod ctrl 0 0x00 R/W
0x02b9,// dpll tod ctrl 1 0x00 R/W
0x02ba,// dpll tod ctrl 2 0x00 R/W
0x02bb,// dpll tod ctrl 3 0x00 R/W
0x02bc,// dpll tod ctrl 4 0x00 R/W
0x02bd,// dpll tod ctrl 5 0x00 R/W
0x02be,// dpll tod ctrl 6 0x00 R/W
0x02bf,// dpll tod ctrl 7 0x00 R/W
0x02d0,// dpll_meas ctlr 0x00 R/W
0x02d1,// dpll_meas  indx 0x00 R/W
0x02d2,// ref_los_3_0 0x00 R/W
0x02d3,// ref_los_3_0 0x00 R/W
0x02d4,// ref_los_3_0 0x00 R/W
0x02d5,
0x02d6,
0x02d7,
0x02d8,
0x02d9,
0x02da,
0x02db,
0x02dc,
0x02dd,
0x02de,
0x02df,
0x02e0,
0x02e1,
0x02e2,
0x02e3,
0x02e4,
0x02e5,
0x02e6,
0x02e7,
0x02e8,
0x02e9,
0x02ea,
0x02eb,
0x02ec,
0x02ed,
0x02ee,
0x02ef,
0x02f0,
0x02f1,
0x02f2,
0x02fe, // uport
0x02ff, //page sel
};

//Register Map Page 6, Status 0x300
u16 zl30733_reg_page_6[] = {
0x0300,//:0x0305 dpll_df_offset_0 0x000000000000 R/W
0x0301,// dpll_df_offset_0 0x000000000000 R/W
0x0302,// dpll_df_offset_0 0x000000000000 R/W
0x0303,// dpll_df_offset_0 0x000000000000 R/W
0x0304,// dpll_df_offset_0 0x000000000000 R/W
0x0305,// dpll_df_offset_0 0x000000000000 R/W
0x030c,// dpll_tie_data_0 0x000000000000 R/W
0x030d,// dpll_tie_data_0 0x000000000000 R/W
0x030e,// dpll_tie_data_0 0x000000000000 R/W
0x030f,// dpll_tie_data_0 0x000000000000 R/W
0x0310,// dpll_tie_data_0 0x000000000000 R/W
0x0311,// dpll_tie_data_0 0x000000000000 R/W
0x0312,// dpll_tod_sec_0 0x000000000000 R/W
0x0313,// dpll_tod_sec_0 0x000000000000 R/W
0x0314,// dpll_tod_sec_0 0x000000000000 R/W
0x0315,// dpll_tod_sec_0 0x000000000000 R/W
0x0316,// dpll_tod_sec_0 0x000000000000 R/W
0x0317,// dpll_tod_sec_0 0x000000000000 R/W
0x0318,// dpll_tod_ns_0 0x000000000000 R/W
0x0319,// dpll_tod_ns_0 0x000000000000 R/W
0x031a,// dpll_tod_ns_0 0x000000000000 R/W
0x031b,// dpll_tod_ns_0 0x000000000000 R/W
0x0320,//:0x0305 dpll_df_offset_1 0x000000000000 R/W
0x0321,// dpll_df_offset_1 0x000000000000 R/W
0x0322,// dpll_df_offset_1 0x000000000000 R/W
0x0323,// dpll_df_offset_1 0x000000000000 R/W
0x0324,// dpll_df_offset_1 0x000000000000 R/W
0x0325,// dpll_df_offset_1 0x000000000000 R/W
0x032c,// dpll_tie_data_1 0x000000000000 R/W
0x032d,// dpll_tie_data_1 0x000000000000 R/W
0x032e,// dpll_tie_data_1 0x000000000000 R/W
0x032f,// dpll_tie_data_1 0x000000000000 R/W
0x0330,// dpll_tie_data_1 0x000000000000 R/W
0x0331,// dpll_tie_data_1 0x000000000000 R/W
0x0332,// dpll_tod_sec_1 0x000000000000 R/W
0x0333,// dpll_tod_sec_1 0x000000000000 R/W
0x0334,// dpll_tod_sec_1 0x000000000000 R/W
0x0335,// dpll_tod_sec_1 0x000000000000 R/W
0x0336,// dpll_tod_sec_1 0x000000000000 R/W
0x0337,// dpll_tod_sec_1 0x000000000000 R/W
0x0338,// dpll_tod_ns_1 0x000000000000 R/W
0x0339,// dpll_tod_ns_1 0x000000000000 R/W
0x033a,// dpll_tod_ns_1 0x000000000000 R/W
0x033b,// dpll_tod_ns_1 0x000000000000 R/W
0x0340,//:0x0305 dpll_df_offset_2 0x000000000000 R/W
0x0341,// dpll_df_offset_2 0x000000000000 R/W
0x0342,// dpll_df_offset_2 0x000000000000 R/W
0x0343,// dpll_df_offset_2 0x000000000000 R/W
0x0344,// dpll_df_offset_2 0x000000000000 R/W
0x0345,// dpll_df_offset_2 0x000000000000 R/W
0x034c,// dpll_tie_data_2 0x000000000000 R/W
0x034d,// dpll_tie_data_2 0x000000000000 R/W
0x034e,// dpll_tie_data_2 0x000000000000 R/W
0x034f,// dpll_tie_data_2 0x000000000000 R/W
0x0350,// dpll_tie_data_2 0x000000000000 R/W
0x0351,// dpll_tie_data_2 0x000000000000 R/W
0x0352,// dpll_tod_sec_2 0x000000000000 R/W
0x0353,// dpll_tod_sec_2 0x000000000000 R/W
0x0354,// dpll_tod_sec_2 0x000000000000 R/W
0x0355,// dpll_tod_sec_2 0x000000000000 R/W
0x0356,// dpll_tod_sec_2 0x000000000000 R/W
0x0357,// dpll_tod_sec_2 0x000000000000 R/W

0x0358,// dpll_tod_ns_2 0x000000000000 R/W
0x0359,// dpll_tod_ns_2 0x000000000000 R/W
0x035a,// dpll_tod_ns_2 0x000000000000 R/W
0x035b,// dpll_tod_ns_2 0x000000000000 R/W

0x0360,//:0x0305 dpll_df_offset_4 0x000000000000 R/W
0x0361,// dpll_df_offset_3 0x000000000000 R/W
0x0362,// dpll_df_offset_3 0x000000000000 R/W
0x0363,// dpll_df_offset_3 0x000000000000 R/W
0x0364,// dpll_df_offset_3 0x000000000000 R/W
0x0365,// dpll_df_offset_3 0x000000000000 R/W

0x036c,// dpll_tie_data_3 0x000000000000 R/W
0x036d,// dpll_tie_data_3 0x000000000000 R/W
0x036e,// dpll_tie_data_3 0x000000000000 R/W
0x036f,// dpll_tie_data_3 0x000000000000 R/W
0x0370,// dpll_tie_data_3 0x000000000000 R/W
0x0371,// dpll_tie_data_3 0x000000000000 R/W

0x0372,// dpll_tod_sec_3 0x000000000000 R/W
0x0373,// dpll_tod_sec_3 0x000000000000 R/W
0x0374,// dpll_tod_sec_3 0x000000000000 R/W
0x0375,// dpll_tod_sec_3 0x000000000000 R/W
0x0376,// dpll_tod_sec_3 0x000000000000 R/W
0x0377,// dpll_tod_sec_3 0x000000000000 R/W

0x0378,// dpll_tod_ns_3 0x000000000000 R/W
0x0379,// dpll_tod_ns_3 0x000000000000 R/W
0x037a,// dpll_tod_ns_3 0x000000000000 R/W
0x037b,// dpll_tod_ns_3 0x000000000000 R/W

0x037e, // uport
0x037f,// page sel
};

/* manual offset correction based on ptp4l frequency offset in ppb
 *  x               1                     2^48
 * -------   =  -----------    =>  x =  ---------------
 * 2^48         1000000000(ppb)          1000000000(ppb)
 *
 * dpll_df_manual_0_step_val = (freq_offset(ppb) << 48) /1000000000
 * set dpll_df_manual_0 register  with dpll_df_manual_0_step_val
 * and keep dpll_df_manual_0_step_val sign same as ptp4l freq offset
 */

static ssize_t zl30733_dpll_manual_freq_offset_show(struct device *dev,
						    struct device_attribute *attr,
						    char *buf)
{
	int ret = 0, len = 0, i = 0;
	static u8 dpll_df_data[5] = {0};
	static u16 dpll_df_addr = 0x0300;
	struct i2c_client *i2c = to_i2c_client(dev);

	for (i = 0; i < 5; i++) {
		ret = i2c_zl30733_read_byte_data(i2c, (dpll_df_addr + i),  &dpll_df_data[i], 1);

		if (ret)
			dev_err(&i2c->dev, "I2C read error %d\n", ret);

		dev_info(&i2c->dev, "%s:  dpll_df_manual_0 addr = 0x%x  data = 0x%x\n",
			 __func__, (dpll_df_addr + i), dpll_df_data[i]);
	}

	return len;
}

static ssize_t
zl30733_dpll_manual_freq_offset_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t len)
{
	int ret = 0, i = 0;
	int sscanf_ret = 0;
	u8 dpll_df_data[6] = {0};
	u16 dpll_df_addr = 0;
	struct i2c_client *i2c = to_i2c_client(dev);

	sscanf_ret = sscanf(buf, "%hx %hhx %hhx %hhx %hhx %hhx %hhx", &dpll_df_addr,
			    &dpll_df_data[0], &dpll_df_data[1], &dpll_df_data[2],
			    &dpll_df_data[3], &dpll_df_data[4], &dpll_df_data[5]);

	if (sscanf_ret != 6) {
		dev_err(&i2c->dev, "sscanf didn't read the inputs properly and read only %d\n",
			sscanf_ret);
		goto zl_dpll_err;
	}

	dev_info(&i2c->dev, "%s:  input <dpll_df_offset start address [%x]>  and <data is %x %x %x %x %x %x>\n",
		 __func__, dpll_df_addr, dpll_df_data[0],
		 dpll_df_data[1], dpll_df_data[2],
		 dpll_df_data[3], dpll_df_data[4], dpll_df_data[5]);

	for (i = 0; i < 6; i++) {
		ret = i2c_zl30733_write_byte_data(i2c, (dpll_df_addr + i), &dpll_df_data[i], 1);
		if (ret) {
			dev_err(&i2c->dev, "I2c write error %d\n", ret);
			goto zl_dpll_err;
		}
	}

zl_dpll_err:
	return len;
}

static ssize_t
zl30733_register_dump_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	int ret = 0, i;
	u8  val;
	struct i2c_client *i2c_cli = to_i2c_client(dev);

	pr_info("\n\nzl30733_clock_reg_dump()  page 0 General Info\n");
	for (i = 0 ; i < (sizeof(zl30733_reg_page_0) / sizeof(u16)); i++) {
		ret = i2c_zl30733_read_byte_data(i2c_cli, zl30733_reg_page_0[i],  &val, 1);
		if (ret) {
			dev_err(&i2c_cli->dev, "i2c read error %d\n", ret);
			goto zl_reg_dump_err;
		}
		pr_info("page 0[%d] addr 0x%x  0x%x\n", i, zl30733_reg_page_0[i], val);
	}

	pr_info("\n\nzl30733_clock_reg_dump()  page 2 Status\n");
	for (i = 0 ; i < (sizeof(zl30733_reg_page_2) / sizeof(u16)); i++) {
		ret = i2c_zl30733_read_byte_data(i2c_cli, zl30733_reg_page_2[i],  &val, 1);
		if (ret) {
			dev_err(&i2c_cli->dev, "i2c read error %d\n", ret);
			goto zl_reg_dump_err;
		}
		pr_info("page 2[%d] addr 0x%x  0x%x\n", i, zl30733_reg_page_2[i], val);
	}

	pr_info("\n\nzl30733_clock_reg_dump()  page 3 sticky\n");
	for (i = 0 ; i < (sizeof(zl30733_reg_page_3) / sizeof(u16)); i++) {
		ret = i2c_zl30733_read_byte_data(i2c_cli, zl30733_reg_page_3[i],  &val, 1);
		if (ret) {
			dev_err(&i2c_cli->dev, "i2c read error %d\n", ret);
			goto zl_reg_dump_err;
		}
		pr_info("page 3[%d] addr 0x%x  0x%x\n", i, zl30733_reg_page_3[i], val);
	}

	pr_info("\n\nzl30733_clock_reg_dump()  page 4 control\n");
	for (i = 0 ; i < (sizeof(zl30733_reg_page_4) / sizeof(u16)); i++) {
		ret = i2c_zl30733_read_byte_data(i2c_cli, zl30733_reg_page_4[i],  &val, 1);
		if (ret) {
			dev_err(&i2c_cli->dev, "i2c read error %d\n", ret);
			goto zl_reg_dump_err;
		}
		pr_info("page 4[%d] addr 0x%x  0x%x\n", i, zl30733_reg_page_4[i], val);
	}

	pr_info("\n\nzl30733_clock_reg_dump()  page 5 dpll\n");
	for (i = 0 ; i < (sizeof(zl30733_reg_page_5) / sizeof(u16)); i++) {
		ret = i2c_zl30733_read_byte_data(i2c_cli, zl30733_reg_page_5[i],  &val, 1);
		if (ret) {
			dev_err(&i2c_cli->dev, "i2c read error %d\n", ret);
			goto zl_reg_dump_err;
		}
		pr_info("page 5[%d] addr 0x%x  0x%x\n", i, zl30733_reg_page_5[i], val);
	}
	pr_info("\n\nzl30733_clock_reg_dump()  page 6 DPLL\n");
	for (i = 0 ; i < (sizeof(zl30733_reg_page_6) / sizeof(u16)); i++) {
		ret = i2c_zl30733_read_byte_data(i2c_cli, zl30733_reg_page_6[i],  &val, 1);
		if (ret) {
			dev_err(&i2c_cli->dev, "i2cread error %d\n", ret);
			goto zl_reg_dump_err;
		}
		pr_info("page 6[%d] addr 0x%x  0x%x\n", i, zl30733_reg_page_6[i], val);
	}

zl_reg_dump_err:
	return ret;
}

static ssize_t
zl30733_register_dump_store(struct device *dev,
			    struct device_attribute *attr, const char *buf,
			    size_t count)
{
	return 0;
}

static DEVICE_ATTR_RW(zl30733_dpll_manual_freq_offset);
static DEVICE_ATTR_RW(zl30733_register_dump);

static struct attribute *zl30733_sysfs_attrs[] = {
	&dev_attr_zl30733_dpll_manual_freq_offset.attr,
	&dev_attr_zl30733_register_dump.attr,
	NULL
};

static const struct attribute_group zl30733_attr_group = {
	.attrs = zl30733_sysfs_attrs,
};

int zl30733_sysfs_configure(struct i2c_client *i2c_cli)
{
	if (i2c_cli) {
		struct device *dev = &i2c_cli->dev;
		struct kobject *kobj = &dev->kobj;

		/* force an update based on parameters read from the device */
		sysfs_update_group(kobj, &zl30733_attr_group);
	}
	return 0;
}
