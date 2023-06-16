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
#include "intel_freq_control_zl30793.h"

/* Register Map Page 0, General */
u16 zl30793_reg_page_0[] = {
0x0000,// info 0x22 R
0x0001,//:0x0002 id see register definition R
0x0002,//:0x0002 id see register definition R
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
0x0011,// auto_config_sel 0x03 R
0x0012,// warm_start 0x00 R/W
0x0013,// trap_status 0x00 R/W
0x0014,// trap_mask 0x00 R/W
0x0019,//:0x001A gpio_at_startup 0x0000 R
0x001A,//:0x001A gpio_at_startup 0x0000 R
0x0026,//:0x002A master_clk_ofst 0x0000000000 R/W
0x0027,//:0x002A master_clk_ofst 0x0000000000 R/W
0x0028,//:0x002A master_clk_ofst 0x0000000000 R/W
0x0029,//:0x002A master_clk_ofst 0x0000000000 R/W
0x002A,//:0x002A master_clk_ofst 0x0000000000 R/W
0x002C,// osci_ctrl 0x02 R/W
0x007E,// uport 0x00 R/W
0x007F,
};

//Register Map Page 2, Status 0x100
u16 zl30793_reg_page_2[] = {
0x0100,// gpio_in_status_2_0 0x00 R
0x0101,// gpio_in_status_8_5 0x00 R
0x0108,// ref_mon_status_0P 0x00 R
0x0109,// ref_mon_status_0N 0x00 R
0x010A,// ref_mon_status_1P 0x00 R
0x010B,// ref_mon_status_1N 0x00
0x010C,// ref_mon_status_2P 0x00 R
0x010D,// ref_mon_status_2N 0x00 R
0x010E,// ref_mon_status_3P 0x00 R
0x010F,// ref_mon_status_3N 0x00 R
0x0110,// ref_mon_status_4P 0x00 R
0x0111,// ref_mon_status_4N 0x00 R
0x0118,// dpll_mon_status_0 0x00 R
0x0119,// dpll_mon_status_1 0x00 R
0x011A,// dpll_mon_status_2 0x00 R
0x011B,// dpll_mon_status_3 0x00 R
0x011C,// dpll_mon_status_4 0x00 R
0x011D,// dpll_mon_status_5 0x00 R
0x0120,// dpll_state_refsel_0 0x00 R
0x0121,// dpll_state_refsel_1 0x00 R
0x0122,// dpll_state_refsel_2 0x00 R
0x0123,// dpll_state_refsel_3 0x00 R
0x0128,// split_xo_sel_status 0x00 R
0x012A,//:0x012E split_xo_switch_df 0x0000000000 R
0x012B,// split_xo_switch_df 0x0000000000 R
0x012C,// split_xo_switch_df 0x0000000000 R
0x012D,// split_xo_switch_df 0x0000000000 R
0x012E,// split_xo_switch_df 0x0000000000 R
0x013F,// synth_fine_shift_status 0x00 R
0x0140,// gp_mon_status 0x00 R
0x0141,// hp_mon_status_1 0x00 R
0x0142,// hp_mon_status_2 0x00 R
0x0143,// hp_out_mon_status_0 0x00 R
0x0144,// hp_out_mon_status_1 0x00 R
0x0145,// hp_out_mon_status_2 0x00 R
0x0146,// hp_out_mon_status_3 0x00 R
0x0147,// hp_out_mon_status_4 0x00 R
0x0148,// hp_out_mon_status_5 0x00 R
0x0149,// hp_out_mon_status_6 0x00 R
0x014A,// hp_out_mon_status_7 0x00 R
0x0150,// hp_fb_msdiv_sel_1 0x00 R
0x0151,//:0x0154 hp_fb_lsdiv_sel_1 0x00000000 R
0x0152,// hp_fb_lsdiv_sel_1 0x00000000 R
0x0153,// hp_fb_lsdiv_sel_1 0x00000000 R
0x0154,// hp_fb_lsdiv_sel_1 0x00000000 R
0x0158,// hp_fb_msdiv_sel_2 0x00 R
0x0159,//:0x015C hp_fb_lsdiv_sel_2 0x00000000 R
0x015A,// hp_fb_lsdiv_sel_2 0x00000000 R
0x015B,// hp_fb_lsdiv_sel_2 0x00000000 R
0x015C,// hp_fb_lsdiv_sel_2 0x00000000 R
0x017E,// uport 0x00 R/W
0x017F,// page_sel 0x00 R/W
};

//Register Map Page 3, sticky
u16 zl30793_reg_page_3[] = {
0x0180,// sticky_lock 0x00 R/W
0x0181,// gpio_latch_status_8_5 0x00 S
0x0186,// ref_dpll_freq_sticky_3_0 0x00 S
0x0187,// ref_dpll_freq_sticky_4 0x00 S
0x0188,//dpll_meas_ref_sticky_3_0 0x00 S
0x0189,//dpll_meas_ref_sticky_4 0x00 S
0x018A,//dpll_fast_lock_sticky 0x00 S
0x018B,//dpll_fast_lock_phase_sticky 0x00 S
0x018C,//dpll_fast_lock_freq_sticky 0x00 S
0x018D,//dpll_tie_wr_sticky 0x00 S
0x0190,//gp_out_phase_step_sticky 0x00 S
0x0191,//hp_out_phase_step_sticky 0x00 S
0x01A8,//ref_irq_active_3_0 0x00 S
0x01A9,//ref_irq_active_4 0x00 S
0x01AB,//dpll_irq_active 0x00 S
0x01AC,//synth_irq_active 0x00 S
0x01AD,//hp_out_irq_active 0x00 S
0x01B0,//ref_mon_th_sticky_0P 0x00 S
0x01B1,//ref_mon_tl_sticky_0P 0x00 S
0x01B2,//ref_mon_th_sticky_0N 0x00 S
0x01B3,//ref_mon_tl_sticky_0N 0x00 S
0x01B4,//ref_mon_th_sticky_1P 0x00 S
0x01B5,//ref_mon_tl_sticky_1P 0x00 S
0x01B6,//ref_mon_th_sticky_1N 0x00 S
0x01B7,//ref_mon_tl_sticky_1N 0x00 S
0x01B8,//ref_mon_th_sticky_2P 0x00 S
0x01B9,//ref_mon_tl_sticky_2P 0x00 S
0x01BA,//ref_mon_th_sticky_2N 0x00 S
0x01BB,//ref_mon_tl_sticky_2N 0x00 S
0x01BC,//ref_mon_th_sticky_3P 0x00 S
0x01BD,//ref_mon_tl_sticky_3P 0x00 S
0x01BE,//ref_mon_th_sticky_3N 0x00 S
0x01BF,//ref_mon_tl_sticky_3N 0x00 S
0x01C0,//ref_mon_th_sticky_4P 0x00 S
0x01C1,//ref_mon_tl_sticky_4P 0x00 S
0x01C2,//ref_mon_th_sticky_4N 0x00 S
0x01C3,//ref_mon_tl_sticky_4N 0x00 S
0x01D0,//dpll_mon_th_sticky_0 0x00 S
0x01D1,//dpll_mon_tl_sticky_0 0x00 S
0x01D2,//dpll_mon_th_sticky_1 0x00 S
0x01D3,//dpll_mon_tl_sticky_1 0x00 S
0x01D4,//dpll_mon_th_sticky_2 0x00 S
0x01D5,//dpll_mon_tl_sticky_2 0x00 S
0x01D6,//dpll_mon_th_sticky_3 0x00 S
0x01D7,//dpll_mon_tl_sticky_3 0x00 S
0x01D8,//dpll_mon_th_sticky_4 0x00 S
0x01D9,//dpll_mon_tl_sticky_4 0x00 S
0x01DA,//dpll_mon_th_sticky_5 0x00 S
0x01DB,//dpll_mon_tl_sticky_5 0x00 S
0x01E0,//gp_mon_th_sticky 0x00 S
0x01E1,//gp_mon_tl_sticky 0x00 S
};

//Register Map Page 4, Control
u16 zl30793_reg_page_4[] = {
0x0200,// ref_los_3_0 0x00 R/W
0x0201,// ref_los_4 0x00 R/W
0x0203,// ref_sfm_clr_3_0 0x00 R/W
0x0204,// ref_sfm_clr_4 0x00 R/W
0x0205,// ref_freq_cmd 0x00 R/W
0x0206,// dpll_freq_cmd 0x00 R/W
0x0207,// split_xo_cmd 0x00 R/W
0x0208,// dpll_enable 0x03 R/W
0x0209,// split_xo_ctrl 0x00 R/W
0x020A,// split_xo_ref 0x09 R/W
0x020B,// ext_fb_ctrl 0x00 R/W
0x020C,// ext_fb_sel 0x00 R/W
0x020D,// dpll_meas_ref_freq_ctrl 0x41 R/W
0x0210,// dpll_mode_refsel_0 0x00 R/W
0x0211,// dpll_ctrl_0 0x02 R/W
0x0212,// dpll_cmd_0 0x00 R/W
0x0214,// dpll_mode_refsel_1 0x00 R/W
0x0215,// dpll_ctrl_1 0x02 R/W
0x0216,// dpll_cmd_1 0x00 R/W
0x0218,// dpll_mode_refsel_2 0x00 R/W
0x0219,// dpll_ctrl_2 0x02 R/W
0x021A,// dpll_cmd_2 0x00 R/W
0x021C,// dpll_mode_refsel_3 0x00 R/W
0x021D,// dpll_ctrl_3 0x00 R/W
0x021E,// dpll_cmd_3 0x00 R/W
0x0230,// phase_step_ctrl 0x00 R/W
0x0234,//:0x0237 phase_step_data 0x00000000 R/W
0x0235,//phase_step_data 0x00000000 R/W
0x0236,//phase_step_data 0x00000000 R/W
0x0237,//phase_step_data 0x00000000 R/W
0x0238,// phase_step_mask_gp 0x00 R/W
0x0239,// phase_step_mask_hp 0x00 R/W
0x023A,// step_time_mask_gp 0x00 R/W
0x023B,// step_time_mask_hp 0x00 R/W
0x023E,// phase_step_max 0x31 R/W
0x0240,// dpll_meas_ctrl 0x00 R/W
0x0241,// dpll_meas_idx 0x00 R/W
0x0242,// dpll_meas_ref_edge_3_0 0x00 R/W
0x0243,// dpll_meas_ref_edge_4 0x00 R/W
0x024C,// out_squelch_ctrl 0x00 R/W
0x024D,// gp_squelch_mask 0x00 R/W
0x024E,// hp_squelch_mask 0x00 R/W
0x025F,// pherr_read_rqst 0x00 R/W
0x0260,// dpll_phase_err_read_mask 0x00 R/W
0x0261,//:0x0266 dpll_phase_err_data_0 0x000000000000 R
0x0262,//dpll_phase_err_data_0 0x000000000000 R
0x0263,//dpll_phase_err_data_0 0x000000000000 R
0x0264,//dpll_phase_err_data_0 0x000000000000 R
0x0265,//dpll_phase_err_data_0 0x000000000000 R
0x0266,//dpll_phase_err_data_0 0x000000000000 R
0x0267,//:0x026C dpll_phase_err_data_1 0x000000000000 R
0x0268,// dpll_phase_err_data_1 0x000000000000 R
0x0269,// dpll_phase_err_data_1 0x000000000000 R
0x026A,// dpll_phase_err_data_1 0x000000000000 R
0x026B,// dpll_phase_err_data_1 0x000000000000 R
0x026C,// dpll_phase_err_data_1 0x000000000000 R
0x026D, // dpll_phase_err_data_2 0x000000000000 R
	//ZL30791, ZL30795, ZL30793 Data Sheet
0x026E,// dpll_phase_err_data_2 0x000000000000 R
0x026F,// dpll_phase_err_data_2 0x000000000000 R
0x0270,// dpll_phase_err_data_2 0x000000000000 R
0x0271,// dpll_phase_err_data_2 0x000000000000 R
0x0272,// dpll_phase_err_data_2 0x000000000000 R
0x0273,//:0x0278 dpll_phase_err_data_3 0x000000000000 R
0x0274,// dpll_phase_err_data_3 0x000000000000 R
0x0275,// dpll_phase_err_data_3 0x000000000000 R
0x0276,// dpll_phase_err_data_3 0x000000000000 R
0x0277,// dpll_phase_err_data_3 0x000000000000 R
0x0278,// dpll_phase_err_data_3 0x000000000000 R
0x027E,// uport 0x00 R/W
0x027F,// page_sel 0x00 R/W
};

//Register Map Page 6, Status 0x300
u16 zl30793_reg_page_6[] = {
0x0300,//:0x0305 dpll_df_offset_0 0x000000000000 R/W
0x0301,// dpll_df_offset_0 0x000000000000 R/W
0x0302,// dpll_df_offset_0 0x000000000000 R/W
0x0303,// dpll_df_offset_0 0x000000000000 R/W
0x0304,// dpll_df_offset_0 0x000000000000 R/W
0x0305,// dpll_df_offset_0 0x000000000000 R/W
0x0306,// dpll_df_ctrl_0 0x00 R/W
0x0307,//:0x030B dpll_df_manual_0 0x0000000000 R/W
0x0308,// dpll_df_manual_0 0x0000000000 R/W
0x0309,// dpll_df_manual_0 0x0000000000 R/W
0x030A,// dpll_df_manual_0 0x0000000000 R/W
0x030B,// dpll_df_manual_0 0x0000000000 R/W
0x0313,//:0x0316 dpll_tie_data_0 0x00000000 R/W
0x0314,// dpll_tie_data_0 0x00000000 R/W
0x0315,// dpll_tie_data_0 0x00000000 R/W
0x0316,// dpll_tie_data_0 0x00000000 R/W
0x0317,// dpll_tie_ctrl_0 0x00 R/W
0x0318,//:0x031D dpll_step_data_0 0x000000000000 R/W
0x0319,// dpll_step_data_0 0x000000000000 R/W
0x031A,// dpll_step_data_0 0x000000000000 R/W
0x031B,// dpll_step_data_0 0x000000000000 R/W
0x031C,// dpll_step_data_0 0x000000000000 R/W
0x031D,// dpll_step_data_0 0x000000000000 R/W
0x031E,// dpll_step_ctrl_0 0x00 R/W
0x0320,//:0x0325 dpll_df_offset_1 0x000000000000 R/W
0x0321,// dpll_df_offset_1 0x000000000000 R/W
0x0322,// dpll_df_offset_1 0x000000000000 R/W
0x0323,// dpll_df_offset_1 0x000000000000 R/W
0x0324,// dpll_df_offset_1 0x000000000000 R/W
0x0325,// dpll_df_offset_1 0x000000000000 R/W
0x0326,// dpll_df_ctrl_1 0x00 R/W
0x0327,//:0x032B dpll_df_manual_1 0x0000000000 R/W
0x0328,// dpll_df_manual_1 0x0000000000 R/W
0x0329,// dpll_df_manual_1 0x0000000000 R/W
0x032A,// dpll_df_manual_1 0x0000000000 R/W
0x032B,// dpll_df_manual_1 0x0000000000 R/W
0x0333,//:0x0336 dpll_tie_data_1 0x00000000 R/W
0x0334,//dpll_tie_data_1 0x00000000 R/W
0x0335,//dpll_tie_data_1 0x00000000 R/W
0x0336,//dpll_tie_data_1 0x00000000 R/W
0x0337,// dpll_tie_ctrl_1 0x00 R/W
0x0338,//:0x033D dpll_step_data_1 0x000000000000 R/W
0x0339,//dpll_step_data_1 0x000000000000 R/W
0x033A,//dpll_step_data_1 0x000000000000 R/W
0x033B,//dpll_step_data_1 0x000000000000 R/W
0x033C,//dpll_step_data_1 0x000000000000 R/W
0x033D,//dpll_step_data_1 0x000000000000 R/W
0x033E,// dpll_step_ctrl_1 0x00 R/W
0x0340,//:0x0345 dpll_df_offset_2 0x000000000000 R/W
0x0341,//dpll_df_offset_2 0x000000000000 R/W
0x0342,//dpll_df_offset_2 0x000000000000 R/W
0x0343,//dpll_df_offset_2 0x000000000000 R/W
0x0344,//dpll_df_offset_2 0x000000000000 R/W
0x0345,//dpll_df_offset_2 0x000000000000 R/W
0x0346,// dpll_df_ctrl_2 0x00 R/W
0x0347,//:0x034B dpll_df_manual_2 0x0000000000 R/W
0x0348,//dpll_df_manual_2 0x0000000000 R/W
0x0349,//dpll_df_manual_2 0x0000000000 R/W
0x034A,//dpll_df_manual_2 0x0000000000 R/W
0x034B,//dpll_df_manual_2 0x0000000000 R/W
0x0353,//:0x0356 dpll_tie_data_2 0x00000000 R/W
0x0354,//dpll_tie_data_2 0x00000000 R/W
0x0355,//dpll_tie_data_2 0x00000000 R/W
0x0356,//dpll_tie_data_2 0x00000000 R/W
0x0357,// dpll_tie_ctrl_2 0x00 R/W
0x0358,//:0x035D dpll_step_data_2 0x000000000000 R/W
0x0359,// dpll_step_data_2 0x000000000000 R/W
0x035A,// dpll_step_data_2 0x000000000000 R/W
0x035B,// dpll_step_data_2 0x000000000000 R/W
0x035C,// dpll_step_data_2 0x000000000000 R/W
0x035D,// dpll_step_data_2 0x000000000000 R/W
0x035E,// dpll_step_ctrl_2 0x00 R/W
0x0360,//:0x0365 dpll_df_offset_3
0x0361,// dpll_df_offset_3
0x0362,// dpll_df_offset_3
0x0363,// dpll_df_offset_3
0x0364,// dpll_df_offset_3
0x0365,// dpll_df_offset_3
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

static ssize_t zl30793_dpll_manual_freq_offset_show(struct device *dev,
						    struct device_attribute *attr,
						    char *buf)
{
	int ret = 0, len = 0, i = 0;
	static u8 dpll_df_data[5] = {0};
	static u16 dpll_df_addr = 0x0307;
	u16 *dma_buf;
	struct spi_device *spi = to_spi_device(dev);

	// Allocate DMA-safe buffer for transfers
	dma_buf = kmalloc(16, GFP_KERNEL);
	if (!dma_buf)
		return ret;

	for (i = 0; i < 5; i++) {
		ret = zl30793_spi_read(spi, dma_buf, (dpll_df_addr + i),
				       &dpll_df_data[i]);
		if (ret)
			dev_err(&spi->dev, "SPI read error %d\n", ret);
		dev_info(&spi->dev, "%s:  dpll_df_manual_0 addr = 0x%x  data = 0x%x\n",
			 __func__, (dpll_df_addr + i), dpll_df_data[i]);
	}

	kfree(dma_buf);
	return len;
}

static ssize_t
zl30793_dpll_manual_freq_offset_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t len)
{
	int ret = 0, i = 0;
	u8 dpll_df_data[5] = {0};
	s32 freq_offset = 0;
	s64 manual_offset = 0;
	s8* man_ptr = NULL;
	s8  pres_sign = 1;
	u16 dpll_df_addr = 0x307;
	u16 *dma_buf;
	struct spi_device *spi = to_spi_device(dev);

	sscanf(buf, "%d", &freq_offset);

	if (freq_offset < 0)
		/* preserve the sign of the number */	
		pres_sign = -1;

	/* if the sign is negative, overturn it prior to perform the arithmetic */
	manual_offset = pres_sign *
		( ((unsigned long)(freq_offset * pres_sign ) << 48 )/ 1000000000L);
	
	man_ptr = (s8*)&manual_offset + 4;

	// Allocate DMA-safe buffer for transfers
	dma_buf = kmalloc(PLL_SPI_MAX_FRAME_SIZE, GFP_KERNEL);
	if (!dma_buf)
		return FREQ_CTRL_ERROR_FAIL;

	for (i = 0; i < 5; i++) {
		dpll_df_data[i] = (u8)*man_ptr;
		ret = zl30793_spi_write(spi, dma_buf, (dpll_df_addr + i),
					&dpll_df_data[i]);
		if (ret) {
			dev_err(&spi->dev, "SPI select error %d\n", ret);
			goto zl_dpll_err;
		}
		man_ptr--;
	}

zl_dpll_err:
	kfree(dma_buf);
	return len;
}

static ssize_t
zl30793_reg_dump_show(struct device *dev,
		      struct device_attribute *attr, char *buf)
{
	int ret = 0, i;
	u8  val;
	u16 *dma_buf;
	struct spi_device *spi = to_spi_device(dev);

	// Allocate DMA-safe buffer for transfers
	dma_buf = kmalloc(16, GFP_KERNEL);
	if (!dma_buf)
		return ret;

	pr_info("\n\nzl30793_clock_reg_dump()  page 0 General Info\n");
	for (i = 0 ; i < (sizeof(zl30793_reg_page_0) / sizeof(u16)); i++) {
		ret = zl30793_spi_read(spi, dma_buf, zl30793_reg_page_0[i], &val);
		if (ret) {
			dev_err(&spi->dev, "SPI read error %d\n", ret);
			goto zl_reg_dump_err;
		}
		pr_info("page 0[%d] addr 0x%x  0x%x\n", i, zl30793_reg_page_0[i], val);
	}

	pr_info("\n\nzl30793_clock_reg_dump()  page 2 Status\n");
	for (i = 0 ; i < (sizeof(zl30793_reg_page_2) / sizeof(u16)); i++) {
		ret = zl30793_spi_read(spi, dma_buf, zl30793_reg_page_2[i], &val);
		if (ret) {
			dev_err(&spi->dev, "SPI read error %d\n", ret);
			goto zl_reg_dump_err;
		}
		pr_info("page 2[%d] addr 0x%x  0x%x\n", i, zl30793_reg_page_2[i], val);
	}

	pr_info("\n\nzl30793_clock_reg_dump()  page 3 sticky\n");
	for (i = 0 ; i < (sizeof(zl30793_reg_page_3) / sizeof(u16)); i++) {
		ret = zl30793_spi_read(spi, dma_buf, zl30793_reg_page_3[i], &val);
		if (ret) {
			dev_err(&spi->dev, "SPI read error %d\n", ret);
			goto zl_reg_dump_err;
		}
		pr_info("page 3[%d] addr 0x%x  0x%x\n", i, zl30793_reg_page_3[i], val);
	}

	pr_info("\n\nzl30793_clock_reg_dump()  page 4 control\n");
	for (i = 0 ; i < (sizeof(zl30793_reg_page_4) / sizeof(u16)); i++) {
		ret = zl30793_spi_read(spi, dma_buf, zl30793_reg_page_4[i], &val);
		if (ret) {
			dev_err(&spi->dev, "SPI read error %d\n", ret);
			goto zl_reg_dump_err;
		}
		pr_info("page 4[%d] addr 0x%x  0x%x\n", i, zl30793_reg_page_4[i], val);
	}

	pr_info("\n\nzl30793_clock_reg_dump()  page 6 DPLL\n");
	for (i = 0 ; i < (sizeof(zl30793_reg_page_6) / sizeof(u16)); i++) {
		ret = zl30793_spi_read(spi, dma_buf, zl30793_reg_page_6[i], &val);
		if (ret) {
			dev_err(&spi->dev, "SPI read error %d\n", ret);
			goto zl_reg_dump_err;
		}
		pr_info("page 6[%d] addr 0x%x  0x%x\n", i, zl30793_reg_page_6[i], val);
	}

zl_reg_dump_err:
	kfree(dma_buf);
	return ret;
}

static DEVICE_ATTR(zl30793_dpll_manual_freq_offset, 0644,
		   zl30793_dpll_manual_freq_offset_show,
		   zl30793_dpll_manual_freq_offset_store);
static DEVICE_ATTR(zl30793_reg_dump, 0644, zl30793_reg_dump_show, NULL);

static struct attribute *zl30793_sysfs_attrs[] = {
	&dev_attr_zl30793_dpll_manual_freq_offset.attr,
	&dev_attr_zl30793_reg_dump.attr,
	NULL
};

static const struct attribute_group zl30793_attr_group = {
	.attrs = zl30793_sysfs_attrs,
};

int zl30793_sysfs_configure(struct spi_device *spi)
{
	if (spi) {
		struct device *dev = &spi->dev;
		struct kobject *kobj = &dev->kobj;

		/* force an update based on parameters read from the device */
		sysfs_update_group(kobj, &zl30793_attr_group);
	}
	return 0;
}