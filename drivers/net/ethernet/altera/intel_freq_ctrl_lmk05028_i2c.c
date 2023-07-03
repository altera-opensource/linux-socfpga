// SPDX-License-Identifier: GPL
/* Intel FPGA I2C driver 
 * Copyright (C) 2023 Intel Corporation. All rights reserved
 *
 * Contributors:
 *
 */


#include "intel_freq_control.h"
#include "intel_freq_ctrl_lmk05028_i2c.h"

static u8 i2c_lmk05028_write_byte_data(const struct i2c_client *client,
                                       u16 reg, u8 data)
{
        int status;
        int ret = 0;
        u8 reg_buf[3];
        struct i2c_msg msg = {0};

        reg_buf[0] = (reg & 0xff00) >> 8;
        reg_buf[1] = (reg & 0xff);
        reg_buf[2] = data;

        msg.addr = client->addr;
        msg.len = 3;
        msg.buf = reg_buf;

        status = i2c_transfer(client->adapter, &msg, 1);
        if (status != 1) {
                pr_alert("LMK05028 i2c write failed");
                ret = 1;
        }
        return ret;
}

static u8 i2c_lmk05028_read_byte_data(const struct i2c_client *client,
                                      u16 reg, u8 *buf)
{
        int status;
        int ret = 0;
        u8 reg_buf[2];
        struct i2c_msg msg[2] = {0};

        reg_buf[0] = (reg & 0xff00) >> 8;
        reg_buf[1] = (reg & 0xff);

        msg[0].addr = client->addr;
        msg[0].len = 2;
        msg[0].buf = reg_buf;

        msg[1].addr = client->addr;
        msg[1].flags = I2C_M_RD;
        msg[1].len = 1;
        msg[1].buf = buf;

        status = i2c_transfer(client->adapter, msg, 2);
        if (status != 2) {
                pr_alert("LMK05028 i2c read failed");
                ret = 1;
        }
        return ret;
}

static void lmk05028_read_constant_values(struct i2c_client *i2c_cli,
		u16 *r_in, u8 *post_div_val, u8* prescaler_div_val,
		u64 *constant_results, u64 *denominator)
{
        short int i;
        u8 temp                  = 0;
        u16 ref_den_reg          =  LMK05028_REG_PLL2_REF_DENH;

        i2c_lmk05028_read_byte_data(i2c_cli, LMK05028_REG_PLL2_REF1_RH, &temp);
        temp &= 0xff;
        *r_in = temp;
        *r_in <<= 8;
        i2c_lmk05028_read_byte_data(i2c_cli, LMK05028_REG_PLL2_REF1_RH + 1,
                                    &temp);
        temp &= 0xff;
        *r_in |= temp;

        i2c_lmk05028_read_byte_data(i2c_cli, LMK05028_REG_PLL2_P2_P1,
                                    post_div_val);
        *post_div_val &= 0xf;

        i2c_lmk05028_read_byte_data(i2c_cli, LMK05028_REG_PLL2_REF_PR,
                                    prescaler_div_val);

        *prescaler_div_val &= 0x0f;
        *prescaler_div_val += 2;

        switch (*post_div_val) {
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 0xa:
        case 0xc:
                *post_div_val += 1;
                break;
        default:
                pr_err("not a valid primary post-divide value ");
        }
        /* Read Denominator value of PLL2 */
        for (i = 0, temp = 0, *denominator = 0; i < 5; i++, ref_den_reg++) {
                i2c_lmk05028_read_byte_data(i2c_cli, ref_den_reg, &temp);
                *denominator <<= 8;
                *denominator |= ((u64)temp & 0xff);
        }
        if (!*denominator)
                *denominator = int_pow(2, 40);

        *constant_results = (LMK05028_PLL2_FVCO * (*denominator) / LMK05028_PLL2_FIN1);
        *constant_results *= *r_in;
        *constant_results /= ((*post_div_val) * (*prescaler_div_val));

        pr_info("Reading LMK05028 FDEV constants done!");
}

void intel_freq_control_lmk05028(struct work_struct *work)
{
        short int i;
        u8 temp = 0;
	u8  post_div_val = 0, prescaler_div_val;
	u16 r_in = 0;
        u64 step_val = 0;
        u16 pll2_fdev_reg = LMK05028_REG_PLL2_FDEVH;
        long scaled_ppm;
	u64 constant_results = 0;
	u64 denominator = 0;
        struct freq_work *p_work;
        static bool previous_sign;
	struct intel_freq_control_private *priv;
	struct i2c_client * i2c_cli;

        p_work = container_of(work, struct freq_work, w);
	priv = container_of(p_work, struct intel_freq_control_private,
			queued_work);

	i2c_cli = priv->fc_acc_type.i2c_cli;

        // if (!(r_in && post_div_val && prescaler_div_val && denominator))  -- condition necessity to be identified 
        lmk05028_read_constant_values(i2c_cli, &r_in,
				&post_div_val,
				&prescaler_div_val,
				&constant_results,
				&denominator);

        scaled_ppm = p_work->scaled_ppm;

        step_val = (constant_results) >> 16;
        step_val *= abs(scaled_ppm);
        /* 10^9 from LMK equation for PPB its scaled down by 10^3 for PPM */
        step_val /= int_pow(10, 6);

        /* negating the previous ppb update to return to the nominal
         * frequency before applying new ppb update
         */
        if (!previous_sign) {
                /* Now decrementing, because we had previously
                 * incremented the existing FDEV value
                 */
                i2c_lmk05028_write_byte_data(i2c_cli, LMK05028_REG_PLL2_FSTEP, 0x01);

        } else {
                /* Now incrementing, because we had previously
                 * decremented the existing FDEV value
                 */
                i2c_lmk05028_write_byte_data(i2c_cli, LMK05028_REG_PLL2_FSTEP, 0x00);
        }

        /*Write new step value into DPLL2_FDEV registers*/
        for (i = 4; i >= 0; i--, pll2_fdev_reg++) {
                temp = (step_val >> (i * 8)) & 0xff;
                i2c_lmk05028_write_byte_data(i2c_cli, pll2_fdev_reg, temp);
        }

        if (scaled_ppm < 0) {
                /* Decrement Frequency */
                i2c_lmk05028_write_byte_data(i2c_cli, LMK05028_REG_PLL2_FSTEP, 0x01);
                previous_sign = 1;
        } else {
                /* Increment Frequency */
                i2c_lmk05028_write_byte_data(i2c_cli, LMK05028_REG_PLL2_FSTEP, 0x00);
                previous_sign = 0;
        }
}