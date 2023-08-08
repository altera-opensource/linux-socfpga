// SPDX-License-Identifier: GPL
/* Intel zarlink i2c driver
 * Copyright (C) 2023 Intel Corporation. All rights reserved
 *
 * Contributors:
 *
 *	Inian Pavel Sakthi <inian.pavel.sakthi@intel.com>
 *
 */

#include <linux/init.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/spi/spi.h>
#include "intel_freq_control.h"
#include "intel_freq_control_zl30733_debugfs.h"
#include "intel_freq_ctrl_zl30733_i2c.h"

u8 i2c_zl30733_write_byte_data(const struct i2c_client *client, u16 reg, u8 data[], u8 data_len)
{
	int status;
	int ret = 0;
	u8 reg_buf[2], buf[16];
	struct i2c_msg msg[2] = {0};

	// select page:
	reg_buf[0] = ZL30733_ADDR_ADDR(ZL30733_REG_PAGE_SEL);
	reg_buf[1] = ZL30733_ADDR_TO_PAGE(reg);

	msg[0].addr = client->addr;
	msg[0].len  = 2;
	msg[0].buf  = reg_buf;

	// burst write data to reg:
	buf[0] = ZL30733_ADDR_ADDR(reg);
	memcpy(&buf[1], data, data_len);
	msg[1].addr = client->addr;
	msg[1].len  = 1 + data_len;
	msg[1].buf  = buf;

	status = i2c_transfer(client->adapter, msg, 2);
	if (status != 2) {
		pr_alert("%s: ZL30733 i2c write failed", __func__);
		ret = 1;
	}
	return ret;
}

u8 i2c_zl30733_read_byte_data(const struct i2c_client *client, u16 reg, u8 *buf, u8 data_len)
{
	int status;
	int ret = 0;
	u8 reg_buf[3];
	struct i2c_msg msg[3] = {0};

	// select page:
	reg_buf[0] = ZL30733_ADDR_ADDR(ZL30733_REG_PAGE_SEL);
	reg_buf[1] = ZL30733_ADDR_TO_PAGE(reg);

	msg[0].addr = client->addr;
	msg[0].len  = 2;
	msg[0].buf  = reg_buf;

	// burst read data from reg:
	reg_buf[2] = ZL30733_ADDR_ADDR(reg);

	msg[1].addr = client->addr;
	msg[1].len = 1;
	msg[1].buf = &reg_buf[2];

	msg[2].addr = client->addr;
	msg[2].flags = I2C_M_RD;
	msg[2].len = data_len;
	msg[2].buf = buf;

	status = i2c_transfer(client->adapter, msg, 3);
	if (status != 3) {
		pr_alert("%s: ZL30733 i2c read failed", __func__);
		ret = 1;
	}
	return ret;
}

static int zl30733_dpll_wait_for_df_ctrl_readsem(struct i2c_client *i2c_cli)
{
	u8 ctrl_val;
	int ret = 0;
	u8 attempts = 0;

	/*Short timeout when accessing registers that should return quickly*/
	u32 zl30733_short_timeout = 100;

	if (!i2c_cli)
		return FREQ_CTRL_ERROR_FAIL;

	do {
		// read : ZL30793_PAGE6_REG_DPLL_DF_CTRL_0
		ret = i2c_zl30733_read_byte_data(i2c_cli, ZL30733_PAGE5_REG_DPLL_DF_READ_0,
						 &ctrl_val, 1);
		if (ret) {
			dev_err(&i2c_cli->dev, "i2c read error %d\n", ret);
			ret = FREQ_CTRL_ERROR_FAIL;
			goto zl_dpll_readsem_err;
		}

		if (ZL30733_DPLL_DF_CTRL_SEM_GET(ctrl_val) == 0) {
			dev_dbg(&i2c_cli->dev,
				"%s: pll: ZL30733_PAGE5_REG_DPLL_DF_READ_0[0x%x] read : 0x%x\n",
				__func__, ZL30733_PAGE5_REG_DPLL_DF_READ_0, ctrl_val);

			ret = FREQ_CTRL_ERROR_SUCCESS;
			goto zl_dpll_readsem_err;
		}

		dev_info(&i2c_cli->dev,
			 "%s: pll: ZL30733_PAGE5_REG_DPLL_DF_READ_0[0x%x] read : 0x%x attempt %d\n",
			 __func__, ZL30733_PAGE5_REG_DPLL_DF_READ_0, ctrl_val, attempts);

		/* Device is not ready, wait a few ms */
		udelay(ZL30733_REG_READ_INTERVAL);

		if (++attempts > (zl30733_short_timeout / ZL30733_REG_READ_INTERVAL)) {
			dev_info(&i2c_cli->dev, "%s timeout!\n", __func__);
			ret = FREQ_CTRL_ERROR_FAIL;//timeout
			goto zl_dpll_readsem_err;
		}

	} while (1);

zl_dpll_readsem_err:

	return ret;
}

void intel_freq_control_zl30733(struct work_struct *work)
{
	struct freq_work *p_work;
	long scaled_ppm;
	u64 df_offset;
	short int i;
	u8 data[6];
	struct intel_freq_control_private *priv;
	struct i2c_client *i2c_cli;

	p_work = container_of(work, struct freq_work, w);
	scaled_ppm = p_work->scaled_ppm;
	priv = container_of(p_work, struct intel_freq_control_private, queued_work);

	i2c_cli = priv->fc_acc_type.i2c_cli;
	/* dpll_df_offset_2 contains a 2's complement binary value of delta frequency offset.
	 * The register controls delta frequency of synthesizers that are associated with DPLL0.
	 * Delta frequency is expressed in steps of +/- 2^-48 of nominal setting.
	 * The value to write in NCO mode is:
	 * df_offset = (-X) * 2^48
	 * where X is the desired offset from nominal. E.g., for a
	 * -1 ppm offset, X=-1e-6, df_offset=0x000010C6F7A0
	 * Note 1: The delta frequency offset should not exceed +/-1% of the nominal value
	 * Note 2: This register can be written as fast as once per 600us, but no faster.
	 * Note 3: This register should not be written while a read operation is pending.
	 * Note 4: The read value of this register during NCO mode is the value latched just prior
	 * to entering NCO mode.
	 */
	// ppm is scaled_ppm >> 16, hence << 32 below due to 48-16=32
	df_offset = (((u64)(abs(scaled_ppm))) << 32) / 1000000;
	if (scaled_ppm > 0)
		df_offset = 0 - df_offset;

	if (zl30733_dpll_wait_for_df_ctrl_readsem(i2c_cli) == 0) {
		// Write df_offset into DPLL_DF_OFFSET_0 register:
		for (i = 0; i < 6; ++i) {
			data[i] = df_offset >> (40 - 8 * i);
			(void)i2c_zl30733_write_byte_data(i2c_cli,
							  ZL30733_REG_DPLL_DF_OFFSET_0_0 + i,
							  &data[i], 1);
		}
		pr_devel("%s:ZL30733  scaled ppm %lx  writing data %x-%x-%x-%x-%x-%x in work queue",
			 __func__, scaled_ppm, data[0], data[1], data[2], data[3], data[4], data[5]);
	}
	//(void)i2c_zl30733_write_byte_data(i2c_cli, ZL30733_REG_DPLL_DF_OFFSET_0_0, data, 6);
}

static void zl30733_check_dpll(struct i2c_client *i2c_cli)
{
	u8 i = 1, buf; // DPLL1 is being used
	int loop;
	int ret = 0;

	pr_info("%s: Wait for ZL30733 DPLL1 lock...\n", __func__);
	for (loop = 0; loop < 10000; ++loop) {
		ret = i2c_zl30733_read_byte_data(i2c_cli, ZL30733_REG_DPLL_MON_STATUS(i), &buf, 1);
		if (ret == 0 && (buf & 0x1)) {
			pr_info("ZL30733_REG_DPLL_MON_STATUS(%u): 0x%02x - OK, lock\n", i, buf);
			return;
		}
		mdelay(10);
	}
	pr_err("%s: ZL30733_REG_DPLL_MON_STATUS(%u) : 0x%02x - FAIL, timeout waiting for lock\n",
	       __func__, i, buf);
}

static int zl30733_dpll_nco_modeset(struct i2c_client *i2c_cli)
{
	u8 ctrl_val;
	int ret;

	if (!i2c_cli)
		return FREQ_CTRL_ERROR_FAIL;

	/* read : ZL30733 dpll_mode_refsel_0 - 0x284*/
	ret = i2c_zl30733_read_byte_data(i2c_cli, ZL30733_REG_DPLL_MODE_REFSEL_0, &ctrl_val, 1);
	if (ret) {
		dev_err(&i2c_cli->dev, "I2c readt error %d\n", ret);
		goto zl_ctrl_err;
	}
	if (ZL30733_DPLL_CHECK_NCO_MODE(ctrl_val)) {
		dev_info(&i2c_cli->dev, "ZL30733 is already in NCO mode (0x%x)\n", ctrl_val);
		ret = FREQ_CTRL_ERROR_SUCCESS;
		goto zl_ctrl_err;
	}

	dev_info(&i2c_cli->dev, "ZL30733 is not in NCO mode(0x%x); set Dpll in NCO mode\n",
		 ctrl_val);
	ctrl_val = ZL30733_DPLL_SET_NCO_MODE(ctrl_val);
	ret = i2c_zl30733_write_byte_data(i2c_cli, ZL30733_REG_DPLL_MODE_REFSEL_0, &ctrl_val, 1);
	if (ret) {
		dev_err(&i2c_cli->dev, "I2C write error %d\n", ret);
		goto zl_ctrl_err;
	}

zl_ctrl_err:
	return ret;
}

int i2c_dev_check_zl30733_clock(struct intel_freq_control_private *priv)
{
	struct i2c_client *i2c_cli = NULL;
	int ret = FREQ_CTRL_ERROR_SUCCESS;
	u8 rdbuf[2];

	i2c_cli = priv->fc_acc_type.i2c_cli;
	if (!i2c_cli)
		return FREQ_CTRL_ERROR_FAIL;

	ret = i2c_zl30733_read_byte_data(i2c_cli, ZL30733_REG_ID_0, rdbuf, sizeof(rdbuf));
	if (ret == 0) {
		pr_info("%s: i2c_zl30733_read_byte_data read ID:0x%02x%02x\n", __func__,
			rdbuf[0], rdbuf[1]);

		if (((rdbuf[0] << 8) | rdbuf[1]) == ZL30733_ID_VALUE) {
			zl30733_check_dpll(i2c_cli);
			zl30733_dpll_nco_modeset(i2c_cli);
#ifdef CONFIG_DEBUG_FS
			zl30733_dbgfs_init(i2c_cli);
#endif
		}
	}

	return ret;
}
