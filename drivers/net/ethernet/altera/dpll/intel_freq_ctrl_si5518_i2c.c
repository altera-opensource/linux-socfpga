// SPDX-License-Identifier: GPL
/* Altera Skyworks si5518 i2c driver
 * Copyright (C) 2023 Intel Corporation. All rights reserved
 *
 * Contributors:
 *	Lubana Badakar <lubana.badakar@altera.com>
 */

#include "../intel_freq_control.h"
#include "intel_freq_ctrl_si5518_i2c.h"
#include "intel_freq_ctrl_common_i2c.h"

/* TODO: Sometimes the i2c read response is not valid */
#define SI5518_I2C_WA		1
static u32 si5518_ma_step_size;

//#define SYNC_TIMING_MAX_DATA_TRANSFER_SIZE	255
#define SI5518_PLL_I2C_MAX_FRAME_SIZE	64

#define SI5518_PLL_DEVICE_ID		0x5518

#define SI5518_PLL_CMD_I2C_REPLY	0xFF0
#define SI5518_PLL_CMD_I2C_LO 		(SI5518_PLL_CMD_I2C_REPLY & 0xff)
#define SI5518_PLL_CMD_I2C_HI 		((SI5518_PLL_CMD_I2C_REPLY >> 8) & 0xff)
#define SI5518_PLL_CMD_REPLY_CTS_STATUS	0x80
#define SI5518_PLL_CMD_DEVICE_INFO	0x08
#define SI5518_PLL_CMD_METADATA		0x15
#define SI5518_PLL_CMD_PLL_STATUS	0x13
#define SI5518_CMD_INPUT_STATUS	0x12
#define SI5518_MANUAL_INPUT_CLOCK_SELECT	0x23
#define SI5518_PLL_CMD_VARIABLE_OFFSET_DCO 0x24
#define SI5518_PLL_FORCE_HOLDOVER		0x25

#define SI5518_LOCK_CHECK_INTERVAL_IN_MS (500)
#define SI5518_DPLL_IS_LOCKED(data)      ((data) & 0x01)
#define SI5518_MAX_PLL_LOCK_CHECK_COUNTER   (200)

#define SI5518_RFPLL	0x1
#define SI5518_DSPLLA	0x2
#define SI5518_DSPLLB	0x4
#define SI5518_PPSPLL	0x80

#define SI5518_INPUT_IN0	0x0
#define SI5518_INPUT_IN1	0x2
#define SI5518_INPUT_IN2	0x4
#define SI5518_INPUT_IN2B	0x5
#define SI5518_INPUT_IN3	0x6
#define SI5518_INPUT_IN3B	0x7

#define SI5518_NO_HOLDOVER		0x0
#define SI5518_FORCE_HOLDOVER	0x1

#define SI5518_PLL_STATUS_FORCE_HOLDOVER	(0x1 << 1)
#define SI5518_PLL_STATUS_HOLDOVER 			0x1
#define SI5518_PLL_STATUS_INITIAL_LOCK		(0x1 << 5)
#define SI5518_PLL_STATUS_LOL				(0x1 << 4)
#define SI5518_PLL_OUT_OF_PHASE				(0x1 << 2)
#define SI5518_PLL_OUT_OF_FREQ				(0x1 << 1)

/* TODO: change the return type */
static int i2c_si5518_write(const struct i2c_client *client, u8 txbuf[], u8 txlen)
{
	int status;
	int ret = 0;
	struct i2c_msg msg = {0};

	msg.addr = client->addr;
	msg.len  = txlen;
	msg.buf  = txbuf;

	status = i2c_transfer(client->adapter, &msg, 1);
	if (status != 1) {
		pr_alert("%s: SI5518 i2c write failed: %d\n", __func__, status);
		ret = FREQ_CTRL_ERROR_FAIL;
	}

	return ret;
}

static int i2c_si5518_read(const struct i2c_client *client, u8 rxbuf[], u8 rxlen)
{
	int status;
	int count = 3;
	struct i2c_msg msg[2] = {0};
	u8 txbuf[2] = {SI5518_PLL_CMD_I2C_LO, SI5518_PLL_CMD_I2C_HI};

	do {
		msg[0].addr = client->addr;
		msg[0].len  = 2;
		msg[0].buf  = txbuf;

		msg[1].addr = client->addr;
		msg[1].flags = I2C_M_RD;
		msg[1].len = rxlen;
		msg[1].buf = rxbuf;

		status = i2c_transfer(client->adapter, msg, 2);
		if (status != 2) {
			pr_alert("%s %d: SI5518 i2c read failed", __func__, status);
			return FREQ_CTRL_ERROR_FAIL;
		}

		count--;
		if (rxbuf[0] == SI5518_PLL_CMD_REPLY_CTS_STATUS)
			return 0;

	} while(count);

	return FREQ_CTRL_ERROR_FAIL;
}

static int i2c_si5518_write_with_read_response(const struct i2c_client *client, u8 *txbuf, u8 txlen, u8 *rxbuf, u8 rxlen)
{
	int ret = 0;
#if SI5518_I2C_WA

	ret = i2c_si5518_write(client, txbuf, txlen);
	if (ret) {
		dev_err(&client->dev, "%s: failed to write in2b command %d\n", __func__, ret);
		return FREQ_CTRL_ERROR_FAIL;
	}

	ret = i2c_si5518_read(client, rxbuf, rxlen);
	if (ret) {
		dev_err(&client->dev, "%s: failed to read reply for i2b command %d\n", __func__,ret);
		return FREQ_CTRL_ERROR_FAIL;
	}
#else
	struct i2c_msg msg[2] = {0};

	msg[0].addr = client->addr;
	msg[0].len  = txlen;
	msg[0].buf  = txbuf;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = rxlen;
	msg[1].buf = rxbuf;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != 2) {
		pr_alert("%s %d: SI5518 i2c read failed", __func__, status);
		ret = 1;
	}
#endif
	return ret;
}

void intel_freq_control_i2c_si5518(struct work_struct *work)
{
	struct freq_work *p_work;
	u8 cmd[8] = {SI5518_PLL_CMD_I2C_LO,
		     SI5518_PLL_CMD_I2C_HI,
		     SI5518_PLL_CMD_VARIABLE_OFFSET_DCO,
		     0x04, // VARIABLE_OFFSET_DCO, MA divider select
		     0,
		     0,
		     0,
		     0}; // VARIABLE_OFFSET_DCO, MA divider select
	long scaled_ppm;
	u32 num_steps;
	int ret;
	u8 *rxbuf;
	struct intel_freq_control_private *priv;
	struct i2c_client *i2c_cli;

	p_work = container_of(work, struct freq_work, w);
	scaled_ppm = p_work->scaled_ppm;
	priv = container_of(p_work, struct intel_freq_control_private, queued_work);
	if (priv->scaled_ppm_programmed == scaled_ppm)
		return;

	i2c_cli = priv->fc_acc_type.i2c_cli;

	if (!i2c_cli)
		return;

	if (priv->pll_lock_check_ctr) {
		dev_dbg(&i2c_cli->dev, "%s: ERROR - SI5518 PLL OOL, freq adjust exiting!",__func__ );
		return;
	}

	/* Allocate DMA-safe buffer for transfers */
	rxbuf = kmalloc(SI5518_PLL_I2C_MAX_FRAME_SIZE, GFP_KERNEL);
	if (!rxbuf)
		return;

	if (abs(scaled_ppm) > (10 << 16))
		dev_warn(&i2c_cli->dev, "%s: scaled_ppm:%li (ppm:%li) outside range of +/-10ppm\n",
			 __func__, scaled_ppm, scaled_ppm / (1 << 16));
	// unit of si5518_ma_step_size is ppt, 1ppm = 1000000ppt
	// scaled_ppm is ppm with 16 fractional bits
	num_steps = ((abs(scaled_ppm) << 1) * 1000000) / (si5518_ma_step_size << 16);
	num_steps = (num_steps >> 1) + (num_steps & 1);
	if (scaled_ppm < 0)
		num_steps = 0 - num_steps;
	//dev_info(&i2c_cli->dev, "%s: scaled_ppm:%li (ppm:%li) si5518_ma_step_size:%u num_steps:%i\n",
	//       __func__, scaled_ppm, scaled_ppm / (1 << 16), si5518_ma_step_size, (int32_t)num_steps);
	cmd[4] = (num_steps >> 0)  & 0xff;
	cmd[5] = (num_steps >> 8)  & 0xff;
	cmd[6] = (num_steps >> 16) & 0xff;
	cmd[7] = (num_steps >> 24) & 0xff;

	memset(rxbuf, 0, SI5518_PLL_I2C_MAX_FRAME_SIZE);
	ret = i2c_si5518_write(i2c_cli, cmd, 8);
	if (ret) {
		dev_err(&i2c_cli->dev, "%s: I2C write error %d\n", __func__, ret);
                ret = FREQ_CTRL_ERROR_FAIL;
		goto si_freq_ctrl_err;
	}

	ret = i2c_si5518_read(i2c_cli, rxbuf, SI5518_PLL_I2C_MAX_FRAME_SIZE);
	if (ret) {
		dev_err(&i2c_cli->dev, "%s: I2C read error %d\n", __func__, ret);
                ret = FREQ_CTRL_ERROR_FAIL;
		goto si_freq_ctrl_err;
	}

	if (rxbuf[0] != SI5518_PLL_CMD_REPLY_CTS_STATUS) {
		dev_info(&i2c_cli->dev, "%s: SI5518_PLL_CMD_VARIABLE_OFFSET_DCO reply: 0x%02x 0x%02x 0x%02x\n - FAIL", __func__,
			rxbuf[0], rxbuf[1], rxbuf[2]);
		dev_err(&i2c_cli->dev, "%s: ERROR - SI5518_PLL_CMD_VARIABLE_OFFSET_DCO CTS not set reply not ready\n", __func__);
                ret = FREQ_CTRL_ERROR_FAIL;
		goto si_freq_ctrl_err;
	}

	priv->scaled_ppm_programmed = scaled_ppm;

si_freq_ctrl_err:
	kfree(rxbuf);
}
EXPORT_SYMBOL(intel_freq_control_i2c_si5518);

bool si5518_clock_pre_modify_check(struct intel_freq_control_private *fq,
				   long scaled_ppm)
{
	return scaled_ppm < (10 << 16) ? true : false;
}
EXPORT_SYMBOL(si5518_clock_pre_modify_check);

static int si5518_i2c_dco_centering(struct i2c_client *i2c_cli)
{
	int ret = FREQ_CTRL_ERROR_SUCCESS;
	u8 *rxbuf;
	u8 cmd[8] = {SI5518_PLL_CMD_I2C_LO,
		     SI5518_PLL_CMD_I2C_HI,
		     SI5518_PLL_CMD_VARIABLE_OFFSET_DCO,
		     0x04, // VARIABLE_OFFSET_DCO, MA divider select
		     0,
		     0,
		     0,
		     0};

	if (!i2c_cli)
		return FREQ_CTRL_ERROR_FAIL;

        /* Allocate DMA-safe buffer for transfers */
        rxbuf = kmalloc(SI5518_PLL_I2C_MAX_FRAME_SIZE, GFP_KERNEL);
        if (!rxbuf)
                return FREQ_CTRL_ERROR_FAIL;

        memset(rxbuf, 0, SI5518_PLL_I2C_MAX_FRAME_SIZE);

	ret = i2c_si5518_write(i2c_cli, cmd, 8);
	if (ret) {
		dev_err(&i2c_cli->dev, "I2C write error %d\n", ret);
                ret = FREQ_CTRL_ERROR_FAIL;
		goto si_dco_err;
	}

	ret = i2c_si5518_read(i2c_cli, rxbuf, SI5518_PLL_I2C_MAX_FRAME_SIZE);
	if (ret) {
		dev_err(&i2c_cli->dev, "I2C read error %d\n", ret);
                ret = FREQ_CTRL_ERROR_FAIL;
		goto si_dco_err;
	}

	if (rxbuf[0] != SI5518_PLL_CMD_REPLY_CTS_STATUS) {
		dev_info(&i2c_cli->dev, "%s: SI5518_PLL_CMD_VARIABLE_OFFSET_DCO reply: 0x%02x 0x%02x 0x%02x - FAIL\n", __func__,
			rxbuf[0], rxbuf[1], rxbuf[2]);
		dev_err(&i2c_cli->dev, "%s: ERROR - SI5518_PLL_CMD_VARIABLE_OFFSET_DCO CTS not set reply not ready\n", __func__);
                ret = FREQ_CTRL_ERROR_FAIL;
		goto si_dco_err;
	}

	dev_info(&i2c_cli->dev, "%s: SI5518_PLL_CMD_VARIABLE_OFFSET_DCO(0x24) reply: 0x%02x 0x%02x 0x%02x 0x%02x\n",
			__func__, rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3]);
	ret = FREQ_CTRL_ERROR_SUCCESS;

si_dco_err :
	kfree(rxbuf);
	return ret;
}

static void rfpll_lock_handler(struct work_struct *work)
{
	struct i2c_client *i2c_cli = NULL;
	struct intel_freq_control_private *priv;
	struct delayed_work *dwork;
	int ret = FREQ_CTRL_ERROR_SUCCESS;
	u8 pll_sel = 1;
	u8 loop = 0;
	u8 *rxbuf;
	u8 cmd[8] = {SI5518_PLL_CMD_I2C_LO,
		     SI5518_PLL_CMD_I2C_HI,
		     SI5518_PLL_CMD_METADATA};

	dwork = to_delayed_work(work);
	priv = container_of(dwork, struct intel_freq_control_private, pll_lock_dwork);
	i2c_cli = priv->fc_acc_type.i2c_cli;

        /* Allocate DMA-safe buffer for transfers */
        rxbuf = kmalloc(SI5518_PLL_I2C_MAX_FRAME_SIZE, GFP_KERNEL);
        if (!rxbuf)
                return;

        memset(rxbuf, 0, SI5518_PLL_I2C_MAX_FRAME_SIZE);

	ret = i2c_si5518_write(i2c_cli, cmd, 3);
	if (ret) {
		dev_err(&i2c_cli->dev, "I2C write error %d\n", ret);
                ret = FREQ_CTRL_ERROR_FAIL;
		goto si_pll_lock_ret;
	}

	ret = i2c_si5518_read(i2c_cli, rxbuf, SI5518_PLL_I2C_MAX_FRAME_SIZE);
	if (ret) {
		dev_err(&i2c_cli->dev, "I2C read error %d\n", ret);
                ret = FREQ_CTRL_ERROR_FAIL;
		goto si_pll_lock_ret;
	}

	if (rxbuf[0] != SI5518_PLL_CMD_REPLY_CTS_STATUS) {
		dev_info(&i2c_cli->dev, "%s: METADATA reply: %02x %02x %02x %02x - FAIL\n",
			 __func__, rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3]);
		dev_err(&i2c_cli->dev, "%s: ERROR - METADATA CTS not set reply not ready\n", __func__);
                ret = FREQ_CTRL_ERROR_FAIL;
		goto si_pll_lock_ret;
	}

	si5518_ma_step_size = rxbuf[9] | (rxbuf[10] << 8) | (rxbuf[11] << 16) | (rxbuf[12] << 24);
	dev_info(&i2c_cli->dev, "%s: Si5518 found, read ma_step_size=%u\n", __func__, si5518_ma_step_size);

	// check RFPLL_STATUS
	for (loop = 0; loop < 2; ++loop) {
		cmd[0] = SI5518_PLL_CMD_I2C_LO;
		cmd[1] = SI5518_PLL_CMD_I2C_HI;
		cmd[2] = SI5518_PLL_CMD_PLL_STATUS; // PLL_STATUS
		cmd[3] = pll_sel;

		memset(rxbuf, 0, SI5518_PLL_I2C_MAX_FRAME_SIZE);

		ret = i2c_si5518_write(i2c_cli, cmd, 4);
		if (ret) {
			dev_err(&i2c_cli->dev, "I2C write error %d\n", ret);
			ret = FREQ_CTRL_ERROR_FAIL;
			goto si_pll_lock_err;
		}

		ret = i2c_si5518_read(i2c_cli, rxbuf, SI5518_PLL_I2C_MAX_FRAME_SIZE);
		if (ret) {
			dev_err(&i2c_cli->dev, "I2C read error %d\n", ret);
			ret = FREQ_CTRL_ERROR_FAIL;
			goto si_pll_lock_err;
		}

		if (rxbuf[0] != SI5518_PLL_CMD_REPLY_CTS_STATUS) {
			dev_info(&i2c_cli->dev, "%s: PLL_STATUS reply: 0x%02x 0x%02x 0x%02x - FAIL\n",
					 __func__, rxbuf[0], rxbuf[1], rxbuf[2]);
			dev_err(&i2c_cli->dev, "%s: ERROR - PLL_STATUS CTS not set reply not ready\n", __func__);
		            ret = FREQ_CTRL_ERROR_FAIL;
			goto si_pll_lock_err;
		}

		if(!loop)
			continue; //PLL_STATUS : first loop clears sticky bits

		if (SI5518_DPLL_IS_LOCKED(rxbuf[1]) || SI5518_DPLL_IS_LOCKED(rxbuf[2])) {
			dev_err_ratelimited(&i2c_cli->dev, "%s: PLL_STATUS(%u) reply: %02x %02x %02x - FAIL\n",
				__func__, pll_sel, rxbuf[0], rxbuf[1], rxbuf[2]);
			dev_err_ratelimited(&i2c_cli->dev, "%s: ERROR - PLL_STATUS indicates out of lock\n", __func__);
			ret = FREQ_CTRL_ERROR_FAIL;
			goto si_pll_lock_err;
		} else {
			dev_info(&i2c_cli->dev, "%s: PLL_STATUS(%u) reply: %02x %02x %02x - OK\n",
					__func__, pll_sel, rxbuf[0], rxbuf[1], rxbuf[2]);
		}
	}

	ret = si5518_i2c_dco_centering(i2c_cli); //centering dco by setting step value to zero
        if (ret) {
		ret = FREQ_CTRL_ERROR_FAIL;
                goto si_pll_lock_ret;
	}

	priv->pll_lock_check_ctr = 0;
	ret = FREQ_CTRL_ERROR_SUCCESS;
	goto si_pll_lock_ret;

si_pll_lock_err:
	priv->pll_lock_check_ctr++;
	if (priv->pll_lock_check_ctr <= SI5518_MAX_PLL_LOCK_CHECK_COUNTER)
		schedule_delayed_work(&priv->pll_lock_dwork,
				      msecs_to_jiffies(SI5518_LOCK_CHECK_INTERVAL_IN_MS));
	ret = FREQ_CTRL_ERROR_FAIL;
si_pll_lock_ret:
        kfree(rxbuf);
}

static int i2c_si5518_pll_status(struct i2c_client *i2c_cli, u8 pll, u8 rxbuf[], u8 rxlen)
{
	int ret;
	u8 cmd[4] = {SI5518_PLL_CMD_I2C_LO,
					SI5518_PLL_CMD_I2C_HI,
					SI5518_PLL_CMD_PLL_STATUS,
					SI5518_DSPLLA};

	cmd[3] = pll;

	ret = i2c_si5518_write(i2c_cli, cmd, 4);
	if (ret) {
		dev_err(&i2c_cli->dev, "%s: write failed for pll(%x) status with error %d\n",
								__func__, pll, ret);
		return -EIO;
	}

	ret = i2c_si5518_read(i2c_cli, rxbuf, 10);
	if (ret) {
		dev_err(&i2c_cli->dev, "%s: read failed for pll(%x) status with error %d\n",
								__func__, pll, ret);
		return -EIO;
	}

	return 0;
}

static int i2c_si5518_set_holdover(struct i2c_client *i2c_cli, u8 pll, u8 state)
{
	int ret, count = 10;
	u8 cmd[5] = {SI5518_PLL_CMD_I2C_LO,
					SI5518_PLL_CMD_I2C_HI,
					SI5518_PLL_FORCE_HOLDOVER,
					SI5518_DSPLLA,
					SI5518_NO_HOLDOVER};
	u8 rxbuf[SI5518_PLL_I2C_MAX_FRAME_SIZE] = {0};

	cmd[3] = pll;
	cmd[4] = state;

	dev_info(&i2c_cli->dev, "%s: pll: %hhx, state: %hhx\n", __func__, pll, state);
	ret = i2c_si5518_write(i2c_cli, cmd, 5);
	if (ret) {
		dev_err(&i2c_cli->dev, "%s: write failed for exit holdover for pll(%x) with error %d\n",
								__func__, pll, ret);
		return -EIO;
	}

	ret = i2c_si5518_read(i2c_cli, rxbuf, 1);
	if (ret) {
		dev_err(&i2c_cli->dev, "%s: read failed for exit holdover for pll(%x) with error %d\n",
								__func__, pll, ret);
		return -EIO;
	}

	do {
		memset(rxbuf, 0, SI5518_PLL_I2C_MAX_FRAME_SIZE);

		ret = i2c_si5518_pll_status(i2c_cli, pll, rxbuf, 10);
		if (ret)
			return ret;

		/* if state is exit holdover */
		if ((state == 0x0) && (!(rxbuf[6] & SI5518_PLL_STATUS_FORCE_HOLDOVER)))
			return 0;

		/* if state is force holdover */
		if ((state == 0x1) && (rxbuf[6] & SI5518_PLL_STATUS_FORCE_HOLDOVER))
			return 0;

	} while (--count);

	return -EAGAIN;
}

static int i2c_si5518_select_input(struct i2c_client *i2c_cli, u8 pll, u8 in)
{
	int ret;
	u8 cmd[5] = {SI5518_PLL_CMD_I2C_LO,
					SI5518_PLL_CMD_I2C_HI,
					SI5518_MANUAL_INPUT_CLOCK_SELECT,
					SI5518_DSPLLA,
					SI5518_INPUT_IN2B};
	u8 rxbuf[SI5518_PLL_I2C_MAX_FRAME_SIZE] = {0};

	cmd[3] = pll;
	cmd[4] = in;

	dev_info(&i2c_cli->dev, "%s: pll: %hhx, input: %hhx\n", __func__, pll, in);
	ret = i2c_si5518_write(i2c_cli, cmd, 5);
	if (ret) {
		dev_err(&i2c_cli->dev, "%s: write failed for input(%x) select for pll(%x) with error %d\n",
								__func__, in, pll, ret);
		return -EIO;
	}

	ret = i2c_si5518_read(i2c_cli, rxbuf, 1);
	if (ret) {
		dev_err(&i2c_cli->dev, "%s: write failed for input(%x) select for pll(%x) with error %d\n",
								__func__, in, pll, ret);
		return -EIO;
	}

	return 0;
}

static ssize_t input_sel_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "echo \"pll in\" > input_sel\n" \
							"in0:0x0, in1:0x2, in2:0x4, in2b:0x5, in3:0x6, in3b:0x7\n" \
							"rfpll:0x1, dsplla:0x2, dspllb:0x4, ppspll:0x80\n");
}

static ssize_t input_sel_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t len)
{
	struct i2c_client *i2c_cli = to_i2c_client(dev);
	u8 pll, in;
	int ret;

	ret = sscanf(buf, "%hhx %hhx", &pll, &in);
	if (ret != 2)
		return -EINVAL;

	ret = i2c_si5518_select_input(i2c_cli, pll, in);
	if (ret)
		return ret;

	return len;
}

static ssize_t holdover_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "echo \"pll holdover_state\" > input_sel\n" \
					"force_holdover:0x1, exit_holdover:0x0\n" \
					"rfpll:0x1, dsplla:0x2, dspllb:0x4, ppspll:0x80\n");
}

static ssize_t holdover_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t len)
{
	struct i2c_client *i2c_cli = to_i2c_client(dev);
	u8 pll, state;
	int ret;

	ret = sscanf(buf, "%hhx %hhx", &pll, &state);
	if (ret != 2)
		return -EINVAL;

	ret = i2c_si5518_set_holdover(i2c_cli, pll, state);
	if (ret)
		return ret;

	return len;
}

static ssize_t pll_wait_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "echo \"pll <holdover>/<lock>\" > pll_wait\n" \
					"holdover:0x0, lock:0x1\n" \
					"rfpll:0x1, dsplla:0x2, dspllb:0x4, ppspll:0x80\n");
}
static ssize_t pll_wait_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t len)
{
	struct i2c_client *i2c_cli = to_i2c_client(dev);
	u8 rxbuf[SI5518_PLL_I2C_MAX_FRAME_SIZE] = {0};
	u8 pll, config;
	int ret;

	ret = sscanf(buf, "%hhx %hhx", &pll, &config);
	if (ret != 2)
		return -EINVAL;

	ret = i2c_si5518_pll_status(i2c_cli, pll, rxbuf, 10);
	if (ret)
		return ret;

	if ((config == 0x0) && (!(rxbuf[6] & SI5518_PLL_STATUS_HOLDOVER)))
		return -EIO;

	if ((config == 0x1) &&
			((rxbuf[1] & SI5518_PLL_STATUS_INITIAL_LOCK) ||
			(rxbuf[1] & SI5518_PLL_STATUS_LOL) ||
			(rxbuf[1] & SI5518_PLL_OUT_OF_PHASE) ||
			(rxbuf[1] & SI5518_PLL_OUT_OF_FREQ)))
		return -EIO;

	return len;
}

static int i2c_si5518_input_status(struct i2c_client *i2c_cli, u8 in, u8 rxbuf[], u8 rxlen)
{
	int ret;
	u8 cmd[5] = {SI5518_PLL_CMD_I2C_LO,
					SI5518_PLL_CMD_I2C_HI,
					SI5518_CMD_INPUT_STATUS,
					SI5518_INPUT_IN0};

	cmd[3] = in;


	ret = i2c_si5518_write(i2c_cli, cmd, 4);
	if (ret) {
		dev_err(&i2c_cli->dev, "%s: write failed for input(%x) status with error %d\n",
								__func__, in, ret);
		return -EIO;
	}

	ret = i2c_si5518_read(i2c_cli, rxbuf, rxlen);
	if (ret) {
		dev_err(&i2c_cli->dev, "%s: read failed for input(%x) status with error %d\n",
								__func__, in, ret);
		return -EIO;
	}

	return 0;
}

static ssize_t input_status_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "echo \"in\" > input_status\n" \
							"in0:0x0, in1:0x2, in2:0x4, in2b:0x5, in3:0x6, in3b:0x7\n" \
							"Returns success if the input is valid else error\n");
}

static ssize_t input_status_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t len)
{
	struct i2c_client *i2c_cli = to_i2c_client(dev);
	u8 rxbuf[SI5518_PLL_I2C_MAX_FRAME_SIZE] = {0};
	u8 in;
	int ret, count = 5;

	ret = sscanf(buf, "%hhx", &in);
	if (ret != 1)
		return -EINVAL;

	dev_info(&i2c_cli->dev, "%s: input: %hhx\n", __func__, in);
	do {
		ret = i2c_si5518_input_status(i2c_cli, in, rxbuf, 5);
		if (ret)
			return ret;

		if (rxbuf[1] == 0x0)
			return len;

	} while(--count);

	return -EAGAIN;
}

static ssize_t plla_status_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *i2c_cli = to_i2c_client(dev);
	u8 rxbuf[SI5518_PLL_I2C_MAX_FRAME_SIZE] = {0};
	int ret, count;

	ret = i2c_si5518_pll_status(i2c_cli, SI5518_DSPLLA , rxbuf, 10);
	if (ret)
		return ret;

	count = sprintf(buf, "DSPLLA: status=%hhx\n", rxbuf[0]);
	count += sprintf(buf + count, "loss_of_lock=%hhx\n", rxbuf[1]);
	count += sprintf(buf + count, "pll_status=%hhx\n", rxbuf[2]);
	count += sprintf(buf + count, "slip_count=%hhx\n", rxbuf[3]);
	count += sprintf(buf + count, "slip_count_net=%hhx\n", rxbuf[4]);
	count += sprintf(buf + count, "holdover_valid=%hhx\n", rxbuf[5]);
	count += sprintf(buf + count, "pll_holdover=%hhx\n", rxbuf[6]);
	count += sprintf(buf + count, "short_term_holdover=%hhx\n", rxbuf[7]);
	count += sprintf(buf + count, "phase_pullin=%hhx\n", rxbuf[8]);
	count += sprintf(buf + count, "loop_filter_status=%hhx\n", rxbuf[9]);

	return count;
}

static DEVICE_ATTR_RW(input_sel);
static DEVICE_ATTR_RW(holdover);
static DEVICE_ATTR_RW(pll_wait);
static DEVICE_ATTR_RW(input_status);
static DEVICE_ATTR_RO(plla_status);

static struct attribute *si5518_sysfs_attrs[] = {
	&dev_attr_input_sel.attr,
	&dev_attr_holdover.attr,
	&dev_attr_pll_wait.attr,
	&dev_attr_input_status.attr,
	&dev_attr_plla_status.attr,
	NULL
};

static const struct attribute_group si5518_attr_group = {
	.attrs = si5518_sysfs_attrs,
};


void i2c_dev_si5518_init(struct intel_freq_control_private *priv)
{
	struct i2c_client *i2c_cli = priv->fc_acc_type.i2c_cli;

	if (!i2c_cli) {
		dev_warn(&i2c_cli->dev, "%s - Failed to initialized freq stering\n", __func__);
		return;
	}

	/*
	 * initialized the stering. Set saved scaled_ppm a different value
	 * to make the initial zero scaled_ppm programmed successfully.
	 */
	priv->scaled_ppm_programmed = (10 << 16);
	priv->queued_work.scaled_ppm = 0;
	priv->freqctrl_ops.freqctrl(&priv->queued_work);
	flush_workqueue(priv->queued_work.workqueue);
	dev_info(&i2c_cli->dev, "%s - freq stering initialized\n", __func__);
}

int i2c_dev_check_si5518_clock(struct intel_freq_control_private *priv)
{
	struct i2c_client *i2c_cli = NULL;
	int ret = FREQ_CTRL_ERROR_SUCCESS;
	int count = 0;
	u8 *rxbuf;
	u8 cmd[8] = {SI5518_PLL_CMD_I2C_LO,
		     SI5518_PLL_CMD_I2C_HI,
		     SI5518_PLL_CMD_DEVICE_INFO};

	i2c_cli = priv->fc_acc_type.i2c_cli;
	if (!i2c_cli)
		return FREQ_CTRL_ERROR_FAIL;

	/* Allocate DMA-safe buffer for transfers */
	rxbuf = kmalloc(SI5518_PLL_I2C_MAX_FRAME_SIZE, GFP_KERNEL);
	if (!rxbuf)
		return FREQ_CTRL_ERROR_FAIL;

	memset(rxbuf, 0, SI5518_PLL_I2C_MAX_FRAME_SIZE);

#if SI5518_I2C_WA
read_again:
#endif

#if 0 /* TODO: Bug with repeated start condition after write with si5518 */
	ret = i2c_si5518_write_with_read_response(i2c_cli, cmd, 3, rxbuf, SI5518_PLL_I2C_MAX_FRAME_SIZE);
#endif
	ret = i2c_si5518_write(i2c_cli, cmd, 3);
	if (ret) {
		dev_err(&i2c_cli->dev, "%s: I2C write error %d\n", __func__,ret);
                ret = FREQ_CTRL_ERROR_FAIL;
		goto si_dev_chk_err;
	}

	ret = i2c_si5518_read(i2c_cli, rxbuf, 3);
	if (ret) {
		dev_err(&i2c_cli->dev, "%s: I2C write error %d\n", __func__,ret);
                ret = FREQ_CTRL_ERROR_FAIL;
		goto si_dev_chk_err;
	}

	if (rxbuf[0] != SI5518_PLL_CMD_REPLY_CTS_STATUS) {
		dev_info(&i2c_cli->dev, "%s: DEVICE_INFO reply: 0x%02x 0x%02x 0x%02x - FAIL\n", __func__,
			rxbuf[0], rxbuf[1], rxbuf[2]);
		dev_err(&i2c_cli->dev, "%s: ERROR - DEVICE_INFO CTS not set reply not ready\n", __func__);
                ret = FREQ_CTRL_ERROR_FAIL;
		goto si_dev_chk_err;
	}

		dev_info(&i2c_cli->dev, "%s: DEVICE_INFO reply: 0x%02x 0x%02x 0x%02x \n", __func__,
			rxbuf[0], rxbuf[1], rxbuf[2]);

#if SI5518_I2C_WA
	if ((((rxbuf[2] << 8) | rxbuf[1]) != SI5518_PLL_DEVICE_ID) && (count < 3)) {
		count++;
		ssleep(10);
		goto  read_again;
	}
#endif

	if (((rxbuf[2] << 8) | rxbuf[1]) == SI5518_PLL_DEVICE_ID) {
		dev_info(&i2c_cli->dev, "%s: DEVICE_INFO reply: 0x%02x 0x%02x 0x%02x - OK\n", __func__,
			rxbuf[0], rxbuf[1], rxbuf[2]);
		priv->pll_lock_check_ctr = 1;

		INIT_DELAYED_WORK(&priv->pll_lock_dwork, rfpll_lock_handler);
		schedule_delayed_work(&priv->pll_lock_dwork,
				      msecs_to_jiffies(1));
	}

	sysfs_update_group(&i2c_cli->dev.kobj, &si5518_attr_group);

	ret = FREQ_CTRL_ERROR_SUCCESS;

si_dev_chk_err:
    kfree(rxbuf);
	return ret;
}
EXPORT_SYMBOL(i2c_dev_check_si5518_clock);

MODULE_LICENSE("GPL");
