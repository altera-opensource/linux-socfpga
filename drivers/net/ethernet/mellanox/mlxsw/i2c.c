/*
 * drivers/net/ethernet/mellanox/mlxsw/i2c.c
 * Copyright (c) 2016 Mellanox Technologies. All rights reserved.
 * Copyright (c) 2016 Vadim Pasternak <vadimp@mellanox.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the names of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/slab.h>

#include "cmd.h"
#include "core.h"
#include "i2c.h"

static const char mlxsw_i2c_driver_name[] = "mlxsw_i2c";

#define MLXSW_I2C_CIR2_BASE		0x72000
#define MLXSW_I2C_CIR_STATUS_OFF	0x18
#define MLXSW_I2C_CIR2_OFF_STATUS	(MLXSW_I2C_CIR2_BASE + \
					 MLXSW_I2C_CIR_STATUS_OFF)
#define MLXSW_I2C_OPMOD_SHIFT		12
#define MLXSW_I2C_GO_BIT_SHIFT		23
#define MLXSW_I2C_CIR_CTRL_STATUS_SHIFT	24
#define MLXSW_I2C_GO_BIT		BIT(MLXSW_I2C_GO_BIT_SHIFT)
#define MLXSW_I2C_GO_OPMODE		BIT(MLXSW_I2C_OPMOD_SHIFT)
#define MLXSW_I2C_SET_IMM_CMD		(MLXSW_I2C_GO_OPMODE | \
					 MLXSW_CMD_OPCODE_QUERY_FW)
#define MLXSW_I2C_PUSH_IMM_CMD		(MLXSW_I2C_GO_BIT | \
					 MLXSW_I2C_SET_IMM_CMD)
#define MLXSW_I2C_SET_CMD		(MLXSW_CMD_OPCODE_ACCESS_REG)
#define MLXSW_I2C_PUSH_CMD		(MLXSW_I2C_GO_BIT | MLXSW_I2C_SET_CMD)
#define MLXSW_I2C_TLV_HDR_SIZE		0x10
#define MLXSW_I2C_ADDR_WIDTH		4
#define MLXSW_I2C_PUSH_CMD_SIZE		(MLXSW_I2C_ADDR_WIDTH + 4)
#define MLXSW_I2C_READ_SEMA_SIZE	4
#define MLXSW_I2C_PREP_SIZE		(MLXSW_I2C_ADDR_WIDTH + 28)
#define MLXSW_I2C_MBOX_SIZE		20
#define MLXSW_I2C_MBOX_OUT_PARAM_OFF	12
#define MLXSW_I2C_MAX_BUFF_SIZE		32
#define MLXSW_I2C_MBOX_OFFSET_BITS	20
#define MLXSW_I2C_MBOX_SIZE_BITS	12
#define MLXSW_I2C_ADDR_BUF_SIZE		4
#define MLXSW_I2C_BLK_MAX		32
#define MLXSW_I2C_RETRY			5
#define MLXSW_I2C_TIMEOUT_MSECS		5000

/**
 * struct mlxsw_i2c - device private data:
 * @cmd.mb_size_in: input mailbox size;
 * @cmd.mb_off_in: input mailbox offset in register space;
 * @cmd.mb_size_out: output mailbox size;
 * @cmd.mb_off_out: output mailbox offset in register space;
 * @cmd.lock: command execution lock;
 * @dev: I2C device;
 * @core: switch core pointer;
 * @bus_info: bus info block;
 */
struct mlxsw_i2c {
	struct {
		u32 mb_size_in;
		u32 mb_off_in;
		u32 mb_size_out;
		u32 mb_off_out;
		struct mutex lock;
	} cmd;
	struct device *dev;
	struct mlxsw_core *core;
	struct mlxsw_bus_info bus_info;
};

#define MLXSW_I2C_READ_MSG(_client, _addr_buf, _buf, _len) {	\
	{ .addr = (_client)->addr,				\
	  .buf = (_addr_buf),					\
	  .len = MLXSW_I2C_ADDR_BUF_SIZE,			\
	  .flags = 0 },						\
	{ .addr = (_client)->addr,				\
	  .buf = (_buf),					\
	  .len = (_len),					\
	  .flags = I2C_M_RD } }

#define MLXSW_I2C_WRITE_MSG(_client, _buf, _len)		\
	{ .addr = (_client)->addr,				\
	  .buf = (u8 *)(_buf),					\
	  .len = (_len),					\
	  .flags = 0 }

/* Routine converts in and out mail boxes offset and size. */
static inline void
mlxsw_i2c_convert_mbox(struct mlxsw_i2c *mlxsw_i2c, u8 *buf)
{
	u32 tmp;

	/* Local in/out mailboxes: 20 bits for offset, 12 for size */
	tmp = be32_to_cpup((__be32 *) buf);
	mlxsw_i2c->cmd.mb_off_in = tmp &
				   GENMASK(MLXSW_I2C_MBOX_OFFSET_BITS - 1, 0);
	mlxsw_i2c->cmd.mb_size_in = (tmp & GENMASK(31,
					MLXSW_I2C_MBOX_OFFSET_BITS)) >>
					MLXSW_I2C_MBOX_OFFSET_BITS;

	tmp = be32_to_cpup((__be32 *) (buf + MLXSW_I2C_ADDR_WIDTH));
	mlxsw_i2c->cmd.mb_off_out = tmp &
				    GENMASK(MLXSW_I2C_MBOX_OFFSET_BITS - 1, 0);
	mlxsw_i2c->cmd.mb_size_out = (tmp & GENMASK(31,
					MLXSW_I2C_MBOX_OFFSET_BITS)) >>
					MLXSW_I2C_MBOX_OFFSET_BITS;
}

/* Routine obtains register size from mail box buffer. */
static inline int mlxsw_i2c_get_reg_size(u8 *in_mbox)
{
	u16  tmp = be16_to_cpup((__be16 *) (in_mbox + MLXSW_I2C_TLV_HDR_SIZE));

	return (tmp & 0x7ff) * 4 + MLXSW_I2C_TLV_HDR_SIZE;
}

/* Routine sets I2C device internal offset in the transaction buffer. */
static inline void mlxsw_i2c_set_slave_addr(u8 *buf, u32 off)
{
	__be32 *val = (__be32 *) buf;

	*val = htonl(off);
}

/* Routine waits until go bit is cleared. */
static int mlxsw_i2c_wait_go_bit(struct i2c_client *client,
				 struct mlxsw_i2c *mlxsw_i2c, u8 *p_status)
{
	u8 addr_buf[MLXSW_I2C_ADDR_BUF_SIZE];
	u8 buf[MLXSW_I2C_READ_SEMA_SIZE];
	int len = MLXSW_I2C_READ_SEMA_SIZE;
	struct i2c_msg read_sema[] =
		MLXSW_I2C_READ_MSG(client, addr_buf, buf, len);
	bool wait_done = false;
	unsigned long end;
	int i = 0, err;

	mlxsw_i2c_set_slave_addr(addr_buf, MLXSW_I2C_CIR2_OFF_STATUS);

	end = jiffies + msecs_to_jiffies(MLXSW_I2C_TIMEOUT_MSECS);
	do {
		u32 ctrl;

		err = i2c_transfer(client->adapter, read_sema,
				   ARRAY_SIZE(read_sema));

		ctrl = be32_to_cpu(*(__be32 *) buf);
		if (err == ARRAY_SIZE(read_sema)) {
			if (!(ctrl & MLXSW_I2C_GO_BIT)) {
				wait_done = true;
				*p_status = ctrl >>
					    MLXSW_I2C_CIR_CTRL_STATUS_SHIFT;
				break;
			}
		}
		cond_resched();
	} while ((time_before(jiffies, end)) || (i++ < MLXSW_I2C_RETRY));

	if (wait_done) {
		if (*p_status)
			err = -EIO;
	} else {
		return -ETIMEDOUT;
	}

	return err > 0 ? 0 : err;
}

/* Routine posts a command to ASIC though mail box. */
static int mlxsw_i2c_write_cmd(struct i2c_client *client,
			       struct mlxsw_i2c *mlxsw_i2c,
			       int immediate)
{
	__be32 push_cmd_buf[MLXSW_I2C_PUSH_CMD_SIZE / 4] = {
		0, cpu_to_be32(MLXSW_I2C_PUSH_IMM_CMD)
	};
	__be32 prep_cmd_buf[MLXSW_I2C_PREP_SIZE / 4] = {
		0, 0, 0, 0, 0, 0,
		cpu_to_be32(client->adapter->nr & 0xffff),
		cpu_to_be32(MLXSW_I2C_SET_IMM_CMD)
	};
	struct i2c_msg push_cmd =
		MLXSW_I2C_WRITE_MSG(client, push_cmd_buf,
				    MLXSW_I2C_PUSH_CMD_SIZE);
	struct i2c_msg prep_cmd =
		MLXSW_I2C_WRITE_MSG(client, prep_cmd_buf, MLXSW_I2C_PREP_SIZE);
	int err;

	if (!immediate) {
		push_cmd_buf[1] = cpu_to_be32(MLXSW_I2C_PUSH_CMD);
		prep_cmd_buf[7] = cpu_to_be32(MLXSW_I2C_SET_CMD);
	}
	mlxsw_i2c_set_slave_addr((u8 *)prep_cmd_buf,
				 MLXSW_I2C_CIR2_BASE);
	mlxsw_i2c_set_slave_addr((u8 *)push_cmd_buf,
				 MLXSW_I2C_CIR2_OFF_STATUS);

	/* Prepare Command Interface Register for transaction */
	err = i2c_transfer(client->adapter, &prep_cmd, 1);
	if (err < 0)
		return err;
	else if (err != 1)
		return -EIO;

	/* Write out Command Interface Register GO bit to push transaction */
	err = i2c_transfer(client->adapter, &push_cmd, 1);
	if (err < 0)
		return err;
	else if (err != 1)
		return -EIO;

	return 0;
}

/* Routine obtains mail box offsets from ASIC register space. */
static int mlxsw_i2c_get_mbox(struct i2c_client *client,
			      struct mlxsw_i2c *mlxsw_i2c)
{
	u8 addr_buf[MLXSW_I2C_ADDR_BUF_SIZE];
	u8 buf[MLXSW_I2C_MBOX_SIZE];
	struct i2c_msg mbox_cmd[] =
		MLXSW_I2C_READ_MSG(client, addr_buf, buf, MLXSW_I2C_MBOX_SIZE);
	int err;

	/* Read mail boxes offsets. */
	mlxsw_i2c_set_slave_addr(addr_buf, MLXSW_I2C_CIR2_BASE);
	err = i2c_transfer(client->adapter, mbox_cmd, 2);
	if (err != 2) {
		dev_err(&client->dev, "Could not obtain mail boxes\n");
		if (!err)
			return -EIO;
		else
			return err;
	}

	/* Convert mail boxes. */
	mlxsw_i2c_convert_mbox(mlxsw_i2c, &buf[MLXSW_I2C_MBOX_OUT_PARAM_OFF]);

	return err;
}

/* Routine sends I2C write transaction to ASIC device. */
static int
mlxsw_i2c_write(struct device *dev, size_t in_mbox_size, u8 *in_mbox, int num,
		u8 *p_status)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mlxsw_i2c *mlxsw_i2c = i2c_get_clientdata(client);
	unsigned long timeout = msecs_to_jiffies(MLXSW_I2C_TIMEOUT_MSECS);
	u8 tran_buf[MLXSW_I2C_MAX_BUFF_SIZE + MLXSW_I2C_ADDR_BUF_SIZE];
	int off = mlxsw_i2c->cmd.mb_off_in, chunk_size, i, j;
	unsigned long end;
	struct i2c_msg write_tran =
		MLXSW_I2C_WRITE_MSG(client, tran_buf, MLXSW_I2C_PUSH_CMD_SIZE);
	int err;

	for (i = 0; i < num; i++) {
		chunk_size = (in_mbox_size > MLXSW_I2C_BLK_MAX) ?
			     MLXSW_I2C_BLK_MAX : in_mbox_size;
		write_tran.len = MLXSW_I2C_ADDR_WIDTH + chunk_size;
		mlxsw_i2c_set_slave_addr(tran_buf, off);
		memcpy(&tran_buf[MLXSW_I2C_ADDR_BUF_SIZE], in_mbox +
		       MLXSW_I2C_BLK_MAX * i, chunk_size);

		j = 0;
		end = jiffies + timeout;
		do {
			err = i2c_transfer(client->adapter, &write_tran, 1);
			if (err == 1)
				break;

			cond_resched();
		} while ((time_before(jiffies, end)) ||
			 (j++ < MLXSW_I2C_RETRY));

		if (err != 1) {
			if (!err)
				err = -EIO;
			return err;
		}

		off += chunk_size;
		in_mbox_size -= chunk_size;
	}

	/* Prepare and write out Command Interface Register for transaction. */
	err = mlxsw_i2c_write_cmd(client, mlxsw_i2c, 0);
	if (err) {
		dev_err(&client->dev, "Could not start transaction");
		return -EIO;
	}

	/* Wait until go bit is cleared. */
	err = mlxsw_i2c_wait_go_bit(client, mlxsw_i2c, p_status);
	if (err) {
		dev_err(&client->dev, "HW semaphore is not released");
		return err;
	}

	/* Validate transaction completion status. */
	if (*p_status) {
		dev_err(&client->dev, "Bad transaction completion status %x\n",
			*p_status);
		return -EIO;
	}

	return 0;
}

/* Routine executes I2C command. */
static int
mlxsw_i2c_cmd(struct device *dev, size_t in_mbox_size, u8 *in_mbox,
	      size_t out_mbox_size, u8 *out_mbox, u8 *status)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mlxsw_i2c *mlxsw_i2c = i2c_get_clientdata(client);
	unsigned long timeout = msecs_to_jiffies(MLXSW_I2C_TIMEOUT_MSECS);
	u8 tran_buf[MLXSW_I2C_ADDR_BUF_SIZE];
	int num, chunk_size, reg_size, i, j;
	int off = mlxsw_i2c->cmd.mb_off_out;
	unsigned long end;
	struct i2c_msg read_tran[] =
		MLXSW_I2C_READ_MSG(client, tran_buf, NULL, 0);
	int err;

	WARN_ON(in_mbox_size % sizeof(u32) || out_mbox_size % sizeof(u32));

	reg_size = mlxsw_i2c_get_reg_size(in_mbox);
	num = reg_size / MLXSW_I2C_BLK_MAX;
	if (reg_size % MLXSW_I2C_BLK_MAX)
		num++;

	if (mutex_lock_interruptible(&mlxsw_i2c->cmd.lock) < 0) {
		dev_err(&client->dev, "Could not acquire lock");
		return -EINVAL;
	}

	err = mlxsw_i2c_write(dev, reg_size, in_mbox, num, status);
	if (err)
		goto cmd_fail;

	/* No out mailbox is case of write transaction. */
	if (!out_mbox) {
		mutex_unlock(&mlxsw_i2c->cmd.lock);
		return 0;
	}

	/* Send read transaction to get output mailbox content. */
	read_tran[1].buf = out_mbox;
	for (i = 0; i < num; i++) {
		chunk_size = (reg_size > MLXSW_I2C_BLK_MAX) ?
			     MLXSW_I2C_BLK_MAX : reg_size;
		read_tran[1].len = chunk_size;
		mlxsw_i2c_set_slave_addr(tran_buf, off);

		j = 0;
		end = jiffies + timeout;
		do {
			err = i2c_transfer(client->adapter, read_tran,
					   ARRAY_SIZE(read_tran));
			if (err == ARRAY_SIZE(read_tran))
				break;

			cond_resched();
		} while ((time_before(jiffies, end)) ||
			 (j++ < MLXSW_I2C_RETRY));

		if (err != ARRAY_SIZE(read_tran)) {
			if (!err)
				err = -EIO;

			goto cmd_fail;
		}

		off += chunk_size;
		reg_size -= chunk_size;
		read_tran[1].buf += chunk_size;
	}

	mutex_unlock(&mlxsw_i2c->cmd.lock);

	return 0;

cmd_fail:
	mutex_unlock(&mlxsw_i2c->cmd.lock);
	return err;
}

static int mlxsw_i2c_cmd_exec(void *bus_priv, u16 opcode, u8 opcode_mod,
			      u32 in_mod, bool out_mbox_direct,
			      char *in_mbox, size_t in_mbox_size,
			      char *out_mbox, size_t out_mbox_size,
			      u8 *status)
{
	struct mlxsw_i2c *mlxsw_i2c = bus_priv;

	return mlxsw_i2c_cmd(mlxsw_i2c->dev, in_mbox_size, in_mbox,
			     out_mbox_size, out_mbox, status);
}

static bool mlxsw_i2c_skb_transmit_busy(void *bus_priv,
					const struct mlxsw_tx_info *tx_info)
{
	return false;
}

static int mlxsw_i2c_skb_transmit(void *bus_priv, struct sk_buff *skb,
				  const struct mlxsw_tx_info *tx_info)
{
	return 0;
}

static int
mlxsw_i2c_init(void *bus_priv, struct mlxsw_core *mlxsw_core,
	       const struct mlxsw_config_profile *profile,
	       struct mlxsw_res *resources)
{
	struct mlxsw_i2c *mlxsw_i2c = bus_priv;

	mlxsw_i2c->core = mlxsw_core;

	return 0;
}

static void mlxsw_i2c_fini(void *bus_priv)
{
	struct mlxsw_i2c *mlxsw_i2c = bus_priv;

	mlxsw_i2c->core = NULL;
}

static const struct mlxsw_bus mlxsw_i2c_bus = {
	.kind			= "i2c",
	.init			= mlxsw_i2c_init,
	.fini			= mlxsw_i2c_fini,
	.skb_transmit_busy	= mlxsw_i2c_skb_transmit_busy,
	.skb_transmit		= mlxsw_i2c_skb_transmit,
	.cmd_exec		= mlxsw_i2c_cmd_exec,
};

static int mlxsw_i2c_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct mlxsw_i2c *mlxsw_i2c;
	u8 status;
	int err;

	mlxsw_i2c = devm_kzalloc(&client->dev, sizeof(*mlxsw_i2c), GFP_KERNEL);
	if (!mlxsw_i2c)
		return -ENOMEM;

	i2c_set_clientdata(client, mlxsw_i2c);
	mutex_init(&mlxsw_i2c->cmd.lock);

	/* In order to use mailboxes through the i2c, special area is reserved
	 * on the i2c address space that can be used for input and output
	 * mailboxes. Such mailboxes are called local mailboxes. When using a
	 * local mailbox, software should specify 0 as the Input/Output
	 * parameters. The location of the Local Mailbox addresses on the i2c
	 * space can be retrieved through the QUERY_FW command.
	 * For this purpose QUERY_FW is to be issued with opcode modifier equal
	 * 0x01. For such command the output parameter is an immediate value.
	 * Here QUERY_FW command is invoked for ASIC probing and for getting
	 * local mailboxes addresses from immedate output parameters.
	 */

	/* Prepare and write out Command Interface Register for transaction */
	err = mlxsw_i2c_write_cmd(client, mlxsw_i2c, 1);
	if (err) {
		dev_err(&client->dev, "Could not start transaction");
		goto errout;
	}

	/* Wait until go bit is cleared. */
	err = mlxsw_i2c_wait_go_bit(client, mlxsw_i2c, &status);
	if (err) {
		dev_err(&client->dev, "HW semaphore is not released");
		goto errout;
	}

	/* Validate transaction completion status. */
	if (status) {
		dev_err(&client->dev, "Bad transaction completion status %x\n",
			status);
		err = -EIO;
		goto errout;
	}

	/* Get mailbox offsets. */
	err = mlxsw_i2c_get_mbox(client, mlxsw_i2c);
	if (err < 0) {
		dev_err(&client->dev, "Fail to get mailboxes\n");
		goto errout;
	}

	dev_info(&client->dev, "%s mb size=%x off=0x%08x out mb size=%x off=0x%08x\n",
		 id->name, mlxsw_i2c->cmd.mb_size_in,
		 mlxsw_i2c->cmd.mb_off_in, mlxsw_i2c->cmd.mb_size_out,
		 mlxsw_i2c->cmd.mb_off_out);

	/* Register device bus. */
	mlxsw_i2c->bus_info.device_kind = id->name;
	mlxsw_i2c->bus_info.device_name = client->name;
	mlxsw_i2c->bus_info.dev = &client->dev;
	mlxsw_i2c->dev = &client->dev;

	err = mlxsw_core_bus_device_register(&mlxsw_i2c->bus_info,
					     &mlxsw_i2c_bus, mlxsw_i2c, false,
					     NULL);
	if (err) {
		dev_err(&client->dev, "Fail to register core bus\n");
		return err;
	}

	return 0;

errout:
	i2c_set_clientdata(client, NULL);

	return err;
}

static int mlxsw_i2c_remove(struct i2c_client *client)
{
	struct mlxsw_i2c *mlxsw_i2c = i2c_get_clientdata(client);

	mlxsw_core_bus_device_unregister(mlxsw_i2c->core, false);
	mutex_destroy(&mlxsw_i2c->cmd.lock);

	return 0;
}

int mlxsw_i2c_driver_register(struct i2c_driver *i2c_driver)
{
	i2c_driver->probe = mlxsw_i2c_probe;
	i2c_driver->remove = mlxsw_i2c_remove;
	return i2c_add_driver(i2c_driver);
}
EXPORT_SYMBOL(mlxsw_i2c_driver_register);

void mlxsw_i2c_driver_unregister(struct i2c_driver *i2c_driver)
{
	i2c_del_driver(i2c_driver);
}
EXPORT_SYMBOL(mlxsw_i2c_driver_unregister);

MODULE_AUTHOR("Vadim Pasternak <vadimp@mellanox.com>");
MODULE_DESCRIPTION("Mellanox switch I2C interface driver");
MODULE_LICENSE("Dual BSD/GPL");
