// SPDX-License-Identifier: GPL
/* Intel FPGA I2C client code
 * Copyright (C) 2023 Intel Corporation. All rights reserved
 *
 * Contributors:
 *
 */

#include "intel_freq_control.h"
#include "intel_freq_ctrl_common_i2c.h"

int determine_i2c_client(struct clock_cleaner *clockcleaner_info)
{
	struct i2c_adapter *i2c_adap;
	struct i2c_board_info i2c_info;
	int adapter;
	int addr;
	const char *type;
	struct i2c_client *i2c_cli;
        struct intel_freq_control_private *priv =
                container_of (clockcleaner_info,
                        struct intel_freq_control_private,
                        clockcleaner_info);

	i2c_cli = priv->fc_acc_type.i2c_cli;

	if (!clockcleaner_info)
		return 0;

	adapter = clockcleaner_info->bus_num;
	addr = clockcleaner_info->bus_address;
	type = clockcleaner_info->clock_name;

	if (i2c_cli)
		return 1;

	i2c_adap = i2c_get_adapter(adapter);
	if (!i2c_adap) {
		pr_err("[ %s ] i2c_get_adapter is NULL", __func__);
		return 0;
	}
	
	memset(&i2c_info, 0, sizeof(struct i2c_board_info));
	strlcpy(i2c_info.type, type, I2C_NAME_SIZE);
	i2c_info.addr = addr;
	i2c_cli = i2c_new_client_device(i2c_adap, &i2c_info);
	

	
	if (IS_ERR(i2c_cli)) {
		pr_err("can't create i2c device %s\n", i2c_info.type);
		return 0;
	} else {

		pr_info("created i2c device %p\n", (uint32_t *)i2c_cli);
		priv->fc_acc_type.i2c_cli =i2c_cli;
		return 1;
	}
}
