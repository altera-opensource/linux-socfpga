// SPDX-License-Identifier: GPL-2.0
/* Intel FPGA SPI access code
 * Copyright (C) 2023 Intel Corporation. All rights reserved
 *
 * Contributors:
 *	Preetam Narayan
 *
 */

#include "intel_freq_control.h"
#include "intel_freq_ctrl_common_spi.h"

static int spi_dev_check(struct device *dev, void *data)
{
	int ret = INTEL_FPGA_SPI_SUCCESS;
	struct spi_device *spi = to_spi_device(dev);
	struct clock_cleaner *clockcleaner_info =
				(struct clock_cleaner *)data;

	struct intel_freq_control_private *priv =
		container_of(clockcleaner_info,
			     struct intel_freq_control_private,
			     clockcleaner_info);

	if (!clockcleaner_info || !spi) {
		dev_err(&spi->dev, "NULL check (%s) failed\n",
			dev_name(dev));
		ret = INTEL_FPGA_SPI_ERROR;
		goto spi_client_ret;
	}

	if ((clockcleaner_info->bus_num != spi->controller->bus_num) ||
	    (clockcleaner_info->chip_select != spi->chip_select)) {
		ret = INTEL_FPGA_SPI_ERROR;
		goto spi_client_ret;
	}

	dev_info(&spi->dev, "spi->modalias:%s\n", spi->modalias);

	priv->fc_acc_type.spi_dev = spi;

spi_client_ret:
	return ret;
}

/**
 * @brief info updated in the dts file is used to select the specific
 * spi device used for the frequency cleaning
 */
int determine_spi_client(struct clock_cleaner *clockcleaner_info)
{
	int ret = FREQ_CTRL_ERROR_FAIL;

	struct intel_freq_control_private *priv =
		container_of(clockcleaner_info,
			     struct intel_freq_control_private,
			     clockcleaner_info);

	if (priv->fc_acc_type.spi_dev) {
		ret = FREQ_CTRL_ERROR_SUCCESS;
		goto err;
	}

	/* ret = -EINVAL : if the bus is not registered yet
	 * ret > 0       : we have found our device
	 * ret = 0       : continue checking devices we not found match device
	 */
	ret = bus_for_each_dev(&spi_bus_type, NULL,
			       clockcleaner_info,
			       spi_dev_check);

	/* case can happen that the spi bus is not registered yet */
	if (ret > 0) {
		ret = FREQ_CTRL_ERROR_SUCCESS;
		goto err;
	}

err:
	return ret;
}

/**
 * @brief this function handles the send and receive to the spi device
 * @param parameter1 is the spi frequency controller device
 * @param parameter2 is the tx buffer pointer
 * @param parameter3 is the rx buffer pointeri
 * @return success state of the transfer to spi device
 */
u8 spi_msg_transfer(struct spi_device *spi, void *tx_buf, void *rx_buf)
{
	u8 ret;
	struct spi_transfer x;
	struct spi_message spi_message;

	spi_message_init(&spi_message);

	memset(&x, 0, sizeof(x));
	x.len = 2;
	x.tx_buf = tx_buf;
	x.rx_buf = rx_buf;

	spi_message_add_tail(&x, &spi_message);

	//perform I/O
	ret = spi_sync(spi, &spi_message);

	return ret;
}

