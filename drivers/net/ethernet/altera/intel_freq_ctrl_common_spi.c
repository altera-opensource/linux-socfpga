// SPDX-License-Identifier: GPL
/* Intel FPGA SPI access code
 * Copyright (C) 2023 Intel Corporation. All rights reserved
 *
 * Contributors:
 * 	Preetam Narayan
 *
 */

#include "intel_freq_control.h"
#include "intel_freq_ctrl_common_spi.h"

static int spi_dev_check(struct device *dev, void *data)
{
	struct spi_device *spi = to_spi_device(dev);
	struct clock_cleaner *clockcleaner_info =
				(struct clock_cleaner *)data;

	struct intel_freq_control_private *priv =
		container_of (clockcleaner_info,
			struct intel_freq_control_private,
			clockcleaner_info);

	if (!clockcleaner_info || !spi) {

		dev_err(&spi->dev, "NULL check (%s) failed\n",
			dev_name(dev));

		return INTEL_FPGA_SPI_ERROR;
	}

	if ((clockcleaner_info->bus_num != spi->master->bus_num) ||
	    (clockcleaner_info->chip_select != spi->chip_select))
		return INTEL_FPGA_SPI_ERROR;

	dev_info(&spi->dev, "spi->modalias:%s\n", spi->modalias);

	if ( FREQ_CTRL_ERROR_SUCCESS == 
		priv->intf_ops->clock_check(spi, priv) )
		return INTEL_FPGA_SPI_SUCCESS;

	return INTEL_FPGA_SPI_ERROR;
}

/**
 * @brief info updated in the dts file is used to select the specific 
 * spi device used for the frequency cleaning 
 */
int determine_spi_client(struct clock_cleaner *clockcleaner_info)
{
	struct intel_freq_control_private *priv =
		container_of (clockcleaner_info,
			struct intel_freq_control_private,
			clockcleaner_info);

	if (priv->fc_acc_type.spi_dev)
		return 1;

	return bus_for_each_dev(&spi_bus_type,
			NULL,
			clockcleaner_info,
			spi_dev_check);
}

/**
 * @brief this function handles the send and receive to the spi device
 * @param parameter1 is the spi frequency controller device 
 * @param parameter2 is the tx buffer pointer 
 * @param parameter3 is the rx buffer pointeri
 * @return success state of the transfer to spi device
 */
u8 spi_msg_transfer(struct spi_device *spi, void* tx_buf, void* rx_buf) {

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