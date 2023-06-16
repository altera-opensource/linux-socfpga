// SPDX-License-Identifier: GPL
 /* Copyright (C) 2023 Intel Corporation. All rights reserved.
 *
 * Author(s):
 */

#ifndef HAVE_INTEL_FREQ_CONTROL_COMMON_SPI_H
#define HAVE_INTEL_FREQ_CONTROL_COMMON_SPI_H

#define INTEL_FPGA_SPI_ERROR   0
#define INTEL_FPGA_SPI_SUCCESS 1

#define FREQ_CTRL_ERROR_SUCCESS 0
#define FREQ_CTRL_ERROR_FAIL    1

struct clock_cleaner;
int determine_spi_client(struct clock_cleaner *);
u8 spi_msg_transfer(struct spi_device *spi, void* tx_buf, void* rx_buf);

#endif