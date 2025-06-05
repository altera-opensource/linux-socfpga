/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2023 Altera Corporation. All rights reserved.
 *
 * Author(s):
 */

#ifndef HAVE_INTEL_FREQ_CONTROL_COMMON_SPI_H
#define HAVE_INTEL_FREQ_CONTROL_COMMON_SPI_H

#define INTEL_FPGA_SPI_ERROR   0
#define INTEL_FPGA_SPI_SUCCESS 1

struct clock_cleaner;
int determine_spi_client(struct clock_cleaner *cc);
u8 spi_msg_transfer(struct spi_device *spi, void *tx_buf, void *rx_buf);

#endif
