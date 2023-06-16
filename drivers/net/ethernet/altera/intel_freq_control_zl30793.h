// SPDX-License-Identifier: GPL-2.0
/* Intel FPGA Clock Cleaner Frequency Adjustment Driver
 * Copyright (C) 2023 Intel Corporation. All rights reserved.
 *
 * Author(s):
 *	Lubana Badakar <lubana.badakar@intel.com>
 */

#ifndef HAVE_INTEL_FREQUENCY_CONTROL_ZL30793_H
#define HAVE_INTEL_FREQUENCY_CONTROL_ZL30793_H

#define PLL_SPI_MAX_FRAME_SIZE                  64

#define FREQ_CTRL_ERROR_SUCCESS 0
#define FREQ_CTRL_ERROR_FAIL    1

int zl30793_sysfs_configure(struct spi_device *spi);
u8 zl30793_spi_read(struct spi_device *spi,  u16 *dma_safe_buf,
		    u16 addr, u8 *val);
u8 zl30793_spi_write(struct spi_device *spi, u16 *dma_safe_buf,
		     u16 addr, u8 *val);

#endif //HAVE_INTEL_FREQUENCY_CONTROL_ZL30793_H