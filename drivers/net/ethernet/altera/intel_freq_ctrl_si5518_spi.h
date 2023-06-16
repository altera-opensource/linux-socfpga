// SPDX-License-Identifier: GPL
 /* Copyright (C) 2023 Intel Corporation. All rights reserved.
 *
 * Author(s):
 */

#ifndef HAVE_INTEL_FREQ_CONTROL_SI5518_SPI_H
#define HAVE_INTEL_FREQ_CONTROL_SI5518_SPI_H

int spi_dev_check_si5518_clock(struct spi_device *spi,
		struct intel_freq_control_private *priv);
void intel_freq_control_si5518(struct work_struct *);
#endif
