/* SPDX-License-Identifier: GPL-2.0 */
/*
 * DT binding constants for Intel FPGA HSSI subsystem.
 * Copyright (C) 2025, Intel Corporation
 */

#ifndef __INTEL_FPGA_HSSI_H__
#define __INTEL_FPGA_HSSI_H__

/* FEC mode encoding for the "dr-profiles" DTS property (fec cell).
 * Use these macros instead of raw integers for readability.
 */
#define FTILE_FEC_NONE		0	/* No FEC */
#define FTILE_FEC_BASER		1	/* Base-R / KR FEC */
#define FTILE_FEC_RS		2	/* Reed-Solomon FEC */

#endif /* __INTEL_FPGA_HSSI_H__ */
