
#ifndef __INTEL_FPGA_ETH_QSFP_H__
#define  __INTEL_FPGA_ETH_QSFP_H__

#include "intel_fpga_eth_hssi_itf.h"

#define QSFP_POLL_TIMEOUT 1

struct qsfp_reg_space {

	u8   ns_area0[27];
	union status_reg {
		struct {
			u32 mod_presence:1;
			u32 qsfp_intr:1;
			u32 reserved:30;
			u32 reserved2;
		}qsfp_stat_bits;
		u64 qsfp_stat;
	}status_reg;
};

#endif		
