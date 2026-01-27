/* SPDX-License-Identifier: GPL-2.0 */
/* Altera FPGA Ethernet MAC ANLT driver header
 * Copyright (C) 2025 Altera Corporation. All rights reserved
 *
 * Contributors:
 *      Krishna Kumar S R
 *
 */

#ifndef __FTILE_ANLT_H__
#define __FTILE_ANLT_H__
#include "intel_fpga_eth_main.h"
#include "intel_fpga_eth_hssi_itf.h"
#include "intel_fpga_hssi_driver.h"

enum anlt_err_code {
	ANLT_SUCCESS = 0, // Negotiation completed
	EANLT_IP_NOT_PRESENT, //has-anlt is false
	EANLT_NOT_ENABLED, //ANLT is not enabled
	EANLT_PHY_NOT_ABLE, //AN module is included in the Ethernet core
	EANLT_NEG_FAILURE, //AN completed, unable to find HCD
	EANLT_CHK_LP, // PHY capable but LP not capable of ANLT
};

struct anlt_err {
	enum anlt_err_code err_code;
	char *err_message;
};

enum an_ability_types {
	AN_IEEE_ABILITY_TYPE,
	AN_IEEE_NEG_ABILITY_TYPE,
	AN_IEEE_NEG_ABILITY_TYPE_EXT,
	AN_CONSORTIUM_ABILITY_TYPE,
};

enum ieee_an_ability {
	IEEE_AN_1000BASE_KX,
	IEEE_AN_10GBASE_KX4,
	IEEE_AN_10GBASE_KR,
	IEEE_AN_40GBASE_KR4,
	IEEE_AN_40GBASE_CR4,
	IEEE_AN_100GBASE_CR10,
	IEEE_AN_100GBASE_KP4,
	IEEE_AN_100GBASE_KR4,
	IEEE_AN_100GBASE_CR4,
	IEEE_AN_25GBASE_KR_S_CR_S,
	IEEE_AN_25GBASE_KR_CR,
	IEEE_AN_2500BASE_KX,
	IEEE_AN_5000BASE_KR,
	IEEE_AN_50GBASE_KR_CR,
	IEEE_AN_100GBASE_KR2_CR2,
	IEEE_AN_200GBASE_KR4_CR4,
	IEEE_AN_100GBASE_KR_CR,
	IEEE_AN_200GBASE_KR2_CR2,
	IEEE_AN_400GBASE_KR4_CR4,
};

enum ieee_neg_ability {
	AN_10GBASE_KR, // bit 12 of an status
	AN_40GBASE_KR4,
	AN_40GBASE_CR4,
	AN_100GBASE_KR4,
	AN_100GBASE_CR4,
	AN_25GBASE_KR_S_CR_S,
	AN_25GBASE_KR_CR,
	AN_50GBASE_KR_CR,
	AN_100GBASE_KR2_CR2,
	AN_200GBASE_KR4_CR4,
	AN_100GBASE_KR_CR,
	AN_200GBASE_KR2_CR2,
};

enum ieee_neg_ability_ext {
	AN_400GBASE_KR4_CR4,
};

enum consortium_ability {
	AN_25GBASE_KR1,
	AN_25GBASE_CR1,
	AN_50GBASE_KR2,
	AN_50GBASE_CR2,
	AN_400GBASE_KR8_CR8,
};

#define MAX_SUPPORTED_ETHTOOL_LINK_MODES 2
struct negotiated_ability {
	enum an_ability_types an_ability_type;
	u16 an_ability;
	int speed;
	char *fec_type;
	phy_interface_t phy_iface;
	int num_supp;
	int linkmode_bit[MAX_SUPPORTED_ETHTOOL_LINK_MODES];
};

void set_ethtool_linkmodes(unsigned long *link_mode_mask, u32 ability_type, u32 ability);

#endif //__FTILE_ANLT_H__
