// SPDX-License-Identifier: GPL-2.0
/* Altera FPGA Ethernet MAC ANLT IP driver
 * Copyright (C) 2025 Altera Corporation. All rights reserved
 *
 * Contributors:
 *      Krishna Kumar S R
 *
 */

#include "altera_fpga_anlt.h"
#include <linux/ethtool.h>

#define NELEMENTS(arr) (sizeof(arr) / sizeof((arr)[0]))

#define SET_LMM(__lmm) __set_bit(__lmm, link_mode_mask)

static struct anlt_err anlt_error[] = {
	{ANLT_SUCCESS, "ANLT completed successfully"},
	{EANLT_IP_NOT_PRESENT, "ANLT IP is not included in this design"},
	{EANLT_NOT_ENABLED, "ANLT IP is disabled"},
	{EANLT_PHY_NOT_ABLE, "PHY is not able to perform AN"},
	{EANLT_NEG_FAILURE, "Could not negotiate. No abilites GCD found"},
	{EANLT_CHK_LP, "LP not able to perform AN. Check LP AN abilities."},

};

static struct negotiated_ability an_ieee_ability[] = {
	{AN_IEEE_ABILITY_TYPE, IEEE_AN_1000BASE_KX, 1000, "no-fec", PHY_INTERFACE_MODE_NA,
		1, {ETHTOOL_LINK_MODE_1000baseKX_Full_BIT, 0}},
	{AN_IEEE_ABILITY_TYPE, IEEE_AN_10GBASE_KX4, 10000, "kr-fec", PHY_INTERFACE_MODE_NA,
		1, {ETHTOOL_LINK_MODE_10000baseKX4_Full_BIT, 0}},
	{AN_IEEE_ABILITY_TYPE, IEEE_AN_10GBASE_KR, 10000, "kr-fec", PHY_INTERFACE_MODE_10GKR,
		1, {ETHTOOL_LINK_MODE_10000baseKR_Full_BIT, 0}},
	{AN_IEEE_ABILITY_TYPE, IEEE_AN_40GBASE_KR4, 40000, "kr-fec", PHY_INTERFACE_MODE_NA,
		1, {ETHTOOL_LINK_MODE_40000baseKR4_Full_BIT, 0}},
	{AN_IEEE_ABILITY_TYPE, IEEE_AN_40GBASE_CR4, 40000, "kr-fec", PHY_INTERFACE_MODE_NA,
		1, {ETHTOOL_LINK_MODE_40000baseCR4_Full_BIT, 0}},
	{AN_IEEE_ABILITY_TYPE, IEEE_AN_100GBASE_CR10, 100000, "kr-fec", PHY_INTERFACE_MODE_NA,
		1, {ETHTOOL_LINK_MODE_100000baseCR_Full_BIT, 0}},
	{AN_IEEE_ABILITY_TYPE, IEEE_AN_100GBASE_KP4, 100000, "kr-fec", PHY_INTERFACE_MODE_NA,
		1, {ETHTOOL_LINK_MODE_100000baseKR4_Full_BIT, 0}},
	{AN_IEEE_ABILITY_TYPE, IEEE_AN_100GBASE_KR4, 100000, "kr-fec", PHY_INTERFACE_MODE_NA,
		1, {ETHTOOL_LINK_MODE_100000baseKR4_Full_BIT, 0}},
	{AN_IEEE_ABILITY_TYPE, IEEE_AN_100GBASE_CR4, 100000, "kr-fec", PHY_INTERFACE_MODE_NA,
		1, {ETHTOOL_LINK_MODE_100000baseCR4_Full_BIT, 0}},
	{AN_IEEE_ABILITY_TYPE, IEEE_AN_25GBASE_KR_S_CR_S, 25000, "kr-fec", PHY_INTERFACE_MODE_NA,
		2, {ETHTOOL_LINK_MODE_25000baseKR_Full_BIT,
			ETHTOOL_LINK_MODE_25000baseCR_Full_BIT}},
	{AN_IEEE_ABILITY_TYPE, IEEE_AN_25GBASE_KR_CR, 25000, "kr-fec", PHY_INTERFACE_MODE_25GKR,
		2, {ETHTOOL_LINK_MODE_25000baseKR_Full_BIT,
			ETHTOOL_LINK_MODE_25000baseCR_Full_BIT}},
	{AN_IEEE_ABILITY_TYPE, IEEE_AN_2500BASE_KX, 2500, "no-fec", PHY_INTERFACE_MODE_NA,
		1, {ETHTOOL_LINK_MODE_2500baseX_Full_BIT, 0}},
	{AN_IEEE_ABILITY_TYPE, IEEE_AN_5000BASE_KR, 5000, "no-fec", PHY_INTERFACE_MODE_NA,
		1, {ETHTOOL_LINK_MODE_5000baseT_Full_BIT, 0}},
	{AN_IEEE_ABILITY_TYPE, IEEE_AN_50GBASE_KR_CR, 50000, "kr-fec", PHY_INTERFACE_MODE_NA,
		2, {ETHTOOL_LINK_MODE_50000baseKR_Full_BIT,
			ETHTOOL_LINK_MODE_50000baseCR_Full_BIT}},
	{AN_IEEE_ABILITY_TYPE, IEEE_AN_100GBASE_KR2_CR2, 100000, "kr-fec", PHY_INTERFACE_MODE_NA,
		2, {ETHTOOL_LINK_MODE_100000baseKR2_Full_BIT,
			ETHTOOL_LINK_MODE_100000baseCR2_Full_BIT}},
	{AN_IEEE_ABILITY_TYPE, IEEE_AN_200GBASE_KR4_CR4, 200000, "kr-fec", PHY_INTERFACE_MODE_NA,
		2, {ETHTOOL_LINK_MODE_200000baseKR4_Full_BIT,
			ETHTOOL_LINK_MODE_200000baseCR4_Full_BIT}},
	{AN_IEEE_ABILITY_TYPE, IEEE_AN_100GBASE_KR_CR, 100000, "kr-fec", PHY_INTERFACE_MODE_NA,
		2, {ETHTOOL_LINK_MODE_100000baseKR_Full_BIT,
			ETHTOOL_LINK_MODE_100000baseCR_Full_BIT}},
	{AN_IEEE_ABILITY_TYPE, IEEE_AN_200GBASE_KR2_CR2, 200000, "kr-fec", PHY_INTERFACE_MODE_NA,
		2, {ETHTOOL_LINK_MODE_200000baseKR2_Full_BIT,
			ETHTOOL_LINK_MODE_200000baseCR2_Full_BIT}},
	{AN_IEEE_ABILITY_TYPE, IEEE_AN_400GBASE_KR4_CR4, 400000, "kr-fec", PHY_INTERFACE_MODE_NA,
		2, {ETHTOOL_LINK_MODE_400000baseKR4_Full_BIT,
			ETHTOOL_LINK_MODE_400000baseCR4_Full_BIT}},
};

//negotiated IEEE abilty
static struct negotiated_ability neg_ieee_ability[] = {
	{AN_IEEE_NEG_ABILITY_TYPE, AN_10GBASE_KR, 10000, "kr-fec", PHY_INTERFACE_MODE_10GKR,
		1, {ETHTOOL_LINK_MODE_10000baseKR_Full_BIT, 0}},
	{AN_IEEE_NEG_ABILITY_TYPE, AN_40GBASE_KR4, 40000, "kr-fec", PHY_INTERFACE_MODE_NA,
		1, {ETHTOOL_LINK_MODE_40000baseKR4_Full_BIT, 0}},
	{AN_IEEE_NEG_ABILITY_TYPE, AN_40GBASE_CR4, 40000, "kr-fec", PHY_INTERFACE_MODE_NA,
		1, {ETHTOOL_LINK_MODE_40000baseCR4_Full_BIT, 0}},
	{AN_IEEE_NEG_ABILITY_TYPE, AN_100GBASE_KR4, 100000, "kr-fec", PHY_INTERFACE_MODE_NA,
		1, {ETHTOOL_LINK_MODE_100000baseKR4_Full_BIT, 0}},
	{AN_IEEE_NEG_ABILITY_TYPE, AN_100GBASE_CR4, 100000, "kr-fec", PHY_INTERFACE_MODE_NA,
		1, {ETHTOOL_LINK_MODE_100000baseCR4_Full_BIT, 0}},
	{AN_IEEE_NEG_ABILITY_TYPE, AN_25GBASE_KR_S_CR_S, 25000, "kr-fec", PHY_INTERFACE_MODE_NA,
		2, {ETHTOOL_LINK_MODE_25000baseKR_Full_BIT,
			ETHTOOL_LINK_MODE_25000baseCR_Full_BIT}},
	{AN_IEEE_NEG_ABILITY_TYPE, AN_25GBASE_KR_CR, 25000, "kr-fec", PHY_INTERFACE_MODE_25GKR,
		2, {ETHTOOL_LINK_MODE_25000baseKR_Full_BIT,
			ETHTOOL_LINK_MODE_25000baseCR_Full_BIT}},
	{AN_IEEE_NEG_ABILITY_TYPE, AN_50GBASE_KR_CR, 50000, "kr-fec", PHY_INTERFACE_MODE_NA,
		2, {ETHTOOL_LINK_MODE_50000baseKR_Full_BIT,
			ETHTOOL_LINK_MODE_50000baseCR_Full_BIT}},
	{AN_IEEE_NEG_ABILITY_TYPE, AN_100GBASE_KR2_CR2, 100000, "kr-fec", PHY_INTERFACE_MODE_NA,
		2, {ETHTOOL_LINK_MODE_100000baseKR2_Full_BIT,
			ETHTOOL_LINK_MODE_100000baseCR2_Full_BIT}},
	{AN_IEEE_NEG_ABILITY_TYPE, AN_200GBASE_KR4_CR4, 200000, "kr-fec", PHY_INTERFACE_MODE_NA,
		2, {ETHTOOL_LINK_MODE_200000baseKR4_Full_BIT,
			ETHTOOL_LINK_MODE_200000baseCR4_Full_BIT}},
	{AN_IEEE_NEG_ABILITY_TYPE, AN_100GBASE_KR_CR, 100000, "kr-fec", PHY_INTERFACE_MODE_NA,
		2, {ETHTOOL_LINK_MODE_100000baseKR_Full_BIT,
			ETHTOOL_LINK_MODE_100000baseCR_Full_BIT}},
	{AN_IEEE_NEG_ABILITY_TYPE, AN_200GBASE_KR2_CR2, 200000, "kr-fec", PHY_INTERFACE_MODE_NA,
		2, {ETHTOOL_LINK_MODE_200000baseKR2_Full_BIT,
			ETHTOOL_LINK_MODE_200000baseCR2_Full_BIT}},
};

//negotiated IEEE ability extended
static struct negotiated_ability neg_ieee_abl_ext[] = {
	{AN_IEEE_NEG_ABILITY_TYPE_EXT, AN_400GBASE_KR4_CR4, 400000, "kr-fec", PHY_INTERFACE_MODE_NA,
		2, {ETHTOOL_LINK_MODE_400000baseKR4_Full_BIT,
			ETHTOOL_LINK_MODE_400000baseCR4_Full_BIT}},
};

//negotiated consortium ability
static struct negotiated_ability neg_cons_abl[] = {
	{AN_CONSORTIUM_ABILITY_TYPE, AN_25GBASE_KR1, 25000, "kr-fec", PHY_INTERFACE_MODE_25GKR,
		1, {ETHTOOL_LINK_MODE_25000baseKR_Full_BIT, 0}},
	{AN_CONSORTIUM_ABILITY_TYPE, AN_25GBASE_CR1, 25000, "no-fec", PHY_INTERFACE_MODE_NA,
		1, {ETHTOOL_LINK_MODE_25000baseCR_Full_BIT}},
	{AN_CONSORTIUM_ABILITY_TYPE, AN_50GBASE_KR2, 50000, "kr-fec", PHY_INTERFACE_MODE_50GKR,
		1, {ETHTOOL_LINK_MODE_50000baseKR2_Full_BIT}},
	{AN_CONSORTIUM_ABILITY_TYPE, AN_50GBASE_CR2, 50000, "no-fec", PHY_INTERFACE_MODE_NA,
		1, {ETHTOOL_LINK_MODE_50000baseCR2_Full_BIT}},
	{AN_CONSORTIUM_ABILITY_TYPE, AN_400GBASE_KR8_CR8, 400000, "kr-fec", PHY_INTERFACE_MODE_NA,
		2, {ETHTOOL_LINK_MODE_400000baseKR8_Full_BIT,
			ETHTOOL_LINK_MODE_400000baseCR8_Full_BIT}},
};

void set_ethtool_linkmodes(unsigned long *link_mode_mask, u32 ability_type, u32 abilitymask)
{
	int count = 0;
	int num = 0;
	int i;
	struct negotiated_ability *abilitymatrix;

	switch (ability_type) {
	case AN_IEEE_ABILITY_TYPE:
		abilitymatrix = an_ieee_ability;
		num = NELEMENTS(an_ieee_ability);
		break;
	case AN_IEEE_NEG_ABILITY_TYPE:
		abilitymatrix = neg_ieee_ability;
		num = NELEMENTS(neg_ieee_ability);
		break;
	case AN_IEEE_NEG_ABILITY_TYPE_EXT:
		abilitymatrix = neg_ieee_abl_ext;
		num = NELEMENTS(neg_ieee_abl_ext);
		break;
	case AN_CONSORTIUM_ABILITY_TYPE:
		abilitymatrix = neg_cons_abl;
		num = NELEMENTS(neg_cons_abl);
		break;
	default: //error
		abilitymatrix = neg_ieee_ability;
	}

	while (abilitymask != 0) {
		if (abilitymask & 0x1) {
			if (count >= num)
				return;
			for (i = 0; i < abilitymatrix[count].num_supp; i++)
				SET_LMM(abilitymatrix[count].linkmode_bit[i]);
		}
		count++;
		abilitymask = abilitymask >> 1;
	}
}

/* get_set_bit_position: returns the first set bit in the register */
static int get_set_bit_position(u32 reg)
{
	if (reg == 0)
		return INT_MAX;
	return find_first_bit((unsigned long *)&reg, sizeof(u32) * 8);
}

char *get_anlt_error(u32 err_code)
{
	if (err_code >= NELEMENTS(anlt_error))
		return "error";
	return anlt_error[err_code].err_message;
}

static int altera_fpga_anlt_set_neg(intel_fpga_xtile_eth_private *priv, u32 an_status)
{
	u32 an_abl = 0;

	if (an_status & HSSISS_CSR_IEEE_NEG_PORT_MASK) {
		an_abl = an_status & HSSISS_CSR_IEEE_NEG_PORT_MASK;
		an_abl = altera_rm_trail0(an_abl,
					 HSSISS_CSR_IEEE_NEG_PORT_MASK_START);
		if (an_abl == 0)
			return EANLT_PHY_NOT_ABLE;
		if (get_set_bit_position(an_abl) >= NELEMENTS(neg_ieee_ability))
			return EANLT_PHY_NOT_ABLE;
		priv->link_speed = neg_ieee_ability[get_set_bit_position(an_abl)].speed;
		if (an_status & HSSISS_CSR_RS_FEC_NEGOTIATED)
			priv->fec_type = neg_ieee_ability[get_set_bit_position(an_abl)].fec_type;
	} else if (an_status & HSSISS_CSR_CONS_NEG_PORT_MASK) {
		an_abl = an_status & HSSISS_CSR_CONS_NEG_PORT_MASK;
		an_abl = altera_rm_trail0(an_abl,
					 HSSISS_CSR_CONS_NEG_PORT_MASK_START);
		if (an_abl == 0)
			return EANLT_PHY_NOT_ABLE;
		if (get_set_bit_position(an_abl) >= NELEMENTS(neg_cons_abl))
			return EANLT_PHY_NOT_ABLE;
		priv->link_speed = neg_cons_abl[get_set_bit_position(an_abl)].speed;
		if (an_status & HSSISS_CSR_RS_FEC_NEGOTIATED)
			priv->fec_type = neg_cons_abl[get_set_bit_position(an_abl)].fec_type;
	} else if (an_status & HSSISS_CSR_CONS_NEG_PORT_400_KR4_CR4) {
		an_abl = an_status & HSSISS_CSR_CONS_NEG_PORT_400_KR4_CR4;
		an_abl = altera_rm_trail0(an_abl,
					 HSSISS_IEEE_EXT_NEG_PORT_MASK_START);
		if (an_abl == 0)
			return EANLT_PHY_NOT_ABLE;
		if (get_set_bit_position(an_abl) >= NELEMENTS(neg_ieee_abl_ext))
			return EANLT_PHY_NOT_ABLE;
		priv->link_speed = neg_ieee_abl_ext[get_set_bit_position(an_abl)].speed;
		if (an_status & HSSISS_CSR_RS_FEC_NEGOTIATED)
			priv->fec_type = neg_ieee_abl_ext[get_set_bit_position(an_abl)].fec_type;
	}
	priv->duplex = DUPLEX_FULL;
	if (an_status & HSSISS_CSR_FEC_MODES_MASK) {
		if (an_status & HSSISS_CSR_LL_FEC_NEGOTIATED)
			priv->fec_type = "ll-fec";
	}
	return ANLT_SUCCESS;
}

int altera_fpga_anlt_get_capabilities(intel_fpga_xtile_eth_private *priv)
{
	int ret = 0;
	u32 an_status;
	struct platform_device *pdev = NULL;
	bool autoneg_phy_ability, autoneg_link_status;
	bool autoneg_complete, rs_fec_negotiated, negotiation_failure;

	autoneg_phy_ability = false;
	autoneg_link_status = false;
	autoneg_complete = false;
	negotiation_failure = false;

	if (!priv)
		return -ENODEV;
	pdev = priv->pdev_hssi;

	an_status = hssi_anlt_get_status(pdev, priv->hssi_port);
	autoneg_complete = an_status & HSSISS_CSR_AN_COMPLETE;

	/* check if phy an ability is present
	 * PHY Autonegotiation Ability
	 * 1: PHY is able to perform AN
	 * 0: PHY is not able to perform AN
	 * This bit is tied high when AN module is included in the Ethernet core, low otherwise
	 */
	autoneg_phy_ability = an_status & HSSISS_CSR_PHY_AN_ABILITY;

	/* AutoNegotiation Status
	 * 1: Link is up
	 * 0: Link is down
	 */
	autoneg_link_status =  an_status & HSSISS_CSR_AN_STATUS_BIT;

	/* Autoneg Complete
	 * 1: AN Complete
	 * 0: AN in progress
	 * Corresponds to state variable mr_autoneg_complete in CL 73.10.1
	 */
	autoneg_complete = an_status & HSSISS_CSR_AN_COMPLETE;

	/* RS-FEC Negotiated
	 * 1: RS-FEC was negotiated for use on the link
	 * 0: The link will not use RS-FEC
	 */
	rs_fec_negotiated = an_status & HSSISS_CSR_RS_FEC_NEGOTIATED;

	/* negotiation failure
	 * 1: AN complete, but unable to get GCD
	 */
	negotiation_failure = an_status & HSSISS_CSR_NEG_FAILURE;

	if (autoneg_phy_ability) {
		if (autoneg_complete) {
			if (!negotiation_failure)
				ret = altera_fpga_anlt_set_neg(priv, an_status);
			else
				ret = EANLT_NEG_FAILURE;
		} else {
			ret = EANLT_CHK_LP;
		}
	} else {
		ret = EANLT_PHY_NOT_ABLE;
	}
	return ret;
}

