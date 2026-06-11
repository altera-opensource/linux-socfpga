// SPDX-License-Identifier: GPL-2.0
/* Altera FPGA HSSI-SS interface API
 * Copyright (C) 2022, 2025 Altera Corporation. All rights reserved
 *
 * Contributors:
 *   Preetam Narayan
 *
 */

#include "intel_fpga_eth_hssi_itf.h"

static int hssi_csrrd32_errcheck(struct platform_device *pdev,
				 enum hssiss_tile_regbank regbank,
			  u32 chan,
			  u32 offset,
			  u32 *ret_value,
			  bool is_byte_addressing)
{
	struct get_set_csr_data csr_access;
	int ret_status = !INTEL_FPGA_RET_SUCCESS;

	csr_access.offs      = offset;
	csr_access.ch        = chan;
	if (is_byte_addressing)
		csr_access.word      = BYTE_ACCESS;
	else
		csr_access.word      = WORD_ACCESS;
	csr_access.reg_type  = regbank;

	ret_status = hssiss_execute_sal_cmd(pdev, SAL_GET_CSR,  &csr_access);

	*ret_value  = csr_access.data;

	return ret_status;
}

static u32 hssi_csrrd32_local(struct platform_device *pdev,
			      enum hssiss_tile_regbank regbank,
		 u32 chan,
		 u32 offset,
		 bool is_byte_addressing)
{
	u32 ret_value;

	int ret_status = !INTEL_FPGA_RET_SUCCESS;

	ret_status = hssi_csrrd32_errcheck(pdev,
					   regbank,
					   chan,
					   offset,
					   &ret_value,
					   is_byte_addressing);

	if (ret_status != INTEL_FPGA_RET_SUCCESS) {
		dev_err(&pdev->dev,
			"Error reading the 32 bit regbank %d offset 0x%x rc %d\n",
			regbank, offset, ret_status);
	}
	return ret_value;
}

u32 hssi_csrrd32(struct platform_device *pdev,
		 enum hssiss_tile_regbank regbank,
		 u32 chan,
		 u32 offset)
{
	return hssi_csrrd32_local(pdev, regbank, chan, offset, false);
}

u32 hssi_csrrd32_ba(struct platform_device *pdev,
		    enum hssiss_tile_regbank regbank,
		 u32 chan,
		 u32 offset)
{
	return hssi_csrrd32_local(pdev, regbank, chan, offset, true);
}

static void hssi_csrwr32_local(struct platform_device *pdev,
			       enum hssiss_tile_regbank regbank,
			       u32 chan,
			       u32 offset,
			       u32 reg_value,
			       bool is_byte_addressing)
{
	struct get_set_csr_data csr_access;
	int ret_status = !INTEL_FPGA_RET_SUCCESS;

	csr_access.offs      = offset;
	csr_access.ch        = chan;
	if (is_byte_addressing)
		csr_access.word      = BYTE_ACCESS;
	else
		csr_access.word      = WORD_ACCESS;
	csr_access.reg_type  = regbank;
	csr_access.data      = reg_value;

	ret_status = hssiss_execute_sal_cmd(pdev, SAL_SET_CSR,  &csr_access);

	if (ret_status != INTEL_FPGA_RET_SUCCESS) {
		dev_err(&pdev->dev,
			"Error writing 32 bit regbank %d offset 0x%x rc %d\n",
			regbank, offset, ret_status);
	}
}

void hssi_csrwr32(struct platform_device *pdev,
		  enum hssiss_tile_regbank regbank,
		  u32 chan,
		  u32 offset,
		  u32 reg_value)
{
	hssi_csrwr32_local(pdev, regbank, chan, offset, reg_value, false);
}

void hssi_csrwr32_ba(struct platform_device *pdev,
		     enum hssiss_tile_regbank regbank,
		  u32 chan,
		  u32 offset,
		  u32 reg_value)
{
	hssi_csrwr32_local(pdev, regbank, chan, offset, reg_value, true);
}

u8 hssi_csrrd8(struct platform_device *pdev,
	       enum hssiss_tile_regbank regbank,
	       u32 chan,
	       u32 offset)
{
	u8 ret_value;

	hssi_csrrd8_errcheck(pdev, regbank, chan, offset, &ret_value);

	return ret_value;
}

int hssi_csrrd8_errcheck(struct platform_device *pdev,
			 enum hssiss_tile_regbank regbank,
			 u32 chan,
			 u32 offset,
			 u8 *reg_value)
{
	int ret_status = !INTEL_FPGA_RET_SUCCESS;
	struct get_set_csr_data csr_access;

	csr_access.offs      = offset;
	csr_access.ch        = chan;
	csr_access.word      = BYTE_ACCESS;
	csr_access.reg_type  = regbank;

	ret_status = hssiss_execute_sal_cmd(pdev, SAL_GET_CSR,  &csr_access);

	if (ret_status == INTEL_FPGA_RET_SUCCESS) {
		*reg_value = csr_access.data & 0xFF;
	} else {
		dev_err(&pdev->dev,
			"csr read access error 8 bit regbank %d offset 0x%x rc %d\n",
			regbank, offset, ret_status);
	}

	return ret_status;
}

void hssi_csrwr8(struct platform_device *pdev,
		 enum hssiss_tile_regbank regbank,
		 u32 chan,
		 u32 offset,
		 u8 reg_value)
{
	struct get_set_csr_data csr_access;
	int ret_status = !INTEL_FPGA_RET_SUCCESS;

	csr_access.offs      = offset;
	csr_access.ch        = chan;
	csr_access.word      = BYTE_ACCESS;
	csr_access.reg_type  = regbank;
	csr_access.data      = reg_value;

	ret_status = hssiss_execute_sal_cmd(pdev, SAL_SET_CSR,  &csr_access);

	if (ret_status != INTEL_FPGA_RET_SUCCESS) {
		dev_err(&pdev->dev,
			"Error reading access 8 bit, regbank %d offset 0x%x rc %d\n",
			regbank, offset, ret_status);
	}
}

void hssi_set_bit(struct platform_device *pdev,
		  enum hssiss_tile_regbank regbank,
		  u32 chan,
		  u32 offset,
		  u32 bit_mask)
{
	u32 value;

	value = hssi_csrrd32(pdev, regbank, chan, offset);
	value |= bit_mask;
	hssi_csrwr32(pdev, regbank, chan, offset, value);
}

void hssi_set_bit_ba(struct platform_device *pdev,
		     enum hssiss_tile_regbank regbank,
		     u32 chan,
		     u32 offset,
		     u32 bit_mask)
{
	u32 value;

	value = hssi_csrrd32_ba(pdev, regbank, chan, offset);
	value |= bit_mask;
	hssi_csrwr32_ba(pdev, regbank, chan, offset, value);
}

void hssi_clear_bit(struct platform_device *pdev,
		    enum hssiss_tile_regbank regbank,
		    u32 chan,
		    u32 offset,
		    u32 bit_mask)
{
	u32 value;

	value = hssi_csrrd32(pdev, regbank, chan, offset);
	value &= ~bit_mask;
	hssi_csrwr32(pdev, regbank, chan, offset, value);
}

void hssi_clear_bit_ba(struct platform_device *pdev,
		       enum hssiss_tile_regbank regbank,
		       u32 chan,
		       u32 offset,
		       u32 bit_mask)
{
	u32 value;

	value = hssi_csrrd32_ba(pdev, regbank, chan, offset);
	value &= ~bit_mask;
	hssi_csrwr32_ba(pdev, regbank, chan, offset, value);
}

bool hssi_bit_is_set(struct platform_device *pdev,
		     enum hssiss_tile_regbank regbank,
		     u32 chan,
		     u32 offset,
		     u32 bit_mask)
{
	u32 value;

	value = hssi_csrrd32(pdev, regbank, chan, offset);

	return (value & bit_mask) ? true : false;
}

bool hssi_bit_is_set_ba(struct platform_device *pdev,
			enum hssiss_tile_regbank regbank,
			u32 chan,
			u32 offset,
			u32 bit_mask)
{
	u32 value;

	value = hssi_csrrd32_ba(pdev, regbank, chan, offset);

	return (value & bit_mask) ? true : false;
}

bool hssi_bit_is_clear(struct platform_device *pdev,
		       enum hssiss_tile_regbank regbank,
		       u32 chan,
		       u32 offset,
		       u32 bit_mask)
{
	u32 value;

	value = hssi_csrrd32(pdev, regbank, chan, offset);

	return (value & bit_mask) ? false : true;
}

bool hssi_bit_is_clear_ba(struct platform_device *pdev,
			  enum hssiss_tile_regbank regbank,
			  u32 chan,
			  u32 offset,
			  u32 bit_mask)
{
	u32 value;

	value = hssi_csrrd32_ba(pdev, regbank, chan, offset);

	return (value & bit_mask) ? false : true;
}

void hssi_reset_mac_stats(struct platform_device *pdev,
			  u32 port,
			  bool tx_rst,
			  bool rx_rst)
{
	struct reset_mac_stat_data rst_data = {
						.port = port,
						.tx = tx_rst,
						.rx = rx_rst
	};

	int ret_status;

	ret_status = hssiss_execute_sal_cmd(pdev,
					    SAL_RESET_MAC_STAT,
					    (void *)&rst_data);

	if (ret_status != INTEL_FPGA_RET_SUCCESS)
		dev_err(&pdev->dev, "Error on resetting mac statistics\n");
}

static u64  hssi_read_mac_stats(struct platform_device *pdev,
				u32 port,
				enum hssiss_mac_stat_counter_type stat_type,
				bool is_lsb)
{
	struct read_mac_stat_data mac_stat_data = {
						    .port_data = port,
						    .type = stat_type,
						    .lsb = is_lsb
						  };
	int ret_status;

	ret_status = hssiss_execute_sal_cmd(pdev,
					    SAL_READ_MAC_STAT,
					    (void *)&mac_stat_data);

	if (ret_status != INTEL_FPGA_RET_SUCCESS) {
		dev_err(&pdev->dev,
			"Error on reading mac statistics %d\n", ret_status);
	}

	return mac_stat_data.port_data;
}

u64 hssi_read_mac_stats64(struct platform_device *pdev, u32 port,
			  enum hssiss_mac_stat_counter_type stat_type)
{
	return (u64)(hssi_read_mac_stats(pdev, port, stat_type, false) << 32) |
		     hssi_read_mac_stats(pdev, port, stat_type, true);
}

int hssi_lock_mac_stats(struct platform_device *pdev, u32 port)
{
	return hssiss_lock_stats(pdev, port);
}

int hssi_set_mtu(struct platform_device *pdev, u32 cmd, void *mtu_data)
{
	return hssiss_set_mtu(pdev, cmd, mtu_data);
}

int hssi_unlock_mac_stats(struct platform_device *pdev, u32 port)
{
	return hssiss_unlock_stats(pdev, port);
}

int hssi_en_serial_loopback(struct platform_device *pdev,
			    enum hssiss_loopback_type type,
			    u32 port)
{
	int ret_status;
	struct set_loopback_data data;

	data.type = type;
	data.port = port;

	ret_status = hssiss_execute_sal_cmd(pdev,
					    SAL_ENABLE_LOOPBACK,
					    (void *)&data);

	if (ret_status != INTEL_FPGA_RET_SUCCESS) {
		dev_err(&pdev->dev,
			"Error enabling loopback rc: %d\n", ret_status);
	}

	return ret_status;
}

int hssi_dis_serial_loopback(struct platform_device *pdev, enum hssiss_loopback_type type,
			     u32 port)
{
	int ret_status;
	struct set_loopback_data data;

	data.type = type;
	data.port = port;

	ret_status = hssiss_execute_sal_cmd(pdev,
					    SAL_DISABLE_LOOPBACK,
					    (void *)&data);

	if (ret_status != INTEL_FPGA_RET_SUCCESS) {
		dev_err(&pdev->dev,
			"Error disabling loopback rc: %d\n", ret_status);
	}

	return ret_status;
}

void hssi_disable_hotplug(struct platform_device *pdev)
{
	hssiss_hotplug_enable(pdev, false);
}

void hssi_enable_hotplug(struct platform_device *pdev)
{
	hssiss_hotplug_enable(pdev, true);
}

bool hssi_ethport_is_stable(struct platform_device *pdev, u32 port, bool logging)
{
	bool retstatus;
	hssi_eth_port_sts pstatus;

	pstatus = hssiss_get_ethport_status(pdev, port);

	/* tx_lanes_stable, rx_pcs_ready and tx_pll_locked should be set for the */
	/* transmission to begin */
	retstatus =  pstatus.part.tx_lanes_stable &
		     pstatus.part.rx_pcs_ready    &
		     pstatus.part.tx_pll_locked;

	/* only print if the logging is enabled */
	/* logging is needed only at start up   */
	if (!logging)
		goto res;

	if (!retstatus) {
		dev_err(&pdev->dev,
			"Error Ethport is not stable\n");
	} else {
		dev_info(&pdev->dev,
			 "Ethport is stable now\n");
	}

	dev_err(&pdev->dev,
		"tx_lane:%d,rx_pcs:%d,tx_pll:%d\n",
		pstatus.part.tx_lanes_stable,
		pstatus.part.rx_pcs_ready,
		pstatus.part.tx_pll_locked);
res:
	return retstatus;
}

void hssi_reset_port(struct platform_device *pdev, u32 port)
{
	hssiss_reset_port(pdev, port);
}

int hssi_get_profile_lane_speed(struct platform_device *pdev, u32 port)
{
	hssi_eth_port_attr port_attr;
	int lane_speed = 0;

	port_attr = hssiss_get_ethport_attr(pdev, port);

	switch (port_attr.part.profile) {
	case HSSI_PORT_PROFILE_10GBE:
		lane_speed = LANE_10G;
		break;
	case HSSI_PORT_PROFILE_200GAUI_8:
	case HSSI_PORT_PROFILE_100GCAUI_4:
	case HSSI_PORT_PROFILE_25GBE:
	case HSSI_PORT_PROFILE_50GAUI_2:
		lane_speed = LANE_25G;
		break;
	case HSSI_PORT_PROFILE_400GAUI_8:
	case HSSI_PORT_PROFILE_200GAUI_4:
	case HSSI_PORT_PROFILE_100GAUI_2:
	case HSSI_PORT_PROFILE_50GAUI_1:
		lane_speed = LANE_50G;
		break;
	case HSSI_PORT_PROFILE_100GAUI_1:
	case HSSI_PORT_PROFILE_200GAUI_2:
	case HSSI_PORT_PROFILE_400GAUI_4:
		lane_speed = LANE_100G;
		break;
	default:
		break;
	}

	return lane_speed;
}

int hssi_get_pma_lane_count(struct platform_device *pdev, u32 port)
{
	hssi_eth_port_attr port_attr;
	int lane_count = 1; //default value

	port_attr = hssiss_get_ethport_attr(pdev, port);

	switch (port_attr.part.profile) {
	case HSSI_PORT_PROFILE_200GAUI_8:
	case HSSI_PORT_PROFILE_400GAUI_8:
		lane_count = 8;
		break;
	case HSSI_PORT_PROFILE_400GAUI_4:
	case HSSI_PORT_PROFILE_200GAUI_4:
	case HSSI_PORT_PROFILE_100GCAUI_4:
		lane_count = 4;
		break;
	case HSSI_PORT_PROFILE_100GAUI_2:
	case HSSI_PORT_PROFILE_200GAUI_2:
	case HSSI_PORT_PROFILE_50GAUI_2:
		lane_count = 2;
		break;
	default:
		break;
	}
	return lane_count;
}

int hssi_anlt_enable(struct platform_device *pdev, u32 port)
{
	return hssiss_anlt_update(pdev, port, true);
}

int hssi_anlt_disable(struct platform_device *pdev, u32 port)
{
	return hssiss_anlt_update(pdev, port, false);
}

u32 hssi_anlt_get_status(struct platform_device *pdev, u32 port)
{
	return hssiss_anlt_get_status(pdev, port);
}

void hssi_errpkt_cnt_reset(struct platform_device *pdev, int port)
{
	hssiss_usrspace_pkterr_cnt_rst(pdev, port);
}

u32 hssi_errpkt_cnt_read(struct platform_device *pdev, u32 addr_offs)
{
	return hssiss_usrspace_pkterr_cnt(pdev, addr_offs);
}

void hssi_errpkt_logic_en(struct platform_device *pdev, int port, bool enable)
{
	hssiss_usrspace_pkterr_logic_en(pdev, port, enable);
}

int hssi_get_dr_profile(struct platform_device *pdev, void *dr_data)
{
	return hssiss_execute_sal_cmd(pdev, SAL_GET_HSSI_PROFILE, dr_data);
}

int hssi_set_dr_profile(struct platform_device *pdev, void *dr_data)
{
	return hssiss_execute_sal_cmd(pdev, SAL_SET_HSSI_PROFILE, dr_data);
}

/**
 * hssi_num_dr_profiles - return the number of DR profiles from the DTS table.
 * @pdev: HSSI subsystem platform device
 *
 * Returns the profile count, or 0 if unavailable.
 */
u32 hssi_num_dr_profiles(struct platform_device *pdev)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	return (priv && priv->dr_profiles) ? priv->num_dr_profiles : 0;
}

/**
 * hssi_dr_profiles_available - check whether speed/FEC switching is possible.
 * @pdev: HSSI subsystem platform device
 *
 * Returns true only when the DTS has provided a non-empty profile table.
 */
bool hssi_dr_profiles_available(struct platform_device *pdev)
{
	return hssi_num_dr_profiles(pdev) ? true : false;
}

/**
 * hssi_find_dr_profile - find a DR profile array index matching the given speed
 *                        and FEC mode from the DTS-provided profile table.
 * @pdev:        HSSI subsystem platform device
 * @speed:       link speed in Mbps (e.g. 10000, 25000)
 * @fec:         FEC mode: 0 = no-fec, 1 = baser, 2 = rs
 * @rel_port:	 its the relative hssi port index from the base port 0
 * @profile_idx: output - array index into dr_profiles[] for the matching entry
 *
 * Returns 0 on success, -ENOENT when no matching profile is found.
 */
int hssi_find_dr_profile(struct platform_device *pdev, u32 speed, u32 fec,
			 u32 rel_port, u32 *profile_idx)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	u32 i;

	if (!hssi_num_dr_profiles(pdev))
		return -ENOENT;

	for (i = 0; i < priv->num_dr_profiles; i++) {
		if (priv->dr_profiles[i].speed == speed &&
		    priv->dr_profiles[i].fec   == fec &&
		    priv->dr_profiles[i].lane == rel_port){
			*profile_idx = i;
			return 0;
		}
	}

	return -ENOENT;
}

static struct hssi_dr_profile *hssi_find_active_dr_profile(struct hssiss_private *priv)
{
	return &priv->dr_profiles[priv->active_profile_idx];
}

/**
 * hssi_active_profile_fec - return the FEC mode of the currently active profile.
 * @pdev: HSSI subsystem platform device
 * @fec:  output - FEC mode (0 = no-fec, 1 = baser, 2 = rs)
 *
 * Looks up the active profile index from HSSI private data and returns the
 * corresponding FEC field from the DTS-provided profile table.
 * Returns 0 on success, -ENOENT when the active profile is not in the table.
 */
int hssi_active_profile_fec(struct platform_device *pdev, u32 *fec)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	struct hssi_dr_profile *p;

	if (!hssi_num_dr_profiles(pdev))
		return -ENOENT;

	p = hssi_find_active_dr_profile(priv);
	if (!p)
		return -ENOENT;

	*fec = p->fec;
	return 0;
}

bool hssi_active_profile_valid(struct platform_device *pdev)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	return priv && priv->active_profile_valid;
}

u32 hssi_active_profile_idx(struct platform_device *pdev)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	return priv->active_profile_idx;
}

void hssi_update_active_profile(struct platform_device *pdev, u32 idx)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	priv->active_profile_idx   = idx;
	priv->active_profile_valid = true;
}

void hssi_invalidate_active_profile(struct platform_device *pdev)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	priv->active_profile_idx   = 0;
	priv->active_profile_valid = false;
}

u32 hssi_get_dr_profile_hw_id(struct platform_device *pdev, u32 arr_idx)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);

	return priv->dr_profiles[arr_idx].profile_idx;
}

int hssi_get_active_profile(struct platform_device *pdev, u32 *profile)
{
	if (!hssi_active_profile_valid(pdev))
		return -EINVAL;

	*profile = hssi_active_profile_idx(pdev);

	return 0;
}

int hssi_get_active_profile_lane(struct platform_device *pdev)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	struct hssi_dr_profile *p;

	if (!priv->active_profile_valid)
		return -EINVAL;

	p = hssi_find_active_dr_profile(priv);
	return p ? (int)p->lane : -EINVAL;
}

int hssi_get_active_fec_mode(struct platform_device *pdev)
{
	u32 fec;

	if (hssi_active_profile_valid(pdev) &&
	    hssi_active_profile_fec(pdev, &fec) == 0)
		return fec;

	return -EINVAL;
}

const char *hssi_fec_type_str(enum ftile_fec_type fec)
{
	switch (fec) {
	case FTILE_FEC_RS:    return RSFEC;
	case FTILE_FEC_BASER: return BASER;
	case FTILE_FEC_NONE:
	default:              return NOFEC;
	}
}

/**
 * hssi_get_active_profile_speed - return the link speed (Mbps) of the active
 *                                 DR profile.
 * @pdev: HSSI subsystem platform device
 *
 * Returns the speed in Mbps on success, or -EINVAL when no valid active
 * profile is found.
 */
int hssi_get_active_profile_speed(struct platform_device *pdev)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	struct hssi_dr_profile *p;

	if (!hssi_active_profile_valid(pdev) || !priv->dr_profiles)
		return -EINVAL;

	p = hssi_find_active_dr_profile(priv);
	return p ? (int)p->speed : -EINVAL;
}
