// SPDX-License-Identifier: GPL
/* Intel FPGA HSSI-SS interface API
 * Copyright (C) 2022,2023 Intel Corporation. All rights reserved
 *
    * Contributors:
    *   Preetam Narayan
 *
 */

#include "intel_fpga_eth_hssi_itf.h"


u32 hssi_csrrd32(struct platform_device *pdev, 
		 enum hssiss_tile_regbank regbank,
		 u32 chan, 
		 u32 offset) {
	u32 ret_value;

	int ret_status = !INTEL_FPGA_RET_SUCCESS;

	ret_status = hssi_csrrd32_errcheck(pdev,
					   regbank,
					   chan,
					   offset,
					   false,
					   &ret_value);

	if (ret_status != INTEL_FPGA_RET_SUCCESS) {
		dev_err(&pdev->dev,
			"Error reading the 32 bit regbank %d offset %x rc %x\n",
			regbank, offset, ret_status);
	}
	return ret_value;
}

u32 hssi_csrrd32_atomic(struct platform_device *pdev, 
			enum hssiss_tile_regbank regbank, 
			u32 chan, 
			u32 offset) {
	u32 ret_value;
	int ret_status = !INTEL_FPGA_RET_SUCCESS;

	ret_status = hssi_csrrd32_errcheck(pdev,
					   regbank,
					   chan,
					   offset,
					   true,
					   &ret_value);

	if (ret_status != INTEL_FPGA_RET_SUCCESS) {
		dev_err(&pdev->dev,
			"Error reading the 32 bit regbank %d offset %x rc %x\n",
			regbank, offset, ret_status);
	}

	return ret_value;
}

int hssi_csrrd32_errcheck(struct platform_device *pdev,
			  enum hssiss_tile_regbank regbank,
			  u32 chan,
			  u32 offset,
			  bool atomicity,
			  u32 *ret_value) {

	struct get_set_csr_data csr_access;
	int ret_status = !INTEL_FPGA_RET_SUCCESS;

	csr_access.offs      = offset;
	csr_access.ch        = chan;
	csr_access.word      = WORD_ACCESS;
	csr_access.reg_type  = regbank;

	if (atomicity)
		ret_status = hssiss_execute_sal_cmd_atomic(pdev,
							   SAL_GET_CSR,
							   &csr_access);
	else
		ret_status = hssiss_execute_sal_cmd(pdev, SAL_GET_CSR,  &csr_access);

	*ret_value  = csr_access.data;

	 return ret_status;
}


static void hssi_csrwr32_local(struct platform_device *pdev,
		  	       enum hssiss_tile_regbank regbank,
		  	       u32 chan,
		  	       u32 offset,
		  	       bool atomicity,
		  	       u32 reg_value) {
	struct get_set_csr_data csr_access;
	int ret_status = !INTEL_FPGA_RET_SUCCESS;

	csr_access.offs      = offset;
	csr_access.ch        = chan;
	csr_access.word      = WORD_ACCESS;
	csr_access.reg_type  = regbank;
	csr_access.data      = reg_value;

	if (atomicity)
		ret_status = hssiss_execute_sal_cmd_atomic(pdev,
							   SAL_SET_CSR,
							   &csr_access);
	else
		ret_status = hssiss_execute_sal_cmd(pdev, SAL_SET_CSR,  &csr_access);

	if (ret_status != INTEL_FPGA_RET_SUCCESS) {
		dev_err(&pdev->dev,
			"Error writing 32 bit regbank %d offset %x rc %x\n",
			regbank, offset, ret_status);
	}
}

void hssi_csrwr32(struct platform_device *pdev,
                  enum hssiss_tile_regbank regbank,
                  u32 chan,
                  u32 offset,
                  u32 reg_value) {

	hssi_csrwr32_local(pdev, regbank, chan, offset, false, reg_value);
}

void hssi_csrwr32_atomic(struct platform_device *pdev,
                  	 enum hssiss_tile_regbank regbank,
                  	 u32 chan,
                  	 u32 offset,
                  	 u32 reg_value) {

        hssi_csrwr32_local(pdev, regbank, chan, offset, true, reg_value);
}

u8 hssi_csrrd8(struct platform_device *pdev, 
	       enum hssiss_tile_regbank regbank, 
	       u32 chan, 
	       u32 offset) {
	
	u8 ret_value;

	hssi_csrrd8_errcheck(pdev, regbank, chan, offset, false, &ret_value);

	return ret_value;
}

u8 hssi_csrrd8_atomic(struct platform_device *pdev, 
		      enum hssiss_tile_regbank regbank, 
		      u32 chan, 
		      u32 offset) {
	u8 ret_value;

	hssi_csrrd8_errcheck(pdev, regbank, chan, offset, true, &ret_value);

	return ret_value;
}

int hssi_csrrd8_errcheck(struct platform_device *pdev,
			 enum hssiss_tile_regbank regbank,
			 u32 chan,
			 u32 offset,
			 bool atomic, u8 *reg_value) {
	int ret_status = !INTEL_FPGA_RET_SUCCESS;
	struct get_set_csr_data csr_access;

	csr_access.offs      = offset;
	csr_access.ch        = chan;
	csr_access.word      = BYTE_ACCESS;
	csr_access.reg_type  = regbank;

	if (atomic == true)
		ret_status = hssiss_execute_sal_cmd_atomic(pdev,
				                           SAL_GET_CSR,
							   &csr_access);
	else
		ret_status = hssiss_execute_sal_cmd(pdev, SAL_GET_CSR,  &csr_access);

	if (ret_status == INTEL_FPGA_RET_SUCCESS) {
		*reg_value = csr_access.data & 0xFF;
  	}
	else {
		dev_err(&pdev->dev,
			"csr read access error 8 bit regbank %d offset %x rc %x\n",
			regbank, offset, ret_status);
	}

	return ret_status;
}

void hssi_csrwr8(struct platform_device *pdev, 
		 enum hssiss_tile_regbank regbank, 
		 u32 chan, 
		 u32 offset, 
		 u8 reg_value) {
	
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
			"Error reading access 8 bit, regbank %d offset %x rc %d\n",
			regbank, offset, ret_status);
	}
}

void hssi_set_bit(struct platform_device *pdev, 
		  enum hssiss_tile_regbank regbank, 
		  u32 chan, 
		  u32 offset, 
		  u32 bit_mask) {
	u32 value;

	value = hssi_csrrd32(pdev, regbank, chan, offset);

	value |= bit_mask;

	hssi_csrwr32(pdev, regbank, chan, offset, value);
}

void hssi_set_bit_atomic(struct platform_device *pdev,
                 	  enum hssiss_tile_regbank regbank,
                  	  u32 chan,
                 	  u32 offset,
                 	  u32 bit_mask) {
        u32 value;

        value = hssi_csrrd32_atomic(pdev, regbank, chan, offset);

        value |= bit_mask;

        hssi_csrwr32_atomic(pdev, regbank, chan, offset, value);
}


void hssi_clear_bit(struct platform_device *pdev, 
		    enum hssiss_tile_regbank regbank, 
		    u32 chan, 
		    u32 offset, 
		    u32 bit_mask) {
	u32 value;

	value = hssi_csrrd32(pdev, regbank, chan, offset);

	value &= ~bit_mask;

	hssi_csrwr32(pdev, regbank, chan, offset, value);
}

void hssi_clear_bit_atomic(struct platform_device *pdev,
                    	    enum hssiss_tile_regbank regbank,
                    	    u32 chan,
                     	    u32 offset,
                    	    u32 bit_mask) {
        u32 value;

        value = hssi_csrrd32_atomic(pdev, regbank, chan, offset);

        value &= ~bit_mask;

        hssi_csrwr32_atomic(pdev, regbank, chan, offset, value);
}

bool hssi_bit_is_set(struct platform_device *pdev, 
		     enum hssiss_tile_regbank regbank, 
		     u32 chan, 
		     u32 offset, 
		     u32 bit_mask) {
	u32 value;

	value = hssi_csrrd32(pdev, regbank, chan, offset);

	return (value & bit_mask) ? true : false;
}

bool hssi_bit_is_clear(struct platform_device *pdev, 
		       enum hssiss_tile_regbank regbank, 
		       u32 chan, 
		       u32 offset, 
		       u32 bit_mask) {
	u32 value;

	value = hssi_csrrd32(pdev, regbank, chan, offset);

	return (value & bit_mask) ? false : true;
}

void hssi_reset_mac_stats(struct platform_device *pdev,
			  u32 port,
			  bool tx_rst,
			  bool rx_rst) {

	struct reset_mac_stat_data rst_data = {
						.port = port,
						.tx = tx_rst,
						.rx = rx_rst
	};

	int ret_status;

	ret_status = hssiss_execute_sal_cmd(pdev,
					    SAL_RESET_MAC_STAT,
					    (void *)&rst_data);

	if (ret_status != INTEL_FPGA_RET_SUCCESS) {
		dev_err(&pdev->dev, "Error on resetting mac statistics\n");
	}

}

static u64  hssi_read_mac_stats(struct platform_device *pdev,
				u32 port,
			  	enum hssiss_mac_stat_counter_type stat_type,
			       	bool is_lsb) {

	struct read_mac_stat_data mac_stat_data = {
						    .port_data = port,
						    .type = stat_type,
						    .lsb = is_lsb
						  };
	int ret_status;

	 ret_status = hssiss_execute_sal_cmd_atomic(pdev,
			 			    SAL_READ_MAC_STAT,
						    (void *)&mac_stat_data);

	 if (ret_status != INTEL_FPGA_RET_SUCCESS) {
		dev_err(&pdev->dev,
			"Error on reading mac statistics %x\n", ret_status);
	}

	return mac_stat_data.port_data;
}

u64 hssi_read_mac_stats64(struct platform_device *pdev, u32 port,
			  enum hssiss_mac_stat_counter_type stat_type) {

	return (u64)(hssi_read_mac_stats(pdev, port, stat_type, false) << 32) |
		     hssi_read_mac_stats(pdev, port, stat_type, true);
}

int hssi_en_serial_loopback(struct platform_device *pdev, u32 port)
{
	int ret_status;

	ret_status = hssiss_execute_sal_cmd(pdev,
					    SAL_ENABLE_LOOPBACK,
					    (void *)&port);

	if (ret_status != INTEL_FPGA_RET_SUCCESS)
	{
		dev_err(&pdev->dev,
			"Error enabling loopback rc: %x\n", ret_status);
	}

	return ret_status;
}

int hssi_dis_serial_loopback(struct platform_device *pdev, u32 port)
{
	int ret_status;

	ret_status = hssiss_execute_sal_cmd(pdev,
					    SAL_DISABLE_LOOPBACK,
					    (void *)&port);

	if (ret_status != INTEL_FPGA_RET_SUCCESS)
	{
		dev_err(&pdev->dev,
			"Error disabling loopback rc: %x\n", ret_status);
	}

	return ret_status;
}

void hssi_disable_hotplug(struct platform_device *pdev) {
	hssiss_hotplug_enable(pdev, false);
}

void hssi_enable_hotplug(struct platform_device *pdev) {
	hssiss_hotplug_enable(pdev, true);
}

bool hssi_ethport_is_stable(struct platform_device *pdev, u32 port, bool logging) {

	bool retstatus;
	hssi_eth_port_sts pstatus;

	pstatus = hssiss_get_ethport_status(pdev, port);

	/* tx_lanes_stable, rx_pcs_ready and tx_pll_locked should be set for the
	 * transmission to begin
	*/
	retstatus =  pstatus.part.tx_lanes_stable &
		     pstatus.part.rx_pcs_ready    &
		     pstatus.part.tx_pll_locked;

	/* only print if the logging is enabled */
	/* logging is needed only at start up   */
	if (logging == false) {
		goto res;
	}

	if (!retstatus) {

		dev_err(&pdev->dev,
			"Error Ethport is not stable\n");
	}
	else {
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
