// SPDX-License-Identifier: GPL-2.0
/* Altera FPGA HSSI GTS glue logic driver
 * Copyright (C) 2024, 2025 Altera Corporation. All rights reserved
 *
 * Contributors:
 *   Preetam Narayan
 *
 */
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include "altera_utils.h"
#include "intel_fpga_hssiss.h"
#include "intel_fpga_hssi_tile_ops.h"
#include "intel_fpga_eth_gts.h"
#include "intel_fpga_hssigl_gts_driver.h"

static const struct of_device_id intel_fpga_gts_ids[];

static void hssigldrv_gts_probe_init(struct platform_device *pdev)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	struct eth_sub_system *eth_ssr;

	priv->dev_specific = devm_kmalloc(priv->dev,
					  sizeof(struct eth_sub_system), GFP_KERNEL);

	eth_ssr = (struct eth_sub_system *)priv->dev_specific;

	eth_ssr->anlt_base_start = ANLT_BASE_START;
	eth_ssr->anlt_base_len = ANLT_BASE_LEN;

	eth_ssr->dr_base_start = DR_BASE_START;
	eth_ssr->dr_base_len = DR_BASE_LEN;

	eth_ssr->softip_base_start = SOFTIP_BASE_START;
	eth_ssr->softip_base_len = SOFTIP_BASE_LEN;

	eth_ssr->softip_ptp_start = SOFTIP_PTP_START;
	eth_ssr->softip_ptp_len = SOFTIP_PTP_LEN;

	eth_ssr->hardip_pld_start = HARDIP_PLD_START;
	eth_ssr->hardip_pld_len = HARDIP_PLD_LEN;

	eth_ssr->hardip_ptp_start = HARDIP_PTP_START;
	eth_ssr->hardip_ptp_len = HARDIP_PTP_LEN;

	eth_ssr->hardip_emac_start = HARDIP_EMAC_START;
	eth_ssr->hardip_emac_len = HARDIP_EMAC_LEN;

	eth_ssr->hardip_pcs_fec_start = HARDIP_PCS_FEC_START;
	eth_ssr->hardip_pcs_fec_len = HARDIP_PCS_FEC_LEN;

	eth_ssr->hardip_xcvr_pma_start = HARDIP_XCVR_PMA_START;
	eth_ssr->hardip_xcvr_pma_len = HARDIP_XCVR_PMA_LEN;

	eth_ssr->hardip_pma_start = HARDIP_PMA_START;
	eth_ssr->hardip_pma_len = HARDIP_PMA_LEN;

	eth_ssr->user_csr_start = USER_CSR_START;
	eth_ssr->user_csr_len = USER_CSR_LEN;
}

static u32 hssigldrv_gts_make_usrcsr_addr_offs(struct platform_device *pdev,
					       u8 port,
					       u32 offs)
{
        struct hssiss_private *priv = platform_get_drvdata(pdev);
        struct eth_sub_system *eth_ssr;
        u32 offs_addr = offs;

        eth_ssr = (struct eth_sub_system *)priv->dev_specific;
	
	offs_addr += eth_ssr->user_csr_start;
	BUG_ON(offs_addr >= eth_ssr->user_csr_start + 
				eth_ssr->user_csr_len);

	return offs_addr;
}

static u32 hssigldrv_gts_make_csr_addr_offs(struct platform_device *pdev,
					    u8 port,
					    enum hssiss_tile_regbank regbank,
					    u32 offs)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	struct eth_sub_system *eth_ssr;
	u32 offs_addr = offs;

	eth_ssr = (struct eth_sub_system *)priv->dev_specific;

	switch (regbank) {
	case HSSI_ANLT:
		//TBD
		break;
	case HSSI_DRCTRL:
		//TBD
		break;
	case HSSI_BASE_SOFTIP:
		offs_addr += eth_ssr->softip_base_start;
		BUG_ON(offs_addr >=
		       eth_ssr->hardip_pcs_fec_start + eth_ssr->softip_base_len);
		break;
	case HSSI_PTP_SOFTIP:
		offs_addr += eth_ssr->softip_ptp_start;
		BUG_ON(offs_addr >=
		       eth_ssr->hardip_pcs_fec_start + eth_ssr->softip_ptp_len);
		break;
	case HSSI_PCS_FEC_HARDIP:
		offs_addr += eth_ssr->hardip_pcs_fec_start;
		BUG_ON(offs_addr >=
		       eth_ssr->hardip_pcs_fec_start + eth_ssr->hardip_pcs_fec_len);
		break;
	case HSSI_EMAC_HARDIP:
		offs_addr += eth_ssr->hardip_emac_start;
		BUG_ON(offs_addr >=
		       eth_ssr->hardip_emac_start + eth_ssr->hardip_emac_len);
		break;
	case HSSI_PTP_HARDIP:
		offs_addr += eth_ssr->hardip_ptp_start;
		BUG_ON(offs_addr >=
		       eth_ssr->hardip_ptp_start + eth_ssr->hardip_ptp_len);
		break;
	case HSSI_XCVR_PMA_HARDIP:
		offs_addr += eth_ssr->hardip_xcvr_pma_start;
		BUG_ON(offs_addr >=
		       eth_ssr->hardip_xcvr_pma_start + eth_ssr->hardip_xcvr_pma_len);
		break;
	case HSSI_PMA_HARDIP:
		offs_addr += eth_ssr->hardip_pma_start;
		BUG_ON(offs_addr >=
		       eth_ssr->hardip_pma_start + eth_ssr->hardip_pma_len);
		break;
	case USERSPACE_CSR:
		offs_addr += eth_ssr->user_csr_start;
		BUG_ON(offs_addr >=
		       eth_ssr->user_csr_start + eth_ssr->user_csr_len);
		break;
	default:
		dev_err(&pdev->dev, "Non supported regbank\n");
	}

	return offs_addr + (port * CHANNEL_OFFSET);
}

static int hssigldrv_gts_en_loopback_mode(struct platform_device *pdev,
					  enum hssiss_loopback_type lb_type,
					  int port)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	void __iomem *base = priv->sscsr;
	u32 addr_offs;
	u32 val;
	u32 delay_wait = DELAY_READY_WAIT;

	switch (lb_type) {
	case NEAREND_FEC_LOOPBACK:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_PCS_FEC_HARDIP,
							     eth_hardip_pcsfec_csroffs(rsfec_tx_top));
		tse_set_bit(base, addr_offs, ETH_ENABLE_FEC_LOOPBACK);
		break;
	case FAREND_PAR_PMA_LOOPBACK:
		break;
	case NEAREND_SER_PMA_LOOPBACK:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_BASE_SOFTIP,
							     eth_soft_csroffs(eth_reset));
		tse_set_bit(base, addr_offs, ETH_SOFT_RX_RESET);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_BASE_SOFTIP,
				       eth_soft_csroffs(eth_reset_status));

		delay_wait = DELAY_READY_WAIT;
		do {
			udelay(5);
		} while ((delay_wait-- > 0) &&
			 (csrrd32(base, addr_offs) & ETH_SOFT_RST_ACK));

		if (!(csrrd32(base, addr_offs) & ETH_SOFT_RST_ACK))
			return -ETIME;

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_PMA_HARDIP,
				eth_hardip_pma_csroffs(SCMNG_PM_LINK_MNG_SIDE_CPI_REGS));
		csrwr32(ETH_ASSERT_PMA_SER_LBK_EN, base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_PMA_HARDIP,
				eth_hardip_pma_csroffs(SCMNG_PM_PHY_SIDE_CPI_REGS));

		delay_wait = DELAY_READY_WAIT;
		do {
			val = csrrd32(base, addr_offs);
			udelay(5);

		} while ((delay_wait-- > 0) &&
			 (val & ETH_ASSERT_PMA_SER_LBK_ACK) != ETH_DEASSERT_PMA_SER_LBK_DONE);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_PMA_HARDIP,
				eth_hardip_pma_csroffs(SCMNG_PM_LINK_MNG_SIDE_CPI_REGS));
		csrwr32(ETH_DEASSERT_PMA_SER_LBK_EN, base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_PMA_HARDIP,
				eth_hardip_pma_csroffs(SCMNG_PM_PHY_SIDE_CPI_REGS));

		delay_wait = DELAY_READY_WAIT;
		do {
			val = csrrd32(base, addr_offs);
			udelay(5);

		} while ((delay_wait-- > 0) &&
			 (val & ETH_ASSERT_PMA_SER_LBK_ACK) !=
					ETH_DEASSERT_PMA_SER_LBK_DONE);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev,
							     port, HSSI_BASE_SOFTIP,
					eth_soft_csroffs(eth_reset));
		tse_clear_bit(base, addr_offs, ETH_SOFT_RX_RESET);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_BASE_SOFTIP,
				      eth_soft_csroffs(eth_reset_status));

		delay_wait = DELAY_READY_WAIT;
		do {
			val = csrrd32(base, addr_offs);
			udelay(5);

		} while ((delay_wait-- > 0) && (val & ETH_SOFT_RST_ACK));

		if ((csrrd32(base, addr_offs) & ETH_SOFT_RST_ACK))
			return -ETIME;

		return 0;
	
	case NEAREND_PAR_PMA_LOOPBACK:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port, HSSI_PMA_HARDIP,
							     eth_hardip_pma_csroffs(pre_pma_lblk));

		tse_set_bit(base, addr_offs, ETH_ENABLE_NEAREND_PAR_PMA_LOOPBACK);
		break;
	
	case NEAREND_XCVRIF_LOOPBACK:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_XCVR_PMA_HARDIP,
				eth_hardip_xcvr_pma_csroffs(sm_xcvrif_reg_9));

		tse_set_bit(base, addr_offs, ETH_ENABLE_XCVRIF_LOOPBACK);
		break;
	
	case NEAREND_MAC_LOOPBACK:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_EMAC_HARDIP,
					eth_hardip_emac_csroffs(rxmac_ehip_cfg));

		tse_set_bit(base, addr_offs, ETH_ENABLE_MAC_LOOPBACK);
		break;
	
	case NEAREND_PAR_PCS_LOOPBACK:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_PCS_FEC_HARDIP,
					eth_hardip_pcsfec_csroffs(phy_ehip_pcs_modes));

		tse_set_bit(base, addr_offs, ETH_ENABLE_NEAREND_PCS_LOOPBACK);
		break;
	
	case FAREND_PAR_PCS_LOOPBACK:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_PCS_FEC_HARDIP,
					eth_hardip_pcsfec_csroffs(phy_ehip_pcs_modes));

		tse_set_bit(base, addr_offs, ETH_ENABLE_FAREND_PCS_LOOPBACK);
		break;
	}

	return 0;
}

static int hssigldrv_gts_dis_loopback_mode(struct platform_device *pdev,
					   enum hssiss_loopback_type lb_type,
					   int port)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	void __iomem *base = priv->sscsr;
	u32 addr_offs;
	u32 delay_wait = DELAY_READY_WAIT;
	u32 val;

	switch (lb_type) {
	case NEAREND_FEC_LOOPBACK:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_PCS_FEC_HARDIP,
					eth_hardip_xcvr_pma_csroffs(sm_xcvrif_reg_9));

		tse_clear_bit(base, addr_offs, ETH_DISABLE_FEC_LOOPBACK);
		break;
	case FAREND_PAR_PMA_LOOPBACK:
		break;
	case NEAREND_SER_PMA_LOOPBACK:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_BASE_SOFTIP,
						eth_soft_csroffs(eth_reset));

		tse_set_bit(base, addr_offs, ETH_SOFT_RX_RESET);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_BASE_SOFTIP,
						eth_soft_csroffs(eth_reset_status));

		delay_wait = DELAY_READY_WAIT;
		do {
			val = csrrd32(base, addr_offs);
			udelay(5);
		} while ((delay_wait-- > 0) && (val & ETH_SOFT_RST_ACK));

		if (!(csrrd32(base, addr_offs) & ETH_SOFT_RST_ACK))
			return -ETIME;

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_PMA_HARDIP,
				eth_hardip_pma_csroffs(SCMNG_PM_LINK_MNG_SIDE_CPI_REGS));
		csrwr32(ETH_ASSERT_PMA_SER_LBK_DIS, base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_PMA_HARDIP,
				eth_hardip_pma_csroffs(SCMNG_PM_PHY_SIDE_CPI_REGS));

		delay_wait = DELAY_READY_WAIT;
		do {
			val = csrrd32(base, addr_offs);
			udelay(5);

		} while ((delay_wait-- > 0) && (val & ETH_ASSERT_PMA_SER_LBK_ACK)
							!= ETH_DEASSERT_PMA_SER_LBK_DONE);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_PMA_HARDIP,
				eth_hardip_pma_csroffs(SCMNG_PM_LINK_MNG_SIDE_CPI_REGS));
		csrwr32(ETH_DEASSERT_PMA_SER_LBK_DIS, base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_PMA_HARDIP,
				eth_hardip_pma_csroffs(SCMNG_PM_PHY_SIDE_CPI_REGS));

		delay_wait = DELAY_READY_WAIT;
		do {
			val = csrrd32(base, addr_offs);
			udelay(5);

		} while ((delay_wait-- > 0) && (val & ETH_DEASSERT_PMA_SER_LBK_ACK) !=
							ETH_DEASSERT_PMA_SER_LBK_DONE);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_BASE_SOFTIP,
					eth_soft_csroffs(eth_reset));
		tse_clear_bit(base, addr_offs, ETH_SOFT_RX_RESET);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_BASE_SOFTIP,
					eth_soft_csroffs(eth_reset_status));

		delay_wait = DELAY_READY_WAIT;
		do {
			val = csrrd32(base, addr_offs);
			udelay(5);
		} while ((delay_wait-- > 0) && (val & ETH_SOFT_RST_ACK));

		if ((csrrd32(base, addr_offs) & ETH_SOFT_RST_ACK))
			return -ETIME;

		return 0;

	case NEAREND_PAR_PMA_LOOPBACK:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_PMA_HARDIP,
					eth_hardip_pma_csroffs(pre_pma_lblk));

		tse_clear_bit(base, addr_offs, ETH_ENABLE_NEAREND_PAR_PMA_LOOPBACK);
		break;
	case NEAREND_XCVRIF_LOOPBACK:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_XCVR_PMA_HARDIP,
					eth_hardip_xcvr_pma_csroffs(sm_xcvrif_reg_9));

		tse_clear_bit(base, addr_offs, ETH_DISABLE_XCVRIF_LOOPBACK);
		break;
	case NEAREND_MAC_LOOPBACK:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_EMAC_HARDIP,
					eth_hardip_emac_csroffs(rxmac_ehip_cfg));

		tse_clear_bit(base, addr_offs, ETH_DISABLE_MAC_LOOPBACK);
		break;
	case NEAREND_PAR_PCS_LOOPBACK:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_PCS_FEC_HARDIP,
					eth_hardip_pcsfec_csroffs(phy_ehip_pcs_modes));

		tse_clear_bit(base, addr_offs, ETH_DISABLE_NEAREND_PCS_LOOPBACK);
		break;
	case FAREND_PAR_PCS_LOOPBACK:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
							     HSSI_PCS_FEC_HARDIP,
					eth_hardip_pcsfec_csroffs(phy_ehip_pcs_modes));

		tse_clear_bit(base, addr_offs, ETH_DISABLE_FAREND_PCS_LOOPBACK);
		break;
	}

	return 0;
}

static hssi_eth_port_sts hssigldrv_gts_get_ethport_status(struct platform_device *pdev,
							  int port)
{
	u32 pcs_status;
	u32 addr_offs;
	hssi_eth_port_sts port_sts;
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	void __iomem *base = priv->sscsr;

	port_sts.full = 0;

	addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
						     HSSI_BASE_SOFTIP,
					eth_soft_csroffs(phy_tx_pll_locked));
	port_sts.part.tx_pll_locked = csrrd32(base, addr_offs);

	addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
						     HSSI_BASE_SOFTIP,
					eth_soft_csroffs(pcs_status));
	pcs_status = csrrd32(base, addr_offs);
	port_sts.part.rx_pcs_ready = RX_PCS_READY_STATUS(pcs_status);
	port_sts.part.tx_lanes_stable = TX_LANE_STABLE_STATUS(pcs_status);

	return port_sts;
}

static int hssigldrv_gts_reset_mac_stat(struct platform_device *pdev, int port)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	void __iomem *base = priv->sscsr;
	u32 addr_offs;

	addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
						     HSSI_EMAC_HARDIP,
					eth_hardip_emac_csroffs(cntr_tx_config));

	tse_set_bit(base, addr_offs, ETH_TX_CNTR_CFG_RST_ALL);
	tse_clear_bit(base, addr_offs, ETH_TX_CNTR_CFG_RST_ALL);

	addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
						     HSSI_EMAC_HARDIP,
					eth_hardip_emac_csroffs(cntr_rx_config));

	tse_set_bit(base, addr_offs, ETH_RX_CNTR_CFG_RST_ALL);
	tse_clear_bit(base, addr_offs, ETH_RX_CNTR_CFG_RST_ALL);

	return 0;
}

static int hssigldrv_gts_get_mtu(struct platform_device *pdev,
				 void *mtu_data)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	void __iomem *base = priv->sscsr;
	struct get_mtu_data *data = mtu_data;
	u32 addr_offs;

	addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port,
						     HSSI_EMAC_HARDIP,
					eth_hardip_emac_csroffs(max_tx_size_config));
	data->max_tx_frame_size = csrrd32(base, addr_offs);

	addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port,
						     HSSI_EMAC_HARDIP,
					eth_hardip_emac_csroffs(max_rx_size_config));
	data->max_rx_frame_size = csrrd32(base, addr_offs);

	return 0;
}

static int hssigldrv_gts_set_mtu(struct platform_device *pdev,
                                 void *mtu_data)
{
        struct hssiss_private *priv = platform_get_drvdata(pdev);
        void __iomem *base = priv->sscsr;
	struct set_mtu_data *data = mtu_data;
        u32 addr_offs;

        addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port,
                                                     HSSI_EMAC_HARDIP,
                                        eth_hardip_emac_csroffs(max_tx_size_config));
        csrwr32(data->max_tx_frame_size, base, addr_offs);

        addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port,
                                                     HSSI_EMAC_HARDIP,
                                        eth_hardip_emac_csroffs(max_rx_size_config));
        csrwr32(data->max_rx_frame_size, base, addr_offs);

        return 0;
}

static int hssigldrv_gts_read_mac_stats(struct platform_device *pdev,
					struct read_mac_stat_data *data)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	void __iomem *base = priv->sscsr;
	u32 addr_offs;
	u64 rdata = 0;
	u64 rdata_lo = 0;
	u64 rdata_hi = 0;

	switch (data->type) {
	case MACSTAT_TX_PACKETS:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_mcast_data_ok_lo));
		rdata_lo = csrrd32(base, addr_offs);
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_mcast_data_ok_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_bcast_data_ok_lo));
		rdata_lo = csrrd32(base, addr_offs);
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_bcast_data_ok_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_ucast_data_ok_lo));
		rdata_lo = csrrd32(base, addr_offs);
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_ucast_data_ok_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_mcast_ctrl_lo));
		rdata_lo = csrrd32(base, addr_offs);
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_mcast_ctrl_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_bcast_ctrl_lo));
		rdata_lo = csrrd32(base, addr_offs);
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_bcast_ctrl_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_ucast_ctrl_lo));
		rdata_lo = csrrd32(base, addr_offs);
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_ucast_ctrl_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		break;
	case MACSTAT_RX_PACKETS:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_mcast_data_ok_lo));
		rdata_lo = csrrd32(base, addr_offs);
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_mcast_data_ok_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_bcast_data_ok_lo));
		rdata_lo = csrrd32(base, addr_offs);
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_bcast_data_ok_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_ucast_data_ok_lo));
		rdata_lo = csrrd32(base, addr_offs);
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_ucast_data_ok_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_mcast_ctrl_lo));
		rdata_lo = csrrd32(base, addr_offs);
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_mcast_ctrl_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_bcast_ctrl_lo));
		rdata_lo = csrrd32(base, addr_offs);
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_bcast_ctrl_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_ucast_ctrl_lo));
		rdata_lo = csrrd32(base, addr_offs);
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_ucast_ctrl_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		break;
	case MACSTAT_RX_CRC_ERRORS:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_fcs_lo));
		rdata = csrrd32(base, addr_offs);

		break;
	case MACSTAT_RX_ALIGN_ERRORS:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_fcs_lo));
		rdata = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_runt_lo));
		rdata -= csrrd32(base, addr_offs);

		break;
        case MACSTAT_TX_CRC_ERRORS:
                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_fcs_lo));
                rdata = csrrd32(base, addr_offs);

                break;
        case MACSTAT_TX_ALIGN_ERRORS:
                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_fcs_lo));
                rdata = csrrd32(base, addr_offs);

                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_runt_lo));
                rdata -= csrrd32(base, addr_offs);

                break;
	case MACSTAT_TX_BYTES:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_payloadoctetsok_lo));
		rdata_lo = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_payloadoctetsok_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		break;
	case MACSTAT_RX_BYTES:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_payloadoctetsok_lo));
		rdata_lo = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_payloadoctetsok_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		break;
	case MACSTAT_TX_PAUSE:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_pause_lo));
		rdata_lo = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_pause_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		break;
	case MACSTAT_RX_PAUSE:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_pause_lo));
		rdata_lo = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_pause_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		break;
	case MACSTAT_RX_ERRORS:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_dropped_lo));
		rdata = csrrd32(base, addr_offs);

		break;
	case MACSTAT_TX_ERRORS:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_dropped_lo));
		rdata = csrrd32(base, addr_offs);

		break;
	case MACSTAT_RX_UNICAST:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_ucast_data_ok_lo));
		rdata_lo = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_ucast_data_ok_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_ucast_ctrl_lo));
		rdata_lo = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_ucast_ctrl_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		break;
	case MACSTAT_RX_MULTICAST:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_mcast_data_ok_lo));
		rdata_lo = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_mcast_data_ok_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_mcast_ctrl_lo));
		rdata_lo = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_mcast_ctrl_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		break;
	case MACSTAT_RX_BROADCAST:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_bcast_data_ok_lo));
		rdata_lo = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_bcast_data_ok_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_bcast_ctrl_lo));
		rdata_lo = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_bcast_ctrl_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		break;
	case MACSTAT_TX_DISCARDS:
		rdata = 0;

		break;

	case MACSTAT_TX_UNICAST:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_ucast_data_ok_lo));
		rdata_lo = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_ucast_data_ok_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_ucast_ctrl_lo));
		rdata_lo = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_ucast_ctrl_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		break;
	case MACSTAT_TX_MULTICAST:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_mcast_data_ok_lo));
		rdata_lo = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_mcast_data_ok_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_mcast_ctrl_lo));
		rdata_lo = csrrd32(base, addr_offs);
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_mcast_ctrl_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		break;
	case MACSTAT_TX_BROADCAST:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_bcast_data_ok_lo));
		rdata_lo = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_bcast_data_ok_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_bcast_ctrl_lo));
		rdata_lo = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_bcast_ctrl_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		break;
        case MACSTAT_TX_ETHER_DROPS:
                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_dropped_lo));
                rdata = csrrd32(base, addr_offs);

                break;
	case MACSTAT_ETHER_DROPS:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_dropped_lo));
		rdata = csrrd32(base, addr_offs);

		break;
        case MACSTAT_TX_TOTAL_BYTES:
                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_octetsok_lo));
                rdata_lo = csrrd32(base, addr_offs);

                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_octetsok_hi));
                rdata_hi = csrrd32(base, addr_offs);
                rdata += U64_FROM_32(rdata_hi, rdata_lo);

                break;
	case MACSTAT_RX_TOTAL_BYTES:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_octetsok_lo));
		rdata_lo = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_octetsok_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		break;
	case MACSTAT_RX_TOTAL_PACKETS:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_mcast_data_ok_lo));
		rdata_lo = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_mcast_data_ok_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_bcast_data_ok_lo));
		rdata_lo = csrrd32(base, addr_offs);
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_bcast_data_ok_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_ucast_data_ok_lo));
		rdata_lo = csrrd32(base, addr_offs);
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_ucast_data_ok_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_mcast_ctrl_lo));
		rdata_lo = csrrd32(base, addr_offs);
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_mcast_ctrl_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_bcast_ctrl_lo));
		rdata_lo = csrrd32(base, addr_offs);
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_bcast_ctrl_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_ucast_ctrl_lo));
		rdata_lo = csrrd32(base, addr_offs);
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_ucast_ctrl_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_mcast_data_err_lo));
		rdata += csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_bcast_data_err_lo));
		rdata += csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_ucast_data_err_lo));
		rdata += csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_mcast_ctrl_err_lo));
		rdata += csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_bcast_ctrl_err_lo));
		rdata += csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_ucast_ctrl_err_lo));
		rdata += csrrd32(base, addr_offs);

		break;
	case MACSTAT_TX_TOTAL_PACKETS:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_mcast_data_ok_lo));
		rdata_lo = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_mcast_data_ok_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_bcast_data_ok_lo));
		rdata_lo = csrrd32(base, addr_offs);
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_bcast_data_ok_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_ucast_data_ok_lo));
		rdata_lo = csrrd32(base, addr_offs);
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_ucast_data_ok_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_mcast_ctrl_lo));
		rdata_lo = csrrd32(base, addr_offs);
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_mcast_ctrl_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_bcast_ctrl_lo));
		rdata_lo = csrrd32(base, addr_offs);
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_bcast_ctrl_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_ucast_ctrl_lo));
		rdata_lo = csrrd32(base, addr_offs);
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_ucast_ctrl_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_mcast_data_err_lo));
		rdata += csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_bcast_data_err_lo));
		rdata += csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_ucast_data_err_lo));
		rdata += csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_mcast_ctrl_err_lo));
		rdata += csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_bcast_ctrl_err_lo));
		rdata += csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_tx_ucast_ctrl_err_lo));
		rdata += csrrd32(base, addr_offs);

		break;
        case MACSTAT_TX_UNDERSIZE:
                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_runt_lo));
                rdata = csrrd32(base, addr_offs);
                break;
	case MACSTAT_RX_UNDERSIZE:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_runt_lo));
		rdata = csrrd32(base, addr_offs);
		break;
        case MACSTAT_TX_OVERSIZE:
                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_oversize_lo));
                rdata = csrrd32(base, addr_offs);
                break;
	case MACSTAT_RX_OVERSIZE:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_oversize_lo));
		rdata = csrrd32(base, addr_offs);
		break;
        case MACSTAT_TX_64_BYTES:
                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_64b_lo));
                rdata_lo = csrrd32(base, addr_offs);

                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_64b_hi));
                rdata_hi = csrrd32(base, addr_offs);
                rdata += U64_FROM_32(rdata_hi, rdata_lo);

                break;
	case MACSTAT_RX_64_BYTES:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_64b_lo));
		rdata_lo = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_64b_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		break;
        case MACSTAT_TX_65_127_BYTES:
                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_65to127b_lo));
                rdata_lo = csrrd32(base, addr_offs);
                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_65to127b_hi));
                rdata_hi = csrrd32(base, addr_offs);
                rdata += U64_FROM_32(rdata_hi, rdata_lo);

                break;
	case MACSTAT_RX_65_127_BYTES:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_65to127b_lo));
		rdata_lo = csrrd32(base, addr_offs);
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_65to127b_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		break;
        case MACSTAT_TX_128_255_BYTES:
                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_128to255b_lo));
                rdata_lo = csrrd32(base, addr_offs);

                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_128to255b_hi));
                rdata_hi = csrrd32(base, addr_offs);
                rdata += U64_FROM_32(rdata_hi, rdata_lo);

                break;
	case MACSTAT_RX_128_255_BYTES:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_128to255b_lo));
		rdata_lo = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_128to255b_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		break;
        case MACSTAT_TX_256_511_BYTES:
                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_256to511b_lo));
                rdata_lo = csrrd32(base, addr_offs);

                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_256to511b_hi));
                rdata_hi = csrrd32(base, addr_offs);
                rdata += U64_FROM_32(rdata_hi, rdata_lo);

                break;
	case MACSTAT_RX_256_511_BYTES:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_256to511b_lo));
		rdata_lo = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_256to511b_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		break;
        case MACSTAT_TX_512_1023_BYTES:
                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_512to1023b_lo));
                rdata_lo = csrrd32(base, addr_offs);

                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_512to1023b_hi));
                rdata_hi = csrrd32(base, addr_offs);
                rdata += U64_FROM_32(rdata_hi, rdata_lo);

                break;
	case MACSTAT_RX_512_1023_BYTES:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_512to1023b_lo));
		rdata_lo = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_512to1023b_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		break;
        case MACSTAT_TX_1024_1518_BYTES:
                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_1024to1518b_lo));
                rdata_lo = csrrd32(base, addr_offs);

                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_1024to1518b_hi));
                rdata_hi = csrrd32(base, addr_offs);
                rdata += U64_FROM_32(rdata_hi, rdata_lo);

                break;
	case MACSTAT_RX_1024_1518_BYTES:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_1024to1518b_lo));
		rdata_lo = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_1024to1518b_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		break;
        case MACSTAT_TX_GTE_1519_BYTES:
                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_1519tomaxb_lo));
                rdata_lo = csrrd32(base, addr_offs);

                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_1519tomaxb_hi));
                rdata_hi = csrrd32(base, addr_offs);
                rdata += U64_FROM_32(rdata_hi, rdata_lo);

                break;
	case MACSTAT_RX_GTE_1519_BYTES:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_1519tomaxb_lo));
		rdata_lo = csrrd32(base, addr_offs);

		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_1519tomaxb_hi));
		rdata_hi = csrrd32(base, addr_offs);
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

		break;
        case MACSTAT_TX_JABBERS:
                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_jabbers_lo));
                rdata = csrrd32(base, addr_offs);

                break;
	case MACSTAT_RX_JABBERS:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_jabbers_lo));
		rdata = csrrd32(base, addr_offs);

		break;
        case MACSTAT_TX_RUNTS:
                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_fragments_lo));
                rdata = csrrd32(base, addr_offs);

                break;
	case MACSTAT_RX_RUNTS:
		addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
							     HSSI_EMAC_HARDIP,
						eth_hardip_emac_csroffs(cntr_rx_fragments_lo));
		rdata = csrrd32(base, addr_offs);

		break;
        case MACSTAT_TX_SOP_COUNT:
                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_st_lo));
                rdata_lo = csrrd32(base, addr_offs);

                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_tx_st_hi));
                rdata_hi = csrrd32(base, addr_offs);
                
		rdata += U64_FROM_32(rdata_hi, rdata_lo);

                break;
        case MACSTAT_RX_SOP_COUNT:
                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_rx_st_lo));
                rdata_lo = csrrd32(base, addr_offs);

                addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, data->port_data,
                                                             HSSI_EMAC_HARDIP,
                                                eth_hardip_emac_csroffs(cntr_rx_st_hi));
                rdata_hi = csrrd32(base, addr_offs);

                rdata += U64_FROM_32(rdata_hi, rdata_lo);

                break;
	default:
		dev_err(&pdev->dev, "Unknown stat type\n");
	}

	if (data->lsb)
		rdata = lower_32_bits(rdata);
	else
		rdata = upper_32_bits(rdata);

	data->port_data = rdata;
	return 0;
}

static int hssigldrv_gts_freeze_stats(struct platform_device *pdev, int port)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	void __iomem *base = priv->sscsr;
	u32 addr_offs;

	addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
						     HSSI_EMAC_HARDIP,
					eth_hardip_emac_csroffs(cntr_tx_config));
	tse_set_bit(base, addr_offs, ETH_FREEZE_TX_MAC_STATS);

	addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
						     HSSI_EMAC_HARDIP,
					eth_hardip_emac_csroffs(cntr_rx_config));
	tse_set_bit(base, addr_offs, ETH_FREEZE_RX_MAC_STATS);
	
	return 0;
}

static int hssigldrv_gts_defreeze_stats(struct platform_device *pdev, int port)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	void __iomem *base = priv->sscsr;
	u32 addr_offs;

	addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
						     HSSI_EMAC_HARDIP,
					eth_hardip_emac_csroffs(cntr_rx_config));
	tse_clear_bit(base, addr_offs, ETH_DEFREEZE_RX_MAC_STATS);

	addr_offs = hssigldrv_gts_make_csr_addr_offs(pdev, port,
						     HSSI_EMAC_HARDIP,
					eth_hardip_emac_csroffs(cntr_tx_config));
	tse_clear_bit(base, addr_offs, ETH_DEFREEZE_TX_MAC_STATS);

	return 0;
}

static void hssigldrv_gts_reset_port(struct platform_device *pdev, int port)
{
	struct hssiss_private *priv = platform_get_drvdata(pdev);
	void __iomem *base = priv->usrcsr;
	u32 addr_offs;

	addr_offs = hssigldrv_gts_make_usrcsr_addr_offs(pdev, port,
					eth_userspace_csroffs(control_reg));

	tse_clear_bit(base, addr_offs, (1 << port));
}

struct hssi_dev_ops device_ops_gdr = {
	.probe_init = hssigldrv_gts_probe_init,
	.get_addr_offset = hssigldrv_gts_make_csr_addr_offs,
	.get_ethport_status = hssigldrv_gts_get_ethport_status,
	.reset_mac_stat = hssigldrv_gts_reset_mac_stat,
	.read_mac_stat = hssigldrv_gts_read_mac_stats,
	.get_mtu = hssigldrv_gts_get_mtu,
	.set_mtu = hssigldrv_gts_set_mtu,
	.enable_loopback = hssigldrv_gts_en_loopback_mode,
	.disable_loopback = hssigldrv_gts_dis_loopback_mode,
	.freeze_mac_stats = hssigldrv_gts_freeze_stats,
	.defreeze_mac_stats = hssigldrv_gts_defreeze_stats,
	.reset_port = hssigldrv_gts_reset_port,
};

static void intel_fpga_gts_unregister(struct platform_device *pdev)
{
}

/* Common PTP probe function */
static int intel_fpga_gts_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	dev_set_drvdata(dev, (void *)&device_ops_gdr);

	return 0;
}

static const struct of_device_id intel_fpga_gts_ids[] = {
		{.compatible = "intel, gts",},
		{ }
};

MODULE_DEVICE_TABLE(of, intel_fpga_gts_ids);

static struct platform_driver intel_fpga_gts_driver = {
	.probe          = intel_fpga_gts_probe,
	.remove         = intel_fpga_gts_unregister,
	.suspend        = NULL,
	.resume         = NULL,
	.driver         = {
		.name = "gts-module",
		.owner  = THIS_MODULE,
		.of_match_table = intel_fpga_gts_ids,
	},
};

module_platform_driver(intel_fpga_gts_driver);

MODULE_DESCRIPTION("Altera FPGA GTS driver");
MODULE_AUTHOR("Altera Corporation");
MODULE_LICENSE("GPL");

