// SPDX-License-Identifier: GPL
/* Intel zarlink spi driver
 * Copyright (C) 2023 Intel Corporation. All rights reserved
 *
 * Contributors:
 *
 */

#include "intel_freq_control.h"
#include "intel_freq_control_zl30793.h"
#include "intel_freq_ctrl_zl30793_spi.h"
#include "intel_freq_ctrl_common_spi.h"

static u8 zl30793_page_write(struct spi_device *spi, u16 *dma_safe_buf,
                             u16 addr)
{
	u8 ret;

	dma_safe_buf[0] = PLL_SPI_WRITE(PLL_SPI_PAGE_REG) | PLL_SPI_PAGE(addr);
	/* dev_info(&spi->dev,"%s: pll: page send : 0x%x\n",
	 * __func__, dma_safe_buf[0]);
	 */
	ret = spi_msg_transfer(spi, &dma_safe_buf[0], NULL);
		
	/* dev_info(&spi->dev,"%s: pll: page reply : 0x%x\n",
	 * __func__, dma_safe_buf[1]);
	 */
	return ret;
}

u8 zl30793_spi_write(struct spi_device *spi, u16 *dma_safe_buf,
                     u16 addr, u8 *val)
{
	u8 ret;
	
	if (!dma_safe_buf)
		return FREQ_CTRL_ERROR_FAIL;

	memset(dma_safe_buf, 0, PLL_SPI_MAX_FRAME_SIZE);
	
	ret = zl30793_page_write(spi,  dma_safe_buf, addr);
	if (ret)
		return FREQ_CTRL_ERROR_FAIL;

	dma_safe_buf[0] = PLL_SPI_WRITE(addr) | (*val);
	/* dev_info(&spi->dev, "%s: pll: send : 0x%x\n",
	 * __func__, dma_safe_buf[0]);
	 */
	ret = spi_msg_transfer(spi, &dma_safe_buf[0], NULL);

	return ret;
}

u8 zl30793_spi_read(struct spi_device *spi,  u16 *dma_safe_buf,
                    u16 addr, u8 *val)
{
	u8 ret;

	if (!dma_safe_buf)
		return FREQ_CTRL_ERROR_FAIL;

	memset(dma_safe_buf, 0, PLL_SPI_MAX_FRAME_SIZE);
	ret = zl30793_page_write(spi, dma_safe_buf, addr);
	if (ret)
		return FREQ_CTRL_ERROR_FAIL;

	dma_safe_buf[0] = PLL_SPI_READ(addr);
	/*dev_info(&spi->dev,"%s: pll: send : 0x%x\n",
	 *__func__,dma_safe_buf[0]);
	 */
	
	ret = spi_msg_transfer(spi, &dma_safe_buf[0], &dma_safe_buf[2]);

	*val = (dma_safe_buf[2] & 0xff);
	/*dev_info(&spi->dev,"%s: pll: reply val: 0x%x\n",
	 *__func__, *val);
	 */
	
	return ret;
}

/* wait for the df read semaphore before writing*/
static int zl30793_dpll_wait_for_df_ctrl_readsem(struct spi_device *spi)
{
	u16 *buf;
	u8 ctrl_val;
	int ret = 0;
	u8 attempts = 0;
	
	/*Short timeout when accessing registers that should return quickly*/
	u32 zl30793_short_timeout = 100;

	if (!spi)
		return FREQ_CTRL_ERROR_FAIL;
	
	/* Allocate DMA-safe buffer for transfers */
	buf = kmalloc(PLL_SPI_MAX_FRAME_SIZE, GFP_KERNEL);
	if (!buf)
		return FREQ_CTRL_ERROR_FAIL;

	do {
		// read : ZL30793_PAGE6_REG_DPLL_DF_CTRL_0
		ret = zl30793_spi_read(spi,
				buf,
				ZL30793_PAGE6_REG_DPLL_DF_CTRL_0,
				&ctrl_val);
		if (ret) {
			dev_err(&spi->dev, "SPI read error %d\n", ret);
			ret = FREQ_CTRL_ERROR_FAIL;
			goto zl_dpll_readsem_err;
		}

		if (ZL30793_DPLL_DF_CTRL_SEM_GET(ctrl_val) == 0) {
			/*dev_info(&spi->dev,"%s: pll: ZL30793_PAGE6_REG_DPLL_DF_CTRL_0[0x%x] "
			 *"read : 0x%x\n", __func__, ZL30793_PAGE6_REG_DPLL_DF_CTRL_0,
			 *ctrl_val);
			 */
			ret = FREQ_CTRL_ERROR_SUCCESS;
			goto zl_dpll_readsem_err;
		}

		/* dev_info(&spi->dev,"%s: pll: ZL30793_PAGE6_REG_DPLL_DF_CTRL_0[0x%x] "
		 * "read : 0x%x attempt %d\n", __func__,
		 * ZL30793_PAGE6_REG_DPLL_DF_CTRL_0, ctrl_val, attempts);
		 */

		/* Device is not ready, wait a few ms */
		udelay(ZL30793_REG_READ_INTERVAL);

		if (++attempts > (zl30793_short_timeout / ZL30793_REG_READ_INTERVAL)) {
			dev_info(&spi->dev, "%s timeout!\n", __func__);
			ret = FREQ_CTRL_ERROR_FAIL;//timeout
			goto zl_dpll_readsem_err;
		}

	} while (1);

zl_dpll_readsem_err:
	kfree(buf);

	return ret;
}

void intel_freq_control_zl30793(struct work_struct *work)
{
	struct freq_work *p_work;
	s64 scaled_ppm;
	short int i;
	u8 offset_val = 0;
	int ret;
	u16 *buf;
	u64 step_val = 0;
	struct intel_freq_control_private *priv;
	struct spi_device *spi_dev;

	p_work = container_of(work, struct freq_work, w);
	scaled_ppm = p_work->scaled_ppm;

	priv = container_of(p_work, struct intel_freq_control_private, queued_work);
	spi_dev = priv->fc_acc_type.spi_dev;

	if (!spi_dev)
		return;

	/* dpll_df_offset_2 contains a 2's complement binary value of delta
	 * frequency offset. The register controls delta frequency of synthesizers
	 * that are associated with DPLL2. Delta frequency is expressed in steps
	 * of +/- 2^-48 of nominal setting. The output frequency should be
	 * calculated as per formula: f_out = (1 - X/2^48)*f_nom
	 * where,
	 * X is 2's complement number specified in this register,
	 * f_nom is the nominal frequency set by Bs,
	 * Ks, Ms, Ns and postdivider number for particular Synthesizer and f_out
	 * is the desired output frequency
	 * Note 1:The delta frequency offset should not exceed +/-1% of the
	 *        nominal value (+/-0.4% for DPLL2 when GP-Synth is enabled).
	 * Note 2:This register can be written as fast as once per 600us
	 * Note 3:This register should not be written while a read operation is
	 *        pending
	 * Note 4:The read value of this register during NCO mode is the value
	 *        latched just prior to entering NCO mode.
	 *
	 * 1-(X/2^48)
	 * Z ppb:
	 * 1 - Z/10^9
	 * Z/10^9 = X/2^48
	 * => X = (Z*2^48)/10^9
	 * => X = ((scaled_ppm * (10^3/2^16)) * 2^48)/10^9
	 * => X = (scaled_ppm * (10^3/2^16) * 2^48) /10^9
	 * => X = ((scaled_ppm* 10^3 * 2^32) / 10^9
	 * => X =  scaled_ppm * 2^32 / 10^6 ==>  X=  scaled_ppm * 4,294.967
	 */

	/* Allocate DMA-safe buffer for transfers */
	buf = kmalloc(PLL_SPI_MAX_FRAME_SIZE, GFP_KERNEL);
	if (!buf)
		return;

	step_val = (((u64)(abs(scaled_ppm))) << 32) / 1000000;

	if (scaled_ppm > 0)
		step_val = 0 - step_val;

	/*printk("scaled_ppm = %ld abs(scaled_ppm) = %ld step_val = 0x%llx\n",
	 *scaled_ppm, abs(scaled_ppm), step_val);
	 */
	if (zl30793_dpll_wait_for_df_ctrl_readsem(spi_dev) == 0) {
		for (i = 0; i < 6; i++) {
			offset_val = step_val >> (40 - 8 * i);
			ret = zl30793_spi_write(spi_dev, buf,
						ZL30793_PAGE6_REG_DPLL_DF_OFFSET_0_0 + i,
						&offset_val);
			if (ret) {
				dev_err(&spi_dev->dev, "SPI write error %d\n", ret);
				goto zl_offset_err;
			}
		}
	}

zl_offset_err:
	kfree(buf);
}

/* zl30793_Dpll_NCO_ModeSet : Sets the current DPLL mode of operation */
static int zl30793_dpll_nco_modeset(struct spi_device *spi)
{
	u8 ctrl_val;
	int ret;
	u16 *buf;

	if (!spi)
		return FREQ_CTRL_ERROR_FAIL;

	/* Allocate DMA-safe buffer for transfers */
	buf = kmalloc(PLL_SPI_MAX_FRAME_SIZE, GFP_KERNEL);
	if (!buf)
		return FREQ_CTRL_ERROR_FAIL;

	memset(buf, 0, PLL_SPI_MAX_FRAME_SIZE);

	/* read : ZL30793_PAGE2_REG_DPLL_STATE_OFFSET_0 */
	ret = zl30793_spi_read(spi, buf, ZL30793_PAGE2_REG_DPLL_STATE_OFFSET_0,
			       &ctrl_val);
	if (ret) {
		dev_err(&spi->dev, "SPI select error %d\n", ret);
		goto zl_ctrl_err;
	}

	dev_info(&spi->dev,
		"pll: ZL30793_PAGE2_REG_DPLL_STATE_OFFSET_0[0x%x] read : 0x%x\n",
		 ZL30793_PAGE2_REG_DPLL_STATE_OFFSET_0,
		 ctrl_val);

	if (!(ctrl_val & 0x3)) {
		dev_info(&spi->dev,
			"ZL30793 is already in NCO mode (0x%x)\n", ctrl_val);

		ret = FREQ_CTRL_ERROR_SUCCESS;
		goto zl_ctrl_err;
	}

	dev_info(&spi->dev, "ZL30793 is not in NCO mode(0x%x); set Dpll in NCO mode\n",
		 ctrl_val);

	/* read : ZL30793_PAGE4_REG_DPLL_CTRL_0 */
	ret = zl30793_spi_read(spi,
			buf,
			ZL30793_PAGE4_REG_DPLL_CTRL_0,
			&ctrl_val);
	if (ret) {
		dev_err(&spi->dev, "SPI select error %d\n", ret);
		goto zl_ctrl_err;
	}

	dev_info(&spi->dev,
		"pll: ZL30793_PAGE4_REG_DPLL_CTRL_0[0x%x] read: 0x%x\n",
		ZL30793_PAGE4_REG_DPLL_CTRL_0, ctrl_val);

	/* write : ZL30793_PAGE4_REG_DPLL_CTRL_0 */
	ctrl_val |= (1 << 3);
	dev_info(&spi->dev, "pll: ZL30793_PAGE4_REG_DPLL_CTRL_0[0x%x] write: 0x%x\n",
		ZL30793_PAGE4_REG_DPLL_CTRL_0, ctrl_val);

	ret = zl30793_spi_write(spi, buf, ZL30793_PAGE4_REG_DPLL_CTRL_0,
				&ctrl_val);
	if (ret) {
		dev_err(&spi->dev, "SPI select error %d\n", ret);
		goto zl_ctrl_err;
	}

	/* read : ZL30793_PAGE4_REG_DPLL_CTRL_0 */
	ret = zl30793_spi_read(spi, buf, ZL30793_PAGE4_REG_DPLL_CTRL_0,
			       &ctrl_val);
	if (ret) {
		dev_err(&spi->dev, "SPI select error %d\n", ret);
		goto zl_ctrl_err;
	}

	dev_info(&spi->dev, "pll: ZL30793_PAGE4_REG_DPLL_CTRL_0[0x%x] read: 0x%x\n",
		 ZL30793_PAGE4_REG_DPLL_CTRL_0, ctrl_val);

	/* read : ZL30793_PAGE4_REG_DPLL_MODE_REFSEL_0 */
	ret = zl30793_spi_read(spi, buf, ZL30793_PAGE4_REG_DPLL_MODE_REFSEL_0,
			       &ctrl_val);
	if (ret) {
		dev_err(&spi->dev, "SPI select error %d\n", ret);
		goto zl_ctrl_err;
	}

	dev_info(&spi->dev, "pll: ZL30793_PAGE4_REG_DPLL_MODE_REFSEL_0[0x%x] read: 0x%x\n",
		ZL30793_PAGE4_REG_DPLL_MODE_REFSEL_0, ctrl_val);

	/* write : ZL30793_PAGE4_REG_DPLL_MODE_REFSEL_0 */
	ctrl_val = ((ctrl_val & 0xf8) | 0x04);

	dev_info(&spi->dev, "pll: ZL30793_PAGE4_REG_DPLL_MODE_REFSEL_0[0x%x] write: 0x%x\n",
		 ZL30793_PAGE4_REG_DPLL_MODE_REFSEL_0, ctrl_val);

	ret = zl30793_spi_write(spi, buf, ZL30793_PAGE4_REG_DPLL_MODE_REFSEL_0,
				&ctrl_val);
	if (ret) {
		dev_err(&spi->dev, "SPI select error %d\n", ret);
		goto zl_ctrl_err;
	}

	/* read : ZL30793_PAGE4_REG_DPLL_MODE_REFSEL_0 */
	ret = zl30793_spi_read(spi, buf, ZL30793_PAGE4_REG_DPLL_MODE_REFSEL_0,
			       &ctrl_val);
	if (ret) {
		dev_err(&spi->dev, "SPI select error %d\n", ret);
		goto zl_ctrl_err;
	}

	dev_info(&spi->dev, "pll: ZL30793_PAGE4_REG_DPLL_MODE_REFSEL_0[0x%x] read: 0x%x\n",
		 ZL30793_PAGE4_REG_DPLL_MODE_REFSEL_0, ctrl_val);

	// read : ZL30793_PAGE2_REG_DPLL_STATE_OFFSET_0
	ret = zl30793_spi_read(spi, buf, ZL30793_PAGE2_REG_DPLL_STATE_OFFSET_0,
			       &ctrl_val);
	if (ret) {
		dev_err(&spi->dev, "SPI select error %d\n", ret);
		goto zl_ctrl_err;
	}

	dev_info(&spi->dev, "pll: ZL30793_PAGE2_REG_DPLL_STATE_OFFSET_0[0x%x] read : 0x%x\n",
		 ZL30793_PAGE2_REG_DPLL_STATE_OFFSET_0, ctrl_val);

	if (!(ctrl_val & 0x3)) {
		dev_info(&spi->dev, "ZL30793 is in NCO mode (0x%x)\n", ctrl_val);
		ret = FREQ_CTRL_ERROR_SUCCESS;

	} else {
		dev_info(&spi->dev, "failed to set ZL30793 in NCO mode (0x%x)\n",
			 ctrl_val);

		ret = FREQ_CTRL_ERROR_FAIL;
	}

zl_ctrl_err:
	kfree(buf);
	return ret;
}

int spi_dev_check_zl30793_clock(struct spi_device *spi, 
		struct intel_freq_control_private *priv)
{
        int ret;
        u16 *buf;
        u8 rx_data[2];
        u16 exp_reply = 0x0ED1; // 0x0ED1: ZL30793

        if (!spi)
                return FREQ_CTRL_ERROR_FAIL;

        spi->mode = 0;
        spi->max_speed_hz = 10000000;
        spi->bits_per_word = 16;
        ret = spi_setup(spi);

        if (ret) {
                dev_err(&spi->dev, "spi_setup(%s) failed\n",
                        dev_name(&spi->dev));
                
		return FREQ_CTRL_ERROR_FAIL;
        }

        /* Allocate DMA-safe buffer for transfers */
        buf = kmalloc(PLL_SPI_MAX_FRAME_SIZE, GFP_KERNEL);
        if (!buf)
                return FREQ_CTRL_ERROR_FAIL;

        memset(buf, 0, PLL_SPI_MAX_FRAME_SIZE);

        /* Device  id*/
        ret = zl30793_spi_read(spi, buf, ZL30793_PAGE0_REG_GENERAL_ID_0,
                               &rx_data[0]);
        if (ret) {
                
		dev_err(&spi->dev, "SPI read error %d\n", ret);
                ret = FREQ_CTRL_ERROR_FAIL;
                
		goto zl_spi_dev_check_err;
        }

	ret = zl30793_spi_read(spi, buf, ZL30793_PAGE0_REG_GENERAL_ID_0 + 1,
			       &rx_data[1]);
	if (ret) {
		dev_err(&spi->dev, "SPI read error %d\n", ret);
		ret = FREQ_CTRL_ERROR_FAIL;
		goto zl_spi_dev_check_err;
	}

        /* expected reply for ZL30793 is 0x0ED1 */
        if (((rx_data[0] << 8) | rx_data[1]) != exp_reply) {

                dev_info(&spi->dev,
			"0x0ED1 : ZL30793 chip identification number does not match! reply: %04x\n",
                        ((rx_data[0] << 8) | rx_data[1]));

                ret = FREQ_CTRL_ERROR_FAIL;

                goto zl_spi_dev_check_err;
        
	} else {
                dev_info(&spi->dev, "0x0ED1 : ZL30793 chip identification number match! reply: %04x\n",
                         ((rx_data[0] << 8) | rx_data[1]));
        }

        zl30793_sysfs_configure(spi);   //append to spi sysfs

        ret = zl30793_dpll_nco_modeset(spi);
        if (ret)
                goto zl_spi_dev_check_err;

        priv->fc_acc_type.spi_dev = spi;
        
	ret = FREQ_CTRL_ERROR_SUCCESS;

zl_spi_dev_check_err:
        kfree(buf);
        
	return ret;
}