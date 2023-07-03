// SPDX-License-Identifier: GPL
/* Intel SI Spi driver
 * Copyright (C) 2023 Intel Corporation. All rights reserved
 *
 * Contributors:
 *
 */

#include "intel_freq_control.h"
#include "intel_freq_ctrl_si5518_spi.h"
#include "intel_freq_ctrl_common_spi.h"

void intel_freq_control_si5518(struct work_struct *work)
{
        u8 *buf;
        /* VARIABLE_OFFSET_DCO, MA divider select */
        u8 cmd[7] = {0xc0, 0x24, 0x04, 0, 0, 0, 0};
        long scaled_ppm;
        u32 num_steps;
        struct freq_work *p_work;
	struct spi_device *spi;
	struct intel_freq_control_private *priv;
        u32 si5518_ma_step_size;
        int ret;

        /* Allocate DMA-safe buffer for transfers */
        buf = kmalloc(16, GFP_KERNEL);
        if (!buf)
                return;

        p_work = container_of(work, struct freq_work, w);
        scaled_ppm = p_work->scaled_ppm;
        
        priv = container_of(p_work, struct intel_freq_control_private, queued_work);
	spi = priv->fc_acc_type.spi_dev;
	si5518_ma_step_size = priv->step_size;

	

        if (abs(scaled_ppm) > (10 << 16))
                dev_warn(&spi->dev, "%s: scaled_ppm:%li (ppm:%li) outside range of +/-10ppm\n",
                         __func__, scaled_ppm, scaled_ppm / (1 << 16));

        /* unit of si5518_ma_step_size is ppt, 1ppm = 1000000ppt
         * scaled_ppm is ppm with 16 fractional bits
         */
        num_steps =
        ((abs(scaled_ppm) << 1) * 1000000) / (si5518_ma_step_size << 16);

        num_steps = (num_steps >> 1) + (num_steps & 1);

        if (scaled_ppm < 0)
                num_steps = 0 - num_steps;

        /* dev_info(&spi->dev, "%s: scaled_ppm:%li (ppm:%li) "
         * "si5518_ma_step_size:%u num_steps:%i\n",__func__, scaled_ppm,
         * scaled_ppm /(1 << 16), si5518_ma_step_size,(int32_t)num_steps);
         */

        cmd[3] = (num_steps >> 0)  & 0xff;
        cmd[4] = (num_steps >> 8)  & 0xff;
        cmd[5] = (num_steps >> 16) & 0xff;
        cmd[6] = (num_steps >> 24) & 0xff;
        memcpy(buf, cmd, 7);

	ret = spi_msg_transfer(spi, &buf[0], &buf[8]);

        if (ret < 0)
                dev_err(&spi->dev, "SPI read error %d\n", ret);

        udelay(500);

        buf[0] = 0xd0;
        buf[1] = 0;
        buf[2] = 0;
        buf[3] = 0;

	ret = spi_msg_transfer(spi, &buf[0], &buf[4]);
        if (ret < 0)
                dev_err(&spi->dev, "SPI read error %d\n", ret);

        if ((buf[5] >> 4) != 8 || buf[6] & 1) {
                dev_err(&spi->dev, "%s: spi_sync(0x24) reply: 0x%02x 0x%02x error\n",
                        __func__, buf[5], buf[6]);
        }
        kfree(buf);
}

static void si5518_dco_centering(struct spi_device *spi)
{
        u8 *buf;
        /* VARIABLE_OFFSET_DCO, MA divider select*/
        u8 cmd[7] = {0xc0, 0x24, 0x04, 0, 0, 0, 0};
        int ret;

        // Allocate DMA-safe buffer for transfers
        buf = kmalloc(16, GFP_KERNEL);
        if (!buf) {
                dev_err(&spi->dev, "%s: Memory allocation failure\n", __func__);
                return;
        }
        memset(buf, 0, 16);
        memcpy(buf, cmd, 7);
	
	ret = spi_msg_transfer(spi, &buf[0], &buf[8]);
        if (ret < 0)
                dev_err(&spi->dev, "SPI read error %d\n", ret);

        udelay(500);

        buf[0] = 0xd0;
        buf[1] = 0;
        buf[2] = 0;
        buf[3] = 0;

	ret = spi_msg_transfer(spi, &buf[0], &buf[4]);
        if (ret < 0)
                dev_err(&spi->dev, "SPI read error %d\n", ret);

        dev_info(&spi->dev, "%s: spi_sync(0x24) reply: 0x%02x 0x%02x 0x%02x 0x%02x\n",
                        __func__, buf[4], buf[5], buf[6], buf[7]);
        kfree(buf);
}

// for Si5518 using SPI
/* root@agilex:~# ./spidev_test -D /dev/spidev1.0 -p  \
 * "\xc0\x08\x00\x00\x00\x00\x00" -v -O -H -s 8000000
 */
// spi mode: 0x3
// bits per word: 8
// max speed: 8000000 Hz (8000 kHz)
// TX | C0 08 00 00 00 00 00 __ __ __ __ __ __ __ __ __ __ __ __  |.......|
// RX | 00 00 00 00 00 00 00 __ __ __ __ __ __ __ __ __ __ __ __  |.......|
/* root@agilex:~# ./spidev_test -D /dev/spidev1.0 -p \
 * "\xd0\x00\x00\x00\x00\x00\x00" -v -O -H -s 8000000
 */
// spi mode: 0x3
// bits per word: 8
// max speed: 8000000 Hz (8000 kHz)
// TX | D0 00 00 00 00 00 00 __ __ __ __ __ __ __ __ __ __ __ __  |.......|
// RX | 00 80 18 55 41 42 7F __ __ __ __ __ __ __ __ __ __ __ __  |...UAB.|
int spi_dev_check_si5518_clock(struct spi_device *spi,
       	struct intel_freq_control_private *priv)
{
        u8 exp_reply[4] = {0x00, 0x80, 0x18, 0x55}; // 0x5518 = Si5518
	
	u32 si5518_ma_step_size;
        u8 *buf, pll_sel, loop;
        int ret;

        // dev_info(&spi->dev, "%s: spi->modalias:%s\n", __func__, spi->modalias);
        if (spi && strncmp(spi->modalias, "spidev", 6) == 0) {
                spi->mode = 3;
                spi->max_speed_hz = 8000000;
                ret = spi_setup(spi);
                if (ret) {
                        dev_err(&spi->dev, "%s: spi_setup(%s) failed\n",
                                __func__, dev_name(&spi->dev));
                        return 0;
                }

                /* Allocate DMA-safe buffer for transfers */
                buf = kmalloc(64, GFP_KERNEL);
                if (!buf)
                        return 0;

                // DEVICE_INFO:
                buf[0] = 0xc0;
                buf[1] = 0x08;

		ret = spi_msg_transfer(spi, &buf[0], &buf[1]);
                if (ret < 0)
                        dev_err(&spi->dev, "SPI read error %d\n", ret);

                udelay(500);
		
		ret = spi_msg_transfer(spi, &buf[0], &buf[4]);
                if (ret < 0) {
                        dev_err(&spi->dev, "SPI read error %d\n", ret);
                } else {
                        dev_info(&spi->dev, "%s: DEVICE_INFO reply: %02x %02x %02x %02x\n",
                                 __func__, buf[4], buf[5], buf[6], buf[7]);
                }

                /* Ignore the 1st byte comparison because it is SI5518 grade specific
                 * exp_reply on SI5518A (base carrier) board is {0x00, 0x80, 0x18, 0x55}
                 * exp_reply on SI5518B (MXL board) is {0xFF, 0x80, 0x18, 0x55}
                 */
                if (memcmp(&buf[5], &exp_reply[1], 3) != 0) {
                        kfree(buf);
                        return 0;
                }

                // METADATA:
                buf[0] = 0xc0;
                buf[1] = 0x15;
		
		ret = spi_msg_transfer(spi, &buf[0], &buf[1]);
                if (ret < 0)
                        dev_err(&spi->dev, "SPI read error %d\n", ret);

                udelay(500);
                buf[0] = 0xd0;
                memset(&buf[1], 0, 16);
		
		ret = spi_msg_transfer(spi, &buf[0], &buf[16]);
                if (ret < 0) {
                        dev_err(&spi->dev, "SPI read error %d\n", ret);
                } else {
                        dev_info(&spi->dev, "%s: METADATA reply: %02x %02x %02x %02x\n",
                                 __func__, buf[16], buf[17], buf[18], buf[19]);
                }
                if (buf[17] != 0x80) {
                        dev_err(&spi->dev, "%s: ERROR - METADATA reply not ready\n", __func__);
                        kfree(buf);
                        return 0;
                }
                si5518_ma_step_size = (buf[9 + 17] | (buf[10 + 17] << 8) |
                                      (buf[11 + 17] << 16) | (buf[12 + 17] << 24));

		priv->step_size = si5518_ma_step_size;

                dev_info(&spi->dev, "%s: Si5518 found, read ma_step_size=%u\n",
                         __func__, si5518_ma_step_size);
                priv->fc_acc_type.spi_dev = spi;

                // check PLL_STATUS (first loop clears sticky bits):
                exp_reply[2] = 0;
                exp_reply[3] = 0;
                for (loop = 0; loop < 2; ++loop) {
                        for (pll_sel = 1; pll_sel < 8; pll_sel = pll_sel << 1) {
                                buf[0] = 0xc0;
                                buf[1] = 0x13;
                                buf[2] = pll_sel;

				ret = spi_msg_transfer(spi, &buf[0], &buf[4]);
                                if (ret < 0)
                                        dev_err(&spi->dev, "SPI read error %d\n", ret);

                                udelay(500);
                                buf[0] = 0xd0;
                                memset(&buf[1], 0, 16);
				
				ret = spi_msg_transfer(spi, &buf[0], &buf[16]);
                                if (ret < 0) {
                                        dev_err(&spi->dev, "SPI read error %d\n", ret);
                                } else {
                                        if (loop && memcmp(&buf[17], &exp_reply[1], 3) != 0) {
                                                /* Ignore the 1st byte comparison
                                                 * because it is SI5518 grade specific
                                                 * exp_repl on SI5518A base carrier
                                                 * board is {0x00, 0x80, 0x18, 0x55}
                                                 * on SI5518B MXL board is
                                                 * {0xFF, 0x80, 0x18, 0x55}
                                                 */
                                                dev_err(&spi->dev, "%s: PLL_STATUS(%u) reply: %02x %02x %02x %02x - FAIL\n",
                                                        __func__, pll_sel, buf[16], buf[17],
                                                        buf[18], buf[19]);
                                                dev_err(&spi->dev, "%s: ERROR - PLL_STATUS indicates out of lock\n",
                                                        __func__);
                                        } else if (loop) {
                                                dev_info(&spi->dev, "%s: PLL_STATUS(%u) reply: %02x %02x %02x %02x - OK\n",
                                                         __func__, pll_sel, buf[16], buf[17],
                                                         buf[18], buf[19]);
                                        }
                                }
                        }
                }
                kfree(buf);
                /* centering dco by setting step value to zero */
                si5518_dco_centering(spi);
                return 1; // found
        }
        return 0;
}