/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>

#define LPASS_CDC_CLK_RX_RESET_CTL		(0x000)
#define LPASS_CDC_CLK_TX_RESET_B1_CTL		(0x004)
#define CLK_RX_RESET_B1_CTL_TX1_RESET_MASK	BIT(0)
#define CLK_RX_RESET_B1_CTL_TX2_RESET_MASK	BIT(1)
#define LPASS_CDC_CLK_DMIC_B1_CTL		(0x008)
#define DMIC_B1_CTL_DMIC0_CLK_SEL_MASK		GENMASK(3, 1)
#define DMIC_B1_CTL_DMIC0_CLK_SEL_DIV2		(0x0 << 1)
#define DMIC_B1_CTL_DMIC0_CLK_SEL_DIV3		(0x1 << 1)
#define DMIC_B1_CTL_DMIC0_CLK_SEL_DIV4		(0x2 << 1)
#define DMIC_B1_CTL_DMIC0_CLK_SEL_DIV6		(0x3 << 1)
#define DMIC_B1_CTL_DMIC0_CLK_SEL_DIV16		(0x4 << 1)
#define DMIC_B1_CTL_DMIC0_CLK_EN_MASK		BIT(0)
#define DMIC_B1_CTL_DMIC0_CLK_EN_ENABLE		BIT(0)

#define LPASS_CDC_CLK_RX_I2S_CTL		(0x00C)
#define RX_I2S_CTL_RX_I2S_MODE_MASK		BIT(5)
#define RX_I2S_CTL_RX_I2S_MODE_16		BIT(5)
#define RX_I2S_CTL_RX_I2S_MODE_32		0
#define RX_I2S_CTL_RX_I2S_FS_RATE_MASK		GENMASK(2, 0)
#define RX_I2S_CTL_RX_I2S_FS_RATE_F_8_KHZ	0x0
#define RX_I2S_CTL_RX_I2S_FS_RATE_F_16_KHZ	0x1
#define RX_I2S_CTL_RX_I2S_FS_RATE_F_32_KHZ	0x2
#define RX_I2S_CTL_RX_I2S_FS_RATE_F_48_KHZ	0x3
#define RX_I2S_CTL_RX_I2S_FS_RATE_F_96_KHZ	0x4
#define RX_I2S_CTL_RX_I2S_FS_RATE_F_192_KHZ	0x5
#define LPASS_CDC_CLK_TX_I2S_CTL		(0x010)
#define TX_I2S_CTL_TX_I2S_MODE_MASK		BIT(5)
#define TX_I2S_CTL_TX_I2S_MODE_16		BIT(5)
#define TX_I2S_CTL_TX_I2S_MODE_32		0
#define TX_I2S_CTL_TX_I2S_FS_RATE_MASK		GENMASK(2, 0)
#define TX_I2S_CTL_TX_I2S_FS_RATE_F_8_KHZ	0x0
#define TX_I2S_CTL_TX_I2S_FS_RATE_F_16_KHZ	0x1
#define TX_I2S_CTL_TX_I2S_FS_RATE_F_32_KHZ	0x2
#define TX_I2S_CTL_TX_I2S_FS_RATE_F_48_KHZ	0x3
#define TX_I2S_CTL_TX_I2S_FS_RATE_F_96_KHZ	0x4
#define TX_I2S_CTL_TX_I2S_FS_RATE_F_192_KHZ	0x5

#define LPASS_CDC_CLK_OTHR_RESET_B1_CTL		(0x014)
#define LPASS_CDC_CLK_TX_CLK_EN_B1_CTL		(0x018)
#define LPASS_CDC_CLK_OTHR_CTL			(0x01C)
#define LPASS_CDC_CLK_RX_B1_CTL			(0x020)
#define LPASS_CDC_CLK_MCLK_CTL			(0x024)
#define MCLK_CTL_MCLK_EN_MASK			BIT(0)
#define MCLK_CTL_MCLK_EN_ENABLE			BIT(0)
#define MCLK_CTL_MCLK_EN_DISABLE		0
#define LPASS_CDC_CLK_PDM_CTL			(0x028)
#define LPASS_CDC_CLK_PDM_CTL_PDM_EN_MASK	BIT(0)
#define LPASS_CDC_CLK_PDM_CTL_PDM_EN		BIT(0)
#define LPASS_CDC_CLK_PDM_CTL_PDM_CLK_SEL_MASK	BIT(1)
#define LPASS_CDC_CLK_PDM_CTL_PDM_CLK_SEL_FB	BIT(1)
#define LPASS_CDC_CLK_PDM_CTL_PDM_CLK_PDM_CLK	0

#define LPASS_CDC_CLK_SD_CTL			(0x02C)
#define LPASS_CDC_RX1_B1_CTL			(0x040)
#define LPASS_CDC_RX2_B1_CTL			(0x060)
#define LPASS_CDC_RX3_B1_CTL			(0x080)
#define LPASS_CDC_RX1_B2_CTL			(0x044)
#define LPASS_CDC_RX2_B2_CTL			(0x064)
#define LPASS_CDC_RX3_B2_CTL			(0x084)
#define LPASS_CDC_RX1_B3_CTL			(0x048)
#define LPASS_CDC_RX2_B3_CTL			(0x068)
#define LPASS_CDC_RX3_B3_CTL			(0x088)
#define LPASS_CDC_RX1_B4_CTL			(0x04C)
#define LPASS_CDC_RX2_B4_CTL			(0x06C)
#define LPASS_CDC_RX3_B4_CTL			(0x08C)
#define LPASS_CDC_RX1_B5_CTL			(0x050)
#define LPASS_CDC_RX2_B5_CTL			(0x070)
#define LPASS_CDC_RX3_B5_CTL			(0x090)
#define LPASS_CDC_RX1_B6_CTL			(0x054)
#define RXn_B6_CTL_MUTE_MASK			BIT(0)
#define RXn_B6_CTL_MUTE_ENABLE			BIT(0)
#define RXn_B6_CTL_MUTE_DISABLE			0
#define LPASS_CDC_RX2_B6_CTL			(0x074)
#define LPASS_CDC_RX3_B6_CTL			(0x094)
#define LPASS_CDC_RX1_VOL_CTL_B1_CTL		(0x058)
#define LPASS_CDC_RX2_VOL_CTL_B1_CTL		(0x078)
#define LPASS_CDC_RX3_VOL_CTL_B1_CTL		(0x098)
#define LPASS_CDC_RX1_VOL_CTL_B2_CTL		(0x05C)
#define LPASS_CDC_RX2_VOL_CTL_B2_CTL		(0x07C)
#define LPASS_CDC_RX3_VOL_CTL_B2_CTL		(0x09C)
#define LPASS_CDC_TOP_GAIN_UPDATE		(0x0A0)
#define LPASS_CDC_TOP_CTL			(0x0A4)
#define TOP_CTL_DIG_MCLK_FREQ_MASK		BIT(0)
#define TOP_CTL_DIG_MCLK_FREQ_F_12_288MHZ	0
#define TOP_CTL_DIG_MCLK_FREQ_F_9_6MHZ		BIT(0)

#define LPASS_CDC_DEBUG_DESER1_CTL		(0x0E0)
#define LPASS_CDC_DEBUG_DESER2_CTL		(0x0E4)
#define LPASS_CDC_DEBUG_B1_CTL_CFG		(0x0E8)
#define LPASS_CDC_DEBUG_B2_CTL_CFG		(0x0EC)
#define LPASS_CDC_DEBUG_B3_CTL_CFG		(0x0F0)
#define LPASS_CDC_IIR1_GAIN_B1_CTL		(0x100)
#define LPASS_CDC_IIR2_GAIN_B1_CTL		(0x140)
#define LPASS_CDC_IIR1_GAIN_B2_CTL		(0x104)
#define LPASS_CDC_IIR2_GAIN_B2_CTL		(0x144)
#define LPASS_CDC_IIR1_GAIN_B3_CTL		(0x108)
#define LPASS_CDC_IIR2_GAIN_B3_CTL		(0x148)
#define LPASS_CDC_IIR1_GAIN_B4_CTL		(0x10C)
#define LPASS_CDC_IIR2_GAIN_B4_CTL		(0x14C)
#define LPASS_CDC_IIR1_GAIN_B5_CTL		(0x110)
#define LPASS_CDC_IIR2_GAIN_B5_CTL		(0x150)
#define LPASS_CDC_IIR1_GAIN_B6_CTL		(0x114)
#define LPASS_CDC_IIR2_GAIN_B6_CTL		(0x154)
#define LPASS_CDC_IIR1_GAIN_B7_CTL		(0x118)
#define LPASS_CDC_IIR2_GAIN_B7_CTL		(0x158)
#define LPASS_CDC_IIR1_GAIN_B8_CTL		(0x11C)
#define LPASS_CDC_IIR2_GAIN_B8_CTL		(0x15C)
#define LPASS_CDC_IIR1_CTL			(0x120)
#define LPASS_CDC_IIR2_CTL			(0x160)
#define LPASS_CDC_IIR1_GAIN_TIMER_CTL		(0x124)
#define LPASS_CDC_IIR2_GAIN_TIMER_CTL		(0x164)
#define LPASS_CDC_IIR1_COEF_B1_CTL		(0x128)
#define LPASS_CDC_IIR2_COEF_B1_CTL		(0x168)
#define LPASS_CDC_IIR1_COEF_B2_CTL		(0x12C)
#define LPASS_CDC_IIR2_COEF_B2_CTL		(0x16C)
#define LPASS_CDC_CONN_RX1_B1_CTL		(0x180)
#define LPASS_CDC_CONN_RX1_B2_CTL		(0x184)
#define LPASS_CDC_CONN_RX1_B3_CTL		(0x188)
#define LPASS_CDC_CONN_RX2_B1_CTL		(0x18C)
#define LPASS_CDC_CONN_RX2_B2_CTL		(0x190)
#define LPASS_CDC_CONN_RX2_B3_CTL		(0x194)
#define LPASS_CDC_CONN_RX3_B1_CTL		(0x198)
#define LPASS_CDC_CONN_RX3_B2_CTL		(0x19C)
#define LPASS_CDC_CONN_TX_B1_CTL		(0x1A0)
#define LPASS_CDC_CONN_EQ1_B1_CTL		(0x1A8)
#define LPASS_CDC_CONN_EQ1_B2_CTL		(0x1AC)
#define LPASS_CDC_CONN_EQ1_B3_CTL		(0x1B0)
#define LPASS_CDC_CONN_EQ1_B4_CTL		(0x1B4)
#define LPASS_CDC_CONN_EQ2_B1_CTL		(0x1B8)
#define LPASS_CDC_CONN_EQ2_B2_CTL		(0x1BC)
#define LPASS_CDC_CONN_EQ2_B3_CTL		(0x1C0)
#define LPASS_CDC_CONN_EQ2_B4_CTL		(0x1C4)
#define LPASS_CDC_CONN_TX_I2S_SD1_CTL		(0x1C8)
#define LPASS_CDC_TX1_VOL_CTL_TIMER		(0x280)
#define LPASS_CDC_TX2_VOL_CTL_TIMER		(0x2A0)
#define LPASS_CDC_TX1_VOL_CTL_GAIN		(0x284)
#define LPASS_CDC_TX2_VOL_CTL_GAIN		(0x2A4)
#define LPASS_CDC_TX1_VOL_CTL_CFG		(0x288)
#define TX_VOL_CTL_CFG_MUTE_EN_MASK		BIT(0)
#define TX_VOL_CTL_CFG_MUTE_EN_ENABLE		BIT(0)

#define LPASS_CDC_TX2_VOL_CTL_CFG		(0x2A8)
#define LPASS_CDC_TX1_MUX_CTL			(0x28C)
#define TX_MUX_CTL_CUT_OFF_FREQ_MASK		GENMASK(5, 4)
#define TX_MUX_CTL_CUT_OFF_FREQ_SHIFT		4
#define TX_MUX_CTL_CF_NEG_3DB_4HZ		(0x0 << 4)
#define TX_MUX_CTL_CF_NEG_3DB_75HZ		(0x1 << 4)
#define TX_MUX_CTL_CF_NEG_3DB_150HZ		(0x2 << 4)
#define TX_MUX_CTL_HPF_BP_SEL_MASK		BIT(3)
#define TX_MUX_CTL_HPF_BP_SEL_BYPASS		BIT(3)
#define TX_MUX_CTL_HPF_BP_SEL_NO_BYPASS		0

#define LPASS_CDC_TX2_MUX_CTL			(0x2AC)
#define LPASS_CDC_TX1_CLK_FS_CTL		(0x290)
#define LPASS_CDC_TX2_CLK_FS_CTL		(0x2B0)
#define LPASS_CDC_TX1_DMIC_CTL			(0x294)
#define LPASS_CDC_TX2_DMIC_CTL			(0x2B4)
#define TXN_DMIC_CTL_CLK_SEL_MASK		GENMASK(2, 0)
#define TXN_DMIC_CTL_CLK_SEL_DIV2		0x0
#define TXN_DMIC_CTL_CLK_SEL_DIV3		0x1
#define TXN_DMIC_CTL_CLK_SEL_DIV4		0x2
#define TXN_DMIC_CTL_CLK_SEL_DIV6		0x3
#define TXN_DMIC_CTL_CLK_SEL_DIV16		0x4

#define MSM8916_WCD_DIGITAL_RATES (SNDRV_PCM_RATE_8000 | \
				   SNDRV_PCM_RATE_16000 | \
				   SNDRV_PCM_RATE_32000 | \
				   SNDRV_PCM_RATE_48000)
#define MSM8916_WCD_DIGITAL_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
				     SNDRV_PCM_FMTBIT_S32_LE)

struct msm8916_wcd_digital_priv {
	struct clk *ahbclk, *mclk;
};

static const unsigned long rx_gain_reg[] = {
	LPASS_CDC_RX1_VOL_CTL_B2_CTL,
	LPASS_CDC_RX2_VOL_CTL_B2_CTL,
	LPASS_CDC_RX3_VOL_CTL_B2_CTL,
};

static const unsigned long tx_gain_reg[] = {
	LPASS_CDC_TX1_VOL_CTL_GAIN,
	LPASS_CDC_TX2_VOL_CTL_GAIN,
};

static const char *const rx_mix1_text[] = {
	"ZERO", "IIR1", "IIR2", "RX1", "RX2", "RX3"
};

static const char *const dec_mux_text[] = {
	"ZERO", "ADC1", "ADC2", "ADC3", "DMIC1", "DMIC2"
};

static const char *const cic_mux_text[] = { "AMIC", "DMIC" };
static const char *const rx_mix2_text[] = { "ZERO", "IIR1", "IIR2" };
static const char *const adc2_mux_text[] = { "ZERO", "INP2", "INP3" };

/* RX1 MIX1 */
static const struct soc_enum rx_mix1_inp_enum[] = {
	SOC_ENUM_SINGLE(LPASS_CDC_CONN_RX1_B1_CTL, 0, 6, rx_mix1_text),
	SOC_ENUM_SINGLE(LPASS_CDC_CONN_RX1_B1_CTL, 3, 6, rx_mix1_text),
	SOC_ENUM_SINGLE(LPASS_CDC_CONN_RX1_B2_CTL, 0, 6, rx_mix1_text),
};

/* RX1 MIX2 */
static const struct soc_enum rx_mix2_inp1_chain_enum = SOC_ENUM_SINGLE(
				LPASS_CDC_CONN_RX1_B3_CTL, 0, 3, rx_mix2_text);

/* RX2 MIX1 */
static const struct soc_enum rx2_mix1_inp_enum[] = {
	SOC_ENUM_SINGLE(LPASS_CDC_CONN_RX2_B1_CTL, 0, 6, rx_mix1_text),
	SOC_ENUM_SINGLE(LPASS_CDC_CONN_RX2_B1_CTL, 3, 6, rx_mix1_text),
	SOC_ENUM_SINGLE(LPASS_CDC_CONN_RX2_B2_CTL, 0, 6, rx_mix1_text),
};

/* RX2 MIX2 */
static const struct soc_enum rx2_mix2_inp1_chain_enum = SOC_ENUM_SINGLE(
				LPASS_CDC_CONN_RX2_B3_CTL, 0, 3, rx_mix2_text);

/* RX3 MIX1 */
static const struct soc_enum rx3_mix1_inp_enum[] = {
	SOC_ENUM_SINGLE(LPASS_CDC_CONN_RX3_B1_CTL, 0, 6, rx_mix1_text),
	SOC_ENUM_SINGLE(LPASS_CDC_CONN_RX3_B1_CTL, 3, 6, rx_mix1_text),
	SOC_ENUM_SINGLE(LPASS_CDC_CONN_RX3_B2_CTL, 0, 6, rx_mix1_text),
};

/* DEC */
static const struct soc_enum dec1_mux_enum = SOC_ENUM_SINGLE(
				LPASS_CDC_CONN_TX_B1_CTL, 0, 6, dec_mux_text);
static const struct soc_enum dec2_mux_enum = SOC_ENUM_SINGLE(
				LPASS_CDC_CONN_TX_B1_CTL, 3, 6, dec_mux_text);

/* CIC */
static const struct soc_enum cic1_mux_enum = SOC_ENUM_SINGLE(
				LPASS_CDC_TX1_MUX_CTL, 0, 2, cic_mux_text);
static const struct soc_enum cic2_mux_enum = SOC_ENUM_SINGLE(
				LPASS_CDC_TX2_MUX_CTL, 0, 2, cic_mux_text);

/* RDAC2 MUX */
static const struct snd_kcontrol_new dec1_mux = SOC_DAPM_ENUM(
				"DEC1 MUX Mux", dec1_mux_enum);
static const struct snd_kcontrol_new dec2_mux = SOC_DAPM_ENUM(
				"DEC2 MUX Mux",	dec2_mux_enum);
static const struct snd_kcontrol_new cic1_mux = SOC_DAPM_ENUM(
				"CIC1 MUX Mux", cic1_mux_enum);
static const struct snd_kcontrol_new cic2_mux = SOC_DAPM_ENUM(
				"CIC2 MUX Mux",	cic2_mux_enum);
static const struct snd_kcontrol_new rx_mix1_inp1_mux = SOC_DAPM_ENUM(
				"RX1 MIX1 INP1 Mux", rx_mix1_inp_enum[0]);
static const struct snd_kcontrol_new rx_mix1_inp2_mux = SOC_DAPM_ENUM(
				"RX1 MIX1 INP2 Mux", rx_mix1_inp_enum[1]);
static const struct snd_kcontrol_new rx_mix1_inp3_mux = SOC_DAPM_ENUM(
				"RX1 MIX1 INP3 Mux", rx_mix1_inp_enum[2]);
static const struct snd_kcontrol_new rx2_mix1_inp1_mux = SOC_DAPM_ENUM(
				"RX2 MIX1 INP1 Mux", rx2_mix1_inp_enum[0]);
static const struct snd_kcontrol_new rx2_mix1_inp2_mux = SOC_DAPM_ENUM(
				"RX2 MIX1 INP2 Mux", rx2_mix1_inp_enum[1]);
static const struct snd_kcontrol_new rx2_mix1_inp3_mux = SOC_DAPM_ENUM(
				"RX2 MIX1 INP3 Mux", rx2_mix1_inp_enum[2]);
static const struct snd_kcontrol_new rx3_mix1_inp1_mux = SOC_DAPM_ENUM(
				"RX3 MIX1 INP1 Mux", rx3_mix1_inp_enum[0]);
static const struct snd_kcontrol_new rx3_mix1_inp2_mux = SOC_DAPM_ENUM(
				"RX3 MIX1 INP2 Mux", rx3_mix1_inp_enum[1]);
static const struct snd_kcontrol_new rx3_mix1_inp3_mux = SOC_DAPM_ENUM(
				"RX3 MIX1 INP3 Mux", rx3_mix1_inp_enum[2]);

/* Digital Gain control -38.4 dB to +38.4 dB in 0.3 dB steps */
static const DECLARE_TLV_DB_SCALE(digital_gain, -3840, 30, 0);

/* Cutoff Freq for High Pass Filter at -3dB */
static const char * const hpf_cutoff_text[] = {
	"4Hz", "75Hz", "150Hz",
};

static SOC_ENUM_SINGLE_DECL(tx1_hpf_cutoff_enum, LPASS_CDC_TX1_MUX_CTL, 4,
			    hpf_cutoff_text);
static SOC_ENUM_SINGLE_DECL(tx2_hpf_cutoff_enum, LPASS_CDC_TX2_MUX_CTL, 4,
			    hpf_cutoff_text);

/* cut off for dc blocker inside rx chain */
static const char * const dc_blocker_cutoff_text[] = {
	"4Hz", "75Hz", "150Hz",
};

static SOC_ENUM_SINGLE_DECL(rx1_dcb_cutoff_enum, LPASS_CDC_RX1_B4_CTL, 0,
			    dc_blocker_cutoff_text);
static SOC_ENUM_SINGLE_DECL(rx2_dcb_cutoff_enum, LPASS_CDC_RX2_B4_CTL, 0,
			    dc_blocker_cutoff_text);
static SOC_ENUM_SINGLE_DECL(rx3_dcb_cutoff_enum, LPASS_CDC_RX3_B4_CTL, 0,
			    dc_blocker_cutoff_text);

static const struct snd_kcontrol_new msm8916_wcd_digital_snd_controls[] = {
	SOC_SINGLE_S8_TLV("RX1 Digital Volume", LPASS_CDC_RX1_VOL_CTL_B2_CTL,
			  -128, 127, digital_gain),
	SOC_SINGLE_S8_TLV("RX2 Digital Volume", LPASS_CDC_RX2_VOL_CTL_B2_CTL,
			  -128, 127, digital_gain),
	SOC_SINGLE_S8_TLV("RX3 Digital Volume", LPASS_CDC_RX3_VOL_CTL_B2_CTL,
			  -128, 127, digital_gain),
	SOC_SINGLE_S8_TLV("TX1 Digital Volume", LPASS_CDC_TX1_VOL_CTL_GAIN,
			  -128, 127, digital_gain),
	SOC_SINGLE_S8_TLV("TX2 Digital Volume", LPASS_CDC_TX2_VOL_CTL_GAIN,
			  -128, 127, digital_gain),
	SOC_ENUM("TX1 HPF Cutoff", tx1_hpf_cutoff_enum),
	SOC_ENUM("TX2 HPF Cutoff", tx2_hpf_cutoff_enum),
	SOC_SINGLE("TX1 HPF Switch", LPASS_CDC_TX1_MUX_CTL, 3, 1, 0),
	SOC_SINGLE("TX2 HPF Switch", LPASS_CDC_TX2_MUX_CTL, 3, 1, 0),
	SOC_ENUM("RX1 DCB Cutoff", rx1_dcb_cutoff_enum),
	SOC_ENUM("RX2 DCB Cutoff", rx2_dcb_cutoff_enum),
	SOC_ENUM("RX3 DCB Cutoff", rx3_dcb_cutoff_enum),
	SOC_SINGLE("RX1 DCB Switch", LPASS_CDC_RX1_B5_CTL, 2, 1, 0),
	SOC_SINGLE("RX2 DCB Switch", LPASS_CDC_RX2_B5_CTL, 2, 1, 0),
	SOC_SINGLE("RX3 DCB Switch", LPASS_CDC_RX3_B5_CTL, 2, 1, 0),
	SOC_SINGLE("RX1 Mute Switch", LPASS_CDC_RX1_B6_CTL, 0, 1, 0),
	SOC_SINGLE("RX2 Mute Switch", LPASS_CDC_RX2_B6_CTL, 0, 1, 0),
	SOC_SINGLE("RX3 Mute Switch", LPASS_CDC_RX3_B6_CTL, 0, 1, 0),
};

static int msm8916_wcd_digital_enable_interpolator(
						struct snd_soc_dapm_widget *w,
						struct snd_kcontrol *kcontrol,
						int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/* apply the digital gain after the interpolator is enabled */
		usleep_range(10000, 10100);
		snd_soc_write(codec, rx_gain_reg[w->shift],
			      snd_soc_read(codec, rx_gain_reg[w->shift]));
		break;
	}
	return 0;
}

static int msm8916_wcd_digital_enable_dec(struct snd_soc_dapm_widget *w,
					  struct snd_kcontrol *kcontrol,
					  int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	unsigned int decimator = w->shift + 1;
	u16 dec_reset_reg, tx_vol_ctl_reg, tx_mux_ctl_reg;
	u8 dec_hpf_cut_of_freq;

	dec_reset_reg = LPASS_CDC_CLK_TX_RESET_B1_CTL;
	tx_vol_ctl_reg = LPASS_CDC_TX1_VOL_CTL_CFG + 32 * (decimator - 1);
	tx_mux_ctl_reg = LPASS_CDC_TX1_MUX_CTL + 32 * (decimator - 1);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Enable TX digital mute */
		snd_soc_update_bits(codec, tx_vol_ctl_reg,
				    TX_VOL_CTL_CFG_MUTE_EN_MASK,
				    TX_VOL_CTL_CFG_MUTE_EN_ENABLE);
		dec_hpf_cut_of_freq = snd_soc_read(codec, tx_mux_ctl_reg) &
					TX_MUX_CTL_CUT_OFF_FREQ_MASK;
		dec_hpf_cut_of_freq >>= TX_MUX_CTL_CUT_OFF_FREQ_SHIFT;
		if (dec_hpf_cut_of_freq != TX_MUX_CTL_CF_NEG_3DB_150HZ) {
			/* set cut of freq to CF_MIN_3DB_150HZ (0x1) */
			snd_soc_update_bits(codec, tx_mux_ctl_reg,
					    TX_MUX_CTL_CUT_OFF_FREQ_MASK,
					    TX_MUX_CTL_CF_NEG_3DB_150HZ);
		}
		break;
	case SND_SOC_DAPM_POST_PMU:
		/* enable HPF */
		snd_soc_update_bits(codec, tx_mux_ctl_reg,
				    TX_MUX_CTL_HPF_BP_SEL_MASK,
				    TX_MUX_CTL_HPF_BP_SEL_NO_BYPASS);
		/* apply the digital gain after the decimator is enabled */
		snd_soc_write(codec, tx_gain_reg[w->shift],
			      snd_soc_read(codec, tx_gain_reg[w->shift]));
		snd_soc_update_bits(codec, tx_vol_ctl_reg,
				    TX_VOL_CTL_CFG_MUTE_EN_MASK, 0);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, tx_vol_ctl_reg,
				    TX_VOL_CTL_CFG_MUTE_EN_MASK,
				    TX_VOL_CTL_CFG_MUTE_EN_ENABLE);
		snd_soc_update_bits(codec, tx_mux_ctl_reg,
				    TX_MUX_CTL_HPF_BP_SEL_MASK,
				    TX_MUX_CTL_HPF_BP_SEL_BYPASS);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, dec_reset_reg, 1 << w->shift,
				    1 << w->shift);
		snd_soc_update_bits(codec, dec_reset_reg, 1 << w->shift, 0x0);
		snd_soc_update_bits(codec, tx_mux_ctl_reg,
				    TX_MUX_CTL_HPF_BP_SEL_MASK,
				    TX_MUX_CTL_HPF_BP_SEL_BYPASS);
		snd_soc_update_bits(codec, tx_vol_ctl_reg,
				    TX_VOL_CTL_CFG_MUTE_EN_MASK, 0);
		break;
	}

	return 0;
}

static int msm8916_wcd_digital_enable_dmic(struct snd_soc_dapm_widget *w,
					   struct snd_kcontrol *kcontrol,
					   int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	unsigned int dmic;
	int ret;
	/* get dmic number out of widget name */
	char *dmic_num = strpbrk(w->name, "12");

	if (dmic_num == NULL) {
		dev_err(codec->dev, "Invalid DMIC\n");
		return -EINVAL;
	}
	ret = kstrtouint(dmic_num, 10, &dmic);
	if (ret < 0 || dmic > 2) {
		dev_err(codec->dev, "Invalid DMIC line on the codec\n");
		return -EINVAL;
	}

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, LPASS_CDC_CLK_DMIC_B1_CTL,
				    DMIC_B1_CTL_DMIC0_CLK_SEL_MASK,
				    DMIC_B1_CTL_DMIC0_CLK_SEL_DIV3);
		switch (dmic) {
		case 1:
			snd_soc_update_bits(codec, LPASS_CDC_TX1_DMIC_CTL,
					    TXN_DMIC_CTL_CLK_SEL_MASK,
					    TXN_DMIC_CTL_CLK_SEL_DIV3);
			break;
		case 2:
			snd_soc_update_bits(codec, LPASS_CDC_TX2_DMIC_CTL,
					    TXN_DMIC_CTL_CLK_SEL_MASK,
					    TXN_DMIC_CTL_CLK_SEL_DIV3);
			break;
		}
		break;
	}

	return 0;
}

static const struct snd_soc_dapm_widget msm8916_wcd_digital_dapm_widgets[] = {
	/*RX stuff */
	SND_SOC_DAPM_AIF_IN("I2S RX1", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("I2S RX2", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("I2S RX3", NULL, 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_OUTPUT("PDM_RX1"),
	SND_SOC_DAPM_OUTPUT("PDM_RX2"),
	SND_SOC_DAPM_OUTPUT("PDM_RX3"),

	SND_SOC_DAPM_INPUT("LPASS_PDM_TX"),

	SND_SOC_DAPM_MIXER("RX1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX2 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX3 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),

	/* Interpolator */
	SND_SOC_DAPM_MIXER_E("RX1 INT", LPASS_CDC_CLK_RX_B1_CTL, 0, 0, NULL,
			     0, msm8916_wcd_digital_enable_interpolator,
			     SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("RX2 INT", LPASS_CDC_CLK_RX_B1_CTL, 1, 0, NULL,
			     0, msm8916_wcd_digital_enable_interpolator,
			     SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("RX3 INT", LPASS_CDC_CLK_RX_B1_CTL, 2, 0, NULL,
			     0, msm8916_wcd_digital_enable_interpolator,
			     SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MUX("RX1 MIX1 INP1", SND_SOC_NOPM, 0, 0,
			 &rx_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX1 MIX1 INP2", SND_SOC_NOPM, 0, 0,
			 &rx_mix1_inp2_mux),
	SND_SOC_DAPM_MUX("RX1 MIX1 INP3", SND_SOC_NOPM, 0, 0,
			 &rx_mix1_inp3_mux),
	SND_SOC_DAPM_MUX("RX2 MIX1 INP1", SND_SOC_NOPM, 0, 0,
			 &rx2_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX2 MIX1 INP2", SND_SOC_NOPM, 0, 0,
			 &rx2_mix1_inp2_mux),
	SND_SOC_DAPM_MUX("RX2 MIX1 INP3", SND_SOC_NOPM, 0, 0,
			 &rx2_mix1_inp3_mux),
	SND_SOC_DAPM_MUX("RX3 MIX1 INP1", SND_SOC_NOPM, 0, 0,
			 &rx3_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX3 MIX1 INP2", SND_SOC_NOPM, 0, 0,
			 &rx3_mix1_inp2_mux),
	SND_SOC_DAPM_MUX("RX3 MIX1 INP3", SND_SOC_NOPM, 0, 0,
			 &rx3_mix1_inp3_mux),

	SND_SOC_DAPM_MUX("CIC1 MUX", SND_SOC_NOPM, 0, 0, &cic1_mux),
	SND_SOC_DAPM_MUX("CIC2 MUX", SND_SOC_NOPM, 0, 0, &cic2_mux),
	/* TX */
	SND_SOC_DAPM_MIXER("ADC1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("ADC2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("ADC3", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MUX_E("DEC1 MUX", LPASS_CDC_CLK_TX_CLK_EN_B1_CTL, 0, 0,
			   &dec1_mux, msm8916_wcd_digital_enable_dec,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MUX_E("DEC2 MUX", LPASS_CDC_CLK_TX_CLK_EN_B1_CTL, 1, 0,
			   &dec2_mux, msm8916_wcd_digital_enable_dec,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT("I2S TX1", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("I2S TX2", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("I2S TX3", NULL, 0, SND_SOC_NOPM, 0, 0),

	/* Digital Mic Inputs */
	SND_SOC_DAPM_ADC_E("DMIC1", NULL, SND_SOC_NOPM, 0, 0,
			   msm8916_wcd_digital_enable_dmic,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("DMIC2", NULL, SND_SOC_NOPM, 0, 0,
			   msm8916_wcd_digital_enable_dmic,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("DMIC_CLK", LPASS_CDC_CLK_DMIC_B1_CTL, 0, 0,
			    NULL, 0),
	SND_SOC_DAPM_SUPPLY("RX_I2S_CLK", LPASS_CDC_CLK_RX_I2S_CTL,
			    4, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("TX_I2S_CLK", LPASS_CDC_CLK_TX_I2S_CTL, 4, 0,
			    NULL, 0),

	SND_SOC_DAPM_SUPPLY("MCLK", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("PDM_CLK", LPASS_CDC_CLK_PDM_CTL, 0, 0, NULL, 0),
	/* Connectivity Clock */
	SND_SOC_DAPM_SUPPLY_S("CDC_CONN", -2, LPASS_CDC_CLK_OTHR_CTL, 2, 0,
			      NULL, 0),
	SND_SOC_DAPM_MIC("Digital Mic1", NULL),
	SND_SOC_DAPM_MIC("Digital Mic2", NULL),

};

static int msm8916_wcd_digital_get_clks(struct platform_device *pdev,
					struct msm8916_wcd_digital_priv	*priv)
{
	struct device *dev = &pdev->dev;

	priv->ahbclk = devm_clk_get(dev, "ahbix-clk");
	if (IS_ERR(priv->ahbclk)) {
		dev_err(dev, "failed to get ahbix clk\n");
		return PTR_ERR(priv->ahbclk);
	}

	priv->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(priv->mclk)) {
		dev_err(dev, "failed to get mclk\n");
		return PTR_ERR(priv->mclk);
	}

	return 0;
}

static int msm8916_wcd_digital_codec_probe(struct snd_soc_codec *codec)
{
	struct msm8916_wcd_digital_priv *priv = dev_get_drvdata(codec->dev);

	snd_soc_codec_set_drvdata(codec, priv);

	return 0;
}

static int msm8916_wcd_digital_codec_set_sysclk(struct snd_soc_codec *codec,
						int clk_id, int source,
						unsigned int freq, int dir)
{
	struct msm8916_wcd_digital_priv *p = dev_get_drvdata(codec->dev);

	return clk_set_rate(p->mclk, freq);
}

static int msm8916_wcd_digital_hw_params(struct snd_pcm_substream *substream,
					 struct snd_pcm_hw_params *params,
					 struct snd_soc_dai *dai)
{
	u8 tx_fs_rate;
	u8 rx_fs_rate;

	switch (params_rate(params)) {
	case 8000:
		tx_fs_rate = TX_I2S_CTL_TX_I2S_FS_RATE_F_8_KHZ;
		rx_fs_rate = RX_I2S_CTL_RX_I2S_FS_RATE_F_8_KHZ;
		break;
	case 16000:
		tx_fs_rate = TX_I2S_CTL_TX_I2S_FS_RATE_F_16_KHZ;
		rx_fs_rate = RX_I2S_CTL_RX_I2S_FS_RATE_F_16_KHZ;
		break;
	case 32000:
		tx_fs_rate = TX_I2S_CTL_TX_I2S_FS_RATE_F_32_KHZ;
		rx_fs_rate = RX_I2S_CTL_RX_I2S_FS_RATE_F_32_KHZ;
		break;
	case 48000:
		tx_fs_rate = TX_I2S_CTL_TX_I2S_FS_RATE_F_48_KHZ;
		rx_fs_rate = RX_I2S_CTL_RX_I2S_FS_RATE_F_48_KHZ;
		break;
	default:
		dev_err(dai->codec->dev, "Invalid sampling rate %d\n",
			params_rate(params));
		return -EINVAL;
	}

	switch (substream->stream) {
	case SNDRV_PCM_STREAM_CAPTURE:
		snd_soc_update_bits(dai->codec, LPASS_CDC_CLK_TX_I2S_CTL,
				    TX_I2S_CTL_TX_I2S_FS_RATE_MASK, tx_fs_rate);
		break;
	case SNDRV_PCM_STREAM_PLAYBACK:
		snd_soc_update_bits(dai->codec, LPASS_CDC_CLK_RX_I2S_CTL,
				    RX_I2S_CTL_RX_I2S_FS_RATE_MASK, rx_fs_rate);
		break;
	default:
		return -EINVAL;
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		snd_soc_update_bits(dai->codec, LPASS_CDC_CLK_TX_I2S_CTL,
				    TX_I2S_CTL_TX_I2S_MODE_MASK,
				    TX_I2S_CTL_TX_I2S_MODE_16);
		snd_soc_update_bits(dai->codec, LPASS_CDC_CLK_RX_I2S_CTL,
				    RX_I2S_CTL_RX_I2S_MODE_MASK,
				    RX_I2S_CTL_RX_I2S_MODE_16);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		snd_soc_update_bits(dai->codec, LPASS_CDC_CLK_TX_I2S_CTL,
				    TX_I2S_CTL_TX_I2S_MODE_MASK,
				    TX_I2S_CTL_TX_I2S_MODE_32);
		snd_soc_update_bits(dai->codec, LPASS_CDC_CLK_RX_I2S_CTL,
				    RX_I2S_CTL_RX_I2S_MODE_MASK,
				    RX_I2S_CTL_RX_I2S_MODE_32);
		break;
	default:
		dev_err(dai->dev, "%s: wrong format selected\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dapm_route msm8916_wcd_digital_audio_map[] = {

	{"I2S RX1",  NULL, "AIF1 Playback"},
	{"I2S RX2",  NULL, "AIF1 Playback"},
	{"I2S RX3",  NULL, "AIF1 Playback"},

	{"AIF1 Capture", NULL, "I2S TX1"},
	{"AIF1 Capture", NULL, "I2S TX2"},
	{"AIF1 Capture", NULL, "I2S TX3"},

	{"CIC1 MUX", "DMIC", "DEC1 MUX"},
	{"CIC1 MUX", "AMIC", "DEC1 MUX"},
	{"CIC2 MUX", "DMIC", "DEC2 MUX"},
	{"CIC2 MUX", "AMIC", "DEC2 MUX"},

	/* Decimator Inputs */
	{"DEC1 MUX", "DMIC1", "DMIC1"},
	{"DEC1 MUX", "DMIC2", "DMIC2"},
	{"DEC1 MUX", "ADC1", "ADC1"},
	{"DEC1 MUX", "ADC2", "ADC2"},
	{"DEC1 MUX", "ADC3", "ADC3"},
	{"DEC1 MUX", NULL, "CDC_CONN"},

	{"DEC2 MUX", "DMIC1", "DMIC1"},
	{"DEC2 MUX", "DMIC2", "DMIC2"},
	{"DEC2 MUX", "ADC1", "ADC1"},
	{"DEC2 MUX", "ADC2", "ADC2"},
	{"DEC2 MUX", "ADC3", "ADC3"},
	{"DEC2 MUX", NULL, "CDC_CONN"},

	{"DMIC1", NULL, "DMIC_CLK"},
	{"DMIC2", NULL, "DMIC_CLK"},

	{"I2S TX1", NULL, "CIC1 MUX"},
	{"I2S TX2", NULL, "CIC2 MUX"},

	{"I2S TX1", NULL, "TX_I2S_CLK"},
	{"I2S TX2", NULL, "TX_I2S_CLK"},

	{"TX_I2S_CLK", NULL, "MCLK"},
	{"TX_I2S_CLK", NULL, "PDM_CLK"},

	{"ADC1", NULL, "LPASS_PDM_TX"},
	{"ADC2", NULL, "LPASS_PDM_TX"},
	{"ADC3", NULL, "LPASS_PDM_TX"},

	{"I2S RX1", NULL, "RX_I2S_CLK"},
	{"I2S RX2", NULL, "RX_I2S_CLK"},
	{"I2S RX3", NULL, "RX_I2S_CLK"},

	{"RX_I2S_CLK", NULL, "PDM_CLK"},
	{"RX_I2S_CLK", NULL, "MCLK"},
	{"RX_I2S_CLK", NULL, "CDC_CONN"},

	/* RX1 PATH.. */
	{"PDM_RX1", NULL, "RX1 INT"},
	{"RX1 INT", NULL, "RX1 MIX1"},

	{"RX1 MIX1", NULL, "RX1 MIX1 INP1"},
	{"RX1 MIX1", NULL, "RX1 MIX1 INP2"},
	{"RX1 MIX1", NULL, "RX1 MIX1 INP3"},

	{"RX1 MIX1 INP1", "RX1", "I2S RX1"},
	{"RX1 MIX1 INP1", "RX2", "I2S RX2"},
	{"RX1 MIX1 INP1", "RX3", "I2S RX3"},

	{"RX1 MIX1 INP2", "RX1", "I2S RX1"},
	{"RX1 MIX1 INP2", "RX2", "I2S RX2"},
	{"RX1 MIX1 INP2", "RX3", "I2S RX3"},

	{"RX1 MIX1 INP3", "RX1", "I2S RX1"},
	{"RX1 MIX1 INP3", "RX2", "I2S RX2"},
	{"RX1 MIX1 INP3", "RX3", "I2S RX3"},

	/* RX2 PATH */
	{"PDM_RX2", NULL, "RX2 INT"},
	{"RX2 INT", NULL, "RX2 MIX1"},

	{"RX2 MIX1", NULL, "RX2 MIX1 INP1"},
	{"RX2 MIX1", NULL, "RX2 MIX1 INP2"},
	{"RX2 MIX1", NULL, "RX2 MIX1 INP3"},

	{"RX2 MIX1 INP1", "RX1", "I2S RX1"},
	{"RX2 MIX1 INP1", "RX2", "I2S RX2"},
	{"RX2 MIX1 INP1", "RX3", "I2S RX3"},

	{"RX2 MIX1 INP2", "RX1", "I2S RX1"},
	{"RX2 MIX1 INP2", "RX2", "I2S RX2"},
	{"RX2 MIX1 INP2", "RX3", "I2S RX3"},

	{"RX2 MIX1 INP3", "RX1", "I2S RX1"},
	{"RX2 MIX1 INP3", "RX2", "I2S RX2"},
	{"RX2 MIX1 INP3", "RX3", "I2S RX3"},

	/* RX3 PATH */
	{"PDM_RX3", NULL, "RX3 INT"},
	{"RX3 INT", NULL, "RX3 MIX1"},

	{"RX3 MIX1", NULL, "RX3 MIX1 INP1"},
	{"RX3 MIX1", NULL, "RX3 MIX1 INP2"},
	{"RX3 MIX1", NULL, "RX3 MIX1 INP3"},

	{"RX3 MIX1 INP1", "RX1", "I2S RX1"},
	{"RX3 MIX1 INP1", "RX2", "I2S RX2"},
	{"RX3 MIX1 INP1", "RX3", "I2S RX3"},

	{"RX3 MIX1 INP2", "RX1", "I2S RX1"},
	{"RX3 MIX1 INP2", "RX2", "I2S RX2"},
	{"RX3 MIX1 INP2", "RX3", "I2S RX3"},

	{"RX3 MIX1 INP3", "RX1", "I2S RX1"},
	{"RX3 MIX1 INP3", "RX2", "I2S RX2"},
	{"RX3 MIX1 INP3", "RX3", "I2S RX3"},

};

static int msm8916_wcd_digital_startup(struct snd_pcm_substream *substream,
				       struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct msm8916_wcd_digital_priv *msm8916_wcd;
	unsigned long mclk_rate;

	msm8916_wcd = snd_soc_codec_get_drvdata(codec);
	snd_soc_update_bits(codec, LPASS_CDC_CLK_MCLK_CTL,
			    MCLK_CTL_MCLK_EN_MASK,
			    MCLK_CTL_MCLK_EN_ENABLE);
	snd_soc_update_bits(codec, LPASS_CDC_CLK_PDM_CTL,
			    LPASS_CDC_CLK_PDM_CTL_PDM_CLK_SEL_MASK,
			    LPASS_CDC_CLK_PDM_CTL_PDM_CLK_SEL_FB);

	mclk_rate = clk_get_rate(msm8916_wcd->mclk);
	switch (mclk_rate) {
	case 12288000:
		snd_soc_update_bits(codec, LPASS_CDC_TOP_CTL,
				    TOP_CTL_DIG_MCLK_FREQ_MASK,
				    TOP_CTL_DIG_MCLK_FREQ_F_12_288MHZ);
		break;
	case 9600000:
		snd_soc_update_bits(codec, LPASS_CDC_TOP_CTL,
				    TOP_CTL_DIG_MCLK_FREQ_MASK,
				    TOP_CTL_DIG_MCLK_FREQ_F_9_6MHZ);
		break;
	default:
		dev_err(codec->dev, "Invalid mclk rate %ld\n", mclk_rate);
		break;
	}
	return 0;
}

static void msm8916_wcd_digital_shutdown(struct snd_pcm_substream *substream,
					 struct snd_soc_dai *dai)
{
	snd_soc_update_bits(dai->codec, LPASS_CDC_CLK_PDM_CTL,
			    LPASS_CDC_CLK_PDM_CTL_PDM_CLK_SEL_MASK, 0);
}

static const struct snd_soc_dai_ops msm8916_wcd_digital_dai_ops = {
	.startup = msm8916_wcd_digital_startup,
	.shutdown = msm8916_wcd_digital_shutdown,
	.hw_params = msm8916_wcd_digital_hw_params,
};

static struct snd_soc_dai_driver msm8916_wcd_digital_dai[] = {
	[0] = {
	       .name = "msm8916_wcd_digital_i2s_rx1",
	       .id = 0,
	       .playback = {
			    .stream_name = "AIF1 Playback",
			    .rates = MSM8916_WCD_DIGITAL_RATES,
			    .formats = MSM8916_WCD_DIGITAL_FORMATS,
			    .channels_min = 1,
			    .channels_max = 3,
			    },
	       .ops = &msm8916_wcd_digital_dai_ops,
	       },
	[1] = {
	       .name = "msm8916_wcd_digital_i2s_tx1",
	       .id = 1,
	       .capture = {
			   .stream_name = "AIF1 Capture",
			   .rates = MSM8916_WCD_DIGITAL_RATES,
			   .formats = MSM8916_WCD_DIGITAL_FORMATS,
			   .channels_min = 1,
			   .channels_max = 4,
			   },
	       .ops = &msm8916_wcd_digital_dai_ops,
	       },
};

static const struct snd_soc_codec_driver msm8916_wcd_digital = {
	.probe = msm8916_wcd_digital_codec_probe,
	.set_sysclk = msm8916_wcd_digital_codec_set_sysclk,
	.component_driver = {
		.controls = msm8916_wcd_digital_snd_controls,
		.num_controls = ARRAY_SIZE(msm8916_wcd_digital_snd_controls),
		.dapm_widgets = msm8916_wcd_digital_dapm_widgets,
		.num_dapm_widgets =
				 ARRAY_SIZE(msm8916_wcd_digital_dapm_widgets),
		.dapm_routes = msm8916_wcd_digital_audio_map,
		.num_dapm_routes = ARRAY_SIZE(msm8916_wcd_digital_audio_map),
	},
};

static const struct regmap_config msm8916_codec_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = LPASS_CDC_TX2_DMIC_CTL,
	.cache_type = REGCACHE_FLAT,
};

static int msm8916_wcd_digital_probe(struct platform_device *pdev)
{
	struct msm8916_wcd_digital_priv *priv;
	struct device *dev = &pdev->dev;
	void __iomem *base;
	struct resource *mem_res;
	struct regmap *digital_map;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, mem_res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	digital_map =
	    devm_regmap_init_mmio(&pdev->dev, base,
				  &msm8916_codec_regmap_config);
	if (IS_ERR(digital_map))
		return PTR_ERR(digital_map);

	ret = msm8916_wcd_digital_get_clks(pdev, priv);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(priv->ahbclk);
	if (ret < 0) {
		dev_err(dev, "failed to enable ahbclk %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(priv->mclk);
	if (ret < 0) {
		dev_err(dev, "failed to enable mclk %d\n", ret);
		return ret;
	}

	dev_set_drvdata(dev, priv);

	return snd_soc_register_codec(dev, &msm8916_wcd_digital,
				      msm8916_wcd_digital_dai,
				      ARRAY_SIZE(msm8916_wcd_digital_dai));
}

static int msm8916_wcd_digital_remove(struct platform_device *pdev)
{
	struct msm8916_wcd_digital_priv *priv = dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_codec(&pdev->dev);
	clk_disable_unprepare(priv->mclk);
	clk_disable_unprepare(priv->ahbclk);

	return 0;
}

static const struct of_device_id msm8916_wcd_digital_match_table[] = {
	{ .compatible = "qcom,msm8916-wcd-digital-codec" },
	{ }
};

MODULE_DEVICE_TABLE(of, msm8916_wcd_digital_match_table);

static struct platform_driver msm8916_wcd_digital_driver = {
	.driver = {
		   .name = "msm8916-wcd-digital-codec",
		   .of_match_table = msm8916_wcd_digital_match_table,
	},
	.probe = msm8916_wcd_digital_probe,
	.remove = msm8916_wcd_digital_remove,
};

module_platform_driver(msm8916_wcd_digital_driver);

MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org>");
MODULE_DESCRIPTION("MSM8916 WCD Digital Codec driver");
MODULE_LICENSE("GPL v2");
