/*
 *  cht-bsw-rt5645.c - ASoc Machine driver for Intel Cherryview-based platforms
 *                     Cherrytrail and Braswell, with RT5645 codec.
 *
 *  Copyright (C) 2015 Intel Corp
 *  Author: Fang, Yang A <yang.a.fang@intel.com>
 *	        N,Harshapriya <harshapriya.n@intel.com>
 *  This file is modified from cht_bsw_rt5672.c
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/dmi.h>
#include <linux/slab.h>
#include <asm/cpu_device_id.h>
#include <asm/platform_sst_audio.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/soc-acpi.h>
#include "../../codecs/rt5645.h"
#include "../atom/sst-atom-controls.h"

#define CHT_PLAT_CLK_3_HZ	19200000
#define CHT_CODEC_DAI1	"rt5645-aif1"
#define CHT_CODEC_DAI2	"rt5645-aif2"

struct cht_acpi_card {
	char *codec_id;
	int codec_type;
	struct snd_soc_card *soc_card;
};

struct cht_mc_private {
	struct snd_soc_jack jack;
	struct cht_acpi_card *acpi_card;
	char codec_name[SND_ACPI_I2C_ID_LEN];
	struct clk *mclk;
};

#define CHT_RT5645_MAP(quirk)	((quirk) & GENMASK(7, 0))
#define CHT_RT5645_SSP2_AIF2     BIT(16) /* default is using AIF1  */
#define CHT_RT5645_SSP0_AIF1     BIT(17)
#define CHT_RT5645_SSP0_AIF2     BIT(18)

static unsigned long cht_rt5645_quirk = 0;

static void log_quirks(struct device *dev)
{
	if (cht_rt5645_quirk & CHT_RT5645_SSP2_AIF2)
		dev_info(dev, "quirk SSP2_AIF2 enabled");
	if (cht_rt5645_quirk & CHT_RT5645_SSP0_AIF1)
		dev_info(dev, "quirk SSP0_AIF1 enabled");
	if (cht_rt5645_quirk & CHT_RT5645_SSP0_AIF2)
		dev_info(dev, "quirk SSP0_AIF2 enabled");
}

static int platform_clock_control(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int  event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct snd_soc_dai *codec_dai;
	struct cht_mc_private *ctx = snd_soc_card_get_drvdata(card);
	int ret;

	codec_dai = snd_soc_card_get_codec_dai(card, CHT_CODEC_DAI1);
	if (!codec_dai)
		codec_dai = snd_soc_card_get_codec_dai(card, CHT_CODEC_DAI2);

	if (!codec_dai) {
		dev_err(card->dev, "Codec dai not found; Unable to set platform clock\n");
		return -EIO;
	}

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		ret = clk_prepare_enable(ctx->mclk);
		if (ret < 0) {
			dev_err(card->dev,
				"could not configure MCLK state");
			return ret;
		}
	} else {
		/* Set codec sysclk source to its internal clock because codec PLL will
		 * be off when idle and MCLK will also be off when codec is
		 * runtime suspended. Codec needs clock for jack detection and button
		 * press. MCLK is turned off with clock framework or ACPI.
		 */
		ret = snd_soc_dai_set_sysclk(codec_dai, RT5645_SCLK_S_RCCLK,
					48000 * 512, SND_SOC_CLOCK_IN);
		if (ret < 0) {
			dev_err(card->dev, "can't set codec sysclk: %d\n", ret);
			return ret;
		}

		clk_disable_unprepare(ctx->mclk);
	}

	return 0;
}

static const struct snd_soc_dapm_widget cht_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_MIC("Int Analog Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_SUPPLY("Platform Clock", SND_SOC_NOPM, 0, 0,
			platform_clock_control, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route cht_rt5645_audio_map[] = {
	{"IN1P", NULL, "Headset Mic"},
	{"IN1N", NULL, "Headset Mic"},
	{"DMIC L1", NULL, "Int Mic"},
	{"DMIC R1", NULL, "Int Mic"},
	{"IN2P", NULL, "Int Analog Mic"},
	{"IN2N", NULL, "Int Analog Mic"},
	{"Headphone", NULL, "HPOL"},
	{"Headphone", NULL, "HPOR"},
	{"Ext Spk", NULL, "SPOL"},
	{"Ext Spk", NULL, "SPOR"},
	{"Headphone", NULL, "Platform Clock"},
	{"Headset Mic", NULL, "Platform Clock"},
	{"Int Mic", NULL, "Platform Clock"},
	{"Int Analog Mic", NULL, "Platform Clock"},
	{"Int Analog Mic", NULL, "micbias1"},
	{"Int Analog Mic", NULL, "micbias2"},
	{"Ext Spk", NULL, "Platform Clock"},
};

static const struct snd_soc_dapm_route cht_rt5650_audio_map[] = {
	{"IN1P", NULL, "Headset Mic"},
	{"IN1N", NULL, "Headset Mic"},
	{"DMIC L2", NULL, "Int Mic"},
	{"DMIC R2", NULL, "Int Mic"},
	{"Headphone", NULL, "HPOL"},
	{"Headphone", NULL, "HPOR"},
	{"Ext Spk", NULL, "SPOL"},
	{"Ext Spk", NULL, "SPOR"},
	{"Headphone", NULL, "Platform Clock"},
	{"Headset Mic", NULL, "Platform Clock"},
	{"Int Mic", NULL, "Platform Clock"},
	{"Ext Spk", NULL, "Platform Clock"},
};

static const struct snd_soc_dapm_route cht_rt5645_ssp2_aif1_map[] = {
	{"AIF1 Playback", NULL, "ssp2 Tx"},
	{"ssp2 Tx", NULL, "codec_out0"},
	{"ssp2 Tx", NULL, "codec_out1"},
	{"codec_in0", NULL, "ssp2 Rx" },
	{"codec_in1", NULL, "ssp2 Rx" },
	{"ssp2 Rx", NULL, "AIF1 Capture"},
};

static const struct snd_soc_dapm_route cht_rt5645_ssp2_aif2_map[] = {
	{"AIF2 Playback", NULL, "ssp2 Tx"},
	{"ssp2 Tx", NULL, "codec_out0"},
	{"ssp2 Tx", NULL, "codec_out1"},
	{"codec_in0", NULL, "ssp2 Rx" },
	{"codec_in1", NULL, "ssp2 Rx" },
	{"ssp2 Rx", NULL, "AIF2 Capture"},
};

static const struct snd_soc_dapm_route cht_rt5645_ssp0_aif1_map[] = {
	{"AIF1 Playback", NULL, "ssp0 Tx"},
	{"ssp0 Tx", NULL, "modem_out"},
	{"modem_in", NULL, "ssp0 Rx" },
	{"ssp0 Rx", NULL, "AIF1 Capture"},
};

static const struct snd_soc_dapm_route cht_rt5645_ssp0_aif2_map[] = {
	{"AIF2 Playback", NULL, "ssp0 Tx"},
	{"ssp0 Tx", NULL, "modem_out"},
	{"modem_in", NULL, "ssp0 Rx" },
	{"ssp0 Rx", NULL, "AIF2 Capture"},
};

static const struct snd_kcontrol_new cht_mc_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
	SOC_DAPM_PIN_SWITCH("Int Analog Mic"),
	SOC_DAPM_PIN_SWITCH("Ext Spk"),
};

static struct snd_soc_jack_pin cht_bsw_jack_pins[] = {
	{
		.pin	= "Headphone",
		.mask	= SND_JACK_HEADPHONE,
	},
	{
		.pin	= "Headset Mic",
		.mask	= SND_JACK_MICROPHONE,
	},
};

static int cht_aif1_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;

	/* set codec PLL source to the 19.2MHz platform clock (MCLK) */
	ret = snd_soc_dai_set_pll(codec_dai, 0, RT5645_PLL1_S_MCLK,
				  CHT_PLAT_CLK_3_HZ, params_rate(params) * 512);
	if (ret < 0) {
		dev_err(rtd->dev, "can't set codec pll: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, RT5645_SCLK_S_PLL1,
				params_rate(params) * 512, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(rtd->dev, "can't set codec sysclk: %d\n", ret);
		return ret;
	}

	return 0;
}

/* uncomment when we have a real quirk
static int cht_rt5645_quirk_cb(const struct dmi_system_id *id)
{
	cht_rt5645_quirk = (unsigned long)id->driver_data;
	return 1;
}
*/

static const struct dmi_system_id cht_rt5645_quirk_table[] = {
	{
	},
};

static int cht_codec_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_card *card = runtime->card;
	struct cht_mc_private *ctx = snd_soc_card_get_drvdata(runtime->card);
	struct snd_soc_codec *codec = runtime->codec;
	int jack_type;
	int ret;

	if ((cht_rt5645_quirk & CHT_RT5645_SSP2_AIF2) ||
	    (cht_rt5645_quirk & CHT_RT5645_SSP0_AIF2)) {
		/* Select clk_i2s2_asrc as ASRC clock source */
		rt5645_sel_asrc_clk_src(codec,
					RT5645_DA_STEREO_FILTER |
					RT5645_DA_MONO_L_FILTER |
					RT5645_DA_MONO_R_FILTER |
					RT5645_AD_STEREO_FILTER,
					RT5645_CLK_SEL_I2S2_ASRC);
	} else {
		/* Select clk_i2s1_asrc as ASRC clock source */
		rt5645_sel_asrc_clk_src(codec,
					RT5645_DA_STEREO_FILTER |
					RT5645_DA_MONO_L_FILTER |
					RT5645_DA_MONO_R_FILTER |
					RT5645_AD_STEREO_FILTER,
					RT5645_CLK_SEL_I2S1_ASRC);
	}

	if (cht_rt5645_quirk & CHT_RT5645_SSP2_AIF2) {
		ret = snd_soc_dapm_add_routes(&card->dapm,
					cht_rt5645_ssp2_aif2_map,
					ARRAY_SIZE(cht_rt5645_ssp2_aif2_map));
	} else if (cht_rt5645_quirk & CHT_RT5645_SSP0_AIF1) {
		ret = snd_soc_dapm_add_routes(&card->dapm,
					cht_rt5645_ssp0_aif1_map,
					ARRAY_SIZE(cht_rt5645_ssp0_aif1_map));
	} else if (cht_rt5645_quirk & CHT_RT5645_SSP0_AIF2) {
		ret = snd_soc_dapm_add_routes(&card->dapm,
					cht_rt5645_ssp0_aif2_map,
					ARRAY_SIZE(cht_rt5645_ssp0_aif2_map));
	} else {
		ret = snd_soc_dapm_add_routes(&card->dapm,
					cht_rt5645_ssp2_aif1_map,
					ARRAY_SIZE(cht_rt5645_ssp2_aif1_map));
	}
	if (ret)
		return ret;

	if (ctx->acpi_card->codec_type == CODEC_TYPE_RT5650)
		jack_type = SND_JACK_HEADPHONE | SND_JACK_MICROPHONE |
					SND_JACK_BTN_0 | SND_JACK_BTN_1 |
					SND_JACK_BTN_2 | SND_JACK_BTN_3;
	else
		jack_type = SND_JACK_HEADPHONE | SND_JACK_MICROPHONE;

	ret = snd_soc_card_jack_new(runtime->card, "Headset",
				    jack_type, &ctx->jack,
				    cht_bsw_jack_pins, ARRAY_SIZE(cht_bsw_jack_pins));
	if (ret) {
		dev_err(runtime->dev, "Headset jack creation failed %d\n", ret);
		return ret;
	}

	rt5645_set_jack_detect(codec, &ctx->jack, &ctx->jack, &ctx->jack);


	/*
	 * The firmware might enable the clock at
	 * boot (this information may or may not
	 * be reflected in the enable clock register).
	 * To change the rate we must disable the clock
	 * first to cover these cases. Due to common
	 * clock framework restrictions that do not allow
	 * to disable a clock that has not been enabled,
	 * we need to enable the clock first.
	 */
	ret = clk_prepare_enable(ctx->mclk);
	if (!ret)
		clk_disable_unprepare(ctx->mclk);

	ret = clk_set_rate(ctx->mclk, CHT_PLAT_CLK_3_HZ);

	if (ret)
		dev_err(runtime->dev, "unable to set MCLK rate\n");

	return ret;
}

static int cht_codec_fixup(struct snd_soc_pcm_runtime *rtd,
			    struct snd_pcm_hw_params *params)
{
	int ret;
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	/* The DSP will covert the FE rate to 48k, stereo, 24bits */
	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	if ((cht_rt5645_quirk & CHT_RT5645_SSP0_AIF1) ||
		(cht_rt5645_quirk & CHT_RT5645_SSP0_AIF2)) {

		/* set SSP0 to 16-bit */
		params_set_format(params, SNDRV_PCM_FORMAT_S16_LE);

		/*
		 * Default mode for SSP configuration is TDM 4 slot, override config
		 * with explicit setting to I2S 2ch 16-bit. The word length is set with
		 * dai_set_tdm_slot() since there is no other API exposed
		 */
		ret = snd_soc_dai_set_fmt(rtd->cpu_dai,
					SND_SOC_DAIFMT_I2S     |
					SND_SOC_DAIFMT_NB_NF   |
					SND_SOC_DAIFMT_CBS_CFS
			);
		if (ret < 0) {
			dev_err(rtd->dev, "can't set format to I2S, err %d\n", ret);
			return ret;
		}

		ret = snd_soc_dai_set_fmt(rtd->codec_dai,
					SND_SOC_DAIFMT_I2S     |
					SND_SOC_DAIFMT_NB_NF   |
					SND_SOC_DAIFMT_CBS_CFS
			);
		if (ret < 0) {
			dev_err(rtd->dev, "can't set format to I2S, err %d\n", ret);
			return ret;
		}

		ret = snd_soc_dai_set_tdm_slot(rtd->cpu_dai, 0x3, 0x3, 2, 16);
		if (ret < 0) {
			dev_err(rtd->dev, "can't set I2S config, err %d\n", ret);
			return ret;
		}

	} else {

		/* set SSP2 to 24-bit */
		params_set_format(params, SNDRV_PCM_FORMAT_S24_LE);

		/*
		 * Default mode for SSP configuration is TDM 4 slot
		 */
		ret = snd_soc_dai_set_fmt(rtd->codec_dai,
					SND_SOC_DAIFMT_DSP_B |
					SND_SOC_DAIFMT_IB_NF |
					SND_SOC_DAIFMT_CBS_CFS);
		if (ret < 0) {
			dev_err(rtd->dev, "can't set format to TDM %d\n", ret);
			return ret;
		}

		/* TDM 4 slots 24 bit, set Rx & Tx bitmask to 4 active slots */
		ret = snd_soc_dai_set_tdm_slot(rtd->codec_dai, 0xF, 0xF, 4, 24);
		if (ret < 0) {
			dev_err(rtd->dev, "can't set codec TDM slot %d\n", ret);
			return ret;
		}
	}
	return 0;
}

static int cht_aif1_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_single(substream->runtime,
			SNDRV_PCM_HW_PARAM_RATE, 48000);
}

static const struct snd_soc_ops cht_aif1_ops = {
	.startup = cht_aif1_startup,
};

static const struct snd_soc_ops cht_be_ssp2_ops = {
	.hw_params = cht_aif1_hw_params,
};

static struct snd_soc_dai_link cht_dailink[] = {
	[MERR_DPCM_AUDIO] = {
		.name = "Audio Port",
		.stream_name = "Audio",
		.cpu_dai_name = "media-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-mfld-platform",
		.nonatomic = true,
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ops = &cht_aif1_ops,
	},
	[MERR_DPCM_DEEP_BUFFER] = {
		.name = "Deep-Buffer Audio Port",
		.stream_name = "Deep-Buffer Audio",
		.cpu_dai_name = "deepbuffer-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-mfld-platform",
		.nonatomic = true,
		.dynamic = 1,
		.dpcm_playback = 1,
		.ops = &cht_aif1_ops,
	},
	/* CODEC<->CODEC link */
	/* back ends */
	{
		.name = "SSP2-Codec",
		.id = 0,
		.cpu_dai_name = "ssp2-port",
		.platform_name = "sst-mfld-platform",
		.no_pcm = 1,
		.codec_dai_name = "rt5645-aif1",
		.codec_name = "i2c-10EC5645:00",
		.init = cht_codec_init,
		.be_hw_params_fixup = cht_codec_fixup,
		.nonatomic = true,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ops = &cht_be_ssp2_ops,
	},
};

/* SoC card */
static struct snd_soc_card snd_soc_card_chtrt5645 = {
	.name = "chtrt5645",
	.owner = THIS_MODULE,
	.dai_link = cht_dailink,
	.num_links = ARRAY_SIZE(cht_dailink),
	.dapm_widgets = cht_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cht_dapm_widgets),
	.dapm_routes = cht_rt5645_audio_map,
	.num_dapm_routes = ARRAY_SIZE(cht_rt5645_audio_map),
	.controls = cht_mc_controls,
	.num_controls = ARRAY_SIZE(cht_mc_controls),
};

static struct snd_soc_card snd_soc_card_chtrt5650 = {
	.name = "chtrt5650",
	.owner = THIS_MODULE,
	.dai_link = cht_dailink,
	.num_links = ARRAY_SIZE(cht_dailink),
	.dapm_widgets = cht_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cht_dapm_widgets),
	.dapm_routes = cht_rt5650_audio_map,
	.num_dapm_routes = ARRAY_SIZE(cht_rt5650_audio_map),
	.controls = cht_mc_controls,
	.num_controls = ARRAY_SIZE(cht_mc_controls),
};

static struct cht_acpi_card snd_soc_cards[] = {
	{"10EC5640", CODEC_TYPE_RT5645, &snd_soc_card_chtrt5645},
	{"10EC5645", CODEC_TYPE_RT5645, &snd_soc_card_chtrt5645},
	{"10EC5648", CODEC_TYPE_RT5645, &snd_soc_card_chtrt5645},
	{"10EC3270", CODEC_TYPE_RT5645, &snd_soc_card_chtrt5645},
	{"10EC5650", CODEC_TYPE_RT5650, &snd_soc_card_chtrt5650},
};

static char cht_rt5645_codec_name[SND_ACPI_I2C_ID_LEN];
static char cht_rt5645_codec_aif_name[12]; /*  = "rt5645-aif[1|2]" */
static char cht_rt5645_cpu_dai_name[10]; /*  = "ssp[0|2]-port" */

static bool is_valleyview(void)
{
	static const struct x86_cpu_id cpu_ids[] = {
		{ X86_VENDOR_INTEL, 6, 55 }, /* Valleyview, Bay Trail */
		{}
	};

	if (!x86_match_cpu(cpu_ids))
		return false;
	return true;
}

struct acpi_chan_package {   /* ACPICA seems to require 64 bit integers */
	u64 aif_value;       /* 1: AIF1, 2: AIF2 */
	u64 mclock_value;    /* usually 25MHz (0x17d7940), ignored */
};

static int snd_cht_mc_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = snd_soc_cards[0].soc_card;
	struct snd_soc_acpi_mach *mach;
	struct cht_mc_private *drv;
	const char *i2c_name = NULL;
	bool found = false;
	bool is_bytcr = false;
	int dai_index = 0;
	int ret_val = 0;
	int i;

	drv = devm_kzalloc(&pdev->dev, sizeof(*drv), GFP_ATOMIC);
	if (!drv)
		return -ENOMEM;

	mach = (&pdev->dev)->platform_data;

	for (i = 0; i < ARRAY_SIZE(snd_soc_cards); i++) {
		if (acpi_dev_found(snd_soc_cards[i].codec_id) &&
			(!strncmp(snd_soc_cards[i].codec_id, mach->id, 8))) {
			dev_dbg(&pdev->dev,
				"found codec %s\n", snd_soc_cards[i].codec_id);
			card = snd_soc_cards[i].soc_card;
			drv->acpi_card = &snd_soc_cards[i];
			found = true;
			break;
		}
	}

	if (!found) {
		dev_err(&pdev->dev, "No matching HID found in supported list\n");
		return -ENODEV;
	}

	card->dev = &pdev->dev;
	sprintf(drv->codec_name, "i2c-%s:00", drv->acpi_card->codec_id);

	/* set correct codec name */
	for (i = 0; i < ARRAY_SIZE(cht_dailink); i++)
		if (!strcmp(card->dai_link[i].codec_name, "i2c-10EC5645:00")) {
			card->dai_link[i].codec_name = drv->codec_name;
			dai_index = i;
		}

	/* fixup codec name based on HID */
	i2c_name = acpi_dev_get_first_match_name(mach->id, NULL, -1);
	if (i2c_name) {
		snprintf(cht_rt5645_codec_name, sizeof(cht_rt5645_codec_name),
			"%s%s", "i2c-", i2c_name);
		cht_dailink[dai_index].codec_name = cht_rt5645_codec_name;
	}

	/*
	 * swap SSP0 if bytcr is detected
	 * (will be overridden if DMI quirk is detected)
	 */
	if (is_valleyview()) {
		struct sst_platform_info *p_info = mach->pdata;
		const struct sst_res_info *res_info = p_info->res_info;

		if (res_info->acpi_ipc_irq_index == 0)
			is_bytcr = true;
	}

	if (is_bytcr) {
		/*
		 * Baytrail CR platforms may have CHAN package in BIOS, try
		 * to find relevant routing quirk based as done on Windows
		 * platforms. We have to read the information directly from the
		 * BIOS, at this stage the card is not created and the links
		 * with the codec driver/pdata are non-existent
		 */

		struct acpi_chan_package chan_package;

		/* format specified: 2 64-bit integers */
		struct acpi_buffer format = {sizeof("NN"), "NN"};
		struct acpi_buffer state = {0, NULL};
		struct snd_soc_acpi_package_context pkg_ctx;
		bool pkg_found = false;

		state.length = sizeof(chan_package);
		state.pointer = &chan_package;

		pkg_ctx.name = "CHAN";
		pkg_ctx.length = 2;
		pkg_ctx.format = &format;
		pkg_ctx.state = &state;
		pkg_ctx.data_valid = false;

		pkg_found = snd_soc_acpi_find_package_from_hid(mach->id,
							       &pkg_ctx);
		if (pkg_found) {
			if (chan_package.aif_value == 1) {
				dev_info(&pdev->dev, "BIOS Routing: AIF1 connected\n");
				cht_rt5645_quirk |= CHT_RT5645_SSP0_AIF1;
			} else  if (chan_package.aif_value == 2) {
				dev_info(&pdev->dev, "BIOS Routing: AIF2 connected\n");
				cht_rt5645_quirk |= CHT_RT5645_SSP0_AIF2;
			} else {
				dev_info(&pdev->dev, "BIOS Routing isn't valid, ignored\n");
				pkg_found = false;
			}
		}

		if (!pkg_found) {
			/* no BIOS indications, assume SSP0-AIF2 connection */
			cht_rt5645_quirk |= CHT_RT5645_SSP0_AIF2;
		}
	}

	/* check quirks before creating card */
	dmi_check_system(cht_rt5645_quirk_table);
	log_quirks(&pdev->dev);

	if ((cht_rt5645_quirk & CHT_RT5645_SSP2_AIF2) ||
		(cht_rt5645_quirk & CHT_RT5645_SSP0_AIF2)) {

		/* fixup codec aif name */
		snprintf(cht_rt5645_codec_aif_name,
			sizeof(cht_rt5645_codec_aif_name),
			"%s", "rt5645-aif2");

		cht_dailink[dai_index].codec_dai_name =
			cht_rt5645_codec_aif_name;
	}

	if ((cht_rt5645_quirk & CHT_RT5645_SSP0_AIF1) ||
		(cht_rt5645_quirk & CHT_RT5645_SSP0_AIF2)) {

		/* fixup cpu dai name name */
		snprintf(cht_rt5645_cpu_dai_name,
			sizeof(cht_rt5645_cpu_dai_name),
			"%s", "ssp0-port");

		cht_dailink[dai_index].cpu_dai_name =
			cht_rt5645_cpu_dai_name;
	}

	drv->mclk = devm_clk_get(&pdev->dev, "pmc_plt_clk_3");
	if (IS_ERR(drv->mclk)) {
		dev_err(&pdev->dev,
			"Failed to get MCLK from pmc_plt_clk_3: %ld\n",
			PTR_ERR(drv->mclk));
		return PTR_ERR(drv->mclk);
	}

	snd_soc_card_set_drvdata(card, drv);
	ret_val = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret_val) {
		dev_err(&pdev->dev,
			"snd_soc_register_card failed %d\n", ret_val);
		return ret_val;
	}
	platform_set_drvdata(pdev, card);
	return ret_val;
}

static struct platform_driver snd_cht_mc_driver = {
	.driver = {
		.name = "cht-bsw-rt5645",
	},
	.probe = snd_cht_mc_probe,
};

module_platform_driver(snd_cht_mc_driver)

MODULE_DESCRIPTION("ASoC Intel(R) Braswell Machine driver");
MODULE_AUTHOR("Fang, Yang A,N,Harshapriya");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:cht-bsw-rt5645");
