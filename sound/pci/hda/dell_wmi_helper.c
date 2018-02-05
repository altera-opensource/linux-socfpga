// SPDX-License-Identifier: GPL-2.0
/* Helper functions for Dell Mic Mute LED control;
 * to be included from codec driver
 */

#if IS_ENABLED(CONFIG_DELL_LAPTOP)
#include <linux/dell-led.h>

enum {
	MICMUTE_LED_ON,
	MICMUTE_LED_OFF,
	MICMUTE_LED_FOLLOW_CAPTURE,
	MICMUTE_LED_FOLLOW_MUTE,
};

static int dell_led_mode = MICMUTE_LED_FOLLOW_MUTE;
static int dell_capture;
static int dell_led_value;
static int (*dell_micmute_led_set_func)(int);
static void (*dell_old_cap_hook)(struct hda_codec *,
			         struct snd_kcontrol *,
				 struct snd_ctl_elem_value *);

static void call_micmute_led_update(void)
{
	int val;

	switch (dell_led_mode) {
	case MICMUTE_LED_ON:
		val = 1;
		break;
	case MICMUTE_LED_OFF:
		val = 0;
		break;
	case MICMUTE_LED_FOLLOW_CAPTURE:
		val = dell_capture;
		break;
	case MICMUTE_LED_FOLLOW_MUTE:
	default:
		val = !dell_capture;
		break;
	}

	if (val == dell_led_value)
		return;
	dell_led_value = val;
	dell_micmute_led_set_func(dell_led_value);
}

static void update_dell_wmi_micmute_led(struct hda_codec *codec,
				        struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	if (dell_old_cap_hook)
		dell_old_cap_hook(codec, kcontrol, ucontrol);

	if (!ucontrol || !dell_micmute_led_set_func)
		return;
	if (strcmp("Capture Switch", ucontrol->id.name) == 0 && ucontrol->id.index == 0) {
		/* TODO: How do I verify if it's a mono or stereo here? */
		dell_capture = (ucontrol->value.integer.value[0] ||
				ucontrol->value.integer.value[1]);
		call_micmute_led_update();
	}
}

static int dell_mic_mute_led_mode_info(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_info *uinfo)
{
	static const char * const texts[] = {
		"On", "Off", "Follow Capture", "Follow Mute",
	};

	return snd_ctl_enum_info(uinfo, 1, ARRAY_SIZE(texts), texts);
}

static int dell_mic_mute_led_mode_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = dell_led_mode;
	return 0;
}

static int dell_mic_mute_led_mode_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	unsigned int mode;

	mode = ucontrol->value.enumerated.item[0];
	if (mode > MICMUTE_LED_FOLLOW_MUTE)
		mode = MICMUTE_LED_FOLLOW_MUTE;
	if (mode == dell_led_mode)
		return 0;
	dell_led_mode = mode;
	call_micmute_led_update();
	return 1;
}

static const struct snd_kcontrol_new dell_mic_mute_mode_ctls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Mic Mute-LED Mode",
		.info = dell_mic_mute_led_mode_info,
		.get = dell_mic_mute_led_mode_get,
		.put = dell_mic_mute_led_mode_put,
	},
	{}
};

static void alc_fixup_dell_wmi(struct hda_codec *codec,
			       const struct hda_fixup *fix, int action)
{
	struct alc_spec *spec = codec->spec;
	bool removefunc = false;

	if (action == HDA_FIXUP_ACT_PROBE) {
		if (!dell_micmute_led_set_func)
			dell_micmute_led_set_func = symbol_request(dell_micmute_led_set);
		if (!dell_micmute_led_set_func) {
			codec_warn(codec, "Failed to find dell wmi symbol dell_micmute_led_set\n");
			return;
		}

		removefunc = true;
		if (dell_micmute_led_set_func(false) >= 0) {
			dell_led_value = 0;
			if (spec->gen.num_adc_nids > 1 && !spec->gen.dyn_adc_switch)
				codec_dbg(codec, "Skipping micmute LED control due to several ADCs");
			else {
				dell_old_cap_hook = spec->gen.cap_sync_hook;
				spec->gen.cap_sync_hook = update_dell_wmi_micmute_led;
				removefunc = false;
				add_mixer(spec, dell_mic_mute_mode_ctls);
			}
		}

	}

	if (dell_micmute_led_set_func && (action == HDA_FIXUP_ACT_FREE || removefunc)) {
		symbol_put(dell_micmute_led_set);
		dell_micmute_led_set_func = NULL;
		dell_old_cap_hook = NULL;
	}
}

#else /* CONFIG_DELL_LAPTOP */
static void alc_fixup_dell_wmi(struct hda_codec *codec,
			       const struct hda_fixup *fix, int action)
{
}

#endif /* CONFIG_DELL_LAPTOP */
