// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2010-2011,2013-2015 The Linux Foundation. All rights reserved.
 *
 * max98357a.c -- MAX98357A ALSA SoC Codec driver
 */

#include <linux/acpi.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/hrtimer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/timekeeping.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/soc-dapm.h>

/* Diagnostic only: one window per module lifetime, never a volume protector. */
#define J71B_TEST_WINDOW_MS 30
static bool j71b_test_arm;
static atomic_t j71b_test_used = ATOMIC_INIT(0);
#if IS_ENABLED(CONFIG_SND_SOC_MAX98357A_J71B_TEST)
module_param_named(j71b_test_arm, j71b_test_arm, bool, 0400);
MODULE_PARM_DESC(j71b_test_arm,
		"Permit one 30 ms J71b amplifier test window (default: false)");
#endif

struct max98357a_priv {
	struct gpio_desc *sdmode;
	unsigned int sdmode_delay;
	int sdmode_switch;
	bool test_gate;
	bool test_terminal;
	raw_spinlock_t test_lock;
	struct hrtimer test_timer;
};

/* Called with test_lock held. Do not stop BCLK/LRCLK to mute the amplifier. */
static void max98357a_test_mute(struct max98357a_priv *max98357a)
{
	gpiod_set_value(max98357a->sdmode, 0);
}

static enum hrtimer_restart max98357a_test_expire(struct hrtimer *timer)
{
	struct max98357a_priv *max98357a =
		container_of(timer, struct max98357a_priv, test_timer);
	unsigned long flags;

	raw_spin_lock_irqsave(&max98357a->test_lock, flags);
	max98357a_test_mute(max98357a);
	raw_spin_unlock_irqrestore(&max98357a->test_lock, flags);
	return HRTIMER_NORESTART;
}

static int max98357a_test_trigger(struct max98357a_priv *max98357a, int cmd)
{
	ktime_t deadline;
	unsigned long flags;
	int ret = 0;

	raw_spin_lock_irqsave(&max98357a->test_lock, flags);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		/* No automatic resume, retry, rebind or second-stream rearming. */
		if (max98357a->test_terminal || !j71b_test_arm ||
		    !max98357a->sdmode_switch ||
		    atomic_cmpxchg(&j71b_test_used, 0, 1)) {
			max98357a_test_mute(max98357a);
			ret = -EPERM;
			break;
		}
		/* Arm before unmute, with no sleeping GPIO or delay in this path. */
		deadline = ktime_add_ms(ktime_get(), J71B_TEST_WINDOW_MS);
		hrtimer_start(&max98357a->test_timer, deadline,
			      HRTIMER_MODE_ABS_HARD);
		if (ktime_compare(ktime_get(), deadline) >= 0) {
			max98357a_test_mute(max98357a);
			ret = -ETIME;
			break;
		}
		gpiod_set_value(max98357a->sdmode, 1);
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		max98357a_test_mute(max98357a);
		ret = -EPERM;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		max98357a_test_mute(max98357a);
		break;
	default:
		max98357a_test_mute(max98357a);
		ret = -EINVAL;
		break;
	}
	raw_spin_unlock_irqrestore(&max98357a->test_lock, flags);
	/* A pending callback may only mute; teardown synchronously drains it. */
	return ret;
}

/* Runs before devres releases the GPIO and after ASoC unregisters the codec. */
static void max98357a_test_quiesce(void *data)
{
	struct max98357a_priv *max98357a = data;
	unsigned long flags;

	if (!max98357a->test_gate)
		return;
	raw_spin_lock_irqsave(&max98357a->test_lock, flags);
	max98357a->test_terminal = true;
	max98357a_test_mute(max98357a);
	raw_spin_unlock_irqrestore(&max98357a->test_lock, flags);
	hrtimer_cancel(&max98357a->test_timer);
}

static int max98357a_test_init(struct device *dev,
			     struct max98357a_priv *max98357a)
{
	if (!IS_ENABLED(CONFIG_SND_SOC_MAX98357A_J71B_TEST) ||
	    !of_machine_is_compatible("apple,j71b"))
		return 0;

	/* Fail closed on this board if the diagnostic prerequisites differ. */
	if (!device_property_read_bool(dev, "maxim,tdm") ||
	    !max98357a->sdmode || gpiod_cansleep(max98357a->sdmode) ||
	    max98357a->sdmode_delay)
		return dev_err_probe(dev, -EINVAL,
				     "J71b test gate requires TDM, atomic SD_MODE and no delay\n");

	raw_spin_lock_init(&max98357a->test_lock);
	hrtimer_setup(&max98357a->test_timer, max98357a_test_expire,
		      CLOCK_MONOTONIC, HRTIMER_MODE_ABS_HARD);
	max98357a->test_gate = true;
	dev_notice(dev, "diagnostic J71b gate: one %u ms software window, armed=%u\n",
		   J71B_TEST_WINDOW_MS, j71b_test_arm);
	return devm_add_action_or_reset(dev, max98357a_test_quiesce, max98357a);
}

static int max98357a_daiops_trigger(struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct max98357a_priv *max98357a =
		snd_soc_component_get_drvdata(component);

	if (max98357a->test_gate)
		return max98357a_test_trigger(max98357a, cmd);

	if (!max98357a->sdmode)
		return 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		mdelay(max98357a->sdmode_delay);
		if (max98357a->sdmode_switch) {
			gpiod_set_value(max98357a->sdmode, 1);
			dev_dbg(component->dev, "set sdmode to 1");
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		gpiod_set_value(max98357a->sdmode, 0);
		dev_dbg(component->dev, "set sdmode to 0");
		break;
	}

	return 0;
}

static int max98357a_sdmode_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);
	struct max98357a_priv *max98357a =
		snd_soc_component_get_drvdata(component);

	if (max98357a->test_gate) {
		unsigned long flags;

		raw_spin_lock_irqsave(&max98357a->test_lock, flags);
		if (event & SND_SOC_DAPM_POST_PMU)
			max98357a->sdmode_switch = 1;
		if (event & SND_SOC_DAPM_POST_PMD) {
			max98357a->sdmode_switch = 0;
			max98357a_test_mute(max98357a);
		}
		raw_spin_unlock_irqrestore(&max98357a->test_lock, flags);
		return 0;
	}

	if (event & SND_SOC_DAPM_POST_PMU)
		max98357a->sdmode_switch = 1;
	else if (event & SND_SOC_DAPM_POST_PMD)
		max98357a->sdmode_switch = 0;

	return 0;
}

static const struct snd_soc_dapm_widget max98357a_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("Speaker"),
	SND_SOC_DAPM_OUT_DRV_E("SD_MODE", SND_SOC_NOPM, 0, 0, NULL, 0,
			max98357a_sdmode_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route max98357a_dapm_routes[] = {
	{"SD_MODE", NULL, "HiFi Playback"},
	{"Speaker", NULL, "SD_MODE"},
};

static const struct snd_soc_component_driver max98357a_component_driver = {
	.dapm_widgets		= max98357a_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(max98357a_dapm_widgets),
	.dapm_routes		= max98357a_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(max98357a_dapm_routes),
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static const struct snd_soc_dai_ops max98357a_dai_ops = {
	.trigger        = max98357a_daiops_trigger,
};

static struct snd_soc_dai_driver max98357a_dai_driver = {
	.name = "HiFi",
	.playback = {
		.stream_name	= "HiFi Playback",
		.formats	= SNDRV_PCM_FMTBIT_S16 |
					SNDRV_PCM_FMTBIT_S24 |
					SNDRV_PCM_FMTBIT_S32,
		.rates		= SNDRV_PCM_RATE_8000 |
					SNDRV_PCM_RATE_16000 |
					SNDRV_PCM_RATE_32000 |
					SNDRV_PCM_RATE_44100 |
					SNDRV_PCM_RATE_48000 |
					SNDRV_PCM_RATE_88200 |
					SNDRV_PCM_RATE_96000,
		.rate_min	= 8000,
		.rate_max	= 96000,
		.channels_min	= 1,
		.channels_max	= 2,
	},
	.ops    = &max98357a_dai_ops,
};

/* The amplifier picks its slot of the frame by SD_MODE and GAIN_SLOT. */
static struct snd_soc_dai_driver max98357a_tdm_dai_driver = {
	.name = "HiFi",
	.playback = {
		.stream_name	= "HiFi Playback",
		.formats	= SNDRV_PCM_FMTBIT_S16 |
					SNDRV_PCM_FMTBIT_S32,
		.rates		= SNDRV_PCM_RATE_8000 |
					SNDRV_PCM_RATE_16000 |
					SNDRV_PCM_RATE_32000 |
					SNDRV_PCM_RATE_44100 |
					SNDRV_PCM_RATE_48000 |
					SNDRV_PCM_RATE_88200 |
					SNDRV_PCM_RATE_96000,
		.rate_min	= 8000,
		.rate_max	= 96000,
		.channels_min	= 1,
		.channels_max	= 8,
	},
	.ops    = &max98357a_dai_ops,
};

static int max98357a_platform_probe(struct platform_device *pdev)
{
	struct snd_soc_dai_driver *dai_driver;
	struct max98357a_priv *max98357a;
	int ret;

	max98357a = devm_kzalloc(&pdev->dev, sizeof(*max98357a), GFP_KERNEL);
	if (!max98357a)
		return -ENOMEM;

	max98357a->sdmode = devm_gpiod_get_optional(&pdev->dev,
				"sdmode", GPIOD_OUT_LOW);
	if (IS_ERR(max98357a->sdmode))
		return PTR_ERR(max98357a->sdmode);

	ret = device_property_read_u32(&pdev->dev, "sdmode-delay",
					&max98357a->sdmode_delay);
	if (ret) {
		max98357a->sdmode_delay = 0;
		dev_dbg(&pdev->dev,
			"no optional property 'sdmode-delay' found, "
			"default: no delay\n");
	}

	ret = max98357a_test_init(&pdev->dev, max98357a);
	if (ret)
		return ret;

	dev_set_drvdata(&pdev->dev, max98357a);

	dai_driver = &max98357a_dai_driver;
	if (device_property_read_bool(&pdev->dev, "maxim,tdm"))
		dai_driver = &max98357a_tdm_dai_driver;

	return devm_snd_soc_register_component(&pdev->dev,
			&max98357a_component_driver,
			dai_driver, 1);
}

static void max98357a_platform_shutdown(struct platform_device *pdev)
{
	max98357a_test_quiesce(dev_get_drvdata(&pdev->dev));
}

#ifdef CONFIG_OF
static const struct of_device_id max98357a_device_id[] = {
	{ .compatible = "maxim,max98357a" },
	{ .compatible = "maxim,max98360a" },
	{}
};
MODULE_DEVICE_TABLE(of, max98357a_device_id);
#endif

#ifdef CONFIG_ACPI
static const struct acpi_device_id max98357a_acpi_match[] = {
	{ "MX98357A", 0 },
	{ "MX98360A", 0 },
	{},
};
MODULE_DEVICE_TABLE(acpi, max98357a_acpi_match);
#endif

static struct platform_driver max98357a_platform_driver = {
	.driver = {
		.name = "max98357a",
		.of_match_table = of_match_ptr(max98357a_device_id),
		.acpi_match_table = ACPI_PTR(max98357a_acpi_match),
	},
	.probe	= max98357a_platform_probe,
	.shutdown = max98357a_platform_shutdown,
};
module_platform_driver(max98357a_platform_driver);

MODULE_DESCRIPTION("Maxim MAX98357A Codec Driver");
MODULE_LICENSE("GPL v2");
