// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * J71b MCA2 digital-link qualification. Amplifiers remain in reset.
 * This is not a speaker codec or an implementation of speaker protection.
 */
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

struct j71b_digital {
	struct snd_soc_card card;
	struct snd_soc_dai_link link;
	struct snd_soc_dai_link_component cpu, platform;
	struct gpio_desc *reset;
};

static int j71b_hold_reset(struct j71b_digital *priv)
{
	int value, ret;

	/* Probe requires a non-sleeping GPIO, so this is safe in trigger too. */
	ret = gpiod_set_value(priv->reset, 1);
	if (ret)
		return ret;
	value = gpiod_get_value(priv->reset);
	if (value < 0)
		return value;
	return value == 1 ? 0 : -EIO;
}

static int j71b_reset_init(struct j71b_digital *priv)
{
	int ret;

	/* Validate polarity before any output write: high would enable the amp. */
	if (!gpiod_is_active_low(priv->reset) || gpiod_cansleep(priv->reset))
		return -EINVAL;
	ret = gpiod_direction_output(priv->reset, 1);
	if (ret)
		return ret;
	return j71b_hold_reset(priv);
}

static struct j71b_digital *j71b_substream_priv(struct snd_pcm_substream *s)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(s);

	return snd_soc_card_get_drvdata(rtd->card);
}

static int j71b_startup(struct snd_pcm_substream *substream)
{
	int ret;

	ret = j71b_hold_reset(j71b_substream_priv(substream));
	if (ret)
		return ret;
	ret = snd_pcm_hw_constraint_single(substream->runtime,
					   SNDRV_PCM_HW_PARAM_CHANNELS, 4);
	if (ret)
		return ret;
	ret = snd_pcm_hw_constraint_single(substream->runtime,
					   SNDRV_PCM_HW_PARAM_RATE, 48000);
	if (ret)
		return ret;
	return snd_pcm_hw_constraint_mask64(substream->runtime,
					    SNDRV_PCM_HW_PARAM_FORMAT,
					    SNDRV_PCM_FMTBIT_S16_LE);
}

static int j71b_prepare(struct snd_pcm_substream *substream)
{
	return j71b_hold_reset(j71b_substream_priv(substream));
}

static void j71b_shutdown_stream(struct snd_pcm_substream *substream)
{
	struct j71b_digital *priv = j71b_substream_priv(substream);

	if (j71b_hold_reset(priv))
		dev_err(priv->card.dev, "cannot confirm amplifier reset on close\n");
}

static int j71b_hw_params(struct snd_pcm_substream *substream,
			  struct snd_pcm_hw_params *params)
{
	if (params_format(params) != SNDRV_PCM_FORMAT_S16_LE ||
	    params_channels(params) != 4 ||
	    params_rate(params) != 48000)
		return -EINVAL;
	return j71b_hold_reset(j71b_substream_priv(substream));
}

static int j71b_trigger(struct snd_pcm_substream *substream, int cmd)
{
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		return j71b_hold_reset(j71b_substream_priv(substream));
	default:
		return -EINVAL;
	}
}

static int j71b_link_init(struct snd_soc_pcm_runtime *rtd)
{
	int ret;

	ret = j71b_hold_reset(snd_soc_card_get_drvdata(rtd->card));
	if (ret)
		return ret;
	/* Do not translate the recovered electrical profile into guessed fmt. */
	return snd_soc_dai_set_tdm_slot(snd_soc_rtd_to_cpu(rtd, 0), 0xf, 0, 8, 16);
}

static void j71b_link_exit(struct snd_soc_pcm_runtime *rtd)
{
	int ret;

	ret = j71b_hold_reset(snd_soc_card_get_drvdata(rtd->card));
	if (ret)
		dev_err(rtd->card->dev, "cannot confirm amplifier reset on unlink\n");
	ret = snd_soc_dai_set_tdm_slot(snd_soc_rtd_to_cpu(rtd, 0), 0, 0, 0, 0);
	if (ret)
		dev_err(rtd->card->dev, "cannot release MCA reference profile: %d\n", ret);
}

static int j71b_card_hold_reset(struct snd_soc_card *card)
{
	return j71b_hold_reset(snd_soc_card_get_drvdata(card));
}

/* No physical speaker positions have been established for these slots. */
static const struct snd_pcm_chmap_elem j71b_slot_map[] = {
	{ .channels = 4,
	  .map = { SNDRV_CHMAP_UNKNOWN, SNDRV_CHMAP_UNKNOWN,
		   SNDRV_CHMAP_UNKNOWN, SNDRV_CHMAP_UNKNOWN } },
	{}
};

static int j71b_late_probe(struct snd_soc_card *card)
{
	struct snd_soc_pcm_runtime *rtd;

	for_each_card_rtds(card, rtd)
		return snd_pcm_add_chmap_ctls(rtd->pcm, SNDRV_PCM_STREAM_PLAYBACK,
					    j71b_slot_map, 4, 0, NULL);
	return -ENODEV;
}

static const struct snd_soc_ops j71b_ops = {
	.startup = j71b_startup,
	.shutdown = j71b_shutdown_stream,
	.hw_params = j71b_hw_params,
	.prepare = j71b_prepare,
	.trigger = j71b_trigger,
};

static void j71b_release_node(void *data)
{
	of_node_put(data);
}

static void j71b_reset_on_release(void *data)
{
	struct j71b_digital *priv = data;

	if (j71b_hold_reset(priv))
		dev_err(priv->card.dev, "cannot confirm amplifier reset on release\n");
}

static int j71b_digital_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct j71b_digital *priv;
	struct device_node *cpu;
	int ret;

	if (!of_machine_is_compatible("apple,j71b"))
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->card.dev = dev;
	/* Mandatory, exclusive ownership; logical assertion means physical low. */
	priv->reset = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(priv->reset))
		return dev_err_probe(dev, PTR_ERR(priv->reset),
				     "failed to hold amplifiers in reset\n");
	ret = j71b_reset_init(priv);
	if (ret)
		return dev_err_probe(dev, ret, "amplifier reset readback failed\n");
	ret = devm_add_action_or_reset(dev, j71b_reset_on_release, priv);
	if (ret)
		return ret;

	cpu = of_parse_phandle(dev->of_node, "audio-cpu", 0);
	if (!cpu)
		return -EINVAL;
	if (!of_device_is_compatible(cpu, "apple,j71b-mca")) {
		of_node_put(cpu);
		return -EINVAL;
	}
	ret = devm_add_action_or_reset(dev, j71b_release_node, cpu);
	if (ret)
		return ret;

	priv->cpu.of_node = cpu;
	priv->platform.of_node = cpu;
	priv->link.name = "MCA2 Digital";
	priv->link.stream_name = "MCA2 Slots 0-3 (Amplifiers Reset)";
	priv->link.cpus = &priv->cpu;
	priv->link.num_cpus = 1;
	priv->link.platforms = &priv->platform;
	priv->link.num_platforms = 1;
	/* Intentional passive digital sink, not a stand-in for a working amp. */
	priv->link.codecs = &snd_soc_dummy_dlc;
	priv->link.num_codecs = 1;
	priv->link.playback_only = 1;
	priv->link.init = j71b_link_init;
	priv->link.exit = j71b_link_exit;
	priv->link.ops = &j71b_ops;
	/* dai_fmt remains zero: the CPU DAI has an explicit fixed profile. */
	priv->card.name = "J71b Digital Qualification";
	priv->card.driver_name = "J71bDigital";
	priv->card.owner = THIS_MODULE;
	priv->card.dai_link = &priv->link;
	priv->card.num_links = 1;
	priv->card.late_probe = j71b_late_probe;
	snd_soc_card_set_drvdata(&priv->card, priv);
	platform_set_drvdata(pdev, &priv->card);

	return devm_snd_soc_register_card(dev, &priv->card);
}

static int j71b_digital_suspend(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	int ret;

	ret = j71b_card_hold_reset(card);
	return ret ? ret : snd_soc_suspend(dev);
}

static int j71b_digital_resume(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	int ret;

	ret = j71b_reset_init(snd_soc_card_get_drvdata(card));
	return ret ? ret : snd_soc_resume(dev);
}

static int j71b_digital_poweroff(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	int ret;

	ret = j71b_card_hold_reset(card);
	return ret ? ret : snd_soc_poweroff(dev);
}

static const struct dev_pm_ops j71b_digital_pm_ops = {
	.suspend = j71b_digital_suspend,
	.resume = j71b_digital_resume,
	.freeze = j71b_digital_suspend,
	.thaw = j71b_digital_resume,
	.poweroff = j71b_digital_poweroff,
	.restore = j71b_digital_resume,
};

static void j71b_digital_shutdown(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	if (j71b_card_hold_reset(card))
		dev_err(&pdev->dev, "cannot confirm amplifier reset at shutdown\n");
	snd_soc_poweroff(&pdev->dev);
}

static const struct of_device_id j71b_digital_match[] = {
	{ .compatible = "apple,j71b-audio" },
	{}
};
MODULE_DEVICE_TABLE(of, j71b_digital_match);

static struct platform_driver j71b_digital_driver = {
	.probe = j71b_digital_probe,
	.shutdown = j71b_digital_shutdown,
	.driver = {
		.name = "j71b-digital-audio",
		.of_match_table = j71b_digital_match,
		.pm = &j71b_digital_pm_ops,
	},
};
module_platform_driver(j71b_digital_driver);

MODULE_DESCRIPTION("J71b fixed-profile digital audio link with amplifiers held in reset");
MODULE_AUTHOR("Codex <codex@openai.com>");
MODULE_LICENSE("Dual MIT/GPL");
