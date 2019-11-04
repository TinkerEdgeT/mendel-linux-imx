/*
 * imx8mm ASOC Driver for the Maxim 98357a chip
 *
 * Author: Cindy Liu <hcindyl@google.com>
 *         Copyright 2019
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/control.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <linux/pinctrl/consumer.h>
#include <linux/mfd/syscon.h>

struct imx_priv {
  struct platform_device *pdev;
  struct snd_soc_card card;
  struct clk *codec_clk;
  unsigned int clk_frequency;
};

static const struct snd_soc_dapm_widget imx_max98357a_dapm_widgets[] = {
  SND_SOC_DAPM_SPK("Speakers", NULL),
};

static const struct snd_soc_dapm_route audio_routes[] = {
  {"Speakers", NULL, "Speaker"},
};

static const struct snd_kcontrol_new card_controls[] = {
  SOC_DAPM_PIN_SWITCH("Speaker"),
};

static int imx_hifi_hw_params(struct snd_pcm_substream *substream,
    struct snd_pcm_hw_params *params) {
  struct snd_soc_pcm_runtime *rtd = substream->private_data;
  struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
  struct snd_soc_card *card = rtd->card;
  struct device *dev = card->dev;
  unsigned int fmt;
  int ret = 0;

  /* max98357a supports these sample rates */
  switch (params_rate(params)) {
    case 8000:
    case 16000:
    case 48000:
    case 96000:
      break;
    default:
      dev_err(rtd->card->dev, "%s() doesn't support this sample rate: %d\n",
          __func__, params_rate(params));
      return -EINVAL;
  }

  /* Set max98375a as slave */
  fmt = SND_SOC_DAIFMT_I2S |
      SND_SOC_DAIFMT_NB_NF |
      SND_SOC_DAIFMT_CBS_CFS;

  ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
  if (ret) {
    dev_err(dev, "failed to set cpu dai fmt: %d\n", ret);
    return ret;
  }

  ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0, 0, 2,
            params_physical_width(params));
  if (ret) {
    dev_err(dev, "failed to set cpu dai tdm slot: %d\n", ret);
    return ret;
  }
  return ret;
}

static struct snd_soc_ops imx_hifi_ops = {
  .hw_params = imx_hifi_hw_params,
};

static struct snd_soc_dai_link imx_max98357a_dai[] = {
  {
    .name = "MAX98357A",
    .stream_name = "MAX98357A",
    .codec_dai_name = "HiFi",
    .ops = &imx_hifi_ops,
  },
};

static int imx_max98357a_late_probe(struct snd_soc_card *card)
{
  struct snd_soc_pcm_runtime *rtd = list_first_entry(
    &card->rtd_list, struct snd_soc_pcm_runtime, list);
  struct snd_soc_dai *codec_dai = rtd->codec_dai;
  struct imx_priv *priv = snd_soc_card_get_drvdata(card);
  int ret;

  priv->clk_frequency = clk_get_rate(priv->codec_clk);

  ret = snd_soc_dai_set_sysclk(codec_dai, 0, priv->clk_frequency,
            SND_SOC_CLOCK_IN);
  dev_info(rtd->card->dev, "%s() sets sysclk to %u\n", __func__,
      priv->clk_frequency);

  return 0;
}

static int imx_max98357a_probe(struct platform_device *pdev)
{
  struct device_node *cpu_np, *codec_np = NULL;
  struct platform_device *cpu_pdev;
  struct imx_priv *priv;
  struct platform_device *codec_pdev = NULL;
  int ret;

  priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
  if (!priv)
    return -ENOMEM;

  priv->pdev = pdev;

  cpu_np = of_parse_phandle(pdev->dev.of_node, "audio-cpu", 0);
  if (!cpu_np) {
    dev_err(&pdev->dev, "cpu dai phandle missing or invalid\n");
    ret = -EINVAL;
    goto fail;
  }

  codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
  if (!codec_np) {
    dev_err(&pdev->dev, "phandle missing or invalid\n");
    ret = -EINVAL;
    goto fail;
  }

  cpu_pdev = of_find_device_by_node(cpu_np);
  if (!cpu_pdev) {
    dev_err(&pdev->dev, "failed to find SAI platform device\n");
    ret = -EINVAL;
    goto fail;
  }

  codec_pdev = of_find_device_by_node(codec_np);
  if (!codec_pdev || !codec_pdev->dev.driver) {
    dev_err(&pdev->dev, "failed to find codec platform device\n");
    ret = -EINVAL;
    goto fail;
  }

  priv->codec_clk = devm_clk_get(&cpu_pdev->dev, "mclk1");
  if (IS_ERR(priv->codec_clk)) {
    ret = PTR_ERR(priv->codec_clk);
    dev_err(&pdev->dev, "failed to get codec clk: %d\n", ret);
    goto fail;
  }

  priv->card.dai_link = imx_max98357a_dai;

  imx_max98357a_dai[0].codec_of_node  = codec_np;
  imx_max98357a_dai[0].cpu_dai_name = dev_name(&cpu_pdev->dev);
  imx_max98357a_dai[0].platform_of_node = cpu_np;
  imx_max98357a_dai[0].playback_only  = 1;

  priv->card.late_probe = imx_max98357a_late_probe;
  priv->card.num_links = 1;
  priv->card.dev = &pdev->dev;
  priv->card.owner = THIS_MODULE;
  priv->card.dapm_widgets = imx_max98357a_dapm_widgets;
  priv->card.num_dapm_widgets = ARRAY_SIZE(imx_max98357a_dapm_widgets);
  priv->card.controls = card_controls;
  priv->card.num_controls = ARRAY_SIZE(card_controls);
  priv->card.dapm_routes = audio_routes,
  priv->card.num_dapm_routes = ARRAY_SIZE(audio_routes);

  ret = snd_soc_of_parse_card_name(&priv->card, "model");
  if (ret)
    goto fail;

  snd_soc_card_set_drvdata(&priv->card, priv);

  ret = devm_snd_soc_register_card(&pdev->dev, &priv->card);
  if (ret) {
    dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
    goto fail;
  }
  dev_info(&pdev->dev, "snd_soc_register_card successed.\n");

  ret = 0;
fail:
  if (cpu_np)
    of_node_put(cpu_np);
  if (codec_np)
    of_node_put(codec_np);

  return ret;
}

static const struct of_device_id imx_max98357a_dt_ids[] = {
  { .compatible = "fsl,imx-audio-max98357a", },
  { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_max98357a_dt_ids);

static struct platform_driver imx_max98357a_driver = {
  .driver = {
    .name = "imx-max98357a",
    .pm = &snd_soc_pm_ops,
    .of_match_table = imx_max98357a_dt_ids,
  },
  .probe = imx_max98357a_probe,
};
module_platform_driver(imx_max98357a_driver);

MODULE_AUTHOR("Cindy Liu <hcindyl@google.com>");
MODULE_DESCRIPTION("i.MX MAX98357a ASoC machine driver");
MODULE_LICENSE("GPL v2");
