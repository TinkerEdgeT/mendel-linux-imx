#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

struct akira_priv {
	struct gpio_desc *power_gpio;
	struct gpio_desc *reset_gpio;
};

static int akira_probe(struct platform_device *pdev)
{
	struct akira_priv *priv =
		devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	platform_set_drvdata(pdev, priv);
	priv->power_gpio =
		devm_gpiod_get_index(&pdev->dev, "tpu", 0, GPIOD_OUT_LOW);
	if (IS_ERR(priv->power_gpio)) {
		if (PTR_ERR(priv->power_gpio) != -EPROBE_DEFER) {
			dev_err(&pdev->dev, "Could not get power gpio");
		}
		return PTR_ERR(priv->power_gpio);
	}

	priv->reset_gpio =
		devm_gpiod_get_index(&pdev->dev, "tpu", 1, GPIOD_OUT_LOW);
	if (IS_ERR(priv->reset_gpio)) {
		if (PTR_ERR(priv->reset_gpio) != -EPROBE_DEFER) {
			dev_err(&pdev->dev, "Could not get reset gpio");
		}
		return PTR_ERR(priv->reset_gpio);
	}

	gpiod_set_value_cansleep(priv->reset_gpio, true);
	gpiod_set_value_cansleep(priv->power_gpio, true);

	msleep(100);
	gpiod_set_value_cansleep(priv->reset_gpio, false);

	return 0;
}

static int akira_remove(struct platform_device *pdev)
{
	struct akira_priv *priv = platform_get_drvdata(pdev);
	gpiod_set_value_cansleep(priv->power_gpio, false);
	return 0;
}

static const struct of_device_id akira_of_id_table[] = {
	{ .compatible = "google,akira" }, {}
};
MODULE_DEVICE_TABLE(of, akira_of_id_table);

static struct platform_driver akira_driver = {
    .driver = {
        .name           = "akira",
        .of_match_table = akira_of_id_table,
    },
    .probe          = akira_probe,
    .remove         = akira_remove,
};
module_platform_driver(akira_driver);

MODULE_AUTHOR("Alex Van Damme <atv@google.com>");
MODULE_DESCRIPTION("Akira M.2 Driver");
MODULE_LICENSE("GPL v2");
