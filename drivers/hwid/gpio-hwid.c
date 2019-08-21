#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>

static int id0_gpio, id1_gpio, id2_gpio;

static const struct of_device_id of_gpio_hwid_match[] = {
	{ .compatible = "gpio-hwid", },
	{},
};
MODULE_DEVICE_TABLE(of, of_gpio_hwid_match);

static ssize_t hwid_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int id0, id1, id2;
	int hwid;

	id0 = gpio_get_value(id0_gpio);
	id1 = gpio_get_value(id1_gpio);
	id2 = gpio_get_value(id2_gpio);

	hwid = (id2 << 2) + (id1 << 1) + id0;

	return sprintf(buf, "%d\n", hwid);
}

static DEVICE_ATTR(HWID, S_IRUGO, hwid_show, NULL);

static int gpio_hwid_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	id0_gpio = of_get_named_gpio(dev->of_node, "id0-gpios", 0);
	if (!gpio_is_valid(id0_gpio)) {
		printk("No id0-gpio pin available in gpio-hwid\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, id0_gpio, GPIOF_DIR_IN, "GPIO_ID0");
		if (ret < 0) {
			printk("Failed to request ID0 gpio: %d\n", ret);
			return ret;
		}
	}

	id1_gpio = of_get_named_gpio(dev->of_node, "id1-gpios", 0);
	if (!gpio_is_valid(id1_gpio)) {
		printk("No id1-gpio pin available in gpio-hwid\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, id1_gpio, GPIOF_DIR_IN, "GPIO_ID1");
		if (ret < 0) {
			printk("Failed to request ID1 gpio: %d\n", ret);
			return ret;
		}
	}

	id2_gpio = of_get_named_gpio(dev->of_node, "id2-gpios", 0);
	if (!gpio_is_valid(id2_gpio)) {
		printk("No id2-gpio pin available in gpio-hwid\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, id2_gpio, GPIOF_DIR_IN, "GPIO_ID2");
		if (ret < 0) {
			printk("Failed to request ID2 gpio: %d\n", ret);
			return ret;
		}
	}

	ret = device_create_file(dev, &dev_attr_HWID);
	if (ret) {
		printk("Cannot create HWID attribute\n");
		device_remove_file(dev, &dev_attr_HWID);
		return -ENODEV;
	}

	return 0;
}

static int gpio_hwid_remove(struct platform_device *pdev)
{
	gpio_free(id0_gpio);
	gpio_free(id1_gpio);
	gpio_free(id2_gpio);

	return 0;
}

static struct platform_driver gpio_hwid_driver = {
	.probe		= gpio_hwid_probe,
	.remove		= gpio_hwid_remove,
	.driver = {
		.name	= "gpio-hwid",
#ifdef CONFIG_OF_GPIO
		.of_match_table = of_match_ptr(of_gpio_hwid_match),
#endif
	},
};

module_platform_driver(gpio_hwid_driver);
