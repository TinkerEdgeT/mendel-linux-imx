#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>

static int hwid;

static const struct of_device_id of_gpio_hwid_match[] = {
	{ .compatible = "gpio-hwid", },
	{},
};
MODULE_DEVICE_TABLE(of, of_gpio_hwid_match);

static int set_hwid(int id2_pin, int id1_pin, int id0_pin)
{
	int id;
	id = (id2_pin << 2) + (id1_pin << 1) + id0_pin;
	return id;
}

static ssize_t hwid_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", hwid);
}

static DEVICE_ATTR(HWID, S_IRUGO, hwid_show, NULL);

static int gpio_hwid_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int id0_gpio, id1_gpio, id2_gpio;
	int id0, id1, id2;
	int ret;

	id0_gpio = of_get_named_gpio(dev->of_node, "id0-gpios", 0);
	if (!gpio_is_valid(id0_gpio)) {
		printk("No id0-gpio pin available in gpio-hwid\n");
		return 1;
	} else {
		id0 = devm_gpio_request_one(dev, id0_gpio, GPIOF_DIR_IN, "GPIO_ID0");
		if (id0 < 0)
			printk("Fail to set id0 pin\n");
	}

	id1_gpio = of_get_named_gpio(dev->of_node, "id1-gpios", 0);
	if (!gpio_is_valid(id1_gpio)) {
		printk("No id1-gpio pin available in gpio-hwid\n");
		return 1;
	} else {
		id1 = devm_gpio_request_one(dev, id1_gpio, GPIOF_DIR_IN, "GPIO_ID1");
		if (id1 < 0)
			printk("Fail to set id1 pin\n");
	}

	id2_gpio = of_get_named_gpio(dev->of_node, "id2-gpios", 0);
	if (!gpio_is_valid(id2_gpio)) {
		printk("No id2-gpio pin available in gpio-hwid\n");
		return 1;
	} else {
		id2 = devm_gpio_request_one(dev, id2_gpio, GPIOF_DIR_IN, "GPIO_ID2");
		if (id2 < 0)
			printk("Fail to set id2 pin\n");
	}

	hwid = set_hwid(id2, id1, id0);
	printk("HWID = %d\n", hwid);

	ret = device_create_file(dev, &dev_attr_HWID);
	if (ret) {
		printk("Cannot create HWID attribute\n");
		device_remove_file(dev, &dev_attr_HWID);
		return 1;
	}

	return 0;
}

static struct platform_driver gpio_hwid_driver = {
	.probe          = gpio_hwid_probe,
	.driver = {
		.name   = "gpio-hwid",
#ifdef CONFIG_OF_GPIO
		.of_match_table = of_match_ptr(of_gpio_hwid_match),
#endif
	},
};

module_platform_driver(gpio_hwid_driver);
