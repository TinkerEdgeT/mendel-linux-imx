// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the Microchip USB5744 4-port hub.
 *
 * Copyright (C) 2018 Google, Inc.
 */

#include <linux/kernel.h>
#include <linux/byteorder/generic.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>

#define USB_CFG_REGISTER 0x0000
#define USB_CFG_COMMIT_REGISTER 0x9937

struct usb5744 {
	struct gpio_desc *reset_gpio;
};

static int usb5744_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct device *dev = &client->dev;
	struct usb5744 *data =
		devm_kzalloc(dev, sizeof(struct usb5744), GFP_KERNEL);

	i2c_set_clientdata(client, data);

	data->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(data->reset_gpio)) {
		dev_err(dev, "Failed to bind reset gpio");
		return -ENODEV;
	}

	// Toggle RESET_N to reset the hub.
	gpiod_set_value(data->reset_gpio, 1);
	usleep_range(5, 20);
	gpiod_set_value(data->reset_gpio, 0);
	msleep(5);

	// Send SMBus command to boot hub.
	ret = i2c_smbus_write_word_data(client, 0xAA, htons(0x5600));
	if (ret < 0) {
		dev_err(dev, "Sending boot command failed");
		return ret;
	}

	return 0;
}

static const struct i2c_device_id usb5744_id[] = {
	{ "usb5744", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, usb5744_id);

static struct i2c_driver usb5744_driver = {
	.driver = {
		.name = "usb5744",
	},
	.probe = usb5744_probe,
	.id_table = usb5744_id,
};

module_i2c_driver(usb5744_driver);

MODULE_AUTHOR("Alex Van Damme <atv@google.com>");
MODULE_DESCRIPTION("USB5744 Hub");
MODULE_LICENSE("GPL v2");
