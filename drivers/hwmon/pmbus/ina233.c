// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the TI INA233 current/power monitor.
 *
 * Copyright (C) 2018 Google, Inc.
 */

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include "pmbus.h"

#define CAL_FACTOR 5120000 // 1mA / LSB
#define MFR_CALIBRATION 0xD4

static struct pmbus_driver_info ina233_info = {
	.pages = 1,
	.format[PSC_VOLTAGE_IN] = direct,
	.format[PSC_VOLTAGE_OUT] = direct,
	.format[PSC_CURRENT_IN] = direct,
	.format[PSC_CURRENT_OUT] = direct,
	.format[PSC_POWER] = direct,
	.m[PSC_VOLTAGE_IN] = 8,
	.b[PSC_VOLTAGE_IN] = 0,
	.R[PSC_VOLTAGE_IN] = 2,
	.m[PSC_VOLTAGE_OUT] = 8,
	.b[PSC_VOLTAGE_OUT] = 0,
	.R[PSC_VOLTAGE_OUT] = 2,
	.m[PSC_CURRENT_IN] = 1000,
	.b[PSC_CURRENT_IN] = 0,
	.R[PSC_CURRENT_IN] = 0,
	.m[PSC_CURRENT_OUT] = 1000,
	.b[PSC_CURRENT_OUT] = 0,
	.R[PSC_CURRENT_OUT] = 0,
	.m[PSC_POWER] = 40,
	.b[PSC_POWER] = 0,
	.R[PSC_POWER] = 0,
	.func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT | PMBUS_HAVE_IIN |
		   PMBUS_HAVE_IOUT | PMBUS_HAVE_PIN | PMBUS_HAVE_POUT |
		   PMBUS_HAVE_STATUS_IOUT | PMBUS_HAVE_STATUS_INPUT,
};

static int ina233_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	u32 shunt_val;
	struct device *dev = &client->dev;

	// Setup calibration register
	if (of_property_read_u32(dev->of_node, "shunt-resistor", &shunt_val) <
		0) {
		dev_err(dev, "No shunt-resistor property found!");
		return -ENODEV;
	}
	i2c_smbus_write_word_data(
		client, MFR_CALIBRATION, CAL_FACTOR / shunt_val);

	return pmbus_do_probe(client, id, &ina233_info);
}

static const struct i2c_device_id ina233_id[] = {
	{ "ina233", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ina233_id);

static struct i2c_driver ina233_driver = {
	.driver = {
		.name = "ina233",
	},
	.probe = ina233_probe,
	.remove = pmbus_do_remove,
	.id_table = ina233_id,
	};
module_i2c_driver(ina233_driver);

MODULE_AUTHOR("Alex Van Damme <atv@google.com>");
MODULE_DESCRIPTION("PMBus driver for TI INA233");
MODULE_LICENSE("GPL v2");
