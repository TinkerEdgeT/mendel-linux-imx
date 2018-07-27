/*
 * Google Apex PMIC Driver
 *
 * Copyright 2018 Google LLC
 *
 * Author: Michael Brooks <mrbrooks@google.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>

#define ISL91301_NUM_REGULATORS 4

/* Registers */
#define ISL91301_IO_MODE_CONTROL 0x24
#define APEX_ALL_ON 0xF0
#define APEX_CORE_OFF 0x20 // For Pre-DVT Devices, buck3 must remain on.

/* PMIC details */
struct apex_pmic {
	struct i2c_client *client;
	struct regulator_dev *rdev;
	struct mutex lock;
};

static const struct regulator_init_data apex_pmic_default = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.boot_on = 1,
	},
};

static int apex_is_enabled(struct regulator_dev *rdev)
{
	struct apex_pmic *pmic = rdev_get_drvdata(rdev);
	int reg;

	// For Pre-DVT devices, use I2C to verify all four bucks are enabled.
	mutex_lock(&pmic->lock);
	reg = i2c_smbus_read_byte_data(pmic->client, ISL91301_IO_MODE_CONTROL);
	mutex_unlock(&pmic->lock);
	if (reg < 0) {
		dev_err(&pmic->client->dev, "Failed to communicate: %d\n", reg);
		return reg;
	}

	return (reg & APEX_ALL_ON) == APEX_ALL_ON;
}

static int apex_set_enabled_disabled(
	struct regulator_dev *rdev, bool enable)
{
	struct apex_pmic *pmic = rdev_get_drvdata(rdev);
	int reg;

	mutex_lock(&pmic->lock);
	reg = i2c_smbus_read_byte_data(pmic->client, ISL91301_IO_MODE_CONTROL);

	// For Pre-DVT Devices, leave Buck3 on when disabling.
	i2c_smbus_write_byte_data(pmic->client, ISL91301_IO_MODE_CONTROL,
		(enable ? reg | APEX_ALL_ON
		: reg & (APEX_CORE_OFF | 0xF)));
	mutex_unlock(&pmic->lock);

	// Verify the pmic has been enabled/disabled.
	reg = apex_is_enabled(rdev);
	if (reg < 0) {
		return reg;
	}

	return !(reg == enable);
}

static int apex_enable(struct regulator_dev *rdev)
{
	return apex_set_enabled_disabled(rdev, true);
}

static int apex_disable(struct regulator_dev *rdev)
{
	return apex_set_enabled_disabled(rdev, false);
}

static struct regulator_ops apex_regulator_ops = {
	.is_enabled = apex_is_enabled,
	.enable = apex_enable,
	.disable = apex_disable,
};

static struct regulator_desc apex_regulators = {
	.name = "apex_regulators",
	.id = 0,
	.ops = &apex_regulator_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.of_match =	of_match_ptr("apex_regulators"),
	.regulators_node = of_match_ptr("regulators"),
};

static int apex_pmic_probe(
	struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct regulator_config config = {};
	struct apex_pmic *pmic;
	int ret;

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		return -EIO;
	}

	pmic = devm_kzalloc(&i2c->dev, sizeof(struct apex_pmic), GFP_KERNEL);
	if (!pmic) {
		return -ENOMEM;
	}

	pmic->client = i2c;

	mutex_init(&pmic->lock);

	i2c_set_clientdata(i2c, pmic);
	config.dev = &i2c->dev;
	config.driver_data = pmic;

	config.init_data = &apex_pmic_default;


	pmic->rdev = devm_regulator_register(&i2c->dev, &apex_regulators, &config);
	if (IS_ERR(pmic->rdev)) {
		ret = PTR_ERR(pmic->rdev);
		dev_err(&i2c->dev, "Failed to register %s: %d\n",
			apex_regulators.name, ret);
		return ret;
	} else {
		printk("Succesful registering of %s\n",
			apex_regulators.name);
	}

	return 0;
}

static const struct of_device_id apex_pmic_ids[] = {
	{ .compatible = "apex_pmic" },
	{},
};
MODULE_DEVICE_TABLE(of, apex_pmic_ids);

static const struct i2c_device_id apex_pmic_i2c_ids[] = {
	{ "apex_pmic", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, apex_pmic_i2c_id);

static struct i2c_driver apex_pmic_driver = {
	.driver = {
		.name = "apex_pmic",
		.of_match_table	= of_match_ptr(apex_pmic_ids),
	},
	.probe = apex_pmic_probe,
	.id_table = apex_pmic_i2c_ids,
};

module_i2c_driver(apex_pmic_driver);

MODULE_AUTHOR("Michael Brooks <mrbrooks@google.com>");
MODULE_DESCRIPTION("Google Apex PMIC Driver");
MODULE_LICENSE("GPL v2");
