/*
 * isl91301 - Renesas ISL91301 Driver
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
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>

#define ISL91301_NUM_REGULATORS 4

/* Registers */
#define ISL91301_IO_MODE_CONTROL 0x24
#define ISL91301_BUCKx_EN_MASK(x) (1 << (8 - x))

/* PMIC details */
struct isl_pmic {
	struct i2c_client *client;
	struct regulator_dev *rdev[ISL91301_NUM_REGULATORS];
	struct mutex lock;
};

static const struct regulator_init_data isl91301_default = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
};

static const struct regulator_init_data isl91301_always_on = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = 1,
	},
};

static int isl91301_is_enabled(struct regulator_dev *rdev)
{
	struct isl_pmic *pmic = rdev_get_drvdata(rdev);
	int rid = rdev_get_id(rdev);
	int reg;

	mutex_lock(&pmic->lock);
	reg = i2c_smbus_read_byte_data(pmic->client, ISL91301_IO_MODE_CONTROL);
	mutex_unlock(&pmic->lock);
	if (reg < 0) {
		dev_err(&pmic->client->dev, "Failed to communicate: %d\n", reg);
		return reg;
	}

	return (reg & ISL91301_BUCKx_EN_MASK(rid));
}

static int isl91301_set_enabled_disabled(
	struct regulator_dev *rdev, bool enable)
{
	struct isl_pmic *pmic = rdev_get_drvdata(rdev);
	int rid = rdev_get_id(rdev);
	int reg;

	mutex_lock(&pmic->lock);
	reg = i2c_smbus_read_byte_data(pmic->client, ISL91301_IO_MODE_CONTROL);

	i2c_smbus_write_byte_data(pmic->client, ISL91301_IO_MODE_CONTROL,
		(enable ? reg | ISL91301_BUCKx_EN_MASK(rid)
			: reg & ~(ISL91301_BUCKx_EN_MASK(rid))));
	mutex_unlock(&pmic->lock);

	// Verify the buck has been enabled/disabled.
	reg = isl91301_is_enabled(rdev);
	if (reg < 0) {
		return reg;
	}

	return !(reg == enable);
}

static int isl91301_enable(struct regulator_dev *rdev)
{
	return isl91301_set_enabled_disabled(rdev, true);
}

static int isl91301_disable(struct regulator_dev *rdev)
{
	return isl91301_set_enabled_disabled(rdev, false);
}

static struct regulator_ops isl_buck_ops = {
	.is_enabled = isl91301_is_enabled,
	.enable = isl91301_enable,
	.disable = isl91301_disable,
};

static struct regulator_desc isl91301_regulators[] = {
	{
		.name = "isl91301_buck1",
		.id = 1,
		.ops = &isl_buck_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "isl91301_buck2",
		.id = 2,
		.ops = &isl_buck_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "isl91301_buck3",
		.id = 3,
		.ops = &isl_buck_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "isl91301_buck4",
		.id = 4,
		.ops = &isl_buck_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
};

static int isl91301_probe(
	struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct regulator_config config = {};
	struct isl_pmic *pmic;
	int i, ret;

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		return -EIO;
	}

	pmic = devm_kzalloc(&i2c->dev, sizeof(struct isl_pmic), GFP_KERNEL);
	if (!pmic) {
		return -ENOMEM;
	}

	pmic->client = i2c;

	mutex_init(&pmic->lock);

	i2c_set_clientdata(i2c, pmic);
	config.dev = &i2c->dev;
	config.driver_data = pmic;

	for (i = 0; i < ARRAY_SIZE(isl91301_regulators); i++) {
		// Temporarily hard code BUCK3 to be always on.
		if (i == 2) {
			config.init_data = &isl91301_always_on;
		} else {
			config.init_data = &isl91301_default;
		}

		pmic->rdev[i] = devm_regulator_register(
			&i2c->dev, &isl91301_regulators[i], &config);
		if (IS_ERR(pmic->rdev[i])) {
			ret = PTR_ERR(pmic->rdev[i]);
			dev_err(&i2c->dev, "Failed to register %s: %d\n",
				isl91301_regulators[i].name, ret);
			return ret;
		} else {
			printk("Succesful registering of %s\n",
				isl91301_regulators[i].name);
		}
	}

	return 0;
}

static const struct i2c_device_id isl91301_id[] = {
	{ .name = "isl91301", 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, isl91301_id);

static struct i2c_driver isl91301_i2c_driver = {
	.driver = {
		.name = "isl91301",
	},
	.probe = isl91301_probe,
	.id_table = isl91301_id,
};

static int __init isl91301_init(void)
{
	return i2c_add_driver(&isl91301_i2c_driver);
}

static void __exit isl91301_cleanup(void)
{
	i2c_del_driver(&isl91301_i2c_driver);
}

subsys_initcall(isl91301_init);
module_exit(isl91301_cleanup);

MODULE_AUTHOR("Michael Brooks <mrbrooks@google.com>");
MODULE_DESCRIPTION("Renesas ISL91301 PMIC Driver");
MODULE_LICENSE("GPL v2");
