/*
 * @file bd71840-regulator.c ROHM BD71840MWV regulator driver
 *
 * @author: cpham2403@gmail.com
 * Copyright 2017.
 *
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 */
#define DEBUG
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/mfd/bd71840.h>
#include <linux/regulator/of_regulator.h>

#define BD71840_DVS_BUCK_NUM		4	/* Buck 1/2/3/4 support DVS */
#define BD71840_DVS_RUN_IDLE_SUSP	3
#define BD71840_DVS_RUN_IDLE		2
#define BD71840_DVS_RUN				1

struct bd71840_buck_dvs {
	u32 voltage[BD71840_DVS_RUN_IDLE_SUSP];
};

/** @brief bd71840 regulator type */
struct bd71840_pmic {
	struct regulator_desc descs[BD71840_REGULATOR_CNT];	/**< regulator description to system */
	struct bd71840 *mfd;									/**< parent device */
	struct device *dev;										/**< regulator kernel device */
	struct regulator_dev *rdev[BD71840_REGULATOR_CNT];		/**< regulator device of system */
	struct bd71840_buck_dvs buck_dvs[BD71840_DVS_BUCK_NUM];			/**< buck1/2 dvs */
	int	reg_index;
};

/*
 * BUCK1/2/3/4 for BD71837 or BUCK1/2 for BD71847
 * BUCK1RAMPRATE[1:0] BUCK1 DVS ramp rate setting
 * 00: 10.00mV/usec 10mV 1uS
 * 01: 5.00mV/usec	10mV 2uS
 * 10: 2.50mV/usec	10mV 4uS
 * 11: 1.25mV/usec	10mV 8uS
 */
static int bd71840_buck1234_set_ramp_delay(struct regulator_dev *rdev, int ramp_delay)
{
	struct bd71840_pmic *pmic = rdev_get_drvdata(rdev);
	struct bd71840 *mfd = pmic->mfd;
	int id = rdev->desc->id;
	unsigned int ramp_value = BUCK1_RAMPRATE_10P00MV;

	dev_dbg(pmic->dev, "Buck[%d] Set Ramp = %d\n", id + 1, ramp_delay);
	switch (ramp_delay) {
	case 1 ... 1250:
		ramp_value = BUCK1_RAMPRATE_1P25MV;
		break;
	case 1251 ... 2500:
		ramp_value = BUCK1_RAMPRATE_2P50MV;
		break;
	case 2501 ... 5000:
		ramp_value = BUCK1_RAMPRATE_5P00MV;
		break;
	case 5001 ... 10000:
		ramp_value = BUCK1_RAMPRATE_10P00MV;
		break;
	default:
		ramp_value = BUCK1_RAMPRATE_10P00MV;
		dev_err(pmic->dev, "%s: ramp_delay: %d not supported, setting 10000mV//us\n",
			rdev->desc->name, ramp_delay);
	}

	return regmap_update_bits(mfd->regmap, BD71840_REG_BUCK1_CTRL + id,
			BUCK1_RAMPRATE_MASK, ramp_value << 6);
}

static struct regulator_ops bd71840_ldo_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
};

static struct regulator_ops bd71840_fixed_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
};

static struct regulator_ops bd71840_buck_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
};

static struct regulator_ops bd71840_buck1234_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
	.set_ramp_delay = bd71840_buck1234_set_ramp_delay,
};

/*
 * BUCK1/2/3/4
 * 0.70 to 1.30V (10mV step)
 */
static const struct regulator_linear_range bd71840_buck1234_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(700000,  0x00, 0x3C, 10000),
	REGULATOR_LINEAR_RANGE(1300000,  0x3D, 0x3F, 0),
};

/*
 * BUCK5 for BD71837
 * 0.7V to 1.35V ()
 */
static const struct regulator_linear_range bd71840_bd71837_buck5_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(700000, 0x00, 0x03, 100000),
	REGULATOR_LINEAR_RANGE(1050000, 0x04, 0x05, 50000),
	REGULATOR_LINEAR_RANGE(1200000, 0x06, 0x07, 150000),
};

/*
 * BUCK5 for BD71847
 *
 * 00 = 0.70~1.35V
 * 01 = 0.55~0.90V
 * 10 = 0.675~1.325V
 * 11 = reserved
 */

/*
 * SEL = 00
 * 000 = 0.7V
 * 001 = 0.8V
 * 010 = 0.9V (Initial)
 * 011 = 1.0V
 * 100 = 1.05V
 * 101 = 1.1V
 * 110 = 1.2V
 * 111 = 1.35V
 */
static const struct regulator_linear_range bd71840_bd71847_buck5_sel_00_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(700000, 0x00, 0x03, 100000),
	REGULATOR_LINEAR_RANGE(1050000, 0x04, 0x05, 50000),
	REGULATOR_LINEAR_RANGE(1200000, 0x06, 0x07, 150000),
};

/*
 * SEL = 01
 * 000 = 0.55V
 * 001 = 0.60V
 * 010 = 0.65V
 * 011 = 0.70V
 * 100 = 0.75V
 * 101 = 0.80V
 * 110 = 0.85V
 * 111 = 0.90V
 */
static const struct regulator_linear_range bd71840_bd71847_buck5_sel_01_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(550000, 0x00, 0x07, 50000),
};

/*
 * SEL = 10
 * 000 = 0.675V
 * 001 = 0.775V
 * 010 = 0.875V
 * 011 = 0.975V
 * 100 = 1.025V
 * 101 = 1.075V
 * 110 = 1.175V
 * 111 = 1.325V
 */
static const struct regulator_linear_range bd71840_bd71847_buck5_sel_10_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(675000, 0x00, 0x03, 100000),
	REGULATOR_LINEAR_RANGE(1025000, 0x04, 0x05, 50000),
	REGULATOR_LINEAR_RANGE(1175000, 0x06, 0x07, 150000),
};

/**
 * bd71840_bd71847_buck5_list_voltage_linear_range - List voltages for linear ranges
 *
 * @rdev: Regulator device
 * @selector: Selector to convert into a voltage
 *
 * Regulators with a series of simple linear mappings between voltages
 * and selectors can set linear_ranges in the regulator descriptor and
 * then use this function as their list_voltage() operation,
 */
static int bd71840_bd71847_buck5_list_voltage_linear_range(struct regulator_dev *rdev,
					unsigned int selector)
{
	const struct regulator_linear_range *linear_ranges;
	int n_linear_ranges;
	const struct regulator_linear_range *range;
	int i, ret, val;

	/* Dynamic upate linear_ranges based on new SEL mode */
	ret = regmap_read( rdev->regmap, rdev->desc->vsel_reg, &val);
	if (ret != 0)
		return ret;

	switch(val & BUCK5_VOLT_SEL_MASK) {
	default:
	case BUCK5_VOLT_SEL_00:
		linear_ranges = bd71840_bd71847_buck5_sel_00_voltage_ranges;
		n_linear_ranges = ARRAY_SIZE(bd71840_bd71847_buck5_sel_00_voltage_ranges);
		break;
	case BUCK5_VOLT_SEL_01:
		linear_ranges = bd71840_bd71847_buck5_sel_01_voltage_ranges;
		n_linear_ranges = ARRAY_SIZE(bd71840_bd71847_buck5_sel_01_voltage_ranges);
		break;
	case BUCK5_VOLT_SEL_10:
		linear_ranges = bd71840_bd71847_buck5_sel_10_voltage_ranges;
		n_linear_ranges = ARRAY_SIZE(bd71840_bd71847_buck5_sel_10_voltage_ranges);
		break;
	}

	if (!n_linear_ranges) {
		BUG_ON(!n_linear_ranges);
		return -EINVAL;
	}

	for (i = 0; i < n_linear_ranges; i++) {
		range = &linear_ranges[i];

		if (!(selector >= range->min_sel &&
		      selector <= range->max_sel))
			continue;

		selector -= range->min_sel;

		return range->min_uV + (range->uV_step * selector);
	}

	return -EINVAL;
}

/**
 * bd71840_bd71847_buck5_map_voltage_linear_range - map_voltage() for multiple linear ranges
 *
 * @rdev: Regulator to operate on
 * @min_uV: Lower bound for voltage
 * @max_uV: Upper bound for voltage
 *
 * Drivers providing linear_ranges in their descriptor can use this as
 * their map_voltage() callback.
 */
static int bd71840_bd71847_buck5_map_voltage_linear_range(struct regulator_dev *rdev,
				       int min_uV, int max_uV)
{
	const struct regulator_linear_range *linear_ranges;
	int n_linear_ranges;
	const struct regulator_linear_range *range;
	int ret = -EINVAL;
	int voltage, i, val;

	/* Dynamic upate linear_ranges based on new SEL mode */
	ret = regmap_read( rdev->regmap, rdev->desc->vsel_reg, &val);
	if (ret != 0)
		return ret;

	switch(val & BUCK5_VOLT_SEL_MASK) {
	default:
	case BUCK5_VOLT_SEL_00:
		linear_ranges = bd71840_bd71847_buck5_sel_00_voltage_ranges;
		n_linear_ranges = ARRAY_SIZE(bd71840_bd71847_buck5_sel_00_voltage_ranges);
		break;
	case BUCK5_VOLT_SEL_01:
		linear_ranges = bd71840_bd71847_buck5_sel_01_voltage_ranges;
		n_linear_ranges = ARRAY_SIZE(bd71840_bd71847_buck5_sel_01_voltage_ranges);
		break;
	case BUCK5_VOLT_SEL_10:
		linear_ranges = bd71840_bd71847_buck5_sel_10_voltage_ranges;
		n_linear_ranges = ARRAY_SIZE(bd71840_bd71847_buck5_sel_10_voltage_ranges);
		break;
	}

	for (i = 0; i < rdev->desc->n_linear_ranges; i++) {
		int linear_max_uV;

		range = &linear_ranges[i];
		linear_max_uV = range->min_uV +
			(range->max_sel - range->min_sel) * range->uV_step;

		if (!(min_uV <= linear_max_uV && max_uV >= range->min_uV))
			continue;

		if (min_uV <= range->min_uV)
			min_uV = range->min_uV;

		/* range->uV_step == 0 means fixed voltage range */
		if (range->uV_step == 0) {
			ret = 0;
		} else {
			ret = DIV_ROUND_UP(min_uV - range->min_uV,
					   range->uV_step);
			if (ret < 0)
				return ret;
		}

		ret += range->min_sel;

		break;
	}

	if (i == n_linear_ranges)
		return -EINVAL;

	/* Map back into a voltage to verify we're still in bounds */
	voltage = rdev->desc->ops->list_voltage(rdev, ret);
	if (voltage < min_uV || voltage > max_uV)
		return -EINVAL;

	return ret;
}

static struct regulator_ops bd71840_bd71847_buck5_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = bd71840_bd71847_buck5_list_voltage_linear_range,
	.map_voltage = bd71840_bd71847_buck5_map_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
};

/*
 * BUCK6
 * 3.0V to 3.3V (step 100mV)
 */
static const struct regulator_linear_range bd71840_buck6_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(3000000, 0x00, 0x03, 100000),
};

/*
 * BUCK6 for BD71847
 *
 * 0 = 3.0~3.3V
 * 1 = 2.6~2.9V
 */

/*
 * SEL = 00
 * 00 = 3.0V
 * 01 = 3.1V
 * 10 = 3.2V
 * 11 = 3.3V(Initial)
 */
static const struct regulator_linear_range bd71840_bd71847_buck6_sel_0_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(3000000, 0x00, 0x03, 100000),
};

/*
 * SEL = 1
 * 00 = 2.6V
 * 01 = 2.7V
 * 10 = 2.8V
 * 11 = 2.9V
 */
static const struct regulator_linear_range bd71840_bd71847_buck6_sel_1_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(2600000, 0x00, 0x03, 100000),
};

/**
 * bd71840_bd71847_buck6_list_voltage_linear_range - List voltages for linear ranges
 *
 * @rdev: Regulator device
 * @selector: Selector to convert into a voltage
 *
 * Regulators with a series of simple linear mappings between voltages
 * and selectors can set linear_ranges in the regulator descriptor and
 * then use this function as their list_voltage() operation,
 */
static int bd71840_bd71847_buck6_list_voltage_linear_range(struct regulator_dev *rdev,
					unsigned int selector)
{
	const struct regulator_linear_range *linear_ranges;
	int n_linear_ranges;
	const struct regulator_linear_range *range;
	int i, ret, val;

	/* Dynamic upate linear_ranges based on new SEL mode */
	ret = regmap_read( rdev->regmap, rdev->desc->vsel_reg, &val);
	if (ret != 0)
		return ret;

	switch(val & BUCK6_VOLT_SEL_MASK) {
	default:
	case BUCK6_VOLT_SEL_0:
		linear_ranges = bd71840_bd71847_buck6_sel_0_voltage_ranges;
		n_linear_ranges = ARRAY_SIZE(bd71840_bd71847_buck6_sel_0_voltage_ranges);
		break;
	case BUCK6_VOLT_SEL_1:
		linear_ranges = bd71840_bd71847_buck6_sel_1_voltage_ranges;
		n_linear_ranges = ARRAY_SIZE(bd71840_bd71847_buck6_sel_1_voltage_ranges);
		break;
	}

	if (!n_linear_ranges) {
		BUG_ON(!n_linear_ranges);
		return -EINVAL;
	}

	for (i = 0; i < n_linear_ranges; i++) {
		range = &linear_ranges[i];

		if (!(selector >= range->min_sel &&
		      selector <= range->max_sel))
			continue;

		selector -= range->min_sel;

		return range->min_uV + (range->uV_step * selector);
	}

	return -EINVAL;
}

/**
 * bd71840_bd71847_buck6_map_voltage_linear_range - map_voltage() for multiple linear ranges
 *
 * @rdev: Regulator to operate on
 * @min_uV: Lower bound for voltage
 * @max_uV: Upper bound for voltage
 *
 * Drivers providing linear_ranges in their descriptor can use this as
 * their map_voltage() callback.
 */
static int bd71840_bd71847_buck6_map_voltage_linear_range(struct regulator_dev *rdev,
				       int min_uV, int max_uV)
{
	const struct regulator_linear_range *linear_ranges;
	int n_linear_ranges;
	const struct regulator_linear_range *range;
	int ret = -EINVAL;
	int voltage, i, val;

	/* Dynamic upate linear_ranges based on new SEL mode */
	ret = regmap_read( rdev->regmap, rdev->desc->vsel_reg, &val);
	if (ret != 0)
		return ret;

	switch(val & BUCK6_VOLT_SEL_MASK) {
	default:
	case BUCK6_VOLT_SEL_0:
		linear_ranges = bd71840_bd71847_buck6_sel_0_voltage_ranges;
		n_linear_ranges = ARRAY_SIZE(bd71840_bd71847_buck6_sel_0_voltage_ranges);
		break;
	case BUCK6_VOLT_SEL_1:
		linear_ranges = bd71840_bd71847_buck6_sel_1_voltage_ranges;
		n_linear_ranges = ARRAY_SIZE(bd71840_bd71847_buck6_sel_1_voltage_ranges);
		break;
	}

	for (i = 0; i < rdev->desc->n_linear_ranges; i++) {
		int linear_max_uV;

		range = &linear_ranges[i];
		linear_max_uV = range->min_uV +
			(range->max_sel - range->min_sel) * range->uV_step;

		if (!(min_uV <= linear_max_uV && max_uV >= range->min_uV))
			continue;

		if (min_uV <= range->min_uV)
			min_uV = range->min_uV;

		/* range->uV_step == 0 means fixed voltage range */
		if (range->uV_step == 0) {
			ret = 0;
		} else {
			ret = DIV_ROUND_UP(min_uV - range->min_uV,
					   range->uV_step);
			if (ret < 0)
				return ret;
		}

		ret += range->min_sel;

		break;
	}

	if (i == n_linear_ranges)
		return -EINVAL;

	/* Map back into a voltage to verify we're still in bounds */
	voltage = rdev->desc->ops->list_voltage(rdev, ret);
	if (voltage < min_uV || voltage > max_uV)
		return -EINVAL;

	return ret;
}

static struct regulator_ops bd71840_bd71847_buck6_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = bd71840_bd71847_buck6_list_voltage_linear_range,
	.map_voltage = bd71840_bd71847_buck6_map_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
};

/*
 * BUCK7
 * 1.605V to 1.995V ()
 * 000 = 1.605V
 * 001 = 1.695V
 * 010 = 1.755V
 * 011 = 1.8V (Initial)
 * 100 = 1.845V
 * 101 = 1.905V
 * 110 = 1.95V
 * 111 = 1.995V
 */
static const struct regulator_linear_range bd71840_buck7_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(1605000, 0x00, 0x01, 90000),
	REGULATOR_LINEAR_RANGE(1755000, 0x02, 0x03, 45000),
	REGULATOR_LINEAR_RANGE(1845000, 0x04, 0x05, 60000),
	REGULATOR_LINEAR_RANGE(1950000, 0x06, 0x07, 45000),
};

/*
 * BUCK8
 * 0.8V to 1.40V (step 10mV)
 */
static const struct regulator_linear_range bd71840_buck8_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(800000,  0x00, 0x3C, 10000),
	REGULATOR_LINEAR_RANGE(1400000,  0x3D, 0x3F, 0),
};

/*
 * LDO1
 * 3.0 to 3.3V (100mV step)
 */
static const struct regulator_linear_range bd71840_bd71837_ldo1_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(3000000, 0x00, 0x03, 100000),
};

/*
 * LDO1 for BD71847
 *
 * 0 = 3.0~3.3V
 * 1 = 1.6~1.9V
 */

/*
 * SEL = 0
 * 00 = 3.0V
 * 01 = 3.1V
 * 10 = 3.2V
 * 11 = 3.3V
 */
static const struct regulator_linear_range bd71840_bd71847_ldo1_sel_0_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(3000000, 0x00, 0x03, 100000),
};

/*
 * SEL = 1
 * 00 = 1.6V
 * 01 = 1.7V
 * 10 = 1.8V (Initial)
 * 11 = 1.9V
 */
static const struct regulator_linear_range bd71840_bd71847_ldo1_sel_1_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(1600000, 0x00, 0x03, 100000),
};

/**
 * bd71840_bd71847_ldo1_list_voltage_linear_range - List voltages for linear ranges
 *
 * @rdev: Regulator device
 * @selector: Selector to convert into a voltage
 *
 * Regulators with a series of simple linear mappings between voltages
 * and selectors can set linear_ranges in the regulator descriptor and
 * then use this function as their list_voltage() operation,
 */
static int bd71840_bd71847_ldo1_list_voltage_linear_range(struct regulator_dev *rdev,
					unsigned int selector)
{
	const struct regulator_linear_range *linear_ranges;
	int n_linear_ranges = 0;
	const struct regulator_linear_range *range;
	int i, ret, val;

	/* Dynamic upate linear_ranges based on new SEL mode */
	ret = regmap_read( rdev->regmap, rdev->desc->vsel_reg, &val);
	if (ret != 0)
		return ret;

	switch(val & LDO1_VOLT_SEL_MASK) {
	default:
	case LDO1_VOLT_SEL_0:
		linear_ranges = bd71840_bd71847_ldo1_sel_0_voltage_ranges;
		n_linear_ranges = ARRAY_SIZE(bd71840_bd71847_ldo1_sel_0_voltage_ranges);
		break;
	case LDO1_VOLT_SEL_1:
		linear_ranges = bd71840_bd71847_ldo1_sel_1_voltage_ranges;
		n_linear_ranges = ARRAY_SIZE(bd71840_bd71847_ldo1_sel_1_voltage_ranges);
		break;
	}

	if (!n_linear_ranges) {
		BUG_ON(!n_linear_ranges);
		return -EINVAL;
	}

	for (i = 0; i < n_linear_ranges; i++) {
		range = &linear_ranges[i];

		if (!(selector >= range->min_sel &&
		      selector <= range->max_sel))
			continue;

		selector -= range->min_sel;

		return range->min_uV + (range->uV_step * selector);
	}

	return -EINVAL;
}

/**
 * bd71840_bd71847_ldo1_map_voltage_linear_range - map_voltage() for multiple linear ranges
 *
 * @rdev: Regulator to operate on
 * @min_uV: Lower bound for voltage
 * @max_uV: Upper bound for voltage
 *
 * Drivers providing linear_ranges in their descriptor can use this as
 * their map_voltage() callback.
 */
static int bd71840_bd71847_ldo1_map_voltage_linear_range(struct regulator_dev *rdev,
				       int min_uV, int max_uV)
{
	const struct regulator_linear_range *linear_ranges;
	int n_linear_ranges = 0;
	const struct regulator_linear_range *range;
	int ret = -EINVAL;
	int voltage, i, val;

	/* Dynamic upate linear_ranges based on new SEL mode */
	ret = regmap_read( rdev->regmap, rdev->desc->vsel_reg, &val);
	if (ret != 0)
		return ret;

	switch(val & LDO1_VOLT_SEL_MASK) {
	default:
	case LDO1_VOLT_SEL_0:
		linear_ranges = bd71840_bd71847_ldo1_sel_0_voltage_ranges;
		n_linear_ranges = ARRAY_SIZE(bd71840_bd71847_ldo1_sel_0_voltage_ranges);
		break;
	case LDO1_VOLT_SEL_1:
		linear_ranges = bd71840_bd71847_ldo1_sel_1_voltage_ranges;
		n_linear_ranges = ARRAY_SIZE(bd71840_bd71847_ldo1_sel_1_voltage_ranges);
		break;
	}

	for (i = 0; i < rdev->desc->n_linear_ranges; i++) {
		int linear_max_uV;

		range = &linear_ranges[i];
		linear_max_uV = range->min_uV +
			(range->max_sel - range->min_sel) * range->uV_step;

		if (!(min_uV <= linear_max_uV && max_uV >= range->min_uV))
			continue;

		if (min_uV <= range->min_uV)
			min_uV = range->min_uV;

		/* range->uV_step == 0 means fixed voltage range */
		if (range->uV_step == 0) {
			ret = 0;
		} else {
			ret = DIV_ROUND_UP(min_uV - range->min_uV,
					   range->uV_step);
			if (ret < 0)
				return ret;
		}

		ret += range->min_sel;

		break;
	}

	if (i == n_linear_ranges)
		return -EINVAL;

	/* Map back into a voltage to verify we're still in bounds */
	voltage = rdev->desc->ops->list_voltage(rdev, ret);
	if (voltage < min_uV || voltage > max_uV)
		return -EINVAL;

	return ret;
}

static struct regulator_ops bd71840_bd71847_ldo1_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = bd71840_bd71847_ldo1_list_voltage_linear_range,
	.map_voltage = bd71840_bd71847_ldo1_map_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
};

/**
 * bd71840_bd71847_ldo2_list_voltage_linear_range - List voltages for linear ranges
 *
 * @rdev: Regulator device
 * @selector: Selector to convert into a voltage
 *
 * Regulators with a series of simple linear mappings between voltages
 * and selectors can set linear_ranges in the regulator descriptor and
 * then use this function as their list_voltage() operation,
 */
static int bd71840_bd71847_ldo2_list_voltage_linear_range(struct regulator_dev *rdev,
					unsigned int selector)
{
	unsigned int min_uV;
	int ret, val;

	/* Dynamic upate linear_ranges based on new SEL mode */
	ret = regmap_read( rdev->regmap, rdev->desc->vsel_reg, &val);
	if (ret != 0)
		return ret;

	switch(val & LDO2_VOLT_SEL_MASK) {
	default:
	case LDO2_VOLT_SEL_0:
		min_uV = 900000;
		break;
	case LDO2_VOLT_SEL_1:
		min_uV = 800000;
		break;
	}

	return min_uV;
}

static struct regulator_ops bd71840_bd71847_ldo2_fixed_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = bd71840_bd71847_ldo2_list_voltage_linear_range,
};

/*
 * LDO3
 * 1.8 to 3.3V (100mV step)
 */
static const struct regulator_linear_range bd71840_ldo3_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(1800000, 0x00, 0x0F, 100000),
};

/*
 * LDO4
 * 0.9 to 1.8V (100mV step)
 */
static const struct regulator_linear_range bd71840_ldo4_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(900000, 0x00, 0x09, 100000),
	REGULATOR_LINEAR_RANGE(1800000, 0x0A, 0x0F, 0),
};

/*
 * LDO5
 * 1.8 to 3.3V (100mV step)
 */
static const struct regulator_linear_range bd71840_bd71837_ldo5_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(1800000,  0x00, 0x0F, 100000),
};

/*
 * LDO5 for BD71847
 *
 * 0 = 1.8~3.3V
 * 1 = 0.8~2.3V
 */

/*
 * SEL = 0
 * 0x0 = 1.8V
 * 0x1 = 1.9V
 * 0x2 = 2.0V
 * 0x3 = 2.1V
 * 0x4 = 2.2V
 * 0x5 = 2.3V
 * 0x6 = 2.4V
 * 0x7 = 2.5V
 * 0x8 = 2.6V
 * 0x9 = 2.7V
 * 0xA = 2.8V
 * 0xB = 2.9V
 * 0xC = 3.0V
 * 0xD = 3.1V
 * 0xE = 3.2V
 * 0xF = 3.3V(initial)
 */
static const struct regulator_linear_range bd71840_bd71847_ldo5_sel_0_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(1800000, 0x00, 0x0F, 100000),
};

/*
 * SEL = 1
 * 0x0 = 0.8V
 * 0x1 = 0.9V
 * 0x2 = 1.0V
 * 0x3 = 1.1V
 * 0x4 = 1.2V
 * 0x5 = 1.3V
 * 0x6 = 1.4V
 * 0x7 = 1.5V
 * 0x8 = 1.6V
 * 0x9 = 1.7V
 * 0xA = 1.8V
 * 0xB = 1.9V
 * 0xC = 2.0V
 * 0xD = 2.1V
 * 0xE = 2.2V
 * 0xF = 2.3V
 */
static const struct regulator_linear_range bd71840_bd71847_ldo5_sel_1_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(800000, 0x00, 0x0F, 100000),
};

/**
 * bd71840_bd71847_ldo5_list_voltage_linear_range - List voltages for linear ranges
 *
 * @rdev: Regulator device
 * @selector: Selector to convert into a voltage
 *
 * Regulators with a series of simple linear mappings between voltages
 * and selectors can set linear_ranges in the regulator descriptor and
 * then use this function as their list_voltage() operation,
 */
static int bd71840_bd71847_ldo5_list_voltage_linear_range(struct regulator_dev *rdev,
					unsigned int selector)
{
	const struct regulator_linear_range *linear_ranges;
	int n_linear_ranges = 0;
	const struct regulator_linear_range *range;
	int i, ret, val;

	/* Dynamic upate linear_ranges based on new SEL mode */
	ret = regmap_read( rdev->regmap, rdev->desc->vsel_reg, &val);
	if (ret != 0)
		return ret;

	switch(val & LDO5_VOLT_SEL_MASK) {
	default:
	case LDO5_VOLT_SEL_0:
		linear_ranges = bd71840_bd71847_ldo5_sel_0_voltage_ranges;
		n_linear_ranges = ARRAY_SIZE(bd71840_bd71847_ldo5_sel_0_voltage_ranges);
		break;
	case LDO5_VOLT_SEL_1:
		linear_ranges = bd71840_bd71847_ldo5_sel_1_voltage_ranges;
		n_linear_ranges = ARRAY_SIZE(bd71840_bd71847_ldo5_sel_1_voltage_ranges);
		break;
	}

	if (!n_linear_ranges) {
		BUG_ON(!n_linear_ranges);
		return -EINVAL;
	}

	for (i = 0; i < n_linear_ranges; i++) {
		range = &linear_ranges[i];

		if (!(selector >= range->min_sel &&
		      selector <= range->max_sel))
			continue;

		selector -= range->min_sel;

		return range->min_uV + (range->uV_step * selector);
	}

	return -EINVAL;
}

/**
 * bd71840_bd71847_ldo5_map_voltage_linear_range - map_voltage() for multiple linear ranges
 *
 * @rdev: Regulator to operate on
 * @min_uV: Lower bound for voltage
 * @max_uV: Upper bound for voltage
 *
 * Drivers providing linear_ranges in their descriptor can use this as
 * their map_voltage() callback.
 */
static int bd71840_bd71847_ldo5_map_voltage_linear_range(struct regulator_dev *rdev,
				       int min_uV, int max_uV)
{
	const struct regulator_linear_range *linear_ranges;
	int n_linear_ranges = 0;
	const struct regulator_linear_range *range;
	int ret = -EINVAL;
	int voltage, i, val;

	/* Dynamic upate linear_ranges based on new SEL mode */
	ret = regmap_read( rdev->regmap, rdev->desc->vsel_reg, &val);
	if (ret != 0)
		return ret;

	switch(val & LDO5_VOLT_SEL_MASK) {
	default:
	case LDO5_VOLT_SEL_0:
		linear_ranges = bd71840_bd71847_ldo5_sel_0_voltage_ranges;
		n_linear_ranges = ARRAY_SIZE(bd71840_bd71847_ldo5_sel_0_voltage_ranges);
		break;
	case LDO5_VOLT_SEL_1:
		linear_ranges = bd71840_bd71847_ldo5_sel_1_voltage_ranges;
		n_linear_ranges = ARRAY_SIZE(bd71840_bd71847_ldo5_sel_1_voltage_ranges);
		break;
	}

	for (i = 0; i < rdev->desc->n_linear_ranges; i++) {
		int linear_max_uV;

		range = &linear_ranges[i];
		linear_max_uV = range->min_uV +
			(range->max_sel - range->min_sel) * range->uV_step;

		if (!(min_uV <= linear_max_uV && max_uV >= range->min_uV))
			continue;

		if (min_uV <= range->min_uV)
			min_uV = range->min_uV;

		/* range->uV_step == 0 means fixed voltage range */
		if (range->uV_step == 0) {
			ret = 0;
		} else {
			ret = DIV_ROUND_UP(min_uV - range->min_uV,
					   range->uV_step);
			if (ret < 0)
				return ret;
		}

		ret += range->min_sel;

		break;
	}

	if (i == n_linear_ranges)
		return -EINVAL;

	/* Map back into a voltage to verify we're still in bounds */
	voltage = rdev->desc->ops->list_voltage(rdev, ret);
	if (voltage < min_uV || voltage > max_uV)
		return -EINVAL;

	return ret;
}

static struct regulator_ops bd71840_bd71847_ldo5_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = bd71840_bd71847_ldo5_list_voltage_linear_range,
	.map_voltage = bd71840_bd71847_ldo5_map_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
};

/*
 * LDO6
 * 0.9 to 1.8V (100mV step)
 */
static const struct regulator_linear_range bd71840_ldo6_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(900000,  0x00, 0x09, 100000),
	REGULATOR_LINEAR_RANGE(1800000, 0x0A, 0x0F, 0),
};

/*
 * LDO7
 * 1.8 to 3.3V (100mV step)
 */
static const struct regulator_linear_range bd71840_ldo7_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(1800000, 0x00, 0x0F, 100000),
};

static const struct regulator_desc bd71840_bd71837_regulators[] = {
	{
		.name = "BUCK1",
		.id = BD71840_BUCK1,
		.ops = &bd71840_buck1234_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_BUCK1_VOLTAGE_NUM,
		.linear_ranges = bd71840_buck1234_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71840_buck1234_voltage_ranges),
		.vsel_reg = BD71840_REG_BUCK1_VOLT_RUN,
		.vsel_mask = BUCK1_RUN_MASK,
		.enable_reg = BD71840_REG_BUCK1_CTRL,
		//.enable_mask = BUCK1_SEL|BUCK1_EN,
		.enable_is_inverted = 1,
		.enable_mask = BUCK1_SEL,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK2",
		.id = BD71840_BUCK2,
		.ops = &bd71840_buck1234_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_BUCK2_VOLTAGE_NUM,
		.linear_ranges = bd71840_buck1234_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71840_buck1234_voltage_ranges),
		.vsel_reg = BD71840_REG_BUCK2_VOLT_RUN,
		.vsel_mask = BUCK2_RUN_MASK,
		.enable_reg = BD71840_REG_BUCK2_CTRL,
		//.enable_mask = BUCK2_SEL|BUCK2_EN,
		.enable_is_inverted = 1,
		.enable_mask = BUCK2_SEL,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK3",
		.id = BD71840_BUCK3,
		.ops = &bd71840_buck1234_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_BUCK3_VOLTAGE_NUM,
		.linear_ranges = bd71840_buck1234_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71840_buck1234_voltage_ranges),
		.vsel_reg = BD71840_REG_BUCK3_VOLT_RUN,
		.vsel_mask = BUCK3_RUN_MASK,
		.enable_reg = BD71840_REG_BUCK3_CTRL,
		//.enable_mask = BUCK3_SEL|BUCK3_EN,
		.enable_is_inverted = 1,
		.enable_mask = BUCK3_SEL,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK4",
		.id = BD71840_BUCK4,
		.ops = &bd71840_buck1234_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_BUCK4_VOLTAGE_NUM,
		.linear_ranges = bd71840_buck1234_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71840_buck1234_voltage_ranges),
		.vsel_reg = BD71840_REG_BUCK4_VOLT_RUN,
		.vsel_mask = BUCK4_RUN_MASK,
		.enable_reg = BD71840_REG_BUCK4_CTRL,
		//.enable_mask = BUCK4_SEL|BUCK4_EN,
		.enable_is_inverted = 1,
		.enable_mask = BUCK4_SEL,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK5",
		.id = BD71840_BUCK5,
		.ops = &bd71840_buck_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_BUCK5_VOLTAGE_NUM,
		.linear_ranges = bd71840_bd71837_buck5_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71840_bd71837_buck5_voltage_ranges),
		.vsel_reg = BD71840_REG_BUCK5_VOLT,
		.vsel_mask = BUCK5_MASK,
		.enable_reg = BD71840_REG_BUCK5_CTRL,
		//.enable_mask = BUCK5_SEL|BUCK5_EN,
		.enable_is_inverted = 1,
		.enable_mask = BUCK5_SEL,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK6",
		.id = BD71840_BUCK6,
		.ops = &bd71840_buck_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_BUCK6_VOLTAGE_NUM,
		.linear_ranges = bd71840_buck6_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71840_buck6_voltage_ranges),
		.vsel_reg = BD71840_REG_BUCK6_VOLT,
		.vsel_mask = BUCK6_MASK,
		.enable_reg = BD71840_REG_BUCK6_CTRL,
		//.enable_mask = BUCK6_SEL|BUCK6_EN,
		.enable_is_inverted = 1,
		.enable_mask = BUCK6_SEL,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK7",
		.id = BD71840_BUCK7,
		.ops = &bd71840_buck_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_BUCK7_VOLTAGE_NUM,
		.linear_ranges = bd71840_buck7_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71840_buck7_voltage_ranges),
		.vsel_reg = BD71840_REG_BUCK7_VOLT,
		.vsel_mask = BUCK7_MASK,
		.enable_reg = BD71840_REG_BUCK7_CTRL,
		//.enable_mask = BUCK7_SEL|BUCK7_EN,
		.enable_is_inverted = 1,
		.enable_mask = BUCK7_SEL,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK8",
		.id = BD71840_BUCK8,
		.ops = &bd71840_buck_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_BUCK8_VOLTAGE_NUM,
		.linear_ranges = bd71840_buck8_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71840_buck8_voltage_ranges),
		.vsel_reg = BD71840_REG_BUCK8_VOLT,
		.vsel_mask = BUCK8_MASK,
		.enable_reg = BD71840_REG_BUCK8_CTRL,
		//.enable_mask = BUCK8_SEL|BUCK8_EN,
		.enable_is_inverted = 1,
		.enable_mask = BUCK8_SEL,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO1",
		.id = BD71840_LDO1,
		.ops = &bd71840_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_LDO1_VOLTAGE_NUM,
		.linear_ranges = bd71840_bd71837_ldo1_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71840_bd71837_ldo1_voltage_ranges),
		.vsel_reg = BD71840_REG_LDO1_VOLT,
		.vsel_mask = LDO1_MASK,
		.enable_reg = BD71840_REG_LDO1_VOLT,
		//.enable_mask = LDO1_SEL|LDO1_EN,
		.enable_is_inverted = 1,
		.enable_mask = LDO1_SEL,
		.owner = THIS_MODULE,
	},
	/*
	 * LDO2 0.9V
	 * Fixed voltage
	 */
	{
		.name = "LDO2",
		.id = BD71840_LDO2,
		.ops = &bd71840_fixed_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_LDO2_VOLTAGE_NUM,
		.min_uV = 900000,
		.enable_reg = BD71840_REG_LDO2_VOLT,
		//.enable_mask = LDO2_SEL|LDO2_EN,
		.enable_is_inverted = 1,
		.enable_mask = LDO2_SEL,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO3",
		.id = BD71840_LDO3,
		.ops = &bd71840_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_LDO3_VOLTAGE_NUM,
		.linear_ranges = bd71840_ldo3_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71840_ldo3_voltage_ranges),
		.vsel_reg = BD71840_REG_LDO3_VOLT,
		.vsel_mask = LDO3_MASK,
		.enable_reg = BD71840_REG_LDO3_VOLT,
		//.enable_mask = LDO3_SEL|LDO3_EN,
		.enable_is_inverted = 1,
		.enable_mask = LDO3_SEL,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO4",
		.id = BD71840_LDO4,
		.ops = &bd71840_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_LDO4_VOLTAGE_NUM,
		.linear_ranges = bd71840_ldo4_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71840_ldo4_voltage_ranges),
		.vsel_reg = BD71840_REG_LDO4_VOLT,
		.vsel_mask = LDO4_MASK,
		.enable_reg = BD71840_REG_LDO4_VOLT,
		//.enable_mask = LDO4_SEL|LDO4_EN,
		.enable_is_inverted = 1,
		.enable_mask = LDO4_SEL,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO5",
		.id = BD71840_LDO5,
		.ops = &bd71840_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_LDO5_VOLTAGE_NUM,
		.linear_ranges = bd71840_bd71837_ldo5_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71840_bd71837_ldo5_voltage_ranges),
		.vsel_reg = BD71840_REG_LDO5_VOLT,
		.vsel_mask = LDO5_MASK,
		.enable_reg = BD71840_REG_LDO5_VOLT,
		.enable_mask = LDO5_EN,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO6",
		.id = BD71840_LDO6,
		.ops = &bd71840_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_LDO6_VOLTAGE_NUM,
		.linear_ranges = bd71840_ldo6_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71840_ldo6_voltage_ranges),
		.vsel_reg = BD71840_REG_LDO6_VOLT,
		.vsel_mask = LDO6_MASK,
		.enable_reg = BD71840_REG_LDO6_VOLT,
		.enable_mask = LDO6_EN,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO7",
		.id = BD71840_LDO7,
		.ops = &bd71840_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_LDO7_VOLTAGE_NUM,
		.linear_ranges = bd71840_ldo7_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71840_ldo7_voltage_ranges),
		.vsel_reg = BD71840_REG_LDO7_VOLT,
		.vsel_mask = LDO7_MASK,
		.enable_reg = BD71840_REG_LDO7_VOLT,
		.enable_mask = LDO7_EN,
		.owner = THIS_MODULE,
	},
};

static const struct regulator_desc bd71840_bd71847_regulators[] = {
	{
		.name = "BUCK1",
		.id = BD71840_BUCK1,
		.ops = &bd71840_buck1234_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_BUCK1_VOLTAGE_NUM,
		.linear_ranges = bd71840_buck1234_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71840_buck1234_voltage_ranges),
		.vsel_reg = BD71840_REG_BUCK1_VOLT_RUN,
		.vsel_mask = BD71847_BUCK1_RUN_MASK,
		.enable_reg = BD71840_REG_BUCK1_CTRL,
		//.enable_mask = BUCK1_SEL|BUCK1_EN,
		.enable_is_inverted = 1,
		.enable_mask = BUCK1_SEL,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK2",
		.id = BD71840_BUCK2,
		.ops = &bd71840_buck1234_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_BUCK2_VOLTAGE_NUM,
		.linear_ranges = bd71840_buck1234_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71840_buck1234_voltage_ranges),
		.vsel_reg = BD71840_REG_BUCK2_VOLT_RUN,
		.vsel_mask = BUCK2_RUN_MASK,
		.enable_reg = BD71840_REG_BUCK2_CTRL,
		//.enable_mask = BUCK2_SEL|BUCK2_EN,
		.enable_is_inverted = 1,
		.enable_mask = BUCK2_SEL,
		.owner = THIS_MODULE,
	},
	/*{
		.name = "BUCK3",
		.id = BD71840_BUCK3,
		.ops = &bd71840_buck1234_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_BUCK3_VOLTAGE_NUM,
		.linear_ranges = bd71840_buck1234_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71840_buck1234_voltage_ranges),
		.vsel_reg = BD71840_REG_BUCK3_VOLT_RUN,
		.vsel_mask = BUCK3_RUN_MASK,
		.enable_reg = BD71840_REG_BUCK3_CTRL,
		.enable_mask = BUCK3_SEL|BUCK3_EN,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK4",
		.id = BD71840_BUCK4,
		.ops = &bd71840_buck1234_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_BUCK4_VOLTAGE_NUM,
		.linear_ranges = bd71840_buck1234_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71840_buck1234_voltage_ranges),
		.vsel_reg = BD71840_REG_BUCK4_VOLT_RUN,
		.vsel_mask = BUCK4_RUN_MASK,
		.enable_reg = BD71840_REG_BUCK4_CTRL,
		.enable_mask = BUCK4_SEL|BUCK4_EN,
		.owner = THIS_MODULE,
	},*/
	{
		.name = "BUCK5",
		.id = BD71840_BUCK5,
		.ops = &bd71840_bd71847_buck5_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_BUCK5_VOLTAGE_NUM,
		.vsel_reg = BD71840_REG_BUCK5_VOLT,
		.vsel_mask = BUCK5_MASK,
		.enable_reg = BD71840_REG_BUCK5_CTRL,
		//.enable_mask = BUCK5_SEL|BUCK5_EN,
		.enable_is_inverted = 1,
		.enable_mask = BUCK5_SEL,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK6",
		.id = BD71840_BUCK6,
		.ops = &bd71840_bd71847_buck6_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_BUCK6_VOLTAGE_NUM,
		.vsel_reg = BD71840_REG_BUCK6_VOLT,
		.vsel_mask = BUCK6_MASK,
		.enable_reg = BD71840_REG_BUCK6_CTRL,
		//.enable_mask = BUCK6_SEL|BUCK6_EN,
		.enable_is_inverted = 1,
		.enable_mask = BUCK6_SEL,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK7",
		.id = BD71840_BUCK7,
		.ops = &bd71840_buck_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_BUCK7_VOLTAGE_NUM,
		.linear_ranges = bd71840_buck7_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71840_buck7_voltage_ranges),
		.vsel_reg = BD71840_REG_BUCK7_VOLT,
		.vsel_mask = BUCK7_MASK,
		.enable_reg = BD71840_REG_BUCK7_CTRL,
		//.enable_mask = BUCK7_SEL|BUCK7_EN,
		.enable_is_inverted = 1,
		.enable_mask = BUCK7_SEL,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK8",
		.id = BD71840_BUCK8,
		.ops = &bd71840_buck_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_BUCK8_VOLTAGE_NUM,
		.linear_ranges = bd71840_buck8_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71840_buck8_voltage_ranges),
		.vsel_reg = BD71840_REG_BUCK8_VOLT,
		.vsel_mask = BD71847_BUCK8_MASK,
		.enable_reg = BD71840_REG_BUCK8_CTRL,
		//.enable_mask = BUCK8_SEL|BUCK8_EN,
		.enable_is_inverted = 1,
		.enable_mask = BUCK8_SEL,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO1",
		.id = BD71840_LDO1,
		.ops = &bd71840_bd71847_ldo1_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_LDO1_VOLTAGE_NUM,
		.vsel_reg = BD71840_REG_LDO1_VOLT,
		.vsel_mask = LDO1_MASK,
		.enable_reg = BD71840_REG_LDO1_VOLT,
		//.enable_mask = LDO1_SEL|LDO1_EN,
		.enable_is_inverted = 1,
		.enable_mask = LDO1_SEL,
		.owner = THIS_MODULE,
	},
	/*
	 * LDO2 0.9V or 0.8V
	 * Fixed voltage
	 */
	{
		.name = "LDO2",
		.id = BD71840_LDO2,
		.ops = &bd71840_bd71847_ldo2_fixed_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_LDO2_VOLTAGE_NUM,
		.min_uV = 900000,
		.enable_reg = BD71840_REG_LDO2_VOLT,
		//.enable_mask = LDO2_SEL|LDO2_EN,
		.enable_is_inverted = 1,
		.enable_mask = LDO2_SEL,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO3",
		.id = BD71840_LDO3,
		.ops = &bd71840_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_LDO3_VOLTAGE_NUM,
		.linear_ranges = bd71840_ldo3_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71840_ldo3_voltage_ranges),
		.vsel_reg = BD71840_REG_LDO3_VOLT,
		.vsel_mask = LDO3_MASK,
		.enable_reg = BD71840_REG_LDO3_VOLT,
		//.enable_mask = LDO3_SEL|LDO3_EN,
		.enable_is_inverted = 1,
		.enable_mask = LDO3_SEL,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO4",
		.id = BD71840_LDO4,
		.ops = &bd71840_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_LDO4_VOLTAGE_NUM,
		.linear_ranges = bd71840_ldo4_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71840_ldo4_voltage_ranges),
		.vsel_reg = BD71840_REG_LDO4_VOLT,
		.vsel_mask = LDO4_MASK,
		.enable_reg = BD71840_REG_LDO4_VOLT,
		//.enable_mask = LDO4_SEL|LDO4_EN,
		.enable_is_inverted = 1,
		.enable_mask = LDO4_SEL,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO5",
		.id = BD71840_LDO5,
		.ops = &bd71840_bd71847_ldo5_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_LDO5_VOLTAGE_NUM,
		.vsel_reg = BD71840_REG_LDO5_VOLT,
		.vsel_mask = LDO5_MASK,
		.enable_reg = BD71840_REG_LDO5_VOLT,
		//.enable_mask = LDO5_SEL|LDO5_EN,
		.enable_is_inverted = 1,
		.enable_mask = LDO5_SEL,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO6",
		.id = BD71840_LDO6,
		.ops = &bd71840_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_LDO6_VOLTAGE_NUM,
		.linear_ranges = bd71840_ldo6_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71840_ldo6_voltage_ranges),
		.vsel_reg = BD71840_REG_LDO6_VOLT,
		.vsel_mask = LDO6_MASK,
		.enable_reg = BD71840_REG_LDO6_VOLT,
		//.enable_mask = LDO6_SEL|LDO6_EN,
		.enable_is_inverted = 1,
		.enable_mask = LDO6_SEL,
		.owner = THIS_MODULE,
	},
	/*{
		.name = "LDO7",
		.id = BD71840_LDO7,
		.ops = &bd71840_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71840_LDO7_VOLTAGE_NUM,
		.linear_ranges = bd71840_ldo7_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71840_ldo7_voltage_ranges),
		.vsel_reg = BD71840_REG_LDO7_VOLT,
		.vsel_mask = LDO7_MASK,
		.enable_reg = BD71840_REG_LDO7_VOLT,
		.enable_mask = LDO7_EN,
		.owner = THIS_MODULE,
	},*/
};

#ifdef CONFIG_OF

static struct of_regulator_match bd71840_bd71837_matches[] = {
	{ .name = "buck1",	},
	{ .name = "buck2",	},
	{ .name = "buck3",	},
	{ .name = "buck4",	},
	{ .name = "buck5",	},
	{ .name = "buck6",	},
	{ .name = "buck7",	},
	{ .name = "buck8",	},
	{ .name = "ldo1",	},
	{ .name = "ldo2",	},
	{ .name = "ldo3",	},
	{ .name = "ldo4",	},
	{ .name = "ldo5",	},
	{ .name = "ldo6",	},
	{ .name = "ldo7",	},
};

static struct of_regulator_match bd71840_bd71847_matches[] = {
	{ .name = "buck1",	},
	{ .name = "buck2",	},
	/*{ .name = "buck3",	},
	{ .name = "buck4",	},*/
	{ .name = "buck5",	},
	{ .name = "buck6",	},
	{ .name = "buck7",	},
	{ .name = "buck8",	},
	{ .name = "ldo1",	},
	{ .name = "ldo2",	},
	{ .name = "ldo3",	},
	{ .name = "ldo4",	},
	{ .name = "ldo5",	},
	{ .name = "ldo6",	},
	/*{ .name = "ldo7",	},*/
};

/**@brief parse bd71840 regulator device tree
 * @param pdev platform device of bd71840 regulator
 * @param bd71840_reg_matches return regualtor matches
 * @retval 0 parse success
 * @retval NULL parse fail
 */
static int bd71840_parse_dt_reg_data(
		struct platform_device *pdev,
		struct of_regulator_match **reg_matches)
{
	// struct bd71840 *bd71840 = dev_get_drvdata(pdev->dev.parent);
	struct device_node *np, *regulators;
	struct of_regulator_match *matches;
	int ret, count;

	np = of_node_get(pdev->dev.parent->of_node);
	regulators = of_find_node_by_name(np, "regulators");
	if (!regulators) {
		dev_err(&pdev->dev, "regulator node not found\n");
		return -EINVAL;
	}

	if(bd71840_otpver & BD71840_BD71847) {
		count = ARRAY_SIZE(bd71840_bd71847_matches);
		matches = bd71840_bd71847_matches;
	} else {
		count = ARRAY_SIZE(bd71840_bd71837_matches);
		matches = bd71840_bd71837_matches;
	}

	ret = of_regulator_match(&pdev->dev, regulators, matches, count);
	of_node_put(regulators);
	if (ret < 0) {
		dev_err(&pdev->dev, "Error parsing regulator init data: %d\n",
			ret);
		return ret;
	}

	*reg_matches = matches;

	return 0;
}
#else
static inline int bd71840_parse_dt_reg_data(
			struct platform_device *pdev,
			struct of_regulator_match **reg_matches)
{
	*reg_matches = NULL;
	return 0;
}
#endif

/** @brief retrive muxsw output value */
static ssize_t muxsw_show_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bd71840_pmic *pmic = dev_get_drvdata(dev);
	int o;

	o = bd71840_reg_read(pmic->mfd, BD71840_REG_MUXSW_EN);
	o = (o & MUXSW_EN) != 0;

	return sprintf(buf, "%d\n", o);
}

/** @brief set o output value */
static ssize_t muxsw_set_value(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct bd71840_pmic *pmic = dev_get_drvdata(dev);
	int o, r;

	if (sscanf(buf, "%d", &o) < 1) {
		return -EINVAL;
	}

	if (o != 0) {
		o = MUXSW_EN;
	}
	r = bd71840_update_bits(pmic->mfd, BD71840_REG_MUXSW_EN, MUXSW_EN, o);
	if (r < 0) {
		return r;
	}
	return count;
}

/** @brief retrive out32k output value */
static ssize_t out32_show_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bd71840_pmic *pmic = dev_get_drvdata(dev);
	int o;

	o = bd71840_reg_read(pmic->mfd, BD71840_REG_OUT32K);
	o = (o & OUT32K_EN) != 0;

	return sprintf(buf, "%d\n", o);
}

/** @brief set o output value */
static ssize_t out32_set_value(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct bd71840_pmic *pmic = dev_get_drvdata(dev);
	int o, r;

	if (sscanf(buf, "%d", &o) < 1) {
		return -EINVAL;
	}

	if (o != 0) {
		o = OUT32K_EN;
	}
	r = bd71840_update_bits(pmic->mfd, BD71840_REG_OUT32K, OUT32K_EN, o);
	if (r < 0) {
		return r;
	}
	return count;
}

/** @brief list all supported values */
static ssize_t available_values(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0 1 \n");
}

/** @brief directly set raw value to chip register, format: 'register value' */
static ssize_t bd71840_sysfs_set_registers(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	struct bd71840_pmic *pmic = dev_get_drvdata(dev);
	ssize_t ret = 0;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret < 1) {
		pmic->reg_index = -1;
		dev_err(pmic->dev, "registers set: <reg> <value>\n");
		return count;
	}

	if (ret == 1 && reg < BD71840_MAX_REGISTER) {
		pmic->reg_index = reg;
		dev_info(pmic->dev, "registers set: reg=0x%x\n", reg);
		return count;
	}

	if (reg > BD71840_MAX_REGISTER) {
		dev_err(pmic->dev, "reg=%d out of Max=%d\n", reg, BD71840_MAX_REGISTER);
		return -EINVAL;
	}
	dev_info(pmic->dev, "registers set: reg=0x%x, val=0x%x\n", reg, val);
	ret = bd71840_reg_write(pmic->mfd, reg, val);
	if (ret < 0)
		return ret;
	return count;
}

/** @brief print value of chip register, format: 'register=value' */
static ssize_t bd71840_sysfs_print_reg(struct bd71840_pmic *pmic,
				       u8 reg,
				       char *buf)
{
	int ret = bd71840_reg_read(pmic->mfd, reg);
	if (ret < 0)
		return sprintf(buf, "%#.2x=error %d\n", reg, ret);
	return sprintf(buf, "[0x%.2X] = %.2X\n", reg, ret);
}

/** @brief show all raw values of chip register, format per line: 'register=value' */
static ssize_t bd71840_sysfs_show_registers(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct bd71840_pmic *pmic = dev_get_drvdata(dev);
	ssize_t ret = 0;
	int i;

	dev_info(pmic->dev, "register: index[0x%x]\n", pmic->reg_index);
	if (pmic->reg_index >= 0) {
		ret += bd71840_sysfs_print_reg(pmic, pmic->reg_index, buf + ret);
	} else {
		for (i = 0; i < BD71840_MAX_REGISTER; i++) {
			ret += bd71840_sysfs_print_reg(pmic, i, buf + ret);
		}
	}
	return ret;
}

static DEVICE_ATTR(muxsw_value, S_IWUSR | S_IRUGO, muxsw_show_value, muxsw_set_value);
static DEVICE_ATTR(out32k_value, S_IWUSR | S_IRUGO, out32_show_value, out32_set_value);
static DEVICE_ATTR(available_value, S_IWUSR | S_IRUGO, available_values, NULL);
static DEVICE_ATTR(registers, S_IWUSR | S_IRUGO,
		bd71840_sysfs_show_registers, bd71840_sysfs_set_registers);

/** @brief device sysfs attribute table, about o */
static struct attribute *clk_attributes[] = {
	&dev_attr_muxsw_value.attr,
	&dev_attr_out32k_value.attr,
	&dev_attr_available_value.attr,
	&dev_attr_registers.attr,
	NULL
};

static const struct attribute_group clk_attr_group = {
	.attrs	= clk_attributes,
};

/*----------------------------------------------------------------------*/
#ifdef CONFIG_OF
/** @brief buck1/2 dvs enable/voltage from device tree
 * @param pdev platfrom device pointer
 * @param buck_dvs pointer
 * @return void
 */
static void of_bd71840_buck_dvs(struct platform_device *pdev, struct bd71840_buck_dvs *buck_dvs)
{
	struct device_node *pmic_np;

	pmic_np = of_node_get(pdev->dev.parent->of_node);
	if (!pmic_np) {
		dev_err(&pdev->dev, "could not find pmic sub-node\n");
		return;
	}

	if (of_get_property(pmic_np, "bd71840,pmic-buck1-uses-i2c-dvs", NULL)) {
		if (of_property_read_u32_array(pmic_np,
							"bd71840,pmic-buck1-dvs-voltage",
							&buck_dvs[0].voltage[0], BD71840_DVS_RUN_IDLE_SUSP)) {
			dev_err(&pdev->dev, "buck1 voltages not specified\n");
		}
	}

	if (of_get_property(pmic_np, "bd71840,pmic-buck2-uses-i2c-dvs", NULL)) {
		if (of_property_read_u32_array(pmic_np,
							"bd71840,pmic-buck2-dvs-voltage",
						&buck_dvs[1].voltage[0], BD71840_DVS_RUN_IDLE)) {
			dev_err(&pdev->dev, "buck2 voltages not specified\n");
		}
	}

	if(bd71840_otpver & BD71840_BD71847) {
		/* Do nothing */
	} else {
		if (of_get_property(pmic_np, "bd71840,pmic-buck3-uses-i2c-dvs", NULL)) {
			if (of_property_read_u32_array(pmic_np,
								"bd71840,pmic-buck3-dvs-voltage",
							&buck_dvs[2].voltage[0], BD71840_DVS_RUN)) {
				dev_err(&pdev->dev, "buck3 voltages not specified\n");
			}
		}
		if (of_get_property(pmic_np, "bd71840,pmic-buck4-uses-i2c-dvs", NULL)) {
			if (of_property_read_u32_array(pmic_np,
								"bd71840,pmic-buck4-dvs-voltage",
							&buck_dvs[3].voltage[0], BD71840_DVS_RUN)) {
				dev_err(&pdev->dev, "buck4 voltages not specified\n");
			}
		}
	}
}
#else
static void of_bd71840_buck_dvs(struct platform_device *pdev, struct bd71840_buck_dvs *buck_dvs)
{
	buck_dvs[0].voltage[0] = BUCK1_RUN_DEFAULT;
	buck_dvs[0].voltage[1] = BUCK1_IDLE_DEFAULT;
	buck_dvs[0].voltage[2] = BUCK1_SUSP_DEFAULT;
	buck_dvs[1].voltage[0] = BUCK2_RUN_DEFAULT;
	buck_dvs[1].voltage[1] = BUCK2_IDLE_DEFAULT;
	buck_dvs[1].voltage[2] = 0; /* Not supported */
	if(bd71840_otpver & BD71840_BD71847) {
		/* Do nothing */
	} else {
		buck_dvs[2].voltage[0] = BUCK3_RUN_DEFAULT;
		buck_dvs[2].voltage[1] = 0; /* Not supported */
		buck_dvs[2].voltage[2] = 0; /* Not supported */
		buck_dvs[3].voltage[0] = BUCK4_RUN_DEFAULT;
		buck_dvs[3].voltage[1] = 0; /* Not supported */
		buck_dvs[3].voltage[2] = 0; /* Not supported */
	}
}
#endif

static int bd71840_buck1234_dvs_init(struct bd71840_pmic *pmic)
{
	struct bd71840 *bd71840 = pmic->mfd;
	struct bd71840_buck_dvs *buck_dvs = &pmic->buck_dvs[0];
	int i, ret, val, selector = 0;
	u8 reg_run, reg_idle, reg_susp;
	u8 reg_run_msk, reg_idle_msk, reg_susp_msk;

	for(i = 0; i < BD71840_DVS_BUCK_NUM; i++, buck_dvs++) {
		if(bd71840_otpver & BD71840_BD71847) {
			if(i == 2 || i == 3)
				continue;
		}
		switch(i) {
		case 0:
		default:
			reg_run = BD71840_REG_BUCK1_VOLT_RUN;
			reg_run_msk = BUCK1_RUN_MASK;
			reg_idle = BD71840_REG_BUCK1_VOLT_IDLE;
			reg_idle_msk = BUCK1_IDLE_MASK;
			reg_susp = BD71840_REG_BUCK1_VOLT_SUSP;
			reg_susp_msk = BUCK1_SUSP_MASK;
			if(bd71840_otpver & BD71840_BD71847) {
				reg_run_msk = BD71847_BUCK1_RUN_MASK;
				reg_idle_msk = BD71847_BUCK1_IDLE_MASK;
				reg_susp_msk = BD71847_BUCK1_SUSP_MASK;
			}
			break;
		case 1:
			reg_run = BD71840_REG_BUCK2_VOLT_RUN;
			reg_run_msk = BUCK2_RUN_MASK;
			reg_idle = BD71840_REG_BUCK2_VOLT_IDLE;
			reg_idle_msk = BUCK2_IDLE_MASK;
			if(bd71840_otpver & BD71840_BD71847) {
				reg_run_msk = BD71847_BUCK2_RUN_MASK;
				reg_idle_msk = BD71847_BUCK2_IDLE_MASK;
			}
			reg_susp = 0;
			break;
		case 2:
			reg_run = BD71840_REG_BUCK3_VOLT_RUN;
			reg_run_msk = BUCK3_RUN_MASK;
			reg_idle = 0;
			reg_susp = 0;
			break;
		case 3:
			reg_run = BD71840_REG_BUCK4_VOLT_RUN;
			reg_run_msk = BUCK4_RUN_MASK;
			reg_idle = 0;
			reg_susp = 0;
			break;
		}

		dev_info(pmic->dev, "Buck%d: DVS Run-Idle-Susp[%d - %d - %d].\n", i+1, buck_dvs->voltage[0], buck_dvs->voltage[1], buck_dvs->voltage[2]);
		if(reg_run > 0) {
			selector = regulator_map_voltage_iterate(pmic->rdev[i], buck_dvs->voltage[0], buck_dvs->voltage[0]);
			if(selector < 0) {
				dev_err(pmic->dev, "%s(): not found selector for Run voltage [%d]\n", __func__, buck_dvs->voltage[0]);
			} else {
				val = (selector & reg_run_msk);
				ret = bd71840_reg_write(bd71840, reg_run, val);
				if(ret < 0)
					return ret;
			}
		}
		if(reg_idle > 0) {
			selector = regulator_map_voltage_iterate(pmic->rdev[i], buck_dvs->voltage[1], buck_dvs->voltage[1]);
			if(selector < 0) {
				dev_err(pmic->dev, "%s(): not found selector for Idle voltage [%d]\n", __func__, buck_dvs->voltage[1]);
			} else {
				val = (selector & reg_idle_msk);
				ret = bd71840_reg_write(bd71840, reg_idle, val);
				if(ret < 0)
					return ret;
			}
		}
		if(reg_susp > 0) {
			selector = regulator_map_voltage_iterate(pmic->rdev[i], buck_dvs->voltage[2], buck_dvs->voltage[2]);
			if(selector < 0) {
				dev_err(pmic->dev, "%s(): not found selector for Susp voltage [%d]\n", __func__, buck_dvs->voltage[2]);
			} else {
				val = (selector & reg_susp_msk);
				ret = bd71840_reg_write(bd71840, reg_susp, val);
				if(ret < 0)
					return ret;
			}
		}
	}
	return 0;
}

/**@brief bd71840 pmic interrupt
 * @param irq system irq
 * @param pwrsys bd71840 power device of system
 * @retval IRQ_HANDLED success
 * @retval IRQ_NONE error
 */
static irqreturn_t bd71840_pmic_interrupt(int irq, void *pwrsys)
{
	struct device *dev = pwrsys;
	struct bd71840 *mfd = dev_get_drvdata(dev->parent);
	// struct bd71840_power *pwr = dev_get_drvdata(dev);
	int reg;

	bd71840_debug(BD71840_DBG0, "bd71840_pmic_interrupt() in.\n");

	reg = bd71840_reg_read(mfd, BD71840_REG_IRQ);
	if (reg < 0)
		return IRQ_NONE;

	if(reg & IRQ_SWRST) {
		bd71840_debug(BD71840_DBG0, "IRQ_SWRST\n");
	}
	if(reg & IRQ_PWRON_S) {
		bd71840_debug(BD71840_DBG0, "IRQ_PWRON_S\n");
	}
	if(reg & IRQ_PWRON_L) {
		bd71840_debug(BD71840_DBG0, "IRQ_PWRON_L\n");
	}
	if(reg & IRQ_PWRON) {
		bd71840_debug(BD71840_DBG0, "IRQ_PWRON\n");
	}
	if(reg & IRQ_WDOG) {
		bd71840_debug(BD71840_DBG0, "IRQ_WDOG\n");
	}
	if(reg & IRQ_ON_REQ) {
		bd71840_debug(BD71840_DBG0, "IRQ_ON_REQ\n");
	}
	if(reg & IRQ_STBY_REQ) {
		bd71840_debug(BD71840_DBG0, "IRQ_STBY_REQ\n");
	}

	reg = bd71840_reg_write(mfd, BD71840_REG_IRQ, reg);
	if (reg < 0)
		return IRQ_NONE;

	return IRQ_HANDLED;
}

/**@brief probe bd71840 regulator device
 @param pdev bd71840 regulator platform device
 @retval 0 success
 @retval negative fail
*/
static __init int bd71840_probe(struct platform_device *pdev)
{
	struct bd71840_pmic *pmic;
	struct bd71840_board *pdata;
	struct regulator_config config = {};
	struct bd71840 *bd71840 = dev_get_drvdata(pdev->dev.parent);
	struct of_regulator_match *matches = NULL;
	struct of_regulator_match *bd71840_matches = NULL;
	int i = 0, err, irq = 0, ret = 0;

	pmic = kzalloc(sizeof(*pmic), GFP_KERNEL);
	if (!pmic) {
		dev_err(&pdev->dev, "Memory allocation failed for pmic\n");
		return -ENOMEM;
	}

	if(bd71840_otpver & BD71840_BD71847) {
		dev_info(&pdev->dev, "Regulator for %s registering...\n", "BD71847");
		memcpy(pmic->descs, bd71840_bd71847_regulators,	sizeof(pmic->descs));
		bd71840_matches = bd71840_bd71847_matches;
	} else {
		dev_info(&pdev->dev, "Regulator for %s registering...\n", "BD71837");
		memcpy(pmic->descs, bd71840_bd71837_regulators,	sizeof(pmic->descs));
		bd71840_matches = bd71840_bd71837_matches;
	}
	pmic->dev = &pdev->dev;
	pmic->mfd = bd71840;
	platform_set_drvdata(pdev, pmic);
	pdata = dev_get_platdata(bd71840->dev);

	if (!pdata && bd71840->dev->of_node) {
		bd71840_parse_dt_reg_data(pdev,	&matches);
		if (matches == NULL) {
			dev_err(&pdev->dev, "Platform data not found\n");
			return -EINVAL;
		}
	}

	/* Get buck dvs parameters */
	of_bd71840_buck_dvs(pdev, &pmic->buck_dvs[0]);

	/* Register LOCK release */
	err = bd71840_reg_write(bd71840, BD71840_REG_REGLOCK, 0x0);
	if (err != 0) {
		dev_err(&pdev->dev, "Failed to write LOCK register(%d)\n", err);
		goto err;
	}

#if 0 // Keep default
	/* Enable LDO5 */
	err = bd71840_set_bits(bd71840, BD71840_REG_LDO5_VOLT, LDO5_EN);
	if (err != 0) {
		dev_err(&pdev->dev, "Failed to enable LDO5 register(%d)\n", err);
		goto err;
	}
	/* Enable LDO6 */
	err = bd71840_set_bits(bd71840, BD71840_REG_LDO6_VOLT, LDO6_EN);
	if (err != 0) {
		dev_err(&pdev->dev, "Failed to enable LDO6 register(%d)\n", err);
		goto err;
	}

	if(bd71840_otpver & BD71840_BD71847) {
		/* Do nothing */
	} else {
		/* Enable LDO7 */
		err = bd71840_set_bits(bd71840, BD71840_REG_LDO7_VOLT, LDO7_EN);
		if (err != 0) {
			dev_err(&pdev->dev, "Failed to enable LDO7 register(%d)\n", err);
			goto err;
		}
	}
#endif

	for (i = 0; i < BD71840_REGULATOR_CNT; i++) {
		struct regulator_init_data *init_data;
		struct regulator_desc *desc;
		struct regulator_dev *rdev;

		desc = &pmic->descs[i];
		desc->name = bd71840_matches[i].name;
		
		if (pdata) {
			init_data = pdata->init_data[i];
		} else {
			init_data = matches[i].init_data;
		}

		config.dev = pmic->dev;
		config.init_data = init_data;
		config.driver_data = pmic;
		config.regmap = bd71840->regmap;
		config.of_node = matches[i].of_node;
		dev_info(config.dev, "regulator register name '%s'\n", desc->name);

		rdev = regulator_register(desc, &config);
		if (IS_ERR(rdev)) {
			dev_err(bd71840->dev,
				"failed to register %s regulator\n",
				desc->name);
			err = PTR_ERR(rdev);
			goto err;
		}
		pmic->rdev[i] = rdev;
	}

	/* Init sysfs registers */
	pmic->reg_index = -1;

	err = sysfs_create_group(&pdev->dev.kobj, &clk_attr_group);
	if (err != 0) {
		dev_err(&pdev->dev, "Failed to create attribute group: %d\n", err);
		goto err;
	}

	/* Init Buck1/2/3/4 dvs */
	err = bd71840_buck1234_dvs_init(pmic);
	if (err != 0) {
		dev_err(&pdev->dev, "Failed to buck12 dvs: %d\n", err);
		goto err;
	}

	/* Add Interrupt */
	irq  = platform_get_irq(pdev, 0); // get irq number
	if (irq <= 0) {
		dev_warn(&pdev->dev, "platform irq error # %d\n", irq);
		return -ENXIO;
	}
	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
			bd71840_pmic_interrupt, IRQF_TRIGGER_LOW | IRQF_EARLY_RESUME,
		dev_name(&pdev->dev), &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "IRQ %d is not free.\n", irq);
	}

	/* Un-mask IRQ Interrupt */
	ret = bd71840_reg_write(bd71840, BD71840_REG_MIRQ, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "Write Un-mask 'BD71840_REG_MIRQ': failed!\n");
		ret = -EIO;
		goto err;
	}

	return 0;

err:
	while (--i >= 0)
		regulator_unregister(pmic->rdev[i]);

	kfree(pmic);
	return err;
}

/**@brief remove bd71840 regulator device
 @param pdev bd71840 regulator platform device
 @return 0
*/
static int __exit bd71840_remove(struct platform_device *pdev)
{
	struct bd71840_pmic *pmic = platform_get_drvdata(pdev);
	int i;

	sysfs_remove_group(&pdev->dev.kobj, &clk_attr_group);

	for (i = 0; i < BD71840_REGULATOR_CNT; i++)
		regulator_unregister(pmic->rdev[i]);

	kfree(pmic);
	return 0;
}

static struct platform_driver bd71840_driver = {
	.driver = {
		.name = "bd71840-pmic",
		.owner = THIS_MODULE,
	},
	.probe = bd71840_probe,
	.remove = bd71840_remove,
};

/**@brief module initialize function */
static int __init bd71840_init(void)
{
	return platform_driver_register(&bd71840_driver);
}
subsys_initcall(bd71840_init);

/**@brief module deinitialize function */
static void __exit bd71840_cleanup(void)
{
	platform_driver_unregister(&bd71840_driver);
}
module_exit(bd71840_cleanup);

MODULE_AUTHOR("Cong Pham <cpham2403@gmail.com>");
MODULE_DESCRIPTION("BD71840 voltage regulator driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bd71840-pmic");

/*-------------------------------------------------------*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#define PROCFS_NAME 		"bd71840"
#define BD71840_REV			"BD71840 Driver: Rev004\n"

#define BD71840_BUF_SIZE	1024
static char procfs_buffer[BD71840_BUF_SIZE];
/**
 * This function is called then the /proc file is read
 *
 */
static int onetime = 0;
static ssize_t bd71840_proc_read (struct file *file, char __user *buffer, size_t count, loff_t *data)
{
	int ret = 0, error = 0;
	if(onetime==0) {
		onetime = 1;
		memset( procfs_buffer, 0, BD71840_BUF_SIZE);
		sprintf(procfs_buffer, "%s", BD71840_REV);
		ret = strlen(procfs_buffer);
		error = copy_to_user(buffer, procfs_buffer, strlen(procfs_buffer));
	} else {
		//Clear for next time
		onetime = 0;
	}
	return (error!=0)?0:ret;
}

static ssize_t bd71840_proc_write (struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	sscanf(buffer, "0x%x", &bd71840_debug_mask);
	printk("bd71840: bd71840_debug_mask=0x%08x\n", bd71840_debug_mask);
	return count;
}

static const struct file_operations bd71840_proc_fops = {
	.owner		= THIS_MODULE,
	.read		= bd71840_proc_read,
	.write		= bd71840_proc_write,
};

/**
 *This function is called when the module is loaded
 *
 */
int bd71840_revision_init(void)
{
	struct proc_dir_entry *bd71840_proc_entry;

	/* create the /proc/bd71840 */
	bd71840_proc_entry = proc_create(PROCFS_NAME, 0644, NULL, &bd71840_proc_fops);
	if (bd71840_proc_entry == NULL) {
		printk("Error: Could not initialize /proc/%s\n", PROCFS_NAME);
		return -ENOMEM;
	}

	return 0;
}
module_init(bd71840_revision_init);
/*-------------------------------------------------------*/
