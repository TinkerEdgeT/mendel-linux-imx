/*
 * @file bd71840.c  --  ROHM BD71840MWV mfd driver
 * 
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 * @author: cpham2403@gmail.com
 * Copyright 2018.
 */
#define DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/gpio.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/mfd/core.h>
#include <linux/mfd/bd71840.h>

/* Default enable debug message All Level */
unsigned int bd71840_debug_mask = BD71840_DBG0;
/*
 * OTPVER(0xFF) Register settings for different silicons:
 * [7] i.MX version: 0=i.MX8M Quad, 1=i.MX8M Mini
 * [6] PMIC hardware version: 0=BD71837, 1=BD71847
 * [5:3] micro-controller software version (sequencer setting)
 * [2:0] Register OTP version (initial voltage setting)
 */
unsigned int bd71840_otpver = 0;

/** @brief bd71840 irq resource */
static struct resource pmic_resources[] = {
	// irq# 0
	{
		.start	= BD71840_IRQ,
		.end	= BD71840_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

/** @brief bd71840 multi function cells */
static struct mfd_cell bd71840_mfd_cells[] = {
	{
		.name = "bd71840-pmic",
		.num_resources = ARRAY_SIZE(pmic_resources),
		.resources = &pmic_resources[0],
	},
};

/** @brief bd71840 irqs */
static const struct regmap_irq bd71840_irqs[] = {
	[BD71840_IRQ] = {
		.mask = BD71840_INT_MASK,
		.reg_offset = 0,
	},
};

/** @brief bd71840 irq chip definition */
static struct regmap_irq_chip bd71840_irq_chip = {
	.name = "bd71840",
	.irqs = bd71840_irqs,
	.num_irqs = ARRAY_SIZE(bd71840_irqs),
	.num_regs = 1,
	.irq_reg_stride = 1,
	.status_base = BD71840_REG_IRQ,
	.mask_base = BD71840_REG_MIRQ,
	.mask_invert = true,
	// .ack_base = BD71840_REG_IRQ,
};

/** @brief bd71840 irq initialize
 *  @param bd71840 bd71840 device to init
 *  @param bdinfo platform init data
 *  @retval 0 probe success
 *  @retval negative error number
 */
static int bd71840_irq_init(struct bd71840 *bd71840, struct bd71840_board* bdinfo)
{
	int irq;
	int ret = 0;

	if (!bdinfo) {
		dev_warn(bd71840->dev, "No interrupt support, no pdata\n");
		return -EINVAL;
	}
	
	dev_info(bd71840->dev, "gpio_intr = %d \n", bdinfo->gpio_intr);
	irq = gpio_to_irq(bdinfo->gpio_intr);

	bd71840->chip_irq = irq;
	dev_info(bd71840->dev, "chip_irq=%d \n", bd71840->chip_irq);
	ret = regmap_add_irq_chip(bd71840->regmap, bd71840->chip_irq,
		IRQF_ONESHOT | IRQF_TRIGGER_FALLING, bdinfo->irq_base,
		&bd71840_irq_chip, &bd71840->irq_data);
	if (ret < 0) {
		dev_warn(bd71840->dev, "Failed to add irq_chip %d\n", ret);
	}
	return ret;
}

/** @brief bd71840 irq initialize
 *  @param bd71840 bd71840 device to init
 *  @retval 0 probe success
 *  @retval negative error number
 */
static int bd71840_irq_exit(struct bd71840 *bd71840)
{
	if (bd71840->chip_irq > 0)
		regmap_del_irq_chip(bd71840->chip_irq, bd71840->irq_data);
	return 0;
}

/** @brief check whether volatile register 
 *  @param dev kernel device pointer
 *  @param reg register index
 */
static bool is_volatile_reg(struct device *dev, unsigned int reg)
{
	// struct bd71840 *bd71840 = dev_get_drvdata(dev);

	/*
	 * Caching all regulator registers.
	 */
	return true;
}

/** @brief regmap configures */
static const struct regmap_config bd71840_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_reg = is_volatile_reg,
	.max_register = BD71840_MAX_REGISTER - 1,
	.cache_type = REGCACHE_RBTREE,
};

#ifdef CONFIG_OF
static struct of_device_id bd71840_of_match[] = {
	{ .compatible = "rohm,bd71840", .data = (void *)0},
	{ },
};
MODULE_DEVICE_TABLE(of, bd71840_of_match);


/** @brief parse device tree data of bd71840
 *  @param client client object provided by system
 *  @param chip_id return chip id back to caller
 *  @return board initialize data
 */
static struct bd71840_board *bd71840_parse_dt(struct i2c_client *client,
						int *chip_id)
{
	struct device_node *np = client->dev.of_node;
	struct bd71840_board *board_info;
	unsigned int prop;
	const struct of_device_id *match;
	int r = 0;

	match = of_match_device(bd71840_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return NULL;
	}

	*chip_id  = (int)match->data;

	board_info = devm_kzalloc(&client->dev, sizeof(*board_info),
			GFP_KERNEL);
	if (!board_info) {
		dev_err(&client->dev, "Failed to allocate pdata\n");
		return NULL;
	}

	board_info->gpio_intr = of_get_named_gpio(np, "gpio_intr", 0);
	if (!gpio_is_valid(board_info->gpio_intr)) {
		dev_err(&client->dev, "no pmic intr pin available\n");
		goto err_intr;
	}

	r = of_property_read_u32(np, "irq_base", &prop);
	if (!r) {
		board_info->irq_base = prop;
	} else {
		board_info->irq_base = -1;
	}

	return board_info;

err_intr:
	devm_kfree(&client->dev, board_info);
	return NULL;
}
#endif

/** @brief probe bd71840 device
 *  @param i2c client object provided by system
 *  @param id chip id
 *  @retval 0 probe success
 *  @retval negative error number
 */
static int bd71840_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct bd71840 *bd71840;
	struct bd71840_board *pmic_plat_data;
	struct bd71840_board *of_pmic_plat_data = NULL;
	int chip_id = id->driver_data;
	int ret = 0, revision;

	pmic_plat_data = dev_get_platdata(&i2c->dev);

	if (!pmic_plat_data && i2c->dev.of_node) {
		pmic_plat_data = bd71840_parse_dt(i2c, &chip_id);
		of_pmic_plat_data = pmic_plat_data;
	}

	if (!pmic_plat_data)
		return -EINVAL;

	bd71840 = kzalloc(sizeof(struct bd71840), GFP_KERNEL);
	if (bd71840 == NULL)
		return -ENOMEM;

	bd71840->of_plat_data = of_pmic_plat_data;
	i2c_set_clientdata(i2c, bd71840);
	bd71840->dev = &i2c->dev;
	bd71840->i2c_client = i2c;
	bd71840->id = chip_id;
	mutex_init(&bd71840->io_mutex);

	bd71840->regmap = devm_regmap_init_i2c(i2c, &bd71840_regmap_config);
	if (IS_ERR(bd71840->regmap)) {
		ret = PTR_ERR(bd71840->regmap);
		dev_err(&i2c->dev, "regmap initialization failed: %d\n", ret);
		return ret;
	}

	ret = bd71840_reg_read(bd71840, BD71840_REG_REV);
	if(ret < 0) {
		dev_err(bd71840->dev, "%s(): Read BD71840_REG_DEVICE failed!\n", __func__);
		goto err;
	}
	revision = ret;

	ret = bd71840_reg_read(bd71840, BD71840_REG_OTPVER);
	if(ret < 0) {
		dev_err(bd71840->dev, "%s(): Read BD71840_REG_DEVICE failed!\n", __func__);
		goto err;
	}
	bd71840_otpver = ret;
	dev_info(bd71840->dev, "Device Rev=0x%X, OTPVER=0x%X, %s driver\n",
			revision, bd71840_otpver, (bd71840_otpver&BD71840_BD71847)?"BD71847":"BD71837");

	bd71840_irq_init(bd71840, of_pmic_plat_data);

	ret = mfd_add_devices(bd71840->dev, -1,
			      bd71840_mfd_cells, ARRAY_SIZE(bd71840_mfd_cells),
			      NULL, 0,
			      regmap_irq_get_domain(bd71840->irq_data));
	if (ret < 0)
		goto err;

	return ret;

err:
	mfd_remove_devices(bd71840->dev);
	kfree(bd71840);
	return ret;
}

/** @brief remove bd71840 device
 *  @param i2c client object provided by system
 *  @return 0
 */
static int bd71840_i2c_remove(struct i2c_client *i2c)
{
	struct bd71840 *bd71840 = i2c_get_clientdata(i2c);

	bd71840_irq_exit(bd71840);
	mfd_remove_devices(bd71840->dev);
	kfree(bd71840);

	return 0;
}

static const struct i2c_device_id bd71840_i2c_id[] = {
	{ "bd71840", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bd71840_i2c_id);

static struct i2c_driver bd71840_i2c_driver = {
	.driver = {
		.name = "bd71840",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(bd71840_of_match),
	},
	.probe = bd71840_i2c_probe,
	.remove = bd71840_i2c_remove,
	.id_table = bd71840_i2c_id,
};

static int __init bd71840_i2c_init(void)
{
	return i2c_add_driver(&bd71840_i2c_driver);
}
/* init early so consumer devices can complete system boot */
subsys_initcall(bd71840_i2c_init);

static void __exit bd71840_i2c_exit(void)
{
	i2c_del_driver(&bd71840_i2c_driver);
}
module_exit(bd71840_i2c_exit);

MODULE_AUTHOR("Cong Pham <cpham2403@gmail.com>");
MODULE_DESCRIPTION("BD71840 chip multi-function driver");
MODULE_LICENSE("GPL");
