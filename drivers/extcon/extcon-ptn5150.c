/*
 * extcon-ptn5150.c - NXP CC logic for USB Type-C applications
 *
 * Copyright 2017 NXP
 * Author: Peter Chen <peter.chen@nxp.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/err.h>
#include <linux/extcon.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/usb/typec.h>

/* PTN_REG_CONTROL */
#define CONTROL_PORT_STATE_MASK (3 << 1)
#define CONTROL_PORT_STATE_UFP (0 << 1)
#define CONTROL_PORT_STATE_DFP (1 << 1)
#define CONTROL_PORT_STATE_DRP (2 << 1)
#define CONTROL_PORT_INT_MASK (1 << 0)
/* PTN5150_REG_INT_STATUS */
#define CABLE_ATTACHED (1 << 0)
#define CABLE_DETACHED (1 << 1)
/* PTN5150_REG_CC_STATUS */
#define IS_DFP_ATTATCHED(val) (((val)&0x1c) == 0x4)
#define IS_UFP_ATTATCHED(val) (((val)&0x1c) == 0x8)
#define IS_NOT_CONNECTED(val) (((val)&0x1c) == 0x0)
/* PTN5150_REG_CON_DET */
#define DISABLE_CON_DET (1 << 0)
/* PTN5150_REG_INT_MASK */
#define MASK_COMP_CHANGE (1 << 4)
#define MASK_ROLE_CHANGE (1 << 3)
#define MASK_ORIENT_FOUND (1 << 2)
#define MASK_DEBUG_FOUND (1 << 1)
#define MASK_AUDIO_FOUND (1 << 0)

struct ptn5150_info {
	struct device *dev;
	struct extcon_dev *edev;
	struct typec_capability typec_caps;
	struct typec_port *typec_port;
	struct work_struct wq_detect_cable;
	struct regmap *regmap;
};

/* List of detectable cables */
static const unsigned int ptn5150_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

enum ptn5150_reg {
	PTN5150_REG_DEVICE_ID = 0x1,
	PTN5150_REG_CONTROL,
	PTN5150_REG_INT_STATUS,
	PTN5150_REG_CC_STATUS,
	PTN5150_REG_RSVD_5,
	PTN5150_REG_RSVD_6,
	PTN5150_REG_RSVD_7,
	PTN5150_REG_RSVD_8,
	PTN5150_REG_CON_DET,
	PTN5150_REG_VCONN_STATUS,
	PTN5150_REG_RESET = 0x10,
	PTN5150_REG_RSVD_11,
	PTN5150_REG_RSVD_12,
	PTN5150_REG_RSVD_13,
	PTN5150_REG_RSVD_14,
	PTN5150_REG_RSVD_15,
	PTN5150_REG_RSVD_16,
	PTN5150_REG_RSVD_17,
	PTN5150_REG_INT_MASK,
	PTN5150_REG_INT_REG_STATUS,

	PTN5150_REG_END,
};

static const struct regmap_config ptn5150_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = PTN5150_REG_END,
	.cache_type = REGCACHE_NONE,
};

static irqreturn_t ptn5150_i2c_irq_handler(int irq, void *dev_id)
{
	int ret;
	unsigned int val = 0;
	struct ptn5150_info *info = dev_id;

	ret = regmap_read(info->regmap, PTN5150_REG_INT_REG_STATUS, &val);
	if (ret) {
		dev_err(info->dev, "Failed to read interrupt status: %d", ret);
		return IRQ_NONE;
	}

	queue_work(system_power_efficient_wq, &info->wq_detect_cable);
	return IRQ_HANDLED;
}

static void ptn5150_detect_cable(struct work_struct *work)
{
	struct ptn5150_info *info =
		container_of(work, struct ptn5150_info, wq_detect_cable);
	int ret;
	unsigned int val;

	ret = regmap_read(info->regmap, PTN5150_REG_CC_STATUS, &val);
	if (ret) {
		dev_err(info->dev, "failed to get CC status:%d\n", ret);
	}

	if (IS_UFP_ATTATCHED(val)) {
		extcon_set_state_sync(info->edev, EXTCON_USB, false);
		extcon_set_state_sync(info->edev, EXTCON_USB_HOST, true);
	} else if (IS_DFP_ATTATCHED(val)) {
		extcon_set_state_sync(info->edev, EXTCON_USB_HOST, false);
		extcon_set_state_sync(info->edev, EXTCON_USB, true);
	} else if (IS_NOT_CONNECTED(val)) {
		extcon_set_state_sync(info->edev, EXTCON_USB, false);
		extcon_set_state_sync(info->edev, EXTCON_USB_HOST, false);
	} else {
		dev_dbg(info->dev, "other CC status is :0x%x\n", val);
	}
}

static int ptn5150_clear_interrupt(struct ptn5150_info *info)
{
	unsigned int val;
	int ret;

	ret = regmap_read(info->regmap, PTN5150_REG_INT_STATUS, &val);
	if (ret) {
		dev_err(info->dev, "failed to clear interrupt status:%d\n",
			ret);
	}

	return (ret < 0) ? ret : (int)val;
}

static irqreturn_t ptn5150_connect_irq_handler(int irq, void *dev_id)
{
	struct ptn5150_info *info = dev_id;

	if (ptn5150_clear_interrupt(info) > 0) {
		queue_work(system_power_efficient_wq, &info->wq_detect_cable);
	}

	return IRQ_HANDLED;
}

static int ptn5150_setup_con_det(
	struct ptn5150_info *info, struct gpio_desc *connect_gpiod)
{
	int ret, connect_irq, gpio_val, count = 1000;

	connect_irq = gpiod_to_irq(connect_gpiod);
	if (connect_irq < 0) {
		dev_err(info->dev, "failed to get connect IRQ\n");
		return connect_irq;
	}

	/* Clear the pending interrupts */
	ret = ptn5150_clear_interrupt(info);
	if (ret < 0) {
		return ret;
	}

	gpio_val = gpiod_get_value(connect_gpiod);
	/* Delay until the GPIO goes to high if it is low before */
	while (gpio_val == 0 && count >= 0) {
		gpio_val = gpiod_get_value(connect_gpiod);
		usleep_range(10, 20);
		count--;
	}

	if (count < 0) {
		dev_err(info->dev, "timeout for waiting gpio becoming high\n");
	}

	ret = regmap_update_bits(info->regmap, PTN5150_REG_CON_DET,
		DISABLE_CON_DET, ~DISABLE_CON_DET);
	if (ret) {
		dev_err(info->dev,
			"failed to configure CON_DET output on pin 5:%d\n",
			ret);
		return ret;
	}

	ret = devm_request_threaded_irq(info->dev, connect_irq, NULL,
		ptn5150_connect_irq_handler, IRQF_TRIGGER_LOW | IRQF_ONESHOT,
		dev_name(info->dev), info);
	if (ret < 0) {
		dev_err(info->dev, "failed to request connect IRQ\n");
		return ret;
	}

	return 0;
}

static int ptn5150_setup_i2c_det(
	struct ptn5150_info *info, struct i2c_client *i2c)
{
	int ret;

	ret = regmap_update_bits(info->regmap, PTN5150_REG_INT_MASK,
		(MASK_ROLE_CHANGE | MASK_COMP_CHANGE | MASK_ORIENT_FOUND),
		~(MASK_ROLE_CHANGE | MASK_COMP_CHANGE | MASK_ORIENT_FOUND));
	if (ret) {
		dev_err(info->dev, "Failed to set I2C interrupt masks: %d",
			ret);
		return ret;
	}

	ret = regmap_update_bits(info->regmap, PTN5150_REG_CONTROL,
		CONTROL_PORT_STATE_MASK | CONTROL_PORT_INT_MASK,
		CONTROL_PORT_STATE_UFP | CONTROL_PORT_INT_MASK);
	if (ret) {
		dev_err(info->dev, "Failed to set port mode: %d", ret);
		return ret;
	}

	ret = devm_request_threaded_irq(info->dev, i2c->irq, NULL,
		ptn5150_i2c_irq_handler, IRQF_ONESHOT | IRQF_TRIGGER_LOW,
		dev_name(info->dev), info);
	if (ret < 0) {
		dev_err(info->dev, "Failed to register i2c interrupt: %d", ret);
		return ret;
	}

	return 0;
}

static int ptn5150_set_dr(
	const struct typec_capability *caps, enum typec_data_role role)
{
	struct ptn5150_info *info =
		container_of(caps, struct ptn5150_info, typec_caps);
	int ret;

	if (role == TYPEC_DEVICE) {
		ret = regmap_update_bits(info->regmap, PTN5150_REG_CONTROL,
			CONTROL_PORT_STATE_MASK | CONTROL_PORT_INT_MASK,
			CONTROL_PORT_STATE_UFP | CONTROL_PORT_INT_MASK);

		if (ret < 0) {
			dev_err(info->dev,
				"Failed to change data role to device: %d\n",
				ret);
			return ret;
		}
		typec_set_data_role(info->typec_port, TYPEC_DEVICE);
		dev_dbg(info->dev, "Setting Type-C port to peripheral mode.\n");
	}

	else if (role == TYPEC_HOST) {
		ret = regmap_update_bits(info->regmap, PTN5150_REG_CONTROL,
			CONTROL_PORT_STATE_MASK | CONTROL_PORT_INT_MASK,
			CONTROL_PORT_STATE_DFP | CONTROL_PORT_INT_MASK);

		if (ret < 0) {
			dev_err(info->dev,
				"Failed to change data role to host: %d\n",
				ret);
			return ret;
		}
		typec_set_data_role(info->typec_port, TYPEC_HOST);
		dev_dbg(info->dev, "Setting Type-C port to host mode.\n");
	}

	else {
		dev_err(info->dev, "Unable to change Type-C data role\n.");
		return -1;
	}
	return 0;
}

struct typec_port *ptn5150_register_port(struct ptn5150_info *info)
{
	info->typec_caps.type = TYPEC_PORT_DRP;
	info->typec_caps.revision = 0x0110;
	info->typec_caps.prefer_role = TYPEC_SINK;
	info->typec_caps.dr_set = ptn5150_set_dr;

	info->typec_port = typec_register_port(info->dev, &info->typec_caps);
	if (!info->typec_port) {
		dev_err(info->dev, "Unable to register Type-C port.\n");
	}

	return info->typec_port;
}

static int ptn5150_i2c_probe(
	struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct device_node *np = i2c->dev.of_node;
	struct ptn5150_info *info;
	int ret;
	unsigned int dev_id;
	struct gpio_desc *connect_gpiod;
	if (!np) {
		return -EINVAL;
	}

	info = devm_kzalloc(&i2c->dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		return -ENOMEM;
	}

	i2c_set_clientdata(i2c, info);
	info->dev = &i2c->dev;
	info->regmap = devm_regmap_init_i2c(i2c, &ptn5150_regmap_config);
	if (IS_ERR(info->regmap)) {
		ret = PTR_ERR(info->regmap);
		dev_err(info->dev, "failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	ret = regmap_update_bits(info->regmap, PTN5150_REG_RESET, 1, 1);
	usleep_range(500, 500);

	/* Allocate extcon device */
	info->edev = devm_extcon_dev_allocate(info->dev, ptn5150_extcon_cable);
	if (IS_ERR(info->edev)) {
		dev_err(info->dev, "failed to allocate memory for extcon\n");
		return -ENOMEM;
	}

	/* Register extcon device */
	ret = devm_extcon_dev_register(info->dev, info->edev);
	if (ret) {
		dev_err(info->dev, "failed to register extcon device\n");
		return ret;
	}

	INIT_WORK(&info->wq_detect_cable, ptn5150_detect_cable);

	connect_gpiod = devm_gpiod_get(info->dev, "connect", GPIOD_IN);
	if (!IS_ERR(connect_gpiod)) {
		ret = ptn5150_setup_con_det(info, connect_gpiod);
		if (ret) {
			dev_err(info->dev,
				"Failed to setup connection detection feature\n");
			return ret;
		}
	}

	ret = regmap_read(info->regmap, PTN5150_REG_DEVICE_ID, &dev_id);
	if (ret) {
		dev_err(info->dev, "failed to read device id:%d\n", ret);
		return ret;
	}

	dev_dbg(info->dev, "NXP PTN5150: Version ID:0x%x, Vendor ID:0x%x\n",
		(dev_id >> 3), (dev_id & 0x3));

	if (i2c->irq) {
		ret = ptn5150_setup_i2c_det(info, i2c);
		if (ret) {
			dev_err(info->dev,
				"Failed to configure i2c interrupts: %d\n",
				ret);
			return ret;
		}
	}

	ret = ptn5150_clear_interrupt(info);
	if (ret < 0) {
		dev_err(info->dev, "Failed to clear interrupt: %d\n", ret);
		return ret;
	}

	device_set_wakeup_capable(info->dev, true);

	/* Do cable detect now */
	ptn5150_detect_cable(&info->wq_detect_cable);

	/* Register port to sysfs */
	info->typec_port = ptn5150_register_port(info);

	return ret;
}

static int ptn5150_i2c_remove(struct i2c_client *i2c)
{
	return 0;
}

static const struct i2c_device_id ptn5150_id[] = {
	{ "ptn5150", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, ptn5150_id);

#ifdef CONFIG_OF
static const struct of_device_id ptn5150_of_match[] = {
	{
		.compatible = "nxp,ptn5150",
	},
	{
		.compatible = "nxp,ptn5150a",
	},
	{},
};
MODULE_DEVICE_TABLE(of, ptn5150_of_match);
#endif

static struct i2c_driver ptn5150_i2c_driver = {
    .driver =
        {
            .name = "ptn5150",
            .of_match_table = of_match_ptr(ptn5150_of_match),
        },
    .probe = ptn5150_i2c_probe,
    .remove = ptn5150_i2c_remove,
    .id_table = ptn5150_id,
};
module_i2c_driver(ptn5150_i2c_driver);

MODULE_DESCRIPTION("NXP PTN5150 CC logic driver for USB Type-C");
MODULE_AUTHOR("Peter Chen <peter.chen@nxp.com>");
MODULE_LICENSE("GPL v2");
