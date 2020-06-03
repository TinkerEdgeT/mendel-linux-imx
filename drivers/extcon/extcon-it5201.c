/*
 * extcon-it5201.c - ITE CC logic for USB Type-C applications
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
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/usb/typec.h>


/* IT5201_REG_GCR */
#define VBUS_INTERRUPT (1 << 6)
/* IT5201_REG_IOCPCR */
#define TYPEC_INTERRUPT (1 << 7)
/* IT5201_REG_CC_STATUS */
#define IS_NOT_CONNECTED(val) (val == 0)
#define IS_UFP_ATTATCHED(val) (val&0x1)
#define IS_DFP_ATTATCHED(val) (val&0x2)

struct it5201_info {
	struct device *dev;
	struct extcon_dev *edev;
	struct typec_capability typec_caps;
	struct typec_port *typec_port;
	struct work_struct wq_detect_cable;
	struct regmap *regmap;
};

/* List of detectable cables */
static const unsigned int it5201_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

enum it5201_reg {
	IT5201_REG_CID7R = 0x0,
	IT5201_REG_CID6R = 0x1,
	IT5201_REG_CID5R = 0x2,
	IT5201_REG_CID4R = 0x3,
	IT5201_REG_CID3R = 0x4,
	IT5201_REG_CID2R = 0x5,
	IT5201_REG_CID1R = 0x6,
	IT5201_REG_CID0R = 0x7,
	IT5201_REG_CVR = 0xf,
	IT5201_REG_GCR = 0x10,
	IT5201_REG_VCR = 0x13,
	IT5201_REG_TCFSMCR0 = 0x14,
	IT5201_REG_TCFSMCR1 = 0x15,
	IT5201_REG_TCASR = 0x16,
	IT5201_REG_CCSRCVR = 0x18,
	IT5201_REG_CCSNKVR = 0x19,
	IT5201_REG_IOCPCR = 0x1a,
	IT5201_REG_SPSR = 0x1b,
};

static const struct regmap_config it5201_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = IT5201_REG_SPSR,
	.cache_type = REGCACHE_NONE,
};

static int it5201_clear_interrupt(struct it5201_info *info)
{
	unsigned int val;
	int ret;

	ret = regmap_write(info->regmap, IT5201_REG_IOCPCR, TYPEC_INTERRUPT);
	if (ret) {
		dev_err(info->dev, "failed to clear interrupt status:%d\n",
			ret);
	}

	ret = regmap_read(info->regmap, IT5201_REG_IOCPCR, &val);
	if (ret) {
		dev_err(info->dev, "failed to get interrupt status:%d\n",
			ret);
	}
	dev_info(info->dev, "ITE IT5201: clear IOCPCR status:0x%x\n", val);

	return (ret < 0) ? ret : (int)val;
}

static irqreturn_t it5201_i2c_irq_handler(int irq, void *dev_id)
{
	int ret;
	unsigned int val = 0;
	struct it5201_info *info = dev_id;

	queue_work(system_power_efficient_wq, &info->wq_detect_cable);

	it5201_clear_interrupt(info);

	return IRQ_HANDLED;
}

static void it5201_detect_cable(struct work_struct *work)
{
	struct it5201_info *info =
		container_of(work, struct it5201_info, wq_detect_cable);
	int ret;
	unsigned int val;

	ret = regmap_read(info->regmap, IT5201_REG_TCASR, &val);
	if (ret) {
		dev_err(info->dev, "failed to get CC status:%d\n", ret);
	}
	dev_info(info->dev, "ITE IT5201: CC status in detect_cable:0x%x\n", val);

	if (IS_UFP_ATTATCHED(val)) {
		dev_info(info->dev, "ITE IT5201: UFP_ATTATCHED\n");
		extcon_set_state_sync(info->edev, EXTCON_USB, false);
		extcon_set_state_sync(info->edev, EXTCON_USB_HOST, true);
	} else if (IS_DFP_ATTATCHED(val)) {
		dev_info(info->dev, "ITE IT5201: DFP_ATTATCHED\n");
		extcon_set_state_sync(info->edev, EXTCON_USB_HOST, false);
		extcon_set_state_sync(info->edev, EXTCON_USB, true);
	} else if (IS_NOT_CONNECTED(val)) {
		dev_info(info->dev, "ITE IT5201: NOT_CONNECTED\n");
		extcon_set_state_sync(info->edev, EXTCON_USB, false);
		extcon_set_state_sync(info->edev, EXTCON_USB_HOST, false);
	} else {
		dev_dbg(info->dev, "other CC status is :0x%x\n", val);
	}
}

static int it5201_setup_i2c_det(
	struct it5201_info *info, struct i2c_client *i2c)
{
	int ret;
	unsigned int val = 0;

	/* Set VBUS detect */
	ret = regmap_update_bits(info->regmap, IT5201_REG_GCR,
		VBUS_INTERRUPT, VBUS_INTERRUPT);
	if (ret) {
		dev_err(info->dev, "failed to enable vbus interrupt:%d\n", ret);
	}

	ret = regmap_read(info->regmap, IT5201_REG_GCR, &val);
	if (ret) {
		dev_err(info->dev, "failed to get General Control Reg:%d\n", ret);
		return ret;
	}
        dev_info(info->dev, "ITE IT5201: General Control Reg:0x%x\n", val);

	/* Clear chip interrupt status */
	ret = it5201_clear_interrupt(info);
	if (ret < 0) {
		dev_err(info->dev, "Failed to clear interrupt: %d\n", ret);
		return ret;
	}

	ret = devm_request_threaded_irq(info->dev, i2c->irq, NULL,
		it5201_i2c_irq_handler, IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
		dev_name(info->dev), info);
	if (ret < 0) {
		dev_err(info->dev, "Failed to register i2c interrupt: %d", ret);
		return ret;
	}

	return 0;
}

static int it5201_set_dr(
	const struct typec_capability *caps, enum typec_data_role role)
{
	struct it5201_info *info =
		container_of(caps, struct it5201_info, typec_caps);
	int ret;

	dev_err(info->dev, "TO DO: Should to change Type-C data role\n.");
/*
	if (role == TYPEC_DEVICE) {
		ret = regmap_update_bits(info->regmap, , );

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
		ret = regmap_update_bits(info->regmap, , );

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
*/	return 0;
}

struct typec_port *it5201_register_port(struct it5201_info *info)
{
	info->typec_caps.type = TYPEC_PORT_DRP;
	info->typec_caps.revision = 0x0110;
	info->typec_caps.prefer_role = TYPEC_SINK;
	info->typec_caps.dr_set = it5201_set_dr;

	info->typec_port = typec_register_port(info->dev, &info->typec_caps);
	if (!info->typec_port) {
		dev_err(info->dev, "Unable to register Type-C port.\n");
	}

	return info->typec_port;
}

static int it5201_i2c_probe(
	struct i2c_client *i2c, const struct i2c_device_id *id)
{

	struct device_node *np = i2c->dev.of_node;
	struct it5201_info *info;
	int ret;
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

	info->regmap = devm_regmap_init_i2c(i2c, &it5201_regmap_config);
	if (IS_ERR(info->regmap)) {
		ret = PTR_ERR(info->regmap);
		dev_err(info->dev, "failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	/* Allocate extcon device */
	info->edev = devm_extcon_dev_allocate(info->dev, it5201_extcon_cable);
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

	INIT_WORK(&info->wq_detect_cable, it5201_detect_cable);

	/* Register interrupt */
	if (i2c->irq) {
		ret = it5201_setup_i2c_det(info, i2c);
		if (ret) {
			dev_err(info->dev,
				"Failed to configure i2c interrupts: %d\n",
				ret);
			return ret;
		}
	}

	device_set_wakeup_capable(info->dev, true);

	/* Do cable detect now */
	it5201_detect_cable(&info->wq_detect_cable);

	/* Register port to sysfs */
	info->typec_port = it5201_register_port(info);

	return ret;
}

static int it5201_i2c_remove(struct i2c_client *i2c)
{
	return 0;
}

static const struct i2c_device_id it5201_id[] = {
	{ "it5201", 0 },
};
MODULE_DEVICE_TABLE(i2c, it5201_id);

#ifdef CONFIG_OF
static const struct of_device_id it5201_of_match[] = {
	{
		.compatible = "ite,it5201",
	},
	{},
};
MODULE_DEVICE_TABLE(of, it5201_of_match);
#endif

static struct i2c_driver it5201_i2c_driver = {
    .driver =
        {
            .name = "it5201",
            .of_match_table = of_match_ptr(it5201_of_match),
        },
    .probe = it5201_i2c_probe,
    .remove = it5201_i2c_remove,
    .id_table = it5201_id,
};
module_i2c_driver(it5201_i2c_driver);

MODULE_DESCRIPTION("ITE IT5201 CC logic driver for USB Type-C");
MODULE_AUTHOR("TzuWen Chang <TzuWen_Chang@asus.com>");
MODULE_LICENSE("GPL v2");
