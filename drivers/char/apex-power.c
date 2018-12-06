/*
 * Driver to expose PCI-express hot-removal of devices via file
 * handles.
 *
 * Copyright (C) 2018 Google, Inc.
 * Author: June Tate-Gans <jtgans@google.com>
 *
 * Shamelessly the general structure of the character driver has been
 * borrowed from bsr.c, the POWER architecture's method for accessing
 * BSR registers.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/types.h>
#include <linux/module.h>

#include <asm/cmpxchg.h>
#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#if defined(CONFIG_IMX8MQ_PHANBELL_POWERSAVE)
#include <linux/busfreq-imx.h>
#endif

#define APEX_PCI_VENDOR_ID 0x1ac1
#define APEX_PCI_DEVICE_ID 0x089a

#define APEX_ALLOWED_RETRIES 5
#define APEX_RETRY_DELAY_MS 100

static atomic_t apex_power_minor = ATOMIC_INIT(0);
static struct class *apex_power_class;
static int apex_power_major;

struct apex_power_priv {
	struct platform_device *pdev;
	struct device *device;
	struct cdev cdev;
	bool owned;
	struct mutex owned_lock;
	struct delayed_work delayed_init;
	struct regulator *supply;
};

static struct pci_dev *get_apex_pci_device(struct apex_power_priv *apex_power_data, bool rescan) {
	int retries = 0;
	struct pci_bus *pci_bus = NULL;
	struct pci_dev *apex_dev = NULL;

	for (retries = 0; retries < APEX_ALLOWED_RETRIES; retries++) {
		/* For powering up, rescan the pci bus each retry */
		if (rescan) {
			pci_lock_rescan_remove();
			while ((pci_bus = pci_find_next_bus(pci_bus)) != NULL) {
				pci_rescan_bus(pci_bus);
			}
			pci_unlock_rescan_remove();
		}

		apex_dev = pci_get_device(APEX_PCI_VENDOR_ID, APEX_PCI_DEVICE_ID, NULL);
		if (apex_dev) {
			break;
		}
		dev_err(&apex_power_data->pdev->dev, "Unable to find apex device, retrying");
		msleep(APEX_RETRY_DELAY_MS);
	}
	return apex_dev;
}

static int apex_power_down(struct apex_power_priv *apex_power_data)
{
	struct pci_dev *apex_dev = NULL;
	struct pci_dev *apex_connected_bus = NULL;
	int ret = 0;

#if defined(CONFIG_IMX8MQ_PHANBELL_POWERSAVE)
	if (apex_power_data->owned) {
		release_bus_freq(BUS_FREQ_HIGH);
	}
#endif

	apex_dev = get_apex_pci_device(apex_power_data, /*rescan=*/ false);
	if (!apex_dev) {
		dev_err(&apex_power_data->pdev->dev, "can't find Apex on PCI bus?!");
		return -EIO;
	}
	apex_connected_bus = apex_dev->bus->self;
	pci_dev_put(apex_dev);

	pci_stop_and_remove_bus_device_locked(apex_connected_bus);

	ret = regulator_disable(apex_power_data->supply);
	if (ret) {
		dev_err(&apex_power_data->pdev->dev, "Unable to disable regulator.");
	}

	return 0;
}

static int apex_power_up(struct apex_power_priv *apex_power_data)
{
	struct pci_dev *apex_dev = NULL;
	int ret = 0;

	ret = regulator_enable(apex_power_data->supply);
	if (ret) {
		dev_err(&apex_power_data->pdev->dev, "Unable to enable regulator.");
	}

#if defined(CONFIG_IMX8MQ_PHANBELL_POWERSAVE)
	request_bus_freq(BUS_FREQ_HIGH);
#endif

	apex_dev = get_apex_pci_device(apex_power_data, /*rescan=*/ true);
	if (!apex_dev) {
		dev_err(&apex_power_data->pdev->dev, "can't find Apex on PCI bus?!");

#if defined(CONFIG_IMX8MQ_PHANBELL_POWERSAVE)
		release_bus_freq(BUS_FREQ_HIGH);
#endif

		return -EIO;
	}
	pci_dev_put(apex_dev);

	return 0;
}

static int apex_power_release(struct inode* inode, struct file* filep)
{
	int ret = 0;
	struct apex_power_priv *apex_power_data = 
		container_of(inode->i_cdev, struct apex_power_priv, cdev);
	if (mutex_lock_interruptible(&apex_power_data->owned_lock)) {
		return -ENOLCK;
	}

	if (apex_power_data->owned != true) {
		ret = -EPERM;
		goto out;
	}

	ret = apex_power_down(apex_power_data);
	apex_power_data->owned = false;

out:
	mutex_unlock(&apex_power_data->owned_lock);
	return ret;
}

static int apex_power_open(struct inode* inode, struct file* filep)
{
	int ret = 0;
	struct apex_power_priv *apex_power_data = 
		container_of(inode->i_cdev, struct apex_power_priv, cdev);

	if (mutex_lock_interruptible(&apex_power_data->owned_lock)) {
		return -ENOLCK;
	}

	if (apex_power_data->owned != false) {
		ret = -EPERM;
		goto out;
	}
	apex_power_data->owned = true;

	/* Ensure that we don't accidentally run afoul of the initial delayed
	   power down. */
	cancel_delayed_work_sync(&apex_power_data->delayed_init);

	ret = apex_power_up(apex_power_data);

out:
	mutex_unlock(&apex_power_data->owned_lock);
	return ret;
}

static const struct file_operations apex_power_fops = {
	.owner = THIS_MODULE,
	.open = apex_power_open,
	.release = apex_power_release,
};

static void apex_power_delayed_init_callback(struct work_struct *work)
{
	struct pci_dev *apex_dev = NULL;
	struct apex_power_priv *apex_power_data =
		container_of(
			container_of(work, struct delayed_work, work),
			struct apex_power_priv,
			delayed_init);
	apex_dev = pci_get_device(APEX_PCI_VENDOR_ID, APEX_PCI_DEVICE_ID, NULL);

	if (!apex_dev) {
		dev_info(&apex_power_data->pdev->dev, "rescheduling late init power down");
		schedule_delayed_work(&apex_power_data->delayed_init,
				      msecs_to_jiffies(1000));
		return;
	}

	pci_dev_put(apex_dev);

	dev_info(&apex_power_data->pdev->dev, "init routines powering down apex");

	if (mutex_lock_interruptible(&apex_power_data->owned_lock)) {
		dev_err(&apex_power_data->pdev->dev, "Unable to lock mutex.");
		return;
	}

	apex_power_down(apex_power_data);

	mutex_unlock(&apex_power_data->owned_lock);
}

static int apex_power_probe(struct platform_device *pdev)
{
	dev_t apex_power_dev;
	int ret = 0;
	struct apex_power_priv *apex_power_data = 
		devm_kzalloc(&pdev->dev, sizeof(struct apex_power_priv), GFP_KERNEL);
	if (!apex_power_data) {
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, apex_power_data);
	apex_power_data->pdev = pdev;
	mutex_init(&apex_power_data->owned_lock);

	// Enable the supply in regulator framework to ensure power isn't removed
	// until init is complete.
	apex_power_data->supply = regulator_get_exclusive(&pdev->dev, "power");
	if (IS_ERR(apex_power_data->supply)) {
		dev_err(&pdev->dev, "Unable to find regulator.");
		return -ENODEV;
	}
	
	ret = regulator_enable(apex_power_data->supply);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable regulator.");
		return -ENODEV;
	}

	apex_power_dev = MKDEV(apex_power_major, atomic_inc_return(&apex_power_minor));
	cdev_init(&apex_power_data->cdev, &apex_power_fops);
	ret = cdev_add(&apex_power_data->cdev, apex_power_dev, 1);

	if (ret) {
		dev_err(&pdev->dev, "Unable to add cdev!");
	}

	apex_power_data->device = device_create(apex_power_class, NULL, apex_power_dev,
					  NULL, "%s", dev_name(&pdev->dev));
	if (IS_ERR(apex_power_data->device)) {
		dev_err(&pdev->dev, "device_create failed.");
		cdev_del(&apex_power_data->cdev);
	}

	INIT_DELAYED_WORK(&apex_power_data->delayed_init,
			  apex_power_delayed_init_callback);
	schedule_delayed_work(&apex_power_data->delayed_init, msecs_to_jiffies(7000));

	dev_info(&pdev->dev, "initialized.");

	return 0;
}

static int apex_power_remove(struct platform_device *pdev)
{
	struct apex_power_priv *apex_power_data = platform_get_drvdata(pdev); 
	device_del(apex_power_data->device);
	cdev_del(&apex_power_data->cdev);
	regulator_disable(apex_power_data->supply);
	regulator_put(apex_power_data->supply);
	return 0;
}

static const struct of_device_id apex_power_dt_ids[] = {
	{ .compatible = "google,apex-power", },
	{}
};
MODULE_DEVICE_TABLE(of, apex_power_dt_ids);

static struct platform_driver apex_power_driver = {
	.driver = {
		.name = "apex-power",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(apex_power_dt_ids),
	},
	.probe = apex_power_probe,
	.remove = apex_power_remove,
};

static int __init apex_power_init(void) {
	int ret;
	dev_t apex_power_dev;
	apex_power_class = class_create(THIS_MODULE, "apex_power");
	if (IS_ERR(apex_power_class)) {
		printk(KERN_ERR "apex_power: could not allocate device class\n");
		return -ENODEV;
	}
	ret = alloc_chrdev_region(&apex_power_dev, 0, 255, "apex_power");
	apex_power_major = MAJOR(apex_power_dev);
	if (ret < 0) {
		printk(KERN_ERR "alloc_chrdev_region() failed\n");
		class_destroy(apex_power_class);
		return -ENODEV;
	}

	return platform_driver_register(&apex_power_driver);
}
module_init(apex_power_init);

static void __exit apex_power_exit(void) {
	platform_driver_unregister(&apex_power_driver);
	unregister_chrdev_region(MKDEV(apex_power_major, 0), 1);
	class_destroy(apex_power_class);
}
module_exit(apex_power_exit);

MODULE_DESCRIPTION("Google Apex power management driver");
MODULE_AUTHOR("June Tate-Gans <jtgans@google.com>");
MODULE_ALIAS("platform:apex-power");
MODULE_LICENSE("GPL");
