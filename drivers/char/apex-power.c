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
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
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

static struct class *apex_power_class;
static struct device *apex_power_device;
static struct cdev apex_power_cdev;
static int apex_power_major;
static bool apex_power_owned;
static struct delayed_work apex_power_delayed_init;

static struct regulator *get_apex_regulator(void) {
	struct regulator *supply;
	supply = regulator_get(NULL, "apex_regulators");
	if (IS_ERR(supply)) {
		printk(KERN_ERR "apex_power: Unable to find regulator.\n");
	}
	return supply;
}
static int apex_power_down(void)
{
	struct pci_dev *apex_dev = NULL;
	struct pci_dev *apex_connected_bus = NULL;
	struct regulator *supply;
	int ret = 0;

#if defined(CONFIG_IMX8MQ_PHANBELL_POWERSAVE)
	if (apex_power_owned) {
		release_bus_freq(BUS_FREQ_HIGH);
	}
#endif

	apex_dev = pci_get_device(APEX_PCI_VENDOR_ID, APEX_PCI_DEVICE_ID, NULL);
	if (!apex_dev) {
		printk(KERN_ERR "apex_power: can't find Apex on PCI bus?!\n");
		return -EIO;
	}
	apex_connected_bus = apex_dev->bus->self;
	pci_dev_put(apex_dev);

	pci_stop_and_remove_bus_device_locked(apex_connected_bus);

	supply = get_apex_regulator();
	ret = regulator_disable(supply);
	if (ret) {
		printk(KERN_ERR "apex_power: Unable to disable regulator.\n");
	}

	return 0;
}

static int apex_power_up(void)
{
	struct pci_bus *pci_bus = NULL;
	struct pci_dev *apex_dev = NULL;
	int ret = 0;
	struct regulator *supply;

	supply = get_apex_regulator();
	ret = regulator_enable(supply);
	if (ret) {
		printk(KERN_ERR "apex_power: Unable to enable regulator.\n");
	}

	// Allot time for the PMIC to sequence and the Apex device to boot.
	msleep(100);

#if defined(CONFIG_IMX8MQ_PHANBELL_POWERSAVE)
	request_bus_freq(BUS_FREQ_HIGH);
#endif

	pci_lock_rescan_remove();
	while ((pci_bus = pci_find_next_bus(pci_bus)) != NULL) {
		pci_rescan_bus(pci_bus);
	}
	pci_unlock_rescan_remove();

	apex_dev = pci_get_device(APEX_PCI_VENDOR_ID, APEX_PCI_DEVICE_ID, NULL);
	if (!apex_dev) {
		printk(KERN_ERR "apex_power: can't find Apex on PCI bus?!\n");

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

	if (apex_power_owned != true) {
		return -EPERM;
	}

	ret = apex_power_down();
	apex_power_owned = false;
	return ret;
}

static int apex_power_open(struct inode* inode, struct file* filep)
{
	if (apex_power_owned != false) {
		return -EPERM;
	}
	apex_power_owned = true;

	/* Ensure that we don't accidentally run afoul of the initial delayed
	   power down. */
	cancel_delayed_work_sync(&apex_power_delayed_init);

	return apex_power_up();
}

static const struct file_operations apex_power_fops = {
	.owner = THIS_MODULE,
	.open = apex_power_open,
	.release = apex_power_release,
};

static void apex_power_delayed_init_callback(struct work_struct *work)
{
	struct pci_dev *apex_dev = NULL;
	apex_dev = pci_get_device(APEX_PCI_VENDOR_ID, APEX_PCI_DEVICE_ID, NULL);

	if (!apex_dev) {
		printk(KERN_INFO "apex_power: rescheduling late init power down\n");
		schedule_delayed_work(&apex_power_delayed_init,
				      msecs_to_jiffies(1000));
		return;
	}

	pci_dev_put(apex_dev);

	printk(KERN_INFO "apex_power: init routines powering down apex\n");
	apex_power_down();
}

static int __init apex_power_init(void)
{
	dev_t apex_power_dev;
	int apex_power_major;
	int ret = 0;
	struct regulator *supply;
	// Enable the supply in regulator framework to ensure power isn't removed
	// until init is complete.

	supply = get_apex_regulator();
	ret = regulator_enable(supply);
	if (ret) {
		printk(KERN_ERR "apex_power: Unable to enable regulator.\n");
	}

	apex_power_class = class_create(THIS_MODULE, "apex_power");
	if (IS_ERR(apex_power_class)) {
		printk(KERN_ERR "apex_power: could not allocate device class\n");
		return -ENODEV;
	}

	ret = alloc_chrdev_region(&apex_power_dev, 0, 1, "apex_power");
	apex_power_major = MAJOR(apex_power_dev);
	if (ret < 0) {
		printk(KERN_ERR "apex_power: alloc_chrdev_region() failed\n");
		class_destroy(apex_power_class);
		return -ENODEV;
	}

	apex_power_dev = MKDEV(apex_power_major, 0);
	cdev_init(&apex_power_cdev, &apex_power_fops);
	ret = cdev_add(&apex_power_cdev, apex_power_dev, 1);

	if (ret) {
		printk(KERN_ERR "apex_power: Unable to add cdev!\n");
		unregister_chrdev_region(apex_power_dev, 1);
		class_destroy(apex_power_class);
	}

	apex_power_device = device_create(apex_power_class, NULL, apex_power_dev,
					  NULL, "%s", "apex_power");
	if (IS_ERR(apex_power_device)) {
		printk(KERN_ERR "apex_power: device_create failed.\n");
		cdev_del(&apex_power_cdev);
		unregister_chrdev_region(apex_power_dev, 1);
		class_destroy(apex_power_class);
	}

	INIT_DELAYED_WORK(&apex_power_delayed_init,
			  apex_power_delayed_init_callback);
	schedule_delayed_work(&apex_power_delayed_init, msecs_to_jiffies(1000));

	printk(KERN_INFO "apex_power: initialized.\n");

	return 0;
}

static void __exit apex_power_exit(void)
{
	device_del(apex_power_device);
	cdev_del(&apex_power_cdev);
	unregister_chrdev_region(MKDEV(apex_power_major, 0), 1);
	class_destroy(apex_power_class);
}

module_init(apex_power_init);
module_exit(apex_power_exit);
MODULE_DESCRIPTION("Google Apex power management driver");
MODULE_AUTHOR("June Tate-Gans <jtgans@google.com>");
MODULE_ALIAS("platform:apex-power");
MODULE_LICENSE("GPL");
