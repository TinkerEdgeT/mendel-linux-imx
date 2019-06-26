#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pci.h>

struct yorktown_priv {
	struct platform_device *pdev;
	struct delayed_work rescan_work;
};

static void yorktown_tpu_rescan(struct work_struct *work)
{
	struct pci_dev *switch_device, *root_device;
	struct pci_bus *pci_bus = NULL;
	struct yorktown_priv *priv =
		container_of(container_of(work, struct delayed_work, work),
			struct yorktown_priv, rescan_work);
	wait_for_device_probe();

	switch_device = pci_get_device(0x12D8, 0x2912, NULL);
	if (!switch_device) {
		dev_err(&priv->pdev->dev,
			"Could not find PCI switch! Giving up!");
		return;
	}
	root_device = switch_device->bus->self;
	pci_stop_and_remove_bus_device_locked(root_device);

	pci_lock_rescan_remove();
	while ((pci_bus = pci_find_next_bus(pci_bus)) != NULL) {
		pci_rescan_bus(pci_bus);
	}
	pci_unlock_rescan_remove();
}

static int yorktown_tpu_probe(struct platform_device *pdev)
{
	int ret;
	struct yorktown_priv *priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	priv->pdev = pdev;
	platform_set_drvdata(pdev, priv);

	ret = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "of_platform error: %d\n", ret);
	}

	INIT_DELAYED_WORK(&priv->rescan_work, yorktown_tpu_rescan);
	schedule_delayed_work(&priv->rescan_work, msecs_to_jiffies(1000));

	return 0;
}

static int yorktown_tpu_remove(struct platform_device *pdev)
{
	of_platform_depopulate(&pdev->dev);
	return 0;
}

static const struct of_device_id yorktown_tpu_dt_ids[] = {
	{
		.compatible = "google,yorktown-tpu",
	},
	{}
};
MODULE_DEVICE_TABLE(of, yorktown_tpu_dt_ids);

static struct platform_driver yorktown_tpu_driver = {
    .driver = {
        .name = "yorktown-tpu",
        .of_match_table = yorktown_tpu_dt_ids,
    },
    .probe = yorktown_tpu_probe,
    .remove = yorktown_tpu_remove,
};
module_platform_driver(yorktown_tpu_driver);

MODULE_AUTHOR("Alex Van Damme <atv@google.com>");
MODULE_DESCRIPTION("Yorktown TPU Driver");
MODULE_LICENSE("GPL v2");
