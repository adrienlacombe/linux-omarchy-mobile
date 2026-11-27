// SPDX-License-Identifier: GPL-2.0-only

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

static int malibu_cdphy_not_ready(struct phy *phy)
{
	return -EOPNOTSUPP;
}

static const struct phy_ops malibu_cdphy_ops = {
	.init = malibu_cdphy_not_ready,
	.power_on = malibu_cdphy_not_ready,
	.owner = THIS_MODULE,
};

static int malibu_cdphy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy *phy;

	if (!platform_get_resource_byname(pdev, IORESOURCE_MEM, "cdphy-tx") ||
	    !platform_get_resource_byname(pdev, IORESOURCE_MEM, "apb"))
		return dev_err_probe(dev, -EINVAL, "missing PHY register resource\n");

	/* Register the provider without mapping registers or enabling the clock. */
	phy = devm_phy_create(dev, dev->of_node, &malibu_cdphy_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);

	return PTR_ERR_OR_ZERO(devm_of_phy_provider_register(dev, of_phy_simple_xlate));
}

static const struct of_device_id malibu_cdphy_of_match[] = {
	{ .compatible = "google,malibu-dsi-cdphy" },
	{ }
};
MODULE_DEVICE_TABLE(of, malibu_cdphy_of_match);

static struct platform_driver malibu_cdphy_driver = {
	.probe = malibu_cdphy_probe,
	.driver = {
		.name = "malibu-cdphy",
		.of_match_table = malibu_cdphy_of_match,
	},
};
module_platform_driver(malibu_cdphy_driver);

MODULE_DESCRIPTION("Google Malibu display C/D-PHY scaffold");
MODULE_LICENSE("GPL");
