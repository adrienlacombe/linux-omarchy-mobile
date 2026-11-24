// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/usb/role.h>

#define MALIBU_USB_TOP_CFG1		0x4
#define MALIBU_USB_TOP_CFG1_USB2_ONLY	BIT(5)

static const char * const malibu_dwc3_pd_names[] = { "psw", "top" };

static const struct dev_pm_domain_attach_data malibu_dwc3_pd_data = {
	.pd_names = malibu_dwc3_pd_names,
	.num_pd_names = ARRAY_SIZE(malibu_dwc3_pd_names),
	.pd_flags = PD_FLAG_DEV_LINK_ON,
};

struct malibu_dwc3 {
	struct device *dev;
	struct clk_bulk_data clks[3];
	struct reset_control_bulk_data resets[5];
	struct usb_role_switch *phy_switch;
	struct usb_role_switch *repeater_switch;
	struct usb_role_switch *role_switch;
};

/* Serialize callback entry with clearing a removed switch's private data. */
static DEFINE_MUTEX(malibu_dwc3_role_lock);

static int malibu_dwc3_disconnect(struct malibu_dwc3 *malibu)
{
	int ret;

	ret = usb_role_switch_set_role(malibu->phy_switch, USB_ROLE_NONE);
	if (ret)
		return ret;
	return usb_role_switch_set_role(malibu->repeater_switch, USB_ROLE_NONE);
}

static int malibu_dwc3_set_role(struct usb_role_switch *sw, enum usb_role role)
{
	struct malibu_dwc3 *malibu;
	int ret;

	guard(mutex)(&malibu_dwc3_role_lock);
	malibu = usb_role_switch_get_drvdata(sw);
	if (!malibu)
		return -ENODEV;
	if (role == USB_ROLE_NONE)
		return malibu_dwc3_disconnect(malibu);
	if (role != USB_ROLE_DEVICE)
		return -EOPNOTSUPP;

	ret = usb_role_switch_set_role(malibu->repeater_switch, USB_ROLE_DEVICE);
	if (!ret)
		ret = usb_role_switch_set_role(malibu->phy_switch, USB_ROLE_DEVICE);
	if (ret && malibu_dwc3_disconnect(malibu))
		dev_warn(malibu->dev, "failed to fully roll back USB2 role\n");
	return ret;
}

static void malibu_dwc3_unregister_role(void *data)
{
	struct malibu_dwc3 *malibu = data;
	struct usb_role_switch *sw = malibu->role_switch;

	if (!sw)
		return;
	scoped_guard(mutex, &malibu_dwc3_role_lock) {
		usb_role_switch_set_drvdata(sw, NULL);
		if (malibu_dwc3_disconnect(malibu))
			dev_warn(malibu->dev, "failed to fully disconnect USB2 on removal\n");
	}
	usb_role_switch_unregister(sw);
}

static void malibu_dwc3_put_switch(void *data)
{
	usb_role_switch_put(data);
}

static int malibu_dwc3_get_switch(struct device *dev, struct device_node *node,
				  const struct bus_type *bus, const char *compatible,
				  struct usb_role_switch **out)
{
	struct device *supplier __free(put_device) = NULL;
	struct device_link *link;
	struct usb_role_switch *sw;
	int ret;

	if (!node || !of_device_is_compatible(node, compatible))
		return -EINVAL;
	supplier = bus_find_device_by_fwnode(bus, of_fwnode_handle(node));
	if (!supplier)
		return -EPROBE_DEFER;
	link = device_link_add(dev, supplier, DL_FLAG_AUTOREMOVE_CONSUMER | DL_FLAG_PM_RUNTIME);
	if (!link)
		return -EINVAL;
	if (READ_ONCE(link->status) != DL_STATE_CONSUMER_PROBE)
		return -EPROBE_DEFER;
	sw = usb_role_switch_find_by_fwnode(of_fwnode_handle(node));
	if (!sw)
		return -EPROBE_DEFER;
	ret = devm_add_action_or_reset(dev, malibu_dwc3_put_switch, sw);
	if (!ret)
		*out = sw;
	return ret;
}

static int malibu_dwc3_get_role_endpoints(struct malibu_dwc3 *malibu)
{
	struct device *dev = malibu->dev;
	struct device_node *child __free(device_node) =
		of_get_next_available_child(dev->of_node, NULL);
	struct device_node *phy __free(device_node) = of_parse_phandle(child, "phys", 0);
	struct device_node *repeater __free(device_node) =
		of_parse_phandle(dev->of_node, "google,usb2-repeater", 0);
	int ret;

	ret = malibu_dwc3_get_switch(dev, phy, &platform_bus_type,
				     "google,malibu-usb2-phy", &malibu->phy_switch);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get USB2 PHY switch\n");
	ret = malibu_dwc3_get_switch(dev, repeater, &i2c_bus_type,
				     "ti,tusb2e11", &malibu->repeater_switch);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get USB2 repeater switch\n");
	return 0;
}

static int malibu_dwc3_register_role(struct malibu_dwc3 *malibu)
{
	struct device *dev = malibu->dev;
	struct usb_role_switch_desc desc = {
		.fwnode = dev_fwnode(dev),
		.set = malibu_dwc3_set_role,
		.driver_data = malibu,
	};
	struct usb_role_switch *sw;
	int ret;

	ret = malibu_dwc3_disconnect(malibu);
	if (ret)
		return ret;
	ret = devm_add_action(dev, malibu_dwc3_unregister_role, malibu);
	if (ret)
		return ret;
	sw = usb_role_switch_register(dev, &desc);
	if (IS_ERR(sw))
		return PTR_ERR(sw);
	malibu->role_switch = sw;
	return 0;
}

static int malibu_dwc3_check_child(struct device_node *node)
{
	const char *mode, *speed;
	bool found = false;

	for_each_available_child_of_node_scoped(node, child) {
		if (found || !of_device_is_compatible(child, "snps,dwc3"))
			return -EINVAL;

		if (of_property_read_string(child, "dr_mode", &mode) ||
		    strcmp(mode, "peripheral") ||
		    of_property_read_string(child, "maximum-speed", &speed) ||
		    strcmp(speed, "high-speed"))
			return -EINVAL;

		if (of_property_count_strings(child, "phy-names") != 1 ||
		    of_property_match_string(child, "phy-names", "usb2-phy") ||
		    !of_property_present(child, "phys") ||
		    !of_property_present(child, "iommus") ||
		    of_property_present(child, "usb-role-switch"))
			return -EINVAL;

		found = true;
	}

	return found ? 0 : -ENODEV;
}

static void malibu_dwc3_put_runtime(void *data)
{
	pm_runtime_put_sync_suspend(data);
}

static void malibu_dwc3_disable_clks(void *data)
{
	struct malibu_dwc3 *malibu = data;

	clk_bulk_disable_unprepare(ARRAY_SIZE(malibu->clks), malibu->clks);
}

static void malibu_dwc3_assert_resets(void *data)
{
	struct malibu_dwc3 *malibu = data;
	int ret;

	ret = reset_control_bulk_assert(ARRAY_SIZE(malibu->resets), malibu->resets);
	if (ret)
		dev_warn(malibu->dev, "failed to assert wrapper resets: %pe\n",
			 ERR_PTR(ret));
}

static int malibu_dwc3_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dev_pm_domain_list *domains;
	struct malibu_dwc3 *malibu;
	struct resource *res;
	void __iomem *top;
	u32 value;
	bool role_routing;
	int ret;

	ret = malibu_dwc3_check_child(dev->of_node);
	if (ret)
		return dev_err_probe(dev, ret, "expected one USB2 peripheral controller\n");

	malibu = devm_kzalloc(dev, sizeof(*malibu), GFP_KERNEL);
	if (!malibu)
		return -ENOMEM;
	malibu->dev = dev;

	role_routing = of_property_read_bool(dev->of_node, "usb-role-switch");
	if (role_routing) {
		ret = malibu_dwc3_get_role_endpoints(malibu);
		if (ret)
			return ret;
	}

	top = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(top))
		return PTR_ERR(top);
	if (resource_size(res) < MALIBU_USB_TOP_CFG1 + sizeof(value))
		return dev_err_probe(dev, -EINVAL, "USB top register range is too small\n");

	malibu->clks[0].id = "non-sticky";
	malibu->clks[1].id = "sticky";
	malibu->clks[2].id = "usb2-apb";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(malibu->clks), malibu->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get wrapper clocks\n");

	malibu->resets[0].id = "non-sticky";
	malibu->resets[1].id = "sticky";
	malibu->resets[2].id = "drd-bus";
	malibu->resets[3].id = "usb2-apb";
	malibu->resets[4].id = "top";
	ret = devm_reset_control_bulk_get_exclusive(dev, ARRAY_SIZE(malibu->resets),
						    malibu->resets);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get wrapper resets\n");

	ret = devm_pm_domain_attach_list(dev, &malibu_dwc3_pd_data, &domains);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to attach USB power domains\n");
	if (ret != ARRAY_SIZE(malibu_dwc3_pd_names))
		return dev_err_probe(dev, -ENODEV, "USB power domains unavailable\n");

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	/* Retain wrapper power through generic DWC3 child probe and removal. */
	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to power USB wrapper\n");
	ret = devm_add_action_or_reset(dev, malibu_dwc3_put_runtime, dev);
	if (ret)
		return ret;

	value = readl(top + MALIBU_USB_TOP_CFG1);
	writel(value | MALIBU_USB_TOP_CFG1_USB2_ONLY, top + MALIBU_USB_TOP_CFG1);

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(malibu->clks), malibu->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable wrapper clocks\n");
	ret = devm_add_action_or_reset(dev, malibu_dwc3_disable_clks, malibu);
	if (ret)
		return ret;

	ret = reset_control_bulk_deassert(ARRAY_SIZE(malibu->resets), malibu->resets);
	if (ret)
		return dev_err_probe(dev, ret, "failed to release wrapper resets\n");
	ret = devm_add_action_or_reset(dev, malibu_dwc3_assert_resets, malibu);
	if (ret)
		return ret;

	if (!(readl(top + MALIBU_USB_TOP_CFG1) & MALIBU_USB_TOP_CFG1_USB2_ONLY))
		return dev_err_probe(dev, -EIO, "USB2-only mode did not latch\n");

	/* The child PHY accesses its configuration registers via the USB2 APB. */
	ret = devm_of_platform_populate(dev);
	if (ret)
		return ret;
	if (role_routing)
		return malibu_dwc3_register_role(malibu);
	return 0;
}

static const struct of_device_id malibu_dwc3_of_match[] = {
	{ .compatible = "google,malibu-dwc3" },
	{ }
};
MODULE_DEVICE_TABLE(of, malibu_dwc3_of_match);

static struct platform_driver malibu_dwc3_driver = {
	.probe = malibu_dwc3_probe,
	.driver = {
		.name = "dwc3-google-malibu",
		.of_match_table = malibu_dwc3_of_match,
	},
};
module_platform_driver(malibu_dwc3_driver);

MODULE_DESCRIPTION("Google Malibu DWC3 USB2 platform driver");
MODULE_LICENSE("GPL");
