// SPDX-License-Identifier: GPL-2.0
/*
 * phy-google-usb.c - Google USB PHY driver
 *
 * Copyright (C) 2025, Google LLC
 */

#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/usb/role.h>
#include <linux/usb/typec_mux.h>

#define USBCS_USB2PHY_CFG19_OFFSET 0x0
#define USBCS_USB2PHY_CFG19_PHY_CFG_PLL_FB_DIV GENMASK(19, 8)
#define USBCS_USB2PHY_CFG19_REXT_OVERRIDE BIT(29)

#define USBCS_USB2PHY_CFG20_OFFSET 0x4
#define USBCS_USB2PHY_CFG20_REXT_FIELDS GENMASK(22, 15)
#define USBCS_USB2PHY_CFG20_REXT_VALUE GENMASK(18, 15)

#define USBCS_USB2PHY_CFG21_OFFSET 0x8
#define USBCS_USB2PHY_CFG21_PHY_ENABLE BIT(12)
#define USBCS_USB2PHY_CFG21_REF_FREQ_SEL GENMASK(15, 13)
#define USBCS_USB2PHY_CFG21_PHY_TX_DIG_BYPASS_SEL BIT(19)

#define USBCS_PHY_CFG1_OFFSET 0x28
#define USBCS_PHY_CFG1_SYS_VBUSVALID BIT(17)
#define USBCS_PHY_CFG2_OFFSET 0x2c
#define USBCS_PHY_CFG2_USB2_VBUSVALID BIT(16)

enum google_usb_phy_id {
	GOOGLE_USB2_PHY,
	GOOGLE_USB_PHY_NUM,
};

struct google_usb_phy_instance {
	struct google_usb_phy *parent;
	unsigned int index;
	struct phy *phy;
	unsigned int num_clks;
	struct clk_bulk_data *clks;
	unsigned int num_rsts;
	struct reset_control_bulk_data *rsts;
};

struct google_usb_phy {
	struct device *dev;
	struct regmap *usb_cfg_regmap;
	unsigned int usb2_cfg_offset;
	void __iomem *usbdp_top_base;
	struct google_usb_phy_instance *insts;
	/*
	 * Protect phy registers from concurrent access, specifically via
	 * google_usb_set_orientation callback.
	 */
	struct mutex phy_mutex;
	struct typec_switch_dev *sw;
	struct usb_role_switch *role_switch;
	enum typec_orientation orientation;
	enum usb_role role;
	u8 rext;
	bool malibu;
	bool usb2_ready;
	bool apply_rext;
	bool reset_recovery;
};

/* Serialize role callback entry with clearing the provider's private data. */
static DEFINE_MUTEX(google_usb_role_lock);

static void set_vbus_bit(void __iomem *addr, u32 mask, bool valid)
{
	u32 reg = readl(addr);

	if (valid)
		reg |= mask;
	else
		reg &= ~mask;
	writel(reg, addr);
}

static void set_vbus_valid(struct google_usb_phy *gphy, bool valid)
{
	set_vbus_bit(gphy->usbdp_top_base + USBCS_PHY_CFG1_OFFSET,
		     USBCS_PHY_CFG1_SYS_VBUSVALID, valid);
	if (gphy->malibu)
		set_vbus_bit(gphy->usbdp_top_base + USBCS_PHY_CFG2_OFFSET,
			     USBCS_PHY_CFG2_USB2_VBUSVALID, valid);
}

static int google_usb_set_role(struct usb_role_switch *sw, enum usb_role role)
{
	struct google_usb_phy *gphy;

	guard(mutex)(&google_usb_role_lock);
	gphy = usb_role_switch_get_drvdata(sw);
	if (!gphy)
		return -ENODEV;
	guard(mutex)(&gphy->phy_mutex);
	gphy->role = role;
	if (gphy->usb2_ready)
		set_vbus_valid(gphy, role == USB_ROLE_DEVICE);
	return 0;
}

static void google_usb_unregister_role(void *data)
{
	struct google_usb_phy *gphy = data;
	struct usb_role_switch *sw = gphy->role_switch;

	if (!sw)
		return;
	scoped_guard(mutex, &google_usb_role_lock)
		usb_role_switch_set_drvdata(sw, NULL);
	usb_role_switch_unregister(sw);
}

static int google_usb_apply_rext(struct google_usb_phy *gphy)
{
	int ret;

	if (!gphy->apply_rext)
		return 0;
	ret = regmap_update_bits(gphy->usb_cfg_regmap,
				 gphy->usb2_cfg_offset + USBCS_USB2PHY_CFG19_OFFSET,
				 USBCS_USB2PHY_CFG19_REXT_OVERRIDE,
				 USBCS_USB2PHY_CFG19_REXT_OVERRIDE);
	if (ret)
		return ret;
	return regmap_update_bits(gphy->usb_cfg_regmap,
				  gphy->usb2_cfg_offset + USBCS_USB2PHY_CFG20_OFFSET,
				  USBCS_USB2PHY_CFG20_REXT_FIELDS,
				  FIELD_PREP(USBCS_USB2PHY_CFG20_REXT_VALUE, gphy->rext));
}

static int google_usb_set_orientation(struct typec_switch_dev *sw,
				      enum typec_orientation orientation)
{
	struct google_usb_phy *gphy = typec_switch_get_drvdata(sw);

	dev_dbg(gphy->dev, "set orientation %d\n", orientation);

	gphy->orientation = orientation;

	if (pm_runtime_suspended(gphy->dev))
		return 0;

	guard(mutex)(&gphy->phy_mutex);

	set_vbus_valid(gphy, gphy->orientation != TYPEC_ORIENTATION_NONE);

	return 0;
}

static int google_usb2_retry_reset(struct google_usb_phy_instance *inst)
{
	struct google_usb_phy *gphy = inst->parent;
	int ret;

	if (!gphy->reset_recovery)
		return 0;

	ret = reset_control_bulk_assert(inst->num_rsts, inst->rsts);
	if (ret)
		return ret;
	gphy->reset_recovery = false;
	clk_bulk_disable_unprepare(inst->num_clks, inst->clks);
	pm_runtime_put(gphy->dev);
	return 0;
}

static void google_usb2_release_recovery(void *data)
{
	struct google_usb_phy_instance *inst = data;
	struct google_usb_phy *gphy = inst->parent;
	int ret;

	guard(mutex)(&gphy->phy_mutex);
	ret = google_usb2_retry_reset(inst);
	if (ret) {
		dev_warn(gphy->dev, "failed to reset USB2 PHY on removal: %pe\n", ERR_PTR(ret));
		clk_bulk_disable_unprepare(inst->num_clks, inst->clks);
		pm_runtime_put(gphy->dev);
	}
}

static int google_usb2_phy_init(struct phy *_phy)
{
	struct google_usb_phy_instance *inst = phy_get_drvdata(_phy);
	struct google_usb_phy *gphy = inst->parent;
	unsigned int cfg21 = gphy->usb2_cfg_offset + USBCS_USB2PHY_CFG21_OFFSET;
	unsigned int cfg19 = gphy->usb2_cfg_offset + USBCS_USB2PHY_CFG19_OFFSET;
	int ret, reset_ret;

	dev_dbg(gphy->dev, "initializing usb2 phy\n");
	if (gphy->malibu) {
		ret = pm_runtime_resume_and_get(gphy->dev);
		if (ret)
			return ret;
	}

	guard(mutex)(&gphy->phy_mutex);

	if (gphy->malibu) {
		ret = google_usb2_retry_reset(inst);
		if (ret)
			goto put_power;
	}

	ret = regmap_write_bits(gphy->usb_cfg_regmap, cfg21,
				USBCS_USB2PHY_CFG21_PHY_TX_DIG_BYPASS_SEL |
				USBCS_USB2PHY_CFG21_REF_FREQ_SEL, 0);
	if (ret)
		goto put_power;
	ret = regmap_write_bits(gphy->usb_cfg_regmap, cfg19,
				USBCS_USB2PHY_CFG19_PHY_CFG_PLL_FB_DIV,
				FIELD_PREP(USBCS_USB2PHY_CFG19_PHY_CFG_PLL_FB_DIV, 368));
	if (ret)
		goto put_power;

	set_vbus_valid(gphy, gphy->malibu ? gphy->role == USB_ROLE_DEVICE :
					  gphy->orientation != TYPEC_ORIENTATION_NONE);
	ret = google_usb_apply_rext(gphy);
	if (ret)
		goto clear_vbus;

	ret = clk_bulk_prepare_enable(inst->num_clks, inst->clks);
	if (ret)
		goto clear_vbus;

	ret = reset_control_bulk_deassert(inst->num_rsts, inst->rsts);
	if (ret) {
		if (gphy->malibu)
			goto assert_resets;
		goto disable_clocks;
	}

	ret = regmap_write_bits(gphy->usb_cfg_regmap, cfg21,
				USBCS_USB2PHY_CFG21_PHY_ENABLE, USBCS_USB2PHY_CFG21_PHY_ENABLE);
	if (ret)
		goto assert_resets;
	gphy->usb2_ready = true;

	return 0;

assert_resets:
	reset_ret = reset_control_bulk_assert(inst->num_rsts, inst->rsts);
	if (reset_ret && gphy->malibu) {
		/* Keep the clock and PM reference until the reset can be asserted. */
		dev_warn(gphy->dev, "failed to reset USB2 PHY after init error: %pe\n",
			 ERR_PTR(reset_ret));
		set_vbus_valid(gphy, false);
		gphy->reset_recovery = true;
		return ret;
	}
disable_clocks:
	clk_bulk_disable_unprepare(inst->num_clks, inst->clks);
clear_vbus:
	if (gphy->malibu)
		set_vbus_valid(gphy, false);
put_power:
	if (gphy->malibu)
		pm_runtime_put(gphy->dev);
	return ret;
}

static int google_usb2_phy_exit(struct phy *_phy)
{
	struct google_usb_phy_instance *inst = phy_get_drvdata(_phy);
	struct google_usb_phy *gphy = inst->parent;
	unsigned int cfg21 = gphy->usb2_cfg_offset + USBCS_USB2PHY_CFG21_OFFSET;
	int ret;

	dev_dbg(gphy->dev, "exiting usb2 phy\n");

	guard(mutex)(&gphy->phy_mutex);
	ret = regmap_write_bits(gphy->usb_cfg_regmap, cfg21, USBCS_USB2PHY_CFG21_PHY_ENABLE, 0);
	if (ret && !gphy->malibu)
		return ret;
	if (gphy->malibu) {
		gphy->usb2_ready = false;
		set_vbus_valid(gphy, false);
	}
	ret = reset_control_bulk_assert(inst->num_rsts, inst->rsts);
	if (ret) {
		if (gphy->malibu) {
			/*
			 * The role is disconnected. Defer the reset and clock
			 * release so the next init can retry with power held.
			 */
			dev_warn(gphy->dev, "deferring USB2 PHY reset: %pe\n", ERR_PTR(ret));
			gphy->reset_recovery = true;
			return 0;
		}
		regmap_write_bits(gphy->usb_cfg_regmap, cfg21,
				  USBCS_USB2PHY_CFG21_PHY_ENABLE,
				  USBCS_USB2PHY_CFG21_PHY_ENABLE);
		return ret;
	}
	clk_bulk_disable_unprepare(inst->num_clks, inst->clks);
	if (gphy->malibu)
		pm_runtime_put(gphy->dev);

	return 0;
}

static const struct phy_ops google_usb2_phy_ops = {
	.init		= google_usb2_phy_init,
	.exit		= google_usb2_phy_exit,
};

static struct phy *google_usb_phy_xlate(struct device *dev,
					const struct of_phandle_args *args)
{
	struct google_usb_phy *gphy = dev_get_drvdata(dev);

	if (args->args[0] >= GOOGLE_USB_PHY_NUM) {
		dev_err(dev, "invalid PHY index requested from DT\n");
		return ERR_PTR(-ENODEV);
	}
	return gphy->insts[args->args[0]].phy;
}

static int google_usb_read_rext(struct google_usb_phy *gphy)
{
	struct device *dev = gphy->dev;
	u8 mode;
	int ret;

	if (!gphy->malibu || !device_property_present(dev, "nvmem-cells"))
		return 0;
	ret = nvmem_cell_read_u8(dev, "usb2-rext-external-mode", &mode);
	if (ret)
		return ret;
	if (mode > 1)
		return -EINVAL;

	ret = nvmem_cell_read_u8(dev, "usb2-rext-ctrl", &gphy->rext);
	if (ret)
		return ret;
	if (!FIELD_FIT(USBCS_USB2PHY_CFG20_REXT_VALUE, gphy->rext))
		return -EINVAL;
	gphy->apply_rext = !mode;
	return 0;
}

static int google_usb_register_role(struct google_usb_phy *gphy)
{
	struct usb_role_switch_desc desc = {
		.fwnode = dev_fwnode(gphy->dev),
		.set = google_usb_set_role,
		.driver_data = gphy,
	};
	struct usb_role_switch *sw;
	int ret;

	ret = devm_add_action(gphy->dev, google_usb_unregister_role, gphy);
	if (ret)
		return ret;
	sw = usb_role_switch_register(gphy->dev, &desc);
	if (IS_ERR(sw))
		return PTR_ERR(sw);
	gphy->role_switch = sw;
	return 0;
}

static int google_usb_phy_probe(struct platform_device *pdev)
{
	struct typec_switch_desc sw_desc = { };
	struct google_usb_phy_instance *inst;
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	struct google_usb_phy *gphy;
	struct resource *res;
	struct phy *phy;
	u32 args[1];
	int ret;

	gphy = devm_kzalloc(dev, sizeof(*gphy), GFP_KERNEL);
	if (!gphy)
		return -ENOMEM;

	dev_set_drvdata(dev, gphy);
	gphy->dev = dev;
	gphy->malibu = of_device_is_compatible(dev->of_node, "google,malibu-usb2-phy");

	ret = devm_mutex_init(dev, &gphy->phy_mutex);
	if (ret)
		return ret;
	ret = google_usb_read_rext(gphy);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read USB2 PHY calibration\n");

	gphy->usb_cfg_regmap =
		syscon_regmap_lookup_by_phandle_args(dev->of_node,
						     "google,usb-cfg-csr",
						     ARRAY_SIZE(args), args);
	if (IS_ERR(gphy->usb_cfg_regmap)) {
		return dev_err_probe(dev, PTR_ERR(gphy->usb_cfg_regmap),
				     "invalid usb cfg csr\n");
	}

	gphy->usb2_cfg_offset = args[0];

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "usbdp_top");
	if (gphy->malibu && (!res || resource_size(res) < USBCS_PHY_CFG2_OFFSET + sizeof(u32)))
		return dev_err_probe(dev, -EINVAL, "invalid Malibu USB PHY top register range\n");
	gphy->usbdp_top_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(gphy->usbdp_top_base))
		return dev_err_probe(dev, PTR_ERR(gphy->usbdp_top_base),
				    "invalid usbdp top\n");

	gphy->insts = devm_kcalloc(dev, GOOGLE_USB_PHY_NUM, sizeof(*gphy->insts), GFP_KERNEL);
	if (!gphy->insts)
		return -ENOMEM;

	inst = &gphy->insts[GOOGLE_USB2_PHY];
	inst->parent = gphy;
	inst->index = GOOGLE_USB2_PHY;
	if (gphy->malibu) {
		ret = devm_pm_runtime_enable(dev);
		if (ret)
			return ret;
	}
	phy = devm_phy_create(dev, NULL, &google_usb2_phy_ops);
	if (IS_ERR(phy))
		return dev_err_probe(dev, PTR_ERR(phy),
				     "failed to create usb2 phy instance\n");
	inst->phy = phy;
	phy_set_drvdata(phy, inst);

	inst->num_clks = gphy->malibu ? 1 : 2;
	inst->clks = devm_kcalloc(dev, inst->num_clks, sizeof(*inst->clks), GFP_KERNEL);
	if (!inst->clks)
		return -ENOMEM;
	inst->clks[0].id = "usb2";
	if (!gphy->malibu)
		inst->clks[1].id = "usb2_apb";
	ret = devm_clk_bulk_get(dev, inst->num_clks, inst->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get u2 phy clks\n");

	inst->num_rsts = gphy->malibu ? 1 : 2;
	inst->rsts = devm_kcalloc(dev, inst->num_rsts, sizeof(*inst->rsts), GFP_KERNEL);
	if (!inst->rsts)
		return -ENOMEM;
	inst->rsts[0].id = "usb2";
	if (!gphy->malibu)
		inst->rsts[1].id = "usb2_apb";
	ret = devm_reset_control_bulk_get_exclusive(dev, inst->num_rsts, inst->rsts);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get u2 phy resets\n");
	if (gphy->malibu) {
		ret = devm_add_action_or_reset(dev, google_usb2_release_recovery, inst);
		if (ret)
			return ret;
	} else {
		ret = devm_pm_runtime_enable(dev);
		if (ret)
			return ret;
	}

	phy_provider = devm_of_phy_provider_register(dev, google_usb_phy_xlate);
	if (IS_ERR(phy_provider))
		return dev_err_probe(dev, PTR_ERR(phy_provider),
				     "failed to register phy provider\n");

	if (gphy->malibu) {
		ret = google_usb_register_role(gphy);
		if (ret)
			return dev_err_probe(dev, ret, "failed to register USB2 PHY role switch\n");
		return 0;
	}

	sw_desc.fwnode = dev_fwnode(dev);
	sw_desc.drvdata = gphy;
	sw_desc.name = fwnode_get_name(dev_fwnode(dev));
	sw_desc.set = google_usb_set_orientation;

	gphy->sw = typec_switch_register(dev, &sw_desc);
	if (IS_ERR(gphy->sw))
		return dev_err_probe(dev, PTR_ERR(gphy->sw),
				     "failed to register typec switch\n");

	return 0;
}

static void google_usb_phy_remove(struct platform_device *pdev)
{
	struct google_usb_phy *gphy = dev_get_drvdata(&pdev->dev);

	if (!gphy->malibu)
		typec_switch_unregister(gphy->sw);
}

static const struct of_device_id google_usb_phy_of_match[] = {
	{
		.compatible = "google,lga-usb-phy",
	},
	{
		.compatible = "google,malibu-usb2-phy",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, google_usb_phy_of_match);

static struct platform_driver google_usb_phy = {
	.probe	= google_usb_phy_probe,
	.remove = google_usb_phy_remove,
	.driver = {
		.name		= "google-usb-phy",
		.of_match_table	= google_usb_phy_of_match,
	}
};

module_platform_driver(google_usb_phy);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Google USB phy driver");
