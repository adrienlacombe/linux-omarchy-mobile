// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitmap.h>
#include <linux/delay.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#define MALIBU_HSIOS_PAD_STRIDE		0x1000
#define MALIBU_HSIOS_PARAM		0x00
#define MALIBU_HSIOS_DMUX		0x04
#define MALIBU_HSIOS_TXDATA		0x08
#define MALIBU_HSIOS_RXDATA		0x1c

#define MALIBU_HSIOS_PARAM_DRIVE		GENMASK(3, 0)
#define MALIBU_HSIOS_DMUX_SELECT		GENMASK(8, 0)
#define MALIBU_HSIOS_PULL		GENMASK(1, 0)
#define MALIBU_HSIOS_PULL_DOWN		BIT(1)
#define MALIBU_HSIOS_OUTPUT_LATCH	BIT(2)
#define MALIBU_HSIOS_OUTPUT_ENABLE	BIT(3)
#define MALIBU_HSIOS_INPUT		BIT(0)

#define MALIBU_HSIOS_RESETB		0
#define MALIBU_HSIOS_POWER		8
#define MALIBU_HSIOS_LSS			11
#define MALIBU_HSIOS_REFCLK		0
#define MALIBU_HSIOS_NUM_GPIOS		13
#define MALIBU_HSIOS_DRIVE_CODE		(PIN_CONFIG_END + 1)

struct malibu_hsios_variant {
	const struct pinctrl_pin_desc *pins;
	unsigned int npins;
	const char * const *groups;
	const char *function;
	u32 mux;
	bool has_gpio;
};

struct malibu_hsios {
	const struct malibu_hsios_variant *variant;
	struct device *dev;
	struct mutex lock; /* Serializes access to the pad control registers. */
	void __iomem *base;
	struct pinctrl_desc desc;
	struct gpio_chip gc;
};

static const struct pinctrl_pin_desc malibu_hsios_main_pins[] = {
	PINCTRL_PIN(MALIBU_HSIOS_RESETB, "XHSIOS_UFS_RESETB"),
	PINCTRL_PIN(MALIBU_HSIOS_POWER, "XHSIOS_UFS_PWR_EN"),
	PINCTRL_PIN(MALIBU_HSIOS_LSS, "XHSIOS_UFS_LSS"),
};

static const char * const malibu_hsios_main_groups[] = {
	"XHSIOS_UFS_RESETB", "XHSIOS_UFS_PWR_EN", "XHSIOS_UFS_LSS",
};

static const struct malibu_hsios_variant malibu_hsios_main = {
	.pins = malibu_hsios_main_pins,
	.npins = ARRAY_SIZE(malibu_hsios_main_pins),
	.groups = malibu_hsios_main_groups,
	.function = "hsios_gpio",
	.mux = BIT(0),
	.has_gpio = true,
};

static const struct pinctrl_pin_desc malibu_hsios_stby_pins[] = {
	PINCTRL_PIN(MALIBU_HSIOS_REFCLK, "XHSIOS_UFS_REFCLK"),
};

static const char * const malibu_hsios_stby_groups[] = {
	"XHSIOS_UFS_REFCLK",
};

static const struct malibu_hsios_variant malibu_hsios_stby = {
	.pins = malibu_hsios_stby_pins,
	.npins = ARRAY_SIZE(malibu_hsios_stby_pins),
	.groups = malibu_hsios_stby_groups,
	.function = "hsios_stby_ufs",
	.mux = BIT(1),
};

static int malibu_hsios_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct malibu_hsios *hsios = pinctrl_dev_get_drvdata(pctldev);

	return hsios->variant->npins;
}

static const char *malibu_hsios_get_group_name(struct pinctrl_dev *pctldev,
					       unsigned int group)
{
	struct malibu_hsios *hsios = pinctrl_dev_get_drvdata(pctldev);

	return hsios->variant->groups[group];
}

static int malibu_hsios_get_group_pins(struct pinctrl_dev *pctldev, unsigned int group,
				      const unsigned int **pins, unsigned int *npins)
{
	struct malibu_hsios *hsios = pinctrl_dev_get_drvdata(pctldev);

	*pins = &hsios->variant->pins[group].number;
	*npins = 1;
	return 0;
}

static const struct pinctrl_ops malibu_hsios_pinctrl_ops = {
	.get_groups_count = malibu_hsios_get_groups_count,
	.get_group_name = malibu_hsios_get_group_name,
	.get_group_pins = malibu_hsios_get_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinconf_generic_dt_free_map,
};

static bool malibu_hsios_has_pin(struct malibu_hsios *hsios, unsigned int pin)
{
	unsigned int i;

	for (i = 0; i < hsios->variant->npins; i++)
		if (hsios->variant->pins[i].number == pin)
			return true;

	return false;
}

static void malibu_hsios_update_bits(void __iomem *reg, u32 mask, u32 value)
{
	u32 old = readl(reg);
	u32 new = (old & ~mask) | value;

	if (old != new)
		writel(new, reg);
}

static int malibu_hsios_access(struct malibu_hsios *hsios, unsigned int pin,
			       void __iomem **pad)
{
	int ret;

	if (!malibu_hsios_has_pin(hsios, pin))
		return -EINVAL;

	ret = pm_runtime_resume_and_get(hsios->dev);
	if (ret < 0)
		return ret;

	mutex_lock(&hsios->lock);
	*pad = hsios->base + pin * MALIBU_HSIOS_PAD_STRIDE;
	return 0;
}

static void malibu_hsios_access_done(struct malibu_hsios *hsios)
{
	mutex_unlock(&hsios->lock);
	pm_runtime_put_autosuspend(hsios->dev);
}

static int malibu_hsios_select_mux(struct malibu_hsios *hsios, unsigned int pin)
{
	void __iomem *pad;
	int ret;

	ret = malibu_hsios_access(hsios, pin, &pad);
	if (ret)
		return ret;

	malibu_hsios_update_bits(pad + MALIBU_HSIOS_DMUX,
				 MALIBU_HSIOS_DMUX_SELECT, hsios->variant->mux);
	malibu_hsios_access_done(hsios);
	return 0;
}

static int malibu_hsios_get_functions_count(struct pinctrl_dev *pctldev)
{
	return 1;
}

static const char *malibu_hsios_get_function_name(struct pinctrl_dev *pctldev,
						  unsigned int function)
{
	struct malibu_hsios *hsios = pinctrl_dev_get_drvdata(pctldev);

	return hsios->variant->function;
}

static int malibu_hsios_get_function_groups(struct pinctrl_dev *pctldev,
					    unsigned int function,
					    const char * const **groups,
					    unsigned int *num_groups)
{
	struct malibu_hsios *hsios = pinctrl_dev_get_drvdata(pctldev);

	*groups = hsios->variant->groups;
	*num_groups = hsios->variant->npins;
	return 0;
}

static bool malibu_hsios_function_is_gpio(struct pinctrl_dev *pctldev,
					 unsigned int function)
{
	struct malibu_hsios *hsios = pinctrl_dev_get_drvdata(pctldev);

	return hsios->variant->has_gpio && function == 0;
}

static int malibu_hsios_set_mux(struct pinctrl_dev *pctldev, unsigned int function,
				unsigned int group)
{
	struct malibu_hsios *hsios = pinctrl_dev_get_drvdata(pctldev);

	if (function || group >= hsios->variant->npins)
		return -EINVAL;

	return malibu_hsios_select_mux(hsios, hsios->variant->pins[group].number);
}

static int malibu_hsios_gpio_request_enable(struct pinctrl_dev *pctldev,
					    struct pinctrl_gpio_range *range,
					    unsigned int pin)
{
	struct malibu_hsios *hsios = pinctrl_dev_get_drvdata(pctldev);

	return malibu_hsios_select_mux(hsios, pin);
}

static const struct pinmux_ops malibu_hsios_pinmux_ops = {
	.get_functions_count = malibu_hsios_get_functions_count,
	.get_function_name = malibu_hsios_get_function_name,
	.get_function_groups = malibu_hsios_get_function_groups,
	.function_is_gpio = malibu_hsios_function_is_gpio,
	.set_mux = malibu_hsios_set_mux,
	.gpio_request_enable = malibu_hsios_gpio_request_enable,
	.strict = true,
};

static const struct pinconf_generic_params malibu_hsios_pinconf_params[] = {
	{ "google,drive-strength-code", MALIBU_HSIOS_DRIVE_CODE, 0 },
};

static const struct pin_config_item malibu_hsios_pinconf_items[] = {
	PCONFDUMP(MALIBU_HSIOS_DRIVE_CODE, "drive strength code", "%u", true),
};

static int malibu_hsios_pinconf_set(struct pinctrl_dev *pctldev, unsigned int pin,
				    unsigned long *configs, unsigned int num_configs)
{
	struct malibu_hsios *hsios = pinctrl_dev_get_drvdata(pctldev);
	int pull = -1, drive = -1;
	void __iomem *pad;
	unsigned int i;
	int ret;

	for (i = 0; i < num_configs; i++) {
		u32 arg = pinconf_to_config_argument(configs[i]);

		switch ((unsigned int)pinconf_to_config_param(configs[i])) {
		case PIN_CONFIG_BIAS_DISABLE:
			pull = 0;
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			if (hsios->variant->has_gpio || arg != 1)
				return -EINVAL;
			pull = MALIBU_HSIOS_PULL_DOWN;
			break;
		case MALIBU_HSIOS_DRIVE_CODE:
			if (hsios->variant->has_gpio || arg > 1)
				return -EINVAL;
			drive = arg;
			break;
		default:
			return -ENOTSUPP;
		}
	}

	ret = malibu_hsios_access(hsios, pin, &pad);
	if (ret)
		return ret;

	if (pull == MALIBU_HSIOS_PULL_DOWN &&
	    (readl(pad + MALIBU_HSIOS_TXDATA) & MALIBU_HSIOS_PULL) != pull) {
		malibu_hsios_update_bits(pad + MALIBU_HSIOS_TXDATA, MALIBU_HSIOS_PULL, 0);
		udelay(10);
	}
	if (drive >= 0)
		malibu_hsios_update_bits(pad + MALIBU_HSIOS_PARAM, MALIBU_HSIOS_PARAM_DRIVE, drive);
	if (pull >= 0)
		malibu_hsios_update_bits(pad + MALIBU_HSIOS_TXDATA, MALIBU_HSIOS_PULL, pull);

	malibu_hsios_access_done(hsios);
	return 0;
}

static const struct pinconf_ops malibu_hsios_pinconf_ops = {
	.is_generic = true,
	.pin_config_set = malibu_hsios_pinconf_set,
};

static int malibu_hsios_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct malibu_hsios *hsios = gpiochip_get_data(gc);
	void __iomem *pad;
	bool output;
	int ret;

	ret = malibu_hsios_access(hsios, offset, &pad);
	if (ret)
		return ret;

	output = readl(pad + MALIBU_HSIOS_TXDATA) & MALIBU_HSIOS_OUTPUT_ENABLE;
	malibu_hsios_access_done(hsios);

	return output ? GPIO_LINE_DIRECTION_OUT : GPIO_LINE_DIRECTION_IN;
}

static int malibu_hsios_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct malibu_hsios *hsios = gpiochip_get_data(gc);
	void __iomem *pad;
	int ret;

	ret = malibu_hsios_access(hsios, offset, &pad);
	if (ret)
		return ret;

	ret = !!(readl(pad + MALIBU_HSIOS_RXDATA) & MALIBU_HSIOS_INPUT);
	malibu_hsios_access_done(hsios);
	return ret;
}

static int malibu_hsios_gpio_write(struct gpio_chip *gc, unsigned int offset,
				   u32 mask, u32 value)
{
	struct malibu_hsios *hsios = gpiochip_get_data(gc);
	void __iomem *pad;
	int ret;

	ret = malibu_hsios_access(hsios, offset, &pad);
	if (ret)
		return ret;

	malibu_hsios_update_bits(pad + MALIBU_HSIOS_TXDATA, mask, value);
	malibu_hsios_access_done(hsios);
	return 0;
}

static int malibu_hsios_gpio_direction_output(struct gpio_chip *gc,
					      unsigned int offset, int value)
{
	u32 output = MALIBU_HSIOS_OUTPUT_ENABLE;

	if (value)
		output |= MALIBU_HSIOS_OUTPUT_LATCH;

	return malibu_hsios_gpio_write(gc, offset,
				      MALIBU_HSIOS_OUTPUT_ENABLE | MALIBU_HSIOS_OUTPUT_LATCH,
				      output);
}

static int malibu_hsios_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	return malibu_hsios_gpio_write(gc, offset, MALIBU_HSIOS_OUTPUT_LATCH,
				      value ? MALIBU_HSIOS_OUTPUT_LATCH : 0);
}

static int malibu_hsios_gpio_init_valid_mask(struct gpio_chip *gc,
					     unsigned long *valid_mask, unsigned int ngpios)
{
	bitmap_zero(valid_mask, ngpios);
	__set_bit(MALIBU_HSIOS_RESETB, valid_mask);
	__set_bit(MALIBU_HSIOS_POWER, valid_mask);
	__set_bit(MALIBU_HSIOS_LSS, valid_mask);
	return 0;
}

static int malibu_hsios_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct malibu_hsios *hsios;
	struct pinctrl_dev *pctldev;
	struct resource *res;
	unsigned int last_pin;
	int ret;

	hsios = devm_kzalloc(dev, sizeof(*hsios), GFP_KERNEL);
	if (!hsios)
		return -ENOMEM;
	hsios->variant = of_device_get_match_data(dev);
	if (!hsios->variant)
		return -ENODEV;
	if (hsios->variant->has_gpio && !dev->pm_domain)
		return dev_err_probe(dev, -ENODEV, "missing HSIO-S CSR power domain\n");
	if (hsios->variant->has_gpio && !of_property_present(dev->of_node, "gpio-ranges"))
		return dev_err_probe(dev, -EINVAL, "missing HSIO-S GPIO pin range\n");

	hsios->dev = dev;
	mutex_init(&hsios->lock);
	hsios->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(hsios->base))
		return PTR_ERR(hsios->base);

	last_pin = hsios->variant->pins[hsios->variant->npins - 1].number;
	if (resource_size(res) < last_pin * MALIBU_HSIOS_PAD_STRIDE +
				 MALIBU_HSIOS_RXDATA + sizeof(u32))
		return dev_err_probe(dev, -EINVAL, "HSIO-S pad range is too small\n");

	ret = devm_pm_runtime_set_active_enabled(dev);
	if (ret)
		return ret;
	pm_runtime_set_autosuspend_delay(dev, 200);
	pm_runtime_use_autosuspend(dev);

	hsios->desc.name = dev_name(dev);
	hsios->desc.pins = hsios->variant->pins;
	hsios->desc.npins = hsios->variant->npins;
	hsios->desc.pctlops = &malibu_hsios_pinctrl_ops;
	hsios->desc.pmxops = &malibu_hsios_pinmux_ops;
	hsios->desc.confops = &malibu_hsios_pinconf_ops;
	hsios->desc.owner = THIS_MODULE;
	hsios->desc.num_custom_params = ARRAY_SIZE(malibu_hsios_pinconf_params);
	hsios->desc.custom_params = malibu_hsios_pinconf_params;
	hsios->desc.custom_conf_items = malibu_hsios_pinconf_items;
	hsios->desc.link_consumers = true;

	ret = devm_pinctrl_register_and_init(dev, &hsios->desc, hsios, &pctldev);
	if (ret)
		return ret;

	ret = pinctrl_enable(pctldev);
	if (ret)
		return ret;

	if (hsios->variant->has_gpio) {
		hsios->gc.label = dev_name(dev);
		hsios->gc.parent = dev;
		hsios->gc.owner = THIS_MODULE;
		hsios->gc.base = -1;
		hsios->gc.ngpio = MALIBU_HSIOS_NUM_GPIOS;
		hsios->gc.can_sleep = true;
		hsios->gc.request = gpiochip_generic_request;
		hsios->gc.free = gpiochip_generic_free;
		hsios->gc.get_direction = malibu_hsios_gpio_get_direction;
		hsios->gc.get = malibu_hsios_gpio_get;
		hsios->gc.direction_output = malibu_hsios_gpio_direction_output;
		hsios->gc.set = malibu_hsios_gpio_set;
		hsios->gc.init_valid_mask = malibu_hsios_gpio_init_valid_mask;

		ret = devm_gpiochip_add_data(dev, &hsios->gc, hsios);
		if (ret)
			return ret;
	}

	pm_runtime_mark_last_busy(dev);
	pm_runtime_idle(dev);
	return 0;
}

static const struct of_device_id malibu_hsios_of_match[] = {
	{ .compatible = "google,malibu-hsios-ufs-pinctrl", .data = &malibu_hsios_main },
	{ .compatible = "google,malibu-hsios-ufs-stby-pinctrl", .data = &malibu_hsios_stby },
	{ }
};
MODULE_DEVICE_TABLE(of, malibu_hsios_of_match);

static struct platform_driver malibu_hsios_driver = {
	.probe = malibu_hsios_probe,
	.driver = {
		.name = "malibu-hsios-ufs-pinctrl",
		.of_match_table = malibu_hsios_of_match,
	},
};
module_platform_driver(malibu_hsios_driver);

MODULE_DESCRIPTION("Google Malibu HSIO-S UFS pin control");
MODULE_LICENSE("GPL");
