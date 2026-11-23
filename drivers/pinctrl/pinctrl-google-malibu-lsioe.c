// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#define MALIBU_LSIOE_PAD_STRIDE		0x1000
#define MALIBU_LSIOE_PARAM		0x00
#define MALIBU_LSIOE_DMUX		0x04
#define MALIBU_LSIOE_TXDATA		0x08

#define MALIBU_LSIOE_PARAM_INPUT	BIT(8)
#define MALIBU_LSIOE_PARAM_DRIVE		GENMASK(3, 0)
#define MALIBU_LSIOE_DMUX_FUNCTION	GENMASK(8, 0)
#define MALIBU_LSIOE_TX_PULL		GENMASK(1, 0)
#define MALIBU_LSIOE_TX_PULL_UP		BIT(0)
#define MALIBU_LSIOE_TX_VALUE		BIT(2)
#define MALIBU_LSIOE_TX_OUTPUT		BIT(3)

#define MALIBU_LSIOE_SCL		25
#define MALIBU_LSIOE_SDA		26
#define MALIBU_LSIOE_SCL_NAME		"XLSIOE_I23C8_SCL"
#define MALIBU_LSIOE_SDA_NAME		"XLSIOE_I23C8_SDA"
#define MALIBU_LSIOE_CSR_AUTOSUSPEND_MS	200

enum malibu_lsioe_function {
	MALIBU_LSIOE_GPIO,
	MALIBU_LSIOE_I2C,
};

struct malibu_lsioe {
	struct device *dev;
	void __iomem *base;
	struct mutex lock; /* Serializes pad read-modify-write operations. */
};

static const struct pinctrl_pin_desc malibu_lsioe_pins[] = {
	PINCTRL_PIN(MALIBU_LSIOE_SCL, MALIBU_LSIOE_SCL_NAME),
	PINCTRL_PIN(MALIBU_LSIOE_SDA, MALIBU_LSIOE_SDA_NAME),
};

static const unsigned int malibu_lsioe_pad_ids[] = {
	MALIBU_LSIOE_SCL,
	MALIBU_LSIOE_SDA,
};

static const char * const malibu_lsioe_groups[] = {
	MALIBU_LSIOE_SCL_NAME,
	MALIBU_LSIOE_SDA_NAME,
};

static const char * const malibu_lsioe_functions[] = {
	[MALIBU_LSIOE_GPIO] = "lsioe_gpio",
	[MALIBU_LSIOE_I2C] = "lsioe_i2c",
};

static int malibu_lsioe_get_groups_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(malibu_lsioe_groups);
}

static const char *malibu_lsioe_get_group_name(struct pinctrl_dev *pctldev,
					       unsigned int group)
{
	return malibu_lsioe_groups[group];
}

static int malibu_lsioe_get_group_pins(struct pinctrl_dev *pctldev, unsigned int group,
				       const unsigned int **pins, unsigned int *npins)
{
	*pins = &malibu_lsioe_pad_ids[group];
	*npins = 1;
	return 0;
}

static const struct pinctrl_ops malibu_lsioe_pinctrl_ops = {
	.get_groups_count = malibu_lsioe_get_groups_count,
	.get_group_name = malibu_lsioe_get_group_name,
	.get_group_pins = malibu_lsioe_get_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinconf_generic_dt_free_map,
};

static int malibu_lsioe_get_functions_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(malibu_lsioe_functions);
}

static const char *malibu_lsioe_get_function_name(struct pinctrl_dev *pctldev,
						  unsigned int function)
{
	return malibu_lsioe_functions[function];
}

static int malibu_lsioe_get_function_groups(struct pinctrl_dev *pctldev, unsigned int function,
					    const char * const **groups,
					    unsigned int *num_groups)
{
	*groups = malibu_lsioe_groups;
	*num_groups = ARRAY_SIZE(malibu_lsioe_groups);
	return 0;
}

static int malibu_lsioe_set_mux(struct pinctrl_dev *pctldev, unsigned int function,
				unsigned int group)
{
	struct malibu_lsioe *lsioe = pinctrl_dev_get_drvdata(pctldev);
	void __iomem *pad;
	u32 dmux, param, tx, select;
	int ret;

	if (function >= ARRAY_SIZE(malibu_lsioe_functions) ||
	    group >= ARRAY_SIZE(malibu_lsioe_groups))
		return -EINVAL;
	pad = lsioe->base + malibu_lsioe_pad_ids[group] * MALIBU_LSIOE_PAD_STRIDE;
	select = BIT(function);

	ret = pm_runtime_resume_and_get(lsioe->dev);
	if (ret < 0)
		return ret;

	mutex_lock(&lsioe->lock);
	dmux = readl(pad + MALIBU_LSIOE_DMUX);
	if ((dmux & MALIBU_LSIOE_DMUX_FUNCTION) != select) {
		/* Prevent a transient driven GPIO when selecting the sleep mux. */
		tx = readl(pad + MALIBU_LSIOE_TXDATA);
		writel(tx & ~MALIBU_LSIOE_TX_OUTPUT, pad + MALIBU_LSIOE_TXDATA);
		writel((dmux & ~MALIBU_LSIOE_DMUX_FUNCTION) | select,
		       pad + MALIBU_LSIOE_DMUX);
	}
	if (function == MALIBU_LSIOE_I2C) {
		/* D1's I2C3 pad drive code has no verified conversion to mA. */
		param = readl(pad + MALIBU_LSIOE_PARAM);
		param &= ~MALIBU_LSIOE_PARAM_DRIVE;
		param |= FIELD_PREP(MALIBU_LSIOE_PARAM_DRIVE, 3);
		writel(param, pad + MALIBU_LSIOE_PARAM);
	}
	mutex_unlock(&lsioe->lock);
	pm_runtime_put_autosuspend(lsioe->dev);
	return 0;
}

static const struct pinmux_ops malibu_lsioe_pinmux_ops = {
	.get_functions_count = malibu_lsioe_get_functions_count,
	.get_function_name = malibu_lsioe_get_function_name,
	.get_function_groups = malibu_lsioe_get_function_groups,
	.set_mux = malibu_lsioe_set_mux,
	.strict = true,
};

static int malibu_lsioe_pinconf_set(struct pinctrl_dev *pctldev, unsigned int pin,
				    unsigned long *configs, unsigned int num_configs)
{
	struct malibu_lsioe *lsioe = pinctrl_dev_get_drvdata(pctldev);
	u32 param_set = 0, tx_mask = 0, tx_set = 0;
	void __iomem *pad;
	unsigned int i;
	u32 tx;
	int ret;

	if (pin != MALIBU_LSIOE_SCL && pin != MALIBU_LSIOE_SDA)
		return -EINVAL;
	for (i = 0; i < num_configs; i++) {
		u32 arg = pinconf_to_config_argument(configs[i]);

		switch (pinconf_to_config_param(configs[i])) {
		case PIN_CONFIG_BIAS_DISABLE:
			tx_mask |= MALIBU_LSIOE_TX_PULL;
			tx_set &= ~MALIBU_LSIOE_TX_PULL;
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			if (arg != 1)
				return -EINVAL;
			tx_mask |= MALIBU_LSIOE_TX_PULL;
			tx_set = (tx_set & ~MALIBU_LSIOE_TX_PULL) | MALIBU_LSIOE_TX_PULL_UP;
			break;
		case PIN_CONFIG_INPUT_ENABLE:
			if (arg != 1)
				return -EINVAL;
			param_set |= MALIBU_LSIOE_PARAM_INPUT;
			break;
		case PIN_CONFIG_OUTPUT_ENABLE:
			if (arg > 1)
				return -EINVAL;
			tx_mask |= MALIBU_LSIOE_TX_OUTPUT;
			tx_set = (tx_set & ~MALIBU_LSIOE_TX_OUTPUT) |
				 (arg ? MALIBU_LSIOE_TX_OUTPUT : 0);
			break;
		case PIN_CONFIG_LEVEL:
			if (arg != 1)
				return -EINVAL;
			tx_mask |= MALIBU_LSIOE_TX_OUTPUT | MALIBU_LSIOE_TX_VALUE;
			tx_set |= MALIBU_LSIOE_TX_OUTPUT | MALIBU_LSIOE_TX_VALUE;
			break;
		default:
			return -EOPNOTSUPP;
		}
	}

	pad = lsioe->base + pin * MALIBU_LSIOE_PAD_STRIDE;
	ret = pm_runtime_resume_and_get(lsioe->dev);
	if (ret < 0)
		return ret;

	mutex_lock(&lsioe->lock);
	if (param_set)
		writel(readl(pad + MALIBU_LSIOE_PARAM) | param_set,
		       pad + MALIBU_LSIOE_PARAM);
	if (tx_mask) {
		tx = (readl(pad + MALIBU_LSIOE_TXDATA) & ~tx_mask) | tx_set;
		if (tx_set & MALIBU_LSIOE_TX_PULL_UP) {
			writel(tx & ~MALIBU_LSIOE_TX_PULL, pad + MALIBU_LSIOE_TXDATA);
			fsleep(10);
		}
		writel(tx, pad + MALIBU_LSIOE_TXDATA);
	}
	mutex_unlock(&lsioe->lock);
	pm_runtime_put_autosuspend(lsioe->dev);
	return 0;
}

static const struct pinconf_ops malibu_lsioe_pinconf_ops = {
	.is_generic = true,
	.pin_config_set = malibu_lsioe_pinconf_set,
};

static const struct pinctrl_desc malibu_lsioe_desc = {
	.name = "malibu-lsioe-i2c3",
	.owner = THIS_MODULE,
	.pins = malibu_lsioe_pins,
	.npins = ARRAY_SIZE(malibu_lsioe_pins),
	.pctlops = &malibu_lsioe_pinctrl_ops,
	.pmxops = &malibu_lsioe_pinmux_ops,
	.confops = &malibu_lsioe_pinconf_ops,
};

static int malibu_lsioe_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct malibu_lsioe *lsioe;
	struct pinctrl_dev *pctldev;
	struct resource *res;
	int ret;

	if (!dev->pm_domain)
		return dev_err_probe(dev, -ENODEV, "missing LSIOE CSR power domain\n");
	lsioe = devm_kzalloc(dev, sizeof(*lsioe), GFP_KERNEL);
	if (!lsioe)
		return -ENOMEM;
	lsioe->dev = dev;
	mutex_init(&lsioe->lock);
	lsioe->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(lsioe->base))
		return PTR_ERR(lsioe->base);
	if (resource_size(res) < MALIBU_LSIOE_SDA * MALIBU_LSIOE_PAD_STRIDE +
				 MALIBU_LSIOE_TXDATA + sizeof(u32))
		return dev_err_probe(dev, -EINVAL, "LSIOE pad range is too small\n");

	ret = devm_pm_runtime_set_active_enabled(dev);
	if (ret)
		return ret;
	pm_runtime_set_autosuspend_delay(dev, MALIBU_LSIOE_CSR_AUTOSUSPEND_MS);
	pm_runtime_use_autosuspend(dev);

	pctldev = devm_pinctrl_register(dev, &malibu_lsioe_desc, lsioe);
	if (IS_ERR(pctldev))
		return PTR_ERR(pctldev);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_idle(dev);
	return 0;
}

static const struct of_device_id malibu_lsioe_of_match[] = {
	{ .compatible = "google,malibu-lsioe-i2c3-pinctrl" },
	{ }
};
MODULE_DEVICE_TABLE(of, malibu_lsioe_of_match);

static struct platform_driver malibu_lsioe_driver = {
	.probe = malibu_lsioe_probe,
	.driver = {
		.name = "malibu-lsioe-i2c3",
		.of_match_table = malibu_lsioe_of_match,
	},
};
module_platform_driver(malibu_lsioe_driver);

MODULE_DESCRIPTION("Google Malibu LSIOE I2C3 pad control");
MODULE_LICENSE("GPL");
