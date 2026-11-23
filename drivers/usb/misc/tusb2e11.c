// SPDX-License-Identifier: GPL-2.0-only

#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/role.h>

static const char * const tusb2e11_supply_names[] = {
	"vdd33", "vdd18", "vdd-reset",
};

/* Serialize callback entry with clearing a removed switch's private data. */
static DEFINE_MUTEX(tusb2e11_role_lock);

struct tusb2e11 {
	struct device *dev;
	struct usb_role_switch *role_switch;
	struct regulator *supplies[ARRAY_SIZE(tusb2e11_supply_names)];
	unsigned int votes;
};

static int tusb2e11_drop_votes(struct tusb2e11 *repeater, unsigned int votes)
{
	int ret = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(repeater->supplies); i++) {
		int err;

		if (!(votes & BIT(i)))
			continue;
		err = regulator_disable(repeater->supplies[i]);
		if (err) {
			if (!ret)
				ret = err;
			continue;
		}
		repeater->votes &= ~BIT(i);
	}
	return ret;
}

static int tusb2e11_power_on(struct tusb2e11 *repeater)
{
	unsigned int previous = repeater->votes;
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(repeater->supplies); i++) {
		if (repeater->votes & BIT(i))
			continue;
		ret = regulator_enable(repeater->supplies[i]);
		if (ret) {
			if (tusb2e11_drop_votes(repeater, repeater->votes & ~previous))
				dev_warn(repeater->dev, "failed to release repeater supplies\n");
			return ret;
		}
		repeater->votes |= BIT(i);
	}

	if (repeater->votes != previous)
		fsleep(5000);
	return 0;
}

static int tusb2e11_set_role(struct usb_role_switch *sw, enum usb_role role)
{
	struct tusb2e11 *repeater;

	guard(mutex)(&tusb2e11_role_lock);
	repeater = usb_role_switch_get_drvdata(sw);
	if (!repeater)
		return -ENODEV;

	switch (role) {
	case USB_ROLE_NONE:
		return tusb2e11_drop_votes(repeater, repeater->votes);
	case USB_ROLE_HOST:
	case USB_ROLE_DEVICE:
		return tusb2e11_power_on(repeater);
	default:
		return -EINVAL;
	}
}

static int tusb2e11_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct usb_role_switch_desc role_desc = { };
	struct tusb2e11 *repeater;
	unsigned int i;

	repeater = devm_kzalloc(dev, sizeof(*repeater), GFP_KERNEL);
	if (!repeater)
		return -ENOMEM;
	repeater->dev = dev;

	for (i = 0; i < ARRAY_SIZE(repeater->supplies); i++) {
		const char *name = tusb2e11_supply_names[i];

		/* Require a real regulator for each supply. */
		repeater->supplies[i] = devm_regulator_get_optional(dev, name);
		if (IS_ERR(repeater->supplies[i]))
			return dev_err_probe(dev, PTR_ERR(repeater->supplies[i]),
					     "failed to get %s supply\n", name);
	}

	role_desc.fwnode = dev_fwnode(dev);
	role_desc.set = tusb2e11_set_role;
	role_desc.driver_data = repeater;
	repeater->role_switch = usb_role_switch_register(dev, &role_desc);
	if (IS_ERR(repeater->role_switch))
		return dev_err_probe(dev, PTR_ERR(repeater->role_switch),
				     "failed to register role switch\n");

	i2c_set_clientdata(client, repeater);
	return 0;
}

static void tusb2e11_remove(struct i2c_client *client)
{
	struct tusb2e11 *repeater = i2c_get_clientdata(client);
	int ret;

	scoped_guard(mutex, &tusb2e11_role_lock) {
		usb_role_switch_set_drvdata(repeater->role_switch, NULL);
		ret = tusb2e11_drop_votes(repeater, repeater->votes);
	}
	usb_role_switch_unregister(repeater->role_switch);
	if (ret)
		dev_warn(&client->dev, "failed to release repeater supplies\n");
}

static const struct of_device_id tusb2e11_of_match[] = {
	{ .compatible = "ti,tusb2e11" },
	{ }
};
MODULE_DEVICE_TABLE(of, tusb2e11_of_match);

static struct i2c_driver tusb2e11_driver = {
	.driver = {
		.name = "tusb2e11",
		.of_match_table = tusb2e11_of_match,
	},
	.probe = tusb2e11_probe,
	.remove = tusb2e11_remove,
};
module_i2c_driver(tusb2e11_driver);

MODULE_DESCRIPTION("Texas Instruments TUSB2E11 eUSB2 repeater");
MODULE_LICENSE("GPL");
