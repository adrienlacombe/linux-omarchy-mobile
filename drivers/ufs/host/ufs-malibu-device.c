// SPDX-License-Identifier: GPL-2.0-only

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/malibu-cpm.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>

#include "ufs-malibu.h"

#define MALIBU_UFS_VDD075_UV	750000
#define MALIBU_UFS_VDD12_UV	1200000
#define MALIBU_UFS_VCC_UV	2500000
#define MALIBU_UFS_VCCQ_UV	1200000

static int malibu_check_voltage(struct regulator *rail, int expected)
{
	int voltage = regulator_get_voltage(rail);

	if (voltage < 0)
		return voltage;
	return voltage == expected ? 0 : -ERANGE;
}

static int malibu_check_rail(struct regulator *rail, int voltage)
{
	int enabled = regulator_is_enabled(rail);

	if (enabled < 0)
		return enabled;
	if (!enabled)
		return -EIO;
	return malibu_check_voltage(rail, voltage);
}

/* Call with the regulators acquired, before driving the UFS enable pin. */
int ufs_malibu_device_prepare(struct ufs_malibu_device *device, struct device *cpm)
{
	int ret;

	if (device->stage != MALIBU_DEVICE_OFF)
		return -EBUSY;
	if (device->prepared)
		return 0;

	ret = malibu_check_rail(device->vdd075, MALIBU_UFS_VDD075_UV);
	if (ret)
		return ret;
	ret = malibu_check_rail(device->vdd12, MALIBU_UFS_VDD12_UV);
	if (ret)
		return ret;
	ret = regulator_set_voltage(device->vcc, MALIBU_UFS_VCC_UV, MALIBU_UFS_VCC_UV);
	if (ret)
		return ret;
	ret = regulator_set_voltage(device->vccq, MALIBU_UFS_VCCQ_UV, MALIBU_UFS_VCCQ_UV);
	if (ret)
		return ret;
	ret = malibu_cpm_ufs_decouple_enable(cpm);
	if (!ret)
		device->prepared = true;
	return ret;
}

int ufs_malibu_device_off(struct ufs_malibu_device *device)
{
	enum malibu_device_stage stage = device->stage;
	int err, ret = 0;

	if (stage == MALIBU_DEVICE_OFF)
		return 0;
	if (stage == MALIBU_DEVICE_FAULT)
		return -EIO;

	if (stage >= MALIBU_DEVICE_CLOCK) {
		clk_disable_unprepare(device->refclk);
		usleep_range(10, 12);
	}
	if (device->lss_active)
		ret = gpiod_direction_output(device->lss, 0);
	if (stage >= MALIBU_DEVICE_RESET) {
		err = gpiod_direction_output(device->reset, 1);
		ret = ret ?: err;
	}
	if (stage >= MALIBU_DEVICE_POWER) {
		err = gpiod_direction_output(device->power, 0);
		ret = ret ?: err;
	}
	err = pinctrl_select_state(device->pinctrl, device->refclk_off);
	ret = ret ?: err;
	if (!ret)
		device->lss_active = false;
	device->stage = ret ? MALIBU_DEVICE_FAULT : MALIBU_DEVICE_OFF;

	return ret;
}

/* The HSIO-S pad bank must be accessible before selecting pads or GPIOs. */
int ufs_malibu_device_on(struct ufs_malibu_device *device, bool hs_lss)
{
	int ret, cleanup;

	if (device->stage == MALIBU_DEVICE_ON)
		return 0;
	if (device->stage != MALIBU_DEVICE_OFF || !device->prepared)
		return -EIO;
	if (!gpiod_is_active_low(device->reset) || gpiod_is_active_low(device->power) ||
	    (hs_lss && (!device->lss || gpiod_is_active_low(device->lss))))
		return -EINVAL;

	ret = pinctrl_select_state(device->pinctrl, device->gpio_prepare);
	if (ret)
		return ret;

	device->stage = MALIBU_DEVICE_PINS;
	ret = pinctrl_select_state(device->pinctrl, device->refclk_on);
	if (ret)
		goto fail;
	device->stage = MALIBU_DEVICE_RESET;
	ret = gpiod_direction_output(device->reset, 1);
	if (ret)
		goto fail;
	device->stage = MALIBU_DEVICE_POWER;
	ret = gpiod_direction_output(device->power, 1);
	if (ret)
		goto fail;
	usleep_range(1500, 2000);

	ret = malibu_check_rail(device->vcc, MALIBU_UFS_VCC_UV);
	if (ret)
		goto fail;
	ret = malibu_check_rail(device->vccq, MALIBU_UFS_VCCQ_UV);
	if (ret)
		goto fail;

	if (hs_lss) {
		device->lss_active = true;
		ret = gpiod_direction_output(device->lss, 1);
		if (ret)
			goto fail;
		usleep_range(100, 110);
	}
	ret = clk_prepare_enable(device->refclk);
	if (ret)
		goto fail;
	device->stage = MALIBU_DEVICE_CLOCK;
	usleep_range(300, 310);
	ret = gpiod_set_value_cansleep(device->reset, 0);
	if (ret)
		goto fail;

	device->stage = MALIBU_DEVICE_ON;
	return 0;

fail:
	cleanup = ufs_malibu_device_off(device);
	if (cleanup)
		dev_err(device->dev, "UFS power sequence rollback failed: %pe\n", ERR_PTR(cleanup));
	return ret;
}
