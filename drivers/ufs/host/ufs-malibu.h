/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _UFS_MALIBU_H
#define _UFS_MALIBU_H

#include "ufs-malibu-phy.h"

struct clk;
struct gpio_desc;
struct pinctrl;
struct pinctrl_state;
struct regulator;
struct ufs_hba_variant_ops;

#define MALIBU_UFS_DMA_BITS 36

enum malibu_device_stage {
	MALIBU_DEVICE_OFF,
	MALIBU_DEVICE_PINS,
	MALIBU_DEVICE_RESET,
	MALIBU_DEVICE_POWER,
	MALIBU_DEVICE_CLOCK,
	MALIBU_DEVICE_ON,
	MALIBU_DEVICE_FAULT,
};

struct ufs_malibu_device {
	struct device *dev;
	struct regulator *vdd075;
	struct regulator *vdd12;
	struct regulator *vcc;
	struct regulator *vccq;
	struct gpio_desc *reset;
	struct gpio_desc *power;
	struct gpio_desc *lss;
	struct clk *refclk;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_prepare;
	struct pinctrl_state *refclk_on;
	struct pinctrl_state *refclk_off;
	enum malibu_device_stage stage;
	bool lss_active;
	bool prepared;
};

struct ufs_malibu {
	struct ufs_malibu_phy phy;
	struct ufs_malibu_device device;
	struct clk *deep_h8;
	void __iomem *psm_hci;
	int deep_h8_release_result;
};

int ufs_malibu_device_prepare(struct ufs_malibu_device *device, struct device *cpm);
int ufs_malibu_device_on(struct ufs_malibu_device *device, bool hs_lss);
int ufs_malibu_device_off(struct ufs_malibu_device *device);

extern const struct ufs_hba_variant_ops ufs_malibu_vops;

#endif /* _UFS_MALIBU_H */
