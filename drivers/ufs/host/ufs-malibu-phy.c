// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/reset.h>

#include <ufs/ufshcd.h>
#include "ufs-malibu-phy.h"

#define MALIBU_UFS_TOP_STATUS		0x008
#define MALIBU_UFS_TOP_CTRL		0x100
#define MALIBU_UFS_TOP_REXT		0x110
#define MALIBU_UFS_TOP_LSS_MODE		0x514
#define MALIBU_UFS_TOP_POWER_READY	BIT(4)
#define MALIBU_UFS_TOP_PHY_READY		BIT(21)
#define MALIBU_UFS_TOP_NO_PATCH_MASK	GENMASK(19, 16)
#define MALIBU_UFS_REXT_TRIM		GENMASK(5, 0)
#define MALIBU_UFS_REXT_OVERRIDE	BIT(6)
#define MALIBU_UFS_SS_PHY_STATUS	0x014
#define MALIBU_UFS_SS_PHY_BUSY		GENMASK(3, 0)
#define MALIBU_UFS_PSM_STATE		GENMASK(4, 0)
#define MALIBU_UFS_PSM_OFF		0x11
#define MALIBU_UFS_PSM_ON		0x10

#define MALIBU_MPHY_ADDR_LO		0x810e
#define MALIBU_MPHY_ADDR_HI		0x810f
#define MALIBU_MPHY_DATA_IN_LO		0x8110
#define MALIBU_MPHY_DATA_IN_HI		0x8111
#define MALIBU_MPHY_DATA_OUT_LO		0x8112
#define MALIBU_MPHY_DATA_OUT_HI		0x8113
#define MALIBU_MPHY_RW			0x8114

struct malibu_dme_setting {
	u32 mib;
	u32 value;
};

static int malibu_set_attrs(struct ufs_hba *hba, const struct malibu_dme_setting *settings,
			    size_t count)
{
	size_t i;
	int ret;

	for (i = 0; i < count; i++) {
		ret = ufshcd_dme_set(hba, settings[i].mib, settings[i].value);
		if (ret)
			return ret;
	}
	return 0;
}

static int malibu_mphy_write(struct ufs_hba *hba, u16 addr, u16 value)
{
	const struct malibu_dme_setting settings[] = {
		{ UIC_ARG_MIB(MALIBU_MPHY_ADDR_LO), addr & 0xff },
		{ UIC_ARG_MIB(MALIBU_MPHY_ADDR_HI), addr >> 8 },
		{ UIC_ARG_MIB(MALIBU_MPHY_DATA_IN_LO), value & 0xff },
		{ UIC_ARG_MIB(MALIBU_MPHY_DATA_IN_HI), value >> 8 },
		{ UIC_ARG_MIB(MALIBU_MPHY_RW), 1 },
	};

	return malibu_set_attrs(hba, settings, ARRAY_SIZE(settings));
}

static int malibu_mphy_set_bits(struct ufs_hba *hba, u16 addr, u16 bits)
{
	const struct malibu_dme_setting settings[] = {
		{ UIC_ARG_MIB(MALIBU_MPHY_ADDR_LO), addr & 0xff },
		{ UIC_ARG_MIB(MALIBU_MPHY_ADDR_HI), addr >> 8 },
		{ UIC_ARG_MIB(MALIBU_MPHY_RW), 0 },
	};
	u32 lo, hi;
	int ret;

	ret = malibu_set_attrs(hba, settings, ARRAY_SIZE(settings));
	if (ret)
		return ret;
	ret = ufshcd_dme_get(hba, UIC_ARG_MIB(MALIBU_MPHY_DATA_OUT_LO), &lo);
	if (ret)
		return ret;
	ret = ufshcd_dme_get(hba, UIC_ARG_MIB(MALIBU_MPHY_DATA_OUT_HI), &hi);
	if (ret)
		return ret;
	if (lo > U8_MAX || hi > U8_MAX)
		return -EIO;

	return malibu_mphy_write(hba, addr, (hi << 8) | lo | bits);
}

int ufs_malibu_phy_read_config(struct device *dev, struct ufs_malibu_phy *phy)
{
	struct device_node *np = dev->of_node;
	int count, ret;

	if (of_property_read_bool(np, "google,enable-phy-patching"))
		return dev_err_probe(dev, -EOPNOTSUPP, "PHY SRAM patching is unsupported\n");

	count = of_property_count_u32_elems(np, "phy-cal-data");
	if (count != ARRAY_SIZE(phy->calibration))
		return dev_err_probe(dev, count < 0 ? count : -EINVAL,
				     "expected %zu ABL PHY calibration cells\n",
				     ARRAY_SIZE(phy->calibration));

	ret = of_property_read_u32_array(np, "phy-cal-data", phy->calibration,
					 ARRAY_SIZE(phy->calibration));
	if (ret)
		return dev_err_probe(dev, ret, "unreadable ABL PHY calibration\n");
	ret = of_property_read_u32(np, "google,rext-val", &phy->rext);
	if (ret)
		return dev_err_probe(dev, ret, "missing ABL PHY resistor trim\n");
	if (phy->rext > MALIBU_UFS_REXT_TRIM)
		return dev_err_probe(dev, -ERANGE, "invalid ABL PHY resistor trim\n");

	phy->hs_lss = of_property_read_bool(np, "google,enable-hs-lss");
	phy->pmc_workaround = of_property_read_bool(np, "google,mphy-pmc-war");
	return 0;
}

static void malibu_top_update(struct ufs_malibu_phy *phy, u32 reg, u32 mask, u32 val)
{
	writel((readl(phy->top + reg) & ~mask) | val, phy->top + reg);
}

/* The preparation domain must be powered and the core still off. */
int ufs_malibu_phy_prepare(struct ufs_malibu_phy *phy)
{
	u32 status;
	int ret;

	phy->initialized = false;
	ret = readl_poll_timeout(phy->psm, status,
				 (status & MALIBU_UFS_PSM_STATE) == MALIBU_UFS_PSM_OFF,
				 10, 10000);
	if (ret)
		return ret;

	malibu_top_update(phy, MALIBU_UFS_TOP_CTRL, MALIBU_UFS_TOP_NO_PATCH_MASK,
			  MALIBU_UFS_TOP_NO_PATCH_MASK);
	malibu_top_update(phy, 0x208, BIT(1), BIT(1));
	malibu_top_update(phy, 0x214, BIT(0), BIT(0));
	malibu_top_update(phy, 0x210, BIT(1), BIT(1));
	malibu_top_update(phy, MALIBU_UFS_TOP_REXT, MALIBU_UFS_REXT_TRIM | MALIBU_UFS_REXT_OVERRIDE,
			  MALIBU_UFS_REXT_OVERRIDE | phy->rext);
	writel(phy->hs_lss, phy->top + MALIBU_UFS_TOP_LSS_MODE);
	return 0;
}

/* Check the PHY PSM and UFS TOP before permitting accesses to the HCI. */
int ufs_malibu_phy_check_power(struct ufs_malibu_phy *phy)
{
	u32 status;
	int ret;

	ret = readl_poll_timeout(phy->psm, status,
				 (status & MALIBU_UFS_PSM_STATE) == MALIBU_UFS_PSM_ON,
				 10, 10000);
	if (ret)
		return ret;

	return readl_poll_timeout(phy->top + MALIBU_UFS_TOP_STATUS, status,
				  status & MALIBU_UFS_TOP_POWER_READY, 10, 400);
}

static int malibu_apply_calibration(struct ufs_hba *hba, struct ufs_malibu_phy *phy)
{
	static const struct {
		u16 reg;
		u16 mask;
	} lane_tuning[] = {
		{ 0x3128, 0x000c }, { 0x3328, 0x000c },
		{ 0x3101, 0x0002 }, { 0x3301, 0x0002 },
		{ 0x3103, 0x0060 }, { 0x3303, 0x0060 },
	};
	size_t i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(phy->calibration); i++) {
		u32 cell = phy->calibration[i];

		ret = malibu_mphy_write(hba, cell, cell >> 16);
		if (ret)
			return ret;
	}
	ret = malibu_mphy_write(hba, 0x017c, 2);
	if (ret)
		return ret;
	for (i = 0; i < ARRAY_SIZE(lane_tuning); i++) {
		ret = malibu_mphy_set_bits(hba, lane_tuning[i].reg, lane_tuning[i].mask);
		if (ret)
			return ret;
	}
	return 0;
}

/* Run with HCE and local DME commands available, before UIC link startup. */
int ufs_malibu_phy_init(struct ufs_hba *hba, struct ufs_malibu_phy *phy)
{
	const u32 slew = phy->hs_lss ? 0x4a : 0xba;
	const struct malibu_dme_setting tx_eq[] = {
		{ UIC_ARG_MIB_SEL(0x8014, 0), 0x2e },
		{ UIC_ARG_MIB_SEL(0x8014, 1), 0x2e },
		{ UIC_ARG_MIB_SEL(0x8016, 0), 0 },
		{ UIC_ARG_MIB_SEL(0x8016, 1), 0 },
		{ UIC_ARG_MIB_SEL(0x800a, 0), slew },
		{ UIC_ARG_MIB_SEL(0x800a, 1), slew },
		{ UIC_ARG_MIB(0xd085), 1 },
		{ UIC_ARG_MIB(0x8101), 5 },
		{ UIC_ARG_MIB(0x8115), 1 },
	};
	u32 status;
	int ret, reset_ret;

	if (phy->initialized)
		return 0;

	ret = malibu_set_attrs(hba, tx_eq, ARRAY_SIZE(tx_eq));
	if (ret)
		return ret;
	ret = reset_control_deassert(phy->reset);
	if (ret)
		return ret;
	ret = readl_poll_timeout(phy->top + MALIBU_UFS_TOP_CTRL, status,
				 status & MALIBU_UFS_TOP_PHY_READY, 50000, 570000);
	if (ret)
		goto assert_reset;

	ret = malibu_apply_calibration(hba, phy);
	if (ret)
		goto assert_reset;
	if (phy->hs_lss && phy->pmc_workaround) {
		ret = malibu_mphy_write(hba, 0x10fc, 0x2d56);
		if (ret)
			goto assert_reset;
		ret = malibu_mphy_write(hba, 0x12fc, 0x2d56);
		if (ret)
			goto assert_reset;
	}
	ret = ufshcd_dme_set(hba, UIC_ARG_MIB(0xd0c1), 0);
	if (ret)
		goto assert_reset;
	ret = readl_poll_timeout(phy->subsystem + MALIBU_UFS_SS_PHY_STATUS, status,
				 !(status & MALIBU_UFS_SS_PHY_BUSY), 1000, 1000000);
	if (!ret) {
		phy->initialized = true;
		return 0;
	}

assert_reset:
	reset_ret = reset_control_assert(phy->reset);
	if (reset_ret)
		dev_err(hba->dev, "failed to reassert M-PHY reset: %d\n", reset_ret);
	return ret;
}

MODULE_DESCRIPTION("Google Tensor G6 Malibu UFS PHY initialization");
MODULE_LICENSE("GPL");
