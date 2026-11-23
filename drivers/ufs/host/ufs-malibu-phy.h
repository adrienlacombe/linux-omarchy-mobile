/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _UFS_MALIBU_PHY_H
#define _UFS_MALIBU_PHY_H

#include <linux/types.h>

struct device;
struct reset_control;
struct ufs_hba;

struct ufs_malibu_phy {
	void __iomem *top;
	void __iomem *subsystem;
	void __iomem *psm;
	struct reset_control *reset;
	u32 calibration[74];
	u32 rext;
	bool hs_lss;
	bool pmc_workaround;
	bool initialized;
	bool power_ready;
};

int ufs_malibu_phy_read_config(struct device *dev, struct ufs_malibu_phy *phy);
int ufs_malibu_phy_prepare(struct ufs_malibu_phy *phy);
int ufs_malibu_phy_post_power_on(struct ufs_malibu_phy *phy, u32 refclk_wait_us);
int ufs_malibu_phy_init(struct ufs_hba *hba, struct ufs_malibu_phy *phy);

#endif /* _UFS_MALIBU_PHY_H */
