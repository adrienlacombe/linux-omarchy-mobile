/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _UFS_MALIBU_PHY_H
#define _UFS_MALIBU_PHY_H

#include <linux/bits.h>
#include <linux/types.h>

#define MALIBU_UFS_PSM_STATE		GENMASK(4, 0)
#define MALIBU_UFS_PSM_OFF		0x11
#define MALIBU_UFS_PSM_ON		0x10

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
	u32 saved_refclk_ctrl;
	u32 saved_refclk_delay;
	u32 saved_refclk_aux;
	bool hs_lss;
	bool pmc_workaround;
	bool initialized;
	bool power_ready;
};

int ufs_malibu_phy_read_config(struct device *dev, struct ufs_malibu_phy *phy);
int ufs_malibu_phy_prepare(struct ufs_malibu_phy *phy);
int ufs_malibu_phy_wait_power(struct device *dev, struct ufs_malibu_phy *phy,
			      const char *phase);
int ufs_malibu_phy_post_power_on(struct device *dev, struct ufs_malibu_phy *phy,
				 u32 refclk_wait_us);
void ufs_malibu_phy_restore_refclk(struct ufs_malibu_phy *phy);
int ufs_malibu_phy_init(struct ufs_hba *hba, struct ufs_malibu_phy *phy);

#endif /* _UFS_MALIBU_PHY_H */
