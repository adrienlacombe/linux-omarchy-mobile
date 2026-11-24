/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_MALIBU_CPM_H
#define _LINUX_MALIBU_CPM_H

struct device;

enum malibu_cpm_ufs_refclk_sample {
	MALIBU_CPM_UFS_GPIO_REFCLK_GATE,
	MALIBU_CPM_UFS_GPIO_REFCLK_ACG,
	MALIBU_CPM_UFS_MPHY_REFCLK_GATE,
};

int malibu_cpm_ufs_decouple_enable(struct device *supplier);
void malibu_cpm_ufs_log_rails(struct device *supplier, const char *phase);
int malibu_cpm_ufs_refclk_sample(struct device *supplier, enum malibu_cpm_ufs_refclk_sample sample,
				const char *phase);

#endif /* _LINUX_MALIBU_CPM_H */
