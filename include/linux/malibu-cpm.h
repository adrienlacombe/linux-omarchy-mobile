/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_MALIBU_CPM_H
#define _LINUX_MALIBU_CPM_H

#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

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
int malibu_cpm_ufs_check_idle_stage(struct device *supplier, bool prep_resumed);

#if IS_ENABLED(CONFIG_COMMON_CLK_GOOGLE_MALIBU_CPM)
int malibu_cpm_check_be_csr(struct device *supplier);
#else
static inline int malibu_cpm_check_be_csr(struct device *supplier)
{
	return -EOPNOTSUPP;
}
#endif

#endif /* _LINUX_MALIBU_CPM_H */
