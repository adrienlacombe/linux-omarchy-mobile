// SPDX-License-Identifier: GPL-2.0-only

#include <linux/io.h>

#include <ufs/ufshcd.h>
#include "ufs-malibu.h"

#define MALIBU_UFS_HCI_LINK_CONFIG	0x0f8
#define MALIBU_UFS_TOP_LINK_CONFIG	0x200

static int ufs_malibu_prepare_link_startup(struct ufs_hba *hba)
{
	struct ufs_malibu *ufs = ufshcd_get_variant(hba);

	if (!ufs || ufs->device.stage != MALIBU_DEVICE_ON)
		return -EHOSTDOWN;

	ufshcd_writel(hba, 200, MALIBU_UFS_HCI_LINK_CONFIG);
	writel(0x1111, ufs->phy.top + MALIBU_UFS_TOP_LINK_CONFIG);
	return ufs_malibu_phy_init(hba, &ufs->phy);
}

const struct ufs_hba_variant_ops ufs_malibu_vops = {
	.name = "malibu",
	.prepare_link_startup = ufs_malibu_prepare_link_startup,
};
