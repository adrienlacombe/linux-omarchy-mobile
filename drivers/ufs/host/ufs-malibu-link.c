// SPDX-License-Identifier: GPL-2.0-only

#include <linux/dma-mapping.h>
#include <linux/io.h>

#include <ufs/ufshcd.h>
#include "ufs-malibu.h"

#define MALIBU_UFS_HCI_LINK_CONFIG	0x0f8
#define MALIBU_UFS_TOP_LINK_CONFIG	0x200

static int ufs_malibu_prepare_link_startup(struct ufs_hba *hba)
{
	struct ufs_malibu *ufs = ufshcd_get_variant(hba);

	if (!ufs || ufs->device.stage != MALIBU_DEVICE_ON || !ufs->phy.power_ready)
		return -EHOSTDOWN;

	ufshcd_writel(hba, 200, MALIBU_UFS_HCI_LINK_CONFIG);
	writel(0x1111, ufs->phy.top + MALIBU_UFS_TOP_LINK_CONFIG);
	return ufs_malibu_phy_init(hba, &ufs->phy);
}

static int ufs_malibu_set_dma_mask(struct ufs_hba *hba)
{
	if (!(hba->capabilities & MASK_64_ADDRESSING_SUPPORT))
		return -EOPNOTSUPP;

	return dma_set_mask_and_coherent(hba->dev, DMA_BIT_MASK(MALIBU_UFS_DMA_BITS));
}

const struct ufs_hba_variant_ops ufs_malibu_vops = {
	.name = "malibu",
	.set_dma_mask = ufs_malibu_set_dma_mask,
	.prepare_link_startup = ufs_malibu_prepare_link_startup,
};
