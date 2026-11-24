/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_MALIBU_UFS_PSM_H
#define _LINUX_MALIBU_UFS_PSM_H

#include <linux/device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>

#define MALIBU_HSIOS_PSM_STATE	0x1f
#define MALIBU_HSIOS_PSM_ON	0x10
#define MALIBU_HSIOS_PSM_ADDR	0x3c00b108
#define MALIBU_UFS_HC_PSM_ADDR	0x3c00c210
#define MALIBU_UFS_PHY_PSM_ADDR	0x3c00d108

struct malibu_ufs_psm_regs {
	void __iomem *hsios;
	void __iomem *hc;
	void __iomem *phy;
};

struct malibu_ufs_psm_sample {
	u32 hsios;
	u32 hc;
	u32 phy;
	bool children_valid;
};

static inline void __iomem *
malibu_ufs_map_psm(struct device *dev, struct device_node *ufs,
		   const char *name, phys_addr_t expected)
{
	struct resource res;
	int index;

	index = of_property_match_string(ufs, "reg-names", name);
	if (index < 0 || of_address_to_resource(ufs, index, &res) ||
	    res.start != expected || resource_size(&res) != sizeof(u32))
		return NULL;

	/* Share the status registers with the UFS consumer's resource mappings. */
	return devm_ioremap(dev, res.start, sizeof(u32));
}

static inline bool malibu_ufs_map_psms(struct device *dev, struct malibu_ufs_psm_regs *regs)
{
	struct device_node *ufs;

	if (!of_machine_is_compatible("google,malibu-grizzly"))
		return false;

	ufs = of_find_node_by_path("ufs0");
	if (ufs && of_device_is_available(ufs) &&
	    of_device_is_compatible(ufs, "google,malibu-ufshc")) {
		regs->hsios = malibu_ufs_map_psm(dev, ufs, "psm-hsios", MALIBU_HSIOS_PSM_ADDR);
		regs->hc = malibu_ufs_map_psm(dev, ufs, "psm-hci", MALIBU_UFS_HC_PSM_ADDR);
		regs->phy = malibu_ufs_map_psm(dev, ufs, "psm-phy", MALIBU_UFS_PHY_PSM_ADDR);
	}
	of_node_put(ufs);

	if (regs->hsios && regs->hc && regs->phy)
		return true;

	dev_info(dev, "ufs-uic-v33 early PSM resources unavailable\n");
	return false;
}

static inline void malibu_ufs_sample_psms(const struct malibu_ufs_psm_regs *regs,
					  struct malibu_ufs_psm_sample *sample)
{
	/* D1 DBL polls this wrapper status before accessing the child PSMs. */
	sample->hsios = readl(regs->hsios);
	sample->children_valid = (sample->hsios & MALIBU_HSIOS_PSM_STATE) == MALIBU_HSIOS_PSM_ON;
	if (sample->children_valid) {
		sample->hc = readl(regs->hc);
		sample->phy = readl(regs->phy);
	}
}

static inline void malibu_ufs_log_psms(struct device *dev, const char *phase,
				       const struct malibu_ufs_psm_sample *sample)
{
	if (sample->children_valid)
		dev_info(dev, "ufs-uic-v33 %s PSM HSIOS=0x%08x HC=0x%08x PHY=0x%08x\n",
			 phase, sample->hsios, sample->hc, sample->phy);
	else
		dev_info(dev, "ufs-uic-v33 %s PSM HSIOS=0x%08x child PSMs skipped\n",
			 phase, sample->hsios);
}

#endif /* _LINUX_MALIBU_UFS_PSM_H */
