// SPDX-License-Identifier: GPL-2.0-only

#include <dt-bindings/clock/google,malibu-cpm-io.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-map-ops.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/iopoll.h>
#include <linux/iommu.h>
#include <linux/iommu-dma.h>
#include <linux/irq.h>
#include <linux/malibu-cpm.h>
#include <linux/malibu-ufs-psm.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/slab.h>

#include <ufs/ufshcd.h>
#include "ufs-malibu.h"

#define MALIBU_AMB_BASE			0x04190000
#define MALIBU_UFS_SID			0x8000
#define MALIBU_UFS_SGIA_BASE		0x3c272040
#define MALIBU_UFS_HCI_INPUT		0x1a
#define MALIBU_UFS_HCI_SPI		0x17a
#define MALIBU_UFS_MCQ_SPI_BASE		0x168
#define MALIBU_UFS_MCQ_IRQS		16
#define MALIBU_GIC_SPI			0
#define MALIBU_LPCM8_BASE		0x3c009000
#define MALIBU_LPCM8_PSM_COUNT		GENMASK(13, 8)
#define MALIBU_LPCM8_PSM_DESC(psm)	(0x10 + 8 * (psm))
#define MALIBU_LPCM8_PSM_STATE		0xf
#define MALIBU_LPCM8_PSM3		3
#define MALIBU_LPCM8_CORE_OFF_PSM3_STATE	0x1
#define MALIBU_LPCM8_D1_PSM3_DESC_VALUE	0x00021325
#define MALIBU_LPCM8_D1_PSM3_STATUS	0x3c00e108
#define MALIBU_LPCM8_D1_PHY_RESET_ACK	0x3c01b05c
#define MALIBU_LPCM8_PHY_RESET_RELEASED	BIT(12)

static int ufs_malibu_check_dma(struct device *dev)
{
	struct iommu_domain *domain = iommu_get_domain_for_dev(dev);
	struct of_phandle_args iommu;
	struct resource res;
	phys_addr_t phys;
	dma_addr_t addr;
	void *buf;
	int ret;

	if (of_count_phandle_with_args(dev->of_node, "iommus", "#iommu-cells") != 1)
		return -EINVAL;
	ret = of_parse_phandle_with_args(dev->of_node, "iommus", "#iommu-cells", 0, &iommu);
	if (ret)
		return ret;

	if (iommu.args_count != 1 || iommu.args[0] != MALIBU_UFS_SID ||
	    !of_device_is_compatible(iommu.np, "arm,smmu-v3") ||
	    !of_device_is_available(iommu.np) ||
	    of_address_to_resource(iommu.np, 0, &res) || res.start != MALIBU_AMB_BASE)
		ret = -EINVAL;
	of_node_put(iommu.np);
	if (ret)
		return ret;

	if (!domain || domain->type != IOMMU_DOMAIN_DMA ||
	    !use_dma_iommu(dev) || !dev_is_dma_coherent(dev))
		return -ENODEV;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(MALIBU_UFS_DMA_BITS));
	if (ret)
		return ret;

	buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	/* Exercise the live DMA page table without submitting work to the HCI. */
	addr = dma_map_single(dev, buf, PAGE_SIZE, DMA_TO_DEVICE);
	if (dma_mapping_error(dev, addr)) {
		ret = -EIO;
		goto free_buf;
	}

	phys = virt_to_phys(buf);
	if (addr > DMA_BIT_MASK(MALIBU_UFS_DMA_BITS) - (PAGE_SIZE - 1) ||
	    iommu_iova_to_phys(domain, addr) != phys ||
	    iommu_iova_to_phys(domain, addr + PAGE_SIZE - 1) != phys + PAGE_SIZE - 1)
		ret = -EIO;
	dma_unmap_single(dev, addr, PAGE_SIZE, DMA_TO_DEVICE);
free_buf:
	kfree(buf);
	return ret;
}

static bool ufs_malibu_gic_spi(const struct of_phandle_args *route, u32 spi)
{
	return of_device_is_compatible(route->np, "arm,gic-v3") &&
	       route->args_count == 4 && route->args[0] == MALIBU_GIC_SPI &&
	       route->args[1] == spi && route->args[2] == IRQ_TYPE_LEVEL_HIGH &&
	       route->args[3] == 0;
}

static int ufs_malibu_get_hci_irq(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct of_phandle_args sgia, parent, direct;
	struct resource res;
	int irq, i, ret;

	if (of_irq_count(np) != 1 + MALIBU_UFS_MCQ_IRQS)
		return -EINVAL;
	ret = of_irq_parse_one(np, 0, &sgia);
	if (ret)
		return ret;

	if (!of_device_is_compatible(sgia.np, "google,malibu-level-sgia") ||
	    sgia.args_count != 1 || sgia.args[0] != MALIBU_UFS_HCI_INPUT ||
	    of_address_to_resource(sgia.np, 0, &res) || res.start != MALIBU_UFS_SGIA_BASE)
		ret = -EINVAL;
	else
		ret = of_irq_parse_one(sgia.np, 0, &parent);
	of_node_put(sgia.np);
	if (ret)
		return ret;

	if (!ufs_malibu_gic_spi(&parent, MALIBU_UFS_HCI_SPI)) {
		ret = -EINVAL;
		goto put_parent;
	}
	for (i = 0; i < MALIBU_UFS_MCQ_IRQS; i++) {
		ret = of_irq_parse_one(np, i + 1, &direct);
		if (ret)
			goto put_parent;
		if (direct.np != parent.np ||
		    !ufs_malibu_gic_spi(&direct, MALIBU_UFS_MCQ_SPI_BASE + i))
			ret = -EINVAL;
		of_node_put(direct.np);
		if (ret)
			goto put_parent;
	}

	irq = platform_get_irq_byname(pdev, "hci");
	if (irq < 0) {
		ret = irq;
		goto put_parent;
	}
	ret = platform_get_irq(pdev, 0);
	if (ret >= 0)
		ret = ret == irq ? irq : -EINVAL;
put_parent:
	of_node_put(parent.np);
	return ret;
}

static struct device *ufs_malibu_get_cpm(struct device *dev)
{
	struct of_phandle_args prep, core;
	struct device *cpm = ERR_PTR(-EINVAL);
	int ret;

	if (of_count_phandle_with_args(dev->of_node, "power-domains", "#power-domain-cells") != 2)
		return cpm;
	ret = of_parse_phandle_with_args(dev->of_node, "power-domains", "#power-domain-cells",
					0, &prep);
	if (ret)
		return ERR_PTR(ret);
	ret = of_parse_phandle_with_args(dev->of_node, "power-domains", "#power-domain-cells",
					1, &core);
	if (ret) {
		cpm = ERR_PTR(ret);
		goto put_prep;
	}

	if (prep.np == core.np && of_device_is_compatible(prep.np, "google,malibu-cpm-io") &&
	    prep.args_count == 1 && prep.args[0] == MALIBU_CPM_PD_UFS_PREP &&
	    core.args_count == 1 && core.args[0] == MALIBU_CPM_PD_UFS)
		cpm = bus_find_device_by_of_node(&platform_bus_type, prep.np) ?:
			      ERR_PTR(-EPROBE_DEFER);
	of_node_put(core.np);
put_prep:
	of_node_put(prep.np);
	return cpm;
}

static int ufs_malibu_get_supplies(struct ufs_malibu_device *device)
{
	struct device *dev = device->dev;
	const char * const names[] = { "vdd0p75", "vdd1p2", "vcc", "vccq" };
	struct regulator **rails[] = {
		&device->vdd075, &device->vdd12, &device->vcc, &device->vccq,
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		*rails[i] = devm_regulator_get_optional(dev, names[i]);
		if (IS_ERR(*rails[i]))
			return dev_err_probe(dev, PTR_ERR(*rails[i]),
					     "getting %s rail\n", names[i]);
	}

	device->refclk = devm_clk_get(dev, "gpio-ref");
	if (IS_ERR(device->refclk))
		return PTR_ERR(device->refclk);

	device->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(device->pinctrl))
		return PTR_ERR(device->pinctrl);
	device->gpio_prepare = pinctrl_lookup_state(device->pinctrl, "gpio-prepare");
	if (IS_ERR(device->gpio_prepare))
		return PTR_ERR(device->gpio_prepare);
	device->refclk_on = pinctrl_lookup_state(device->pinctrl, "refclk-on");
	if (IS_ERR(device->refclk_on))
		return PTR_ERR(device->refclk_on);
	device->refclk_off = pinctrl_lookup_state(device->pinctrl, "refclk-off");
	return PTR_ERR_OR_ZERO(device->refclk_off);
}

static int ufs_malibu_get_gpios(struct ufs_malibu_device *device)
{
	struct device *dev = device->dev;

	device->reset = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(device->reset))
		return PTR_ERR(device->reset);
	device->power = devm_gpiod_get(dev, "power", GPIOD_ASIS);
	if (IS_ERR(device->power))
		return PTR_ERR(device->power);
	device->lss = devm_gpiod_get(dev, "lss", GPIOD_ASIS);
	return PTR_ERR_OR_ZERO(device->lss);
}

static int ufs_malibu_check_idle(void __iomem *hci)
{
	u32 cap = readl(hci + REG_CONTROLLER_CAPABILITIES);

	if (!(cap & MASK_64_ADDRESSING_SUPPORT) || cap & MASK_LSDB_SUPPORT)
		return -EOPNOTSUPP;
	if ((cap & MASK_MCQ_SUPPORT) && (readl(hci + REG_UFS_MEM_CFG) & MCQ_MODE_SELECT))
		return -EBUSY;
	if (readl(hci + REG_UTP_TRANSFER_REQ_DOOR_BELL) ||
	    readl(hci + REG_UTP_TASK_REQ_DOOR_BELL))
		return -EBUSY;

	return 0;
}

static void __iomem *ufs_malibu_map(struct platform_device *pdev, const char *name,
				   resource_size_t base, resource_size_t size)
{
	struct resource *res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);

	if (!res || res->start != base || resource_size(res) != size)
		return ERR_PTR(-EINVAL);

	return devm_ioremap_resource(&pdev->dev, res);
}

struct malibu_ufs_psm_status {
	u32 hci;
	u32 phy;
};

struct malibu_ufs_setup_sample {
	const char *phase;
	struct malibu_ufs_psm_status before;
	struct malibu_ufs_psm_status after;
	int ret;
};

struct malibu_ufs_lpcm_sample {
	u32 desc;
	u32 addr;
	u32 raw;
	int ret;
};

struct malibu_ufs_vote_regs {
	struct malibu_ufs_psm_regs ap;
	void __iomem *lpcm;
	void __iomem *psm3;
	void __iomem *phy_reset_ack;
};

struct malibu_ufs_vote_sample {
	struct malibu_ufs_psm_sample ap;
	struct malibu_ufs_lpcm_sample psm1;
	struct malibu_ufs_lpcm_sample psm3;
	u32 lpcm_header;
};

static int ufs_malibu_read_lpcm_psm(const struct malibu_ufs_vote_regs *regs, unsigned int psm,
				  unsigned int count, const struct malibu_ufs_psm_sample *ap,
				  struct malibu_ufs_lpcm_sample *sample)
{
	u32 desc;

	if (psm >= count)
		return -ENODATA;

	/* D1 ABL and CPM use the same LPCM status-address calculation. */
	desc = readl(regs->lpcm + MALIBU_LPCM8_PSM_DESC(psm));
	sample->desc = desc;
	sample->addr = MALIBU_LPCM8_BASE + ((desc & 0xf) << 12) +
		       ((desc >> 12) & 0x1f0) + 0x64 * ((desc >> 4) & 0xf) + 0x20;

	/* Reuse sampled AP words. Only the captured D1 PSM3 mapping gets a new read. */
	switch (sample->addr) {
	case MALIBU_HSIOS_PSM_ADDR:
		sample->raw = ap->hsios;
		break;
	case MALIBU_UFS_HC_PSM_ADDR:
		sample->raw = ap->hc;
		break;
	case MALIBU_UFS_PHY_PSM_ADDR:
		sample->raw = ap->phy;
		break;
	case MALIBU_LPCM8_D1_PSM3_STATUS:
		if (psm != MALIBU_LPCM8_PSM3 || desc != MALIBU_LPCM8_D1_PSM3_DESC_VALUE)
			return -ERANGE;
		if (!regs->psm3)
			return -ENXIO;
		sample->raw = readl(regs->psm3);
		break;
	default:
		return -ERANGE;
	}
	return 0;
}

static void ufs_malibu_read_vote_psms(const struct malibu_ufs_vote_regs *regs,
				     struct malibu_ufs_vote_sample *sample)
{
	unsigned int count;

	*sample = (struct malibu_ufs_vote_sample) {
		.psm1.ret = -ENXIO,
		.psm3.ret = -ENXIO,
	};
	if (!regs->ap.hsios)
		return;

	malibu_ufs_sample_psms(&regs->ap, &sample->ap);
	if (!sample->ap.children_valid) {
		sample->psm1.ret = -EAGAIN;
		sample->psm3.ret = -EAGAIN;
		return;
	}
	if (!regs->lpcm)
		return;

	sample->lpcm_header = readl(regs->lpcm);
	count = sample->lpcm_header == U32_MAX ? 0 :
		FIELD_GET(MALIBU_LPCM8_PSM_COUNT, sample->lpcm_header);
	sample->psm1.ret = ufs_malibu_read_lpcm_psm(regs, 1, count, &sample->ap, &sample->psm1);
	sample->psm3.ret = ufs_malibu_read_lpcm_psm(regs, MALIBU_LPCM8_PSM3, count,
						  &sample->ap, &sample->psm3);
}

static void ufs_malibu_read_psms(struct ufs_malibu *ufs,
				 struct malibu_ufs_psm_status *status)
{
	status->hci = readl(ufs->psm_hci);
	status->phy = readl(ufs->phy.psm);
}

static void ufs_malibu_finish_setup_sample(struct ufs_malibu *ufs,
					   struct malibu_ufs_setup_sample *sample,
					   const char *phase, int ret)
{
	ufs_malibu_read_psms(ufs, &sample->after);
	sample->phase = phase;
	sample->ret = ret;
}

static void ufs_malibu_log_setup_psms(struct device *dev,
				      const struct malibu_ufs_setup_sample *samples,
				      unsigned int count)
{
	for (unsigned int i = 0; i < count; i++) {
		const struct malibu_ufs_setup_sample *sample = &samples[i];

		dev_info(dev, "ufs-uic-v36 setup %s PSM before HC=0x%08x PHY=0x%08x\n",
			 sample->phase, sample->before.hci, sample->before.phy);
		dev_info(dev, "ufs-uic-v36 setup %s PSM after HC=0x%08x PHY=0x%08x ret=%d\n",
			 sample->phase, sample->after.hci, sample->after.phy, sample->ret);
	}
}

static bool ufs_malibu_psms_off(struct ufs_malibu *ufs,
				struct malibu_ufs_psm_status *status)
{
	ufs_malibu_read_psms(ufs, status);
	return (status->hci & MALIBU_UFS_PSM_STATE) == MALIBU_UFS_PSM_OFF &&
	       (status->phy & MALIBU_UFS_PSM_STATE) == MALIBU_UFS_PSM_OFF;
}

static void ufs_malibu_format_lpcm_psm(char *buf, size_t size,
				     const struct malibu_ufs_lpcm_sample *psm)
{
	if (psm->ret)
		scnprintf(buf, size, "skipped desc=0x%08x addr=0x%08x ret=%d",
			  psm->desc, psm->addr, psm->ret);
	else
		scnprintf(buf, size, "raw=0x%08x state=0x%x desc=0x%08x addr=0x%08x",
			  psm->raw, psm->raw & MALIBU_LPCM8_PSM_STATE, psm->desc, psm->addr);
}

static void ufs_malibu_log_vote_psm(struct device *dev, const char *domain, const char *when,
				   const struct malibu_ufs_vote_sample *sample, int vote_ret)
{
	char psm1[80], psm3[80];
	const char *psm3_eq_1 = "unknown";

	if (!sample->ap.children_valid) {
		dev_info(dev,
			 "ufs-uic-v36 %s PSM %s HSIOS=0x%08x child PSMs skipped ret=%d PSM3_eq_1=unknown vote_ret=%d\n",
			 domain, when, sample->ap.hsios, sample->psm3.ret, vote_ret);
		return;
	}

	ufs_malibu_format_lpcm_psm(psm1, sizeof(psm1), &sample->psm1);
	ufs_malibu_format_lpcm_psm(psm3, sizeof(psm3), &sample->psm3);
	if (!sample->psm3.ret)
		psm3_eq_1 = (sample->psm3.raw & MALIBU_LPCM8_PSM_STATE) ==
			MALIBU_LPCM8_CORE_OFF_PSM3_STATE ? "yes" : "no";
	dev_info(dev,
		 "ufs-uic-v36 %s PSM %s HC=0x%08x PHY=0x%08x LPCM8 header=0x%08x PSM1 [%s] PSM3 [%s] PSM3_eq_1=%s vote_ret=%d\n",
		 domain, when, sample->ap.hc, sample->ap.phy, sample->lpcm_header,
		 psm1, psm3, psm3_eq_1, vote_ret);
}

static void ufs_malibu_log_vote_psms(struct device *dev, const char *domain,
				     const struct malibu_ufs_vote_sample *before,
				     const struct malibu_ufs_vote_sample *after, int vote_ret)
{
	ufs_malibu_log_vote_psm(dev, domain, "before", before, vote_ret);
	ufs_malibu_log_vote_psm(dev, domain, "after", after, vote_ret);
}

static bool ufs_malibu_log_phy_reset_ack(struct device *dev,
					const struct malibu_ufs_vote_regs *regs)
{
	struct malibu_ufs_psm_sample sample = {};
	u32 raw;

	if (!regs->ap.hsios || !regs->phy_reset_ack) {
		dev_info(dev, "ufs-uic-v36 PHY reset ack skipped: mapping unavailable\n");
		return false;
	}
	malibu_ufs_sample_psms(&regs->ap, &sample);
	if (!sample.children_valid || sample.hc != MALIBU_UFS_PSM_ON ||
	    sample.phy != MALIBU_UFS_PSM_OFF) {
		dev_info(dev, "ufs-uic-v36 PHY reset ack skipped: HSIOS=%#x HC=%#x PHY=%#x\n",
			 sample.hsios, sample.hc, sample.phy);
		return false;
	}

	/* D1 DBL polls this status word when changing UfsPhyReset. */
	dev_info(dev, "ufs-uic-v36 reading PHY reset ack at %#x\n", MALIBU_LPCM8_D1_PHY_RESET_ACK);
	raw = readl(regs->phy_reset_ack);
	if (raw == U32_MAX) {
		dev_info(dev, "ufs-uic-v36 PHY reset ack raw=%#x status=unknown\n", raw);
		return false;
	}
	dev_info(dev, "ufs-uic-v36 PHY reset ack raw=%#x released=%d\n",
		 raw, !!(raw & MALIBU_LPCM8_PHY_RESET_RELEASED));
	return raw & MALIBU_LPCM8_PHY_RESET_RELEASED;
}

static int ufs_malibu_check_device_on_after_timeout(struct ufs_malibu *ufs,
						    struct device *cpm,
						    const struct malibu_ufs_psm_regs *regs)
{
	struct device *dev = ufs->device.dev;
	struct malibu_ufs_psm_sample sample;
	int ret, cleanup;

	malibu_ufs_sample_psms(regs, &sample);
	malibu_ufs_log_psms(dev, "before-device-on", &sample);
	if (!sample.children_valid || sample.hc != MALIBU_UFS_PSM_ON ||
	    sample.phy != MALIBU_UFS_PSM_OFF)
		return -EAGAIN;

	ret = malibu_cpm_ufs_refclk_sample(cpm, MALIBU_CPM_UFS_GPIO_REFCLK_GATE,
					  "before-device-on");
	if (ret)
		return ret;
	ret = malibu_cpm_ufs_refclk_sample(cpm, MALIBU_CPM_UFS_GPIO_REFCLK_ACG,
					  "before-device-on");
	if (ret)
		return ret;
	ret = malibu_cpm_ufs_refclk_sample(cpm, MALIBU_CPM_UFS_MPHY_REFCLK_GATE,
					  "before-device-on");
	if (ret)
		return ret;
	ret = ufs_malibu_device_prepare(&ufs->device);
	dev_info(dev, "ufs-uic-v36 device rail prepare ret=%d\n", ret);
	if (ret)
		return ret;
	ret = ufs_malibu_device_on(&ufs->device, ufs->phy.hs_lss);
	dev_info(dev, "ufs-uic-v36 device-on ret=%d\n", ret);
	if (ret)
		return ret;

	malibu_ufs_sample_psms(regs, &sample);
	malibu_ufs_log_psms(dev, "after-device-on", &sample);
	if (!sample.children_valid)
		return -EAGAIN;

	ret = -EAGAIN;
	if (sample.hc == MALIBU_UFS_PSM_ON)
		ret = ufs_malibu_phy_wait_power(dev, &ufs->phy, "after-device-on", NULL);
	dev_info(dev, "ufs-uic-v36 device-on diagnostic result before device-off: %d\n", ret);
	malibu_cpm_ufs_refclk_sample(cpm, MALIBU_CPM_UFS_GPIO_REFCLK_GATE, "before-device-off");
	cleanup = ufs_malibu_device_off(&ufs->device);
	dev_info(dev, "ufs-uic-v36 diagnostic device-off ret=%d\n", cleanup);
	malibu_cpm_ufs_refclk_sample(cpm, MALIBU_CPM_UFS_GPIO_REFCLK_GATE, "after-device-off");
	return ret ?: cleanup;
}

/* Called before the UFS core stops HCI or releases the IRQ. */
static void ufs_malibu_before_hci_stop(struct ufs_hba *hba, int link, int hibern8)
{
	struct ufs_malibu *ufs = ufshcd_get_variant(hba);
	struct malibu_ufs_psm_status status;
	bool off;
	int ret;

	dev_info(hba->dev, "ufs-uic-v36 probe result before HCI stop: link=%d hibern8=%d\n",
		 link, hibern8);
	if (link || hibern8)
		return;

	ret = read_poll_timeout(ufs_malibu_psms_off, off, off, 5, 10000, false,
				ufs, &status);
	ufs->deep_h8_release_result = ret;
	dev_info(hba->dev, "ufs-uic-v36 Deep H8 gate: HC=%#x PHY=%#x ret=%d\n",
		 status.hci, status.phy, ret);
	if (ret)
		return;

	/* Keep the core clock, domains and device powered for this LPCM request. */
	clk_unprepare(ufs->deep_h8);
	dev_info(hba->dev, "ufs-uic-v36 Deep H8 unprepare returned\n");
}

static int ufs_malibu_run_probe(struct platform_device *pdev, bool *retain)
{
	static const char * const domain_names[] = { "prep", "core" };
	const struct dev_pm_domain_attach_data attach = {
		.pd_names = domain_names,
		.num_pd_names = ARRAY_SIZE(domain_names),
		.pd_flags = PD_FLAG_NO_DEV_LINK,
	};
	struct device *dev = &pdev->dev;
	struct dev_pm_domain_list *domains;
	struct device *cpm, *prep, *core;
	struct ufs_hba *hba;
	struct ufs_malibu *ufs;
	struct clk *mphy, *core_clk;
	struct malibu_ufs_vote_sample before_prep, after_prep;
	struct malibu_ufs_vote_sample before_core, after_core;
	struct malibu_ufs_vote_regs vote_regs = {};
	struct malibu_ufs_setup_sample setup[4];
	unsigned int setup_count = 0;
	void __iomem *hci;
	const char *phase;
	bool phy_timed_out = false, phy_reset_released = false;
	u32 status;
	int irq, ret, core_ret, prep_ret;

	irq = ufs_malibu_get_hci_irq(pdev);
	if (irq < 0)
		return dev_err_probe(dev, irq, "invalid or unavailable UFS IRQ route\n");
	ret = ufs_malibu_check_dma(dev);
	if (ret)
		return dev_err_probe(dev, ret, "AMB DMA preflight failed\n");

	ufs = devm_kzalloc(dev, sizeof(*ufs), GFP_KERNEL);
	if (!ufs)
		return -ENOMEM;
	ufs->device.dev = dev;
	ret = ufs_malibu_phy_read_config(dev, &ufs->phy);
	if (ret)
		return ret;
	ret = ufs_malibu_get_supplies(&ufs->device);
	if (ret)
		return ret;

	hci = ufs_malibu_map(pdev, "hci", 0x3c2d0000, 0x3000);
	if (IS_ERR(hci))
		return PTR_ERR(hci);
	ufs->phy.top = ufs_malibu_map(pdev, "top", 0x3c2e0000, 0x718);
	if (IS_ERR(ufs->phy.top))
		return PTR_ERR(ufs->phy.top);
	ufs->phy.subsystem = ufs_malibu_map(pdev, "subsystem", 0x3c34a000, 0x18);
	if (IS_ERR(ufs->phy.subsystem))
		return PTR_ERR(ufs->phy.subsystem);
	ufs->phy.psm = ufs_malibu_map(pdev, "psm-phy", MALIBU_UFS_PHY_PSM_ADDR, sizeof(u32));
	if (IS_ERR(ufs->phy.psm))
		return PTR_ERR(ufs->phy.psm);
	ufs->psm_hci = ufs_malibu_map(pdev, "psm-hci", MALIBU_UFS_HC_PSM_ADDR, sizeof(u32));
	if (IS_ERR(ufs->psm_hci))
		return PTR_ERR(ufs->psm_hci);
	if (of_machine_is_compatible("google,malibu-grizzly")) {
		vote_regs.ap.hsios = malibu_ufs_map_psm(dev, dev->of_node, "psm-hsios",
						      MALIBU_HSIOS_PSM_ADDR);
		vote_regs.ap.hc = ufs->psm_hci;
		vote_regs.ap.phy = ufs->phy.psm;
		vote_regs.lpcm = devm_ioremap(dev, MALIBU_LPCM8_BASE,
					    MALIBU_LPCM8_PSM_DESC(MALIBU_LPCM8_PSM3) + sizeof(u32));
		vote_regs.psm3 = devm_ioremap(dev, MALIBU_LPCM8_D1_PSM3_STATUS, sizeof(u32));
		vote_regs.phy_reset_ack = devm_ioremap(dev, MALIBU_LPCM8_D1_PHY_RESET_ACK,
						     sizeof(u32));
	}
	ufs->phy.reset = devm_reset_control_get_exclusive(dev, "phy");
	if (IS_ERR(ufs->phy.reset))
		return PTR_ERR(ufs->phy.reset);

	mphy = devm_clk_get(dev, "mphy-ref");
	if (IS_ERR(mphy))
		return PTR_ERR(mphy);
	if (clk_get_rate(mphy) != 38400000)
		return -EINVAL;

	ufs->deep_h8 = devm_clk_get(dev, "deep-h8");
	if (IS_ERR(ufs->deep_h8))
		return PTR_ERR(ufs->deep_h8);
	core_clk = devm_clk_get(dev, "core");
	if (IS_ERR(core_clk))
		return PTR_ERR(core_clk);
	ret = devm_pm_domain_attach_list(dev, &attach, &domains);
	if (ret != ARRAY_SIZE(domain_names))
		return ret < 0 ? ret : -ENODEV;
	prep = domains->pd_devs[0];
	core = domains->pd_devs[1];

	cpm = ufs_malibu_get_cpm(dev);
	if (IS_ERR(cpm))
		return PTR_ERR(cpm);
	ret = ufshcd_alloc_host(dev, &hba);
	if (ret)
		goto put_cpm;
	hba->vops = &ufs_malibu_vops;
	hba->lanes_per_direction = 2;
	hba->quirks |= UFSHCD_QUIRK_READ_ONLY_REF_CLK;
	ufshcd_parse_dev_ref_clk_freq(hba, mphy);
	ufshcd_set_variant(hba, ufs);

	/* The HCI interrupt aggregator holds the HSIO-S parent domain. */
	ufs_malibu_read_psms(ufs, &setup[setup_count].before);
	ret = ufs_malibu_get_gpios(&ufs->device);
	if (ret) {
		ufs_malibu_finish_setup_sample(ufs, &setup[setup_count++], "gpio-setup", ret);
		dev_info(dev, "ufs-uic-v36 early GPIO acquisition ret=%d\n", ret);
		goto put_cpm;
	}
	ret = ufs_malibu_device_init_gpios(&ufs->device);
	ufs_malibu_finish_setup_sample(ufs, &setup[setup_count++], "gpio-setup", ret);
	dev_info(dev, "ufs-uic-v36 early GPIO output-low ret=%d\n", ret);
	if (ret)
		goto put_cpm;

	/*
	 * CPM gates these clocks in prepare/unprepare. Bulk prepare would
	 * unprepare Deep H8 on a core error without qualifying Hibern8.
	 */
	*retain = true;
	phase = "deep-h8-prepare";
	ufs_malibu_read_psms(ufs, &setup[setup_count].before);
	ret = clk_prepare(ufs->deep_h8);
	ufs_malibu_finish_setup_sample(ufs, &setup[setup_count++], phase, ret);
	if (ret)
		goto hold;
	phase = "core-clock-prepare";
	ufs_malibu_read_psms(ufs, &setup[setup_count].before);
	ret = clk_prepare(core_clk);
	ufs_malibu_finish_setup_sample(ufs, &setup[setup_count++], phase, ret);
	if (ret)
		goto hold;
	phase = "pmic-decouple";
	ufs_malibu_read_psms(ufs, &setup[setup_count].before);
	ret = malibu_cpm_ufs_decouple_enable(cpm);
	ufs_malibu_finish_setup_sample(ufs, &setup[setup_count++], phase, ret);
	dev_info(dev, "ufs-uic-v36 early PMIC decouple ret=%d\n", ret);
	if (ret)
		goto hold;
	phase = "prep-domain";
	ufs_malibu_read_vote_psms(&vote_regs, &before_prep);
	prep_ret = pm_runtime_resume_and_get(prep);
	ufs_malibu_read_vote_psms(&vote_regs, &after_prep);
	ret = prep_ret;
	if (ret < 0)
		goto log_prep_failure;
	/* Sample before TOP programming so the reads do not delay its core vote. */
	malibu_cpm_ufs_log_rails(cpm, "before-core");
	phase = "phy-prepare";
	ret = ufs_malibu_phy_prepare(&ufs->phy);
	if (ret)
		goto log_prep_failure;
	phase = "core-domain";
	ufs_malibu_read_vote_psms(&vote_regs, &before_core);
	core_ret = pm_runtime_resume_and_get(core);
	ufs_malibu_read_vote_psms(&vote_regs, &after_core);
	ret = core_ret;
	if (ret >= 0) {
		phase = "phy-power-ready";
		ret = ufs_malibu_phy_wait_power(dev, &ufs->phy, phase, &phy_timed_out);
	}
	/* Print the saved samples after the first readiness check. */
	ufs_malibu_log_setup_psms(dev, setup, setup_count);
	setup_count = 0;
	ufs_malibu_log_vote_psms(dev, "prep", &before_prep, &after_prep, prep_ret);
	ufs_malibu_log_vote_psms(dev, "core", &before_core, &after_core, core_ret);
	if (core_ret >= 0 && ret == -ETIMEDOUT)
		phy_reset_released = ufs_malibu_log_phy_reset_ack(dev, &vote_regs);
	/* Keep the readiness poll ahead of the post-core mailbox reads. */
	malibu_cpm_ufs_log_rails(cpm, core_ret < 0 ? "after-core-error" : "after-core-poll");
	if (phy_timed_out && phy_reset_released) {
		phase = "device-on-after-phy-timeout";
		ret = ufs_malibu_check_device_on_after_timeout(ufs, cpm, &vote_regs.ap);
		/* This fallback ends before HCI access, even when the PHY responds. */
		goto hold;
	}
	if (ret)
		goto hold;
	phase = "hci-idle";
	ret = ufs_malibu_check_idle(hci);
	if (ret)
		goto hold;
	phase = "phy-refclk";
	ret = ufs_malibu_phy_post_power_on(dev, &ufs->phy, 0);
	if (ret)
		goto hold;
	phase = "device-rails";
	ret = ufs_malibu_device_prepare(&ufs->device);
	if (ret)
		goto hold;
	phase = "device-power-on";
	ret = ufs_malibu_device_on(&ufs->device, ufs->phy.hs_lss);
	if (ret)
		goto hold;

	phase = "link-hibern8";
	ufs->deep_h8_release_result = -ECANCELED;
	ret = ufshcd_probe_link_hibern8(hba, hci, irq, ufs_malibu_before_hci_stop);
	if (ret)
		goto hold;
	phase = "deep-h8-qualification";
	ret = ufs->deep_h8_release_result;
	if (ret)
		goto hold;
	phase = "hci-stop";
	ret = readl_poll_timeout(hci + REG_CONTROLLER_ENABLE, status,
				 !(status & CONTROLLER_ENABLE), 10, 1000);
	if (ret)
		goto hold;
	phase = "device-power-off";
	ret = ufs_malibu_device_off(&ufs->device);
	if (ret)
		goto hold;
	ufs_malibu_phy_restore_refclk(&ufs->phy);
	clk_unprepare(core_clk);
	ret = pm_runtime_put_sync_suspend(core);
	prep_ret = pm_runtime_put_sync_suspend(prep);
	if (ret >= 0)
		ret = min(prep_ret, 0);
	*retain = false;
	goto put_cpm;

log_prep_failure:
	ufs_malibu_log_vote_psms(dev, "prep", &before_prep, &after_prep, prep_ret);
hold:
	dev_info(dev, "ufs-uic-v36 probe phase=%s ret=%d; retaining remaining UFS votes\n",
		 phase, ret);
put_cpm:
	ufs_malibu_log_setup_psms(dev, setup, setup_count);
	put_device(cpm);
	return ret;
}

static int ufs_malibu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	void *group;
	bool retain = false;
	int ret;

	if (!of_machine_is_compatible("google,malibu-grizzly"))
		return -ENODEV;

	group = devres_open_group(dev, NULL, GFP_KERNEL);
	if (!group)
		return -ENOMEM;

	ret = ufs_malibu_run_probe(pdev, &retain);
	dev_set_drvdata(dev, NULL);
	if (retain) {
		/* Leave the diagnostic bound and hold the resources for this boot. */
		devres_remove_group(dev, group);
		return 0;
	}
	dev_info(dev, "ufs-uic-v36 probe result before devres release: %d\n", ret);
	devres_release_group(dev, group);
	if (ret)
		return dev_err_probe(dev, ret, "UFS link diagnostic failed\n");

	dev_info(dev, "ufs-uic-v36 UFS link, HCI IRQ and Hibern8 verified; release sequence finished\n");
	return 0;
}

static const struct of_device_id ufs_malibu_of_match[] = {
	{ .compatible = "google,malibu-ufshc" },
	{ }
};
MODULE_DEVICE_TABLE(of, ufs_malibu_of_match);

static struct platform_driver ufs_malibu_driver = {
	.probe = ufs_malibu_probe,
	.driver = {
		.name = "ufs-malibu",
		.of_match_table = ufs_malibu_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(ufs_malibu_driver);
