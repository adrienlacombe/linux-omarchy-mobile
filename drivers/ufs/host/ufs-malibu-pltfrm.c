// SPDX-License-Identifier: GPL-2.0-only

#include <dt-bindings/clock/google,malibu-cpm-io.h>
#include <linux/clk.h>
#include <linux/dma-map-ops.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/iopoll.h>
#include <linux/iommu.h>
#include <linux/iommu-dma.h>
#include <linux/irq.h>
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

static int ufs_malibu_get_gpios(struct ufs_malibu *ufs)
{
	struct ufs_malibu_device *device = &ufs->device;
	struct device *dev = device->dev;

	device->reset = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(device->reset))
		return PTR_ERR(device->reset);
	device->power = devm_gpiod_get(dev, "power", GPIOD_ASIS);
	if (IS_ERR(device->power))
		return PTR_ERR(device->power);
	if (!ufs->phy.hs_lss)
		return 0;

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

static int ufs_malibu_run_probe(struct platform_device *pdev)
{
	static const char * const domain_names[] = { "prep", "core" };
	const struct dev_pm_domain_attach_data attach = {
		.pd_names = domain_names,
		.num_pd_names = ARRAY_SIZE(domain_names),
		.pd_flags = PD_FLAG_NO_DEV_LINK,
	};
	struct clk_bulk_data clocks[] = { { .id = "deep-h8" }, { .id = "core" } };
	struct device *dev = &pdev->dev;
	struct dev_pm_domain_list *domains;
	struct device *cpm, *prep, *core;
	struct ufs_hba *hba;
	struct ufs_malibu *ufs;
	struct clk *mphy;
	void __iomem *hci;
	u32 status;
	int irq, ret, cleanup, i;

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
	ufs->phy.psm = ufs_malibu_map(pdev, "psm-phy", 0x3c00d108, 4);
	if (IS_ERR(ufs->phy.psm))
		return PTR_ERR(ufs->phy.psm);
	ufs->phy.reset = devm_reset_control_get_exclusive(dev, "phy");
	if (IS_ERR(ufs->phy.reset))
		return PTR_ERR(ufs->phy.reset);

	mphy = devm_clk_get(dev, "mphy-ref");
	if (IS_ERR(mphy))
		return PTR_ERR(mphy);
	if (clk_get_rate(mphy) != 38400000)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(clocks); i++) {
		clocks[i].clk = devm_clk_get(dev, clocks[i].id);
		if (IS_ERR(clocks[i].clk))
			return PTR_ERR(clocks[i].clk);
	}
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
	ufshcd_set_variant(hba, ufs);

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(clocks), clocks);
	if (ret)
		goto put_cpm;
	ret = pm_runtime_resume_and_get(prep);
	if (ret < 0)
		goto disable_clocks;
	ret = ufs_malibu_phy_prepare(&ufs->phy);
	if (ret)
		goto put_prep;
	ret = pm_runtime_resume_and_get(core);
	if (ret < 0)
		goto put_prep;
	ret = ufs_malibu_phy_wait_power(&ufs->phy);
	if (ret)
		goto put_core;
	ret = ufs_malibu_check_idle(hci);
	if (ret)
		goto put_core;
	ret = ufs_malibu_phy_post_power_on(&ufs->phy, 0);
	if (ret)
		goto put_core;
	ret = ufs_malibu_get_gpios(ufs);
	if (ret)
		goto put_core;
	ret = ufs_malibu_device_prepare(&ufs->device, cpm);
	if (ret)
		goto put_core;
	ret = ufs_malibu_device_on(&ufs->device, ufs->phy.hs_lss);
	if (ret)
		goto put_core;

	ret = ufshcd_probe_link(hba, hci, irq);
	cleanup = readl_poll_timeout(hci + REG_CONTROLLER_ENABLE, status,
				     !(status & CONTROLLER_ENABLE), 10, 1000);
	ret = ret ?: cleanup;
	cleanup = ufs_malibu_device_off(&ufs->device);
	ret = ret ?: cleanup;
put_core:
	ufs_malibu_phy_restore_refclk(&ufs->phy);
	cleanup = pm_runtime_put_sync_suspend(core);
	if (cleanup < 0)
		ret = ret ?: cleanup;
put_prep:
	cleanup = pm_runtime_put_sync_suspend(prep);
	if (cleanup < 0)
		ret = ret ?: cleanup;
disable_clocks:
	clk_bulk_disable_unprepare(ARRAY_SIZE(clocks), clocks);
put_cpm:
	put_device(cpm);
	return ret;
}

static int ufs_malibu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	void *group;
	int ret;

	if (!of_machine_is_compatible("google,malibu-grizzly"))
		return -ENODEV;

	group = devres_open_group(dev, NULL, GFP_KERNEL);
	if (!group)
		return -ENOMEM;

	ret = ufs_malibu_run_probe(pdev);
	dev_set_drvdata(dev, NULL);
	devres_release_group(dev, group);
	if (ret)
		return dev_err_probe(dev, ret, "UFS link diagnostic failed\n");

	dev_info(dev, "UFS link and HCI interrupt verified; controller released\n");
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
	},
};
module_platform_driver(ufs_malibu_driver);
