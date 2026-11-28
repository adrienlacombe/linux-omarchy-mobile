// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/limits.h>
#include <linux/malibu-ufs-psm.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/spinlock.h>

#define MALIBU_SGIA_NUM_IRQS		32
#define MALIBU_SGIA_STATUS		0x00
#define MALIBU_SGIA_PULSE_STATUS		0x04
#define MALIBU_SGIA_CLEAR		0x08
#define MALIBU_SGIA_ENABLE		0x0c
#define MALIBU_SGIA_UFS_BASE		0x3c272040

struct malibu_sgia {
	void __iomem *base;
	struct irq_domain *domain;
	raw_spinlock_t lock;
	u32 enabled;
	int parent_irq;
	bool pulse;
};

static void malibu_sgia_set_enabled(struct irq_data *data, bool enable)
{
	struct malibu_sgia *sgia = irq_data_get_irq_chip_data(data);
	unsigned long flags;
	u32 bit = BIT(data->hwirq);

	raw_spin_lock_irqsave(&sgia->lock, flags);
	if (enable)
		sgia->enabled |= bit;
	else
		sgia->enabled &= ~bit;
	writel(sgia->enabled, sgia->base + MALIBU_SGIA_ENABLE);
	raw_spin_unlock_irqrestore(&sgia->lock, flags);
}

static void malibu_sgia_mask(struct irq_data *data)
{
	malibu_sgia_set_enabled(data, false);
}

static void malibu_sgia_unmask(struct irq_data *data)
{
	malibu_sgia_set_enabled(data, true);
}

static void malibu_sgia_eoi(struct irq_data *data)
{
	/* Level interrupts are cleared at their source. Pulses are cleared on entry. */
}

static int malibu_sgia_get_state(struct irq_data *data,
				enum irqchip_irq_state which, bool *state)
{
	struct malibu_sgia *sgia = irq_data_get_irq_chip_data(data);
	unsigned long flags;
	u32 enabled;

	if (which != IRQCHIP_STATE_MASKED)
		return -EINVAL;

	/* The SGIA holds its runtime PM reference for its IRQ domain's lifetime. */
	raw_spin_lock_irqsave(&sgia->lock, flags);
	enabled = readl(sgia->base + MALIBU_SGIA_ENABLE) | sgia->enabled;
	raw_spin_unlock_irqrestore(&sgia->lock, flags);
	*state = !(enabled & BIT(data->hwirq));
	return 0;
}

static struct irq_chip malibu_sgia_chip = {
	.name = "malibu-sgia",
	.irq_mask = malibu_sgia_mask,
	.irq_unmask = malibu_sgia_unmask,
	.irq_eoi = malibu_sgia_eoi,
	.irq_get_irqchip_state = malibu_sgia_get_state,
	.flags = IRQCHIP_MASK_ON_SUSPEND,
};

static int malibu_sgia_map(struct irq_domain *domain, unsigned int irq,
			   irq_hw_number_t hwirq)
{
	struct malibu_sgia *sgia = domain->host_data;

	irq_set_chip_data(irq, sgia);
	irq_set_chip_and_handler(irq, &malibu_sgia_chip, handle_fasteoi_irq);
	irq_set_status_flags(irq, IRQ_NOPROBE | (sgia->pulse ? 0 : IRQ_LEVEL));
	return 0;
}

static void malibu_sgia_unmap(struct irq_domain *domain, unsigned int irq)
{
	irq_set_chip_and_handler(irq, NULL, NULL);
	irq_set_chip_data(irq, NULL);
}

static const struct irq_domain_ops malibu_sgia_domain_ops = {
	.map = malibu_sgia_map,
	.unmap = malibu_sgia_unmap,
	.xlate = irq_domain_xlate_onecell,
};

static void malibu_sgia_dispatch(struct malibu_sgia *sgia, u32 offset)
{
	void __iomem *status = sgia->base + offset;
	unsigned long pending;
	unsigned int bit;

	raw_spin_lock(&sgia->lock);
	pending = readl(status) & sgia->enabled;
	if (sgia->pulse && pending)
		writel(pending, status);
	raw_spin_unlock(&sgia->lock);

	for_each_set_bit(bit, &pending, MALIBU_SGIA_NUM_IRQS)
		generic_handle_domain_irq(sgia->domain, bit);
}

static void malibu_sgia_chained_irq(struct irq_desc *desc)
{
	struct irq_chip *parent = irq_desc_get_chip(desc);
	struct malibu_sgia *sgia = irq_desc_get_handler_data(desc);

	chained_irq_enter(parent, desc);
	malibu_sgia_dispatch(sgia, MALIBU_SGIA_STATUS);
	if (sgia->pulse)
		malibu_sgia_dispatch(sgia, MALIBU_SGIA_PULSE_STATUS);
	chained_irq_exit(parent, desc);
}

static void malibu_sgia_restore(struct malibu_sgia *sgia)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&sgia->lock, flags);
	writel(U32_MAX, sgia->base + MALIBU_SGIA_CLEAR);
	writel(sgia->enabled, sgia->base + MALIBU_SGIA_ENABLE);
	raw_spin_unlock_irqrestore(&sgia->lock, flags);
}

static int malibu_sgia_suspend_noirq(struct device *dev)
{
	struct malibu_sgia *sgia = dev_get_drvdata(dev);

	disable_irq_nosync(sgia->parent_irq);
	synchronize_hardirq(sgia->parent_irq);
	return 0;
}

static int malibu_sgia_resume_noirq(struct device *dev)
{
	struct malibu_sgia *sgia = dev_get_drvdata(dev);

	malibu_sgia_restore(sgia);
	enable_irq(sgia->parent_irq);
	return 0;
}

static const struct dev_pm_ops malibu_sgia_pm_ops = {
	NOIRQ_SYSTEM_SLEEP_PM_OPS(malibu_sgia_suspend_noirq, malibu_sgia_resume_noirq)
};

static int malibu_sgia_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct malibu_ufs_psm_sample before, after;
	struct malibu_ufs_psm_regs ufs_psm = {};
	struct malibu_sgia *sgia;
	struct resource *res;
	bool trace_ufs;
	int parent_irq;
	int ret;

	parent_irq = platform_get_irq(pdev, 0);
	if (parent_irq < 0)
		return parent_irq;

	sgia = devm_kzalloc(dev, sizeof(*sgia), GFP_KERNEL);
	if (!sgia)
		return -ENOMEM;
	sgia->parent_irq = parent_irq;

	sgia->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(sgia->base))
		return PTR_ERR(sgia->base);
	if (resource_size(res) < MALIBU_SGIA_ENABLE + sizeof(u32))
		return dev_err_probe(dev, -EINVAL, "interrupt range is too small\n");

	sgia->pulse = of_device_is_compatible(dev->of_node, "google,malibu-pulse-sgia");
	if (of_property_present(dev->of_node, "power-domains") && !dev->pm_domain)
		return dev_err_probe(dev, -ENODEV, "missing interrupt aggregator power domain\n");

	trace_ufs = res->start == MALIBU_SGIA_UFS_BASE && malibu_ufs_map_psms(dev, &ufs_psm);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;
	/* The chained handler accesses registers from hard IRQ context. */
	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to power interrupt aggregator\n");
	if (trace_ufs)
		malibu_ufs_sample_psms(&ufs_psm, &before);

	raw_spin_lock_init(&sgia->lock);
	malibu_sgia_restore(sgia);
	sgia->domain = irq_domain_create_linear(dev_fwnode(dev), MALIBU_SGIA_NUM_IRQS,
						&malibu_sgia_domain_ops, sgia);
	if (!sgia->domain) {
		if (trace_ufs)
			malibu_ufs_log_psms(dev, "sgia-before-irq-setup", &before);
		pm_runtime_put_sync(dev);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, sgia);
	irq_set_status_flags(parent_irq, IRQ_DISABLE_UNLAZY);
	irq_set_chained_handler_and_data(parent_irq, malibu_sgia_chained_irq, sgia);
	if (trace_ufs) {
		malibu_ufs_sample_psms(&ufs_psm, &after);
		malibu_ufs_log_psms(dev, "sgia-before-irq-setup", &before);
		malibu_ufs_log_psms(dev, "sgia-after-irq-setup", &after);
	}
	return 0;
}

static void malibu_sgia_remove(struct platform_device *pdev)
{
	struct malibu_sgia *sgia = platform_get_drvdata(pdev);
	unsigned int bit;

	disable_irq(sgia->parent_irq);
	irq_set_chained_handler_and_data(sgia->parent_irq, NULL, NULL);
	irq_clear_status_flags(sgia->parent_irq, IRQ_DISABLE_UNLAZY);

	for (bit = 0; bit < MALIBU_SGIA_NUM_IRQS; bit++) {
		unsigned int irq = irq_find_mapping(sgia->domain, bit);

		if (irq)
			irq_dispose_mapping(irq);
	}
	irq_domain_remove(sgia->domain);
	writel(0, sgia->base + MALIBU_SGIA_ENABLE);
	pm_runtime_put_sync(&pdev->dev);
}

static const struct of_device_id malibu_sgia_of_match[] = {
	{ .compatible = "google,malibu-level-sgia" },
	{ .compatible = "google,malibu-pulse-sgia" },
	{ }
};

static struct platform_driver malibu_sgia_driver = {
	.probe = malibu_sgia_probe,
	.remove = malibu_sgia_remove,
	.driver = {
		.name = "google-malibu-sgia",
		.of_match_table = malibu_sgia_of_match,
		.pm = pm_sleep_ptr(&malibu_sgia_pm_ops),
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(malibu_sgia_driver);
