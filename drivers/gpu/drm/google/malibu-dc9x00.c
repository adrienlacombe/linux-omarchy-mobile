// SPDX-License-Identifier: GPL-2.0-only

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/malibu-cpm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/timekeeping.h>

#include <dt-bindings/clock/google,malibu-cpm-io.h>

#define MALIBU_DC_BE_MUX	0x0f6d4004
#define MALIBU_BE_SGIA	0x0f46e020

static bool sample_be_csr;
module_param(sample_be_csr, bool, 0400);
MODULE_PARM_DESC(sample_be_csr, "Read the Grizzly BE output mux once with guarded AP votes");

static int malibu_dc9x00_sample_be_csr(struct platform_device *pdev, int irq)
{
	struct irq_data *irq_data = irq_get_irq_data(irq);
	struct device *dev = &pdev->dev;
	struct device_node *router;
	struct platform_device *supplier;
	struct of_phandle_args domain;
	struct resource *res;
	void __iomem *mux;
	bool masked;
	u32 value;
	int ret;

	if (!of_machine_is_compatible("google,malibu-grizzly") || !dev->pm_domain)
		return -ENODEV;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || res->start != 0x0f600000 || res->end < MALIBU_DC_BE_MUX + 3)
		return -EINVAL;
	if (!irq_data || irqd_to_hwirq(irq_data) != 4 || !irq_data->domain)
		return -EINVAL;
	router = irq_domain_get_of_node(irq_data->domain);
	if (!of_device_is_compatible(router, "google,malibu-level-sgia"))
		return -EINVAL;
	supplier = of_find_device_by_node(router);
	if (!supplier)
		return -ENODEV;
	res = platform_get_resource(supplier, IORESOURCE_MEM, 0);
	ret = res && res->start == MALIBU_BE_SGIA ? 0 : -EINVAL;
	put_device(&supplier->dev);
	if (ret)
		return ret;

	ret = of_parse_phandle_with_args(dev->of_node, "power-domains", "#power-domain-cells",
					 0, &domain);
	if (ret)
		return ret;
	supplier = NULL;
	if (domain.args_count == 1 && domain.args[0] == MALIBU_CPM_PD_DPU_BE)
		supplier = of_find_device_by_node(domain.np);
	of_node_put(domain.np);
	if (!supplier)
		return -ENODEV;

	mux = devm_ioremap(dev, MALIBU_DC_BE_MUX, sizeof(value));
	if (!mux) {
		ret = -ENOMEM;
		goto put_supplier;
	}

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		goto put_supplier;
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		goto put_supplier;
	ret = malibu_cpm_check_be_csr(&supplier->dev);
	if (ret)
		goto put_pm;

	ret = irq_get_irqchip_state(irq, IRQCHIP_STATE_MASKED, &masked);
	if (ret || !masked) {
		if (!ret)
			ret = -EBUSY;
		goto put_pm;
	}

	dev_info(dev, "BE CSR before_read addr=0x%08x frame-start irq=%d masked\n",
		 MALIBU_DC_BE_MUX, irq);
	value = readl(mux);
	dev_info(dev, "BE CSR sampled raw=0x%08x mono_ns=%llu\n", value, ktime_get_mono_fast_ns());

	/* Keep BE active across the driver core's runtime PM get/put at unbind. */
	goto put_supplier;

put_pm:
	pm_runtime_put_sync(dev);
put_supplier:
	put_device(&supplier->dev);
	return ret;
}

static irqreturn_t malibu_dc9x00_frame_start(int irq, void *data)
{
	/* The BE status and acknowledgment path is not qualified yet. */
	return IRQ_NONE;
}

static int malibu_dc9x00_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int irq, ret;

	irq = platform_get_irq_byname(pdev, "frame-start");
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, malibu_dc9x00_frame_start,
			       IRQF_NO_AUTOEN, dev_name(dev), dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to reserve frame-start IRQ\n");

	dev_info(dev, "frame-start IRQ %d reserved and masked\n", irq);
	if (sample_be_csr) {
		ret = malibu_dc9x00_sample_be_csr(pdev, irq);
		if (ret)
			return dev_err_probe(dev, ret, "BE CSR guard failed\n");
	}

	return 0;
}

static void malibu_dc9x00_remove(struct platform_device *pdev)
{
	if (sample_be_csr)
		pm_runtime_put_sync(&pdev->dev);
}

static const struct of_device_id malibu_dc9x00_of_match[] = {
	{ .compatible = "google,malibu-dc9x00" },
	{ }
};
MODULE_DEVICE_TABLE(of, malibu_dc9x00_of_match);

static struct platform_driver malibu_dc9x00_driver = {
	.probe = malibu_dc9x00_probe,
	.remove = malibu_dc9x00_remove,
	.driver = {
		.name = "malibu-dc9x00",
		.of_match_table = malibu_dc9x00_of_match,
	},
};
module_platform_driver(malibu_dc9x00_driver);

MODULE_DESCRIPTION("Google Malibu DC9x00 frame-start IRQ scaffold");
MODULE_LICENSE("GPL");
