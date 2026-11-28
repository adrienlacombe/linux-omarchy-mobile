// SPDX-License-Identifier: GPL-2.0-only

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

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
	return 0;
}

static const struct of_device_id malibu_dc9x00_of_match[] = {
	{ .compatible = "google,malibu-dc9x00" },
	{ }
};
MODULE_DEVICE_TABLE(of, malibu_dc9x00_of_match);

static struct platform_driver malibu_dc9x00_driver = {
	.probe = malibu_dc9x00_probe,
	.driver = {
		.name = "malibu-dc9x00",
		.of_match_table = malibu_dc9x00_of_match,
	},
};
module_platform_driver(malibu_dc9x00_driver);

MODULE_DESCRIPTION("Google Malibu DC9x00 frame-start IRQ scaffold");
MODULE_LICENSE("GPL");
