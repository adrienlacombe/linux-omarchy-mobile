// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitmap.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdesc.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rcupdate.h>
#include <linux/spinlock.h>

#define MALIBU_AOSS_NGPIO		123
#define MALIBU_AOSS_ALERT_PAD		0x61
#define MALIBU_AOSS_PAD_STRIDE		0x1000
#define MALIBU_AOSS_ALERT_OFFSET	(MALIBU_AOSS_ALERT_PAD * MALIBU_AOSS_PAD_STRIDE)

#define MALIBU_AOSS_PARAM		0x00
#define MALIBU_AOSS_DMUX		0x04
#define MALIBU_AOSS_TXDATA		0x08
#define MALIBU_AOSS_RXDATA		0x1c
#define MALIBU_AOSS_ISR			0x20
#define MALIBU_AOSS_ISROVF		0x24
#define MALIBU_AOSS_IER			0x28
#define MALIBU_AOSS_IMR			0x2c

#define MALIBU_AOSS_PARAM_INPUT		BIT(8)
#define MALIBU_AOSS_DMUX_FUNC		GENMASK(8, 0)
#define MALIBU_AOSS_DMUX_GPIO		BIT(0)
#define MALIBU_AOSS_TX_PULL		GENMASK(1, 0)
#define MALIBU_AOSS_TX_PULL_UP		BIT(0)
#define MALIBU_AOSS_TX_OUTPUT		BIT(3)
#define MALIBU_AOSS_RX_VALUE		BIT(0)
#define MALIBU_AOSS_RX_TRIGGER		GENMASK(3, 1)
#define MALIBU_AOSS_RX_LEVEL_LOW	BIT(3)

struct malibu_aoss_alert {
	struct gpio_chip gc;
	void __iomem *pad;
	raw_spinlock_t lock;
	unsigned int parent_irq;
	u32 param;
	u32 dmux;
	u32 tx;
	u32 rx;
	u32 ier;
	u32 imr;
	bool stopping;
};

static void malibu_aoss_update(struct malibu_aoss_alert *alert, unsigned int reg,
			       u32 mask, u32 value)
{
	u32 old = readl(alert->pad + reg);

	writel((old & ~mask) | (value & mask), alert->pad + reg);
}

static int malibu_aoss_init_valid_mask(struct gpio_chip *gc,
				       unsigned long *valid_mask, unsigned int ngpio)
{
	if (!test_bit(MALIBU_AOSS_ALERT_PAD, valid_mask))
		return -EINVAL;
	bitmap_zero(valid_mask, ngpio);
	__set_bit(MALIBU_AOSS_ALERT_PAD, valid_mask);
	return 0;
}

static int malibu_aoss_get(struct gpio_chip *gc, unsigned int line)
{
	struct malibu_aoss_alert *alert = gpiochip_get_data(gc);

	return !!(readl(alert->pad + MALIBU_AOSS_RXDATA) & MALIBU_AOSS_RX_VALUE);
}

static int malibu_aoss_get_direction(struct gpio_chip *gc, unsigned int line)
{
	struct malibu_aoss_alert *alert = gpiochip_get_data(gc);

	if (readl(alert->pad + MALIBU_AOSS_TXDATA) & MALIBU_AOSS_TX_OUTPUT)
		return GPIO_LINE_DIRECTION_OUT;
	return GPIO_LINE_DIRECTION_IN;
}

static int malibu_aoss_direction_input(struct gpio_chip *gc, unsigned int line)
{
	struct malibu_aoss_alert *alert = gpiochip_get_data(gc);
	unsigned long flags;

	raw_spin_lock_irqsave(&alert->lock, flags);
	malibu_aoss_update(alert, MALIBU_AOSS_TXDATA, MALIBU_AOSS_TX_OUTPUT, 0);
	malibu_aoss_update(alert, MALIBU_AOSS_PARAM, MALIBU_AOSS_PARAM_INPUT,
			   MALIBU_AOSS_PARAM_INPUT);
	raw_spin_unlock_irqrestore(&alert->lock, flags);
	return 0;
}

static int malibu_aoss_irq_set_type(struct irq_data *data, unsigned int type)
{
	return type == IRQ_TYPE_LEVEL_LOW ? 0 : -EINVAL;
}

static void malibu_aoss_irq_enable(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct malibu_aoss_alert *alert = gpiochip_get_data(gc);
	unsigned long flags;

	gpiochip_enable_irq(gc, irqd_to_hwirq(data));
	raw_spin_lock_irqsave(&alert->lock, flags);
	if (!alert->stopping) {
		writel(1, alert->pad + MALIBU_AOSS_IER);
		writel(1, alert->pad + MALIBU_AOSS_IMR);
	}
	raw_spin_unlock_irqrestore(&alert->lock, flags);
}

static void malibu_aoss_irq_disable(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct malibu_aoss_alert *alert = gpiochip_get_data(gc);
	unsigned long flags;

	raw_spin_lock_irqsave(&alert->lock, flags);
	writel(0, alert->pad + MALIBU_AOSS_IER);
	writel(0, alert->pad + MALIBU_AOSS_IMR);
	raw_spin_unlock_irqrestore(&alert->lock, flags);
	gpiochip_disable_irq(gc, irqd_to_hwirq(data));
}

static void malibu_aoss_irq_mask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct malibu_aoss_alert *alert = gpiochip_get_data(gc);
	unsigned long flags;

	raw_spin_lock_irqsave(&alert->lock, flags);
	writel(0, alert->pad + MALIBU_AOSS_IMR);
	raw_spin_unlock_irqrestore(&alert->lock, flags);
	gpiochip_disable_irq(gc, irqd_to_hwirq(data));
}

static void malibu_aoss_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct malibu_aoss_alert *alert = gpiochip_get_data(gc);
	unsigned long flags;

	gpiochip_enable_irq(gc, irqd_to_hwirq(data));
	raw_spin_lock_irqsave(&alert->lock, flags);
	if (!alert->stopping) {
		writel(1, alert->pad + MALIBU_AOSS_ISR);
		writel(1, alert->pad + MALIBU_AOSS_ISROVF);
		writel(1, alert->pad + MALIBU_AOSS_IMR);
	}
	raw_spin_unlock_irqrestore(&alert->lock, flags);
}

static const struct irq_chip malibu_aoss_irqchip = {
	.name = "malibu-aoss-alert",
	.irq_enable = malibu_aoss_irq_enable,
	.irq_disable = malibu_aoss_irq_disable,
	.irq_mask = malibu_aoss_irq_mask,
	.irq_unmask = malibu_aoss_irq_unmask,
	.irq_set_type = malibu_aoss_irq_set_type,
	.flags = IRQCHIP_IMMUTABLE | IRQCHIP_MASK_ON_SUSPEND,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static void malibu_aoss_chained_irq(struct irq_desc *desc)
{
	struct irq_chip *parent = irq_desc_get_chip(desc);
	struct malibu_aoss_alert *alert;
	struct irq_domain *domain;
	u32 status, overflow;

	rcu_read_lock();
	alert = READ_ONCE(desc->irq_common_data.handler_data);
	if (!alert)
		goto out;

	chained_irq_enter(parent, desc);
	raw_spin_lock(&alert->lock);
	domain = READ_ONCE(alert->gc.irq.domain);
	if (alert->stopping || !domain) {
		raw_spin_unlock(&alert->lock);
		goto chain_exit;
	}
	status = readl(alert->pad + MALIBU_AOSS_ISR);
	overflow = readl(alert->pad + MALIBU_AOSS_ISROVF);
	raw_spin_unlock(&alert->lock);

	/* The dedicated parent identifies this source, even with empty latches. */
	generic_handle_domain_irq(domain, MALIBU_AOSS_ALERT_PAD);

	raw_spin_lock(&alert->lock);
	writel(status, alert->pad + MALIBU_AOSS_ISR);
	if (overflow)
		writel(overflow, alert->pad + MALIBU_AOSS_ISROVF);
	raw_spin_unlock(&alert->lock);
chain_exit:
	chained_irq_exit(parent, desc);
out:
	rcu_read_unlock();
}

static void malibu_aoss_stop(void *data)
{
	struct malibu_aoss_alert *alert = data;
	unsigned long flags;

	raw_spin_lock_irqsave(&alert->lock, flags);
	alert->stopping = true;
	writel(0, alert->pad + MALIBU_AOSS_IER);
	writel(0, alert->pad + MALIBU_AOSS_IMR);
	readl(alert->pad + MALIBU_AOSS_IMR);
	raw_spin_unlock_irqrestore(&alert->lock, flags);

	/* Drain dispatchers before gpiolib tears down the GPIO irqdomain. */
	synchronize_rcu();
}

static void malibu_aoss_restore(void *data)
{
	struct malibu_aoss_alert *alert = data;

	/* gpiolib has detached the chained handler before this action. */
	synchronize_rcu();
	malibu_aoss_update(alert, MALIBU_AOSS_RXDATA, MALIBU_AOSS_RX_TRIGGER, alert->rx);
	malibu_aoss_update(alert, MALIBU_AOSS_DMUX, MALIBU_AOSS_DMUX_FUNC, alert->dmux);
	malibu_aoss_update(alert, MALIBU_AOSS_PARAM, MALIBU_AOSS_PARAM_INPUT, alert->param);
	malibu_aoss_update(alert, MALIBU_AOSS_TXDATA, MALIBU_AOSS_TX_PULL, 0);
	fsleep(10);
	malibu_aoss_update(alert, MALIBU_AOSS_TXDATA,
			   MALIBU_AOSS_TX_PULL | MALIBU_AOSS_TX_OUTPUT, alert->tx);
	writel(alert->ier, alert->pad + MALIBU_AOSS_IER);
	writel(alert->imr, alert->pad + MALIBU_AOSS_IMR);
}

static int malibu_aoss_alert_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct malibu_aoss_alert *alert;
	struct gpio_irq_chip *girq;
	struct resource *res;
	void __iomem *base;
	int ret;

	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		return ret;

	alert = devm_kzalloc(dev, sizeof(*alert), GFP_KERNEL);
	if (!alert)
		return -ENOMEM;
	alert->parent_irq = ret;
	raw_spin_lock_init(&alert->lock);

	base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(base))
		return PTR_ERR(base);
	if (resource_size(res) < MALIBU_AOSS_ALERT_OFFSET + MALIBU_AOSS_IMR + sizeof(u32))
		return dev_err_probe(dev, -EINVAL, "AOSS pad range is too small\n");
	alert->pad = base + MALIBU_AOSS_ALERT_OFFSET;

	alert->param = readl(alert->pad + MALIBU_AOSS_PARAM);
	alert->dmux = readl(alert->pad + MALIBU_AOSS_DMUX);
	alert->tx = readl(alert->pad + MALIBU_AOSS_TXDATA);
	alert->rx = readl(alert->pad + MALIBU_AOSS_RXDATA);
	alert->ier = readl(alert->pad + MALIBU_AOSS_IER) & BIT(0);
	alert->imr = readl(alert->pad + MALIBU_AOSS_IMR) & BIT(0);
	ret = devm_add_action(dev, malibu_aoss_restore, alert);
	if (ret)
		return ret;

	writel(0, alert->pad + MALIBU_AOSS_IMR);
	writel(0, alert->pad + MALIBU_AOSS_IER);
	malibu_aoss_update(alert, MALIBU_AOSS_TXDATA,
			   MALIBU_AOSS_TX_PULL | MALIBU_AOSS_TX_OUTPUT, 0);
	fsleep(10);
	malibu_aoss_update(alert, MALIBU_AOSS_TXDATA, MALIBU_AOSS_TX_PULL,
			   MALIBU_AOSS_TX_PULL_UP);
	malibu_aoss_update(alert, MALIBU_AOSS_PARAM, MALIBU_AOSS_PARAM_INPUT,
			   MALIBU_AOSS_PARAM_INPUT);
	malibu_aoss_update(alert, MALIBU_AOSS_DMUX, MALIBU_AOSS_DMUX_FUNC,
			   MALIBU_AOSS_DMUX_GPIO);
	malibu_aoss_update(alert, MALIBU_AOSS_RXDATA, MALIBU_AOSS_RX_TRIGGER,
			   MALIBU_AOSS_RX_LEVEL_LOW);

	alert->gc.label = dev_name(dev);
	alert->gc.parent = dev;
	alert->gc.owner = THIS_MODULE;
	alert->gc.base = -1;
	alert->gc.ngpio = MALIBU_AOSS_NGPIO;
	alert->gc.init_valid_mask = malibu_aoss_init_valid_mask;
	alert->gc.direction_input = malibu_aoss_direction_input;
	alert->gc.get_direction = malibu_aoss_get_direction;
	alert->gc.get = malibu_aoss_get;

	girq = &alert->gc.irq;
	gpio_irq_chip_set_chip(girq, &malibu_aoss_irqchip);
	girq->handler = handle_level_irq;
	girq->default_type = IRQ_TYPE_NONE;
	girq->parent_handler = malibu_aoss_chained_irq;
	girq->parent_handler_data = alert;
	girq->num_parents = 1;
	girq->parents = &alert->parent_irq;

	ret = devm_gpiochip_add_data(dev, &alert->gc, alert);
	if (ret)
		return ret;
	return devm_add_action_or_reset(dev, malibu_aoss_stop, alert);
}

static const struct of_device_id malibu_aoss_alert_of_match[] = {
	{ .compatible = "google,malibu-aoss-alert-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, malibu_aoss_alert_of_match);

static struct platform_driver malibu_aoss_alert_driver = {
	.probe = malibu_aoss_alert_probe,
	.driver = {
		.name = "malibu-aoss-alert-gpio",
		.of_match_table = malibu_aoss_alert_of_match,
	},
};
module_platform_driver(malibu_aoss_alert_driver);

MODULE_DESCRIPTION("Google Malibu AOSS Type-C alert GPIO");
MODULE_LICENSE("GPL");
