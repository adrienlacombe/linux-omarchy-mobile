// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/limits.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/time64.h>
#include <linux/watchdog.h>

#define MALIBU_WDT_ID			0x00
#define MALIBU_WDT_CTRL			0x08
#define MALIBU_WDT_RELOAD		0x0c
#define MALIBU_WDT_KEY			0x10

#define MALIBU_WDT_ID_VALUE		0x574454
#define MALIBU_WDT_CTRL_ENABLE		BIT(0)
#define MALIBU_WDT_CTRL_UNLOCKED	BIT(2)
#define MALIBU_WDT_UNLOCK		0x00c0ffee
#define MALIBU_WDT_LOCK			0

struct malibu_wdt {
	void __iomem *base;
	spinlock_t lock; /* Serializes the key and reload register sequence. */
	u32 reload;
	struct watchdog_device wdd;
};

static int malibu_wdt_ping(struct watchdog_device *wdd)
{
	struct malibu_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned long flags;
	u32 ctrl;
	int ret = 0;

	spin_lock_irqsave(&wdt->lock, flags);

	ctrl = readl(wdt->base + MALIBU_WDT_CTRL);
	if (!(ctrl & MALIBU_WDT_CTRL_ENABLE)) {
		ret = -EOPNOTSUPP;
		goto out;
	}
	if (ctrl & MALIBU_WDT_CTRL_UNLOCKED) {
		ret = -EBUSY;
		goto out;
	}

	/* Preserve the firmware's control and expiry settings. */
	writel(MALIBU_WDT_UNLOCK, wdt->base + MALIBU_WDT_KEY);
	if (!(readl(wdt->base + MALIBU_WDT_CTRL) & MALIBU_WDT_CTRL_UNLOCKED)) {
		ret = -EIO;
		goto out;
	}
	writel(wdt->reload, wdt->base + MALIBU_WDT_RELOAD);
	writel(MALIBU_WDT_LOCK, wdt->base + MALIBU_WDT_KEY);
	if (readl(wdt->base + MALIBU_WDT_CTRL) & MALIBU_WDT_CTRL_UNLOCKED)
		ret = -EIO;

out:
	spin_unlock_irqrestore(&wdt->lock, flags);
	if (ret && test_bit(WDOG_HW_RUNNING, &wdd->status))
		dev_warn_ratelimited(wdd->parent, "watchdog reload failed: %d\n", ret);
	return ret;
}

static const struct watchdog_info malibu_wdt_info = {
	.identity = "Google Malibu watchdog",
	.options = WDIOF_KEEPALIVEPING,
};

static const struct watchdog_ops malibu_wdt_ops = {
	.start = malibu_wdt_ping,
	.ping = malibu_wdt_ping,
};

static int malibu_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct malibu_wdt *wdt;
	unsigned long rate;
	struct clk *clk;
	u32 timeout;
	int ret;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(wdt->base))
		return PTR_ERR(wdt->base);

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "cannot enable watchdog clock\n");

	rate = clk_get_rate(clk);
	ret = device_property_read_u32(dev, "timeout-sec", &timeout);
	if (ret || !rate || !timeout || timeout > U32_MAX / rate ||
	    timeout > U32_MAX / MSEC_PER_SEC)
		return dev_err_probe(dev, -EINVAL, "invalid watchdog clock or timeout\n");

	if (readl(wdt->base + MALIBU_WDT_ID) != MALIBU_WDT_ID_VALUE)
		return dev_err_probe(dev, -ENODEV, "unrecognized watchdog hardware\n");

	if (!(readl(wdt->base + MALIBU_WDT_CTRL) & MALIBU_WDT_CTRL_ENABLE)) {
		dev_info(dev, "watchdog is inactive at handoff\n");
		return -ENODEV;
	}

	spin_lock_init(&wdt->lock);
	wdt->reload = timeout * rate;
	wdt->wdd.info = &malibu_wdt_info;
	wdt->wdd.ops = &malibu_wdt_ops;
	wdt->wdd.parent = dev;
	wdt->wdd.timeout = timeout;
	wdt->wdd.max_hw_heartbeat_ms = timeout * MSEC_PER_SEC;
	watchdog_set_drvdata(&wdt->wdd, wdt);
	watchdog_set_nowayout(&wdt->wdd, true);

	ret = malibu_wdt_ping(&wdt->wdd);
	if (ret)
		return dev_err_probe(dev, ret, "could not service the running watchdog\n");

	set_bit(WDOG_HW_RUNNING, &wdt->wdd.status);
	ret = devm_watchdog_register_device(dev, &wdt->wdd);
	if (ret)
		return dev_err_probe(dev, ret, "could not register the running watchdog\n");

	dev_info(dev, "servicing firmware-started watchdog (%u s)\n", timeout);
	return 0;
}

static const struct of_device_id malibu_wdt_of_match[] = {
	{ .compatible = "google,malibu-wdt" },
	{ }
};

static struct platform_driver malibu_wdt_driver = {
	.probe = malibu_wdt_probe,
	.driver = {
		.name = "google-malibu-wdt",
		.of_match_table = malibu_wdt_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(malibu_wdt_driver);
