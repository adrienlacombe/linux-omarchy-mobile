// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define MALIBU_MBA_KICK			0x00
#define MALIBU_MBA_IRQ_CONTROL		0x04
#define MALIBU_MBA_IRQ_STATUS		0x08
#define MALIBU_MBA_OUTSTANDING		0x10
#define MALIBU_MBA_STORAGE_WORDS	0x34
#define MALIBU_MBA_FRAME_BASE		0x100

#define MALIBU_MBA_RX_IRQ		BIT(0)
#define MALIBU_MBA_TX_IRQ		BIT(8)
#define MALIBU_MBA_IRQ_ENABLE_MASK	0x01010100
#define MALIBU_MBA_FRAME_WORDS		4
#define MALIBU_MBA_RESET_FRAME		0x100

struct malibu_mba {
	void __iomem *base;
	struct mbox_controller controller;
	struct mbox_chan channel;
	spinlock_t lock; /* Serialize queue cursors and IRQ control. */
	unsigned int slots;
	unsigned int tx_slot;
	unsigned int rx_slot;
	int irq;
	bool started;
	bool used;
	bool tx_pending;
	bool tx_reset;
};

static void __iomem *malibu_mba_slot(struct malibu_mba *mba, unsigned int slot)
{
	return mba->base + MALIBU_MBA_FRAME_BASE + slot * MALIBU_MBA_FRAME_WORDS * sizeof(u32);
}

static void malibu_mba_complete_tx(struct malibu_mba *mba)
{
	if (mba->tx_reset) {
		mba->rx_slot = 0;
		mba->tx_slot = 0;
	}
	mba->tx_pending = false;
	mba->tx_reset = false;
}

static int malibu_mba_send(struct mbox_chan *chan, void *data)
{
	struct malibu_mba *mba = chan->con_priv;
	const u32 *words = data;
	void __iomem *slot;
	unsigned int i;

	if (!words)
		return -EINVAL;

	guard(spinlock_irqsave)(&mba->lock);
	if (!mba->started || mba->tx_pending)
		return -EBUSY;

	slot = malibu_mba_slot(mba, mba->tx_slot);
	for (i = 0; i < MALIBU_MBA_FRAME_WORDS; i++)
		writel(words[i], slot + i * sizeof(u32));
	mba->tx_reset = mba->tx_slot == 0 && words[0] == MALIBU_MBA_RESET_FRAME;
	mba->tx_slot = (mba->tx_slot + 1) % mba->slots;
	mba->tx_pending = true;
	mba->used = true;
	writel(1, mba->base + MALIBU_MBA_KICK);
	return 0;
}

static void malibu_mba_tx_irq(struct malibu_mba *mba)
{
	scoped_guard(spinlock_irqsave, &mba->lock) {
		if (!mba->started)
			return;
		writel(MALIBU_MBA_TX_IRQ, mba->base + MALIBU_MBA_IRQ_STATUS);
		if (!mba->tx_pending || readl(mba->base + MALIBU_MBA_OUTSTANDING))
			return;
		malibu_mba_complete_tx(mba);
	}

	/* Mailbox core can submit the next frame from the completion. */
	mbox_chan_txdone(&mba->channel, 0);
}

static void malibu_mba_rx_irq(struct malibu_mba *mba)
{
	u32 words[MALIBU_MBA_FRAME_WORDS];
	void __iomem *slot;
	unsigned int i;

	scoped_guard(spinlock_irqsave, &mba->lock) {
		if (!mba->started)
			return;
		slot = malibu_mba_slot(mba, mba->rx_slot);
		for (i = 0; i < MALIBU_MBA_FRAME_WORDS; i++)
			words[i] = readl(slot + i * sizeof(u32));
		mba->rx_slot = (mba->rx_slot + 1) % mba->slots;
		mba->used = true;
	}

	mbox_chan_received_data(&mba->channel, words);
	writel(MALIBU_MBA_RX_IRQ, mba->base + MALIBU_MBA_IRQ_STATUS);
}

static irqreturn_t malibu_mba_irq(int irq, void *data)
{
	struct malibu_mba *mba = data;
	u32 status = readl(mba->base + MALIBU_MBA_IRQ_STATUS);

	if (!(status & (MALIBU_MBA_RX_IRQ | MALIBU_MBA_TX_IRQ)))
		return IRQ_NONE;
	if (status & MALIBU_MBA_TX_IRQ)
		malibu_mba_tx_irq(mba);
	if (status & MALIBU_MBA_RX_IRQ)
		malibu_mba_rx_irq(mba);
	return IRQ_HANDLED;
}

static int malibu_mba_startup(struct mbox_chan *chan)
{
	struct malibu_mba *mba = chan->con_priv;
	u32 config;
	u32 outstanding, status;
	bool used;

	/* The mailbox core's blocking timeout cannot cancel a hardware send. */
	if (chan->cl->tx_block)
		return -EOPNOTSUPP;

	scoped_guard(spinlock_irqsave, &mba->lock) {
		status = readl(mba->base + MALIBU_MBA_IRQ_STATUS);
		outstanding = readl(mba->base + MALIBU_MBA_OUTSTANDING);
		used = mba->used;
		/* A used queue has no verified protocol for a new client's starting slot. */
		if (used || outstanding || (status & MALIBU_MBA_RX_IRQ))
			goto busy;
		writel(MALIBU_MBA_TX_IRQ, mba->base + MALIBU_MBA_IRQ_STATUS);
		config = readl(mba->base + MALIBU_MBA_IRQ_CONTROL);
		writel(config | MALIBU_MBA_IRQ_ENABLE_MASK, mba->base + MALIBU_MBA_IRQ_CONTROL);
		mba->started = true;
	}

	enable_irq(mba->irq);
	return 0;

busy:
	dev_warn(mba->controller.dev,
		 "mailbox startup rejected: used=%u outstanding=%#x irq_status=%#x\n",
		 used, outstanding, status);
	return -EBUSY;
}

static void malibu_mba_shutdown(struct mbox_chan *chan)
{
	struct malibu_mba *mba = chan->con_priv;
	u32 config;

	scoped_guard(spinlock_irqsave, &mba->lock) {
		mba->started = false;
		config = readl(mba->base + MALIBU_MBA_IRQ_CONTROL);
		writel(config & ~MALIBU_MBA_IRQ_ENABLE_MASK, mba->base + MALIBU_MBA_IRQ_CONTROL);
		readl(mba->base + MALIBU_MBA_IRQ_CONTROL);
	}

	disable_irq(mba->irq);
}

static const struct mbox_chan_ops malibu_mba_ops = {
	.send_data = malibu_mba_send,
	.startup = malibu_mba_startup,
	.shutdown = malibu_mba_shutdown,
};

static int malibu_mba_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct malibu_mba *mba;
	u32 words;
	int ret;

	mba = devm_kzalloc(dev, sizeof(*mba), GFP_KERNEL);
	if (!mba)
		return -ENOMEM;
	mba->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(mba->base))
		return PTR_ERR(mba->base);
	if (resource_size(res) < MALIBU_MBA_FRAME_BASE + MALIBU_MBA_FRAME_WORDS * sizeof(u32))
		return dev_err_probe(dev, -EINVAL, "mailbox window is too small\n");

	words = readl(mba->base + MALIBU_MBA_STORAGE_WORDS);
	if (words < MALIBU_MBA_FRAME_WORDS || words % MALIBU_MBA_FRAME_WORDS ||
	    words > (resource_size(res) - MALIBU_MBA_FRAME_BASE) / sizeof(u32))
		return dev_err_probe(dev, -EINVAL, "invalid mailbox frame storage\n");
	mba->slots = words / MALIBU_MBA_FRAME_WORDS;
	spin_lock_init(&mba->lock);

	mba->irq = platform_get_irq(pdev, 0);
	if (mba->irq < 0)
		return mba->irq;
	ret = devm_request_irq(dev, mba->irq, malibu_mba_irq, IRQF_NO_AUTOEN, dev_name(dev), mba);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request mailbox IRQ\n");

	mba->channel.con_priv = mba;
	mba->controller.dev = dev;
	mba->controller.ops = &malibu_mba_ops;
	mba->controller.chans = &mba->channel;
	mba->controller.num_chans = 1;
	mba->controller.txdone_irq = true;
	return devm_mbox_controller_register(dev, &mba->controller);
}

static const struct of_device_id malibu_mba_of_match[] = {
	{ .compatible = "google,malibu-mba-v3-queue" },
	{ }
};
MODULE_DEVICE_TABLE(of, malibu_mba_of_match);

static struct platform_driver malibu_mba_driver = {
	.probe = malibu_mba_probe,
	.driver = {
		.name = "google-malibu-mba",
		.of_match_table = malibu_mba_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(malibu_mba_driver);

MODULE_DESCRIPTION("Google Malibu MBA v3 queued mailbox");
MODULE_LICENSE("GPL");
