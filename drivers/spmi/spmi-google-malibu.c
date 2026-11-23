// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/spmi.h>

#define MALIBU_SPMI_CONFIG		0x00
#define MALIBU_SPMI_STATUS		0x08
#define MALIBU_SPMI_COMMAND		0x10
#define MALIBU_SPMI_DATA			0x14

#define MALIBU_SPMI_CONFIG_IDLE		0x190
#define MALIBU_SPMI_CONFIG_ENABLE	BIT(0)
#define MALIBU_SPMI_STATUS_DONE		BIT(0)
#define MALIBU_SPMI_STATUS_ERRORS	GENMASK(9, 2)
#define MALIBU_SPMI_CMD_START		0x3
#define MALIBU_SPMI_CMD_OP		GENMASK(6, 2)
#define MALIBU_SPMI_CMD_SLAVE		GENMASK(10, 7)
#define MALIBU_SPMI_CMD_LENGTH		GENMASK(14, 11)
#define MALIBU_SPMI_CMD_ADDR		GENMASK(30, 15)
#define MALIBU_SPMI_DATA_WORDS		5
#define MALIBU_SPMI_POLL_US		10
#define MALIBU_SPMI_TIMEOUT_US		100000

struct malibu_spmi {
	void __iomem *base;
	struct mutex lock; /* Serializes transfers through the command and data registers. */
	bool stalled;
};

static int malibu_spmi_opcode(u8 opcode, u16 addr, size_t len, bool write)
{
	switch (opcode) {
	case SPMI_CMD_READ:
		return !write && addr <= 0x1f && len == 1 ? 1 : -EINVAL;
	case SPMI_CMD_WRITE:
		return write && addr <= 0x1f && len == 1 ? 0 : -EINVAL;
	case SPMI_CMD_EXT_READ:
		return !write && addr <= 0xff && len <= 16 ? 3 : -EINVAL;
	case SPMI_CMD_EXT_WRITE:
		return write && addr <= 0xff && len <= 16 ? 2 : -EINVAL;
	case SPMI_CMD_EXT_READL:
		return !write && len <= 8 ? 5 : -EINVAL;
	case SPMI_CMD_EXT_WRITEL:
		return write && len <= 8 ? 4 : -EINVAL;
	case SPMI_CMD_ZERO_WRITE:
		return write && !addr && len == 1 ? 6 : -EINVAL;
	default:
		return -EOPNOTSUPP;
	}
}

static void malibu_spmi_write_data(struct malibu_spmi *spmi, const u8 *buf, size_t len)
{
	u32 word;
	size_t i, j;

	for (i = 0; i < len; i += 4) {
		word = 0;
		for (j = 0; j < 4 && i + j < len; j++)
			word |= (u32)buf[i + j] << (8 * j);
		writel(word, spmi->base + MALIBU_SPMI_DATA + i);
	}
}

static void malibu_spmi_read_data(struct malibu_spmi *spmi, u8 *buf, size_t len)
{
	u32 word;
	size_t i, j;

	for (i = 0; i < len; i += 4) {
		word = readl(spmi->base + MALIBU_SPMI_DATA + i);
		for (j = 0; j < 4 && i + j < len; j++)
			buf[i + j] = word >> (8 * j);
	}
}

static int malibu_spmi_transfer(struct spmi_controller *ctrl, u8 opcode, u8 sid,
				u16 addr, const u8 *tx, u8 *rx, size_t len)
{
	struct malibu_spmi *spmi = spmi_controller_get_drvdata(ctrl);
	u32 cmd, status;
	unsigned int i;
	int op, ret;

	if (sid >= SPMI_MAX_SLAVE_ID || !len)
		return -EINVAL;
	op = malibu_spmi_opcode(opcode, addr, len, tx);
	if (op < 0)
		return op;

	cmd = MALIBU_SPMI_CMD_START | FIELD_PREP(MALIBU_SPMI_CMD_OP, op) |
	      FIELD_PREP(MALIBU_SPMI_CMD_SLAVE, sid) |
	      FIELD_PREP(MALIBU_SPMI_CMD_LENGTH, len - 1) |
	      FIELD_PREP(MALIBU_SPMI_CMD_ADDR, addr);

	guard(mutex)(&spmi->lock);
	if (spmi->stalled)
		return -ETIMEDOUT;

	if (tx)
		malibu_spmi_write_data(spmi, tx, len);
	else
		for (i = 0; i < MALIBU_SPMI_DATA_WORDS; i++)
			writel(0, spmi->base + MALIBU_SPMI_DATA + i * sizeof(u32));

	writel(MALIBU_SPMI_CONFIG_IDLE | MALIBU_SPMI_CONFIG_ENABLE,
	       spmi->base + MALIBU_SPMI_CONFIG);
	writel(cmd, spmi->base + MALIBU_SPMI_COMMAND);
	ret = readl_poll_timeout(spmi->base + MALIBU_SPMI_STATUS, status,
				 status & MALIBU_SPMI_STATUS_DONE,
				 MALIBU_SPMI_POLL_US, MALIBU_SPMI_TIMEOUT_US);
	if (status)
		writel(status, spmi->base + MALIBU_SPMI_STATUS);
	writel(MALIBU_SPMI_CONFIG_IDLE, spmi->base + MALIBU_SPMI_CONFIG);
	if (ret) {
		/* The old command may still complete after the timeout. */
		spmi->stalled = true;
		return ret;
	}
	if (status & MALIBU_SPMI_STATUS_ERRORS)
		return -EIO;
	if (rx)
		malibu_spmi_read_data(spmi, rx, len);
	return 0;
}

static int malibu_spmi_read_cmd(struct spmi_controller *ctrl, u8 opcode, u8 sid,
				u16 addr, u8 *buf, size_t len)
{
	if (!buf)
		return -EINVAL;
	return malibu_spmi_transfer(ctrl, opcode, sid, addr, NULL, buf, len);
}

static int malibu_spmi_write_cmd(struct spmi_controller *ctrl, u8 opcode, u8 sid,
				 u16 addr, const u8 *buf, size_t len)
{
	if (!buf)
		return -EINVAL;
	return malibu_spmi_transfer(ctrl, opcode, sid, addr, buf, NULL, len);
}

static int malibu_spmi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spmi_controller *ctrl;
	struct malibu_spmi *spmi;
	struct resource *res;

	ctrl = devm_spmi_controller_alloc(dev, sizeof(*spmi));
	if (IS_ERR(ctrl))
		return PTR_ERR(ctrl);
	spmi = spmi_controller_get_drvdata(ctrl);
	mutex_init(&spmi->lock);
	spmi->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(spmi->base))
		return PTR_ERR(spmi->base);
	if (resource_size(res) < MALIBU_SPMI_DATA + sizeof(u32) * MALIBU_SPMI_DATA_WORDS)
		return -EINVAL;

	ctrl->read_cmd = malibu_spmi_read_cmd;
	ctrl->write_cmd = malibu_spmi_write_cmd;
	return devm_spmi_controller_add(dev, ctrl);
}

static const struct of_device_id malibu_spmi_of_match[] = {
	{ .compatible = "google,malibu-smartdv-spmi" },
	{ }
};
MODULE_DEVICE_TABLE(of, malibu_spmi_of_match);

static struct platform_driver malibu_spmi_driver = {
	.probe = malibu_spmi_probe,
	.driver = {
		.name = "spmi-google-malibu",
		.of_match_table = malibu_spmi_of_match,
	},
};
module_platform_driver(malibu_spmi_driver);

MODULE_DESCRIPTION("Google Malibu SmartDV SPMI controller");
MODULE_LICENSE("GPL");
