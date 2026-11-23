// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/clk-provider.h>
#include <linux/completion.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/regulator/driver.h>
#include <linux/reset-controller.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>

#include <dt-bindings/clock/google,malibu-cpm-io.h>

#define MALIBU_CPM_REQUEST		0xa0000000
#define MALIBU_CPM_MSG_TYPE		GENMASK(30, 29)
#define MALIBU_CPM_REPLY		2
#define MALIBU_CPM_TOKEN		GENMASK(23, 20)
#define MALIBU_CPM_SEQUENCE		GENMASK(19, 16)
#define MALIBU_CPM_TOKEN_COUNT		16
#define MALIBU_CPM_SEQ_LAST		13
#define MALIBU_CPM_RESET_FRAME		0x100
#define MALIBU_CPM_DESTINATION		GENMASK(7, 0)
#define MALIBU_CPM_NOTIFICATION		0
#define MALIBU_CPM_NOTIFICATION_ALT	1
#define MALIBU_CPM_GENPD_CLIENT		7
#define MALIBU_CPM_CLOCK_SERVICE	2
#define MALIBU_CPM_RESET_SERVICE	4
#define MALIBU_CPM_LPCM_DESTINATION	8
#define MALIBU_CPM_HSION_DOMAIN		7
#define MALIBU_CPM_LSIOE_DOMAIN		5
#define MALIBU_CPM_CLOCK_TIMEOUT_MS	1000
#define MALIBU_CPM_RESET_TIMEOUT_MS	3000

#define MALIBU_MBFS_DESTINATION		0x14
#define MALIBU_MBFS_TIMEOUT_MS		10000
#define MALIBU_MBFS_TRANSITION_MS	3000
#define MALIBU_MBFS_STATUS		GENMASK(31, 29)
#define MALIBU_MBFS_HANDLE		GENMASK(27, 0)
#define MALIBU_MBFS_DEFERRED		5
#define MALIBU_MBFS_ROOT			0x10000000
#define MALIBU_MBFS_LOOKUP		0x40000000
#define MALIBU_MBFS_LOOKUP_TAIL		0x50000000
#define MALIBU_MBFS_READ			0x70000000
#define MALIBU_MBFS_WRITE		0x80000000
#define MALIBU_MBFS_NAME_SIZE		16
#define MALIBU_MBFS_DOMAIN_COUNT	(MALIBU_CPM_PD_LSIOE_CLI + 1)
#define MALIBU_MBFS_NO_PARENT		-1

struct malibu_cpm {
	struct device *dev;
	struct mbox_client tx_client;
	struct mbox_client rx_client;
	struct mbox_chan *tx;
	struct completion tx_done;
	struct completion rx_done;
	struct completion transition_done;
	struct reset_controller_dev reset;
	/* Serialize callers and keep the TX buffer alive until completion. */
	struct mutex request_lock;
	/* Serialize tokenless power transitions across all domains. */
	struct mutex transition_lock;
	/* Protect response and transition state against the RX callback. */
	spinlock_t response_lock;
	u32 frame[4];
	u32 response[3];
	u32 transition_id;
	int tx_result;
	u8 sequence;
	u8 token;
	u8 pending_token;
	bool awaiting_reply;
	bool awaiting_transition;
	bool transition_stalled;
	bool running;
	bool remote_reset;
	bool stalled;
};

struct malibu_cpm_clock {
	struct clk_hw hw;
	struct malibu_cpm *cpm;
	u8 domain;
	u8 resource;
};

static const struct {
	u8 domain;
	u8 resource;
} malibu_cpm_resets[] = {
	[MALIBU_CPM_RST_USBC_STICKY] = { MALIBU_CPM_HSION_DOMAIN, 0 },
	[MALIBU_CPM_RST_USBC_NON_STICKY] = { MALIBU_CPM_HSION_DOMAIN, 1 },
	[MALIBU_CPM_RST_USB2_PHY] = { MALIBU_CPM_HSION_DOMAIN, 2 },
	[MALIBU_CPM_RST_USB2_APB] = { MALIBU_CPM_HSION_DOMAIN, 5 },
	[MALIBU_CPM_RST_USB_DRD_BUS] = { MALIBU_CPM_HSION_DOMAIN, 6 },
	[MALIBU_CPM_RST_USB_TOP] = { MALIBU_CPM_HSION_DOMAIN, 7 },
	[MALIBU_CPM_RST_I2C3] = { MALIBU_CPM_LSIOE_DOMAIN, 0x23 },
};

struct malibu_cpm_domain {
	struct generic_pm_domain genpd;
	struct malibu_cpm *cpm;
	struct malibu_cpm_domain *parent;
	u32 id;
	u32 handle;
};

struct malibu_cpm_domains {
	struct malibu_cpm_domain domain[MALIBU_MBFS_DOMAIN_COUNT];
	struct generic_pm_domain *genpds[MALIBU_MBFS_DOMAIN_COUNT];
	struct genpd_onecell_data onecell;
	struct device_node *node;
	bool published;
};

static void malibu_cpm_tx_done(struct mbox_client *client, void *msg, int ret)
{
	struct malibu_cpm *cpm = container_of(client, struct malibu_cpm, tx_client);

	WRITE_ONCE(cpm->tx_result, ret);
	complete(&cpm->tx_done);
}

static void malibu_cpm_rx(struct mbox_client *client, void *msg)
{
	struct malibu_cpm *cpm = container_of(client, struct malibu_cpm, rx_client);
	const u32 *words = msg;
	u32 header = words[0];
	u32 type = FIELD_GET(MALIBU_CPM_MSG_TYPE, header);

	guard(spinlock_irqsave)(&cpm->response_lock);
	if (header == MALIBU_CPM_RESET_FRAME && cpm->running) {
		WRITE_ONCE(cpm->remote_reset, true);
		cpm->awaiting_reply = false;
		cpm->awaiting_transition = false;
		complete(&cpm->rx_done);
		complete(&cpm->transition_done);
		return;
	}
	if ((type == MALIBU_CPM_NOTIFICATION || type == MALIBU_CPM_NOTIFICATION_ALT) &&
	    FIELD_GET(MALIBU_CPM_DESTINATION, header) == MALIBU_CPM_GENPD_CLIENT &&
	    cpm->awaiting_transition && words[1] == cpm->transition_id) {
		cpm->awaiting_transition = false;
		complete(&cpm->transition_done);
		return;
	}
	if (type != MALIBU_CPM_REPLY)
		return;

	if (!cpm->awaiting_reply || FIELD_GET(MALIBU_CPM_TOKEN, header) != cpm->pending_token)
		return;
	memcpy(cpm->response, words + 1, sizeof(cpm->response));
	cpm->awaiting_reply = false;
	complete(&cpm->rx_done);
}

static int malibu_cpm_transmit(struct malibu_cpm *cpm, unsigned long *remaining,
			       u32 transition_id)
{
	int ret;

	reinit_completion(&cpm->tx_done);
	if (transition_id)
		reinit_completion(&cpm->transition_done);
	scoped_guard(spinlock_irqsave, &cpm->response_lock) {
		if (cpm->remote_reset)
			return -EPROTO;
		/* A deferred notification can arrive before the write reply. */
		if (transition_id) {
			cpm->transition_id = transition_id;
			cpm->awaiting_transition = true;
		}
		ret = mbox_send_message(cpm->tx, cpm->frame);
	}
	if (ret < 0)
		return ret;
	*remaining = wait_for_completion_timeout(&cpm->tx_done, *remaining);
	if (!*remaining)
		return -ETIMEDOUT;
	return READ_ONCE(cpm->tx_result);
}

static int malibu_cpm_request(struct malibu_cpm *cpm, u8 destination, const u32 *payload,
			      unsigned int timeout_ms, u32 transition_id, u32 *reply)
{
	unsigned long remaining = msecs_to_jiffies(timeout_ms);
	int ret;

	guard(mutex)(&cpm->request_lock);
	if (cpm->stalled)
		return -ETIMEDOUT;

	reinit_completion(&cpm->rx_done);
	scoped_guard(spinlock_irqsave, &cpm->response_lock) {
		cpm->pending_token = cpm->token;
		cpm->awaiting_reply = true;
	}
	cpm->frame[0] = MALIBU_CPM_REQUEST | FIELD_PREP(MALIBU_CPM_TOKEN, cpm->token) |
			FIELD_PREP(MALIBU_CPM_SEQUENCE, cpm->sequence) | destination;
	memcpy(cpm->frame + 1, payload, sizeof(cpm->response));
	cpm->token = (cpm->token + 1) % MALIBU_CPM_TOKEN_COUNT;
	cpm->sequence = cpm->sequence % MALIBU_CPM_SEQ_LAST + 1;

	ret = malibu_cpm_transmit(cpm, &remaining, transition_id);
	if (!ret && !wait_for_completion_timeout(&cpm->rx_done, remaining))
		ret = -ETIMEDOUT;
	scoped_guard(spinlock_irqsave, &cpm->response_lock) {
		cpm->awaiting_reply = false;
		if (!ret && cpm->remote_reset)
			ret = -EPROTO;
		if (!ret)
			memcpy(reply, cpm->response, sizeof(cpm->response));
	}
	/* A timeout can leave a physical transfer or a reply in flight. */
	if (ret == -ETIMEDOUT)
		WRITE_ONCE(cpm->stalled, true);
	return ret;
}

#define MALIBU_CPM_REGULATOR_DESTINATION	0x0a
#define MALIBU_CPM_REGULATOR_TIMEOUT_MS	3000
#define MALIBU_CPM_REGULATOR_REQUEST	0x01000000
#define MALIBU_CPM_REGULATOR_STATUS	GENMASK(31, 24)
#define MALIBU_CPM_RAIL_GET_ENABLED	1
#define MALIBU_CPM_RAIL_SET_ENABLED	2
#define MALIBU_CPM_RAIL_GET_SELECTOR	3
#define MALIBU_CPM_LDO_19M		0x1f

struct malibu_cpm_usb2_rail {
	struct regulator_desc desc;
	u8 voltage_selector;
	bool must_be_on;
};

static int malibu_cpm_usb2_rail_request(struct malibu_cpm *cpm, unsigned int id,
					unsigned int op, bool enable, u32 *value)
{
	const u32 payload[] = {
		MALIBU_CPM_REGULATOR_REQUEST | (op << 16) | id, enable, 0,
	};
	u32 reply[3];
	int ret;

	ret = malibu_cpm_request(cpm, MALIBU_CPM_REGULATOR_DESTINATION, payload,
				 MALIBU_CPM_REGULATOR_TIMEOUT_MS, 0, reply);
	if (ret)
		return ret;

	if (reply[0] & MALIBU_CPM_REGULATOR_STATUS)
		return -EPIPE;

	if (value)
		*value = reply[1];
	return 0;
}

static int malibu_cpm_usb2_rail_state(struct malibu_cpm *cpm, unsigned int id)
{
	u32 value;
	int ret;

	ret = malibu_cpm_usb2_rail_request(cpm, id, MALIBU_CPM_RAIL_GET_ENABLED, false,
					   &value);
	if (ret)
		return ret;

	/* The status result must be a boolean before exposing it to the core. */
	if (value > 1)
		return -EPROTO;
	return value;
}

static int malibu_cpm_usb2_rail_is_enabled(struct regulator_dev *rdev)
{
	return malibu_cpm_usb2_rail_state(rdev_get_drvdata(rdev), rdev_get_id(rdev));
}

static int malibu_cpm_usb2_rail_set_enabled(struct regulator_dev *rdev, bool enable)
{
	unsigned int id = rdev_get_id(rdev);

	if (id != MALIBU_CPM_LDO_19M)
		return -EOPNOTSUPP;

	return malibu_cpm_usb2_rail_request(rdev_get_drvdata(rdev), id,
					  MALIBU_CPM_RAIL_SET_ENABLED, enable, NULL);
}

static int malibu_cpm_usb2_rail_enable(struct regulator_dev *rdev)
{
	return malibu_cpm_usb2_rail_set_enabled(rdev, true);
}

static int malibu_cpm_usb2_rail_disable(struct regulator_dev *rdev)
{
	return malibu_cpm_usb2_rail_set_enabled(rdev, false);
}

static const struct regulator_ops malibu_cpm_usb2_switchable_ops = {
	.enable = malibu_cpm_usb2_rail_enable,
	.disable = malibu_cpm_usb2_rail_disable,
	.is_enabled = malibu_cpm_usb2_rail_is_enabled,
};

static const struct regulator_ops malibu_cpm_usb2_observe_ops = {
	.is_enabled = malibu_cpm_usb2_rail_is_enabled,
};

static const struct malibu_cpm_usb2_rail malibu_cpm_usb2_rails[] = {
	{
		.desc = {
			.name = "ldo_19m",
			.of_match = "ldo-19m",
			.regulators_node = "regulators",
			.id = MALIBU_CPM_LDO_19M,
			.type = REGULATOR_VOLTAGE,
			.owner = THIS_MODULE,
			.ops = &malibu_cpm_usb2_switchable_ops,
			.n_voltages = 1,
			.fixed_uV = 3300000,
		},
		.voltage_selector = 210,
	}, {
		.desc = {
			.name = "buck_12s",
			.of_match = "buck-12s",
			.regulators_node = "regulators",
			.id = 0x34,
			.type = REGULATOR_VOLTAGE,
			.owner = THIS_MODULE,
			.ops = &malibu_cpm_usb2_observe_ops,
			.n_voltages = 1,
			.fixed_uV = 1850000,
		},
		.voltage_selector = 105,
		.must_be_on = true,
	}, {
		.desc = {
			.name = "ldo_12s",
			.of_match = "ldo-12s",
			.regulators_node = "regulators",
			.id = 0x42,
			.type = REGULATOR_VOLTAGE,
			.owner = THIS_MODULE,
			.ops = &malibu_cpm_usb2_observe_ops,
			.n_voltages = 1,
			.fixed_uV = 1200000,
		},
		.voltage_selector = 160,
		.must_be_on = true,
	},
};

static int malibu_cpm_verify_usb2_rail(struct malibu_cpm *cpm,
				       const struct malibu_cpm_usb2_rail *rail)
{
	u32 selector;
	int ret;

	ret = malibu_cpm_usb2_rail_request(cpm, rail->desc.id, MALIBU_CPM_RAIL_GET_SELECTOR,
					   false, &selector);
	if (ret)
		return ret;
	if (selector != rail->voltage_selector)
		return -ERANGE;

	ret = malibu_cpm_usb2_rail_state(cpm, rail->desc.id);
	if (ret < 0)
		return ret;
	if (!ret && rail->must_be_on)
		return -ENODEV;
	return 0;
}

static int malibu_cpm_register_usb2_rails(struct malibu_cpm *cpm)
{
	const struct regulator_config config = {
		.dev = cpm->dev,
		.driver_data = cpm,
	};
	struct device_node *regulators __free(device_node) =
		of_get_child_by_name(cpm->dev->of_node, "regulators");
	const struct malibu_cpm_usb2_rail *rail;
	struct regulator_dev *rdev;
	void *group;
	unsigned int i;
	int ret;

	if (!regulators || !of_device_is_available(regulators))
		return 0;

	for (i = 0; i < ARRAY_SIZE(malibu_cpm_usb2_rails); i++) {
		struct device_node *node __free(device_node) = NULL;

		rail = &malibu_cpm_usb2_rails[i];
		node = of_get_child_by_name(regulators, rail->desc.of_match);
		if (!node || !of_device_is_available(node))
			return dev_err_probe(cpm->dev, -EINVAL, "missing USB2 rail %s\n",
					     rail->desc.name);
		ret = malibu_cpm_verify_usb2_rail(cpm, rail);
		if (ret)
			return dev_err_probe(cpm->dev, ret, "unusable USB2 rail %s\n",
					     rail->desc.name);
	}

	group = devres_open_group(cpm->dev, NULL, GFP_KERNEL);
	if (!group)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(malibu_cpm_usb2_rails); i++) {
		rail = &malibu_cpm_usb2_rails[i];
		rdev = devm_regulator_register(cpm->dev, &rail->desc, &config);
		if (IS_ERR(rdev)) {
			devres_release_group(cpm->dev, group);
			return dev_err_probe(cpm->dev, PTR_ERR(rdev), "registering USB2 rail %s\n",
					     rail->desc.name);
		}
	}

	devres_remove_group(cpm->dev, group);
	return 0;
}

static int malibu_cpm_lpcm_request(struct malibu_cpm *cpm, u32 command, u32 arg,
				   unsigned int timeout_ms, s32 *status)
{
	const u32 payload[] = { command, arg, 0 };
	u32 reply[3];
	int ret;

	ret = malibu_cpm_request(cpm, MALIBU_CPM_LPCM_DESTINATION, payload, timeout_ms, 0, reply);
	if (!ret)
		*status = (s32)reply[0];
	return ret;
}

static int malibu_cpm_translate_error(s32 reply, bool clock)
{
	switch (reply) {
	case 0:
		return 0;
	case -1:
		return -ESERVERFAULT;
	case -3:
		return clock ? -ENODATA : -EPROTO;
	case -8:
		return -EINVAL;
	case -13:
		return -ETIMEDOUT;
	default:
		return -EPROTO;
	}
}

static u32 malibu_cpm_command(u8 service, u8 domain, u8 resource, u8 op)
{
	return ((u32)op << 24) | ((u32)resource << 16) | ((u32)domain << 8) | service;
}

static int malibu_cpm_clock_gate(struct malibu_cpm_clock *clock, unsigned int op)
{
	u32 command = malibu_cpm_command(MALIBU_CPM_CLOCK_SERVICE, clock->domain,
					clock->resource, op);
	s32 reply;
	int ret;

	ret = malibu_cpm_lpcm_request(clock->cpm, command, 1, MALIBU_CPM_CLOCK_TIMEOUT_MS, &reply);
	if (ret)
		return ret;
	return malibu_cpm_translate_error(reply, true);
}

static int malibu_cpm_clock_prepare(struct clk_hw *hw)
{
	return malibu_cpm_clock_gate(container_of(hw, struct malibu_cpm_clock, hw), 0);
}

static void malibu_cpm_clock_unprepare(struct clk_hw *hw)
{
	struct malibu_cpm_clock *clock = container_of(hw, struct malibu_cpm_clock, hw);
	int ret;

	ret = malibu_cpm_clock_gate(clock, 1);
	if (ret)
		dev_err_ratelimited(clock->cpm->dev, "failed to unprepare clock %s: %d\n",
				    clk_hw_get_name(hw), ret);
}

static unsigned long malibu_cpm_clock_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct malibu_cpm_clock *clock = container_of(hw, struct malibu_cpm_clock, hw);
	u32 command = malibu_cpm_command(MALIBU_CPM_CLOCK_SERVICE, clock->domain,
					clock->resource, 3);
	s32 reply;
	int ret;

	ret = malibu_cpm_lpcm_request(clock->cpm, command, 0, MALIBU_CPM_CLOCK_TIMEOUT_MS, &reply);
	if (!ret && reply >= 0)
		return reply;
	if (!ret)
		ret = malibu_cpm_translate_error(reply, true);
	dev_err_ratelimited(clock->cpm->dev, "failed to read clock %s rate: %d\n",
			    clk_hw_get_name(hw), ret);
	return 0;
}

static const struct clk_ops malibu_cpm_clock_ops = {
	.prepare = malibu_cpm_clock_prepare,
	.unprepare = malibu_cpm_clock_unprepare,
	.recalc_rate = malibu_cpm_clock_rate,
};

static bool malibu_cpm_reset_valid(unsigned long id)
{
	return id < ARRAY_SIZE(malibu_cpm_resets) && malibu_cpm_resets[id].domain;
}

static int malibu_cpm_reset_xlate(struct reset_controller_dev *rcdev,
				  const struct of_phandle_args *spec)
{
	if (spec->args_count != 1 || !malibu_cpm_reset_valid(spec->args[0]))
		return -EINVAL;
	return spec->args[0];
}

static int malibu_cpm_reset_set(struct reset_controller_dev *rcdev,
				unsigned long id, unsigned int op)
{
	struct malibu_cpm *cpm = container_of(rcdev, struct malibu_cpm, reset);
	u32 command;
	s32 reply;
	int ret;

	if (!malibu_cpm_reset_valid(id))
		return -EINVAL;
	command = malibu_cpm_command(MALIBU_CPM_RESET_SERVICE, malibu_cpm_resets[id].domain,
				     malibu_cpm_resets[id].resource, op);
	ret = malibu_cpm_lpcm_request(cpm, command, 0, MALIBU_CPM_RESET_TIMEOUT_MS, &reply);
	if (ret)
		return ret;
	return malibu_cpm_translate_error(reply, false);
}

static int malibu_cpm_reset_assert(struct reset_controller_dev *rcdev, unsigned long id)
{
	return malibu_cpm_reset_set(rcdev, id, 0);
}

static int malibu_cpm_reset_deassert(struct reset_controller_dev *rcdev, unsigned long id)
{
	return malibu_cpm_reset_set(rcdev, id, 1);
}

static const struct reset_control_ops malibu_cpm_reset_ops = {
	.assert = malibu_cpm_reset_assert,
	.deassert = malibu_cpm_reset_deassert,
};

static int malibu_mbfs_request(struct malibu_cpm *cpm, const u32 *payload, u32 *reply)
{
	return malibu_cpm_request(cpm, MALIBU_MBFS_DESTINATION, payload,
				  MALIBU_MBFS_TIMEOUT_MS, 0, reply);
}

static int malibu_mbfs_lookup(struct malibu_cpm *cpm, u32 parent, const char *name, u32 *handle)
{
	char key[MALIBU_MBFS_NAME_SIZE] = {};
	u32 payload[3], reply[3];
	u32 tail_lo, tail_hi;
	int ret;

	if (strscpy(key, name, sizeof(key)) < 0)
		return -EINVAL;

	tail_lo = get_unaligned_le32(key + 8);
	tail_hi = get_unaligned_le32(key + 12);
	payload[0] = MALIBU_MBFS_LOOKUP | parent;
	payload[1] = get_unaligned_le32(key);
	payload[2] = get_unaligned_le32(key + 4);
	ret = malibu_mbfs_request(cpm, payload, reply);
	if (ret)
		return ret;
	if (FIELD_GET(MALIBU_MBFS_STATUS, reply[0]))
		return -ENOENT;
	if (reply[1] != tail_lo || reply[2] != tail_hi) {
		payload[0] = MALIBU_MBFS_LOOKUP_TAIL | FIELD_GET(MALIBU_MBFS_HANDLE, reply[0]);
		payload[1] = tail_lo;
		payload[2] = tail_hi;
		ret = malibu_mbfs_request(cpm, payload, reply);
		if (ret)
			return ret;
		if (FIELD_GET(MALIBU_MBFS_STATUS, reply[0]))
			return -ENOENT;
	}
	*handle = FIELD_GET(MALIBU_MBFS_HANDLE, reply[0]);
	return 0;
}

static int malibu_mbfs_root(struct malibu_cpm *cpm, u32 *root)
{
	static const char * const path[] = { "syspm", "clients", "vm1" };
	u32 payload[3] = {}, reply[3];
	int i, ret;

	ret = malibu_mbfs_request(cpm, payload, reply);
	if (ret)
		return ret;
	if (FIELD_GET(MALIBU_MBFS_STATUS, reply[0]) || !reply[1] || reply[1] > U16_MAX)
		return -ENODEV;

	payload[0] = MALIBU_MBFS_ROOT;
	ret = malibu_mbfs_request(cpm, payload, reply);
	if (ret)
		return ret;
	if (FIELD_GET(MALIBU_MBFS_STATUS, reply[0]))
		return -ENOENT;
	*root = FIELD_GET(MALIBU_MBFS_HANDLE, reply[0]);

	for (i = 0; i < ARRAY_SIZE(path); i++) {
		ret = malibu_mbfs_lookup(cpm, *root, path[i], root);
		if (ret)
			return ret;
	}
	return 0;
}

static int malibu_mbfs_read_domain(struct malibu_cpm_domain *domain, bool *on)
{
	const u32 payload[] = { MALIBU_MBFS_READ | domain->handle, 0, 0 };
	u32 reply[3];
	int ret;

	ret = malibu_mbfs_request(domain->cpm, payload, reply);
	if (ret)
		return ret;
	if (FIELD_GET(MALIBU_MBFS_STATUS, reply[0]))
		return -EREMOTEIO;
	if (reply[2] || reply[1] > 1)
		return -EPROTO;
	*on = reply[1];
	return 0;
}

static int malibu_mbfs_set_domain(struct malibu_cpm_domain *domain, bool on)
{
	unsigned long timeout = msecs_to_jiffies(MALIBU_MBFS_TRANSITION_MS);
	struct malibu_cpm *cpm = domain->cpm;
	const u32 payload[] = { MALIBU_MBFS_WRITE | domain->handle, on, 0 };
	u32 reply[3], status;
	bool actual;
	int ret;

	guard(mutex)(&cpm->transition_lock);
	if (cpm->transition_stalled)
		return -ETIMEDOUT;
	if (READ_ONCE(cpm->remote_reset))
		return -EPROTO;

	ret = malibu_cpm_request(cpm, MALIBU_MBFS_DESTINATION, payload,
				 MALIBU_MBFS_TIMEOUT_MS, domain->id, reply);
	if (!ret) {
		status = FIELD_GET(MALIBU_MBFS_STATUS, reply[0]);
		if (status == MALIBU_MBFS_DEFERRED) {
			if (!wait_for_completion_timeout(&cpm->transition_done, timeout))
				ret = -ETIMEDOUT;
		} else if (status) {
			ret = -EREMOTEIO;
		}
	}
	scoped_guard(spinlock_irqsave, &cpm->response_lock) {
		cpm->awaiting_transition = false;
		if (!ret && cpm->remote_reset)
			ret = -EPROTO;
	}
	if (ret == -ETIMEDOUT)
		cpm->transition_stalled = true;
	if (ret)
		return ret;

	/* A notification carries a resource ID without a transaction token. */
	ret = malibu_mbfs_read_domain(domain, &actual);
	if (!ret && actual != on)
		ret = -EIO;
	if (ret)
		cpm->transition_stalled = true;
	return ret;
}

static int malibu_mbfs_power_on(struct generic_pm_domain *genpd)
{
	return malibu_mbfs_set_domain(container_of(genpd, struct malibu_cpm_domain, genpd), true);
}

static int malibu_mbfs_power_off(struct generic_pm_domain *genpd)
{
	return malibu_mbfs_set_domain(container_of(genpd, struct malibu_cpm_domain, genpd), false);
}

static void malibu_cpm_remove_domain(void *data)
{
	struct malibu_cpm_domain *domain = data;

	WARN_ON(pm_genpd_remove(&domain->genpd));
}

static void malibu_cpm_remove_subdomain(void *data)
{
	struct malibu_cpm_domain *domain = data;

	WARN_ON(pm_genpd_remove_subdomain(&domain->parent->genpd, &domain->genpd));
}

static void malibu_cpm_remove_domains_provider(void *data)
{
	struct malibu_cpm_domains *provider = data;

	if (provider->published)
		of_genpd_del_provider(provider->node);
}

static int malibu_cpm_register_domains(struct malibu_cpm *cpm)
{
	static const struct {
		const char *name;
		u32 id;
		int parent;
		bool active_wakeup;
	} domains[] = {
		[MALIBU_CPM_PD_HSION] = { "hsio_n.v", 0x19, MALIBU_MBFS_NO_PARENT },
		[MALIBU_CPM_PD_USB_TOP] = { "HsioNUsb.v", 0x210, MALIBU_CPM_PD_HSION, true },
		[MALIBU_CPM_PD_USB_PSW] = { "HsioNUsbPsw.v", 0x211, MALIBU_CPM_PD_USB_TOP, true },
		[MALIBU_CPM_PD_LSIOE] = { "lsio_e.v", 0x1d, MALIBU_MBFS_NO_PARENT },
		[MALIBU_CPM_PD_LSIOE_CLI] = { "LsioECli.v", 0x230, MALIBU_CPM_PD_LSIOE },
	};
	struct device *dev = cpm->dev;
	struct malibu_cpm_domains *provider;
	bool initially_on[MALIBU_MBFS_DOMAIN_COUNT];
	unsigned int i;
	u32 root;
	int ret;

	static_assert(ARRAY_SIZE(domains) == MALIBU_MBFS_DOMAIN_COUNT);

	if (!of_property_present(dev->of_node, "#power-domain-cells"))
		return 0;
	provider = devm_kzalloc(dev, sizeof(*provider), GFP_KERNEL);
	if (!provider)
		return -ENOMEM;

	ret = malibu_mbfs_root(cpm, &root);
	if (ret)
		return ret;
	for (i = 0; i < ARRAY_SIZE(domains); i++) {
		struct malibu_cpm_domain *domain = &provider->domain[i];
		int parent = domains[i].parent;

		domain->cpm = cpm;
		domain->id = domains[i].id;
		ret = malibu_mbfs_lookup(cpm, root, domains[i].name, &domain->handle);
		if (ret)
			return ret;
		ret = malibu_mbfs_read_domain(domain, &initially_on[i]);
		if (ret)
			return ret;
		if (parent != MALIBU_MBFS_NO_PARENT && initially_on[i] && !initially_on[parent])
			return -EPROTO;

		domain->genpd.name = domains[i].name;
		domain->genpd.power_on = malibu_mbfs_power_on;
		domain->genpd.power_off = malibu_mbfs_power_off;
		if (domains[i].active_wakeup)
			domain->genpd.flags = GENPD_FLAG_ACTIVE_WAKEUP;
		ret = pm_genpd_init(&domain->genpd, NULL, !initially_on[i]);
		if (ret)
			return ret;
		ret = devm_add_action_or_reset(dev, malibu_cpm_remove_domain, domain);
		if (ret)
			return ret;
		provider->genpds[i] = &domain->genpd;
		if (parent == MALIBU_MBFS_NO_PARENT)
			continue;
		domain->parent = &provider->domain[parent];
		ret = pm_genpd_add_subdomain(&domain->parent->genpd, &domain->genpd);
		if (ret)
			return ret;
		ret = devm_add_action_or_reset(dev, malibu_cpm_remove_subdomain, domain);
		if (ret)
			return ret;
	}
	provider->onecell.domains = provider->genpds;
	provider->onecell.num_domains = ARRAY_SIZE(domains);
	provider->node = dev->of_node;
	ret = devm_add_action_or_reset(dev, malibu_cpm_remove_domains_provider, provider);
	if (ret)
		return ret;
	ret = of_genpd_add_provider_onecell(provider->node, &provider->onecell);
	if (!ret)
		provider->published = true;
	return ret;
}

static int malibu_cpm_register_clocks(struct malibu_cpm *cpm)
{
	static const struct {
		const char *name;
		u8 domain;
		u8 resource;
	} gates[] = {
		[MALIBU_CPM_CLK_USBC_STICKY] = { "malibu-usbc-sticky", MALIBU_CPM_HSION_DOMAIN, 0 },
		[MALIBU_CPM_CLK_USBC_NON_STICKY] = {
			"malibu-usbc-non-sticky", MALIBU_CPM_HSION_DOMAIN, 1
		},
		[MALIBU_CPM_CLK_USB2_PHY] = { "malibu-usb2-phy", MALIBU_CPM_HSION_DOMAIN, 4 },
		[MALIBU_CPM_CLK_USB2_APB] = { "malibu-usb2-apb", MALIBU_CPM_HSION_DOMAIN, 7 },
		[MALIBU_CPM_CLK_I2C3] = { "malibu-i2c3", MALIBU_CPM_LSIOE_DOMAIN, 0x6 },
		[MALIBU_CPM_CLK_I2C3_APB] = { "malibu-i2c3-apb", MALIBU_CPM_LSIOE_DOMAIN, 0x14 },
	};
	struct clk_hw_onecell_data *clocks;
	unsigned int id;

	clocks = devm_kzalloc(cpm->dev, struct_size(clocks, hws, ARRAY_SIZE(gates)),
			      GFP_KERNEL);
	if (!clocks)
		return -ENOMEM;
	clocks->num = ARRAY_SIZE(gates);

	for (id = 0; id < ARRAY_SIZE(gates); id++) {
		struct clk_init_data init = {
			.name = gates[id].name,
			.ops = &malibu_cpm_clock_ops,
			.flags = CLK_GET_RATE_NOCACHE,
		};
		struct malibu_cpm_clock *clock;
		int ret;

		if (!gates[id].name) {
			clocks->hws[id] = ERR_PTR(-ENOENT);
			continue;
		}
		clock = devm_kzalloc(cpm->dev, sizeof(*clock), GFP_KERNEL);
		if (!clock)
			return -ENOMEM;
		clock->cpm = cpm;
		clock->domain = gates[id].domain;
		clock->resource = gates[id].resource;
		clock->hw.init = &init;
		ret = devm_clk_hw_register(cpm->dev, &clock->hw);
		if (ret)
			return ret;
		clocks->hws[id] = &clock->hw;
	}
	if (READ_ONCE(cpm->stalled))
		return -ETIMEDOUT;
	if (READ_ONCE(cpm->remote_reset))
		return -EPROTO;
	return devm_of_clk_add_hw_provider(cpm->dev, of_clk_hw_onecell_get, clocks);
}

static void malibu_cpm_free_channel(void *data)
{
	mbox_free_channel(data);
}

static int malibu_cpm_get_channel(struct mbox_client *client, const char *name,
				  struct mbox_chan **out)
{
	struct mbox_chan *chan = mbox_request_channel_byname(client, name);
	int ret;

	if (IS_ERR(chan))
		return PTR_ERR(chan);
	ret = devm_add_action_or_reset(client->dev, malibu_cpm_free_channel, chan);
	if (!ret)
		*out = chan;
	return ret;
}

static int malibu_cpm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	unsigned long remaining = msecs_to_jiffies(MALIBU_CPM_CLOCK_TIMEOUT_MS);
	struct mbox_chan *rx;
	struct malibu_cpm *cpm;
	int ret;

	cpm = devm_kzalloc(dev, sizeof(*cpm), GFP_KERNEL);
	if (!cpm)
		return -ENOMEM;
	cpm->dev = dev;
	cpm->sequence = 1;
	mutex_init(&cpm->request_lock);
	mutex_init(&cpm->transition_lock);
	spin_lock_init(&cpm->response_lock);
	init_completion(&cpm->tx_done);
	init_completion(&cpm->rx_done);
	init_completion(&cpm->transition_done);
	cpm->rx_client.dev = dev;
	cpm->rx_client.rx_callback = malibu_cpm_rx;
	cpm->tx_client.dev = dev;
	cpm->tx_client.tx_done = malibu_cpm_tx_done;

	ret = malibu_cpm_get_channel(&cpm->rx_client, "rx", &rx);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get CPM response channel\n");
	ret = malibu_cpm_get_channel(&cpm->tx_client, "tx", &cpm->tx);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get CPM request channel\n");

	/* The startup frame completes at TX retirement. */
	cpm->frame[0] = MALIBU_CPM_RESET_FRAME;
	ret = malibu_cpm_transmit(cpm, &remaining, 0);
	if (ret)
		return dev_err_probe(dev, ret, "CPM mailbox handshake failed\n");
	scoped_guard(spinlock_irqsave, &cpm->response_lock)
		cpm->running = true;

	ret = malibu_cpm_register_clocks(cpm);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register CPM clocks\n");

	cpm->reset.ops = &malibu_cpm_reset_ops;
	cpm->reset.owner = THIS_MODULE;
	cpm->reset.of_node = dev->of_node;
	cpm->reset.of_reset_n_cells = 1;
	cpm->reset.of_xlate = malibu_cpm_reset_xlate;
	cpm->reset.nr_resets = ARRAY_SIZE(malibu_cpm_resets);
	ret = devm_reset_controller_register(dev, &cpm->reset);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register CPM resets\n");
	ret = malibu_cpm_register_domains(cpm);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register CPM USB power domains\n");

	ret = malibu_cpm_register_usb2_rails(cpm);
	if (ret)
		dev_warn(dev, "USB2 regulator service unavailable: %pe\n", ERR_PTR(ret));
	return 0;
}

static const struct of_device_id malibu_cpm_of_match[] = {
	{ .compatible = "google,malibu-cpm-io" },
	{ }
};
MODULE_DEVICE_TABLE(of, malibu_cpm_of_match);

static struct platform_driver malibu_cpm_driver = {
	.probe = malibu_cpm_probe,
	.driver = {
		.name = "google-malibu-cpm-io",
		.of_match_table = malibu_cpm_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(malibu_cpm_driver);

MODULE_DESCRIPTION("Google Malibu CPM I/O clock, reset and power domain provider");
MODULE_LICENSE("GPL");
