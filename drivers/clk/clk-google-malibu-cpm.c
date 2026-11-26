// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/clk-provider.h>
#include <linux/completion.h>
#include <linux/devm-helpers.h>
#include <linux/errno.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/mailbox_client.h>
#include <linux/malibu-cpm.h>
#include <linux/malibu-ufs-psm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
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
#define MALIBU_CPM_HSIOS_DOMAIN		8
#define MALIBU_CPM_LSIOE_DOMAIN		5
#define MALIBU_CPM_CLOCK_TIMEOUT_MS	1000
#define MALIBU_CPM_RESET_TIMEOUT_MS	3000
#define MALIBU_CPM_DISPLAY_DOMAIN	0x23
#define MALIBU_CPM_CLOCK_GET_RATE	3

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
#define MALIBU_MBFS_DOMAIN_COUNT	(MALIBU_CPM_PD_UFS + 1)
#define MALIBU_MBFS_NO_PARENT		-1
#define MALIBU_MBFS_HSIOS_ID		0x1a
#define MALIBU_MBFS_UFS_PREP_ID		0x222
#define MALIBU_MBFS_UFS_CORE_ID		0x220
#define MALIBU_MBFS_UFS_CORE_NAME	"HsioSUfs.v"

enum malibu_ufs_idle_stage {
	MALIBU_UFS_IDLE_WAIT_OFF,
	MALIBU_UFS_IDLE_RELEASED,
	MALIBU_UFS_IDLE_PREP_ON,
	MALIBU_UFS_IDLE_CORE_ON,
	MALIBU_UFS_IDLE_STOPPED,
};

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
	struct malibu_ufs_psm_regs ufs_psm;
	struct delayed_work ufs_observe_work;
	struct device *ufs_idle_prep;
	struct device *ufs_idle_core;
	int ufs_idle_stage;
	u32 frame[4];
	u32 response[3];
	u32 transition_id;
	u32 mbfs_root;
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
	bool trace_ufs_psm;
	bool ufs_observe_late;
	bool ufs_idle_prep_held;
	bool ufs_idle_core_held;
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
	[MALIBU_CPM_RST_UFS_PHY] = { MALIBU_CPM_HSIOS_DOMAIN, 0 },
};

struct malibu_cpm_domain {
	struct generic_pm_domain genpd;
	struct malibu_cpm *cpm;
	struct malibu_cpm_domain *parent;
	u32 id;
	u32 handle;
	bool lazy;
	bool powered_here;
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
#define MALIBU_CPM_RAIL_SET_SELECTOR	4
#define MALIBU_CPM_LDO_19M		0x1f
#define MALIBU_CPM_BUCK_11S		0x33
#define MALIBU_CPM_BUCK_13S		0x35
#define MALIBU_CPM_LDO_1S		0x37
#define MALIBU_CPM_LDO_24S		0x4e

#define MALIBU_CPM_PMIC_WRITE		0x00020000
#define MALIBU_CPM_UFS_VCC_CONTROL	0x1857
#define MALIBU_CPM_UFS_VCCQ_CONTROL	0x1867
#define MALIBU_CPM_DA9189_SID		0x0e

static struct platform_driver malibu_cpm_driver;

static int malibu_cpm_write_ufs_enable(struct malibu_cpm *cpm, u16 reg)
{
	const u32 payload[] = { MALIBU_CPM_PMIC_WRITE | reg, MALIBU_CPM_DA9189_SID, 0 };
	u32 reply[3];
	int ret;

	ret = malibu_cpm_request(cpm, MALIBU_CPM_REGULATOR_DESTINATION, payload,
				 MALIBU_CPM_REGULATOR_TIMEOUT_MS, 0, reply);
	if (ret)
		return ret;

	return reply[0] & MALIBU_CPM_REGULATOR_STATUS ? -EPIPE : 0;
}

/* Apply the stock UFS enable decouple settings to both device rails. */
int malibu_cpm_ufs_decouple_enable(struct device *supplier)
{
	struct malibu_cpm *cpm;
	int ret;

	guard(device)(supplier);
	if (supplier->driver != &malibu_cpm_driver.driver)
		return -EPROBE_DEFER;

	cpm = dev_get_drvdata(supplier);
	if (!cpm)
		return -EPROBE_DEFER;
	ret = malibu_cpm_write_ufs_enable(cpm, MALIBU_CPM_UFS_VCC_CONTROL);
	if (ret)
		return ret;

	return malibu_cpm_write_ufs_enable(cpm, MALIBU_CPM_UFS_VCCQ_CONTROL);
}
EXPORT_SYMBOL_GPL(malibu_cpm_ufs_decouple_enable);

struct malibu_cpm_usb2_rail {
	struct regulator_desc desc;
	u8 voltage_selector;
	bool must_be_on;
};

static int malibu_cpm_rail_request(struct malibu_cpm *cpm, unsigned int id,
				  unsigned int op, u32 argument, u32 *value)
{
	const u32 payload[] = {
		MALIBU_CPM_REGULATOR_REQUEST | (op << 16) | id, argument, 0,
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

static int malibu_cpm_rail_state(struct malibu_cpm *cpm, unsigned int id)
{
	u32 value;
	int ret;

	ret = malibu_cpm_rail_request(cpm, id, MALIBU_CPM_RAIL_GET_ENABLED, 0, &value);
	if (ret)
		return ret;

	/* The status result must be a boolean before exposing it to the core. */
	if (value > 1)
		return -EPROTO;
	return value;
}

static int malibu_cpm_rail_is_enabled(struct regulator_dev *rdev)
{
	return malibu_cpm_rail_state(rdev_get_drvdata(rdev), rdev_get_id(rdev));
}

static int malibu_cpm_usb2_rail_set_enabled(struct regulator_dev *rdev, bool enable)
{
	unsigned int id = rdev_get_id(rdev);

	if (id != MALIBU_CPM_LDO_19M)
		return -EOPNOTSUPP;

	return malibu_cpm_rail_request(rdev_get_drvdata(rdev), id,
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
	.is_enabled = malibu_cpm_rail_is_enabled,
};

static const struct regulator_ops malibu_cpm_usb2_observe_ops = {
	.is_enabled = malibu_cpm_rail_is_enabled,
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

	ret = malibu_cpm_rail_request(cpm, rail->desc.id, MALIBU_CPM_RAIL_GET_SELECTOR, 0,
				      &selector);
	if (ret)
		return ret;
	if (selector != rail->voltage_selector)
		return -ERANGE;

	ret = malibu_cpm_rail_state(cpm, rail->desc.id);
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

static int malibu_cpm_rail_selector(struct malibu_cpm *cpm, unsigned int id)
{
	u32 selector;
	int ret;

	ret = malibu_cpm_rail_request(cpm, id, MALIBU_CPM_RAIL_GET_SELECTOR, 0, &selector);
	if (ret)
		return ret;
	if (selector > U8_MAX)
		return -EPROTO;

	return selector;
}

static int malibu_cpm_ufs_get_voltage_sel(struct regulator_dev *rdev)
{
	return malibu_cpm_rail_selector(rdev_get_drvdata(rdev), rdev_get_id(rdev));
}

void malibu_cpm_ufs_log_rails(struct device *supplier, const char *phase)
{
	static const struct {
		const char *group;
		const char *name;
		unsigned int id;
	} rails[] = {
		{ "analog", "LDO1S", MALIBU_CPM_LDO_1S },
		{ "analog", "LDO24S", MALIBU_CPM_LDO_24S },
		{ "device", "BUCK11S", MALIBU_CPM_BUCK_11S },
		{ "device", "BUCK13S", MALIBU_CPM_BUCK_13S },
	};
	struct malibu_cpm *cpm;
	unsigned int i;

	guard(device)(supplier);
	cpm = dev_get_drvdata(supplier);
	if (supplier->driver != &malibu_cpm_driver.driver || !cpm) {
		dev_info(supplier, "ufs-uic-v38 rails phase=%s unavailable ret=%d\n",
			 phase, -EPROBE_DEFER);
		return;
	}

	/* Use fresh CPM GETs because regulator consumers may cache always-on. */
	for (i = 0; i < ARRAY_SIZE(rails); i++) {
		int enabled = malibu_cpm_rail_state(cpm, rails[i].id);
		int selector = malibu_cpm_rail_selector(cpm, rails[i].id);

		dev_info(supplier, "ufs-uic-v38 %s rail phase=%s name=%s enabled=%d selector=%d\n",
			 rails[i].group, phase, rails[i].name, enabled, selector);
	}
}
EXPORT_SYMBOL_GPL(malibu_cpm_ufs_log_rails);

static int malibu_cpm_ufs_set_voltage_sel(struct regulator_dev *rdev, unsigned int selector)
{
	int id = rdev_get_id(rdev);

	/* The device GPIO switches the rails. Only their D1 voltages are writable. */
	switch (id) {
	case MALIBU_CPM_BUCK_11S:
		if (selector != 170)
			return -EINVAL;
		break;
	case MALIBU_CPM_BUCK_13S:
		if (selector != 192)
			return -EINVAL;
		break;
	default:
		return -EOPNOTSUPP;
	}

	return malibu_cpm_rail_request(rdev_get_drvdata(rdev), id,
				       MALIBU_CPM_RAIL_SET_SELECTOR, selector, NULL);
}

static const struct regulator_ops malibu_cpm_ufs_analog_ops = {
	.is_enabled = malibu_cpm_rail_is_enabled,
	.list_voltage = regulator_list_voltage_linear,
	.get_voltage_sel = malibu_cpm_ufs_get_voltage_sel,
};

static const struct regulator_ops malibu_cpm_ufs_buck_ops = {
	.is_enabled = malibu_cpm_rail_is_enabled,
	.list_voltage = regulator_list_voltage_linear,
	.get_voltage_sel = malibu_cpm_ufs_get_voltage_sel,
	.set_voltage_sel = malibu_cpm_ufs_set_voltage_sel,
};

static int malibu_cpm_ufs_rail_init(struct regulator_dev *rdev, struct regulator_config *config)
{
	/* Apply the board voltage only when the host asks for it. */
	rdev->constraints->apply_uV = false;
	if (rdev->desc->ops->set_voltage_sel)
		rdev->constraints->valid_ops_mask |= REGULATOR_CHANGE_VOLTAGE;

	return 0;
}

#define MALIBU_UFS_RAIL(_name, _match, _id, _min, _step, _type) {		\
	.name = (_name),							\
	.of_match = (_match),						\
	.regulators_node = "ufs-regulators",				\
	.id = (_id),							\
	.min_uV = (_min),							\
	.uV_step = (_step),						\
	.n_voltages = 256,						\
	.init_cb = malibu_cpm_ufs_rail_init,				\
	.type = REGULATOR_VOLTAGE,					\
	.owner = THIS_MODULE,						\
	.ops = &malibu_cpm_ufs_ ## _type ## _ops,				\
}

static const struct regulator_desc malibu_cpm_ufs_rails[] = {
	MALIBU_UFS_RAIL("ldo_1s", "ldo-1s", MALIBU_CPM_LDO_1S, 400000, 5000, analog),
	MALIBU_UFS_RAIL("ldo_24s", "ldo-24s", MALIBU_CPM_LDO_24S, 400000, 5000, analog),
	MALIBU_UFS_RAIL("buck_11s", "buck-11s", MALIBU_CPM_BUCK_11S, 800000, 10000, buck),
	MALIBU_UFS_RAIL("buck_13s", "buck-13s", MALIBU_CPM_BUCK_13S, 240000, 5000, buck),
};

static int malibu_cpm_register_ufs_rails(struct malibu_cpm *cpm)
{
	const struct regulator_config config = {
		.dev = cpm->dev,
		.driver_data = cpm,
	};
	struct device_node *regulators __free(device_node) =
		of_get_child_by_name(cpm->dev->of_node, "ufs-regulators");
	struct regulator_dev *rdev;
	void *group;
	unsigned int i;

	if (!regulators || !of_device_is_available(regulators))
		return 0;

	for (i = 0; i < ARRAY_SIZE(malibu_cpm_ufs_rails); i++) {
		struct device_node *node __free(device_node) =
			of_get_child_by_name(regulators, malibu_cpm_ufs_rails[i].of_match);

		if (!node || !of_device_is_available(node))
			return -EINVAL;
	}

	group = devres_open_group(cpm->dev, NULL, GFP_KERNEL);
	if (!group)
		return -ENOMEM;
	for (i = 0; i < ARRAY_SIZE(malibu_cpm_ufs_rails); i++) {
		rdev = devm_regulator_register(cpm->dev, &malibu_cpm_ufs_rails[i], &config);
		if (IS_ERR(rdev)) {
			devres_release_group(cpm->dev, group);
			return PTR_ERR(rdev);
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

int malibu_cpm_ufs_refclk_sample(struct device *supplier, enum malibu_cpm_ufs_refclk_sample sample,
				const char *phase)
{
	static const struct {
		const char *name;
		u8 selector;
		u32 mask;
	} queries[] = {
		[MALIBU_CPM_UFS_GPIO_REFCLK_GATE] = { "GPIO REFCLK gate", 0, BIT(0) },
		[MALIBU_CPM_UFS_GPIO_REFCLK_ACG] = { "GPIO REFCLK ACG bit", 0, BIT(1) },
		[MALIBU_CPM_UFS_MPHY_REFCLK_GATE] = { "M-PHY REFCLK gate", 1, BIT(0) },
	};
	struct malibu_cpm *cpm;
	u32 command;
	s32 reply;
	int ret;

	if ((unsigned int)sample >= ARRAY_SIZE(queries))
		return -EINVAL;
	guard(device)(supplier);
	cpm = dev_get_drvdata(supplier);
	if (!cpm)
		return -ENODEV;

	/* D1 clock op 2 compares the selected status bits with the request mask. */
	command = malibu_cpm_command(MALIBU_CPM_CLOCK_SERVICE, MALIBU_CPM_HSIOS_DOMAIN,
				     queries[sample].selector, 2);
	ret = malibu_cpm_lpcm_request(cpm, command, queries[sample].mask,
				     MALIBU_CPM_CLOCK_TIMEOUT_MS, &reply);
	if (ret) {
		dev_info(supplier, "ufs-uic-v38 %s phase=%s transport=%d\n",
			 queries[sample].name, phase, ret);
		return ret;
	}
	dev_info(supplier, "ufs-uic-v38 %s phase=%s firmware=%d\n",
		 queries[sample].name, phase, reply);
	return reply == 0 || reply == 1 ? 0 : -EPROTO;
}
EXPORT_SYMBOL_GPL(malibu_cpm_ufs_refclk_sample);

static int malibu_cpm_clock_gate(struct malibu_cpm_clock *clock, unsigned int op)
{
	u32 command = malibu_cpm_command(MALIBU_CPM_CLOCK_SERVICE, clock->domain,
					clock->resource, op);
	s32 reply;
	int ret;

	ret = malibu_cpm_lpcm_request(clock->cpm, command, 1, MALIBU_CPM_CLOCK_TIMEOUT_MS, &reply);
	if (clock->cpm->trace_ufs_psm && clock->domain == MALIBU_CPM_HSIOS_DOMAIN &&
	    !clock->resource) {
		if (ret)
			dev_info(clock->cpm->dev, "ufs-uic-v38 GPIO REFCLK op=%u transport=%d\n",
				 op, ret);
		else
			dev_info(clock->cpm->dev, "ufs-uic-v38 GPIO REFCLK op=%u firmware=%d\n",
				 op, reply);
	}
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

static unsigned long malibu_cpm_ufs_mphy_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	return 38400000;
}

static const struct clk_ops malibu_cpm_ufs_clock_ops = {
	.prepare = malibu_cpm_clock_prepare,
	.unprepare = malibu_cpm_clock_unprepare,
};

static const struct clk_ops malibu_cpm_ufs_mphy_ops = {
	.prepare = malibu_cpm_clock_prepare,
	.unprepare = malibu_cpm_clock_unprepare,
	.recalc_rate = malibu_cpm_ufs_mphy_rate,
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

static int malibu_mbfs_root(struct malibu_cpm *cpm, u32 *root, u32 *namespace_root)
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
	*namespace_root = *root;

	for (i = 0; i < ARRAY_SIZE(path); i++) {
		ret = malibu_mbfs_lookup(cpm, *root, path[i], root);
		if (ret)
			return ret;
	}
	return 0;
}

static bool display_query;
module_param(display_query, bool, 0400);
MODULE_PARM_DESC(display_query, "Log Grizzly display MBFS handles and CPM clock rates");

static void malibu_cpm_query_display(struct malibu_cpm *cpm)
{
	static const char * const domains[] = {
		"dpu.v", "DpuBe.v", "DpuFe0.v", "DpuDsi0.v",
	};
	static const struct {
		const char *name;
		u8 id;
	} clocks[] = {
		{ "dsi0-pixel", 0x01 },
		{ "be", 0x04 },
		{ "fe0", 0x05 },
	};
	unsigned int i;

	dev_info(cpm->dev, "display-cpm: read-only queries start\n");
	for (i = 0; i < ARRAY_SIZE(domains); i++) {
		u32 handle;
		int ret;

		ret = malibu_mbfs_lookup(cpm, cpm->mbfs_root, domains[i], &handle);
		if (ret)
			dev_info(cpm->dev, "display-cpm: MBFS %s error=%d\n",
				 domains[i], ret);
		else
			dev_info(cpm->dev, "display-cpm: MBFS %s handle=0x%x\n",
				 domains[i], handle);
	}
	for (i = 0; i < ARRAY_SIZE(clocks); i++) {
		u32 command = malibu_cpm_command(MALIBU_CPM_CLOCK_SERVICE,
						 MALIBU_CPM_DISPLAY_DOMAIN,
						 clocks[i].id,
						 MALIBU_CPM_CLOCK_GET_RATE);
		s32 reply;
		int ret;

		ret = malibu_cpm_lpcm_request(cpm, command, 0,
					      MALIBU_CPM_CLOCK_TIMEOUT_MS, &reply);
		if (ret)
			dev_info(cpm->dev, "display-cpm: %s command=0x%08x transport=%d\n",
				 clocks[i].name, command, ret);
		else
			dev_info(cpm->dev, "display-cpm: %s command=0x%08x reply=%d\n",
				 clocks[i].name, command, reply);
	}
	dev_info(cpm->dev, "display-cpm: read-only queries complete\n");
}

static int malibu_mbfs_read_value(struct malibu_cpm *cpm, u32 handle, u64 *value)
{
	const u32 payload[] = { MALIBU_MBFS_READ | handle, 0, 0 };
	u32 reply[3];
	int ret;

	ret = malibu_mbfs_request(cpm, payload, reply);
	if (ret)
		return ret;
	if (FIELD_GET(MALIBU_MBFS_STATUS, reply[0]))
		return -EREMOTEIO;
	*value = (u64)reply[2] << 32 | reply[1];
	return 0;
}

static void malibu_cpm_query_display_perf(struct malibu_cpm *cpm, u32 root)
{
	/* Stock paths name the CPM backend before the firmware components. */
	static const char * const path[] = { "perf", "domains" };
	static const char * const domains[] = { "dpu", "fabdisp" };
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(path); i++) {
		ret = malibu_mbfs_lookup(cpm, root, path[i], &root);
		if (ret) {
			dev_info(cpm->dev, "display-perf: path segment %s error=%d\n",
				 path[i], ret);
			return;
		}
	}
	for (i = 0; i < ARRAY_SIZE(domains); i++) {
		u32 domain, opp;
		u64 level;

		ret = malibu_mbfs_lookup(cpm, root, domains[i], &domain);
		if (ret) {
			dev_info(cpm->dev, "display-perf: %s lookup error=%d\n", domains[i], ret);
			continue;
		}
		ret = malibu_mbfs_lookup(cpm, domain, "CurOpp", &opp);
		if (!ret)
			ret = malibu_mbfs_read_value(cpm, opp, &level);
		if (ret)
			dev_info(cpm->dev, "display-perf: %s CurOpp error=%d\n", domains[i], ret);
		else
			dev_info(cpm->dev, "display-perf: %s CurOpp handle=0x%x level=%llu\n",
				 domains[i], opp, level);
	}
}

static int malibu_mbfs_read_vote(struct malibu_cpm *cpm, u32 handle, bool *on)
{
	u64 value;
	int ret;

	ret = malibu_mbfs_read_value(cpm, handle, &value);
	if (ret)
		return ret;
	if (value > 1)
		return -EPROTO;
	*on = value;
	return 0;
}

static int malibu_mbfs_read_domain(struct malibu_cpm_domain *domain, bool *on)
{
	return malibu_mbfs_read_vote(domain->cpm, domain->handle, on);
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

static void malibu_mbfs_log_ufs_vote(struct malibu_cpm_domain *domain,
				     const char *source, int ret)
{
	if (domain->id == MALIBU_MBFS_UFS_PREP_ID || domain->id == MALIBU_MBFS_UFS_CORE_ID ||
	    (domain->id == MALIBU_MBFS_HSIOS_ID && domain->cpm->trace_ufs_psm))
		dev_info(domain->cpm->dev, "ufs-uic-v38 domain=%s vote=%s ret=%d\n",
			 domain->genpd.name, source, ret);
}

static int malibu_mbfs_lazy_on(struct malibu_cpm_domain *domain, const char **source)
{
	struct malibu_cpm *cpm = domain->cpm;
	bool on;
	int ret;

	/* Leave domains inherited from the bootloader in their original state. */
	*source = "lookup-error";
	ret = malibu_mbfs_lookup(cpm, cpm->mbfs_root, domain->genpd.name, &domain->handle);
	if (ret)
		return ret;
	*source = "read-error";
	ret = malibu_mbfs_read_domain(domain, &on);
	if (ret)
		return ret;
	*source = "inherited";
	if (on)
		return 0;

	ret = malibu_mbfs_set_domain(domain, true);
	if (!ret)
		domain->powered_here = true;
	*source = ret ? "request-error" : "issued";
	return ret;
}

static bool malibu_ufs_idle_diag(struct malibu_cpm *cpm)
{
	return IS_ENABLED(CONFIG_COMMON_CLK_GOOGLE_MALIBU_CPM_UFS_IDLE_DIAG) &&
	       cpm->trace_ufs_psm &&
	       (!IS_ENABLED(CONFIG_SCSI_UFS_MALIBU) ||
		IS_ENABLED(CONFIG_COMMON_CLK_GOOGLE_MALIBU_CPM_UFS_PREP_RELEASE));
}

int malibu_cpm_ufs_check_idle_stage(struct device *supplier, bool prep_resumed)
{
	struct malibu_cpm *cpm;
	int expected = prep_resumed ? MALIBU_UFS_IDLE_PREP_ON : MALIBU_UFS_IDLE_RELEASED;
	int stage;

	guard(device)(supplier);
	if (supplier->driver != &malibu_cpm_driver.driver)
		return -EPROBE_DEFER;
	cpm = dev_get_drvdata(supplier);
	if (!cpm)
		return -EPROBE_DEFER;
	if (!malibu_ufs_idle_diag(cpm))
		return -ENODEV;
	if (prep_resumed)
		stage = READ_ONCE(cpm->ufs_idle_stage);
	else
		read_poll_timeout(READ_ONCE, stage, stage == expected, 1000, 200000,
				  false, cpm->ufs_idle_stage);
	dev_info(supplier, "ufs-uic-v43 host gate prep_resumed=%d stage=%d expected=%d\n",
		 prep_resumed, stage, expected);
	if (!prep_resumed && stage != expected)
		return -EPROBE_DEFER;

	return stage == expected ? 0 : -EAGAIN;
}

static bool malibu_ufs_idle_sample(struct malibu_cpm *cpm, const char *phase,
				   unsigned int psm1)
{
	struct malibu_ufs_psm_sample sample;

	malibu_ufs_sample_psms(&cpm->ufs_psm, &sample);
	malibu_ufs_log_psms(cpm->dev, phase, &sample);
	return sample.children_valid && (sample.hc & 0x1f) == (0x10 | psm1) &&
	       (sample.phy & 0x1f) == 0x11;
}

static int malibu_ufs_idle_power_on(struct malibu_cpm_domain *domain)
{
	struct malibu_cpm *cpm = domain->cpm;
	bool prep = domain->id == MALIBU_MBFS_UFS_PREP_ID;
	bool on;
	int expected = prep ? MALIBU_UFS_IDLE_RELEASED : MALIBU_UFS_IDLE_PREP_ON;
	const char *source = "blocked";
	int ret = -EAGAIN;

	if (cmpxchg(&cpm->ufs_idle_stage, expected, MALIBU_UFS_IDLE_STOPPED) != expected)
		goto out;

	source = "lookup-error";
	ret = malibu_mbfs_lookup(cpm, cpm->mbfs_root, domain->genpd.name, &domain->handle);
	if (ret)
		goto out;
	source = "read-error";
	ret = malibu_mbfs_read_domain(domain, &on);
	if (ret)
		goto out;
	if (on) {
		source = "inherited";
		ret = -EBUSY;
		goto out;
	}

	source = "blocked";
	ret = -EAGAIN;
	if (!malibu_ufs_idle_sample(cpm, prep ? "ufs-uic-v40 prep-on-before" :
				    "ufs-uic-v40 core-on-before", prep ? 3 : 2))
		goto out;

	ret = malibu_mbfs_set_domain(domain, true);
	source = ret ? "request-error" : "issued";
	if (ret)
		goto out;
	domain->powered_here = true;

	if (prep) {
		if (!malibu_ufs_idle_sample(cpm, "ufs-uic-v40 prep-on-after", 2)) {
			/* The owned vote is on, so report ON to genpd and block core. */
			dev_info(cpm->dev, "ufs-uic-v40 prep did not reach PSM1=2\n");
			goto out;
		}
		WRITE_ONCE(cpm->ufs_idle_stage, MALIBU_UFS_IDLE_PREP_ON);
	} else {
		struct malibu_ufs_psm_sample sample;

		malibu_ufs_sample_psms(&cpm->ufs_psm, &sample);
		malibu_ufs_log_psms(cpm->dev, "ufs-uic-v40 core-on-after", &sample);
		WRITE_ONCE(cpm->ufs_idle_stage, MALIBU_UFS_IDLE_CORE_ON);
	}
out:
	dev_info(cpm->dev, "ufs-uic-v40 domain=%s on=%s owned=%d ret=%d\n",
		 domain->genpd.name, source, domain->powered_here, ret);
	return ret;
}

static int malibu_ufs_prep_off_status(struct malibu_cpm *cpm,
				      struct malibu_ufs_psm_sample *sample,
				      bool *saw_hc_transition)
{
	malibu_ufs_sample_psms(&cpm->ufs_psm, sample);
	if (!sample->children_valid)
		return -EHOSTDOWN;
	if ((sample->phy & 0x1f) != 0x11)
		return -EIO;

	/* D1 sampled 0x32 on the way from 0x10 to 0x13 after prep OFF. */
	if (sample->hc == 0x32) {
		*saw_hc_transition = true;
		return -EINPROGRESS;
	}

	switch (sample->hc & 0x1f) {
	case 0x13:
		return 0;
	case 0x10:
		return -EINPROGRESS;
	default:
		return -EIO;
	}
}

static bool malibu_ufs_wait_prep_off(struct malibu_cpm *cpm)
{
	struct malibu_ufs_psm_sample initial = {}, final;
	bool saw_hc_transition = false;
	ktime_t start = ktime_get();
	s64 elapsed_us;
	int status, ret = 0;

	status = malibu_ufs_prep_off_status(cpm, &initial, &saw_hc_transition);
	final = initial;
	if (status == -EINPROGRESS)
		ret = read_poll_timeout(malibu_ufs_prep_off_status, status,
					status != -EINPROGRESS, 20, 10000,
					true, cpm, &final, &saw_hc_transition);
	elapsed_us = ktime_us_delta(ktime_get(), start);
	malibu_ufs_log_psms(cpm->dev, "ufs-uic-v45 prep-off-initial", &initial);
	malibu_ufs_log_psms(cpm->dev, "ufs-uic-v40 prep-off-after", &final);
	dev_info(cpm->dev, "ufs-uic-v45 prep OFF settling elapsed_us=%lld qualification=%d\n",
		 elapsed_us, ret ?: status);

	dev_info(cpm->dev, "ufs-uic-v52 prep OFF saw_hc_0x32=%u\n", saw_hc_transition);

	return !ret && !status;
}

static int malibu_ufs_idle_power_off(struct malibu_cpm_domain *domain)
{
	struct malibu_cpm *cpm = domain->cpm;
	bool core_on;
	bool owned = domain->powered_here;
	u32 handle;
	int ret = -EBUSY;

	if (domain->id != MALIBU_MBFS_UFS_PREP_ID ||
	    cmpxchg(&cpm->ufs_idle_stage, MALIBU_UFS_IDLE_WAIT_OFF,
		    MALIBU_UFS_IDLE_STOPPED) != MALIBU_UFS_IDLE_WAIT_OFF ||
	    !owned)
		goto out;

	ret = malibu_mbfs_lookup(cpm, cpm->mbfs_root, MALIBU_MBFS_UFS_CORE_NAME, &handle);
	if (ret)
		goto out;
	ret = malibu_mbfs_read_vote(cpm, handle, &core_on);
	if (ret)
		goto out;
	dev_info(cpm->dev, "ufs-uic-v40 pre-idle core client vote=%d\n", core_on);
	if (core_on || !malibu_ufs_idle_sample(cpm, "ufs-uic-v40 prep-off-before", 0)) {
		ret = -EBUSY;
		goto out;
	}

	ret = malibu_mbfs_set_domain(domain, false);
	if (ret)
		goto out;
	domain->powered_here = false;
	if (malibu_ufs_wait_prep_off(cpm))
		WRITE_ONCE(cpm->ufs_idle_stage, MALIBU_UFS_IDLE_RELEASED);
out:
	dev_info(cpm->dev, "ufs-uic-v40 domain=%s off owned_before=%d owned_after=%d ret=%d\n",
		 domain->genpd.name, owned, domain->powered_here, ret);
	return ret;
}

static int malibu_mbfs_power_on(struct generic_pm_domain *genpd)
{
	struct malibu_cpm_domain *domain = container_of(genpd, struct malibu_cpm_domain, genpd);
	struct malibu_cpm *cpm = domain->cpm;
	struct malibu_ufs_psm_sample before, after;
	bool trace_hsios = domain->id == MALIBU_MBFS_HSIOS_ID && cpm->trace_ufs_psm;
	const char *source;
	int ret;

	if (malibu_ufs_idle_diag(cpm) &&
	    (domain->id == MALIBU_MBFS_UFS_PREP_ID || domain->id == MALIBU_MBFS_UFS_CORE_ID))
		return malibu_ufs_idle_power_on(domain);

	if (!domain->lazy)
		return malibu_mbfs_set_domain(domain, true);

	if (trace_hsios)
		malibu_ufs_sample_psms(&cpm->ufs_psm, &before);
	ret = malibu_mbfs_lazy_on(domain, &source);
	if (trace_hsios) {
		if (!ret)
			malibu_ufs_sample_psms(&cpm->ufs_psm, &after);
		malibu_ufs_log_psms(cpm->dev, "hsios-power-before", &before);
		if (!ret)
			malibu_ufs_log_psms(cpm->dev, "hsios-power-after", &after);
		else
			dev_info(cpm->dev, "ufs-uic-v38 hsios-power-after PSM skipped ret=%d\n",
				 ret);
	}
	malibu_mbfs_log_ufs_vote(domain, source, ret);
	return ret;
}

static int malibu_mbfs_initial_ufs_vote(struct malibu_cpm_domain *domain)
{
	struct malibu_cpm *cpm = domain->cpm;
	struct malibu_ufs_psm_sample sample;
	const char *name = domain->genpd.name;
	const char *source;
	char phase[64];
	int ret;

	malibu_ufs_sample_psms(&cpm->ufs_psm, &sample);
	snprintf(phase, sizeof(phase), "v39-initial-before domain=%s", name);
	malibu_ufs_log_psms(cpm->dev, phase, &sample);
	if (domain->id != MALIBU_MBFS_HSIOS_ID && !sample.children_valid) {
		dev_info(cpm->dev, "ufs-uic-v39 domain=%s vote=skipped wrapper-off\n", name);
		return -EHOSTDOWN;
	}
	if (malibu_ufs_idle_diag(cpm) && domain->id == MALIBU_MBFS_UFS_PREP_ID &&
	    ((sample.hc & 0x1f) != 0x10 || (sample.phy & 0x1f) != 0x11)) {
		dev_info(cpm->dev, "ufs-uic-v40 prep seed skipped outside handoff state\n");
		return -EAGAIN;
	}

	ret = malibu_mbfs_lazy_on(domain, &source);
	dev_info(cpm->dev, "ufs-uic-v39 domain=%s initial-vote=%s ret=%d\n", name, source, ret);
	if (!ret) {
		malibu_ufs_sample_psms(&cpm->ufs_psm, &sample);
		snprintf(phase, sizeof(phase), "v39-initial-after domain=%s", name);
		malibu_ufs_log_psms(cpm->dev, phase, &sample);
	}
	return ret;
}

static int malibu_mbfs_power_off(struct generic_pm_domain *genpd)
{
	struct malibu_cpm_domain *domain = container_of(genpd, struct malibu_cpm_domain, genpd);
	int ret;

	if (malibu_ufs_idle_diag(domain->cpm) &&
	    (domain->id == MALIBU_MBFS_UFS_PREP_ID || domain->id == MALIBU_MBFS_UFS_CORE_ID))
		return malibu_ufs_idle_power_off(domain);

	if (domain->lazy && !domain->powered_here)
		return 0;

	ret = malibu_mbfs_set_domain(domain, false);
	if (!ret)
		domain->powered_here = false;
	return ret;
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
		bool lazy;
	} domains[] = {
		[MALIBU_CPM_PD_HSION] = { "hsio_n.v", 0x19, MALIBU_MBFS_NO_PARENT },
		[MALIBU_CPM_PD_USB_TOP] = { "HsioNUsb.v", 0x210, MALIBU_CPM_PD_HSION, true },
		[MALIBU_CPM_PD_USB_PSW] = { "HsioNUsbPsw.v", 0x211, MALIBU_CPM_PD_USB_TOP, true },
		[MALIBU_CPM_PD_LSIOE] = { "lsio_e.v", 0x1d, MALIBU_MBFS_NO_PARENT },
		[MALIBU_CPM_PD_LSIOE_CLI] = { "LsioECli.v", 0x230, MALIBU_CPM_PD_LSIOE },
		[MALIBU_CPM_PD_HSIOS] = {
			"hsio_s.v", MALIBU_MBFS_HSIOS_ID, MALIBU_MBFS_NO_PARENT, .lazy = true,
		},
		[MALIBU_CPM_PD_UFS_PREP] = {
			"HsioSUfsPrep.v", MALIBU_MBFS_UFS_PREP_ID, MALIBU_CPM_PD_HSIOS,
			.lazy = true,
		},
		[MALIBU_CPM_PD_UFS] = {
			MALIBU_MBFS_UFS_CORE_NAME, MALIBU_MBFS_UFS_CORE_ID, MALIBU_CPM_PD_UFS_PREP,
			.lazy = true,
		},
	};
	struct device *dev = cpm->dev;
	struct malibu_cpm_domains *provider;
	bool initial_ufs_votes = IS_ENABLED(CONFIG_COMMON_CLK_GOOGLE_MALIBU_CPM_UFS_VOTE_DIAG) &&
				 cpm->trace_ufs_psm &&
				 (!IS_ENABLED(CONFIG_SCSI_UFS_MALIBU) || malibu_ufs_idle_diag(cpm));
	const bool prep_release = IS_ENABLED(CONFIG_COMMON_CLK_GOOGLE_MALIBU_CPM_UFS_PREP_RELEASE);
	bool initially_on[MALIBU_MBFS_DOMAIN_COUNT];
	u32 namespace_root;
	unsigned int i;
	int ret;

	static_assert(ARRAY_SIZE(domains) == MALIBU_MBFS_DOMAIN_COUNT);

	if (!of_property_present(dev->of_node, "#power-domain-cells"))
		return 0;
	provider = devm_kzalloc(dev, sizeof(*provider), GFP_KERNEL);
	if (!provider)
		return -ENOMEM;

	ret = malibu_mbfs_root(cpm, &cpm->mbfs_root, &namespace_root);
	if (ret)
		return ret;
	for (i = 0; i < ARRAY_SIZE(domains); i++) {
		struct malibu_cpm_domain *domain = &provider->domain[i];
		int parent = domains[i].parent;

		domain->cpm = cpm;
		domain->id = domains[i].id;
		domain->lazy = domains[i].lazy;
		domain->genpd.name = domains[i].name;
		/* Optional domains perform their first lookup at power-on. */
		initially_on[i] = false;
		if (!domain->lazy) {
			ret = malibu_mbfs_lookup(cpm, cpm->mbfs_root, domains[i].name,
						 &domain->handle);
			if (ret)
				return ret;
			ret = malibu_mbfs_read_domain(domain, &initially_on[i]);
			if (ret)
				return ret;
		} else if (initial_ufs_votes &&
			   (i == MALIBU_CPM_PD_HSIOS || i == MALIBU_CPM_PD_UFS_PREP ||
			    (i == MALIBU_CPM_PD_UFS && !malibu_ufs_idle_diag(cpm)))) {
			ret = malibu_mbfs_initial_ufs_vote(domain);
			if (ret) {
				dev_warn(dev, "ufs-uic-v39 initial vote sequence stopped at %s: %pe\n",
					 domain->genpd.name, ERR_PTR(ret));
				initial_ufs_votes = false;
			} else {
				initially_on[i] = true;
				if (!malibu_ufs_idle_diag(cpm) || i == MALIBU_CPM_PD_HSIOS)
					domain->genpd.flags |= GENPD_FLAG_ALWAYS_ON;
				else if (prep_release && i == MALIBU_CPM_PD_UFS_PREP &&
					 domain->powered_here)
					domain->genpd.flags |= GENPD_FLAG_NO_STAY_ON;
			}
		}
		if (parent != MALIBU_MBFS_NO_PARENT && initially_on[i] && !initially_on[parent])
			return -EPROTO;

		domain->genpd.power_on = malibu_mbfs_power_on;
		domain->genpd.power_off = malibu_mbfs_power_off;
		if (domains[i].active_wakeup)
			domain->genpd.flags |= GENPD_FLAG_ACTIVE_WAKEUP;
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
	if (ret)
		return ret;

	provider->published = true;
	if (display_query && of_machine_is_compatible("google,malibu-grizzly")) {
		malibu_cpm_query_display(cpm);
		malibu_cpm_query_display_perf(cpm, namespace_root);
	}
	return 0;
}

static int malibu_cpm_register_clocks(struct malibu_cpm *cpm)
{
	static const struct {
		const char *name;
		u8 domain;
		u8 resource;
		const struct clk_ops *ops;
	} gates[] = {
		[MALIBU_CPM_CLK_USBC_STICKY] = { "malibu-usbc-sticky", MALIBU_CPM_HSION_DOMAIN, 0 },
		[MALIBU_CPM_CLK_USBC_NON_STICKY] = {
			"malibu-usbc-non-sticky", MALIBU_CPM_HSION_DOMAIN, 1
		},
		[MALIBU_CPM_CLK_USB2_PHY] = { "malibu-usb2-phy", MALIBU_CPM_HSION_DOMAIN, 4 },
		[MALIBU_CPM_CLK_USB2_APB] = { "malibu-usb2-apb", MALIBU_CPM_HSION_DOMAIN, 7 },
		[MALIBU_CPM_CLK_I2C3] = { "malibu-i2c3", MALIBU_CPM_LSIOE_DOMAIN, 0x6 },
		[MALIBU_CPM_CLK_I2C3_APB] = { "malibu-i2c3-apb", MALIBU_CPM_LSIOE_DOMAIN, 0x14 },
		[MALIBU_CPM_CLK_UFS_GPIO_REF] = {
			"malibu-ufs-gpio-ref", MALIBU_CPM_HSIOS_DOMAIN, 0,
			&malibu_cpm_ufs_clock_ops,
		},
		[MALIBU_CPM_CLK_UFS_MPHY_REF] = {
			"malibu-ufs-mphy-ref", MALIBU_CPM_HSIOS_DOMAIN, 1, &malibu_cpm_ufs_mphy_ops,
		},
		[MALIBU_CPM_CLK_UFS_CORE] = {
			"malibu-ufs-core", MALIBU_CPM_HSIOS_DOMAIN, 3, &malibu_cpm_ufs_clock_ops,
		},
		[MALIBU_CPM_CLK_UFS_DEEP_H8] = {
			"malibu-ufs-deep-h8", MALIBU_CPM_HSIOS_DOMAIN, 4, &malibu_cpm_ufs_clock_ops,
		},
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
			.ops = gates[id].ops ?: &malibu_cpm_clock_ops,
			.flags = gates[id].ops ? 0 : CLK_GET_RATE_NOCACHE,
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

static void malibu_cpm_log_ufs_psms(struct malibu_cpm *cpm, const char *phase)
{
	struct malibu_ufs_psm_sample sample;

	malibu_ufs_sample_psms(&cpm->ufs_psm, &sample);
	malibu_ufs_log_psms(cpm->dev, phase, &sample);
}

static void malibu_cpm_idle_resume(struct malibu_cpm *cpm)
{
	struct platform_device *ufs;
	struct device_node *node;
	struct device *prep, *core;
	int ret;

	ret = READ_ONCE(cpm->ufs_idle_stage);
	if (ret != MALIBU_UFS_IDLE_RELEASED) {
		dev_info(cpm->dev, "ufs-uic-v40 idle release not qualified stage=%d\n", ret);
		return;
	}

	node = of_find_node_by_path("ufs0");
	ufs = node ? of_find_device_by_node(node) : NULL;
	of_node_put(node);
	if (!ufs) {
		dev_info(cpm->dev, "ufs-uic-v40 UFS domain consumer unavailable\n");
		return;
	}
	prep = dev_pm_domain_attach_by_name(&ufs->dev, "prep");
	core = dev_pm_domain_attach_by_name(&ufs->dev, "core");
	put_device(&ufs->dev);
	if (!IS_ERR_OR_NULL(prep))
		cpm->ufs_idle_prep = prep;
	if (!IS_ERR_OR_NULL(core))
		cpm->ufs_idle_core = core;
	if (!cpm->ufs_idle_prep || !cpm->ufs_idle_core) {
		dev_info(cpm->dev, "ufs-uic-v40 could not attach prep and core\n");
		return;
	}

	dev_info(cpm->dev, "ufs-uic-v40 genpd before resume prep_on=%d core_on=%d\n",
		 dev_pm_genpd_is_on(prep), dev_pm_genpd_is_on(core));
	if (dev_pm_genpd_is_on(prep) || dev_pm_genpd_is_on(core))
		return;

	ret = pm_runtime_resume_and_get(prep);
	cpm->ufs_idle_prep_held = ret >= 0;
	dev_info(cpm->dev, "ufs-uic-v40 prep resume ret=%d genpd_on=%d\n",
		 ret, dev_pm_genpd_is_on(prep));
	if (ret < 0 || READ_ONCE(cpm->ufs_idle_stage) != MALIBU_UFS_IDLE_PREP_ON)
		return;

	ret = pm_runtime_resume_and_get(core);
	cpm->ufs_idle_core_held = ret >= 0;
	dev_info(cpm->dev, "ufs-uic-v40 core resume ret=%d genpd_on=%d\n",
		 ret, dev_pm_genpd_is_on(core));
}

static void malibu_cpm_stop_idle_diag(void *data)
{
	struct malibu_cpm *cpm = data;

	cancel_delayed_work_sync(&cpm->ufs_observe_work);
	WRITE_ONCE(cpm->ufs_idle_stage, MALIBU_UFS_IDLE_STOPPED);
	if (cpm->ufs_idle_core) {
		if (cpm->ufs_idle_core_held)
			pm_runtime_put_noidle(cpm->ufs_idle_core);
		dev_pm_domain_detach(cpm->ufs_idle_core, false);
	}
	if (cpm->ufs_idle_prep) {
		if (cpm->ufs_idle_prep_held)
			pm_runtime_put_noidle(cpm->ufs_idle_prep);
		dev_pm_domain_detach(cpm->ufs_idle_prep, false);
	}
}

static void malibu_cpm_observe_ufs(struct work_struct *work)
{
	struct malibu_cpm *cpm = container_of(to_delayed_work(work), struct malibu_cpm,
					     ufs_observe_work);

	if (cpm->ufs_observe_late) {
		malibu_cpm_log_ufs_psms(cpm, "cpm-observe-20s");
		return;
	}

	malibu_cpm_log_ufs_psms(cpm, "cpm-observe-2s");
	if (malibu_ufs_idle_diag(cpm) && !IS_ENABLED(CONFIG_SCSI_UFS_MALIBU))
		malibu_cpm_idle_resume(cpm);
	cpm->ufs_observe_late = true;
	schedule_delayed_work(&cpm->ufs_observe_work, msecs_to_jiffies(18000));
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

	cpm->trace_ufs_psm = malibu_ufs_map_psms(dev, &cpm->ufs_psm);
	if (cpm->trace_ufs_psm)
		malibu_cpm_log_ufs_psms(cpm, "cpm-before-handshake");

	/* The startup frame completes at TX retirement. */
	cpm->frame[0] = MALIBU_CPM_RESET_FRAME;
	ret = malibu_cpm_transmit(cpm, &remaining, 0);
	if (ret)
		return dev_err_probe(dev, ret, "CPM mailbox handshake failed\n");
	if (cpm->trace_ufs_psm)
		malibu_cpm_log_ufs_psms(cpm, "cpm-after-handshake");
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
	ret = malibu_cpm_register_ufs_rails(cpm);
	if (ret)
		dev_warn(dev, "UFS regulator service unavailable: %pe\n", ERR_PTR(ret));
	platform_set_drvdata(pdev, cpm);
	if (cpm->trace_ufs_psm &&
	    (!IS_ENABLED(CONFIG_SCSI_UFS_MALIBU) || malibu_ufs_idle_diag(cpm))) {
		if (malibu_ufs_idle_diag(cpm)) {
			INIT_DELAYED_WORK(&cpm->ufs_observe_work, malibu_cpm_observe_ufs);
			ret = devm_add_action_or_reset(dev, malibu_cpm_stop_idle_diag, cpm);
		} else {
			ret = devm_delayed_work_autocancel(dev, &cpm->ufs_observe_work,
							   malibu_cpm_observe_ufs);
		}
		if (ret)
			dev_warn(dev, "ufs-uic-v38 failed to register delayed PSM samples: %pe\n",
				 ERR_PTR(ret));
		else
			schedule_delayed_work(&cpm->ufs_observe_work, msecs_to_jiffies(2000));
	}
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
