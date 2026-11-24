// SPDX-License-Identifier: GPL-2.0-only

#include <linux/cleanup.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/spmi.h>
#include <linux/usb/role.h>
#include <linux/usb/tcpci.h>
#include <linux/usb/tcpm.h>

#include "tcpci_maxim.h"

struct max77779_spmi {
	struct device *dev;
	struct tcpci_data data;
	struct tcpci *tcpci;
	struct usb_role_switch *session;
};

static int max77779_spmi_disconnect(struct max77779_spmi *chip)
{
	int ret, gate_ret;

	ret = usb_role_switch_set_role(chip->session, USB_ROLE_NONE);
	gate_ret = regmap_write(chip->data.regmap, TCPC_VENDOR_USBSW_CTRL,
				TCPC_VENDOR_USBSW_CTRL_DISABLE_USB_DATA);
	return ret ?: gate_ret;
}

static int max77779_spmi_set_roles(struct tcpci *tcpci, struct tcpci_data *data,
				   bool attached, enum typec_role role,
				   enum typec_data_role data_role)
{
	struct max77779_spmi *chip = container_of(data, struct max77779_spmi, data);
	unsigned int value;
	int ret;

	if (!attached)
		return max77779_spmi_disconnect(chip);
	if (role != TYPEC_SINK || data_role != TYPEC_DEVICE)
		return -EOPNOTSUPP;

	ret = regmap_read(data->regmap, TCPC_POWER_STATUS, &value);
	if (ret)
		goto disconnect;
	if ((value & (TCPC_POWER_STATUS_UNINIT | TCPC_POWER_STATUS_DBG_ACC_CON |
		      TCPC_POWER_STATUS_VBUS_PRES)) !=
	    TCPC_POWER_STATUS_VBUS_PRES) {
		ret = -ENOLINK;
		goto disconnect;
	}

	ret = regmap_write(data->regmap, TCPC_VENDOR_USBSW_CTRL,
			   TCPC_VENDOR_USBSW_CTRL_ENABLE_USB_DATA);
	if (ret)
		goto disconnect;
	ret = regmap_read(data->regmap, TCPC_VENDOR_USBSW_CTRL, &value);
	if (ret)
		goto disconnect;
	if (value != TCPC_VENDOR_USBSW_CTRL_ENABLE_USB_DATA) {
		ret = -EIO;
		goto disconnect;
	}

	ret = usb_role_switch_set_role(chip->session, USB_ROLE_DEVICE);
	if (!ret)
		return 0;

disconnect:
	if (max77779_spmi_disconnect(chip))
		dev_warn(chip->dev, "failed to fully disconnect the USB2 session\n");
	return ret;
}

static int max77779_spmi_set_vbus(struct tcpci *tcpci, struct tcpci_data *data,
				  bool source, bool sink)
{
	/* Bypass TCPCI sourcing and charging control for this data-only sink. */
	return source ? -EOPNOTSUPP : 1;
}

static int max77779_spmi_init(struct tcpci *tcpci, struct tcpci_data *data)
{
	int ret;

	ret = regmap_write(data->regmap, TCPC_RX_DETECT, 0);
	if (ret)
		return ret;
	ret = regmap_update_bits(data->regmap, TCPC_POWER_CTRL,
				 TCPC_FAST_ROLE_SWAP_EN, 0);
	if (ret)
		return ret;
	return regmap_write(data->regmap, TCPC_ALERT_EXTENDED_MASK, 0);
}

static irqreturn_t max77779_spmi_irq(int irq, void *data)
{
	struct max77779_spmi *chip = data;
	struct tcpm_port *port = tcpci_get_tcpm_port(chip->tcpci);
	struct regmap *regmap = chip->data.regmap;
	__le16 status;
	unsigned int mask;
	int ret;

	ret = regmap_raw_read(regmap, TCPC_ALERT, &status, sizeof(status));
	if (ret) {
		tcpm_vbus_change(port);
		return IRQ_NONE;
	}
	if (!status)
		return IRQ_NONE;

	/* Acknowledge any stale PD events without reading a PD buffer. */
	ret = regmap_raw_write(regmap, TCPC_ALERT, &status, sizeof(status));
	if (ret) {
		tcpm_vbus_change(port);
		return IRQ_HANDLED;
	}
	if (status & cpu_to_le16(TCPC_ALERT_CC_STATUS))
		tcpm_cc_change(port);
	if (status & cpu_to_le16(TCPC_ALERT_POWER_STATUS)) {
		ret = regmap_read(regmap, TCPC_POWER_STATUS_MASK, &mask);
		if (!ret && mask == 0xff)
			tcpm_tcpc_reset(port);
		else
			tcpm_vbus_change(port);
	}
	return IRQ_HANDLED;
}

static int max77779_spmi_check_connector(struct device *dev)
{
	struct fwnode_handle *connector __free(fwnode_handle) =
		device_get_named_child_node(dev, "connector");
	struct fwnode_handle *endpoint __free(fwnode_handle) = NULL;
	const char *role;

	if (!connector || !fwnode_device_is_compatible(connector, "usb-c-connector") ||
	    !fwnode_property_read_bool(connector, "pd-disable") ||
	    fwnode_property_read_bool(connector, "self-powered"))
		return -EINVAL;
	endpoint = fwnode_graph_get_next_endpoint(connector, NULL);
	if (endpoint)
		return -EINVAL;
	if (fwnode_property_read_string(connector, "power-role", &role) ||
	    strcmp(role, "sink"))
		return -EINVAL;
	if (fwnode_property_read_string(connector, "data-role", &role) ||
	    strcmp(role, "device"))
		return -EINVAL;
	if (fwnode_property_read_string(connector, "typec-power-opmode", &role) ||
	    strcmp(role, "default"))
		return -EINVAL;
	return 0;
}

static void max77779_spmi_put_session(void *data)
{
	usb_role_switch_put(data);
}

static void max77779_spmi_shutdown_session(void *data)
{
	struct max77779_spmi *chip = data;
	const __le16 mask = 0;

	if (regmap_raw_write(chip->data.regmap, TCPC_ALERT_MASK, &mask, sizeof(mask)))
		dev_warn(chip->dev, "failed to mask Type-C alerts on removal\n");
	if (max77779_spmi_disconnect(chip))
		dev_warn(chip->dev, "failed to fully disconnect on removal\n");
}

static void max77779_spmi_unregister(void *data)
{
	tcpci_unregister_port(data);
}

static const struct regmap_config max77779_spmi_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x97,
};

static int max77779_spmi_probe(struct spmi_device *sdev)
{
	struct device *dev = &sdev->dev;
	struct device_node *session __free(device_node) = NULL;
	struct device *supplier __free(put_device) = NULL;
	struct device_link *link;
	struct max77779_spmi *chip;
	__le16 mask = 0;
	int irq, ret;

	ret = max77779_spmi_check_connector(dev);
	if (ret)
		return dev_err_probe(dev, ret, "expected a USB2 sink without PD\n");
	irq = of_irq_get(dev->of_node, 0);
	if (irq <= 0)
		return dev_err_probe(dev, irq ?: -EINVAL, "failed to get alert IRQ\n");
	session = of_parse_phandle(dev->of_node, "maxim,usb2-role-switch", 0);
	if (!session || !of_device_is_compatible(session, "google,malibu-dwc3"))
		return dev_err_probe(dev, -EINVAL, "USB2 role switch missing\n");
	supplier = bus_find_device_by_fwnode(&platform_bus_type, of_fwnode_handle(session));
	if (!supplier)
		return -EPROBE_DEFER;
	link = device_link_add(dev, supplier, DL_FLAG_AUTOREMOVE_CONSUMER | DL_FLAG_PM_RUNTIME);
	if (!link)
		return dev_err_probe(dev, -EINVAL, "failed to link the USB2 role provider\n");
	if (READ_ONCE(link->status) != DL_STATE_CONSUMER_PROBE)
		return -EPROBE_DEFER;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->dev = dev;
	chip->session = usb_role_switch_find_by_fwnode(of_fwnode_handle(session));
	if (!chip->session)
		return dev_err_probe(dev, -EPROBE_DEFER, "USB2 role switch not ready\n");
	ret = devm_add_action_or_reset(dev, max77779_spmi_put_session, chip->session);
	if (ret)
		return ret;

	chip->data.regmap = devm_regmap_init_spmi_ext(sdev, &max77779_spmi_regmap_config);
	if (IS_ERR(chip->data.regmap))
		return PTR_ERR(chip->data.regmap);
	ret = regmap_raw_write(chip->data.regmap, TCPC_ALERT_MASK, &mask, sizeof(mask));
	if (ret)
		return dev_err_probe(dev, ret, "failed to mask Type-C alerts\n");
	ret = devm_add_action_or_reset(dev, max77779_spmi_shutdown_session, chip);
	if (ret)
		return ret;
	ret = max77779_spmi_disconnect(chip);
	if (ret)
		return dev_err_probe(dev, ret, "failed to start with USB2 disconnected\n");

	chip->data.skip_pd = true;
	chip->data.init = max77779_spmi_init;
	chip->data.set_vbus = max77779_spmi_set_vbus;
	chip->data.set_roles = max77779_spmi_set_roles;
	chip->tcpci = tcpci_register_port(dev, &chip->data);
	if (IS_ERR(chip->tcpci))
		return dev_err_probe(dev, PTR_ERR(chip->tcpci), "failed to register TCPM port\n");
	ret = devm_add_action_or_reset(dev, max77779_spmi_unregister, chip->tcpci);
	if (ret)
		return ret;
	return devm_request_threaded_irq(dev, irq, NULL, max77779_spmi_irq,
					 IRQF_TRIGGER_LOW | IRQF_ONESHOT, dev_name(dev), chip);
}

static const struct of_device_id max77779_spmi_of_match[] = {
	{ .compatible = "maxim,max77779-tcpc-spmi" },
	{ }
};
MODULE_DEVICE_TABLE(of, max77779_spmi_of_match);

static struct spmi_driver max77779_spmi_driver = {
	.driver = {
		.name = "max77779-tcpc-spmi",
		.of_match_table = max77779_spmi_of_match,
	},
	.probe = max77779_spmi_probe,
};
module_spmi_driver(max77779_spmi_driver);

MODULE_DESCRIPTION("Maxim MAX77779 SPMI USB Type-C sink controller");
MODULE_LICENSE("GPL");
