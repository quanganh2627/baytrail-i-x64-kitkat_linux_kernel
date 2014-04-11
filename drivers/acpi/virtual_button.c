/*
 * Virtual button driver to handle non acpi button devices.
 * Copyright (c) 2013, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <acpi/acpi_bus.h>
#include <acpi/acpi_drivers.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/input.h>

#include "internal.h"

/* Non acpi button codes */
#define BYT_EC_SCI_VOLUMEUP_BTN         0x80    /* SCI from vol up button */
#define BYT_EC_SCI_VOLUMEDOWN_BTN       0x81    /* SCI from vol down button */
#define BYT_EC_SCI_HOME_BTN             0x85    /* SCI from home button */
#define BYT_EC_SCI_POWER_BTN            0x86    /* SCI from power button */
#define BYT_EC_SCI_RESUME		0x55	/* SCI from Resuming from S3 */
#define VOL_UP_MASK			(1 << 0)
#define VOL_DOWN_MASK			(1 << 1)
#define VOL_HOME_MASK			(1 << 2)
#define POWER_MASK			(1 << 3)

#define	EC_S3_WAKEUP_STATUS		0xB7
#define	PWRBTN_HID_WAKE			0x01
#define	PWRBTN_HID_CLEAR		0x0

struct virtual_button {
	u8 key_press;
	struct input_dev *input;
};

struct virtual_keys_button {
	unsigned int code;      /* input event code */
	unsigned int type;      /* input event type */
	const char *desc;
	int active_low;
	int wakeup;
};

struct virtual_keys_platform_data {
	struct virtual_keys_button *buttons;
	int nbuttons;
};

static struct platform_device *virtual_key_pdev;
static struct virtual_button *priv;

static struct virtual_keys_button virtual_buttons[] = {
	{ KEY_VOLUMEUP,		EV_KEY, "Volume_up", 1 },
	{ KEY_VOLUMEDOWN,	EV_KEY, "Volume_down", 1 },
	{ KEY_HOME,		EV_KEY, "Home_btn", 1 },
	{ KEY_POWER,		EV_KEY, "Power_btn", 1 },
};

static struct virtual_keys_platform_data virtual_key_pdata = {
	.buttons = virtual_buttons,
	.nbuttons = 4,
};

static void power_button_work(int event)
{
	u8 val;
	int ret;

	ret = ec_read(EC_S3_WAKEUP_STATUS, &val);
	if (ret)
		pr_err("%s: ec read fail\n", __func__);
	else if (val & PWRBTN_HID_WAKE) {
		if (event == BYT_EC_SCI_RESUME) {
			input_event(priv->input, EV_KEY,
				KEY_POWER, 1);
			input_sync(priv->input);
			input_event(priv->input, EV_KEY,
				KEY_POWER, 0);
			input_sync(priv->input);
		}
		ret = ec_write(EC_S3_WAKEUP_STATUS, PWRBTN_HID_CLEAR);
		if (ret)
			pr_err("%s: ec write fail\n", __func__);
	}
}

static int button_event_handler(void *data)
{
	int event;
	event = (int *)data;
	switch (event) {
	case BYT_EC_SCI_VOLUMEUP_BTN:
		input_event(priv->input, EV_KEY,
			KEY_VOLUMEUP, !(priv->key_press & VOL_UP_MASK));
		priv->key_press ^= VOL_UP_MASK;
		break;
	case BYT_EC_SCI_VOLUMEDOWN_BTN:
		input_event(priv->input, EV_KEY,
			KEY_VOLUMEDOWN, !(priv->key_press & VOL_DOWN_MASK));
		priv->key_press ^= VOL_DOWN_MASK;
		break;
	case BYT_EC_SCI_HOME_BTN:
		input_event(priv->input, EV_KEY,
			KEY_HOME, !(priv->key_press & VOL_HOME_MASK));
		priv->key_press ^= VOL_HOME_MASK;
		break;
	case BYT_EC_SCI_POWER_BTN:
		input_event(priv->input, EV_KEY,
			KEY_POWER, !(priv->key_press & POWER_MASK));
		priv->key_press ^= POWER_MASK;
		acpi_clear_event(ACPI_EVENT_POWER_BUTTON);
		power_button_work(BYT_EC_SCI_POWER_BTN);
		break;
	case BYT_EC_SCI_RESUME:
		power_button_work(BYT_EC_SCI_RESUME);
		return 0;
	default:
		pr_err("%s: non acpi button unhandled event\n", __func__);
		return 0;
	}
	input_sync(priv->input);
	return 0;
}

typedef int (*acpi_ec_query_func) (void *data);
extern int acpi_ec_add_query_handler(struct acpi_ec *ec, u8 query_bit,
				acpi_handle handle, acpi_ec_query_func func,
				void *data);

static int virtual_buttons_probe(struct platform_device *pdev)
{
	int i;
	int ret;
	struct virtual_keys_platform_data *pdata = pdev->dev.platform_data;
	struct virtual_keys_button *button;

	if (pdata == NULL) {
		dev_err(&pdev->dev, "No virtual button platform data\n");
		return -EINVAL;
	}

	priv = kzalloc(sizeof(struct virtual_button), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->input = input_allocate_device();
	if (!priv->input)
		return -ENOMEM;

	priv->input->name = pdev->name;
	priv->input->phys = "buttons/input0";
	priv->input->id.bustype = BUS_HOST;
	priv->input->dev.parent = &pdev->dev;
	__set_bit(EV_REP, priv->input->evbit);

	for (i = 0; i < pdata->nbuttons; i++) {
		button = &pdata->buttons[i];
		input_set_capability(priv->input,
			EV_KEY, button->code);
	}

	ret = input_register_device(priv->input);
	if (ret) {
		dev_err(&pdev->dev,
			"unable to register input dev, error %d\n", ret);
		goto err;
	}
	/* first_ec is defined in acpi ec driver */
	if (first_ec == NULL) {
		dev_err(&pdev->dev, "acpi ec ptr not found\n");
		goto err_ec;
	}
	/* Register for volume and home buttons */
	acpi_ec_add_query_handler(first_ec, BYT_EC_SCI_VOLUMEUP_BTN, NULL,
		button_event_handler, BYT_EC_SCI_VOLUMEUP_BTN);
	acpi_ec_add_query_handler(first_ec, BYT_EC_SCI_VOLUMEDOWN_BTN, NULL,
		button_event_handler, BYT_EC_SCI_VOLUMEDOWN_BTN);
	acpi_ec_add_query_handler(first_ec, BYT_EC_SCI_HOME_BTN, NULL,
		button_event_handler, BYT_EC_SCI_HOME_BTN);
#ifdef CONFIG_ACPI_VIRTUAL_POWER_BUTTON
	acpi_ec_add_query_handler(first_ec, BYT_EC_SCI_POWER_BTN, NULL,
		button_event_handler, BYT_EC_SCI_POWER_BTN);
	acpi_ec_add_query_handler(first_ec, BYT_EC_SCI_RESUME, NULL,
		button_event_handler, BYT_EC_SCI_RESUME);
#endif

	dev_info(&pdev->dev, "Probed %s device\n", pdev->name);
	return 0;

err:
	input_free_device(priv->input);
	goto exit;
err_ec:
	input_unregister_device(priv->input);
	goto exit;
exit:
	kfree(priv);
	return ret;
}

static int virtual_buttons_remove(struct platform_device *pdev)
{
	input_unregister_device(priv->input);
	kfree(priv);
	return 0;
}

static const struct platform_device_id virtual_buttons_table[] = {
	{"non_acpi_btns", 1},
};

static struct platform_driver virtual_buttons_driver = {
	.driver = {
		.name = "virtual_buttons",
		.owner = THIS_MODULE,
	},
	.probe  = virtual_buttons_probe,
	.remove = virtual_buttons_remove,
	.id_table = virtual_buttons_table,
};

static int __init virtual_buttons_module_init(void)
{
	int ret = 0;
	char *name = "non_acpi_btns";
	/* Register platform device and driver for non acpi button */
	virtual_key_pdev = platform_device_alloc(name, -1);
	if (!virtual_key_pdev) {
		pr_err("out of memory for platform dev %s\n", name);
		return ret;
	}
	virtual_key_pdev->dev.platform_data =
		&virtual_key_pdata;
	ret = platform_device_add(virtual_key_pdev);
	if (ret) {
		pr_err("failed to add %s platform device\n", name);
		platform_device_put(virtual_key_pdev);
		return ret;
	}
	return platform_driver_register(&virtual_buttons_driver);
}

static void __exit virtual_buttons_module_exit(void)
{
	platform_driver_unregister(&virtual_buttons_driver);
	platform_device_del(virtual_key_pdev);
}

device_initcall(virtual_buttons_module_init);
module_exit(virtual_buttons_module_exit);

MODULE_AUTHOR("Jagadish Krishnamoorthy<jagadish.krishnamoorthy@intel.com>");
MODULE_LICENSE("GPL");
