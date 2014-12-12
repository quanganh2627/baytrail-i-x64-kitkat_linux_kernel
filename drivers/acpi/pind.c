/*
 *  pind.c - ACPI PIND (Platform Indicator Device) Driver
 *  Copyright (c) 2014, Intel Corporation.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or (at
 *  your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/string.h>
#include <acpi/acpi_bus.h>
#include <acpi/acpi_drivers.h>


#define ACPI_PIND_HID			"PNP0A06"
#define ACPI_PIND_DEVICE_NAME		"Plat Ind"
#define ACPI_PIND_NOTIFY		0x80
#define ACPI_PIND_TYPE_NOTIFY		0x02
#define PIND_MEMBER_SIZE		0x02
#define PIND_TYPE_PACKAGE_COUNT		0x01

ACPI_MODULE_NAME("pind");

MODULE_AUTHOR("Jagadish Krishnamoorthy");
MODULE_DESCRIPTION("ACPI PIND Driver");
MODULE_LICENSE("GPL");

static const struct acpi_device_id pind_device_ids[] = {
	{ACPI_PIND_HID,    0},
	{"", 0},
};
MODULE_DEVICE_TABLE(acpi, pind_device_ids);

static int acpi_pind_add(struct acpi_device *device);
static int acpi_pind_remove(struct acpi_device *device);
static void acpi_pind_notify(struct acpi_device *device, u32 event);

static struct acpi_driver acpi_pind_driver = {
	.name = "pind",
	.ids = pind_device_ids,
	.ops = {
		.add = acpi_pind_add,
		.remove = acpi_pind_remove,
		.notify = acpi_pind_notify,
	},
};

struct notify_package {
	u8 index;
	u8 key_type;
	u8 key_code;
	struct list_head list;
	u8 pressed:1;
};

struct acpi_pind {
	u8 count;
	struct acpi_device *acpi_dev;
	struct input_dev *input;
	struct list_head list;
};

static void acpi_pind_notify(struct acpi_device *device, u32 event)
{
	struct notify_package *obj;
	struct acpi_pind *pind = acpi_driver_data(device);

	/* ACPI_PIND_NOTIFY is an index to the first
	  (zero based) entry in the _IND package */
	event ^= ACPI_PIND_NOTIFY;
	list_for_each_entry(obj, &pind->list, list) {
		if (obj->index == event) {
			input_event(pind->input, obj->key_type, obj->key_code,
				!(obj->pressed));
			input_sync(pind->input);
			/* Toggle the press and release events manually till
			   we get the events in notify call */
			obj->pressed = !obj->pressed;
		}
	}
}

static void add_notify_list(int key_type, int key_code, struct acpi_pind *pind)
{
	struct notify_package *obj = NULL;

	input_set_capability(pind->input,
		key_type, key_code);
	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (NULL != obj) {
		obj->index = pind->count;
		obj->key_type = key_type;
		obj->key_code = key_code;
		list_add_tail(&obj->list, &pind->list);
	}
}

static void type_notify_handler(char *name, struct acpi_pind *pind)
{
	if (!strcmp("VOLUME_UP", name)) {
		__set_bit(EV_REP, pind->input->evbit);
		add_notify_list(EV_KEY, KEY_VOLUMEUP, pind);
	} else if (!strcmp("VOLUME_DOWN", name)) {
		__set_bit(EV_REP, pind->input->evbit);
		add_notify_list(EV_KEY, KEY_VOLUMEDOWN, pind);
	} else if (!strcmp("HOME_BUTTON", name)) {
		__set_bit(EV_REP, pind->input->evbit);
		add_notify_list(EV_KEY, KEY_HOME, pind);
	} else
		pr_warn("%s PIND Device %s Unsupported\n", __func__, name);
}

/* Acpi pind package for an element is represented as
   Package (2) {"VOLUME_UP",   Package (1) {0x02 }}
   Presently supporting type 0x02
   0x02  - interrupt event that is not listed in CRS table. This interrupt is delivered
   through a notify () event where the notify event value is a pointer
*/
static int extract_pind_package(struct acpi_buffer *buffer, struct acpi_pind *pind)
{
	int i;
	union acpi_object *pdat;
	union acpi_object *member;
	union acpi_object *type_name;
	union acpi_object *type_package;
	union acpi_object *type;

	pdat = buffer->pointer;
	if (pdat->type !=  ACPI_TYPE_PACKAGE)
		return -EINVAL;
	for (i = 0; i < pdat->package.count; i++) {
		member = &pdat->package.elements[i];
		if (member->package.count == PIND_MEMBER_SIZE) {
			type_name = &member->package.elements[0];
			type_package = &member->package.elements[1];
			if (type_package->package.count == PIND_TYPE_PACKAGE_COUNT) {
				type = &type_package->package.elements[0];
				switch (type->integer.value) {
				case ACPI_PIND_TYPE_NOTIFY:
					type_notify_handler(type_name->string.pointer, pind);
					break;
				default:
					pr_err("%s Type Unsupported\n", __func__);
					return -EINVAL;
				}
			} else {
				pr_err("%s TBD: Type package count more than 1\n", __func__);
				return -EINVAL;
			}
		} else {
			pr_err("%sElement package count lesser than expected\n", __func__);
			return -EINVAL;
		}
		pind->count++;
	}
	return 0;
}

static int acpi_pind_add(struct acpi_device *device)
{
	int error;
	char *name;
	acpi_status status = 0;
	struct acpi_pind *pind;
	const char *hid = acpi_device_hid(device);
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	struct notify_package *obj;

	pind = kzalloc(sizeof(struct acpi_pind), GFP_KERNEL);
	if (!pind)
		return -ENOMEM;
	pind->acpi_dev = device;
	device->driver_data = pind;
	pind->input = input_allocate_device();
	if (!pind->input) {
		error = -ENOMEM;
		goto err_free_pind;
	}
	name = acpi_device_name(device);
	strcpy(name, ACPI_PIND_DEVICE_NAME);

	pind->input->name = name;
	pind->input->phys = "buttons/input0";
	pind->input->id.bustype = BUS_HOST;
	pind->input->dev.parent = &device->dev;

	INIT_LIST_HEAD(&pind->list);
	status = acpi_evaluate_object(device->handle, "_IND",
				NULL, &buffer);
	if (ACPI_FAILURE(status)) {
		pr_err("%s _IND failure\n", __func__);
		error = -ENODEV;
		goto err_free_acpi_buffer;
	}
	error = extract_pind_package(&buffer, pind);
	if (error) {
		pr_err("%s Package parsing error\n", __func__);
		goto err_free_list;
	}
	error = input_register_device(pind->input);
	if (error)
		goto err_free_list;

	kfree(buffer.pointer);
	return 0;

err_free_list:
	list_for_each_entry(obj, &pind->list, list)
		kfree(obj);
err_free_acpi_buffer:
	input_free_device(pind->input);
	kfree(buffer.pointer);
err_free_pind:
	kfree(pind);

	return error;
}

static int acpi_pind_remove(struct acpi_device *device)
{
	struct notify_package *obj;
	struct acpi_pind *pind = acpi_driver_data(device);

	list_for_each_entry(obj, &pind->list, list)
		kfree(obj);
	input_unregister_device(pind->input);
	kfree(pind);
	return 0;
}

module_acpi_driver(acpi_pind_driver);
