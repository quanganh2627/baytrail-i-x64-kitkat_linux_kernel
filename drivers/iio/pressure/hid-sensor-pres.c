/*
 * HID Sensors Driver
 * Copyright (c) 2014, Intel Corporation.
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
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/hid-sensor-hub.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include "../common/hid-sensors/hid-sensor-trigger.h"

/*Format: HID-SENSOR-usage_id_in_hex*/
/*Usage ID from spec for Pressure: 0x200031*/
#define DRIVER_NAME "HID-SENSOR-200031"

#define CHANNEL_SCAN_INDEX_PRES 0

struct pres_state {
	struct hid_sensor_hub_callbacks callbacks;
	struct hid_sensor_common common_attributes;
	struct hid_sensor_hub_attribute_info pres_attri_info;
	u32 pres_data;
};

/* Channel definitions */
static const struct iio_chan_spec pres_channels[] = {
	{
		.type = IIO_PRESSURE,
		.modified = 0,
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_OFFSET) |
		BIT(IIO_CHAN_INFO_SCALE) |
		BIT(IIO_CHAN_INFO_SAMP_FREQ) |
		BIT(IIO_CHAN_INFO_HYSTERESIS),
		.scan_index = CHANNEL_SCAN_INDEX_PRES,
	}
};

/* Adjust channel real bits based on report descriptor */
static void pres_adjust_channel_bit_mask(struct iio_chan_spec *channels,
					int channel, int size)
{
	channels[channel].scan_type.sign = 'u';
	/* Real storage bits will change based on the report desc. */
	channels[channel].scan_type.realbits = size * 8;
	/* Maximum size of a sample to capture is u32 */
	channels[channel].scan_type.storagebits = sizeof(u32) * 8;
}

/* Channel read_raw handler */
static int pres_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2,
			      long mask)
{
	struct pres_state *pres_state = iio_priv(indio_dev);
	int report_id = -1;
	u32 address;
	int ret;
	int ret_type;

	*val = 0;
	*val2 = 0;
	switch (mask) {
	case 0:
		switch (chan->scan_index) {
		case  CHANNEL_SCAN_INDEX_PRES:
			report_id = pres_state->pres_attri_info.report_id;
			address =
			HID_USAGE_SENSOR_DATA_ENVIRONMENTAL_ATMOSPHERIC;
			break;
		default:
			report_id = -1;
			break;
		}
		if (report_id >= 0)
			*val = sensor_hub_input_attr_get_raw_value(
				pres_state->common_attributes.hsdev,
				HID_USAGE_SENSOR_PRESSURE, address,
				report_id);
		else {
			*val = 0;
			return -EINVAL;
		}
		ret_type = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_SCALE:
		*val = pres_state->pres_attri_info.units;
		ret_type = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_OFFSET:
		*val = hid_sensor_convert_exponent(
				pres_state->pres_attri_info.unit_expo);
		ret_type = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = hid_sensor_read_samp_freq_value(
				&pres_state->common_attributes, val, val2);
		ret_type = IIO_VAL_INT_PLUS_MICRO;
		break;
	case IIO_CHAN_INFO_HYSTERESIS:
		ret = hid_sensor_read_raw_hyst_value(
				&pres_state->common_attributes, val, val2);
		ret_type = IIO_VAL_INT_PLUS_MICRO;
		break;
	default:
		ret_type = -EINVAL;
		break;
	}

	return ret_type;
}

/* Channel write_raw handler */
static int pres_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct pres_state *pres_state = iio_priv(indio_dev);
	int ret = 0;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = hid_sensor_write_samp_freq_value(
				&pres_state->common_attributes, val, val2);
		break;
	case IIO_CHAN_INFO_HYSTERESIS:
		ret = hid_sensor_write_raw_hyst_value(
				&pres_state->common_attributes, val, val2);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int pres_write_raw_get_fmt(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       long mask)
{
	return IIO_VAL_INT_PLUS_MICRO;
}

static const struct iio_info pres_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &pres_read_raw,
	.write_raw = &pres_write_raw,
	.write_raw_get_fmt = &pres_write_raw_get_fmt,
};

/* Function to push data to buffer */
static void hid_sensor_push_data(struct iio_dev *indio_dev, u8 *data, int len)
{
	dev_dbg(&indio_dev->dev, "hid_sensor_push_data\n");
	iio_push_to_buffers(indio_dev, (u8 *)data);
}

/* Callback handler to send event after all samples are received and captured */
static int pres_proc_event(struct hid_sensor_hub_device *hsdev,
				unsigned usage_id,
				void *priv)
{
	struct iio_dev *indio_dev = platform_get_drvdata(priv);
	struct pres_state *pres_state = iio_priv(indio_dev);

	dev_dbg(&indio_dev->dev, "pres_proc_event [%d]\n",
				pres_state->common_attributes.data_ready);
	if (pres_state->common_attributes.data_ready)
		hid_sensor_push_data(indio_dev,
				(u8 *)&pres_state->pres_data,
				sizeof(pres_state->pres_data));

	return 0;
}

/* Capture samples in local storage */
static int pres_capture_sample(struct hid_sensor_hub_device *hsdev,
				unsigned usage_id,
				size_t raw_len, char *raw_data,
				void *priv)
{
	struct iio_dev *indio_dev = platform_get_drvdata(priv);
	struct pres_state *pres_state = iio_priv(indio_dev);
	int ret = -EINVAL;

	switch (usage_id) {
	case HID_USAGE_SENSOR_DATA_ENVIRONMENTAL_ATMOSPHERIC:
		pres_state->pres_data = *(u32 *)raw_data;
		ret = 0;
		break;
	default:
		break;
	}

	return ret;
}

/* Parse report which is specific to an usage id*/
static int pres_parse_report(struct platform_device *pdev,
				struct hid_sensor_hub_device *hsdev,
				struct iio_chan_spec *channels,
				unsigned usage_id,
				struct pres_state *st)
{
	int ret;

	ret = sensor_hub_input_get_attribute_info(hsdev, HID_INPUT_REPORT,
			usage_id,
			HID_USAGE_SENSOR_DATA_ENVIRONMENTAL_ATMOSPHERIC,
			&st->pres_attri_info);
	if (ret < 0)
		return ret;
	pres_adjust_channel_bit_mask(channels, CHANNEL_SCAN_INDEX_PRES,
					st->pres_attri_info.size);

	dev_dbg(&pdev->dev, "pres %x:%x\n", st->pres_attri_info.index,
			st->pres_attri_info.report_id);

	return ret;
}

/* Function to initialize the processing for usage id */
static int hid_pres_probe(struct platform_device *pdev)
{
	int ret = 0;
	static const char *name = "pressure";
	struct iio_dev *indio_dev;
	struct pres_state *pres_state;
	struct hid_sensor_hub_device *hsdev = pdev->dev.platform_data;
	struct iio_chan_spec *channels;

	indio_dev = iio_device_alloc(sizeof(struct pres_state));
	if (indio_dev == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}
	platform_set_drvdata(pdev, indio_dev);

	pres_state = iio_priv(indio_dev);
	pres_state->common_attributes.hsdev = hsdev;
	pres_state->common_attributes.pdev = pdev;

	ret = hid_sensor_parse_common_attributes(hsdev, HID_USAGE_SENSOR_PRESSURE,
					&pres_state->common_attributes);
	if (ret) {
		dev_err(&pdev->dev, "failed to setup common attributes\n");
		goto error_free_dev;
	}

	channels = kmemdup(pres_channels, sizeof(pres_channels), GFP_KERNEL);
	if (!channels) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "failed to duplicate channels\n");
		goto error_free_dev;
	}

	if (pres_state->common_attributes.sensitivity.report_id == -1) {
		sensor_hub_input_get_attribute_info(hsdev,
		HID_FEATURE_REPORT,
		HID_USAGE_SENSOR_PRESSURE,
		HID_USAGE_SENSOR_DATA_ENVIRONMENTAL_ATMOSPHERIC |
		HID_USAGE_SENSOR_MODIFIER_CHG_SENSITIVITY_ABS,
		&pres_state->common_attributes.sensitivity);
	}

	ret = pres_parse_report(pdev, hsdev, channels,
				HID_USAGE_SENSOR_PRESSURE, pres_state);
	if (ret) {
		dev_err(&pdev->dev, "failed to setup attributes\n");
		goto error_free_dev_mem;
	}

	indio_dev->channels = channels;
	indio_dev->num_channels =
				ARRAY_SIZE(pres_channels);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &pres_info;
	indio_dev->name = name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = iio_triggered_buffer_setup(indio_dev, &iio_pollfunc_store_time,
		NULL, NULL);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialize trigger buffer\n");
		goto error_free_dev_mem;
	}
	pres_state->common_attributes.data_ready = false;
	ret = hid_sensor_setup_trigger(indio_dev, name,
				&pres_state->common_attributes);
	if (ret < 0) {
		dev_err(&pdev->dev, "trigger setup failed\n");
		goto error_unreg_buffer_funcs;
	}

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&pdev->dev, "device register failed\n");
		goto error_remove_trigger;
	}

	pres_state->callbacks.send_event = pres_proc_event;
	pres_state->callbacks.capture_sample = pres_capture_sample;
	pres_state->callbacks.pdev = pdev;
	ret = sensor_hub_register_callback(hsdev, HID_USAGE_SENSOR_PRESSURE,
					&pres_state->callbacks);
	if (ret < 0) {
		dev_err(&pdev->dev, "callback reg failed\n");
		goto error_iio_unreg;
	}

	return ret;

error_iio_unreg:
	iio_device_unregister(indio_dev);
error_remove_trigger:
	hid_sensor_remove_trigger(indio_dev);
error_unreg_buffer_funcs:
	iio_triggered_buffer_cleanup(indio_dev);
error_free_dev_mem:
	kfree(indio_dev->channels);
error_free_dev:
	iio_device_free(indio_dev);
error_ret:
	return ret;
}

/* Function to deinitialize the processing for usage id */
static int hid_pres_remove(struct platform_device *pdev)
{
	struct hid_sensor_hub_device *hsdev = pdev->dev.platform_data;
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	sensor_hub_remove_callback(hsdev, HID_USAGE_SENSOR_PRESSURE);
	iio_device_unregister(indio_dev);
	hid_sensor_remove_trigger(indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);
	kfree(indio_dev->channels);
	iio_device_free(indio_dev);

	return 0;
}

static struct platform_driver hid_pres_platform_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= hid_pres_probe,
	.remove		= hid_pres_remove,
};
module_platform_driver(hid_pres_platform_driver);

MODULE_DESCRIPTION("HID Sensor Pressure");
MODULE_AUTHOR("Jie Chen <jie.y.chen@intel.com>");
MODULE_LICENSE("GPL");
