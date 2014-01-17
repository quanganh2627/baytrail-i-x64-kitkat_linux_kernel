/*
 * platform_byt_audio.c: Baytrail audio platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Omair Md Abudllah <omair.m.abdullah@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sfi.h>
#include <linux/platform_device.h>
#include <asm/intel-mid.h>
#include <asm/platform_sst_audio.h>
#include <asm/platform_byt_audio.h>

static int __init byt_audio_platform_init(void)
{
	struct platform_device *pdev;
	int ret;

	pr_debug("in %s\n", __func__);

	ret = add_sst_platform_device();
	if (ret < 0) {
		pr_err("%s failed to sst_platform device\n", __func__);
		return 0;
	}

	pdev = platform_device_alloc("hdmi-audio", -1);
	if (!pdev) {
		pr_err("failed to allocate hdmi-audio platform device\n");
		return 0;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add hdmi-audio platform device\n");
		platform_device_put(pdev);
		return 0;
	}

	return 0;
}
device_initcall(byt_audio_platform_init);
