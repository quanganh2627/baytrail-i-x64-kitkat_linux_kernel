/*
 * mrst.c: Intel Moorestown platform specific setup code
 *
 * (C) Copyright 2008 Intel Corporation
 * Author: Jacob Pan (jacob.jun.pan@intel.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mfd/intel_msic.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/intel_mid_pm.h>
#include <linux/usb/penwell_otg.h>

#include <asm/setup.h>
#include <asm/mpspec_def.h>
#include <asm/hw_irq.h>
#include <asm/apic.h>
#include <asm/io_apic.h>
#include <asm/intel-mid.h>
#include <asm/mrst-vrtc.h>
#include <asm/io.h>
#include <asm/i8259.h>
#include <asm/intel_scu_ipc.h>
#include <asm/apb_timer.h>
#include <asm/intel_mid_gpadc.h>
#include <asm/reboot.h>

void intel_mid_power_off(void)
{
	intel_scu_ipc_simple_command(IPCMSG_COLD_RESET, 1);
}

unsigned long __init intel_mid_calibrate_tsc(void)
{
	unsigned long flags, fast_calibrate;

	local_irq_save(flags);
	fast_calibrate = apbt_quick_calibrate();
	local_irq_restore(flags);

	if (fast_calibrate)
		return fast_calibrate;

	return 0;
}
