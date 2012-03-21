/*
 * platform_max17042.c: max17042 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/lnw_gpio.h>
#include <linux/power_supply.h>
#include <linux/power/max17042_battery.h>
#include <linux/power/intel_mdf_battery.h>
#include <linux/power/bq24192_charger.h>
#include <asm/intel-mid.h>
#include <asm/delay.h>
#include "platform_max17042.h"

void max17042_i2c_reset_workaround(void)
{
/* toggle clock pin of I2C-1 to recover devices from abnormal status.
 * currently, only max17042 on I2C-1 needs such workaround */
#define I2C_1_GPIO_PIN 27
	lnw_gpio_set_alt(I2C_1_GPIO_PIN, LNW_GPIO);
	gpio_direction_output(I2C_1_GPIO_PIN, 0);
	gpio_set_value(I2C_1_GPIO_PIN, 1);
	udelay(10);
	gpio_set_value(I2C_1_GPIO_PIN, 0);
	udelay(10);
	lnw_gpio_set_alt(I2C_1_GPIO_PIN, LNW_ALT_1);
#undef I2C_1_GPIO_PIN 27
}
EXPORT_SYMBOL(max17042_i2c_reset_workaround);

static bool msic_battery_check(void)
{
	if (get_oem0_table() == NULL) {
		pr_info("invalid battery detected\n");
		return false;
	} else {
		pr_info("valid battery detected\n");
		return true;
	}
	return false;
}


void *max17042_platform_data(void *info)
{
	static struct max17042_platform_data platform_data;
	struct i2c_board_info *i2c_info = (struct i2c_board_info *)info;
#ifdef CONFIG_BATTERY_MAX17050
	int intr = get_gpio_by_name("max17050");
#else
	int intr = get_gpio_by_name("max17042");
#endif

	i2c_info->irq = intr + INTEL_MID_IRQ_OFFSET;

	if (msic_battery_check()) {
		platform_data.enable_current_sense = true;
		platform_data.technology = POWER_SUPPLY_TECHNOLOGY_LION;
	} else {
		platform_data.enable_current_sense = false;
		platform_data.technology = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
	}

	platform_data.is_init_done = 0;
	platform_data.reset_i2c_lines = max17042_i2c_reset_workaround;
#ifdef CONFIG_BATTERY_INTEL_MDF
	platform_data.current_sense_enabled =
		intel_msic_is_current_sense_enabled;
	platform_data.battery_present = intel_msic_check_battery_present;
	platform_data.battery_health = intel_msic_check_battery_health;
	platform_data.battery_status = intel_msic_check_battery_status;
	platform_data.battery_pack_temp = intel_msic_get_battery_pack_temp;
	platform_data.save_config_data = intel_msic_save_config_data;
	platform_data.restore_config_data = intel_msic_restore_config_data;

	platform_data.is_cap_shutdown_enabled =
					intel_msic_is_capacity_shutdown_en;
	platform_data.is_volt_shutdown_enabled = intel_msic_is_volt_shutdown_en;
	platform_data.is_lowbatt_shutdown_enabled =
					intel_msic_is_lowbatt_shutdown_en;
	platform_data.get_vmin_threshold = intel_msic_get_vsys_min;
#endif
#ifdef CONFIG_CHARGER_BQ24192
	platform_data.battery_status = bq24192_query_battery_status;
#endif

	return &platform_data;
}
