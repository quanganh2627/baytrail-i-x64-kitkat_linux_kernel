/*
 * -------------------------------------------------------------------------
 *  Copyright (C) 2014 Intel Mobile Communications GmbH
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#include "rt9455_charger.h"

#define REG_DEVICE_ID 0x03
#define REG_CHARGE_CTRL1	0x00
#define REG_CHARGE_CTRL2	0x01
#define REG_CHARGE_CTRL3	0x02
#define REG_CHARGE_CTRL4	0x04
#define REG_CHARGE_CTRL5	0x05
#define REG_CHARGE_CTRL6	0x06
#define REG_CHARGE_CTRL7	0x07
#define REG_CHARGE_IRQ1	0x08
#define REG_CHARGE_IRQ2	0x09
#define REG_CHARGE_IRQ3	0x0A
#define REG_CHARGE_MASK1	0x0B
#define REG_CHARGE_MASK2	0x0C
#define REG_CHARGE_MASK3	0x0D

/* REG_IC_INFO */
#define VENDOR_O 4
#define VENDOR_M 0x0F

#define REV_O 0
#define REV_M 0x0F

/* CHARGE_CTRL1_REG - Ready only status register */
#define STAT_O 4
#define STAT_M 0x3

#define BOOST_O 3
#define BOOST_M 0x1

#define PWR_RDY_O 2
#define PWR_RDY_M 0x1

#define OTG_PINP_O 1
#define OTG_PINP_M 0x1

/* CHARGE_CTRL2_REG */
#define IAICR_O 6
#define IAICR_M 0x3

#define TE_SHDN_EN_O 5
#define TE_SHDN_EN_M 0x1

#define HIGHER_OCP_O 4
#define HIGHER_OCP_M 0x1

#define TE_O 3
#define TE_M 0x1

#define IAICR_INT_O 2
#define IAICR_INT_M 0x1

#define HZ_O 1
#define HZ_M 0x1

#define OPA_MODE_O 0
#define OPA_MODE_M 0x1

/* CHARGE_CTRL3_REG */
#define VOREG_O 2
#define VOREG_M 0x3F

#define OTG_PL_O 1
#define OTG_PL_M 0x1

#define OTG_EN_O 0
#define OTG_EN_M 0x1

/* CHARGE_CTRL4_REG - Reset Register */
#define RST_O 7
#define RST_M 1

/* CHARGE_CTRL5_REG */
#define TMR_EN_O 7
#define TMR_EN_M 0x1

#define MIVR_O 4
#define MIVR_M 0x3

#define IPREC_O 2
#define IPREC_M 0x3

#define IEOC_O 0
#define IEOC_M 0x3

/* CHARGE_CTRL6_REG */
#define IAICR_SEL_O 7
#define IAICR_SEL_M 0x1

#define ICHRG_O 4
#define ICHRG_M 0x7

#define VPREC_O 0
#define VPREC_M 0x7

/* CHARGE_CTRL6_REG */
#define BATD_EN_O 6
#define BATD_EN_M 0x1

#define CHG_EN_O 4
#define CHG_EN_M 0x1

#define VMREG_O 0
#define VMREG_M 0x0F

enum {

	TSD_OCCURRED = 1,

	OVP_OCCURRED = 1,

	BOOSTOV_OCCURRED = 1,

	BATUV_OCCURRED = 1,

	BAT_OVP_OCCURRED = 1,

	T32_TO_OCCURRED = 1,

	VBUS_FAULT = 1,

	POOR_INPUT_SOURCE_OCCURRED = 1,

	SLEEP_MODE_OCCURRED = 1,

	NO_BAT_OCCURRED = 1,

	VBUS_OFF = 0,
	VBUS_ON,

};

enum RT9455_FAULT_STAT {
	NO_FAULT = 0,
	VBUS_OVP = 1,
	SLEEP_MODE = 2,
	BOOST_OV = 2,
	POOR_INPUT_SOURCE = 3,
	BAT_UV = 3,
	BATTERY_OVP = 4,
	THERMAL_SHUTDOWN = 5,
	TIMER_FAULT = 6,
	NO_BATTERY = 7,
};

struct charger_attrmap rt9455_charger_attr_map[ATTR_MAX] = {
	[DEVICE_ID_REG] = {"IC_info_reg", FULL_REG, REG_DEVICE_ID, 0, 0xFF},
	[CHARGE_CTRL1_REG] = {"ctrl1_reg", FULL_REG, REG_CHARGE_CTRL1, 0, 0xFF},
	[CHARGE_CTRL2_REG] = {"ctrl2_reg", FULL_REG, REG_CHARGE_CTRL2, 0, 0xFF},
	[CHARGE_CTRL3_REG] = {"ctrl3_reg", FULL_REG, REG_CHARGE_CTRL3, 0, 0xFF},
	[CHARGE_CTRL4_REG] = {"ctrl4_reg", FULL_REG, REG_CHARGE_CTRL4, 0, 0xFF},
	[CHARGE_CTRL5_REG] = {"ctrl5_reg", FULL_REG, REG_CHARGE_CTRL5, 0, 0xFF},
	[CHARGE_CTRL6_REG] = {"ctrl6_reg", FULL_REG, REG_CHARGE_CTRL6, 0, 0xFF},
	[CHARGE_CTRL7_REG] = {"ctrl7_reg", FULL_REG, REG_CHARGE_CTRL7, 0, 0xFF},
	[CHARGE_IRQ1_REG] = {"irq1_reg", FULL_REG, REG_CHARGE_IRQ1, 0, 0xFF},
	[CHARGE_IRQ2_REG] = {"irq2_reg", FULL_REG, REG_CHARGE_IRQ2, 0, 0xFF},
	[CHARGE_IRQ3_REG] = {"irq3_reg", FULL_REG, REG_CHARGE_IRQ3, 0, 0xFF},
	[CHARGE_MASK1_REG] = {"mask1_reg", FULL_REG, REG_CHARGE_MASK1, 0, 0xFF},
	[CHARGE_MASK2_REG] = {"mask2_reg", FULL_REG, REG_CHARGE_MASK2, 0, 0xFF},
	[CHARGE_MASK3_REG] = {"mask3_reg", FULL_REG, REG_CHARGE_MASK3, 0, 0xFF},

	[VENDOR_ID] = {"Vendor", BITS, REG_DEVICE_ID, VENDOR_O, VENDOR_M},
	[CHIP_REV] = {"Chip_Rev", BITS, REG_DEVICE_ID, REV_O, REV_M},
	[STAT] = {"Stat", BITS, REG_CHARGE_CTRL1, STAT_O, STAT_M},
	[BOOST] = {"Boost", BITS, REG_CHARGE_CTRL1, BOOST_O, BOOST_M},
	[PWR_RDY] = {"Power_Ready",
		BITS, REG_CHARGE_CTRL1, PWR_RDY_O, PWR_RDY_M},
	[OTG_PINP] = {"OTG_Pin_Pol",
		BITS, REG_CHARGE_CTRL1, OTG_PINP_O, OTG_PINP_M},
	[IAICR] = {"IAICR", BITS, REG_CHARGE_CTRL2, IAICR_O, IAICR_M},
	[TE_SHDN_EN] = {"TE_Shutdown_EN",
		BITS, REG_CHARGE_CTRL2, TE_SHDN_EN_O, TE_SHDN_EN_M},
	[HIGHER_OCP] = {"Higher_OCP",
		BITS, REG_CHARGE_CTRL2, HIGHER_OCP_O, HIGHER_OCP_M},
	[TE] = {"End_of_chgr_det", BITS, REG_CHARGE_CTRL2, TE_O, TE_M},
	[IAICR_INT] = {"IAICR_Setting",
		BITS, REG_CHARGE_CTRL2, IAICR_INT_O, IAICR_INT_M},
	[HZ_MODE] = {"HZ_Mode", BITS, REG_CHARGE_CTRL2, HZ_O, HZ_M},
	[OPA_MODE] = {"OPA_mode",
		BITS, REG_CHARGE_CTRL2, OPA_MODE_O, OPA_MODE_M},
	[VOREG] = {"Voreg", BITS, REG_CHARGE_CTRL3, VOREG_O, VOREG_M},
	[OTG_PL] = {"OTG_Level",
		BITS, REG_CHARGE_CTRL3, OTG_PL_O, OTG_PL_M},
	[OTG_EN] = {"OTG_Enable",
		BITS, REG_CHARGE_CTRL3, OTG_EN_O, OTG_EN_M},
	[RST] = {"Reset", BITS, REG_CHARGE_CTRL4, RST_O, RST_M},
	[TMR_EN] = {"Timer_Enable",
		BITS, REG_CHARGE_CTRL5, TMR_EN_O, TMR_EN_M},
	[MIVR] = {"VMIVR", BITS, REG_CHARGE_CTRL5, MIVR_O, MIVR_M},
	[IPREC] = {"I_Precharge", BITS, REG_CHARGE_CTRL5, IPREC_O, IPREC_M},
	[IEOC] = {"I_EndofChgr", BITS, REG_CHARGE_CTRL5, IEOC_O, IEOC_M},
	[IAICR_SEL] = {"IACR_Sel",
		BITS, REG_CHARGE_CTRL6, IAICR_SEL_O, IAICR_SEL_M},
	[ICHRG] = {"iCharge", BITS, REG_CHARGE_CTRL6, ICHRG_O, ICHRG_M},
	[VPREC] = {"vPreCharge", BITS, REG_CHARGE_CTRL6, VPREC_O, VPREC_M},
	[BATD_EN] = {"batDet_En", BITS, REG_CHARGE_CTRL7, BATD_EN_O, BATD_EN_M},
	[CHG_EN] = {"Chgr_En", BITS, REG_CHARGE_CTRL7, CHG_EN_O, CHG_EN_M},
	[VMREG] = {"Max_vBat", BITS, REG_CHARGE_CTRL7, VMREG_O, VMREG_M},
};

static int rt9455_enable_charger(struct rt9455_charger *chrgr, bool enable)
{
	rt9455_attr_write(chrgr->client, CHG_EN, enable);
	return 0;
}

static int rt9455_configure_chip(struct rt9455_charger *chrgr,
							bool enable_charging)
{
	int ret;
	/* Reset Charger */
	ret = rt9455_attr_write(chrgr->client, RST, 1);
	if (ret != 0)
		goto fail;
	/* Disable the timer */
	ret = rt9455_attr_write(chrgr->client, TMR_EN, 0);
	if (ret != 0)
		goto fail;

	return rt9455_enable_charger(chrgr, enable_charging);
fail:
	pr_err("rt9455_configure_chip fail due to attr_write fail!\n");
	return -1;
}

static int rt9455_get_charger_state(struct rt9455_charger *chrgr)
{
	u8 ctrl1_reg, irq1_reg, irq2_reg, irq3_reg, stat, pwr_rdy;
	int ret;

	ret = rt9455_attr_read(chrgr->client,
			CHARGE_CTRL1_REG, &ctrl1_reg);
	if (ret != 0)
		goto fail;

	ret = rt9455_attr_read(chrgr->client, STAT, &stat);
	if (ret != 0)
		goto fail;
	ret = rt9455_attr_read(chrgr->client,
		PWR_RDY, &pwr_rdy);
	if (ret != 0)
		goto fail;

	pr_info("%s: ctrl1_reg=0x%x, stat=0x%x, pwr_rdy=0x%x\n",
		__func__, ctrl1_reg, stat, pwr_rdy);

	chrgr->state.vbus = pwr_rdy;
	chrgr->state.vbus_ovp = 0;
	chrgr->state.sleep_mode = 0;
	chrgr->state.poor_input_source = 0;
	chrgr->state.bat_ovp = 0;
	chrgr->state.tsd_flag = 0;
	chrgr->state.t32s_timer_expired = 0;
	chrgr->state.no_bat = 0;

	chrgr->state.health = (chrgr->state.vbus == VBUS_ON) ?
		POWER_SUPPLY_HEALTH_GOOD : POWER_SUPPLY_HEALTH_UNKNOWN;


	/* Retrieve the IRQ flags */
	rt9455_attr_read(chrgr->client, CHARGE_IRQ1_REG, &irq1_reg);
	rt9455_attr_read(chrgr->client, CHARGE_IRQ2_REG, &irq2_reg);
	rt9455_attr_read(chrgr->client, CHARGE_IRQ3_REG, &irq3_reg);
	pr_info("IRQ1=0x%x, IRQ2=0x%x, IRQ3=0x%x\n",
		irq1_reg, irq2_reg, irq3_reg);
	/* TODO: Check what should be perform according to the IRQ flags */
fail:
	return ret;
}

static int rt9455_iocharge_list[8] = {
			500, 650, 800, 950, 1100, 1250, 1400, 1550};

static int rt9455_calc_iocharge_regval
	(struct rt9455_charger *chrgr, int current_to_set_ma)
{
	int i;
	for (i = 0; i < 7; i++) {
		if (current_to_set_ma >= rt9455_iocharge_list[i]
			&& current_to_set_ma < rt9455_iocharge_list[i+1])
			break;
	}
	if (current_to_set_ma >= rt9455_iocharge_list[7])
		i = 7;
	return i;
}

static int rt9455_get_iocharge_val(int regval)
{
	return rt9455_iocharge_list[regval];
}

struct rt9455_charger rt9455_chrgr_data = {
	.vendor = 0,
	.rev = 9,
	.max_voreg = 4450,
	.min_voreg = 3500,
	.max_iocharge = 1550,
	.min_iocharge = 500,
	.max_ibus_limit = 1000,
	.min_ibus_limit = 100,
	.default_cc = 500,
	.default_cv = 4000,
	.model_name = "RT9455",
	.manufacturer = "Richtek",

	.otg_nb = {
		.notifier_call = rt9455_otg_notification_handler,
	},

	.chgint_bh = {
		.in_suspend = false,
		.pending_evt = false,
	},
	.boost_op_bh = {
		.in_suspend = false,
		.pending_evt = false,
	},
	.fake_vbus = -1,

	.state = {
		.status = RT9455_STATUS_UNKNOWN,
		.vbus = -1,
		.cc = 500,
		.max_cc = 1550,
		.cv = 4000,
		.iterm = 0,
		.health = POWER_SUPPLY_HEALTH_UNKNOWN,
		.cable_type = POWER_SUPPLY_CHARGER_TYPE_NONE,
		.charger_enabled = false,
		.charging_enabled = true, /* initially HZ mode is switched off*/
		.ovp_flag = 0,
		.t32s_timer_expired = 0,
		.tsd_flag = 0,
		.vbus_ovp = 0,
		.sleep_mode = 0,
		.poor_input_source = 0,
		.bat_ovp = 0,
		.no_bat = 0,
	},

	.attrmap = rt9455_charger_attr_map,
	.configure_chip = rt9455_configure_chip,
	.enable_charger = rt9455_enable_charger,
	.get_charger_state = rt9455_get_charger_state,
	.calc_iocharge_regval = rt9455_calc_iocharge_regval,
	.get_iocharge_val = rt9455_get_iocharge_val,
};
