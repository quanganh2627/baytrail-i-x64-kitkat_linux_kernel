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

#define RT9455_NAME "rt9455_charger"
#define pr_fmt(fmt) RT9455_NAME": "fmt

/* Whether using charger detect irq? */
#define USE_CDET_IRQ 1

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/usb/otg.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/pinctrl/consumer.h>
#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_controller.h>
#include <linux/idi/idi_ids.h>
#include <linux/idi/idi_bus.h>
#include <linux/idi/idi_device_pm.h>
#include <linux/usb/phy-intel.h>

#include <linux/time.h>
#include <linux/wakelock.h>
#include "rt9455_charger.h"

/*
 * Development debugging is not enabled in release image to prevent
 * loss of event history in the debug array which has a limited size
 */
#include <linux/power/charger_debug.h>

#define VOREG_STEP_MV 20
#define IOCHARGE_STEP_MA 150
#define IBUS_LIMIT_STEP_MA 400
#define IBUS_NO_LIMIT 3

enum {
	VBUS_OFF = 0,
	VBUS_ON,

	T32_TO_OCCURRED = 1,
};


#define CHARGER_CONTROL_O 0x0
#define CHARGER_CONTROL(_base) ((_base) + CHARGER_CONTROL_O)
	#define CHARGER_CONTROL_CIRSP_O 27
	#define CHARGER_CONTROL_CIRSP_M 0x3
	#define CHARGER_CONTROL_CIEDG_O 26
	#define CHARGER_CONTROL_CIEDG_M 0x1
	#define CHARGER_CONTROL_CILVL_O 25
	#define CHARGER_CONTROL_CILVL_M 0x1
	#define CHARGER_CONTROL_CISENS_O 24
	#define CHARGER_CONTROL_CISENS_M 0x1
	#define CHARGER_CONTROL_CIEN_O 23
	#define CHARGER_CONTROL_CIEN_M 0x1
	#define CHARGER_CONTROL_CDETSENS_O 10
	#define CHARGER_CONTROL_CDETSENS_M 0x1
	#define CHARGER_CONTROL_CIDBT_O 17
	#define CHARGER_CONTROL_CIDBT_M 0x7
	#define CHARGER_CONTROL_CHGLVL_O 1
	#define CHARGER_CONTROL_CHGLVL_M 0x1

#define CHARGER_CONTROL_CIEDG_FALLING 0
#define CHARGER_CONTROL_CILVL_LOW 0
#define CHARGER_CONTROL_CISENS_EDGE 1
#define CHARGER_CONTROL_CIEN_EN 0
#define CHARGER_CONTROL_CHGLVL_LOW 0
#define CHARGER_CONTROL_IRQ_DEBOUNCE_DISABLE 0
#define CHARGER_CONTROL_CDETSENS_EDGE 1

#define CHARGER_STATUS_O 0x4
#define CHARGER_STATUS(_base) ((_base) + CHARGER_STATUS_O)

#define CHARGER_CONTROL_WR_O 0x8
#define CHARGER_CONTROL_WR(_base) ((_base) + CHARGER_CONTROL_WR_O)
	#define CHARGER_CONTROL_WR_WS_O 0
	#define CHARGER_CONTROL_WR_WS_M 0x1

#define CHARGER_WR_O 0xC
#define CHARGER_WR(_base) ((_base) + CHARGER_WR_O)
	#define CHARGER_WR_WS_O 0
	#define CHARGER_WR_WS_M 0x1

static int rt9455_configure_chip(
			struct rt9455_charger *chrgr, bool enable_charging);

static int rt9455_enable_charging(
			struct rt9455_charger *chrgr, bool enable);

struct charger_debug_data chrgr_dbg = {
	.printk_logs_en = 0,
};

static struct power_supply_throttle rt9455_dummy_throttle_states[] = {
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
	},
};

static char *rt9455_supplied_to[] = {
		"battery",
};

static enum power_supply_property rt9455_power_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_INLMT,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static void rt9455_dump_all_regs(struct rt9455_charger *chrgr)
{
	int i, ret;
	u8 val;
	pr_info("%s:\n", __func__);
	for (i = 0; i < ATTR_MAX; i++) {
		if (chrgr->attrmap[i].rpt &&
			chrgr->attrmap[i].type == FULL_REG) {
			ret = rt9455_attr_read(chrgr->client, i, &val);
			if (ret) {
				pr_err("chrgr->attrmap[%d].rpt=%s, read fail!\n",
						i, chrgr->attrmap[i].rpt);
				continue;
			}
			pr_info("%s = 0x%x\n",
					chrgr->attrmap[i].rpt,  val);
		}
	}
}

#ifdef SYSFS_FAKE_VBUS_SUPPORT

/**
 * fake_vbus_show	Called when value queried from driver to sysfs
 * @dev			[in] pointer to device
 * @attr		[in] pointer to devices's attribute
 * @buf			[out] pointer to buffer that is to be filled with
 *			string. Passed from driver to sysfs
 *
 * Returns:		number of characters read
 */
static ssize_t fake_vbus_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct rt9455_charger *chrgr = i2c_get_clientdata(to_i2c_client(dev));
	size_t size_copied;
	int value;

	value = chrgr->fake_vbus;
	size_copied = sprintf(buf, "%d\n", value);

	return size_copied;
}

/**
 * fake_vbus_store	Called when value written from sysfs to driver
 * @dev			[in] pointer to device
 * @attr		[in] pointer to devices's attribute
 * @buf			[in] pointer to buffer containing string passed
 *			from sysfs
 * @count		[in] number of characters in string
 *
 * Returns:		number of characters written
 */
static ssize_t fake_vbus_store(
			struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct rt9455_charger *chrgr = i2c_get_clientdata(to_i2c_client(dev));
	int sysfs_val;
	int ret;
	size_t size_to_cpy;
	char strvalue[SYSFS_INPUT_VAL_LEN + 1];

	size_to_cpy = (count > SYSFS_INPUT_VAL_LEN) ?
				SYSFS_INPUT_VAL_LEN : count;

	strncpy(strvalue, buf, size_to_cpy);
	strvalue[size_to_cpy] = '\0';

	ret = kstrtoint(strvalue, 10, &sysfs_val);
	if (ret != 0)
		return ret;

	sysfs_val = (sysfs_val == 0) ? 0 : 1;

	down(&chrgr->prop_lock);

	chrgr->fake_vbus = -1;
	pr_info("%s: sysfs_val=%d, chrgr->state.charging_enabled=%d\n",
			__func__, sysfs_val, chrgr->state.charging_enabled);
	if (chrgr->state.vbus != 1) {
		pr_err("fake vbus event requested when USB cable removed !\n");
		up(&chrgr->prop_lock);
		return count;
	}

	if (sysfs_val == 0) {
		chrgr->fake_vbus = 0;
		atomic_notifier_call_chain(&chrgr->otg_handle->notifier,
				USB_EVENT_VBUS, &chrgr->fake_vbus);

		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_FAKE_VBUS,
						chrgr->fake_vbus, 0);
		pr_info("fake vbus removal sent\n");

	} else if (sysfs_val == 1 && !chrgr->state.charging_enabled) {
		chrgr->fake_vbus = 1;
		atomic_notifier_call_chain(&chrgr->otg_handle->notifier,
				USB_EVENT_VBUS, &chrgr->fake_vbus);

		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_FAKE_VBUS,
						chrgr->fake_vbus, 0);
		pr_info("fake vbus connection sent\n");
	}

	up(&chrgr->prop_lock);

	return count;
}

static struct device_attribute rt9455_fake_vbus_attr = {
	.attr = {
		.name = "fake_vbus_event",
		.mode = S_IRUSR | S_IWUSR,
	},
	.show = fake_vbus_show,
	.store = fake_vbus_store,
};

/**
 * rt9455_setup_sysfs_attr	Sets up sysfs entries for rt9455 i2c device
 * @chrgr			[in] pointer to charger driver internal
 *				structure
 */
static void rt9455_setup_fake_vbus_sysfs_attr(struct rt9455_charger *chrgr)
{
	struct device *dev = &chrgr->client->dev;
	int err;

	err = device_create_file(dev, &rt9455_fake_vbus_attr);
	if (err)
		pr_err("Unable to create sysfs entry: '%s'\n",
				rt9455_fake_vbus_attr.attr.name);
}

#else

static inline void rt9455_setup_fake_vbus_sysfs_attr(
					struct rt9455_charger *chrgr)
{
	(void) chrgr;
}

#endif /*SYSFS_FAKE_VBUS_SUPPORT*/





#ifdef CONFIG_DEBUG_FS


static int dbg_evt_open(struct inode *inode, struct file *file)
{

	/* save private data (the address of test_mod) in file struct
	(will be used by read()) */
	file->private_data = inode->i_private;

	return 0;
}

static ssize_t dbg_evt_read(
			struct file *filp, char __user *buf,
					size_t count, loff_t *f_pos)
{

	/* obtaining private data saved by open method */
	struct charger_debug_data *dbg_array =
			(struct charger_debug_data *) filp->private_data;

	unsigned long time_stamp_jiffies, time_stamp_s;
	enum charger_debug_event event;
	const char *event_str;
	u32 cnt, read_idx;
	int prm, prm2;
	ssize_t retval;

	int i, chars_count, total_chars = 0;

	char log_line[LOG_LINE_LENGTH];

	for (i = 0; i < LINES_PER_PAGE; i++) {

		chars_count = 0;

		spin_lock(&dbg_array->lock);
		cnt = dbg_array->count;

		if (!cnt) {
			spin_unlock(&dbg_array->lock);
			return total_chars;
		}

		read_idx = dbg_array->read_index;
		event = dbg_array->log_array[read_idx].event;
		event_str = dbg_array->log_array[read_idx].event_string;
		time_stamp_jiffies =
			dbg_array->log_array[read_idx].time_stamp_jiffies;

		prm = dbg_array->log_array[read_idx].param;
		prm2 = dbg_array->log_array[read_idx].param2;
		dbg_array->count--;
		dbg_array->read_index++;
		dbg_array->read_index &= (CHARGER_DEBUG_DATA_SIZE-1);


		spin_unlock(&dbg_array->lock);


		time_stamp_s = time_stamp_jiffies/HZ;
		chars_count += snprintf(
			log_line, LOG_LINE_LENGTH, "[%5lu.%3lu]: %s ",
			time_stamp_s, (time_stamp_jiffies - time_stamp_s*HZ),
								event_str);

		if (event == CHG_DBG_REG) {
			chars_count += snprintf(
				log_line + chars_count,
				LOG_LINE_LENGTH - chars_count,
				"addr=0x%x, val=0x%x\n", prm, prm2);

		} else if (event == CHG_DBG_I2C_READ_ERROR ||
				event == CHG_DBG_I2C_WRITE_ERROR) {

			chars_count += snprintf(
				log_line + chars_count,
				LOG_LINE_LENGTH - chars_count,
				"err=%d, at addr=0x%x\n", prm, prm2);

		} else if (event < CHG_DBG_LAST_NO_PARAM_EVENT) {

			chars_count += snprintf(
				log_line + chars_count,
				LOG_LINE_LENGTH - chars_count, "\n");
		} else {
			chars_count += snprintf(
				log_line + chars_count,
				LOG_LINE_LENGTH - chars_count, "val=%d\n", prm);
		}

		/* copy data from driver to user space buffer */
		if (copy_to_user(buf + total_chars, log_line, chars_count)) {
			retval = -EFAULT;
			goto out;
		}
		total_chars += chars_count;

	}

	retval = total_chars;

out:
	return retval;
}

static const struct file_operations rt9455_evt_dbg_fops = {
	.open = dbg_evt_open,
	.read = dbg_evt_read,
};



static int rt9455_dbg_regs_show(struct seq_file *m, void *data)
{
	int i, ret = 0;
	struct rt9455_charger *chrgr = (struct rt9455_charger *) m->private;
	u8 val;
	unsigned long timestamp_jiffies, timestamp_s;

	if (!chrgr->attrmap)
		return -ENODEV;

	down(&chrgr->prop_lock);

	timestamp_jiffies = jiffies - INITIAL_JIFFIES;
	timestamp_s = timestamp_jiffies/HZ;
	seq_printf(m, "[%5lu.%3lu] :\n", timestamp_s,
				(timestamp_jiffies - timestamp_s*HZ));

	for (i = 0; i < ATTR_MAX; i++) {
		if (chrgr->attrmap[i].rpt &&
			chrgr->attrmap[i].type == FULL_REG) {
			ret = rt9455_attr_read(chrgr->client, i, &val);
			if (ret)
				goto out;
			seq_printf(m, "%s = 0x%x\n",
					chrgr->attrmap[i].rpt,	val);
		}
	}

out:
	up(&chrgr->prop_lock);

	return ret;
}

static int rt9455_dbg_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, rt9455_dbg_regs_show, inode->i_private);
}

static const struct file_operations rt9455_dbg_regs_fops = {
	.open = rt9455_dbg_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static int rt9455_dbg_state_show(struct seq_file *m, void *data)
{
	struct rt9455_charger *chrgr = (struct rt9455_charger *) m->private;
	unsigned long timestamp_jiffies, timestamp_s;

	(void)data;

	timestamp_jiffies = jiffies - INITIAL_JIFFIES;
	timestamp_s = timestamp_jiffies/HZ;
	seq_printf(m, "[%5lu.%3lu] :\n", timestamp_s,
				(timestamp_jiffies - timestamp_s*HZ));

	seq_printf(m, "vbus = %d\n", chrgr->state.vbus);
	seq_printf(m, "cc = %d\n", chrgr->state.cc);
	seq_printf(m, "max_cc = %d\n", chrgr->state.max_cc);
	seq_printf(m, "cv = %d\n", chrgr->state.cv);
	seq_printf(m, "iterm = %d\n", chrgr->state.iterm);
	seq_printf(m, "inlmt = %d\n", chrgr->state.inlmt);
	seq_printf(m, "health = %d\n", chrgr->state.health);
	seq_printf(m, "cable_type = %d\n", chrgr->state.cable_type);
	seq_printf(m, "charger_enabled = %d\n", chrgr->state.charger_enabled);
	seq_printf(m, "charging_enabled = %d\n",
				chrgr->state.charging_enabled);
	seq_printf(m, "to_enable_boost = %d\n\n",
				chrgr->state.to_enable_boost);
	seq_printf(m, "boost_enabled = %d\n\n",
				chrgr->state.boost_enabled);

	seq_printf(m, "pok_b = %u\n", chrgr->state.pok_b);
	seq_printf(m, "ovp_flag = %u\n", chrgr->state.ovp_flag);
	seq_printf(m, "ovp_recov = %u\n", chrgr->state.ovp_recov);
	seq_printf(m, "t32s_timer_expired = %u\n",
				chrgr->state.t32s_timer_expired);

	seq_printf(m, "vbus_fault = %u\n", chrgr->state.vbus_fault);
	seq_printf(m, "treg_flag = %u\n", chrgr->state.treg_flag);
	seq_printf(m, "ot_recov_flag = %u\n\n", chrgr->state.ot_recov_flag);

	seq_printf(m, "vbus_ovp = %u\n", chrgr->state.vbus_ovp);
	seq_printf(m, "sleep_mode = %u\n", chrgr->state.sleep_mode);
	seq_printf(m, "poor_input_source = %u\n",
					chrgr->state.poor_input_source);
	seq_printf(m, "bat_ovp = %u\n", chrgr->state.bat_ovp);
	seq_printf(m, "no_bat = %u\n", chrgr->state.no_bat);
	seq_printf(m, "bat_uv = %u\n", chrgr->state.bat_uv);
	seq_printf(m, "boost_ov = %u\n\n", chrgr->state.boost_ov);

	return 0;
}

static int rt9455_dbg_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, rt9455_dbg_state_show, inode->i_private);
}

static const struct file_operations rt9455_dbg_state_fops = {
	.open = rt9455_dbg_state_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


/**
 * rt9455_setup_debugfs - sets up debugfs entries for rt9455 charger driver
 * @chrgr		[in] pointer to charger driver internal structure
 * @dbg_data		[in] pointer to debug array containing events logs.
 */
static void rt9455_setup_debugfs(struct rt9455_charger *chrgr,
				struct charger_debug_data *dbg_data)
{
	struct dentry *dbgfs_entry;

	dbgfs_entry = debugfs_create_dir(RT9455_NAME, NULL);
	if (!dbgfs_entry)
		return;

	chrgr->debugfs_root_dir = dbgfs_entry;

	(void)debugfs_create_file(EVENTS_LOG_FILENAME, S_IRUGO,
			dbgfs_entry, dbg_data, &rt9455_evt_dbg_fops);

	(void)debugfs_create_file(DBG_REGS_FILENAME, S_IRUGO,
			dbgfs_entry, chrgr, &rt9455_dbg_regs_fops);

	(void)debugfs_create_file(DBG_STATE_FILENAME, S_IRUGO,
			dbgfs_entry, chrgr, &rt9455_dbg_state_fops);

	return;
}

/**
 * rt9455_remove_debugfs_dir	recursively removes debugfs root directory
 *				of RT9455 charger driver
 * @chrgr			[in] pointer to charger driver's
 *				internal structure
 */
static void rt9455_remove_debugfs_dir(struct rt9455_charger *chrgr)
{
	debugfs_remove_recursive(chrgr->debugfs_root_dir);
	return;
}

#else

static inline void rt9455_setup_debugfs(struct rt9455_charger *chrgr,
				struct charger_debug_data *dbg_data)
{

}

static inline void rt9455_remove_debugfs_dir(struct rt9455_charger *chrgr)
{

}

#endif /* CONFIG_DEBUG_FS  */

static int rt9455_trigger_wtd(struct rt9455_charger *chrgr)
{
	int ret = 0;
	/* Currently disable the watchdog timer */
#if 0
	ret = rt9455_attr_write(chrgr->client, TMR_EN, 0);
	ret = rt9455_attr_write(chrgr->client, TMR_EN, 1);
#endif
	return ret;
}


static int rt9455_set_voreg(struct rt9455_charger *chrgr,
				int volt_to_set, int *volt_set, bool propagate)
{
	int ret, volt_to_set_mv;
	int mid_volt_mv = 4300;
	u8 volt_regval, readback_val;
	pr_info("%s: volt_to_set=%d mV\n", __func__, volt_to_set);
	volt_to_set_mv = fit_in_range(volt_to_set,
				chrgr->min_voreg, chrgr->max_voreg);

	/* It is non-liner above 4.3V, temp limit it to 4.3V */
	if (volt_to_set_mv > mid_volt_mv)
		volt_to_set_mv = mid_volt_mv;

	volt_regval = (u8)((volt_to_set_mv - chrgr->min_voreg) / VOREG_STEP_MV);

	pr_info("volt_regval=0x%x\n", volt_regval);

	if (propagate) {
		ret = rt9455_attr_write(chrgr->client, VOREG, volt_regval);
		if (ret)
			return ret;

		/* Reading back the register value becuase it could ignore our
		setting as a result of being limited by the value of
		SAFETY register  */
		ret = rt9455_attr_read(chrgr->client, VOREG, &readback_val);
		if (ret)
			return ret;

		if (volt_regval != readback_val) {
			pr_err("I2C write() error! Register VOREG still contains the old value\n");
			return -EIO;
		}

	}

	*volt_set = ((int)volt_regval * VOREG_STEP_MV) + chrgr->min_voreg;
	pr_info("End of %s: volt_set=%d mV\n", __func__, *volt_set);

	return 0;
}

static int rt9455_set_iocharge(
			struct rt9455_charger *chrgr, int curr_to_set,
						int *curr_set, bool propagate)
{
	int ret, current_to_set_ma;
	u8 regval, readback_val;
	pr_info("%s: curr_to_set=%d mA\n", __func__, curr_to_set);
	if (curr_to_set < chrgr->min_iocharge) {
		if (chrgr->state.charging_enabled && propagate) {
			ret = rt9455_enable_charging(chrgr, false);
			if (ret != 0)
				return ret;
			chrgr->state.charging_enabled = 0;
		}
		*curr_set = 0;
		return 0;
	}

	current_to_set_ma = fit_in_range(curr_to_set, chrgr->min_iocharge,
							chrgr->max_iocharge);
	if (chrgr->calc_iocharge_regval)
		regval = chrgr->calc_iocharge_regval(chrgr, current_to_set_ma);
	else
		regval = (u8)((current_to_set_ma - chrgr->min_iocharge) /
							IOCHARGE_STEP_MA);

	pr_info("current_to_set_ma=%d, regval=0x%x\n",
			current_to_set_ma, regval);
	if (propagate) {
		ret = rt9455_attr_write(chrgr->client, ICHRG, regval);
		if (ret)
			return ret;

		/* Reading back the register value because it could ignore our
		setting as a result of being limited by the value of
		SAFETY register  */
		ret = rt9455_attr_read(chrgr->client, ICHRG, &readback_val);
		if (ret)
			return ret;

		if (readback_val != regval) {
			pr_err("I2C write() error! Register IBAT still contains the old value\n");
			return -EIO;
		}


		regval = readback_val;
	}
	if (chrgr->get_iocharge_val)
		*curr_set = chrgr->get_iocharge_val(regval);
	else
		*curr_set = (regval * IOCHARGE_STEP_MA + chrgr->min_iocharge);

	pr_info("End of %s: curr_set = %d mA\n", __func__, *curr_set);
	return 0;
}

static int rt9455_set_ibus_limit(struct rt9455_charger *chrgr,
						int ilim_to_set, int *ilim_set)
{
	int ret, current_to_set_ma;
	u8 regval, readback_val;
	pr_info("%s:, ilim_to_set=%d mA\n", __func__, ilim_to_set);

	if (ilim_to_set < chrgr->min_ibus_limit) {
		if (chrgr->state.charging_enabled) {
			ret = rt9455_enable_charging(chrgr, false);
			if (ret != 0)
				return ret;
			chrgr->state.charging_enabled = 0;
		}
		*ilim_set = 0;
		return 0;
	}

	if (ilim_to_set <= chrgr->max_ibus_limit) {
		current_to_set_ma = fit_in_range(ilim_to_set,
				chrgr->min_ibus_limit, chrgr->max_ibus_limit);
		/* Not accurate while 900mA, keep it as simple here */
		regval = (u8)((current_to_set_ma - chrgr->min_ibus_limit) /
							IBUS_LIMIT_STEP_MA);
	} else {
		regval = IBUS_NO_LIMIT;
	}

	ret = rt9455_attr_write(chrgr->client, IAICR, regval);
	if (ret)
		return ret;

	ret = rt9455_attr_read(chrgr->client, IAICR, &readback_val);
	if (ret)
		return ret;

	if (readback_val != regval) {
		pr_err("I2C write() error! Register IBUS still contains the old value\n");
		return -EIO;
	}


	*ilim_set = (regval == IBUS_NO_LIMIT) ? ilim_to_set :
			(regval * IBUS_LIMIT_STEP_MA + chrgr->min_ibus_limit);
	pr_info("End of %s:, ilim_set=%d mA\n", __func__, *ilim_set);
	return 0;
}


static inline bool rt9455_is_online(struct rt9455_charger *chrgr,
						struct power_supply *psy)
{
	if (!(chrgr->state.health == POWER_SUPPLY_HEALTH_GOOD) ||
		!chrgr->state.charger_enabled)
		return false;


	if (psy->type == POWER_SUPPLY_TYPE_MAINS)
		return ((chrgr->state.cable_type ==
			POWER_SUPPLY_CHARGER_TYPE_USB_CDP) ||
			(chrgr->state.cable_type ==
			POWER_SUPPLY_CHARGER_TYPE_USB_DCP));

	else if (psy->type == POWER_SUPPLY_TYPE_USB)
		return ((chrgr->state.cable_type ==
			POWER_SUPPLY_CHARGER_TYPE_USB_SDP) ||
			(chrgr->state.cable_type ==
			POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING));

	return false;
}

static void rt9455_charging_worker(struct work_struct *work)
{
	int ret;
	struct rt9455_charger *chrgr =
		container_of(work, struct rt9455_charger, charging_work.work);

	if (!time_after(jiffies, chrgr->ack_time + (60*HZ))) {
		down(&chrgr->prop_lock);
		ret = rt9455_trigger_wtd(chrgr);
		up(&chrgr->prop_lock);
		if (ret != 0)
			return;
	}
	power_supply_changed(chrgr->current_psy);
	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_TRIG_POWER_SUPPLY_CHARGER, 0, 0);

	schedule_delayed_work(&chrgr->charging_work, CHRGR_WORK_DELAY);

	return;
}

static int rt9455_enable_charging(struct rt9455_charger *chrgr, bool enable)
{
	int ret;

	if (chrgr->enable_charger) {
		ret = chrgr->enable_charger(chrgr, enable);
		if (ret)
			return ret;
	} else
		return -EINVAL;

	if (enable) {
		/*
		 * Obtain Wake Lock to prevent suspend during charging
		 * because the charger watchdog needs to be cont. retriggered.
		 */
		wake_lock(&chrgr->suspend_lock);
		schedule_delayed_work(&chrgr->charging_work, CHRGR_WORK_DELAY);
	} else {
		/* Release Wake Lock to allow suspend during discharging */
		wake_unlock(&chrgr->suspend_lock);
		cancel_delayed_work(&chrgr->charging_work);
	}
	return 0;
}


static int rt9455_charger_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	int value_to_set, value_set = 0, ret = 0;
	struct rt9455_charger *chrgr =
			i2c_get_clientdata(to_i2c_client(psy->dev->parent));

	bool call_psy_changed = false;

	down(&chrgr->prop_lock);

	if (chrgr->state.status == RT9455_STATUS_FAULT) {
		ret = -EFAULT;
		CHARGER_DEBUG_REL(
				chrgr_dbg, CHG_DBG_SET_PROPERTY_ERROR, ret, 0);

		CHARGER_DEBUG_REL(
				chrgr_dbg, CHG_DBG_DRIVER_IN_FAULT_STATE, 0, 0);
		up(&chrgr->prop_lock);
		return ret;
	}

	switch (psp) {

	case POWER_SUPPLY_PROP_CABLE_TYPE:
		CHARGER_DEBUG_REL(
			chrgr_dbg, CHG_DBG_SET_PROP_CABLE_TYPE, val->intval, 0);

		value_set = val->intval;
		if (chrgr->state.cable_type == val->intval)
			break;
		chrgr->state.cable_type = val->intval;

		if ((chrgr->state.cable_type ==
			POWER_SUPPLY_CHARGER_TYPE_USB_SDP) ||
			(chrgr->state.cable_type ==
			POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING))

			chrgr->current_psy = &chrgr->usb_psy;
		else
			chrgr->current_psy = &chrgr->ac_psy;

		call_psy_changed = true;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_CHARGE_CURRENT,
								val->intval, 0);

		value_to_set = fit_in_range(val->intval, 0,
						chrgr->state.max_cc);

		rt9455_set_iocharge(chrgr, value_to_set, &value_set, false);
		if (value_set == chrgr->state.cc)
			break;
		ret = rt9455_set_iocharge(chrgr, value_to_set,
							&value_set, true);

		if (ret) {
			chrgr->state.status = RT9455_STATUS_FAULT;
			chrgr->state.health =
					POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			break;
		}

		chrgr->state.cc = value_set;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_CHARGE_VOLTAGE,
								val->intval, 0);

		rt9455_set_voreg(chrgr, val->intval, &value_set, false);
		if (value_set == chrgr->state.cv)
			break;
		ret = rt9455_set_voreg(chrgr, val->intval, &value_set, true);
		if (ret) {
			chrgr->state.status = RT9455_STATUS_FAULT;
			chrgr->state.health =
					POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			break;
		}

		chrgr->state.cv = value_set;
		break;

	case POWER_SUPPLY_PROP_ENABLE_CHARGER:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_ENABLE_CHARGER,
								val->intval, 0);

		chrgr->state.charger_enabled = val->intval;
		value_set = val->intval;
		break;

	case POWER_SUPPLY_PROP_ENABLE_CHARGING:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_ENABLE_CHARGING,
								val->intval, 0);

		if (val->intval == chrgr->state.charging_enabled) {
			value_set = val->intval;
			break;
		}
		ret = rt9455_enable_charging(chrgr, val->intval);
		if (!ret) {
			chrgr->state.charging_enabled = val->intval;
			value_set = val->intval;
			call_psy_changed = true;
		}
		break;

	case POWER_SUPPLY_PROP_INLMT:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_INLMT,
							val->intval, 0);

		value_to_set = val->intval;
		ret = rt9455_set_ibus_limit(chrgr, value_to_set, &value_set);
		if (ret) {
			chrgr->state.status = RT9455_STATUS_FAULT;
			chrgr->state.health =
					POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			break;
		}

		chrgr->state.inlmt = value_set;
		if (value_set > chrgr->max_ibus_limit)
			chrgr->state.max_cc =
				fit_in_range(value_set, 0, chrgr->max_iocharge);
		break;

	case POWER_SUPPLY_PROP_CONTINUE_CHARGING:
		CHARGER_DEBUG_REL(
				chrgr_dbg, CHG_DBG_SET_PROP_CONTINUE_CHARGING,
								val->intval, 0);

		chrgr->ack_time = jiffies;
		value_set = val->intval;
		break;

	case POWER_SUPPLY_PROP_CHARGE_TERM_CUR:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_CHARGE_TERM_CUR,
								val->intval, 0);
		chrgr->state.iterm = value_set;
		break;

	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_MIN_TEMP,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_MAX_TEMP,
								val->intval, 0);
		break;

	default:
		break;
	};

	if (!ret)
		CHARGER_DEBUG_REL(
			chrgr_dbg, CHG_DBG_SET_PROPERTY_VALUE_SET,
							value_set, 0);
	else
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROPERTY_ERROR,
							ret, 0);

	up(&chrgr->prop_lock);

	if (call_psy_changed)
		power_supply_changed(psy);

	return ret;
}

static int rt9455_charger_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct rt9455_charger *chrgr =
			i2c_get_clientdata(to_i2c_client(psy->dev->parent));

	down(&chrgr->prop_lock);

	switch (psp) {

	case POWER_SUPPLY_PROP_PRESENT:
		if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = ((chrgr->state.cable_type ==
				POWER_SUPPLY_CHARGER_TYPE_USB_SDP) ||
				(chrgr->state.cable_type ==
				POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING));

		else if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = ((chrgr->state.cable_type ==
				 POWER_SUPPLY_CHARGER_TYPE_USB_DCP) ||
				(chrgr->state.cable_type ==
				 POWER_SUPPLY_CHARGER_TYPE_USB_CDP));
		else
			val->intval = 0;

		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_PRESENT,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = rt9455_is_online(chrgr, psy);
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_ONLINE,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chrgr->state.health;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_HEALTH,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_TYPE:
		val->intval = POWER_SUPPLY_TYPE_USB;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_TYPE,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_CABLE_TYPE:
		val->intval = chrgr->state.cable_type;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_CABLE_TYPE,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_ENABLE_CHARGER:
		val->intval = chrgr->state.charger_enabled;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_ENABLE_CHARGER,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_ENABLE_CHARGING:
		val->intval = chrgr->state.charging_enabled;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_ENABLE_CHARGING,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		val->intval = 0;
		CHARGER_DEBUG_DEV(
			chrgr_dbg, CHG_DBG_GET_PROP_CHARGE_CONTROL_LIMIT,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = chrgr->model_name;
		break;

	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = chrgr->manufacturer;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		val->intval = chrgr->state.cc;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_CHARGE_CURRENT,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		val->intval = chrgr->state.cv;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_CHARGE_VOLTAGE,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = chrgr->state.max_cc;
		CHARGER_DEBUG_DEV(
			chrgr_dbg, CHG_DBG_GET_PROP_MAX_CHARGE_CURRENT,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_INLMT:
		val->intval = chrgr->state.inlmt;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_INLMT,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_CHARGE_TERM_CUR:
		val->intval = chrgr->state.iterm;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_CHARGE_TERM_CUR,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_PRIORITY:
		val->intval = 0;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_PRIORITY,
								val->intval, 0);
		break;

	default:
		ret = -EINVAL;
		break;
	};

	up(&chrgr->prop_lock);

	return ret;
}

static void rt9455_boost_worker(struct work_struct *work)
{
	int ret;
	u8 val;
	struct rt9455_charger *chrgr =
		container_of(work, struct rt9455_charger, boost_work.work);

	down(&chrgr->prop_lock);

	ret = rt9455_attr_read(chrgr->client, BOOST, &val);

	if ((!ret) && val && chrgr->state.boost_enabled) {
		rt9455_trigger_wtd(chrgr);
		schedule_delayed_work(&chrgr->boost_work, BOOST_WORK_DELAY);
	}

	up(&chrgr->prop_lock);
	return;
}

static void rt9455_set_boost(struct work_struct *work)
{
	struct rt9455_charger *chrgr =
		container_of(work, struct rt9455_charger,
					boost_op_bh.work);
	int on = chrgr->state.to_enable_boost;
	int ret = 0;
	u8 chr_reg;

	down(&chrgr->prop_lock);

	/* Clear state flags */
	chrgr->state.boost_ov = 0;
	chrgr->state.bat_uv = 0;

	if (on) {
		/* Enable boost regulator */
		ret = rt9455_attr_write(chrgr->client, OPA_MODE, 1);
		if (ret)
			goto exit_boost;

		/* Boost startup time is 2 ms max */
		mdelay(2);

		/* Ensure Boost is in regulation */
		ret = rt9455_attr_read(chrgr->client, BOOST, &chr_reg);
		if (ret)
			goto exit_boost;

		if (!chr_reg) {
			/*
			 * In case of BOOST fault, BOOST_EN bit is automatically
			 * cleared
			 */
			pr_err("%s: boost mode didn't go in regulation\n",
					__func__);
			ret = -EINVAL;
			goto exit_boost;
		}

		/* Enable boost mode flag */
		chrgr->state.boost_enabled = 1;

		wake_lock(&chrgr->suspend_lock);
		schedule_delayed_work(&chrgr->boost_work, BOOST_WORK_DELAY);
	} else {
		cancel_delayed_work(&chrgr->boost_work);

		/* Release Wake Lock */
		wake_unlock(&chrgr->suspend_lock);

		/* Disable boost mode flag */
		chrgr->state.boost_enabled = 0;

		/* Disable boost regulator */
		ret = rt9455_attr_write(chrgr->client, OPA_MODE, 0);
		if (ret)
			pr_err("%s: fail to disable boost mode\n", __func__);
	}

exit_boost:
	if (on && ret)
		atomic_notifier_call_chain(&chrgr->otg_handle->notifier,
					INTEL_USB_DRV_VBUS_ERR, NULL);
	up(&chrgr->prop_lock);
	return;
}

/**
 * rt9455_chgint_cb_work_func	function executed by
 *				rt9455_charger::chgint_cb_work work
 * @work			[in] pointer to associated 'work' structure
 */
static void rt9455_chgint_cb_work_func(struct work_struct *work)
{
	int ret, vbus_state_prev, health_prev;
	struct rt9455_charger *chrgr =
		container_of(work, struct rt9455_charger,
					chgint_bh.work);

	down(&chrgr->prop_lock);

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_CHGINT_CB, 0, 0);

	if (chrgr->state.status == RT9455_STATUS_FAULT) {
		CHARGER_DEBUG_REL(
				chrgr_dbg, CHG_DBG_DRIVER_IN_FAULT_STATE, 0, 0);
		up(&chrgr->prop_lock);
		pr_err("%s:chrgr->state.status == RT9455_STATUS_FAULT!\n",
						 __func__);
		return;
	}

	vbus_state_prev = chrgr->state.vbus;
	health_prev = chrgr->state.health;
	pr_info("%s: vbus_state_prev=%d, health_prev=%d\n",
				__func__, vbus_state_prev, health_prev);
	msleep(500); /* Wait for charger IC state change */
	ret = chrgr->get_charger_state(chrgr);
	if (ret != 0)
		goto fail;

	if (chrgr->state.boost_enabled) {
		if (chrgr->state.boost_ov || chrgr->state.bat_uv
			|| chrgr->state.tsd_flag || chrgr->state.ovp_flag ||
			 chrgr->state.t32s_timer_expired) {
			/*
			 * In case of BOOST fault, BOOST_EN bit is automatically
			 * cleared. Only need to clear boost enabled sw flag and
			 * stop the booster workqueue
			 */
			atomic_notifier_call_chain(&chrgr->otg_handle->notifier,
					INTEL_USB_DRV_VBUS_ERR, NULL);

			wake_unlock(&chrgr->suspend_lock);
			if (chrgr->state.boost_ov)
				pr_err("%s: boost mode overcurrent detected\n",
						__func__);
			if (chrgr->state.bat_uv)
				pr_err("%s: boost mode under voltage detected\n",
						__func__);

			goto fail;
		}
	}

	if (chrgr->state.treg_flag)
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_TREG_IS_ON, 0, 0);

	if (chrgr->state.ot_recov_flag)
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_OT_RECOVERY, 0, 0);

	chrgr->state.health = (chrgr->state.vbus == VBUS_ON) ?
			POWER_SUPPLY_HEALTH_GOOD : POWER_SUPPLY_HEALTH_UNKNOWN;

	if (chrgr->state.vbus_fault) {
		chrgr->state.health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		pr_err("VBUS POWER_SUPPLY_HEALTH_UNSPEC_FAILURE");
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_VBUS_FAULT, 0, 0);
	}

	if (chrgr->state.ovp_flag || chrgr->state.vbus_ovp) {
		chrgr->state.health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		pr_err("VBUS Over-Voltage Shutdown!");
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_VBUS_OVP, 0, 0);
	}

	if (chrgr->state.tsd_flag) {
		chrgr->state.health = POWER_SUPPLY_HEALTH_OVERHEAT;
		pr_err("Chip Over-temperature Shutdown!");
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_TSD_IS_ON, 0, 0);
	}

	if (chrgr->state.ovp_recov) {
		pr_info("VBUS OVP Recovery occurred!");
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_VBUS_OVP_RECOV, 0, 0);
	}

	if (chrgr->state.poor_input_source) {
		pr_err("Poor Input Source!\n");
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_POOR_INPUT_SOURCE, 0, 0);
	}

	if (chrgr->state.sleep_mode) {
		pr_info("Sleep Mode\n");
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SLEEP, 0, 0);
	}

	if (chrgr->state.bat_ovp) {
		pr_debug("Battery Over-Voltage!");
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_BAT_OVP, 0, 0);
	}

	if (chrgr->state.no_bat) {
		pr_err("No Battery!\n");
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_NO_BAT, 0, 0);
	}

	if (health_prev != chrgr->state.health &&
		chrgr->state.health != POWER_SUPPLY_HEALTH_GOOD &&
		 chrgr->state.health != POWER_SUPPLY_HEALTH_UNKNOWN)
				power_supply_changed(chrgr->current_psy);


	/* when vbus is disconnected all registers are reseted (known HW bug)
	therefore need to reconfigure the chip */
#if 1
	if (chrgr->state.vbus == VBUS_OFF && vbus_state_prev == VBUS_ON) {
		chrgr->state.cc = chrgr->default_cc;
		chrgr->state.cv = chrgr->default_cv;
		ret = rt9455_configure_chip(
					chrgr, chrgr->state.charging_enabled);
		if (ret != 0)
			goto fail;
	}
#endif
	/* If vbus status changed, then notify USB OTG */
	if (chrgr->state.vbus != vbus_state_prev) {
		if (chrgr->otg_handle && (vbus_state_prev >= 0)) {
			atomic_notifier_call_chain(&chrgr->otg_handle->notifier,
					USB_EVENT_VBUS, &chrgr->state.vbus);

			CHARGER_DEBUG_REL(
				chrgr_dbg, CHG_DBG_VBUS, chrgr->state.vbus, 0);
		}
	}
	pr_info("Leave %s: chrgr->state.health=%d, chrgr->state.vbus=%d\n",
				__func__,
				chrgr->state.health, chrgr->state.vbus);
fail:
	up(&chrgr->prop_lock);
}

static int unfreezable_bh_create(struct unfreezable_bh_struct *bh,
		const char *wq_name, const char *wakelock_name,
		void (*work_func)(struct work_struct *work))
{
	spin_lock_init(&bh->lock);

	/* Create private, single-threaded workqueue instead of using one of
	the system predefined workqueues to reduce latency */
	bh->wq = create_singlethread_workqueue(wq_name);

	if (NULL == bh->wq)
		return -ENOMEM;

	INIT_WORK(&bh->work, work_func);

	wake_lock_init(&bh->evt_wakelock,
			WAKE_LOCK_SUSPEND,
			wakelock_name);

	return 0;
}

static void unfreezable_bh_destroy(struct unfreezable_bh_struct *bh)
{
	cancel_work_sync(&bh->work);

	destroy_workqueue(bh->wq);

	wake_lock_destroy(&bh->evt_wakelock);
}

static void unfreezable_bh_schedule(struct unfreezable_bh_struct *bh)
{
	spin_lock(&bh->lock);

	if (!bh->in_suspend) {
		queue_work(bh->wq, &bh->work);

		wake_lock_timeout(&bh->evt_wakelock, EVT_WAKELOCK_TIMEOUT);
	} else {
		bh->pending_evt = true;
	}

	spin_unlock(&bh->lock);
}

static void unfreezable_bh_resume(struct unfreezable_bh_struct *bh)
{
	unsigned long flags;

	spin_lock_irqsave(&bh->lock, flags);

	bh->in_suspend = false;
	if (bh->pending_evt) {
		bh->pending_evt = false;

		queue_work(bh->wq, &bh->work);

		wake_lock_timeout(&bh->evt_wakelock, EVT_WAKELOCK_TIMEOUT);
	}

	spin_unlock_irqrestore(&bh->lock, flags);
}

static void unfreezable_bh_suspend(struct unfreezable_bh_struct *bh)
{
	unsigned long flags;

	spin_lock_irqsave(&bh->lock, flags);
	bh->in_suspend = true;
	spin_unlock_irqrestore(&bh->lock, flags);
}

static irqreturn_t rt9455_charger_chgint_cb(int irq, void *dev)
{
	struct rt9455_charger *chrgr = dev;

	pr_info("%s, irq=%d\n", __func__, irq);

	unfreezable_bh_schedule(&chrgr->chgint_bh);

	return IRQ_HANDLED;
}

/**
 * rt9455_configure_pmu_irq - function configuring PMU's Charger status IRQ
 * @chrgr		[in] pointer to charger driver internal structure
 */
static int rt9455_configure_pmu_irq(struct rt9455_charger *chrgr)
{
	u32 regval;
	int ret;

	/* register callback with PMU for CHGINT irq */
	ret = request_irq(chrgr->irq,
		rt9455_charger_chgint_cb,
			IRQF_NO_SUSPEND, RT9455_NAME, chrgr);
	if (ret != 0) {
		pr_err("Failed to register @PMU for CHGINT irq! ret=%d", ret);
		return ret;
	}
#if USE_CDET_IRQ
	ret = request_irq(chrgr->irq_det,
		rt9455_charger_chgint_cb,
			IRQF_NO_SUSPEND, "rt9455_det", chrgr);
	if (ret != 0) {
		pr_err("Failed to register @PMU for CHG DET irq! ret=%d", ret);
		return ret;
	}
#endif
	return 0;
}

#define IRQSM_CDET_O 8
static int rt9455_configure_pmu_irqsm(struct rt9455_charger *chrgr)
{
	int ret = 0;
	u32 regval;
	void __iomem *irqsm_io;
	struct resource *res;
	struct idi_peripheral_device *ididev;
	ididev = chrgr->ididev;
	res = idi_get_resource_byname(&ididev->resources,
			IORESOURCE_MEM, "pmu_c1_irqsm");
	if (res == NULL) {
		pr_err("getting PMU's C1_IRQSM registers resources failed!\n");
		return -EINVAL;
	}
	irqsm_io = ioremap(res->start, resource_size(res));
	if (!irqsm_io) {
		pr_err("mapping PMU's C1_IRQSM registers failed!\n");
		return -EINVAL;
	}
	regval = ioread32(irqsm_io);
	pr_info("%s: irqsm_io = 0x%x, regval=0x%x\n",
		__func__, irqsm_io, regval);
	regval |= (1 << IRQSM_CDET_O);
	iowrite32(regval, irqsm_io);

	return ret;
}

/**
 * rt9455_configure_pmu_regs -function configuring PMU's Charger
 *				part registers
 * @chrgr			[in] pointer to charger driver's internal struct
 */
static int rt9455_configure_pmu_regs(struct rt9455_charger *chrgr)
{
	u32 regval;
	struct device_state_pm_state *pm_state_en, *pm_state_dis;
	int ret;


	if (!chrgr->ctrl_io || !chrgr->ididev)
		return -EINVAL;

	pm_state_en =
		idi_peripheral_device_pm_get_state_handler(
						chrgr->ididev, "enable");
	if (pm_state_en == NULL) {
		pr_err("Unable to get handler for PM state 'enable'!\n");
		return -EINVAL;
	}

	pm_state_dis =
		idi_peripheral_device_pm_get_state_handler(
						chrgr->ididev, "disable");
	if (pm_state_dis == NULL) {
		pr_err("Unable to get handler for PM state 'disable'!\n");
		return -EINVAL;
	}

	pr_info("Getting PM state handlers: OK\n");

	ret = idi_set_power_state(chrgr->ididev, pm_state_en, true);

	if (ret) {
		pr_err("setting PM state '%s' failed!\n", pm_state_en->name);
		return -EIO;
	}

	regval = ioread32(CHARGER_CONTROL(chrgr->ctrl_io));
	pr_info("%s: io=0x%x, regval=0x%x\n", __func__, chrgr->ctrl_io, regval);
	/* ChargerIRQEdge - CHARGER_CONTROL_CIEDG_FALLING */
	regval &= ~(CHARGER_CONTROL_CIEDG_M << CHARGER_CONTROL_CIEDG_O);
	regval |= (CHARGER_CONTROL_CIEDG_FALLING << CHARGER_CONTROL_CIEDG_O);

	/* ChargerIRQLevel - CHARGER_CONTROL_CILVL_LOW */
	regval &= ~(CHARGER_CONTROL_CILVL_M << CHARGER_CONTROL_CILVL_O);
	regval |= (CHARGER_CONTROL_CILVL_LOW << CHARGER_CONTROL_CILVL_O);

	/* ChargerIRQSensibility - CHARGER_CONTROL_CISENS_EDGE */
	regval &= ~(CHARGER_CONTROL_CISENS_M << CHARGER_CONTROL_CISENS_O);
	regval |= (CHARGER_CONTROL_CISENS_EDGE << CHARGER_CONTROL_CISENS_O);

	/* ChargerIRQEnable - CHARGER_CONTROL_CIEN_EN */
	regval &= ~(CHARGER_CONTROL_CIEN_M << CHARGER_CONTROL_CIEN_O);
	regval |= (CHARGER_CONTROL_CIEN_EN << CHARGER_CONTROL_CIEN_O);

	/* ChargerResetLevel - CHARGER_CONTROL_CHGLVL_LOW */
	regval &= ~(CHARGER_CONTROL_CHGLVL_M << CHARGER_CONTROL_CHGLVL_O);
	regval |= (CHARGER_CONTROL_CHGLVL_LOW << CHARGER_CONTROL_CHGLVL_O);

	/* ChargerIRQDebounce - CHARGER_CONTROL_IRQ_DEBOUNCE_DISABLE - as vbus
	signal is clear enough (not bouncing) */
	regval &= ~(CHARGER_CONTROL_CIDBT_M << CHARGER_CONTROL_CIDBT_O);
	regval |= (CHARGER_CONTROL_IRQ_DEBOUNCE_DISABLE <<
				CHARGER_CONTROL_CIDBT_O);

#if USE_CDET_IRQ
	/* ChargerDetecSensibility - EDGE */
	regval &= ~(CHARGER_CONTROL_CDETSENS_M << CHARGER_CONTROL_CDETSENS_O);
	regval |= (CHARGER_CONTROL_CDETSENS_EDGE << CHARGER_CONTROL_CDETSENS_O);
#endif
	/* ChargerIRQResponse - SWIRQ */
	regval &= ~(CHARGER_CONTROL_CIRSP_M << CHARGER_CONTROL_CIRSP_O);

	iowrite32(regval, CHARGER_CONTROL(chrgr->ctrl_io));

	/* charger control WR strobe */
	iowrite32((1 << CHARGER_CONTROL_WR_WS_O),
					CHARGER_CONTROL_WR(chrgr->ctrl_io));
	iowrite32((1 << CHARGER_CONTROL_WR_WS_O),
					CHARGER_CONTROL_WR(chrgr->ctrl_io));

	/* Set CHGRST ball to low for MRD P1 (RT9455) */
	/* charger WR */
	iowrite32((1 << CHARGER_WR_WS_O), CHARGER_WR(chrgr->ctrl_io));

#if USE_CDET_IRQ
	ret = rt9455_configure_pmu_irqsm(chrgr);
	if (ret)
		pr_err("rt9455_configure_pmu_irqsm fail!\n");
#endif
	ret = idi_set_power_state(chrgr->ididev, pm_state_dis, false);

	if (ret)
		pr_err("setting PM state '%s' failed!\n", pm_state_dis->name);

	return 0;
}

/**
 * rt9455_get_clr_wdt_expiry_flag -	func. gets WDT expiry status and clears
 *					expired status if it had expired
 * @chrgr		[in] pointer to charger driver internal structure
 */
static int rt9455_get_clr_wdt_expiry_flag(struct rt9455_charger *chrgr)
{
	if (chrgr->get_clr_wdt_expiry_flag)
		return chrgr->get_clr_wdt_expiry_flag(chrgr);
	else
		return 0;
}


/**
 * rt9455_configure_chip - function configuring RT9455 chip registers
 * @chrgr		[in] pointer to charger driver internal structure
 * @enable_charging	[in] controls if charging should be anebled or disabled
 */
static int rt9455_configure_chip(struct rt9455_charger *chrgr,
							bool enable_charging)
{
	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_CONFIGURE_CHIP, 0, 0);

	if (chrgr->configure_chip)
		return chrgr->configure_chip(chrgr, enable_charging);
	else
		return -EINVAL;
}

static int rt9455_set_pinctrl_state(struct i2c_client *client,
		struct pinctrl_state *state)
{
	struct rt9455_charger *chrgr = i2c_get_clientdata(client);
	int ret;

	ret = pinctrl_select_state(chrgr->pinctrl, state);
	if (ret != 0) {
		pr_err("failed to configure CHGRESET pin !\n");
		return -EIO;
	}

	return 0;
}

int rt9455_otg_notification_handler(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct rt9455_charger *chrgr =
		container_of(nb, struct rt9455_charger, otg_nb);
	pr_info("%s: event=%d\n", __func__, event);
	switch (event) {
	case INTEL_USB_DRV_VBUS:
		if (!data)
			return NOTIFY_BAD;
		chrgr->state.to_enable_boost = *((bool *)data);
		pr_info("set chrgr->state.to_enable_boost=%d\n",
				chrgr->state.to_enable_boost);
		unfreezable_bh_schedule(&chrgr->boost_op_bh);
		break;

	default:
		break;
	}

	return NOTIFY_OK;
}

static ssize_t dbg_logs_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	size_t size_copied;
	int value;

	value = chrgr_dbg.printk_logs_en;
	size_copied = sprintf(buf, "%d\n", value);

	return size_copied;
}

static ssize_t dbg_logs_store(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	int sysfs_val;
	int ret;
	size_t size_to_cpy;
	char strvalue[SYSFS_INPUT_VAL_LEN + 1];

	size_to_cpy = (count > SYSFS_INPUT_VAL_LEN) ?
						SYSFS_INPUT_VAL_LEN : count;
	strncpy(strvalue, buf, size_to_cpy);
	strvalue[size_to_cpy] = '\0';

	ret = kstrtoint(strvalue, 10, &sysfs_val);
	if (ret != 0)
		return ret;

	sysfs_val = (sysfs_val == 0) ? 0 : 1;

	chrgr_dbg.printk_logs_en = sysfs_val;

	pr_info("sysfs attr %s=%d\n", attr->attr.name, sysfs_val);

	return count;
}

static struct device_attribute dbg_logs_on_off_attr = {
	.attr = {
		.name = "dbg_logs_on_off",
		.mode = S_IRUSR | S_IWUSR,
	},
	.show = dbg_logs_show,
	.store = dbg_logs_store,
};

/**
 * rt9455_setup_dbglogs_sysfs_attr	Sets up dbg_logs_on_off sysfs entry
 *					for debug logs control for rt9455
 *					i2c device
 * @dev					[in] pointer to device structure
 *
 */
static void rt9455_setup_dbglogs_sysfs_attr(struct device *dev)
{
	int err;

	err = device_create_file(dev, &dbg_logs_on_off_attr);
	if (err)
		pr_err("Unable to create sysfs entry: '%s'\n",
				dbg_logs_on_off_attr.attr.name);
}


static int rt9455_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct usb_phy *otg_handle;
	struct rt9455_charger *chrgr;
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	u8 vendor_info, rev_info;
	int ret;

	INIT_CHARGER_DEBUG_ARRAY(chrgr_dbg);

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_I2C_PROBE, 0, 0);

	chrgr = (struct rt9455_charger *)id->driver_data;
	if (!chrgr)
		return -ENODEV;

	/* Get pinctrl configurations */
	chrgr->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(chrgr->pinctrl))
		BUG();

	chrgr->pins_default = pinctrl_lookup_state(chrgr->pinctrl,
						PINCTRL_STATE_DEFAULT);
	if (IS_ERR(chrgr->pins_default))
		dev_err(dev, "could not get default pinstate\n");

	chrgr->pins_sleep = pinctrl_lookup_state(chrgr->pinctrl,
						PINCTRL_STATE_SLEEP);
	if (IS_ERR(chrgr->pins_sleep))
		dev_err(dev, "could not get sleep pinstate\n");

	chrgr->pins_inactive = pinctrl_lookup_state(chrgr->pinctrl,
						"inactive");
	if (IS_ERR(chrgr->pins_inactive))
		dev_err(dev, "could not get inactive pinstate\n");

	chrgr->pins_active = pinctrl_lookup_state(chrgr->pinctrl,
						"active");
	if (IS_ERR(chrgr->pins_active))
		dev_err(dev, "could not get active pinstate\n");

	/* Register i2c clientdata before calling pinctrl */
	i2c_set_clientdata(client, chrgr);

	pr_info("%s\n", __func__);

	chrgr->client = client;

	otg_handle = usb_get_phy(USB_PHY_TYPE_USB2);
	if (otg_handle == NULL) {
		pr_err("ERROR!: getting OTG transceiver failed\n");
		return -EINVAL;
	}
	chrgr->otg_handle = otg_handle;

	/* Wait for rt9455 idi device to map io */
	if (chrgr->ctrl_io == NULL)
		BUG();

	/*
	 * Get interrupt from device tree
	 */
	chrgr->irq = irq_of_parse_and_map(np, 0);
	if (!chrgr->irq) {
		ret = -EINVAL;
		pr_err("can't get irq\n");
		goto remap_fail;
	}
#if USE_CDET_IRQ
	chrgr->irq_det = irq_of_parse_and_map(np, 1);
	if (!chrgr->irq_det) {
		ret = -EINVAL;
		pr_err("can't get irq_det\n");
		goto remap_fail;
	}
#endif
	client->irq = chrgr->irq;

	/* Setup the PMU registers. The charger IC reset line
	is deasserted at this point. From this point onwards
	the charger IC will act upon the values stored in its
	registers instead of displaying default behaviour  */
	ret = rt9455_configure_pmu_regs(chrgr);
	if (ret != 0)
		goto pre_fail;

	/* Check if the HW is supported */
	rt9455_attr_read(client, VENDOR_ID, &vendor_info);
	rt9455_attr_read(client, CHIP_REV, &rev_info);
	pr_info("Read charger IC info: vendor=%d, rev=%d\n",
					vendor_info, rev_info);

	INIT_DELAYED_WORK(&chrgr->charging_work, rt9455_charging_worker);
	INIT_DELAYED_WORK(&chrgr->boost_work, rt9455_boost_worker);

	sema_init(&chrgr->prop_lock, 1);

	/* Set up the wake lock to prevent suspend when charging. */
	wake_lock_init(&chrgr->suspend_lock,
			WAKE_LOCK_SUSPEND,
			"rt9455_wake_lock");

	if (unfreezable_bh_create(&chrgr->chgint_bh, "chrgr_wq",
			"rt9455_evt_lock", rt9455_chgint_cb_work_func)) {
		ret = -ENOMEM;
		goto wq_creation_fail;
	}

	ret = rt9455_configure_chip(chrgr, chrgr->state.charging_enabled);
	if (ret != 0)
		goto fail;

	ret = rt9455_set_pinctrl_state(client, chrgr->pins_active);
	if (ret != 0)
		return ret;

	chrgr->usb_psy.name           = "usb_charger";
	chrgr->usb_psy.type           = POWER_SUPPLY_TYPE_USB;
	chrgr->usb_psy.properties     = rt9455_power_props;
	chrgr->usb_psy.num_properties = ARRAY_SIZE(rt9455_power_props);
	chrgr->usb_psy.get_property   = rt9455_charger_get_property;
	chrgr->usb_psy.set_property   = rt9455_charger_set_property;
	chrgr->usb_psy.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB_SDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_DCP |
					POWER_SUPPLY_CHARGER_TYPE_USB_CDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING;
	chrgr->usb_psy.supplied_to = rt9455_supplied_to;
	chrgr->usb_psy.num_supplicants = ARRAY_SIZE(rt9455_supplied_to);
	chrgr->usb_psy.throttle_states = rt9455_dummy_throttle_states;
	chrgr->usb_psy.num_throttle_states =
				ARRAY_SIZE(rt9455_dummy_throttle_states);

	chrgr->current_psy = &chrgr->usb_psy;

	ret = power_supply_register(&client->dev, &chrgr->usb_psy);
	if (ret)
		goto fail;

	chrgr->ac_psy.name           = "ac_charger";
	chrgr->ac_psy.type           = POWER_SUPPLY_TYPE_MAINS;
	chrgr->ac_psy.properties     = rt9455_power_props;
	chrgr->ac_psy.num_properties = ARRAY_SIZE(rt9455_power_props);
	chrgr->ac_psy.get_property   = rt9455_charger_get_property;
	chrgr->ac_psy.set_property   = rt9455_charger_set_property;
	chrgr->ac_psy.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB_SDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_DCP |
					POWER_SUPPLY_CHARGER_TYPE_USB_CDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING;
	chrgr->ac_psy.supplied_to = rt9455_supplied_to;
	chrgr->ac_psy.num_supplicants = ARRAY_SIZE(rt9455_supplied_to);
	chrgr->ac_psy.throttle_states = rt9455_dummy_throttle_states;
	chrgr->ac_psy.num_throttle_states =
				ARRAY_SIZE(rt9455_dummy_throttle_states);

	ret = power_supply_register(&client->dev, &chrgr->ac_psy);
	if (ret)
		goto fail_ac_registr;


	chrgr->ack_time = jiffies;

	if (chrgr->state.charging_enabled)
		schedule_delayed_work(&chrgr->charging_work, 0);

	ret = rt9455_configure_pmu_irq(chrgr);
	if (ret != 0)
		goto pmu_irq_fail;

	i2c_set_clientdata(client, chrgr);

	rt9455_setup_debugfs(chrgr, &chrgr_dbg);
	rt9455_setup_fake_vbus_sysfs_attr(chrgr);

	chrgr->state.status = RT9455_STATUS_READY;

	/* Read the VBUS presence status for initial update by
	making a dummy interrupt bottom half invocation */
	queue_work(chrgr->chgint_bh.wq, &chrgr->chgint_bh.work);

	if (unfreezable_bh_create(&chrgr->boost_op_bh, "boost_op_wq",
			"rt9455_boost_lock", rt9455_set_boost)) {
		ret = -ENOMEM;
		goto pmu_irq_fail;
	}

	ret = usb_register_notifier(otg_handle, &chrgr->otg_nb);
	if (ret) {
		pr_err("ERROR!: registration for OTG notifications failed\n");
		goto boost_fail;
	}

	rt9455_setup_dbglogs_sysfs_attr(&client->dev);

	device_init_wakeup(&client->dev, true);

	return 0;

boost_fail:
	unfreezable_bh_destroy(&chrgr->boost_op_bh);
pmu_irq_fail:
	power_supply_unregister(&chrgr->ac_psy);
fail_ac_registr:
	power_supply_unregister(&chrgr->usb_psy);
fail:
	unfreezable_bh_destroy(&chrgr->chgint_bh);
pre_fail:
wq_creation_fail:
remap_fail:
	usb_put_phy(otg_handle);
	return ret;
}

static int __exit rt9455_i2c_remove(struct i2c_client *client)
{
	int ret = 0;
	struct rt9455_charger *chrgr = i2c_get_clientdata(client);

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_I2C_REMOVE, 0, 0);
	pr_info("%s\n", __func__);
	free_irq(client->irq, chrgr);
	power_supply_unregister(&chrgr->usb_psy);
	power_supply_unregister(&chrgr->ac_psy);
	wake_lock_destroy(&chrgr->suspend_lock);

	unfreezable_bh_destroy(&chrgr->chgint_bh);
	unfreezable_bh_destroy(&chrgr->boost_op_bh);

	cancel_delayed_work_sync(&chrgr->charging_work);
	if (chrgr->otg_handle)
		usb_put_phy(chrgr->otg_handle);

	ret = rt9455_set_pinctrl_state(client, chrgr->pins_inactive);
	if (ret != 0)
		return ret;
	rt9455_remove_debugfs_dir(chrgr);

	return 0;
}

static void idi_init(struct rt9455_charger *chrgr,
			void *__iomem ctrl_io,
			struct idi_peripheral_device *ididev)
{
	chrgr->ctrl_io = ctrl_io;
	chrgr->ididev = ididev;
}

static void idi_remove(struct rt9455_charger *chrgr)
{
	iounmap(chrgr->ctrl_io);
}

static int rt9455_idi_probe(struct idi_peripheral_device *ididev,
					const struct idi_device_id *id)
{
	struct resource *res;
	void __iomem *ctrl_io;
	int ret = 0;

	spin_lock_init(&chrgr_dbg.lock);

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_IDI_PROBE, 0, 0);
	pr_info("%s\n", __func__);

	res = idi_get_resource_byname(&ididev->resources,
				IORESOURCE_MEM, "registers");

	if (res == NULL) {
		pr_err("getting PMU's Charger registers resources failed!\n");
		return -EINVAL;
	}

	ctrl_io = ioremap(res->start, resource_size(res));

	if (!ctrl_io) {
		pr_err("mapping PMU's Charger registers failed!\n");
		return -EINVAL;
	}

	idi_init(&rt9455_chrgr_data, ctrl_io, ididev);

	ret = idi_device_pm_set_class(ididev);
	if (ret) {
		pr_err("%s: Unable to register for generic pm class\n",
								__func__);
		return ret;
	}

	return 0;
}

static int __exit rt9455_idi_remove(struct idi_peripheral_device *ididev)
{
	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_IDI_REMOVE, 0, 0);
	pr_info("%s\n", __func__);

	idi_remove(&rt9455_chrgr_data);

	return 0;
}

/**
 * rt9455_suspend() - Called when the system is attempting to suspend.
 * If charging is in progress EBUSY is returned to abort the suspend and
 * an error is logged, as the wake lock should prevent the situation.
 * @dev		[in] Pointer to the device.(not used)
 * returns	EBUSY if charging is ongoing, else 0
 */
static int rt9455_suspend(struct device *dev)
{
	struct rt9455_charger *chrgr = i2c_get_clientdata(to_i2c_client(dev));

	if (chrgr->state.charging_enabled) {
		/* If charging is in progess, prevent suspend. */
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SUSPEND_ERROR, 0, 0);
		return -EBUSY;
	} else {
		/* Not charging - allow suspend. */
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SUSPEND_OK, 0, 0);
		if (device_may_wakeup(dev)) {
			pr_info("rt: enable wakeirq\n");
			enable_irq_wake(chrgr->irq);
			enable_irq_wake(chrgr->irq_det);
		}
		unfreezable_bh_suspend(&chrgr->chgint_bh);
		unfreezable_bh_suspend(&chrgr->boost_op_bh);
		return 0;
	}
}

/**
 * rt9455_resume() - Called when the system is resuming from suspend.
 * @dev		[in] Pointer to the device.(not used)
 * returns	0
 */
static int rt9455_resume(struct device *dev)
{
	struct rt9455_charger *chrgr = i2c_get_clientdata(to_i2c_client(dev));

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_RESUME, 0, 0);

	unfreezable_bh_resume(&chrgr->chgint_bh);
	unfreezable_bh_resume(&chrgr->boost_op_bh);
	if (device_may_wakeup(dev)) {
		pr_info("rt: disable wakeirq\n");
		disable_irq_wake(chrgr->irq);
		disable_irq_wake(chrgr->irq_det);
	}

	return 0;
}

const struct dev_pm_ops rt9455_pm = {
	.suspend = rt9455_suspend,
	.resume = rt9455_resume,
};


static const struct i2c_device_id rt9455_id[] = {
	{ "rt9455_charger", (kernel_ulong_t)&rt9455_chrgr_data },
	{ }
};

MODULE_DEVICE_TABLE(i2c, rt9455_id);

static struct i2c_driver rt9455_i2c_driver = {
	.probe          = rt9455_i2c_probe,
	.remove         = rt9455_i2c_remove,
	.id_table       = rt9455_id,
	.driver = {
		.name   = RT9455_NAME,
		.owner  = THIS_MODULE,
		.pm = &rt9455_pm,
	},
};

static const struct idi_device_id idi_ids[] = {
	{
		.vendor = IDI_ANY_ID,
		.device = IDI_DEVICE_ID_INTEL_AG620,
		.subdevice = IDI_SUBDEVICE_ID_INTEL_CHG,
	},

	{ /* end: all zeroes */},
};

static struct idi_peripheral_driver rt9455_idi_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = "chgr_idi",
		.pm = NULL,
	},
	.p_type = IDI_CHG,
	.id_table = idi_ids,
	.probe  = rt9455_idi_probe,
	.remove = rt9455_idi_remove,
};


static int __init rt9455_init(void)
{
	int ret;

	ret = idi_register_peripheral_driver(&rt9455_idi_driver);
	if (ret)
		return ret;


	ret = i2c_add_driver(&rt9455_i2c_driver);
	if (ret)
		return ret;

	return 0;
}

late_initcall(rt9455_init);

static void __exit rt9455_exit(void)
{
	i2c_del_driver(&rt9455_i2c_driver);
}
module_exit(rt9455_exit);


MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Charger Driver for RT9455 charger IC");
