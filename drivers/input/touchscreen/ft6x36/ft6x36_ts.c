/* drivers/input/touchscreen/ft6x36_ts.c
 *
 * FocalTech ft6x36 TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/hrtimer.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>

//#define FTS_CTL_FACE_DETECT
#define FTS_CTL_IIC
#define SYSFS_DEBUG
#define FTS_APK_DEBUG
//#define ft6x36_DOWNLOAD
//#define FTS_FW_AUTO_UPGRADE

#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif
#ifdef FTS_CTL_FACE_DETECT
#include "ft_psensor_drv.h"
#endif
#ifdef SYSFS_DEBUG
#include "ft6x36_ex_fun.h"
#endif
#include "ft6x36_ts.h"

struct ts_event {
	u16 au16_x[CFG_MAX_TOUCH_POINTS];	/*x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];	/*y coordinate */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];	/*touch event:
					0 -- down; 1-- up; 2 -- contact */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];	/*touch ID */
	u16 pressure;
	u8 touch_point;
};

struct ft6x36_ts_data {
	unsigned int irq;
	unsigned int x_max;
	unsigned int y_max;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	struct ft6x36_platform_data *pdata;
#ifdef CONFIG_PM
	struct early_suspend *early_suspend;
#endif
};

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
struct device_state_pm_state ft6x36_pm_states[] = {
	{.name = "disable", }, /* D3 */
	{.name = "enable", }, /* D0 */
};
#define FT6X36_STATE_D0		1
#define FT6X36_STATE_D3		0

/* Touchscreen PM states & class */
static int ft6x36_set_pm_state(struct device *dev,
		struct device_state_pm_state *state)
{
	struct ft6x36_platform_data *ft6x36_pdata = dev->platform_data;
	int id = device_state_pm_get_state_id(dev, state->name);
	int ret = 0;

	switch (id) {
	case FT6X36_STATE_D0:
		if (ft6x36_pdata->power)
			ret = regulator_enable(ft6x36_pdata->power);
		if (ret)
			return ret;
		mdelay(50);

		if (ft6x36_pdata->power2)
			ret = regulator_enable(ft6x36_pdata->power2);
		if (ret)
			return ret;
		mdelay(50);

		break;

	case FT6X36_STATE_D3:
		if (ft6x36_pdata->power)
			regulator_disable(ft6x36_pdata->power);

		if (ft6x36_pdata->power2)
			regulator_disable(ft6x36_pdata->power2);

		break;

	default:
		return id;
	}

	return 0;
}

static struct device_state_pm_state *ft6x36_get_initial_state(
		struct device *dev)
{
	return &ft6x36_pm_states[FT6X36_STATE_D3];
}

static struct device_state_pm_ops ft6x36_pm_ops = {
	.set_state = ft6x36_set_pm_state,
	.get_initial_state = ft6x36_get_initial_state,
};

DECLARE_DEVICE_STATE_PM_CLASS(ft6x36);
#endif

#define FTS_POINT_UP		0x01
#define FTS_POINT_DOWN		0x00
#define FTS_POINT_CONTACT	0x02

#ifdef CONFIG_OF

#define OF_FT6X36_SUPPLY	"i2c"
#define OF_FT6X36_SUPPLY_A	"i2ca"
#define OF_FT6X36_PIN_RESET	"intel,ts-gpio-reset"

static struct ft6x36_platform_data *ft6x36_ts_of_get_platdata(
		struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct ft6x36_platform_data *ft6x36_pdata;
	struct regulator *pow_reg;
	int ret;

	ft6x36_pdata = devm_kzalloc(&client->dev,
			sizeof(*ft6x36_pdata), GFP_KERNEL);
	if (!ft6x36_pdata)
		return ERR_PTR(-ENOMEM);

	/* regulator */
	pow_reg = regulator_get(&client->dev, OF_FT6X36_SUPPLY);
	ft6x36_pdata->power = pow_reg;
	if (IS_ERR(pow_reg)) {
		dev_warn(&client->dev, "%s can't get %s-supply handle\n",
			np->name, OF_FT6X36_SUPPLY);
		ft6x36_pdata->power = NULL;
	}

	pow_reg = regulator_get(&client->dev, OF_FT6X36_SUPPLY_A);
	ft6x36_pdata->power2 = pow_reg;
	if (IS_ERR(pow_reg)) {
		dev_warn(&client->dev, "%s can't get %s-supply handle\n",
			np->name, OF_FT6X36_SUPPLY_A);
		ft6x36_pdata->power2 = NULL;
	}

	/* pinctrl */
	ft6x36_pdata->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(ft6x36_pdata->pinctrl)) {
		ret = PTR_ERR(ft6x36_pdata->pinctrl);
		goto out;
	}

	ft6x36_pdata->pins_default = pinctrl_lookup_state(
			ft6x36_pdata->pinctrl, PINCTRL_STATE_DEFAULT);
	if (IS_ERR(ft6x36_pdata->pins_default))
		dev_err(&client->dev, "could not get default pinstate\n");

	ft6x36_pdata->pins_sleep = pinctrl_lookup_state(
			ft6x36_pdata->pinctrl, PINCTRL_STATE_SLEEP);
	if (IS_ERR(ft6x36_pdata->pins_sleep))
		dev_err(&client->dev, "could not get sleep pinstate\n");

	ft6x36_pdata->pins_inactive = pinctrl_lookup_state(
			ft6x36_pdata->pinctrl, "inactive");
	if (IS_ERR(ft6x36_pdata->pins_inactive))
		dev_err(&client->dev, "could not get inactive pinstate\n");

	/* gpio reset */
	ft6x36_pdata->reset = of_get_named_gpio_flags(client->dev.of_node,
			OF_FT6X36_PIN_RESET, 0, NULL);
	if (ft6x36_pdata->reset <= 0) {
		dev_err(&client->dev,
			"error getting gpio for %s\n", OF_FT6X36_PIN_RESET);
		ret = -EINVAL;
		goto out;
	}

	/* interrupt mode */
	if (of_property_read_bool(np, "intel,polling-mode"))
		ft6x36_pdata->polling_mode = true;

	/* pm */
	ft6x36_pdata->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(ft6x36_pdata->pm_platdata)) {
		dev_warn(&client->dev, "Error during device state pm init\n");
		ret = PTR_ERR(ft6x36_pdata->pm_platdata);
		goto out;
	}

	return ft6x36_pdata;

out:
	return ERR_PTR(ret);
}
#endif

static inline int ft6x36_set_pinctrl_state(struct device *dev,
		struct pinctrl_state *state)
{
	struct ft6x36_platform_data *pdata = dev_get_platdata(dev);
	int ret = 0;

	if (!IS_ERR(state)) {
		ret = pinctrl_select_state(pdata->pinctrl, state);
		if (ret)
			dev_err(dev, "%d:could not set pins\n", __LINE__);
	}

	return ret;
}

/*
*ft6x36_i2c_Read-read data and write data by i2c
*@client: handle of i2c
*@writebuf: Data that will be written to the slave
*@writelen: How many bytes to write
*@readbuf: Where to store data read from slave
*@readlen: How many bytes to read
*
*Returns negative errno, else the number of messages executed
*
*
*/
int ft6x36_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}
/*write data by i2c*/
int ft6x36_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);

	return ret;
}

/*Read touch point information when the interrupt  is asserted.*/
static int ft6x36_read_Touchdata(struct ft6x36_ts_data *data)
{
	struct ts_event *event = &data->event;
	u8 buf[POINT_READ_BUF] = { 0 };
	int ret = -1;
	int i = 0;
	u8 pointid = FT_MAX_ID;

	ret = ft6x36_i2c_Read(data->client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		dev_err(&data->client->dev, "%s read touchdata failed.\n",
			__func__);
		return ret;
	}
	memset(event, 0, sizeof(struct ts_event));

	//event->touch_point = buf[2] & 0x0F;

	//event->touch_point = 0;
	
	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++)
	{
		pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		if (pointid >= FT_MAX_ID)
			break;
		else
			event->touch_point++;
		event->au16_x[i] =
		    (s16) (buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i];
		event->au16_y[i] =
		    (s16) (buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i];
		event->au8_touch_event[i] =
		    buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;
		event->au8_finger_id[i] =
		    (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
	}
	
	//event->pressure = FT_PRESS;

	return 0;
}

/*
*report the point information
*/
static void ft6x36_report_value(struct ft6x36_ts_data *data)
{
	struct ts_event *event = &data->event;
	int i = 0;
	int up_point = 0;

	for (i = 0; i < event->touch_point; i++) 
	{
		if(data->x_max > event->au16_x[i] && data->y_max > event->au16_y[i])
		{
			input_mt_slot(data->input_dev, event->au8_finger_id[i]);

		if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2)
			{
				input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER,
					true);
				input_report_abs(data->input_dev, ABS_MT_PRESSURE,
					0x3f);
				input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
					0x05);
				input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					event->au16_x[i]);
				input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					event->au16_y[i]);

			}
			else
			{
				up_point++;
				input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER,
					false);
			}				
			
		}
		else
		{
			if ((event->au16_y[i] == 1340) && (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2))
			{
				if (event->au16_x[i] >= 500 &&  event->au16_x[i] <= 600)
				{
					input_report_key(data->input_dev, KEY_BACK, 1);
				}
				else if (event->au16_x[i] >= 310 &&  event->au16_x[i] <= 400)
				{
					input_report_key(data->input_dev, KEY_HOME, 1);
				}
				else if (event->au16_x[i] >= 100 &&  event->au16_x[i] <= 220)
				{
					input_report_key(data->input_dev, KEY_MENU, 1);
				}
			}
			else if (event->au16_y[i] == 1340)
			{
				if (event->au16_x[i] >= 500 &&  event->au16_x[i] <= 600)
				{
					input_report_key(data->input_dev, KEY_BACK, 0);
				}
				else if (event->au16_x[i] >= 310 &&  event->au16_x[i] <= 400)
				{
					input_report_key(data->input_dev, KEY_HOME, 0);
				}
				else if (event->au16_x[i] >= 100 &&  event->au16_x[i] <= 220)
				{
					input_report_key(data->input_dev, KEY_MENU, 0);	
				}
				up_point++;		
			}
			input_sync(data->input_dev);
			return;
		}
	}
	
	if(event->touch_point == up_point)
		input_report_key(data->input_dev, BTN_TOUCH, 0);
	else
		input_report_key(data->input_dev, BTN_TOUCH, 1);

	input_sync(data->input_dev);

}

/*The ft6x36 device will signal the host about TRIGGER_FALLING.
*Processed when the interrupt is asserted.
*/
static irqreturn_t ft6x36_ts_interrupt(int irq, void *dev_id)
{
	struct ft6x36_ts_data *ft6x36_ts = dev_id;
	int ret = 0;
	disable_irq_nosync(ft6x36_ts->irq);

	ret = ft6x36_read_Touchdata(ft6x36_ts);
	if (ret == 0)
		ft6x36_report_value(ft6x36_ts);

	enable_irq(ft6x36_ts->irq);

	//printk(KERN_WARNING "interrupt \n");

	return IRQ_HANDLED;
}

static int ft6x36_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{

	struct ft6x36_ts_data *ft6x36_ts;
	struct input_dev *input_dev;
	struct ft6x36_platform_data *pdata;
	int err = 0;
	unsigned char uc_reg_value;
	unsigned char uc_reg_addr;

	dev_dbg(&client->dev, "FT6X36 touchscreen driver probing\n");

#ifdef CONFIG_OF
	pdata = client->dev.platform_data =
		ft6x36_ts_of_get_platdata(client);
	if (IS_ERR(pdata)) {
		err = PTR_ERR(pdata);
		return err;
	}
#else
	pdata = (struct ft6x36_platform_data *)client->dev.platform_data;
#endif

	ft6x36_set_pinctrl_state(&client->dev, pdata->pins_default);

	if (pdata->pm_platdata) {
		err = device_state_pm_set_class(&client->dev,
			pdata->pm_platdata->pm_user_name);
		if (err) {
			dev_err(&client->dev,
				"Error while setting the pm class\n");
			goto exit_pm_class;
		}

		err = device_state_pm_set_state_by_name(&client->dev,
				pdata->pm_platdata->pm_state_D0_name);
		if (err)
			goto exit_pm_class;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ft6x36_ts = kzalloc(sizeof(struct ft6x36_ts_data), GFP_KERNEL);

	if (!ft6x36_ts) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}
	
	dev_dbg(&client->dev, "%s: i2c_set_clientdata\n", __func__);
	i2c_set_clientdata(client, ft6x36_ts);
	ft6x36_ts->irq = client->irq;
	ft6x36_ts->client = client;
	ft6x36_ts->pdata = pdata;
	ft6x36_ts->x_max = SCREEN_MAX_X - 1;
	ft6x36_ts->y_max = SCREEN_MAX_Y - 1;
	ft6x36_ts->pdata->irq = ft6x36_ts->irq;

	pr_info("irq = %d\n", client->irq);
	
#ifdef CONFIG_PM
	#if 0
	err = gpio_request(pdata->reset, "ft6x36 reset");
	if (err < 0) {
		dev_err(&client->dev, "%s:failed to set gpio reset.\n",
			__func__);
		goto exit_request_reset;
	}
	#endif
#endif

	err = request_threaded_irq(client->irq, NULL, ft6x36_ts_interrupt,
				   IRQF_TRIGGER_FALLING|IRQF_ONESHOT, client->dev.driver->name,
				   ft6x36_ts);
	
	if (err < 0) {
		dev_err(&client->dev, "ft6x36_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
	disable_irq(client->irq);

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	ft6x36_ts->input_dev = input_dev;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(KEY_HOME, input_dev->keybit);   
	__set_bit(KEY_BACK, input_dev->keybit);  
	__set_bit(KEY_MENU, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	
	input_mt_init_slots(input_dev, MT_MAX_TOUCH_POINTS,0);

	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, ft6x36_ts->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, ft6x36_ts->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
					 0, PRESS_MAX, 0, 0);

	input_dev->name = FT6X36_NAME;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
			"ft6x36_ts_probe: failed to register input device: %s\n",
			dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
	/*make sure CTP already finish startup process */
	msleep(150);

	/*get some register information */
	uc_reg_addr = FT6X36_REG_FW_VER;
	ft6x36_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	dev_dbg(&client->dev, "[FTS] Firmware version = 0x%x\n", uc_reg_value);

	uc_reg_addr = FT6X36_REG_POINT_RATE;
	ft6x36_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	dev_dbg(&client->dev, "[FTS] report rate is %dHz.\n",
		uc_reg_value * 10);

	uc_reg_addr = FT6X36_REG_THGROUP;
	ft6x36_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	dev_dbg(&client->dev, "[FTS] touch threshold is %d.\n",
		uc_reg_value * 4);

#ifdef SYSFS_DEBUG
	ft6x36_create_sysfs(client);
#endif

#ifdef FTS_CTL_IIC
	if (ft_rw_iic_drv_init(client) < 0)
		dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n",
				__func__);
#endif

#ifdef FTS_APK_DEBUG
	ft6x36_create_apk_debug_channel(client);
#endif

#ifdef FTS_FW_AUTO_UPGRADE
	fts_ctpm_auto_upgrade(client);
#endif

#ifdef FTS_CTL_FACE_DETECT
		if (ft_psensor_drv_init(client) < 0)
			dev_err(&client->dev, "%s:[FTS] create fts control psensor driver failed\n",
					__func__);
#endif

	enable_irq(client->irq);
	return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);

exit_input_dev_alloc_failed:
	free_irq(client->irq, ft6x36_ts);
#ifdef CONFIG_PM
	gpio_free(ft6x36_ts->pdata->reset);
#endif

exit_irq_request_failed:
	i2c_set_clientdata(client, NULL);
	kfree(ft6x36_ts);

exit_alloc_data_failed:
exit_check_functionality_failed:
exit_pm_class:
	return err;
}


static int ft6x36_ts_power_off(struct i2c_client *client)
{
	struct ft6x36_platform_data *ft6x36_pdata =
		client->dev.platform_data;
	int ret = 0;

	disable_irq_nosync(client->irq);

	if (ft6x36_pdata->pins_sleep)
		ft6x36_set_pinctrl_state(&client->dev,
				ft6x36_pdata->pins_sleep);

	if (ft6x36_pdata->pm_platdata &&
			ft6x36_pdata->pm_platdata->pm_state_D3_name) {
		ret = device_state_pm_set_state_by_name(&client->dev,
				ft6x36_pdata->pm_platdata->pm_state_D3_name);
	}

	return ret;
}

static int ft6x36_ts_power_on(struct i2c_client *client)
{
	struct ft6x36_platform_data *ft6x36_pdata =
		client->dev.platform_data;
	int ret = 0;


	if (ft6x36_pdata->pins_default)
		ret = ft6x36_set_pinctrl_state(&client->dev,
					ft6x36_pdata->pins_default);

	if (ft6x36_pdata->pm_platdata &&
			ft6x36_pdata->pm_platdata->pm_state_D0_name)
		ret = device_state_pm_set_state_by_name(&client->dev,
			ft6x36_pdata->pm_platdata->pm_state_D0_name);

	if (!ret) {
		enable_irq(client->irq);
	}

	return ret;
}

static int ft6x36_ts_reset(struct i2c_client *client, bool show)
{
	struct ft6x36_platform_data *ft6x36_pdata =
		client->dev.platform_data;

	mdelay(50);
	gpio_set_value(ft6x36_pdata->reset, 0);
	mdelay(20);
	gpio_set_value(ft6x36_pdata->reset, 1);
	mdelay(50);


	if (!show)
		goto out;

out:
	return 0;
}


#ifdef CONFIG_PM
static int ft6x36_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	dev_dbg(&client->dev, "%s\n", __func__);
	return ft6x36_ts_power_off(client);
}

static int ft6x36_ts_resume(struct i2c_client *client)
{
	int ret;

	dev_dbg(&client->dev, "%s\n", __func__);
	ret = ft6x36_ts_power_on(client);
	if (ret) {
		dev_err(&client->dev, "%s: Error during power on\n",
				__func__);
		return ret;
	}

	return ft6x36_ts_reset(client, false);
}

#else
#define ft6x36_ts_suspend	NULL
#define ft6x36_ts_resume		NULL
#endif

static int ft6x36_ts_remove(struct i2c_client *client)
{
	struct ft6x36_ts_data *ft6x36_ts;
	struct ft6x36_platform_data *pdata =
		dev_get_platdata(&client->dev);
	ft6x36_ts = i2c_get_clientdata(client);	
	
	dev_dbg(&client->dev, "%s\n", __func__);
	ft6x36_set_pinctrl_state(&client->dev, pdata->pins_inactive);//need modify
	
	input_unregister_device(ft6x36_ts->input_dev);
	#ifdef CONFIG_PM
	gpio_free(ft6x36_ts->pdata->reset);
	#endif

	#ifdef SYSFS_DEBUG
	ft6x36_release_sysfs(client);
	#endif
	#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
	#endif
	#ifdef FTS_CTL_FACE_DETECT
	ft_psensor_drv_exit();
	#endif
	free_irq(client->irq, ft6x36_ts);
	kfree(ft6x36_ts);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id ft6x36_ts_id[] = {
	{FT6X36_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ft6x36_ts_id);

static struct i2c_driver ft6x36_ts_driver = {
	.probe = ft6x36_ts_probe,
	.remove = ft6x36_ts_remove,
	.id_table = ft6x36_ts_id,
	.suspend = ft6x36_ts_suspend,
	.resume = ft6x36_ts_resume,
	.driver = {
		   .name = FT6X36_NAME,
		   .owner = THIS_MODULE,
		   },
};

static int __init ft6x36_ts_init(void)
{

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	int ret = device_state_pm_add_class(&ft6x36_pm_class);
	if (ret)
		return ret;
#endif


	int ret;
	ret = i2c_add_driver(&ft6x36_ts_driver);
	if (ret) {
		printk(KERN_WARNING "Adding ft6x36 driver failed "
		       "(errno = %d)\n", ret);
	} else {
		pr_info("Successfully added driver %s\n",
			ft6x36_ts_driver.driver.name);
	}
	return ret;
}

static void __exit ft6x36_ts_exit(void)
{
	i2c_del_driver(&ft6x36_ts_driver);
}

module_init(ft6x36_ts_init);
module_exit(ft6x36_ts_exit);

MODULE_AUTHOR("<luowj>");
MODULE_DESCRIPTION("FocalTech ft6x36 TouchScreen driver");
MODULE_LICENSE("GPL");
