#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include "mpu6500.h"


static struct invsens_platform_data_t inv_sensors_pdata_default = {

	.orientation = {  0,  1 , 0,
			 -1,  0,  0,

			  0,  0,  1 },

};

/* I2C I/O function */
static int inv_i2c_read(struct i2c_client *i2c,
				unsigned char reg_addr,
				unsigned short length, unsigned char *data)
{
	struct i2c_msg msgs[2];
	msgs[0].addr = i2c->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &reg_addr;
	
	msgs[1].addr = i2c->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = length;
	msgs[1].buf = data;

	if (i2c_transfer(i2c->adapter, msgs, 2) < 0) {
		printk(KERN_INFO "%s: transfer failed.", __func__);
		return -EIO;
	}
	return 0;
}

static int inv_i2c_single_write(struct i2c_client *i2c,
                         unsigned char reg, unsigned char value)
{
    struct i2c_msg msgs[1];
    unsigned char data[2];

    data[0] = reg;
    data[1] = value;

    /* Write Message */
    msgs[0].addr = i2c->addr;
    msgs[0].flags = 0;
    msgs[0].buf = (unsigned char *) data;
    msgs[0].len = 2;

    if (i2c_transfer(i2c->adapter, msgs, 1) < 0) {
		printk(KERN_INFO "%s: transfer failed.", __func__);
		return -EIO;
	}
	return 0;
}

/* sensor register operation */
static int invn_i2c_check_device(
	struct i2c_client *client)
{
	unsigned char buffer[2];
	int err = 0;

	buffer[0] = MPUREG_WHOAMI;
	err = inv_i2c_read(client, MPUREG_WHOAMI, 1, buffer);
	if (err < 0) {
		printk(KERN_INFO "%s: Can not read WIA.", __func__);
		return err;
	}

	/* Check read data */
	switch(buffer[0])
	{
		case MPU6050_ID:
			printk(KERN_INFO "%s: MPU6050 is mounted.", __func__);
			break;
		case MPU6500_ID:
			printk(KERN_INFO "%s: MPU6500 is mounted.", __func__);
			break;
		case MPU6515_ID:
			printk(KERN_INFO "%s: MPU6515 is mounted.", __func__);
			break;
		case MPU6880_ID:
			printk(KERN_INFO "%s: MPU6880 is mounted.", __func__);
			break;
		default:
			printk(KERN_INFO "%s: unknow sensor is mounted.", __func__);
			return -ENXIO;
	}
	return err;
}

static int invn_set_power_state(struct INVN_DATA *s_data, bool en)
{
	unsigned char data[1];
	int err = 0;
	mutex_lock(&s_data->invn_mutex);
	err = inv_i2c_read(s_data->client, MPUREG_PWR_MGMT_1, 1, data);
	if(err != 0) {
		mutex_unlock(&s_data->invn_mutex);
		return err;
	}
	
	if(en) {
		data[0] &= ~BIT_SLEEP;
	}
	else {
		data[0] |= BIT_SLEEP;
	}
	err = inv_i2c_single_write(s_data->client, MPUREG_PWR_MGMT_1, data[0]);
	mutex_unlock(&s_data->invn_mutex);
	return err;
}

static int invn_set_accel_fsr(struct INVN_DATA *s_data, int fsr)
{
	unsigned char data;
	int err = 0;
	data = ((unsigned char)fsr << 3);
	mutex_lock(&s_data->invn_mutex);
	err = inv_i2c_single_write(s_data->client, MPUREG_ACCEL_CONFIG, data);
	mutex_unlock(&s_data->invn_mutex);
	return err;
}

static int invn_set_accel_enable(struct INVN_DATA *s_data, bool en)
{
	unsigned char data = 0;
	int err;
	if(s_data->gyro.enable == 0)
		data = 0x07;
	if(en == 0)
		data |= 0x38;
	mutex_lock(&s_data->invn_mutex);
	err = inv_i2c_single_write(s_data->client, MPUREG_PWR_MGMT_2, data);
	mutex_unlock(&s_data->invn_mutex);
	return err;
}

static int invn_read_accel(struct INVN_DATA *s_data)
{
	unsigned char buffer[6];
	int err = 0;
	mutex_lock(&s_data->invn_mutex);
	err = inv_i2c_read(s_data->client, MPUREG_ACCEL_DATA, 6, buffer);
	mutex_unlock(&s_data->invn_mutex);
	s_data->accel.raw[0] = (short)((buffer[0] << 8) | buffer[1]);
	s_data->accel.raw[1] = (short)((buffer[2] << 8) | buffer[3]);
	s_data->accel.raw[2] = (short)((buffer[4] << 8) | buffer[5]);

	s_data->accel.value[0] = s_data->accel.raw[0] - (s_data->accel.bias[0] >> s_data->accel.fsr);
	s_data->accel.value[1] = s_data->accel.raw[1] - (s_data->accel.bias[1] >> s_data->accel.fsr);
	s_data->accel.value[2] = s_data->accel.raw[2] - (s_data->accel.bias[2] >> s_data->accel.fsr);
	return err;
}

static int invn_set_gyro_fsr(struct INVN_DATA *s_data, int fsr)
{
	int err = 0;
	unsigned char data = ((unsigned char)fsr << 3);
	mutex_lock(&s_data->invn_mutex);
	err = inv_i2c_single_write(s_data->client, MPUREG_GYRO_CONFIG, data);
	mutex_unlock(&s_data->invn_mutex);
	return err;
}

static int invn_set_gyro_enable(struct INVN_DATA *s_data, bool en)
{
	unsigned char data = 0;
	int err = 0;
	if(s_data->accel.enable == 0)
		data = 0x38;
	if(en == 0)
		data |= 0x07;
	mutex_lock(&s_data->invn_mutex);
	err = inv_i2c_single_write(s_data->client, MPUREG_PWR_MGMT_2, data);
	mutex_unlock(&s_data->invn_mutex);
	return err;
}

static int invn_read_gyro(struct INVN_DATA *s_data)
{
	unsigned char buffer[6];
	int err = 0;
	mutex_lock(&s_data->invn_mutex);
	err = inv_i2c_read(s_data->client, MPUREG_GYRO_DATA, 6, buffer);
	mutex_unlock(&s_data->invn_mutex);
	s_data->gyro.raw[0] = (short)((buffer[0] << 8) | buffer[1]);
	s_data->gyro.raw[1] = (short)((buffer[2] << 8) | buffer[3]);
	s_data->gyro.raw[2] = (short)((buffer[4] << 8) | buffer[5]);

	s_data->gyro.value[0] = s_data->gyro.raw[0] - (s_data->gyro.bias[0] >> (3 - s_data->gyro.fsr));
	s_data->gyro.value[1] = s_data->gyro.raw[1] - (s_data->gyro.bias[1] >> (3 - s_data->gyro.fsr));
	s_data->gyro.value[2] = s_data->gyro.raw[2] - (s_data->gyro.bias[2] >> (3 - s_data->gyro.fsr));
	return err;
}

static int invn_update_accel(struct INVN_DATA *s_data)
{
	int err = 0;
	err = invn_read_accel(s_data);
	if(err == 0) {
		input_report_abs(s_data->input_dev_accel, ABS_X, s_data->accel.value[0]);
		input_report_abs(s_data->input_dev_accel, ABS_Y, s_data->accel.value[1]);
		input_report_abs(s_data->input_dev_accel, ABS_Z, s_data->accel.value[2]);
		input_sync(s_data->input_dev_accel);
	}
	return err;
}

static int invn_update_gyro(struct INVN_DATA *s_data)
{
	int err = 0;
	err = invn_read_gyro(s_data);
	if(err == 0) {
		input_report_rel(s_data->input_dev_gyro, REL_X, s_data->gyro.value[0]);
		input_report_rel(s_data->input_dev_gyro, REL_Y, s_data->gyro.value[1]);
		input_report_rel(s_data->input_dev_gyro, REL_Z, s_data->gyro.value[2]);
                printk("gyro-jinhuijie:%d,%d,%d",s_data->gyro.value[0],s_data->gyro.value[1],s_data->gyro.value[2]);
#if 0

		input_report_abs(s_data->input_dev_gyro, ABS_RX, s_data->gyro.value[0]);
		input_report_abs(s_data->input_dev_gyro, ABS_RY, s_data->gyro.value[1]);
		input_report_abs(s_data->input_dev_gyro, ABS_RZ, s_data->gyro.value[2]);
#endif
		input_sync(s_data->input_dev_gyro);
	}
	return err;
}

static int invn_update_delay(struct INVN_DATA *s_data)
{
	if(s_data->accel.delay < 15)
		s_data->accel.delay_s = 1;
	else if(s_data->accel.delay < 40)
		s_data->accel.delay_s = 2;
	else if(s_data->accel.delay < 100)
		s_data->accel.delay_s = 6;
	else
		s_data->accel.delay_s = 20;
	
	if(s_data->gyro.delay < 15)
		s_data->gyro.delay_s = 1;
	else if(s_data->gyro.delay < 40)
		s_data->gyro.delay_s = 2;
	else if(s_data->gyro.delay < 100)
		s_data->gyro.delay_s = 6;
	else
		s_data->gyro.delay_s = 20;
		
	return 0;
}

static int invn_update_enable(struct INVN_DATA *s_data)
{
	if(s_data->enable) {
		if(s_data->accel.enable || s_data->gyro.enable) {
			cancel_delayed_work(&s_data->invn_delay);
			schedule_delayed_work(&s_data->invn_delay, msecs_to_jiffies(s_data->delay));
			s_data->accel.counter = 0;
			s_data->gyro.counter = 0;
		}
		else 
			cancel_delayed_work(&s_data->invn_delay);
	}
	else {	/* cancel delay work*/
		cancel_delayed_work(&s_data->invn_delay);
	}
	return 0;
}	

static void invn_delay_handler(struct work_struct *work)
{
	struct INVN_DATA *s_data = container_of(work, struct INVN_DATA, invn_delay.work);
	//struct i2c_client *client = s_data->client;
	if(s_data->accel.enable) {
		s_data->accel.counter++;
		if(s_data->accel.counter > s_data->accel.delay_s) {
			s_data->accel.counter = 0;
			invn_update_accel(s_data);
		}
	}
	if(s_data->gyro.enable) {
		s_data->gyro.counter++;
		if(s_data->gyro.counter > s_data->gyro.delay_s) {
			s_data->gyro.counter = 0;
			invn_update_gyro(s_data);
		}
	}
	schedule_delayed_work(&s_data->invn_delay, msecs_to_jiffies(s_data->delay));
}

/* sysfs function */
static ssize_t inv_sensor_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
    struct INVN_DATA *s_data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", s_data->enable);
}

static ssize_t inv_sensor_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int en;
	struct i2c_client *client = to_i2c_client(dev);
    struct INVN_DATA *s_data = i2c_get_clientdata(client);
    if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;
    
    if(sscanf(buf, "%d",&en) < 0)
		return -EINVAL;
	if(en) {
		s_data->enable = 1;
		invn_set_power_state(s_data, true);
	}
	else {
		s_data->enable = 0;
		invn_set_power_state(s_data, false);
	}
    return count;
}

static ssize_t inv_accel_show(struct device *dev,
	struct device_attribute *attr, char *buf, int ch)
{
	ssize_t bytes_printed = 0;
	short accel_x, accel_y, accel_z;
	struct i2c_client *client = to_i2c_client(dev);
    struct INVN_DATA *s_data = i2c_get_clientdata(client);
    s8 *m = s_data->inv_sensors_pdata.orientation;
	switch(ch)
	{
	case ENABLE_CH:
		bytes_printed = sprintf(buf, "%d\n", 
			s_data->accel.enable);
		break;
	case FSR_CH:
		bytes_printed = sprintf(buf, "%d\n", 
			s_data->accel.fsr);
		break;
	case DELAY_CH:
		bytes_printed = sprintf(buf, "%d\n", 
			s_data->accel.delay);
		break;
	case RAW_CH:
		invn_read_accel(s_data);
		accel_x = s_data->accel.raw[0] * m[0] + s_data->accel.raw[1] * m[1] + s_data->accel.raw[2] * m[2];
		accel_y = s_data->accel.raw[0] * m[3] + s_data->accel.raw[1] * m[4] + s_data->accel.raw[2] * m[5];
		accel_z = s_data->accel.raw[0] * m[6] + s_data->accel.raw[1] * m[7] + s_data->accel.raw[2] * m[8];
		bytes_printed = sprintf(buf, "%d,%d,%d\n", 
			accel_x, accel_y, accel_z);
		break;
	case BIAS_CH:
		accel_x = s_data->accel.bias[0] * m[0] + s_data->accel.bias[1] * m[1] + s_data->accel.bias[2] * m[2];
		accel_y = s_data->accel.bias[0] * m[3] + s_data->accel.bias[1] * m[4] + s_data->accel.bias[2] * m[5];
		accel_z = s_data->accel.bias[0] * m[6] + s_data->accel.bias[1] * m[7] + s_data->accel.bias[2] * m[8];
		bytes_printed = sprintf(buf, "%d,%d,%d\n", 
			accel_x, accel_y, accel_z);
		break;
	case VALUE_CH:
		invn_read_accel(s_data);
		accel_x = s_data->accel.value[0] * m[0] + s_data->accel.value[1] * m[1] + s_data->accel.value[2] * m[2];
		accel_y = s_data->accel.value[0] * m[3] + s_data->accel.value[1] * m[4] + s_data->accel.value[2] * m[5];
		accel_z = s_data->accel.value[0] * m[6] + s_data->accel.value[1] * m[7] + s_data->accel.value[2] * m[8];
		bytes_printed = sprintf(buf, "%d,%d,%d\n", 
			accel_x, accel_y, accel_z);
		break;
	case MATRIX_CH:
		bytes_printed = sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n", 
			m[0],m[1],m[2],m[3],m[4],m[5],m[6],m[7],m[8]);
		break;
	default:
		break;
	}
	return bytes_printed;
}

static ssize_t inv_accel_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count, int ch)
{
	int data[3];
	s8 *m;
	struct i2c_client *client = to_i2c_client(dev);
    struct INVN_DATA *s_data = i2c_get_clientdata(client);
    if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;
    switch(ch)
    {
    case ENABLE_CH:
    	if(sscanf(buf, "%d",&data[0]) < 0)
			return -EINVAL;
		if(data[0] < 0 || data[0] > 1)
			return -EINVAL;
		if(s_data->accel.enable != data[0]) {
			if(data[0]) 
				invn_set_accel_enable(s_data, true);
			else
				invn_set_accel_enable(s_data, false);
			s_data->accel.enable = data[0];
			invn_update_enable(s_data);
		}
    	break;
    case FSR_CH:
    	if(sscanf(buf, "%d",&data[0]) < 0)
			return -EINVAL;
		if(data[0] < 0 || data[0] > 3)
			return -EINVAL;
		if(s_data->accel.fsr != data[0]) {
			invn_set_accel_fsr(s_data, data[0]);
			s_data->accel.fsr = data[0];
		}
    	break;
    case DELAY_CH:
    	if(sscanf(buf, "%d",&data[0]) < 0)
			return -EINVAL;
		if(data[0] < 0)
			return -EINVAL;
		if(s_data->accel.delay != data[0]) {
			s_data->accel.delay = data[0];
			invn_update_delay(s_data);
		}		
    	break;
    case BIAS_CH:
    	if(sscanf(buf, "%d,%d,%d",&data[0], &data[1], &data[2]) < 0)
			return -EINVAL;
		m = s_data->inv_sensors_pdata.orientation;
		s_data->accel.bias[0] = data[0] * m[0] + data[1] * m[3] + data[2] * m[6];
		s_data->accel.bias[1] = data[0] * m[1] + data[1] * m[4] + data[2] * m[7];
		s_data->accel.bias[2] = data[0] * m[2] + data[1] * m[5] + data[2] * m[8];
    	break;
    default:
    	break;
    }
    return count;
}


static ssize_t inv_gyro_show(struct device *dev,
	struct device_attribute *attr, char *buf, int ch)
{
	ssize_t bytes_printed = 0;
	short gyro_x, gyro_y, gyro_z;
	struct i2c_client *client = to_i2c_client(dev);
    struct INVN_DATA *s_data = i2c_get_clientdata(client);
    s8 *m = s_data->inv_sensors_pdata.orientation;
	switch(ch)
	{
	case ENABLE_CH:
		bytes_printed = sprintf(buf, "%d\n", 
			s_data->gyro.enable);
		break;
	case FSR_CH:
		bytes_printed = sprintf(buf, "%d\n", 
			s_data->gyro.fsr);
		break;
	case DELAY_CH:
		bytes_printed = sprintf(buf, "%d\n", 
			s_data->gyro.delay);
		break;
	case RAW_CH:
		invn_read_gyro(s_data);
		gyro_x = s_data->gyro.raw[0] * m[0] + s_data->gyro.raw[1] * m[1] + s_data->gyro.raw[2] * m[2];
		gyro_y = s_data->gyro.raw[0] * m[3] + s_data->gyro.raw[1] * m[4] + s_data->gyro.raw[2] * m[5];
		gyro_z = s_data->gyro.raw[0] * m[6] + s_data->gyro.raw[1] * m[7] + s_data->gyro.raw[2] * m[8];
		bytes_printed = sprintf(buf, "%d,%d,%d\n", 
			gyro_x, gyro_y, gyro_z);
		break;
	case BIAS_CH:
		gyro_x = s_data->gyro.bias[0] * m[0] + s_data->gyro.bias[1] * m[1] + s_data->gyro.bias[2] * m[2];
		gyro_y = s_data->gyro.bias[0] * m[3] + s_data->gyro.bias[1] * m[4] + s_data->gyro.bias[2] * m[5];
		gyro_z = s_data->gyro.bias[0] * m[6] + s_data->gyro.bias[1] * m[7] + s_data->gyro.bias[2] * m[8];
		bytes_printed = sprintf(buf, "%d,%d,%d\n", 
			gyro_x, gyro_y, gyro_z);
		break;
	case VALUE_CH:
		invn_read_gyro(s_data);
		gyro_x = s_data->gyro.value[0] * m[0] + s_data->gyro.value[1] * m[1] + s_data->gyro.value[2] * m[2];
		gyro_y = s_data->gyro.value[0] * m[3] + s_data->gyro.value[1] * m[4] + s_data->gyro.value[2] * m[5];
		gyro_z = s_data->gyro.value[0] * m[6] + s_data->gyro.value[1] * m[7] + s_data->gyro.value[2] * m[8];
		bytes_printed = sprintf(buf, "%d,%d,%d\n", 
			gyro_x, gyro_y, gyro_z);
		break;
	case MATRIX_CH:
		bytes_printed = sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n", 
			m[0],m[1],m[2],m[3],m[4],m[5],m[6],m[7],m[8]);
		break;
	default:
		break;
	}
	return bytes_printed;
}

static ssize_t inv_gyro_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count, int ch)
{
	int data[3];
	s8 *m;
	struct i2c_client *client = to_i2c_client(dev);
    struct INVN_DATA *s_data = i2c_get_clientdata(client);
    if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;
    switch(ch)
    {
    case ENABLE_CH:
    	if(sscanf(buf, "%d",&data[0]) < 0)
			return -EINVAL;
		if(data[0] < 0 || data[0] > 1)
			return -EINVAL;
		if(s_data->gyro.enable != data[0]) {
			if(data[0]) 
				invn_set_gyro_enable(s_data, true);
			else
				invn_set_gyro_enable(s_data, false);
			s_data->gyro.enable = data[0];
			invn_update_enable(s_data);
		}
    	break;
    case FSR_CH:
    	if(sscanf(buf, "%d",&data[0]) < 0)
			return -EINVAL;
		if(data[0] < 0 || data[0] > 3)
			return -EINVAL;
		if(s_data->gyro.fsr != data[0]) {
			invn_set_gyro_fsr(s_data, data[0]);
			s_data->gyro.fsr = data[0];
		}
    	break;
    case DELAY_CH:
    	if(sscanf(buf, "%d",&data[0]) < 0)
			return -EINVAL;
		if(data[0] < 0)
			return -EINVAL;
		if(s_data->gyro.delay != data[0]) {
			s_data->gyro.delay = data[0];
			invn_update_delay(s_data);
		}		
    	break;
    case BIAS_CH:
    	if(sscanf(buf, "%d,%d,%d",&data[0], &data[1], &data[2]) < 0)
			return -EINVAL;
		m = s_data->inv_sensors_pdata.orientation;
		s_data->gyro.bias[0] = data[0] * m[0] + data[1] * m[3] + data[2] * m[6];
		s_data->gyro.bias[1] = data[0] * m[1] + data[1] * m[4] + data[2] * m[7];
		s_data->gyro.bias[2] = data[0] * m[2] + data[1] * m[5] + data[2] * m[8];
    	break;
    default:
    	break;
    }
    return count;
}

/*accel*/
static ssize_t inv_accel_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return inv_accel_show(dev, attr, buf, ENABLE_CH);
}

static ssize_t inv_accel_fsr_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return inv_accel_show(dev, attr, buf, FSR_CH);
}

static ssize_t inv_accel_delay_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return inv_accel_show(dev, attr, buf, DELAY_CH);
}

static ssize_t inv_accel_raw_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return inv_accel_show(dev, attr, buf, RAW_CH);
}

static ssize_t inv_accel_bias_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return inv_accel_show(dev, attr, buf, BIAS_CH);
}

static ssize_t inv_accel_value_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return inv_accel_show(dev, attr, buf, VALUE_CH);
}

static ssize_t inv_accel_matrix_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return inv_accel_show(dev, attr, buf, MATRIX_CH);
}

static ssize_t inv_accel_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	return inv_accel_store(dev, attr, buf, count, ENABLE_CH);
}

static ssize_t inv_accel_fsr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	return inv_accel_store(dev, attr, buf, count, FSR_CH);
}

static ssize_t inv_accel_delay_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	return inv_accel_store(dev, attr, buf, count, DELAY_CH);
}

static ssize_t inv_accel_bias_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	return inv_accel_store(dev, attr, buf, count, BIAS_CH);
}

/*gyro*/
static ssize_t inv_gyro_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return inv_gyro_show(dev, attr, buf, ENABLE_CH);
}

static ssize_t inv_gyro_fsr_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return inv_gyro_show(dev, attr, buf, FSR_CH);
}

static ssize_t inv_gyro_delay_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return inv_gyro_show(dev, attr, buf, DELAY_CH);
}

static ssize_t inv_gyro_raw_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return inv_gyro_show(dev, attr, buf, RAW_CH);
}

static ssize_t inv_gyro_bias_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return inv_gyro_show(dev, attr, buf, BIAS_CH);
}

static ssize_t inv_gyro_value_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return inv_gyro_show(dev, attr, buf, VALUE_CH);
}

static ssize_t inv_gyro_matrix_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return inv_gyro_show(dev, attr, buf, MATRIX_CH);
}

static ssize_t inv_gyro_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	return inv_gyro_store(dev, attr, buf, count, ENABLE_CH);
}

static ssize_t inv_gyro_fsr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	return inv_gyro_store(dev, attr, buf, count, FSR_CH);
}

static ssize_t inv_gyro_delay_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	return inv_gyro_store(dev, attr, buf, count, DELAY_CH);
}

static ssize_t inv_gyro_bias_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	return inv_gyro_store(dev, attr, buf, count, BIAS_CH);
}

/*accel sysfs function */
static DEVICE_ATTR(sensor_enable, S_IRUGO | S_IWUGO, 
	inv_sensor_enable_show, inv_sensor_enable_store);

static DEVICE_ATTR(accel_enable, S_IRUGO | S_IWUGO, 
	inv_accel_enable_show, inv_accel_enable_store);

static DEVICE_ATTR(accel_delay, S_IRUGO | S_IWUGO, 
	inv_accel_delay_show, inv_accel_delay_store);
	
static DEVICE_ATTR(accel_fsr, S_IRUGO | S_IWUGO, 
	inv_accel_fsr_show, inv_accel_fsr_store);

static DEVICE_ATTR(accel_bias, S_IRUGO | S_IWUGO, 
	inv_accel_bias_show, inv_accel_bias_store);

static DEVICE_ATTR(accel_raw, S_IRUGO, inv_accel_raw_show, NULL);
static DEVICE_ATTR(accel_value, S_IRUGO, inv_accel_value_show, NULL);
static DEVICE_ATTR(accel_matrix, S_IRUGO, inv_accel_matrix_show, NULL);

/*gyro sysfs function */
static DEVICE_ATTR(gyro_enable, S_IRUGO | S_IWUGO, 
	inv_gyro_enable_show, inv_gyro_enable_store);

static DEVICE_ATTR(gyro_delay, S_IRUGO | S_IWUGO, 
	inv_gyro_delay_show, inv_gyro_delay_store);
	
static DEVICE_ATTR(gyro_fsr, S_IRUGO | S_IWUGO, 
	inv_gyro_fsr_show, inv_gyro_fsr_store);

static DEVICE_ATTR(gyro_bias, S_IRUGO | S_IWUGO, 
	inv_gyro_bias_show, inv_gyro_bias_store);

static DEVICE_ATTR(gyro_raw, S_IRUGO, inv_gyro_raw_show, NULL);
static DEVICE_ATTR(gyro_value, S_IRUGO, inv_gyro_value_show, NULL);
static DEVICE_ATTR(gyro_matrix, S_IRUGO, inv_gyro_matrix_show, NULL);

static struct attribute *invn_attributes[] = {
	/*sensor sysfs*/
	&dev_attr_sensor_enable.attr,
	/*accel sysfs*/
	&dev_attr_accel_enable.attr,
	&dev_attr_accel_delay.attr,
	&dev_attr_accel_fsr.attr,
	&dev_attr_accel_raw.attr,
	&dev_attr_accel_bias.attr,
	&dev_attr_accel_value.attr,
	&dev_attr_accel_matrix.attr,
	/*gyro sysfs*/
	&dev_attr_gyro_enable.attr,
	&dev_attr_gyro_delay.attr,
	&dev_attr_gyro_fsr.attr,
	&dev_attr_gyro_raw.attr,
	&dev_attr_gyro_bias.attr,
	&dev_attr_gyro_value.attr,
	&dev_attr_gyro_matrix.attr,
	NULL
};

static const struct attribute_group invn_attr_group = {
	.attrs = invn_attributes,
};

static int invn_client_init(struct INVN_DATA *s_data)
{
	s_data->enable = 1;
	s_data->delay = 10;
	memset(&s_data->accel, 0, sizeof(struct sensor_data));
	memset(&s_data->gyro, 0, sizeof(struct sensor_data));
	
	s_data->accel.fsr = 0;			//2G
	s_data->accel.enable = 0;		//disable
	s_data->accel.delay = 200;		//
	
	s_data->gyro.fsr = 3;			//2000dps
	s_data->gyro.enable = 0;		//disable
	s_data->gyro.delay = 10;		//
	
	invn_update_delay(s_data);
	return 0;
}

static int invn_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct INVN_DATA *s_data = i2c_get_clientdata(client);
	printk(KERN_INFO "%s\n", __FUNCTION__);
	invn_set_power_state(s_data, false);
	return 0;
}

static int invn_resume(struct i2c_client *client)
{
	struct INVN_DATA *s_data = i2c_get_clientdata(client);
	printk(KERN_INFO "%s\n", __FUNCTION__);
	invn_set_power_state(s_data, true);
	return 0;
}

int invn_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct INVN_DATA *s_data;
	int err = 0;
	unsigned char data[2];
	printk(KERN_INFO "mpu65xx start probing.");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_INFO "%s: check_functionality failed.", __func__);
		err = -ENODEV;
		goto exit0;
	}

	/* Allocate memory for driver data */
	s_data = kzalloc(sizeof(struct INVN_DATA), GFP_KERNEL);
	if (!s_data) {
		printk(KERN_INFO "%s: memory allocation failed.", __func__);
		err = -ENOMEM;
		goto exit1;
	}
	
	/***** I2C initialization *****/
	s_data->client = client;
	/* check connection */
	err = invn_i2c_check_device(client);
	if (err < 0)
		goto exit2;
	/* set client data */
	i2c_set_clientdata(client, s_data);

	s_data->inv_sensors_pdata =  inv_sensors_pdata_default;//  *(struct invsens_platform_data_t *)dev_get_platdata(&client->dev);
		
	/* Configure Power Management Registers
	 * Write 0x00 to PWR_MGMT_1 (0x6B).
	 * Write 0x07 to PWR_MGMT_2 (0x6C). Enable Accel, disable Gyro.
	 *Note: This is also the hardware reset value for these registers.
	 */
	data[0] = 0x00;
	data[1] = 0x07;
	inv_i2c_single_write(client, MPUREG_PWR_MGMT_1, data[0]);
	msleep(100);
	inv_i2c_single_write(client, MPUREG_PWR_MGMT_2, data[1]);

	/* Configure Gyroscope Parameters
	 * Write 0x03 to CONFIG (0x1A).
	 * Write 0x18 to GYRO_CONFIG (0x1B).
	 * Sets the cut-off frequency of the Digital Low-Pass Frequency (DLPF) filter to 42Hz.
	 * Sets the Full Scale Range (FSR) of the gyroscope to 2000dps.
	 */
	data[0] = 0x02;
	data[1] = 0x18;
	inv_i2c_single_write(client, MPUREG_CONFIG, data[0]);
	inv_i2c_single_write(client, MPUREG_GYRO_CONFIG, data[1]);

        
	/* Configure Accelerometer Parameters
	 * Write 0x00 to ACCEL_CONFIG (0x1C)
	 * Write 0x40 to ACCEL_CONFIG2 (0x1D).
	 * Sets the Accelerometer FSR to 2g.
	 */
	data[0] = 0x00;
	data[1] = 0x40;
	inv_i2c_single_write(client, MPUREG_ACCEL_CONFIG, data[0]);
	inv_i2c_single_write(client, MPUREG_ACCEL_CONFIG2, data[1]);
                
	/* Configure FIFO and Interrupts
	 * Write 0x00 to FIFO_EN (0x23).
	 * Write 0x00 to INT_ENABLE (0x38).
	 * Defers control of the FIFO and the interrupts from the MPU to the DMP.
	 */
	data[0] = 0x00;
	data[1] = 0x00;
	inv_i2c_single_write(client, MPUREG_FIFO_EN, data[0]);
	inv_i2c_single_write(client, MPUREG_INT_ENABLE, data[1]);
        
	/* Reset the FIFO
	 * Write 0x04 to USER_CTRL (0x6A)
	 */
	data[0] = 0x04;
	inv_i2c_single_write(client, MPUREG_USER_CTRL, data[0]);
        
	/* Configure Sensor Sample Rate
	 * Write 0x04 to SMPLRT_DIV (0x19)
	 * Sets sample rate to 200Hz.
	 */
	data[0] = 0x04;
	inv_i2c_single_write(client, MPUREG_SMPLRT_DIV, data[0]);

	/* Register to Input Device */
	s_data->input_dev_accel = input_allocate_device();
	if (!s_data->input_dev_accel) {
		err = -ENOMEM;
		printk("%s: Failed to allocate input device als\n", __func__);
		goto exit3;
	}

	s_data->input_dev_gyro = input_allocate_device();
	if (!s_data->input_dev_gyro) {
		err = -ENOMEM;
		printk("%s: Failed to allocate input device ps\n", __func__);
		goto exit4;
	}
	
	input_set_drvdata(s_data->input_dev_accel, s_data);
	input_set_drvdata(s_data->input_dev_gyro, s_data);
	
	s_data->input_dev_accel->name = "accel";
	s_data->input_dev_gyro->name = "gyro";
	s_data->input_dev_accel->id.bustype = BUS_I2C;
	s_data->input_dev_accel->dev.parent =&s_data->client->dev;
	s_data->input_dev_gyro->id.bustype = BUS_I2C;
	s_data->input_dev_gyro->dev.parent =&s_data->client->dev;
	
	set_bit(EV_ABS, s_data->input_dev_accel->evbit);
	set_bit(EV_REL, s_data->input_dev_gyro->evbit);
	input_set_abs_params(s_data->input_dev_accel, ABS_X,
			-32768, 32768, 0, 0);
	input_set_abs_params(s_data->input_dev_accel, ABS_Y,
			-32768, 32768, 0, 0);
	input_set_abs_params(s_data->input_dev_accel, ABS_Z,
			-32768, 32768, 0, 0);
#if 0
	input_set_abs_params(s_data->input_dev_gyro, ABS_RX,
			-32768, 32768, 0, 0);
	input_set_abs_params(s_data->input_dev_gyro, ABS_RY,
			-32768, 32768, 0, 0);
	input_set_abs_params(s_data->input_dev_gyro, ABS_RZ,
			-32768, 32768, 0, 0);
	
	input_set_abs_params(s_data->input_dev_gyro, REL_X,
			-32768, 32768, 0, 0);
	input_set_abs_params(s_data->input_dev_gyro, REL_Y,
			-32768, 32768, 0, 0);
	input_set_abs_params(s_data->input_dev_gyro, REL_Z,
			-32768, 32768, 0, 0);
#endif

        set_bit(REL_X, s_data->input_dev_gyro->relbit);
        set_bit(REL_Y, s_data->input_dev_gyro->relbit);
        set_bit(REL_Z, s_data->input_dev_gyro->relbit);
		
	err = input_register_device(s_data->input_dev_accel);
	if (err) {
		err = -ENOMEM;
		printk("%s: Unable to register input device accel: %s\n", __func__, 
		       s_data->input_dev_accel->name);
		goto exit5;
	}

	err = input_register_device(s_data->input_dev_gyro);
	if (err) {
		err = -ENOMEM;
		printk("%s: Unable to register input device gyro: %s\n", __func__, 
		       s_data->input_dev_gyro->name);
		goto exit6;
	}
	
	/* Register sysfs */
	err = sysfs_create_group(&client->dev.kobj, &invn_attr_group);
	if (err)
	{
		printk("%s sysfs_create_group\n", __func__);
		goto exit7;
	}
	
	invn_client_init(s_data);
	mutex_init(&s_data->invn_mutex);
	INIT_DELAYED_WORK(&s_data->invn_delay, invn_delay_handler);
	
	printk(KERN_INFO "%s successfully probed.", __func__);
	return 0;

	sysfs_remove_group(&client->dev.kobj, &invn_attr_group);
exit7:
	input_unregister_device(s_data->input_dev_gyro);
exit6:
	input_unregister_device(s_data->input_dev_accel);
exit5:
	input_free_device(s_data->input_dev_gyro);	
exit4:	
	input_free_device(s_data->input_dev_accel);
exit3:
exit2:
	kfree(s_data);
exit1:
exit0:
	return err;
}

static int invn_remove(struct i2c_client *client)
{
	struct INVN_DATA *s_data = i2c_get_clientdata(client);
	input_unregister_device(s_data->input_dev_accel);
	input_unregister_device(s_data->input_dev_gyro);
	
	input_free_device(s_data->input_dev_accel);
	input_free_device(s_data->input_dev_gyro);

	sysfs_remove_group(&client->dev.kobj, &invn_attr_group);
	
	kfree(s_data);
	printk(KERN_INFO "successfully removed.");
	return 0;
}


static const struct i2c_device_id invn_id[] = {
	{INVN_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver invn_driver = {
	.probe		= invn_probe,
	.remove 	= invn_remove,
	.suspend	= invn_suspend,
	.resume		= invn_resume,
	.id_table	= invn_id,
	.driver = {
		.name = INVN_I2C_NAME,
	},
};

static int __init invn_init(void)
{
	printk(KERN_INFO "invn mpu65xx driver: initialize.");
	return i2c_add_driver(&invn_driver);
}

static void __exit invn_exit(void)
{
	printk(KERN_INFO "invn mpu65xx driver: release.");
	i2c_del_driver(&invn_driver);
}

module_init(invn_init);
module_exit(invn_exit);

MODULE_AUTHOR("joe du <jdu@invensense.com>");
MODULE_DESCRIPTION("mpu65xx driver");
MODULE_LICENSE("GPL");

