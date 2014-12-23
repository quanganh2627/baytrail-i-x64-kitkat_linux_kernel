/*
 *  stk8baxx.c - Linux kernel modules for sensortek stk8ba20 / stk8ba22 / 
 * 	stk8ba50 / stk8ba55 accelerometer
 *
 *  Copyright (C) 2012~2014 Lex Hsieh / sensortek <lex_hsieh@sensortek.com.tw>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */   
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/input.h>
#include <asm/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h> 
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/pm_runtime.h>
#include   <linux/fs.h>   
#include <linux/module.h>
#include  <asm/uaccess.h>
#define STK8BAXX_PERMISSION_THREAD
//#ifdef STK8BAXX_PERMISSION_THREAD
#include <linux/fcntl.h>
#include <linux/syscalls.h>
//#endif 	//	#ifdef STK8BAXX_PERMISSION_THREAD

//#define ALLWINNER_PLATFORM
//#define STK_ROCKCHIP_PLATFORM
#ifdef ALLWINNER_PLATFORM
	#include "stk8baxx.h"
	#include <mach/sys_config.h>
	#include <asm/atomic.h>
	#include <linux/init.h>
#else
	#include "stk8baxx.h"
#endif

#include <asm/div64.h>

#define STK_ACC_DRIVER_VERSION	"3.5.0"
/*------------------User-defined settings-------------------------*/
#define STK_DEBUG_PRINT
#define STK_ACC_POLLING_MODE	1		/*choose polling or interrupt mode*/
#define STK_LOWPASS
#define STK_FIR_LEN	8		/* 1~32 */
//#define STK_RESUME_RE_INIT
#define STK_TUNE
#define STK_ZG_FILTER
//#define STK_HOLD_ODR

/*------------------Miscellaneous settings-------------------------*/
#ifdef IS_STK8BA50
	#define STK_ZG_COUNT	2
#elif defined (IS_STK8BA22)
	#define STK_ZG_COUNT	1
#endif

#if (!STK_ACC_POLLING_MODE)
	#define ADDITIONAL_GPIO_CFG 1
#endif

#define STK8BAXX_I2C_NAME	"stk8baxx"
#define ACC_IDEVICE_NAME		"accelerometer"

#define STK8BAXX_INIT_ODR		0xB		//0xC:150Hz, 0xB:75Hz, 0xA:37.5Hz
#define  STK8BAXX_SPTIME_NO		4
const static int STK8BAXX_SAMPLE_TIME[STK8BAXX_SPTIME_NO] = {53333, 26667, 13333, 6667};
#define  STK8BAXX_SPTIME_BASE	0x9

#ifdef IS_STK8BA50
	#define STK_TUNE_XYOFFSET 	36
	#define STK_TUNE_ZOFFSET 	72
	#define STK_TUNE_NOISE 		56
#elif defined(IS_STK8BA22)
	#define STK_TUNE_XYOFFSET 	9
	#define STK_TUNE_ZOFFSET 	18
	#define STK_TUNE_NOISE 		14	
#endif
#define STK_TUNE_NUM 			125
#define STK_TUNE_DELAY 			125

/*------------------Calibration prameters-------------------------*/
#define STK_ACC_CALI_VER0			0x18
#define STK_ACC_CALI_VER1			0x02
#define STK_ACC_CALI_FILE "/data/misc/stkacccali.conf"
//#define STK_ACC_CALI_FILE "/system/etc/stkacccali.conf"
#define STK_ACC_CALI_FILE_SIZE 10

#define STK_K_SUCCESS_TUNE			0x04
#define STK_K_SUCCESS_FT2			0x03
#define STK_K_SUCCESS_FT1			0x02
#define STK_K_SUCCESS_FILE			0x01
#define STK_K_NO_CALI				0xFF
#define STK_K_RUNNING				0xFE
#define STK_K_FAIL_LRG_DIFF			0xFD
#define STK_K_FAIL_OPEN_FILE			0xFC
#define STK_K_FAIL_W_FILE				0xFB
#define STK_K_FAIL_R_BACK				0xFA
#define STK_K_FAIL_R_BACK_COMP		0xF9
#define STK_K_FAIL_I2C				0xF8
#define STK_K_FAIL_K_PARA				0xF7
#define STK_K_FAIL_OUT_RG			0xF6
#define STK_K_FAIL_ENG_I2C			0xF5
#define STK_K_FAIL_FT1_USD			0xF4
#define STK_K_FAIL_FT2_USD			0xF3
#define STK_K_FAIL_WRITE_NOFST		0xF2
#define STK_K_FAIL_OTP_5T				0xF1
#define STK_K_FAIL_PLACEMENT			0xF0

/*------------------stk8baxx registers-------------------------*/
#define 	STK8BAXX_XOUT1			0x02
#define 	STK8BAXX_XOUT2			0x03
#define 	STK8BAXX_YOUT1			0x04	
#define 	STK8BAXX_YOUT2			0x05
#define 	STK8BAXX_ZOUT1			0x06
#define 	STK8BAXX_ZOUT2			0x07
#define 	STK8BAXX_INTSTS1		0x09
#define 	STK8BAXX_INTSTS2		0x0A
#define 	STK8BAXX_EVENTINFO1	0x0B
#define 	STK8BAXX_EVENTINFO2	0x0C
#define 	STK8BAXX_RANGESEL		0x0F
#define 	STK8BAXX_BWSEL			0x10
#define 	STK8BAXX_POWMODE		0x11
#define  	STK8BAXX_DATASETUP		0x13
#define  	STK8BAXX_SWRST			0x14
#define  	STK8BAXX_INTEN1			0x16
#define  	STK8BAXX_INTEN2			0x17
#define  	STK8BAXX_INTMAP1		0x19
#define  	STK8BAXX_INTMAP2		0x1A
#define  	STK8BAXX_INTMAP3		0x1B
#define  	STK8BAXX_DATASRC		0x1E
#define  	STK8BAXX_INTCFG1		0x20
#define  	STK8BAXX_INTCFG2		0x21
#define  	STK8BAXX_LGDLY			0x22
#define  	STK8BAXX_LGTHD			0x23
#define  	STK8BAXX_HLGCFG		0x24
#define  	STK8BAXX_HGDLY			0x25
#define  	STK8BAXX_HGTHD			0x26
#define  	STK8BAXX_SLOPEDLY		0x27
#define  	STK8BAXX_SLOPETHD		0x28
#define  	STK8BAXX_TAPTIME		0x2A
#define  	STK8BAXX_TAPCFG		0x2B
#define  	STK8BAXX_ORIENTCFG		0x2C
#define  	STK8BAXX_ORIENTTHETA	0x2D
#define  	STK8BAXX_FLATTHETA		0x2E
#define  	STK8BAXX_FLATHOLD		0x2F
#define  	STK8BAXX_SLFTST			0x32	
#define  	STK8BAXX_INTFCFG		0x34
#define  	STK8BAXX_OFSTCOMP1	0x36
#define  	STK8BAXX_OFSTCOMP2	0x37
#define  	STK8BAXX_OFSTFILTX		0x38
#define  	STK8BAXX_OFSTFILTY		0x39
#define  	STK8BAXX_OFSTFILTZ		0x3A
#define  	STK8BAXX_OFSTUNFILTX	0x3B
#define  	STK8BAXX_OFSTUNFILTY	0x3C
#define  	STK8BAXX_OFSTUNFILTZ	0x3D

/*	ZOUT1 register	*/
#define STK8BAXX_O_NEW			0x01

/*	SWRST register	*/
#define  	STK8BAXX_SWRST_VAL		0xB6

/*	STK8BAXX_POWMODE register	*/
#define STK8BAXX_MD_SUSPEND	0x80
#define STK8BAXX_MD_NORMAL		0x00
#define STK8BAXX_MD_SLP_MASK	0x1E

/*	RANGESEL register	*/
#define STK8BAXX_RANGE_MASK	0x0F
#define STK8BAXX_RNG_2G			0x3
#define STK8BAXX_RNG_4G			0x5
#define STK8BAXX_RNG_8G			0x8
#define STK8BAXX_RNG_16G		0xC

/* OFSTCOMP1 register*/
#define STK8BAXX_OF_CAL_DRY_MASK	0x10
#define CAL_AXIS_X_EN				0x20
#define CAL_AXIS_Y_EN				0x40
#define CAL_AXIS_Z_EN				0x60 
#define CAL_OFST_RST				0x00

/* OFSTCOMP2 register*/
#define CAL_TG_X0_Y0_ZPOS1		0x20
#define CAL_TG_X0_Y0_ZNEG1		0x40


/*------------------Data structure-------------------------*/
struct stk8baxx_acc
{
	s16 x;
	s16 y;
	s16 z;
};

#if defined(STK_LOWPASS)
#define MAX_FIR_LEN 32
struct data_filter {
    s16 raw[MAX_FIR_LEN][3];
    int sum[3];
    int num;
    int idx;
};
#endif

struct stk8baxx_data 
{
	struct i2c_client *client;	
	struct input_dev *input_dev;
	struct mutex value_lock;
	struct mutex write_lock;
	int irq;	
	int int_pin;
	struct stk8baxx_acc acc_xyz;
	atomic_t enabled;
	bool first_enable;
	struct work_struct stk_work;
	struct hrtimer acc_timer;		
	ktime_t poll_delay;	
	struct workqueue_struct *stk_mems_work_queue;
	unsigned char stk8baxx_placement;
	atomic_t cali_status;	
	atomic_t recv_reg;	
	bool re_enable;	
#if defined(STK_LOWPASS)
	atomic_t                firlength;
	atomic_t                fir_en;
	struct data_filter      fir;
#endif 	
};
/*------------------Function prototype-------------------------*/
static int32_t stk8baxx_get_file_content(char * r_buf, int8_t buf_size);
static int stk8baxx_store_in_file(u8 offset[], u8 status);
static int STK8BAXX_GetEnable(struct stk8baxx_data *stk, char *gState);
static int STK8BAXX_SetEnable(struct stk8baxx_data *stk, char en);
static int STK8BAXX_SetOffset(struct stk8baxx_data *stk, u8 offset[]);

/*------------------Global variables-------------------------*/
static struct stk8baxx_data *stk8baxx_data_ptr;
static int event_since_en = 0;
static int event_since_en_limit = 0;
static struct stk8baxx_platform_data p_stk8baxx_platdata = 
       {   
            .direction = 0,
            .interrupt_pin = -1,
              
       };
#ifdef STK_TUNE
static u8 stk_tune_offset_record[3] = {0};
static int stk_tune_offset[3] = {0};
static int stk_tune_sum[3] = {0};
static int stk_tune_max[3] = {0};
static int stk_tune_min[3] = {0};
static int stk_tune_index = 0;
static int stk_tune_done = 0;
#endif

/*------------------Main functions-------------------------*/

static int stk_i2c_rx(char *rxData, int length)
{
	uint8_t retry;	
#ifdef STK_ROCKCHIP_PLATFORM	
	int scl_clk_rate = 100 * 1000;
#endif	
	struct i2c_msg msgs[] = 
	{
		{
			.addr = stk8baxx_data_ptr->client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
#ifdef STK_ROCKCHIP_PLATFORM				
			.scl_rate = scl_clk_rate,
#endif			
		},
		{
			.addr = stk8baxx_data_ptr->client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
#ifdef STK_ROCKCHIP_PLATFORM				
			.scl_rate = scl_clk_rate,			
#endif			
		},
	};
	
	for (retry = 0; retry <= 3; retry++) 
	{
		if (i2c_transfer(stk8baxx_data_ptr->client->adapter, msgs, 2) > 0)
			break;
		else
			mdelay(10);
	}
	
	if (retry > 3) 
	{
		printk(KERN_ERR "stk8baxx i2c_transfer error, retry over 3\n");
		return -EIO;
	} 
	else
		return 0;	
}

static int stk_i2c_tx(char *txData, int length)
{
	int tx_retry;
	int result;
	char buffer;	
	int overall_retry;
#ifdef STK_ROCKCHIP_PLATFORM	
	int scl_clk_rate = 100 * 1000;
#endif		
	struct i2c_msg msg[] = 
	{
		{
			.addr = stk8baxx_data_ptr->client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
#ifdef STK_ROCKCHIP_PLATFORM				
			.scl_rate = scl_clk_rate,			
#endif				
		},
	};
	
	for(overall_retry = 0; overall_retry < 3; overall_retry++)
	{
		for (tx_retry = 0; tx_retry <= 3; tx_retry++) 
		{
			if (i2c_transfer(stk8baxx_data_ptr->client->adapter, msg, 1) > 0)
				break;
			else
				mdelay(10);
		}
		
		if (tx_retry > 3) 
		{
			printk(KERN_ERR "stk8baxx i2c_transfer error, tx_retry over 3\n");	
			return -EIO;
		}
		
		// skip read-only CAL_RDY
		if(txData[0] == STK8BAXX_OFSTCOMP1)
			return 0;
		
		buffer = txData[0];
		result = stk_i2c_rx(&buffer, 1);	
		if(result < 0)
		{
			printk(KERN_ERR "stk8baxx i2c_transfer fail to read back\n");
			return result;
		}
		
		if(buffer == txData[1])
			return 0;		
	}
	printk(KERN_ERR "stk8baxx i2c_transfer read back error,w=0x%x,r=0x%x\n", txData[1], buffer);
	return -EIO;	
}


static s32 stk8baxx_smbus_write_byte_data(u8 command, u8 value)
{
	int result;
	char buffer[2] = "";
	
	buffer[0] = command;
	buffer[1] = value;
	result = stk_i2c_tx(buffer, 2);
	return result;
}

static s32 stk8baxx_smbus_read_byte_data(u8 command)
{
	int result;
	char buffer = command;
	result = stk_i2c_rx(&buffer, 1);	
	return (result < 0) ? result : buffer;
}

static s32 stk8axxx_smbus_read_i2c_block_data(u8 command, u8 length, u8 *values)
{
	int result;
	char buffer[16] = "";
	
	buffer[0] = command;
	result = stk_i2c_rx(buffer, length);	
	if(result < 0)
		return result;
	memcpy(values, buffer, length);
	return length;
}


#ifdef ALLWINNER_PLATFOMR
/* Addresses to scan */
static union{
	unsigned short dirty_addr_buf[2];
	const unsigned short normal_i2c[2];
}u_i2c_addr = {{0x00},};
static __u32 twi_id = 0;
/**
 * gsensor_fetch_sysconfig_para - get config info from sysconfig.fex file.
 * return value:  
 *                    = 0; success;
 *                    < 0; err
 */
static int gsensor_fetch_sysconfig_para(void)
{
	int ret = -1;
	int device_used = -1;
	__u32 twi_addr = 0;
	char name[I2C_NAME_SIZE];
	script_parser_value_type_t type = SCIRPT_PARSER_VALUE_TYPE_STRING;
		
	printk("========%s===================\n", __func__);
	 
	if(SCRIPT_PARSER_OK != (ret = script_parser_fetch("gsensor_para", "gsensor_used", &device_used, 1))){
	                pr_err("%s: script_parser_fetch err.ret = %d. \n", __func__, ret);
	                goto script_parser_fetch_err;
	}
	if(1 == device_used){
		if(SCRIPT_PARSER_OK != script_parser_fetch_ex("gsensor_para", "gsensor_name", (int *)(&name), &type, sizeof(name)/sizeof(int))){
			pr_err("%s: line: %d script_parser_fetch err. \n", __func__, __LINE__);
			goto script_parser_fetch_err;
		}
		if(strcmp("stk8ba", name)){
			pr_err("%s: name %s does not match SENSOR_NAME=%s. \n", __func__, name,SENSOR_NAME);
			pr_err(SENSOR_NAME);
			//ret = 1;
			return ret;
		}
		if(SCRIPT_PARSER_OK != script_parser_fetch("gsensor_para", "gsensor_twi_addr", &twi_addr, sizeof(twi_addr)/sizeof(__u32))){
			pr_err("%s: line: %d: script_parser_fetch err. \n", name, __LINE__);
			goto script_parser_fetch_err;
		}
		u_i2c_addr.dirty_addr_buf[0] = twi_addr;
		u_i2c_addr.dirty_addr_buf[1] = I2C_CLIENT_END;
		printk("%s: after: gsensor_twi_addr is 0x%x, dirty_addr_buf: 0x%hx. dirty_addr_buf[1]: 0x%hx \n", \
			__func__, twi_addr, u_i2c_addr.dirty_addr_buf[0], u_i2c_addr.dirty_addr_buf[1]);

		if(SCRIPT_PARSER_OK != script_parser_fetch("gsensor_para", "gsensor_twi_id", &twi_id, 1)){
			pr_err("%s: script_parser_fetch err. \n", name);
			goto script_parser_fetch_err;
		}
		printk("%s: twi_id is %d. \n", __func__, twi_id);
		
		ret = 0;
		
	}else{
		pr_err("%s: gsensor_unused. \n",  __func__);
		ret = -1;
	}

	return ret;

script_parser_fetch_err:
	pr_notice("=========script_parser_fetch_err============\n");
	return ret;

}

/**
 * gsensor_detect - Device detection callback for automatic device creation
 * return value:  
 *                    = 0; success;
 *                    < 0; err
 */
static int gsensor_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	if(twi_id == adapter->nr){
		pr_info("%s: Detected chip %s at adapter %d, address 0x%02x\n",
			 __func__, SENSOR_NAME, i2c_adapter_id(adapter), client->addr);

		strlcpy(info->type, SENSOR_NAME, I2C_NAME_SIZE);
		return 0;
	}else{
		return -ENODEV;
	}
}

#endif


static int STK8BAXX_Init(struct stk8baxx_data *stk, struct i2c_client *client)
{
	int result;		
	unsigned short org_addr = 0;
	
	/*	sw reset	*/
	org_addr = client->addr;
	
	client->addr = org_addr & 0xFE;
	result = stk8baxx_smbus_write_byte_data(STK8BAXX_SWRST, STK8BAXX_SWRST_VAL);	
	printk(KERN_INFO "%s:issue sw reset to 0x%x, result=%d\n", __func__, client->addr, result);		

	if(result < 0)
	{
		client->addr = org_addr | 0x01;
		result = stk8baxx_smbus_write_byte_data(STK8BAXX_SWRST, STK8BAXX_SWRST_VAL);
		printk(KERN_INFO "%s:issue sw reset to 0x%x, result=%d\n", __func__, client->addr, result);				
	}
	msleep(1);
	
	client->addr = org_addr;
	printk(KERN_INFO "%s Revise I2C Address = 0x%x\n", __func__, client->addr);			

	result = stk8baxx_smbus_write_byte_data(STK8BAXX_POWMODE, STK8BAXX_MD_NORMAL);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_POWMODE, result);
		return result;
	}
	
#if (!STK_ACC_POLLING_MODE)
	/* map new data int to int1	*/
	result = stk8baxx_smbus_write_byte_data(STK8BAXX_INTMAP2, 0x01);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_INTMAP2, result);		
		return result;
	}	
	/*	enable new data int	*/
	result = stk8baxx_smbus_write_byte_data(STK8BAXX_INTEN2, 0x10);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_INTEN2, result);		
		return result;
	}			
	/*	non-latch int	*/
	result = stk8baxx_smbus_write_byte_data(STK8BAXX_INTCFG2, 0x00);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_INTCFG2, result);		
		return result;
	}	
	/*	filtered data source for new data int	*/
	result = stk8baxx_smbus_write_byte_data(STK8BAXX_DATASRC, 0x00);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_DATASRC, result);		
		return result;
	}		
	/*	int1, push-pull, active high	*/
	result = stk8baxx_smbus_write_byte_data(STK8BAXX_INTCFG1, 0x01);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_INTCFG1, result);		
		return result;
	}	
#endif	
	/*	+- 2g	*/
	result = stk8baxx_smbus_write_byte_data(STK8BAXX_RANGESEL, 0x03);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_RANGESEL, result);		
		return result;
	}	
	/*	ODR = 75 Hz	*/
	result = stk8baxx_smbus_write_byte_data(STK8BAXX_BWSEL, STK8BAXX_INIT_ODR);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_BWSEL, result);		
		return result;
	}		

	/*	i2c watchdog enable, 1 ms timer perios	*/
	result = stk8baxx_smbus_write_byte_data(STK8BAXX_INTFCFG, 0x04);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_INTFCFG, result);		
		return result;
	}			
	
	result = stk8baxx_smbus_write_byte_data(STK8BAXX_POWMODE, STK8BAXX_MD_SUSPEND);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_POWMODE, result);
		return result;
	}	
	printk(KERN_ERR "%s:return to suspend mode after init\n", __func__);	
	atomic_set(&stk->enabled, 0);
	stk->first_enable = true;	
	atomic_set(&stk->cali_status, STK_K_NO_CALI);	
	atomic_set(&stk->recv_reg, 0);	
#ifdef STK_LOWPASS
	memset(&stk->fir, 0x00, sizeof(stk->fir));  
	atomic_set(&stk->firlength, STK_FIR_LEN);
	atomic_set(&stk->fir_en, 1);
#endif

#ifdef STK_TUNE	
	stk_tune_offset[0] = 0;
	stk_tune_offset[1] = 0;
	stk_tune_offset[2] = 0;	
	stk_tune_done = 0;
#endif	
	return 0;
}

#ifdef STK_LOWPASS
static void STK8BAXX_LowPass(struct stk8baxx_data *stk, struct stk8baxx_acc *acc_lp)
{
	int idx, firlength = atomic_read(&stk->firlength);   
#ifdef STK_ZG_FILTER	
	s16 zero_fir = 0;	
#endif

	if(atomic_read(&stk->fir_en))
	{
		if(stk->fir.num < firlength)
		{                
			stk->fir.raw[stk->fir.num][0] = acc_lp->x;
			stk->fir.raw[stk->fir.num][1] = acc_lp->y;
			stk->fir.raw[stk->fir.num][2] = acc_lp->z;
			stk->fir.sum[0] += acc_lp->x;
			stk->fir.sum[1] += acc_lp->y;
			stk->fir.sum[2] += acc_lp->z;
			stk->fir.num++;
			stk->fir.idx++;
		}
		else
		{
			idx = stk->fir.idx % firlength;
			stk->fir.sum[0] -= stk->fir.raw[idx][0];
			stk->fir.sum[1] -= stk->fir.raw[idx][1];
			stk->fir.sum[2] -= stk->fir.raw[idx][2];
			stk->fir.raw[idx][0] = acc_lp->x;
			stk->fir.raw[idx][1] = acc_lp->y;
			stk->fir.raw[idx][2] = acc_lp->z;
			stk->fir.sum[0] += acc_lp->x;
			stk->fir.sum[1] += acc_lp->y;
			stk->fir.sum[2] += acc_lp->z;
			stk->fir.idx++;	
#ifdef STK_ZG_FILTER
			if( abs(stk->fir.sum[0]/firlength) <= STK_ZG_COUNT)
			{
				acc_lp->x = (stk->fir.sum[0]*zero_fir)/firlength;
			}
			else
			{
				acc_lp->x = stk->fir.sum[0]/firlength;
			}
			if( abs(stk->fir.sum[1]/firlength) <= STK_ZG_COUNT)
			{
				acc_lp->y = (stk->fir.sum[1]*zero_fir)/firlength;
			}
			else
			{
				acc_lp->y = stk->fir.sum[1]/firlength;
			}				
			if( abs(stk->fir.sum[2]/firlength) <= STK_ZG_COUNT)
			{
				acc_lp->z = (stk->fir.sum[2]*zero_fir)/firlength;
			}
			else
			{
				acc_lp->z = stk->fir.sum[2]/firlength;
			}				
#else
			acc_lp->x = stk->fir.sum[0]/firlength;
			acc_lp->y = stk->fir.sum[1]/firlength;
			acc_lp->z = stk->fir.sum[2]/firlength;
#endif 		
#ifdef STK_DEBUG_PRINT		
			/*
			printk("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
				stk->fir.raw[idx][0], stk->fir.raw[idx][1], stk->fir.raw[idx][2],
				stk->fir.sum[0], stk->fir.sum[1], stk->fir.sum[2],
				acc_lp->x, acc_lp->y, acc_lp->z);
			*/		
#endif		
		}
	}	
}
#endif


#ifdef STK_TUNE
static void STK8BAXX_ResetPara(void)
{
	int ii;
	for(ii=0;ii<3;ii++)
	{
		stk_tune_sum[ii] = 0;
		stk_tune_min[ii] = 4096;
		stk_tune_max[ii] = -4096;
	}
	return;
}

static void STK8BAXX_Tune(struct stk8baxx_data *stk, struct stk8baxx_acc *acc_xyz)
{	
	int ii;
	u8 offset[3];		
	s16 acc[3];
		
	if (stk_tune_done !=0)
		return;
	
	acc[0] = acc_xyz->x;
	acc[1] = acc_xyz->y;
	acc[2] = acc_xyz->z;
	
	if( event_since_en >= STK_TUNE_DELAY)
	{	
		if ((abs(acc[0]) <= STK_TUNE_XYOFFSET) && (abs(acc[1]) <= STK_TUNE_XYOFFSET)
			&& (abs(abs(acc[2])-LSB_1G) <= STK_TUNE_ZOFFSET))				
			stk_tune_index++;
		else
			stk_tune_index = 0;

		if (stk_tune_index==0)			
			STK8BAXX_ResetPara();			
		else
		{
			for(ii=0;ii<3;ii++)
			{
				stk_tune_sum[ii] += acc[ii];
				if(acc[ii] > stk_tune_max[ii])
					stk_tune_max[ii] = acc[ii];
				if(acc[ii] < stk_tune_min[ii])
					stk_tune_min[ii] = acc[ii];						
			}	
		}			

		if(stk_tune_index == STK_TUNE_NUM)
		{
			for(ii=0;ii<3;ii++)
			{
				if((stk_tune_max[ii] - stk_tune_min[ii]) > STK_TUNE_NOISE)
				{
					stk_tune_index = 0;
					STK8BAXX_ResetPara();
					return;
				}
			}				
			
			stk_tune_offset[0] = stk_tune_sum[0]/STK_TUNE_NUM;
			stk_tune_offset[1] = stk_tune_sum[1]/STK_TUNE_NUM;
			if (acc[2] > 0)
				stk_tune_offset[2] = stk_tune_sum[2]/STK_TUNE_NUM - LSB_1G;
			else
				stk_tune_offset[2] = stk_tune_sum[2]/STK_TUNE_NUM - (-LSB_1G);				
			
			offset[0] = (u8) (-stk_tune_offset[0]);
			offset[1] = (u8) (-stk_tune_offset[1]);
			offset[2] = (u8) (-stk_tune_offset[2]);

			STK8BAXX_SetOffset(stk, offset);
			stk_tune_offset_record[0] = offset[0];
			stk_tune_offset_record[1] = offset[1];
			stk_tune_offset_record[2] = offset[2];
						
			stk8baxx_store_in_file(offset, STK_K_SUCCESS_TUNE);		
			stk_tune_done = 1;				
			atomic_set(&stk->cali_status, STK_K_SUCCESS_TUNE);				
			event_since_en = 0;				
			printk(KERN_INFO "%s:TUNE done, %d,%d,%d\n", __func__, 
				offset[0], offset[1],offset[2]);		
		}	
	}		

	return;
}
#endif

#ifdef IS_STK8BA50
static int STK8BAXX_CheckReading(s16 acc[])
{	
	static int check_result = 0;
	static int event_no = 0;
	
	if(event_no > 20)
		return 0;
	if(event_no > 15 && event_no < 19)
	{
		if(acc[0] == 511 || acc[0] == -512 || acc[1] == 511 || acc[1] == -512 || 
				acc[2] == 511 || acc[2] == -512)
		{
			printk(KERN_INFO "%s: acc:%o,%o,%o\n", __func__, acc[0], acc[1], acc[2]);
			check_result++;		
		}	

		if(check_result == 3)
		{
			event_since_en_limit = 10000;
			printk(KERN_INFO "%s: incorrect reading\n", __func__);		
			return 1;
		}	
	}
	event_no++;
	return 0;		
}

static void STK8BAXX_SignConv(s16 raw_acc_data[], u8 acc_reg_data[])
{
	raw_acc_data[0] = acc_reg_data[1] << 8 | acc_reg_data[0];
	raw_acc_data[0] >>= 6;
	raw_acc_data[1] = acc_reg_data[3] << 8 | acc_reg_data[2];
	raw_acc_data[1] >>= 6;
	raw_acc_data[2] = acc_reg_data[5] << 8 | acc_reg_data[4];	
	raw_acc_data[2] >>= 6;
}

static int STK8BAXX_ReadSensorData(struct stk8baxx_data *stk)
{	
	int result;
	u8 acc_reg[6];
	int ii;
	struct stk8baxx_acc acc;
	int placement_no = (int)stk->stk8baxx_placement;
	s16 raw_acc[3];
	int k_status = atomic_read(&stk->cali_status);
	
	acc.x = acc.y = acc.z = 0;
	result = stk8axxx_smbus_read_i2c_block_data(STK8BAXX_XOUT1, 6, acc_reg);
	if (result < 0) 
	{
		printk(KERN_ERR "%s: failed to read acc data, error=%d\n", __func__, result);
		return result;
	}	
/*
	result = stk8baxx_smbus_read_byte_data(STK8BAXX_XOUT1);
	if (result < 0) 
	{
		printk(KERN_ERR "%s: failed to read acc data, error=%d\n", __func__, result);
		return result;
	}		
	acc_reg[0] = (u8)result;
	result = stk8baxx_smbus_read_byte_data(STK8BAXX_XOUT2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s: failed to read acc data, error=%d\n", __func__, result);
		return result;
	}		
	acc_reg[1] = (u8)result;	
	result = stk8baxx_smbus_read_byte_data(STK8BAXX_YOUT1);
	if (result < 0) 
	{
		printk(KERN_ERR "%s: failed to read acc data, error=%d\n", __func__, result);
		return result;
	}		
	acc_reg[2] = (u8)result;	
	result = stk8baxx_smbus_read_byte_data(STK8BAXX_YOUT2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s: failed to read acc data, error=%d\n", __func__, result);
		return result;
	}		
	acc_reg[3] = (u8)result;	
	result = stk8baxx_smbus_read_byte_data(STK8BAXX_ZOUT1);
	if (result < 0) 
	{
		printk(KERN_ERR "%s: failed to read acc data, error=%d\n", __func__, result);
		return result;
	}		
	acc_reg[4] = (u8)result;	
	result = stk8baxx_smbus_read_byte_data(STK8BAXX_ZOUT2);			
	if (result < 0) 
	{
		printk(KERN_ERR "%s: failed to read acc data, error=%d\n", __func__, result);
		return result;
	}	
	acc_reg[5] = (u8)result;	
*/	
	STK8BAXX_SignConv(raw_acc, acc_reg);
	STK8BAXX_CheckReading(raw_acc);
#ifdef STK_DEBUG_PRINT	
	//printk(KERN_INFO "%s: before trans, \t\t\t\t raw_acc=%4d, %4d, %4d\n", __func__, (int)raw_acc[0], (int)raw_acc[1], (int)raw_acc[2]);
#endif	
	for(ii=0;ii<3;ii++)
	{
		acc.x += raw_acc[ii] * coordinate_trans[placement_no][0][ii];
		acc.y += raw_acc[ii] * coordinate_trans[placement_no][1][ii];
		acc.z += raw_acc[ii] * coordinate_trans[placement_no][2][ii];
	}

	if(k_status == STK_K_RUNNING)
	{
		mutex_lock(&stk->value_lock);	
		stk->acc_xyz.x = acc.x;
		stk->acc_xyz.y = acc.y;
		stk->acc_xyz.z = acc.z;
		mutex_unlock(&stk->value_lock);		
		return 0;
	}	
	
#ifdef STK_LOWPASS
	STK8BAXX_LowPass(stk, &acc);
#endif  		
#ifdef STK_TUNE
	if((k_status & 0xF0) != 0)
		STK8BAXX_Tune(stk, &acc);		
#endif	
	mutex_lock(&stk->value_lock);	
	stk->acc_xyz.x = acc.x;
	stk->acc_xyz.y = acc.y;
	stk->acc_xyz.z = acc.z;
	mutex_unlock(&stk->value_lock);
#ifdef STK_DEBUG_PRINT	
	printk(KERN_INFO "stk8baxx acc= %4d, %4d, %4d\n", (int)stk->acc_xyz.x, (int)stk->acc_xyz.y, (int)stk->acc_xyz.z);
#endif	
	return 0;	
}
#elif (defined(IS_STK8BA22))	
static int STK8BAXX_CheckReading(s16 acc[])
{	
	static int check_result = 0;
	static int event_no = 0;
	
	if(event_no > 20)
		return 0;
	if(event_no > 15 && event_no < 19)
	{
		if(acc[0] == 127 || acc[0] == -128 || acc[1] == 127 || acc[1] == -128 || 
				acc[2] == 127 || acc[2] == -128)
		{
			printk(KERN_INFO "%s: acc:%o,%o,%o\n", __func__, acc[0], acc[1], acc[2]);
			check_result++;		
		}	

		if(check_result == 3)
		{
			event_since_en_limit = 10000;
			printk(KERN_INFO "%s: incorrect reading\n", __func__);		
			return 1;
		}	
	}
	event_no++;
	return 0;		
}

static void STK8BAXX_SignConv(s16 raw_acc_data[], u8 acc_reg_data[])
{	
	if(acc_reg_data[1] & 0x80)
	{
		raw_acc_data[0] = ~acc_reg_data[1];	
		raw_acc_data[0] &= 0xFF;	
		raw_acc_data[0] += 1;	
		raw_acc_data[0] = -raw_acc_data[0];	
	}
	else
		raw_acc_data[0] = acc_reg_data[1];

	if(acc_reg_data[3] & 0x80)
	{
		raw_acc_data[1] = ~acc_reg_data[3];	
		raw_acc_data[1] &= 0xFF;	
		raw_acc_data[1] += 1;	
		raw_acc_data[1] = -raw_acc_data[1];	
	}
	else
		raw_acc_data[1] = acc_reg_data[3];

	if(acc_reg_data[5] & 0x80)
	{
		raw_acc_data[2] = ~acc_reg_data[5];	
		raw_acc_data[2] &= 0xFF;	
		raw_acc_data[2] += 1;	
		raw_acc_data[2] = -raw_acc_data[2];	
	}
	else
		raw_acc_data[2] = acc_reg_data[5];	
}

static int STK8BAXX_ReadSensorData(struct stk8baxx_data *stk)
{	
	int result;
	u8 acc_reg[6];
	int ii;
	struct stk8baxx_acc acc;
	int placement_no = (int)stk->stk8baxx_placement;
	s16 raw_acc[3];
	int k_status = atomic_read(&stk->cali_status);
		
	acc.x = acc.y = acc.z = 0;
	
	result = stk8baxx_smbus_read_byte_data(STK8BAXX_XOUT2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s: failed to read acc data, error=%d\n", __func__, result);
		return result;
	}
	acc_reg[1] = (u8) result;
	result = stk8baxx_smbus_read_byte_data(STK8BAXX_YOUT2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s: failed to read acc data, error=%d\n", __func__, result);
		return result;
	}
	acc_reg[3] = (u8) result;
	result = stk8baxx_smbus_read_byte_data(STK8BAXX_ZOUT2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s: failed to read acc data, error=%d\n", __func__, result);
		return result;
	}
	
	acc_reg[5] = (u8) result;
	
	STK8BAXX_SignConv(raw_acc, acc_reg);
	STK8BAXX_CheckReading(raw_acc);
	
#ifdef STK_DEBUG_PRINT	
	//printk(KERN_INFO "%s: before trans, \t\t\t\t raw_acc=%4d, %4d, %4d\n", __func__, (int)raw_acc[0], (int)raw_acc[1], (int)raw_acc[2]);
#endif	
	for(ii=0;ii<3;ii++)
	{
		acc.x += raw_acc[ii] * coordinate_trans[placement_no][0][ii];
		acc.y += raw_acc[ii] * coordinate_trans[placement_no][1][ii];
		acc.z += raw_acc[ii] * coordinate_trans[placement_no][2][ii];
	}
	
	if(k_status == STK_K_RUNNING)
	{
		mutex_lock(&stk->value_lock);	
		stk->acc_xyz.x = acc.x;
		stk->acc_xyz.y = acc.y;
		stk->acc_xyz.z = acc.z;
		mutex_unlock(&stk->value_lock);		
		return 0;
	}		
	
#ifdef STK_LOWPASS
	STK8BAXX_LowPass(stk, &acc);	
#endif  	
#ifdef STK_TUNE
	if((k_status & 0xF0) != 0)
		STK8BAXX_Tune(stk, &acc);		
#endif	
	
	mutex_lock(&stk->value_lock);	
	stk->acc_xyz.x = acc.x;
	stk->acc_xyz.y = acc.y;
	stk->acc_xyz.z = acc.z;
	mutex_unlock(&stk->value_lock);

#ifdef STK_DEBUG_PRINT	
	printk(KERN_INFO "stk8baxx acc= %4d, %4d, %4d\n", (int)stk->acc_xyz.x, (int)stk->acc_xyz.y, (int)stk->acc_xyz.z);
#endif	
	return 0;	
}
#endif


static int STK8BAXX_ReportValue(struct stk8baxx_data *stk)
{
	if(event_since_en < 1200)
		event_since_en++;	
	
	if(event_since_en < event_since_en_limit)
		return 0;	

	//mutex_lock(&stk->value_lock);
#ifdef STK_DEBUG_PRINT	
	printk(KERN_INFO "%s:%4d,%4d,%4d\n", __func__, stk->acc_xyz.x, stk->acc_xyz.y, stk->acc_xyz.z);	
#endif	
	input_report_abs(stk->input_dev, ABS_X, stk->acc_xyz.x);  
	input_report_abs(stk->input_dev, ABS_Y, stk->acc_xyz.y);
	input_report_abs(stk->input_dev, ABS_Z, stk->acc_xyz.z);
	//mutex_unlock(&stk->value_lock);
	input_sync(stk->input_dev);
	return 0;
}

static int STK8BAXX_EnterActive(struct stk8baxx_data *stk)
{	
	int result;
	result = stk8baxx_smbus_write_byte_data(STK8BAXX_POWMODE, STK8BAXX_MD_NORMAL);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_POWMODE, result);
		return result;
	}
	return 0;
}

static int STK8BAXX_EnterSuspend(struct stk8baxx_data *stk)
{
	int result;
	result = stk8baxx_smbus_write_byte_data(STK8BAXX_POWMODE, STK8BAXX_MD_SUSPEND);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_POWMODE, result);
		return result;
	}
	return 0;
}

#ifdef STK_HOLD_ODR
static int STK8BAXX_SetDelay(struct stk8baxx_data *stk, uint32_t sdelay_ns)
{
	printk(KERN_INFO "%s: ODR is fixed\n", __func__);
	return 0;
}
#else
static int STK8BAXX_SetDelay(struct stk8baxx_data *stk, uint32_t sdelay_ns)
{
	unsigned char sr_no;	
	int result;	
	uint32_t sdelay_us = sdelay_ns / 1000;
	char en;
        
        pr_info("sdelay_ns = %d\n", sdelay_ns);	
	for(sr_no=0;sr_no<STK8BAXX_SPTIME_NO;sr_no++)
	{
		if(sdelay_us >= STK8BAXX_SAMPLE_TIME[sr_no])	
			break;		
	}	

	if(sr_no == STK8BAXX_SPTIME_NO)
	{
		sdelay_ns = STK8BAXX_SAMPLE_TIME[STK8BAXX_SPTIME_NO-1] * 1000;				
		sr_no--;
	}
	sr_no += STK8BAXX_SPTIME_BASE;
#ifdef STK_DEBUG_PRINT		
	printk(KERN_INFO "%s:sdelay_us=%u, sr_no=0x%x\n", __func__, sdelay_ns/1000, sr_no);
#endif	
	
	result = STK8BAXX_GetEnable(stk, &en);
	if(result < 0)
		printk(KERN_ERR "%s: STK8BAXX_GetEnable failed, error=%d\n", __func__, result);
	
	mutex_lock(&stk->write_lock);		
	if(en == 0)
		STK8BAXX_EnterActive(stk);
	
	result = stk8baxx_smbus_write_byte_data(STK8BAXX_BWSEL, sr_no);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_BWSEL, result);
		return result;
	}
	if(en == 0)
		STK8BAXX_EnterSuspend(stk);
	
	stk->poll_delay = ns_to_ktime(sdelay_ns);
	
#if defined(STK_LOWPASS)
	stk->fir.num = 0;
	stk->fir.idx = 0;
	stk->fir.sum[0] = 0;
	stk->fir.sum[1] = 0;
	stk->fir.sum[2] = 0;
#endif		
	mutex_unlock(&stk->write_lock);	
	
	return 0;	
}
#endif

static s64 STK8BAXX_GetDelay(struct stk8baxx_data *stk)
{
	int result, sample_time;
	s64 delay_ns;
	char en;
	
	result = STK8BAXX_GetEnable(stk, &en);
	if(result < 0)
		printk(KERN_ERR "%s: STK8BAXX_GetEnable failed, error=%d\n", __func__, result);
	
	mutex_lock(&stk->write_lock);		
	if(en == 0)
		STK8BAXX_EnterActive(stk);
	sample_time = stk8baxx_smbus_read_byte_data(STK8BAXX_BWSEL);
	if (sample_time < 0) 
	{
		printk(KERN_ERR "%s: failed to read reg 0x%x, error=%d\n", __func__, STK8BAXX_BWSEL, sample_time);
		return sample_time;
	}
	if(en == 0)
		STK8BAXX_EnterSuspend(stk);
	mutex_unlock(&stk->write_lock);		
        pr_info("sampl3_time=%d,STK8BAXX_SPTIME_BASE=%d\n",sample_time,STK8BAXX_SPTIME_BASE);	
	delay_ns = STK8BAXX_SAMPLE_TIME[sample_time - STK8BAXX_SPTIME_BASE];
	printk(KERN_INFO "%s: delay =%lld us\n", __func__,  delay_ns);	
	return (delay_ns*1000);	
}

static int STK8BAXX_SetOffset(struct stk8baxx_data *stk, u8 offset[])
{
	int result;
	char en;
	result = STK8BAXX_GetEnable(stk, &en);
	if(result < 0)
	{
		printk(KERN_ERR "%s: STK8BAXX_GetEnable failed, error=%d\n", __func__, result);
	}
	
	if(!en)
		STK8BAXX_EnterActive(stk);
		
	result = stk8baxx_smbus_write_byte_data(STK8BAXX_OFSTFILTX, offset[0]);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_OFSTFILTX, result);
		return result;
	}	
	result = stk8baxx_smbus_write_byte_data(STK8BAXX_OFSTFILTY, offset[1]);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_OFSTFILTX, result);
		return result;
	}	
	result = stk8baxx_smbus_write_byte_data(STK8BAXX_OFSTFILTZ, offset[2]);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_OFSTFILTX, result);
		return result;
	}	

	if(!en)
		STK8BAXX_EnterSuspend(stk);
	return 0;
}

static int STK8BAXX_GetOffset(struct stk8baxx_data *stk, u8 offset[])
{
	int result;	
	char en;
	
	result = STK8BAXX_GetEnable(stk, &en);
	if(result < 0)
		printk(KERN_ERR "%s: STK8BAXX_GetEnable failed, error=%d\n", __func__, result);
	if(en == 0)
		STK8BAXX_EnterActive(stk);	
	
	result = stk8axxx_smbus_read_i2c_block_data(STK8BAXX_OFSTFILTX, 3, offset);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_OFSTFILTX, result);
		return result;
	}	
		
	/*	
	
	result = stk8baxx_smbus_read_byte_data(STK8BAXX_OFSTFILTX);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to read from reg 0x%x, error=%d\n", __func__, STK8BAXX_OFSTFILTX, result);
		return result;
	}
	offset[0] = result;
	result = stk8baxx_smbus_read_byte_data(STK8BAXX_OFSTFILTY);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to read from reg 0x%x, error=%d\n", __func__, STK8BAXX_OFSTFILTY, result);
		return result;
	}
	offset[1] = result;
	result = stk8baxx_smbus_read_byte_data(STK8BAXX_OFSTFILTZ);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to read from reg 0x%x, error=%d\n", __func__, STK8BAXX_OFSTFILTZ, result);
		return result;
	}	
	offset[2] = result;
	*/
	if(en == 0)
		STK8BAXX_EnterSuspend(stk);		
	return 0;
}

static int STK8BAXX_SetRange(struct stk8baxx_data *stk, char srange)
{
	int result;
	s8 write_buffer = 0;
	char en;
		
	if(srange > STK8BAXX_RANGE_MASK)
	{
		printk(KERN_ERR "%s:range=0x%x, out of range\n", __func__, srange);
		return -EINVAL;
	}
#ifdef STK_DEBUG_PRINT	
	printk(KERN_INFO "%s:range=0x%x\n", __func__, srange);		
#endif	
	switch(srange)
	{
	case STK8BAXX_RNG_2G:
		write_buffer = STK8BAXX_RNG_2G;
		break;
	case STK8BAXX_RNG_4G:
		write_buffer = STK8BAXX_RNG_4G;
		break;
	case STK8BAXX_RNG_8G:	
		write_buffer = STK8BAXX_RNG_8G;
		break;
	case STK8BAXX_RNG_16G:	
		write_buffer = STK8BAXX_RNG_16G;
		break;
	default:
		write_buffer = STK8BAXX_RNG_2G;
		printk(KERN_ERR "%s: unknown range, set as STK8BAXX_RNG_2G\n", __func__);
	}
	
	result = STK8BAXX_GetEnable(stk, &en);
	if(result < 0)
		printk(KERN_ERR "%s: STK8BAXX_GetEnable failed, error=%d\n", __func__, result);
	
	if(en == 0)
		STK8BAXX_EnterActive(stk);	
	result = stk8baxx_smbus_write_byte_data(STK8BAXX_RANGESEL, write_buffer);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_RANGESEL, result);
		return result;
	}	
	if(en == 0)
		STK8BAXX_EnterSuspend(stk);
	
	return 0;	
}

static int STK8BAXX_GetRange(struct stk8baxx_data *stk, char *grange)
{
	int result;
	char en;
	
	result = STK8BAXX_GetEnable(stk, &en);
	if(result < 0)
		printk(KERN_ERR "%s: STK8BAXX_GetEnable failed, error=%d\n", __func__, result);
		
	if(en == 0)
		STK8BAXX_EnterActive(stk);		
	
	result = stk8baxx_smbus_read_byte_data(STK8BAXX_RANGESEL);
	if (result < 0) 
	{
		printk(KERN_ERR "%s: failed to read reg 0x%x, error=%d\n", __func__, STK8BAXX_RANGESEL, result);
		return result;
	}	
	
	if(en == 0)
		STK8BAXX_EnterSuspend(stk);
			
	switch(result)
	{
	case STK8BAXX_RNG_2G:
		*grange = 2;
		break;
	case STK8BAXX_RNG_4G:
		*grange = 4;
		break;
	case STK8BAXX_RNG_8G:
		*grange = 8;
		break;
	case STK8BAXX_RNG_16G:
		*grange = 16;	
		break;
	}
	
	return 0;
}


static int STK8BAXX_GetCali(struct stk8baxx_data *stk)
{	
	char r_buf[STK_ACC_CALI_FILE_SIZE] = {0};
	
#ifdef STK_TUNE		
	printk(KERN_INFO "%s: stk_tune_done=%d, stk_tune_index=%d, stk_tune_offset=%d,%d,%d\n", __func__, 
		stk_tune_done, stk_tune_index, stk_tune_offset_record[0], stk_tune_offset_record[1], 
		stk_tune_offset_record[2]);
#endif		
	
	if ((stk8baxx_get_file_content(r_buf, STK_ACC_CALI_FILE_SIZE)) == 0)
	{
		if(r_buf[0] == STK_ACC_CALI_VER0 && r_buf[1] == STK_ACC_CALI_VER1)
		{
			atomic_set(&stk->cali_status, (int)r_buf[5]);
			printk(KERN_INFO "%s: offset:%d,%d,%d, mode=0x%x\n", __func__, 
							r_buf[2], r_buf[3], r_buf[4], r_buf[5]);							
		}
		else
		{
			printk(KERN_ERR "%s: cali version number error! r_buf=0x%x,0x%x,0x%x,0x%x,0x%x\n", 
							__func__, r_buf[0], r_buf[1], r_buf[2], r_buf[3], r_buf[4]);									
		}
	}
	return 0;
}

static int STK8BAXX_SetOfstTarget(struct stk8baxx_data *stk)
{
	int result, retry = 0;
	s16 raw_acc_z;
	u8 target = 0;
	
	do {
		msleep(5);
		result = stk8baxx_smbus_read_byte_data(STK8BAXX_ZOUT1);
		if (result < 0) 
		{
			printk(KERN_ERR "%s: failed to read reg 0x%x, error=%d\n", 
							__func__, STK8BAXX_ZOUT1, result);
			return result;
		}	
		retry++;
	} while (!(result & STK8BAXX_O_NEW) && retry<5);	
	
#ifdef IS_STK8BA50	
	raw_acc_z = result;
	result = stk8baxx_smbus_read_byte_data(STK8BAXX_ZOUT2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s: failed to read acc data, error=%d\n", __func__, result);
		return result;
	}	
	raw_acc_z |= (result << 8);
	raw_acc_z >>= 6;		
#elif defined (IS_STK8BA22)
	result = stk8baxx_smbus_read_byte_data(STK8BAXX_ZOUT2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s: failed to read acc data, error=%d\n", __func__, result);
		return result;
	}
	raw_acc_z = result;
	if(raw_acc_z & 0x80)
	{
		raw_acc_z = ~raw_acc_z;	
		raw_acc_z &= 0xFF;	
		raw_acc_z += 1;	
		raw_acc_z = -raw_acc_z;	
	}
	
#endif
	if(raw_acc_z <= 0)
		target = CAL_TG_X0_Y0_ZNEG1;
	else
		target = CAL_TG_X0_Y0_ZPOS1;
	printk(KERN_INFO "%s: read az=%d, target=0x%x\n", __func__, raw_acc_z, target);	
	
	result = stk8baxx_smbus_write_byte_data(STK8BAXX_OFSTCOMP2, target);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, 
						STK8BAXX_OFSTCOMP2, result);
		return result;
	}		
	/*
		if(stk->stk8baxx_placement >=0 && stk->stk8baxx_placement <=3)
		target = CAL_TG_X0_Y0_ZPOS1;
		else if(stk->stk8baxx_placement >=4 && stk->stk8baxx_placement <=7)
		target = CAL_TG_X0_Y0_ZNEG1;
		else
		printk(KERN_ERR "%s: invalid stk8baxx_placement(%d)\n", __func__, stk->stk8baxx_placement);
	 */		
	
	return 0;
}

static int STK8BAXX_SetKEachAxis(struct stk8baxx_data *stk)
{
	int result, retry = 0;
	
	result = stk8baxx_smbus_write_byte_data(STK8BAXX_OFSTCOMP1, CAL_AXIS_X_EN);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_OFSTCOMP1, result);
		return result;
	}	
	msleep(250);
	do {
		msleep(5);
		result = stk8baxx_smbus_read_byte_data(STK8BAXX_OFSTCOMP1);
		if (result < 0) 
		{
			printk(KERN_ERR "%s: failed to read reg 0x%x, error=%d\n", __func__, STK8BAXX_OFSTCOMP1, result);
			return result;
		}	
		retry++;
	} while (!(result & STK8BAXX_OF_CAL_DRY_MASK) && retry<25);
	result = stk8baxx_smbus_write_byte_data(STK8BAXX_OFSTCOMP1, CAL_AXIS_Y_EN);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_OFSTCOMP1, result);
		return result;
	}	
	retry = 0;
	msleep(250);
	do {
		msleep(5);		
		result = stk8baxx_smbus_read_byte_data(STK8BAXX_OFSTCOMP1);
		if (result < 0) 
		{
			printk(KERN_ERR "%s: failed to read reg 0x%x, error=%d\n", __func__, STK8BAXX_OFSTCOMP1, result);
			return result;
		}	
		retry++;
	} while (!(result & STK8BAXX_OF_CAL_DRY_MASK) && retry<25);
	result = stk8baxx_smbus_write_byte_data(STK8BAXX_OFSTCOMP1, CAL_AXIS_Z_EN);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_OFSTCOMP1, result);
		return result;
	}	
	retry = 0;
	msleep(250);
	do {
		msleep(5);		
		result = stk8baxx_smbus_read_byte_data(STK8BAXX_OFSTCOMP1);
		if (result < 0) 
		{
			printk(KERN_ERR "%s: failed to read reg 0x%x, error=%d\n", __func__, STK8BAXX_OFSTCOMP1, result);
			return result;
		}	
		retry++;
	} while (!(result & STK8BAXX_OF_CAL_DRY_MASK) && retry<25);		
	return 0;
}

static int STK8BAXX_SetCali(struct stk8baxx_data *stk, char sstate)
{
	int result;
	u8 offset[3];
	char enabled;
	s64 ord_delay;

	atomic_set(&stk->cali_status, STK_K_RUNNING);		
	STK8BAXX_GetEnable(stk, &enabled);
	ord_delay = STK8BAXX_GetDelay(stk);
	result = STK8BAXX_SetDelay(stk, 13333000);	/*	75 Hz ODR */	
	if(result < 0)
	{
		printk(KERN_ERR "%s:failed to STK8BAXX_SetDelay, error=%d\n", __func__, result);
		atomic_set(&stk->cali_status, STK_K_FAIL_I2C);	
		goto k_exit;
	}
	mutex_lock(&stk->write_lock);	
	mutex_lock(&stk->value_lock);	
	
	if(enabled)
	{
#if STK_ACC_POLLING_MODE
		hrtimer_cancel(&stk->acc_timer);
#else
		disable_irq((unsigned int)stk->irq);	
#endif	//#if STK_ACC_POLLING_MODE	
	}		
	else
	{		
		result = stk8baxx_smbus_write_byte_data(STK8BAXX_POWMODE, STK8BAXX_MD_NORMAL);
		if (result < 0) 
		{
			printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_POWMODE, result);
			atomic_set(&stk->cali_status, STK_K_FAIL_I2C);	
			goto k_exit;
		}		
	}	
	
	result = stk8baxx_smbus_write_byte_data(STK8BAXX_OFSTCOMP1, CAL_OFST_RST);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_OFSTCOMP1, result);
		atomic_set(&stk->cali_status, STK_K_FAIL_I2C);
		goto k_exit;
	}
	
	msleep(10);
	result = STK8BAXX_SetOfstTarget(stk);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to STK8BAXX_SetOfstTarget, error=%d\n", __func__, result);
		atomic_set(&stk->cali_status, STK_K_FAIL_I2C);
		goto k_exit;
	}
	
	result = STK8BAXX_SetKEachAxis(stk);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to STK8BAXX_SetKEachAxis, error=%d\n", __func__, result);
		atomic_set(&stk->cali_status, STK_K_FAIL_I2C);
		goto k_exit;
	}
	
	STK8BAXX_GetOffset(stk, offset);
	printk(KERN_INFO "%s: after compensation, offset=0x%x,0x%x,0x%x\n", __func__, offset[0], offset[1], offset[2]);	
	result = stk8baxx_store_in_file(offset, STK_K_SUCCESS_FILE);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to stk8baxx_store_in_file, error=%d\n", __func__, result);
		atomic_set(&stk->cali_status, STK_K_FAIL_W_FILE);
		goto k_exit;
	}			
	
#ifdef STK_TUNE	
	stk_tune_offset_record[0] = 0;
	stk_tune_offset_record[1] = 0;
	stk_tune_offset_record[2] = 0;
	stk_tune_done = 1;
#endif		
	
	if(enabled)
	{
#if STK_ACC_POLLING_MODE
		hrtimer_start(&stk->acc_timer, stk->poll_delay, HRTIMER_MODE_REL);	
#else
		enable_irq((unsigned int)stk->irq);	
#endif	//#if STK_ACC_POLLING_MODE			
	}		
	else
	{		
		result = stk8baxx_smbus_write_byte_data(STK8BAXX_POWMODE, STK8BAXX_MD_SUSPEND);
		if (result < 0) 
		{
			printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_POWMODE, result);
			atomic_set(&stk->cali_status, STK_K_FAIL_I2C);
			goto k_exit;
		}		
	}	

	mutex_unlock(&stk->value_lock);	
	mutex_unlock(&stk->write_lock);	
	STK8BAXX_SetDelay(stk, ord_delay);
	atomic_set(&stk->cali_status, STK_K_SUCCESS_FILE);
	printk(KERN_INFO "%s: successful calibration\n", __func__);
	return 0;
	
k_exit:	
	if(enabled)
	{
#if STK_ACC_POLLING_MODE
		hrtimer_start(&stk->acc_timer, stk->poll_delay, HRTIMER_MODE_REL);	
#else
		enable_irq((unsigned int)stk->irq);	
#endif	//#if STK_ACC_POLLING_MODE			
	}		
	else
	{		
		stk8baxx_smbus_write_byte_data(STK8BAXX_POWMODE, STK8BAXX_MD_SUSPEND);		
	}	
	mutex_unlock(&stk->value_lock);	
	mutex_unlock(&stk->write_lock);	
	STK8BAXX_SetDelay(stk, ord_delay);
	return result;
}

static void STK8BAXX_LoadCali(struct stk8baxx_data *stk)
{
	char r_buf[STK_ACC_CALI_FILE_SIZE] = {0};
	u8 offset[3], mode;		
	
	if ((stk8baxx_get_file_content(r_buf, STK_ACC_CALI_FILE_SIZE)) == 0)
	{
		if(r_buf[0] == STK_ACC_CALI_VER0 && r_buf[1] == STK_ACC_CALI_VER1)
		{
			offset[0] = r_buf[2];
			offset[1] = r_buf[3];
			offset[2] = r_buf[4];
			mode =  r_buf[5];
			STK8BAXX_SetOffset(stk, offset);
			atomic_set(&stk->cali_status, mode);
			printk(KERN_INFO "%s: set offset:%d,%d,%d, mode=%d\n", __func__, 
							offset[0], offset[1], offset[2], mode);
#ifdef STK_TUNE			
			stk_tune_offset_record[0] = offset[0];
			stk_tune_offset_record[1] = offset[1];
			stk_tune_offset_record[2] = offset[2];
#endif 								
		}
		else
		{
			printk(KERN_ERR "%s: cali version number error! r_buf=0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n", 
					__func__, r_buf[0], r_buf[1], r_buf[2], r_buf[3], r_buf[4], r_buf[5]);
		}
	}
#ifdef STK_TUNE		
	else if(stk_tune_offset_record[0] != 0 || stk_tune_offset_record[1] != 0 || stk_tune_offset_record[2] != 0)
	{
		STK8BAXX_SetOffset(stk, stk_tune_offset_record);	
		stk_tune_done = 1;				
		atomic_set(&stk->cali_status, STK_K_SUCCESS_TUNE);	
		printk(KERN_INFO "%s: set offset:%d,%d,%d\n", __func__, stk_tune_offset_record[0], 
			stk_tune_offset_record[1],stk_tune_offset_record[2]);	
	}	
#endif		
	else
	{
		offset[0] = offset[1] = offset[2] = 0;
		stk8baxx_store_in_file(offset, STK_K_NO_CALI);
		atomic_set(&stk->cali_status, STK_K_NO_CALI);	
	}
	printk(KERN_INFO "%s: cali_status=0x%x\n", __func__, atomic_read(&stk->cali_status));
}

static int STK8BAXX_SetEnable(struct stk8baxx_data *stk, char en)
{
	s8 result;
	s8 write_buffer = 0;
	int new_enabled = (en)?1:0; 
	int k_status = atomic_read(&stk->cali_status);
	
	if(stk->first_enable && k_status != STK_K_RUNNING)
	{
		stk->first_enable = false;
		STK8BAXX_LoadCali(stk);
	}
	mutex_lock(&stk->write_lock);	
	if(new_enabled == atomic_read(&stk->enabled))
	{
         pr_info("the same value");	
         
	mutex_unlock(&stk->write_lock);	
         return 0;
        }
	printk(KERN_INFO "%s:%x\n", __func__, en);

	if(en)
	{
#ifdef STK_TUNE		
		if((k_status & 0xF0) != 0 && stk_tune_done == 0)
		{
			stk_tune_index = 0;
			STK8BAXX_ResetPara();
		}
#endif	
	
#if STK_ACC_POLLING_MODE
		hrtimer_start(&stk->acc_timer, stk->poll_delay, HRTIMER_MODE_REL);	
#else
		enable_irq((unsigned int)stk->irq);	
#endif	//#if STK_ACC_POLLING_MODE	
	}
	else
	{
#if STK_ACC_POLLING_MODE
		hrtimer_cancel(&stk->acc_timer);
		cancel_work_sync(&stk->stk_work);	
#else
		disable_irq((unsigned int)stk->irq);	
#endif	//#if STK_ACC_POLLING_MODE
	}
	
	if(en)
		write_buffer = STK8BAXX_MD_NORMAL;
	else
		write_buffer = STK8BAXX_MD_SUSPEND;		
	
	result = stk8baxx_smbus_write_byte_data(STK8BAXX_POWMODE, write_buffer);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, STK8BAXX_POWMODE, result);
		return result;
	}
	
	atomic_set(&stk->enabled, new_enabled);
	mutex_unlock(&stk->write_lock);	
	return 0;
}

static int STK8BAXX_GetEnable(struct stk8baxx_data *stk, char *gState)
{
	*gState = atomic_read(&stk->enabled);
	return 0;
}

#ifdef STK8BAXX_PERMISSION_THREAD
//SYSCALL_DEFINE3(fchmodat, int, dfd, const char __user *, filename, mode_t, mode);
static struct task_struct *STKPermissionThread = NULL;

static int stk8baxx_permis_thread(void *data)
{
#if 0
	int ret = 0;
	int retry = 0;
	mm_segment_t fs = get_fs();
	set_fs(KERNEL_DS);	
	msleep(20000);
	do{
		msleep(5000);
		ret = sys_fchmodat(AT_FDCWD, "/sys/class/input/input0/driver/cali" , 0666);
		ret = sys_fchmodat(AT_FDCWD, "/sys/class/input/input1/driver/cali" , 0666);
		ret = sys_fchmodat(AT_FDCWD, "/sys/class/input/input2/driver/cali" , 0666);
		ret = sys_fchmodat(AT_FDCWD, "/sys/class/input/input3/driver/cali" , 0666);
		ret = sys_fchmodat(AT_FDCWD, "/sys/class/input/input4/driver/cali" , 0666);
		ret = sys_fchmodat(AT_FDCWD, "/sys/class/input/input0/cali" , 0666);
		ret = sys_fchmodat(AT_FDCWD, "/sys/class/input/input1/cali" , 0666);
		ret = sys_fchmodat(AT_FDCWD, "/sys/class/input/input2/cali" , 0666);
		ret = sys_fchmodat(AT_FDCWD, "/sys/class/input/input3/cali" , 0666);		
		ret = sys_fchmodat(AT_FDCWD, "/sys/class/input/input4/cali" , 0666);	
		ret = sys_chmod(STK_ACC_CALI_FILE , 0666);	
		ret = sys_fchmodat(AT_FDCWD, STK_ACC_CALI_FILE , 0666);
		if(retry++ > 20)
			break;
	}while(ret == -ENOENT);
	set_fs(fs);
	printk(KERN_INFO "%s exit, retry=%d\n", __func__, retry);
	return 0;
#endif
}
#endif	/*	#ifdef STK8BAXX_PERMISSION_THREAD	*/

static int32_t stk8baxx_get_file_content(char * r_buf, int8_t buf_size)
{
	struct file  *cali_file;
	mm_segment_t fs;	
	ssize_t ret;	

    cali_file = filp_open(STK_ACC_CALI_FILE, O_RDONLY,0);
    if(IS_ERR(cali_file))
	{
        printk(KERN_ERR "%s: filp_open error, no offset file!\n", __func__);
        return -ENOENT;
	}
	else
	{
		fs = get_fs();
		set_fs(get_ds());
		ret = cali_file->f_op->read(cali_file,r_buf, STK_ACC_CALI_FILE_SIZE,&cali_file->f_pos);
		if(ret < 0)
		{
			printk(KERN_ERR "%s: read error, ret=%d\n", __func__, ret);
			filp_close(cali_file,NULL);
			return -EIO;
		}		
		set_fs(fs);
    }
    filp_close(cali_file,NULL);	
	return 0;	
}


static int stk8baxx_store_in_file(u8 offset[], u8 status)
{
	struct file  *cali_file;
	char r_buf[STK_ACC_CALI_FILE_SIZE] = {0};
	char w_buf[STK_ACC_CALI_FILE_SIZE] = {0};	
	mm_segment_t fs;	
	ssize_t ret;
	int8_t i;
	
	w_buf[0] = STK_ACC_CALI_VER0;
	w_buf[1] = STK_ACC_CALI_VER1;
	w_buf[2] = offset[0];
	w_buf[3] = offset[1];
	w_buf[4] = offset[2];
	w_buf[5] = status;
	
    cali_file = filp_open(STK_ACC_CALI_FILE, O_CREAT | O_RDWR,0666);
	
    if(IS_ERR(cali_file))
	{
        printk(KERN_ERR "%s: filp_open error!\n", __func__);
        return -STK_K_FAIL_OPEN_FILE;
	}
	else
	{
		fs = get_fs();
		set_fs(get_ds());
		
		ret = cali_file->f_op->write(cali_file,w_buf, STK_ACC_CALI_FILE_SIZE,&cali_file->f_pos);
		if(ret != STK_ACC_CALI_FILE_SIZE)
		{
			printk(KERN_ERR "%s: write error!\n", __func__);
			filp_close(cali_file,NULL);
			return -STK_K_FAIL_W_FILE;
		}
		cali_file->f_pos=0x00;
		ret = cali_file->f_op->read(cali_file,r_buf, STK_ACC_CALI_FILE_SIZE,&cali_file->f_pos);
		if(ret < 0)
		{
			printk(KERN_ERR "%s: read error!\n", __func__);
			filp_close(cali_file,NULL);
			return -STK_K_FAIL_R_BACK;

		}		
		set_fs(fs);		

		for(i=0;i<STK_ACC_CALI_FILE_SIZE;i++)
		{
			if(r_buf[i] != w_buf[i])
			{
				printk(KERN_ERR "%s: read back error, r_buf[%x](0x%x) != w_buf[%x](0x%x)\n", __func__, i, r_buf[i], i, w_buf[i]);
				filp_close(cali_file,NULL);
				return -STK_K_FAIL_R_BACK_COMP;
			}
		}
    }
    filp_close(cali_file,NULL);	

#ifdef STK_PERMISSION_THREAD	
	fs = get_fs();
	set_fs(KERNEL_DS);	
	ret = sys_chmod(STK_ACC_CALI_FILE , 0666);	
	ret = sys_fchmodat(AT_FDCWD, STK_ACC_CALI_FILE , 0666);
	set_fs(fs);	
#endif
	printk(KERN_INFO "%s successfully\n", __func__);	
	return 0;		
}

static ssize_t stk8baxx_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	
	struct stk8baxx_data *stk = dev_get_drvdata(dev);
	char en;
	STK8BAXX_GetEnable(stk, &en);
	return scnprintf(buf, PAGE_SIZE, "%d\n", en);
}

static ssize_t stk8baxx_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct stk8baxx_data *stk = dev_get_drvdata(dev);	
	unsigned long data;
	int error;
	error = strict_strtoul(buf, 10, &data);
	if (error)
	{
		printk(KERN_ERR "%s: strict_strtoul failed, error=%d\n", __func__, error);
		return error;
	}

	if ((data == 0)||(data==1)) 
	{
		STK8BAXX_SetEnable(stk,data);
	}
	else
		printk(KERN_ERR "%s: invalud argument, data=%ld\n", __func__, data);
	return count;
}

static ssize_t stk8baxx_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct stk8baxx_data *stk = dev_get_drvdata(dev);	
	int ddata[3];
	
	printk(KERN_INFO "driver version:%s\n",STK_ACC_DRIVER_VERSION);	
	STK8BAXX_ReadSensorData(stk);
	mutex_lock(&stk->value_lock);
	ddata[0]= stk->acc_xyz.x;
	ddata[1]= stk->acc_xyz.y;
	ddata[2]= stk->acc_xyz.z;
	mutex_unlock(&stk->value_lock);
	return scnprintf(buf, PAGE_SIZE, "%d %d %d\n", ddata[0], ddata[1], ddata[2]);
}

static ssize_t stk8baxx_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct stk8baxx_data *stk = dev_get_drvdata(dev);	
        s64  data = STK8BAXX_GetDelay(stk);
        do_div(data, 1000000);
        /* HAL in ms*/
	return scnprintf(buf, PAGE_SIZE, "%lld\n", data);	
}

static ssize_t stk8baxx_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct stk8baxx_data *stk = dev_get_drvdata(dev);	
	unsigned long data;
	int error;
	
	error = strict_strtoul(buf, 10, &data);
	if (error)
	{
		printk(KERN_ERR "%s: strict_strtoul failed, error=%d\n", __func__, error);
		return error;
	}
        /* HAL in ms*/
        data = data * 1000000;	
	STK8BAXX_SetDelay(stk, data);
	return count;
}

static ssize_t stk8baxx_cali_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct stk8baxx_data *stk = dev_get_drvdata(dev);	
	int status = atomic_read(&stk->cali_status);
	
	if(status != STK_K_RUNNING)
		STK8BAXX_GetCali(stk);	
	return scnprintf(buf, PAGE_SIZE, "%02x\n", atomic_read(&stk->cali_status));
}

static ssize_t stk8baxx_cali_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct stk8baxx_data *stk = dev_get_drvdata(dev);	
	
	if (sysfs_streq(buf, "1"))
		STK8BAXX_SetCali(stk, 1);
	else 
	{
		printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	return count;
}

static ssize_t stk8baxx_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk8baxx_data *stk = dev_get_drvdata(dev);	
	u8 offset[3] = {0};
	STK8BAXX_GetOffset(stk, offset);
	printk(KERN_INFO "%s: offset = 0x%x, 0x%x, 0x%x\n", __func__, offset[0], offset[1], offset[2]);		
	return scnprintf(buf, PAGE_SIZE, "%x %x %x\n", offset[0], offset[1], offset[2]);	
}

static ssize_t stk8baxx_offset_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct stk8baxx_data *stk = dev_get_drvdata(dev);	
	u8 offset[3] = {0};
	int error, i;
	int r_offset[3];
	char *token[10];
	
	for (i = 0; i < 3; i++)
		token[i] = strsep((char **)&buf, " ");
	if((error = strict_strtoul(token[0], 16, (unsigned long *)&(r_offset[0]))) < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, error=%d\n", __func__, error);
		return error;	
	}
	if((error = strict_strtoul(token[1], 16, (unsigned long *)&(r_offset[1]))) < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, error=%d\n", __func__, error);
		return error;	
	}
	if((error = strict_strtoul(token[2], 16, (unsigned long *)&(r_offset[2]))) < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, error=%d\n", __func__, error);
		return error;	
	}
	printk(KERN_INFO "%s: offset = 0x%x, 0x%x, 0x%x\n", __func__, r_offset[0], r_offset[1], r_offset[2]);	
	
	for(i=0;i<3;i++)
		offset[i] = (u8)r_offset[i];
	STK8BAXX_SetOffset(stk, offset);
	return count;
}

static ssize_t stk8baxx_send_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t stk8baxx_send_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct stk8baxx_data *stk = dev_get_drvdata(dev);	
	int error;	
	int addr, cmd, i;
	char *token[10];
	int result;	
	char en;
	
	for (i = 0; i < 2; i++)
		token[i] = strsep((char **)&buf, " ");
	if((error = strict_strtoul(token[0], 16, (unsigned long *)&(addr))) < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, error=%d\n", __func__, error);
		return error;	
	}
	if((error = strict_strtoul(token[1], 16, (unsigned long *)&(cmd))) < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, error=%d\n", __func__, error);
		return error;	
	}
	printk(KERN_INFO "%s: write reg 0x%x=0x%x\n", __func__, addr, cmd);	
	
	result = STK8BAXX_GetEnable(stk, &en);
	if(result < 0)
		printk(KERN_ERR "%s: STK8BAXX_GetEnable failed, error=%d\n", __func__, result);
	
	if(en == 0)
	{
		STK8BAXX_EnterActive(stk);	
		msleep(2);	/* for eng reg */
	}		
	
	error = stk8baxx_smbus_write_byte_data((u8)addr, (u8)cmd);
	if (error < 0) 
	{
		printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, (u8)addr, error);		
		return error;
	}		

	if(en == 0)
		STK8BAXX_EnterSuspend(stk);	
	return count;	
}

static ssize_t stk8baxx_recv_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk8baxx_data *stk = dev_get_drvdata(dev);		
	return scnprintf(buf, PAGE_SIZE, "0x%x\n", atomic_read(&stk->recv_reg));	
}

static ssize_t stk8baxx_recv_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct stk8baxx_data *stk = dev_get_drvdata(dev);	
	unsigned long data;
	int result;	
	char en;
	
	result = strict_strtoul(buf, 16, &data);
	if (result)
	{
		printk(KERN_ERR "%s: strict_strtoul failed, result=%d\n", __func__, result);
		return result;
	}	

	result = STK8BAXX_GetEnable(stk, &en);
	if(result < 0)
		printk(KERN_ERR "%s: STK8BAXX_GetEnable failed, error=%d\n", __func__, result);
	
	if(en == 0)
	{
		STK8BAXX_EnterActive(stk);
		msleep(2);	/* for eng reg */
	}	
	
	result = stk8baxx_smbus_read_byte_data((u8)data);
	if (result < 0) 
	{
		printk(KERN_ERR "%s: failed to read reg 0x%x, result=%d\n", __func__, (int)data, result);
		return result;
	}	
	atomic_set(&stk->recv_reg, result);
	printk(KERN_INFO "%s: reg 0x%x=0x%x\n", __func__, (unsigned int)data, result);
	
	if(en == 0)
		STK8BAXX_EnterSuspend(stk);	

	return count;
}

static ssize_t stk8baxx_firlen_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef STK_LOWPASS
	struct stk8baxx_data *stk = dev_get_drvdata(dev);	
	int len = atomic_read(&stk->firlength);
	
	if(atomic_read(&stk->firlength))
	{
		printk(KERN_INFO "len = %2d, idx = %2d\n", stk->fir.num, stk->fir.idx);			
		printk(KERN_INFO "sum = [%5d %5d %5d]\n", stk->fir.sum[0], stk->fir.sum[1], stk->fir.sum[2]);
		printk(KERN_INFO "avg = [%5d %5d %5d]\n", stk->fir.sum[0]/len, stk->fir.sum[1]/len, stk->fir.sum[2]/len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&stk->firlength));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}

static ssize_t stk8baxx_firlen_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef STK_LOWPASS
	struct stk8baxx_data *stk = dev_get_drvdata(dev);
	int error;
	unsigned long data;
	
	error = strict_strtoul(buf, 10, &data);
	if (error)
	{
		printk(KERN_ERR "%s: strict_strtoul failed, error=%d\n", __func__, error);
		return error;
	}			

	if(data > MAX_FIR_LEN)
	{
		printk(KERN_ERR "%s: firlen exceed maximum filter length\n", __func__);
	}
	else if (data < 1)
	{
		atomic_set(&stk->firlength, 1);
		atomic_set(&stk->fir_en, 0);	
		memset(&stk->fir, 0x00, sizeof(stk->fir));
	}
	else
	{ 
		atomic_set(&stk->firlength, data);
		memset(&stk->fir, 0x00, sizeof(stk->fir));
		atomic_set(&stk->fir_en, 1);	
	}
#else
	printk(KERN_ERR "%s: firlen is not supported\n", __func__);
#endif    
	return count;
}


static ssize_t stk8baxx_allreg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct stk8baxx_data *stk = dev_get_drvdata(dev);
	u8 buffer[16] = "";
	int aa,bb, no;	
	int result;
	char en;
	
	result = STK8BAXX_GetEnable(stk, &en);
	if(result < 0)
		printk(KERN_ERR "%s: STK8BAXX_GetEnable failed, error=%d\n", __func__, result);
	
	if(en == 0)
		STK8BAXX_EnterActive(stk);
	
	for(bb=0;bb<4;bb++)
	{
		result = stk8axxx_smbus_read_i2c_block_data(bb*0x10, 16, buffer);		
		if (result < 0) 
		{
			printk(KERN_ERR "%s:failed\n", __func__);
			return result;
		}
		for(aa=0;aa<16;aa++)
		{
			no = bb*0x10+aa;
			printk(KERN_INFO "stk reg[0x%x]=0x%x\n", no, buffer[aa]);
		}
	}	
	
	if(en == 0)
		STK8BAXX_EnterSuspend(stk);
	return 0;		
}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, stk8baxx_enable_show, stk8baxx_enable_store);
static DEVICE_ATTR(value, S_IRUGO, stk8baxx_value_show, NULL);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, stk8baxx_delay_show, stk8baxx_delay_store);
static DEVICE_ATTR(cali, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, stk8baxx_cali_show, stk8baxx_cali_store);
static DEVICE_ATTR(offset, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, stk8baxx_offset_show, stk8baxx_offset_store);
static DEVICE_ATTR(send, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, stk8baxx_send_show, stk8baxx_send_store);
static DEVICE_ATTR(recv, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, stk8baxx_recv_show, stk8baxx_recv_store);
static DEVICE_ATTR(firlen, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, stk8baxx_firlen_show, stk8baxx_firlen_store);
static DEVICE_ATTR(allreg, S_IRUGO, stk8baxx_allreg_show, NULL);

static struct attribute *stk8baxx_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_value.attr,
	&dev_attr_delay.attr,
	&dev_attr_cali.attr,
	&dev_attr_offset.attr,
	&dev_attr_send.attr,
	&dev_attr_recv.attr,
	&dev_attr_firlen.attr,
	&dev_attr_allreg.attr,	
	NULL
};

static struct attribute_group stk8baxx_attribute_group = {
	.name = "driver",
	.attrs = stk8baxx_attributes,
};



static int stk8baxx_open(struct inode *inode, struct file *file)
{
	int ret;
	ret = nonseekable_open(inode, file);
	if(ret < 0)
		return ret;
	file->private_data = stk8baxx_data_ptr;
	return 0;
}

static int stk8baxx_release(struct inode *inode, struct file *file)
{
	return 0;
}

#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,36))	
static long stk8baxx_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#else	
static int stk8baxx_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
	void __user *argp = (void __user *)arg;
	int retval = 0;
	char state = 0;
	char rwbuf[10] = "";
	uint32_t delay_ns;	
	char char3_buffer[3];
	int result;
	int int3_buffer[3];
	struct stk8baxx_data *stk = file->private_data;
	char en;
	
	result = STK8BAXX_GetEnable(stk, &en);
	if(result < 0)
		printk(KERN_ERR "%s: STK8BAXX_GetEnable failed, error=%d\n", __func__, result);
	
	if(en == 0 && cmd != STK_IOCTL_SET_ENABLE)
		STK8BAXX_EnterActive(stk);
	
	switch (cmd) 
	{
	case STK_IOCTL_SET_OFFSET:	
		if(copy_from_user(&char3_buffer, argp, sizeof(char3_buffer)))
		{
			retval = -EFAULT;							
			goto ioctl_err;
		}
		break;
	case STK_IOCTL_SET_DELAY:	
		if(copy_from_user(&delay_ns, argp, sizeof(uint32_t)))
		{
			retval = -EFAULT;							
			goto ioctl_err;
		}		
		break;
	case STK_IOCTL_WRITE:	
	case STK_IOCTL_READ:
		if (copy_from_user(&char3_buffer, argp, sizeof(char3_buffer)))
		{
			retval = -EFAULT;							
			goto ioctl_err;
		}
		break;	
	case STK_IOCTL_SET_ENABLE:
	case STK_IOCTL_SET_RANGE:
	case STK_IOCTL_SET_CALI:
		if(copy_from_user(&state, argp, sizeof(char)))
		{
			retval = -EFAULT;							
			goto ioctl_err;
		}
		break;		
	default:
		break;
	}
	
	switch (cmd) 
	{
	case STK_IOCTL_WRITE:
		if (rwbuf[0] < 2)
		{
			retval = -EINVAL;							
			goto ioctl_err;
		}		
		result = stk8baxx_smbus_write_byte_data(rwbuf[1], rwbuf[2]);	/*force write one byte*/
		if (result < 0) 
		{
			printk(KERN_ERR "%s:failed to write reg 0x%x, error=%d\n", __func__, rwbuf[1], result);
			retval = result;							
			goto ioctl_err;
		}			
		break;	
	case STK_IOCTL_SET_OFFSET:					
		STK8BAXX_SetOffset(stk, char3_buffer);
		break;
	case STK_IOCTL_SET_DELAY:	
		STK8BAXX_SetDelay(stk, delay_ns);
		break;		
	case STK_IOCTL_READ:
		if (char3_buffer[0] < 1 || char3_buffer[0] > 10)
		{
			retval = -EINVAL;							
			goto ioctl_err;
		}		
		rwbuf[0] = char3_buffer[0];
		result = stk8axxx_smbus_read_i2c_block_data(char3_buffer[1], char3_buffer[0], &rwbuf[1]);
		if (result < 0) 
		{
			retval = result;							
			goto ioctl_err;
		}		
		break;	
	case STK_IOCTL_GET_DELAY:
		delay_ns = STK8BAXX_GetDelay(stk);
		break;
	case STK_IOCTL_GET_OFFSET:
		STK8BAXX_GetOffset(stk, char3_buffer);
		break;
	case STK_IOCTL_GET_ACCELERATION:		
		STK8BAXX_ReadSensorData(stk);
		mutex_lock(&stk->value_lock);
		int3_buffer[0] = stk->acc_xyz.x;
		int3_buffer[1] = stk->acc_xyz.y;
		int3_buffer[2] = stk->acc_xyz.z;			
		mutex_unlock(&stk->value_lock);		
		break;
	case STK_IOCTL_SET_ENABLE:
		STK8BAXX_SetEnable(stk, state);
			break;
	case STK_IOCTL_GET_ENABLE:
		STK8BAXX_GetEnable(stk, &state);
		break;
	case STK_IOCTL_SET_RANGE:
		STK8BAXX_SetRange(stk, state);
		break;
	case STK_IOCTL_GET_RANGE:
		STK8BAXX_GetRange(stk, &state);
		break;
	case STK_IOCTL_SET_CALI:
		STK8BAXX_SetCali(stk, state);
		break;		
	default:
		retval = -ENOTTY;
		break;
	}	
	if(en == 0 && cmd != STK_IOCTL_SET_ENABLE)
		STK8BAXX_EnterSuspend(stk);
	
	switch (cmd) 
	{
	case STK_IOCTL_GET_ACCELERATION:		
		if(copy_to_user(argp, &int3_buffer, sizeof(int3_buffer)))
			return -EFAULT;			
		break;	
	case STK_IOCTL_READ:
		if(copy_to_user(argp, &rwbuf, sizeof(rwbuf)))
			return -EFAULT;			
		break;	
	case STK_IOCTL_GET_DELAY:
		if(copy_to_user(argp, &delay_ns, sizeof(delay_ns)))
			return -EFAULT;			
		break;			
	case STK_IOCTL_GET_OFFSET:
		if(copy_to_user(argp, &char3_buffer, sizeof(char3_buffer)))
			return -EFAULT;		
		break;		
	case STK_IOCTL_GET_RANGE:		
	case STK_IOCTL_GET_ENABLE:
		if(copy_to_user(argp, &state, sizeof(char)))
			return -EFAULT;		
		break;
	default:
		break;
	}		

	
	return retval;

ioctl_err:	
	if(en == 0)
		STK8BAXX_EnterSuspend(stk);	
	return retval;
}


static struct file_operations stk_fops = {
	.owner = THIS_MODULE,
	.open = stk8baxx_open,
	.release = stk8baxx_release,
#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,36))	
	.unlocked_ioctl = stk8baxx_ioctl,
#else	
	.ioctl = stk8baxx_ioctl,
#endif
};

static struct miscdevice stk_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "stk8baxx",
	.fops = &stk_fops,
};


static void stk_mems_wq_function(struct work_struct *work)
{
	struct stk8baxx_data *stk = container_of(work, struct stk8baxx_data, stk_work);		
	pr_info("%s\n",__func__);
        STK8BAXX_ReadSensorData(stk);
	STK8BAXX_ReportValue(stk);	
#if (!STK_ACC_POLLING_MODE)
	enable_irq(stk->irq);
#endif	
}


#if STK_ACC_POLLING_MODE
static enum hrtimer_restart stk_acc_timer_func(struct hrtimer *timer)
{
	struct stk8baxx_data *stk = container_of(timer, struct stk8baxx_data, acc_timer);
	queue_work(stk->stk_mems_work_queue, &stk->stk_work);
	hrtimer_forward_now(&stk->acc_timer, stk->poll_delay);
	return HRTIMER_RESTART;	
}

#else

static irqreturn_t stk_mems_irq_fun(int irq, void *data)
{
	struct stk8baxx_data *stk = data;	
	disable_irq_nosync(stk->irq);
	queue_work(stk->stk_mems_work_queue,&stk->stk_work);
	return IRQ_HANDLED;
}


static int stk8baxx_irq_setup(struct i2c_client *client, struct stk8baxx_data *stk_dat, int int_pin)
{
	int error;
	int irq= -1;	
#if ADDITIONAL_GPIO_CFG 
	if (gpio_request(int_pin, "EINT"))
	{
		printk(KERN_ERR "%s:gpio_request() failed\n",__func__);
		return -1;
	}
	gpio_direction_input(int_pin);
	
	irq = gpio_to_irq(int_pin);
	if ( irq < 0 )
	{
		printk(KERN_ERR "%s:gpio_to_irq() failed\n",__func__);		
		return -1;
	}
	client->irq = irq;
	stk_dat->irq = irq;	
#endif //#if ADDITIONAL_GPIO_CFG 
#ifdef STK_DEBUG_PRINT	
	printk(KERN_INFO "%s: irq # = %d\n", __func__, irq);
#endif	

	error = request_any_context_irq(client->irq, stk_mems_irq_fun, IRQF_TRIGGER_RISING, "stk8baxx", stk_dat);	
	if (error < 0) 
	{
		printk(KERN_ERR "%s: request_threaded_irq(%d) failed for (%d)\n", __func__, client->irq, error);
		return -1;
	}	
	
	disable_irq(irq);	
	return irq;	
}

#endif	//#if STK_ACC_POLLING_MODE


static int stk8baxx_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int error;
	struct stk8baxx_data *stk;
	struct stk8baxx_platform_data *stk_platdata;
#ifdef IS_STK8BA22
	printk(KERN_INFO "stk8ba22_probe: driver version:%s\n",STK_ACC_DRIVER_VERSION);	
#elif defined IS_STK8BA50	
	printk(KERN_INFO "stk8ba50_probe: driver version:%s\n",STK_ACC_DRIVER_VERSION);	
#endif
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_I2C_BLOCK)) 
	{
		error = i2c_get_functionality(client->adapter);
		printk(KERN_ERR "%s:i2c_check_functionality error, functionality=0x%x\n", __func__, error);
		error = -ENODEV;
		goto exit_i2c_check_functionality_error;
	}	
	
	stk = kzalloc(sizeof(struct stk8baxx_data),GFP_KERNEL);
	if (!stk) 
	{	
		printk(KERN_ERR "%s:memory allocation error\n", __func__);
		error = -ENOMEM;
		goto exit_kzalloc_error;
	}
	if(client->dev.platform_data != NULL)		
	{
		stk_platdata = client->dev.platform_data;
		stk->int_pin = stk_platdata->interrupt_pin;
		stk->stk8baxx_placement = stk_platdata->direction;
	}
	else
	{
                stk_platdata = &p_stk8baxx_platdata;
                client->dev.platform_data = stk_platdata;
                stk->int_pin = stk_platdata->interrupt_pin;
                stk->stk8baxx_placement = stk_platdata->direction;
                  
		printk(KERN_ERR "%s: set stk8baxx platform data to p_stk8baxx_platdata !\n", __func__);

		
                //error = -EINVAL;
		//goto exit_no_platform_data;
	}		
	
	stk8baxx_data_ptr = stk;
	mutex_init(&stk->value_lock);
	mutex_init(&stk->write_lock);
	
	stk->stk_mems_work_queue = create_singlethread_workqueue("stk_mems_wq");
	if(stk->stk_mems_work_queue)
		INIT_WORK(&stk->stk_work, stk_mems_wq_function);
	else
	{
		printk(KERN_ERR "%s:create_singlethread_workqueue error\n", __func__);
		error = -EPERM;
		goto exit_create_workqueue_error;
	}		
#if (!STK_ACC_POLLING_MODE)	
	error = stk8baxx_irq_setup(client, stk, stk_platdata->interrupt_pin);
	if(!error)
	{
		goto exit_irq_setup_error;
	}
#else
	hrtimer_init(&stk->acc_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stk->poll_delay = ns_to_ktime(STK8BAXX_SAMPLE_TIME[STK8BAXX_INIT_ODR-STK8BAXX_SPTIME_BASE] * 1000);
	stk->acc_timer.function = stk_acc_timer_func;	
#endif	//#if STK_ACC_POLLING_MODE	
	stk->client = client;
	i2c_set_clientdata(client, stk);	
	error = STK8BAXX_Init(stk, client);
	if (error) 
	{		
		printk(KERN_ERR "%s:stk8baxx initialization failed\n", __func__);	
		goto exit_stk_init_error;
	}
	stk->re_enable = false;
	stk->input_dev = input_allocate_device();
	if (!stk->input_dev) 
	{
		error = -ENOMEM;
		printk(KERN_ERR "%s:input_allocate_device failed\n", __func__);
		goto exit_input_dev_alloc_error;
	}	
	stk->input_dev->name = ACC_IDEVICE_NAME;
	set_bit(EV_ABS, stk->input_dev->evbit);	
#ifdef IS_STK8BA50		
	input_set_abs_params(stk->input_dev, ABS_X, -512, 511, 0, 0);
	input_set_abs_params(stk->input_dev, ABS_Y, -512, 511, 0, 0);
	input_set_abs_params(stk->input_dev, ABS_Z, -512, 511, 0, 0);	
#elif defined(IS_STK8BA22)	
	input_set_abs_params(stk->input_dev, ABS_X, -128, 127, 0, 0);
	input_set_abs_params(stk->input_dev, ABS_Y, -128, 127, 0, 0);
	input_set_abs_params(stk->input_dev, ABS_Z, -128, 127, 0, 0);	
#endif
	error = input_register_device(stk->input_dev);
	if (error) 
	{
		printk(KERN_ERR "%s:Unable to register input device: %s\n", __func__, stk->input_dev->name);					 
		goto exit_input_register_device_error;
	}
	input_set_drvdata(stk->input_dev, stk);
	error = sysfs_create_group(&stk->input_dev->dev.kobj, &stk8baxx_attribute_group);
	if (error) 
	{
		printk(KERN_ERR "%s: sysfs_create_group failed\n", __func__);
		goto exit_sysfs_create_group_error;
	}	
	
	error = misc_register(&stk_device);
	if (error) 
	{
		printk(KERN_ERR "%s: misc_register failed\n", __func__);
		goto exit_misc_device_register_error;
	}		
	
#ifdef STK_DEBUG_PRINT	
	printk(KERN_INFO "%s successfully\n", __func__);
#endif
		
	return 0;
	
exit_misc_device_register_error:
	sysfs_remove_group(&stk->input_dev->dev.kobj, &stk8baxx_attribute_group);
exit_sysfs_create_group_error:	
	input_unregister_device(stk->input_dev);	
exit_input_register_device_error:	
exit_input_dev_alloc_error:	
exit_stk_init_error:	
#if (!STK_ACC_POLLING_MODE)
	free_irq(client->irq, stk);
#if ADDITIONAL_GPIO_CFG 
exit_irq_setup_error:
	gpio_free(stk->int_pin);	
	cancel_work_sync(&stk->stk_work);		
#endif 	//#if ADDITIONAL_GPIO_CFG 
#else
	hrtimer_try_to_cancel(&stk->acc_timer);
#endif 	//#if (!STK_ACC_POLLING_MODE)
	destroy_workqueue(stk->stk_mems_work_queue);		
exit_create_workqueue_error:
	mutex_destroy(&stk->value_lock);
	mutex_destroy(&stk->write_lock);
exit_no_platform_data:	
	kfree(stk);	
	stk = NULL;	
exit_kzalloc_error:	
exit_i2c_check_functionality_error:	
	return error;
}

static int stk8baxx_remove(struct i2c_client *client)
{
	struct stk8baxx_data *stk = i2c_get_clientdata(client);
	
	misc_deregister(&stk_device);
	sysfs_remove_group(&stk->input_dev->dev.kobj, &stk8baxx_attribute_group);	
	input_unregister_device(stk->input_dev);
#if (!STK_ACC_POLLING_MODE)	
	free_irq(client->irq, stk);
#if ADDITIONAL_GPIO_CFG
	gpio_free(stk->int_pin);
#endif //#if ADDITIONAL_GPIO_CFG 
#else
	hrtimer_try_to_cancel(&stk->acc_timer);
#endif	
	cancel_work_sync(&stk->stk_work);	
	if(stk->stk_mems_work_queue)
		destroy_workqueue(stk->stk_mems_work_queue);		
	mutex_destroy(&stk->value_lock);
	mutex_destroy(&stk->write_lock);	
	kfree(stk);
	stk = NULL;		
	return 0;
}

static const struct i2c_device_id stk8baxx[] = {
	{ STK8BAXX_I2C_NAME, 0 },
	{ }
};

#ifdef CONFIG_PM_SLEEP
static int stk8baxx_suspend(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct stk8baxx_data *stk = i2c_get_clientdata(client);
	
	if(atomic_read(&stk->enabled))
	{
		STK8BAXX_SetEnable(stk, 0);
		stk->re_enable = true;			
	}	
	return 0;
}


static int stk8baxx_resume(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct stk8baxx_data *stk = i2c_get_clientdata(client);
#ifdef STK_RESUME_RE_INIT
	int error;

	if(atomic_read(&stk->enabled))
		stk->re_enable = true;	
	
	error = STK8BAXX_Init(stk, client);
	if (error) 
	{		
		printk(KERN_ERR "%s:stk8baxx initialization failed\n", __func__);			
	}	
	stk->first_enable = true;	
#endif
	
	if(stk->re_enable)
	{
		stk->re_enable = false;
		STK8BAXX_SetEnable(stk, 1);
	}	
	return 0;		
}

static const struct dev_pm_ops stk8baxx_pm_ops = {
	.suspend = stk8baxx_suspend,
	.resume = stk8baxx_resume,
};
	
#endif /* CONFIG_PM_SLEEP */



static struct i2c_driver stk8baxx_driver = {
#ifdef ALLWINNER_PLATFORM
	.class = I2C_CLASS_HWMON,
#endif
	.probe = stk8baxx_probe,
	.remove = stk8baxx_remove,
	.id_table	= stk8baxx,
	.driver = {
		   .name = STK8BAXX_I2C_NAME,
#ifdef CONFIG_PM_SLEEP		   
		   .pm = &stk8baxx_pm_ops,
#endif		   
	},
#ifdef ALLWINNER_PLATFORM
	.address_list	= u_i2c_addr.normal_i2c,
#endif
};


static int __init stk8baxx_init(void)
{
	int ret = -1;
#ifdef ALLWINNER_PLATFORM
	//bma_dbg("STK8BAXX: init\n");
	if(gsensor_fetch_sysconfig_para()){
		printk("%s: err.\n", __func__);
		return -1;
	}

	printk("%s: after fetch_sysconfig_para:  normal_i2c: 0x%hx. normal_i2c[1]: 0x%hx \n", \
	__func__, u_i2c_addr.normal_i2c[0], u_i2c_addr.normal_i2c[1]);

    stk8baxx_driver.detect = gsensor_detect;
#endif
        pr_info("%s\n",__func__);
	ret = i2c_add_driver(&stk8baxx_driver);
#ifdef STK8BAXX_PERMISSION_THREAD
	STKPermissionThread = kthread_run(stk8baxx_permis_thread,"stk","Permissionthread");
	if(IS_ERR(STKPermissionThread))
	STKPermissionThread = NULL;
#endif // STK8BAXX_PERMISSION_THREAD		
	return ret;
}

static void __exit stk8baxx_exit(void)
{
	i2c_del_driver(&stk8baxx_driver);
#ifdef STK8BAXX_PERMISSION_THREAD
	if(STKPermissionThread)
	STKPermissionThread = NULL;
#endif // STK8BAXX_PERMISSION_THREAD		
}

module_init(stk8baxx_init);
module_exit(stk8baxx_exit);

MODULE_AUTHOR("Lex Hsieh, sensortek");
MODULE_DESCRIPTION("stk8baxx 3-Axis accelerometer driver");
MODULE_LICENSE("GPL");	
MODULE_VERSION(STK_ACC_DRIVER_VERSION);
