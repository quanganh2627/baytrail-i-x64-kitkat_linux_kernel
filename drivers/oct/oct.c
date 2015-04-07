/*
* Copyright (C) 2013 Intel Mobile Communications GmbH
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
#include <linux/kernel.h>		/* printk() */
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/cdev.h>			/* character device */
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/workqueue.h>	/*for events*/
#include <linux/delay.h>		/* for msleep*/
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/device.h>		/*for class & device */
#include <linux/slab.h>			/*for kmalloc, kfree */
#include <linux/interrupt.h>		/*for irq */
#include <linux/delay.h>		/*for events */
#include <linux/io.h>			/*for ioremap */
#include <asm/termios.h>		/* for file attribute */
#include <linux/dma-mapping.h>		/* for DMA alloc */
#include <linux/poll.h>
#include <linux/device_pm_data.h>
#include <linux/platform_device.h>	/*register platform for PM callbacks */
#include <linux/pm_runtime.h>		/*PM runtime*/
#include <linux/of.h>

#include <linux/syscalls.h>

#include "oct_io.h"
#include "oct.h"
#include <linux/tadi.h>

/* Constants */
#define OCT_MODULE_NAME "oct"
#define PLATFORM_DRIVER_NAME "oct_drv"

#define OCT_DRIVER_VER 5

#ifndef OCT_HOST_TEST
	#define USB_CH_NAME "/dev/ttyGS1"
	#define OCT_INT oct_int
#else
	#define USB_CH_NAME "/dev/ttyS0"
	#define OCT_INT 1 /* keyboard int */
#endif

#define OCT_EV_UNKNOWN 0  /* if the reason for event is unknown*/
#define OCT_EV_RINGBUFF_TIMEOUT_WITH_DATA    1
#define OCT_EV_RINGBUFF_FILL_LEVEL_1_REACHED 2
#define OCT_EV_RINGBUFF_FILL_LEVEL_2_REACHED 4

/* OCT hw constants */
#define TRACE_DEBUG_OCT_TIMEOUT  0x80
#define OCT_TRIG_CYCLE_SLEEP   (5 * 1000 * 1000 * 26) /* 5s */
#define OCT_TRIG_LEVEL_2 ((OCT_EXT_RING_BUFF_SIZE / 4) * 3)
#define OCT_REG_ADDRESS_BASE oct_base
#define OCT_REG_SIZE sizeof(struct _soct_trform)

/* useful macros */
#define OCT_DBG(fmt, arg...) {\
	if (debug_on) \
		pr_info(OCT_MODULE_NAME": " fmt"\n", ##arg); }

#define OCT_LOG(fmt, arg...) \
	pr_info(OCT_MODULE_NAME": " fmt"\n", ##arg)

#define OCT_CLR_OCT_ON	SET_OCT_OCT_CNF_CLR_OCTM(oct_trform, 1)
#define OCT_CLR_OCT_OFF	SET_OCT_OCT_CNF_CLR_OCTM(oct_trform, 0)

/* Global variables */
void * __iomem oct_base;
unsigned int oct_int;
static int major;
static int minor;
static struct class *class_oct;
static struct file *fp;
loff_t pos = 0;

static struct task_struct *task;
static wait_queue_head_t oct_wq, oct_poll_wq;
static dma_addr_t phy_base_addr;

static int oct_ext_ring_buff_len = OCT_EXT_RING_BUFF_SIZE;
static unsigned int oct_mode = DEFAULT_OCT_MODE;
static unsigned int current_timeout = DEFAULT_OCT_TRIG_CYCLE;
static unsigned int oct_trig_level1 = DEFAULT_OCT_TRIG_LEVEL;
static unsigned int oct_out_path = DEFAULT_OCT_PATH;
static unsigned int debug_on;

#define oct_trform oct_base
static void *oct_ext_rbuff_ptr;
static unsigned int oct_ext_mem_full;
static unsigned int oct_irq;
static char oct_events;
static unsigned int oct_read_ptr;
static unsigned int data_available;
struct device_pm_platdata *pm_platdata;

// offline bplog definition
#define OFFLOG_PATH_LEN 255

#define OFFLOG_CONFIG_FILE "/system/etc/offlogcfg.txt"
#define OFFLOG_CONFIG_FILE_SIZE 500

#define OFFLOG_CONFIG_TAG_ONOFF "on_off"
#define OFFLOG_CONFIG_TAG_PATH  "offlog_path"
#define OFFLOG_CONFIG_TAG_SIZE  "offlog_size"
#define OFFLOG_CONFIG_TAG_NUM   "offlog_number"

#define OFFLOG_CONFIG_MAX_VALUE_LENGTH 50

//#define DEFAULT_OCT_OFFLOG_SIZE 500000000
#define DEFAULT_OCT_OFFLOG_SIZE 10000000
#define DEFAULT_OCT_OFFLOG_NUMBER 5
#define DEFAULT_OFFLOG_PATH "/data/logs/"
#define DEFAULT_OFFLOG_NAME "bplog"
static char oct_offlog_path[OFFLOG_PATH_LEN+1] = {0}; // /data/logs/bplog
static unsigned int oct_offlog_onoff = 1;
static unsigned int oct_offlog_size = DEFAULT_OCT_OFFLOG_SIZE;
static unsigned int oct_offlog_number = DEFAULT_OCT_OFFLOG_NUMBER;
/* power management section */

static int pm_suspend_fct(struct device *dev)
{
	int irq;
	/* steps to move subsystem to system suspend safe state */
	OCT_DBG("suspend");

	/* adjust sleep TIMEOUT */
	SET_OCT_OCT_MASTER_TRIG_CYCLE_DMA_TRIG_CYCLE(oct_trform,
							OCT_TRIG_CYCLE_SLEEP);

	irq = GET_OCT_OCT_MASTER_RXIRQ_STAT(oct_trform);
	/* reset the interrupts */
	SET_OCT_OCT_MASTER_RXIRQ_CON(oct_trform, irq);
	SET_OCT_OCT_MASTER_RXIRQ_CON(oct_trform, 0);
	/* trace_debug_oct_dma_off(); */
	return 0;
}

static int pm_resume_fct(struct device *dev)
{
	/* steps to resume from suspend */
	OCT_DBG("resume");
	/* reset & enable CYCLE interrupt */
	SET_OCT_OCT_MASTER_TRIG_CYCLE_DMA_TRIG_CYCLE(oct_trform,
							current_timeout);
	return 0;
}

static int pm_runtime_suspend_fct(struct device *dev)
{
	OCT_DBG("Runtime suspend");
	/* steps to move subsystem to runtime suspend safe state */
	return 0;
}

static int pm_runtime_resume_fct(struct device *dev)
{
	/* steps to resume from suspend */
	OCT_DBG("Runtime resume");
	return 0;
}

static int pm_runtime_idle_fct(struct device *dev)
{
	/* This function is the entry point when runtime counter is 0
	 * This function must check if the condition(s)
	 * for suspend is/are fulfiled.
	 * Before allowing it to enter suspend
	 * This function will indirect reach runtime_suspend
	*/
	OCT_DBG("Runtime Idle");
	/* If abc conditions are met.. */
	pm_runtime_suspend(dev);
	return 0;
}

static const struct dev_pm_ops my_pm_ops = {
	/* For system suspend/resume */
	.suspend = pm_suspend_fct,
	.resume = pm_resume_fct,
	/* For runtime PM */
	.runtime_suspend = pm_runtime_suspend_fct,
	.runtime_resume = pm_runtime_resume_fct,
	.runtime_idle = pm_runtime_idle_fct,
};

/* Tasklets for BH */
static void oct_tasklet_bh(unsigned long data);
DECLARE_TASKLET(oct_tasklet, oct_tasklet_bh, 0);

static int  __init oct_init(void);
static void __exit oct_exit(void);

irq_handler_t oct_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	/* clear OCT external buffer interrupt for LEVEL1, LEVEL2 and CYCLE*/
	irq = GET_OCT_OCT_MASTER_RXIRQ_STAT(oct_trform);

	/* disable timer interrupt, will be re-enabled if ext mem is empty */
	SET_OCT_OCT_CNF_ENABLE_TRIG_CYCLE_INT(oct_trform,
			OCT_CNF_ENABLE_TRIG_CYCLE_INT_DISABLED);
	SET_OCT_OCT_CNF_ENABLE_TRIG_LEVEL1_INT(oct_trform,
			OCT_CNF_ENABLE_TRIG_LEVEL1_INT_DISABLED);
	SET_OCT_OCT_CNF_ENABLE_TRIG_LEVEL2_INT(oct_trform,
			OCT_CNF_ENABLE_TRIG_LEVEL2_INT_DISABLED);
	/* SET_OCT_OCT_CNF_ENABLE_TRIG_CYCLE_INT(oct_trform,
				  OCT_CNF_ENABLE_TRIG_CYCLE_INT_DISABLED); */

	oct_irq = OCT_EV_UNKNOWN;

	if (irq & 1<<OCT_MASTER_RXIRQ_STAT_LSB_IRQ_CYCLE)
		oct_irq |= OCT_EV_RINGBUFF_TIMEOUT_WITH_DATA;
	if (irq & 1<<OCT_MASTER_RXIRQ_STAT_LSB_IRQ_LEVEL1)
		oct_irq |= OCT_EV_RINGBUFF_FILL_LEVEL_1_REACHED;
	if (irq & 1<<OCT_MASTER_RXIRQ_STAT_LSB_IRQ_LEVEL2)
		oct_irq |= OCT_EV_RINGBUFF_FILL_LEVEL_2_REACHED;

	/* reset the interrupts */
	SET_OCT_OCT_MASTER_RXIRQ_CON(oct_trform, irq);
	SET_OCT_OCT_MASTER_RXIRQ_CON(oct_trform, 0);

	/* activate bottom half task */
	tasklet_schedule(&oct_tasklet);
	OCT_DBG("IRQ: %d", oct_irq);

	return (irq_handler_t)IRQ_HANDLED;
}

static void oct_clr_bit(tcflag_t *p_flags, tcflag_t mask)
{
	OCT_DBG("CLR Bit 0x%07o", mask);
	OCT_DBG("---- Before 0x%07o", *p_flags);
	*p_flags &= ~mask;
	OCT_DBG("---- After  0x%07o", *p_flags);
}

static void oct_set_bit(tcflag_t *p_flags, tcflag_t mask)
{
	OCT_DBG("SET Bit 0x%07o", mask);
	OCT_DBG("---- Before 0x%07o", *p_flags);
	*p_flags |= mask;
	OCT_DBG("---- After  0x%07o", *p_flags);
}

static void oct_set_ccc(struct ktermios *attr,
					 unsigned int item, int val)
{
	if (item < NCCS) {
		OCT_DBG("Change c_cc value [%d]", item);
		OCT_DBG("---- Before %d", attr->c_cc[item]);
		attr->c_cc[item] = val;
		OCT_DBG("---- After  %d", attr->c_cc[item]);
	} else {
		OCT_DBG("Bad Item [%d] for Change C-CC", item);
	}
}

void oct_set_raw_gadget(struct file *fp)
{
	struct ktermios my_attr;

	/* Read the current set of terminal attribute flags & dump the result */
	fp->f_op->unlocked_ioctl(fp, TCGETS, (unsigned long) &my_attr);

	/* Modify the attributes to be what we want instead of the default */
	oct_clr_bit(&my_attr.c_iflag, IGNBRK);
	oct_clr_bit(&my_attr.c_iflag, BRKINT);
	oct_clr_bit(&my_attr.c_iflag, PARMRK);
	oct_clr_bit(&my_attr.c_iflag, ISTRIP);
	oct_clr_bit(&my_attr.c_iflag, INLCR);
	oct_clr_bit(&my_attr.c_iflag, IGNCR);
	oct_clr_bit(&my_attr.c_iflag, ICRNL);
	oct_clr_bit(&my_attr.c_iflag, IXON);

	oct_clr_bit(&my_attr.c_oflag, OPOST);

	oct_clr_bit(&my_attr.c_lflag, ECHO);
	oct_clr_bit(&my_attr.c_lflag, ECHONL);
	oct_clr_bit(&my_attr.c_lflag, ICANON);
	oct_clr_bit(&my_attr.c_lflag, ISIG);
	oct_clr_bit(&my_attr.c_lflag, IEXTEN);

	oct_clr_bit(&my_attr.c_cflag, CSIZE);
	oct_clr_bit(&my_attr.c_cflag, PARENB);
	oct_set_bit(&my_attr.c_cflag, CS8);

	oct_set_ccc(&my_attr, VTIME, 0);
	oct_set_ccc(&my_attr, VMIN,  1);

	fp->f_op->unlocked_ioctl(fp, TCSETS, (unsigned long)&my_attr);
}

static void oct_detect_off_flag(char *gadget_name)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	int retry = 5;
	OCT_DBG("oct_detect_off_flag[%s]", gadget_name);

	while (!kthread_should_stop()&&retry>=0) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		{
			fp = filp_open(gadget_name, O_RDONLY, 0);
		}
		if (NULL == fp) {
			OCT_LOG("NULL Gadget File Handle");
			msleep(2000);
			continue;
		} else if (IS_ERR(fp)) {
			OCT_LOG("Gadget fp=%x open failed!", (unsigned int)fp);
			retry--;
			msleep(2000);
			continue;
		} else {
			OCT_LOG("Gadget %s open succeeded. fp=%x", gadget_name,(unsigned int)fp);
			oct_out_path = OCT_PATH_FILE;
            filp_close(fp, NULL);
			break;
		}

		set_fs(old_fs);
		set_current_state(TASK_INTERRUPTIBLE);
	}
	return;
}

static struct file *oct_open_gadget(char *gadget_name)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	OCT_DBG("Opening Gadget[%s]", gadget_name);

	while (!kthread_should_stop()) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		{
			if (oct_out_path == OCT_PATH_TTY)
				fp = filp_open(gadget_name, O_RDWR, 0);
			else if (oct_out_path == OCT_PATH_FILE)		
				fp = filp_open(gadget_name, O_RDWR | O_CREAT | O_TRUNC, 0666);	
		}
		set_fs(old_fs);
		if (NULL == fp) {
			OCT_LOG("NULL Gadget File Handle");
			msleep(2000);
			continue;
		} else if (IS_ERR(fp)) {
			OCT_LOG("Gadget %s open failed.", gadget_name);
			msleep(2000);
			continue;
		} else {
			OCT_LOG("Gadget %s open succeeded. fp=%x", gadget_name,(unsigned int)fp);
			if (oct_out_path == OCT_PATH_TTY)
				oct_set_raw_gadget(fp);
			break;
		}
		set_current_state(TASK_INTERRUPTIBLE);
	}
	return fp;
}

static void oct_mode_off(void)
{
	/* flush remaining data from internal OCT memory*/
	SET_OCT_OCT_CNF_OCTM_TIMEOUT(oct_trform,
					     TRACE_DEBUG_OCT_TIMEOUT);
	OCT_CLR_OCT_ON;
	/* flush DMA machine */
	SET_OCT_OCT_CNF_FRAME_TIMEOUT(oct_trform, 1);
	udelay(1);	 /* wait for 1 ms that remaining data can be flushed */
	SET_OCT_OCT_MASTER_RXCON_FLUSH(oct_trform, 1);
	/* stop writing to external memory */
	SET_OCT_OCT_CNF_ENABLE_MANU_STALL(oct_trform,
				  OCT_CNF_ENABLE_MANU_STALL_ENABLED);
	/* disable Arm11 interrupt for OCT HW block */
	free_irq(OCT_INT, (void *)(oct_irq_handler));

	/* set read pointer to write pointer to clear memory buffer */
	SET_OCT_OCT_SW_RPTR_SW_RPTR(oct_trform,
		GET_OCT_OCT_MASTER_WPTR_MASTER_WPTR(oct_trform));
	/* reset TIMEOUT setting, clear DMA channel and disable channel */
	SET_OCT_OCT_MASTER_RXCON_TIMEOUT(oct_trform, 0);
	SET_OCT_OCT_MASTER_RXCON_CLR_CH(oct_trform,
				    OCT_MASTER_RXCON_CLR_CH_CLEAR);
	SET_OCT_OCT_MASTER_RXCON_EN_CH(oct_trform, 0);
	SET_OCT_CLC(oct_trform, 0x03);
}

static void trace_debug_oct_define_ringbuff(void *ring_buff_start,
		unsigned int ring_buff_size)
{
	/* if '-1' the set our default values */
	if (ring_buff_start != (void *)-1)
		oct_ext_rbuff_ptr = ring_buff_start;
	if (ring_buff_size != -1)
		oct_ext_ring_buff_len = ring_buff_size;

	oct_ext_rbuff_ptr = kzalloc(OCT_EXT_RING_BUFF_SIZE, GFP_KERNEL);
	phy_base_addr = dma_map_single(NULL, oct_ext_rbuff_ptr,
			OCT_EXT_RING_BUFF_SIZE, DMA_FROM_DEVICE);

	OCT_DBG("DMA addr:%#x; KERN Addr:%p", (unsigned int)phy_base_addr,
			oct_ext_rbuff_ptr);

	/* if NULL ptr or zero size defined -> Error
	 * or bad alignment */
	if (!(oct_ext_rbuff_ptr && oct_ext_ring_buff_len) ||
		((uintptr_t)oct_ext_rbuff_ptr & 15) ||
		(oct_ext_ring_buff_len & 15) ||
		(oct_ext_ring_buff_len < 0x4000) ||
		(oct_ext_ring_buff_len > 0x04000000))
		OCT_DBG("Define Ringbuff: Fatal error!");

	/* set pointer and size now in hardware */
	SET_OCT_OCT_MASTER_RXCH0_BASE_BASE(oct_trform,
			(unsigned int)(phy_base_addr >> 4));
	SET_OCT_OCT_MASTER_RXCH0_SIZE_SIZE(oct_trform,
			oct_ext_ring_buff_len >> 2);

	/* reset old write ptr and read ptr */
	SET_OCT_OCT_SW_RPTR_RST(oct_trform, OCT_SW_RPTR_RST_RESET);
	SET_OCT_OCT_SW_RPTR_RST(oct_trform, OCT_SW_RPTR_RST_NORMAL);
	return;
}

static int oct_check_power(void)
{
	SET_OCT_CLC(oct_trform, 0x100);
	if (GET_OCT_CLC(oct_trform) == 0x100)
		return 1;
	else
		return 0;
}

static void oct_hw_init(void)
{
	SET_OCT_CLC(oct_trform, 0x100);
	SET_OCT_OCT_CNF2_TRFORM_STOP(oct_trform,
			OCT_CNF2_TRFORM_STOP_DISABLED);
	SET_OCT_OCT_MASTER_RXCON_EN_CH(oct_trform, 0);
	SET_OCT_OCT_CNF_OCTM_TIMEOUT(oct_trform,
			TRACE_DEBUG_OCT_TIMEOUT);
	SET_OCT_OCT_CNF_FRAME_TIMEOUT(oct_trform, 1);

	/* flush / clear */
	SET_OCT_OCT_CNF_ENABLE_MANU_STALL(oct_trform,
			OCT_CNF_ENABLE_MANU_STALL_ENABLED);
	SET_OCT_OCT_CNF_ENABLE_OCTM_RINGBF(oct_trform,
			OCT_CNF_ENABLE_OCTM_RINGBF_DISABLED);
	if (oct_mode == OCT_MODE_OW)
		SET_OCT_OCT_CNF_ENABLE_EXTM_RINGBF(oct_trform,
				OCT_CNF_ENABLE_EXTM_RINGBF_ENABLED);
	else if (oct_mode == OCT_MODE_STALL)
		SET_OCT_OCT_CNF_ENABLE_EXTM_RINGBF(oct_trform,
				OCT_CNF_ENABLE_EXTM_RINGBF_DISABLED);

	OCT_CLR_OCT_ON;
	SET_OCT_OCT_MASTER_RXCON_CLR_CH(oct_trform,
			OCT_MASTER_RXCON_CLR_CH_CLEAR);
	SET_OCT_OCT_MASTER_RXCON_FLUSH(oct_trform, 1);
	SET_OCT_OCT_MASTER_RXCON_FLUSH(oct_trform, 0);
	SET_OCT_OCT_MASTER_RXCON_CLR_CH(oct_trform,
			OCT_MASTER_RXCON_CLR_CH_DEFAULT);
	OCT_CLR_OCT_OFF;

	SET_OCT_OCT_CNF_ENABLE_MANU_STALL(oct_trform,
			OCT_CNF_ENABLE_MANU_STALL_DISABLED);

	trace_debug_oct_define_ringbuff((void *)-1, -1);

	/* set fill levels */
	SET_OCT_OCT_MASTER_TRIG_LEVEL1_DMA_TRIG_LEVEL(oct_trform,
			oct_trig_level1 >> 2);
	SET_OCT_OCT_MASTER_TRIG_LEVEL2_DMA_TRIG_LEVEL(oct_trform,
			OCT_TRIG_LEVEL_2 >> 2);

	/* reset & enable CYCLE interrupt */
	SET_OCT_OCT_MASTER_TRIG_CYCLE_DMA_TRIG_CYCLE(oct_trform,
			current_timeout);
	SET_OCT_OCT_CNF_ENABLE_TRIG_CYCLE_INT(oct_trform,
			OCT_CNF_ENABLE_TRIG_CYCLE_INT_ENABLED);
	SET_OCT_OCT_MASTER_RXCON_TIMEOUT(oct_trform, 1024);
	SET_OCT_OCT_MASTER_RXCON_EN_CH(oct_trform, 1);
	return;
}


static int str2int(char* str){
    int value = 0;
    char *s = str;
    if(NULL == str || strlen(str) == 0){
         return 0;
    }
    do{
        if(*s > '9' || *s < '0'){
             continue;
        }
        value *= 10;
        value += *s - '0';
    }while(*(++s) != 0);

    return value;

}

bool get_value_by_tag(char *config, char *tag, char *value, int size){
    int i = 0;
    char *tv = strstr(config, tag);
    if(value == NULL || size==0){
         return false;
    }
    if( tv != NULL && strlen(tv) != 0){
        bool b_equal_symbol = false;
        while(*(++tv) != '\n' && *tv != '\0' && i<size){
            if( !((*tv >= 'a' && *tv <= 'z')
                || (*tv >= 'A' && *tv <= 'Z')
                || (*tv >= '0' && *tv <= '9')
                || *tv == '_' || *tv == '.' || *tv == '=' || *tv == '/' || *tv == '\\')){
                  continue;
            }

            if(b_equal_symbol){
                value[i++] = *tv;
            }

            if(*tv == '='){
                 b_equal_symbol = true;
            }
        }
        value[i++] = '\0';
    }
    return true;
}


static bool oct_offlog_get_config(char* cfg_path){
    char cfg_context[OFFLOG_CONFIG_FILE_SIZE+1] = {0};
    char value_onoff[OFFLOG_CONFIG_MAX_VALUE_LENGTH+1] = {0};
    char value_path[OFFLOG_CONFIG_MAX_VALUE_LENGTH+1] = {0};
    char value_size[OFFLOG_CONFIG_MAX_VALUE_LENGTH+1] = {0};
    char value_num[OFFLOG_CONFIG_MAX_VALUE_LENGTH+1] = {0};
    struct file *fp_cfg;
    loff_t pos_cfg = 0;
    //loff_t pos_dir = 0;
    mm_segment_t old_fs;
    int file_len = 0;

    OCT_LOG("oct_get_offlog_config");

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    fp_cfg = filp_open(cfg_path, O_RDONLY, 0);

    if(!IS_ERR_OR_NULL(fp_cfg)){
        file_len = fp_cfg->f_op->llseek(fp_cfg, 0, SEEK_END);
        if( file_len > 0){
            fp_cfg->f_op->read(fp_cfg, cfg_context, file_len, &pos_cfg);
        }
        if( strlen(cfg_context) != 0 ){
            get_value_by_tag(cfg_context, OFFLOG_CONFIG_TAG_ONOFF, value_onoff, OFFLOG_CONFIG_MAX_VALUE_LENGTH);  
            get_value_by_tag(cfg_context, OFFLOG_CONFIG_TAG_PATH, value_path, OFFLOG_CONFIG_MAX_VALUE_LENGTH);  
            get_value_by_tag(cfg_context, OFFLOG_CONFIG_TAG_SIZE, value_size, OFFLOG_CONFIG_MAX_VALUE_LENGTH);  
            get_value_by_tag(cfg_context, OFFLOG_CONFIG_TAG_NUM, value_num, OFFLOG_CONFIG_MAX_VALUE_LENGTH);  
        }
        filp_close(fp_cfg, NULL);
        fp_cfg = NULL;
    } else {
        OCT_LOG("oct_get_offlog_config config not exist");
    }
	set_fs(old_fs);

    // config onoff
    oct_offlog_onoff = (value_onoff == NULL || strlen(value_onoff) == 0)? 0 : str2int(value_onoff);

    // config path
    memset (oct_offlog_path, 0, OFFLOG_PATH_LEN + 1);
    if(value_path == NULL || strlen(value_path) == 0){
        snprintf(oct_offlog_path, OFFLOG_PATH_LEN, "%s%s", DEFAULT_OFFLOG_PATH, DEFAULT_OFFLOG_NAME);
    } else {
        snprintf(oct_offlog_path, OFFLOG_PATH_LEN, "%s%s", value_path, DEFAULT_OFFLOG_NAME);
    }

    // config size
    oct_offlog_size = str2int(value_size);
    if(oct_offlog_size == 0){
        oct_offlog_size = DEFAULT_OCT_OFFLOG_SIZE;
    }

    // config file numbers
    oct_offlog_number = str2int(value_num);
    if(oct_offlog_number == 0){
        oct_offlog_number = DEFAULT_OCT_OFFLOG_NUMBER;
    }

    OCT_LOG("oct_offlog_get_config, onoff: %d, path: %s, size:%d, number:%d",oct_offlog_onoff, oct_offlog_path, oct_offlog_size, oct_offlog_number);

    return true;
}

void oct_offlog_check_mk_dirs(void){
     mm_segment_t old_fs;
     int offlog_path_length = strlen(oct_offlog_path);
     char c='/';
     char *nextdir = NULL;

     nextdir = strchr(oct_offlog_path+1, c);

     old_fs = get_fs();
     set_fs(KERNEL_DS);

     while(NULL != nextdir && strlen(nextdir) != 0){
         char curdir[OFFLOG_PATH_LEN+1] = {0};
         //bool if_path_exist = false;

         strncpy(curdir, oct_offlog_path, offlog_path_length - strlen(nextdir) +1);
         // TODO: test if curdir exist. if not exist, create it.

         sys_mkdir(curdir, 0777);
         nextdir = strchr(nextdir+1, c);
     }
	 set_fs(old_fs);

}


static bool oct_offlog_if_rotate(void){
    // if bplog exist
    int err = 0;
    if(NULL == oct_offlog_path){
        OCT_LOG("oct_offlog_if_rotate offlog path null");
        return false;
    }
    // TODO: check if bplog exist to deside if need rotate
    /*
    err = sys_access(oct_offlog_path, R_OK);
    if( !err ){
        OCT_LOG("oct_offlog_if_rotate offlog not exist");
        return false;
    }
    */
    OCT_LOG("oct_offlog_if_rotate return true");
    return true;
}

static int oct_offlog_rotate_files(void)
{
    mm_segment_t old_fs;
    int err;
    int i;

    OCT_LOG("oct_offlog_rotate_files");

    //remove bplog.4.istp
	old_fs = get_fs();
	set_fs(KERNEL_DS);
    {
        char path_del[OFFLOG_PATH_LEN+1] = {0};
        snprintf(path_del, OFFLOG_PATH_LEN, "%s.%d.istp", oct_offlog_path, oct_offlog_number-1);
        err = sys_unlink(path_del);
    }
    //rename other files
    for (i = oct_offlog_number-2; i >= 0; i--)
    {
        char oldpath[OFFLOG_PATH_LEN+1] = {0};
        char newpath[OFFLOG_PATH_LEN+1] = {0};

        if(i == 0){
            snprintf(oldpath, OFFLOG_PATH_LEN, "%s", oct_offlog_path);
        } else{
            snprintf(oldpath, OFFLOG_PATH_LEN, "%s.%d.istp", oct_offlog_path, i);
        }
        snprintf(newpath, OFFLOG_PATH_LEN, "%s.%d.istp", oct_offlog_path, i+1);
        sys_rename (oldpath, newpath);
    }

	set_fs(old_fs);

    return 1;
}

static void oct_offlog_chat_cmd(const char *devname,char *atcmd)
{
    mm_segment_t oldfs;
    struct file *filp;
    unsigned int writed = 0;
#define AT_CMD_LEN 254
    char readbuf[AT_CMD_LEN] = {0};
    unsigned int wait_ms= 200;
    unsigned int max_wait_times = 5;
    unsigned int wait_times = 0;


    oldfs = get_fs();
    set_fs(KERNEL_DS);

	filp = filp_open(devname, O_RDWR | O_NONBLOCK, 0);
	OCT_LOG("filp_open %s O_RDWR | O_NONBLOCK\n", devname);

    if (!filp || IS_ERR(filp)) {
        OCT_LOG("can not open [%s],filp=%ld\n", devname, PTR_ERR(filp));
    } else {
        writed = filp->f_op->write(filp, atcmd, strlen(atcmd), &filp->f_pos);
        OCT_LOG("write[%s][%d]: [%s]",devname,writed,atcmd);
        do{
            msleep(wait_ms);
            filp->f_op->read(filp, readbuf, AT_CMD_LEN - 1, &filp->f_pos);
            OCT_LOG("read[%s]: [%s]",devname, readbuf);
        }while((strlen(readbuf) == 0) && (wait_times++ < max_wait_times));

        filp_close(filp, NULL);
    }
    set_fs(oldfs);
}

static void oct_offlog_send_init_at(void)
{
//    oct_offlog_chat_cmd("/dev/vbpipe14", "ATE0V1\r");
//    oct_offlog_chat_cmd("/dev/vbpipe14", "at+trace=1\r");
//    oct_offlog_chat_cmd("/dev/vbpipe14", "at+xsio=0\r");
	oct_offlog_chat_cmd("/dev/mvpipe-bplog",
		"at+xsystrace=1,\"bb_sw=1;3g_sw=1;digrf=1\",\"digrf=0x84;bb_sw=sdl:th,tr,st,db,pr,lt,li,gt,ae,mo\",\"oct=4\"\r");
}

static void oct_offlog_send_shut_at(void)
{
	oct_offlog_chat_cmd("/dev/mvpipe-bplog", "at+xsystrace=0\r");
}


static void oct_write_data_to_usb(void *ptr, int num_bytes)
{
	mm_segment_t old_fs;
	int written_bytes;
	int verify_fp = 2;

	OCT_DBG("Sending to USB @%p:0x%x", ptr, num_bytes);
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	while (verify_fp > 0) {
		if (IS_ERR_OR_NULL(fp)) {
			OCT_LOG("Gadget fp=%p is not valid=%ld!",
					fp, PTR_ERR(fp));
			/* re-open the tty gadget */
			fp = filp_open(USB_CH_NAME, O_RDWR, 0);
		}
		if (IS_ERR_OR_NULL(fp)) {
			OCT_LOG("Gadget fp=%p re-open failed with error=%ld!",
						fp, PTR_ERR(fp));
			/* skip bytes in buffer and leave loop */
			written_bytes = num_bytes;
			verify_fp = -1;
		} else {
			/* try a first time write */
			written_bytes =
				fp->f_op->write(fp, (char *)ptr, num_bytes, 0);
			if (written_bytes <= 0) {
				OCT_LOG(
					"Error: USB write Error with valid fp=%p, %d",
					fp, written_bytes);
				/* come back and force the loop to re-open fp */
				fp = NULL;
				/* avoid infinity loop doing only
				 * one time the loop */
				verify_fp--;
				/* skip bytes in buffer and leave loop */
				written_bytes = num_bytes;
			} else {
				/* write sucessfuly to USB and leave the loop */
				verify_fp = -1;
			}
		}
	}
	set_fs(old_fs);

	/* increment the read ptr anyhow to re-start timeout */
	oct_read_ptr += written_bytes;
	/* check if result is consistent */
	if (oct_read_ptr > oct_ext_ring_buff_len) {
		/* too many bytes */
		OCT_LOG("Error: ReadPtr exceeds the buffer length");
		OCT_LOG("written bytes 0x%x, oct_read_ptr 0x%x",
				written_bytes, oct_read_ptr);
		oct_read_ptr = 0;
	} else if (oct_read_ptr == oct_ext_ring_buff_len) {
		oct_read_ptr = 0; /*wrap around*/
		OCT_DBG("ReadPtr wrap around:");
		OCT_DBG(" written bytes 0x%x, oct_read_ptr 0x%x",
				written_bytes, oct_read_ptr);
	}
	SET_OCT_OCT_SW_RPTR(oct_trform, oct_read_ptr);
	data_available = 0;
	/* reset & enable CYCLE interrupt */
	SET_OCT_OCT_MASTER_TRIG_CYCLE_DMA_TRIG_CYCLE(oct_trform,
							   current_timeout);
	SET_OCT_OCT_CNF_ENABLE_TRIG_CYCLE_INT(oct_trform,
				    OCT_CNF_ENABLE_TRIG_CYCLE_INT_ENABLED);
}

static void oct_write_data_to_file(void *ptr, int num_bytes)
{
	mm_segment_t old_fs;
	int written_bytes = 0;
	int verify_fp = 2;
	static int total_written_bytes = 0;

	old_fs = get_fs();
	set_fs(KERNEL_DS);
    if(total_written_bytes >= oct_offlog_size){
        if (!IS_ERR_OR_NULL(fp)){
            filp_close(fp, NULL);
            fp = NULL;
            pos = 0;
        }
        oct_offlog_rotate_files();
        total_written_bytes = 0;
    }
	while (verify_fp > 0) {

		if (IS_ERR_OR_NULL(fp)) {
			OCT_LOG("Gadget fp=%p is not valid=%ld!",
					fp, PTR_ERR(fp));
			/* re-open the tty gadget */
			fp = filp_open(oct_offlog_path, O_RDWR | O_CREAT | O_TRUNC, 0666);
		}
		if (IS_ERR_OR_NULL(fp)) {
			OCT_LOG("Gadget fp=%p re-open failed with error=%ld!",
						fp, PTR_ERR(fp));
			/* skip bytes in buffer and leave loop */
			written_bytes = num_bytes;
			verify_fp = -1;
		} else {
			/* try a first time write */

			//OCT_LOG("Sending to File fp=%p @0x%x:0x%x ",fp,(char *)ptr, num_bytes);
			written_bytes =
				fp->f_op->write(fp, (char *)ptr, num_bytes, &pos);
			if (written_bytes <= 0) {
				OCT_LOG(
					"Error: file write Error with valid fp=%p, %d",
					fp, written_bytes);
				/* come back and force the loop to re-open fp */
				fp = NULL;
				/* avoid infinity loop doing only
				 * one time the loop */
				verify_fp--;
				/* skip bytes in buffer and leave loop */
				written_bytes = num_bytes;
			} else {
				/* write sucessfuly to USB and leave the loop */
				verify_fp = -1;
                total_written_bytes += written_bytes;
			}
		}
	}
	set_fs(old_fs);

	/* increment the read ptr anyhow to re-start timeout */
	oct_read_ptr += written_bytes;
	/* check if result is consistent */
	if (oct_read_ptr > oct_ext_ring_buff_len) {
		/* too many bytes */
		OCT_LOG("Error: ReadPtr exceeds the buffer length");
		OCT_LOG("written bytes 0x%x, oct_read_ptr 0x%x",
				written_bytes, oct_read_ptr);
		oct_read_ptr = 0;
	} else if (oct_read_ptr == oct_ext_ring_buff_len) {
		oct_read_ptr = 0; /*wrap around*/
		OCT_DBG("ReadPtr wrap around:");
		OCT_DBG(" written bytes 0x%x, oct_read_ptr 0x%x",
				written_bytes, oct_read_ptr);
	}
	SET_OCT_OCT_SW_RPTR(oct_trform, oct_read_ptr);
	data_available = 0;
	/* reset & enable CYCLE interrupt */
	SET_OCT_OCT_MASTER_TRIG_CYCLE_DMA_TRIG_CYCLE(oct_trform,
							   current_timeout);
	SET_OCT_OCT_CNF_ENABLE_TRIG_CYCLE_INT(oct_trform,
				    OCT_CNF_ENABLE_TRIG_CYCLE_INT_ENABLED);
}


static const struct file_operations oct_proc_fops = {
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static int oct_thread(void *param)
{
	unsigned int num_bytes = 0, wptr_un = 0;
	OCT_DBG("Enter thread");

	oct_detect_off_flag(OFFLOG_CONFIG_FILE);
    OCT_DBG("oct_detect_off_flag oct_out_path: %d", oct_out_path);

	if (oct_out_path == OCT_PATH_FILE)	{
        oct_offlog_get_config(OFFLOG_CONFIG_FILE);
        if(oct_offlog_onoff == 0){
            oct_out_path = DEFAULT_OCT_PATH;
        }
    }
    OCT_DBG("oct_detect_off_flag oct_out_path: %d", oct_out_path);
    OCT_LOG("oct_out_path %d",oct_out_path);


	if (oct_out_path == OCT_PATH_TTY) {
		oct_offlog_send_shut_at();
		fp = oct_open_gadget(USB_CH_NAME);
	} else if (oct_out_path == OCT_PATH_FILE) {
		oct_offlog_check_mk_dirs();
		if (oct_offlog_if_rotate())
			oct_offlog_rotate_files();

		oct_offlog_send_init_at();
		fp = oct_open_gadget(oct_offlog_path);
    }
	while (!kthread_should_stop()) {
		wait_event_interruptible(oct_wq, oct_events);
		oct_events = 0;
		OCT_DBG("Event received");
		/* read the whole WPTR register including flags */
		wptr_un = GET_OCT_OCT_MASTER_WPTR(oct_trform);

		if (OCT_MASTER_WPTR_DAT(wptr_un)) { /* if data */
			/* check ext mem and get
			 * nb of bytes available to be sent */
			unsigned int read_ptr, write_ptr;
			OCT_DBG("DATA available to be sent out");
			data_available = 1;
			wake_up(&oct_poll_wq);
			read_ptr = oct_read_ptr;
			write_ptr = OCT_MASTER_WPTR_MASTER_WPTR(wptr_un) << 2;

			/* not sure if we have to flush -but I did it */
			SET_OCT_OCT_MASTER_RXCON_FLUSH(oct_trform, 1);
			SET_OCT_OCT_MASTER_RXCON_FLUSH(oct_trform, 0);

			if (write_ptr > read_ptr) {
				num_bytes = write_ptr - read_ptr;
			} else {
				num_bytes = oct_ext_ring_buff_len - read_ptr;
				if (write_ptr == read_ptr)
					oct_ext_mem_full++;
			}
			if (oct_out_path == OCT_PATH_TTY) {
				/* Sync cache */
				dma_sync_single_for_cpu(NULL, phy_base_addr +
					oct_read_ptr, num_bytes,
					DMA_FROM_DEVICE);

				/* call subscribed function to forward data */

				oct_write_data_to_usb((char *)
					(&((char *)oct_ext_rbuff_ptr)[oct_read_ptr]),
						num_bytes);

					OCT_DBG("Sent out %d bytes", num_bytes);

			}
			else if (oct_out_path == OCT_PATH_FILE)
			{
				dma_sync_single_for_cpu(NULL, phy_base_addr +
					oct_read_ptr, num_bytes,
					DMA_FROM_DEVICE);
				/* call subscribed function to forward data */
					oct_write_data_to_file((char *)
					(&((char *)oct_ext_rbuff_ptr)[oct_read_ptr]),
						num_bytes);

//					OCT_DBG("Sent out %d bytes", num_bytes);
			}
		} else /* if no data available*/
			OCT_DBG("NO data available to be sent out");

		/*schedule_timeout(100);*/
		/*msleep(10); - wait some time till next thread run*/
		set_current_state(TASK_INTERRUPTIBLE);
		SET_OCT_OCT_CNF_ENABLE_TRIG_LEVEL1_INT(oct_trform,
				OCT_CNF_ENABLE_TRIG_LEVEL1_INT_ENABLED);
		SET_OCT_OCT_CNF_ENABLE_TRIG_LEVEL2_INT(oct_trform,
				OCT_CNF_ENABLE_TRIG_LEVEL2_INT_ENABLED);
	}
	OCT_DBG("WHILE exited");
	return 0;
}

static void oct_tasklet_bh(unsigned long data)
{
	/*__set_current_state(TASK_RUNNING);*/
	oct_events = 1;
	wake_up(&oct_wq);
	/*wake_up_process(task);*/
}

static ssize_t oct_read(struct file *file_ptr, char __user *user_buffer,
		size_t count, loff_t *position)
{
	unsigned int read_ptr, write_ptr;
	unsigned int num_bytes = 0;
	unsigned  wptr_un = 0;
	unsigned int result = 0;

	OCT_DBG("read at offset = %i, bytes count = %u",
			(int)*position,
			(unsigned int)count);
	if (oct_out_path != OCT_PATH_APP_POLL)
		return 0;
	/* read the whole WPTR register including flags */
	wptr_un  =
		GET_OCT_OCT_MASTER_WPTR(oct_trform);

	if (OCT_MASTER_WPTR_DAT(wptr_un)) { /* if data */
		write_ptr = OCT_MASTER_WPTR_MASTER_WPTR(wptr_un) << 2;
		read_ptr  = oct_read_ptr;

		if (write_ptr > read_ptr) {
			num_bytes = write_ptr - read_ptr;
		} else {
		    num_bytes = oct_ext_ring_buff_len - read_ptr;
		    if (write_ptr == read_ptr)
				oct_ext_mem_full++;
		}
		if (num_bytes > count)/*if requested less bytes than available*/
		    num_bytes = count;/*send only the requested size*/
		else {
		    data_available = 0;/*all the bytes will be sent */
		    /* reset & enable CYCLE interrupt */
		    SET_OCT_OCT_MASTER_TRIG_CYCLE_DMA_TRIG_CYCLE(
								oct_trform,
							    current_timeout);
		    SET_OCT_OCT_CNF_ENABLE_TRIG_CYCLE_INT(oct_trform,
				    OCT_CNF_ENABLE_TRIG_CYCLE_INT_ENABLED);
		}
		/* Sync cache */
		dma_sync_single_for_cpu(NULL, phy_base_addr +
			oct_read_ptr, num_bytes, DMA_FROM_DEVICE);

		result = copy_to_user(user_buffer,
			    &((char *)oct_ext_rbuff_ptr)[oct_read_ptr],
			    num_bytes);
		if (result)
			OCT_LOG("Error copy_to_user");

		oct_read_ptr += num_bytes;
		/* check if result is consistent*/
		if (oct_read_ptr > oct_ext_ring_buff_len) {
			/* too many bytes */
			OCT_DBG("Error: ReadPtr exceeds the buffer length");
			oct_read_ptr = 0;
		} else if (oct_read_ptr == oct_ext_ring_buff_len)
			oct_read_ptr = 0; /* wrap around*/

		SET_OCT_OCT_SW_RPTR(oct_trform, oct_read_ptr);
	}
	OCT_DBG("%x bytes sent out", num_bytes);
	return num_bytes;
}

static unsigned int oct_poll(struct file *file, poll_table *wait)
{
	unsigned int ret = 0;
	OCT_DBG("Poll-%d", data_available);
	if (data_available) {
		ret |= POLLIN | POLLRDNORM;
		return ret;
	}
	poll_wait(file, &oct_poll_wq, wait);
	if (data_available)
		ret |= POLLIN | POLLRDNORM;
	return ret;
}

static unsigned int get_oct_mode(void)
{
	/* OCT_CNF_ENABLE_EXTM_RINGBF_ENABLED */
	if (GET_OCT_OCT_CNF_ENABLE_EXTM_RINGBF(oct_trform))
		return OCT_MODE_OW;
	/* OCT_CNF_ENABLE_EXTM_RINGBF_DISABLED */
	else
		return OCT_MODE_STALL;
}

static ssize_t oct_write(struct file *p_file, const char __user *user_buffer,
		size_t count, loff_t *position)
{
	unsigned char *buffer;
	mm_segment_t old_fs;
	int result;
	buffer = kmalloc(count+1, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;
	result = copy_from_user(buffer, user_buffer, count);
	if (result)
		OCT_DBG("Error copy_from_user");
	buffer[count] = '\0';
	OCT_DBG("Buffer: %s Count: %d", (char *)buffer, (uint32_t)count);
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	{
	if (!strncmp(buffer, "stop", 4)) {
		free_irq(OCT_INT, (void *)(oct_irq_handler));
		OCT_DBG("Oct Stopped");
	} else if (!strncmp(buffer, "start", 5)) {
		result = request_irq(OCT_INT, (irq_handler_t)oct_irq_handler,
			     IRQF_SHARED, "OCT_Drv", (void *)(oct_irq_handler));
		if (result)
			OCT_LOG("Oct can't start");
		else
			OCT_LOG("Oct Started");
	} else if (!strncmp(buffer, "init", 4)) {
		/*oct_open_port(USB_CH_NAME);*/
		oct_mode = get_oct_mode();
		OCT_DBG("get_oct_mode returns:  %x", oct_mode);
		oct_hw_init();
		if (oct_mode == OCT_MODE_OW)
			oct_out_path = OCT_PATH_NONE;
	} else if (!strncmp(buffer, "info", 4)) {
		void *tadihandle;

		OCT_LOG("OCT driver version: %d", OCT_DRIVER_VER);
		OCT_LOG("OCT Power (1-ON/0-OFF) %d", oct_check_power());
		OCT_LOG("oct_trform=%p",
				oct_trform);
		OCT_LOG("Ptr: %p Len: %d",
				oct_ext_rbuff_ptr, oct_ext_ring_buff_len);
		OCT_LOG("Wptr: %d",
				GET_OCT_OCT_MASTER_WPTR(oct_trform));
		OCT_LOG("Rptr: %d",
				GET_OCT_OCT_SW_RPTR(oct_trform));
		OCT_LOG("Base %#x",
				GET_OCT_OCT_MASTER_RXCH0_BASE(oct_trform));
		OCT_LOG("Size:%#x",
				GET_OCT_OCT_MASTER_RXCH0_SIZE(oct_trform));
		OCT_LOG("IRQ Stat:%#x",
				GET_OCT_OCT_MASTER_RXIRQ_STAT(oct_trform));
		OCT_LOG("SW-Rptr: %#x", oct_read_ptr);
		OCT_LOG("Data_avail: %d", data_available);
		tadihandle = trc_tadi_open(MT_PRINTF);
		trc_tadi_write(tadihandle,
				"Including Tadi Kernel Interface", 31);
		trc_tadi_write(tadihandle, "Ending", 6);
		trc_tadi_close(tadihandle);

	} else if (!strncmp(buffer, "off", 5)) {
		oct_mode_off();
	} else if (!strncmp(buffer, "debug", 5)) {
		debug_on ^= 1;
	} else if (!strncmp(buffer, "flush", 5)) {
		unsigned int num_bytes = 0;
		unsigned  wptr_un = 0;
		OCT_LOG("Flush command received");
		/* read the whole WPTR register including flags*/
		wptr_un  =
			GET_OCT_OCT_MASTER_WPTR(oct_trform);

		if (OCT_MASTER_WPTR_DAT(wptr_un)) {/* if data */
		    /*check ext mem and get nb of bytes available to be sent*/
		    unsigned int read_ptr, write_ptr;
		    OCT_LOG("flush: DATA available to be sent out");

		    read_ptr  = oct_read_ptr;
		    write_ptr =
			OCT_MASTER_WPTR_MASTER_WPTR(wptr_un) << 2;

		    /*not sure if we have to flush -but I did it */
		    SET_OCT_OCT_MASTER_RXCON_FLUSH(oct_trform, 1);
		    SET_OCT_OCT_MASTER_RXCON_FLUSH(oct_trform, 0);

		    if (write_ptr > read_ptr) {
			num_bytes = write_ptr - read_ptr;
		    } else {
			num_bytes = oct_ext_ring_buff_len - read_ptr;
			if (write_ptr == read_ptr)
				oct_ext_mem_full++;
		    }

			OCT_LOG("flush: %x write_ptr_B", write_ptr);
			OCT_LOG("flush: %x read_ptr_B", read_ptr);
			OCT_LOG("flush: %x oct_read_ptr_B", oct_read_ptr);
			OCT_LOG("flush: %x num_bytes_B", num_bytes);

			/* Sync cache */
			dma_sync_single_for_cpu(NULL, phy_base_addr +
				oct_read_ptr, num_bytes, DMA_FROM_DEVICE);

			/* call subscribed function to forward data */
			oct_write_data_to_usb((char *)
			    (&((char *)oct_ext_rbuff_ptr)[oct_read_ptr]),
			    num_bytes);
			OCT_LOG("flush: %x write_ptr_A", write_ptr);
			OCT_LOG("flush: %x read_ptr_A", read_ptr);
			OCT_LOG("flush: %x oct_read_ptr_A", oct_read_ptr);
			OCT_LOG("flush: %x num_bytes_A", num_bytes);
		} else { /* if no data available */
		    OCT_DBG("flush: NO data available to be sent out");
		}
	} else if (!strncmp(buffer, "usb", 3)) {
		if (oct_out_path == OCT_PATH_TTY)
			oct_out_path = OCT_PATH_NONE;
		else
			oct_out_path = OCT_PATH_TTY;
	} else if (!strncmp(buffer, "open", 4)) {
		fp = filp_open("/dev/ttyGS1", O_RDWR, 0);
		OCT_LOG("Opening Gadget returned %ld", PTR_ERR(fp));
	} else {
		int nb_of_bytes;
		nb_of_bytes =  fp->f_op->write(fp, buffer, count, 0);
		OCT_LOG("Bytes to write %d, Bytes written %d",
				(uint32_t)count, nb_of_bytes);
	}
	if (count <= 0)
		OCT_LOG("OCT to USB write Error");
	}
	set_fs(old_fs);
	kfree(buffer);
	return count;
}

static int oct_open(struct inode *p_inode, struct file *p_file)
{
	return 0;
}

static int oct_release(struct inode *p_inode, struct file *p_file)
{
	return 0;
}

static long oct_ioctl(struct file *p_file,
		unsigned int cmnd, unsigned long param)
{
	unsigned wptr_un = 0;
	struct s_oct_info oct_info;
	int result;

	OCT_DBG("Ioctl cmd %d, param: %ld", cmnd, param);

	switch (cmnd) {
	case OCT_IOCTL_SET_PATH:
	    oct_out_path = param;
	    break;
	case OCT_IOCTL_SET_MODE:
	    oct_mode = param;
	    if (param == OCT_MODE_OFF)
		oct_mode_off();
	    else
		oct_hw_init();
		if (param == OCT_MODE_OW)
			oct_out_path = OCT_PATH_NONE;
	    break;
	case OCT_IOCTL_CONF_TRIG_CYCLE:
	    current_timeout = param * 1000 * 26;
	    /* reset & enable CYCLE interrupt */
	    SET_OCT_OCT_MASTER_TRIG_CYCLE_DMA_TRIG_CYCLE(oct_trform,
							    current_timeout);
	    SET_OCT_OCT_CNF_ENABLE_TRIG_CYCLE_INT(oct_trform,
				OCT_CNF_ENABLE_TRIG_CYCLE_INT_ENABLED);
	    break;
	case OCT_IOCTL_CONF_TRIG_L1:
	    /* configuration in procentage 0-100% */
	    oct_trig_level1 = (param * OCT_EXT_RING_BUFF_SIZE) / 100;
	    SET_OCT_OCT_MASTER_TRIG_LEVEL1_DMA_TRIG_LEVEL(oct_trform,
							oct_trig_level1 >> 2);
	    break;
	case OCT_IOCTL_GET_INFO:
	    wptr_un  =
		GET_OCT_OCT_MASTER_WPTR(oct_trform);
	    oct_info.wr_ptr = (unsigned int)
		OCT_MASTER_WPTR_MASTER_WPTR(wptr_un) << 2;
	    oct_info.rd_ptr =
		(unsigned int)GET_OCT_OCT_SW_RPTR(oct_trform);
	    oct_info.is_full =
		OCT_MASTER_WPTR_EXT_MEM_FULL(wptr_un);
	    result = copy_to_user((struct s_oct_info *)param,
			    &oct_info,
			    sizeof(oct_info));
	    if (result)
		OCT_LOG("Error in Ioctl Get info");
	    break;
	case OCT_IOCTL_ENTER_CD:
	    /* todo: - for coredump*/
	    break;
	case OCT_IOCTL_FLUSH:
	    /*todo: - for coredump*/
	    break;
	default:
	    OCT_DBG("Ioctl command out of range");
	}
	return 0;
}

static const struct file_operations oct_fops = {
	.owner			= THIS_MODULE,
	.aio_write		= NULL,
	.read			= oct_read,
	.poll			= oct_poll,
	.write			= oct_write,
	.open			= oct_open,
	.release		= oct_release,
	.unlocked_ioctl	= oct_ioctl
};

static int oct_driver_probe(struct platform_device *pdev)
{
	int result = 0;
	int data = 0;
	struct device *dev_oct;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct resource *mem;
	int size, err;

	OCT_DBG("%s", __func__);

	result = register_chrdev(0, OCT_MODULE_NAME, &oct_fops);
	if (result < 0) {
		OCT_LOG("can\'t register dev errorcode=%i", result);
		return -1;
	}

	major = result;
	class_oct = class_create(THIS_MODULE, OCT_MODULE_NAME);

	if (IS_ERR(class_oct)) {
		unregister_chrdev(major, OCT_MODULE_NAME);
		OCT_LOG("error creating class");
		return -1;
	}

	dev_oct = device_create(class_oct, NULL,
			MKDEV(major, 0),
			NULL, OCT_MODULE_NAME);
	if (IS_ERR(dev_oct)) {
		class_destroy(class_oct);
		unregister_chrdev(major, OCT_MODULE_NAME);
		OCT_LOG("Error creating device");
		return -1;
	}
	OCT_DBG("registered char device, major number=%i", major);

	mem = platform_get_resource_byname(pdev,
			IORESOURCE_MEM, "oct-registers");
	if (!mem) {
		OCT_LOG("no oct-registers resource?\n");
		return -1;
	}
	size = (mem->end - mem->start) + 1;
	oct_base = ioremap(mem->start, size);
	OCT_LOG("oct resource: %pR - ioremap: %p\n", mem, oct_base);
	if (!oct_base) {
		pr_err("%s: unable to remap memory region\n", __func__);
		release_mem_region(mem->start, size);
		return -1;
	}
	/* pm */
	pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(pm_platdata)) {
		pr_err("%s: Error during device state pm init\n", __func__);
		return -1;
	}
	if (pm_platdata) {
		err = device_state_pm_set_class(dev, pm_platdata->pm_user_name);
		if (err) {
			pr_err("%s: Error while setting the pm class for user %s\n",
				__func__, pm_platdata->pm_user_name);
			return -1;
		}
		err = device_state_pm_set_state_by_name(dev,
				pm_platdata->pm_state_D0_name);
		if (err) {
			pr_err("%s: Error while setting the pm state: %s\n",
				__func__, pm_platdata->pm_state_D0_name);
			return -1;
		} else
			pr_debug("%s: %s requested for user %s\n", __func__,
					pm_platdata->pm_state_D0_name,
					pm_platdata->pm_user_name);
	} else
		pr_debug("pm_platdata is NULL\n");

	oct_mode = get_oct_mode();
	OCT_DBG("get_oct_mode returns: %x", oct_mode);
	oct_hw_init();
	if (oct_mode == OCT_MODE_OW)
		oct_out_path = OCT_PATH_NONE;
	oct_ext_mem_full = 0;
	init_waitqueue_head(&oct_wq);
	init_waitqueue_head(&oct_poll_wq);
	task = kthread_run(oct_thread, (void *)&data, "OCT Thread");
	OCT_DBG("Kernel OCT Thread: %s", task->comm);

	oct_int = platform_get_irq_byname(pdev, "OCT_INT");
	if (oct_mode != OCT_MODE_OW && !IS_ERR_VALUE(oct_int)) {
		result = request_irq(OCT_INT,
				(irq_handler_t) oct_irq_handler,
				IRQF_SHARED,
				"OCT_Drv",
				(void *)(oct_irq_handler));
		if (result)
			OCT_LOG("can't get shared interrupt");
		else
			OCT_DBG("Interrupt %d installed", oct_int);
	}

	/* Runtime PM initialization */
	pm_runtime_enable(dev);
	OCT_DBG("PM runtime enabled");
	pm_runtime_irq_safe(dev);
	pm_runtime_get_sync(dev);
	OCT_DBG("PM runtime get");

	/* Do some other initialization.. if any.
	 * And put when no longer needed */
	pm_runtime_put_sync(dev);
	OCT_DBG("PM runtime put");

	return 0;
}

static int oct_driver_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int err;

	OCT_LOG("Removing...");

	/* Deregister IRQ */
	free_irq(OCT_INT, (void *)(oct_irq_handler));
	oct_events = 1;
	wake_up(&oct_wq);
	kthread_stop(task);
	iounmap(OCT_REG_ADDRESS_BASE);
	dma_unmap_single(NULL, phy_base_addr,
			OCT_EXT_RING_BUFF_SIZE, DMA_FROM_DEVICE);
	kfree(oct_ext_rbuff_ptr);

	device_destroy(class_oct, MKDEV(major, minor));
	class_destroy(class_oct);
	unregister_chrdev(major, OCT_MODULE_NAME);
	pm_runtime_disable(dev);
	OCT_DBG("device unregistration");
	if (pm_platdata) {
		err = device_state_pm_set_state_by_name(dev,
				pm_platdata->pm_state_D3_name);
		if (err) {
			pr_err("%s: Error while setting the pm state: %s\n",
				__func__, pm_platdata->pm_state_D3_name);
			return -1;
		}
	}

	return 0;
}

static const struct of_device_id xgold_oct_of_match[] = {
	{
		.compatible = "intel,oct",
	},
	{}
};

MODULE_DEVICE_TABLE(of, xgold_oct_of_match);

static struct platform_driver oct_driver = {
	.probe = oct_driver_probe,
	.remove = oct_driver_remove,
	.driver = {
		.name = PLATFORM_DRIVER_NAME,
		.pm	= &my_pm_ops,
		.of_match_table = of_match_ptr(xgold_oct_of_match),
	}
};

static int __init oct_init(void)
{
	int err;
	OCT_DBG("Module loading...ver %d", OCT_DRIVER_VER);

	err = platform_driver_register(&oct_driver);
	if (err) {
		OCT_LOG("Unable to register platform driver");
		return -1;
	}
	OCT_DBG("Platform driver registration ok...");
	return 0;
}

static void __exit oct_exit(void)
{
	/* Deregister Driver */
	platform_driver_unregister(&oct_driver);
	OCT_DBG("module removed");
}

module_init(oct_init);
module_exit(oct_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("OCT driver");
