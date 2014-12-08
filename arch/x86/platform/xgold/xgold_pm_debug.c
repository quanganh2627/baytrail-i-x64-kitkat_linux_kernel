/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/irqchip.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/suspend.h>

#ifdef CONFIG_PLATFORM_DEVICE_PM_VIRT
#ifdef CONFIG_X86_INTEL_SOFIA
#include <sofia/mv_gal.h>
#include <sofia/pal_shared_data.h>
#include <sofia/vmm_pmic.h>
#include <sofia/mv_hypercalls.h>
#include <sofia/mv_svc_hypercalls.h>
#endif
#endif


/*****************************************************************************
  helper functions for bit operation
******************************************************************************/

uint getbit(uint val, uint high, uint low)
{
	uint maxofuint = sizeof(uint)*8 - 1;
	/*input must check outside*/
	if (high < low) {
		high ^= low;
		low  ^= high;
		high ^= low;
	}
	val <<= (maxofuint - high);		/* must pad 0 */
	val >>= (maxofuint - high + low);	/* must pad 0 */
	return val;
}
EXPORT_SYMBOL_GPL(getbit);

uint getaddrbit(uint addr, uint high, uint low)
{
	uint maxofuint = sizeof(uint)*8 - 1;
	if (high > maxofuint || low > maxofuint)
		return (uint)-1;

	return getbit(readl((void __iomem *)addr), high, low);
}
EXPORT_SYMBOL_GPL(getaddrbit);

uint setbit(uint val, uint high, uint low, uint fill)
{
	uint mask = 0;
	uint lenofuint = sizeof(uint)*8;
	uint lenoffill;

	/*input must check outside*/
	if (high < low) {
		high ^= low;
		low  ^= high;
		high ^= low;
	}
	lenoffill = high - low + 1;

	mask = ((uint)-1) >> (lenofuint - lenoffill);
	fill &= mask;

	mask = (((uint)-1)<<(high+1)) + ((uint)-1>>(lenofuint-low));
	val &= mask;
	val |= (fill << low);

	return val;
}
EXPORT_SYMBOL_GPL(setbit);

uint setaddrbit(uint addr, uint high, uint low, uint fill)
{
	uint val;
	uint maxofuint = sizeof(uint) * 8 - 1;
	if (high > maxofuint || low > maxofuint)
		return (uint)-1;

	val = readl((void __iomem *)addr);
	val = setbit(val, high, low, fill);
	writel(val, (void __iomem *)addr);
	return val;
}
EXPORT_SYMBOL_GPL(setaddrbit);


static struct dentry *dentry_spcu_reg;

#define MAX_ERRLEN 255
static char err_buf[MAX_ERRLEN+1];

#define SPCU_BASE 0xE4700000

static LIST_HEAD(spcu_regs);

struct regval {
	struct list_head link;
	uint offset;
	uint val;
};

#if 0
static void push_reg(uint offset)
{
	struct regval *r;

	r = kzalloc(sizeof(struct regval), GFP_KERNEL);
	if (r) {
		r->offset = offset;
		list_add_tail(&r->link, &spcu_regs);
	}
	return;
}
#endif
static uint spcu_reg_read(uint offset)
{
	uint val = 0;
	int ret;
	ret = mv_svc_reg_read(SPCU_BASE+offset, &val, (uint)-1);
	if (ret)
		sprintf(err_buf,
			"%s: failed, ret=%d, offset:%x\n", __func__,
			ret, offset);
	return val;

}

static void spcu_regs_snapshot(void)
{
	struct regval *r;
	list_for_each_entry(r , &spcu_regs, link) {
		r->val = spcu_reg_read(r->offset);
	}
}

/*
static uint spcu_get_reg_from_snapshot(uint offset)
{
	struct regval *r;
	list_for_each_entry(r , &spcu_regs, link) {
		if (r->offset == offset)
			return r->val;
	}
	return 0;
}
*/
/*****************************************************************************
	xgold_pm_snapshot
@state: 0 - just before halt
	1 - just after halt
******************************************************************************/

#define AMOUNT_REGS 2
struct snapshot {
	struct list_head link;
	unsigned long long time_us;	/* utc time */
	u64 jiffies;
	int  state;		/* 0-sleep;1-wake */
	uint dur;
	char *trigger;
	uint regs[AMOUNT_REGS];
};

#define PM_SNAPSHOT_MAX 200
static int pm_snapshot_count;
static LIST_HEAD(list_pm_snapshot);
static unsigned long long total_sleep;
static unsigned long long total_wake;

void xgold_pm_snapshot(int state)
{

	struct snapshot *r, *r_prev;
	struct timespec boot_time;
	r = kzalloc(sizeof(struct snapshot), GFP_KERNEL);
	if (r) {
		/*getnstimeofday(&(r->ts));*/
		mv_svc_rtc_get_time_us(&r->time_us);
		r->state = state;
#if 0
		r->regs[0] = spcu_reg_read(0x828);
		r->regs[1] = spcu_reg_read(0x82c);
#endif
		list_add_tail(&r->link, &list_pm_snapshot);
		pm_snapshot_count++;
		if (pm_snapshot_count == 1) {
			boot_time = get_monotonic_coarse();
			total_wake += boot_time.tv_sec * 1000 +
				boot_time.tv_nsec / 1000000;
			r->dur = total_wake;
		} else {
			r_prev = list_entry(r->link.prev,
				struct snapshot, link);
			r->dur = (uint)((r->time_us - r_prev->time_us));
			r->dur /= 1000;
			if (state)
				total_sleep += r->dur;
			else
				total_wake += r->dur;

		}
		if (pm_snapshot_count >= PM_SNAPSHOT_MAX) {
			r = list_entry(list_pm_snapshot.next,
			struct snapshot, link);
			list_del(list_pm_snapshot.next);
			kfree(r);
		}
	}
	return;
}

static struct dentry *dentry_sleep_history;

static int sleep_history_show(struct seq_file *s, void *unused)
{
	struct snapshot *r;
	struct rtc_time tm;
	seq_puts(s, "state    wall_t    dur(s)    log\n");

	list_for_each_entry(r , &list_pm_snapshot, link) {
		do_div((r->time_us), 1000000);
		rtc_time_to_tm(r->time_us, &tm);
		seq_printf(s,
		"%s %d-%02d-%02d %02d:%02d:%02d %d.%03d\n",
		r->state ? "wake ":"sleep",
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec, r->dur/1000, r->dur%1000);
	}

	return 0;
}

static int sleep_history_open(struct inode *inode, struct file *file)
{
	return single_open(file, sleep_history_show, NULL);
}

static const struct file_operations sleep_history_fops = {
	.owner		=	THIS_MODULE,
	.open		=	sleep_history_open,
	.read		=	seq_read,
	.llseek		=	seq_lseek,
	.release	=	single_release,
};


/*****************************************************************************
	residency
******************************************************************************/
static struct dentry *dentry_residency;

static int residency_show(struct seq_file *s, void *unused)
{
	uint residency = 0;
	struct timespec xtim, wtom, sleep, boot;

	get_xtime_and_monotonic_and_sleep_offset(&xtim, &wtom, &sleep);
	boot = get_monotonic_coarse();

	seq_printf(s, "sleep:%u.%03u\n",
			(uint)sleep.tv_sec, (uint)sleep.tv_nsec / 1000000);
	/*seq_printf(s, "alive:%u.%03u\n",
			(uint)xtim.tv_sec, (uint)xtim.tv_nsec / 1000000);*/
	seq_printf(s, "alive:%u.%03u\n",
			(uint)boot.tv_sec, (uint)boot.tv_nsec / 1000000);
	/*seq_printf(s, "wtom :%u.%03u\n",
			(uint)wtom.tv_sec, (uint)wtom.tv_nsec / 1000000);*/

	if (xtim.tv_sec != 0)
		residency = sleep.tv_sec * 10000 / boot.tv_sec;

	seq_printf(s, "sleep percentage for linux : %02d.%02d%%\n",
		residency / 100, residency % 100);

	return 0;
}

static int residency_open(struct inode *inode, struct file *file)
{
	return single_open(file, residency_show, NULL);
}

static const struct file_operations residency_fops = {
	.owner		=	THIS_MODULE,
	.open		=	residency_open,
	.read		=	seq_read,
	.llseek		=	seq_lseek,
	.release	=	single_release,
};

/*****************************************************************************
	spcu_reg
******************************************************************************/
static struct dentry *dentry_spcu_reg;

static int dump_spcu_reg_show(struct seq_file *s, void *unused)
{
	int ret = 0;
	struct regval *r;

	spcu_regs_snapshot();
	seq_puts(s, "spcu reg:\n");

	list_for_each_entry(r , &spcu_regs, link)
		seq_printf(s, "%x:%x\n", r->offset, r->val);

	seq_printf(s, "err:%s\n", err_buf);

	/*seq_printf(s, "\nPower State Status(A44):\n");
	seq_printf(s, "3G resource:%s\n",
	getbit(spcu_get_reg_from_snapshot(0xA44), 31, 31) ? "ON":"OFF");
	*/
	return ret;
}

static int dump_spcu_reg_open(struct inode *inode, struct file *file)
{
	return single_open(file, dump_spcu_reg_show, NULL);
}

static const struct file_operations dump_spcu_reg_fops = {
	.owner		=	THIS_MODULE,
	.open		=	dump_spcu_reg_open,
	.read		=	seq_read,
	.llseek		=	seq_lseek,
	.release	=	single_release,
};

/*****************************************************************************
debug interface to dump value of a given virtual address, must mapped before
read, otherwise will cause system hang.
******************************************************************************/

static struct dentry *dentry_read_addr;
/*
#define LOG_IN_PMEM 0x2FF00000
*/
static uint addr_any;

#define BUFSIZE 256

static ssize_t dump_addr_write(struct file *file, const char __user *userbuf,
	size_t count, loff_t *ppos)
{
	char cmd[BUFSIZE];

	memset(err_buf, 0, sizeof(err_buf));

	if (count > BUFSIZE) {
		sprintf(err_buf, "input too much!\n");
		return count;
	}

	memset(cmd, 0, BUFSIZE);
	if (copy_from_user(cmd, userbuf, count)) {
		sprintf(err_buf, "copy_from_user fail.\n");
		return count;
	}

	if (sscanf(cmd, "0x%x", &addr_any) != 1) {
		sprintf(err_buf, "input format wrong!\n");
		return count;
	}
	return count;
}


static int dump_addr_show(struct seq_file *s, void *unused)
{
	int ret = 0;

	seq_printf(s, "Intel Xgold addr %x: %x\n",
			addr_any, *((uint *)addr_any));

	return ret;
}

static int dump_addr_open(struct inode *inode, struct file *file)
{
	return single_open(file, dump_addr_show, NULL);
}

static const struct file_operations dump_addr_fops = {
	.owner		=	THIS_MODULE,
	.open		=	dump_addr_open,
	.read		=	seq_read,
	.write          =       dump_addr_write,
	.llseek		=	seq_lseek,
	.release	=	single_release,
};


/*****************************************************************************
pm notifier
******************************************************************************/
#ifdef CONFIG_PM
static struct notifier_block pm_notify;
static int pm_debug_notify(struct notifier_block *notify_block,
		unsigned long mode, void *unused)
{
	switch (mode) {
	case PM_SUSPEND_PREPARE:
		xgold_pm_snapshot(0);
		break;
	case PM_POST_SUSPEND:
		xgold_pm_snapshot(1);
		break;
	default:
		break;
	}
	return 0;
}
#endif
/*****************************************************************************
	module init
******************************************************************************/
static struct dentry *dentry_xgold_pm_debug;

static int __init xgold_pm_debug_init(void)
{

	dentry_xgold_pm_debug = debugfs_create_dir("xgold_pm_debug", NULL);
	if (!dentry_xgold_pm_debug || IS_ERR(dentry_xgold_pm_debug))
		return -EFAULT;

	dentry_spcu_reg = debugfs_create_file("spcu_reg",
			S_IFREG | S_IRUGO, dentry_xgold_pm_debug, NULL,
			&dump_spcu_reg_fops);
	if (!dentry_spcu_reg)
		goto CLEAN_UP;

	dentry_read_addr = debugfs_create_file("read_addr",
			S_IFREG | S_IRUGO, dentry_xgold_pm_debug, NULL,
			&dump_addr_fops);
	if (!dentry_read_addr)
		goto CLEAN_UP;

	dentry_sleep_history = debugfs_create_file("sleep_history",
			S_IFREG | S_IRUGO, dentry_xgold_pm_debug, NULL,
			&sleep_history_fops);
	if (!dentry_sleep_history)
		goto CLEAN_UP;

	dentry_residency = debugfs_create_file("ratio_linux",
			S_IFREG | S_IRUGO, dentry_xgold_pm_debug, NULL,
			&residency_fops);
	if (!dentry_residency)
		goto CLEAN_UP;

#ifdef CONFIG_PM
	pm_notify.notifier_call = pm_debug_notify;
	register_pm_notifier(&pm_notify);
#endif



#if 0
	/**
	power gating & clock gating related regs:
	**/
	push_reg(0x820); /* hardware wakeup config 0 */
	push_reg(0x824); /* hardware wakeup config 1 */
	push_reg(0x828); /* HW wakeup status 0 */
	push_reg(0x82C); /* HW wakeup status 1 */
	push_reg(0xa20); /* system sleep config */
	push_reg(0xa3c); /* system isolation ctrl */
	push_reg(0xa40); /* power state control */
	push_reg(0xa44); /* power state status */
	push_reg(0xa48); /* 2G modem DSPs memory power config */
	push_reg(0xa4c); /* audio DSPs memory power config */
	push_reg(0xa50); /* main system memory power config */
#endif

	return 0;
CLEAN_UP:
	debugfs_remove(dentry_xgold_pm_debug);
	dentry_xgold_pm_debug = NULL;
	return -EFAULT;
}
module_init(xgold_pm_debug_init);

static void __exit xgold_pm_debug_exit(void)
{
	debugfs_remove(dentry_xgold_pm_debug);
#ifdef CONFIG_PM
	unregister_pm_notifier(&pm_notify);
#endif
}
module_exit(xgold_pm_debug_exit);


MODULE_DESCRIPTION("power management debug driver");
MODULE_AUTHOR("Fino Xiangfu Meng <fino.xiangfu.meng@intel.com>");
MODULE_LICENSE("GPL V2");
