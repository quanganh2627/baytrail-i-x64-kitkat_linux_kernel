/*
 *
 *      Copyright (C) 2013-2014 Intel Corporation. All rights reserved.
 *
 *      This program is free software; you can redistribute it and/or
 *      modify it under the terms of version 2 of the GNU General
 *      Public License as published by the Free Software Foundation.
 *
 *      This program is distributed in the hope that it will be
 *      useful, but WITHOUT ANY WARRANTY; without even the implied
 *      warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 *      PURPOSE.  See the GNU General Public License for more details.
 *      The full GNU General Public License is included in this
 *      distribution in the file called COPYING.
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/atomic.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#ifdef CONFIG_X86_INTEL_SOFIA
#include <sofia/mv_svc_hypercalls.h>
#endif

#define PFX "intel_sofia_watchdog: "

struct intel_scu_watchdog_dev {
	struct miscdevice miscdev;
	struct mutex lock;
	unsigned int threshold;
	unsigned int enable;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dfs_wd;
	struct dentry *dfs_scu_wdt_enable;
	struct dentry *dfs_kwd;
	struct dentry *dfs_kwd_enable;
	struct dentry *dfs_kwd_trigger;
#endif /* CONFIG_DEBUG_FS */
} sofia_watchdog_device;

static int intel_sofia_watchdog_keepalive(void)
{
	mv_svc_watchdog_pet();

	return 0;
}

/*
 * /dev/watchdog handling
 */
static int intel_sofia_watchdog_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}


static ssize_t intel_sofia_watchdog_write(struct file *file,
		char const *data,
		size_t len,
		loff_t *ppos)
{
	mutex_lock(&sofia_watchdog_device.lock);

	if (sofia_watchdog_device.enable)
		intel_sofia_watchdog_keepalive();

	mutex_unlock(&sofia_watchdog_device.lock);

	return len;
}

static long intel_sofia_watchdog_ioctl(struct file *file,
		unsigned int cmd,
		unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int ret = 0;
	u32 __user *p = argp;
	u32 new_margin;

	mutex_lock(&sofia_watchdog_device.lock);

	switch (cmd) {
	case WDIOC_KEEPALIVE:
		ret = intel_sofia_watchdog_keepalive();
		break;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_margin, p)) {
			ret = -EFAULT;
			break;
		}
		sofia_watchdog_device.threshold = new_margin;
		ret = mv_svc_watchdog_enable(new_margin);
		if (ret == 0)
			sofia_watchdog_device.enable = 1;
		break;
	case WDIOC_GETTIMEOUT:
		ret =  put_user(sofia_watchdog_device.threshold, p);
		break;
	default:
		ret =  -ENOTTY;
	}

	mutex_unlock(&sofia_watchdog_device.lock);

	return ret;
}

static int intel_sofia_watchdog_release(struct inode *inode, struct file *file)
{
	/* This watchdog should not be closed, here just return and wait
	watchdog timeout */

	return 0;
}

static const struct file_operations intel_sofia_watchdog_fops = {
	.owner          = THIS_MODULE,
	.llseek         = no_llseek,
	.write          = intel_sofia_watchdog_write,
	.unlocked_ioctl = intel_sofia_watchdog_ioctl,
	.open           = intel_sofia_watchdog_open,
	.release        = intel_sofia_watchdog_release,
};


#ifdef CONFIG_DEBUG_FS
static int intel_sofia_watchdog_enable_debugfs_show(void *data, uint64_t *value)
{
	mutex_lock(&sofia_watchdog_device.lock);
	*value = sofia_watchdog_device.enable;
	mutex_unlock(&sofia_watchdog_device.lock);

	return 0;
}

static int intel_sofia_watchdog_enable_debugfs_set(void *data, uint64_t value)
{

	mutex_lock(&sofia_watchdog_device.lock);

	if (value != 0 && !sofia_watchdog_device.enable)
		mv_svc_watchdog_enable(sofia_watchdog_device.threshold);

	else if (value == 0 && sofia_watchdog_device.enable)
		mv_svc_watchdog_disable();

	sofia_watchdog_device.enable = !!value;

	mutex_unlock(&sofia_watchdog_device.lock);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(intel_sofia_watchdog_enable_attr,
		intel_sofia_watchdog_enable_debugfs_show,
		intel_sofia_watchdog_enable_debugfs_set,
		"%llu\n");


static int intel_sofia_watchdog_trigger_debugfs_set(void *data, uint64_t value)
{
	pr_debug(PFX "kwdt_trigger\n");
	BUG();

	return 0;

}
DEFINE_SIMPLE_ATTRIBUTE(kwd_trigger_attr,
		NULL,
		intel_sofia_watchdog_trigger_debugfs_set,
		"%llu\n");

static unsigned int vmm_scu_wdt_enable;

static int vmm_scu_watchdog_debugfs_show(void *data, uint64_t *value)
{
	*value = vmm_scu_wdt_enable;

	return 0;
}

static int vmm_scu_watchdog_debugfs_set(void *data, uint64_t value)
{
	vmm_scu_wdt_enable = !!value;
	mv_svc_scu_watchdog_switch(vmm_scu_wdt_enable);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(vmm_scu_wdt_attr,
		vmm_scu_watchdog_debugfs_show,
		vmm_scu_watchdog_debugfs_set,
		"%llu\n");

static int remove_debugfs_entries(void)
{
	struct intel_scu_watchdog_dev *dev = &sofia_watchdog_device;

	/* /sys/kernel/debug/watchdog */
	debugfs_remove_recursive(dev->dfs_wd);

	return 0;
}

static int create_debugfs_entries(void)
{
	struct intel_scu_watchdog_dev *dev = &sofia_watchdog_device;

	/* /sys/kernel/debug/watchdog */
	dev->dfs_wd = debugfs_create_dir("watchdog", NULL);
	if (!dev->dfs_wd) {
		pr_err(PFX "%s: Cannot create main dir\n", __func__);
		goto error;
	}


	/* /sys/kernel/debug/watchdog/vmm_scu_wdt_enable */
	dev->dfs_scu_wdt_enable = debugfs_create_file("vmm_scu_wdt_enable",
			0644, dev->dfs_wd, NULL, &vmm_scu_wdt_attr);
	if (!dev->dfs_scu_wdt_enable) {
		pr_err(PFX "%s: Cannot create vmm_scu_wdt_enable\n", __func__);
		goto error;
	}

	vmm_scu_wdt_enable = 1;

	/* /sys/kernel/debug/watchdog/kernel_watchdog */
	dev->dfs_kwd = debugfs_create_dir("kernel_watchdog", dev->dfs_wd);
	if (!dev->dfs_kwd) {
		pr_err(PFX "%s: Cannot create kwd dir\n", __func__);
		goto error;
	}

	/* /sys/kernel/debug/watchdog/kernel_watchdog/enable */
	dev->dfs_kwd_enable = debugfs_create_file("enable", 0644,
			dev->dfs_kwd, NULL, &intel_sofia_watchdog_enable_attr);
	if (!dev->dfs_kwd_enable) {
		pr_err(PFX "%s: Cannot create kwd enable file\n", __func__);
		goto error;
	}

	/* /sys/kernel/debug/watchdog/kernel_watchdog/trigger */
	dev->dfs_kwd_trigger = debugfs_create_file("trigger", 0220,
			dev->dfs_kwd, NULL, &kwd_trigger_attr);

	if (!dev->dfs_kwd_trigger) {
		pr_err(PFX "%s: Cannot create kwd trigger file\n", __func__);
		goto error;
	}


	return 0;
error:
	remove_debugfs_entries();
	return 1;

}
#endif

static int intel_sofia_watchdog_probe(struct platform_device *pdev)
{
	int ret;

	sofia_watchdog_device.miscdev.minor = 0;
	sofia_watchdog_device.miscdev.name = "watchdog";
	sofia_watchdog_device.miscdev.fops = &intel_sofia_watchdog_fops;
	sofia_watchdog_device.enable = 0;
	mutex_init(&sofia_watchdog_device.lock);

	ret = misc_register(&sofia_watchdog_device.miscdev);
	if (ret) {
		pr_err("cannot register miscdev %d err =%d\n",
				WATCHDOG_MINOR, ret);
		return ret;
	}

#ifdef CONFIG_DEBUG_FS
	create_debugfs_entries();
#endif
	return 0;
}


static int intel_sofia_watchdog_remove(struct platform_device *pdev)
{
	misc_deregister(&sofia_watchdog_device.miscdev);
#ifdef CONFIG_DEBUG_FS
	remove_debugfs_entries();
#endif
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int intel_sofia_watchdog_suspend(struct device *dev)
{
	/* need to disabe watchdog when system suspend, in case vmm is
	   still alive */
	if (sofia_watchdog_device.enable) {
		pr_debug(PFX "disable watchdog.\n");
		mv_svc_watchdog_disable();
	}

	return 0;
}

static int intel_sofia_watchdog_resume(struct device *dev)
{

	if (sofia_watchdog_device.enable) {
		pr_debug(PFX "enable watchdog.\n");
		mv_svc_watchdog_enable(sofia_watchdog_device.threshold);
	}

	return 0;
}
#endif

static const struct dev_pm_ops watchdog_dev_pm_ops = {

	SET_SYSTEM_SLEEP_PM_OPS(intel_sofia_watchdog_suspend,
			intel_sofia_watchdog_resume)
};

static struct platform_device watchdog_dev = {
	.name = "intel-sofia-watchdog",
	.id   = -1,
};

static struct platform_driver watchdog_driver = {
	.probe  = intel_sofia_watchdog_probe,
	.remove =  intel_sofia_watchdog_remove,
	.driver = {
		.name   = "intel-sofia-watchdog",
		.pm     = &watchdog_dev_pm_ops,
	},
};


static int __init intel_sofia_watchdog_init(void)
{
	int ret;
	ret = platform_device_register(&watchdog_dev);
	if (ret) {
		pr_err("[%s] register platform device failed.\n", __func__);
		return ret;
	}

	ret = platform_driver_register(&watchdog_driver);
	if (ret) {
		pr_err("[%s] register platform driver failed.\n", __func__);
		platform_device_unregister(&watchdog_dev);
	}

	return ret;
}

static void __exit intel_sofia_watchdog_exit(void)
{
	platform_driver_unregister(&watchdog_driver);
	platform_device_unregister(&watchdog_dev);
}

late_initcall(intel_sofia_watchdog_init);
module_exit(intel_sofia_watchdog_exit);

MODULE_AUTHOR("linx.z.chen@intel.com");
MODULE_DESCRIPTION("Intel Sofia Watchdog Device Driver");
MODULE_LICENSE("GPL");
