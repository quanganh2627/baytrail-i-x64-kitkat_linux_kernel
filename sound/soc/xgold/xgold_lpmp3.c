/*
 * Component: XGOLD lpmp3 driver
 *
 * Copyright (C) 2014, Intel Mobile Communications GmbH.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * You should have received a copy of the GNU General Public License Version 2
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Contributor(s):
 * Yang, Bin
 */

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/amba/pl08x.h>
#include <linux/miscdevice.h>
#include <linux/wakelock.h>
#include <asm/cacheflush.h>
#include "sofia/lpmp3.h"

static struct wake_lock lpmp3_wakelock;
static int lpmp3_mode;
static struct device *lpmp3_dev;
static phys_addr_t lpmp3_dmac_addr;
static __iomem void *lpmp3_dmac_base;
static int lpmp3_dmac_size;
static struct dma_chan *lpmp3_dmach;
static dma_addr_t lpmp3_dma_addr;
static dma_addr_t lpmp3_ctrl_addr;
struct lpmp3_ctrl_t *lpmp3_ctrl;

int xgold_lpmp3_mode(void)
{
	return lpmp3_mode;
}

static void lpmp3_buf_prepare(void)
{
	struct page *page;

	page = alloc_pages(GFP_KERNEL, get_order(LPMP3_DMA_TOTAL_SIZE));
	lpmp3_dma_addr = dma_map_page(NULL, page, 0,
			LPMP3_DMA_TOTAL_SIZE, DMA_TO_DEVICE);

	page = alloc_pages(GFP_KERNEL, get_order(sizeof(*lpmp3_ctrl)));
	lpmp3_ctrl_addr = dma_map_page(NULL, page, 0,
			sizeof(*lpmp3_ctrl), DMA_TO_DEVICE);
}

static u32 lpmp3_find_dma(void)
{
	int i;
	u32 ret = 0;
	u32 *cfg, *src;

	for (i = 0; i < 16; i++) {
		src = (u32 *)((char *)lpmp3_dmac_base + 0x100 + i*0x10);
		cfg = (u32 *)((char *)src + 0x10);
		if (((*cfg & 0x3df) == 0x141)
				&& (*src >= lpmp3_dma_addr
				&& *src <= lpmp3_dma_addr
				+ LPMP3_DMA_BLOCK_SIZE * LPMP3_DMA_BLOCK_NUM)) {
			ret = lpmp3_dmac_addr + 0x100 + i*0x10;
			break;
		}
	}
	return ret;
}

static struct dma_async_tx_descriptor *dma_setup(struct dma_chan *dmach)
{
	struct dma_async_tx_descriptor *desc;

	dev_info(lpmp3_dev, "%s\n", __func__);
	lpmp3_dmach = dmach;
	desc = lpmp3_dmach->device->device_prep_dma_cyclic(
				lpmp3_dmach,
				lpmp3_dma_addr,
				(LPMP3_DMA_BLOCK_SIZE * LPMP3_DMA_BLOCK_NUM),
				LPMP3_DMA_BLOCK_SIZE,
				DMA_SL_MEM_TO_MEM,
				0,
				NULL);
	memset(phys_to_virt(lpmp3_dma_addr), 0,
			LPMP3_DMA_BLOCK_SIZE * LPMP3_DMA_BLOCK_NUM);
	lpmp3_mode = 1;
	return desc;
}

static void dma_release(struct dma_chan *dmach)
{
	dev_info(lpmp3_dev, "%s\n", __func__);
	lpmp3_mode = 0;
}

static void dma_trigger(int start)
{
	dev_info(lpmp3_dev, "%s, %s\n", __func__, start ? "START" : "STOP");
	if (start)
		lpmp3_ctrl->dma_ptr = (u32 *)lpmp3_find_dma();
}

static int lpmp3_stop(void)
{
	return 0;
}

static int lpmp3_start(void)
{
	return 0;
}

static int lpmp3_do_mmap(unsigned long start, struct vm_area_struct *vma,
		unsigned long pfn, int page_num)
{
	int i;

	for (i = 0; i < page_num; i++) {
		if (remap_pfn_range(vma, start, pfn, PAGE_SIZE,
					vma->vm_page_prot))
			return -EINVAL;
		pfn++;
		start += PAGE_SIZE;
	}
	return 0;
}

static int lpmp3_fs_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long start = vma->vm_start;
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long offset = vma->vm_pgoff;
	int i;
	unsigned long pfn;
	int page_num = (size + PAGE_SIZE - 1) / PAGE_SIZE;
	int ret = -EINVAL;

	if (LPMP3_MMAP_DMA_MEM == offset) {
		pfn = lpmp3_dma_addr >> PAGE_SHIFT;
		ret = lpmp3_do_mmap(start, vma, pfn, page_num);
	} else if (LPMP3_MMAP_CTRL == offset) {
		pfn = lpmp3_ctrl_addr >> PAGE_SHIFT;
		ret = lpmp3_do_mmap(start, vma, pfn, page_num);
	} else if (LPMP3_MMAP_CTRL == offset) {
		pfn = lpmp3_ctrl_addr >> PAGE_SHIFT;
		ret = lpmp3_do_mmap(start, vma, pfn, page_num);
	} else if (LPMP3_MMAP_DATA == offset) {
		for (i = 0; i < page_num; i++) {
			pfn = lpmp3_ctrl->data_pages[i] >> PAGE_SHIFT;
			ret = lpmp3_do_mmap(start, vma, pfn, 1);
			if (ret)
				break;
			start += PAGE_SIZE;
		}
	}

	return ret;
}

static long lpmp3_fs_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	u32 ret;

	dev_dbg(lpmp3_dev, "%s: cmd: %x\n", __func__, cmd);
	switch (cmd) {
	case LPMP3_IOCTRL_START:
		dev_info(lpmp3_dev, "%s: lpmp3 start\n", __func__);
		ret = lpmp3_start();
		break;
	case LPMP3_IOCTRL_STOP:
		dev_info(lpmp3_dev, "%s: lpmp3 stop\n", __func__);
		ret = lpmp3_stop();
		break;
	case LPMP3_IOCTRL_ENABLE:
		dev_info(lpmp3_dev, "%s: enable lpmp3\n", __func__);
		lpmp3_dma_setup = dma_setup;
		lpmp3_dma_release = dma_release;
		lpmp3_trigger = dma_trigger;
		break;
	case LPMP3_IOCTRL_DISABLE:
		dev_info(lpmp3_dev, "%s: disable lpmp3\n", __func__);
		lpmp3_dma_setup = NULL;
		lpmp3_dma_release = NULL;
		lpmp3_trigger = NULL;
		break;
	default:
		return -EINVAL;
	}
	return copy_to_user((void __user *)arg, &ret, sizeof(ret));
}

static int lpmp3_fs_open(struct inode *inode, struct file *file)
{
	int i;
	struct page *page;

	if (lpmp3_ctrl->open)
		return -EBUSY;
	for (i = 0; i < LPMP3_DATA_SIZE/PAGE_SIZE; i++) {
		page = alloc_page(GFP_KERNEL);
		if (!page)
			goto err;
		lpmp3_ctrl->data_pages[i] = page_to_phys(page);
	}
	lpmp3_ctrl->ctrl_addr = lpmp3_ctrl_addr;
	lpmp3_ctrl->data_page_num = LPMP3_DATA_SIZE/PAGE_SIZE;
	lpmp3_ctrl->output_addr = lpmp3_dma_addr;
	lpmp3_ctrl->output_len = LPMP3_DMA_BLOCK_SIZE * LPMP3_DMA_BLOCK_NUM;
	lpmp3_ctrl->output_padding_len = LPMP3_DMA_PADDING_SIZE;
	lpmp3_ctrl->open = 1;
	return 0;
err:
	pr_err("%s: alloc input buf error\n", __func__);
	for (i = 0; i < LPMP3_DATA_SIZE/PAGE_SIZE; i++) {
		if (lpmp3_ctrl->data_pages[i] != 0) {
			free_page((unsigned long)phys_to_virt(
						lpmp3_ctrl->data_pages[i]));
			lpmp3_ctrl->data_pages[i] = 0;
		}
	}
	return -ENOMEM;
}

static int lpmp3_fs_close(struct inode *inode, struct file *file)
{
	int i;

	lpmp3_ctrl->open = 0;
	while (LPMP3_CTRL_STOP != lpmp3_ctrl->play_status) {
		dev_err(lpmp3_dev, "%s: force lpmp3 stop\n", __func__);
		lpmp3_ctrl->play_ctrl = LPMP3_CTRL_STOP;
		msleep(100);
	}
	for (i = 0; i < LPMP3_DATA_SIZE/PAGE_SIZE; i++) {
		if (lpmp3_ctrl->data_pages[i] != 0) {
			free_page((unsigned long)phys_to_virt(
						lpmp3_ctrl->data_pages[i]));
			lpmp3_ctrl->data_pages[i] = 0;
		}
	}
	return 0;
}

static const struct file_operations lpmp3_fops = {
	.mmap		= lpmp3_fs_mmap,
	.unlocked_ioctl	= lpmp3_fs_ioctl,
	.llseek		= no_llseek,
	.open		= lpmp3_fs_open,
	.release	= lpmp3_fs_close,
};

static struct miscdevice lpmp3_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lpmp3",
	.fops = &lpmp3_fops,
};

static int lpmp3_probe(struct platform_device *pdev)
{
	struct resource *res;

	lpmp3_dev = &pdev->dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!res)) {
		dev_err(lpmp3_dev, "%s: Invalid mem resource\n", __func__);
		return -ENODEV;
	}
	lpmp3_dmac_size = resource_size(res);
	lpmp3_dmac_addr = res->start;
	lpmp3_dmac_base = devm_ioremap(&pdev->dev,
			lpmp3_dmac_addr, lpmp3_dmac_size);
	lpmp3_buf_prepare();

	lpmp3_ctrl = phys_to_virt(lpmp3_ctrl_addr);
	memset(lpmp3_ctrl, 0, sizeof(*lpmp3_ctrl));

	wake_lock_init(&lpmp3_wakelock, WAKE_LOCK_SUSPEND, "lpmp3_wakelock");
	misc_register(&lpmp3_misc_dev);
	return 0;
}

static int lpmp3_suspend(struct device *dev)
{
	return 0;
}

static int lpmp3_resume(struct device *dev)
{
	if (xgold_lpmp3_mode())
		wake_lock_timeout(&lpmp3_wakelock, 3 * HZ);
	return 0;
}

static const struct dev_pm_ops lpmp3_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(lpmp3_suspend, lpmp3_resume)
};

static struct of_device_id lpmp3_of_match[] = {
	{ .compatible = "intel,xgold-lpmp3", },
	{ },
};

static struct platform_driver lpmp3_drv = {
	.driver = {
		.name = "XGOLD_LPMP3",
		.owner = THIS_MODULE,
		.of_match_table = lpmp3_of_match,
		.pm = &lpmp3_pm,
	},
	.probe = lpmp3_probe,
};

static int __init lpmp3_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&lpmp3_drv);
	if (ret < 0) {
		pr_err("%s : Unable to register lpmp3 driver\n",
		__func__);
		return -ENODEV;
	}
	return ret;
}
module_init(lpmp3_init);

static void __exit lpmp3_exit(void)
{
	platform_driver_unregister(&lpmp3_drv);
}
module_exit(lpmp3_exit);

MODULE_DESCRIPTION("XGOLD Low Power Audio driver");
MODULE_AUTHOR("Yang, Bin <bin.yang@intel.com");
MODULE_LICENSE("GPL V2");
MODULE_DEVICE_TABLE(of, lpmp3_of_match);
