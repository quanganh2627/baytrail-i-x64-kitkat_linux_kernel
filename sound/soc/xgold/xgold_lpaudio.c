/*
 * Component: XGOLD low power audio driver
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
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/amba/pl08x.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/kthread.h>
#include <asm/cacheflush.h>

#include "dsp_audio_platform.h"
#include "aud_lib_dsp_internal.h"
#include "dsp_audio_hal_internal.h"
#include "xgold_pcm.h"

#include "lpaudio.h"

extern struct dma_async_tx_descriptor *
	       (*lpaudio_dma_setup)(struct dma_chan *dmach);

struct device *lpaudio_dev;
static struct T_AUD_DSP_CMD_PCM_PLAY_PAR pcm_par = {
	.mode = 0,
	.rate = 8,
};
static phys_addr_t lpaudio_dmac_addr;
static __iomem void *lpaudio_dmac_base;
static int lpaudio_dmac_size;
static struct dma_chan *lpaudio_dmach;
static dma_addr_t lpaudio_dma_addr;
static dma_addr_t lpaudio_workbuf_addr;
static int lpaudio_enable_dma = 1;

static void lpaudio_buf_prepare(void)
{
	struct page *page;

	page = alloc_pages(GFP_KERNEL, get_order(DMA_TOTAL_SIZE));
	lpaudio_dma_addr = dma_map_page(NULL, page, 0,
			DMA_TOTAL_SIZE, DMA_TO_DEVICE);

	page = alloc_pages(GFP_KERNEL, get_order(WORK_BUF_SIZE));
	lpaudio_workbuf_addr = dma_map_page(NULL, page, 0,
			WORK_BUF_SIZE, DMA_TO_DEVICE);
}

static struct dma_async_tx_descriptor *dma_setup(struct dma_chan *dmach)
{
	struct dma_async_tx_descriptor *desc;

	memset(phys_to_virt(lpaudio_dma_addr), 0,
				DMA_BLOCK_SIZE * DMA_BLOCK_NUM);
	lpaudio_dmach = dmach;
	desc = lpaudio_dmach->device->device_prep_dma_cyclic(
				lpaudio_dmach,
				lpaudio_dma_addr,
				(DMA_BLOCK_SIZE * DMA_BLOCK_NUM),
				DMA_BLOCK_SIZE,
				DMA_SL_MEM_TO_MEM,
				0,
				NULL);
	return desc;
}

static u32 lpaudio_find_dma(void)
{
	int i;
	u32 ret = 0;
	u32 *cfg, *src;

	for (i = 0; i < 16; i++) {
		src = (u32 *)((char *)lpaudio_dmac_base + 0x100 + i*0x10);
		cfg = (u32 *)((char *)src + 0x10);
		if (((*cfg & 0x3df) == 0x141)
				&& (*src >= lpaudio_dma_addr
					&& *src <= lpaudio_dma_addr
					+ DMA_BLOCK_SIZE * DMA_BLOCK_NUM)) {
			ret = lpaudio_dmac_addr + 0x100 + i*0x10;
			break;
		}
	}
	return ret;
}

static int lpaudio_stop(void)
{
	if (lpaudio_enable_dma)
		dmaengine_terminate_all(lpaudio_dmach);
	else {
		pcm_par.setting = 0;
		dsp_audio_cmd(
				DSP_AUD_PCM2_PLAY,
				sizeof(struct T_AUD_DSP_CMD_PCM_PLAY_PAR),
				(u16 *)&pcm_par);
		dsp_audio_irq_deactivate(p_dsp_audio_dev, DSP_IRQ_1);
	}

	return 0;
}

static int lpaudio_start(void)
{
	pcm_par.setting = 1;
	if (lpaudio_enable_dma) {
		dma_async_issue_pending(lpaudio_dmach);
		pcm_par.req = 1; /*dma*/
	} else {
		pcm_par.req = 0; /*pio*/
		dsp_audio_cmd(
				DSP_AUD_PCM2_PLAY,
				sizeof(struct T_AUD_DSP_CMD_PCM_PLAY_PAR),
				(u16 *) &pcm_par);
		dsp_audio_irq_activate(p_dsp_audio_dev,
				DSP_IRQ_1);
	}
	return 0;
}

static int lpaudio_fs_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long start = vma->vm_start;
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long offset = vma->vm_pgoff;
	int i;
	unsigned long pfn;
	int page_num;

	if (LPAUDIO_MMAP_DMAC == offset) {
		pfn = DMA_CTRL_ADDR >> PAGE_SHIFT;
		page_num = 1;
	} else if (LPAUDIO_MMAP_DMA_MEM == offset) {
		pfn = lpaudio_dma_addr >> PAGE_SHIFT;
		page_num = (DMA_TOTAL_SIZE + (PAGE_SIZE - 1)) / PAGE_SIZE;
	} else if (LPAUDIO_MMAP_WORK_BUF == offset) {
		pfn = lpaudio_workbuf_addr >> PAGE_SHIFT;
		page_num = (WORK_BUF_SIZE + (PAGE_SIZE - 1)) / PAGE_SIZE;
	} else
		return -EINVAL;

	for (i = 0; i < page_num; i++) {
		if (size >= PAGE_SIZE) {
			if (remap_pfn_range(vma, start, pfn, PAGE_SIZE,
						vma->vm_page_prot))
				return -EINVAL;
			pfn++;
			start += PAGE_SIZE;
			size -= PAGE_SIZE;
		} else
			break;
	}

	return 0;
}

static long lpaudio_fs_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	unsigned short *shm;
	u32 ret;
	int i;

	dev_dbg(lpaudio_dev, "%s: cmd: %d\n", __func__, cmd);
	switch (cmd) {
	case LPAUDIO_IOCTRL_DSP_DL:
		shm = (void *)dsp_get_audio_shmem_base_addr() +
			OFFSET_SM_AUDIO_BUFFER_1_DL;
		ret = (u32)shm;
		break;
	case LPAUDIO_IOCTRL_DSP_UL:
		shm = (void *)dsp_get_audio_shmem_base_addr() +
			OFFSET_SM_AUDIO_BUFFER_UL;
		ret = (u32)shm;
		break;
	case LPAUDIO_IOCTRL_DSP_PLAY:
		pcm_par.setting = 2;
		dsp_audio_cmd(
			DSP_AUD_PCM1_PLAY,
			sizeof(struct T_AUD_DSP_CMD_PCM_PLAY_PAR),
			(u16 *) &pcm_par);
		break;
	case LPAUDIO_IOCTRL_DMAC:
		ret = lpaudio_find_dma();
		if (ret == 0)
			dev_err(lpaudio_dev,
				"%s: cannot find lpaudio dmach\n", __func__);
		break;
	case LPAUDIO_IOCTRL_DMAMEM:
		ret = lpaudio_dma_addr;
		break;
	case LPAUDIO_IOCTRL_WORKBUF:
		ret = lpaudio_workbuf_addr;
		break;
	case LPAUDIO_IOCTRL_DMASIZE:
		ret = DMA_BLOCK_SIZE * DMA_BLOCK_NUM;
		break;
	case LPAUDIO_IOCTRL_DMAPADDING:
		ret = DMA_PADDING_SIZE;
		break;
	case LPAUDIO_IOCTRL_WBUFSIZE:
		ret = WORK_BUF_SIZE;
		break;
	case LPAUDIO_IOCTRL_START:
		lpaudio_start();
		break;
	case LPAUDIO_IOCTRL_STOP:
		ret = 0;
		lpaudio_stop();
		break;
	case LPAUDIO_IOCTRL_ENABLE:
		dev_info(lpaudio_dev, "%s: enable lpaudio\n", __func__);
		lpaudio_dma_setup = dma_setup;
		break;
	case LPAUDIO_IOCTRL_DISABLE:
		dev_info(lpaudio_dev, "%s: disable lpaudio\n", __func__);
		lpaudio_dma_setup = NULL;
		break;
	default:
		return -EINVAL;
	}
	return copy_to_user((void __user *)arg, &ret, sizeof(ret));
}

static ssize_t lpaudio_fs_write(struct file *file, const char __user *user_buf,
		size_t count, loff_t *ppos)
{
	char kbuf[256];
	char cmd[256];
	int val;
	int i;

	memset(kbuf, 0, 256);
	if (copy_from_user(kbuf, user_buf, count > 256 ? 256 : count))
		return -EFAULT;

	i = sscanf(kbuf, "%s %d\n", cmd, &val);
	if (i != 1 && i != 2)
		return -EINVAL;
	if (!strcmp(cmd, "disable")) {
		lpaudio_dma_setup = NULL;
		dev_info(lpaudio_dev, "%s: disable lpaudio\n", __func__);
	} else if (!strcmp(cmd, "enable")) {
		lpaudio_dma_setup = dma_setup;
		dev_info(lpaudio_dev, "%s: ensable lpaudio\n", __func__);
	} else if (!strcmp(cmd, "pio")) {
		dev_info(lpaudio_dev, "%s: enable pio mode\n", __func__);
		lpaudio_enable_dma = 0;
	} else if (!strcmp(cmd, "dma")) {
		dev_info(lpaudio_dev, "%s: enable dma mode\n", __func__);
		lpaudio_enable_dma = 1;
	} else {
		dev_info(lpaudio_dev, "%s: unknown command\n", __func__);
	}

	return count;
}

static const struct file_operations lpaudio_fops = {
	.write		= lpaudio_fs_write,
	.mmap		= lpaudio_fs_mmap,
	.unlocked_ioctl	= lpaudio_fs_ioctl,
	.llseek		= no_llseek,
};

static struct miscdevice lpaudio_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lpaudio",
	.fops = &lpaudio_fops,
};

static int lpaudio_probe(struct platform_device *pdev)
{
	struct resource *res;

	lpaudio_dev = &pdev->dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!res)) {
		dev_err(lpaudio_dev, "%s: Invalid mem resource\n", __func__);
		return -ENODEV;
	}
	lpaudio_dmac_size = resource_size(res);
	lpaudio_dmac_addr = res->start;
	lpaudio_dmac_base = devm_ioremap(&pdev->dev,
			lpaudio_dmac_addr, lpaudio_dmac_size);
	lpaudio_buf_prepare();
	misc_register(&lpaudio_misc_dev);
	return 0;
}

static struct of_device_id lpaudio_of_match[] = {
	{ .compatible = "intel,xgold-lpaudio", },
	{ },
};

static struct platform_driver lpaudio_drv = {
	.driver = {
		.name = "AGOLD_LPAUDIO",
		.owner = THIS_MODULE,
		.of_match_table = lpaudio_of_match,
	},
	.probe = lpaudio_probe,
};

static int __init lpaudio_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&lpaudio_drv);
	if (ret < 0) {
		pr_err("%s : Unable to register lpaudio driver\n",
		__func__);
		return -ENODEV;
	}
	return ret;
}
module_init(lpaudio_init);

static void __exit lpaudio_exit(void)
{
	platform_driver_unregister(&lpaudio_drv);
}
module_exit(lpaudio_exit);

MODULE_DESCRIPTION("XGOLD Low Power Audio driver");
MODULE_AUTHOR("Yang, Bin <bin.yang@intel.com");
MODULE_LICENSE("GPL V2");
MODULE_DEVICE_TABLE(of, lpaudio_of_match);
