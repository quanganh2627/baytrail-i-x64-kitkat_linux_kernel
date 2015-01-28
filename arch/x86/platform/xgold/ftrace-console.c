/*intel mid ram console support
 *
 * Copyright (C) 2012 Motorola Mobility, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/console.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/io.h>

#include <linux/platform_device.h>
#include "ftrace-console.h"
#include <linux/memblock.h>

#include <linux/slab.h>

struct ram_console_buffer {
	uint32_t    sig;
	uint32_t    start;
	uint32_t    size;
	uint8_t     data[0];
};

struct ftrace_reserved_buffer {
	uint32_t  sig;
	uint32_t  start;
	uint32_t  size;
	uint8_t   data[0];
};

#define FTRACE_BUFFER_SIG (0x23456789)

static char *ftrace_old_buffer;
static size_t ftrace_old_buffer_size;
static struct ftrace_reserved_buffer *ftrace_reserved_buffer;
static size_t ftrace_reserved_buffer_size;

static void ftrace_reserved_buffer_init(
	struct ftrace_reserved_buffer *ftrace_buffer,
	size_t ftrace_buffer_size);



static struct resource ram_console_resources[] = {
	{
		.flags  = IORESOURCE_MEM,
		.start  = INTEL_FTRACE_BUFFER_START_DEFAULT,
		.end    = INTEL_FTRACE_BUFFER_START_DEFAULT +
			  INTEL_FTRACE_BUFFER_SIZE_DEFAULT - 1,
	},
};

static struct ram_console_platform_data ram_console_pdata;

static struct platform_device ram_console_device = {
	.name		= "ram_console",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ram_console_resources),
	.resource	= ram_console_resources,
	.dev		= {.platform_data = &ram_console_pdata,	},
};


/**
 * intel_mid_ram_console_register() - device_initcall to register ramconsole device
 */
static int __init intel_mid_ram_console_register(void)
{
	int ret = 0;
	size_t start;
	size_t buffer_size;
	void *buffer;
	size_t ftrace_buffer_size;
	void *ftrace_buffer;
	const char *bootinfo = NULL;

	pr_info("enter into intel_mid_ram_console_register\n");
	ftrace_buffer_size = ram_console_resources[0].end -
				ram_console_resources[0].start + 1;
	ftrace_buffer = ioremap(ram_console_resources[0].start,
				ftrace_buffer_size);
	if (ftrace_buffer == NULL) {
		pr_err("ftrace_reserved_buffer:  ioreamp failed.\n");
		return -ENOMEM;
	}
	ftrace_old_buffer_size = 0;
	ftrace_reserved_buffer_init(ftrace_buffer, ftrace_buffer_size);
	pr_info("ftrace_reserved_buffer:  ioreamp ok, ftrace_buffer %08x,
		res->start %08x\n", ftrace_buffer,
		ram_console_resources[0].start);

	return ret;
}
device_initcall(intel_mid_ram_console_register);

void __init ram_consle_reserve_memory(void)
{
	phys_addr_t mem;
	size_t size;

	size = INTEL_FTRACE_BUFFER_SIZE_DEFAULT;
	size = ALIGN(size, PAGE_SIZE);
	mem = memblock_find_in_range(0, 1<<28, size, PAGE_SIZE);
	if (!mem)
		pr_err("Cannot allocate ram_consle_reserve_memory step2\n");

	ram_console_resources[0].start = mem;
	ram_console_resources[0].end = mem + size - 1;
	memblock_reserve(mem, size);
}

static void ftrace_reserved_buffer_init(
		struct ftrace_reserved_buffer *ftrace_buffer,
		size_t ftrace_buffer_size)
{
	ftrace_reserved_buffer = ftrace_buffer;
	ftrace_reserved_buffer_size = ftrace_buffer_size -
					sizeof(struct ftrace_reserved_buffer);
	pr_info("%s: ftrace_buffer->size %d, ftrace_buffer_size %d,\n",
		__func__,
		ftrace_buffer->size, ftrace_buffer_size);

	if (ftrace_reserved_buffer_size > ftrace_buffer_size) {
		pr_err("%s: buffer %p, invalid size %zu, datasize %zu\n",
			__func__,
			ftrace_buffer, ftrace_buffer_size,
			ftrace_reserved_buffer_size);
		return;
	}

	if (ftrace_buffer->sig == FTRACE_BUFFER_SIG) {
		if (ftrace_buffer->size > ftrace_reserved_buffer_size ||
				ftrace_buffer->start > ftrace_buffer->size)
			pr_info("%s: found existing invalid buffer", __func__);
		else {
			/* save old buffer */
			ftrace_old_buffer = vmalloc(ftrace_buffer->size);
			if (!ftrace_old_buffer) {
				pr_err("%s: vmalloc failed, ftrace_buffer->size %d\n",
					__func__, ftrace_buffer->size);

			} else {
				pr_info("ftrace_old_buffer alloc ok , size %d , start %08x\n",
					ftrace_buffer->size,
					ftrace_buffer->start);
				ftrace_old_buffer_size = ftrace_buffer->size;
/* if ring buffer is not overlap,
ftrace_buffer->start equals ftrace_buffer->size
*/
				memcpy(ftrace_old_buffer,
				&ftrace_buffer->data[ftrace_buffer->start],
				ftrace_buffer->size - ftrace_buffer->start);
				memcpy(ftrace_old_buffer + ftrace_buffer->size -
				ftrace_buffer->start,
				&ftrace_buffer->data[0],
				ftrace_buffer->start);
			}
		}
	}
	ftrace_buffer->sig = FTRACE_BUFFER_SIG;
	ftrace_buffer->start = 0;
	ftrace_buffer->size = 0;

}
static void ftrace_buffer_update(char *s, unsigned int count)
{
	struct ftrace_reserved_buffer *buffer = ftrace_reserved_buffer;

	memcpy(buffer->data + buffer->start, s, count);
}

ssize_t ftrace_reserved_buffer_write(const char *s, ssize_t count)
{
	int rem;
	struct ftrace_reserved_buffer *buffer = ftrace_reserved_buffer;

	if (count > ftrace_reserved_buffer_size) {
		s += count - ftrace_reserved_buffer_size;
		count = ftrace_reserved_buffer_size;
	}
	rem = ftrace_reserved_buffer_size - buffer->start;
	/* ring buffer */
	if (rem < count) {
		ftrace_buffer_update(s, rem);
		s += rem;
		count -= rem;
		buffer->start = 0;
		buffer->size = ftrace_reserved_buffer_size;

	}

	ftrace_buffer_update(s, count);

	buffer->start += count;
	if (buffer->size < ftrace_reserved_buffer_size)
		buffer->size += count;
	return 0;
}
EXPORT_SYMBOL_GPL(ftrace_reserved_buffer_write);

ssize_t ftrace_reserved_buffer_read_old(struct file *filp,
		const char __user *buf,
		unsigned long len, loff_t *offset)
{

	ssize_t count;
	loff_t pos = *offset;

	if (pos >= ftrace_old_buffer_size || ftrace_old_buffer_size == 0)
		return 0;
	count = min(len, (size_t)(ftrace_old_buffer_size - pos));

	if (copy_to_user(buf, ftrace_old_buffer + pos, count))
		return -EFAULT;

	*offset += count;

	return count;
}


static const struct file_operations ftrace_reserved_buffer_file_ops = {
	.owner = THIS_MODULE,
	.read = ftrace_reserved_buffer_read_old,
};

static int __init ram_console_late_init(void)
{
	struct proc_dir_entry *entry;
	pr_info("ram_console_late_init\n");

	if (ftrace_old_buffer == NULL)
		return 0;
	entry = proc_create("last_trace", S_IFREG | S_IRUGO, NULL,
				&ftrace_reserved_buffer_file_ops);
	if (!entry) {
		pr_err("ftrace_reserved: failed to create proc entry\n");
		vfree(ftrace_old_buffer);
		ftrace_old_buffer = NULL;
		return 0;
	}
	pr_info("ftrace_reserved:create proc entry\n");

	return 0;
}
late_initcall(ram_console_late_init);
