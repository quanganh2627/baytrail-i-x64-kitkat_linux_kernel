/*
 * Copyright (c) 2014, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/memblock.h>
#include <linux/bootmem.h>
#include <linux/pstore_ram.h>

#define SZ_4K 0x00001000
#define SZ_1M 0x00100000

#define PSTORE_RAM_SIZE_DEFAULT (4 * SZ_1M)

#ifdef CONFIG_X86_32
#define RAM_MAX_MEM (max_low_pfn << PAGE_SHIFT)
#else
#define RAM_MAX_MEM (1 << 28)
#endif

static struct ramoops_platform_data pstore_ram_data = {
	.record_size	= SZ_4K,
	.console_size	= 2*SZ_1M,
	.ftrace_size	= 2*SZ_4K,
	.dump_oops	= 1,
};

static struct platform_device pstore_ram_dev = {
	.name = "ramoops",
	.dev = {
		.platform_data = &pstore_ram_data,
	},
};

static __initdata bool pstore_ram_stand_inited;

static int __init pstore_ram_stand_register(void)
{
	int ret;



	if (!pstore_ram_stand_inited)
		return -ENODEV;

	ret = platform_device_register(&pstore_ram_dev);
	if (ret) {
		pr_err("%s: unable to register pstore_ram device: "
		       "start=0x%llx, size=0x%lx, ret=%d\n", __func__,
		       (unsigned long long)pstore_ram_data.mem_address,
		       pstore_ram_data.mem_size, ret);
	}
	return ret;
}
device_initcall(pstore_ram_stand_register);

void __init pstore_ram_stand_reserve_memory(void)
{
	phys_addr_t mem;
	size_t size;
	int ret;

	size = PSTORE_RAM_SIZE_DEFAULT;
	size = ALIGN(size, PAGE_SIZE);

	mem = memblock_find_in_range(0, RAM_MAX_MEM, size, PAGE_SIZE);
	if (!mem) {
		pr_err("Cannot find memblock range for pstore_ram\n");
		return;
	}

	ret = memblock_reserve(mem, size);
	if (ret) {
		pr_err("Failed to reserve memory from 0x%llx-0x%llx\n",
		       (unsigned long long)mem,
		       (unsigned long long)(mem + size - 1));
		return;
	}

	pstore_ram_data.mem_address = mem;
	pstore_ram_data.mem_size = size;

	pr_info("reserved RAM buffer (0x%zx@0x%llx)\n",
		size, (unsigned long long)mem);

	pstore_ram_stand_inited = true;
}
