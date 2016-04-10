/*
 * Thinpad platform setup
 *
 * Copyright (C) 2016 Tsinghua University
 * Author: Zhang Yuxiang <zz593141477@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/of_fdt.h>
#include <linux/of_platform.h>
#include <linux/serial_8250.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/platform_device.h>

#include <asm/prom.h>

#define EARLY_PRINT_UART_BASE	0xbfd003f8

const char *get_system_type(void)
{
	return "Thinpad";
}

void __init plat_mem_setup(void)
{
	__dt_setup_arch(__dtb_start);
	strlcpy(arcs_cmdline, boot_command_line, COMMAND_LINE_SIZE);
}

void __init prom_init(void)
{
	setup_8250_early_printk_port(EARLY_PRINT_UART_BASE, 2, 50000);
}

void __init prom_free_prom_memory(void)
{
}

void __init device_tree_init(void)
{
	if (!initial_boot_params)
		return;

	unflatten_and_copy_device_tree();
}

static int __init plat_of_setup(void)
{
	if (!of_have_populated_dt())
		panic("Device tree not present");

	if (of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL))
		panic("Failed to populate DT");

	return 0;
}
arch_initcall(plat_of_setup);


static struct plat_serial8250_port uart8250_data[] = {
	{
		.mapbase	= 0x1fd003F8,	
		.irq		= 4,
		.uartclk	= 1843200,
		.iotype		= UPIO_MEM,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST | UPF_IOREMAP,
		// .regshift	= 2,
	},
	{ },
};

static struct platform_device thinpad_uart8250_device = {
	.name			= "serial8250",
	.id			= PLAT8250_DEV_PLATFORM,
	.dev			= {
		.platform_data	= uart8250_data,
	},
};

static struct platform_device *thinpad_devices[] __initdata = {
	&thinpad_uart8250_device,
};

static int __init thinpad_add_devices(void)
{
	int err;

	err = platform_add_devices(thinpad_devices, ARRAY_SIZE(thinpad_devices));
	if (err)
		return err;

	return 0;
}

device_initcall(thinpad_add_devices);
