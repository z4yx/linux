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

#define EARLY_PRINT_UART_DR	((uint32_t*)0xbfd003f8)
#define EARLY_PRINT_UART_SR	((uint32_t*)0xbfd003fc)

const char *get_system_type(void)
{
	return "Thinpad";
}

void __init plat_mem_setup(void)
{
	__dt_setup_arch(__dtb_start);
	strlcpy(arcs_cmdline, boot_command_line, COMMAND_LINE_SIZE);
}

void prom_putchar(char c)
{
	while(!(readl(EARLY_PRINT_UART_SR) & 1));
	writel(c, EARLY_PRINT_UART_DR);
}

void __init prom_init(void)
{
	writel(0, EARLY_PRINT_UART_SR);
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
