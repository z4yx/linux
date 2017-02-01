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
#include <linux/usb/sl811.h>

#include <asm/prom.h>

#if IS_ENABLED(CONFIG_THINPAD_SPANTAN6) || IS_ENABLED(CONFIG_THINPAD_NG_ARTIX7)
#define EARLY_PRINT_UART_DR	((uint32_t*)0xbfd003f8)
#define EARLY_PRINT_UART_SR	((uint32_t*)0xbfd003fc)
#elif IS_ENABLED(CONFIG_DE2I_CYCLONE4)
#define EARLY_PRINT_UART_BASE  0xbfd003e0
#endif

#if IS_ENABLED(CONFIG_USB_SL811_HCD)
static struct resource sl811_hcd_resources[] = {
	{
		.start = 0x1c020000,
		.end = 0x1c020000,
		.flags = IORESOURCE_MEM,
	}, {
		.start = 0x1c020004,
		.end = 0x1c020004,
		.flags = IORESOURCE_MEM,
	}, {
#if IS_ENABLED(CONFIG_DE2I_CYCLONE4)
		.start = 8+30,
		.end = 8+30,
#else
		.start = 5,
		.end = 5,
#endif
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	},
};

#if defined(CONFIG_USB_SL811_BFIN_USE_VBUS)
void sl811_port_power(struct device *dev, int is_on)
{
	// gpio_request(CONFIG_USB_SL811_BFIN_GPIO_VBUS, "usb:SL811_VBUS");
	// gpio_direction_output(CONFIG_USB_SL811_BFIN_GPIO_VBUS, is_on);
}
#endif

static struct sl811_platform_data sl811_priv = {
	.potpg = 10,
	.power = 250,       /* == 500mA */
#if defined(CONFIG_USB_SL811_BFIN_USE_VBUS)
	.port_power = &sl811_port_power,
#endif
};

static struct platform_device sl811_hcd_device = {
	.name = "sl811-hcd",
	.id = 0,
	.dev = {
		.platform_data = &sl811_priv,
	},
	.num_resources = ARRAY_SIZE(sl811_hcd_resources),
	.resource = sl811_hcd_resources,
};
#endif

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
#if IS_ENABLED(CONFIG_THINPAD_SPANTAN6) || IS_ENABLED(CONFIG_THINPAD_NG_ARTIX7)
	while(!(readl(EARLY_PRINT_UART_SR) & 1));
	writel(c, EARLY_PRINT_UART_DR);
#elif IS_ENABLED(CONFIG_DE2I_CYCLONE4)
	while(!(readl((uint32_t*)(EARLY_PRINT_UART_BASE+8)) & 0x40));
	writel(c, (uint32_t*)(EARLY_PRINT_UART_BASE+4));
#endif
}

void __init prom_init(void)
{
#if IS_ENABLED(CONFIG_THINPAD_SPANTAN6) || IS_ENABLED(CONFIG_THINPAD_NG_ARTIX7)
	writel(0, EARLY_PRINT_UART_SR);
#elif IS_ENABLED(CONFIG_DE2I_CYCLONE4)
	writel(0, (uint32_t*)(EARLY_PRINT_UART_BASE+0xc));
#endif
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

#ifdef CONFIG_USB_SL811_HCD
	platform_device_register(&sl811_hcd_device);
#endif

	return 0;
}
arch_initcall(plat_of_setup);
