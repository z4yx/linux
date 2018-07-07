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
#include <linux/usb/isp1362.h>

#include <asm/prom.h>

#if IS_ENABLED(CONFIG_THINPAD_SPANTAN6) || IS_ENABLED(CONFIG_THINPAD_NG_ARTIX7) || IS_ENABLED(CONFIG_FPGA_K7_EES)
#define EARLY_PRINT_UART_DR	((uint32_t*)0xbfd003f8)
#define EARLY_PRINT_UART_SR	((uint32_t*)0xbfd003fc)
#elif IS_ENABLED(CONFIG_DE2I_CYCLONE4)
#define EARLY_PRINT_UART_BASE  0xbfd003e0
#elif IS_ENABLED(CONFIG_FPGA_A7_NSCSCC)
#define EARLY_PRINT_UART_BASE  0xbfd03000
#endif

#if IS_ENABLED(CONFIG_USB_SL811_HCD) || IS_ENABLED(CONFIG_USB_ISP1362_HCD)
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
//#if IS_ENABLED(CONFIG_USB_SL811_HCD)
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
//#else
//		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
//#endif
	},
};
#endif

#if IS_ENABLED(CONFIG_USB_SL811_HCD)
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
#endif //CONFIG_USB_SL811_HCD

#if IS_ENABLED(CONFIG_USB_ISP1362_HCD)
static struct isp1362_platform_data isp1362_priv = {
	.sel15Kres = 1,
	.clknotstop = 0,
	.oc_enable = 0,
	.int_act_high = 1,
	.int_edge_triggered = 0,
	.remote_wakeup_connected = 0,
	.no_power_switching = 1,
	.power_switching_mode = 0,
};

static struct platform_device isp1362_hcd_device = {
	.name = "isp1362-hcd",
	.id = 0,
	.dev = {
		.platform_data = &isp1362_priv,
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
#if IS_ENABLED(CONFIG_THINPAD_SPANTAN6) || IS_ENABLED(CONFIG_THINPAD_NG_ARTIX7) || IS_ENABLED(CONFIG_FPGA_K7_EES)
	while(!(readl(EARLY_PRINT_UART_SR) & 1));
	writel(c, EARLY_PRINT_UART_DR);
#elif IS_ENABLED(CONFIG_DE2I_CYCLONE4)
	while(!(readl((uint32_t*)(EARLY_PRINT_UART_BASE+8)) & 0x40));
	writel(c, (uint32_t*)(EARLY_PRINT_UART_BASE+4));
#elif IS_ENABLED(CONFIG_FPGA_A7_NSCSCC)
	while(!(readl((uint32_t*)(EARLY_PRINT_UART_BASE+0x14)) & 0x20)); //LSR
	writel(c, (uint32_t*)(EARLY_PRINT_UART_BASE+0));
#endif
}

void __init prom_init(void)
{
#if IS_ENABLED(CONFIG_THINPAD_SPANTAN6) || IS_ENABLED(CONFIG_THINPAD_NG_ARTIX7) || IS_ENABLED(CONFIG_FPGA_K7_EES)
	writel(0, EARLY_PRINT_UART_SR);
#elif IS_ENABLED(CONFIG_DE2I_CYCLONE4)
	writel(0, (uint32_t*)(EARLY_PRINT_UART_BASE+0xc));
#elif IS_ENABLED(CONFIG_FPGA_A7_NSCSCC)
	//Uart16550 initialize
	writel(0, (uint32_t*)(EARLY_PRINT_UART_BASE+8)); //Turn off FIFO
	writel(0x80, (uint32_t*)(EARLY_PRINT_UART_BASE+0xc)); //DLAB=1
	writel(54, (uint32_t*)(EARLY_PRINT_UART_BASE+0)); //DLL=54=100000000/(16*115200)
	writel(0, (uint32_t*)(EARLY_PRINT_UART_BASE+4)); //DLM=0
	writel(3, (uint32_t*)(EARLY_PRINT_UART_BASE+0xc)); //DLAB=0,8N1 Mode
	writel(0, (uint32_t*)(EARLY_PRINT_UART_BASE+4)); //IER=0
	writel(0, (uint32_t*)(EARLY_PRINT_UART_BASE+0x10)); //MCR=0
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
#if IS_ENABLED(CONFIG_USB_ISP1362_HCD)
	platform_device_register(&isp1362_hcd_device);
#endif

	return 0;
}
arch_initcall(plat_of_setup);
