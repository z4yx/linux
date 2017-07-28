/*
 * NaiveMIPS interrupt controller setup
 *
 * Copyright (C) 2017 Tsinghua Univ.
 * Author: Yuxiang Zhang
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
#include <linux/irqchip.h>

#include <asm/irq_cpu.h>

void __init arch_init_irq(void)
{
    pr_devel("arch_init_irq\n");
	// of_irq_init(of_irq_ids);
    irqchip_init();
}

IRQCHIP_DECLARE(mips_cpu_intc, "mti,cpu-interrupt-controller",
         mips_cpu_irq_of_init);