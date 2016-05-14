/*
 * Copyright (C) NaiveMIPS Team 2016
 * Author:  Zhang Yuxiang <zz593141477@gmail.com>
 * License terms:  GNU General Public License (GPL), version 2
 *
 */

#if defined(CONFIG_SERIAL_NAIVEMIPS_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/serial_core.h>
#include <linux/clk.h>

/* Register offsets */
#define USART_DR        0x00
#define USART_SR        0x04

/* USART_SR */
#define USART_SR_TXE        BIT(0)
#define USART_SR_RXNE       BIT(1)
#define USART_SR_ORE        BIT(2)
#define USART_SR_TXEIE      BIT(3)
#define USART_SR_RXNEIE     BIT(4)
#define USART_SR_DUMMY_RX   BIT(16)

/* USART_DR */
#define USART_DR_MASK       GENMASK(7, 0)

#define DRIVER_NAME "naivemips-uart"
#define NAIVEMIPS_SERIAL_NAME "ttyS"
#define NAIVEMIPS_MAX_PORTS 1

struct naivemips_port {
    struct uart_port port;
    // struct clk *clk;
};

static struct naivemips_port naivemips_ports[NAIVEMIPS_MAX_PORTS];
static struct uart_driver naivemips_usart_driver;

static void naivemips_stop_tx(struct uart_port *port);

static inline struct naivemips_port *to_naivemips_port(struct uart_port *port)
{
    return container_of(port, struct naivemips_port, port);
}

static void naivemips_set_bits(struct uart_port *port, u32 reg, u32 bits)
{
    u32 val;

    val = readl_relaxed(port->membase + reg);
    val |= bits;
    writel_relaxed(val, port->membase + reg);
}

static void naivemips_clr_bits(struct uart_port *port, u32 reg, u32 bits)
{
    u32 val;

    val = readl_relaxed(port->membase + reg);
    val &= ~bits;
    writel_relaxed(val, port->membase + reg);
}

static void naivemips_receive_chars(struct uart_port *port)
{
    struct tty_port *tport = &port->state->port;
    unsigned long c;
    u32 sr;
    char flag;

    if (port->irq_wake)
        pm_wakeup_event(tport->tty->dev, 0);

    while ((sr = readl_relaxed(port->membase + USART_SR)) & USART_SR_RXNE) {
        sr |= USART_SR_DUMMY_RX;
        c = readl_relaxed(port->membase + USART_DR);
        flag = TTY_NORMAL;
        port->icount.rx++;

        if (uart_handle_sysrq_char(port, c))
            continue;
        uart_insert_char(port, sr, USART_SR_ORE, c, flag);
    }

    spin_unlock(&port->lock);
    tty_flip_buffer_push(tport);
    spin_lock(&port->lock);
}

static void naivemips_transmit_chars(struct uart_port *port)
{
    struct circ_buf *xmit = &port->state->xmit;

    if (port->x_char) {
        writel_relaxed(port->x_char, port->membase + USART_DR);
        port->x_char = 0;
        port->icount.tx++;
        return;
    }

    if (uart_tx_stopped(port)) {
        naivemips_stop_tx(port);
        return;
    }

    if (uart_circ_empty(xmit)) {
        naivemips_stop_tx(port);
        return;
    }

    writel_relaxed(xmit->buf[xmit->tail], port->membase + USART_DR);
    xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
    port->icount.tx++;

    if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
        uart_write_wakeup(port);

    if (uart_circ_empty(xmit))
        naivemips_stop_tx(port);
}

static irqreturn_t naivemips_interrupt(int irq, void *ptr)
{
    struct uart_port *port = ptr;
    u32 sr;

    spin_lock(&port->lock);

    sr = readl_relaxed(port->membase + USART_SR);

    if (sr & USART_SR_RXNE)
        naivemips_receive_chars(port);

    if (sr & USART_SR_TXE)
        naivemips_transmit_chars(port);

    spin_unlock(&port->lock);

    return IRQ_HANDLED;
}

static unsigned int naivemips_tx_empty(struct uart_port *port)
{
    return readl_relaxed(port->membase + USART_SR) & USART_SR_TXE;
}

static void naivemips_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static unsigned int naivemips_get_mctrl(struct uart_port *port)
{
    /* This routine is used to get signals of: DCD, DSR, RI, and CTS */
    return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
}

/* Transmit stop */
static void naivemips_stop_tx(struct uart_port *port)
{
    naivemips_clr_bits(port, USART_SR, USART_SR_TXEIE);
}

/* There are probably characters waiting to be transmitted. */
static void naivemips_start_tx(struct uart_port *port)
{
    struct circ_buf *xmit = &port->state->xmit;

    if (uart_circ_empty(xmit))
        return;

    naivemips_set_bits(port, USART_SR, USART_SR_TXEIE);
}

/* Throttle the remote when input buffer is about to overflow. */
static void naivemips_throttle(struct uart_port *port)
{
    unsigned long flags;

    spin_lock_irqsave(&port->lock, flags);
    naivemips_clr_bits(port, USART_SR, USART_SR_RXNEIE);
    spin_unlock_irqrestore(&port->lock, flags);
}

/* Unthrottle the remote, the input buffer can now accept data. */
static void naivemips_unthrottle(struct uart_port *port)
{
    unsigned long flags;

    spin_lock_irqsave(&port->lock, flags);
    naivemips_set_bits(port, USART_SR, USART_SR_RXNEIE);
    spin_unlock_irqrestore(&port->lock, flags);
}

/* Receive stop */
static void naivemips_stop_rx(struct uart_port *port)
{
    naivemips_clr_bits(port, USART_SR, USART_SR_RXNEIE);
}

/* Handle breaks - ignored by us */
static void naivemips_break_ctl(struct uart_port *port, int break_state)
{
}

static int naivemips_startup(struct uart_port *port)
{
    const char *name = to_platform_device(port->dev)->name;
    u32 val;
    int ret;

    ret = request_irq(port->irq, naivemips_interrupt, 0, name, port);
    if (ret)
        return ret;

    val = USART_SR_TXEIE|USART_SR_RXNEIE;
    naivemips_set_bits(port, USART_SR, val);

    return 0;
}

static void naivemips_shutdown(struct uart_port *port)
{
    u32 val;

    val = USART_SR_TXEIE | USART_SR_RXNEIE;
    naivemips_clr_bits(port, USART_SR, val);

    free_irq(port->irq, port);
}

static void naivemips_set_termios(struct uart_port *port, struct ktermios *termios,
                struct ktermios *old)
{
    // struct naivemips_port *naivemips_port = to_naivemips_port(port);
    tcflag_t cflag = termios->c_cflag;
    unsigned int baud = 115200;
    unsigned long flags;

    cflag &= ~CRTSCTS;

    spin_lock_irqsave(&port->lock, flags);

    port->status &= ~(UPSTAT_AUTOCTS | UPSTAT_AUTORTS);
    if (cflag & CRTSCTS) {
        port->status |= UPSTAT_AUTOCTS | UPSTAT_AUTORTS;
    }
    uart_update_timeout(port, cflag, baud);

    port->read_status_mask = USART_SR_ORE;

    /* Characters to ignore */
    port->ignore_status_mask = 0;
    if (termios->c_iflag & IGNBRK) {
        /*
         * If we're ignoring parity and break indicators,
         * ignore overruns too (for real raw support).
         */
        if (termios->c_iflag & IGNPAR)
            port->ignore_status_mask |= USART_SR_ORE;
    }

    /* Ignore all characters if CREAD is not set */
    if ((termios->c_cflag & CREAD) == 0)
        port->ignore_status_mask |= USART_SR_DUMMY_RX;

    spin_unlock_irqrestore(&port->lock, flags);
}

static const char *naivemips_type(struct uart_port *port)
{
    return (port->type == PORT_NAIVEMIPS) ? DRIVER_NAME : NULL;
}

static void naivemips_release_port(struct uart_port *port)
{
}

static int naivemips_request_port(struct uart_port *port)
{
    return 0;
}

static void naivemips_config_port(struct uart_port *port, int flags)
{
    if (flags & UART_CONFIG_TYPE)
        port->type = PORT_NAIVEMIPS;
}

static int
naivemips_verify_port(struct uart_port *port, struct serial_struct *ser)
{
    /* No user changeable parameters */
    return -EINVAL;
}

static void naivemips_pm(struct uart_port *port, unsigned int state,
        unsigned int oldstate)
{
}

static const struct uart_ops naivemips_uart_ops = {
    .tx_empty   = naivemips_tx_empty,
    .set_mctrl  = naivemips_set_mctrl,
    .get_mctrl  = naivemips_get_mctrl,
    .stop_tx    = naivemips_stop_tx,
    .start_tx   = naivemips_start_tx,
    .throttle   = naivemips_throttle,
    .unthrottle = naivemips_unthrottle,
    .stop_rx    = naivemips_stop_rx,
    .break_ctl  = naivemips_break_ctl,
    .startup    = naivemips_startup,
    .shutdown   = naivemips_shutdown,
    .set_termios    = naivemips_set_termios,
    .pm     = naivemips_pm,
    .type       = naivemips_type,
    .release_port   = naivemips_release_port,
    .request_port   = naivemips_request_port,
    .config_port    = naivemips_config_port,
    .verify_port    = naivemips_verify_port,
};

static int naivemips_init_port(struct naivemips_port *naivemipsport,
              struct platform_device *pdev)
{
    struct uart_port *port = &naivemipsport->port;
    struct resource *res;
    int ret = 0;

    port->iotype    = UPIO_MEM;
    port->flags = UPF_BOOT_AUTOCONF;
    port->ops   = &naivemips_uart_ops;
    port->dev   = &pdev->dev;
    port->irq   = platform_get_irq(pdev, 0);

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    port->membase = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(port->membase))
        return PTR_ERR(port->membase);
    port->mapbase = res->start;

    naivemipsport->port.uartclk = 1843200;
    if (!naivemipsport->port.uartclk)
        ret = -EINVAL;

    spin_lock_init(&port->lock);

    return ret;
}

static struct naivemips_port *naivemips_of_get_naivemips_port(struct platform_device *pdev)
{
    struct device_node *np = pdev->dev.of_node;
    int id;

    id = 0;

    if (WARN_ON(id >= NAIVEMIPS_MAX_PORTS))
        return NULL;

    return &naivemips_ports[id];
}

#ifdef CONFIG_OF
static const struct of_device_id naivemips_match[] = {
    { .compatible = "tsinghua,naivemips-uart", },
    {},
};

MODULE_DEVICE_TABLE(of, naivemips_match);
#endif

static int naivemips_serial_probe(struct platform_device *pdev)
{
    int ret;
    struct naivemips_port *naivemipsport;

    naivemipsport = naivemips_of_get_naivemips_port(pdev);
    if (!naivemipsport)
        return -ENODEV;

    ret = naivemips_init_port(naivemipsport, pdev);
    if (ret)
        return ret;

    ret = uart_add_one_port(&naivemips_usart_driver, &naivemipsport->port);
    if (ret)
        return ret;

    platform_set_drvdata(pdev, &naivemipsport->port);

    return 0;
}

static int naivemips_serial_remove(struct platform_device *pdev)
{
    struct uart_port *port = platform_get_drvdata(pdev);

    return uart_remove_one_port(&naivemips_usart_driver, port);
}


#ifdef CONFIG_SERIAL_NAIVEMIPS_CONSOLE
static void naivemips_console_putchar(struct uart_port *port, int ch)
{
    while (!(readl_relaxed(port->membase + USART_SR) & USART_SR_TXE))
        cpu_relax();

    writel_relaxed(ch, port->membase + USART_DR);
}

static void naivemips_console_write(struct console *co, const char *s, unsigned cnt)
{
    struct uart_port *port = &naivemips_ports[co->index].port;
    unsigned long flags;
    u32 old_cr1, new_cr1;
    int locked = 1;

    local_irq_save(flags);
    if (port->sysrq)
        locked = 0;
    else if (oops_in_progress)
        locked = spin_trylock(&port->lock);
    else
        spin_lock(&port->lock);

    /* Save and disable interrupts */
    old_cr1 = readl_relaxed(port->membase + USART_SR);
    new_cr1 = old_cr1 & ~(USART_SR_TXEIE|USART_SR_RXNEIE);
    writel_relaxed(new_cr1, port->membase + USART_SR);

    uart_console_write(port, s, cnt, naivemips_console_putchar);

    /* Restore interrupt state */
    writel_relaxed(old_cr1, port->membase + USART_SR);

    if (locked)
        spin_unlock(&port->lock);
    local_irq_restore(flags);
}

static int naivemips_console_setup(struct console *co, char *options)
{
    struct naivemips_port *naivemipsport;
    int baud = 115200;
    int bits = 8;
    int parity = 'n';
    int flow = 'n';

    if (co->index >= NAIVEMIPS_MAX_PORTS)
        return -ENODEV;

    naivemipsport = &naivemips_ports[co->index];

    /*
     * This driver does not support early console initialization
     * (use ARM early printk support instead), so we only expect
     * this to be called during the uart port registration when the
     * driver gets probed and the port should be mapped at that point.
     */
    if (naivemipsport->port.mapbase == 0 || naivemipsport->port.membase == NULL)
        return -ENXIO;

    if (options)
        uart_parse_options(options, &baud, &parity, &bits, &flow);

    return uart_set_options(&naivemipsport->port, co, baud, parity, bits, flow);
}

static struct console naivemips_console = {
    .name       = NAIVEMIPS_SERIAL_NAME,
    .device     = uart_console_device,
    .write      = naivemips_console_write,
    .setup      = naivemips_console_setup,
    .flags      = CON_PRINTBUFFER,
    .index      = -1,
    .data       = &naivemips_usart_driver,
};

#define STM32_SERIAL_CONSOLE (&naivemips_console)

#else
#define STM32_SERIAL_CONSOLE NULL
#endif /* CONFIG_SERIAL_NAIVEMIPS_CONSOLE */

static struct uart_driver naivemips_usart_driver = {
    .driver_name    = DRIVER_NAME,
    .dev_name   = NAIVEMIPS_SERIAL_NAME,
    .major      = TTY_MAJOR,
    .minor      = 64,
    .nr     = NAIVEMIPS_MAX_PORTS,
    .cons       = STM32_SERIAL_CONSOLE,
};

static struct platform_driver naivemips_serial_driver = {
    .probe      = naivemips_serial_probe,
    .remove     = naivemips_serial_remove,
    .driver = {
        .name   = DRIVER_NAME,
        .of_match_table = of_match_ptr(naivemips_match),
    },
};

static int __init usart_init(void)
{
    static char banner[] __initdata = "NaiveMIPS UART driver initialized";
    int ret;

    pr_info("%s\n", banner);

    ret = uart_register_driver(&naivemips_usart_driver);
    if (ret)
        return ret;

    ret = platform_driver_register(&naivemips_serial_driver);
    if (ret)
        uart_unregister_driver(&naivemips_usart_driver);

    return ret;
}

static void __exit usart_exit(void)
{
    platform_driver_unregister(&naivemips_serial_driver);
    uart_unregister_driver(&naivemips_usart_driver);
}

module_init(usart_init);
module_exit(usart_exit);

MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_DESCRIPTION("NaiveMIPS serial port driver");
MODULE_ALIAS_CHARDEV_MAJOR(TTY_MAJOR);
MODULE_LICENSE("GPL v2");
