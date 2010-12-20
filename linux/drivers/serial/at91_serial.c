/*
 *  linux/drivers/char/at91_serial.c
 *
 *  Driver for Atmel AT91RM9200 Serial ports
 *
 *  Copyright (C) 2003 Rick Bronson
 *
 *  Based on drivers/char/serial_sa1100.c, by Deep Blue Solutions Ltd.
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/tty_flip.h>

#include <asm/arch/AT91RM9200_USART.h>
#include <asm/mach/serial_at91rm9200.h>
#include <asm/arch/board.h>
#include <asm/arch/pio.h>


#if defined(CONFIG_SERIAL_AT91_CONSOLE)
int at91_console_port;
EXPORT_SYMBOL(at91_console_port);
#if defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif
#endif

#include <linux/serial_core.h>

#define SERIAL_AT91_MAJOR	TTY_MAJOR
#define CALLOUT_AT91_MAJOR	TTYAUX_MAJOR
#define MINOR_START		64

#define AT91C_VA_BASE_DBGU	((unsigned long) &(AT91_SYS->DBGU_CR))
#define AT91_ISR_PASS_LIMIT	256

#define UART_PUT_CR(port,v)	((AT91PS_USART)(port)->membase)->US_CR = v
#define UART_GET_MR(port)	((AT91PS_USART)(port)->membase)->US_MR
#define UART_PUT_MR(port,v)	((AT91PS_USART)(port)->membase)->US_MR = v
#define UART_PUT_IER(port,v)	((AT91PS_USART)(port)->membase)->US_IER = v
#define UART_PUT_IDR(port,v)	((AT91PS_USART)(port)->membase)->US_IDR = v
#define UART_GET_IMR(port)	((AT91PS_USART)(port)->membase)->US_IMR
#define UART_GET_CSR(port)	((AT91PS_USART)(port)->membase)->US_CSR
#define UART_GET_CHAR(port)	((AT91PS_USART)(port)->membase)->US_RHR
#define UART_PUT_CHAR(port,v)	((AT91PS_USART)(port)->membase)->US_THR = v
#define UART_GET_BRGR(port)	((AT91PS_USART)(port)->membase)->US_BRGR
#define UART_PUT_BRGR(port,v)	((AT91PS_USART)(port)->membase)->US_BRGR = v
#define UART_PUT_RTOR(port,v)	((AT91PS_USART)(port)->membase)->US_RTOR = v

// #define UART_GET_CR(port)	((AT91PS_USART)(port)->membase)->US_CR		// is write-only

 /* PDC registers */
#define UART_PUT_PTCR(port,v)	((AT91PS_USART)(port)->membase)->US_PTCR = v
#define UART_PUT_RPR(port,v)	((AT91PS_USART)(port)->membase)->US_RPR = v
#define UART_PUT_RCR(port,v)	((AT91PS_USART)(port)->membase)->US_RCR = v
#define UART_GET_RCR(port)	((AT91PS_USART)(port)->membase)->US_RCR
#define UART_PUT_RNPR(port,v)	((AT91PS_USART)(port)->membase)->US_RNPR = v
#define UART_PUT_RNCR(port,v)	((AT91PS_USART)(port)->membase)->US_RNCR = v


static int (*at91_open)(struct uart_port *);
static void (*at91_close)(struct uart_port *);

#ifdef SUPPORT_SYSRQ
static struct console at91_console;
#endif

int at91_serial_map[AT91C_NR_UART];

EXPORT_SYMBOL(at91_serial_map);


/*
 * Return TIOCSER_TEMT when transmitter FIFO and Shift register is empty.
 */
static u_int at91_tx_empty(struct uart_port *port)
{
	return (UART_GET_CSR(port) & AT91C_US_TXEMPTY) ? TIOCSER_TEMT : 0;
}

/*
 * Set state of the modem control output lines
 */
static void at91_set_mctrl(struct uart_port *port, u_int mctrl)
{
	unsigned int control = 0;

	/*
	 * Errata #39: RTS0 is not internally connected to PA21.  We need to drive
	 *  the pin manually.
	 */
	if (port->mapbase == AT91C_VA_BASE_US0) {
		if (mctrl & TIOCM_RTS)
			AT91_SYS->PIOA_CODR = AT91C_PA21_RTS0;
		else
			AT91_SYS->PIOA_SODR = AT91C_PA21_RTS0;
	}

	if (mctrl & TIOCM_RTS)
		control |= AT91C_US_RTSEN;
	else
		control |= AT91C_US_RTSDIS;

	if (mctrl & TIOCM_DTR)
		control |= AT91C_US_DTREN;
	else
		control |=  AT91C_US_DTRDIS;

	UART_PUT_CR(port,control);
}

/*
 * Get state of the modem control input lines
 */
static u_int at91_get_mctrl(struct uart_port *port)
{
	unsigned int status, ret = 0;

	status = UART_GET_CSR(port);
	if (status & AT91C_US_DCD)
		ret |= TIOCM_CD;
	if (status & AT91C_US_CTS)
		ret |= TIOCM_CTS;
	if (status & AT91C_US_DSR)
		ret |= TIOCM_DSR;
	if (status & AT91C_US_RI)
		ret |= TIOCM_RI;

	return ret;
}

/*
 * Stop transmitting.
 */
static void at91_stop_tx(struct uart_port *port, u_int from_tty)
{
	UART_PUT_IDR(port, AT91C_US_TXRDY);
	port->read_status_mask &= ~AT91C_US_TXRDY;
}

/*
 * Start transmitting.
 */
static void at91_start_tx(struct uart_port *port, u_int from_tty)
{
	unsigned long flags;

	local_irq_save(flags);
	port->read_status_mask |= AT91C_US_TXRDY;
	UART_PUT_IER(port, AT91C_US_TXRDY);
	local_irq_restore(flags);

}

/*
 * Stop receiving - port is in process of being closed.
 */
static void at91_stop_rx(struct uart_port *port)
{
	UART_PUT_IDR(port, AT91C_US_RXRDY);
}

/*
 * Enable modem status interrupts
 */
static void at91_enable_ms(struct uart_port *port)
{
	UART_PUT_IER(port, AT91C_US_RIIC | AT91C_US_DSRIC | AT91C_US_DCDIC | AT91C_US_CTSIC);
}

/*
 * Control the transmission of a break signal
 */
static void at91_break_ctl(struct uart_port *port, int break_state)
{
	if (break_state != 0)
		UART_PUT_CR(port, AT91C_US_STTBRK);	/* start break */
	else
		UART_PUT_CR(port, AT91C_US_STPBRK);	/* stop break */
}

/*
 * Characters received (called from interrupt handler)
 */
static void at91_rx_chars(struct uart_port *port, struct pt_regs *regs)
{
	struct tty_struct *tty = port->info->tty;
	unsigned int status, ch, flg, ignored = 0;

	status = UART_GET_CSR(port) & port->read_status_mask;
	while (status & (AT91C_US_RXRDY)) {
		ch = UART_GET_CHAR(port);

		if (tty->flip.count >= TTY_FLIPBUF_SIZE)
			goto ignore_char;
		port->icount.rx++;

		flg = TTY_NORMAL;

		/*
		 * note that the error handling code is
		 * out of the main execution path
		 */
		if (status & (AT91C_US_PARE | AT91C_US_FRAME | AT91C_US_OVRE))
			goto handle_error;

		if (uart_handle_sysrq_char(port, ch, regs))
			goto ignore_char;

	error_return:
		tty_insert_flip_char(tty, ch, flg);
	ignore_char:
		status = UART_GET_CSR(port) & port->read_status_mask;
	}
out:
	tty_flip_buffer_push(tty);
	return;

handle_error:
	if (status & (AT91C_US_PARE | AT91C_US_FRAME | AT91C_US_OVRE))
		UART_PUT_CR(port, AT91C_US_RSTSTA);  /* clear error */
	if (status & (AT91C_US_PARE))
		port->icount.parity++;
	else if (status & (AT91C_US_FRAME))
		port->icount.frame++;
	if (status & (AT91C_US_OVRE))
		port->icount.overrun++;

	if (status & port->ignore_status_mask) {
		if (++ignored > 100)
			goto out;
		goto ignore_char;
	}

	status &= port->read_status_mask;

	UART_PUT_CR(port, AT91C_US_RSTSTA);  /* clear error */
	if (status & AT91C_US_PARE)
		flg = TTY_PARITY;
	else if (status & AT91C_US_FRAME)
		flg = TTY_FRAME;

	if (status & AT91C_US_OVRE) {
		/*
		 * overrun does *not* affect the character
		 * we read from the FIFO
		 */
		tty_insert_flip_char(tty, ch, flg);
		ch = 0;
		flg = TTY_OVERRUN;
	}
#ifdef SUPPORT_SYSRQ
	port->sysrq = 0;
#endif
	goto error_return;
}

/*
 * Transmit characters (called from interrupt handler)
 */
static void at91_tx_chars(struct uart_port *port)
{
	struct circ_buf *xmit = &port->info->xmit;

	if (port->x_char) {
		UART_PUT_CHAR(port, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		at91_stop_tx(port, 0);
		return;
	}

	while (UART_GET_CSR(port) & AT91C_US_TXRDY) {
		UART_PUT_CHAR(port, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		at91_stop_tx(port, 0);
}

/*
 * Interrupt handler
 */
static irqreturn_t at91_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	struct uart_port *port = dev_id;
	unsigned int status, pending, pass_counter = 0;

	status = UART_GET_CSR(port);
	pending = status & port->read_status_mask;
	if (pending) {
		do {
			if (pending & AT91C_US_RXRDY)
				at91_rx_chars(port, regs);

			/* Hmm. Break seems to be reported as a frame error...
			   Clear the relevent break bits */
			if (pending & (AT91C_US_RXBRK | AT91C_US_FRAME)) {
				UART_PUT_CR(port, AT91C_US_RSTSTA);
				port->icount.brk++;
				uart_handle_break(port);
			}

			// TODO: All reads to CSR will clear these interrupts!
			if (pending & AT91C_US_RIIC) port->icount.rng++;
			if (pending & AT91C_US_DSRIC) port->icount.dsr++;
			if (pending & AT91C_US_DCDIC) {
				port->icount.dcd++;
				uart_handle_dcd_change(port, status & AT91C_US_DCD);
			}
			if (pending & AT91C_US_CTSIC) {
				port->icount.cts++;
				uart_handle_cts_change(port, status & AT91C_US_CTS);
			}
			if (pending & (AT91C_US_RIIC | AT91C_US_DSRIC | AT91C_US_DCDIC | AT91C_US_CTSIC))
				wake_up_interruptible(&port->info->delta_msr_wait);

			if (pending & AT91C_US_TXRDY)
				at91_tx_chars(port);
			if (pass_counter++ > AT91_ISR_PASS_LIMIT)
				break;

			status = UART_GET_CSR(port);
			pending = status & port->read_status_mask;
		} while (pending);
	}
	return IRQ_HANDLED;
}

/*
 * Perform initialization and enable port for reception
 */
static int at91_startup(struct uart_port *port)
{
	int retval;

	/* 
	 * Ensure that no interrupts are enabled otherwise when 
	 * request_irq() is called we could get stuck trying to 
	 * handle an unexpected interrupt
	 */
	UART_PUT_IDR(port, -1);

	/*
	 * Allocate the IRQ
	 */
	retval = request_irq(port->irq, at91_interrupt, SA_SHIRQ, "at91_serial", port);
	if (retval) {
		printk("at91_serial: at91_startup - Can't get irq\n");
		return retval;
	}

	/*
	 * If there is a specific "open" function (to register
	 * control line interrupts)
	 */
	if (at91_open) {
		retval = at91_open(port);
		if (retval) {
			free_irq(port->irq, port);
			return retval;
		}
	}

	port->read_status_mask = AT91C_US_RXRDY | AT91C_US_TXRDY | AT91C_US_OVRE
			| AT91C_US_FRAME | AT91C_US_PARE | AT91C_US_RXBRK;
	/*
	 * Finally, enable the serial port
	 */
	UART_PUT_CR(port, AT91C_US_RSTSTA | AT91C_US_RSTRX);
	UART_PUT_CR(port, AT91C_US_TXEN | AT91C_US_RXEN);  /* enable xmit & rcvr */
	UART_PUT_IER(port, AT91C_US_RXRDY);  /* do receive only */
	return 0;
}

/*
 * Disable the port
 */
static void at91_shutdown(struct uart_port *port)
{
	/*
	 * Disable all interrupts, port and break condition.
	 */
	UART_PUT_CR(port, AT91C_US_RSTSTA);
	UART_PUT_IDR(port, -1);

	/*
	 * Free the interrupt
	 */
	free_irq(port->irq, port);

	/*
	 * If there is a specific "close" function (to unregister
	 * control line interrupts)
	 */
	if (at91_close)
		at91_close(port);
}

/*
 * Power / Clock management.
 */
static void at91_serial_pm(struct uart_port *port, unsigned int state, unsigned int oldstate)
{
	switch (state) {
		case 0:
			/*
			 * Enable the peripheral clock for this serial port.
			 * This is called on uart_open() or a resume event.
			 */
			AT91_SYS->PMC_PCER = 1 << port->irq;
			break;
		case 3:
			/*
			 * Disable the peripheral clock for this serial port.
			 * This is called on uart_close() or a suspend event.
			 */
			if (port->irq != AT91C_ID_SYS)			/* is this a shared clock? */
				AT91_SYS->PMC_PCDR = 1 << port->irq;
			break;
		default:
			printk(KERN_ERR "at91_serial: unknown pm %d\n", state);
	}
}

static struct uart_ops at91_pops;		/* forward declaration */

/*
 * Change the port parameters
 */
static void at91_set_termios(struct uart_port *port, struct termios * termios, struct termios * old)
{
	unsigned long flags;
	unsigned int mode, imr, quot, baud;

	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16); 
	quot = uart_get_divisor(port, baud);

	/* Get current mode register */
	mode = UART_GET_MR(port) & ~(AT91C_US_CHRL | AT91C_US_NBSTOP | AT91C_US_PAR);

	uart_update_timeout(port, termios->c_cflag, baud);

	/* byte size */
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		mode |= AT91C_US_CHRL_5_BITS;
		break;
	case CS6:
		mode |= AT91C_US_CHRL_6_BITS;
		break;
	case CS7:
		mode |= AT91C_US_CHRL_7_BITS;
		break;
	default:
		mode |= AT91C_US_CHRL_8_BITS;
		break;
	}

	/* stop bits */
	if (termios->c_cflag & CSTOPB)
		mode |= AT91C_US_NBSTOP_2_BIT;

	/* parity */
	if (termios->c_cflag & PARENB) {
		if (termios->c_cflag & CMSPAR) {			/* Mark or Space parity */
			if (termios->c_cflag & PARODD)
				mode |= AT91C_US_PAR_MARK;
			else
				mode |= AT91C_US_PAR_SPACE;
		}
		else if (termios->c_cflag & PARODD)
			mode |= AT91C_US_PAR_ODD;
		else
			mode |= AT91C_US_PAR_EVEN;
	}
	else
		mode |= AT91C_US_PAR_NONE;

	port->read_status_mask |= AT91C_US_OVRE;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= AT91C_US_FRAME | AT91C_US_PARE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= AT91C_US_RXBRK;

	/*
	 * Characters to ignore
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= (AT91C_US_FRAME | AT91C_US_PARE);
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= AT91C_US_RXBRK;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= AT91C_US_OVRE;
	}

	// TODO: Ignore all characters if CREAD is set.

	/* first, disable interrupts and drain transmitter */
	local_irq_save(flags);
	imr = UART_GET_IMR(port);	/* get interrupt mask */
	UART_PUT_IDR(port, -1);		/* disable all interrupts */
	local_irq_restore(flags);
	while (!(UART_GET_CSR(port) & AT91C_US_TXEMPTY)) { barrier(); }

	/* disable receiver and transmitter */
	UART_PUT_CR(port, AT91C_US_TXDIS | AT91C_US_RXDIS);

	/* set the parity, stop bits and data size */
	UART_PUT_MR(port, mode);

	/* set the baud rate */
	UART_PUT_BRGR(port, quot);
	UART_PUT_CR(port, AT91C_US_RSTSTA | AT91C_US_RSTRX);
	UART_PUT_CR(port, AT91C_US_TXEN | AT91C_US_RXEN);

	/* restore interrupts */
	UART_PUT_IER(port, imr);

	/* CTS flow-control and modem-status interrupts */
	if (UART_ENABLE_MS(port, termios->c_cflag))
		at91_pops.enable_ms(port);
}

/*
 * Return string describing the specified port
 */
static const char *at91_type(struct uart_port *port)
{
	return (port->type == PORT_AT91RM9200) ? "AT91_SERIAL" : NULL;
}

/*
 * Release the memory region(s) being used by 'port'.
 */
static void at91_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase,
		(port->mapbase == AT91C_VA_BASE_DBGU) ? 512 : SZ_16K);
}

/*
 * Request the memory region(s) being used by 'port'.
 */
static int at91_request_port(struct uart_port *port)
{
	return request_mem_region(port->mapbase,
		(port->mapbase == AT91C_VA_BASE_DBGU) ? 512 : SZ_16K,
		"at91_serial") != NULL ? 0 : -EBUSY;

}

/*
 * Configure/autoconfigure the port.
 */
static void at91_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_AT91RM9200;
		at91_request_port(port);
	}
}

/*
 * Verify the new serial_struct (for TIOCSSERIAL).
 */
static int at91_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_AT91RM9200)
		ret = -EINVAL;
	if (port->irq != ser->irq)
		ret = -EINVAL;
	if (ser->io_type != SERIAL_IO_MEM)
		ret = -EINVAL;
	if (port->uartclk / 16 != ser->baud_base)
		ret = -EINVAL;
	if ((void *)port->mapbase != ser->iomem_base)
		ret = -EINVAL;
	if (port->iobase != ser->port)
		ret = -EINVAL;
	if (ser->hub6 != 0)
		ret = -EINVAL;
	return ret;
}

static struct uart_ops at91_pops = {
	.tx_empty	= at91_tx_empty,
	.set_mctrl	= at91_set_mctrl,
	.get_mctrl	= at91_get_mctrl,
	.stop_tx	= at91_stop_tx,
	.start_tx	= at91_start_tx,
	.stop_rx	= at91_stop_rx,
	.enable_ms	= at91_enable_ms,
	.break_ctl	= at91_break_ctl,
	.startup	= at91_startup,
	.shutdown	= at91_shutdown,
	.set_termios	= at91_set_termios,
	.type		= at91_type,
	.release_port	= at91_release_port,
	.request_port	= at91_request_port,
	.config_port 	= at91_config_port,
	.verify_port 	= at91_verify_port,
	.pm		= at91_serial_pm,
};

static struct uart_port at91_ports[AT91C_NR_UART];

void __init at91_init_ports(void)
{
	static int first = 1;
	int i;

	if (!first)
		return;
	first = 0;

	for (i = 0; i < AT91C_NR_UART; i++) {
		at91_ports[i].iotype	= SERIAL_IO_MEM;
		at91_ports[i].flags     = ASYNC_BOOT_AUTOCONF;
		at91_ports[i].uartclk   = at91_master_clock;
		at91_ports[i].ops	= &at91_pops;
		at91_ports[i].fifosize  = 1;
		at91_ports[i].line	= i;
 	}
}

void __init at91_register_uart_fns(struct at91rm9200_port_fns *fns)
{
	if (fns->enable_ms)
		at91_pops.enable_ms = fns->enable_ms;
	if (fns->get_mctrl)
		at91_pops.get_mctrl = fns->get_mctrl;
	if (fns->set_mctrl)
		at91_pops.set_mctrl = fns->set_mctrl;
	at91_open          = fns->open;
	at91_close         = fns->close;
	at91_pops.pm       = fns->pm;
	at91_pops.set_wake = fns->set_wake;
}

/*
 * Setup ports.
 */
void __init at91_register_uart(int idx, int port)
{
	if ((idx < 0) || (idx >= AT91C_NR_UART)) {
		printk(KERN_ERR "%s: bad index number %d\n", __FUNCTION__, idx);
		return;
	}

	switch (port) {
	case 0:
		at91_ports[idx].membase = (void *) AT91C_VA_BASE_US0;
		at91_ports[idx].mapbase = AT91C_VA_BASE_US0;
		at91_ports[idx].irq     = AT91C_ID_US0;
		AT91_CfgPIO_USART0();
		break;
	case 1:
		at91_ports[idx].membase = (void *) AT91C_VA_BASE_US1;
		at91_ports[idx].mapbase = AT91C_VA_BASE_US1;
		at91_ports[idx].irq     = AT91C_ID_US1;
		AT91_CfgPIO_USART1();
		break;
	case 2:
		at91_ports[idx].membase = (void *) AT91C_VA_BASE_US2;
		at91_ports[idx].mapbase = AT91C_VA_BASE_US2;
		at91_ports[idx].irq     = AT91C_ID_US2;
		AT91_CfgPIO_USART2();
		break;
	case 3:
		at91_ports[idx].membase = (void *) AT91C_VA_BASE_US3;
		at91_ports[idx].mapbase = AT91C_VA_BASE_US3;
		at91_ports[idx].irq     = AT91C_ID_US3;
		AT91_CfgPIO_USART3();
		break;
	case 4:
		at91_ports[idx].membase = (void *) AT91C_VA_BASE_DBGU;
		at91_ports[idx].mapbase = AT91C_VA_BASE_DBGU;
		at91_ports[idx].irq     = AT91C_ID_SYS;
		AT91_CfgPIO_DBGU();
		break;
	default:
		printk(KERN_ERR  "%s : bad port number %d\n", __FUNCTION__, port);
	}
}

#ifdef CONFIG_SERIAL_AT91_CONSOLE

/*
 * Interrupts are disabled on entering
 */
static void at91_console_write(struct console *co, const char *s, u_int count)
{
	struct uart_port *port = at91_ports + co->index;
	unsigned int status, i, imr;

	/*
	 *	First, save IMR and then disable interrupts
	 */
	imr = UART_GET_IMR(port);	/* get interrupt mask */
	UART_PUT_IDR(port, AT91C_US_RXRDY | AT91C_US_TXRDY);

	/*
	 *	Now, do each character
	 */
	for (i = 0; i < count; i++) {
		do {
			status = UART_GET_CSR(port);
		} while (!(status & AT91C_US_TXRDY));
		UART_PUT_CHAR(port, s[i]);
		if (s[i] == '\n') {
			do {
				status = UART_GET_CSR(port);
			} while (!(status & AT91C_US_TXRDY));
			UART_PUT_CHAR(port, '\r');
		}
	}

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore IMR
	 */
	do {
		status = UART_GET_CSR(port);
	} while (!(status & AT91C_US_TXRDY));
	UART_PUT_IER(port, imr);	/* set interrupts back the way they were */
}

/*
 * If the port was already initialised (eg, by a boot loader), try to determine
 * the current setup.
 */
static void __init at91_console_get_options(struct uart_port *port, int *baud, int *parity, int *bits)
{
	unsigned int mr, quot;

// TODO: CR is a write-only register
//	unsigned int cr;
//
//	cr = UART_GET_CR(port) & (AT91C_US_RXEN | AT91C_US_TXEN);
//	if (cr == (AT91C_US_RXEN | AT91C_US_TXEN)) {
//		/* ok, the port was enabled */
//	}

	mr = UART_GET_MR(port) & AT91C_US_CHRL;
	if (mr == AT91C_US_CHRL_8_BITS)
		*bits = 8;
	else
		*bits = 7;

	mr = UART_GET_MR(port) & AT91C_US_PAR;
	if (mr == AT91C_US_PAR_EVEN)
		*parity = 'e';
	else if (mr == AT91C_US_PAR_ODD)
		*parity = 'o';

	quot = UART_GET_BRGR(port);
	*baud = port->uartclk / (16 * (quot));
}

static int __init at91_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	port = uart_get_console(at91_ports, AT91C_NR_UART, co);

	/*
	 * Enable the serial console, in-case bootloader did not do it.
	 */
	AT91_SYS->PMC_PCER = 1 << port->irq;		/* enable clock */
	UART_PUT_IDR(port, -1);				/* disable interrupts */
	UART_PUT_CR(port, AT91C_US_RSTSTA | AT91C_US_RSTRX);
	UART_PUT_CR(port, AT91C_US_TXEN | AT91C_US_RXEN);
	
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		at91_console_get_options(port, &baud, &parity, &bits);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

extern struct uart_driver at91_uart;

static struct console at91_console = {
	.name		= "ttyS",
	.write		= at91_console_write,
	.device		= uart_console_device,
	.setup		= at91_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &at91_uart,
};

#define AT91_CONSOLE_DEVICE	&at91_console

static int  __init at91_console_init(void)
{
	at91_init_ports();

	at91_console.index = at91_console_port;
	register_console(&at91_console);
	return 0;
}
console_initcall(at91_console_init);

#else
#define AT91_CONSOLE_DEVICE	NULL
#endif

static struct uart_driver at91_uart = {
	.driver_name		= "ttyS",
	.dev_name		= "ttyS",
	.devfs_name		= "ttyS",
	.major			= SERIAL_AT91_MAJOR,
	.minor			= MINOR_START,
	.nr			= AT91C_NR_UART,
	.cons			= AT91_CONSOLE_DEVICE,
};

static int __init at91_serial_init(void)
{
	int ret, i;

	at91_init_ports();

	ret = uart_register_driver(&at91_uart);
	if (ret)
		return ret;

	for (i = 0; i < AT91C_NR_UART; i++) {
		if (at91_serial_map[i] >= 0)
			uart_add_one_port(&at91_uart, &at91_ports[i]);
	}

	return 0;
}

static void __exit at91_serial_exit(void)
{
	int i;

	for (i = 0; i < AT91C_NR_UART; i++) {
 		if (at91_serial_map[i] >= 0)
			uart_remove_one_port(&at91_uart, &at91_ports[i]);
  	}

	uart_unregister_driver(&at91_uart);
}

module_init(at91_serial_init);
module_exit(at91_serial_exit);

MODULE_AUTHOR("Rick Bronson");
MODULE_DESCRIPTION("AT91 generic serial port driver");
MODULE_LICENSE("GPL");
