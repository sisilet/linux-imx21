/*
 *  linux/arch/arm/mach-imx21/irq.c
 *
 *  Copyright (C) 1999 ARM Limited
 *  Copyright (C) 2002 Shane Nay (shane@minirl.com)
 *  Modified By: Ron Melvin (ron.melvin@timesys.com)
 *  Copyright (C) 2005, TimeSys Corporation 
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
 *  03/03/2004   Sascha Hauer <sascha@saschahauer.de>
 *               Copied from the motorola bsp package and added gpio demux
 *               interrupt handler
  */
#include <linux/init.h>
extern int x1;
#include <linux/list.h>
extern int x2;
#include <linux/timer.h>
extern int x3;

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <asm/mach/irq.h>

/*
 *
 * We simply use the ENABLE DISABLE registers inside of the IMX21
 * to turn on/off specific interrupts.  FIXME- We should
 * also add support for the accelerated interrupt controller
 * by putting offets to irq jump code in the appropriate
 * places.
 *
 */

#define INTENNUM_OFF              0x8
#define INTDISNUM_OFF             0xC

#define VA_AITC_BASE              IO_ADDRESS(MX2ADS_AITC_BASE)
#define IMX21_AITC_INTDISNUM       (VA_AITC_BASE + INTDISNUM_OFF)
#define IMX21_AITC_INTENNUM        (VA_AITC_BASE + INTENNUM_OFF)

#if 0
#define DEBUG_IRQ(fmt...)	printk(fmt)
#else
#define DEBUG_IRQ(fmt...)	do { } while (0)
#endif

static void
imx21_mask_irq(unsigned int irq)
{
	__raw_writel(irq, IMX21_AITC_INTDISNUM);
}

static void
imx21_unmask_irq(unsigned int irq)
{
	__raw_writel(irq, IMX21_AITC_INTENNUM);
}

static int
imx21_gpio_irq_type(unsigned int _irq, unsigned int type)
{
	unsigned int irq_type = 0, irq, reg, bit;

	irq = _irq - IRQ_GPIOA(0);
	reg = irq >> 5;
	bit = 1 << (irq % 32);

	if (type == IRQT_PROBE) {
		/* Don't mess with enabled GPIOs using preconfigured edges or
		   GPIOs set to alternate function during probe */
		/* TODO: support probe */
//              if ((GPIO_IRQ_rising_edge[idx] | GPIO_IRQ_falling_edge[idx]) &
//                  GPIO_bit(gpio))
//                      return 0;
//              if (GAFR(gpio) & (0x3 << (((gpio) & 0xf)*2)))
//                      return 0;
//              type = __IRQT_RISEDGE | __IRQT_FALEDGE;
	}

	GIUS(reg) |= bit;
	DDIR(reg) &= ~(bit);

	DEBUG_IRQ("setting type of irq %d to ", _irq);

	if (type & __IRQT_RISEDGE) {
		DEBUG_IRQ("rising edges\n");
		irq_type = 0x0;
	}
	if (type & __IRQT_FALEDGE) {
		DEBUG_IRQ("falling edges\n");
		irq_type = 0x1;
	}
	if (type & __IRQT_LOWLVL) {
		DEBUG_IRQ("low level\n");
		irq_type = 0x3;
	}
	if (type & __IRQT_HIGHLVL) {
		DEBUG_IRQ("high level\n");
		irq_type = 0x2;
	}

	if (irq % 32 < 16) {
		ICR1(reg) = (ICR1(reg) & ~(0x3 << ((irq % 16) * 2))) |
		    (irq_type << ((irq % 16) * 2));
	} else {
		ICR2(reg) = (ICR2(reg) & ~(0x3 << ((irq % 16) * 2))) |
		    (irq_type << ((irq % 16) * 2));
	}

	return 0;

}

static void
imx21_gpio_ack_irq(unsigned int irq)
{
	DEBUG_IRQ("%s: irq %d  isr %d\n", __FUNCTION__, irq,
                  IRQ_TO_REG(irq));
	ISR(IRQ_TO_REG(irq)) |= 1 << ((irq - IRQ_GPIOA(0)) % 32);
}

static void
imx21_gpio_mask_irq(unsigned int irq)
{
	DEBUG_IRQ("%s: irq %d\n", __FUNCTION__, irq);
	IMR(IRQ_TO_REG(irq)) &= ~( 1 << ((irq - IRQ_GPIOA(0)) % 32));
}

static void
imx21_gpio_unmask_irq(unsigned int irq)
{
	DEBUG_IRQ("%s: irq %d\n", __FUNCTION__, irq);
	IMR(IRQ_TO_REG(irq)) |= 1 << ((irq - IRQ_GPIOA(0)) % 32);
        PMASK = PMASK | (1 << IRQ_TO_REG(irq));
}

static void
imx21_gpio_handler(unsigned int irq_unused, struct irqdesc *desc,
                   struct pt_regs *regs)
{
    unsigned int mask;
    unsigned int port;
    unsigned int irq_base;
    unsigned int irq = 0;

    for (port = 0; port < 6; port++) {
        if (ISR(port) & IMR(port)) {
            break;
        }
    }

    mask = ISR(port);
    irq_base = IRQ_GPIOA(0) + (port * 32);

    desc = irq_desc + irq_base;
    while (mask) {
        if (mask & 1) {
            DEBUG_IRQ("handling irq %d (port %d)\n", irq, port);
            desc->handle(irq + irq_base, desc, regs);
            ISR(port) = (1 << irq);
        }
        irq++;
        desc++;
        mask >>= 1;
    }
}

static struct irqchip imx21_gpio_chip = {
	.ack = imx21_gpio_ack_irq,
	.mask = imx21_gpio_mask_irq,
	.unmask = imx21_gpio_unmask_irq,
	.type = imx21_gpio_irq_type,
};

static struct irqchip imx21_internal_chip = {
	.ack = imx21_mask_irq,
	.mask = imx21_mask_irq,
	.unmask = imx21_unmask_irq,
};

void __init
imx21_init_irq(void)
{
	unsigned int irq;

	DEBUG_IRQ("Initializing imx21 interrupts\n");

	/* Mask all interrupts initially */
	IMR(0) = 0;
	IMR(1) = 0;
	IMR(2) = 0;
	IMR(3) = 0;
	IMR(4) = 0;
	IMR(5) = 0;

	for (irq = 0; irq < IMX21_IRQS; irq++) {
		set_irq_chip(irq, &imx21_internal_chip);
		set_irq_handler(irq, do_level_IRQ);
		set_irq_flags(irq, IRQF_VALID);
	}

	for (irq = IRQ_GPIOA(0); irq < IRQ_GPIOF(32); irq++) {
		set_irq_chip(irq, &imx21_gpio_chip);
		set_irq_handler(irq, do_edge_IRQ);
		set_irq_flags(irq, IRQF_VALID);
	}

	set_irq_chained_handler(INT_GPIO, imx21_gpio_handler);

	/* Disable all interrupts initially. */
	/* In IMX21 this is done in the bootloader. */
}
