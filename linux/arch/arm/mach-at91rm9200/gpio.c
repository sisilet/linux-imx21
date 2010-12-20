/*
 * linux/arch/arm/mach-at91rm9200/gpio.c
 *
 * Copyright (C) 2005 HP Labs
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>

#include <asm/io.h>
#include <asm/mach/irq.h>

#include <asm/arch/hardware.h>
#include <asm/arch/gpio.h>

/*
 * Some chips have all four pio controllers (A..D); others have fewer.
 * This code assumes each controller supports all 32 pins; some don't.
 */
struct pio_controller {
	u32	per;			/* pio controller owns? */
	u32	pdr;
	u32	psr;
	u32	__reserved0;

	u32	oer;			/* pio output (vs input) */
	u32	odr;
	u32	osr;
	u32	__reserved1;

	u32	ifer;			/* input glitch filter */
	u32	ifdr;
	u32	ifsr;
	u32	__reserved2;

	u32	sodr;			/* pio output bit status */
	u32	codr;
	u32	odsr;
	u32	pdsr;			/* current pin state */

	u32	ier;			/* (input) interrupt */
	u32	idr;
	u32	imr;			/* interrupt mask */
	u32	isr;			/* interrupt status */

	u32	mder;			/* multi-driver */
	u32	mddr;
	u32	mdsr;
	u32	__reserved3;

	u32	pudr;			/* pullup */
	u32	puer;
	u32	pusr;
	u32	__reserved4;

	u32	asr;			/* A-periph */
	u32	bsr;
	u32	absr;
	u32	__reserved5;

	u32	__reserved6[0x20];

	u32	ower;			/* output write */
	u32	owdr;
	u32	owsr;
	u32	__reserved7;
};

static const u32 pio_virt[4] = {
	AT91_IO_P2V(0xfffff400),
	AT91_IO_P2V(0xfffff600),
	AT91_IO_P2V(0xfffff800),
	AT91_IO_P2V(0xfffffa00),
};

static inline struct pio_controller __iomem *pin_to_controller(unsigned pin)
{
	pin -= PIN_BASE;
	pin /= 32;
	if (likely(pin < BGA_GPIO_BANKS))
		return (struct pio_controller __iomem *) pio_virt[pin];
	return NULL;
}

static inline unsigned pin_to_mask(unsigned pin)
{
	pin -= PIN_BASE;
	return 1 << (pin % 32);
}


/*--------------------------------------------------------------------------*/

/* Not all hardware capabilities are exposed through these calls; they
 * only encapsulate the most common features and modes.  (So if you
 * want to change signals in groups, do it directly.)
 *
 * Bootloaders will usually handle some of the pin multiplexing setup.
 * The intent is certainly that by the time Linux is fully booted, all
 * pins should have been fully initialized.  These setup calls should
 * only be used by board setup routines, or possibly in driver probe().
 *
 * For bootloaders doing all that setup, these calls could be inlined
 * as NOPs so Linux won't duplicate any setup code
 */


/*
 * mux the pin to the "A" internal peripheral role.
 */
int __init_or_module at91_set_A_periph(unsigned pin, int use_pullup)
{
	struct pio_controller __iomem	*pio = pin_to_controller(pin);
	unsigned			mask = pin_to_mask(pin);

	if (!pio)
		return -EINVAL;

	__raw_writel(mask, &pio->idr);
	__raw_writel(mask, use_pullup ? &pio->puer : &pio->pudr);
	__raw_writel(mask, &pio->asr);
	__raw_writel(mask, &pio->pdr);
	return 0;
}
EXPORT_SYMBOL(at91_set_A_periph);


/*
 * mux the pin to the "B" internal peripheral role.
 */
int __init_or_module at91_set_B_periph(unsigned pin, int use_pullup)
{
	struct pio_controller __iomem	*pio = pin_to_controller(pin);
	unsigned			mask = pin_to_mask(pin);

	if (!pio)
		return -EINVAL;

	__raw_writel(mask, &pio->idr);
	__raw_writel(mask, use_pullup ? &pio->puer : &pio->pudr);
	__raw_writel(mask, &pio->bsr);
	__raw_writel(mask, &pio->pdr);
	return 0;
}
EXPORT_SYMBOL(at91_set_B_periph);


/*
 * mux the pin to the gpio controller (instead of "A" or "B" peripheral).
 *  for input, mode == use_pullup
 *  for output, mode == initial value
 */
int __init_or_module at91_set_gpio_direction(unsigned pin, int is_input, int mode)
{
	struct pio_controller __iomem	*pio = pin_to_controller(pin);
	unsigned			mask = pin_to_mask(pin);

	if (!pio)
		return -EINVAL;

	__raw_writel(mask, &pio->idr);
	if (is_input) {
		__raw_writel(mask, mode ? &pio->puer : &pio->pudr);
		__raw_writel(mask, &pio->odr);
	} else {
		__raw_writel(mask, &pio->pudr);
		__raw_writel(mask, mode ? &pio->sodr : &pio->codr);
		__raw_writel(mask, &pio->oer);
	}
	__raw_writel(mask, &pio->per);
	return 0;
}
EXPORT_SYMBOL(at91_set_gpio_direction);


/*
 * enable/disable the glitch filter; mostly used with IRQ handling.
 */
int __init_or_module at91_set_deglitch(unsigned pin, int is_on)
{
	struct pio_controller __iomem	*pio = pin_to_controller(pin);
	unsigned			mask = pin_to_mask(pin);

	if (!pio)
		return -EINVAL;
	__raw_writel(mask, is_on ? &pio->ifer : &pio->ifdr);
	return 0;
}
EXPORT_SYMBOL(at91_set_deglitch);

/*--------------------------------------------------------------------------*/


/*
 * assuming the pin is muxed as a gpio output, set its value.
 */
int at91_set_gpio_value(unsigned pin, int value)
{
	struct pio_controller __iomem	*pio = pin_to_controller(pin);
	unsigned			mask = pin_to_mask(pin);

	if (!pio)
		return -EINVAL;
	__raw_writel(mask, value ? &pio->sodr : &pio->codr);
	return 0;
}
EXPORT_SYMBOL(at91_set_gpio_value);


/*
 * read the pin's value (works even if it's not muxed as a gpio).
 */
int at91_get_gpio_value(unsigned pin)
{
	struct pio_controller __iomem	*pio = pin_to_controller(pin);
	unsigned			mask = pin_to_mask(pin);
	u32				pdsr;

	if (!pio)
		return -EINVAL;
	pdsr = __raw_readl(&pio->pdsr);
	return (pdsr & mask) != 0;
}
EXPORT_SYMBOL(at91_get_gpio_value);

/*--------------------------------------------------------------------------*/


/* Several AIC controller irqs are dispatched through this GPIO handler.
 * To use any AT91_PIN_* as an externally triggered IRQ, first call
 * at91_set_gpio_direction() then maybe enable its glitch filter.
 * Then just request_irq() with the pin ID; it works like any ARM IRQ
 * handler, though it always triggers on rising and falling edges.
 *
 * Alternatively, certain pins may be used directly as IRQ0..IRQ6 after
 * configuring them with at91_set_a_periph() or at91_set_b_periph().
 * IRQ0..IRQ6 should be configurable, e.g. level vs edge triggering.
 */

static void gpio_irq_mask(unsigned pin)
{
	struct pio_controller __iomem	*pio = pin_to_controller(pin);
	unsigned			mask = pin_to_mask(pin);

	if (pio)
		__raw_writel(mask, &pio->idr);
}

static void gpio_irq_unmask(unsigned pin)
{
	struct pio_controller __iomem	*pio = pin_to_controller(pin);
	unsigned			mask = pin_to_mask(pin);

	if (pio)
		__raw_writel(mask, &pio->ier);
}

static int gpio_irq_type(unsigned pin, unsigned type)
{
	return (type == IRQT_BOTHEDGE) ? 0 : -EINVAL;
}

static struct irqchip gpio_irqchip = {
	.mask = 	gpio_irq_mask,
	.unmask = 	gpio_irq_unmask,
	.type =		gpio_irq_type,
};

static void gpio_irq_handler(unsigned irq, struct irqdesc *desc, struct pt_regs *regs)
{
	unsigned			pin;
	struct irqdesc			*gpio;
	struct pio_controller __iomem	*pio;
	u32				isr;

	pio = (void __force __iomem *) desc->chipdata;

	/* temporarily mask (level sensitive) parent IRQ */
	desc->chip->ack(irq);
	for (;;) {
		isr = __raw_readl(&pio->isr) & __raw_readl(&pio->imr);
		if (!isr)
			break;

		pin = (unsigned) desc->data;
		gpio = &irq_desc[pin];

		while (isr) {
			if (isr & 1)
				gpio->handle(pin, gpio, regs);
			pin++;
			gpio++;
			isr >>= 1;
		}
	}
	desc->chip->unmask(irq);
	/* now it may re-trigger */
}

/* call this from board-specific init_irq */
void __init at91_gpio_irq_setup(unsigned banks)
{
	unsigned	pioc, pin, id;

	if (banks > 4)
		banks = 4;
	for (pioc = 0, pin = PIN_BASE, id = AT91C_ID_PIOA;
			pioc < banks;
			pioc++, id++) {
		struct pio_controller __iomem	*controller;
		unsigned			i;

		controller = (void __force __iomem *) pio_virt[pioc];
		__raw_writel(~0, &controller->idr);

		set_irq_data(id, (void *) pin);
		set_irq_chipdata(id, (void __force *) controller);

		for (i = 0; i < 32; i++, pin++) {
			set_irq_chip(pin, &gpio_irqchip);
			set_irq_handler(pin, do_simple_IRQ);
			set_irq_flags(pin, IRQF_VALID);
		}

		set_irq_chained_handler(id, gpio_irq_handler);

		/* enable the PIO peripheral clock */
		AT91_SYS->PMC_PCER = 1 << id;
	}
	pr_info("AT91: %d gpio irqs in %d banks\n", pin - PIN_BASE, banks);
}
