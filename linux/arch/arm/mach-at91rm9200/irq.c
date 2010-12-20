/*
 * linux/arch/arm/mach-at91rm9200/irq.c
 *
 *  Copyright (C) 2004 SAN People
 *  Copyright (C) 2004 ATMEL
 *  Copyright (C) Rick Bronson
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
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/types.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/setup.h>

#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/map.h>

#include <asm/arch/hardware.h>

#include "generic.h"

/*
 * The default interrupt priority levels.
 */
static unsigned int at91rm9200_default_irq_priority[NR_AIC_IRQS] __initdata = {
	(AT91C_AIC_PRIOR_HIGHEST),	// Advanced Interrupt Controller
	(AT91C_AIC_PRIOR_HIGHEST),	// System Peripheral
	(AT91C_AIC_PRIOR_LOWEST),	// Parallel IO Controller A
	(AT91C_AIC_PRIOR_LOWEST),	// Parallel IO Controller B
	(AT91C_AIC_PRIOR_LOWEST),	// Parallel IO Controller C
	(AT91C_AIC_PRIOR_LOWEST),	// Parallel IO Controller D
	(AT91C_AIC_PRIOR_6),		// USART 0
	(AT91C_AIC_PRIOR_6),		// USART 1
	(AT91C_AIC_PRIOR_6),		// USART 2
	(AT91C_AIC_PRIOR_6),		// USART 3
	(AT91C_AIC_PRIOR_LOWEST),	// Multimedia Card Interface
	(AT91C_AIC_PRIOR_4),		// USB Device Port
	(AT91C_AIC_PRIOR_LOWEST),	// Two-Wire Interface
	(AT91C_AIC_PRIOR_6),		// Serial Peripheral Interface
	(AT91C_AIC_PRIOR_5),		// Serial Synchronous Controller
	(AT91C_AIC_PRIOR_5),		// Serial Synchronous Controller
	(AT91C_AIC_PRIOR_5),		// Serial Synchronous Controller
	(AT91C_AIC_PRIOR_LOWEST),	// Timer Counter 0
	(AT91C_AIC_PRIOR_LOWEST),	// Timer Counter 1
	(AT91C_AIC_PRIOR_LOWEST),	// Timer Counter 2
	(AT91C_AIC_PRIOR_LOWEST),	// Timer Counter 3
	(AT91C_AIC_PRIOR_LOWEST),	// Timer Counter 4
	(AT91C_AIC_PRIOR_LOWEST),	// Timer Counter 5
	(AT91C_AIC_PRIOR_3),		// USB Host port
	(AT91C_AIC_PRIOR_3),		// Ethernet MAC
	(AT91C_AIC_PRIOR_LOWEST),	// Advanced Interrupt Controller
	(AT91C_AIC_PRIOR_LOWEST),	// Advanced Interrupt Controller
	(AT91C_AIC_PRIOR_LOWEST),	// Advanced Interrupt Controller
	(AT91C_AIC_PRIOR_LOWEST),	// Advanced Interrupt Controller
	(AT91C_AIC_PRIOR_LOWEST),	// Advanced Interrupt Controller
	(AT91C_AIC_PRIOR_LOWEST),	// Advanced Interrupt Controller
	(AT91C_AIC_PRIOR_LOWEST)	// Advanced Interrupt Controller
};


static void at91rm9200_ack_irq(unsigned int irq)
{
	/* Acknowledge and disable interrupt. */
	AT91_SYS->AIC_IDCR =  1 << irq;
	AT91_SYS->AIC_EOICR = 0;
}

static void at91rm9200_mask_irq(unsigned int irq)
{
	/* Disable interrupt on AIC */
	AT91_SYS->AIC_IDCR =  1 << irq;
}

static void at91rm9200_unmask_irq(unsigned int irq)
{
	/* Enable interrupt on AIC */
	AT91_SYS->AIC_IECR =  1 << irq;
}

static int at91rm9200_irq_type(unsigned irq, unsigned type)
{
	unsigned int srctype;

	/* change triggering only for external IRQ0..IRQ6 */
	if (irq < AT91C_ID_IRQ0)
		return -EINVAL;

	switch (type) {
	case IRQT_HIGH:
		srctype = AT91C_AIC_SRCTYPE_EXT_HIGH_LEVEL;
		break;
	case IRQT_RISING:
		srctype = AT91C_AIC_SRCTYPE_EXT_POSITIVE_EDGE;
		break;
	case IRQT_LOW:
		srctype = (0 << 5);
		break;
	case IRQT_FALLING:
		srctype = (1 << 5);
		break;
	default:
		return -EINVAL;
	}

	AT91_SYS->AIC_SMR[irq] = (AT91_SYS->AIC_SMR[irq] & ~AT91C_AIC_SRCTYPE) | srctype;
	return 0;
}

static struct irqchip at91rm9200_irq_chip = {
	.ack	= at91rm9200_ack_irq,
	.mask	= at91rm9200_mask_irq,
	.unmask	= at91rm9200_unmask_irq,
	.type	= at91rm9200_irq_type,
};

/*
 * Initialize the AIC interrupt controller.
 */
void __init at91rm9200_init_irq(unsigned int priority[NR_AIC_IRQS])
{
	unsigned int i;

	/* No priority list specified for this board -> use defaults */
	if (priority == NULL)
		priority = at91rm9200_default_irq_priority;

	/*
	 * The IVR is used by macro get_irqnr_and_base to read and verify.
	 * The irq number is NR_AIC_IRQS when a spurious interrupt has occurred.
	 */
	for (i = 0; i < NR_AIC_IRQS; i++) {
		/* Put irq number in Source Vector Register: */
		AT91_SYS->AIC_SVR[i] = i;
		/* Store the Source Mode Register as defined in table above */
		AT91_SYS->AIC_SMR[i] = (AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE | priority[i]);

		set_irq_chip(i, &at91rm9200_irq_chip);
		set_irq_handler(i, do_level_IRQ);
		set_irq_flags(i, IRQF_VALID | IRQF_PROBE);

		/* Perform 8 End Of Interrupt Command to make sure AIC will not Lock out nIRQ */
		if (i < 8)
			AT91_SYS->AIC_EOICR = AT91_SYS->AIC_EOICR;
	}

	/*
	 * Spurious Interrupt ID in Spurious Vector Register is NR_AIC_IRQS
	 * When there is no current interrupt, the IRQ Vector Register reads the value stored in AIC_SPU
	 */
	AT91_SYS->AIC_SPU = NR_AIC_IRQS;

	/* No debugging in AIC: Debug (Protect) Control Register */
	AT91_SYS->AIC_DCR = 0;

	/* Disable and clear all interrupts initially */
	AT91_SYS->AIC_IDCR = 0xFFFFFFFF;
	AT91_SYS->AIC_ICCR = 0xFFFFFFFF;
}
