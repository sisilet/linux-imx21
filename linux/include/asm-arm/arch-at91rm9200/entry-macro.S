/*
 * include/asm-arm/arch-at91rm9200/entry-macro.S
 *
 *  Copyright (C) 2003-2005 SAN People
 *
 *  Copyright (C) 2003-2005 SAN People
 *
 * Low-level IRQ helper macros for AT91RM9200 platforms
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <asm/hardware.h>

	.macro	disable_fiq
	.endm

	.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp
	ldr	\base, =(AT91C_VA_BASE_SYS)	@ base virtual address of SYS peripherals
	ldr	\irqnr, [\base, #AIC_IVR]	@ read IRQ vector register: de-asserts nIRQ to processor (and clears interrupt)
	ldr	\irqstat, [\base, #AIC_ISR]	@ read interrupt source number
	teq	\irqstat, #0			@ ISR is 0 when no current interrupt, or spurious interrupt
	streq	\tmp, [\base, #AIC_EOICR]	@ not going to be handled further, then ACK it now.
	.endm

