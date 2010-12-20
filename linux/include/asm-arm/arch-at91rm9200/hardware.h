/*
 * linux/include/asm-arm/arch-at91rm9200/hardware.h
 *
 *  Copyright (C) 2003 SAN People
 *  Copyright (C) 2003 ATMEL
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <asm/sizes.h>

#include <asm/arch/AT91RM9200.h>
#include <asm/arch/AT91RM9200_SYS.h>


/*
 * Remap the peripherals from address 0xFFFA0000 .. 0xFFFFFFFF
 * to 0xFEFA0000 .. 0xFF000000.  (384Kb)
 */
#define AT91C_IO_PHYS_BASE	0xFFFA0000
#define AT91C_IO_SIZE		(0xFFFFFFFF - AT91C_IO_PHYS_BASE + 1)
#define AT91C_IO_VIRT_BASE	(0xFF000000 - AT91C_IO_SIZE)

 /* Convert a physical IO address to virtual IO address */
#define AT91_IO_P2V(x)	((x) - AT91C_IO_PHYS_BASE + AT91C_IO_VIRT_BASE)

/*
 * Virtual to Physical Address mapping for IO devices.
 */
#define AT91C_VA_BASE_SYS	AT91_IO_P2V(AT91C_BASE_SYS)
#define AT91C_VA_BASE_SPI	AT91_IO_P2V(AT91C_BASE_SPI)
#define AT91C_VA_BASE_SSC2	AT91_IO_P2V(AT91C_BASE_SSC2)
#define AT91C_VA_BASE_SSC1	AT91_IO_P2V(AT91C_BASE_SSC1)
#define AT91C_VA_BASE_SSC0	AT91_IO_P2V(AT91C_BASE_SSC0)
#define AT91C_VA_BASE_US3	AT91_IO_P2V(AT91C_BASE_US3)
#define AT91C_VA_BASE_US2	AT91_IO_P2V(AT91C_BASE_US2)
#define AT91C_VA_BASE_US1	AT91_IO_P2V(AT91C_BASE_US1)
#define AT91C_VA_BASE_US0	AT91_IO_P2V(AT91C_BASE_US0)
#define AT91C_VA_BASE_EMAC	AT91_IO_P2V(AT91C_BASE_EMAC)
#define AT91C_VA_BASE_TWI	AT91_IO_P2V(AT91C_BASE_TWI)
#define AT91C_VA_BASE_MCI	AT91_IO_P2V(AT91C_BASE_MCI)
#define AT91C_VA_BASE_UDP	AT91_IO_P2V(AT91C_BASE_UDP)
#define AT91C_VA_BASE_TCB1	AT91_IO_P2V(AT91C_BASE_TCB1)
#define AT91C_VA_BASE_TCB0	AT91_IO_P2V(AT91C_BASE_TCB0)

/*
 * Pointers to peripheral structures.
 */
#define AT91_SYS		((AT91PS_SYS) AT91C_VA_BASE_SYS)

/* Internal SRAM */
#define AT91C_BASE_SRAM		0x00200000	/* Internal SRAM base address */
#define AT91C_SRAM_SIZE		0x00004000	/* Internal SRAM SIZE (16Kb) */

/* Serial ports */
#define AT91C_NR_UART		5		/* 4 USART3's and one DBGU port */

/* FLASH */
#define AT91_FLASH_BASE		0x10000000	// NCS0: Flash physical base address

/* SDRAM */
#define AT91_SDRAM_BASE		0x20000000	// NCS1: SDRAM physical base address

/* SmartMedia */
#define AT91_SMARTMEDIA_BASE	0x40000000	// NCS3: Smartmedia physical base address

/* Multi-Master Memory controller */
#define AT91_UHP_BASE		0x00300000	// USB Host controller

 /* Definition of interrupt priority levels */
#define AT91C_AIC_PRIOR_0 AT91C_AIC_PRIOR_LOWEST
#define AT91C_AIC_PRIOR_1 ((unsigned int) 0x1)
#define AT91C_AIC_PRIOR_2 ((unsigned int) 0x2)
#define AT91C_AIC_PRIOR_3 ((unsigned int) 0x3)
#define AT91C_AIC_PRIOR_4 ((unsigned int) 0x4)
#define AT91C_AIC_PRIOR_5 ((unsigned int) 0x5)
#define AT91C_AIC_PRIOR_6 ((unsigned int) 0x6)
#define AT91C_AIC_PRIOR_7 AT91C_AIC_PRIOR_HIGEST

/* Clocks */
#define AT91C_SLOW_CLOCK	32768		/* slow clock */

#ifndef __ASSEMBLY__

/* Main clock; determined on startup */
extern unsigned long at91_main_clock;

/* System clock; calculated on startup. */
extern unsigned long at91_master_clock;

/* PLL B setting for USB; calculated on startup. */
extern unsigned long at91_pllb_init;

#endif

#define AT91C_SLOW_CLOCK	32768		/* slow clock */

/* FLASH */
#define AT91_FLASH_BASE		0x10000000	// NCS0: Flash physical base address

/* SDRAM */
#define AT91_SDRAM_BASE		0x20000000	// NCS1: SDRAM physical base address

/* SmartMedia */
#define AT91_SMARTMEDIA_BASE	0x40000000	// NCS3: Smartmedia physical base address

/* Multi-Master Memory controller */
#define AT91_UHP_BASE		0x00300000	// USB Host controller

/* Peripheral interrupt configuration */
#define AT91_SMR_FIQ	(AT91C_AIC_PRIOR_HIGHEST | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Advanced Interrupt Controller (FIQ)
#define AT91_SMR_SYS	(AT91C_AIC_PRIOR_HIGHEST | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// System Peripheral
#define AT91_SMR_PIOA	(AT91C_AIC_PRIOR_LOWEST	 | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Parallel IO Controller A
#define AT91_SMR_PIOB	(AT91C_AIC_PRIOR_LOWEST  | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Parallel IO Controller B
#define AT91_SMR_PIOC	(AT91C_AIC_PRIOR_LOWEST  | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Parallel IO Controller C
#define AT91_SMR_PIOD	(AT91C_AIC_PRIOR_LOWEST  | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Parallel IO Controller D
#define AT91_SMR_US0	(AT91C_AIC_PRIOR_6       | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// USART 0
#define AT91_SMR_US1	(AT91C_AIC_PRIOR_6       | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// USART 1
#define AT91_SMR_US2	(AT91C_AIC_PRIOR_6       | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// USART 2
#define AT91_SMR_US3	(AT91C_AIC_PRIOR_6       | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// USART 3
#define AT91_SMR_MCI	(AT91C_AIC_PRIOR_LOWEST  | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Multimedia Card Interface
#define AT91_SMR_UDP	(AT91C_AIC_PRIOR_4       | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// USB Device Port
#define AT91_SMR_TWI	(AT91C_AIC_PRIOR_LOWEST  | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Two-Wire Interface
#define AT91_SMR_SPI	(AT91C_AIC_PRIOR_6       | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Serial Peripheral Interface
#define AT91_SMR_SSC0	(AT91C_AIC_PRIOR_5       | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Serial Synchronous Controller 0
#define AT91_SMR_SSC1	(AT91C_AIC_PRIOR_5       | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Serial Synchronous Controller 1
#define AT91_SMR_SSC2	(AT91C_AIC_PRIOR_5       | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Serial Synchronous Controller 2
#define AT91_SMR_TC0	(AT91C_AIC_PRIOR_LOWEST  | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Timer Counter 0
#define AT91_SMR_TC1	(AT91C_AIC_PRIOR_LOWEST  | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Timer Counter 1
#define AT91_SMR_TC2	(AT91C_AIC_PRIOR_LOWEST  | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Timer Counter 2
#define AT91_SMR_TC3	(AT91C_AIC_PRIOR_LOWEST  | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Timer Counter 3
#define AT91_SMR_TC4	(AT91C_AIC_PRIOR_LOWEST  | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Timer Counter 4
#define AT91_SMR_TC5	(AT91C_AIC_PRIOR_LOWEST  | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Timer Counter 5
#define AT91_SMR_UHP	(AT91C_AIC_PRIOR_3       | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// USB Host port
#define AT91_SMR_EMAC	(AT91C_AIC_PRIOR_3       | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Ethernet MAC
#define AT91_SMR_IRQ0	(AT91C_AIC_PRIOR_LOWEST  | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Advanced Interrupt Controller (IRQ0)
#define AT91_SMR_IRQ1	(AT91C_AIC_PRIOR_LOWEST  | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Advanced Interrupt Controller (IRQ1)
#define AT91_SMR_IRQ2	(AT91C_AIC_PRIOR_LOWEST  | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Advanced Interrupt Controller (IRQ2)
#define AT91_SMR_IRQ3	(AT91C_AIC_PRIOR_LOWEST  | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Advanced Interrupt Controller (IRQ3)
#define AT91_SMR_IRQ4	(AT91C_AIC_PRIOR_LOWEST  | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Advanced Interrupt Controller (IRQ4)
#define AT91_SMR_IRQ5	(AT91C_AIC_PRIOR_LOWEST  | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Advanced Interrupt Controller (IRQ5)
#define AT91_SMR_IRQ6	(AT91C_AIC_PRIOR_LOWEST  | AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE)	// Advanced Interrupt Controller (IRQ6)

/*
 * Serial port configuration.
 *    0 .. 3 = USART0 .. USART3
 *    4      = DBGU
 */
#define AT91C_UART_MAP		{ 4, 1, -1, -1, -1 }	/* ttyS0, ..., ttyS4 */
#define AT91C_CONSOLE		0			/* ttyS0 */

#endif
