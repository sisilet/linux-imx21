/*
 * linux/include/asm-arm/arch-pxa/csb226.h
 *
 * (c) 2003 Robert Schwebel <r.schwebel@pengutronix.de>, Pengutronix
 *  
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _INCLUDE_ASMARM_ARCHPXA_CSB226_H_
#define _INCLUDE_ASMARM_ARCHPXA_CSB226_H_

/* 
 * GPIOs 
 */
#define GPIO_CSB226_ETH		14

/* 
 * ethernet chip (CS8900) 
 */
#define CSB226_ETH_PHYS		PXA_CS2_PHYS	/* 0x08000000 */
#define CSB226_ETH_VIRT		(0xf8000000)
#define CSB226_ETH_SIZE		(1*1024*1024)
#define CSB226_ETH_IRQ		IRQ_GPIO(GPIO_CSB226_ETH)
#define CSB226_ETH_IRQ_EDGE	GPIO_RISING_EDGE

/* CF slot 0 */
#define CSB226_CF0_IRQ   10
#define CSB226_CF0_POWER 11
#define CSB226_CF0_CD    9
#define CSB226_CF0_RESET 13


#endif
