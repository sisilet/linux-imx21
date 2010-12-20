/*
 * linux/include/asm-arm/arch-pxa/csb625.h
 *
 * (c) 2005 Bill Gatliff <bgat@billgatliff.com>
 *  
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _INCLUDE_ASMARM_ARCHPXA_CSB625_H_
#define _INCLUDE_ASMARM_ARCHPXA_CSB625_H_

/* 
 * GPIOs 
 */
#define GPIO_CSB625_ETH		14

/* 
 * ethernet chip (CS8900) 
 */
#define CSB625_ETH_PHYS		PXA_CS2_PHYS	/* 0x08000000 */
#define CSB625_ETH_VIRT		(0xf8000000)
#define CSB625_ETH_SIZE		(1*1024*1024)
#define CSB625_ETH_IRQ		IRQ_GPIO(GPIO_CSB625_ETH)
#define CSB625_ETH_IRQ_EDGE	GPIO_RISING_EDGE

/*
 * USB disconnect interrupt & USB on/off GPIO
 */
#define CSB625_USB_PHYS                 PXA_CS5_PHYS
#define CSB625_USB_VIRT 		(0xf4000000)
#define CSB625_USB_IRQ                  22

/* CF slot 0 */
#define CSB625_CF0_IRQ   10
#define CSB625_CF0_POWER 11
#define CSB625_CF0_CD    12
#define CSB625_CF0_RESET 13


#endif
