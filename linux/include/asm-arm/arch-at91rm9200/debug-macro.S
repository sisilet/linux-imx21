/* 
 * include/asm-arm/arch-at91rm9200/debug-macro.S
 *
 *  Copyright (C) 2003-2005 SAN People
 *
 *  Copyright (C) 2003-2005 SAN People
 *
 * Debugging macro include header
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <asm/hardware.h>

	.macro	addruart,rx
	mrc	p15, 0, \rx, c1, c0
	tst	\rx, #1				@ MMU enabled?
	ldreq	\rx, =AT91C_BASE_SYS		@ System peripherals (phys address)
	ldrne	\rx, =AT91C_VA_BASE_SYS		@ System peripherals (virt address)
	.endm

	.macro	senduart,rd,rx
	strb	\rd, [\rx, #DBGU_THR]		@ DBGU_THR
	.endm

	.macro	waituart,rd,rx
1001:	ldr	\rd, [\rx, #DBGU_CSR]		@ DBGU_CSR
	tst	\rd, #AT91C_DBGU_TXRDY		@ DBGU_TXRDY = 1 when ready to transmit
	beq	1001b
	.endm

	.macro	busyuart,rd,rx
1001:	ldr	\rd, [\rx, #DBGU_CSR]		@ DBGU_CSR
	tst	\rd, #AT91C_DBGU_TXEMPTY	@ DBGU_TXEMPTY = 1 when transmission complete
	beq	1001b
	.endm

