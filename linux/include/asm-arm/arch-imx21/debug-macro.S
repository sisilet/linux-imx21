/* linux/include/asm-arm/arch-imx21/debug-macro.S
 *
 * Debugging macro include header
 *
 *  Copyright (C) 1994-1999 Russell King
 *  Moved from linux/arch/arm/kernel/debug.S by Ben Dooks
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

		.macro	addruart,rx
	        mrc     p15, 0, \rx, c1, c0
                tst     \rx, #1                 @ MMU enabled?
                moveq   \rx, #0x10000000        @ Physical
                movne   \rx, #0xe4000000        @ Virtual
                orr     \rx, \rx, #0xa000
                orr     \rx, \rx, #0x40
		.endm

		.macro	senduart,rd,rx
		str	\rd, [\rx]	@ TXDATA
		.endm

		.macro	waituart,rd,rx
		.endm

		.macro	busyuart,rd,rx
1002:		ldrb    \rd, [\rx, #0x54]
		@ Check TRDY in USR1_1 (phys=0x1000a094)
                and     \rd, \rd, #0x2000  
                cmp     \rd, #0x2000
                bne     1002b
		.endm
