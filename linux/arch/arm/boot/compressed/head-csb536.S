/* 
 * linux/arch/arm/boot/compressed/head-csb536.S
 * 
 * Copyright (C) 2005 Jay Monkman <jtm@lopingdog.com>
 * Copyright (C) 2000 Steve Hill <sjhill@cotw.com>
 * 
 * Some code borrowed from Nicolas Pitre's 'head-sa1100.S' file. This
 * is merged with head.S by the linker.
 */

#include <linux/config.h>
#include <asm/mach-types.h>

#ifndef CONFIG_MACH_CSB536
#error What am I doing here...
#endif

	.section        ".start", "ax"

__CSB536_start:
	b	2f
1:	.word	#MACH_TYPE_CSB536	@ Set architecture ID
2:	
	ldr	r7, 1b


	/* 
	 * umon doesn't enable the MCU PLL, so we do it here -
	 *    set to 196 MHz.
	 */
	b	4f
1:	.word   0x0021b000    /* CSCR */
2:	.word	0x003f1437    /* Value for MPCTL0 - 196 MHz */
3:	.word	0x00200001    /* Mask for CSCR, to enable MCU PLL */
4:
	ldr	r1, 1b
	ldr	r2, 2b
	str	r2, [r1, #4]  /* Write to MPCTL0 */

	ldr	r2, [r1, #0]  /* Read CSCR */
	ldr	r3, 3b
	orr	r2, r2, r3
	str	r2, [r1, #0]  /* Write CSCR */
