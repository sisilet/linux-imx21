/*
 * linux/include/asm-arm/arch-pxa/uncompress.h
 *
 * Author:	Nicolas Pitre
 * Copyright:	(C) 2001 MontaVista Software Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define FFUART		((volatile unsigned long *)0x40100000)
#define BTUART		((volatile unsigned long *)0x40200000)
#define STUART		((volatile unsigned long *)0x40700000)


/* TODO: find a common location for this and the equivalent
   definition in drivers/serial/pxa.c */
#if defined(CONFIG_PXA_UART_ASSIGNMENTS)
#define CONFIG_ASSIGNMENT_FFUART CONFIG_PXA_FFUART_ASSIGNMENT
#define CONFIG_ASSIGNMENT_BTUART CONFIG_PXA_BTUART_ASSIGNMENT
#define CONFIG_ASSIGNMENT_STUART CONFIG_PXA_STUART_ASSIGNMENT
#else
#define CONFIG_ASSIGNMENT_FFUART 0
#define CONFIG_ASSIGNMENT_BTUART 1
#define CONFIG_ASSIGNMENT_STUART 2
#endif


static volatile unsigned long * const uart_table[] = {
  [CONFIG_ASSIGNMENT_FFUART] = FFUART,
  [CONFIG_ASSIGNMENT_BTUART] = BTUART,
  [CONFIG_ASSIGNMENT_STUART] = STUART
};

#if 0
#if defined(CONFIG_MACH_CSB625)
#  if (CONFIG_MACH_CSB625_REV >= 3)
#    define UART            STUART
#  else
#    define UART            FFUART
#  endif

#else
#  define UART		FFUART
#endif
#else

#define UART (uart_table[0])
#endif

static __inline__ void putc(char c)
{
	while (!(UART[5] & 0x20));
	UART[0] = c;
}

/*
 * This does not append a newline
 */
static void putstr(const char *s)
{
	while (*s) {
		putc(*s);
		if (*s == '\n')
			putc('\r');
		s++;
	}
}

/*
 * nothing to do
 */
#define arch_decomp_setup()
#define arch_decomp_wdog()
