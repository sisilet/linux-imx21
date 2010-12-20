/*
 *  linux/include/asm-arm/arch-imx21/uncompress.h
 *
 *
 *
 *  Copyright (C) 1999 ARM Limited
 *  Copyright (C) Shane Nay (shane@minirl.com)
 *
 * Modified By: Ron Melvin (ron.melvin@timesys.com)
 * Copyright (C) 2005 TimeSys Corporation 
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


#if 0		// TODO: use similar code (automatically pick port

#define UART_TX  ((char *) (0xcc200000))
#define UART_LSR ((char *) (0xcc200005))
#if 1
#define UARTDELAY { int x = 8000;		\
	while (x--);				\
    }	
#else	
#define UARTDELAY {				\
	while ( ((*UART_LSR) & 0x20) ) {	\
	    ;					\
	}					\
    }
#endif
void dputc(char c)
{
    UARTDELAY;
    *UART_TX = c;
}

static void puts(const char *s)
{
    while (*s) {
	dputc(*s);
	if ( *s == '\n' ) {
	    dputc('\r');
	}
	s++;
    }
}

static void
putstr(const char *s)
{
    puts(s);
}

void puthex(int x)
{
    int i;
    char tmp[8];
    
    for ( i = 0; i <= 7; i++ ) {
	int y = x & 0xf;
	if ( y <= 9 )
	    tmp[i] = y + '0';	    
	else
	    tmp[i] = y + 'a' - 10;
	
	x >>= 4;
    }

    puts("0x");
    for ( i = 7; i >= 0; i-- ) {
	dputc(tmp[i]);
    }
    
}

void pval(char *s, int v)
{
    puts(s);
    puts(": ");
    puthex(v);
    puts("\n");
    
}


#else		// orig 2.6 mx1 code:

#define UART(x) (*(volatile unsigned long *)(serial_port + (x)))

#define UART1_BASE 0x1000a000
#define UART2_BASE 0x1000b000
#define USR2 0x98
#define USR2_TXFE (1<<14)
#define TXR  0x40
#define UCR1 0x80
#define UCR1_UARTEN 1

/*
 * The following code assumes the serial port has already been
 * initialized by the bootloader.  We search for the first enabled
 * port in the most probable order.  If you didn't setup a port in
 * your bootloader then nothing will appear (which might be desired).
 *
 * This does not append a newline
 */
static void
putstr(const char *s)
{
	unsigned long serial_port;

	do {
		serial_port = UART1_BASE;
		if ( UART(UCR1) & UCR1_UARTEN )
			break;
		serial_port = UART2_BASE;
		if ( UART(UCR1) & UCR1_UARTEN )
			break;
		return;
	} while(0);

	while (*s) {
		while ( !(UART(USR2) & USR2_TXFE) )
			barrier();

		UART(TXR) = *s;

		if (*s == '\n') {
			while ( !(UART(USR2) & USR2_TXFE) )
				barrier();

			UART(TXR) = '\r';
		}
		s++;
	}
}

void puthex(int x)
{
    int i;
    char tmp[8];
    char num[11];
    num[0]  = '0';
    num[1]  = 'x';
    num[10] = 0;

    for ( i = 0; i <= 7; i++ ) {
	int y = x & 0xf;
	if ( y <= 9 )
	    tmp[i] = y + '0';	    
	else
	    tmp[i] = y + 'a' - 10;
	
	x >>= 4;
    }

    for ( i = 7; i >= 0; i-- ) {
        num[9-i] = tmp[i];
    }

    putstr(num);
    
}
#endif		// TMP UART CODE....

/*
 * nothing to do
 */
#define arch_decomp_setup()

#define arch_decomp_wdog()
