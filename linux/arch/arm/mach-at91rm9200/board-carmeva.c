/*
 * linux/arch/arm/mach-at91rm9200/board-carmeva.c
 *
 *  Copyright (c) 2005 Peer Georgi
 *  		       Conitec Datasystems
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
#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/device.h>

#include <asm/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <asm/arch/hardware.h>
#include <asm/mach/serial_at91rm9200.h>
#include <asm/arch/board.h>

#include "generic.h"

static void __init carmeva_init_irq(void)
{
	/* Initialize AIC controller */
	at91rm9200_init_irq(NULL);

	/* Set up the GPIO interrupts */
	at91_gpio_irq_setup(BGA_GPIO_BANKS);
}

/*
 * Serial port configuration.
 *    0 .. 3 = USART0 .. USART3
 *    4      = DBGU
 */
#define CARMEVA_UART_MAP		{ 4, 1, -1, -1, -1 }	/* ttyS0, ..., ttyS4 */
#define CARMEVA_SERIAL_CONSOLE		0			/* ttyS0 */

/* Macros for PLL setup (maybe to hardware.h) */
#define AT91_PLL_VALUE_FAST(div,mult)	((div) | (1024 << 8) | ((mult-1) << 16) | (1 << 15) | (1 << 29))
#define AT91_PLLB_VALUE_USB(div,mult)	((div) | (63 << 8) | ((mult-1) << 16) | (1 << 28))

static void __init carmeva_map_io(void)
{
	int serial[AT91C_NR_UART] = CARMEVA_UART_MAP;
	int i;

	at91rm9200_map_io();
	
	/* Initialize clocks
	 * Theese are the initial settings made by LdBigAppUSB or LdLinux++
	 * To modify them you have to reconfigure theese tools.
	 * (a command line interface may be useful)
	 */
	at91_main_clock = 180000000;				/* 20MHz / 8 * 72 = 180MHz */
	at91_master_clock = 60000000;				/* 180MHz / 3 = 60MHz */
	at91_pllb_clock = AT91_PLLB_VALUE_USB(45, 216);		/* ((20MHz / 45) * 216) = 96Mhz */

#ifdef CONFIG_SERIAL_AT91
	at91_console_port = CARMEVA_SERIAL_CONSOLE;
	memcpy(at91_serial_map, serial, sizeof(serial));
	
	/* Register UARTs */
	for (i = 0; i < AT91C_NR_UART; i++) {
		if (at91_serial_map[i] >= 0)
			at91_register_uart(i, at91_serial_map[i]);
	}
#endif
}

static struct at91_eth_data __initdata carmeva_eth_data = {
	.phy_irq_pin	= AT91_PIN_PC4,
	.is_rmii	= 1,
};

static struct at91_usbh_data __initdata carmeva_usbh_data = {
	.ports		= 2,
};

static struct at91_udc_data __initdata carmeva_udc_data = {
	.vbus_pin	= AT91_PIN_PD12,
	.pullup_pin	= AT91_PIN_PD9,
};

/* FIXME: user dependend */
// static struct at91_cf_data __initdata carmeva_cf_data = {
//	.det_pin	= AT91_PIN_PB0,
//	.rst_pin	= AT91_PIN_PC5,
	// .irq_pin	= ... not connected
	// .vcc_pin	= ... always powered
// };

static struct at91_mmc_data __initdata carmeva_mmc_data = {
	.is_b		= 0,
	.wire4		= 1,
};

static void __init carmeva_board_init(void)
{
	/* Ethernet */
	at91_add_device_eth(&carmeva_eth_data);
	/* USB Host */
	at91_add_device_usbh(&carmeva_usbh_data);
	/* USB Device */
	at91_add_device_udc(&carmeva_udc_data);
	/* Compact Flash */
//	at91_add_device_cf(&carmeva_cf_data);
	/* MMC */
	at91_add_device_mmc(&carmeva_mmc_data);
}

MACHINE_START(CARMEVA, "Carmeva")
	MAINTAINER("Conitec Datasystems")
	BOOT_MEM(AT91_SDRAM_BASE, AT91C_BASE_SYS, AT91C_VA_BASE_SYS)
	BOOT_PARAMS(AT91_SDRAM_BASE + 0x100)
	.timer	= &at91rm9200_timer,

	MAPIO(carmeva_map_io)
	INITIRQ(carmeva_init_irq)
	INIT_MACHINE(carmeva_board_init)
MACHINE_END
