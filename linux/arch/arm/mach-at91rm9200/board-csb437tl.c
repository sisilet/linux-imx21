/*
 * linux/arch/arm/mach-at91rm9200/board-csb437tl.c
 *
 *  Copyright (C) 2005 Bill Gatliff <bgat@billgatliff.com>
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

#include <asm/hardware/clock.h>
#include <asm/arch/hardware.h>
#include <asm/mach/serial_at91rm9200.h>
#include <asm/arch/board.h>

#include "generic.h"

static void __init csb437tl_init_irq(void)
{
	/* Initialize AIC controller */
	at91rm9200_init_irq(NULL);

	/* Set up the GPIO interrupts */
	at91_gpio_irq_setup(BGA_GPIO_BANKS);
}

/* Serial port configuration.
 *    0 .. 3 = USART0 .. USART3
 *    4      = DBGU
 */
#define CSB437TL_UART_MAP { 4, 1, -1, -1, -1 }
#define CSB437TL_SERIAL_CONSOLE	0

static void __init csb437tl_map_io(void)
{
	int serial[AT91C_NR_UART] = CSB437TL_UART_MAP;
	int i;

	at91rm9200_map_io();
	
	/* Initialize clocks; 3.6864 MHz crystal ("0" lets the clock
           code determine this for itself) */
	at91_clock_init(3686400);

#ifdef CONFIG_SERIAL_AT91
	at91_console_port = CSB437TL_SERIAL_CONSOLE;
	memcpy(at91_serial_map, serial, sizeof(serial));

	/* Register UARTs */
	for (i = 0; i < AT91C_NR_UART; i++) {
		if (serial[i] >= 0)
			at91_register_uart(i, serial[i]);
	}
#endif
}

static struct at91_eth_data __initdata csb437tl_eth_data = {
	.phy_irq_pin	= AT91_PIN_PC2,
	.is_rmii	= 0,
};

static struct at91_udc_data __initdata csb437tl_udc_data = {
        .vbus_pin       = AT91_PIN_PB28,
	.pullup_pin	= AT91_PIN_PB1,
};

static struct at91_cf_data __initdata csb437tl_cf_data = {
	.det_pin	= AT91_PIN_PC3,
	.irq_pin	= AT91_PIN_PA19,
	.vcc_pin	= AT91_PIN_PD0,
	.rst_pin	= AT91_PIN_PD2,
};

static struct at91_mmc_data __initdata csb437tl_mmc_data = {
	.det_pin	= AT91_PIN_PD5,
	.is_b		= 0,
#warning TODO: reenable wire4 after fixing problems in sd/mmc driver
	.wire4		= 1,
	.wp_pin		= AT91_PIN_PD6,
        .vcc_pin        = AT91_PIN_PD3,
};


#define CONFIG_VOYAGER_GX_VRAM  0x30000000
#define CONFIG_VOYAGER_GX_VRAM_LEN 0x400000

#define CONFIG_VOYAGER_GX_REGS  (CONFIG_VOYAGER_GX_VRAM + 0x3e00000)
#define CONFIG_VOYAGER_GX_REGS_LEN 0x00200000

#if defined(CONFIG_FB_VOYAGER_GX)

static struct resource at91_panel_resources[] = {
  [0] = {
    /* framebuffer memory base address, size */
    .start              = CONFIG_VOYAGER_GX_VRAM,
    .end                = CONFIG_VOYAGER_GX_VRAM
                                + CONFIG_VOYAGER_GX_VRAM_LEN - 1,
    .flags              = IORESOURCE_MEM,
  },
  [1] = {
    /* framebuffer register base address, size */
    .start              = CONFIG_VOYAGER_GX_REGS,
    .end                = CONFIG_VOYAGER_GX_REGS
                                + CONFIG_VOYAGER_GX_REGS_LEN - 1,
    .flags              = IORESOURCE_MEM,
  },
};

static struct platform_device at91_panel_dev = {
  .name                 = "voyager_panel_fb",
  .id                   = -1,
  .num_resources        = ARRAY_SIZE(at91_panel_resources),
  .resource             = at91_panel_resources,
};
#endif



#warning TODO: make this conditional on a CONFIG_x
#warning TODO: replace magic numbers here with macros
static struct resource ac97_dev_resources[] = {
  [0] = {
    .start              = CONFIG_VOYAGER_GX_REGS + 0xa0000,
    .end                = CONFIG_VOYAGER_GX_REGS + 0xa01fff,
    .flags              = IORESOURCE_MEM,
  },
};

static struct platform_device ac97_dev = {
  .name = "sm501_ac97",
  .id = -1,
  .num_resources = 1,
  .resource = ac97_dev_resources,
};



#warning TODO: make this conditional on a CONFIG_x
#warning TODO: replace magic numbers here with macros
static struct resource sm501_8051_dev_resources[] = {
  [0] = {
    .start              = CONFIG_VOYAGER_GX_REGS + 0xb0000,
    .end                = CONFIG_VOYAGER_GX_REGS + 0xcffff,
    .flags              = IORESOURCE_MEM,
  },
};

static struct platform_device sm501_8051_dev = {
  .name = "sm501_8051",
  .id = -1,
  .num_resources = 1,
  .resource = sm501_8051_dev_resources,
};




#if (CONFIG_MACH_CSB437TL_REV == 2)
#define LED AT91_PIN_PB0
#define USBHVCC0 AT91_PIN_PA21
#define USBHVCC1 AT91_PIN_PA22
#else
#define LED AT91_PIN_PB2
#define USBHVCC0 AT91_PIN_PA22
#define USBHVCC1 AT91_PIN_PA23
#endif

static void __init csb437tl_board_init(void)
{
        /* activate the LED */
        at91_set_gpio_direction(LED, 0, 1);

#if (CONFIG_MACH_CSB437TL_REV == 4)
        /* power up vcc5 */
        at91_set_gpio_direction(AT91_PIN_PB0, 0, 1);
#endif

        /* power up USBH ports */
        /* TODO: let USBH controller manage USB power */
#if (CONFIG_MACH_CSB437TL_REV == 2)
        /* the P2 uses active-low USBHVCC switches */
        at91_set_gpio_direction(USBHVCC0, 0, 0);
        at91_set_gpio_direction(USBHVCC1, 0, 0);
#else
        at91_set_gpio_direction(USBHVCC0, 0, 1);
        at91_set_gpio_direction(USBHVCC1, 0, 1);
#endif
        
#if defined(CONFIG_FB)
        {
          struct clk *plla, *pck0;
          
          plla = clk_get(NULL, "plla");
          pck0 = clk_get(NULL, "pck0");
          if (plla && pck0) {
            clk_set_rate(plla, 48000000);
            clk_set_parent(pck0, plla);
            clk_set_rate(pck0, 24000000);
            at91_set_A_periph(AT91_PIN_PB27, 0);
          }
        }
#endif

	/* Ethernet */
	at91_add_device_eth(&csb437tl_eth_data);
	/* USB Host */
	at91_add_device_usbh();
	/* USB Device */
	at91_add_device_udc(&csb437tl_udc_data);
	/* Compact Flash */
	at91_set_gpio_direction(AT91_PIN_PB22, 1, 1);	/* IOIS16 */
	at91_add_device_cf(&csb437tl_cf_data);
	/* MMC */
	at91_add_device_mmc(&csb437tl_mmc_data);

#if defined(CONFIG_FB_VOYAGER_GX)
       platform_device_register(&at91_panel_dev);
#endif
       platform_device_register(&sm501_8051_dev);
       platform_device_register(&ac97_dev);

}

MACHINE_START(CSB437TL, "Cogent CSB437TL")
	MAINTAINER("Bill Gatliff <bgat@billgatliff.com>")
	BOOT_MEM(AT91_SDRAM_BASE, AT91C_BASE_SYS, AT91C_VA_BASE_SYS)
	BOOT_PARAMS(AT91_SDRAM_BASE + 0x100)
	.timer	= &at91rm9200_timer,
	MAPIO(csb437tl_map_io)
	INITIRQ(csb437tl_init_irq)
	INIT_MACHINE(csb437tl_board_init)
MACHINE_END
