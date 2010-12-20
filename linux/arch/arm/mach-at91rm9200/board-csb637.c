/*
 * linux/arch/arm/mach-at91rm9200/board-csb637.c
 *
 *  Copyright (C) 2005 SAN People
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
#include <linux/delay.h>

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

#if defined(CONFIG_FB_S1D13XXX)
#include <video/s1d13xxxfb.h>
#endif

#include "generic.h"

static void __init csb637_init_irq(void)
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
#define CSB637_UART_MAP		{ 4, 1, -1, -1, -1 }
#define CSB637_SERIAL_CONSOLE	0

static void __init csb637_map_io(void)
{
	int serial[AT91C_NR_UART] = CSB637_UART_MAP;
	int i;

	at91rm9200_map_io();

	/* Initialize clocks; 3.6864 MHz crystal ("0" lets the clock
           code determine this for itself */
	at91_clock_init(3686400);

#ifdef CONFIG_SERIAL_AT91
	at91_console_port = CSB637_SERIAL_CONSOLE;
	memcpy(at91_serial_map, serial, sizeof(serial));

	/* Register UARTs */
	for (i = 0; i < AT91C_NR_UART; i++) {
		if (serial[i] >= 0)
			at91_register_uart(i, serial[i]);
	}
#endif
}

#if defined(CONFIG_ARM_AT91_ETHER)
static struct at91_eth_data __initdata csb637_eth_data = {
	.phy_irq_pin	= AT91_PIN_PC0,
	.is_rmii	= 0,
};
#endif

#if defined(CONFIG_USB_GADGET_AT91)
static struct at91_udc_data __initdata csb637_udc_data = {
	.vbus_pin     = AT91_PIN_PB28,
	.pullup_pin   = AT91_PIN_PB1,
};
#endif

#if defined(CONFIG_AT91_CF)
static struct at91_cf_data __initdata csb637_cf_data = {
	.det_pin	= AT91_PIN_PC3,
	.irq_pin	= AT91_PIN_PA20,
	/* .vcc_pin	= AT91_PIN_PC14, */
	.rst_pin	= AT91_PIN_PC9,
};
#endif

#if defined(CONFIG_MMC_AT91RM9200)
static struct at91_mmc_data __initdata csb637_mmc_data = {
	.det_pin	= AT91_PIN_PC4,
	.is_b		= 0,
	.wire4		= 1,
	.wp_pin		= AT91_PIN_PC5,
        .vcc_pin        = AT91_PIN_PC2,
};
#endif

#if defined(CONFIG_FB_S1D13XXX)

#define CONFIG_FB_VRAM  0x30200000
#define CONFIG_FB_VRAM_LEN 0x200000

#define CONFIG_FB_REGS  0x30000000
#define CONFIG_FB_REGS_LEN 0x400

static void __init at91_init_video(void)
{
}

static const struct s1d13xxxfb_regval at91_s1dfb_initregs[] = {
  /* TODO: fill this in */
};

static struct s1d13xxxfb_pdata at91_s1dfb_pdata = {
  	.initregs		= at91_s1dfb_initregs,
	.initregssize		= ARRAY_SIZE(at91_s1dfb_initregs),
	.platform_init_video	= at91_init_video,
};

static u64 s1dfb_dmamask = 0xffffffffUL;

static struct resource at91_panel_resources[] = {
  [0] = {
    /* framebuffer memory (vram) base address, size */
    .start              = CONFIG_FB_VRAM,
    .end                = CONFIG_FB_VRAM + CONFIG_FB_VRAM_LEN - 1,
    .flags              = IORESOURCE_MEM,
  },
  [1] = {
    /* framebuffer register base address, size */
    .start              = CONFIG_FB_REGS,
    .end                = CONFIG_FB_REGS + CONFIG_FB_REGS_LEN - 1,
    .flags              = IORESOURCE_MEM,
  },
};

static struct platform_device at91_panel_dev = {
  .name                 = S1D_DEVICENAME,
  .id                   = -1,
  .dev                  = {
    .dma_mask = &s1dfb_dmamask,
    .coherent_dma_mask = 0xffffffff,
    /*    .platform_data = &at91_s1dfb_pdata,*/
  },
  .num_resources        = ARRAY_SIZE(at91_panel_resources),
  .resource             = at91_panel_resources,
};
#endif

static void __init csb637_board_init(void)
{
        /* power up vcc5 */
        at91_set_gpio_direction(AT91_PIN_PB0, 0, 1);
        mdelay(10);

#if defined(CONFIG_FB_S1D13XXX)
        platform_device_register(&at91_panel_dev);
#endif

#if defined(CONFIG_ARM_AT91_ETHER)
	at91_add_device_eth(&csb637_eth_data);
#endif
#if defined(CONFIG_USB_OHCI_HCD)
	at91_add_device_usbh();
#endif
#if defined(CONFIG_USB_GADGET_AT91)
	at91_add_device_udc(&csb637_udc_data);
#endif
#if defined(CONFIG_AT91_CF)
        at91_add_device_cf(&csb637_cf_data);
#endif
#if defined(CONFIG_MMC_AT91RM9200)
	at91_add_device_mmc(&csb637_mmc_data);
#endif
}

MACHINE_START(CSB637, "Cogent CSB637")
MAINTAINER("Bill Gatliff <bgat@billgatliff.com>")
BOOT_MEM(AT91_SDRAM_BASE, AT91C_BASE_SYS, AT91C_VA_BASE_SYS)
BOOT_PARAMS(AT91_SDRAM_BASE + 0x100)
.timer	= &at91rm9200_timer,
MAPIO(csb637_map_io)
INITIRQ(csb637_init_irq)
INIT_MACHINE(csb637_board_init)
MACHINE_END
