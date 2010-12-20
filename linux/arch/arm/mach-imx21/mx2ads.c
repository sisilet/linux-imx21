/*
 * arch/arm/mach-imx21/mx2ads.c
 *
 * Initially based on:
 *	linux-2.6.7-imx/arch/arm/ mach-imx /scb9328.c
 *	Copyright (c) 2004 Sascha Hauer <sascha@saschahauer.de>
 *
 * 2004 (c) MontaVista Software, Inc.
 *
 * Modified By: Ron Melvin (ron.melvin@timesys.com)
 * Copyright (c) 2005 TimeSys Corporation 
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <asm/system.h>
#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/page.h>

#include <asm/mach/map.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <linux/interrupt.h>
#include "generic.h"
#include <asm/serial.h>
#include <asm/arch/imx21fb.h>

static struct resource enet_resources[] = {
	[0] = {
		.start	= IMX21_CS1_VIRT,
		.end	= IMX21_CS1_VIRT + 16,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
	        .start	= INT_UART3,
		.end	= INT_UART3,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device enet_device = {
	.name		= "imx21-ethernet",
	.num_resources	= ARRAY_SIZE(enet_resources),
	.resource	= enet_resources,
};

/* TODO, figure out correct IRQ */
static struct resource extuart_resources[] = {
	[0] = {
		.start	= IMX21_CS1_VIRT+0x200000,
		.end	= IMX21_CS1_VIRT+0x200000 + 16,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
	    .start	= 0,
		.end	= 0,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device extuart_device = {
	.name		= "imx21-ext-uart",
	.num_resources	= ARRAY_SIZE(extuart_resources),
	.resource	= extuart_resources,
};

/* TODO, figure out correct IRQ */
static struct resource cpuvers_resources[] = {
	[0] = {
		.start	= IMX21_CS1_VIRT+0x400000,
		.end	= IMX21_CS1_VIRT+0x400000 + 2,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
	    .start	= 0,
		.end	= 0,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device cpuvers_device = {
	.name		= "imx21-cpuvers",
	.num_resources	= ARRAY_SIZE(cpuvers_resources),
	.resource	= cpuvers_resources,
};

/* TODO, figure out correct IRQ */
static struct resource mx21cfg_resources[] = {
	[0] = {
		.start	= IMX21_CS1_VIRT+0x800000,
		.end	= IMX21_CS1_VIRT+0x800000 + 2,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
	    .start	= 0,
		.end	= 0,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device mx21cfg_device = {
	.name		= "imx21-mx21cfg",
	.num_resources	= ARRAY_SIZE(mx21cfg_resources),
	.resource	= mx21cfg_resources,
};

static struct platform_device *devices[] __initdata = {
	&enet_device,
	&extuart_device,
	&cpuvers_device,
	&mx21cfg_device,
};

static void __init
fixup_mx21(struct machine_desc *desc, struct tag *tags,
	   char **cmdline, struct meminfo *mi)

{
}

static void mx2ads_lcd_power(int on)
{
	if(on)
		MMIO_REG |= MMIO_LCDON;
	else
		MMIO_REG &= ~MMIO_LCDON;
}

// #define EXPERIMENTAL
#ifdef EXPERIMENTAL

/* LCD settings for Microtips MTG-F24160BFWNSEB-01 */
static struct imxfb_mach_info mx2ads_fb_info __initdata = {
	.pixclock	= 0, 		.bpp		= 1,
	.xres		= 240,		.yres		= 160,

	.hsync_len	= 0,		.vsync_len	= 0,
	.left_margin	= 0,		.upper_margin	= 0,
	.right_margin	= 0,		.lower_margin	= 0,

	.lpcr	= LPCR_PBSIZ(LPCR_PBSIZ_PANEL_BUS_WIDTH_4) | LPCR_BPIX(LPCR_BPIX_BITS_PER_PIXEL_1) | PCR_SCLKSEL | PCR_PCD(7),
	.lpccr	= 0,
	.lscr	= 0, 

	.lcd_power = mx2ads_lcd_power,
};
#else

/* LCD settings for Sharp LCD LQ035Q7DB02 */
static struct imxfb_mach_info mx2ads_fb_info __initdata = {
	.pixclock	= 0, 		.bpp		= 16,
	.xres		= 240,		.yres		= 320,

	.hsync_len	= 1,		.vsync_len	= 1,
	.left_margin	= 15,		.upper_margin	= 9,
	.right_margin	= 6,		.lower_margin	= 7,

	.lpcr	= LPCR_TFT | LPCR_COLOR | LPCR_PBSIZ(LPCR_PBSIZ_PANEL_BUS_WIDTH_8) | LPCR_BPIX(LPCR_BPIX_BITS_PER_PIXEL_16) | LPCR_PIXPOL | LPCR_OEPOL | LPCR_SCLKSEL | LPCR_SHARP | LPCR_PCD(7),
	.lpccr	= LPCCR_CLS_HI_WIDTH(169) | LPCCR_SCR(LPCCR_SCR_PIXEL_CLOCK) | LPCCR_CC_EN | LPCCR_PW(0xff),
	.lscr	= LSCR_PS_RISE_DELAY(0) | LSCR_CLS_RISE_DELAY(18) | LSCR_REV_TOGGLE_DELAY(3) | LSCR_GRAY2(0) | LSCR_GRAY1(0), 

	.lcd_power = mx2ads_lcd_power,
};
#endif

static void __init
mx2ads_init(void)
{
	/* Configure the system clocks */
	imx21_system_clk_init();	
	
	/* Set LCD display platform_data information*/
	set_imx_fb_info(&mx2ads_fb_info);
		
	/* Enable the LCD display */
	mx2ads_lcd_power(1);
	
	platform_add_devices(devices, ARRAY_SIZE(devices));
}

static struct map_desc mx2ads_io_desc[] __initdata = {
	/* virtual     physical    length      type */
	{IMX21_CS0_VIRT, IMX21_CS0_PHYS, IMX21_CS0_SIZE, MT_DEVICE},
	{IMX21_CS1_VIRT, IMX21_CS1_PHYS, IMX21_CS1_SIZE, MT_DEVICE},
};

static void __init
mx2ads_map_io(void)
{
	imx21_map_io();
	iotable_init(mx2ads_io_desc, ARRAY_SIZE(mx2ads_io_desc));
}

MACHINE_START(MX2ADS, "Motorola MX2ADS")
	MAINTAINER("Timesys Inc.")
	BOOT_MEM(0xc0000000, 0xc8000000, 0xe0200000)
	BOOT_PARAMS(0xc0000100)
        FIXUP(fixup_mx21)
	MAPIO(mx2ads_map_io)
	INITIRQ(imx21_init_irq)
	.timer		= &imx21_timer,
	INIT_MACHINE(mx2ads_init)
MACHINE_END
