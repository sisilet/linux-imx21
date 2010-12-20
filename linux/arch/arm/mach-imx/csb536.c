/*
 * arch/arm/mach-imx/csb536.c
 *
 * Initially based on:
 *	linux-2.6.7-imx/arch/arm/mach-imx/mx1ads.c
 *	Copyright (c) 2004 Sascha Hauer <sascha@saschahauer.de>
 *
 * 2004 (c) MontaVista Software, Inc.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/config.h>
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

#if defined(CONFIG_MACH_CSB536_PC)
#include <linux/serial_8250.h>
#endif

#if defined(CONFIG_FB_IMX)
#include <asm/arch/imxfb.h>
static void csb536_lcd_power(int on);

static struct imxfb_mach_info csb536_fb_info __initdata = {
	.pixclock	= 62500, 	.bpp		= 16,
	.xres		= 240,		.yres		= 320,

	.hsync_len	= 0x3e,		.vsync_len	= 1,
	.left_margin	= 0x12,		.upper_margin	= 0,
	.right_margin	= 0x39,		.lower_margin	= 3,

	.pcr	= (PCR_COLOR    | 
                   PCR_TFT      | 
                   PCR_FLMPOL   | 
                   PCR_LPPOL    | 
                   PCR_CLKPOL   |
                   PCR_PBSIZ_8  |
                   PCR_BPIX_16  | 
                   PCR_SCLK_SEL |
                   PCR_PCD(3)),
/*	.pwmr	= PWMR_SCR0 | PWMR_CC_EN | PWMR_PW(0x70), */
	.pwmr	= 0, /* default value */
	.lscr1	= 0x400c0373, /* reset default */

	.lcd_power = csb536_lcd_power,
};

static void csb536_lcd_power(int on)
{
    if(on) {
        /* set VEE line on port D */
        DR(3)	|= 0x00000800;
        /* set VEN lines on port C */
        DR(2)	|= 0x00000100;
        /* set BKL line on port A */
        DR(0)   |= 0x00000004;

    } else {
        /* clear BKL line on port A */
        DR(0)   &= ~0x00000004;
        /* clear VEN line on port C */
        DR(2)	&= ~0x00000100;
        /* clear VEE line on port D */
        DR(3)	&= ~0x00000800;

    }
}

#endif /* defined(CONFIG_FB_IMX) */


#if defined(CONFIG_CIRRUS)
static struct resource cs8900_resources[] = {
  [0] = {
    .start  = CSB536_ETH_PHYS,
    .end    = CSB536_ETH_PHYS + CSB536_ETH_SIZE - 1,
    .flags  = IORESOURCE_MEM,
  },
  [1] = {
    .start  = CSB536_ETH_IRQ,
    .end    = CSB536_ETH_IRQ,
    .flags  = IORESOURCE_IRQ,
  },
};

static struct platform_device cs8900_device = {
  .name         = "cs8900",
  .id           = 0,
  .num_resources        = ARRAY_SIZE(cs8900_resources),
  .resource     = cs8900_resources,
};
#endif /* defined(CONFIG_CIRRUS) */


#if defined(CONFIG_MACH_CSB536_PC)

static struct resource serial_device2_resources[] = {
  [0] = {
    .start  = CSB536_UART2_PHYS,
    .end    = CSB536_UART2_PHYS + CSB536_UART2_SIZE - 1,
    .flags  = IORESOURCE_MEM,
  },
  [1] = {
    .start  = CSB536_UART2_IRQ,
    .end    = CSB536_UART2_IRQ,
    .flags  = IORESOURCE_IRQ,
  },
};

static struct plat_serial8250_port serial_platform_data2[] = {
    {
        .mapbase        = 0x14000000,  /* CS3 (Active Low) */
        .irq            = IRQ_GPIOA(12),          /* PA12 */ 
        .uartclk        = 16000000,
        .regshift       = 0,
        .iotype         = UPIO_MEM,
        .flags          = UPF_BOOT_AUTOCONF,
    },
    {},
};

static struct platform_device serial_device2 = {
    .name                   = "imx-uart",
    .id                     = 2,
    .resource               = serial_device2_resources,
    .dev                    = {
      .platform_data  = serial_platform_data2,
    },
};

static struct resource serial_device3_resources[] = {
  [0] = {
    .start  = CSB536_UART3_PHYS,
    .end    = CSB536_UART3_PHYS + CSB536_UART3_SIZE - 1,
    .flags  = IORESOURCE_MEM,
  },
  [1] = {
    .start  = CSB536_UART3_IRQ,
    .end    = CSB536_UART3_IRQ,
    .flags  = IORESOURCE_IRQ,
  },
};

static struct plat_serial8250_port serial_platform_data3[] = {
    {
        .mapbase        = 0x15000000, /* CS4 (Active Low) */
        .irq            = IRQ_GPIOA(13),         /* PA13 */
        .uartclk        = 16000000,
        .regshift       = 0,
        .iotype         = UPIO_MEM,
        .flags          = UPF_BOOT_AUTOCONF,
    },
    {},
};

static struct platform_device serial_device3 = {
    .name                   = "imx-uart",
    .id                     = 3,
    .resource               = serial_device3_resources,
    .dev                    = {
      .platform_data  = serial_platform_data3,
    },
};

#endif /* defined(CONFIG_MACH_CSB536_PC) */

static struct platform_device *devices[] __initdata = {
#if defined(CONFIG_CIRRUS)
        &cs8900_device,
#endif

#if 0
#if defined(CONFIG_MACH_CSB536_PC)
//        &serial_device1,
        &serial_device2,
        &serial_device3,
#endif
#endif


};

static void __init
csb536_init(void)
{
#if 0
#ifdef CONFIG_LEDS
    imx_gpio_mode(GPIO_PORTA | GPIO_OUT | GPIO_GPIO | 2);
#endif
#endif

#if defined(CONFIG_FB_IMX)
    /* Configure I/O for LCD */
    imx_gpio_mode(10 | GPIO_OUT | GPIO_PORTD | GPIO_GPIO | GPIO_PUEN);
    imx_gpio_mode(11 | GPIO_OUT | GPIO_PORTD | GPIO_GPIO | GPIO_PUEN);
    imx_gpio_mode(2  | GPIO_OUT | GPIO_PORTA | GPIO_GPIO | GPIO_PUEN);
    
    set_imx_fb_info(&csb536_fb_info);
    csb536_lcd_power(0);
#endif

#if (defined(CONFIG_SND_CSB536FS) || defined(CONFIG_SND_CSB536PC))
    /* Configure I/O for SSI on port C */
    imx_gpio_mode(3 | GPIO_PORTC | GPIO_PF);  /* SSI_RXFS */
    imx_gpio_mode(4 | GPIO_PORTC | GPIO_PF);  /* SSI_RXCLK */
    imx_gpio_mode(5 | GPIO_PORTC | GPIO_PF);  /* SSI_RXDAT */
    imx_gpio_mode(6 | GPIO_PORTC | GPIO_PF);  /* SSI_TXDAT */
    imx_gpio_mode(7 | GPIO_PORTC | GPIO_PF);  /* SSI_TXFS */
    imx_gpio_mode(8 | GPIO_PORTC | GPIO_PF);  /* SSI_TXCLK */

    FMCR &= ~((1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7));

#endif

#if defined(CONFIG_MACH_CSB536_PC)

    /* Configure IO lines as inputs for XR16C2550 DUART interrupts
     * PA12 is connected to UART "INTA"
     * PA12 is connected to UART "INTB"
     */
    imx_gpio_mode(12  | GPIO_PORTA | GPIO_PF | GPIO_PUEN);
    imx_gpio_mode(13  | GPIO_PORTA | GPIO_PF | GPIO_PUEN);

#endif /* defined(CONFIG_MACH_CSB536_PC) */

#if defined(CONFIG_MMC_IMX)
#warning FIXME: initialize CSB536 I/O for MMC
#endif

	platform_add_devices(devices, ARRAY_SIZE(devices));
}

static struct map_desc csb536_io_desc[] __initdata = {
	/* virtual     physical    length      type */
	{IMX_CS0_VIRT, IMX_CS0_PHYS, IMX_CS0_SIZE, MT_DEVICE},
	{IMX_CS1_VIRT, IMX_CS1_PHYS, IMX_CS1_SIZE, MT_DEVICE},
	{IMX_CS2_VIRT, IMX_CS2_PHYS, IMX_CS2_SIZE, MT_DEVICE},
	{IMX_CS3_VIRT, IMX_CS3_PHYS, IMX_CS3_SIZE, MT_DEVICE},
	{IMX_CS4_VIRT, IMX_CS4_PHYS, IMX_CS4_SIZE, MT_DEVICE},
	{IMX_CS5_VIRT, IMX_CS5_PHYS, IMX_CS5_SIZE, MT_DEVICE},
/*	{IMX_FB_VIRT,  IMX_FB_PHYS,  IMX_FB_SIZE,  MT_DEVICE}, */


};

static void __init
csb536_init_cs(void)
{

#if defined(CONFIG_MACH_CSB536_PC)
    /* Set clock for UART to 16MHz */
    CSCR = (CSCR & ~0xe0000000) | 0x60000000;

    /* Set up *cs lines */
    FMCR &= ~(1<<1);  /* *CS3 */
    imx_gpio_mode(22  | GPIO_PORTA | GPIO_PF | GPIO_PUEN); /* *CS4*/

    /* Configure chip selects for XR16C2550
     * CS3 is connected to UART "A"
     * CS4 is connected to UART "B"
     */

    EIM_CS3U = 
        EIM_CSnU_CNC(3) |  /* Minimum Chip Select is 3 cycles */
        EIM_CSnU_WSC(14) |  /* # if wait cycles ( > 70 ns at 48MHz HCLK) */
        EIM_CSnU_EDC(4);   /* Add one extra dead cycle */


    EIM_CS3L = 
        EIM_CSnL_OEA(1) | /* OE (IOR) asserts 1 half cycle after CS 
                           * Read Address Setup time (5 ns)
                           */

        EIM_CSnL_OEN(1) | /* OE (IOR) deasserts 1 half cycle before CS 
                           * Read Address Hold time (10 ns)
                           */

        EIM_CSnL_WEA(1) | /* WE (IOW) asserts 1 half cycle after CS 
                           * Write Address Setup time (5 ns)
                           */

        EIM_CSnL_WEN(1) | /* WE (IOW) deasserts 1 half cycle before CS 
                           * Write Address Hold time (10 ns)
                           */

        EIM_CSnL_DSZ(5) |  /* 8-bit port data on D[7:0] */
//        EIM_CSnL_DSZ(7) |  /* 32-bit port data */
        EIM_CSnU_CSEN;     /* Enable the chip select */


    EIM_CS4U = 
        EIM_CSnU_CNC(3) |  /* Minimum Chip Select is 3 cycles */
        EIM_CSnU_WSC(14) |  /* cycle is seven clocks ( > 70 ns at 48MHz HCLK) */
        EIM_CSnU_EDC(4);   /* Add one extra dead cycle */

    EIM_CS4L = 
        EIM_CSnL_OEA(1) | /* OE (IOR) asserts 1 half cycle after CS 
                           * Read Address Setup time (5 ns)
                           */

        EIM_CSnL_OEN(1) | /* OE (IOR) deasserts 1 half cycle before CS 
                           * Read Address Hold time (10 ns)
                           */

        EIM_CSnL_WEA(1) | /* WE (IOW) asserts 1 half cycle after CS 
                           * Write Address Setup time (5 ns)
                           */

        EIM_CSnL_WEN(1) | /* WE (IOW) deasserts 1 half cycle before CS 
                           * Write Address Hold time (10 ns)
                           */

        EIM_CSnL_DSZ(5)|   /* 8-bit port data on D[7:0] */
//        EIM_CSnL_DSZ(7)|   /* 32-bit port data */
        EIM_CSnU_CSEN;     /* Enable the chip select */

#endif /* defined(CONFIG_MACH_CSB536_PC) */
 
}

static void __init
csb536_map_io(void)
{
#if defined(CONFIG_MACH_CSB536_PC)
    int i;
    unsigned long *src = (unsigned long *)0xe0007080;
    unsigned long *dst = (unsigned long *)0xe0006080;
#endif

	imx_map_io();
	iotable_init(csb536_io_desc, ARRAY_SIZE(csb536_io_desc));
    csb536_init_cs();

#if defined(CONFIG_MACH_CSB536_PC)
    for (i = 0; i < 12; i++) {
        dst[i] = src[i];
    }
#endif

    /* Setup PERCLK3 to be /8 -> 96MHz/8 = 12KHz */
    PCDR = (PCDR & 0xffff) | (0x7 << 16);
}

MACHINE_START(CSB536, "Cogent CSB536")
	MAINTAINER("Jay Monkman <jtm@lopingdog.com>")
	BOOT_MEM(0x08000000, 0x00200000, 0xe0200000)
	BOOT_PARAMS(0x08000100)
	MAPIO(csb536_map_io)
	INITIRQ(imx_init_irq)
	.timer		= &imx_timer,
	INIT_MACHINE(csb536_init)
MACHINE_END
