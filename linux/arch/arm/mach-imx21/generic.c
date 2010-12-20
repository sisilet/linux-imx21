/*
 *  arch/arm/mach-imx21/generic.c
 *
 *  author: Sascha Hauer
 *  Created: april 20th, 2004
 *  Copyright: Synertronixx GmbH
 *
 * Modified By: Ron Melvin (ron.melvin@timesys.com)
 * Copyright (C) 2005 TimeSys Corporation 
 *
 * Modified By: Stephen Donecker (sdonecker@sbcglobal.net)
 *
 * Common code for i.MX machines
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
 *
 */
#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/hardware.h>

#include <asm/mach/map.h>

#include <asm-arm/arch-imx21/imx21fb.h>

/* List current CRM register contents */
#define DEBUG_CRM 1

void imx21_gpio_mode(int gpio_mode);
void imx_set_enet_irq(void);
void imx_clear_enet_irq(void);
void imx21_system_clk_init(void);

void imx21_gpio_mode(int gpio_mode)
{
	unsigned int pin = gpio_mode & GPIO_PIN_MASK;
	unsigned int port = (gpio_mode & GPIO_PORT_MASK) >> GPIO_PORT_POS;
	unsigned int ocr = (gpio_mode & GPIO_OCR_MASK) >> GPIO_OCR_POS;
	unsigned int tmp;

	/* Pullup enable */
	if(gpio_mode & GPIO_PU_MASK)
		PUEN(port) |= (1<<pin);
	else
		PUEN(port) &= ~(1<<pin);

	/* Data direction */
	if(gpio_mode & GPIO_DIR_MASK)
		DDIR(port) |= (1<<pin);
	else
		DDIR(port) &= ~(1<<pin);

	/* Primary function, alternate function, or GPIO */
        switch(gpio_mode & GPIO_FUNC_MASK)
	{
		case GPIO_PF:
			GIUS(port) &= ~(1<<pin);
			GPR(port) &= ~(1<<pin);
			break;
		case GPIO_AF:
			GIUS(port) &= ~(1<<pin);
			GPR(port) |= (1<<pin);
			break;
		case GPIO_GP:
			GIUS(port) |= (1<<pin);
			/* GPR register: don't care when GIUS set to GPIO */
			break;
		default:
			GIUS(port) &= ~(1<<pin);
			GPR(port) &= ~(1<<pin);
	}
	
	if(pin<16) {
		tmp = OCR1(port);
		tmp &= ~( 3<<(pin*2));
		tmp |= (ocr << (pin*2));
		OCR1(port) = tmp;

		if( gpio_mode &	GPIO_AOUT )
			ICONFA1(port) &= ~( 3<<(pin*2));
		if( gpio_mode &	GPIO_BOUT )
			ICONFB1(port) &= ~( 3<<(pin*2));
	} else {
		tmp = OCR2(port);
		tmp &= ~( 3<<((pin-16)*2));
		tmp |= (ocr << ((pin-16)*2));
		OCR2(port) = tmp;

		if( gpio_mode &	GPIO_AOUT )
			ICONFA2(port) &= ~( 3<<((pin-16)*2));
		if( gpio_mode &	GPIO_BOUT )
			ICONFB2(port) &= ~( 3<<((pin-16)*2));
	}
}
EXPORT_SYMBOL(imx21_gpio_mode);

void imx_set_enet_irq()
{
    IMR(GPIO_E) |= NET_IRQ_BIT;
    DDIR(GPIO_E) &= ~NET_IRQ_BIT;
}

void imx_clr_enet_irq()
{
    ISR(GPIO_E) = NET_IRQ_BIT;
}

EXPORT_SYMBOL(imx_clr_enet_irq);

void imx21_system_clk_init()
{
	/*
	 * System clock initialization
	 *
	 * The following register settings are similar to the ones currently
	 * configured in the Freescale distributions of Redboot and Grub. 
	 * All common clock settings should go here and specific settings should
	 * go in their respective drivers. 
	 */
	
	printk("Initializing system clocks\n");
#if 1 
	/* Enable the frequency premultiplier (FPM) to multiply by 512 */
	CRM_CSCR |= CSCR_FPM_EN;
	
	/* Select the FPM output as the input to the MPLL and the SPLL */
	CRM_CSCR &= ~(CSCR_MCU_SEL | CSCR_SP_SEL);
	
	/* Enable the MPLL and the SPLL */
	CRM_CSCR |= (CSCR_MPEN | CSCR_SPEN);

	/*
	 * Set the MPLL so the output frequency is 266MHz by setting 
	 * PD=0, MFD=123, MFI=7, and MFN=115 when fref=32.768kHz
	 */
#warning "Changed by JTM"
//	CRM_MPCTL0 = (MPCTL0_PD(0) | MPCTL0_MFD(123) | MPCTL0_MFI(7) | MPCTL0_MFN(115));

	/* Set the prescaler (PRESC) to divide by 1 */
	CRM_CSCR &= ~CSCR_PRESC_MASK;
	CRM_CSCR |= CSCR_PRESC(0);	

	/* Enable the peripheral clock divider (IPDIV) to divide by 2 */
	CRM_CSCR |= CSCR_IPDIV; 

	/* Set the system bus clock divider (BCLKDIV) to divide by 2 */
//	CRM_CSCR &= ~CSCR_BCLKDIV_MASK;
	CRM_CSCR |= CSCR_BCLKDIV(1);

#warning "Changed by JTM"
	/* Set the peripheral clock divider 1 (PERDIV1) to divide by 6 */
//	CRM_PCDR1 &= ~PCDR1_PERDIV1_MASK;
//	CRM_PCDR1 |= PCDR1_PERDIV1(5); 

	/* Enable HCLK input to the BROM module */
	CRM_PCCR0 |= PCCR0_HCLK_BROM_EN;

	/* Restart the MPLL and wait for the CSCR_MPLL_RESTART bit to clear */
	CRM_CSCR |= CSCR_MPLL_RESTART;
	while (CRM_CSCR & CSCR_MPLL_RESTART)
	    ;
	
	/* 
	 * Set the SPLL so the output frequency is 288MHz by setting
	 * PD=0, MFD=626, MFI=8, and MFN=365 when fref=32.768kHz
	 */
	CRM_SPCTL0 = (SPCTL0_PD(0) | SPCTL0_MFD(626) | SPCTL0_MFI(8) | SPCTL0_MFN(365)); 
	
	/* Restart the SPLL and wait for the CSCR_SPLL_RESTART bit to clear */
	CRM_CSCR |= CSCR_SPLL_RESTART;
	while (CRM_CSCR & CSCR_SPLL_RESTART)
	    ;
	
	/* Set the peripheral clock divider 3 (PERDIV3) to divide by 6 */
	CRM_PCDR1 &= ~PCDR1_PERDIV3_MASK;
	CRM_PCDR1 |= PCDR1_PERDIV3(5);
#endif
#if DEBUG_CRM
	/* Let's take a look at the current CRM register settings */
	printk("CRM_CSCR: 0x%08x\n",CRM_CSCR);
	printk("CRM_MPCTL0: 0x%08x\n",CRM_MPCTL0);
	printk("CRM_MPCTL1: 0x%08x\n",CRM_MPCTL1);
	printk("CRM_SPCTL0: 0x%08x\n",CRM_SPCTL0);
	printk("CRM_SPCTL1: 0x%08x\n",CRM_SPCTL1);
	printk("CRM_OSC26MCTL: 0x%08x\n",CRM_OSC26MCTL);
	printk("CRM_PCDR0: 0x%08x\n",CRM_PCDR0);
	printk("CRM_PCDR1: 0x%08x\n",CRM_PCDR1);
	printk("CRM_PCCR0: 0x%08x\n",CRM_PCCR0);
	printk("CRM_PCCR1: 0x%08x\n",CRM_PCCR1);
	printk("CRM_CCSR: 0x%08x\n",CRM_CCSR);
	printk("CRM_WKGDCTL: 0x%08x\n",CRM_WKGDCTL);
#endif
	
}

/*
 *  get the system pll clock in Hz
 *
 *                  mfi + mfn / (mfd +1)
 *  f = 2 * f_ref * --------------------
 *                        pd + 1
 */

/* TODO: Remove hardcoded values and write routines to retrieve all clk info. */
static unsigned int imx_decode_pll(unsigned int pll)
{
	u32 mfi = (pll >> 10) & 0xf;
	u32 mfn = pll & 0x3ff;
	u32 mfd = (pll >> 16) & 0x3ff;
	u32 pd =  (pll >> 26) & 0xf;
	u32 f_ref = (CRM_CSCR & CSCR_MCU_SEL) ? 26000000 : (CLK32 * 512);

	mfi = mfi <= 5 ? 5 : mfi;

	return (2 * (f_ref>>10) * ( (mfi<<10) + (mfn<<10) / (mfd+1) )) / (pd+1);
}

unsigned int imx_get_system_clk(void)
{
	return imx_decode_pll(CRM_SPCTL0);
}
EXPORT_SYMBOL(imx_get_system_clk);

unsigned int imx_get_mcu_clk(void)
{
	return imx_decode_pll(CRM_MPCTL0);
}
EXPORT_SYMBOL(imx_get_mcu_clk);

/*
 *  get peripheral clock 1 ( UART[12], Timer[12], PWM )
 */
unsigned int imx_get_perclk1(void)
{
	return imx_get_system_clk() / (((CRM_PCDR1 & PCDR1_PERDIV1_MASK) >> PCDR1_PERDIV1_POS)+1);
}
EXPORT_SYMBOL(imx_get_perclk1);

/*
 *  get peripheral clock 2 ( SDHC, CSPI )
 */
unsigned int imx_get_perclk2(void)
{
	return imx_get_system_clk() / (((CRM_PCDR1 & PCDR1_PERDIV2_MASK) >> PCDR1_PERDIV2_POS)+1);
}
EXPORT_SYMBOL(imx_get_perclk2);

/*
 *  get peripheral clock 3 ( LCDC )
 */
unsigned int imx_get_perclk3(void)
{
	return imx_get_system_clk() / (((CRM_PCDR1 & PCDR1_PERDIV3_MASK) >> PCDR1_PERDIV3_POS)+1);
}
EXPORT_SYMBOL(imx_get_perclk3);

/*
 *  get peripheral clock 4 (CSI)
 */
unsigned int imx_get_perclk4(void)
{
	return imx_get_system_clk() / (((CRM_PCDR1 & PCDR1_PERDIV4_MASK) >> PCDR1_PERDIV4_POS)+1);
}

EXPORT_SYMBOL(imx_get_perclk4);

/*
 *  get hclk ( SDRAM, CSI, Memory Stick, I2C, DMA )
 */
#if 0
unsigned int imx_get_hclk(void)
{
	return imx_get_system_clk() / (((CSCR>>10) & 0xf)+1);
}
EXPORT_SYMBOL(imx_get_hclk);

#endif

#if 0
static struct resource imx_mmc_resources[] = {
	[0] = {
		.start	= 0x10013000,
		.end	= 0x100130FF,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= (SDHC_INT),
		.end	= (SDHC_INT),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device imx_mmc_device = {
	.name		= "imx-mmc",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(imx_mmc_resources),
	.resource	= imx_mmc_resources,
};

#endif

static struct resource imx21_uart1_resources[] = {
	[0] = {
		.start	= 0x1000A000,
		.end	= 0x1000A0FF,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= (INT_UART1),
		.end	= (INT_UART1),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device imx21_uart1_device = {
	.name		= "imx21-uart",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(imx21_uart1_resources),
	.resource	= imx21_uart1_resources,
};

static struct resource imx21_uart2_resources[] = {
	[0] = {
		.start	= 0x1000B000,
		.end	= 0x1000B0FF,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= (INT_UART2),
		.end	= (INT_UART2),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device imx21_uart2_device = {
	.name		= "imx21-uart",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(imx21_uart2_resources),
	.resource	= imx21_uart2_resources,
};


static struct resource imx21_uart3_resources[] = {
	[0] = {
		.start	= 0x1000C000,
		.end	= 0x1000C0FF,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= (INT_UART3),
		.end	= (INT_UART3),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device imx21_uart3_device = {
	.name		= "imx21-uart",
	.id		= 2,
	.num_resources	= ARRAY_SIZE(imx21_uart3_resources),
	.resource	= imx21_uart3_resources,
};

static struct resource imx21_uart4_resources[] = {
	[0] = {
		.start	= 0x1000a000,
		.end	= 0x1000a0FF,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= (INT_UART4),
		.end	= (INT_UART4),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device imx21_uart4_device = {
	.name		= "imx21-uart",
	.id		= 3,
	.num_resources	= ARRAY_SIZE(imx21_uart4_resources),
	.resource	= imx21_uart4_resources,
};

static struct imxfb_mach_info imx_fb_info;

void __init set_imx_fb_info(struct imxfb_mach_info *hard_imx_fb_info)
{
	memcpy(&imx_fb_info,hard_imx_fb_info,sizeof(struct imxfb_mach_info));
}
EXPORT_SYMBOL(set_imx_fb_info);

static struct resource imxfb_resources[] = {
	[0] = {
		.start	= 0x10021000,
		.end	= 0x100210ff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= INT_LCDC,
		.end	= INT_LCDC,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 fb_dma_mask = ~(u64)0;

static struct platform_device imxfb_device = {
	.name		= "imx-fb",
	.id		= 0,
	.dev		= {
 		.platform_data	= &imx_fb_info,
		.dma_mask	= &fb_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(imxfb_resources),
	.resource	= imxfb_resources,
};

static struct platform_device *devices[] __initdata = {
    /* 	&imx_mmc_device, */
	&imxfb_device,
	&imx21_uart1_device,
	&imx21_uart2_device,
	// add these later
	//	&imx21_uart3_device,
	//	&imx21_uart4_device,
};

static struct map_desc imx21_io_desc[] __initdata = {
	/* virtual     physical    length      type */
    {IMX21_IO_BASE, IMX21_IO_PHYS, IMX21_IO_SIZE, MT_DEVICE},
    //    {IMX21_EMI_VIRT, IMX21_EMI_PHYS, IMX21_EMI_SIZE, MT_DEVICE},
};

void __init
imx21_map_io(void)
{
	iotable_init(imx21_io_desc, ARRAY_SIZE(imx21_io_desc));
}

static int __init imx21_init(void)
{
    return platform_add_devices(devices, ARRAY_SIZE(devices));
}

subsys_initcall(imx21_init);
