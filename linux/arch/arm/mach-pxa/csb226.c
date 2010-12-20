/*
 *  linux/arch/arm/mach-pxa/csb226.c
 *
 * (c) 2005 Bill Gatliff <bgat@billgatliff.com>
 * (c) 2002, 2003 Robert Schwebel <r.schwebel@pengutronix.de>, Pengutronix
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/major.h>
#include <linux/fs.h>
#include <linux/interrupt.h>

#include <asm/setup.h>
#include <asm/memory.h>
#include <asm/mach-types.h>
#include <asm/hardware.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <asm/arch/pxa-regs.h>
#include <asm/arch/irq.h>
#include <asm/arch/irqs.h>
#include <asm/arch/csb226.h>
#include <asm/arch/udc.h>
#include <asm/arch/pxafb.h>
#include <asm/arch/mmc.h>

#include "generic.h"


static void __init csb226_init_irq(void)
{
  pxa_init_irq();
}

#if defined(CONFIG_CIRRUS)

static struct resource cs8900_resources[] = {
  [0] = {
    .start  = CSB226_ETH_PHYS,
    .end    = CSB226_ETH_PHYS + CSB226_ETH_SIZE - 1,
    .flags  = IORESOURCE_MEM,
  },
  [1] = {
    .start  = CSB226_ETH_IRQ,
    .end    = CSB226_ETH_IRQ,
    .flags  = IORESOURCE_IRQ,
  },
};

static struct platform_device cs8900_device = {
  .name		= "cs8900",
  .id		= 0,
  .num_resources	= ARRAY_SIZE(cs8900_resources),
  .resource	= cs8900_resources,
};
#endif

static struct platform_device *devices[] __initdata = {
#if defined(CONFIG_CIRRUS)
  &cs8900_device,
#endif
};

#if defined(CONFIG_MMC_PXA)
#define IRQ_MMC_DETECT IRQ_GPIO(3)

static irqreturn_t (*csb226_mmc_cd_handler)(int, void *, struct pt_regs *);

static void csb226_mmc_cd_debounce_timeout (unsigned long data)
{
  csb226_mmc_cd_handler(IRQ_MMC_DETECT, (void *)data, NULL);
}

static struct timer_list csb226_mmc_cd_debounce_timer =
TIMER_INITIALIZER(csb226_mmc_cd_debounce_timeout, 0, 0);

static irqreturn_t csb226_mmc_cd_irq (int irq, void *data, struct pt_regs *regs)
{
  mod_timer(&csb226_mmc_cd_debounce_timer, jiffies + 5 * HZ / 100);
  return IRQ_HANDLED;
}

static int csb226_mci_init(struct device *dev,
			   irqreturn_t (*cd_handler)(int, void *, struct pt_regs *),
			   void *data)
{
  int ret;

  /* set up gpio */
  pxa_gpio_mode(GPIO6_MMCCLK_MD);
  
  csb226_mmc_cd_handler = cd_handler;
  csb226_mmc_cd_debounce_timer.data = (unsigned long)data;
  
  ret = request_irq(IRQ_MMC_DETECT, csb226_mmc_cd_irq, 0, "pxamcicd", data);
  if (0 == ret)
    set_irq_type(IRQ_MMC_DETECT, IRQT_BOTHEDGE);
  else
    printk(KERN_ERR "%s: request for IRQ_MMC_DETECT failed\n", __FUNCTION__);
  
  return ret;
}

static void csb226_mci_exit (struct device *dev, void *data)
{
  free_irq(IRQ_MMC_DETECT, data);
}

static void csb226_mci_setpower (struct device *dev, unsigned int vdd_bits)
{
  return;
}

static struct pxamci_platform_data csb226_mci_platform_data = {
  .ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
  .init		= csb226_mci_init,
  .setpower     = csb226_mci_setpower,
  .exit         = csb226_mci_exit
};
#endif


static irqreturn_t csb226_gpio0_irq (int irq, void *data, struct pt_regs *regs)
{
  printk(KERN_ERR "%s: detected\n", __FUNCTION__);
  return IRQ_HANDLED;
}

#if defined(CONFIG_FB_PXA)
static void csb226_backlight_power (int on)
{
  if (on) {
    pxa_gpio_mode(GPIO17_PWM1_MD | GPIO_OUT);
    pxa_set_cken(CKEN1_PWM1, 1);
    PWM_CTRL1 = 0;
    PWM_PWDUTY1 = 0x3ff;
    PWM_PERVAL1 = 0x3ff;
  } else {
    PWM_CTRL1 = 0;
    PWM_PWDUTY1 = 0x0;
    PWM_PERVAL1 = 0x3FF;
    pxa_set_cken(CKEN1_PWM1, 0);
  }
  return;
}

static void csb226_lcd_power (int on)
{
#if 0
  #define GPIO40 (1 << 8)
  #define LCD_VEN GPIO40

  unsigned long flags;

  local_irq_save(flags);
  GPDR1 |= LCD_VEN;
  local_irq_restore(flags);

  if (on)
    GPSR1 = LCD_VEN;
  else
    GPCR1 = LCD_VEN;

#else
#warning csb226_lcd_power() not implemented
#endif
  return;
}

static struct pxafb_mach_info sharp_todo __initdata = {
	.pixclock		= 25000,
	.xres			= 640,
	.yres			= 480,
	.bpp			= 16,
	.hsync_len		= 1,
	.left_margin		= 1,
	.right_margin		= 161,
	.vsync_len		= 32,
	.upper_margin		= 0,
	.lower_margin		= 31,
	.lccr0			= LCCR0_Act,
	.lccr3			= LCCR3_OutEnH | LCCR3_PixFlEdg,
	.pxafb_backlight_power	= csb226_backlight_power,
	.pxafb_lcd_power	= csb226_lcd_power,
};
#endif

static void __init csb226_init(void)
{
  (void) platform_add_devices(devices, ARRAY_SIZE(devices));

#if defined(CONFIG_MMC_PXA)
  pxa_set_mci_info(&csb226_mci_platform_data);
#endif

#if defined(CONFIG_CIRRUS)
  set_irq_type(IRQ_GPIO(GPIO_CSB226_ETH), IRQT_RISING);
#endif

  if (!request_irq(IRQ_GPIO(0), csb226_gpio0_irq, 0, "gpio0", 0))
    set_irq_type(IRQ_GPIO(0), IRQT_FALLING);
  else
    printk(KERN_ERR "%s: request for GPIO0 IRQ failed\n", __FUNCTION__);

#if defined(CONFIG_FB_PXA)
  set_pxa_fb_info(&sharp_todo);
#endif
}

/* memory mapping */ 
static struct map_desc csb226_io_desc[] __initdata = {
  /* virtual         physical         length           domain */
#if defined(CONFIG_CIRRUS)
  { CSB226_ETH_VIRT, CSB226_ETH_PHYS, CSB226_ETH_SIZE, MT_DEVICE },
#endif
};


static void __init csb226_map_io(void)
{
  pxa_map_io();
  iotable_init(csb226_io_desc, ARRAY_SIZE(csb226_io_desc));

  /* This enables the BTUART */
  CKEN |= CKEN7_BTUART;
  pxa_gpio_mode(GPIO42_BTRXD_MD);
  pxa_gpio_mode(GPIO43_BTTXD_MD);
  pxa_gpio_mode(GPIO44_BTCTS_MD);
  pxa_gpio_mode(GPIO45_BTRTS_MD);

#if defined(CONFIG_CIRRUS)
  /* This is for the CS8900 chip select */
  pxa_gpio_mode(GPIO78_nCS_2_MD);
#endif

  /* setup sleep mode values */
  PWER  = 0x00000002;
  PFER  = 0x00000000;
  PRER  = 0x00000002;
  PGSR0 = 0x00008000;
  PGSR1 = 0x003F0202;
  PGSR2 = 0x0001C000;
  PCFR |= PCFR_OPDE;
}


MACHINE_START(CSB226, "Cogent CSB226 Development Platform")
     MAINTAINER("Bill Gatliff <bgat@billgatliff.com>")
     BOOT_MEM(0xa0000000, 0x40000000, io_p2v(0x40000000))
     BOOT_PARAMS(0xa0001000)
     MAPIO(csb226_map_io)
     INITIRQ(csb226_init_irq)
     .timer = &pxa_timer,
     INIT_MACHINE(csb226_init)
MACHINE_END
