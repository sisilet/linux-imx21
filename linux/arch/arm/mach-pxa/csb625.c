/*
 *  linux/arch/arm/mach-pxa/csb625.c
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
#include <linux/fb.h>

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
#include <asm/arch/audio.h>
#include <asm/arch/csb625.h>
#include <asm/arch/udc.h>
#include <asm/arch/pxafb.h>
#include <asm/arch/mmc.h>

#if defined(CONFIG_USB_ISP116X_HCD)
#include <linux/usb_isp116x.h>
#endif

#include "generic.h"


static void __init csb625_init_irq(void)
{
  pxa_init_irq();
}

#if defined(CONFIG_CIRRUS)

static struct resource cs8900_resources[] = {
  [0] = {
    .start  = CSB625_ETH_PHYS,
    .end    = CSB625_ETH_PHYS + CSB625_ETH_SIZE - 1,
    .flags  = IORESOURCE_MEM,
  },
  [1] = {
    .start  = CSB625_ETH_IRQ,
    .end    = CSB625_ETH_IRQ,
    .flags  = IORESOURCE_IRQ,
  },
};

static struct platform_device cs8900_device = {
  .name		= "cs8900",
  .id		= 0,
  .num_resources = ARRAY_SIZE(cs8900_resources),
  .resource	= cs8900_resources,
};
#endif

#if defined(CONFIG_USB_ISP116X_HCD)
static struct resource isp116x_resources[] = {
  [0] = {
    .start  = CSB625_USB_PHYS,
    .end    = CSB625_USB_PHYS + 1,
    .flags  = IORESOURCE_MEM,
  },
  [1] = {
    .start  = CSB625_USB_PHYS + 2,
    .end    = CSB625_USB_PHYS + 3,
    .flags  = IORESOURCE_MEM,
  },
  [2] = {
    .start  = IRQ_GPIO(CSB625_USB_IRQ),
    .end    = IRQ_GPIO(CSB625_USB_IRQ),
    .flags  = IORESOURCE_IRQ,
  },
};

static struct isp116x_platform_data csb625_isp116x_platform_data = {
  .sel15Kres = 1,
  .int_act_high = 0,
  .int_edge_triggered = 0,
  .no_power_switching = 1,
};

static struct platform_device isp116x_device = {
  .name		= "isp116x-hcd",
  .id		= 0,
  .num_resources = ARRAY_SIZE(isp116x_resources),
  .resource	= isp116x_resources,
  .dev          = {
    .platform_data = &csb625_isp116x_platform_data,
  },
};

#endif


#if defined(CONFIG_MMC_PXA)
#define IRQ_MMC_DETECT IRQ_GPIO(3)

static irqreturn_t (*csb625_mmc_cd_handler)(int, void *, struct pt_regs *);

static void csb625_mmc_cd_debounce_timeout (unsigned long data)
{
  csb625_mmc_cd_handler(IRQ_MMC_DETECT, (void *)data, NULL);
}

static struct timer_list csb625_mmc_cd_debounce_timer =
TIMER_INITIALIZER(csb625_mmc_cd_debounce_timeout, 0, 0);

static irqreturn_t csb625_mmc_cd_irq (int irq, void *data, struct pt_regs *regs)
{
  mod_timer(&csb625_mmc_cd_debounce_timer, jiffies + 5 * HZ / 100);
  return IRQ_HANDLED;
}

static int csb625_mci_init(struct device *dev,
			   irqreturn_t (*cd_handler)(int, void *, struct pt_regs *),
			   void *data)
{
  int ret;

  /* set up gpio */
  pxa_gpio_mode(GPIO6_MMCCLK_MD);
  pxa_gpio_mode(GPIO8_MMCCS0);  
  
  csb625_mmc_cd_handler = cd_handler;
  csb625_mmc_cd_debounce_timer.data = (unsigned long)data;
  
  ret = request_irq(IRQ_MMC_DETECT, csb625_mmc_cd_irq, 0, "pxamcicd", data);
  if (0 == ret)
    set_irq_type(IRQ_MMC_DETECT, IRQT_BOTHEDGE);
  else
    printk(KERN_ERR "%s: request for IRQ_MMC_DETECT failed\n", __FUNCTION__);
  
  return ret;
}

static void csb625_mci_exit (struct device *dev, void *data)
{
  free_irq(IRQ_MMC_DETECT, data);
}

static void csb625_mci_setpower (struct device *dev, unsigned int vdd_bits)
{
  pxa_gpio_mode(GPIO41_FFRTS | GPIO_OUT);
  switch (vdd_bits)
    {
    case 0:
      GPCR(GPIO41_FFRTS) = 1;
      break;
    default:
      GPSR(GPIO41_FFRTS) = 1;
    }
  return;
}

static struct pxamci_platform_data csb625_mci_platform_data = {
  .ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
  .init		= csb625_mci_init,
  .setpower     = csb625_mci_setpower,
  .exit         = csb625_mci_exit
};
#endif

#if defined(CONFIG_FB_PXA)
static void csb625_backlight_power (int on)
{
  if (on) {
    pxa_gpio_mode(GPIO17_PWM1_MD);
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

static void csb625_lcd_power (int on)
{
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
	.pxafb_backlight_power	= csb625_backlight_power,
	.pxafb_lcd_power	= csb625_lcd_power,
};
#endif

#if defined(CONFIG_SND_PXA2XX_AC97)
static int audio_startup(snd_pcm_substream_t *substream, void *priv)
{
  printk(KERN_ERR "%s\n", __FUNCTION__);
  return 0;
}
static void audio_shutdown(snd_pcm_substream_t *substream, void *priv)
{printk(KERN_ERR "%s\n", __FUNCTION__);}
static void audio_suspend(void *priv){printk(KERN_ERR "%s\n", __FUNCTION__);}
static void audio_resume(void *priv){printk(KERN_ERR "%s\n", __FUNCTION__);}

static pxa2xx_audio_ops_t audio_ops = {
	.startup  = audio_startup,
	.shutdown = audio_shutdown,
	.suspend  = audio_suspend,
	.resume	= audio_resume,
};

static struct platform_device audio_device = {
	.name	= "pxa2xx-ac97",
	.id       = -1,
	.dev      = { .platform_data = &audio_ops },
};
#endif

static struct platform_device *devices[] __initdata = {
#if defined(CONFIG_CIRRUS)
  &cs8900_device,
#endif
#if defined(CONFIG_USB_ISP116X_HCD)
  &isp116x_device,
#endif
#if defined(CONFIG_SND_PXA2XX_AC97)
  &audio_device,
#endif

};

static irqreturn_t gpio_irq (int irq, void *data, struct pt_regs *regs)
{
  printk(KERN_ERR "%s: detected\n", __FUNCTION__);
  return IRQ_NONE;
}

static void __init csb625_init(void)
{
#if defined(CONFIG_USB_ISP116X_HCD)
  pxa_gpio_mode(GPIO33_nCS_5_MD);
  pxa_gpio_mode(CSB625_USB_IRQ | GPIO_IN );
  set_irq_type(IRQ_GPIO(CSB625_USB_IRQ), IRQT_FALLING);
#endif

  (void) platform_add_devices(devices, ARRAY_SIZE(devices));

#if defined(CONFIG_MMC_PXA)
  pxa_set_mci_info(&csb625_mci_platform_data);
#endif

#if defined(CONFIG_CIRRUS)
  set_irq_type(IRQ_GPIO(GPIO_CSB625_ETH), IRQT_RISING);
#endif

#if 0
#warning TODO: get rid of this example
  if (!request_irq(IRQ_GPIO(GPIO1_RST), gpio_irq, 0, "gpio_irq", 0))
    {
    }
  else
    printk(KERN_ERR "%s: request for GPIO IRQ failed\n", __FUNCTION__);
#endif
 
#if defined(CONFIG_FB_PXA)
  set_pxa_fb_info(&sharp_todo);
#endif
}

/* memory mapping */ 
static struct map_desc csb625_io_desc[] __initdata = {
  /* virtual         physical         length           domain */
#if defined(CONFIG_CIRRUS)
  { CSB625_ETH_VIRT, CSB625_ETH_PHYS, CSB625_ETH_SIZE, MT_DEVICE },
#endif
};


static void __init csb625_map_io(void)
{
  pxa_map_io();
  iotable_init(csb625_io_desc, ARRAY_SIZE(csb625_io_desc));
}


MACHINE_START(CSB625, "Cogent CSB625 Development Platform")
     MAINTAINER("Bill Gatliff <bgat@billgatliff.com>")
     BOOT_MEM(0xa0000000, 0x40000000, io_p2v(0x40000000))
     BOOT_PARAMS(0xa0000100)
     MAPIO(csb625_map_io)
     INITIRQ(csb625_init_irq)
     .timer = &pxa_timer,
     INIT_MACHINE(csb625_init)
MACHINE_END
