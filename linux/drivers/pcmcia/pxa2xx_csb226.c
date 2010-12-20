/*
 * linux/drivers/pcmcia/pxa2xx_csb226.c
 *
 * CSB226 PCMCIA specific routines.
 *
 * Unlike Mainstone, Corgi, etc., the CF slot and all associated i/o
 * on the CSB226 is tied directly to the pxa2xx chip.  Thus, we interact
 * directly with pxa gpio registers and whatnot, and can dump the additional
 * work of dealing with an external CPLD the way those other guys do.  :^)
 *
 * Created:	February 7, 2005
 * Author:	Bill Gatliff <bgat@billgatliff.com>
 * Copyright:	Bill Gatliff <bgat@billgatliff.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/device.h>

#include <pcmcia/ss.h>

#include <asm/hardware.h>
#include <asm/irq.h>

#include <asm/arch/pxa-regs.h>
#include <asm/arch/csb226.h>

#include "soc_common.h"


static struct pcmcia_irqs irqs[] = {
  { 0, IRQ_GPIO(CSB226_CF0_CD), "CF0 CD" },
  { 0, IRQ_GPIO(CSB226_CF0_IRQ), "CF0 IRQ" },
};

static int
csb226_pcmcia_hw_init (struct soc_pcmcia_socket *skt)
{
  pxa_gpio_mode(CSB226_CF0_CD | GPIO_IN);
  pxa_gpio_mode(CSB226_CF0_IRQ | GPIO_IN);
  pxa_gpio_mode(CSB226_CF0_POWER | GPIO_OUT);
  pxa_gpio_mode(CSB226_CF0_RESET | GPIO_OUT);

  pxa_gpio_mode(GPIO48_nPOE_MD);
  pxa_gpio_mode(GPIO49_nPWE_MD);
  pxa_gpio_mode(GPIO50_nPIOR_MD);
  pxa_gpio_mode(GPIO51_nPIOW_MD);
  pxa_gpio_mode(GPIO52_nPCE_1_MD);
  pxa_gpio_mode(GPIO53_nPCE_2_MD);
  pxa_gpio_mode(GPIO54_pSKTSEL_MD);
  pxa_gpio_mode(GPIO55_nPREG_MD);
  pxa_gpio_mode(GPIO56_nPWAIT_MD);
  pxa_gpio_mode(GPIO57_nIOIS16_MD);

  skt->irq = IRQ_GPIO(CSB226_CF0_IRQ);
 
  return 0;
}

static void
csb226_pcmcia_hw_shutdown (struct soc_pcmcia_socket *skt){}

static void
csb226_pcmcia_socket_state (struct soc_pcmcia_socket *skt,
			    struct pcmcia_state *state)
{
  state->detect = GPLR(CSB226_CF0_CD) & GPIO_bit(CSB226_CF0_CD) ? 0 : 1;
  state->ready  = GPLR(CSB226_CF0_IRQ) & GPIO_bit(CSB226_CF0_IRQ) ? 1 : 0;
  state->bvd1   = 0;
  state->bvd2   = 0;
  state->vs_3v  = 1;
  state->vs_Xv  = 0;
  state->wrprot = 0;  /* not available */

  return;
}

static int
csb226_pcmcia_configure_socket (struct soc_pcmcia_socket *skt,
				const socket_state_t *state)
{
  int ret = 0;

  switch (state->Vcc) {
  case 0:
    GPSR(CSB226_CF0_POWER) = GPIO_bit(CSB226_CF0_POWER);
    break;
  case 33:
  case 50:
    GPCR(CSB226_CF0_POWER) = GPIO_bit(CSB226_CF0_POWER);
    break;
  }

  if (state->flags & SS_RESET)
    GPSR(CSB226_CF0_RESET) = GPIO_bit(CSB226_CF0_RESET);
  else
    GPCR(CSB226_CF0_RESET) = GPIO_bit(CSB226_CF0_RESET);

  return ret;
}

static void
csb226_pcmcia_socket_init (struct soc_pcmcia_socket *skt) {}

static void
csb226_pcmcia_socket_suspend (struct soc_pcmcia_socket *skt) {}

static struct pcmcia_low_level csb226_pcmcia_ops = {
  .owner		= THIS_MODULE,
  .hw_init		= csb226_pcmcia_hw_init,
  .hw_shutdown		= csb226_pcmcia_hw_shutdown,
  .socket_state		= csb226_pcmcia_socket_state,
  .configure_socket	= csb226_pcmcia_configure_socket,
  .socket_init		= csb226_pcmcia_socket_init,
  .socket_suspend	= csb226_pcmcia_socket_suspend,
  .nr			= 1,
};

static struct platform_device *csb226_pcmcia_device;

static int __init
csb226_pcmcia_init (void)
{
  int ret;

  csb226_pcmcia_device = kmalloc(sizeof(*csb226_pcmcia_device), GFP_KERNEL);
  if (!csb226_pcmcia_device)
    return -ENOMEM;
  memset(csb226_pcmcia_device, 0, sizeof(*csb226_pcmcia_device));
  csb226_pcmcia_device->name = "pxa2xx-pcmcia";
  csb226_pcmcia_device->dev.platform_data = &csb226_pcmcia_ops;

  ret = platform_device_register(csb226_pcmcia_device);
  if (ret)
    kfree(csb226_pcmcia_device);

  return ret;
}

static void __exit
csb226_pcmcia_exit (void)
{
  platform_device_unregister(csb226_pcmcia_device);
}

module_init(csb226_pcmcia_init);
module_exit(csb226_pcmcia_exit);

MODULE_LICENSE("GPL");
