/*
 * linux/drivers/pcmcia/pxa2xx_csb625.c
 *
 * CSB625 PCMCIA specific routines.
 *
 * Unlike Mainstone, Corgi, etc., the CF slot and all associated i/o
 * on the CSB625 is tied directly to the pxa2xx chip.  Thus, we interact
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
#include <asm/arch/csb625.h>

#include "soc_common.h"


static int
csb625_pcmcia_hw_init (struct soc_pcmcia_socket *skt)
{
  pxa_gpio_mode(CSB625_CF0_CD | GPIO_IN);
  pxa_gpio_mode(CSB625_CF0_IRQ | GPIO_IN);
  pxa_gpio_mode(CSB625_CF0_POWER | GPIO_OUT);
  pxa_gpio_mode(CSB625_CF0_RESET | GPIO_OUT);

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
 
  skt->irq = IRQ_GPIO(CSB625_CF0_IRQ);
 
  /* TODO: these settings get lost in soc_common; they also don't get
     set before the first call to set_timing, which is equally bad.
     To compensate for both, I have hacked pcmcia/pxa2xx_base.c to
     prevent it from changing the values MCMEM0 et al. bgat Aug05 */
  skt->spd_io[0] = 1000;
  skt->spd_mem[0] = 1000;
  skt->spd_attr[0] = 1000;

  return 0;
}

static void
csb625_pcmcia_hw_shutdown (struct soc_pcmcia_socket *skt) {}

static void
csb625_pcmcia_socket_state (struct soc_pcmcia_socket *skt,
			    struct pcmcia_state *state)
{
  state->detect = GPLR(CSB625_CF0_CD) & GPIO_bit(CSB625_CF0_CD) ? 0 : 1;
  state->ready  = GPLR(CSB625_CF0_IRQ) & GPIO_bit(CSB625_CF0_IRQ) ? 1 : 0;
  state->bvd1   = 0;
  state->bvd2   = 0;
  state->vs_3v  = 1;
  state->vs_Xv  = 0;
  state->wrprot = 0;  /* not available */

  return;
}

static int
csb625_pcmcia_configure_socket (struct soc_pcmcia_socket *skt,
				const socket_state_t *state)
{
  int ret = 0;

  switch (state->Vcc) {
  case 0:
    /* TODO: VCC control doesn't quite work on the '625; leave it
       turned on all the time */
#if 0
    GPCR(CSB625_CF0_POWER) = GPIO_bit(CSB625_CF0_POWER);
#endif
    break;
  default:
  case 33:
  case 50:
    GPSR(CSB625_CF0_POWER) = GPIO_bit(CSB625_CF0_POWER);
    break;
  }

  if (state->flags & SS_RESET)
    GPSR(CSB625_CF0_RESET) = GPIO_bit(CSB625_CF0_RESET);
  else
    GPCR(CSB625_CF0_RESET) = GPIO_bit(CSB625_CF0_RESET);

  return ret;
}

static void
csb625_pcmcia_socket_init (struct soc_pcmcia_socket *skt) {}

static void
csb625_pcmcia_socket_suspend (struct soc_pcmcia_socket *skt) {}

static struct pcmcia_low_level csb625_pcmcia_ops = {
  .owner		= THIS_MODULE,
  .hw_init		= csb625_pcmcia_hw_init,
  .hw_shutdown		= csb625_pcmcia_hw_shutdown,
  .socket_state		= csb625_pcmcia_socket_state,
  .configure_socket	= csb625_pcmcia_configure_socket,
  .socket_init		= csb625_pcmcia_socket_init,
  .socket_suspend	= csb625_pcmcia_socket_suspend,
  .nr			= 1,
};

static struct platform_device *csb625_pcmcia_device;

static int __init
csb625_pcmcia_init (void)
{
  int ret;

  csb625_pcmcia_device = kmalloc(sizeof(*csb625_pcmcia_device), GFP_KERNEL);
  if (!csb625_pcmcia_device)
    return -ENOMEM;
  memset(csb625_pcmcia_device, 0, sizeof(*csb625_pcmcia_device));
  csb625_pcmcia_device->name = "pxa2xx-pcmcia";
  csb625_pcmcia_device->dev.platform_data = &csb625_pcmcia_ops;

  ret = platform_device_register(csb625_pcmcia_device);
  if (ret)
    kfree(csb625_pcmcia_device);

  return ret;
}

static void __exit
csb625_pcmcia_exit (void)
{
  platform_device_unregister(csb625_pcmcia_device);
}

module_init(csb625_pcmcia_init);
module_exit(csb625_pcmcia_exit);

MODULE_LICENSE("GPL");
