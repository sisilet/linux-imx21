/*
 *
 * Author:     Bill Gatliff
 * Created:    March 28, 2005
 * Copyright:  Bill Gatliff
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/kernel.h>

#include <asm/hardware.h>
#include <asm/leds.h>
#include <asm/system.h>

#include <asm/arch/pxa-regs.h>
#include <asm/arch/csb226.h>

#include "leds.h"


#define LED 5

#define LED_STATE_ENABLED	1
#define LED_STATE_CLAIMED	2

static unsigned int led_state;
static unsigned int hw_led_state;

void csb226_leds_event(led_event_t evt)
{
  unsigned long flags;

  local_irq_save(flags);
  pxa_gpio_mode(LED | GPIO_OUT);

  switch (evt) {
  case led_start:
    led_state = LED_STATE_ENABLED;
    hw_led_state = 0;
    break;

  case led_stop:
    led_state &= ~LED_STATE_ENABLED;
    hw_led_state = 0;
    break;

#if defined(CONFIG_LEDS_TIMER)
  case led_timer:
    hw_led_state ^= LED;
    break;
#endif

#if defined(CONFIG_LEDS_CPU)
  case led_idle_start:
    hw_led_state &= ~LED;
    break;

  case led_idle_end:
    hw_led_state |= LED;
    break;
#endif

  case led_halted:
  case led_green_on:
  case led_green_off:
  case led_amber_on:
  case led_amber_off:
  case led_red_on:
  case led_red_off:
  default:
    break;
  }

  if (hw_led_state) {
    GPSR(LED) = GPIO_bit(LED);
  }
  else {
    GPCR(LED) = GPIO_bit(LED);
  }
	
  local_irq_restore(flags);
}
