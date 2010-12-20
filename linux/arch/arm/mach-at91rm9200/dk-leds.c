/*
 * LED driver for the Atmel AT91RM9200 Development Kit.
 *
 *  Copyright (C) SAN People (Pty) Ltd
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
*/

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>

#include <asm/mach-types.h>
#include <asm/leds.h>
#include <asm/arch/hardware.h>
#include <asm/arch/pio.h>

#ifdef CONFIG_MACH_AT91RM9200EK
#define AT91_LEDS_CPU		AT91C_PIO_PB1
#define AT91_LEDS_TIMER		AT91C_PIO_PB2

#elif defined(CONFIG_MACH_CSB437TL)
#  if (CONFIG_MACH_CSB437TL_REV == 2)
#   define AT91_LEDS_CPU		AT91C_PIO_PB0
#   define AT91_LEDS_TIMER		AT91C_PIO_PB0
#  else
#   define AT91_LEDS_CPU		AT91C_PIO_PB2
#   define AT91_LEDS_TIMER		AT91C_PIO_PB2
#  endif

#else
#define AT91_LEDS_CPU		AT91C_PIO_PB2
#define AT91_LEDS_TIMER		AT91C_PIO_PB2
#endif
#warning "Should not be statically defined"

static inline void at91_led_on(unsigned int led)
{
	AT91_SYS->PIOB_CODR = led;
}

static inline void at91_led_off(unsigned int led)
{
	AT91_SYS->PIOB_SODR = led;
}

static inline void at91_led_toggle(unsigned int led)
{
	unsigned long curr = AT91_SYS->PIOB_ODSR;
	if (curr & led)
		AT91_SYS->PIOB_CODR = led;
	else
		AT91_SYS->PIOB_SODR = led;
}


/*
 * Handle LED events.
 */
static void at91rm9200_leds_event(led_event_t evt)
{
	unsigned long flags;

	local_irq_save(flags);

	switch(evt) {
	case led_start:		/* System startup */
		at91_led_on(AT91_LEDS_CPU);
		break;

	case led_stop:		/* System stop / suspend */
		at91_led_off(AT91_LEDS_CPU);
		break;

#ifdef CONFIG_LEDS_TIMER
	case led_timer:		/* Every 50 timer ticks */
		at91_led_toggle(AT91_LEDS_TIMER);
		break;
#endif

#ifdef CONFIG_LEDS_CPU
	case led_idle_start:	/* Entering idle state */
		at91_led_off(AT91_LEDS_CPU);
		break;

	case led_idle_end:	/* Exit idle state */
		at91_led_on(AT91_LEDS_CPU);
		break;
#endif

	default:
		break;
	}

	local_irq_restore(flags);
}


static int __init leds_init(void)
{
	/* Enable PIO to access the LEDs */
	AT91_SYS->PIOB_PER = AT91_LEDS_TIMER | AT91_LEDS_CPU;
	AT91_SYS->PIOB_OER = AT91_LEDS_TIMER | AT91_LEDS_CPU;

	leds_event = at91rm9200_leds_event;

	leds_event(led_start);
	return 0;
}

__initcall(leds_init);
