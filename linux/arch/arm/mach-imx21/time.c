/*
 *  linux/arch/arm/mach-imx21/time.c
 *
 *  Copyright (C) 2000-2001 Deep Blue Solutions
 *  Copyright (C) 2002 Shane Nay (shane@minirl.com)
 *
 *  Modified By: Ron Melvin (ron.melvin@timesys.com)
 *  Copyright (C) 2005 TimeSys Corporation *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/time.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/leds.h>
#include <asm/irq.h>
#include <asm/mach/time.h>

/*
 * Returns number of us since last clock interrupt.  Note that interrupts
 * will have been disabled by do_gettimeoffset()
 */
static unsigned long imx21_gettimeoffset(void)
{
	unsigned long ticks;

	/*
	 * Get the current number of ticks.  Note that there is a race
	 * condition between us reading the timer and checking for
	 * an interrupt.  We get around this by ensuring that the
	 * counter has not reloaded between our two reads.
	 */
	ticks = IMX21_TCN(GPT1);

	/*
	 * Interrupt pending?  If so, we've reloaded once already.
	 */
	if (IMX21_TSTAT(GPT1) & TSTAT_COMP)
		ticks += LATCH;

	/*
	 * Convert the ticks to usecs
	 */
	return (1000000 / CLK32) * ticks;
}

/*
 * IRQ handler for the timer
 */
static irqreturn_t
imx21_timer_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	write_seqlock(&xtime_lock);

	/* if any bits set, rupt has occurred, clear it by  writing a 1 */
	if ( IMX21_TSTAT(GPT1) ) {
	    IMX21_TSTAT(GPT1) = TSTAT_CAPT | TSTAT_COMP;
	}
	
	timer_tick(regs);
	write_sequnlock(&xtime_lock);

	return IRQ_HANDLED;
}

static struct irqaction imx21_timer_irq = {
	.name		= "i.MX21 Timer Tick",
	.flags		= SA_INTERRUPT,
	.handler	= imx21_timer_interrupt
};

/*
 * Set up timer interrupt, and return the current time in seconds.
 */
static void __init imx21_timer_init(void)
{
	/*
	 * Initialise to a known state (all timers off, and timing reset)
	 */
	IMX21_TCTL(GPT1) = 0;
	IMX21_TPRER(GPT1) = 0;
	IMX21_TCMP(GPT1) = LATCH - 1;
	IMX21_TCTL(GPT1) = TCTL_CLK_32 | TCTL_IRQEN | TCTL_TEN;

	/*
	 * Make irqs happen for the system timer
	 */
	setup_irq(INT_GPT1, &imx21_timer_irq);
}

struct sys_timer imx21_timer = {
	.init		= imx21_timer_init,
	.offset		= imx21_gettimeoffset,
};
