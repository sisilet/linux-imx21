/*
 *	Real Time Clock interface for Linux on Atmel AT91RM9200
 *
 *	Copyright (C) 2002 Rick Bronson
 *
 *	Ported to Linux 2.6 by Steven Scholz
 *	Based on s3c2410-rtc.c Simtec Electronics
 *
 *      Based on sa1100-rtc.c by Nils Faerber
 *	Based on rtc.c by Paul Gortmaker
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>

#include <asm/uaccess.h>
#include <asm/rtc.h>

#include <asm/mach/time.h>

#define DRIVER_NAME "at91_rtc"

#define AT91_RTC_FREQ		1
#define rtc_epoch		1900UL	/* just like arch/arm/common/rtctime.c */

/* Those are the bits from a classic RTC we want to mimic (see linux/mc146818rtc.h) */
#define AT91_RTC_IRQF		0x80	/* any of the following 3 is active */
#define AT91_RTC_PF		0x40
#define AT91_RTC_AF		0x20
#define AT91_RTC_UF		0x10

static DECLARE_WAIT_QUEUE_HEAD(at91_rtc_update);

static spinlock_t at91_rtc_updlock;	/* some spinlocks for saving/restoring interrupt levels */
extern spinlock_t at91_rtc_lock;

static unsigned long rtc_status = 0;
static unsigned long rtc_irq_data;

static unsigned int at91_alarm_year = rtc_epoch;

/*
 * A few helper functions ...
 */

static const unsigned char days_in_mo[] =
    { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

#define is_leap(year) \
	((year) % 4 == 0 && ((year) % 100 != 0 || (year) % 400 == 0))

static const unsigned short int __mon_yday[2][13] =
{
	/* Normal years. */
	{ 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365 },
	/* Leap years. */
	{ 0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335, 366 }
};

/*
 * Returns day since start of the year [0-365]
 *  (from drivers/char/efirtc.c)
 */
static inline int compute_yday(int year, int month, int day)
{
	return __mon_yday[is_leap(year)][month] + day-1;
}

/*
 * Decode time/date into rtc_time structure
 */
static void at91_rtc_decodetime(AT91_REG *timereg, AT91_REG *calreg, struct rtc_time *tm)
{
	unsigned int time, date;

	/* must read twice in case it changes */
	do {
		time = *timereg;
		date = *calreg;
	} while ((time != *timereg) || (date != *calreg));

	tm->tm_sec  = BCD2BIN((time & AT91C_RTC_SEC) >> 0);
	tm->tm_min  = BCD2BIN((time & AT91C_RTC_MIN) >> 8);
	tm->tm_hour = BCD2BIN((time & AT91C_RTC_HOUR) >> 16);

	/* The Calendar Alarm register does not have a field for
	   the year - so these will return an invalid value.  When an
	   alarm is set, at91_alarm_year wille store the current year. */

	tm->tm_year  = BCD2BIN(date & AT91C_RTC_CENT) * 100;		/* century */
	tm->tm_year += BCD2BIN((date & AT91C_RTC_YEAR) >> 8);		/* year */

	tm->tm_wday = BCD2BIN((date & AT91C_RTC_DAY) >> 21) - 1;	/* day of the week [0-6], Sunday=0 */
	tm->tm_mon  = BCD2BIN((date & AT91C_RTC_MONTH) >> 16) - 1;
	tm->tm_mday = BCD2BIN((date & AT91C_RTC_DATE) >> 24);
}

/*
 * Read current time and date in RTC
 */
static int at91_rtc_readtime(struct rtc_time *tm)
{
	at91_rtc_decodetime(&(AT91_SYS->RTC_TIMR), &(AT91_SYS->RTC_CALR), tm);
	tm->tm_yday = compute_yday(tm->tm_year, tm->tm_mon, tm->tm_mday);
	tm->tm_year = tm->tm_year - 1900;

	pr_debug("%s(): %4d-%02d-%02d %02d:%02d:%02d\n", __FUNCTION__,
		1900 + tm->tm_year, tm->tm_mon, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);

	return 0;
}

/*
 * Set current time and date in RTC
 */
static int at91_rtc_settime(struct rtc_time *tm)
{
	unsigned long flags;

	pr_debug("%s(): %4d-%02d-%02d %02d:%02d:%02d\n", __FUNCTION__,
		1900 + tm->tm_year, tm->tm_mon, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);

	/* Stop Time/Calendar from counting */
	AT91_SYS->RTC_CR |= (AT91C_RTC_UPDCAL | AT91C_RTC_UPDTIM);

	spin_lock_irqsave(&at91_rtc_updlock, flags);	/* stop int's else we wakeup b4 we sleep */

	AT91_SYS->RTC_IER = AT91C_RTC_ACKUPD;
	interruptible_sleep_on(&at91_rtc_update);	/* wait for ACKUPD interrupt to hit */
	spin_unlock_irqrestore(&at91_rtc_updlock, flags);
	AT91_SYS->RTC_IDR = AT91C_RTC_ACKUPD;

	AT91_SYS->RTC_TIMR = BIN2BCD(tm->tm_sec) << 0
			| BIN2BCD(tm->tm_min) << 8
			| BIN2BCD(tm->tm_hour) << 16;

	AT91_SYS->RTC_CALR = BIN2BCD((tm->tm_year + 1900) / 100)	/* century */
			| BIN2BCD(tm->tm_year % 100) << 8		/* year */
			| BIN2BCD(tm->tm_mon + 1) << 16			/* tm_mon starts at zero */
			| BIN2BCD(tm->tm_wday + 1) << 21		/* day of the week [0-6], Sunday=0 */
			| BIN2BCD(tm->tm_mday) << 24;

	/* Restart Time/Calendar */
	AT91_SYS->RTC_CR &= ~(AT91C_RTC_UPDCAL | AT91C_RTC_UPDTIM);

	return 0;
}

/*
 * Read alarm time and date in RTC
 */
static int at91_rtc_readalarm(struct rtc_wkalrm *alrm)
{
	struct rtc_time *tm = &alrm->time;

	at91_rtc_decodetime(&(AT91_SYS->RTC_TIMALR), &(AT91_SYS->RTC_CALALR), tm);
	tm->tm_yday = compute_yday(tm->tm_year, tm->tm_mon, tm->tm_mday);
	tm->tm_year = at91_alarm_year - 1900;

	pr_debug("%s(): %4d-%02d-%02d %02d:%02d:%02d\n", __FUNCTION__,
		1900 + tm->tm_year, tm->tm_mon, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
	
	return 0;
}

/*
 * Set alarm time and date in RTC
 */
static int at91_rtc_setalarm(struct rtc_wkalrm *alrm)
{
	struct rtc_time *tm2 = &alrm->time;
	struct rtc_time tm;

	at91_rtc_decodetime(&(AT91_SYS->RTC_TIMR), &(AT91_SYS->RTC_CALR), &tm);

	at91_alarm_year = tm.tm_year;

	if ((unsigned) tm2->tm_hour < 24)	/* do some range checking */
		tm.tm_hour = tm2->tm_hour;
	if ((unsigned) tm2->tm_min < 60)
		tm.tm_min = tm2->tm_min;
	if ((unsigned) tm2->tm_sec < 60)
		tm.tm_sec = tm2->tm_sec;

	AT91_SYS->RTC_TIMALR = BIN2BCD(tm.tm_sec) << 0
		| BIN2BCD(tm.tm_min) << 8
		| BIN2BCD(tm.tm_hour) << 16
		| AT91C_RTC_HOUREN | AT91C_RTC_MINEN
		| AT91C_RTC_SECEN;
	AT91_SYS->RTC_CALALR = BIN2BCD(tm.tm_mon + 1) << 16	/* tm_mon starts at zero */
		| BIN2BCD(tm.tm_mday) << 24
		| AT91C_RTC_DATEEN | AT91C_RTC_MONTHEN;

	pr_debug("%s(): %4d-%02d-%02d %02d:%02d:%02d\n", __FUNCTION__,
		at91_alarm_year, tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

	return 0;
}

/*
 * Handle commands from user-space
 */
static int at91_rtc_ioctl(unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	pr_debug("%s(): cmd=%08x, arg=%08lx.\n", __FUNCTION__, cmd, arg);

	spin_lock_irq(&at91_rtc_lock);
	switch (cmd) {
	case RTC_AIE_OFF:	/* alarm off */
		AT91_SYS->RTC_IDR = AT91C_RTC_ALARM;
		rtc_irq_data = 0;
		break;
	case RTC_AIE_ON:	/* alarm on */
		AT91_SYS->RTC_IER = AT91C_RTC_ALARM;
		rtc_irq_data = 0;
		break;
	case RTC_UIE_OFF:	/* update off */
		AT91_SYS->RTC_IDR = AT91C_RTC_SECEV;
		rtc_irq_data = 0;
		break;
	case RTC_UIE_ON:	/* update on */
		AT91_SYS->RTC_IER = AT91C_RTC_SECEV;
		rtc_irq_data = 0;
		break;
	case RTC_PIE_OFF:	/* periodic off */
		AT91_SYS->RTC_IDR = AT91C_RTC_SECEV;
		rtc_irq_data = 0;
		break;
	case RTC_PIE_ON:	/* periodic on */
		AT91_SYS->RTC_IER = AT91C_RTC_SECEV;
		rtc_irq_data = 0;
		break;
	case RTC_IRQP_READ:	/* read periodic alarm frequency */
		ret = put_user(AT91_RTC_FREQ, (unsigned long *) arg);
		break;
	case RTC_IRQP_SET:	/* set periodic alarm frequency */
		if (arg != AT91_RTC_FREQ) {
			ret = -EINVAL;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}
	spin_unlock_irq(&at91_rtc_lock);

	return ret;
}

/*
 * Provide additional RTC information in /proc/driver/rtc
 */
static int at91_rtc_read_proc(char *buf)
{
	char *p = buf;

	p += sprintf(p, "alarm_IRQ\t: %s\n", (AT91_SYS->RTC_IMR & AT91C_RTC_ALARM) ? "yes" : "no");
	p += sprintf(p, "update_IRQ\t: %s\n", (AT91_SYS->RTC_IMR & AT91C_RTC_ACKUPD) ? "yes" : "no");
	p += sprintf(p, "periodic_IRQ\t: %s\n", (AT91_SYS->RTC_IMR & AT91C_RTC_SECEV) ? "yes" : "no");
	p += sprintf(p, "periodic_freq\t: %ld\n", (unsigned long) AT91_RTC_FREQ);

	return p - buf;
}

/*
 * IRQ handler for the RTC
 */
static irqreturn_t at91_rtc_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	unsigned int rtsr = AT91_SYS->RTC_SR & AT91_SYS->RTC_IMR;

	/* update irq data & counter */
	if (rtsr) {		/* this interrupt is shared!  Is it ours? */
		if (rtsr & AT91C_RTC_ALARM)
			rtc_irq_data |= (AT91_RTC_AF | AT91_RTC_IRQF);
		if (rtsr & AT91C_RTC_SECEV)
			rtc_irq_data |= (AT91_RTC_UF | AT91_RTC_IRQF);
		if (rtsr & AT91C_RTC_ACKUPD)
			wake_up_interruptible(&at91_rtc_update);

		AT91_SYS->RTC_SCCR = rtsr;		/* clear status reg */

	#if 0
		rtc_irq_data += 0x100;			/* counter */
		rtc_update(rtc_irq_data >> 8, rtc_irq_data & 0x000000FF);
	#else
		/* I guess this would be better. Steven */
		rtc_update(1, rtc_irq_data & 0x000000FF);
	#endif

		pr_debug("%s(): num=%ld, events=0x%02lx\n", __FUNCTION__,
			rtc_irq_data >> 8, rtc_irq_data & 0x000000FF);

		return IRQ_HANDLED;
	}
	return IRQ_NONE;		/* not handled */
}

static struct rtc_ops rtc_ops = {
	.owner		= THIS_MODULE,
	.ioctl		= at91_rtc_ioctl,
	.read_time	= at91_rtc_readtime,
	.set_time	= at91_rtc_settime,
	.read_alarm	= at91_rtc_readalarm,
	.set_alarm	= at91_rtc_setalarm,
	.proc		= at91_rtc_read_proc,
};

/*
 * Initialize and install RTC driver
 */
static int at91_rtc_probe(struct device *dev)
{
	int ret;

	pr_debug("%s()\n", __FUNCTION__);

	AT91_SYS->RTC_CR = 0;
	AT91_SYS->RTC_MR = 0;	/* put in 24 hour format */

	/* Disable all interrupts */
	AT91_SYS->RTC_IDR = AT91C_RTC_ACKUPD | AT91C_RTC_ALARM | AT91C_RTC_SECEV | AT91C_RTC_TIMEV | AT91C_RTC_CALEV;

	spin_lock_init(&at91_rtc_updlock);
	spin_lock_init(&at91_rtc_lock);

	ret = request_irq(AT91C_ID_SYS, at91_rtc_interrupt, SA_SHIRQ, "at91_rtc", &rtc_status);
	if (ret) {
		printk(KERN_ERR "at91_rtc: IRQ %d already in use.\n", AT91C_ID_SYS);
		return ret;
	}

	ret = register_rtc(&rtc_ops);
	if (ret) {
		printk(KERN_ERR "at91_rtc: could not register RTC.\n");
		free_irq(AT91C_ID_SYS, &rtc_status);
		return ret;
	}

	printk(KERN_INFO "AT91 Real Time Clock driver.\n");

	return 0;
}

/*
 * Disable and remove the RTC driver
 */
static int at91_rtc_remove(struct device *dev)
{
	pr_debug("%s()\n", __FUNCTION__);

	/* Disable all interrupts */
	AT91_SYS->RTC_IDR = AT91C_RTC_ACKUPD | AT91C_RTC_ALARM | AT91C_RTC_SECEV | AT91C_RTC_TIMEV | AT91C_RTC_CALEV;
	free_irq(AT91C_ID_SYS, &rtc_status);

	rtc_status = 0;

	unregister_rtc(&rtc_ops);

	return 0;
}

#ifdef CONFIG_PM

/* AT91RM9200 RTC Power management control */

static struct timespec at91_rtc_delta;

static int at91_rtc_suspend(struct device *dev, u32 state, u32 level)
{
	struct rtc_time tm;
	struct timespec time;

	/*pr_debug("%s(): level=%08x\n", __FUNCTION__, level);*/

	time.tv_nsec = 0;

	if (level == SUSPEND_SAVE_STATE) {
		/* calculate time delta for suspend */
		at91_rtc_readtime(&tm);
		rtc_tm_to_time(&tm, &time.tv_sec);
		save_time_delta(&at91_rtc_delta, &time);
		pr_debug("%s(): %4d-%02d-%02d %02d:%02d:%02d\n", __FUNCTION__,
			1900 + tm.tm_year, tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	}

	return 0;
}

static int at91_rtc_resume(struct device *dev, u32 level)
{
	struct rtc_time tm;
	struct timespec time;

	time.tv_nsec = 0;

	if (level == RESUME_RESTORE_STATE) {
		at91_rtc_readtime(&tm);
		rtc_tm_to_time(&tm, &time.tv_sec);
		restore_time_delta(&at91_rtc_delta, &time);
		pr_debug("%s(): %4d-%02d-%02d %02d:%02d:%02d\n", __FUNCTION__,
			1900 + tm.tm_year, tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	}
	return 0;
}
#else
#define at91_rtc_suspend NULL
#define at91_rtc_resume  NULL
#endif

static struct platform_device at91_rtc_device = {
	.name		= DRIVER_NAME,
	.id		= -1,
};

static struct device_driver at91_rtc_driver = {
	.name		= DRIVER_NAME,
	.bus		= &platform_bus_type,
	.probe		= at91_rtc_probe,
	.remove		= at91_rtc_remove,
	.suspend	= at91_rtc_suspend,
	.resume		= at91_rtc_resume,
};

static int __init at91_rtc_init(void)
{
	int result;

	result = driver_register(&at91_rtc_driver);
	if (result < 0) {
		printk(KERN_ERR "AT91 RTC: Failed to register driver\n");
		return result;
	}

	result = platform_device_register(&at91_rtc_device);
	if (result < 0) {
		printk(KERN_ERR "AT91 RTC: Failed to register device\n");
		return result;
	}

	return result;
}

static void __exit at91_rtc_exit(void)
{
	platform_device_unregister(&at91_rtc_device);
	driver_unregister(&at91_rtc_driver);

	printk(KERN_INFO "AT91RM9200 RTC removed\n");
}


module_init(at91_rtc_init);
module_exit(at91_rtc_exit);

MODULE_AUTHOR("Rick Bronson");
MODULE_DESCRIPTION("RTC driver for Atmel AT91RM9200");
MODULE_LICENSE("GPL");
