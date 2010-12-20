/*
 * $Id:$
 *
 *  Copyright (c) 2005 Jay Monkman <jtm@lopingdog.com>
 *
 * Touchscreen driver for CSB536
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#include <linux/config.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>

#include <asm/irq.h>
#include <asm/io.h>

#define DRIVER_DESC	"CSB536 ADS7846 touchscreen driver"

MODULE_AUTHOR("Jay Monkman <jtm@lopingdog.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

/*
 * Definitions & globals
 */

//static char *dev_name = "CSB536 ADS7846 touchscreen driver";
#define ADS7846_CTRL_START     (1 << 7)
#define ADS7846_GET_Y          (1 << 4)
#define ADS7846_GET_Z1         (3 << 4)
#define ADS7846_GET_Z2         (4 << 4)
#define ADS7846_GET_X          (5 << 4)
#define ADS7846_CTRL_8BIT      (1 << 3)
#define ADS7846_CTRL_12BIT     (0 << 3)
#define ADS7846_CTRL_DIFMODE   (0 << 2)

#define STATE_NONE  0
#define STATE_X     1
#define STATE_Y     2
#define STATE_Z1    3
#define STATE_Z2    4

/* These were determined empirically. They may need to change */
#define X_MIN 130
#define X_MAX 1900
#define Y_MIN 90
#define Y_MAX 1950
#define Z1_MIN 120
#define Z1_MAX 1300
#define Z2_MIN 700
#define Z2_MAX 1900

#define PRES_MIN 0
#define PRES_MAX (X_MAX * ((Z2_MAX / Z1_MIN) - 1))

typedef struct {
    struct timer_list timer;
    struct input_dev input;
    int state;
    int pendown;
    int x;
    int y;
    int z1;
    int z2;
} ads7846_stuff_t;

static ads7846_stuff_t ads;

/*
 * Per-touchscreen data.
 */
static void ads7846_acquire(void);
struct input_dev ads7846_dev;

static irqreturn_t ads7846_spi_isr(int irq, void *dummy, struct pt_regs *fp)
{
    unsigned char junk;
    unsigned char t0;
    unsigned char t1;
    int n;


    /* Disable the tx interrupt */
    SSP1_INT_REG &= ~SSP_INT_TEEN;

    while (SSP1_CTRL_REG & SSP_XCH) continue;

    n = (SSP1_TEST_REG >> 4) & 0xf;
    if (n != 3) {
        printk("Error reading SPI - 3 words expected, got %d\n", n);
    }

    junk = (SSP1_RX_REG & 0xff);
    t0 = (SSP1_RX_REG & 0xff);
    t1 = (SSP1_RX_REG & 0xff);

    switch(ads.state) {
    case STATE_X:
        ads.x = ((t0 << 8) | t1) >> 4;
        ads.state = STATE_Y;
        ads7846_acquire();
        break;

    case STATE_Y:
        ads.y = ((t0 << 8) | t1) >> 4;
        ads.state = STATE_Z1;
        ads7846_acquire();
        break;

    case STATE_Z1:
        ads.z1 = ((t0 << 8) | t1) >> 4;
        ads.state = STATE_Z2;
        ads7846_acquire();
        break;

    case STATE_Z2:
        ads.z2 = ((t0 << 8) | t1) >> 4;
        ads.state = STATE_NONE;
        input_report_abs(&ads.input, ABS_X, ads.x);
        input_report_abs(&ads.input, ABS_Y, ads.y);
        if (ads.pendown) {
            int p;
            input_report_abs(&ads.input, BTN_TOUCH, 1);
            p = ads.x * ((ads.z2 / (ads.z1 + 1)) - 1);
            input_report_abs(&ads.input, ABS_PRESSURE, p);
            if (p == 0) {
                printk("z1:%5d z2:%5d\n", ads.z1, ads.z2);
            }
        } else {
            input_report_abs(&ads.input, BTN_TOUCH, 1);
            input_report_abs(&ads.input, ABS_PRESSURE, 0);
        }
        input_sync(&ads.input);
//        printk("x:%5d y:%5d z1:%5d z2:%5d\n", ads.x, ads.y, ads.z1, ads.z2);
        break;

    default:
        printk("Unknown state\n");
        ads.state = STATE_NONE;
        return IRQ_RETVAL(IRQ_HANDLED);;
        break;
    }

    return IRQ_RETVAL(IRQ_HANDLED);
}


static void ads7846_timer(unsigned long data)
{
    if (SSR(1) & (1 << 18)) {  /* Pen is now up */
        if (ads.pendown == 1) {
            ads.pendown = 0;
            ads.state = STATE_X;
            ads7846_acquire();
        }
    } else {
        if (ads.pendown == 0) {
            ads.pendown = 1;
        }
        ads.state = STATE_X;
        ads7846_acquire();
    }

    mod_timer(&ads.timer, jiffies + (HZ/100));
}

static void ads7846_acquire(void)
{
    unsigned char ctrl;
    unsigned char junk;

    while (SSP1_TEST_REG & 0xf0) {
        printk("%s:%d :: %d words in RX fifo\n", 
               __FUNCTION__, __LINE__, (SSP1_TEST_REG >> 4) & 0xf);
	junk = SSP1_RX_REG;
    }

    switch(ads.state) {
    case STATE_X:
        ctrl = (ADS7846_CTRL_START     |
                ADS7846_GET_X          |
                ADS7846_CTRL_12BIT     |
                ADS7846_CTRL_DIFMODE);
        break;

    case STATE_Y:
        ctrl = (ADS7846_CTRL_START     |
                ADS7846_GET_Y          |
                ADS7846_CTRL_12BIT     |
                ADS7846_CTRL_DIFMODE);
        break;

    case STATE_Z1:
        ctrl = (ADS7846_CTRL_START     |
                ADS7846_GET_Z1         |
                ADS7846_CTRL_12BIT     |
                ADS7846_CTRL_DIFMODE);
        break;

    case STATE_Z2:
        ctrl = (ADS7846_CTRL_START     |
                ADS7846_GET_Z2         |
                ADS7846_CTRL_12BIT     |
                ADS7846_CTRL_DIFMODE);
        break;

    default:
        printk("Unknown state\n");
        ads.state = STATE_NONE;
        return;
        break;
    }

    SSP1_TX_REG = ctrl;
    SSP1_TX_REG = 0;
    SSP1_TX_REG = 0;
    SSP1_CTRL_REG |= SSP_XCH;

    SSP1_INT_REG |= SSP_INT_TEEN;
}



static int __init ads7846_init(void)
{
    int rc;
    unsigned long tmp;

//    printk("%s\n", DRIVER_DESC);

    /* Configure GPIO */
    imx_gpio_mode(14 | GPIO_PORTC | GPIO_OUT);    /* SCLK */
    imx_gpio_mode(15 | GPIO_PORTC | GPIO_OUT);    /* *SS */
    imx_gpio_mode(16 | GPIO_PORTC | GPIO_IN);     /* MISO */
    imx_gpio_mode(17 | GPIO_PORTC | GPIO_OUT);    /* MOSI */

    imx_gpio_mode(18 | GPIO_PORTB | GPIO_IN | GPIO_PUEN);     /* *PENIRQ */

    if (request_mem_region(__REG(IMX_SPI1_BASE), 32, "imx-spi1") == NULL) {
      printk("Error requesting region for SPI\n");
    }

    /* Install ISR */
    rc = request_irq(CSPI_INT, &ads7846_spi_isr, 0, "ADS7846-SPI", NULL);
    if (rc < 0) {
        printk(KERN_ERR "Could not register ADS7846 SPI interrupt\n");
        return rc;
    }

    /* Configure SPI port */
    SSP1_CTRL_REG = (
        SSP_RATE_DIV256  |  /*  PCLK2 = SYSCLK/4 = 24MHz. 24MHZ/125KHz = 192 */
        SSP_MODE_MASTER  |
        SSP_SS_POL_LOW   |
        SSP_ENABLE       |
        SSP_PHA1         |
        SSP_POL1         |
        SSP_WS(8));


    /* Configure ADS7846 */
    SSP1_TX_REG = (
        ADS7846_CTRL_12BIT      |
        ADS7846_CTRL_DIFMODE);

    SSP1_CTRL_REG |= SSP_XCH;
    while ((SSP1_CTRL_REG & SSP_XCH) || (SSP1_INT_REG & SSP_INT_RR)) {
        tmp = SSP1_RX_REG;
    }

    /* Configure local data */
    ads.state = STATE_NONE;
    ads.pendown = 0;
    init_timer(&ads.timer);
    ads.timer.data = 0;
    ads.timer.function = ads7846_timer;
    mod_timer(&ads.timer, jiffies + (HZ/100));


    /* Initialize input device */
    init_input_dev(&ads.input);
    ads.input.evbit[0] = BIT(EV_KEY) | BIT(EV_ABS);
    ads.input.keybit[LONG(BTN_TOUCH)] = BIT(BTN_TOUCH);
    input_set_abs_params(&ads.input, ABS_X, X_MIN, X_MAX, 0, 0);
    input_set_abs_params(&ads.input, ABS_Y, Y_MIN, Y_MAX, 0, 0);
    input_set_abs_params(&ads.input, ABS_PRESSURE, PRES_MIN, PRES_MAX, 0, 0);
    ads.input.name = "CSB536 ADS7846 Touchscreen";
    ads.input.id.bustype = BUS_HOST;
    ads.input.id.vendor = 1;
    ads.input.id.product = 2;
    ads.input.id.version = 1;

    input_register_device(&ads.input);

    return 0;
}

static void __exit ads7846_exit(void)
{
#warning "Need to finish ads7846_exit()"
}


module_init(ads7846_init);
module_exit(ads7846_exit);

