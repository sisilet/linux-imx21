/*
 * ds1307.c
 *
 * Device driver for Dallas Semiconductor's Real Time Controller DS1307.
 *
 * Copyright (C) 2002 Intrinsyc Software Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/miscdevice.h>

#include <asm/rtc.h>

#define DS1307_RAM_ADDR_START	0x08
#define DS1307_RAM_ADDR_END	0x3F
#define DS1307_RAM_SIZE 	0x40

#define SQW_ENABLE		0x10	/* Square Wave Enable */
#define SQW_DISABLE		0x00	/* Square Wave disable */

#define RATE_32768HZ		0x03	/* Rate Select 32.768KHz */
#define RATE_8192HZ		0x02	/* Rate Select 8.192KHz */
#define RATE_4096HZ		0x01	/* Rate Select 4.096KHz */
#define RATE_1HZ		0x00	/* Rate Select 1Hz */

#define CLOCK_HALT		0x80	/* Clock Halt */

#define TWELVE_HOUR_MODE(n)	(((n)>>6)&1)
#define HOURS_AP(n)		(((n)>>5)&1)
#define HOURS_12(n)		BCD2BIN((n)&0x1F)
#define HOURS_24(n)		BCD2BIN((n)&0x3F)

struct ds1307_data {
	unsigned char control;
};

//#define WHEREAMI()	printk("[SZ DS1307]: %s(%d)\n", __FUNCTION__, __LINE__);
#define WHEREAMI()

// The DS1307 can only ever be present at address 0x68
static unsigned short normal_i2c[]      = { 0x68, I2C_CLIENT_END };
static unsigned short normal_i2c_range[] = { I2C_CLIENT_END };

static int sqwave = -1;

I2C_CLIENT_INSMOD;

struct i2c_driver ds1307_driver;
//static int ds1307_id;

static struct i2c_client *rtc_client = NULL;

static int
ds1307_readram(struct i2c_client *client, char *buf, int len)
{
        unsigned char ad[1] = { 0 };
        int ret;
        struct i2c_msg msgs[2] = {
                { client->addr, 0,        1, ad  },
                { client->addr, I2C_M_RD, len, buf } };

        ret = i2c_transfer(client->adapter, msgs, 2);

        return ret;
}

static void
ds1307_convert_to_time(struct rtc_time *dt, char *buf)
{
	dt->tm_sec = BCD2BIN(buf[0]);
	dt->tm_min = BCD2BIN(buf[1]);

	if (TWELVE_HOUR_MODE(buf[2])) {
		dt->tm_hour = HOURS_12(buf[2]);
		if (HOURS_AP(buf[2])) { /* PM */
			dt->tm_hour += 12;
		}
	} else { /* 24-hour-mode */
		dt->tm_hour = HOURS_24(buf[2]);
	}

	dt->tm_mday = BCD2BIN(buf[4]);
	/* dt->tm_mon is zero-based */
	dt->tm_mon = BCD2BIN(buf[5]) - 1;
	/* year is 1900 + dt->tm_year */
	dt->tm_year = BCD2BIN(buf[6]) + 100;
}

static int ds1307_proc(char *buf)
{
	struct i2c_client *client = rtc_client;
#define CHECK(ctrl,bit) ((ctrl & bit) ? "yes" : "no")
	unsigned char ram[DS1307_RAM_SIZE];
	int ret;

	char *p = buf;
	int i;
	char text[9];

	ret = ds1307_readram(client, ram, DS1307_RAM_SIZE);
	if (ret < 0) {
		p += sprintf(p, "Failed to read RTC memory!\n");
		return p - buf;
	}

	p += sprintf(p, "halted\t\t: %s\n", CHECK(ram[0],0x80));
	p += sprintf(p, "24hr\t\t: %s\n", CHECK(ram[2],0x40));
	p += sprintf(p, "square_wave\t: %s\n", CHECK(ram[7],0x10));
	p += sprintf(p, "freq\t\t: ");

	switch (ram[7] & 0x03) {
	case RATE_1HZ:
		p += sprintf(p, "1Hz\n");
		break;
	case RATE_4096HZ:
		p += sprintf(p, "4.096kHz\n");
		break;
	case RATE_8192HZ:
		p += sprintf(p, "8.192kHz\n");
		break;
	case RATE_32768HZ:
	default:
		p += sprintf(p, "32.768kHz\n");
		break;
	}

	p += sprintf(p, "ram:\n");
	text[8]='\0';
	for (i=0; i<DS1307_RAM_SIZE; i++) {
		p += sprintf(p, "%02X ", ram[i]);

		if ((ram[i] < 32) || (ram[i]>126)) ram[i]='.';
		text[i%8] = ram[i];
		if ((i%8) == 7) p += sprintf(p, "%s\n",text);
	}

	p += sprintf(p, "\n");
	return p - buf;
}

static inline int
ds1307_set_ctrl(struct i2c_client *client, unsigned char *cinfo)
{
	int ret;
	struct ds1307_data *data = i2c_get_clientdata(client);
	data->control = *cinfo;
	ret = i2c_smbus_write_byte_data(client, 0x07, data->control);
	return ret;
}

static void ds1307_enable_sqwave(struct i2c_client *client, int rate)
{
	unsigned char ctrl_info;

	if (rate == -1)
		ctrl_info = SQW_DISABLE;
	else
		ctrl_info = SQW_ENABLE | rate;

	ds1307_set_ctrl(client, &ctrl_info);

}

static void ds1307_enable_clock(struct i2c_client *client)
{
	/* Clear Clock-Halt bit if necessary. If CLOCK_HALT is set
	 * then the second counter isn't useful anway so we may as
	 * well zero it. */
	if (i2c_smbus_read_byte_data(client, 0x0) & CLOCK_HALT)
		i2c_smbus_write_byte_data(client, 0, 0);
}

static int
ds1307_open(void)
{
	struct i2c_client *client = rtc_client;
	struct ds1307_data *data;
	u32 reg;

	if (client == NULL)
		return -ENODEV;

	data = i2c_get_clientdata(client);

	if (data == NULL)
		return -ENODEV;

	/* re-read ctrl register to ensure there really is a device
	 * there. if we have a situation where the device is present
	 * but incorrectly connected (or just faulty) then we may seem
	 * to be reading/writing OK but really we are getting junk --
	 * so lets test that the CTRL register is really what we think
	 * it is. If it isn't then it is likely that we don't have a
	 * valid device attached */

	reg = i2c_smbus_read_byte_data(client, 0x07);
	if (reg == -1) {
		printk (KERN_DEBUG "ds1307_open: could not verify ctrl - read returned %d.\n", reg);
		return -ENODEV;
	} else if ((reg&~0x20) != (data->control&~0x20)) { /* 0x20 is OSF bit, ignore it */
		printk(KERN_DEBUG "ds1307_open: failed to verify ctrl register. "
		       "got 0x%02x wanted 0x%02x\n",
		       reg, data->control);
		return -ENODEV;
	}

	return 0;
}

static void
ds1307_release(void)
{
	/* FIXME: nothing to do? */
}

static int
ds1307_read_time(struct rtc_time *time)
{
	struct i2c_client *client = rtc_client;
        unsigned char buf[7], addr[1] = { 0 };
        struct i2c_msg msgs[2] = {
                { client->addr, 0,        1, addr },
                { client->addr, I2C_M_RD, 7, buf  }
        };

        memset(buf, 0, sizeof(buf));
	buf[0]= 0xDE;
	buf[1]= 0xAD;
	buf[2]= 0xBE;
	buf[3]= 0xEF;
	buf[4]= 0x13;
	buf[5]= 0x37;
	buf[6]= 0x00;

        if (i2c_transfer(client->adapter, msgs, 2) == 2) {
		ds1307_convert_to_time(time, buf);
		return 0;
        } else {
                printk("ds1307_read_time failed\n");
	}

	return -EIO;
}

static int
ds1307_set_time(struct rtc_time *time)
{
	struct i2c_client *client = rtc_client;
        unsigned char buf[8];

        buf[0] = 0;     /* register address on DS1307 */
        buf[1] = (BIN2BCD(time->tm_sec));
        buf[2] = (BIN2BCD(time->tm_min));
        buf[3] = (BIN2BCD(time->tm_hour));
	/* we skip buf[4] as we don't use day-of-week. */
	buf[5] = (BIN2BCD(time->tm_mday));
	buf[6] = (BIN2BCD(time->tm_mon + 1));
	/* The year only ranges from 0-99, we are being passed an offset from 1900,
	 * and the chip calulates leap years based on 2000, thus we adjust by 100.
	 */
	buf[7] = (BIN2BCD(time->tm_year - 100));

        if (i2c_master_send(client, (char *)buf, 8) == 8)
		return 0;

	printk("ds1307_set_time failed\n");
        return -EIO;
}

static int
ds1307_ioctl(unsigned int cmd, unsigned long arg)
{
	return -EINVAL;
}

static struct rtc_ops ds1307_ops = {
        .owner          = THIS_MODULE,
        .open           = ds1307_open,
        .release        = ds1307_release,
        .ioctl          = ds1307_ioctl,

        .read_time      = ds1307_read_time,
        .set_time       = ds1307_set_time,

        .proc           = ds1307_proc,
};

static void ds1307_init_client(struct i2c_client *client)
{
	struct ds1307_data *data = i2c_get_clientdata(client);
	data->control = i2c_smbus_read_byte_data(client, 0x07);
	if ( sqwave < -1 || sqwave > 3 )
		sqwave=-1;
	ds1307_enable_sqwave(client, sqwave);
	ds1307_enable_clock(client);
}

/* The `kind' parameter contains 0 if this call is due to a `force'
   parameter, and -1 otherwise */
static int
ds1307_detect(struct i2c_adapter *adapter, int address, int kind)
{
WHEREAMI();
	struct i2c_client *new_client;
	struct ds1307_data *data;
	int err = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA |
				              I2C_FUNC_I2C)) {
		printk(KERN_ERR "ds1307: required I2C functionality not supported by adapter\n");
		/*goto exit;*/
	}
WHEREAMI();
	if (kind < 0 && address != 0x68) /* DS1307 is always at address 0x68 */
		goto exit;

	/* OK. For now, we presume we have a valid client. We now create the
	   client structure. */
WHEREAMI();
	if (!(new_client = kmalloc(sizeof(struct i2c_client) +
				   sizeof(struct ds1307_data),
				   GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}
WHEREAMI();
	memset(new_client, 0, sizeof(struct i2c_client) +
	       sizeof(struct ds1307_data));
	data = (struct ds1307_data *) (new_client + 1);
	i2c_set_clientdata(new_client, data);
	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &ds1307_driver;
	new_client->flags = 0;

	/* Could do any remaing detection here -- if kind < 0 */

	/* Fill in remaining client fields and put it into the global list */
	strlcpy(new_client->name, "ds1307", I2C_NAME_SIZE);

//	new_client->id = ds1307_id++;

WHEREAMI();
	/* Tell the I2C layer a new client has arrived */
	if ((err = i2c_attach_client(new_client)))
		goto exit_free;
WHEREAMI();

	/* Initialize the DS1307 chip */
	ds1307_init_client(new_client);

	rtc_client = new_client;

	err = register_rtc(&ds1307_ops);
	if (err) {
		printk(KERN_ERR "ds1307: could not register RTC.\n");
	}
WHEREAMI();

	return 0;

      exit_free:
	kfree(new_client);
      exit:
	return err;
}

static int ds1307_attach_adapter(struct i2c_adapter *adapter)
{
WHEREAMI();
        /*
         * Probing seems to confuse the RTC.
         * Not sure if it's true for other boards though.
         */
	//return ds1307_detect(adapter, 0x68, 0);

	return i2c_probe(adapter, &addr_data, ds1307_detect);
}

static int ds1307_detach_client(struct i2c_client *client)
{
	int err;

	unregister_rtc(&ds1307_ops);

	if ((err = i2c_detach_client(client))) {
		dev_err(&client->dev,
		        "ds1307.o: Client deregistration failed, client not detached.\n");
		return err;
	}

	kfree(client);
	return 0;
}

struct i2c_driver ds1307_driver = {
	.name           = "ds1307",
	.id		= I2C_DRIVERID_DS1307,
	.flags		= I2C_DF_NOTIFY,
	.attach_adapter	= ds1307_attach_adapter,
	.detach_client	= ds1307_detach_client,
};

static __init int ds1307_init(void)
{
WHEREAMI();
	return i2c_add_driver(&ds1307_driver);
}

static __exit void ds1307_exit(void)
{
	i2c_del_driver(&ds1307_driver);
}

module_init(ds1307_init);
module_exit(ds1307_exit);
module_param(sqwave, int, -1);
MODULE_PARM_DESC(sqwave, "set the square wave output. -1=off, 0=1Hz, 1=4.096kHz, 2=8.192kHz, 3=32.768kHz");

MODULE_AUTHOR ("Intrinsyc Software Inc.");
MODULE_LICENSE("GPL");

MODULE_ALIAS_MISCDEV(RTC_MINOR);
