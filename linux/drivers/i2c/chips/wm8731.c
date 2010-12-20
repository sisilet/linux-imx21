/*
 *  wm8731.c - Driver for I2C interface on WM8731 CODEC
 *  Copyright (C) 2005 Jay Monkman <jtm@lopingdog.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/i2c-sensor.h>

/* Used for size of register mirror array, some are nonexistant */
#define WM8731_NUM_REGS 16

/* Each client has this additional data */
typedef struct {
    struct i2c_client client;
    struct semaphore update_lock;

    /* The WM8731 is write only, so we mirror writes and fake read accesses */
    short regs[WM8731_NUM_REGS];
} wm8731_data_t;

static int wm8731_attach_adapter(struct i2c_adapter *adapter);
static int wm8731_detect(struct i2c_adapter *adapter, int address, int kind);
static int wm8731_detach_client(struct i2c_client *client);
static void wm8731_init_client(struct i2c_client *client);
int wm8731_write_reg(struct i2c_client *client, u8 reg, u16 val);
int wm8731_read_reg(struct i2c_client *client, u8 reg, u16 *val);


/* This is the driver that will be inserted */
static struct i2c_driver wm8731_driver = {
        .owner          = THIS_MODULE,
        .name           = "Wolfson wm8731 CODEC driver",
        .id             = I2C_DRIVERID_EXP0, /* FIXME: Should be assigned ID */
        .flags          = I2C_DF_NOTIFY,
        .attach_adapter = wm8731_attach_adapter,
        .detach_client  = wm8731_detach_client,
};

/* Used to keep track of number of devices */
static int wm8731_id;

/* Addresses to scan */
static unsigned short wm8731_addrs[] = { 0x1a, 0x1b, I2C_CLIENT_END};

#if defined(CONFIG_SND_CSB536FS) || defined(CONFIG_SND_CSB536PC)
#warning "FIXME: Find a better way to do this"
struct i2c_client *csb536_wm8731_client = NULL;
#endif

int wm8731_write_reg(struct i2c_client *client, u8 reg, u16 val)
{
    u8 d0;
    u8 d1;
    int rc;
    wm8731_data_t *data = i2c_get_clientdata(client);

    if (reg & 0x80) {
        return -1;
    }

    d0 = reg << 1 | ((val >> 8) & 1);
    d1 = val & 0xff;

    rc = i2c_smbus_write_byte_data(client, d0, d1);

    if (rc == 0) {
        /* successful write, mirror the data */
        data->regs[reg] = val;
    }

    return rc;
}

int wm8731_read_reg(struct i2c_client *client, u8 reg, u16 *val)
{
    wm8731_data_t *data = i2c_get_clientdata(client);

    if (reg & 0x80) {
        return -1;
    }

    *val = data->regs[reg];
    return 0;
}


/* Probe for devices */
static int wm8731_attach_adapter(struct i2c_adapter *adapter)
{
    int i;

    /* dummy client */
    struct i2c_client client;
    wm8731_data_t data;

    for (i = 0; i < sizeof(wm8731_addrs)/sizeof(wm8731_addrs[0]); i++ ) {
        /* create a dummy client */
        client.adapter = adapter;
        client.addr = wm8731_addrs[i];
        i2c_set_clientdata(&client, &data);

        if (wm8731_write_reg(&client, 0, 0) == 0) {
            printk("Found WM8731 at address 0x%x\n", wm8731_addrs[i]);

            wm8731_detect(adapter, wm8731_addrs[i], -1);
        }
    }

    return 0;
}

int wm8731_detect(struct i2c_adapter *adapter, int address, int kind)
{
    struct i2c_client *new_client;
    wm8731_data_t *data;
    int err = 0;

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
        goto exit;

    /*
     * OK. For now, we presume we have a valid client. We now create the
     * client structure, even though we cannot fill it completely yet.
     */
    if (!(data = kmalloc(sizeof(wm8731_data_t), GFP_KERNEL))) {
        err = -ENOMEM;
        goto exit;
    }
    memset(data, 0, sizeof(wm8731_data_t));
    
    new_client = &data->client;
    i2c_set_clientdata(new_client, data);
    new_client->addr = address;
    new_client->adapter = adapter;
    new_client->driver = &wm8731_driver;
    new_client->flags = 0;

    /* Fill in the remaining client fields and put it into the global list */
    strlcpy(new_client->name, "wm8731", I2C_NAME_SIZE);
    
    init_MUTEX(&data->update_lock);
    
    /* Tell the I2C layer a new client has arrived */
    if ((err = i2c_attach_client(new_client)))
        goto exit_free;
        
    /* Initialize the WM8731 chip */
    wm8731_init_client(new_client);

    return 0;

 exit_free:
    kfree(data);
 exit:
    return err;
}

static int wm8731_detach_client(struct i2c_client *client)
{
    int err;

    if ((err = i2c_detach_client(client))) {
        dev_err(&client->dev,
                "Client deregistration failed, client not detached.\n");
        return err;
    }

    kfree(i2c_get_clientdata(client));
    return 0;
}

/* Called when we have found a new WM8731. */
static void wm8731_init_client(struct i2c_client *client)
{
    /*
     * Write default values to registers, mainly to initialize 
     * mirror of them.
     */
    wm8731_write_reg(client, 15, 0x00);   /* Reset */
    udelay(1000);
    
    wm8731_write_reg(client,  0, 0x97);   /* Left line in */
    wm8731_write_reg(client,  1, 0x97);   /* Right line in */
    wm8731_write_reg(client,  2, 0x79);   /* Left headphone out */
    wm8731_write_reg(client,  3, 0x79);   /* Right headphone out */
    wm8731_write_reg(client,  4, 0x0a);   /* Analog audio path ctrl */
    wm8731_write_reg(client,  5, 0x08);   /* Digital audio path ctrl */
    wm8731_write_reg(client,  6, 0x9f);   /* Power down */
    wm8731_write_reg(client,  7, 0x0a);   /* Digital audio iface fmt */
    wm8731_write_reg(client,  8, 0x00);   /* Sampling ctrl */
    wm8731_write_reg(client,  9, 0x00);   /* Active ctrl */

#if (defined(CONFIG_SND_CSB536FS) || defined(CONFIG_SND_CSB536PC))
    csb536_wm8731_client = client;
#endif
}

static int __init wm8731_init(void)
{
    return i2c_add_driver(&wm8731_driver);
}

static void __exit wm8731_exit(void)
{
    i2c_del_driver(&wm8731_driver);
}


MODULE_AUTHOR("Jay Monkman <jtm@lopingdog.com>");
MODULE_DESCRIPTION("WM8731 driver");
MODULE_LICENSE("GPL");

module_init(wm8731_init);
module_exit(wm8731_exit);

EXPORT_SYMBOL(wm8731_write_reg);
EXPORT_SYMBOL(wm8731_read_reg);
