/******************************************************************************
 *	Copyright (C) 2002 Motorola GSG-China
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version 2
 *	of the License, or (at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program; if not, write to the Free Software
 *	Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307,
 *      USA.
 *
 *  File Name   : i2c-scb9328.c
 *  Description : Implementation of i2c Adapter/Algorithm Driver
 *  Author      : Motorola GSG-China
 *  History     : 2002/2/7 use msgs[]
 *                2004/3/3 port to linux kernel v2.6.x for IMX
 *                         adding  cpu- and bus-usage optimization
 *                         (by T.Koschorrek, http://www.synertronixx.de/)
 *
 *  Additional Description (T. Koschorrek) :
 *  - to enable debug messages just #define DEBUG
 *  - this module supports cpu- and bus-usage-optimization for ad-converter
 *    'ads1110' and for serial access timekeeper 'm41t00'
 *  - support for other chips is NOT yet implemented (you can implement other
 *    chips by adding two lines in the init-function (i2c_imx_init)
 *  - this module can be accessed via the i2c-dev interface from user-space
 *  - access from kernel-space is not supported (add this if you want)
 *
 *  - TODO: support for chips (ads1110, m41t00) works fine here but should be
 *          handled in seperate modules
 *
 ******************************************************************************/

/******************************************************************************
 * global stuff
 ******************************************************************************/
/*-----------------------------------------------------------------------------
 * included header files
 *----------------------------------------------------------------------------*/
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/spinlock.h>

#include <asm/arch/irqs.h>
#include <asm/arch/hardware.h>

#include <asm/leds.h>

/* #define DEBUG */

/*-----------------------------------------------------------------------------
 * defines
 *----------------------------------------------------------------------------*/
#define DEFAULT_FREQ 	   0x16
#define I2C_IO_CHANGE_FREQ 0xaa
#define I2C_IO_GET_STATUS  0xab
//#define I2C_IMX_TIMEOUT    5000000 
#define I2C_IMX_TIMEOUT    5000000 // blaschke

/*-----------------------------------------------------------------------------
 * function declarations
 *----------------------------------------------------------------------------*/
static int i2c_imx_xfer(struct i2c_adapter *i2c_adap, struct i2c_msg msgs[ ],
			int num);
static int i2c_imx_smbus(struct i2c_adapter * adapter, u16 addr,
			 unsigned short flags, char read_write, u8 command,
			 int size, union i2c_smbus_data * data);
static int i2c_imx_ioctl(struct i2c_adapter * adapter, unsigned int cmd,
			 unsigned long arg);

/*-----------------------------------------------------------------------------
 * structs for i2c
 *----------------------------------------------------------------------------*/
/* algorithm structure for handling with i2c-bus */
static struct i2c_algorithm i2c_imx_algorithm = {
	/* name: */		"IMX I2C algorithm",
	/* id: */		I2C_ALGO_BIT,
	/* master_xfer: */	i2c_imx_xfer,
	/* smbus_xfer: */	i2c_imx_smbus,
	/* slave_send: */	NULL,
	/* slave_recv: */	NULL,
	/* algo_control: */	i2c_imx_ioctl,
	/* functionality: */	NULL
};

/* adapter structure to the i2c-bus */
static struct i2c_adapter i2c_imx_adapter = {
	/* name: */           .name = "IMX I2C adapter",
	/* i2c_algorithm: */  .algo = &i2c_imx_algorithm,
	/* class: */          .class = I2C_CLASS_HWMON,
	/* ID: */             .id = I2C_ALGO_BIT | I2C_HW_B_SER,
};

/* imx-registers for i2c */
struct i2c_imx_i2creg {
  	volatile u32 iadr;
	volatile u32 ifdr;
	volatile u32 i2cr;
	volatile u32 i2sr;
	volatile u32 i2dr;
};

struct i2c_imx_i2cslp {
	unsigned long jif;
	unsigned int slp_time;
};

/*-----------------------------------------------------------------------------
 * variables
 *----------------------------------------------------------------------------*/
static struct i2c_imx_i2creg *i2c_imx_reg = (struct i2c_imx_i2creg *)MX2ADS_I2C_BASE;
static spinlock_t i2c_imx_irqlock = SPIN_LOCK_UNLOCKED;
static int i2c_imx_irq_ok;
static int i2c_imx_i2sr;
static struct i2c_imx_i2cslp i2c_imx_slp[128];

#ifdef DEBUG
static int i2c_imx_errs_rxack = 0;
static int i2c_imx_errs_txcomplete = 0;
static int i2c_imx_errs_busbusy = 0;
static int i2c_imx_errs_busgrab = 0;
#endif

/******************************************************************************
 * functions (i2c-driver for IMX)
 ******************************************************************************/
/*-----------------------------------------------------------------------------
 * bus grab
 *----------------------------------------------------------------------------*/
static int i2c_imx_bus_grab(void)
{
	long int i = 0;
  #define MAX_GRAB_TIMEOUT 2000
	
	/* wait for bus grab */
	/*while(  !((i2c_imx_reg->i2sr & (u32)0x30)));
	if (i == I2C_IMX_TIMEOUT) 
	  return 4;
	return 0;
	*/
	//blaschke
	while(  (!((i2c_imx_reg->i2sr & (u32)0x30))) && (i<MAX_GRAB_TIMEOUT) )
	{i++;};
	
	if (i == MAX_GRAB_TIMEOUT)	
		return 4;
	return 0;
}

/*-----------------------------------------------------------------------------
 * bus busy
 *----------------------------------------------------------------------------*/
static int i2c_imx_bus_busy(void)
{
	int i = 0;

	
	/* wait for bus not busy */
	while((i2c_imx_reg->i2sr & (u32)0x20) && (i < I2C_IMX_TIMEOUT)) i++;

	if (i == I2C_IMX_TIMEOUT) return 3;
	return 0;
}

/*-----------------------------------------------------------------------------
 * received ack
 *----------------------------------------------------------------------------*/
static int i2c_imx_received_acknowledge(void)
{
	if(i2c_imx_i2sr & 0x01)
	{ 
	  return 1;
	}
	return 0;
}

/*-----------------------------------------------------------------------------
 * transfer complete
 *----------------------------------------------------------------------------*/
static int i2c_imx_transfer_complete(void)
{
	int i = 0;
	unsigned long flags;

	i2c_imx_i2sr = 0x00;

	/* wait for transfer complete interrupt */
	while ((!(i2c_imx_irq_ok)) && (i < I2C_IMX_TIMEOUT)){
		spin_lock_irqsave(&i2c_imx_irqlock, flags);
		i++;
		spin_unlock_irqrestore(&i2c_imx_irqlock, flags);
	}

	i2c_imx_irq_ok = 0;

	if (i == I2C_IMX_TIMEOUT) return 3;
	return 0;
}

/*-----------------------------------------------------------------------------
 * bus release
 *----------------------------------------------------------------------------*/
static int i2c_imx_bus_release(void)
{	
	int i=0, dummy;
	
	/* if bus busy reset the module process suggested in reference manual
	   (tahiti) */
	if (i2c_imx_bus_busy() && (i < I2C_IMX_TIMEOUT)) {
		i2c_imx_reg->i2cr  =   (u32)0x00;
		i2c_imx_reg->i2cr |=   (u32)0x0a;
		dummy = (u8)i2c_imx_reg->i2dr;
		i2c_imx_reg->i2sr  =   (u32)0x00;
		i2c_imx_reg->i2cr  =   (u32)0x00;
		i2c_imx_reg->i2cr &= ~((u32)0x80);
		i++;
	}

	return 0;
}

/*-----------------------------------------------------------------------------
 * start
 *----------------------------------------------------------------------------*/
static int i2c_imx_start(void)
{
	int i = 0;

	/* Set Master Mode */
	i2c_imx_reg->i2cr |= 0x20;

	/* wait while bus grab */
	//while((i2c_imx_bus_grab()) && (i < I2C_IMX_TIMEOUT)) {// blaschke
	while((i2c_imx_bus_grab()) && (i < 5)) {	
		i2c_imx_reg->i2cr &= ~((u32)0x20);	  /* bus stop */
		i2c_imx_bus_release();
		if (i2c_imx_bus_busy()) 
		  return 3;
		i2c_imx_reg->i2cr |=   (u32)0x80; /* [I2CR:IEN] (I2C Enable) */
		i2c_imx_reg->i2cr |=   (u32)0x08; /* [I2CR:TXAK] dable txack */
		i2c_imx_reg->i2cr |=   (u32)0x20;         /* Set Master Mode */
		i++;
	}

	if (i == I2C_IMX_TIMEOUT) return 4;
	return 0;
}

/*-----------------------------------------------------------------------------
 * write
 *----------------------------------------------------------------------------*/
static int i2c_imx_write(int i, int *count, struct i2c_msg *msgs)
{
	int err, j;

	/* select slave */
	i2c_imx_reg->i2dr = msgs->addr << 1;
	if ((err = i2c_imx_transfer_complete())) return err;
	if ((err = i2c_imx_received_acknowledge())) return err;

	/* write data */
	for ( j = 0; j < msgs->len; j ++ )
	{
		i2c_imx_reg->i2dr = msgs->buf[j];
		if ((err = i2c_imx_transfer_complete())) return err;
	}

	*count += msgs->len;

	return 0;
}

/*-----------------------------------------------------------------------------
 * read
 *----------------------------------------------------------------------------*/
static int i2c_imx_read(int i, int *count, struct i2c_msg *msgs)
{
	int err, j, dummy;

	/* select slave */
	i2c_imx_reg->i2dr = ((u32)msgs->addr << 1) | (u32)0x01;
	if ((err = i2c_imx_transfer_complete())) return err;
	if ((err = i2c_imx_received_acknowledge())) return err;

	/* setup bus to read data */
	i2c_imx_reg->i2cr &= ~(u32)0x10;
	if(msgs->len-1) i2c_imx_reg->i2cr &= ~(u32)0x08;
	dummy = (u8)i2c_imx_reg->i2dr; /* trigger rec. of next byte */
	if ((err = i2c_imx_transfer_complete())) return err;

	/* read data */
	for ( j = 0; j < msgs->len; j ++ )
	{
		if(j== (msgs->len - 1)) i2c_imx_reg->i2cr |= 0x08; //blaschke: letztes Byte ohne Ack
		
		msgs->buf[j] = (u8)i2c_imx_reg->i2dr;	              
		if ((err = i2c_imx_transfer_complete())) return err;
	}

	
	/*	torsten   for ( j = 0; j < msgs->len; j ++ ){
		f(j== (msgs->len - 2)) i2c_imx_reg->i2cr |= 0x08;
		msgs->buf[j] = (u8)i2c_imx_reg->i2dr;	              
		if ((err = i2c_imx_transfer_complete())) return err;
	}
  */
	
	count += msgs->len;

	return 0;
}

/*-----------------------------------------------------------------------------
 * chip_sleep functions: gets timeout-values for supported chips
 * you have to add own functions to get optimal timeouts for your own chips 
 * supported chips: m41t00 at 0x68, ads1110 at 0x48
 *----------------------------------------------------------------------------*/
/* support for ads1110 at address 0x48 */
void chip_slp_48(struct i2c_msg *msgs) {
	switch(msgs->buf[0] & 0x0c) {
	case 0x00:
		i2c_imx_slp[0x48-1].slp_time = 1;
		break;
	case 0x04:
		i2c_imx_slp[0x48-1].slp_time = 20;
		break;
	case 0x08:
		i2c_imx_slp[0x48-1].slp_time = 40;
		break;
	case 0x0c:
		i2c_imx_slp[0x48-1].slp_time = 70;
		break;
	}
	return;
}

/* chip_slp */
void chip_slp(struct i2c_msg *msgs) {
	switch(msgs->addr) {
	case 0x48:
		chip_slp_48(msgs);
		return;
	}
}

/*-----------------------------------------------------------------------------
 * void i2c_imx_isr (s16 irq, void * dev_id, struct pt_regs * reg)
 * This function deals with the interrupt for the I2C module.
 *
 * Parameters:	irq		the interrupt number
 *              dev_id		device id
 * 	        reg		processor register	
 * Return: 	IRQ_HANDLED	the own irq was handled
 *----------------------------------------------------------------------------*/
static irqreturn_t i2c_imx_isr (s16 irq, void *dev_id, struct pt_regs * reg)
{
	/* safe status register */
	i2c_imx_i2sr = i2c_imx_reg->i2sr;

	/* if data transfer is complete set ok */
	if (i2c_imx_i2sr & (u32)0x80)  /* [I2SR:ICF] TX complete */
		i2c_imx_irq_ok = 1;

	/* clear irq */
	i2c_imx_reg->i2sr &= ~(u32)0x02; /* clear [I2SR:IIF] Interrupt */

	return IRQ_HANDLED;
}

/*-----------------------------------------------------------------------------
 * int i2c_imx_xfer(struct i2c_adapter *i2c_adap, struct i2c_msg msgs[ ],
 * SINT16 num)
 * This function is responsible for the transfer of the data through the
 * I2C bus
 * 
 * Parameter:   i2c_adap	associated with the related hardware
 *              msgs[ ] 	the body of the message to be send out
 *              num		number of message
 * Return:      Success		Number of message has been transferred
 *	        Failure		-err (error code)
 *----------------------------------------------------------------------------*/
static int i2c_imx_xfer(struct i2c_adapter *i2c_adap, struct i2c_msg msgs[],
			int num)
{
	int i, count=0, err=0;

	//  20041202 Torsten
	/*unsigned long jif = i2c_imx_slp[msgs[0].addr-1].jif;
	int slp_time = i2c_imx_slp[msgs[0].addr-1].slp_time;
	int slp = slp_time * HZ / 1000 - (jiffies - jif);
	DECLARE_WAIT_QUEUE_HEAD(wait);


	// scheduler: sleep while chip not ready
	// 20041202 Torsten
	if ((slp > 1) && (msgs[0].flags & I2C_M_RD)) {
		up(&i2c_adap->bus_lock);
		sleep_on_timeout(&wait, slp);
		down(&i2c_adap->bus_lock);
	}
*/
	
	/* enable the i2c-bus, disable the I2C transmit ACK */
	if(i2c_imx_bus_busy()) goto ERROR;
	i2c_imx_reg->i2cr |= (u32)0x80;
	i2c_imx_reg->i2cr |= (u32)0x08;

	for ( i = 0; i < num; i ++)
	{
		/* repeat start else start the bus-transfer */
		
		if(i)
			i2c_imx_reg->i2cr |= (u32)0x04;
		else
			if ((err = i2c_imx_start())) 
			  goto ERROR;

		/* enable interrupt, enable master transmit */
		i2c_imx_reg->i2cr |= (u32)0x40;
		i2c_imx_reg->i2cr |= (u32)0x10;

		/* write/read data */
		if (!(msgs[i].flags & I2C_M_RD )) 
		{
			if ((err = i2c_imx_write(i, &count, &msgs[i]))) goto ERROR;
		} else {
			if ((err = i2c_imx_read(i, &count, &msgs[i]))) goto ERROR;
		}
	}

	
	/*exit function and error handler */
	i2c_imx_reg->i2cr &= ~((u32)0x20); /* bus stop */
	i2c_imx_bus_release();
	i2c_imx_reg->i2cr &= ~((u32)0x80); /* disable I2C */

	if (!(msgs[0].flags & I2C_M_RD )) 
	  chip_slp(&msgs[0]);
	  	
	i2c_imx_slp[msgs[0].addr-1].jif = jiffies;

	return count;
 
ERROR:
#ifdef DEBUG
	printk("ERROR%d\n", err);
	if(err == 1) i2c_imx_errs_rxack++;
	if(err == 2) i2c_imx_errs_txcomplete++;
	if(err == 3) i2c_imx_errs_busbusy++;
	if(err == 4) i2c_imx_errs_busgrab++;

	printk("Last Update: 2004.12.07: 11:25\n");
	printk("RX ACK      ERRORs: %d\n", i2c_imx_errs_rxack);
	printk("TX COMPLETE ERRORs: %d\n", i2c_imx_errs_txcomplete);
	printk("BUS BUSY    ERRORs: %d\n", i2c_imx_errs_busbusy);
	printk("BUS GRAB    ERRORs: %d\n", i2c_imx_errs_busgrab);

#endif
	// blaschke
	i2c_imx_reg->i2cr &= ~((u32)0x20); /* bus stop */
	i2c_imx_bus_release();
	i2c_imx_reg->i2cr &= ~((u32)0x80); /* disable I2C */

	if (!(msgs[0].flags & I2C_M_RD )) 
	  chip_slp(&msgs[0]);
	  	
	i2c_imx_slp[msgs[0].addr-1].jif = jiffies;
  // blaschke ende
	
	/* reset the i2c-bus (hopefully not needed */
/*	i2c_imx_reg->i2cr = (u32)0x00;
	GIUS(0) |=  0x00018000;
	udelay(100);
  	imx21_gpio_mode(PA15_PF_I2C_SDA);
	mdelay(1);

	i2c_imx_slp[msgs[0].addr-1].jif = jiffies;
*/
	return -err;
}

/*-----------------------------------------------------------------------------
 * int i2c_smbus_imx_xfer(struct i2c_adapter *i2c_adap, struct i2c_msg msgs[ ],
 * SINT16 num)
 * This function is responsible for the transfer of the data through the
 * I2C-bus
 * 
 * Parameter : i2c_adap	the structure associated with the related hardware
 *             msgs[ ]	the body of the message to be send out
 *             num	number of message
 * Return :    Success  Number of message has been transferred
 *	       Failure  -1	
 *----------------------------------------------------------------------------*/
static int i2c_imx_smbus(struct i2c_adapter * adapter, u16 addr,
			 unsigned short flags, char read_write, u8 command,
			 int size,  union i2c_smbus_data * data)
{
	char msgbuf0[34];
	char msgbuf1[34];
	int num = read_write == I2C_SMBUS_READ?2:1;
	struct i2c_msg msg[2] = {
		{ addr, flags, 1, msgbuf0 }, 
		{ addr, flags | I2C_M_RD, 10, msgbuf1 }
	};
	int i;
	msgbuf0[0] = command;

	/* select the smbus-command and create the message */
	switch(size) {
	case I2C_SMBUS_QUICK:
		msg[0].len = 0;
		msg[0].flags=flags|(read_write==I2C_SMBUS_READ)?I2C_M_RD:0;
		num = 1;
		break;
	case I2C_SMBUS_BYTE:
		if (read_write == I2C_SMBUS_READ) {
			msg[0].flags = I2C_M_RD | flags;
			num = 1;
		}
		break;
	case I2C_SMBUS_BYTE_DATA:
		if (read_write == I2C_SMBUS_READ)
			msg[1].len = 1;
		else {
			msg[0].len = 2;
			msgbuf0[1] = data->byte;
		}
		break;
	case I2C_SMBUS_WORD_DATA:
		if (read_write == I2C_SMBUS_READ)
			msg[1].len = 2;
		else {
			msg[0].len=3;
			msgbuf0[1] = data->word & 0xff;
			msgbuf0[2] = (data->word >> 8) & 0xff;
		}
		break;
	case I2C_SMBUS_PROC_CALL:
		num = 2;
		msg[0].len = 3;
		msg[1].len = 2;
		msgbuf0[1] = data->word & 0xff;
		msgbuf0[2] = (data->word >> 8) & 0xff;
		break;
	case I2C_SMBUS_BLOCK_DATA:
		if (read_write == I2C_SMBUS_READ) {
			return -1;
		} else {
			msg[0].len = data->block[0] + 2;
			if (msg[0].len > 34) {
				return -1;
			}
			for (i = 1; i <= msg[0].len; i++)
				msgbuf0[i] = data->block[i-1];
		}
		break;
	default:
		return -1;
	}

	/* call the bus access routine */
	if (i2c_imx_xfer(adapter, msg, num) < 0) return -1;

	/* copy the readed bytes to the data-field */
	if (read_write == I2C_SMBUS_READ)
		switch(size) {
		case I2C_SMBUS_BYTE:
			data->byte = msgbuf0[0];
			break;
		case I2C_SMBUS_BYTE_DATA:
			data->byte = msgbuf1[0];
			break;
		case I2C_SMBUS_WORD_DATA: 
		case I2C_SMBUS_PROC_CALL:
			data->word = msgbuf1[0] | (msgbuf1[1] << 8);
			break;
		}

	return 0;
}

/*-----------------------------------------------------------------------------
 * int i2c_imx_ioctl(struct i2c_adapter * adapter, unsigned int cmd,
 * unsigned long arg)
 * This function control the I2C module itself
 *
 * Parameters:	Adapter		the adapter associated to the I2C module
 * 	        Cmd		IO control command
 * 	        Arg		argument associated with the command
 * Return : 	Success		0
 *----------------------------------------------------------------------------*/
static int i2c_imx_ioctl(struct i2c_adapter * adapter, unsigned int cmd,
			 unsigned long arg)
{
	switch( cmd ) {
	case I2C_IO_CHANGE_FREQ:
		i2c_imx_reg->ifdr = (u32)(arg & 0x003f);
		break;
	case I2C_IO_GET_STATUS:
		arg = i2c_imx_reg->i2sr;
 		break;
	}

	return 0;
}

/*-----------------------------------------------------------------------------
 * int __init i2c_imx_init(void)
 * initializes the I2C module in the DBIMX, and registers itself to the 
 * Linux I2C system
 *
 * Parameters: None
 * Return:      0: indicates SUCCESS
 *             -1: indicates FAILURE
 *----------------------------------------------------------------------------*/
static int __init i2c_imx_init(void)
{
	/* Pin Configuration for I2C:
	 * 2 Pins are available for the I2C-Module. These Pins are multiplexed
	 * with other functions on the device and must be configured for SPI-
	 * Operation.
	 * The Data Direction Bits in the GPIO Data Direction Register Port A
	 * must be set for Output.
	 * The Function Bits in the GPIO In Use Register Port A must be set
	 * for Multiplexed.
	 * Data Direction (DDIR): Output
	 * GPIO-Function (GIUS):  SPI-Operation
	 * GPR, PUEN:             for Interrupt operations */
  	imx21_gpio_mode(PD17_PF_I2C_SDA);
  	imx21_gpio_mode(PD18_PF_I2C_SCL);

	/* install the I2C_IMX ISR to the Linux Kernel */
	if(request_irq(INT_I2C, (void *)i2c_imx_isr, SA_INTERRUPT,
		       "I2C_IMX", "i2c_bus")<0) return -1;

	CRM_PCCR0 |= PCCR0_I2C_EN;
  
	/* Set clock Freq. */
	i2c_imx_reg->ifdr = (u32)DEFAULT_FREQ;
  
	/* add the I2C adapter/algorithm driver to the linux kernel */
	if (i2c_add_adapter(&i2c_imx_adapter)) return -1;
#ifdef DEBUG
	printk("I2C-Adapter %d installed. use device with Minor %d\n",
	       i2c_imx_adapter.nr, i2c_imx_adapter.nr);
#endif

	/* define ms to sleep after read from chip*/
	i2c_imx_slp[0x48-1].jif = jiffies;
	i2c_imx_slp[0x48-1].slp_time = 1;
	i2c_imx_slp[0x68-1].jif = jiffies;
	i2c_imx_slp[0x68-1].slp_time = 200;

	return 0;
}

/*-----------------------------------------------------------------------------
 * void __exit i2c_imx_cleanup(void)
 * This routine is called when the driver is unloaded
 *
 * Parameters: None
 * Return:     None
 *----------------------------------------------------------------------------*/
static void __exit i2c_imx_cleanup(void)
{
#ifdef DEBUG
	printk("RX ACK      ERRORs: %d\n", i2c_imx_errs_rxack);
	printk("TX COMPLETE ERRORs: %d\n", i2c_imx_errs_txcomplete);
	printk("BUS BUSY    ERRORs: %d\n", i2c_imx_errs_busbusy);
	printk("BUS GRAB    ERRORs: %d\n", i2c_imx_errs_busgrab);
#endif

	/* unset IEN[I2CR:7] (I2C Disable) */
	i2c_imx_reg->i2cr = (u32)0x00;
  
	/* Free IRQ */
	free_irq (INT_I2C, "i2c_bus");
  
	/* Delete Adapter from Kernel */  
	i2c_del_adapter(&i2c_imx_adapter);	
}

/******************************************************************************
 * Module Init/Exit
 ******************************************************************************/
MODULE_AUTHOR("GSG China");
MODULE_DESCRIPTION("I2C Adapter/Algorithm driver");
MODULE_LICENSE("GPL");

module_init(i2c_imx_init);
module_exit(i2c_imx_cleanup);
