/*
 *  linux/drivers/mmc/imxmmc.c - Motorola i.MX MMCI driver
 *
 *  Copyright (C) 2004 Sascha Hauer, Pengutronix <sascha@saschahauer.de>
 *
 *  derived from pxamci.c by Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/blkdev.h>
#include <linux/dma-mapping.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/protocol.h>

#include <asm/dma.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/sizes.h>
#include <asm/arch/mmc.h>
#include <asm/arch/imx-dma.h>

#include "imxmmc.h"

#define DRIVER_NAME "IMXMMC"

#ifdef CONFIG_MMC_DEBUG
#define DBG(x...)	printk(KERN_DEBUG x)
#else
#define DBG(x...)	do { } while (0)
#endif

struct imxmci_host {
	struct mmc_host		*mmc;
	spinlock_t		lock;
	struct resource		*res;
	int			irq;
	imx_dmach_t		dma;
	unsigned int		clkrt;
	unsigned int		cmdat;
	unsigned int		imask;
	unsigned int		power_mode;
	unsigned int		present;
	struct imxmmc_platform_data *pdata;

	struct mmc_request	*req;
	struct mmc_command	*cmd;
	struct mmc_data		*data;

	struct timer_list	timer;

	unsigned int		dma_nents;
	unsigned int		dma_size;
	unsigned int		dma_dir;
	int			dma_allocated;

	unsigned char		actual_bus_width;
};

static void imxmci_stop_clock(struct imxmci_host *host)
{
	int i;
	while(1) {
	        MMC_STR_STP_CLK |= STR_STP_CLK_STOP_CLK;
		i = 0;
		while( (MMC_STATUS & STATUS_CARD_BUS_CLK_RUN) && (i++ < 100));

		if ( !(MMC_STATUS & STATUS_CARD_BUS_CLK_RUN) )
			break;
	}
}

static void imxmci_start_clock(struct imxmci_host *host)
{
	int i;
	while(1) {
	        MMC_STR_STP_CLK |= STR_STP_CLK_START_CLK;
		i = 0;
		while( !(MMC_STATUS & STATUS_CARD_BUS_CLK_RUN) && (i++ < 100));

		if ( MMC_STATUS & STATUS_CARD_BUS_CLK_RUN )
			break;
	}
}

static void imxmci_softreset(void)
{
	/* reset sequence */
	MMC_STR_STP_CLK = 0x8;
	MMC_STR_STP_CLK = 0xD;
	MMC_STR_STP_CLK = 0x5;
	MMC_STR_STP_CLK = 0x5;
	MMC_STR_STP_CLK = 0x5;
	MMC_STR_STP_CLK = 0x5;
	MMC_STR_STP_CLK = 0x5;
	MMC_STR_STP_CLK = 0x5;
	MMC_STR_STP_CLK = 0x5;
	MMC_STR_STP_CLK = 0x5;

	MMC_RES_TO = 0xff;
	MMC_BLK_LEN = 512;
	MMC_NOB = 1;
}

static void imxmci_setup_data(struct imxmci_host *host, struct mmc_data *data)
{
	unsigned int nob = data->blocks;
	int i;

	if (data->flags & MMC_DATA_STREAM)
		nob = 0xffff;

	host->data = data;

	MMC_NOB = nob;
	MMC_BLK_LEN = 1 << data->blksz_bits;

	if (data->flags & MMC_DATA_READ) {
		host->dma_dir = DMA_FROM_DEVICE;
		host->dma_nents = dma_map_sg(mmc_dev(host->mmc), data->sg,
						data->sg_len,  host->dma_dir);

		imx_dma_setup_sg2dev(host->dma, data->sg, data->sg_len,
			host->res->start + MMC_BUFFER_ACCESS_OFS, DMA_MODE_READ);

		imx_dma_setup_mem2dev_ccr(host->dma, DMA_MODE_READ, IMX_DMA_WIDTH_16, CCR_REN);
	} else {
		host->dma_dir = DMA_TO_DEVICE;

		host->dma_nents = dma_map_sg(mmc_dev(host->mmc), data->sg,
						data->sg_len,  host->dma_dir);

		imx_dma_setup_sg2dev(host->dma, data->sg, data->sg_len,
			host->res->start + MMC_BUFFER_ACCESS_OFS, DMA_MODE_WRITE);

		imx_dma_setup_mem2dev_ccr(host->dma, DMA_MODE_WRITE, IMX_DMA_WIDTH_16, CCR_REN);
	}

	host->dma_size=0;
	for(i=0; i<host->dma_nents; i++)
		host->dma_size+=data->sg[i].length;

	wmb();

	BLR(imx_dma_c2n(host->dma)) = 0x08;

	RSSR(imx_dma_c2n(host->dma)) = DMA_REQ_SDHC;

	/* finally start DMA engine */
	imx_dma_enable(host->dma);
}

static void imxmci_start_cmd(struct imxmci_host *host, struct mmc_command *cmd, unsigned int cmdat)
{
	WARN_ON(host->cmd != NULL);
	host->cmd = cmd;

	if (cmd->flags & MMC_RSP_BUSY)
		cmdat |= CMD_DAT_CONT_BUSY;

	switch (cmd->flags & (MMC_RSP_MASK | MMC_RSP_CRC)) {
	case MMC_RSP_SHORT | MMC_RSP_CRC:
		cmdat |= CMD_DAT_CONT_RESPONSE_FORMAT_R1;
		break;
	case MMC_RSP_SHORT:
		cmdat |= CMD_DAT_CONT_RESPONSE_FORMAT_R3;
		break;
	case MMC_RSP_LONG | MMC_RSP_CRC:
		cmdat |= CMD_DAT_CONT_RESPONSE_FORMAT_R2;
		break;
	default:
		break;
	}

	if ( cmd->opcode == MMC_GO_IDLE_STATE )
		cmdat |= CMD_DAT_CONT_INIT; /* This command needs init */

	if ( host->actual_bus_width == MMC_BUS_WIDTH_4 )
		cmdat |= CMD_DAT_CONT_BUS_WIDTH_4;

	MMC_CMD = cmd->opcode;
	MMC_ARGH = cmd->arg >> 16;
	MMC_ARGL = cmd->arg & 0xffff;
	MMC_CMD_DAT_CONT = cmdat;

	imxmci_start_clock(host);
}

static void imxmci_finish_request(struct imxmci_host *host, struct mmc_request *req)
{
	host->req = NULL;
	host->cmd = NULL;
	host->data = NULL;
	mmc_request_done(host->mmc, req);
}

static int imxmci_cmd_done(struct imxmci_host *host, unsigned int stat)
{
	struct mmc_command *cmd = host->cmd;
	int i;
	u32 a,b,c;

	if (!cmd)
		return 0;

	host->cmd = NULL;

	if (stat & STATUS_TIME_OUT_RESP) {
		DBG("%s: CMD TIMEOUT\n",DRIVER_NAME);
		cmd->error = MMC_ERR_TIMEOUT;
	} else if (stat & STATUS_RESP_CRC_ERR && cmd->flags & MMC_RSP_CRC) {
		DBG("%s: cmd crc error\n",DRIVER_NAME);
		cmd->error = MMC_ERR_BADCRC;
	}

	switch (cmd->flags & (MMC_RSP_MASK | MMC_RSP_CRC)) {
	case MMC_RSP_SHORT | MMC_RSP_CRC:
		a = MMC_RES_FIFO & 0xffff;
		b = MMC_RES_FIFO & 0xffff;
		c = MMC_RES_FIFO & 0xffff;
		cmd->resp[0] = a<<24 | b<<8 | c>>8;
		break;
	case MMC_RSP_SHORT:
		a = MMC_RES_FIFO & 0xffff;
		b = MMC_RES_FIFO & 0xffff;
		c = MMC_RES_FIFO & 0xffff;
		cmd->resp[0] = a<<24 | b<<8 | c>>8;
		break;
	case MMC_RSP_LONG | MMC_RSP_CRC:
		for (i = 0; i < 4; i++) {
			u32 a = MMC_RES_FIFO & 0xffff;
			u32 b = MMC_RES_FIFO & 0xffff;
			cmd->resp[i] = a<<16 | b;
		}
		break;
	default:
		break;
	}

	DBG("%s: RESP 0x%08x, 0x%08x, 0x%08x, 0x%08x, error %d\n",DRIVER_NAME,cmd->resp[0],
	        cmd->resp[1],cmd->resp[2],cmd->resp[3], cmd->error);


	if (host->data && (cmd->error == MMC_ERR_NONE)) {
		/* nothing */
	} else {
		struct mmc_request *req;
		imxmci_stop_clock(host);
		req = host->req;
		if( req ) {
			imxmci_finish_request(host, req);
		} else {
			printk(KERN_WARNING "%s: imxmci_cmd_done: no request to finish\n", DRIVER_NAME);
		}
	}

	return 1;
}

static int imxmci_data_done(struct imxmci_host *host, unsigned int stat)
{
	struct mmc_data *data = host->data;

	imx_dma_disable(host->dma);

	if (!data)
		return 0;

	dma_unmap_sg(mmc_dev(host->mmc), data->sg, host->dma_nents,
		     host->dma_dir);

	if ( MMC_STATUS & STATUS_ERR_MASK ) {
		DBG("%s: request failed. status: 0x%08x\n",DRIVER_NAME,MMC_STATUS);
	}

	host->data = NULL;
	data->bytes_xfered = host->dma_size;

	if (host->req->stop && data->error == MMC_ERR_NONE) {
		imxmci_stop_clock(host);
		imxmci_start_cmd(host, host->req->stop, 0);
	} else {
		struct mmc_request *req;
		req = host->req;
		if( req ) {
			imxmci_finish_request(host, req);
		} else {
			printk(KERN_WARNING "%s: imxmci_data_done: no request to finish\n", DRIVER_NAME);
		}
	}

	return 1;
}

static irqreturn_t imxmci_irq(int irq, void *devid, struct pt_regs *regs)
{
	struct imxmci_host *host = devid;
	unsigned int status = 0;
	int handled = 0,timeout = 10000;
	unsigned long flags;


	spin_lock_irqsave(host->lock, flags);

	status = MMC_STATUS;

	DBG("%s: IRQ initial status %08x\n", DRIVER_NAME, status);

	while( !handled && --timeout ) {
		if(status & STATUS_END_CMD_RESP) {
			imxmci_cmd_done(host, status);
			handled = 1;
		}
		status = MMC_STATUS;
	}

	DBG("%s: IRQ final status %08x\n", DRIVER_NAME, status);

	if(!timeout) {
		printk("%s: fake interrupt\n",DRIVER_NAME);
		handled = 1;
	}

	MMC_INT_MASK = host->imask;

	spin_unlock_irqrestore(host->lock, flags);

	return IRQ_RETVAL(handled);
}

static void imxmci_request(struct mmc_host *mmc, struct mmc_request *req)
{
	struct imxmci_host *host = mmc_priv(mmc);
	unsigned int cmdat;

	WARN_ON(host->req != NULL);

	host->req = req;

	cmdat = 0;

	if (req->data) {
		imxmci_setup_data(host, req->data);

		cmdat |= CMD_DAT_CONT_DATA_ENABLE;

		if (req->data->flags & MMC_DATA_WRITE)
			cmdat |= CMD_DAT_CONT_WRITE;

		if (req->data->flags & MMC_DATA_STREAM) {
			cmdat |= CMD_DAT_CONT_STREAM_BLOCK;
		}
	}

	imxmci_start_cmd(host, req->cmd, cmdat);
}

#define CLK_RATE 19200000

static void imxmci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct imxmci_host *host = mmc_priv(mmc);
	int prescaler;

	DBG("%s: clock %u power %u vdd %u width %u\n", DRIVER_NAME,
	    ios->clock, ios->power_mode, ios->vdd,
	    (ios->bus_width==MMC_BUS_WIDTH_4)?4:1);

	if( ios->bus_width==MMC_BUS_WIDTH_4 ) {
		host->actual_bus_width = MMC_BUS_WIDTH_4;
		imx_gpio_mode(PB11_PF_SD_DAT3);
	}else{
		host->actual_bus_width = MMC_BUS_WIDTH_1;
		imx_gpio_mode(GPIO_PORTB | GPIO_IN | GPIO_PUEN | GPIO_GPIO | 11);
	}

	if ( ios->clock ) {
		unsigned int clk;

		/* The prescaller is 5 for PERCLK2 equal to 96MHz
		 * then 96MHz / 5 = 19.2 MHz
		 */
		clk=imx_get_perclk2();
		prescaler=(clk+(CLK_RATE*7)/8)/CLK_RATE;
		switch(prescaler) {
		case 0:
		case 1:	prescaler = 0;
			break;
		case 2:	prescaler = 1;
			break;
		case 3:	prescaler = 2;
			break;
		case 4:	prescaler = 4;
			break;
		default:
		case 5:	prescaler = 5;
			break;
		}

		DBG("%s:PERCLK2 %d MHz -> prescaler %d\n",
			DRIVER_NAME, clk, prescaler);

		for(clk=0; clk<8; clk++) {
			int x;
			x = CLK_RATE / (1<<clk);
			if( x <= ios->clock)
				break;
		}

		MMC_STR_STP_CLK |= STR_STP_CLK_ENABLE; /* enable controller */

		imxmci_stop_clock(host);
		MMC_CLK_RATE = (prescaler<<3) | clk;
		imxmci_start_clock(host);

		DBG("%s:MMC_CLK_RATE: 0x%08x\n",DRIVER_NAME,MMC_CLK_RATE);
	} else {
		imxmci_stop_clock(host);
	}

        if (host->power_mode != ios->power_mode) {
                host->power_mode = ios->power_mode;

                if (host->pdata && host->pdata->setpower)
                        host->pdata->setpower(mmc->dev, ios->vdd);

        }

}

static struct mmc_host_ops imxmci_ops = {
	.request	= imxmci_request,
	.set_ios	= imxmci_set_ios,
};

static struct resource *platform_device_resource(struct platform_device *dev, unsigned int mask, int nr)
{
	int i;

	for (i = 0; i < dev->num_resources; i++)
		if (dev->resource[i].flags == mask && nr-- == 0)
			return &dev->resource[i];
	return NULL;
}

static int platform_device_irq(struct platform_device *dev, int nr)
{
	int i;

	for (i = 0; i < dev->num_resources; i++)
		if (dev->resource[i].flags == IORESOURCE_IRQ && nr-- == 0)
			return dev->resource[i].start;
	return NO_IRQ;
}

static void imxmci_dma_irq(int dma, void *devid, struct pt_regs *regs)
{
	struct imxmci_host *host = devid;
	u32 status;
	long loopcnt=0;
	unsigned long flags;


	do {
		status = MMC_STATUS;
		if(host->dma_dir == DMA_TO_DEVICE) {
			if(status & STATUS_WRITE_OP_DONE)
				break;
		}else{
			if(status & STATUS_DATA_TRANS_DONE)
				break;
		}
	} while(loopcnt++ < 0x100000);

	if(loopcnt>0x10000) {
		printk(KERN_ERR "%s: %s at %s (%d) long busy loop count %ld\n", DRIVER_NAME,
			__FUNCTION__, __FILE__, __LINE__, loopcnt);
                /*dump_stack();*/
		__backtrace();
	}else if(loopcnt){
		DBG("%s: imxmci_dma_irq busy wait %ld loops\n",
			DRIVER_NAME,loopcnt);
	}

	spin_lock_irqsave(host->lock, flags);

	imxmci_data_done(host, status);
	MMC_INT_MASK = host->imask;

	spin_unlock_irqrestore(host->lock, flags);
}

static void imxmci_check_status(unsigned long data)
{
	struct imxmci_host *host = (struct imxmci_host *)data;

	if( host->pdata->card_present() != host->present ) {
		host->present ^= 1;
		printk(KERN_INFO "%s: card %s\n",DRIVER_NAME,
		      host->present ? "inserted" : "removed");
		mmc_detect_change(host->mmc);
	}
	mod_timer(&host->timer, jiffies + (HZ>>1));
}

static int imxmci_probe(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mmc_host *mmc;
	struct imxmci_host *host = NULL;
	struct resource *r;
	int ret = 0, irq;

	printk(KERN_INFO "i.MX mmc driver\n");

	r = platform_device_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_device_irq(pdev, 0);
	if (!r || irq == NO_IRQ)
		return -ENXIO;

	r = request_mem_region(r->start, 0x100, "IMXMCI");
	if (!r)
		return -EBUSY;

	mmc = mmc_alloc_host(sizeof(struct imxmci_host), dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto out;
	}

	mmc->ops = &imxmci_ops;
	mmc->f_min = 150000;
	mmc->f_max = CLK_RATE/2;
	mmc->ocr_avail = MMC_VDD_32_33;
	mmc->caps |= MMC_CAP_4_BIT_DATA;

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->dma_allocated = 0;
	host->pdata = pdev->dev.platform_data;

	spin_lock_init(&host->lock);
	host->res = r;
	host->irq = irq;

	imx_gpio_mode(PB8_PF_SD_DAT0);
	imx_gpio_mode(PB9_PF_SD_DAT1);
	imx_gpio_mode(PB10_PF_SD_DAT2);
	/* Configured as GPIO with pull-up to ensure right MCC card mode */
	/* Swithed to PB11_PF_SD_DAT3 if 4 bit bus is configured */
	imx_gpio_mode(GPIO_PORTB | GPIO_IN | GPIO_PUEN | GPIO_GPIO | 11);
	imx_gpio_mode(PB12_PF_SD_CLK);
	imx_gpio_mode(PB13_PF_SD_CMD);

	imxmci_softreset();

	if ( MMC_REV_NO != 0x390 ) {
		printk(KERN_ERR "%s: wrong rev.no. 0x%08x. aborting.\n",
		                                    DRIVER_NAME,MMC_REV_NO);
		goto out;
	}

	MMC_READ_TO = 0x2db4; /* recommended in data sheet */

	host->imask = INT_MASK_BUF_READY | INT_MASK_DATA_TRAN |
	              INT_MASK_WRITE_OP_DONE | INT_MASK_SDIO |
		      INT_MASK_AUTO_CARD_DETECT;
	MMC_INT_MASK = host->imask;


	if(imx_dma_request_by_prio(&host->dma, DRIVER_NAME, DMA_PRIO_LOW)<0){
		printk(KERN_ERR "%s: imx_dma_request_by_prio failed\n", DRIVER_NAME);
		ret = -EBUSY;
		goto out;
	}
	host->dma_allocated=1;
	imx_dma_setup_handlers(host->dma, imxmci_dma_irq, NULL, host);

	ret = request_irq(host->irq, imxmci_irq, 0, DRIVER_NAME, host);
	if (ret)
		goto out;

	host->present = host->pdata->card_present();
	init_timer(&host->timer);
	host->timer.data = (unsigned long)host;
	host->timer.function = imxmci_check_status;
	add_timer(&host->timer);
	mod_timer(&host->timer, jiffies + (HZ>>1));

	dev_set_drvdata(dev, mmc);

	mmc_add_host(mmc);

	return 0;

out:
	if (host) {
		if(host->dma_allocated){
			imx_dma_free(host->dma);
			host->dma_allocated=0;
		}
	}
	if (mmc)
		mmc_free_host(mmc);
	release_resource(r);
	return ret;
}

static int imxmci_remove(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);

	dev_set_drvdata(dev, NULL);

	if (mmc) {
		struct imxmci_host *host = mmc_priv(mmc);

		del_timer_sync(&host->timer);
		mmc_remove_host(mmc);

		free_irq(host->irq, host);
		if(host->dma_allocated){
			imx_dma_free(host->dma);
			host->dma_allocated=0;
		}


		release_resource(host->res);

		mmc_free_host(mmc);
	}
	return 0;
}

#ifdef CONFIG_PM
static int imxmci_suspend(struct device *dev, u32 state, u32 level)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	int ret = 0;

	if (mmc && level == SUSPEND_DISABLE)
		ret = mmc_suspend_host(mmc, state);

	return ret;
}

static int imxmci_resume(struct device *dev, u32 level)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	int ret = 0;

	if (mmc && level == RESUME_ENABLE)
		ret = mmc_resume_host(mmc);

	return ret;
}
#else
#define imxmci_suspend  NULL
#define imxmci_resume   NULL
#endif /* CONFIG_PM */

static struct device_driver imxmci_driver = {
	.name		= "imx-mmc",
	.bus		= &platform_bus_type,
	.probe		= imxmci_probe,
	.remove		= imxmci_remove,
	.suspend	= imxmci_suspend,
	.resume		= imxmci_resume,
};

static int __init imxmci_init(void)
{
	return driver_register(&imxmci_driver);
}

static void __exit imxmci_exit(void)
{
	driver_unregister(&imxmci_driver);
}

module_init(imxmci_init);
module_exit(imxmci_exit);

MODULE_DESCRIPTION("i.MX Multimedia Card Interface Driver");
MODULE_AUTHOR("Sascha Hauer, Pengutronix");
MODULE_LICENSE("GPL");
