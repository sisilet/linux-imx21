/*
 *  linux/arch/arm/mach-imx/dma.c
 *
 *  imx DMA registration and IRQ dispatching
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  2004-03-03 Sascha Hauer <sascha@saschahauer.de>
 *             initial version heavily inspired by
 *             linux/arch/arm/mach-pxa/dma.c
 *
 *  2005-04-17 Pavel Pisa <pisa@cmp.felk.cvut.cz>
 *             Changed to support scatter gather DMA
 *             by taking Russell's code from RiscPC
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/errno.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/dma.h>
#include <asm/arch/imx-dma.h>

#if 1
#define DEBUG_IMXDMA(fmt...)       printk(fmt)
#else
#define DEBUG_IMXDMA(fmt...)       do { } while (0)
#endif

struct imx_dma_channel imx_dma_channels[IMX_DMA_CHANNELS];

/*
 * imx_dma_sg_next - scatter-gather DMA emulation
 * @imxdma: pointer to i.MX DMA extension structure
 * @lastcount: number of bytes transferred during last transfer
 *
 * Functions prepares DMA controller for next sg data chunk transfer.
 * The last count informs function about number of bytes transferred @lastcount
 * during last block. Zero value can be used for @lastcount to setup DMA
 * for the first chunk.
 */

static inline int
imx_dma_sg_next(imx_dmach_t dma_ch, unsigned int lastcount)
{
	struct imx_dma_channel *imxdma=imx_dma_c2i(dma_ch);
	unsigned int nextcount;
	unsigned int nextaddr;

	if (!imxdma->name) {
		printk(KERN_CRIT "%s: called for  not allocated channel %d\n",
		       __FUNCTION__, imx_dma_c2n(dma_ch));
		return 0;
	}

	if(!imxdma->sg){
		DEBUG_IMXDMA(KERN_DEBUG "imxdma%d: no sg data\n",imx_dma_c2n(dma_ch));
		return 0;
	}

	imxdma->sgbc+=lastcount;
	if(imxdma->sgbc>=imxdma->sg->length){
		if(imxdma->sgcount>1){
			imxdma->sgcount--;
			imxdma->sgbc=0;
		}else{
			DEBUG_IMXDMA(KERN_DEBUG "imxdma%d: sg src limit reached\n",imx_dma_c2n(dma_ch));
			imxdma->sg=NULL;
			return 0;
		}
	}
	nextcount=imxdma->sg->length-imxdma->sgbc;
	nextaddr=imxdma->sg->dma_address+imxdma->sgbc;

	if((imxdma->dma_mode&DMA_MODE_MASK)==DMA_MODE_READ)
		DAR(imx_dma_c2n(dma_ch))=nextaddr;
	else
		SAR(imx_dma_c2n(dma_ch))=nextaddr;

	CNTR(imx_dma_c2n(dma_ch))=nextcount;
	DEBUG_IMXDMA(KERN_DEBUG "imxdma%d: next sg chunk dst 0x%08x, src 0x%08x, size 0x%08x\n",
				imx_dma_c2n(dma_ch),DAR(imx_dma_c2n(dma_ch)),SAR(imx_dma_c2n(dma_ch)),
				CNTR(imx_dma_c2n(dma_ch)));

	return nextcount;
}


static int
imx_dma_setup_sg_base(imx_dmach_t dma_ch,
		 struct scatterlist *sg, unsigned int sgcount)
{
	struct imx_dma_channel *imxdma=imx_dma_c2i(dma_ch);

	imxdma->sg=sg;
	imxdma->sgcount=sgcount;
	imxdma->sgbc=0;
	return imx_dma_sg_next(dma_ch, 0);
}

unsigned int
imx_dma_setup_mem2dev_ccr(imx_dmach_t dma_ch, dmamode_t dmamode,
		int devwidth, unsigned int imx_mode)
{
	unsigned int ccr=imx_mode;

	if((dmamode&DMA_MODE_MASK)==DMA_MODE_READ){
		ccr |= CCR_SMOD_LINEAR;
		ccr |= CCR_DSIZ_32;
		ccr |= CCR_SMOD_FIFO;
		switch(devwidth){
			case IMX_DMA_WIDTH_8:
				ccr |= CCR_SSIZ_8;
				break;
			case IMX_DMA_WIDTH_16:
				ccr |= CCR_SSIZ_16;
				break;
			case IMX_DMA_WIDTH_32:
				ccr |= CCR_SSIZ_32;
				break;
		}
	}else if((dmamode&DMA_MODE_MASK)==DMA_MODE_WRITE){
		ccr |= CCR_SMOD_LINEAR;
		ccr |= CCR_SSIZ_32;
		ccr |= CCR_DMOD_FIFO;
		switch(devwidth){
			case IMX_DMA_WIDTH_8:
				ccr |= CCR_DSIZ_8;
				break;
			case IMX_DMA_WIDTH_16:
				ccr |= CCR_DSIZ_16;
				break;
			case IMX_DMA_WIDTH_32:
				ccr |= CCR_DSIZ_32;
				break;
		}
	}
	CCR(imx_dma_c2n(dma_ch)) = ccr;

	DEBUG_IMXDMA(KERN_DEBUG "imxdma%d: CCR set to 0x%08x\n",
			imx_dma_c2n(dma_ch),ccr);

	return ccr;
}


int
imx_dma_setup_single2dev(imx_dmach_t dma_ch, dma_addr_t dma_address,
		unsigned int dma_length, unsigned int dev_addr, dmamode_t dmamode)
{
	struct imx_dma_channel *imxdma=imx_dma_c2i(dma_ch);

	imxdma->sg=NULL;
	imxdma->sgcount=0;

	if(!dma_address){
		printk(KERN_ERR "imxdma%d: imx_dma_setup_single2dev null address\n",imx_dma_c2n(dma_ch));
		return -EINVAL;
	}

	if(!dma_length){
		printk(KERN_ERR "imxdma%d: imx_dma_setup_single2dev zero length\n",imx_dma_c2n(dma_ch));
		return -EINVAL;
	}

	if((dmamode&DMA_MODE_MASK)==DMA_MODE_READ){
		DEBUG_IMXDMA(KERN_DEBUG "imxdma%d: mx_dma_setup_single2dev dma_addressg=0x%08x dma_length=%d dev_addr=0x%08x for read\n",
				imx_dma_c2n(dma_ch), (unsigned int)dma_address, dma_length, dev_addr);
		SAR(imx_dma_c2n(dma_ch))=dev_addr;
		DAR(imx_dma_c2n(dma_ch))=(unsigned int)dma_address;
	}else if((dmamode&DMA_MODE_MASK)==DMA_MODE_WRITE){
		DEBUG_IMXDMA(KERN_DEBUG "imxdma%d: mx_dma_setup_single2dev dma_addressg=0x%08x dma_length=%d dev_addr=0x%08x for write\n",
				imx_dma_c2n(dma_ch), (unsigned int)dma_address, dma_length, dev_addr);
		SAR(imx_dma_c2n(dma_ch))=(unsigned int)dma_address;
		DAR(imx_dma_c2n(dma_ch))=dev_addr;
	}else{
		printk(KERN_ERR "imxdma%d: imx_dma_setup_single2dev bad dmamode\n",imx_dma_c2n(dma_ch));
		return -EINVAL;
	}

	CNTR(imx_dma_c2n(dma_ch))=dma_length;

	return 0;
}


int
imx_dma_setup_sg2dev(imx_dmach_t dma_ch,
		 struct scatterlist *sg, unsigned int sgcount,
		 unsigned int dev_addr, dmamode_t dmamode)
{
	int res;
	struct imx_dma_channel *imxdma=imx_dma_c2i(dma_ch);

	imxdma->sg=NULL;
	imxdma->sgcount=0;

	if(!sg || !sgcount){
		printk(KERN_ERR "imxdma%d: imx_dma_setup_sg2dev epty sg list\n",imx_dma_c2n(dma_ch));
		return -EINVAL;
	}

	if(!sg->length){
		printk(KERN_ERR "imxdma%d: imx_dma_setup_sg2dev zero length\n",imx_dma_c2n(dma_ch));
		return -EINVAL;
	}

	if((dmamode&DMA_MODE_MASK)==DMA_MODE_READ){
		DEBUG_IMXDMA(KERN_DEBUG "imxdma%d: mx_dma_setup_sg2dev sg=%p sgcount=%d dev_addr=0x%08x for read\n",
				imx_dma_c2n(dma_ch), sg, sgcount, dev_addr);
		SAR(imx_dma_c2n(dma_ch))=dev_addr;
	}else if((dmamode&DMA_MODE_MASK)==DMA_MODE_WRITE){
		DEBUG_IMXDMA(KERN_DEBUG "imxdma%d: mx_dma_setup_sg2dev sg=%p sgcount=%d dev_addr=0x%08x for write\n",
				imx_dma_c2n(dma_ch), sg, sgcount, dev_addr);
		DAR(imx_dma_c2n(dma_ch))=dev_addr;
	}else{
		printk(KERN_ERR "imxdma%d: imx_dma_setup_sg2dev bad dmamode\n",imx_dma_c2n(dma_ch));
		return -EINVAL;
	}

	res=imx_dma_setup_sg_base(dma_ch, sg, sgcount);
	if(res<=0){
		printk(KERN_ERR "imxdma%d: no sg chunk ready\n",imx_dma_c2n(dma_ch));
		return -EINVAL;
	}

	return 0;
}

/* set err_handler to NULL to have the standard info-only error handler */
int
imx_dma_setup_handlers(imx_dmach_t dma_ch,
		void (*irq_handler) (int, void *, struct pt_regs *),
		void (*err_handler) (int, void *, struct pt_regs *), void *data)
{
	struct imx_dma_channel *imxdma=imx_dma_c2i(dma_ch);
	unsigned long flags;

	if (!imxdma->name) {
		printk(KERN_CRIT "%s: called for  not allocated channel %d\n",
		       __FUNCTION__, imx_dma_c2n(dma_ch));
		return -ENODEV;
	}

	local_irq_save(flags);
	DISR = (1 << imx_dma_c2n(dma_ch));
	imxdma->irq_handler = irq_handler;
	imxdma->err_handler = err_handler;
	imxdma->data = data;
	local_irq_restore(flags);
	return 0;
}

void
imx_dma_enable(imx_dmach_t dma_ch)
{
	struct imx_dma_channel *imxdma=imx_dma_c2i(dma_ch);
	unsigned long flags;

	DEBUG_IMXDMA(KERN_DEBUG "imxdma%d: imx_dma_enable\n",
			imx_dma_c2n(dma_ch));

	if (!imxdma->name) {
		printk(KERN_CRIT "%s: called for  not allocated channel %d\n",
		       __FUNCTION__, imx_dma_c2n(dma_ch));
		return;

	}
	local_irq_save(flags);
	DISR = (1 << imx_dma_c2n(dma_ch));
	DIMR &= ~(1 << imx_dma_c2n(dma_ch));
	CCR(imx_dma_c2n(dma_ch)) |= CCR_CEN;
	local_irq_restore(flags);
}


void
imx_dma_disable(imx_dmach_t dma_ch)
{
	unsigned long flags;

	DEBUG_IMXDMA(KERN_DEBUG "imxdma%d: imx_dma_disable\n",
			imx_dma_c2n(dma_ch));

	local_irq_save(flags);
	DIMR |= (1 << imx_dma_c2n(dma_ch));
	CCR(imx_dma_c2n(dma_ch)) &= ~CCR_CEN;
	DISR = (1 << imx_dma_c2n(dma_ch));
	local_irq_restore(flags);
}


int
imx_dma_request(imx_dmach_t dma_ch, const char *name)
{
	struct imx_dma_channel *imxdma=imx_dma_c2i(dma_ch);
	unsigned long flags;

	/* basic sanity checks */
	if (!name)
		return -EINVAL;

	if(imx_dma_c2n(dma_ch)>=IMX_DMA_CHANNELS){
		printk(KERN_CRIT "%s: called for  non-existed channel %d\n",
		       __FUNCTION__, imx_dma_c2n(dma_ch));
		return -EINVAL;
	}

	local_irq_save(flags);
	if(imxdma->name){
		local_irq_restore(flags);
		return -ENODEV;
        }

	imxdma->name = name;
	imxdma->irq_handler = NULL;
	imxdma->err_handler = NULL;
	imxdma->data = NULL;
	imxdma->sg=NULL;
	local_irq_restore(flags);
	return 0;
}

void
imx_dma_free(imx_dmach_t dma_ch)
{
	unsigned long flags;
	struct imx_dma_channel *imxdma=imx_dma_c2i(dma_ch);

	if (!imxdma->name) {
		printk(KERN_CRIT
		       "%s: trying to free channel %d which is already freed\n",
		       __FUNCTION__, imx_dma_c2n(dma_ch));
		return;
	}

	local_irq_save(flags);
	/* Disable interrupts */
	DIMR |= (1 << imx_dma_c2n(dma_ch));
	CCR(imx_dma_c2n(dma_ch)) &= ~CCR_CEN;
	imxdma->name = NULL;
	local_irq_restore(flags);
}

int
imx_dma_request_by_prio(imx_dmach_t *pdma_ch, const char *name, imx_dma_prio prio)
{
	int i;
	int best;

	switch(prio){
		case(DMA_PRIO_HIGH):
			best=8;
			break;
		case(DMA_PRIO_MEDIUM):
			best=4;
			break;
		case(DMA_PRIO_LOW):
		default:
			best=0;
			break;
	}

	for(i=best;i<IMX_DMA_CHANNELS;i++){
		if(!imx_dma_request(imx_dma_n2c(i), name)){
			*pdma_ch=imx_dma_n2c(i);
			return 0;
		}
	}

	for(i=best-1;i>=0;i--){
		if(!imx_dma_request(imx_dma_n2c(i), name)){
			*pdma_ch=imx_dma_n2c(i);
			return 0;
		}
	}

	printk(KERN_ERR "%s: no free DMA channel found\n",
	       __FUNCTION__);

	return -ENODEV;
}

static irqreturn_t
dma_err_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	int i;
        int disr = DISR;
	struct imx_dma_channel *channel;
	unsigned int err_mask = DBTOSR | DRTOSR | DSESR | DBOSR;

        DISR = disr;
	for (i = 0; i < IMX_DMA_CHANNELS; i++) {
		channel = &imx_dma_channels[i];

		if ( (err_mask & 1<<i) && channel->name && channel->err_handler) {
			channel->err_handler(i, channel->data, regs);
			continue;
		}

		imx_dma_channels[i].sg=NULL;

		if (DBTOSR & (1 << i)) {
			printk(KERN_WARNING
			       "Burst timeout on channel %d (%s)\n",
			       i, channel->name);
			DBTOSR |= (1 << i);
		}
		if (DRTOSR & (1 << i)) {
			printk(KERN_WARNING
			       "Request timeout on channel %d (%s)\n",
			       i, channel->name);
			DRTOSR |= (1 << i);
		}
		if (DSESR & (1 << i)) {
			printk(KERN_WARNING
			       "Transfer timeout on channel %d (%s)\n",
			       i, channel->name);
			DSESR |= (1 << i);
		}
		if (DBOSR & (1 << i)) {
			printk(KERN_WARNING
			       "Buffer overflow timeout on channel %d (%s)\n",
			       i, channel->name);
			DBOSR |= (1 << i);
		}
	}
	return IRQ_HANDLED;
}

static irqreturn_t
dma_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	int i, disr = DISR;

	DEBUG_IMXDMA(KERN_DEBUG "imxdma: dma_irq_handler called, disr=0x%08x\n", disr);

        DISR = disr;
	for (i = 0; i < IMX_DMA_CHANNELS; i++) {
		if (disr & (1 << i)) {
			struct imx_dma_channel *channel = &imx_dma_channels[i];
			if (channel->name) {
				if(imx_dma_sg_next(imx_dma_n2c(i), CNTR(i))){
					CCR(i) &= ~CCR_CEN;
					mb();
					CCR(i) |= CCR_CEN;
				}else{
					if(channel->irq_handler)
						channel->irq_handler(i, channel->data, regs);
				}
			} else {
				/*
				 * IRQ for an unregistered DMA channel:
				 * let's clear the interrupts and disable it.
				 */
				printk(KERN_WARNING
				       "spurious IRQ for DMA channel %d\n", i);
			}
		}
	}

	return IRQ_HANDLED;
}

/*
static struct dma_ops imx_dma_ops = {
	.type		= "IMXDMA",
	.request	= imx_dma_request,
	.free		= imx_dma_free,
	.enable		= imx_dma_enable,
	.disable	= imx_dma_disable,
	.setspeed	= imx_dma_set_speed,
};
*/


static int __init
imx_dma_init(void)
{
	int ret;
	int i;

	/* reset DMA module */
	DCR = DCR_DRST;

	ret = request_irq(DMA_INT, dma_irq_handler, 0, "DMA", NULL);
	if (ret) {
		printk(KERN_CRIT "Wow!  Can't register IRQ for DMA\n");
		return ret;
	}

	ret = request_irq(DMA_ERR, dma_err_handler, 0, "DMA", NULL);
	if (ret) {
		printk(KERN_CRIT "Wow!  Can't register ERRIRQ for DMA\n");
		free_irq(DMA_INT, NULL);
	}

	/* enable DMA module */
	DCR = DCR_DEN;

	/* clear all interrupts */
	DISR = (1<<IMX_DMA_CHANNELS)-1;

	/* enable interrupts */
	DIMR = (1<<IMX_DMA_CHANNELS)-1;

	for(i=0; i<IMX_DMA_CHANNELS; i++){
		imx_dma_channels[i].sg=NULL;
		imx_dma_channels[i].dma_num=i;
	}

	return ret;
}

arch_initcall(imx_dma_init);

EXPORT_SYMBOL(imx_dma_setup_mem2dev_ccr);
EXPORT_SYMBOL(imx_dma_setup_single2dev);
EXPORT_SYMBOL(imx_dma_setup_sg2dev);
EXPORT_SYMBOL(imx_dma_setup_handlers);
EXPORT_SYMBOL(imx_dma_enable);
EXPORT_SYMBOL(imx_dma_disable);
EXPORT_SYMBOL(imx_dma_request);
EXPORT_SYMBOL(imx_dma_free);
EXPORT_SYMBOL(imx_dma_request_by_prio);
EXPORT_SYMBOL(imx_dma_channels);
