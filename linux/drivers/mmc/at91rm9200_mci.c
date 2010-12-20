/*
 *  linux/drivers/mmc/at91rm9200_mci.c - ATMEL AT91RM9200 MCI Driver
 *
 *  Copyright (C) 2005 Cougar Creek Computing Devices Ltd, All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
   This is the AT91RM9200 MCI driver that has been tested with MMC cards only.
   The CCAT91SBC001 board does not support SD cards.

   The two entry points are at91rm9200_mci_request and at91rm9200_mci_set_ios.

   SET IOS
     This configures the device to put it into the correct mode and clock speed
     required.

   MCI REQUEST
     MCI request processes the commands sent in the mmc_request structure. This
     can consist of a processing command and a stop command in the case of
     multiple block transfers.

     There are three main types of request, commands, reads and writes.

     Commands are straight forward. The command is submitted to the controller and
     the request function returns. When the controller generates an interrupt to indicate
     the command is finished, the response to the command are read and the mmc_request_done
     function called to end the request.

     Reads and writes work in a similar manner to normal commands but involve the PDC (DMA)
     controller to manage the transfers.

     A read is done from the controller directly to the scatterlist passed in from the request.
     Due to a bug in the controller, when a read is completed, all the words are byte
     swapped in the scatterlist buffers.

     The sequence of read interrupts is: ENDRX, RXBUFF, CMDRDY

     A write is slightly different in that the bytes to write are read from the scatterlist
     into a dma memory buffer (this is in case the source buffer should be read only). The
     entire write buffer is then done from this single dma memory buffer.

     The sequence of write interrupts is: ENDTX, TXBUFE, NOTBUSY, CMDRDY
*/

#include <linux/config.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>

#include <linux/mmc/host.h>
#include <linux/mmc/protocol.h>
#include <linux/mmc/card.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach/mmc.h>
#include <asm/arch/board.h>
#include <asm/arch/AT91RM9200_MCI.h>

#define BYTE_SWAP4(x) \
  (((x & 0xFF000000) >> 24) | \
   ((x & 0x00FF0000) >> 8) | \
   ((x & 0x0000FF00) << 8)  | \
   ((x & 0x000000FF) << 24))

#define DRIVER_NAME "at91rm9200_mci"

#ifdef CONFIG_MMC_DEBUG
#define DBG(fmt...)	\
	printk(KERN_DEBUG fmt)
#else
#define DBG(fmt...)	do { } while (0)
#endif

/* MCI Controller device */
static AT91PS_MCI controller = (AT91PS_MCI)AT91C_VA_BASE_MCI;

/* The command register values for each command */
static const u32 commands[64] = {
	/* Class 1  (0) */
	MMC_GO_IDLE_STATE,
	MMC_SEND_OP_COND	| AT91C_MCI_RSPTYP_48,
	MMC_ALL_SEND_CID	| AT91C_MCI_RSPTYP_136,
	MMC_SET_RELATIVE_ADDR	| AT91C_MCI_RSPTYP_48	| AT91C_MCI_MAXLAT,
	MMC_SET_DSR		| AT91C_MCI_MAXLAT,
	0,
	SD_APP_SET_BUS_WIDTH	| AT91C_MCI_RSPTYP_48 	| AT91C_MCI_MAXLAT,
        MMC_SELECT_CARD		| AT91C_MCI_RSPTYP_48	| AT91C_MCI_MAXLAT,
	0,
	MMC_SEND_CSD		| AT91C_MCI_RSPTYP_136	| AT91C_MCI_MAXLAT,
	MMC_SEND_CID		| AT91C_MCI_RSPTYP_136	| AT91C_MCI_MAXLAT,
	MMC_READ_DAT_UNTIL_STOP	| AT91C_MCI_RSPTYP_48	| AT91C_MCI_MAXLAT | AT91C_MCI_TRDIR | AT91C_MCI_TRCMD_START | AT91C_MCI_TRTYP_STREAM,
	MMC_STOP_TRANSMISSION	| AT91C_MCI_RSPTYP_48	| AT91C_MCI_MAXLAT | AT91C_MCI_TRCMD_STOP,
	MMC_SEND_STATUS		| AT91C_MCI_RSPTYP_48	| AT91C_MCI_MAXLAT,
	0,
	MMC_GO_INACTIVE_STATE,

	/* Class 2 (16) */
	MMC_SET_BLOCKLEN	| AT91C_MCI_RSPTYP_48	| AT91C_MCI_MAXLAT,
	MMC_READ_SINGLE_BLOCK	| AT91C_MCI_RSPTYP_48	| AT91C_MCI_MAXLAT | AT91C_MCI_TRDIR | AT91C_MCI_TRCMD_START | AT91C_MCI_TRTYP_BLOCK,
	MMC_READ_MULTIPLE_BLOCK	| AT91C_MCI_RSPTYP_48	| AT91C_MCI_MAXLAT | AT91C_MCI_TRDIR | AT91C_MCI_TRCMD_START | AT91C_MCI_TRTYP_MULTIPLE,
	0,

	/* Class 3 (20) */
	MMC_WRITE_DAT_UNTIL_STOP| AT91C_MCI_RSPTYP_48	| AT91C_MCI_MAXLAT                   | AT91C_MCI_TRCMD_START | AT91C_MCI_TRTYP_STREAM,
	0,
	0,

	/* Class 4 (23) */
	MMC_SET_BLOCK_COUNT	| AT91C_MCI_RSPTYP_48	| AT91C_MCI_MAXLAT,
	MMC_WRITE_BLOCK		| AT91C_MCI_RSPTYP_48	| AT91C_MCI_MAXLAT                   | AT91C_MCI_TRCMD_START | AT91C_MCI_TRTYP_BLOCK,
	MMC_WRITE_MULTIPLE_BLOCK| AT91C_MCI_RSPTYP_48	| AT91C_MCI_MAXLAT                   | AT91C_MCI_TRCMD_START | AT91C_MCI_TRTYP_MULTIPLE,
	MMC_PROGRAM_CID		| AT91C_MCI_RSPTYP_48	| AT91C_MCI_MAXLAT,
	MMC_PROGRAM_CSD		| AT91C_MCI_RSPTYP_48,

	/* Class 6 (28) */
	MMC_SET_WRITE_PROT	| AT91C_MCI_RSPTYP_48,
	MMC_CLR_WRITE_PROT	| AT91C_MCI_RSPTYP_48,
	MMC_SEND_WRITE_PROT	| AT91C_MCI_RSPTYP_48,
	0,

	/* Class 5 (32) */
	0,
	0,
	0,
	MMC_ERASE_GROUP_START	| AT91C_MCI_RSPTYP_48,
	MMC_ERASE_GROUP_END	| AT91C_MCI_RSPTYP_48,
	MMC_ERASE		| AT91C_MCI_RSPTYP_48 	| AT91C_MCI_MAXLAT,
	0,

	/* Class 9 (39) */
	MMC_FAST_IO		| AT91C_MCI_RSPTYP_48 	| AT91C_MCI_MAXLAT,
	MMC_GO_IRQ_STATE	| AT91C_MCI_RSPTYP_48 	| AT91C_MCI_MAXLAT,
	SD_APP_OP_COND		| AT91C_MCI_RSPTYP_48,

	/* Class 7 (42) */
	MMC_LOCK_UNLOCK		| AT91C_MCI_RSPTYP_48 	| AT91C_MCI_MAXLAT,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,

	/* Class 8 (55) */
	MMC_APP_CMD		| AT91C_MCI_RSPTYP_48 	| AT91C_MCI_MAXLAT,
	MMC_GEN_CMD		| AT91C_MCI_RSPTYP_48 	| AT91C_MCI_MAXLAT,
	0,
	0,
	0,
	0,
	0,
	0,
	0
};

#define FL_SENT_COMMAND (1 << 6)
#define FL_SENT_STOP (1 << 7)

/* Low level type for this driver
 */
struct at91mci_host
{
	struct mmc_host* mmc;
	struct mmc_command* cmd;
	struct mmc_request* request;

	struct at91_mmc_data *board;
	int present;

	/*
	 * Flag indicating when the command has been sent. This is used to
	 * work out whether or not to send the stop
	 */
	unsigned int flags;

	/* DMA buffer used for transmitting */
	unsigned int* buffer;
	dma_addr_t physical_address;
	unsigned int total_length;

	/* Latest in the scatterlist that has been enabled for transfer, but not freed */
	int in_use_index;

	/* Latest in the scatterlist that has been enabled for transfer */
	int transfer_index;
};

/*
 * Copy from sg to a dma block - used for transfers
 */
static inline void at91mci_sg_to_dma(struct at91mci_host* host, struct mmc_data* data)
{
	unsigned int len, i, size;
	unsigned* dmabuf = host->buffer;

	size = host->total_length;

	len = data->sg_len;

	/*
	 * Just loop through all entries. Size might not
	 * be the entire list though so make sure that
	 * we do not transfer too much.
	 */
	for (i = 0;i < len;i++)
	{
		struct scatterlist* sg;
		int amount;
		int index;
		unsigned int* sgbuffer;

		sg = &data->sg[i];

		sgbuffer = kmap_atomic(sg->page, KM_BIO_SRC_IRQ) + sg->offset;
		amount = min(size, sg->length);
		size -= amount;
		amount /= 4;

		for (index = 0; index < amount; index++) {
			*dmabuf++ = BYTE_SWAP4(sgbuffer[index]);
		}
		kunmap_atomic(sg->page, KM_BIO_SRC_IRQ);

		if (size == 0)
			break;
	}

	/*
	 * Check that we didn't get a request to transfer
	 * more data than can fit into the SG list.
	 */
	BUG_ON(size != 0);
}

/*
 * Prepare a dma read
 */
static void at91mci_pre_dma_read(struct at91mci_host* host)
{
	int i;
	struct scatterlist* sg;
	struct mmc_command* cmd;
	struct mmc_data* data;

	DBG("pre dma read\n");

	cmd = host->cmd;
	if (!cmd) {
		DBG("no command\n");
		return;
	}

	data = cmd->data;
	if (!data) {
		DBG("no data\n");
		return;
	}

	for (i = 0; i < 2; i++) {
		/* nothing left to transfer */
		if (host->transfer_index >= data->sg_len) {
			DBG("Nothing left to transfer (index = %d)\n", host->transfer_index);
			break;
		}

		/* Check to see if this needs filling */
		if (i == 0) {
			if (controller->MCI_RCR != 0) {
				DBG("Transfer active in current\n");
				continue;
			}
		}
		else {
			if (controller->MCI_RNCR != 0) {
				DBG("Transfer active in next\n");
				continue;
			}
		}

		/* Setup the next transfer */
		DBG("Using transfer index %d\n", host->transfer_index);

		sg = &data->sg[host->transfer_index++];
		DBG("sg = %p\n", sg);

		sg->dma_address =
			dma_map_page(NULL, sg->page, sg->offset, sg->length, DMA_FROM_DEVICE);

		DBG("dma address = %08X, length = %d\n", sg->dma_address, sg->length);

		if (i == 0) {
			controller->MCI_RPR = sg->dma_address;
			controller->MCI_RCR = sg->length / 4;
		}
		else {
			controller->MCI_RNPR = sg->dma_address;
			controller->MCI_RNCR = sg->length / 4;
		}
	}

	DBG("pre dma read done\n");
}

/* Handle after a dma read
 */
static void at91mci_post_dma_read(struct at91mci_host* host)
{
	struct mmc_command* cmd;
	struct mmc_data* data;

	DBG("post dma read\n");

	cmd = host->cmd;
	if (!cmd) {
		DBG("no command\n");
		return;
	}

	data = cmd->data;
	if (!data) {
		DBG("no data\n");
		return;
	}

	while (host->in_use_index < host->transfer_index) {
		unsigned int* buffer;
		int index;
		int len;

		struct scatterlist* sg;

		DBG("finishing index %d\n", host->in_use_index);

		sg = &data->sg[host->in_use_index++];

		DBG("Unmapping page %08X\n", sg->dma_address);

		dma_unmap_page(NULL, sg->dma_address, sg->length, DMA_FROM_DEVICE);

		/* Swap the contents of the buffer */
		buffer = kmap_atomic(sg->page, KM_BIO_SRC_IRQ) + sg->offset;
		DBG("buffer = %p, length = %d\n", buffer, sg->length);

		data->bytes_xfered += sg->length;

		len = sg->length / 4;

		for (index = 0; index < len; index++) {
			buffer[index] = BYTE_SWAP4(buffer[index]);
		}
		flush_dcache_page(sg->page);
		kunmap_atomic(sg->page, KM_BIO_SRC_IRQ);
	}

	/* Is there another transfer to trigger? */
	if (host->transfer_index < data->sg_len) {
		at91mci_pre_dma_read(host);
	}
	else {
		controller->MCI_IER = AT91C_MCI_RXBUFF;
		controller->MCI_PTCR = AT91C_PDC_RXTDIS | AT91C_PDC_TXTDIS;
	}

	DBG("post dma read done\n");
}

/*
 * Handle transmitted data
 */
static void at91rm9200_mci_handle_transmitted(struct at91mci_host* host)
{
	struct mmc_command* cmd;
	struct mmc_data* data;

	DBG("Handling the transmit\n");

	/* Disable the transfer */
	controller->MCI_PTCR = AT91C_PDC_RXTDIS | AT91C_PDC_TXTDIS;

	/* Now wait for cmd ready */
	controller->MCI_IDR = AT91C_MCI_TXBUFE;
	controller->MCI_IER = AT91C_MCI_NOTBUSY;

	cmd = host->cmd;
	if (!cmd) return;

	data = cmd->data;
	if (!data) return;

	data->bytes_xfered = host->total_length;
}

/*
 * Enable the controller
 */
static void at91rm9200_mci_enable(void)
{
	AT91_SYS->PMC_PCER = 1 << AT91C_ID_MCI;		/* Enable the peripheral clock */
	controller->MCI_CR = AT91C_MCI_MCIEN;
	controller->MCI_IDR = 0xFFFFFFFF;
	controller->MCI_DTOR = AT91C_MCI_DTOMUL_1048576 | AT91C_MCI_DTOCYC;
	controller->MCI_MR = 0x834A;
	controller->MCI_SDCR = 0x0;
}

/*
 * Disable the controller
 */
static void at91rm9200_mci_disable(void)
{
	controller->MCI_CR = AT91C_MCI_MCIDIS | 0x80;
	AT91_SYS->PMC_PCDR = 1 << AT91C_ID_MCI;		/* Disable the peripheral clock */
}

/*
 * Send a command
 * return the interrupts to enable
 */
static unsigned int at91rm9200_mci_send_command(struct at91mci_host* host, struct mmc_command* cmd)
{
	unsigned int cmdr;
	unsigned int block_length;
	struct mmc_data* data = cmd->data;
	struct mmc_host* mmc = host->mmc;

	unsigned int blocks;
	unsigned int ier = 0;

	host->cmd = cmd;

	/* Not sure if this is needed */
#if 0
	if ((controller->MCI_SR & AT91C_MCI_RTOE) && (cmd->opcode == 1)) {
		DBG("Clearing timeout\n");
		controller->MCI_ARGR = 0;
		controller->MCI_CMDR = AT91C_MCI_OPDCMD;
		while (!(controller->MCI_SR & AT91C_MCI_CMDRDY)) {
			/* spin */
			DBG("Clearing: SR = %08X\n", controller->MCI_SR);
		}
	}
#endif

	cmdr = commands[cmd->opcode];
	if (data) {
		block_length = 1 << data->blksz_bits;
		blocks = data->blocks;
	}
	else {
		block_length = 0;
		blocks = 0;
	}

	if (mmc->ios.bus_mode == MMC_BUSMODE_OPENDRAIN) {
		cmdr |= AT91C_MCI_OPDCMD;
	}

	/*
	 * Set the arguments and send the command
	 */
	DBG("Sending command %d as %08X, arg = %08X, blocks = %d, length = %d (MR = %08X)\n",
		cmd->opcode, cmdr, cmd->arg, blocks, block_length, controller->MCI_MR);

	if (!data) {
		controller->MCI_PTCR = (AT91C_PDC_TXTDIS | AT91C_PDC_RXTDIS);
		controller->MCI_RPR  = 0;
		controller->MCI_RCR  = 0;
		controller->MCI_RNPR  = 0;
		controller->MCI_RNCR  = 0;
		controller->MCI_TPR  = 0;
		controller->MCI_TCR  = 0;
		controller->MCI_TNPR  = 0;
		controller->MCI_TNCR  = 0;

		controller->MCI_ARGR = cmd->arg;
		controller->MCI_CMDR = cmdr;
		return AT91C_MCI_CMDRDY;
	}

	controller->MCI_MR &= ~0xFFFF8000;	/* zero block length and PDC mode */
	controller->MCI_MR |= (block_length << 16) | AT91C_MCI_PDCMODE;

	/*
	 * Disable the PDC controller
	 */
	controller->MCI_PTCR = AT91C_PDC_RXTDIS | AT91C_PDC_TXTDIS;

	if (cmdr & AT91C_MCI_TRCMD_START) {
		data->bytes_xfered = 0;
		host->transfer_index = 0;
		host->in_use_index = 0;
		if (cmdr & AT91C_MCI_TRDIR) {
			/*
			 * Handle a read
			 */
			host->buffer = NULL;
			host->total_length = 0;

			at91mci_pre_dma_read(host);
			ier = AT91C_MCI_ENDRX /* | AT91C_MCI_RXBUFF */;
		} else {
			/*
			 * Handle a write
			 */
			host->total_length = block_length * blocks;
			host->buffer = dma_alloc_coherent(NULL,
							  host->total_length,
							  &host->physical_address, GFP_KERNEL);

			at91mci_sg_to_dma(host, data);

			DBG("Transmitting %d bytes\n", host->total_length);

			controller->MCI_TPR = host->physical_address;
			controller->MCI_TCR = host->total_length / 4;
			ier = AT91C_MCI_TXBUFE;
		}
	}

	/*
	 * Send the command and then enable the PDC - not the other way round as
	 * the data sheet says
	 */

	controller->MCI_ARGR = cmd->arg;
	controller->MCI_CMDR = cmdr;

	if (cmdr & AT91C_MCI_TRCMD_START) {
		if (cmdr & AT91C_MCI_TRDIR) {
			controller->MCI_PTCR = AT91C_PDC_RXTEN;
		}
		else {
			controller->MCI_PTCR = AT91C_PDC_TXTEN;
		}
	}

	return ier;
}

/*
 * Wait for a command to complete
 */
static void at91mci_process_command(struct at91mci_host* host, struct mmc_command* cmd)
{
	unsigned int ier;

	ier = at91rm9200_mci_send_command(host, cmd);

	DBG("setting ier to %08X\n", ier);

	/* Stop on errors or the required value */
	controller->MCI_IER = 0xffff0000 | ier;
}

/*
 * Process the next step in the request
 */
static void at91mci_process_next(struct at91mci_host* host)
{
	if (!(host->flags & FL_SENT_COMMAND)) {
		host->flags |= FL_SENT_COMMAND;
		at91mci_process_command(host, host->request->cmd);
	}
	else if ((!(host->flags & FL_SENT_STOP)) && host->request->stop) {
		host->flags |= FL_SENT_STOP;
		at91mci_process_command(host, host->request->stop);
	}
	else {
		mmc_request_done(host->mmc, host->request);
	}
}

/*
 * Handle a command that has been completed
 */
static void at91mci_completed_command(struct at91mci_host* host)
{
	struct mmc_command* cmd = host->cmd;
	unsigned int status;

	controller->MCI_IDR = 0xffffffff;

	cmd->resp[0] = controller->MCI_RSPR[0];
	cmd->resp[1] = controller->MCI_RSPR[1];
	cmd->resp[2] = controller->MCI_RSPR[2];
	cmd->resp[3] = controller->MCI_RSPR[3];

	if (host->buffer) {
		dma_free_coherent(NULL, host->total_length, host->buffer, host->physical_address);
		host->buffer = NULL;
	}

	status = controller->MCI_SR;

	DBG("Status = %08X [%08X %08X %08X %08X]\n",
		 status, cmd->resp[0], cmd->resp[1], cmd->resp[2], cmd->resp[3]);

	if (status & (AT91C_MCI_RINDE | AT91C_MCI_RDIRE | AT91C_MCI_RCRCE |
			AT91C_MCI_RENDE | AT91C_MCI_RTOE | AT91C_MCI_DCRCE |
			AT91C_MCI_DTOE | AT91C_MCI_OVRE | AT91C_MCI_UNRE)) {
		if ((cmd->opcode == MMC_SEND_OP_COND) || (cmd->opcode == SD_APP_OP_COND)) {
			cmd->error = MMC_ERR_NONE;
		}
		else {
			if (status & (AT91C_MCI_RTOE | AT91C_MCI_DTOE)) {
				cmd->error = MMC_ERR_TIMEOUT;
			}
			else if (status & (AT91C_MCI_RCRCE | AT91C_MCI_DCRCE)) {
				cmd->error = MMC_ERR_BADCRC;
			}
			else if (status & (AT91C_MCI_OVRE | AT91C_MCI_UNRE)) {
				cmd->error = MMC_ERR_FIFO;
			}
			else {
				cmd->error = MMC_ERR_FAILED;
			}

			DBG("Error detected and set to %d (cmd = %d, retries = %d)\n",
				 cmd->error, cmd->opcode, cmd->retries);
		}
	}
	else {
		cmd->error = MMC_ERR_NONE;
	}
	
	if ((cmd->opcode == SD_APP_SET_BUS_WIDTH) && (cmd->error == MMC_ERR_NONE)) {
		if (cmd->arg == 2)
			controller->MCI_SDCR |= 0x80;
		else
			controller->MCI_SDCR &= ~0x80;
	}

	at91mci_process_next(host);
}

/*
 * Handle an MMC request
 */
static void at91rm9200_mci_request(struct mmc_host* mmc, struct mmc_request* mrq)
{
	struct at91mci_host* host = mmc_priv(mmc);
	host->request = mrq;
	host->flags = 0;
	
	at91mci_process_next(host);
}

/*
 * Set the IOS
 */
static void at91rm9200_mci_set_ios(struct mmc_host* mmc, struct mmc_ios* ios)
{
	int clkdiv;
	struct at91mci_host *host = mmc_priv(mmc);

	DBG("Clock %uHz, busmode %u, powermode %u, Vdd %u\n",
		ios->clock, ios->bus_mode, ios->power_mode, ios->vdd);

	if (ios->clock == 0) {
		/* Disable the MCI controller */
		controller->MCI_CR = AT91C_MCI_MCIDIS;
		clkdiv = 0;
 		controller->MCI_SDCR &= ~0x80;
	}
	else {
		/* Enable the MCI controller */
		controller->MCI_CR = AT91C_MCI_MCIEN;

                /* figure out a clock divisor for MCCK */

                /* TODO: refactor all this math, to be sure that the
                   resulting mcck is truly less than or equal to
                   ios->clock */
                DBG("ios->clock = %d\n", ios->clock);
                DBG("at91_master_clock = %ld\n", at91_master_clock);

		clkdiv = ((at91_master_clock / ios->clock) / 2) - 1;

                DBG("unadjusted clkdiv = %d\n", clkdiv);

 		/* TODO: this band-aid prevents us from getting the
                   wrong divisor; we shouldn't need it, but the maths
                   above and below are still flawed in some way that
                   needs to be investigated */
                if (clkdiv <= 0) clkdiv = 1;
                while ((at91_master_clock / (2 * (clkdiv + 1))) > ios->clock)
                  clkdiv++;

                DBG("clkdiv = %d mcck = %ld\n", clkdiv,
                    at91_master_clock / (2 * (clkdiv + 1)));
	}

	/* Set the clock divider */
	controller->MCI_MR = (controller->MCI_MR & ~0x000000FF) | clkdiv;

	/* maybe switch power to the card */
	if (host && host->board->vcc_pin) {
		switch (ios->power_mode) {
		case MMC_POWER_OFF:
			at91_set_gpio_direction(host->board->vcc_pin, 0, 0);
			break;
		case MMC_POWER_UP:
		case MMC_POWER_ON:
			at91_set_gpio_direction(host->board->vcc_pin, 0, 1);
			break;
		}
	}
}

/*
 * Handle an interrupt
 */
static irqreturn_t at91rm9200_mci_irq(int irq, void* devid, struct pt_regs* regs)
{
	struct at91mci_host* host = devid;
	int completed = 0;

	unsigned int int_status;

	if (host == NULL) {
		return IRQ_HANDLED;
	}

	int_status = controller->MCI_SR;
	DBG("MCI irq: status = %08X, %08X, %08X\n", int_status, controller->MCI_IMR,
		int_status & controller->MCI_IMR);

	if ((int_status & controller->MCI_IMR) & 0xffff0000) {
		completed = 1;
	}
	int_status &= controller->MCI_IMR;

	if (int_status & AT91C_MCI_UNRE) {
		DBG("Underrun error\n");
	}

	if (int_status & AT91C_MCI_OVRE) {
		DBG("Overrun error\n");
	}

	if (int_status & AT91C_MCI_DTOE) {
		DBG("Data timeout\n");
	}

	if (int_status & AT91C_MCI_DCRCE) {
		DBG("CRC error in data\n");
	}

	if (int_status & AT91C_MCI_RTOE) {
		DBG("Response timeout\n");
	}

	if (int_status & AT91C_MCI_RENDE) {
		DBG("Response end bit error\n");
	}

	if (int_status & AT91C_MCI_RCRCE) {
		DBG("Response CRC error\n");
	}

	if (int_status & AT91C_MCI_RINDE) {
		DBG("Response index error\n");
	}

	if (int_status & AT91C_MCI_TXBUFE) {
		DBG("TX buffer empty\n");
		at91rm9200_mci_handle_transmitted(host);
	}

	if (int_status & AT91C_MCI_RXBUFF) {
		DBG("RX buffer full\n");
		controller->MCI_IER = AT91C_MCI_CMDRDY;
	}

	if (int_status & AT91C_MCI_ENDTX) {
		DBG("Transmit has ended\n");
	}

	if (int_status & AT91C_MCI_ENDRX) {
		DBG("Receive has ended\n");
		at91mci_post_dma_read(host);
	}

	if (int_status & AT91C_MCI_NOTBUSY) {
		DBG("Card is ready\n");
		controller->MCI_IER = AT91C_MCI_CMDRDY;
	}

	if (int_status & AT91C_MCI_DTIP) {
		DBG("Data transfer in progress\n");
	}

	if (int_status & AT91C_MCI_BLKE) {
		DBG("Black transfer has ended\n");
	}

	if (int_status & AT91C_MCI_TXRDY) {
		DBG("Ready to transmit\n");
	}

	if (int_status & AT91C_MCI_RXRDY) {
		DBG("Ready to receive\n");
	}

	if (int_status & AT91C_MCI_CMDRDY) {
		DBG("Command ready\n");
		completed = 1;
	}

	controller->MCI_IDR = int_status;

	if (completed) {
		DBG("Completed command\n");
		controller->MCI_IDR = 0xffffffff;
		at91mci_completed_command(host);
	}

	return IRQ_HANDLED;
}

static irqreturn_t
at91rm9200_mmc_det_irq(int irq, void *_host, struct pt_regs* regs)
{
	struct at91mci_host *host = _host;
	int present = !at91_get_gpio_value(irq);

	/* we expect this irq on both insert and remove,
	 * and use a short delay to debounce.
	 */
	if (present != host->present) {
		host->present = present;
		DBG("%s: card %s\n", host->mmc->host_name,
			present ? "insert" : "remove");
		schedule_delayed_work(&host->mmc->detect,
				msecs_to_jiffies(100));
	}
	return IRQ_HANDLED;
}

static struct mmc_host_ops at91rm9200_mci_ops = {
	.request	= at91rm9200_mci_request,
	.set_ios	= at91rm9200_mci_set_ios,
};

/*
 * Probe for the device
 */
static int at91rm9200_mci_probe(struct device* dev)
{
	struct mmc_host* mmc;
	struct at91mci_host* host;
	int ret;

	DBG("Probe MCI devices\n");

	mmc = mmc_alloc_host(sizeof(struct at91mci_host), dev);
	if (!mmc) {
		DBG("Failed to allocate mmc host\n");

		ret = -ENOMEM;
		return ret;
	}

	mmc->ops = &at91rm9200_mci_ops;
	mmc->f_min = 400000;
	mmc->f_max = 25000000;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->buffer = NULL;
	host->board = dev->platform_data;

	/*
	 * Allocate the MCI interrupt
	 */
	ret = request_irq(AT91C_ID_MCI, at91rm9200_mci_irq, SA_SHIRQ, DRIVER_NAME, host);
	if (ret) {
		DBG("Failed to request MCI interrupt\n");
		return ret;
	}

	dev_set_drvdata(dev, mmc);

	/*
	 * Add host to MMC layer
	 */
	if (host->board->det_pin)
		host->present = !at91_get_gpio_value(host->board->det_pin);
	else
		host->present = -1;

	if (host->board->wire4)
		host->mmc->flags |= HOST_CAP_SD_4_BIT;

	mmc_add_host(mmc);

	/*
	 * monitor card insertion/removal if we can
	 */
	if (host->board->det_pin) {
		ret = request_irq(host->board->det_pin, at91rm9200_mmc_det_irq,
				SA_SAMPLE_RANDOM, DRIVER_NAME, host);
		if (ret)
			DBG("couldn't allocate MMC detect irq\n");
	}

	DBG(KERN_INFO "Added MCI driver\n");

	return 0;
}

/*
 * Remove a device
 */
static int at91rm9200_mci_remove(struct device* dev)
{
	struct mmc_host* mmc = dev_get_drvdata(dev);
	struct at91mci_host* host;

	if (!mmc) return -1;

	host = mmc_priv(mmc);

	if (host->present != -1) {
		free_irq(host->board->det_pin, host);
		cancel_delayed_work(&host->mmc->detect);
	}

	free_irq(AT91C_ID_MCI, host);
	mmc_free_host(mmc);

	DBG("Removed\n");

	return 0;
}

static struct device_driver at91rm9200_mci_driver = {
	.name		= DRIVER_NAME,
	.bus		= &platform_bus_type,
	.probe		= at91rm9200_mci_probe,
	.remove		= at91rm9200_mci_remove,
};

/* Initialise the MCI controller
 */
static int __init at91rm9200_mci_init(void)
{
	int result;

	at91rm9200_mci_disable();
	at91rm9200_mci_enable();

	result = driver_register(&at91rm9200_mci_driver);
	if (result < 0) {
		printk(KERN_ERR "AT91 MCI: Failed to register driver\n");
		return result;
	}

	printk(KERN_INFO "AT91RM9200 MCI initialized\n");

	return 0;
}

/*
 * Release the controller
 */
static void __exit at91rm9200_mci_exit(void)
{
	at91rm9200_mci_disable();
	driver_unregister(&at91rm9200_mci_driver);

	printk(KERN_INFO "AT91RM9200 MCI removed\n");
}

module_init(at91rm9200_mci_init);
module_exit(at91rm9200_mci_exit);

MODULE_DESCRIPTION("AT91RM9200 Multimedia Card Interface driver");
MODULE_AUTHOR("Nick Randell");
MODULE_LICENSE("GPL");
