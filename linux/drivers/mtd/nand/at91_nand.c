/*
 * drivers/at91/mtd/at91_nand.c
 *
 *  Copyright (C) 2003 Rick Bronson
 *
 *  Derived from drivers/mtd/nand/autcpu12.c
 *	 Copyright (c) 2001 Thomas Gleixner (gleixner@autronix.de)
 *
 *  Derived from drivers/mtd/spia.c
 *	 Copyright (C) 2000 Steven J. Hill (sjhill@cotw.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/sizes.h>

#include <asm/arch/pio.h>

#define AT91_SMART_MEDIA_ALE (1 << 22)  /* our ALE is AD22 */
#define AT91_SMART_MEDIA_CLE (1 << 21)  /* our CLE is AD21 */
/*
 * MTD structure for AT91 board
 */

static struct mtd_info *at91_mtd = NULL;

static int at91_fio_base;

#ifdef CONFIG_MTD_PARTITIONS

/*
 * Define partitions for flash devices
 */

static struct mtd_partition partition_info32k[] = {
	{ name: "AT91 NAND partition 1, kernel",
	  offset:  0,
	  size:   1 * SZ_1M },
	{ name: "AT91 NAND partition 2, filesystem",
	  offset:  1 * SZ_1M,
	  size:   16 * SZ_1M },
	{ name: "AT91 NAND partition 3a, storage",
	  offset: (1 * SZ_1M) + (16 * SZ_1M),
	  size:   1 * SZ_1M },
	{ name: "AT91 NAND partition 3b, storage",
	  offset: (2 * SZ_1M) + (16 * SZ_1M),
	  size:   1 * SZ_1M },
	{ name: "AT91 NAND partition 3c, storage",
	  offset: (3 * SZ_1M) + (16 * SZ_1M),
	  size:   1 * SZ_1M },
	{ name: "AT91 NAND partition 3d, storage",
	  offset: (4 * SZ_1M) + (16 * SZ_1M),
	  size:   1 * SZ_1M },
};

static struct mtd_partition partition_info64k[] = {
	{ name: "AT91 NAND partition 1, kernel",
	  offset:  0,
	  size:   1 * SZ_1M },
	{ name: "AT91 NAND partition 2, filesystem",
	  offset:  1 * SZ_1M,
	  size:   16 * SZ_1M },
	{ name: "AT91 NAND partition 3, storage",
	  offset: (1 * SZ_1M) + (16 * SZ_1M),
	  size:   47 * SZ_1M },
};

#endif

/*
 * Hardware specific access to control-lines
 */
static void at91_hwcontrol (struct mtd_info *mtd, int cmd)
{
	struct nand_chip *my_nand = mtd->priv;
	switch(cmd)
	{
	case NAND_CTL_SETCLE:
		my_nand->IO_ADDR_W = (void __iomem *)(at91_fio_base + AT91_SMART_MEDIA_CLE);
		break;
	case NAND_CTL_CLRCLE:
		my_nand->IO_ADDR_W = (void __iomem *)(at91_fio_base);
		break;
	case NAND_CTL_SETALE:
		my_nand->IO_ADDR_W = (void __iomem *)(at91_fio_base + AT91_SMART_MEDIA_ALE);
		break;
	case NAND_CTL_CLRALE:
		my_nand->IO_ADDR_W = (void __iomem *)(at91_fio_base);
		break;
	case NAND_CTL_SETNCE:
		break;
	case NAND_CTL_CLRNCE:
		break;
	}
}

/*
 * Send command to NAND device
 */
static void at91_nand_command (struct mtd_info *mtd, unsigned command, int column, int page_addr)
{
	register struct nand_chip *my_nand = mtd->priv;

	/* Begin command latch cycle */
	register unsigned long NAND_IO_ADDR = (unsigned long)(my_nand->IO_ADDR_W + AT91_SMART_MEDIA_CLE);

	/*
	 * Write out the command to the device.
	 */
	if (command != NAND_CMD_SEQIN)
		writeb (command, NAND_IO_ADDR);
	else {
		if (mtd->oobblock == 256 && column >= 256) {
			column -= 256;
			writeb (NAND_CMD_RESET, NAND_IO_ADDR);
			writeb (NAND_CMD_READOOB, NAND_IO_ADDR);
			writeb (NAND_CMD_SEQIN, NAND_IO_ADDR);
		}
		else
			if (mtd->oobblock == 512 && column >= 256) {
				if (column < 512) {
					column -= 256;
					writeb (NAND_CMD_READ1, NAND_IO_ADDR);
					writeb (NAND_CMD_SEQIN, NAND_IO_ADDR);
				} else {
					column -= 512;
					writeb (NAND_CMD_READOOB, NAND_IO_ADDR);
					writeb (NAND_CMD_SEQIN, NAND_IO_ADDR);
				}
			} else {
				writeb (NAND_CMD_READ0, NAND_IO_ADDR);
				writeb (NAND_CMD_SEQIN, NAND_IO_ADDR);
			}
	}

	/* Set ALE and clear CLE to start address cycle */
	NAND_IO_ADDR = at91_fio_base;

	if (column != -1 || page_addr != -1)
		NAND_IO_ADDR += AT91_SMART_MEDIA_ALE;

	/* Serially input address */
	if (column != -1)
		writeb (column, NAND_IO_ADDR);
	if (page_addr != -1) {
		writeb ((unsigned char) (page_addr & 0xff), NAND_IO_ADDR);
		writeb ((unsigned char) ((page_addr >> 8) & 0xff), NAND_IO_ADDR);
		/* One more address cycle for higher density devices */
		if (mtd->size & 0x0c000000) {
			writeb ((unsigned char) ((page_addr >> 16) & 0x0f), NAND_IO_ADDR);
		}
	}

	/* wait until command is processed */
	while (!my_nand->dev_ready(mtd))
		;
}

/*
 * Read the Device Ready pin.
 */
static int at91_device_ready (struct mtd_info *mtd)
{
	return AT91_PIO_SmartMedia_RDY();
}
/*
 * Main initialization routine
 */
static int __init at91_init (void)
{
	struct nand_chip *my_nand;
	int err = 0;

	/* Allocate memory for MTD device structure and private data */
	at91_mtd = kmalloc (sizeof(struct mtd_info) + sizeof (struct nand_chip), GFP_KERNEL);
	if (!at91_mtd) {
		printk ("Unable to allocate AT91 NAND MTD device structure.\n");
		err = -ENOMEM;
		goto out;
	}

	/* map physical adress */
	at91_fio_base = (unsigned long) ioremap(AT91_SMARTMEDIA_BASE, SZ_8M);
	if(!at91_fio_base) {
		printk("ioremap AT91 NAND failed\n");
		err = -EIO;
		goto out_mtd;
	}

	/* Get pointer to private data */
	my_nand = (struct nand_chip *) (&at91_mtd[1]);

	/* Initialize structures */
	memset((char *) at91_mtd, 0, sizeof(struct mtd_info));
	memset((char *) my_nand, 0, sizeof(struct nand_chip));

	/* Link the private data with the MTD structure */
	at91_mtd->priv = my_nand;

	/* Set address of NAND IO lines */
	my_nand->IO_ADDR_R = (void __iomem *)(at91_fio_base);
	my_nand->IO_ADDR_W = (void __iomem *)(at91_fio_base);
	my_nand->hwcontrol = at91_hwcontrol;
	my_nand->dev_ready = at91_device_ready;
	my_nand->cmdfunc = at91_nand_command;	/* we need our own */
	my_nand->eccmode = NAND_ECC_SOFT;	/* enable ECC */
	/* 20 us command delay time */
	my_nand->chip_delay = 20;

	/* Setup Smart Media, first enable the address range of CS3 */
	AT91_SYS->EBI_CSA |= AT91C_EBI_CS3A_SMC_SmartMedia;
	/* set the bus interface characteristics based on
	   tDS Data Set up Time 30 - ns
	   tDH Data Hold Time 20 - ns
	   tALS ALE Set up Time 20 - ns
	   16ns at 60 MHz ~= 3  */
#define AT91C_SM_ID_RWH	(5 << 28)		/* orig = 5 */
#define AT91C_SM_RWH	(1 << 28)		/* orig = 1 */
#define AT91C_SM_RWS	(0 << 24)		/* orig = 0 */
#define AT91C_SM_TDF	(1 << 8)		/* orig = 1 */
#define AT91C_SM_NWS	(5)			/* orig = 3 */
	AT91_SYS->EBI_SMC2_CSR[3] = ( AT91C_SM_RWH | AT91C_SM_RWS |
					 AT91C_SMC2_ACSS_STANDARD |
					 AT91C_SMC2_DBW_8 | AT91C_SM_TDF |
					 AT91C_SMC2_WSEN | AT91C_SM_NWS);

	AT91_CfgPIO_SmartMedia();

	if (AT91_PIO_SmartMedia_CardDetect())
		printk ("No ");
	printk ("SmartMedia card inserted.\n");

	/* Scan to find existance of the device */
	if (nand_scan (at91_mtd, 1)) {
		err = -ENXIO;
		goto out_ior;
	}

#ifdef CONFIG_MTD_PARTITIONS
	/* Register the partitions */
	switch(at91_mtd->size)
	{
	case SZ_32M:
		err = add_mtd_partitions(at91_mtd, partition_info32k,
				ARRAY_SIZE (partition_info32k));
		break;
	case SZ_64M:
		err = add_mtd_partitions(at91_mtd, partition_info64k,
				ARRAY_SIZE (partition_info64k));
		break;
	default:
		printk ("Unsupported SmartMedia device\n");
		err = -ENXIO;
		goto out_cac;
	}
#else
	err = add_mtd_device(at91_mtd);
#endif
	goto out;

 out_cac:
 out_ior:
	iounmap((void *)at91_fio_base);
 out_mtd:
	kfree (at91_mtd);
 out:
	return err;
}

/*
 * Clean up routine
 */
static void __exit at91_cleanup (void)
{
	/* Unregister partitions */
	del_mtd_partitions(at91_mtd);

	/* Unregister the device */
	del_mtd_device (at91_mtd);

	/* unmap physical adress */
	iounmap((void *)at91_fio_base);

	/* Free the MTD device structure */
	kfree (at91_mtd);
}

module_init(at91_init);
module_exit(at91_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Rick Bronson");
MODULE_DESCRIPTION("Glue layer for SmartMediaCard on ATMEL AT91RM9200");
