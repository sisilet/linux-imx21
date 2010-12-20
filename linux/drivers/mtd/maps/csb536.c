/*
 * $Id: csb536.c,v 1.2 2005/03/30 18:26:49 arch-linux Exp $
 *
 * Map driver for the Cogent CSB536.
 *
 * Copyright (C) 2005 Jay Monkman <jtm@lopingdog.com>
 *
 * Copied from csb637.c:
 * Author:	Bill Gatliff
 * Copyright:	(C) 2005 Bill Gatliff <bgat@billgatliff.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <asm/io.h>
#include <asm/hardware.h>

#define MTDID "flash00"
#define MSG_PREFIX "CSB536: "

#define WINDOW_ADDR 0x10000000      /* physical properties of flash */
#define WINDOW_SIZE 0x800000

#define NB_OF(a) (sizeof(a)/sizeof(*(a)))

static void csb536_map_inval_cache(struct map_info *map, unsigned long from, ssize_t len)
{
  consistent_sync((char *)map->cached + from, len, DMA_FROM_DEVICE);
}

static struct map_info csb536_map = {
    .size = WINDOW_SIZE,
    .phys = WINDOW_ADDR,
    .inval_cache = csb536_map_inval_cache,
    .bankwidth = 2,
    .name = MTDID,
};

/* This is here just for reference; this memory is not truly 
   unused in the default uMON setup */
static struct mtd_partition csb536_partitions[] = {
  {
    .name = "unused flash",
    .size = 0x10800000UL - 0x10080000UL,
    .offset = 0x8000,
    .mask_flags = MTD_WRITEABLE  /* force read-only */
  },
};

static const char *probes[] = { "cmdlinepart", NULL };

static struct mtd_info *mymtd = 0;
static int mtd_parts_nb = 0;
static struct mtd_partition *mtd_parts = 0;

static int __init init_csb536(void)
{
  int ret = 0;
  const char *part_type = 0;

  csb536_map.virt = ioremap(csb536_map.phys, WINDOW_SIZE);
  if (!csb536_map.virt) {
    printk(KERN_WARNING "Failed to ioremap %s\n", csb536_map.name);
    ret = -ENOMEM;
    goto err;
  }
  csb536_map.cached = ioremap_cached(csb536_map.phys, WINDOW_SIZE);
  if (!csb536_map.cached)
    printk(KERN_WARNING "Failed to ioremap cached %s\n", csb536_map.name);

  simple_map_init(&csb536_map);
  
  printk(KERN_NOTICE "Probing %s at physical address 0x%08lx (%d-bit bankwidth)\n",
	 csb536_map.name, csb536_map.phys, csb536_map.bankwidth * 8);
  
  mymtd = do_map_probe("cfi_probe", &csb536_map);
  
  if (!mymtd)
    goto err;

  mymtd->owner = THIS_MODULE;
  
  mtd_parts_nb = parse_mtd_partitions(mymtd, probes, &mtd_parts, 0);

  if (mtd_parts_nb > 0)
    part_type = "command line";
  else if (mtd_parts_nb == 0)
    {
      mtd_parts = csb536_partitions;
      mtd_parts_nb = NB_OF(csb536_partitions);
      part_type = "static";
    }
  else goto err;

#if 1
#warning "TODO: is add_mtd_device needed?"
#else
  add_mtd_device(mymtd);
#endif
  if (mtd_parts_nb == 0)
    printk(KERN_NOTICE MSG_PREFIX "no partition info available\n");
  else
    {
      printk(KERN_NOTICE MSG_PREFIX
	     "using %s partition definition\n", part_type);
      add_mtd_partitions(mymtd, mtd_parts, mtd_parts_nb);
    }
  
  return 0;

err:
  if (csb536_map.virt)
    iounmap(csb536_map.virt);
  if (csb536_map.cached)
    iounmap(csb536_map.cached);
  if (!ret)
    ret = -EIO;

  return ret;
}

static void __exit cleanup_csb536(void)
{
  if (!mymtd)
    return;

  del_mtd_partitions(mymtd);
  
  map_destroy(mymtd);
  iounmap((void *)csb536_map.virt);
  if (csb536_map.cached)
    iounmap(csb536_map.cached);
}

module_init(init_csb536);
module_exit(cleanup_csb536);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jay Monkman <jtm@lopingdog.com>");
MODULE_DESCRIPTION("MTD map driver for Cogent CSB536");
