/*
 *
 * Map driver for the Cogent CSBxxx boards.
 *
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
#define MSG_PREFIX "csbxxx: "

#if defined(CONFIG_MACH_CSB337) \
  || defined(CONFIG_MACH_CSB437TL) \
  || defined(CONFIG_MACH_CSB637)
#define WINDOW_ADDR 0x10000000
#define WINDOW_SIZE 0x1000000
#elif defined(CONFIG_MACH_CSB625)
#define WINDOW_ADDR 0x00000000
#define WINDOW_SIZE 0x1000000
#elif defined(CONFIG_MACH_CSB535)
#define WINDOW_ADDR 0xc8000000
#define WINDOW_SIZE 0x00800000
#elif defined(CONFIG_ARCH_CSB226)
#error TODO: finish MTD map for CSB226
#else
#error TODO: the MTD map you need goes here...
#endif

#define NB_OF(a) (sizeof(a)/sizeof(*(a)))

static void csbxxx_map_inval_cache(struct map_info *map, unsigned long from, ssize_t len)
{
  consistent_sync((char *)map->cached + from, len, DMA_FROM_DEVICE);
}

static struct map_info csbxxx_map = {
    .size = WINDOW_SIZE,
    .phys = WINDOW_ADDR,
    .inval_cache = csbxxx_map_inval_cache,
    .bankwidth = 2,
    .name = MTDID,
};

/* default map definition,
   generally overridden on the command line */
static struct mtd_partition csbxxx_partitions[] = {
  {
    .name = "uMON flash",
    .size = WINDOW_SIZE,
    .mask_flags = MTD_WRITEABLE  /* force read-only */
  },
};

static const char *probes[] = { "cmdlinepart", NULL };

static struct mtd_info *mymtd = 0;
static int mtd_parts_nb = 0;
static struct mtd_partition *mtd_parts = 0;

static int __init init_csbxxx(void)
{
  int ret = 0;
  const char *part_type = 0;

  csbxxx_map.virt = ioremap(csbxxx_map.phys, WINDOW_SIZE);
  if (!csbxxx_map.virt) {
    printk(KERN_WARNING "Failed to ioremap %s, MTD disabled\n", csbxxx_map.name);
    ret = -ENOMEM;
    goto err;
  }
  csbxxx_map.cached = ioremap_cached(csbxxx_map.phys, WINDOW_SIZE);
  if (!csbxxx_map.cached)
    printk(KERN_WARNING "Failed to ioremap cached %s\n", csbxxx_map.name);

  simple_map_init(&csbxxx_map);
  
  printk(KERN_NOTICE "Probing %s at physical address 0x%08lx (%d-bit bankwidth)\n",
	 csbxxx_map.name, csbxxx_map.phys, csbxxx_map.bankwidth * 8);
  
  mymtd = do_map_probe("cfi_probe", &csbxxx_map);
  
  if (!mymtd)
    goto err;

  mymtd->owner = THIS_MODULE;
  
  mtd_parts_nb = parse_mtd_partitions(mymtd, probes, &mtd_parts, 0);

  if (mtd_parts_nb > 0)
    part_type = "command line";
  else if (mtd_parts_nb == 0)
    {
      mtd_parts = csbxxx_partitions;
      mtd_parts_nb = NB_OF(csbxxx_partitions);
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
  if (csbxxx_map.virt)
    iounmap(csbxxx_map.virt);
  if (csbxxx_map.cached)
    iounmap(csbxxx_map.cached);
  if (!ret)
    ret = -EIO;

  return ret;
}

static void __exit cleanup_csbxxx(void)
{
  if (!mymtd)
    return;

  del_mtd_partitions(mymtd);
  
  map_destroy(mymtd);
  iounmap((void *)csbxxx_map.virt);
  if (csbxxx_map.cached)
    iounmap(csbxxx_map.cached);
}

module_init(init_csbxxx);
module_exit(cleanup_csbxxx);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bill Gatliff <bgat@billgatliff.com>");
MODULE_DESCRIPTION("MTD map driver for Cogent CSBXXX");
