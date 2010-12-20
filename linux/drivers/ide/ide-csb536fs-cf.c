/*
 * Compact Flash driver for the CSB536FS
 *
 * Copyright (c) 2005 Jay Monkman <jtm@lopingdog.com>
 *
 * This is derived from Cadenux's au1100 compact flash driver:
 *   Copyright (C) 2005 Cadenux, LLC (http://www.cadenux.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 */

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <linux/config.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/ide.h>

#include <asm/irq.h>
#include <asm/io.h>

#include <asm/arch/irqs.h>



/* To enable DEBUG mode for the driver, define the following: */
#define DEBUG 1 

/**************************************************************************
 * Definitions
 **************************************************************************/

#ifdef DEBUG
# define dbg(format, arg...) printk(format, ##arg)
#else
# define dbg(x...)
#endif

#define info(format, arg...)   printk(KERN_INFO format , ##arg)
#define warn(format, arg...)   printk(KERN_WARNING format , ##arg)
#define err(format, arg...)    printk(KERN_ERR format , ##arg)

#define CSB536_CF_IO_PA         0x16000000  /* I/O memory */
#define CSB536_CF_IO_VA         0xee000000  /* I/O memory */
#define CSB536_CF_ATTR_PA       0x16080000  /* Attribute memory */
#define CSB536_CF_ATTR_VA       0xee080000  /* Attribute memory */
#define CSB536_CF_MEM_PA        0x160c0000  /* Common memory */
#define CSB536_CF_MEM_VA        0xee0c0000  /* Common memory */

/* GPIO line assignment */
#define CSB536_GPIO_CD     7    /* Input */
#define CSB536_GPIO_RST    8    /* Output */
#define CSB536_GPIO_RDY    9    /* Input */
#define CSB536_GPIO_VEN   10    /* Output */


/* This is where we expect to find the IDE registers in IO and
 * memory mapped modes.
 */

# define CF_REGISTER_BASE    cf_common_memory

/* CF registers in attribute memory.  That registers begin at an offset
 * of 0x0200 into card memory.
 */

#define CF_CONFIG_OPTION_REG   (cf_attribute_memory + 0x00000200)
#define CF_CONFIG_STATUS_REG   (cf_attribute_memory + 0x00000202)
#define CF_PIN_REPLACEMENT_REG (cf_attribute_memory + 0x00000204)
#define CF_SOCKET_AND_COPY_REG (cf_attribute_memory + 0x00000206)

/* Delays (based only a few experiments) */

#define CF_POWER_DELAY  100  /* msec to delay after power applied */
#define CF_RESET_HOLD   100  /* msec to hold processor in reset */
#define CF_RESET_DELAY  500  /* msec delay after reset */
#define CF_MODE_DELAY   500  /* msec delay after CF mode changes */

#define IDE_DATA        0x00
#define IDE_ERROR       0x01 /* see err-bits */
#define IDE_NSECTOR     0x02 /* nr of sectors to read/write */
#define IDE_SECTOR      0x03 /* starting sector */
#define IDE_LCYL        0x04 /* starting cylinder */
#define IDE_HCYL        0x05 /* high byte of starting cyl */
#define IDE_SELECT      0x06 /* 101dhhhh , d=drive, hhhh=head */
#define IDE_STATUS      0x07 /* see status-bits */
#define IDE_CONTROL     0x0e /* control/altstatus */
#define IDE_IRQ         0x0f /* irq */

/**************************************************************************
 * Private Data
 **************************************************************************/

/* Mappings of the 36-bit physical addresses */

static unsigned long cf_io_memory;
static unsigned long cf_attribute_memory;
static unsigned long cf_common_memory;

/**************************************************************************
 * Common Private and Inline Functions
 **************************************************************************/

static inline void csb536_dump_memory(unsigned long addr, int nbytes)
{
#ifdef DEBUG
  int i, j;
  for (i = 0 ; i < nbytes; i += 16)
    {
      printk("  %08lx:", addr);
      for (j = 0 ; j < 16 && i+j < nbytes; j++)
	{
	  printk(" %02x", inb(addr));
	  addr++;
	}
      printk("\n");
    }
#endif
}

static inline void csb536_cf_memory(void)
{
#ifdef DEBUG
  printk("CF IO MEMORY:\n");
  csb536_dump_memory(cf_io_memory, 64);
  printk("CF ATTRIBUTE MEMORY:\n");
  csb536_dump_memory(cf_attribute_memory, 64);
  printk("  ...\n");
  csb536_dump_memory(cf_attribute_memory + 0x00000200, 64);
  printk("CF COMMON MEMORY:\n");
  csb536_dump_memory(cf_common_memory, 64);
#endif
}

static int csb536_ide_ack_intr(ide_hwif_t *hwif)
{
  dbg("\t[ -> csb536_ide_ack_intr <- ]\n");
//  return IRQ_HANDLED;
  return 0;
}

static int cfide_offsets[IDE_NR_PORTS] =
{
  IDE_DATA, IDE_ERROR,  IDE_NSECTOR, IDE_SECTOR, IDE_LCYL,
  IDE_HCYL, IDE_SELECT, IDE_STATUS,  IDE_CONTROL, IDE_IRQ
};

/**************************************************************************
 * Platform Customizations
 **************************************************************************/


static inline void csb536_cf_configure(void)
{
    
  /* All CSB655 PCMCIA/CF initialization is performed in
   * linux/arch/arm/mach-imx/csb536fs.c
   */
}

void csb536_cf_reset_pin(unsigned int on)
{
  if (on) {
    dbg("Setting CF RESET High\n");
    DR(3) |= (1 << CSB536_GPIO_RST);
  } else {
    dbg("Setting CF RESET Low\n");
    DR(3) &= ~(1 << CSB536_GPIO_RST);
  }
}

void csb536_cf_power_pin(unsigned int on)
{
  if (on) {
    dbg("Setting CF POWER High\n");
    DR(3) |= (1 << CSB536_GPIO_VEN);
  } else {
    dbg("Setting CF POWER Low\n");
    DR(3) &= ~(1 << CSB536_GPIO_VEN);
  }
}

int csb536_cf_cd_pin(void)
{
    unsigned long cd;
  
    cd = SSR(3) & (1 << CSB536_GPIO_CD);
    if (cd) {
        dbg("Compact flash not detected\n");
    } else {
        dbg("Compact flash detected\n");
    }

    return !cd;
}

void csb536_cf_reset(ide_drive_t *drive)
{
  dbg("Resetting CF\n");

  /* The CF spec defines that a card is resetted when the RESET
   * pin is high upon power-up. So we remove power to the card
   * drive RESET high , power the card, wait, and take the card
   * out of reset. (Note that in True-IDE mode the RESET pin is
   * active (~RESET)
   */

  /* Power done the card */
  dbg("POWER down the card\n");
  csb536_cf_power_pin(0);
  mdelay(CF_POWER_DELAY);

  /* Set RESET pin high */
  dbg("Set RESET High\n");
  csb536_cf_reset_pin(1);
  mdelay(CF_RESET_DELAY);

  /* power up the card */
  dbg("POWER up the card\n");
  csb536_cf_power_pin(1);
  mdelay(CF_POWER_DELAY);

  /* remove reset condition */
  dbg("Remove RESET\n");
  csb536_cf_reset_pin(0);
  mdelay(CF_RESET_DELAY);

  dbg("+Reset Done\n");
}


/**************************************************************************
 * Public Functions
 **************************************************************************/

# define CF_RDYBSY_IRQ		IRQ_GPIOD(CSB536_GPIO_RDY)
static inline int csb536_cf_init(void)
{
  ide_hwif_t    *hwif;
  ide_drive_t   *drive;
  hw_regs_t      hw;
  int            cf_idx;

  dbg("CSB536 CF Probe\n");

  dbg("Using CF_REGISTER_BASE 0x%08lx\n",CF_REGISTER_BASE);
  ide_setup_ports(&hw, CF_REGISTER_BASE, cfide_offsets, 0, 0, &csb536_ide_ack_intr, CF_RDYBSY_IRQ);

  /* Reset the CF card */

  csb536_cf_reset(NULL);

  /* Dump the initial state of CF registers */

  dbg("CF Config Option Reg(0x%08lx)=0x%02x\n",CF_CONFIG_OPTION_REG, inb(CF_CONFIG_OPTION_REG));
  dbg("CF Config/Status Reg(0x%08lx)=0x%02x\n",CF_CONFIG_STATUS_REG, inb(CF_CONFIG_STATUS_REG));
  dbg("CF Pin Replacement Reg(0x%08lx)=0x%02x\n",CF_PIN_REPLACEMENT_REG, inw(CF_PIN_REPLACEMENT_REG));
  dbg("CF Socket and Copy Reg(0x%08lx)=0x%02x\n",CF_SOCKET_AND_COPY_REG, inb(CF_SOCKET_AND_COPY_REG));

  /* Make sure that the CF card in is memory mapped mode by
   * writing 0x0000 to the Configuration Option Register.
   */

  dbg("Using Memory Mapped Mode\n");
  outb(0x00, CF_CONFIG_OPTION_REG);


  /* Dump the current state of CF registers */

  dbg("CF Config Option Reg(0x%08lx)=0x%02x\n",
      CF_CONFIG_OPTION_REG, inb(CF_CONFIG_OPTION_REG));
  dbg("CF Config/Status Reg(0x%08lx)=0x%02x\n",
      CF_CONFIG_STATUS_REG, inb(CF_CONFIG_STATUS_REG));
  dbg("CF Pin Replacement Reg(0x%08lx)=0x%02x\n",
      CF_PIN_REPLACEMENT_REG, inb(CF_PIN_REPLACEMENT_REG));
  dbg("CF Socket and Copy Reg(0x%08lx)=0x%02x\n",
      CF_SOCKET_AND_COPY_REG, inb(CF_SOCKET_AND_COPY_REG));

  udelay(CF_MODE_DELAY);

  /* Register the hardware interface.  NOTE:  The CF hardware
   * must be completely initialized at this point because if
   * ide_register_hw() is called AFTER initialization of the built-in,
   * drivers it will also probe the CF interface.
   */

  cf_idx = ide_register_hw(&hw, &hwif);
  if (cf_idx < 0)
    {
      err("Could not register CSB536 CompactFlash driver\n");
      return -ENODEV;
    }
    
  dbg("Using ide_hwifs[%d]\n", cf_idx);

  /* Provide function pointers for special handling */
  hwif->resetproc                       = csb536_cf_reset;


  /* We assume only one "drive," the master driver.
   * Probe only the master drive
   */

  drive = &hwif->drives[0];
  drive->noprobe                        = 0;
  hwif->drives[1].noprobe               = 1;

  /* This is the signature for a CF card */

  drive->removable                      = 1;
  drive->is_flash                       = 1;

  /* Dump all IDE registers */

  dbg("CF DATA(0x%08lx)         = 0x%04x\n",
      hwif->hw.io_ports[IDE_DATA_OFFSET],
      inw(hwif->hw.io_ports[IDE_DATA_OFFSET]));
  dbg("CF ERROR(0x%08lx)        = 0x%02x\n",
      hwif->hw.io_ports[IDE_ERROR_OFFSET],
      inb(hwif->hw.io_ports[IDE_ERROR_OFFSET]));
  dbg("CF NSECTOR(0x%08lx)      = 0x%02x\n",
      hwif->hw.io_ports[IDE_NSECTOR_OFFSET],
      inb(hwif->hw.io_ports[IDE_NSECTOR_OFFSET]));
  dbg("CF SECTOR(0x%08lx)       = 0x%02x\n",
      hwif->hw.io_ports[IDE_SECTOR_OFFSET],
      inb(hwif->hw.io_ports[IDE_SECTOR_OFFSET]));
  dbg("CF LCYL(0x%08lx)         = 0x%02x\n",
      hwif->hw.io_ports[IDE_LCYL_OFFSET],
      inb(hwif->hw.io_ports[IDE_LCYL_OFFSET]));
  dbg("CF HCYL(0x%08lx)         = 0x%02x\n",
      hwif->hw.io_ports[IDE_HCYL_OFFSET],
      inb(hwif->hw.io_ports[IDE_HCYL_OFFSET]));
  dbg("CF SELECT(0x%08lx)       = 0x%02x\n",
      hwif->hw.io_ports[IDE_SELECT_OFFSET],
      inb(hwif->hw.io_ports[IDE_SELECT_OFFSET]));
  dbg("CF STATUS(0x%08lx)       = 0x%02x\n",
      hwif->hw.io_ports[IDE_STATUS_OFFSET],
      inb(hwif->hw.io_ports[IDE_STATUS_OFFSET]));
  dbg("CF CONTROL(0x%08lx)      = 0x%02x\n",
      hwif->hw.io_ports[IDE_CONTROL_OFFSET],
      inb(hwif->hw.io_ports[IDE_CONTROL_OFFSET]));
  dbg("CF IRQ(0x%08lx)          = 0x%02x\n",
      hwif->hw.io_ports[IDE_IRQ_OFFSET],
      inb(hwif->hw.io_ports[IDE_IRQ_OFFSET]));

  csb536_cf_memory();

  /* Probe the hardware interface.  NOTE:  If ide_register_hw() is
   * called AFTER initialization of IDE built-in drivers, it will do
   * the following automatically.
   */

  probe_hwif_init(hwif);
  create_proc_ide_interfaces();

  return cf_idx;
}

void __init csb536_cf_probe(void)
{
  int cf_idx;

  printk("CSB536FS CompactFlash driver\n");

  if (!csb536_cf_cd_pin()) {
    printk("Slot Empty, No CompactFlash card inserted!\n\n");
    return;
  }

  /* One time initialization */

  /* Establish mappings for the CompactFlash address space.  The IDE
   * driver uses the inb/outb macros to access the IO registers. Since
   * mips_io_port_base is added to the access address, we need to
   * subtract it here.
   */

  dbg("Remmaping cf_io_memory\n");
  cf_io_memory = (unsigned long)CSB536_CF_IO_VA;

  dbg("CF IO memory mapped to 0x%08lx\n", cf_io_memory);

  dbg("Remmaping cf_attribute_memory\n");
  cf_attribute_memory = (unsigned long)CSB536_CF_ATTR_VA;

  dbg("CF attribute memory mapped to 0x%08lx\n", cf_attribute_memory);

  dbg("Remmaping cf_common_memory\n");
  cf_common_memory = (unsigned long)CSB536_CF_MEM_VA;

  dbg("CF common memory mapped to 0x%08lx\n",
      cf_common_memory);

  csb536_cf_memory();

  /* Configure CF interface as necessary for this platform */

  csb536_cf_configure();

  /* Initialization that must be peformed each time that a card
   * is inserted ... We do this now ONLY if CF detection logic has
   * not been enabled.
   */

  cf_idx = csb536_cf_init();
  if (cf_idx >= 0)
    {
      printk("CSB536 CompactFlash configured as device %i\n", cf_idx);
    }
}



module_init(csb536_cf_probe);
//module_exit(csb536_wm8731_exit);
