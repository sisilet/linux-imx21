/*
 * Ethernet driver for the Atmel AT91RM9200 (Thunder)
 *
 *  Copyright (C) 2003 SAN People (Pty) Ltd
 *
 * Based on an earlier Atmel EMAC macrocell driver by Atmel and Lineo Inc.
 * Initial version by Rick Bronson 01/11/2003
 *
 * Intel LXT971A PHY support by Christopher Bahns & David Knickerbocker
 *   (Polaroid Corporation)
 *
 * Realtek RTL8201(B)L PHY support by Roman Avramenko <roman@imsystems.ru>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/config.h>
#include <linux/mii.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/skbuff.h>
#include <linux/dma-mapping.h>
#include <linux/crc32.h>
#include <linux/ethtool.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>

#include <asm/arch/AT91RM9200_EMAC.h>
#include <asm/arch/gpio.h>
#include <asm/arch/board.h>

#include "at91_ether.h"

#define DRV_NAME	"at91_ether"
#define DRV_VERSION	"1.0"

static struct net_device *at91_dev = NULL;
static struct timer_list check_timer;
#define LINK_POLL_INTERVAL  (HZ)

/* ........................... PHY INTERFACE ........................... */

/*
 * Enable the MDIO bit in MAC control register
 * When not called from an interrupt-handler, access to the PHY must be
 *  protected by a spinlock.
 */
static void enable_mdi(AT91PS_EMAC regs)
{
	regs->EMAC_CTL |= AT91C_EMAC_MPE;	/* enable management port */
}

/*
 * Disable the MDIO bit in the MAC control register
 */
static void disable_mdi(AT91PS_EMAC regs)
{
	regs->EMAC_CTL &= ~AT91C_EMAC_MPE;	/* disable management port */
}

/*
 * Write value to the a PHY register
 * Note: MDI interface is assumed to already have been enabled.
 */
static void write_phy(AT91PS_EMAC regs, unsigned char phy_addr, unsigned char address, unsigned int value)
{
	regs->EMAC_MAN = (AT91C_EMAC_HIGH | AT91C_EMAC_CODE_802_3 | AT91C_EMAC_RW_W
		| ((phy_addr & 0x1f) << 23) | (address << 18)) + (value & 0xffff);

	/* Wait until IDLE bit in Network Status register is cleared */
	// TODO: Enforce some maximum loop-count?
	while (!(regs->EMAC_SR & AT91C_EMAC_IDLE)) { barrier(); }
}

/*
 * Read value stored in a PHY register.
 * Note: MDI interface is assumed to already have been enabled.
 */
static void read_phy(AT91PS_EMAC regs, unsigned char phy_addr, unsigned char address, unsigned int *value)
{
	regs->EMAC_MAN = AT91C_EMAC_HIGH | AT91C_EMAC_CODE_802_3 | AT91C_EMAC_RW_R
		| ((phy_addr & 0x1f) << 23) | (address << 18);

	/* Wait until IDLE bit in Network Status register is cleared */
	// TODO: Enforce some maximum loop-count?
	while (!(regs->EMAC_SR & AT91C_EMAC_IDLE)) { barrier(); }

	*value = (regs->EMAC_MAN & 0x0000ffff);
}

/* ........................... PHY MANAGEMENT .......................... */

/*
 * Access the PHY to determine the current Link speed and Mode, and update the
 * MAC accordingly.
 * If no link or auto-negotiation is busy, then no changes are made.
 * Returns:  0 : OK
 *          -1 : No link
 *          -2 : AutoNegotiation still in progress
 */
static int update_linkspeed(struct net_device *dev, AT91PS_EMAC regs, int silent) {
	struct at91_private *lp = (struct at91_private *) dev->priv;
	unsigned int bmsr, bmcr, lpa, mac_cfg;
	unsigned int speed, duplex;

	/* Link status is latched, so read twice to get current value */
	read_phy(regs, lp->phy_address, MII_BMSR, &bmsr);
	read_phy(regs, lp->phy_address, MII_BMSR, &bmsr);
	if (!(bmsr & BMSR_LSTATUS))
		return -1;			/* no link */

	read_phy(regs, lp->phy_address, MII_BMCR, &bmcr);
	if (bmcr & BMCR_ANENABLE) {				/* AutoNegotiation is enabled */
		if (!(bmsr & BMSR_ANEGCOMPLETE)) return -2;	/* auto-negotitation in progress */

		read_phy(regs, lp->phy_address, MII_LPA, &lpa);
		if ((lpa & LPA_100FULL) || (lpa & LPA_100HALF)) speed = SPEED_100;
		else speed = SPEED_10;
		if ((lpa & LPA_100FULL) || (lpa & LPA_10FULL)) duplex = DUPLEX_FULL;
		else duplex = DUPLEX_HALF;
	} else {
		speed = (bmcr & BMCR_SPEED100) ? SPEED_100 : SPEED_10;
		duplex = (bmcr & BMCR_FULLDPLX) ? DUPLEX_FULL : DUPLEX_HALF;
	}

	/* Update the MAC */
	mac_cfg = regs->EMAC_CFG & ~(AT91C_EMAC_SPD | AT91C_EMAC_FD);
	if (speed == SPEED_100) {
		if (duplex == DUPLEX_FULL)		/* 100 Full Duplex */
			regs->EMAC_CFG = mac_cfg | AT91C_EMAC_SPD | AT91C_EMAC_FD;
		else					/* 100 Half Duplex */
			regs->EMAC_CFG = mac_cfg | AT91C_EMAC_SPD;
	} else {
		if (duplex == DUPLEX_FULL)		/* 10 Full Duplex */
			regs->EMAC_CFG = mac_cfg | AT91C_EMAC_FD;
		else					/* 10 Half Duplex */
			regs->EMAC_CFG = mac_cfg;
	}

	if (!silent)
		printk(KERN_INFO "%s: Link now %i-%s\n", dev->name, speed, (duplex == DUPLEX_FULL) ? "FullDuplex" : "HalfDuplex");
	return 0;
}

/*
 * Handle interrupts from the PHY
 */
static irqreturn_t at91ether_phy_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	struct net_device *dev = (struct net_device *) dev_id;
	struct at91_private *lp = (struct at91_private *) dev->priv;
	AT91PS_EMAC emac = (AT91PS_EMAC) dev->base_addr;
	int status;
	unsigned int phy;

	/*
	 * This hander is triggered on both edges, but the PHY chips expect
	 * level-triggering.  We therefore have to check if the PHY actually has
	 * an IRQ pending.
	 */
	enable_mdi(emac);
	if (lp->phy_type == MII_DM9161_ID) {
		read_phy(emac, lp->phy_address, MII_DSINTR_REG, &phy);	/* ack interrupt in Davicom PHY */
		if (!(phy & (1 << 0)))
			goto done;
	}
	else if (lp->phy_type == MII_LXT971A_ID) {
		read_phy(emac, lp->phy_address, MII_ISINTS_REG, &phy);	/* ack interrupt in Intel PHY */
		if (!(phy & (1 << 2)))
			goto done;
	}
	else if (lp->phy_type == MII_BCM5221_ID) {
		read_phy(emac, lp->phy_address, MII_BCMINTR_REG, &phy);	/* ack interrupt in Broadcom PHY */
		if (!(phy & (1 << 0)))
			goto done;
	}

	status = update_linkspeed(dev, emac, 0);
	if (status == -1) {			/* link is down */
		netif_carrier_off(dev);
		printk(KERN_INFO "%s: Link down.\n", dev->name);
	} else if (status == -2) {		/* auto-negotiation in progress */
		/* Do nothing - another interrupt generated when negotiation complete */
	} else {				/* link is operational */
		netif_carrier_on(dev);
	}

done:
	disable_mdi(emac);

	return IRQ_HANDLED;
}

static void check_link(unsigned long data)
{
	struct net_device *dev = (struct net_device *) data;
	AT91PS_EMAC emac = (AT91PS_EMAC) dev->base_addr;
	int status;
	
	enable_mdi(emac);
	status = update_linkspeed(dev, emac, 1);
	if (status == -1) {			/* link is down */
		netif_carrier_off(dev);
	} else if (status == -2) {		/* auto-negotiation in progress */
		/* Do nothing - another interrupt generated when negotiation complete */
	} else {				/* link is operational */
		netif_carrier_on(dev);
	}
	check_timer.expires = jiffies + LINK_POLL_INTERVAL;
	add_timer(&check_timer);
	disable_mdi(emac);
}

/*
 * Initialize and enable the PHY interrupt for link-state changes
 */
static void enable_phyirq(struct net_device *dev, AT91PS_EMAC regs)
{
	struct at91_private *lp = (struct at91_private *) dev->priv;
	unsigned int dsintr, irq_number;
	int status;

	if (!lp->board_data.phy_irq_pin) {
		init_timer(&check_timer);
		check_timer.data = (unsigned long) dev;
		check_timer.function = check_link;
		check_timer.expires = jiffies + LINK_POLL_INTERVAL;
		add_timer(&check_timer);
		return;
	}
	
	irq_number = lp->board_data.phy_irq_pin;
	status = request_irq(irq_number, at91ether_phy_interrupt, 0, dev->name, dev);
	if (status) {
		printk(KERN_ERR "at91_ether: PHY IRQ %d request failed - status %d!\n", irq_number, status);
		return;
	}

	spin_lock_irq(&lp->lock);
	enable_mdi(regs);

	if (lp->phy_type == MII_DM9161_ID) {		/* for Davicom PHY */
		read_phy(regs, lp->phy_address, MII_DSINTR_REG, &dsintr);
		dsintr = dsintr & ~0xf00;		/* clear bits 8..11 */
		write_phy(regs, lp->phy_address, MII_DSINTR_REG, dsintr);
	}
	else if (lp->phy_type == MII_LXT971A_ID) {	/* for Intel PHY */
		read_phy(regs, lp->phy_address, MII_ISINTE_REG, &dsintr);
		dsintr = dsintr | 0xf2;			/* set bits 1, 4..7 */
		write_phy(regs, lp->phy_address, MII_ISINTE_REG, dsintr);
	}
	else if (lp->phy_type == MII_BCM5221_ID) {	/* for Broadcom PHY */
		dsintr = (1 << 15) | ( 1 << 14);
		write_phy(regs, lp->phy_address, MII_BCMINTR_REG, dsintr);
	}

	disable_mdi(regs);
	spin_unlock_irq(&lp->lock);
}

/*
 * Disable the PHY interrupt
 */
static void disable_phyirq(struct net_device *dev, AT91PS_EMAC regs)
{
	struct at91_private *lp = (struct at91_private *) dev->priv;
	unsigned int dsintr;
	unsigned int irq_number;

	if (!lp->board_data.phy_irq_pin) {
		del_timer_sync(&check_timer);
		return;
	}

	spin_lock_irq(&lp->lock);
	enable_mdi(regs);

	if (lp->phy_type == MII_DM9161_ID) {		/* for Davicom PHY */
		read_phy(regs, lp->phy_address, MII_DSINTR_REG, &dsintr);
		dsintr = dsintr | 0xf00;			/* set bits 8..11 */
		write_phy(regs, lp->phy_address, MII_DSINTR_REG, dsintr);
	}
	else if (lp->phy_type == MII_LXT971A_ID) {	/* for Intel PHY */
		read_phy(regs, lp->phy_address, MII_ISINTE_REG, &dsintr);
		dsintr = dsintr & ~0xf2;			/* clear bits 1, 4..7 */
		write_phy(regs, lp->phy_address, MII_ISINTE_REG, dsintr);
	}
	else if (lp->phy_type == MII_BCM5221_ID) {	/* for Broadcom PHY */
		read_phy(regs, lp->phy_address, MII_BCMINTR_REG, &dsintr);
		dsintr = ~(1 << 14);
		write_phy(regs, lp->phy_address, MII_BCMINTR_REG, dsintr);
	}

	disable_mdi(regs);
	spin_unlock_irq(&lp->lock);

	irq_number = lp->board_data.phy_irq_pin;
	free_irq(irq_number, dev);			/* Free interrupt handler */
}

/*
 * Perform a software reset of the PHY.
 */
#if 0
static void reset_phy(struct net_device *dev, AT91PS_EMAC regs)
{
	struct at91_private *lp = (struct at91_private *) dev->priv;
	unsigned int bmcr;

	spin_lock_irq(&lp->lock);
	enable_mdi(regs);

	/* Perform PHY reset */
	write_phy(regs, lp->phy_address, MII_BMCR, BMCR_RESET);

	/* Wait until PHY reset is complete */
	do {
		read_phy(regs, lp->phy_address, MII_BMCR, &bmcr);
	} while (!(bmcr && BMCR_RESET));

	disable_mdi(regs);
	spin_unlock_irq(&lp->lock);
}
#endif

/* ......................... ADDRESS MANAGEMENT ........................ */

/*
 * NOTE: Your bootloader must always set the MAC address correctly before
 * booting into Linux.
 *
 * - It must always set the MAC address after reset, even if it doesn't
 *   happen to access the Ethernet while it's booting.  Some versions of
 *   U-Boot on the AT91RM9200-DK do not do this.
 *
 * - Likewise it must store the addresses in the correct byte order.
 *   MicroMonitor (uMon) on the CSB337 does this incorrectly (and
 *   continues to do so, for bug-compatibility).
 */

static short __init unpack_mac_address(struct net_device *dev, unsigned int hi, unsigned int lo)
{
	char addr[6];

	if (machine_is_csb337()) {
		addr[5] = (lo & 0xff);			/* The CSB337 bootloader stores the MAC the wrong-way around */
		addr[4] = (lo & 0xff00) >> 8;
		addr[3] = (lo & 0xff0000) >> 16;
		addr[2] = (lo & 0xff000000) >> 24;
		addr[1] = (hi & 0xff);
		addr[0] = (hi & 0xff00) >> 8;
	}
	else {
		addr[0] = (lo & 0xff);
		addr[1] = (lo & 0xff00) >> 8;
		addr[2] = (lo & 0xff0000) >> 16;
		addr[3] = (lo & 0xff000000) >> 24;
		addr[4] = (hi & 0xff);
		addr[5] = (hi & 0xff00) >> 8;
	}
	if (is_valid_ether_addr(addr)) {
		memcpy(dev->dev_addr, &addr, 6);
		return 1;
	}
	return 0;
}

/*
 * Set the ethernet MAC address in dev->dev_addr
 */
static void __init get_mac_address(struct net_device *dev)
{
	AT91PS_EMAC regs = (AT91PS_EMAC) dev->base_addr;

	if (unpack_mac_address(dev, regs->EMAC_SA1H, regs->EMAC_SA1L))		/* check Specific-Address 1 */
		return;
	if (unpack_mac_address(dev, regs->EMAC_SA2H, regs->EMAC_SA2L))		/* check Specific-Address 2 */
		return;
	if (unpack_mac_address(dev, regs->EMAC_SA3H, regs->EMAC_SA3L))		/* check Specific-Address 3 */
		return;
	if (unpack_mac_address(dev, regs->EMAC_SA4H, regs->EMAC_SA4L))		/* check Specific-Address 4 */
		return;

	printk(KERN_ERR "at91_ether: Your bootloader did not configure a MAC address.\n");
}

/*
 * Program the hardware MAC address from dev->dev_addr.
 */
static void update_mac_address(struct net_device *dev)
{
	AT91PS_EMAC regs = (AT91PS_EMAC) dev->base_addr;

	regs->EMAC_SA1L = (dev->dev_addr[3] << 24) | (dev->dev_addr[2] << 16) | (dev->dev_addr[1] << 8) | (dev->dev_addr[0]);
	regs->EMAC_SA1H = (dev->dev_addr[5] << 8) | (dev->dev_addr[4]);

	regs->EMAC_SA2L = 0;
	regs->EMAC_SA2H = 0;
}

/*
 * Store the new hardware address in dev->dev_addr, and update the MAC.
 */
static int set_mac_address(struct net_device *dev, void* addr)
{
	struct sockaddr *address = addr;

	if (!is_valid_ether_addr(address->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(dev->dev_addr, address->sa_data, dev->addr_len);
	update_mac_address(dev);

	printk("%s: Setting MAC address to %02x:%02x:%02x:%02x:%02x:%02x\n", dev->name,
		dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2],
		dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);

	return 0;
}

/*
 * Add multicast addresses to the internal multicast-hash table.
 */
static void at91ether_sethashtable(struct net_device *dev, AT91PS_EMAC regs)
{
	struct dev_mc_list *curr;
	unsigned char mc_filter[2];
	unsigned int i, bitnr;

	mc_filter[0] = mc_filter[1] = 0;

	curr = dev->mc_list;
	for (i = 0; i < dev->mc_count; i++, curr = curr->next) {
		if (!curr) break;	/* unexpected end of list */

		bitnr = ether_crc(ETH_ALEN, curr->dmi_addr) >> 26;
		mc_filter[bitnr >> 5] |= 1 << (bitnr & 31);
	}

	regs->EMAC_HSH = mc_filter[1];
	regs->EMAC_HSL = mc_filter[0];
}

/*
 * Enable/Disable promiscuous and multicast modes.
 */
static void at91ether_set_rx_mode(struct net_device *dev)
{
	AT91PS_EMAC regs = (AT91PS_EMAC) dev->base_addr;

	if (dev->flags & IFF_PROMISC) {			/* Enable promiscuous mode */
		regs->EMAC_CFG |= AT91C_EMAC_CAF;
	} else if (dev->flags & (~IFF_PROMISC)) {	/* Disable promiscuous mode */
		regs->EMAC_CFG &= ~AT91C_EMAC_CAF;
	}

	if (dev->flags & IFF_ALLMULTI) {		/* Enable all multicast mode */
		regs->EMAC_HSH = -1;
		regs->EMAC_HSL = -1;
		regs->EMAC_CFG |= AT91C_EMAC_MTI;
	} else if (dev->mc_count > 0) {			/* Enable specific multicasts */
		at91ether_sethashtable(dev, regs);
		regs->EMAC_CFG |= AT91C_EMAC_MTI;
	} else if (dev->flags & (~IFF_ALLMULTI)) {	/* Disable all multicast mode */
		regs->EMAC_HSH = 0;
		regs->EMAC_HSL = 0;
		regs->EMAC_CFG &= ~AT91C_EMAC_MTI;
	}
}

/* ............................... IOCTL ............................... */

static int mdio_read(struct net_device *dev, int phy_id, int location)
{
	AT91PS_EMAC regs = (AT91PS_EMAC) dev->base_addr;
	unsigned int value;

	read_phy(regs, phy_id, location, &value);
	return value;
}

static void mdio_write(struct net_device *dev, int phy_id, int location, int value)
{
	AT91PS_EMAC regs = (AT91PS_EMAC) dev->base_addr;

	write_phy(regs, phy_id, location, value);
}

/*
 * ethtool support.
 */
static int at91ether_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct at91_private *lp = (struct at91_private *) dev->priv;
	int ret;
	AT91PS_EMAC regs = (AT91PS_EMAC) dev->base_addr;
	
	spin_lock_irq(&lp->lock);
	enable_mdi(regs);
	ret = mii_ethtool_gset(&lp->mii, cmd);
	disable_mdi(regs);
	spin_unlock_irq(&lp->lock);
	if (lp->phy_media == PORT_FIBRE) {		/* override media type since mii.c doesn't know */
		cmd->supported = SUPPORTED_FIBRE;
		cmd->port = PORT_FIBRE;
	}
	
	return ret;
}

static int at91ether_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct at91_private *lp = (struct at91_private *) dev->priv;
	int ret;
	AT91PS_EMAC regs = (AT91PS_EMAC) dev->base_addr;

	spin_lock_irq(&lp->lock);
	enable_mdi(regs);
	ret = mii_ethtool_sset(&lp->mii, cmd);
	disable_mdi(regs);
	spin_unlock_irq(&lp->lock);

	return ret;
}

static int at91ether_nwayreset(struct net_device *dev)
{
	struct at91_private *lp = (struct at91_private *) dev->priv;
	int ret;
	AT91PS_EMAC regs = (AT91PS_EMAC) dev->base_addr;

	spin_lock_irq(&lp->lock);
	enable_mdi(regs);
	ret = mii_nway_restart(&lp->mii);
	disable_mdi(regs);
	spin_unlock_irq(&lp->lock);

	return ret;
}

static void at91ether_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, DRV_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_VERSION, sizeof(info->version));
	strlcpy(info->bus_info, dev->class_dev.dev->bus_id,
		sizeof(info->bus_info));
}

static struct ethtool_ops at91ether_ethtool_ops = {
	.get_settings	= at91ether_get_settings,
	.set_settings	= at91ether_set_settings,
	.get_drvinfo	= at91ether_get_drvinfo,

	.nway_reset	= at91ether_nwayreset,
	.get_link	= ethtool_op_get_link,
};

/* ................................ MAC ................................ */

/*
 * Initialize and start the Receiver and Transmit subsystems
 */
static void at91ether_start(struct net_device *dev)
{
	AT91PS_EMAC regs = (AT91PS_EMAC) dev->base_addr;
	struct at91_private *lp = (struct at91_private *) dev->priv;
	int i;
	struct recv_desc_bufs *dlist, *dlist_phys;

	dlist = lp->dlist;
	dlist_phys = lp->dlist_phys;

	for (i = 0; i < MAX_RX_DESCR; i++) {
		dlist->descriptors[i].addr = (unsigned int) &dlist_phys->recv_buf[i][0];
		dlist->descriptors[i].size = 0;
	}

	/* Set the Wrap bit on the last descriptor */
	dlist->descriptors[i-1].addr |= EMAC_DESC_WRAP;

	/* Reset buffer index */
	lp->rxBuffIndex = 0;

	/* Program address of descriptor list in Rx Buffer Queue register */
	regs->EMAC_RBQP = (AT91_REG) dlist_phys;

	/* Enable Receive and Transmit */
	regs->EMAC_CTL |= (AT91C_EMAC_RE | AT91C_EMAC_TE);
}

/*
 * Open the ethernet interface
 */
static int at91ether_open(struct net_device *dev)
{
	struct at91_private *lp = (struct at91_private *) dev->priv;
	AT91PS_EMAC regs = (AT91PS_EMAC) dev->base_addr;
	int link;

        if (!is_valid_ether_addr(dev->dev_addr))
        	return -EADDRNOTAVAIL;

	AT91_SYS->PMC_PCER = 1 << AT91C_ID_EMAC;	/* Re-enable Peripheral clock */
	regs->EMAC_CTL |= AT91C_EMAC_CSR;		/* Clear internal statistics */

	/* Update the MAC address (incase user has changed it) */
	update_mac_address(dev);

	/* Enable PHY interrupt */
	enable_phyirq(dev, regs);

	/* Enable MAC interrupts */
	regs->EMAC_IER = AT91C_EMAC_RCOM | AT91C_EMAC_RBNA
			| AT91C_EMAC_TUND | AT91C_EMAC_RTRY | AT91C_EMAC_TCOM
			| AT91C_EMAC_ROVR | AT91C_EMAC_HRESP;

	/* Determine current link speed */
	spin_lock_irq(&lp->lock);
	enable_mdi(regs);
	link = update_linkspeed(dev, regs, 0);
	if (link == -1) {				/* link is down */
		netif_carrier_off(dev);
		printk(KERN_INFO "%s: Link down.\n", dev->name);
	} else if (link == -2) {			/* auto-negotiation in progress */
		/* Do nothing - another interrupt generated when negotiation complete */
	} else {					/* link is operational */
		netif_carrier_on(dev);
	}
	disable_mdi(regs);
	spin_unlock_irq(&lp->lock);

	at91ether_start(dev);
	netif_start_queue(dev);
	return 0;
}

/*
 * Close the interface
 */
static int at91ether_close(struct net_device *dev)
{
	AT91PS_EMAC regs = (AT91PS_EMAC) dev->base_addr;

	/* Disable Receiver and Transmitter */
	regs->EMAC_CTL &= ~(AT91C_EMAC_TE | AT91C_EMAC_RE);

	/* Disable PHY interrupt */
	disable_phyirq(dev, regs);

	/* Disable MAC interrupts */
	regs->EMAC_IDR = AT91C_EMAC_RCOM | AT91C_EMAC_RBNA
			| AT91C_EMAC_TUND | AT91C_EMAC_RTRY | AT91C_EMAC_TCOM
			| AT91C_EMAC_ROVR | AT91C_EMAC_HRESP;

	netif_stop_queue(dev);

	AT91_SYS->PMC_PCDR = 1 << AT91C_ID_EMAC;	/* Disable Peripheral clock */

	return 0;
}

/*
 * Transmit packet.
 */
static int at91ether_tx(struct sk_buff *skb, struct net_device *dev)
{
	AT91PS_EMAC regs = (AT91PS_EMAC) dev->base_addr;
	struct at91_private *lp = (struct at91_private *) dev->priv;

	if (regs->EMAC_TSR & AT91C_EMAC_BNQ) {
		netif_stop_queue(dev);

		/* Store packet information (to free when Tx completed) */
		lp->skb = skb;
		lp->skb_length = skb->len;
		lp->skb_physaddr = dma_map_single(NULL, skb->data, skb->len, DMA_TO_DEVICE);
		lp->stats.tx_bytes += skb->len;

		/* Set address of the data in the Transmit Address register */
		regs->EMAC_TAR = lp->skb_physaddr;
		/* Set length of the packet in the Transmit Control register */
		regs->EMAC_TCR = skb->len;

		dev->trans_start = jiffies;
	} else {
		printk(KERN_ERR "at91_ether.c: at91ether_tx() called, but device is busy!\n");
		return 1;	/* if we return anything but zero, dev.c:1055 calls kfree_skb(skb)
				on this skb, he also reports -ENETDOWN and printk's, so either
				we free and return(0) or don't free and return 1 */
	}

	return 0;
}

/*
 * Update the current statistics from the internal statistics registers.
 */
static struct net_device_stats *at91ether_stats(struct net_device *dev)
{
	struct at91_private *lp = (struct at91_private *) dev->priv;
	AT91PS_EMAC regs = (AT91PS_EMAC) dev->base_addr;
	int ale, lenerr, seqe, lcol, ecol;

	if (netif_running(dev)) {
		lp->stats.rx_packets += regs->EMAC_OK;			/* Good frames received */
		ale = regs->EMAC_ALE;
		lp->stats.rx_frame_errors += ale;			/* Alignment errors */
		lenerr = regs->EMAC_ELR + regs->EMAC_USF;
		lp->stats.rx_length_errors += lenerr;			/* Excessive Length or Undersize Frame error */
		seqe = regs->EMAC_SEQE;
		lp->stats.rx_crc_errors += seqe;			/* CRC error */
		lp->stats.rx_fifo_errors += regs->EMAC_DRFC;		/* Receive buffer not available */
		lp->stats.rx_errors += (ale + lenerr + seqe + regs->EMAC_CDE + regs->EMAC_RJB);

		lp->stats.tx_packets += regs->EMAC_FRA;			/* Frames successfully transmitted */
		lp->stats.tx_fifo_errors += regs->EMAC_TUE;		/* Transmit FIFO underruns */
		lp->stats.tx_carrier_errors += regs->EMAC_CSE;		/* Carrier Sense errors */
		lp->stats.tx_heartbeat_errors += regs->EMAC_SQEE;	/* Heartbeat error */

		lcol = regs->EMAC_LCOL;
		ecol = regs->EMAC_ECOL;
		lp->stats.tx_window_errors += lcol;			/* Late collisions */
		lp->stats.tx_aborted_errors += ecol;			/* 16 collisions */

		lp->stats.collisions += (regs->EMAC_SCOL + regs->EMAC_MCOL + lcol + ecol);
	}
	return &lp->stats;
}

/*
 * Extract received frame from buffer descriptors and sent to upper layers.
 * (Called from interrupt context)
 */
static void at91ether_rx(struct net_device *dev)
{
	struct at91_private *lp = (struct at91_private *) dev->priv;
	struct recv_desc_bufs *dlist;
	unsigned char *p_recv;
	struct sk_buff *skb;
	unsigned int pktlen;

	dlist = lp->dlist;
	while (dlist->descriptors[lp->rxBuffIndex].addr & EMAC_DESC_DONE) {
		p_recv = dlist->recv_buf[lp->rxBuffIndex];
		pktlen = dlist->descriptors[lp->rxBuffIndex].size & 0x7ff;	/* Length of frame including FCS */
		skb = alloc_skb(pktlen + 2, GFP_ATOMIC);
		if (skb != NULL) {
			skb_reserve(skb, 2);
			memcpy(skb_put(skb, pktlen), p_recv, pktlen);

			skb->dev = dev;
			skb->protocol = eth_type_trans(skb, dev);
			skb->len = pktlen;
			dev->last_rx = jiffies;
			lp->stats.rx_bytes += pktlen;
			netif_rx(skb);
		}
		else {
			lp->stats.rx_dropped += 1;
			printk(KERN_NOTICE "%s: Memory squeeze, dropping packet.\n", dev->name);
		}

		if (dlist->descriptors[lp->rxBuffIndex].size & EMAC_MULTICAST)
			lp->stats.multicast++;

		dlist->descriptors[lp->rxBuffIndex].addr &= ~EMAC_DESC_DONE;	/* reset ownership bit */
		if (lp->rxBuffIndex == MAX_RX_DESCR-1)				/* wrap after last buffer */
			lp->rxBuffIndex = 0;
		else
			lp->rxBuffIndex++;
	}
}

/*
 * MAC interrupt handler
 */
static irqreturn_t at91ether_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	struct net_device *dev = (struct net_device *) dev_id;
	struct at91_private *lp = (struct at91_private *) dev->priv;
	AT91PS_EMAC emac = (AT91PS_EMAC) dev->base_addr;
	unsigned long intstatus;

	/* MAC Interrupt Status register indicates what interrupts are pending.
	   It is automatically cleared once read. */
	intstatus = emac->EMAC_ISR;

	if (intstatus & AT91C_EMAC_RCOM)		/* Receive complete */
		at91ether_rx(dev);

	if (intstatus & AT91C_EMAC_TCOM) {		/* Transmit complete */
		/* The TCOM bit is set even if the transmission failed. */
		if (intstatus & (AT91C_EMAC_TUND | AT91C_EMAC_RTRY))
			lp->stats.tx_errors += 1;

		if (lp->skb) {
			dev_kfree_skb_irq(lp->skb);
			lp->skb = NULL;
			dma_unmap_single(NULL, lp->skb_physaddr, lp->skb_length, DMA_TO_DEVICE);
		}
		netif_wake_queue(dev);
	}

	/* Work-around for Errata #11 */
	if (intstatus & AT91C_EMAC_RBNA) {
		emac->EMAC_CTL &= ~AT91C_EMAC_RE;
		emac->EMAC_CTL |= AT91C_EMAC_RE;
	}

	if (intstatus & AT91C_EMAC_ROVR)
		printk("%s: ROVR error\n", dev->name);

	return IRQ_HANDLED;
}

/*
 * Initialize the ethernet interface
 */
static int __init at91ether_setup(unsigned long phy_type, unsigned short phy_address, struct at91_eth_data *board_data)
{
	struct net_device *dev;
	struct at91_private *lp;
	AT91PS_EMAC regs;
	unsigned int val;
	int res;

	if (at91_dev)			/* already initialized */
		return 0;

	dev = alloc_etherdev(sizeof(struct at91_private));
	if (!dev)
		return -ENOMEM;

	dev->base_addr = AT91C_VA_BASE_EMAC;
	dev->irq = AT91C_ID_EMAC;
	SET_MODULE_OWNER(dev);

	/* Install the interrupt handler */
	if (request_irq(dev->irq, at91ether_interrupt, 0, dev->name, dev)) {
		free_netdev(dev);
		return -EBUSY;
	}

	/* Allocate memory for DMA Receive descriptors */
	lp = (struct at91_private *)dev->priv;
	lp->dlist = (struct recv_desc_bufs *) dma_alloc_coherent(NULL, sizeof(struct recv_desc_bufs), (dma_addr_t *) &lp->dlist_phys, GFP_KERNEL);
	if (lp->dlist == NULL) {
		free_irq(dev->irq, dev);
		free_netdev(dev);
		return -ENOMEM;
	}
	lp->board_data = *board_data;

	spin_lock_init(&lp->lock);

	ether_setup(dev);
	dev->open = at91ether_open;
	dev->stop = at91ether_close;
	dev->hard_start_xmit = at91ether_tx;
	dev->get_stats = at91ether_stats;
	dev->set_multicast_list = at91ether_set_rx_mode;
	dev->set_mac_address = set_mac_address;
	dev->ethtool_ops = &at91ether_ethtool_ops;

	get_mac_address(dev);		/* Get ethernet address and store it in dev->dev_addr */
	update_mac_address(dev);	/* Program ethernet address into MAC */

	regs = (AT91PS_EMAC) dev->base_addr;
	regs->EMAC_CTL = 0;

	if (lp->board_data.is_rmii)
		regs->EMAC_CFG = AT91C_EMAC_BIG | AT91C_EMAC_RMII;
	else
		regs->EMAC_CFG = AT91C_EMAC_BIG;

	if ((phy_type == MII_LXT971A_ID) || (phy_type == MII_RTL8201_ID) || (phy_type == MII_BCM5221_ID))
		regs->EMAC_CFG |= AT91C_EMAC_CLK_HCLK_64;	/* MDIO clock = system clock/64 */

	/* Perform PHY-specific initialization */
	spin_lock_irq(&lp->lock);
	enable_mdi(regs);
	if (phy_type == MII_DM9161_ID) {
		read_phy(regs, phy_address, MII_DSCR_REG, &val);
		if ((val & (1 << 10)) == 0)			/* DSCR bit 10 is 0 -- fiber mode */
			lp->phy_media = PORT_FIBRE;
	} else if (machine_is_csb337()) {
		/* mix link activity status into LED2 link state */
		write_phy(regs, phy_address, MII_LEDCTRL_REG, 0x0d22);
	}
	disable_mdi(regs);
	spin_unlock_irq(&lp->lock);

	lp->mii.dev = dev;		/* Support for ethtool */
	lp->mii.mdio_read = mdio_read;
	lp->mii.mdio_write = mdio_write;

	lp->phy_type = phy_type;	/* Type of PHY connected */
	lp->phy_address = phy_address;	/* MDI address of PHY */

	/* Register the network interface */
	res = register_netdev(dev);
	if (res) {
		free_irq(dev->irq, dev);
		free_netdev(dev);
		dma_free_coherent(NULL, sizeof(struct recv_desc_bufs), lp->dlist, (dma_addr_t)lp->dlist_phys);
		return res;
	}
	at91_dev = dev;

	/* Determine current link speed */
	spin_lock_irq(&lp->lock);
	enable_mdi(regs);
	(void) update_linkspeed(dev, regs, 0);
	disable_mdi(regs);
	spin_unlock_irq(&lp->lock);
	netif_carrier_off(dev);		/* will be enabled in open() */

	/* Display ethernet banner */
	printk(KERN_INFO "%s: AT91 ethernet at 0x%08x int=%d %s%s (%02x:%02x:%02x:%02x:%02x:%02x)\n",
		dev->name, (uint) dev->base_addr, dev->irq,
		regs->EMAC_CFG & AT91C_EMAC_SPD ? "100-" : "10-",
		regs->EMAC_CFG & AT91C_EMAC_FD ? "FullDuplex" : "HalfDuplex",
		dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2],
		dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);
	if (phy_type == MII_DM9161_ID)
		printk(KERN_INFO "%s: Davicom 9196 PHY %s\n", dev->name, (lp->phy_media == PORT_FIBRE) ? "(Fiber)" : "(Copper)");
	else if (phy_type == MII_LXT971A_ID)
		printk(KERN_INFO "%s: Intel LXT971A PHY\n", dev->name);
	else if (phy_type == MII_RTL8201_ID)
		printk(KERN_INFO "%s: Realtek RTL8201(B)L PHY\n", dev->name);
	else if (phy_type == MII_BCM5221_ID)
		printk(KERN_INFO "%s: Broadcom BCM5221 PHY\n", dev->name);
	else if (phy_type == MII_DP83847_ID)
		printk(KERN_INFO "%s: National Semiconductor DP83847 PHY\n", dev->name);

	return 0;
}

/*
 * Detect MAC and PHY and perform initialization
 */
static int __init at91ether_probe(struct device *dev)
{
	AT91PS_EMAC regs = (AT91PS_EMAC) AT91C_VA_BASE_EMAC;
	unsigned int phyid1, phyid2;
	int detected = -1;
	unsigned long phy_id;
	unsigned short phy_address = 0;

	AT91_SYS->PMC_PCER = 1 << AT91C_ID_EMAC;	/* Enable Peripheral clock */

	while ((detected != 0) && (phy_address < 32)) {
		/* Read the PHY ID registers */
		enable_mdi(regs);
		read_phy(regs, phy_address, MII_PHYSID1, &phyid1);
		read_phy(regs, phy_address, MII_PHYSID2, &phyid2);
		disable_mdi(regs);

		phy_id = (phyid1 << 16) | (phyid2 & 0xfff0);
		switch (phy_id) {
			case MII_DM9161_ID:		/* Davicom 9161: PHY_ID1 = 0x181, PHY_ID2 = B881 */
			case MII_LXT971A_ID:		/* Intel LXT971A: PHY_ID1 = 0x13, PHY_ID2 = 78E0 */
			case MII_RTL8201_ID:		/* Realtek RTL8201: PHY_ID1 = 0, PHY_ID2 = 0x8201 */
			case MII_BCM5221_ID:		/* Broadcom BCM5221: PHY_ID1 = 0x40, PHY_ID2 = 0x61e0 */
			case MII_DP83847_ID:		/* National Semiconductor DP83847:  */
				detected = at91ether_setup(phy_id, phy_address, dev->platform_data);
		}

		phy_address++;
	}

	AT91_SYS->PMC_PCDR = 1 << AT91C_ID_EMAC;	/* Disable Peripheral clock */

	return detected;
}

static int __devexit at91ether_remove(struct device *dev)
{
	struct at91_private *lp = (struct at91_private *) at91_dev->priv;

	unregister_netdev(at91_dev);
	free_irq(at91_dev->irq, at91_dev);
	dma_free_coherent(NULL, sizeof(struct recv_desc_bufs), lp->dlist, (dma_addr_t)lp->dlist_phys);

	free_netdev(at91_dev);
	at91_dev = NULL;
	return 0;
}

static struct device_driver at91ether_driver = {
	.name		= DRV_NAME,
	.bus		= &platform_bus_type,
	.owner		= THIS_MODULE,
	.probe		= at91ether_probe,
	.remove		= __devexit_p(at91ether_remove),
	/* FIXME:  support suspend and resume */
};

static int __init at91ether_init(void)
{
	return driver_register(&at91ether_driver);
}

static void __exit at91ether_exit(void)
{
	driver_unregister(&at91ether_driver);
}

module_init(at91ether_init)
module_exit(at91ether_exit)

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AT91RM9200 EMAC Ethernet driver");
MODULE_AUTHOR("Andrew Victor");
