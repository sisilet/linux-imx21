/*
 * Copyright (C) 2004 by Thomas Rathbone, HP Labs
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#ifndef AT91_UDC_H
#define AT91_UDC_H

#define	NUM_ENDPOINTS	6

#define udc_regs	((struct _AT91S_UDP __iomem *)AT91C_VA_BASE_UDP)

/* bugfixed flags from glb_state */
#define	AT91C_UDP_G_ESR		(1 << 2)
#define	AT91C_UDP_G_RWMUPE	(1 << 4)

#define	AT91C_UDP_STALLSENT	AT91C_UDP_ISOERROR

/* register */
#define USB_TXVC	(AT91C_VA_BASE_UDP + 0x74)
#define TXVDIS		(1 << 8)

#define	UDC_REG32(offset)	((u32 __iomem *)(AT91C_VA_BASE_UDP + (offset)))

static inline u8 __iomem *csrp_to_fifop(u32 __iomem *csrp)
{
	return &((u8 __iomem *)csrp)[0x20];
}


/*
 * hardware won't disable bus reset, or resume while the controller
 * is suspended ... watching suspend helps keep the logic symmetric.
 */
#define	MINIMUS_INTERRUPTUS \
	(AT91C_UDP_ENDBUSRES | AT91C_UDP_RXRSM | AT91C_UDP_RXSUSP)

struct at91_ep {
	struct usb_ep			ep;
	struct list_head		queue;
	struct at91_udc			*udc;
	u32 __iomem			*creg;

	unsigned			maxpacket:16;
	u8				int_mask;
	unsigned			is_pingpong:1;

	unsigned			stopped:1;
	unsigned			is_in:1;
	unsigned			is_iso:1;
	unsigned			fifo_bank:1;

	const struct usb_endpoint_descriptor
					*desc;
};

/*
 * driver is non-SMP, and just blocks IRQs whenever it needs
 * access protection for chip registers or driver state
 */
struct at91_udc {
	struct usb_gadget		gadget;
	struct at91_ep			ep[NUM_ENDPOINTS];
	struct usb_gadget_driver	*driver;
	unsigned			vbus:1;
	unsigned			enabled:1;
	unsigned			clocked:1;
	unsigned			suspended:1;
	unsigned			req_pending:1;
	unsigned			wait_for_addr_ack:1;
	unsigned			wait_for_config_ack:1;
	unsigned			selfpowered:1;
	u8				addr;
	struct at91_udc_data		board;
	struct clk			*iclk, *fclk;
	struct platform_device		*pdev;
	struct proc_dir_entry		*pde;
};

static inline struct at91_udc *to_udc(struct usb_gadget *g)
{
	return container_of(g, struct at91_udc, gadget);
}

struct at91_request {
	struct usb_request		req;
	struct list_head		queue;
};

/*-------------------------------------------------------------------------*/

#ifdef DEBUG
#define DBG(stuff...)		printk(KERN_DEBUG "udc: " stuff)
#else
#define DBG(stuff...)		do{}while(0)
#endif

#ifdef VERBOSE
#    define VDBG		DBG
#else
#    define VDBG(stuff...)	do{}while(0)
#endif

#ifdef PACKET_TRACE
#    define PACKET		VDBG
#else
#    define PACKET(stuff...)	do{}while(0)
#endif

#define ERR(stuff...)		printk(KERN_ERR "udc: " stuff)
#define WARN(stuff...)		printk(KERN_WARNING "udc: " stuff)
#define INFO(stuff...)		printk(KERN_INFO "udc: " stuff)

#endif

