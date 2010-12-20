/*
 * linux/include/asm-arm/arch-imx/csb536.h
 *
 * Copyright (C) 2005 Jay Monkman <jtm@lopingdog.com>
 * Modified from linux/include/asm-arm/arch-imx/mx1ads.h, original 
 * copyright below:
 *
 * Copyright (C) 2004 Robert Schwebel, Pengutronix
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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef __ASM_ARCH_CSB536_H
#define __ASM_ARCH_CSB536_H

/* ------------------------------------------------------------------------ */
/* Memory Map for the Cogent CSB536 Board                                   */
/* ------------------------------------------------------------------------ */

#define CLK32 32768

#define CSB536_ETH_VIRT IMX_CS1_VIRT
#define CSB536_ETH_PHYS IMX_CS1_PHYS
#define CSB536_ETH_SIZE IMX_CS1_SIZE
#define CSB536_ETH_IRQ  IRQ_GPIOB(17)

#define CSB536_UART2_VIRT IMX_CS3_VIRT
#define CSB536_UART2_PHYS IMX_CS3_PHYS
#define CSB536_UART2_SIZE IMX_CS3_SIZE
#define CSB536_UART2_IRQ  IRQ_GPIOA(12)

#define CSB536_UART3_VIRT IMX_CS4_VIRT
#define CSB536_UART3_PHYS IMX_CS4_PHYS
#define CSB536_UART3_SIZE IMX_CS4_SIZE
#define CSB536_UART3_IRQ  IRQ_GPIOA(13)
                        
#endif /* __ASM_ARCH_CSB536_H */
