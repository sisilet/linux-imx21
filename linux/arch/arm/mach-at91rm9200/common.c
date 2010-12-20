/*
 * arch/arm/mach-at91rm9200/common.c
 *
 *  Copyright (C) 2005 SAN People
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/config.h>
#include <linux/device.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

static struct map_desc at91rm9200_io_desc[] __initdata = {
	/* virtual,             physical,          length,   type */
	{ AT91C_VA_BASE_SYS,    AT91C_BASE_SYS,    SZ_4K,    MT_DEVICE},
	{ AT91C_VA_BASE_SPI,    AT91C_BASE_SPI,    SZ_16K,   MT_DEVICE},
	{ AT91C_VA_BASE_SSC2,   AT91C_BASE_SSC2,   SZ_16K,   MT_DEVICE},
	{ AT91C_VA_BASE_SSC1,   AT91C_BASE_SSC1,   SZ_16K,   MT_DEVICE},
	{ AT91C_VA_BASE_SSC0,   AT91C_BASE_SSC0,   SZ_16K,   MT_DEVICE},
	{ AT91C_VA_BASE_US3,    AT91C_BASE_US3,    SZ_16K,   MT_DEVICE},
	{ AT91C_VA_BASE_US2,    AT91C_BASE_US2,    SZ_16K,   MT_DEVICE},
	{ AT91C_VA_BASE_US1,    AT91C_BASE_US1,    SZ_16K,   MT_DEVICE},
	{ AT91C_VA_BASE_US0,    AT91C_BASE_US0,    SZ_16K,   MT_DEVICE},
	{ AT91C_VA_BASE_EMAC,   AT91C_BASE_EMAC,   SZ_16K,   MT_DEVICE},
	{ AT91C_VA_BASE_TWI,    AT91C_BASE_TWI,    SZ_16K,   MT_DEVICE},
	{ AT91C_VA_BASE_MCI,    AT91C_BASE_MCI,    SZ_16K,   MT_DEVICE},
	{ AT91C_VA_BASE_UDP,    AT91C_BASE_UDP,    SZ_16K,   MT_DEVICE},
	{ AT91C_VA_BASE_TCB1,   AT91C_BASE_TCB1,   SZ_16K,   MT_DEVICE},
	{ AT91C_VA_BASE_TCB0,   AT91C_BASE_TCB0,   SZ_16K,   MT_DEVICE},
};

void at91rm9200_map_io(void)
{
        iotable_init(at91rm9200_io_desc, ARRAY_SIZE(at91rm9200_io_desc));
}



