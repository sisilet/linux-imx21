/*
 *  linux/arch/arm/mach-imx21/generic.h
 *
 * Author:	Sascha Hauer <sascha@saschahauer.de>
 * Copyright:	Synertronixx GmbH
 *
 * Modified By: Ron Melvin (ron.melvin@timesys.com)
 * Copyright (C) 2005 TimeSys Corporation 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

extern void __init imx21_map_io(void);
extern void __init imx21_init_irq(void);

struct sys_timer;
extern struct sys_timer imx21_timer;
