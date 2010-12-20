#ifndef __SOUND_WM8731_H
#define __SOUND_WM8731_H

/*
 *  Copyright (c) by Jay Monkman <jtm@lopingdog.com>
 *
 *  Definitions for Wolfson WM8731 CODEC
 *
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/i2c.h>
#include "pcm.h"

int wm8731_write_reg(struct i2c_client *client, u8 reg, u16 val);
int wm8731_read_reg(struct i2c_client *client, u8 reg, u16 *val);

#define WM8731_REG_LLI 0  /* Left line in */
#define WM8731_REG_RLI 1  /* Right line out */
#define WM8731_REG_LHO 2  /* Left headphone out */
#define WM8731_REG_RHO 3  /* Right headphone out */
#define WM8731_REG_AAP 4  /* Analog audio path control */
#define WM8731_REG_DAP 5  /* Digital audio path control */
#define WM8731_REG_PDC 6  /* Power down control */
#define WM8731_REG_DAI 7  /* Digital audio interface format */
#define WM8731_REG_SMP 8  /* Sampling control */
#define WM8731_REG_ACT 9  /* Active control */
#define WM8731_REG_RST 15 /* Reset */

#define WM8731_LIN_VOL(x) ((x) & 0x1f)   /* Line in volume */
#define WM8731_LIN_MUTE   (1 << 7)       /* Line in mute */
#define WM8731_LIN_BOTH   (1 << 8)       /* Update both channels */

#define WM8731_HPO_VOL(x) ((x) & 0x7f)   /* Headphone volume */
#define WM8731_HPO_ZCEN   (1 << 7)       /* Zero crossing detect enable */
#define WM8731_HPO_BOTH   (1 << 8)       /* Update both channels */

#define WM8731_AAP_MICBOOST    (1 << 0)
#define WM8731_AAP_MUTEMIX     (1 << 1)
#define WM8731_AAP_INSEL       (1 << 2)
#define WM8731_AAP_BYPASS      (1 << 3)
#define WM8731_AAP_DACSEL      (1 << 4)
#define WM8731_AAP_SIDETONE    (1 << 5)
#define WM8731_AAP_SIDEATT15   (3 << 6)
#define WM8731_AAP_SIDEATT12   (2 << 6)
#define WM8731_AAP_SIDEATT9    (1 << 6)
#define WM8731_AAP_SIDEATT6    (0 << 6)

#define WM8731_DAP_ADCHPD      (1 << 0)
#define WM8731_DAP_DEEMP48     (3 << 1)
#define WM8731_DAP_DEEMP44     (2 << 1)
#define WM8731_DAP_DEEMP32     (1 << 1)
#define WM8731_DAP_DACMU       (1 << 3)
#define WM8731_DAP_HPOR        (1 << 4)

#define WM8731_PDC_LINEINPD    (1 << 0)
#define WM8731_PDC_MICPD       (1 << 1)
#define WM8731_PDC_ADCPD       (1 << 2)
#define WM8731_PDC_DACPD       (1 << 3)
#define WM8731_PDC_OUTPD       (1 << 4)
#define WM8731_PDC_OSCPD       (1 << 5)
#define WM8731_PDC_CLKOUTPD    (1 << 6)
#define WM8731_PDC_POWEROFF    (1 << 7)

#define WM8731_DAI_FMT_DSP     (3 << 0)
#define WM8731_DAI_FMT_I2S     (2 << 0)
#define WM8731_DAI_FMT_LEFT    (1 << 0)
#define WM8731_DAI_FMT_RIGHT   (0 << 0)
#define WM8731_DAI_IWL32       (3 << 2)
#define WM8731_DAI_IWL24       (2 << 2)
#define WM8731_DAI_IWL20       (1 << 2)
#define WM8731_DAI_IWL16       (0 << 2)
#define WM8731_DAI_LRP_HI      (1 << 4)
#define WM8731_DAI_LRP_LO      (0 << 4)
#define WM8731_DAI_LRP2        (1 << 4)
#define WM8731_DAI_LRP1        (0 << 4)
#define WM8731_DAI_LRSWAP      (1 << 5)
#define WM8731_DAI_MASTER      (1 << 6)
#define WM8731_DAI_BCLKINV     (1 << 7)

#define WM8731_SMP_USB         (1 << 0)
#define WM8731_SMP_BOSR        (1 << 1)
#define WM8731_SMP_SR(x)       (((x) & 0xf) << 2)
#define WM8731_SMP_CLKDIV2     (1 << 6)
#define WM8731_SMP_CLKODIV     (1 << 7)

#endif /* __SOUND_WM8731_H */
