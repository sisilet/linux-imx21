/*
 * arch/arm/mach-at91rm9200/ics1523.c
 *
 *  Copyright (C) 2003 ATMEL Rousset
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
 */

#include <asm/hardware.h>
#include <linux/delay.h>

#include <asm/arch/ics1523.h>
#include <asm/arch/AT91RM9200_TWI.h>
#include <asm/arch/AT91RM9200_SYS.h>

//-----------------------------------------------------------------------------
//
// Initialization of TWI CLOCK
//
//-----------------------------------------------------------------------------

static void AT91F_SetTwiClock(unsigned int mck_khz)
{
	int sclock;

	/* Here, CKDIV = 1 and CHDIV = CLDIV  ==> CLDIV = CHDIV = 1/4*((Fmclk/FTWI) -6) */
	sclock = (10*mck_khz /ICS_TRANSFER_RATE);
	if (sclock % 10 >= 5)
		sclock = (sclock /10) - 5;
	else
		sclock = (sclock /10)- 6;
	sclock = (sclock + (4 - sclock %4)) >> 2;	// div 4

	((AT91PS_TWI)AT91C_VA_BASE_TWI)->TWI_CWGR = 0x00010000 | sclock | (sclock << 8);
}

//-----------------------------------------------------------------------------
//
// Read a byte with TWI Interface from the Clock Generator ICS1523
//
//-----------------------------------------------------------------------------

static int AT91F_ICS1523_ReadByte(unsigned char reg_address, unsigned char *data_in)
{
	AT91PS_TWI p_twi = (AT91PS_TWI)AT91C_VA_BASE_TWI;
	int Status, nb_trial ;

	p_twi->TWI_MMR = AT91C_TWI_MREAD | AT91C_TWI_IADRSZ_1_BYTE | ((ICS_ADD << 0x10) & AT91C_TWI_DADR);
	p_twi->TWI_IADR = (unsigned int) reg_address;
	p_twi->TWI_CR = (unsigned int) (AT91C_TWI_START | AT91C_TWI_STOP);

	// Program temporizing period (300us)
	udelay(300);

	// Wait TXcomplete ...
	nb_trial = 0;
	Status = p_twi->TWI_SR;
	while (!(Status & AT91C_TWI_TXCOMP) && (nb_trial < 10)) {
		nb_trial++;
		Status = p_twi->TWI_SR;
	}

	if (Status & AT91C_TWI_TXCOMP) {
		*data_in = (char) p_twi->TWI_RHR;
		return ((int) ICS1523_ACCESS_OK);
	}
	return ((int) ICS1523_ACCESS_ERROR);
}

//-----------------------------------------------------------------------------
//
// Write a byte with TWI Interface to the Clock Generator ICS1523
//
//-----------------------------------------------------------------------------

static int AT91F_ICS1523_WriteByte(unsigned char reg_address, unsigned char data_out)
{
	AT91PS_TWI p_twi = (AT91PS_TWI)AT91C_VA_BASE_TWI;
   	int Status, nb_trial;

	p_twi->TWI_MMR = (AT91C_TWI_MREAD & 0x0) | AT91C_TWI_IADRSZ_1_BYTE | ((ICS_ADD << 0x10) & AT91C_TWI_DADR);
	p_twi->TWI_IADR = (unsigned int) reg_address;
	p_twi->TWI_THR = (unsigned int) data_out;
	p_twi->TWI_CR = (unsigned int) (AT91C_TWI_START | AT91C_TWI_STOP);

	// Program temporizing period (300us)
	udelay(300);
	
	nb_trial = 0;
	Status = p_twi->TWI_SR;
	while (!( Status & AT91C_TWI_TXCOMP) && (nb_trial < 10)) {
		nb_trial++;
		if (Status & AT91C_TWI_ERROR) {
			// Si Under run OR NACK Start again
			p_twi->TWI_CR	= (unsigned int) (AT91C_TWI_START | AT91C_TWI_STOP);

			//  Program temporizing period (300us)
			udelay(300);
		}
		Status = p_twi->TWI_SR;
	};

	if (Status & AT91C_TWI_TXCOMP)
		return ((int) ICS1523_ACCESS_OK);
	else
		return ((int) ICS1523_ACCESS_ERROR);
}

//-----------------------------------------------------------------------------
//
// Initialization of the Clock Generator ICS1523
//
//-----------------------------------------------------------------------------

int AT91F_ICS1523_clockinit(void)
{
	int		ack,nb_trial,error_status;
	unsigned int	status = 0xffffffff;
	AT91PS_TWI	p_twi = (AT91PS_TWI)AT91C_VA_BASE_TWI;

	error_status = (int) ICS1523_ACCESS_OK;

	// Configure TWI PIOs
	AT91_SYS->PIOA_ASR = AT91C_PA25_TWD | AT91C_PA26_TWCK;
	AT91_SYS->PIOA_BSR = 0;
	AT91_SYS->PIOA_PDR = AT91C_PA25_TWD | AT91C_PA26_TWCK;

	// Configure the multi-drive option for TWD
	AT91_SYS->PIOA_MDDR = ~AT91C_PA25_TWD;
	AT91_SYS->PIOA_MDER = AT91C_PA25_TWD;

	// Configure PMC by enabling TWI clock
	AT91_SYS->PMC_PCER = ((unsigned int) 1 << AT91C_ID_TWI);

	// Configure TWI in master mode
	// Disable interrupts
	p_twi->TWI_IDR = (unsigned int) -1;

	// Reset peripheral
	p_twi->TWI_CR = AT91C_TWI_SWRST;

	// Set Master mode
	p_twi->TWI_CR = AT91C_TWI_MSEN | AT91C_TWI_SVDIS;

	// Set TWI Clock Waveform Generator Register
	AT91F_SetTwiClock(60000);     // MCK in KHz = 60000 KHz

	// ICS1523 Initialisation
	ack = AT91F_ICS1523_WriteByte ((unsigned char) ICS_ICR, (unsigned char) 0);
	error_status |= ack;
	ack = AT91F_ICS1523_WriteByte ((unsigned char) ICS_OE, (unsigned char) (ICS_OEF | ICS_OET2 | ICS_OETCK));
	error_status |= ack;
	ack = AT91F_ICS1523_WriteByte ((unsigned char) ICS_OD, (unsigned char) (ICS_INSEL | 0x7F));
	error_status |= ack;
	ack = AT91F_ICS1523_WriteByte ((unsigned char) ICS_DPAO, (unsigned char) 0);
	error_status |= ack;

	nb_trial = 0;
	do {
		nb_trial++;
		ack = AT91F_ICS1523_WriteByte ((unsigned char) ICS_ICR, (unsigned char) (ICS_ENDLS | ICS_ENPLS | ICS_PDEN /*| ICS_FUNCSEL*/));
		error_status |= ack;
		ack = AT91F_ICS1523_WriteByte ((unsigned char) ICS_LCR, (unsigned char) (ICS_PSD | ICS_PFD));
		error_status |= ack;
		ack = AT91F_ICS1523_WriteByte ((unsigned char) ICS_FD0, (unsigned char) 0x39) ; /* 0x7A */
		error_status |= ack;
		ack = AT91F_ICS1523_WriteByte ((unsigned char) ICS_FD1, (unsigned char) 0x00);
		error_status |= ack;
		ack = AT91F_ICS1523_WriteByte ((unsigned char) ICS_SWRST, (unsigned char) (ICS_PLLR));
		error_status |= ack;

		// Program 1ms temporizing period
		mdelay(1);

		AT91F_ICS1523_ReadByte ((unsigned char) ICS_SR, (char *)&status);
	} while (!((unsigned int) status & (unsigned int) ICS_PLLLOCK) && (nb_trial < 10));

	ack = AT91F_ICS1523_WriteByte ((unsigned char) ICS_DPAC, (unsigned char) 0x03) ; /* 0x01 */
	error_status |= ack;
	ack = AT91F_ICS1523_WriteByte ((unsigned char) ICS_SWRST, (unsigned char) (ICS_DPAR));
	error_status |= ack;

	/* Program 1ms temporizing period */
	mdelay(1);

	ack = AT91F_ICS1523_WriteByte ((unsigned char) ICS_DPAO, (unsigned char) 0x00);
	error_status |= ack;

	/* Program 1ms temporizing period */
	mdelay(1);

	return (error_status);
}
