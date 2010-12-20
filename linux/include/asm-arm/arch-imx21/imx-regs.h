#ifndef _MX2ADS_REGS_H
#define _MX2ADS_REGS_H
/* ------------------------------------------------------------------------
 *  Freescale i.iMX21 system registers
 * ------------------------------------------------------------------------
 *
 * Modified By: Ron Melvin (ron.melvin@timesys.com)
 * Copyright (C) 2005 TimeSys Corporation 
 *
 * Modified by: Stephen Donecker (sdonecker@sbcglobal.net)
 */

/*
 * Memory mapped I/O for the M9328MX21ADS    
 *
 * TODO: move these defines to an appropiate board specific file
 */

#define MX2ADS_MMIO_PHYS	(0xCC800000)
#define MX2ADS_MMIO_OFFSET	MX2ADS_MMIO_PHYS-IMX21_CS1_PHYS
#define MMIO_REG		(*((volatile u16 *)(IMX21_CS1_VIRT+MX2ADS_MMIO_OFFSET)))

#define MMIO_SD_WP		(1<<0)
#define MMIO_SW_SEL		(1<<1)
#define MMIO_RESET_E_UART	(1<<2)
#define MMIO_RESET_BASE		(1<<3)
#define MMIO_CSI_CTL2		(1<<4)
#define MMIO_CSI_CTL1		(1<<5)
#define MMIO_CSI_CTL0		(1<<6)
#define MMIO_UART1_EN		(1<<7)
#define MMIO_UART4_EN		(1<<8)
#define	MMIO_LCDON		(1<<9)
#define MMIO_IRDA_EN		(1<<10)
#define MMIO_IRDA_FIR_SEL	(1<<11)
#define MMIO_IRDA_MD0_B		(1<<12)
#define MMIO_IRDA_MD1		(1<<13)
#define MMIO_LED4_ON		(1<<14)
#define MMIO_LED3_ON		(1<<15)

/*
 *  Register BASEs, based on OFFSETs
 *
 */

/* TODO: Rename MX2ADS to processor specific name, not a board name */

#define MX2ADS_AIPI1_BASE             (0x00000 + IMX21_IO_BASE)
#define MX2ADS_AIPI2_BASE             (0x20000 + IMX21_IO_BASE)

#define MX2ADS_DMA_BASE               (0x01000 + IMX21_IO_BASE)
#define MX2ADS_WDT_BASE               (0x02000 + IMX21_IO_BASE)
#define MX2ADS_GPT1_BASE              (0x03000 + IMX21_IO_BASE)
#define MX2ADS_GPT2_BASE              (0x04000 + IMX21_IO_BASE)
#define MX2ADS_GPT3_BASE              (0x05000 + IMX21_IO_BASE)
#define MX2ADS_PWM_BASE               (0x06000 + IMX21_IO_BASE)
#define MX2ADS_RTC_BASE               (0x07000 + IMX21_IO_BASE)
#define MX2ADS_KPP_BASE               (0x08000 + IMX21_IO_BASE)
#define MX2ADS_OWIRE_BASE             (0x09000 + IMX21_IO_BASE)
#define MX2ADS_UART1_BASE             (0x0A000 + IMX21_IO_BASE)
#define MX2ADS_UART2_BASE             (0x0B000 + IMX21_IO_BASE)
#define MX2ADS_UART3_BASE             (0x0C000 + IMX21_IO_BASE)
#define MX2ADS_UART4_BASE             (0x0D000 + IMX21_IO_BASE)
#define MX2ADS_CSPI1_BASE             (0x0E000 + IMX21_IO_BASE)
#define MX2ADS_CSPI2_BASE             (0x0F000 + IMX21_IO_BASE)
#define MX2ADS_SSI1_BASE              (0x10000 + IMX21_IO_BASE)
#define MX2ADS_SSI2_BASE              (0x11000 + IMX21_IO_BASE)
#define MX2ADS_I2C_BASE               (0x12000 + IMX21_IO_BASE)
#define MX2ADS_SDHC1_BASE             (0x13000 + IMX21_IO_BASE)
#define MX2ADS_SDHC2_BASE             (0x14000 + IMX21_IO_BASE)
#define MX2ADS_GPIO_BASE              (0x15000 + IMX21_IO_BASE)
#define MX2ADS_AUDMUX_BASE            (0x16000 + IMX21_IO_BASE)
#define MX2ADS_CSPI3_BASE		(IMX21_IO_BASE + 0x17000)
#define MX2ADS_LCDC_BASE              (0x21000 + IMX21_IO_BASE)
#define MX2ADS_SLCDC_BASE             (0x22000 + IMX21_IO_BASE)
#define MX2ADS_SAHARA_BASE            (0x23000 + IMX21_IO_BASE)
#define MX2ADS_USBOTG_BASE            (0x24000 + IMX21_IO_BASE)
#define MX2ADS_EMMA_BASE              (0x26000 + IMX21_IO_BASE)
#define MX2ADS_CRM_BASE               (0x27000 + IMX21_IO_BASE)
#define MX2ADS_FIRI_BASE              (0x28000 + IMX21_IO_BASE)
#define MX2ADS_RNGA_BASE		(IMX21_IO_BASE + 0x29000)
#define MX2ADS_RTIC_BASE		(IMX21_IO_BASE + 0x2A000)
#define MX2ADS_JAM_BASE               (0x3E000 + IMX21_IO_BASE)
#define MX2ADS_MAX_BASE               (0x3F000 + IMX21_IO_BASE)
#define MX2ADS_AITC_BASE              (0x40000 + IMX21_IO_BASE)

#define GPIO_A	0
#define GPIO_B	1
#define GPIO_C	2
#define GPIO_D	3
#define GPIO_E	4
#define GPIO_F	5

/*
 *  GPIO Module and I/O Multiplexer
 *  x = 0..5 for reg_A, reg_B, reg_C, reg_D, reg_E, reg_F
 */

#define DDIR(x)    __REG2(MX2ADS_GPIO_BASE + 0x00, ((x) & 7) << 8)
#define OCR1(x)    __REG2(MX2ADS_GPIO_BASE + 0x04, ((x) & 7) << 8)
#define OCR2(x)    __REG2(MX2ADS_GPIO_BASE + 0x08, ((x) & 7) << 8)
#define ICONFA1(x) __REG2(MX2ADS_GPIO_BASE + 0x0c, ((x) & 7) << 8)
#define ICONFA2(x) __REG2(MX2ADS_GPIO_BASE + 0x10, ((x) & 7) << 8)
#define ICONFB1(x) __REG2(MX2ADS_GPIO_BASE + 0x14, ((x) & 7) << 8)
#define ICONFB2(x) __REG2(MX2ADS_GPIO_BASE + 0x18, ((x) & 7) << 8)
#define DR(x)      __REG2(MX2ADS_GPIO_BASE + 0x1c, ((x) & 7) << 8)
#define GIUS(x)    __REG2(MX2ADS_GPIO_BASE + 0x20, ((x) & 7) << 8)
#define SSR(x)     __REG2(MX2ADS_GPIO_BASE + 0x24, ((x) & 7) << 8)
#define ICR1(x)    __REG2(MX2ADS_GPIO_BASE + 0x28, ((x) & 7) << 8)
#define ICR2(x)    __REG2(MX2ADS_GPIO_BASE + 0x2c, ((x) & 7) << 8)
#define IMR(x)     __REG2(MX2ADS_GPIO_BASE + 0x30, ((x) & 7) << 8)
#define ISR(x)     __REG2(MX2ADS_GPIO_BASE + 0x34, ((x) & 7) << 8)
#define GPR(x)     __REG2(MX2ADS_GPIO_BASE + 0x38, ((x) & 7) << 8)
#define SWR(x)     __REG2(MX2ADS_GPIO_BASE + 0x3c, ((x) & 7) << 8)
#define PUEN(x)    __REG2(MX2ADS_GPIO_BASE + 0x40, ((x) & 7) << 8)
#define PMASK      __REG(MX2ADS_GPIO_BASE + 0x600)

/*  ethernet controller IRQ is tied to UART3_RTS */ 
#define NET_IRQ_BIT		(1 << 11)

/*
 *  GPIO Mode
 *
 *  The pin, port, data direction, pull-up enable, primary/alternate
 *  function, output configuration, and input configuration are encoded in a 
 *  single word as follows.
 *
 *  4:0 Pin (31-0)
 *  7:5 Port (F-A)
 *  8 Direction
 *  9 PUEN
 *  10:11 Primary Function,Alternate Function,GPIO
 *  13:12 OCR
 *  15:14 ICONF
 * 
 *  [ 15 14 | 13 12 | 11 10 | 9  |  8  | 7 6 5 | 4 3 2 1 0 ]
 *  [ ICONF |  OCR  | P/A/G | PU | Dir | Port  |    Pin    ]
 */

#define GPIO_PIN_MASK		(0x1f<<0)

#define GPIO_PORT_POS		5
#define GPIO_PORT_MASK		(0x3 << GPIO_PORT_POS)
#define GPIO_PORTA 		(0 << GPIO_PORT_POS)
#define GPIO_PORTB		(1 << GPIO_PORT_POS)
#define GPIO_PORTC		(2 << GPIO_PORT_POS)
#define GPIO_PORTD		(3 << GPIO_PORT_POS)
#define GPIO_PORTE		(4 << GPIO_PORT_POS)
#define GPIO_PORTF		(5 << GPIO_PORT_POS)

#define GPIO_DIR_MASK		(1<<8)
#define GPIO_IN			(0<<8)
#define GPIO_OUT		(1<<8)

#define GPIO_PU_MASK		(1<<9)
#define GPIO_PUDIS		(0<<9)
#define GPIO_PUEN		(1<<9)

#define GPIO_FUNC_MASK		(0x3<<10)
#define GPIO_PF			(0<<10)
#define GPIO_AF			(1<<10)
#define GPIO_GP			(2<<10)

#define GPIO_OCR_POS		12
#define GPIO_OCR_MASK		(0x3 << GPIO_OCR_POS)
#define GPIO_AIN		(0 << GPIO_OCR_POS)
#define GPIO_BIN		(1 << GPIO_OCR_POS)
#define GPIO_CIN		(2 << GPIO_OCR_POS)
#define GPIO_DR			(3 << GPIO_OCR_POS)

#define GPIO_ICONF_MASK		(0x3<<14)
#define GPIO_AOUT		(1<<14)
#define GPIO_BOUT		(2<<14)

/*
 *  Freescale i.MX21 GPIO signal multiplexing mode defines
 */

/*
 *  The GPIO pin naming convention was developed by the original unknown author. 
 *  Although using defines for variables is always a good idea for portability, 
 *  in this case the names are as specific as the values, and thus loose their 
 *  portability. Ultimately the pin names should be changed to reflect the signal
 *  name only.  
 *
 *  The current naming convention is as follows.
 *
 *  P(port)(pin)_(function)_(signal)
 *
 *  port = (A-F)
 *  pin = (0-31)
 *  function = (PF|AF|AIN|BIN|CIN|DR|AOUT|BOUT)
 *  signal = signal name from datasheet
 *
 *  Remember that when in GPIO mode, AIN, BIN, CIN, and DR are inputs to the GPIO
 *  peripheral module and represent outputs to the pin. Similarly AOUT, and BOUT
 *  are outputs from the GPIO peripheral module and represent inputs to the physical 
 *  pin in question. Refer to the multiplexing table in the section titled "Signal 
 *  Descriptions and Pin Assignments" in the reference manual.
 */
   
#define PE14_PF_UART1_CTS	( GPIO_PORTE | 14 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PE15_PF_UART1_RTS	( GPIO_PORTE | 15 | GPIO_PF | GPIO_IN | GPIO_PUDIS )
#define PE12_PF_UART1_TXD	( GPIO_PORTE | 12 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PE13_PF_UART1_RXD	( GPIO_PORTE | 13 | GPIO_PF | GPIO_IN | GPIO_PUDIS )

#define PA5_PF_LSCLK		( GPIO_PORTA | 5 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA6_PF_LD0		( GPIO_PORTA | 6 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA7_PF_LD1		( GPIO_PORTA | 7 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA8_PF_LD2		( GPIO_PORTA | 8 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA9_PF_LD3		( GPIO_PORTA | 9 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA10_PF_LD4		( GPIO_PORTA | 10 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA11_PF_LD5		( GPIO_PORTA | 11 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA12_PF_LD6		( GPIO_PORTA | 12 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA13_PF_LD7		( GPIO_PORTA | 13 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA14_PF_LD8		( GPIO_PORTA | 14 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA15_PF_LD9		( GPIO_PORTA | 15 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA16_PF_LD10		( GPIO_PORTA | 16 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA17_PF_LD11		( GPIO_PORTA | 17 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA18_PF_LD12		( GPIO_PORTA | 18 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA19_PF_LD13		( GPIO_PORTA | 19 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA20_PF_LD14		( GPIO_PORTA | 20 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA21_PF_LD15		( GPIO_PORTA | 21 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA22_PF_LD16		( GPIO_PORTA | 22 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA23_PF_LD17		( GPIO_PORTA | 23 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA24_PF_REV		( GPIO_PORTA | 24 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA25_PF_CLS		( GPIO_PORTA | 25 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA26_PF_PS		( GPIO_PORTA | 26 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA27_PF_SPL_SPR		( GPIO_PORTA | 27 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA28_PF_HSYNC		( GPIO_PORTA | 28 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA29_PF_VSYNC		( GPIO_PORTA | 29 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA30_PF_CONTRAST	( GPIO_PORTA | 30 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )
#define PA31_PF_OE_ACD		( GPIO_PORTA | 31 | GPIO_PF | GPIO_OUT | GPIO_PUDIS )

/*
 *  Freescale i.MX21 register defines
 *
 *  All register and field defines should conform to the following naming
 *  convention, and preferably use the exact names from the corresponding
 *  datasheets. 
 *
 *  #define <module>_<register>		<value>
 *  #define <register>_<field><type>	<value>
 *
 *  Where <type> can be one of the following.
 *
 *  _POS = defines the field LSB bit position	
 *  _MASK = defines the field mask
 *  (x) = writes x to the field location
 *
 *  The POS and MASK entries will be defined on an as needed basis. 
 */

/*
 *  Freescale i.MX21 clock controller registers
 */

#define CRM_CSCR	__REG(MX2ADS_CRM_BASE + 0x0)
#define CSCR_PRESC_MASK		(0x7<<29)
#define CSCR_PRESC(x)		(((x) & 0x7) << 29)
#define CSCR_USB_DIV_MASK	(0x7<<26)
#define CSCR_USB_DIV(x)		(((x) & 0x7) << 26)
#define CSCR_SD_CNT_MASK	(0x3<<24)
#define CSCR_SD_CNT(x)		(((x) & 0x3) << 24)
#define CSCR_SPLL_RESTART	(1<<22)
#define CSCR_MPLL_RESTART	(1<<21)
#define CSCR_SSI2_SEL		(1<<20)
#define CSCR_SSI1_SEL		(1<<19)
#define CSCR_FIR_SEL		(1<<18)
#define CSCR_SP_SEL		(1<<17)
#define CSCR_MCU_SEL		(1<<16)
#define CSCR_BCLKDIV_MASK	(0xf<<10)
#define CSCR_BCLKDIV(x)		(((x) & 0xf) << 10)
#define CSCR_IPDIV		(1<<9)
#define CSCR_OSC26M_DIV1P5	(1<<4)
#define CSCR_OSC28M_DIS		(1<<3)
#define CSCR_FPM_EN		(1<<2)
#define CSCR_SPEN		(1<<1)
#define CSCR_MPEN		(1<<0)

#define CRM_MPCTL0	__REG(MX2ADS_CRM_BASE + 0x4)
#define MPCTL0_CPLM		(1<<31)
#define MPCTL0_PD(x)		(((x) & 0xf) << 26)
#define MPCTL0_MFD(x)		(((x) & 0x3ff) << 16)
#define MPCTL0_MFI(x)		(((x) & 0xf) << 10)
#define MPCTL0_MFN(x)		(((x) & 0x3ff) << 0)

#define CRM_MPCTL1	__REG(MX2ADS_CRM_BASE + 0x8)
#define MPCTL1_LF		(1<<15)
#define MPCTL1_BRMO		(1<<6)

#define CRM_SPCTL0	__REG(MX2ADS_CRM_BASE + 0xc)
#define SPCTL0_CPLM		(1<<31)
#define SPCTL0_PD(x)		(((x) & 0xf) << 26)
#define SPCTL0_MFD(x)		(((x) & 0x3ff) << 16)
#define SPCTL0_MFI(x)		(((x) & 0xf) << 10)
#define SPCTL0_MFN(x)		(((x) & 0x3ff) << 0)

#define CRM_SPCTL1	__REG(MX2ADS_CRM_BASE + 0x10)
#define SPCTL1_LF		(1<<15)
#define SPCTL1_BRMO		(1<<6)

#define CRM_OSC26MCTL	__REG(MX2ADS_CRM_BASE + 0x14)
#define OSC26MCTL_OSC26M_PEAK	(0x2<<16)
#define OSC25MCTL_AGC(x)	(((x) & 0x3f) << 8)

#define CRM_PCDR0	__REG(MX2ADS_CRM_BASE + 0x18)
#define PCDR0_SSI2DIV(x)	(((x) & 0x3f) << 26)
#define PCDR0_SSI1DIV(x)	(((x) & 0x3f) << 16)
#define PCDR0_NFCDIV(x)		(((x) & 0xf) << 12)
#define PCDR0_CLKO_48MDIV(x)	(((x) & 0x7) << 5)
#define PCDR0_FIRI_DIV(x)	(((x) & 0x1f) << 0)
		
#define CRM_PCDR1	__REG(MX2ADS_CRM_BASE + 0x1c)
#define PCDR1_PERDIV4_POS	24
#define PCDR1_PERDIV4_MASK	(0x3f << PCDR1_PERDIV4_POS)
#define PCDR1_PERDIV4(x)	(((x) << PCDR1_PERDIV4_POS) & PCDR1_PERDIV4_MASK)
#define PCDR1_PERDIV3_POS	16
#define PCDR1_PERDIV3_MASK	(0x3f << PCDR1_PERDIV3_POS)
#define PCDR1_PERDIV3(x) 	(((x) << PCDR1_PERDIV3_POS) & PCDR1_PERDIV3_MASK)
#define PCDR1_PERDIV2_POS	8
#define PCDR1_PERDIV2_MASK	(0x3f << PCDR1_PERDIV2_POS)
#define PCDR1_PERDIV2(x) 	(((x) << PCDR1_PERDIV2_POS) & PCDR1_PERDIV2_MASK)
#define PCDR1_PERDIV1_POS	0
#define PCDR1_PERDIV1_MASK	(0x3f << PCDR1_PERDIV1_POS)
#define PCDR1_PERDIV1(x) 	(((x) << PCDR1_PERDIV1_POS) & PCDR1_PERDIV1_MASK)

#define CRM_PCCR0	__REG(MX2ADS_CRM_BASE + 0x20) 
#define PCCR0_HCLK_CSI_EN 	(1<<31)
#define PCCR0_HCLK_DMA_EN 	(1<<30)
#define PCCR0_HCLK_BROM_EN 	(1<<28)
#define PCCR0_HCLK_EMMA_EN 	(1<<27)
#define PCCR0_HCLK_LCDC_EN 	(1<<26)
#define PCCR0_HCLK_SLCDC_EN 	(1<<25)
#define PCCR0_HCLK_USBOTG_EN 	(1<<24)
#define PCCR0_HCLK_BMI_EN 	(1<<23)
#define PCCR0_PERCLK4_EN 	(1<<22)
#define PCCR0_SLCDC_EN	 	(1<<21)
#define PCCR0_FIRI_BAUD_EN 	(1<<20)
#define PCCR0_NFC_EN		(1<<19)
#define PCCR0_PERCLK3_EN 	(1<<18)
#define PCCR0_SSI1_BAUD_EN 	(1<<17)
#define PCCR0_SSI2_BAUD_EN 	(1<<16)
#define PCCR0_EMMA_EN 		(1<<15)
#define PCCR0_USBOTG_EN	 	(1<<14)
#define PCCR0_DMA_EN 		(1<<13)
#define PCCR0_I2C_EN 		(1<<12)
#define PCCR0_GPIO_EN	 	(1<<11)
#define PCCR0_SDHC2_EN	 	(1<<10)
#define PCCR0_SDHC1_EN	 	(1<<9)
#define PCCR0_FIRI_EN	 	(1<<8)
#define PCCR0_SSI2_EN	 	(1<<7)
#define PCCR0_SSI1_EN		(1<<6)
#define PCCR0_CSPI2_EN	 	(1<<5)
#define PCCR0_CSPI1_EN	 	(1<<4)
#define PCCR0_UART4_EN	 	(1<<3)
#define PCCR0_UART3_EN	 	(1<<2)
#define PCCR0_UART2_EN 		(1<<1)
#define PCCR0_UART1_EN	 	(1<<0)

#define CRM_PCCR1	__REG(MX2ADS_CRM_BASE + 0x24)
#define PCCR1_OWIRE_EN		(1<<31)
#define PCCR1_KPP_EN		(1<<30)
#define PCCR1_RTC_EN		(1<<29)
#define PCCR1_PWM_EN		(1<<28)
#define PCCR1_GPT3_EN		(1<<27)
#define PCCR1_GPT2_EN		(1<<26)
#define PCCR1_GPT1_EN		(1<<25)
#define PCCR1_WDT_EN		(1<<24)
#define PCCR1_CSPI3_EN		(1<<23)
#define PCCR1_RTIC_EN		(1<<22)
#define PCCR1_RNGA_EN		(1<<21)

#define CRM_CCSR	__REG(MX2ADS_CRM_BASE + 0x28)
#define CCSR_32K_SR		(1<<15)
#define CCSR_CLK0_SEL(x)	(((x) & 0x1f) << 0)
	
#define CRM_WKGDCTL	__REG(MX2ADS_CRM_BASE + 0x34)
#define WKGDCTL_WKDG_EN		(1<<0)

/*
 * LCD controller (LCDC) registers
 */

#define LCDC_LSSAR	__REG(MX2ADS_LCDC_BASE + 0x0)
#define LSSAR_SSA(x)		(((x) & 0x3fffffff) << 2)

#define LCDC_LSR 	__REG(MX2ADS_LCDC_BASE + 0x4)
#define LSR_XMAX(x)		(((x) & 0x3f) << 20)
#define LSR_YMAX(x)		(((x) & 0x3ff) << 0)

#define LCDC_LVPWR 	__REG(MX2ADS_LCDC_BASE + 0x8)
#define LVPWR_VPW(x)		(((x) & 0x3ff) << 0)

#define LCDC_LCPR 	__REG(MX2ADS_LCDC_BASE + 0xc)
#define LCPR_CC(x)		(((x) & 0x3) << 30)
#define LCPR_OP			(1<<28)
#define LCPR_CXP(x)		(((x) & 0x3ff) << 16)
#define LCPR_CYP(x)		(((x) & 0x3ff) << 0)

#define LCPR_CC_DISABLED	0
#define LCPR_CC_OR		1
#define LCPR_CC_XOR		2
#define LCPR_CC_AND		3

#define LCDC_LCWHBR 	__REG(MX2ADS_LCDC_BASE + 0x10)
#define LCWHBR_BK_EN		(1<<31)
#define LCWHBR_CW(x)		(((x) & 0x1f) << 24)
#define LCWHBR_CH(x)		(((x) & 0x1f) << 16)
#define LCWHBR_BD(x)		(((x) & 0xff) << 0)

#define LCDC_LCCMR 	__REG(MX2ADS_LCDC_BASE + 0x14)
#define LCCMR_CUR_COL_R(x)	(((x) & 0x3f) << 12)
#define LCCMR_CUR_COL_G(x)	(((x) & 0x3f) << 6)
#define LCCMR_CUR_COL_B(x)	(((x) & 0x3f) << 0)

#define LCDC_LPCR 	__REG(MX2ADS_LCDC_BASE + 0x18)
#define LPCR_TFT		(1<<31)
#define LPCR_COLOR		(1<<30)
#define LPCR_PBSIZ_POS		28
#define LPCR_PBSIZ_MASK		(0x3 << LPCR_PBSIZ_POS)
#define LPCR_PBSIZ(x)		(((x) << LPCR_PBSIZ_POS) & LPCR_PBSIZ_MASK)
#define LPCR_BPIX_POS		25
#define LPCR_BPIX_MASK		(0x7 << LPCR_BPIX_POS)
#define LPCR_BPIX(x)		(((x) << LPCR_BPIX_POS) & LPCR_BPIX_MASK)
#define LPCR_PIXPOL		(1<<24)
#define LPCR_FLMPOL		(1<<23)
#define LPCR_LPPOL		(1<<22)
#define LPCR_CLKPOL		(1<<21)
#define LPCR_OEPOL		(1<<20)
#define LPCR_SCLKIDLE		(1<<19)
#define LPCR_END_SEL		(1<<18)
#define LPCR_SWAP_SEL		(1<<17)
#define LPCR_REV_VS		(1<<16)
#define LPCR_ACDSEL		(1<<15)
#define LPCR_ACD(x)		(((x) & 0x7f) << 8)
#define LPCR_SCLKSEL		(1<<7)
#define LPCR_SHARP		(1<<6)
#define LPCR_PCD(x)		(((x) & 0x3f) << 0)

#define LPCR_PBSIZ_PANEL_BUS_WIDTH_1	0
#define LPCR_PBSIZ_PANEL_BUS_WIDTH_4	2
#define LPCR_PBSIZ_PANEL_BUS_WIDTH_8	3

#define LPCR_BPIX_BITS_PER_PIXEL_1	0
#define LPCR_BPIX_BITS_PER_PIXEL_2	1
#define LPCR_BPIX_BITS_PER_PIXEL_4	2
#define LPCR_BPIX_BITS_PER_PIXEL_8	3
#define LPCR_BPIX_BITS_PER_PIXEL_12	4
#define LPCR_BPIX_BITS_PER_PIXEL_16	5
#define LPCR_BPIX_BITS_PER_PIXEL_18	6

#define LCDC_LHCR 	__REG(MX2ADS_LCDC_BASE + 0x1c)
#define LHCR_H_WIDTH(x)		(((x) & 0x3f) << 26)
#define LHCR_H_WAIT_1(x)	(((x) & 0x1ff) << 8)
#define LHCR_H_WAIT_2(x)	(((x) & 0x1ff) << 0)

#define LCDC_LVCR 	__REG(MX2ADS_LCDC_BASE + 0x20)
#define LVCR_V_WIDTH(x)		(((x) & 0x3f) << 26)
#define LVCR_V_WAIT_1(x)	(((x) & 0x1ff) << 8)
#define LVCR_V_WAIT_2(x)	(((x) & 0x1ff) << 0)

#define LCDC_LPOR 	__REG(MX2ADS_LCDC_BASE + 0x24)
#define LPOR_POS(x)		(((x) & 0x1f) << 0)

#define LCDC_LSCR 	__REG(MX2ADS_LCDC_BASE + 0x28)
#define LSCR_PS_RISE_DELAY(x)		(((x) & 0x3f) << 26)
#define LSCR_CLS_RISE_DELAY(x)		(((x) & 0xff) << 16)
#define LSCR_REV_TOGGLE_DELAY(x)	(((x) & 0xf) << 8)
#define LSCR_GRAY2(x)			(((x) & 0xf) << 4)
#define LSCR_GRAY1(x)			(((x) & 0xf) << 0)

#define LCDC_LPCCR 	__REG(MX2ADS_LCDC_BASE + 0x2c)
#define LPCCR_CLS_HI_WIDTH(x)	(((x) & 0x1ff) << 16)
#define LPCCR_LDMSK		(1<<15)
#define LPCCR_SCR(x)		(((x) & 0x3) << 9)
#define LPCCR_CC_EN		(1<<8)
#define LPCCR_PW(x)		(((x) & 0xff) << 0)

#define LPCCR_SCR_LINE_PULSE	0
#define LPCCR_SCR_PIXEL_CLOCK	1
#define LPCCR_SCR_LCD_CLOCK	2

#define LCDC_LDCR 	__REG(MX2ADS_LCDC_BASE + 0x30)
#define LDCR_BURST		(1<<31)
#define LDCR_HM(x)		(((x) & 0x1f) << 16)
#define LDCR_TM(x)		(((x) & 0x1f) << 0)

#define LCDC_LRMCR 	__REG(MX2ADS_LCDC_BASE + 0x34)
#define LRMCR_SELF_REF		(1<<0)

#define LCDC_LICR 	__REG(MX2ADS_LCDC_BASE + 0x38)
#define LICR_GW_INIT_CON	(1<<4)
#define LICR_INTSYN		(1<<2)
#define LICR_INTCON		(1<<0)

#define LCDC_LIER 	__REG(MX2ADS_LCDC_BASE + 0x3c)
#define LIER_GW_UDR_ERR_EN	(1<<7)
#define LIER_GW_ERR_RES_EN	(1<<6)
#define LIER_GW_EOF_EN		(1<<5)
#define LIER_GW_BOF_EN		(1<<4)
#define LIER_UDR_ERR_EN		(1<<3)
#define LIER_ERR_RES_EN		(1<<2)
#define LIER_EOF_EN		(1<<1)
#define LIER_BOF_EN		(1<<0)

#define LCDC_LISR 	__REG(MX2ADS_LCDC_BASE + 0x40)
#define LISR_GW_UDR_ERR_MASK	(1<<7)
#define LISR_GW_ERR_RES_MASK	(1<<6)
#define LISR_GW_EOF_MASK	(1<<5)
#define LISR_GW_BOF_MASK	(1<<4)
#define LISR_UDR_ERR_MASK	(1<<3)
#define LISR_ERR_RES_MASK	(1<<2)
#define LISR_EOF_MASK		(1<<1)
#define LISR_BOF_MASK		(1<<0)

#define LCDC_LGWSAR 	__REG(MX2ADS_LCDC_BASE + 0x50)
#define LGWSAR_GWSA(x)		(((x) & 0x3fffffff) << 2)

#define LCDC_LGWSR 	__REG(MX2ADS_LCDC_BASE + 0x54)
#define LGWSR_GWW(x)		(((x) & 0x3f) << 20)
#define LGWSR_GWH(x)		(((x) & 0x3fff) << 0)

#define LCDC_LGWVPWR 	__REG(MX2ADS_LCDC_BASE + 0x58)
#define LGWVPWR_GWVPW(x)	(((x) & 0x3ff) << 0)

#define LCDC_LGWPOR 	__REG(MX2ADS_LCDC_BASE + 0x5c)
#define LGWPOR(x)		(((x) & 0x1f) << 0)

#define LCDC_LGWPR 	__REG(MX2ADS_LCDC_BASE + 0x60)
#define LGWPR_GWXP(x)		(((x) & 0x3ff) << 16)
#define LGWPR_GWYP(x)		(((x) & 0x3ff) << 0)

#define LCDC_LGWCR 	__REG(MX2ADS_LCDC_BASE + 0x64)
#define LGWCR_GWAV(x)		(((x) & 0xff) << 24)
#define LGWCR_GWCKE		(1<<23)
#define LGWCR_GWE		(1<<22)
#define LGWCR_GW_RVS		(1<<21)
#define LGWCR_GWCKR(x)		(((x) & 0x3f) << 12)
#define LGWCR_GWCKG(x)		(((x) & 0x3f) << 6)
#define LGWCR_GWCKB(x)		(((x) & 0x3f) << 0)

#define LCDC_LGWDCR 	__REG(MX2ADS_LCDC_BASE + 0x68)
#define LGWDCR_GWBT		(1<<31)
#define LGWDCR_GWHM(x)		(((x) & 0x1f) << 16)
#define LGWDCR_GWTM(x)		(((x) & 0x1f) << 0)

/*
 *  General purpose timers
 */

#define WHICH_TIMER(x)     ((x)*0x1000)
#define IMX21_TCTL(x)        __REG( 0x00 + MX2ADS_GPT1_BASE + WHICH_TIMER(x) )
#define TCTL_SWR           (1<<15)
#define TCTL_FRR           (1<<8)
#define TCTL_CAP_RIS       (1<<6)
#define TCTL_CAP_FAL       (2<<6)
#define TCTL_CAP_RIS_FAL   (3<<6)
#define TCTL_OM            (1<<5)
#define TCTL_IRQEN         (1<<4)
#define TCTL_CLK_PCLK1     (1<<1)
#define TCTL_CLK_PCLK1_16  (2<<1)
#define TCTL_CLK_TIN       (3<<1)
#define TCTL_CLK_32        (4<<1)
#define TCTL_TEN           (1<<0)

#define IMX21_TPRER(x)       __REG( 0x04 + MX2ADS_GPT1_BASE + WHICH_TIMER(x))
#define IMX21_TCMP(x)        __REG( 0x08 + MX2ADS_GPT1_BASE + WHICH_TIMER(x))
#define IMX21_TCR(x)         __REG( 0x0C + MX2ADS_GPT1_BASE + WHICH_TIMER(x))
#define IMX21_TCN(x)         __REG( 0x10 + MX2ADS_GPT1_BASE + WHICH_TIMER(x))
#define IMX21_TSTAT(x)       __REG( 0x14 + MX2ADS_GPT1_BASE + WHICH_TIMER(x))
#define TSTAT_CAPT         (1<<1)
#define TSTAT_COMP         (1<<0)

#define GPT1		0
#define GPT2		1
#define GPT3		2

/*
 *  Uart controller registers 
 */

#define UART_1  0
#define UART_2  1
#define UART_3  2
#define UART_4  3


#define URXD0(x) __REG( 0x0 + (x)) /* Receiver Register */
#define URTX0(x) __REG( 0x40 + (x)) /* Transmitter Register */
#define UCR1(x)  __REG( 0x80 + (x)) /* Control Register 1 */
#define UCR2(x)  __REG( 0x84 + (x)) /* Control Register 2 */
#define UCR3(x)  __REG( 0x88 + (x)) /* Control Register 3 */
#define UCR4(x)  __REG( 0x8c + (x)) /* Control Register 4 */
#define UFCR(x)  __REG( 0x90 + (x)) /* FIFO Control Register */
#define USR1(x)  __REG( 0x94 + (x)) /* Status Register 1 */
#define USR2(x)  __REG( 0x98 + (x)) /* Status Register 2 */
#define UESC(x)  __REG( 0x9c + (x)) /* Escape Character Register */
#define UTIM(x)  __REG( 0xa0 + (x)) /* Escape Timer Register */
#define UBIR(x)  __REG( 0xa4 + (x)) /* BRM Incremental Register */
#define UBMR(x)  __REG( 0xa8 + (x)) /* BRM Modulator Register */
#define UBRC(x)  __REG( 0xac + (x)) /* Baud Rate Count Register */
#define ONMES(x) __REG( 0xb0 + (x)) /* One Millisecond Register */
#define UTS(x)   __REG( 0xb4 + (x)) /* UART Test Register */



/* UART Control Register Bit Fields.*/
#define  URXD_CHARRDY    (1<<15)
#define  URXD_ERR        (1<<14)
#define  URXD_OVRRUN     (1<<13)
#define  URXD_FRMERR     (1<<12)
#define  URXD_BRK        (1<<11)
#define  URXD_PRERR      (1<<10)

#define  UCR1_ADEN       (1<<15) /* Auto dectect interrupt */
#define  UCR1_ADBR       (1<<14) /* Auto detect baud rate */
#define  UCR1_TRDYEN     (1<<13) /* Transmitter ready interrupt enable */
#define  UCR1_IDEN       (1<<12) /* Idle condition interrupt */
#define  UCR1_RRDYEN     (1<<9)	 /* Recv ready interrupt enable */
#define  UCR1_RDMAEN     (1<<8)	 /* Recv ready DMA enable */
#define  UCR1_IREN       (1<<7)	 /* Infrared interface enable */
#define  UCR1_TXMPTYEN   (1<<6)	 /* Transimitter empty interrupt enable */
#define  UCR1_RTSDEN     (1<<5)	 /* RTS delta interrupt enable */
#define  UCR1_SNDBRK     (1<<4)	 /* Send break */
#define  UCR1_TDMAEN     (1<<3)	 /* Transmitter ready DMA enable */
// not on mx21 #define  UCR1_UARTCLKEN  (1<<2)	 /* UART clock enabled */
#define  UCR1_DOZE       (1<<1)	 /* Doze */
#define  UCR1_UARTEN     (1<<0)	 /* UART enabled */

#define  UCR2_ESCI     	 (1<<15) /* Escape seq interrupt enable */
#define  UCR2_IRTS  	 (1<<14) /* Ignore RTS pin */
#define  UCR2_CTSC  	 (1<<13) /* CTS pin control */
#define  UCR2_CTS        (1<<12) /* Clear to send */
#define  UCR2_ESCEN      (1<<11) /* Escape enable */
#define  UCR2_PREN       (1<<8)  /* Parity enable */
#define  UCR2_PROE       (1<<7)  /* Parity odd/even */
#define  UCR2_STPB       (1<<6)	 /* Stop */
#define  UCR2_WS         (1<<5)	 /* Word size */
#define  UCR2_RTSEN      (1<<4)	 /* Request to send interrupt enable */
#define  UCR2_ATEN	 (1<<3)  /* Aging Timer Enable */
#define  UCR2_TXEN       (1<<2)	 /* Transmitter enabled */
#define  UCR2_RXEN       (1<<1)	 /* Receiver enabled */
#define  UCR2_SRST 	 (1<<0)	 /* SW reset */

#define  UCR3_DTREN 	 (1<<13) /* DTR interrupt enable */
#define  UCR3_PARERREN   (1<<12) /* Parity enable */
#define  UCR3_FRAERREN   (1<<11) /* Frame error interrupt enable */
#define  UCR3_DSR        (1<<10) /* Data set ready */
#define  UCR3_DCD        (1<<9)  /* Data carrier detect */
#define  UCR3_RI         (1<<8)  /* Ring indicator */
#define  UCR3_ADNIMP     (1<<7)  /* Timeout interrupt enable */
#define  UCR3_RXDSEN	 (1<<6)  /* Receive status interrupt enable */
#define  UCR3_AIRINTEN   (1<<5)  /* Async IR wake interrupt enable */
#define  UCR3_AWAKEN	 (1<<4)  /* Async wake interrupt enable */
// not on mx21 #define  UCR3_REF25 	 (1<<3)  /* Ref freq 25 MHz */
#define  UCR3_RXDMUXSEL  (1<<2)  /* RXD Mux Input Select */
#define  UCR3_INVT  	 (1<<1)  /* Inverted Infrared transmission */
#define  UCR3_ACIEN  	 (1<<0)  /* Autobaud Counter  Interrupt Enable */


#define  UCR4_INVR  	 (1<<9)  /* Inverted infrared reception */
#define  UCR4_ENIRI 	 (1<<8)  /* Serial infrared interrupt enable */
#define  UCR4_WKEN  	 (1<<7)  /* Wake interrupt enable */
// not on mx21 #define  UCR4_REF16 	 (1<<6)  /* Ref freq 16 MHz */
#define  UCR4_IRSC  	 (1<<5)  /* IR special case */
#define  UCR4_LPBYP  	 (1<<5)  /* Low Power Bypass */
#define  UCR4_TCEN  	 (1<<3)  /* Transmit complete interrupt enable */
#define  UCR4_BKEN  	 (1<<2)  /* Break condition interrupt enable */
#define  UCR4_OREN  	 (1<<1)  /* Receiver overrun interrupt enable */
#define  UCR4_DREN  	 (1<<0)  /* Recv data ready interrupt enable */

#define  UFCR_RXTL_SHF   0       /* Receiver trigger level shift */
#define  UFCR_RFDIV      (7<<7)  /* Reference freq divider mask */
#define  UFCR_RFDIV_SHF  7	 /* Reference freq divider shift */
#define  UFCR_TXTL_SHF   10      /* Transmitter trigger level shift */
#define  UFCR_DCEDTE	 (1<<6)  /* DCE/DTE Mode */

#define  USR1_PARITYERR  (1<<15) /* Parity error interrupt flag */
#define  USR1_RTSS  	 (1<<14) /* RTS pin status */
#define  USR1_TRDY  	 (1<<13) /* Transmitter ready interrupt/dma flag */
#define  USR1_RTSD  	 (1<<12) /* RTS delta */
#define  USR1_ESCF  	 (1<<11) /* Escape seq interrupt flag */
#define  USR1_FRAMERR    (1<<10) /* Frame error interrupt flag */
#define  USR1_RRDY       (1<<9)	 /* Receiver ready interrupt/dma flag */
#define  USR1_AGTIM      (1<<8)	 /* Aging Timer Interrupt Flag */
// not on mx21 #define  USR1_TIMEOUT    (1<<7)	 /* Receive timeout interrupt status */
#define  USR1_RXDS  	 (1<<6)	 /* Receiver idle interrupt flag */
#define  USR1_AIRINT	 (1<<5)	 /* Async IR wake interrupt flag */
#define  USR1_AWAKE 	 (1<<4)	 /* Aysnc wake interrupt flag */

#define  USR2_ADET  	 (1<<15) /* Auto baud rate detect complete */
#define  USR2_TXFE  	 (1<<14) /* Transmit buffer FIFO empty */
#define  USR2_DTRF  	 (1<<13) /* DTR edge interrupt flag */
#define  USR2_IDLE  	 (1<<12) /* Idle condition */
#define  USR2_ACST  	 (1<<11) /* Autobaud Controller Stopped*/
#define  USR2_RIDELT  	 (1<<10) /* Ring Indicator Delta */
#define  USR2_RIIN  	 (1<<9)  /* Ring Indicator Input*/
#define  USR2_IRINT 	 (1<<8)	 /* Serial infrared interrupt flag */
#define  USR2_WAKE  	 (1<<7)	 /* Wake */
#define  USR2_DCDDELT  	 (1<<6)	 /* Data Carrier Delta Detect */
#define  USR2_DCDIN  	 (1<<5)	 /* Data Carrier Detect Input */
#define  USR2_RTSF  	 (1<<4)	 /* RTS edge interrupt flag */
#define  USR2_TXDC  	 (1<<3)	 /* Transmitter complete */
#define  USR2_BRCD  	 (1<<2)	 /* Break condition */
#define  USR2_ORE        (1<<1)	 /* Overrun error */
#define  USR2_RDR        (1<<0)	 /* Recv data ready */

#define  UTS_FRCPERR	 (1<<13) /* Force parity error */
#define  UTS_LOOP        (1<<12) /* Loop tx and rx */
#define  UTS_DBGEN       (1<<11) /* /Debug Enable */
#define  UTS_LOOPIR      (1<<10) /* Loop tx and rx for IR */
#define  UTS_RXFIFO	 (1<<9)	 /* RXFifo Debug */
#define  UTS_TXEMPTY	 (1<<6)	 /* TxFIFO empty */
#define  UTS_RXEMPTY	 (1<<5)	 /* RxFIFO empty */
#define  UTS_TXFULL 	 (1<<4)	 /* TxFIFO full */
#define  UTS_RXFULL 	 (1<<3)	 /* RxFIFO full */
#define  UTS_SOFTRST	 (1<<0)	 /* Software reset */

#endif				// _MX2ADS_REGS_H
