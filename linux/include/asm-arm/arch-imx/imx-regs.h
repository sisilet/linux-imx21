#ifndef _IMX_REGS_H
#define _IMX_REGS_H
/* ------------------------------------------------------------------------
 *  Motorola IMX system registers
 * ------------------------------------------------------------------------
 *
 */

/*
 *  Register BASEs, based on OFFSETs
 *
 */
#define IMX_AIPI1_BASE             (0x00000 + IMX_IO_BASE)
#define IMX_WDT_BASE               (0x01000 + IMX_IO_BASE)
#define IMX_TIM1_BASE              (0x02000 + IMX_IO_BASE)
#define IMX_TIM2_BASE              (0x03000 + IMX_IO_BASE)
#define IMX_RTC_BASE               (0x04000 + IMX_IO_BASE)
#define IMX_LCDC_BASE              (0x05000 + IMX_IO_BASE)
#define IMX_UART1_BASE             (0x06000 + IMX_IO_BASE)
#define IMX_UART2_BASE             (0x07000 + IMX_IO_BASE)
#define IMX_PWM_BASE               (0x08000 + IMX_IO_BASE)
#define IMX_DMAC_BASE              (0x09000 + IMX_IO_BASE)
#define IMX_AIPI2_BASE             (0x10000 + IMX_IO_BASE)
#define IMX_SIM_BASE               (0x11000 + IMX_IO_BASE)
#define IMX_USBD_BASE              (0x12000 + IMX_IO_BASE)
#define IMX_SPI1_BASE              (0x13000 + IMX_IO_BASE)
#define IMX_MMC_BASE               (0x14000 + IMX_IO_BASE)
#define IMX_ASP_BASE               (0x15000 + IMX_IO_BASE)
#define IMX_BTA_BASE               (0x16000 + IMX_IO_BASE)
#define IMX_I2C_BASE               (0x17000 + IMX_IO_BASE)
#define IMX_SSI_BASE               (0x18000 + IMX_IO_BASE)
#define IMX_SPI2_BASE              (0x19000 + IMX_IO_BASE)
#define IMX_MSHC_BASE              (0x1A000 + IMX_IO_BASE)
#define IMX_PLL_BASE               (0x1B000 + IMX_IO_BASE)
#define IMX_GPIO_BASE              (0x1C000 + IMX_IO_BASE)
#define IMX_EIM_BASE               (0x20000 + IMX_IO_BASE)
#define IMX_SDRAMC_BASE            (0x21000 + IMX_IO_BASE)
#define IMX_MMA_BASE               (0x22000 + IMX_IO_BASE)
#define IMX_AITC_BASE              (0x23000 + IMX_IO_BASE)
#define IMX_CSI_BASE               (0x24000 + IMX_IO_BASE)

/* PLL registers */
#define CSCR   __REG(IMX_PLL_BASE)        /* Clock Source Control Register */
#define CSCR_SYSTEM_SEL (1<<16)

#define MPCTL0 __REG(IMX_PLL_BASE + 0x4)  /* MCU PLL Control Register 0 */
#define MPCTL1 __REG(IMX_PLL_BASE + 0x8)  /* MCU PLL and System Clock Register 1 */
#define SPCTL0 __REG(IMX_PLL_BASE + 0xc)  /* System PLL Control Register 0 */
#define SPCTL1 __REG(IMX_PLL_BASE + 0x10) /* System PLL Control Register 1 */
#define PCDR   __REG(IMX_PLL_BASE + 0x20) /* Peripheral Clock Divider Register */
#define FMCR   __REG(IMX_PLL_BASE + 0x808) /* Function multiplexing ctrl reg */

#define CSCR_MPLL_RESTART (1<<21)

/*
 *  GPIO Module and I/O Multiplexer
 *  x = 0..3 for reg_A, reg_B, reg_C, reg_D
 */
#define DDIR(x)    __REG2(IMX_GPIO_BASE + 0x00, ((x) & 3) << 8)
#define OCR1(x)    __REG2(IMX_GPIO_BASE + 0x04, ((x) & 3) << 8)
#define OCR2(x)    __REG2(IMX_GPIO_BASE + 0x08, ((x) & 3) << 8)
#define ICONFA1(x) __REG2(IMX_GPIO_BASE + 0x0c, ((x) & 3) << 8)
#define ICONFA2(x) __REG2(IMX_GPIO_BASE + 0x10, ((x) & 3) << 8)
#define ICONFB1(x) __REG2(IMX_GPIO_BASE + 0x14, ((x) & 3) << 8)
#define ICONFB2(x) __REG2(IMX_GPIO_BASE + 0x18, ((x) & 3) << 8)
#define DR(x)      __REG2(IMX_GPIO_BASE + 0x1c, ((x) & 3) << 8)
#define GIUS(x)    __REG2(IMX_GPIO_BASE + 0x20, ((x) & 3) << 8)
#define SSR(x)     __REG2(IMX_GPIO_BASE + 0x24, ((x) & 3) << 8)
#define ICR1(x)    __REG2(IMX_GPIO_BASE + 0x28, ((x) & 3) << 8)
#define ICR2(x)    __REG2(IMX_GPIO_BASE + 0x2c, ((x) & 3) << 8)
#define IMR(x)     __REG2(IMX_GPIO_BASE + 0x30, ((x) & 3) << 8)
#define ISR(x)     __REG2(IMX_GPIO_BASE + 0x34, ((x) & 3) << 8)
#define GPR(x)     __REG2(IMX_GPIO_BASE + 0x38, ((x) & 3) << 8)
#define SWR(x)     __REG2(IMX_GPIO_BASE + 0x3c, ((x) & 3) << 8)
#define PUEN(x)    __REG2(IMX_GPIO_BASE + 0x40, ((x) & 3) << 8)

#define GPIO_PIN_MASK 0x1f
#define GPIO_PORT_MASK (0x3 << 5)

#define GPIO_PORTA (0<<5)
#define GPIO_PORTB (1<<5)
#define GPIO_PORTC (2<<5)
#define GPIO_PORTD (3<<5)

#define GPIO_OUT   (1<<7)
#define GPIO_IN    (0<<7)
#define GPIO_PUEN  (1<<8)

#define GPIO_PF    (0<<9)
#define GPIO_AF    (1<<9)

#define GPIO_OCR_MASK (3<<10)
#define GPIO_AIN   (0<<10)
#define GPIO_BIN   (1<<10)
#define GPIO_CIN   (2<<10)
#define GPIO_GPIO  (3<<10)

#define GPIO_AOUT  (1<<12)
#define GPIO_BOUT  (1<<13)

/* assignements for GPIO alternate/primary functions */

/* FIXME: This list is not completed. The correct directions are
 * missing on some (many) pins
 */
#define PA0_PF_A24           ( GPIO_PORTA | GPIO_PF | 0 )
#define PA0_AIN_SPI2_CLK     ( GPIO_PORTA | GPIO_OUT | GPIO_AIN | 0 )
#define PA0_AF_ETMTRACESYNC  ( GPIO_PORTA | GPIO_AF | 0 )
#define PA1_AOUT_SPI2_RXD    ( GPIO_PORTA | GPIO_IN | GPIO_AOUT | 1 )
#define PA1_PF_TIN           ( GPIO_PORTA | GPIO_PF | 1 )
#define PA2_PF_PWM0          ( GPIO_PORTA | GPIO_OUT | GPIO_PF | 2 )
#define PA3_PF_CSI_MCLK      ( GPIO_PORTA | GPIO_PF | 3 )
#define PA4_PF_CSI_D0        ( GPIO_PORTA | GPIO_PF | 4 )
#define PA5_PF_CSI_D1        ( GPIO_PORTA | GPIO_PF | 5 )
#define PA6_PF_CSI_D2        ( GPIO_PORTA | GPIO_PF | 6 )
#define PA7_PF_CSI_D3        ( GPIO_PORTA | GPIO_PF | 7 )
#define PA8_PF_CSI_D4        ( GPIO_PORTA | GPIO_PF | 8 )
#define PA9_PF_CSI_D5        ( GPIO_PORTA | GPIO_PF | 9 )
#define PA10_PF_CSI_D6       ( GPIO_PORTA | GPIO_PF | 10 )
#define PA11_PF_CSI_D7       ( GPIO_PORTA | GPIO_PF | 11 )
#define PA12_PF_CSI_VSYNC    ( GPIO_PORTA | GPIO_PF | 12 )
#define PA13_PF_CSI_HSYNC    ( GPIO_PORTA | GPIO_PF | 13 )
#define PA14_PF_CSI_PIXCLK   ( GPIO_PORTA | GPIO_PF | 14 )
#define PA15_PF_I2C_SDA      ( GPIO_PORTA | GPIO_OUT | GPIO_PF | 15 )
#define PA16_PF_I2C_SCL      ( GPIO_PORTA | GPIO_OUT | GPIO_PF | 16 )
#define PA17_AF_ETMTRACEPKT4 ( GPIO_PORTA | GPIO_AF | 17 )
#define PA17_AIN_SPI2_SS     ( GPIO_PORTA | GPIO_AIN | 17 )
#define PA18_AF_ETMTRACEPKT5 ( GPIO_PORTA | GPIO_AF | 18 )
#define PA19_AF_ETMTRACEPKT6 ( GPIO_PORTA | GPIO_AF | 19 )
#define PA20_AF_ETMTRACEPKT7 ( GPIO_PORTA | GPIO_AF | 20 )
#define PA21_PF_A0           ( GPIO_PORTA | GPIO_PF | 21 )
#define PA22_PF_CS4          ( GPIO_PORTA | GPIO_PF | 22 )
#define PA23_PF_CS5          ( GPIO_PORTA | GPIO_PF | 23 )
#define PA24_PF_A16          ( GPIO_PORTA | GPIO_PF | 24 )
#define PA24_AF_ETMTRACEPKT0 ( GPIO_PORTA | GPIO_AF | 24 )
#define PA25_PF_A17          ( GPIO_PORTA | GPIO_PF | 25 )
#define PA25_AF_ETMTRACEPKT1 ( GPIO_PORTA | GPIO_AF | 25 )
#define PA26_PF_A18          ( GPIO_PORTA | GPIO_PF | 26 )
#define PA26_AF_ETMTRACEPKT2 ( GPIO_PORTA | GPIO_AF | 26 )
#define PA27_PF_A19          ( GPIO_PORTA | GPIO_PF | 27 )
#define PA27_AF_ETMTRACEPKT3 ( GPIO_PORTA | GPIO_AF | 27 )
#define PA28_PF_A20          ( GPIO_PORTA | GPIO_PF | 28 )
#define PA28_AF_ETMPIPESTAT0 ( GPIO_PORTA | GPIO_AF | 28 )
#define PA29_PF_A21          ( GPIO_PORTA | GPIO_PF | 29 )
#define PA29_AF_ETMPIPESTAT1 ( GPIO_PORTA | GPIO_AF | 29 )
#define PA30_PF_A22          ( GPIO_PORTA | GPIO_PF | 30 )
#define PA30_AF_ETMPIPESTAT2 ( GPIO_PORTA | GPIO_AF | 30 )
#define PA31_PF_A23          ( GPIO_PORTA | GPIO_PF | 31 )
#define PA31_AF_ETMTRACECLK  ( GPIO_PORTA | GPIO_AF | 31 )
#define PB8_PF_SD_DAT0       ( GPIO_PORTB | GPIO_PF | GPIO_PUEN | 8 )
#define PB8_AF_MS_PIO        ( GPIO_PORTB | GPIO_AF | 8 )
#define PB9_PF_SD_DAT1       ( GPIO_PORTB | GPIO_PF | GPIO_PUEN  | 9 )
#define PB9_AF_MS_PI1        ( GPIO_PORTB | GPIO_AF | 9 )
#define PB10_PF_SD_DAT2      ( GPIO_PORTB | GPIO_PF | GPIO_PUEN  | 10 )
#define PB10_AF_MS_SCLKI     ( GPIO_PORTB | GPIO_AF | 10 )
#define PB11_PF_SD_DAT3      ( GPIO_PORTB | GPIO_PF | 11 )
#define PB11_AF_MS_SDIO      ( GPIO_PORTB | GPIO_AF | 11 )
#define PB12_PF_SD_CLK       ( GPIO_PORTB | GPIO_PF | 12 )
#define PB12_AF_MS_SCLK0     ( GPIO_PORTB | GPIO_AF | 12 )
#define PB13_PF_SD_CMD       ( GPIO_PORTB | GPIO_PF | GPIO_PUEN | 13 )
#define PB13_AF_MS_BS        ( GPIO_PORTB | GPIO_AF | 13 )
#define PB14_AF_SSI_RXFS     ( GPIO_PORTB | GPIO_AF | 14 )
#define PB15_AF_SSI_RXCLK    ( GPIO_PORTB | GPIO_AF | 15 )
#define PB16_AF_SSI_RXDAT    ( GPIO_PORTB | GPIO_IN | GPIO_AF | 16 )
#define PB17_AF_SSI_TXDAT    ( GPIO_PORTB | GPIO_OUT | GPIO_AF | 17 )
#define PB18_AF_SSI_TXFS     ( GPIO_PORTB | GPIO_AF | 18 )
#define PB19_AF_SSI_TXCLK    ( GPIO_PORTB | GPIO_AF | 19 )
#define PB20_PF_USBD_AFE     ( GPIO_PORTB | GPIO_PF | 20 )
#define PB21_PF_USBD_OE      ( GPIO_PORTB | GPIO_PF | 21 )
#define PB22_PFUSBD_RCV      ( GPIO_PORTB | GPIO_PF | 22 )
#define PB23_PF_USBD_SUSPND  ( GPIO_PORTB | GPIO_PF | 23 )
#define PB24_PF_USBD_VP      ( GPIO_PORTB | GPIO_PF | 24 )
#define PB25_PF_USBD_VM      ( GPIO_PORTB | GPIO_PF | 25 )
#define PB26_PF_USBD_VPO     ( GPIO_PORTB | GPIO_PF | 26 )
#define PB27_PF_USBD_VMO     ( GPIO_PORTB | GPIO_PF | 27 )
#define PB28_PF_UART2_CTS    ( GPIO_PORTB | GPIO_OUT | GPIO_PF | 28 )
#define PB29_PF_UART2_RTS    ( GPIO_PORTB | GPIO_IN | GPIO_PF | 29 )
#define PB30_PF_UART2_TXD    ( GPIO_PORTB | GPIO_OUT | GPIO_PF | 30 )
#define PB31_PF_UART2_RXD    ( GPIO_PORTB | GPIO_IN | GPIO_PF | 31 )
#define PC3_PF_SSI_RXFS      ( GPIO_PORTC | GPIO_PF | 3 )
#define PC4_PF_SSI_RXCLK     ( GPIO_PORTC | GPIO_PF | 4 )
#define PC5_PF_SSI_RXDAT     ( GPIO_PORTC | GPIO_IN | GPIO_PF | 5 )
#define PC6_PF_SSI_TXDAT     ( GPIO_PORTC | GPIO_OUT | GPIO_PF | 6 )
#define PC7_PF_SSI_TXFS      ( GPIO_PORTC | GPIO_PF | 7 )
#define PC8_PF_SSI_TXCLK     ( GPIO_PORTC | GPIO_PF | 8 )
#define PC9_PF_UART1_CTS     ( GPIO_PORTC | GPIO_OUT | GPIO_PF | 9 )
#define PC10_PF_UART1_RTS    ( GPIO_PORTC | GPIO_IN | GPIO_PF | 10 )
#define PC11_PF_UART1_TXD    ( GPIO_PORTC | GPIO_OUT | GPIO_PF | 11 )
#define PC12_PF_UART1_RXD    ( GPIO_PORTC | GPIO_IN | GPIO_PF | 12 )
#define PC13_PF_SPI1_SPI_RDY ( GPIO_PORTC | GPIO_PF | 13 )
#define PC14_PF_SPI1_SCLK    ( GPIO_PORTC | GPIO_PF | 14 )
#define PC15_PF_SPI1_SS      ( GPIO_PORTC | GPIO_PF | 15 )
#define PC16_PF_SPI1_MISO    ( GPIO_PORTC | GPIO_PF | 16 )
#define PC17_PF_SPI1_MOSI    ( GPIO_PORTC | GPIO_PF | 17 )
#define PD6_PF_LSCLK         ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 6 )
#define PD7_PF_REV           ( GPIO_PORTD | GPIO_PF | 7 )
#define PD7_AF_UART2_DTR     ( GPIO_PORTD | GPIO_IN | GPIO_AF | 7 )
#define PD7_AIN_SPI2_SCLK    ( GPIO_PORTD | GPIO_AIN | 7 )
#define PD8_PF_CLS           ( GPIO_PORTD | GPIO_PF | 8 )
#define PD8_AF_UART2_DCD     ( GPIO_PORTD | GPIO_OUT | GPIO_AF | 8 )
#define PD8_AIN_SPI2_SS      ( GPIO_PORTD | GPIO_AIN | 8 )
#define PD9_PF_PS            ( GPIO_PORTD | GPIO_PF | 9 )
#define PD9_AF_UART2_RI      ( GPIO_PORTD | GPIO_OUT | GPIO_AF | 9 )
#define PD9_AOUT_SPI2_RXD    ( GPIO_PORTD | GPIO_IN | GPIO_AOUT | 9 )
#define PD10_PF_SPL_SPR      ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 10 )
#define PD10_AF_UART2_DSR    ( GPIO_PORTD | GPIO_OUT | GPIO_AF | 10 )
#define PD10_AIN_SPI2_TXD    ( GPIO_PORTD | GPIO_OUT | GPIO_AIN | 10 )
#define PD11_PF_CONTRAST     ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 11 )
#define PD12_PF_ACD_OE       ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 12 )
#define PD13_PF_LP_HSYNC     ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 13 )
#define PD14_PF_FLM_VSYNC    ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 14 )
#define PD15_PF_LD0          ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 15 )
#define PD16_PF_LD1          ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 16 )
#define PD17_PF_LD2          ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 17 )
#define PD18_PF_LD3          ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 18 )
#define PD19_PF_LD4          ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 19 )
#define PD20_PF_LD5          ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 20 )
#define PD21_PF_LD6          ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 21 )
#define PD22_PF_LD7          ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 22 )
#define PD23_PF_LD8          ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 23 )
#define PD24_PF_LD9          ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 24 )
#define PD25_PF_LD10         ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 25 )
#define PD26_PF_LD11         ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 26 )
#define PD27_PF_LD12         ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 27 )
#define PD28_PF_LD13         ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 28 )
#define PD29_PF_LD14         ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 29 )
#define PD30_PF_LD15         ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 30 )
#define PD31_PF_TMR2OUT      ( GPIO_PORTD | GPIO_PF | 31 )
#define PD31_BIN_SPI2_TXD    ( GPIO_PORTD | GPIO_BIN | 31 )

/*
 * PWM controller
 */
#define PWMC	__REG(IMX_PWM_BASE + 0x00)	/* PWM Control Register		*/
#define PWMS	__REG(IMX_PWM_BASE + 0x04)	/* PWM Sample Register		*/
#define PWMP	__REG(IMX_PWM_BASE + 0x08)	/* PWM Period Register		*/
#define PWMCNT	__REG(IMX_PWM_BASE + 0x0C)	/* PWM Counter Register		*/

#define PWMC_HCTR		(0x01<<18)		/* Halfword FIFO Data Swapping	*/
#define PWMC_BCTR		(0x01<<17)		/* Byte FIFO Data Swapping	*/
#define PWMC_SWR		(0x01<<16)		/* Software Reset		*/
#define PWMC_CLKSRC		(0x01<<15)		/* Clock Source			*/
#define PWMC_PRESCALER(x)	(((x-1) & 0x7F) << 8)	/* PRESCALER			*/
#define PWMC_IRQ		(0x01<< 7)		/* Interrupt Request		*/
#define PWMC_IRQEN		(0x01<< 6)		/* Interrupt Request Enable	*/
#define PWMC_FIFOAV		(0x01<< 5)		/* FIFO Available		*/
#define PWMC_EN			(0x01<< 4)		/* Enables/Disables the PWM	*/
#define PWMC_REPEAT(x)		(((x) & 0x03) << 2)	/* Sample Repeats		*/
#define PWMC_CLKSEL(x)		(((x) & 0x03) << 0)	/* Clock Selection		*/

#define PWMS_SAMPLE(x)		((x) & 0xFFFF)		/* Contains a two-sample word	*/
#define PWMP_PERIOD(x)		((x) & 0xFFFF)		/* Represents the PWM's period	*/
#define PWMC_COUNTER(x)		((x) & 0xFFFF)		/* Represents the current count value	*/

/*
 *  DMA Controller
 */
#define DCR     __REG(IMX_DMAC_BASE +0x00)	/* DMA Control Register */
#define DISR    __REG(IMX_DMAC_BASE +0x04)	/* DMA Interrupt status Register */
#define DIMR    __REG(IMX_DMAC_BASE +0x08)	/* DMA Interrupt mask Register */
#define DBTOSR  __REG(IMX_DMAC_BASE +0x0c)	/* DMA Burst timeout status Register */
#define DRTOSR  __REG(IMX_DMAC_BASE +0x10)	/* DMA Request timeout Register */
#define DSESR   __REG(IMX_DMAC_BASE +0x14)	/* DMA Transfer Error Status Register */
#define DBOSR   __REG(IMX_DMAC_BASE +0x18)	/* DMA Buffer overflow status Register */
#define DBTOCR  __REG(IMX_DMAC_BASE +0x1c)	/* DMA Burst timeout control Register */
#define WSRA    __REG(IMX_DMAC_BASE +0x40)	/* W-Size Register A */
#define XSRA    __REG(IMX_DMAC_BASE +0x44)	/* X-Size Register A */
#define YSRA    __REG(IMX_DMAC_BASE +0x48)	/* Y-Size Register A */
#define WSRB    __REG(IMX_DMAC_BASE +0x4c)	/* W-Size Register B */
#define XSRB    __REG(IMX_DMAC_BASE +0x50)	/* X-Size Register B */
#define YSRB    __REG(IMX_DMAC_BASE +0x54)	/* Y-Size Register B */
#define SAR(x)  __REG2( IMX_DMAC_BASE + 0x80, (x) << 6)	/* Source Address Registers */
#define DAR(x)  __REG2( IMX_DMAC_BASE + 0x84, (x) << 6)	/* Destination Address Registers */
#define CNTR(x) __REG2( IMX_DMAC_BASE + 0x88, (x) << 6)	/* Count Registers */
#define CCR(x)  __REG2( IMX_DMAC_BASE + 0x8c, (x) << 6)	/* Control Registers */
#define RSSR(x) __REG2( IMX_DMAC_BASE + 0x90, (x) << 6)	/* Request source select Registers */
#define BLR(x)  __REG2( IMX_DMAC_BASE + 0x94, (x) << 6)	/* Burst length Registers */
#define RTOR(x) __REG2( IMX_DMAC_BASE + 0x98, (x) << 6)	/* Request timeout Registers */
#define BUCR(x) __REG2( IMX_DMAC_BASE + 0x98, (x) << 6)	/* Bus Utilization Registers */

#define DCR_DRST           (1<<1)
#define DCR_DEN            (1<<0)
#define DBTOCR_EN          (1<<15)
#define DBTOCR_CNT(x)      ((x) & 0x7fff )
#define CNTR_CNT(x)        ((x) & 0xffffff )
#define CCR_DMOD_LINEAR    ( 0x0 << 12 )
#define CCR_DMOD_2D        ( 0x1 << 12 )
#define CCR_DMOD_FIFO      ( 0x2 << 12 )
#define CCR_DMOD_EOBFIFO   ( 0x3 << 12 )
#define CCR_SMOD_LINEAR    ( 0x0 << 10 )
#define CCR_SMOD_2D        ( 0x1 << 10 )
#define CCR_SMOD_FIFO      ( 0x2 << 10 )
#define CCR_SMOD_EOBFIFO   ( 0x3 << 10 )
#define CCR_MDIR_DEC       (1<<9)
#define CCR_MSEL_B         (1<<8)
#define CCR_DSIZ_32        ( 0x0 << 6 )
#define CCR_DSIZ_8         ( 0x1 << 6 )
#define CCR_DSIZ_16        ( 0x2 << 6 )
#define CCR_SSIZ_32        ( 0x0 << 4 )
#define CCR_SSIZ_8         ( 0x1 << 4 )
#define CCR_SSIZ_16        ( 0x2 << 4 )
#define CCR_REN            (1<<3)
#define CCR_RPT            (1<<2)
#define CCR_FRC            (1<<1)
#define CCR_CEN            (1<<0)
#define RTOR_EN            (1<<15)
#define RTOR_CLK           (1<<14)
#define RTOR_PSC           (1<<13)

/*
 *  Interrupt controller
 */

#define IMX_INTCNTL        __REG(IMX_AITC_BASE+0x00)
#define INTCNTL_FIAD       (1<<19)
#define INTCNTL_NIAD       (1<<20)

#define IMX_NIMASK         __REG(IMX_AITC_BASE+0x04)
#define IMX_INTENNUM       __REG(IMX_AITC_BASE+0x08)
#define IMX_INTDISNUM      __REG(IMX_AITC_BASE+0x0c)
#define IMX_INTENABLEH     __REG(IMX_AITC_BASE+0x10)
#define IMX_INTENABLEL     __REG(IMX_AITC_BASE+0x14)

/*
 *  General purpose timers
 */
#define IMX_TCTL(x)        __REG( 0x00 + (x))
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

#define IMX_TPRER(x)       __REG( 0x04 + (x))
#define IMX_TCMP(x)        __REG( 0x08 + (x))
#define IMX_TCR(x)         __REG( 0x0C + (x))
#define IMX_TCN(x)         __REG( 0x10 + (x))
#define IMX_TSTAT(x)       __REG( 0x14 + (x))
#define TSTAT_CAPT         (1<<1)
#define TSTAT_COMP         (1<<0)

/*
 * LCD Controller
 */

#define LCDC_SSA	__REG(IMX_LCDC_BASE+0x00)

#define LCDC_SIZE	__REG(IMX_LCDC_BASE+0x04)
#define SIZE_XMAX(x)	((((x) >> 4) & 0x3f) << 20)
#define SIZE_YMAX(y)    ( (y) & 0x1ff )

#define LCDC_VPW	__REG(IMX_LCDC_BASE+0x08)
#define VPW_VPW(x)	( (x) & 0x3ff )

#define LCDC_CPOS	__REG(IMX_LCDC_BASE+0x0C)
#define CPOS_CC1        (1<<31)
#define CPOS_CC0        (1<<30)
#define CPOS_OP         (1<<28)
#define CPOS_CXP(x)     (((x) & 3ff) << 16)
#define CPOS_CYP(y)     ((y) & 0x1ff)

#define LCDC_LCWHB	__REG(IMX_LCDC_BASE+0x10)
#define LCWHB_BK_EN     (1<<31)
#define LCWHB_CW(w)     (((w) & 0x1f) << 24)
#define LCWHB_CH(h)     (((h) & 0x1f) << 16)
#define LCWHB_BD(x)     ((x) & 0xff)

#define LCDC_LCHCC	__REG(IMX_LCDC_BASE+0x14)
#define LCHCC_CUR_COL_R(r) (((r) & 0x1f) << 11)
#define LCHCC_CUR_COL_G(g) (((g) & 0x3f) << 5)
#define LCHCC_CUR_COL_B(b) ((b) & 0x1f)

#define LCDC_PCR	__REG(IMX_LCDC_BASE+0x18)
#define PCR_TFT         (1<<31)
#define PCR_COLOR       (1<<30)
#define PCR_PBSIZ_1     (0<<28)
#define PCR_PBSIZ_2     (1<<28)
#define PCR_PBSIZ_4     (2<<28)
#define PCR_PBSIZ_8     (3<<28)
#define PCR_BPIX_1      (0<<25)
#define PCR_BPIX_2      (1<<25)
#define PCR_BPIX_4      (2<<25)
#define PCR_BPIX_8      (3<<25)
#define PCR_BPIX_12     (4<<25)
#define PCR_BPIX_16     (4<<25)
#define PCR_PIXPOL      (1<<24)
#define PCR_FLMPOL      (1<<23)
#define PCR_LPPOL       (1<<22)
#define PCR_CLKPOL      (1<<21)
#define PCR_OEPOL       (1<<20)
#define PCR_SCLKIDLE    (1<<19)
#define PCR_END_SEL     (1<<18)
#define PCR_END_BYTE_SWAP (1<<17)
#define PCR_REV_VS      (1<<16)
#define PCR_ACD_SEL     (1<<15)
#define PCR_ACD(x)      (((x) & 0x7f) << 8)
#define PCR_SCLK_SEL    (1<<7)
#define PCR_SHARP       (1<<6)
#define PCR_PCD(x)      ((x) & 0x3f)

#define LCDC_HCR	__REG(IMX_LCDC_BASE+0x1C)
#define HCR_H_WIDTH(x)  (((x) & 0x3f) << 26)
#define HCR_H_WAIT_1(x) (((x) & 0xff) << 8)
#define HCR_H_WAIT_2(x) ((x) & 0xff)

#define LCDC_VCR	__REG(IMX_LCDC_BASE+0x20)
#define VCR_V_WIDTH(x)  (((x) & 0x3f) << 26)
#define VCR_V_WAIT_1(x) (((x) & 0xff) << 8)
#define VCR_V_WAIT_2(x) ((x) & 0xff)

#define LCDC_POS	__REG(IMX_LCDC_BASE+0x24)
#define POS_POS(x)      ((x) & 1f)

#define LCDC_LSCR1	__REG(IMX_LCDC_BASE+0x28)
#define LSCR1_PS_RISE_DELAY(x)    (((x) & 0x7f) << 26)
#define LSCR1_CLS_RISE_DELAY(x)   (((x) & 0x3f) << 16)
#define LSCR1_REV_TOGGLE_DELAY(x) (((x) & 0xf) << 8)
#define LSCR1_GRAY2(x)            (((x) & 0xf) << 4)
#define LSCR1_GRAY1(x)            (((x) & 0xf))

#define LCDC_PWMR	__REG(IMX_LCDC_BASE+0x2C)
#define PWMR_CLS(x)     (((x) & 0x1ff) << 16)
#define PWMR_LDMSK      (1<<15)
#define PWMR_SCR1       (1<<10)
#define PWMR_SCR0       (1<<9)
#define PWMR_CC_EN      (1<<8)
#define PWMR_PW(x)      ((x) & 0xff)

#define LCDC_DMACR	__REG(IMX_LCDC_BASE+0x30)
#define DMACR_BURST     (1<<31)
#define DMACR_HM(x)     (((x) & 0xf) << 16)
#define DMACR_TM(x)     ((x) &0xf)

#define LCDC_RMCR	__REG(IMX_LCDC_BASE+0x34)
#define RMCR_LCDC_EN		(1<<1)
#define RMCR_SELF_REF		(1<<0)

#define LCDC_LCDICR	__REG(IMX_LCDC_BASE+0x38)
#define LCDICR_INT_SYN  (1<<2)
#define LCDICR_INT_CON  (1)

#define LCDC_LCDISR	__REG(IMX_LCDC_BASE+0x40)
#define LCDISR_UDR_ERR (1<<3)
#define LCDISR_ERR_RES (1<<2)
#define LCDISR_EOF     (1<<1)
#define LCDISR_BOF     (1<<0)

/*
 *  UART Module. Takes the UART base address as argument
 */
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
#define BIPR1(x) __REG( 0xb0 + (x)) /* Incremental Preset Register 1 */
#define BIPR2(x) __REG( 0xb4 + (x)) /* Incremental Preset Register 2 */
#define BIPR3(x) __REG( 0xb8 + (x)) /* Incremental Preset Register 3 */
#define BIPR4(x) __REG( 0xbc + (x)) /* Incremental Preset Register 4 */
#define BMPR1(x) __REG( 0xc0 + (x)) /* BRM Modulator Register 1 */
#define BMPR2(x) __REG( 0xc4 + (x)) /* BRM Modulator Register 2 */
#define BMPR3(x) __REG( 0xc8 + (x)) /* BRM Modulator Register 3 */
#define BMPR4(x) __REG( 0xcc + (x)) /* BRM Modulator Register 4 */
#define UTS(x)   __REG( 0xd0 + (x)) /* UART Test Register */

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
#define  UCR1_UARTCLKEN  (1<<2)	 /* UART clock enabled */
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
#define  UCR2_TXEN       (1<<2)	 /* Transmitter enabled */
#define  UCR2_RXEN       (1<<1)	 /* Receiver enabled */
#define  UCR2_SRST 	 (1<<0)	 /* SW reset */
#define  UCR3_DTREN 	 (1<<13) /* DTR interrupt enable */
#define  UCR3_PARERREN   (1<<12) /* Parity enable */
#define  UCR3_FRAERREN   (1<<11) /* Frame error interrupt enable */
#define  UCR3_DSR        (1<<10) /* Data set ready */
#define  UCR3_DCD        (1<<9)  /* Data carrier detect */
#define  UCR3_RI         (1<<8)  /* Ring indicator */
#define  UCR3_TIMEOUTEN  (1<<7)  /* Timeout interrupt enable */
#define  UCR3_RXDSEN	 (1<<6)  /* Receive status interrupt enable */
#define  UCR3_AIRINTEN   (1<<5)  /* Async IR wake interrupt enable */
#define  UCR3_AWAKEN	 (1<<4)  /* Async wake interrupt enable */
#define  UCR3_REF25 	 (1<<3)  /* Ref freq 25 MHz */
#define  UCR3_REF30 	 (1<<2)  /* Ref Freq 30 MHz */
#define  UCR3_INVT  	 (1<<1)  /* Inverted Infrared transmission */
#define  UCR3_BPEN  	 (1<<0)  /* Preset registers enable */
#define  UCR4_CTSTL_32   (32<<10) /* CTS trigger level (32 chars) */
#define  UCR4_INVR  	 (1<<9)  /* Inverted infrared reception */
#define  UCR4_ENIRI 	 (1<<8)  /* Serial infrared interrupt enable */
#define  UCR4_WKEN  	 (1<<7)  /* Wake interrupt enable */
#define  UCR4_REF16 	 (1<<6)  /* Ref freq 16 MHz */
#define  UCR4_IRSC  	 (1<<5)  /* IR special case */
#define  UCR4_TCEN  	 (1<<3)  /* Transmit complete interrupt enable */
#define  UCR4_BKEN  	 (1<<2)  /* Break condition interrupt enable */
#define  UCR4_OREN  	 (1<<1)  /* Receiver overrun interrupt enable */
#define  UCR4_DREN  	 (1<<0)  /* Recv data ready interrupt enable */
#define  UFCR_RXTL_SHF   0       /* Receiver trigger level shift */
#define  UFCR_RFDIV      (7<<7)  /* Reference freq divider mask */
#define  UFCR_TXTL_SHF   10      /* Transmitter trigger level shift */
#define  USR1_PARITYERR  (1<<15) /* Parity error interrupt flag */
#define  USR1_RTSS  	 (1<<14) /* RTS pin status */
#define  USR1_TRDY  	 (1<<13) /* Transmitter ready interrupt/dma flag */
#define  USR1_RTSD  	 (1<<12) /* RTS delta */
#define  USR1_ESCF  	 (1<<11) /* Escape seq interrupt flag */
#define  USR1_FRAMERR    (1<<10) /* Frame error interrupt flag */
#define  USR1_RRDY       (1<<9)	 /* Receiver ready interrupt/dma flag */
#define  USR1_TIMEOUT    (1<<7)	 /* Receive timeout interrupt status */
#define  USR1_RXDS  	 (1<<6)	 /* Receiver idle interrupt flag */
#define  USR1_AIRINT	 (1<<5)	 /* Async IR wake interrupt flag */
#define  USR1_AWAKE 	 (1<<4)	 /* Aysnc wake interrupt flag */
#define  USR2_ADET  	 (1<<15) /* Auto baud rate detect complete */
#define  USR2_TXFE  	 (1<<14) /* Transmit buffer FIFO empty */
#define  USR2_DTRF  	 (1<<13) /* DTR edge interrupt flag */
#define  USR2_IDLE  	 (1<<12) /* Idle condition */
#define  USR2_IRINT 	 (1<<8)	 /* Serial infrared interrupt flag */
#define  USR2_WAKE  	 (1<<7)	 /* Wake */
#define  USR2_RTSF  	 (1<<4)	 /* RTS edge interrupt flag */
#define  USR2_TXDC  	 (1<<3)	 /* Transmitter complete */
#define  USR2_BRCD  	 (1<<2)	 /* Break condition */
#define  USR2_ORE        (1<<1)	 /* Overrun error */
#define  USR2_RDR        (1<<0)	 /* Recv data ready */
#define  UTS_FRCPERR	 (1<<13) /* Force parity error */
#define  UTS_LOOP        (1<<12) /* Loop tx and rx */
#define  UTS_TXEMPTY	 (1<<6)	 /* TxFIFO empty */
#define  UTS_RXEMPTY	 (1<<5)	 /* RxFIFO empty */
#define  UTS_TXFULL 	 (1<<4)	 /* TxFIFO full */
#define  UTS_RXFULL 	 (1<<3)	 /* RxFIFO full */
#define  UTS_SOFTRST	 (1<<0)	 /* Software reset */

/* 
 * SSP Module.
 */
#define SSP1_RX_REG          __REG(IMX_SPI1_BASE + 0x00)
#define SSP1_TX_REG          __REG(IMX_SPI1_BASE + 0x04)
#define SSP1_CTRL_REG        __REG(IMX_SPI1_BASE + 0x08)
#define SSP1_INT_REG         __REG(IMX_SPI1_BASE + 0x0c)
#define SSP1_TEST_REG        __REG(IMX_SPI1_BASE + 0x10)
#define SSP1_PER_REG         __REG(IMX_SPI1_BASE + 0x14)
#define SSP1_DMA_REG         __REG(IMX_SPI1_BASE + 0x18)
#define SSP1_RESET_REG       __REG(IMX_SPI1_BASE + 0x1c)

#define SSP_RATE_DIV4       (0 << 13)
#define SSP_RATE_DIV8       (1 << 13)
#define SSP_RATE_DIV16      (2 << 13)
#define SSP_RATE_DIV32      (3 << 13)
#define SSP_RATE_DIV64      (4 << 13)
#define SSP_RATE_DIV128     (5 << 13)
#define SSP_RATE_DIV256     (6 << 13)
#define SSP_RATE_DIV512     (7 << 13)
#define SSP_MODE_MASTER     (1 << 10)
#define SSP_MODE_SLAVE      (0 << 10)
#define SSP_ENABLE          (1 << 9)
#define SSP_DISABLE         (0 << 9)
#define SSP_XCH             (1 << 8)
#define SSP_SS_POL_HIGH     (1 << 7)
#define SSP_SS_POL_LOW      (0 << 7)
#define SSP_SS_PULSE        (1 << 6)
#define SSP_PHA0            (0 << 5)
#define SSP_PHA1            (1 << 5)
#define SSP_POL0            (0 << 4)
#define SSP_POL1            (1 << 4)
#define SSP_WS(x)           ((x-1) & 0xf)

#define SSP_INT_BOEN        (1 << 15)
#define SSP_INT_ROEN        (1 << 14)
#define SSP_INT_RFEN        (1 << 13)
#define SSP_INT_RHEN        (1 << 12)
#define SSP_INT_RREN        (1 << 11)
#define SSP_INT_TFEN        (1 << 10)
#define SSP_INT_THEN        (1 << 9)
#define SSP_INT_TEEN        (1 << 8)
#define SSP_INT_BO          (1 << 7)
#define SSP_INT_RO          (1 << 6)
#define SSP_INT_RF          (1 << 5)
#define SSP_INT_RH          (1 << 4)
#define SSP_INT_RR          (1 << 3)
#define SSP_INT_TF          (1 << 2)
#define SSP_INT_TH          (1 << 1)
#define SSP_INT_TE          (1 << 0)
#define SSP_TEST_LOOPBACK   (1 << 14)
#define SSP_PER_CLK_BIT     (0 << 15)
#define SSP_PER_CLK_32KHZ   (1 << 15)
#define SSP_PER_WAIT(x)     ((x) & 0x7fff)
#define SSP_RESET_START     (1 << 0)


/*
 * SSI Module
 */
#define SSI_STX   __REG(IMX_SSI_BASE + 0x00)
#define SSI_SRX   __REG(IMX_SSI_BASE + 0x04)
#define SSI_SCSR  __REG(IMX_SSI_BASE + 0x08)
#define SSI_STCR  __REG(IMX_SSI_BASE + 0x0c)
#define SSI_SRCR  __REG(IMX_SSI_BASE + 0x10)
#define SSI_STCCR __REG(IMX_SSI_BASE + 0x14)
#define SSI_SRCCR __REG(IMX_SSI_BASE + 0x18)
#define SSI_STSR  __REG(IMX_SSI_BASE + 0x1c)
#define SSI_SFCSR __REG(IMX_SSI_BASE + 0x20)
#define SSI_SOR   __REG(IMX_SSI_BASE + 0x28)

#define SSI_SCSR_SCR         (1 << 15)
#define SSI_SCSR_MODE_NORM   (0 << 13)
#define SSI_SCSR_MODE_MSTR   (1 << 13)
#define SSI_SCSR_MODE_SLAVE  (2 << 13)
#define SSI_SCSR_SYN         (1 << 12)
#define SSI_SCSR_NET         (1 << 11)
#define SSI_SCSR_RE          (1 << 10)
#define SSI_SCSR_TE          (1 <<  9)
#define SSI_SCSR_EN          (1 <<  8)
#define SSI_SCSR_RDR         (1 <<  7)
#define SSI_SCSR_TDE         (1 <<  6)
#define SSI_SCSR_ROE         (1 <<  5)
#define SSI_SCSR_TUE         (1 <<  4)
#define SSI_SCSR_TFS         (1 <<  3)
#define SSI_SCSR_RFS         (1 <<  2)
#define SSI_SCSR_RFF         (1 <<  1)
#define SSI_SCSR_TFE         (1 <<  0)

#define SSI_STCR_TXBIT0      (1 << 10)
#define SSI_STCR_TDMAE       (1 <<  9)
#define SSI_STCR_TIE         (1 <<  8)
#define SSI_STCR_TFEN        (1 <<  7)
#define SSI_STCR_TFDIR       (1 <<  6)
#define SSI_STCR_TXDIR       (1 <<  5)
#define SSI_STCR_TSHFD       (1 <<  4)
#define SSI_STCR_TSCKP       (1 <<  3)
#define SSI_STCR_TFSI        (1 <<  2)
#define SSI_STCR_TFSL        (1 <<  1)
#define SSI_STCR_TEFS        (1 <<  0)

#define SSI_SRCR_RXBIT0      (1 << 10)
#define SSI_SRCR_RDMAE       (1 <<  9)
#define SSI_SRCR_RIE         (1 <<  8)
#define SSI_SRCR_RFEN        (1 <<  7)
#define SSI_SRCR_RFDIR       (1 <<  6)
#define SSI_SRCR_RXDIR       (1 <<  5)
#define SSI_SRCR_RSHFD       (1 <<  4)
#define SSI_SRCR_RSCKP       (1 <<  3)
#define SSI_SRCR_RFSI        (1 <<  2)
#define SSI_SRCR_RFSL        (1 <<  1)
#define SSI_SRCR_REFS        (1 <<  0)

#define SSI_STCCR_PS         (1 << 15)
#define SSI_STCCR_WL8        (0 << 13)
#define SSI_STCCR_WL10       (1 << 13)
#define SSI_STCCR_WL12       (2 << 13)
#define SSI_STCCR_WL16       (3 << 13)
#define SSI_STCCR_DC(x)      (((x) & 0x1f) << 8)
#define SSI_STCCR_PM(x)      (((x) & 0xff) << 0)

#define SSI_SRCCR_PS         (1 << 15)
#define SSI_SRCCR_WL8        (0 << 13)
#define SSI_SRCCR_WL10       (1 << 13)
#define SSI_SRCCR_WL12       (2 << 13)
#define SSI_SRCCR_WL16       (3 << 13)
#define SSI_SRCCR_DC(x)      (((x) & 0x1f) << 8)
#define SSI_SRCCR_PM(x)      (((x) & 0xff) << 0)

#define SSI_SFCSR_RFCNT(x)   (((x) & 0xf) << 12)
#define SSI_SFCSR_TFCNT(x)   (((x) & 0xf) <<  8)
#define SSI_SFCSR_RFWM(x)    (((x) & 0xf) <<  4)
#define SSI_SFCSR_TFWM(x)    (((x) & 0xf) <<  0)

#define SSI_SOR_CLKOFF       (1 << 6)
#define SSI_SOR_RXCLR        (1 << 5)
#define SSI_SOR_TXCLR        (1 << 4)
#define SSI_SOR_SYNRST       (1 << 0)


/*
 * External Interface Module
 */
#define EIM_CS0U   __REG(IMX_EIM_BASE +0x00)	/* Chip Select 0 Upper Control */
#define EIM_CS0L   __REG(IMX_EIM_BASE +0x04)	/* Chip Select 0 Lower Control */
#define EIM_CS1U   __REG(IMX_EIM_BASE +0x08)	/* Chip Select 1 Upper Control */
#define EIM_CS1L   __REG(IMX_EIM_BASE +0x0c)	/* Chip Select 1 Lower Control */
#define EIM_CS2U   __REG(IMX_EIM_BASE +0x10)	/* Chip Select 2 Upper Control */
#define EIM_CS2L   __REG(IMX_EIM_BASE +0x14)	/* Chip Select 2 Lower Control */
#define EIM_CS3U   __REG(IMX_EIM_BASE +0x18)	/* Chip Select 3 Upper Control */
#define EIM_CS3L   __REG(IMX_EIM_BASE +0x1c)	/* Chip Select 3 Lower Control */
#define EIM_CS4U   __REG(IMX_EIM_BASE +0x20)	/* Chip Select 4 Upper Control */
#define EIM_CS4L   __REG(IMX_EIM_BASE +0x24)	/* Chip Select 4 Lower Control */
#define EIM_CS5U   __REG(IMX_EIM_BASE +0x28)	/* Chip Select 5 Upper Control */
#define EIM_CS5L   __REG(IMX_EIM_BASE +0x2c)	/* Chip Select 5 Lower Control */
#define EIM_EIM    __REG(IMX_EIM_BASE +0x30)	/* EIM Configuration */

#define EIM_CSnU_DTACK_SEL (1 << 31)
#define EIM_CSnU_BCD(x)  (((x) & 0x3)  << 28)
#define EIM_CSnU_BCS(x)  (((x) & 0xf)  << 24)
#define EIM_CSnU_PSZ(x)  (((x) & 0x3)  << 22)
#define EIM_CSnU_PME     (1 << 21)
#define EIM_CSnU_SYNC    (1 << 20)
#define EIM_CSnU_DOL(x)  (((x) & 0xf)  << 16)
#define EIM_CSnU_CNC(x)  (((x) & 0x3)  << 14)
#define EIM_CSnU_WSC(x)  (((x) & 0x3f) <<  8)
#define EIM_CSnU_WSC_DTACK (0x3f <<  8)
#define EIM_CSnU_WWS(x)  (((x) & 0x7)  <<  4)
#define EIM_CSnU_EDC(x)  (((x) & 0xf)  <<  0)

#define EIM_CSnL_OEA(x)  (((x) & 0xf)  << 28)
#define EIM_CSnL_OEN(x)  (((x) & 0xf)  << 24)
#define EIM_CSnL_WEA(x)  (((x) & 0xf)  << 20)
#define EIM_CSnL_WEN(x)  (((x) & 0xf)  << 16)
#define EIM_CSnL_CSA(x)  (((x) & 0xf)  << 12)
#define EIM_CSnU_EBC     (1 << 11)
#define EIM_CSnL_DSZ(x)  (((x) & 0x7)  <<  8)
#define EIM_CSnU_SP      (1 << 6)
#define EIM_CSnU_WP      (1 << 4)
#define EIM_CSnU_CSEN    (1 << 0)
#define EIM_CSnU_PA      (1 << 1)

#endif				// _IMX_REGS_H
