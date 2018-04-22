#ifndef _ARCH_CM3_NXP_MACH_LPC_SC_H_
#define _ARCH_CM3_NXP_MACH_LPC_SC_H_

/*
 * Copyright 2011 by egnite GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 */

/*!
 * \file arch/cm3/nxp/mach/lpc_sc.h
 * \brief LPC system control.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

/*!
 * \addtogroup xgNutArchArmv7mLpcSc
 */
/*@{*/

/*! \name External Interrupt Flag Register */
/*@{*/
#define SC_EXTINT_OFF   0x00000140  /*!< \brief EXTINT register offset. */
#define SC_EXTINT   (LPC_SC_BASE + SC_EXTINT_OFF) /*!< \brief EXTINT register address. */
#define SC_EINT0    (1 << 0)        /*!< \brief External interrupt 0. */
#define SC_EINT1    (1 << 1)        /*!< \brief External interrupt 1. */
#define SC_EINT2    (1 << 2)        /*!< \brief External interrupt 2. */
#define SC_EINT3    (1 << 3)        /*!< \brief External interrupt 3. */
/*@}*/

/*! \name External Interrupt Mode Register */
/*@{*/
#define SC_EXTMODE_OFF  0x00000148
#define SC_EXTMODE      (LPC_SC_BASE + SC_EXTMODE_OFF)
#define SC_EXTMODE0     (1 << 0)    /*!< \brief Interrupt 0 is edge sensitive. */
#define SC_EXTMODE1     (1 << 1)    /*!< \brief Interrupt 1 is edge sensitive. */
#define SC_EXTMODE2     (1 << 2)    /*!< \brief Interrupt 2 is edge sensitive. */
#define SC_EXTMODE3     (1 << 3)    /*!< \brief Interrupt 3 is edge sensitive. */
/*@}*/

/*! \name External Interrupt Polarity Register */
/*@{*/
#define SC_EXTPOLAR_OFF 0x0000014C
#define SC_EXTPOLAR     (LPC_SC_BASE + SC_EXTPOLAR_OFF)
#define SC_EXTPOLAR0    (1 << 0)    /*!< \brief Interrupt 0 is high or rising edge active. */
#define SC_EXTPOLAR1    (1 << 1)    /*!< \brief Interrupt 1 is high or rising edge active. */
#define SC_EXTPOLAR2    (1 << 2)    /*!< \brief Interrupt 2 is high or rising edge active. */
#define SC_EXTPOLAR3    (1 << 3)    /*!< \brief Interrupt 3 is high or rising edge active. */
/*@}*/

/*! \name Reset Source Identification Register */
/*@{*/
#define SC_RSID_OFF     0x00000180  /*!< \brief RSID register offset. */
#define SC_RSID     (LPC_SC_BASE + SC_RSID_OFF) /*!< \brief RSID register address. */
#define SC_RSID_POR     (1 << 0)    /*!< \brief Power on reset. */
#define SC_RSID_EXTR    (1 << 1)    /*!< \brief External reset. */
#define SC_RSID_WDTR    (1 << 2)    /*!< \brief Watchdog reset. */
#define SC_RSID_BODR    (1 << 3)    /*!< \brief Brown out detection. */
/*@}*/

/*! \name System Control and Status Register */
/*@{*/
#define SC_SCS_OFF      0x000001A0
#define SC_SCS          (LPC_SC_BASE + SC_SCS_OFF)
#define SC_OSCRANGE     (1 << 4)    /*!< \brief Range is 15 to 25 MHz. */
#define SC_OSCEN        (1 << 5)    /*!< \brief Oscillator enabled. */
#define SC_OSCSTAT      (1 << 6)    /*!< \brief Oscillator ready. */
/*@}*/

/*! \name  Clock Source Select Register */
/*@{*/
#define SC_CLKSRCSEL_OFF    0x0000010C
#define SC_CLKSRCSEL        (LPC_SC_BASE + SC_CLKSRCSEL_OFF)
#define SC_CLKSRC           0x00000003
#define SC_CLKSRC_RCCLK     0x00000000
#define SC_CLKSRC_MCLK      0x00000001
#define SC_CLKSRC_RTCCLK    0x00000002
/*@}*/

/*! \name PLL Control Registers */
/*@{*/
#define SC_PLL0CON_OFF      0x00000080
#define SC_PLL0CON          (LPC_SC_BASE + SC_PLL0CON_OFF)
#define SC_PLL1CON_OFF      0x000000A0
#define SC_PLL1CON          (LPC_SC_BASE + SC_PLL1CON_OFF)
#define SC_PLLE             (1 << 0)
#define SC_PLLC             (1 << 1)
/*@}*/

/*! \name PLL Configuration Registers */
/*@{*/
#define SC_PLL0CFG_OFF      0x00000084
#define SC_PLL0CFG          (LPC_SC_BASE + SC_PLL0CFG_OFF)
#define SC_PLL1CFG_OFF      0x000000A4
#define SC_PLL1CFG          (LPC_SC_BASE + SC_PLL1CFG_OFF)
#define SC_MSEL_LSB         0
#define SC_MSEL             0x00007FFF
#define SC_NSEL_LSB         16
#define SC_NSEL             0x00FF0000
/*@}*/

/*! \name PLL Status Registers */
/*@{*/
#define SC_PLL0STAT_OFF     0x00000088
#define SC_PLL0STAT         (LPC_SC_BASE + SC_PLL0STAT_OFF)
#define SC_PLL1STAT_OFF     0x000000A8
#define SC_PLL1STAT         (LPC_SC_BASE + SC_PLL1STAT_OFF)
#define SC_PLLE_STAT        (1 << 24)
#define SC_PLLC_STAT        (1 << 25)
#define SC_PLOCK            (1 << 26)
/*@}*/

/*! \name PLL Feed Registers */
/*@{*/
#define SC_PLL0FEED_OFF     0x0000008C
#define SC_PLL0FEED         (LPC_SC_BASE + SC_PLL0FEED_OFF)
#define SC_PLL1FEED_OFF     0x000000AC
#define SC_PLL1FEED         (LPC_SC_BASE + SC_PLL1FEED_OFF)
#define PLLFEED_FEED1       0xAA
#define PLLFEED_FEED2       0x55
/*@}*/

/*! \name CPU Clock Configuration Register */
/*@{*/
#define SC_CCLKCFG_OFF      0x00000104
#define SC_CCLKCFG          (LPC_SC_BASE + SC_CCLKCFG_OFF)
#define SC_CCLKSEL          0x000000FF
#define SC_CCLKSEL_LSB      0
/*@}*/

/*! \name USB Clock Configuration Register */
/*@{*/
#define SC_USBCLKCFG_OFF    0x00000108
#define SC_USBCLKCFG        (LPC_SC_BASE + SC_USBCLKCFG_OFF)
#define SC_USBSEL           0x0000000F
#define SC_USBSEL_LSB       0
/*@}*/

/*! \name Peripheral Clock Selection Register 0 */
/*@{*/
#define SC_PCLKSEL0_OFF     0x000001A8
#define SC_PCLKSEL0         (LPC_SC_BASE + SC_PCLKSEL0_OFF)
#define SC_PCLK_DIV1        0x1
#define SC_PCLK_DIV2        0x2
#define SC_PCLK_DIV4        0x0
#define SC_PCLK_DIV8        0x3
#define SC_PCLK_WDT_LSB     0
#define SC_PCLK_WDT         (0x3 << SC_PCLK_WDT_LSB)
#define SC_PCLK_TIMER0_LSB  2
#define SC_PCLK_TIMER0      (0x3 << SC_PCLK_TIMER0_LSB)
#define SC_PCLK_TIMER1_LSB  4
#define SC_PCLK_TIMER1      (0x3 << SC_PCLK_TIMER1_LSB)
#define SC_PCLK_UART0_LSB   6
#define SC_PCLK_UART0       (0x3 << SC_PCLK_UART0_LSB)
#define SC_PCLK_UART1_LSB   8
#define SC_PCLK_UART1       (0x3 << SC_PCLK_UART1_LSB)
#define SC_PCLK_PWM1_LSB    12
#define SC_PCLK_PWM1        (0x3 << SC_PCLK_PWM1_LSB)
#define SC_PCLK_I2C0_LSB    14
#define SC_PCLK_I2C0        (0x3 << SC_PCLK_I2C0_LSB)
#define SC_PCLK_SPI_LSB     16
#define SC_PCLK_SPI         (0x3 << SC_PCLK_SPI_LSB)
#define SC_PCLK_SSP1_LSB    20
#define SC_PCLK_SSP1        (0x3 << SC_PCLK_SSP1_LSB)
#define SC_PCLK_DAC_LSB     22
#define SC_PCLK_DAC         (0x3 << SC_PCLK_DAC_LSB)
#define SC_PCLK_ADC_LSB     24
#define SC_PCLK_ADC         (0x3 << SC_PCLK_ADC_LSB)
#define SC_PCLK_CAN1_LSB    26
#define SC_PCLK_CAN1        (0x3 << SC_PCLK_CAN1_LSB)
#define SC_PCLK_CAN2_LSB    28
#define SC_PCLK_CAN2        (0x3 << SC_PCLK_CAN2_LSB)
#define SC_PCLK_ACF_LSB     30
#define SC_PCLK_ACF         (0x3 << SC_PCLK_ACF_LSB)
/*@}*/


/*! \name Peripheral Clock Selection Register 1 */
/*@{*/
#define SC_PCLKSEL1_OFF     0x000001AC
#define SC_PCLKSEL1         (LPC_SC_BASE + SC_PCLKSEL1_OFF)
#define SC_PCLK_QEI_LSB     0
#define SC_PCLK_QEI         (0x3 << SC_PCLK_QEI_LSB)
#define SC_PCLK_GPIOINT_LSB 2
#define SC_PCLK_GPIOINT     (0x3 << SC_PCLK_GPIOINT_LSB)
#define SC_PCLK_PCB_LSB     4
#define SC_PCLK_PCB         (0x3 << SC_PCLK_PCB_LSB)
#define SC_PCLK_I2C1_LSB    6
#define SC_PCLK_I2C1        (0x3 << SC_PCLK_I2C1_LSB)
#define SC_PCLK_SSP0_LSB    10
#define SC_PCLK_SSP0        (0x3 << SC_PCLK_SSP0_LSB)
#define SC_PCLK_TIMER2_LSB  12
#define SC_PCLK_TIMER2      (0x3 << SC_PCLK_TIMER2_LSB)
#define SC_PCLK_TIMER3_LSB  14
#define SC_PCLK_TIMER3      (0x3 << SC_PCLK_TIMER3_LSB)
#define SC_PCLK_UART2_LSB   16
#define SC_PCLK_UART2       (0x3 << SC_PCLK_UART2_LSB)
#define SC_PCLK_UART3_LSB   18
#define SC_PCLK_UART3       (0x3 << SC_PCLK_UART3_LSB)
#define SC_PCLK_I2C2_LSB    20
#define SC_PCLK_I2C2        (0x3 << SC_PCLK_I2C2_LSB)
#define SC_PCLK_I2S_LSB     22
#define SC_PCLK_I2S         (0x3 << SC_PCLK_I2S_LSB)
#define SC_PCLK_RIT_LSB     26
#define SC_PCLK_RIT         (0x3 << SC_PCLK_RIT_LSB)
#define SC_PCLK_SYSCON_LSB  28
#define SC_PCLK_SYSCON      (0x3 << SC_PCLK_SYSCON_LSB)
#define SC_PCLK_MC_LSB      30
#define SC_PCLK_MC          (0x3 << SC_PCLK_MC_LSB)
/*@}*/

/*! \name Power Mode Control Register */
/*@{*/
#define SC_PCON_OFF         0x000000C0
#define SC_PCON             (LPC_SC_BASE + SC_PCON_OFF)
#define SC_PM0              (1 << 0)
#define SC_PM1              (1 << 1)
#define SC_BODRPM           (1 << 2)
#define SC_BOGD             (1 << 3)
#define SC_BORD             (1 << 4)
#define SC_SMFLAG           (1 << 8)
#define SC_DSFLAG           (1 << 9)
#define SC_PDFLAG           (1 << 10)
#define SC_DPDFLAG          (1 << 11)
/*@}*/

/*! \name Peripheral Power Control Register */
/*@{*/
#define SC_PCONP_OFF        0x000000C4
#define SC_PCONP            (LPC_SC_BASE + SC_PCONP_OFF)
#define SC_PCTIM0           (1 << 1)
#define SC_PCTIM1           (1 << 2)
#define SC_PCUART0          (1 << 3)
#define SC_PCUART1          (1 << 4)
#define SC_PCPWM1           (1 << 6)
#define SC_PCI2C0           (1 << 7)
#define SC_PCSPI            (1 << 8)
#define SC_PCRTC            (1 << 9)
#define SC_PCSSP1           (1 << 10)
#define SC_PCADC            (1 << 12)
#define SC_PCCAN1           (1 << 13)
#define SC_PCCAN2           (1 << 14)
#define SC_PCGPIO           (1 << 15)
#define SC_PCRIT            (1 << 16)
#define SC_PCMCPWM          (1 << 17)
#define SC_PCQEI            (1 << 18)
#define SC_PCI2C1           (1 << 19)
#define SC_PCSSP0           (1 << 21)
#define SC_PCTIM2           (1 << 22)
#define SC_PCTIM3           (1 << 23)
#define SC_PCUART2          (1 << 24)
#define SC_PCUART3          (1 << 25)
#define SC_PCI2C2           (1 << 26)
#define SC_PCI2S            (1 << 27)
#define SC_PCGPDMA          (1 << 29)
#define SC_PCENET           (1 << 30)
#define SC_PCUSB            (1 << 31)
/*@}*/

/*! \name Clock Output Configuration Register */
/*@{*/
#define SC_CLKOUTCFG_OFF    0x000001C8
#define SC_CLKOUTCFG        (LPC_SC_BASE + SC_CLKOUTCFG_OFF)
#define SC_CLKOUTSEL        0x0000000F
#define SC_CLKOUTSEL_CCLK   0x0 /*!< CPU clock. */
#define SC_CLKOUTSEL_MCLK   0x1 /*!< Main oscillator. */
#define SC_CLKOUTSEL_RCCLK  0x2 /*!< RC oscillator. */
#define SC_CLKOUTSEL_USBCLK 0x3 /*!< USB clock. */
#define SC_CLKOUTSEL_RTCCLK 0x4 /*!< RTC clock. */
#define SC_CLKOUTDIV        0x000000F0
#define SC_CLKOUTDIV_LSB    4
#define SC_CLKOUT_EN        (1 << 8)
#define SC_CLKOUT_ACT       (1 << 9)
/*@}*/

/*! \name Flash Accelerator Configuration Register */
/*@{*/
#define SC_FLASHCFG_OFF     0x00000000
#define SC_FLASHCFG         (LPC_SC_BASE + SC_FLASHCFG_OFF)
#define SC_FLASHTIM_LSB     12
#define SC_FLASHTIM         0x0000F000
/*@}*/


/*@}*/

#endif
