#ifndef _ARCH_CM3_NXP_MACH_LPC1768_H_
#define _ARCH_CM3_NXP_MACH_LPC1768_H_

/*
 * Copyright 2011 by egnite GmbH
 *
 * All rights reserved.
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
 * \file arch/cm3/nxp/mach/lpc1768.h
 * \brief LPC1768 peripherals
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#ifndef LPC_FLASH_BASE
/* Base addresses */
#define LPC_FLASH_BASE        0x00000000
#define LPC_RAM_BASE          0x10000000
#define LPC_GPIO_BASE         0x2009C000
#define LPC_APB0_BASE         0x40000000
#define LPC_APB1_BASE         0x40080000
#define LPC_AHB_BASE          0x50000000
#define LPC_CM3_BASE          0xE0000000

/* APB0 peripherals */
#define LPC_WDT_BASE          (LPC_APB0_BASE + 0x00000000)
#define LPC_TIM0_BASE         (LPC_APB0_BASE + 0x00004000)
#define LPC_TIM1_BASE         (LPC_APB0_BASE + 0x00008000)
#define LPC_UART0_BASE        (LPC_APB0_BASE + 0x0000C000)
#define LPC_UART1_BASE        (LPC_APB0_BASE + 0x00010000)
#define LPC_PWM1_BASE         (LPC_APB0_BASE + 0x00018000)
#define LPC_I2C0_BASE         (LPC_APB0_BASE + 0x0001C000)
#define LPC_SPI_BASE          (LPC_APB0_BASE + 0x00020000)
#define LPC_RTC_BASE          (LPC_APB0_BASE + 0x00024000)
#define LPC_GPIOINT_BASE      (LPC_APB0_BASE + 0x00028080)
#define LPC_PINCON_BASE       (LPC_APB0_BASE + 0x0002C000)
#define LPC_SSP1_BASE         (LPC_APB0_BASE + 0x00030000)
#define LPC_ADC_BASE          (LPC_APB0_BASE + 0x00034000)
#define LPC_CANAF_RAM_BASE    (LPC_APB0_BASE + 0x00038000)
#define LPC_CANAF_BASE        (LPC_APB0_BASE + 0x0003C000)
#define LPC_CANCR_BASE        (LPC_APB0_BASE + 0x00040000)
#define LPC_CAN1_BASE         (LPC_APB0_BASE + 0x00044000)
#define LPC_CAN2_BASE         (LPC_APB0_BASE + 0x00048000)
#define LPC_I2C1_BASE         (LPC_APB0_BASE + 0x0005C000)

/* APB1 peripherals */
#define LPC_SSP0_BASE         (LPC_APB1_BASE + 0x00008000)
#define LPC_DAC_BASE          (LPC_APB1_BASE + 0x0000C000)
#define LPC_TIM2_BASE         (LPC_APB1_BASE + 0x00010000)
#define LPC_TIM3_BASE         (LPC_APB1_BASE + 0x00014000)
#define LPC_UART2_BASE        (LPC_APB1_BASE + 0x00018000)
#define LPC_UART3_BASE        (LPC_APB1_BASE + 0x0001C000)
#define LPC_I2C2_BASE         (LPC_APB1_BASE + 0x00020000)
#define LPC_I2S_BASE          (LPC_APB1_BASE + 0x00028000)
#define LPC_RIT_BASE          (LPC_APB1_BASE + 0x00030000)
#define LPC_MCPWM_BASE        (LPC_APB1_BASE + 0x00038000)
#define LPC_QEI_BASE          (LPC_APB1_BASE + 0x0003C000)
#define LPC_SC_BASE           (LPC_APB1_BASE + 0x0007C000)

/* AHB peripherals */
#define LPC_EMAC_BASE         (LPC_AHB_BASE  + 0x00000000)
#define LPC_GPDMA_BASE        (LPC_AHB_BASE  + 0x00004000)
#define LPC_GPDMACH0_BASE     (LPC_AHB_BASE  + 0x00004100)
#define LPC_GPDMACH1_BASE     (LPC_AHB_BASE  + 0x00004120)
#define LPC_GPDMACH2_BASE     (LPC_AHB_BASE  + 0x00004140)
#define LPC_GPDMACH3_BASE     (LPC_AHB_BASE  + 0x00004160)
#define LPC_GPDMACH4_BASE     (LPC_AHB_BASE  + 0x00004180)
#define LPC_GPDMACH5_BASE     (LPC_AHB_BASE  + 0x000041A0)
#define LPC_GPDMACH6_BASE     (LPC_AHB_BASE  + 0x000041C0)
#define LPC_GPDMACH7_BASE     (LPC_AHB_BASE  + 0x000041E0)
#define LPC_USB_BASE          (LPC_AHB_BASE  + 0x0000C000)

/* GPIOs */
#define LPC_GPIO0_BASE        (LPC_GPIO_BASE + 0x00000000)
#define LPC_GPIO1_BASE        (LPC_GPIO_BASE + 0x00000020)
#define LPC_GPIO2_BASE        (LPC_GPIO_BASE + 0x00000040)
#define LPC_GPIO3_BASE        (LPC_GPIO_BASE + 0x00000060)
#define LPC_GPIO4_BASE        (LPC_GPIO_BASE + 0x00000080)
#endif

#include <arch/cm3/nxp/mach/lpc_adc.h>
#include <arch/cm3/nxp/mach/lpc_can.h>
#include <arch/cm3/nxp/mach/lpc_dac.h>
#include <arch/cm3/nxp/mach/lpc_emac.h>
/* TODO #include <arch/cm3/nxp/mach/lpc_gpdma.h> */
#include <arch/cm3/nxp/mach/lpc_gpio.h>
#include <arch/cm3/nxp/mach/lpc_i2c.h>
/* TODO #include <arch/cm3/nxp/mach/lpc_i2s.h> */
/* TODO #include <arch/cm3/nxp/mach/lpc_mcpwm.h> */
#include <arch/cm3/nxp/mach/lpc_pincon.h>
/* TODO #include <arch/cm3/nxp/mach/lpc_pwm.h> */
/* TODO #include <arch/cm3/nxp/mach/lpc_qei.h> */
/* TODO #include <arch/cm3/nxp/mach/lpc_rit.h> */
/* TODO #include <arch/cm3/nxp/mach/lpc_rtc.h> */
#include <arch/cm3/nxp/mach/lpc_sc.h>
#include <arch/cm3/nxp/mach/lpc_spi.h>
#include <arch/cm3/nxp/mach/lpc_ssp.h>
#include <arch/cm3/nxp/mach/lpc_tim.h>
#include <arch/cm3/nxp/mach/lpc_uart.h>
/* TODO #include <arch/cm3/nxp/mach/lpc_usb.h> */
/* TODO #include <arch/cm3/nxp/mach/lpc_wdt.h> */

/*! \name Interrupt IDs */
/*@{*/
#define WDT_ID      0
#define TIMER0_ID   1
#define TIMER1_ID   2
#define TC2_ID      3
#define TIMER3_ID   4
#define UART0_ID    5
#define UART1_ID    6
#define UART2_ID    7
#define UART3_ID    8
#define PWM1_ID     9
#define I2C0_ID     10
#define I2C1_ID     11
#define I2C2_ID     12
#define SPI_ID      13
#define SSP0_ID     14
#define SSP1_ID     15
#define PLL0_ID     16
#define RTC_ID      17
#define EINT0_ID    18
#define EINT1_ID    19
#define EINT2_ID    20
#define EINT3_ID    21
#define ADC_ID      22
#define BOD_ID      23
#define USB_ID      24
#define CAN_ID      25
#define DMA_ID      26
#define I2S_ID      27
#define ENET_ID     28
#define RIT_ID      29
#define MCPWM_ID    30
#define QEI_ID      31
#define PLL1_ID     32
#define USBACTIVITY_ID      33
#define CANACTIVITY_ID      34
/*@}*/


/*! \name External Clock Multiplexing */
/*@{*/
#define PS7_P3_26_STCLK         (1 << 20)
#define PS3_P1_27_CLKOUT        (1 << 22)
/*@}*/

/*! \name External Interrupt Multiplexing */
/*@{*/
#define PS4_P2_10_NMI           (2 << 20)
#define PS4_P2_10_EINT0         (1 << 20)
#define PS4_P2_11_EINT1         (1 << 22)
#define PS4_P2_12_EINT2         (1 << 24)
#define PS4_P2_13_EINT3         (1 << 26)
/*@}*/

/*! \name Timer0/Counter0 Peripheral Multiplexing */
/*@{*/
#define PS3_P1_26_CAP0_0        (3 << 20)
#define PS3_P1_27_CAP0_1        (3 << 22)
#define PS3_P1_28_MAT0_0        (3 << 24)
#define PS7_P3_25_MAT0_0        (2 << 18)
#define PS3_P1_29_MAT0_1        (3 << 26)
#define PS7_P3_26_MAT0_1        (2 << 20)
/*@}*/

/*! \name Timer1/Counter1 Peripheral Multiplexing */
/*@{*/
#define PS3_P1_18_CAP1_0        (3 << 4)
#define PS3_P1_19_CAP1_1        (3 << 6)
#define PS3_P1_22_MAT1_0        (3 << 12)
#define PS3_P1_25_MAT1_1        (3 << 18)
/*@}*/

/*! \name Timer2/Counter2 Peripheral Multiplexing */
/*@{*/
#define PS0_P0_4_CAP2_0         (3 << 8)
#define PS0_P0_5_CAP2_1         (3 << 10)
#define PS0_P0_6_MAT2_0         (3 << 12)
#define PS9_P4_28_MAT2_0        (2 << 24)
#define PS0_P0_7_MAT2_1         (3 << 14)
#define PS9_P4_29_MAT2_1        (2 << 26)
#define PS0_P0_8_MAT2_2         (3 << 16)
#define PS0_P0_9_MAT2_3         (3 << 18)
/*@}*/

/*! \name Timer3/Counter3 Peripheral Multiplexing */
/*@{*/
#define PS1_P0_23_CAP3_0        (3 << 14)
#define PS1_P0_24_CAP3_1        (3 << 16)
#define PS0_P0_10_MAT3_0        (3 << 20)
#define PS0_P0_11_MAT3_1        (3 << 22)
/*@}*/

/*! \name ADC Peripheral Multiplexing */
/*@{*/
#define PS1_P0_23_AD0_0         (1 << 14)
#define PS1_P0_24_AD0_1        (1 << 16)
#define PS1_P0_25_AD0_2        (1 << 18)
#define PS1_P0_26_AD0_3        (1 << 20)
#define PS3_P1_30_AD0_4         (3 << 28)
#define PS3_P1_31_AD0_5         (3 << 30)
#define PS0_P0_2_AD0_7          (2 << 4)
#define PS0_P0_3_AD0_6          (2 << 6)
/*@}*/

/*! \name DAC Peripheral Multiplexing */
/*@{*/
#define PS1_P0_26_AOUT          (2 << 20)
/*@}*/

/*! \name SSP0 Peripheral Multiplexing */
/*@{*/
#define PS1_P0_17_MISO0         (2 << 2)
#define PS3_P1_23_MISO0         (3 << 14)
#define PS1_P0_18_MOSI0         (2 << 4)
#define PS3_P1_24_MOSI0         (3 << 16)
#define PS0_P0_15_SCK0          (2 << 30)
#define PS3_P1_20_SCK0          (3 << 8)
#define PS1_P0_16_SSEL0         (2 << 0)
#define PS3_P1_21_SSEL0         (3 << 10)
/*@}*/

/*! \name SSP1 Peripheral Multiplexing */
/*@{*/
#define PS0_P0_8_MISO1          (2 << 16)
#define PS0_P0_9_MOSI1          (2 << 18)
#define PS0_P0_7_SCK1           (2 << 14)
#define PS3_P1_31_SCK1          (2 << 30)
#define PS0_P0_6_SSEL1          (2 << 12)
/*@}*/

/*! \name SPI Peripheral Multiplexing */
/*@{*/
#define PS1_P0_17_MISO          (3 << 2)
#define PS1_P0_18_MOSI          (3 << 4)
#define PS0_P0_15_SCK           (3 << 30)
#define PS1_P0_16_SSEL          (3 << 0)
/*@}*/

/*! \name I2C0 Peripheral Multiplexing */
/*@{*/
#define PS1_P0_27_SDA0          (1 << 22)
#define PS1_P0_28_SCL0          (1 << 24)
/*@}*/

/*! \name I2C1 Peripheral Multiplexing */
/*@{*/
#define PS0_P0_0_SDA1           (3 << 0)
#define PS1_P0_19_SDA1          (3 << 6)
#define PS0_P0_1_SCL1           (3 << 2)
#define PS1_P0_20_SCL1          (3 << 8)
/*@}*/

/*! \name I2C2 Peripheral Multiplexing */
/*@{*/
#define PS0_P0_10_SDA2          (2 << 20)
#define PS0_P0_11_SCL2          (2 << 22)
/*@}*/

/*! \name I2S Peripheral Multiplexing */
/*@{*/
#define PS0_P0_6_I2SRX_SDA      (1 << 12)
#define PS1_P0_25_I2SRX_SDA     (2 << 18)
#define PS0_P0_5_I2SRX_WS       (1 << 10)
#define PS1_P0_24_I2SRX_WS      (2 << 16)
#define PS0_P0_4_I2SRX_CLK      (1 << 8)
#define PS1_P0_23_I2SRX_CLK     (2 << 14)
#define PS9_P4_28_RX_MCLK       (1 << 24)
#define PS0_P0_9_I2STX_SDA      (1 << 18)
#define PS4_P2_13_I2STX_SDA     (3 << 26)
#define PS0_P0_8_I2STX_WS       (1 << 16)
#define PS4_P2_12_I2STX_WS      (3 << 24)
#define PS0_P0_7_I2STX_CLK      (1 << 14)
#define PS4_P2_11_I2STX_CLK     (3 << 22)
#define PS9_P4_29_TX_MCLK       (1 << 26)
/*@}*/

/*! \name PWM1 Peripheral Multiplexing */
/*@{*/
#define PS3_P1_18_PWM1_1        (2 << 4)
#define PS4_P2_0_PWM1_1         (1 << 0)
#define PS3_P1_20_PWM1_2        (2 << 8)
#define PS4_P2_1_PWM1_2         (1 << 2)
#define PS7_P3_25_PWM1_2        (3 << 18)
#define PS3_P1_21_PWM1_3        (2 << 10)
#define PS4_P2_2_PWM1_3         (1 << 4)
#define PS7_P3_26_PWM1_3        (3 << 20)
#define PS3_P1_23_PWM1_4        (2 << 14)
#define PS4_P2_3_PWM1_4         (1 << 6)
#define PS3_P1_24_PWM1_5        (2 << 16)
#define PS4_P2_4_PWM1_5         (1 << 8)
#define PS3_P1_26_PWM1_6        (2 << 20)
#define PS4_P2_5_PWM1_6         (1 << 10)
#define PS3_P1_28_PCAP1_0       (2 << 24)
#define PS4_P2_6_PCAP1_0        (1 << 12)
#define PS3_P1_29_PCAP1_1       (2 << 26)
/*@}*/

/*! \name Motor Control Channel 0 Peripheral Multiplexing */
/*@{*/
#define PS3_P1_19_MCOA0         (1 << 6)
#define PS3_P1_22_MCOB0         (1 << 12)
#define PS3_P1_20_MCI0          (1 << 8)
#define PS3_P1_21_MCABORT       (1 << 10)
/*@}*/

/*! \name Motor Control Channel 1 Peripheral Multiplexing */
/*@{*/
#define PS3_P1_25_MCOA1         (1 << 18)
#define PS3_P1_26_MCOB1         (1 << 20)
#define PS3_P1_23_MCI1          (1 << 14)
/*@}*/

/*! \name Motor Control Channel 2 Peripheral Multiplexing */
/*@{*/
#define PS3_P1_28_MCOA2         (1 << 24)
#define PS3_P1_29_MCOB2         (1 << 26)
#define PS3_P1_24_MCI2          (1 << 16)
/*@}*/

/*! \name UART0 Peripheral Multiplexing */
/*@{*/
#define PS0_P0_2_TXD0           (1 << 4)
#define PS0_P0_3_RXD0           (1 << 6)
/*@}*/

/*! \name UART1 Peripheral Multiplexing */
/*@{*/
#define PS0_P0_15_TXD1          (1 << 30)
#define PS4_P2_0_TXD1           (2 << 0)
#define PS1_P0_16_RXD1          (1 << 0)
#define PS4_P2_1_RXD1           (2 << 2)
#define PS1_P0_17_CTS1          (1 << 2)
#define PS4_P2_2_CTS1           (2 << 4)
#define PS1_P0_22_RTS1          (1 << 12)
#define PS4_P2_7_RTS1           (2 << 14)
#define PS1_P0_19_DSR1          (1 << 6)
#define PS4_P2_4_DSR1           (2 << 8)
#define PS1_P0_20_DTR1          (1 << 8)
#define PS4_P2_5_DTR1           (2 << 10)
#define PS1_P0_18_DCD1          (1 << 4)
#define PS4_P2_3_DCD1           (2 << 6)
#define PS1_P0_21_RI1           (1 << 10)
#define PS4_P2_6_RI1            (2 << 12)
/*@}*/

/*! \name UART2 Peripheral Multiplexing */
/*@{*/
#define PS0_P0_10_TXD2          (1 << 20)
#define PS4_P2_8_TXD2           (2 << 16)
#define PS0_P0_11_RXD2          (1 << 22)
#define PS4_P2_9_RXD2           (2 << 18)
/*@}*/

/*! \name UART3 Peripheral Multiplexing */
/*@{*/
#define PS0_P0_0_TXD3           (2 << 0)
#define PS1_P0_25_TXD3          (3 << 18)
#define PS9_P4_28_TXD3          (3 << 24)
#define PS0_P0_1_RXD3           (2 << 2)
#define PS1_P0_26_RXD3          (3 << 20)
#define PS9_P4_29_RXD3          (3 << 26)
/*@}*/

/*! \name CAN1 Peripheral Multiplexing */
/*@{*/
#define PS0_P0_1_TD1            (1 << 2)
#define PS1_P0_22_TD1           (3 << 12)
#define PS0_P0_0_RD1            (1 << 0)
#define PS1_P0_21_RD1           (3 << 10)
/*@}*/

/*! \name CAN2 Peripheral Multiplexing */
/*@{*/
#define PS0_P0_5_TD2            (2 << 10)
#define PS4_P2_8_TD2            (1 << 16)
#define PS0_P0_4_RD2            (2 << 8)
#define PS4_P2_7_RD2            (1 << 14)
/*@}*/

/*! \name EMAC Peripheral Multiplexing */
/*@{*/
#define PS2_P1_0_ENET_TXD0      (1 << 0)
#define PS2_P1_1_ENET_TXD1      (1 << 2)
#define PS2_P1_9_ENET_RXD0      (1 << 18)
#define PS2_P1_10_ENET_RXD1     (1 << 20)
#define PS2_P1_4_ENET_TX_EN     (1 << 8)
#define PS2_P1_14_ENET_RX_ER    (1 << 28)
#define PS2_P1_8_ENET_CRS       (1 << 16)
#define PS2_P1_15_ENET_REF_CLK  (1 << 30)
#define PS3_P1_16_ENET_MDC      (1 << 0)
#define PS4_P2_8_ENET_MDC       (3 << 16)
#define PS3_P1_17_ENET_MDIO     (1 << 2)
#define PS4_P2_9_ENET_MDIO      (3 << 18)
/*@}*/

/*! \name USB Peripheral Multiplexing */
/*@{*/
#define PS1_P0_29_USB_DP        (1 << 26)
#define PS1_P0_30_USB_DM        (1 << 28)
#define PS1_P0_28_USB_SCL       (2 << 24)
#define PS1_P0_27_USB_SDA       (2 << 22)
#define PS3_P1_18_USB_UP_LED    (1 << 4)
#define PS3_P1_19_USB_PPWR      (2 << 6)
#define PS3_P1_22_USB_PWRD      (2 << 12)
#define PS4_P2_9_USB_CONNECT    (1 << 18)
#define PS3_P1_27_USB_OVRCR     (2 << 22)
#define PS3_P1_30_VBUS          (2 << 28)
/*@}*/


/*! \name Port 0 Peripheral Pins */
/*@{*/
#define P0_0_RD1_TXD3_SDA1          0
#define P0_1_TD1_RXD3_SCL1          1
#define P0_2_TXD0_AD0_7             2
#define P0_3_RXD0_AD0_6             3
#define P0_4_I2SRX_CLK_RD2_CAP2_0   4
#define P0_5_I2SRX_WS_TD2_CAP2_1    5
#define P0_6_I2SRX_SDA_SSEL1_MAT2_0 6
#define P0_7_I2STX_CLK_SCK1_MAT2_1  7
#define P0_8_I2STX_WS_MISO1_MAT2_2  8
#define P0_9_I2STX_SDA_MOSI1_MAT2_3 9
#define P0_10_TXD2_SDA2_MAT3_0      10
#define P0_11_RXD2_SCL2_MAT3_1      11
#define P0_15_TXD1_SCK0_SCK         15
#define P0_16_RXD1_SSEL0_SSEL       16
#define P0_17_CTS1_MISO0_MISO       17
#define P0_18_DCD1_MOSI0_MOSI       18
#define P0_19_DSR1_SDA1             19
#define P0_20_DTR1_SCL1             20
#define P0_21_RI1_RD1               21
#define P0_22_RTS1_TD1              22
#define P0_23_AD0_0_I2SRX_CLK_CAP3_0 23
#define P0_24_AD0_1_I2SRX_WS_CAP3_1 24
#define P0_25_AD0_2_I2SRX_SDA_TXD3  25
#define P0_26_AD0_3_AOUT_RXD3       26
#define P0_27_SDA0_USB_SDA          27
#define P0_28_SCL0_USB_SCL          28
#define P0_29_USB_DP                29
#define P0_30_USB_DM                30
/*@}*/

/*! \name Port 1 Peripheral Pins */
/*@{*/
#define P1_0_ENET_TXD0              0
#define P1_1_ENET_TXD1              1
#define P1_4_ENET_TX_EN             4
#define P1_8_ENET_CRS               8
#define P1_9_ENET_RXD0              9
#define P1_10_ENET_RXD1             10
#define P1_14_ENET_RX_ER            14
#define P1_15_ENET_REF_CLK          15
#define P1_16_ENET_MDC              16
#define P1_17_ENET_MDIO             17
#define P1_18_USB_UP_LED_PWM1_1_CAP1_0 18
#define P1_19_MCOA0_USB_PPWR_CAP1_1 19
#define P1_20_MCI0_PWM1_2_SCK0      20
#define P1_21_MCABORT_PWM1_3_SSEL0  21
#define P1_22_MCOB0_USB_PWRD_MAT1_0 22
#define P1_23_MCI1_PWM1_4_MISO0     23
#define P1_24_MCI2_PWM1_5_MOSI0     24
#define P1_25_MCOA1_MAT1_1          25
#define P1_26_MCOB1_PWM1_6_CAP0_0   26
#define P1_27_CLKOUT_USB_OVRCR_CAP0_1 27
#define P1_28_MCOA2_PCAP1_0_MAT0_0  28
#define P1_29_MCOB2_PCAP1_1_MAT0_1  29
#define P1_30_VBUS_AD0_4            30
#define P1_31_SCK1_AD0_5            31
/*@}*/

/*! \name Port 2 Peripheral Pins */
/*@{*/
#define P2_0_PWM1_1_TXD1            0
#define P2_1_PWM1_2_RXD1            1
#define P2_2_PWM1_3_CTS1_TRACEDATA3 2
#define P2_3_PWM1_4_DCD1_TRACEDATA2 3
#define P2_4_PWM1_5_DSR1_TRACEDATA1 4
#define P2_5_PWM1_6_DTR1_TRACEDATA0 5
#define P2_6_PCAP1_0_RI1_TRACECLK   6
#define P2_7_RD2_RTS1               7
#define P2_8_TD2_TXD2_ENET_MDC      8
#define P2_9_USB_CONNECT_RXD2_ENET_MDIO 9
#define P2_10_EINT0_NMI             10
#define P2_11_EINT1_I2STX_CLK       11
#define P2_12_EINT2_I2STX_WS        12
#define P2_13_EINT3_I2STX_SDA       13
/*@}*/

/*! \name Port 3 Peripheral Pins */
/*@{*/
#define P3_25_MAT0_0_PWM1_2         25
#define P3_26_STCLK_MAT0_1_PWM1_3   26
/*@}*/

/*! \name Port 4 Peripheral Pins */
/*@{*/
#define P4_28_RX_MCLK_MAT2_0_TXD3   28
#define P4_29_TX_MCLK_MAT2_1_RXD3   29
/*@}*/

/*@}*/
#endif
