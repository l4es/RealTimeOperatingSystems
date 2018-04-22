/*
 * Copyright (C) 2013 by Ole Reinhardt (ole.reinhardt@embedded-it.de)
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

/**************************************************************************/
/* The follwing copyright notice applies to the clock initialisation code */
/************************************************************************
*
* $Id: system_LPC407x_8x.c 59014 2012-11-01 15:09:57Z anderslu $    
*      system_LPC407x_8x.c      2012-01-16
*//**
* @file    system_LPC407x_8x.c
* @file    system_LPC407x_8x.c
* @brief  CMSIS Cortex-M3, M4 Device Peripheral Access Layer Source File
*            for the NXP LPC407x_8x_177x_8x Device Series
*
*      ARM Limited (ARM) is supplying this software for use with
*      Cortex-M processor based microcontrollers.  This file can be
*      freely distributed within development tools that are supporting
*      such ARM based processors.
*
* @version  1.0
* @date    30. May. 2012
* @author  NXP MCU SW Application Team
*
* Copyright(C) 2012, NXP Semiconductor
* All rights reserved.
*
***********************************************************************
* Software that is described herein is for illustrative purposes only
* which provides customers with programming information regarding the
* products. This software is supplied "AS IS" without any warranties.
* NXP Semiconductors assumes no responsibility or liability for the
* use of the software, conveys no license or title under any patent,
* copyright, or mask work right to the product. NXP Semiconductors
* reserves the right to make changes in the software without
* notification. NXP Semiconductors also make no representation or
* warranty that such application will be suitable for the specified
* use without further testing or modification.
**********************************************************************/


/*
 * \verbatim
 * $Id: $
 * \endverbatim
 */

#include <stdint.h>

#include <cfg/arch.h>
#include <arch/cm3.h>
#include <arch/cm3/timer.h>
#include <arch/cm3/nxp/lpc407x_8x_clk.h>
#include <cfg/clock.h>

#include <arch/cm3/nxp/lpc407x_8x.h>

#define __CLK_DIV(x,y) (((y) == 0) ? 0: (x)/(y))

/*
//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
*/
/*--------------------- Clock Configuration ----------------------------------
//
//  <e>  Clock Configuration
//    <h>  System Controls and Status Register (SCS - address 0x400F C1A0)
//      <o1.0>  EMC Shift Control Bit
//          <i>    Controls how addresses are output on the EMC address pins for static memories
//          <0=>  Static CS addresses match bus width; AD[1] = 0 for 32 bit, AD[0] = 0 for 16+32 bit (Bit 0 is 0)
//          <1=>  Static CS addresses start at LSB 0 regardless of memory width (Bit 0 is 1)
//
//      <o1.1>  EMC Reset Disable Bit
//          <i>    If 0 (zero), all registers and functions of the EMC are initialized upon any reset condition
//          <i>    If 1, EMC is still retained its state through a warm reset
//          <0=>  Both EMC resets are asserted when any type of chip reset event occurs (Bit 1 is 0)
//          <1=>  Portions of EMC will only be reset by POR or BOR event (Bit 1 is 1)
//
//      <o1.2>  EMC Burst Control
//          <i>    Set to 1 to prevent multiple sequential accesses to memory via EMC static memory chip selects
//          <0=>  Burst enabled (Bit 2 is 0)
//          <1=>  Bust disbled (Bit 2 is 1)
//
//      <o1.3>  MCIPWR Active Level
//          <i>    Selects the active level for the SD card interface signal SD_PWR
//          <0=>  SD_PWR is active low (inverted output of the SD Card interface block) (Bit 3 is 0)
//          <1=>  SD_PWR is active high (follows the output of the SD Card interface block) (Bit 3 is 1)
//
//      <o1.4>  Main Oscillator Range Select
//          <0=>  In Range 1 MHz to 20 MHz (Bit 4 is 0)
//          <1=>  In Range 15 MHz to 25 MHz (Bit 4 is 1)
//
//      <o1.5>  Main Oscillator enable
//          <i>    0 (zero) means disabled, 1 means enable
//
//      <o1.6>  Main Oscillator status (Read-Only)
//    </h>
//
//    <h>  Clock Source Select Register (CLKSRCSEL - address 0x400F C10C)
//      <o2.0>  CLKSRC: Select the clock source for sysclk to PLL0 clock
//          <0=>  Internal RC oscillator (Bit 0 is 0)
//          <1=>  Main oscillator (Bit 0 is 1)
//    </h>
//
//    <e3>PLL0 Configuration (Main PLL PLL0CFG - address 0x400F C084)
//      <i>      F_in  is in the range of 1 MHz to 25 MHz
//      <i>      F_cco = (F_in * M * 2 * P) is in range of 156 MHz to 320 MHz
//      <i>      PLL out clock = (F_cco / (2 * P)) is in rane of 9.75 MHz to 160 MHz
//
//      <o4.0..4>   MSEL: PLL Multiplier Value
//            <i>        M Value
//            <1-32><#-1>
//
//      <o4.5..6>  PSEL: PLL Divider Value
//            <i>        P Value
//            <0=>       1
//            <1=>      2
//            <2=>      4
//            <3=>      8
//    </e>
//
//    <e5>PLL1 Configuration (Alt PLL PLL1CFG - address 0x400F C0A4)
//      <i>      F_in  is in the range of 1 MHz to 25 MHz
//      <i>      F_cco = (F_in * M * 2 * P) is in range of 156 MHz to 320 MHz
//      <i>      PLL out clock = (F_cco / (2 * P)) is in rane of 9.75 MHz to 160 MHz
//
//      <o6.0..4>   MSEL: PLL Multiplier Value
//            <i>        M Value
//            <1-32><#-1>
//
//      <o6.5..6>  PSEL: PLL Divider Value
//            <i>    P Value
//            <0=>  1
//            <1=>  2
//            <2=>  4
//            <3=>  8
//    </e>
//
//    <h>  CPU Clock Selection Register (CCLKSEL - address 0x400F C104)
//      <o7.0..4>  CCLKDIV: Select the value for divider of CPU clock (CCLK)
//            <i>    0: The divider is turned off. No clock will be provided to the CPU
//            <i>    n: The input clock is divided by n to produce the CPU clock
//            <0-31>
//
//      <o7.8>    CCLKSEL: Select the input to the divider of CPU clock
//            <0=>  sysclk clock is used
//            <1=>  Main PLL0 clock is used
//    </h>
//
//    <h>  USB Clock Selection Register (USBCLKSEL - 0x400F C108)
//      <o8.0..4>  USBDIV: USB clock (source PLL0) divider selection
//            <0=>  Divider is off and no clock provides to USB subsystem
//            <4=>  Divider value is 4 (The source clock is divided by 4)
//            <6=>  Divider value is 6 (The source clock is divided by 6)
//
//      <o8.8..9>  USBSEL: Select the source for USB clock divider
//            <i>    When CPU clock is selected, the USB can be accessed
//            <i>    by software but cannot perform USB functions
//            <0=>  sysclk clock (the clock input to PLL0)
//            <1=>  The clock output from PLL0
//            <2=>  The clock output from PLL1
//    </h>
//
//    <h>  EMC Clock Selection Register (EMCCLKSEL - address 0x400F C100)
//      <o9.0>  EMCDIV: Set the divider for EMC clock
//          <0=> Divider value is 1
//          <1=> Divider value is 2 (EMC clock is equal a half of input clock)
//    </h>
//
//    <h>  Peripheral Clock Selection Register (PCLKSEL - address 0x400F C1A8)
//      <o10.0..4>  PCLKDIV: APB Peripheral clock divider
//            <i>  0: The divider is turned off. No clock will be provided to APB peripherals
//            <i>  n: The input clock is divided by n to produce the APB peripheral clock
//            <0-31>
//    </h>
//
//    <h>  SPIFI Clock Selection Register (SPIFICLKSEL - address 0x400F C1B4)
//      <o11.0..4>  SPIFIDIV: Set the divider for SPIFI clock
//            <i>  0: The divider is turned off. No clock will be provided to the SPIFI
//            <i>  n: The input clock is divided by n to produce the SPIFI clock
//            <0-31>
//
//      <o11.8..9>  SPIFISEL: Select the input clock for SPIFI clock divider
//            <0=>  sysclk clock (the clock input to PLL0)
//            <1=>  The clock output from PLL0
//            <2=>  The clock output from PLL1
//    </h>
//
//    <h>  Power Control for Peripherals Register (PCONP - address 0x400F C1C8)
//      <o12.0>   PCLCD: LCD controller power/clock enable (bit 0)
//      <o12.1>   PCTIM0: Timer/Counter 0 power/clock enable (bit 1)
//      <o12.2>   PCTIM1: Timer/Counter 1 power/clock enable (bit 2)
//      <o12.3>   PCUART0: UART 0 power/clock enable (bit 3)
//      <o12.4>   PCUART1: UART 1 power/clock enable (bit 4)
//      <o12.5>   PCPWM0: PWM0 power/clock enable (bit 5)
//      <o12.6>   PCPWM1: PWM1 power/clock enable (bit 6)
//      <o12.7>   PCI2C0: I2C 0 interface power/clock enable (bit 7)
//      <o12.8>   PCUART4: UART 4 power/clock enable (bit 8)
//      <o12.9>   PCRTC: RTC and Event Recorder power/clock enable (bit 9)
//      <o12.10>  PCSSP1: SSP 1 interface power/clock enable (bit 10)
//      <o12.11>  PCEMC: External Memory Controller power/clock enable (bit 11)
//      <o12.12>  PCADC: A/D converter power/clock enable (bit 12)
//      <o12.13>  PCCAN1: CAN controller 1 power/clock enable (bit 13)
//      <o12.14>  PCCAN2: CAN controller 2 power/clock enable (bit 14)
//      <o12.15>  PCGPIO: IOCON, GPIO, and GPIO interrupts power/clock enable (bit 15)
//      <o12.17>  PCMCPWM: Motor Control PWM power/clock enable (bit 17)
//      <o12.18>  PCQEI: Quadrature encoder interface power/clock enable (bit 18)
//      <o12.19>  PCI2C1: I2C 1 interface power/clock enable (bit 19)
//      <o12.20>  PCSSP2: SSP 2 interface power/clock enable (bit 20)
//      <o12.21>  PCSSP0: SSP 0 interface power/clock enable (bit 21)
//      <o12.22>  PCTIM2: Timer 2 power/clock enable (bit 22)
//      <o12.23>  PCTIM3: Timer 3 power/clock enable (bit 23)
//      <o12.24>  PCUART2: UART 2 power/clock enable (bit 24)
//      <o12.25>  PCUART3: UART 3 power/clock enable (bit 25)
//      <o12.26>  PCI2C2: I2C 2 interface power/clock enable (bit 26)
//      <o12.27>  PCI2S: I2S interface power/clock enable (bit 27)
//      <o12.28>  PCSDC: SD Card interface power/clock enable (bit 28)
//      <o12.29>  PCGPDMA: GPDMA function power/clock enable (bit 29)
//      <o12.30>  PCENET: Ethernet block power/clock enable (bit 30)
//      <o12.31>  PCUSB: USB interface power/clock enable (bit 31)
//    </h>
//
//    <h>  Clock Output Configuration Register (CLKOUTCFG)
//      <o13.0..3>  CLKOUTSEL: Clock Source for CLKOUT Selection
//            <0=>  CPU clock
//            <1=>  Main Oscillator
//            <2=>  Internal RC Oscillator
//            <3=>  USB clock
//            <4=>  RTC Oscillator
//            <5=>  unused
//            <6=>  Watchdog Oscillator
//
//      <o13.4..7>  CLKOUTDIV: Output Clock Divider
//            <1-16><#-1>
//
//      <o13.8>    CLKOUT_EN: CLKOUT enable
//    </h>
//
//  </e>
*/
#define CLOCK_SETUP           1
#define SCS_Val               0x00000020
#define CLKSRCSEL_Val         0x00000001
#define PLL0_SETUP            1
#define PLL0CFG_Val           0x00000007
#define PLL1_SETUP            1
#define PLL1CFG_Val           0x0000000C
#define CCLKSEL_Val           0x00000101
#define USBCLKSEL_Val         0x00000102
#define EMCCLKSEL_Val         0x00000000
#define PCLKSEL_Val           0x00000002
#define SPIFICLKSEL_Val       0x00000202
#define PCONP_Val             0x042887DE
#define CLKOUTCFG_Val         0x00000100

#define LPC_CPACR             0xE000ED88

#define SCB_MVFR0             0xE000EF40
#define SCB_MVFR0_RESET       0x10110021

#define SCB_MVFR1             0xE000EF44
#define SCB_MVFR1_RESET       0x11000011


/*--------------------- Flash Accelerator Configuration ----------------------
//
//  <e>  Flash Accelerator Configuration register (FLASHCFG - address 0x400F C000)
//    <o1.12..15>  FLASHTIM: Flash Access Time
//          <0=>  1 CPU clock (for CPU clock up to 20 MHz)
//          <1=>  2 CPU clocks (for CPU clock up to 40 MHz)
//          <2=>  3 CPU clocks (for CPU clock up to 60 MHz)
//          <3=>  4 CPU clocks (for CPU clock up to 80 MHz)
//          <4=>  5 CPU clocks (for CPU clock up to 100 MHz)
//          <5=>  6 CPU clocks (for any CPU clock)
//  </e>
*/
#define FLASH_SETUP           1
#define FLASHCFG_Val          0x00005000

/*----------------------------------------------------------------------------
  Check the register settings
 *----------------------------------------------------------------------------*/
#define CHECK_RANGE(val, min, max)                ((val < min) || (val > max))
#define CHECK_RSVD(val, mask)                     (val & mask)

/* Clock Configuration -------------------------------------------------------*/
#if (CHECK_RSVD((SCS_Val),       ~0x0000003F))
   #error "SCS: Invalid values of reserved bits!"
#endif

#if (CHECK_RANGE((CLKSRCSEL_Val), 0, 1))
   #error "CLKSRCSEL: Value out of range!"
#endif

#if (CHECK_RSVD((PLL0CFG_Val),   ~0x0000007F))
   #error "PLL0CFG: Invalid values of reserved bits!"
#endif

#if (CHECK_RSVD((PLL1CFG_Val),   ~0x0000007F))
   #error "PLL1CFG: Invalid values of reserved bits!"
#endif

#if (CHECK_RSVD((CCLKSEL_Val),   ~0x0000011F))
   #error "CCLKSEL: Invalid values of reserved bits!"
#endif

#if (CHECK_RSVD((USBCLKSEL_Val), ~0x0000031F))
   #error "USBCLKSEL: Invalid values of reserved bits!"
#endif

#if (CHECK_RSVD((EMCCLKSEL_Val), ~0x00000001))
   #error "EMCCLKSEL: Invalid values of reserved bits!"
#endif

#if (CHECK_RSVD((PCLKSEL_Val), ~0x0000001F))
   #error "PCLKSEL: Invalid values of reserved bits!"
#endif

#if (CHECK_RSVD((PCONP_Val), ~0xFFFEFFFF))
   #error "PCONP: Invalid values of reserved bits!"
#endif

#if (CHECK_RSVD((CLKOUTCFG_Val), ~0x000001FF))
   #error "CLKOUTCFG: Invalid values of reserved bits!"
#endif

/* Flash Accelerator Configuration -------------------------------------------*/
#if (CHECK_RSVD((FLASHCFG_Val), ~0x0000F000))
   #warning "FLASHCFG: Invalid values of reserved bits!"
#endif


/*----------------------------------------------------------------------------
  DEFINES
 *----------------------------------------------------------------------------*/
/* pll_out_clk = F_cco / (2 × P)
   F_cco = pll_in_clk × M × 2 × P */
#define __M(__PLLCFG_Val)     ((__PLLCFG_Val & 0x1F) + 1)
#define __PLL0_CLK(__F_IN)    (__F_IN * __M(PLL0CFG_Val))
#define __PLL1_CLK            (OSC_CLK * __M(PLL1CFG_Val))
#define __CCLK_DIV            (CCLKSEL_Val & 0x1F)
#define __PCLK_DIV            (PCLKSEL_Val & 0x1F)
#define __ECLK_DIV            ((EMCCLKSEL_Val & 0x01) + 1)
#define __SPIFI_DIV           (SPIFICLKSEL_Val & 0x1F)

/* Determine core clock frequency according to settings */
#if (CLOCK_SETUP)                       /* Clock Setup                        */

  #if ((CLKSRCSEL_Val & 0x01) == 1) && ((SCS_Val & 0x20)== 0)
   #error "Main Oscillator is selected as clock source but is not enabled!"
  #endif

  #if ((CCLKSEL_Val & 0x100) == 0x100) && (PLL0_SETUP == 0)
   #error "Main PLL is selected as clock source but is not enabled!"
  #endif

  #if ((CCLKSEL_Val & 0x100) == 0)      /* cclk = sysclk */
    #if ((CLKSRCSEL_Val & 0x01) == 0)   /* sysclk = irc_clk */
      #define __CORE_CLK (IRC_OSC / __CCLK_DIV)
      #define __PER_CLK  (IRC_OSC/  __PCLK_DIV)
      #define __EMC_CLK  (__CORE_CLK/  __ECLK_DIV)
    #else                               /* sysclk = osc_clk */
      #define __CORE_CLK (OSC_CLK / __CCLK_DIV)
      #define __PER_CLK  (OSC_CLK/  __PCLK_DIV)
      #define __EMC_CLK  (__CORE_CLK/  __ECLK_DIV)
    #endif
  #else                                 /* cclk = pll_clk */
    #if ((CLKSRCSEL_Val & 0x01) == 0)   /* sysclk = irc_clk */
      #define __CORE_CLK (__PLL0_CLK(IRC_OSC) / __CCLK_DIV)
      #define __PER_CLK  (__PLL0_CLK(IRC_OSC) / __PCLK_DIV)
      #define __EMC_CLK  (__CORE_CLK / __ECLK_DIV)
    #else                               /* sysclk = osc_clk */
      #define __CORE_CLK (__PLL0_CLK(OSC_CLK) / __CCLK_DIV)
      #define __PER_CLK  (__PLL0_CLK(OSC_CLK) / __PCLK_DIV)
      #define __EMC_CLK  (__CORE_CLK / __ECLK_DIV)
    #endif
  #endif
  #if ((SPIFICLKSEL_Val & 0x300) == 0)
    #if ((CLKSRCSEL_Val & 0x01) == 0)   /* sysclk = irc_clk */
      #define __SPIFI_CLK (IRC_OSC / __SPIFI_DIV)
    #else                               /* sysclk = osc_clk */
      #define __SPIFI_CLK (OSC_CLK / __SPIFI_DIV)      
    #endif
  #elif ((SPIFICLKSEL_Val & 0x300) == 1)
    #if ((CLKSRCSEL_Val & 0x01) == 0)   /* sysclk = irc_clk */
      #define __CORE_CLK (__PLL0_CLK(IRC_OSC) / __SPIFI_DIV)
    #else                               /* sysclk = osc_clk */
      #define __CORE_CLK (__PLL0_CLK(OSC_CLK) / __SPIFI_DIV)
    #endif
  #else
    #define __SPIFI_CLK (__PLL1_CLK / __SPIFI_DIV)
  #endif
#else
        #define __CORE_CLK  (IRC_OSC)
        #define __PER_CLK   (IRC_OSC)
        #define __EMC_CLK   (__CORE_CLK)
        #define __SPIFI_CLK (0)
#endif

/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock = __CORE_CLK;   /*!< System Clock Frequency (Core Clock)*/
uint32_t PeripheralClock = __PER_CLK;    /*!< Peripheral Clock Frequency (Pclk)  */
uint32_t EMCClock        = __EMC_CLK;    /*!< EMC Clock Frequency                */
uint32_t USBClock        = (48000000UL); /*!< USB Clock Frequency                */
uint32_t SPIFIClock      = __SPIFI_CLK;  /*!< SPIFI Clock Frequecy               */


/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/
void SystemCoreClockUpdate (void)            /* Get Core Clock Frequency      */
{
  /* Determine clock frequency according to clock register values             */
  if ((LPC_SC->CCLKSEL &0x100) == 0) {            /* cclk = sysclk    */
    if ((LPC_SC->CLKSRCSEL & 0x01) == 0) {    /* sysclk = irc_clk */
      SystemCoreClock = __CLK_DIV(IRC_OSC , (LPC_SC->CCLKSEL & 0x1F));
          PeripheralClock = __CLK_DIV(IRC_OSC , (LPC_SC->PCLKSEL & 0x1F));
          EMCClock        = (SystemCoreClock / ((LPC_SC->EMCCLKSEL & 0x01)+1));
    }
    else {                                        /* sysclk = osc_clk */
      if ((LPC_SC->SCS & 0x40) == 0) {
          SystemCoreClock = 0;                      /* this should never happen! */
          PeripheralClock = 0;
          EMCClock        = 0;
      }
      else {
          SystemCoreClock = __CLK_DIV(OSC_CLK , (LPC_SC->CCLKSEL & 0x1F));
          PeripheralClock = __CLK_DIV(OSC_CLK , (LPC_SC->PCLKSEL & 0x1F));
          EMCClock        = (SystemCoreClock / ((LPC_SC->EMCCLKSEL & 0x01)+1));
      }
    }
  }
  else {                                          /* cclk = pll_clk */
    if ((LPC_SC->PLL0STAT & 0x100) == 0) {        /* PLL0 not enabled */
          SystemCoreClock = 0;                      /* this should never happen! */
          PeripheralClock = 0;
          EMCClock       = 0;
    }
    else {
      if ((LPC_SC->CLKSRCSEL & 0x01) == 0) {    /* sysclk = irc_clk */
          uint8_t mul = ((LPC_SC->PLL0STAT & 0x1F) + 1);
          uint8_t cpu_div = (LPC_SC->CCLKSEL & 0x1F);
          uint8_t per_div = (LPC_SC->PCLKSEL & 0x1F);
          uint8_t emc_div = (LPC_SC->EMCCLKSEL & 0x01)+1;
          SystemCoreClock = __CLK_DIV(IRC_OSC * mul , cpu_div);
          PeripheralClock = __CLK_DIV(IRC_OSC * mul , per_div);
          EMCClock        = SystemCoreClock / emc_div;
      }
      else {                                        /* sysclk = osc_clk */
        if ((LPC_SC->SCS & 0x40) == 0) {
          SystemCoreClock = 0;                      /* this should never happen! */
          PeripheralClock = 0;
          EMCClock       = 0;
        }
        else {
          uint8_t mul = ((LPC_SC->PLL0STAT & 0x1F) + 1);
          uint8_t cpu_div = (LPC_SC->CCLKSEL & 0x1F);
          uint8_t per_div = (LPC_SC->PCLKSEL & 0x1F);
      uint8_t emc_div = (LPC_SC->EMCCLKSEL & 0x01)+1;
          SystemCoreClock = __CLK_DIV(OSC_CLK * mul , cpu_div);
          PeripheralClock = __CLK_DIV(OSC_CLK * mul , per_div);
          EMCClock        = SystemCoreClock / emc_div;
        }
      }
    }
  }
  /* ---update USBClock------------------*/
  if(LPC_SC->USBCLKSEL & (0x01<<8))//Use PLL0 as the input to the USB clock divider
  {
    switch (LPC_SC->USBCLKSEL & 0x1F)
    {
    case 0:
      USBClock = 0; //no clock will be provided to the USB subsystem
      break;
    case 2:
    case 4:
    case 6:
            {
                 uint8_t mul = ((LPC_SC->PLL0STAT & 0x1F) + 1);
                 uint8_t usb_div = (LPC_SC->USBCLKSEL & 0x1F);
      if(LPC_SC->CLKSRCSEL & 0x01)  //pll_clk_in = main_osc
        USBClock = OSC_CLK * mul / usb_div;
      else //pll_clk_in = irc_clk
        USBClock = IRC_OSC * mul / usb_div;
            }
            break;
    default:
      USBClock = 0;  /* this should never happen! */
    }
  }
  else if(LPC_SC->USBCLKSEL & (0x02<<8))//usb_input_clk = alt_pll (pll1)
  {
    USBClock = (OSC_CLK * ((LPC_SC->PLL1STAT & 0x1F) + 1));
  }
  else
    USBClock = 0; /* this should never happen! */
    
  /* ---update SPIFIClock------------------*/
  switch((LPC_SC->SPIFICLKSEL & 0x300)>>8)
  {
  case 0: /*spifi_clk = sysclk*/
    if ((LPC_SC->CLKSRCSEL & 0x01) == 0) {    /* sysclk = irc_clk */
      SPIFIClock = __CLK_DIV(IRC_OSC , (LPC_SC->SPIFICLKSEL & 0x1F));
    }
    else {                                        /* sysclk = osc_clk */
      if ((LPC_SC->SCS & 0x40) == 0) {
          SPIFIClock = 0;                         /* */
      }
      else {
          SPIFIClock = __CLK_DIV(OSC_CLK , (LPC_SC->SPIFICLKSEL & 0x1F));
      }
    }
    break;
  case 1: /*spifi_clk = pll0*/
    if ((LPC_SC->PLL0STAT & 0x100) == 0) {        /* PLL0 not enabled */
          SPIFIClock = 0;                    /* this should never happen! */
    }
    else {
      if ((LPC_SC->CLKSRCSEL & 0x01) == 0) {    /* sysclk = irc_clk */
          uint8_t mul = ((LPC_SC->PLL0STAT & 0x1F) + 1);
          uint8_t spifi_div = (LPC_SC->SPIFICLKSEL & 0x1F);
          SPIFIClock = __CLK_DIV(IRC_OSC * mul , spifi_div);
      }
      else {                                        /* sysclk = osc_clk */
        if ((LPC_SC->SCS & 0x40) == 0) {
          SPIFIClock = 0;                           /* */
        }
        else {
          uint8_t mul = ((LPC_SC->PLL0STAT & 0x1F) + 1);
          uint8_t spifi_div = (LPC_SC->SPIFICLKSEL & 0x1F);
          SPIFIClock = __CLK_DIV(OSC_CLK * mul , spifi_div);
        }
      }
    }
    break;
  case 2: /*spifi_clk = pll1*/
    if ((LPC_SC->PLL1STAT & 0x100) == 0) {        /* PLL1 not enabled */
          SPIFIClock = 0;                         /* this should never happen! */
    }
    else {
      if ((LPC_SC->SCS & 0x40) == 0) {
        SPIFIClock = 0;                           /* */
      }
      else {
        uint8_t mul = ((LPC_SC->PLL1STAT & 0x1F) + 1);
        uint8_t spifi_div = (LPC_SC->SPIFICLKSEL & 0x1F);
        SPIFIClock = __CLK_DIV(OSC_CLK * mul , spifi_div);
      }
    }
    break;
  default:
      SPIFIClock = 0; /* this should never happen! */
    break;
  }
}

/**
  * @brief  Sets System clock frequency to the configured defaults.
  * @note   This function should be used only after reset.
  * @param  None
  * @retval None
  */
int SetSysClock(void)
{
    int rc = 0;

#if (CLOCK_SETUP)                       /* Clock Setup                        */
  LPC_SC->SCS       = SCS_Val;
  if (SCS_Val & (1 << 5)) {             /* If Main Oscillator is enabled      */
    while ((LPC_SC->SCS & (1<<6)) == 0);/* Wait for Oscillator to be ready    */
  }

  LPC_SC->CLKSRCSEL = CLKSRCSEL_Val;    /* Select Clock Source for sysclk/PLL0*/

#if (PLL0_SETUP)
  LPC_SC->PLL0CFG   = PLL0CFG_Val;
  LPC_SC->PLL0CON   = 0x01;             /* PLL0 Enable                        */
  LPC_SC->PLL0FEED  = 0xAA;
  LPC_SC->PLL0FEED  = 0x55;
  while (!(LPC_SC->PLL0STAT & (1<<10)));/* Wait for PLOCK0                    */
#endif

#if (PLL1_SETUP)
  LPC_SC->PLL1CFG   = PLL1CFG_Val;
  LPC_SC->PLL1CON   = 0x01;             /* PLL1 Enable                        */
  LPC_SC->PLL1FEED  = 0xAA;
  LPC_SC->PLL1FEED  = 0x55;
  while (!(LPC_SC->PLL1STAT & (1<<10)));/* Wait for PLOCK1                    */
#endif

  LPC_SC->CCLKSEL   = CCLKSEL_Val;      /* Setup Clock Divider                */
  LPC_SC->USBCLKSEL = USBCLKSEL_Val;    /* Setup USB Clock Divider            */
  LPC_SC->EMCCLKSEL = EMCCLKSEL_Val;    /* EMC Clock Selection                */
  LPC_SC->SPIFICLKSEL  = SPIFICLKSEL_Val;  /* SPIFI Clock Selection              */
  LPC_SC->PCLKSEL   = PCLKSEL_Val;      /* Peripheral Clock Selection         */
  LPC_SC->PCONP     = PCONP_Val;        /* Power Control for Peripherals      */
  LPC_SC->CLKOUTCFG = CLKOUTCFG_Val;    /* Clock Output Configuration         */
#endif

  LPC_SC->PBOOST   |= 0x03;      /* Power Boost control        */

#if (FLASH_SETUP == 1)                  /* Flash Accelerator Setup            */
  LPC_SC->FLASHCFG  = FLASHCFG_Val|0x03A;
#endif

    SystemCoreClockUpdate();
    return rc;
}

/**
  * @brief  requests System clock frequency
  *
  * @note   This function should be used only after reset.
  * @param  None
  * @retval None
  */
uint32_t SysCtlClockGet(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}


/**
  * @brief  requests frequency of the given clock
  *
  * @param  idx NUT_HWCLK Index
  * @retval clock or 0 if idx points to an invalid clock
  */
uint32_t Lpc17xx_ClockGet(int idx)
{
    SystemCoreClockUpdate();
    switch(idx) {
        case NUT_HWCLK_CPU:
            return SystemCoreClock;
            break;

        case NUT_HWCLK_PCLK:
            return PeripheralClock;
            break;

        case NUT_HWCLK_EMC:
            return EMCClock;
            break;

        case NUT_HWCLK_USB:
            return USBClock;
            break;

        case NUT_HWCLK_SPIFI:
            return SPIFIClock;
            break;

        default:
            return 0;
            break;
    }
}
