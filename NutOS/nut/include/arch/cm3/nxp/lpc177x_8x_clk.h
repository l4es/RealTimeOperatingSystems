#ifndef _LPC177X_8X_CLK_H_
#define _LPC177X_8X_CLK_H_

/*
 * Copyright (C) 2012 by Ole Reinhardt <ole.reinhardt@embedded-it.de>
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

/*
 * \verbatim
 * $Id: lpc177x_8x_clk.h $
 * \endverbatim
 */

/*----------------------------------------------------------------------------*
  Define clocks
 *----------------------------------------------------------------------------*/

#define XTAL        (12000000UL)   /* Oscillator frequency               */
#define OSC_CLK     (      XTAL)   /* Main oscillator frequency          */
#define RTC_CLK     (   32768UL)   /* RTC oscillator frequency           */
#define IRC_OSC     (12000000UL)   /* Internal RC oscillator frequency   */
#define WDT_OSC     (  500000UL)   /* Internal WDT oscillator frequency  */


/*----------------------------------------------------------------------------*
  Peripheral power control bit positions
 *----------------------------------------------------------------------------*/

#define  CLKPWR_PCONP_PCLCD     0  /* LCD controller  */
#define  CLKPWR_PCONP_PCTIM0    1  /* Timer/Counter 0 */
#define  CLKPWR_PCONP_PCTIM1    2  /* Timer/Counter 1 */
#define  CLKPWR_PCONP_PCUART0   3  /* UART 0          */
#define  CLKPWR_PCONP_PCUART1   4  /* UART 1          */
#define  CLKPWR_PCONP_PCPWM0    5  /* PWM0            */
#define  CLKPWR_PCONP_PCPWM1    6  /* PWM1            */
#define  CLKPWR_PCONP_PCI2C0    7  /* I2C 0           */
#define  CLKPWR_PCONP_PCUART4   8  /* UART 4          */
#define  CLKPWR_PCONP_PCRTC     9  /* RTC             */
#define  CLKPWR_PCONP_PCSSP1    10 /* SSP 1           */
#define  CLKPWR_PCONP_PCEMC     11 /* EMC             */
#define  CLKPWR_PCONP_PCADC     12 /* ADC 0           */
#define  CLKPWR_PCONP_PCCAN1    13 /* CAN 1           */
#define  CLKPWR_PCONP_PCCAN2    14 /* CAN 2           */
#define  CLKPWR_PCONP_PCGPIO    15 /* GPIO            */

#define  CLKPWR_PCONP_PCMCPWM   17 /* Motor PWM       */
#define  CLKPWR_PCONP_PCQEI     18 /* QEI             */
#define  CLKPWR_PCONP_PCI2C1    19 /* I2C 1           */
#define  CLKPWR_PCONP_PCSSP2    20 /* SSP 2           */
#define  CLKPWR_PCONP_PCSSP0    21 /* SSP 0           */
#define  CLKPWR_PCONP_PCTIM2    22 /* Timer 2         */
#define  CLKPWR_PCONP_PCTIM3    23 /* Timer 3         */
#define  CLKPWR_PCONP_PCUART2   24 /* UART 2          */
#define  CLKPWR_PCONP_PCUART3   25 /* UART 3          */
#define  CLKPWR_PCONP_PCI2C2    26 /* I2C 2           */
#define  CLKPWR_PCONP_PCI2S     27 /* I2S             */
#define  CLKPWR_PCONP_PCSDC     28 /* SD Card         */
#define  CLKPWR_PCONP_PCGPDMA   29 /* GP DMA          */
#define  CLKPWR_PCONP_PCENET    30 /* Ethernet        */
#define  CLKPWR_PCONP_PCUSB     31 /* USB             */


/*----------------------------------------------------------------------------*
  Peripheral reset control definitions
 *----------------------------------------------------------------------------*/

#define  CLKPWR_RSTCON0_LCD     0  /* LCD controller  */
#define  CLKPWR_RSTCON0_TIM0    1  /* Timer/Counter 0 */
#define  CLKPWR_RSTCON0_TIM1    2  /* Timer/Counter 1 */
#define  CLKPWR_RSTCON0_UART0   3  /* UART 0          */
#define  CLKPWR_RSTCON0_UART1   4  /* UART 1          */
#define  CLKPWR_RSTCON0_PWM0    5  /* PWM0            */
#define  CLKPWR_RSTCON0_PWM1    6  /* PWM1            */
#define  CLKPWR_RSTCON0_I2C0    7  /* I2C 0           */
#define  CLKPWR_RSTCON0_UART4   8  /* UART 4          */
#define  CLKPWR_RSTCON0_RTC     9  /* RTC             */
#define  CLKPWR_RSTCON0_SSP1    10  /* SSP 1           */
#define  CLKPWR_RSTCON0_EMC     11  /* EMC             */
#define  CLKPWR_RSTCON0_ADC     12  /* ADC 0           */
#define  CLKPWR_RSTCON0_CAN1    13  /* CAN 1           */
#define  CLKPWR_RSTCON0_CAN2    14  /* CAN 2           */
#define  CLKPWR_RSTCON0_GPIO    15  /* GPIO            */

#define  CLKPWR_RSTCON0_MCPWM   17  /* Motor PWM       */
#define  CLKPWR_RSTCON0_QEI     18  /* QEI             */
#define  CLKPWR_RSTCON0_I2C1    19  /* I2C 1           */
#define  CLKPWR_RSTCON0_SSP2    20  /* SSP 2           */
#define  CLKPWR_RSTCON0_SSP0    21  /* SSP 0           */
#define  CLKPWR_RSTCON0_TIM2    22  /* Timer 2         */
#define  CLKPWR_RSTCON0_TIM3    23  /* Timer 3         */
#define  CLKPWR_RSTCON0_UART2   24  /* UART 2          */
#define  CLKPWR_RSTCON0_UART3   25  /* UART 3          */
#define  CLKPWR_RSTCON0_I2C2    26  /* I2C 2           */
#define  CLKPWR_RSTCON0_I2S     27  /* I2S             */
#define  CLKPWR_RSTCON0_SDC     28  /* SD Card         */
#define  CLKPWR_RSTCON0_GPDMA   29  /* GP DMA          */
#define  CLKPWR_RSTCON0_ENET    30  /* Ethernet        */
#define  CLKPWR_RSTCON0_USB     31  /* USB             */

#define  CLKPWR_RSTCON1_IOCON   32  /* IOCON           */
#define  CLKPWR_RSTCON1_DAC     33  /* DAC             */
#define  CLKPWR_RSTCON1_CANACC  34  /* CAN ACC         */


#define SysCtlPeripheralClkEnable(bit)    CM3BBSET(LPC_SC_BASE, LPC_SC_TypeDef, PCONP, (bit))
#define SysCtlPeripheralClkDisable(bit)   CM3BBCLR(LPC_SC_BASE, LPC_SC_TypeDef, PCONP, (bit))
#define SysCtlPeripheralClkGet(bit)       CM3BBGET(LPC_SC_BASE, LPC_SC_TypeDef, PCONP, (bit))

#define SysCtlPeripheralResetEnable(bit)  if ((bit) < 32) \
                                          CM3BBSET(LPC_SC_BASE, LPC_SC_TypeDef, RSTCON0, (bit)); else \
                                          CM3BBSET(LPC_SC_BASE, LPC_SC_TypeDef, RSTCON1, (bit - 32));
#define SysCtlPeripheralResetDisable(bit) if ((bit) < 32) \
                                          CM3BBCLR(LPC_SC_BASE, LPC_SC_TypeDef, RSTCON0, (bit)); else \
                                          CM3BBCLR(LPC_SC_BASE, LPC_SC_TypeDef, RSTCON1, (bit - 32));

extern uint32_t Lpc17xx_ClockGet(int idx);
extern uint32_t SysCtlClockGet(void);
extern int SetSysClock(void);

/* Transform ns into clock cycles (runtime, only 32bit multiplications):
 * Valid input range: Clock < 995 MHz, ns = 0..1000000 (1ms)
 *
 * a) Divide Clock by 16. This gives enough headroom for step b).
 * b) Multiply Clock by 69. This will adjust for decimal/binary divisor.
 *    This computation will overflow for a clock > 995 MHz!
 *    This computation will give results 0,5% larger than the real value.
 * c) Divide Clock by 1048576 (2^20). This will give enough headroom for step d).
 * d) Multiply Clock by ns. This will not overflow for ns = 0..1000000 (1ms).
 * e) Divide Clock by 4096 (2^12). This will give value in clocks.
 * f) Add 1 to account for rounding.
 * (Use runtime computations because frequencies may change in runtime).
 */
#define NS_2_CLKS(clock, ns) (((((((clock) >> 4)*69) >> 20)*(ns))>>12)+1)

/* Delay loop for short busy waits */
__attribute__( ( always_inline ) ) static inline void wait_clocks( unsigned int clocks)
{
   clocks >>= 1;          // 2 clocks per cycle
   while(clocks--) {
      asm("":::"memory"); // hint for gcc: do not remove this loop!
   }
}

#endif /* _LPC177X_8X_CLK_H_ */
