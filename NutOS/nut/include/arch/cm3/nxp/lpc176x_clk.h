#ifndef _LPC17XX_CLK_H_
#define _LPC17XX_CLK_H_

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
 * $Id: $
 * \endverbatim
 */

/*----------------------------------------------------------------------------*
  Define clocks
 *----------------------------------------------------------------------------*/

#define XTAL        (12000000UL)        /* Oscillator frequency               */
#define OSC_CLK     (      XTAL)        /* Main oscillator frequency          */
#define RTC_CLK     (   32000UL)        /* RTC oscillator frequency           */
#define IRC_OSC     ( 4000000UL)        /* Internal RC oscillator frequency   */


/*----------------------------------------------------------------------------*
  Peripheral clock divider bit positions
 *----------------------------------------------------------------------------*/

#define  CLKPWR_PCLKSEL_WDT          0  /* WDT     */
#define  CLKPWR_PCLKSEL_TIMER0       2  /* TIMER 0 */
#define  CLKPWR_PCLKSEL_TIMER1       4  /* TIMER 1 */
#define  CLKPWR_PCLKSEL_UART0        6  /* UART 0  */
#define  CLKPWR_PCLKSEL_UART1        8  /* UART 1  */
#define  CLKPWR_PCLKSEL_PWM1        12  /* PWM 1   */
#define  CLKPWR_PCLKSEL_I2C0        14  /* I2C 0   */
#define  CLKPWR_PCLKSEL_SPI         16  /* SPI     */
#define  CLKPWR_PCLKSEL_SSP1        20  /* SSP 1   */
#define  CLKPWR_PCLKSEL_DAC         22  /* DAC     */
#define  CLKPWR_PCLKSEL_ADC         24  /* ADC     */
#define  CLKPWR_PCLKSEL_CAN1        26  /* CAN 1   */
#define  CLKPWR_PCLKSEL_CAN2        28  /* CAN 2   */
#define  CLKPWR_PCLKSEL_ACF         30  /* ACF     */

#define  CLKPWR_PCLKSEL_QEI         32  /* QEI     */
#define  CLKPWR_PCLKSEL_PCB         36  /* PCB     */
#define  CLKPWR_PCLKSEL_I2C1        38  /* I2C 1   */
#define  CLKPWR_PCLKSEL_SSP0        42  /* SSP 0   */
#define  CLKPWR_PCLKSEL_TIMER2      44  /* TIMER 2 */
#define  CLKPWR_PCLKSEL_TIMER3      46  /* TIMER 3 */
#define  CLKPWR_PCLKSEL_UART2       48  /* UART 2  */
#define  CLKPWR_PCLKSEL_UART3       50  /* UART 3  */
#define  CLKPWR_PCLKSEL_I2C2        52  /* I2C 2   */
#define  CLKPWR_PCLKSEL_I2S         54  /* I2S     */
#define  CLKPWR_PCLKSEL_RIT         58  /* RIT     */
#define  CLKPWR_PCLKSEL_SYSCON      60  /* SYSCON  */
#define  CLKPWR_PCLKSEL_MC          62  /* MC      */

/* Peripheral clock devider selection values
 * Note: When CCLK_DIV_8, Peripheral's clock is selected to
 * PCLK_xyz = CCLK/8 except for CAN1, CAN2, and CAN filtering
 * when 0b11 selects PCLK_xyz = CCLK/6
 */

#define  CLKPWR_PCLKSEL_CCLK_DIV_4  0    /* CCLK / 4 */
#define  CLKPWR_PCLKSEL_CCLK_DIV_1  1    /* CCLK     */
#define  CLKPWR_PCLKSEL_CCLK_DIV_2  2    /* CCLK / 2 */
#define  CLKPWR_PCLKSEL_CCLK_DIV_8  3    /* CCLK / 8 */

/*----------------------------------------------------------------------------*
  Peripheral power control bit positions
 *----------------------------------------------------------------------------*/

#define  CLKPWR_PCONP_PCTIM0         1  /* Timer/Counter 0 */
#define  CLKPWR_PCONP_PCTIM1         2  /* Timer/Counter 1 */
#define  CLKPWR_PCONP_PCUART0        3  /* UART 0          */
#define  CLKPWR_PCONP_PCUART1        4  /* UART 1          */

#define  CLKPWR_PCONP_PCPWM1         6  /* PWM1            */
#define  CLKPWR_PCONP_PCI2C0         7  /* I2C 0           */
#define  CLKPWR_PCONP_PCSPI          8  /* SPI             */
#define  CLKPWR_PCONP_PCRTC          9  /* RTC             */
#define  CLKPWR_PCONP_PCSSP1        10  /* SSP 1           */

#define  CLKPWR_PCONP_PCAD          12  /* ADC 0           */
#define  CLKPWR_PCONP_PCCAN1        13  /* CAN 1           */
#define  CLKPWR_PCONP_PCCAN2        14  /* CAN 2           */
#define  CLKPWR_PCONP_PCGPIO        15  /* GPIO            */
#define  CLKPWR_PCONP_PCRIT         16  /* RIT             */
#define  CLKPWR_PCONP_PCMC          17  /* Motor PWM       */
#define  CLKPWR_PCONP_PCQEI         18  /* QEI             */
#define  CLKPWR_PCONP_PCI2C1        19  /* I2C 1           */

#define  CLKPWR_PCONP_PCSSP0        21  /* SSP 0           */
#define  CLKPWR_PCONP_PCTIM2        22  /* Timer 2         */
#define  CLKPWR_PCONP_PCTIM3        23  /* Timer 3         */
#define  CLKPWR_PCONP_PCUART2       24  /* UART 2          */
#define  CLKPWR_PCONP_PCUART3       25  /* UART 3          */
#define  CLKPWR_PCONP_PCI2C2        26  /* I2C 2           */
#define  CLKPWR_PCONP_PCI2S         27  /* I2S             */

#define  CLKPWR_PCONP_PCGPDMA       29  /* GP DMA          */
#define  CLKPWR_PCONP_PCENET        30  /* Ethernet        */
#define  CLKPWR_PCONP_PCUSB         31  /* USB             */

#define SysCtlPeripheralClkEnable(bit)  CM3BBSET(LPC_SC_BASE, LPC_SC_TypeDef, PCONP, (bit))
#define SysCtlPeripheralClkDisable(bit) CM3BBCLR(LPC_SC_BASE, LPC_SC_TypeDef, PCONP, (bit))
#define SysCtlPeripheralClkGet(bit)     CM3BBGET(LPC_SC_BASE, LPC_SC_TypeDef, PCONP, (bit))

extern uint32_t SysCtlClockGet(void);
extern int SetSysClock(void);


extern uint32_t Lpc17xx_ClockGet(int idx);
extern int Lpc176x_PclkDivGet(int id);
extern void Lpc176x_PclkDivSet(int id, int div) ;

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

#endif /* _LPC17XX_CLK_H_ */
