#ifndef _ARCH_ARM_AT91_TC_H_
#define _ARCH_ARM_AT91_TC_H_

/*
 * Copyright (C) 2005-2006 by egnite Software GmbH. All rights reserved.
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
 * \file arch/arm/at91_tc.h
 * \brief AT91 peripherals.
 *
 * \verbatim
 *
 * $Log$
 * Revision 1.1  2006/07/05 07:45:28  haraldkipp
 * Split on-chip interface definitions.
 *
 *
 * \endverbatim
 */

/*!
 * \addtogroup xgNutArchArmAt91Tc
 */
/*@{*/

/*! \name Timer Counter Base Addresses */
/*@{*/
#define TC0_BASE        (TC_BASE + 0x00)        /*!< \brief Channel 0 base address. */
#define TC1_BASE        (TC_BASE + 0x40)        /*!< \brief Channel 1 base address. */
#define TC2_BASE        (TC_BASE + 0x80)        /*!< \brief Channel 2 base address. */
#define TC3_BASE        (TC_BLK1_BASE + 0x00)   /*!< \brief Channel 3 base address. */
#define TC4_BASE        (TC_BLK1_BASE + 0x40)   /*!< \brief Channel 4 base address. */
#define TC5_BASE        (TC_BLK1_BASE + 0x80)   /*!< \brief Channel 5 base address. */
/*@}*/

/*! \name Timer Counter Control Register */
/*@{*/
#define TC_CCR_OFF              0x00            /*!< \brief Control register offset. */

#define TC0_CCR     (TC0_BASE + TC_CCR_OFF)     /*!< \brief Channel 0 control register address. */
#define TC1_CCR     (TC1_BASE + TC_CCR_OFF)     /*!< \brief Channel 1 control register address. */
#define TC2_CCR     (TC2_BASE + TC_CCR_OFF)     /*!< \brief Channel 2 control register address. */
#define TC3_CCR     (TC3_BASE + TC_CCR_OFF)     /*!< \brief Channel 3 control register address. */
#define TC4_CCR     (TC4_BASE + TC_CCR_OFF)     /*!< \brief Channel 4 control register address. */
#define TC5_CCR     (TC5_BASE + TC_CCR_OFF)     /*!< \brief Channel 5 control register address. */

#define TC_CLKEN                0x00000001      /*!< \brief Clock enable command. */
#define TC_CLKDIS               0x00000002      /*!< \brief Clock disable command. */
#define TC_SWTRG                0x00000004      /*!< \brief Software trigger command. */
/*@}*/

/*! \name Timer Counter Channel Mode Register */
/*@{*/
#define TC_CMR_OFF              0x04            /*!< \brief Mode register offset. */

#define TC0_CMR     (TC0_BASE + TC_CMR_OFF)     /*!< \brief Channel 0 mode register address. */
#define TC1_CMR     (TC1_BASE + TC_CMR_OFF)     /*!< \brief Channel 1 mode register address. */
#define TC2_CMR     (TC2_BASE + TC_CMR_OFF)     /*!< \brief Channel 2 mode register address. */
#define TC3_CMR     (TC3_BASE + TC_CMR_OFF)     /*!< \brief Channel 3 mode register address. */
#define TC4_CMR     (TC4_BASE + TC_CMR_OFF)     /*!< \brief Channel 4 mode register address. */
#define TC5_CMR     (TC5_BASE + TC_CMR_OFF)     /*!< \brief Channel 5 mode register address. */

#define TC_CLKS                 0x00000007      /*!< \brief Clock selection mask. */
#define TC_CLKS_TIMER_CLOCK1    0x00000000      /*!< \brief Selects timer clock 1. */
#define TC_CLKS_MCK2    TC_CLKS_TIMER_CLOCK1    /*!< \brief Selects MCK / 2. */
#define TC_CLKS_TIMER_CLOCK2    0x00000001      /*!< \brief Selects timer clock 2. */
#define TC_CLKS_MCK8    TC_CLKS_TIMER_CLOCK2    /*!< \brief Selects MCK / 8. */
#define TC_CLKS_TIMER_CLOCK3    0x00000002      /*!< \brief Selects timer clock 3. */
#define TC_CLKS_MCK32   TC_CLKS_TIMER_CLOCK3    /*!< \brief Selects MCK / 32. */
#define TC_CLKS_TIMER_CLOCK4    0x00000003      /*!< \brief Selects timer clock 4. */
#define TC_CLKS_MCK128  TC_CLKS_TIMER_CLOCK4    /*!< \brief Selects MCK / 128. */
#define TC_CLKS_TIMER_CLOCK5    0x00000004      /*!< \brief Selects timer clock 5. */
#define TC_CLKS_MCK1024 TC_CLKS_TIMER_CLOCK5    /*!< \brief Selects MCK / 1024 on older CPUs. */
#define TC_CLKS_SLCK    TC_CLKS_TIMER_CLOCK5    /*!< \brief Selects slow clock on newer CPUs. */
#define TC_CLKS_XC0             0x00000005      /*!< \brief Selects external clock 0. */
#define TC_CLKS_XC1             0x00000006      /*!< \brief Selects external clock 1. */
#define TC_CLKS_XC2             0x00000007      /*!< \brief Selects external clock 2. */

#define TC_CLKI                 0x00000008      /*!< \brief Increments on falling edge. */

#define TC_BURST                0x00000030      /*!< \brief Burst signal selection mask. */
#define TC_BURST_NONE           0x00000000      /*!< \brief Clock is not gated by an external signal. */
#define TC_BUSRT_XC0            0x00000010      /*!< \brief ANDed with external clock 0. */
#define TC_BURST_XC1            0x00000020      /*!< \brief ANDed with external clock 1. */
#define TC_BURST_XC2            0x00000030      /*!< \brief ANDed with external clock 2. */

#define TC_CPCTRG               0x00004000      /*!< \brief RC Compare Enable Trigger Enable. */

#define TC_WAVE                 0x00008000      /*!< \brief Selects waveform mode. */
#define TC_CAPT                 0x00000000      /*!< \brief Selects capture mode. */
/*@}*/

/*! \name Capture Mode */
/*@{*/
#define TC_LDBSTOP              0x00000040      /*!< \brief Counter clock stopped on RB loading. */
#define TC_LDBDIS               0x00000080      /*!< \brief Counter clock disabled on RB loading. */

#define TC_ETRGEDG              0x00000300      /*!< \brief External trigger edge selection mask. */
#define TC_ETRGEDG_RISING_EDGE  0x00000100      /*!< \brief Trigger on external rising edge. */
#define TC_ETRGEDG_FALLING_EDGE 0x00000200      /*!< \brief Trigger on external falling edge. */
#define TC_ETRGEDG_BOTH_EDGE    0x00000300      /*!< \brief Trigger on both external edges. */

#define TC_ABETRG               0x00000400      /*!< \brief TIOA or TIOB external trigger selection mask. */
#define TC_ABETRG_TIOB          0x00000000      /*!< \brief TIOB used as an external trigger. */
#define TC_ABETRG_TIOA          0x00000400      /*!< \brief TIOA used as an external trigger. */

#define TC_LDRA                 0x00030000      /*!< \brief RA loading selection mask. */
#define TC_LDRA_RISING_EDGE     0x00010000      /*!< \brief Load RA on rising edge of TIOA. */
#define TC_LDRA_FALLING_EDGE    0x00020000      /*!< \brief Load RA on falling edge of TIOA. */
#define TC_LDRA_BOTH_EDGE       0x00030000      /*!< \brief Load RA on any edge of TIOA. */

#define TC_LDRB                 0x000C0000      /*!< \brief RB loading selection mask. */
#define TC_LDRB_RISING_EDGE     0x00040000      /*!< \brief Load RB on rising edge of TIOA. */
#define TC_LDRB_FALLING_EDGE    0x00080000      /*!< \brief Load RB on falling edge of TIOA. */
#define TC_LDRB_BOTH_EDGE       0x000C0000      /*!< \brief Load RB on any edge of TIOA. */

/*@}*/

/*! \name Waveform Mode */
/*@{*/
#define TC_CPCSTOP              0x00000040      /*!< \brief Counter clock stopped on RC compare. */
#define TC_CPCDIS               0x00000080      /*!< \brief Counter clock disabled on RC compare. */

#define TC_EEVTEDG              0x00000300      /*!< \brief External event edge selection mask. */
#define TC_EEVTEDG_RISING_EDGE  0x00000100      /*!< \brief External event on rising edge.. */
#define TC_EEVTEDG_FALLING_EDGE 0x00000200      /*!< \brief External event on falling edge.. */
#define TC_EEVTEDG_BOTH_EDGE    0x00000300      /*!< \brief External event on any edge.. */

#define TC_EEVT                 0x00000C00      /*!< \brief External event selection mask. */
#define TC_EEVT_TIOB            0x00000000      /*!< \brief TIOB selected as external event. */
#define TC_EEVT_XC0             0x00000400      /*!< \brief XC0 selected as external event. */
#define TC_EEVT_XC1             0x00000800      /*!< \brief XC1 selected as external event. */
#define TC_EEVT_XC2             0x00000C00      /*!< \brief XC2 selected as external event. */

#define TC_ENETRG               0x00001000      /*!< \brief External event trigger enable. */

#define TC_WAVSEL_UPDOWN        0x00002000      /*!< \brief Count up and down. */
#define TC_WAVSEL_RCTRIG        0x00004000      /*!< \brief Automatic trigger on RC compare. */

#define TC_ACPA                 0x00030000      /*!< \brief Masks RA compare effect on TIOA. */
#define TC_ACPA_SET_OUTPUT      0x00010000      /*!< \brief RA compare sets TIOA. */
#define TC_ACPA_CLEAR_OUTPUT    0x00020000      /*!< \brief RA compare clears TIOA. */
#define TC_ACPA_TOGGLE_OUTPUT   0x00030000      /*!< \brief RA compare toggles TIOA. */

#define TC_ACPC                 0x000C0000      /*!< \brief Masks RC compare effect on TIOA. */
#define TC_ACPC_SET_OUTPUT      0x00040000      /*!< \brief RC compare sets TIOA. */
#define TC_ACPC_CLEAR_OUTPUT    0x00080000      /*!< \brief RC compare clears TIOA. */
#define TC_ACPC_TOGGLE_OUTPUT   0x000C0000      /*!< \brief RC compare toggles TIOA. */

#define TC_AEEVT                0x00300000      /*!< \brief Masks external event effect on TIOA. */
#define TC_AEEVT_SET_OUTPUT     0x00100000      /*!< \brief External event sets TIOA. */
#define TC_AEEVT_CLEAR_OUTPUT   0x00200000      /*!< \brief External event clears TIOA. */
#define TC_AEEVT_TOGGLE_OUTPUT  0x00300000      /*!< \brief External event toggles TIOA. */

#define TC_ASWTRG               0x00C00000      /*!< \brief Masks software trigger effect on TIOA. */
#define TC_ASWTRG_SET_OUTPUT    0x00400000      /*!< \brief Software trigger sets TIOA. */
#define TC_ASWTRG_CLEAR_OUTPUT  0x00800000      /*!< \brief Software trigger clears TIOA. */
#define TC_ASWTRG_TOGGLE_OUTPUT 0x00C00000      /*!< \brief Software trigger toggles TIOA. */

#define TC_BCPB                 0x03000000      /*!< \brief Masks RB compare effect on TIOB. */
#define TC_BCPB_SET_OUTPUT      0x01000000      /*!< \brief RB compare sets TIOB. */
#define TC_BCPB_CLEAR_OUTPUT    0x02000000      /*!< \brief RB compare clears TIOB. */
#define TC_BCPB_TOGGLE_OUTPUT   0x03000000      /*!< \brief RB compare toggles TIOB. */

#define TC_BCPC                 0x0C000000      /*!< \brief Masks RC compare effect on TIOB. */
#define TC_BCPC_SET_OUTPUT      0x04000000      /*!< \brief RC compare sets TIOB. */
#define TC_BCPC_CLEAR_OUTPUT    0x08000000      /*!< \brief RC compare clears TIOB. */
#define TC_BCPC_TOGGLE_OUTPUT   0x0C000000      /*!< \brief RC compare toggles TIOB. */

#define TC_BEEVT                0x30000000      /*!< \brief Masks external event effect on TIOB. */
#define TC_BEEVT_SET_OUTPUT     0x10000000      /*!< \brief External event sets TIOB. */
#define TC_BEEVT_CLEAR_OUTPUT   0x20000000      /*!< \brief External event clears TIOB. */
#define TC_BEEVT_TOGGLE_OUTPUT  0x30000000      /*!< \brief External event toggles TIOB. */

#define TC_BSWTRG               0xC0000000      /*!< \brief Masks software trigger effect on TIOB. */
#define TC_BSWTRG_SET_OUTPUT    0x40000000      /*!< \brief Software trigger sets TIOB. */
#define TC_BSWTRG_CLEAR_OUTPUT  0x80000000      /*!< \brief Software trigger clears TIOB. */
#define TC_BSWTRG_TOGGLE_OUTPUT 0xC0000000      /*!< \brief Software trigger toggles TIOB. */
/*@}*/

/*! \name Counter Value Register */
/*@{*/
#define TC_CV_OFF               0x10            /*!< \brief Counter value register offset. */

#define TC0_CV      (TC0_BASE + TC_CV_OFF)      /*!< \brief Counter 0 value. */
#define TC1_CV      (TC1_BASE + TC_CV_OFF)      /*!< \brief Counter 1 value. */
#define TC2_CV      (TC2_BASE + TC_CV_OFF)      /*!< \brief Counter 2 value. */
#define TC3_CV      (TC3_BASE + TC_CV_OFF)      /*!< \brief Counter 3 value. */
#define TC4_CV      (TC4_BASE + TC_CV_OFF)      /*!< \brief Counter 4 value. */
#define TC5_CV      (TC5_BASE + TC_CV_OFF)      /*!< \brief Counter 5 value. */
/*@}*/

/*! \name Timer Counter Register A */
/*@{*/
#define TC_RA_OFF               0x14            /*!< \brief Register A offset. */

#define TC0_RA      (TC0_BASE + TC_RA_OFF)      /*!< \brief Channel 0 register A. */
#define TC1_RA      (TC1_BASE + TC_RA_OFF)      /*!< \brief Channel 1 register A. */
#define TC2_RA      (TC2_BASE + TC_RA_OFF)      /*!< \brief Channel 2 register A. */
#define TC3_RA      (TC3_BASE + TC_RA_OFF)      /*!< \brief Channel 3 register A. */
#define TC4_RA      (TC4_BASE + TC_RA_OFF)      /*!< \brief Channel 4 register A. */
#define TC5_RA      (TC5_BASE + TC_RA_OFF)      /*!< \brief Channel 5 register A. */
/*@}*/

/*! \name Timer Counter Register B */
/*@{*/
#define TC_RB_OFF               0x18            /*!< \brief Register B offset. */

#define TC0_RB      (TC0_BASE + TC_RB_OFF)      /*!< \brief Channel 0 register B. */
#define TC1_RB      (TC1_BASE + TC_RB_OFF)      /*!< \brief Channel 1 register B. */
#define TC2_RB      (TC2_BASE + TC_RB_OFF)      /*!< \brief Channel 2 register B. */
#define TC3_RB      (TC3_BASE + TC_RB_OFF)      /*!< \brief Channel 3 register B. */
#define TC4_RB      (TC4_BASE + TC_RB_OFF)      /*!< \brief Channel 4 register B. */
#define TC5_RB      (TC5_BASE + TC_RB_OFF)      /*!< \brief Channel 5 register B. */
/*@}*/

/*! \name Timer Counter Register C */
/*@{*/
#define TC_RC_OFF               0x1C            /*!< \brief Register C offset. */

#define TC0_RC      (TC0_BASE + TC_RC_OFF)      /*!< \brief Channel 0 register C. */
#define TC1_RC      (TC1_BASE + TC_RC_OFF)      /*!< \brief Channel 1 register C. */
#define TC2_RC      (TC2_BASE + TC_RC_OFF)      /*!< \brief Channel 2 register C. */
#define TC3_RC      (TC3_BASE + TC_RC_OFF)      /*!< \brief Channel 3 register C. */
#define TC4_RC      (TC4_BASE + TC_RC_OFF)      /*!< \brief Channel 4 register C. */
#define TC5_RC      (TC5_BASE + TC_RC_OFF)      /*!< \brief Channel 5 register C. */
/*@}*/


/*! \name Timer Counter Status and Interrupt Registers */
/*@{*/
#define TC_SR_OFF               0x20            /*!< \brief Status register offset. */

#define TC0_SR      (TC0_BASE + TC_SR_OFF)      /*!< \brief Channel 0 status register address. */
#define TC1_SR      (TC1_BASE + TC_SR_OFF)      /*!< \brief Channel 1 status register address. */
#define TC2_SR      (TC2_BASE + TC_SR_OFF)      /*!< \brief Channel 2 status register address. */
#define TC3_SR      (TC3_BASE + TC_SR_OFF)      /*!< \brief Channel 3 status register address. */
#define TC4_SR      (TC4_BASE + TC_SR_OFF)      /*!< \brief Channel 4 status register address. */
#define TC5_SR      (TC5_BASE + TC_SR_OFF)      /*!< \brief Channel 5 status register address. */

#define TC_IER_OFF               0x24           /*!< \brief Interrupt enable register offset. */

#define TC0_IER     (TC0_BASE + TC_IER_OFF)     /*!< \brief Channel 0 interrupt enable register address. */
#define TC1_IER     (TC1_BASE + TC_IER_OFF)     /*!< \brief Channel 1 interrupt enable register address. */
#define TC2_IER     (TC2_BASE + TC_IER_OFF)     /*!< \brief Channel 2 interrupt enable register address. */
#define TC3_IER     (TC3_BASE + TC_IER_OFF)     /*!< \brief Channel 3 interrupt enable register address. */
#define TC4_IER     (TC4_BASE + TC_IER_OFF)     /*!< \brief Channel 4 interrupt enable register address. */
#define TC5_IER     (TC5_BASE + TC_IER_OFF)     /*!< \brief Channel 5 interrupt enable register address. */

#define TC_IDR_OFF               0x28           /*!< \brief Interrupt disable register offset. */

#define TC0_IDR     (TC0_BASE + TC_IDR_OFF)     /*!< \brief Channel 0 interrupt disable register address. */
#define TC1_IDR     (TC1_BASE + TC_IDR_OFF)     /*!< \brief Channel 1 interrupt disable register address. */
#define TC2_IDR     (TC2_BASE + TC_IDR_OFF)     /*!< \brief Channel 2 interrupt disable register address. */
#define TC3_IDR     (TC3_BASE + TC_IDR_OFF)     /*!< \brief Channel 3 interrupt disable register address. */
#define TC4_IDR     (TC4_BASE + TC_IDR_OFF)     /*!< \brief Channel 4 interrupt disable register address. */
#define TC5_IDR     (TC5_BASE + TC_IDR_OFF)     /*!< \brief Channel 5 interrupt disable register address. */

#define TC_IMR_OFF               0x2C           /*!< \brief Interrupt mask register offset. */

#define TC0_IMR     (TC0_BASE + TC_IMR_OFF)     /*!< \brief Channel 0 interrupt mask register address. */
#define TC1_IMR     (TC1_BASE + TC_IMR_OFF)     /*!< \brief Channel 1 interrupt mask register address. */
#define TC2_IMR     (TC2_BASE + TC_IMR_OFF)     /*!< \brief Channel 2 interrupt mask register address. */
#define TC3_IMR     (TC3_BASE + TC_IMR_OFF)     /*!< \brief Channel 3 interrupt mask register address. */
#define TC4_IMR     (TC4_BASE + TC_IMR_OFF)     /*!< \brief Channel 4 interrupt mask register address. */
#define TC5_IMR     (TC5_BASE + TC_IMR_OFF)     /*!< \brief Channel 5 interrupt mask register address. */

#define TC_COVFS                0x00000001      /*!< \brief Counter overflow flag. */
#define TC_LOVRS                0x00000002      /*!< \brief Load overrun flag. */
#define TC_CPAS                 0x00000004      /*!< \brief RA compare flag. */
#define TC_CPBS                 0x00000008      /*!< \brief RB compare flag. */
#define TC_CPCS                 0x00000010      /*!< \brief RC compare flag. */
#define TC_LDRAS                0x00000020      /*!< \brief RA loading flag. */
#define TC_LDRBS                0x00000040      /*!< \brief RB loading flag. */
#define TC_ETRGS                0x00000080      /*!< \brief External trigger flag. */
#define TC_CLKSTA               0x00010000      /*!< \brief Clock enable flag. */
#define TC_MTIOA                0x00020000      /*!< \brief TIOA flag. */
#define TC_MTIOB                0x00040000      /*!< \brief TIOB flag. */
/*@}*/

/*! \name Timer Counter Block Control Register */
/*@{*/
#define TC_BCR_OFF              0xC0            /*!< \brief Block control register offset. */

#define TC_BKL0_CR  (TC_BASE + TC_BCR_OFF)      /*!< \brief Block 0 control register address. */
#define TC_BCR                  TC_BKL0_CR      /*!< \brief Block control register address. */
#define TC_BKL1_CR  (TC_BLK1_BASE + TC_BCR_OFF) /*!< \brief Block 1 control register address. */

#define TC_SYNC                 0x00000001      /*!< \brief Synchronisation trigger */
/*@}*/

/*! \name Timer Counter Block Mode Register */
/*@{*/
#define TC_BMR_OFF              0xC4            /*!< \brief Block mode register offset. */

#define TC_BLK0_MR  (TC_BASE + TC_BMR_OFF)      /*!< \brief Block 0 mode register address. */
#define TC_BMR                  TC_BKL0_MR      /*!< \brief Block control register address. */
#define TC_BLK1_MR  (TC_BLK1_BASE + TC_BMR_OFF) /*!< \brief Block 1 mode register address. */

#define TC_TC0XC0S              0x00000003      /*!< \brief External clock signal 0 selection mask. */
#define TC_TCLK0XC0             0x00000000      /*!< \brief Selects TCLK0. */
#define TC_NONEXC0              0x00000001      /*!< \brief None selected. */
#define TC_TIOA1XC0             0x00000002      /*!< \brief Selects TIOA1. */
#define TC_TIOA2XC0             0x00000003      /*!< \brief Selects TIOA2. */

#define TC_TC1XC1S              0x0000000C      /*!< \brief External clock signal 1 selection mask. */
#define TC_TCLK1XC1             0x00000000      /*!< \brief Selects TCLK1. */
#define TC_NONEXC1              0x00000004      /*!< \brief None selected. */
#define TC_TIOA0XC1             0x00000008      /*!< \brief Selects TIOA0. */
#define TC_TIOA2XC1             0x0000000C      /*!< \brief Selects TIOA2. */

#define TC_TC2XC2S              0x00000030      /*!< \brief External clock signal 2 selection mask. */
#define TC_TCLK2XC2             0x00000000      /*!< \brief Selects TCLK2. */
#define TC_NONEXC2              0x00000010      /*!< \brief None selected. */
#define TC_TIOA0XC2             0x00000020      /*!< \brief Selects TIOA0. */
#define TC_TIOA1XC2             0x00000030      /*!< \brief Selects TIOA1. */
/*@}*/

/*@} xgNutArchArmAt91Tc */

#endif                          /* _ARCH_ARM_AT91_TC_H_ */
