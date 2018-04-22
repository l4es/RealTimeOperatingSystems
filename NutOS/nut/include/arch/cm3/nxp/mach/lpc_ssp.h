#ifndef _ARCH_CM3_NXP_MACH_LPC_SSP_H_
#define _ARCH_CM3_NXP_MACH_LPC_SSP_H_

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
 * \file arch/cm3/nxp/mach/lpc_ssp.h
 * \brief LPC SSP definitions
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

/*!
 * \addtogroup xgNutArchArmv7mLpcSsp
 */
/*@{*/


/*! \name SSP Control Register 0 */
/*@{*/
#define SSP_CR0_OFF         0x00000000
#define SSP_CR0_DSS_MSK     0x0000000F
#define SSP_CR0_DSS_LSB     0
#define SSP_CR0_DSS(n)      (((n) - 1) << SSP_CR0_DSS_LSB)
#define SSP_CR0_FRF_MSK     0x00000030
#define SSP_CR0_FRF_SPI     0x00000000
#define SSP_CR0_FRF_TI      0x00000010
#define SSP_CR0_FRF_MW      0x00000020
#define SSP_CR0_CPOL        0x00000040
#define SSP_CR0_CPHA        0x00000080
#define SSP_CR0_SPI_MODE0   0
#define SSP_CR0_SPI_MODE1   SSP_CR0_CPHA
#define SSP_CR0_SPI_MODE2   SSP_CR0_CPOL
#define SSP_CR0_SPI_MODE3   (SSP_CR0_CPOL | SSP_CR0_CPHA)
#define SSP_CR0_SCR_MSK     0x0000FF00
#define SSP_CR0_SCR_LSB     8
/*@}*/

/*! \name SSP Control Register 1 */
/*@{*/
#define SSP_CR1_OFF         0x00000004
#define SSP_CR1_LBM         0x00000001
#define SSP_CR1_SSE         0x00000002
#define SSP_CR1_MS          0x00000004
#define SSP_CR1_SOD         0x00000008
/*@}*/

/*! \name SSP Data Register */
/*@{*/
#define SSP_DR_OFF          0x00000008
/*@}*/

/*! \name SSP Status Register */
/*@{*/
#define SSP_SR_OFF          0x0000000C
#define SSP_SR_TFE          0x00000001
#define SSP_SR_TNF          0x00000002
#define SSP_SR_RNE          0x00000004
#define SSP_SR_RFF          0x00000008
#define SSP_SR_BSY          0x00000010
/*@}*/

/*! \name SSP Clock Prescale Register */
/*@{*/
#define SSP_CPSR_OFF        0x00000010
/*@}*/

/*! \name SSP Interrupt Registers */
/*@{*/
#define SSP_IMSC_OFF        0x00000014
#define SSP_RIS_OFF         0x00000018
#define SSP_MIS_OFF         0x0000001C
#define SSP_ICR_OFF         0x00000020
#define SSP_RORI            0x00000001
#define SSP_RTI             0x00000002
#define SSP_RXI             0x00000004
#define SSP_TXI             0x00000008
/*@}*/

/*! \name SSP DMA Control Register */
/*@{*/
#define SSP_DMACR_OFF       0x00000024
#define SSP_RXDMAE          0x00000001
#define SSP_TXDMAE          0x00000002
/*@}*/

/*! \name SSP0 Register Addresses */
/*@{*/
#ifdef LPC_SSP0_BASE
#define SSP0CR0             (LPC_SSP0_BASE + SSP_CR0_OFF)
#define SSP0CR1             (LPC_SSP0_BASE + SSP_CR1_OFF)
#define SSP0DR              (LPC_SSP0_BASE + SSP_DR_OFF)
#define SSP0SR              (LPC_SSP0_BASE + SSP_SR_OFF)
#define SSP0CPSR            (LPC_SSP0_BASE + SSP_CPSR_OFF)
#define SSP0IMSC            (LPC_SSP0_BASE + SSP_IMSC_OFF)
#define SSP0RIS             (LPC_SSP0_BASE + SSP_RIS_OFF)
#define SSP0MIS             (LPC_SSP0_BASE + SSP_MIS_OFF)
#define SSP0ICR             (LPC_SSP0_BASE + SSP_ICR_OFF)
#define SSP0DMACR           (LPC_SSP0_BASE + SSP_DMACR_OFF)
#endif

/*! \name SSP0 Register Addresses */
/*@{*/
#ifdef LPC_SSP1_BASE
#define SSP1CR0             (LPC_SSP1_BASE + SSP_CR0_OFF)
#define SSP1CR1             (LPC_SSP1_BASE + SSP_CR1_OFF)
#define SSP1DR              (LPC_SSP1_BASE + SSP_DR_OFF)
#define SSP1SR              (LPC_SSP1_BASE + SSP_SR_OFF)
#define SSP1CPSR            (LPC_SSP1_BASE + SSP_CPSR_OFF)
#define SSP1IMSC            (LPC_SSP1_BASE + SSP_IMSC_OFF)
#define SSP1RIS             (LPC_SSP1_BASE + SSP_RIS_OFF)
#define SSP1MIS             (LPC_SSP1_BASE + SSP_MIS_OFF)
#define SSP1ICR             (LPC_SSP1_BASE + SSP_ICR_OFF)
#define SSP1DMACR           (LPC_SSP1_BASE + SSP_DMACR_OFF)
#endif

/*@}*/
#endif

