#ifndef _ARCH_CM3_NXP_MACH_LPC_CAN_H_
#define _ARCH_CM3_NXP_MACH_LPC_CAN_H_

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
 * \file arch/cm3/nxp/mach/lpc_can.h
 * \brief LPC CAN definitions
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

/*!
 * \addtogroup xgNutArchArmv7mLpcUart
 */
/*@{*/


/*! \name CAN Register */
/*@{*/
#define CAN_AFMR_OFF        0x00000000
#define CAN_AFMR            (LPC_CANAF_BASE + CAN_AFMR_OFF)
/*@}*/

/*! \name CAN Register */
/*@{*/
#define CAN_SFF_SA_OFF      0x00000004
#define CAN_SFF_SA          (LPC_CANAF_BASE + CAN_SFF_SA_OFF)
/*@}*/

/*! \name CAN Register */
/*@{*/
#define CAN_SFF_GRP_SA_OFF  0x00000008
#define CAN_SFF_GRP_SA      (LPC_CANAF_BASE + CAN_SFF_GRP_SA_OFF)
/*@}*/

/*! \name CAN Register */
/*@{*/
#define CAN_EFF_SA_OFF      0x0000000C
#define CAN_EFF_SA          (LPC_CANAF_BASE + CAN_EFF_SA_OFF)
/*@}*/

/*! \name CAN Register */
/*@{*/
#define CAN_EFF_GRP_SA_OFF  0x00000010
#define CAN_EFF_GRP_SA      (LPC_CANAF_BASE + CAN_EFF_GRP_SA_OFF)
/*@}*/

/*! \name CAN Register */
/*@{*/
#define CAN_ENDOFTABLE_OFF  0x00000014
#define CAN_ENDOFTABLE      (LPC_CANAF_BASE + CAN_ENDOFTABLE_OFF)
/*@}*/

/*! \name CAN Register */
/*@{*/
#define CAN_LUTERRAD_OFF    0x00000018
#define CAN_LUTERRAD        (LPC_CANAF_BASE + CAN_LUTERRAD_OFF)
/*@}*/

/*! \name CAN Register */
/*@{*/
#define CAN_LUTERR_OFF      0x0000001C
#define CAN_LUTERR          (LPC_CANAF_BASE + CAN_LUTERR_OFF)
/*@}*/

/*! \name CAN Register */
/*@{*/
#define CAN_TXSR_OFF        0x00000000
#define CAN_TXSR            (LPC_CANCR_BASE + CAN_TXSR_OFF)
/*@}*/

/*! \name CAN Register */
/*@{*/
#define CAN_RXSR_OFF        0x00000004
#define CAN_RXSR            (LPC_CANCR_BASE + CAN_RXSR_OFF)
/*@}*/

/*! \name CAN Registers */
/*@{*/
#define CAN_MSR_OFF         0x00000008
#define CAN_MSR             (LPC_CANCR_BASE + CAN_MSR_OFF)
/*@}*/

/*! \name CAN Registers */
/*@{*/
#define CAN_MOD_OFF         0x00000000
/*@}*/

/*! \name CAN Registers */
/*@{*/
#define CAN_CMR_OFF         0x00000004
/*@}*/

/*! \name CAN Registers */
/*@{*/
#define CAN_GSR_OFF         0x00000008
/*@}*/

/*! \name CAN Registers */
/*@{*/
#define CAN_ICR_OFF         0x0000000C
/*@}*/

/*! \name CAN Registers */
/*@{*/
#define CAN_IER_OFF         0x00000010
/*@}*/

/*! \name CAN Registers */
/*@{*/
#define CAN_BTR_OFF         0x00000014
/*@}*/

/*! \name CAN Registers */
/*@{*/
#define CAN_EWL_OFF         0x00000018
/*@}*/

/*! \name CAN Registers */
/*@{*/
#define CAN_SR_OFF          0x0000001C
/*@}*/

/*! \name CAN Registers */
/*@{*/
#define CAN_RFS_OFF         0x00000020
/*@}*/

/*! \name CAN Registers */
/*@{*/
#define CAN_RID_OFF         0x00000024
/*@}*/

/*! \name CAN Registers */
/*@{*/
#define CAN_RDA_OFF         0x00000028
/*@}*/

/*! \name CAN Registers */
/*@{*/
#define CAN_RDB_OFF         0x0000002C
/*@}*/

/*! \name CAN Registers */
/*@{*/
#define CAN_TFI_OFF(x)      (x * 16 + 0x00000020)
/*@}*/

/*! \name CAN Registers */
/*@{*/
#define CAN_TID_OFF(x)      (x * 16 + 0x00000024)
/*@}*/

/*! \name CAN Registers */
/*@{*/
#define CAN_TDA_OFF(x)      (x * 16 + 0x00000028)
/*@}*/

/*! \name CAN Registers */
/*@{*/
#define CAN_TDB_OFF(x)      (x * 16 + 0x0000002C)
/*@}*/

/*! \name CAN1 Register Addresses */
/*@{*/
#ifdef LPC_CAN1_BASE
#define CAN1_MOD            (LPC_CAN1_BASE + CAN_MOD_OFF)
#define CAN1_CMR            (LPC_CAN1_BASE + CAN_CMR_OFF)
#define CAN1_GSR            (LPC_CAN1_BASE + CAN_GSR_OFF)
#define CAN1_ICR            (LPC_CAN1_BASE + CAN_ICR_OFF)
#define CAN1_IER            (LPC_CAN1_BASE + CAN_IER_OFF)
#define CAN1_BTR            (LPC_CAN1_BASE + CAN_BTR_OFF)
#define CAN1_EWL            (LPC_CAN1_BASE + CAN_EWL_OFF)
#define CAN1_SR             (LPC_CAN1_BASE + CAN_SR_OFF)
#define CAN1_RFS            (LPC_CAN1_BASE + CAN_RFS_OFF)
#define CAN1_RID            (LPC_CAN1_BASE + CAN_RID_OFF)
#define CAN1_RDA            (LPC_CAN1_BASE + CAN_RDA_OFF)
#define CAN1_RDB            (LPC_CAN1_BASE + CAN_RDB_OFF)
#define CAN1_TFI(x)         (LPC_CAN1_BASE + CAN_TFI_OFF(x))
#define CAN1_TID(x)         (LPC_CAN1_BASE + CAN_TID_OFF(x))
#define CAN1_TDA(x)         (LPC_CAN1_BASE + CAN_TDA_OFF(x))
#define CAN1_TDB(x)         (LPC_CAN1_BASE + CAN_TDB_OFF(x))
#endif
/*@}*/

/*! \name CAN2 Register Addresses */
/*@{*/
#ifdef LPC_CAN2_BASE
#define CAN2_MOD            (LPC_CAN2_BASE + CAN_MOD_OFF)
#define CAN2_CMR            (LPC_CAN2_BASE + CAN_CMR_OFF)
#define CAN2_GSR            (LPC_CAN2_BASE + CAN_GSR_OFF)
#define CAN2_ICR            (LPC_CAN2_BASE + CAN_ICR_OFF)
#define CAN2_IER            (LPC_CAN2_BASE + CAN_IER_OFF)
#define CAN2_BTR            (LPC_CAN2_BASE + CAN_BTR_OFF)
#define CAN2_EWL            (LPC_CAN2_BASE + CAN_EWL_OFF)
#define CAN2_SR             (LPC_CAN2_BASE + CAN_SR_OFF)
#define CAN2_RFS            (LPC_CAN2_BASE + CAN_RFS_OFF)
#define CAN2_RID            (LPC_CAN2_BASE + CAN_RID_OFF)
#define CAN2_RDA            (LPC_CAN2_BASE + CAN_RDA_OFF)
#define CAN2_RDB            (LPC_CAN2_BASE + CAN_RDB_OFF)
#define CAN2_TFI(x)         (LPC_CAN2_BASE + CAN_TFI_OFF(x))
#define CAN2_TID(x)         (LPC_CAN2_BASE + CAN_TID_OFF(x))
#define CAN2_TDA(x)         (LPC_CAN2_BASE + CAN_TDA_OFF(x))
#define CAN2_TDB(x)         (LPC_CAN2_BASE + CAN_TDB_OFF(x))
#endif
/*@}*/


/*@}*/
#endif
