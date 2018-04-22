#ifndef _ARCH_CM3_NXP_MACH_LPC__EMAC_H_
#define _ARCH_CM3_NXP_MACH_LPC__EMAC_H_

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
 * \file arch/cm3/nxp/mach/lpc_emac.h
 * \brief LPC Ethernet MAC definitions.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <stdint.h>

/*!
 * \addtogroup xgNutArchArmLpcEmac
 */
/*@{*/


/*! \name MAC Configuration Register 1 */
/*@{*/
#define EMAC_MAC1_OFF       0x000
#define EMAC_MAC1           (LPC_EMAC_BASE + EMAC_MAC1_OFF)

#define EMAC_MAC1_REC_EN_LSB        0
#define EMAC_MAC1_PASS_ALL_LSB      1
#define EMAC_MAC1_RX_FLOWC_LSB      2
#define EMAC_MAC1_TX_FLOWC_LSB      3
#define EMAC_MAC1_LOOPB_LSB         4
#define EMAC_MAC1_RES_TX_LSB        8
#define EMAC_MAC1_RES_MCS_TX_LSB    9
#define EMAC_MAC1_RES_RX_LSB        10
#define EMAC_MAC1_RES_MCS_RX_LSB    11
#define EMAC_MAC1_SIM_RES_LSB       14
#define EMAC_MAC1_SOFT_RES_LSB      15

#define EMAC_MAC1_REC_EN        (1 << EMAC_MAC1_REC_EN_LSB)
#define EMAC_MAC1_PASS_ALL      (1 << EMAC_MAC1_PASS_ALL_LSB)
#define EMAC_MAC1_RX_FLOWC      (1 << EMAC_MAC1_RX_FLOWC_LSB)
#define EMAC_MAC1_TX_FLOWC      (1 << EMAC_MAC1_TX_FLOWC_LSB)
#define EMAC_MAC1_LOOPB         (1 << EMAC_MAC1_LOOPB_LSB)
#define EMAC_MAC1_RES_TX        (1 << EMAC_MAC1_RES_TX_LSB)
#define EMAC_MAC1_RES_MCS_TX    (1 << EMAC_MAC1_RES_MCS_TX_LSB)
#define EMAC_MAC1_RES_RX        (1 << EMAC_MAC1_RES_RX_LSB)
#define EMAC_MAC1_RES_MCS_RX    (1 << EMAC_MAC1_RES_MCS_RX_LSB)
#define EMAC_MAC1_SIM_RES       (1 << EMAC_MAC1_SIM_RES_LSB)
#define EMAC_MAC1_SOFT_RES      (1 << EMAC_MAC1_SOFT_RES_LSB)
/*@}*/

/*! \name MAC Configuration Register 2 */
/*@{*/
#define EMAC_MAC2_OFF       0x004
#define EMAC_MAC2           (LPC_EMAC_BASE + EMAC_MAC2_OFF)

#define EMAC_MAC2_FULL_DUP_LSB      0
#define EMAC_MAC2_FRM_LEN_CHK_LSB   1
#define EMAC_MAC2_HUGE_FRM_EN_LSB   2
#define EMAC_MAC2_DLY_CRC_LSB       3
#define EMAC_MAC2_CRC_EN_LSB        4
#define EMAC_MAC2_PAD_EN_LSB        5
#define EMAC_MAC2_VLAN_PAD_EN_LSB   6
#define EMAC_MAC2_ADET_PAD_EN_LSB   7
#define EMAC_MAC2_PPREAM_ENF_LSB    8
#define EMAC_MAC2_LPREAM_ENF_LSB    9
#define EMAC_MAC2_NO_BACKOFF_LSB    12
#define EMAC_MAC2_BACK_PRESSURE_LSB 13
#define EMAC_MAC2_EXCESS_DEF_LSB    14

#define EMAC_MAC2_FULL_DUP      (1 << EMAC_MAC2_FULL_DUP_LSB)
#define EMAC_MAC2_FRM_LEN_CHK   (1 << EMAC_MAC2_FRM_LEN_CHK_LSB)
#define EMAC_MAC2_HUGE_FRM_EN   (1 << EMAC_MAC2_HUGE_FRM_EN_LSB)
#define EMAC_MAC2_DLY_CRC       (1 << EMAC_MAC2_DLY_CRC_LSB)
#define EMAC_MAC2_CRC_EN        (1 << EMAC_MAC2_CRC_EN_LSB)
#define EMAC_MAC2_PAD_EN        (1 << EMAC_MAC2_PAD_EN_LSB)
#define EMAC_MAC2_VLAN_PAD_EN   (1 << EMAC_MAC2_VLAN_PAD_EN_LSB)
#define EMAC_MAC2_ADET_PAD_EN   (1 << EMAC_MAC2_ADET_PAD_EN_LSB)
#define EMAC_MAC2_PPREAM_ENF    (1 << EMAC_MAC2_PPREAM_ENF_LSB)
#define EMAC_MAC2_LPREAM_ENF    (1 << EMAC_MAC2_LPREAM_ENF_LSB)
#define EMAC_MAC2_NO_BACKOFF    (1 << EMAC_MAC2_NO_BACKOFF_LSB)
#define EMAC_MAC2_BACK_PRESSURE (1 << EMAC_MAC2_BACK_PRESSURE_LSB)
#define EMAC_MAC2_EXCESS_DEF    (1 << EMAC_MAC2_EXCESS_DEF_LSB)
/*@}*/

/*! \name Back-to-Back Inter-Packet-Gap Register */
/*@{*/
#define EMAC_IPGT_OFF       0x008
#define EMAC_IPGT           (LPC_EMAC_BASE + EMAC_IPGT_OFF)

#define EMAC_IPGT_BB_LSB        0
#define EMAC_IPGT_BB_MSB        6
/*@}*/

/*! \name Non Back-to-Back Inter-Packet-Gap Register */
/*@{*/
#define EMAC_IPGR_OFF       0x00C
#define EMAC_IPGR           (LPC_EMAC_BASE + EMAC_IPGR_OFF)

#define EMAC_IPGR_NBB2_LSB      0
#define EMAC_IPGR_NBB2_MSB      6
#define EMAC_IPGR_NBB1_LSB      8
#define EMAC_IPGR_NBB1_MSB      14
/*@}*/

/*! \name Collision Window / Retry Register */
/*@{*/
#define EMAC_CLRT_OFF       0x010
#define EMAC_CLRT           (LPC_EMAC_BASE + EMAC_CLRT_OFF)

#define EMAC_CLRT_RMAX_LSB      0
#define EMAC_CLRT_RMAX_MSB      3
#define EMAC_CLRT_COLLWIN_LSB   8
#define EMAC_CLRT_COLLWIN_MSB   13
/*@}*/

/*! \name Maximum Frame Register */
/*@{*/
#define EMAC_MAXF_OFF       0x014
#define EMAC_MAXF           (LPC_EMAC_BASE + EMAC_MAXF_OFF)
/*@}*/

/*! \name PHY Support Register */
/*@{*/
#define EMAC_SUPP_OFF       0x018
#define EMAC_SUPP           (LPC_EMAC_BASE + EMAC_SUPP_OFF)

#define EMAC_SUPP_SPEED_LSB     8
#define EMAC_SUPP_RES_RMII_LSB  11

#define EMAC_SUPP_SPEED     (1 << EMAC_SUPP_SPEED_LSB)
#define EMAC_SUPP_RES_RMII  (1 << EMAC_SUPP_RES_RMII_LSB)
/*@}*/

/*! \name Test Register */
/*@{*/
#define EMAC_TEST_OFF       0x01C
#define EMAC_TEST           (LPC_EMAC_BASE + EMAC_TEST_OFF)

#define EMAC_TEST_SHCUT_PQUANTA_LSB 0
#define EMAC_TEST_TST_PAUSE_LSB     1
#define EMAC_TEST_TST_BACKP_LSB     2

#define EMAC_TEST_SHCUT_PQUANTA (1 << EMAC_TEST_SHCUT_PQUANTA_LSB)
#define EMAC_TEST_TST_PAUSE     (1 << EMAC_TEST_TST_PAUSE_LSB)
#define EMAC_TEST_TST_BACKP     (1 << EMAC_TEST_TST_BACKP_LSB)
/*@}*/

/*! \name MII Management Configuration Register */
/*@{*/
#define EMAC_MCFG_OFF       0x020
#define EMAC_MCFG           (LPC_EMAC_BASE + EMAC_MCFG_OFF)

#define EMAC_MCFG_SCAN_INC_LSB      0
#define EMAC_MCFG_SUPP_PREAM_LSB    1
#define EMAC_MCFG_CLK_SEL_LSB       2
#define EMAC_MCFG_CLK_SEL_MSB       5
#define EMAC_MCFG_RES_MII_LSB       15

#define EMAC_MCFG_SCAN_INC      (1 << EMAC_MCFG_SCAN_INC_LSB)
#define EMAC_MCFG_SUPP_PREAM    (1 << EMAC_MCFG_SUPP_PREAM_LSB)
#define EMAC_MCFG_RES_MII       (1 << EMAC_MCFG_RES_MII_LSB)
/*@}*/

/*! \name MII Management Command Register */
/*@{*/
#define EMAC_MCMD_OFF       0x024
#define EMAC_MCMD           (LPC_EMAC_BASE + EMAC_MCMD_OFF)

#define EMAC_MCMD_READ_LSB  0
#define EMAC_MCMD_SCAN_LSB  1

#define EMAC_MCMD_READ      (1 << EMAC_MCMD_READ_LSB)
#define EMAC_MCMD_SCAN      (1 << EMAC_MCMD_SCAN_LSB)
/*@}*/

/*! \name MII Management Address Register */
/*@{*/
#define EMAC_MADR_OFF       0x028
#define EMAC_MADR           (LPC_EMAC_BASE + EMAC_MADR_OFF)

#define EMAC_MADR_REG_ADR_LSB   0
#define EMAC_MADR_REG_ADR_MSB   4
#define EMAC_MADR_PHY_ADR_LSB   8
#define EMAC_MADR_PHY_ADR_MSB   12
/*@}*/

/*! \name MII Management Write Data Register */
/*@{*/
#define EMAC_MWTD_OFF       0x02C
#define EMAC_MWTD           (LPC_EMAC_BASE + EMAC_MWTD_OFF)
/*@}*/

/*! \name MII Management Read Data Register */
/*@{*/
#define EMAC_MRDD_OFF       0x030
#define EMAC_MRDD           (LPC_EMAC_BASE + EMAC_MRDD_OFF)
/*@}*/

/*! \name MII Management Indicators Register */
/*@{*/
#define EMAC_MIND_OFF       0x034
#define EMAC_MIND           (LPC_EMAC_BASE + EMAC_MIND_OFF)

#define EMAC_MIND_BUSY_LSB          0
#define EMAC_MIND_SCAN_LSB          1
#define EMAC_MIND_NOT_VAL_LSB       2
#define EMAC_MIND_MII_LINK_FAIL_LSB 3

#define EMAC_MIND_BUSY          (1 << EMAC_MIND_BUSY_LSB)
#define EMAC_MIND_SCAN          (1 << EMAC_MIND_SCAN_LSB)
#define EMAC_MIND_NOT_VAL       (1 << EMAC_MIND_NOT_VAL_LSB)
#define EMAC_MIND_MII_LINK_FAIL (1 << EMAC_MIND_MII_LINK_FAIL_LSB)
/*@}*/

/*! \name Station Address Registers */
/*@{*/
#define EMAC_SA0_OFF        0x040
#define EMAC_SA0            (LPC_EMAC_BASE + EMAC_SA0_OFF)
#define EMAC_SA1_OFF        0x044
#define EMAC_SA1            (LPC_EMAC_BASE + EMAC_SA1_OFF)
#define EMAC_SA2_OFF        0x048
#define EMAC_SA2            (LPC_EMAC_BASE + EMAC_SA2_OFF)

#define EMAC_SA_HI_LSB  0
#define EMAC_SA_HI_MSB  7
#define EMAC_SA_LO_LSB  8
#define EMAC_SA_LO_MSB  15
/*@}*/

/*! \name Command Register */
/*@{*/
#define EMAC_CR_OFF         0x100
#define EMAC_CR             (LPC_EMAC_BASE + EMAC_CR_OFF)

#define EMAC_CR_RX_EN_LSB           0
#define EMAC_CR_TX_EN_LSB           1
#define EMAC_CR_REG_RES_LSB         3
#define EMAC_CR_TX_RES_LSB          4
#define EMAC_CR_RX_RES_LSB          5
#define EMAC_CR_PASS_RUNT_FRM_LSB   6
#define EMAC_CR_PASS_RX_FILT_LSB    7
#define EMAC_CR_TX_FLOW_CTRL_LSB    8
#define EMAC_CR_RMII_LSB            9
#define EMAC_CR_FULL_DUP_LSB        10

#define EMAC_CR_RX_EN           (1 << EMAC_CR_RX_EN_LSB)
#define EMAC_CR_TX_EN           (1 << EMAC_CR_TX_EN_LSB)
#define EMAC_CR_REG_RES         (1 << EMAC_CR_REG_RES_LSB)
#define EMAC_CR_TX_RES          (1 << EMAC_CR_TX_RES_LSB)
#define EMAC_CR_RX_RES          (1 << EMAC_CR_RX_RES_LSB)
#define EMAC_CR_PASS_RUNT_FRM   (1 << EMAC_CR_PASS_RUNT_FRM_LSB)
#define EMAC_CR_PASS_RX_FILT    (1 << EMAC_CR_PASS_RX_FILT_LSB)
#define EMAC_CR_TX_FLOW_CTRL    (1 << EMAC_CR_TX_FLOW_CTRL_LSB)
#define EMAC_CR_RMII            (1 << EMAC_CR_RMII_LSB)
#define EMAC_CR_FULL_DUP        (1 << EMAC_CR_FULL_DUP_LSB)
/*@}*/

/*! \name Status Register */
/*@{*/
#define EMAC_SR_OFF         0x104
#define EMAC_SR             (LPC_EMAC_BASE + EMAC_SR_OFF)

#define EMAC_SR_RX_EN_LSB   0
#define EMAC_SR_TX_EN_LSB   1

#define EMAC_SR_RX_EN      (1 << EMAC_SR_RX_EN_LSB)
#define EMAC_SR_TX_EN      (1 << EMAC_SR_TX_EN_LSB)
/*@}*/

/*! \name Receive Descriptor Base Address Register */
/*@{*/
#define EMAC_RXDESCR_OFF    0x108
#define EMAC_RXDESCR        (LPC_EMAC_BASE + EMAC_RXDESCR_OFF)
/*@}*/

/*! \name Receive Status Base Address Register */
/*@{*/
#define EMAC_RXSTAT_OFF     0x10C
#define EMAC_RXSTAT         (LPC_EMAC_BASE + EMAC_RXSTAT_OFF)
/*@}*/

/*! \name Receive Number of Descriptors Register */
/*@{*/
#define EMAC_RXDESCR_NUM_OFF 0x110
#define EMAC_RXDESCR_NUM     (LPC_EMAC_BASE + EMAC_RXDESCR_NUM_OFF)
/*@}*/

/*! \name Receive Produce Index Register */
/*@{*/
#define EMAC_RXPROD_IDX_OFF 0x114
#define EMAC_RXPROD_IDX     (LPC_EMAC_BASE + EMAC_RXPROD_IDX_OFF)
/*@}*/

/*! \name Receive Consume Index Register */
/*@{*/
#define EMAC_RXCONS_IDX_OFF 0x118
#define EMAC_RXCONS_IDX     (LPC_EMAC_BASE + EMAC_RXCONS_IDX_OFF)
/*@}*/

/*! \name Transmit Descriptor Base Address Register */
/*@{*/
#define EMAC_TXDESCR_OFF    0x11C
#define EMAC_TXDESCR        (LPC_EMAC_BASE + EMAC_TXDESCR_OFF)
/*@}*/

/*! \name Transmit Status Base Address Register */
/*@{*/
#define EMAC_TXSTAT_OFF     0x120
#define EMAC_TXSTAT         (LPC_EMAC_BASE + EMAC_TXSTAT_OFF)
/*@}*/

/*! \name Transmit Number of Descriptors Register */
/*@{*/
#define EMAC_TXDESCR_NUM_OFF 0x124
#define EMAC_TXDESCR_NUM     (LPC_EMAC_BASE + EMAC_TXDESCR_NUM_OFF)
/*@}*/

/*! \name Transmit Produce Index Register */
/*@{*/
#define EMAC_TXPROD_IDX_OFF 0x128
#define EMAC_TXPROD_IDX     (LPC_EMAC_BASE + EMAC_TXPROD_IDX_OFF)
/*@}*/

/*! \name Transmit Consume Index Register */
/*@{*/
#define EMAC_TXCONS_IDX_OFF 0x12C
#define EMAC_TXCONS_IDX     (LPC_EMAC_BASE + EMAC_TXCONS_IDX_OFF)
/*@}*/

/*! \name Transmit Status Vector 0 Register */
/*@{*/
#define EMAC_TSV0_OFF       0x158
#define EMAC_TSV0           (LPC_EMAC_BASE + EMAC_TSV0_OFF)

#define EMAC_TSV0_CRC_ERR_LSB       0
#define EMAC_TSV0_LEN_CHKERR_LSB    1
#define EMAC_TSV0_LEN_OUTRNG_LSB    2
#define EMAC_TSV0_DONE_LSB          3
#define EMAC_TSV0_MCAST_LSB         4
#define EMAC_TSV0_BCAST_LSB         5
#define EMAC_TSV0_PKT_DEFER_LSB     6
#define EMAC_TSV0_EXC_DEFER_LSB     7
#define EMAC_TSV0_EXC_COLL_LSB      8
#define EMAC_TSV0_LATE_COLL_LSB     9
#define EMAC_TSV0_GIANT_LSB         10
#define EMAC_TSV0_UNDERRUN_LSB      11
#define EMAC_TSV0_BYTES_LSB         12
#define EMAC_TSV0_BYTES_MSB         27
#define EMAC_TSV0_CTRL_FRAME_LSB    28
#define EMAC_TSV0_PAUSE_LSB         29
#define EMAC_TSV0_BACK_PRESS_LSB    30
#define EMAC_TSV0_VLAN_LSB          31

#define EMAC_TSV0_CRC_ERR       (1 << EMAC_TSV0_CRC_ERR_LSB)
#define EMAC_TSV0_LEN_CHKERR    (1 << EMAC_TSV0_LEN_CHKERR_LSB)
#define EMAC_TSV0_LEN_OUTRNG    (1 << EMAC_TSV0_LEN_OUTRNG_LSB)
#define EMAC_TSV0_DONE          (1 << EMAC_TSV0_DONE_LSB)
#define EMAC_TSV0_MCAST         (1 << EMAC_TSV0_MCAST_LSB)
#define EMAC_TSV0_BCAST         (1 << EMAC_TSV0_BCAST_LSB)
#define EMAC_TSV0_PKT_DEFER     (1 << EMAC_TSV0_PKT_DEFER_LSB)
#define EMAC_TSV0_EXC_DEFER     (1 << EMAC_TSV0_EXC_DEFER_LSB)
#define EMAC_TSV0_EXC_COLL      (1 << EMAC_TSV0_EXC_COLL_LSB)
#define EMAC_TSV0_LATE_COLL     (1 << EMAC_TSV0_LATE_COLL_LSB)
#define EMAC_TSV0_GIANT         (1 << EMAC_TSV0_GIANT_LSB)
#define EMAC_TSV0_UNDERRUN      (1 << EMAC_TSV0_UNDERRUN_LSB)
#define EMAC_TSV0_CTRL_FRAME    (1 << EMAC_TSV0_CTRL_FRAME_LSB)
#define EMAC_TSV0_PAUSE         (1 << EMAC_TSV0_PAUSE_LSB)
#define EMAC_TSV0_BACK_PRESS    (1 << EMAC_TSV0_BACK_PRESS_LSB)
#define EMAC_TSV0_VLAN          (1 << EMAC_TSV0_VLAN_LSB)
/*@}*/

/*! \name Transmit Status Vector 1 Register */
/*@{*/
#define EMAC_TSV1_OFF       0x15C
#define EMAC_TSV1           (LPC_EMAC_BASE + EMAC_TSV1_OFF)

#define EMAC_TSV1_BYTE_CNT_LSB  0
#define EMAC_TSV1_BYTE_CNT_MSB  15
#define EMAC_TSV1_COLL_CNT_LSB  16
#define EMAC_TSV1_COLL_CNT_MSB  19
/*@}*/

/*! \name Receive Status Vector Register */
/*@{*/
#define EMAC_RSV_OFF        0x160
#define EMAC_RSV            (LPC_EMAC_BASE + EMAC_RSV_OFF)

#define EMAC_RSV_BYTE_CNT_LSB       0
#define EMAC_RSV_BYTE_CNT_MSB       15
#define EMAC_RSV_PKT_IGNORED_LSB    16
#define EMAC_RSV_RXDV_SEEN_LSB      17
#define EMAC_RSV_CARR_SEEN_LSB      18
#define EMAC_RSV_REC_CODEV_LSB      19
#define EMAC_RSV_CRC_ERR_LSB        20
#define EMAC_RSV_LEN_CHKERR_LSB     21
#define EMAC_RSV_LEN_OUTRNG_LSB     22
#define EMAC_RSV_REC_OK_LSB         23
#define EMAC_RSV_MCAST_LSB          24
#define EMAC_RSV_BCAST_LSB          25
#define EMAC_RSV_DRIB_NIBB_LSB      26
#define EMAC_RSV_CTRL_FRAME_LSB     27
#define EMAC_RSV_PAUSE_LSB          28
#define EMAC_RSV_UNSUPP_OPC_LSB     29
#define EMAC_RSV_VLAN_LSB           30

#define EMAC_RSV_PKT_IGNORED    (1 << EMAC_RSV_PKT_IGNORED_LSB)
#define EMAC_RSV_RXDV_SEEN      (1 << EMAC_RSV_RXDV_SEEN_LSB)
#define EMAC_RSV_CARR_SEEN      (1 << EMAC_RSV_CARR_SEEN_LSB)
#define EMAC_RSV_REC_CODEV      (1 << EMAC_RSV_REC_CODEV_LSB)
#define EMAC_RSV_CRC_ERR        (1 << EMAC_RSV_CRC_ERR_LSB)
#define EMAC_RSV_LEN_CHKERR     (1 << EMAC_RSV_LEN_CHKERR_LSB)
#define EMAC_RSV_LEN_OUTRNG     (1 << EMAC_RSV_LEN_OUTRNG_LSB)
#define EMAC_RSV_REC_OK         (1 << EMAC_RSV_REC_OK_LSB)
#define EMAC_RSV_MCAST          (1 << EMAC_RSV_MCAST_LSB)
#define EMAC_RSV_BCAST          (1 << EMAC_RSV_BCAST_LSB)
#define EMAC_RSV_DRIB_NIBB      (1 << EMAC_RSV_DRIB_NIBB_LSB)
#define EMAC_RSV_CTRL_FRAME     (1 << EMAC_RSV_CTRL_FRAME_LSB)
#define EMAC_RSV_PAUSE          (1 << EMAC_RSV_PAUSE_LSB)
#define EMAC_RSV_UNSUPP_OPC     (1 << EMAC_RSV_UNSUPP_OPC_LSB)
#define EMAC_RSV_VLAN           (1 << EMAC_RSV_VLAN_LSB)
/*@}*/

/*! \name Flow Control Counter Register */
/*@{*/
#define EMAC_FCC_OFF        0x170
#define EMAC_FCC            (LPC_EMAC_BASE + EMAC_FCC_OFF)

#define EMAC_FCC_MIRR_CNT_LSB   0
#define EMAC_FCC_MIRR_CNT_MSB   15
#define EMAC_FCC_PAUSE_TIM_LSB  16
#define EMAC_FCC_PAUSE_TIM_MSB  31
/*@}*/

/*! \name Flow Control Status Register */
/*@{*/
#define EMAC_FCS_OFF        0x174
#define EMAC_FCS            (LPC_EMAC_BASE + EMAC_FCS_OFF)

#define EMAC_FCS_MIRR_CNT_LSB   0
#define EMAC_FCS_MIRR_CNT_MSB   15
/*@}*/

/*! \name Receiver Filter Control Register */
/*@{*/
#define EMAC_RFC_OFF        0x200
#define EMAC_RFC            (LPC_EMAC_BASE + EMAC_RFC_OFF)

#define EMAC_RFC_UCAST_EN_LSB       0
#define EMAC_RFC_BCAST_EN_LSB       1
#define EMAC_RFC_MCAST_EN_LSB       2
#define EMAC_RFC_UCAST_HASH_EN_LSB  3
#define EMAC_RFC_MCAST_HASH_EN_LSB  4
#define EMAC_RFC_PERFECT_EN_LSB     5
#define EMAC_RFC_MAGP_WOL_EN_LSB    12
#define EMAC_RFC_PFILT_WOL_EN_LSB   13

#define EMAC_RFC_UCAST_EN       (1 << EMAC_RFC_UCAST_EN_LSB)
#define EMAC_RFC_BCAST_EN       (1 << EMAC_RFC_BCAST_EN_LSB)
#define EMAC_RFC_MCAST_EN       (1 << EMAC_RFC_MCAST_EN_LSB)
#define EMAC_RFC_UCAST_HASH_EN  (1 << EMAC_RFC_UCAST_HASH_EN_LSB)
#define EMAC_RFC_MCAST_HASH_EN  (1 << EMAC_RFC_MCAST_HASH_EN_LSB)
#define EMAC_RFC_PERFECT_EN     (1 << EMAC_RFC_PERFECT_EN_LSB)
#define EMAC_RFC_MAGP_WOL_EN    (1 << EMAC_RFC_MAGP_WOL_EN_LSB)
#define EMAC_RFC_PFILT_WOL_EN   (1 << EMAC_RFC_PFILT_WOL_EN_LSB)
/*@}*/

/*! \name Receiver Filter WoL Registers */
/*@{*/
#define EMAC_WOLSR_OFF      0x204
#define EMAC_WOLSR          (LPC_EMAC_BASE + EMAC_WOLSR_OFF)
#define EMAC_WOLCR_OFF      0x208
#define EMAC_WOLCR          (LPC_EMAC_BASE + EMAC_WOLCR_OFF)

#define EMAC_WOL_UCAST_LSB          0
#define EMAC_WOL_BCAST_LSB          1
#define EMAC_WOL_MCAST_LSB          2
#define EMAC_WOL_UCAST_HASH_LSB     3
#define EMAC_WOL_MCAST_HASH_LSB     4
#define EMAC_WOL_PERFECT_LSB        5
#define EMAC_WOL_RX_FILTER_LSB      7
#define EMAC_WOL_MAG_PACKET_LSB     8

#define EMAC_WOL_UCAST          (1 << EMAC_WOL_UCAST_LSB)
#define EMAC_WOL_BCAST          (1 << EMAC_WOL_BCAST_LSB)
#define EMAC_WOL_MCAST          (1 << EMAC_WOL_MCAST_LSB)
#define EMAC_WOL_UCAST_HASH     (1 << EMAC_WOL_UCAST_HASH_LSB)
#define EMAC_WOL_MCAST_HASH     (1 << EMAC_WOL_MCAST_HASH_LSB)
#define EMAC_WOL_PERFECT        (1 << EMAC_WOL_PERFECT_LSB)
#define EMAC_WOL_RX_FILTER      (1 << EMAC_WOL_RX_FILTER_LSB)
#define EMAC_WOL_MAG_PACKET     (1 << EMAC_WOL_MAG_PACKET_LSB)
/*@}*/

/*! \name Hash Filter Table Registers */
/*@{*/
#define EMAC_HASHFILTERL_OFF 0x210
#define EMAC_HASHFILTERL     (LPC_EMAC_BASE + EMAC_HASHFILTERL_OFF)
#define EMAC_HASHFILTERH_OFF 0x214
#define EMAC_HASHFILTERH     (LPC_EMAC_BASE + EMAC_HASHFILTERH_OFF)
/*@}*/

/*! \name Interrupt Registers */
/*@{*/
#define EMAC_INT_STAT_OFF   0xFE0
#define EMAC_INT_STAT       (LPC_EMAC_BASE + EMAC_INT_STAT_OFF)
#define EMAC_INT_ENA_OFF    0xFE4
#define EMAC_INT_ENA        (LPC_EMAC_BASE + EMAC_INT_ENA_OFF)
#define EMAC_INT_CLR_OFF    0xFE8
#define EMAC_INT_CLR        (LPC_EMAC_BASE + EMAC_INT_CLR_OFF)
#define EMAC_INT_SET_OFF    0xFEC
#define EMAC_INT_SET        (LPC_EMAC_BASE + EMAC_INT_SET_OFF)

#define EMAC_INT_RX_OVERRUN_LSB     0
#define EMAC_INT_RX_ERR_LSB         1
#define EMAC_INT_RX_FIN_LSB         2
#define EMAC_INT_RX_DONE_LSB        3
#define EMAC_INT_TX_UNDERRUN_LSB    4
#define EMAC_INT_TX_ERR_LSB         5
#define EMAC_INT_TX_FIN_LSB         6
#define EMAC_INT_TX_DONE_LSB        7
#define EMAC_INT_SOFT_INT_LSB       12
#define EMAC_INT_WAKEUP_LSB         13

#define EMAC_INT_RX_OVERRUN     (1 << EMAC_INT_RX_OVERRUN_LSB)
#define EMAC_INT_RX_ERR         (1 << EMAC_INT_RX_ERR_LSB)
#define EMAC_INT_RX_FIN         (1 << EMAC_INT_RX_FIN_LSB)
#define EMAC_INT_RX_DONE        (1 << EMAC_INT_RX_DONE_LSB)
#define EMAC_INT_TX_UNDERRUN    (1 << EMAC_INT_TX_UNDERRUN_LSB)
#define EMAC_INT_TX_ERR         (1 << EMAC_INT_TX_ERR_LSB)
#define EMAC_INT_TX_FIN         (1 << EMAC_INT_TX_FIN_LSB)
#define EMAC_INT_TX_DONE        (1 << EMAC_INT_TX_DONE_LSB)
#define EMAC_INT_SOFT_INT       (1 << EMAC_INT_SOFT_INT_LSB)
#define EMAC_INT_WAKEUP         (1 << EMAC_INT_WAKEUP_LSB)
/*@}*/

/*! \name Power-Down Register */
/*@{*/
#define EMAC_PD_OFF         0xFF4
#define EMAC_PD             (LPC_EMAC_BASE + EMAC_PD_OFF)

#define EMAC_PD_POWER_DOWN_LSB  31

#define EMAC_PD_POWER_DOWN      (1 << EMAC_PD_POWER_DOWN_LSB)
/*@}*/

/*! \name Module ID Register */
/*@{*/
#define EMAC_MODULE_ID_OFF  0xFFC
#define EMAC_MODULE_ID      (LPC_EMAC_BASE + EMAC_MODULE_ID_OFF)
/*@}*/

/*! \name Descriptor Structures */
/*@{*/
typedef struct _EMAC_DESCRIPTOR {
    uint8_t *desc_packet;
    uint32_t desc_control;
} EMAC_DESCRIPTOR;

typedef struct EMAC_RXSTATUS {
    uint32_t rxs_info;
    uint32_t rxs_hashcrc;
} EMAC_RXSTATUS;
/*@}*/

/*! \name Receive Descriptor Control Word */
/*@{*/
#define EMAC_RCTRL_SIZE(n)        (n&0x7FF)
#define EMAC_RCTRL_INT            0x80000000
/*@}*/

/*! \name Receive Descriptor Status Word */
/*@{*/
#define EMAC_RINFO_SIZE           0x000007FF
#define EMAC_RINFO_CTRL_FRAME     0x00040000
#define EMAC_RINFO_VLAN           0x00080000
#define EMAC_RINFO_FAIL_FILT      0x00100000
#define EMAC_RINFO_MCAST          0x00200000
#define EMAC_RINFO_BCAST          0x00400000
#define EMAC_RINFO_CRC_ERR        0x00800000
#define EMAC_RINFO_SYM_ERR        0x01000000
#define EMAC_RINFO_LEN_ERR        0x02000000
#define EMAC_RINFO_RANGE_ERR      0x04000000
#define EMAC_RINFO_ALIGN_ERR      0x08000000
#define EMAC_RINFO_OVERRUN        0x10000000
#define EMAC_RINFO_NO_DESCR       0x20000000
#define EMAC_RINFO_LAST_FLAG      0x40000000
#define EMAC_RINFO_ERR            0x80000000
#define EMAC_RINFO_ERR_MASK      (EMAC_RINFO_FAIL_FILT | EMAC_RINFO_CRC_ERR | \
                                  EMAC_RINFO_SYM_ERR | EMAC_RINFO_LEN_ERR   | \
                                  EMAC_RINFO_ALIGN_ERR | EMAC_RINFO_OVERRUN)
/*@}*/

/*! \name Transmit Descriptor Control Word */
/*@{*/
#define EMAC_TCTRL_SIZE           0x000007FF
#define EMAC_TCTRL_OVERRIDE       0x04000000
#define EMAC_TCTRL_HUGE           0x08000000
#define EMAC_TCTRL_PAD            0x10000000
#define EMAC_TCTRL_CRC            0x20000000
#define EMAC_TCTRL_LAST           0x40000000
#define EMAC_TCTRL_INT            0x80000000
/*@}*/

/*! \name Transmit Descriptor Status Word */
/*@{*/
#define EMAC_TINFO_COL_CNT        0x01E00000
#define EMAC_TINFO_DEFER          0x02000000
#define EMAC_TINFO_EXCESS_DEF     0x04000000
#define EMAC_TINFO_EXCESS_COL     0x08000000
#define EMAC_TINFO_LATE_COL       0x10000000
#define EMAC_TINFO_UNDERRUN       0x20000000
#define EMAC_TINFO_NO_DESCR       0x40000000
#define EMAC_TINFO_ERR            0x80000000
/*@}*/

#endif
