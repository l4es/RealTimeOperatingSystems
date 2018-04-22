#ifndef _ARCH_ARM_AT91_DDRSDRC_H_
#define _ARCH_ARM_AT91_DDRSDRC_H_

/*
 * Copyright (C) 2013 by egnite GmbH
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
 * \file arch/arm/at91_ddrsdrc.h
 * \brief AT91 DDR SDR SDRAM Controller.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

/*!
 * \addtogroup xgNutArchArmAt91DdrSdrc
 */
/*@{*/

/*! \name DDRSDRC Mode Register */
/*@{*/
#define DDRSDRC_MR_OFF                  0x00000000      /*!< \brief DDRSDRC mode register offset. */
#define DDRSDRC0_MR (DDRSDRC0_BASE + DDRSDRC_MR_OFF)    /*!< \brief DDRSDRC0 mode register address. */
#define DDRSDRC1_MR (DDRSDRC1_BASE + DDRSDRC_MR_OFF)    /*!< \brief DDRSDRC1 mode register address. */
#define DDRSDRC_MODE                    0x00000007      /*!< \brief Command mode mask. */
#define DDRSDRC_MODE_NORMAL             0x00000000      /*!< \brief Normal mode. */
#define DDRSDRC_MODE_NOP                0x00000001      /*!< \brief Issue NOP command when accessed. */
#define DDRSDRC_MODE_PRCGALL            0x00000002      /*!< \brief Issue "All Banks Precharge" command when accessed. */
#define DDRSDRC_MODE_LMR                0x00000003      /*!< \brief Issue "Load Mode Register" command when accessed. */
#define DDRSDRC_MODE_RFSH               0x00000004      /*!< \brief Issue "Auto Refresh" command when accessed. */
#define DDRSDRC_MODE_EXT_LMR            0x00000005      /*!< \brief Issue "Extended Load Mode Register" command when accessed. */
#define DDRSDRC_MODE_DEEP               0x00000006      /*!< \brief Enter deep power down mode. */
/*@}*/

/*! \name DDRSDRC Refresh Timer Register */
/*@{*/
#define DDRSDRC_RTR_OFF                 0x00000004      /*!< \brief DDRSDRC refresh timer register offset. */
#define DDRSDRC0_RTR (DDRSDRC0_BASE + DDRSDRC_RTR_OFF)  /*!< \brief DDRSDRC0 refresh timer register address. */
#define DDRSDRC1_RTR (DDRSDRC1_BASE + DDRSDRC_RTR_OFF)  /*!< \brief DDRSDRC1 refresh timer register address. */
#define DDRSDRC_COUNT                   0x00000FFF      /*!< \brief Refresh timer count mask. */
/*@}*/

/*! \name DDRSDRC Configuration Register */
/*@{*/
#define DDRSDRC_CR_OFF                  0x00000008      /*!< \brief DDRSDRC configuration register offset. */
#define DDRSDRC0_CR (DDRSDRC0_BASE + DDRSDRC_CR_OFF)    /*!< \brief DDRSDRC0 configuration register address. */
#define DDRSDRC1_CR (DDRSDRC1_BASE + DDRSDRC_CR_OFF)    /*!< \brief DDRSDRC1 configuration register address. */
#define DDRSDRC_NC                      0x00000003      /*!< \brief Number of column bits. */
#define DDRSDRC_NC_LSB                           0
#define DDRSDRC_NC_DDROFF                        9
#define DDRSDRC_NC_SDROFF                        8
#define DDRSDRC_NC_DDR9_SDR8            0x00000000      /*!< \brief 9 DDR or 8 SDR column bits. */
#define DDRSDRC_NC_DDR10_SDR9           0x00000001      /*!< \brief 10 DDR or 9 SDR column bits. */
#define DDRSDRC_NC_DDR11_SDR10          0x00000002      /*!< \brief 11 DDR or 10 SDR column bits. */
#define DDRSDRC_NC_DDR12_SDR11          0x00000003      /*!< \brief 12 DDR or 11 SDR column bits. */
#define DDRSDRC_NR                      0x0000000C      /*!< \brief Number of row bits. */
#define DDRSDRC_NR_LSB                           2
#define DDRSDRC_NR_OFF                          11
#define DDRSDRC_NR_11                   0x00000000      /*!< \brief 11 row bits. */
#define DDRSDRC_NR_12                   0x00000004      /*!< \brief 12 row bits. */
#define DDRSDRC_NR_13                   0x00000008      /*!< \brief 13 row bits. */
#define DDRSDRC_NR_14                   0x0000000C      /*!< \brief 14 row bits. */
#define DDRSDRC_CAS                     0x00000070      /*!< \brief CAS latency. */
#define DDRSDRC_CAS_2                   0x00000020      /*!< \brief CAS latency of 2 cycles. */
#define DDRSDRC_CAS_3                   0x00000030      /*!< \brief CAS latency of 3 cycles. */

#define DDRSDRC_DLL                     0x00000080      /*!< \brief Reset DLL. */
#define DDRSDRC_DDIC_DS                 0x00000100      /*!< \brief Output driver impedance control. */
#define DDRSDRC_DIS_DLL                 0x00000200      /*!< \brief Disable DLL. */
#define DDRSDRC_OCD                     0x00007000      /*!< \brief Off-chip driver mask. */
#define DDRSDRC_OCD_LSB                         12      /*!< \brief Off-chip driver mask LSB. */
#define DDRSDRC_EBISHARE                0x00010000      /*!< \brief External bus interface is shared. */
#define DDRSDRC_ACTBST                  0x00040000      /*!< \brief Active bank X to burst stop read access bank Y. */
/*@}*/

/*! \name DDRSDRC Timing Parameter 0 Register */
/*@{*/
#define DDRSDRC_T0PR_OFF                0x0000000C      /*!< \brief DDRSDRC timing parameter 0 register offset. */
#define DDRSDRC0_T0PR (DDRSDRC0_BASE + DDRSDRC_T0PR_OFF)/*!< \brief DDRSDRC0 timing parameter 0 register address. */
#define DDRSDRC1_T0PR (DDRSDRC1_BASE + DDRSDRC_T0PR_OFF)/*!< \brief DDRSDRC1 timing parameter 0 register address. */
#define DDRSDRC_TRAS                    0x0000000F
#define DDRSDRC_TRAS_LSB                         0
#define DDRSDRC_TRCD                    0x000000F0
#define DDRSDRC_TRCD_LSB                         4
#define DDRSDRC_TWR                     0x00000F00
#define DDRSDRC_TWR_LSB                          8
#define DDRSDRC_TRC                     0x0000F000
#define DDRSDRC_TRC_LSB                         12
#define DDRSDRC_TRP                     0x000F0000
#define DDRSDRC_TRP_LSB                         16
#define DDRSDRC_TRRD                    0x00F00000
#define DDRSDRC_TRRD_LSB                        20
#define DDRSDRC_TWTR                    0x07000000
#define DDRSDRC_TWTR_LSB                        24
#define DDRSDRC_REDUCE_WRRD             0x08000000
#define DDRSDRC_TMRD                    0xF0000000
#define DDRSDRC_TMRD_LSB                        28

/*@}*/

/*! \name DDRSDRC Timing Parameter 1 Register */
/*@{*/
#define DDRSDRC_T1PR_OFF                0x00000010      /*!< \brief DDRSDRC timing parameter 1 register offset. */
#define DDRSDRC0_T1PR (DDRSDRC0_BASE + DDRSDRC_T1PR_OFF)/*!< \brief DDRSDRC0 timing parameter 1 register address. */
#define DDRSDRC1_T1PR (DDRSDRC1_BASE + DDRSDRC_T1PR_OFF)/*!< \brief DDRSDRC1 timing parameter 1 register address. */
#define DDRSDRC_TRFC                    0x0000001F
#define DDRSDRC_TRFC_LSB                         0
#define DDRSDRC_TXSNR                   0x0000FF00
#define DDRSDRC_TXSNR_LSB                        8
#define DDRSDRC_TXSRD                   0x00FF0000
#define DDRSDRC_TXSRD_LSB                       16
#define DDRSDRC_TXP                     0x0F000000
#define DDRSDRC_TXP_LSB                         24
/*@}*/

/*! \name DDRSDRC Timing Parameter 2 Register */
/*@{*/
#define DDRSDRC_T2PR_OFF                0x00000014      /*!< \brief DDRSDRC timing parameter 2 register offset. */
#define DDRSDRC0_T2PR (DDRSDRC0_BASE + DDRSDRC_T2PR_OFF)/*!< \brief DDRSDRC0 timing parameter 2 register address. */
#define DDRSDRC1_T2PR (DDRSDRC1_BASE + DDRSDRC_T2PR_OFF)/*!< \brief DDRSDRC1 timing parameter 2 register address. */
#define DDRSDRC_TXARD                   0x0000000F
#define DDRSDRC_TXARD_LSB                        0
#define DDRSDRC_TXARDS                  0x000000F0
#define DDRSDRC_TXARDS_LSB                       4
#define DDRSDRC_TRPA                    0x00000F00
#define DDRSDRC_TRPA_LSB                         8
#define DDRSDRC_TRTP                    0x00007000
#define DDRSDRC_TRTP_LSB                        12
/*@}*/

/*! \name DDRSDRC Low-Power Register */
/*@{*/
#define DDRSDRC_LPR_OFF                 0x0000001C      /*!< \brief DDRSDRC low-power register offset. */
#define DDRSDRC0_LPR (DDRSDRC0_BASE + DDRSDRC_LPR_OFF)  /*!< \brief DDRSDRC0 low-power register address. */
#define DDRSDRC1_LPR (DDRSDRC1_BASE + DDRSDRC_LPR_OFF)  /*!< \brief DDRSDRC1 low-power register address. */
#define DDRSDRC_LPCB                    0x00000003
#define DDRSDRC_LPCB_DISABLED           0x00000000      /*!< \brief Low-power feature inhibited. */
#define DDRSDRC_LPCB_SELFREFRESH        0x00000001      /*!< \brief Issues a self refresh, clocks are disabled and CKE is set low. */
#define DDRSDRC_LPCB_POWERDOWN          0x00000002      /*!< \brief Issues a power-down after each access, CKE set to low. */
#define DDRSDRC_LPCB_DEEP_PWD           0x00000003      /*!< \brief Issues a deep power-down. */
#define DDRSDRC_CLK_FR                  0x00000004
#define DDRSDRC_PASR                    0x00000070
#define DDRSDRC_PASR_LSB                         4
#define DDRSDRC_TCR                     0x00000300
#define DDRSDRC_TCR_LSB                          8
#define DDRSDRC_DS                      0x00000C00
#define DDRSDRC_DS_LSB                          10
#define DDRSDRC_TIMEOUT                 0x00003000
#define DDRSDRC_TIMEOUT_0               0x00000000      /*!< \brief Activate low-power mode immediately. */
#define DDRSDRC_TIMEOUT_64              0x00001000      /*!< \brief Activate low-power mode after 64 clock cycles. */
#define DDRSDRC_TIMEOUT_128             0x00002000      /*!< \brief Activate low-power mode after 128 clock cycles. */
#define DDRSDRC_APDE                    0x00010000
#define DDRSDRC_UPD_MR                  0x00300000
#define DDRSDRC_UPD_MR_DISABLED         0x00000000
#define DDRSDRC_UPD_MR_EBI              0x00100000
#define DDRSDRC_UPD_MR_NO_EBI           0x00200000
/*@}*/

/*! \name DDRSDRC Memory Device Register */
/*@{*/
#define DDRSDRC_MD_OFF                  0x00000020      /*!< \brief DDRSDRC memory device register offset. */
#define DDRSDRC0_MD (DDRSDRC0_BASE + DDRSDRC_MD_OFF)    /*!< \brief DDRSDRC0 memory device register address. */
#define DDRSDRC1_MD (DDRSDRC1_BASE + DDRSDRC_MD_OFF)    /*!< \brief DDRSDRC1 memory device register address. */
#define DDRSDRC_MD_SDR_SDRAM            0x00000000      /*!< \brief SDR SDRAM. */
#define DDRSDRC_MD_LP_SDR_SDRAM         0x00000001      /*!< \brief Low-power SDR SDRAM. */
#define DDRSDRC_MD_DDR_SDRAM            0x00000002      /*!< \brief DDR SDRAM. */
#define DDRSDRC_MD_LP_DDR_SDRAM         0x00000003      /*!< \brief Low-power DDR SDRAM. */
#define DDRSDRC_MD_DDR2_SDRAM           0x00000006      /*!< \brief DDR2 SDRAM. */
#define DDRSDRC_MD_DBW_16               0x00000010      /*!< \brief 16 Bits datas bus. */
/*@}*/

/*! \name DDRSDRC DLL Information Register */
/*@{*/
#define DDRSDRC_DLL_OFF                 0x00000024      /*!< \brief DDRSDRC DLL information register offset. */
#define DDRSDRC0_DLL (DDRSDRC0_BASE + DDRSDRC_DLL_OFF)  /*!< \brief DDRSDRC0 DLL information register address. */
#define DDRSDRC1_DLL (DDRSDRC1_BASE + DDRSDRC_DLL_OFF)  /*!< \brief DDRSDRC1 DLL information register address. */
#define DDRSDRC_DLL_MDINC               0x00000001
#define DDRSDRC_DLL_MDDEC               0x00000002
#define DDRSDRC_DLL_MDOVF               0x00000004
#define DDRSDRC_DLL_MDVAL               0x0000FF00
#define DDRSDRC_DLL_MDVAL_LSB                    8
/*@}*/

/*! \name DDRSDRC High Speed Register */
/*@{*/
#define DDRSDRC_HS_OFF                  0x0000002C      /*!< \brief DDRSDRC high speed register offset. */
#define DDRSDRC0_HS (DDRSDRC0_BASE + DDRSDRC_HS_OFF)    /*!< \brief DDRSDRC0 high speed register address. */
#define DDRSDRC1_HS (DDRSDRC1_BASE + DDRSDRC_HS_OFF)    /*!< \brief DDRSDRC1 high speed register address. */
#define DDRSDRC_DIS_ANTICIP_READ        0x00000004      /*!< \brief Disable anticipated read access. */
/*@}*/

/*! \name DDRSDRC Delay I/O Registers */
/*@{*/
#define DDRSDRC_DELAY_OFF(x) ((x) * 4 + 0x00000040)     /*!< \brief DDRSDRC delay I/O register offsets. */
#define DDRSDRC0_DELAY(x) (DDRSDRC0_BASE + DDRSDRC_DELAY_OFF(x)) /*!< \brief DDRSDRC0 delay I/O register addresses. */
#define DDRSDRC1_DELAY(x) (DDRSDRC1_BASE + DDRSDRC_DELAY_OFF(x)) /*!< \brief DDRSDRC1 delay I/O register addresses. */
#define DDRSDRC_DELAY1                  0x0000000F
#define DDRSDRC_DELAY1_LSB                       0
#define DDRSDRC_DELAY2                  0x000000F0
#define DDRSDRC_DELAY2_LSB                       4
#define DDRSDRC_DELAY3                  0x00000F00
#define DDRSDRC_DELAY3_LSB                       8
#define DDRSDRC_DELAY4                  0x0000F000
#define DDRSDRC_DELAY4_LSB                      12
#define DDRSDRC_DELAY5                  0x000F0000
#define DDRSDRC_DELAY5_LSB                      16
#define DDRSDRC_DELAY6                  0x00F00000
#define DDRSDRC_DELAY6_LSB                      20
#define DDRSDRC_DELAY7                  0x0F000000
#define DDRSDRC_DELAY7_LSB                      24
#define DDRSDRC_DELAY8                  0xF0000000
#define DDRSDRC_DELAY8_LSB                      28
/*@}*/

/*! \name DDRSDRC Write Protect Mode Register */
/*@{*/
#define DDRSDRC_WPMR_OFF                0x000000E4      /*!< \brief DDRSDRC write protect mode register offset. */
#define DDRSDRC0_WPMR (DDRSDRC0_BASE + DDRSDRC_WPMR_OFF)/*!< \brief DDRSDRC0 write protect mode register address. */
#define DDRSDRC1_WPMR (DDRSDRC1_BASE + DDRSDRC_WPMR_OFF)/*!< \brief DDRSDRC1 write protect mode register address. */
#define DDRSDRC_WPEN                    0x00000001
#define DDRSDRC_WPKEY                   0x44445200
/*@}*/

/*! \name DDRSDRC Write Protect Status Register */
/*@{*/
#define DDRSDRC_WPSR_OFF                0x000000E8      /*!< \brief DDRSDRC write protect status register offset. */
#define DDRSDRC0_WPSR (DDRSDRC0_BASE + DDRSDRC_WPSR_OFF)/*!< \brief DDRSDRC0 write protect status register address. */
#define DDRSDRC1_WPSR (DDRSDRC1_BASE + DDRSDRC_WPSR_OFF)/*!< \brief DDRSDRC1 write protect status register address. */
#define DDRSDRC_WPVS                    0x00000001      /*!< \brief Write protect violation. */
#define DDRSDRC_WPVSRC                  0x00FFFF00      /*!< \brief Write protect violation source mask. */
#define DDRSDRC_WPVSRC_LSB                       8      /*!< \brief Write protect violation source LSB. */
/*@}*/

/*@} xgNutArchArmAt91DdrSdrc */

#endif /* _ARCH_ARM_AT91_DDRSDRC_H_ */
