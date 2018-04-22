#ifndef _ARCH_ARM_AT91_CCFG_H_
#define _ARCH_ARM_AT91_CCFG_H_

/*
 * Copyright (C) 2006 by egnite Software GmbH. All rights reserved.
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
 * \file arch/arm/at91_ccfg.h
 * \brief AT91 chip configuration.
 *
 * \verbatim
 *
 * $Log$
 * Revision 1.1  2006/08/31 19:10:37  haraldkipp
 * New peripheral register definitions for the AT91SAM9260.
 *
 *
 * \endverbatim
 */

/*!
 * \addtogroup xgNutArchArmAt91Ccfg
 */
/*@{*/

/*! \name Bus Matrix TCM Configuration Register */
/*@{*/
#define CCFG_TCMR_OFF               0x00000000  /*!< \brief Bus matrix TCM configuration register offset. */
#define CCFG_TCMR   (CCFG_BASE + CCFG_TCMR_OFF) /*!< \brief Bus matrix TCM configuration register address. */
#define CCFG_ITCM_SIZE              0x0000000F  /*!< \brief ITCM enabled memory size mask. */
#define CCFG_ITCM_SIZE_32KB         0x00000006  /*!< \brief 32 kByte ITCM enabled. */
#define CCFG_DTCM_SIZE              0x000000F0  /*!< \brief DTCM enabled memory size mask. */
#define CCFG_DTCM_SIZE_32KB         0x00000060  /*!< \brief 32 kByte DTCM enabled. */
#define CCFG_DTCM_SIZE_64KB         0x00000070  /*!< \brief 64 kByte DTCM enabled. */
#define CCFG_TCM_NWS_1              0x00000800  /*!< \brief 1 TCM wait state. */
/*@}*/

/*! \name DDR Multiport Register */
/*@{*/
#define CCFG_DDRMPR_OFF             0x00000008  /*!< \brief DDR multiport register offset. */
#define CCFG_DDRMPR (CCFG_BASE + CCFG_DDRMPR_OFF) /*!< \brief DDR multiport register address. */
#define CCFG_DDRMP_DIS              0x00000001  /*!< \brief Disable multiport. */
/*@}*/

/*! \name Chip Select Assignment Register */
/*@{*/
#define CCFG_CSA_OFF                0x0000000C  /*!< \brief Chip select assignment register offset. */
#define CCFG_CSA    (CCFG_BASE + CCFG_CSA_OFF)  /*!< \brief Chip select assignment register address. */
#define CCFG_CS1A                   0x00000002  /*!< \brief SDRAM at chip select 1. */
#define CCFG_CS3A                   0x00000008  /*!< \brief SmartMedia at chip select 3. */
#define CCFG_CS4A                   0x00000010  /*!< \brief First CompactFlash slot at chip select 4. */
#define CCFG_CS5A                   0x00000020  /*!< \brief Second CompactFlash slot at chip select 5. */
#define CCFG_DBPUC                  0x00000100  /*!< \brief Data bus pull-ups disabled. */
#define CCFG_VDDIOMSEL              0x00010000  /*!< \brief 3.3V memory. */
/*@}*/

/*! \name Chip Select Assignment Register */
/*@{*/
#define CCFG_EBICSA_OFF             0x00000018  /*!< \brief EBI chip select assignment register offset. */
#define CCFG_EBICSA (CCFG_BASE + CCFG_EBICSA_OFF) /*!< \brief EBI chip select assignment register address. */
#define CCFG_EBI_CS1A               CCFG_CS1A   /*!< \brief SDRAM controller at chip select 1. */
#define CCFG_EBI_CS3A               CCFG_CS3A   /*!< \brief SmartMedia at chip select 3. */
#define CCFG_EBI_CS4A               CCFG_CS4A   /*!< \brief First CompactFlash slot at chip select 4. */
#define CCFG_EBI_CS5A               CCFG_CS5A   /*!< \brief Second CompactFlash slot at chip select 5. */
#define CCFG_EBI_DBPUC              CCFG_DBPUC  /*!< \brief Data bus pull-ups disabled. */
#define CCFG_EBI_DRIVE              0x00030000  /*!< \brief EBI I/O drive configuration mask. */
#define CCFG_EBI_DRIVE_3V3LO        0x00010000  /*!< \brief Enable 3.3V EBI low drive. */
#define CCFG_EBI_DRIVE_1V8HI        0x00020000  /*!< \brief Enable 1.8V EBI high drive. */
#define CCFG_EBI_DRIVE_3V3HI        0x00030000  /*!< \brief Enable 3.3V EBI high drive. */
#define CCFG_DDR_DRIVE_HI           0x00040000  /*!< \brief Enable DDR2 high drive. */
/*@}*/

/*@} xgNutArchArmAt91Ccfg */

#endif                          /* _ARCH_ARM_AT91_CCFG_H_ */
