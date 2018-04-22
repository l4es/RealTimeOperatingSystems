#ifndef _LPC177x_8x_MMCARD_SDIO_H_
#define _LPC177x_8x_MMCARD_SDIO_H_

/*
 * Copyright (C) 2005 by egnite Software GmbH. All rights reserved.
 * Copyright (C) 2012 by Rob van Lieshout (info@pragmalab.nl)
 * Copyright (C) 2012 by Ole Reinhardt (ole.reinhardt@embedded-it.de)
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
 * \file arch/cm3/nxp/lpc177x_8x_mmcard_sdio.h
 * \brief Header file for multimedia card driver.
 *
 * \verbatim
 * $Id: $
 * \endverbatim
 */

#include <cfg/mmci.h>
#include <sys/device.h>
#include <sys/file.h>


/*
 *  MCI power active level -> set to (0) or (1) depending your board HW
 */
#ifndef BRD_MCI_POWERED_ACTIVE_LEVEL
#define BRD_MCI_POWERED_ACTIVE_LEVEL    (1)     /* IAR LPC1788 devboared needs active HIGH level */
#endif

#ifndef MMC_BLOCK_SIZE
#define MMC_BLOCK_SIZE  512
#endif


/*!
 * \addtogroup xgMmCard
 */
/*@{*/

/*!
 * \name Control Codes
 */
/*@{*/

/*! \brief Retrieve card status. */
#define MMCARD_GETSTATUS    0x2001
/*! \brief Retrieve operation condition register. */
#define MMCARD_GETOCR       0x2002
/*! \brief Retrieve card identification. */
#define MMCARD_GETCID       0x2003
/*! \brief Retrieve card specific data. */
#define MMCARD_GETCSD       0x2004
/*! \brief Retrieve extended card specific data. */
#define MMCARD_GETEXTCSD    0x2005

/*!
 * \brief Multimedia card identification register.
 */
typedef struct NUT_PACKED_TYPE _MMC_CSD {
    /*! \brief Card specification. */
    u_char mmcsd_spec;
    /*! \brief Data read access time. */
    u_char mmcsd_taac;
    /*! \brief Data read access time 2. */
    u_char mmcsd_nsac;
    /*! \brief Maximum data transfer rate. */
    u_char mmcsd_speed;
    /*! \brief Card command classes and max. read block length. */
    u_char mmcsd_ccc_bl[2];
    /*! \brief Read-only fields.
     *
     * - [0] 0..1 Device size bits 10..11.
     * - [0] 2..3 Reserved.
     * - [0] 4    DSR implemented.
     * - [0] 5    Read block misalignment.
     * - [0] 6    Write block misalignment.
     * - [0] 7    Partial blocks for read allowed.
     * - [1] 0..7 Device size bits 2..9.
     * - [2] 0..2 Max. read current at VDD max.
     * - [2] 3..5 Max. read current at VDD min.
     * - [2] 6..7 Device size bits 0..1.
     * - [3] 0..1 Device size multiplier bits 1..2.
     * - [3] 2..4 Max. write current at VDD max.
     * - [3] 5..7 Max. write current at VDD min.
     * - [4] 0..1 Erase group size multiplier bits 3..4.
     * - [4] 2..6 Erase group size.
     * - [4] 7    Device size multiplier bit 0.
     * - [5] 0..4 Write protect group size.
     * - [5] 5..7 Erase group size multiplier bits 0..2.
     * - [6] 0..1 Max. write data block length bits 2..3.
     * - [6] 2..4 Read to write speed factor.
     * - [6] 5..6 Reserved.
     * - [6] 7    Write protect group enable.
     * - [7] 0    Content protection application.
     * - [7] 1..4 Reserved.
     * - [7] 5    Partial blocks for write allowed.
     * - [7] 6..7 Max. write data block length bits 0..1.
     */
    u_char mmcsd_rfld[8];
    /*! \brief Programmable field. */
    u_char mmcsd_pfld;
    /*! \brief Checksum. */
    u_char mmcsd_crc;
} MMC_CSD;

/*@}*/

#endif
