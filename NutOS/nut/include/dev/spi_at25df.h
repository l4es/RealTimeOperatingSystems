#ifndef _DEV_AT25DF_H_
#define _DEV_AT25DF_H_
/*
 * Copyright (C) 2015 by Ole Reinhardt <ole.reinhardt@embedded-it.de>
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
 * \file dev/spi_at25df.h
 * \brief Dataflash helper routines.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <sys/types.h>
#include <stdint.h>
#include <dev/blockdev.h>
#include <dev/spibus.h>


#define MAX_AT25_CMDLEN         6
#define AT25_BLOCK_ERASE_WAIT   1500      // max: 4K: 200ms, 32K: 600ms, 64K: 950ms
#define AT25_CHIP_ERASE_WAIT    50000     // max: 40s
#define AT25_WRITE_POLLS        1000      // Poll cycles busy looping. Might be better to implement same wait as for other operations. 

#define AT25_MAX_SPEED_SLOW     50000000
#define AT25_MAX_SPEED_MED      85000000
#define AT25_MAX_SPEED_HIGH     100000000


/*!
 * \name AT25 DataFlash Commands
 */
/*@{*/
/*! \brief Continuos read (high frequency).
 *
 * Reads a continous stream in high speed mode.
 */
#define DFCMD_READ_ARRAY_SLOW    0x03
#define DFCMD_READ_ARRAY_MED     0x0B
#define DFCMD_READ_ARRAY_FAST    0x1B

/*! \brief Block erase 4k.
 */
#define DFCMD_BLOCK_ERASE_4K     0x20
/*! \brief Block erase 32k.
 */
#define DFCMD_BLOCK_ERASE_32K    0x52
/*! \brief Block erase 64k.
 */
#define DFCMD_BLOCK_ERASE_64K    0xd8
/*! \brief Chip erase
 */
#define DFCMD_CHIP_ERASE         0xC7
/*! \brief Write bytes/page.
 */
#define DFCMD_WRITE              0x02
/*! \brief Read / Write status register.
 */
#define DFCMD_READ_STATUS        0x05
#define DFCMD_READ_DEVICEID      0x9F

#define DFCMD_WRITE_STATUS1      0x01

#define DFCMD_WRITE_ENABLE       0x06
#define DFCMD_WRITE_DISABLE      0x04
/*@}*/



/*!
 * \brief AT25D DataFlash parameter structure type.
 */
typedef struct _AT25DF_INFO AT25DF_INFO;

/*!
 * \brief AT25DF DataFlash parameter structure.
 */
struct _AT25DF_INFO {
    /*! \brief Number of bits the erase block number needs to be shifted to retreive the address. */
    uint_fast8_t at25df_ebshft;
    /*! \brief Total number of pages. */
    uint32_t at25df_erase_blocks;
    /*! \brief Number of bytes per erase block. */
    size_t at25df_ebsize;
    /*! \brief Number of bytes per page */
    size_t at25df_psize;
    /*! \brief device id1. */
    uint_fast8_t at25df_id1;
    /*! \brief device id2. */
    uint_fast8_t at25df_id2;
};


extern AT25DF_INFO at25df_info[];
extern uint_fast8_t at25df_known_types;

#endif
