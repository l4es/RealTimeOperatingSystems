/*
 * Copyright (C) 2015 by Ole Reinhardt <ole.reinhardt@embedded-it.de>
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
 * \file dev/spi_at25df_info.c
 * \brief Adesto/Atmel AT25DF SPI DataFlash types.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <dev/spi_at25df.h>

/*!
 * \addtogroup xgSpiInfoAt25df
 */
/*@{*/

/*! \brief Parameter table of known DataFlash types. */
AT25DF_INFO at25df_info[] = {
    {12, 1024, 4096, 256, 0x47, 0x00}, /* AT25DF321  - 4MB */
    {12, 1024, 4096, 256, 0x47, 0x01}, /* AT25DF321A - 4MB */
    {12, 2048, 4096, 256, 0x48, 0x00}, /* AT25DF641  - 8MB */
    {12, 2048, 4096, 256, 0x48, 0x01}, /* AT25DF641A - 8MB */
    {12,  256, 4096, 256, 0x20, 0x14}, /* Macronix MX25L8006E - 1MB */
    {12,  512, 4096, 256, 0x20, 0x15}, /* Macronix MX25L1606E - 2MB */
};

/*! \brief Number of known Dataflash types. */
uint_fast8_t at25df_known_types = sizeof(at25df_info) / sizeof(AT25DF_INFO);

/*@}*/
