#ifndef _DEV_AT45DB_H_
#define _DEV_AT45DB_H_
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
 * \file dev/at45db.h
 * \brief Dataflash helper routines.
 *
 * \verbatim
 *
 * $Log$
 * Revision 1.5  2009/01/17 11:26:47  haraldkipp
 * Getting rid of two remaining BSD types in favor of stdint.
 * Replaced 'u_int' by 'unsinged int' and 'uptr_t' by 'uintptr_t'.
 *
 * Revision 1.4  2008/08/11 06:59:59  haraldkipp
 * BSD types replaced by stdint types (feature request #1282721).
 *
 * Revision 1.3  2008/02/15 17:10:44  haraldkipp
 * At25dfPageErase selected the wrong bank. Fixed. Parameter pgn (page number)
 * of At25dfPageWrite() changed from unsigned int to unsigned long.
 * New routines At25dfPages() and At25dfPageSize() allow to determine the
 * chip's layout.
 *
 * Revision 1.2  2006/10/08 16:48:09  haraldkipp
 * Documentation fixed
 *
 * Revision 1.1  2006/09/29 12:41:55  haraldkipp
 * Added support for AT45 serial DataFlash memory chips. Currently limited
 * to AT91 builds.
 *
 *
 * \endverbatim
 */

#include <sys/types.h>
#include <stdint.h>
#include <dev/blockdev.h>
#include <dev/spibus.h>

/*!
 * \brief AT25D DataFlash parameter structure type.
 */
typedef struct _AT25D_INFO AT25D_INFO;

/*!
 * \brief AT25D DataFlash parameter structure.
 */
struct _AT25D_INFO {
    /*! \brief Least significant bit of the page parameter. */
    uint_fast8_t at25d_pshft;
    /*! \brief Total number of pages. */
    uint32_t at25d_pages;
    /*! \brief Number of bytes per page. */
    size_t at25d_psize;
    /*! \brief Type determination value. */
    uint_fast8_t at25d_srval;
};

extern NUTSPINODE at25df;
extern NUTDEVICE devDataFlash0;


int At25dfInit(NUTDEVICE* dev);
static uint8_t At25dStatus(NUTSPINODE * node);

#endif
