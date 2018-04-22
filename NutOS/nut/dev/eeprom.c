/*
 * Copyright (C) 2009 by Rittal GmbH & Co. KG. All rights reserved.
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
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
 * THIS SOFTWARE IS PROVIDED BY EMBEDDED IT AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL EMBEDDED IT
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 *
 */

/*!
 * \verbatim
 * $Id$
 *\endverbatim
 */

#ifndef _DEV_EEPROM_H_
#define _DEV_EEPROM_H_

#include <cfg/os.h>
#include <cfg/eeprom.h>

#include <sys/timer.h>
#include <sys/event.h>
#include <stdint.h>
#include <dev/twif.h>
#include <dev/at24c.h>
#include <dev/eeprom.h>

static struct at24c at24c32s;

/****************************************************************************/
int EEInit( void )
/****************************************************************************/
{
    uint8_t dummy;
    at24c32s.PageSize = AT24C_ROW_SIZE;
    at24c32s.EepromSize = AT24C_CHIP_SIZE;
    at24c32s.SlaveAddress = NUT_CONFIG_AT24_ADR;
    at24c32s.IAddrW = AT24C_ADR_SIZE;
#ifdef AT24C_BLOCK_ADDR
    at24c32s.BlInSla = 1;
#endif
    at24c32s.Timeout = 20;

    NutEventPost( &at24c32s.ee_mutex);

    if( TwInit(0))
        return -1;
    /* Do a dummy read for communication test */
    return At24cRead( &at24c32s, &dummy, 1, 0);
}

/****************************************************************************/
int EEWriteData( uint16_t addr, const void *data, uint16_t len )
/****************************************************************************/
{
    return At24cWrite( &at24c32s, (uint8_t *)data, len, addr );
}

/****************************************************************************/
int EEReadData( uint16_t addr, void *data, uint16_t len )
/****************************************************************************/
{
    return At24cRead( &at24c32s, (uint8_t *)data, len, addr );
}
#endif /* _DEV_EEPROM_H_ */
