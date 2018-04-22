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
 *
 */

/*!
 * \file arch/avr/dev/eeprom.c
 * \brief AVR On-Chip EEPROM support.
 *
 * \verbatim
 * $Id: eeprom.c 5954 2014-12-16 12:57:32Z thiagocorrea $
 * \endverbatim
 */

#if defined(__IMAGECRAFT__)
#include <eeprom.h>
#elif defined(__GNUC__)
#include <avr/eeprom.h>
#endif

#include <dev/nvmem.h>

#include <stdint.h>

/*!
 * \addtogroup xgArchAvrDevEeprom
 */
/*@{*/

/*!
 * \brief Load data from AVR EEPROM.
 *
 * \return Always 0.
 */
int OnChipNvMemLoad(unsigned int addr, void *buff, size_t siz)
{
#if defined(__IMAGECRAFT__)
    EEPROMReadBytes((int)addr, buff, siz);
#elif defined(__GNUC__)
    eeprom_read_block (buff, (void *)addr, siz);
#endif
    return 0;
}

/*!
 * \brief Save data in AVR EEPROM.
 *
 * \return Always 0.
 */
int OnChipNvMemSave(unsigned int addr, const void *buff, size_t len)
{
#if defined(__IMAGECRAFT__)
    uint8_t *cp;
    size_t i;

    for (cp = (uint8_t *) buff, i = 0; i < len; cp++, i++) {
        if (EEPROMread((int) (addr + i)) != *cp) {
            EEPROMwrite((int) (addr + i), *cp);
        }
		
    }
#elif defined(__GNUC__)
	eeprom_busy_wait();
	eeprom_update_block(buff, (void *)addr, len);
#endif
	return 0;
}


/*!
 * \brief Erase data in AVR EEPROM.
 *
 * \return Always 0.
 */
int OnChipNvMemErase(unsigned int addr, size_t len)
{
	size_t i;

	if ( len == -1 || len > E2END - addr )
		len = E2END - addr;

	for (i = 0; i < len; ++i) {
#if defined(__IMAGECRAFT__)
		if (EEPROMread((int) (addr + i)) != 0xFF) {
			EEPROMwrite((int) (addr + i), 0xFF);
		}

#elif defined(__GNUC__)
		eeprom_busy_wait();
		eeprom_update_byte((void *)addr + i, 0xFF);
#endif
	}

	return 0;
}
/*@}*/
