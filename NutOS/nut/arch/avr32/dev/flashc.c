/*!
 * Copyright (C) 2001-2010 by egnite Software GmbH
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
* \file arch/avr32/dev/flashc.c
* \brief AVR32 embedded flash controller support.
*
*/

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <avr32/io.h>
#include <arch/avr32.h>
#include <arch/avr32/flashc.h>

#include <sys/atom.h>
#include <sys/nutdebug.h>

#include <dev/nvmem.h>

#include <toolchain.h>

/*!
 * \addtogroup xgAvr32Efc
 */
/*@{*/

#ifndef FLASH_WRITE_WAIT
#define FLASH_WRITE_WAIT        60000
#endif

#ifndef FLASH_ERASE_WAIT
#define FLASH_ERASE_WAIT        60000
#endif

#ifndef FLASH_COMMAND_WAIT
#define FLASH_COMMAND_WAIT      6000
#endif

#ifndef FLASH_CHIP_ERASE_WAIT
#define FLASH_CHIP_ERASE_WAIT   600000
#endif

typedef uint32_t flashdat_t;
typedef unsigned long flashadr_t;
typedef volatile flashdat_t *flashptr_t;

#if !defined(__flash_nvram_size__)
#define __flash_nvram_size__ 4*1024
#endif

//! NVRAM data structure located in the flash array.
#if defined (__GNUC__)
__attribute__((__section__(".flash_nvram")))
#endif
static uint8_t flash_nvram_data[__flash_nvram_size__] NUT_LINKER_SECT(".flash_nvram");

#define WORKAROUND_FLASH_CONTROLER_BUG

RAMFUNC int flashc_wait_until_ready(uint32_t tmo)
{
    while (!(AVR32_FLASHC.fsr & AVR32_FLASHC_FSR_FRDY_MASK)) {
	    if (tmo && --tmo < 1) {
		    return -1;
	    }
    }
	
#if defined(WORKAROUND_FLASH_CONTROLER_BUG)
    // Make sure we wait at least tmo cycles.
	while( --tmo > 1 )
		_NOP();
#endif
	return 0;
}

/*!
 * \brief Execute flash controller command.
 *
 */
RAMFUNC int Avr32FlashcCmd(unsigned int cmd, int page, uint32_t tmo)
{
    int rc = 0;
	union {
		uint32_t            fcmd;
		avr32_flashc_fcmd_t	FCMD;
	} flashc_fcmd;
	

    /* Make sure that the previous command has finished. */
    flashc_wait_until_ready(tmo);

    /* IRQ handlers are located in flash. Disable them. */
    NutEnterCritical();

    /* Write command. */
	flashc_fcmd.fcmd = AVR32_FLASHC.fcmd;
	flashc_fcmd.FCMD.cmd = cmd;
	if (page >= 0) {
		flashc_fcmd.FCMD.pagen = page;
	}
	flashc_fcmd.FCMD.key = AVR32_FLASHC_FCMD_KEY_KEY;
	AVR32_FLASHC.fcmd = flashc_fcmd.fcmd;

    /* Wait for ready flag set. */
    flashc_wait_until_ready(tmo);

    /* Flash command finished. Re-enable IRQ handlers. */
    NutExitCritical();

    /* Check result. */
    if (AVR32_FLASHC.fsr & (AVR32_FLASHC_FSR_LOCKE_MASK | AVR32_FLASHC_FSR_PROGE_MASK)) {
        rc = -1;
    }
    return rc;
}

void Avr32FlashClearPageBuffer(void)
{
	Avr32FlashcCmd(AVR32_FLASHC_FCMD_CMD_CPB, -1, FLASH_COMMAND_WAIT);
}

void Avr32FlashEraseCurrentPage(void)
{
	Avr32FlashcCmd(AVR32_FLASHC_FCMD_CMD_EP, -1, FLASH_ERASE_WAIT);
}

void Avr32FlashWriteCurrentPage(void)
{
	Avr32FlashcCmd(AVR32_FLASHC_FCMD_CMD_WP, -1, FLASH_WRITE_WAIT);
}

int Avr32FlashFillPageBuffer( volatile uint8_t* dst, const uint8_t* src, size_t nbytes )
{
	union {
		uint64_t uint64;
		uint8_t  uint8[8];
	} flash_dword;
	uint8_t* flash_add;
	int consumedBytes = 0;
	int page_pos;
	int i;

	// Point to start of the requested address's page
	flash_add = (uint8_t*)((uint32_t)dst - ((uint32_t)dst % AVR32_FLASHC_PAGE_SIZE));
	
	Avr32FlashClearPageBuffer();

	for (page_pos = 0; page_pos < AVR32_FLASHC_PAGE_SIZE; page_pos += sizeof(uint64_t) ) {

		// Load original flash contents to page buffer
		flash_dword.uint64 = *((volatile uint64_t*)flash_add);

		// Update page buffer contents only if overlaps with [dst..dst+nbytes]
		for (i = 0; i < sizeof(uint64_t); ++i) {
			if (consumedBytes < nbytes && (flash_add == dst)) {
				// Update page with source contents
				flash_dword.uint8[i] = *src++;
				dst++;
				consumedBytes++;
			}
			flash_add++;
		}

		// Write flash page buffer in chunks of 64 bits as required by FLASHC
		(*(volatile uint64_t*)((uint32_t)flash_add - sizeof(uint64_t))) = flash_dword.uint64;
	}
	return consumedBytes;	
}

int Avr32FlashErasePageBuffer( volatile uint8_t* dst, size_t nbytes )
{
	union {
		uint64_t uint64;
		uint8_t  uint8[8];
	} flash_dword;
	uint8_t* flash_add;
	int consumedBytes = 0;
	int page_pos;
	int i;

	// Point to start of the requested address's page
	flash_add = (uint8_t*)((uint32_t)dst - ((uint32_t)dst % AVR32_FLASHC_PAGE_SIZE));

	Avr32FlashClearPageBuffer();

	for (page_pos = 0; page_pos < AVR32_FLASHC_PAGE_SIZE; page_pos += sizeof(uint64_t) ) {

		// Load original flash contents to page buffer
		flash_dword.uint64 = *((volatile uint64_t*)flash_add);

		// Update page buffer contents only if overlaps with [dst..dst+nbytes]
		for (i = 0; i < sizeof(uint64_t); ++i) {
			if (consumedBytes < nbytes && (flash_add == dst)) {
				// Update page with source contents
				flash_dword.uint8[i] = 0xFF;
				dst++;
				consumedBytes++;
			}
			flash_add++;
		}

		// Write flash page buffer in chunks of 64 bits as required by FLASHC
		(*(volatile uint64_t*)((uint32_t)flash_add - sizeof(uint64_t))) = flash_dword.uint64;
	}
	return consumedBytes;	
}


/*!
 * \brief Load configuration parameters from embedded flash memory.
 *
 * Applications should call NutNvMemLoad().
 *
 * \param pos   Start location within configuration sector.
 * \param data  Points to a buffer that receives the contents.
 * \param len   Number of bytes to read.
 *
 * \return 0 if successful.
 */
int Avr32FlashcParamRead(unsigned int pos, void *data, unsigned int len)
{
	if ( len > sizeof(flash_nvram_data) )
		return -1;

    memcpy(data, (void *) (uptr_t) (flash_nvram_data + pos), len);

    return 0;
}

/*!
 * \brief Store configuration parameters in embedded flash memory.
 *
 * Applications should call NutNvMemSave().
 *
 * \param pos   Start location within configuration sector.
 * \param data  Points to a buffer that contains the bytes to store.
 * \param len   Number of bytes to store.
 *
 * \return 0 on success or -1 in case of an error.
 */
int Avr32FlashcParamWrite(unsigned int pos, const void *data, unsigned int len)
{
	NUTASSERT( pos + len <= __flash_nvram_size__ );

	int byteCount = 0;
	
	while( byteCount < len )
	{
		byteCount += Avr32FlashFillPageBuffer( (void *)&flash_nvram_data + pos + byteCount, data + byteCount, len - byteCount );
		Avr32FlashEraseCurrentPage();
		Avr32FlashWriteCurrentPage();
	}

    return 0;
}

/*!
 * \brief Erase configuration parameters in embedded flash memory.
 *
 * Applications should call NutNvMemErase().
 *
 * \param pos   Start location within configuration sector.
 * \param len   Number of bytes to erase. 
 *
 * \return 0 on success or -1 in case of an error.
 */
int Avr32FlashcParamErase(unsigned int pos, int len)
{
	NUTASSERT( pos + len <= __flash_nvram_size__ );

	int byteCount = 0;
	int currentPageByteCount;

	if ( len == -1 || len > __flash_nvram_size__ - pos)
		len = __flash_nvram_size__ - pos;

	while( byteCount < len )
	{
		currentPageByteCount = Avr32FlashErasePageBuffer( (void *)&flash_nvram_data + pos + byteCount, len - byteCount );
		Avr32FlashEraseCurrentPage();
		if ( currentPageByteCount != AVR32_FLASHC_PAGE_SIZE )
			Avr32FlashWriteCurrentPage();

		byteCount += currentPageByteCount;
	}

    return 0;
}