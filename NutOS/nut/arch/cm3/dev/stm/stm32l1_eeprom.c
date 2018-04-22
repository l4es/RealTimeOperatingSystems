/*
 * Copyright (C) 2013, 2015-2017 Uwe Bonnes
 *                           (bon@elektron.ikp.physik.tu-darmstadt.de)
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

#include <stdint.h>
#include <string.h>

#include <cfg/arch.h>
#include <cfg/memory.h>
#include <cfg/eeprom.h>
#include <sys/nutdebug.h>
#include <dev/iap_flash.h>

#if !defined(MCU_STM32L0) && !defined(MCU_STM32L1)
#warning "STM32 family has no L1 compatible FLASH/EEPROM"
#endif
#include <arch/cm3/stm/stm32xxxx.h>

#define ERASED_PATTERN_32  0

#define FLASH_PEKEY1 0x89abcdef
#define FLASH_PEKEY2 0x02030405

#if defined(FLASH_EEPROM_END)
#  define STM32L1_EEPROM_SIZE (FLASH_EEPROM_END - FLASH_EEPROM_BASE)
#elif defined(DATA_EEPROM_END)
#  define STM32L1_EEPROM_SIZE (DATA_EEPROM_END - DATA_EEPROM_BASE)
#elif defined(DATA_EEPROM_BANK2_END)
#  define STM32L1_EEPROM_SIZE (DATA_EEPROM_BANK2_END - DATA_EEPROM_BASE)
#else
# warning Unhandled STM32 family
#endif
#define STM32L1_EEPROM_BASE 0x08080000

/*!
 * \brief       Read data from EEPROM at specific address
 *
 * \param       address EEPROM address that start to write data, it must be
 *              in range 0..STM32L1_EEPROM_SIZE
 * \param       buff    buffer to place the read data in
 * \param       size    number of bytes to be read
 *
 * \return      none
 */

int Stm32l1_EepromRead(uint16_t addr, void* buff, size_t size)
{
    uint32_t ee_addr;

    if ((uint32_t)addr + size > STM32L1_EEPROM_SIZE )
        return FLASH_BOUNDARY;

    ee_addr = STM32L1_EEPROM_BASE + addr;

    while((ee_addr & 3) && size) {
        *(uint8_t*)buff = *(uint8_t*)ee_addr;
        buff++;
        ee_addr++;
        size--;
    }
    while(size > 3) {
        *(uint32_t*)buff = *(uint32_t*)ee_addr;
        buff += 4;
        ee_addr += 4;
        size -= 4;
    }
    while(size) {
        *(uint8_t*)buff = *(uint8_t*)ee_addr;
        buff++;
        ee_addr++;
        size--;
    }
    return FLASH_COMPLETE;
}

/*!
 * \brief       Write data to EEPROM at specific address
 *
 * \param       address EEPROM address that start to write data, it must be
 *              in range 0..STM32L1_EEPROM_SIZE
 * \param       buff    buffer that contain data that will be written to buffer
 * \param       size    number of bytes to be written
 *
 * \return      FLASH_STATUS
 */

int Stm32l1_EepromWrite(uint16_t addr, const void* buff, size_t size)
{
    uint32_t ee_addr;
    int rs;

    if (size == 0)
        return FLASH_COMPLETE;

    if ((uint32_t)addr + size > STM32L1_EEPROM_SIZE )
        return FLASH_BOUNDARY;

    ee_addr = STM32L1_EEPROM_BASE + addr;

    if (FLASH->PECR & FLASH_PECR_PELOCK) {
        FLASH->PEKEYR = FLASH_PEKEY1;
        FLASH->PEKEYR = FLASH_PEKEY2;
        if (FLASH->PECR & FLASH_PECR_PELOCK)
            return FLASH_ERROR_PG;
    }
    if (ee_addr & 1) {
        uint8_t data = *(uint8_t *)buff;
        if (data != *(uint8_t *)ee_addr) {
            if (*(uint8_t *)ee_addr != (uint8_t)ERASED_PATTERN_32)
                FLASH->PECR =  FLASH_PECR_FIX;
            else
                FLASH->PECR = 0;
            *(uint8_t *)ee_addr = data;
            }
        buff    ++;
        ee_addr ++;
        size --;
    }
    if ((ee_addr  & 2)  && (size > 1)) {
        uint16_t data = *(uint16_t *)buff;
        if (data != *(uint16_t *)ee_addr) {
            if (*(uint16_t *)ee_addr != (uint16_t)ERASED_PATTERN_32)
                FLASH->PECR =  FLASH_PECR_FIX;
            else
                FLASH->PECR = 0;
            *(uint16_t *)ee_addr = data;
            }
        buff    += 2;
        ee_addr += 2;
        size -= 2;
    }
    while (size > 3) {
        uint32_t data = *(uint32_t *)buff;
        if (data != *(uint32_t *)ee_addr) {
            if (*(uint32_t *)ee_addr != ERASED_PATTERN_32)
                FLASH->PECR =  FLASH_PECR_FIX;
            else
                FLASH->PECR = 0;
            *(uint32_t *)ee_addr = data;
        }
        buff    += 4;
        ee_addr += 4;
        size -= 4;
    }
    if (size > 1) {
        uint16_t data = *(uint16_t *)buff;
        if (data != *(uint16_t *)ee_addr) {
            if (*(uint16_t *)ee_addr != (uint16_t)ERASED_PATTERN_32)
                FLASH->PECR =  FLASH_PECR_FIX;
            else
                FLASH->PECR = 0;
            *(uint16_t *)ee_addr = data;
            }
        buff    += 2;
        ee_addr += 2;
        size -= 2;
    }
    if (size) {
        uint8_t data = *(uint8_t *)buff;
        if (data != *(uint8_t *)ee_addr) {
            if (*(uint8_t *)ee_addr != (uint8_t)ERASED_PATTERN_32)
                FLASH->PECR =  FLASH_PECR_FIX;
            else
                FLASH->PECR = 0;
            *(uint8_t *)ee_addr = data;
            }
    }
    rs = FLASH_COMPLETE;
    FLASH->PECR = FLASH_PECR_PELOCK;
    return rs;
}
