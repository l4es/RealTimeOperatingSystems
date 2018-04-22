/*
 * Copyright (C) 2013-2017 Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
#include <arch/cm3.h>
#include <dev/iap_flash.h>

#if !defined(MCU_STM32L0) && !defined(MCU_STM32L1)
#warning "STM32 family has no STM32L0 or STM32L1 compatible FLASH/EEPROM"
#endif

#if !defined(FLASH_SR_OPTVERRUSR)
#define FLASH_SR_OPTVERRUSR 0
#endif

#define FLASH_SECTOR_SIZE  (1<<12)
#define FLASH_SECTOR_MASK  0xfffff000
#define FLASH_SECTOR_SHIFT 12
#define ERASED_PATTERN_32  0

uint32_t program_end_raw;

#if defined(MCU_STM32L1)
/*Sectors are the unit for write protection, pages for erase */
# define FLASH_PAGE_SIZE    (1<<8)
# define FLASH_PAGE_MASK    0xffffff00
# define FLASH_PAGE_SHIFT   8

# if defined (MCU_STM32L1_CAT3)
static uint32_t pagelist[32]; /* 256 k*/
# elif defined (MCU_STM32L1_CAT4) ||  defined (MCU_STM32L1_CAT6) /* 384k*/
static uint32_t pagelist[48]; /* 384k*/
# elif defined (MCU_STM32L1_CAT5)
static uint32_t pagelist[64]; /* 512 k*/
# else
static uint32_t pagelist[16]; /* Up to 128 k*/
# endif
#elif defined(MCU_STM32L0)
/*Sectors are the unit for write protection, pages for erase */
# define FLASH_PAGE_SIZE    (1<<7)
# define FLASH_PAGE_MASK    0xffffff80
# define FLASH_PAGE_SHIFT   7

# if   defined (MCU_STM32L0_CAT1) /*  16 k*/
static uint32_t pagelist[ 4];
# elif defined (MCU_STM32L0_CAT2) /*  32 k*/
static uint32_t pagelist[ 8];
# elif defined (MCU_STM32L0_CAT3) /*  64 k*/
static uint32_t pagelist[16];
# elif defined (MCU_STM32L0_CAT5) /* 192 k*/
static uint32_t pagelist[48];
# endif
#else
#warning Unhandled family
#endif

#if defined(FLASH_WRPR_WRP) && !defined( FLASH_WRPR1_WRP)
# define FLASH_WRPR (FLASH->WRPR)
#else
# define  FLASH_WRPR (FLASH->WRPR1)
#endif

#define FLASH_PEKEY1 0x89abcdef
#define FLASH_PEKEY2 0x02030405

#define FLASH_PRGKEY1 0x8c9daebf
#define FLASH_PRGKEY2 0x13141516

#define FLASH_OPTKEY1 0xfbead9c8
#define FLASH_OPTKEY2 0x24252627

void FlashUntouch(void)
{
    memset(pagelist, 0, sizeof(pagelist));
}

size_t IapFlashEnd(void)
{
    uint16_t size;
    size = *(__I uint16_t *) FLASHSIZE_BASE;
    uint32_t retval = FLASH_BASE - 1;
#if defined(MCU_STM32L1)
    uint32_t dev_id;
    dev_id = *(volatile uint32_t *)0xe0042000 & 0xfff;
    switch (dev_id) {
    case 0x436:
        /* RM0038 Rev.12 p 883:
         *  "For DEV_ID = 0x436, the field value can be `0' or `1',
         *  with `0' for 384 Kbytes and `1' for 256 Kbytes."
         */
        if (size == 0)
            retval += 384 * 1024;
        else
            retval += 256 * 1024;
        break;
    case 0x429:
        /* RM0038 Rev.12 p 883:
         * "For DEV_ID = 0x429, only LSB part of F_SIZE: F_SIZE[7:0] is valid."
        */
        size  &= 0xff;
    }
#endif
    retval +=  size * 1024;
    return retval;
}

/*!
 * \brief Get start address in first page above program storage.
 *
 * \param NONE
 * \return Start address in first page above program storage.
 */
extern size_t __end_rom;
size_t IapProgramEnd(void)
{
    size_t program_end = (size_t)&__end_rom;
    program_end +=  FLASH_PAGE_SIZE - 1;
    program_end &=  FLASH_PAGE_MASK;
    return program_end;
}

/*!
 * \brief Return pagesize of given address.
 *
 * \param addr Adress
 * \return Size of current page.
 */
size_t IapPageSize(size_t addr)
{
    return FLASH_PAGE_SIZE;
}

/*!
  * \brief  Unlocks the FLASH Program Erase Controller.
  * \retval 0 on success, FLASH_LOCKED else.
  */
static FLASH_Status FLASH_Unlock( void )
{

    FLASH->PEKEYR = FLASH_PEKEY1;
    FLASH->PEKEYR = FLASH_PEKEY2;
    if (FLASH->PECR & FLASH_PECR_PELOCK)
        return FLASH_LOCKED;
    FLASH->PRGKEYR = FLASH_PRGKEY1;
    FLASH->PRGKEYR = FLASH_PRGKEY2;
    return (FLASH->PECR & FLASH_PECR_PRGLOCK)?FLASH_LOCKED:FLASH_COMPLETE;
}

/*!
  * \brief  Waits for a Flash operation to complete or a TIMEOUT to occur.
  * \param  Timeout: FLASH progamming Timeout in Microseconds
  *
  * \retval FLASH Status: FLASH_COMPLETE or appropriate error.
  */
static FLASH_Status FLASH_GetStatus(void)
{
    FLASH_Status rs = FLASH_COMPLETE;

    /* Decode the Flash Status
     * Check BSY last, so maybe it has completed meanwhile*/
    if (FLASH->SR & (FLASH_SR_SIZERR |FLASH_SR_PGAERR))
        rs = FLASH_ERROR_PG;
    else if (FLASH->SR & FLASH_SR_WRPERR)
        rs = FLASH_ERROR_WRP;
    else if (FLASH->SR & FLASH_SR_BSY)
        rs = FLASH_BUSY;

    /* Return the Flash Status */
    return rs;
}

/*!
  * \brief  Waits for a Flash operation to complete
  *
  * \retval FLASH Status: FLASH_COMPLETE or appropriate error.
  * Every flash access stalls while FLASH erase/program is running
  * The erase/program process however will finish at some point
  * and may indicate failure then
  */
static FLASH_Status FlashWaitReady(void)
{
    FLASH_Status status;
    do
        status = FLASH_GetStatus();
    while(status == FLASH_BUSY);

    /* Return the operation status */
    return status;
}

/*!
 * \brief Erase specified FLASH sector. Use "Program memory page erase"
 *
 * \param sector Sector to erase.
 *
 * \return FLASH Status: FLASH_COMPLETE or appropriate error.
 */
static FLASH_Status FlashErasePage(void *address_in_page)
{
    FLASH_Status rs = FLASH_COMPLETE;
    uint32_t first_address_in_page = (uint32_t)address_in_page & FLASH_PAGE_MASK;
    uint32_t *page = (uint32_t*)first_address_in_page;
    uint32_t current_page = ((uint32_t)address_in_page - FLASH_BASE) >> FLASH_PAGE_SHIFT;
    int i;

    /* Check, if page really needs erase */
    for(i = 0; i < FLASH_PAGE_SIZE >> 2; i++) {
        if (page[i] != ERASED_PATTERN_32)
            break;
    }
    if ( i >= FLASH_PAGE_SIZE >> 2)
        rs = FLASH_COMPLETE;
    else {
        /* Wait for last operation to be completed */
        rs = FlashWaitReady();
        if(rs == FLASH_COMPLETE) {
            FLASH->PECR = FLASH_PECR_ERASE;
            FLASH->PECR = FLASH_PECR_ERASE | FLASH_PECR_PROG;
            rs = FlashWaitReady();
            if (rs != FLASH_COMPLETE)
                return rs;
            *page = 0;
            rs = FlashWaitReady();
            FLASH->PECR = 0;
        }
    }
    if (rs == FLASH_COMPLETE )
        pagelist[current_page / 32] &= ~(1 <<(current_page % 32));
    /* Return the Erase Status */
    return rs;
}

/*!
 * \brief Program any data to FLASH.
 *
 * It always erases the page, eventually keeping old content
 *
 * \param dst Pointer to address anywhere in FLASH.
 * \param src Pointer to source data. With SRC == NULL, the region
 *        is checked for write protection
 * \param len Number of bytes to be written/checked.
 * \param mode Erase mode (Always, on first access to block, never).
 *
 * \return FLASH Status: FLASH_COMPLETE or appropriate error.
 */
FLASH_Status IapFlashWrite( void* dst, const void* src, size_t len,
                                FLASH_ERASE_MODE mode)
{
    FLASH_Status rs = FLASH_COMPLETE;
    int sector_start, sector_end;
    int i;
    void *wptr = dst;
    const void *rptr = src;
    uint32_t length = len;

    if (len == 0)
        return FLASH_COMPLETE;

    /* Check top boundary */
    if ((((uint32_t)dst - 1 + len) > IapFlashEnd()) ||
        ((uint32_t)dst < FLASH_BASE))
    {
        return FLASH_BOUNDARY;
    }

    if ((rs = FLASH_Unlock()) != FLASH_COMPLETE)
    {
        /* Unlocking failed for any reason */
        return FLASH_LOCKED;
    }

    /* Check for write protected sectors */
    sector_start = ((uint32_t)dst - FLASH_BASE) >> FLASH_SECTOR_SHIFT;
    sector_end = ((uint32_t)dst + len - FLASH_BASE) >> FLASH_SECTOR_SHIFT;
    rs = FLASH_ERROR_WRP;
    for (i = sector_start; (i < sector_end) && (i < 32); i++) {
        if (FLASH_WRPR & (1 << i)) {
            goto done;
        }
    }
#if defined (FLASH_WRPR2_WR)
    uint32_t * wrpr2 = (uint32_t *) FLASH->WRPR2;
    for (i = 0; i < (sector_end - 32); i++) {
        if (wrpr2[i / 32] & (1 << ( i % 32))) {
            goto done;
        }
    }
#endif
    rs = FLASH_COMPLETE;
     if (src == NULL) {
         /* Only a check for write protection was requested */
         goto done;
     }

    while( (length) && (rs==FLASH_COMPLETE))
    {
        uint32_t current_page = ((uint32_t)wptr - FLASH_BASE) >> FLASH_PAGE_SHIFT;
        uint32_t current_length = length;
        uint32_t offset_in_page = (uint32_t)wptr & ~FLASH_PAGE_MASK;
        uint8_t  page_buffer[FLASH_PAGE_SIZE];

        if (offset_in_page + current_length > FLASH_PAGE_SIZE)
            current_length = FLASH_PAGE_SIZE - offset_in_page;

        if ((offset_in_page == 0) && (current_length == FLASH_PAGE_SIZE)) {
            /* Handle full page writes */
            volatile uint32_t *cwptr = (uint32_t*) wptr;
            volatile uint32_t *crptr = (uint32_t*) rptr;

            /* Check if content really changed */
            /* FIXME: Check if write only sets bits.
               At least on STM32L0, setting more bits w/o resetting any bits
               will succeed, but with NOTZEROERR set*/
            rs = memcmp(page_buffer, (void*)((uint32_t)wptr & FLASH_PAGE_MASK), FLASH_PAGE_SIZE);
            if (rs == 0)
                continue;

            rs = FlashErasePage(wptr);
            if (rs != FLASH_COMPLETE)
                goto done;

            for (i = 0; i < FLASH_PAGE_SIZE >> 2; i++)
                cwptr[i] = crptr[i];
            rs = FlashWaitReady();
            if(rs != FLASH_COMPLETE)
                goto done;
            wptr += current_length;
            rptr += current_length;
            length -= current_length;
            continue;
        }
        else { /* Handle partial page writes */
            uint8_t page_buffer[FLASH_PAGE_SIZE];
            int do_page_erase;
            int page_status;

            page_status = pagelist[current_page / 32] & (1 <<(current_page % 32));
            do_page_erase = ((mode == FLASH_ERASE_ALWAYS) ||
                             ((mode == FLASH_ERASE_FIRST_TOUCH) &&
                              (0 == page_status)));
            if (do_page_erase)
                memset(page_buffer, (uint8_t) ERASED_PATTERN_32, FLASH_PAGE_SIZE);
            else {
                rs = memcmp(rptr, wptr, current_length);
                if (rs == 0)
                    goto chunk_done;

                if ((offset_in_page & 3 ) || (current_length & 3)) {
                    memcpy(page_buffer,
                           (void*)((uint32_t)wptr & FLASH_PAGE_MASK),
                           FLASH_PAGE_SIZE);
                }
                else
                {
                    volatile uint32_t *cwptr = (uint32_t*)wptr;
                    volatile uint32_t *crptr = (uint32_t*)rptr;
                    int n_words = current_length >> 2;

                    for (i = 0; i < n_words; i++) {
                        /* We can only write dwords with all bit flashed
                         * or the flash erased!*/
                        if ((crptr[i] != ~ERASED_PATTERN_32) ||
                            (cwptr[i] != ERASED_PATTERN_32))
                            break;
                    }
                    if ( i < n_words) {
                        /* Copy old content for pattern  reason */
                        memcpy(page_buffer,
                               (void*)((uint32_t)wptr & FLASH_PAGE_MASK),
                               FLASH_PAGE_SIZE);
                    }
                    else {
                        for (i = 0; i < n_words; i++)
                            if (cwptr[i] != crptr[i])
                                cwptr[i] = crptr[i];
                        rs = FlashWaitReady();
                    if(rs != FLASH_COMPLETE)
                        goto done;
                    goto chunk_done;
                    }
                }
            }
            /* FIXME: Check if write only sets bits.
               At least on STM32L0, setting more bits w/o resetting any bits
               will succeed, but with NOTZEROERR set*/
            memcpy(page_buffer + offset_in_page, rptr, current_length);

            rs = FlashErasePage(wptr);
            if (rs != FLASH_COMPLETE)
                goto done;

            /* Program the sector */
            rs = FlashWaitReady();
            if (rs != FLASH_COMPLETE)
                goto done;
            else
            {
                int i;
                volatile uint32_t *cwptr =
                    (uint32_t*) ((uint32_t)wptr & FLASH_PAGE_MASK);
                volatile uint32_t *crptr = (uint32_t*) page_buffer;

                for (i=0; i < (FLASH_PAGE_SIZE >> 2); i++)
                    if (cwptr[i] != crptr[i])
                        cwptr[i] = crptr[i];
                rs = FlashWaitReady();
                if(rs != FLASH_COMPLETE)
                    goto done;
            }
        chunk_done:
            wptr += current_length;
            rptr += current_length;
            length -= current_length;
        }
    }
    /* Check the written data */
    wptr = dst;
    rptr = src;
    length = len;
    /* Align flash access to 4 Byte Boundary*/
    while (length && ((uint32_t)wptr & FLASH_PAGE_MASK)) {
        if(*(volatile uint8_t*)wptr++ != *(uint8_t*)rptr++) {
            rs = FLASH_COMPARE;
            goto done;
        }
        length--;
    }
    /* Now compare 32-bit  at a time*/
    while (length > 3) {
        if(*(volatile uint32_t*)wptr != *(uint32_t*)rptr) {
            rs = FLASH_COMPARE;
            goto done;
        }
        length -= 4;
        wptr += +4;
        rptr += +4;
    }

    while (length) {
        if((*(volatile uint8_t*)wptr++) != *(uint8_t*)rptr++) {
            rs = FLASH_COMPARE;
            goto done;
        }
        length--;
    }

    /* Lock the FLASH again */
done:
    FLASH->PECR = FLASH_PECR_PELOCK;
    return rs;
}

/*!
 * \brief Try to protect/unprotect the requested flash region.
 *
 * \param dst Pointer to address anywhere in FLASH.
 * \param len Length of region in bytes.
 * \param ena 0 disables write protection anything else write-protects.
 *
 * \return FLASH_Status: FLASH_COMPLETE or appropriate error.
 */
FLASH_Status IapFlashWriteProtect(void *dst, size_t len, int ena)
{
    int sector_start, sector_end;
    int i;
    uint32_t optcr;
    FLASH_Status rs = FLASH_COMPLETE;
    uint32_t flash_end_addr = IapFlashEnd();
    uint8_t nwrpr;
    uint32_t cwrpr;
    uint16_t *wrpr16;
    volatile uint32_t *ob_wpr;

 /* Array of 32-bit old write protection state, later accessed as 16 bit */
#if defined (FLASH_WRPR4_WR)
    uint32_t wrpr[4] = {(FLASH_WRPR), FLASH->WRPR2, FLASH->WRPR3, FLASH->WRPR4};
#elif defined (FLASH_WRPR3_WR)
    uint32_t wrpr[3] = {(FLASH_WRPR), FLASH->WRPR2, FLASH->WRPR3};
#elif defined (FLASH_WRPR3_WR)
    uint32_t wrpr[2] = {(FLASH_WRPR), FLASH->WRPR2};
#else
    uint32_t wrpr[1] = {(FLASH_WRPR)};
#endif
    wrpr16 = (uint16_t*) wrpr;
    ob_wpr = &OB->WRP01;

    /* Check boundaries */
    if ((((uint32_t)dst + len) > flash_end_addr) || ((uint32_t)dst < FLASH_BASE))
    {
        return FLASH_BOUNDARY;
    }

    if (len == 0)
        return FLASH_COMPLETE;

    /* Wait for last operation to be completed */
    rs = FlashWaitReady();
    if(rs != FLASH_COMPLETE)
        return rs;
    FLASH->OPTKEYR = FLASH_OPTKEY1;
    FLASH->OPTKEYR = FLASH_OPTKEY2;
    optcr = FLASH->PECR;
    if (optcr & FLASH_PECR_OPTLOCK)
        return FLASH_ERROR_PG;
    sector_start = (uint32_t) dst & FLASH_PAGE_MASK;
    sector_end = ((uint32_t) dst +len -1) & FLASH_PAGE_MASK;
    for (i = sector_start; (i <= sector_end) && ( i < 32); i++) {
        if (ena) {
            FLASH_WRPR |=  (1 << i);
        } else {
            FLASH_WRPR &= ~(1 << i);
        }
    }
#if defined (FLASH_WRPR2_WR)
    uint32_t * wrpr2 = (uint32_t *) FLASH->WRPR2;
    for (i = 0; i < (sector_end - 32); i++) {
        if (ena) {
            wrpr2[i / 32] |=  (1 <<(i % 32));
        } else {
            wrpr2[i / 32] &= ~(1 <<(i % 32));
        }
    }
#endif
    FLASH->PEKEYR = FLASH_PEKEY1;
    FLASH->PEKEYR = FLASH_PEKEY2;
    if (FLASH->PECR & FLASH_PECR_PELOCK)
        return FLASH_LOCKED;
    FLASH->OPTKEYR = FLASH_OPTKEY1;
    FLASH->OPTKEYR = FLASH_OPTKEY2;
    if (FLASH->PECR & FLASH_PECR_OPTLOCK) {
        FLASH->PECR |= FLASH_PECR_PELOCK;
        return FLASH_ERROR_PG;
    }
    i = sector_start >> 4;
    nwrpr = wrpr16[i] >> 8;
    cwrpr = ~nwrpr << 8  | nwrpr;
    nwrpr = wrpr16[i] ;
    cwrpr = cwrpr << 16 | ~nwrpr << 8  | nwrpr;
    ob_wpr[i] = cwrpr;
    /* pm0062 tells to clear option byte errors */
    FLASH->SR = FLASH_SR_OPTVERRUSR | FLASH_SR_OPTVERR;

    /* Wait for last operation to be completed */
    rs = FlashWaitReady();
    FLASH->PECR |= FLASH_PECR_PELOCK;
    return rs;
}
