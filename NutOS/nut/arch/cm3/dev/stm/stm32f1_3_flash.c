/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Rittal GmbH & Co. KG. All rights reserved.
 * Copyright (C) 2012, 2013, 2017 Uwe Bonnes
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
#include <sys/heap.h>
#include <dev/iap_flash.h>

uint32_t program_end_raw;

#if defined (MCU_STM32F0)
# if  defined(STM32F030xC) ||  defined(STM32F070xB) || \
    defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx) || \
    defined(STM32F091xC) || defined(STM32F098xx)
#  define FLASH_PAGE_SIZE (1 << 11)
# else
#  define FLASH_PAGE_SIZE 1024
# endif
# if  defined(STM32F030xC) || defined(STM32F091xC)
static uint32_t pagelist[4];
# else
/* Lets waste one word here. So no need to break down further*/
static uint32_t pagelist[2];
# endif
#elif defined(MCU_STM32F1)
# if defined (MCU_STM32F1_LD) || defined (MCU_STM32F1_LD_VL)
#  define FLASH_PAGE_SIZE 1024
   static uint32_t pagelist[1];
# elif defined (MCU_STM32F1_MD) || defined(MCU_STM32F1_MD_VL)
#  define FLASH_PAGE_SIZE 1024
   static uint32_t pagelist[4];
# elif defined (MCU_STM32F1_HD) || defined (MCU_STM32F1_HD_VL) ||\
    defined (MCU_STM32F1_CL)
#  define FLASH_PAGE_SIZE 2048
   static uint32_t pagelist[8];
# elif defined(MCU_STM32F1_XL)
#  define FLASH_PAGE_SIZE 2048
   static uint32_t pagelist[16];
# endif
#elif defined(MCU_STM32F3)
# define FLASH_PAGE_SIZE 2048
  static uint32_t pagelist[4];
#else
# warning Unknown STM32 Type
#endif

#define ERASED_PATTERN_16 0xffff
#define FLASH_ACCESS_SIZE 2

#ifndef FLASH_CONF_SIZE
#define FLASH_CONF_SIZE         FLASH_PAGE_SIZE
#elif  ((FLASH_CONF_SIZE != 256) && (FLASH_CONF_SIZE != 512) && \
        (FLASH_CONF_SIZE != 1024) && (FLASH_CONF_SIZE != 2048) && \
        (FLASH_CONF_SIZE != 4096) && (FLASH_CONF_SIZE != 8192))
#error FLASH_CONF_SIZE has to be either FLASH_PAGE_SIZE (default), 256, 512, 1024, 2048, 4096 or 8192
#endif

#define FLASH_PAGE_MASK (~(FLASH_PAGE_SIZE - 1))

#if !defined(RDP_KEY)
#define RDP_KEY 0x00aa;
#endif

#if !defined(FLASH_KEY1)
#define FLASH_KEY1 0x45670123L
#endif
#if !defined(FLASH_KEY2)
#define FLASH_KEY2 0xCDEF89ABL
#endif
#if !defined(FLASH_OPTKEY1)
#define FLASH_OPTKEY1 0x08192A3B
#endif
#if !defined(FLASH_OPTKEY2)
#define FLASH_OPTKEY2 0x4C5D6E7F
#endif

#if !defined(FLASH_SR_WRPRTERR) && defined(FLASH_SR_WRPERR)
#define FLASH_SR_WRPRTERR FLASH_SR_WRPERR
#endif

void FlashUntouch(void)
{
    int i;
    for (i = 0; i< sizeof(pagelist)/4; i++)
        pagelist[i] = 0;
}

static size_t FlashEnd(void)
{
    uint16_t size;
    size = *(uint16_t *) FLASHSIZE_BASE;
    return FLASH_BASE - 1 + size * 1024;
}

/*!
  * \brief  Returns the FLASH Status.
  *
  * \retval FLASH Status: FLASH_COMPLETE or appropriate error.
  */
static FLASH_Status FLASH_GetStatus(void)
{
    FLASH_Status rs = FLASH_COMPLETE;

    /* Decode the Flash Status */
    if (FLASH->SR & FLASH_SR_WRPRTERR)
        rs = FLASH_ERROR_WRP;
    else if (FLASH->SR & FLASH_SR_PGERR) {
        rs = FLASH_ERROR_PG;
        FLASH->CR = 0; /* Reset STRT */
    }
    else if (FLASH->SR & FLASH_SR_BSY)
        rs = FLASH_BUSY;
    FLASH->SR =  FLASH_SR_EOP | FLASH_SR_WRPRTERR | FLASH_SR_PGERR;

#if defined(MCU_STM32F1_XL)
    /* Decode the Flash Status */
    if (FLASH->SR2 & FLASH_SR_WRPRTERR)
        rs |= FLASH_ERROR_WRP;
    else if (FLASH->SR2 & FLASH_SR_PGERR)
        rs |= FLASH_ERROR_PG;
    else if (FLASH->SR2 & FLASH_SR_BSY)
        rs |= FLASH_BUSY;
#endif /* MCU_STM32F1_XL */

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
 * \brief Erase FLASH Page at specified address.
 *
 * This routine is called by Stm32FlashWritePage to erase
 * before programming.
 *
 * \param page Page to erase.
 *
 * \return FLASH Status.
 *
 * Even for page erase, AR takes the memory address while
 * rm0316 ands pm0075 tell about the page as argument
 */
static FLASH_Status FlashErasePage(uint32_t mem)
{
    FLASH_Status rs = FLASH_COMPLETE;

    uint32_t current_page = ((uint32_t)mem - FLASH_BASE)/FLASH_PAGE_SIZE;
    uint32_t pagestart, pageend, *pagemem;

    /* Check if page is erased */
    pagestart = mem & FLASH_PAGE_MASK;
    pageend   =  pagestart + FLASH_PAGE_SIZE;
    for(pagemem = (uint32_t*) pagestart; (uint32_t)pagemem < pageend; pagemem++) {
        if (*pagemem != ((ERASED_PATTERN_16 << 16) | ERASED_PATTERN_16))
            break;
        }
    if ((uint32_t) pagemem >= pageend)
        goto erase_done;
    /* Wait for last operation to be completed */
    rs = FlashWaitReady();
    if(rs == FLASH_COMPLETE) {
#if defined(MCU_STM32F1_XL)
        if ((current_page / FLASH_PAGE_SIZE) >255) {
            FLASH->CR2 = FLASH_CR_PER;
            FLASH->AR2 = mem;
            FLASH->CR2 = FLASH_CR_PER | FLASH_CR_STRT;
            rs = FlashWaitReady();
            FLASH->CR2 = 0;
            goto erase_done;
        }
#endif

        /* if the previous operation is completed, proceed to erase the page */
        FLASH->CR = FLASH_CR_PER;
        FLASH->AR = mem;
        FLASH->CR = FLASH_CR_PER | FLASH_CR_STRT;
        rs = FlashWaitReady();
        FLASH->CR = 0;
    }
erase_done:
    if (rs != FLASH_COMPLETE)
        pagelist[current_page/32] &= ~(1 <<(current_page%32));
    else
        pagelist[current_page/32] |= (1 <<(current_page%32));
    /* Return the Erase Status */
    return rs;
}

/*!
 * \brief Program any data to FLASH.
 *
 * This function writes data from source address to FLASH.
 *
 * \param dst Pointer to address anywhere in FLASH. Must be word
 *        aligned
 * \param src Pointer to source data. With SRC == NULL, the region
 *        is checked for write protection
 * \param len Number of bytes to be written/checked. Must be even.
 *
 * \return FLASH Status: FLASH_COMPLETE or appropriate error.
 */
static FLASH_Status FlashWrite( void* dst, const void* src, size_t len,
                            FLASH_ERASE_MODE mode)
{
    FLASH_Status rs = FLASH_COMPLETE;
    uint32_t wrpr = FLASH->WRPR; /* Old write protection state */
    uint32_t page_start, page_end;
    int i;
    void *wptr = dst;
    const void *rptr = src;
    uint32_t length = len;
    if (len == 0)
        return FLASH_COMPLETE;

    /* Check boundaries */
    if ((((uint32_t)dst + len - 1) > FlashEnd()) || ((uint32_t)dst < FLASH_BASE)) {
        return FLASH_BOUNDARY;
    }

    /* Check for write protected sectors */
    page_start = ((uint32_t)dst        - FLASH_BASE)/FLASH_PAGE_SIZE;
    page_end = (((uint32_t)dst + len -1) - FLASH_BASE)/FLASH_PAGE_SIZE;
    /* Write protection happens in 4 kiByte unites. The uppermost bit in
       FLASH->WRPR handles all remaining units*/
#if  FLASH_PAGE_SIZE  == 1024
    for (i = page_start>>2; i <= page_end>>2; i++)
#else
    for (i = page_start>>1; i <= page_end>>1; i++)
#endif
        if (((i < 31) && ((wrpr & (1<<i)) == 0)) || ((i >= 31) && (wrpr & 0x80000000) == 0))
                return FLASH_ERROR_WRP;
    if (src == NULL)
        /* Only a check for write protection was requested */
        return  FLASH_COMPLETE;

    /* Unlock related banks*/
    if (((uint32_t)dst - FLASH_BASE)/FLASH_PAGE_SIZE <256) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
        rs = FLASH->CR & FLASH_CR_LOCK;
    }
#if defined(MCU_STM32F1_XL)
    if (((uint32_t)dst + len - FLASH_BASE)/FLASH_PAGE_SIZE > 255) {
        FLASH->KEYR2 = FLASH_KEY1;
        FLASH->KEYR2 = FLASH_KEY2;
        rs |= FLASH->SR2;
    }
#endif
    if (rs != FLASH_COMPLETE)
        return FLASH_LOCKED;
    while (length && (rs == FLASH_COMPLETE))
    {
        uint32_t current_page = ((uint32_t)wptr - FLASH_BASE)/FLASH_PAGE_SIZE;
        uint32_t current_length = length;
        uint32_t offset_in_page = (uint32_t)wptr % FLASH_PAGE_SIZE;
        int prepend = 0, append = 0;
        uint16_t prepend_data = 0, append_data = 0;

        /* Check if page needs erase*/
        if ((mode == FLASH_ERASE_ALWAYS) ||
            ((mode == FLASH_ERASE_FIRST_TOUCH) &&
             (((pagelist[current_page/32] & (1 <<(current_page%32))) == 0)))) {
            if (i < (FLASH_PAGE_SIZE>>1)) {
                rs = FlashErasePage((uint32_t)wptr);
                if (rs != FLASH_COMPLETE)
                    /* Erase failed for any reason */
                    goto done;
            }
        }
        if (length > FLASH_PAGE_SIZE)
            current_length = FLASH_PAGE_SIZE;
        length -= current_length;
        prepend = (offset_in_page & 1);
        if (prepend) {
            uint8_t saved_data = *(uint8_t*)wptr;
            if (saved_data != (ERASED_PATTERN_16 & 0xff))
                return FLASH_ERR_ALIGNMENT;
            prepend_data = (*(uint8_t*)( rptr)<< 8) | (*(uint8_t *)wptr);
        }
        else {
            append  = (current_length &1);
            if(append) {
                uint8_t saved_data = *(uint8_t*)( wptr + current_length );
                if (saved_data != (ERASED_PATTERN_16 & 0xff))
                    return FLASH_ERR_ALIGNMENT;
                append_data  = *(uint8_t*)( rptr + current_length -1) | (saved_data << 8);
            }
        }
        /* Program the sector */
        rs = FlashWaitReady();
        if (rs != FLASH_COMPLETE)
            goto done;
        if (((uint32_t)wptr -FLASH_BASE)/FLASH_PAGE_SIZE <256)
            FLASH->CR = FLASH_CR_PG;
#if defined(MCU_STM32F1_XL)
        else
            FLASH->CR2 = FLASH_CR_PG;
#endif
        if (prepend) {
            if(prepend_data != ERASED_PATTERN_16)
                *(volatile uint16_t*)((uint32_t)wptr & ~1) = prepend_data;
            current_length -= 1;
            wptr += 1;
            rptr += 1;
        }
        while (current_length > 1 && (rs == FLASH_COMPLETE)) {
            rs = FlashWaitReady();
            if(*(volatile uint16_t*)rptr != ERASED_PATTERN_16)
                *(volatile uint16_t*)wptr = *(volatile uint16_t*)rptr;
            current_length -=2;
            wptr += 2;
            rptr += 2;
        }
        if (append) {
            if(append_data != ERASED_PATTERN_16)
                *(volatile uint16_t*)(wptr) = append_data;
            current_length -= 1;
            wptr += 1;
            rptr += 1;
        }
        rs = FlashWaitReady();
        FLASH->CR = 0;
        if(rs != FLASH_COMPLETE)
            goto done;
    }
    rs = FlashWaitReady();
    /* Check the written data */
    wptr = dst;
    rptr = src;
    length = len;
    /* Align flash access to 4 Byte Boundary*/
    while (length && ((uint32_t)wptr & 3)) {
        if(*(volatile uint8_t*)wptr++ != *(uint8_t*)rptr++)
            goto cmp_err;
        length--;
    }
    /* Now compare word at a time*/
    while (length > 3) {
        if(*(volatile uint32_t*)wptr != *(uint32_t*)rptr)
            goto cmp_err;
        length -= 4;
        wptr += +4;
        rptr += +4;
    }
    /* Compare the rest */
    while (length) {
        if((*(volatile uint8_t*)wptr++) != *(uint8_t*)rptr++)
            goto cmp_err;
        length--;
    }
    goto done;
cmp_err:
    rs = FLASH_COMPARE;

done:
    /* Lock the FLASH again */
    FLASH->CR = FLASH_CR_LOCK;
#if defined(MCU_STM32F1_XL)
    FLASH->CR2 = FLASH_CR_LOCK;
#endif
    return rs;
}

/*!
 * \brief Program any data to FLASH.
 *
 * This function writes data from source address to FLASH. Write to
 * configuration area, if configured, is denied.
 *
 * \param dst Pointer to address anywhere in FLASH. Must be word
 *        aligned
 * \param src Pointer to source data. With SRC == NULL, the region
 *        is checked for write protection
 * \param len Number of bytes to be written/checked. Must be even.
 *
 * \return FLASH Status: FLASH_COMPLETE or appropriate error.
 */
FLASH_Status IapFlashWrite( void* dst, const void* src, size_t len,
                            FLASH_ERASE_MODE mode)
{
    size_t iap_flash_end = FlashEnd();
#if defined(NUT_CONFIG_STM32_IAP) && FLASH_CONF_SIZE > FLASH_PAGE_SIZE
    iap_flash_end -= FLASH_CONF_SIZE ;
#elif defined(NUT_CONFIG_STM32_IAP)
    iap_flash_end -= FLASH_PAGE_SIZE ;
#endif
    if (len == 0)
        return FLASH_COMPLETE;

    /* Check top boundary */
    if ((((uint32_t)dst + len - 1) > iap_flash_end ) || ((uint32_t)dst < FLASH_BASE))
    {
        return FLASH_BOUNDARY;
    }
    return FlashWrite(dst, src, len, mode);
}

/*!
 * \brief Get Upper limit of Flash.
 *
 * This function writes data from source address to FLASH. Write to
 * configuration area, if configured, is denied.
 *
 * \param NONE
 * \return Last Flash Address.
 */
size_t IapFlashEnd(void)
{
#if defined(NUT_CONFIG_STM32_IAP) && FLASH_CONF_SIZE > FLASH_PAGE_SIZE
    return FlashEnd() - FLASH_CONF_SIZE;
#elif defined(NUT_CONFIG_STM32_IAP)
    return FlashEnd() - FLASH_PAGE_SIZE;
#else
    return FlashEnd();
#endif
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
    uint32_t page_start, page_end;
    int i;
    FLASH_Status rs = FLASH_COMPLETE;
    uint32_t wrpr = FLASH->WRPR; /* Old write protection state */
    __IO uint16_t *WRP = &OB->WRP0;
    uint32_t iap_flash_end = FlashEnd();

    if (len == 0)
        return FLASH_COMPLETE;

    /* Check boundaries */
    if ((((uint32_t)dst+len) > iap_flash_end ) || ((uint32_t)dst < FLASH_BASE))
    {
        return FLASH_BOUNDARY;
    }

    /* Wait for last operation to be completed */
    rs = FlashWaitReady();
    if(rs != FLASH_COMPLETE)
        return rs;
    /* erase option bytes */
    /* set OPTWRE by wrinting key*/
    FLASH->OPTKEYR = FLASH_KEY1;
    FLASH->OPTKEYR = FLASH_KEY2;
    FLASH->CR = FLASH_CR_OPTER;
    FLASH->CR = FLASH_CR_OPTER | FLASH_CR_STRT;
    rs = FlashWaitReady();
    if(rs != FLASH_COMPLETE)
        return rs;
    FLASH->CR = FLASH_CR_OPTPG;
    OB->RDP = RDP_KEY;

    page_start = ((uint32_t)dst          - FLASH_BASE)/FLASH_PAGE_SIZE;
    page_end = (((uint32_t)dst + len -1) - FLASH_BASE)/FLASH_PAGE_SIZE;

    rs = FlashWaitReady();
#if  FLASH_PAGE_SIZE  == 1024
    for (i = page_start>>2; i <= page_end>>2; i++)
#else
    for (i = page_start>>1; i <= page_end>>1; i++)
#endif
    {
        if (i < 31) {
            if (ena)
                wrpr &= ~(1<<(i));
            else
                wrpr |=  (1<<(i));
        }
        /* If one high page is write protected, protect all high pages*/
        else if (ena) wrpr &= ~0x80000000;
    }
#if defined (MCU_STM32F1_LD) || defined (MCU_STM32F1_LD_VL)
    for(i = 0; i < 1 && rs == FLASH_COMPLETE; i++)
#else
    for(i = 0; i < 4 && rs == FLASH_COMPLETE; i++)
#endif
    {
        uint8_t val =  (wrpr>>(8*i) && 0xff);
        uint16_t pval =  ~val<<8 |val;

        if (val == 0xff)
            continue;
        rs = FlashWaitReady();
        if(rs != FLASH_COMPLETE)
            break;
        *WRP++ = pval;
    }
    rs = FlashWaitReady();
    FLASH->CR = 0;
    return rs;
}
/*!
 * \brief Nut/OS specific handling for parameters in FLASH.
 *
 * This function enables to read system specific parameters
 * from processors FLASH. The sectors used for storage are
 * configureable via nutconf.
 *
 * If multiple FLASH_CONF_SIZE fit into the FLASH_PAGE_SIZE,
 * implement a rolling scheme. The first FLASH_CONF_SIZE unit
 * in FLASH_PAGE_SIZE with with the uppermost word erased is
 * considered valid.
 *
 * \param pos Offset of parameter(s) in configured page(s).
 * \param data Pointer where to copy data from flash to.
 * \param len Number of bytes to be copied.
 *
 * \return FLASH_Status: FLASH_COMPLETE or appropriate error.
 */
FLASH_Status Stm32FlashParamRead(uint32_t pos, void *data, size_t len)
{
    uint32_t flash_conf_sector = FlashEnd() & FLASH_PAGE_MASK;

    if (FLASH_CONF_SIZE > FLASH_PAGE_SIZE)
        return FLASH_ERR_CONF_LAYOUT;
#if defined(FLASH_CONF_SIZE) && (FLASH_CONF_SIZE << 1) <= FLASH_PAGE_SIZE
/* More than one FLASH_CONF_SIZE sectors fit into one FLASH_PAGE_SIZE */

    uint8_t  conf_page = 0;
    uint16_t marker = *(uint16_t*) (flash_conf_sector + ((conf_page + 1)  * FLASH_CONF_SIZE) - FLASH_ACCESS_SIZE);

    if (len == 0)
        return FLASH_COMPLETE;

    /* Check boundaries */
    if (pos + len + FLASH_ACCESS_SIZE > FLASH_CONF_SIZE)
    {
        return FLASH_CONF_OVERFLOW;
    }

    /* Find configuration page in CONF_SECTOR*/
    while ((marker !=  ERASED_PATTERN_16) && (conf_page < ((FLASH_PAGE_SIZE/FLASH_CONF_SIZE) - 1 ))) {
        conf_page++;
        marker = *(uint32_t*)(flash_conf_sector + ((conf_page + 1)* FLASH_CONF_SIZE) - FLASH_ACCESS_SIZE);
    }
    if (marker !=  ERASED_PATTERN_16)
        /* no page sizes unit in CONF_SECTOR has a valid mark */
        return FLASH_ERR_CONF_LAYOUT;

    memcpy( data, (uint8_t *)((uint8_t*)flash_conf_sector + conf_page * FLASH_CONF_SIZE + pos), len);
#else
    /* FLASH_CONF size and FLASH page size are the same */
    if (len == 0)
        return FLASH_COMPLETE;

    /* Check boundaries */
    if (pos + len > FLASH_PAGE_SIZE)
    {
        return FLASH_CONF_OVERFLOW;
    }

    memcpy( data, (uint8_t *)((uint8_t*)flash_conf_sector + pos), len);
#endif

    /* Return success or fault code */
    return FLASH_COMPLETE;
}

/*!
 * \brief Nut/OS specific handling for parameters in FLASH.
 *
 * This function enables to store system specific parameters
 * in processors FLASH. The sectors used for storage are
 * configurable via nutconf.
 *
 * If multiple FLASH_CONF_SIZE fit into FLASH_PAGE_SIZE,
 * implement a rolling scheme. The first FLASH_CONF_SIZE unit
 * in FLASH_PAGE_SIZE with with the uppermost word erased is
 * considered valid.
 *
 * \param pos Offset of parameter(s) in configured page(s).
 * \param data Pointer to source data.
 * \param len Number of bytes to be written.
 *
 * \return FLASH_Status: FLASH_COMPLETE or appropriate error.
 */
FLASH_Status Stm32FlashParamWrite(unsigned int pos, const void *data,
                                  size_t len)
{
    FLASH_Status rs = 0;
    uint8_t *buffer;
    void* flash_conf_sector = (void*)(FlashEnd() & FLASH_PAGE_MASK);
    int i;
    uint8_t  *mem, *src;
    if(FLASH_CONF_SIZE > FLASH_PAGE_SIZE)
        return FLASH_ERR_CONF_LAYOUT;
#if defined(FLASH_CONF_SIZE) && (FLASH_CONF_SIZE << 1) <= FLASH_PAGE_SIZE
/* More than one FLASH_CONF_SIZE sectors fit into one FLASH_PAGE_SIZE */
    uint8_t  conf_page = 0;
    uint16_t marker = *(uint16_t*) (flash_conf_sector + ((conf_page +1) * FLASH_CONF_SIZE) - FLASH_ACCESS_SIZE);
    FLASH_ERASE_MODE mode;

    if (len == 0)
        return FLASH_COMPLETE;

    /* Check top boundaries */
    if (pos + len + FLASH_ACCESS_SIZE > FLASH_CONF_SIZE)
    {
        return FLASH_BOUNDARY;
    }

    /* Find configuration page in CONF_SECTOR*/
    while ((marker !=  ERASED_PATTERN_16) && conf_page < ((FLASH_PAGE_SIZE/FLASH_CONF_SIZE) -1)) {
        conf_page++;
        marker = *(uint16_t*)(flash_conf_sector + ((conf_page + 1)* FLASH_CONF_SIZE) - FLASH_ACCESS_SIZE);
    }
    if (marker !=  ERASED_PATTERN_16) {
        /* no page sizes unit in CONF_SECTOR has a valid mark
         * Erase Sector and write provided data to position at first sector */

        rs = FlashWrite( flash_conf_sector + pos, data, len, FLASH_ERASE_ALWAYS);
        /* Return success or fault code */
        return rs;
    }

    /* Check if target area is erased.
     * It seems no C standard function provides this functionality!
     */
    mem = (uint8_t*) (flash_conf_sector + conf_page * FLASH_CONF_SIZE + pos);
    src = (uint8_t*) data;
    for (i = 0; i < len; i++) {
        if ((mem[i] != 0xff) && (mem[i] != src[i]))
            break;
    }
    if (i >= len) {
        /* Needed area is erased, simply write the data to the requested area*/
        rs = FlashWrite( flash_conf_sector + conf_page * FLASH_CONF_SIZE + pos, data, len, FLASH_ERASE_NEVER);
        return rs;
    }

    /* Check if content needs no update. */
    if (memcmp(flash_conf_sector + conf_page * FLASH_CONF_SIZE + pos, data, len) == 0)
        return FLASH_COMPLETE;

    /* Save configuration page in RAM and write updated data to next configuration page,
     * eventually erasing the sector wrapping to the first page
     */
    buffer = NutHeapAlloc(FLASH_CONF_SIZE);
    if (buffer == NULL)
    {
        /* Not enough memory */
        return FLASH_OUT_OF_MEMORY;
    }
    /* Get the content of the whole config page*/
    memcpy( buffer, flash_conf_sector + conf_page * FLASH_CONF_SIZE , FLASH_CONF_SIZE);
    /* Overwrite new data region*/
    memcpy (buffer + pos, data, len);
    conf_page++;
    mode = FLASH_ERASE_NEVER;
    if (conf_page < FLASH_PAGE_SIZE/FLASH_CONF_SIZE) {
        uint16_t indicator = (uint16_t)~ERASED_PATTERN_16;
        rs = FlashWrite( flash_conf_sector + conf_page * FLASH_CONF_SIZE - sizeof(indicator),
                         &indicator, sizeof(indicator), FLASH_ERASE_NEVER);
    }
    else {
         /* All pages used, mark the sector as not yet erases to force erase*/
        FlashUntouch();
        conf_page = 0;
        mode = FLASH_ERASE_ALWAYS;
    }
    rs = FlashWrite( flash_conf_sector + conf_page * FLASH_CONF_SIZE , buffer, FLASH_CONF_SIZE, mode);
#else
    /* FLASH_CONF size and FLASH page size are the same */
    /* Check top boundaries */
    if (pos + len > FLASH_PAGE_SIZE)
    {
        return FLASH_BOUNDARY;
    }
    /* Check if target area is erased.
     * It seems no C standard function provides this functionality!
     */
    mem = (uint8_t*) (flash_conf_sector + pos);
    src = (uint8_t*) data;
    for (i = 0; i < len; i++) {
        if ((mem[i] != 0xff) && (mem[i] != src[i]))
            break;
    }
    if (i >= len) {
        /* Needed area is erased, simply write the data to the requested area*/
        rs = FlashWrite( flash_conf_sector + pos, data, len, FLASH_ERASE_NEVER);
        return rs;
    }
    buffer = NutHeapAlloc(FLASH_CONF_SIZE);
    if (buffer == NULL)
    {
        /* Not enough memory */
        return FLASH_OUT_OF_MEMORY;
    }
    /* Get the content of the whole config page*/
    memcpy( buffer, flash_conf_sector , FLASH_PAGE_SIZE);
    /* Overwrite new data region*/
    memcpy (buffer + pos, data, len);
    rs = FlashWrite( flash_conf_sector, buffer, FLASH_PAGE_SIZE, FLASH_ERASE_ALWAYS);

#endif
    NutHeapFree(buffer);
    /* Return success or fault code */
    return rs;
}
