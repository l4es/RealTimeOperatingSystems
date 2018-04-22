/*
 * Copyright (C) 2015 by Ole Reinhardt <ole.reinhardt@embedded-it.de>
 * Copyright (C) 2008-2011 by egnite GmbH
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
 * \file dev/spi_flash_at25df.c
 * \brief Low level SPI DataFlash routines for Adesto/Atmel AT25DF chips.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <cfg/memory.h>
#include <sys/nutdebug.h>

#include <dev/spi_node_at25df.h>

#include <stdlib.h>
#include <string.h>

/*!
 * \addtogroup xgSpiFlashAt25df
 */
/*@{*/

#ifndef FLASH_BUFFERS_AT25DF
#define FLASH_BUFFERS_AT25DF  2
#endif

#ifdef AT25DF_CRC_PAGE
#define AT25DF_CRC_SIZE  2
#else
#define AT25DF_CRC_SIZE  0
#endif

/*! \brief RAM buffer dirty flag. */
#define FLASH_BUFFER_DIRTY      0x0001

/*!
 * \brief Internal information structure.
 *
 * This structure is mainly used to keep track of the serial flash's page buffers.
 */
typedef struct _AT25DF_FLASH {
    /* \brief Flash info 
     *
     * Contains the parameters of the used AT25 dataflash chip
     */    
    AT25DF_INFO *dxb_dfinfo;
    
    /*! \brief Page number.
     *
     * Contains the number of the pages currently loaded.
     */
    sf_unit_t dxb_page[FLASH_BUFFERS_AT25DF];

    /*! \brief Attribute flags.
     *
     * Contains FLASH_BUFFER_DIRTY, if the buffer has been modified and not
     * yet written back to flash memory.
     */
    uint_fast8_t flags[FLASH_BUFFERS_AT25DF];

    /*! \brief Lock count.
     *
     * A buffer must not be written back to flash memory when this counter
     * is not equal to zero.
     */
    int dxb_locks[FLASH_BUFFERS_AT25DF];

    /*! \brief Unlock event queue.
     *
     * Queued threads waiting for an unlocked buffer.
     */
    HANDLE dxb_lque;

    /*! \brief The page buffers. */
    uint8_t *dxb_pbuf[FLASH_BUFFERS_AT25DF];
} AT25DF_FLASH;

#ifdef AT25DF_CRC_PAGE

/*
 * http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html
 */
static uint16_t crc_ccitt_update(uint16_t crc, uint8_t data)
{
    data ^= (uint8_t) (crc);
    data ^= data << 4;

    return ((((uint16_t) data << 8) | (crc >> 8)) ^ (uint8_t) (data >> 4) ^ ((uint16_t) data << 3));
}

/*!
 * \brief Execute serial flash CRC.
 *
 * \param at    Specifies the device.
 * \param b     Number of the buffer, either 0 or 1.
 * \param crc16 Pointer to the variable that receives the checksum.
 * \param xlen  Number of byte to use for the calculation.
 * \return 0 on success, -1 on errors.
 */
static int CalculateChecksum(AT25DF_FLASH * at, int_fast16_t b, uint16_t * crc16, int xlen)
{
    /* Sanity checks. */
    NUTASSERT(at != NULL);
    NUTASSERT(crc16 != NULL);

    *crc16 = 0xffff;
    if (xlen) {
        int i;
        uint8_t c;
        uint_fast8_t ne = 0;

        for (i = 0; i < xlen; i++) {
            c = at->dxb_pbuf[b][i];
            if (ne || c != 0xff) {
                ne = 1;
                *crc16 = crc_ccitt_update(*crc16, c);
            }
        }
        if (*crc16 == 0) {
            *crc16 = 0xffff;
        }
    }
    return 0;
}

#endif                          /* AT25DF_CRC_PAGE */

/*!
 * \brief Flash the given buffer.
 *
 * \param sfi Specifies the serial flash interface.
 * \param b Number of the buffer, either 0 or 1.
 *
 * \return 0 on success or -1 in case of an error.
 */
static int At25dfFlashSaveUnit(NUTSERIALFLASH * sfi, int_fast16_t b)
{
    int rc;
    AT25DF_FLASH *at;

    at = (AT25DF_FLASH *) sfi->sf_info;

#ifdef AT25DF_CRC_PAGE
    /* Store a 16 bit CRC at the end of the page, if configured. */
    {
        uint16_t crc16;

        CalculateChecksum(at, b, &crc16, sfi->sf_unit_size);
        memcpy(at->dxb_pbuf[b] + sfi->sf_unit_size, &crc16, sizeof(crc16));
    }
#endif

    rc = At25dfNodeLock(sfi->sf_node);

    if (rc == 0) {
        uint32_t pga;
        size_t offset;
        size_t len;

        pga = at->dxb_page[b];
        pga <<= at->dxb_dfinfo->at25df_ebshft;

        len = at->dxb_dfinfo->at25df_ebsize;

        for (offset = 0; (len > 0) && (rc == 0); len -= at->dxb_dfinfo->at25df_psize, offset += at->dxb_dfinfo->at25df_psize) {
            /* Put the flash in write enable mode. */
            rc = At25dfNodeTransfer(sfi->sf_node, DFCMD_WRITE_ENABLE, 0, 1, NULL, NULL, 0);
            
            /* Put the flash in write enable mode. */
            if (rc == 0) {
                rc = At25dfNodeTransfer(sfi->sf_node, DFCMD_WRITE, pga + offset, 4, at->dxb_pbuf[b] + offset, NULL, at->dxb_dfinfo->at25df_psize);
            }

            /* Wait for the erasing to be finished */
            if (rc == 0) {
                rc = At25dfNodeWaitReady(sfi->sf_node, AT25_WRITE_POLLS, 1);
            }            
        }    
        
        At25dfNodeUnlock(sfi->sf_node);

        if (rc == 0) {
            at->flags[b] &= ~FLASH_BUFFER_DIRTY;
        }
    }
    return rc;
}

/*!
 * \brief Release the specified buffer.
 *
 * \param sfi Specifies the serial flash interface.
 * \param b   Number of the buffer.
 */
static void At25dfFlashRelease(NUTSERIALFLASH * sfi, int b)
{
    AT25DF_FLASH *at = (AT25DF_FLASH *) sfi->sf_info;

    at->dxb_locks[b]--;
    NutEventPost(&at->dxb_lque);
}

/*!
 * \brief Load a given page into an available buffer.
 *
 * If all buffers are locked, the current thread will be suspended until
 * a buffer becomes available again.
 *
 * If all buffers are marked dirty, the routine will write back one of
 * the buffers back to flash.
 *
 * \param sfi  Specifies the serial flash interface.
 * \param pgn  Number of the page to load.
 * \param lock If not 0, then the RAM buffer will be locked and marked
 *             dirty. To unlock the buffer, call At25dfFlashRelease().
 *             To clean the buffer, call SpiAt25dfFlashCommit().
 *
 * \return Number of the buffer.
 */
static int_fast16_t At25dfFlashLoadUnit(NUTSERIALFLASH * sfi, sf_unit_t pgn, int_fast8_t lock)
{
    static int_fast16_t bnxt = 0;
    int_fast16_t b;
    AT25DF_FLASH *at;
    uint8_t read_cmd;
    uint8_t oplen;

    /* Select the flash read command depending on the selected SPI speed */
    if (sfi->sf_node->node_rate <= AT25_MAX_SPEED_SLOW) {
        read_cmd = DFCMD_READ_ARRAY_SLOW;
        oplen = 4;
    } else 
    if (sfi->sf_node->node_rate <= AT25_MAX_SPEED_MED) {
        read_cmd = DFCMD_READ_ARRAY_MED;
        oplen = 5;
    } else {
        read_cmd = DFCMD_READ_ARRAY_FAST;
        oplen = 6;
    }

    at = (AT25DF_FLASH *) sfi->sf_info;
    for (;;) {
        /*
         * Check, if the page is already loaded.
         */
        for (b = 0; b < FLASH_BUFFERS_AT25DF; b++) {
            if (at->dxb_page[b] == pgn) {
                if (lock) {
                    at->dxb_locks[b]++;
                    at->flags[b] |= FLASH_BUFFER_DIRTY;
                }
                return b;
            }
        }

        /*
         * Search for a clean unlocked buffer and load the page.
         */
        b = bnxt;
        do {
            if (at->dxb_locks[b] == 0 && (at->flags[b] & FLASH_BUFFER_DIRTY) == 0) {
                break;
            }
            if (++b >= FLASH_BUFFERS_AT25DF) {
                b = 0;
            }
            if (b == bnxt) {
                b = -1;
            }
        } while (b >= 0);
        if (b >= 0) {
            int rc;
            uint32_t pga = pgn;

            if (At25dfNodeLock(sfi->sf_node) == 0) {
                pga <<= at->dxb_dfinfo->at25df_ebshft;
                if (lock) {
                    at->dxb_locks[b]++;
                }

                /* Load the selected page from flash */
                rc = At25dfNodeTransfer(sfi->sf_node, read_cmd, pga, oplen, NULL, at->dxb_pbuf[b], sfi->sf_unit_size + AT25DF_CRC_SIZE);

                At25dfNodeUnlock(sfi->sf_node);
                if (rc == 0) {
                    at->dxb_page[b] = pgn;
                    if (lock) {
                        at->flags[b] |= FLASH_BUFFER_DIRTY;
                    }
                    if (++bnxt >= FLASH_BUFFERS_AT25DF) {
                        bnxt = 0;
                    }
                    return b;
                }
                if (lock) {
                    At25dfFlashRelease(sfi, b);
                }
            }
            return -1;
        }

        /*
         * No clean buffer. If not locked, save one of them.
         */
        for (b = 0; b < FLASH_BUFFERS_AT25DF; b++) {
            if (at->dxb_locks[b] == 0 && (at->flags[b] & FLASH_BUFFER_DIRTY) == FLASH_BUFFER_DIRTY) {
                if (At25dfFlashSaveUnit(sfi, b)) {
                    return -1;
                }
                break;
            }
        }

        /*
         * All buffers are locked. Wait for an unlock event.
         */
        if (b >= FLASH_BUFFERS_AT25DF) {
            NutEventWait(&at->dxb_lque, 0);
        }
    }
}

/*!
 * \brief Initialize the serial flash interface.
 *
 * \param sfi Specifies the serial flash interface.
 *
 * \return 0 on success or -1 in case of an error.
 */
static int SpiAt25dfFlashInit(NUTSERIALFLASH * sfi)
{
    int           rc;
    int_fast16_t   b;
    AT25DF_INFO  *df;
    AT25DF_FLASH *at;

    /* Sanity checks. */
    NUTASSERT(sfi != NULL);
    NUTASSERT(sfi->sf_node != NULL);

    df = At25dfNodeProbe(sfi->sf_node);
    if (df == NULL) {
        /* Unknown DataFlash type. */
        return -1;
    }
    /* Known DataFlash type. */
    at = calloc(1, sizeof(AT25DF_FLASH));
    if (at == NULL) {
        return -1;
    }
    at->dxb_dfinfo = df;
    for (b = 0; b < FLASH_BUFFERS_AT25DF; b++) {
        at->dxb_page[b] = SERIALFLASH_MAX_UNITS;
        at->dxb_pbuf[b] = calloc(1, df->at25df_ebsize);
    }
    sfi->sf_info = (void *) at;
    sfi->sf_units = (sf_unit_t) df->at25df_erase_blocks;
    sfi->sf_unit_size = df->at25df_ebsize - AT25DF_CRC_SIZE;


    /* Put the flash in write enable mode. */
    rc = At25dfNodeTransfer(sfi->sf_node, DFCMD_WRITE_ENABLE, 0, 1, NULL, NULL, 0);
            
    /* Global unprotect the flash */
    if (rc == 0) {
        rc = At25dfNodeTransfer(sfi->sf_node, DFCMD_WRITE_STATUS1, 0, 2, NULL, NULL, 0);
    }
           
    /* Wait for the erase operation to complete */
    if (rc == 0) {
        rc = At25dfNodeWaitReady(sfi->sf_node, AT25_WRITE_POLLS, 1);
    }

    return rc;
}

/*!
 * \brief Release the interface.
 *
 * \param sfi Specifies the serial flash interface.
 */
static void SpiAt25dfFlashExit(NUTSERIALFLASH * sfi)
{
    int_fast16_t b;
    AT25DF_FLASH *at;

    NUTASSERT(sfi != NULL);
    NUTASSERT(sfi->sf_info != NULL);

    at = (AT25DF_FLASH *) sfi->sf_info;
    for (b = 0; b < FLASH_BUFFERS_AT25DF; b++) {
        free(at->dxb_pbuf[b]);
    }
    free(sfi->sf_info);
}

/*!
 * \brief Verify CRC of consecutive pages.
 *
 * \param sfi Specifies the serial flash interface.
 * \param pgn First page to verify.
 * \param cnt Number of consecutive pages to verify.
 *
 * \return 0 on success or if CRC checking is disabled (compile option).
 *         In case of an error -1 is returned.
 */
static int SpiAt25dfFlashCheck(NUTSERIALFLASH * sfi, sf_unit_t pgn, int cnt)
{
    int rc = 0;

#ifdef AT25DF_CRC_PAGE
    AT25DF_FLASH *at;
    int_fast16_t b;
    uint16_t crc16 = 0;

    /* Sanity checks. */
    NUTASSERT(sfi != NULL);
    NUTASSERT(sfi->sf_info != NULL);
    NUTASSERT(pgn + cnt <= sfi->sf_units);
    NUTASSERT(cnt >= 0);

    at = (AT25DF_FLASH *) sfi->sf_info;
    while (cnt--) {
        b = At25dfFlashLoadUnit(sfi, pgn + cnt, 0);
        if (b >= 0) {
            CalculateChecksum(at, b, &crc16, sfi->sf_unit_size + AT25DF_CRC_SIZE);
        }
        if (crc16 != 0xFFFF) {
            rc = -1;
            break;
        }
    }
#endif
    return rc;
}

/*!
 * \brief Read data from a given page.
 *
 * \param sfi  Specifies the serial flash interface.
 * \param pgn  Page to read from.
 * \param off  Read offset into the page. If negative, the offset counts
 *             from the end of the page.
 * \param data Pointer to the buffer that receives the data.
 * \param len  Number of data bytes to read.
 *
 * \return 0 on success or -1 in case of an error.
 */
static int SpiAt25dfFlashRead(NUTSERIALFLASH * sfi, sf_unit_t pgn, int off, void *data, int len)
{
    int rc = 0;

    /* Sanity checks. */
    NUTASSERT(sfi != NULL);
    NUTASSERT(sfi->sf_info != NULL);
    NUTASSERT(pgn < sfi->sf_units);
    NUTASSERT(off >= -(int) sfi->sf_unit_size && off <= (int) sfi->sf_unit_size);
    NUTASSERT(data != NULL);
    NUTASSERT(len >= 0 && len <= sfi->sf_unit_size);

    if (len) {
        int_fast16_t b;
        AT25DF_FLASH *at = (AT25DF_FLASH *) sfi->sf_info;

        /* Normalize the offset. */
        if (off < 0) {
            off += sfi->sf_unit_size;
        }
        NUTASSERT(off + len <= (int) sfi->sf_unit_size);

        b = At25dfFlashLoadUnit(sfi, pgn, 0);
        if (b < 0) {
            rc = -1;
        } else {
            memcpy(data, at->dxb_pbuf[b] + off, len);
        }
    }
    return rc;
}

/*!
 * \brief Compare data with the contents of a given page.
 *
 * \param sfi  Specifies the serial flash interface.
 * \param pgn  Page to compare.
 * \param off  Offset into the page. If negative, the offset counts from
 *             the end of the page.
 * \param data Pointer to the data to compare.
 * \param len  Number of data bytes to compare.
 *
 * \return 0 on success or -1 if the contents differs or in case of an error.
 */
static int SpiAt25dfFlashCompare(NUTSERIALFLASH * sfi, sf_unit_t pgn, int off, const void *data, int len)
{
    int rc = 0;

    /* Sanity checks. */
    NUTASSERT(sfi != NULL);
    NUTASSERT(sfi->sf_info != NULL);
    NUTASSERT(pgn < sfi->sf_units);
    NUTASSERT(off >= -(int) sfi->sf_unit_size && off <= (int) sfi->sf_unit_size);
    NUTASSERT(data != NULL);
    NUTASSERT(len >= 0 && len <= sfi->sf_unit_size);

    if (len) {
        int_fast16_t b;
        AT25DF_FLASH *at = (AT25DF_FLASH *) sfi->sf_info;

        /* Normalize the offset. */
        if (off < 0) {
            off += sfi->sf_unit_size;
        }
        NUTASSERT(off + len <= (int) sfi->sf_unit_size);

        b = At25dfFlashLoadUnit(sfi, pgn, 0);
        if (b < 0 || memcmp(at->dxb_pbuf[b] + off, data, len)) {
            rc = -1;
        }
    }
    return rc;
}

/*!
 * \brief Return the number of bytes used in a given page.
 *
 * Simply counts the number of trailing 0xFF bytes and may fail when used
 * with binary data.
 *
 * \param sfi  Specifies the serial flash interface.
 * \param pgn  Page to check.
 * \param skip Number of bytes to ingore. May be a positive or negative
 *             value, to ingnore bytes at the beginning or at the end, resp.
 *
 * \return 0 on success or -1 if the contents differs or in case of an error.
 */
static int SpiAt25dfFlashUsed(NUTSERIALFLASH * sfi, sf_unit_t pgn, int skip)
{
    int rc;
    int len;
    AT25DF_FLASH *at;
    int_fast16_t b;

    /* Sanity checks. */
    NUTASSERT(sfi != NULL);
    NUTASSERT(pgn < sfi->sf_units);
    NUTASSERT(skip <= (int) sfi->sf_unit_size);

    at = (AT25DF_FLASH *) sfi->sf_info;

    /* Determine length and offset. */
    len = (int) sfi->sf_unit_size;
    if (skip < 0) {
        len += skip;
        skip = 0;
    } else {
        len -= skip;
    }

    b = At25dfFlashLoadUnit(sfi, pgn, 0);
    if (b < 0) {
        rc = -1;
    } else {
        rc = 0;
        if (len) {
            uint8_t *cp = at->dxb_pbuf[b] + skip;
            int i;

            for (i = 0; i < len; i++) {
                if (*cp++ != 0xff) {
                    rc = i + 1;
                }
            }
        }
    }
    return rc;
}

/*!
 * \brief Write data to a given page.
 *
 * Loads the page into a buffer and replaces the contents with the given
 * data. The buffer will be marked dirty, but not written back. See
 * SpiAt25dfFlashCommit().
 *
 * \param sfi  Specifies the serial flash interface.
 * \param pgn  Page to write to.
 * \param off  Offset into the page, where the new data should be placed.
 *             If negative, the offset counts from the end of the page.
 * \param data Pointer to the data to write.
 * \param len  Number of data bytes to write.
 *
 * \return 0 on success or -1 in case of an error.
 */
static int SpiAt25dfFlashWrite(NUTSERIALFLASH * sfi, sf_unit_t pgn, int off, const void *data, int len)
{
    int rc = 0;

    /* Sanity checks. */
    NUTASSERT(sfi != NULL);
    NUTASSERT(pgn < sfi->sf_units);
    NUTASSERT(off >= -(int) sfi->sf_unit_size && off <= (int) sfi->sf_unit_size);
    NUTASSERT(len == 0 || data != NULL);
    NUTASSERT(len >= 0 && len <= sfi->sf_unit_size);

    if (len) {
        int_fast16_t b;
        AT25DF_FLASH *at = (AT25DF_FLASH *) sfi->sf_info;

        /* Normalize the offset. */
        if (off < 0) {
            off += sfi->sf_unit_size;
        }
        NUTASSERT(off + len <= (int) sfi->sf_unit_size);

        /* Load the page. */
        b = At25dfFlashLoadUnit(sfi, pgn, 1);
        if (b < 0) {
            return -1;
        }
        /* Transfer the data and release the page. */
        memcpy(at->dxb_pbuf[b] + off, data, len);
        At25dfFlashRelease(sfi, b);
    }
    return rc;
}

/*!
 * \brief Copy page.
 *
 * Loads the source page into a buffer und designates the buffer to the
 * destination page. The buffer will be marked dirty, but not written back.
 * See SpiAt25dfFlashCommit().
 *
 * \param sfi Specifies the serial flash interface.
 * \param spg Source page to copy from.
 * \param dpg Destination page to copy to.
 *
 * \return 0 on success or -1 in case of an error.
 */
static int SpiAt25dfFlashCopy(NUTSERIALFLASH * sfi, sf_unit_t spg, sf_unit_t dpg)
{
    int_fast16_t b;

    /* Sanity checks. */
    NUTASSERT(sfi != NULL);
    NUTASSERT(sfi->sf_info != NULL);
    NUTASSERT(spg < sfi->sf_units);
    NUTASSERT(dpg < sfi->sf_units);

    /* If source and destination page numbers are equal, just load it. */
    if (spg == dpg) {
        b = At25dfFlashLoadUnit(sfi, spg, 1);
    } else {
        AT25DF_FLASH *at = (AT25DF_FLASH *) sfi->sf_info;

        /* Invalidate any buffered destination. */
        for (b = 0; b < FLASH_BUFFERS_AT25DF; b++) {
            if (at->dxb_page[b] == dpg) {
                /* But do not touch locked pages. */
                if (at->dxb_locks[b]) {
                    return -1;
                }
                at->dxb_page[b] = SERIALFLASH_MAX_UNITS;
            }
        }
        /* Try to load the source page and make it the destination page. */
        b = At25dfFlashLoadUnit(sfi, spg, 1);
        if (b >= 0) {
            at->dxb_page[b] = dpg;
        }
    }
    if (b >= 0) {
        At25dfFlashRelease(sfi, b);
        return 0;
    }
    return -1;
}

/*!
 * \brief Commit page.
 *
 * If the contents of the given page is in one of the internal RAM
 * buffers and if the contents had been changed since the last page
 * load, then the buffer is written back to the flash.
 *
 * \param sfi Specifies the serial flash interface.
 * \param pgn Page to commit.
 *
 * \return 0 on success or -1 in case of an error.
 */
static int SpiAt25dfFlashCommit(NUTSERIALFLASH * sfi, sf_unit_t pgn)
{
    int rc = 0;
    int_fast16_t b;
    AT25DF_FLASH *at;

    /* Sanity checks. */
    NUTASSERT(sfi != NULL);
    NUTASSERT(pgn < sfi->sf_units);

    at = (AT25DF_FLASH *) sfi->sf_info;
    for (b = 0; b < FLASH_BUFFERS_AT25DF && at->dxb_page[b] != pgn; b++);
    if (b < FLASH_BUFFERS_AT25DF && (at->flags[b] & FLASH_BUFFER_DIRTY) == FLASH_BUFFER_DIRTY) {
        rc = At25dfFlashSaveUnit(sfi, b);
    }
    return rc;
}

/*!
 * \brief Erase consecutive pages.
 *
 * \param sfi Specifies the serial flash interface.
 * \param pgn First page to erase.
 * \param cnt Number of consecutive pages to erase.
 *
 * \return 0 on success or -1 in case of an error.
 */
static int SpiAt25dfFlashErase(NUTSERIALFLASH * sfi, sf_unit_t pgn, int cnt)
{
    int           rc = 0;
    int_fast16_t   b;
    AT25DF_FLASH *at;
    uint8_t       erase_cmd;
    int           erase_wait;
    uint32_t      pga;

    /* Sanity checks. */
    NUTASSERT(sfi != NULL);
    NUTASSERT(pgn + cnt <= sfi->sf_units);
    NUTASSERT(cnt >= 0);

    at = (AT25DF_FLASH *) sfi->sf_info;
    /* Invalidate any buffered page. */
    for (b = 0; b < FLASH_BUFFERS_AT25DF; b++) {
        if (at->dxb_page[b] >= pgn && at->dxb_page[b] < pgn + cnt) {
            at->dxb_page[b] = SERIALFLASH_MAX_UNITS;
            at->flags[b] &= ~FLASH_BUFFER_DIRTY;
        }
    }
    while (rc == 0 && cnt) {
        if ((pgn == 0) && (cnt == at->dxb_dfinfo->at25df_erase_blocks)) {
            erase_cmd = DFCMD_CHIP_ERASE;
            pga = pgn << at->dxb_dfinfo->at25df_ebshft;
            cnt -= at->dxb_dfinfo->at25df_erase_blocks;
            pgn += at->dxb_dfinfo->at25df_erase_blocks;
            erase_wait = AT25_CHIP_ERASE_WAIT;
        } else
        if (((pgn & 0x0F) == 0) && (cnt >= 16)) {
            erase_cmd = DFCMD_BLOCK_ERASE_64K;
            pga = pgn << at->dxb_dfinfo->at25df_ebshft;
            cnt -= 16;
            pgn += 16;
            erase_wait = AT25_BLOCK_ERASE_WAIT;
        } else 
        if (((pgn & 0x07) == 0) && (cnt >= 8)) {
            erase_cmd = DFCMD_BLOCK_ERASE_32K;
            pga = pgn << at->dxb_dfinfo->at25df_ebshft;
            cnt -= 8;
            pgn += 8;
            erase_wait = AT25_BLOCK_ERASE_WAIT;
        } else {
            erase_cmd = DFCMD_BLOCK_ERASE_4K;
            pga = pgn << at->dxb_dfinfo->at25df_ebshft;
            cnt -= 1;
            pgn += 1;
            erase_wait = AT25_BLOCK_ERASE_WAIT;
        }

        rc = At25dfNodeLock(sfi->sf_node);

        if (rc == 0) {

            /* Put the flash in write enable mode. */
            rc = At25dfNodeTransfer(sfi->sf_node, DFCMD_WRITE_ENABLE, 0, 1, NULL, NULL, 0);
            
            /* Erase the erase block. */
            if (rc == 0) {
                rc = At25dfNodeTransfer(sfi->sf_node, erase_cmd, pga, 4, NULL, NULL, 0);
            }
            
            /* Wait for the erase operation to complete */
            if (rc == 0) {
                rc = At25dfNodeWaitReady(sfi->sf_node, erase_wait, 0);
            }

            At25dfNodeUnlock(sfi->sf_node);
        }
    }

    return rc;
}

#ifndef FLASH_MOUNT_OFFSET_AT25DF0
#ifdef MOUNT_OFFSET_AT25DF0
#define FLASH_MOUNT_OFFSET_AT25DF0       MOUNT_OFFSET_AT25DF0
#else
#define FLASH_MOUNT_OFFSET_AT25DF0       0
#endif
#endif

#ifndef FLASH_MOUNT_TOP_RESERVE_AT25DF0
#ifdef MOUNT_TOP_RESERVE_AT25DF0
#define FLASH_MOUNT_TOP_RESERVE_AT25DF0  MOUNT_TOP_RESERVE_AT25DF0
#else
#define FLASH_MOUNT_TOP_RESERVE_AT25DF0  0
#endif
#endif

/*!
 * \brief First AT25DF DataFlash interface implementation structure.
 */
NUTSERIALFLASH flashAt25df0 = {
    &nodeAt25df0,                /*!< \brief Pointer to the SPI node structure, sf_node. */
    NULL,                        /*!< \brief Pointer to a local information structure, sf_info. */
    0,                           /*!< \brief Size of an erase/write unit, sf_unit_size. */
    0,                           /*!< \brief Total number of units, sf_units. */
    FLASH_MOUNT_OFFSET_AT25DF0,  /*!< \brief Reserved units at the bottom, sf_rsvbot. */
    FLASH_MOUNT_TOP_RESERVE_AT25DF0,     /*!< \brief Reserved units at the top, sf_rsvtop. */
    SpiAt25dfFlashInit,          /*!< \brief Flash device initialization function, sf_init. */
    SpiAt25dfFlashExit,          /*!< \brief Flash device release function, sf_exit. */
    SpiAt25dfFlashCheck,         /*!< \brief Unit validation function, sf_check. */
    SpiAt25dfFlashRead,          /*!< \brief Data read function, sf_read. */
    SpiAt25dfFlashCompare,       /*!< \brief Data compare function, sf_compare. */
    SpiAt25dfFlashUsed,          /*!< \brief Unit usage query function, sf_used. */
    SpiAt25dfFlashWrite,         /*!< \brief Data write function, sf_write. */
    SpiAt25dfFlashCopy,          /*!< \brief Unit copy function, sf_copy. */
    SpiAt25dfFlashCommit,        /*!< \brief Unit commit function, sf_commit. */
    SpiAt25dfFlashErase          /*!< \brief Unit erase function, sf_erase. */
};

#ifndef FLASH_MOUNT_OFFSET_AT25DF1
#ifdef MOUNT_OFFSET_AT25DF1
#define FLASH_MOUNT_OFFSET_AT25DF1       MOUNT_OFFSET_AT25DF1
#else
#define FLASH_MOUNT_OFFSET_AT25DF1       0
#endif
#endif

#ifndef FLASH_MOUNT_TOP_RESERVE_AT25DF1
#ifdef MOUNT_TOP_RESERVE_AT25DF1
#define FLASH_MOUNT_TOP_RESERVE_AT25DF1  MOUNT_TOP_RESERVE_AT25DF1
#else
#define FLASH_MOUNT_TOP_RESERVE_AT25DF1  0
#endif
#endif

/*!
 * \brief Second AT25DF DataFlash interface implementation structure.
 */
NUTSERIALFLASH flashAt25df1 = {
    &nodeAt25df1,                /*!< \brief Pointer to the SPI node structure, sf_node. */
    NULL,                        /*!< \brief Pointer to a local information structure, sf_info. */
    0,                           /*!< \brief Size of an erase/write unit, sf_unit_size. */
    0,                           /*!< \brief Total number of units, sf_units. */
    FLASH_MOUNT_OFFSET_AT25DF1,  /*!< \brief Reserved units at the bottom, sf_rsvbot. */
    FLASH_MOUNT_TOP_RESERVE_AT25DF1,     /*!< \brief Reserved units at the top, sf_rsvtop. */
    SpiAt25dfFlashInit,          /*!< \brief Flash device initialization function, sf_init. */
    SpiAt25dfFlashExit,          /*!< \brief Flash device release function, sf_exit. */
    SpiAt25dfFlashCheck,         /*!< \brief Unit validation function, sf_check. */
    SpiAt25dfFlashRead,          /*!< \brief Data read function, sf_read. */
    SpiAt25dfFlashCompare,       /*!< \brief Data compare function, sf_compare. */
    SpiAt25dfFlashUsed,          /*!< \brief Unit usage query function, sf_used. */
    SpiAt25dfFlashWrite,         /*!< \brief Data write function, sf_write. */
    SpiAt25dfFlashCopy,          /*!< \brief Unit copy function, sf_copy. */
    SpiAt25dfFlashCommit,        /*!< \brief Unit commit function, sf_commit. */
    SpiAt25dfFlashErase          /*!< \brief Unit erase function, sf_erase. */
};

#ifndef FLASH_MOUNT_OFFSET_AT25DF2
#ifdef MOUNT_OFFSET_AT25DF2
#define FLASH_MOUNT_OFFSET_AT25DF2       MOUNT_OFFSET_AT25DF2
#else
#define FLASH_MOUNT_OFFSET_AT25DF2       0
#endif
#endif

#ifndef FLASH_MOUNT_TOP_RESERVE_AT25DF2
#ifdef MOUNT_TOP_RESERVE_AT25DF2
#define FLASH_MOUNT_TOP_RESERVE_AT25DF2  MOUNT_TOP_RESERVE_AT25DF2
#else
#define FLASH_MOUNT_TOP_RESERVE_AT25DF2  0
#endif
#endif

/*!
 * \brief Third AT25DF DataFlash interface implementation structure.
 */
NUTSERIALFLASH flashAt25df2 = {
    &nodeAt25df2,                /*!< \brief Pointer to the SPI node structure, sf_node. */
    NULL,                        /*!< \brief Pointer to a local information structure, sf_info. */
    0,                           /*!< \brief Size of an erase/write unit, sf_unit_size. */
    0,                           /*!< \brief Total number of units, sf_units. */
    FLASH_MOUNT_OFFSET_AT25DF2,  /*!< \brief Reserved units at the bottom, sf_rsvbot. */
    FLASH_MOUNT_TOP_RESERVE_AT25DF2,     /*!< \brief Reserved units at the top, sf_rsvtop. */
    SpiAt25dfFlashInit,          /*!< \brief Flash device initialization function, sf_init. */
    SpiAt25dfFlashExit,          /*!< \brief Flash device release function, sf_exit. */
    SpiAt25dfFlashCheck,         /*!< \brief Unit validation function, sf_check. */
    SpiAt25dfFlashRead,          /*!< \brief Data read function, sf_read. */
    SpiAt25dfFlashCompare,       /*!< \brief Data compare function, sf_compare. */
    SpiAt25dfFlashUsed,          /*!< \brief Unit usage query function, sf_used. */
    SpiAt25dfFlashWrite,         /*!< \brief Data write function, sf_write. */
    SpiAt25dfFlashCopy,          /*!< \brief Unit copy function, sf_copy. */
    SpiAt25dfFlashCommit,        /*!< \brief Unit commit function, sf_commit. */
    SpiAt25dfFlashErase          /*!< \brief Unit erase function, sf_erase. */
};

#ifndef FLASH_MOUNT_OFFSET_AT25DF3
#ifdef MOUNT_OFFSET_AT25DF3
#define FLASH_MOUNT_OFFSET_AT25DF3       MOUNT_OFFSET_AT25DF3
#else
#define FLASH_MOUNT_OFFSET_AT25DF3       0
#endif
#endif

#ifndef FLASH_MOUNT_TOP_RESERVE_AT25DF3
#ifdef MOUNT_TOP_RESERVE_AT25DF3
#define FLASH_MOUNT_TOP_RESERVE_AT25DF3  MOUNT_TOP_RESERVE_AT25DF3
#else
#define FLASH_MOUNT_TOP_RESERVE_AT25DF3  0
#endif
#endif

/*!
 * \brief Forth AT25DF DataFlash interface implementation structure.
 */
NUTSERIALFLASH flashAt25df3 = {
    &nodeAt25df3,                /*!< \brief Pointer to the SPI node structure, sf_node. */
    NULL,                        /*!< \brief Pointer to a local information structure, sf_info. */
    0,                           /*!< \brief Size of an erase/write unit, sf_unit_size. */
    0,                           /*!< \brief Total number of units, sf_units. */
    FLASH_MOUNT_OFFSET_AT25DF3,  /*!< \brief Reserved units at the bottom, sf_rsvbot. */
    FLASH_MOUNT_TOP_RESERVE_AT25DF3,     /*!< \brief Reserved units at the top, sf_rsvtop. */
    SpiAt25dfFlashInit,          /*!< \brief Flash device initialization function, sf_init. */
    SpiAt25dfFlashExit,          /*!< \brief Flash device release function, sf_exit. */
    SpiAt25dfFlashCheck,         /*!< \brief Unit validation function, sf_check. */
    SpiAt25dfFlashRead,          /*!< \brief Data read function, sf_read. */
    SpiAt25dfFlashCompare,       /*!< \brief Data compare function, sf_compare. */
    SpiAt25dfFlashUsed,          /*!< \brief Unit usage query function, sf_used. */
    SpiAt25dfFlashWrite,         /*!< \brief Data write function, sf_write. */
    SpiAt25dfFlashCopy,          /*!< \brief Unit copy function, sf_copy. */
    SpiAt25dfFlashCommit,        /*!< \brief Unit commit function, sf_commit. */
    SpiAt25dfFlashErase          /*!< \brief Unit erase function, sf_erase. */
};

/*@}*/
