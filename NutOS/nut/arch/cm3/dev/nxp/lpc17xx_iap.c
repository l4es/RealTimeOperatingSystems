/*
 * Copyright (C) 2012 by Ole Reinhardt (ole.reinhardt@embedded-it.de)
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
 *
 */

/*!
 * \file arch/cm3/dev/nxp/lpc17xx_iap.c
 * \brief LPC17xx in application programming flash controller support.
 *
 * \verbatim
 * $Id: $
 * \endverbatim
 */

#include <sys/timer.h>
#include <dev/nvmem.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <arch/cm3/nxp/lpc17xx_iap.h>

/*!
 * \addtogroup xgLpc17xxIap
 */
/*@{*/

/*! \brief Base address of the flash memory chip.
 */
#ifndef FLASH_CHIP_BASE
#define FLASH_CHIP_BASE  0x00000000
#endif

/*! \brief Address offset of the configuration sector.
 */
#ifndef FLASH_CONF_SECTOR
#define FLASH_CONF_SECTOR  0x00078000
#endif

#if ((FLASH_CONF_SECTOR & 0x000000FF) != 0)
#error FLASH_CONF_SECTOR has to be 256 byte aligned
#endif

/*! \brief Size of the configuration area.
 *
 * During write operations a buffer with this size is allocated
 * from heap and may cause memory problems with large sectors.
 * Thus, this value may be less than the size of the configuration
 * sector, in which case the rest of the sector is unused.
 *
 * Currently only 1 sector can be used for system configurations.
 */
#ifndef FLASH_CONF_SIZE
#define FLASH_CONF_SIZE         256
#elif  ((FLASH_CONF_SIZE != 256) && (FLASH_CONF_SIZE != 512) && \
        (FLASH_CONF_SIZE != 1024) && (FLASH_CONF_SIZE != 4096))
#error FLASH_CONF_SIZE has to be either 256 (default), 512, 1024 or 4096
#endif


typedef void (*IAP)(uint32_t *cmd, uint32_t *result);
IAP iap_entry = (IAP) IAP_LOCATION;
#define IAP_Call    iap_entry


typedef uint32_t flashdat_t;
typedef unsigned long flashadr_t;
typedef volatile flashdat_t *flashptr_t;


/*!
 * \brief   Get Sector Number
 *
 * \param   addr    Sector address
 *
 * \return  sector number
 *
 */

uint32_t Lpc17xxIapGetSectorNr (uint32_t addr)
{
    uint32_t n;

    n = addr >> 12;             /*  4kB Sector  */
    if (n >= 0x10) {
        n = 0x0E + (n >> 3);    /* 32kB Sector  */
    }

    return n;
}


/*!
 * \brief   Prepare sector(s) for write operation
 *
 * \param   start_sec   Number of start sector
 * \param   end_sec     Number of end sector
 *
 * \return  CMD_SUCCESS/BUSY/INVALID_SECTOR
 *
 */

static IAP_STATUS_CODE Lpc17xxIapSectorPrepare(uint32_t start_sec, uint32_t end_sec)
{
    IAP_COMMAND_Type command;

    command.cmd      = IAP_PREPARE;             /* Prepare Sector for Write   */
    command.param[0] = start_sec;               /* Start Sector               */
    command.param[1] = end_sec;                 /* End Sector                 */
    IAP_Call (&command.cmd, &command.status);   /* Call IAP Command           */
    return (IAP_STATUS_CODE)command.status;
}


/*!
 * \brief Read data from flash memory.
 *
 * \param addr  Start location within the chip, starting at 0.
 * \param data  Points to a buffer that receives the data.
 * \param len   Number of bytes to read.
 *
 * \return Always CMD_SUCCESS.
 */
IAP_STATUS_CODE Lpc17xxIapSectorRead(uint32_t addr, void *data, size_t len)
{
    if (data == NULL) {
        return DST_ADDR_ERROR;
    }

    memcpy(data, (void *) (uptr_t) (FLASH_CHIP_BASE + addr), len);

    return CMD_SUCCESS;
}


/*!
 * \brief   Write data to flash
 *
 * \param   dest        destination buffer (in Flash memory) (must be 256 byte aligned).
 * \param   source      source buffer (in RAM) (should be word aligned)
 * \param   size        the write size.
 *
 * \return  CMD_SUCCESS. SRC_ADDR_ERROR/DST_ADDR_ERROR
 *          SRC_ADDR_NOT_MAPPED/DST_ADDR_NOT_MAPPED
 *          COUNT_ERROR/SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION
 *          BUSY
 *
 */

IAP_STATUS_CODE Lpc17xxIapSectorWrite(uint32_t dest, void* source, IAP_WRITE_SIZE size)
{
    uint32_t sec;
    IAP_STATUS_CODE status;
    IAP_COMMAND_Type command;

    /* Prepare sectors */
    sec = Lpc17xxIapGetSectorNr(dest);
    status = Lpc17xxIapSectorPrepare(sec, sec);
    if(status != CMD_SUCCESS) {
        return status;
    }

    /* Write data to flash */
    command.cmd      = IAP_COPY_RAM2FLASH;      /* Copy RAM to Flash          */
    command.param[0] = dest;                    /* Destination Flash Address  */
    command.param[1] = (uint32_t)source;        /* Source RAM Address         */
    command.param[2] = size;                    /* Number of bytes            */
    command.param[3] = NutArchClockGet(NUT_HWCLK_CPU) / 1000; /* CCLK in kHz */
    IAP_Call (&command.cmd, &command.status);   /* Call IAP Command           */

    return (IAP_STATUS_CODE)command.status;
}


/*!
 * \brief   Erase sector(s)
 *
 * \param   start_sec   Number of start sector
 * \param   end_sec     Number of end sector
 *
 * \return  CMD_SUCCESS, INVALID_SECTOR,
 *          SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION, BUSY
 *
 */

IAP_STATUS_CODE Lpc17xxIapSectorErase(uint32_t start_sec, uint32_t end_sec)
{
    IAP_COMMAND_Type command;
    IAP_STATUS_CODE status;

    /* Prepare sectors */
    status = Lpc17xxIapSectorPrepare(start_sec, end_sec);
    if(status != CMD_SUCCESS) {
        return status;
    }

    /* Erase sectors */
    command.cmd      = IAP_ERASE;               /* Prepare Sector for Write   */
    command.param[0] = start_sec;               /* Start Sector               */
    command.param[1] = end_sec;                 /* End Sector                 */
    command.param[2] = NutArchClockGet(NUT_HWCLK_CPU) / 1000; /* CCLK in kHz */
    IAP_Call (&command.cmd, &command.status);   /* Call IAP Command           */
    return (IAP_STATUS_CODE)command.status;
}


/*!
 * \brief   Blank check sector(s)
 *
 * \param   start_sec   Number of start sector
 * \param   end _sec    Number of end sector
 *
 * \param   first_nblank_off  The offset of the first non-blank word
 * \param   first_nblank_val  The value of the first non-blank word
 * \return  CMD_SUCCESS, INVALID_SECTOR, SECTOR_NOT_BLANK, BUSY
 *
 */

IAP_STATUS_CODE Lpc17xxIapSectorBlankCheck(uint32_t start_sec, uint32_t end_sec,
                                 uint32_t *first_nblank_off,
                                 uint32_t *first_nblank_val)
{
    IAP_COMMAND_Type command;

    command.cmd      = IAP_BLANK_CHECK;         /* Prepare Sector for Write   */
    command.param[0] = start_sec;               /* Start Sector               */
    command.param[1] = end_sec;                 /* End Sector                 */
    IAP_Call (&command.cmd, &command.status);   /* Call IAP Command           */

    if(command.status == SECTOR_NOT_BLANK) {
        if(first_nblank_off != NULL) {
            *first_nblank_off =  command.result[0];
        }

        if(first_nblank_val != NULL) {
            *first_nblank_val =  command.result[1];
        }
    }

    return (IAP_STATUS_CODE)command.status;
}


/*!
 * \brief   Read boot code version. The version is interpreted as <major>.<minor>.
 *
 * \param   major   Returns major version number
 * \param   minor   Returns minor version number
 *
 * \return  CMD_SUCCESS
 *
 */

IAP_STATUS_CODE Lpc17xxIapReadBootCodeVersion(uint8_t *major, uint8_t* minor)
{
    IAP_COMMAND_Type command;
    command.cmd = IAP_READ_BOOT_VER;
    IAP_Call (&command.cmd, &command.status);   /* Call IAP Command           */

    if(command.status == CMD_SUCCESS) {
        if(major != NULL) {
            *major = (command.result[0] >> 8) & 0xFF;
        }

        if(minor != NULL) {
            *minor = (command.result[0]) & 0xFF;
        }
    }

    return (IAP_STATUS_CODE)command.status;
}

/*!
 * \brief   Read Device serial number.
 *
 * \param   uid     Serial number
 *
 * \return  CMD_SUCCESS
 *
 */

IAP_STATUS_CODE Lpc17xxIapReadDeviceSerialNumber(uint32_t *uid)
{
    IAP_COMMAND_Type command;
    command.cmd = IAP_READ_SERIAL_NUMBER;
    IAP_Call (&command.cmd, &command.status);   /* Call IAP Command           */

    if(command.status == CMD_SUCCESS) {
        if(uid != NULL) {
            uint32_t i = 0;
            for(i = 0; i < 4; i++) {
                uid[i] =  command.result[i];
            }
        }
    }

    return (IAP_STATUS_CODE)command.status;
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
 * \return Always 0.
 */
int Lpc17xxIapParamRead(unsigned int pos, void *data, size_t len)
{
    Lpc17xxIapSectorRead(FLASH_CONF_SECTOR + pos, data, len);
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
int Lpc17xxIapParamWrite(unsigned int pos, const void *data, size_t len)
{
    int rc = -1;
    uint8_t *buff;

    /* Load the complete configuration area. */
    if ((buff = malloc(FLASH_CONF_SIZE)) != NULL) {

        rc = Lpc17xxIapSectorRead(FLASH_CONF_SECTOR, buff, FLASH_CONF_SIZE);
        /* Compare old with new contents. */
        if (memcmp(buff + pos, data, len)) {
            /* New contents differs. Copy it into the sector buffer. */
            memcpy(buff + pos, data, len);
            /* Write back new data. Maintain region lock. */
            if (Lpc17xxIapSectorWrite(FLASH_CONF_SECTOR, buff, FLASH_CONF_SIZE) == CMD_SUCCESS) {
                rc = 0;
            }
        }
        free(buff);
    }
    return rc;
}

/*@}*/
