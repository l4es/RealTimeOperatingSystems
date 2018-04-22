#ifndef _LPC17XX_IAP_H_
#define _LPC17XX_IAP_H_

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
 *
 * Parts taken from lpc177x_8x_iap.h        2011-11-21
 *
 * file     lpc177x_8x_iap.h
 * brief    Contains all functions support for IAP
 *          on LPC177x_8x
 * version  1.0
 * date     21. November. 2011
 * author   NXP MCU SW Application Team
 *
 * Copyright(C) 2011, NXP Semiconductor
 * All rights reserved.
 *
 ***********************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors'
 * relevant copyright in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 **********************************************************************/

/*----------------------------------------------------------------------------*
  General defines
 *----------------------------------------------------------------------------*/

/* IAP entry location */
#define IAP_LOCATION            (0x1FFF1FF1UL)


/**
 * \brief IAP command code definitions
 */
typedef enum
{
    IAP_PREPARE = 50,            /* Prepare sector(s) for write operation     */
    IAP_COPY_RAM2FLASH = 51,     /* Copy RAM to Flash                         */
    IAP_ERASE = 52,              /* Erase sector(s)                           */
    IAP_BLANK_CHECK = 53,        /* Blank check sector(s)                     */
    IAP_READ_PART_ID = 54,       /* Read chip part ID                         */
    IAP_READ_BOOT_VER = 55,      /* Read chip boot code version               */
    IAP_COMPARE = 56,            /* Compare memory areas                      */
    IAP_REINVOKE_ISP = 57,       /* Reinvoke ISP                              */
    IAP_READ_SERIAL_NUMBER = 58, /* Read serial number                        */
} IAP_COMMAND_CODE;

/**
 * \brief IAP status code definitions
 */
typedef enum
{
    CMD_SUCCESS,                 /* Command is executed successfully.         */
    INVALID_COMMAND,             /* Invalid command.                          */
    SRC_ADDR_ERROR,              /* Source address is not on a word boundary. */
    DST_ADDR_ERROR,              /* Destination address is not on a correct boundary.    */
    SRC_ADDR_NOT_MAPPED,         /* Source address is not mapped in the memory map.      */
    DST_ADDR_NOT_MAPPED,         /* Destination address is not mapped in the memory map. */
    COUNT_ERROR,                 /* Byte count is not multiple of 4 or is not a permitted value. */
    INVALID_SECTOR,              /* Sector number is invalid.                 */
    SECTOR_NOT_BLANK,            /* Sector is not blank.                      */
    SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION,    /* Command to prepare sector for write operation was not executed. */
    COMPARE_ERROR,               /* Source and destination data is not same.  */
    BUSY,                        /* Flash programming hardware interface is busy. */
} IAP_STATUS_CODE;

/**
 * \brief IAP write length definitions
 */
typedef enum {
    IAP_WRITE_256  = 256,
    IAP_WRITE_512  = 512,
    IAP_WRITE_1024 = 1024,
    IAP_WRITE_4096 = 4096,
} IAP_WRITE_SIZE;

/**
 * \brief IAP command structure
 */
typedef struct {
    uint32_t cmd;                 /* Command     */
    uint32_t param[4];            /* Parameters  */
    uint32_t status;              /* status code */
    uint32_t result[4];           /* Result      */
} IAP_COMMAND_Type;

/**
 * \brief IAP API
 */

uint32_t Lpc17xxIapGetSectorNr (uint32_t addr);
IAP_STATUS_CODE Lpc17xxIapSectorRead(uint32_t addr, void *data, size_t len);
IAP_STATUS_CODE Lpc17xxIapSectorWrite(uint32_t dest, void* source, IAP_WRITE_SIZE size);
IAP_STATUS_CODE Lpc17xxIapSectorErase(uint32_t start_sec, uint32_t end_sec);
IAP_STATUS_CODE Lpc17xxIapSectorBlankCheck(uint32_t start_sec, uint32_t end_sec,
                                 uint32_t *first_nblank_off, uint32_t *first_nblank_val);
IAP_STATUS_CODE Lpc17xxIapReadBootCodeVersion(uint8_t *major, uint8_t* minor);
IAP_STATUS_CODE Lpc17xxIapReadDeviceSerialNumber(uint32_t *uid);
int Lpc17xxIapParamRead(unsigned int pos, void *data, size_t len);
int Lpc17xxIapParamWrite(unsigned int pos, const void *data, size_t len);

#endif /* _LPC177X_IAP_H_ */

