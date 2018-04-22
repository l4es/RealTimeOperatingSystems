#ifndef _OWIBUS_H_
#define _OWIBUS_H_

/*
 * Copyright (C) 2012-14, 2016 by Uwe Bonnes
 *                              (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * \file dev/owibus.c
 * \brief Header for the One-Wire API.
 *
 * \verbatim
 * $Id: owibus.h 6533 2016-09-07 10:21:35Z u_bonnes $
 * \endverbatim
 */

#include <cfg/arch.h>
#include <stdint.h>
#include <sys/device.h>

/*!
 * \addtogroup xgOwibus
 */
/*@{*/

/* Overdrive Commands as in
 * http://www.ovro.caltech.edu/~dwh/correlator/pdf/onewire.pdf
 */
/*!
 * \brief One-Wire command codes for devices
 */
#define OWI_READ_ROM            0x33    /*!< \brief Read 64-bit serial ID (single device only). */
#define OWI_OVERDRIVE_SKIP_ROM  0x3c    /*!< \brief As OWI_SKIP_ROM, but with command phase in fast mode. */
#define OWI_CONVERT_T           0x44    /*!< \brief DS18B20: Start conversion on selected device(s). */
#define OWI_MATCH_ROM           0x55    /*!< \brief Qualify command to single device. */
#define OWI_OVERDRIVE_MATCH_ROM 0x69    /*!< \brief As OWI_MATCH_ROM but with command phase in fast mode. */
#define OWI_SKIP_ROM            0xCC    /*!< \brief Qualify command as broadcast. */
#define OWI_READ                0xBE    /*!< \brief Read data from addressed device. */
#define OWI_SEARCH_ALARM        0xEC    /*!< \brief Prepare devices in alarm state for ID search. */
#define OWI_SEARCH_ROM          0xF0    /*!< \brief Prepare devices for ID search. */

/*!
 * \brief Constanst used for OwiSearch
 */
#define OWI_LAST_DEVICE     0xFF   /*!< \brief No more devices. */
#define OWI_SEARCH_FIRST    0x40   /*!< \brief Start value for ID search. */

/*!
 * \brief Known Owi-Wire families
 */
#define W1_FAMILY_DEFAULT       0
#define W1_FAMILY_SMEM_01       0x01
#define W1_FAMILY_SMEM_81       0x81
#define W1_THERM_DS18S20        0x10
#define W1_FAMILY_DS28E04       0x1C
#define W1_COUNTER_DS2423       0x1D
#define W1_THERM_DS1822         0x22
#define W1_EEPROM_DS2433        0x23
#define W1_THERM_DS18B20        0x28
#define W1_FAMILY_DS2408        0x29
#define W1_EEPROM_DS2431        0x2D
#define W1_FAMILY_DS2760        0x30
#define W1_FAMILY_DS2780        0x32
#define W1_THERM_DS1825         0x3B
#define W1_FAMILY_DS2781        0x3D
#define W1_THERM_DS28EA00       0x42

/*!
 * \brief OWI return codes.
 *
 * \note The enum type itself is never used.
 */
enum OWI_ERRORS {
    OWI_SUCCESS = 0,            /*!< \brief All fine. */
    OWI_PRESENCE_ERR = -1,      /*!< \brief No device present. */
    OWI_INVALID_HW = -2,        /*!< \brief Selected Hardware can't do OWI transactions. */
    OWI_OUT_OF_MEM = -3,        /*!< \brief No more memory. */
    OWI_HW_ERROR = -4,          /*!< \brief Unexpected hardware behavior. */
    OWI_DATA_ERROR = -5,        /*!< \brief Unexpected data. */
    OWI_CRC_ERROR       = -6,   /*!< \bried CRC error detected. */
    OWI_NOT_IMPLEMENTED = -7,   /*!< \brief No OWI implementation available. */
};

/*!
 * \brief OWI speed modes.
 *
 * \note The enum type itself is never used.
 */
enum OWIBUS_MODE {
    OWI_MODE_NORMAL = 0,        /*!< \brief Normal speed. */
    OWI_MODE_OVERDRIVE = 1,     /*!< \brief High speed (only some device types). */
    OWI_MODE_NONE = 2,          /*!< \brief List end marker. */
};

/*!
 * \brief OWI transactions.
 *
 * \note The enum type itself is never used.
 */
enum STM32_OWITIMER_COMMANDS {
    OWI_CMD_RESET = 0,          /*!< \brief Presence pulse. */
    OWI_CMD_RWBIT = 1,          /*!< \brief Bit transaction. */
    OWI_CMD_NONE = 2,           /*!< \brief List end marker. */
};

/*!
 * \brief OWI transactions phases.
 *
 * \note The enum type itself is never used.
 */
enum STM32_OWITIMER_PHASES {
    OWI_PHASE_SETUP = 0,        /*!< \brief Pull bus low. */
    OWI_PHASE_SYNC_PULSE = 1,   /*!< \brief Release bus high for presence pulse, read and write '1'. */
    OWI_PHASE_SYNC_PULSE_LOW = 2,/*!< \brief Release bus high to write '0'. */
    OWI_PHASE_RW = 3,           /*!< \brief Pull read bus state for presence and read. */
    OWI_PHASE_RELEASE = 4,      /*!< \brief Unconditionally release bus. */
    OWI_PHASE_NONE = 5,         /*!< \brief List end marker. */
};

/*!
 * \brief OWI bus modes
 */
#define OWI_OVERDRIVE 0x1       /*!< \brief Overdrive capable devices on the bus. */
#define OWI_PULLUP    0x2       /*!< \brief Active pull-up for loaded buses. */

typedef struct _NUTOWIBUS NUTOWIBUS;

#include "owibus_bbif.h"
#include "owibus_uart.h"

/*!
 * \brief The OWIBUS structure.
 *
 * Storage provided by the caller.
 */
struct _NUTOWIBUS {
    uintptr_t owibus_info;
    uint32_t mode;
    int(*OwiSetup) (NUTOWIBUS *);
    int(*OwiTouchReset) (NUTOWIBUS *);
    int(*OwiReadBlock) (NUTOWIBUS *bus, uint8_t *data, uint_fast8_t);
    int(*OwiWriteBlock) (NUTOWIBUS *bus, const uint8_t *data, uint_fast8_t);
};

int OwiInit(NUTOWIBUS *bus);
int OwiRomSearch(NUTOWIBUS *bus, uint8_t *diff, const uint8_t *last_hid,
                 uint8_t *hid);
int OwiAlarmSearch(NUTOWIBUS *bus, uint8_t *diff, const uint8_t *last_hid,
                 uint8_t *hid);
int OwiCommand(NUTOWIBUS *bus, const uint8_t cmd, uint8_t *hid);
int OwiReadBlock(NUTOWIBUS *bus, uint8_t *data, uint_fast8_t len);
int OwiWriteBlock(NUTOWIBUS *bus, const uint8_t *data, uint_fast8_t len);
int OwiSetMode(NUTOWIBUS *bus, const uint_fast8_t mode);
int OwiGetMode(NUTOWIBUS *bus);

extern NUTOWIBUS owiBus0Uart;
extern NUTOWIBUS owiBus0Gpio;
/*@}*/

#endif
