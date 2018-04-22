/*
 * Copyright (C) 2012,2013, 2016-2017  by Uwe Bonnes
 *                               (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * \brief Implementation of the One-Wire API.
 *
 * \verbatim
 * $Id: owibus.c 6592 2017-02-15 15:35:36Z u_bonnes $
 * \endverbatim
 */

#include <stdint.h>
#include <sys/timer.h>
#include <dev/owibus.h>

/*!
 * \addtogroup xgOwibus
 */
/*@{*/

/* Values from http://www.maxim-ic.com/app-notes/index.mvp/id/126 */
const uint16_t owi_timervalues_250ns[OWI_MODE_NONE][OWI_CMD_NONE][OWI_PHASE_NONE] = {
    {
        {
            4 * 3,
            4 * (3 + 480),           /* H */
            4 * (3 + 480),           /* H*/
            4 * (3 + 480 + 70),      /* H + I */
            4 * (3 + 480 + 70 + 410) /* H + I + J */
        },
        {
            4 * 3,
            4 * (3 + 6),             /* A */
            4 * (3 + 60),            /* C */
            4 * (3 + 6 + 9),         /* A + E */
            4 * (3 + 6 + 9 + 55)     /* A + E + F*/
        }
    },
    {
        {
            10,
            10 + 280,
            10 + 280,
            10 + 280 + 34,
            10 + 280 + 34 + 160
        },
        {
            10,
            10 + 4,
            10 + 30,
            10 + 4 + 4,
            10 + 4 + 4 + 28
        }
    }
};

/*!
 * \brief Calculate onewire CRC over a data block.
 *
 * There are many implemenations out there. Here taken from
 * https://github.com/paeaetech/paeae/blob/master/Libraries/ds2482/DS2482.cpp
 *
 *
 * \param addr Pointer to data block
 * \param len  Length of data block
 *
 * \return Calculate 8 bit Onewire CRC
 */
static uint8_t crc8(const uint8_t *addr, uint8_t len)
{
     uint8_t crc = 0;
     uint_fast8_t i, j;

     for (i = 0; i < len; i++) {
           uint8_t inbyte = addr[i];
           for (j = 0;j < 8; j++) {
                 uint8_t mix = (crc ^ inbyte) & 0x01;
                 crc >>= 1;
                 if (mix) {
                       crc ^= 0x8C;
                 }
                 inbyte >>= 1;
           }
     }
     return crc;
}

/*!
 * \brief Search the connected One-Wire bus for devices.
 *
 * Eventually read Maxim AN187, Atmel doc2579.pdf and/or TI spma057b.pdf
 *
 * \param bus   Specifies the One-Wire bus.
 * \param diff  Entry: Collision bit position of last scan.
 *              Use OWI_SEARCH_FIRST for first scan.
 *              Exit: Collision bit position of current scan.
 *              OWI_LAST_DEVICE when no more devices found.
 * \param last_hid Hardware ID found during last scan. Must be 0 on first scan!
 * \param hid      Hardware ID found in this scan.
 *
 * \return OWI_SUCCESS on success, a negative value otherwise.
 */
static int OwiSearch(NUTOWIBUS *bus, uint8_t *diff, const uint8_t command,
                    const uint8_t *last_hid, uint8_t *hid)
{
    int i;                 /* Bit position in scan. */
    uint8_t mask = 1;      /* Mask for bit position in current byte.*/
    uint8_t new_diff = 0 ; /* Collision bit position during current scan. */
    uint8_t bit;           /* Target for OWI read/write transaction.*/
    int branch_flag = 0;   /* During scan, a collision found and 0 path
                            * choosen. So next scan must use 1 .path.*/
    int res;

    res = bus->OwiTouchReset(bus);
    if (res) {
        return res;
    }
    res = bus->OwiWriteBlock(bus, &command, 8);
    if (res) {
        return res;
    }
    for (i = 0; i < 64; i++) {
        res = bus->OwiReadBlock(bus, &bit, 2); /* Read bit and complement.*/
        switch (bit) {
        case 1:
        case 2:
            /* No conflict, use bit and go on. */
            break;
        case 3:
            /* No response. Abort. */
            return OWI_DATA_ERROR;
            break;
        case 0: /* Bit conflict at current bit position. */
            if (i < *diff) {
            /* Below collisions position found in last scan.
             * Use bit from last scan. */
                if ((last_hid[i / 8])  & mask) {
                    bit = 1;
                } else {
                    /* Use 0 path now, handle 1 path in next scan!*/
                    /* bit = 0; */
                    new_diff = i;
                    branch_flag = 1;
                }
            } else if (i == *diff) {
                /* Last scan took 0 path, choose 1 path now!*/
                bit = 1;
            } else {
                /* First collision at this position.*/
                /* Start with 0 path, handle 1 path in next scan.*/
                /* bit = 0; */
                new_diff = i;
                branch_flag = 1;
            }
            break;
        }

        res |= bus->OwiWriteBlock(bus, &bit, 1);  /* write choosen path */
        if (bit & 1) { /* Set bit value in new hid. */
            hid[i / 8] |= mask;
        } else {
            hid[i / 8] &= ~mask;
        }
        if (mask & 0x80) {
            mask = 1;
        } else {
            mask <<= 1;
        }
    }
#if 0
#include <stdio.h>
    fprintf(stdout,"\nlast hid: %02x%02x%02x%02x%02x%02x%02x%02x ",
            last_hid[7], last_hid[6], last_hid[5], last_hid[4],
            last_hid[3], last_hid[2], last_hid[1], last_hid[0]);
    printf("last diff :%2d, new diff %2d, branch %1d, ",
           *diff, new_diff, branch_flag);
    fprintf(stdout,"new hid: %02x%02x%02x%02x%02x%02x%02x%02x\n",
            hid[7], hid[6], hid[5], hid[4],
            hid[3], hid[2], hid[1], hid[0]);
#endif
    if (!branch_flag) {
        *diff = OWI_LAST_DEVICE;
    } else {
        *diff = new_diff;
    }
    if (crc8(hid, 7) != hid[7]) {
        return OWI_CRC_ERROR;
    }
    return res;
}

/*!
 * \brief Search the connected One-Wire bus for devices.
 *
 * Eventually read Maxim AN187, Atmel doc2579.pdf and/or TI spma057b.pdf
 *
 * \param bus   Specifies the One-Wire bus.
 * \param diff  Entry: Collision bit position of last scan.
 *              Use OWI_SEARCH_FIRST for first scan.
 *              Exit: Collision bit position of current scan.
 *              OWI_LAST_DEVICE when no more devices found.
 * \param last_hid Hardware ID found during last scan. Must be 0 on first scan!
 * \param hid      Hardware ID found in this scan.
 *
 * \return OWI_SUCCESS on success, a negative value otherwise.
 */
int OwiRomSearch(NUTOWIBUS *bus, uint8_t *diff, const uint8_t *last_hid,
                        uint8_t *hid)
{
    return OwiSearch(bus, diff, OWI_SEARCH_ROM, last_hid, hid);
}

/*!
 * \brief Search the connected One-Wire bus for devices in Alarm state.
 *
 * Eventually read Maxim AN187, Atmel doc2579.pdf and/or TI spma057b.pdf
 *
 * \param bus   Specifies the One-Wire bus.
 * \param diff  Entry: Collision bit position of last scan.
 *              Use OWI_SEARCH_FIRST for first scan.
 *              Exit: Collision bit position of current scan.
 *              OWI_LAST_DEVICE when no more devices found.
 * \param last_hid Hardware ID found during last scan. Must be 0 on first scan!
 * \param hid      Hardware ID found in this scan.
 *
 * \return OWI_SUCCESS on success, a negative value otherwise.
 */
int OwiAlarmSearch(NUTOWIBUS *bus, uint8_t *diff,
                          const uint8_t *last_hid, uint8_t *hid)
{
    return OwiSearch(bus, diff, OWI_SEARCH_ALARM, last_hid, hid);
}

/*!
 * \brief Send a command to the connected devices.
 *
 * \param bus Specifies the One-Wire bus.
 * \param cmd Command to send.
 * \param hid Device to select or NULL for broadcast.
 *
 * \return OWI_SUCCESS on success, a negative value otherwise.
 */
int OwiCommand(NUTOWIBUS *bus, const uint8_t cmd, uint8_t *hid)
{
    int res;
    uint8_t data[10];
    uint8_t *c;
    int len;

    res = bus->OwiTouchReset(bus);
    if (res) {
        return res;
    }
    c = data;
    if (hid) {
        *c++ = OWI_MATCH_ROM;
        memcpy(c, hid, 8);
        len = 80;
        c += 8;
    } else {
        *c++ = OWI_SKIP_ROM; /* to all devices */
        len = 16;
    }
    *c = cmd;
    res = bus->OwiWriteBlock(bus, data, len);
    return res;
}

/*!
 * \brief Read a block of data, evt checking CRC.
 *
 * \param bus  Specifies the One-Wire bus.
 * \param data Data read.
 * \param len  Number of bits to read. Check CRC if 9 bytes to read.
 *
 * \return OWI_SUCCESS on success, a negative value otherwise.
 */
int OwiReadBlock(NUTOWIBUS *bus, uint8_t *data, uint_fast8_t len)
{
    int res;
    res = bus->OwiReadBlock(bus, data, len);
    if (!res && (len == (9 * 8))) {
        if (crc8(data, 8) != data[8]) {
            return OWI_CRC_ERROR;
        }
    }
    return res;
}

/*!
 * \brief Write a block of data
 *
 * \param bus  Specifies the One-Wire bus.
 * \param data Data to write.
 * \param len  Number of bits to write.
 *
 * \return OWI_SUCCESS on success, a negative value otherwise.
 */
int OwiWriteBlock(NUTOWIBUS *bus, const uint8_t *data, uint_fast8_t len)
{
    return bus->OwiWriteBlock(bus, data, len);
}

/*!
 * \brief Set/Reset One-Wire Mode(s)
 *
 * \param bus  Specifies the One-Wire bus.
 * \param mode Bitmask of mode to set, at present only OWI_OVERDRIVE
 *
 * \return OWI_SUCCESS on success, a negative value otherwise.
 */
int OwiSetMode(NUTOWIBUS *bus, uint_fast8_t mode)
{
    int res;

    if (mode & OWI_OVERDRIVE) {
        uint8_t command[1] = { OWI_OVERDRIVE_SKIP_ROM };

        res = bus->OwiTouchReset(bus);
        if (res) {
            return res;
        }
        bus->OwiWriteBlock(bus, command, 8);

        bus->mode |= OWI_OVERDRIVE;

        res = bus->OwiTouchReset(bus);
        if (res) {
            bus->mode &= ~OWI_OVERDRIVE;
            return res;
        }
    } else {
        res = bus->OwiTouchReset(bus);
    }
    return res;
}

/*!
 * \brief Set/Reset One-Wire Mode(s)
 *
 * \param bus Specifies the One-Wire bus.
 *
 * \return Mask of set modes
 */
int OWIGetMode(NUTOWIBUS *bus)
{
    return bus->mode;
}

/*!
 * \brief Initialize the Owi Bus
 *
 * \param bus Specifies the One-Wire bus.
 *
 * \return OWI_SUCCESS on success, a negative value otherwise.
 */
int OwiInit(NUTOWIBUS *bus)
{
    if (!bus) {
        return OWI_INVALID_HW;
    }
    if (bus->OwiSetup) {
        int res = bus->OwiSetup(bus);
        NutSleep(1);
        return res;
    } else {
        return OWI_SUCCESS;
    }
}

/*@}*/
