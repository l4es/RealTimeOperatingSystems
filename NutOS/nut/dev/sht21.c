/*
 * Copyright (C) 2010 by Rittal GmbH & Co. KG,
 * Dawid Sadji <sadji.d@rittal.de> All rights reserved.
 * Ulrich Prinz <prinz.u@rittal.de> All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY EMBEDDED IT AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL EMBEDDED IT
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 *
 */

/*
 * \file dev/sht21.c
 * \brief Driver for SHT2x temperatuer and humidity sensors from Sensirion.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */
#include <compiler.h>

#include <cfg/os.h>

#include <sys/heap.h>
#include <sys/timer.h>
#include <sys/event.h>

#include <dev/twif.h>

#include <cfg/sht21.h>
#include <dev/sht21.h>

/* SHT21 packet size */
#define SHT_READ_COUNT 3

/* SHT21 CRC8 Polinomial */
#define POLYNOMIAL 0x131

/* SHT21 calculation factors */
#define T1x100  -4685L
#define T2x100  17572L

#define H1x100   -600L
#define H2x100  12500L

#define SHT_POLL_TOUT   90
#define SHT_POLL_CYCLE   5

#ifndef SHT21_PRECISION
#define SHT21_PRECISION SHT_RES_12_14
#endif

HANDLE sht_mutex;

/*!
 * \brief Calculate 8-Bit checksum with given polynomial.
 *
 * This function calculates the checksum of a sensirion device
 * returned data and validates it against the last byte transmitted.
 * P(x) = x^8 + x^5 + x^4 + 1 = 100110001
 *
 * \para Data Pointer to data to be checked.
 * \para Size Size of data.
 *
 * \return -1 if CRC failed else 0
 */
int ShtCrc(uint8_t *Data, uint8_t Size)
{
    uint_fast8_t i   = 0;
    uint_fast8_t bit = 0;
    uint_fast8_t checksum = 0;

    for (i = 0; i < Size; i++)
    {
        checksum ^= (Data[i]);
        for (bit = 8; bit > 0; --bit)
        {
            if (checksum & 0x80)
                checksum = (checksum << 1) ^ POLYNOMIAL;
            else
                checksum = (checksum << 1);
        }
    }

    if(checksum == Data[Size])
        return 0;
    else
        return -1;
}

/*!
 * \brief Read data from sensor device.
 *
 * This function reads the raw value of a SHT2x sensor device.
 * To get the real sensor value, the formulas given by the
 * datasheet have to be applied to this raw value.
 *
 * \param cmd can be SHT_GET_HUM or SHT_GET_TMP.
 * \param data Pointer to data value.
 *
 * \return -1 read failed else 0.
 */
int ShtCommand(uint8_t cmd, uint16_t *data)
{
    int rc = 0;
    uint8_t buf[4];
#ifdef SHT_ACK_POLLING
    int tout = SHT_POLL_TOUT;
#endif

    rc = NutEventWait(&sht_mutex, 500);
    if (rc) return rc;

    switch (cmd)
    {
        case SHT_SOFT_RESET:
            rc = TwMasterTransact( I2C_SLA_SHT21, &cmd, 1, NULL, 0, 10);
            NutSleep(15);
            break;
        case SHT_GET_TEMP:
        case SHT_GET_HUM:
#ifdef SHT_ACK_POLLING
            /* Read SHT2x by ACK-polling. */
            rc = TwMasterTransact( I2C_SLA_SHT21, &cmd, 1, NULL, 0, 10);
            while (tout>0) {
                NutSleep(SHT_POLL_CYCLE);
                tout -= SHT_POLL_CYCLE;
                rc = TwMasterTransact(I2C_SLA_SHT21, NULL, 0, buf, SHT_READ_COUNT, 10);
                if (rc==SHT_READ_COUNT)
                    break;
            }
#else
            /* Read SHT2x by SHT in Master Hold mode. */
            rc = TwMasterTransact( I2C_SLA_SHT21, &cmd, 1, buf, SHT_READ_COUNT, SHT_POLL_TOUT);
#endif
            if (rc == SHT_READ_COUNT) {
                rc = ShtCrc( buf, 2);
                *data = (buf[0]<<8)|buf[1];
            }
            else
                rc = -1;
            break;
        case SHT_GET_USER:
            rc = TwMasterRegRead(I2C_SLA_SHT21, cmd, 1, buf, 1, 100);
            if (rc==1) {
                *(uint8_t*)data = buf[0];
            }
            break;
        case SHT_SET_USER:
            buf[0] = cmd;
            buf[1] = (uint8_t)*data;
            rc = TwMasterTransact(I2C_SLA_SHT21, buf, 2, NULL, 0, 100);
            break;
        default:
            rc = -1;
            break;
    }

    /* Filter positive results from twi actions. */
    if (rc>0) rc = 0;

    /* Free the sensor access */
    NutEventPost(&sht_mutex);

    return rc;
}

/*!
 * \brief Return real sensor value.
 *
 * This function returns either the temperature or the humidity
 * value to a supplied int16_t pointer.
 * The value is decimal with 2 decimal digits:
 * val=2604 -> 26.04°C
 * val=2335 -> 23.35rH
 *
 * \param cmd Either SHT_GET_TEMP or SHT_GET_HUM.
 * \param val Pointer to store the value to.
 *
 * \return -1 if failed else 0.
 */
int ShtRead( uint8_t cmd, int16_t *val)
{
    int rc = -1;
    uint16_t th = 0;

    rc=ShtCommand( cmd, &th);
    if (rc==0) {
        th &= ~0x0003;   /* filter status bits from value */
        if (cmd == SHT_GET_TEMP) {
            /* -46.85 + (175.72 * (St / 2^16)) */
            *val = (int16_t)(T1x100 + (T2x100 * (uint32_t)th / 65536L));
            rc = 0;
        }
        else if (cmd == SHT_GET_HUM) {
            /* -6 + (125 * (Srh / 2^16)) */
            *val = (int16_t)(H1x100 + (H2x100 * (uint32_t)th / 65536L));
            rc = 0;
        }
    }
    return rc;
}

/*!
 * \brief Register and initialize SHT2x sensor device.
 *
 * This function initializes the structures and I2C bus for
 * use with SHT2x sensor device. It checks communication too.
 *
 * \return -1 if init failed else 0.
 */
int ShtInit(void)
{
    int rc = 0;
    uint16_t dat;

    NutEventPost(&sht_mutex);

    rc = ShtCommand(SHT_SOFT_RESET, &dat);
    if (rc<0)
        return rc;

    /* Setup calculation precision */
    rc = ShtCommand( SHT_GET_USER, &dat);
    dat &= ~SHT_RES_MASK;
    dat |= SHT21_PRECISION;
    rc = ShtCommand( SHT_SET_USER, &dat);

    return rc;
}

