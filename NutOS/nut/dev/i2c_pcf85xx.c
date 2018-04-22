/*
 * Copyright (C) 2012 by egnite GmbH
 * Copyright (C) 2006 by egnite Software GmbH
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
 * \file dev/pcf85xx.c
 * \brief RTC for NXP PCF85XX clock chips.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <sys/event.h>
#include <sys/timer.h>

#include <time.h>

#include <dev/i2cbus.h>
#include <dev/i2c_pcf85xx.h>

/*!
 * \addtogroup xgI2cPcf85xx
 */
/*@{*/

#ifndef I2C_SLA_RTC
#define I2C_SLA_RTC     0x51
#endif

static uint32_t rtc_status;

/*!
 * \brief Get date and time from an PCF85XX hardware clock.
 *
 * \param rtc Specifies the RTC device.
 * \param tm  Points to a structure that receives the date and time
 *            information.
 *
 * \return 0 on success or -1 in case of an error.
 */
static int I2cPcfGetClock(NUTRTC *rtc, struct _tm *tm)
{
    uint8_t data[7];

    data[0] = 2;
    if (NutI2cMasterTransceive(rtc->dcb, data, 1, data, 7) != 7) {
        return -1;
    }
    tm->tm_sec = BCD2BIN(data[0] & 0x7F);
    tm->tm_min = BCD2BIN(data[1] & 0x7F);
    tm->tm_hour = BCD2BIN(data[2] & 0x3F);
    tm->tm_mday = BCD2BIN(data[3] & 0x3F);
    tm->tm_mon = BCD2BIN(data[5] & 0x1F) - 1;
    tm->tm_year = BCD2BIN(data[6]);
    if (data[5] & 0x80) {
        tm->tm_year += 100;
    }
    tm->tm_wday = data[4] & 0x07;

    return 0;
}

/*!
 * \brief Set an PCF85XX hardware clock.
 *
 * New time will be taken over at the beginning of the next second.
 *
 * \param rtc Specifies the RTC device.
 * \param tm  Points to a structure which contains the date and time
 *            information.
 *
 * \return 0 on success or -1 in case of an error.
 */
static int I2cPcfSetClock(NUTRTC *rtc, const struct _tm *tm)
{
    uint8_t data[8];

    data[0] = 2;
    data[1] = BIN2BCD(tm->tm_sec);
    data[2] = BIN2BCD(tm->tm_min);
    data[3] = BIN2BCD(tm->tm_hour);
    data[4] = BIN2BCD(tm->tm_mday);
    data[5] = tm->tm_wday;
    data[6] = BIN2BCD(tm->tm_mon + 1);
    if (tm->tm_year > 99) {
        data[7] = BIN2BCD(tm->tm_year - 100);
        data[6] |= 0x80;
    }
    else {
        data[7] = BIN2BCD(tm->tm_year);
    }
    return NutI2cMasterTransceive(rtc->dcb, data, 8, NULL, 0);
}

/*!
 * \brief Get alarm date and time of an PCF85XX hardware clock.
 *
 * Not implemented.
 *
 * \param idx   Zero based index. Two alarms are supported.
 * \param tm    Points to a structure that receives the date and time
 *              information.
 * \param aflgs Points to an unsigned long that receives the enable flags.
 *
 * \return 0 on success or -1 in case of an error.
 *
 */
static int I2cPcfGetAlarm(NUTRTC *rtc, int idx, struct _tm *tm, int *aflgs)
{
    return -1;
}

/*!
 * \brief Set alarm of an PCF85XX hardware clock.
 *
 * Not implemented.
 *
 * \param idx   Zero based index. Two alarms are supported.
 * \param tm    Points to a structure which contains the date and time
 *              information. May be NULL to clear the alarm.
 * \param aflgs Each bit enables a specific comparison.
 *              - Bit 0: Seconds
 *              - Bit 1: Minutes
 *              - Bit 2: Hours
 *              - Bit 3: Day of month
 *              - Bit 4: Month
 *              - Bit 7: Day of week (Sunday is zero)
 *
 * \return 0 on success or -1 in case of an error.
 */
static int I2cPcfSetAlarm(NUTRTC *rtc, int idx, const struct _tm *tm, int aflgs)
{
    return -1;
}

/*!
 * \brief Query RTC status flags.
 *
 * \param rtc   Specifies the RTC device.
 * \param sflgs Points to an unsigned long that receives the status flags.
 *              - Bit 0: Power fail.
 *              - Bit 5: Alarm 0 occured (not implemented).
 *              - Bit 6: Alarm 1 occured (not implemented).
 *
 * \return 0 on success or -1 in case of an error.
 */
static int I2cPcfGetStatus(NUTRTC *rtc, uint32_t *sflgs)
{
    uint8_t data;

    data = 2;
    if (NutI2cMasterTransceive(rtc->dcb, &data, 1, &data, 1) != 1) {
        return -1;
    }
    if (data & 0x80) {
        rtc_status |= RTC_STATUS_PF;
    }
    *sflgs = rtc_status;

    return 0;
}

/*!
 * \brief Clear RTC status flags.
 *
 * \param rtc   Specifies the RTC device.
 * \param sflgs Status flags to clear.
 *
 * \return Always 0.
 */
static int I2cPcfClearStatus(NUTRTC *rtc, uint32_t sflgs)
{
    rtc_status &= ~sflgs;

    return 0;
}

/*!
 * \brief Initialize the interface to a PCF85XX hardware clock.
 *
 * \return 0 on success or -1 in case of an error.
 */
static int I2cPcfInit(NUTRTC *rtc)
{
    uint32_t tmp;

    return I2cPcfGetStatus(rtc, &tmp);
}

/*!
 * \brief I2C driver control block.
 */
static NUTI2C_SLAVE i2cPcf85xx = {
    NULL,
    I2C_SLA_RTC,
    100,
    NULL
};

/*!
 * \brief PCF85XX RTC driver information structure.
 *
 * This driver has been tested with the PCF8563, but may also work
 * with PCF8564, PCF8565, PCF8583 and PCF8593 chips. It will not
 * work with PFC8523.
 *
 * Two steps are required to register this driver. First, we must
 * attach the I2C slave to an I2C bus by calling
 * /code
 * NutRegisterI2cSlave((NUTI2C_SLAVE *) rtcI2cPcf85xx.dcb, &i2cBus);
 * /endcode
 * In a second step the driver must be registered as the system's
 * real time clock by calling
 * /code
 * NutRegisterRtc(&rtcI2cPcf85xx);
 * /endcode
 */
NUTRTC rtcI2cPcf85xx = {
    &i2cPcf85xx,        /*!< Driver control block */
    I2cPcfInit,         /*!< Hardware initialization, rtc_init */
    I2cPcfGetClock,     /*!< Read date and time, rtc_gettime */
    I2cPcfSetClock,     /*!< Set date and time, rtc_settime */
    I2cPcfGetAlarm,     /*!< Read alarm date and time, rtc_getalarm */
    I2cPcfSetAlarm,     /*!< Set alarm date and time, rtc_setalarm */
    I2cPcfGetStatus,    /*!< Read status flags, rtc_getstatus */
    I2cPcfClearStatus,  /*!< Clear status flags, rtc_clrstatus */
    NULL                /*!< Handle for alarm event queue, not supported right now */
};

/*@}*/
