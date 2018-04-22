/*
 * Copyright (C) 2006 by egnite Software GmbH. All rights reserved.
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
 * \file dev/rtc.c
 * \brief RTC for Philips PCF8563 clock chip.
 *
 * \verbatim
 *
 * $Log$
 * Revision 1.3  2008/08/11 06:59:42  haraldkipp
 * BSD types replaced by stdint types (feature request #1282721).
 *
 * Revision 1.2  2006/10/08 16:48:09  haraldkipp
 * Documentation fixed
 *
 * Revision 1.1  2006/10/05 17:18:49  haraldkipp
 * Hardware independant RTC layer added.
 *
 *
 * \endverbatim
 */

#include <dev/rtc.h>
#include <time.h>

#include <pro/rfctime.h>
#include <cfg/crt.h>

static NUTRTC *reg_rtc;


/*!
 * \brief Register a specified RTC.
 *
 * This will also initialize any hardware, if required.
 *
 * \param rtc Pointer to the NUTRTC structure.
 *
 * \return 0 on success or -1 in case of an error.
 *
 */
int NutRegisterRtc(NUTRTC * rtc)
{
    reg_rtc = rtc;
    if (rtc && reg_rtc->rtc_init) {
        if ((*reg_rtc->rtc_init) (reg_rtc)) {
            reg_rtc = NULL;
            return -1;
        }
    }
    return 0;
}

/*!
 * \brief Get date and time from the registered RTC.
 *
 * Portable applications should use standard C functions.
 *
 * \param tm Points to a structure that receives the date and time
 *           information.
 *
 * \return 0 on success or -1 in case of an error.
 */
int NutRtcGetTime(struct _tm *tm)
{
    if (reg_rtc && reg_rtc->rtc_gettime && tm) {
        return (*reg_rtc->rtc_gettime) (reg_rtc, tm);
    }
    return -1;
}

/*!
 * \brief Set date and time of the registered RTC.
 *
 * Portable applications should stime().
 *
 * \param tm Points to a structure which contains the date and time
 *           information.
 *
 * \return 0 on success or -1 in case of an error.
 */
int NutRtcSetTime(const struct _tm *tm)
{
    if (reg_rtc && reg_rtc->rtc_settime && tm) {
        return (*reg_rtc->rtc_settime) (reg_rtc, tm);
    }
    return -1;
}

/*!
 * \brief Get alarm date and time from the registered RTC.
 *
 * \param idx    Zero based index. Two alarms are supported.
 * \param tm     Points to a structure that receives the date and time
 *               information.
 * \param aflags Points to an unsigned long that receives the enable flags.
 *
 * \return 0 on success or -1 in case of an error.
 *
 */
int NutRtcGetAlarm(int idx, struct _tm *tm, int *aflags)
{
    if (reg_rtc && reg_rtc->rtc_getalarm) {
        return (*reg_rtc->rtc_getalarm) (reg_rtc, idx, tm, aflags);
    }
    return -1;
}

/*!
 * \brief Set alarm date and time of the registered RTC.
 *
 * \param idx    Zero based index. Two alarms are supported.
 * \param tm     Points to a structure which contains the date and time
 *               information. May be NULL to clear the alarm.
 * \param aflags Each bit enables a specific comparision.
 *               - Bit 0: Seconds
 *               - Bit 1: Minutes
 *               - Bit 2: Hours
 *               - Bit 3: Day of month
 *               - Bit 4: Month
 *               - Bit 7: Day of week (Sunday is zero)
 *
 * \return 0 on success or -1 in case of an error.
 */
int NutRtcSetAlarm(int idx, const struct _tm *tm, int aflags)
{
    if (reg_rtc && reg_rtc->rtc_setalarm) {
        return (*reg_rtc->rtc_setalarm) (reg_rtc, idx, tm, aflags);
    }
    return -1;
}

/*!
 * \brief Query status flags from the registered RTC.
 *
 * \param sflags Points to an unsigned long that receives the status flags.
 *               - Bit 0: Power fail.
 *               - Bit 5: Alarm 0 occured.
 *               - Bit 6: Alarm 1 occured.
 *
 * \return 0 on success or -1 in case of an error.
 */
int NutRtcGetStatus(uint32_t * sflags)
{
    if (reg_rtc && reg_rtc->rtc_getstatus) {
        return (*reg_rtc->rtc_getstatus) (reg_rtc, sflags);
    }
    return -1;
}

/*!
 * \brief Clear status flags of the registered RTC.
 *
 * \param sflags Status flags to clear.
 *
 * \return Always 0.
 */
int NutRtcClearStatus(uint32_t sflags)
{
    if (reg_rtc && reg_rtc->rtc_clrstatus) {
        return (*reg_rtc->rtc_clrstatus) (reg_rtc, sflags);
    }
    return -1;
}

HANDLE *NutRtcGetHandle(void)
{
    if (!reg_rtc)
        return NULL;
    return &reg_rtc->alarm;
}

/* Function for fall back dummy RTC device*/

/*!
 * \brief Get date and time from an system time
 *
 * \param tm Points to a structure that receives the date and time
 *           information.
 *
 * \return 0 on success or -1 in case of an error.
 */
int RtcNullGetClock(NUTRTC *rtc, struct _tm *tm)
{
    if (tm) {
        time_t now;
        now = time(NULL);
        gmtime_r(&now, tm);
        return 0;
    }
    return -1;
}

/*!
 * \brief Set system time from time string.
 *
 * \param tm Points to a structure that receives the date and time
 *           information.
 *
 * \return 0 on success or -1 in case of an error.
 */
int RtcNullSetClock(NUTRTC *rtc, const struct _tm *tm)
{
    time_t now;

    now = RfcTimeParse("Unk, " __DATE__ " " __TIME__);
    now -= (CRT_TIMEZONE - CRT_DAYLIGHT) * 60;
    stime(&now);
    return 0;
}

NUTRTC rtcNull = {
    .rtc_gettime   =  RtcNullGetClock,
    .rtc_settime   =  RtcNullSetClock,
};
