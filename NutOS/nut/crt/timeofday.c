/*
 * Copyright (C) 2013 Ole Reinhardt <ole.reinhardt@embedded-it.de>
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

/*
 * \verbatim
 * $Id: $
 * \endverbatim
 */

#include <stdint.h>

#include <cfg/os.h>
#include <time.h>
#include <sys/time.h>
#include <sys/timer.h>

/*!
 * \addtogroup xgCrtTime
 *
 */
/*@{*/


/*!
 * \brief Get current time and timezone
 *
 * \note  This function returns the software clock system time and does not
 *        read the RTC. In case you want to get the time from RTC, you have to call
 *        NutRtcGetTime manually.
 *
 * \param tv    Pointer to timeval struct which shall be filled with the current time
 * \param tz    Pointer to timezone struct. Should be NULL, as using this values is
 *              deprecated
 *
 * \return      0 on success and -1 in case of an error
 *
 */
int gettimeofday (struct timeval *tv, struct timezone *tz)
{
    if (tv != NULL) {
        /* Take system time, add the offset to epoc and place the result in tv */
#ifndef NUT_USE_OLD_TIME_API
        timeradd(&system_time, &epo_offs, tv);
#else
        tv->tv_sec  = time(NULL);
        tv->tv_usec = (NutGetMillis() % 1000) * 1000;
#endif
    }

    if (tz != NULL) {
        /* Set timezone information. NOTE: struct timezone is obsolete and should not be used. */
        tz->tz_minuteswest = _timezone / 60;
        tz->tz_dsttime = _daylight;
    }
    return 0;
}


/*!
 * \brief Set current time and timezone
 *
 * \note  This function does not automatically update the RTC, you have to
 *        call NutRtcSetTime manually.
 *
 *        This function is not supported in the old clock API and will always return -1
 *
 * \param tv    Pointer to timeval struct which holds the current time
 * \param tz    Pointer to timezone struct. Should be NULL, as using this values is
 *              deprecated
 *
 * \return      0 on success and -1 in case of an error
 *
 */
int settimeofday (struct timeval *tv, struct timezone *tz)
{
#ifndef NUT_USE_OLD_TIME_API
    if (tv != NULL) {
        /* Substract tv from system time. The result is the offset to epoc. Place into epo_offs */
        timersub(tv, &system_time, &epo_offs);
    }

    if (tz != NULL) {
        /* Set timezone information. NOTE: struct timezone is obsolete and should not be used. */
        _timezone = tz->tz_minuteswest * 60;
        _daylight = tz->tz_dsttime;
    }
    return 0;
#else
    /* This function is not supported in the old clock API */
    return -1;
#endif
}

/*@}*/
