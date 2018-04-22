/*
 * Copyright (C) 2014 by Ole Reinhardt <ole.reinhardt@embedded-it.de>
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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <errno.h>

#include "ctime.h"


/*!
 * \addtogroup xgCrtTime
 * @{
 */

#define ASCTIME_FMT "%.3s %.3s%3d %2.2d:%2.2d:%2.2d %4d\n"

#define ASCTIME_BUF_SIZE    26

static char buf_asctime[ASCTIME_BUF_SIZE];

/*!
 * \brief Convert the broken-down time value tm into a null-terminated string.
 *
 * The output format is the same as ctime() generates.
 * (ISO/IEC 9945-1, ANSI/IEEE Std 1003.1, 2004 Edition)
 *
 * This is the thread safe version of asctime(). It does the same but stores the
 * generated string in a user supplied buffer, which should have root for at
 * least 26 chars.
 *
 * \param timeptr Pointer to structure ::tm where the time is stored
 * \param buf     Pointer to the result buffer. Should have root for at least
 *                26 chars.
 *
 * \return Pointer to the converted string or NULL in case of an error.
 */
char *asctime_r(const struct _tm *timeptr, char *buf)
{
    static const char wday_name[][3] = {
        "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"
    };
    static const char mon_name[][3] = {
        "Jan", "Feb", "Mar", "Apr", "May", "Jun",
        "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
    };
    const char *wn;
    const char *mn;

    if (timeptr == NULL) {
        errno = EINVAL;
        if (buf) {
            strcpy(buf, "??? ??? ?? ??:??:?? ????\n");
        }
        return buf;
    }

    if (timeptr->tm_wday < 0 || timeptr->tm_wday > 6) {
        wn = "???";
    } else {
        wn = wday_name[timeptr->tm_wday];
    }

    if (timeptr->tm_mon < 0 || timeptr->tm_mon > 11) {
        mn = "???";
    } else {
        mn = mon_name[timeptr->tm_mon];
    }

    if (buf) {
        sprintf(buf, ASCTIME_FMT, wn, mn,
            timeptr->tm_mday, timeptr->tm_hour,
            timeptr->tm_min, timeptr->tm_sec,
            timeptr->tm_year + 1900);
    } else {
        errno = EINVAL;
        return NULL;
    }
    return buf;
}

/*!
 * \brief Convert the broken-down time value tm into a null-terminated string.
 *
 * The output format is the same as ctime() generates.
 * (ISO/IEC 9945-1, ANSI/IEEE Std 1003.1, 2004 Edition)
 *
 * The return value points to a statically allocated string which might be
 * overwritten by subsequent calls to asctime.
 *
 * \param timeptr  Pointer to structure ::tm where the time is stored
 *
 * \return Pointer to the converted string or NULL in case of an error.
 */

/*
  Note: This function is *not* thread safe, because it uses a static variable
  to store the calculated values. To be safe, you must surround the call to asctime
  _and_ the usage of the returned pointer with NutEnterCritical() and NutExitCritical()!
  Provided for compatibility to std c lib.
*/
char *asctime(register const struct _tm *timeptr)
{
    return asctime_r(timeptr, buf_asctime);
}

/*!
 * \brief Convert the broken-down localtime value tm into a null-terminated
 *        string.
 *
 * Same as asctime(localtime(t))
 *
 * \param timep Pointer to time_t value where the time is stored
 *
 * \return Pointer to the converted string or NULL in case of an error.
 */

/*
  Note: This function is *not* thread safe, because it uses a static variable
  to store the calculated values. To be safe, you must surround the call to asctime
  _and_ the usage of the returned pointer with NutEnterCritical() and NutExitCritical()!
  Provided for compatibility to std c lib.
*/
char *ctime(const time_t *const timep)
{
	return asctime(localtime(timep));
}


/*!
 * \brief Convert the broken-down localtime value tm into a null-terminated
 *        string.
 *
 * Reentrant version of ctime.
 *
 * \param timep Pointer to time_t value where the time is stored
 *
 * \param buf     Pointer to the result buffer. Should have root for at least
 *                26 chars.
 *
 * \return Pointer to the converted string or NULL in case of an error.
 */
char *ctime_r(const time_t *const timep, char *buf)
{
    struct _tm time;
    localtime_r(timep, &time);
    return asctime_r(&time, buf);
}


/*@}*/
