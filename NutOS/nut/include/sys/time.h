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

#ifndef _SYS_TIME_H_
#define _SYS_TIME_H_

#include <cfg/os.h>
#include <time.h>

/*!
 * \addtogroup xgCrtTime
 * @{
 */

/*!
 * \brief Timezone structure type.
 *
 * \note Obsolete and should not be used, but is needed for historic reasons
 *
 */
struct timezone {
	int tz_minuteswest;		/* Minutes west of GMT.  */
	int tz_dsttime;			/* Non zero in case of dailight saving time (summer time)  */
};

/*!
 * \brief structure to store a timestamp with microseconds resolution
 */

struct timeval {
    int32_t tv_sec;         /* seconds */
    int32_t tv_usec;        /* microseconds */
};

extern int gettimeofday (struct timeval *tv, struct timezone *tz);
extern int settimeofday (struct timeval *tv, struct timezone *tz);

/* Convenience macros to manipulate or check timevals */


/*!
 * \brief Check if a timeval value is != 0
 *
 * \param tvp		Pointer to timeval struct
 *
 * \return      true is timeval is != 0, false if not
 *
 */
#define timerisset(tvp)    ((tvp)->tv_sec || (tvp)->tv_usec)

/*!
 * \brief Clear a timeval (set both entries to 0)
 *
 * \param tvp		Pointer to timeval struct
 *
 */
#define timerclear(tvp)    ((tvp)->tv_sec = (tvp)->tv_usec = 0)


/*!
 * \brief Compare two timevals
 *
 * \note  does not work for >= or <=
 *
 * \param a		Pointer to first timeval struct
 * \param b		Pointer to second timeval struct
 * \param cmp	Operand to use for comparision
 *
 * \returns     comparision result
 */
#define timercmp(a, b, cmp)             \
    (((a)->tv_sec == (b)->tv_sec) ?     \
     ((a)->tv_usec cmp (b)->tv_usec) :  \
     ((a)->tv_sec cmp (b)->tv_sec))

/*!
 * \brief Add two timevals and return as res
 *
 * \param a		Pointer to first timeval struct
 * \param b		Pointer to second timeval struct
 * \param res	Pointer to result timeval struct
 */
#define timeradd(a, b, res)                           \
    {                                                 \
        (res)->tv_sec = (a)->tv_sec + (b)->tv_sec;    \
        (res)->tv_usec = (a)->tv_usec + (b)->tv_usec; \
        if ((res)->tv_usec >= 1000000) {              \
            (res)->tv_sec++;                          \
            (res)->tv_usec -= 1000000;                \
        }                                             \
    }

/*!
 * \brief Substract two timevals and return as res
 *
 * \param a		Pointer to first timeval struct
 * \param b		Pointer to second timeval struct
 * \param res	Pointer to result timeval struct
 */
#define timersub(a, b, res)                           \
    {                                                 \
        (res)->tv_sec = (a)->tv_sec - (b)->tv_sec;    \
        (res)->tv_usec = (a)->tv_usec - (b)->tv_usec; \
        if ((res)->tv_usec < 0) {                     \
            (res)->tv_sec--;                          \
            (res)->tv_usec += 1000000;                \
        }                                             \
    }

/*@}*/

#endif

