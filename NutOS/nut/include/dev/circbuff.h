#ifndef CIRCBUFF_H_
#define CIRCBUFF_H_

/*
 * Copyright (C) 2012 by egnite GmbH
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

#include <toolchain.h>
#include <sys/event.h>
#include <stdint.h>
#include <stdlib.h>

/*!
 * \defgroup xgCircBuff Circular Buffer
 * \ingroup xgDevSerial
 * \anchor xrCircBuff
 * \brief Generic circular buffer routines.
 */
/*@{*/

#ifndef CIRCBUFF_MAX
/*!
 * \brief Maximum circular buffer size.
 *
 * On 8-bit targets this may be configured to less than 256,
 * in which case the buffer indices will fit into one byte.
 *
 * See \ref _CIRCBUFF.
 */
#define CIRCBUFF_MAX   16383
#endif

#if CIRCBUFF_MAX <= 255
typedef uint_fast8_t cb_size_t;
#elif CIRCBUFF_MAX <= 16383
typedef uint_fast16_t cb_size_t;
#else
typedef uint_fast32_t cb_size_t;
#endif

/*!
 * \brief Circular buffer structure type.
 *
 * See \ref _CIRCBUFF.
 */
typedef struct _CIRCBUFF CIRCBUFF;

/*!
 * \brief Circular buffer structure.
 *
 * This circular buffer implementation is intended to be used by
 * device drivers, specifically UART drivers.
 */
struct _CIRCBUFF {
    /*! \brief Pointer to the buffer. */
    uint8_t *cb_buff;
    /*! \brief Buffer size. */
    cb_size_t cb_size;
    /*! \brief Next position to read from. */
    cb_size_t cb_ridx;
    /*! \brief Next position to write to. */
    cb_size_t cb_widx;
};

/*!
 * \brief Get the number of consecutive bytes available for writing.
 *
 * The caller must make sure, that this function has exclusive access
 * to the buffer.
 *
 * \param cb Pointer to a \ref CIRCBUFF structure.
 *
 * \return The number of bytes available.
 */
static NUT_INLINE_FUNC size_t CircBuffWriteSize(CIRCBUFF *cb)
{
    size_t rc;

    if (cb->cb_ridx <= cb->cb_widx) {
        rc = cb->cb_size - cb->cb_widx;
        rc += cb->cb_ridx != 0;
    } else {
        rc = cb->cb_ridx - cb->cb_widx - 1;
    }
    return rc;
}

/*!
 * \brief Get the number of consecutive bytes available for reading.
 *
 * \param cb   Pointer to a \ref CIRCBUFF structure.
 *
 * \return The number of bytes available.
 */
static NUT_INLINE_FUNC size_t CircBuffReadSize(CIRCBUFF *cb)
{
    size_t rc;

    if (cb->cb_ridx > cb->cb_widx) {
        rc = cb->cb_size - cb->cb_ridx + 1;
    } else {
        rc = cb->cb_widx - cb->cb_ridx;
    }
    return rc;
}

/*!
 * \brief Reset a circular buffer.
 *
 * The caller must make sure, that this function has exclusive access
 * to the buffer.
 *
 * \param cb   Pointer to a \ref CIRCBUFF structure.
 * \param size Requested size of the buffer. The actual buffer size
 *             will be 2^n - 1, where n = 5 to 32. However, the maximum
 *             size is globally limited by \ref CIRCBUFF_MAX, which is
 *             16383 by default.
 *
 * \return If not enough memory is available, -1 is returned. Otherwise
 *         the return value will be 0, which indicates, that the buffer
 *         can be used. In this case it might be possible, that the buffer
 *         is actually smaller than requested.
 */
extern int CircBuffReset(CIRCBUFF *cb, size_t size);

/*@}*/

#endif
