#if !defined(_DEV_QENC_H_)
#define      _DEV_QENC_H_

/*
 * Copyright (C) 2015 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * \file dev/qenc.h
 * \brief Abstraction for Quadrature encoder counters.
 */

/*!
 * \addtogroup xgQenc
 */
/*@{*/

/*!
 * Qenc Device structure.
 *
 */
typedef struct _NUTQENC NUTQENC;

/*!
 * \brief Device structure.
 *
 */
struct _NUTQENC {
    /*!
     * \brief Generic container for device configuration and data
     */
    uintptr_t qenc_info;
    /*!
     * \brief Driver initialization routine.
     *
     * This routine is called during device registration.
     */
    int       (*QencInit) (NUTQENC *qenc_dev);
    /*!
     * \brief Set Qenc value.
     *
     * Used to modify Qenc settings.
     */
    void      (*QencSet) (NUTQENC *qenc_dev, int value);
    /*!
     * \brief Get  value.
     *
     * Used to query Qenc settings.
     */
    int (*QencGet) (NUTQENC *qenc_dev);
};

extern int QencInit(NUTQENC *qenc_dev);
extern void QencSet(NUTQENC *qenc_dev, int value);
extern int QencGet(NUTQENC *qenc_dev);

/* List of implemented QENC devices */
extern NUTQENC  Stm32Qenc0;
#endif
