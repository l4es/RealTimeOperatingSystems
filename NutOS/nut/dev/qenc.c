/*
 * Copyright (C) 2015 Uwe Bonnes (bon@elektron,ikp,physik.tu-darmstadt.de)
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
 * \file dev/qenc.c
 * \brief Hardware independent part Qenc devices.
 *
 * Wrapper functions for easier calling.
 */

/*!
 * \brief Initialize the Pwn device.
 *
 * \param qenc_dev Identifies the Qenc device to initialize.
 * \param reload  Set period of QENC in (prescaled) clock cycles
 * \param prescaler Sets prescaler division factor.
 *
 * \return 0 on success, -1 otherwise, e.g. if reload or prescaler can not
 *              be set to the given function.
 *
 */

#include <stdint.h>
#include <dev/qenc.h>

int QencInit(NUTQENC *qenc_dev)
{
    if (!qenc_dev->QencInit) {
        return -1;
    }
    return qenc_dev->QencInit(qenc_dev);
}

/*!
 * \brief Set value of QENC.
 *
 * Use e.g. to set to zero at some position switch.
 *
 * \param qenc_dev Identifies the Qenc device to set.
 * \param value    Value to set.
 *
 * \return None.
 *
 */
void QencSet(NUTQENC *qenc_dev, int value)
{
    qenc_dev->QencSet(qenc_dev, value);
}

/*!
 * \brief Query value of QENC.
 *
 * \param qenc_dev Identifies the Qenc device to query.
 *
 * \return Value.
 *
 */
int QencGet(NUTQENC *qenc_dev)
{
    return qenc_dev->QencGet(qenc_dev);
}
