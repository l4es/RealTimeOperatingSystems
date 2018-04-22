#ifndef DEV_PPP_HDLC_H_
#define DEV_PPP_HDLC_H_

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

#include <sys/device.h>

/*!
 * \defgroup xgPppHdlc PPP AHDLC driver
 * \ingroup xgDevSerial
 * \brief Generic PPP AHDLC UART driver.
 *
 * This new driver and all related drivers had been partly tested
 * by running PPP on the Ethernut 3.1D reference board, using a new
 * \ref xrUsartCb "driver frame". No other tests had been done. This
 * early code is incomplete and probably buggy.
 *
 * The original AHDLC driver had been implemented as a full UART driver
 * to provide maximum performance for the 8 bit AVR family. Unfortunately,
 * later implementations for newly supported 32 bit targets had been done
 * in the same way. Over the time several drivers were created, which do
 * not only duplicate the code of the original driver, but also duplicate
 * code of the non-HDLC UART drivers. Consequently, as HDLC is not often
 * used, these drivers rarely catch up with the latest changes in the
 * generic UART drivers.
 *
 * While compatible with previous AHDLC drivers in general, this new
 * implementation is done in a different way. Instead of implementing a
 * full UART device driver, it attaches itself to an existing driver,
 * adding HDLC capability to this driver. Applications, who want to use
 * this new driver, needs to be slightly modified: They must register
 * a standard UART driver before registering the HDLC driver. In addition,
 * this driver's name differs from the UART driver ("luartx" instead of
 * "uartx"), which will require a change in the PPP open call.
 */
/*@{*/

/*!
 * \brief PPP HDLC device attached to "uart0".
 */
extern NUTDEVICE devPppHdlc0;

/*!
 * \brief PPP HDLC device attached to "uart1".
 */
extern NUTDEVICE devPppHdlc1;

/*@}*/
#endif
