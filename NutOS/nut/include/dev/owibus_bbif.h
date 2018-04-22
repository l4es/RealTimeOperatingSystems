#ifndef _OWI_BB_H_
#define _OWI_BB_H_

/*
 * Copyright (C) 2012 by Uwe Bonnes(bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * \file dev/owibus_uartif.c
 * \brief Header for the One-Wire API bitbang Implementation
 *
 * \verbatim
 * $Id: owibus_bbif.h 4424 2012-08-17 12:12:06Z haraldkipp $
 * \endverbatim
 */

/*!
 * \brief Array of delay values for the different command phases.
 */
#include <stdint.h>
#include <sys/types.h>

/*!
 * \addtogroup xgOwibusBb
 */
/*@{*/

extern const uint16_t owi_timervalues_250ns[OWI_MODE_NONE][OWI_CMD_NONE][OWI_PHASE_NONE];

/*!
 * \brief OWI runtime controlblock container.
 *
 * This is installed in heap at initialization.
 */
struct _NUTOWIINFO_BB {
    int txrx_port;
    int txrx_pin;
    int pp_port;
    int pp_pin;
};

typedef struct _NUTOWIINFO_BB NUTOWIINFO_BB;

int NutRegisterOwiBus_BB(NUTOWIBUS *bus, int tx_port, uint_fast8_t tx_pin, int pullup_port, uint_fast8_t pullup_pin);

/*@}*/

#endif
