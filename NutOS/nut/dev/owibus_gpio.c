/*
 * Copyright (C) 2013, 2016-2017 by Uwe Bonnes
 *                              (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * \file dev/owibus_gpio.c
 * \brief OWI bus for GPIO code include file.
 *
 * \verbatim
 * $Id: owibus_gpio.c 6593 2017-02-15 15:35:39Z u_bonnes $
 * \endverbatim
 */

/*!
 * \addtogroup xgOwibusBb
 */
/*@{*/

#include <sys/atom.h>
/*!
 * \brief Perform One-Wire transaction.
 *
 * \param bus     Specifies the One-Wire bus.
 * \param command Either OWI_CMD_RESET or OWI_CMD_RWBIT.
 * \param value   The value to send.
 *
 * \return The value read on success, a negative value otherwise.
 */
static int Gpio_OwiTransaction(NUTOWIBUS *bus, int_fast8_t command, int_fast8_t value)
{
    int res = 0;
    int16_t delay1;
    int16_t delay2;
    int16_t delay3 =
        (owi_timervalues_250ns[bus->mode & OWI_OVERDRIVE][command][OWI_PHASE_RELEASE] -
         owi_timervalues_250ns[bus->mode & OWI_OVERDRIVE][command][OWI_PHASE_RW]) >> 2;

    if (value) {
        delay1 =
            (owi_timervalues_250ns[bus->mode & OWI_OVERDRIVE][command][OWI_PHASE_SYNC_PULSE] -
             owi_timervalues_250ns[bus->mode & OWI_OVERDRIVE][command][OWI_PHASE_SETUP]) >> 2;
        delay2 =
            (owi_timervalues_250ns[bus->mode & OWI_OVERDRIVE][command][OWI_PHASE_RW] -
             owi_timervalues_250ns[bus->mode & OWI_OVERDRIVE][command][OWI_PHASE_SYNC_PULSE]) >> 2;
    }
    else {
        delay1 =
            (owi_timervalues_250ns[bus->mode & OWI_OVERDRIVE][command][OWI_PHASE_SYNC_PULSE_LOW] -
             owi_timervalues_250ns[bus->mode & OWI_OVERDRIVE][command][OWI_PHASE_SETUP]) >> 2;
        delay2 =
            (owi_timervalues_250ns[bus->mode & OWI_OVERDRIVE][command][OWI_PHASE_RELEASE] -
             owi_timervalues_250ns[bus->mode & OWI_OVERDRIVE][command][OWI_PHASE_SYNC_PULSE_LOW]) >> 2;
    }

    /* Be nice! Allow other thing to happen now before we block
     * cooperative multitasking for up to 480 us
     */
    NutSleep(0);
    NutEnterCritical();
    OWI_LO();
    NutMicroDelay(delay1);
    OWI_HI();
    if (value) {
        NutMicroDelay(delay2);
        res = OWI_GET();
        NutMicroDelay(delay3);
    } else {
        NutMicroDelay(delay2);
    }
    NutExitCritical();
    return res;
}

/*!
 * \brief Reset the One-Wire bus and check if device(s) present.
 *
 * \param bus Specifies the One-Wire bus.
 *
 * \return OWI_SUCCESS on success, a negative value otherwise.
 */
static int Gpio_OwiTouchReset(NUTOWIBUS *bus)
{
    return Gpio_OwiTransaction(bus, OWI_CMD_RESET, 1);
}

/*!
 * \brief Exchange one bit on the One-Wire bus.
 *
 * \param bus Specifies the One-Wire bus.
 * \param bit Value for the bit to send.
 *
 * \return The bus state at the read slot on success, a negative value
 *         otherwise.
 */
static int Gpio_OwiRWBit(NUTOWIBUS *bus, uint_fast8_t bit)
{
    return Gpio_OwiTransaction(bus, OWI_CMD_RWBIT, bit);
}

/*!
 * \brief Write a block of data bits to the One-Wire bus.
 *
 * \param bus  Specifies the One-Wire bus.
 * \param data Data bits to send.
 * \param len  Number of bits to send.
 *
 * \return OWI_SUCCESS on success, a negative value otherwise.
 */
static int Gpio_OwiWriteBlock(NUTOWIBUS *bus, const uint8_t *data,
                              uint_fast8_t len)
{
    int res;
    int i;

    for (i = 0; i < len; i++) {
        res = Gpio_OwiRWBit(bus, data[i >> 3] & (1 << (i & 0x7)));
        if (res < 0)
            return OWI_HW_ERROR;
    }
    return OWI_SUCCESS;
}

/*!
 * \brief Read a block of data bits from the One-Wire bus.
 *
 * \param bus  Specifies the One-Wire bus.
 * \param data Data bits received.
 * \param len  Number of bits to read.
 *
 * \return OWI_SUCCESS on success, a negative value otherwise.
 */
static int Gpio_OwiReadBlock(NUTOWIBUS *bus, uint8_t *data, uint_fast8_t len)
{
    int res;
    int i;

    memset(data, 0, (len >> 3) + 1);
    for (i = 0; i < len; i++) {
        res = Gpio_OwiRWBit(bus, 1);
        if (res < 0)
            return OWI_HW_ERROR;
        data[i >> 3] |= (res << (i & 0x7));
    }
    return OWI_SUCCESS;
}

/*!
 * \brief Read a block of data bits from the One-Wire bus.
 *
 * \param bus  Specifies the One-Wire bus.
 * \param data Data bits received.
 * \param len  Number of bits to read.
 *
 * \return OWI_SUCCESS on success, a negative value otherwise.
 */
static int Gpio_Setup(NUTOWIBUS *bus)
{
    (void) bus;

#if !defined(OWI_HW)
    return OWI_INVALID_HW;
#else
    OWI_INIT();
    return OWI_SUCCESS;
#endif
}

/*@}*/
