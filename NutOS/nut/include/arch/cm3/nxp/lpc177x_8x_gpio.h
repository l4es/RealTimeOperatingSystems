#ifndef _LPC177X_8X_GPIO_H_
#define _LPC177X_8X_GPIO_H_

/*
 * Copyright (C) 2012 by Ole Reinhardt (ole.reinhardt@embedded-it.de)
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

/*
 * \verbatim
 * $Id:$
 * \endverbatim
 */

#define IOCON_FUNC_POS              0
#define IOCON_FUNC_BITMASK          0x00000007

#define IOCON_MODE_POS              3
#define IOCON_MODE_BITMASK          0x00000018

#define IOCON_HYSTERESIS_POS        5
#define IOCON_HYSTERESIS_BITMASK    0x00000020

#define IOCON_INVERT_POS            6
#define IOCON_INVERT_BITMASK        0x00000040

#define IOCON_ADMODE_POS            7
#define IOCON_ADMODE_BITMASK        0x00000080

#define IOCON_GLITCH_FILTER_POS     8
#define IOCON_GLITCH_FILTER_BITMASK 0x00000100

#define IOCON_I2C_MODE_POS          8
#define IOCON_I2C_MODE_BITMASK      0x00000300

#define IOCON_SLEW_POS              9
#define IOCON_SLEW_BITMASK          0x00000200

#define IOCON_ODMODE_POS            10
#define IOCON_ODMODE_BITMASK        0x00000400

#define IOCON_DACEN_POS             16
#define IOCON_DACEN_BITMASK         0x00010000



#define IOCON_MODE_PLAIN            (0 << 3)
#define IOCON_MODE_PULLDOWN         (1 << 3)
#define IOCON_MODE_PULLUP           (2 << 3)
#define IOCON_MODE_REPEATER         (3 << 3)

#define IOCON_HYSTERESIS            (1 << 5)

#define IOCON_INVERTED              (1 << 6)

#define IOCON_ADMODE                (1 << 7)

#define IOCON_GLITCH_FILTER         (1 << 8)

#define IOCON_SLEW                  (1 << 9)

#define IOCON_I2CMODE_FAST          (0 << 8)
#define IOCON_I2CMODE_OPENDRAIN     (1 << 8)
#define IOCON_I2CMODE_FASTPLUS      (2 << 8)
#define IOCON_I2CMODE_HIGHOPENDRAIN (3 << 8)

#define IOCON_ODMODE                (1 << 10)

#define IOCON_DACEN                 (1 << 16)

#endif /* _LPC177X_8X_GPIO_H_ */
