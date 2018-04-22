/*
 * Copyright (C) 2001-2006 by egnite Software GmbH
 * Copyright (C) 2009 by egnite GmbH
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
 * $Id: dataflash.c 4115 2012-04-12 21:06:13Z olereinhardt $
 *
 * WARNING! Do not use any part of Basemon for your own applications. WARNING!
 *
 * This is not a typical application sample. It overrides parts of Nut/OS to
 * keep it running on broken hardware.
 */

#include <stdint.h>

#include "dataflash.h"

#if defined (__AVR__)
/*!
 * \brief Exchange SPI byte.
 *
 * Inlined?
 */
static uint8_t SpiByte(uint8_t c)
{
    outb(SPDR, c);
    while ((inb(SPSR) & 0x80) == 0);

    return inb(SPDR);
}

/*!
 * \brief Init SPI interface.
 *
 * Should be more general (Dataflash, VS1001 etc.)
 * Dataflash requires SPI mode 3 (enable flash when SCK is high)
 *
 * - PB1(SCK) -> Mem-SCK
 * - PB2(MOSI) -> Mem-SI
 * - PB3(MISO) <- Mem-SO
 * - PB4 -> Mem-CS (through inverter)
 *
 */
static void SpiInit(void)
{
    /*
     * Init SS pin. When configured as input, we will lose master
     * mode, if this pin goes low. Thus we switch on the pull-up.
     */
    if(bit_is_clear(DDRB, 0)) {
        sbi(PORTB, 0);
    }

    /* Set SCK output. */
    sbi(DDRB, 1);
    /* Set MOSI output. */
    sbi(DDRB, 2);
    /* Enable MISO pullup. */
    sbi(PORTB, 3);

#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega2561__)
    /* Set double speed. */
    outb(SPSR, _BV(SPI2X));
#endif

    /* Enable SPI master mode 3, fosc/2. */
    outb(SPCR, _BV(SPE) | _BV(MSTR) | _BV(CPHA) | _BV(CPOL));

    /* Clean-up the status. */
    outb(SPSR, inb(SPSR));
    inb(SPDR);
}

/*!
 * \brief Read memory chip status.
 */
static uint8_t SpiMemStatus(uint8_t cs)
{
    uint8_t rc;

    if (cs == SPIMEM_CS_BIT) {
        sbi(SPIMEM_CS_PORT, cs);
    } else {
        cbi(SPIMEM_CS_PORT, cs);
    }
    SpiByte(0x57);
    rc = SpiByte(0);

    if (cs == SPIMEM_CS_BIT) {
        cbi(SPIMEM_CS_PORT, cs);
    } else {
        sbi(SPIMEM_CS_PORT, cs);
    }

    return rc;
}

static int SpiMemInit(uint8_t cs, uint16_t *pages, uint16_t *pagesize)
{
    uint8_t fs;

    /* Init SPI memory chip select. */
    if (cs == SPIMEM_CS_BIT) {
        cbi(SPIMEM_CS_PORT, cs);
    } else {
        sbi(SPIMEM_CS_PORT, cs);
    }
    sbi(SPIMEM_CS_DDR, cs);

    /* Initialize the SPI interface. */
    SpiInit();

    /* Read the status register for a rudimentary check. */
    fs = SpiMemStatus(cs);
    if(fs & 0x80) {
        fs = (fs >> 2) & 0x0F;
        *pagesize = 264;
        if(fs == 3) {
            *pages = 512;
            return 0;
        }
        else if(fs == 5) {
            *pages = 1024;
            return 0;
        }
        else if(fs == 7) {
            *pages = 2048;
            return 0;
        }
        else if(fs == 13) {
            *pagesize = 528;
            *pages = 8192;
            return 0;
        }
    }
    return -1;
}

#endif  /* __AVR__ */

/*
 * \return Number of bytes.
 */
long SpiMemTest(void)
{
    uint16_t pages = 0;
    uint16_t pagesize = 0;

#if defined (__AVR__)
    if (SpiMemInit(SPIMEM_CS_BIT, &pages, &pagesize)) {
        SpiMemInit(SPIMEM_CS_BIT_ALT, &pages, &pagesize);
    }

    /* Disable SPI */
    outb(SPCR, 0);
#endif  /* __AVR__ */

    return (long)pages * (long)pagesize;
}
