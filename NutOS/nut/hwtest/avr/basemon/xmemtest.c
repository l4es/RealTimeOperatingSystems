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
 * $Id: xmemtest.c 3597 2011-10-21 16:17:18Z haraldkipp $
 *
 * WARNING! Do not use any part of Basemon for your own applications. WARNING!
 *
 * This is not a typical application sample. It overrides parts of Nut/OS to
 * keep it running on broken hardware.
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "utils.h"
#include "uart.h"
#include "xmemtest.h"

extern uint8_t GetChar(void);

#define MERR_BAD_ADDR       1
#define MERR_BAD_DATA       2
#define MERR_STICKY_WRITE   3
#define MERR_STICKY_READ    4
#define MERR_STICKY_ALE     5
#define MERR_STICKY_DATABUS 6
#define MERR_STICKY_ADDRBUS 7
#define MERR_SHORT_DATABUS  8
#define MERR_SHORT_ADDRBUS  9

static int merr;
static uint8_t merr_bits;
static void *merr_addr;

/* PORTA: AD0..7 */
/* PORTC: A8..15 */
/*
 * PG0: \WR
 * PG1: \RD
 * PG2: ALE
 * PG3: TOSC2
 * PG4: TOSC1
 */

/*!
 * \brief Disable external memory interface.
 *
 * On return, control bus signals are high and address
 * and data bus are low.
 */
void XMemDisable(void)
{
#if defined (__AVR__)
    /* Disable external memory bus. */
#if defined(__AVR_AT90CAN128__) || defined(__AVR_ATmega2561__)
    cbi(XMCRA, SRE);
#else
    cbi(MCUCR, SRE);
#endif

#ifdef __AVR_ENHANCED__
    /* Disable write signal. */
    sbi(PORTG, PORTG0);
    sbi(DDRG, PORTG0);

    /* Disable read signal. */
    sbi(PORTG, PORTG1);
    sbi(DDRG, PORTG1);

    /* Set ALE high, which makes latch transparent. */
    sbi(PORTG, PORTG2);
    sbi(DDRG, PORTG2);
#endif

    /* Set latch register to zero. */
    outb(PORTA, 0);
    outb(DDRA, 0xFF);
#ifdef __AVR_ENHANCED__
    cbi(PORTG, PORTG2);
    sbi(PORTG, PORTG2);
#endif

    /* Set high address bus to zero. */
    outb(PORTC, 0);
#ifdef __AVR_ENHANCED__
    outb(DDRC, 0xFF);
#endif
#endif /* __AVR__ */
}

#ifdef __AVR_ENHANCED__

void XMemLatch(uint8_t val)
{
    /* Port A is connected to latch inputs. */
    outb(PORTA, val);
    outb(DDRA, 0xFF);

    /* Falling edge will set the latch. */
    cbi(PORTG, PORTG2);
}

void MemWrite(uintptr_t addr, ureg_t data)
{
    /* Set high address byte. */
    outb(PORTC, addr >> 8);

    /* Set low address byte. */
    outb(PORTA, (uint8_t)addr);

    /* ALE falling */
    cbi(PORTG, PORTG2);

    /* Set data byte. */
    outb(PORTC, data);

    /* Low pulse on write. */
    cbi(PORTG, PORTG0);
    cbi(PORTG, PORTG0);

    /* If ALE is high, then latch is transparent. */
    sbi(PORTG, PORTG2);
}

ureg_t MemRead(uintptr_t addr)
{
    uint8_t rc;

    /* Set high address byte. */
    outb(PORTC, addr >> 8);

    /* Set low address byte. */
    outb(PORTC, (uint8_t)addr);

    /* ALE falling */
    cbi(PORTG, PORTG2);

    /* Read signal falling. */
    cbi(PORTG, PORTG1);

    /* Read data byte. */
    rc = inb(PORTC);

    /* Read signal rising. */
    sbi(PORTG, PORTG1);

    /* If ALE is high, then latch is transparent. */
    sbi(PORTG, PORTG2);

    return rc;
}

/*!
 * \brief Detect sticky memory bus bits and shortcuts.
 */
int XMemTestBus(void)
{
    uint8_t wb;

    /*
     * Disable external memory bus. This will set the control bus
     * signals high, clear the address latch and set all data and
     * address bus lines to low.
     */
    XMemDisable();

    /*
     * First check for sticky control bus signals. We switch them
     * to inputs, but they should be kept high because the pullup
     * is enabled. If they become low, something is wrong.
     */
    cbi(DDRG, PORTG0);
    if((inb(PING) & _BV(PORTG0)) != _BV(PORTG0)) {
        merr = MERR_STICKY_WRITE;
        return -1;
    }
    sbi(DDRG, PORTG0);

    cbi(DDRG, PORTG1);
    if((inb(PING) & _BV(PORTG0)) != _BV(PORTG0)) {
        merr = MERR_STICKY_READ;
        return -1;
    }
    sbi(DDRG, PORTG1);

    cbi(DDRG, PORTG2);
    if((inb(PING) & _BV(PORTG0)) != _BV(PORTG0)) {
        merr = MERR_STICKY_ALE;
        return -1;
    }

    /*
     * Check the data bus for any sticky bits. Note, that ALE is
     * still low, so the latch contents (all zero) will appear on
     * the high byte address bus lines. We switch AD0..AD7 into
     * input mode, enable the pullups and expect all line kept
     * high.
     */
    outb(DDRA, 0);
    outb(PORTA, 0xFF);
    Delay(16);
    if(inb(PINA) != 0xFF) {
        merr_bits = ~inb(PINA);
        merr = MERR_STICKY_DATABUS;
        return -1;
    }

    /*
     * Do a walking bit test to find shortcuts.
     */
    for(wb = 1; wb; wb <<= 1) {
        outb(PORTA, ~wb);
        outb(DDRA, wb);
        Delay(16);
        if(inb(PINA) != (uint8_t)~wb) {
            merr = MERR_SHORT_DATABUS;
            return -1;
        }
    };

    outb(PORTA, 0);
    outb(DDRA, 0xFF);

#ifdef __AVR_ENHANCED__
    /*
     * Checking the high address bus is easy.
     */
    outb(DDRC, 0);
    outb(PORTC, 0xFF);
    Delay(16);
    if(inb(PINC) != 0xFF) {
        merr_bits = ~inb(PINC);
        merr = MERR_STICKY_ADDRBUS;
        return -1;
    }
    for(wb = 1; wb; wb <<= 1) {
        outb(PORTC, ~wb);
        outb(DDRC, wb);
        Delay(16);
        if(inb(PINC) != (uint8_t)~wb) {
            merr = MERR_SHORT_ADDRBUS;
            return -1;
        }
    };
    outb(PORTC, 0);
    outb(DDRC, 0xFF);
#endif

    return 0;
}

#endif

/*!
 * \brief Test external SRAM.
 *
 * \return Number of bytes available.
 */
size_t XMemTest(void)
{
    size_t first = RAMEND + 1;
    size_t last = (size_t)-1;
    size_t addr;
    volatile uint8_t *mem;
    uint8_t wb;
    uint8_t pattern[] = { 0x00, 0xFF, 0x55, 0xAA };

#if defined(__AVR_ENHANCED__) && !defined(__AVR_ATmega2561__)
    /*
     * Test external memory bus on ATmega128 systems.
     * Fails on the ATmega2561. Why?
     */
    if(XMemTestBus()) {
        return 0;
    }
#endif

    /*
     * Enable external RAM.
     */
#if defined(__AVR_AT90CAN128__) || defined(__AVR_ATmega2561__)
    outb(XMCRA, _BV(SRE));
#elif defined (__AVR__)
    outb(MCUCR, _BV(SRE));
#endif

    /*
     * Let's see, how many kBytes we have. A simple pattern test on the
     * first bytes of each kilobyte boundary will do.
     */
    for (addr = first; addr >= first; addr += 1024) {
        mem = (uint8_t *)((addr + 1024) - sizeof(pattern));
        memcpy((void *)mem, pattern, sizeof(pattern));
        if(memcmp((void *)mem, pattern, sizeof(pattern))) {
            last = addr - 1;
        }

        /*
         * External RAM may not start at kilobyte boundary. Set the
         * address to full kilobytes after first test.
         */
        else if(addr == RAMEND + 1) {
            addr &= ~0x3FF;
        }
    }

    /*
     * Set all bits in RAM.
     */
    for (addr = first; addr >= first && addr < last; addr += 256) {
        memset((void *)addr, 0xFF, 256);
    }

    /*
     * Do an extensive test.
     */
    for (addr = first; addr >= first && addr < last; addr++) {
        mem = (uint8_t *)addr;
        /*
         * The next RAM location must still have all bits set. If not,
         * any manipulation on lower addresses may have modified this
         * one. Probably an address bus problem.
         */
        if(*mem != 0xFF) {
            merr_bits = ~*mem;
            merr_addr = (void *)mem;
            merr = MERR_BAD_ADDR;
            break;
        }

        /*
         * Do a walking bit test including all bits zero on the current
         * RAM location.
         */
        wb = 1;
        *mem = wb;
        do {
            if(*mem != wb) {
                break;
            }
            wb <<= 1;
            *mem = wb;
        } while(wb);

        /*
         * If the walking bit test hasn't finished or if not all bits
         * are cleared, then this data byte is broken.
         */
        if(wb || *mem) {
            if(wb) {
                merr_bits = *mem & ~wb;
            }
            else {
                merr_bits = *mem;
            }
            merr_addr = (void *)mem;
            merr = MERR_BAD_DATA;
            break;
        }
    }
    return last - first + 1;
}

/*!
 * \brief Test external banked SRAM.
 *
 * Banked memory is asumed on top of non-banked memory and the
 * start of banked memory is assumed on a 256 byte boundary.
 *
 * \return Number of banks available.
 */
int XMemBankTest(size_t *xramsize)
{
    volatile uint8_t *breg = (uint8_t *)((size_t)-1 & ~0xFF);
    volatile uint8_t *bmem = 0;
    size_t bsize = 16384;
    uint8_t *bend = (uint8_t *)bmem + bsize;
    uint8_t *xramend = (uint8_t *)(*xramsize + RAMEND + 1);
    volatile uint8_t *cp;
    uint8_t i;
    uint8_t j;

    /*
     * Determine the start address of banked memory. Switch to
     * bank 1 and set every 256th byte to 1. Then switch to bank 0
     * and clear every 256th byte. Next switch back to bank 1
     * and find the first location set to 1.
     */
    *(breg + 1) = 1;
    for (cp = (uint8_t *)(RAMEND + 1); cp < xramend; cp += 256) {
        *cp = 1;
    }
    *breg = 0;
    for (cp = (uint8_t *)(RAMEND + 1); cp < xramend; cp += 256) {
        *cp = 0;
    }

    *(breg + 1) = 1;
    for (cp = (uint8_t *)(RAMEND + 1); cp < xramend; cp += 256) {
        if(*cp == 1) {
            bmem = cp;
            break;
        }
    }

    /*
     * No RAM location kept the value of 1. There is no banked memory.
     */
    if(bmem == 0) {
        return 0;
    }

    *xramsize = xramend - bmem;


    for(i = 1; i; i++) {
        *(breg + i) = i;
        memset((void *)bmem, i, bsize);
        for(cp = bmem; cp < bend; cp++) {
            if(*cp != i) {
                break;
            }
        }
        if(cp < bend) {
            break;
        }

        /*
         * Test if non-banked memory is intact.
         */
        for (cp = (uint8_t *)(RAMEND + 1); cp < bmem; cp += 256) {
            if(*cp) {
                break;
            }
        }
        if(cp < bmem) {
            break;
        }
        for(j = 0; j < i; j++) {
            *(breg + j) = j;
            for(cp = bmem; cp < bend; cp++) {
                if(*cp != j) {
                    break;
                }
            }
            if(cp < bend) {
                break;
            }
        }
        if(j < i) {
            break;
        }
    }
    return i;
}

/*
 * Read/write external SRAM until key pressed.
 */
void LoopSRAM(void)
{
    register unsigned int pattern;
    volatile uint8_t *mem;
    uintptr_t faddr = 0xFFFF;
    puts("Check address and data bus.");
    printf_P(presskey_P);
    for (;;) {
        for (pattern = 1; pattern; pattern <<= 1) {
            if (pattern <= RAMEND)
                mem = (uint8_t *) (pattern | (RAMEND + 1));
            else
                mem = (uint8_t *) pattern;
            *mem = (uint8_t) ((pattern >> 8) | pattern);
        }
        for (pattern = 1; pattern; pattern <<= 1) {
            if (pattern <= RAMEND)
                mem = (uint8_t *) (pattern | (RAMEND + 1));
            else
                mem = (uint8_t *) pattern;
            if (*mem != (uint8_t) ((pattern >> 8) | pattern))
                faddr = (uintptr_t) mem;
        }
        Delay(5000);
        if (GetChar()) {
            printf("No RAM at 0x%04X\n", (unsigned int)faddr);
            return;
        }
    }
}

