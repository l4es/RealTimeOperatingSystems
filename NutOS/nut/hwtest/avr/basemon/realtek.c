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
 * $Id: realtek.c 3597 2011-10-21 16:17:18Z haraldkipp $
 *
 * WARNING! Do not use any part of Basemon for your own applications. WARNING!
 *
 * This is not a typical application sample. It overrides parts of Nut/OS to
 * keep it running on broken hardware.
 */

#include <stdio.h>
#include <sys/timer.h>

#include "basemon.h"
#include "utils.h"
#include "uart.h"
#include "rtlregs.h"
#include "realtek.h"


#define RTL_EESK_BIT    5
#define RTL_EESK_PORT   PORTC
#define RTL_EESK_PIN    PINC
#define RTL_EESK_DDR    DDRC

#define RTL_EEDO_BIT    6
#define RTL_EEDO_PORT   PORTC
#define RTL_EEDO_DDR    DDRC

#define RTL_EEMU_BIT    7
#define RTL_EEMU_PORT   PORTC
#define RTL_EEMU_DDR    DDRC

#define RTL_EE_MEMBUS

static uint8_t *nic_base;

static int NicReset(void)
{
    volatile uint8_t *base = nic_base;
    uint8_t i;
    uint8_t j;

    /* Start command clears the reset bit. */
    nic_write(NIC_CR, NIC_CR_STA | NIC_CR_RD2);
    //printf("[%02X]", nic_read(NIC_PG0_ISR));
    for (j = 0; j < 20; j++) {
        printf("SW-Reset...");
        i = nic_read(NIC_RESET);
        Delay(500);
        nic_write(NIC_RESET, i);
        for (i = 0; i < 20; i++) {
            //Delay(5000);
            /*
             * ID detection added for version 1.1 boards.
             */
            if ((nic_read(NIC_PG0_ISR) & NIC_ISR_RST) != 0 && nic_read(NIC_PG0_RBCR0) == 0x50 && nic_read(NIC_PG0_RBCR1) == 0x70) {
                puts("OK");
                return 0;
            }
        }
        puts("failed\x07");
        /*
         * Toggle the hardware reset line. Since Ethernut version 1.3 the
         * hardware reset pin of the nic is no longer connected to bit 4
         * on port E, but wired to the board reset line.
         */
        if (j == 10) {
            puts("HW-Reset");
#if defined (__AVR__)
            sbi(DDRE, 4);
            sbi(PORTE, 4);
            Delay(100000);
            cbi(PORTE, 4);
            Delay(250000);
#endif
        }
    }
    return -1;
}

static int DetectNicEeprom(void)
{
#ifdef __AVR_ENHANCED__
    register unsigned int cnt = 0;

    cli();
    /*
     * Prepare the EEPROM emulation port bits. Configure the EEDO
     * and the EEMU lines as outputs and set both lines to high.
     */
    outb(PORTC, 0xC0);
    outb(DDRC, 0xC0);

    /*
     * Force the chip to re-read the EEPROM contents.
     */
    nic_outlb(NIC_CR, NIC_CR_STP | NIC_CR_RD2 | NIC_CR_PS0 | NIC_CR_PS1);
    nic_outlb(NIC_PG3_EECR, NIC_EECR_EEM0);

    /*
     * No external memory access beyond this point.
     */
#if defined(__AVR_AT90CAN128__) || defined(__AVR_ATmega2561__)
    cbi(XMCRA, SRE);
#else
    cbi(MCUCR, SRE);
#endif

    /*
     * Check, if the chip toggles our EESK input. If not, we do not
     * have EEPROM emulation hardware.
     */
    if(bit_is_set(PINC, 5)) {
        while(++cnt && bit_is_set(PINC, 5));
    }
    else {
        while(++cnt && bit_is_clear(PINC, 5));
    }

    /*
     * Enable memory interface.
     */
#if defined(__AVR_AT90CAN128__) || defined(__AVR_ATmega2561__)
    sbi(XMCRA, SRE);
#else
    sbi(MCUCR, SRE);
#endif

    /* Reset port outputs to default. */
    outb(PORTC, 0x00);
    outb(DDRC, 0x00);

    /* Wait until controller ready. */
    while(nic_inlb(NIC_CR) != (NIC_CR_STP | NIC_CR_RD2));

    sei();
    return cnt ? 0 : -1;
#else
    return -1;
#endif
}

/*
 * Emulated EEPROM contents.
 *
 * In jumper mode our influence is quite limited, only CONFIG3 and CONFIG4
 * can be modified.
 */
static prog_char nic_eeprom[18] = {
    0xFF,   /* CONFIG2: jPL1 jPL0   0      jBS4   jBS3   jBS2  jBS1  jBS0  */
    0xFF,   /* CONFIG1: 1    jIRQS2 jIRQS1 jIRQS0 jIOS3  jIOS2 jIOS1 jIOS0 */

    0xFF,   /* CONFIG4: -    -      -      -      -      -     -     IOMS  */
    0x30,   /* CONFIG3  PNP  FUDUP  LEDS1  LEDS0  -      0     PWRDN ACTB  */

    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, /* MAC */
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF /* ID */
};

/*!
 * \brief EEPROM emulator.
 *
 * Forces the chip to re-read the EEPROM contents and emulates a serial
 * EEPROM.
 *
 * If the hardware does not support this feature, then this call will
 * never return. Thus, make sure to have the driver properly configured.
 */
static void EmulateNicEeprom(void)
{
#ifdef __AVR_ENHANCED__
    register uint8_t clk;
    register uint8_t cnt;
    register uint8_t val;

    /*
     * Prepare the EEPROM emulation port bits. Configure the EEDO and
     * the EEMU lines as outputs and set EEDO to low and EEMU to high.
     */
    outb(PORTC, 0xC0);
    outb(DDRC, 0xC0);

    /*
     * Start EEPROM configuration. Stop/abort any activity and select
     * configuration page 3. Setting bit EEM0 will force the controller
     * to read the EEPROM contents.
     */

    /* Select page 3, stop and abort/complete. */
    nic_outlb(NIC_CR, NIC_CR_STP | NIC_CR_RD2 | NIC_CR_PS0 | NIC_CR_PS1);
    nic_outlb(NIC_PG3_EECR, NIC_EECR_EEM0);

    /*
     * We can avoid wasting port pins for EEPROM emulation by using the
     * upper bits of the address bus.
     */
    /*
     * No external memory access beyond this point.
     */
#if defined(__AVR_AT90CAN128__) || defined(__AVR_ATmega2561__)
    cbi(XMCRA, SRE);
#else
    cbi(MCUCR, SRE);
#endif

    /*
     * Loop for all EEPROM words.
     */
    for(cnt = 0; cnt < sizeof(nic_eeprom); ) {

        /*
         *
         * 1 start bit, always high
         * 2 op-code bits
         * 7 address bits
         * 1 dir change bit, always low
         */
        for(clk = 0; clk < 11; clk++) {
            while(bit_is_clear(RTL_EESK_PIN, RTL_EESK_BIT));
            while(bit_is_set(RTL_EESK_PIN, RTL_EESK_BIT));
        }

        /*
         * Shift out the high byte, MSB first. Our data changes at the EESK
         * rising edge. Data is sampled by the Realtek at the falling edge.
         */
        val = PRG_RDB(nic_eeprom + cnt);
        cnt++;
        for(clk = 0x80; clk; clk >>= 1) {
            while(bit_is_clear(RTL_EESK_PIN, RTL_EESK_BIT));
            if(val & clk)
                sbi(RTL_EEDO_PORT, RTL_EEDO_BIT);
            while(bit_is_set(RTL_EESK_PIN, RTL_EESK_BIT));
            cbi(RTL_EEDO_PORT, RTL_EEDO_BIT);
        }

        /*
         * Shift out the low byte.
         */
        val = PRG_RDB(nic_eeprom + cnt);
        cnt++;
        for(clk = 0x80; clk; clk >>= 1) {
            while(bit_is_clear(RTL_EESK_PIN, RTL_EESK_BIT));
            if(val & clk)
                sbi(RTL_EEDO_PORT, RTL_EEDO_BIT);
            while(bit_is_set(RTL_EESK_PIN, RTL_EESK_BIT));
            cbi(RTL_EEDO_PORT, RTL_EEDO_BIT);
        }


        /* 5 remaining clock cycles. */
        for(clk = 0; clk < 5; clk++) {
            while(bit_is_clear(RTL_EESK_PIN, RTL_EESK_BIT));
            while(bit_is_set(RTL_EESK_PIN, RTL_EESK_BIT));
        }
    }

    /*
     * Enable memory interface.
     */
#if defined(__AVR_AT90CAN128__) || defined(__AVR_ATmega2561__)
    sbi(XMCRA, SRE);
#else
    sbi(MCUCR, SRE);
#endif

    /* Reset port outputs to default. */
    outb(PORTC, 0x00);
    outb(DDRC, 0x00);
#endif
}

static int RealtekProbe(size_t addr)
{
    uint8_t bv;

    nic_base = (uint8_t *)addr;
    bv = nic_inlb(NIC_CR);
    if(bv & (NIC_CR_PS0 | NIC_CR_PS1)) {
        nic_outlb(NIC_CR, NIC_CR_STP | NIC_CR_RD2);
    }
    if(nic_inw(NIC_PG0_RBCR0) != 0x7050) {
        return -1;
    }
    return 0;
}

/*!
 * \brief Detect an RTL8019AS chip.
 */
int RealtekDetect(void)
{
    size_t addr;

    if (RealtekProbe(NIC_BASE) == 0) {
        return 0;
    }
    for (addr = 0x8000; addr; addr += 0x100) {
        if (RealtekProbe(addr) == 0) {
            return 0;
        }
    }
    return -1;
}

void RealtekLoop(void)
{
    printf_P(presskey_P);
    for (;;) {
        nic_outlb(NIC_CR, NIC_CR_STP | NIC_CR_RD2);
        printf("\rid=0x%04X", nic_inw(NIC_PG0_RBCR0));
        if (GetChar()) {
            puts("");
            return;
        }
    }
}

int RealtekTest(void)
{
    int i;
    uint8_t force_swreset = 0;
    volatile uint8_t *base = nic_base;
    uint16_t nic_id;

    /*
     * NIC ID detection loop.
     */
    for (;;) {
        /*
         * Reset loop.
         */
        for (;;) {

            /*
             * The hardware reset pin of the nic is connected
             * to bit 4 on port E. Make this pin an output pin
             * and toggle it.
             */
#if defined(__AVR__)
            sbi(DDRE, 4);
            sbi(PORTE, 4);
            Delay(2000);
            cbi(PORTE, 4);
            Delay(20000);
#endif
            /*
             * If the hardware reset didn't set the nic in reset
             * state, perform an additional software reset and
             * wait until the controller enters the reset state.
             */
            if (force_swreset || (*(base + 7) & 0x80) == 0) {
                *(base + 0x1f) = *(base + 0x1f);
                Delay(200000);
                for (i = 0; i < 255; i++) {
                    if (*(base + 7) & 0x80) {
                        break;
                    }
                }
                if (i < 255) {
                    break;
                }
                puts("failed\x07");
            } else {
                break;
            }
            if (uart_bs >= 0)
                break;
        }
        nic_id = nic_inw(NIC_PG0_RBCR0);
        if (nic_id == 0x7050) {
            puts("OK");
            break;
        }
        printf("failed, got 0x%04X, expected 0x7050\n\x07", nic_id);
        if (force_swreset && uart_bs >= 0)
            break;
        force_swreset = 1;
    }
    return 0;
}

#define NIC_PAGE_SIZE       0x100
#define NIC_START_PAGE      0x40
#define NIC_STOP_PAGE       0x60
#define NIC_TX_PAGES        6
#define NIC_TX_BUFFERS      2
#define NIC_FIRST_TX_PAGE   NIC_START_PAGE
#define NIC_FIRST_RX_PAGE   (NIC_FIRST_TX_PAGE + NIC_TX_PAGES * NIC_TX_BUFFERS)
#define TX_PAGES            12

void RealtekSend(void)
{
    uint8_t mac[] = {
        0x00, 0x06, 0x98, 0x00, 0x00, 0x00
    };
    uint16_t sz;
    uint16_t i;
    volatile uint8_t *base = nic_base;
    uint8_t rb;
    uint32_t cnt = 0;

    printf("Init controller...");
    nic_write(NIC_PG0_IMR, 0);
    nic_write(NIC_PG0_ISR, 0xff);
    if (NicReset())
        return;

    printf(" detecting...");
    Delay(200000);
    if (DetectNicEeprom() == 0) {
        printf("EEPROM Emulation...");
        EmulateNicEeprom();
    }

    nic_write(NIC_CR, NIC_CR_STP | NIC_CR_RD2 | NIC_CR_PS0 | NIC_CR_PS1);
    nic_write(NIC_PG3_EECR, NIC_EECR_EEM0 | NIC_EECR_EEM1);
    nic_write(NIC_PG3_CONFIG3, 0);
    nic_write(NIC_PG3_CONFIG2, NIC_CONFIG2_BSELB);
    nic_write(NIC_PG3_EECR, 0);

    Delay(50000);
    nic_write(NIC_CR, NIC_CR_STP | NIC_CR_RD2);
    nic_write(NIC_PG0_DCR, NIC_DCR_LS | NIC_DCR_FT1);
    nic_write(NIC_PG0_RBCR0, 0);
    nic_write(NIC_PG0_RBCR1, 0);
    nic_write(NIC_PG0_RCR, NIC_RCR_MON);
    nic_write(NIC_PG0_TCR, NIC_TCR_LB0);
    nic_write(NIC_PG0_TPSR, NIC_FIRST_TX_PAGE);
    nic_write(NIC_PG0_BNRY, NIC_STOP_PAGE - 1);
    nic_write(NIC_PG0_PSTART, NIC_FIRST_RX_PAGE);
    nic_write(NIC_PG0_PSTOP, NIC_STOP_PAGE);
    nic_write(NIC_PG0_ISR, 0xff);
    nic_write(NIC_CR, NIC_CR_STP | NIC_CR_RD2 | NIC_CR_PS0);
    for (i = 0; i < 6; i++)
        nic_write(NIC_PG1_PAR0 + i, mac[i]);
    for (i = 0; i < 8; i++)
        nic_write(NIC_PG1_MAR0 + i, 0);
    nic_write(NIC_PG1_CURR, NIC_START_PAGE + TX_PAGES);
    nic_write(NIC_CR, NIC_CR_STP | NIC_CR_RD2);
    nic_write(NIC_PG0_RCR, NIC_RCR_AB);
    nic_write(NIC_PG0_ISR, 0xff);
    nic_write(NIC_PG0_IMR, 0);
    nic_write(NIC_CR, NIC_CR_STA | NIC_CR_RD2);
    nic_write(NIC_PG0_TCR, 0);
    Delay(1000000);
    puts("done");
    nic_write(NIC_CR, NIC_CR_STA | NIC_CR_RD2 | NIC_CR_PS0 | NIC_CR_PS1);
    rb = nic_read(NIC_PG3_CONFIG0);
    switch (rb & 0xC0) {
    case 0x00:
        printf("RTL8019AS ");
        if (rb & 0x08)
            printf("jumper mode: ");
        if (rb & 0x20)
            printf("AUI ");
        if (rb & 0x10)
            printf("PNP ");
        break;
    case 0xC0:
        printf("RTL8019 ");
        if (rb & 0x08)
            printf("jumper mode: ");
        break;
    default:
        printf("Unknown chip ");
        break;
    }
    if (rb & 0x04)
        printf("BNC\x07 ");
    if (rb & 0x03)
        printf("Failed\x07 ");
    rb = nic_read(NIC_PG3_CONFIG1);
    printf("IRQ%u ", (rb >> 4) & 7);
    rb = nic_read(NIC_PG3_CONFIG2);
    switch (rb & 0xC0) {
    case 0x00:
        printf("Auto ");
        break;
    case 0x40:
        printf("10BaseT ");
        break;
    case 0x80:
        printf("10Base5 ");
        break;
    case 0xC0:
        printf("10Base2 ");
        break;
    }


    printf("\n");
    nic_write(NIC_CR, NIC_CR_STA | NIC_CR_RD2);
    for (;;) {
        Delay(500000);
        printf("\r%lu", cnt++);
        sz = 1500;
        nic_write(NIC_PG0_RBCR0, sz);
        nic_write(NIC_PG0_RBCR1, sz >> 8);
        nic_write(NIC_PG0_RSAR0, 0);
        nic_write(NIC_PG0_RSAR1, NIC_FIRST_TX_PAGE);
        nic_write(NIC_CR, NIC_CR_STA | NIC_CR_RD1);
        /*
         * Transfer ethernet physical header.
         */
        for (i = 0; i < 6; i++)
            nic_write(NIC_IOPORT, 0xFF);
        for (i = 0; i < 6; i++)
            nic_write(NIC_IOPORT, mac[i]);
        nic_write(NIC_IOPORT, 0x08);
        nic_write(NIC_IOPORT, 0x00);
        /*
         * Add pad bytes.
         */
        for (i = 0; i < sz; i++)
            nic_write(NIC_IOPORT, 0);
        /*
         * Complete remote dma.
         */
        nic_write(NIC_CR, NIC_CR_STA | NIC_CR_RD2);
        for (i = 0; i <= 20; i++)
            if (nic_read(NIC_PG0_ISR) & NIC_ISR_RDC)
                break;
        nic_write(NIC_PG0_ISR, NIC_ISR_RDC);
        /*
         * Number of bytes to be transmitted.
         */
        nic_write(NIC_PG0_TBCR0, (sz & 0xff));
        nic_write(NIC_PG0_TBCR1, ((sz >> 8) & 0xff));
        /*
         * First page of packet to be transmitted.
         */
        nic_write(NIC_PG0_TPSR, NIC_FIRST_TX_PAGE);
        /*
         * Start transmission.
         */
        nic_write(NIC_CR, NIC_CR_STA | NIC_CR_TXP | NIC_CR_RD2);
        if (GetChar())
            break;
        Delay(10000);
    }
}

