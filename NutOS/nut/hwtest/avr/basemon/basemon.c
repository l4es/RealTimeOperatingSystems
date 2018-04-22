/*
 * Copyright (C) 2001-2008 by egnite Software GmbH
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
 * $Id: basemon.c 4116 2012-04-12 22:35:23Z olereinhardt $
 *
 * WARNING! Do not use any part of Basemon for your own applications. WARNING!
 *
 * This is not a typical application sample. It overrides parts of Nut/OS to
 * keep it running on broken hardware.
 */

/*!
 * \example basemon/basemon.c
 *
 * Basic hardware test monitor for AVR targets.
 */

#include <stdio.h>
#include <string.h>

#include "rtlregs.h"
#include <dev/debug.h>

#include <sys/version.h>
#include <sys/heap.h>
#include <sys/thread.h>
#include <sys/timer.h>
#include <sys/confnet.h>
#include <sys/confos.h>
#include <sys/socket.h>

#include <arpa/inet.h>
#include <net/if_var.h>

#include "realtek.h"
#include "smsc.h"
#include "webdemo.h"
#include "xmemtest.h"
#include "dataflash.h"
#include "uart.h"
#include "utils.h"

#if defined(__GNUC__) && defined(__AVR__)
void NutInit(void) __attribute__ ((naked)) __attribute__ ((section(".init8")));
extern int NutAppMain(void) __attribute__ ((noreturn));
#endif


int uart_bs;
uint8_t nic;

static char *version = "4.4.0";
static size_t sram;
static uint8_t banks;
static size_t banksize;

static long srom;

char my_ip[32];
char my_mask[32];
char my_gate[32];
uint8_t my_mac[32];
uint8_t eg_mac[] = { 0x00, 0x06, 0x98, 0x00, 0x00, 0x00 };

volatile uint8_t ms62_5;

THREAD(idle, arg)
{
    NutTimerInit();
    NutThreadCreate("main", WebDemo, 0, 768);
    NutThreadSetPriority(254);
    for (;;) {
#ifdef HEARTBEAT_BIT
        HeartBeat();
#endif
        NutThreadYield();
    }
}

/*
 * Test ports.
 */
int TestPorts(void)
{
#if defined(__AVR__)
    uint8_t pat;
    uint8_t ipat;
    uint8_t val;
    uint8_t bpat = 0;
    uint8_t dpat = 0;
    uint8_t epat = 0;
    uint8_t fpat = 0;

    /*
     * Port B.
     */
    for (pat = 1; pat; pat <<= 1) {
        ipat = ~pat;
        /* Keep dataflash select low. */
        ipat &= 0xEF;
        outb(DDRB, pat | 0x10);
        outb(PORTB, ipat);
        Delay(100);
        val = inb(PINB);
        if (val != ipat) {
            bpat |= (ipat ^ val);
        }
    }
    outb(DDRB, 0);
    if (bpat)
        printf("PORTB bits 0x%02X\n\x07", bpat);

#ifndef INTECH21
    /*
     * Port D.
     */
    for (pat = 1; pat; pat <<= 1) {
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega2561__)
        /* Exclude PD5 used for RS485 direction select. */
        if (pat & 0x20) {
            continue;
        }
#endif
        ipat = ~pat;
        outb(DDRD, pat);
        outb(PORTD, ipat);
        Delay(1000);
        if ((val = inb(PIND) | 0x20) != ipat) {
            printf("[%02X-%02X]", ipat, val);
            dpat |= pat;
        }
    }
    outb(DDRD, 0);
    if (dpat)
        printf("PORTD bits 0x%02X\n\x07", dpat);

    /*
     * Port E. Exclude PE0, PE1 and PE5.
     */
    for (pat = 4; pat; pat <<= 1) {
        if (pat == 0x20)
            continue;
        ipat = ~pat;
        outb(DDRE, pat);
        outb(PORTE, ipat);
        Delay(1000);
        if ((val = inb(PINE) | 0x23) != ipat) {
            epat |= pat;
        }
    }
    outb(DDRE, 0);
    if (epat)
        printf("PORTE bits 0x%02X\n\x07", epat);

#ifdef __AVR_ENHANCED__
    /*
     * Port F. Exclude PF4, PF5, PF6 and PF7.
     */
    for (pat = 1; pat & 0x0F; pat <<= 1) {
        ipat = ~pat;
        outb(DDRF, pat);
        outb(PORTF, ipat);
        Delay(1000);
        if ((val = inb(PINF) | 0xF0) != ipat) {
            fpat |= pat;
        }
    }
    outb(DDRF, 0);
    if (fpat)
        printf("PORTF bits 0x%02X\n\x07", fpat);
#endif
#endif /* INTECH21 */

    if (bpat || dpat || epat || fpat)
        return -1;
#endif /* __AVR__ */
    return 0;
}

/*
 * Basic monitor function.
 */
void BaseMon(void)
{
    char ch;
    int i;
    int n;
    char hostname[sizeof(confos.hostname)];
    static prog_char menu1_P[] = "\nPress any of the following keys:";
    static prog_char menu2_P[] = "    B - Send broadcasts";
    static prog_char menu3_P[] = "    E - Ethernet controller read/write\n"
                                 "    J - Jump to bootloader\n"
                                 "    S - SRAM read/write";
    static prog_char menu4_P[] = "    X - Exit BaseMon, configure network and start WebServer";

    /*
     * Print banner.
     */
    printf("\n\nBaseMon %s\nNut/OS %s\n", version, NutVersionString());
#ifdef INTECH21
    puts("Customized for Intech21");
#endif
    printf("Compiled by ");
#ifdef __IMAGECRAFT__
    printf("ICCAVR");
#else
    printf("AVRGCC");
#endif
    printf(" for ");
#if defined(__AVR_ATmega128__)
    puts("ATmega128");
#elif defined(__AVR_ATmega103__)
    puts("ATmega103");
#elif defined(__AVR_AT90CAN128__)
    puts("AT90CAN128");
#elif defined(__AVR_ATmega2561__)
    puts("ATmega2561");
#else
    puts("unknown");
#endif

    if (uart_bs >= 0) {
        printf("Baudrate select = %d\n", uart_bs);
    }
    else {
        puts("No user input detected");
    }

    /*
     * Test external SRAM.
     */
    printf("External RAM Test... ");
    sram = XMemTest();
    printf("%u bytes\n", (unsigned int)sram);

    printf("Banked RAM Test...   ");
    banksize = sram;
    banks = XMemBankTest(&banksize);
    if (banks)
        printf("%u banks, %u bytes ea.\n", banks, (unsigned int)banksize);
    else
        puts("none");

    /*
     * Test external Flash.
     */
    printf("Serial FLASH...      ");
    srom = SpiMemTest();
    printf("%lu bytes\n", srom);

    /*
     * Test Ethernet controller hardware.
     */
    printf("Detecting NIC...     ");
    if (SmscDetect()) {
        if (RealtekDetect()) {
            nic = 0;
            puts("none\x07");
        } else {
            nic = 1;
            puts("RTL8019AS");
        }
    } else {
        nic = 2;
        puts("LAN91C111");
    }

    if (nic) {
        printf("Testing NIC...       ");
        if (nic == 1) {
            RealtekTest();
        } else {
            SmscTest();
        }
    }

    /*
     * Test I/O ports.
     */
#if defined(ETHERNUT1) || defined(ETHERNUT2)
    printf("I/O Port Test...     ");
    if (TestPorts() == 0)
        puts("OK");
#endif

    /*
     * Return if running without serial port.
     */
    if (uart_bs < 0)
        return;
    for (;;) {
        for (;;) {
            while (GetChar());
            puts_P(menu1_P);
            if (sram)
                puts_P(menu2_P);
            puts_P(menu3_P);
            if (sram > 8191 && nic)
                puts_P(menu4_P);
            while ((ch = GetChar()) == 0 || ch == ' ');
            if (sram > 8191 && (ch == 'x' || ch == 'X'))
                break;
            if (sram && (ch == 'b' || ch == 'B')) {
                if (nic == 1)
                    RealtekSend();
                else
                    SmscSend();
            } else if (ch == 'e' || ch == 'E') {
                if (nic == 1) {
                    RealtekLoop();
                } else {
                    SmscLoop();
                }
            } else if (ch == 'j' || ch == 'J') {
                puts("Booting...");
#if defined(__AVR__)
                asm("jmp 0x1F000");
#endif
            } else if (ch == 's' || ch == 'S')
                LoopSRAM();
        }

        /*
         * Input MAC address.
         */
        for (;;) {
            printf("\nMAC address (%02X%02X%02X%02X%02X%02X): ", my_mac[0], my_mac[1], my_mac[2], my_mac[3], my_mac[4],
                   my_mac[5]);
            GetLine(inbuff, sizeof(inbuff));
            if ((n = strlen(inbuff)) == 0)
                break;
            for (i = 0; i < n; i++)
                if (inbuff[i] < '0' ||
                    (inbuff[i] > '9' && inbuff[i] < 'A') || (inbuff[i] > 'F' && inbuff[i] < 'a') || inbuff[i] > 'f')
                    n = 13;
            if (n <= 12 && (n & 1) == 0) {
                n >>= 1;
                for (i = 0; i < n; i++)
                    my_mac[6 - n + i] = hex2bin(inbuff[i * 2]) * 16 + hex2bin(inbuff[i * 2 + 1]);
                break;
            }
            printf("Bad MAC address");
        }

        /*
         * Input host name.
         */
        printf("Host name (%s): ", confos.hostname);
        GetLine(hostname, sizeof(confos.hostname) - 1);

        /*
         * Input IP address.
         */
        GetIP("IP address", my_ip);
        /*
         * Input netmask and gateway, if non-zero IP address.
         */
        if (inet_addr(my_ip)) {
            GetIP("Net mask", my_mask);
            GetIP("Default route", my_gate);
            if (inet_addr(my_gate) == 0 || (inet_addr(my_ip) & inet_addr(my_mask)) == (inet_addr(my_gate) & inet_addr(my_mask)))
                break;
        } else {
            printf("Using DHCP or ARP method (Y): ");
            GetLine(inbuff, sizeof(inbuff));
            if (strlen(inbuff) == 0 || inbuff[0] == 'Y' || inbuff[0] == 'y')
                break;
        }
    }
    puts("\n");
    confnet.cdn_cip_addr = inet_addr(my_ip);
    confnet.cdn_ip_mask = inet_addr(my_mask);
    confnet.cdn_gateway = inet_addr(my_gate);
    memcpy(confnet.cdn_mac, my_mac, sizeof(confnet.cdn_mac));
#ifdef HEARTBEAT_BIT
    HeartBeat();
#endif
    NutNetSaveConfig();
    if (strlen(hostname) > 0) {
        strncpy(confos.hostname, hostname, sizeof(confos.hostname) - 1);
        confos.hostname[sizeof(confos.hostname) - 1] = 0;
    }
#ifdef HEARTBEAT_BIT
    HeartBeat();
#endif
    NutSaveConfig();
}

void NutInit(void)
{
    extern void *__bss_end;

    /*
     * Make sure that stack pointer points into internal RAM.
     */
#if defined(__AVR__)
    outb(SPH, inb(SPH) & (RAMEND >> 8));
#endif

    /*
     * Use the rest of our internal RAM for our heap. Re-opening
     * standard output will use malloc. We do not use any external
     * RAM before passing the memory test.
     */
    NutHeapAdd(&__bss_end, (uintptr_t) RAMEND - 256 - (uintptr_t) (&__bss_end));

    /*
     * Use the debug UART driver for output. In opposite
     * to the standard UART driver, it uses non-buffered
     * polling mode, which is better suited to run on
     * untested hardware.
     */
#if defined(__AVR__)
    NutRegisterDevice(&devDebug0, 0, 0);
#endif
    uart_bs = DetectSpeed();
    freopen("uart0", "w", stdout);
    /*
     * Load os configuration from EEPROM.
     */
    if (NutLoadConfig())
        strcpy (confos.hostname, "ethernut");
    /*
     * Load network configuration from EEPROM.
     */
    NutNetLoadConfig("eth0");
    if ((confnet.cdn_mac[0] & confnet.cdn_mac[1] & confnet.cdn_mac[2]) == 0xFF ||
        (confnet.cdn_mac[0] | confnet.cdn_mac[1] | confnet.cdn_mac[2]) == 0x00) {
        memcpy(confnet.cdn_mac, eg_mac, sizeof(confnet.cdn_mac));
    }
    strcpy(my_ip, inet_ntoa(confnet.cdn_cip_addr));
    strcpy(my_mask, inet_ntoa(confnet.cdn_ip_mask));
    strcpy(my_gate, inet_ntoa(confnet.cdn_gateway));
    memcpy(my_mac, confnet.cdn_mac, 6);
    /*
     * Perform basic monitor functions.
     */
    BaseMon();
    /*
     * Initialize heap and create the idle thread. This will in turn
     * start the WebDemo thread.
     */
    if (sram) {
        NutHeapAdd((void *) (RAMEND + 1), sram);
        NutThreadCreate("idle", idle, 0, 384);
    }

    NutThreadSetPriority(128);
    for (;;) {
#ifdef HEARTBEAT_BIT
        HeartBeat();
#endif
        NutSleep(500);
    }
}

#ifdef __IMAGECRAFT__

void main(void)
{
    NutInit();
}

#endif
