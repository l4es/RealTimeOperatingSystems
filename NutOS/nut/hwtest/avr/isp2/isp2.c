/*
 * Copyright (C) 2003 by egnite Software GmbH
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
 * $Log$
 * Revision 1.1  2003/11/03 15:51:31  haraldkipp
 * First check in
 *
 */

/*
 * This application will update the ISP Adapter software.
 */

#include <stdio.h>
#include <io.h>
#include <sys/timer.h>
#include <dev/debug.h>
#include <dev/urom.h>

/*
 * Port Layout
 *
 * Coconut
 * -------
 * PB0(O): SS
 * PB1(O): SCK
 * PB2(O): MOSI
 * PB3(I): MISO
 *
 * PB4(O): Reset target
 * PB5(-): Unused
 * PB6(-): Unused
 * PB7(-): Unused
 *
 * ISP at Ethernut 2
 * -----------------
 * PE0(O): ISP-MOSI
 * PE1(I): ISP-MISO
 * PB1(O): ISP-SCK
 */

#define ISPMOSI_PORT    PORTE
#define ISPMOSI_DDR     DDRE
#define ISPMOSI_BIT     0

#define ISPMISO_PORT    PORTE
#define ISPMISO_DDR     DDRE
#define ISPMISO_PIN     PINE
#define ISPMISO_BIT     1

#define ISPSCK_PORT     PORTB
#define ISPSCK_DDR      DDRB
#define ISPSCK_BIT      1

/*!
 * \brief Exchange SPI byte.
 */
static uint8_t SpiByte(uint8_t c)
{
    uint8_t i;

    for(i = 0; i < 8; i++) {
        if(c & 0x80)
            sbi(ISPMOSI_PORT, ISPMOSI_BIT);
        else
            cbi(ISPMOSI_PORT, ISPMOSI_BIT);
        sbi(ISPSCK_PORT, ISPSCK_BIT);
        c <<= 1;
        if(bit_is_set(ISPMISO_PIN, ISPMISO_BIT))
            c++;
        cbi(ISPSCK_PORT, ISPSCK_BIT);
    }
    cbi(ISPMOSI_PORT, ISPMOSI_BIT);

    return c;
}

/*!
 * \brief Enable SPI device flash programming.
 *
 * \return 0 if device could be located, -1 otherwise.
 */
int SpiFlashEnable(void)
{
    uint8_t i;
    uint8_t rc;

    cbi(ISPMOSI_PORT, ISPMOSI_BIT);
    sbi(ISPMOSI_DDR, ISPMOSI_BIT);

    cbi(ISPMISO_PORT, ISPMISO_BIT);
    cbi(ISPMISO_DDR, ISPMISO_BIT);

    cbi(ISPSCK_PORT, ISPSCK_BIT);
    sbi(ISPSCK_DDR, ISPSCK_BIT);

    for (i = 0; i < 32; i++) {

        /*
         * Try to enable programming.
         */
        SpiByte(0xAC);
        SpiByte(0x53);
        rc = SpiByte(0xFF);
        SpiByte(0xff);

        if (rc == 0x53)
            return 0;

        /*
         * Programming enable failed. This may be because the
         * target is not synchronized. A positive pulse on the
         * clock line should help.
         */
        sbi(ISPSCK_PORT, ISPSCK_BIT);
        cbi(ISPSCK_PORT, ISPSCK_BIT);
    }
    return -1;
}

/*!
 * Read SPI device ID.
 *
 * \param id Three byte character array, which receives
 *           the CPU ID.
 */
void SpiFlashId(uint8_t * id)
{
    uint8_t i;

    for (i = 0; i < 3; i++) {
        SpiByte(0x30);
        SpiByte(0x00);
        SpiByte(i);
        id[i] = SpiByte(0x00);
    }
}

/*!
 * \brief Write byte to the target's flash memory.
 *
 * The target must have been erased by a previous call
 * to SpiFlashErase().
 *
 * \param high Must be 0 to write the low byte or 8 to
 *             write the high byte.
 * \param addr Word address to write to.
 * \param data Byte value to write.
 *
 * \return 0 on success, -1 otherwise.
 */
int SpiFlashWriteByte(uint8_t high, uint16_t addr, uint8_t data)
{
    uint8_t d;

    if (data != 0xff) {
        SpiByte(0x40 | high);
        SpiByte(addr >> 8);
        SpiByte(addr & 0xFF);
        SpiByte(data);

        /*
         * During programming a value of 0x7F appears at the memory location.
         * If we are programming this value, we delay execution by 10 ms.
         * Otherwise we poll the memory location until we read back the
         * programmed value.
         */
        if (data == 0x7f)
            NutDelay(10);
        else {
            for (d = 0; d < 255; d++) {
                /*
                 * Read program flash byte.
                 */
                SpiByte(0x20 | high);
                SpiByte(addr >> 8);
                SpiByte(addr & 0xFF);
                if (SpiByte(0xFF) == data)
                    break;
            }
            if (d == 255) {
                return -1;
            }
        }
    }
    return 0;
}

/*!
 * \brief Write word to the target's flash memory.
 *
 * \param addr Word address to write to.
 * \param data Word value to write.
 *
 * \return 0 on success, -1 otherwise.
 */
int SpiFlashWriteWord(uint16_t addr, uint16_t data)
{
    if (SpiFlashWriteByte(0, addr, data & 0xFF))
        return -1;
    if (SpiFlashWriteByte(8, addr, data >> 8))
        return -1;

    return 0;
}

/*!
 * \brief Erase target's flash memory.
 *
 * Sets all bytes on the target's flash memory to 0xFF.
 * In addtion all lock bits are set to 1 (unprogrammed).
 */
void SpiFlashErase(void)
{
    /*
     * Send chip erase command.
     */
    SpiByte(0xAC);
    SpiByte(0x80);
    SpiByte(0x00);
    SpiByte(0x00);
    NutDelay(50);
}


int main(void)
{
    uint8_t id[3];
    uint32_t baud = 115200;
    char *filename = "UROM:sisp.bin";
    int val;
    uint16_t word;
    uint16_t addr;
    FILE *fp;

    /*
     * Register Nut/OS devices. We can't use UART0, because
     * it uses the same pins as the ISP port.
     */
    NutRegisterDevice(&devDebug1, 0, 0);
    NutRegisterDevice(&devUrom, 0, 0);

    /*
     * Open stdout for displaying our progress.
     */
    freopen("uart1", "w", stdout);
    _ioctl(_fileno(stdout), UART_SETSPEED, &baud);
    puts("\nISP2 1.0.1");

    /*
     * Open the file to burn into the adapter's flash memory.
     */
    fp = fopen(filename, "rb");
    if (fp == 0) {
        printf("ERROR: Failed to open %s\n", filename);
        for(;;);
    }

    /*
     * Try to enter programming mode.
     */
    printf("Enable programming...");
    if(SpiFlashEnable()) {
        puts("failed\n"
             "Make sure that the ISP adapter is connected to the\n"
             "Ethernut ISP port and that the MCU on the adapter\n"
             "is held in reset state.");
        for(;;);
    }
    puts("OK");

    /*
     * Read the target device's signature.
     */
    printf("Reading signature... ");
    SpiFlashId(id);
    if(id[0] != 0x1E || id[1] != 0x91 || id[2] != 0x01) {
        printf("unexpected %02X%02X%02X\n", id[0], id[1], id[2]);
    }
    else {
        puts("OK");
    }

    /*
     * Erase the target device.
     */
    printf("Erasing device...    ");
    SpiFlashErase();
    puts("OK");

    printf("Programming device");
    addr = 0;
    for (;;) {
        if((addr & 0xFF) == 0)
            putchar('.');
        if((val = fgetc(fp)) == EOF) {
            puts("OK");
            break;
        }
        word = ((uint8_t)fgetc(fp) << 8) + (uint8_t)val;
        if (SpiFlashWriteWord(addr, word)) {
            printf("failed at %04X\n", addr);
            break;
        }
        addr++;
    }
    for(;;);
}
