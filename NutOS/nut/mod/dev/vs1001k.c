/*
 * Copyright (C) 2003 by Pavel Chromy. All rights reserved.
 * Copyright (C) 2001-2003 by egnite Software GmbH. All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY EGNITE SOFTWARE GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL EGNITE
 * SOFTWARE GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 * -
 *
 * This software has been inspired by all the valuable work done by
 * Jesper Hansen <jesperh@telia.com>. Many thanks for all his help.
 */

/*
 * $Log$
 * Revision 1.1  2003/07/21 18:09:51  haraldkipp
 * Backup of old driver
 *
 * Revision 1.1.1.1  2003/05/09 14:40:58  haraldkipp
 * Initial using 3.2.1
 *
 * Revision 1.12  2003/05/06 18:35:21  harald
 * ICCAVR port
 *
 * Revision 1.11  2003/04/21 16:43:54  harald
 * Added more comments.
 * Avoid initializing static globals to zero.
 * New function VsSdiWrite/_P checks DREQ
 * Removed decoder interrupt en/disable from low level routines.
 * Keep decoder in reset state until ports have been initialized.
 * Do not send initial zero bytes as the datasheet recommends.
 * A single nop is sufficient delay during reset active.
 * Clear interrupt flag after reset to avoid useless interrupt.
 * Available buffer size corrected.
 * New function to read header information.
 * New function invokes decoder memory test.
 * Beep makes use of VsSdiWrite.
 *
 * Revision 1.10  2003/04/18 14:46:08  harald
 * Copyright update by the maintainer, after none of the original code had
 * been left. We have a clean BSD licence now.
 * This release had been prepared by Pavel Chromy.
 * BSYNC vs. transfer in progress issue in VsSdiPutByte().
 * Fixed possible transfer in progress issue in VsPlayerFeed().
 * HW reset may be forced by VS_SM_RESET mode bit
 *
 * Revision 1.9  2003/04/07 20:29:20  harald
 * Redesigned by Pavel Chromy
 *
 * Revision 1.9  2003/04/04 15:01:00  mac
 * VS_STATUS_EMTY is reported correctly.
 *
 * Revision 1.9  2003/02/14 13:39:00  mac
 * Several serious bugs fixed,
 * interrupt routine completely remade.
 * Unreliable spurious interrupts detection removed.
 * Mpeg frame detection removed.
 * Watermark check removed (this was rather limiting)
 * Can be optionaly compiled not to use SPI
 *
 * Revision 1.8  2003/02/04 17:50:55  harald
 * Version 3 released
 *
 * Revision 1.7  2003/01/14 16:15:19  harald
 * Sending twice the number of zeros to end MP3 stream.
 * Check for spurious interrupts to detect hanging chip.
 * Simpler portable inline assembler for short delays.
 *
 * Revision 1.6  2002/11/02 15:15:13  harald
 * Library dependencies removed
 *
 * Revision 1.5  2002/09/15 16:44:14  harald
 * *** empty log message ***
 *
 * Revision 1.4  2002/08/16 17:49:02  harald
 * First public release
 *
 * Revision 1.3  2002/06/26 17:29:08  harald
 * First pre-release with 2.4 stack
 *
 */

#include <sys/atom.h>
#include <sys/event.h>
#include <sys/timer.h>
#include <sys/heap.h>

#include <dev/irqreg.h>
#include <dev/vs1001k.h>

/*!
 * \addtogroup xgVs1001
 */
/*@{*/


static volatile u_char vs_status = VS_STATUS_STOPPED;
static volatile u_short vs_flush;

static u_char *vs_databuff;
static u_char *vs_dataend;

static u_char *vs_writeptr;
static u_char *volatile vs_readptr;


/*
 * \brief Write a byte to the VS1001 data interface.
 *
 * The caller is responsible for checking the DREQ line. Also make sure,
 * that decoder interrupts are disabled.
 *
 * \param b Byte to be shifted to the decoder's data interface.
 */
static INLINE void VsSdiPutByte(u_char b)
{
#ifdef VS_NOSPI
    u_char mask = 0x80;
    sbi(VS_BSYNC_PORT, VS_BSYNC_BIT);
    while (mask) {
        if (b & mask)
            sbi(VS_SI_PORT, VS_SI_BIT);
        else
            cbi(VS_SI_PORT, VS_SI_BIT);

        sbi(VS_SS_PORT, VS_SS_BIT);
        mask >>= 1;
        cbi(VS_SS_PORT, VS_SS_BIT);
        cbi(VS_BSYNC_PORT, VS_BSYNC_BIT);
    }
#else
    /* Wait for previous SPI transfer to finish. */
    loop_until_bit_is_set(SPSR, SPIF);
    sbi(VS_BSYNC_PORT, VS_BSYNC_BIT);
    outp(b, SPDR);
    _NOP();
    _NOP();
    _NOP();
    _NOP();
    cbi(VS_BSYNC_PORT, VS_BSYNC_BIT);
#endif
}

/*!
 * \brief Write a specified number of bytes to the VS1001 data interface.
 *
 * This function will check the DREQ line. Decoder interrupts must have
 * been disabled before calling this function.
 */
static int VsSdiWrite(CONST u_char * data, u_short len)
{
    u_short try = 5000;

    while (len--) {
        while (try-- && bit_is_clear(VS_DREQ_PIN, VS_DREQ_BIT));
        VsSdiPutByte(*data);
        data++;
    }
    return try ? 0 : -1;
}

/*!
 * \brief Write a specified number of bytes from program space to the
 *        VS1001 data interface.
 *
 * This function is similar to VsSdiWrite() except that the data is
 * located in program space.
 */
static int VsSdiWrite_P(PGM_P data, u_short len)
{
    u_short try = 5000;

    while (len--) {
        while (try-- && bit_is_clear(VS_DREQ_PIN, VS_DREQ_BIT));
        VsSdiPutByte(PRG_RDB(data));
        data++;
    }
    return try ? 0 : -1;
}

/*!
 * \brief Write a byte to the serial control interface.
 *
 * Decoder interrupts must have been disabled and the decoder chip
 * select must have been enabled before calling this function.
 */
static INLINE void VsSciPutByte(u_char data)
{
    u_char mask = 0x80;

    /*
     * Loop until all 8 bits are processed.
     */
    while (mask) {

        /* Set data line. */
        if (data & mask)
            sbi(VS_SI_PORT, VS_SI_BIT);
        else
            cbi(VS_SI_PORT, VS_SI_BIT);

        /* Toggle clock and shift mask. */
        sbi(VS_SCK_PORT, VS_SCK_BIT);
        mask >>= 1;
        cbi(VS_SCK_PORT, VS_SCK_BIT);
    }
}

/*!
 * \brief Read a byte from the serial control interface.
 *
 * Decoder interrupts must have been disabled and the decoder chip
 * select must have been enabled before calling this function.
 */
static INLINE u_char VsSciGetByte(void)
{
    u_char mask = 0x80;
    u_char data = 0;

    /*
     * Loop until all 8 bits are processed.
     */
    while (mask) {
        /* Toggle clock and get the data. */
        sbi(VS_SCK_PORT, VS_SCK_BIT);
        if (bit_is_set(VS_SO_PIN, VS_SO_BIT))
            data |= mask;
        mask >>= 1;
        cbi(VS_SCK_PORT, VS_SCK_BIT);
    }
    return data;
}

/*!
 * \brief Write to a decoder register.
 *
 * Decoder interrupts must have been disabled before calling this function.
 */
static void VsRegWrite(u_char reg, u_short data)
{
    /* Select chip. */
    cbi(VS_XCS_PORT, VS_XCS_BIT);

#ifndef VS_NOSPI
    /* Disable SPI */
    cbi(SPCR, SPE);
#endif

    VsSciPutByte(VS_OPCODE_WRITE);
    VsSciPutByte(reg);
    VsSciPutByte((u_char) (data >> 8));
    VsSciPutByte((u_char) data);

#ifndef VS_NOSPI
    /* Re-enable SPI. Hint given by Jesper Hansen. */
    outp(BV(MSTR) | BV(SPE), SPCR);
    outp(inp(SPSR), SPSR);
#endif

    /* Deselect chip. */
    sbi(VS_XCS_PORT, VS_XCS_BIT);
}

/*
 * \brief Read from a register.
 *
 * Decoder interrupts must have been disabled before calling this function.
 *
 * \return Register contents.
 */
static u_short VsRegRead(u_char reg)
{
    u_short data;

    /* Disable interrupts and select chip. */
    cbi(EIMSK, VS_DREQ_BIT);
    cbi(VS_XCS_PORT, VS_XCS_BIT);

#ifndef VS_NOSPI
    /* Disable SPI. */
    cbi(SPCR, SPE);
#endif

    VsSciPutByte(VS_OPCODE_READ);
    VsSciPutByte(reg);
    data = VsSciGetByte() << 8;
    data |= VsSciGetByte();

#ifndef VS_NOSPI
    /* Re-enable SPI. Changed due to a hint by Jesper. */
    outp(BV(MSTR) | BV(SPE), SPCR);
    outp(inp(SPSR), SPSR);
#endif

    /* Deselect chip and enable interrupts. */
    sbi(VS_XCS_PORT, VS_XCS_BIT);
    sbi(EIMSK, VS_DREQ_BIT);

    return data;
}

/*
 * \brief Feed the decoder with data.
 *
 * This function serves two purposes:
 * - It is called by VsPlayerKick() to initially fill the decoder buffer.
 * - It is used as an interrupt handler for the decoder.
 */
static void VsPlayerFeed(void *arg)
{
    u_char j;

    /* Cancel interrupt if not running. */
    if (vs_status != VS_STATUS_RUNNING)
        return;

    /* We may feed 32 bytes to the decoder. */
    if (bit_is_set(VS_DREQ_PIN, VS_DREQ_BIT))
        j = 32;
    else
        j = 0;

    /*
     * Feed the decoder until its buffer is full.
     */
    while (j--) {

        /* Is main buffer empty? */
        if (vs_readptr == vs_writeptr) {
            if (vs_flush) {
                /* Auto flush the internal VS buffer. */
                VsSdiPutByte(0);
                if (--vs_flush == 0) {
                    /* Decoder internal buffer is flushed. */
                    vs_status = VS_STATUS_EMPTY;
                    break;
                }
            } else {
                /* End of stream. */
                vs_status = VS_STATUS_EOF;
                break;
            }
        }
        /* We have some data in the buffer, feed it. */
        else {
            VsSdiPutByte(*vs_readptr);
            if (++vs_readptr == vs_dataend)
                vs_readptr = vs_databuff;
        }

        /* If DREQ is high, allow 31 bytes to be sent, SPI transfer may
           still be in progress, which is the 32nd byte. */
        if (bit_is_set(VS_DREQ_PIN, VS_DREQ_BIT))
            j = 31;
    }
}

/*!
 * \brief Start playback.
 *
 * This routine will send the first MP3 data bytes to the
 * decoder, until it is completely filled. The data buffer
 * should have been filled before calling this routine.
 *
 * \return 0 on success, -1 otherwise.
 */
int VsPlayerKick(void)
{
    cbi(EIMSK, VS_DREQ_BIT);

    /*
     * Start feeding the decoder with data and
     * enable interrupts.
     */
    vs_status = VS_STATUS_RUNNING;
    VsPlayerFeed(NULL);

    sbi(EIMSK, VS_DREQ_BIT);
    return 0;
}

/*!
 * \brief Stops the playback.
 *
 * This routine will stops the MP3 playback, VsPlayerKick() may be used
 * to resume the playback.
 *
 * \return 0 on success, -1 otherwise.
 */
int VsPlayerStop(void)
{
    cbi(EIMSK, VS_DREQ_BIT);
    /* Check whether we need to stop at all to not overwrite other than running status */
    if (vs_status == VS_STATUS_RUNNING)
        vs_status = VS_STATUS_STOPPED;
    sbi(EIMSK, VS_DREQ_BIT);

    return 0;
}


/*!
 * \brief Sets up decoder internal buffer flushing.
 *
 * This routine will set up internal VS buffer flushing,
 * unless the buffer is already empty and starts the playback
 * if necessary. The internal VS buffer is flushed in VsPlayerFeed()
 * at the end of the stream.
 *
 * \return 0 on success, -1 otherwise.
 */
int VsPlayerFlush(void)
{
    cbi(EIMSK, VS_DREQ_BIT);
    /* Set up fluhing unless both buffers are empty. */
    if (vs_status != VS_STATUS_EMPTY || vs_readptr != vs_writeptr) {
        if (vs_flush == 0)
            vs_flush = VS_FLUSH_BYTES;
        /* start the playback if necessary */
        if (vs_status != VS_STATUS_RUNNING)
            VsPlayerKick();
    }
    sbi(EIMSK, VS_DREQ_BIT);
    return 0;
}


/*!
 * \brief Initialize the VS1001 hardware interface.
 *
 * \return 0 on success, -1 otherwise.
 */
int VsPlayerInit(void)
{
    /* Disable decoder interrupts. */
    cbi(EIMSK, VS_DREQ_BIT);

    /* Keep decoder in reset state. */
    cbi(VS_RESET_PORT, VS_RESET_BIT);
    sbi(VS_RESET_DDR, VS_RESET_BIT);

    /* Set BSYNC output low. */
    cbi(VS_BSYNC_PORT, VS_BSYNC_BIT);
    sbi(VS_BSYNC_DDR, VS_BSYNC_BIT);

    /* Set MP3 chip select output low. */
    sbi(VS_XCS_PORT, VS_XCS_BIT);
    sbi(VS_XCS_DDR, VS_XCS_BIT);

    /* Set DREQ input with pullup. */
    sbi(VS_DREQ_PORT, VS_DREQ_BIT);
    cbi(VS_DREQ_DDR, VS_DREQ_BIT);

    /* Init SPI Port. */
    sbi(VS_SI_DDR, VS_SI_BIT);
    sbi(VS_SS_DDR, VS_SS_BIT);
    cbi(VS_SO_DDR, VS_SO_BIT);

    /* Set SCK output low. */
    cbi(VS_SCK_PORT, VS_SCK_BIT);
    sbi(VS_SCK_DDR, VS_SCK_BIT);

#ifndef VS_NOSPI
    {
        u_char dummy;           /* Required by some compilers. */

        /*
         * Init SPI mode to no interrupts, enabled, MSB first, master mode,
         * rising clock and fosc/4 clock speed. Send an initial zero byte to
         * make sure SPIF is set. Note, that the decoder reset line is still
         * active.
         */
        outp(BV(MSTR) | BV(SPE), SPCR);
        dummy = inp(SPSR);
        outp(0, SPDR);
    }
#endif

    /* Rising edge will generate interrupts. */
    sbi(EICR, 5);
    sbi(EICR, 4);

    /* Register the interrupt routine */
    NutRegisterIrqHandler(&sig_INTERRUPT6, VsPlayerFeed, NULL);

    /* Release decoder reset line. */
    sbi(VS_RESET_PORT, VS_RESET_BIT);
    NutDelay(4);

    /* Force frequency change (see datasheet). */
    VsRegWrite(VS_CLOCKF_REG, 0x9800);
    VsRegWrite(VS_INT_FCTLH_REG, 0x8008);
    NutDelay(200);

    /* Clear any spurious interrupt and enable decoder interrupts. */
    outp(BV(VS_DREQ_BIT), EIFR);
    sbi(EIMSK, VS_DREQ_BIT);

    return 0;
}

/*!
 * \brief Software reset the decoder.
 *
 * This function is typically called after VsPlayerInit() and at the end
 * of each track.
 *
 * \param mode Any of the following flags may be or'ed
 * - VS_SM_DIFF Left channel inverted.
 * - VS_SM_FFWD Fast forward.
 * - VS_SM_RESET Force hardware reset.
 * - VS_SM_PDOWN Switch to power down mode.
 * - VS_SM_BASS Bass/treble enhancer.
 *
 * \return 0 on success, -1 otherwise.
 */
int VsPlayerReset(u_short mode)
{
    /* Disable decoder interrupts and feeding. */
    cbi(EIMSK, VS_DREQ_BIT);
    vs_status = VS_STATUS_STOPPED;

    /* Software reset, set modes of decoder. */
    VsRegWrite(VS_MODE_REG, VS_SM_RESET | mode);
    NutDelay(2);

    /*
     * Check for correct reset.
     */
    if ((mode & VS_SM_RESET) != 0 || bit_is_clear(VS_DREQ_PIN, VS_DREQ_BIT)) {
        /* If not succeeded, give it one more chance and try hw reset,
           HW reset may also be forced by VS_SM_RESET mode bit. */
        cbi(VS_RESET_PORT, VS_RESET_BIT);
        _NOP();
        sbi(VS_RESET_PORT, VS_RESET_BIT);
        NutDelay(4);

        /* Set the requested modes. */
        VsRegWrite(VS_MODE_REG, VS_SM_RESET | mode);
        NutDelay(2);
        if (bit_is_clear(VS_DREQ_PIN, VS_DREQ_BIT))
            return -1;
    }

    /* Force frequency change (see datasheet). */
    VsRegWrite(VS_CLOCKF_REG, 0x9800);
    VsRegWrite(VS_INT_FCTLH_REG, 0x8008);
    NutDelay(2);

    /* Clear any spurious interrupt and enable decoder interrupts. */
    outp(BV(VS_DREQ_BIT), EIFR);
    sbi(EIMSK, VS_DREQ_BIT);

    return 0;
}

/*!
 * \brief Set mode register of the decoder.
 *
 * \param mode Any of the following flags may be or'ed
 * - VS_SM_DIFF Left channel inverted.
 * - VS_SM_FFWD Fast forward.
 * - VS_SM_RESET Software reset.
 * - VS_SM_PDOWN Switch to power down mode.
 * - VS_SM_BASS Bass/treble enhancer.
 *
 * \return 0 on success, -1 otherwise.
 */
int VsPlayerSetMode(u_short mode)
{
    cbi(EIMSK, VS_DREQ_BIT);
    VsRegWrite(VS_MODE_REG, mode);
    sbi(EIMSK, VS_DREQ_BIT);

    return 0;
}

/*!
 * \brief Initialize the MP3 data buffer.
 *
 * \param size Number of bytes to allocate for the data buffer.
 *             Should be at least 4k. If this parameter is 0,
 *             all available memory minus 8k are allocated.
 *
 * \return Pointer to the data buffer or null on failures.
 */
u_char *VsBufferInit(u_short size)
{
    if (size == 0)
        size = NutHeapAvailable() - 8192;
    if ((vs_databuff = NutHeapAlloc(size)) != 0)
        vs_dataend = vs_databuff + size;

    vs_readptr = vs_writeptr = vs_databuff;
    return vs_writeptr;
}

/*!
 * \brief Reset all MP3 data buffer pointers.
 *
 * \return Pointer to the data buffer.
 */
u_char *VsBufferReset(void)
{
    cbi(EIMSK, VS_DREQ_BIT);
    vs_readptr = vs_writeptr = vs_databuff;
    sbi(EIMSK, VS_DREQ_BIT);

    return vs_databuff;
}

/*!
 * \brief Request MP3 data buffer space.
 *
 * \param sizep Pointer to an unsigned short, which receives the
 *              number of bytes available in the buffer.
 *
 * \return Pointer to the next write position.
 */
u_char *VsBufferRequest(u_short * sizep)
{
    cbi(EIMSK, VS_DREQ_BIT);
    if (vs_writeptr >= vs_readptr)
        *sizep = vs_dataend - vs_writeptr - (vs_readptr == vs_databuff);
    else
        *sizep = vs_readptr - vs_writeptr - 1;
    sbi(EIMSK, VS_DREQ_BIT);

    return vs_writeptr;
}

/*!
 * \brief Acknowledge filled buffer space.
 *
 * \return Pointer to the next write position.
 */
u_char *VsBufferAcknowledge(u_short nbytes)
{
    cbi(EIMSK, VS_DREQ_BIT);
    vs_writeptr += nbytes;
    if (vs_writeptr == vs_dataend)
        vs_writeptr = vs_databuff;
    /* Cancel flushing in progress. */
    vs_flush = 0;
    sbi(EIMSK, VS_DREQ_BIT);

    return vs_writeptr;
}


/*!
 * \brief Returns total free buffer space.
 *
 * \return Total number of free bytes in the buffer.
 */
u_short VsBufferAvailable(void)
{
    u_short avail;

    cbi(EIMSK, VS_DREQ_BIT);
    if (vs_writeptr >= vs_readptr)
        avail = (vs_dataend - vs_databuff) - (vs_writeptr - vs_readptr);
    else
        avail = (vs_readptr - vs_writeptr) - 1;
    sbi(EIMSK, VS_DREQ_BIT);

    return avail;
}

/*!
 * \brief Returns play time since last reset.
 *
 * \return Play time since reset in seconds
 */
u_short VsPlayTime(void)
{
    u_short rc;

    cbi(EIMSK, VS_DREQ_BIT);
    rc = VsRegRead(VS_DECODE_TIME_REG);
    sbi(EIMSK, VS_DREQ_BIT);

    return rc;
}

/*!
 * \brief Returns status of the player.
 *
 * \return Any of the following value:
 * - VS_STATUS_STOPPED Player is ready to be started by VsPlayerKick().
 * - VS_STATUS_RUNNING Player is running.
 * - VS_STATUS_EOF Player has reached the end of a stream after VsPlayerFlush() has been called.
 * - VS_STATUS_EMPTY Player runs out of data. VsPlayerKick() will restart it.
 */
u_char VsGetStatus(void)
{
    return vs_status;
}

#ifdef __GNUC__

/*!
 * \brief Query MP3 stream header information.
 *
 * \param vshi Pointer to VS_HEADERINFO structure.
 *
 * \return 0 on success, -1 otherwise.
 */
int VsGetHeaderInfo(VS_HEADERINFO * vshi)
{
    u_short *usp = (u_short *) vshi;

    cbi(EIMSK, VS_DREQ_BIT);
    *usp = VsRegRead(VS_HDAT1_REG);
    *++usp = VsRegRead(VS_HDAT0_REG);
    sbi(EIMSK, VS_DREQ_BIT);

    return 0;
}
#endif

/*!
 * \brief Initialize decoder memory test and return result.
 *
 * \return Memory test result.
 * - Bit 0: Good X ROM
 * - Bit 1: Good Y ROM (high)
 * - Bit 2: Good Y ROM (low)
 * - Bit 3: Good Y RAM
 * - Bit 4: Good X RAM
 * - Bit 5: Good Instruction RAM (high)
 * - Bit 6: Good Instruction RAM (low)
 */
u_short VsMemoryTest(void)
{
    u_short rc;
    static prog_char mtcmd[] = { 0x4D, 0xEA, 0x6D, 0x54, 0x00, 0x00, 0x00, 0x00 };

    cbi(EIMSK, VS_DREQ_BIT);
    VsSdiWrite_P(mtcmd, sizeof(mtcmd));
    NutDelay(40);
    rc = VsRegRead(VS_HDAT0_REG);
    sbi(EIMSK, VS_DREQ_BIT);

    return rc;
}

/*!
 * \brief Set volume.
 *
 * \param left  Left channel volume.
 * \param right Right channel volume.
 *
 * \return 0 on success, -1 otherwise.
 */
int VsSetVolume(u_char left, u_char right)
{
    cbi(EIMSK, VS_DREQ_BIT);
    VsRegWrite(VS_VOL_REG, (((u_short) left) << 8) | (u_short) right);
    sbi(EIMSK, VS_DREQ_BIT);

    return 0;
}

/*!
 * \brief Sine wave beep.
 *
 * \param fsin Frequency.
 * \param ms   Duration.
 *
 * \return 0 on success, -1 otherwise.
 */
int VsBeep(u_char fsin, u_char ms)
{
    static prog_char on[] = { 0x53, 0xEF, 0x6E };
    static prog_char off[] = { 0x45, 0x78, 0x69, 0x74 };
    static prog_char end[] = { 0x00, 0x00, 0x00, 0x00 };

    /* Disable decoder interrupts. */
    cbi(EIMSK, VS_DREQ_BIT);

    fsin = 56 + (fsin & 7) * 9;
    VsSdiWrite_P(on, sizeof(on));
    VsSdiWrite(&fsin, 1);
    VsSdiWrite_P(end, sizeof(end));
    NutDelay(ms);
    VsSdiWrite_P(off, sizeof(off));
    VsSdiWrite_P(end, sizeof(end));

    /* Enable decoder interrupts. */
    sbi(EIMSK, VS_DREQ_BIT);

    return 0;
}


/*@}*/
