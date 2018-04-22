/*
 * Copyright (C) 2016 by Uwe Bonnes bon@elektron.ikp.physik.tu-dadrmstadt.de
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
 * \file dev/mux_7seg.c
 * \brief Routines for multiplexed 7 segment driver
 *
 * Tested on X-NUCLEO-6180X. X-NUCLEO-6180XA1 has I2C connection.
 *
 */

/*
 * This file is for multiplexed 7-segment displays
 *
 */
#include <cfg/os.h>
#include <cfg/arch.h>
#include <compiler.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/nutdebug.h>
#include <sys/timer.h>
#include <sys/event.h>

#include <dev/gpio.h>
#include <dev/term.h>

#include <cfg/mux_7seg.h>
#include <font/seg7.h>

#undef GPIO_ID
#if defined(SEG7_A_PORT) && defined(SEG7_A_PIN)
# define GPIO_ID SEG7_A_PORT
# include <cfg/arch/porttran.h>
static INLINE void SEGMENT_A_INIT(void)
{ GPIO_ENABLE(SEG7_A_PIN); GPIO_OUTPUT(SEG7_A_PIN);  }
# if defined(SEG7_SEG_NEG)
static INLINE void SEGMENT_A_SET(void)  { GPIO_SET_LO(SEG7_A_PIN); }
static INLINE void SEGMENT_A_CLR(void)  { GPIO_SET_HI(SEG7_A_PIN); }
# else
static INLINE void SEGMENT_A_SET(void)  { GPIO_SET_HI(SEG7_A_PIN); }
static INLINE void SEGMENT_A_CLR(void)  { GPIO_SET_LO(SEG7_A_PIN); }
# endif
#else
# define SEGMENT_A_INIT()
# define SEGMENT_A_SET()
# define SEGMENT_A_CLR()
#endif

#undef GPIO_ID
#if defined(SEG7_B_PORT) && defined(SEG7_B_PIN)
# define GPIO_ID SEG7_B_PORT
# include <cfg/arch/porttran.h>
static INLINE void SEGMENT_B_INIT(void)
{ GPIO_ENABLE(SEG7_B_PIN); GPIO_OUTPUT(SEG7_B_PIN);  }
# if defined(SEG7_SEG_NEG)
static INLINE void SEGMENT_B_SET(void)  { GPIO_SET_LO(SEG7_B_PIN); }
static INLINE void SEGMENT_B_CLR(void)  { GPIO_SET_HI(SEG7_B_PIN); }
# else
static INLINE void SEGMENT_B_SET(void)  { GPIO_SET_HI(SEG7_B_PIN); }
static INLINE void SEGMENT_B_CLR(void)  { GPIO_SET_LO(SEG7_B_PIN); }
# endif
#else
# define SEGMENT_B_INIT()
# define SEGMENT_B_SET()
# define SEGMENT_B_CLR()
#endif

#undef GPIO_ID
#if defined(SEG7_C_PORT) && defined(SEG7_C_PIN)
# define GPIO_ID SEG7_C_PORT
# include <cfg/arch/porttran.h>
static INLINE void SEGMENT_C_INIT(void)
{ GPIO_ENABLE(SEG7_C_PIN); GPIO_OUTPUT(SEG7_C_PIN);  }
# if defined(SEG7_SEG_NEG)
static INLINE void SEGMENT_C_SET(void)  { GPIO_SET_LO(SEG7_C_PIN); }
static INLINE void SEGMENT_C_CLR(void)  { GPIO_SET_HI(SEG7_C_PIN); }
# else
static INLINE void SEGMENT_C_SET(void)  { GPIO_SET_HI(SEG7_C_PIN); }
static INLINE void SEGMENT_C_CLR(void)  { GPIO_SET_LO(SEG7_C_PIN); }
# endif
#else
# define SEGMENT_C_INIT()
# define SEGMENT_C_SET()
# define SEGMENT_C_CLR()
#endif

#undef GPIO_ID
#if defined(SEG7_D_PORT) && defined(SEG7_D_PIN)
# define GPIO_ID SEG7_D_PORT
# include <cfg/arch/porttran.h>
static INLINE void SEGMENT_D_INIT(void)
{ GPIO_ENABLE(SEG7_D_PIN); GPIO_OUTPUT(SEG7_D_PIN);  }
# if defined(SEG7_SEG_NEG)
static INLINE void SEGMENT_D_SET(void)  { GPIO_SET_LO(SEG7_D_PIN); }
static INLINE void SEGMENT_D_CLR(void)  { GPIO_SET_HI(SEG7_D_PIN); }
# else
static INLINE void SEGMENT_D_SET(void)  { GPIO_SET_HI(SEG7_D_PIN); }
static INLINE void SEGMENT_D_CLR(void)  { GPIO_SET_LO(SEG7_D_PIN); }
# endif
#else
# define SEGMENT_D_INIT()
# define SEGMENT_D_SET()
# define SEGMENT_D_CLR()
#endif

#undef GPIO_ID
#if defined(SEG7_E_PORT) && defined(SEG7_E_PIN)
# define GPIO_ID SEG7_E_PORT
# include <cfg/arch/porttran.h>
static INLINE void SEGMENT_E_INIT(void)
{ GPIO_ENABLE(SEG7_E_PIN); GPIO_OUTPUT(SEG7_E_PIN);  }
# if defined(SEG7_SEG_NEG)
static INLINE void SEGMENT_E_SET(void)  { GPIO_SET_LO(SEG7_E_PIN); }
static INLINE void SEGMENT_E_CLR(void)  { GPIO_SET_HI(SEG7_E_PIN); }
# else
static INLINE void SEGMENT_E_SET(void)  { GPIO_SET_HI(SEG7_E_PIN); }
static INLINE void SEGMENT_E_CLR(void)  { GPIO_SET_LO(SEG7_E_PIN); }
# endif
#else
# define SEGMENT_E_INIT()
# define SEGMENT_E_SET()
# define SEGMENT_E_CLR()
#endif

#undef GPIO_ID
#if defined(SEG7_F_PORT) && defined(SEG7_F_PIN)
# define GPIO_ID SEG7_F_PORT
# include <cfg/arch/porttran.h>
static INLINE void SEGMENT_F_INIT(void)
{ GPIO_ENABLE(SEG7_F_PIN); GPIO_OUTPUT(SEG7_F_PIN);  }
# if defined(SEG7_SEG_NEG)
static INLINE void SEGMENT_F_SET(void)  { GPIO_SET_LO(SEG7_F_PIN); }
static INLINE void SEGMENT_F_CLR(void)  { GPIO_SET_HI(SEG7_F_PIN); }
# else
static INLINE void SEGMENT_F_SET(void)  { GPIO_SET_HI(SEG7_F_PIN); }
static INLINE void SEGMENT_F_CLR(void)  { GPIO_SET_LO(SEG7_F_PIN); }
# endif
#else
# define SEGMENT_F_INIT()
# define SEGMENT_F_SET()
# define SEGMENT_F_CLR()
#endif

#undef GPIO_ID
#if defined(SEG7_G_PORT) && defined(SEG7_G_PIN)
# define GPIO_ID SEG7_G_PORT
# include <cfg/arch/porttran.h>
static INLINE void SEGMENT_G_INIT(void)
{ GPIO_ENABLE(SEG7_G_PIN); GPIO_OUTPUT(SEG7_G_PIN);  }
# if defined(SEG7_SEG_NEG)
static INLINE void SEGMENT_G_SET(void)  { GPIO_SET_LO(SEG7_G_PIN); }
static INLINE void SEGMENT_G_CLR(void)  { GPIO_SET_HI(SEG7_G_PIN); }
# else
static INLINE void SEGMENT_G_SET(void)  { GPIO_SET_HI(SEG7_G_PIN); }
static INLINE void SEGMENT_G_CLR(void)  { GPIO_SET_LO(SEG7_G_PIN); }
# endif
#else
# define SEGMENT_G_INIT()
# define SEGMENT_G_SET()
# define SEGMENT_G_CLR()
#endif

#undef GPIO_ID
#if defined(SEG7_DP_PORT) && defined(SEG7_DP_PIN)
# define GPIO_ID SEG7_DP_PORT
# include <cfg/arch/porttran.h>
static INLINE void SEGMENT_DP_INIT(void)
{ GPIO_ENABLE(SEG7_DP_PIN); GPIO_OUTPUT(SEG7_DP_PIN);  }
# if defined(SEG7_SEG_NEG)
static INLINE void SEGMENT_DP_SET(void)  { GPIO_SET_LO(SEG7_DP_PIN); }
static INLINE void SEGMENT_DP_CLR(void)  { GPIO_SET_HI(SEG7_DP_PIN); }
# else
static INLINE void SEGMENT_DP_SET(void)  { GPIO_SET_HI(SEG7_DP_PIN); }
static INLINE void SEGMENT_DP_CLR(void)  { GPIO_SET_LO(SEG7_DP_PIN); }
# endif
#else
# define SEGMENT_DP_INIT()
# define SEGMENT_DP_SET()
# define SEGMENT_DP_CLR()
#endif

#undef GPIO_ID
#if defined(SEG7_DIGIT0_PORT) && defined(SEG7_DIGIT0_PIN)
# define GPIO_ID SEG7_DIGIT0_PORT
# include <cfg/arch/porttran.h>
static INLINE int SEGMENT_DIGIT0_INIT(void)
{ GPIO_ENABLE(SEG7_DIGIT0_PIN); GPIO_OUTPUT(SEG7_DIGIT0_PIN);  return 1;}
# if defined(SEG7_DIGIT_NEG)
static INLINE void SEGMENT_DIGIT0_SET(void)  { GPIO_SET_LO(SEG7_DIGIT0_PIN); }
static INLINE void SEGMENT_DIGIT0_CLR(void)  { GPIO_SET_HI(SEG7_DIGIT0_PIN); }
# else
static INLINE void SEGMENT_DIGIT0_SET(void)  { GPIO_SET_HI(SEG7_DIGIT0_PIN); }
static INLINE void SEGMENT_DIGIT0_CLR(void)  { GPIO_SET_LO(SEG7_DIGIT0_PIN); }
# endif
#else
# define SEGMENT_DIGIT0_INIT() 0
# define SEGMENT_DIGIT0_SET()
# define SEGMENT_DIGIT0_CLR()
#endif

#undef GPIO_ID
#if defined(SEG7_DIGIT1_PORT) && defined(SEG7_DIGIT1_PIN)
# define GPIO_ID SEG7_DIGIT1_PORT
# include <cfg/arch/porttran.h>
static INLINE int SEGMENT_DIGIT1_INIT(void)
{ GPIO_ENABLE(SEG7_DIGIT1_PIN); GPIO_OUTPUT(SEG7_DIGIT1_PIN); return 2; }
# if defined(SEG7_DIGIT_NEG)
static INLINE void SEGMENT_DIGIT1_SET(void)  { GPIO_SET_LO(SEG7_DIGIT1_PIN); }
static INLINE void SEGMENT_DIGIT1_CLR(void)  { GPIO_SET_HI(SEG7_DIGIT1_PIN); }
# else
static INLINE void SEGMENT_DIGIT1_SET(void)  { GPIO_SET_HI(SEG7_DIGIT1_PIN); }
static INLINE void SEGMENT_DIGIT1_CLR(void)  { GPIO_SET_LO(SEG7_DIGIT1_PIN); }
# endif
#else
# define SEGMENT_DIGIT1_INIT() 0
# define SEGMENT_DIGIT1_SET()
# define SEGMENT_DIGIT1_CLR()
#endif

#undef GPIO_ID
#if defined(SEG7_DIGIT2_PORT) && defined(SEG7_DIGIT2_PIN)
# define GPIO_ID SEG7_DIGIT2_PORT
# include <cfg/arch/porttran.h>
static INLINE int  SEGMENT_DIGIT2_INIT(void)
{ GPIO_ENABLE(SEG7_DIGIT2_PIN); GPIO_OUTPUT(SEG7_DIGIT2_PIN); return 3; }
# if defined(SEG7_DIGIT_NEG)
static INLINE void SEGMENT_DIGIT2_SET(void)  { GPIO_SET_LO(SEG7_DIGIT2_PIN); }
static INLINE void SEGMENT_DIGIT2_CLR(void)  { GPIO_SET_HI(SEG7_DIGIT2_PIN); }
# else
static INLINE void SEGMENT_DIGIT2_SET(void)  { GPIO_SET_HI(SEG7_DIGIT2_PIN); }
static INLINE void SEGMENT_DIGIT2_CLR(void)  { GPIO_SET_LO(SEG7_DIGIT2_PIN); }
# endif
#else
# define SEGMENT_DIGIT2_INIT() 0
# define SEGMENT_DIGIT2_SET()
# define SEGMENT_DIGIT2_CLR()
#endif

#undef GPIO_ID
#if defined(SEG7_DIGIT3_PORT) && defined(SEG7_DIGIT3_PIN)
# define GPIO_ID SEG7_DIGIT3_PORT
# include <cfg/arch/porttran.h>
static INLINE int  SEGMENT_DIGIT3_INIT(void)
{ GPIO_ENABLE(SEG7_DIGIT3_PIN); GPIO_OUTPUT(SEG7_DIGIT3_PIN);  return 4;  }
# if defined(SEG7_DIGIT_NEG)
static INLINE void SEGMENT_DIGIT3_SET(void)  { GPIO_SET_LO(SEG7_DIGIT3_PIN); }
static INLINE void SEGMENT_DIGIT3_CLR(void)  { GPIO_SET_HI(SEG7_DIGIT3_PIN); }
# else
static INLINE void SEGMENT_DIGIT3_SET(void)  { GPIO_SET_HI(SEG7_DIGIT3_PIN); }
static INLINE void SEGMENT_DIGIT3_CLR(void)  { GPIO_SET_LO(SEG7_DIGIT3_PIN); }
# endif
#else
# define SEGMENT_DIGIT3_INIT() 0
# define SEGMENT_DIGIT3_SET()
# define SEGMENT_DIGIT3_CLR()
#endif

#undef GPIO_ID
#if defined(SEG7_DIGIT4_PORT) && defined(SEG7_DIGIT4_PIN)
# define GPIO_ID SEG7_DIGIT4_PORT
# include <cfg/arch/porttran.h>
static INLINE int  SEGMENT_DIGIT4_INIT(void)
{ GPIO_ENABLE(SEG7_DIGIT4_PIN); GPIO_OUTPUT(SEG7_DIGIT4_PIN); return 5; }
# if defined(SEG7_DIGIT_NEG)
static INLINE void SEGMENT_DIGIT4_SET(void)  { GPIO_SET_LO(SEG7_DIGIT4_PIN); }
static INLINE void SEGMENT_DIGIT4_CLR(void)  { GPIO_SET_HI(SEG7_DIGIT4_PIN); }
# else
static INLINE void SEGMENT_DIGIT4_SET(void)  { GPIO_SET_HI(SEG7_DIGIT4_PIN); }
static INLINE void SEGMENT_DIGIT4_CLR(void)  { GPIO_SET_LO(SEG7_DIGIT4_PIN); }
# endif
#else
# define SEGMENT_DIGIT4_INIT() 0
# define SEGMENT_DIGIT4_SET()
# define SEGMENT_DIGIT4_CLR()
#endif

#undef GPIO_ID
#if defined(SEG7_DIGIT5_PORT) && defined(SEG7_DIGIT5_PIN)
# define GPIO_ID SEG7_DIGIT5_PORT
# include <cfg/arch/porttran.h>
static INLINE int  SEGMENT_DIGIT5_INIT(void)
{ GPIO_ENABLE(SEG7_DIGIT5_PIN); GPIO_OUTPUT(SEG7_DIGIT5_PIN);  return 6; }
# if defined(SEG7_DIGIT_NEG)
static INLINE void SEGMENT_DIGIT5_SET(void)  { GPIO_SET_LO(SEG7_DIGIT5_PIN); }
static INLINE void SEGMENT_DIGIT5_CLR(void)  { GPIO_SET_HI(SEG7_DIGIT5_PIN); }
# else
static INLINE void SEGMENT_DIGIT5_SET(void)  { GPIO_SET_HI(SEG7_DIGIT5_PIN); }
static INLINE void SEGMENT_DIGIT5_CLR(void)  { GPIO_SET_LO(SEG7_DIGIT5_PIN); }
# endif
#else
# define SEGMENT_DIGIT5_INIT() 0
# define SEGMENT_DIGIT5_SET()
# define SEGMENT_DIGIT5_CLR()
#endif

#undef GPIO_ID
#if defined(SEG7_DIGIT6_PORT) && defined(SEG7_DIGIT6_PIN)
# define GPIO_ID SEG7_DIGIT6_PORT
# include <cfg/arch/porttran.h>
static INLINE int  SEGMENT_DIGIT6_INIT(void)
{ GPIO_ENABLE(SEG7_DIGIT6_PIN); GPIO_OUTPUT(SEG7_DIGIT6_PIN);  return 7;}
# if defined(SEG7_DIGIT_NEG)
static INLINE void SEGMENT_DIGIT6_SET(void)  { GPIO_SET_LO(SEG7_DIGIT6_PIN); }
static INLINE void SEGMENT_DIGIT6_CLR(void)  { GPIO_SET_HI(SEG7_DIGIT6_PIN); }
# else
static INLINE void SEGMENT_DIGIT6_SET(void)  { GPIO_SET_HI(SEG7_DIGIT6_PIN); }
static INLINE void SEGMENT_DIGIT6_CLR(void)  { GPIO_SET_LO(SEG7_DIGIT6_PIN); }
# endif
#else
# define SEGMENT_DIGIT6_INIT() 0
# define SEGMENT_DIGIT6_SET()
# define SEGMENT_DIGIT6_CLR()
#endif

#undef GPIO_ID
#if defined(SEG7_DIGIT7_PORT) && defined(SEG7_DIGIT7_PIN)
# define GPIO_ID SEG7_DIGIT7_PORT
# include <cfg/arch/porttran.h>
static INLINE int  SEGMENT_DIGIT7_INIT(void)
{ GPIO_ENABLE(SEG7_DIGIT7_PIN); GPIO_OUTPUT(SEG7_DIGIT7_PIN);  return 8;}
# if defined(SEG7_DIGIT_NEG)
static INLINE void SEGMENT_DIGIT7_SET(void)  { GPIO_SET_LO(SEG7_DIGIT7_PIN); }
static INLINE void SEGMENT_DIGIT7_CLR(void)  { GPIO_SET_HI(SEG7_DIGIT7_PIN); }
# else
static INLINE void SEGMENT_DIGIT7_SET(void)  { GPIO_SET_HI(SEG7_DIGIT7_PIN); }
static INLINE void SEGMENT_DIGIT7_CLR(void)  { GPIO_SET_LO(SEG7_DIGIT7_PIN); }
# endif
#else
# define SEGMENT_DIGIT7_INIT() 0
# define SEGMENT_DIGIT7_SET()
# define SEGMENT_DIGIT7_CLR()
#endif

#define SEG7_DIGITS 8
#define ICMD_ESCAPE     0x80

/*!
 * \brief Device Control Block for 16-Segment Display
 */
typedef struct {
    uint8_t encoded_data[SEG7_DIGITS]; /*!< Buffer for Display content */
    uint_fast8_t num_digits;         /*!< Number of active digits.   */
    uint_fast8_t current_digit;      /*!< Current acctive displayed digit. */
    uint_fast8_t dip;                /*!< Actual cursor position. */
    uint_fast8_t icmd;               /*!< Internal driver control register,
                                      *   lower 7 bits for blinking. */
    uint_fast8_t blink_cnt;          /*!< Counter for blink action. */
} DCB_MUX7SEG;


/*!
 * \brief Generate File Handle for multiplexed 7-Segment Display.
 *
 * \param dev Specifies the 16seg device.
 *
 * \return 0 on success or -1 if no valid 16seg was found.
 */
NUTFILE *Mux7SegOpen(NUTDEVICE * dev, const char *name, int mode, int acc)
{
    NUTFILE *fp;

    NUTASSERT( dev != NULL);

    if ((fp = malloc(sizeof(NUTFILE))) == 0) {
        return NUTFILE_EOF;
    }

    fp->nf_dev = dev;
    fp->nf_fcb = NULL;

    return fp;
}

/*!
 * \brief Send single character to 7-Segment Display.
 *
 * A newline character will reset write pointer to digit 0.
 * Carriage return is ignored.
 *
 * \return Number of characters sent.
 */
static int Mux7SegPutc(NUTDEVICE * dev, char c)
{
    DCB_MUX7SEG * dcb;

    dcb  = dev->dev_dcb;
    if (dcb->icmd & ICMD_ESCAPE) {

        dcb->icmd &= ~ICMD_ESCAPE;
        /* Handle ESC sequences */
        switch (c) {
            case 'b':       /* blink slow */
                dcb->icmd |= 50;
                return 0;
                break;
            case 'f':       /* blink fast */
                dcb->icmd |= 12;
                return 0;
                break;
            case 'n':       /* stop blinking */
                dcb->icmd |= 0x7f;
                return 0;
                break;
            case 'h':       /* home */
                dcb->dip = 0;
                return 0;
                break;
            case 'c':       /* clear */
                memset( dcb->encoded_data, 0, SEG7_DIGITS);
                break;
            case 't':       /* test, all digits on */
                memset( dcb->encoded_data, 0xFF, SEG7_DIGITS);
                break;
            default:
                break;
        }

    } else {
        /* Non-ESC Character incoming */

        /* Start ESC Sequence? */
        if (c == 0x1b) {
            dcb->icmd |= ICMD_ESCAPE;
            return 0;
        }

        if (c == '\n' ) {  /* Return to Digit 0 */
            dcb->dip = 0;
            NutSleep(500);
            return 0;
        }

        if (c == '.') {
            /* Add decimal point to previous digit */
            if (dcb->dip > 0) {
                dcb->encoded_data[dcb->dip - 1] |= 0x80;
            }
        } else if (c >= ' ') {
            uint8_t data;
            if (c > 0x5F) {
                c -= 0x20;  /* convert lower case to upper case */
            }
            data = Seg7CharTab[(c & 0xff)-' '];
            if (dcb->dip < dcb->num_digits) {
                dcb->encoded_data[dcb->dip] = data;
                dcb->dip++;
            } else {
                /* scroll data */
                NutSleep(500);
                uint8_t i;
                for( i = 1 ; i < dcb->num_digits; i++) {
                    dcb->encoded_data[i - 1] = dcb->encoded_data[i];
                }
                dcb->encoded_data[dcb->num_digits - 1] = data;
            }
        }
    }

    return 0;
}

/*!
 * \brief Handle I/O controls for 7-Segment Display.
 *
 * The 7-Seg Display is controlled by ESC-Sequences only.
 *
 * \return 0.
 */
static int Mux7SegIOCtl(NUTDEVICE * dev, int req, void *conf)
{
    WINSIZE *win_size;
    DCB_MUX7SEG * dcb;

    dcb  = dev->dev_dcb;
    switch (req) {
    case TIOCGWINSZ:
        win_size = (WINSIZE *)conf;
        win_size->ws_col    = 1;
        win_size->ws_row    = dcb->num_digits;
        win_size->ws_xpixel = 0;
        win_size->ws_ypixel = 0;
        break;
    }
    return 0;
}

/*!
 * \brief Send characters to 7-Segment Display.
 *
 * A newline character will reset write pointer to digit 0.
 * Carriage return is ignored.
 *
 * \return Number of characters sent.
 */
int Mux7SegWrite(NUTFILE * fp, const void *buffer, int len)
{
    int i=len;
    const char *cp = buffer;

    NUTASSERT(fp->nf_dev != NULL);

    while (i--)
    {
        Mux7SegPutc( fp->nf_dev, *cp++);
    }
    return len;
}

/*!
 * \brief Close 7-Segment Device.
 *
 * \return 0 if closed and was opened before, else -1.
 */
static int Mux7SegClose(NUTFILE * fp)
{
    if (fp != NULL) {
        free( fp);
        return 0;
    }
    return -1;
}

/*!
 * \brief Timer function for display update
 *
 * Running with 80 .. 120 Hz.
 *
 * \return 0 if closed and was opened before, else -1.
 */
void Mux7SegUpdate(HANDLE timer, void *arg)
{
    DCB_MUX7SEG *dcb = (DCB_MUX7SEG *)arg;
    uint8_t data;
    uint8_t last_digit;
    uint8_t blink_limit;

    blink_limit =  dcb->icmd & 0x7f;
    if (!dcb->current_digit) {
        last_digit = dcb->num_digits - 1;
    } else {
        last_digit = dcb->current_digit -1;
    }
    switch (last_digit) {
    case 0:
        SEGMENT_DIGIT0_CLR();
        break;
    case 1:
        SEGMENT_DIGIT1_CLR();
        break;
    case 2:
        SEGMENT_DIGIT2_CLR();
        break;
    case 3:
        SEGMENT_DIGIT3_CLR();
        break;
    case 4:
        SEGMENT_DIGIT4_CLR();
        break;
    case 5:
        SEGMENT_DIGIT5_CLR();
        break;
    case 6:
        SEGMENT_DIGIT6_CLR();
        break;
    case 7:
        SEGMENT_DIGIT7_CLR();
        break;
    }

    if (dcb->blink_cnt > blink_limit) {
        goto next_digit;
    }
    data = dcb->encoded_data[dcb->current_digit];
    if (data & 0x40) {
        SEGMENT_A_SET();
    } else {
        SEGMENT_A_CLR();
    }
    if (data & 0x20) {
        SEGMENT_B_SET();
    } else {
        SEGMENT_B_CLR();
    }
    if (data & 0x10) {
        SEGMENT_C_SET();
    } else {
        SEGMENT_C_CLR();
    }
    if (data & 0x08) {
        SEGMENT_D_SET();
    } else {
        SEGMENT_D_CLR();
    }
    if (data & 0x04) {
        SEGMENT_E_SET();
    } else {
        SEGMENT_E_CLR();
    }
    if (data & 0x02) {
        SEGMENT_F_SET();
    } else {
        SEGMENT_F_CLR();
    }
    if (data & 0x01) {
        SEGMENT_G_SET();
    } else {
        SEGMENT_G_CLR();
    }
    if (data & 0x80) {
        SEGMENT_DP_SET();
    } else {
        SEGMENT_DP_CLR();
    }
    switch (dcb->current_digit) {
    case 0:
        SEGMENT_DIGIT0_SET();
        break;
    case 1:
        SEGMENT_DIGIT1_SET();
        break;
    case 2:
        SEGMENT_DIGIT2_SET();
        break;
    case 3:
        SEGMENT_DIGIT3_SET();
        break;
    case 4:
        SEGMENT_DIGIT4_SET();
        break;
    case 5:
        SEGMENT_DIGIT5_SET();
        break;
    case 6:
        SEGMENT_DIGIT6_SET();
        break;
    case 7:
        SEGMENT_DIGIT7_SET();
        break;
    }
next_digit:
    if (dcb->current_digit < (dcb->num_digits - 1)) {
        dcb->current_digit++;
    } else {
        dcb->current_digit = 0;
        if (blink_limit) {
            if (dcb->blink_cnt >= blink_limit * 2 ) {
                dcb->blink_cnt = 0;
            } else {
                dcb->blink_cnt ++;
            }
        }
    }
}

/*********************************************************************************
 **
 **         7-Segment Driver: Initialization & Device Description
 **
 **/

/*!
 * \brief Initialize the LCD 16 Seg device.
 *
 * This routine determines the 16seg type. It is internally called
 * by Nut/OS during device registration.
 *
 * The driver framework may call this function more than once.
 *
 * \param dev Specifies the Mux7Seg device.
 *
 * \return 0 on success or -1 if no valid 16seg was found.
 */
int Mux7SegInit(NUTDEVICE * dev)
{
    DCB_MUX7SEG *dcb;
    int i, j;
    uint8_t nr_digits2ticks[SEG7_DIGITS] = {10, 5, 3, 2, 2, 2, 1, 1};
    uint32_t ticks;

    SEGMENT_A_INIT();
    SEGMENT_B_INIT();
    SEGMENT_C_INIT();
    SEGMENT_D_INIT();
    SEGMENT_E_INIT();
    SEGMENT_F_INIT();
    SEGMENT_G_INIT();
    SEGMENT_DP_INIT();

    i = SEGMENT_DIGIT0_INIT();
    j = SEGMENT_DIGIT1_INIT();
    if (j) {
        i = j;
    }
    j = SEGMENT_DIGIT2_INIT();
    if (j) {
        i = j;
    }
    j = SEGMENT_DIGIT3_INIT();
    if (j) {
        i = j;
    }
    j = SEGMENT_DIGIT4_INIT();
    if (j) {
        i = j;
    }
    j = SEGMENT_DIGIT5_INIT();
    if (j) {
        i = j;
    }
    j = SEGMENT_DIGIT6_INIT();
    if (j) {
        i = j;
    }
    j = SEGMENT_DIGIT7_INIT();
    if (j) {
        i = j;
    }
    SEGMENT_DIGIT0_CLR();
    SEGMENT_DIGIT1_CLR();
    SEGMENT_DIGIT2_CLR();
    SEGMENT_DIGIT3_CLR();
    SEGMENT_DIGIT4_CLR();
    SEGMENT_DIGIT5_CLR();
    SEGMENT_DIGIT6_CLR();
    SEGMENT_DIGIT7_CLR();
    if (!i) {
        return -1;
    }
    dcb = (DCB_MUX7SEG *) dev->dev_dcb;
    dcb->num_digits = i;
    ticks = nr_digits2ticks[i - 1];
    memset(dcb->encoded_data, 0, SEG7_DIGITS);
    NutTimerStart(ticks, Mux7SegUpdate, dcb, 0);
    return 0;
}

/* Only one device supported, so static allocation will do! */
DCB_MUX7SEG dcbMux7Seg;

/*!
 * \brief 16seg device implementation structure.
 */
NUTDEVICE devMux7Seg = {
    0,                   /*!< \brief Pointer to next device. */
    {'m', 'u', 'x', '7', 's', 'e', 'g', 0},  /*!< \brief Unique device name. */
    IFTYP_CHAR,    /*!< \brief Type of device. */
    0,             /*!< \brief Base address. */
    0,             /*!< \brief First interrupt number. */
    NULL,          /*!< \brief Interface control block. */
    &dcbMux7Seg,   /*!< \brief Driver control block. */
    Mux7SegInit,   /*!< \brief Driver initialization routine. */
    Mux7SegIOCtl,  /*!< \brief Driver specific control function. */
    0,             /*!< \brief Read from device. */
    Mux7SegWrite,  /*!< \brief Write to device. */
#ifdef __HARVARD_ARCH__
    0,             /*!< \brief Write from program space data to device. */
#endif
    Mux7SegOpen,   /*!< \brief Open a device or file. */
    Mux7SegClose,  /*!< \brief Close a device or file. */
    0,             /*!< \brief Request file size. */
    0,             /*!< \brief Select function, optional, not yet implemented */
};
