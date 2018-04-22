/*
 * Copyright (C) 2015 Uwe Bonnes, bon@elektron.ikp.physik.tu-darmstadt.de.
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

#if !defined(__LCD_24X4_H)
# define __LCD_24X4_H
/*!
 * \file include/dev/lcd_24_4.h
 * \brief Segment mapping on LCD 24x4 glass like GH08172T used on STM32L1|4_disco
 */

static const uint16_t seg2offset[] = {
    /* These are the numbers from  UM1079 Rev. 3, Table 7 */
    /* Digit 1*/
    COM1 + LCDSEG22, COM0 + LCDSEG22, COM1 + LCDSEG01, COM1 + LCDSEG00, /* ABCD */
    COM0 + LCDSEG00, COM1 + LCDSEG23, COM0 + LCDSEG23, COM3 + LCDSEG23, /* EFGH */
    COM3 + LCDSEG22, COM2 + LCDSEG22, COM0 + LCDSEG01, COM3 + LCDSEG00, /* JKMN */
    COM2 + LCDSEG00, COM2 + LCDSEG23, COM3 + LCDSEG01, COM2 + LCDSEG01, /* PQ,: */
    /* Digit 2*/
    COM1 + LCDSEG20, COM0 + LCDSEG20, COM1 + LCDSEG03, COM1 + LCDSEG02, /* ABCD */
    COM0 + LCDSEG02, COM1 + LCDSEG21, COM0 + LCDSEG21, COM3 + LCDSEG21, /* EFGH */
    COM3 + LCDSEG20, COM2 + LCDSEG20, COM0 + LCDSEG03, COM3 + LCDSEG02, /* JKMN */
    COM2 + LCDSEG02, COM2 + LCDSEG21, COM3 + LCDSEG03, COM2 + LCDSEG03, /* PQ,: */
    /* Digit 3*/
    COM1 + LCDSEG18, COM0 + LCDSEG18, COM1 + LCDSEG05, COM1 + LCDSEG04, /* ABCD */
    COM0 + LCDSEG04, COM1 + LCDSEG19, COM0 + LCDSEG19, COM3 + LCDSEG19, /* EFGH */
    COM3 + LCDSEG18, COM2 + LCDSEG18, COM0 + LCDSEG05, COM3 + LCDSEG04, /* JKMN */
    COM2 + LCDSEG04, COM2 + LCDSEG19, COM3 + LCDSEG05, COM2 + LCDSEG05, /* PQ,: */
    /* Digit 4*/
    COM1 + LCDSEG16, COM0 + LCDSEG16, COM1 + LCDSEG07, COM1 + LCDSEG06, /* ABCD */
    COM0 + LCDSEG06, COM1 + LCDSEG17, COM0 + LCDSEG17, COM3 + LCDSEG17, /* EFGH */
    COM3 + LCDSEG16, COM2 + LCDSEG16, COM0 + LCDSEG07, COM3 + LCDSEG06, /* JKMN */
    COM2 + LCDSEG06, COM2 + LCDSEG17, COM3 + LCDSEG07, COM2 + LCDSEG07, /* PQ,: */
    /* Digit 5, '.' -> BAR2, ':' -> BAR1*/
    COM1 + LCDSEG14, COM0 + LCDSEG14, COM1 + LCDSEG09, COM1 + LCDSEG08, /* ABCD */
    COM0 + LCDSEG08, COM1 + LCDSEG15, COM0 + LCDSEG15, COM3 + LCDSEG15, /* EFGH */
    COM3 + LCDSEG14, COM2 + LCDSEG14, COM0 + LCDSEG09, COM3 + LCDSEG08, /* JKMN */
    COM2 + LCDSEG08, COM2 + LCDSEG15, COM3 + LCDSEG09, COM2 + LCDSEG09, /* PQ(BAR2)(BAR3) */
    /* Digit 6, '.' -> BAR0, ':' -> BAR3*/
    COM1 + LCDSEG12, COM0 + LCDSEG12, COM1 + LCDSEG11, COM1 + LCDSEG10, /* ABCD */
    COM0 + LCDSEG10, COM1 + LCDSEG13, COM0 + LCDSEG13, COM3 + LCDSEG13, /* EFGH */
    COM3 + LCDSEG12, COM2 + LCDSEG12, COM0 + LCDSEG11, COM3 + LCDSEG10, /* JKMN */
    COM2 + LCDSEG10, COM2 + LCDSEG13, COM3 + LCDSEG11, COM2 + LCDSEG11  /* PQ,(BAR0)(BAR1) */
};

/* 1/4 Duty */
 #define LCD_DUTY  (3 * LCD_CR_DUTY_0)
/* 1/3 Bias */
 #define LCD_BIAS  (2 * LCD_CR_BIAS_0)
/* Number of used or connected digits */
 #if !defined(SEG16_DIGITS)
  #define SEG16_DIGITS 6
 #endif
/* Number of digits with extra decimal point */
 #define DP_DIGITS    4
/* Number of digits with extra colon */
 #define COLON_DIGITS 4

#endif
