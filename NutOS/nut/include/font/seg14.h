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

/*!
 * \file include/dev/seg14.h
 * \brief Translation table for 14-segment display
 */

/*******************************************************************************
**
 **
 **         14-Segement Driver: Character Table
 **
 **
 *
 * Display Interconnection
 *
 *      ---A---
 *     | H J K |
 *     F  \|/  B      Bit:  13 12 11 10 9 8 7 6 5 4 3 2 1 0
 *     +-G-- M-+      Led:   Q  P  N  M K J H G F E D C B A
 *     E  /|\  C
 *     | Q P N |
 *      ---D---
 *
 * 14 Segment Display Character Table
 * Implement font from MAX6954
 *
 * Char 0 ... 15 for single segments.
 */
const uint16_t chars2seg14[] = {
    0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080,
    0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x2000, 0x4000, 0x8000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x1549, 0x16c0, 0x1539, 0x0463, 0x02b0, 0x1548, 0x3900, 0x1380,
/*          !       "       #       $       %       &       '       */
    0x0000, 0x0006, 0x0120, 0x3fff, 0x156d, 0x2ee4, 0x2a8e, 0x0200,
/*  (       )       *       +       ,       -       .       /       */
    0x0a00, 0x2080, 0x3fc0, 0x1540, 0x2000, 0x0440, 0x1058, 0x2200,
/*  0       1       2       3       4       5       6       7       */
    0x223f, 0x0006, 0x045b, 0x044f, 0x0466, 0x0869, 0x047d, 0x0007,
/*  8       9       :       ;       <       =       >       ?       */
    0x047f, 0x046f, 0x1100, 0x2100, 0x0a00, 0x0448, 0x2080, 0x1403,
/*  @       A       B       C       D       E       F       G       */
    0x043b, 0x0477, 0x150f, 0x0039, 0x110f, 0x0479, 0x0071, 0x043d,
/*  H       I       J       K       L       M       N       O       */
    0x0476, 0x1100, 0x001e, 0x0a70, 0x0038, 0x02b6, 0x08b6, 0x003f,
/*  P       Q       R       S       T       U       V       W       */
    0x0473, 0x083f, 0x0c73, 0x046d, 0x1101, 0x003e, 0x2230, 0x2836,
/*  X       Y       Z       [       \       ]       ^       _       */
    0x2a80, 0x1280, 0x2209, 0x0039, 0x2100, 0x000f, 0x2203, 0x0008,
/*  `       a       b       c       d       e       f       g       */
    0x0080, 0x0477, 0x150f, 0x0039, 0x110f, 0x0479, 0x0071, 0x043d,
/*  h       i       j       k       l       m       n       o       */
    0x0476, 0x1100, 0x001e, 0x0a70, 0x0038, 0x02b6, 0x08b6, 0x003f,
/*  p       q       r       s       t       u       v       w       */
    0x0473, 0x083f, 0x0c73, 0x046d, 0x1101, 0x003e, 0x2230, 0x2836,
/*  x       y       z       {       |       }       ~               */
    0x2a80, 0x1280, 0x2209, 0x0a40, 0x1100, 0x2480, 0x0452, 0x003f
};

