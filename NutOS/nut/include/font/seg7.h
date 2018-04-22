/*
 * Copyright (C) 2009 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2009 by Rittal GmbH & Co. KG. All rights reserved.
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
 * \file font/seg7.h
 * \brief Font table for 7 segment display/
 */

/*****************************************************************************
 **
 **         7-Segement Driver: Character Table
 **
 **/

/* Display Interconnection
 *
 *      --A--
 *     |     |
 *     F     B      Bit:  7 6 5 4 3 2 1 0
 *     +--G--+      Led: dp A B C D E F G
 *     E     C
 *     |     |
 *      --D--  dp
 */
/* 7 Segment Display Character Table */
const uint8_t Seg7CharTab[] = {
    /* ' ' */(0x00),
    /* '!' */(0x28),
    /* '"' */(0x22),
    /* '#' */(0x00),
    /* '$' */(0x5B),
    /* '%' */(0x00),
    /* '&' */(0x6F),
    /* '´' */(0x20),
    /* '(' */(0x4E),
    /* ')' */(0x78),
    /* '*' */(0x00),
    /* '+' */(0x31),
    /* ''' */(0x20),
    /* '-' */(0x01),
    /* '.' */(0x01),
    /* '/' */(0x15),
    /* '0' */(0x7E),
    /* '1' */(0x30),
    /* '2' */(0x6D),
    /* '3' */(0x79),
    /* '4' */(0x33),
    /* '5' */(0x5B),
    /* '6' */(0x5F),
    /* '7' */(0x70),
    /* '8' */(0x7F),
    /* '9' */(0x7B),
    /* ':' */(0x00),
    /* ';' */(0x00),
    /* '<' */(0x00),
    /* '=' */(0x09),
    /* '>' */(0x00),
    /* '?' */(0x65),
    /* '@' */(0x00),
    /* 'A' */(0x77),
    /* 'b' */(0x1F),
    /* 'c' */(0x0D),
    /* 'd' */(0x3D),
    /* 'E' */(0x4F),
    /* 'F' */(0x47),
    /* 'G' */(0x5F),
    /* 'H' */(0x37),
    /* 'i' */(0x10),
    /* 'J' */(0x3C),
    /* 'K' */(0x0F),
    /* 'L' */(0x0E),
    /* 'M' */(0x76),
    /* 'N' */(0x15),
    /* 'O' */(0x1D),
    /* 'P' */(0x67),
    /* 'Q' */(0x73),
    /* 'R' */(0x05),
    /* 'S' */(0x5B),
    /* 'T' */(0x0F),
    /* 'U' */(0x3E),
    /* 'V' */(0x1C),
    /* 'W' */(0x3F),
    /* 'X' */(0x37),
    /* 'Y' */(0x3B),
    /* 'Z' */(0x6D),
    /* '[' */(0x4E),
    /* '\' */(0x13),
    /* ']' */(0x78),
    /* '^' */(0x42),
    /* '_' */(0x01),
};
