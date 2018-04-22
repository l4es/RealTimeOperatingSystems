/*
 * Copyright (C) 2013 by egnite GmbH
 * Copyright (c) 1989, 1993 The Regents of the University of California
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

#include <sys/types.h>
#include <limits.h>
#include <ctype.h>
#include <string.h>
#include <vis.h>

#define isoctal(c)  (((uint8_t)(c)) >= '0' && ((uint8_t)(c)) <= '7')
#define isvisible(c)    \
    (((uint32_t)(c) <= UCHAR_MAX && isascii((uint8_t)(c)) &&        \
    (((c) != '*' && (c) != '?' && (c) != '[' && (c) != '#') ||  \
        (flag & VIS_GLOB) == 0) && isgraph((uint8_t)(c))) ||     \
    ((flag & VIS_SP) == 0 && (c) == ' ') ||                     \
    ((flag & VIS_TAB) == 0 && (c) == '\t') ||                   \
    ((flag & VIS_NL) == 0 && (c) == '\n') ||                    \
    ((flag & VIS_SAFE) && ((c) == '\b' ||                       \
        (c) == '\007' || (c) == '\r' ||                         \
        isgraph((uint8_t)(c)))))

/*!
 * \brief Visually encode a single character.
 *
 * This function copies into dst a string which represents the character c.
 * If c needs no encoding, it is copied in unaltered. The string is null
 * terminated and the maximum length of any encoding is four characters.
 *
 * \param dst   Pointer to a buffer that receives the encoded string.
 *              At least 5 bytes of buffer space should be available.
 * \param c     The character to be encoded.
 * \param flag  Used for altering the default range of characters considered
 *              for encoding and for altering the visual representation.
 *              The following range flags are supported:
 *
 *              - VIS_GLOB  Also encodes '*', '?', '[' and '#'.
 *              - VIS_SP    Also encodes space.
 *              - VIS_TAB   Also encodes tab.
 *              - VIS_NL    Also encodes newline.
 *              - VIS_WHITE Same as VIS_SP | VIS_TAB | VIS_NL.
 *              - VIS_SAFE  Only encodes "unsafe" characters, which may
 *                          cause unexpected functions in terminal emulators.
 *
 *              The following flags specify the visual format:
 *
 *              - VIS_CSTYLE    Use C-style backslash sequences.
 *              - VIS_HTTPSTYLE Use URI encoding as described in RFC1808.
 *              - VIS_OCTAL     Use 3 digit octal sequences.
 *
 *              In addition, the flag VIS_NOSLASH can be used to inhibit
 *              the doubling of backslashes.
 * \param nextc Only used when selecting the VIS_CSTYLE encoding format.
 *
 * \return Pointer to the end of the destination string.
 */
char *vis(char *dst, int c, int flag, int nextc)
{
    static char *hex = "0123456789abcdef";

    if (flag & VIS_HTTPSTYLE) {
        /* Described in RFC 1808 */
        if (!(isalnum(c) /* alpha-numeric */
            /* safe */
            || c == '$' || c == '-' || c == '_' || c == '.' || c == '+'
            /* extra */
            || c == '!' || c == '*' || c == '\'' || c == '('
            || c == ')' || c == ',')) {
            *dst++ = '%';
            *dst++ = hex[((unsigned int) c >> 4) & 0xf];
            *dst++ = hex[(unsigned int) c & 0xf];
            goto done;
        }
    }
    if (isvisible(c)) {
        *dst++ = c;
        if (c == '\\' && (flag & VIS_NOSLASH) == 0)
            *dst++ = '\\';
        *dst = '\0';
        return dst;
    }

    if (flag & VIS_CSTYLE) {
        switch (c) {
        case '\n':
            *dst++ = '\\';
            *dst++ = 'n';
            goto done;
        case '\r':
            *dst++ = '\\';
            *dst++ = 'r';
            goto done;
        case '\b':
            *dst++ = '\\';
            *dst++ = 'b';
            goto done;
        case '\a':
            *dst++ = '\\';
            *dst++ = 'a';
            goto done;
        case '\v':
            *dst++ = '\\';
            *dst++ = 'v';
            goto done;
        case '\t':
            *dst++ = '\\';
            *dst++ = 't';
            goto done;
        case '\f':
            *dst++ = '\\';
            *dst++ = 'f';
            goto done;
        case ' ':
            *dst++ = '\\';
            *dst++ = 's';
            goto done;
        case '\0':
            *dst++ = '\\';
            *dst++ = '0';
            if (isoctal(nextc)) {
                *dst++ = '0';
                *dst++ = '0';
            }
            goto done;
        }
    }
    if (((c & 0177) == ' ') || (flag & VIS_OCTAL) ||
        ((flag & VIS_GLOB)
         && (c == '*' || c == '?' || c == '[' || c == '#'))) {
        *dst++ = '\\';
        *dst++ = ((uint8_t) c >> 6 & 07) + '0';
        *dst++ = ((uint8_t) c >> 3 & 07) + '0';
        *dst++ = ((uint8_t) c & 07) + '0';
        goto done;
    }
    if ((flag & VIS_NOSLASH) == 0)
        *dst++ = '\\';
    if (c & 0200) {
        c &= 0177;
        *dst++ = 'M';
    }
    if (iscntrl((uint8_t) c)) {
        *dst++ = '^';
        if (c == 0177)
            *dst++ = '?';
        else
            *dst++ = c + '@';
    } else {
        *dst++ = '-';
        *dst++ = c;
    }
  done:
    *dst = '\0';

    return dst;
}

/*!
 * \brief Visually encode a string.
 *
 * \param dst  Pointer to a buffer that receives the encoded string.
 *             The size of this buffer must be at least four times the
 *             number of characters encoded, plus one for the string
 *             terminator.
 * \param src  The string to be encoded.
 * \param flag Used for altering the range of characters and the visual
 *             representation, see vis().
 *
 * \return The length of the resulting string.
 */
int strvis(char *dst, const char *src, int flag)
{
    char c;
    char *start;

    for (start = dst; (c = *src);)
        dst = vis(dst, c, flag, *++src);
    *dst = '\0';
    return dst - start;
}

/*!
 * \brief Visually encode a string up to a maximum number of characters.
 *
 * \param dst  Pointer to a buffer that receives the encoded string.
 * \param src  Pointer to the buffer to be encoded.
 * \param siz  The size of the destination buffer.
 * \param flag Used for altering the range of characters and the visual
 *             representation, see vis().
 *
 * \return The length of the resulting string.
 */
int strnvis(char *dst, const char *src, size_t siz, int flag)
{
    char *start, *end;
    char tbuf[5];
    int c, i;

    i = 0;
    for (start = dst, end = start + siz - 1; (c = *src) && dst < end;) {
        if (isvisible(c)) {
            i = 1;
            *dst++ = c;
            if (c == '\\' && (flag & VIS_NOSLASH) == 0) {
                /* need space for the extra '\\' */
                if (dst < end)
                    *dst++ = '\\';
                else {
                    dst--;
                    i = 2;
                    break;
                }
            }
            src++;
        } else {
            i = vis(tbuf, c, flag, *++src) - tbuf;
            if (dst + i <= end) {
                memcpy(dst, tbuf, i);
                dst += i;
            } else {
                src--;
                break;
            }
        }
    }
    if (siz > 0)
        *dst = '\0';
    if (dst + i > end) {
        /* adjust return value for truncation */
        while ((c = *src))
            dst += vis(tbuf, c, flag, *++src) - tbuf;
    }
    return dst - start;
}

/*!
 * \brief Visually encode a buffer.
 *
 * This function is useful for encoding a block of data that may contain
 * NULs.
 *
 * \param dst  Pointer to a buffer that receives the encoded string.
 *             The size of this buffer must be at least four times the
 *             number of characters encoded, plus one for the string
 *             terminator.
 * \param src  The buffer to encode.
 * \param len  The number of characters to encode.
 * \param flag Used for altering the range of characters and the visual
 *             representation, see vis().
 *
 * \return The length of the resulting string.
 */
int strvisx(char *dst, const char *src, size_t len, int flag)
{
    char c;
    char *start;

    for (start = dst; len > 1; len--) {
        c = *src;
        dst = vis(dst, c, flag, *++src);
    }
    if (len)
        dst = vis(dst, *src, flag, '\0');
    *dst = '\0';
    return dst - start;
}
