#ifndef _PRO_UHTTP_STREAMIO_H_
#define _PRO_UHTTP_STREAMIO_H_

/*
 * Copyright (C) 2012 by egnite GmbH
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
 * $Id$
 */

#include <cfg/http.h>
#include <pro/uhttp/compiler.h>

#if defined WIN32
#include <pro/uhttp/os/win/streamio.h>
#elif defined __linux__
#include <unistd.h>
#elif defined(HTTP_PLATFORM_STREAMS)
#include <pro/uhttp/os/nut/streamio.h>
#endif

/*!
 * \addtogroup xgUHTTPStreamIo Stream I/O
 * \ingroup xgUHTTP
 */
/*@{*/

/* \brief Platform dependent stream information structure type. */
#ifdef HTTP_PLATFORM_STREAMS
typedef struct _HTTP_STREAM HTTP_STREAM;
#else
#include <stdio.h>
typedef FILE HTTP_STREAM;
#endif

/* \brief Client handler type. */
typedef void (*HTTP_CLIENT_HANDLER) (HTTP_STREAM *);


/*!
 * \brief Initialize the stream.
 *
 * This function must be called before before calling StreamClientAccept()
 * and will initialize the platform dependent stream processing.
 *
 * \return 0 on success or -1 on error.
 */
extern int StreamInit(void);

/*!
 * \brief Accept stream clients.
 *
 * If no error occurs, this function will never return.
 *
 * \param handler Client handler to be called when a client successfully
 *                connected.
 * \param params  Stream specific parameters. For TCP/IP connections this
 *                is currently used to specify the port number. If NULL,
 *                then the server will listen at default port 80.
 *
 * \return -1 on error.
 */
extern int StreamClientAccept(HTTP_CLIENT_HANDLER handler, const char *params);

/*!
 * \brief Read data from a stream until any of the specified characters appears.
 *
 * \param sp     Pointer to the stream's information structure.
 * \param delim  The function will return as soon as any of the characters
 *               in this string is read.
 * \param ignore String of all characters that will be skipped.
 * \param buf    Pointer to the buffer that will receive the data read
 *               from the stream. Characters to ignore are not stored.
 *               The delimiter will be replaced by a string terminator
 *               (ASCII 0).
 * \param siz    Size of the buffer, given in bytes.
 *
 * \return The number of bytes consumed from the stream, including any
 *         skipped characters and the delimiter. A return value of -1
 *         indicates an error.
 */
extern int StreamReadUntilChars(HTTP_STREAM *sp, const char *delim, const char *ignore, char *buf, int siz);

/*!
 * \brief Read data from a stream until a specified string appears.
 *
 * \param sp Pointer to the stream's information structure.
 * \param delim  The function will return as soon as this string
 *               appears.
 * \param buf    Pointer to the buffer that will receive the data read
 *               from the stream. Characters to ignore are not stored.
 *               The searched string will be replaced by a string
 *               terminator (ASCII 0).
 * \param siz    Size of the buffer, given in bytes.
 *
 * \return The number of bytes consumed from the stream. A return value
 *         of -1 indicates an error.
 */
extern int StreamReadUntilString(HTTP_STREAM *sp, const char *delim, char *buf, int siz);

/*!
 * \brief Write a variable number of strings to a stream.
 *
 * \param sp Pointer to the stream's information structure.
 *
 * \return A non-negative value if successful or EOF to indicate an error.
 */
extern int s_vputs(HTTP_STREAM *sp, ...);

#ifdef HTTP_PLATFORM_STREAMS

/*!
 * \brief Write data items to a stream.
 *
 * This function is similar to the standard function fwrite().
 *
 * \param buf   Pointer to items to be written.
 * \param size  Item size in bytes.
 * \param count Number of items to write.
 * \param sp    Pointer to the stream's information structure.
 *
 * \return The number of items written, which may be less than the
 *         specified number.
 */
extern int s_write(const void *buf, size_t size, size_t count, HTTP_STREAM *sp);

/*!
 * \brief Write a string to a stream.
 *
 * This function is similar to the standard function fputs().
 *
 * \param sp  Pointer to the stream's information structure.
 * \param str String to write.
 *
 * \return A non-negative value if successful or EOF to indicate an error.
 */
extern int s_puts(const char *str, HTTP_STREAM *sp);

/*!
 * \brief Print formatted data to a stream.
 *
 * This function is similar to the standard function fprintf().
 *
 * \param sp  Pointer to the stream's information structure.
 * \param fmt Format string containing conversion specifications.
 *
 * \return The number of characters written or a negative value to
 *         indicate an error.
 */
extern int s_printf(HTTP_STREAM *sp, const char *fmt, ...);

/*!
 * \brief Flush a stream.
 *
 * This function is similar to the standard function fflush().
 *
 * The calling thread may be suspended until all buffered output data
 * has been written.
 *
 * \param sp Pointer to the stream's information structure.
 *
 * \return 0 if the buffer was successfully flushed, EOF if an error
 *         occured.
 */
extern int s_flush(HTTP_STREAM *sp);

#else
#define s_write     fwrite
#define s_puts      fputs
#define s_printf    fprintf
#define s_flush     fflush
#endif

/*! \name Stream flags */
/*@{*/
/*! \brief Enable chunked transfer. */
#define S_FLG_CHUNKED       1
/*@}*/

/*!
 * \brief Set stream flags.
 *
 * \param sp    Pointer to the stream's information structure.
 * \param flags The flags to set. Currently only \ref S_FLG_CHUNKED
 *              is supported.
 *
 * \return 0 if the flag has been successfully set or a negative value
 *         to indicate an error.
 */
extern int s_set_flags(HTTP_STREAM *sp, unsigned int flags);

/*!
 * \brief Reset stream flags.
 *
 * \param sp    Pointer to the stream's information structure.
 * \param flags The flags to clear. Currently only \ref S_FLG_CHUNKED
 *              is supported.
 *
 * \return 0 if the flag has been successfully set or a negative value
 *         to indicate an error.
 */
extern int s_clr_flags(HTTP_STREAM *sp, unsigned int flags);

/*! \name Default environment variables */
/*@{*/
/*! \brief The IP address of the host making this request. */
#define SITEM_REMOTE_ADDR  1
/*! \brief The port number used by the remote host when making this request. */
#define SITEM_REMOTE_PORT  2
/*! \brief The IP address of the server for this URL. */
#define SITEM_SERVER_ADDR  3
/*! \brief The servers host name, DNS alias or IP address. */
#define SITEM_SERVER_NAME  4
/*! \brief The port number on this server to which this request was directed. */
#define SITEM_SERVER_PORT  5
/*@}*/

extern const char *StreamInfo(HTTP_STREAM *hs, int item);

/*@}*/
#endif
