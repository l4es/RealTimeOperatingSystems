#ifndef _PRO_UHTTP_UHTTP_H_
#define _PRO_UHTTP_UHTTP_H_

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

/*!
 * \file pro/uhttp/uhttpd.h
 * \brief Micro HTTP library.
 *
 * \verbatim File version $Id$ \endverbatim
 */

#include <cfg/http.h>

#include <pro/uhttp/compiler.h>
#include <pro/uhttp/streamio.h>

#if !defined(HTTPD_EXCLUDE_DATE)
#include <time.h>
#endif

/*!
 * \addtogroup xgUHTTP uHTTP
 * \ingroup xgUserPro
 * \brief Micro hypertext transfer protocol library.
 *
 * This is the second version of a HTTP library for Nut/OS. It has been
 * introduced in Nut/OS 5.0 and may later replace the original
 * \ref xgHTTPD "HTTP library".
 *
 * The main advantage of this new version is its flexibility, which not
 * only allows to replace almost any default behavior, but also offers
 * various ways to extend its capabilities without modification of the
 * original source code. To give an example: The list of hard coded media
 * types can be configured with the Nut/OS Configurator. Furthermore, new
 * media types may be either added to this list to be processed by a
 * default handler or may be processed by an application specific handler.
 * In the old library new media types had to be added to the hard coded
 * default list, modifying the original source code.
 *
 * Operating system (Nut/OS) dependent functions are no longer
 * scattered in the source code. Instead, they are collected in a single
 * module, which makes porting the library to other operating systems
 * quite easy. So far the code had been successfully tested on Windows and
 * a Linux port will probably follow some day.
 */
/*@{*/

#if 0
/* Testing versions. */
#define HTTP_VERSION    0x09
#undef HTTP_MAJOR_VERSION
#define HTTP_MAJOR_VERSION  0
#undef HTTP_MINOR_VERSION
#define HTTP_MINOR_VERSION  9
#endif

#ifndef HTTP_MAJOR_VERSION
#define HTTP_MAJOR_VERSION  1
#endif

#ifndef HTTP_MINOR_VERSION
#define HTTP_MINOR_VERSION  1
#endif

#ifndef HTTP_VERSION
#if HTTP_MAJOR_VERSION == 0
#if HTTP_MINOR_VERSION == 9
#define HTTP_VERSION    0x09
#endif
#elif HTTP_MAJOR_VERSION == 1
#if HTTP_MINOR_VERSION == 0
#define HTTP_VERSION    0x10
#elif HTTP_MINOR_VERSION == 1
#define HTTP_VERSION    0x11
#endif
#endif
#endif

#ifndef HTTP_VERSION
#error Cannot determine HTTP version
#endif

/*! \name HTTP request methods */
/*@{*/
/*! \brief Get method. */
#define HTTP_METHOD_GET         1
/*! \brief Post method. */
#define HTTP_METHOD_POST        2
/*! \brief Head method. */
#define HTTP_METHOD_HEAD        3
/*@}*/

/*! \name HTTP connection types */
/*@{*/
/*! \brief Connection will be closed. */
#define HTTP_CONN_CLOSE         1
/*! \brief Connection will not be closed. */
#define HTTP_CONN_KEEP_ALIVE    2
/*@}*/

/*! \brief HTTP request information structure type. */
typedef struct _HTTP_REQUEST HTTP_REQUEST;

/*! \brief HTTP request information structure. */
struct _HTTP_REQUEST {
    int req_method;             /*!< \brief Request method */
    char *req_url;              /*!< \brief URI portion of the GET or POST request line */
    int req_version;            /*!< \brief BCD coded HTTP version, either 0x09, 0x10 or 0x11 */
    char *req_query;            /*!< \brief Argument string */
    char *req_argp;             /*!< \brief Current argument pointer */
    char *req_argn;             /*!< \brief Escaped argument name */
    char *req_argv;             /*!< \brief Escaped argument value */
    int req_connection;         /*!< \brief Connection type, HTTP_CONN_ */
#if HTTP_VERSION >= 0x10
    long req_length;            /*!< \brief Content length */
    char *req_realm;            /*!< \brief Realm of the requested URI */
    char *req_type;             /*!< \brief Content type */
    char *req_cookie;           /*!< \brief Cookie */
    char *req_auth;             /*!< \brief Authorization info */
    char *req_agent;            /*!< \brief User agent */
#if !defined(HTTPD_EXCLUDE_DATE)
    time_t req_ims;             /*!< \brief If-modified-since condition */
#endif
    char *req_referer;          /*!< \brief Misspelled HTTP referrer */
    char *req_host;             /*!< \brief Server host */
    char *req_encoding;         /*!< \brief Accept encoding */
    char *req_bnd_dispo;        /*!< \brief Content disposition */
    char *req_bnd_type;         /*!< \brief Boundary type */
#endif
};

/*! \brief HTTP session information structure type. */
typedef struct _HTTPD_SESSION HTTPD_SESSION;

/*! \brief HTTP session information structure. */
struct _HTTPD_SESSION {
    HTTP_STREAM *s_stream;
    HTTP_REQUEST s_req;
};

/*! \name HTTP static texts */
/*@{*/
extern const char ct_GET[];
extern const char ct_HEAD[];
extern const char ct_POST[];
extern const char ct_Content_Disposition[];
extern const char ct_Content_Type[];
extern const char ct_Accept_Encoding[];
extern const char ct_Authorization[];
extern const char ct_Connection[];
extern const char ct_close[];
extern const char ct_Keep_Alive[];
extern const char ct_Content_Length[];
extern const char ct_Cookie[];
extern const char ct_Host[];
extern const char ct_Referer[];
extern const char ct_User_Agent[];
extern const char ct_Content_Encoding[];
extern const char ct_Location[];

#if !defined(HTTPD_EXCLUDE_DATE)
extern const char ct_If_Modified_Since[];
extern const char ct_Last_Modified[];
extern const char ct_Expires[];
extern const char ct_Date[];
#endif
/*@}*/

extern char *http_root_path;
/*! \brief Default file system. */
#ifndef HTTP_DEFAULT_ROOT
#define HTTP_DEFAULT_ROOT   "UROM:"
#endif
#define HTTP_ROOT   (http_root_path ? http_root_path : HTTP_DEFAULT_ROOT)

/*! \brief HTTP authentication function type. */
typedef int (*HTTP_AUTH_VALIDATOR) (HTTPD_SESSION*);
/*! \brief HTTP authentication function pointer. */
extern HTTP_AUTH_VALIDATOR httpd_auth_validator;

/*! \brief HTTP redirection function type. */
typedef int (*HTTP_LOC_REDIRECTOR) (HTTPD_SESSION*);
/*! \brief HTTP redirection function pointer. */
extern HTTP_LOC_REDIRECTOR httpd_loc_redirector;


/*!
 * \brief Register the HTTP server's root directory.
 *
 * Only one root directory is supported. Subsequent calls will override
 * previous settings.
 *
 * \param path Absolute path name of the root directory. If NULL, then
 *             the path will be reset to the configured default.
 *
 * \return 0 on success, -1 otherwise.
 */
extern int HttpRegisterRootPath(char *path);

/*!
 * \brief Parse HTTP header.
 *
 * \param hs Pointer to the session info structure.
 */
extern int HttpParseHeader(HTTPD_SESSION *hs);

/*!
 * \brief Parse HTTP multipart header.
 *
 * \param hs Pointer to the session info structure.
 */
extern int HttpParseMultipartHeader(HTTPD_SESSION *hs, const char *bnd, long *avail);

/*!
 * \brief Send initial HTTP response header.
 *
 * \param hs Pointer to the session info structure.
 */
extern void HttpSendHeaderTop(HTTPD_SESSION *hs, int status);

extern void HttpSendStreamHeaderTop(HTTP_STREAM *stream, int status);

#if !defined(HTTPD_EXCLUDE_DATE)

/*!
 * \brief Send HTTP date response header.
 *
 * \param hs Pointer to the session info structure.
 */
extern void HttpSendHeaderDate(HTTPD_SESSION *hs, time_t mtime);

extern void HttpSendStreamHeaderDate(HTTP_STREAM *stream, time_t mtime);

#endif

/*!
 * \brief Send final HTTP response header.
 *
 * \param hs Pointer to the session info structure.
 */
extern void HttpSendHeaderBottom(HTTPD_SESSION *hs, const char *type, const char *subtype, long bytes);

extern void HttpSendStreamHeaderBottom(HTTP_STREAM *stream, const char *type, const char *subtype, int conn, long bytes);

/*!
 * \brief Send HTTP error response.
 *
 * \param hs Pointer to the session info structure.
 */
extern void HttpSendError(HTTPD_SESSION *hs, int status);

extern void HttpSendStreamError(HTTP_STREAM *stream, int status, const char *realm);

/*!
 * \brief Send HTTP redirection response.
 *
 * \param hs Pointer to the session info structure.
 */
extern int HttpSendRedirection(HTTPD_SESSION *hs, int code, ...);

/*!
 * \brief Get first argument from an HTTP request line.
 *
 * The function returns the name only. Use HttpArgValue() to retrieve
 * its value.
 *
 * \param req Pointer to the request information structure.
 *
 */
extern char *HttpArgParseFirst(HTTP_REQUEST * req);

/*!
 * \brief Get next argument from an HTTP request line.
 *
 * \param req Pointer to the request information structure.
 */
extern char *HttpArgParseNext(HTTP_REQUEST * req);

/*!
 * \brief Read next argument from an HTTP post request.
 *
 * The function returns a pointer to the name. Use HttpArgValue() to
 * retrieve its value.
 *
 * \param hs    Pointer to the session info structure.
 * \param avail Pointer to the variable that contains the number of bytes
 *              available in the request body. It will be updated by this
 *              function.
 *
 * \return Pointer to a string that contains the name of the argument.
 *         NULL is returned, if all arguments had been read or in case
 *         of an error.
 */
extern char *HttpArgReadNext(HTTPD_SESSION *hs, long *avail);

/*!
 * \brief Get value of the last read argument.
 *
 * Returns the value of an argument previously read by HttpArgParseFirst(),
 * HttpArgParseNext() or HttpArgReadNext().
 *
 * \param req Pointer to the request information structure.
 *
 * \return Pointer to a string that contains the name of the argument.
 *         NULL is returned, if no value is available or in case of an
 *         error.
 */
extern char *HttpArgValue(HTTP_REQUEST * req);

/*!
 * \brief Retrieve parameter value from HTTP header line.
 *
 * A HTTP header line may contain a list of parameter name/value pairs,
 * separated by semicolons.
 *
 * <header-name>: <param-name>="<param-value>"; <param-name>="<param-value>"..
 *
 * This function can be used to retrieve a value of a parameter that is
 * specified by its name. Surrounding quotation marks are excluded from
 * the result.
 *
 * \param str  Pointer to the text buffer that contains the header line.
 * \param name Name of the parameter to retrieve.
 * \param len  Pointer to a variable that receives the length of the value.
 *
 * \return Pointer to the value.
 */
extern const char *HttpArgValueSub(const char *str, const char *name, int *len);

/*!
 * \brief Default client handler.
 *
 * \param sp Pointer to the stream's information structure.
 */
void HttpdClientHandler(HTTP_STREAM *sp);

/*@}*/

/*!
 * \defgroup xgUHTTPModules Modules
 * \ingroup xgUHTTP
 */
/*@{*/
/*@}*/

#endif
