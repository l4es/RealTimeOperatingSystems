#ifndef _DEV_UART_H
#define _DEV_UART_H

/*
 * Copyright (C) 2001-2004 by egnite Software GmbH
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
 * $Id: uart.h 6453 2016-05-24 17:43:03Z olereinhardt $
 */

#include <sys/device.h>

/*!
 * \file dev/uart.h
 * \brief UART I/O control functions.
 * \verbatim
 * $Id: uart.h 6453 2016-05-24 17:43:03Z olereinhardt $
 * \endverbatim
 */

/*!
 * \addtogroup xgUARTIOCTL
 *
 * \brief UART _ioctl() commands.
 *
 * These commands are used to control and retrieve hardware specific
 * configurations. The definitions are kept independent from the
 * underlying hardware, but not all commands may be fully implemented
 * in each UART driver.
 *
 * The \ref _ioctl() function expects three parameters:
 * - A device descriptor.
 * - A command code, any of the UART_... commands listed below.
 * - A pointer to a configuration parameter, in most cases an uint32_t.
 */
/*@{*/

/*! \brief UART _ioctl() command code to set the line speed.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which contains the bit rate per second (baud rate). The command sets
 * both, input and output speed.
 *
 * Which speeds are available depends on the target platform and the
 * current peripheral clock frequency. If available, the initial speed
 * will be 115200, but may be re-configured via UART_INIT_BAUDRATE.
 *
 * See also \ref UART_GETSPEED.
 */
#define UART_SETSPEED           0x0101

/*! \brief UART _ioctl() command code to query the line speed.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which receives the current bit rate per second.
 *
 * See also \ref UART_SETSPEED.
 */
#define UART_GETSPEED           0x0102

/*! \brief UART _ioctl() command code to set the number of data bits.
 *
 * The configuration parameter must be a pointer to a uint8_t variable,
 * which contains the number of data bits.
 *
 * Typically the number of data bits is initially set to 8. Most
 * platforms aloow to change this to 7 bits, some may support 5, 6 or
 * 9 bits.
 *
 * See also \ref UART_GETDATABITS.
 */
#define UART_SETDATABITS        0x0103

/*! \brief UART _ioctl() command code to query the number of data bits.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which receives the current number of data bits.
 *
 * See also \ref UART_SETDATABITS.
 */
#define UART_GETDATABITS        0x0104

/*! \brief UART _ioctl() command code to set the parity mode.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which contains the requested parity mode, 0 (none), 1 (odd) or
 * 2 (even).
 *
 * On most platforms the initial parity mode is set to 0.
 *
 * See also \ref UART_GETPARITY.
 */
#define UART_SETPARITY          0x0105

/*! \brief UART _ioctl() command code to query the parity mode.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which receives the current parity mode, 0 (none), 1 (odd) or 2 (even).
 *
 * See also \ref UART_SETPARITY.
 */
#define UART_GETPARITY          0x0106

/*! \brief UART _ioctl() command code to set the number of stop bits.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which contains the number of stop bits.
 *
 * Most platforms support 1 or 2 stop bits, initially set to 1.
 *
 * See also \ref UART_GETSTOPBITS.
 */
#define UART_SETSTOPBITS        0x0107

/*! \brief UART _ioctl() command code to query the number of stop bits.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which receives the current number of stop bits.
 *
 * See also \ref UART_SETSTOPBITS.
 */
#define UART_GETSTOPBITS        0x0108

/*! \brief UART _ioctl() command code to set the status.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which contains the \ref xaUARTStatus "UART status" to be set.
 *
 * See also \ref UART_GETSTATUS.
 */
#define UART_SETSTATUS          0x0109

/*! \brief UART _ioctl() command code to query the status.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which receives the current \ref xaUARTStatus "UART status".
 *
 * See also \ref UART_SETSTATUS.
 */
#define UART_GETSTATUS          0x010a

/*! \brief UART _ioctl() command code to set the read timeout.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which contains the read timeout in milliseconds.
 *
 * See also \ref UART_GETREADTIMEOUT.
 */
#define UART_SETREADTIMEOUT     0x010b

/*! \brief UART _ioctl() command code to query the read timeout.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which receives the current read timeout in milliseconds.
 *
 * See also \ref UART_SETREADTIMEOUT.
 */
#define UART_GETREADTIMEOUT     0x010c

/*! \brief UART _ioctl() command code to set the write timeout.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which contains the write timeout in milliseconds.
 *
 * See also \ref UART_GETWRITETIMEOUT.
 */
#define UART_SETWRITETIMEOUT    0x010d

/*! \brief UART _ioctl() command code to query the write timeout.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which receives the current write timeout in milliseconds.
 *
 * See also \ref UART_SETWRITETIMEOUT.
 */
#define UART_GETWRITETIMEOUT    0x010e

/*! \brief UART _ioctl() command code to set the local echo mode.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which contains the requested echo mode, either 0 (off) or 1 (on).
 *
 * See also \ref UART_GETLOCALECHO.
 */
#define UART_SETLOCALECHO       0x010f

/*! \brief UART _ioctl() command code to query the local echo mode.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which receives the current echo mode, either 0 (off) or 1 (on).
 *
 * See also \ref UART_SETLOCALECHO.
 */
#define UART_GETLOCALECHO       0x0110

/*! \brief UART _ioctl() command code to set the flow control mode.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which contains the requested \ref xaUARTHS "flow control mode".
 *
 * See also \ref UART_GETFLOWCONTROL.
 */
#define UART_SETFLOWCONTROL     0x0111

/*! \brief UART _ioctl() command code to query the flow control mode.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which receives the current \ref xaUARTHS "flow control mode".
 *
 * See also \ref UART_SETFLOWCONTROL.
 */
#define UART_GETFLOWCONTROL     0x0112

/*! \brief UART _ioctl() command code to set the cooking mode.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which contains the requested cooking mode, either 0 (raw) or 1 (EOL
 * translation).
 *
 * See also \ref UART_GETCOOKEDMODE.
 */
#define UART_SETCOOKEDMODE      0x0113

/*! \brief UART _ioctl() command code to query the cooking mode.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which receives the current cooking mode, either 0 (raw) or 1 (EOL
 * translation).
 *
 * See also \ref UART_SETCOOKEDMODE.
 */
#define UART_GETCOOKEDMODE      0x0114

/*! \brief UART _ioctl() command code to set the buffering mode.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which contains the requested buffering mode.
 *
 * See also \ref UART_GETBUFFERMODE.
 */
#define UART_SETBUFFERMODE      0x0115

/*! \brief UART _ioctl() command code to query the buffering mode.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which receives the current buffering mode.
 *
 * See also \ref UART_SETBUFFERMODE.
 */
#define UART_GETBUFFERMODE      0x0116

/*! \brief UART _ioctl() command code to set the network interface mode.
 *
 * The configuration parameter must be a pointer to the \ref NUTDEVICE
 * structure of the network device to enable HDLC mode, or a NULL pointer
 * to disable it.
 *
 * See also \ref HDLC_GETIFNET.
 */
#define HDLC_SETIFNET           0x0117

/*! \brief UART _ioctl() command code to query the network interface mode.
 *
 * The configuration parameter must be a pointer to a pointer variable,
 * which receives a \ref NUTDEVICE pointer to the current network device
 * if HDLC is enabled, or NULL if disabled.
 *
 * See also \ref HDLC_SETIFNET.
 */
#define HDLC_GETIFNET           0x0118

/*! \brief UART _ioctl() command code to set the clock mode.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which contains the requested \ref xaUARTClock "clock mode".
 *
 * See also \ref UART_GETCLOCKMODE.
 */
#define UART_SETCLOCKMODE       0x0119

/*! \brief UART _ioctl() command code to query the clock mode.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which receives the current \ref xaUARTClock "clock mode".
 *
 * See also \ref UART_SETCLOCKMODE.
 */
#define UART_GETCLOCKMODE       0x011a

/*! \brief UART _ioctl() command code to set the transmit buffer size.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which contains the requested size of the transmit buffer in bytes.
 *
 * See also \ref UART_GETTXBUFSIZ.
 */
#define UART_SETTXBUFSIZ        0x011b

/*! \brief UART _ioctl() command code to query the transmit buffer size.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which receives the current size of the transmit buffer in bytes.
 *
 * See also \ref UART_SETTXBUFSIZ.
 */
#define UART_GETTXBUFSIZ        0x011c

/*! \brief UART _ioctl() command code to set the receive buffer size.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which contains the requested size of the receive buffer in bytes.
 *
 * See also \ref UART_GETRXBUFSIZ.
 */
#define UART_SETRXBUFSIZ        0x011d

/*! \brief UART _ioctl() command code to query the receive buffer size.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which receives the current size of the receive buffer in bytes.
 *
 * See also \ref UART_SETRXBUFSIZ.
 */
#define UART_GETRXBUFSIZ        0x011e

/*! \brief UART _ioctl() command code to set the transmit buffer low watermark.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which contains the requested low watermark of the transmit buffer.
 *
 * See also \ref UART_GETTXBUFLWMARK.
 */
#define UART_SETTXBUFLWMARK     0x0120

/*! \brief UART _ioctl() command code to query the transmit buffer low watermark.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which receives the current low watermark of the transmit buffer.
 *
 * See also \ref UART_SETTXBUFLWMARK.
 */
#define UART_GETTXBUFLWMARK     0x0121

/*! \brief UART _ioctl() command code to set the transmit buffer high watermark.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which contains the requested high watermark of the transmit buffer.
 *
 * See also \ref UART_GETTXBUFHWMARK.
 */
#define UART_SETTXBUFHWMARK     0x0122

/*! \brief UART _ioctl() command code to query the transmit buffer high watermark.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which receives the current high watermark of the transmit buffer.
 *
 * See also \ref UART_SETTXBUFHWMARK.
 */
#define UART_GETTXBUFHWMARK     0x0123

/*! \brief UART _ioctl() command code to set the receive buffer low watermark.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which contains the requested low watermark of the receive buffer.
 *
 * See also \ref UART_GETRXBUFLWMARK.
 */
#define UART_SETRXBUFLWMARK     0x0124

/*! \brief UART _ioctl() command code to query the receive buffer low watermark.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which receives the current low watermark of the receive buffer.
 *
 * See also \ref UART_SETRXBUFLWMARK.
 */
#define UART_GETRXBUFLWMARK     0x0125

/*! \brief UART _ioctl() command code to set the receive buffer high watermark.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which contains the requested high watermark of the receive buffer.
 *
 * See also \ref UART_GETRXBUFHWMARK.
 */
#define UART_SETRXBUFHWMARK     0x0126

/*! \brief UART _ioctl() command code to query the receive buffer high watermark.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which receives the current high watermark of the receive buffer.
 *
 * See also \ref UART_SETRXBUFHWMARK.
 */
#define UART_GETRXBUFHWMARK     0x0127

/*! \brief UART _ioctl() command code to set the block read mode.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which contains the requested block read mode, either 0 (disabled)
 * or 1 (enabled).
 *
 * See also \ref UART_GETBLOCKREAD.
 */
#define UART_SETBLOCKREAD       0x0128

/*! \brief UART _ioctl() command code to query the the block read mode.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which receives the current read block mode, either 0 (disabled)
 * or 1 (enabled).
 *
 * See also \ref UART_SETBLOCKREAD.
 */
#define UART_GETBLOCKREAD       0x0129

/*! \brief UART _ioctl() command code to set the block write mode
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which contains the requested block write mode, either 0 (disabled)
 * or 1 (enabled).
 *
 * See also \ref UART_GETBLOCKWRITE.
 */
#define UART_SETBLOCKWRITE      0x012a

/*! \brief UART _ioctl() command code to query the the block write mode.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which receives the current write block mode, either 0 (disabled)
 * or 1 (enabled).
 *
 * See also \ref UART_SETBLOCKWRITE.
 */
#define UART_GETBLOCKWRITE      0x012b

/*! \brief UART _ioctl() command code to set physical device to the raw mode.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which contains the requested raw mode, either 0 (disabled) or 1 (enabled).
 *
 * In raw mode data encapsulation is not allowed to be done. This allows other
 * processing to be done on physical device after a network device has been
 * attached.
 *
 * See also \ref UART_GETRAWMODE.
 */
#define UART_SETRAWMODE         0x012c

/*! \brief UART _ioctl() command code to query the raw mode.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which receives the current raw mode, either 0 (disabled) or 1 (enabled).
 *
 * See also \ref UART_SETRAWMODE.
 */
#define UART_GETRAWMODE         0x012d

/*! \brief AHDLC _ioctl() command code to set the ACCM (Control Character Mask).
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which contains the requested character mask.
 *
 * During the LCP stage of PPP negotiation both peers inform each other which
 * of the control characters (0-31) will not require escaping when being
 * transmitted.  This allows the PPP layer to tell the HDLC layer about this
 * so that data may be transmitted quicker (no escapes).
 *
 * See also \ref HDLC_GETTXACCM.
 */
#define HDLC_SETTXACCM          0x012e

/*! \brief AHDLC _ioctl() command code to get the ACCM (Control Character Mask).
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which receives the current character mask.
 *
 * See also \ref HDLC_SETTXACCM.
 */
#define HDLC_GETTXACCM          0x012f

/*! \brief UART _ioctl() command code to set physical device to half duplex mode.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which contains the requested half duplex mode, either 0 (disabled) or
 * 1 (enabled).
 *
 * See also \ref UART_GETHDPXMODE.
 */
#define UART_SETHDPXMODE        0x0130

/*! \brief UART _ioctl() command code to query the halfduplex mode.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which receives the current half duplex mode, either 0 (disabled) or
 * 1 (enabled).
 *
 * See also \ref UART_SETHDPXMODE.
 */
#define UART_GETHDPXMODE        0x0131

/*! \brief UART _ioctl() command code to set the OWI halfduplex mode.
 *
 * If device supports, Rx is connected internally to TX, TX is set to
 * multidrive and TX is pulled up. Ioctl may succeed even when pull-up
 * is not realized. Contrary to
 * Contrary to half duplex mode, receiver is active during transmit.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which contains the requested owi mode, either 0 (disabled) or
 * 1 (enabled).
 *
 * See also \ref UART_GETOWIMODE.
 */
#define UART_SETOWIMODE        0x0132

/*! \brief UART _ioctl() command code to query the OWI halfduplex mode.
 *
 * The configuration parameter must be a pointer to a uint32_t variable,
 * which receives the current OWI half duplex mode, either 0 (disabled) or
 * 1 (enabled).
 *
 * See also \ref UART_SETOWIMODE.
 */
#define UART_GETOWIMODE        0x0133

/*!
 * \addtogroup xgUARTStatus
 * \brief UART device status flags,
 *
 * \anchor xaUARTStatus
 * A combination of these status flags is used by the _ioctl() commands
 * \ref UART_SETSTATUS and \ref UART_GETSTATUS.
 */
/*@{*/

/*! \brief Framing error.
 *
 * \ref UART_SETSTATUS will clear this error.
 */
#define UART_FRAMINGERROR   0x00000001UL

/*! \brief Overrun error.
 *
 * \ref UART_SETSTATUS will clear this error.
 */
#define UART_OVERRUNERROR   0x00000002UL

/*! \brief Parity error.
 *
 * \ref UART_SETSTATUS will clear this error.
 */
#define UART_PARITYERROR    0x00000004UL

/*! \brief Break condition.
 *
 * \ref UART_SETSTATUS will clear this status flag.
 */
#define UART_BREAKCONDITION 0x00000008UL

/*! \brief UART errors and status flags.
 *
 * \ref UART_SETSTATUS will clear all errors and flags.
 */
#define UART_ERRORS         (UART_FRAMINGERROR | UART_OVERRUNERROR | UART_PARITYERROR | UART_BREAKCONDITION)

/*! \brief Receiver buffer empty.
 */
#define UART_RXBUFFEREMPTY  0x00000040UL

/*! \brief Transmitter buffer empty.
 *
 * \ref UART_SETSTATUS will immediately clear the buffer. It will not
 * wait until the remaining characters have been transmitted.
 */
#define UART_TXBUFFEREMPTY  0x00000080UL

/*! \brief RTS handshake output enabled.
 */
#define UART_RTSENABLED     0x00000100UL

/*! \brief RTS handshake output disabled.
 */
#define UART_RTSDISABLED    0x00000200UL

/*! \brief CTS handshake input enabled.
 */
#define UART_CTSENABLED     0x00000400UL

/*! \brief CTS handshake input disabled.
 */
#define UART_CTSDISABLED    0x00000800UL

/*! \brief DTR handshake output enabled.
 */
#define UART_DTRENABLED     0x00001000UL

/*! \brief DTR handshake output disabled.
 */
#define UART_DTRDISABLED    0x00002000UL

/*! \brief Receiver enabled.
 */
#define UART_RXENABLED      0x00010000UL

/*! \brief Receiver disabled.
 */
#define UART_RXDISABLED     0x00020000UL

/*! \brief Transmitter enabled.
 */
#define UART_TXENABLED      0x00040000UL

/*! \brief Transmitter disabled.
 */
#define UART_TXDISABLED     0x00080000UL

/*! \brief Receive address frames only.
 *
 * Used in multidrop communication. May only work if 9 databits have
 * been configured.
 */
#define UART_RXADDRFRAME    0x00100000UL

/*! \brief Receive all frames.
 *
 * Used in multidrop communication.
 */
#define UART_RXNORMFRAME    0x00200000UL

/*! \brief Transmit as address frame.
 *
 * Used in multidrop communication. May only work if 9 databits have
 * been configured.
 */
#define UART_TXADDRFRAME    0x00400000UL

/*! \brief Transmit as normal frame.
 *
 * Used in multidrop communication.
 */
#define UART_TXNORMFRAME    0x00800000UL


/*@}*/

/*!
 * \addtogroup xgUARTHS
 * \brief UART handshake modes.
 *
 * \anchor xaUARTHS
 * Any of these values may be used by the _ioctl() commands
 * \ref UART_SETFLOWCONTROL and \ref UART_GETFLOWCONTROL.
 */
/*@{*/

/*! \brief CTS hardware handshake.
 *
 * Nut/OS uses DTE definitions, where CTS is input.
 */
#define UART_HS_CTS      0x0001

/*! \brief RTS hardware handshake.
 *
 * Nut/OS uses DTE definitions, where RTS is output.
 */
#define UART_HS_RTS      0x0002

/*! \brief RTS / CTS hardware handshake.
 *
 * Nut/OS uses DTE definitions, where RTS is output and CTS is input.
 */
#define UART_HS_RTSCTS   (UART_HS_CTS | UART_HS_RTS)

/*! \brief Full modem hardware handshake.
 *
 * Not supported yet by the standard drivers.
 */
#define UART_HS_MODEM       0x001F

/*! \brief XON / XOFF software handshake.
 *
 * It is recommended to set a proper read timeout with software handshake.
 * In this case a timeout may occur, if the communication peer lost our
 * last XON character. The application may then use ioctl() to disable the
 * receiver and do the read again. This will send out another XON.
 */
#define UART_HS_SOFT        0x0020

/*! \brief Half duplex mode.
 */
#define UART_HS_HALFDUPLEX  0x0400

/*@}*/


/*!
 * \addtogroup xgUARTClock
 * \brief UART device clock modes.
 *
 * \anchor xaUARTClock
 * Any of these values may be used by the _ioctl() commands
 * \ref UART_SETCLOCKMODE and \ref UART_GETCLOCKMODE. Most drivers
 * require to set the bit rate after modifying the clock mode. In order
 * to avoid unknown clock output frequencies in master mode, set the
 * clock mode to \ref UART_SYNCSLAVE first, than use \ref UART_SETSPEED
 * to select the bit rate and finally switch to \ref UART_SYNCMASTER or
 * \ref UART_NSYNCMASTER.
 */
/*@{*/

#define UART_SYNC           0x01
#define UART_MASTER         0x02
#define UART_NCLOCK         0x04
#define UART_HIGHSPEED      0x20

/*! \brief Normal asynchronous mode.
 */
#define UART_ASYNC          0x00

/*! \brief Synchronous slave mode.
 *
 * Transmit data changes on rising edge and receive data is sampled on
 * the falling edge of the clock input.
 */
#define UART_SYNCSLAVE     UART_SYNC

/*! \brief Synchronous master mode.
 *
 * Transmit data changes on rising edge and receive data is sampled on
 * the falling edge of the clock output.
 */
#define UART_SYNCMASTER    (UART_SYNC | UART_MASTER)

/*! \brief Synchronous slave mode, clock negated.
 *
 * Similar to \ref UART_SYNCSLAVE, but transmit data changes on falling
 * edge and receive data is sampled on the rising edge of the clock input.
 */
#define UART_NSYNCSLAVE    (UART_SYNC | UART_NCLOCK)

/*! \brief Synchronous master mode, clock negated
 *
 * Similar to \ref UART_SYNCMASTER, but transmit data changes on falling
 * edge and receive data is sampled on the rising edge of the clock output.
 */
#define UART_NSYNCMASTER   (UART_SYNC | UART_NCLOCK | UART_MASTER)

/*! \brief Asynchronous high speed mode.
 *
 * More deviation sensitive than normal mode, but supports higher speed.
 */
#define UART_ASYNC_HS      UART_HIGHSPEED

/*@}*/

/*@}*/

#endif
