#ifndef DEV_USART_CB_H_
#define DEV_USART_CB_H_

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

#include <sys/device.h>
#include <dev/circbuff.h>

/*!
 * \defgroup xgUsartCb Generic USART Driver Frame
 * \ingroup xgDevSerial
 * \anchor xrUsartCb
 * \brief Universal synchronous/asynchronous receiver/transmitter device driver.
 *
 * This new driver frame and all related low level drivers had been
 * partly tested by running the new PPP HDLC driver on the Ethernut 3.1D
 * reference board. No other tests had been done. This early code is
 * incomplete and probably buggy.
 *
 * Quite early it had been discovered, that UART drivers require a
 * set of functions, which are common in all implementations. To
 * simplify porting the code to different driver strategies and
 * hardware interfaces, the UART drivers had been divided into a general,
 * upper level part, also called the frame, and a lower level hardware
 * dependant part.
 *
 * This is the second version of a USART driver frame. Although
 * optimized for the 8 bit AVR family, the
 * \ref xrUsart "original USART driver frame"
 * had been directly ported to almost all supported 32 bit platforms.
 * This resulted in several problems:
 *
 * - To support AVR's single bit operations, UART register accesses were
 *   done using absolute addresses. For the 32 bit targets this creates
 *   duplicate code without benefit.
 * - The new targets offer new features, which were added to the original
 *   code, making it larger without benefit for devices, which do not
 *   support such functions. Over the time, the code became less readable
 *   and maintainable, and sometimes broke existing code.
 * - Many variable sizes do not fit well, either to 8 bit or 32 bit
 *   targets.
 *
 * This new approach tries to solve the mentioned issues.
 *
 * - Low level driver functions are called with a pointer to the driver
 *   control block, so that these functions may be re-used for all UART
 *   interfaces. In any case, it is still up to the low level driver
 *   designer whether he wants to re-use code or implement individual
 *   routines for each interface.
 * - Lower level drivers may enhance or replace single functions of this
 *   upper level driver. Actually this would have been possible with the
 *   previous version as well, but was not well understood and documented.
 * - Low level drivers implement a local ioctl function to keep devices
 *   specific enhancements local. This also significantly reduces the
 *   the number of interface functions between the upper and lower level
 *   driver.
 * - Wherever it makes sense, stdint fast types are preferred, providing
 *   the best performance / size ratio on all targets.
 * - New generic circular buffer functions are used, which automatically
 *   adjust to a maximum buffer size. This allows the AVR to use 8 bit
 *   buffer indices without constraining 32 bit targets.
 *
 * There are a few substantial changes compared to the original driver:
 *
 * - The low level interface is not compatible. Existing implementations
 *   must be re-written, if they want to use this new driver frame.
 *   However, the old driver will be still available.
 * - Transmit watermarks had been removed. The transmitter is now started
 *   as soon as data is available in the transmit buffer.
 */
/*@{*/

/*!
 * \brief Circular receive buffer structure type.
 */
typedef struct _USARTCB_RXBUFF USARTCB_RXBUFF;

/*!
 * \brief Circular receive buffer structure.
 *
 * See also \ref _USARTCB_TXBUFF.
 */
struct _USARTCB_RXBUFF {
    volatile uint8_t *rxb_buf;
    cb_size_t rxb_siz;
    cb_size_t rxb_rdi;
    volatile cb_size_t rxb_wri;
    HANDLE rxb_que;
    volatile cb_size_t rxb_cnt;
};

/*!
 * \brief Circular transmit buffer structure type.
 */
typedef struct _USARTCB_TXBUFF USARTCB_TXBUFF;

/*!
 * \brief Circular transmit buffer structure.
 *
 * \todo Both, this transmit buffer and the related receive buffer
 *       \ref _USARTCB_RXBUFF are directly "mapped" on to the generic
 *       \ref _CIRCBUFF. However, all variables, which are modified
 *       in the related receive or transmit interrupt are marked
 *       volatile here, but are non-volatile in the generic buffer.
 *       This seems to work, because all accesses to generic buffer
 *       are done exclusively. Nevertheless, this is a miserable
 *       hack and may fail with some compilers. We need to further
 *       evaluate, if volatile is required or helpful at all or
 *       whether it may be replaced by memory barriers. Furthermore,
 *       instead of casting buffer structure types, we may use
 *       unions for the volatile parts. But it is not yet known,
 *       how compilers will handle unions of volatile and non-
 *       volatile structure members.
 */
struct _USARTCB_TXBUFF {
    uint8_t *txb_buf;
    cb_size_t txb_siz;
    volatile cb_size_t txb_rdi;
    cb_size_t txb_wri;
    HANDLE txb_que;
    volatile cb_size_t txb_cnt;
};

/*!
 * \brief USART device driver control structure type.
 *
 * See \ref USARTCB_DCB.
 */
typedef struct _USARTCB_DCB USARTCB_DCB;

/*!
 * \brief USART device driver control structure.
 */
struct _USARTCB_DCB {
    /*! \brief Hardware interface specific information.
     *
     * In most cases this will simply contain the base address of the
     * hardware interface. However, complex drivers may use this entry
     * as a pointer to a private structure.
     */
    uintptr_t usart_hwif;
    /*! \brief Enable the low level driver.
     *
     * Called by the upper level driver during device registration.
     * The low level driver may do all required hardware initialization
     * but must not enable any receive or transmit interrupt, because
     * no buffers are available at this stage.
     */
    int (*usart_enable) (USARTCB_DCB *);
    /*! \brief Disable the low level driver.
     *
     * Once called, the low level driver must stop all activities
     * until it will be enabled again via _USARTCB_DCB::usart_enable.
     */
    int (*usart_disable) (USARTCB_DCB *);
    /*! \brief Perform USART hardware control functions.
     *
     * See \ref UsartCbIoCtrl.
     */
    int (*usart_control) (USARTCB_DCB *, int, void *);
    /*! \brief Mode flags.
     *
     * The flags indicate the mode the driver is currently running in.
     */
    uint32_t usart_mode;
    /*! \brief Capability flags.
     *
     * The flags indicate the driver's capabilities.
     *
     * \todo The capability flags are not yet used, but look useful.
     */
    uint32_t usart_caps;
    /*! \brief Set and query the current status. */
    uint32_t (*usart_status) (USARTCB_DCB *, uint32_t);
    /*! \brief Transmit buffer. */
    USARTCB_TXBUFF usart_tx_buff;
    /*! \brief Write timeout. */
    uint32_t usart_wr_tmo;
    /*! \brief Enable transmitter. */
    void (*usart_tx_start) (USARTCB_DCB *);
    /*! \brief Disable transmitter. */
    void (*usart_tx_stop) (USARTCB_DCB *);
    /*! \brief Receive buffer. */
    USARTCB_RXBUFF usart_rx_buff;
    /*! \brief Read timeout. */
    uint32_t usart_rd_tmo;
    /*! \brief Flag if last character was a carriage return. */
    uint_fast8_t usart_rx_cr;
    /*! \brief Receiver low watermark. */
    cb_size_t usart_rx_lowm;
    /*! \brief Receiver high watermark. */
    cb_size_t usart_rx_hiwm;
    /*! \brief Enable receiver. */
    void (*usart_rx_start) (USARTCB_DCB *);
    /*! \brief Disable receiver. */
    void (*usart_rx_stop) (USARTCB_DCB *);
};

/*!
 * \brief Initialize the USART device.
 * \internal
 *
 * This function will call the low level driver's _USARTCB_DCB::usart_enable
 * routine to initialize the hardware. Typically, these low level drivers
 * use a pointer to this function for their _NUTDEVICE::dev_init entry.
 *
 * \param dev Identifies the device to initialize. The validity is checked
 *            only if NUTDEBUG_USE_ASSERT has been defined.
 *
 * \return 0 on success, -1 otherwise.
 *
 * \todo Set initial speed. However, the current implementation does
 *       not allow to configure or disable USART_INITSPEED. Many
 *       applications require individual baud rates at each device
 *       and setting USART_INITSPEED here would result in wasted
 *       code space.
 */
extern int UsartCbInit(NUTDEVICE *dev);

/*!
 * \brief Perform USART driver control functions.
 * \internal
 *
 * This function is called by the _ioctl() function of the C runtime
 * library.
 *
 * \param dev  Identifies the device that receives the device-control
 *             function.
 * \param req  Requested control function, of which the following are
 *             processed by the hardware independent part:
 *             - \ref UART_SETREADTIMEOUT
 *             - \ref UART_GETREADTIMEOUT
 *             - \ref UART_SETWRITETIMEOUT
 *             - \ref UART_GETWRITETIMEOUT
 *             - \ref UART_SETCOOKEDMODE
 *             - \ref UART_GETCOOKEDMODE
 *             - \ref UART_SETRXBUFSIZ
 *             - \ref UART_GETRXBUFSIZ
 *             - \ref UART_SETTXBUFSIZ
 *             - \ref UART_GETTXBUFSIZ
 *             - \ref UART_SETRXBUFLWMARK
 *             - \ref UART_GETRXBUFLWMARK
 *             - \ref UART_SETRXBUFHWMARK
 *             - \ref UART_GETRXBUFHWMARK
 *             Any other function is passed to the ioctl function of the
 *             low level driver.
 * \param conf Points to a buffer that contains any data required for
 *             the given control function or receives data from that
 *             function.
 *
 * \return 0 on success, -1 otherwise.
 *
 * \todo Right now this upper layer function is called first, passing
 *       all unhandled functions to the lower level driver. It seems
 *       to make more sense to do it the other way round, calling the
 *       lower level first.
 *
 * \note Not all control functions may be supported on all platforms.
 *       In any case applications should check the returned result.
 */
extern int UsartCbIoCtrl(NUTDEVICE *dev, int req, void *conf);

/*!
 * \brief Open a USART device.
 * \internal
 *
 * This function will initialize the circular buffer and then call the
 * low level driver's _USARTCB_DCB::usart_rx_start routine to enable
 * the interrupt driven receiver. Typically, these low level drivers
 * use a pointer to this function for their _NUTDEVICE::dev_open entry.
 *
 * Note, that by default the caller has to take care, that all parameters
 * passed to this function are valid. For debugging purposes
 * NUTDEBUG_USE_ASSERT may be defined to enable internal sanity checks.
 *
 * \param dev  Pointer to the NUTDEVICE structure.
 * \param name Ignored, should point to an empty string.
 * \param mode Operation mode. Any of the following values may be or-ed:
 *             - \ref _O_BINARY
 *             - \ref _O_RDONLY
 *             - \ref _O_WRONLY
 * \param acc  Ignored, should be zero.
 *
 * \return Pointer to a NUTFILE structure if successful or NUTFILE_EOF
 *         otherwise.
 *
 * \todo This function ignores _O_RDONLY and _O_WRONLY and always creates
 *       receive and transmit buffers.
 * \todo The initial buffer sizes are currently set to 255, but should
 *       be configurable. However, the current implementation does not
 *       allow to configure or disable USART_TXBUFSIZ and USART_RXBUFSIZ.
 * \todo The initial watermarks are currently set to 1/4 and 3/4 of the
 *       buffer size, but should be configurable. However, the current
 *       implementation does not allow to configure or disable
 *       USART_RXLOWMARK and USART_RXHIWMARK. The hard coded values do
 *       not consider USART_RXBUFSIZ.
 */
extern NUTFILE *UsartCbOpen(NUTDEVICE *dev, const char *name, int mode, int acc);

/*!
 * \brief Close a USART device.
 * \internal
 *
 * If the transmit buffer contains any data, then the low level transmitter
 * will be enabled by calling the _USARTCB_DCB::usart_tx_start routine
 * and the function blocks the calling thread until either all data has
 * been transmitted or a write timeout occurred, whichever comes first.
 *
 * Then the low level functions _USARTCB_DCB::usart_rx_stop and
 * _USARTCB_DCB::usart_tx_stop are called to disable all interrupts
 * and the related circular buffers are released as well.
 *
 * Typical low level UART drivers contain a pointer to this function
 * in their _NUTDEVICE::dev_close entry.
 *
 * \param nfp Pointer to a _NUTFILE structure, obtained by a previous call
 *            to UsartCbOpen(). The validity is checked only if
 *            NUTDEBUG_USE_ASSERT has been defined.
 *
 * \return 0 if all buffered data had been transmitted or -1 otherwise.
 *
 * \todo We may support shared open and use dev_irq as an open counter.
 */
extern int UsartCbClose(NUTFILE *fp);

/*!
 * \brief Read from device.
 * \internal
 *
 * The function blocks the calling thread until at least one character
 * is available in the receive buffer or until a read timeout occurs,
 * whichever comes first.
 *
 * Typical low level UART drivers contain a pointer to this function
 * in their _NUTDEVICE::dev_read entry.
 *
 * Note, that by default the caller has to take care, that all parameters
 * passed to this function are valid. For debugging purposes
 * NUTDEBUG_USE_ASSERT may be defined to enable internal sanity checks.
 *
 * \param nfp    Pointer to a \ref _NUTFILE structure, obtained by a
 *               previous call to UsartCbOpen().
 * \param buffer Pointer to the buffer that receives the data. If NULL,
 *               then all characters in the input buffer will be
 *               discarded.
 * \param size   Maximum number of bytes to read. If the buffer pointer
 *               is NULL, then this should be 0.
 *
 * \return The number of bytes read, which may be less than the number
 *         of bytes specified. A return value of -1 indicates an error,
 *         while zero is returned in case of a timeout.
 */
extern int UsartCbRead(NUTFILE *fp, void *buffer, int size);

/*!
 * \brief Write to device.
 * \internal
 *
 * Depending on the available transmit buffer space, the calling function
 * may or may not be blocked. In any case, the low level transmitter will
 * be started by calling _USARTCB_DCB::usart_tx_start.
 *
 * Typical low level UART drivers contain a pointer to this function
 * in their _NUTDEVICE::dev_write entry.
 *
 * Note, that by default the caller has to take care, that all parameters
 * passed to this function are valid. For debugging purposes
 * NUTDEBUG_USE_ASSERT may be defined to enable internal sanity checks.
 *
 * \param nfp    Pointer to a \ref _NUTFILE structure, obtained by a
 *               previous call to UsartCbOpen().
 * \param buffer Pointer to the data to be written. If NULL, then the
 *               the function blocks the calling thread until all
 *               characters in the transmit buffer had been sent out or
 *               until a write timeout occurs, whichever comes first.
 * \param len    Number of bytes to write. If the buffer pointer
 *               is NULL, then this should be 0.
 *
 * \return The number of bytes written, which, in case of a write timeout
 *         may be less than the number of bytes requested by the caller.
 *         A return value of -1 indicates an error.
 */
extern int UsartCbWrite(NUTFILE *fp, const void *buffer, int len);
#ifdef __HARVARD_ARCH__
extern int UsartCbWrite_P(NUTFILE *nfp, PGM_P buffer, int len);
#endif

/*!
 * \brief Retrieves the number of characters in input buffer.
 * \internal
 *
 * This function allows to query the number of bytes in the input buffer
 * by using standard C function for querying the size of an open file.
 *
 * \param nfp Pointer to a _NUTFILE structure, obtained by a previous call
 *            to UsartCbOpen(). The validity is checked only if
 *            NUTDEBUG_USE_ASSERT has been defined.
 *
 * \return The number of bytes currently available in the receive buffer.
 */
extern long UsartCbSize (NUTFILE *fp);

/*@}*/

#endif
