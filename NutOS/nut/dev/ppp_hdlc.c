/*
 * Copyright (C) 2012 by egnite GmbH
 * Copyright (C) 2008 by Szemzo András
 * Copyright (C) 2003-2004 by egnite Software GmbH
 * Copyright (c) 1996 - 2001 Brian Somers <brian@Awfulhak.org>
 *          based on work by Toshiharu OHNO <tony-o@iij.ad.jp>
 *                           Internet Initiative Japan, Inc (IIJ)
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

/*!
 * \file dev/ppp_hdlc.c
 * \brief Generic AHDLC driver for PPP.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <dev/uart.h>
#include <dev/ahdlc.h>
#include <dev/ppp.h>

#include <sys/event.h>
#include <sys/timer.h>
#include <sys/thread.h>

#include <net/if_var.h>

#include <stdlib.h>
#include <string.h>
#include <io.h>
#include <fcntl.h>

#include <dev/ppp_hdlc.h>

/*!
 * \brief AHDLC receiver thread stack size.
 */
#ifndef NUT_THREAD_AHDLCRXSTACK
#define NUT_THREAD_AHDLCRXSTACK 1024
#endif
#define RXTHREADSTACK ((NUT_THREAD_AHDLCRXSTACK) * (NUT_THREAD_STACK_MULT) + (NUT_THREAD_STACK_ADD))

/*!
 * \brief Check character against Async-Control-Character-Map.
 *
 * Checks the 32-bit ACCM to see if the byte needs un-escaping
 */
#define IN_ACC_MAP(c, m) (( ((uint8_t) (c)) < 0x20)  && ((m) & (1UL << (c))) != 0)

/*
 * FCS lookup table.
 */
static const uint16_t fcstab[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

typedef struct _PPPHDLC_DCB PPPHDLC_DCB;

struct _PPPHDLC_DCB {
    /* ! \brief Downlink device. */
    int dcb_fd;

    /*! \brief HDLC mode change event queue.
     */
    HANDLE dcb_mode_evt;

    /*! \brief Peer's Async-Control-Character-Map.
     */
    uint32_t dcb_rx_accm;

    /*! \brief Our Async-Control-Character-Map.
     */
    uint32_t dcb_tx_accm;

    /*! \brief Maximum receive and transmit MRU.
     */
    uint_fast16_t dcb_mru;
};

static PPPHDLC_DCB dcb_ppp0;
static PPPHDLC_DCB dcb_ppp1;

/*!
 * \brief Send byte to physical device.
 *
 * \return 0 on success, -1 in case of any errors.
 */
static int INLINE PppHdlcSendByte(int fd, uint8_t ch, uint8_t flush)
{
    if (_write(fd, &ch, 1) != 1)
        return -1;
    return 0;
}

/*
 * Characters are properly escaped and checksum is updated.
 *
 * \return 0 on success, -1 in case of any errors.
 */
static int PppHdlcSend(PPPHDLC_DCB *dcb, const uint8_t *data, uint16_t len, uint16_t *txfcs)
{
    uint16_t tbx;
    register uint16_t fcs;
    uint8_t ch;

    if (txfcs)
        fcs = *txfcs;
    else
        fcs = 0;
    while (len) {
        tbx = (uint16_t) ((uint8_t) fcs ^ *data);
        fcs >>= 8;
        fcs ^= fcstab[tbx];
        if (IN_ACC_MAP(*data, dcb->dcb_tx_accm) || *data == AHDLC_FLAG || *data == AHDLC_ESCAPE) {
            ch = AHDLC_ESCAPE;
            if (PppHdlcSendByte(dcb->dcb_fd, AHDLC_ESCAPE, 0))
                return -1;
            if (PppHdlcSendByte(dcb->dcb_fd, *data ^ AHDLC_TRANS, 0))
            if (_write(dcb->dcb_fd, &ch, 1) != 1)
                return -1;
        } else if (PppHdlcSendByte(dcb->dcb_fd, *data, 0))
            return -1;
        data++;
        len--;
    }
    if (txfcs)
        *txfcs = fcs;

    return 0;
}

/*!
 * \brief Send HDLC frame.
 *
 * \param dev Identifies the device to use.
 * \param nb  Network buffer structure containing the packet to be sent.
 *            The structure must have been allocated by a previous
 *            call NutNetBufAlloc().
 *
 * \return 0 on success, -1 in case of any errors.
 */
static int PppHdlcOutput(NUTDEVICE *dev, NETBUF *nb)
{
    uint16_t txfcs;
    PPPHDLC_DCB *dcb = dev->dev_dcb;
    uint_fast16_t sz;

    /*
     * Calculate the number of bytes to be send. Do not
     * send packets larger than transmit mru.
     */
    sz = nb->nb_dl.sz + nb->nb_nw.sz + nb->nb_tp.sz + nb->nb_ap.sz;
    if (sz > dcb->dcb_mru) {
        return -1;
    }

    /* Send the starting flag. */
    PppHdlcSendByte(dcb->dcb_fd, AHDLC_FLAG, 0);

    /* Initialize the checksum and send the NETBUF. */
    txfcs = AHDLC_INITFCS;
    if (PppHdlcSend(dcb, nb->nb_dl.vp, nb->nb_dl.sz, &txfcs))
        return -1;
    if (PppHdlcSend(dcb, nb->nb_nw.vp, nb->nb_nw.sz, &txfcs))
        return -1;
    if (PppHdlcSend(dcb, nb->nb_tp.vp, nb->nb_tp.sz, &txfcs))
        return -1;
    if (PppHdlcSend(dcb, nb->nb_ap.vp, nb->nb_ap.sz, &txfcs))
        return -1;

    /* Send the checksum and the final flag. */
    txfcs ^= 0xffff;
    if (PppHdlcSend(dcb, (uint8_t *) & txfcs, 2, 0))
        return -1;
    PppHdlcSendByte(dcb->dcb_fd, AHDLC_FLAG, 1);

    return 0;
}

/*! \fn PppHdlcReceive(void *arg)
 * \brief Asynchronous HDLC receiver thread.
 *
 * Running at high priority.
 */
THREAD(PppHdlcReceive, arg)
{
    NUTDEVICE *dev = (NUTDEVICE *) arg;
    PPPHDLC_DCB *dcb = (PPPHDLC_DCB *) dev->dev_dcb;
    NUTDEVICE *netdev = (NUTDEVICE *) dev->dev_icb;
    IFNET *ifn = (IFNET *) netdev->dev_icb;
    uint_fast8_t ch;
    uint8_t *rd_buf;
    uint8_t *rd_ptr;
    uint8_t *rx_buf;
    uint8_t *rx_ptr;
    static const int rd_siz = 64;
    int rd_len = 0;
    uint16_t rx_fcs = AHDLC_INITFCS;
    int rx_cnt = 0;
    uint_fast8_t inframe = 0;
    uint_fast8_t escaped = 0;
    uint32_t tmo = 1000;

    rd_buf = malloc(rd_siz);
    rd_ptr = rd_buf;

    _ioctl(dcb->dcb_fd, UART_SETREADTIMEOUT, &tmo);

    dev->dev_type = IFTYP_NET;
    dcb->dcb_mru = ifn->if_mtu;
    rx_buf = malloc(dcb->dcb_mru + 2);
    rx_ptr = rx_buf;
    /* Signal the link driver that we are up. */
    ifn->if_send = PppHdlcOutput;
    netdev->dev_ioctl(netdev, LCP_LOWERUP, 0);
    NutEventPost(&dcb->dcb_mode_evt);

    for (;;) {
        /* Read next character from downlink. */
        while (rd_len == 0) {
            rd_ptr = rd_buf;
            rd_len = _read(dcb->dcb_fd, rd_buf, rd_siz);
            /* Check, if the net interface is still attached. */
            if (dev->dev_icb == NULL) {
                rd_len = -1;
            }
        }
        if (rd_len < 0) {
            /* Device error. */
            break;
        }
        ch = *rd_ptr++;
        rd_len--;

        if (inframe) {
            if (ch != AHDLC_FLAG) {
                if (ch == AHDLC_ESCAPE) {
                    /* Escape next character. */
                    escaped = 1;
                    /* Jump to get next character. */
                    continue;
                }
                if (escaped) {
                    /* Last character was escape. */
                    ch ^= AHDLC_TRANS;
                    escaped = 0;
                }

                if (rx_cnt++ < dcb->dcb_mru + 2) {
                    /* Update calculated checksum and store character in buffer. */
                    uint16_t tbx = (uint16_t) ((uint8_t) rx_fcs ^ ch);

                    rx_fcs >>= 8;
                    rx_fcs ^= fcstab[tbx];
                    *rx_ptr++ = ch;
                } else {
                    /* Size overflow, skip this frame. */
                    inframe = 0;
                }
                /* Jump to get next character. */
                continue;
            }

            /* Frame flag received, check length and checksum. */
            if (rx_cnt >= 2 && rx_fcs == AHDLC_GOODFCS) {
                NETBUF *nb;

                /* Frame is valid. Create a NETBUF and pass it to the
                   network specific receive handler. */
                rx_cnt -= 2;
                nb = NutNetBufAlloc(NULL, NBAF_DATALINK, rx_cnt);
                if (nb) {
                    memcpy(nb->nb_dl.vp, rx_buf, rx_cnt);
                    (*ifn->if_recv) (netdev, nb);
                }
            }
        }

        /* If frame flag is received, re-sync frame processing. */
        if (ch == AHDLC_FLAG) {
            inframe = 1;
            escaped = 0;
            rx_ptr = rx_buf;
            rx_cnt = 0;
            rx_fcs = AHDLC_INITFCS;
        }
    }
    /* Signal the link driver that we are down. */
    netdev->dev_ioctl(netdev, LCP_LOWERDOWN, 0);
    dev->dev_type = IFTYP_CHAR;
    NutEventPost(&dcb->dcb_mode_evt);

    NutThreadExit();
    for (;;);
}

/*
 * \brief Attach or detach the network interface.
 *
 * \param dev    PPP device.
 * \param netdev Network device.
 *
 * \return 0 on success, -1 otherwise.
 */
static int PppHdlcAttach(NUTDEVICE *dev, NUTDEVICE *netdev)
{
    PPPHDLC_DCB *dcb = (PPPHDLC_DCB *) dev->dev_dcb;

    if (netdev) {
        if (dev->dev_icb) {
            return -1;
        }
        dev->dev_icb = netdev;
        if (NutThreadCreate("ppphdlc", PppHdlcReceive, dev, RXTHREADSTACK) == NULL) {
            return -1;
        }
    }
    else if (dev->dev_icb == NULL) {
        return 0;
    }
    else {
        dev->dev_icb = NULL;
    }
    return NutEventWait(&dcb->dcb_mode_evt, 2000);
}

/*!
 * \brief Perform device specific control functions.
 *
 * \param dev  Identifies the device that receives the device-control
 *             function.
 * \param req  Requested control function. The following functions are
 *             handled by this device:
 *             - \ref HDLC_SETIFNET
 *             - \ref HDLC_GETIFNET
 *             All other functions will be passed to the UART driver.
 * \param conf Points to a buffer that contains any data required for
 *             the given control function or receives data from that
 *             function.
 *
 * \return 0 on success, -1 otherwise.
 */
static int PppHdlcIoCtl(NUTDEVICE *dev, int req, void *conf)
{
    int rc = 0;
    PPPHDLC_DCB *dcb;

    dcb = (PPPHDLC_DCB *) dev->dev_dcb;
    switch (req) {
    case HDLC_SETIFNET:
        rc = PppHdlcAttach(dev, *(NUTDEVICE **) conf);
        break;
    case HDLC_GETIFNET:
        *(NUTDEVICE **)conf = (NUTDEVICE *) dev->dev_icb;
        break;

    case HDLC_SETTXACCM:
        dcb->dcb_tx_accm = *((uint32_t *) conf);
        break;
    case HDLC_GETTXACCM:
        *((uint32_t *) conf) = dcb->dcb_tx_accm;
        break;

    default:
        rc = _ioctl(((PPPHDLC_DCB *) (dev->dev_dcb))->dcb_fd, req, conf);
        break;
    }
    return rc;
}

/*!
 * \brief Initialize asynchronous HDLC device.
 *
 * This function will be called during device registration.
 *
 * \param dev  Identifies the device to initialize.
 *
 * \return 0 on success, -1 otherwise.
 */
static int PppHdlcInit(NUTDEVICE * dev)
{
    return 0;
}

/*!
 * \brief Open the asynchronous HDLC device.
 *
 * This function is called internally when opening the PPP driver and
 * will in turn open the physical UART device. Therefore it is required,
 * that the UART driver has been registered before opening the PPP device.
 *
 * \param dev  Pointer to the \ref NUTDEVICE structure.
 * \param name Ignored, should point to an empty string.
 * \param mode Operation mode, should be set to \ref _O_RDWR or-ed with
 *             \ref _O_BINARY.
 * \param acc  Ignored, should be zero.
 *
 * \return Pointer to a NUTFILE structure if successful or NUTFILE_EOF otherwise.
 */
static NUTFILE *PppHdlcOpen(NUTDEVICE * dev, const char *name, int mode, int acc)
{
    PPPHDLC_DCB *dcb;
    NUTFILE *nfp;

    dcb = (PPPHDLC_DCB *) dev->dev_dcb;
    /* Open the physical device. */
    dcb->dcb_fd = _open(dev->dev_name + 1, mode);
    if (dcb->dcb_fd == -1) {
        return NUTFILE_EOF;
    }
    /* Set transmit ACCM prior to negotiation. */
    dcb->dcb_tx_accm = 0xffffffff;
    /* Allocate a file structure to return. */
    nfp = calloc(1, sizeof(*nfp));
    if (nfp == NULL) {
        return NUTFILE_EOF;
    }
    nfp->nf_dev = dev;

    return nfp;
}

/*!
 * \brief Close the asynchronous HDLC device.
 *
 * This function is called internally when closing the PPP driver. It
 * will detach the network interface and close the UART device driver.
 *
 * \param nfp Pointer to a _NUTFILE structure, obtained by a previous call
 *            to PppHdlcOpen().
 *
 * \return 0 on success or -1 otherwise.
 */
static int PppHdlcClose(NUTFILE * nfp)
{
    int rc;

    PppHdlcAttach(nfp->nf_dev, NULL);
    rc = _close(((PPPHDLC_DCB *) (nfp->nf_dev->dev_dcb))->dcb_fd);
    free(nfp);

    return rc;
}

/*!
 * \brief Read raw data from the asynchronous HDLC device.
 *
 * This call is simply passed to the UART driver.
 *
 * \param nfp    Pointer to a \ref _NUTFILE structure, obtained by a
 *               previous call to PppHdlcOpen().
 * \param buffer Pointer to the buffer that receives the data.
 * \param size   Maximum number of bytes to read.
 *
 * \return The number of bytes read, which may be less than the number
 *         of bytes specified. A return value of -1 indicates an error,
 *         while zero is returned in case of a timeout.
 */
static int PppHdlcRead(NUTFILE *nfp, void *buffer, int size)
{
    return _read(((PPPHDLC_DCB *) (nfp->nf_dev->dev_dcb))->dcb_fd, buffer, size);
}

/*!
 * \brief Write raw data to the asynchronous HDLC device.
 *
 * This call is simply passed to the UART driver.
 *
 * \param nfp    Pointer to a \ref _NUTFILE structure, obtained by a
 *               previous call to PppHdlcOpen().
 * \param buffer Pointer the data to write.
 * \param len    Number of data bytes to write.
 *
 * \return The number of bytes written, which may be less than the number
 *         of bytes specified if a timeout occured. A return value of -1
 *         indicates an error.
 */
static int PppHdlcWrite(NUTFILE *nfp, const void *buffer, int len)
{
    return _write(((PPPHDLC_DCB *) (nfp->nf_dev->dev_dcb))->dcb_fd, buffer, len);
}

NUTDEVICE devPppHdlc0 = {
    NULL,           /* Pointer to next device, dev_next. */
    { 'l', 'u', 'a', 'r', 't', '0', 0, 0, 0 }, /* Hardware device name, dev_name. */
    IFTYP_CHAR,     /* Type of device, dev_type. */
    0,              /* Base address, dev_base (not used). */
    0,              /* First interrupt number, dev_irq (not used). */
    NULL,           /* Interface control block, dev_icb. */
    &dcb_ppp0,      /* Driver control block, dev_dcb. */
    PppHdlcInit,    /* Driver initialization routine, dev_init. */
    PppHdlcIoCtl,   /* Driver specific control function, dev_ioctl. */
    PppHdlcRead,    /* Read from device, dev_read. */
    PppHdlcWrite,   /* Write to device, dev_write. */
#ifdef __HARVARD_ARCH__
    NULL,           /* Write data from program space to device, dev_write_P. */
#endif
    PppHdlcOpen,    /* Open a device or file, dev_open. */
    PppHdlcClose,   /* Close a device or file, dev_close. */
    NULL,           /* Request file size, dev_size. */
    NULL,           /* Select function, optional, not yet implemented */
};

NUTDEVICE devPppHdlc1 = {
    NULL,           /* Pointer to next device, dev_next. */
    { 'l', 'u', 'a', 'r', 't', '1', 0, 0, 0 }, /* Hardware device name, dev_name. */
    IFTYP_CHAR,     /* Type of device, dev_type. */
    0,              /* Base address, dev_base (not used). */
    0,              /* First interrupt number, dev_irq (not used). */
    NULL,           /* Interface control block, dev_icb. */
    &dcb_ppp1,      /* Driver control block, dev_dcb. */
    PppHdlcInit,    /* Driver initialization routine, dev_init. */
    PppHdlcIoCtl,   /* Driver specific control function, dev_ioctl. */
    PppHdlcRead,    /* Read from device, dev_read. */
    PppHdlcWrite,   /* Write to device, dev_write. */
#ifdef __HARVARD_ARCH__
    NULL,           /* Write data from program space to device, dev_write_P. */
#endif
    PppHdlcOpen,    /* Open a device or file, dev_open. */
    PppHdlcClose,   /* Close a device or file, dev_close. */
    NULL,           /* Request file size, dev_size. */
    NULL,           /* Select function, optional, not yet implemented */
};
