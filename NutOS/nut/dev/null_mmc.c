/*
 * Copyright (C) 2016 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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

#include <sys/device.h>
#include <dev/mmcard.h>

/*!
 * \brief Perform MMC control functions.
 *
 * This function is called by the ioctl() function of the C runtime
 * library. Applications should not directly call this function.
 *
 * \todo Card change detection should verify the serial card number.
 *
 * \param dev  Identifies the device that receives the device-control
 *             function.
 * \param req  Requested control function. May be set to one of the
 *             following constants:
 *             - \ref NUTBLKDEV_MEDIACHANGE
 *             - \ref NUTBLKDEV_INFO
 *             - \ref NUTBLKDEV_SEEK
 *             - \ref MMCARD_GETCID
 *             - \ref MMCARD_GETCSD
 *
 * \param conf Points to a buffer that contains any data required for
 *             the given control function or receives data from that
 *             function.
 * \return -1 always, as no functionality provided
 */
static int NullMmCardIOCtl(NUTDEVICE * dev, int req, void *conf)
{
    return -1;
}

/*!
 * \brief Read data blocks from a mounted partition.
 *
 * Applications should not call this function directly, but use the
 * stdio interface.
 *
 * \param nfp    Pointer to a ::NUTFILE structure, obtained by a previous
 *               call to MmCardMount().
 * \param buffer Pointer to the data buffer to fill.
 * \param num    Maximum number of blocks to read. However, reading
 *               multiple blocks is not yet supported by this driver.
 *
 * \return Always -1, as no functionality is implemented.
 */
static int NullMmCardBlockRead(NUTFILE * nfp, void *buffer, int num)
{
    return -1;
}

/*!
 * \brief Write data blocks to a mounted partition.
 *
 * Applications should not call this function directly, but use the
 * stdio interface.
 *
 * \param nfp    Pointer to a \ref NUTFILE structure, obtained by a previous
 *               call to MmCardMount().
 * \param buffer Pointer to the data to be written.
 * \param num    Maximum number of blocks to write. However, writing
 *               multiple blocks is not yet supported by this driver.
 *
 * \return Always -1, as no functionality is implemented.
 */
static int NullMmCardBlockWrite(NUTFILE * nfp, const void *buffer, int num)
{
    return -1;
}

/*!
 * \brief Mount a partition.
 *
 * Nut/OS doesn't provide specific routines for mounting. Instead routines
 * for opening files are used.
 *
 * Applications should not directly call this function, but use the high
 * level stdio routines for opening a file.
 *
 * \param dev  Pointer to the MMC device.
 * \param name Partition number followed by a slash followed by a name
 *             of the file system device. Both items are optional. If no
 *             file system driver name is given, the first file system
 *             driver found in the list of registered devices will be
 *             used. If no partition number is specified or if partition
 *             zero is given, the first active primary partition will be
 *             used.
 * \param mode Opening mode. Currently ignored, but
 *             \code _O_RDWR | _O_BINARY \endcode should be used for
 *             compatibility with future enhancements.
 * \param acc  File attributes, ignored.
 *
 * \return Always NUTFILE_EOF as no functionality is implemented.
 */
NUTFILE *NullMmCardMount(NUTDEVICE * dev, const char *name, int mode, int acc)
{
    return  NUTFILE_EOF;
}

/*!
 * \brief Multimedia card device information structure.
 *
 * A pointer to this structure must be passed to NutRegisterDevice()
 * to bind this driver to the Nut/OS kernel. An application may then
 * call
 * /verbatim
 * _open("MMC0:", _O_RDWR | _O_BINARY);
 * /endverbatim
 * to mount the first active primary partition with any previously
 * registered file system driver (typically devPhat0).
 */
NUTDEVICE devNullMmc0 = {
    0,                    /*!< Pointer to next device, dev_next. */
    {'M', 'M', 'C', '0', 0, 0, 0, 0, 0}
    ,                     /*!< Unique device name, dev_name. */
    0,                    /*!< Type of device, dev_type. Obsolete. */
    0,                    /*!< Base address, dev_base. Unused. */
    0,                    /*!< First interrupt number, dev_irq. Unused. */
    NULL,                 /*!< Interface control block, dev_icb. */
    NULL,                 /*!< Driver control block used by the low level part, dev_dcb. */
    MmCardDevInit,        /*!< Driver initialization routine, dev_init. */
    NullMmCardIOCtl,      /*!< Driver specific control function, dev_ioctl. */
    NullMmCardBlockRead,  /*!< Read data from a file, dev_read. */
    NullMmCardBlockWrite, /*!< Write data to a file, dev_write. */
#ifdef __HARVARD_ARCH__
    MmCardBlockWrite_P,   /*!< Write data from program space to a file, dev_write_P. */
#endif
    NullMmCardMount,     /*!< Mount a file system, dev_open. */
    MmCardUnmount,       /*!< Unmount a file system, dev_close. */
    NULL,                /*!< Return file size, dev_size. */
    NULL,                /*!< Select function, optional, not yet implemented */
};
