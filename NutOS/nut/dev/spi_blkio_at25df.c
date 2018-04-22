/*
 * Copyright (C) 2015 by Ole Reinhardt <ole.reinhardt@embedded-it.de>
 * Copyright (C) 2008-2011 by egnite GmbH
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
 * \file dev/spi_blkio_at25df.c
 * \brief Low level block I/O routines for Adesto/Atmel AT25DF SPI DataFlash.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <cfg/os.h>
#include <cfg/memory.h>
#include <sys/nutdebug.h>

#include <dev/blockdev.h>
#include <dev/spi_node_at25df.h>

/*!
 * \addtogroup xgSpiBlockIoAt25df
 */
/*@{*/

/*!
 * \brief Initialize the block I/O interface.
 *
 * \param dev Specifies the blcok I/O device.
 *
 * \return 0 on success or -1 in case of an error.
 */
static int At25dfBlkIoInit(NUTDEVICE * dev)
{
    int          rc = -1;
    NUTBLOCKIO  *blkio;
    NUTSPINODE  *node;
    AT25DF_INFO *df;

    NUTASSERT(dev != NULL);
    NUTASSERT(dev->dev_dcb != NULL);
    NUTASSERT(dev->dev_icb != NULL);
    blkio = dev->dev_dcb;
    node = dev->dev_icb;

    df = At25dfNodeProbe(node);
    if (df) {
        /* Known DataFlash type detected. */
        blkio->blkio_info = (void *) df;
        blkio->blkio_blk_cnt = df->at25df_erase_blocks;
        blkio->blkio_blk_siz = df->at25df_ebsize;
        rc = 0;
    }

    /* Put the flash in write enable mode. */
    if (rc == 0) {
        rc = At25dfNodeTransfer(node, DFCMD_WRITE_ENABLE, 0, 1, NULL, NULL, 0);
    }

    /* Global unprotect the flash */
    if (rc == 0) {
        rc = At25dfNodeTransfer(node, DFCMD_WRITE_STATUS1, 0, 2, NULL, NULL, 0);
    }
           
    /* Wait for the erase operation to complete */
    if (rc == 0) {
        rc = At25dfNodeWaitReady(node, AT25_WRITE_POLLS, 1);
    }    
    return rc;
}

/*!
 * \brief Read data from DataFlash memory.
 *
 * \param dev   Specifies the registered DataFlash device.
 * \param block Erase block number to read, starting at 0.
 * \param data  Points to a buffer that receives the data.
 * \param len   Number of bytes to read.
 *
 * \return The number of bytes actually read. A return value of -1 indicates
 *         an error.
 */
static int At25dfBlkIoRead(NUTDEVICE * dev, uint32_t block, void *data, int len)
{
    NUTBLOCKIO *blkio;
    NUTSPINODE *node;
    AT25DF_INFO *info;
    uint8_t     read_cmd;
    uint8_t     oplen;

    NUTASSERT(dev != NULL);
    NUTASSERT(dev->dev_dcb != NULL);
    blkio = dev->dev_dcb;

    info = (AT25DF_INFO *) blkio->blkio_info;
    NUTASSERT(blkio->blkio_info != NULL);

    if (block >= info->at25df_erase_blocks) {
        return -1;
    }
    block <<= info->at25df_ebshft;

    node = (NUTSPINODE *) dev->dev_icb;

    /* Select the flash read command depending on the selected SPI speed */
    if (node->node_rate <= AT25_MAX_SPEED_SLOW) {
        read_cmd = DFCMD_READ_ARRAY_SLOW;
        oplen = 4;
    } else 
    if (node->node_rate <= AT25_MAX_SPEED_MED) {
        read_cmd = DFCMD_READ_ARRAY_MED;
        oplen = 5;
    } else {
        read_cmd = DFCMD_READ_ARRAY_FAST;
        oplen = 6;
    }

    if (At25dfNodeLock((NUTSPINODE *) dev->dev_icb) ||
        At25dfNodeTransfer((NUTSPINODE *) dev->dev_icb, read_cmd, block, oplen, NULL, data, len)) {
        At25dfNodeUnlock((NUTSPINODE *) dev->dev_icb);
        return -1;
    }
    At25dfNodeUnlock((NUTSPINODE *) dev->dev_icb);
    return len;
}

/*!
 * \brief Write data to DataFlash memory.
 *
 * Each erae block will be automatically erased before writing the data. If the
 * erase block is not completely filled with new data, the contents of
 * remaining bytes at the end of the page is undetermined.
 * 
 * Writing only pages (sub-erase block) is not supported
 *
 * \param dev   Specifies the registered DataFlash device.
 * \param block The erase block number.
 * \param data  Points to the buffer that contains the bytes to be written.
 * \param len   Number of bytes available in the buffer. This may be less
 *              than the erase block size, in which case the remaining bytes of
 *              the erase_block will be set to 0xff.
 *
 * \return The number of bytes actually written. A return value of -1 indicates
 *         an error.
 */
static int At25dfBlkIoWrite(NUTDEVICE * dev, uint32_t block, const void *data, int len)
{
    int          rc = -1;
    uint8_t     *dp = (uint8_t *) data;
    int          step;
    uint_fast8_t ebshft;
    int          psize;
    uint32_t     limit;
    NUTBLOCKIO  *blkio;
    NUTSPINODE  *node;

    /* Sanity check. */
    if (len == 0) {
        return 0;
    }
    NUTASSERT(data != NULL);
    NUTASSERT(dev != NULL);
    NUTASSERT(dev->dev_dcb != NULL);
    NUTASSERT(dev->dev_icb != NULL);
    blkio = (NUTBLOCKIO *) dev->dev_dcb;
    node = (NUTSPINODE *) dev->dev_icb;
    NUTASSERT(blkio->blkio_info != NULL);
    ebshft = ((AT25DF_INFO *) blkio->blkio_info)->at25df_ebshft;
    psize  = ((AT25DF_INFO *) blkio->blkio_info)->at25df_psize;
    limit  = ((AT25DF_INFO *) blkio->blkio_info)->at25df_erase_blocks;

    while (len) {
        step = ((AT25DF_INFO *) blkio->blkio_info)->at25df_ebsize;
        if (step > len) {
            step = len;
        }
        if (At25dfNodeLock(node)) {
            break;
        }

        /* Put the flash in write enable mode. */
        if (At25dfNodeTransfer(node, DFCMD_WRITE_ENABLE, 0, 1, NULL, NULL, 0)) {
            At25dfNodeUnlock(node);
            break;
        }

        /* Erase the erase block. */
        if (At25dfNodeTransfer(node, DFCMD_BLOCK_ERASE_4K, block << ebshft, 4, NULL, NULL, 0)) {
            At25dfNodeUnlock(node);
            break;
        }

        /* Wait for the erasing to be finished */
        if (At25dfNodeWaitReady(node, AT25_BLOCK_ERASE_WAIT, 0)) {
            At25dfNodeUnlock(node);
            break;
        }

        while ((len > 0) && (step > 0)) {
            if (psize > len) {
                /* Make sure to not write more data than needed on the last page */
                psize = len;
            }

            /* Put the flash in write enable mode. */
            if (At25dfNodeTransfer(node, DFCMD_WRITE_ENABLE, 0, 1, NULL, NULL, 0)) {
                At25dfNodeUnlock(node);
                break;
            }

            /* Put the flash in write enable mode. */
            if (At25dfNodeTransfer(node, DFCMD_WRITE, (block << ebshft) + (dp - (uint8_t*)data), 4, dp, NULL, psize)) {
                At25dfNodeUnlock(node);
                break;
            }

            /* Wait for the erasing to be finished */
            if (At25dfNodeWaitReady(node, AT25_WRITE_POLLS, 1)) {
                At25dfNodeUnlock(node);
                break;
            }


            if (rc < 0) {
                rc = 0;
            }
            rc   += psize;
            dp   += psize;
            len  -= psize;
            step -= psize;
        }

        if (++block >= limit) {
            break;
        }        
    }
    return rc;
}

#ifdef __HARVARD_ARCH__
static int At25dfBlkIoWrite_P(NUTDEVICE * dev, uint32_t pgn, PGM_P data, int len)
{
    return -1;
}
#endif

/*!
 * \brief Perform block I/O device control functions.
 *
 * This function is called by the ioctl() function of the C runtime
 * library.
 *
 * \param dev  Identifies the device that receives the control command.
 * \param req  Requested control command. May be set to one of the
 *             following constants:
 *             - \ref NUTBLKDEV_MEDIAAVAIL
 *             - \ref NUTBLKDEV_MEDIACHANGE
 *
 * \param conf Points to a buffer that contains any data required for
 *             the given control function or receives data from that
 *             function.
 * \return 0 on success, -1 otherwise.
 */
static int At25dfBlkIoCtl(NUTDEVICE * dev, int req, void *conf)
{
    int rc = 0;

    switch (req) {
    case NUTBLKDEV_MEDIAAVAIL:
        /* Modification required for DataFlash cards. */
        {
            int *flg;
            NUTASSERT(conf != NULL);
            flg = (int *) conf;
            *flg = 1;
        }
        break;
    case NUTBLKDEV_MEDIACHANGE:
        /* Modification required for DataFlash cards. */
        {
            int *flg;
            NUTASSERT(conf != NULL);
            flg = (int *) conf;
            *flg = 0;
        }
        break;
    default:
        rc = -1;
        break;
    }
    return rc;
}

#ifndef BLKIO_MOUNT_OFFSET_AT25DF0
#ifdef MOUNT_OFFSET_AT25DF0
#define BLKIO_MOUNT_OFFSET_AT25DF0       MOUNT_OFFSET_AT25DF0
#else
#define BLKIO_MOUNT_OFFSET_AT25DF0       0
#endif
#endif

#ifndef BLKIO_MOUNT_TOP_RESERVE_AT25DF0
#ifdef MOUNT_TOP_RESERVE_AT25DF0
#define BLKIO_MOUNT_TOP_RESERVE_AT25DF0  MOUNT_TOP_RESERVE_AT25DF0
#else
#define BLKIO_MOUNT_TOP_RESERVE_AT25DF0  1
#endif
#endif

/*!
 * \brief First AT25DF block I/O control implementation structure.
 */
static NUTBLOCKIO blkIoAt25df0 = {
    NULL,                       /*!< \brief Device specific parameters, blkio_info. */
    0,                          /*!< \brief Total number of pages, blkio_blk_cnt. */
    0,                          /*!< \brief Number of bytes per page, blkio_blk_siz. */
    BLKIO_MOUNT_OFFSET_AT25DF0, /*!< \brief Number of sectors reserved at bottom, blkio_vol_bot. */
    BLKIO_MOUNT_TOP_RESERVE_AT25DF0,     /*!< \brief Number of sectors reserved at top, blkio_vol_top. */
    At25dfBlkIoRead,            /*!< \brief Read from node, blkio_read. */
    At25dfBlkIoWrite,           /*!< \brief Write to node, blkio_write. */
#ifdef __HARVARD_ARCH__
    At25dfBlkIoWrite_P,         /*!< \brief Write program memory to node, blkio_write_P. */
#endif
    At25dfBlkIoCtl              /*!< \brief Control functions, blkio_ioctl. */
};

/*!
 * \brief First AT25DF block I/O device implementation structure.
 */
NUTDEVICE devSpiBlkAt25df0 = {
    NULL,                       /*!< \brief Pointer to next device, dev_next. */
    {'A', 'T', '2', '5', 'D', 'F', '0', 0, 0},    /*!< \brief Unique device name, dev_name. */
    IFTYP_BLKIO | IF_LAYER_SPI, /*!< \brief Type of device, dev_type. */
    0,                          /*!< \brief Base address, dev_base (not used). */
    0,                          /*!< \brief First interrupt number, dev_irq (not used). */
    &nodeAt25df0,               /*!< \brief Interface control block, dev_icb. */
    &blkIoAt25df0,              /*!< \brief Driver control block, dev_dcb. */
    At25dfBlkIoInit,            /*!< \brief Driver initialization routine, dev_init. */
    NutBlockDeviceIOCtl,        /*!< \brief Driver specific control function, dev_ioctl. */
    NutBlockDeviceRead,         /*!< \brief Read from device, dev_read. */
    NutBlockDeviceWrite,        /*!< \brief Write to device, dev_write. */
#ifdef __HARVARD_ARCH__
    NutBlockDeviceWrite_P,      /*!< \brief Write data from program space to device, dev_write_P. */
#endif
    NutBlockDeviceOpen,         /*!< \brief Mount volume, dev_open. */
    NutBlockDeviceClose,        /*!< \brief Unmount volume, dev_close. */
    NutBlockDeviceSize,         /*!< \brief Request file size, dev_size. */
    NULL,                       /*!< \brief Select function, optional, not yet implemented */
};

#ifndef BLKIO_MOUNT_OFFSET_AT25DF1
#ifdef MOUNT_OFFSET_AT25DF1
#define BLKIO_MOUNT_OFFSET_AT25DF1       MOUNT_OFFSET_AT25DF1
#else
#define BLKIO_MOUNT_OFFSET_AT25DF1       0
#endif
#endif

#ifndef BLKIO_MOUNT_TOP_RESERVE_AT25DF1
#ifdef MOUNT_TOP_RESERVE_AT25DF1
#define BLKIO_MOUNT_TOP_RESERVE_AT25DF1  MOUNT_TOP_RESERVE_AT25DF1
#else
#define BLKIO_MOUNT_TOP_RESERVE_AT25DF1  1
#endif
#endif

/*!
 * \brief Second AT25DF block I/O control implementation structure.
 */
static NUTBLOCKIO blkIoAt25df1 = {
    NULL,                       /*!< \brief Device specific parameters, blkio_info. */
    0,                          /*!< \brief Total number of pages, blkio_blk_cnt. */
    0,                          /*!< \brief Number of bytes per page, blkio_blk_siz. */
    BLKIO_MOUNT_OFFSET_AT25DF1,  /*!< \brief Number of sectors reserved at bottom, blkio_vol_bot. */
    BLKIO_MOUNT_TOP_RESERVE_AT25DF1,     /*!< \brief Number of sectors reserved at top, blkio_vol_top. */
    At25dfBlkIoRead,            /*!< \brief Read from node, blkio_read. */
    At25dfBlkIoWrite,           /*!< \brief Write to node, blkio_write. */
#ifdef __HARVARD_ARCH__
    At25dfBlkIoWrite_P,         /*!< \brief Write program memory to node, blkio_write_P. */
#endif
    At25dfBlkIoCtl              /*!< \brief Control functions, blkio_ioctl. */
};

/*!
 * \brief Second AT25DF block I/O device implementation structure.
 */
NUTDEVICE devSpiBlkAt25df1 = {
    NULL,                       /*!< \brief Pointer to next device, dev_next. */
    {'A', 'T', '2', '5', 'D', 'F', '1', 0, 0},    /*!< \brief Unique device name, dev_name. */
    IFTYP_BLKIO | IF_LAYER_SPI, /*!< \brief Type of device, dev_type. */
    0,                          /*!< \brief Base address, dev_base (not used). */
    0,                          /*!< \brief First interrupt number, dev_irq (not used). */
    &nodeAt25df1,               /*!< \brief Interface control block, dev_icb. */
    &blkIoAt25df1,              /*!< \brief Driver control block, dev_dcb. */
    At25dfBlkIoInit,            /*!< \brief Driver initialization routine, dev_init. */
    NutBlockDeviceIOCtl,        /*!< \brief Driver specific control function, dev_ioctl. */
    NutBlockDeviceRead,         /*!< \brief Read from device, dev_read. */
    NutBlockDeviceWrite,        /*!< \brief Write to device, dev_write. */
#ifdef __HARVARD_ARCH__
    NutBlockDeviceWrite_P,      /*!< \brief Write data from program space to device, dev_write_P. */
#endif
    NutBlockDeviceOpen,         /*!< \brief Mount volume, dev_open. */
    NutBlockDeviceClose,        /*!< \brief Unmount volume, dev_close. */
    NutBlockDeviceSize,         /*!< \brief Request file size, dev_size. */
    NULL,                       /*!< \brief Select function, optional, not yet implemented */
};

#ifndef BLKIO_MOUNT_OFFSET_AT25DF2
#ifdef MOUNT_OFFSET_AT25DF2
#define BLKIO_MOUNT_OFFSET_AT25DF2       MOUNT_OFFSET_AT25DF2
#else
#define BLKIO_MOUNT_OFFSET_AT25DF2       0
#endif
#endif

#ifndef BLKIO_MOUNT_TOP_RESERVE_AT25DF2
#ifdef MOUNT_TOP_RESERVE_AT25DF2
#define BLKIO_MOUNT_TOP_RESERVE_AT25DF2  MOUNT_TOP_RESERVE_AT25DF2
#else
#define BLKIO_MOUNT_TOP_RESERVE_AT25DF2  1
#endif
#endif

/*!
 * \brief Third AT25DF block I/O control implementation structure.
 */
static NUTBLOCKIO blkIoAt25df2 = {
    NULL,                       /*!< \brief Device specific parameters, blkio_info. */
    0,                          /*!< \brief Total number of pages, blkio_blk_cnt. */
    0,                          /*!< \brief Number of bytes per page, blkio_blk_siz. */
    BLKIO_MOUNT_OFFSET_AT25DF2, /*!< \brief Number of sectors reserved at bottom, blkio_vol_bot. */
    BLKIO_MOUNT_TOP_RESERVE_AT25DF2,     /*!< \brief Number of sectors reserved at top, blkio_vol_top. */
    At25dfBlkIoRead,            /*!< \brief Read from node, blkio_read. */
    At25dfBlkIoWrite,           /*!< \brief Write to node, blkio_write. */
#ifdef __HARVARD_ARCH__
    At25dfBlkIoWrite_P,         /*!< \brief Write program memory to node, blkio_write_P. */
#endif
    At25dfBlkIoCtl              /*!< \brief Control functions, blkio_ioctl. */
};

/*!
 * \brief Third AT25DF block I/O device implementation structure.
 */
NUTDEVICE devSpiBlkAt25df2 = {
    NULL,                       /*!< \brief Pointer to next device, dev_next. */
    {'A', 'T', '2', '5', 'D', 'F', '2', 0, 0},    /*!< \brief Unique device name, dev_name. */
    IFTYP_BLKIO | IF_LAYER_SPI, /*!< \brief Type of device, dev_type. */
    0,                          /*!< \brief Base address, dev_base (not used). */
    0,                          /*!< \brief First interrupt number, dev_irq (not used). */
    &nodeAt25df2,               /*!< \brief Interface control block, dev_icb. */
    &blkIoAt25df2,              /*!< \brief Driver control block, dev_dcb. */
    At25dfBlkIoInit,            /*!< \brief Driver initialization routine, dev_init. */
    NutBlockDeviceIOCtl,        /*!< \brief Driver specific control function, dev_ioctl. */
    NutBlockDeviceRead,         /*!< \brief Read from device, dev_read. */
    NutBlockDeviceWrite,        /*!< \brief Write to device, dev_write. */
#ifdef __HARVARD_ARCH__
    NutBlockDeviceWrite_P,      /*!< \brief Write data from program space to device, dev_write_P. */
#endif
    NutBlockDeviceOpen,         /*!< \brief Mount volume, dev_open. */
    NutBlockDeviceClose,        /*!< \brief Unmount volume, dev_close. */
    NutBlockDeviceSize,         /*!< \brief Request file size, dev_size. */
    NULL,                       /*!< \brief Select function, optional, not yet implemented */
};

#ifndef BLKIO_MOUNT_OFFSET_AT25DF3
#ifdef MOUNT_OFFSET_AT25DF3
#define BLKIO_MOUNT_OFFSET_AT25DF3       MOUNT_OFFSET_AT25DF3
#else
#define BLKIO_MOUNT_OFFSET_AT25DF3       0
#endif
#endif

#ifndef BLKIO_MOUNT_TOP_RESERVE_AT25DF3
#ifdef MOUNT_TOP_RESERVE_AT25DF3
#define BLKIO_MOUNT_TOP_RESERVE_AT25DF3  MOUNT_TOP_RESERVE_AT25DF3
#else
#define BLKIO_MOUNT_TOP_RESERVE_AT25DF3  1
#endif
#endif

/*!
 * \brief Forth AT25DF block I/O control implementation structure.
 */
static NUTBLOCKIO blkIoAt25df3 = {
    NULL,                       /*!< \brief Device specific parameters, blkio_info. */
    0,                          /*!< \brief Total number of pages, blkio_blk_cnt. */
    0,                          /*!< \brief Number of bytes per page, blkio_blk_siz. */
    BLKIO_MOUNT_OFFSET_AT25DF3,  /*!< \brief Number of sectors reserved at bottom, blkio_vol_bot. */
    BLKIO_MOUNT_TOP_RESERVE_AT25DF3,     /*!< \brief Number of sectors reserved at top, blkio_vol_top. */
    At25dfBlkIoRead,            /*!< \brief Read from node, blkio_read. */
    At25dfBlkIoWrite,           /*!< \brief Write to node, blkio_write. */
#ifdef __HARVARD_ARCH__
    At25dfBlkIoWrite_P,         /*!< \brief Write program memory to node, blkio_write_P. */
#endif
    At25dfBlkIoCtl              /*!< \brief Control functions, blkio_ioctl. */
};

/*!
 * \brief Forth AT25DF block I/O device implementation structure.
 */
NUTDEVICE devSpiBlkAt25df3 = {
    NULL,                       /*!< \brief Pointer to next device, dev_next. */
    {'A', 'T', '2', '5', 'D', 'F', '3', 0, 0},    /*!< \brief Unique device name, dev_name. */
    IFTYP_BLKIO | IF_LAYER_SPI, /*!< \brief Type of device, dev_type. */
    0,                          /*!< \brief Base address, dev_base (not used). */
    0,                          /*!< \brief First interrupt number, dev_irq (not used). */
    &nodeAt25df3,               /*!< \brief Interface control block, dev_icb. */
    &blkIoAt25df3,              /*!< \brief Driver control block, dev_dcb. */
    At25dfBlkIoInit,            /*!< \brief Driver initialization routine, dev_init. */
    NutBlockDeviceIOCtl,        /*!< \brief Driver specific control function, dev_ioctl. */
    NutBlockDeviceRead,         /*!< \brief Read from device, dev_read. */
    NutBlockDeviceWrite,        /*!< \brief Write to device, dev_write. */
#ifdef __HARVARD_ARCH__
    NutBlockDeviceWrite_P,      /*!< \brief Write data from program space to device, dev_write_P. */
#endif
    NutBlockDeviceOpen,         /*!< \brief Mount volume, dev_open. */
    NutBlockDeviceClose,        /*!< \brief Unmount volume, dev_close. */
    NutBlockDeviceSize,         /*!< \brief Request file size, dev_size. */
    NULL,                       /*!< \brief Select function, optional, not yet implemented */
};

/*@}*/
