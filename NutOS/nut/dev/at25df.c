/*
 * Copyright (C) 2006 by egnite Software GmbH. All rights reserved.
 * Copyright (C) 2008 by egnite GmbH. All rights reserved.
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
 *
 */

/*!
 * \file dev/at25db.c
 * \brief Routines for Adesto/Atmel AT25 serial dataflash memory chips.
 *
 * \verbatim
 *
 * $Log$
 * Revision 1.9  2009/02/06 15:53:42  haraldkipp
 * Corrected a bug with non-negated chip selects.
 *
 * Revision 1.8  2009/01/17 11:26:46  haraldkipp
 * Getting rid of two remaining BSD types in favor of stdint.
 * Replaced 'u_int' by 'unsinged int' and 'uptr_t' by 'uintptr_t'.
 *
 * Revision 1.7  2008/12/15 19:18:49  haraldkipp
 * Enable DataFlash support for EIR board.
 *
 * Revision 1.6  2008/08/11 06:59:41  haraldkipp
 * BSD types replaced by stdint types (feature request #1282721).
 *
 * Revision 1.5  2008/08/06 12:51:09  haraldkipp
 * Added support for Ethernut 5 (AT91SAM9XE reference design).
 *
 * Revision 1.4  2008/02/15 17:10:43  haraldkipp
 * At25dbPageErase selected the wrong bank. Fixed. Parameter pgn (page number)
 * of At25dbPageWrite() changed from unsigned int to unsigned long.
 * New routines At25dbPages() and At25dbPageSize() allow to determine the
 * chip's layout.
 *
 * Revision 1.3  2007/08/17 10:45:21  haraldkipp
 * Enhanced documentation.
 *
 * Revision 1.2  2006/10/08 16:48:09  haraldkipp
 * Documentation fixed
 *
 * Revision 1.1  2006/09/29 12:41:55  haraldkipp
 * Added support for AT25 serial DataFlash memory chips. Currently limited
 * to AT91 builds.
 *
 *
 * \endverbatim
 */

#include <cfg/os.h>
#include <cfg/memory.h>

#include <sys/timer.h>

#include <string.h>
#include <stdlib.h>

//#include <dev/at91_spi.h>

#include <dev/at25df.h>
#include <dev/blockdev.h>
#include <dev/spibus.h>
#include <sys/nutdebug.h>

#define MAX_AT25_CMDLEN         6
#define AT25_ERASE_WAIT         3000
#define AT25_CHIP_ERASE_WAIT    50000
#define AT25_WRITE_POLLS        1000

/*!
 * \name AT25 DataFlash Commands
 */
/*@{*/
/*! \brief Continuos read (high frequency).
 *
 * Reads a continous stream in high speed mode.
 */
#define DFCMD_READ_PAGE      0x0B
/*! \brief Block erase 4k.
 */
#define DFCMD_BLOCK_ERASE_4K    0x20
/*! \brief Block erase 32k.
 */
#define DFCMD_BLOCK_ERASE_32K   0x52
/*! \brief Block erase 64k.
 */
#define DFCMD_BLOCK_ERASE_64K   0xd8
/*! \brief Chip erase
 */
#define DFCMD_CHIP_ERASE    0xC7
/*! \brief Write bytes/page.
 */
#define DFCMD_WRITE         0x02
/*! \brief Read status register.
 */
#define DFCMD_READ_STATUS       0x05
#define DFCMD_READ_DEVICEID 0x9F
#define DFCMD_WRITE_ENABLE  0x06
#define DFCMD_WRITE_DISABLE 0x04
/*@}*/

#define MOUNT_OFFSET_AT45D0   0
#define MOUNT_TOP_RESERVE_AT45D0 0


/*! \brief Parameter table of known DataFlash types. */
AT25D_INFO at25d_info[] = {
    {12, 2048, 4096, 0x48}, /* AT25DF641 - 8MB */
};

/*! \brief Number of known Dataflash types. */
uint_fast8_t at25d_known_types = sizeof(at25d_info) / sizeof(AT25D_INFO);

/*!
 * \brief Send DataFlash command.
 *
 * \param node  Specifies the SPI node.
 * \param op    Command operation code.
 * \param parm  Optional command parameter.
 * \param oplen Command length.
 * \param txbuf Pointer to the transmit data buffer, may be set to NULL.
 * \param rxbuf Pointer to the receive data buffer, may be set to NULL.
 * \param xlen  Number of byte to receive and/or transmit.
 */
static int At25dCommand(NUTSPINODE * node, uint8_t op, uint32_t parm, int oplen, const void *txbuf, void *rxbuf, int xlen)
{
    int rc = -1;
    NUTSPIBUS *bus;
    uint8_t cmd[8];

    NUTASSERT(node != NULL);
    bus = (NUTSPIBUS *) node->node_bus;
    NUTASSERT(bus != NULL);
    NUTASSERT(bus->bus_alloc != NULL);
    NUTASSERT(bus->bus_transfer != NULL);
    NUTASSERT(bus->bus_release != NULL);

    NUTASSERT(oplen <= 8);
    memset(cmd, 0, oplen);
    cmd[0] = op;
    if (parm) {
        cmd[1] = (uint8_t) (parm >> 16);
        cmd[2] = (uint8_t) (parm >> 8);
        cmd[3] = (uint8_t) parm;
    }
    rc = (*bus->bus_alloc) (node, 1000);
    if (rc == 0) {
        rc = (*bus->bus_transfer) (node, cmd, NULL, oplen);
        if (rc == 0 && xlen) {
            rc = (*bus->bus_transfer) (node, txbuf, rxbuf, xlen);
        }
        (*bus->bus_release) (node);
    }
    return rc;
};

/*!
 * \brief Query the device ID of the DataFlash.
 *
 * \param node  Specifies the SPI node.
 *
 * \return 0 on success or -1 in case of an error.
 */
static uint8_t At25dDeviceID(NUTSPINODE * node)
{
    int rc;
    uint8_t cmd[5] = { DFCMD_READ_DEVICEID, 0xFF, 0xFF, 0xFF, 0xFF };
    NUTSPIBUS *bus;

    NUTASSERT(node != NULL);
    NUTASSERT(node->node_bus != NULL);
    bus = (NUTSPIBUS *) node->node_bus;

    NUTASSERT(bus->bus_alloc != NULL);
    NUTASSERT(bus->bus_transfer != NULL);
    NUTASSERT(bus->bus_wait != NULL);
    NUTASSERT(bus->bus_release != NULL);

    rc = (*bus->bus_alloc) (node, 1000);
    if (rc == 0) {
        rc = (*bus->bus_transfer) (node, cmd, cmd, 3);
        if (rc == 0) {
            (*bus->bus_wait) (node, NUT_WAIT_INFINITE);
            rc = cmd[2];
        }
        (*bus->bus_release) (node);
    }
    return (uint8_t) rc;
}

/*!
 * \brief Wait until DataFlash memory cycle finished.
 *
 * \param node  Specifies the SPI node.
 *
 * \return 0 on success or -1 in case of an error.
 */
static int At25dWaitReady(NUTSPINODE * node, uint32_t tmo, int poll)
{
    uint8_t sr;

    while (((sr = At25dStatus(node)) & 0x01) == 0) {
        if (!poll) {
            NutSleep(1);
        }
        if (tmo-- == 0) {
            return -1;
        }
    }
    return 0;
}



/*!
 * \brief Query the status of the DataFlash.
 *
 * \param node  Specifies the SPI node.
 *
 * \return 0 on success or -1 in case of an error.
 */
static uint8_t At25dStatus(NUTSPINODE * node)
{
    int rc;
    uint8_t cmd[3] = { DFCMD_READ_STATUS, 0xFF, 0xFF };
    NUTSPIBUS *bus;

    NUTASSERT(node != NULL);
    NUTASSERT(node->node_bus != NULL);
    bus = (NUTSPIBUS *) node->node_bus;

    NUTASSERT(bus->bus_alloc != NULL);
    NUTASSERT(bus->bus_transfer != NULL);
    NUTASSERT(bus->bus_wait != NULL);
    NUTASSERT(bus->bus_release != NULL);

    rc = (*bus->bus_alloc) (node, 1000);
    if (rc == 0) {
        rc = (*bus->bus_transfer) (node, cmd, cmd, 3);
        if (rc == 0) {
            (*bus->bus_wait) (node, NUT_WAIT_INFINITE);
            rc = cmd[1];
        }
        (*bus->bus_release) (node);
    }
    return (uint8_t) rc;
}


/*!
 * \brief Initialize dataflash at specified interface and chip select.
 *
 * \param spibas Interface base address. For ARM MCUs this may be the
 *               I/O base address of the hardware SPI.
 * \param spipcs Device chip select.
 *
 * \return Device descriptor or -1 in case of an error.
 */
int At25dfInit(NUTDEVICE* dev)
{
    NUTBLOCKIO *blkio;
    NUTSPINODE *node;
    uint8_t sr;
    int_fast8_t i;

    NUTASSERT(dev != NULL);
    NUTASSERT(dev->dev_dcb != NULL);
    NUTASSERT(dev->dev_icb != NULL);
    blkio = dev->dev_dcb;
    node = dev->dev_icb;

    /* Read the status byte and locate the related table entry. */
    sr = At25dDeviceID(node);
//    sr &= AT45D_STATUS_DENSITY | AT45D_STATUS_PAGE_SIZE;
    for (i = at25d_known_types; --i >= 0;) {
        if (sr == at25d_info[i].at25d_srval) {
            /* Known DataFlash type. */
            blkio->blkio_info = &at25d_info[i];
            blkio->blkio_blk_cnt = at25d_info[i].at25d_pages;
            blkio->blkio_blk_siz = at25d_info[i].at25d_psize;
            return 0;
        }
    }
    /* Unknown DataFlash type. */
    return -1;

}

int SpiAt25PageRead (NUTDEVICE * dev, uint32_t pgn, void * data, int len){//Should work TODO: check
    NUTBLOCKIO *blkio;
    AT25D_INFO *info;

    NUTASSERT(dev != NULL);
    NUTASSERT(dev->dev_dcb != NULL);
    blkio = dev->dev_dcb;

    info = (AT25D_INFO *) blkio->blkio_info;
    NUTASSERT(blkio->blkio_info != NULL);

    if (pgn >= info->at25d_pages) {
        return -1;
    }
    pgn <<= info->at25d_pshft;

    if (At25dCommand((NUTSPINODE *) dev->dev_icb, DFCMD_READ_PAGE, pgn, 5, NULL, data, len)) {
        return -1;
    }
    return len;
};

int SpiAt25PageWrite (NUTDEVICE * dev, uint32_t pgn, const void *data, int len){//FIXME: fix for at25
    int rc = -1;
    uint8_t *dp = (uint8_t *) data;
    int step;
    uint_fast8_t pshft;
    uint32_t limit;
    int sector;
    NUTBLOCKIO *blkio;
    NUTSPINODE *node;

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
    pshft = ((AT25D_INFO *) blkio->blkio_info)->at25d_pshft;
    step = ((AT25D_INFO *) blkio->blkio_info)->at25d_psize;
    limit = ((AT25D_INFO *) blkio->blkio_info)->at25d_pages;
    sector = 256;

    while (len) {
        if (step > len) {
            step = len;
        }
    //TODO: write real code
    //1. Block erase 4k
    //2. write page
    //3. wait ready
    //4. repeat 2,3,4 up to 8 times

    //erase page
    if (At25dCommand(node, DFCMD_WRITE_ENABLE, 0, 1, NULL,NULL,0)){
        break;
    };
    if (At25dCommand(node, DFCMD_BLOCK_ERASE_4K,pgn << pshft, 4, NULL,NULL, 0)){
            break;
        }
        if (At25dWaitReady(node, AT25_WRITE_POLLS, 1)) {
            break;
        }
    while(len) {
        if(sector > len) {
            sector = len;
        };
        if (At25dCommand(node, DFCMD_WRITE_ENABLE, 0, 1, NULL,NULL,0)){
            break;
        };
        if (At25dCommand(node, DFCMD_WRITE, (pgn << pshft)+(uint8_t*)data-dp, 4, dp, NULL, step)){
            break;
        };
            if (At25dWaitReady(node, AT25_WRITE_POLLS, 1)) {
                break;
            }
            if (rc < 0) {
                rc = 0;
            }
            rc += sector;
            dp += sector;
            len -= sector;
            if (++pgn >= limit) {
                break;
            }
    }
    }
    return rc;
};

#ifdef __HARVARD_ARCH__
int SpiAt25PageWrite_P (NUTDEVICE * dev, uint32_t pgn, PGM_P data, int len){
    return -1;
}
#endif

int SpiAt25IOCtl (NUTDEVICE * dev, int req, void *conf){
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
};


NUTSPINODE at25df = {
    NULL,   /* SPI bus */
    NULL,   /* additional parameters (dcb) */
    1000000,/* SPI data rate TODO ? */
    0,      /* SPI mode */
    8,      /* data bits */
    0       /* chip select index */
};

/*!
 * \brief AT45D DataFlash block I/O implementation structure.
 */
static NUTBLOCKIO blkIoAt25df = {
    NULL,                      /*!< \brief Device specific parameters, blkio_info. */
    0,                         /*!< \brief Total number of pages, blkio_blk_cnt. */
    0,                         /*!< \brief Number of bytes per page, blkio_blk_siz. */
    MOUNT_OFFSET_AT45D0,       /*!< \brief Number of sectors reserved at bottom, blkio_vol_bot. */
    MOUNT_TOP_RESERVE_AT45D0,  /*!< \brief Number of sectors reserved at top, blkio_vol_top. */
    SpiAt25PageRead,           /*!< \brief Read from node, blkio_read. */
    SpiAt25PageWrite,          /*!< \brief Write to node, blkio_write. */
#ifdef __HARVARD_ARCH__
    SpiAt25PageWrite_P,        /*!< \brief Write program memory to node, blkio_write_P. */
#endif
    SpiAt25IOCtl               /*!< \brief Control functions, blkio_ioctl. */
};

NUTDEVICE devDataFlash0 = {
    0,                         /* Pointer to next device, dev_next. */
    {'a', 't', '2', '5', 'd', 'f', '0', 0, 0},    /* Unique device name, dev_name. */
    IFTYP_BLKIO,               /* Type of device, dev_type. */
    0,                         /* Codec number, dev_base. */
    0,                         /* First interrupt number, dev_irq (not used). */
    &at25df,                   /* Interface control block, dev_icb (not used). */
    &blkIoAt25df,              /* Driver control block, dev_dcb. */
    At25dfInit,                /* Driver initialization routine, dev_init. */
    NutBlockDeviceIOCtl,       /* Driver specific control function, dev_ioctl. */
    NutBlockDeviceRead,        /* Read from device, dev_read. */
    NutBlockDeviceWrite,       /* Write to device, dev_write. */
#ifdef __HARVARD_ARCH__
    NutBlockDeviceWrite_P,     /* Write data from program space to device, dev_write_P. */
#endif
    NutBlockDeviceOpen,        /* Open a device or file, dev_open. */
    NutBlockDeviceClose,       /* Close a device or file, dev_close. */
    NutBlockDeviceSize,        /* Request file size, dev_size. */
    NULL,                      /* Select function, optional, not yet implemented */
};
