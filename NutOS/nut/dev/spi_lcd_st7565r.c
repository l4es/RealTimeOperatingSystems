/*
 * Copyright (C) 2013 Ole Reinhardt <ole.reinhardt@embedded-it.de>
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
 *
 */

#include <stdlib.h>
#include <string.h>
#include <dev/gpio.h>
#include <sys/timer.h>
#include <sys/heap.h>
#include <sys/nutdebug.h>

#include <dev/spi_lcd_st7565r.h>
#include <dev/framebuffer.h>


/*!
 * \file dev/lcd_st7565r.c
 *
 * Driver for ST7565R LCD controller which provides functionality to
 * initialize the display and write data and commands to the LCD controller.
 *
 */

#ifndef ST7565R_SPI_MODE
#define ST7565R_SPI_MODE     SPI_MODE_3
#endif

#ifndef ST7565R_SPI_RATE
#define ST7565R_SPI_RATE     20000000
#endif

#define ST17565R_DEFAULT_WIDTH 128
#define ST17565R_DEFAULT_HEIGHT 32

#ifndef ST17565R_0_WIDTH
#define ST17565R_0_WIDTH ST17565R_DEFAULT_WIDTH
#endif

#ifndef ST17565R_0_HEIGHT
#define ST17565R_0_HEIGHT ST17565R_DEFAULT_HEIGHT
#endif

#ifndef ST17565R_1_WIDTH
#define ST17565R_1_WIDTH ST17565R_DEFAULT_WIDTH
#endif

#ifndef ST17565R_1_HEIGHT
#define ST17565R_1_HEIGHT ST17565R_DEFAULT_HEIGHT
#endif

#define min(a, b) ((a) < (b) ? (a) : (b))

/*!
 * \brief Lock LCD controller SPI node from concurrent access
 *
 * \param dev  		Specifies LCD device
 *
 */
int St7565rNodeLock(NUTDEVICE * dev)
{
	NUTSPINODE  *node;
	ST7565R_DCB *dcb;

	NUTASSERT (dev != NULL && dev->dev_icb != NULL);

	node = dev->dev_icb;

	NUTASSERT(node->node_dcb != NULL);

    dcb = (ST7565R_DCB *) node->node_dcb;

    return NutEventWait(&dcb->dcb_lock, NUT_WAIT_INFINITE);
}

/*!
 * \brief Unlock LCD controller SPI node from concurrent access
 *
 * \param dev  		Specifies LCD device
 *
 */
void St7565rNodeUnlock(NUTDEVICE * dev)
{
	NUTSPINODE  *node;
	ST7565R_DCB *dcb;

	NUTASSERT (dev != NULL && dev->dev_icb != NULL);

	node = dev->dev_icb;

	NUTASSERT(node->node_dcb != NULL);

    dcb = (ST7565R_DCB *) node->node_dcb;

    NutEventPost(&dcb->dcb_lock);
}

/*!
 * \brief Send a command to the display controller
 *
 * Pull A0 low before writing to the controller.
 *
 * \param node  Specifies the SPI node.
 * \param cmd   Command code.
 *
 * \return 0 on success, -1 on errors.
 */
static inline int St7565rWriteCmd(NUTSPINODE * node, uint8_t cmd)
{
	ST7565R_DCB *dcb = (ST7565R_DCB *) node->node_dcb;
	NUTSPIBUS   *bus = (NUTSPIBUS *) node->node_bus;
	int rc;

	GpioPinSetLow(dcb->a0_port, dcb->a0_pin);

	rc = (*bus->bus_alloc) (node, 0);
    if (rc == 0) {
        rc = (*bus->bus_transfer) (node, &cmd, NULL, 1);
        if (rc == 0) {
            (*bus->bus_wait) (node, NUT_WAIT_INFINITE);
        }
        (*bus->bus_release) (node);
    }

    return rc;
}

/*!
 * \brief Write data to the display controller
 *
 * Pull A0 high before writing to the controller.
 *
 * \param node  Specifies the SPI node.
 * \param data  data buffer
 * \param size  Number of bytes to write
 *
 * \return 0 on success, -1 on errors.
 */
static inline int St7565rWriteData(NUTSPINODE * node, uint8_t *data, size_t size)
{
	ST7565R_DCB *dcb = (ST7565R_DCB *) node->node_dcb;
	NUTSPIBUS   *bus = (NUTSPIBUS *) node->node_bus;
	int rc;

	GpioPinSetHigh(dcb->a0_port, dcb->a0_pin);

	rc = (*bus->bus_alloc) (node, 0);
    if (rc == 0) {
        rc = (*bus->bus_transfer) (node,data, NULL, size);
        if (rc == 0) {
            (*bus->bus_wait) (node, NUT_WAIT_INFINITE);
        }
        (*bus->bus_release) (node);
    }

	GpioPinSetLow(dcb->a0_port, dcb->a0_pin);
	return rc;
}

/*!
 * \brief Reset the LCD controller.
 *
 * Hard- or soft reset can be selected:
 *    hard: Hard reset the LCD controller by pulling the reset line low
 *    soft: Send soft reset command to the LCD controller
 *
 * \param node		Specifies the SPI node.
 * \param hard  	1: hard reset, 0: soft reset
 */
static int St7565rReset(NUTSPINODE * node, int hard)
{
	ST7565R_DCB *dcb = (ST7565R_DCB *) node->node_dcb;
	if (hard) {
        GpioPinSetHigh(dcb->reset_port, dcb->reset_pin);
        NutMicroDelay(10);
		GpioPinSetLow(dcb->reset_port, dcb->reset_pin);
		NutMicroDelay(10);
		GpioPinSetHigh(dcb->reset_port, dcb->reset_pin);
		NutMicroDelay(10);
		return 0;
	} else {
		return St7565rWriteCmd(node, ST7565R_CMD_RESET);
	}
}

/*!
 * \brief Enable / Disable the LCD sleep mode
 *
 * \param dev  		Specifies LCD device
 * \param sleep		1: enable sleep mode, 0: enter normal operation mode
 *
 * \return 0 on success, -1 on errors.
 */
int St7565rSetMode(NUTDEVICE * dev, int sleep)
{
	NUTSPINODE  *node;

	NUTASSERT (dev != NULL && dev->dev_icb != NULL);

	node = dev->dev_icb;

	return St7565rWriteCmd(node, sleep ? ST7565R_CMD_SLEEP_MODE : ST7565R_CMD_NORMAL_MODE);
}

/*!
 * \brief Set page in display RAM
 *
 * Next the column address should be set.
 *
 * \param dev  		Specifies LCD device
 * \param address	page address (range: 0..8)
 *
 * \return 0 on success, -1 on errors.
 */
int St7565rSetPageAddress(NUTDEVICE * dev, uint8_t address)
{
	NUTSPINODE  *node;

	NUTASSERT (dev != NULL && dev->dev_icb != NULL);

	node = dev->dev_icb;

	return St7565rWriteCmd(node, ST7565R_CMD_PAGE_ADDRESS_SET(address & 0x0F));
}

/*!
 * \brief Set column in display RAM
 *
 * \param dev  		Specifies LCD device
 * \param address	column address (range: 0..131)
 *
 * \return 0 on success, -1 on errors.
 */
int St7565rSetColAddress(NUTDEVICE * dev, uint8_t address)
{
	NUTSPINODE  *node;
	int rc;

	NUTASSERT (dev != NULL && dev->dev_icb != NULL);

	node = dev->dev_icb;

	rc  = St7565rWriteCmd(node, ST7565R_CMD_COLUMN_ADDRESS_SET_MSB(address >> 4));
	rc |= St7565rWriteCmd(node, ST7565R_CMD_COLUMN_ADDRESS_SET_LSB(address & 0x0F));
	return rc;
}


/*!
 * \brief Set display start draw line address
 *
 * Configure the line to start drawing on the LCD
 *
 * \param dev  		Specifies LCD device
 * \param address	display start line address (range: 0..63)
 *
 * \return 0 on success, -1 on errors.
 */
int St7565rSetDisplayStartLineAddress(NUTDEVICE * dev, uint8_t address)
{
	NUTSPINODE  *node;

	NUTASSERT (dev != NULL && dev->dev_icb != NULL);

	node = dev->dev_icb;

	return St7565rWriteCmd(node, ST7565R_CMD_START_LINE_SET(address & 0x3F));
}


/*!
 * \brief Turn on / off the display
 *
 * \param dev  		Specifies LCD device
 * \param enable	1: Turn on display, 0: turn off display
 *
 * \return 0 on success, -1 on errors.
 */
int St7565rDisplayEnable(NUTDEVICE * dev, int enable)
{
	NUTSPINODE  *node;

	NUTASSERT (dev != NULL && dev->dev_icb != NULL);

	node = dev->dev_icb;

	return St7565rWriteCmd(node, enable ? ST7565R_CMD_DISPLAY_ON : ST7565R_CMD_DISPLAY_OFF);
}


/*!
 * \brief Set LCD contrast
 *
 * The contrast value will be set by modifying the LCD voltage. Out of range
 * settings might damage the LCD, therefor the contrast value is clamped to the
 * configured contrast range. (\ref ST7565R_DISPLAY_CONTRAST_MAX ... \ref ST7565R_DISPLAY_CONTRAST_MIN.)
 *
 * \param dev  		Specifies LCD device
 * \param contrast  a number between 0 and 63 where the max values is given by
 *                  the LCD.
 *
 * \return 0 on success, -1 on errors.
 */
int St7565rSetContrast(NUTDEVICE * dev, uint8_t contrast)
{
	NUTSPINODE  *node;
	int rc;

	NUTASSERT (dev != NULL && dev->dev_icb != NULL);

	node = dev->dev_icb;

	if (contrast < ST7565R_CONTRAST_MIN) {
		contrast = ST7565R_CONTRAST_MIN;
	}
	if (contrast > ST7565R_CONTRAST_MAX) {
		contrast = ST7565R_CONTRAST_MAX;
	}
	rc  = St7565rWriteCmd(node, ST7565R_CMD_ELECTRONIC_VOLUME_MODE_SET);
	rc |= St7565rWriteCmd(node, ST7565R_CMD_ELECTRONIC_VOLUME(contrast));
	return rc;
}

/*!
 * \brief Invert LCD
 *
 * \param dev  		Specifies LCD device
 * \param enable	1: invert, 0: normal mode
 *
 * \return 0 on success, -1 on errors.
 */
int St7565rDisplayInvert(NUTDEVICE * dev, int invert)
{
	NUTSPINODE  *node;

	NUTASSERT (dev != NULL && dev->dev_icb != NULL);

	node = dev->dev_icb;

	return St7565rWriteCmd(node, invert ? ST7565R_CMD_DISPLAY_REVERSE : ST7565R_CMD_DISPLAY_NORMAL);
}

/*!
 * \brief Set all pixels for debugging purposes
 *
 * Turn on all pixels for debugging purposes. This will not affect the LCD RAM
 *
 * \param dev  		Specifies LCD device
 * \param all_on	1: All pixels on, 0: normal mode, show content of the LCD RAM
 *
 * \return 0 on success, -1 on errors.
 */
int St7565rDebugPixelsAllOn(NUTDEVICE * dev, int all_on)
{
	NUTSPINODE  *node;

	NUTASSERT (dev != NULL && dev->dev_icb != NULL);

	node = dev->dev_icb;

	return St7565rWriteCmd(node, all_on ? ST7565R_CMD_DISPLAY_ALL_POINTS_ON : ST7565R_CMD_DISPLAY_ALL_POINTS_OFF);
}

/*!
 * \brief Copy RAM Framebuffer to LCD
 *
 * \param dev  		Specifies LCD device
 *
 * \return 0 on success, -1 on errors.
 */
int St7565rUpdateFb(NUTDEVICE * dev)
{
	NUTSPINODE  *node;
	FBINFO  *dcb;
	size_t   fb_size;
	size_t   offset;
	int      page;
	int rc = 0;

	NUTASSERT (dev != NULL && dev->dev_icb != NULL);

	node = dev->dev_icb;
	dcb  = (FBINFO *) dev->dev_dcb;

	/* Calculate size of framebuffer in bytes. Each bit represents on pixel */
	fb_size = dcb->width * dcb->height / 8;

	for (offset = 0, page = 0; offset < fb_size; offset += 128, page ++) {
		rc |= St7565rWriteCmd(node, ST7565R_CMD_PAGE_ADDRESS_SET(page));
		rc |= St7565rWriteCmd(node, ST7565R_CMD_COLUMN_ADDRESS_SET_MSB(0));
		rc |= St7565rWriteCmd(node, ST7565R_CMD_COLUMN_ADDRESS_SET_LSB(0));

		rc |= St7565rWriteData(node, dcb->fb + offset, min(128, fb_size - offset));
	}
	return rc;
}

/*!
 * \brief Handle I/O controls for St7565 Framebuffer.
 *
 * \return 0.
 */
static int St7565rIOCtl(NUTDEVICE * dev, int req, void *conf)
{
	NUTASSERT (dev != NULL && dev->dev_icb != NULL);

    return 0;
}

/*!
 * \brief Write data to ST7565R framebuffer.
 *
 * Each write operation updates the LCD RAM with the shadow framebuffer
 *
 * \param  fp		Pointer to the NUTFILE struct
 * \param  buffer   Data to be written or NULL if the buffer shall be just
 *                  synced with the display. 
 * \param  len      Number of bytes to write
 *
 * \return Number of bytes written.
 */
static int St7565rWrite(NUTFILE * fp, const void *buffer, int len)
{
    FBINFO     *dcb;

	NUTASSERT(fp != NULL && fp != NUTFILE_EOF && fp->nf_dev != NULL);
    NUTASSERT(fp->nf_dev->dev_icb != NULL && fp->nf_dev->dev_dcb != NULL);

	dcb  = fp->nf_dev->dev_dcb;

	if (buffer == NULL) {
		/* Flush the LCD shadow buffer / sync with LCD */
		St7565rUpdateFb(fp->nf_dev);
		return 0;
	}

    len = min(len, dcb->width * dcb->height / 8);

	if (dcb->fb != NULL) {
        int i, bcount;
        int bytes_per_line = dcb->width / 8;
        uint8_t *fb = dcb->fb;
        uint8_t *src = (uint8_t*) buffer;
        uint8_t y0, y1;

        /* The ST7565r LCD controller has a really brain dead memory layout.
         * Each byte in the framebuffer represents 8 vertical pixels.
         * The framebuffer is organized in several banks. On bank represents
         * width * 8 vertical pixels. 
         *
         * The following loop re-orders a framebuffer content organized in the
         * "normal" layout (1 byte represents 8 horizontal pixels) to the LCD
         * controllers layout.
         */
        
        y1 =  0x01;
        y0 = ~y1;
        bcount = bytes_per_line;
        for (i = 0; i < len; i++) {
            bcount --;
            if (*src & 0x01) (*(fb++)) |= y1; else (*(fb++)) &= y0;
            if (*src & 0x02) (*(fb++)) |= y1; else (*(fb++)) &= y0;
            if (*src & 0x04) (*(fb++)) |= y1; else (*(fb++)) &= y0;
            if (*src & 0x08) (*(fb++)) |= y1; else (*(fb++)) &= y0;
            if (*src & 0x10) (*(fb++)) |= y1; else (*(fb++)) &= y0;
            if (*src & 0x20) (*(fb++)) |= y1; else (*(fb++)) &= y0;
            if (*src & 0x40) (*(fb++)) |= y1; else (*(fb++)) &= y0;
            if (*src & 0x80) (*(fb++)) |= y1; else (*(fb++)) &= y0;
            if (bcount == 0) {
                bcount = bytes_per_line;
                y1 =  y1 << 1;
                if (y1 == 0) {
                    y1 = 0x01;
                }
                y0 = ~y1;
                fb = &dcb->fb[((i+1) / dcb->width) * dcb->width];
            }
            src ++;
        }

		St7565rUpdateFb(fp->nf_dev);
	} else {
		return 0;
	}

	return len;
}

/*!
 * \brief Read data from ST7565R framebuffer.
 *
 * The Read operation does not sync the shadow framebuffer with the LCD RAM.
 * Read returns data from the shadow framebuffer.
 *
 * \param  fp		Pointer to the NUTFILE struct
 * \param  buffer   Pointer to the buffer that receives the data.
 * \param  len      Number of bytes to read
 *
 * \return Number of bytes read.
 */
static int St7565rRead(NUTFILE * fp, void *buffer, int len)
{
    FBINFO     *dcb;

	NUTASSERT(fp != NULL && fp != NUTFILE_EOF && fp->nf_dev != NULL);
    NUTASSERT(fp->nf_dev->dev_icb != NULL && fp->nf_dev->dev_dcb != NULL);

	dcb  = fp->nf_dev->dev_dcb;

	if ((dcb->fb != NULL) && (buffer != NULL)) {
        int i, bcount;
        int bytes_per_line = dcb->width / 8;
        uint8_t *fb = dcb->fb;
        uint8_t *dest = (uint8_t*) buffer;
        uint8_t y1;

        /* The ST7565r LCD controller has a really brain dead memory layout.
         * Each byte in the framebuffer represents 8 vertical pixels.
         * The framebuffer is organized in several banks. On bank represents
         * width * 8 vertical pixels. 
         *
         * The following loop re-orders the LCD framebuffer layout to the
         * "normal" layout (1 byte represents 8 horizontal pixels) to the LCD
         * controllers layout and fills the read buffer.
         */

        len = min(len, dcb->width * dcb->height / 8);
        
        y1 =  0x01;
        bcount = bytes_per_line;
        for (i = 0; i < len; i++) {
            bcount --;
            *dest = 0;
            if ((*fb++) & y1) *dest |= 0x01;
            if ((*fb++) & y1) *dest |= 0x02;
            if ((*fb++) & y1) *dest |= 0x04;
            if ((*fb++) & y1) *dest |= 0x08;
            if ((*fb++) & y1) *dest |= 0x10;
            if ((*fb++) & y1) *dest |= 0x20;
            if ((*fb++) & y1) *dest |= 0x40;
            if ((*fb++) & y1) *dest |= 0x80;
            
            if (bcount == 0) {
                bcount = bytes_per_line;
                y1 =  y1 << 1;
                if (y1 == 0) {
                    y1 = 0x01;
                }
                fb = &dcb->fb[((i+1) / dcb->width) * dcb->width];
            }
            dest ++;
        }
	} else {
		return 0;
	}

	return len;
}


/*!
 * \brief Generate File Handle for ST7565R framebuffer device.
 *
 * \param dev Specifies the ST7565R LCD controller device.
 *
 * \return 0 on success or -1 in case of an error.
 */
static NUTFILE *St7565rOpen(NUTDEVICE * dev, const char *name, int mode, int acc)
{
    NUTFILE *fp;

    NUTASSERT( dev != NULL);

    if ((fp = malloc(sizeof(NUTFILE))) == 0) {
        return NUTFILE_EOF;
    }

    fp->nf_fcb = 0;
    fp->nf_dev = dev;

    return fp;

}

/*!
 * \brief Close  ST7565R framebuffer device.
 *
 * \return 0 if closed and was opened before, else -1.
 */
static int St7565rClose(NUTFILE * fp)
{
    if( fp != NULL) {
        free( fp);
        return 0;
    }
    return -1;
}

/*!
 * \brief Initialize the LCD controller device
 *
 * Call this function to initialize the hardware interface and the LCD
 * controller. When initialization is done the display is turned on and ready
 * to receive data.
 *
 * Compared to other SPI bus node devices (and other Nut/OS devices in general)
 * some settings have to be configured by the user before registering the
 * SPI node. These are:
 *     - devSt7565rFb0.dev_dcb
 *     - devSt7565rFb.dev_icb->node_dcb
 *
 * This routine is internally called by Nut/OS during device registration.
 *
 * The driver framework may call this function more than once.
 *
 * \param dev Specifies the ST7565R LCD controller device.
 *
 * \return 0 on success or -1 in case of an error
 */
static int St7565rInit(NUTDEVICE * dev)
{
    NUTSPINODE  *node;
    FBINFO      *dcb;
	ST7565R_DCB *node_dcb;
	int     rc = 0;

    NUTASSERT(dev != NULL);
    NUTASSERT(dev->dev_icb != NULL);
	NUTASSERT(dev->dev_dcb != NULL);

	node = dev->dev_icb;
	node_dcb = (ST7565R_DCB *) node->node_dcb;
	node_dcb->dcb_lock = SIGNALED;

	dcb = dev->dev_dcb;

	if (dcb->fb != NULL) {
		/* The device is just initialised */
		return 0;
	}

	/* Allocate framebuffer memory, w*h / 8, as we have only 1 BPP */
	dcb->bpp = 1;
	dcb->fb  = malloc(dcb->width * dcb->height / 8);
	if (dcb->fb == NULL) {
		return -1;
	}
    memset(dcb->fb, 0, dcb->width * dcb->height / 8);

	GpioPinConfigSet(node_dcb->a0_port, node_dcb->a0_pin, GPIO_CFG_OUTPUT);
 	/* Configure A0 pin to command mode */
	GpioPinSetLow(node_dcb->a0_port, node_dcb->a0_pin);

	GpioPinConfigSet(node_dcb->reset_port, node_dcb->reset_pin, GPIO_CFG_OUTPUT);
    GpioPinSetHigh(node_dcb->reset_port, node_dcb->reset_pin);

 
	/* Apply hard reset of the LCD display controller */
	rc |= St7565rReset(node, 1);

	/* Turn on the display */
	rc |= St7565rDisplayEnable(dev, 0);    

	/* Set the voltage bias ratio to 1/6 */
	rc |= St7565rWriteCmd(node, ST7565R_CMD_LCD_BIAS_1_DIV_6_DUTY33);
    
	/* Set column address mode to increasing */
	rc |= St7565rWriteCmd(node, ST7565R_CMD_ADC_NORMAL);

    /* Reverse the common mode scan direction COM31->COM0 */
	rc |= St7565rWriteCmd(node, ST7565R_CMD_REVERSE_SCAN_DIRECTION);

	/* Set voltage resistor ratio to 2 */
	rc |= St7565rWriteCmd(node, ST7565R_CMD_VOLTAGE_RESISTOR_RATIO_2);
    
	/* Set booster circuit, voltage regulator and voltage follower all to on */
	rc |= St7565rWriteCmd(node, ST7565R_CMD_POWER_CTRL_ALL_ON);
    
	/* Set the booster ratio to 2X,3X,4X */
	rc |= St7565rWriteCmd(node, ST7565R_CMD_BOOSTER_RATIO_SET);
	rc |= St7565rWriteCmd(node, ST7565R_CMD_BOOSTER_RATIO_2X_3X_4X);

	/* Set contrast to min value */
	rc |= St7565rSetContrast(dev, ST7565R_CONTRAST_MAX);

    rc |= St7565rSetDisplayStartLineAddress(dev, 0);
    
	/* Non-inverted display */
	rc |= St7565rDisplayInvert(dev, 0);
    
	/* Turn on the display */
	rc |= St7565rDisplayEnable(dev, 1);

	if (rc != 0) {
		free(dcb->fb);
		return -1;
	}

	return 0;
}

/*!
 * \brief First ST7565R LCD Controller SPI node implementation structure.
 */
NUTSPINODE nodeSt7565r0 = {
    NULL,                       /*!< \brief Pointer to the bus controller driver, node_bus. */
    NULL,                       /*!< \brief Pointer to the bus device driver specific settings, node_stat. */
    ST7565R_SPI_RATE,           /*!< \brief Initial clock rate, node_rate. */
    ST7565R_SPI_MODE,           /*!< \brief Initial mode, node_mode. */
    8,                          /*!< \brief Initial data bits, node_bits. */
    0,                          /*!< \brief Chip select, node_cs. */
    NULL,                       /*!< \brief Pointer to our private device control block, node_dcb. */
};


/*!
 * \brief 7seg device implementation structure.
 */
NUTDEVICE devSt7565rFb0 = {
    NULL,                           /*!< \brief Pointer to next device, dev_next. */
    {'F', 'B', '0', 0, 0, 0, 0, 0, 0},  /*!< \brief Unique device name, dev_name. */
    IFTYP_FB | IF_LAYER_SPI,        /*!< \brief Type of device, dev_type. */
    0,                              /*!< \brief Base address, dev_base (not used). */
    0,                              /*!< \brief First interrupt number, dev_irq (not used). */
    &nodeSt7565r0,                  /*!< \brief Interface control block, dev_icb. */
    0,                              /*!< \brief Driver control block, dev_dcb. */
    St7565rInit,                    /*!< \brief Driver initialization routine, dev_init. */
    St7565rIOCtl,                   /*!< \brief Driver specific control function, dev_ioctl. */
    St7565rRead,                    /*!< \brief Read from device, dev_read. */
    St7565rWrite,                   /*!< \brief Write to device, dev_write. */
#ifdef __HARVARD_ARCH__
    0,                              /*!< \brief Write data from program space to device, dev_write_P. */
#endif
    St7565rOpen,                    /*!< \brief Mount volume, dev_open. */
    St7565rClose,                   /*!< \brief Unmount volume, dev_close. */
    0,                              /*!< \brief Request file size, dev_size. */
    0,                              /*!< \brief Select, currently not implemented, dev_select. */        
};
