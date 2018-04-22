/*
 * Copyright (C) 2015 by Uwe Bonnes bon@elektron.ikp.physik.tu-dadrmstadt.de
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
 * \file arch/cm3/stm/stm32_lcd16seg.c
 * \brief Routines for LCD connected to Stm32L
 *
 * \verbatim
 * $Id: stm32_lcd16seg.c,$
 * \endverbatim
 */

/* LCD
 *
 * This file is for LCD Displays with up to 16 segments
 *
 * Access layering
 *
 * LCD access with the STM32L LCD peripheral has three levels of
 * interchangability:
 * - Wiring between STM32 and the glass
 * - Display of segments on the glass caused by signals on the
 *   LCD-glass pins (segments and commons).
 * - The font for displaying characters2
 *
 * Wiring is defined in this file by connecting the STM segments/commons to the
 * LCD segments/commons like
 * #define LCDSEG05   9
 * Here LCD segment 5 is connected to STM32L segment 9. These definitions
 * are platform specific.
 *
 * LCD layout is defined in an include like "lcd_24x4.h". This include
 * is platform specific and provides a look-up table seg2offset.
 *
 */
#include <cfg/os.h>
#include <cfg/arch.h>
#include <compiler.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/nutdebug.h>
#include <sys/timer.h>
#include <sys/event.h>

#include <dev/gpio.h>
#include <dev/blockdev.h>

#include <arch/cm3/stm/stm32xxxx.h>
#include <arch/cm3/stm/stm32_gpio.h>
#include <arch/cm3/stm/stm32_clk.h>

#include <font/seg14.h>

/* Bitmask for driver control, as in dev/spi_7seg.c */
#define ICMD_UPDATE     0x01
#define ICMD_INTENS     0x02
#define ICMD_ESCAPE     0x80

#define SEG_DP          0x4000
#define SEG_COLON       0x8000

#if defined(MCU_STM32L0)
#define LCD_AF  1
#elif defined(MCU_STM32L1) || defined(MCU_STM32L4)
#define LCD_AF 11
#else
#define LCD_AF -1
#endif

#if PLATFORM == L1_DISCOVERY
/* An 24 segment, 4 common LCD display is connected on this board*/
const nutgpio_t lcd_pins[] = {
    PA01, PA02, PA03, PB03, PB04, PB05, PB10, PB11,
    PB12, PB13, PB14, PB15, PB09, PA10, PA09, PA08,
    PA15, PB08, PC00, PC01, PC02, PC03, PC06, PC07,
    PC08, PC09, PC10, PC11, PIN_NONE
};
/* Combined segments from PC10/PC11/PC12/PD02 are used! */
#define LCD_MUX   LCD_CR_MUX_SEG

/* Define interconnections between controller and LCD */
#define COM0 (0 * 64)
#define COM1 (1 * 64)
#define COM2 (2 * 64)
#define COM3 (3 * 64)

/* Define here the connection of the LCD glass segements to the
 * Stm32 Segment pins.
 * E.g.on the L1-Discovery, LCD Segment 3 is connected to PB3(LCD_SEG7).
 *
*/
#define LCDSEG00   0
#define LCDSEG01   1
#define LCDSEG02   2
#define LCDSEG03   7
#define LCDSEG04   8
#define LCDSEG05   9
#define LCDSEG06  10
#define LCDSEG07  11
#define LCDSEG08  12
#define LCDSEG09  13
#define LCDSEG10  14
#define LCDSEG11  15
#define LCDSEG12  17
#define LCDSEG13  16
#define LCDSEG14  18
#define LCDSEG15  19
#define LCDSEG16  20
#define LCDSEG17  21
#define LCDSEG18  24
#define LCDSEG19  25
#define LCDSEG20  26
#define LCDSEG21  27
#define LCDSEG22  28
#define LCDSEG23  29

#include <dev/lcd_24x4.h>

/* FIXME: Better configurability!
 *
 * Values here for 30.12 Hertz with LSE clocking the RTC module
 * =>   PS = 3, DIV = 1,
 * No blinking segment, BLINK = 0
 * Blink at 2 Hz => BLINKF = 1
 * Contrast: VLCD3 = 3.12 V,  CC = 4
 * No Dead time DEAD = 0
 * 4 clk_ps Pulse On => PON = 4
 */

/* Vendor headers misses LCD_FCR_PS_0 and LCD_FCR_DIV_0 ! */
#define LCD_FCR_PS_0 ((LCD_FCR_PS / 15) & LCD_FCR_PS)
#define LCD_FCR_DIV_0 ((LCD_FCR_DIV / 15) & LCD_FCR_DIV)

#define LCD_FCR ((3 * LCD_FCR_PS_0) | (1 * LCD_FCR_DIV_0) |             \
                 (0 * LCD_FCR_BLINK_0) | (1 *  LCD_FCR_BLINKF_0) |      \
                 (4 * LCD_FCR_CC_0) | (9 * LCD_FCR_DEAD_0) | \
                 (4 * LCD_FCR_PON_0))
#elif PLATFORM == L4_DISCOVERY
/* An 24 segment, 4 common LCD display is connected on this board*/
const nutgpio_t lcd_pins[] = {
    PA08, PA09, PA10, PB09, PA07, PC05, PB01, PB13,
    PB15, PD09, PD11, PD13, PD15, PC07, PA15, PB04,
    PB05, PC08, PC06, PD14, PD12, PD10, PD08, PB14,
    PB12, PB00, PC04, PA06, PC03, PIN_NONE
};
/* No combined segments are used! */
#define LCD_MUX   0
/* Define interconnections between controller and LCD */
#define COM0 (0 * 64)
#define COM1 (1 * 64)
#define COM2 (2 * 64)
#define COM3 (3 * 64)

#define LCDSEG00   4
#define LCDSEG01  23
#define LCDSEG02   6
#define LCDSEG03  13
#define LCDSEG04  15
#define LCDSEG05  29
#define LCDSEG06  31
#define LCDSEG07  33
#define LCDSEG08  35
#define LCDSEG09  25
#define LCDSEG10  17
#define LCDSEG11   8
#define LCDSEG12   9
#define LCDSEG13  26
#define LCDSEG14  24
#define LCDSEG15  34
#define LCDSEG16  32
#define LCDSEG17  30
#define LCDSEG18  28
#define LCDSEG19  14
#define LCDSEG20  12
#define LCDSEG21   5
#define LCDSEG22  22
#define LCDSEG23   3

#include <dev/lcd_24x4.h>

/* FIXME: Better configurability!
 *
 * Values here for 30.12 Hertz with LSE clocking the RTC module
 * =>   PS = 3, DIV = 1,
 * No blinking segment, BLINK = 0
 * Blink at 2 Hz => BLINKF = 1
 * Contrast: VLCD3 = 3.12 V,  CC = 4
 * No Dead time DEAD = 0
 * 4 clk_ps Pulse On => PON = 4
 */

/* Vendor headers misses LCD_FCR_PS_0 and LCD_FCR_DIV_0 ! */
#define LCD_FCR_PS_0 ((LCD_FCR_PS / 15) & LCD_FCR_PS)
#define LCD_FCR_DIV_0 ((LCD_FCR_DIV / 15) & LCD_FCR_DIV)

#define LCD_FCR ((3 * LCD_FCR_PS_0) | (1 * LCD_FCR_DIV_0) |             \
                 (0 * LCD_FCR_BLINK_0) | (1 *  LCD_FCR_BLINKF_0) |      \
                 (4 * LCD_FCR_CC_0) | (9 * LCD_FCR_DEAD_0) | \
                 (4 * LCD_FCR_PON_0))
#else
/* No LCD defined*/
static const nutgpio_t lcd_pins[] = {PIN_NONE};
static const uint16_t seg2offset[1] = {0};
#define LCD_MUX      0
#define LCD_DUTY     0
#define LCD_BIAS     0
#define LCD_FCR      0
#define SEG16_DIGITS 0

#endif
/*!
 * \brief Device Control Block for 16-Segment Display
 */
typedef struct {
    uint16_t digit[SEG16_DIGITS]; /*!< Buffer for Display content */
    HANDLE   update_done;         /*!<  Handle for update event */
    uint32_t last_update;         /*!< Timestamp of last update */
    uint16_t display_delay;       /*!< Delay in ms between updates */
    uint8_t  dip;                 /*!< Actual cursor position */
    uint8_t  icmd;                /*!< internal driver control register */
} DCB_16SEG;

#if defined(MCU_CM_NO_BITBAND)
static void Stm32Lcd16SegWriteRam(DCB_16SEG * dcb)
{
    int i;
    for (i = 0; i < 16 * SEG16_DIGITS; i++) {
        if ( dcb->digit[i / 16 ] & (1 << (i % 16))) {
            uint16_t ram_offset = seg2offset[i];
            LCD->RAM[ram_offset / 32] |= (1 << ram_offset % 32);
        }
    }
}
#else
static void Stm32Lcd16SegWriteRam(DCB_16SEG * dcb)
{
    int i;
    volatile uint32_t *digit_bb_base;
    volatile uint32_t *lcd_bb_base;

    digit_bb_base = CM3BB_BASE(dcb->digit);
    lcd_bb_base   = CM3BB_BASE(LCD->RAM);

    for (i = 0; i < 16 * SEG16_DIGITS; i++) {
        if (digit_bb_base[i]) {
            uint16_t ram_offset = seg2offset[i];
            lcd_bb_base[ram_offset] = 1;
        }
    }
}
#endif
/********************************************************************************
 **
 **
 */
static int Stm32Lcd16SegUpdate(DCB_16SEG * dcb)
{
    uint32_t delta, millis;

    /* Check if last display was visible for display_delay*/
    millis = NutGetMillis();
    delta = millis - dcb->last_update;
    if (delta <  dcb->display_delay) {
        NutSleep(dcb->display_delay - delta);
    }
    if (LCD->SR & LCD_SR_UDR) {
        /* Update still in progress */
        NutEventWait(&dcb->update_done, NUT_WAIT_INFINITE);
    }
    memset((void*)LCD->RAM, 0, 2 * sizeof(uint32_t) * SEG16_DIGITS);
    Stm32Lcd16SegWriteRam(dcb);
    LCD->SR |= LCD_SR_UDR;
    dcb->last_update = NutGetMillis();
    return 0;
}

/****************************************************************************
 **
 **         16-Segement Driver: Character Driver
 **
 **/

/*!
 * \brief Encode single characters to Segment Display.
 *
 * A newline\CR character will reset write pointer to digit 0.
 *
 * \return Number of LCD digits used for the character
 */
static int Stm32Lcd16SegPutc(NUTDEVICE * dev, char c)
{
    DCB_16SEG * dcb;

    dcb  = dev->dev_dcb;

    if (dcb->icmd & ICMD_ESCAPE) {

        dcb->icmd &= ~ICMD_ESCAPE;
        /* ESC sequences not handled*/
        return 0;
    } else {
        /* Non-ESC Character incoming */

        /* Start ESC Sequence? */
        if (c == 0x1b) {
            dcb->icmd |= ICMD_ESCAPE;
            return 0;
        }

        if ((c == '\n') || (c == '\r')) {  /* Return to Digit 0 */
            if (dcb->dip) {
                Stm32Lcd16SegUpdate(dcb);
            }
            dcb->dip = 0;
            return 0;
        }

#if defined(DP_DIGITS)
        if ((c == '.') && (dcb->dip > 0) && (dcb->dip < DP_DIGITS + 1)) {
            /* Add decimal point to previous digit */
            dcb->digit[dcb->dip-1] |= SEG_DP;
            return 0;
        }
#endif
#if defined(COLON_DIGITS)
        if ((c == ':') && (dcb->dip > 0) && (dcb->dip < COLON_DIGITS + 1)) {
            /* Add decimal point to previous digit */
            dcb->digit[dcb->dip-1] |= SEG_COLON;
            return 0;
        }
#endif
        if (dcb->dip >= SEG16_DIGITS) {
            /* Scoll digits */
            int i;
            Stm32Lcd16SegUpdate(dcb);

            for (i = 0; i  < SEG16_DIGITS -1; i++) {
                dcb->digit[i] = dcb->digit[i + 1];
            }
            dcb->dip --;
        }
        dcb->digit[dcb->dip] = chars2seg14[c & 0x7f];
        dcb->dip++;
    }
    return 1;
}

/*****************************************************************************
 **
 **         16-Segment Driver: File Device Handling
 **
 **/

/*!
 * \brief Handle I/O controls for 16-Segment Display.
 *
 * The 16-Seg Display is controlled by ESC-Sequences only.
 *
 * \return 0.
 */
static int Stm32Lcd16SegIOCtl(NUTDEVICE * dev, int req, void *conf)
{
/* FIXME: Allow To set BAR segments and scroll speed*/
    return 0;
}

/*!
 * \brief Send characters to 16-Segment Display.
 *
 * A newline character will reset write pointer to digit 0.
 * Carriage return is ignored.
 *
 * \return Number of characters sent.
 */
int Stm32Lcd16SegWrite(NUTFILE * fp, const void *buffer, int len)
{
    int i=len;
    const char *cp = buffer;

    NUTASSERT(fp->nf_dev != NULL);
    NUTASSERT(fp->nf_dev->dev_dcb != NULL);

    while (i--) {
        Stm32Lcd16SegPutc( fp->nf_dev, *cp++);
    }
    return len;
}

/*!
 * \brief Generate File Handle for 16-Segment Display.
 *
 * \param dev Specifies the 16seg device.
 *
 * \return 0 on success or -1 if no valid 16seg was found.
 */
NUTFILE *Stm32Lcd16SegOpen(NUTDEVICE * dev, const char *name, int mode, int acc)
{
    NUTFILE *fp;

    NUTASSERT( dev != NULL);

    if ((fp = malloc(sizeof(NUTFILE))) == 0) {
        return NUTFILE_EOF;
    }

    fp->nf_dev = dev;
    fp->nf_fcb = NULL;

    return fp;

}

/*!
 * \brief Close 16-Segment Device.
 *
 * \return 0 if closed and was opened before, else -1.
 */
static int Stm32Lcd16SegClose(NUTFILE * fp)
{
    if (fp != NULL) {
        free( fp);
        return 0;
    }
    return -1;
}

/*!
 * \brief Interrupt handler for LCD Interrupt
 *
 */
static void Stm32LcdInterrupt(void *arg)
{
    DCB_16SEG * dcb;
    uint32_t status;

    dcb = (DCB_16SEG *)arg;
    status = LCD->SR;
    if (status & LCD_SR_UDD) {
        LCD->CLR |= LCD_CLR_UDDC;
        NutEventPostFromIrq(&dcb->update_done);
    }
}

/*********************************************************************************
 **
 **         16-Segment Driver: Initialization & Device Description
 **
 **/

/*!
 * \brief Initialize the LCD 16 Seg device.
 *
 * This routine determines the 16seg type. It is internally called
 * by Nut/OS during device registration.
 *
 * The driver framework may call this function more than once.
 *
 * \param dev Specifies the 16seg device.
 *
 * \return 0 on success or -1 if no valid 16seg was found.
 */
int Stm32Lcd16SegInit(NUTDEVICE * dev)
{
    DCB_16SEG * dcb;
    const nutgpio_t *gpio;
    int res;

    NUTASSERT(dev != NULL);

    res  = Stm32EnableRtcClock();
    if (res) {
        /* FIXME: Set RTC clock*/
        return res;
    }
    /* Allocate device control block */
    dcb = malloc( sizeof( DCB_16SEG));
    if (dcb == NULL) {
        return -1;
    }
    memset( dcb, 0, sizeof( DCB_16SEG));
    dcb->display_delay = 500;
    dcb->last_update = NutGetMillis();
    dev->dev_dcb = dcb;

    /* Initialize pins*/
    for (gpio = lcd_pins; *gpio != PIN_NONE; gpio++) {
        Stm32GpioConfigSet(
            *gpio, GPIO_CFG_PERIPHAL | GPIO_CFG_SPEED_SLOW, LCD_AF);
    }
    RCC->APB1ENR  |= RCC_APB1ENR_LCDEN;

    LCD->CR = LCD_MUX | LCD_DUTY | LCD_BIAS;
    memset((void*)LCD->RAM, 0, 2 * sizeof(uint32_t) * SEG16_DIGITS);
    LCD->FCR = LCD_FCR;

    LCD->CR = LCD_MUX | LCD_DUTY | LCD_BIAS | LCD_CR_LCDEN;

    if (NutRegisterIrqHandler(&sig_LCD, Stm32LcdInterrupt, dcb) != 0) {
        free(dcb);
        return -1;
    }
    LCD->CLR |= LCD_CLR_UDDC;
    LCD->FCR |= LCD_FCR_UDDIE;
    NutIrqEnable(&sig_LCD);

    return 0;
}

/*!
 * \brief 16seg device implementation structure.
 */
NUTDEVICE devStm32Lcd16Seg = {
    .dev_name = {'1','6', 'S', 'E', 'G', 0, 0, 0},
    .dev_type = IFTYP_CHAR,
    .dev_init = Stm32Lcd16SegInit,
    .dev_ioctl = Stm32Lcd16SegIOCtl,
    .dev_write = Stm32Lcd16SegWrite,
    .dev_open  = Stm32Lcd16SegOpen,
    .dev_close = Stm32Lcd16SegClose
};

