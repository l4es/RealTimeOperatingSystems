/*
 * Copyright (C) 2001-2007 by egnite Software GmbH.
 * Copyright (C) 2013 by Comm5 Tecnologia Ltda.
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

#include <cfg/arch.h>
#include <cfg/lcd.h>

#include <dev/hd44780.h>
#include <dev/term.h>
#include <dev/gpio.h>
#include <dev/board.h>

#include <sys/thread.h>
#include <sys/nutconfig.h>
#include <sys/timer.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * Configuration and defaults block
 *
 */

// Disable compilation if essential settings are not defined
#if defined( LCD_ROWS ) && defined( LCD_COLS ) && defined( LCD_EN_PIO_ID ) && defined( LCD_DATA_PIO_ID )

#ifndef LCD_ROWS
#define LCD_ROWS    2
#endif

#ifndef LCD_COLS
#define LCD_COLS    16
#endif

#if !defined(LCD_IF_8BIT) && !defined(LCD_IF_4BIT)
#define LCD_IF_4BIT
#endif

#ifdef LCD_IF_4BIT
# define LCD_DATA_MASK     (0x0F << LCD_DATA_LSB)
#else
# define LCD_DATA_MASK     (0xFF << LCD_DATA_LSB)
#endif

#ifndef LCD_SHORT_DELAY
#define LCD_SHORT_DELAY 200
#endif

#ifndef LCD_LONG_DELAY
#define LCD_LONG_DELAY  400
#endif

#define GPIO_ID LCD_EN_PIO_ID
#include <cfg/arch/porttran.h>
static INLINE void LCD_EN_SET(void) { GPIO_SET_HI(LCD_EN_BIT); }
static INLINE void LCD_EN_CLR(void) { GPIO_SET_LO(LCD_EN_BIT); }
static INLINE void LCD_EN_INIT(void) { GPIO_OUTPUT(LCD_EN_BIT); }
#undef GPIO_ID

#define GPIO_ID LCD_RS_PIO_ID
#include <cfg/arch/porttran.h>
static INLINE void LCD_RS_SET(void) { GPIO_SET_HI(LCD_RS_BIT); }
static INLINE void LCD_RS_CLR(void) { GPIO_SET_LO(LCD_RS_BIT); }
static INLINE void LCD_RS_INIT(void) { GPIO_OUTPUT(LCD_RS_BIT); }
#undef GPIO_ID

#define GPIO_ID LCD_RW_PIO_ID
#include <cfg/arch/porttran.h>
static INLINE void LCD_RW_SET(void) { GPIO_SET_HI(LCD_RW_BIT); }
static INLINE void LCD_RW_CLR(void) { GPIO_SET_LO(LCD_RW_BIT); }
static INLINE void LCD_RW_INIT(void) { GPIO_OUTPUT(LCD_RW_BIT); }
#undef GPIO_ID

/*!
 * \addtogroup xgDisplay
 */
/*@{*/

#if defined(LCD_RW_BIT)

static INLINE uint8_t LcdReadNibble(void)
{
    uint8_t ret;
	LCD_RW_SET();
	GpioPortConfigSet( LCD_DATA_PIO_ID, LCD_DATA_MASK, GPIO_CFG_INPUT | GPIO_CFG_PULLUP );
	LCD_EN_SET();
	NutMicroDelay(LCD_SHORT_DELAY);
	ret = GpioPortGet( LCD_DATA_PIO_ID ) & LCD_DATA_MASK;
	ret >>= LCD_DATA_LSB;
	LCD_EN_CLR();
	GpioPortConfigSet( LCD_DATA_PIO_ID, LCD_DATA_MASK, GPIO_CFG_OUTPUT );
	NutMicroDelay(LCD_SHORT_DELAY);
	return ret;
}

static INLINE uint8_t LcdReadByte(void)
{
    uint8_t data;
#if defined(LCD_IF_4BIT)
    data = LcdReadNibble();
    data = data | (LcdReadNibble() << 4);
#else
    data = LcdReadNibble();
#endif
    return data;
}

static uint8_t LcdReadCmd(void)
{
	LCD_RS_CLR();
	return LcdReadByte();
}
#endif //  LCD_RW_BIT

static void LcdDelay(int xt)
{
#if defined(LCD_RW_BIT)
	while (LcdReadCmd() & (1 << LCD_BUSY))
		NutMicroDelay(2);
    /* If configured, let the task sleep before next character */
#elif defined(LCD_SLEEP_DLY)
    NutSleep(1);
#else
    /* or add a fixed delay and immediately process next char */
    NutMicroDelay(xt);
#endif
}

/*!
 * \brief Send half byte to LCD controller.
 *
 * \param nib The four least significant bits are sent.
 */
static void LcdWriteNibble(unsigned int nib)
{
    nib <<= LCD_DATA_LSB;
	GpioPortSetHigh( LCD_DATA_PIO_ID, nib & LCD_DATA_MASK );
	GpioPortSetLow( LCD_DATA_PIO_ID, ~nib & LCD_DATA_MASK );

    /* Create Enable Pulse:
     * For HD44780 Displays we need:
     * Vcc = 5.0V -> PWeh >= 230ns
     * Vcc = 3.3V -> PWeh >= 500ns
     */
    LCD_EN_SET();
    NutMicroDelay(LCD_SHORT_DELAY);
    LCD_EN_CLR();
	NutMicroDelay(LCD_SHORT_DELAY);
}

/*!
 * \brief Send byte to LCD controller.
 *
 * \param data Byte to send.
 */
static void LcdWriteByte(unsigned int data)
{
    /* If configured set RW low */
#ifdef LCD_RW_BIT
    LCD_RW_CLR();
#endif

    /* If using 4-bit access, write two nibbles now */
#ifdef LCD_IF_4BIT
    LcdWriteNibble(data >> 4);
    LcdDelay(LCD_SHORT_DELAY);
    LcdWriteNibble(data);
#else
    /* else write one byte */
    LcdWriteNibble(data);
#endif
	LcdDelay(LCD_SHORT_DELAY);
}

/*!
 * \brief Send command byte to LCD controller.
 *
 * \param cmd Byte to send.
 */
static void LcdWriteCmd(uint8_t cmd)
{
    /* RS low selects instruction register. */
    LCD_RS_CLR();
    LcdWriteByte(cmd);
}

static void LcdWriteInstruction(uint8_t cmd, uint8_t xt)
{
    LcdWriteCmd(cmd);
}

/*!
 * \brief Send data byte to LCD controller.
 *
 * \param data Byte to send.
 */
static void LcdWriteData(uint8_t data)
{
    /* RS high selects data register. */
    LCD_RS_SET();
    LcdWriteByte(data);
}

static void LcdSetCursor(uint8_t pos)
{
    uint8_t offset[] = {
#ifdef KS0073_CONTROLLER
        0x00, 0x20, 0x40, 0x60
#elif LCD_COLS == 20
        0x00, 0x40, 0x14, 0x54
#else
        0x00, 0x40, 0x10, 0x50
#endif
    };

    pos = offset[(pos / LCD_COLS) % LCD_ROWS] + pos % LCD_COLS;
    LcdWriteCmd(1 << LCD_DDRAM | pos);
}

static void LcdCursorHome(void)
{
    LcdWriteCmd(1 << LCD_HOME);
    NutSleep(2);
}

static void LcdCursorLeft(void)
{
    LcdWriteCmd(1 << LCD_MOVE);
}

static void LcdCursorRight(void)
{
    LcdWriteCmd(1 << LCD_MOVE | 1 << LCD_MOVE_RIGHT);
}

static void LcdClear(void)
{
    LcdWriteCmd(_BV(LCD_CLR));
    NutSleep(2);
}

static void LcdCursorMode(uint8_t on)
{
    LcdWriteCmd(1 << LCD_ON_CTRL | on ? 1 << LCD_ON_CURSOR : 0x00);
}

static int LcdInit(NUTDEVICE * dev)
{
	LCD_RS_INIT();
	LCD_RW_INIT();
	GpioPortConfigSet( LCD_DATA_PIO_ID, LCD_DATA_MASK, GPIO_CFG_OUTPUT );

    LCD_RS_CLR();
    LCD_RW_CLR();
    GpioPortSetLow( LCD_DATA_PIO_ID, LCD_DATA_MASK );
    NutMicroDelay(30);

	LCD_EN_INIT();
    LCD_EN_CLR();
    NutMicroDelay(30);
    NutSleep(18);

    /* This initialization will make sure, that the LCD is switched
     * to 8-bit mode, no matter which mode we start from or we finally
     * need.
     */
    LcdWriteCmd(_BV(LCD_FUNCTION) | _BV(LCD_FUNCTION_8BIT));
    NutSleep(15);
    LcdWriteCmd(_BV(LCD_FUNCTION) | _BV(LCD_FUNCTION_8BIT));
    NutSleep(4);
    LcdWriteCmd(_BV(LCD_FUNCTION) | _BV(LCD_FUNCTION_8BIT));
    NutSleep(2);

#ifdef LCD_IF_4BIT
    /* We now switch to 4-bit mode */
    LcdWriteNibble(_BV(LCD_FUNCTION) >> 4);
    NutSleep(2);
#endif

	LcdWriteCmd(_BV(LCD_FUNCTION) | _BV(LCD_FUNCTION_8BIT) | ((((TERMDCB *) dev->dev_dcb)->dcb_nrows > 1) ? _BV(LCD_FUNCTION_2LINES) : 0));
	NutSleep(2);

    /* Clear display. */
    LcdClear();

    /* Set entry mode. */
    LcdWriteCmd(_BV(LCD_ENTRY_MODE) | _BV(LCD_ENTRY_INC));

    /* Switch display on. */
    LcdWriteCmd(_BV(LCD_ON_CTRL) | _BV(LCD_ON_DISPLAY));

    /* Move cursor home. */
    LcdCursorHome();

    /* Set data address to zero. */
    LcdWriteCmd(_BV(LCD_DDRAM));

    return 0;
}

/*!
 * \brief Terminal device control block structure.
 */
TERMDCB dcb_term = {
    LcdInit,                    /*!< \brief Initialize display subsystem, dss_init. */
    LcdWriteData,               /*!< \brief Write display character, dss_write. */
    LcdWriteInstruction,        /*!< \brief Write display command, dss_command. */
    LcdClear,                   /*!< \brief Clear display, dss_clear. */
    LcdSetCursor,               /*!< \brief Set display cursor, dss_set_cursor. */
    LcdCursorHome,              /*!< \brief Set display cursor home, dss_cursor_home. */
    LcdCursorLeft,              /*!< \brief Move display cursor left, dss_cursor_left. */
    LcdCursorRight,             /*!< \brief Move display cursor right, dss_cursor_right. */
    LcdCursorMode,              /*!< \brief Switch cursor on/off, dss_cursor_mode. */
    0,                          /*!< \brief Mode flags. */
    0,                          /*!< \brief Status flags. */
    LCD_ROWS,                   /*!< \brief Number of rows. */
    LCD_COLS,                   /*!< \brief Number of columns per row. */
    LCD_COLS,                   /*!< \brief Number of visible columns. */
    0,                          /*!< \brief Cursor row. */
    0,                          /*!< \brief Cursor column. */
    0                           /*!< \brief Display shadow memory. */
};

/*!
 * \brief LCD device information structure.
 */
NUTDEVICE devLcd = {
    0,                          /*!< Pointer to next device. */
    {'l', 'c', 'd', 0, 0, 0, 0, 0, 0},  /*!< Unique device name. */
    IFTYP_STREAM,               /*!< Type of device. */
    0,                          /*!< Base address. */
    0,                          /*!< First interrupt number. */
    0,                          /*!< Interface control block. */
    &dcb_term,                  /*!< Driver control block. */
    TermInit,                   /*!< Driver initialization routine. */
    TermIOCtl,                  /*!< Driver specific control function. */
    0,
    TermWrite,
#ifdef __HARVARD_ARCH__
	TermWrite_P,
#endif
    TermOpen,
    TermClose,
    0,
    0,                          /*!< Select function, dev_select, optional. */
};

#endif // Disable compilation if essential settings are not defined
/*@}*/
