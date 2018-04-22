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

#ifndef _SPI_LCD_ST7565_H_
#define _SPI_LCD_ST7565_H_

#include <toolchain.h>
#include <dev/spibus.h>

/* ST7565R clock settings */

#define ST7565R_CLK_MAX         20000000
#define ST7565R_CONTRAST_MAX    40
#define ST7565R_CONTRAST_MIN    30

/* Framebuffer settings */

#define ST7565R_BPP             1
#define ST7565R_MAX_WIDTH       132
#define ST7565R_MAX_HEIGHT      65

/* LCD controller commands*/

#define ST7565R_CMD_DISPLAY_ON                     0xAF
#define ST7565R_CMD_DISPLAY_OFF                    0xAE
#define ST7565R_CMD_START_LINE_SET(line)          (0x40 | (line))
#define ST7565R_CMD_PAGE_ADDRESS_SET(page)        (0xB0 | (page))
#define ST7565R_CMD_COLUMN_ADDRESS_SET_MSB(col)   (0x10 | (col))
#define ST7565R_CMD_COLUMN_ADDRESS_SET_LSB(col)   (0x00 | (col))
#define ST7565R_CMD_ADC_NORMAL                     0xA0
#define ST7565R_CMD_ADC_REVERSE                    0xA1
#define ST7565R_CMD_DISPLAY_NORMAL                 0xA6
#define ST7565R_CMD_DISPLAY_REVERSE                0xA7
#define ST7565R_CMD_DISPLAY_ALL_POINTS_OFF         0xA4
#define ST7565R_CMD_DISPLAY_ALL_POINTS_ON          0xA5
#define ST7565R_CMD_LCD_BIAS_1_DIV_6_DUTY33        0xA2
#define ST7565R_CMD_LCD_BIAS_1_DIV_5_DUTY33        0xA3
#define ST7565R_CMD_NORMAL_SCAN_DIRECTION          0xC0
#define ST7565R_CMD_REVERSE_SCAN_DIRECTION         0xC8
#define ST7565R_CMD_VOLTAGE_RESISTOR_RATIO_0       0x20
#define ST7565R_CMD_VOLTAGE_RESISTOR_RATIO_1       0x21
#define ST7565R_CMD_VOLTAGE_RESISTOR_RATIO_2       0x22
#define ST7565R_CMD_VOLTAGE_RESISTOR_RATIO_3       0x23
#define ST7565R_CMD_VOLTAGE_RESISTOR_RATIO_4       0x24
#define ST7565R_CMD_VOLTAGE_RESISTOR_RATIO_5       0x25
#define ST7565R_CMD_VOLTAGE_RESISTOR_RATIO_6       0x26
#define ST7565R_CMD_VOLTAGE_RESISTOR_RATIO_7       0x27
#define ST7565R_CMD_POWER_CTRL_ALL_ON              0x2F
#define ST7565R_CMD_SLEEP_MODE                     0xAC
#define ST7565R_CMD_NORMAL_MODE                    0xAD
#define ST7565R_CMD_RESET                          0xE2
#define ST7565R_CMD_NOP                            0xE3
#define ST7565R_CMD_END                            0xEE
#define ST7565R_CMD_READ_MODIFY_WRITE              0xE0
#define ST7565R_CMD_ELECTRONIC_VOLUME_MODE_SET     0x81
#define ST7565R_CMD_ELECTRONIC_VOLUME(volume)      (0x3F & (~volume))
#define ST7565R_CMD_BOOSTER_RATIO_SET              0xF8
#define ST7565R_CMD_BOOSTER_RATIO_2X_3X_4X         0x00
#define ST7565R_CMD_BOOSTER_RATIO_5X               0x01
#define ST7565R_CMD_BOOSTER_RATIO_6X               0x03
#define ST7565R_CMD_STATUS_READ                    0x00

typedef struct _ST7565R_DCB {
    HANDLE dcb_lock;
	int    a0_port;
	int    a0_pin;
	int    reset_port;
	int    reset_pin;
} ST7565R_DCB;

extern NUTSPINODE nodeSt7565r0;
extern NUTDEVICE devSt7565rFb0;

extern int  St7565rNodeLock(NUTDEVICE * dev);
extern void St7565rNodeUnlock(NUTDEVICE * dev);
extern int  St7565rSetMode(NUTDEVICE * dev, int sleep);
extern int  St7565rSetPageAddress(NUTDEVICE * dev, uint8_t address);
extern int  St7565rSetColAddress(NUTDEVICE * dev, uint8_t address);
extern int  St7565rSetDisplayStartLineAddress(NUTDEVICE * dev, uint8_t address);
extern int  St7565rDisplayEnable(NUTDEVICE * dev, int enable);
extern int  St7565rSetContrast(NUTDEVICE * dev, uint8_t contrast);
extern int  St7565rDisplayInvert(NUTDEVICE * dev, int invert);
extern int  St7565rDebugPixelsAllOn(NUTDEVICE * dev, int all_on);
extern int  St7565rUpdateFb(NUTDEVICE * dev);

#endif
