/*
 * Copyright (C) 2014 Ole Reinhardt <ole.reinhardt@embedded-it.de>
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

#include <toolchain.h>

#include <stdio.h>
#include <stdlib.h>
#include <io.h>
#include <ctype.h>
#include <cfg/arch.h>
#include <dev/gpio.h>
#include <sys/thread.h>
#include <sys/time.h>
#include <sys/timer.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <dev/debug.h>
#include <dev/usart_lpc17xx.h>
#include <dev/spibus_lpc17xx_ssp.h>

#include <dev/spi_lcd_st7565r.h>
#include <dev/framebuffer.h>

#define DBG_BAUDRATE 115200
#define DEV_CONSOLE devUsartLpc17xx_0

#define LED1 18
#define LED2 20
#define LED3 21
#define LED4 23

#define DISPLAY_WIDTH  128
#define DISPLAY_HEIGHT  32
#define FRAMEBUFFER_SIZE (DISPLAY_WIDTH * DISPLAY_HEIGHT / 8)

#define ST7565R_SPI_CHIPSELECT_GPIO  (0 * 32 + 18)   /* GPIO PORT 0, PIN 18 */

ST7565R_DCB st7565r_dcb = {
	.a0_port    = NUTGPIO_PORT0,
	.a0_pin     = 6,
	.reset_port = NUTGPIO_PORT0,
	.reset_pin  = 8,
};

FBINFO st7565r_fb = {
	.width    = 128,
	.height   = 32,
};

THREAD(Thread1, arg)
{
	int i = 0;
    while(1) {
        /* Blink the MBED board LEDs */
		GpioPinSet(NUTGPIO_PORT1, LED1, i & 0x01);
		GpioPinSet(NUTGPIO_PORT1, LED2, i & 0x02);
		GpioPinSet(NUTGPIO_PORT1, LED3, i & 0x04);
		GpioPinSet(NUTGPIO_PORT1, LED4, i & 0x08);
		NutSleep(250);
		i ++;
	}
}

int lcd_framebuffer_init(void)
{
	NUTSPINODE *node;
	int rc;

	/* Setup custom settings of the framebuffer device. */

	/* Get the SPI node settings to configure address- and reset line and the spi node chipselect */
	node = devSt7565rFb0.dev_icb;
	node->node_dcb = &st7565r_dcb;

	/* Configure framebuffer / LCD resolution */
	devSt7565rFb0.dev_dcb = &st7565r_fb;

    /* Register the SPI bus node. The LPC17xx SSP driver needs the chip select
     * parameter to be given as GPIO pin number in format: GPIO_PORT * 32 + PIN
     */ 
	printf("Registering SPI bus and framebuffer device... ");
	rc = NutRegisterSpiDevice(&devSt7565rFb0, &spiBus1Lpc17xxSsp, ST7565R_SPI_CHIPSELECT_GPIO);
    if (rc != 0){
        /* If it fails, leave a message and end here */
        printf("FAILED\n");
        fflush(stdout);
        while(1);
    }
    printf("OK\n");

	return 0;
}


/*!
* \brief Main application routine.
*
*/
int main(void)
{
    uint8_t *fb;
    uint8_t *tmp_fb;
    uint32_t baud = 115200;
    int      fb_fd;
    int      i;
    
	FILE *uart;

	GpioPinConfigSet(NUTGPIO_PORT1, LED1, GPIO_CFG_OUTPUT);
	GpioPinConfigSet(NUTGPIO_PORT1, LED2, GPIO_CFG_OUTPUT);
	GpioPinConfigSet(NUTGPIO_PORT1, LED3, GPIO_CFG_OUTPUT);
	GpioPinConfigSet(NUTGPIO_PORT1, LED4, GPIO_CFG_OUTPUT);

	NutRegisterDevice(&DEV_CONSOLE, 0, 0);
	uart = fopen(DEV_CONSOLE.dev_name, "r+");
	_ioctl(_fileno(uart), UART_SETSPEED, &baud);

	freopen(DEV_CONSOLE.dev_name, "w", stdout);
	freopen(DEV_CONSOLE.dev_name, "w", stderr);
	freopen(DEV_CONSOLE.dev_name, "r", stdin);

	puts ("ST7565r framebuffer test - Nut/OS");

	lcd_framebuffer_init();
    
	NutThreadCreate("t1", Thread1, 0, 512);

    fb_fd = _open(devSt7565rFb0.dev_name, _O_RDWR);

    fb = malloc(FRAMEBUFFER_SIZE);
    tmp_fb = malloc(FRAMEBUFFER_SIZE);
    
    /* Slowly fill the display with some (stripe) pattern. Always write 8 (new) 
       pixels, then update the display, until it is fully written.
     */
    for (i = 0; i < FRAMEBUFFER_SIZE; i++) {
        fb[i] = 0x33;
        _write(fb_fd, fb, FRAMEBUFFER_SIZE);
    }

    /* Now read the pattern from the framebuffer, and alternating show a 
     * blank display, or the saved pattern.
     */ 
    while (1) {
        /* Read back the framebuffer */
        _read(fb_fd, tmp_fb, FRAMEBUFFER_SIZE);
        /* Clean the current frame buffer and update the display */
        memset(fb, 0, FRAMEBUFFER_SIZE);
        _write(fb_fd, fb, FRAMEBUFFER_SIZE);
        NutSleep(1000);
        /* Copy back the saved content and update the display */
        memcpy(fb, tmp_fb, FRAMEBUFFER_SIZE);
        _write(fb_fd, fb, FRAMEBUFFER_SIZE);
        NutSleep(1000);
    }
    /* Cleanup everything (never reached, but shall show how to close the 
       framebuffer device and cleanup allocated ressources 
     */
    _close(fb_fd);
    free(fb);
    free(tmp_fb);
    
	return 0;
}
