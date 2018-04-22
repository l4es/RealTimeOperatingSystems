/*
 * Copyright (C) 2009 by Ulrich Prinz, <uprinz2@netscape.net>
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

/*
 * $Id$
 */

/*!
 * \example buttons/pbtest.c
 *
 * Push button driver example.
 *
 * This sample demonstrates Nut/OS LEd and key handling.
 * It is written for the ATMEL SAM7X-EK but can be adapted to any other
 * hardware within minutes.
 *
 */

#include <stdio.h>
#include <io.h>

#include <cfg/arch.h>
#include <cfg/dev.h>
#include <dev/board.h>
#include <sys/timer.h>
#include <sys/event.h>
#include <sys/thread.h>

#include <dev/gpio.h>

#include <dev/keys.h>
#include <dev/led.h>


/* This sample application only supports the following boards */
#if defined(AT91SAM7X_EK) || defined(ENET_SAM7X) || \
    defined(EVK1100) || defined(EVK1101) || defined(EVK1105) || \
    defined(STM3210C_EVAL) || defined(STM3210E_EVAL)


#if defined(AT91SAM7X_EK) || defined(ENET_SAM7X)
#define LED1_PORT    NUTGPIO_PORTB
#define LED1_PIN     19
#define LED2_PORT    NUTGPIO_PORTB
#define LED2_PIN     20
#define LED3_PORT    NUTGPIO_PORTB
#define LED3_PIN     21
#define LED4_PORT    NUTGPIO_PORTB
#define LED4_PIN     22

#define KEY_MI_PORT  NUTGPIO_PORTA
#define KEY_MI_PIN   25
#define KEY_DN_PORT  NUTGPIO_PORTA
#define KEY_DN_PIN   22
#define KEY_LT_PORT  NUTGPIO_PORTA
#define KEY_LT_PIN   23
#define KEY_RT_PORT  NUTGPIO_PORTA
#define KEY_RT_PIN   24
#define KEY_UP_PORT  NUTGPIO_PORTA
#define KEY_UP_PIN   21
#elif defined(EVK1100)
#define LED1_PORT    NUTGPIO_PORTB
#define LED1_PIN     27
#define LED2_PORT    NUTGPIO_PORTB
#define LED2_PIN     28
#define LED3_PORT    NUTGPIO_PORTB
#define LED3_PIN     29
#define LED4_PORT    NUTGPIO_PORTB
#define LED4_PIN     30

#define KEY_MI_PORT  NUTGPIO_PORTA
#define KEY_MI_PIN   20
#define KEY_DN_PORT  NUTGPIO_PORTA
#define KEY_DN_PIN   27
#define KEY_LT_PORT  NUTGPIO_PORTA
#define KEY_LT_PIN   25
#define KEY_RT_PORT  NUTGPIO_PORTA
#define KEY_RT_PIN   28
#define KEY_UP_PORT  NUTGPIO_PORTA
#define KEY_UP_PIN   26

#elif defined(EVK1101)
#define LED1_PORT    NUTGPIO_PORTA
#define LED1_PIN     7
#define LED2_PORT    NUTGPIO_PORTA
#define LED2_PIN     8
#define LED3_PORT    NUTGPIO_PORTA
#define LED3_PIN     21
#define LED4_PORT    NUTGPIO_PORTA
#define LED4_PIN     22

#define KEY_MI_PORT  NUTGPIO_PORTA
#define KEY_MI_PIN   13
#define KEY_DN_PORT  NUTGPIO_PORTB
#define KEY_DN_PIN   8
#define KEY_LT_PORT  NUTGPIO_PORTB
#define KEY_LT_PIN   6
#define KEY_RT_PORT  NUTGPIO_PORTB
#define KEY_RT_PIN   9
#define KEY_UP_PORT  NUTGPIO_PORTB
#define KEY_UP_PIN   7

#elif defined(EVK1105)
#define LED1_PORT    NUTGPIO_PORTB
#define LED1_PIN     27
#define LED2_PORT    NUTGPIO_PORTB
#define LED2_PIN     28
#define LED3_PORT    NUTGPIO_PORTA
#define LED3_PIN     5
#define LED4_PORT    NUTGPIO_PORTA
#define LED4_PIN     6

#define KEY_MI_PORT  NUTGPIO_PORTB
#define KEY_MI_PIN   26
#define KEY_DN_PORT  NUTGPIO_PORTB
#define KEY_DN_PIN   23
#define KEY_LT_PORT  NUTGPIO_PORTB
#define KEY_LT_PIN   25
#define KEY_RT_PORT  NUTGPIO_PORTB
#define KEY_RT_PIN   24
#define KEY_UP_PORT  NUTGPIO_PORTB
#define KEY_UP_PIN   22

#elif defined(STM3210C_EVAL)
#define LED1_PORT    NUTGPIO_PORTD
#define LED1_PIN     7
#define LED2_PORT    NUTGPIO_PORTD
#define LED2_PIN     13
#define LED3_PORT    NUTGPIO_PORTD
#define LED3_PIN     3
#define LED4_PORT    NUTGPIO_PORTD
#define LED4_PIN     4

/* For STM3210C-EVAL Kit the 5-way controller is connected to
 * a I2C expander. So we use the few single buttons instead.
 */
#define KEY_MI_PORT  NUTGPIO_PORTB  /* User Button */
#define KEY_MI_PIN   9
#define KEY_DN_PORT  NUTGPIO_PORTC  /* Tamper Button */
#define KEY_DN_PIN   13
#define KEY_LT_PORT  NUTGPIO_PORTA  /* Wakeup Button */
#define KEY_LT_PIN   0
//#define KEY_RT_PORT  NUTGPIO_PORTB
//#define KEY_RT_PIN   24
//#define KEY_UP_PORT  NUTGPIO_PORTB
//#define KEY_UP_PIN   22

#elif defined(STM3210E_EVAL)
#define LED1_PORT    NUTGPIO_PORTF
#define LED1_PIN     6
#define LED2_PORT    NUTGPIO_PORTF
#define LED2_PIN     7
#define LED3_PORT    NUTGPIO_PORTF
#define LED3_PIN     8
#define LED4_PORT    NUTGPIO_PORTF
#define LED4_PIN     9

#define KEY_MI_PORT  NUTGPIO_PORTG
#define KEY_MI_PIN   7
#define KEY_DN_PORT  NUTGPIO_PORTD
#define KEY_DN_PIN   3
#define KEY_LT_PORT  NUTGPIO_PORTG
#define KEY_LT_PIN   14
#define KEY_RT_PORT  NUTGPIO_PORTG
#define KEY_RT_PIN   13
#define KEY_UP_PORT  NUTGPIO_PORTG
#define KEY_UP_PIN   15
#endif





/* Handles for the LEDs */
HANDLE led1, led2, led3, led4;
/* Handles for the keys / joystick of the SAM7X-EK */
HANDLE keyUp, keyDn, keyLt, keyRt, keyMi;
/* Handles for two threads controlled by the keys */
HANDLE keyT1w, keyT2w;

/*!
 * \brief First thread responding to pressed keys.
 *
 * This thread handles some of the keys of this example.
 * In response to a key it gives some text on the debug consle
 * and alters the behaviour of the assigned LEDs.
 *
 */
THREAD(Key1Thread, arg)
{
    NutThreadSetPriority(60);
    for (;;) {
        NutEventWait(&keyT1w, NUT_WAIT_INFINITE);
        if (NutGetKeyState(keyMi) & KEY_PENDING) {
            printf("KEY ENTER pressed\n");
            NutSetLed(led2, LED_OFF, 0, 0);
            NutSetLed(led3, LED_OFF, 0, 0);
            NutSetLed(led4, LED_OFF, 0, 0);
        }
        if (NutGetKeyState(keyDn) & KEY_PENDING) {
            printf("\nKEY DOWN pressed\n");
            NutSetLed(led2, LED_BLINK, 100, 900);
            NutSetLed(led3, LED_BLINK, 500, 500);
            NutSetLed(led4, LED_BLINK, 900, 100);
        }
    }
}

/*!
 * \brief Second0 thread responding to pressed keys.
 *
 * This thread handles some of the keys of this example.
 * In response to a key it gives some text on the debug consle
 * and alters the behaviour of the assigned LEDs.
 *
 */
THREAD(Key2Thread, arg)
{
    NutThreadSetPriority(60);
    for (;;) {
        NutEventWait(&keyT2w, NUT_WAIT_INFINITE);
        if (NutGetKeyState(keyLt) & KEY_PENDING) {
            printf("\nKEY LEFT pressed\n");
            NutSetLed(led2, LED_ON, 200, 0);
            NutSetLed(led4, LED_OFF, 0, 0);
        }
        if (NutGetKeyState(keyRt) & KEY_PENDING) {
            printf("\nKEY RIGHT pressed\n");
            NutSetLed(led2, LED_OFF, 0, 0);
            NutSetLed(led4, LED_ON, 200, 0);
        }
        if (NutGetKeyState(keyUp) & KEY_PENDING) {
            printf("\nKEY UP pressed\n");
            NutSetLed(led3, LED_FLIP, 0, 0);
        }
    }
}


/*
 * Main application thread.
 */
int main(void)
{
    uint32_t baud = 115200;
    /*
     * Register the UART device, open it, assign stdout to it and set
     * the baudrate.
     */
    NutRegisterDevice(&DEV_DEBUG, 0, 0);
    freopen(DEV_DEBUG_NAME, "w", stdout);
    _ioctl(_fileno(stdout), UART_SETSPEED, &baud);

#if defined(EVK1104)
    puts("\n*** Sorry, LED and key Test not supported by the EVK1104 ***\n");
    fflush(stdout);
#else
    puts("\n*** LED and key Test ***\n");
    fflush(stdout);

    /* Register LED1 as blinking led */
    NutRegisterLed(&led1, LED1_PORT, LED1_PIN);
    NutSetLed(led1, LED_BLINK, 100, 900);

    /* Register LED2 and let it flash once for 200ms */
    NutRegisterLed(&led2, LED2_PORT, LED2_PIN);
    NutSetLed(led2, LED_ON, 200, 0);

    /* Register LED3 and let it flash once for 200ms */
    NutRegisterLed(&led3, LED3_PORT, LED3_PIN);
    NutSetLed(led3, LED_ON, 200, 0);

    /* Register LED4 and let it flash once for 200ms */
    NutRegisterLed(&led4, LED4_PORT, LED4_PIN);
    NutSetLed(led4, LED_ON, 200, 0);

    /* Register keys for thread */

    /* First we register middle function of the 5-way control of the SAM7X-EK
     * to do something if it is pressed for less than 1s.
     */
    NutRegisterKey(&keyMi, KEY_MI_PORT, KEY_MI_PIN, KEY_ACTION_SHORT, 1000);

    /* Then we assign, what to do if the above event has happened
     * Here we assign to release a handle.
     */
    NutAssignKeyEvt(keyMi, &keyT1w);

    /* Second example is to register the down function of the 5-way controller
     * to a different functionality, it executes something after beeing pressed
     * for at least 2s.
     */
#if !defined(EVK1105)
    NutRegisterKey(&keyDn, KEY_DN_PORT, KEY_DN_PIN, KEY_ACTION_HOLD, 2000);
#else
    /* The EVK1105 have touch keys, therefore we will not wait 2s here */
    NutRegisterKey(&keyDn, KEY_DN_PORT, KEY_DN_PIN, KEY_ACTION_UP, 0);
#endif


    /* Assign event to keyDn too.
     */
    NutAssignKeyEvt(keyDn, &keyT1w);

    /* Register keys for thread 2 mutex.
     * We assign two of the keys to release a mutex.
     */
#ifdef KEY_LT_PORT
    NutRegisterKey(&keyLt, KEY_LT_PORT, KEY_LT_PIN, KEY_ACTION_UP, 0);
    NutAssignKeyEvt(keyLt, &keyT2w);
#endif
#ifdef KEY_RT_PORT
    NutRegisterKey(&keyRt, KEY_RT_PORT, KEY_RT_PIN, KEY_ACTION_DOWN, 0);
    NutAssignKeyEvt(keyRt, &keyT2w);
#endif
#ifdef KEY_UP_PORT
    NutRegisterKey(&keyUp, KEY_UP_PORT, KEY_UP_PIN, KEY_ACTION_DOWN, 0);
    NutAssignKeyEvt(keyUp, &keyT2w);
#endif

    /* Register threads that wait on keys */
    NutThreadCreate("k1", Key1Thread, NULL, 256);
    NutThreadCreate("k2", Key2Thread, NULL, 256);
#endif

    /*
     * Endless loop in main thread.
     */
    for (;;) {
        putchar('.');
        NutSleep(2000);
    }
    return 0;
}

#else  /* defined(AT91SAM7X_EK) || defined(ENET_SAM7X) ||
          defined(EVK1100) || defined(EVK1101) || defined(EVK1105) */
int main(void)
{
    uint32_t baud = 115200;
    /*
     * Register the UART device, open it, assign stdout to it and set
     * the baudrate.
     */
    NutRegisterDevice(&DEV_DEBUG, 0, 0);
    freopen(DEV_DEBUG_NAME, "w", stdout);
    _ioctl(_fileno(stdout), UART_SETSPEED, &baud);
    printf("Sorry, your board is not supported \n");
    while(1);
}
#endif
