/*!
 * Copyright (C) 2012 by egnite GmbH
 * Copyright (C) 2001-2005 by egnite Software GmbH
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
 * $Id: timers.c 6275 2016-01-14 16:18:25Z u_bonnes $
 */

/*!
 * \example timers/timers.c
 *
 * This sample demonstrates the usage of Nut/OS timer functions.
 */
#include <cfg/os.h>
#include <cfg/arch.h>
#include <dev/board.h>

#include <sys/thread.h>
#include <sys/timer.h>
#include <sys/event.h>
#include <sys/heap.h>

#include <stdio.h>
#include <io.h>

/*
 * Timer callback function.
 *
 * Timer callbacks are called during context switch processing. They must
 * return as soon as possible and must not call any potentially blocking
 * function.
 *
 * It would be great to be able to start a thread here, but unfortunately
 * this doesn't work in Nut/OS. :-(
 */
static void TimerCallback(HANDLE timer, void *arg)
{
    NutEventPostAsync(arg);
}

/*
 * Demonstrate Nut/OS one-shot timer.
 *
 * One-shot timers will call the callback function once and then stop
 * the timer automatically.
 *
 * Note that this function also demonstrates event timeouts.
 */
static void OneShotDemo(int s)
{
    HANDLE t;
    HANDLE e = NULL;
    int w;

    printf("Start %d s one-shot timer\n", s);
    t = NutTimerStart(s * 1000UL, TimerCallback, &e, TM_ONESHOT);
    (void) t;
    for (w = 1; w < s + 1; w++) {
        printf("  Waiting %d s...", w);
        if (NutEventWait(&e, 1000UL) == 0) {
            puts("elapsed");
            break;
        }
        puts("timed out");
    }
}

/*
 * Demonstrate Nut/OS periodic timer.
 *
 * Periodic timers will call the callback function and then restart
 * the timer automatically. They must be explicitly stopped, if no
 * longer needed.
 */
static void PeriodicDemo(int s)
{
    HANDLE t;
    HANDLE e = NULL;
    int i;
    uint32_t ms;

    printf("Start %d s periodic timer\n", s);
    t = NutTimerStart(s * 1000UL, TimerCallback, &e, 0);
    for (i = 0; i < 5; i++) {
        ms = NutGetMillis();
        printf("  Waiting...");
        NutEventWait(&e, NUT_WAIT_INFINITE);
        ms = NutGetMillis() - ms;
        printf("elapsed after %lu ms\n", ms);
    }
    puts("  Stop periodic timer");
    NutTimerStop(t);
}

/*
 * Demonstrate Nut/OS delay timer.
 *
 * The delay function will not release the CPU, keeping all other threads
 * blocked. It is a good choice for very short delays, where context
 * switching is undesirable or would take too long. Use it sparingly and
 * for short delays only.
 */
static void DelayDemo(void)
{
    uint32_t req;
    uint32_t act;

    for (req = 1024UL; req <= 8UL * 1024UL * 1024UL; req *= 2) {
        printf("Delaying %lu us, ", req);
        act = NutGetMillis();
        NutMicroDelay(req);
        act = NutGetMillis() - act;
        printf("delayed %lu ms\n", act);
    }
}

/*
 * Demonstrate Nut/OS sleep timer.
 *
 * This often used function serves two purposes: Delaying the current
 * thread and releasing the CPU.
 *
 * This demo will run endlessly.
 */
static void SleepDemo(void)
{
    uint32_t req = 1;
    uint32_t act;

    for (;;) {
        printf("Sleeping %lu ms, ", req);
        act = NutGetMillis();
        NutSleep(req);
        act = NutGetMillis() - act;
        printf("slept %lu ms\n", act);
        req *= 2;
    }
}

/*
 * Main application routine.
 */
int main(void)
{
    uint32_t baud = 115200;

    /*
     * Register the UART device, open it, assign stdout to it and set
     * the baudrate.
     */
    NutRegisterDevice(&DEV_CONSOLE, 0, 0);
    freopen(DEV_CONSOLE.dev_name, "w", stdout);
    _ioctl(_fileno(stdout), UART_SETSPEED, &baud);
    puts("\n\nNut/OS timer sample");

    /*
     * Display some general info.
     */
    printf("CPU running at %lu Hz\n", NutGetCpuClock());
    printf("%lu system ticks per second\n", NutTimerMillisToTicks(1000));

    /*
     * Run the demos.
     */
    OneShotDemo(10);
    PeriodicDemo(2);
    DelayDemo();
    SleepDemo(); /* Never returns. */

    return 0;
}
