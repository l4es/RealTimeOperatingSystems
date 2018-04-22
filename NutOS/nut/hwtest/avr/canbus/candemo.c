/*
 * Copyright (c) 2005 proconX Pty Ltd <www.proconx.com>
 *
 * $Id: candemo.c 4640 2012-09-24 12:05:56Z u_bonnes $
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
 * \example canbus/candemo.c
 *
 * Example program to demonstrate the use of the CAN bus and the ATCAN driver
 * on the <A href="http://www.proconx.com/xnut105"> XNUT-105 DIN rail single
 * board computer with CAN and embedded Ethernet</A>. The XNUT-105 module
 * features the AT90CAN128 AVR CPU with built-in CAN controller and is running
 * the Nut/OS. This demo also compiles for AVR ATmega128 designs with SJA1000
 * CAN controller.
 *
 * This program receives CAN messages and logs them on the serial port via
 * devDebug0. It also continuously broadcasts a CAN frame.
 */


// Nut/OS header
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <io.h>
#include <fcntl.h>
#include <sys/timer.h>
#include <sys/thread.h>
#include <dev/debug.h>
#include <dev/uartavr.h>
#include <dev/can_dev.h>
#include <dev/board.h>

#if defined(MCU_AT90CAN128) // Internal AVR MCU CAN controller
#  include <dev/atcan.h>
#  define DEV_CAN devAtCan
#else
#  include <dev/sja1000.h> // External SJA1000 CAN controller
#  define DEV_CAN devSJA1000
#endif


/*****************************************************************************
 * Main
 *****************************************************************************/

CANFRAME canFrame;
CANINFO *canInfoPtr;


/**
 * Main entry point of application
 */
int main(void)
{
#if defined(__AVR__) && defined(__GNUC__)
   unsigned long i;
   int result;
#endif
   uint32_t baud = 115200;

   NutRegisterDevice(&DEV_CONSOLE, 0, 0);
   freopen(DEV_CONSOLE.dev_name, "w", stdout);
   _ioctl(_fileno(stdout), UART_SETSPEED, &baud);

   printf("CAN driver test program\n");

#if defined(__AVR__) && defined(__GNUC__)

   // Init CAN controller
   result = NutRegisterDevice(&DEV_CAN, 0, 0);
   canInfoPtr = (CANINFO *) DEV_CAN.dev_dcb;

#if defined(MCU_AT90CAN128)
   // Re-configure receive message objects
   AtCanEnableRx(8, // 8 CAN objects as RX buffer, 7 remaining as TX buffer
                 0, // Acceptance code (0 = accept all IDs)
                 1, // Flag if acceptance code is extended (0 = standard, 1 = extended)
                 0, // Acceptance code's remote tag (0 or 1)
                 0, // Acceptance mask
                 0, // 0 to receive extended and standard frames, 1 if message ID type must match acceptance code flag
                 0  // 0 to receive remote and standard frames, 1 if remote tag must match acceptance code's remote tag
                 );
#endif

   // Set CAN bit rate
   CAN_SetSpeed(&DEV_CAN, CAN_SPEED_125K);

   printf("Starting CAN RX/TX loop...\n");
   for (i = 0;;i++)
   {
      // Prepare a sample frame for sending
      memset(&canFrame, 0, sizeof(canFrame));
      canFrame.id = 0x123;
      canFrame.len = 8;
      canFrame.ext = 0; // Set to 1 to send an extended frame
      canFrame.byte[0] = 0x11;
      canFrame.byte[1] = 0x22;
      canFrame.byte[2] = 0x33;
      canFrame.byte[3] = 0x44;
      canFrame.byte[4] = 0x55;
      canFrame.byte[5] = 0x66;
      canFrame.byte[6] = 0x77;
      canFrame.byte[7] = 0x88;
      CAN_TxFrame(&DEV_CAN, &canFrame);

      // Check if we did receive a frame
      if (CAN_TryRxFrame(&DEV_CAN, &canFrame) == 0)
      {
         uint8_t j;

         printf("%ld ", canFrame.id);
         for (j = 0; j < canFrame.len; j++)
            printf("%02X ", canFrame.byte[j]);
         printf(" Stats: %lu %lu %lu %lu\n", canInfoPtr->can_interrupts,
                canInfoPtr->can_rx_frames,
                canInfoPtr->can_tx_frames,
                canInfoPtr->can_overruns);
      }
      NutSleep(10); // Don't overflow the bus and give time to other threads
   }
#else /* __AVR__ && __GNUC__ */
   puts("No CAN device available");
#endif /* __AVR__ && __GNUC__ */
   return 0;
}

