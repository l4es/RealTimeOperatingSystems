/*
 * Copyright (C) 2013 Uwe Bonnes
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
 * THIS SOFTWARE IS PROVIDED BY ETH ZURICH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL ETH ZURICH
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
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

/* @file dev/semihost.c
 * @brief Virtual console using ARM Semihosting
 *
 * Parts of the code taken from
 * gcc-arm-none-eabi-4_7-2013q2-20130614/src/newlib/libgloss/arm
 * syscalls.c and swi.h.
 *
 * Program will only run with semihosting capable debugger attached!
 *
 * Use like:
 * E.g. with stm32 L1Discovery:
 * Start debugger like: openocd  -f board/stm32ldiscovery.cfg
 * Start openocd controlling telnet session: telnet localhost 4444
 * In telnet session, enable semihosting: "arm semihosting enable"
 * Start debug session : arm-none-eabi-gdb uart.elf
 * Attach debugger to Openocd: tar ext :3333
 * Load programm in debugger: load
 * Run program in debugger: r
 * Watch console output in terminal where openocd was started.
 */

#include <stdlib.h>
#include <string.h>

#include <sys/file.h>
#include <sys/timer.h>
#include <sys/device.h>

struct semihosting_handles  {
    int monitor_stdin;
    int monitor_stdout;
    int monitor_stderr;
};

static struct semihosting_handles sh;
/*!
 * \brief Local Semihosting information.
 */

/*!
 * \addtogroup xgDevice
 */
/*@{*/

#define AngelSWI_ARM                    0x123456
#ifdef __thumb__
#define AngelSWI                        0xAB
#else
#define AngelSWI                        AngelSWI_ARM
#endif
/* For thumb only architectures use the BKPT instruction instead of SWI.  */
#ifdef THUMB_V7M_V6M
#define AngelSWIInsn                    "bkpt"
#define AngelSWIAsm                     bkpt
#else
#define AngelSWIInsn                    "swi"
#define AngelSWIAsm                     swi
#endif

/* The reason codes:  */
#define AngelSWI_Reason_Open            0x01
#define AngelSWI_Reason_Close           0x02
#define AngelSWI_Reason_WriteC          0x03
#define AngelSWI_Reason_Write0          0x04
#define AngelSWI_Reason_Write           0x05
#define AngelSWI_Reason_Read            0x06
#define AngelSWI_Reason_ReadC           0x07
#define AngelSWI_Reason_IsTTY           0x09
#define AngelSWI_Reason_Seek            0x0A
#define AngelSWI_Reason_FLen            0x0C
#define AngelSWI_Reason_TmpNam          0x0D
#define AngelSWI_Reason_Remove          0x0E
#define AngelSWI_Reason_Rename          0x0F
#define AngelSWI_Reason_Clock           0x10
#define AngelSWI_Reason_Time            0x11
#define AngelSWI_Reason_System          0x12
#define AngelSWI_Reason_Errno           0x13
#define AngelSWI_Reason_GetCmdLine      0x15
#define AngelSWI_Reason_HeapInfo        0x16
#define AngelSWI_Reason_EnterSVC        0x17
#define AngelSWI_Reason_ReportException 0x18
#define ADP_Stopped_ApplicationExit     ((2 << 16) + 38)
#define ADP_Stopped_RunTimeError        ((2 << 16) + 35)

static inline int
do_AngelSWI (int reason, void *arg)
{
  int value;
  asm volatile ("mov r0, %1; mov r1, %2; " AngelSWIInsn " %a3; mov %0, r0"
       : "=r" (value) /* Outputs */
       : "r" (reason), "r" (arg), "i" (AngelSWI) /* Inputs */
       : "r0", "r1", "r2", "r3", "ip", "lr", "memory", "cc"
                /* Clobbers r0 and r1, and lr if in supervisor mode */);
                /* Accordingly to page 13-77 of ARM DUI 0040D other registers
                   can also be clobbered.  Some memory positions may also be
                   changed by a system call, so they should not be kept in
                   registers. Note: we are assuming the manual is right and
                   Angel is respecting the APCS.  */
  return value;
}

static int SemihostingInit(NUTDEVICE * dev)
{
    int block[3];
    struct  semihosting_handles *sh = (struct semihosting_handles *)dev->dev_icb;
    block[0] = (int) ":tt";
    block[2] = 3;     /* length of filename */
    block[1] = 0;     /* mode "r" */
    sh->monitor_stdin = do_AngelSWI (AngelSWI_Reason_Open, (void *) block);

    block[0] = (int) ":tt";
    block[2] = 3;     /* length of filename */
    block[1] = 4;     /* mode "w" */
    sh->monitor_stdout = do_AngelSWI (AngelSWI_Reason_Open, (void *) block);

    block[0] = (int) ":tt";
    block[2] = 3;     /* length of filename */
    block[1] = 8;     /* mode "a" */
    sh->monitor_stderr = do_AngelSWI (AngelSWI_Reason_Open, (void *) block);
    if (sh->monitor_stderr == -1)
        sh->monitor_stderr = sh->monitor_stdout;
    return 0;
}

/*!
 * \brief Open UnixDev
 *
 * \return Pointer to a static NUTFILE structure.
 */
static NUTFILE *SemihostingOpen(NUTDEVICE * dev, const char *name, int mode, int acc)
{
    NUTFILE *nf;

    nf = malloc(sizeof(NUTFILE));

    if (nf) {
        nf->nf_dev = dev;
    }
    return nf;
}


 /*!
 * \brief Blocking write bytes to file
 *
 * \return Number of characters sent.
 */
static int SemihostingWrite(NUTFILE * nf, const void *buffer, int len)
{
    struct  semihosting_handles *sh = (struct semihosting_handles *)nf->nf_dev->dev_icb;
    int block[3];
    block[0] = sh->monitor_stdout;
    block[1] = (int) buffer;
    block[2] = len;
    do_AngelSWI (AngelSWI_Reason_Write, (void *) block);
    return len;
}

 /*!
 * \brief Blocking write bytes to file
 *
 * \return Number of characters sent.
 */
static int SemihostingRead(NUTFILE * nf, void *buffer, int len)
{
    struct semihosting_handles *sh = (struct semihosting_handles *)nf->nf_dev->dev_icb;
    int block[3];
    int res = -1;
    block[0] = sh->monitor_stdin;
    block[1] = (int) buffer;
    block[2] = len;
    while (res < 0)
        res = do_AngelSWI (AngelSWI_Reason_Read, (void *) block);
    return len;
}


#ifdef __HARVARD_ARCH__
static int SemihostingWriteP(NUTFILE * nf, PGM_P buffer, int len)
{
    return len;
}
#endif

/*!
 * \brief Close ...
 *
 * \return Always 0.
 */
static int SemihostingClose(NUTFILE * nf)
{
    if (nf) {
        free (nf);
    }
    return 0;
}

/*!
 * \brief Perform control functions.
 *
 * This function is called by the ioctl() function of the C runtime
 * library.
 *
 * \param dev  Identifies the device that receives the device-control
 *             function.
 * \param req  Requested control function. We do return ok for any function
 * \param conf Points to a buffer that contains any data required for
 *             the given control function or receives data from that
 *             function.
 * \return 0 on success, -1 otherwise.
 *
 */
int SemihostingIOCTL(NUTDEVICE * dev, int req, void *conf)
{
    return 0;
}

/* ======================= Devices ======================== */
/*!
 * \brief Null device information structure.
 */
NUTDEVICE devSemihosting = {
    0,                                         /*!< Pointer to next device. */
    {'S', 'e', 'm', 'i', 'h', 'o', 's', 't', 0}, /*!< Unique device name. */
    IFTYP_CHAR,                                /*!< Type of device. */
    0,                                         /*!< Base address. */
    0,                                         /*!< First interrupt number. */
    &sh,                                       /*!< Interface control block. */
    0,                                         /*!< Driver control block. */
    SemihostingInit,                           /*!< Driver initialization routine. */
    SemihostingIOCTL,                          /*!< Driver specific control function. */
    SemihostingRead,
    SemihostingWrite,
#ifdef __HARVARD_ARCH__
    SemihostingWriteP,
#endif
    SemihostingOpen,
    SemihostingClose,
    0,                                         /*!< Return file size, dev_size. */
    0,                                         /*!< Select function, optional, not yet implemented */
};


/*@}*/
