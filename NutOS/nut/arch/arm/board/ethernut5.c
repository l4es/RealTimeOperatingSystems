/*
 * Copyright 2010-2012 by egnite GmbH
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
 * \file arch/arm/board/ethernut5.c
 * \brief Ethernut 5 board initialization.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <toolchain.h>

#ifndef PMM_RST_BASE
/* Power management reset port. */
#define PMM_RST_BASE    PIOB_BASE
#endif

#ifndef PMM_RST_PIN
/* Power management reset pin. */
#define PMM_RST_PIN     8
#endif


/*! \name Power Management Registers */
/*@{*/
/*! \brief Version register. */
#define PWRMAN_REG_VERS         0
/*! \brief Feature status register. */
#define PWRMAN_REG_STA          1
/*! \brief Feature enable register. */
#define PWRMAN_REG_ENA          2
/*! \brief Feature disable register. */
#define PWRMAN_REG_DIS          3
/*! \brief Board temperature register. */
#define PWRMAN_REG_TEMP         4
/*! \brief Auxiliary input voltage register. */
#define PWRMAN_REG_VAUX         6
/*! \brief LED blinking timer register. */
#define PWRMAN_REG_LEDCTL       8
/*@}*/

/* \name Feature flags */
/*@{*/
/*! \brief 1.8V and 3.3V supply. */
#define PWRMAN_BOARD    0x01
/*! \brief VBUS input at device connector. */
#define PWRMAN_VBIN     0x02
/*! \brief VBUS output at host connector. */
#define PWRMAN_VBOUT    0x04
/*! \brief Memory card supply. */
#define PWRMAN_MMC      0x08
/*! \brief RS-232 driver shutdown. */
#define PWRMAN_RS232    0x10
/*! \brief Ethernet clock enable. */
#define PWRMAN_ETHCLK   0x20
/*! \brief Ethernet PHY reset. */
#define PWRMAN_ETHRST   0x40
/*! \brief RTC wake-up. */
#define PWRMAN_WAKEUP   0x80
/*@}*/

#define ARM_TTD_DOM_LSB     5
#define ARM_TTD_DOM(x)      ((x) << ARM_TTD_DOM_LSB)

#define ARM_TTD_AP_PN_UN    0x000
#define ARM_TTD_AP_PW_UN    0x400
#define ARM_TTD_AP_PW_UR    0x800
#define ARM_TTD_AP_PW_UW    0xC00

#define ARM_TTD_INVALID     0x0
#define ARM_TTD_COARSE_PAGE 0x1
#define ARM_TTD_SECTION     0x12
#define ARM_TTD_FINE_PAGE   0x3

/* Noncacheable, nonbufferable */
#define ARM_TTD_NC_NB       0x0
/* Noncacheable, bufferable */
#define ARM_TTD_NC_B        0x4
/* Cacheable, write-through */
#define ARM_TTD_C_WT        0x8
/* Cacheable, write-back */
#define ARM_TTD_C_WB        0xC

#define ARM_SET_CP15_TTBR(val) __asm__ __volatile__("mcr p15, 0, %0, c2, c0, 0" :: "r"(val) : "cc")
#define ARM_SET_CP15_DACR(val) __asm__ __volatile__("mcr p15, 0, %0, c3, c0, 0" :: "r"(val) : "cc")

void __set_stacks(void) NUT_NAKED_FUNC;

void __init2(void) NUT_NAKED_FUNC NUT_LINKER_SECT(".init2.user");
void __init2(void)
{
    /*
     * The watchdog is enabled after processor reset.
     */
#if defined(NUT_WDT_START)
#if NUT_WDT_START
    /* Configure the watchdog. */
    outr(WDT_MR, NUT_WDT_START);
#else
    /* Disable the watchdog. */
    outr(WDT_MR, WDT_WDDIS);
#endif
#endif
    /*
     * Enable external reset key.
     */
    outr(RSTC_MR, RSTC_KEY | RSTC_URSTEN);
    /* Continue with runtime initialization. */
    __set_stacks();
}


void __clear_bss(void) NUT_NAKED_FUNC;

void __init3(void) NUT_NAKED_FUNC NUT_LINKER_SECT(".init3.user");
void __init3(void)
{
    /* Enable instruction cache. */
    ARM_SET_CP15_CR(ARM_GET_CP15_CR() | (1 << 12));

    /* Continue with runtime initialization. */
    __clear_bss();
}

void __call_rtos(void) NUT_NAKED_FUNC;

void __init4(void) NUT_NAKED_FUNC NUT_LINKER_SECT(".init4.user");
void __init4(void)
{
    static unsigned int *ttb = (unsigned int *) 0x20000000;
    static const unsigned int dom = 0xC0000000;
    static unsigned int i;

    /* Set translation table base. */
    ARM_SET_CP15_TTBR((unsigned int) ttb);
    /* Do not check access permissions for domain 15. */
    ARM_SET_CP15_DACR(dom);

    for(i = 0; i < 4096; i++) {
        ttb[i] = 0;
    }
    /* Set mapped internal SRAM section mapping. */
    ttb[0x000] = 0x00000000 | ARM_TTD_AP_PW_UN | ARM_TTD_DOM(15) | ARM_TTD_C_WB | ARM_TTD_SECTION;
    /* Set Flash memory section mapping. */
    ttb[0x002] = 0x00200000 | ARM_TTD_AP_PW_UN | ARM_TTD_DOM(15) | ARM_TTD_C_WT | ARM_TTD_SECTION;
    for(i = 0; i < 128; i++) {
        ttb[0x200 + i] = (0x20000000 + (i << 20)) | ARM_TTD_AP_PW_UN | ARM_TTD_DOM(15) | ARM_TTD_C_WB | ARM_TTD_SECTION;
    }
    /* Set external NAND Flash mapping. */
    for(i = 0; i < 256; i++) {
        ttb[0x400 + i] = (0x40000000 + (i << 20)) | ARM_TTD_AP_PW_UN | ARM_TTD_DOM(15) | ARM_TTD_SECTION;
    }
    /* Set peripheral register mapping. */
    ttb[0xFFF] = 0xFFF00000 | ARM_TTD_AP_PW_UN | ARM_TTD_DOM(15) | ARM_TTD_SECTION;

    /* Finally enable the MMU and the data cache. */
    ARM_SET_CP15_CR(ARM_GET_CP15_CR() | (1 << 12) | (1 << 2));

    /* Continue with runtime initialization. */
    __call_rtos();
}

/*!
 * \brief Delay loop.
 *
 * Delay by executing a given number of loops, roughly 5ns each.
 *
 * \param n Number of microseconds.
 */
static void BootLoopDelay(int n)
{
    while (n--) {
        _NOP();
    }
}

/*!
 * \brief Microsecond delay.
 *
 * \param us Number of microseconds.
 */
static void BootMicroDelay(int us)
{
    while (us--) {
        BootLoopDelay(200);
    }
}

/*!
 * \brief Millisecond delay.
 *
 * We are running prior to Nut/OS timer initialization and cannot use
 * NutSleep, not even NutDelay.
 *
 * \param ms Number of milliseconds.
 */
static void BootMilliDelay(int ms)
{
    while (ms--) {
        BootMicroDelay(1000);
    }
}

/*!
 * \brief Initialize the power management interface.
 */
static void PmmInit(void)
{
#if defined(PMM_RST_BASE) && defined(PMM_RST_PIN)
    /* Activate the power management reset pin. */
    outr(PMM_RST_BASE + PIO_SODR_OFF, _BV(PMM_RST_PIN));
    outr(PMM_RST_BASE + PIO_PER_OFF, _BV(PMM_RST_PIN));
    outr(PMM_RST_BASE + PIO_OER_OFF, _BV(PMM_RST_PIN));
    BootMilliDelay(1);
    /* Deactivate the reset. */
    outr(PMM_RST_BASE + PIO_CODR_OFF, _BV(PMM_RST_PIN));
    BootMilliDelay(100);
#endif
    /* Set peripheral lines for TWD and TWCK. */
    outr(PIOA_ASR, _BV(PA23_TWD_A) | _BV(PA24_TWCK_A));
    outr(PIOA_PDR, _BV(PA23_TWD_A) | _BV(PA24_TWCK_A));
    /* Switch TWI lines to open drain. */
    outr(PIOA_MDER, _BV(PA23_TWD_A) | _BV(PA24_TWCK_A));
    /* Enable TWI clock. */
    outr(PMC_PCER, _BV(TWI_ID));
    /* Disable interrupts and reset the interface. */
    outr(TWI_IDR, 0xFFFFFFFF);
    outr(TWI_CR, TWI_SWRST);
    /* Switch to master mode. */
    outr(TWI_CR, TWI_MSEN | TWI_SVDIS);
    /* Set transfer rate. */
    outr(TWI_CWGR, (7 << TWI_CKDIV_LSB) | (128 << TWI_CHDIV_LSB) | (128 << TWI_CLDIV_LSB));
}

/*!
 * \brief Write value to an 8-bit power management register.
 *
 * \param reg PMM register to write to.
 * \param val Value to write.
 *
 * \return 0 on success, -1 otherwise.
 */
static int PmmWriteReg(unsigned int reg, unsigned int val)
{
    volatile int tmo;

    outr(TWI_MMR, 0x22 << TWI_DADR_LSB);
    outr(TWI_CR, TWI_START);
    outr(TWI_THR, reg);
    for (tmo = 0; (inr(TWI_SR) & TWI_TXRDY) == 0; tmo++) {
        if (tmo > 100000) {
            return -1;
        }
    }
    outr(TWI_CR, TWI_STOP);
    outr(TWI_THR, val);
    for (tmo = 0; (inr(TWI_SR) & TWI_TXCOMP) == 0; tmo++) {
        if (tmo > 100000) {
            return -1;
        }
    }
    return 0;
}

#if 0
/* For an unknown reason the system hangs in NutTimer(!) processing
   when trying to read the version. Something is somewhere
   mysteriously broken. Stack? CPU initialization? Keep this code
   for reference. */
static int PmmReadReg(unsigned int reg, unsigned char *val)
{
    unsigned long sr;
    volatile unsigned int tmo;

    outr(TWI_IADRR, reg);
    outr(TWI_MMR, 0x22 << TWI_DADR_LSB | TWI_IADRSZ_1BYTE | TWI_MREAD);
    outr(TWI_CR, TWI_START | TWI_STOP);
    for (tmo = 0; ((sr = inr(TWI_SR)) & TWI_RXRDY) == 0; tmo++) {
        if (tmo > 100000) {
            return -1;
        }
    }
    if (sr & TWI_NACK) {
        return -1;
    }
    *val = inb(TWI_RHR);
    return 0;
}
#endif

/*!
 * \brief Ethernet PHY hardware reset.
 */
static void PmmPhyReset(void)
{
    /* Enable PIO pull-ups at PHY mode strap pins. */
    outr(PIOA_ODR, _BV(14) | _BV(15) | _BV(17));
    outr(PIOA_PUER, _BV(14) | _BV(15) | _BV(17));
    outr(PIOA_PER, _BV(14) | _BV(15) | _BV(17));

    /* Enable PIO at PHY address 0 strap pin. */
    outr(PIOA_ODR, _BV(18));
    outr(PIOA_PUDR, _BV(18));
    outr(PIOA_PER, _BV(18));

    BootMilliDelay(10);
    PmmWriteReg(PWRMAN_REG_ENA, PWRMAN_ETHRST | PWRMAN_ETHCLK);
    BootMilliDelay(1);
    PmmWriteReg(PWRMAN_REG_DIS, PWRMAN_ETHRST);
    BootMilliDelay(10);
}

/*!
 * \brief Early Ethernut 5 hardware initialization.
 */
void NutBoardInit(void)
{
    PmmInit();
    PmmPhyReset();
}
