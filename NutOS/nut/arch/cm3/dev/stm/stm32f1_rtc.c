/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Nikolaj Zamotaev. All rights reserved.
 * Copyright (C) 2017 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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

/*
 * \verbatim
 * $Id: stm32f1_rtc.c 6605 2017-02-15 15:36:30Z u_bonnes $
 * \endverbatim
 */

#include <cfg/arch.h>
#include <cfg/os.h>
#include <cfg/clock.h>
#include <cfg/arch.h>
#include <sys/event.h>
#include <sys/timer.h>
#include <dev/rtc.h>

#include <cfg/arch/gpio.h>
#include <arch/cm3/stm/stm32_clk.h>

#include <stdlib.h>
#include <string.h>
#include <time.h>

# define EXTI_RTC_LINE 17
# define RTC_STATUS_MASK  (RTC_STATUS_HAS_QUEUE | RTC_STATUS_INACCURATE)

typedef struct _stm32_rtcv1_dcb stm32_rtcv1_dcb;

struct _stm32_rtcv1_dcb {
    uint32_t status;
};

static stm32_rtcv1_dcb rtc_dcb;

/* RTC_CRL has both rw and rc_w0 bits. Write exact bits by using bitbanding! */
static volatile uint32_t *rtc_crl = CM3BB_BASE(&RTC->CRL);

#if (RTCCLK_SOURCE != RTCCLK_HSE)
/*!
 * \brief Set/Reset PWR->CR DBP bit if not using HSE
 *
 * Using HSE as RTC clock needs PWR_CR_DBP always enabled.
 *
 * \param none
 */
static void Stm32RtcDbpSet(void)
{
    PWR->CR |= PWR_CR_DBP;
}
static void Stm32RtcDbpRst(void)
{
    PWR->CR &= ~PWR_CR_DBP;
}
#else
# define Stm32RtcDbpSet()
# define Stm32RtcDbpRst()
#endif

/*!
 * \brief Wait for RTC to sync.
 *
 * \param none
 */
static void Stm32RtcSync(void)
{
    rtc_crl[_BI32(RTC_CRL_RSF)] = 0;
    while (rtc_crl[_BI32(RTC_CRL_RSF)] == 0);
}

static void Stm32RtcRtoffPoll(void)
{
    while(rtc_crl[_BI32(RTC_CRL_RTOFF)] == 0);
};

/*!
 * \brief Get the alarm values from the STM32F1 hardware clock.
 *
 * \param Idx Zero based index. Only one alarms are supported>
 * \param tm Points to a structure to receive date and time information.
 * \param aflags Points to an unsigned long that receives the enable flags.
 *
 * \return 0 on success or -1 in case of an error.
 */
static int Stm32RtcGetAlarm(NUTRTC *rtc, int idx, struct _tm *tm, int *aflags)
{
    time_t time;

    if (idx) {
        return -1;
    }
    Stm32RtcDbpSet();
    Stm32RtcSync();
    Stm32RtcDbpRst();
    time = RTC->ALRL | (RTC->ALRH << 16);
    if (tm) {
        localtime_r(&time, tm);
    }
    if (aflags) {
        uint32_t flags;
        if (time) {
            flags = RTC_ALARM_EXACT;
        } else {
            flags = RTC_ALARM_OFF;
        }
        *aflags = flags;
    }
    return 0;
}

/*!
 * \brief Set an alarm using the STM32F1 hardware clock.
 *
 * \param Idx   Zero based index. Only one alarms are supported.
 *
 * \param tm    Points to a structure which contains the date and time
 *              information. May be NULL to clear the alarm.
 * \param aflgs Each bit enables a specific comparision.
 *              - Bit 0: Seconds
 *              - Bit 1: Minutes
 *              - Bit 2: Hours
 *              - Bit 3: Day of month
 *              - Bit 4: Month
 *              - Bit 7: Day of week (Sunday is zero)
 *              - Bit 8: Year
 *              - Bit 9: Day of year
 *
 *                Stm32F1 RTC can only alert on exact time
 *
 * \return 0 on success or -1 in case of an error.
 */
static int Stm32RtcSetAlarm(NUTRTC *rtc, int idx, const struct _tm *tm, int aflags)
{
    time_t time = 0;

    if ((idx) || ((aflags != RTC_ALARM_EXACT) && (aflags != RTC_ALARM_OFF))) {
        return -1;
    }
    if (!tm) {
        if (aflags != RTC_ALARM_OFF) {
            return -1;
        }
    } else {
        time = mktime((struct _tm *)tm);
    }
    Stm32RtcDbpSet();
    Stm32RtcRtoffPoll();
    rtc_crl[_BI32(RTC_CRL_CNF)] = 1;
    if (aflags != RTC_ALARM_OFF) {
        RTC->ALRL = time & 0xffff;
        RTC->ALRH = time >> 16;
    } else {
        RTC->ALRL = 0;
        RTC->ALRH = 0;
    }
    rtc_crl[_BI32(RTC_CRL_CNF)] = 0;
    Stm32RtcRtoffPoll();
    Stm32RtcDbpRst();
    return 0;
}

/*!
 * \brief Interrupt handler for F1 RTC Alarm
 *
 */
static void Stm32RtcInterrupt(void *arg)
{
    NUTRTC *rtc = (NUTRTC *)arg;
    stm32_rtcv1_dcb *dcb = (stm32_rtcv1_dcb *)rtc->dcb;

    int do_alert = 0;
    Stm32RtcDbpSet();
    Stm32RtcSync();
    rtc_crl[_BI32(RTC_CRL_CNF)] = 1;
    if (rtc_crl[_BI32(RTC_CRL_ALRF)]) {
        dcb->status |= RTC_STATUS_AL0;
        /* Clear pending interrupt */
        rtc_crl[_BI32(RTC_CRL_ALRF)] = 0;
        do_alert ++;
    } else {
        rtc_crl[_BI32(RTC_CRL_SECF)] = 0;
        rtc_crl[_BI32(RTC_CRL_OWF)]  = 0;
    }
    rtc_crl[_BI32(RTC_CRL_CNF)] = 0;
    Stm32RtcDbpRst();
    if(do_alert) {
        /* Clear Pending EXTI RTC Interrupt*/
        EXTI_PR =  (1 << EXTI_RTC_LINE);
        /* Signal alarm event queue */
        NutEventPostFromIrq(&rtc->alarm);
    }
}

/*
 * \param sflags Points to an unsigned long that receives the status flags.
 *               - Bit 0: Backup Domain Reset happened.
 *               - Bit 5: Alarm occured.
 * \return 0 on success or -1 in case of an error.
 */
static int Stm32RtcGetStatus(NUTRTC *rtc, uint32_t *sflags)
{
    int res;
    time_t time;
    stm32_rtcv1_dcb *dcb = (stm32_rtcv1_dcb *)rtc->dcb;

    /* Check for RTC power failure, seen by -1 in Alarm register*/
    time = RTC->ALRL | (RTC->ALRH << 16);
    if (time == 0xffffffff) {
        dcb->status |= RTC_STATUS_PF;
    }
    if (sflags) {
        *sflags = dcb->status;
        res = 0;
    } else {
        res = -1;
    }
    return res;
}

/*!
 * \brief Clear status of the Stm32 hardware clock.
 *
 * \param sflags Mask of flags to clear
 *
 * \return 0 on success or -1 in case of an error.
 */
static int Stm32RtcClearStatus(NUTRTC *rtc, uint32_t sflags)
{
    stm32_rtcv1_dcb *dcb = (stm32_rtcv1_dcb *)rtc->dcb;
    /* Don't reset persistant flags*/
    sflags &= ~RTC_STATUS_MASK;
    dcb->status &= ~sflags;
    return 0;
}

/*!
 * \brief Get date and time from an Stm32F1 RTC
 *
 * \param tm Points to a structure that receives the date and time
 *           information.
 *
 * \return 0 on success or -1 in case of an error.
 */
static int Stm32RtcGetClock(NUTRTC *rtc, struct _tm *tm)
{
    time_t time;

    if (!tm) {
        return -1;
    }
    Stm32RtcDbpSet();
    Stm32RtcSync();
    Stm32RtcDbpRst();
    time=((RTC->CNTL|(RTC->CNTH<<16)));
    localtime_r(&time,tm);
    return 0;
}

/*!
 * \brief Set the Stm32F1 RTC
 *
 * \param tm Points to a structure which contains the date and time
 *           information.
 *
 * \return 0 on success or -1 in case of an error.
 */
static int Stm32RtcSetClock(NUTRTC *rtc, const struct _tm *tm)
{
   time_t time;
    stm32_rtcv1_dcb *dcb = (stm32_rtcv1_dcb *)rtc->dcb;

   if (!tm) {
       return -1;
   }
   time = mktime((struct _tm *)tm);
   Stm32RtcDbpSet();
   Stm32RtcRtoffPoll();
   rtc_crl[_BI32(RTC_CRL_CNF)] = 1;
   RTC->CNTL = time & 0xffff;
   RTC->CNTH = time >> 16;
   rtc_crl[_BI32(RTC_CRL_CNF)] = 0;
   Stm32RtcRtoffPoll();
   Stm32RtcDbpRst();
   /* Remove power failure flag after setting time.*/
   dcb->status &= ~RTC_STATUS_PF;
   return 0;
}


/*!
 * \brief Initialize the RTC in stm32f10x controller
 *
 * \return 0 on success or -1 in case of an error.
 *
 */
static int Stm32RtcInit(NUTRTC *rtc)
{
    uint32_t prediv;
    int res;
    stm32_rtcv1_dcb *dcb = (stm32_rtcv1_dcb *)rtc->dcb;
    int rc;

    res = NutRegisterIrqHandler(&sig_RTC, Stm32RtcInterrupt, rtc);
    if (res) {
        return -1;
    }
    /* Alarm Interrupt is on EXTI RTC Line, Rising Edge */
    EXTI_PR   =   (1 << EXTI_RTC_LINE);
    EXTI_IMR  |=  (1 << EXTI_RTC_LINE);
    EXTI_RTSR |=  (1 << EXTI_RTC_LINE);
    EXTI_FTSR &= ~(1 << EXTI_RTC_LINE);
    res = NutIrqEnable(&sig_RTC);
    if (res) {
        /* Some failure happend */
        return res;
    }
    dcb->status = RTC_STATUS_HAS_QUEUE;
    if ((RCC_BDCR & RCC_BDCR_RTCSEL) != (RTCCLK_LSE * RCC_BDCR_RTCSEL_0)) {
        dcb->status |= RTC_STATUS_INACCURATE;
    }
    if (!(RCC->BDCR & RCC_BDCR_RTCEN)) {
        res = Stm32EnableRtcClock();
        dcb->status |= RTC_STATUS_PF;
        if (res) {
            return -1;
        }
    }
    PWR->CR |= PWR_CR_DBP;
    Stm32RtcSync();
    prediv = RTC->PRLH << 16 | RTC->PRLL;
#if RTCCLK_SOURCE == RTCCLK_LSE
# define  RTC_PREDIV (LSE_VALUE - 1)
#elif RTCCLK_SOURCE == RTCCLK_HSE
# define  RTC_PREDIV ((HSE_VALUE / 128) - 1)
#elif RTCCLK_SOURCE == RTCCLK_LSI
# define  RTC_PREDIV (40000 - 1)
#endif
#if (RTCCLK_SOURCE == RTCCLK_KEEP)
    if ((RCC->BDCR & RCC_BDCR_RTCEN) && ((RCC->BDCR & RCC_BDCR_RTCSEL) != 0)  &&
        (prediv!= 0) && ((RTC->CNTL) || (RTC->CNTH))) {
        /* The RTC seems to be running */
        rc =  0;
    }
    rc = -1;
#else
    /* Check for valid or changed settings */
    uint32_t source;
    source = (RCC->BDCR & RCC_BDCR_RTCSEL) / RCC_BDCR_RTCSEL_0;
    if ((RCC->BDCR & RCC_BDCR_RTCEN) && (RTCCLK_SOURCE == source)  &&
        (prediv!= 0) && ((RTC->CNTL) || (RTC->CNTH))) {
        rc = 0;
    }
    Stm32RtcRtoffPoll();
    rtc_crl[_BI32(RTC_CRL_CNF)] = 1;
    RTC->CRH  = RTC_CRH_ALRIE;
    RTC->PRLL = RTC_PREDIV & 0xffff;
    RTC->PRLH = RTC_PREDIV >> 16;
    RTC->CNTL = 0;
    RTC->CNTH = 1; /*1 january 1970 */
    RTC->ALRL = 0;
    RTC->ALRH = 0;
    rtc_crl[_BI32(RTC_CRL_CNF)] = 0;
    Stm32RtcRtoffPoll();
    rc = 0;
#endif
     Stm32RtcDbpRst();
     return rc;
}

NUTRTC rtcStm32 = {
    .dcb           = &rtc_dcb,
    .rtc_init      = Stm32RtcInit,
    .rtc_gettime   = Stm32RtcGetClock,
    .rtc_settime   = Stm32RtcSetClock,
    .rtc_getalarm  = Stm32RtcGetAlarm,
    .rtc_setalarm  = Stm32RtcSetAlarm,
    .rtc_getstatus = Stm32RtcGetStatus,
    .rtc_clrstatus = Stm32RtcClearStatus,
    .alarm         = NULL,
};
