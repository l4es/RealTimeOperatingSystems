/*
 * Copyright (C) 2012 by Rob van Lieshout (info@pragmalab.nl)
 *                       Ole Reinhardt (ole.reinhardt@embedded-it.de)
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

#include <string.h>

#include <cfg/os.h>
#include <cfg/clock.h>
#include <cfg/arch.h>

#include <sys/heap.h>
#include <sys/event.h>
#include <sys/timer.h>

#include <dev/irqreg.h>
#include <dev/rtc.h>

#if defined(MCU_LPC176x)
#include <arch/cm3/nxp/lpc176x_clk.h>
#elif defined(MCU_LPC177x_8x)
#include <arch/cm3/nxp/lpc177x_8x_clk.h>
#elif defined(MCU_LPC407x_8x)
#include <arch/cm3/nxp/lpc407x_8x_clk.h>
#else
#warning "Unknown LPC familiy"
#endif

#include <arch/cm3/nxp/lpc17xx_rtc.h>


typedef struct _lpc17xx_rtc_dcb lpc17xx_rtc_dcb;

struct _lpc17xx_rtc_dcb {
    uint32_t flags;
};

/*!
 * \brief Get date and time from an LPC 17xx hardware clock.
 *
 * \param tm Points to a structure that receives the date and time
 *           information.
 *
 * \return 0 on success or -1 in case of an error.
 */
static int Lpc17xxRtcGetClock(NUTRTC *rtc, struct _tm *tm)
{
    if (tm) {
        tm->tm_mday=  LPC_RTC->DOM   & RTC_DOM_MASK;
        tm->tm_wday=  LPC_RTC->DOW   & RTC_DOW_MASK;
        tm->tm_yday= (LPC_RTC->DOY   & RTC_DOY_MASK) - 1;
        tm->tm_hour=  LPC_RTC->HOUR  & RTC_HOUR_MASK;
        tm->tm_min =  LPC_RTC->MIN   & RTC_MIN_MASK;
        tm->tm_sec =  LPC_RTC->SEC   & RTC_SEC_MASK;
        tm->tm_mon = (LPC_RTC->MONTH & RTC_MONTH_MASK) - 1;
        tm->tm_year= (LPC_RTC->YEAR  & RTC_YEAR_MASK) - 1900;

        return 0;
    } else {
        return -1;
    }
}

/*!
 * \brief Set the LPC 17xx hardware clock.
 *
 * \param tm Points to a structure which contains the date and time
 *           information.
 *
 * \return 0 on success or -1 in case of an error.
 */
static int Lpc17xxRtcSetClock(NUTRTC *rtc, const struct _tm *tm)
{
    if (tm) {
        /* Disable timer / counter for writing, reset the 32Khz clock divider */        
        LPC_RTC->CCR &= ~RTC_CCR_CLKEN;
        LPC_RTC->CCR |=  RTC_CCR_CTCRST;


        LPC_RTC->DOM  =   tm->tm_mday & RTC_DOM_MASK;
        LPC_RTC->DOW  =   tm->tm_wday & RTC_DOW_MASK;
        LPC_RTC->DOY  =  (tm->tm_yday & RTC_DOY_MASK) + 1;
        LPC_RTC->HOUR =   tm->tm_hour & RTC_HOUR_MASK;
        LPC_RTC->MIN  =   tm->tm_min  & RTC_MIN_MASK;
        LPC_RTC->SEC  =   tm->tm_sec  & RTC_SEC_MASK;
        LPC_RTC->MONTH = (tm->tm_mon  & RTC_MONTH_MASK) + 1;
        LPC_RTC->YEAR =  (tm->tm_year & RTC_YEAR_MASK) + 1900;

        /* Enable timer / counter again, disable ctc reset */
        LPC_RTC->CCR &= ~RTC_CCR_CTCRST;
        LPC_RTC->CCR |=  RTC_CCR_CLKEN;
        return 0;
    } else {
        return -1;
    }
}

/*!
 * \brief Interrupt handler for RTC Alarm
 *
 */
static void Lpc17xxRtcInterrupt(void *arg)
{
    NUTRTC *rtc = (NUTRTC *)arg;
    /*
     *  there is only 1 interrupt for the RTC, so check here
     *  if it was the CIIF or the CALF. We need the CALF here
     */
    if (LPC_RTC->ILR & RTC_IRL_RTCCIF) {
        LPC_RTC->ILR |= RTC_IRL_RTCCIF;
    }

    /* Continue to check the Alarm match*/
    if (LPC_RTC->ILR & RTC_IRL_RTCALF) {
        ((lpc17xx_rtc_dcb *)rtc->dcb)->flags |= RTC_STATUS_AL0;

        /* Signal alarm event queue */
        NutEventPostFromIrq(&rtc->alarm);

        /* Clear pending interrupt */
        LPC_RTC->ILR |= RTC_IRL_RTCALF;
    }
}

/*!
 * \brief Set an alarm using the LPC 17xx hardware clock.
 *
 * \param Idx is ignored (LPC 17xx only has 1 Alarm)
 *           information.
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
 * \return 0 on success or -1 in case of an error.
 */
static int Lpc17xxRtcSetAlarm(NUTRTC *rtc, int idx, const struct _tm *tm, int aflags)
{
    uint32_t amr = 0;

    if (tm) {
        if (aflags & RTC_ALARM_SECOND) {
            LPC_RTC->ALSEC  = tm->tm_sec  & RTC_SEC_MASK;
            amr |= RTC_AMR_AMRSEC;
        }
        if (aflags & RTC_ALARM_MINUTE) {
            LPC_RTC->ALMIN  = tm->tm_min  & RTC_MIN_MASK;
            amr |= RTC_AMR_AMRMIN;
        }
        if (aflags & RTC_ALARM_HOUR) {
            LPC_RTC->ALHOUR = tm->tm_hour & RTC_HOUR_MASK;
            amr |= RTC_AMR_AMRHOUR;
        }
        if (aflags & RTC_ALARM_MDAY) {
            LPC_RTC->ALDOM  = tm->tm_mday & RTC_DOM_MASK;
            amr |= RTC_AMR_AMRDOM;
        }
        if (aflags & RTC_ALARM_MONTH) {
            LPC_RTC->ALMON  = (tm->tm_mon  & RTC_MONTH_MASK) + 1;
            amr |= RTC_AMR_AMRMON;
        }
        if (aflags & RTC_ALARM_WDAY) {
            LPC_RTC->ALDOW  = tm->tm_wday & RTC_DOW_MASK;
            amr |= RTC_AMR_AMRDOW;
        }
        if (aflags & RTC_ALARM_YEAR) {
            LPC_RTC->ALYEAR = (tm->tm_year & RTC_YEAR_MASK) + 1900;
            amr |= RTC_AMR_AMRYEAR;
        }
        if (aflags & RTC_ALARM_YDAY) {
            LPC_RTC->ALDOY  = (tm->tm_yday & RTC_DOY_MASK) + 1;
            amr |= RTC_AMR_AMRDOY;
        }
        LPC_RTC->AMR = (~amr) & RTC_AMR_BITMASK;
        return 0;
    } else {
        return -1;
    }
}

/*!
 * \brief Get an alarm using the LPC17xx hardware clock.
 *
 * \param Idx is ignored (LPC17xx only has 1 Alarm)
 *           information.
 * \param tm Points to a structure which contains the date and time
 *           information.
 *
 * \param aflags Points to an unsigned long that receives the enable flags.
 *
 * \return 0 on success or -1 in case of an error.
 */
static int Lpc17xxRtcGetAlarm(NUTRTC *rtc, int idx, struct _tm *tm, int *aflags)
{
    register uint32_t amr;

    memset(tm, 0, sizeof(struct _tm));
    *aflags = 0;

    if (aflags == NULL) {
        return -1;
    }

    if (tm) {
        /* Inverted mask register */
        amr = ~(LPC_RTC->AMR);

        if (amr & RTC_AMR_AMRDOM) {
            tm->tm_mday= LPC_RTC->ALDOM  & RTC_DOM_MASK;
            *aflags |= RTC_ALARM_MDAY;
        }
        if (amr & RTC_AMR_AMRDOW) {
            tm->tm_wday= LPC_RTC->ALDOW  & RTC_DOW_MASK;
            *aflags |= RTC_ALARM_WDAY;
        }

        if (amr & RTC_AMR_AMRDOY) {
            tm->tm_yday= (LPC_RTC->ALDOY  & RTC_DOY_MASK) - 1;
            *aflags |= RTC_ALARM_YDAY;
        }

        if (amr & RTC_AMR_AMRHOUR) {
            tm->tm_hour= LPC_RTC->ALHOUR & RTC_HOUR_MASK;
            *aflags |= RTC_ALARM_HOUR;
        }

        if (amr & RTC_AMR_AMRMIN) {
            tm->tm_min= LPC_RTC->ALMIN & RTC_MIN_MASK;
            *aflags |= RTC_ALARM_MINUTE;
        }

        if (amr & RTC_AMR_AMRSEC) {
            tm->tm_sec = LPC_RTC->ALSEC & RTC_SEC_MASK;
            *aflags |= RTC_ALARM_SECOND;
        }

        if (amr & RTC_AMR_AMRMON) {
            tm->tm_mon = (LPC_RTC->ALMON & RTC_MONTH_MASK) - 1;
            *aflags |= RTC_ALARM_MONTH;
        }

        if (amr & RTC_AMR_AMRYEAR) {
            tm->tm_year= (LPC_RTC->ALYEAR & RTC_YEAR_MASK) - 1900;
            *aflags |= RTC_ALARM_YEAR;
        }

        return 0;
    } else {
        return -1;
    }
}

/*!
 * \brief Get status of the LPC17xx hardware clock.
 *
 * \param sflags Points to an unsigned long that receives the status flags.
 *               - Bit 0: Power fail.
 *               - Bit 5: Alarm occured.
 * \return 0 on success or -1 in case of an error.
 */
static int Lpc17xxRtcGetStatus(NUTRTC *rtc, uint32_t *sflags)
{
    /* Check for power failure and clear status bit */
    if (LPC_RTC->RTC_AUX & RTC_AUX_RTC_OSCF) {
        ((lpc17xx_rtc_dcb *)rtc->dcb)->flags |= RTC_STATUS_PF;
        LPC_RTC->RTC_AUX |= RTC_AUX_RTC_OSCF;
    }

    if (sflags) {
        *sflags = ((lpc17xx_rtc_dcb *)rtc->dcb)->flags;
        return 0;
    } else {
        return -1;
    }
}

/*!
 * \brief Clear status of the LPC17xx hardware clock.
 *
 * \param tm Points to a structure which contains the date and time
 *           information.
 *
 * \return 0 on success or -1 in case of an error.
 */
static int Lpc17xxRtcClearStatus(NUTRTC *rtc, uint32_t sflags)
{
    ((lpc17xx_rtc_dcb *)rtc->dcb)->flags &= ~sflags;

    return 0;
}

/*!
 * \brief Initialize the RTC in LPC 17xx controller
 *
 * \return 0 on success or -1 in case of an error.
 *
 */
static int Lpc17xxRtcInit(NUTRTC *rtc)
{
    rtc->dcb = NutHeapAllocClear(sizeof(lpc17xx_rtc_dcb));
    if (rtc->dcb == NULL) {
        return -1;
    }

    if (NutRegisterIrqHandler(&sig_RTC, Lpc17xxRtcInterrupt, rtc) != 0) {
        NutHeapFree(rtc->dcb);
        return -1;
    }

    /* Set up clock and power for RTC module */
    SysCtlPeripheralClkEnable(CLKPWR_PCONP_PCRTC);

    /* Clear all register to be default */
    LPC_RTC->ILR  = 0x03;
    LPC_RTC->CCR  = 0x00;
    LPC_RTC->CIIR = 0x00;
    LPC_RTC->AMR  = 0xFF;
    LPC_RTC->CALIBRATION = 0x00;

    /* enable RTC (run) */
    LPC_RTC->CCR |= RTC_CCR_CLKEN;

    ((lpc17xx_rtc_dcb *)rtc->dcb)->flags =  RTC_STATUS_HAS_QUEUE;
    rtc->alarm = NULL;

    NutIrqEnable(&sig_RTC);

    return(0);
}

NUTRTC rtcLpc17xx  = {
  /*.dcb           = */ NULL,                   /*!< Driver control block */
  /*.rtc_init      = */ Lpc17xxRtcInit,         /*!< Hardware initialization, rtc_init */
  /*.rtc_gettime   = */ Lpc17xxRtcGetClock,     /*!< Read date and time, rtc_gettime */
  /*.rtc_settime   = */ Lpc17xxRtcSetClock,     /*!< Set date and time, rtc_settime */
  /*.rtc_getalarm  = */ Lpc17xxRtcGetAlarm,     /*!< Read alarm date and time, rtc_getalarm */
  /*.rtc_setalarm  = */ Lpc17xxRtcSetAlarm,     /*!< Set alarm date and time, rtc_setalarm */
  /*.rtc_getstatus = */ Lpc17xxRtcGetStatus,    /*!< Read status flags, rtc_getstatus */
  /*.rtc_clrstatus = */ Lpc17xxRtcClearStatus,  /*!< Clear status flags, rtc_clrstatus */
  /*.alarm         = */ NULL,                   /*!< Handle for alarm event queue */
};
