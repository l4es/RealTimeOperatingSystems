/*!
 * Copyright (C) 2001-2005 by egnite Software GmbH
 * Copyright (C) 2009-2010 by egnite GmbH
 * Copyright (C) 2014 by Uwe Bonnes(bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * $Id: caltime.c 6598 2017-02-15 15:36:00Z u_bonnes $
 */

#include <dev/board.h>
#include <dev/debug.h>

#include <sys/version.h>
#include <sys/timer.h>
#include <sys/event.h>

#include <stdlib.h>
#include <stdio.h>
#include <io.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#ifdef USE_BUILD_TIME
#include <pro/rfctime.h>
#include <cfg/crt.h>
#endif

#ifdef USE_LINE_EDITOR
#include <gorp/edline.h>
#define LINE_MAXLENGTH  80
static char edbuff[LINE_MAXLENGTH];
static EDLINE *edline;
#endif

/*!
 * \example caltime/caltime.c
 *
 * Demonstrates Nut/OS date and time functions, which had been contributed
 * by Oliver Schulz.
 *
 * Check the Makefile for additional options.
 */
static char *version = "2.2";


/* Used for ASCII Art Animation. */
static char *rotor = "|/-\\";

static char *weekday_name[7] = {
    "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"
};

/*
 * Print content of tm structure.
 */
static void PrintDateTime(const struct _tm *stm)
{
    printf("%s, %04d/%02d/%02d, %02d:%02d:%02d%s"
           , weekday_name[stm->tm_wday]
           , stm->tm_year + 1900, stm->tm_mon + 1, stm->tm_mday
           , stm->tm_hour, stm->tm_min, stm->tm_sec
           , stm->tm_isdst ? " DST" : ""
           );
}

static void Alarm(int idx)
{
    struct _tm rtctime, alarmtime;
    int aflag = RTC_ALARM_SECOND;
    uint32_t rtc_stat;
    uint_fast8_t i;
    uint32_t rtc_alarm;
    int has_queue;

    if (NutRtcGetTime(&rtctime) == 0) {
        rtctime.tm_sec += 10;
        rtctime.tm_sec %= 60;
        if (NutRtcSetAlarm(idx, &rtctime, aflag) == 0) {
            NutRtcGetAlarm(idx, &alarmtime, NULL);
            printf("Setting Alarm to: ");
            PrintDateTime(&alarmtime);
            puts("");
        }
        else {
            puts("Setting Alarm Time failed");
            return;
        }
    }
    else
        puts("Getting RTC time failed");

    rtc_alarm = (idx)?RTC_STATUS_AL1:RTC_STATUS_AL0;
    i = 0;
    NutRtcGetStatus(&rtc_stat);
    /* Most RTC Implementations don't have a Event queue yet */
    has_queue = rtc_stat & RTC_STATUS_HAS_QUEUE;
    if (has_queue) {
        HANDLE *alarm = NutRtcGetHandle();
        do {
            NutRtcGetTime(&rtctime);
            printf(" [%c] Alarm%c : Current time: "
                   , rotor[++i & 3]
                   , 'A' + idx);
            PrintDateTime(&rtctime);
            printf("\r");
            if (NutEventWait(alarm, 200) == 0)
                break;
        } while (i < 60);
    }
    else {
        do {
            printf(" [%c] Waiting for Alarm%c\r"
                   , rotor[++i & 3]
                   , 'A' + idx);
            NutSleep(200);
            NutRtcGetStatus(&rtc_stat);
        } while (((rtc_stat & rtc_alarm) != rtc_alarm) && ( i < 60));
    }
    putchar('\n');
    NutRtcGetAlarm(idx, &rtctime, &aflag);
    NutRtcSetAlarm(idx, &rtctime, 0); /* Switch off alarm */
    NutRtcGetStatus(&rtc_stat);
    if ((rtc_stat & rtc_alarm) != rtc_alarm)
        printf(" Failed!");
    else {
        printf("Alarm set time was %4d/%2d/%2d (%s) ",
               rtctime.tm_year + 1900, rtctime.tm_mon + 1, rtctime.tm_mday,
               weekday_name[rtctime.tm_wday]);
        printf("%2d:%02d:%02d (UTC) Flags: %08x\n",
               rtctime.tm_hour, rtctime.tm_min, rtctime.tm_sec, aflag);
    }
}


/*
 * Query calendar date from user.
 *
 * Returns 0 on success, -1 otherwise.
 */
static int EnterDate(struct _tm *stm)
{
    int year = stm->tm_year + 1900;
    int month = stm->tm_mon + 1;
    int day = stm->tm_mday;

    printf("Enter date, use format YYYY/MM/DD");
#ifdef USE_LINE_EDITOR
    printf("  : ");
    sprintf(edbuff, "%04d/%02d/%02d", year, month, day);
    EdLineRead(edline, edbuff, LINE_MAXLENGTH);
    sscanf(edbuff, "%d/%d/%d", &year, &month, &day);
#else
    printf(" (%04d/%02d/%02d): ", year, month, day);
    scanf("%d/%d/%d", &year, &month, &day);
    putchar('\n');
#endif

    if (year < 1970 || year > 2038) {
        printf("Bad year: %d is not within range 1970..2038\n", year);
        return -1;
    }
    if (month < 1 || month > 12) {
        printf("Bad month: %d is not within range 1..12\n", month);
        return -1;
    }
    if (day < 1 || day > 31) {
        printf("Bad day: %d is not within range 1..31\n", day);
        return -1;
    }
    stm->tm_year = year - 1900;
    stm->tm_mon = month - 1;
    stm->tm_mday = day;

    return 0;
}

/*
 * Query time of day from user.
 *
 * Returns 0 on success, -1 otherwise.
 */
static int EnterTime(struct _tm *stm)
{
    int hour = stm->tm_hour;
    int minute = stm->tm_min;
    int second = stm->tm_sec;

    printf("Enter time, use 24h format HH:MM:SS");
#ifdef USE_LINE_EDITOR
    printf(": ");
    sprintf(edbuff, "%02d:%02d:%02d", hour, minute, second);
    EdLineRead(edline, edbuff, LINE_MAXLENGTH);
    sscanf(edbuff, "%d:%d:%d", &hour, &minute, &second);
#else
    printf(" (%02d:%02d:%02d): ", hour, minute, second);
    scanf("%d:%d:%d", &hour, &minute, &second);
    putchar('\n');
#endif

    if (hour < 0 || hour > 23) {
        printf("Bad hour: %d is not within range 0..23\n", hour);
        return -1;
    }
    if (minute < 0 || minute > 59) {
        printf("Bad minute: %d is not within range 0..59\n", minute);
        return -1;
    }
    if (second < 0 || second > 59) {
        printf("Bad second: %d is not within range 0..59\n", second);
        return -1;
    }
    stm->tm_hour = hour;
    stm->tm_min = minute;
    stm->tm_sec = second;

    return 0;
}

/*
 * Query time difference from user, specified in hours and minutes.
 *
 * 'init' specifies the default number of seconds.
 *
 * Returns the number of seconds.
 */
static long EnterTimeDiff(long init)
{
    long hours;
    long minutes;
    long seconds;

    seconds = init;
    minutes = seconds / 60L;
    hours = minutes / 60L;
    minutes = abs(minutes % 60L);

    printf("Enter time difference in format HH:MM");
#ifdef USE_LINE_EDITOR
    printf(": ");
    sprintf(edbuff, "%+03ld:%02ld", hours, minutes);
    EdLineRead(edline, edbuff, LINE_MAXLENGTH);
    sscanf(edbuff, "%ld:%ld", &hours, &minutes);
#else
    printf(" (%+03ld:%02ld): ", hours, minutes);
    scanf("%ld:%ld", &hours, &minutes);
    putchar('\n');
#endif

    if (hours < -12 || hours > 12) {
        printf("\nBad hours: %ld is not within range -12..+12\n", hours);
        return init;
    }
    if (minutes < 0 || minutes > 59) {
        printf("\nBad minutes: %ld is not within range 0..59\n", minutes);
        return init;
    }
    return (hours * 60L + minutes) * 60L;
}

/*
 * Display the elapsed seconds of the epoch.
 *
 * The value is constantly updated until the user presses a key.
 */
static void DisplaySeconds(void)
{
    uint_fast8_t i = 0;

    while (!kbhit()) {
        printf(" [%c] Seconds since epoch: %ld\r"
               , rotor[++i & 3]
               , (long) time(NULL));
        NutSleep(200);
    }
    putchar('\n');
}

/*
 * Display the coordinated universal time.
 *
 * The value is constantly updated until the user presses a key.
 */
static void DisplayZuluTime(void)
{
    time_t secs;
    struct _tm *gmt;
    uint_fast8_t i = 0;

    while (!kbhit()) {
        secs = time(NULL);
        gmt = gmtime(&secs);
        printf(" [%c] Universal time: ", rotor[++i & 3]);
        PrintDateTime(gmt);
        printf("    \r");
        NutSleep(500);
    }
    putchar('\n');
}

/*
 * Display the local time.
 *
 * The value is constantly updated until the user presses a key.
 */
static void DisplayLocalTime(void)
{
    time_t tt;
    struct _tm *ltm;
    uint_fast8_t i = 0;

    while (!kbhit()) {
        /* Get local time and print it. */
        tt = time(NULL);
        ltm = localtime(&tt);
        printf(" [%c] Local time: ", rotor[++i & 3]);
        PrintDateTime(ltm);

        /* Calculate the offset from UTC in minutes. */
        tt = _timezone;
        if (ltm->tm_isdst > 0) {
            tt += _dstbias;
        }
        tt /= -60L;

        /* Print UTC offset in format HH:MM. */
        printf(" UTC%+03ld:%02ld   \r", tt / 60L, abs(tt) % 60L);
        NutSleep(200);
    }
    putchar('\n');
}

/*
 * Display system up time.
 *
 * The value is constantly updated until the user presses a key.
 */
static void DisplayUpTime(void)
{
    uint32_t hours;
    uint32_t minutes;
    uint32_t seconds;
    uint_fast8_t i = 0;

    while (!kbhit()) {
        seconds = NutGetSeconds();
        minutes = seconds / 60UL;
        hours = minutes / 60UL;
        minutes %= 60UL;
        seconds %= 60UL;
        printf(" [%c] System is running %lu hours"
               ", %lu minutes and %lu seconds   \r"
               , rotor[++i & 3], hours, minutes, seconds);
        NutSleep(500);
    }
    putchar('\n');
}

/*
 * Display the week day of a queried calendar date.
 *
 * mktime() updates the structure members tm_yday and tm_wday.
 * This can be used to determine the week day name of any given
 * date.
 */
static void CalcWeekDay(void)
{
    struct _tm date;
    time_t secs;

    /* Use current local time as default. */
    time(&secs);
    memcpy(&date, localtime(&secs), sizeof(date));
    /* Query date and print week day name if we got a valid entry. */
    if (EnterDate(&date) == 0) {
        mktime(&date);
        puts(weekday_name[date.tm_wday]);
    }
}

/*
 * Query user for a new time zone offset.
 */
static void SetTimeZone(void)
{
    /* Nut/OS uses a global variable to store the current TZ offset. */
    _timezone = EnterTimeDiff(_timezone);
}

/*
 * Query user for a new system time.
 */
static void SetLocalTime(void)
{
    struct _tm ltm;
    time_t now;

    /* Use current local time as default. */
    time(&now);
    memcpy(&ltm, localtime(&now), sizeof(ltm));

    /* Query date and time. */
    if (EnterDate(&ltm) == 0 && EnterTime(&ltm) == 0) {
        /* Let mktime determine whether DST is in effect. */
        ltm.tm_isdst = -1;
        /* mktime expects local time and returns seconds since the epoch. */
        now = mktime(&ltm);
        /* stime expects seconds since the epoch. */
#if defined(NUT_USE_OLD_TIME_API)
        stime(&now);
#else
        if (NutRtcSetTime(gmtime(&now)) == 0) {
            puts("Setting RTC Time from Build date");
        }
        if (NutRtcGetTime(&ltm) == 0) {
            time_t now = _mkgmtime(&ltm);
            if (now != -1) {
                stime(&now);
                puts("Setting System Time from RTC");
            }
        }
#endif
    }
}

/*
 * Application entry.
 */
int main(void)
{
    uint32_t baud = 115200;
    int cmd;
    time_t now;
    time_t secs;
    struct _tm *gmt;

    /* Use UART device for stdin and stdout. */
    NutRegisterDevice(&DEV_CONSOLE, 0, 0);
    freopen(DEV_CONSOLE.dev_name, "w", stdout);
    freopen(DEV_CONSOLE.dev_name, "r", stdin);
    _ioctl(_fileno(stdout), UART_SETSPEED, &baud);
    printf("\n\nCalendar Time %s running on Nut/OS %s\n"
           , version, NutVersionString());

#ifdef USE_LINE_EDITOR
    /* Open line editor, if configured. */
    printf("Opening line editor...");
    edline = EdLineOpen(EDIT_MODE_ECHO);
    if (edline) {
        puts("OK");
        puts("Use ^H for Delete");
    } else {
        puts("failed");
    }
#else
    puts("Note: Enable local echo!");
#endif

#if USE_TIME_ZONE
    _timezone = USE_TIME_ZONE;
#endif

#if defined(USE_BUILD_TIME)
    now = RfcTimeParse("Unk, " __DATE__ " " __TIME__);
    now -= (CRT_TIMEZONE - CRT_DAYLIGHT) * 60;
    printf("Built " __DATE__ " " __TIME__ " TZ OFFSet %d CRT_DAYLIGHT %d\n",
           CRT_TIMEZONE,  CRT_DAYLIGHT);
#else
    now = 0;
#endif
#ifdef RTC_CHIP
    /* Register and query hardware RTC, if available. */
    printf("Registering RTC hardware...");
    if (NutRegisterRtc(&RTC_CHIP)) {
        puts("failed");
    } else {
        uint32_t rtc_stat;

        puts("OK");
        NutRtcGetStatus(&rtc_stat);
        if (rtc_stat & RTC_STATUS_PF) {
            puts("Power failure");
#if defined(USE_BUILD_TIME)
#if defined(NUT_USE_OLD_TIME_API)
            puts("Setting Time from Build date");
            /* Initially use the compile date and time. */
            stime(&now);
            puts("Built " __DATE__ " " __TIME__);
#else
            struct _tm gmt;
            if (NutRtcSetTime(gmtime(&now)) == 0) {
                puts("Setting RTC Time from Build date");
            }
            if (NutRtcGetTime(&gmt) == 0) {
                time_t now = _mkgmtime(&gmt);
                if (now != -1) {
                    stime(&now);
                    puts("Setting System Time from RTC");
                }
            }
#endif
#endif
        } else {
#if defined(NUT_USE_OLD_TIME_API)
            puts("OK");
#else
            struct _tm gmt;
            if (NutRtcGetTime(&gmt) == 0) {
                now = _mkgmtime(&gmt);
                if (now != -1) {
                    stime(&now);
                    puts("Setting system time from RTC");
                }
            }
#endif
        }
        if (rtc_stat & RTC_STATUS_INACCURATE) {
            uint32_t rtc_run_status;
            NutRtcGetStatus(&rtc_run_status);
            if (rtc_run_status & RTC_STATUS_INACCURATE)
                puts("RTC running from inaccurate source");
            else
                puts("RTC may have lost clock ticks");
        }

    }
#elif USE_BUILD_TIME
    {
        /* Initially use the compile date and time. */
        stime(&now);
    }
#endif

    secs = time(NULL);
    gmt = gmtime(&secs);
    printf("Universal time: ");
    PrintDateTime(gmt);
    printf("\r");
    for (;;) {
        /* Print command menu. */
        puts("\n  0 - Display seconds counter");
        puts("  1 - Display universal time");
        puts("  2 - Display local time");
        puts("  3 - Display system uptime");
#ifdef RTC_CHIP
        puts("  A - Calculate RTC Alarm A in 10 seconds");
        puts("  B - Calculate RTC Alarm B in 10 seconds");
#endif
        puts("  C - Calculate weekday");
        puts("  S - Set local time");
        puts("  Y - Toggle DST calculation");
        puts("  Z - Set timezone");

        printf("What is thy bidding, my master? ");

        /* Flush input buffer. */
        while (kbhit()) {
            cmd = getchar();
        }

        /* Get the next command. */
        cmd = getchar();
        putchar('\n');

        /* Process the command. */
        switch (cmd) {
        case '0':
            DisplaySeconds();
            break;
        case '1':
            DisplayZuluTime();
            break;
        case '2':
            DisplayLocalTime();
            break;
        case '3':
            DisplayUpTime();
            break;
#ifdef RTC_CHIP
        case 'a':
        case 'A':
        case 'b':
        case 'B':
            Alarm(tolower(cmd) - 'a');
            break;
#endif
        case 'C':
        case 'c':
            CalcWeekDay();
            break;
        case 'S':
        case 's':
            SetLocalTime();
            break;
        case 'Y':
        case 'y':
            /* Nut/OS uses a global variable to enable/disable DST.
               Toggle the current status and display the result. */
            _daylight = _daylight == 0;
            printf("DST calculation %sabled\n", _daylight ? "en" : "dis");
            break;
        case 'Z':
        case 'z':
            SetTimeZone();
            break;
        }
    }
    return 0;
}
