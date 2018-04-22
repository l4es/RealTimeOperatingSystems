/*!
 * Copyright (C) 2013-2015 Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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

#include <cfg/crt.h>    /* Floating point configuration. */

#include <string.h>
#include <stdio.h>
#include <io.h>

#include <dev/board.h>
#include <sys/timer.h>
#include <sys/thread.h>
#include <sys/event.h>
#include <dev/gpio.h>
#include <dev/i2cbus.h>

static const char *banner = "\nNut/OS I2C Sample on " BOARDNAME
    " " __DATE__ " " __TIME__;

#if defined(DEV_I2CBUS)
static char inbuf[128];
#endif

#define MAX_TEST_DEVICES 16
int found_device[MAX_TEST_DEVICES];

#if defined(LED1_PORT) && defined( LED1_PIN)
#define LED1_INIT GpioPinConfigSet( LED1_PORT, LED1_PIN, GPIO_CFG_OUTPUT)
#define LED1_TOGGLE  GpioPinSet(LED1_PORT, LED1_PIN, \
                   !(GpioPinGet(LED1_PORT, LED1_PIN)))
#else
#define LED1_INIT
#define LED1_TOGGLE
#endif

#if defined(LED2_PORT) && defined( LED2_PIN)
#define LED2_START_THREAD                           \
    if (NutThreadCreate("t2", LedBlink, 0, 512)== 0)\
        puts("Can't create LED thread\n");        \
    else                                            \
        puts("LED thread started\n");

THREAD(LedBlink, arg)
{
    GpioPinConfigSet( LED2_PORT, LED2_PIN, GPIO_CFG_OUTPUT);
    for(;;)
    {
        NutSleep(100);
        GpioPinSetHigh(LED2_PORT, LED2_PIN);
        NutSleep(100);
        GpioPinSetLow(LED2_PORT, LED2_PIN);
    }
}
#else
#define LED2_START_THREAD
#endif

void Hardware_Init(void)
{
#if defined(F4_DISCOVERY)
/* Set all CS43L22 I2C relates pins to a safe state*/
    GpioPinConfigSet(NUTGPIO_PORTE, 3, GPIO_CFG_OUTPUT);
    GpioPinSetHigh(NUTGPIO_PORTE, 3);
    GpioPinConfigSet(NUTGPIO_PORTC, 7, GPIO_CFG_OUTPUT);
    GpioPinSetHigh(NUTGPIO_PORTC, 7);
    GpioPinConfigSet(NUTGPIO_PORTC, 10, GPIO_CFG_OUTPUT);
    GpioPinSetHigh(NUTGPIO_PORTC, 10);
    GpioPinConfigSet(NUTGPIO_PORTC, 12, GPIO_CFG_OUTPUT);
    GpioPinSetHigh(NUTGPIO_PORTC, 12);
    GpioPinConfigSet(NUTGPIO_PORTC, 14, GPIO_CFG_OUTPUT);
    GpioPinSetHigh(NUTGPIO_PORTC, 14);
    GpioPinConfigSet(AUDIO_RST_PORT, AUDIO_RST_PIN, GPIO_CFG_OUTPUT);
    GpioPinSetLow(AUDIO_RST_PORT, AUDIO_RST_PIN);
    NutSleep(11);
    GpioPinSetHigh(AUDIO_RST_PORT, AUDIO_RST_PIN);
    NutSleep(10);
#endif
}

#ifndef I2C_SLA_MAX44009
#define I2C_SLA_MAX44009  0x4a
#endif

/*!
 * \brief I2C driver control block.
 */
static NUTI2C_SLAVE i2cMax44009 = {
    NULL,
    I2C_SLA_MAX44009,
    3000,
    NULL
};

int dump_max44009(NUTI2C_BUS *bus)
{
#define MAX44009_CFG      0x2
    int res;
    uint8_t reg_addr, cfg, lux_h, lux_l;
    int lux;
    if (i2cMax44009.slave_bus == 0)
    {
        res = NutRegisterI2cSlave(&i2cMax44009, bus);
        if (res)
            return -1;
    }
    reg_addr = MAX44009_CFG;
    res = NutI2cMasterTransceive(&i2cMax44009, &reg_addr, 1, &cfg, 1);
    if (res < 0)
        return -1;
    reg_addr++;
    res = NutI2cMasterTransceive(&i2cMax44009, &reg_addr, 1, &lux_h, 1);
    if (res < 0)
        return -1;
    reg_addr++;
    /* we should read lux_l after lub_h without sending stop.
     * As of 2013, Feb 13, the Ethernut I2CBus API cant do taht
     */
    res = NutI2cMasterTransceive(&i2cMax44009, &reg_addr, 1, &lux_l, 1);
    if (res < 0)
        return -1;
    lux = ((lux_h & 0xf) * 720)<<((lux_h & 0xf0)>>4);
    printf("MAX44009: Lux  %6d.%03d cfg %02x lux_h %02x lux_l %02x\n",
           lux/1000, lux %1000,cfg, lux_h, lux_l);
    return 0;
}

#define CS43L22_AUTO_INC        0x80

#ifndef I2C_SLA_CS43L22
#define I2C_SLA_CS43L22   0x4a
#endif

/*!
 * \brief I2C driver control block.
 */
static NUTI2C_SLAVE i2cCs43l22 = {
    NULL,
    I2C_SLA_CS43L22,
    3000,
    NULL
};

int dump_cs43l22(NUTI2C_BUS *bus)
{
#define CS43L22_DATA_LEN      0x34
    int i, res;
    uint8_t data[CS43L22_DATA_LEN];
    if (i2cCs43l22.slave_bus == 0)
    {
        res = NutRegisterI2cSlave(&i2cCs43l22, bus);
        if (res)
            return -1;
    }
    data[0] = CS43L22_AUTO_INC;
    res = NutI2cMasterTransceive(&i2cCs43l22, data, 1, data, CS43L22_DATA_LEN);
    for (i=0; i<CS43L22_DATA_LEN; i++)
    {
        if ((i & 0xf) == 0)
            printf(" Addr %02x:", i);
        printf(" %02x", data[i]);
        if ((i & 0xf) == 0xf)
            puts("");
    }
    if ((i & 0xf) != 0)
        puts("");
    return 0;
}

#define LMS303_AUTO_INC        0x80

#ifndef I2C_SLA_LSM303_ACCEL
#define I2C_SLA_LSM303_ACCEL    0x19
#endif

/*!
 * \brief I2C driver control block.
 */
static NUTI2C_SLAVE i2cLsm303_accel = {
    NULL,
    I2C_SLA_LSM303_ACCEL,
    3000,
    NULL
};

int dump_lsm303_accel(NUTI2C_BUS *bus)
{
#define LSM303_ACCEL_DATA_LEN  0x6
#define LSM303_ACCEL_DATA      0x28
    uint8_t data[LSM303_ACCEL_DATA_LEN];

    int  res;
    int16_t ax, ay, az;

    if (i2cLsm303_accel.slave_bus == 0)
    {
        res = NutRegisterI2cSlave(&i2cLsm303_accel, bus);
        if (res)
            return -1;
        /* Enable Acceleration data Output with 1 Hz*/
        data[0] = 0x20;
        data[1] = 0x17;
        res = NutI2cMasterTransceive(&i2cLsm303_accel, data, 2, 0, 0);
    }
    data[0] = LMS303_AUTO_INC |LSM303_ACCEL_DATA;
    res = NutI2cMasterTransceive(&i2cLsm303_accel, data, 1, data,
                                 LSM303_ACCEL_DATA_LEN);
    if (res == -1)
        return -1;

    ax = (int16_t)(data[1]<< 8 | data[0]);
    ay = (int16_t)(data[3]<< 8 | data[2]);
    az = (int16_t)(data[5]<< 8 | data[4]);

    printf("Accel:  ax %6d ay %6d az %6d\n", ax, ay, az);
    return 0;
}

#ifndef I2C_SLA_LSM303_MAGNET
#define I2C_SLA_LSM303_MAGNET   0x1e
#endif

/*!
 * \brief I2C driver control block.
 */
static NUTI2C_SLAVE i2cLsm303_magnet = {
    NULL,
    I2C_SLA_LSM303_MAGNET,
    3000,
    NULL
};

int dump_lsm303_magnet(NUTI2C_BUS *bus)
{
#define LSM303_MAGNET_DATA_LEN  0x6
#define LSM303_MAGNET_DATA      0x3
#define LSM303_MAGNET_DATA      0x3
#define LSM303_MAGNET_TEMP_DATA 0x31
#define LSM303_MAGNET_TEMP_DATA_LEN  0x2
    uint8_t data[LSM303_MAGNET_DATA_LEN];

    int  res;
    int16_t ax, ay, az, temp;

    if (i2cLsm303_magnet.slave_bus == 0)
    {
        res = NutRegisterI2cSlave(&i2cLsm303_magnet, bus);
        if (res)
            return -1;
        data[0] = 0x0; /* Start writing at XRA_REG_M*/
        data[1] = 0x80;/* Enable Magnet Output 0.75 Hz and Temperature*/
        data[2] = 0x20;/* Max sensivity*/
        data[3] = 0x00;/* Continous conversion*/

        res = NutI2cMasterTransceive(&i2cLsm303_magnet, data, 4, 0, 0);
    }
    data[0] = LMS303_AUTO_INC |LSM303_MAGNET_DATA;
    res = NutI2cMasterTransceive(&i2cLsm303_magnet, data, 1, data,
                                 LSM303_MAGNET_DATA_LEN);
    if (res == -1)
        return -1;

    ax = (int16_t)(data[0]<< 8 | data[1]);
    az = (int16_t)(data[2]<< 8 | data[3]);
    ay = (int16_t)(data[4]<< 8 | data[5]);

    printf("Magnet: ax %6d ay %6d az %6d,", ax, ay, az);

    data[0] = LMS303_AUTO_INC |LSM303_MAGNET_TEMP_DATA;
    res = NutI2cMasterTransceive(&i2cLsm303_magnet, data, 1, data,
                                 LSM303_MAGNET_TEMP_DATA_LEN);
    if (res == -1)
        return -1;
    temp = (int16_t)(data[0]<< 8 | data[1]);
    temp = temp >> 4;
    temp = temp *125;
    printf(" temp  %4d.%03d\n", temp/1000, temp%1000);
    return 0;
}

#ifndef I2C_SLA_SHT21
#define I2C_SLA_SHT21   0x40
#endif

/*!
 * \brief I2C driver control block.
 */
static NUTI2C_SLAVE i2cSht21 = {
    NULL,
    I2C_SLA_SHT21,
    3000,
    NULL
};

int dump_sht21(NUTI2C_BUS *bus)
{
    int i, res;
    int rh, temp;
    uint8_t data[2], raw0[8], raw1[6];
    uint32_t rh_raw, temp_raw;
    if (i2cSht21.slave_bus == 0)
    {
        res = NutRegisterI2cSlave(&i2cSht21, bus);
        if (res)
            return -1;
        /* Read serial according to
           Sensirion_Humidity_SHT2x_Electronic_Identification_Code_V1.1.pdf*/
        /* First Memory access */
        data[0] = 0xfa;
        data[1] = 0x0f;
        res = NutI2cMasterTransceive(&i2cSht21, data, 2, raw0, 8);
        if (res < 0)
            return -1;
        /* Second Memory access */
        data[0] = 0xfc;
        data[1] = 0xc9;
        res = NutI2cMasterTransceive(&i2cSht21, data, 2, raw1, 6);
        if (res < 0)
            return -1;
        printf("Serial: %02x%02x%02x%02x%02x%02x%02x%02x,",
               raw1[3], raw1[4], raw0[0], raw0[2], raw0[4], raw0[6],
               raw1[0], raw1[1]);
        for (i = 0; i<8; i++)
            printf(" %02x", raw0[i]);
        for (i = 0; i<6; i++)
            printf(" %02x", raw1[i]);
        puts("\n");
        /* Default is highest resolution. no need to change*/
    }

    data[0] = 0xf5; /*Start RH measurement, no hold*/
    res = NutI2cMasterTransceive(&i2cSht21, data, 1, 0, 0);
    /* Poll for measurement complete*/
    do
    {
        NutSleep(1);
        res = NutI2cMasterTransceive(&i2cSht21, 0, 0, raw0, 3);
    } while(res < 0);
    if(res != 3)
        printf("Short Read: Read %d bytes vs 3 bytes requested\n", res);

    data[0] = 0xf3; /*Start Temp measurement, no hold*/
    res = NutI2cMasterTransceive(&i2cSht21, data, 1, 0, 0);
    do
    {
        NutSleep(5);
        res = NutI2cMasterTransceive(&i2cSht21, 0, 0, raw1, 3);
    } while(res < 0);
    if(res != 3)
        printf("Short Read: Read %d bytes vs 3 bytes requested\n", res);
    rh_raw = raw0[0]<<8|raw0[1];
    temp_raw = raw1[0]<<8|raw1[1];

    /* Calculation according Sensirion_Humidity_SHT21_Datasheet_V3.pdf
     * in units of 10**-3
     */
    rh   = (rh_raw   * (125000/2))/(0x10000/2) -6000;
    temp = (temp_raw * (175720/4))/(0x10000/4) -46850;

    printf("SHT21   : Temp %6d.%03d , Rh %2d.%03d%%\n",
           temp/1000, temp%1000, rh/1000, rh%1000);

    return 0;
}

int ScanBus(NUTI2C_BUS *bus)
{
    int res = 1, i;
    int sla =0;
    for (i=0; i<MAX_TEST_DEVICES-1; i++) {
        sla = NutI2cBusScan(bus, sla, 127);
        if (sla == I2C_SLA_NONE) {
            found_device[i] = 0;
            break;
        }
        found_device[i] = sla;
        printf("%3d (0x%02x) detected\n", sla, sla);
        sla++;
    }
    found_device[i] = 0;
    return res;
}

/*
 * I2C sample: Scan the I2C Bus and look for connected devices
 *
 * Some functions do not work with ICCAVR.
 */
int main(void)
{
    int res;
    uint32_t baud = 115200;

    res = NutRegisterDevice(&DEV_CONSOLE, 0, 0);
    if (res )
        goto error;

    freopen(DEV_CONSOLE.dev_name, "w", stdout);
    freopen(DEV_CONSOLE.dev_name, "r", stdin);
    _ioctl(_fileno(stdout), UART_SETSPEED, &baud);

    puts(banner);

    LED1_INIT;
    LED2_START_THREAD;
    Hardware_Init();

#if !defined(DEV_I2CBUS)
    puts("Please indicate the I2C Bus to scan!");
    goto error;
#else
    res = NutI2cBusRate( &DEV_I2CBUS, 100000);
    if (res !=0)
    {
        puts("NutI2cBusRate failed\n");
        goto error;
    }
    else
    {
        printf("NutI2cBusRate success\n");
    }

    NutI2cBusTimeout(&DEV_I2CBUS, 10);
    ScanBus(&DEV_I2CBUS);

    for (;;) {
        int i;

        LED1_TOGGLE;
        for (i = 0; found_device[i]; i++)
        {
            switch(found_device[i])
            {
#if defined(F3_DISCOVERY)
            case I2C_SLA_LSM303_ACCEL: /* ST LSM303 Accel on F3Discovery*/
                dump_lsm303_accel(&DEV_I2CBUS);
                break;
            case I2C_SLA_LSM303_MAGNET: /* ST LSM303 Magnet on F3Discovery*/
                dump_lsm303_magnet(&DEV_I2CBUS);
                break;
#endif
#if defined(F4_DISCOVERY)
            case I2C_SLA_CS43L22: /* Cirrus CS43L22 on F4Discovery, AD0 == 0*/
                dump_cs43l22(&DEV_I2CBUS);
                break;
#endif
#if defined(STM32_CAN)
            case I2C_SLA_MAX44009: /* MAX44009 on STM32_CAN  AD0 == 0*/
                dump_max44009(&DEV_I2CBUS);
                break;
            case I2C_SLA_SHT21: /* SHT21 on STM32_CAN */
                dump_sht21(&DEV_I2CBUS);
                break;
#endif
            }
        }
        puts("Press \"Enter\" to scan I2C bus for devices ");
        fgets(inbuf, sizeof(inbuf), stdin);
    }
    return 0;
#endif

error:
    while(1)
    {
        LED1_TOGGLE;
        NutSleep(100);
    }
    return 0;
}
