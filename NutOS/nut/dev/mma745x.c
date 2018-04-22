/*
 * Copyright (C) 2010 by Rittal GmbH & Co. KG,
 * Dawid Sadji <sadji.d@rittal.de> All rights reserved.
 * Ulrich Prinz <prinz.u@rittal.de> All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY EMBEDDED IT AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL EMBEDDED IT
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 *
 */

/*
 * \file dev/mma745x.c
 * \brief Driver for Freescale MMA745x velocity sensor.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */
#include <string.h>
#include <stdlib.h>

#include <compiler.h>

#include <cfg/os.h>

#include <sys/heap.h>
#include <sys/timer.h>
#include <sys/event.h>

#include <dev/gpio.h>
#include <dev/twif.h>

#include <dev/mma745x.h>

//#define MMA_DEBUG

#ifdef MMA_DEBUG
    #include <stdio.h>
    #define MPRINTF printf
    #define FFLUSH fflush
#else
    #define MPRINTF(...)
    #define FFLUSH(...)
#endif

#define SGN10BIT 0x0200
#define Con2Cpl( v) if (v & SGN10BIT) { v ^= ~SGN10BIT; v += SGN10BIT; }

const mmaInit_t mmaDefault = {
    /* rMODE */     MMA745X_MODE,
    /* rINTRST */   0x00,
    /* rCONTROL1 */ 0x00 /*MMA745x_CTL1_THOPT*/,
    /* rCONTROL2 */ 0x00,
    /* rLEVEL */    127, /* 128/8x8g = 127 == DISABLED */
    /* rPVALUE */    24, /* 128/8x1,5g = 24*/
    /* rPDUR */     100, /* x0.5ms = 50ms */
    /* rLATTV */      0, /* x1.0ms =  0ms */
    /* rTW */         0, /* x1.0ms =  0ms */
};

/*! brief MMA745x configuration struct.
 */
static mmaInit_t *mmaCfg;
static mma8bit_t *mmaForce;


/*! brief write to MMA7455L via I2C.
 *
 * Write one byte to a register in the sensor.
 * \para reg Register in sensor to address.
 * \para val Pointer to value to write.
 *
 * \return 0 if success, -1 on error.
 */
int Mma745xWrite( uint_fast8_t reg, void *val, size_t len)
{
    int ret;
    ret = TwMasterRegWrite(I2C_SLA_MMA745x, reg, 1, val, len, 500);
    return (ret == len)?0:-1;
}

/*! brief read from MMA7455L via I2C.
 *
 * Read one byte from a register in the sensor.
 * \para reg Register in sensor to address.
 * \para val Pointer to store the value.
 *
 * \return 0 if success, -1 on error.
 */
int Mma745xRead( uint_fast8_t reg, void *val, size_t len)
{
    int ret;
    ret = TwMasterRegRead(I2C_SLA_MMA745x, reg, 1, val, len, 500);
    return (ret == len)?0:-1;
}

/*! bief read all axes of the sensor as raw value.
 *
 * This function read the current sensor values.
 * Depending on the setup the values are in range of:
 * 2g Mode: -2g..0g..2g -> 0x80..0x00..0x7f
 * 4g Mode: -4g..0g..4g -> 0x80..0x00..0x7f
 * 8g Mode: -8g..0g..8g -> 0x80..0x00..0x7f
 *
 * para Pointer to mma8bit_t struct to store results in.
 * result 0 if request was successfull, else -1;
 */
int Mma745xReadVal8( mma8bit_t *val)
{
    return Mma745xRead(MMA745x_REG_XOUT8, val, sizeof(mma8bit_t));
}

/*! bief read all axes of the sensor as raw value.
 *
 * This function read the current sensor values.
 * Depending on the setup the values are in range of:
 * 8g Mode: -8g..0g..8g -> 0x0200..0x0000..0x01ff
 *
 * para Pointer to mma8bit_t struct to store results in.
 * result 0 if request was successfull, else -1;
 */
int Mma745xReadVal10( uint8_t ofs, mma10bit_t *val)
{
    int ret;
    ret = Mma745xRead( ofs, val, sizeof(mma10bit_t));
    /* 2's complement to int conversion */
    Con2Cpl(val->x);
    Con2Cpl(val->y);
    Con2Cpl(val->z);
    return ret;
}

int Mma745xReadG( mma10bit_t *val)
{
    int ret;
    ret = Mma745xReadVal8( mmaForce);

    switch (mmaCfg->rMODE & MMA745X_MCTL_GLVL_MSK) {
        case MMA745X_MCTL_GLVL_8G:
            val->x = ((int16_t)mmaForce->x*100/16);
            val->y = ((int16_t)mmaForce->y*100/16);
            val->z = ((int16_t)mmaForce->z*100/16);
            break;
        case MMA745X_MCTL_GLVL_4G:
            val->x = ((int16_t)mmaForce->x*100/32);
            val->y = ((int16_t)mmaForce->y*100/32);
            val->z = ((int16_t)mmaForce->z*100/32);
            break;
        case MMA745X_MCTL_GLVL_2G:
            val->x = ((int16_t)mmaForce->x*100/64);
            val->y = ((int16_t)mmaForce->y*100/64);
            val->z = ((int16_t)mmaForce->z*100/64);
            break;
    }
    return ret;
}

int Mma745xReadCal( mma10bit_t *cal)
{
    int ret;
    ret = Mma745xReadVal10(MMA745x_REG_XOFFL, cal);
    return ret;
}

int Mma745xWriteCal( mma10bit_t *cal)
{
    int ret;
    ret = Mma745xWrite(MMA745x_REG_XOFFL, cal, sizeof(mma10bit_t));
    return ret;
}

int Mma745xCtl( uint_fast8_t fkt, void *val)
{
    int ret = -1;
    uint8_t irqs;
    MPRINTF("MCTL(%u,%p)\n", fkt, val);

    switch (fkt) {
        case MMA_GET_STATE:
            ret = Mma745xRead(MMA745x_REG_STATUS, val, 2);
            break;
        case MMA_SET_MODE:
            ret = Mma745xWrite(MMA745x_REG_MCTL, val, 1);
            if (ret == 0)
                mmaCfg->rMODE = *(uint8_t*)val;
            break;
        case MMA_GET_IRQ:
            /* Get pending IRQS */
            ret = Mma745xRead(MMA745x_REG_DETSRC, val, 1);
            break;
#if 0
        case MMA_SET_IRQ:
            /* Enable IRQs given in val */
            irqs = (*(uint8_t*)val) & MMA745x_INTRST_MSK;
            ret = Mma745xWrite(MMA745x_REG_INTRST, &irqs, 1);
            break;
#endif
        case MMA_CLR_IRQ:
            /* Clear IRQs given in val */
            irqs = (*(uint8_t*)val) & MMA745x_INTRST_MSK;
            ret = Mma745xWrite(MMA745x_REG_INTRST, &irqs, 1);
            irqs ^= MMA745x_INTRST_MSK;
            ret = Mma745xWrite(MMA745x_REG_INTRST, &irqs, 1);
            break;
    }
    return ret;
}

/*! brief MMA7455L Initialization
 *
 * Configure GPIO connections and preset
 * MMA7455L registers.
 *
 * \return 0 if success, -1 on error.
 */
int Mma745xInit( uint_fast8_t selftest, mmaInit_t *init)
{
    int ret = 0;
    uint8_t ctl = 0;

    MPRINTF("Init MMA... ");

    mmaCfg = NutHeapAlloc(sizeof(mmaInit_t));
    if (mmaCfg==NULL)
        return -1;
    mmaForce = NutHeapAlloc(sizeof(mma8bit_t));
    if (mmaForce==NULL)
        return -1;

    /* Configure INT1/DRDY and INT2 lines from chip */
#if defined(MMA745X_IRQ1_PORT) && defined(MMA745X_IRQ1_PIN)
    GpioPinConfigSet(MMA745X_IRQ1_PORT, MMA745X_IRQ1_PIN, GPIO_CFG_PULLUP);
#endif
#if defined(MMA745X_IRQ2_PORT) && defined(MMA745X_IRQ2_PIN)
    GpioPinConfigSet(MMA745X_IRQ2_PORT, MMA745X_IRQ2_PIN, GPIO_CFG_PULLUP);
#endif

    /* First time initialization of the chip */
    if (init)
        memcpy( mmaCfg, init, sizeof(mmaInit_t));
    else
        memcpy( mmaCfg, &mmaDefault, sizeof(mmaInit_t));
    Mma745xWrite(MMA745x_REG_MCTL, mmaCfg, sizeof(mmaInit_t));

    /* Check if selftest should be done on startup */
    if (selftest) {
        // TODO: Selftest is badly described in docs. Try to figure out how it works...
        ctl = MMA745X_MCTL_STON;
        Mma745xWrite(MMA745x_REG_MCTL, &ctl, 1);
    }

    return ret;
}


