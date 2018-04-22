#ifndef _DEV_MMA745X_H_
#define _DEV_MMA745X_H_
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
 * \file dev/mma745x.h
 * \brief Driver for Freescale MMA745x velocity sensor.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#ifndef I2C_SLA_MMA745x
#define I2C_SLA_MMA745x     0x1D
#endif

/*
 * MMA745x Register Map.
 */
#define MMA745x_REG_XOUTL       0x00        /**< ro: Value X-Axis 10 bit resolution LSB */
#define MMA745x_REG_XOUTH       0x01        /**< ro: Value X-Axis 10 bit resolution MSB */
#define MMA745x_REG_YOUTL       0x02        /**< ro: Value Y-Axis 10 bit resolution LSB */
#define MMA745x_REG_YOUTH       0x03        /**< ro: Value Y-Axis 10 bit resolution MSB */
#define MMA745x_REG_ZOUTL       0x04        /**< ro: Value Z-Axis 10 bit resolution LSB */
#define MMA745x_REG_ZOUTH       0x05        /**< ro: Value Z-Axis 10 bit resolution MSB */

#define MMA745x_REG_XOUT8       0x06        /**< ro: Value X-Axis 8 bit resolution */
#define MMA745x_REG_YOUT8       0x07        /**< ro: Value Y-Axis 8 bit resolution */
#define MMA745x_REG_ZOUT8       0x08        /**< ro: Value Z-Axis 8 bit resolution */

#define MMA745x_REG_STATUS      0x09        /**< ro: Status register */
#define MMA745x_REG_DETSRC      0x0A        /**< ro: Detection source register */

#define MMA745x_REG_TOUT        0x0B        /**< ro: Optional temperature output register */
#define MMA745x_REG_I2CAD       0x0D        /**< rw: I2C device address register */
#define MMA745x_REG_USRINF      0x0E        /**< ro: Optional user information register */
#define MMA745x_REG_WHOAMI      0x0E        /**< ro: Optional chip ID register */

#define MMA745x_REG_XOFFL       0x10        /**< X-Axis offset drift register LSB */
#define MMA745x_REG_XOFFH       0x11        /**< X-Axis offset drift register MSB */
#define MMA745x_REG_YOFFL       0x12        /**< Y-Axis offset drift register LSB */
#define MMA745x_REG_YOFFH       0x13        /**< Y-Axis offset drift register MSB */
#define MMA745x_REG_ZOFFL       0x14        /**< Z-Axis offset drift register LSB */
#define MMA745x_REG_ZOFFH       0x15        /**< Z-Axis offset drift register MSB */

#define MMA745x_REG_MCTL        0x16        /**< Mode control register */
#define MMA745x_REG_INTRST      0x17        /**< Interrupt latch reset register */
#define MMA745x_REG_CTL1        0x18        /**< Control register 1 */
#define MMA745x_REG_CTL2        0x19        /**< Control register 2 */
#define MMA745x_REG_LDTH        0x1A        /**< Level detection threshold value */
#define MMA745x_REG_PDTH        0x1B        /**< Pulse detection threshold value */
#define MMA745x_REG_PW          0x1C        /**< Pulse duration value */
#define MMA745x_REG_LT          0x1D        /**< Latency time value */
#define MMA745x_REG_TW          0x1E        /**< Time window for 2nd pulse value (double-click detection) */

/*
 * MMA745x Status Register definition.
 * This register is read only.
 */
#define MMA745X_STATUS_DRDY     (1 << 0)    /**< 1: Data ready */
#define MMA745X_STATUS_DOVR     (1 << 1)    /**< 1: Overrun (previous data was overwritten before it was read.) */
#define MMA745X_STATUS_PERR     (1 << 2)    /**< 1: Parity error in trim data, self-test is disabled */

/*
 * MMA745x Detection Source Register definition.
 * This register is read only.
 */
#define MMA745x_DETSRC_INT1     (1 << 0)    /**< 1: Interruppt signal INT1 assigned by INTGR detected. */
#define MMA745x_DETSRC_INT2     (1 << 1)    /**< 1: Interruppt signal INT2 assigned by INTGR detected. */
#define MMA745x_DETSRC_PDZ      (1 << 2)    /**< 1: Pulse detection on Z-axis */
#define MMA745x_DETSRC_PDY      (1 << 3)    /**< 1: Pulse detection on Y-axis */
#define MMA745x_DETSRC_PDX      (1 << 4)    /**< 1: Pulse detection on X-axis */
#define MMA745x_DETSRC_LDZ      (1 << 5)    /**< 1: Level detection on Z-axis */
#define MMA745x_DETSRC_LDY      (1 << 6)    /**< 1: Level detection on Y-axis */
#define MMA745x_DETSRC_LDX      (1 << 7)    /**< 1: Level detection on X-axis */

/*
 * MMA745x I2C Address Register definition.
 * This register is read only, bit 7 is read/write.
 */
#define MMA745x_I2CAD_I2CDIS    (1 << 7)    /**< rw: 0: I2C and SPI available / 1: I2C disabled */
#define MMA745x_I2CAD_I2CMSK    (0x7F)      /**< ro: Mask to read chips I2C address */

/*
 * MMA745x Mode Register definitions.
 * This register is read/write.
 */
#define MMA745X_MCTL_STBY       (0x00 << 0) /**< rw: Mode standby */
#define MMA745X_MCTL_MEAS       (0x01 << 0) /**< rw: Mode measurement, INT1/DRDY pin may serve as data ready signal. */
#define MMA745X_MCTL_LVL        (0x02 << 0) /**< rw: Mode level detection, INTx may serve as level interrupt. */
#define MMA745X_MCTL_PLS        (0x03 << 0) /**< rw: Mode pulse detection, INTx may serve as pulse interrupt. */
#define MMA745X_MCTL_MSK        (0x03 << 0) /**< Mask mode bits */

#define MMA745X_MCTL_GLVL_8G    (0x00 << 2) /**< rw: Measurement range is 8g. */
#define MMA745X_MCTL_GLVL_4G    (0x02 << 2) /**< rw: Measurement range is 4g. */
#define MMA745X_MCTL_GLVL_2G    (0x01 << 2) /**< rw: Measurement range is 2g. */
#define MMA745X_MCTL_GLVL_MSK   (0x03 << 2) /**< Mask measurement range. */

#define MMA745X_MCTL_STON       (1 << 4)    /**< rw: 1: Self-test is anebled. */
#define MMA745X_MCTL_SPI3W      (1 << 5)    /**< rw: 1: SPI in 3-wire mode, 0: SPI is 4-wire mode. */
#define MMA745X_MCTL_DRPD       (1 << 6)    /**< rw: 1: Data ready status is output to INT1/DRDY pin. */

/*
 * MMA745x Interrupt Reset Register definitions.
 * This register is read/write.
 */
#define MMA745x_INTRST_CLRINT1  (1 << 0)    /**< rw: 1: Clear INT1, 0: Enable INT1 */
#define MMA745x_INTRST_CLRINT2  (1 << 1)    /**< rw: 1: Clear INT2, 0: Enable INT2 */
#define MMA745x_INTRST_MSK      (MMA745x_INTRST_CLRINT1 | MMA745x_INTRST_CLRINT2)
/*
 * MMA745x Control Register 1 definitions.
 * This register is read/write.
 */
#define MMA745x_CTL1_INTREV     (1 << 0)    /**< rw: 0: Routing: sig INT1 to pin INT1/DRDY, signal INT2 to pin INT2
                                                     1: Routing: sig INT1 to pin INT2, signal INT2 to pin INT1/DRDY */
#define MMA745x_CTL1_L1P2       (0x00 << 1) /**< rw: INT1 signal is level detection, INT2 signal is pulse detection */
#define MMA745x_CTL1_P1L2       (0x01 << 1) /**< rw: INT1 signal is pulse detection, INT2 signal is level detection */
#define MMA745x_CTL1_P1P2       (0x02 << 1) /**< rw: INT1 signal is single pulse, INT2 signal is double pulse detection. */
#define MMA745x_CTL1_XDA        (1 << 3)    /**< rw: 0: Enable / 1: Disable X-axis for detection. */
#define MMA745x_CTL1_YDA        (1 << 4)    /**< rw: 0: Enable / 1: Disable Y-axis for detection. */
#define MMA745x_CTL1_ZDA        (1 << 5)    /**< rw: 0: Enable / 1: Disable Z-axis for detection. */
#define MMA745x_CTL1_THOPT      (1 << 6)    /**< rw: 0: Threshold is absolute, 1: Threshold is signed integer. */
#define MMA745x_CTL1_DFBW       (1 << 7)    /**< rw: 0: Bandwidth filter is 62.5Hz, 1: 125Hz. */

/*
 * MMA745x Control Register 2 definitions.
 * This register is read/write.
 */
#define MMA745x_CTL2_LDPL       (1 << 0)    /**< rw: 0: Level detection polarity positive, 3-axes OR-ed.
                                                     1: Level detection polarity negative, 3-axes AND-ed. */
#define MMA745x_CTL2_PDPL       (1 << 1)    /**< rw: 0: Pulse detection polarity positive, 3-axes OR-ed.
                                                     1: Pulse detection polarity negative, 3-axes AND-ed. */
#define MMA745x_CTL2_DRVO       (1 << 2)    /**< rw: 0: Standard / 1: strong drive strength on SDA/SDO pin. */

/*
 * MMA745x Level Detection Threshold Limit Value.
 * This register is read/write.
 *
 * This register contains the threshold value for level detection.
 * If THOPT in CTL1 is 0 it is an unsigned 7 bit value and bit 7 should be 0.
 * If THOPT in CTL1 is 1 it is an signed 8 bit value.
 */
#define MMA745x_LDTH_SMSK       0x7F        /**< Mask for value if THOPT in CTL1 is 0. */

/*
 * MMA745x Pulse Detection Threshold Limit Value.
 * This register is read/write.
 *
 * This register contains the threshold value for pulse detection.
 * This is an unsigned 7 bit value and bit 7 should be 0.
 */
#define MMA745x_PDTH_SMSK       0x7F        /**< Mask for value if THOPT in CTL1 is 0. */

/*
 * MMA745x Pulse Duration Value.
 * This register is read/write.
 *
 * This register contains the pulse duration value.
 * This is an unsigned 8 bit value in 0.5ms steps.
 */
#define MMA745x_PW_MSK          0xFF        /**< Pulse duration value mask. */

/*
 * MMA745x Latency Time Value.
 * This register is read/write.
 *
 * This register contains the latency time for pulse detection.
 * This is an unsigned 8 bit value in 1ms steps.
 */
#define MMA745x_LT_MSK          0xFF        /**< Latency time value mask. */

/*
 * MMA745x Double Pulse Detection Time Window Value.
 * This register is read/write.
 *
 * This register contains time window for double pulse detection.
 * This is an unsigned 8 bit value in 1ms steps.
 */
#define MMA745x_TW_MSK          0xFF        /**< Time window for 2nd pulse value (double-click detection) */

#include <cfg/mma745x.h>

#ifndef MMA745X_RANGE
#define MMA745X_RANGE   MMA745X_MCTL_GLVL_8G
#endif

#ifndef MMA74xx_MODE
#define MMA74xx_MODE (MMA745X_MCTL_MEAS | MMA745X_RANGE)
#endif

/*! brief MMA7455L 10-bit values and offset register struct
 */
typedef struct NUT_PACKED_TYPE
{
    int16_t x;
    int16_t y;
    int16_t z;
} mma10bit_t;

/*! brief MMA7455L 10-bit values and offset register struct
 */
typedef struct NUT_PACKED_TYPE
{
    int8_t x;
    int8_t y;
    int8_t z;
} mma8bit_t;

/*! brief MMA7455L initialization struct
 */
typedef struct NUT_PACKED_TYPE
{
    uint8_t rMODE;
    uint8_t rINTRST;
    uint8_t rCONTROL1;
    uint8_t rCONTROL2;
    uint8_t rLEVEL;
    uint8_t rPVALUE;
    uint8_t rPDUR;
    uint8_t rLATTV;
    uint8_t rTW;
} mmaInit_t;

/*! brief MMA7455L combined status register
 */
typedef struct NUT_PACKED_TYPE
{
    uint8_t state;
    uint8_t detsrc;
} mmaState_t;

/* brief MMA745x control function calls.
 */
#define MMA_GET_STATE   0
#define MMA_SET_MODE    1
#define MMA_GET_IRQ     2
#define MMA_SET_IRQ     3
#define MMA_CLR_IRQ     4

/* Low Level Access Functions */
int Mma745xWrite( uint_fast8_t reg, void *val, size_t len);
int Mma745xRead( uint_fast8_t reg, void *val, size_t len);

/* Raw Value Access Functions */
int Mma745xReadVal8( mma8bit_t *val);
int Mma745xReadVal10( uint8_t ofs, mma10bit_t *val);

/* g Value Access Functions */
int Mma745xReadG( mma10bit_t *val);

/* Calibration Value Access Functions */
int Mma745xReadCal( mma10bit_t *cal);
int Mma745xWriteCal( mma10bit_t *cal);

/* Control Function */
int Mma745xCtl( uint_fast8_t fkt, void *val);

/* Startup Initialization */
int Mma745xInit( uint_fast8_t selftest, mmaInit_t *init);

#endif /* _DEV_MMA745X_H_ */
