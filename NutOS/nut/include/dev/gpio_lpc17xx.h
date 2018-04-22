/*
 * Copyright (C) 2012 by Ole Reinhardt (ole.reinhardt@embedded-it.de)
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
 * \verbatim
 * $Id: gpio_lpc17xx.h $
 * \endverbatim
 */

typedef uint32_t nutgpio_port_t;
typedef uint32_t nutgpio_pin_t;

#include <cfg/arch.h>
#include <cfg/arch/gpio.h>
#include <dev/irqreg.h>
#if defined(MCU_LPC176x)
#include <arch/cm3/nxp/lpc176x.h>
#elif defined(MCU_LPC177x_8x)
#include <arch/cm3/nxp/lpc177x_8x.h>
#elif defined(MCU_LPC407x_8x)
#include <arch/cm3/nxp/lpc407x_8x.h>
#else
#warning "Unknown LPC family"
#endif

#define NUTGPIO_PORT    0
#define NUTGPIO_PORT0   0
#define NUTGPIO_PORT1   1
#define NUTGPIO_PORT2   2
#define NUTGPIO_PORT3   3
#define NUTGPIO_PORT4   4
#define NUTGPIO_PORT5   5

#define NUTGPIO_PORTA   NUTGPIO_PORT0
#define NUTGPIO_PORTB   NUTGPIO_PORT1
#define NUTGPIO_PORTC   NUTGPIO_PORT2
#define NUTGPIO_PORTD   NUTGPIO_PORT3
#define NUTGPIO_PORTE   NUTGPIO_PORT4
#define NUTGPIO_PORTF   NUTGPIO_PORT5

#define NUTGPIO_EXTINT0     1
#define NUTGPIO_EXTINT1     2
#define NUTGPIO_EXTINT2     3
#define NUTGPIO_EXTINT3     4
#define NUTGPIO_EXTINT4     5


/*!
 * \brief GPIO input.
 *
 * Will configure the pin as input. This is the default state, when no other
 * config option is given.
 */

#define GPIO_CFG_INPUT      0x00000000


/*!
 * \brief GPIO disabled.
 *
 * Will activate the pins alternate function if set. This may not work
 * on all platforms.
 */
#define GPIO_CFG_DISABLED   0x00000001

/*!
 * \brief GPIO output direction enabled.
 *
 * If set, the pin is configured as an output. Otherwise it is in
 * input mode or z-state.
 */
#define GPIO_CFG_OUTPUT     0x00000002

/*!
 * \brief GPIO pull-up enabled.
 */
#define GPIO_CFG_PULLUP     0x00000004

/*!
 * \brief GPIO pull-down enabled.
 */
#define GPIO_CFG_PULLDOWN   0x00000008

/*!
 * \brief GPIO repeater-mode enabled.
 */
#define GPIO_CFG_REPEATER   0x00000010

/*!
 * \brief GPIO open drain output feature enabled.
 *
 * If not set, the output will use push pull mode.
 */
#define GPIO_CFG_MULTIDRIVE 0x00000020

/*!
 * \brief GPIO input glitch filter enabled.
 *
 */
#if defined(MCU_LPC176x)
/*Not supported with the LPC176x family */
#define GPIO_CFG_DEBOUNCE   0x00000000
#elif defined(MCU_LPC177x_8x) || defined(MCU_LPC407x_8x)
#define GPIO_CFG_DEBOUNCE   0x00000040
#define GPIO_CFG_HYSTERESIS 0x00000080
#define GPIO_CFG_INVERT     0x00000100
#define GPIO_CFG_SLEWCTRL   0x00000200
#define GPIO_CFG_ADMODE     0x00000400
#define GPIO_CFG_DAC_ENABLE 0x00000800
#endif

/*!
 * \brief GPIO starts with output high.
 */
#define GPIO_CFG_INIT_HIGH 0x00001000

/*!
 * \brief GPIO starts with output low.
 */
#define GPIO_CFG_INIT_LOW 0x00002000



/*!
 * \brief GPIO set to alternate function 0.
 *
 * LPC176x specific:
 * Enables alternate function 0..3 of the pin.
 * Function 0 is the GPIO function
 */

#define GPIO_CFG_PERIPHERAL_MASK 0x07000000
#define GPIO_CFG_PERIPHERAL_POS  24
#define GPIO_CFG_PERIPHERAL0  0x00000000
#define GPIO_CFG_PERIPHERAL1  0x01000000
#define GPIO_CFG_PERIPHERAL2  0x02000000
#define GPIO_CFG_PERIPHERAL3  0x03000000
#if defined(MCU_LPC177x_8x) || defined(MCU_LPC407x_8x)
#define GPIO_CFG_PERIPHERAL4  0x04000000
#define GPIO_CFG_PERIPHERAL5  0x05000000
#define GPIO_CFG_PERIPHERAL6  0x06000000
#define GPIO_CFG_PERIPHERAL7  0x07000000
#endif

#define GPIO_BANKID2BASE(bank) (LPC_GPIO0_BASE + (bank << 5))

typedef struct {
    void (*iov_handler) (void *);
    void *iov_arg;
} GPIO_VECTOR;

typedef struct _gpio_signal GPIO_SIGNAL;

struct _gpio_signal {
    int   ios_port;
    void (*ios_handler) (void *);
    int (*ios_ctl) (GPIO_SIGNAL * sig, int cmd, void *param, int bit);
    GPIO_VECTOR *ios_vector;
    uint32_t enabled;
    uint32_t mode_rising_enabled;
    uint32_t mode_falling_enabled;
};


extern GPIO_SIGNAL sig_GPIO0;
extern GPIO_SIGNAL sig_GPIO2;

extern uint32_t GpioPinConfigGet(int bank, int bit);
extern int GpioPinConfigSet(int bank, int bit, uint32_t flags);
extern int GpioPortConfigSet(int bank, uint32_t mask, uint32_t flags);

#define GpioPinGet(bank, bit)            CM3BBGET(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIOPIN, (bit))
//#define GpioPinSet(bank, bit, value)   CM3BBSETVAL(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIOPIN, bit, value)

#define GpioPinSet(bank, bit, value)     do { \
                                             if(value) CM3BBSET(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIOSET, (bit)); else \
                                                       CM3BBSET(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIOCLR, (bit)); \
                                         } while (0)

#define GpioPinMaskSet(bank, bit, value) CM3BBSETVAL(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIOMASK, (bit), value)

#define GpioPinSetHigh(bank, bit)        CM3BBSET(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIOSET, (bit))
#define GpioPinSetLow(bank, bit)         CM3BBSET(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIOCLR, (bit))

#define GpioPinDrive(bank, bit)          CM3BBSET(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIODIR, (bit))
#define GpioPinRelease(bank, bit)        CM3BBCLR(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIODIR, (bit))

#define GpioPortGet(bank)                CM3REG(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIOPIN)
#define GpioPortSet(bank, value)         CM3REG(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIOPIN)  = (value)
#define GpioPortSetHigh(bank, mask)      CM3REG(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIOSET)  = (mask)
#define GpioPortSetLow(bank, mask)       CM3REG(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIOCLR)  = (mask)
#define GpioPortMaskSet(bank, mask)      CM3REG(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIOMASK) = (mask)

extern GPIO_SIGNAL *GpioCreateIrqHandler(nutgpio_port_t port, nutgpio_pin_t bit, void (*handler) (void *), void *arg);
extern int GpioRegisterIrqHandler(GPIO_SIGNAL * sig, int bit, void (*handler) (void *), void *arg);
extern int GpioIrqEnable(GPIO_SIGNAL * sig, int bit);
extern int GpioIrqStatus(GPIO_SIGNAL * sig, int bit);
extern int GpioIrqDisable(GPIO_SIGNAL * sig, int bit);
extern int GpioIrqSetMode(GPIO_SIGNAL * sig, int bit, int mode);
