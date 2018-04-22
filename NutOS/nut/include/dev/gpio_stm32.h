/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
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
 * $Id: stm32_can2.c 3108 2010-09-15 21:11:15Z Astralix $
 * \endverbatim
 */

#include <cfg/arch.h>
#include <cfg/arch/gpio.h>
#include <dev/irqreg.h>
#include <arch/cm3/stm/stm32xxxx.h>

typedef uint32_t nutgpio_port_t;
typedef uint_fast16_t nutgpio_pin_t;

#define NUTGPIO_PORT    GPIOA_BASE
#define NUTGPIO_PORTA   GPIOA_BASE
#define NUTGPIO_PORTB   GPIOB_BASE
#define NUTGPIO_PORTC   GPIOC_BASE
#define NUTGPIO_PORTD   GPIOD_BASE
#define NUTGPIO_PORTE   GPIOE_BASE
#define NUTGPIO_PORTF   GPIOF_BASE
#define NUTGPIO_PORTG   GPIOG_BASE
#define NUTGPIO_PORTH   GPIOH_BASE
#define NUTGPIO_PORTI   GPIOI_BASE
#define NUTGPIO_PORTJ   GPIOJ_BASE
#define NUTGPIO_PORTK   GPIOK_BASE

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
 * STM32 Specific:
 * If combined with GPIO_CFG_OUTPUT it enables the alternate function.
 * If used without, it sets the analog input mode of the pin.
 */
#define GPIO_CFG_DISABLED   0x00000001

/*!
 * \brief GPIO output direction enabled.
 *
 * If set, the pin is configured as an output. Otherwise it is in
 * input mode or z-state.
 * For STM32F it is configured as output 50MHz.
 */
#define GPIO_CFG_OUTPUT     0x00000002

/*!
 * \brief GPIO pull-up enabled.
 */
#define GPIO_CFG_PULLUP     0x00000004

/*!
 * \brief GPIO pull-down enabled.
 */
#define GPIO_CFG_PULLDOWN   0x00000100

/*!
 * \brief GPIO open drain output feature enabled.
 *
 * If not set, the output will use push pull mode.
 */
#define GPIO_CFG_MULTIDRIVE 0x00000008

/*!
 * \brief GPIO input glitch filter enabled.
 *
 * Not supported with STM32F
 */
#define GPIO_CFG_DEBOUNCE   0x00000000

/*!
 * \brief GPIO set to alternate function.
 *
 * STM32F specific:
 * Enables alternate function of the pin.
 */
#define GPIO_CFG_PERIPHAL   0x00000020

/*!
 * \brief GPIO pin speed
 *
 * Lower Supply voltage or high loads means lower speed!
 *
 * Speed L0/L1   F0/F1/3 F2/F4
 * SLOW  400 kHz  2 MHz    2 MHz
 * MED     2 MHz  2 MHz   25 MHz
 * FAST   10 MHz 10 MHz   50 MHz
 * HIGH   50 MHz 50 MHz  100 MHz
 *
 * As default, we set GPIO_CFG_SPEED_MED
 */
#define GPIO_CFG_SPEED       0x000000C0
#define GPIO_CFG_SPEED_SLOW  0x00000040
#define GPIO_CFG_SPEED_MED   0x00000000
#define GPIO_CFG_SPEED_FAST  0x00000080
#define GPIO_CFG_SPEED_HIGH  0x000000C0

extern const uint16_t ospeed_values[4];
/*!
 * \brief GPIO Output Register inital value  Low
 */
#define GPIO_CFG_INIT_LOW    0x40000000
/*!
 * \brief GPIO Output Register inital value  High
 */
#define GPIO_CFG_INIT_HIGH   0x80000000

typedef enum {
    GPIO_SLOW = GPIO_CFG_SPEED_SLOW,
    GPIO_MED  = GPIO_CFG_SPEED_MED,
    GPIO_FAST = GPIO_CFG_SPEED_FAST,
    GPIO_HIGH = GPIO_CFG_SPEED_HIGH,
} device_pin_speed_t;

typedef struct _gpio_vector{
    void (*iov_handler) (void *);
    void *iov_arg;
} GPIO_VECTOR;

typedef struct _gpio_signal GPIO_SIGNAL;

struct _gpio_signal{
    int   ios_port;
    int   ios_pin;
    GPIO_VECTOR ios_vector;
    GPIO_SIGNAL *sig_next;
};

extern uint32_t GpioPinConfigGet(int bank, int bit);
extern int GpioPinConfigSet(int bank, int bit, uint32_t flags);
extern int GpioPortConfigSet(int bank, uint32_t mask, uint32_t flags);

#define GpioPinSetHigh(bank, bit)   (CM3REG((bank), GPIO_TypeDef, BSRR ) = (1<<(bit)))
#define GpioPortSetHigh(bank, mask) (CM3REG((bank), GPIO_TypeDef, BSRR) = mask)
#if defined(GPIO_BRR_BR_0)
/* F0/F1/F3/L0/L4 have explicit BRR register */
# define GpioPinSetLow(bank, bit)    (CM3REG((bank), GPIO_TypeDef, BRR )  = (1<<(bit)))
# define GpioPortSetLow(bank, mask)  (CM3REG((bank), GPIO_TypeDef, BRR ) = mask)
#else
/* L1/F2/F4/F7 have only BSRR */
# define GpioPinSetLow(bank, bit)    (((volatile uint16_t*)((bank) + offsetof(GPIO_TypeDef, BSRR)))[1] = (1 << (bit)))
# define GpioPortSetLow(bank, mask)  (((volatile uint16_t*)((bank) + offsetof(GPIO_TypeDef, BSRR)))[1] = mask)
#endif

#if defined(MCU_STM32F0) ||defined(MCU_STM32F3) || defined(MCU_STM32L4)
/* GPIO on AHB2 is outside of bitband region */
#define GpioPinGet(bank, bit)        ((CM3REG((bank), GPIO_TypeDef, IDR ) & (1<<(bit)))?1:0)
#define GpioPinSet(bank, bit, value) ((value)? GpioPinSetHigh(bank, bit): GpioPinSetLow(bank, bit))
#define GpioPinDrive(bank, bit)      (CM3REG((bank), GPIO_TypeDef, MODER) |=  (1<<((bit)<<1)))
#define GpioPinRelease(bank, bit)    (CM3REG((bank), GPIO_TypeDef, MODER) &= ~(1<<((bit)<<1)))
#else
#define GpioPinGet(bank, bit)        CM3BBGET((bank), GPIO_TypeDef, IDR, (bit))
#define GpioPinSet(bank, bit, value) CM3BBSETVAL((bank), GPIO_TypeDef, ODR, bit, value)
#if defined(MCU_STM32F1)
/* We unconditionally switch back to 2 Mhz output speed after we released the pin at least once*/
#define GpioPinDrive(bank, bit)     do {                                \
    __IO uint32_t *cr_bb = &CM3BBREG((bank), GPIO_TypeDef, CRL, ((bit)<<2)); \
    cr_bb[1] = 1; cr_bb[2] = 0; } while (0)
#define GpioPinRelease(bank, bit)   do {                                \
    __IO uint32_t *cr_bb = &CM3BBREG((bank), GPIO_TypeDef, CRL, ((bit)<<2)); \
    cr_bb[1] = 0; cr_bb[2] = 1; } while (0)
#else
#define GpioPinDrive(bank, bit)      CM3BBSET((bank), GPIO_TypeDef, MODER, (bit)<<1)
#define GpioPinRelease(bank, bit)    CM3BBCLR((bank), GPIO_TypeDef, MODER, (bit)<<1)
#endif
#endif

#define GpioPortGet(bank)             CM3REG((bank), GPIO_TypeDef, IDR )
#define GpioPortSet(bank, value)     (CM3REG((bank), GPIO_TypeDef, ODR ) = value)

#if defined(RCC_IOPENR_GPIOAEN)
#define GpioClkEnable(bank) (RCC->IOPENR |= 1 <<((bank-GPIOA_BASE)>>10))
#elif defined(RCC_APB2ENR_IOPAEN)
#define GpioClkEnable(bank) (RCC->APB2ENR |= (1 << ((((bank-GPIOA_BASE) >> 10)) + 2)))
#elif defined(MCU_STM32L1)
#define GpioClkEnable(bank) CM3BBSET(RCC_BASE, RCC_TypeDef, AHBENR, (  bank-GPIOA_BASE)>>10)
#elif defined(MCU_STM32F0) ||defined(MCU_STM32F3)
#define GpioClkEnable(bank) CM3BBSET(RCC_BASE, RCC_TypeDef, AHBENR, (((bank-GPIOA_BASE)>>10) +17))
#elif defined(MCU_STM32L4)
#define GpioClkEnable(bank) CM3BBSET(RCC_BASE, RCC_TypeDef, AHB2ENR,(  bank-GPIOA_BASE)>>10)
#else
#define GpioClkEnable(bank) CM3BBSET(RCC_BASE, RCC_TypeDef, AHB1ENR,(  bank-GPIOA_BASE)>>10)
#endif
extern GPIO_SIGNAL *GpioCreateIrqHandler(nutgpio_port_t port, nutgpio_pin_t bit, void (*handler) (void *), void *arg);
extern int GpioRegisterIrqHandler(GPIO_SIGNAL * sig, int bit, void (*handler) (void *), void *arg);
extern int GpioIrqEnable(GPIO_SIGNAL * sig, int bit);
extern int GpioIrqDisable(GPIO_SIGNAL * sig, int bit);
extern int GpioIrqSetMode(GPIO_SIGNAL * sig, int bit, int mode);
