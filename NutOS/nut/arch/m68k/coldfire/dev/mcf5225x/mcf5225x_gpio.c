/*
 * Copyright 2012 by Embedded Technologies s.r.o
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

#include <arch/m68k.h>
#include <dev/gpio.h>
#include <sys/nutdebug.h>

enum pin_type {
    DUAL, QUAD
};

struct bank_layout {
    enum pin_type pins_type;
    uint8_t pins_configurable;
};

struct bank_layout * BankLayout(int bank)
{
    static struct bank_layout layout;

    switch (bank) {
    case PORTDD:
    case PORTAN:
    case PORTTE:
    case PORTTF:
    case PORTTG:
    case PORTTI:
    case PORTTJ:
        layout.pins_type = DUAL;
        layout.pins_configurable = 0xFF;
        break;

    case PORTAS:
        layout.pins_type = QUAD;
        layout.pins_configurable = 0x07;
        break;

    case PORTQS:
        layout.pins_type = QUAD;
        layout.pins_configurable = 0x6F;
        break;

    case PORTTA:
    case PORTTC:
    case PORTUA:
    case PORTUB:
    case PORTUC:
        layout.pins_type = QUAD;
        layout.pins_configurable = 0x0F;
        break;

    case PORTTH:
        layout.pins_type = QUAD;
        layout.pins_configurable = 0xFF;
        break;

    case PORTNQ:
        layout.pins_type = QUAD;
        layout.pins_configurable = 0xAA;
        break;

    default:
        return NULL;
        break;
    }

    return &layout;
}

/*!
 * \brief Get pin configuration.
 *
 * \param bank GPIO bank/port number.
 * \param bit  Bit number of the specified bank/port.
 *
 * \return Attribute flags of the pin.
 */
uint32_t GpioPinConfigGet(int bank, int bit)
{
    uint32_t rc = 0;
    struct bank_layout *bank_cfg;

    /* Get & check bank layout */
    bank_cfg = BankLayout(bank);
    if ((!bank_cfg) || !(bank_cfg->pins_configurable & _BV(bit)))
        return 0;

    /* Read pin role (pin assignment register) */
    switch (bank_cfg->pins_type) {
    case DUAL:
        rc = (MCF_GPIO_PAR8(bank) >> bit) & 0x1;
        break;

    case QUAD:
        if (bank_cfg->pins_configurable & 0xF0) {
            rc = (MCF_GPIO_PAR16(bank) >> (2 * bit)) & 0x3;
        } else {
            rc = (MCF_GPIO_PAR8(bank) >> (2 * bit)) & 0x3;
        }
        break;
    }

    /* Read GPIO direction */
    if (rc == GPIO_CFG_INPUT) {
        if (MCF_GPIO_DDR(bank) & _BV(bit)) {
            rc = GPIO_CFG_OUTPUT;
        }
    }

    return rc;
}

/*!
 * \brief Set pin configuration.
 *
 * Applications may also use this function to make sure, that a specific
 * attribute is available for a specific pin.
 *
 * \note GPIO pins are typically initialized to a safe state after power
 *       up. This routine is not able to determine the consequences of
 *       changing pin configurations. In the worst case you may permanently
 *       damage your hardware by bad pin settings.
 *
 * \param bank  GPIO bank/port number.
 * \param bit   Bit number of the specified bank/port.
 * \param flags Attribute flags.
 *
 * \return 0 if all attributes had been set, -1 otherwise.
 */
int GpioPinConfigSet(int bank, int bit, uint32_t flags)
{
    GpioPortConfigSet(bank, _BV(bit), flags);

    /* Check the result. */
    if (GpioPinConfigGet(bank, bit) != flags) {
        return -1;
    }
    return 0;
}

/*!
 * \brief Set port wide pin configuration.
 *
 * \note This function does not check for undefined ports and pins or
 *       invalid attributes. If this is required, use GpioPinConfigSet().
 *
 * \param bank  GPIO bank/port number.
 * \param mask  The given attributes are set for a specific pin, if the
 *              corresponding bit in this mask is 1.
 * \param flags Attribute flags to set.
 *
 * \return Always 0.
 */
int GpioPortConfigSet(int bank, uint32_t mask, uint32_t flags)
{
    struct bank_layout *bank_cfg;
    uint32_t role = flags & (GPIO_CFG_PERIPHERAL_MASK | GPIO_CFG_OUTPUT);
    uint32_t role_mask;
    int i;

    /* Get bank layout */
    bank_cfg = BankLayout(bank);

    /* Return if the bank is unknown */
    if (!bank_cfg) {
        return 0;
    }

    /* Return if roles are not applicable. */
    if (bank_cfg->pins_type != QUAD && ((role == GPIO_CFG_PERIPHERAL1) || (role == GPIO_CFG_PERIPHERAL2))) {
        return 0;
    }

    /* Configure the configurable pins only */
    mask &= bank_cfg->pins_configurable;

    /* Configure GPIO direction first */
    if (role == GPIO_CFG_OUTPUT) {
        MCF_GPIO_DDR(bank) |= mask;
    } else if (role == GPIO_CFG_INPUT) {
        MCF_GPIO_DDR(bank) &= ~mask;
    }

    /* Then configure pin role */
    if (bank_cfg->pins_type == DUAL) {
        /* Read old roles */
        role_mask = MCF_GPIO_PAR8(bank);

        /* Update roles */
        if (role == GPIO_CFG_PERIPHERAL0) {
            role_mask |= mask;
        } else {
            role_mask &= ~mask;
        }

        /* Apply new roles */
        MCF_GPIO_PAR8(bank) = (uint8_t) role_mask;
    } else /* if (bank_cfg->pins_type == QUAD) */{
        /* Read old roles */
        role_mask = (bank_cfg->pins_configurable & 0xF0) ? MCF_GPIO_PAR16(bank) : MCF_GPIO_PAR8(bank);

        /* Shift in new roles */
        for (i = 0; i < 7; i++) {
            if (mask & (1 << i)) {
                /* Erase old role */
                role_mask &= ~(GPIO_CFG_PERIPHERAL_MASK << (2 * i));

                /* Set new role */
                role_mask |= (role & GPIO_CFG_PERIPHERAL_MASK) << (2 * i);
            }
        }

        /* Write back new roles */
        if (bank_cfg->pins_configurable & 0xF0) {
            MCF_GPIO_PAR16(bank) = (uint16_t) role_mask;
        } else {
            MCF_GPIO_PAR8(bank) = (uint8_t) role_mask;
        }
    }
    return 0;
}
