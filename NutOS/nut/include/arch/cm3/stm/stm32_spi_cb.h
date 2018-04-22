#ifndef _STM32_SPI_CB_H_
#define _STM32_SPI_CB_H_

#include <dev/gpio.h>
#include <cfg/devices.h>
#include <arch/cm3/stm/stm32_gpio.h>

typedef enum {
    IRQ_MODE = 0,
    DMA_MODE,
    POLLING_MODE,
}spi_transfer_t;

/*!
 * \addtogroup xgSpiBusSTM32
 */
/*@{*/
typedef struct _STM32_SPI_ICB         STM32_SPI_ICB;
typedef struct _STM32_SPI_DCB         STM32_SPI_DCB;
typedef struct _STM32_SPINODE_VALUES  STM32_SPINODE_VALUES;
/*!
 * \brief Constant local data of the STM32 SPI hardware.
 *
 * ToDo: Order in a way that few padding happens!
 * Access will need flash access wait states.
 * Use with care in interrupts!
 */
struct _STM32_SPI_ICB {
    /*! \brief SPIBUS transfer_mode. */
    const spi_transfer_t transfer_mode;
    /*! \brief Configured device Pin speed. */
    const device_pin_speed_t device_pin_speed;
#if defined(MCU_STM32F1)
    /*! \brief Remap Register on F1. */
    volatile uint32_t *const remap_reg;
    /*! \brief Remap mask on F1. */
    const uint32_t remap_mask;
    /*! \brief Remap value on F1. */
    const uint32_t remap_value;
#endif
    /*! \brief Device Clock enable register */
    volatile uint32_t *const enable_reg;
    /*! \brief Device Clock enable mask */
    const uint32_t enable_mask;
    /*! \brief Related DMA TX irq */
    IRQ_HANDLER *const dma_tx_irq;
    /*! \brief Related DMA RX irq */
    IRQ_HANDLER *const dma_rx_irq;
    /*! \brief DMA TX channnel.*/
    const uint8_t dma_tx;
    /*! \brief DMA RX channnel.*/
    const uint8_t dma_rx;
#if defined(HW_DMA_CSELR_STM32)
    /*! \brief DMA RX channnel selectiom.*/
    const uint8_t dma_rx_csel;
    /*! \brief DMA TX channnel selectiom.*/
    const uint8_t dma_tx_csel;
#endif
    /*! \brief SCK_PIN. */
    const nutgpio_t sck;
    /*! \brief SCK Pinmux. */
    const uint8_t sck_af;
    /*! \brief MOSI_PIN. */
    const nutgpio_t mosi;
     /*! \brief MOSI Pinmux. */
    const uint8_t mosi_af;
    /*! \brief MISO_PIN. */
    const nutgpio_t miso;
    /*! \brief MISO Pinmux. */
    const uint8_t miso_af;
    /*! \brief SPI chip select pins */
    const nutgpio_t cs[4];
};

/*!
 * \brief Variable data for the SPIBUS driver
 */
struct _STM32_SPI_DCB {
    const uint8_t   *spi_txp;
    /*! \brief Pointer to receive nuffer. */
    uint8_t         *spi_rxp;
    /*! \brief Remaining items to transmiy. */
    volatile size_t spi_tx_len;
    /*! \brief Remaining items to receive. */
    volatile size_t spi_rx_len;
    /*! \brief Remaining transaction items. */
    volatile size_t spi_len;
    /*! \brief SCK GPIO structure. */
    GPIO_TypeDef *sck_gpio;
    /*! \brief SCK input bit mask. */
    uint32_t sck_input_mask;
    /*! \brief SCK Pull-UP|Down register mask. */
    uint32_t sck_pupdr_mask;
};

/*!
 * \brief Per Node variable data
 */
struct _STM32_SPINODE_VALUES {
    /*! \brief Node specific value for CR1. */
    uint32_t node_CR1;
    /*! \brief Node specific value for CR2. */
    uint32_t node_CR2;
};
#endif
