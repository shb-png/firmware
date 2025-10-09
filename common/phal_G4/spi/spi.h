#ifndef _PHAL_SPI_H
#define _PHAL_SPI_H

#include <stddef.h>

#include "common/phal_G4/dma/dma.h"
#include "common/phal_G4/gpio/gpio.h"
#include "common/phal_G4/phal_G4.h"
#include "common/phal_G4/rcc/rcc.h"
#include "common_defs.h"

/**
 * @brief Configuration entry for SPI initialization
 */
typedef struct
{
    uint32_t data_rate; //!< Target baudrate in b/s
    uint8_t data_len; //!< Number of bits per frame (8 or 16)
    bool nss_sw; //!< Chip Select controlled by software
    bool master;
    GPIO_TypeDef* nss_gpio_port; //!< GPIO Port for CS
    uint32_t nss_gpio_pin; //!< GPIO Pin for CS

    dma_init_t* rx_dma_cfg; //!< DMA configuration for RX
    dma_init_t* tx_dma_cfg; //!< DMA configuration for TX

    volatile bool _busy; //!< SPI peripheral is in a transaction
    volatile bool _error; //!< DMA or SPI transaction error

    SPI_TypeDef* periph; //!< SPI peripheral base
} SPI_InitConfig_t;

/**
 * @brief Initialize SPI peripheral with the configured structure
 */
bool PHAL_SPI_init(SPI_InitConfig_t* handle);

/**
 * @brief Transfer data using DMA
 */

bool PHAL_SPI_transfer(SPI_InitConfig_t* spi, const uint8_t* out_data, const uint32_t data_len, const uint8_t* in_data);

/**
 * @brief Transfer data without DMA (blocking mode)
 */
bool PHAL_SPI_transfer_noDMA(SPI_InitConfig_t* spi, const uint8_t* out_data, uint32_t txlen, uint32_t rxlen, uint8_t* in_data);

/**
 * @brief Check if SPI is busy
 */
bool PHAL_SPI_busy(SPI_InitConfig_t* cfg);

/**
 * @brief Blocking single-byte write
 */
uint8_t PHAL_SPI_writeByte(SPI_InitConfig_t* spi, uint8_t address, uint8_t writeDat);

/**
 * @brief Blocking single-byte read
 */
uint8_t PHAL_SPI_readByte(SPI_InitConfig_t* spi, uint8_t address, bool skipDummy);

/**
 * @brief Force reset SPI peripheral
 */
void PHAL_SPI_ForceReset(SPI_InitConfig_t* spi);

//
// ==== DMA CONFIG MACROS FOR STM32G4 ====
//

// SPI1 RX (DMA2, Channel 2, Request 11)
#define SPI1_RXDMA_CONT_CONFIG(rx_addr_, priority_) \
    { \
        .periph_addr      = (uint32_t)&(SPI1->DR), \
        .mem_addr         = (uint32_t)(rx_addr_), \
        .tx_size          = 1, \
        .increment        = false, \
        .circular         = false, \
        .dir              = 0b0, \
        .mem_inc          = true, \
        .periph_inc       = false, \
        .mem_to_mem       = false, \
        .priority         = (priority_), \
        .mem_size         = 0b00, \
        .periph_size      = 0b00, \
        .tx_isr_en        = false, \
        .dma_chan_request = 11, /* SPI1_RX */ \
        .periph           = DMA2, \
        .channel_idx      = 1, \
        .channel          = DMA2_Channel2}

// SPI1 TX (DMA2, Channel 3, Request 12)
#define SPI1_TXDMA_CONT_CONFIG(tx_addr_, priority_) \
    { \
        .periph_addr      = (uint32_t)&(SPI1->DR), \
        .mem_addr         = (uint32_t)(tx_addr_), \
        .tx_size          = 1, \
        .increment        = false, \
        .circular         = false, \
        .dir              = 0b1, \
        .mem_inc          = true, \
        .periph_inc       = false, \
        .mem_to_mem       = false, \
        .priority         = (priority_), \
        .mem_size         = 0b00, \
        .periph_size      = 0b00, \
        .tx_isr_en        = true, \
        .dma_chan_request = 12, /* SPI1_TX */ \
        .periph           = DMA2, \
        .channel_idx      = 2, \
        .channel          = DMA2_Channel3}

// SPI2 RX (DMA1, Channel 3, Request 13)
#define SPI2_RXDMA_CONT_CONFIG(rx_addr_, priority_) \
    { \
        .periph_addr      = (uint32_t)&(SPI2->DR), \
        .mem_addr         = (uint32_t)(rx_addr_), \
        .tx_size          = 1, \
        .increment        = false, \
        .circular         = false, \
        .dir              = 0b0, \
        .mem_inc          = true, \
        .periph_inc       = false, \
        .mem_to_mem       = false, \
        .priority         = (priority_), \
        .mem_size         = 0b00, \
        .periph_size      = 0b00, \
        .tx_isr_en        = false, \
        .dma_chan_request = 13, /* SPI2_RX */ \
        .periph           = DMA1, \
        .channel_idx      = 2, \
        .channel          = DMA1_Channel3}

// SPI2 TX (DMA1, Channel 4, Request 14)
#define SPI2_TXDMA_CONT_CONFIG(tx_addr_, priority_) \
    { \
        .periph_addr      = (uint32_t)&(SPI2->DR), \
        .mem_addr         = (uint32_t)(tx_addr_), \
        .tx_size          = 1, \
        .increment        = false, \
        .circular         = false, \
        .dir              = 0b1, \
        .mem_inc          = true, \
        .periph_inc       = false, \
        .mem_to_mem       = false, \
        .priority         = (priority_), \
        .mem_size         = 0b00, \
        .periph_size      = 0b00, \
        .tx_isr_en        = true, \
        .dma_chan_request = 14, /* SPI2_TX */ \
        .periph           = DMA1, \
        .channel_idx      = 3, \
        .channel          = DMA1_Channel4}

#endif /* _PHAL_SPI_H */
