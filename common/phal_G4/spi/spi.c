#include "common/phal_G4/spi/spi.h"
#include "stm32g474xx.h"

extern uint32_t APB2ClockRateHz;
extern uint32_t APB1ClockRateHz;
static volatile SPI_InitConfig_t* active_transfer = NULL;

static uint16_t trash_can; //!< Used as an address for DMA to dump data into
static uint16_t zero; //!< Used as a constant zero during transmissions

static void handleTxComplete();

bool PHAL_SPI_init(SPI_InitConfig_t* cfg) {
    zero = 0;
    // Enable RCC Clock - Add new cases for each SPI Peripheral, to enable their clocks
    if (cfg->periph == SPI1) {
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    } else if (cfg->periph == SPI2) {
        RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;
    } else {
        return false;
    }

    // Setup for Master, positive polarity
    cfg->periph->CR1 |= SPI_CR1_MSTR | SPI_CR1_SPE | SPI_CR1_SSM | SPI_CR1_SSI;
    cfg->periph->CR1 &= ~(SPI_CR1_CPOL);
    cfg->periph->CR2 &= ~(SPI_CR2_SSOE);

    if (cfg->data_len != 8 && cfg->data_len != 16)
        return false;

    // Set data frame size for SPI transaction to 8/16 bits depending on user configuration
    cfg->periph->CR2 &= ~(SPI_CR2_DS);
    // Set the new DS value based on data_len
    if (cfg->data_len == 8) {
        cfg->periph->CR2 |= (0b0111) << SPI_CR2_DS_Pos; // 8-bit data size is 0111
    } else if (cfg->data_len == 16) {
        cfg->periph->CR2 |= (0b1111) << SPI_CR2_DS_Pos; // 16-bit data size is 1111
    }

    // Data Rate
    // Divisor is a power of 2, find the closest power of 2 limited to log2(256)
    uint32_t f_div = 0;
    if (cfg->periph == SPI1)
        f_div = LOG2_DOWN(APB2ClockRateHz / cfg->data_rate) - 1;
    // Both SPI2 and SPI3 are on APB1
    else
        f_div = LOG2_DOWN(APB1ClockRateHz / cfg->data_rate) - 1;

    f_div = CLAMP(f_div, 0, 0b111);
    cfg->periph->CR1 &= ~SPI_CR1_BR_Msk;
    cfg->periph->CR1 |= f_div << SPI_CR1_BR_Pos;

    // Setup DMA Streams if required
    if (cfg->rx_dma_cfg && !PHAL_initDMA(cfg->rx_dma_cfg))
        return false;

    if (cfg->tx_dma_cfg && !PHAL_initDMA(cfg->tx_dma_cfg))
        return false;

    // Ensure device CS is disabled
    PHAL_writeGPIO(cfg->nss_gpio_port, cfg->nss_gpio_pin, 1);

    cfg->_busy = false;
    cfg->_error = false;

    return true;
}

bool PHAL_SPI_transfer_noDMA(SPI_InitConfig_t* spi, const uint8_t* out_data, uint32_t txlen, uint32_t rxlen, uint8_t* in_data) {
    if (PHAL_SPI_busy(spi))
        return false;

    spi->_busy = true;
    spi->_error = false;

    // Enable SPI
    spi->periph->CR1 |= SPI_CR1_SPE;

    // Assert chip select
    if (spi->nss_sw)
        PHAL_writeGPIO(spi->nss_gpio_port, spi->nss_gpio_pin, 0);

    // --- TX phase ---
    for (uint32_t i = 0; i < txlen; i++) {
        while (!(spi->periph->SR & SPI_SR_TXE));   // wait for TX empty
        spi->periph->DR = out_data[i];
        while (!(spi->periph->SR & SPI_SR_RXNE));  // wait for RX complete
        (void)spi->periph->DR;                     // throw away received byte
    }

    // --- RX phase ---
    for (uint32_t i = 0; i < rxlen; i++) {
        while (!(spi->periph->SR & SPI_SR_TXE));
        spi->periph->DR = 0xFF;                    // dummy write
        while (!(spi->periph->SR & SPI_SR_RXNE));
        in_data[i] = (uint8_t)spi->periph->DR;
    }

    // Wait until not busy
    while (spi->periph->SR & SPI_SR_BSY);

    // Disable SPI
    spi->periph->CR1 &= ~SPI_CR1_SPE;

    // Deassert chip select
    if (spi->nss_sw)
        PHAL_writeGPIO(spi->nss_gpio_port, spi->nss_gpio_pin, 1);

    spi->_busy = false;
    return true;
}

bool PHAL_SPI_transfer(SPI_InitConfig_t* spi, const uint8_t* out_data, const uint32_t data_len, const uint8_t* in_data) {
    /*
    Each DMA Stream is enabled if a data buffer is provided.
    RX Side interrupts disabled, since the same data lengths are sent to both TX and RX
    */
    // Cannot use DMA without knowing essential configuration info
    if (spi->tx_dma_cfg == 0) {
        return false;
    }

    if (PHAL_SPI_busy(spi))
        return false;

    active_transfer = spi;

    // Enable CS to begin SPI transaction
    if (spi->nss_sw)
        PHAL_writeGPIO(spi->nss_gpio_port, spi->nss_gpio_pin, 0);

    spi->_busy = true;

    // Configure DMA send msg
    spi->periph->CR2 |= SPI_CR2_TXDMAEN;
    if (!out_data) {
        // No data to send, fill with dummy data
        spi->tx_dma_cfg->channel->CCR &= ~DMA_CCR_MINC;
        PHAL_DMA_setMemAddress(spi->tx_dma_cfg, (uint32_t)&zero);
    } else {
        PHAL_DMA_setMemAddress(spi->tx_dma_cfg, (uint32_t)out_data);
    }
    PHAL_DMA_setTxferLength(spi->tx_dma_cfg, data_len);

    //Configure DMA receive Msg
    if (spi->rx_dma_cfg) {
        spi->periph->CR2 |= SPI_CR2_RXDMAEN;
        if (!in_data) {
            // No data to receive, so configure DMA to disregard any received messages
            spi->rx_dma_cfg->channel->CCR &= ~DMA_CCR_MINC;
            PHAL_DMA_setMemAddress(spi->rx_dma_cfg, (uint32_t)&trash_can);
        } else {
            PHAL_DMA_setMemAddress(spi->rx_dma_cfg, (uint32_t)in_data);
        }
        PHAL_DMA_setTxferLength(spi->rx_dma_cfg, data_len);

        // We must clear interrupt flags before enabling DMA
        PHAL_reEnable(spi->rx_dma_cfg);
    }

    // Enable the DMA IRQ - copy + paste enabling selected SPI peripheral's TX DMA Stream ISR
    if (spi->periph == SPI1) {
        NVIC_EnableIRQ(DMA2_Channel3_IRQn);
    }

    // Start transaction
    spi->periph->CR1 |= SPI_CR1_SPE;

    // STM32 HAL Libraries start TX Dma transaction last
    PHAL_reEnable(spi->tx_dma_cfg);

    return true;
}

bool PHAL_SPI_busy(SPI_InitConfig_t* cfg) {
    // Latch in case active_transfer cleared during interrupt
    volatile SPI_InitConfig_t* act = active_transfer;
    if (act && cfg->periph == act->periph)
        return act->_busy;

    return false;
}

void PHAL_SPI_ForceReset(SPI_InitConfig_t* spi) {
    if (spi->periph == SPI1) {
        // Reset SPI1 peripheral (APB2 bus)
        RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
        RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
    } else if (spi->periph == SPI2) {
        // Reset SPI2 peripheral (APB1 bus)
        RCC->APB1RSTR1 |= RCC_APB1RSTR1_SPI2RST;
        RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_SPI2RST;
    }
}

static void handleTxComplete() {
    uint32_t chan = active_transfer->tx_dma_cfg->channel_idx + 1; // DMA channels are 1-based
    uint32_t tcif = DMA_ISR_TCIF1 << (4 * (chan - 1));
    uint32_t teif = DMA_ISR_TEIF1 << (4 * (chan - 1));

    DMA_TypeDef* dma = active_transfer->tx_dma_cfg->periph;

    // Transfer error?
    if (dma->ISR & teif) {
        dma->IFCR = teif;
        active_transfer->_error = true;
    }

    // Transfer complete?
    if (dma->ISR & tcif) {
        while (!(active_transfer->periph->SR & SPI_SR_TXE) ||
               (active_transfer->periph->SR & SPI_SR_BSY));

        // CS high
        if (active_transfer->nss_sw)
            PHAL_writeGPIO(active_transfer->nss_gpio_port, active_transfer->nss_gpio_pin, 1);

        // Stop DMA channels
        PHAL_stopTxfer(active_transfer->rx_dma_cfg);
        PHAL_stopTxfer(active_transfer->tx_dma_cfg);

        // Disable SPI DMA req
        active_transfer->periph->CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

        // Disable SPI
        active_transfer->periph->CR1 &= ~SPI_CR1_SPE;

        // Clear busy/error state
        active_transfer->_busy = false;
        active_transfer->_error = false;

        dma->IFCR = tcif;
        active_transfer = NULL;
    }
}

//DMA TX ISR - copy + paste for selected SPI peripheral's DMA ISR
void DMA2_Channel3_IRQHandler() {
    handleTxComplete();
}

uint8_t PHAL_SPI_readByte(SPI_InitConfig_t* spi, uint8_t address, bool skipDummy) {
    static uint8_t tx_cmd[4] = {(1 << 7), 0, 0};
    static uint8_t rx_dat[4] = {1, 1, 1, 1};
    tx_cmd[0] |= (address & 0x7F);

    // Send address, read byte depending on whether DMA is selected
    while (PHAL_SPI_busy(spi))
        ;
    if (spi->rx_dma_cfg != NULL)
        PHAL_SPI_transfer(spi, tx_cmd, skipDummy ? 2 : 3, rx_dat);
    else
        PHAL_SPI_transfer_noDMA(spi, tx_cmd, 1, skipDummy ? 1 : 2, rx_dat);
    while (PHAL_SPI_busy(spi))
        ;

    // Skip first byte of rx in case of unnecesssary information
    return skipDummy ? rx_dat[1] : rx_dat[2];
}

uint8_t PHAL_SPI_writeByte(SPI_InitConfig_t* spi, uint8_t address, uint8_t writeDat) {
    uint8_t tx_cmd[3] = {0};
    uint8_t rx_dat[3] = {0};
    tx_cmd[0] |= (address & 0x7F);
    tx_cmd[1] |= (writeDat);

    // Send address, write byte depending on whether DMA is selected
    while (PHAL_SPI_busy(spi))
        ;
    if (spi->tx_dma_cfg != NULL)
        PHAL_SPI_transfer(spi, tx_cmd, 2, rx_dat);
    else
        PHAL_SPI_transfer_noDMA(spi, tx_cmd, 2, 0, rx_dat);
    while (PHAL_SPI_busy(spi))
        ;

    return rx_dat[1];
}
