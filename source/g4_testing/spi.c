#include "g4_testing.h"
#if (G4_TESTING_CHOSEN == TEST_SPI)

#include <string.h>

#include "common/freertos/freertos.h"
#include "common/phal_G4/dma/dma.h"
#include "common/phal_G4/gpio/gpio.h"
#include "common/phal_G4/rcc/rcc.h"
#include "common/phal_G4/spi/spi.h"
#include "main.h"

#define TargetCoreClockrateHz 16000000U

ClockRateConfig_t clock_config = {
    .clock_source              = CLOCK_SOURCE_HSI,
    .use_pll                   = false,
    .vco_output_rate_target_hz = 16000000U,
    .system_clock_target_hz    = TargetCoreClockrateHz,
    .ahb_clock_target_hz       = (TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz      = (TargetCoreClockrateHz / 1),
    .apb2_clock_target_hz      = (TargetCoreClockrateHz / 1),
};

GPIOInitConfig_t gpio_config[] = {
    GPIO_INIT_SPI1_SCK_PB3,
    GPIO_INIT_SPI1_MISO_PB4,
    GPIO_INIT_SPI1_MOSI_PB5,
    GPIO_INIT_SPI1_NSS_PB2,
    GPIO_INIT_SPI2_SCK_PB13,
    GPIO_INIT_SPI2_MISO_PB14,
    GPIO_INIT_SPI2_MOSI_PB15,
    GPIO_INIT_SPI2_NSS_PB12,
};

#define SPI_DMA_BUF_SIZE 16
static uint8_t spi1_tx_buf[SPI_DMA_BUF_SIZE];
static uint8_t spi1_rx_buf[SPI_DMA_BUF_SIZE];
static uint8_t spi2_tx_buf[SPI_DMA_BUF_SIZE];
static uint8_t spi2_rx_buf[SPI_DMA_BUF_SIZE];

static dma_init_t spi1_rx_dma = SPI1_RXDMA_CONT_CONFIG((uint32_t)&spi1_rx_buf, 0b01);
static dma_init_t spi1_tx_dma = SPI1_TXDMA_CONT_CONFIG((uint32_t)&spi1_tx_buf, 0b01);
static dma_init_t spi2_rx_dma = SPI2_RXDMA_CONT_CONFIG((uint32_t)&spi2_rx_buf, 0b01);
static dma_init_t spi2_tx_dma = SPI2_TXDMA_CONT_CONFIG((uint32_t)&spi2_tx_buf, 0b01);

static SPI_InitConfig_t spi1_cfg = {
    .periph        = SPI1,
    .data_len      = 8,
    .data_rate     = 1000000U,
    .nss_sw        = true,
    .nss_gpio_port = GPIOB,
    .nss_gpio_pin  = 2,
    .rx_dma_cfg    = &spi1_rx_dma,
    .tx_dma_cfg    = &spi1_tx_dma,
    ._busy         = false,
    ._error        = false,
};

static SPI_InitConfig_t spi2_cfg = {
    .periph        = SPI2,
    .data_len      = 8,
    .data_rate     = 1000000U,
    .nss_sw        = true,
    .nss_gpio_port = GPIOB,
    .nss_gpio_pin  = 12,
    .rx_dma_cfg    = &spi2_rx_dma,
    .tx_dma_cfg    = &spi2_tx_dma,
    ._busy         = false,
    ._error        = false,
};

static void spi_dma_dual_loop(void);
defineThreadStack(spi_dma_dual_loop, 5, osPriorityHigh, 512);

void HardFault_Handler(void);

int main(void) {
    osKernelInitialize();

    if (PHAL_configureClockRates(&clock_config))
        HardFault_Handler();
    if (!PHAL_initGPIO(gpio_config, sizeof(gpio_config) / sizeof(GPIOInitConfig_t)))
        HardFault_Handler();
    if (!PHAL_SPI_init(&spi1_cfg))
        HardFault_Handler();
    if (!PHAL_SPI_init(&spi2_cfg))
        HardFault_Handler();

    createThread(spi_dma_dual_loop);
    osKernelStart();
    return 0;
}

static void spi_dma_dual_loop(void) {
    uint8_t counter = 0;

    for (;;) {
        /* Prepare test data */
        for (uint8_t i = 0; i < SPI_DMA_BUF_SIZE; i++) {
            spi1_tx_buf[i] = counter + i;
            spi2_tx_buf[i] = ~spi1_tx_buf[i]; // echo inverse for test
            spi1_rx_buf[i] = 0;
            spi2_rx_buf[i] = 0;
        }

        /* Assert chip selects */
        PHAL_writeGPIO(spi1_cfg.nss_gpio_port, spi1_cfg.nss_gpio_pin, 0);
        PHAL_writeGPIO(spi2_cfg.nss_gpio_port, spi2_cfg.nss_gpio_pin, 0);

        /* Start RX on both sides first */
        PHAL_reEnable(&spi1_rx_dma);
        PHAL_reEnable(&spi2_rx_dma);

        /* Start TX on both */
        PHAL_SPI_transfer(&spi1_cfg, spi1_tx_buf, SPI_DMA_BUF_SIZE, spi1_rx_buf);
        PHAL_SPI_transfer(&spi2_cfg, spi2_tx_buf, SPI_DMA_BUF_SIZE, spi2_rx_buf);

        /* Wait until both are done */
        while (PHAL_SPI_busy(&spi1_cfg) || PHAL_SPI_busy(&spi2_cfg)) {
            osDelay(1);
        }

        /* Deassert chip selects */
        PHAL_writeGPIO(spi1_cfg.nss_gpio_port, spi1_cfg.nss_gpio_pin, 1);
        PHAL_writeGPIO(spi2_cfg.nss_gpio_port, spi2_cfg.nss_gpio_pin, 1);

        counter++;
        osDelay(500); // 2 Hz cycle
    }
}

void HardFault_Handler(void) {
    while (1) {
        __asm__("nop");
    }
}

#endif // G4_TESTING_CHOSEN == TEST_SPI_DMA
