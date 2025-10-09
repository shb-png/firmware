#include "g4_testing.h"
#if (G4_TESTING_CHOSEN == TEST_SPI)

#include <string.h>

#include "common/freertos/freertos.h"
#include "common/phal_G4/gpio/gpio.h"
#include "common/phal_G4/rcc/rcc.h"
#include "common/phal_G4/spi/spi.h"
#include "main.h"

/* ============================================================
 * SPI1 ↔ SPI2 test (Master ↔ Slave)
 * 
 * SPI1 (Master)    SPI2 (Slave)
 * --------------------------------
 * PA5  -> PB13  (SCK)
 * PA7  -> PB15  (MOSI)
 * PA6  <- PB14  (MISO)
 * PB2  -> PB12  (NSS)
 * ============================================================ */

GPIOInitConfig_t gpio_config[] = {
    /* SPI1 Master Pins */
    GPIO_INIT_SPI1_SCK_PA5,
    GPIO_INIT_SPI1_MISO_PA6,
    GPIO_INIT_SPI1_MOSI_PA7,
    GPIO_INIT_SPI1_NSS_PA4,

    /* SPI2 Slave Pins */
    GPIO_INIT_SPI2_SCK_PB13,
    GPIO_INIT_SPI2_MISO_PB14,
    GPIO_INIT_SPI2_MOSI_PB15,
    GPIO_INIT_SPI2_NSS_PB12,
};

/* -------- Clock Configuration -------- */
#define TargetCoreClockrateHz 16000000U
ClockRateConfig_t clock_config = {
    .clock_source              = CLOCK_SOURCE_HSI,
    .use_pll                   = false,
    .vco_output_rate_target_hz = TargetCoreClockrateHz,
    .system_clock_target_hz    = TargetCoreClockrateHz,
    .ahb_clock_target_hz       = TargetCoreClockrateHz,
    .apb1_clock_target_hz      = TargetCoreClockrateHz,
    .apb2_clock_target_hz      = TargetCoreClockrateHz,
};

/* -------- SPI Configurations -------- */
SPI_InitConfig_t spi1_cfg = {
    .periph        = SPI1,
    .data_rate     = 1000000U, // 1 MHz
    .data_len      = 8,
    .master        = true,
    .nss_sw        = true,
    .nss_gpio_port = GPIOB,
    .nss_gpio_pin  = 2,
    .rx_dma_cfg    = NULL,
    .tx_dma_cfg    = NULL,
};

SPI_InitConfig_t spi2_cfg = {
    .periph        = SPI2,
    .data_rate     = 1000000U,
    .data_len      = 8,
    .master        = false,
    .nss_sw        = false,
    .nss_gpio_port = GPIOB,
    .nss_gpio_pin  = 12,
    .rx_dma_cfg    = NULL,
    .tx_dma_cfg    = NULL,
};

void HardFault_Handler();

static void spi_tx_thread(void);

defineThreadStack(spi_tx_thread, 5, osPriorityNormal, 256);

int main() {
    osKernelInitialize();

    /* Clock Setup */
    if (PHAL_configureClockRates(&clock_config)) {
        HardFault_Handler();
    }

    /* GPIO Init */
    if (!PHAL_initGPIO(gpio_config, sizeof(gpio_config) / sizeof(GPIOInitConfig_t))) {
        HardFault_Handler();
    }

    /* SPI Init */
    if (!PHAL_SPI_init(&spi1_cfg)) {
        HardFault_Handler();
    }
    if (!PHAL_SPI_init(&spi2_cfg)) {
        HardFault_Handler();
    }

    /* Create threads */
    createThread(spi_tx_thread);

    osKernelStart();
    return 0;
}

static void spi_tx_thread(void) {
    uint8_t tx_master[]  = {0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t rx_master[4] = {0};
    uint8_t tx_slave[]   = {0x11, 0x22, 0x33, 0x44};
    uint8_t rx_slave[4]  = {0};

    /* Phase 1: Master sends, slave receives */
    PHAL_SPI_transfer_noDMA(&spi1_cfg, tx_master, sizeof(tx_master), 0, NULL);
    PHAL_SPI_transfer_noDMA(&spi2_cfg, NULL, 0, sizeof(rx_slave), rx_slave);

    osDelay(10);

    /* Phase 2: Slave responds, master reads */
    PHAL_SPI_transfer_noDMA(&spi2_cfg, tx_slave, sizeof(tx_slave), 0, NULL);
    PHAL_SPI_transfer_noDMA(&spi1_cfg, NULL, 0, sizeof(rx_master), rx_master);

    osDelay(250);
}

void HardFault_Handler(void) {
    while (1) {
        __asm__("nop");
    }
}

#endif // G4_TESTING_CHOSEN == TEST_SPI
