/**
 * @file bsp_spi.c
 * @brief Implement SPI SD access helpers for SD card logging.
 */

#include "bsp_spi.h"

#include "bsp_config.h"
#include "stm32f4xx_hal.h"

static SPI_HandleTypeDef s_hspi1;
static uint8_t s_initialized;
static uint32_t s_current_prescaler;

static BSP_Status_t BSP_SPI1_MapHalStatus(HAL_StatusTypeDef status)
{
    if (status == HAL_OK)
    {
        return BSP_STATUS_OK;
    }
    if (status == HAL_BUSY)
    {
        return BSP_STATUS_BUSY;
    }
    if (status == HAL_TIMEOUT)
    {
        return BSP_STATUS_TIMEOUT;
    }
    return BSP_STATUS_ERROR;
}

static BSP_Status_t BSP_SPI1_Configure(uint32_t prescaler)
{
    GPIO_InitTypeDef gpio;

    if ((s_initialized != 0U) && (s_current_prescaler == prescaler))
    {
        return BSP_STATUS_OK;
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    BSP_SD_SPI_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    gpio.Pin = BSP_SD_SPI_SCK_PIN;
    gpio.Alternate = BSP_SD_SPI_SCK_AF;
    HAL_GPIO_Init(BSP_SD_SPI_SCK_PORT, &gpio);

    gpio.Pin = BSP_SD_SPI_MISO_PIN;
    gpio.Alternate = BSP_SD_SPI_MISO_AF;
    HAL_GPIO_Init(BSP_SD_SPI_MISO_PORT, &gpio);

    gpio.Pin = BSP_SD_SPI_MOSI_PIN;
    gpio.Alternate = BSP_SD_SPI_MOSI_AF;
    HAL_GPIO_Init(BSP_SD_SPI_MOSI_PORT, &gpio);

    gpio.Pin = BSP_SD_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(BSP_SD_CS_PORT, &gpio);
    HAL_GPIO_WritePin(BSP_SD_CS_PORT, BSP_SD_CS_PIN, GPIO_PIN_SET);

    if (s_initialized != 0U)
    {
        (void)HAL_SPI_DeInit(&s_hspi1);
    }

    s_hspi1.Instance = BSP_SD_SPI_INS;
    s_hspi1.Init.Mode = SPI_MODE_MASTER;
    s_hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    s_hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    s_hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    s_hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    s_hspi1.Init.NSS = SPI_NSS_SOFT;
    s_hspi1.Init.BaudRatePrescaler = prescaler;
    s_hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    s_hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    s_hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    s_hspi1.Init.CRCPolynomial = 7U;

    if (HAL_SPI_Init(&s_hspi1) != HAL_OK)
    {
        s_initialized = 0U;
        s_current_prescaler = 0U;
        return BSP_STATUS_ERROR;
    }

    s_initialized = 1U;
    s_current_prescaler = prescaler;
    return BSP_STATUS_OK;
}

static BSP_Status_t BSP_SPI1_EnsureReady(void)
{
    if (s_initialized != 0U)
    {
        return BSP_STATUS_OK;
    }
    return BSP_SPI1_SetSpeedLow();
}

void BSP_SPI1_Init(void)
{
    (void)BSP_SPI1_EnsureReady();
}

BSP_Status_t BSP_SPI1_SetSpeedLow(void)
{
    return BSP_SPI1_Configure(SPI_BAUDRATEPRESCALER_256);
}

BSP_Status_t BSP_SPI1_SetSpeedHigh(void)
{
    return BSP_SPI1_Configure(SPI_BAUDRATEPRESCALER_8);
}

BSP_Status_t BSP_SPI1_Transmit(const uint8_t *tx,
                               uint16_t length,
                               uint32_t timeout_ms)
{
    if ((tx == 0) || (length == 0U))
    {
        return BSP_STATUS_ERROR;
    }
    if (BSP_SPI1_EnsureReady() != BSP_STATUS_OK)
    {
        return BSP_STATUS_ERROR;
    }
    return BSP_SPI1_MapHalStatus(
        HAL_SPI_Transmit(&s_hspi1, (uint8_t *)tx, length, timeout_ms));
}

BSP_Status_t BSP_SPI1_Receive(uint8_t *rx,
                              uint16_t length,
                              uint32_t timeout_ms)
{
    if ((rx == 0) || (length == 0U))
    {
        return BSP_STATUS_ERROR;
    }
    if (BSP_SPI1_EnsureReady() != BSP_STATUS_OK)
    {
        return BSP_STATUS_ERROR;
    }
    return BSP_SPI1_MapHalStatus(
        HAL_SPI_Receive(&s_hspi1, rx, length, timeout_ms));
}

BSP_Status_t BSP_SPI1_TransmitReceive(const uint8_t *tx,
                                      uint8_t *rx,
                                      uint16_t length,
                                      uint32_t timeout_ms)
{
    if ((tx == 0) || (rx == 0) || (length == 0U))
    {
        return BSP_STATUS_ERROR;
    }
    if (BSP_SPI1_EnsureReady() != BSP_STATUS_OK)
    {
        return BSP_STATUS_ERROR;
    }
    return BSP_SPI1_MapHalStatus(
        HAL_SPI_TransmitReceive(&s_hspi1,
                                (uint8_t *)tx,
                                rx,
                                length,
                                timeout_ms));
}

void BSP_SPI1_SD_Select(void)
{
    HAL_GPIO_WritePin(BSP_SD_CS_PORT, BSP_SD_CS_PIN, GPIO_PIN_RESET);
}

void BSP_SPI1_SD_Deselect(void)
{
    HAL_GPIO_WritePin(BSP_SD_CS_PORT, BSP_SD_CS_PIN, GPIO_PIN_SET);
}
