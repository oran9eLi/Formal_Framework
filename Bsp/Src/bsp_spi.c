/**
 * @file bsp_spi.c
 * @brief 实现 SD 卡日志使用的板级 SPI 接口。
 */

#include "bsp_spi.h"

#include "bsp_config.h"
#include "stm32f4xx_hal.h"

static SPI_HandleTypeDef s_hspi1;
static uint8_t s_initialized;
static uint32_t s_current_prescaler;

/**
 * @brief 将 HAL SPI 状态映射为 BSP 通用返回码。
 *
 * @param[in] status HAL 返回状态。
 *
 * @return BSP 通用返回码。
 */
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

/**
 * @brief 按指定分频系数配置 SD 卡 SPI 总线。
 *
 * @param[in] prescaler HAL SPI 波特率分频宏。
 *
 * @return BSP 通用返回码。
 */
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

/**
 * @brief 确保 SD 卡 SPI 总线已经初始化。
 *
 * @return BSP_STATUS_OK 表示总线可用，否则表示初始化失败。
 */
static BSP_Status_t BSP_SPI1_EnsureReady(void)
{
    if (s_initialized != 0U)
    {
        return BSP_STATUS_OK;
    }
    return BSP_SPI1_SetSpeedLow();
}

/**
 * @brief 初始化 SD 卡 SPI 总线，默认低速。
 */
void BSP_SPI1_Init(void)
{
    (void)BSP_SPI1_EnsureReady();
}

/**
 * @brief 将 SD 卡 SPI 总线切换到初始化低速。
 */
BSP_Status_t BSP_SPI1_SetSpeedLow(void)
{
    return BSP_SPI1_Configure(SPI_BAUDRATEPRESCALER_256);
}

/**
 * @brief 将 SD 卡 SPI 总线切换到传输高速。
 */
BSP_Status_t BSP_SPI1_SetSpeedHigh(void)
{
    return BSP_SPI1_Configure(SPI_BAUDRATEPRESCALER_8);
}

/**
 * @brief 通过 SD 卡 SPI 总线发送字节。
 */
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

/**
 * @brief 通过 SD 卡 SPI 总线接收字节。
 */
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

/**
 * @brief 通过 SD 卡 SPI 总线同时发送和接收字节。
 */
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

/**
 * @brief 拉低 SD 卡片选。
 */
void BSP_SPI1_SD_Select(void)
{
    HAL_GPIO_WritePin(BSP_SD_CS_PORT, BSP_SD_CS_PIN, GPIO_PIN_RESET);
}

/**
 * @brief 拉高 SD 卡片选。
 */
void BSP_SPI1_SD_Deselect(void)
{
    HAL_GPIO_WritePin(BSP_SD_CS_PORT, BSP_SD_CS_PIN, GPIO_PIN_SET);
}
