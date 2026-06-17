/**
 * @file bsp_adc.c
 * @brief 实现用于电源采样的板级 ADC 接口。
 */

#include "bsp_adc.h"

#include "bsp_config.h"
#include "stm32f4xx_hal.h"

#define BSP_ADC_TIMEOUT_MS       10U
#define BSP_ADC_AVERAGE_COUNT    8U
#define BSP_ADC_REF_MV        3300U
#define BSP_ADC_RAW_MAX       4095U

static ADC_HandleTypeDef s_hadc;
static uint8_t s_initialized;

/**
 * @brief 将 HAL ADC 返回状态映射为 BSP 通用返回码。
 *
 * @param[in] status HAL 返回状态。
 *
 * @return BSP 通用返回码。
 */
static BSP_Status_t BSP_ADC_MapHalStatus(HAL_StatusTypeDef status)
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

BSP_Status_t BSP_ADC_Init(void)
{
    ADC_ChannelConfTypeDef channel;

    if (s_initialized != 0U)
    {
        return BSP_STATUS_OK;
    }

    s_hadc.Instance = BSP_ADC_INS;
    s_hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    s_hadc.Init.Resolution = ADC_RESOLUTION_12B;
    s_hadc.Init.ScanConvMode = DISABLE;
    s_hadc.Init.ContinuousConvMode = DISABLE;
    s_hadc.Init.DiscontinuousConvMode = DISABLE;
    s_hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    s_hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    s_hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    s_hadc.Init.NbrOfConversion = 1U;
    s_hadc.Init.DMAContinuousRequests = DISABLE;
    s_hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&s_hadc) != HAL_OK)
    {
        return BSP_STATUS_ERROR;
    }

    channel.Channel = BSP_ADC_CH;
    channel.Rank = 1U;
    channel.SamplingTime = ADC_SAMPLETIME_144CYCLES;
    channel.Offset = 0U;

    if (HAL_ADC_ConfigChannel(&s_hadc, &channel) != HAL_OK)
    {
        return BSP_STATUS_ERROR;
    }

    s_initialized = 1U;
    return BSP_STATUS_OK;
}

BSP_Status_t BSP_ADC_DeInit(void)
{
    if (s_initialized == 0U)
    {
        return BSP_STATUS_OK;
    }

    if (HAL_ADC_DeInit(&s_hadc) != HAL_OK)
    {
        return BSP_STATUS_ERROR;
    }

    s_initialized = 0U;
    return BSP_STATUS_OK;
}

BSP_Status_t BSP_ADC_Recover(void)
{
    (void)BSP_ADC_DeInit();
    return BSP_ADC_Init();
}

BSP_Status_t BSP_ADC_ReadRaw(uint32_t *raw)
{
    HAL_StatusTypeDef result;

    if (raw == 0)
    {
        return BSP_STATUS_ERROR;
    }

    BSP_ADC_Init();
    if (s_initialized == 0U)
    {
        return BSP_STATUS_ERROR;
    }

    result = HAL_ADC_Start(&s_hadc);
    if (result != HAL_OK)
    {
        return BSP_ADC_MapHalStatus(result);
    }

    result = HAL_ADC_PollForConversion(&s_hadc, BSP_ADC_TIMEOUT_MS);
    if (result == HAL_OK)
    {
        *raw = HAL_ADC_GetValue(&s_hadc);
    }

    (void)HAL_ADC_Stop(&s_hadc);
    return BSP_ADC_MapHalStatus(result);
}

BSP_Status_t BSP_ADC_ReadAverage(uint32_t *raw, uint8_t count)
{
    uint32_t sum = 0U;
    uint8_t i;

    if ((raw == 0) || (count == 0U))
    {
        return BSP_STATUS_ERROR;
    }

    for (i = 0U; i < count; ++i)
    {
        uint32_t sample = 0U;
        BSP_Status_t result = BSP_ADC_ReadRaw(&sample);

        if (result != BSP_STATUS_OK)
        {
            return result;
        }
        sum += sample;
    }

    *raw = (sum + ((uint32_t)count / 2U)) / (uint32_t)count;
    return BSP_STATUS_OK;
}

BSP_Status_t BSP_ADC_ReadVoltageMv(uint32_t *voltage_mv)
{
    uint32_t raw = 0U;
    uint32_t pin_mv;
    BSP_Status_t result;

    if (voltage_mv == 0)
    {
        return BSP_STATUS_ERROR;
    }

    result = BSP_ADC_ReadAverage(&raw, BSP_ADC_AVERAGE_COUNT);
    if (result != BSP_STATUS_OK)
    {
        return result;
    }

    if (raw > BSP_ADC_RAW_MAX)
    {
        raw = BSP_ADC_RAW_MAX;
    }

    pin_mv = ((raw * BSP_ADC_REF_MV) + (BSP_ADC_RAW_MAX / 2U)) /
             BSP_ADC_RAW_MAX;
    *voltage_mv = (pin_mv * BSP_ADC_DIVIDER_NUM) /
                  BSP_ADC_DIVIDER_DEN;
    return BSP_STATUS_OK;
}
