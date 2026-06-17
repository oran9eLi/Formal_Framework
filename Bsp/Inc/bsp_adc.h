/**
 * @file bsp_adc.h
 * @brief Declare board ADC access helpers for power sensing.
 */

#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>

#include "bsp_status.h"

/**
 * @brief Initialize the board ADC power-sense channel.
 */
BSP_Status_t BSP_ADC_Init(void);

/**
 * @brief Re-initialize the ADC after a conversion fault.
 */
BSP_Status_t BSP_ADC_Recover(void);

/**
 * @brief Release the ADC peripheral so it consumes no resources.
 */
BSP_Status_t BSP_ADC_DeInit(void);

/**
 * @brief Read one raw ADC conversion.
 */
BSP_Status_t BSP_ADC_ReadRaw(uint32_t *raw);

/**
 * @brief Read several ADC conversions and return a rounded average.
 */
BSP_Status_t BSP_ADC_ReadAverage(uint32_t *raw, uint8_t count);

/**
 * @brief Read the restored input voltage in millivolts.
 */
BSP_Status_t BSP_ADC_ReadVoltageMv(uint32_t *voltage_mv);

#endif
