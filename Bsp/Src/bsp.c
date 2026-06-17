/**
 * @file bsp.c
 * @brief Implement the central board-level initialization entry point.
 */

#include "bsp.h"

#include "bsp_config.h"

#if BSP_ENABLE_I2C
#include "bsp_i2c.h"
#endif
#if BSP_ENABLE_ADC
#include "bsp_adc.h"
#endif
#if BSP_ENABLE_GNSS
#include "bsp_gnss.h"
#endif

/**
 * @brief Initialize every enabled board peripheral from one central entry.
 *
 * The debug UART is intentionally not initialized here: it is owned by the
 * debug console and brought up earlier so the boot banner can print before the
 * rest of the board. Every other shared peripheral (I2C, ADC, GNSS UART/DMA)
 * is initialized exactly once in this function, gated by its BSP_ENABLE_* macro.
 */
BSP_Status_t BSP_Init(void)
{
    BSP_Status_t status = BSP_STATUS_OK;

#if BSP_ENABLE_I2C
    if (BSP_I2C_Init() != BSP_STATUS_OK)
    {
        status = BSP_STATUS_ERROR;
    }
#endif

#if BSP_ENABLE_ADC
    if (BSP_ADC_Init() != BSP_STATUS_OK)
    {
        status = BSP_STATUS_ERROR;
    }
#endif

#if BSP_ENABLE_GNSS
    if (BSP_GNSS_Init() != BSP_STATUS_OK)
    {
        status = BSP_STATUS_ERROR;
    }
#endif

    return status;
}
