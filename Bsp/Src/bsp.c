/**
 * @file bsp.c
 * @brief 实现板级统一初始化入口。
 */

#include "bsp.h"

#include "bsp_config.h"

#if BSP_ENABLE_I2C
#include "bsp_i2c.h"
#endif
#if BSP_ENABLE_ADC
#include "bsp_adc.h"
#endif
#if BSP_ENABLE_ADC_CURRENT
#include "bsp_adc_current.h"
#endif
#if BSP_ENABLE_GNSS
#include "bsp_gnss.h"
#endif
#if BSP_ENABLE_RTC
#include "bsp_rtc.h"
#endif
#if BSP_ENABLE_BUTTON
#include "bsp_button.h"
#endif

/**
 * @brief 从统一入口初始化所有启用的板级外设。
 *
 * @details
 * 调试 UART 不在这里初始化，它归 debug console 所有，并需要在板级其余外设之前
 * 打印启动横幅。I2C、ADC、GNSS UART/DMA 等共享外设只在本函数中初始化一次。
 */
BSP_Status_t BSP_Init(void)
{
  BSP_Status_t status = BSP_STATUS_OK;

#if BSP_ENABLE_I2C
  if (BSP_I2C_Init() != BSP_STATUS_OK) { status = BSP_STATUS_ERROR; }
#endif

#if BSP_ENABLE_ADC
  if (BSP_ADC_Init() != BSP_STATUS_OK) { status = BSP_STATUS_ERROR; }
#endif

#if BSP_ENABLE_ADC_CURRENT
  if (BSP_ADC_Current_Init() != BSP_STATUS_OK) { status = BSP_STATUS_ERROR; }
#endif

#if BSP_ENABLE_GNSS
  if (BSP_GNSS_Init() != BSP_STATUS_OK) { status = BSP_STATUS_ERROR; }
#endif

#if BSP_ENABLE_RTC
  if (BSP_RTC_Init() != BSP_STATUS_OK) { status = BSP_STATUS_ERROR; }
#endif

#if BSP_ENABLE_BUTTON
  if (BSP_Button_Init() != BSP_STATUS_OK) { status = BSP_STATUS_ERROR; }
#endif

  return status;
}
