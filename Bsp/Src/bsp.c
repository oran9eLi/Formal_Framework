/**
 * @file bsp.c
 * @brief 实现板级统一初始化入口。
 *
 * @details
 * 本文件属于 BSP 层，只负责把已启用的板级资源按固定顺序初始化，并记录启动诊断位图。
 * Debug UART 不在这里初始化，它归 DebugConsole 所有，避免启动日志依赖完整 BSP 链路。
 */

#include "bsp.h"

#include "bsp_config.h"

#if BSP_ENABLE_I2C
#include "bsp_i2c.h"
#endif
#if BSP_ENABLE_ADC
#include "bsp_adc.h"
#include "bsp_adc2.h"
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
#if BSP_ENABLE_PWM
#include "bsp_pwm.h"
#endif

static BSP_InitDebugInfo_t s_bsp_init_debug;

/**
 * @brief 记录单个 BSP 资源初始化结果。
 *
 * @param[in] mask BSP 资源位，使用 `BSP_INIT_*_MASK`。
 * @param[in] result 单个初始化函数返回值。
 */
static void BSP_RecordInitResult(uint32_t mask, BSP_Status_t result)
{
  s_bsp_init_debug.attempted_mask |= mask;
  if (result != BSP_STATUS_OK) { s_bsp_init_debug.failed_mask |= mask; }
}

/**
 * @brief 从统一入口初始化所有启用的板级外设。
 *
 * @details
 * I2C、ADC、GNSS UART/DMA、RTC、按键和 PWM 等共享资源只在本函数中初始化一次。
 * 函数会尝试完成所有启用项，避免后一个成功覆盖前一个失败；最终返回值和逐项失败位图
 * 可通过 `BSP_GetInitDebugInfo()` 查询。
 */
BSP_Status_t BSP_Init(void)
{
  BSP_Status_t status = BSP_STATUS_OK;
  BSP_Status_t result;

  s_bsp_init_debug.enabled_mask   = 0U;
  s_bsp_init_debug.attempted_mask = 0U;
  s_bsp_init_debug.failed_mask    = 0U;
  s_bsp_init_debug.result         = BSP_STATUS_OK;

#if BSP_ENABLE_I2C
  s_bsp_init_debug.enabled_mask |= BSP_INIT_I2C_MASK;
  result = BSP_I2C_Init();
  BSP_RecordInitResult(BSP_INIT_I2C_MASK, result);
  if (result != BSP_STATUS_OK) { status = BSP_STATUS_ERROR; }
#endif

#if BSP_ENABLE_ADC
  s_bsp_init_debug.enabled_mask |= BSP_INIT_ADC_MASK;
  result = BSP_ADC_Init();
  BSP_RecordInitResult(BSP_INIT_ADC_MASK, result);
  if (result != BSP_STATUS_OK) { status = BSP_STATUS_ERROR; }
  result = BSP_ADC2_Init();
  BSP_RecordInitResult(BSP_INIT_ADC_MASK, result);
  if (result != BSP_STATUS_OK) { status = BSP_STATUS_ERROR; }
#endif

#if BSP_ENABLE_GNSS
  s_bsp_init_debug.enabled_mask |= BSP_INIT_GNSS_MASK;
  result = BSP_GNSS_Init();
  BSP_RecordInitResult(BSP_INIT_GNSS_MASK, result);
  if (result != BSP_STATUS_OK) { status = BSP_STATUS_ERROR; }
#endif

#if BSP_ENABLE_RTC
  s_bsp_init_debug.enabled_mask |= BSP_INIT_RTC_MASK;
  result = BSP_RTC_Init();
  BSP_RecordInitResult(BSP_INIT_RTC_MASK, result);
  if (result != BSP_STATUS_OK) { status = BSP_STATUS_ERROR; }
#endif

#if BSP_ENABLE_BUTTON
  s_bsp_init_debug.enabled_mask |= BSP_INIT_BUTTON_MASK;
  result = BSP_Button_Init();
  BSP_RecordInitResult(BSP_INIT_BUTTON_MASK, result);
  if (result != BSP_STATUS_OK) { status = BSP_STATUS_ERROR; }
#endif

#if BSP_ENABLE_PWM
  s_bsp_init_debug.enabled_mask |= BSP_INIT_PWM_MASK;
  result = BSP_PWM_Init();
  BSP_RecordInitResult(BSP_INIT_PWM_MASK, result);
  if (result != BSP_STATUS_OK) { status = BSP_STATUS_ERROR; }
#endif

  s_bsp_init_debug.result = status;
  return status;
}

/**
 * @brief 复制最近一次 BSP 统一初始化诊断快照。
 *
 * @param[out] out 输出缓冲区，允许为 NULL。
 */
void BSP_GetInitDebugInfo(BSP_InitDebugInfo_t *out)
{
  if (out == 0) { return; }

  *out = s_bsp_init_debug;
}
