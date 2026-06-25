/**
 * @file business_display_placeholder.c
 * @brief 将真实 Display 模块适配到 Business 显示任务钩子。
 */

#include "business_task_template.h"

#include "display.h"

#define BUSINESS_DISPLAY_KEY_DEBOUNCE_CYCLES 3U

static uint8_t s_source_key_stable;
static uint8_t s_source_key_last_raw;
static uint8_t s_source_key_count;

/**
 * @brief 将 Display 层结果转换为 Business 服务结果。
 */
static Business_ServiceResult_t Business_DisplayMapResult(Display_Result_t result)
{
  if (result == DISPLAY_OK) { return BUSINESS_SERVICE_OK; }

  if (result == DISPLAY_NOT_READY) { return BUSINESS_SERVICE_NOT_READY; }

  return BUSINESS_SERVICE_ERROR;
}

/**
 * @brief 对 KEY0 做去抖并返回按下边沿。
 */
static uint8_t Business_DisplaySourceKeyPressEdge(void)
{
  uint8_t raw;
  uint8_t edge = 0U;

  raw = Business_DisplaySourceKeyPressed();
  if (raw == s_source_key_last_raw) {
    if (s_source_key_count < BUSINESS_DISPLAY_KEY_DEBOUNCE_CYCLES) { s_source_key_count++; }
  } else {
    s_source_key_last_raw = raw;
    s_source_key_count    = 1U;
  }

  if ((s_source_key_count >= BUSINESS_DISPLAY_KEY_DEBOUNCE_CYCLES) && (raw != s_source_key_stable)) {
    s_source_key_stable = raw;
    if (raw != 0U) { edge = 1U; }
  }

  return edge;
}

/**
 * @brief 通过真实 Display 模块轮询触摸输入和 KEY0 数据源切换输入。
 */
Business_ServiceResult_t Business_DisplayPollTouch(uint32_t now_ms)
{
  Display_Result_t result;

  (void)now_ms;
  result = Display_PollTouch();
  if (Business_DisplaySourceKeyPressEdge() != 0U) {
    result = Display_ToggleDataSource();
  }

  return Business_DisplayMapResult(result);
}

/**
 * @brief 从应用只读 API 准备一份一致的显示快照。
 */
Business_ServiceResult_t Business_DisplayPrepareSnapshot(uint32_t now_ms)
{
  return Business_DisplayMapResult(Display_PrepareSnapshot(now_ms));
}

/**
 * @brief 推进一次带预算限制的显示刷新步骤。
 */
Business_ServiceResult_t Business_DisplayRefreshStep(uint32_t now_ms, uint32_t budget_us)
{
  Display_Result_t result;

  result = Display_RefreshStep(now_ms, budget_us);
  if (result == DISPLAY_NOT_READY) { return BUSINESS_SERVICE_BUSY; }

  return Business_DisplayMapResult(result);
}
