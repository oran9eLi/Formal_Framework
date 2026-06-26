/**
 * @file display_lvgl.h
 * @brief LVGL renderer backend hidden behind the Display facade.
 *
 * @details
 * This module belongs to the Display layer. Business and Framework code must
 * continue to call display.h APIs instead of including LVGL headers directly.
 */

#ifndef DISPLAY_LVGL_H
#define DISPLAY_LVGL_H

#include "display.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

Display_Result_t Display_LvglInit(uint32_t now_ms);
Display_Result_t Display_LvglSelfCheck(uint16_t *error_code);
void Display_LvglRequestRecover(void);
uint8_t Display_LvglNeedsRefresh(void);
Display_Result_t Display_LvglRefreshStep(uint32_t now_ms, uint32_t budget_us);
Display_Result_t Display_LvglSetValue(Display_HmiVariableId_t id, uint32_t value);
Display_Result_t Display_LvglSetPage(Display_HmiPage_t page);
/**
 * @brief 复制 LVGL 后端刷新预算诊断统计。
 *
 * @param[out] out 输出缓冲区，允许为 NULL；为 NULL 时函数不执行任何操作。
 */
void Display_LvglGetDebugStats(Display_DebugStats_t *out);

#ifdef __cplusplus
}
#endif

#endif
