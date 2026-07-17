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
/**
 * @brief 更新电机页一路实际 PWM 高电平脉宽显示。
 *
 * @param[in] motor_index 电机序号，范围 0 到 3。
 * @param[in] pulse_us Control 最终写入 PWM 的高电平脉宽，单位：us。
 * @return 更新结果。
 */
Display_Result_t Display_LvglSetMotorPulseUs(uint8_t motor_index, uint16_t pulse_us);
Display_Result_t Display_LvglSetPage(Display_HmiPage_t page);
/**
 * @brief 批量设置告警行(取代 ALARM_ROW1..5 单值变量，支持最多 DISPLAY_LVGL_ALARM_ROWS 行滚动)。
 *
 * @param[in] packed 告警数组，每项 = (source_id<<16)|fault_code，按显示顺序排列。
 * @param[in] count  有效告警条数，超过容量按容量截断，0 表示无告警(显示"无记录")。
 */
void Display_LvglSetAlarmRows(const uint32_t *packed, uint16_t count);

#ifdef __cplusplus
}
#endif

#endif
