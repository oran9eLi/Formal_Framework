/**
 * @file display_lvgl.h
 * @brief Display 层固定容量 LVGL 工作区；对外隐藏控件和导航实现。
 * @details 数据由 Display 门面/App API 提供，禁止直接消费 BSP、驱动私有数据和 topic。
 */

#ifndef DISPLAY_LVGL_H
#define DISPLAY_LVGL_H

#include "display.h"
#include "app_data_api.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 复制 LCD 和触摸独立运行状态。 */
void Display_LvglGetStatus(Display_RuntimeStatus_t *out);

Display_Result_t Display_LvglInit(uint32_t now_ms);
Display_Result_t Display_LvglSelfCheck(uint16_t *error_code);
void Display_LvglRequestRecover(void);
uint8_t Display_LvglNeedsRefresh(void);
Display_Result_t Display_LvglRefreshStep(uint32_t now_ms, uint32_t budget_us);
/** @brief 撤销字段有效性，将活动标签更新为 N/A。 */
Display_Result_t Display_LvglInvalidateValue(Display_HmiVariableId_t id);
Display_Result_t Display_LvglSetValue(Display_HmiVariableId_t id, uint32_t value);
/**
 * @brief 更新电机页一路实际 PWM 高电平脉宽显示。
 *
 * @param[in] motor_index 电机序号，范围 0 到 3。
 * @param[in] pulse_us Control 最终写入 PWM 的高电平脉宽，单位：us。
 * @return 更新结果。
 */
Display_Result_t Display_LvglSetMotorPulseUs(uint8_t motor_index, uint16_t pulse_us);
/** @brief 撤销一路 PWM 实测脉宽有效性，当前和后续页面均显示 N/A。 */
Display_Result_t Display_LvglInvalidateMotorPulseUs(uint8_t motor_index);
Display_Result_t Display_LvglSetPage(Display_HmiPage_t page);
/**
 * @brief 同时复制告警详情和显示行；只接收门面已成功读取的同一份快照。
 *
 * @param[in] alarm 成功读取的完整告警快照；NULL 表示不可用，所有告警字段呈现 N/A。
 * @param[in] packed 告警数组，每项 = (source_id<<16)|fault_code，按显示顺序排列。
 * @param[in] count 有效条数，最多 16 项；只有 alarm 非 NULL 时，0 才表示无活动告警。
 * @details 电源派生行没有对应源记录时，触发时间显示 N/A。调用者与 UI 同属显示任务。
 */
void Display_LvglSetAlarmSnapshot(const App_AlarmSnapshot_t *alarm, const uint32_t *packed, uint16_t count);

/** @brief 在设置日志窗口后同步其生产环版本；清视图依据追加计数，不能传缓存修订号。 */
void Display_LvglSetLogSnapshotVersion(uint32_t source_version);

/** @brief 只冻结观察数据；门面仍须实时刷新控制、告警、事件和时间。 */
uint8_t Display_LvglObservationFrozen(void);

#ifdef __cplusplus
}
#endif

#endif
