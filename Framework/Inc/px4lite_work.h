/**
 * @file px4lite_work.h
 * @brief 声明固定周期工作项和执行统计接口。
 *
 * @details
 * WorkItem 用于把 FreeRTOS 任务中的周期性业务封装成可统计、可跳帧的执行单元。
 * 调度时间字段使用真实毫秒值，不能填入 RTOS tick。
 */

#ifndef PX4LITE_WORK_H
#define PX4LITE_WORK_H

#include "px4lite_types.h"

/**
 * @brief 固定周期工作项执行函数类型。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 */
typedef void (*Px4Lite_WorkFn_t)(uint32_t now_ms);

/**
 * @brief 固定周期工作项运行状态。
 */
typedef struct {
  const char *name;             /**< 调试名称，生命周期必须长于工作项。 */
  Px4Lite_WorkFn_t run;         /**< 到期后执行的非阻塞工作函数。 */
  uint32_t period_ms;           /**< 工作周期，单位 ms。 */
  uint32_t next_run_ms;         /**< 下一次计划运行时间，单位 ms。 */
  uint32_t run_count;           /**< 已执行次数。 */
  uint32_t deadline_miss_count; /**< 检测到的周期滞后次数。 */
  uint32_t max_execution_us;    /**< 单次执行最长耗时，单位 us。 */
  uint8_t initialized;          /**< 1 表示工作项已经初始化。 */
} Px4Lite_WorkItem_t;

/**
 * @brief 初始化一个固定周期工作项。
 *
 * @param[out] item 工作项实例，不能为 NULL。
 * @param[in] name 调试名称，不能为 NULL。
 * @param[in] period_ms 工作周期，单位 ms，不能为 0。
 * @param[in] start_ms 首次到期时间，单位 ms。
 * @param[in] run 到期后执行的工作函数，不能为 NULL。
 *
 * @retval PX4LITE_OK 初始化成功。
 * @retval PX4LITE_INVALID_PARAM 参数非法。
 */
Px4Lite_Result_t Px4Lite_WorkInit(Px4Lite_WorkItem_t *item, const char *name, uint32_t period_ms, uint32_t start_ms, Px4Lite_WorkFn_t run);

/**
 * @brief 在到期时运行一个工作项并更新执行统计。
 *
 * @param[in,out] item 工作项实例，不能为 NULL。
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @retval PX4LITE_OK 已执行一次工作函数。
 * @retval PX4LITE_IDLE 尚未到期。
 * @retval PX4LITE_INVALID_PARAM 工作项未初始化或参数非法。
 */
Px4Lite_Result_t Px4Lite_WorkRunDue(Px4Lite_WorkItem_t *item, uint32_t now_ms);

#endif
