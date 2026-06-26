/**
 * @file storage_module_event.h
 * @brief 将 Framework 模块状态变化转换为 Storage EVENT 日志事件。
 */

#ifndef STORAGE_MODULE_EVENT_H
#define STORAGE_MODULE_EVENT_H

#include <stdint.h>
#include "px4lite_types.h"

/**
 * @brief 模块状态变化事件。
 */
typedef struct {
  const char *source;  /**< 模块名称，例如 IMU。 */
  uint32_t state;      /**< 当前模块状态。 */
  uint32_t fault;      /**< 当前故障码，恢复事件为 0。 */
  uint32_t severity;   /**< 当前严重度，恢复事件为 0。 */
  uint8_t active;      /**< 1 表示异常触发，0 表示状态恢复。 */
  uint32_t count;      /**< 同一模块异常累计触发次数。 */
  const char *message; /**< 事件说明，例如 imu_offline。 */
} Storage_ModuleEvent_t;

void StorageModuleEvent_Init(void);
Px4Lite_Result_t StorageModuleEvent_Update(const Px4Lite_ModuleStatus_t *status, Storage_ModuleEvent_t *event);

#endif
