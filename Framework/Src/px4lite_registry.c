/**
 * @file px4lite_registry.c
 * @brief 实现有界模块注册表和生命周期调度。
 */

#include "px4lite_registry.h"

#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

typedef struct {
  Px4Lite_ModuleDescriptor_t descriptor; /**< 模块静态描述符。 */
  Px4Lite_ModuleRuntime_t runtime;       /**< 模块运行期生命周期状态。 */
  uint8_t registered;                    /**< 1 表示该槽位已注册。 */
} Px4Lite_RegistrySlot_t;

static Px4Lite_RegistrySlot_t s_registry[PX4LITE_MODULE_COUNT];

/**
 * @brief 更新一个模块生命周期状态和状态切换时间。
 */
static void Px4Lite_RegistrySetState(Px4Lite_RegistrySlot_t *slot, Px4Lite_LifecycleState_t state, Px4Lite_Result_t result, uint32_t now_ms)
{
  slot->runtime.state          = state;
  slot->runtime.last_result    = result;
  slot->runtime.state_since_ms = now_ms;
}

/**
 * @brief 执行一个可选生命周期回调，并将空回调视为成功。
 */
static Px4Lite_Result_t Px4Lite_RegistryCall(Px4Lite_LifecycleFn_t callback)
{
  return (callback != 0) ? callback() : PX4LITE_OK;
}

/**
 * @brief 复位模块注册表和生命周期运行态。
 */
Px4Lite_Result_t Px4Lite_RegistryInit(void)
{
  memset(s_registry, 0, sizeof(s_registry));
  return PX4LITE_OK;
}

/**
 * @brief 在调度器启动前注册一个模块描述符。
 */
Px4Lite_Result_t Px4Lite_RegistryRegister(const Px4Lite_ModuleDescriptor_t *descriptor)
{
  Px4Lite_RegistrySlot_t *slot;

  if ((descriptor == 0) || ((uint32_t)descriptor->module_id >= (uint32_t)PX4LITE_MODULE_COUNT) || (descriptor->name == 0)) { return PX4LITE_INVALID_PARAM; }

  slot = &s_registry[descriptor->module_id];
  if (slot->registered != 0U) { return PX4LITE_BUSY; }

  slot->descriptor          = *descriptor;
  slot->registered          = 1U;
  slot->runtime.state       = (descriptor->enabled != 0U) ? PX4LITE_LIFECYCLE_REGISTERED : PX4LITE_LIFECYCLE_DISABLED;
  slot->runtime.last_result = PX4LITE_OK;
  return PX4LITE_OK;
}

/**
 * @brief 依次运行一个模块的 init、self-check 和 start 回调。
 */
Px4Lite_Result_t Px4Lite_RegistryStart(Px4Lite_ModuleId_t module_id, uint32_t now_ms)
{
  Px4Lite_RegistrySlot_t *slot;
  Px4Lite_Result_t result;

  if ((uint32_t)module_id >= (uint32_t)PX4LITE_MODULE_COUNT) { return PX4LITE_INVALID_PARAM; }

  slot = &s_registry[module_id];
  if (slot->registered == 0U) { return PX4LITE_NOT_READY; }
  if (slot->descriptor.enabled == 0U) {
    Px4Lite_RegistrySetState(slot, PX4LITE_LIFECYCLE_DISABLED, PX4LITE_OK, now_ms);
    return PX4LITE_OK;
  }
  if (slot->runtime.state == PX4LITE_LIFECYCLE_RUNNING) { return PX4LITE_OK; }

  Px4Lite_RegistrySetState(slot, PX4LITE_LIFECYCLE_INITIALIZING, PX4LITE_OK, now_ms);
  result = Px4Lite_RegistryCall(slot->descriptor.init);
  if (result != PX4LITE_OK) {
    Px4Lite_RegistrySetState(slot, PX4LITE_LIFECYCLE_FAILED, result, now_ms);
    return result;
  }

  Px4Lite_RegistrySetState(slot, PX4LITE_LIFECYCLE_SELF_CHECK, PX4LITE_OK, now_ms);
  result = Px4Lite_RegistryCall(slot->descriptor.self_check);
  if (result != PX4LITE_OK) {
    Px4Lite_RegistrySetState(slot, PX4LITE_LIFECYCLE_FAILED, result, now_ms);
    return result;
  }

  Px4Lite_RegistrySetState(slot, PX4LITE_LIFECYCLE_STARTING, PX4LITE_OK, now_ms);
  result = Px4Lite_RegistryCall(slot->descriptor.start);
  if (result != PX4LITE_OK) {
    Px4Lite_RegistrySetState(slot, PX4LITE_LIFECYCLE_FAILED, result, now_ms);
    return result;
  }

  if (slot->runtime.start_count < 65535U) { slot->runtime.start_count++; }
  Px4Lite_RegistrySetState(slot, PX4LITE_LIFECYCLE_RUNNING, PX4LITE_OK, now_ms);
  return PX4LITE_OK;
}

/**
 * @brief 运行一个模块的 stop 回调并更新生命周期状态。
 */
Px4Lite_Result_t Px4Lite_RegistryStop(Px4Lite_ModuleId_t module_id, uint32_t now_ms)
{
  Px4Lite_RegistrySlot_t *slot;
  Px4Lite_Result_t result;

  if ((uint32_t)module_id >= (uint32_t)PX4LITE_MODULE_COUNT) { return PX4LITE_INVALID_PARAM; }

  slot = &s_registry[module_id];
  if (slot->registered == 0U) { return PX4LITE_NOT_READY; }

  result = Px4Lite_RegistryCall(slot->descriptor.stop);
  Px4Lite_RegistrySetState(slot, (result == PX4LITE_OK) ? PX4LITE_LIFECYCLE_STOPPED : PX4LITE_LIFECYCLE_FAILED, result, now_ms);
  return result;
}

/**
 * @brief 运行一个模块的 recover 回调并重新进入启动流程。
 */
Px4Lite_Result_t Px4Lite_RegistryRecover(Px4Lite_ModuleId_t module_id, uint32_t now_ms)
{
  Px4Lite_RegistrySlot_t *slot;
  Px4Lite_Result_t result;

  if ((uint32_t)module_id >= (uint32_t)PX4LITE_MODULE_COUNT) { return PX4LITE_INVALID_PARAM; }

  slot = &s_registry[module_id];
  if (slot->registered == 0U) { return PX4LITE_NOT_READY; }

  Px4Lite_RegistrySetState(slot, PX4LITE_LIFECYCLE_RECOVERING, PX4LITE_OK, now_ms);
  result = Px4Lite_RegistryCall(slot->descriptor.recover);
  if (result != PX4LITE_OK) {
    Px4Lite_RegistrySetState(slot, PX4LITE_LIFECYCLE_FAILED, result, now_ms);
    return result;
  }
  if (slot->runtime.recovery_count < 65535U) { slot->runtime.recovery_count++; }

  slot->runtime.state = PX4LITE_LIFECYCLE_REGISTERED;
  return Px4Lite_RegistryStart(module_id, now_ms);
}

/**
 * @brief 复制一个已注册模块的描述符和生命周期状态。
 */
Px4Lite_Result_t Px4Lite_RegistryGet(Px4Lite_ModuleId_t module_id, Px4Lite_ModuleDescriptor_t *descriptor, Px4Lite_ModuleRuntime_t *runtime)
{
  Px4Lite_RegistrySlot_t *slot;

  if (((uint32_t)module_id >= (uint32_t)PX4LITE_MODULE_COUNT) || ((descriptor == 0) && (runtime == 0))) { return PX4LITE_INVALID_PARAM; }

  taskENTER_CRITICAL();
  slot = &s_registry[module_id];
  if (slot->registered != 0U) {
    if (descriptor != 0) { *descriptor = slot->descriptor; }
    if (runtime != 0) { *runtime = slot->runtime; }
  }
  taskEXIT_CRITICAL();

  return (slot->registered != 0U) ? PX4LITE_OK : PX4LITE_NOT_READY;
}
