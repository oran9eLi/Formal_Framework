/**
 * @file px4lite_recovery.c
 * @brief 实现通用的注册表驱动模块恢复监督。
 *
 * @details
 * 恢复策略运行在 Health 任务：发现异常注册模块后，按固定退避调用其 recover 回调。
 * recover 回调必须很轻，只能请求模块所属 Service 任务后续执行重初始化，不能在
 * Health 任务中访问共享总线。新增模块只要在注册表中提供 recover 回调，即自动纳入
 * 通用恢复监督。
 */

#include "px4lite_recovery.h"

#include "px4lite_config.h"
#include "px4lite_modules.h"
#include "px4lite_registry.h"

#if PX4LITE_RECOVERY_ENABLE
static uint32_t s_last_attempt_ms[PX4LITE_MODULE_COUNT];
static uint16_t s_attempt_count[PX4LITE_MODULE_COUNT];

/**
 * @brief 判断一个模块状态是否需要尝试恢复。
 *
 * @details 只按 OFFLINE/FAILED 判定，不能再看 consecutive_errors：
 * Px4Lite_SetExternalModuleState() 对任何非 ONLINE 状态都累加该计数，STARTING 属于正常
 * 启动过程而不是错误，计数在启动期间必然越过阈值。调用者已在 ONLINE/DEGRADED 时清零并
 * 跳过，因此该计数唯一还能命中的就是 STARTING/UNINITIALIZED/DISABLED——正是本函数下方
 * 注释声明要交给启动流程处理的状态。
 *
 * 曾导致 Control 永久停在 ESC 预解锁：预解锁 2500ms 期间每 20ms 发布一次 STARTING，
 * 100ms 就凑满阈值，恢复监视器每 2000ms 调用 Px4Lite_ControlRecover() 重置 Control，
 * 既清零四路油门目标(滑轨自己掉回 0)，又把解锁计时重新拨回起点(ESC 永远解锁不了)。
 *
 * @param[in] status 模块状态，不能为 NULL。
 *
 * @return 1 表示需要恢复，0 表示不需要。
 */
static uint8_t Px4Lite_RecoveryNeeded(const Px4Lite_ModuleStatus_t *status)
{
  return ((status->state == PX4LITE_STATE_OFFLINE) || (status->state == PX4LITE_STATE_FAILED)) ? 1U : 0U;
}
#endif

/**
 * @brief 评估所有带 recover 回调的已注册模块。
 */
void Px4Lite_RecoveryMonitorRun(uint32_t now_ms)
{
#if PX4LITE_RECOVERY_ENABLE
  uint32_t id;

  for (id = 0U; id < (uint32_t)PX4LITE_MODULE_COUNT; ++id) {
    Px4Lite_ModuleDescriptor_t descriptor;
    Px4Lite_ModuleStatus_t status;

    /* 跳过未注册槽位。 */
    if (Px4Lite_RegistryGet((Px4Lite_ModuleId_t)id, &descriptor, 0) != PX4LITE_OK) { continue; }
    if ((descriptor.enabled == 0U) || (descriptor.recover == 0)) { continue; }
    if (Px4Lite_GetModuleStatus((Px4Lite_ModuleId_t)id, &status) != PX4LITE_OK) { continue; }

    /* 健康或仅数据降级时清空恢复计数。 */
    if ((status.state == PX4LITE_STATE_ONLINE) || (status.state == PX4LITE_STATE_DEGRADED)) {
      s_attempt_count[id]   = 0U;
      s_last_attempt_ms[id] = now_ms;
      continue;
    }

    /* STARTING/UNINITIALIZED/DISABLED 交给启动流程处理。 */
    if (Px4Lite_RecoveryNeeded(&status) == 0U) { continue; }

    /*
     * 对恢复尝试做限速，但不设置总次数上限；设备长时间断开后重新接入仍可恢复，
     * 延迟只用于限制尝试频率。
     */
    if ((uint32_t)(now_ms - s_last_attempt_ms[id]) < PX4LITE_RECOVERY_DELAY_MS) { continue; }

    s_last_attempt_ms[id] = now_ms;
    if (s_attempt_count[id] < 0xFFFFU) { s_attempt_count[id]++; }

    /* 轻量请求型回调；真正 re-init 由模块所属 Service 执行。 */
    (void)descriptor.recover();
  }
#else
  (void)now_ms;
#endif
}
