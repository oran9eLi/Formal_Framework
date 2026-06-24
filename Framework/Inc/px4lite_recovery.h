/**
 * @file px4lite_recovery.h
 * @brief Health task 驱动的模块恢复监控接口。
 *
 * @details
 * 恢复监控只负责根据策略触发注册表中的 recover 回调。recover 回调不得执行
 * 总线 I/O 或阻塞操作，只能设置“需要重新初始化”的请求标志；实际 re-init
 * 必须由资源所属 service task 在自己的上下文完成。
 */

#ifndef PX4LITE_RECOVERY_H
#define PX4LITE_RECOVERY_H

#include "px4lite_types.h"

/**
 * @brief 在 Health task 上下文评估模块恢复策略。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @note 当前恢复策略带 2 s backoff，允许无限重试，保持热插拔友好。
 * 禁用的策略不分配资源，也不执行恢复动作。
 */
void Px4Lite_RecoveryMonitorRun(uint32_t now_ms);

#endif
