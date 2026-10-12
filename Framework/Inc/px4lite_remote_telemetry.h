/**
 * @file px4lite_remote_telemetry.h
 * @brief Framework层远程显示模式、远端节点表和远程遥测快照管理接口。
 *
 * @details
 * 本模块是远程显示状态的唯一真源。MAVLink RX 只负责逐字段解码，解码后的远端快照通过
 * 本模块提交；Business/Display 只能复制本模块发布的只读快照和节点表。所有缓存均为静态
 * 固定容量，不在运行期分配内存；结构与任务栈容量随字段变更一起核验。
 */

#ifndef PX4LITE_REMOTE_TELEMETRY_H
#define PX4LITE_REMOTE_TELEMETRY_H

#include <stdint.h>
#include "px4lite_types.h"
#include "px4lite_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 显示数据源模式。
 */
typedef enum {
  PX4LITE_REMOTE_MODE_LOCAL = 0, /**< 本地模式，显示本机数据。 */
  PX4LITE_REMOTE_MODE_REMOTE     /**< 远程模式，显示当前选中远端节点数据。 */
} Px4Lite_RemoteMode_t;

/**
 * @brief 初始化远端遥测管理器。
 *
 * @param[in] now_ms 当前系统时间，单位：ms。
 */
void Px4Lite_RemoteTelemetryInit(uint32_t now_ms);

/**
 * @brief LoRa 链路断开时清空远端节点和遥测缓存。
 * @param[in] now_ms 当前系统时间，单位：ms。
 * @note 链路断开后远端视图已经失效，复位时回到 LOCAL 模式并清除已发现节点、选中节点和远端数据。
 */
void Px4Lite_RemoteTelemetryResetLink(uint32_t now_ms);

/**
 * @brief 根据真实毫秒时刻刷新调用方快照的各域及导航子字段过期位。
 * @param[in,out] remote 调用方持有的快照；不访问内部槽位。
 * @param[in] now_ms 当前时刻，单位：ms；支持 uint32_t 回绕，比较跨度应小于半周期。
 */
static inline void Px4Lite_RemoteTelemetryRefreshFreshness(Px4Lite_RemoteTelemetry_t *remote, uint32_t now_ms)
{
  uint32_t stale = 0U;
  uint32_t nav_stale = 0U;
  uint32_t battery_stale = 0U;
  uint32_t environment_stale = 0U;

  if (remote == 0) { return; }
  if (((remote->navigation_valid_mask & PX4LITE_NAV_VALID_POSITION) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->position_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { nav_stale |= PX4LITE_NAV_VALID_POSITION; }
  if (((remote->navigation_valid_mask & PX4LITE_NAV_VALID_VELOCITY) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->velocity_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { nav_stale |= PX4LITE_NAV_VALID_VELOCITY; }
  if (((remote->navigation_valid_mask & PX4LITE_NAV_VALID_TIME) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->time_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { nav_stale |= PX4LITE_NAV_VALID_TIME; }
  if (((remote->navigation_valid_mask & PX4LITE_NAV_VALID_ALTITUDE) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->altitude_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { nav_stale |= PX4LITE_NAV_VALID_ALTITUDE; }
  remote->navigation_stale_mask = nav_stale;
  if (remote->navigation_valid_mask != 0U) { remote->valid_mask |= PX4LITE_REMOTE_VALID_NAVIGATION; }
  if (((remote->battery_valid_mask & PX4LITE_REMOTE_BATTERY_VALID_PRIMARY) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->battery1_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { battery_stale |= PX4LITE_REMOTE_BATTERY_VALID_PRIMARY; }
  if (((remote->battery_valid_mask & PX4LITE_REMOTE_BATTERY_VALID_SECONDARY) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->battery2_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { battery_stale |= PX4LITE_REMOTE_BATTERY_VALID_SECONDARY; }
  remote->battery_stale_mask = battery_stale;
  if (remote->battery_valid_mask != 0U) { remote->valid_mask |= PX4LITE_REMOTE_VALID_BATTERY; }
  if (((remote->environment_valid_mask & PX4LITE_REMOTE_ENV_VALID_PRESSURE_TEMPERATURE) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->pressure_temperature_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { environment_stale |= PX4LITE_REMOTE_ENV_VALID_PRESSURE_TEMPERATURE; }
  if (((remote->environment_valid_mask & PX4LITE_REMOTE_ENV_VALID_HUMIDITY) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->humidity_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { environment_stale |= PX4LITE_REMOTE_ENV_VALID_HUMIDITY; }
  remote->environment_stale_mask = environment_stale;
  if (remote->environment_valid_mask != 0U) { remote->valid_mask |= PX4LITE_REMOTE_VALID_ENVIRONMENT; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_HEARTBEAT) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->heartbeat_update_ms) > PX4LITE_REMOTE_HEARTBEAT_STALE_MS)) { stale |= PX4LITE_REMOTE_VALID_HEARTBEAT; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_NAVIGATION) != 0U) && ((remote->navigation_valid_mask & ~nav_stale) == 0U)) { stale |= PX4LITE_REMOTE_VALID_NAVIGATION; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_ATTITUDE) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->attitude_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { stale |= PX4LITE_REMOTE_VALID_ATTITUDE; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_ENVIRONMENT) != 0U) && ((remote->environment_valid_mask & ~environment_stale) == 0U)) { stale |= PX4LITE_REMOTE_VALID_ENVIRONMENT; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_BATTERY) != 0U) && ((remote->battery_valid_mask & ~battery_stale) == 0U)) { stale |= PX4LITE_REMOTE_VALID_BATTERY; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_MODULES) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->modules_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { stale |= PX4LITE_REMOTE_VALID_MODULES; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_ALARM) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->alarm_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { stale |= PX4LITE_REMOTE_VALID_ALARM; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_MOTOR) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->motor_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { stale |= PX4LITE_REMOTE_VALID_MOTOR; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_LOG) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->log_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { stale |= PX4LITE_REMOTE_VALID_LOG; }
  remote->stale_mask = stale;
}



/**
 * @brief 设置本地/远程显示模式。
 *
 * @param[in] mode 目标模式。
 * @param[in] now_ms 当前系统时间，单位：ms。
 *
 * @return 设置结果。
 */
Px4Lite_Result_t Px4Lite_RemoteTelemetrySetMode(Px4Lite_RemoteMode_t mode, uint32_t now_ms);

/**
 * @brief 获取当前显示模式。
 *
 * @return 当前显示模式。
 */
Px4Lite_RemoteMode_t Px4Lite_RemoteTelemetryGetMode(void);

/**
 * @brief 复制指定远端节点快照，供 MAVLink RX 在栈外 scratch 中增量更新。
 *
 * @param[in] node_id 远端节点 ID，由 DCDW-xxx 数字后缀派生。
 * @param[out] out 输出快照，不能为 NULL。
 *
 * @return 复制结果；节点尚无数据时返回 PX4LITE_NOT_READY 并清零输出。
 */
Px4Lite_Result_t Px4Lite_RemoteTelemetryCopyNode(uint8_t node_id, Px4Lite_RemoteTelemetry_t *out);

/**
 * @brief 提交指定远端节点快照。
 *
 * @param[in] node_id 远端节点 ID，由 DCDW-xxx 数字后缀派生。
 * @param[in] snapshot 已解码的远端快照，不能为 NULL。
 * @param[in] now_ms 当前系统时间，单位：ms。
 *
 * @return 提交结果。
 */
Px4Lite_Result_t Px4Lite_RemoteTelemetryCommitNode(uint8_t node_id, const Px4Lite_RemoteTelemetry_t *snapshot, uint32_t now_ms);

/**
 * @brief 复制当前选中节点的远程显示快照。
 *
 * @param[out] out 输出快照，不能为 NULL。
 *
 * @return 复制结果。
 */
Px4Lite_Result_t Px4Lite_CopyRemoteTelemetry(Px4Lite_RemoteTelemetry_t *out);

/**
 * @brief 选择默认远程显示节点。
 *
 * @param[in] node_id 远端节点 ID。
 *
 * @return 选择结果。
 */
Px4Lite_Result_t Px4Lite_SelectRemoteNode(uint8_t node_id);

/**
 * @brief 获取当前选中的远程显示节点。
 *
 * @param[out] node_id 输出远端节点 ID，不能为 NULL。
 *
 * @return 获取结果。
 */
Px4Lite_Result_t Px4Lite_GetSelectedRemoteNode(uint8_t *node_id);

/**
 * @brief 复制远端节点状态表。
 *
 * @param[out] out 输出数组，不能为 NULL。
 * @param[in] capacity 输出数组容量。
 * @param[out] count 实际写入条数，不能为 NULL。
 * @param[in] now_ms 当前系统时间，单位：ms。
 *
 * @return 复制结果。
 */
Px4Lite_Result_t Px4Lite_CopyRemoteNodeStatuses(Px4Lite_RemoteNodeStatus_t *out, uint8_t capacity, uint8_t *count, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
