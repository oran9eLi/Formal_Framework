/**
 * @file px4lite_remote_telemetry.h
 * @brief Framework层远程显示模式、远端节点表和远程遥测快照管理接口。
 *
 * @details
 * 本模块是远程显示状态的唯一真源。MAVLink RX 只负责逐字段解码，解码后的远端快照通过
 * 本模块提交；Business/Display 只能复制本模块发布的只读快照和节点表。所有缓存均为静态
 * 固定容量，不在运行期分配内存，不扩大 Comm task 栈。
 */

#ifndef PX4LITE_REMOTE_TELEMETRY_H
#define PX4LITE_REMOTE_TELEMETRY_H

#include <stdint.h>
#include "px4lite_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 显示数据源模式。
 */
typedef enum {
  PX4LITE_REMOTE_MODE_LOCAL = 0, /**< 本地模式，显示本机数据，不主动租约远端主数据。 */
  PX4LITE_REMOTE_MODE_REMOTE     /**< 远程模式，显示当前选中远端节点数据并维持续租。 */
} Px4Lite_RemoteMode_t;

/**
 * @brief 初始化远端遥测管理器。
 *
 * @param[in] now_ms 当前系统时间，单位：ms。
 */
void Px4Lite_RemoteTelemetryInit(uint32_t now_ms);

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
 * @param[in] node_id 远端节点 ID，主机为 0，从机从 1 递增。
 * @param[out] out 输出快照，不能为 NULL。
 *
 * @return 复制结果；节点尚无数据时返回 PX4LITE_NOT_READY 并清零输出。
 */
Px4Lite_Result_t Px4Lite_RemoteTelemetryCopyNode(uint8_t node_id, Px4Lite_RemoteTelemetry_t *out);

/**
 * @brief 提交指定远端节点快照。
 *
 * @param[in] node_id 远端节点 ID，主机为 0，从机从 1 递增。
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
