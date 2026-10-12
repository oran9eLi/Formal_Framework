/**
 * @file px4lite_remote_telemetry.c
 * @brief Framework层远程显示模式、远端节点表和远程遥测快照管理实现。
 *
 * @details
 * 本文件只保存 MAVLink RX 解码后的远端只读快照，不解析 MAVLink 字节流，也不直接发送
 * COMMAND_LONG。Comm task 是写入者，Business/Display 是读取者；临界区内只复制固定大小
 * 快照和更新序号，不执行阻塞 I/O。
 */

#include "px4lite_remote_telemetry.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "px4lite_config.h"
#include "px4lite_identity.h"
#include "px4lite_platform.h"

static Px4Lite_RemoteMode_t s_mode;
static Px4Lite_RemoteTelemetry_t s_remote[PX4LITE_REMOTE_NODE_MAX];
static uint32_t s_remote_sequence[PX4LITE_REMOTE_NODE_MAX];
static uint8_t s_slot_node[PX4LITE_REMOTE_NODE_MAX]; /* 每槽绑定的对端 node_id(=sysid,1..250)，0=空闲 */
static uint8_t s_selected_remote_node_id;
static uint32_t s_mode_changed_ms;

/**
 * @brief 在调用者已持有临界区时按节点身份查找固定槽。
 * node_id(对端 sysid) 范围 1..250，无法直接索引 NODE_MAX 个槽的小表，
 * 改为动态槽位分配：本函数仅查找已绑定该 node_id 的槽(读取/选择用，不分配)，找不到返回 0。
 */
static uint8_t RemoteTelemetry_NodeIdToIndex(uint8_t node_id, uint8_t *index)
{
  uint8_t i;

  if ((index == 0) || (node_id == 0U)) { return 0U; }
  for (i = 0U; i < PX4LITE_REMOTE_NODE_MAX; i++) {
    if (s_slot_node[i] == node_id) { *index = i; return 1U; }
  }
  return 0U;
}

/**
 * @brief 写者持有临界区时分配槽，按相对 now_ms 的年龄淘汰最旧槽。
 * 收帧写入用：查找已绑定槽；无则分配空槽；表满则淘汰 last_rx_ms 最旧的槽。
 * 分配/淘汰时清空该槽的旧遥测与序号，避免残留脏数据。
 */
static uint8_t RemoteTelemetry_AllocSlot(uint8_t node_id, uint8_t *index, uint32_t now_ms)
{
  uint8_t i;
  uint8_t oldest = 0U;
  uint32_t oldest_age = 0U;
  uint32_t age;

  if ((index == 0) || (node_id == 0U)) { return 0U; }
  if (RemoteTelemetry_NodeIdToIndex(node_id, index) != 0U) { return 1U; }

  for (i = 0U; i < PX4LITE_REMOTE_NODE_MAX; i++) {
    if (s_slot_node[i] == 0U) {
      s_slot_node[i] = node_id;
      memset(&s_remote[i], 0, sizeof(s_remote[i]));
      s_remote_sequence[i] = 0U;
      *index = i;
      return 1U;
    }
  }

  for (i = 0U; i < PX4LITE_REMOTE_NODE_MAX; i++) {
    age = Px4Lite_ElapsedMs(now_ms, s_remote[i].last_rx_ms);
    if (age > oldest_age) { oldest_age = age; oldest = i; }
  }
  s_slot_node[oldest] = node_id;
  memset(&s_remote[oldest], 0, sizeof(s_remote[oldest]));
  s_remote_sequence[oldest] = 0U;
  *index = oldest;
  return 1U;
}

/** @brief 初始化为未选中任何节点。 */
static uint8_t RemoteTelemetry_DefaultNode(void)
{
  return 0U; /* 0 = 无选中 */
}

/** @brief 只比较已经收到的数据域，以无符号年龄选择最近一次真实数据时刻。 */
static uint32_t RemoteTelemetry_LastDataMs(const Px4Lite_RemoteTelemetry_t *remote, uint32_t now_ms)
{
  uint32_t last_data_ms = 0U;
  uint32_t best_age = UINT32_MAX;
  uint32_t age;
  uint32_t i;
  uint32_t times[8];
  static const uint32_t bits[8] = {PX4LITE_REMOTE_VALID_NAVIGATION, PX4LITE_REMOTE_VALID_ATTITUDE,
    PX4LITE_REMOTE_VALID_ENVIRONMENT, PX4LITE_REMOTE_VALID_BATTERY, PX4LITE_REMOTE_VALID_MODULES,
    PX4LITE_REMOTE_VALID_ALARM, PX4LITE_REMOTE_VALID_MOTOR, PX4LITE_REMOTE_VALID_LOG};
  times[0] = remote->navigation_update_ms; times[1] = remote->attitude_update_ms;
  times[2] = remote->environment_update_ms; times[3] = remote->battery_update_ms;
  times[4] = remote->modules_update_ms; times[5] = remote->alarm_update_ms;
  times[6] = remote->motor_update_ms; times[7] = remote->log_update_ms;
  for (i = 0U; i < 8U; i++) {
    age = Px4Lite_ElapsedMs(now_ms, times[i]);
    if (((remote->valid_mask & bits[i]) != 0U) && (age < best_age)) { best_age = age; last_data_ms = times[i]; }
  }
  return last_data_ms;
}

/** @brief 初始化八槽遥测缓存与本机模式，所有共享状态同区复位。 */
void Px4Lite_RemoteTelemetryInit(uint32_t now_ms)
{
  taskENTER_CRITICAL();
  memset(s_remote, 0, sizeof(s_remote));
  memset(s_remote_sequence, 0, sizeof(s_remote_sequence));
  memset(s_slot_node, 0, sizeof(s_slot_node));
  s_mode = PX4LITE_REMOTE_MODE_LOCAL;
  s_selected_remote_node_id = RemoteTelemetry_DefaultNode();
  s_mode_changed_ms = now_ms;
  taskEXIT_CRITICAL();
}

/** @brief 链路复位撤销节点绑定、缓存和选择状态。 */
void Px4Lite_RemoteTelemetryResetLink(uint32_t now_ms)
{
  taskENTER_CRITICAL();
  memset(s_remote, 0, sizeof(s_remote));
  memset(s_remote_sequence, 0, sizeof(s_remote_sequence));
  memset(s_slot_node, 0, sizeof(s_slot_node));
  s_mode = PX4LITE_REMOTE_MODE_LOCAL;
  s_selected_remote_node_id = RemoteTelemetry_DefaultNode();
  s_mode_changed_ms = now_ms;
  taskEXIT_CRITICAL();
}

/** @brief 设置合法显示模式并记录真实毫秒变更时刻。 */
Px4Lite_Result_t Px4Lite_RemoteTelemetrySetMode(Px4Lite_RemoteMode_t mode, uint32_t now_ms)
{
  if ((mode != PX4LITE_REMOTE_MODE_LOCAL) && (mode != PX4LITE_REMOTE_MODE_REMOTE)) { return PX4LITE_INVALID_PARAM; }

  taskENTER_CRITICAL();
  if (s_mode != mode) {
    s_mode = mode;
    s_mode_changed_ms = now_ms;
  }
  taskEXIT_CRITICAL();
  return PX4LITE_OK;
}

/** @brief 复制当前显示模式。 */
Px4Lite_RemoteMode_t Px4Lite_RemoteTelemetryGetMode(void)
{
  Px4Lite_RemoteMode_t mode;

  taskENTER_CRITICAL();
  mode = s_mode;
  taskEXIT_CRITICAL();
  return mode;
}

/** @brief 同一临界区内完成身份查找、核验与复制，未绑定节点返回空快照。 */
Px4Lite_Result_t Px4Lite_RemoteTelemetryCopyNode(uint8_t node_id, Px4Lite_RemoteTelemetry_t *out)
{
  uint8_t index;

  if (out == 0) { return PX4LITE_INVALID_PARAM; }
  /* 未知 node_id(尚无绑定槽) 返回 NOT_READY 而非 INVALID_PARAM：
     收帧首帧走此路径时，RxRun 需据 NOT_READY 清零后继续 decode+CommitNode 才能首次分配槽。
     若返回 INVALID_PARAM，RxRun 会提前 return，新节点永远建不了槽、进不了表。 */
  taskENTER_CRITICAL();
  if (RemoteTelemetry_NodeIdToIndex(node_id, &index) == 0U) {
    taskEXIT_CRITICAL();
    memset(out, 0, sizeof(*out));
    return PX4LITE_NOT_READY;
  }

  if ((s_remote[index].header.valid == 0U) || (s_remote[index].system_id != node_id)) {
    taskEXIT_CRITICAL();
    memset(out, 0, sizeof(*out));
    return PX4LITE_NOT_READY;
  }
  *out = s_remote[index];
  taskEXIT_CRITICAL();
  return PX4LITE_OK;
}

/** @brief Comm 单写者提交快照；槽位分配、重绑定与发布不可被读者拆开观察。 */
Px4Lite_Result_t Px4Lite_RemoteTelemetryCommitNode(uint8_t node_id, const Px4Lite_RemoteTelemetry_t *snapshot, uint32_t now_ms)
{
  Px4Lite_RemoteTelemetry_t local;
  uint8_t index;

  if ((snapshot == 0) || (node_id == 0U) || (node_id > 250U) || (snapshot->system_id != node_id)) { return PX4LITE_INVALID_PARAM; }
  if (node_id == (uint8_t)Px4Lite_IdentityGetNodeId()) { return PX4LITE_INVALID_PARAM; }

  local = *snapshot;
  Px4Lite_RemoteTelemetryRefreshFreshness(&local, now_ms);
  local.header.sample_time_ms = (local.header.sample_time_ms != 0U) ? local.header.sample_time_ms : local.last_rx_ms;
  local.header.publish_time_ms = now_ms;
  local.header.flags = PX4LITE_DATA_VALID;
  local.header.valid = 1U;
  local.header.quality = 100U;

  taskENTER_CRITICAL();
  if (RemoteTelemetry_AllocSlot(node_id, &index, now_ms) == 0U) { taskEXIT_CRITICAL(); return PX4LITE_INVALID_PARAM; }
  local.header.sequence = ++s_remote_sequence[index];
  s_remote[index] = local;
  taskEXIT_CRITICAL();
  return PX4LITE_OK;
}

/** @brief 复制选中节点，并按调用时真实毫秒重新判断所有字段年龄。 */
Px4Lite_Result_t Px4Lite_CopyRemoteTelemetry(Px4Lite_RemoteTelemetry_t *out)
{
  uint8_t node_id;
  Px4Lite_Result_t result;

  if (out == 0) { return PX4LITE_INVALID_PARAM; }

  taskENTER_CRITICAL();
  node_id = s_selected_remote_node_id;
  taskEXIT_CRITICAL();

  result = Px4Lite_RemoteTelemetryCopyNode(node_id, out);
  if (result == PX4LITE_OK) { Px4Lite_RemoteTelemetryRefreshFreshness(out, Px4Lite_PlatformGetMs()); }
  return result;
}

/** @brief 在槽身份确认后原子更新所选节点。 */
Px4Lite_Result_t Px4Lite_SelectRemoteNode(uint8_t node_id)
{
  uint8_t index;

  if (node_id == (uint8_t)Px4Lite_IdentityGetNodeId()) { return PX4LITE_INVALID_PARAM; }
  taskENTER_CRITICAL();
  if (RemoteTelemetry_NodeIdToIndex(node_id, &index) == 0U) { taskEXIT_CRITICAL(); return PX4LITE_INVALID_PARAM; }
  (void)index;
  s_selected_remote_node_id = node_id;
  taskEXIT_CRITICAL();
  return PX4LITE_OK;
}

/** @brief 原子复制选中节点身份。 */
Px4Lite_Result_t Px4Lite_GetSelectedRemoteNode(uint8_t *node_id)
{
  if (node_id == 0) { return PX4LITE_INVALID_PARAM; }

  taskENTER_CRITICAL();
  *node_id = s_selected_remote_node_id;
  taskEXIT_CRITICAL();
  return PX4LITE_OK;
}

/** @brief 每个槽的身份与负载同区复制，时间比较支持 uint32_t 回绕。 */
Px4Lite_Result_t Px4Lite_CopyRemoteNodeStatuses(Px4Lite_RemoteNodeStatus_t *out, uint8_t capacity, uint8_t *count, uint32_t now_ms)
{
  Px4Lite_RemoteTelemetry_t local;
  uint8_t i;
  uint8_t written = 0U;
  uint8_t node_id;

  if ((out == 0) || (count == 0)) { return PX4LITE_INVALID_PARAM; }
  *count = 0U;

  for (i = 0U; (i < PX4LITE_REMOTE_NODE_MAX) && (written < capacity); i++) {
    taskENTER_CRITICAL();
    local = s_remote[i];
    node_id = s_slot_node[i];
    taskEXIT_CRITICAL();

    if ((local.header.valid == 0U) || (local.system_id != node_id)) { continue; }
    Px4Lite_RemoteTelemetryRefreshFreshness(&local, now_ms);

    memset(&out[written], 0, sizeof(out[written]));
    out[written].node_id = node_id; /* 对外用绑定的 node_id(sysid)，非内部槽索引 */
    out[written].system_id = local.system_id;
    out[written].component_id = local.component_id;
    out[written].heartbeat_type = local.heartbeat_type;
    out[written].heartbeat_system_status = local.heartbeat_system_status;
    out[written].last_heartbeat_ms = local.heartbeat_update_ms;
    out[written].last_data_ms = RemoteTelemetry_LastDataMs(&local, now_ms);
    out[written].rx_frame_count = local.rx_frame_count;
    out[written].rx_sequence_lost_count = local.rx_sequence_lost_count;
    out[written].rx_loss_rate_x10 = local.rx_loss_rate_x10;
    if (Px4Lite_ElapsedMs(now_ms, local.heartbeat_update_ms) > PX4LITE_REMOTE_HEARTBEAT_STALE_MS) {
      out[written].state = PX4LITE_REMOTE_NODE_STALE;
    } else if (((local.valid_mask & ~PX4LITE_REMOTE_VALID_HEARTBEAT) != 0U) && (Px4Lite_ElapsedMs(now_ms, out[written].last_data_ms) <= PX4LITE_REMOTE_DATA_STALE_MS)) {
      out[written].state = PX4LITE_REMOTE_NODE_ACTIVE;
    } else {
      out[written].state = PX4LITE_REMOTE_NODE_DISCOVERED;
    }
    written++;
  }

  (void)s_mode_changed_ms;
  *count = written;
  return (written != 0U) ? PX4LITE_OK : PX4LITE_NOT_READY;
}
