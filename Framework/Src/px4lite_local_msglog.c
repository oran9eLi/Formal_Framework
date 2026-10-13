/**
 * @file px4lite_local_msglog.c
 * @brief 本机结构化消息日志环形缓冲实现。
 *
 * @details
 * Business、Control、Estimator 任务写入，Display 与 Comm 任务读取，因此元数据和条目复制
 * 都必须在临界区内完成，避免多字节字段或条目内容被读到半更新状态。
 */
#include "px4lite_local_msglog.h"
#include "px4lite_log_message_ids.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/** @brief 只读事件目录；文本和严重度与编号逐项对应。 */
typedef struct {
  const char *text; /**< 静态短英文，不依赖来源字段解释本机事件。 */
  uint8_t severity; /**< 0 INFO、1 WARN、2 ERROR。 */
} LocalMsgLog_CatalogEntry_t;
static const LocalMsgLog_CatalogEntry_t s_catalog[PX4LITE_LOGMSG_COUNT] = {
  {"System started", 0U}, /* SYSTEM_START */
  {"Checks passed: GNSS IMU Baro LoRa SD", 0U}, /* SELFCHECK_OK */
  {"Checks degraded: GNSS IMU Baro LoRa SD", 1U}, /* SELFCHECK_PART */
  {"Checks failed: GNSS IMU Baro LoRa SD", 2U}, /* SELFCHECK_FAIL */
  {"GNSS fix acquired", 0U}, /* GPS_OK */
  {"GNSS no fix", 1U}, /* GPS_NOSIG */
  {"GNSS RX timeout", 1U}, /* GPS_LOST */
  {"IMU ready", 0U}, /* ATT_OK */
  {"IMU offline", 2U}, /* ATT_LOST */
  {"Baro ready", 0U}, /* ENV_OK */
  {"Baro offline", 1U}, /* ENV_LOST */
  {"LoRa ready", 0U}, /* COMM_OK */
  {"LoRa offline", 1U}, /* COMM_LOST */
  {"SD ready", 0U}, /* STORAGE_OK */
  {"SD offline", 1U}, /* STORAGE_LOST */
  {"Motor supply normal", 0U}, /* MOTOR_OK */
  {"Motor supply out of range", 1U}, /* MOTOR_DISCONNECT */
  {"Motor battery low", 1U}, /* MOTOR_LOWPOWER */
  {"Motor 1 fault reported", 2U}, /* MOTOR1_FAIL */
  {"Motor 2 fault reported", 2U}, /* MOTOR2_FAIL */
  {"Motor 3 fault reported", 2U}, /* MOTOR3_FAIL */
  {"Motor 4 fault reported", 2U}, /* MOTOR4_FAIL */
  {"Motor faults reported", 2U}, /* MOTOR_ALL_FAIL */
  {"Motor battery critical", 2U}, /* MOTOR_DEAD */
  {"Motor battery charge needed", 1U}, /* MOTOR_CHARGE */
  {"Main battery low", 1U}, /* MAIN_CHARGE */
  {"Alarm active", 1U}, /* ALARM_ACTIVE */
  {"No active alarms", 0U}, /* ALARM_NONE */
  {"RID detected", 0U}, /* REMOTEID_OK */
  {"RID not detected", 1U}, /* REMOTEID_LOST */
  {"Network reported online", 0U}, /* 5G_OK */
  {"Network reported offline", 1U}, /* 5G_LOST */
  {"Ramp-up blocked: sensors", 1U}, /* TAKEOFF_SENSOR_FAIL */
  {"Ramp-up blocked: power", 1U}, /* TAKEOFF_POWER_FAIL */
  {"Throttle blocked: power", 1U}, /* THROTTLE_POWER_FAIL */
  {"Ramp-down rejected", 1U}, /* LANDING_FAIL */
  {"Power protection: output withdrawn", 2U}, /* MOTOR_POWER_CUTOFF */
  {"Throttle blocked: ramp-down", 1U}, /* LANDING_THROTTLE_REJECT */
  {"GNSS uninitialized", 1U}, /* GNSS_UNINITIALIZED */
  {"GNSS starting", 0U}, /* GNSS_STARTING */
  {"GNSS ready", 0U}, /* GNSS_ONLINE */
  {"GNSS degraded", 1U}, /* GNSS_DEGRADED */
  {"GNSS offline", 1U}, /* GNSS_OFFLINE */
  {"GNSS failed", 2U}, /* GNSS_FAILED */
  {"GNSS disabled", 0U}, /* GNSS_DISABLED */
  {"GNSS restored", 0U}, /* GNSS_RESTORED */
  {"GNSS status unavailable", 1U}, /* GNSS_UNAVAILABLE */
  {"IMU uninitialized", 1U}, /* IMU_UNINITIALIZED */
  {"IMU starting", 0U}, /* IMU_STARTING */
  {"IMU ready", 0U}, /* IMU_ONLINE */
  {"IMU degraded", 1U}, /* IMU_DEGRADED */
  {"IMU offline", 2U}, /* IMU_OFFLINE */
  {"IMU failed", 2U}, /* IMU_FAILED */
  {"IMU disabled", 0U}, /* IMU_DISABLED */
  {"IMU restored", 0U}, /* IMU_RESTORED */
  {"IMU status unavailable", 1U}, /* IMU_UNAVAILABLE */
  {"Baro uninitialized", 1U}, /* BARO_UNINITIALIZED */
  {"Baro starting", 0U}, /* BARO_STARTING */
  {"Baro ready", 0U}, /* BARO_ONLINE */
  {"Baro degraded", 1U}, /* BARO_DEGRADED */
  {"Baro offline", 1U}, /* BARO_OFFLINE */
  {"Baro failed", 2U}, /* BARO_FAILED */
  {"Baro disabled", 0U}, /* BARO_DISABLED */
  {"Baro restored", 0U}, /* BARO_RESTORED */
  {"Baro status unavailable", 1U}, /* BARO_UNAVAILABLE */
  {"Power monitor uninitialized", 1U}, /* BATTERY_UNINITIALIZED */
  {"Power monitor starting", 0U}, /* BATTERY_STARTING */
  {"Power monitor ready", 0U}, /* BATTERY_ONLINE */
  {"Power monitor degraded", 1U}, /* BATTERY_DEGRADED */
  {"Power monitor offline", 1U}, /* BATTERY_OFFLINE */
  {"Power monitor failed", 2U}, /* BATTERY_FAILED */
  {"Power monitor disabled", 0U}, /* BATTERY_DISABLED */
  {"Power monitor restored", 0U}, /* BATTERY_RESTORED */
  {"Power monitor status unavailable", 1U}, /* BATTERY_UNAVAILABLE */
  {"LoRa uninitialized", 1U}, /* LORA_UNINITIALIZED */
  {"LoRa starting", 0U}, /* LORA_STARTING */
  {"LoRa ready", 0U}, /* LORA_ONLINE */
  {"LoRa degraded", 1U}, /* LORA_DEGRADED */
  {"LoRa offline", 1U}, /* LORA_OFFLINE */
  {"LoRa failed", 2U}, /* LORA_FAILED */
  {"LoRa disabled", 0U}, /* LORA_DISABLED */
  {"LoRa restored", 0U}, /* LORA_RESTORED */
  {"LoRa status unavailable", 1U}, /* LORA_UNAVAILABLE */
  {"Network report uninitialized", 1U}, /* NETWORK_UNINITIALIZED */
  {"Network report starting", 0U}, /* NETWORK_STARTING */
  {"Network report online", 0U}, /* NETWORK_ONLINE */
  {"Network report degraded", 1U}, /* NETWORK_DEGRADED */
  {"Network report offline", 1U}, /* NETWORK_OFFLINE */
  {"Network report failed", 2U}, /* NETWORK_FAILED */
  {"Network report disabled", 0U}, /* NETWORK_DISABLED */
  {"Network report online again", 0U}, /* NETWORK_RESTORED */
  {"Network report status unavailable", 1U}, /* NETWORK_UNAVAILABLE */
  {"SD uninitialized", 1U}, /* STORAGE_UNINITIALIZED */
  {"SD starting", 0U}, /* STORAGE_STARTING */
  {"SD ready", 0U}, /* STORAGE_ONLINE */
  {"SD degraded", 1U}, /* STORAGE_DEGRADED */
  {"SD offline", 1U}, /* STORAGE_OFFLINE */
  {"SD failed", 2U}, /* STORAGE_FAILED */
  {"SD disabled", 0U}, /* STORAGE_DISABLED */
  {"SD restored", 0U}, /* STORAGE_RESTORED */
  {"SD status unavailable", 1U}, /* STORAGE_UNAVAILABLE */
  {"RID uninitialized", 1U}, /* RID_UNINITIALIZED */
  {"RID starting", 0U}, /* RID_STARTING */
  {"RID detected", 0U}, /* RID_ONLINE */
  {"RID degraded", 1U}, /* RID_DEGRADED */
  {"RID offline", 1U}, /* RID_OFFLINE */
  {"RID failed", 2U}, /* RID_FAILED */
  {"RID disabled", 0U}, /* RID_DISABLED */
  {"RID detected again", 0U}, /* RID_RESTORED */
  {"RID status unavailable", 1U}, /* RID_UNAVAILABLE */
  {"Control uninitialized", 1U}, /* CONTROL_UNINITIALIZED */
  {"Control starting", 0U}, /* CONTROL_STARTING */
  {"Control ready", 0U}, /* CONTROL_ONLINE */
  {"Control degraded", 1U}, /* CONTROL_DEGRADED */
  {"Control offline", 2U}, /* CONTROL_OFFLINE */
  {"Control failed", 2U}, /* CONTROL_FAILED */
  {"Control disabled", 0U}, /* CONTROL_DISABLED */
  {"Control restored", 0U}, /* CONTROL_RESTORED */
  {"Control status unavailable", 1U}, /* CONTROL_UNAVAILABLE */
  {"Checking: GNSS IMU Baro LoRa SD", 0U}, /* CHECKING */
  {"Checks unavailable", 1U}, /* CHECKS_UNAVAILABLE */
  {"GNSS fix lost", 1U}, /* GNSS_FIX_LOST */
  {"Network report stale", 1U}, /* NETWORK_STALE */
  {"RID UART TX error", 2U}, /* RID_TX_ERROR */
  {"RID TX submission resumed", 0U}, /* RID_TX_RESTORED */
  {"Main supply normal", 0U}, /* MAIN_NORMAL */
  {"Main supply restored", 0U}, /* MAIN_RESTORED */
  {"Main supply out of range", 1U}, /* MAIN_ABSENT */
  {"Main voltage unavailable", 1U}, /* MAIN_UNAVAILABLE */
  {"Motor supply restored", 0U}, /* MOTOR_RESTORED */
  {"Motor voltage unavailable", 1U}, /* MOTOR_UNAVAILABLE */
  {"Alarm status unavailable", 1U}, /* ALARM_UNAVAILABLE */
  {"Alarm status available", 0U}, /* ALARM_RESTORED */
  {"Power inhibited", 1U}, /* POWER_INHIBITED */
  {"Power inhibit cleared", 0U}, /* POWER_RELEASED */
  {"Output at minimum", 0U}, /* OUTPUT_MINIMUM */
  {"Output write failed", 2U}, /* OUTPUT_WRITE_FAILED */
  {"Ramp-up accepted", 0U}, /* RAMP_UP_ACCEPTED */
  {"Ramp-up started", 0U}, /* RAMP_UP_STARTED */
  {"Ramp-up completed", 0U}, /* RAMP_UP_COMPLETED */
  {"Ramp-down accepted", 0U}, /* RAMP_DOWN_ACCEPTED */
  {"Ramp-down started", 0U}, /* RAMP_DOWN_STARTED */
  {"Ramp-down completed", 0U}, /* RAMP_DOWN_COMPLETED */
  {"Ramp-up blocked: ramp active", 1U}, /* RAMP_UP_BUSY */
  {"Ramp-down blocked: already active", 1U}, /* RAMP_DOWN_BUSY */
  {"Ramp-up aborted", 1U}, /* RAMP_UP_ABORTED */
  {"Ramp-down aborted", 1U}, /* RAMP_DOWN_ABORTED */
  {"Ramp-up aborted: attitude lost", 2U}, /* RAMP_SENSOR_ABORT */
  {"Protection ramp-down started", 1U}, /* PROTECT_DOWN_STARTED */
  {"Local E-stop triggered", 1U}, /* ESTOP */
  {"E-stop released; rearming", 0U}, /* ESTOP_RELEASED */
  {"Control cycle timeout", 2U}, /* CONTROL_TIMEOUT */
  {"Throttle target accepted", 0U}, /* THROTTLE_ACCEPTED */
  {"Throttle rejected: invalid value", 1U}, /* THROTTLE_INVALID */
  {"Bias calibration accepted", 0U}, /* BIAS_ACCEPTED */
  {"Bias calibration started", 0U}, /* BIAS_STARTED */
  {"Bias sampling reset: motion", 1U}, /* BIAS_RESTARTED */
  {"Bias calibration completed", 0U}, /* BIAS_COMPLETED */
  {"Bias rejected: calibration busy", 1U}, /* BIAS_BUSY */
  {"Bias rejected: no fresh IMU", 1U}, /* BIAS_NO_DATA */
  {"Bias aborted: IMU stale", 2U}, /* BIAS_ABORT_NO_DATA */
  {"Bias failed: not still", 1U}, /* BIAS_FAILED_STILL */
  {"Zero reference accepted", 0U}, /* ZERO_ACCEPTED */
  {"Zero reference started", 0U}, /* ZERO_STARTED */
  {"Zero reference completed", 0U}, /* ZERO_COMPLETED */
  {"Zero rejected: calibration busy", 1U}, /* ZERO_BUSY */
  {"Zero rejected: no fresh IMU", 1U}, /* ZERO_NO_DATA */
  {"Reference restore accepted", 0U}, /* RESTORE_ACCEPTED */
  {"Reference restore started", 0U}, /* RESTORE_STARTED */
  {"Reference restored", 0U}, /* RESTORE_COMPLETED */
  {"Restore rejected: calibration busy", 1U}, /* RESTORE_BUSY */
  {"Restore rejected: no fresh IMU", 1U}, /* RESTORE_NO_DATA */
  {"Alarm raised", 1U}, /* ALARM_RAISED */
  {"Alarm cleared", 0U}, /* ALARM_CLEARED */
  {"Alarm severity changed", 1U}, /* ALARM_UPDATED */
  {"LoRa experiment started", 0U}, /* LAB_START */
  {"LoRa experiment stopped", 0U}, /* LAB_STOP */
  {"LoRa send requested", 0U}, /* LAB_SEND */
  {"Node ID saved", 0U}, /* NODE_SAVED */
  {"Node ID save failed", 1U}, /* NODE_SAVE_FAILED */
  {"Channel verified", 0U}, /* CHANNEL_APPLIED */
  {"Channel apply failed", 1U}, /* CHANNEL_FAILED */
  {"Node ID conflict", 1U}, /* ID_CONFLICT */
  {"LoRa send: no fresh data", 1U}, /* LAB_NO_DATA */
  {"LoRa experiment interrupted", 1U}, /* LAB_ABORTED */
  {"Remote node discovered", 0U}, /* REMOTE_DISCOVERED */
  {"Remote heartbeat stale", 1U}, /* REMOTE_STALE */
  {"Remote node seen again", 0U}, /* REMOTE_RESTORED */
  {"Observation started",0U}, {"Observation stopped",0U}, {"Observation reset",0U},
  {"CSV recording started",0U}, {"CSV recording stopping",0U}, {"CSV recording saved",0U},
  {"CSV recording partial",1U}, {"CSV recording failed",2U},
};
/** @brief 返回只读事件短文案；未知消息保持显式未知。 */
const char *Px4Lite_LogMessageText(uint16_t message_id)
{
  return (message_id < PX4LITE_LOGMSG_COUNT) ? s_catalog[message_id].text : "Unknown event";
}
/** @brief 返回默认事件严重度；调用方可保留实际故障严重度。 */
uint8_t Px4Lite_LogMessageSeverity(uint16_t message_id)
{
  return (message_id < PX4LITE_LOGMSG_COUNT) ? s_catalog[message_id].severity : 1U;
}
/** @brief 将事实发生的毫秒 Uptime 转成兼容协议的 HHMMSS；100 小时循环。 */
void Px4Lite_LocalMsgLogEvent(uint16_t message_id, uint32_t now_ms, uint16_t fault_code, uint8_t source_id, uint8_t active)
{
  uint32_t sec = now_ms / 1000U;
  uint32_t time = (((sec / 3600U) % 100U) * 10000U) + (((sec / 60U) % 60U) * 100U) + (sec % 60U);
  Px4Lite_LocalMsgLogPush(message_id, time, fault_code, Px4Lite_LogMessageSeverity(message_id), source_id, active);
}

static Px4Lite_LogEntry_t s_ring[PX4LITE_LOCAL_LOG_CAP];
static uint16_t s_count;
static uint16_t s_head;     /**< 下一个写入位置。 */
static uint16_t s_next_seq = 1U; /**< 下一条序号，从 1 开始，跳过 0。 */
static uint16_t s_last_seq; /**< 最近一条已写序号，0 表示无日志。 */
static uint32_t s_version;

/** @brief 显式复位仅允许系统启动、调度器启动前调用；任务恢复禁止调用。 */
void Px4Lite_LocalMsgLogReset(void)
{
  taskENTER_CRITICAL();
  memset(s_ring, 0, sizeof(s_ring));
  s_count    = 0U;
  s_head     = 0U;
  s_next_seq = 1U;
  s_last_seq = 0U;
  s_version  = 0U;
  taskEXIT_CRITICAL();
}

/** @brief 校验编号与严重度后追加事实；结构准备在临界区外，临界区仅复制和推进固定环。 */
void Px4Lite_LocalMsgLogPush(uint16_t message_id, uint32_t time_hhmmss,
                             uint16_t fault_code, uint8_t severity, uint8_t source_id, uint8_t active)
{
  Px4Lite_LogEntry_t value;
  Px4Lite_LogEntry_t *entry;

  if ((message_id >= PX4LITE_LOGMSG_COUNT) || (severity > 2U)) { return; }
  memset(&value, 0, sizeof(value));
  value.message_id = message_id;
  value.time_hhmmss = time_hhmmss;
  value.fault_code = fault_code;
  value.severity = severity;
  value.source_id = source_id;
  value.active = (active != 0U) ? 1U : 0U;
  taskENTER_CRITICAL();
  entry = &s_ring[s_head];
  *entry = value;
  entry->sequence = s_next_seq;

  s_last_seq = s_next_seq;
  s_next_seq++;
  if (s_next_seq == 0U) { s_next_seq = 1U; }

  s_head = (uint16_t)((s_head + 1U) % PX4LITE_LOCAL_LOG_CAP);
  if (s_count < PX4LITE_LOCAL_LOG_CAP) { s_count++; }
  s_version++;
  taskEXIT_CRITICAL();
}

/** @brief 原子复制最新 cap 条及版本；容量覆盖时自动放弃最旧条目。 */
uint16_t Px4Lite_LocalMsgLogCopy(Px4Lite_LogEntry_t *out, uint16_t cap,
                                 uint32_t *out_version, uint16_t *out_last_seq)
{
  uint16_t i;
  uint16_t n;
  uint16_t start;

  taskENTER_CRITICAL();
  n = (s_count < cap) ? s_count : cap;
  if (out_version != 0) { *out_version = s_version; }
  if (out_last_seq != 0) { *out_last_seq = s_last_seq; }
  if ((out == 0) || (n == 0U)) {
    taskEXIT_CRITICAL();
    return 0U;
  }

  start = (uint16_t)((s_head + PX4LITE_LOCAL_LOG_CAP - s_count) % PX4LITE_LOCAL_LOG_CAP);
  if (s_count > cap) { start = (uint16_t)((start + (s_count - cap)) % PX4LITE_LOCAL_LOG_CAP); }
  for (i = 0U; i < n; ++i) {
    out[i] = s_ring[(uint16_t)((start + i) % PX4LITE_LOCAL_LOG_CAP)];
  }
  taskEXIT_CRITICAL();
  return n;
}

/** @brief 增量读取：游标在环中时取其后，否则从最旧保留条目恢复；序号回绕不比较大小。 */
uint16_t Px4Lite_LocalMsgLogDrainSince(uint16_t after_seq, Px4Lite_LogEntry_t *out, uint16_t max,
                                       uint16_t *out_latest_seq)
{
  uint16_t i, start, begin = 0U, written = 0U;
  taskENTER_CRITICAL();
  if (out_latest_seq != 0) { *out_latest_seq = s_last_seq; }
  if ((out == 0) || (max == 0U) || (s_count == 0U)) { taskEXIT_CRITICAL(); return 0U; }
  start = (uint16_t)((s_head + PX4LITE_LOCAL_LOG_CAP - s_count) % PX4LITE_LOCAL_LOG_CAP);
  if (after_seq != 0U) {
    for (i = 0U; i < s_count; ++i) {
      if (s_ring[(start + i) % PX4LITE_LOCAL_LOG_CAP].sequence == after_seq) { begin = (uint16_t)(i + 1U); break; }
    }
  }
  for (i = begin; (i < s_count) && (written < max); ++i) {
    out[written++] = s_ring[(start + i) % PX4LITE_LOCAL_LOG_CAP];
  }
  taskEXIT_CRITICAL();
  return written;
}
