/**
 * @file px4lite_log_message_ids.h
 * @brief Framework 结构化事件目录；Business、Display 与 RPi 共用。
 * @details 编号只追加；0~37 保留历史编号。目录只解释事实，不承担设备控制或状态判断。
 */
#ifndef PX4LITE_LOG_MESSAGE_IDS_H
#define PX4LITE_LOG_MESSAGE_IDS_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/** @brief 固定事件编号；新增项只能追加到 COUNT 之前。 */
typedef enum {
  PX4LITE_LOGMSG_SYSTEM_START = 0, /**< System started；严重度 0。 */
  PX4LITE_LOGMSG_SELFCHECK_OK = 1, /**< Checks passed: GNSS IMU Baro LoRa SD；严重度 0。 */
  PX4LITE_LOGMSG_SELFCHECK_PART = 2, /**< Checks degraded: GNSS IMU Baro LoRa SD；严重度 1。 */
  PX4LITE_LOGMSG_SELFCHECK_FAIL = 3, /**< Checks failed: GNSS IMU Baro LoRa SD；严重度 2。 */
  PX4LITE_LOGMSG_GPS_OK = 4, /**< GNSS fix acquired；严重度 0。 */
  PX4LITE_LOGMSG_GPS_NOSIG = 5, /**< GNSS no fix；严重度 1。 */
  PX4LITE_LOGMSG_GPS_LOST = 6, /**< GNSS RX timeout；严重度 1。 */
  PX4LITE_LOGMSG_ATT_OK = 7, /**< IMU ready；严重度 0。 */
  PX4LITE_LOGMSG_ATT_LOST = 8, /**< IMU offline；严重度 2。 */
  PX4LITE_LOGMSG_ENV_OK = 9, /**< Baro ready；严重度 0。 */
  PX4LITE_LOGMSG_ENV_LOST = 10, /**< Baro offline；严重度 1。 */
  PX4LITE_LOGMSG_COMM_OK = 11, /**< LoRa ready；严重度 0。 */
  PX4LITE_LOGMSG_COMM_LOST = 12, /**< LoRa offline；严重度 1。 */
  PX4LITE_LOGMSG_STORAGE_OK = 13, /**< SD ready；严重度 0。 */
  PX4LITE_LOGMSG_STORAGE_LOST = 14, /**< SD offline；严重度 1。 */
  PX4LITE_LOGMSG_MOTOR_OK = 15, /**< Motor supply normal；严重度 0。 */
  PX4LITE_LOGMSG_MOTOR_DISCONNECT = 16, /**< Motor supply out of range；严重度 1。 */
  PX4LITE_LOGMSG_MOTOR_LOWPOWER = 17, /**< Motor battery low；严重度 1。 */
  PX4LITE_LOGMSG_MOTOR1_FAIL = 18, /**< Motor 1 fault reported；严重度 2。 */
  PX4LITE_LOGMSG_MOTOR2_FAIL = 19, /**< Motor 2 fault reported；严重度 2。 */
  PX4LITE_LOGMSG_MOTOR3_FAIL = 20, /**< Motor 3 fault reported；严重度 2。 */
  PX4LITE_LOGMSG_MOTOR4_FAIL = 21, /**< Motor 4 fault reported；严重度 2。 */
  PX4LITE_LOGMSG_MOTOR_ALL_FAIL = 22, /**< Motor faults reported；严重度 2。 */
  PX4LITE_LOGMSG_MOTOR_DEAD = 23, /**< Motor battery critical；严重度 2。 */
  PX4LITE_LOGMSG_MOTOR_CHARGE = 24, /**< Motor battery charge needed；严重度 1。 */
  PX4LITE_LOGMSG_MAIN_CHARGE = 25, /**< Main battery low；严重度 1。 */
  PX4LITE_LOGMSG_ALARM_ACTIVE = 26, /**< Alarm active；严重度 1。 */
  PX4LITE_LOGMSG_ALARM_NONE = 27, /**< No active alarms；严重度 0。 */
  PX4LITE_LOGMSG_REMOTEID_OK = 28, /**< RID detected；严重度 0。 */
  PX4LITE_LOGMSG_REMOTEID_LOST = 29, /**< RID not detected；严重度 1。 */
  PX4LITE_LOGMSG_5G_OK = 30, /**< Network reported online；严重度 0。 */
  PX4LITE_LOGMSG_5G_LOST = 31, /**< Network reported offline；严重度 1。 */
  PX4LITE_LOGMSG_TAKEOFF_SENSOR_FAIL = 32, /**< Ramp-up blocked: sensors；严重度 1。 */
  PX4LITE_LOGMSG_TAKEOFF_POWER_FAIL = 33, /**< Ramp-up blocked: power；严重度 1。 */
  PX4LITE_LOGMSG_THROTTLE_POWER_FAIL = 34, /**< Throttle blocked: power；严重度 1。 */
  PX4LITE_LOGMSG_LANDING_FAIL = 35, /**< Ramp-down rejected；严重度 1。 */
  PX4LITE_LOGMSG_MOTOR_POWER_CUTOFF = 36, /**< Power protection: output withdrawn；严重度 2。 */
  PX4LITE_LOGMSG_LANDING_THROTTLE_REJECT = 37, /**< Throttle blocked: ramp-down；严重度 1。 */
  PX4LITE_LOGMSG_GNSS_UNINITIALIZED = 38, /**< GNSS uninitialized；严重度 1。 */
  PX4LITE_LOGMSG_GNSS_STARTING = 39, /**< GNSS starting；严重度 0。 */
  PX4LITE_LOGMSG_GNSS_ONLINE = 40, /**< GNSS ready；严重度 0。 */
  PX4LITE_LOGMSG_GNSS_DEGRADED = 41, /**< GNSS degraded；严重度 1。 */
  PX4LITE_LOGMSG_GNSS_OFFLINE = 42, /**< GNSS offline；严重度 1。 */
  PX4LITE_LOGMSG_GNSS_FAILED = 43, /**< GNSS failed；严重度 2。 */
  PX4LITE_LOGMSG_GNSS_DISABLED = 44, /**< GNSS disabled；严重度 0。 */
  PX4LITE_LOGMSG_GNSS_RESTORED = 45, /**< GNSS restored；严重度 0。 */
  PX4LITE_LOGMSG_GNSS_UNAVAILABLE = 46, /**< GNSS status unavailable；严重度 1。 */
  PX4LITE_LOGMSG_IMU_UNINITIALIZED = 47, /**< IMU uninitialized；严重度 1。 */
  PX4LITE_LOGMSG_IMU_STARTING = 48, /**< IMU starting；严重度 0。 */
  PX4LITE_LOGMSG_IMU_ONLINE = 49, /**< IMU ready；严重度 0。 */
  PX4LITE_LOGMSG_IMU_DEGRADED = 50, /**< IMU degraded；严重度 1。 */
  PX4LITE_LOGMSG_IMU_OFFLINE = 51, /**< IMU offline；严重度 2。 */
  PX4LITE_LOGMSG_IMU_FAILED = 52, /**< IMU failed；严重度 2。 */
  PX4LITE_LOGMSG_IMU_DISABLED = 53, /**< IMU disabled；严重度 0。 */
  PX4LITE_LOGMSG_IMU_RESTORED = 54, /**< IMU restored；严重度 0。 */
  PX4LITE_LOGMSG_IMU_UNAVAILABLE = 55, /**< IMU status unavailable；严重度 1。 */
  PX4LITE_LOGMSG_BARO_UNINITIALIZED = 56, /**< Baro uninitialized；严重度 1。 */
  PX4LITE_LOGMSG_BARO_STARTING = 57, /**< Baro starting；严重度 0。 */
  PX4LITE_LOGMSG_BARO_ONLINE = 58, /**< Baro ready；严重度 0。 */
  PX4LITE_LOGMSG_BARO_DEGRADED = 59, /**< Baro degraded；严重度 1。 */
  PX4LITE_LOGMSG_BARO_OFFLINE = 60, /**< Baro offline；严重度 1。 */
  PX4LITE_LOGMSG_BARO_FAILED = 61, /**< Baro failed；严重度 2。 */
  PX4LITE_LOGMSG_BARO_DISABLED = 62, /**< Baro disabled；严重度 0。 */
  PX4LITE_LOGMSG_BARO_RESTORED = 63, /**< Baro restored；严重度 0。 */
  PX4LITE_LOGMSG_BARO_UNAVAILABLE = 64, /**< Baro status unavailable；严重度 1。 */
  PX4LITE_LOGMSG_BATTERY_UNINITIALIZED = 65, /**< Power monitor uninitialized；严重度 1。 */
  PX4LITE_LOGMSG_BATTERY_STARTING = 66, /**< Power monitor starting；严重度 0。 */
  PX4LITE_LOGMSG_BATTERY_ONLINE = 67, /**< Power monitor ready；严重度 0。 */
  PX4LITE_LOGMSG_BATTERY_DEGRADED = 68, /**< Power monitor degraded；严重度 1。 */
  PX4LITE_LOGMSG_BATTERY_OFFLINE = 69, /**< Power monitor offline；严重度 1。 */
  PX4LITE_LOGMSG_BATTERY_FAILED = 70, /**< Power monitor failed；严重度 2。 */
  PX4LITE_LOGMSG_BATTERY_DISABLED = 71, /**< Power monitor disabled；严重度 0。 */
  PX4LITE_LOGMSG_BATTERY_RESTORED = 72, /**< Power monitor restored；严重度 0。 */
  PX4LITE_LOGMSG_BATTERY_UNAVAILABLE = 73, /**< Power monitor status unavailable；严重度 1。 */
  PX4LITE_LOGMSG_LORA_UNINITIALIZED = 74, /**< LoRa uninitialized；严重度 1。 */
  PX4LITE_LOGMSG_LORA_STARTING = 75, /**< LoRa starting；严重度 0。 */
  PX4LITE_LOGMSG_LORA_ONLINE = 76, /**< LoRa ready；严重度 0。 */
  PX4LITE_LOGMSG_LORA_DEGRADED = 77, /**< LoRa degraded；严重度 1。 */
  PX4LITE_LOGMSG_LORA_OFFLINE = 78, /**< LoRa offline；严重度 1。 */
  PX4LITE_LOGMSG_LORA_FAILED = 79, /**< LoRa failed；严重度 2。 */
  PX4LITE_LOGMSG_LORA_DISABLED = 80, /**< LoRa disabled；严重度 0。 */
  PX4LITE_LOGMSG_LORA_RESTORED = 81, /**< LoRa restored；严重度 0。 */
  PX4LITE_LOGMSG_LORA_UNAVAILABLE = 82, /**< LoRa status unavailable；严重度 1。 */
  PX4LITE_LOGMSG_NETWORK_UNINITIALIZED = 83, /**< Network report uninitialized；严重度 1。 */
  PX4LITE_LOGMSG_NETWORK_STARTING = 84, /**< Network report starting；严重度 0。 */
  PX4LITE_LOGMSG_NETWORK_ONLINE = 85, /**< Network report online；严重度 0。 */
  PX4LITE_LOGMSG_NETWORK_DEGRADED = 86, /**< Network report degraded；严重度 1。 */
  PX4LITE_LOGMSG_NETWORK_OFFLINE = 87, /**< Network report offline；严重度 1。 */
  PX4LITE_LOGMSG_NETWORK_FAILED = 88, /**< Network report failed；严重度 2。 */
  PX4LITE_LOGMSG_NETWORK_DISABLED = 89, /**< Network report disabled；严重度 0。 */
  PX4LITE_LOGMSG_NETWORK_RESTORED = 90, /**< Network report online again；严重度 0。 */
  PX4LITE_LOGMSG_NETWORK_UNAVAILABLE = 91, /**< Network report status unavailable；严重度 1。 */
  PX4LITE_LOGMSG_STORAGE_UNINITIALIZED = 92, /**< SD uninitialized；严重度 1。 */
  PX4LITE_LOGMSG_STORAGE_STARTING = 93, /**< SD starting；严重度 0。 */
  PX4LITE_LOGMSG_STORAGE_ONLINE = 94, /**< SD ready；严重度 0。 */
  PX4LITE_LOGMSG_STORAGE_DEGRADED = 95, /**< SD degraded；严重度 1。 */
  PX4LITE_LOGMSG_STORAGE_OFFLINE = 96, /**< SD offline；严重度 1。 */
  PX4LITE_LOGMSG_STORAGE_FAILED = 97, /**< SD failed；严重度 2。 */
  PX4LITE_LOGMSG_STORAGE_DISABLED = 98, /**< SD disabled；严重度 0。 */
  PX4LITE_LOGMSG_STORAGE_RESTORED = 99, /**< SD restored；严重度 0。 */
  PX4LITE_LOGMSG_STORAGE_UNAVAILABLE = 100, /**< SD status unavailable；严重度 1。 */
  PX4LITE_LOGMSG_RID_UNINITIALIZED = 101, /**< RID uninitialized；严重度 1。 */
  PX4LITE_LOGMSG_RID_STARTING = 102, /**< RID starting；严重度 0。 */
  PX4LITE_LOGMSG_RID_ONLINE = 103, /**< RID detected；严重度 0。 */
  PX4LITE_LOGMSG_RID_DEGRADED = 104, /**< RID degraded；严重度 1。 */
  PX4LITE_LOGMSG_RID_OFFLINE = 105, /**< RID offline；严重度 1。 */
  PX4LITE_LOGMSG_RID_FAILED = 106, /**< RID failed；严重度 2。 */
  PX4LITE_LOGMSG_RID_DISABLED = 107, /**< RID disabled；严重度 0。 */
  PX4LITE_LOGMSG_RID_RESTORED = 108, /**< RID detected again；严重度 0。 */
  PX4LITE_LOGMSG_RID_UNAVAILABLE = 109, /**< RID status unavailable；严重度 1。 */
  PX4LITE_LOGMSG_CONTROL_UNINITIALIZED = 110, /**< Control uninitialized；严重度 1。 */
  PX4LITE_LOGMSG_CONTROL_STARTING = 111, /**< Control starting；严重度 0。 */
  PX4LITE_LOGMSG_CONTROL_ONLINE = 112, /**< Control ready；严重度 0。 */
  PX4LITE_LOGMSG_CONTROL_DEGRADED = 113, /**< Control degraded；严重度 1。 */
  PX4LITE_LOGMSG_CONTROL_OFFLINE = 114, /**< Control offline；严重度 2。 */
  PX4LITE_LOGMSG_CONTROL_FAILED = 115, /**< Control failed；严重度 2。 */
  PX4LITE_LOGMSG_CONTROL_DISABLED = 116, /**< Control disabled；严重度 0。 */
  PX4LITE_LOGMSG_CONTROL_RESTORED = 117, /**< Control restored；严重度 0。 */
  PX4LITE_LOGMSG_CONTROL_UNAVAILABLE = 118, /**< Control status unavailable；严重度 1。 */
  PX4LITE_LOGMSG_CHECKING = 119, /**< Checking: GNSS IMU Baro LoRa SD；严重度 0。 */
  PX4LITE_LOGMSG_CHECKS_UNAVAILABLE = 120, /**< Checks unavailable；严重度 1。 */
  PX4LITE_LOGMSG_GNSS_FIX_LOST = 121, /**< GNSS fix lost；严重度 1。 */
  PX4LITE_LOGMSG_NETWORK_STALE = 122, /**< Network report stale；严重度 1。 */
  PX4LITE_LOGMSG_RID_TX_ERROR = 123, /**< RID UART TX error；严重度 2。 */
  PX4LITE_LOGMSG_RID_TX_RESTORED = 124, /**< RID TX submission resumed；严重度 0。 */
  PX4LITE_LOGMSG_MAIN_NORMAL = 125, /**< Main supply normal；严重度 0。 */
  PX4LITE_LOGMSG_MAIN_RESTORED = 126, /**< Main supply restored；严重度 0。 */
  PX4LITE_LOGMSG_MAIN_ABSENT = 127, /**< Main supply out of range；严重度 1。 */
  PX4LITE_LOGMSG_MAIN_UNAVAILABLE = 128, /**< Main voltage unavailable；严重度 1。 */
  PX4LITE_LOGMSG_MOTOR_RESTORED = 129, /**< Motor supply restored；严重度 0。 */
  PX4LITE_LOGMSG_MOTOR_UNAVAILABLE = 130, /**< Motor voltage unavailable；严重度 1。 */
  PX4LITE_LOGMSG_ALARM_UNAVAILABLE = 131, /**< Alarm status unavailable；严重度 1。 */
  PX4LITE_LOGMSG_ALARM_RESTORED = 132, /**< Alarm status available；严重度 0。 */
  PX4LITE_LOGMSG_POWER_INHIBITED = 133, /**< Power inhibited；严重度 1。 */
  PX4LITE_LOGMSG_POWER_RELEASED = 134, /**< Power inhibit cleared；严重度 0。 */
  PX4LITE_LOGMSG_OUTPUT_MINIMUM = 135, /**< Output at minimum；严重度 0。 */
  PX4LITE_LOGMSG_OUTPUT_WRITE_FAILED = 136, /**< Output write failed；严重度 2。 */
  PX4LITE_LOGMSG_RAMP_UP_ACCEPTED = 137, /**< Ramp-up accepted；严重度 0。 */
  PX4LITE_LOGMSG_RAMP_UP_STARTED = 138, /**< Ramp-up started；严重度 0。 */
  PX4LITE_LOGMSG_RAMP_UP_COMPLETED = 139, /**< Ramp-up completed；严重度 0。 */
  PX4LITE_LOGMSG_RAMP_DOWN_ACCEPTED = 140, /**< Ramp-down accepted；严重度 0。 */
  PX4LITE_LOGMSG_RAMP_DOWN_STARTED = 141, /**< Ramp-down started；严重度 0。 */
  PX4LITE_LOGMSG_RAMP_DOWN_COMPLETED = 142, /**< Ramp-down completed；严重度 0。 */
  PX4LITE_LOGMSG_RAMP_UP_BUSY = 143, /**< Ramp-up blocked: ramp active；严重度 1。 */
  PX4LITE_LOGMSG_RAMP_DOWN_BUSY = 144, /**< Ramp-down blocked: already active；严重度 1。 */
  PX4LITE_LOGMSG_RAMP_UP_ABORTED = 145, /**< Ramp-up aborted；严重度 1。 */
  PX4LITE_LOGMSG_RAMP_DOWN_ABORTED = 146, /**< Ramp-down aborted；严重度 1。 */
  PX4LITE_LOGMSG_RAMP_SENSOR_ABORT = 147, /**< Ramp-up aborted: attitude lost；严重度 2。 */
  PX4LITE_LOGMSG_PROTECT_DOWN_STARTED = 148, /**< Protection ramp-down started；严重度 1。 */
  PX4LITE_LOGMSG_ESTOP = 149, /**< Local E-stop triggered；严重度 1。 */
  PX4LITE_LOGMSG_ESTOP_RELEASED = 150, /**< E-stop released; rearming；严重度 0。 */
  PX4LITE_LOGMSG_CONTROL_TIMEOUT = 151, /**< Control cycle timeout；严重度 2。 */
  PX4LITE_LOGMSG_THROTTLE_ACCEPTED = 152, /**< Throttle target accepted；严重度 0。 */
  PX4LITE_LOGMSG_THROTTLE_INVALID = 153, /**< Throttle rejected: invalid value；严重度 1。 */
  PX4LITE_LOGMSG_BIAS_ACCEPTED = 154, /**< Bias calibration accepted；严重度 0。 */
  PX4LITE_LOGMSG_BIAS_STARTED = 155, /**< Bias calibration started；严重度 0。 */
  PX4LITE_LOGMSG_BIAS_RESTARTED = 156, /**< Bias sampling reset: motion；严重度 1。 */
  PX4LITE_LOGMSG_BIAS_COMPLETED = 157, /**< Bias calibration completed；严重度 0。 */
  PX4LITE_LOGMSG_BIAS_BUSY = 158, /**< Bias rejected: calibration busy；严重度 1。 */
  PX4LITE_LOGMSG_BIAS_NO_DATA = 159, /**< Bias rejected: no fresh IMU；严重度 1。 */
  PX4LITE_LOGMSG_BIAS_ABORT_NO_DATA = 160, /**< Bias aborted: IMU stale；严重度 2。 */
  PX4LITE_LOGMSG_BIAS_FAILED_STILL = 161, /**< Bias failed: not still；严重度 1。 */
  PX4LITE_LOGMSG_ZERO_ACCEPTED = 162, /**< Zero reference accepted；严重度 0。 */
  PX4LITE_LOGMSG_ZERO_STARTED = 163, /**< Zero reference started；严重度 0。 */
  PX4LITE_LOGMSG_ZERO_COMPLETED = 164, /**< Zero reference completed；严重度 0。 */
  PX4LITE_LOGMSG_ZERO_BUSY = 165, /**< Zero rejected: calibration busy；严重度 1。 */
  PX4LITE_LOGMSG_ZERO_NO_DATA = 166, /**< Zero rejected: no fresh IMU；严重度 1。 */
  PX4LITE_LOGMSG_RESTORE_ACCEPTED = 167, /**< Reference restore accepted；严重度 0。 */
  PX4LITE_LOGMSG_RESTORE_STARTED = 168, /**< Reference restore started；严重度 0。 */
  PX4LITE_LOGMSG_RESTORE_COMPLETED = 169, /**< Reference restored；严重度 0。 */
  PX4LITE_LOGMSG_RESTORE_BUSY = 170, /**< Restore rejected: calibration busy；严重度 1。 */
  PX4LITE_LOGMSG_RESTORE_NO_DATA = 171, /**< Restore rejected: no fresh IMU；严重度 1。 */
  PX4LITE_LOGMSG_ALARM_RAISED = 172, /**< 单源告警触发；严重度取告警事实，active=1。 */
  PX4LITE_LOGMSG_ALARM_CLEARED = 173, /**< 单源故障清除；严重度 0，active=0。 */
  PX4LITE_LOGMSG_ALARM_UPDATED = 174, /**< 单源告警严重度变化；严重度取新事实，active=1。 */
  PX4LITE_LOGMSG_COUNT
} Px4Lite_LogMessageId_t;
/** @brief 查询短英文事实；非法编号返回 "Unknown event"。 */
const char *Px4Lite_LogMessageText(uint16_t message_id);
/** @brief 查询默认严重度：0 INFO、1 WARN、2 ERROR；非法编号按 WARN。 */
uint8_t Px4Lite_LogMessageSeverity(uint16_t message_id);
#ifdef __cplusplus
}
#endif
#endif
