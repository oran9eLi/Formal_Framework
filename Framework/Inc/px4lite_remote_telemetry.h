/**
 * @file px4lite_remote_telemetry.h
 * @brief LoRa 远程显示快照、显示源模式和远端设备过滤接口。
 *
 * @details
 * 本文件属于 Framework 层，保存由 MAVLink RX 写入的远端只读快照。
 * Display 和 Business 不直接解析 LoRa/MAVLink 字节流，只通过 Business API 读取这里的
 * 远端数据。远程快照不得覆盖本机 Navigation、Environment、System、Alarm 或 Motor topic。
 */

#ifndef PX4LITE_REMOTE_TELEMETRY_H
#define PX4LITE_REMOTE_TELEMETRY_H

#include <stdint.h>
#include "px4lite_config.h"
#include "px4lite_types.h"

#ifndef PX4LITE_REMOTE_TELEMETRY_TIMEOUT_MS
#define PX4LITE_REMOTE_TELEMETRY_TIMEOUT_MS 3000U /**< 远端遥测失联超时，单位：ms。 */
#endif

#define PX4LITE_REMOTE_TARGET_SYSID_ANY 0U /**< 接收任意远端 MAVLink system id。 */

#define PX4LITE_REMOTE_VALID_IDENTITY    (1UL << 0) /**< 远端身份和心跳字段有效。 */
#define PX4LITE_REMOTE_VALID_NAVIGATION  (1UL << 1) /**< 远端定位/速度字段有效。 */
#define PX4LITE_REMOTE_VALID_ATTITUDE    (1UL << 2) /**< 远端姿态字段有效。 */
#define PX4LITE_REMOTE_VALID_POWER       (1UL << 3) /**< 远端电源字段有效。 */
#define PX4LITE_REMOTE_VALID_ENVIRONMENT (1UL << 4) /**< 远端气压/温度字段有效。 */
#define PX4LITE_REMOTE_VALID_STATUS      (1UL << 5) /**< 远端系统状态字段有效。 */
#define PX4LITE_REMOTE_VALID_TEXT        (1UL << 6) /**< 远端状态文本字段有效。 */

/**
 * @brief 显示数据源模式。
 */
typedef enum {
  PX4LITE_REMOTE_MODE_LOCAL = 0, /**< 本机模式：显示本机数据，只主动发送本机遥测。 */
  PX4LITE_REMOTE_MODE_REMOTE     /**< 远程模式：显示远端数据，停止本机周期遥测。 */
} Px4Lite_RemoteMode_t;

/**
 * @brief 远端显示快照。
 */
typedef struct {
  Px4Lite_TopicHeader_t header; /**< 快照头，发布时间为最近一次远端字段更新时间。 */
  uint32_t valid_mask;          /**< 远端字段有效位，使用 `PX4LITE_REMOTE_VALID_*`。 */
  uint8_t source_sysid;         /**< 当前写入快照的 MAVLink system id。 */
  uint8_t source_compid;        /**< 当前写入快照的 MAVLink component id。 */
  uint8_t target_sysid;         /**< 当前目标 MAVLink system id，0 表示任意目标。 */
  uint8_t mav_type;             /**< MAVLink HEARTBEAT 中的设备类型。 */
  uint8_t base_mode;            /**< MAVLink HEARTBEAT base_mode。 */
  uint8_t system_status;        /**< MAVLink HEARTBEAT system_status。 */
  uint16_t reserved0;           /**< 保留字段，保持结构体对齐。 */
  uint32_t last_heartbeat_ms;   /**< 最近收到目标心跳时间，单位：ms。 */
  uint32_t last_msg_ms;         /**< 最近收到目标有效遥测时间，单位：ms。 */
  uint32_t rx_count;            /**< 目标设备有效消息计数。 */
  uint32_t filtered_count;      /**< 因 system id 不匹配被过滤的消息计数。 */
  uint32_t unsupported_count;   /**< 不支持或未处理的 MAVLink 消息计数。 */
  int32_t latitude_e7;          /**< 纬度，单位：degree * 1e7。 */
  int32_t longitude_e7;         /**< 经度，单位：degree * 1e7。 */
  int32_t altitude_mm;          /**< 高度，单位：mm。 */
  int32_t velocity_north_cms;   /**< 北向速度，单位：cm/s。 */
  int32_t velocity_east_cms;    /**< 东向速度，单位：cm/s。 */
  int32_t velocity_down_cms;    /**< 地向速度，单位：cm/s。 */
  int32_t roll_deg100;          /**< 横滚角，单位：degree * 100。 */
  int32_t pitch_deg100;         /**< 俯仰角，单位：degree * 100。 */
  int32_t yaw_deg100;           /**< 航向角，单位：degree * 100。 */
  int32_t roll_rate_dps100;     /**< 横滚角速度，单位：(degree/s) * 100。 */
  int32_t pitch_rate_dps100;    /**< 俯仰角速度，单位：(degree/s) * 100。 */
  int32_t yaw_rate_dps100;      /**< 航向角速度，单位：(degree/s) * 100。 */
  uint16_t hdop_x100;           /**< 水平精度因子，单位：HDOP * 100。 */
  uint8_t satellites_used;      /**< 当前定位使用的卫星数量。 */
  uint8_t gnss_fix_type;        /**< GNSS 定位类型。 */
  float pressure_pa;            /**< 气压，单位：Pa。 */
  float temperature_c;          /**< 温度，单位：摄氏度。 */
  uint32_t voltage_mv;          /**< 电池或输入电压，单位：mV。 */
  int32_t current_ma;           /**< 电流，单位：mA；未知时可为 0。 */
  uint8_t battery_percent;      /**< 电量百分比，范围：0 到 100；未知时为 0。 */
  uint8_t reserved1[3];         /**< 保留字段，保持结构体对齐。 */
  char status_text[32];         /**< 远端状态文本，UTF-8/ASCII，以 0 结尾。 */
} Px4Lite_RemoteTelemetrySnapshot_t;

/**
 * @brief 初始化远端显示快照和模式状态。
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
 * @brief 切换本地/远程显示模式。
 *
 * @param[in] now_ms 当前系统时间，单位：ms。
 *
 * @return 切换后的模式。
 */
Px4Lite_RemoteMode_t Px4Lite_RemoteTelemetryToggleMode(uint32_t now_ms);

/**
 * @brief 输入模式切换按键原始状态并执行去抖边沿切换。
 *
 * @param[in] pressed 1 表示按键处于按下状态，0 表示释放。
 * @param[in] now_ms 当前系统时间，单位：ms。
 *
 * @return 当前显示源模式。
 */
Px4Lite_RemoteMode_t Px4Lite_RemoteTelemetryUpdateModeButton(uint8_t pressed, uint32_t now_ms);

/**
 * @brief 获取当前显示源模式。
 *
 * @return 当前模式。
 */
Px4Lite_RemoteMode_t Px4Lite_RemoteTelemetryGetMode(void);

/**
 * @brief 设置远端目标 MAVLink system id。
 *
 * @param[in] sysid 目标 system id，0 表示接收任意远端。
 *
 * @return 设置结果。
 */
Px4Lite_Result_t Px4Lite_RemoteTelemetrySetTargetSysId(uint8_t sysid);

/**
 * @brief 获取远端目标 MAVLink system id。
 *
 * @return 当前目标 system id，0 表示任意远端。
 */
uint8_t Px4Lite_RemoteTelemetryGetTargetSysId(void);

/**
 * @brief 判断输入 system id 是否允许写入远端快照。
 *
 * @param[in] sysid 待检查的 MAVLink system id。
 *
 * @return 1 表示允许，0 表示过滤。
 */
uint8_t Px4Lite_RemoteTelemetryAcceptSysId(uint8_t sysid);

/**
 * @brief 记录一次远端设备过滤。
 */
void Px4Lite_RemoteTelemetryRecordFiltered(void);

/**
 * @brief 记录一次未支持的远端消息。
 */
void Px4Lite_RemoteTelemetryRecordUnsupported(void);

/**
 * @brief 获取可写远端快照。
 *
 * @return 远端快照指针，仅 MAVLink RX 所在 comm 上下文写入。
 */
Px4Lite_RemoteTelemetrySnapshot_t *Px4Lite_RemoteTelemetryMutable(void);

/**
 * @brief 标记远端快照有新数据。
 *
 * @param[in] sysid 远端 MAVLink system id。
 * @param[in] compid 远端 MAVLink component id。
 * @param[in] valid_bits 本次更新的字段有效位。
 * @param[in] now_ms 当前系统时间，单位：ms。
 */
void Px4Lite_RemoteTelemetryCommit(uint8_t sysid, uint8_t compid, uint32_t valid_bits, uint32_t now_ms);

/**
 * @brief 复制远端显示快照。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 * @param[in] now_ms 当前系统时间，单位：ms。
 *
 * @return 复制结果。
 */
Px4Lite_Result_t Px4Lite_RemoteTelemetryCopySnapshot(Px4Lite_RemoteTelemetrySnapshot_t *out, uint32_t now_ms);

#endif
