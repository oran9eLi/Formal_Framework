/**
 * @file app_data_api.h
 * @brief Business 层消费者的唯一只读数据接口。
 *
 * @details
 * 本文件定义 Display、日志、存储、通信和业务逻辑读取系统快照的正式入口。
 * Business 层不得直接读取 Framework topic、BSP DMA buffer 或驱动私有变量。
 * 所有 `_ms` 参数和字段均为真实毫秒值，调用方应使用 `PlatformGetMs()` 或
 * `BSP_Time_GetTickMs()` 提供当前时间。
 */

#ifndef APP_DATA_API_H
#define APP_DATA_API_H

#include <stdint.h>
#include "px4lite_time.h"
#include "px4lite_types.h"
#include "px4lite_remote_telemetry.h"

#define APP_NAVIGATION_MAX_AGE_MS  1500U /**< Navigation 快照最大可接受年龄，单位：ms。 */
#define APP_SYSTEM_MAX_AGE_MS      500U  /**< System/Health 快照最大可接受年龄，单位：ms。 */
#define APP_ENVIRONMENT_MAX_AGE_MS 2500U /**< Environment 快照最大可接受年龄，单位：ms。 */
#define APP_ALARM_MAX_AGE_MS       500U  /**< Alarm 快照最大可接受年龄，单位：ms。 */
#define APP_MOTOR_MAX_AGE_MS       500U  /**< Motor 命令快照最大可接受年龄，单位：ms。 */
#define APP_STATUS_COPY_RETRY_MAX  3U    /**< 状态版本一致性复制的最大重试次数。 */

/**
 * @brief 应用层导航快照。
 *
 * @details
 * 该结构体是 Business 层对 Display、LoRa、日志和存储暴露的导航数据视图。
 * 字段来自 Framework 的 Navigation topic，并按业务使用习惯保留固定单位。
 */
typedef struct {
  Px4Lite_TopicHeader_t header; /**< 快照头，包含序号、采样时间、发布时间和有效标志。 */
  uint32_t valid_mask;          /**< 导航字段有效位，使用 `PX4LITE_NAV_VALID_*`。 */
  uint32_t gnss_utc_sec;        /**< GNSS UTC 当日秒数，单位：s；无效时为 0。 */
  uint32_t gnss_utc_date;       /**< RMC 日期，压缩格式 yymmdd；无有效日期时为 0。 */
  int32_t latitude_e7;          /**< 纬度，单位：degree * 1e7。 */
  int32_t longitude_e7;         /**< 经度，单位：degree * 1e7。 */
  int32_t altitude_mm;          /**< 融合或 GNSS 高度，单位：mm。 */
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
  uint8_t gnss_fix_type;        /**< GNSS 定位类型，沿用驱动层 fix type。 */
  uint8_t navigation_quality;   /**< 导航质量，0 表示最低，数值越高表示质量越好。 */
  uint8_t reserved;             /**< 保留字段，保持结构体对齐。 */
} App_NavigationSnapshot_t;

/**
 * @brief 应用层系统健康快照。
 *
 * @details
 * 该结构体汇总模块状态、阻塞故障、警告故障和最高告警，用于显示、日志和
 * 系统状态判断。复制时要求 Health 摘要和模块状态版本一致。
 */
typedef struct {
  Px4Lite_TopicHeader_t header;                         /**< 快照头，包含系统健康发布时间。 */
  Px4Lite_ModuleStatus_t modules[PX4LITE_MODULE_COUNT]; /**< 每个 Framework 模块的最新状态副本。 */
  uint32_t blocking_fault_mask;                         /**< 阻塞类故障位图，bit 对应模块编号。 */
  uint32_t warning_fault_mask;                          /**< 警告类故障位图，bit 对应模块编号。 */
  uint32_t not_ready_mask;                              /**< 未就绪模块位图，bit 对应模块编号。 */
  uint32_t status_version;                              /**< 模块状态版本号，用于一致性复制。 */
  uint16_t active_alarm_count;                          /**< 当前活动告警数量。 */
  uint16_t highest_fault_code;                          /**< 当前最高严重度告警的故障码。 */
  uint16_t highest_source_id;                           /**< 当前最高严重度告警的来源模块或设备 ID。 */
  uint8_t highest_severity;                             /**< 当前最高告警严重度，见 `Px4Lite_AlarmSeverity_t`。 */
  uint8_t system_ready;                                 /**< 系统就绪标志，1 表示关键模块满足运行条件。 */
  uint8_t reserved[2];                                  /**< 保留字段，保持结构体对齐。 */
} App_SystemSnapshot_t;

/**
 * @brief 应用层活动告警记录。
 */
typedef struct {
  uint16_t source_id;  /**< 告警来源模块或设备 ID。 */
  uint16_t fault_code; /**< 故障码，含义由来源模块定义。 */
  uint8_t severity;    /**< 告警严重度，见 `Px4Lite_AlarmSeverity_t`。 */
  uint8_t active;      /**< 活动标志，1 表示当前仍有效。 */
  uint16_t reserved;   /**< 保留字段，保持结构体对齐。 */
  uint32_t raised_ms;  /**< 首次触发时间，单位：ms。 */
  uint32_t updated_ms; /**< 最近更新时间，单位：ms。 */
  uint32_t detail;     /**< 来源模块附加详情，含义由 fault_code 定义。 */
} App_AlarmRecord_t;

/**
 * @brief 应用层告警表快照。
 */
typedef struct {
  Px4Lite_TopicHeader_t header;                    /**< 快照头，包含告警表发布时间。 */
  uint16_t active_count;                           /**< 当前活动告警数量。 */
  uint16_t highest_fault_code;                     /**< 当前最高严重度告警的故障码。 */
  uint16_t highest_source_id;                      /**< 当前最高严重度告警的来源 ID。 */
  uint8_t highest_severity;                        /**< 当前最高告警严重度。 */
  uint8_t reserved[3];                             /**< 保留字段，保持结构体对齐。 */
  App_AlarmRecord_t records[PX4LITE_MODULE_COUNT]; /**< 固定容量告警记录表。 */
} App_AlarmSnapshot_t;

/**
 * @brief 应用层环境和电源快照。
 */
typedef struct {
  Px4Lite_TopicHeader_t header; /**< 快照头，包含环境数据发布时间。 */
  float pressure_pa;            /**< 气压，单位：Pa。 */
  float temperature_c;          /**< 温度，单位：摄氏度。 */
  float relative_humidity_pct;  /**< 相对湿度，单位：%。 */
  uint32_t voltage_mv;          /**< 电池或输入电压，单位：mV。 */
  uint8_t battery_percent;      /**< 电量百分比，范围：0 到 100。 */
  uint8_t low_voltage;          /**< 低电压标志，1 表示低电压。 */
  uint16_t reserved;            /**< 保留字段，保持结构体对齐。 */
} App_EnvironmentSnapshot_t;

/**
 * @brief 应用层电机输出命令快照。
 *
 * @details
 * 该结构体用于显示和日志查看当前 Control 模块已发布的目标油门，不代表 ESC 或电机
 * 的真实反馈速度。
 */
typedef struct {
  Px4Lite_TopicHeader_t header;              /**< 快照头，包含 Control 发布时间。 */
  uint8_t duty_percent[PX4LITE_MOTOR_COUNT]; /**< 每路目标油门百分比，范围 0 到 100。 */
  uint8_t run_state;                         /**< 运行状态，1 表示已完成 ESC 预解锁。 */
  uint8_t speed_level;                       /**< 四路目标油门最大值，范围 0 到 100。 */
  uint16_t reserved;                         /**< 保留字段，保持结构体对齐。 */
} App_MotorSnapshot_t;

/**
 * @brief 应用层日期时间快照。
 */
typedef struct {
  Px4Lite_TopicHeader_t header;       /**< 快照头，包含 Framework 时间更新时间。 */
  uint32_t utc_date_ymd;              /**< UTC 日期，编码 YYYYMMDD。 */
  uint32_t utc_time_hhmmss;           /**< UTC 时间，编码 HHMMSS。 */
  uint32_t local_date_ymd;            /**< 本地显示日期，编码 YYYYMMDD。 */
  uint32_t local_time_hhmmss;         /**< 本地显示时间，编码 HHMMSS。 */
  uint32_t last_sync_ms;              /**< 最近一次 GNSS 校准的系统时间，未知时为 0。 */
  uint32_t sync_age_s;                /**< 最近一次 GNSS 校准距今秒数，未知时为 0。 */
  Px4Lite_TimeSource_t source;        /**< 当前时间来源。 */
  Px4Lite_TimeSyncState_t sync_state; /**< 当前校时状态。 */
} App_DateTimeSnapshot_t;

/**
 * @brief 复制新鲜且一致的导航快照。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 * @param[in] now_ms 当前系统毫秒时间，用于数据新鲜度判断。
 *
 * @return 复制结果。
 * @retval PX4LITE_OK 复制成功，`out` 中数据可用。
 * @retval PX4LITE_INVALID_PARAM 参数为空。
 * @retval PX4LITE_NOT_READY 尚无有效导航快照。
 * @retval PX4LITE_STALE 最新快照超过 `APP_NAVIGATION_MAX_AGE_MS`。
 */
Px4Lite_Result_t App_CopyNavigation(App_NavigationSnapshot_t *out, uint32_t now_ms);

/**
 * @brief 复制版本一致的系统健康和模块状态快照。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 * @param[in] now_ms 当前系统毫秒时间，用于 Health 快照新鲜度判断。
 *
 * @return 复制结果。
 * @retval PX4LITE_OK 复制成功。
 * @retval PX4LITE_INVALID_PARAM 参数为空。
 * @retval PX4LITE_NOT_READY 尚无有效系统健康快照。
 * @retval PX4LITE_STALE Health 快照超过 `APP_SYSTEM_MAX_AGE_MS`。
 * @retval PX4LITE_BUSY 多次复制后模块状态版本仍不一致。
 */
Px4Lite_Result_t App_CopySystem(App_SystemSnapshot_t *out, uint32_t now_ms);

/**
 * @brief 复制活动告警表快照。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 * @param[in] now_ms 当前系统毫秒时间，用于告警快照新鲜度判断。
 *
 * @return 复制结果。
 * @retval PX4LITE_OK 复制成功。
 * @retval PX4LITE_INVALID_PARAM 参数为空。
 * @retval PX4LITE_NOT_READY 尚无有效告警快照。
 * @retval PX4LITE_STALE 告警快照超过 `APP_ALARM_MAX_AGE_MS`。
 */
Px4Lite_Result_t App_CopyAlarm(App_AlarmSnapshot_t *out, uint32_t now_ms);

/**
 * @brief 复制新鲜的环境和电源快照。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 * @param[in] now_ms 当前系统毫秒时间，用于环境快照新鲜度判断。
 *
 * @return 复制结果。
 * @retval PX4LITE_OK 复制成功。
 * @retval PX4LITE_INVALID_PARAM 参数为空。
 * @retval PX4LITE_NOT_READY 尚无有效环境快照，或 Baro/Battery 数据均已过期。
 */
Px4Lite_Result_t App_CopyEnvironment(App_EnvironmentSnapshot_t *out, uint32_t now_ms);

/**
 * @brief 复制新鲜的电机输出命令快照。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 * @param[in] now_ms 当前系统毫秒时间，用于新鲜度判断。
 *
 * @return 复制结果。
 */
Px4Lite_Result_t App_CopyMotor(App_MotorSnapshot_t *out, uint32_t now_ms);

/**
 * @brief 设置单路电机目标油门百分比。
 *
 * @param[in] motor_index 电机编号，范围 0 到 `PX4LITE_MOTOR_COUNT - 1`。
 * @param[in] throttle_percent 目标油门百分比，范围 0 到 100，超过 100 时由 Control 限幅。
 *
 * @return 设置结果。
 * @retval PX4LITE_NOT_READY 当前为 REMOTE 显示模式，电机滑块只读，拒绝写本机 Control。
 */
Px4Lite_Result_t App_SetMotorThrottlePercent(uint8_t motor_index, uint8_t throttle_percent);

/**
 * @brief 复制统一日期时间快照。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 * @param[in] now_ms 当前系统毫秒时间，用于时间快照新鲜度判断。
 *
 * @return 复制结果。
 * @retval PX4LITE_OK 复制成功。
 * @retval PX4LITE_INVALID_PARAM 参数为空。
 * @retval PX4LITE_NOT_READY RTC 尚未有效且 GNSS 尚未完成校时。
 * @retval PX4LITE_STALE 时间快照超过可接受年龄。
 */
Px4Lite_Result_t App_CopyDateTime(App_DateTimeSnapshot_t *out, uint32_t now_ms);

/**
 * @brief 复制单个 Framework 模块的最新状态。
 *
 * @param[in] module_id 模块编号，必须小于 `PX4LITE_MODULE_COUNT`。
 * @param[out] out 输出缓冲区，不能为 NULL。
 *
 * @return 复制结果。
 * @retval PX4LITE_OK 复制成功。
 * @retval PX4LITE_INVALID_PARAM 模块编号非法或输出参数为空。
 * @retval PX4LITE_NOT_READY 模块状态尚未初始化。
 */
Px4Lite_Result_t App_GetModuleStatus(Px4Lite_ModuleId_t module_id, Px4Lite_ModuleStatus_t *out);

/**
 * @brief 复制 LoRa/MAVLink 通信统计。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 *
 * @note 本函数只复制统计值，不访问 LoRa 硬件，不阻塞。
 */
void App_GetCommStats(Px4Lite_CommDebugInfo_t *out);

/**
 * @brief 获取当前显示数据源模式。
 *
 * @return 当前模式，LOCAL 表示本机显示，REMOTE 表示远端显示。
 */
Px4Lite_RemoteMode_t App_GetRemoteDisplayMode(void);

/**
 * @brief 复制远端显示快照。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 * @param[in] now_ms 当前系统毫秒时间，用于远端快照新鲜度判断。
 *
 * @return 复制结果。
 */
Px4Lite_Result_t App_CopyRemoteTelemetry(Px4Lite_RemoteTelemetrySnapshot_t *out, uint32_t now_ms);

#endif
