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

/*
 * 后续任意外设必须先通过模块状态进入 App API；只有存在明确业务数据消费需求时，
 * 才新增命名清楚的 App_Copy* 快照。AT 指令类控制面由所属服务/驱动封装，不向
 * Business 暴露原始 AT 收发。业务、显示、通信不得直接包含 Framework topic、BSP 或 Driver 头文件。
 */
#define APP_NAVIGATION_MAX_AGE_MS  1500U /**< Navigation 快照最大可接受年龄，单位：ms。 */
#define APP_SYSTEM_MAX_AGE_MS      500U  /**< System/Health 快照最大可接受年龄，单位：ms。 */
#define APP_ENVIRONMENT_MAX_AGE_MS 2500U /**< Environment 快照最大可接受年龄，单位：ms。 */
#define APP_ALARM_MAX_AGE_MS       500U  /**< Alarm 快照最大可接受年龄，单位：ms。 */
#define APP_MOTOR_MAX_AGE_MS       500U  /**< Motor 命令快照最大可接受年龄，单位：ms。 */
#define APP_DATETIME_MAX_AGE_MS    2500U /**< DateTime 快照最大可接受年龄，单位：ms。 */
#define APP_REMOTE_MAX_AGE_MS      12000U /**< 远端显示快照最大可接受年龄，单位：ms。 */
#define APP_STATUS_COPY_RETRY_MAX  3U    /**< 状态版本一致性复制的最大重试次数。 */
#define APP_DISPLAY_MOTOR_COUNT    4U    /**< Display 视图固定显示的电机数量。 */
#define APP_DISPLAY_ALARM_MAX      16U   /**< Display 视图固定导出的告警记录容量。 */

/**
 * @brief 应用层显示状态灯语义。
 */
typedef enum {
  APP_VIEW_STATE_UNKNOWN = 0, /**< 状态未知或尚未初始化。 */
  APP_VIEW_STATE_STARTING,    /**< 模块启动中或等待首帧数据。 */
  APP_VIEW_STATE_ONLINE,      /**< 模块在线且数据可用。 */
  APP_VIEW_STATE_DEGRADED,    /**< 模块降级但仍有可用事实。 */
  APP_VIEW_STATE_OFFLINE,     /**< 模块离线。 */
  APP_VIEW_STATE_FAILED       /**< 模块失败。 */
} App_ViewState_t;

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
 * @brief 应用层告警轻量摘要。
 */
typedef struct {
  uint16_t active_count;       /**< 当前活动告警数量。 */
  uint16_t highest_fault_code; /**< 当前最高严重度告警故障码。 */
  uint16_t highest_source_id;  /**< 当前最高严重度告警来源 ID。 */
  uint8_t highest_severity;    /**< 当前最高告警严重度。 */
  uint8_t reserved;            /**< 保留字段，保持结构体对齐。 */
} App_AlarmSummary_t;

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
  uint32_t voltage2_mv;         /**< 第二电池或外设独立供电电压，单位：mV。 */
  int32_t current_ma;           /**< 电池 1 电流，单位：mA；未接入或采样失败时为 0。 */
  int32_t current2_ma;          /**< 电池 2 电流，单位：mA；未接入或采样失败时为 0。 */
  uint8_t battery_percent;      /**< 电量百分比，范围：0 到 100。 */
  uint8_t battery2_percent;     /**< 第二电池电量百分比，范围：0 到 100。 */
  uint8_t low_voltage;          /**< 低电压标志，1 表示低电压。 */
  uint8_t low_voltage2;         /**< 第二电池低电压标志，1 表示低电压。 */
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
 * @brief Display 使用的模块状态视图。
 */
typedef struct {
  App_ViewState_t state; /**< 应用层显示状态。 */
  uint16_t fault_code;   /**< 当前故障码，0 表示无故障。 */
  uint8_t severity;      /**< 告警严重度，值越大越严重。 */
  uint8_t reserved;      /**< 保留字段，保持结构体对齐。 */
} App_ModuleView_t;

/**
 * @brief Display 专用聚合视图。
 *
 * @details
 * 本结构体把页面需要的 Framework 状态、模块编号、告警和有效位统一转换为 App 层字段。
 * Display 只消费该视图，不直接依赖 Framework 模块枚举、故障枚举或 topic 有效位。
 */
typedef struct {
  uint8_t navigation_valid;                              /**< 导航字段是否可显示。 */
  uint8_t attitude_valid;                                /**< 姿态字段是否可显示。 */
  uint8_t date_time_valid;                               /**< 日期时间是否可显示。 */
  uint8_t system_valid;                                  /**< 系统状态快照是否可显示。 */
  uint8_t alarm_valid;                                   /**< 告警表是否可显示。 */
  uint8_t environment_valid;                             /**< 环境/电源字段是否可显示。 */
  uint8_t motor_valid;                                   /**< 电机输出字段是否可显示。 */
  uint8_t any_valid;                                     /**< 至少一个主要快照可用。 */
  uint32_t status_version;                               /**< 模块状态版本号。 */
  uint32_t warning_fault_mask;                           /**< 警告故障位图。 */
  uint32_t blocking_fault_mask;                          /**< 阻塞故障位图。 */
  uint16_t highest_fault_code;                           /**< 当前最高严重度告警故障码。 */
  uint16_t highest_source_id;                            /**< 当前最高严重度告警来源 ID。 */
  uint8_t system_ready;                                  /**< 系统就绪标志，1 表示就绪。 */
  uint8_t view_node_id;                                  /**< 当前显示对象 node_id；Remote ID 接入前作为临时身份。 */
  uint8_t view_system_id;                                /**< 当前显示对象 MAVLink system id。 */
  uint8_t view_remote_id_valid;                          /**< Remote ID 显示有效标志；当前使用板卡身份 DCDW-xxx。 */
  App_ModuleView_t gnss;                                 /**< GNSS 模块显示状态。 */
  App_ModuleView_t imu;                                  /**< IMU 模块显示状态。 */
  App_ModuleView_t baro;                                 /**< Baro 模块显示状态。 */
  App_ModuleView_t battery;                              /**< Battery 模块显示状态。 */
  App_ModuleView_t lora;                                 /**< LoRa 模块显示状态。 */
  App_ModuleView_t storage;                              /**< Storage 模块显示状态。 */
  App_ModuleView_t control;                              /**< Control 模块显示状态。 */
  App_ModuleView_t five_g;                               /**< 5G-A 模块显示状态。 */
  App_ModuleView_t remote_id;                            /**< Remote ID 本地发送通道模块显示状态。 */
  uint32_t gnss_utc_sec;                                 /**< GNSS UTC 当日秒数，单位：s。 */
  uint32_t gnss_utc_date;                                /**< RMC 日期，压缩格式 yymmdd。 */
  int32_t latitude_e7;                                   /**< 纬度，单位：degree * 1e7。 */
  int32_t longitude_e7;                                  /**< 经度，单位：degree * 1e7。 */
  int32_t altitude_mm;                                   /**< 高度，单位：mm。 */
  int32_t velocity_north_cms;                            /**< 北向速度，单位：cm/s。 */
  int32_t velocity_east_cms;                             /**< 东向速度，单位：cm/s。 */
  int32_t roll_deg100;                                   /**< 横滚角，单位：degree * 100。 */
  int32_t pitch_deg100;                                  /**< 俯仰角，单位：degree * 100。 */
  int32_t yaw_deg100;                                    /**< 航向角，单位：degree * 100。 */
  uint16_t hdop_x100;                                    /**< HDOP * 100。 */
  uint8_t satellites_used;                               /**< 当前定位使用卫星数。 */
  uint8_t gnss_fix_type;                                 /**< GNSS 定位类型。 */
  uint32_t local_date_ymd;                               /**< 本地日期，编码 YYYYMMDD。 */
  uint32_t local_time_hhmmss;                            /**< 本地时间，编码 HHMMSS。 */
  float pressure_pa;                                     /**< 气压，单位：Pa。 */
  float temperature_c;                                   /**< 温度，单位：摄氏度。 */
  float relative_humidity_pct;                           /**< 相对湿度，单位：%。 */
  uint32_t voltage_mv;                                   /**< 电压，单位：mV。 */
  uint32_t voltage2_mv;                                  /**< 第二电池或外设独立供电电压，单位：mV。 */
  uint8_t battery_percent;                               /**< 电量百分比，范围 0 到 100。 */
  uint8_t battery2_percent;                              /**< 第二电池电量百分比，范围 0 到 100。 */
  uint8_t low_voltage;                                   /**< 主电池低电压标志，1 表示低电压。 */
  uint8_t low_voltage2;                                  /**< 第二电池低电压标志，1 表示低电压。 */
  uint8_t motor_duty_percent[APP_DISPLAY_MOTOR_COUNT];   /**< 每路电机目标油门百分比。 */
  uint8_t motor_run_state;                               /**< 电机运行状态，1 表示允许输出目标油门。 */
  uint8_t motor_speed_level;                             /**< 四路目标油门最大值，范围 0 到 100。 */
  uint32_t lora_tx_count;                                /**< LoRa 本机发送完成帧计数。 */
  uint32_t lora_rx_count;                                /**< LoRa 接收合法帧计数。 */
  uint32_t lora_lost_count;                              /**< LoRa 接收侧按 MAVLink 序号估算的丢帧数量。 */
  uint32_t lora_ack_count;                               /**< LoRa/MAVLink 真实 COMMAND_ACK 收发计数。 */
  uint16_t lora_loss_rate_x10;                           /**< LoRa 接收侧估算丢包率，单位：0.1%。 */
  uint16_t reserved2;                                    /**< 保留字段，保持结构体对齐。 */
  uint16_t alarm_active_count;                           /**< 活动告警数量。 */
  uint16_t alarm_highest_fault_code;                     /**< 告警表最高故障码。 */
  App_AlarmRecord_t alarms[APP_DISPLAY_ALARM_MAX];       /**< Display 使用的活动告警记录表。 */
} App_DisplaySnapshot_t;

/**
 * @brief App 层远端节点列表显示状态。
 *
 * @details
 * 状态来源为 Framework MAVLink RX 远端槽位表。通信页用该状态决定选择框灯色；
 * `node_id` 从 DCDW-xxx 数字后缀派生，用于 LoRa/MAVLink 路由和临时身份显示。
 */
typedef enum {
  APP_REMOTE_NODE_EMPTY = 0,                             /**< 未发现节点。 */
  APP_REMOTE_NODE_DISCOVERED,                            /**< 只收到新鲜心跳，主数据尚未新鲜。 */
  APP_REMOTE_NODE_ACTIVE,                                /**< 心跳和主数据均新鲜。 */
  APP_REMOTE_NODE_STALE                                  /**< 已发现但心跳过期。 */
} App_RemoteNodeState_t;

/**
 * @brief App 层远端节点选择框视图。
 *
 * @details
 * 本结构体只包含通信页和 Debug 自测需要的轻量字段：身份、状态、最近心跳/主数据时间、
 * 接收计数和按 MAVLink `seq` 估算的丢包率。界面显示的数据必须继续通过
 * `App_CopyRemoteDisplaySnapshot()` 读取，不能在页面层直接消费 Framework 远端槽位。
 */
typedef struct {
  uint8_t node_id;                                       /**< 远端节点 ID，由 DCDW-xxx 数字后缀派生。 */
  uint8_t system_id;                                     /**< 对应 MAVLink system id。 */
  uint8_t heartbeat_type;                                /**< HEARTBEAT type，用于调试和后续节点分类。 */
  uint8_t heartbeat_system_status;                       /**< HEARTBEAT system_status，用于调试和后续健康分类。 */
  App_RemoteNodeState_t state;                           /**< App 层远端节点显示状态。 */
  uint32_t last_heartbeat_ms;                            /**< 最近一次心跳时间，单位 ms。 */
  uint32_t last_data_ms;                                 /**< 最近一次主数据时间，单位 ms。 */
  uint32_t rx_frame_count;                               /**< 当前节点合法接收帧计数。 */
  uint32_t rx_sequence_lost_count;                       /**< 当前节点按 MAVLink seq 估算的丢帧数。 */
  uint16_t rx_loss_rate_x10;                             /**< 当前节点接收侧丢包率，单位 0.1%。 */
  uint16_t reserved;                                     /**< 保留字段，保持结构体对齐。 */
} App_RemoteNodeView_t;

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
 * @brief 复制告警轻量摘要，周期调试和状态显示优先使用本接口。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 * @param[in] now_ms 当前系统毫秒时间，用于新鲜度判断。
 *
 * @return 复制结果。
 */
Px4Lite_Result_t App_CopyAlarmSummary(App_AlarmSummary_t *out, uint32_t now_ms);

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
 */
Px4Lite_Result_t App_SetMotorThrottlePercent(uint8_t motor_index, uint8_t throttle_percent);

/**
 * @brief 设置单路电机目标油门百分比，并返回应用层布尔结果。
 *
 * @param[in] motor_index 电机编号，范围 0 到 `APP_DISPLAY_MOTOR_COUNT - 1`。
 * @param[in] throttle_percent 目标油门百分比，范围 0 到 100。
 *
 * @return 1 表示设置成功，0 表示设置失败。
 */
uint8_t App_CommandMotorThrottlePercent(uint8_t motor_index, uint8_t throttle_percent);

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
 * @brief 复制 Display 专用聚合视图。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 * @param[in] now_ms 当前系统毫秒时间，用于各快照新鲜度判断。
 *
 * @return 1 表示至少一个主要显示快照可用，0 表示无可用显示数据。
 */
uint8_t App_CopyDisplaySnapshot(App_DisplaySnapshot_t *out, uint32_t now_ms);

/**
 * @brief 复制远端节点 Display 专用聚合视图。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 * @param[in] now_ms 当前系统毫秒时间，用于远端快照新鲜度判断。
 *
 * @return 1 表示至少一个远端主要显示快照可用，0 表示无可用远端显示数据。
 */
uint8_t App_CopyRemoteDisplaySnapshot(App_DisplaySnapshot_t *out, uint32_t now_ms);

/**
 * @brief 清空本机固定环形消息日志。
 *
 * @note 这是 Business 层访问 Framework 本机消息日志的唯一 App API 包装入口。
 */
void App_ResetLocalMessageLog(void);

/**
 * @brief 向本机固定环形消息日志写入一条结构化消息。
 *
 * @param[in] message_id 业务消息编号。
 * @param[in] time_hhmmss 显示侧压缩时间，格式 HHMMSS。
 * @param[in] value0 预留数值 0。
 * @param[in] value1 预留数值 1。
 * @param[in] value2 预留数值 2。
 * @param[in] severity 显示和远端转发使用的严重度。
 */
void App_PushLocalMessageLog(uint16_t message_id, uint32_t time_hhmmss, int32_t value0, int32_t value1, int32_t value2, uint8_t severity);

/**
 * @brief 复制本机固定环形消息日志。
 *
 * @param[out] entries 输出日志数组，可为 NULL。
 * @param[in] capacity 输出数组容量。
 * @param[out] version 日志版本号，可为 NULL。
 * @param[out] last_seq 最新日志序号，可为 NULL。
 *
 * @return 已复制条目数量。
 */
uint16_t App_CopyLocalMessageLog(Px4Lite_LogEntry_t *entries, uint16_t capacity, uint32_t *version, uint16_t *last_seq);

uint16_t App_CopyRemoteMessageLog(Px4Lite_LogEntry_t *entries, uint16_t capacity, uint16_t *last_seq, uint32_t now_ms);

/**
 * @brief 返回远端消息日志显示版本。
 *
 * @param[in] now_ms 当前系统毫秒时间，用于判断远端日志新鲜度。
 *
 * @return 远端日志最新序号；无新鲜远端日志时返回 0。
 *
 * @note Display 用该版本驱动 LVGL 日志区重绘，避免远端日志依赖本机日志版本。
 */
uint32_t App_GetRemoteMessageLogVersion(uint32_t now_ms);

/**
 * @brief 开启或关闭当前选中远端节点的主数据查看租约。
 *
 * @param[in] enabled 1 表示开启主数据查看，0 表示关闭。
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @return 1 表示命令已提交，0 表示当前没有合法远端节点或提交失败。
 */
uint8_t App_SetRemoteViewEnabled(uint8_t enabled, uint32_t now_ms);

/**
 * @brief 选择一个远端节点并开启主数据查看。
 *
 * @param[in] node_id 远端节点 ID，不能等于本机 DCDW 后缀。
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @return 1 表示选择成功，0 表示角色不允许、节点非法或命令提交失败。
 */
uint8_t App_SelectRemoteNode(uint8_t node_id, uint32_t now_ms);

/**
 * @brief 查询远端查看租约是否已经超时。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @return 1 表示已超时，0 表示未超时；主机角色当前不会自动超时退回。
 */
uint8_t App_RemoteViewExpired(uint32_t now_ms);

/**
 * @brief 复制当前选中的远端节点 ID。
 *
 * @param[out] node_id 输出远端节点 ID，不能为 NULL。
 *
 * @return 1 表示复制成功，0 表示参数无效或 Framework 尚未就绪。
 */
uint8_t App_GetSelectedRemoteNode(uint8_t *node_id);

/**
 * @brief 返回本机 node_id(=MAVLink sysid，由芯片 UID 派生)，用于本地视图身份显示。
 *
 * @return 本机 node_id，范围 [1,250]。
 */
uint8_t App_GetLocalNodeId(void);

/**
 * @brief 复制 LVGL 通信页使用的远端节点列表。
 *
 * @param[out] out 输出数组，不能为 NULL。
 * @param[in] capacity 输出数组容量。
 * @param[out] count 实际写入节点数量，不能为 NULL。
 * @param[in] now_ms 当前系统毫秒时间，用于状态新鲜度判断。
 *
 * @return 复制结果；无可显示节点时返回 `PX4LITE_NOT_READY`。
 *
 * @note 本接口按主从角色过滤节点，并把 Framework 远端槽位状态映射为 App 层视图。
 */
Px4Lite_Result_t App_CopyRemoteNodeStatuses(App_RemoteNodeView_t *out, uint8_t capacity, uint8_t *count, uint32_t now_ms);

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
 * @brief 复制最近一帧 LoRa/MAVLink 接收数据。
 * @param[out] out 输出缓冲区，不能为 NULL。
 * @return 复制结果。
 * @note 本接口只提供协议帧事实；后续远端外设状态和值应新增独立解码快照。
 */
Px4Lite_Result_t App_CopyCommRxFrame(Px4Lite_CommRxFrame_t *out);

/**
 * @brief 获取模块或故障来源的显示名称。
 *
 * @param[in] source_id 模块或告警来源 ID。
 * @param[in] fault_code 故障码；当 source_id 不是已知模块时用于按故障域回退。
 *
 * @return 指向静态只读字符串的指针，调用方不得修改或释放。
 *
 * @note 本接口用于 Display、日志等应用消费者，不向页面层暴露 Framework 故障枚举细节。
 */
const char *App_GetModuleDisplayName(uint16_t source_id, uint16_t fault_code);

/**
 * @brief 获取故障码对应的显示原因文本。
 *
 * @param[in] fault_code 故障码。
 *
 * @return 指向静态只读字符串的指针，未知故障返回通用告警文本。
 */
const char *App_GetFaultReasonText(uint16_t fault_code);

/**
 * @brief 请求按当前姿态重做水平校准（地平仪以当前姿势归零）。
 *
 * @return 请求结果。
 */
Px4Lite_Result_t App_RequestAttitudeLevelCalibration(void);

/**
 * @brief 获取当前远程显示模式（LOCAL/REMOTE），供显示取数层选源。
 *
 * @return 当前模式，见 `Px4Lite_RemoteMode_t`。
 */
Px4Lite_RemoteMode_t App_GetRemoteDisplayMode(void);

#endif
