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
#include "px4lite_runtime_stats.h"
typedef Px4Lite_RuntimeSnapshot_t App_RuntimeSnapshot_t; /**< 常驻资源只读摘要。 */
typedef Px4Lite_TaskRuntime_t App_TaskRuntime_t; /**< 任务栈只读行。 */
typedef Px4Lite_DisplayRuntime_t App_DisplayRuntime_t; /**< 显示所有者测量摘要。 */
/** @brief biz_system调用低频采集，不调用LVGL。 */
void App_RuntimeStatsStep(uint32_t now_ms);
/** @brief 复制资源摘要，年龄由measured_ms判断。 */
Px4Lite_Result_t App_CopyRuntimeStats(App_RuntimeSnapshot_t *out, uint32_t now_ms);
/** @brief 复制任务资源行，不触发扫描。 */
Px4Lite_Result_t App_CopyTaskRuntime(uint8_t index, App_TaskRuntime_t *out, uint32_t now_ms);
/** @brief 唯一显示任务提交耗时及LVGL测量。 */
void App_ReportDisplayRuntime(const App_DisplayRuntime_t *sample);
/** @brief 单调微秒时基，用于短周期耗时无符号差值。 */
uint32_t App_RuntimeGetUs(void);
#include "px4lite_lora_lab.h"
#include "px4lite_attitude.h"
#include "px4lite_control.h"
typedef Px4Lite_AttitudeStatus_t App_AttitudeStatus_t; /**< 姿态操作只读快照。 */
/** @brief 主动校准/相对归零/恢复请求；远端只读时拒绝，本机请求结果异步查询。 */
Px4Lite_Result_t App_RequestAttitudeAction(Px4Lite_AttitudeAction_t action);
/** @brief 查询本机姿态操作状态，包含校准进度和相对显示基准。 */
Px4Lite_Result_t App_CopyAttitudeStatus(App_AttitudeStatus_t *out);
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

#define APP_ENV_VALID_PRESSURE_TEMPERATURE (1UL << 0) /**< 气压和温度有效。 */
#define APP_ENV_VALID_VOLTAGE1 (1UL << 1) /**< 主电池电压有效。 */
#define APP_ENV_VALID_VOLTAGE2 (1UL << 2) /**< 第二电池电压有效。 */
#define APP_ENV_VALID_CURRENT1 (1UL << 3) /**< 主电池电流有效。 */
#define APP_ENV_VALID_PERCENT1 (1UL << 4) /**< 主电池电量百分比有效。 */
#define APP_ENV_VALID_CURRENT2 (1UL << 5) /**< 第二电池电流有效。 */
#define APP_ENV_VALID_PERCENT2 (1UL << 6) /**< 第二电池电量百分比有效。 */
#define APP_ENV_VALID_HUMIDITY (1UL << 7) /**< 相对湿度有效。 */
#define APP_ENV_VALID_ENVIRONMENT (APP_ENV_VALID_PRESSURE_TEMPERATURE | APP_ENV_VALID_HUMIDITY) /**< 环境来源字段组合。 */
#define APP_ENV_VALID_BATTERY1 (APP_ENV_VALID_VOLTAGE1 | APP_ENV_VALID_CURRENT1 | APP_ENV_VALID_PERCENT1) /**< 主电池来源字段组合。 */
#define APP_ENV_VALID_BATTERY2 (APP_ENV_VALID_VOLTAGE2 | APP_ENV_VALID_CURRENT2 | APP_ENV_VALID_PERCENT2) /**< 第二电池来源字段组合。 */

/**
 * @brief 应用层环境和电源快照；各来源分别报告有效性。
 */
typedef struct {
  Px4Lite_TopicHeader_t header; /**< 快照头，包含环境数据发布时间。 */
  uint32_t valid_mask;          /**< 新鲜字段位，使用 APP_ENV_VALID_*；未置位字段清零。 */
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
  uint8_t duty_percent[PX4LITE_MOTOR_COUNT]; /**< 每路实际输出油门百分比(含姿态修正)，范围 0 到 100。 */
  uint16_t pulse_us[PX4LITE_MOTOR_COUNT];    /**< Control 最终写入各路 PWM 的高电平脉宽，单位：us。 */
  uint8_t run_state;                         /**< 运行状态，1 表示已完成 ESC 预解锁。 */
  uint8_t speed_level;                       /**< 四路目标油门最大值，范围 0 到 100。 */
  uint8_t base_percent[PX4LITE_MOTOR_COUNT]; /**< 姿态修正前的基础油门(滑块目标)，范围 0 到 100。 */
  uint8_t control_mode;                      /**< `Px4Lite_ControlMode_t` 数值；本机快照有效。 */
  uint8_t operation_state;                   /**< 当前实验过程，见 `Px4Lite_MotorOperationState_t`。 */
  uint8_t control_mode_valid;                /**< 1 表示模式由目标设备明确上报。 */
  uint8_t operation_state_valid;              /**< 1 表示过程状态由目标设备明确上报。 */
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
  int32_t current_ma;                                    /**< 主电池电流，单位：mA；未测量时为 0。 */
  int32_t current2_ma;                                   /**< 第二电池电流，单位：mA；未测量时为 0。 */
  uint8_t battery_percent;                               /**< 电量百分比，范围 0 到 100。 */
  uint8_t battery2_percent;                              /**< 第二电池电量百分比，范围 0 到 100。 */
  uint8_t low_voltage;                                   /**< 主电池低电压标志，1 表示低电压。 */
  uint8_t low_voltage2;                                  /**< 第二电池低电压标志，1 表示低电压。 */
  uint8_t motor_duty_percent[APP_DISPLAY_MOTOR_COUNT];   /**< 每路电机实际输出油门百分比(含姿态修正)。 */
  uint16_t motor_pulse_us[APP_DISPLAY_MOTOR_COUNT];      /**< 每路电机最终 PWM 高电平脉宽，单位：us。 */
  uint8_t motor_run_state;                               /**< 电机运行状态，1 表示允许输出目标油门。 */
  uint8_t motor_speed_level;                             /**< 四路目标油门最大值，范围 0 到 100。 */
  uint8_t motor_base_percent[APP_DISPLAY_MOTOR_COUNT];   /**< 每路电机姿态修正前的基础油门(滑块目标)。 */
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
  uint8_t identity_known; /**< 长期身份已确认且新鲜。 */
  uint8_t heartbeat_known; /**< 已收到真实心跳，不把 BASIC_ID 当心跳。 */
  uint8_t id_conflict; /**< 同地址不同身份，禁止混合完整数据。 */
  char product_id[21]; /**< 长期身份，未知为空。 */
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
 * @brief Start a simple four-motor auto takeoff throttle ramp to 60%.
 *
 * @return Command result.
 */
Px4Lite_Result_t App_StartMotorAutoTakeoff(void);

/**
 * @brief Start a simple four-motor auto landing throttle ramp down to 0%.
 *
 * @return Command result.
 */
Px4Lite_Result_t App_StartMotorAutoLanding(void);

/** @brief 本机选择直控或姿态辅助模式；远端显示模式下拒绝。 */
Px4Lite_Result_t App_SetMotorControlMode(Px4Lite_ControlMode_t mode);

Px4Lite_Result_t App_EmergencyStopMotors(void);



/**
 * @brief Report whether the motor-battery power inhibit is currently latched.
 *
 * @return 1 when output is inhibited, 0 when allowed.
 */
uint8_t App_IsMotorPowerInhibited(void);

/** @brief 只读查询本机有效新鲜采样的低压事实，缺帧/过期不冒称低压。 */
uint8_t App_IsMotorPowerLowVoltage(void);

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
 * @note 仅供系统启动且调度器尚未运行时显式复位；页面清视图及任务恢复不得调用。
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

/** @brief 兼容旧远端缓存读取；主页面与本机历史不使用此接口，退出旧同步归 M4。 */
uint16_t App_CopyRemoteMessageLog(Px4Lite_LogEntry_t *entries, uint16_t capacity, uint16_t *last_seq, uint32_t now_ms);

/**
 * @brief 返回远端消息日志显示版本。
 *
 * @param[in] now_ms 当前系统毫秒时间，用于判断远端日志新鲜度。
 *
 * @return 远端序号与更新时间组合值；无新鲜远端日志时返回 0。
 *
 * @note 旧缓存兼容接口；当前 Display 不再导入远端历史。
 */
uint32_t App_GetRemoteMessageLogVersion(uint32_t now_ms);

/**
 * @brief 兼容旧接口：全局远端模式已退出，启用请求返回 0。
 *
 * @param[in] enabled 非零启用被拒绝，零仅确认本机模式，不改变通信发送。
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @return enabled 为零返回 1；非零请求返回 0，不产生通信副作用。
 */
uint8_t App_SetRemoteViewEnabled(uint8_t enabled, uint32_t now_ms);

/**
 * @brief 选择通信诊断对象，不修改本机主页面和发送调度。
 *
 * @param[in] node_id 远端节点 ID，不能等于本机 DCDW 后缀。
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @return 1 表示选择成功，0 表示节点非法或选择失败。
 */
uint8_t App_SelectRemoteNode(uint8_t node_id, uint32_t now_ms);

/**
 * @brief 兼容旧接口：不再存在全局查看租约。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @return 固定返回 0；显式远端数据页须使用快照时间判断 STALE。
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
 * @brief 返回本次启动固定的 node_id(=MAVLink sysid)，来源为保存值或 UID 默认值。
 *
 * @return 本机 node_id，范围 [1,250]。
 */
uint8_t App_GetLocalNodeId(void);
/** @brief 实验观察字段有效且新鲜的显示位；缺测和过期均不置位。 */
enum {
  APP_LAB_POSITION=1UL<<0, APP_LAB_ATTITUDE=1UL<<1,
  APP_LAB_VOLTAGE1=1UL<<2, APP_LAB_VOLTAGE2=1UL<<3,
  APP_LAB_ENVIRONMENT=1UL<<4, APP_LAB_HUMIDITY=1UL<<5,
  APP_LAB_ALTITUDE=1UL<<6, APP_LAB_VELOCITY=1UL<<7,
  APP_LAB_CURRENT1=1UL<<8, APP_LAB_CURRENT2=1UL<<9,
  APP_LAB_PERCENT1=1UL<<10, APP_LAB_PERCENT2=1UL<<11
};
/** @brief 单对端实验观察视图，所有年龄均为本机接收年龄，不表示链路时延。 */
typedef struct {
  uint32_t valid_bits; /**< 各组有效且新鲜的显示位。 */
  uint32_t heartbeat_age_ms; /**< 心跳年龄，单位 ms；UINT32_MAX 未收到。 */
  uint32_t data_age_ms; /**< 最近主数据年龄，单位 ms；UINT32_MAX 未收到。 */
  uint32_t gnss_age_ms,attitude_age_ms; /**< 定位/姿态域接收年龄，ms；UINT32_MAX 未收到。 */
  uint32_t power1_age_ms,power2_age_ms; /**< 两路电源域接收年龄，ms；UINT32_MAX 未收到。 */
  uint32_t environment_age_ms,humidity_age_ms; /**< 温压/湿度接收年龄，ms；UINT32_MAX 未收到。 */
  uint32_t rx_frames; /**< 本次活动快照累计接收帧数。 */
  uint16_t loss_rate_x10; /**< 序号估算丢帧率，百分比 * 10；不是射频丢包实测值。 */
  int32_t latitude_e7,longitude_e7; /**< 经纬度，degree * 1e7。 */
  int32_t altitude_mm; /**< GNSS 高度，mm；仅 ALTITUDE 位有效。 */
  int32_t velocity_north_cms,velocity_east_cms; /**< 北/东速度，cm/s；仅 VELOCITY 位有效。 */
  int32_t roll_deg100,pitch_deg100,yaw_deg100; /**< 姿态，degree * 100。 */
  int32_t roll_rate_dps100,pitch_rate_dps100,yaw_rate_dps100; /**< 三轴角速度，degree/s * 100。 */
  uint32_t voltage_mv,voltage2_mv; /**< 两路电压，单位 mV。 */
  uint32_t current_ma,current2_ma; /**< 两路电流，mA；按独立有效位显示。 */
  uint8_t battery_percent,battery2_percent; /**< 两路电量，0..100%；0 也是合法测量值。 */
  uint8_t gnss_fix_type,satellites_used; /**< Fix 等级和卫星数；仅位置有效时解释。 */
  int32_t temperature_deg100; /**< 温度，摄氏度 * 100。 */
  uint32_t pressure_pa,humidity_pct100; /**< 气压 Pa、湿度 % * 100。 */
  uint8_t node_id; /**< 所选 node，0 表示未选。 */
  uint8_t available; /**< 已有选择后的完整快照。 */
  uint8_t identity_known; /**< 身份明确。 */
  uint8_t id_conflict; /**< 地址冲突。 */
  char product_id[21]; /**< UID 派生的长期身份，20 字符加终止符。 */
} App_RemoteLabView_t;
/** @brief 格式化本机长期产品身份；只读身份，不进行硬件 I/O。 */
void App_FormatLocalProductId(char *out,uint8_t capacity);
/** @brief 教学发送请求入口；不会在显示任务中访问 UART/Flash。 */
Px4Lite_Result_t App_RequestLoraLab(const Px4Lite_LabRequest_t *request);
/** @brief 复制教学实验与配置事实。 */
void App_CopyLoraLab(Px4Lite_LabSnapshot_t *out);
/** @brief 复制单对端显示视图，过期字段不会标为有效。 */
void App_CopyRemoteLabView(App_RemoteLabView_t *out,uint32_t now_ms);

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
 * @brief 兼容入口：请求将当前测量设为本机显示的相对零位，不校准传感器。
 *
 * @return OK 仅表示已排队，最终结果通过 App_CopyAttitudeStatus 获取。
 */
Px4Lite_Result_t App_RequestAttitudeLevelCalibration(void);

/**
 * @brief 兼容旧调用者，主页面的数据来源固定本机。
 *
 * @return 固定返回 `PX4LITE_REMOTE_MODE_LOCAL`。
 */
Px4Lite_RemoteMode_t App_GetRemoteDisplayMode(void);

#endif
