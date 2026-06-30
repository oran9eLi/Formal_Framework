/**
 * @file px4lite_types.h
 * @brief Framework 层通用结果、模块状态、topic 头和业务数据类型。
 *
 * @details
 * 本文件是 Framework 与 Business、Platform Adapter、通信、日志之间的数据契约。
 * 字段单位、有效位和状态含义必须在这里保持稳定；跨层传递时不得直接发送
 * C 结构体内存，通信层必须逐字段编码。
 */

#ifndef PX4LITE_TYPES_H
#define PX4LITE_TYPES_H

#include <stdint.h>
#include "px4lite_local_msglog.h"

#ifndef PX4LITE_REMOTE_LOG_ENTRY_MAX
#define PX4LITE_REMOTE_LOG_ENTRY_MAX 5U
#endif

#define PX4LITE_COMM_RX_PAYLOAD_MAX 255U /**< 通信接收帧 payload 最大长度，单位：byte。 */

/**
 * @brief Framework 通用返回值。
 */
typedef enum {
  PX4LITE_OK = 0,        /**< 操作成功。 */
  PX4LITE_IDLE,          /**< 当前无待处理工作。 */
  PX4LITE_BUSY,          /**< 资源忙或一致性复制暂时失败。 */
  PX4LITE_INVALID_PARAM, /**< 输入参数非法。 */
  PX4LITE_NOT_READY,     /**< 模块尚未就绪或没有有效数据。 */
  PX4LITE_STALE,         /**< 数据存在但超过允许年龄。 */
  PX4LITE_IO_ERROR,      /**< 底层 I/O 或设备访问失败。 */
  PX4LITE_OVERFLOW       /**< 队列、FIFO 或缓冲区溢出。 */
} Px4Lite_Result_t;

/**
 * @brief Framework 公开模块状态。
 *
 * @note 驱动只记录硬件事实并提升 ONLINE/DEGRADED；OFFLINE/FAILED 由 Health task 独占判定。
 */
typedef enum {
  PX4LITE_STATE_UNINITIALIZED = 0, /**< 尚未初始化。 */
  PX4LITE_STATE_STARTING,          /**< 正在启动或自检。 */
  PX4LITE_STATE_ONLINE,            /**< 在线且主功能可用。 */
  PX4LITE_STATE_DEGRADED,          /**< 降级运行，功能部分可用。 */
  PX4LITE_STATE_OFFLINE,           /**< 超时或失联，由 Health 判定。 */
  PX4LITE_STATE_FAILED,            /**< 失败且需要恢复或人工处理，由 Health 判定。 */
  PX4LITE_STATE_DISABLED           /**< 配置关闭，不参与运行。 */
} Px4Lite_State_t;

/**
 * @brief Framework 模块编号。
 *
 * @details
 * 编号用于状态数组、故障位图、告警来源和注册表索引。新增模块时必须同步检查
 * `PX4LITE_MODULE_COUNT`、状态位图宽度和完成度文档。
 */
typedef enum {
  PX4LITE_MODULE_GNSS = 0,  /**< GNSS 模块。 */
  PX4LITE_MODULE_IMU,       /**< IMU 模块。 */
  PX4LITE_MODULE_BARO,      /**< 气压计/环境模块。 */
  PX4LITE_MODULE_BATTERY,   /**< 电源/电池模块。 */
  PX4LITE_MODULE_LORA,      /**< LoRa 通信模块。 */
  PX4LITE_MODULE_5G,        /**< 5G-A 连接管理模块，可发送 AT 指令查询/配置连接，不承载业务数据。 */
  PX4LITE_MODULE_STORAGE,   /**< SD/FatFs 存储模块。 */
  PX4LITE_MODULE_REMOTE_ID, /**< Remote ID 边界状态模块，真实数据链路不从本 STM32 板直通。 */
  PX4LITE_MODULE_DISPLAY,   /**< 显示模块。 */
  PX4LITE_MODULE_CONTROL,   /**< 控制命令预留模块。 */
  PX4LITE_MODULE_ALARM,     /**< 告警模块。 */
  PX4LITE_MODULE_SYSTEM,    /**< 系统健康模块。 */
  PX4LITE_MODULE_ESTIMATOR, /**< 姿态/导航估计模块。 */
  PX4LITE_MODULE_BUSINESS,  /**< Business 应用模块。 */
  PX4LITE_MODULE_COUNT      /**< 模块数量，必须保持为枚举最后一项。 */
} Px4Lite_ModuleId_t;

/**
 * @brief 所有 Framework topic 的公共头。
 */
typedef struct {
  uint32_t sequence;        /**< 发布序号，每次发布递增。 */
  uint32_t sample_time_ms;  /**< 数据采样完成时间，单位：ms。 */
  uint32_t publish_time_ms; /**< Framework 发布时间，单位：ms。 */
  uint32_t flags;           /**< 数据标志位，使用 `PX4LITE_DATA_*`。 */
  uint16_t device_id;       /**< 设备或模块实例 ID，单实例模块可为 0。 */
  uint8_t valid;            /**< 有效标志，1 表示数据可被消费者使用。 */
  uint8_t quality;          /**< 质量等级，0 表示最低，数值越高表示质量越好。 */
} Px4Lite_TopicHeader_t;

/**
 * @brief Framework 内部统一 UTC 日期时间。
 */
typedef struct {
  uint16_t year;   /**< 完整年份，范围 2000 到 2099。 */
  uint8_t month;   /**< 月份，范围 1 到 12。 */
  uint8_t day;     /**< 日期，范围 1 到 31。 */
  uint8_t hours;   /**< 小时，范围 0 到 23。 */
  uint8_t minutes; /**< 分钟，范围 0 到 59。 */
  uint8_t seconds; /**< 秒，范围 0 到 59。 */
} Px4Lite_UtcDateTime_t;

#define PX4LITE_DATA_VALID      (1UL << 0) /**< 数据通过基本有效性检查。 */
#define PX4LITE_DATA_CALIBRATED (1UL << 1) /**< 数据已经过标定补偿。 */
#define PX4LITE_DATA_FILTERED   (1UL << 2) /**< 数据已经过滤波。 */
#define PX4LITE_DATA_FUSED      (1UL << 3) /**< 数据来自多源融合。 */
#define PX4LITE_DATA_DEGRADED   (1UL << 4) /**< 数据可用但处于降级状态。 */

/**
 * @brief 单个 Framework 模块的运行状态。
 */
typedef struct {
  Px4Lite_ModuleId_t module_id; /**< 模块编号。 */
  Px4Lite_State_t state;        /**< 当前公开状态。 */
  uint16_t fault_code;          /**< 当前故障码，0 表示无故障。 */
  uint8_t severity;             /**< 当前故障严重度，见 `Px4Lite_AlarmSeverity_t`。 */
  uint8_t recovery_count;       /**< 已触发恢复次数，饱和或回绕策略由实现定义。 */
  uint32_t state_since_ms;      /**< 进入当前状态的时间，单位：ms。 */
  uint32_t last_rx_ms;          /**< 最近收到原始数据或硬件响应的时间，单位：ms。 */
  uint32_t last_valid_ms;       /**< 最近产生有效数据的时间，单位：ms。 */
  uint32_t error_count;         /**< 累计错误次数。 */
  uint32_t drop_count;          /**< 累计丢弃次数。 */
  uint16_t consecutive_errors;  /**< 连续错误次数。 */
  uint16_t consecutive_valid;   /**< 连续有效次数。 */
} Px4Lite_ModuleStatus_t;

/**
 * @brief GNSS 传感器测量 topic。
 */
typedef struct {
  Px4Lite_TopicHeader_t header; /**< topic 公共头。 */
  int32_t latitude_e7;          /**< 纬度，单位：degree * 1e7。 */
  int32_t longitude_e7;         /**< 经度，单位：degree * 1e7。 */
  int32_t altitude_mm;          /**< 海拔高度，单位：mm。 */
  uint32_t ground_speed_cms;    /**< 地速，单位：cm/s。 */
  uint32_t utc_sec;             /**< UTC 当日秒数，单位：s。 */
  uint32_t utc_date;            /**< RMC 日期，压缩格式 yymmdd；无有效日期时为 0。 */
  uint16_t heading_deg100;      /**< 地面航向，单位：degree * 100。 */
  uint16_t hdop_x100;           /**< 水平精度因子，单位：HDOP * 100。 */
  uint8_t fix_type;             /**< 定位类型，沿用 GNSS 驱动定义。 */
  uint8_t fix_dimension;        /**< 定位维度，0/2D/3D 按驱动解析结果填写。 */
  uint8_t satellites_used;      /**< 当前定位使用的总卫星数。 */
  uint8_t gps_visible;          /**< GPS 可见卫星数。 */
  uint8_t bds_visible;          /**< 北斗可见卫星数。 */
  uint8_t gps_used;             /**< GPS 参与解算卫星数。 */
  uint8_t bds_used;             /**< 北斗参与解算卫星数。 */
  uint8_t antenna_state;        /**< 天线状态，含义由 GNSS 驱动定义。 */
  uint8_t reserved;             /**< 保留字段，保持结构体对齐。 */
} Px4Lite_SensorGnss_t;

/**
 * @brief IMU 传感器测量 topic。
 */
typedef struct {
  Px4Lite_TopicHeader_t header; /**< topic 公共头。 */
  int32_t accel_mg[3];          /**< X/Y/Z 加速度，单位：mg。 */
  int32_t gyro_mdps[3];         /**< X/Y/Z 角速度，单位：mdps。 */
  int16_t temperature_cdeg;     /**< 芯片温度，单位：摄氏度 * 100。 */
  uint16_t sample_period_us;    /**< 相邻 IMU 样本周期，单位：us。 */
} Px4Lite_SensorImu_t;

/**
 * @brief 气压计/环境测量 topic。
 */
typedef struct {
  Px4Lite_TopicHeader_t header; /**< topic 公共头。 */
  float pressure_pa;            /**< 气压，单位：Pa。 */
  float temperature_c;          /**< 温度，单位：摄氏度。 */
  float relative_humidity_pct;  /**< 相对湿度，单位：%。 */
  int32_t pressure_altitude_mm; /**< 气压高度，单位：mm。 */
  int32_t vertical_speed_cms;   /**< 垂直速度，单位：cm/s。 */
} Px4Lite_SensorBaro_t;

/**
 * @brief 电源/电池状态 topic。
 */
typedef struct {
  Px4Lite_TopicHeader_t header; /**< topic 公共头。 */
  uint32_t voltage_mv;          /**< 电压，单位：mV。 */
  int32_t current_ma;           /**< 电流，单位：mA；未知时可为 0。 */
  uint8_t percent;              /**< 电量百分比，范围：0 到 100。 */
  uint8_t low_voltage;          /**< 低电压标志，1 表示低电压。 */
  uint16_t reserved;            /**< 保留字段，保持结构体对齐。 */
} Px4Lite_BatteryStatus_t;

#ifndef PX4LITE_MOTOR_COUNT
#define PX4LITE_MOTOR_COUNT 4U /**< 电机输出通道数量。 */
#endif

/**
 * @brief 电机输出命令快照。
 *
 * @details
 * 该 topic 由 Control 模块单写者发布，用于 Display、日志和后续遥测读取当前命令
 * 状态。字段表示已下发目标，不表示电机真实转速或 ESC 反馈。
 */
typedef struct {
  Px4Lite_TopicHeader_t header;                  /**< topic 公共头。 */
  uint8_t duty_percent[PX4LITE_MOTOR_COUNT];     /**< 每路电机目标油门百分比，范围 0 到 100。 */
  uint8_t run_state;                             /**< 运行状态，1 表示 ESC 已完成预解锁并允许输出目标油门。 */
  uint8_t speed_level;                           /**< 兼容显示字段，当前等于四路目标油门最大值，范围 0 到 100。 */
  uint16_t reserved;                             /**< 保留字段，保持结构体对齐。 */
} Px4Lite_MotorOutputs_t;

#define PX4LITE_NAV_VALID_ATTITUDE (1UL << 0) /**< 姿态角和角速度有效。 */
#define PX4LITE_NAV_VALID_POSITION (1UL << 1) /**< 经纬度位置有效。 */
#define PX4LITE_NAV_VALID_VELOCITY (1UL << 2) /**< NED 速度有效。 */
#define PX4LITE_NAV_VALID_ALTITUDE (1UL << 3) /**< 高度和垂直速度有效。 */
#define PX4LITE_NAV_VALID_YAW_REL  (1UL << 4) /**< 航向相对参考有效。 */

/**
 * @brief 车辆导航域 topic。
 */
typedef struct {
  Px4Lite_TopicHeader_t header; /**< topic 公共头。 */
  uint32_t valid_mask;          /**< 导航字段有效位，使用 `PX4LITE_NAV_VALID_*`。 */
  uint32_t gnss_utc_sec;        /**< GNSS UTC 当日秒数，单位：s。 */
  uint32_t gnss_utc_date;       /**< RMC 日期，压缩格式 yymmdd；无有效日期时为 0。 */
  int32_t latitude_e7;          /**< 纬度，单位：degree * 1e7。 */
  int32_t longitude_e7;         /**< 经度，单位：degree * 1e7。 */
  int32_t fused_altitude_mm;    /**< 融合高度，单位：mm。 */
  int32_t vertical_speed_cms;   /**< 垂直速度，单位：cm/s。 */
  int32_t roll_deg100;          /**< 横滚角，单位：degree * 100。 */
  int32_t pitch_deg100;         /**< 俯仰角，单位：degree * 100。 */
  int32_t yaw_deg100;           /**< 航向角，单位：degree * 100。 */
  int32_t roll_rate_dps100;     /**< 横滚角速度，单位：(degree/s) * 100。 */
  int32_t pitch_rate_dps100;    /**< 俯仰角速度，单位：(degree/s) * 100。 */
  int32_t yaw_rate_dps100;      /**< 航向角速度，单位：(degree/s) * 100。 */
  int32_t velocity_north_cms;   /**< 北向速度，单位：cm/s。 */
  int32_t velocity_east_cms;    /**< 东向速度，单位：cm/s。 */
  int32_t velocity_down_cms;    /**< 地向速度，单位：cm/s。 */
  uint16_t hdop_x100;           /**< 水平精度因子，单位：HDOP * 100。 */
  uint8_t satellites_used;      /**< 当前定位使用的卫星数量。 */
  uint8_t gnss_fix_type;        /**< GNSS 定位类型。 */
  uint8_t navigation_quality;   /**< 导航质量，0 表示最低，数值越高表示质量越好。 */
  uint8_t reserved;             /**< 保留字段，保持结构体对齐。 */
} Px4Lite_VehicleNavigation_t;

/**
 * @brief 系统健康 topic。
 */
typedef struct {
  Px4Lite_TopicHeader_t header;                       /**< topic 公共头。 */
  Px4Lite_State_t module_state[PX4LITE_MODULE_COUNT]; /**< 每个模块的公开状态。 */
  uint32_t blocking_fault_mask;                       /**< 阻塞类故障位图，bit 对应模块编号。 */
  uint32_t warning_fault_mask;                        /**< 警告类故障位图，bit 对应模块编号。 */
  uint32_t not_ready_mask;                            /**< 未就绪模块位图，bit 对应模块编号。 */
  uint16_t imu_fifo_peak;                             /**< IMU FIFO 历史峰值占用。 */
  uint16_t alarm_queue_peak;                          /**< 告警队列历史峰值占用。 */
  uint16_t log_pool_free_min;                         /**< 日志池最小剩余块数。 */
  uint16_t reserved;                                  /**< 保留字段，保持结构体对齐。 */
  uint32_t status_version;                            /**< 模块状态版本号，用于一致性复制。 */
  uint32_t imu_fifo_overflow;                         /**< IMU FIFO 累计溢出次数。 */
  uint32_t log_drop_count;                            /**< 日志累计丢弃次数。 */
} Px4Lite_SystemHealth_t;

/**
 * @brief 告警严重度。
 */
typedef enum {
  PX4LITE_ALARM_INFO = 0, /**< 信息提示。 */
  PX4LITE_ALARM_WARNING,  /**< 警告，不阻塞主流程。 */
  PX4LITE_ALARM_ERROR,    /**< 错误，需要记录和恢复。 */
  PX4LITE_ALARM_CRITICAL, /**< 严重错误，可能影响关键功能。 */
  PX4LITE_ALARM_FATAL     /**< 致命错误，系统应进入失败或保护状态。 */
} Px4Lite_AlarmSeverity_t;

/**
 * @brief 告警事件。
 */
typedef struct {
  uint32_t timestamp_ms;            /**< 事件产生时间，单位：ms。 */
  uint32_t sequence;                /**< 告警事件序号。 */
  uint16_t source_id;               /**< 告警来源模块或设备 ID。 */
  uint16_t fault_code;              /**< 故障码，含义由来源模块定义。 */
  Px4Lite_AlarmSeverity_t severity; /**< 告警严重度。 */
  uint8_t active;                   /**< 活动标志，1 表示触发，0 表示清除。 */
  uint8_t reserved[3];              /**< 保留字段，保持结构体对齐。 */
  uint32_t detail;                  /**< 附加详情，含义由 fault_code 定义。 */
} Px4Lite_AlarmEvent_t;

/**
 * @brief 告警表中的一条活动记录。
 */
typedef struct {
  uint16_t source_id;               /**< 告警来源模块或设备 ID。 */
  uint16_t fault_code;              /**< 故障码，含义由来源模块定义。 */
  Px4Lite_AlarmSeverity_t severity; /**< 告警严重度。 */
  uint8_t active;                   /**< 活动标志，1 表示当前仍有效。 */
  uint8_t reserved[3];              /**< 保留字段，保持结构体对齐。 */
  uint32_t raised_ms;               /**< 首次触发时间，单位：ms。 */
  uint32_t updated_ms;              /**< 最近更新时间，单位：ms。 */
  uint32_t detail;                  /**< 附加详情，含义由 fault_code 定义。 */
} Px4Lite_AlarmRecord_t;

/**
 * @brief 告警表快照。
 */
typedef struct {
  Px4Lite_TopicHeader_t header;                        /**< topic 公共头。 */
  uint16_t active_count;                               /**< 当前活动告警数量。 */
  uint16_t highest_fault_code;                         /**< 当前最高严重度告警的故障码。 */
  uint16_t highest_source_id;                          /**< 当前最高严重度告警的来源 ID。 */
  Px4Lite_AlarmSeverity_t highest_severity;            /**< 当前最高告警严重度。 */
  uint8_t reserved[3];                                 /**< 保留字段，保持结构体对齐。 */
  Px4Lite_AlarmRecord_t records[PX4LITE_MODULE_COUNT]; /**< 固定容量活动告警表。 */
} Px4Lite_AlarmSnapshot_t;

/**
 * @brief 应用命令。
 */
typedef struct {
  uint32_t timestamp_ms; /**< 命令生成时间，单位：ms。 */
  uint32_t sequence;     /**< 命令序号。 */
  uint16_t command_id;   /**< 命令编号。 */
  uint16_t source_id;    /**< 命令来源 ID。 */
  int32_t parameter[4];  /**< 命令参数，单位由 command_id 定义。 */
} Px4Lite_Command_t;

/**
 * @brief 应用命令执行回执。
 */
typedef struct {
  uint32_t timestamp_ms;     /**< ACK 生成时间，单位：ms。 */
  uint32_t command_sequence; /**< 对应的命令序号。 */
  uint16_t command_id;       /**< 对应的命令编号。 */
  uint16_t result;           /**< 执行结果，含义由命令模块定义。 */
} Px4Lite_CommandAck_t;

/**
 * @brief LoRa/MAVLink 通信调试统计。
 */
typedef struct {
  uint32_t rx_frame_count;            /**< 已接收完整帧数量。 */
  uint32_t tx_frame_count;            /**< 本机发送流程完成帧数量，不证明对端在线。 */
  uint32_t tx_busy_count;             /**< 发送忙导致未提交的次数。 */
  uint32_t mav_heartbeat_count;       /**< 已调度 HEARTBEAT 消息数量。 */
  uint32_t mav_gps_raw_count;         /**< 已调度 GPS_RAW_INT 消息数量。 */
  uint32_t mav_gnss_detail_count;     /**< 已调度 GNSS detail/statustext 类消息数量。 */
  uint32_t mav_attitude_count;        /**< 已调度 ATTITUDE 消息数量。 */
  uint32_t mav_position_count;        /**< 已调度 GLOBAL_POSITION_INT 消息数量。 */
  uint32_t mav_sys_status_count;      /**< 已调度 SYS_STATUS 消息数量。 */
  uint32_t mav_module_state_count;    /**< 已调度 MODSTAT0/MODSTAT1 模块状态消息数量。 */
  uint32_t mav_battery_status_count;  /**< 已调度 BATTERY_STATUS 消息数量。 */
  uint32_t mav_scaled_pressure_count; /**< 已调度 SCALED_PRESSURE 消息数量。 */
  uint32_t mav_statustext_count;      /**< 已调度 STATUSTEXT 消息数量。 */
  uint32_t mav_command_count;         /**< 已调度 COMMAND_LONG 控制消息数量。 */
  uint32_t mav_command_ack_tx_count;  /**< 已发送 COMMAND_ACK 数量。 */
  uint32_t mav_command_ack_rx_count;  /**< 已接收 COMMAND_ACK 数量。 */
  uint32_t mav_no_data_count;         /**< 因无数据未发送的次数。 */
  uint32_t mav_stale_count;           /**< 因数据过期未发送的次数。 */
  uint32_t mav_error_count;           /**< MAVLink 编码或发送错误次数。 */
  uint32_t mav_last_tx_msg_id;        /**< 最近调度的 MAVLink message id。 */
  uint32_t crc_error_count;           /**< 接收 CRC 错误次数。 */
  uint32_t send_error_count;          /**< 底层发送错误次数。 */
  uint32_t parse_error_count;         /**< 接收解析错误次数。 */
  uint32_t rx_byte_count;             /**< 接收字节累计数。 */
  uint32_t rx_overflow_count;         /**< 接收缓冲溢出次数。 */
  uint32_t rx_drop_count;             /**< 接收丢弃次数。 */
  uint32_t rx_sequence_expected_count; /**< MAVLink 序号估算的应收帧总数，含已收和跳号丢帧。 */
  uint32_t rx_sequence_lost_count;     /**< MAVLink 序号跳号估算的丢帧数量。 */
  uint32_t last_rx_ms;                /**< 最近收到对端合法 MAVLink 帧时间，单位：ms。 */
  uint32_t last_tx_ms;                /**< 最近本机发送流程完成时间，单位：ms，不证明对端在线。 */
  uint32_t last_msg_id;               /**< 最近接收的 MAVLink message id。 */
  uint16_t rx_loss_rate_x10;          /**< 接收侧估算丢包率，单位：0.1%，1000 表示 100.0%。 */
  uint16_t reserved;                  /**< 保留字段，保持结构体对齐。 */
} Px4Lite_CommDebugInfo_t;

/**
 * @brief 通信层最近接收的一帧 MAVLink 数据副本。
 * @details
 * 该结构只描述协议帧事实，不解释业务含义。后续从机状态、外设数值或 Remote ID
 * 数据应在独立解码层按 msg_id/sysid 路由，不得在 LoRa 驱动内写业务逻辑。
 */
typedef struct {
  uint32_t rx_time_ms;                         /**< 接收完成时间，单位：ms。 */
  uint32_t msg_id;                             /**< MAVLink message id。 */
  uint16_t frame_len;                          /**< 完整 MAVLink 帧长度，单位：byte。 */
  uint8_t system_id;                           /**< MAVLink system id，后续可映射临时节点号。 */
  uint8_t component_id;                        /**< MAVLink component id。 */
  uint8_t sequence;                            /**< MAVLink packet sequence。 */
  uint8_t payload_len;                         /**< payload 长度，单位：byte。 */
  uint8_t payload[PX4LITE_COMM_RX_PAYLOAD_MAX]; /**< MAVLink payload 副本。 */
} Px4Lite_CommRxFrame_t;

#define PX4LITE_REMOTE_VALID_HEARTBEAT   (1UL << 0) /**< 已收到远端 HEARTBEAT。 */
#define PX4LITE_REMOTE_VALID_MODULES     (1UL << 1) /**< 已收到远端模块状态。 */
#define PX4LITE_REMOTE_VALID_NAVIGATION  (1UL << 2) /**< 已收到远端导航数据。 */
#define PX4LITE_REMOTE_VALID_ATTITUDE    (1UL << 3) /**< 已收到远端姿态数据。 */
#define PX4LITE_REMOTE_VALID_ENVIRONMENT (1UL << 4) /**< 已收到远端环境数据。 */
#define PX4LITE_REMOTE_VALID_BATTERY     (1UL << 5) /**< 已收到远端电源数据。 */
#define PX4LITE_REMOTE_VALID_ALARM       (1UL << 6) /**< 已收到远端告警数据。 */
#define PX4LITE_REMOTE_VALID_LOG         (1UL << 7) /**< 已收到远端消息日志序号或增量。 */
#define PX4LITE_REMOTE_VALID_MOTOR       (1UL << 8) /**< 已收到远端电机输出数据。 */

#define PX4LITE_REMOTE_ALARM_RECORD_MAX 5U /**< 远端告警表缓存容量，限制静态 RAM 占用并匹配当前显示行数。 */

typedef enum {
  PX4LITE_REMOTE_NODE_EMPTY = 0,
  PX4LITE_REMOTE_NODE_DISCOVERED,
  PX4LITE_REMOTE_NODE_ACTIVE,
  PX4LITE_REMOTE_NODE_STALE
} Px4Lite_RemoteNodeState_t;

typedef struct {
  uint8_t node_id;
  uint8_t system_id;
  uint8_t component_id;
  uint8_t heartbeat_type;
  Px4Lite_RemoteNodeState_t state;
  uint32_t last_heartbeat_ms;
  uint32_t last_data_ms;
  uint32_t rx_frame_count;
  uint32_t rx_sequence_lost_count;
  uint16_t rx_loss_rate_x10;
  uint8_t heartbeat_system_status;
  uint8_t reserved1;
} Px4Lite_RemoteNodeStatus_t;

/**
 * @brief 远端节点解码后的显示遥测快照。
 * @details
 * Comm task 从 LoRa 收到的 MAVLink 帧中逐字段解码本结构，Business/Display 只消费该快照。
 * LoRa 驱动仍只负责帧事实，不承载业务解释。
 */
typedef struct {
  Px4Lite_TopicHeader_t header;                       /**< topic 公共头。 */
  uint32_t valid_mask;                                /**< 远端数据有效位，使用 `PX4LITE_REMOTE_VALID_*`。 */
  uint32_t stale_mask;                                /**< 远端数据过期位，使用 `PX4LITE_REMOTE_VALID_*`。 */
  uint32_t last_rx_ms;                                /**< 最近收到远端合法 MAVLink 帧时间，单位：ms。 */
  uint32_t heartbeat_update_ms;                       /**< 最近收到 HEARTBEAT 的时间，单位：ms。 */
  uint32_t navigation_update_ms;                      /**< 最近收到导航数据的时间，单位：ms。 */
  uint32_t attitude_update_ms;                        /**< 最近收到姿态数据的时间，单位：ms。 */
  uint32_t environment_update_ms;                     /**< 最近收到环境数据的时间，单位：ms。 */
  uint32_t battery_update_ms;                         /**< 最近收到电源数据的时间，单位：ms。 */
  uint32_t modules_update_ms;                         /**< 最近收到模块状态数据的时间，单位：ms。 */
  uint32_t alarm_update_ms;                           /**< 最近收到告警数据的时间，单位：ms。 */
  uint32_t log_update_ms;                             /**< 最近收到消息日志序号或增量的时间，单位：ms。 */
  uint32_t motor_update_ms;                           /**< 最近收到电机输出数据的时间，单位：ms。 */
  uint32_t last_msg_id;                               /**< 最近解码的 MAVLink message id。 */
  uint32_t rx_frame_count;                            /**< 已接收合法 MAVLink 帧计数。 */
  uint32_t decoded_frame_count;                       /**< 已成功映射到远端快照的帧计数。 */
  uint32_t rx_sequence_expected_count;                /**< 当前远端节点按 MAVLink 序号估算的应收帧总数。 */
  uint32_t rx_sequence_lost_count;                    /**< 当前远端节点按 MAVLink 序号跳号估算的丢帧数量。 */
  uint16_t rx_loss_rate_x10;                          /**< 当前远端节点接收侧估算丢包率，单位：0.1%。 */
  uint8_t last_packet_sequence;                       /**< 当前远端节点最近 MAVLink packet sequence。 */
  uint8_t sequence_seen;                              /**< 当前远端节点是否已有序号基准。 */
  uint8_t heartbeat_type;                             /**< HEARTBEAT type 字段，用于区分飞控、GCS、伴侣等节点类型。 */
  uint8_t heartbeat_autopilot;                        /**< HEARTBEAT autopilot 字段。 */
  uint8_t heartbeat_base_mode;                        /**< HEARTBEAT base_mode 字段。 */
  uint8_t heartbeat_system_status;                    /**< HEARTBEAT system_status 字段。 */
  uint8_t heartbeat_mavlink_version;                  /**< HEARTBEAT mavlink_version 字段。 */
  uint8_t reserved_heartbeat[3];                      /**< 保留字段，保持结构体对齐。 */
  uint32_t module_state_valid_mask;                   /**< 远端模块状态有效位，bit 对应 `Px4Lite_ModuleId_t`。 */
  Px4Lite_State_t module_state[PX4LITE_MODULE_COUNT]; /**< 远端模块公开状态。 */
  uint32_t gnss_utc_sec;                              /**< GNSS UTC 当日秒数，单位：s；未提供时为 0。 */
  uint32_t gnss_utc_date;                             /**< GNSS 日期，压缩格式 yymmdd；未提供时为 0。 */
  int32_t latitude_e7;                                /**< 纬度，单位：degree * 1e7。 */
  int32_t longitude_e7;                               /**< 经度，单位：degree * 1e7。 */
  int32_t altitude_mm;                                /**< 高度，单位：mm。 */
  int32_t velocity_north_cms;                         /**< 北向速度，单位：cm/s。 */
  int32_t velocity_east_cms;                          /**< 东向速度，单位：cm/s。 */
  int32_t velocity_down_cms;                          /**< 地向速度，单位：cm/s。 */
  int32_t roll_deg100;                                /**< 横滚角，单位：degree * 100。 */
  int32_t pitch_deg100;                               /**< 俯仰角，单位：degree * 100。 */
  int32_t yaw_deg100;                                 /**< 航向角，单位：degree * 100。 */
  int32_t roll_rate_dps100;                           /**< 横滚角速度，单位：(degree/s) * 100。 */
  int32_t pitch_rate_dps100;                          /**< 俯仰角速度，单位：(degree/s) * 100。 */
  int32_t yaw_rate_dps100;                            /**< 航向角速度，单位：(degree/s) * 100。 */
  uint16_t hdop_x100;                                 /**< HDOP * 100。 */
  uint8_t satellites_used;                            /**< 远端定位卫星数量。 */
  uint8_t gnss_fix_type;                              /**< 远端 GNSS 定位类型。 */
  uint16_t reserved0;                                 /**< 保留字段，保持对齐。 */
  float pressure_pa;                                  /**< 气压，单位：Pa。 */
  float temperature_c;                                /**< 温度，单位：摄氏度。 */
  float relative_humidity_pct;                        /**< 相对湿度，单位：%。 */
  uint32_t voltage_mv;                                /**< 电压，单位：mV。 */
  uint32_t voltage2_mv;                               /**< 第二电池或外设独立供电电压，单位：mV。 */
  uint32_t alarm_active_mask;                         /**< 远端活动告警来源位图，bit 对应 source_id。 */
  uint16_t highest_fault_code;                        /**< 远端最高告警码。 */
  uint16_t highest_source_id;                         /**< 远端最高告警来源。 */
  uint8_t highest_severity;                           /**< 远端最高告警严重度。 */
  uint8_t battery_percent;                            /**< 电量百分比，范围 0 到 100。 */
  uint8_t battery2_percent;                           /**< 第二电池电量百分比，范围 0 到 100。 */
  uint8_t low_voltage;                                /**< 主电池低电压标志，1 表示低电压。 */
  uint8_t low_voltage2;                               /**< 第二电池低电压标志，1 表示低电压。 */
  uint8_t alarm_record_count;                         /**< 已缓存远端告警记录数量。 */
  uint8_t alarm_table_version;                        /**< 远端告警表版本号。 */
  uint16_t log_latest_seq;                            /**< 远端消息日志最新序号，0 表示未知。 */
  uint8_t log_count;                                  /**< 本次接收到的远端日志条数。 */
  uint8_t motor_run_state;                            /**< 远端电机运行状态，1 表示允许输出。 */
  uint8_t motor_speed_level;                          /**< 远端电机目标油门最大值，范围 0 到 100。 */
  uint8_t reserved_remote2;                           /**< 保留字段，保持结构体对齐。 */
  uint8_t motor_duty_percent[PX4LITE_MOTOR_COUNT];    /**< 远端每路电机目标油门百分比，范围 0 到 100。 */
  Px4Lite_AlarmRecord_t alarm_records[PX4LITE_REMOTE_ALARM_RECORD_MAX]; /**< 远端活动告警表缓存。 */
  uint8_t system_id;                                  /**< 远端 MAVLink system id。 */
  uint8_t component_id;                               /**< 远端 MAVLink component id。 */
  Px4Lite_LogEntry_t log_entries[PX4LITE_REMOTE_LOG_ENTRY_MAX]; /**< 远端结构化消息日志缓存，旧到新排列。 */
} Px4Lite_RemoteTelemetry_t;

/**
 * @brief 计算毫秒时间差，对轻微跨任务未来时间样本饱和为 0。
 *
 * @param[in] now_ms 当前时间，单位：ms。
 * @param[in] then_ms 历史时间，单位：ms。
 *
 * @return `now_ms - then_ms` 的非负差值，单位：ms。
 */
static __inline uint32_t Px4Lite_ElapsedMs(uint32_t now_ms, uint32_t then_ms)
{
  int32_t delta = (int32_t)(now_ms - then_ms);

  return (delta >= 0) ? (uint32_t)delta : 0U;
}

/**
 * @brief 判断 topic 样本是否有效且未超过最大年龄。
 *
 * @param[in] header topic 头指针，不能为 NULL。
 * @param[in] now_ms 当前时间，单位：ms。
 * @param[in] max_age_ms 最大允许数据年龄，单位：ms。
 *
 * @return 1 表示新鲜可用，0 表示无效、未 ready 或已过期。
 */
static __inline uint8_t Px4Lite_IsFresh(const Px4Lite_TopicHeader_t *header, uint32_t now_ms, uint32_t max_age_ms)
{
  if ((header == 0) || (header->valid == 0U)) { return 0U; }

  return (Px4Lite_ElapsedMs(now_ms, header->sample_time_ms) <= max_age_ms) ? 1U : 0U;
}

#endif
