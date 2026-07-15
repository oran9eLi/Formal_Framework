/**
 * @file app_display_model.h
 * @brief 显示层统一取数接口：模式无关的显示视图模型。
 *
 * @details
 * 本文件是"暴露给显示层的契约"，老显示(`Display/`)与后续 LVGL 界面都通过这里取数，
 * 不再各自判断 LOCAL/REMOTE，也不直接读 Framework topic 或 `RemoteTelemetry`。
 * 模式解析(LOCAL/REMOTE 选源)与远端→`App_*Snapshot_t` 归一只在本模块实现一次。
 *
 * 所属层级：Business/App 层。构建在 `app_data_api.h` 之上：
 * - 本机域调用 `App_Copy*`；
 * - 远端域调用 `App_CopyRemoteTelemetry` 并归一到同一套 `App_*Snapshot_t`。
 *
 * 依赖边界：
 * - 允许 `#include "app_data_api.h"`，复用其快照结构体与本机/远端取数。
 * - 不允许包含 `Display/` 或 LVGL 头文件(契约不依赖任何具体显示实现)。
 * - 不允许包含 BSP/Sensor 头文件或读取驱动私有变量。
 *
 * 取数返回值统一表达新鲜度：`PX4LITE_OK` 新鲜、`PX4LITE_STALE` 已收到但过期(UI 保留
 * 最后值并标 stale)、`PX4LITE_NOT_READY` 从未收到(UI 显示无效/清零)。所有 `_ms` 入参均为
 * 真实毫秒值。
 */

#ifndef APP_DISPLAY_MODEL_H
#define APP_DISPLAY_MODEL_H

#include <stdint.h>
#include "app_data_api.h"

#define APP_DISPLAY_LOG_CAP 9U /**< 显示消息日志快照容量，与屏幕可见行数一致。 */

/** @brief 显示视图各域编号，用于 `App_DisplayModel_t` 的 ready/stale 位图。 */
typedef enum {
  APP_DISPLAY_DOMAIN_NAVIGATION = 0, /**< 导航/姿态域。 */
  APP_DISPLAY_DOMAIN_ENVIRONMENT,    /**< 环境/电源域。 */
  APP_DISPLAY_DOMAIN_SYSTEM,         /**< 系统/模块状态域。 */
  APP_DISPLAY_DOMAIN_ALARM,          /**< 告警域。 */
  APP_DISPLAY_DOMAIN_MOTOR,          /**< 电机只读显示域。 */
  APP_DISPLAY_DOMAIN_DATETIME,       /**< 日期时间域。 */
  APP_DISPLAY_DOMAIN_MESSAGE_LOG,    /**< 消息日志域。 */
  APP_DISPLAY_DOMAIN_COUNT           /**< 域数量，保持为枚举最后一项。 */
} App_DisplayDomain_t;

/**
 * @brief 本机链路诊断(白名单域)。
 *
 * @details
 * REMOTE 模式下这些字段仍取**显示端本机**值：它们反映显示端自己的 LoRa 链路健康，
 * 不取远端值。来源为 `App_GetCommStats` 与本机 LoRa 模块状态。
 */
typedef struct {
  uint32_t tx_frame_count; /**< 本机 LoRa 发送完成帧数。 */
  uint32_t rx_frame_count; /**< 本机 LoRa 接收帧数。 */
  uint16_t loss_permille;  /**< 接收链路丢包率，单位 ‰(0~1000)，靠 MAVLink seq 跳变估算。 */
  uint8_t lora_state;      /**< 本机 LoRa 模块公开状态(见 `Px4Lite_State_t`)，用于状态灯。 */
  uint8_t reserved;        /**< 保留字段，保持结构体对齐。 */
} App_DisplayLinkStatus_t;

/**
 * @brief 一条结构化消息日志条目。
 *
 * @details
 * message_id 为与显示无关的协议/枚举编号(由实现层映射)，使本结构不依赖 Display 类型，
 * 供本机屏幕、远端同构显示与 PC 监控共用。
 */
typedef struct {
  uint16_t sequence;     /**< 日志序号，用于去重和远端增量同步。 */
  uint16_t message_id;   /**< 消息编号(协议/枚举)，与显示渲染解耦。 */
  uint32_t time_hhmmss;  /**< 事件时间，编码 HHMMSS；时间来自事件源设备。 */
  uint16_t fault_code;   /**< 关联故障码，0 表示无故障。 */
  uint8_t severity;      /**< 严重度，见 `Px4Lite_AlarmSeverity_t`。 */
  uint8_t source_id;     /**< 来源模块/设备 ID。 */
  uint8_t active;        /**< 告警类消息的触发/恢复标志，1 表示触发。 */
  uint8_t reserved[3];   /**< 保留字段，保持结构体对齐。 */
} App_DisplayLogEntry_t;

/**
 * @brief 显示消息日志快照。
 *
 * @details
 * 按时间顺序保存最近 `count` 条(最多 `APP_DISPLAY_LOG_CAP`)。LOCAL 模式来自本机业务事件，
 * REMOTE 模式来自目标设备同步日志(第二件事落地后)。
 */
typedef struct {
  uint32_t version;                                  /**< 日志缓冲版本号，变化即需重绘。 */
  uint16_t count;                                    /**< 有效条目数，范围 0~`APP_DISPLAY_LOG_CAP`。 */
  uint16_t reserved;                                 /**< 保留字段，保持结构体对齐。 */
  App_DisplayLogEntry_t entries[APP_DISPLAY_LOG_CAP]; /**< 日志条目，按时间从旧到新。 */
} App_DisplayLogSnapshot_t;

/**
 * @brief 一次显示周期的完整视图模型(便利封装)。
 *
 * @details
 * 各域已按当前模式解析。`ready_mask`/`stale_mask` 以 `App_DisplayDomain_t` 为位序：
 * ready 位表示该域本次取到新鲜数据(getter 返回 `PX4LITE_OK`)；stale 位表示已收到但过期
 * (返回 `PX4LITE_STALE`)。两位都为 0 表示从未收到(`PX4LITE_NOT_READY`)。
 */
typedef struct {
  App_NavigationSnapshot_t navigation;  /**< 导航/姿态(模式解析后)。 */
  App_EnvironmentSnapshot_t environment; /**< 环境/电源(模式解析后)。 */
  App_SystemSnapshot_t system;          /**< 系统/模块状态(模式解析后)。 */
  App_AlarmSnapshot_t alarm;            /**< 告警(模式解析后)。 */
  App_MotorSnapshot_t motor;            /**< 电机只读(模式解析后)。 */
  App_DateTimeSnapshot_t date_time;     /**< 本机日期时间，固定使用本机统一时间快照。 */
  App_DisplayLogSnapshot_t message_log; /**< 消息日志(模式解析后)。 */
  App_DisplayLinkStatus_t link;         /**< 本机链路诊断(永远本机来源)。 */
  Px4Lite_RemoteMode_t mode;            /**< 当前显示数据源模式。 */
  uint32_t ready_mask;                  /**< 各域新鲜位，位序见 `App_DisplayDomain_t`。 */
  uint32_t stale_mask;                  /**< 各域过期位，位序见 `App_DisplayDomain_t`。 */
} App_DisplayModel_t;

/**
 * @brief 取当前应显示的导航/姿态(按模式自动选源)。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 * @param[in] now_ms 当前系统毫秒时间，用于新鲜度判断。
 *
 * @return 取数结果。
 * @retval PX4LITE_OK 数据新鲜可用。
 * @retval PX4LITE_STALE 已收到但过期，`out` 保留最后值，UI 应标 stale。
 * @retval PX4LITE_NOT_READY 从未收到，UI 应显示无效/清零。
 * @retval PX4LITE_INVALID_PARAM `out` 为空。
 */
Px4Lite_Result_t App_GetDisplayNavigation(App_NavigationSnapshot_t *out, uint32_t now_ms);

/**
 * @brief 取当前应显示的环境/电源(按模式自动选源)。语义同 `App_GetDisplayNavigation`。
 */
Px4Lite_Result_t App_GetDisplayEnvironment(App_EnvironmentSnapshot_t *out, uint32_t now_ms);

/**
 * @brief 取当前应显示的系统/模块状态(按模式自动选源)。语义同 `App_GetDisplayNavigation`。
 */
Px4Lite_Result_t App_GetDisplaySystem(App_SystemSnapshot_t *out, uint32_t now_ms);

/**
 * @brief 取当前应显示的告警(按模式自动选源)。
 *
 * @note 阶段 A：REMOTE 仅能返回告警摘要(最高项)，完整告警表依赖第二件事(doc 18 同步)。
 */
Px4Lite_Result_t App_GetDisplayAlarm(App_AlarmSnapshot_t *out, uint32_t now_ms);

/**
 * @brief 取当前应显示的电机只读快照(按模式自动选源)。语义同 `App_GetDisplayNavigation`。
 */
Px4Lite_Result_t App_GetDisplayMotor(App_MotorSnapshot_t *out, uint32_t now_ms);

/**
 * @brief 取本机日期时间，远端/本地模式切换不改变时间源。
 *
 * @note 顶栏时间属于本机界面状态，始终通过 `App_CopyDateTime()` 读取本机统一时间快照。
 */
Px4Lite_Result_t App_GetDisplayDateTime(App_DateTimeSnapshot_t *out, uint32_t now_ms);

/**
 * @brief 取当前应显示的消息日志(按模式自动选源)。
 *
 * @note 阶段 A：日志尚未上提到业务层，本函数返回 `PX4LITE_NOT_READY`，老显示暂沿用其内部
 *       日志路径；阶段 B 抽取结构化日志后，本函数在两种模式下返回对应日志。
 */
Px4Lite_Result_t App_GetDisplayMessageLog(App_DisplayLogSnapshot_t *out, uint32_t now_ms);

/**
 * @brief 取本机链路诊断(白名单域，REMOTE 下仍取本机)。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 * @param[in] now_ms 当前系统毫秒时间。
 */
void App_GetDisplayLinkStatus(App_DisplayLinkStatus_t *out, uint32_t now_ms);

/**
 * @brief 一次性构建完整显示视图模型(便利封装，内部逐个调用上面各 getter)。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @note 无动态分配，按 `biz_display` 周期调用一次；当前显示模式见 `out->mode`
 *       (亦可单独调用 `app_data_api.h` 的 `App_GetRemoteDisplayMode()`)。
 */
void App_BuildDisplayModel(App_DisplayModel_t *out, uint32_t now_ms);

#endif /* APP_DISPLAY_MODEL_H */
