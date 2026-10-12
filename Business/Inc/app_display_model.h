/**
 * @file app_display_model.h
 * @brief Business 层本机显示快照契约。
 * @details 通过 app_data_api.h 读取本机测量、控制、告警和事件；主页面没有全局远端模式。
 *          通信诊断使用显式对端接口，不能改变本机页的数据来源或生产收发设置。
 *          不包含 Display、LVGL、BSP 或 Sensor 头文件，不执行硬件 I/O。
 */
#ifndef APP_DISPLAY_MODEL_H
#define APP_DISPLAY_MODEL_H

#include <stdint.h>
#include "app_data_api.h"

#define APP_DISPLAY_LOG_CAP PX4LITE_LOCAL_LOG_CAP /**< 本机历史容量；与可见行数独立。 */

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
 * @brief 本机链路诊断。
 *
 * @details
 * 来源为 `App_GetCommStats` 与本机 LoRa 模块状态；选择通信对端不改变数据来源。
 */
typedef struct {
  uint32_t tx_frame_count; /**< 本机 LoRa 发送完成帧数。 */
  uint32_t rx_frame_count; /**< 本机 LoRa 接收帧数。 */
  uint16_t loss_permille;  /**< 接收链路丢包率，单位 ‰(0~1000)，靠 MAVLink seq 跳变估算。 */
  uint8_t lora_state;      /**< 本机 LoRa 模块公开状态(见 `Px4Lite_State_t`)，用于状态灯。 */
  uint8_t reserved;        /**< 保留字段，保持结构体对齐。 */
} App_DisplayLinkStatus_t;

/** @brief 本机结构化事件；复用 Framework 只读记录类型，避免大栈上的重复转换。 */
typedef Px4Lite_LogEntry_t App_DisplayLogEntry_t;

/**
 * @brief 显示消息日志快照。
 *
 * @details
 * 按时间顺序保存本机最近 `count` 条事件，最多 `APP_DISPLAY_LOG_CAP` 条。
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
 * 各域均来自本机。`ready_mask`/`stale_mask` 以 `App_DisplayDomain_t` 为位序：
 * ready 位表示该域本次取到新鲜数据(getter 返回 `PX4LITE_OK`)；stale 位表示已收到但过期
 * (返回 `PX4LITE_STALE`)。两位都为 0 表示从未收到(`PX4LITE_NOT_READY`)。
 */
typedef struct {
  App_NavigationSnapshot_t navigation;  /**< 导航/姿态(本机)。 */
  App_EnvironmentSnapshot_t environment; /**< 环境/电源(本机)。 */
  App_SystemSnapshot_t system;          /**< 系统/模块状态(本机)。 */
  App_AlarmSnapshot_t alarm;            /**< 告警(本机)。 */
  App_MotorSnapshot_t motor;            /**< 电机只读(本机)。 */
  App_DateTimeSnapshot_t date_time;     /**< 本机日期时间，固定使用本机统一时间快照。 */
  App_DisplayLogSnapshot_t message_log; /**< 消息日志(本机)。 */
  App_DisplayLinkStatus_t link;         /**< 本机链路诊断(永远本机来源)。 */
  Px4Lite_RemoteMode_t mode;            /**< 兼容字段，固定 LOCAL。 */
  uint32_t ready_mask;                  /**< 各域新鲜位，位序见 `App_DisplayDomain_t`。 */
  uint32_t stale_mask;                  /**< 各域过期位，位序见 `App_DisplayDomain_t`。 */
} App_DisplayModel_t;

/**
 * @brief 取本机导航/姿态，应用本机显示参考。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 * @param[in] now_ms 当前系统毫秒时间，用于新鲜度判断。
 *
 * @return 取数结果。
 * @retval PX4LITE_OK 数据新鲜可用。
 * @retval PX4LITE_STALE 数据过期；消费者必须按各域有效位使过期字段失效。
 * @retval PX4LITE_NOT_READY 从未收到，UI 应显示无效/清零。
 * @retval PX4LITE_INVALID_PARAM `out` 为空。
 */
Px4Lite_Result_t App_GetDisplayNavigation(App_NavigationSnapshot_t *out, uint32_t now_ms);

/**
 * @brief 取当前应显示的环境/电源(固定本机)。语义同 `App_GetDisplayNavigation`。
 */
Px4Lite_Result_t App_GetDisplayEnvironment(App_EnvironmentSnapshot_t *out, uint32_t now_ms);

/**
 * @brief 取当前应显示的系统/模块状态(固定本机)。语义同 `App_GetDisplayNavigation`。
 */
Px4Lite_Result_t App_GetDisplaySystem(App_SystemSnapshot_t *out, uint32_t now_ms);

/**
 * @brief 取当前应显示的告警(固定本机)。
 */
Px4Lite_Result_t App_GetDisplayAlarm(App_AlarmSnapshot_t *out, uint32_t now_ms);

/**
 * @brief 取当前应显示的电机只读快照(固定本机)。语义同 `App_GetDisplayNavigation`。
 */
Px4Lite_Result_t App_GetDisplayMotor(App_MotorSnapshot_t *out, uint32_t now_ms);

/**
 * @brief 取本机日期时间，通信对端选择不改变时间源。
 *
 * @note 顶栏时间属于本机界面状态，始终通过 `App_CopyDateTime()` 读取本机统一时间快照。
 */
Px4Lite_Result_t App_GetDisplayDateTime(App_DateTimeSnapshot_t *out, uint32_t now_ms);

/**
 * @brief 取当前应显示的消息日志(固定本机)。
 *
 * @note 只复制业务观察器与状态所有者发布的事件；读取动作不生成或清除日志。
 */
Px4Lite_Result_t App_GetDisplayMessageLog(App_DisplayLogSnapshot_t *out, uint32_t now_ms);

/**
 * @brief 取本机链路诊断。
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
 * @note 无动态分配，完整快照较大，应由调用者提供静态缓冲；`out->mode` 固定 LOCAL。
 */
void App_BuildDisplayModel(App_DisplayModel_t *out, uint32_t now_ms);

#endif /* APP_DISPLAY_MODEL_H */
