/**
 * @file px4lite_config.h
 * @brief 配置 Framework 模块开关、周期、超时、队列和任务栈。
 *
 * @details
 * 本文件只保存 Framework 层配置。BSP 引脚/外设配置放在 `bsp_config.h`，
 * Business 任务配置放在 `business_template_config.h`。所有 `_MS` 后缀宏均为真实
 * 毫秒值，不能填入 RTOS tick。
 */

#ifndef PX4LITE_CONFIG_H
#define PX4LITE_CONFIG_H

#define PX4LITE_ENABLE_GNSS                 1U
#define PX4LITE_ENABLE_IMU                  1U
#define PX4LITE_ENABLE_BARO                 1U
#define PX4LITE_ENABLE_BATTERY              1U
#define PX4LITE_ENABLE_LORA                 1U
#define PX4LITE_ENABLE_5G                   0U
#define PX4LITE_ENABLE_STORAGE              1U
#define PX4LITE_ENABLE_REMOTE_ID            0U
#define PX4LITE_ENABLE_DISPLAY              1U
#define PX4LITE_ENABLE_CONTROL              0U
#define PX4LITE_ENABLE_ALARM                1U
#define PX4LITE_ENABLE_COMMAND              0U
#define PX4LITE_ENABLE_LOG_POOL             0U

#define PX4LITE_SENSOR_WORK_PERIOD_MS      10U
#define PX4LITE_ESTIMATOR_PERIOD_MS        20U
#define PX4LITE_HEALTH_PERIOD_MS          100U
#define PX4LITE_COMM_PERIOD_MS             10U
#define PX4LITE_TASK_HEARTBEAT_TIMEOUT_MS 500U
#define PX4LITE_IMU_WORK_PERIOD_MS         10U
#define PX4LITE_BARO_WORK_PERIOD_MS      1000U
#define PX4LITE_BATTERY_WORK_PERIOD_MS   1000U
#define PX4LITE_STORAGE_PERIOD_MS          50U

#define PX4LITE_PRIORITY_SENSOR       (tskIDLE_PRIORITY + 4U)
#define PX4LITE_PRIORITY_ESTIMATOR    (tskIDLE_PRIORITY + 3U)
#define PX4LITE_PRIORITY_HEALTH       (tskIDLE_PRIORITY + 2U)
#define PX4LITE_PRIORITY_COMM         (tskIDLE_PRIORITY + 2U)
/* Storage 会执行阻塞 SD I/O，优先级必须低于 biz_display（idle+1）。 */
#define PX4LITE_PRIORITY_STORAGE      (tskIDLE_PRIORITY)

/*
 * 硬件 watchdog 刷新入口实现前保持关闭；若未实现 BSP_WatchdogRefresh() 就打开，
 * 链接错误是预期保护。
 */
#define PX4LITE_ENABLE_HARDWARE_WATCHDOG   0U

#define PX4LITE_GNSS_STARTUP_GRACE_MS     5000U
#define PX4LITE_GNSS_OFFLINE_MS           2000U
#define PX4LITE_GNSS_MAX_AGE_MS           1500U
#define PX4LITE_IMU_STARTUP_GRACE_MS      2000U
#define PX4LITE_IMU_OFFLINE_MS             500U
#define PX4LITE_IMU_MAX_AGE_MS             200U
#define PX4LITE_IMU_STABLE_VALID_MIN         3U
#define PX4LITE_BARO_STARTUP_GRACE_MS     5000U
#define PX4LITE_BARO_OFFLINE_MS           3000U
#define PX4LITE_BARO_MAX_AGE_MS           2500U
#define PX4LITE_BATTERY_STARTUP_GRACE_MS  5000U
#define PX4LITE_BATTERY_OFFLINE_MS        3000U
#define PX4LITE_BATTERY_MAX_AGE_MS        2500U
#define PX4LITE_DISPLAY_STARTUP_GRACE_MS  5000U
#define PX4LITE_DISPLAY_OFFLINE_MS        3000U

/*
 * GNSS 专用恢复路径已经挂到 HealthRun，但在 BSP 恢复路径完成硬件故障注入测试前
 * 保持关闭。
 */
#define PX4LITE_GNSS_RECOVERY_ENABLE          0U
#define PX4LITE_GNSS_ERROR_THRESHOLD          3U
#define PX4LITE_GNSS_RECOVERY_DELAY_MS     1000U
#define PX4LITE_GNSS_RECOVERY_MAX_RETRY       5U

/*
 * 通用注册表恢复：Health 只监督并调用轻量 recover 回调，真正 re-init 由模块所属
 * Service 执行。recover 回调必须是“请求重初始化”形式，不能在 Health 任务中执行
 * 阻塞总线 I/O。重试限速但不设总次数上限，以支持长时间拔插后的热恢复。
 */
#define PX4LITE_RECOVERY_ENABLE               1U
#define PX4LITE_RECOVERY_ERROR_THRESHOLD      5U
#define PX4LITE_RECOVERY_DELAY_MS          2000U

#define PX4LITE_IMU_FIFO_CAPACITY           32U
#define PX4LITE_ALARM_QUEUE_LENGTH          16U
#define PX4LITE_COMMAND_QUEUE_LENGTH         8U

#define PX4LITE_STACK_SENSOR               512U
#define PX4LITE_STACK_ESTIMATOR            512U
#define PX4LITE_STACK_HEALTH               768U
#define PX4LITE_STACK_COMM                 384U
#define PX4LITE_STACK_STORAGE              768U

#define PX4LITE_LORA_OFFLINE_MS          3000U
#define PX4LITE_LORA_STARTUP_GRACE_MS    3000U
#define PX4LITE_LORA_RECOVERY_ENABLE        0U
#define PX4LITE_MAVLINK_SYSTEM_ID            1U
#define PX4LITE_MAVLINK_COMPONENT_ID       191U

/*
 * MAVLink 遥测消息选择。CommTask 是唯一发送拥有者，消息必须逐字段编码，不能直接
 * 发送 C 结构体内存。
 */
#define PX4LITE_MAVLINK_ENABLE_HEARTBEAT       1U
#define PX4LITE_MAVLINK_ENABLE_GPS_RAW         1U
#define PX4LITE_MAVLINK_ENABLE_GNSS_DETAIL     1U
#define PX4LITE_MAVLINK_ENABLE_ATTITUDE        1U
#define PX4LITE_MAVLINK_ENABLE_GLOBAL_POSITION 1U
#define PX4LITE_MAVLINK_ENABLE_SYS_STATUS      1U
#define PX4LITE_MAVLINK_ENABLE_BATTERY_STATUS  1U
#define PX4LITE_MAVLINK_ENABLE_SCALED_PRESSURE 1U
#define PX4LITE_MAVLINK_ENABLE_STATUSTEXT      1U

#define PX4LITE_MAVLINK_HEARTBEAT_PERIOD_MS 1000U
#define PX4LITE_MAVLINK_GPS_RAW_PERIOD_MS    1000U
#define PX4LITE_MAVLINK_GNSS_DETAIL_PERIOD_MS 1000U
#define PX4LITE_MAVLINK_ATTITUDE_PERIOD_MS     50U
#define PX4LITE_MAVLINK_POSITION_PERIOD_MS    500U
#define PX4LITE_MAVLINK_SYS_STATUS_PERIOD_MS 1000U
#define PX4LITE_MAVLINK_BATTERY_PERIOD_MS    1000U
#define PX4LITE_MAVLINK_PRESSURE_PERIOD_MS   1000U
#define PX4LITE_MAVLINK_STATUSTEXT_PERIOD_MS 2000U
#define PX4LITE_MAVLINK_RETRY_PERIOD_MS        50U

#endif
