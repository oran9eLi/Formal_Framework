/**
 * @file px4lite_topics.c
 * @brief 实现一致的 latest-value topic、IMU FIFO 和事件队列。
 */

#include "px4lite_topics.h"
#include "px4lite_config.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <string.h>

#define DECLARE_LATEST_SLOT(type, name) \
    static type s_##name;               \
    static uint8_t s_##name##_ready

DECLARE_LATEST_SLOT(Px4Lite_SensorGnss_t, gnss);
DECLARE_LATEST_SLOT(Px4Lite_SensorBaro_t, baro);
DECLARE_LATEST_SLOT(Px4Lite_BatteryStatus_t, battery);
DECLARE_LATEST_SLOT(Px4Lite_VehicleNavigation_t, navigation);
DECLARE_LATEST_SLOT(Px4Lite_SystemHealth_t, health);

typedef struct
{
    Px4Lite_SensorImu_t samples[PX4LITE_IMU_FIFO_CAPACITY]; /**< IMU 样本环形缓冲区。 */
    uint16_t head;                                          /**< 下一次写入位置。 */
    uint16_t tail;                                          /**< 下一次读取位置。 */
    uint16_t count;                                         /**< 当前缓存样本数。 */
    Px4Lite_FifoStats_t stats;                              /**< FIFO 运行统计。 */
} Px4Lite_ImuFifo_t;

static Px4Lite_ImuFifo_t s_imu_fifo;
static QueueHandle_t s_alarm_queue;
static QueueHandle_t s_command_queue;
static QueueHandle_t s_command_ack_queue;

#define DEFINE_LATEST_ACCESSORS(type, suffix, name)                  \
    void Px4Lite_Publish##suffix(const type *data)                   \
    {                                                                \
        if (data == 0)                                                \
        {                                                            \
            return;                                                   \
        }                                                            \
        taskENTER_CRITICAL();                                         \
        s_##name = *data;                                             \
        s_##name##_ready = 1U;                                        \
        taskEXIT_CRITICAL();                                          \
    }                                                                \
    Px4Lite_Result_t Px4Lite_Copy##suffix(type *data)                \
    {                                                                \
        uint8_t ready;                                                \
        if (data == 0)                                                \
        {                                                            \
            return PX4LITE_INVALID_PARAM;                             \
        }                                                            \
        taskENTER_CRITICAL();                                         \
        ready = s_##name##_ready;                                     \
        if (ready != 0U)                                              \
        {                                                            \
            *data = s_##name;                                         \
        }                                                            \
        taskEXIT_CRITICAL();                                          \
        return (ready != 0U) ? PX4LITE_OK : PX4LITE_NOT_READY;        \
    }

DEFINE_LATEST_ACCESSORS(Px4Lite_SensorGnss_t, Gnss, gnss)
DEFINE_LATEST_ACCESSORS(Px4Lite_SensorBaro_t, Baro, baro)
DEFINE_LATEST_ACCESSORS(Px4Lite_BatteryStatus_t, Battery, battery)
DEFINE_LATEST_ACCESSORS(
    Px4Lite_VehicleNavigation_t,
    Navigation,
    navigation)
DEFINE_LATEST_ACCESSORS(Px4Lite_SystemHealth_t, Health, health)

Px4Lite_Result_t Px4Lite_TopicsInit(void)
{
    memset(&s_imu_fifo, 0, sizeof(s_imu_fifo));
    s_gnss_ready = 0U;
    s_baro_ready = 0U;
    s_battery_ready = 0U;
    s_navigation_ready = 0U;
    s_health_ready = 0U;
    s_alarm_queue = 0;
    s_command_queue = 0;
    s_command_ack_queue = 0;

#if PX4LITE_ENABLE_ALARM
    s_alarm_queue = xQueueCreate(
        PX4LITE_ALARM_QUEUE_LENGTH,
        sizeof(Px4Lite_AlarmEvent_t));
    if (s_alarm_queue == 0)
    {
        return PX4LITE_IO_ERROR;
    }
#endif

#if PX4LITE_ENABLE_COMMAND
    s_command_queue = xQueueCreate(
        PX4LITE_COMMAND_QUEUE_LENGTH,
        sizeof(Px4Lite_Command_t));
    s_command_ack_queue = xQueueCreate(
        PX4LITE_COMMAND_QUEUE_LENGTH,
        sizeof(Px4Lite_CommandAck_t));
    if ((s_command_queue == 0) ||
        (s_command_ack_queue == 0))
    {
        return PX4LITE_IO_ERROR;
    }
#endif

    return PX4LITE_OK;
}

/**
 * @brief 将一帧 IMU 样本压入有界 FIFO，溢出时丢弃最旧样本。
 */
Px4Lite_Result_t Px4Lite_PushImu(
    const Px4Lite_SensorImu_t *data)
{
    Px4Lite_Result_t result = PX4LITE_OK;

    if (data == 0)
    {
        return PX4LITE_INVALID_PARAM;
    }

    taskENTER_CRITICAL();

    if (s_imu_fifo.count >= PX4LITE_IMU_FIFO_CAPACITY)
    {
        s_imu_fifo.tail =
            (uint16_t)((s_imu_fifo.tail + 1U) %
                       PX4LITE_IMU_FIFO_CAPACITY);
        s_imu_fifo.count--;
        s_imu_fifo.stats.overflow_count++;
        result = PX4LITE_OVERFLOW;
    }

    s_imu_fifo.samples[s_imu_fifo.head] = *data;
    s_imu_fifo.head =
        (uint16_t)((s_imu_fifo.head + 1U) %
                   PX4LITE_IMU_FIFO_CAPACITY);
    s_imu_fifo.count++;
    s_imu_fifo.stats.count = s_imu_fifo.count;
    s_imu_fifo.stats.push_count++;

    if (s_imu_fifo.count > s_imu_fifo.stats.peak)
    {
        s_imu_fifo.stats.peak = s_imu_fifo.count;
    }

    taskEXIT_CRITICAL();
    return result;
}

/**
 * @brief 从 FIFO 取出最旧的一帧 IMU 样本。
 */
Px4Lite_Result_t Px4Lite_PopImu(Px4Lite_SensorImu_t *data)
{
    if (data == 0)
    {
        return PX4LITE_INVALID_PARAM;
    }

    taskENTER_CRITICAL();

    if (s_imu_fifo.count == 0U)
    {
        taskEXIT_CRITICAL();
        return PX4LITE_NOT_READY;
    }

    *data = s_imu_fifo.samples[s_imu_fifo.tail];
    s_imu_fifo.tail =
        (uint16_t)((s_imu_fifo.tail + 1U) %
                   PX4LITE_IMU_FIFO_CAPACITY);
    s_imu_fifo.count--;
    s_imu_fifo.stats.count = s_imu_fifo.count;
    s_imu_fifo.stats.pop_count++;

    taskEXIT_CRITICAL();
    return PX4LITE_OK;
}

/**
 * @brief 复制当前 IMU FIFO 使用量和溢出统计。
 */
void Px4Lite_GetImuStats(Px4Lite_FifoStats_t *stats)
{
    if (stats == 0)
    {
        return;
    }

    taskENTER_CRITICAL();
    *stats = s_imu_fifo.stats;
    taskEXIT_CRITICAL();
}

/**
 * @brief 向有界告警队列发布一条告警事件。
 */
Px4Lite_Result_t Px4Lite_PublishAlarm(
    const Px4Lite_AlarmEvent_t *event)
{
    if (event == 0)
    {
        return PX4LITE_INVALID_PARAM;
    }
    if (s_alarm_queue == 0)
    {
        return PX4LITE_NOT_READY;
    }

    return (xQueueSend(s_alarm_queue, event, 0U) == pdPASS)
               ? PX4LITE_OK
               : PX4LITE_OVERFLOW;
}

/**
 * @brief 非阻塞取出一条待处理告警事件。
 */
Px4Lite_Result_t Px4Lite_TakeAlarm(Px4Lite_AlarmEvent_t *event)
{
    if (event == 0)
    {
        return PX4LITE_INVALID_PARAM;
    }
    if (s_alarm_queue == 0)
    {
        return PX4LITE_NOT_READY;
    }

    return (xQueueReceive(s_alarm_queue, event, 0U) == pdPASS)
               ? PX4LITE_OK
               : PX4LITE_NOT_READY;
}

/**
 * @brief 向有界命令队列发布一条应用命令。
 */
Px4Lite_Result_t Px4Lite_PublishCommand(
    const Px4Lite_Command_t *command)
{
    if (command == 0)
    {
        return PX4LITE_INVALID_PARAM;
    }
    if (s_command_queue == 0)
    {
        return PX4LITE_NOT_READY;
    }

    return (xQueueSend(s_command_queue, command, 0U) == pdPASS)
               ? PX4LITE_OK
               : PX4LITE_OVERFLOW;
}

/**
 * @brief 非阻塞取出一条待处理应用命令。
 */
Px4Lite_Result_t Px4Lite_TakeCommand(
    Px4Lite_Command_t *command)
{
    if (command == 0)
    {
        return PX4LITE_INVALID_PARAM;
    }
    if (s_command_queue == 0)
    {
        return PX4LITE_NOT_READY;
    }

    return (xQueueReceive(s_command_queue, command, 0U) == pdPASS)
               ? PX4LITE_OK
               : PX4LITE_NOT_READY;
}

/**
 * @brief 向有界应答队列发布一条命令应答。
 */
Px4Lite_Result_t Px4Lite_PublishCommandAck(
    const Px4Lite_CommandAck_t *ack)
{
    if (ack == 0)
    {
        return PX4LITE_INVALID_PARAM;
    }
    if (s_command_ack_queue == 0)
    {
        return PX4LITE_NOT_READY;
    }

    return (xQueueSend(s_command_ack_queue, ack, 0U) == pdPASS)
               ? PX4LITE_OK
               : PX4LITE_OVERFLOW;
}

/**
 * @brief 非阻塞取出一条待处理命令应答。
 */
Px4Lite_Result_t Px4Lite_TakeCommandAck(
    Px4Lite_CommandAck_t *ack)
{
    if (ack == 0)
    {
        return PX4LITE_INVALID_PARAM;
    }
    if (s_command_ack_queue == 0)
    {
        return PX4LITE_NOT_READY;
    }

    return (xQueueReceive(s_command_ack_queue, ack, 0U) == pdPASS)
               ? PX4LITE_OK
               : PX4LITE_NOT_READY;
}

