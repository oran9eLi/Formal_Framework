/**
 * @file px4lite_platform.h
 * @brief Declare MCU platform adapters and task heartbeat services.
 */

#ifndef PX4LITE_PLATFORM_H
#define PX4LITE_PLATFORM_H

#include "px4lite_types.h"

typedef enum
{
    PX4LITE_HEARTBEAT_SENSOR = 0,
    PX4LITE_HEARTBEAT_ESTIMATOR,
    PX4LITE_HEARTBEAT_HEALTH,
    PX4LITE_HEARTBEAT_SYSTEM,
    PX4LITE_HEARTBEAT_BUSINESS,
    PX4LITE_HEARTBEAT_DISPLAY,
    PX4LITE_HEARTBEAT_COMM,
    PX4LITE_HEARTBEAT_COUNT
} Px4Lite_HeartbeatId_t;

/* Platform adapters are the only framework files allowed to call BSP. */
/**
 * @brief Return the monotonic platform time in milliseconds.
 */
uint32_t Px4Lite_PlatformGetMs(void);
/**
 * @brief Return a coherent microsecond timestamp from HAL tick and TIM6.
 */
uint32_t Px4Lite_PlatformGetUs(void);
/**
 * @brief Record the latest successful execution time of one required task.
 */
void Px4Lite_PlatformHeartbeat(Px4Lite_HeartbeatId_t id,
                               uint32_t now_ms);
/**
 * @brief Check whether every required task heartbeat is present and fresh.
 */
uint8_t Px4Lite_PlatformHeartbeatsHealthy(uint32_t now_ms);
/**
 * @brief Refresh the hardware watchdog only when all required heartbeats are healthy.
 */
void Px4Lite_PlatformWatchdogFeed(uint32_t now_ms);

/**
 * @brief Reset platform heartbeat state and initialize adapter-owned services.
 */
Px4Lite_Result_t Px4Lite_PlatformInit(void);
/**
 * @brief Initialize the GNSS BSP and typed sensor driver through the platform adapter.
 */
Px4Lite_Result_t Px4Lite_GnssInit(void);
/**
 * @brief Convert one newly received GNSS driver snapshot into framework measurement format.
 */
Px4Lite_Result_t Px4Lite_GnssRead(Px4Lite_SensorGnss_t *measurement);
/**
 * @brief Initialize the MPU6050 IMU driver.
 */
Px4Lite_Result_t Px4Lite_ImuInit(void);
/**
 * @brief Convert one MPU6050 snapshot into framework IMU format.
 */
Px4Lite_Result_t Px4Lite_ImuRead(Px4Lite_SensorImu_t *measurement);
/**
 * @brief Initialize the BME280 barometer/environment driver.
 */
Px4Lite_Result_t Px4Lite_BaroInit(void);
/**
 * @brief Convert one BME280 driver snapshot into framework barometer format.
 */
Px4Lite_Result_t Px4Lite_BaroRead(Px4Lite_SensorBaro_t *measurement);
/**
 * @brief Initialize the power-sense ADC driver.
 */
Px4Lite_Result_t Px4Lite_BatteryInit(void);
/**
 * @brief Convert one power driver snapshot into framework battery format.
 */
Px4Lite_Result_t Px4Lite_BatteryRead(
    Px4Lite_BatteryStatus_t *measurement);

/* Add future MCU or Linux adapters here without exposing HAL types. */

Px4Lite_Result_t Px4Lite_LoRaInit(void);
Px4Lite_Result_t Px4Lite_LoRaService(uint32_t now_ms);
/* Async-copy: PX4LITE_OK means the frame was copied and staged by the driver
   (caller may reuse its buffer), not that it has been transmitted. Air-side
   completion is tracked via Px4Lite_LoRaGetDebugInfo (tx_frame_count/last_tx_ms).
   Returns PX4LITE_BUSY while a previously staged frame is still in flight. */
Px4Lite_Result_t Px4Lite_LoRaSend(const uint8_t *data, uint16_t len);
Px4Lite_State_t Px4Lite_LoRaGetState(uint32_t now_ms);
void Px4Lite_LoRaGetDebugInfo(Px4Lite_CommDebugInfo_t *out);

/* Cheap "request re-init" hooks used by the recovery monitor. Each only sets a
   flag; the owning driver performs the actual re-init from its own Service. */
void Px4Lite_GnssRequestReinit(void);
void Px4Lite_ImuRequestReinit(void);
void Px4Lite_BaroRequestReinit(void);
void Px4Lite_BatteryRequestReinit(void);
void Px4Lite_LoRaRequestReinit(void);

#endif
