/**
 * @file px4lite_modules.h
 * @brief Declare sensor, estimator, health, and module status services.
 */

#ifndef PX4LITE_MODULES_H
#define PX4LITE_MODULES_H

#include "px4lite_types.h"

/**
 * @brief Reset framework module states, counters, and startup time.
 */
Px4Lite_Result_t Px4Lite_ModulesInit(void);
/**
 * @brief Registry init callbacks: bring up one sensor device and publish its state.
 */
Px4Lite_Result_t Px4Lite_GnssModuleInit(void);
Px4Lite_Result_t Px4Lite_ImuModuleInit(void);
Px4Lite_Result_t Px4Lite_BaroModuleInit(void);
Px4Lite_Result_t Px4Lite_BatteryModuleInit(void);
Px4Lite_Result_t Px4Lite_AlarmModuleInit(void);

/**
 * @brief Registry recover callbacks: cheaply request a re-init (no bus I/O here).
 */
Px4Lite_Result_t Px4Lite_GnssRecover(void);
Px4Lite_Result_t Px4Lite_ImuRecover(void);
Px4Lite_Result_t Px4Lite_BaroRecover(void);
Px4Lite_Result_t Px4Lite_BatteryRecover(void);
Px4Lite_Result_t Px4Lite_LoraRecover(void);
/**
 * @brief Run one non-blocking sensor acquisition and publication cycle.
 */
void Px4Lite_SensorWorkRun(uint32_t now_ms);

/**
 * @brief Initialize the estimator module state.
 */
Px4Lite_Result_t Px4Lite_EstimatorInit(void);
/**
 * @brief Convert each fresh GNSS measurement into a navigation snapshot.
 */
void Px4Lite_EstimatorRun(uint32_t now_ms);

/**
 * @brief Initialize the LoRa/comm module and MAVLink transmitter.
 */
Px4Lite_Result_t Px4Lite_CommModulesInit(void);
/**
 * @brief Run one non-blocking comm (LoRa RX parse + MAVLink TX) cycle.
 */
void Px4Lite_CommWorkRun(uint32_t now_ms);
/**
 * @brief Copy aggregated LoRa and MAVLink transmit diagnostics.
 */
void Px4Lite_GetCommDebugInfo(Px4Lite_CommDebugInfo_t *out);

/**
 * @brief Initialize health monitoring services.
 */
Px4Lite_Result_t Px4Lite_HealthInit(void);
/**
 * @brief Evaluate module timeouts and publish one coherent system health snapshot.
 */
void Px4Lite_HealthRun(uint32_t now_ms);

/**
 * @brief Copy one module status under framework synchronization.
 */
Px4Lite_Result_t Px4Lite_GetModuleStatus(
    Px4Lite_ModuleId_t module_id,
    Px4Lite_ModuleStatus_t *status);
/**
 * @brief Copy a version-consistent array of all module statuses.
 */
Px4Lite_Result_t Px4Lite_CopyModuleStatuses(
    Px4Lite_ModuleStatus_t *status,
    uint16_t count,
    uint32_t *version);
/**
 * @brief Return the current module status generation number.
 */
uint32_t Px4Lite_GetStatusVersion(void);

/**
 * @brief Allow an external service adapter to update its framework module state.
 */
void Px4Lite_SetExternalModuleState(
    Px4Lite_ModuleId_t module_id,
    Px4Lite_State_t state,
    uint16_t fault_code,
    uint32_t now_ms);

#endif
