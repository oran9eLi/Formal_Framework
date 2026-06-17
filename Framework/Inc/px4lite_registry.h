/**
 * @file px4lite_registry.h
 * @brief Declare module registration and lifecycle control.
 */

#ifndef PX4LITE_REGISTRY_H
#define PX4LITE_REGISTRY_H

#include "px4lite_types.h"

typedef enum
{
    PX4LITE_LIFECYCLE_EMPTY = 0,
    PX4LITE_LIFECYCLE_REGISTERED,
    PX4LITE_LIFECYCLE_INITIALIZING,
    PX4LITE_LIFECYCLE_SELF_CHECK,
    PX4LITE_LIFECYCLE_STARTING,
    PX4LITE_LIFECYCLE_RUNNING,
    PX4LITE_LIFECYCLE_RECOVERING,
    PX4LITE_LIFECYCLE_STOPPED,
    PX4LITE_LIFECYCLE_FAILED,
    PX4LITE_LIFECYCLE_DISABLED
} Px4Lite_LifecycleState_t;

typedef Px4Lite_Result_t (*Px4Lite_LifecycleFn_t)(void);

typedef struct
{
    Px4Lite_ModuleId_t module_id;
    const char *name;
    uint8_t enabled;
    uint8_t required;
    Px4Lite_LifecycleFn_t init;
    Px4Lite_LifecycleFn_t self_check;
    Px4Lite_LifecycleFn_t start;
    Px4Lite_LifecycleFn_t stop;
    Px4Lite_LifecycleFn_t recover;
} Px4Lite_ModuleDescriptor_t;

typedef struct
{
    Px4Lite_LifecycleState_t state;
    Px4Lite_Result_t last_result;
    uint32_t state_since_ms;
    uint16_t start_count;
    uint16_t recovery_count;
} Px4Lite_ModuleRuntime_t;

/**
 * @brief Reset the module registry and lifecycle runtime state.
 */
Px4Lite_Result_t Px4Lite_RegistryInit(void);

/**
 * @brief Register one module descriptor before the scheduler starts.
 */
Px4Lite_Result_t Px4Lite_RegistryRegister(
    const Px4Lite_ModuleDescriptor_t *descriptor);

/**
 * @brief Run init, self-check, and start callbacks for one module.
 */
Px4Lite_Result_t Px4Lite_RegistryStart(
    Px4Lite_ModuleId_t module_id,
    uint32_t now_ms);

/**
 * @brief Run the registered stop callback for one module.
 */
Px4Lite_Result_t Px4Lite_RegistryStop(
    Px4Lite_ModuleId_t module_id,
    uint32_t now_ms);

/**
 * @brief Run the registered recovery callback and restart one module.
 */
Px4Lite_Result_t Px4Lite_RegistryRecover(
    Px4Lite_ModuleId_t module_id,
    uint32_t now_ms);

/**
 * @brief Copy one registered module descriptor and its lifecycle state.
 */
Px4Lite_Result_t Px4Lite_RegistryGet(
    Px4Lite_ModuleId_t module_id,
    Px4Lite_ModuleDescriptor_t *descriptor,
    Px4Lite_ModuleRuntime_t *runtime);

#endif
