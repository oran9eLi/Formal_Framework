/**
 * @file px4lite_registry.c
 * @brief Implement bounded module registration and lifecycle sequencing.
 */

#include "px4lite_registry.h"

#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

typedef struct
{
    Px4Lite_ModuleDescriptor_t descriptor;
    Px4Lite_ModuleRuntime_t runtime;
    uint8_t registered;
} Px4Lite_RegistrySlot_t;

static Px4Lite_RegistrySlot_t s_registry[PX4LITE_MODULE_COUNT];

/**
 * @brief Update one lifecycle runtime state and its transition timestamp.
 */
static void Px4Lite_RegistrySetState(
    Px4Lite_RegistrySlot_t *slot,
    Px4Lite_LifecycleState_t state,
    Px4Lite_Result_t result,
    uint32_t now_ms)
{
    slot->runtime.state = state;
    slot->runtime.last_result = result;
    slot->runtime.state_since_ms = now_ms;
}

/**
 * @brief Execute one optional lifecycle callback and normalize a missing callback.
 */
static Px4Lite_Result_t Px4Lite_RegistryCall(
    Px4Lite_LifecycleFn_t callback)
{
    return (callback != 0) ? callback() : PX4LITE_OK;
}

/**
 * @brief Reset the module registry and lifecycle runtime state.
 */
Px4Lite_Result_t Px4Lite_RegistryInit(void)
{
    memset(s_registry, 0, sizeof(s_registry));
    return PX4LITE_OK;
}

/**
 * @brief Register one module descriptor before the scheduler starts.
 */
Px4Lite_Result_t Px4Lite_RegistryRegister(
    const Px4Lite_ModuleDescriptor_t *descriptor)
{
    Px4Lite_RegistrySlot_t *slot;

    if ((descriptor == 0) ||
        ((uint32_t)descriptor->module_id >=
         (uint32_t)PX4LITE_MODULE_COUNT) ||
        (descriptor->name == 0))
    {
        return PX4LITE_INVALID_PARAM;
    }

    slot = &s_registry[descriptor->module_id];
    if (slot->registered != 0U)
    {
        return PX4LITE_BUSY;
    }

    slot->descriptor = *descriptor;
    slot->registered = 1U;
    slot->runtime.state =
        (descriptor->enabled != 0U)
            ? PX4LITE_LIFECYCLE_REGISTERED
            : PX4LITE_LIFECYCLE_DISABLED;
    slot->runtime.last_result = PX4LITE_OK;
    return PX4LITE_OK;
}

/**
 * @brief Run init, self-check, and start callbacks for one module.
 */
Px4Lite_Result_t Px4Lite_RegistryStart(
    Px4Lite_ModuleId_t module_id,
    uint32_t now_ms)
{
    Px4Lite_RegistrySlot_t *slot;
    Px4Lite_Result_t result;

    if ((uint32_t)module_id >= (uint32_t)PX4LITE_MODULE_COUNT)
    {
        return PX4LITE_INVALID_PARAM;
    }

    slot = &s_registry[module_id];
    if (slot->registered == 0U)
    {
        return PX4LITE_NOT_READY;
    }
    if (slot->descriptor.enabled == 0U)
    {
        Px4Lite_RegistrySetState(
            slot, PX4LITE_LIFECYCLE_DISABLED, PX4LITE_OK, now_ms);
        return PX4LITE_OK;
    }
    if (slot->runtime.state == PX4LITE_LIFECYCLE_RUNNING)
    {
        return PX4LITE_OK;
    }

    Px4Lite_RegistrySetState(
        slot, PX4LITE_LIFECYCLE_INITIALIZING, PX4LITE_OK, now_ms);
    result = Px4Lite_RegistryCall(slot->descriptor.init);
    if (result != PX4LITE_OK)
    {
        Px4Lite_RegistrySetState(
            slot, PX4LITE_LIFECYCLE_FAILED, result, now_ms);
        return result;
    }

    Px4Lite_RegistrySetState(
        slot, PX4LITE_LIFECYCLE_SELF_CHECK, PX4LITE_OK, now_ms);
    result = Px4Lite_RegistryCall(slot->descriptor.self_check);
    if (result != PX4LITE_OK)
    {
        Px4Lite_RegistrySetState(
            slot, PX4LITE_LIFECYCLE_FAILED, result, now_ms);
        return result;
    }

    Px4Lite_RegistrySetState(
        slot, PX4LITE_LIFECYCLE_STARTING, PX4LITE_OK, now_ms);
    result = Px4Lite_RegistryCall(slot->descriptor.start);
    if (result != PX4LITE_OK)
    {
        Px4Lite_RegistrySetState(
            slot, PX4LITE_LIFECYCLE_FAILED, result, now_ms);
        return result;
    }

    if (slot->runtime.start_count < 65535U)
    {
        slot->runtime.start_count++;
    }
    Px4Lite_RegistrySetState(
        slot, PX4LITE_LIFECYCLE_RUNNING, PX4LITE_OK, now_ms);
    return PX4LITE_OK;
}

/**
 * @brief Run the registered stop callback for one module.
 */
Px4Lite_Result_t Px4Lite_RegistryStop(
    Px4Lite_ModuleId_t module_id,
    uint32_t now_ms)
{
    Px4Lite_RegistrySlot_t *slot;
    Px4Lite_Result_t result;

    if ((uint32_t)module_id >= (uint32_t)PX4LITE_MODULE_COUNT)
    {
        return PX4LITE_INVALID_PARAM;
    }

    slot = &s_registry[module_id];
    if (slot->registered == 0U)
    {
        return PX4LITE_NOT_READY;
    }

    result = Px4Lite_RegistryCall(slot->descriptor.stop);
    Px4Lite_RegistrySetState(
        slot,
        (result == PX4LITE_OK)
            ? PX4LITE_LIFECYCLE_STOPPED
            : PX4LITE_LIFECYCLE_FAILED,
        result,
        now_ms);
    return result;
}

/**
 * @brief Run the registered recovery callback and restart one module.
 */
Px4Lite_Result_t Px4Lite_RegistryRecover(
    Px4Lite_ModuleId_t module_id,
    uint32_t now_ms)
{
    Px4Lite_RegistrySlot_t *slot;
    Px4Lite_Result_t result;

    if ((uint32_t)module_id >= (uint32_t)PX4LITE_MODULE_COUNT)
    {
        return PX4LITE_INVALID_PARAM;
    }

    slot = &s_registry[module_id];
    if (slot->registered == 0U)
    {
        return PX4LITE_NOT_READY;
    }

    Px4Lite_RegistrySetState(
        slot, PX4LITE_LIFECYCLE_RECOVERING, PX4LITE_OK, now_ms);
    result = Px4Lite_RegistryCall(slot->descriptor.recover);
    if (result != PX4LITE_OK)
    {
        Px4Lite_RegistrySetState(
            slot, PX4LITE_LIFECYCLE_FAILED, result, now_ms);
        return result;
    }
    if (slot->runtime.recovery_count < 65535U)
    {
        slot->runtime.recovery_count++;
    }

    slot->runtime.state = PX4LITE_LIFECYCLE_REGISTERED;
    return Px4Lite_RegistryStart(module_id, now_ms);
}

/**
 * @brief Copy one registered module descriptor and its lifecycle state.
 */
Px4Lite_Result_t Px4Lite_RegistryGet(
    Px4Lite_ModuleId_t module_id,
    Px4Lite_ModuleDescriptor_t *descriptor,
    Px4Lite_ModuleRuntime_t *runtime)
{
    Px4Lite_RegistrySlot_t *slot;

    if (((uint32_t)module_id >= (uint32_t)PX4LITE_MODULE_COUNT) ||
        ((descriptor == 0) && (runtime == 0)))
    {
        return PX4LITE_INVALID_PARAM;
    }

    taskENTER_CRITICAL();
    slot = &s_registry[module_id];
    if (slot->registered != 0U)
    {
        if (descriptor != 0)
        {
            *descriptor = slot->descriptor;
        }
        if (runtime != 0)
        {
            *runtime = slot->runtime;
        }
    }
    taskEXIT_CRITICAL();

    return (slot->registered != 0U)
               ? PX4LITE_OK
               : PX4LITE_NOT_READY;
}
