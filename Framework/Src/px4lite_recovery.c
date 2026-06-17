/**
 * @file px4lite_recovery.c
 * @brief Generic, registry-driven module recovery supervision.
 *
 * Policy lives here (Health task): detect an unhealthy registered module and,
 * rate-limited, invoke its `recover` callback. The recover callback MUST be
 * cheap (no blocking bus I/O) -- it only *requests* a re-init that the owning
 * module performs from its own Service task. This keeps shared-bus re-init off
 * the Health task, where it would race the sensor task on the same peripheral.
 *
 * Modularity: a new module is covered automatically once it is registered with
 * a non-null `recover` callback. No edit to this file is required.
 */

#include "px4lite_recovery.h"

#include "px4lite_config.h"
#include "px4lite_modules.h"
#include "px4lite_registry.h"

#if PX4LITE_RECOVERY_ENABLE
static uint32_t s_last_attempt_ms[PX4LITE_MODULE_COUNT];
static uint16_t s_attempt_count[PX4LITE_MODULE_COUNT];

/**
 * @brief Decide whether one module's state warrants a recovery attempt.
 */
static uint8_t Px4Lite_RecoveryNeeded(const Px4Lite_ModuleStatus_t *status)
{
    return ((status->state == PX4LITE_STATE_OFFLINE) ||
            (status->state == PX4LITE_STATE_FAILED) ||
            (status->consecutive_errors >=
             PX4LITE_RECOVERY_ERROR_THRESHOLD))
               ? 1U
               : 0U;
}
#endif

/**
 * @brief Evaluate recovery for every registered module that exposes a recover hook.
 */
void Px4Lite_RecoveryMonitorRun(uint32_t now_ms)
{
#if PX4LITE_RECOVERY_ENABLE
    uint32_t id;

    for (id = 0U; id < (uint32_t)PX4LITE_MODULE_COUNT; ++id)
    {
        Px4Lite_ModuleDescriptor_t descriptor;
        Px4Lite_ModuleStatus_t status;

        /* Skip slots that are not registered modules. */
        if (Px4Lite_RegistryGet((Px4Lite_ModuleId_t)id,
                                &descriptor, 0) != PX4LITE_OK)
        {
            continue;
        }
        if ((descriptor.enabled == 0U) || (descriptor.recover == 0))
        {
            continue;
        }
        if (Px4Lite_GetModuleStatus((Px4Lite_ModuleId_t)id,
                                    &status) != PX4LITE_OK)
        {
            continue;
        }

        /* Healthy (or only data-degraded): clear the retry budget. */
        if ((status.state == PX4LITE_STATE_ONLINE) ||
            (status.state == PX4LITE_STATE_DEGRADED))
        {
            s_attempt_count[id] = 0U;
            s_last_attempt_ms[id] = now_ms;
            continue;
        }

        /* STARTING/UNINITIALIZED/DISABLED are left to the startup path. */
        if (Px4Lite_RecoveryNeeded(&status) == 0U)
        {
            continue;
        }

        /*
         * Rate-limit attempts. Retries are intentionally unbounded so a device
         * unplugged for an arbitrary time still recovers when it returns
         * (hot-plug tolerance); the delay keeps the attempt rate sane.
         */
        if ((uint32_t)(now_ms - s_last_attempt_ms[id]) <
            PX4LITE_RECOVERY_DELAY_MS)
        {
            continue;
        }

        s_last_attempt_ms[id] = now_ms;
        if (s_attempt_count[id] < 0xFFFFU)
        {
            s_attempt_count[id]++;
        }

        /* Cheap arm-only callback; the owning Service performs the re-init. */
        (void)descriptor.recover();
    }
#else
    (void)now_ms;
#endif
}
