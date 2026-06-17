/**
 * @file px4lite_recovery.h
 * @brief Declare policy-controlled module recovery monitoring.
 */

#ifndef PX4LITE_RECOVERY_H
#define PX4LITE_RECOVERY_H

#include "px4lite_types.h"

/**
 * @brief Evaluate configured recovery policies from the Health task context.
 *
 * The call site is always present. Disabled policies allocate no resources
 * and perform no recovery action.
 */
void Px4Lite_RecoveryMonitorRun(uint32_t now_ms);

#endif
