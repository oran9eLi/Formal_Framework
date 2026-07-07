/**
 * @file storage_module_event.c
 * @brief 跟踪模块状态边沿，避免 EVENT 日志重复记录持续性离线。
 */

#include "storage_module_event.h"

#include <string.h>
#include "px4lite_faults.h"

typedef struct {
  Px4Lite_ModuleId_t id;
  const char *source;
  const char *degraded_message;
  const char *offline_message;
  const char *failed_message;
  const char *recovered_message;
} Storage_ModuleEventDef_t;

typedef struct {
  uint8_t known;
  uint8_t active;
  uint32_t count;
} Storage_ModuleEventState_t;

static const Storage_ModuleEventDef_t s_module_defs[] = {
  {PX4LITE_MODULE_GNSS, "GNSS", "gnss_degraded", "gnss_offline", "gnss_failed", "gnss_recovered"},
  {PX4LITE_MODULE_IMU, "IMU", "imu_degraded", "imu_offline", "imu_failed", "imu_recovered"},
  {PX4LITE_MODULE_BARO, "BARO", "baro_degraded", "baro_offline", "baro_failed", "baro_recovered"},
  {PX4LITE_MODULE_BATTERY, "BATTERY", "battery_degraded", "battery_offline", "battery_failed", "battery_recovered"},
  {PX4LITE_MODULE_LORA, "LORA", "lora_degraded", "lora_offline", "lora_failed", "lora_recovered"},
  {PX4LITE_MODULE_STORAGE, "STORAGE", "storage_degraded", "storage_offline", "storage_failed", "storage_recovered"},
  {PX4LITE_MODULE_DISPLAY, "DISPLAY", "display_degraded", "display_offline", "display_failed", "display_recovered"}
};

static Storage_ModuleEventState_t s_states[PX4LITE_MODULE_COUNT];

static const Storage_ModuleEventDef_t *StorageModuleEvent_FindDef(Px4Lite_ModuleId_t id)
{
  uint32_t i;

  for (i = 0U; i < (uint32_t)(sizeof(s_module_defs) / sizeof(s_module_defs[0])); i++) {
    if (s_module_defs[i].id == id) { return &s_module_defs[i]; }
  }

  return 0;
}

static uint8_t StorageModuleEvent_IsActiveState(Px4Lite_State_t state)
{
  return ((state == PX4LITE_STATE_DEGRADED) || (state == PX4LITE_STATE_OFFLINE) || (state == PX4LITE_STATE_FAILED)) ? 1U : 0U;
}

static const char *StorageModuleEvent_MessageForState(const Storage_ModuleEventDef_t *def, Px4Lite_State_t state)
{
  if (state == PX4LITE_STATE_DEGRADED) { return def->degraded_message; }
  if (state == PX4LITE_STATE_FAILED) { return def->failed_message; }
  return def->offline_message;
}

void StorageModuleEvent_Init(void)
{
  memset(s_states, 0, sizeof(s_states));
}

Px4Lite_Result_t StorageModuleEvent_Update(const Px4Lite_ModuleStatus_t *status, Storage_ModuleEvent_t *event)
{
  const Storage_ModuleEventDef_t *def;
  Storage_ModuleEventState_t *state;
  uint8_t active;

  if ((status == 0) || (event == 0) || (status->module_id >= PX4LITE_MODULE_COUNT)) { return PX4LITE_INVALID_PARAM; }

  def = StorageModuleEvent_FindDef(status->module_id);
  if (def == 0) { return PX4LITE_IDLE; }

  state = &s_states[status->module_id];
  active = StorageModuleEvent_IsActiveState(status->state);

  if (state->known == 0U) {
    state->known = 1U;
    state->active = active;
    if (active == 0U) { return PX4LITE_IDLE; }
  } else {
    if (state->active == active) { return PX4LITE_IDLE; }
    state->active = active;
  }

  memset(event, 0, sizeof(*event));
  event->source = def->source;
  event->state  = (uint32_t)status->state;

  if (active != 0U) {
    if (state->count < 0xFFFFFFFFUL) { state->count++; }
    event->fault    = status->fault_code;
    event->severity = status->severity;
    event->active   = 1U;
    event->count    = state->count;
    event->message  = StorageModuleEvent_MessageForState(def, status->state);
  } else {
    event->fault    = PX4LITE_FAULT_NONE;
    event->severity = PX4LITE_SEVERITY_INFO;
    event->active   = 0U;
    event->count    = state->count;
    event->message  = def->recovered_message;
  }

  return PX4LITE_OK;
}
