#include "sensor_module_template.h"
#include <string.h>

#define SENSOR_TEMPLATE_DEVICE_ID  0x0101U
#define SENSOR_TEMPLATE_OFFLINE_MS 1000U

/*
 * Replace these two declarations with the real BSP header.
 * ReadRaw must return OK only when one complete hardware sample is ready.
 */
extern Integrity_Result_t BSP_SensorTemplate_Init(void);
extern Integrity_Result_t BSP_SensorTemplate_ReadRaw(int32_t *raw_a, int32_t *raw_b, uint32_t *sample_time_ms);

static uint32_t s_sequence;
static uint32_t s_last_rx_ms;
static Integrity_ComponentState_t s_state;

Integrity_Result_t SensorTemplate_Init(uint32_t now_ms)
{
  Integrity_SnapshotInit();
  s_sequence   = 0U;
  s_last_rx_ms = now_ms;
  s_state      = COMPONENT_STARTING;

  if (BSP_SensorTemplate_Init() != INTEGRITY_OK) {
    s_state = COMPONENT_FAILED;
    return INTEGRITY_IO_ERROR;
  }

  return INTEGRITY_OK;
}

Integrity_Result_t SensorTemplate_Service(uint32_t now_ms)
{
  Integrity_ExampleSnapshot_t complete;
  Integrity_Result_t result;
  int32_t raw_a;
  int32_t raw_b;
  uint32_t sample_time_ms;

  result = BSP_SensorTemplate_ReadRaw(&raw_a, &raw_b, &sample_time_ms);

  if (result == INTEGRITY_IDLE) {
    if ((uint32_t)(now_ms - s_last_rx_ms) > SENSOR_TEMPLATE_OFFLINE_MS) { s_state = COMPONENT_OFFLINE; }
    return INTEGRITY_IDLE;
  }

  if (result != INTEGRITY_OK) { return result; }

  /*
   * Build the complete result in a private temporary object.
   * Never update the shared snapshot field by field.
   */
  memset(&complete, 0, sizeof(complete));
  complete.header.device_id       = SENSOR_TEMPLATE_DEVICE_ID;
  complete.header.sequence        = ++s_sequence;
  complete.header.sample_time_ms  = sample_time_ms;
  complete.header.publish_time_ms = now_ms;
  complete.header.valid           = 1U;
  complete.header.quality         = 100U;

  /* Replace with calibration/filtering using integer or bounded math. */
  complete.value_a     = raw_a;
  complete.value_b     = raw_b;
  complete.status_bits = 0U;

  result = Integrity_SnapshotPublish(&complete);
  if (result == INTEGRITY_OK) {
    s_last_rx_ms = now_ms;
    s_state      = COMPONENT_ONLINE;
  }
  return result;
}

Integrity_Result_t SensorTemplate_Copy(Integrity_ExampleSnapshot_t *snapshot)
{
  return Integrity_SnapshotCopy(snapshot);
}

Integrity_ComponentState_t SensorTemplate_GetState(void)
{
  return s_state;
}
