#ifndef SENSOR_MODULE_TEMPLATE_H
#define SENSOR_MODULE_TEMPLATE_H

#include "integrity_snapshot.h"

Integrity_Result_t SensorTemplate_Init(uint32_t now_ms);
Integrity_Result_t SensorTemplate_Service(uint32_t now_ms);
Integrity_Result_t SensorTemplate_Copy(
    Integrity_ExampleSnapshot_t *snapshot);
Integrity_ComponentState_t SensorTemplate_GetState(void);

#endif
