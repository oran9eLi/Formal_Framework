#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "px4lite_faults.h"
#include "storage_module_event.h"

static int ExpectUint32(const char *label, uint32_t actual, uint32_t expected)
{
  if (actual != expected) {
    printf("%s: expected %lu, got %lu\n", label, (unsigned long)expected, (unsigned long)actual);
    return 0;
  }
  return 1;
}

static int ExpectString(const char *label, const char *actual, const char *expected)
{
  if ((actual == 0) || (strcmp(actual, expected) != 0)) {
    printf("%s: expected %s, got %s\n", label, expected, (actual != 0) ? actual : "(null)");
    return 0;
  }
  return 1;
}

static Px4Lite_ModuleStatus_t MakeStatus(Px4Lite_ModuleId_t id, Px4Lite_State_t state, uint16_t fault, uint8_t severity)
{
  Px4Lite_ModuleStatus_t status;

  memset(&status, 0, sizeof(status));
  status.module_id   = id;
  status.state       = state;
  status.fault_code  = fault;
  status.severity    = severity;
  return status;
}

int main(void)
{
  Px4Lite_ModuleStatus_t status;
  Storage_ModuleEvent_t event;
  int ok = 1;

  StorageModuleEvent_Init();

  status = MakeStatus(PX4LITE_MODULE_IMU, PX4LITE_STATE_ONLINE, PX4LITE_FAULT_NONE, PX4LITE_SEVERITY_INFO);
  ok &= ExpectUint32("initial online has no event", (uint32_t)StorageModuleEvent_Update(&status, &event), (uint32_t)PX4LITE_IDLE);

  status = MakeStatus(PX4LITE_MODULE_IMU, PX4LITE_STATE_OFFLINE, PX4LITE_FAULT_SENSOR_OFFLINE, PX4LITE_SEVERITY_ERROR);
  ok &= ExpectUint32("offline emits event", (uint32_t)StorageModuleEvent_Update(&status, &event), (uint32_t)PX4LITE_OK);
  ok &= ExpectString("offline source", event.source, "IMU");
  ok &= ExpectString("offline message", event.message, "imu_offline");
  ok &= ExpectUint32("offline active", event.active, 1U);
  ok &= ExpectUint32("offline state", event.state, (uint32_t)PX4LITE_STATE_OFFLINE);
  ok &= ExpectUint32("offline fault", event.fault, PX4LITE_FAULT_SENSOR_OFFLINE);

  ok &= ExpectUint32("repeat offline has no event", (uint32_t)StorageModuleEvent_Update(&status, &event), (uint32_t)PX4LITE_IDLE);

  status = MakeStatus(PX4LITE_MODULE_IMU, PX4LITE_STATE_ONLINE, PX4LITE_FAULT_NONE, PX4LITE_SEVERITY_INFO);
  ok &= ExpectUint32("online recovered emits event", (uint32_t)StorageModuleEvent_Update(&status, &event), (uint32_t)PX4LITE_OK);
  ok &= ExpectString("recovered source", event.source, "IMU");
  ok &= ExpectString("recovered message", event.message, "imu_recovered");
  ok &= ExpectUint32("recovered active", event.active, 0U);
  ok &= ExpectUint32("recovered state", event.state, (uint32_t)PX4LITE_STATE_ONLINE);
  ok &= ExpectUint32("recovered fault", event.fault, PX4LITE_FAULT_NONE);

  if (ok != 0) {
    printf("storage module event tests passed\n");
    return 0;
  }

  return 1;
}
