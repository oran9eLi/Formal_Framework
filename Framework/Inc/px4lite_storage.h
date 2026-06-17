/**
 * @file px4lite_storage.h
 * @brief Declare the registry-driven SD CSV storage module hooks.
 *
 * The storage module is registered like any other framework module
 * (lifecycle init + recover). It reads framework topics (navigation, baro,
 * battery) through the public copy API, formats CSV rows and lets the Storage
 * driver persist them to the SD card. It must not include Business headers.
 */

#ifndef PX4LITE_STORAGE_H
#define PX4LITE_STORAGE_H

#include "px4lite_types.h"

/**
 * @brief Module init callback: reset the record queue and SD service.
 */
Px4Lite_Result_t Px4Lite_StorageModuleInit(void);

/**
 * @brief Module recover callback (Health task): request a remount only.
 *
 * Sets a flag and returns immediately. The owning storage task performs the
 * actual SD re-init, so no bus I/O runs on the Health task.
 */
Px4Lite_Result_t Px4Lite_StorageRecover(void);

/**
 * @brief One bounded storage work cycle: mount, produce, consume, sync.
 */
void Px4Lite_StorageWorkRun(uint32_t now_ms);

#endif
