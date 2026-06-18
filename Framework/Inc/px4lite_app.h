/**
 * @file px4lite_app.h
 * @brief Declare framework initialization and task creation.
 */

#ifndef PX4LITE_APP_H
#define PX4LITE_APP_H

#include "FreeRTOS.h"

/**
 * @brief Initialize framework data services and create the fixed task set.
 */
BaseType_t Px4Lite_AppInit(void);

#endif
