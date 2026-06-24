/**
 * @file main.h
 * @brief Declare the firmware entry-point support and fatal error handler.
 */

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif
