/**
 * @file display_gt911.h
 * @brief Declare the GT911 capacitive touch driver.
 */

#ifndef DISPLAY_GT911_H
#define DISPLAY_GT911_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    DISPLAY_GT911_OK = 0,
    DISPLAY_GT911_NOT_READY,
    DISPLAY_GT911_NO_POINT,
    DISPLAY_GT911_NO_DATA,
    DISPLAY_GT911_ERROR
} Display_Gt911Result_t;

typedef struct
{
    uint8_t addr;
    uint8_t last_info;
    uint8_t int_level;
    uint8_t scl_level;
    uint8_t sda_level;
    uint8_t last_result;
    uint8_t last_error;
    uint8_t cfg_version;
    uint8_t cfg_verified;
    uint8_t init_error;
    uint8_t touch_status;
    uint8_t orig_ver;
    uint8_t cfg_bytes[4];
} Display_Gt911Debug_t;

/**
 * @brief Initialize the GT911 touch controller.
 */
Display_Gt911Result_t Display_Gt911_Init(void);
/**
 * @brief Scan one touch point and return landscape display coordinates.
 */
Display_Gt911Result_t Display_Gt911_Scan(uint16_t *x, uint16_t *y);
/**
 * @brief Copy touch diagnostic state.
 */
void Display_Gt911_GetDebug(Display_Gt911Debug_t *debug);

#ifdef __cplusplus
}
#endif

#endif
