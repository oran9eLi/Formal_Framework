#ifndef DISPLAY_LOGO_H
#define DISPLAY_LOGO_H

#include <stdint.h>
#include "display.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DISPLAY_HEADER_LOGO_WIDTH             64U
#define DISPLAY_HEADER_LOGO_HEIGHT            64U
#define DISPLAY_HEADER_LOGO_TRANSPARENT_COLOR 0x0000U

/* Raw RGB565 (little-endian) pixel data — 0x0000 is the transparent colour. */
extern const uint8_t display_logo_rgb565_data[DISPLAY_HEADER_LOGO_WIDTH * DISPLAY_HEADER_LOGO_HEIGHT * 2U];

/**
 * @brief       在指定左上角坐标绘制 64x64 页面标题栏 Logo
 * @param       x: 左上角 X 坐标
 * @param       y: 左上角 Y 坐标
 * @retval      Display_Result_t: 绘制结果
 */
Display_Result_t Display_PagesDrawHeaderLogo(uint16_t x, uint16_t y);

#ifdef __cplusplus
}
#endif

#endif
