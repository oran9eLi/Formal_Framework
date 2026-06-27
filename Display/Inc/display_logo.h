/**
 * @file display_logo.h
 * @brief Display 层页眉 Logo 位图资源声明。
 *
 * @details
 * 本文件只声明 LVGL 页眉使用的 RGB565 只读像素数据，不包含页面逻辑、
 * 业务状态或 Framework 内部接口。显示页面应通过 Display/LVGL 后端引用该资源。
 */

#ifndef DISPLAY_LOGO_H
#define DISPLAY_LOGO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DISPLAY_HEADER_LOGO_WIDTH             64U
#define DISPLAY_HEADER_LOGO_HEIGHT            64U
#define DISPLAY_HEADER_LOGO_TRANSPARENT_COLOR 0x0000U

/**
 * @brief 64x64 RGB565 小端像素数据，颜色 0x0000 作为透明色处理。
 */
extern const uint8_t display_logo_rgb565_data[DISPLAY_HEADER_LOGO_WIDTH * DISPLAY_HEADER_LOGO_HEIGHT * 2U];

#ifdef __cplusplus
}
#endif

#endif
