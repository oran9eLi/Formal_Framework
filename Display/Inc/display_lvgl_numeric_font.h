/**
 * @file display_lvgl_numeric_font.h
 * @brief Display层数字等宽适配；复用现有LVGL字形，不修改生产数据。
 * @note 仅提供字体度量和位图回调，禁止依赖Framework/BSP或动态分配。
 */
#ifndef DISPLAY_LVGL_NUMERIC_FONT_H
#define DISPLAY_LVGL_NUMERIC_FONT_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const lv_font_t display_lvgl_numeric_12; /**< 12 px字形，数字步进8 px。 */
extern const lv_font_t display_lvgl_numeric_14; /**< 14 px字形，数字步进10 px。 */
extern const lv_font_t display_lvgl_numeric_16; /**< 16 px字形，数字步进11 px。 */
extern const lv_font_t display_lvgl_hex_14; /**< 原始HEX行；0-9/A-F/空格步进12 px。 */

/**
 * @brief 为当前屏幕使用的Montserrat 12/14/16字体选择数字等宽代理。
 * @param[in] font 原字体或已适配字体；其他字体保持原指针。
 * @return 生命周期覆盖整个固件运行期的只读字体。
 */
const lv_font_t *Display_LvglNumericFont(const lv_font_t *font);

#ifdef __cplusplus
}
#endif
#endif
