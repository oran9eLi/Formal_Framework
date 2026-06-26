/**
 * @file lv_conf.h
 * @brief LVGL 8.3 configuration for STM32F407 + SSD1963.
 *
 * @details
 * LVGL is used only inside the Display module. The display task remains the
 * single owner of LCD FSMC writes. The internal LVGL heap is placed in CCM to
 * keep the main SRAM available for FreeRTOS tasks, queues, and driver state.
 */

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

#if defined(__CC_ARM)
/* LVGL 8.3 is third-party code. ARMCC5 reports benign enum/sign/control-flow
 * diagnostics in the library sources; keep project builds warning-clean without
 * editing vendor files. */
#pragma diag_suppress 68
#pragma diag_suppress 111
#pragma diag_suppress 188
#pragma diag_suppress 546
#endif

#define LV_COLOR_DEPTH 16
#define LV_COLOR_16_SWAP 0
#define LV_COLOR_SCREEN_TRANSP 0
#define LV_COLOR_CHROMA_KEY    lv_color_hex(0x000000) /* logo transparent color = black */

#define LV_MEM_CUSTOM 0
#define LV_MEM_SIZE (40U * 1024U)
#define LV_MEM_ADR 0x10000000U
#define LV_MEM_BUF_MAX_NUM 8
#define LV_MEMCPY_MEMSET_STD 0

#define LV_DISP_DEF_REFR_PERIOD 30
#define LV_INDEV_DEF_READ_PERIOD 30
#define LV_TICK_CUSTOM 0
#define LV_DPI_DEF 130

#define LV_DRAW_COMPLEX 1
#define LV_SHADOW_CACHE_SIZE 0
#define LV_CIRCLE_CACHE_SIZE 1
#define LV_LAYER_SIMPLE_BUF_SIZE (4U * 1024U)
#define LV_LAYER_SIMPLE_FALLBACK_BUF_SIZE (1024U)
#define LV_IMG_CACHE_DEF_SIZE 0
#define LV_GRADIENT_MAX_STOPS 2
#define LV_GRAD_CACHE_DEF_SIZE 0
#define LV_DITHER_GRADIENT 0

#define LV_USE_GPU_STM32_DMA2D 0
#define LV_USE_GPU_ARM2D 0

#define LV_USE_LOG 0
#define LV_USE_ASSERT_NULL 1
#define LV_USE_ASSERT_MALLOC 1
#define LV_USE_ASSERT_STYLE 0
#define LV_USE_ASSERT_MEM_INTEGRITY 0
#define LV_USE_ASSERT_OBJ 0

#define LV_USE_PERF_MONITOR 0
#define LV_USE_MEM_MONITOR 0
#define LV_USE_REFR_DEBUG 0

#define LV_SPRINTF_CUSTOM 0
#define LV_USE_USER_DATA 1
#define LV_ENABLE_GC 0
#define LV_BIG_ENDIAN_SYSTEM 0

#define LV_USE_OS 0

#define LV_FONT_MONTSERRAT_8 0
#define LV_FONT_MONTSERRAT_10 0
#define LV_FONT_MONTSERRAT_12 1
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_MONTSERRAT_18 1
#define LV_FONT_MONTSERRAT_20 0
#define LV_FONT_MONTSERRAT_22 0
#define LV_FONT_MONTSERRAT_24 1
#define LV_FONT_MONTSERRAT_26 0
#define LV_FONT_MONTSERRAT_28 0
#define LV_FONT_MONTSERRAT_30 0
#define LV_FONT_MONTSERRAT_32 0
#define LV_FONT_MONTSERRAT_34 0
#define LV_FONT_MONTSERRAT_36 0
#define LV_FONT_MONTSERRAT_38 0
#define LV_FONT_MONTSERRAT_40 0
#define LV_FONT_MONTSERRAT_42 0
#define LV_FONT_MONTSERRAT_44 0
#define LV_FONT_MONTSERRAT_46 0
#define LV_FONT_MONTSERRAT_48 0
#define LV_FONT_MONTSERRAT_12_SUBPX 0
#define LV_FONT_MONTSERRAT_28_COMPRESSED 0
#define LV_FONT_DEJAVU_16_PERSIAN_HEBREW 0
#define LV_FONT_SIMSUN_16_CJK 0
#define LV_FONT_UNSCII_8 0
#define LV_FONT_UNSCII_16 0
#define LV_FONT_DEFAULT &lv_font_montserrat_16

#define LV_USE_THEME_DEFAULT 1
#define LV_THEME_DEFAULT_DARK 0
#define LV_THEME_DEFAULT_GROW 1
#define LV_THEME_DEFAULT_TRANSITION_TIME 80

#define LV_USE_ARC 1
#define LV_USE_BAR 1
#define LV_USE_BTN 1
#define LV_USE_BTNMATRIX 0
#define LV_USE_CANVAS 0
#define LV_USE_CHECKBOX 0
#define LV_USE_DROPDOWN 0
#define LV_USE_IMG 1
#define LV_USE_LABEL 1
#define LV_USE_LINE 1
#define LV_USE_ROLLER 0
#define LV_USE_SLIDER 1
#define LV_USE_SWITCH 0
#define LV_USE_TEXTAREA 0
#define LV_USE_TABLE 0

#define LV_USE_ANIMIMG 0
#define LV_USE_CALENDAR 0
#define LV_USE_CHART 0
#define LV_USE_COLORWHEEL 0
#define LV_USE_IMGBTN 0
#define LV_USE_KEYBOARD 0
#define LV_USE_LED 0
#define LV_USE_LIST 0
#define LV_USE_MENU 0
#define LV_USE_METER 0
#define LV_USE_MSGBOX 0
#define LV_USE_SPAN 0
#define LV_USE_SPINBOX 0
#define LV_USE_SPINNER 0
#define LV_USE_TABVIEW 0
#define LV_USE_TILEVIEW 0
#define LV_USE_WIN 0

#define LV_USE_FS_STDIO 0
#define LV_USE_FS_POSIX 0
#define LV_USE_FS_WIN32 0
#define LV_USE_FS_FATFS 0
#define LV_USE_PNG 0
#define LV_USE_BMP 0
#define LV_USE_SJPG 0
#define LV_USE_GIF 0
#define LV_USE_QRCODE 0
#define LV_USE_FREETYPE 0
#define LV_USE_RLOTTIE 0
#define LV_USE_FFMPEG 0

#define LV_USE_DEMO_WIDGETS 0
#define LV_USE_DEMO_KEYPAD_AND_ENCODER 0
#define LV_USE_DEMO_BENCHMARK 0
#define LV_USE_DEMO_STRESS 0
#define LV_USE_DEMO_MUSIC 0

#endif
