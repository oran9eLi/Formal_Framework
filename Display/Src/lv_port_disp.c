/**
 * @file lv_port_disp.c
 * @brief LVGL display port for SSD1963 RGB565 GRAM flushing.
 *
 * @details
 * The draw buffer is a fixed static 800x32 RGB565 block in main SRAM. LVGL
 * renders only dirty areas into this local buffer; the flush callback writes
 * pixels into SSD1963 internal GRAM through the Display driver. No full-screen
 * framebuffer is allocated.
 *
 * Buffer sizing (STM32F407, 见 Development_Guide/19): 主 SRAM 128KB，链接占用约
 * 74KB（含 40KB FreeRTOS 堆），栈仅 1KB 在顶部，余约 54KB 空闲；LVGL 堆 40KB 在
 * CCM(0x10000000)，不占主 SRAM。本缓冲取 800x32x2 = 50KB，改后主 SRAM 约用 108KB、
 * 余约 20KB（另有 24KB CCM 空闲），在合理裕量下尽量放大：整屏重绘的条带 flush 次数
 * 由 48 次降到 15 次，明显缩短切页扫描感。flush 不走 DMA，缓冲放主 SRAM 即可。
 */

#include "lv_port_disp.h"

#include "display_ssd1963.h"
#include "lvgl.h"

#define LV_PORT_DISP_BUF_LINES 32U
#define LV_PORT_DISP_HOR_RES   DISPLAY_SSD1963_WIDTH
#define LV_PORT_DISP_VER_RES   DISPLAY_SSD1963_HEIGHT

static lv_disp_draw_buf_t s_draw_buf;
static lv_disp_drv_t s_disp_drv;
static lv_color_t s_draw_buf_1[LV_PORT_DISP_HOR_RES * LV_PORT_DISP_BUF_LINES];
static uint8_t s_disp_registered;

/**
 * @brief Flush one LVGL dirty area to SSD1963 GRAM.
 */
static void LvPortDisp_Flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p)
{
  int32_t width;
  int32_t height;

  if ((drv == 0) || (area == 0) || (color_p == 0)) {
    return;
  }

  if ((area->x2 < 0) || (area->y2 < 0) || (area->x1 >= (lv_coord_t)LV_PORT_DISP_HOR_RES) || (area->y1 >= (lv_coord_t)LV_PORT_DISP_VER_RES)) {
    lv_disp_flush_ready(drv);
    return;
  }

  width  = (int32_t)area->x2 - (int32_t)area->x1 + 1;
  height = (int32_t)area->y2 - (int32_t)area->y1 + 1;
  if ((width > 0) && (height > 0)) {
    Display_Ssd1963_FlushPixels((uint16_t)area->x1, (uint16_t)area->y1, (uint16_t)width, (uint16_t)height, (const uint16_t *)color_p);
  }

  lv_disp_flush_ready(drv);
}

/**
 * @brief Initialize LVGL display driver registration.
 */
uint8_t LvPortDisp_Init(void)
{
  if (Display_Ssd1963_Init() != DISPLAY_SSD1963_OK) {
    s_disp_registered = 0U;
    return 0U;
  }

  if (s_disp_registered == 0U) {
    lv_disp_draw_buf_init(&s_draw_buf, s_draw_buf_1, 0, (uint32_t)(LV_PORT_DISP_HOR_RES * LV_PORT_DISP_BUF_LINES));

    lv_disp_drv_init(&s_disp_drv);
    s_disp_drv.hor_res  = (lv_coord_t)LV_PORT_DISP_HOR_RES;
    s_disp_drv.ver_res  = (lv_coord_t)LV_PORT_DISP_VER_RES;
    s_disp_drv.flush_cb = LvPortDisp_Flush;
    s_disp_drv.draw_buf = &s_draw_buf;
    (void)lv_disp_drv_register(&s_disp_drv);
    s_disp_registered = 1U;
  }

  return 1U;
}

/**
 * @brief Probe the LCD controller while keeping SSD1963 details inside Display.
 */
uint8_t LvPortDisp_IsReady(void)
{
  return (Display_Ssd1963_Probe() == DISPLAY_SSD1963_OK) ? 1U : 0U;
}
