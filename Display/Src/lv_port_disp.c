/**
 * @file lv_port_disp.c
 * @brief Display 层 LVGL 与 SSD1963 RGB565 GRAM 同步刷新适配。
 *
 * @details
 * 主 SRAM 中静态保留 800x16 RGB565 单缓冲；LVGL 仅绘制脏区域，经
 * Display 驱动同步写入 SSD1963 GRAM，不分配整屏缓冲、不直接操作 BSP。
 * 缓冲行数与协作刷新条带上限使用同一常量，满屏 480 行至少分为 30 条带。
 * 原 24 行缓冲中超出既有 16 行上限的部分不被使用，精简释放 12800 B。
 * 仅支持非 full/direct 单缓冲模式；LVGL 堆仍占 CCM 起始 40 KiB。
 */

#include "lv_port_disp.h"

#include "display_ssd1963.h"
#include "lvgl.h"
#include "app_data_api.h"

#define LV_PORT_DISP_BUF_LINES 16U
#define LV_PORT_DISP_HOR_RES   DISPLAY_SSD1963_WIDTH
#define LV_PORT_DISP_VER_RES   DISPLAY_SSD1963_HEIGHT

static lv_disp_draw_buf_t s_draw_buf;
static lv_disp_drv_t s_disp_drv;
static lv_color_t s_draw_buf_1[LV_PORT_DISP_HOR_RES * LV_PORT_DISP_BUF_LINES];
static uint8_t s_disp_registered;
static uint32_t s_started_us;
static uint32_t s_budget_us;
static uint32_t s_flush_us;
static uint32_t s_flush_max_us;
/** @brief 在已完整flush的条带之间检查软预算；单条带超限由统计暴露。 */
static bool LvPortDisp_Continue(lv_disp_drv_t *drv)
{ (void)drv; return (bool)(App_RuntimeGetUs() - s_started_us < s_budget_us); }
/** @brief 重置本轮累计计时，最大单次值跨轮保留。 */
void LvPortDisp_BeginStep(uint32_t started_us, uint32_t budget_us)
{ s_started_us = started_us; s_budget_us = budget_us; s_flush_us = 0U; }
/** @brief 仅读取显示所有者私有计时。 */
void LvPortDisp_GetFlushTime(uint32_t *total_us, uint32_t *maximum_us)
{ if (total_us) { *total_us = s_flush_us; } if (maximum_us) { *maximum_us = s_flush_max_us; } }

/**
 * @brief 同步写入一个完整脏区域；仅显示任务调用，不保留像素指针。
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
    uint32_t started_us = App_RuntimeGetUs();
    uint32_t elapsed_us;
    Display_Ssd1963_FlushPixels((uint16_t)area->x1, (uint16_t)area->y1, (uint16_t)width, (uint16_t)height, (const uint16_t *)color_p);
    elapsed_us = App_RuntimeGetUs() - started_us;
    s_flush_us += elapsed_us;
    if (elapsed_us > s_flush_max_us) { s_flush_max_us = elapsed_us; }
  }

  lv_disp_flush_ready(drv);
}

/**
 * @brief 注册固定容量的协作刷新驱动；显示任务独占初始化和恢复。
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
    s_disp_drv.full_refresh = 0U;
    s_disp_drv.direct_mode = 0U;
    s_disp_drv.hor_res  = (lv_coord_t)LV_PORT_DISP_HOR_RES;
    s_disp_drv.ver_res  = (lv_coord_t)LV_PORT_DISP_VER_RES;
    s_disp_drv.flush_cb = LvPortDisp_Flush;
    s_disp_drv.draw_buf = &s_draw_buf;
    s_disp_drv.refr_continue_cb = LvPortDisp_Continue;
    s_disp_drv.refr_max_rows = LV_PORT_DISP_BUF_LINES;
    {
      lv_disp_t *disp = lv_disp_drv_register(&s_disp_drv);
      if (!disp) { return 0U; }
      lv_timer_set_period(_lv_disp_get_refr_timer(disp), 10U);
    }
    s_disp_registered = 1U;
  }

  return 1U;
}
