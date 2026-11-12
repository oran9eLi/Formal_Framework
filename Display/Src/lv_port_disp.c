/**
 * @file lv_port_disp.c
 * @brief Display 层 LVGL 与 SSD1963 RGB565 GRAM 同步刷新适配。
 *
 * @details
 * 主SRAM保留800x16 RGB565单缓冲；姿态圆圈先离屏构建完整帧再集中呈现，
 * 其余区域原路批量提交，不改其缓冲容量或采样。无半帧跨10ms节拍暴露。
 * 普通区域仍以16行协作；只有完整HUD专属窄条带可用同一缓冲容纳的49行，
 * 减少一帧重复建立绘图几何的次数，不增加缓冲或改变其他区域上限。
 * 原 24 行缓冲中超出既有 16 行上限的部分不被使用，精简释放 12800 B。
 * 仅支持非 full/direct 单缓冲模式；LVGL 堆仍占 CCM 起始 40 KiB。
 */

#include "lv_port_disp.h"

#include "display_ssd1963.h"
#include "display_hud_present.h"
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
/** @brief 显示所有者独占的HUD捕获矩形及批次事实；无跨任务访问。 */
static struct {
  uint16_t x;      /**< 帧矩形左上角X，px。 */
  uint16_t y;      /**< 帧矩形左上角Y，px。 */
  uint16_t width;  /**< 完整帧宽度，px。 */
  uint16_t height; /**< 完整帧高度，px。 */
  uint8_t active;  /**< 批次开始至全部脏区完成期间为1。 */
  uint8_t failed;  /**< 捕获失败后继续抑制本批次圆圈直写，避免漏出半帧。 */
} s_hud_batch;
/** @brief 仅HUD窄区域使用同一缓冲的全部容量；其他失效区域保持16行。 */
static void LvPortDisp_SelectRows(lv_disp_drv_t *drv)
{
  lv_disp_t *disp;
  uint16_t i;
  if(!drv) { return; }
  drv->refr_max_rows=LV_PORT_DISP_BUF_LINES;
  if(!s_hud_batch.active) { return; }
  disp=lv_disp_get_default();if(!disp) { return; }
  for(i=0U;i<disp->inv_p;i++) {
    const lv_area_t *area;
    if(disp->inv_area_joined[i]) { continue; }
    area=&disp->inv_areas[i];
    if(area->x1==(lv_coord_t)s_hud_batch.x &&
        area->x2==(lv_coord_t)(s_hud_batch.x+s_hud_batch.width-1U) &&
        area->y1>=(lv_coord_t)s_hud_batch.y &&
        area->y2<(lv_coord_t)(s_hud_batch.y+s_hud_batch.height)) {
      drv->refr_max_rows=(uint16_t)((LV_PORT_DISP_HOR_RES*LV_PORT_DISP_BUF_LINES)/s_hud_batch.width);
    }
    break;
  }
}
/** @brief 在已完整捕获/提交的条带之间检查软预算；单条带超限由统计暴露。 */
static bool LvPortDisp_Continue(lv_disp_drv_t *drv)
{ LvPortDisp_SelectRows(drv);return (bool)(App_RuntimeGetUs()-s_started_us<s_budget_us); }
/** @brief 重置本轮累计计时；flush含压缩捕获及实际提交，最大单次值跨轮保留。 */
void LvPortDisp_BeginStep(uint32_t started_us, uint32_t budget_us)
{ s_started_us = started_us; s_budget_us = budget_us; s_flush_us = 0U; }
/** @brief 仅读取显示所有者私有计时。 */
void LvPortDisp_GetFlushTime(uint32_t *total_us, uint32_t *maximum_us)
{ if (total_us) { *total_us = s_flush_us; } if (maximum_us) { *maximum_us = s_flush_max_us; } }

/** @brief 合并一次同步提交/捕获的耗时，包含最终整帧呈现。 */
static void LvPortDisp_AccountFlush(uint32_t started_us)
{
  uint32_t elapsed_us=App_RuntimeGetUs()-started_us;
  s_flush_us+=elapsed_us;
  if(elapsed_us>s_flush_max_us) { s_flush_max_us=elapsed_us; }
}

/** @brief 只开启合法的HUD完整帧，普通区域不进入捕获状态。 */
uint8_t LvPortDisp_BeginHudFrame(uint16_t x,uint16_t y,uint16_t width,uint16_t height)
{
  if(s_hud_batch.active || !Display_HudPresent_BeginFrame(x,y,width,height)) { return 0U; }
  s_hud_batch.x=x;s_hud_batch.y=y;s_hud_batch.width=width;s_hud_batch.height=height;
  s_hud_batch.failed=0U;s_hud_batch.active=1U;
  return 1U;
}

/** @brief 只在最后一个LVGL脏区完成后呈现，不在协作条带之间提交。 */
uint8_t LvPortDisp_FinishHudFrame(void)
{
  lv_disp_t *disp;
  uint8_t committed;
  uint32_t started_us;
  if(!s_hud_batch.active) { return 0U; }
  disp=lv_disp_get_default();
  if(disp && disp->inv_p!=0U) { return 0U; }
  started_us=App_RuntimeGetUs();
  committed=(uint8_t)(disp && !s_hud_batch.failed && Display_HudPresent_CommitFrame());
  if(!committed) { Display_HudPresent_AbortFrame(); }
  s_hud_batch.active=0U;s_disp_drv.refr_max_rows=LV_PORT_DISP_BUF_LINES;
  LvPortDisp_AccountFlush(started_us);
  return committed?1U:2U;
}

/** @brief 切页/重建/恢复不允许把旧页面的待呈现帧写到新页面。 */
void LvPortDisp_AbortHudFrame(void)
{
  s_hud_batch.active=0U;s_hud_batch.failed=0U;s_disp_drv.refr_max_rows=LV_PORT_DISP_BUF_LINES;
  Display_HudPresent_Reset();
}

/**
 * @brief 捕获圆圈覆盖区域，圆圈之外保留原批量提交与源行跨度。
 * @details 其他失效矩形可正常推进；意外窄片/乱序令整帧失效，仍不漏出半帧。
 */
static void LvPortDisp_Submit(uint16_t x,uint16_t y,uint16_t width,uint16_t height,const uint16_t *pixels)
{
  uint32_t right=(uint32_t)x+width,bottom=(uint32_t)y+height;
  uint32_t hud_right=(uint32_t)s_hud_batch.x+s_hud_batch.width;
  uint32_t hud_bottom=(uint32_t)s_hud_batch.y+s_hud_batch.height;
  uint16_t top,end,row;
  if(!s_hud_batch.active || right<=s_hud_batch.x || x>=hud_right ||
      bottom<=s_hud_batch.y || y>=hud_bottom) {
    Display_HudPresent_Flush(x,y,width,height,pixels);return;
  }
  top=y<s_hud_batch.y?(uint16_t)(s_hud_batch.y-y):0U;
  end=bottom>hud_bottom?(uint16_t)(hud_bottom-y):height;
  if(top) { Display_HudPresent_Flush(x,y,width,top,pixels); }
  if(x>s_hud_batch.x || right<hud_right) {
    s_hud_batch.failed=1U;Display_HudPresent_AbortFrame();
  }
  for(row=top;row<end;row++) {
    const uint16_t *input=&pixels[(uint32_t)row*width];
    if(x<s_hud_batch.x) {
      Display_HudPresent_Flush(x,(uint16_t)(y+row),(uint16_t)(s_hud_batch.x-x),1U,input);
    }
    if(!s_hud_batch.failed && !Display_HudPresent_Capture(s_hud_batch.x,(uint16_t)(y+row),
        s_hud_batch.width,1U,&input[s_hud_batch.x-x])) {
      s_hud_batch.failed=1U;Display_HudPresent_AbortFrame();
    }
    if(right>hud_right) {
      Display_HudPresent_Flush((uint16_t)hud_right,(uint16_t)(y+row),(uint16_t)(right-hud_right),1U,&input[hud_right-x]);
    }
  }
  if(end<height) {
    Display_HudPresent_Flush(x,(uint16_t)(y+end),width,(uint16_t)(height-end),&pixels[(uint32_t)end*width]);
  }
}

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
    LvPortDisp_Submit((uint16_t)area->x1,(uint16_t)area->y1,(uint16_t)width,(uint16_t)height,(const uint16_t *)color_p);
    LvPortDisp_AccountFlush(started_us);
  }

  lv_disp_flush_ready(drv);
}

/**
 * @brief 注册固定容量的协作刷新驱动；显示任务独占初始化和恢复。
 */
uint8_t LvPortDisp_Init(void)
{
  LvPortDisp_AbortHudFrame();
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
