/**
 * @file display_hud_present.c
 * @brief HUD完整258x258后备帧的逐行RGB565 RLE无损编码与集中提交。
 * @details Display任务独占；Capture不碰可见GRAM，完整帧才允许Commit。
 * CCM前40KB仅作LVGL链接保留，只使用尾24KB；不用于DMA、不动态分配。
 * 普通Flush一次原批量提交，不改变其他区域渲染或驱动时序。
 */
#include "display_hud_present.h"
#include "display_ssd1963.h"
#include <string.h>

/* 仅本单元局部优化，不改变全工程与外设编译设置。 */
#if defined(__CC_ARM)
#pragma push
#pragma O2
#pragma Otime
#endif

#if DISPLAY_HUD_DIFF_ENABLE
#include "lv_conf.h"
#define HUD_SIZE 258U
#define HUD_CCM_BYTES 24576U
#define HUD_CCM_BASE 0x10000000UL
#define HUD_LVGL_BYTES 40960U
#define HUD_CCM_LIMIT 0x10010000UL
#define HUD_RUN_BYTES 4U

#if LV_COLOR_DEPTH != 16 || LV_COLOR_16_SWAP != 0
#error "HUD frame staging requires unswapped RGB565"
#endif
#if LV_MEM_ADR != HUD_CCM_BASE || LV_MEM_SIZE != HUD_LVGL_BYTES
#error "HUD CCM reservation requires LVGL at 0x10000000 with exactly 40KB"
#endif
#if HUD_CCM_BASE+HUD_LVGL_BYTES+HUD_CCM_BYTES != HUD_CCM_LIMIT
#error "HUD cache must fit exactly in the reserved CCM tail"
#endif

#if defined(__CC_ARM) || defined(DISPLAY_HUD_PRESENT_TEST_CCM_RESERVE)
#if defined(__CC_ARM)
/** @brief 防止链接器另用LVGL裸地址区；绝不整体清零，只访问尾24KB。 */
static uint8_t s_hud_ccm_reservation[65536U] __attribute__((at(0x10000000)));
#else
/** @brief 主机模拟完整CCM，前40KB哨兵检测越界。 */
static uint8_t s_hud_ccm_reservation[65536U];
#endif
#define HUD_FRAME_CACHE (s_hud_ccm_reservation+HUD_LVGL_BYTES)
#else
/** @brief 非ARMCC主机仅模拟实际帧缓存。 */
static uint8_t s_hud_ccm_tail[HUD_CCM_BYTES];
#define HUD_FRAME_CACHE s_hud_ccm_tail
#endif

/** @brief 单行RGB565解码scratch，CPU专用，不用于DMA。 */
static uint16_t s_hud_row[HUD_SIZE];

/** @brief HUD配置/当前帧区域与编码长度，不保留像素指针。 */
typedef struct {
  uint16_t x;             /**< 配置HUD左上角X，px。 */
  uint16_t y;             /**< 配置HUD左上角Y，px。 */
  uint16_t width;         /**< 配置宽，px；258才启用。 */
  uint16_t height;        /**< 配置高，px；258才启用。 */
  uint16_t frame_x;       /**< 本帧左上角X，px。 */
  uint16_t frame_y;       /**< 本帧左上角Y，px。 */
  uint16_t encoded_bytes; /**< 当前RLE长度，B，0..24576。 */
  uint8_t active;         /**< 配置身份启用为1。 */
  uint8_t valid;          /**< 配色身份0灰盘、1有效姿态。 */
} HudPresentState_t;
static HudPresentState_t s_hud_present;
static Display_HudPresentStats_t s_hud_stats;

/** @brief 撤销一次在途帧，保留原因与编码量，不改GRAM。 */
static void HudPresent_Invalidate(Display_HudFrameState_t reason)
{
  if(s_hud_stats.state==DISPLAY_HUD_FRAME_BUILDING || s_hud_stats.state==DISPLAY_HUD_FRAME_READY) {
    s_hud_stats.aborted_frames++;
    s_hud_stats.last_encoded_bytes=s_hud_present.encoded_bytes;
  }
  s_hud_present.encoded_bytes=0U;s_hud_stats.captured_rows=0U;s_hud_stats.state=(uint8_t)reason;
}

/** @brief 明确按字节保存(u16长度,u16原色)，不访问可能未对齐的u16缓存。 */
static uint8_t HudPresent_Append(uint16_t count,uint16_t color)
{
  uint16_t n=s_hud_present.encoded_bytes;
  if((uint32_t)n+HUD_RUN_BYTES>HUD_CCM_BYTES) { return 0U; }
  HUD_FRAME_CACHE[n]=(uint8_t)count;HUD_FRAME_CACHE[n+1U]=(uint8_t)(count>>8U);
  HUD_FRAME_CACHE[n+2U]=(uint8_t)color;HUD_FRAME_CACHE[n+3U]=(uint8_t)(color>>8U);
  n=(uint16_t)(n+HUD_RUN_BYTES);s_hud_present.encoded_bytes=n;s_hud_stats.last_encoded_bytes=n;
  if(n>s_hud_stats.maximum_encoded_bytes) { s_hud_stats.maximum_encoded_bytes=n; }
  return 1U;
}

/** @brief 从已证明范围有效的字节偏移读一个run。 */
static void HudPresent_Read(uint16_t offset,uint16_t *count,uint16_t *color)
{
  *count=(uint16_t)((uint16_t)HUD_FRAME_CACHE[offset]|((uint16_t)HUD_FRAME_CACHE[offset+1U]<<8U));
  *color=(uint16_t)((uint16_t)HUD_FRAME_CACHE[offset+2U]|((uint16_t)HUD_FRAME_CACHE[offset+3U]<<8U));
}

/** @brief 提交前验证整个流恰好组成258行，坏流不能先写前半屏。 */
static uint8_t HudPresent_CheckStream(void)
{
  uint16_t offset=0U,row;
  if(!s_hud_present.encoded_bytes || s_hud_present.encoded_bytes>HUD_CCM_BYTES ||
      s_hud_present.encoded_bytes%HUD_RUN_BYTES!=0U) { return 0U; }
  for(row=0U;row<HUD_SIZE;row++) {
    uint16_t pixels=0U;
    while(pixels<HUD_SIZE) {
      uint16_t count,color;
      if((uint32_t)offset+HUD_RUN_BYTES>s_hud_present.encoded_bytes) { return 0U; }
      HudPresent_Read(offset,&count,&color);(void)color;
      if(count==0U || count>HUD_SIZE-pixels) { return 0U; }
      pixels=(uint16_t)(pixels+count);offset=(uint16_t)(offset+HUD_RUN_BYTES);
    }
  }
  return (uint8_t)(offset==s_hud_present.encoded_bytes);
}
#endif

/** @brief 同一身份不打断捕获；配置变化先撤销当前后备帧。 */
void Display_HudPresent_Configure(uint8_t active,uint8_t valid,uint16_t x,uint16_t y,
                                uint16_t width,uint16_t height)
{
#if DISPLAY_HUD_DIFF_ENABLE
  uint8_t enabled=(uint8_t)(active && width==HUD_SIZE && height==HUD_SIZE &&
      (uint32_t)x+width<=DISPLAY_SSD1963_WIDTH && (uint32_t)y+height<=DISPLAY_SSD1963_HEIGHT);
  valid=(uint8_t)(valid!=0U);
  if(s_hud_present.active!=enabled || s_hud_present.valid!=valid || s_hud_present.x!=x ||
      s_hud_present.y!=y || s_hud_present.width!=width || s_hud_present.height!=height) {
    Display_HudPresent_AbortFrame();
    s_hud_present.active=enabled;s_hud_present.valid=valid;s_hud_present.x=x;s_hud_present.y=y;
    s_hud_present.width=width;s_hud_present.height=height;
  }
#else
  (void)active;(void)valid;(void)x;(void)y;(void)width;(void)height;
#endif
}

/** @brief 切页/恢复撤销身份与帧；保留累计统计，不清大缓冲。 */
void Display_HudPresent_Reset(void)
{
#if DISPLAY_HUD_DIFF_ENABLE
  Display_HudPresent_AbortFrame();s_hud_present.active=0U;
#endif
}

/** @brief 普通区域保持一次原批量提交，不参与HUD后备帧。 */
void Display_HudPresent_Flush(uint16_t x,uint16_t y,uint16_t width,uint16_t height,const uint16_t *pixels)
{
  Display_Ssd1963_FlushPixels(x,y,width,height,pixels);
}

/** @brief 验证尺寸/边界/LCD事实后开始新帧，不写屏。 */
uint8_t Display_HudPresent_BeginFrame(uint16_t x,uint16_t y,uint16_t width,uint16_t height)
{
#if DISPLAY_HUD_DIFF_ENABLE
  Display_HudPresent_AbortFrame();
  if(width!=HUD_SIZE || height!=HUD_SIZE || (uint32_t)x+width>DISPLAY_SSD1963_WIDTH ||
      (uint32_t)y+height>DISPLAY_SSD1963_HEIGHT || !Display_Ssd1963_IsReady()) { return 0U; }
  s_hud_present.frame_x=x;s_hud_present.frame_y=y;s_hud_present.encoded_bytes=0U;
  s_hud_stats.captured_rows=0U;s_hud_stats.last_encoded_bytes=0U;
  s_hud_stats.state=DISPLAY_HUD_FRAME_BUILDING;s_hud_stats.started_frames++;return 1U;
#else
  (void)x;(void)y;(void)width;(void)height;return 0U;
#endif
}

/** @brief 只有正在捕获或完整待提交时保留帧所有权。 */
uint8_t Display_HudPresent_IsBuilding(void)
{
#if DISPLAY_HUD_DIFF_ENABLE
  return (uint8_t)(s_hud_stats.state==DISPLAY_HUD_FRAME_BUILDING || s_hud_stats.state==DISPLAY_HUD_FRAME_READY);
#else
  return 0U;
#endif
}

/** @brief 连续完整行编码原始RGB565；错误或容量不足立即撤销整帧。 */
Display_HudCaptureResult_t Display_HudPresent_Capture(uint16_t x,uint16_t y,uint16_t width,
                                                    uint16_t height,const uint16_t *pixels)
{
#if DISPLAY_HUD_DIFF_ENABLE
  uint16_t row;
  if(s_hud_stats.state!=DISPLAY_HUD_FRAME_BUILDING || !pixels || width!=HUD_SIZE || !height ||
      x!=s_hud_present.frame_x || (uint32_t)y!=(uint32_t)s_hud_present.frame_y+s_hud_stats.captured_rows ||
      (uint32_t)s_hud_stats.captured_rows+height>HUD_SIZE || !Display_Ssd1963_IsReady()) {
    s_hud_stats.error_frames++;HudPresent_Invalidate(DISPLAY_HUD_FRAME_ERROR);return DISPLAY_HUD_CAPTURE_ERROR;
  }
  for(row=0U;row<height;row++) {
    const uint16_t *line=&pixels[(uint32_t)row*width];
    uint16_t column=0U;
    while(column<HUD_SIZE) {
      uint16_t count=1U,color=line[column];
      while((uint32_t)column+count<HUD_SIZE && line[column+count]==color) { count++; }
      if(!HudPresent_Append(count,color)) {
        s_hud_stats.overflow_frames++;HudPresent_Invalidate(DISPLAY_HUD_FRAME_OVERFLOW);return DISPLAY_HUD_CAPTURE_ERROR;
      }
      column=(uint16_t)(column+count);
    }
    s_hud_stats.captured_rows++;
  }
  if(s_hud_stats.captured_rows==HUD_SIZE) {
    s_hud_stats.captured_frames++;s_hud_stats.state=DISPLAY_HUD_FRAME_READY;return DISPLAY_HUD_CAPTURE_READY;
  }
  return DISPLAY_HUD_CAPTURE_MORE;
#else
  (void)x;(void)y;(void)width;(void)height;(void)pixels;return DISPLAY_HUD_CAPTURE_ERROR;
#endif
}

/** @brief 完整预检后本次调用集中写258行；不等待任务周期、不关中断。 */
uint8_t Display_HudPresent_CommitFrame(void)
{
#if DISPLAY_HUD_DIFF_ENABLE
  uint16_t offset=0U,row;
  if(s_hud_stats.state!=DISPLAY_HUD_FRAME_READY || !Display_Ssd1963_IsReady() || !HudPresent_CheckStream()) {
    if(Display_HudPresent_IsBuilding()) {
      s_hud_stats.error_frames++;HudPresent_Invalidate(DISPLAY_HUD_FRAME_ERROR);
    }
    return 0U;
  }
  for(row=0U;row<HUD_SIZE;row++) {
    uint16_t column=0U;
    if(!Display_Ssd1963_IsReady()) {
      s_hud_stats.error_frames++;HudPresent_Invalidate(DISPLAY_HUD_FRAME_ERROR);return 0U;
    }
    while(column<HUD_SIZE) {
      uint16_t count,color,i;
      HudPresent_Read(offset,&count,&color);offset=(uint16_t)(offset+HUD_RUN_BYTES);
      for(i=0U;i<count;i++) { s_hud_row[column+i]=color; }
      column=(uint16_t)(column+count);
    }
    Display_Ssd1963_FlushPixels(s_hud_present.frame_x,(uint16_t)(s_hud_present.frame_y+row),HUD_SIZE,1U,s_hud_row);
  }
  if(!Display_Ssd1963_IsReady()) {
    s_hud_stats.error_frames++;HudPresent_Invalidate(DISPLAY_HUD_FRAME_ERROR);return 0U;
  }
  s_hud_stats.committed_frames++;s_hud_stats.state=DISPLAY_HUD_FRAME_COMMITTED;
  s_hud_stats.captured_rows=0U;s_hud_present.encoded_bytes=0U;return 1U;
#else
  return 0U;
#endif
}

/** @brief 显式中止只累计一次，不通过清零大缓存延长显示路径。 */
void Display_HudPresent_AbortFrame(void)
{
#if DISPLAY_HUD_DIFF_ENABLE
  if(Display_HudPresent_IsBuilding()) { HudPresent_Invalidate(DISPLAY_HUD_FRAME_ABORTED); }
#endif
}

/** @brief 同任务复制诊断快照；关闭配置返回全零。 */
void Display_HudPresent_GetStats(Display_HudPresentStats_t *dest)
{
  if(!dest) { return; }
#if DISPLAY_HUD_DIFF_ENABLE
  *dest=s_hud_stats;
#else
  memset(dest,0,sizeof(*dest));
#endif
}
#if defined(__CC_ARM)
#pragma pop
#endif
