/**
 * @file test_hud_present.c
 * @brief HUD整帧离屏捕获、无损集中提交、容量与生命周期回归。
 * @details 独立维护原GRAM期望值，捕获每步验证零写屏；不代表实屏帧率。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define DISPLAY_HUD_PRESENT_TEST_CCM_RESERVE 1
#include "../../Display/Src/display_hud_present.c"

#define SCREEN_PIXELS (800U*480U)
#define STRIP_PIXELS (258U*16U)
static uint16_t s_gram[SCREEN_PIXELS],s_expected[SCREEN_PIXELS],s_input[STRIP_PIXELS];
static uint8_t s_ready=1U;
static uint32_t s_calls,s_written,s_fail_after;

/** @brief 原驱动替身，保留裁剪与输入跨度；可模拟提交中失效。 */
void Display_Ssd1963_FlushPixels(uint16_t x,uint16_t y,uint16_t width,uint16_t height,const uint16_t *pixels)
{
  uint16_t cw=width,ch=height,row;
  s_calls++;
  if(!s_ready || !pixels || !width || !height || x>=800U || y>=480U) { return; }
  if((uint32_t)x+cw>800U) { cw=(uint16_t)(800U-x); }
  if((uint32_t)y+ch>480U) { ch=(uint16_t)(480U-y); }
  for(row=0U;row<ch;row++) {
    memcpy(&s_gram[(uint32_t)(y+row)*800U+x],&pixels[(uint32_t)row*width],(uint32_t)cw*2U);
  }
  s_written+=(uint32_t)cw*ch;
  if(s_fail_after && s_calls==s_fail_after) { s_ready=0U; }
}

/** @brief 仅报告LCD就绪事实，不自动恢复。 */
uint8_t Display_Ssd1963_IsReady(void) { return s_ready; }

#if DISPLAY_HUD_DIFF_ENABLE
/** @brief 输入含任意RGB565与短run，独立于生产编码器，覆盖旧调色板之外的原色。 */
static uint16_t FramePixel(uint16_t x,uint16_t y,uint16_t frame)
{
  uint16_t split=(uint16_t)(68U+(y+frame)%99U);
  if(x<split) { return (uint16_t)(0x1401U+frame); }
  if(x<split+5U) { return (uint16_t)(0x2060U+y*3U+x+frame); }
  if(x<235U) { return (uint16_t)(0xA381U+frame); }
  return (uint16_t)(0x17EFU+y+frame);
}

/** @brief 生成连续完整输入行和独立最终期望，不使用编码器私有缓冲。 */
static void FillStrip(uint16_t x,uint16_t y,uint16_t local_y,uint16_t height,uint16_t frame)
{
  uint16_t row,col;
  assert(height<=16U);
  for(row=0U;row<height;row++) {
    for(col=0U;col<258U;col++) {
      uint16_t p=FramePixel(col,(uint16_t)(local_y+row),frame);
      s_input[(uint32_t)row*258U+col]=p;
      s_expected[(uint32_t)(y+local_y+row)*800U+x+col]=p;
    }
  }
}

/** @brief 前40KB保留哨兵，证明不把LVGL池当作帧缓存清除。 */
static void CheckLvglReservation(void)
{
  uint32_t i;
  for(i=0U;i<40960U;i++) { assert(s_hud_ccm_reservation[i]==0xA5U); }
}

/** @brief 开始前保存可见画面，逐条带无任何GRAM写，完整后一次同步集中提交。 */
static void CaptureFrame(uint16_t x,uint16_t y,uint16_t frame)
{
  uint32_t before_calls=s_calls,before_written=s_written;
  uint16_t row;
  static uint16_t visible_before[SCREEN_PIXELS];
  Display_HudPresentStats_t stats;
  memcpy(visible_before,s_gram,sizeof(s_gram));
  assert(Display_HudPresent_BeginFrame(x,y,258U,258U)==1U);
  assert(Display_HudPresent_IsBuilding());
  for(row=0U;row<258U;row=(uint16_t)(row+16U)) {
    uint16_t height=(uint16_t)((258U-row)>16U?16U:258U-row);
    Display_HudCaptureResult_t result;
    FillStrip(x,y,row,height,frame);
    result=Display_HudPresent_Capture(x,(uint16_t)(y+row),258U,height,s_input);
    assert(result==((row+height==258U)?DISPLAY_HUD_CAPTURE_READY:DISPLAY_HUD_CAPTURE_MORE));
    assert(Display_HudPresent_IsBuilding());
    assert(s_calls==before_calls && s_written==before_written);
    assert(memcmp(s_gram,visible_before,sizeof(s_gram))==0);
    /* 输入存储随后立即复用，提交仍必须完全无损。 */
    memset(s_input,0xEE,sizeof(s_input));
    Display_HudPresent_GetStats(&stats);
    assert(stats.captured_rows==row+height);
  }
  assert(Display_HudPresent_CommitFrame()==1U);
  assert(s_calls==before_calls+258U && s_written==before_written+258U*258U);
  assert(memcmp(s_gram,s_expected,sizeof(s_gram))==0);
  assert(!Display_HudPresent_IsBuilding());
  assert(Display_HudPresent_CommitFrame()==0U && s_calls==before_calls+258U);
  Display_HudPresent_GetStats(&stats);
  assert(stats.state==DISPLAY_HUD_FRAME_COMMITTED && !stats.captured_rows);
  assert(stats.last_encoded_bytes>0U && stats.last_encoded_bytes<=24576U);
  CheckLvglReservation();
}

/** @brief 身份/开始边界及不完整提交都拒绝，不泄露前几行。 */
static void TestIncompleteOrderAndArguments(void)
{
  uint32_t calls=s_calls;
  Display_HudPresentStats_t a,b;
  assert(!Display_HudPresent_BeginFrame(543U,64U,258U,258U));
  assert(!Display_HudPresent_BeginFrame(26U,223U,258U,258U));
  assert(!Display_HudPresent_BeginFrame(26U,64U,257U,258U));
  assert(Display_HudPresent_BeginFrame(26U,64U,258U,258U));
  assert(Display_HudPresent_Capture(26U,64U,258U,16U,s_input)==DISPLAY_HUD_CAPTURE_MORE);
  assert(!Display_HudPresent_CommitFrame());
  assert(!Display_HudPresent_IsBuilding() && s_calls==calls);
  Display_HudPresent_GetStats(&a);
  Display_HudPresent_AbortFrame();Display_HudPresent_AbortFrame();
  Display_HudPresent_GetStats(&b);assert(a.aborted_frames==b.aborted_frames);
  assert(Display_HudPresent_BeginFrame(26U,64U,258U,258U));
  assert(Display_HudPresent_Capture(26U,65U,258U,1U,s_input)==DISPLAY_HUD_CAPTURE_ERROR);
  assert(!Display_HudPresent_CommitFrame() && !Display_HudPresent_IsBuilding());
  assert(Display_HudPresent_BeginFrame(26U,64U,258U,258U));
  assert(Display_HudPresent_Capture(27U,64U,258U,1U,s_input)==DISPLAY_HUD_CAPTURE_ERROR);
  assert(Display_HudPresent_BeginFrame(26U,64U,258U,258U));
  assert(Display_HudPresent_Capture(26U,64U,257U,1U,s_input)==DISPLAY_HUD_CAPTURE_ERROR);
  assert(Display_HudPresent_BeginFrame(26U,64U,258U,258U));
  assert(Display_HudPresent_Capture(26U,64U,258U,0U,s_input)==DISPLAY_HUD_CAPTURE_ERROR);
  assert(Display_HudPresent_BeginFrame(26U,64U,258U,258U));
  assert(Display_HudPresent_Capture(26U,64U,258U,1U,0)==DISPLAY_HUD_CAPTURE_ERROR);
  assert(Display_HudPresent_BeginFrame(26U,64U,258U,258U));
  assert(Display_HudPresent_Capture(26U,64U,258U,259U,s_input)==DISPLAY_HUD_CAPTURE_ERROR);
  assert(s_calls==calls);CheckLvglReservation();
}

/** @brief 恶意高熵输入精确触顶24KB后整帧撤销，随后可重新开始成功帧。 */
static void TestOverflow(void)
{
  uint32_t calls=s_calls,i;
  Display_HudPresentStats_t a,b;
  for(i=0U;i<STRIP_PIXELS;i++) { s_input[i]=(uint16_t)(i&1U); }
  Display_HudPresent_GetStats(&a);
  assert(Display_HudPresent_BeginFrame(26U,64U,258U,258U));
  assert(Display_HudPresent_Capture(26U,64U,258U,16U,s_input)==DISPLAY_HUD_CAPTURE_MORE);
  assert(Display_HudPresent_Capture(26U,80U,258U,16U,s_input)==DISPLAY_HUD_CAPTURE_ERROR);
  assert(s_calls==calls && !Display_HudPresent_IsBuilding() && !Display_HudPresent_CommitFrame());
  Display_HudPresent_GetStats(&b);
  assert(b.state==DISPLAY_HUD_FRAME_OVERFLOW && b.overflow_frames==a.overflow_frames+1U);
  assert(b.aborted_frames==a.aborted_frames+1U && b.last_encoded_bytes==24576U && !b.captured_rows);
  CheckLvglReservation();
}

/** @brief 同一身份保留帧，配色/位置/禁用/重置/新Begin均撤销旧在途帧。 */
static void TestIdentityAndAbort(void)
{
  uint32_t calls=s_calls;
  Display_HudPresent_Configure(1U,1U,26U,64U,258U,258U);
  assert(Display_HudPresent_BeginFrame(26U,64U,258U,258U));
  Display_HudPresent_Configure(1U,1U,26U,64U,258U,258U);
  assert(Display_HudPresent_IsBuilding());
  Display_HudPresent_Configure(1U,0U,26U,64U,258U,258U);
  assert(!Display_HudPresent_IsBuilding() && !Display_HudPresent_CommitFrame());
  assert(Display_HudPresent_BeginFrame(26U,64U,258U,258U));
  Display_HudPresent_Configure(1U,0U,30U,64U,258U,258U);assert(!Display_HudPresent_IsBuilding());
  assert(Display_HudPresent_BeginFrame(26U,64U,258U,258U));
  Display_HudPresent_Configure(0U,0U,30U,64U,258U,258U);assert(!Display_HudPresent_IsBuilding());
  assert(Display_HudPresent_BeginFrame(26U,64U,258U,258U));
  Display_HudPresent_Reset();assert(!Display_HudPresent_IsBuilding() && !Display_HudPresent_CommitFrame());
  assert(Display_HudPresent_BeginFrame(26U,64U,258U,258U));
  assert(Display_HudPresent_BeginFrame(30U,64U,258U,258U));
  assert(Display_HudPresent_Capture(26U,64U,258U,1U,s_input)==DISPLAY_HUD_CAPTURE_ERROR);
  assert(s_calls==calls);CheckLvglReservation();
}

/** @brief 捕获前/LCD恢复前拒绝；提交中失效不再继续或记为完成，下一帧可重试。 */
static void TestReadyAndCommitFailure(void)
{
  uint32_t calls=s_calls;
  uint16_t row;
  Display_HudPresentStats_t a,b;
  s_ready=0U;assert(!Display_HudPresent_BeginFrame(26U,64U,258U,258U));
  s_ready=1U;assert(Display_HudPresent_BeginFrame(26U,64U,258U,258U));
  s_ready=0U;assert(Display_HudPresent_Capture(26U,64U,258U,1U,s_input)==DISPLAY_HUD_CAPTURE_ERROR);
  assert(!Display_HudPresent_CommitFrame() && s_calls==calls);
  s_ready=1U;assert(Display_HudPresent_BeginFrame(26U,64U,258U,258U));
  for(row=0U;row<258U;row++) {
    memset(s_input,0xA7,258U*2U);
    assert(Display_HudPresent_Capture(26U,(uint16_t)(64U+row),258U,1U,s_input)==
        ((row==257U)?DISPLAY_HUD_CAPTURE_READY:DISPLAY_HUD_CAPTURE_MORE));
  }
  s_ready=0U;assert(!Display_HudPresent_CommitFrame() && s_calls==calls);
  s_ready=1U;assert(!Display_HudPresent_CommitFrame());
  assert(Display_HudPresent_BeginFrame(26U,64U,258U,258U));
  for(row=0U;row<258U;row++) {
    assert(Display_HudPresent_Capture(26U,(uint16_t)(64U+row),258U,1U,s_input)==
        ((row==257U)?DISPLAY_HUD_CAPTURE_READY:DISPLAY_HUD_CAPTURE_MORE));
  }
  Display_HudPresent_GetStats(&a);s_fail_after=s_calls+7U;
  assert(!Display_HudPresent_CommitFrame() && s_calls==calls+7U && !Display_HudPresent_IsBuilding());
  Display_HudPresent_GetStats(&b);assert(a.committed_frames==b.committed_frames && b.state==DISPLAY_HUD_FRAME_ERROR);
  s_fail_after=0U;s_ready=1U;assert(!Display_HudPresent_CommitFrame());
  memcpy(s_expected,s_gram,sizeof(s_gram));CheckLvglReservation();
}

/** @brief 预检抓出零长、跨行、截断与尾多余run，拒绝在任何写屏前失败。 */
static void TestCorruptStream(void)
{
  unsigned fault;
  uint32_t calls=s_calls;
  for(fault=0U;fault<6U;fault++) {
    uint16_t row;
    assert(Display_HudPresent_BeginFrame(26U,64U,258U,258U));
    memset(s_input,0x61,258U*2U);
    for(row=0U;row<258U;row++) {
      assert(Display_HudPresent_Capture(26U,(uint16_t)(64U+row),258U,1U,s_input)==
          ((row==257U)?DISPLAY_HUD_CAPTURE_READY:DISPLAY_HUD_CAPTURE_MORE));
    }
    if(fault==0U) { HUD_FRAME_CACHE[0]=0U;HUD_FRAME_CACHE[1]=0U; }
    if(fault==1U) { HUD_FRAME_CACHE[0]=3U;HUD_FRAME_CACHE[1]=1U; } /* 259像素跨行。 */
    if(fault==2U) { s_hud_present.encoded_bytes--; }
    if(fault==3U) { s_hud_present.encoded_bytes=(uint16_t)(s_hud_present.encoded_bytes+4U); }
    if(fault==4U) { s_hud_present.encoded_bytes=24580U; }
    if(fault==5U) { s_hud_present.encoded_bytes=0U; }
    assert(!Display_HudPresent_CommitFrame() && !Display_HudPresent_IsBuilding() && s_calls==calls);
  }
  CheckLvglReservation();
}

/** @brief 在恰好6144个run时允许完整提交，超过一个run则拒绝，不浪费边界容量。 */
static void TestExactCapacity(void)
{
  uint16_t row,col;
  uint32_t calls=s_calls;
  assert(Display_HudPresent_BeginFrame(26U,64U,258U,258U));
  for(row=0U;row<258U;row++) {
    /* 208行24run、50行23run = 6142；首2行各再加1run使总数6144。 */
    uint16_t runs=(uint16_t)(row<208U?24U:23U);
    if(row<2U) { runs++; }
    for(col=0U;col<258U;col++) {
      uint16_t p=(uint16_t)(col<runs-1U?col+1U:0x7415U);
      s_input[col]=p;s_expected[(uint32_t)(64U+row)*800U+26U+col]=p;
    }
    assert(Display_HudPresent_Capture(26U,(uint16_t)(64U+row),258U,1U,s_input)==
        ((row==257U)?DISPLAY_HUD_CAPTURE_READY:DISPLAY_HUD_CAPTURE_MORE));
    assert(s_calls==calls);
  }
  assert(s_hud_present.encoded_bytes==24576U);
  assert(Display_HudPresent_CommitFrame() && memcmp(s_gram,s_expected,sizeof(s_gram))==0);
  CheckLvglReservation();
}
#endif

/** @brief 正常bulk无论开关配置均保持单次同步调用和像素值。 */
static void TestBulkUnchanged(void)
{
  uint32_t calls=s_calls;
  unsigned i;
  for(i=0U;i<16U;i++) { s_input[i]=(uint16_t)(0x3100U+i); }
  Display_HudPresent_Flush(600U,7U,4U,4U,s_input);
  for(i=0U;i<4U;i++) { memcpy(&s_expected[(7U+i)*800U+600U],&s_input[i*4U],8U); }
  assert(s_calls==calls+1U && memcmp(s_gram,s_expected,sizeof(s_gram))==0);
  Display_HudPresent_Flush(800U,0U,1U,1U,s_input);assert(s_calls==calls+2U);
}

/** @brief 启用与关闭两配置的独立入口。 */
int main(void)
{
  memset(s_gram,0x29,sizeof(s_gram));memcpy(s_expected,s_gram,sizeof(s_gram));
#if DISPLAY_HUD_DIFF_ENABLE
  memset(s_hud_ccm_reservation,0xA5,sizeof(s_hud_ccm_reservation));
  Display_HudPresent_Reset();CheckLvglReservation();
  TestBulkUnchanged();CaptureFrame(26U,64U,1U);
  TestIncompleteOrderAndArguments();TestOverflow();TestIdentityAndAbort();TestReadyAndCommitFailure();
  TestCorruptStream();CaptureFrame(30U,70U,93U);TestExactCapacity();TestBulkUnchanged();
  puts("HUD staging: zero GRAM during capture / complete exact RLE / order / overflow / capacity / lifecycle / corrupt precheck / CCM PASS");
#else
  Display_HudPresentStats_t stats;
  assert(!Display_HudPresent_BeginFrame(26U,64U,258U,258U));
  assert(!Display_HudPresent_IsBuilding() && !Display_HudPresent_CommitFrame());
  assert(Display_HudPresent_Capture(26U,64U,258U,1U,s_input)==DISPLAY_HUD_CAPTURE_ERROR);
  Display_HudPresent_Configure(1U,1U,26U,64U,258U,258U);Display_HudPresent_AbortFrame();Display_HudPresent_Reset();
  memset(&stats,0xAA,sizeof(stats));Display_HudPresent_GetStats(&stats);assert(!stats.started_frames && !stats.state);
  Display_HudPresent_GetStats(0);TestBulkUnchanged();
  puts("HUD staging disabled: no frame cache / exact original bulk path PASS");
#endif
  return 0;
}
