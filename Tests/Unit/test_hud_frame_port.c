/**
 * @file test_hud_frame_port.c
 * @brief 真实LVGL端口/后备帧的提交边界、旁路区域和失败整帧抑制回归。
 * @details 模拟GRAM与刷新队列，不模拟实屏帧率；保留真实端口与RLE生产实现。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef int BaseType_t;
typedef void *TaskHandle_t;
#include "../../Display/Src/display_hud_present.c"
#include "../../Display/Src/lv_port_disp.c"

static lv_disp_t s_test_disp;
static lv_disp_drv_t s_test_driver;
static uint16_t s_pixels[800U*16U],s_gram[800U*480U];
static uint32_t s_clock_us,s_calls,s_ready_calls;
static uint8_t s_lcd_ready=1U;

/** @brief 所有同步提交立即写到模拟屏幕，用于检出准备阶段的像素泄露。 */
void Display_Ssd1963_FlushPixels(uint16_t x,uint16_t y,uint16_t width,uint16_t height,const uint16_t *pixels)
{
  uint16_t row;
  s_calls++;
  assert(x+width<=800U && y+height<=480U && pixels);
  if(!s_lcd_ready) { return; }
  for(row=0U;row<height;row++) { memcpy(&s_gram[(uint32_t)(y+row)*800U+x],&pixels[(uint32_t)row*width],width*2U); }
}
/** @brief 主机只提供LCD就绪事实，不调用硬件。 */
uint8_t Display_Ssd1963_IsReady(void) { return s_lcd_ready; }
/** @brief 当前生产端口只读队列是否清空；不在测试中调用LVGL渲染器。 */
lv_disp_t *lv_disp_get_default(void) { return &s_test_disp; }
/** @brief 每个被捕获或普通提交的条带都必须归还LVGL源缓冲所有权。 */
void lv_disp_flush_ready(lv_disp_drv_t *driver) { assert(driver==&s_test_driver);s_ready_calls++; }
/** @brief 单调微秒计时检出最终集中提交是否进入统计。 */
uint32_t App_RuntimeGetUs(void) { s_clock_us+=17U;return s_clock_us; }

/** @brief 复位端口批次及模拟GRAM，值不会与生成帧混淆。 */
static void Reset(void)
{
  LvPortDisp_AbortHudFrame();memset(&s_test_disp,0,sizeof(s_test_disp));
  memset(s_gram,0x33,sizeof(s_gram));s_calls=0U;s_ready_calls=0U;s_clock_us=0U;
  s_lcd_ready=1U;LvPortDisp_BeginStep(0U,2000U);
}
/** @brief 明确生成长短连续原色run，包含16bit最高位，无调色板近似。 */
static uint16_t Pixel(uint16_t x,uint16_t y)
{ return (uint16_t)(0x8000U+(y%5U)*0x101U+x/31U); }
/** @brief 填一个输入条带，可构造不可压缩数据测试整帧丢弃。 */
static void Fill(uint16_t width,uint16_t y,uint16_t rows,uint8_t noise)
{
  uint16_t row,col;
  for(row=0U;row<rows;row++) { for(col=0U;col<width;col++) {
    s_pixels[(uint32_t)row*width+col]=noise?(uint16_t)((y+row)*997U+col):Pixel(col,(uint16_t)(y+row));
  } }
}
/** @brief 推进真实端口flush，源缓冲返回后可立即复用。 */
static void Flush(uint16_t x,uint16_t y,uint16_t width,uint16_t height)
{
  lv_area_t area={(lv_coord_t)x,(lv_coord_t)y,(lv_coord_t)(x+width-1U),(lv_coord_t)(y+height-1U)};
  LvPortDisp_Flush(&s_test_driver,&area,(lv_color_t *)s_pixels);
}
/** @brief 在完整批次结束之前HUD区域必须仍为旧画面。 */
static void AssertOldHud(void)
{
  uint16_t x,y;
  for(y=64U;y<322U;y++) { for(x=26U;x<284U;x++) { assert(s_gram[(uint32_t)y*800U+x]==0x3333U); } }
}
/** @brief 跨17次刷新不写可见圆圈，其他区域照旧；队列清空后一次完成呈现。 */
static void TestCompleteAndOtherRegion(void)
{
  uint16_t row,x,y;uint32_t before,total,maximum;
  Reset();assert(LvPortDisp_BeginHudFrame(26U,64U,258U,258U));s_test_disp.inv_p=1U;
  Fill(20U,0U,2U,0U);Flush(600U,20U,20U,2U);assert(s_calls==1U);
  assert(s_gram[20U*800U+600U]==Pixel(0U,0U));
  for(row=0U;row<258U;row+=16U) {
    uint16_t n=(uint16_t)(258U-row>16U?16U:258U-row);
    Fill(258U,row,n,0U);Flush(26U,(uint16_t)(64U+row),258U,n);
    AssertOldHud();assert(s_calls==1U && LvPortDisp_FinishHudFrame()==0U);
  }
  assert(s_ready_calls==18U);before=s_calls;s_test_disp.inv_p=0U;
  assert(LvPortDisp_FinishHudFrame()==1U && s_calls-before==258U);
  for(y=0U;y<258U;y++) { for(x=0U;x<258U;x++) { assert(s_gram[(uint32_t)(y+64U)*800U+x+26U]==Pixel(x,y)); } }
  LvPortDisp_GetFlushTime(&total,&maximum);assert(total>0U && maximum>0U);
  assert(!s_hud_batch.active && LvPortDisp_FinishHudFrame()==0U);
}
/** @brief 宽失效矩形仍可旁路左右区域，未完整捕获的圆圈绝不提交。 */
static void TestMixedAreaAndMissingRows(void)
{
  Reset();assert(LvPortDisp_BeginHudFrame(26U,64U,258U,258U));s_test_disp.inv_p=1U;
  Fill(800U,0U,16U,0U);Flush(0U,64U,800U,16U);
  AssertOldHud();assert(s_calls==32U);
  assert(s_gram[64U*800U+0U]==Pixel(0U,0U));assert(s_gram[64U*800U+799U]==Pixel(799U,0U));
  s_test_disp.inv_p=0U;assert(LvPortDisp_FinishHudFrame()==2U);AssertOldHud();
}
/** @brief 意外窄片和容量失败都吞掉此批次的其余HUD条带，不回退到半帧直写。 */
static void TestFailedBatchSuppression(void)
{
  unsigned mode;uint16_t row;
  for(mode=0U;mode<2U;mode++) {
    Reset();assert(LvPortDisp_BeginHudFrame(26U,64U,258U,258U));s_test_disp.inv_p=1U;
    if(mode==0U) { Fill(10U,0U,1U,0U);Flush(40U,64U,10U,1U); }
    for(row=0U;row<258U;row+=16U) {
      uint16_t n=(uint16_t)(258U-row>16U?16U:258U-row);
      Fill(258U,row,n,(uint8_t)mode);Flush(26U,(uint16_t)(64U+row),258U,n);AssertOldHud();
    }
    assert(s_calls==0U);s_test_disp.inv_p=0U;assert(LvPortDisp_FinishHudFrame()==2U);AssertOldHud();
  }
}
/** @brief 切页撤销即恢复原一次普通提交，不把新页面像素压入旧HUD帧。 */
static void TestAbortAndOrdinaryPath(void)
{
  Reset();assert(LvPortDisp_BeginHudFrame(26U,64U,258U,258U));
  Fill(258U,0U,16U,0U);Flush(26U,64U,258U,16U);assert(s_calls==0U);
  LvPortDisp_AbortHudFrame();Flush(26U,64U,258U,16U);assert(s_calls==1U);
  assert(s_gram[64U*800U+26U]==Pixel(0U,0U));assert(LvPortDisp_FinishHudFrame()==0U);
}
/** @brief 49行只用于恰好HUD宽的待绘制区域；其他区域和混合宽条带仍16行。 */
static void TestHudOnlyBufferCapacity(void)
{
  Reset();s_test_disp.inv_p=1U;s_test_disp.inv_area_joined[0]=0U;
  s_test_disp.inv_areas[0]=(lv_area_t){26,64,283,321};
  LvPortDisp_SelectRows(&s_test_driver);assert(s_test_driver.refr_max_rows==16U);
  assert(LvPortDisp_BeginHudFrame(26U,64U,258U,258U));
  assert(LvPortDisp_Continue(&s_test_driver));assert(s_test_driver.refr_max_rows==49U);
  s_test_disp.inv_areas[0]=(lv_area_t){400,64,799,321};
  LvPortDisp_SelectRows(&s_test_driver);assert(s_test_driver.refr_max_rows==16U);
  s_test_disp.inv_areas[0]=(lv_area_t){0,64,799,321};
  LvPortDisp_SelectRows(&s_test_driver);assert(s_test_driver.refr_max_rows==16U);
  LvPortDisp_AbortHudFrame();
}
/** @brief 验证呈现边界和其他区域生产路径，不将主机零写入当成物理TE同步。 */
int main(void)
{
  TestCompleteAndOtherRegion();TestMixedAreaAndMissingRows();TestFailedBatchSuppression();TestAbortAndOrdinaryPath();
  TestHudOnlyBufferCapacity();
  puts("HUD frame port: precommit GRAM unchanged / queue fence / other areas / full pixels / abort / overflow PASS");
  return 0;
}
