/**
 * @file test_hud_real_render.c
 * @brief 真实LVGL软件绘图与HUD回调的整帧/16及49行拼合一致性、RLE容量检查。
 * @details 只将LVGL内存池地址改为主机静态池；字体、mask、line、arc和label都用生产实现。
 * 主机像素与压缩长度不作为实屏帧率证据。
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef int BaseType_t;
typedef void *TaskHandle_t;
#ifndef HUD_REAL_RENDER_SOURCE
#define HUD_REAL_RENDER_SOURCE "../../Display/Src/display_lvgl.c"
#endif
#include HUD_REAL_RENDER_SOURCE
#include "../../Third_Party/LVGL/src/draw/sw/lv_draw_sw.h"
#include "display_ssd1963.h"

#define TEST_HUD_SIZE 258U
#define TEST_HUD_PIXELS (TEST_HUD_SIZE*TEST_HUD_SIZE)
#define TEST_RLE_CAPACITY 24576U
static lv_color_t s_full[TEST_HUD_PIXELS];
static lv_color_t s_partial[TEST_HUD_PIXELS];
static lv_color_t s_band[TEST_HUD_SIZE*49U];
static lv_color_t s_gram[TEST_HUD_PIXELS],s_golden[TEST_HUD_PIXELS];
static lv_obj_t s_test_obj;
static uint32_t s_max_rle,s_max_runs,s_cases;
static uint32_t s_write_calls,s_golden_cases;
static int s_max_roll,s_max_pitch,s_max_yaw;
static unsigned s_max_valid;
static const char *s_golden_dir;
static uint8_t s_write_golden;

/** @brief 真实提交后端的同步LCD替身，只记录最终GRAM，不提前复制源帧。 */
__attribute__((externally_visible)) void Display_Ssd1963_FlushPixels(uint16_t x,uint16_t y,uint16_t width,uint16_t height,const uint16_t *pixels)
{
  uint16_t row;
  assert(x==26U && y>=64U && width==258U && (uint32_t)y+height<=322U);
  for(row=0U;row<height;row++) { memcpy(&s_gram[(uint32_t)(y+row-64U)*258U],&pixels[(uint32_t)row*width],width*sizeof(lv_color_t)); }
  s_write_calls++;
}

/** @brief 本回归固定LCD在线，故障与容量撤销由专门backend单元用例覆盖。 */
__attribute__((externally_visible)) uint8_t Display_Ssd1963_IsReady(void) { return 1U; }

/** @brief 固定9组保留基线的完整RGB565像素，检查缓存替换没有改变外观。 */
static void GoldenCheck(void)
{
  static const uint16_t selected[]={0U,45U,90U,150U,250U,600U,1100U,1191U,1192U};
  unsigned i;
  for(i=0U;i<sizeof(selected)/sizeof(selected[0]);i++) {
    if(s_cases==selected[i]) {
      char path[1024];FILE *f;
      assert(s_golden_dir!=0);
      assert(snprintf(path,sizeof(path),"%s/frame_%04lu.r565",s_golden_dir,(unsigned long)s_cases)>0);
      f=fopen(path,s_write_golden?"wb":"rb");assert(f!=0);
      if(s_write_golden) { assert(fwrite(s_full,sizeof(s_full),1U,f)==1U); }
      else { assert(fread(s_golden,sizeof(s_golden),1U,f)==1U);assert(memcmp(s_golden,s_full,sizeof(s_full))==0); }
      assert(fclose(f)==0);s_golden_cases++;
    }
  }
}

/** @brief 观察页不属于本测试，任何跨页调用都视为范围错误。 */
void Display_ObservationDraw(lv_obj_t *object,lv_draw_ctx_t *draw_ctx)
{ (void)object;(void)draw_ctx;assert(0); }

/** @brief 背景沿用HUD所在面板RGB565；绘制回调只覆盖其真实负责的像素。 */
static void FillBackground(lv_color_t *pixels,uint32_t count)
{
  uint32_t i;
  lv_color_t color=lv_color_hex(0x12202D);
  for(i=0U;i<count;i++) { pixels[i]=color; }
}

/** @brief 注入同一帧生产显示快照和三角缓存，不使用替代绘图算法。 */
static void SetFrame(int roll,int pitch,int yaw,unsigned valid)
{
  float angle;
  memset(&s_hud,0,sizeof(s_hud));
  s_hud.ready=1U;s_hud.valid=(uint8_t)valid;
  s_hud.roll_deg10=(int16_t)roll;s_hud.pitch_deg10=(int16_t)pitch;s_hud.yaw_deg10=(int16_t)yaw;
  angle=((float)roll/10.0f)*HZ_ROLL_SIGN*(float)(M_PI/180.0);
  s_hud.roll_sin=sinf(angle);s_hud.roll_cos=cosf(angle);
  angle=(((float)yaw/10.0f)*HZ_YAW_SIGN+HZ_YAW_OFFSET_DEG)*(float)(M_PI/180.0);
  s_hud.yaw_sin=sinf(angle);s_hud.yaw_cos=cosf(angle);
  s_current_lvgl_page=DISPLAY_HMI_PAGE_AIRCRAFT;
}

/** @brief 同步调用真实HUD draw-post回调，保留绝对坐标和当前缓冲跨度。 */
static void Draw(lv_draw_sw_ctx_t *ctx,lv_color_t *pixels,lv_area_t *area)
{
  lv_event_t event;
  memset(&event,0,sizeof(event));
  ctx->base_draw.buf=pixels;ctx->base_draw.buf_area=area;ctx->base_draw.clip_area=area;
  event.target=&s_test_obj;event.current_target=&s_test_obj;
  event.code=LV_EVENT_DRAW_POST;event.param=&ctx->base_draw;
  Display_LvglHorizonDrawCb(&event);
}

/** @brief 逐行RGB565精确RLE：每段2B长度和2B色值，与生产流长度核对。 */
static uint32_t RleBytes(const lv_color_t *pixels,uint32_t *runs_out)
{
  uint32_t row,runs=0U;
  for(row=0U;row<TEST_HUD_SIZE;row++) {
    uint32_t col=0U;
    while(col<TEST_HUD_SIZE) {
      uint32_t length=1U;
      uint16_t color=pixels[row*TEST_HUD_SIZE+col].full;
      while(col+length<TEST_HUD_SIZE && pixels[row*TEST_HUD_SIZE+col+length].full==color) { length++; }
      runs++;col+=length;
    }
  }
  *runs_out=runs;
  return runs*4U;
}

/** @brief 每个姿态核对整帧与16/49行分带，真实后端集中提交保真，并记录最坏RLE量。 */
static void RunCase(lv_draw_sw_ctx_t *ctx,int roll,int pitch,int yaw,unsigned valid)
{
  lv_area_t full={26,64,283,321};
  Display_HudPresentStats_t stats;
  uint32_t row,runs,bytes,mode;
  SetFrame(roll,pitch,yaw,valid);
  FillBackground(s_full,TEST_HUD_PIXELS);Draw(ctx,s_full,&full);
  GoldenCheck();
  for(mode=0U;mode<2U;mode++) {
    uint32_t lines=mode==0U?16U:49U;
    memset(s_gram,0xA7,sizeof(s_gram));s_write_calls=0U;
    assert(Display_HudPresent_BeginFrame(26U,64U,258U,258U));
    assert(Display_HudPresent_IsBuilding() && s_write_calls==0U);
    for(row=0U;row<TEST_HUD_SIZE;row+=lines) {
      uint32_t h=TEST_HUD_SIZE-row<lines?TEST_HUD_SIZE-row:lines;
      lv_area_t area={26,(lv_coord_t)(64U+row),283,(lv_coord_t)(64U+row+h-1U)};
      FillBackground(s_band,TEST_HUD_SIZE*h);Draw(ctx,s_band,&area);
      memcpy(&s_partial[row*TEST_HUD_SIZE],s_band,TEST_HUD_SIZE*h*sizeof(lv_color_t));
      assert(Display_HudPresent_Capture(26U,(uint16_t)(64U+row),258U,(uint16_t)h,(const uint16_t *)s_band)==
          (row+h==258U?DISPLAY_HUD_CAPTURE_READY:DISPLAY_HUD_CAPTURE_MORE));
      assert(s_write_calls==0U);
    }
    assert(Display_HudPresent_CommitFrame());assert(!Display_HudPresent_IsBuilding());
    assert(s_write_calls>0U && memcmp(s_gram,s_full,sizeof(s_full))==0);
    if(memcmp(s_full,s_partial,sizeof(s_full))!=0) {
      uint32_t i;
      for(i=0U;i<TEST_HUD_PIXELS;i++) {
        if(s_full[i].full!=s_partial[i].full) {
          fprintf(stderr,"HUD mismatch r=%d p=%d y=%d valid=%u lines=%lu x=%lu y=%lu full=%04x band=%04x\n",roll,pitch,yaw,valid,(unsigned long)lines,(unsigned long)(i%258U),(unsigned long)(i/258U),(unsigned)s_full[i].full,(unsigned)s_partial[i].full);break;
        }
      }
      assert(0);
    }
  }
  bytes=RleBytes(s_full,&runs);s_cases++;
  Display_HudPresent_GetStats(&stats);
  assert(stats.last_encoded_bytes==bytes && stats.overflow_frames==0U && stats.error_frames==0U);
  if(bytes>s_max_rle) { s_max_rle=bytes;s_max_runs=runs;s_max_roll=roll;s_max_pitch=pitch;s_max_yaw=yaw;s_max_valid=valid; }
  /* 生产流无额外行索引，完整流必须落入CCM尾24KB。 */
  assert(bytes<=TEST_RLE_CAPACITY);
}

/** @brief 覆盖极值、跨界、亚像素角度、随机姿态及无效盘，输出容量证据。 */
int main(int argc,char **argv)
{
  lv_draw_sw_ctx_t ctx;
  lv_disp_drv_t drv;
  lv_disp_t disp;
  int roll,pitch,yaw;
  unsigned i;
  uint32_t random=0x10203040UL;
  assert(argc==3);s_golden_dir=argv[1];s_write_golden=(uint8_t)(strcmp(argv[2],"write")==0);
  lv_init();lv_disp_drv_init(&drv);drv.hor_res=800;drv.ver_res=480;
  memset(&disp,0,sizeof(disp));disp.driver=&drv;_lv_refr_set_disp_refreshing(&disp);
  lv_draw_sw_init_ctx(&drv,&ctx.base_draw);
  memset(&s_test_obj,0,sizeof(s_test_obj));s_test_obj.coords.x1=26;s_test_obj.coords.y1=64;s_test_obj.coords.x2=283;s_test_obj.coords.y2=321;
  for(roll=-1800;roll<=1800;roll+=300) {
    for(pitch=-900;pitch<=900;pitch+=300) {
      yaw=(roll*7+pitch*3)%3600;RunCase(&ctx,roll,pitch,yaw,1U);
    }
  }
  for(i=0U;i<1100U;i++) {
    random=random*1664525UL+1013904223UL;roll=(int)(random%3601U)-1800;
    random=random*1664525UL+1013904223UL;pitch=(int)(random%1801U)-900;
    random=random*1664525UL+1013904223UL;yaw=(int)(random%3600U)-1800;
    RunCase(&ctx,roll,pitch,yaw,1U);
  }
  RunCase(&ctx,0,0,0,0U);RunCase(&ctx,1799,899,-1799,0U);
  assert(s_golden_cases==9U);
  printf("HUD REAL RENDER: cases=%lu full_vs_16_and_49row=PASS capture_no_visible_write=PASS real_commit_exact=PASS real_encoded_length=PASS golden=%lu/%s max_rle_bytes=%lu runs=%lu capacity=%u worst=(%d,%d,%d,%u)\n",(unsigned long)s_cases,(unsigned long)s_golden_cases,s_write_golden?"write":"compare",(unsigned long)s_max_rle,(unsigned long)s_max_runs,TEST_RLE_CAPACITY,s_max_roll,s_max_pitch,s_max_yaw,s_max_valid);
  return 0;
}
