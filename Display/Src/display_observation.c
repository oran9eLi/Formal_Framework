/** @file display_observation.c
 * @brief Display两路固定缓存线图，无LV_USE_CHART和新增堆对象。
 * @details 两路独立纵轴自动范围；横轴实际采样槽时间，缺测不连接。
 */
#include "display_observation.h"
#include <string.h>
static lv_point_t s_coords[2][APP_OBSERVATION_POINTS];
static uint8_t s_valid[2][8],s_segment[2][8];
static uint32_t s_session,s_version;
static uint16_t s_count;
/** @brief 读取固定bitset。 */
static uint8_t ObservationBit(const uint8_t *bits,uint16_t i) { return (uint8_t)((bits[i/8U]>>(i%8U))&1U); }
/** @brief 双遍扫描只复制单点；不持有整个环临界区，不创建控件。 */
void Display_ObservationUpdate(const App_ObservationStatus_t *st,lv_obj_t *object)
{
 App_ObservationPoint_t point;App_ObservationStatus_t after;
 float low[2]={0.0f,0.0f},high[2]={0.0f,0.0f};uint8_t seen[2]={0U,0U};uint16_t i;uint8_t ch;
 uint32_t first=0U,last=0U;
 if(st->session_id!=s_session) { s_count=0U;s_session=st->session_id;s_version=UINT32_MAX; }
 if(st->frozen||st->ring_version==s_version) { return; }
 for(i=0U;i<st->point_count;i++) {
  if(App_CopyObservationPoint(st->session_id,i,&point)!=PX4LITE_OK) { return; }
  if(!i) { first=point.time_ms; }last=point.time_ms;
  for(ch=0U;ch<2U;ch++) { if(point.valid_mask&(1U<<ch)) { if(!seen[ch]) { low[ch]=high[ch]=point.value[ch];seen[ch]=1U; }else { if(point.value[ch]<low[ch]) { low[ch]=point.value[ch]; }if(point.value[ch]>high[ch]) { high[ch]=point.value[ch]; } } } }
 }
 memset(s_valid,0,sizeof(s_valid));memset(s_segment,0,sizeof(s_segment));
 for(i=0U;i<st->point_count;i++) {
  if(App_CopyObservationPoint(st->session_id,i,&point)!=PX4LITE_OK) { s_count=0U;return; }
  for(ch=0U;ch<2U;ch++) { float range=high[ch]-low[ch];
   s_coords[ch][i].x=(lv_coord_t)(6U+((last!=first)?(uint64_t)(point.time_ms-first)*348U/(last-first):0U));
   s_coords[ch][i].y=(lv_coord_t)((ch?112:8)+((range>0.000001f)?(high[ch]-point.value[ch])*86.0f/range:43.0f));
   if(point.valid_mask&(1U<<ch)) { s_valid[ch][i/8U]|=(uint8_t)(1U<<(i%8U)); }
   if(point.segment_mask&(1U<<ch)) { s_segment[ch][i/8U]|=(uint8_t)(1U<<(i%8U)); }
  }
 }
 (void)App_CopyObservation(&after,0U);
 if(after.session_id!=st->session_id||after.ring_version!=st->ring_version) { s_count=0U;lv_obj_invalidate(object);return; }
 s_count=st->point_count;s_version=st->ring_version;lv_obj_invalidate(object);
}
/** @brief 仅用缓存绘图，Freeze后其他控件重绘也保持旧坐标。 */
void Display_ObservationDraw(lv_obj_t *object,lv_draw_ctx_t *dc)
{
 lv_area_t co;lv_draw_line_dsc_t line;lv_point_t a,b;uint8_t ch;uint16_t i;
 lv_obj_get_coords(object,&co);lv_draw_line_dsc_init(&line);line.width=1;line.color=lv_color_hex(0x294052);
 a.x=co.x1+6;a.y=co.y1+102;b.x=co.x1+354;b.y=a.y;lv_draw_line(dc,&line,&a,&b);
 for(ch=0U;ch<2U;ch++) { line.color=lv_color_hex(ch?0xF2C14E:0x28BDCD);line.width=2;
  for(i=1U;i<s_count;i++) { if(ObservationBit(s_valid[ch],i)&&ObservationBit(s_valid[ch],i-1U)&&!ObservationBit(s_segment[ch],i)) { a.x=co.x1+s_coords[ch][i-1U].x;a.y=co.y1+s_coords[ch][i-1U].y;b.x=co.x1+s_coords[ch][i].x;b.y=co.y1+s_coords[ch][i].y;lv_draw_line(dc,&line,&a,&b); } }
 }
}
