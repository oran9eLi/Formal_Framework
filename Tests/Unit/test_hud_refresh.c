/**
 * @file test_hud_refresh.c
 * @brief 真实HUD快路径、显示基准与条带刷新一致性主机回归。
 * @details 包含真实Display实现和App显示模型；导航及LVGL仅替换只读/失效边界。
 * 不访问驱动、硬件或任务，不以主机通过代替LCD实测帧率与触摸响应验收。
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef int BaseType_t;
typedef void *TaskHandle_t;
#include "../../Business/Src/app_display_model.c"
#include "../../Display/Src/display_lvgl.c"

static App_NavigationSnapshot_t s_source;
static App_AttitudeStatus_t s_reference;
static Px4Lite_Result_t s_source_result;
static lv_disp_t s_fake_disp;
static lv_obj_t s_fake_hud;
static lv_area_t s_last_inner;
static uint32_t s_now;
static unsigned s_nav_reads, s_reference_reads, s_full_invalidations, s_inner_invalidations;
static unsigned s_show_calls, s_hide_calls;
static uint8_t s_have_display;
static unsigned s_begin_frames;
/** @brief 写屏缓存配置只接受活动姿态圆圈；测试不替代实际GRAM差分验证。 */
void Display_HudPresent_Configure(uint8_t active,uint8_t valid,uint16_t x,uint16_t y,uint16_t width,uint16_t height)
{ assert(active==1U && valid<=1U && x==26U && y==64U && width==258U && height==258U); }
/** @brief 切页撤销写屏缓存不改变数据采样状态。 */
void Display_HudPresent_Reset(void) {}
/** @brief 验证仅在旧脏区清空后开始完整离屏圆圈，避免混入上一个页面。 */
uint8_t LvPortDisp_BeginHudFrame(uint16_t x,uint16_t y,uint16_t width,uint16_t height)
{ assert(s_fake_disp.inv_p==0U && x==26U && y==64U && width==258U && height==258U);s_begin_frames++;return 1U; }
/** @brief 只撤销显示在途帧，不影响传感器测量。 */
void LvPortDisp_AbortHudFrame(void) {}

/** @brief 仅复制已有导航，调用计数检出非姿态页/忙帧期间偷读。 */
Px4Lite_Result_t App_CopyNavigation(App_NavigationSnapshot_t *out, uint32_t now_ms)
{
  assert(now_ms == s_now);
  ++s_nav_reads;
  *out = s_source;
  return s_source_result;
}

/** @brief 原样注入ZERO状态，参考扣除与±180处理仍走生产显示模型。 */
Px4Lite_Result_t App_CopyAttitudeStatus(App_AttitudeStatus_t *out)
{
  ++s_reference_reads;
  *out = s_reference;
  return PX4LITE_OK;
}

/** @brief 模拟LVGL协作刷新剩余脏区，不创建显示设备。 */
lv_disp_t *lv_disp_get_default(void) { return s_have_display ? &s_fake_disp : 0; }

/** @brief 整盘失效立即标记有未完成条带，直到测试显式完成。 */
void lv_obj_invalidate(const lv_obj_t *obj)
{
  assert(obj == &s_fake_hud);
  ++s_full_invalidations;
  s_fake_disp.inv_p = 1U;
}

/** @brief 记录局部脏矩形，检出不必要的外圈刷新。 */
void lv_obj_invalidate_area(const lv_obj_t *obj, const lv_area_t *area)
{
  assert(obj == &s_fake_hud && area != 0);
  ++s_inner_invalidations;
  s_last_inner = *area;
  s_fake_disp.inv_p = 1U;
}

/** @brief 使用生产姿态页的258×258对象几何。 */
void lv_obj_get_coords(const lv_obj_t *obj, lv_area_t *coords)
{
  assert(obj == &s_fake_hud);
  coords->x1 = 26; coords->y1 = 64;
  coords->x2 = 283; coords->y2 = 321;
}

/** @brief 保留旧ApplyValue可能涉及的隐藏操作计数，实际HUD不应被旧数值重画。 */
void lv_obj_add_flag(lv_obj_t *obj, lv_obj_flag_t flag)
{ assert(obj == &s_fake_hud && flag == LV_OBJ_FLAG_HIDDEN); ++s_hide_calls; }
/** @brief 记录显示操作，不创建或分配控件。 */
void lv_obj_clear_flag(lv_obj_t *obj, lv_obj_flag_t flag)
{ assert(obj == &s_fake_hud && flag == LV_OBJ_FLAG_HIDDEN); ++s_show_calls; }

/** @brief 仅由未执行的ApplyValue其他分支引用；本用例不得触碰电机/标签。 */
void lv_label_set_text_static(lv_obj_t *obj, const char *text)
{ (void)obj; (void)text; assert(0); }
/** @brief 关闭非HUD分支所需的链接边界；任何运行调用均失败。 */
void lv_obj_add_state(lv_obj_t *obj, lv_state_t state)
{ (void)obj; (void)state; assert(0); }
/** @brief 本用例从不变更电机按钮权限。 */
void lv_obj_clear_state(lv_obj_t *obj, lv_state_t state)
{ (void)obj; (void)state; assert(0); }
/** @brief 本用例从不读取电机按钮文字。 */
char *lv_label_get_text(const lv_obj_t *obj) { (void)obj; assert(0); return 0; }
/** @brief 本用例从不写电机滑块。 */
void lv_bar_set_value(lv_obj_t *obj, int32_t value, lv_anim_enable_t anim)
{ (void)obj; (void)value; (void)anim; assert(0); }
/** @brief 仅保持未执行的电机分支可链接。 */
bool lv_slider_is_dragged(const lv_obj_t *obj) { (void)obj; assert(0); return false; }

/** @brief 为每个独立场景复位显示私有状态，不依赖场景执行顺序。 */
static void Reset(void)
{
  memset(&s_hud, 0, sizeof(s_hud));
  memset(&s_fake_disp, 0, sizeof(s_fake_disp));
  memset(&s_fake_hud, 0, sizeof(s_fake_hud));
  memset(&s_source, 0, sizeof(s_source));
  memset(&s_reference, 0, sizeof(s_reference));
  memset(s_values, 0, sizeof(s_values));
  memset(s_value_valid, 0, sizeof(s_value_valid));
  memset(s_value_slots, 0, sizeof(s_value_slots));
  memset(s_value_pending, 0, sizeof(s_value_pending));
  s_hud_poll_ms = 0U; s_hud_polled = 0U;
  s_current_lvgl_page = DISPLAY_HMI_PAGE_AIRCRAFT;
  s_workspace_ready = 1U; s_layout_stage = 0U;
  s_observation_frozen = 0U; s_attitude_obj = &s_fake_hud;
  s_nav_reads = s_reference_reads = s_full_invalidations = s_inner_invalidations = 0U;
  s_show_calls = s_hide_calls = 0U;
  s_begin_frames=0U;
  s_have_display = 1U; s_now = 0U; s_source_result = PX4LITE_OK;
}

/** @brief 注入带独立姿态时标的真实量纲快照，角度单位0.01度。 */
static void Source(uint32_t ms, int32_t roll, int32_t pitch, int32_t yaw)
{
  s_source.header.valid = 1U;
  s_source.header.sample_time_ms = ms;
  s_source.header.sequence++;
  s_source.attitude_measurement_ms = ms;
  s_source.valid_mask = PX4LITE_NAV_VALID_ATTITUDE;
  s_source.roll_deg100 = roll; s_source.pitch_deg100 = pitch; s_source.yaw_deg100 = yaw;
}

/** @brief 依次完成上一整帧，允许下一次提交最新显示快照。 */
static void FinishFrame(void) { s_fake_disp.inv_p = 0U; }
/** @brief 显式推进真实毫秒，App替身验证时基一致。 */
static void Service(uint32_t ms) { s_now = ms; Display_LvglHudService(ms); }

/** @brief 验证0ms首帧、50ms节流与完整离屏帧，静止不无谓失效。 */
static void TestCadenceAndDirtyArea(void)
{
  Reset(); Source(0U, 1230, -450, 17900); Service(0U);
  assert(s_nav_reads == 1U && s_hud_polled && s_hud.ready && s_hud.valid);
  assert(s_hud.roll_deg10 == 123 && s_hud.pitch_deg10 == -45 && s_hud.yaw_deg10 == 1790);
  assert(s_full_invalidations == 1U && s_inner_invalidations == 0U);
  FinishFrame(); Source(49U, 2000, -450, 17900); Service(49U);
  assert(s_nav_reads == 1U && s_hud.roll_deg10 == 123);
  Service(50U);
  assert(s_nav_reads == 2U && s_hud.roll_deg10 == 200);
  assert(s_inner_invalidations == 0U && s_full_invalidations == 2U && s_begin_frames==2U);
  FinishFrame(); Service(100U);
  assert(s_nav_reads == 3U && s_inner_invalidations == 0U && s_full_invalidations == 2U);
  Source(150U, 2000, 300, 17900); Service(150U);
  assert(s_inner_invalidations == 0U && s_full_invalidations == 3U && s_begin_frames==3U);
  FinishFrame(); Source(200U, 2000, 300, -17900); Service(200U);
  assert(s_full_invalidations == 4U && s_hud.yaw_deg10 == -1790);
}

/** @brief 旧条带未完成时保持帧和取数时标，清空后直接取最新而不回放旧角度。 */
static void TestBusyFrameAndLegacyValues(void)
{
  uint8_t saved[sizeof(s_hud)];
  unsigned i;
  Reset(); Source(0U, 1000, 2000, 3000); Service(0U);
  memcpy(saved, &s_hud, sizeof(saved));
  for (i = 1U; i <= 10U; ++i) {
    Source(i*50U, (int32_t)i*100, -(int32_t)i*100, (int32_t)i*200);
    Service(i*50U);
    assert(s_nav_reads == 1U && memcmp(saved, &s_hud, sizeof(saved)) == 0);
    assert(s_full_invalidations == 1U && s_inner_invalidations == 0U);
  }
  assert(s_hud_poll_ms == 0U);
  s_values[DISPLAY_HMI_VAR_ROLL] = (uint32_t)(int32_t)-111;
  s_values[DISPLAY_HMI_VAR_PITCH] = 222U;
  s_values[DISPLAY_HMI_VAR_YAW] = 333U;
  Display_LvglApplyValue(DISPLAY_HMI_VAR_ROLL);
  Display_LvglApplyValue(DISPLAY_HMI_VAR_PITCH);
  Display_LvglApplyValue(DISPLAY_HMI_VAR_YAW);
  assert(memcmp(saved, &s_hud, sizeof(saved)) == 0);
  assert(s_full_invalidations == 1U && s_inner_invalidations == 0U);
  FinishFrame(); Service(501U);
  assert(s_nav_reads == 2U && s_hud.roll_deg10 == 100 && s_hud.pitch_deg10 == -100 && s_hud.yaw_deg10 == 200);
  assert(s_full_invalidations == 2U);
}

/** @brief 所有其他页及未完成布局、工作区或显示设备均不启用快路径。 */
static void TestScopeAndInitialPendingFrame(void)
{
  unsigned page;
  Reset(); Source(0U, 100, 200, 300);
  for (page = 0U; page < DISPLAY_HMI_PAGE_COUNT; ++page) {
    if (page == DISPLAY_HMI_PAGE_AIRCRAFT) { continue; }
    s_current_lvgl_page = (Display_HmiPage_t)page; Service(100U);
  }
  assert(s_nav_reads == 0U && s_full_invalidations == 0U && s_inner_invalidations == 0U);
  s_current_lvgl_page = DISPLAY_HMI_PAGE_AIRCRAFT;
  s_layout_stage = 1U; Service(100U); s_layout_stage = 0U;
  s_workspace_ready = 0U; Service(100U); s_workspace_ready = 1U;
  s_attitude_obj = 0; Service(100U); s_attitude_obj = &s_fake_hud;
  s_have_display = 0U; Service(100U); s_have_display = 1U;
  assert(s_nav_reads == 0U);
  s_fake_disp.inv_p = 1U; Service(100U);
  assert(s_nav_reads == 0U && !s_hud.ready && s_begin_frames==0U);
  FinishFrame();Service(100U);
  assert(s_nav_reads == 1U && s_hud.ready && s_hud.valid);
}

/** @brief 独立姿态时间过期必须变灰，新的GNSS组合header不能给旧姿态续期。 */
static void TestValidityFreshnessAndSameValueRecovery(void)
{
  unsigned before;
  Reset(); Source(0U, 1000, -2000, 3000); Service(0U); FinishFrame();
  s_source.header.sample_time_ms = 201U; s_source.header.sequence++;
  Service(201U);
  assert(!s_hud.valid && s_hud.ready && s_full_invalidations == 2U);
  assert(!Display_LvglAttitudeValid());
  assert(s_hud.roll_sin == 0.0f && s_hud.roll_cos == 1.0f && s_hud.yaw_cos == 1.0f);
  FinishFrame(); before = s_full_invalidations; Service(251U);
  assert(s_full_invalidations == before);
  Source(301U, 1000, -2000, 3000); Service(301U);
  assert(s_hud.valid && Display_LvglAttitudeValid() && s_full_invalidations == before + 1U);
  FinishFrame(); s_source_result = PX4LITE_STALE; Service(351U);
  assert(!s_hud.valid && s_full_invalidations == before + 2U);
  FinishFrame(); s_source_result = PX4LITE_OK; s_source.valid_mask = PX4LITE_NAV_VALID_POSITION; Service(401U);
  assert(!s_hud.valid && s_full_invalidations == before + 2U);
  Source(451U, 1000, -2000, 3000); Service(451U);
  assert(s_hud.valid && s_full_invalidations == before + 3U);
}

/** @brief 200ms边界有效，201ms无效；无符号时间差也正确处理32bit回绕。 */
static void TestTimeBoundaries(void)
{
  Reset(); Source(UINT32_MAX-20U, 1000, 2000, 3000); Service(UINT32_MAX-20U); FinishFrame();
  Source(10U, 2000, 2000, 3000); Service(10U);
  assert(s_nav_reads == 1U);
  Service(29U);
  assert(s_nav_reads == 2U && s_hud.roll_deg10 == 200);
  Reset(); Source(100U, 1000, 2000, 3000); Service(300U);
  assert(s_hud.valid);
  FinishFrame(); Service(350U); assert(!s_hud.valid);
  Reset(); Source(UINT32_MAX-20U, 1000, 2000, 3000); Service(179U);
  assert(s_hud.valid); /* 回绕后的真实年龄200ms。 */
  Reset(); Source(UINT32_MAX-20U, 1000, 2000, 3000); Service(180U);
  assert(!s_hud.valid); /* 201ms。 */
}

/** @brief 现有冻结帧保持不变；首次冻结入页仅复用冻结HMI而不偷读新测量。 */
static void TestFreeze(void)
{
  uint8_t saved[sizeof(s_hud)];
  Reset(); Source(0U, 1000, 2000, 3000); Service(0U); FinishFrame();
  memcpy(saved, &s_hud, sizeof(saved));
  s_observation_frozen = 1U; Source(1000U, -1000, -2000, -3000); Service(1000U);
  assert(s_nav_reads == 1U && memcmp(saved, &s_hud, sizeof(saved)) == 0);
  s_observation_frozen = 0U; Service(1001U);
  assert(s_nav_reads == 2U && s_hud.roll_deg10 == -100 && s_hud.yaw_deg10 == -300);
  Reset(); s_observation_frozen = 1U;
  s_values[DISPLAY_HMI_VAR_ROLL] = 123U;
  s_values[DISPLAY_HMI_VAR_PITCH] = (uint32_t)(int32_t)-456;
  s_values[DISPLAY_HMI_VAR_YAW] = 789U;
  s_value_valid[DISPLAY_HMI_VAR_ROLL] = s_value_valid[DISPLAY_HMI_VAR_PITCH] = s_value_valid[DISPLAY_HMI_VAR_YAW] = 1U;
  s_fake_disp.inv_p = 1U; Service(0U);
  assert(s_nav_reads == 0U && !s_hud.ready && s_begin_frames==0U);
  FinishFrame();Service(0U);
  assert(s_nav_reads == 0U && s_reference_reads == 0U && s_hud.ready && s_hud.valid);
  assert(s_hud.roll_deg10 == 123 && s_hud.pitch_deg10 == -456 && s_hud.yaw_deg10 == 789);
  Reset(); s_observation_frozen = 1U; Service(0U);
  assert(s_nav_reads == 0U && s_hud.ready && !s_hud.valid);
}

/** @brief 真实显示模型在主屏ZERO下扣除偏置，跨±180归一化且不改生产测量。 */
static void TestRealDisplayReference(void)
{
  App_NavigationSnapshot_t unchanged;
  Reset(); Source(10U, -17900, 2500, -17900);
  s_reference.reference_active = 1U;
  s_reference.offset_deg100[0] = 17900; s_reference.offset_deg100[1] = 1000; s_reference.offset_deg100[2] = 17900;
  unchanged = s_source; Service(10U);
  assert(s_reference_reads == 1U && s_hud.roll_deg10 == 20 && s_hud.pitch_deg10 == 150 && s_hud.yaw_deg10 == 20);
  assert(memcmp(&unchanged, &s_source, sizeof(s_source)) == 0);
  FinishFrame(); s_reference.reference_active = 0U; Source(60U, -17900, 2500, -17900); Service(60U);
  assert(s_hud.roll_deg10 == -1790 && s_hud.pitch_deg10 == 250 && s_hud.yaw_deg10 == -1790);
}

/** @brief 固定10°递推在全部航向/刻度上保持原方向，126px处误差小于0.01px。 */
static void TestTickGeometry(void)
{
  int yaw, tick;
  float max_error = 0.0f;
  for (yaw = -1800; yaw <= 1800; yaw += 17) {
    float yaw_r = ((float)yaw/10.0f)*(float)(M_PI/180.0);
    float dx = -sinf(yaw_r), dy = -cosf(yaw_r);
    for (tick = 0; tick < 36; ++tick) {
      float angle = (float)(tick*10)*(float)(M_PI/180.0)-yaw_r;
      float ex = fabsf((dx-sinf(angle))*126.0f);
      float ey = fabsf((dy+cosf(angle))*126.0f);
      if (ex>max_error) { max_error=ex; }
      if (ey>max_error) { max_error=ey; }
      assert(ex<0.01f && ey<0.01f);
      Display_LvglHudNextTick(&dx,&dy);
    }
  }
  printf("HUD tick geometry maximum error %.6f px\n",(double)max_error);
}

/** @brief 独立执行全部快路径场景，无外部设备或动态分配依赖。 */
int main(void)
{
  TestCadenceAndDirtyArea();
  TestBusyFrameAndLegacyValues();
  TestScopeAndInitialPendingFrame();
  TestValidityFreshnessAndSameValueRecovery();
  TestTimeBoundaries();
  TestFreeze();
  TestRealDisplayReference();
  TestTickGeometry();
  puts("HUD refresh: scope/cadence/offscreen-full-frame/latest/freeze/validity/reference/pending/geometry PASS");
  return 0;
}
