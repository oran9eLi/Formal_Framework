/**
 * @file test_display_lvgl_invalidation.c
 * @brief 显示层标签刷新与常驻导航回归。
 * @details 使用生产缓存和真实布局阶段的复位函数，以 LVGL 调用计数验证重绘范围。
 */
#include <assert.h>
#include <stdio.h>
typedef int BaseType_t;
typedef void *TaskHandle_t;
#include "../../Display/Src/display_lvgl.c"

static lv_obj_t labels[3], buttons[UI_BUTTON_COUNT], other;
static unsigned text_updates, mode_updates, color_updates, buttons_hidden;
static lv_color_t colors[UI_NAV_COUNT];
static lv_obj_t sd_label, sd_dot;
static const char *sd_label_text;
static lv_color_t sd_color;
static unsigned sd_repaints;
static Px4Lite_State_t sd_status;
static Px4Lite_Result_t sd_result;

/** @brief 注入本机 SD 快照；读取失败时不写出状态。 */
Px4Lite_Result_t App_GetModuleStatus(Px4Lite_ModuleId_t id, Px4Lite_ModuleStatus_t *out)
{
  assert(id == PX4LITE_MODULE_STORAGE);
  if (sd_result == PX4LITE_OK) { out->state = sd_status; }
  return sd_result;
}

/** @brief 模拟 LVGL 返回已绑定的静态文字。 */
char *lv_label_get_text(const lv_obj_t *obj)
{
  assert(obj == &sd_label);
  return (char *)sd_label_text;
}

/** @brief 确认送给 LVGL 的是持久文本缓冲，非函数栈上的格式化暂存。 */
void lv_label_set_text_static(lv_obj_t *obj, const char *text)
{
  if (obj == &sd_label) { sd_label_text = text; return; }
  assert(obj == &labels[0]);
  assert(text == s_value_slots[DISPLAY_HMI_VAR_GNSS_SPEED].text);
  text_updates++;
}

/** @brief 仅曾使用换行的标签需要重设模式。 */
lv_label_long_mode_t lv_label_get_long_mode(const lv_obj_t *obj)
{
  assert(obj == &labels[0] || obj == &labels[1] || obj == &labels[2]);
  return obj == &labels[1] ? LV_LABEL_LONG_WRAP : LV_LABEL_LONG_CLIP;
}
void lv_label_set_long_mode(lv_obj_t *obj, lv_label_long_mode_t mode)
{
  assert(obj == &labels[1] && mode == LV_LABEL_LONG_CLIP);
  mode_updates++;
}

/** @brief 捕获上页隐藏操作，底部导航被隐藏即失败。 */
void lv_obj_add_flag(lv_obj_t *obj, lv_obj_flag_t flag)
{
  unsigned i;
  assert(obj && flag == LV_OBJ_FLAG_HIDDEN);
  for (i = 0; i < UI_BUTTON_COUNT; i++) {
    if (obj == &buttons[i]) { assert(i >= UI_NAV_COUNT); buttons_hidden++; }
  }
}

/** @brief 捕获导航颜色更新；子页沿用原主入口时应为零次。 */
void lv_obj_set_style_bg_color(lv_obj_t *obj, lv_color_t color, lv_style_selector_t selector)
{
  unsigned i;
  assert(selector == 0);
  if (obj == &sd_dot) { sd_color = color; ++sd_repaints; return; }
  for (i = 0; i < UI_NAV_COUNT; i++) {
    if (obj == &buttons[i]) { colors[i] = color; color_updates++; return; }
  }
  assert(0);
}

/** @brief 速度在同一显示精度内变化应跳过重绘，失效及同原值恢复须分别重绘。 */
static void TestValueText(void)
{
  Display_HmiVariableId_t id = DISPLAY_HMI_VAR_GNSS_SPEED;
  s_value_slots[id].label = &labels[0];
  assert(Display_LvglSetValue(id, 120U) == DISPLAY_OK && UiValuesPending());
  Display_LvglUpdateValueLabel(id);
  assert(text_updates == 1 && !strcmp(s_value_slots[id].text, "1.2m/s"));
  assert(Display_LvglSetValue(id, 129U) == DISPLAY_OK);
  Display_LvglUpdateValueLabel(id);
  assert(text_updates == 1);
  assert(Display_LvglSetValue(id, 130U) == DISPLAY_OK);
  Display_LvglUpdateValueLabel(id);
  assert(text_updates == 2 && !strcmp(s_value_slots[id].text, "1.3m/s"));
  assert(Display_LvglInvalidateValue(id) == DISPLAY_OK && s_values[id] == 130U);
  Display_LvglUpdateValueLabel(id);
  assert(text_updates == 3 && !strcmp(s_value_slots[id].text, "N/A"));
  assert(Display_LvglSetValue(id, 130U) == DISPLAY_OK);
  Display_LvglUpdateValueLabel(id);
  assert(text_updates == 4 && !strcmp(s_value_slots[id].text, "1.3m/s"));
  assert(Display_LvglInvalidateValue(DISPLAY_HMI_VAR_COUNT)==DISPLAY_ERROR);
}

/** @brief 检查跨主页面和同主入口的二级页面，以及上页控件池回收。 */
static void TestNavigation(void)
{
  unsigned i;
  for (i = 0; i < UI_BUTTON_COUNT; i++) {
    s_buttons[i].obj = &buttons[i];
    s_buttons[i].action = UI_ACTION_PAGE;
    s_buttons[i].argument = (uint16_t)(DISPLAY_HMI_PAGE_SELF_CHECK + i);
  }
  for (i = 0; i < 3; i++) { s_labels[i] = &labels[i]; s_panels[i] = &other; }
  for (i = 0; i < UI_STATUS_COUNT; i++) { s_dots[i] = &other; }
  for (i = 0; i < 4; i++) { s_motor_pwm_bars[i] = &other; }
  s_attitude_obj = &other;
  s_button_used = UI_NAV_COUNT + 2U;
  s_label_used = 3U;
  Display_LvglHighlightNavigation();
  assert(color_updates == 1 && colors[0].full == lv_color_hex(0x186577).full);
  Display_LvglResetWorkspace();
  assert(s_button_used == UI_NAV_COUNT && s_label_used == 0 && buttons_hidden == 2 && mode_updates == 1);
  for (i = 0; i < UI_NAV_COUNT; i++) {
    assert(s_buttons[i].action == UI_ACTION_PAGE && s_buttons[i].argument == DISPLAY_HMI_PAGE_SELF_CHECK + i);
  }
  assert(s_buttons[UI_NAV_COUNT].action == UI_ACTION_NONE);
  s_primary_page = DISPLAY_HMI_PAGE_AIRCRAFT;
  Display_LvglHighlightNavigation();
  assert(color_updates == 3 && colors[0].full == lv_color_hex(0x142A3B).full);
  assert(colors[2].full == lv_color_hex(0x186577).full);
  s_current_lvgl_page = DISPLAY_HMI_PAGE_DETAILS;
  Display_LvglHighlightNavigation();
  assert(color_updates == 3);
}

/** @brief 枚举 SD 的全部来源状态与读失败，并核对其他模块保留降级黄灯。 */
static void TestSdIndicator(void)
{
  unsigned i;
  uint32_t color;
  for (i = PX4LITE_STATE_UNINITIALIZED; i <= PX4LITE_STATE_DISABLED; ++i) {
    const char *label = UiSelfCheckIndicator(PX4LITE_MODULE_STORAGE, PX4LITE_OK, (Px4Lite_State_t)i, &color);
    if (i == PX4LITE_STATE_ONLINE) {
      assert(!strcmp(label, "READY") && color == 0x59CC8A);
    } else {
      assert(!strcmp(label, "NOT READY") && color == UI_COLOR_ERROR);
    }
  }
  assert(!strcmp(UiSelfCheckIndicator(PX4LITE_MODULE_STORAGE, PX4LITE_BUSY, PX4LITE_STATE_ONLINE, &color), "NOT READY"));
  assert(color == UI_COLOR_ERROR);
  assert(!strcmp(UiSelfCheckIndicator(PX4LITE_MODULE_GNSS, PX4LITE_OK, PX4LITE_STATE_DEGRADED, &color), "DEGRADED"));
  assert(color == UI_COLOR_WARN);
}

/** @brief 运行实际绑定流程，验证首帧、重复刷新、拔卡、切回和恢复都只提交两种灯色。 */
static void TestSdIndicatorBinding(void)
{
  unsigned index = 0U;
  while (index < UI_STATUS_COUNT && s_modules[index].id != PX4LITE_MODULE_STORAGE) { ++index; }
  assert(index < UI_STATUS_COUNT);
  s_state_labels[index] = &sd_label;
  s_dots[index] = &sd_dot;
  sd_result = PX4LITE_NOT_READY;
  UiUpdateSelfCheckIndicator(index);
  assert(!strcmp(sd_label_text, "NOT READY") && sd_color.full == lv_color_hex(UI_COLOR_ERROR).full);
  sd_result = PX4LITE_OK;
  sd_status = PX4LITE_STATE_ONLINE;
  UiUpdateSelfCheckIndicator(index);
  UiUpdateSelfCheckIndicator(index);
  assert(sd_repaints == 2 && !strcmp(sd_label_text, "READY") && sd_color.full == lv_color_hex(0x59CC8A).full);
  sd_status = PX4LITE_STATE_DEGRADED;
  UiUpdateSelfCheckIndicator(index);
  assert(sd_repaints == 3 && !strcmp(sd_label_text, "NOT READY") && sd_color.full == lv_color_hex(UI_COLOR_ERROR).full);
  s_state_text[index] = NULL;
  sd_label_text = "";
  UiUpdateSelfCheckIndicator(index);
  assert(sd_repaints == 4 && !strcmp(sd_label_text, "NOT READY") && sd_color.full == lv_color_hex(UI_COLOR_ERROR).full);
  sd_status = PX4LITE_STATE_ONLINE;
  UiUpdateSelfCheckIndicator(index);
  assert(sd_repaints == 5 && !strcmp(sd_label_text, "READY") && sd_color.full == lv_color_hex(0x59CC8A).full);
}

/** @brief 主机入口，不访问板级总线或创建任务。 */
int main(void)
{
  TestSdIndicator();
  TestSdIndicatorBinding();
  TestValueText();
  TestNavigation();
  puts("Display refresh PASS: rounded values / invalidation and recovery / persistent navigation / label reuse");
  return 0;
}
