/**
 * @file test_display_module_pages.c
 * @brief 树莓派控制页的切换、关闭和可读统计回归。
 * @details 使用真实 App 和 LVGL 页面逻辑，控件由固定主机替身提供。
 */
#include <assert.h>
#include <stdio.h>

typedef int BaseType_t;
typedef void *TaskHandle_t;
#include "../../Business/Src/app_data_api.c"
#include "../../Display/Src/display_lvgl.c"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "../../Display/Src/display_lvgl_font_zh.c"
#pragma GCC diagnostic pop

#define TEST_OBJECTS (UI_LABEL_COUNT + UI_BUTTON_COUNT * 2U + UI_STATUS_COUNT + 8U)
static lv_obj_t s_objects[TEST_OBJECTS];
static const char *s_text[TEST_OBJECTS];
static Px4Lite_ModuleExperimentDisplay_t s_framework_display;
static Px4Lite_RemoteNodeStatus_t s_framework_peer;
static Px4Lite_ModuleStatus_t s_framework_status;
static Px4Lite_SensorGnss_t s_framework_gnss;
static Px4Lite_SensorImu_t s_framework_imu;
static Px4Lite_LabSnapshot_t s_framework_lab;
static unsigned s_page_requests;

const lv_font_t display_lvgl_numeric_16 = {0};
const lv_font_t lv_font_montserrat_14 = {0};
const lv_font_t lv_font_montserrat_12 = {0};
const lv_font_t lv_font_montserrat_16 = {0};
const lv_font_t lv_font_montserrat_24 = {0};

/** @brief 提供最后受理的实际 Framework 快照类型，页面逻辑由生产代码执行。 */
void Px4Lite_ModuleExperimentCopyDisplay(Px4Lite_ModuleExperimentDisplay_t *out) { *out = s_framework_display; }
Px4Lite_Result_t Px4Lite_GetModuleStatus(Px4Lite_ModuleId_t id, Px4Lite_ModuleStatus_t *out)
{ *out = s_framework_status; out->module_id = id; out->state = PX4LITE_STATE_ONLINE; return PX4LITE_OK; }
uint32_t Px4Lite_PlatformGetMs(void) { return s_last_tick_ms; }
Px4Lite_Result_t Px4Lite_CopyGnss(Px4Lite_SensorGnss_t *out)
{ *out = s_framework_gnss; return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_CopyImuLatest(Px4Lite_SensorImu_t *out)
{ *out = s_framework_imu; return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_CopyMotor(Px4Lite_MotorOutputs_t *out)
{ memset(out, 0, sizeof(*out)); return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyAttitudeStatus(Px4Lite_AttitudeStatus_t *out)
{ memset(out, 0, sizeof(*out)); return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyBaro(Px4Lite_SensorBaro_t *out)
{ memset(out, 0, sizeof(*out)); return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyBattery2(Px4Lite_BatteryStatus_t *out)
{ memset(out, 0, sizeof(*out)); return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopySerialMonitor(Px4Lite_SerialMonitor_t *out)
{ memset(out, 0, sizeof(*out)); return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyRecording(Px4Lite_RecordingStatus_t *out)
{ memset(out, 0, sizeof(*out)); return PX4LITE_NOT_READY; }
uint8_t Px4Lite_ControlIsPowerInhibited(void) { return 1U; }
Px4Lite_Result_t Px4Lite_CaptureCopyStatus(Px4Lite_CaptureStatus_t *out)
{ memset(out, 0, sizeof(*out)); return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopySelectedRemoteStatus(Px4Lite_RemoteNodeStatus_t *out, uint32_t now_ms)
{ (void)now_ms; *out = s_framework_peer; return out->node_id ? PX4LITE_OK : PX4LITE_NOT_READY; }
void Px4Lite_LoraLabCopy(Px4Lite_LabSnapshot_t *out) { *out = s_framework_lab; }
uint8_t Px4Lite_IdentityGetNodeId(void) { return 1U; }
void Px4Lite_IdentityFormatVendorProductId(char *out, uint8_t capacity)
{ (void)snprintf(out, capacity, "DCDW-001"); }
void LvPortDisp_AbortHudFrame(void) {}
uint8_t LvPortIndev_IsRegistered(void) { return 0U; }
void Display_Gt911_GetStatus(Display_Gt911Status_t *out) { memset(out, 0, sizeof(*out)); }
Display_Result_t Display_SetHmiPage(Display_HmiPage_t page)
{ s_page_requests++; return Display_LvglSetPage(page); }
const lv_font_t *Display_LvglNumericFont(const lv_font_t *font) { return font; }
void lv_label_set_text_static(lv_obj_t *obj, const char *text) { s_text[obj - s_objects] = text; }
char *lv_label_get_text(const lv_obj_t *obj)
{ const char *text = s_text[obj - s_objects]; return (char *)(text ? text : ""); }
void lv_obj_set_pos(lv_obj_t *obj, lv_coord_t x, lv_coord_t y) { (void)obj; (void)x; (void)y; }
void lv_obj_set_size(lv_obj_t *obj, lv_coord_t w, lv_coord_t h) { (void)obj; (void)w; (void)h; }
void lv_obj_set_height(lv_obj_t *obj, lv_coord_t h) { (void)obj; (void)h; }
void lv_label_set_long_mode(lv_obj_t *obj, lv_label_long_mode_t mode) { (void)obj; (void)mode; }
lv_label_long_mode_t lv_label_get_long_mode(const lv_obj_t *obj) { (void)obj; return LV_LABEL_LONG_CLIP; }
void lv_obj_set_style_text_font(lv_obj_t *obj, const lv_font_t *font, lv_style_selector_t selector)
{ (void)obj; (void)font; (void)selector; }
void lv_obj_set_style_text_color(lv_obj_t *obj, lv_color_t color, lv_style_selector_t selector)
{ (void)obj; (void)color; (void)selector; }
void lv_obj_set_style_bg_color(lv_obj_t *obj, lv_color_t color, lv_style_selector_t selector)
{ (void)obj; (void)color; (void)selector; }
void lv_obj_add_flag(lv_obj_t *obj, lv_obj_flag_t flag) { obj->flags |= flag; }
void lv_obj_clear_flag(lv_obj_t *obj, lv_obj_flag_t flag) { obj->flags &= ~flag; }
void lv_obj_add_state(lv_obj_t *obj, lv_state_t state) { (void)obj; (void)state; }
void lv_obj_clear_state(lv_obj_t *obj, lv_state_t state) { (void)obj; (void)state; }
lv_indev_t *lv_indev_get_next(lv_indev_t *indev) { (void)indev; return NULL; }
void lv_indev_reset(lv_indev_t *indev, lv_obj_t *obj) { (void)indev; (void)obj; }
void lv_indev_wait_release(lv_indev_t *indev) { (void)indev; }
lv_res_t lv_event_send(lv_obj_t *obj, lv_event_code_t code, void *param)
{ (void)obj; (void)code; (void)param; return LV_RES_OK; }
bool lv_slider_is_dragged(const lv_obj_t *obj) { (void)obj; return false; }
void lv_bar_set_range(lv_obj_t *obj, int32_t min, int32_t max)
{ (void)obj; (void)min; (void)max; }
void lv_bar_set_value(lv_obj_t *obj, int32_t value, lv_anim_enable_t anim)
{ (void)obj; (void)value; (void)anim; }
lv_style_value_t lv_obj_get_style_prop(const lv_obj_t *obj, lv_part_t part, lv_style_prop_t property)
{ lv_style_value_t value; (void)obj; (void)part; (void)property; value.ptr = &display_lvgl_font_zh_16; return value; }

/** @brief 绑定主机侧固定池，绘制调用不能分配对象。 */
static void BindPool(void)
{
  unsigned i;
  memset(s_objects, 0, sizeof(s_objects));
  memset(s_text, 0, sizeof(s_text));
  memset(s_row_labels, 0, sizeof(s_row_labels));
  memset(s_value_slots, 0, sizeof(s_value_slots));
  memset(s_value_valid, 0, sizeof(s_value_valid));
  s_label_used = s_button_used = 0U;
  for (i = 0U; i < UI_LABEL_COUNT; i++) { s_labels[i] = &s_objects[i]; }
  for (i = 0U; i < UI_BUTTON_COUNT; i++) {
    s_buttons[i].obj = &s_objects[UI_LABEL_COUNT + i * 2U];
    s_buttons[i].label = &s_objects[UI_LABEL_COUNT + i * 2U + 1U];
  }
  for (i = 0U; i < UI_STATUS_COUNT; i++) { s_dots[i] = &s_objects[UI_LABEL_COUNT + UI_BUTTON_COUNT * 2U + i]; }
  for (i = 0U; i < 3U; i++) { s_panels[i] = &s_objects[TEST_OBJECTS - 8U + i]; }
  for (i = 0U; i < 4U; i++) { s_motor_pwm_bars[i] = &s_objects[TEST_OBJECTS - 5U + i]; }
  s_attitude_obj = &s_objects[TEST_OBJECTS - 1U];
}

/** @brief 共享状态快照经真实 App 入口进入显示所有者。 */
static void ApplyDisplay(void)
{
  App_ModuleExperimentDisplay_t snapshot;
  App_CopyModuleExperimentDisplay(&snapshot);
  UiApplyExperimentDisplay(&snapshot);
}

/** @brief 只对已挂载的生产标签检查文本，避免缺少布局时解引用空指针。 */
static void AssertLabelContains(lv_obj_t *label, const char *expected)
{
  assert(label != NULL);
  assert(strstr(lv_label_get_text(label), expected) != NULL);
}

/** @brief 按实际刷新过程完成页面各个布局阶段。 */
static void CompleteLayout(void)
{
  assert(s_layout_requested);
  s_layout_requested = 0U;
  s_layout_stage = 1U;
  do { Display_LvglLayout(); } while (s_layout_stage != 0U);
}

/** @brief 通过可见标签验证字段标题和测量值，不依赖标签分配顺序。 */
static int HasVisibleLabel(const char *text)
{
  unsigned i;
  for (i = 0U; i < s_label_used; i++) {
    if (!(s_labels[i]->flags & LV_OBJ_FLAG_HIDDEN) && strcmp(lv_label_get_text(s_labels[i]), text) == 0) {
      return 1;
    }
  }
  return 0;
}

/** @brief 顺序轮询展示五个入口，关闭只影响所属页，重复开启回到通信本机页。 */
static void TestRoutes(void)
{
  static const Px4Lite_ModuleId_t modules[] = {PX4LITE_MODULE_GNSS, PX4LITE_MODULE_IMU,
    PX4LITE_MODULE_BARO, PX4LITE_MODULE_LORA, PX4LITE_MODULE_REMOTE_ID};
  static const Display_HmiPage_t pages[] = {DISPLAY_HMI_PAGE_DATA, DISPLAY_HMI_PAGE_AIRCRAFT,
    DISPLAY_HMI_PAGE_BARO, DISPLAY_HMI_PAGE_HIDDEN, DISPLAY_HMI_PAGE_REMOTE_ID};
  lv_font_glyph_dsc_t glyph;
  unsigned i;
  s_lvgl_display_ready = 1U;
  ApplyDisplay();
  assert(s_page_requests == 0U && s_current_lvgl_page == DISPLAY_HMI_PAGE_SELF_CHECK);

  for (i = 0U; i < sizeof(modules) / sizeof(modules[0]); i++) {
    s_framework_display.page_sequence++;
    s_framework_display.page_module = modules[i];
    s_observation_frozen = 1U;
    ApplyDisplay();
    assert(s_current_lvgl_page == pages[i] && !s_observation_frozen && !UiExperimentPageClosed());
  }

  (void)Display_LvglSetValue(DISPLAY_HMI_VAR_GNSS_SAT_COUNT, 12U);
  s_framework_display.disabled_mask = 1UL << PX4LITE_MODULE_GNSS;
  s_layout_requested = 0U;
  ApplyDisplay();
  assert(s_current_lvgl_page == DISPLAY_HMI_PAGE_REMOTE_ID && !UiExperimentPageClosed() && !s_layout_requested);
  assert(s_value_valid[DISPLAY_HMI_VAR_GNSS_SAT_COUNT] == 0U);
  s_framework_display.disabled_mask |= 1UL << PX4LITE_MODULE_REMOTE_ID;
  ApplyDisplay();
  assert(s_page_requests == 5U && s_current_lvgl_page == DISPLAY_HMI_PAGE_REMOTE_ID);
  assert(s_layout_requested && UiExperimentPageClosed());
  assert(display_lvgl_font_zh_16.get_glyph_dsc(&display_lvgl_font_zh_16, &glyph, 0x5DF2U, 0x5173U));
  assert(display_lvgl_font_zh_16.get_glyph_dsc(&display_lvgl_font_zh_16, &glyph, 0x5173U, 0x95EDU));
  assert(display_lvgl_font_zh_16.get_glyph_dsc(&display_lvgl_font_zh_16, &glyph, 0x95EDU, 0U));
  assert(display_lvgl_font_zh_16.get_glyph_dsc(&display_lvgl_font_zh_16, &glyph, 0x6253U, 0x5F00U));
  assert(display_lvgl_font_zh_16.get_glyph_dsc(&display_lvgl_font_zh_16, &glyph, 0x5F00U, 0U));

  assert(Display_SetHmiPage(DISPLAY_HMI_PAGE_AIRCRAFT) == DISPLAY_OK);
  assert(Display_SetHmiPage(DISPLAY_HMI_PAGE_DETAILS) == DISPLAY_OK && s_details_kind == UI_DETAILS_IMU);
  s_framework_display.disabled_mask |= 1UL << PX4LITE_MODULE_IMU;
  ApplyDisplay();
  assert(s_current_lvgl_page == DISPLAY_HMI_PAGE_DETAILS && UiExperimentPageClosed());

  s_comm_view = UI_COMM_CONFIG;
  s_framework_display.page_sequence++;
  s_framework_display.page_module = PX4LITE_MODULE_LORA;
  ApplyDisplay();
  assert(s_current_lvgl_page == DISPLAY_HMI_PAGE_HIDDEN && s_comm_view == UI_COMM_LOCAL);
  BindPool();
  (void)Display_LvglSetValue(DISPLAY_HMI_VAR_LORA_TX_COUNT, 12U);
  (void)Display_LvglSetValue(DISPLAY_HMI_VAR_LORA_RX_COUNT, 17U);
  Display_LvglLayoutComm();
  assert(strcmp(lv_label_get_text(s_value_slots[DISPLAY_HMI_VAR_LORA_TX_COUNT].label), "12") == 0);
  assert(strcmp(lv_label_get_text(s_value_slots[DISPLAY_HMI_VAR_LORA_RX_COUNT].label), "17") == 0);
}

/** @brief 每个远端主入口关闭时保留框架、读数标为不可用，开启后恢复实时内容。 */
static void TestClosedPrimaryPages(void)
{
  static const struct {
    Px4Lite_ModuleId_t module;
    Display_HmiPage_t page;
    const char *field;
    Display_HmiVariableId_t value_id;
    uint32_t value;
  } pages[] = {
    {PX4LITE_MODULE_GNSS, DISPLAY_HMI_PAGE_DATA, "Sats used", DISPLAY_HMI_VAR_GNSS_SAT_COUNT, 13U},
    {PX4LITE_MODULE_IMU, DISPLAY_HMI_PAGE_AIRCRAFT, "Roll", DISPLAY_HMI_VAR_ROLL, 123U},
    {PX4LITE_MODULE_BARO, DISPLAY_HMI_PAGE_BARO, "Pressure", DISPLAY_HMI_VAR_PRESSURE, 101325U},
    {PX4LITE_MODULE_LORA, DISPLAY_HMI_PAGE_HIDDEN, "RX frames", DISPLAY_HMI_VAR_LORA_RX_COUNT, 356U},
    {PX4LITE_MODULE_REMOTE_ID, DISPLAY_HMI_PAGE_REMOTE_ID, "UART4 submitted frames",
     DISPLAY_HMI_VAR_COUNT, 789U}
  };
  unsigned i;

  BindPool();
  s_workspace_ready = 1U;
  s_framework_display.disabled_mask = 0U;
  ApplyDisplay();
  for (i = 0U; i < sizeof(pages) / sizeof(pages[0]); i++) {
    Px4Lite_ModuleId_t module = pages[i].module;
    Display_HmiVariableId_t value_id = pages[i].value_id;
    lv_obj_t *live_label;
    const char *live_value;
    char previous[DISPLAY_LVGL_VALUE_TEXT_LEN];
    unsigned button;
    unsigned open = 0U;

    s_framework_display.page_sequence++;
    s_framework_display.page_module = module;
    s_framework_display.remote_id_submitted_count = pages[i].value;
    ApplyDisplay();
    if (value_id < DISPLAY_HMI_VAR_COUNT) {
      assert(Display_LvglSetValue(value_id, pages[i].value) == DISPLAY_OK);
    }
    CompleteLayout();
    assert(s_current_lvgl_page == pages[i].page && HasVisibleLabel(pages[i].field));
    live_label = value_id < DISPLAY_HMI_VAR_COUNT ? s_value_slots[value_id].label : s_row_labels[0];
    assert(live_label != NULL);
    live_value = lv_label_get_text(live_label);
    assert(*live_value != '\0' && strcmp(live_value, "N/A") != 0);
    (void)snprintf(previous, sizeof(previous), "%s", live_value);
    if (module == PX4LITE_MODULE_IMU) {
      s_hud.ready = s_hud.valid = s_observation_frozen = 1U;
    } else if (module == PX4LITE_MODULE_LORA) {
      s_comm_view = UI_COMM_CONFIG;
    }

    s_framework_display.disabled_mask = 1UL << module;
    ApplyDisplay();
    CompleteLayout();
    assert(s_current_lvgl_page == pages[i].page && UiExperimentPageClosed());
    assert(HasVisibleLabel(pages[i].field));
    assert(!(live_label->flags & LV_OBJ_FLAG_HIDDEN));
    assert(strcmp(lv_label_get_text(live_label), "N/A") == 0);
    assert(!HasVisibleLabel(previous));
    assert(!(s_panels[0]->flags & LV_OBJ_FLAG_HIDDEN));
    assert(s_row_labels[UI_ALARM_SUMMARY_ROW] != NULL);
    assert(s_value_slots[DISPLAY_HMI_VAR_VIEW_NODE_ID].label != NULL);
    assert(s_value_slots[DISPLAY_HMI_VAR_FLIGHT_TIME_S].label != NULL);
    AssertLabelContains(s_page_info, "\xE5\xB7\xB2\xE5\x85\xB3\xE9\x97\xAD");
    for (button = UI_NAV_COUNT; button < s_button_used; button++) {
      UiButton_t *control = &s_buttons[button];
      if (control->action == UI_ACTION_MODULE_OPEN) {
        assert(control->argument == module);
        AssertLabelContains(control->label, "\xE6\x89\x93\xE5\xBC\x80");
        open++;
      } else {
        assert(control->action == UI_ACTION_PAGE || control->action == UI_ACTION_BACK);
        assert(control->argument != DISPLAY_HMI_PAGE_DETAILS);
      }
    }
    assert(open == 1U);
    switch (module) {
      case PX4LITE_MODULE_GNSS:
        AssertLabelContains(s_row_labels[0], "Position age N/A");
        AssertLabelContains(s_row_labels[1], "Course N/A");
        break;
      case PX4LITE_MODULE_IMU:
        assert(!(s_attitude_obj->flags & LV_OBJ_FLAG_HIDDEN));
        assert(!Display_LvglAttitudeValid() && !s_observation_frozen);
        AssertLabelContains(s_row_labels[0], "Accel mg");
        AssertLabelContains(s_row_labels[1], "Gyro mdps");
        for (unsigned row = 0U; row < 2U; row++) {
          AssertLabelContains(s_row_labels[row], "X N/A");
          AssertLabelContains(s_row_labels[row], "Y N/A");
          AssertLabelContains(s_row_labels[row], "Z N/A");
        }
        AssertLabelContains(s_row_labels[2], "Sample age N/A");
        AssertLabelContains(s_row_labels[2], "Interval N/A");
        assert(HasVisibleLabel("Calibration N/A"));
        break;
      case PX4LITE_MODULE_BARO:
        AssertLabelContains(s_row_labels[0], "Sample age N/A");
        AssertLabelContains(s_row_labels[1], "Read errors N/A");
        AssertLabelContains(s_row_labels[1], "Recoveries N/A");
        break;
      case PX4LITE_MODULE_LORA:
        AssertLabelContains(s_page_info, "Actual CH N/A");
        AssertLabelContains(s_row_labels[0], "RX node N/A");
        AssertLabelContains(s_row_labels[0], "Frames N/A");
        AssertLabelContains(s_row_labels[1], "Peer heartbeat N/A");
        AssertLabelContains(s_row_labels[1], "Peer loss est. N/A");
        AssertLabelContains(s_row_labels[2], "Lab TX N/A");
        AssertLabelContains(s_row_labels[2], "RX CRC errors N/A");
        AssertLabelContains(s_row_labels[2], "RX drops N/A");
        break;
      case PX4LITE_MODULE_REMOTE_ID:
        AssertLabelContains(s_row_labels[1], "TX errors: N/A");
        AssertLabelContains(s_row_labels[1], "DMA busy: N/A");
        AssertLabelContains(s_row_labels[2], "Last submit N/A");
        AssertLabelContains(s_row_labels[3], "N/A");
        assert(!HasVisibleLabel("DCDW-001"));
        break;
      default:
        assert(0);
    }
    if (value_id < DISPLAY_HMI_VAR_COUNT) {
      assert(s_value_slots[value_id].label == NULL);
      assert(Display_LvglSetValue(value_id, pages[i].value) == DISPLAY_OK);
      Display_LvglApplyValue(value_id);
      assert(strcmp(lv_label_get_text(live_label), "N/A") == 0);
      assert(!HasVisibleLabel(previous));
    }

    s_framework_display.disabled_mask = 0U;
    s_framework_display.page_sequence++;
    ApplyDisplay();
    CompleteLayout();
    assert(s_current_lvgl_page == pages[i].page && !UiExperimentPageClosed());
    if (value_id < DISPLAY_HMI_VAR_COUNT) {
      assert(s_value_slots[value_id].label != NULL);
      if (module != PX4LITE_MODULE_LORA) { assert(!HasVisibleLabel(previous)); }
      assert(Display_LvglSetValue(value_id, pages[i].value) == DISPLAY_OK);
      Display_LvglApplyValue(value_id);
    }
    assert(HasVisibleLabel(previous));
    for (button = UI_NAV_COUNT; button < s_button_used; button++) {
      assert(s_buttons[button].action != UI_ACTION_MODULE_OPEN);
    }
  }
}

/** @brief 独立模块页不占用底栏入口，仍能返回切屏前的主页面。 */
static void TestModuleNavigationHighlight(void)
{
  static const Px4Lite_ModuleId_t modules[] = {
    PX4LITE_MODULE_BARO, PX4LITE_MODULE_LORA, PX4LITE_MODULE_REMOTE_ID
  };
  unsigned i;

  BindPool();
  for (i = 0U; i < sizeof(modules) / sizeof(modules[0]); i++) {
    s_framework_display.page_module = PX4LITE_MODULE_IMU;
    s_framework_display.page_sequence++;
    ApplyDisplay();
    CompleteLayout();
    assert(s_nav_highlight == DISPLAY_HMI_PAGE_AIRCRAFT);

    s_framework_display.page_module = modules[i];
    s_framework_display.page_sequence++;
    ApplyDisplay();
    CompleteLayout();
    assert(UiExperimentPageModule() == modules[i] && !UiExperimentPageClosed());
    assert(s_nav_highlight == DISPLAY_HMI_PAGE_LOGO && s_primary_page == DISPLAY_HMI_PAGE_AIRCRAFT);

    s_framework_display.disabled_mask = 1UL << modules[i];
    ApplyDisplay();
    CompleteLayout();
    assert(UiExperimentPageClosed());
    assert(s_nav_highlight == DISPLAY_HMI_PAGE_LOGO && s_primary_page == DISPLAY_HMI_PAGE_AIRCRAFT);

    s_framework_display.disabled_mask = 0U;
    ApplyDisplay();
    CompleteLayout();
    assert(Display_LvglSetPage(s_primary_page) == DISPLAY_OK);
    CompleteLayout();
    assert(s_current_lvgl_page == DISPLAY_HMI_PAGE_AIRCRAFT && s_nav_highlight == DISPLAY_HMI_PAGE_AIRCRAFT);
  }
}

/** @brief 收包摘要显示真实 App 转换后的单个对端状态，区分数据有效与节点冲突。 */
static void TestSelectedLoraPeer(void)
{
  assert(Display_LvglSetPage(DISPLAY_HMI_PAGE_HIDDEN) == DISPLAY_OK);
  s_comm_view = UI_COMM_LOCAL;
  BindPool();
  Display_LvglLayoutComm();
  s_last_tick_ms = 1000U;
  Display_LvglUpdateComm();
  assert(strcmp(lv_label_get_text(s_row_labels[0]), "RX node / waiting") == 0);
  AssertLabelContains(s_page_info, "Actual CH N/A");
  s_framework_lab.radio.channel = 26U;
  s_framework_lab.radio.channel_known = 1U;
  s_framework_lab.crc_errors = 7U;
  s_framework_lab.rx_drops = 4U;

  s_framework_peer.node_id = 21U;
  s_framework_peer.identity_known = 1U;
  s_framework_peer.state = PX4LITE_REMOTE_NODE_ACTIVE;
  s_framework_peer.rx_frame_count = 36U;
  s_framework_peer.heartbeat_known = 1U;
  s_framework_peer.last_heartbeat_ms = 920U;
  s_framework_peer.rx_loss_rate_x10 = 125U;
  Display_LvglUpdateComm();
  assert(strcmp(lv_label_get_text(s_row_labels[0]), "RX node 21 / Receiving / Frames 36") == 0);
  AssertLabelContains(s_page_info, "Actual CH 26");
  AssertLabelContains(s_row_labels[1], "Peer heartbeat 80 ms ago");
  AssertLabelContains(s_row_labels[1], "Peer loss est. 12.5%");
  AssertLabelContains(s_row_labels[2], "CRC errors 7");
  AssertLabelContains(s_row_labels[2], "RX drops 4");

  s_framework_peer.id_conflict = 1U;
  Display_LvglUpdateComm();
  assert(strstr(lv_label_get_text(s_row_labels[0]), "ID CONFLICT") != NULL);
  AssertLabelContains(s_row_labels[1], "Peer heartbeat N/A");

  s_framework_peer.id_conflict = 0U;
  s_framework_peer.state = PX4LITE_REMOTE_NODE_STALE;
  Display_LvglUpdateComm();
  assert(strstr(lv_label_get_text(s_row_labels[0]), "STALE") != NULL);
}

/** @brief 保留发送和配置原有按钮，只在三个有效通信视图间轮换。 */
static void TestLoraControls(void)
{
  assert(UI_COMM_VIEW_COUNT == 3U);
  s_comm_view = UI_COMM_EXPERIMENT;
  BindPool();
  Display_LvglLayoutComm();
  assert(strcmp(lv_label_get_text(s_buttons[0].label), "Config >") == 0);
  assert(s_buttons[3].argument == PX4LITE_LAB_SEND);
  assert(s_buttons[4].argument == PX4LITE_LAB_START);
  assert(s_buttons[5].argument == PX4LITE_LAB_STOP);

  s_comm_view = UI_COMM_CONFIG;
  BindPool();
  Display_LvglLayoutComm();
  assert(strcmp(lv_label_get_text(s_buttons[0].label), "Local >") == 0);
  assert(s_buttons[1].action == UI_ACTION_CHANNEL);
  assert(s_buttons[2].argument == PX4LITE_LAB_SAVE_NODE);
  assert(s_buttons[3].argument == PX4LITE_LAB_APPLY_CHANNEL);
  assert(s_buttons[4].argument == PX4LITE_LAB_RESTORE);
}

/** @brief 底栏跟踪活动告警快照的出现、解除和不可用三种状态。 */
static void TestCurrentAlarmSummary(void)
{
  App_AlarmSnapshot_t alarms = {0};
  const uint32_t fault = ((uint32_t)PX4LITE_MODULE_SYSTEM << 16) | PX4LITE_FAULT_SYSTEM_TASK_LOST;
  assert(Display_LvglSetPage(DISPLAY_HMI_PAGE_AIRCRAFT) == DISPLAY_OK);
  BindPool();
  Display_LvglLayoutSummary();
  s_workspace_ready = 1U;
  Display_LvglSetAlarmSnapshot(&alarms, &fault, 1U);
  assert(strstr(lv_label_get_text(s_row_labels[UI_ALARM_SUMMARY_ROW]), "TASK LOST") != NULL);
  Display_LvglSetAlarmSnapshot(&alarms, NULL, 0U);
  assert(strcmp(lv_label_get_text(s_row_labels[UI_ALARM_SUMMARY_ROW]), "No active alarms") == 0);
  Display_LvglSetAlarmSnapshot(NULL, NULL, 0U);
  assert(strcmp(lv_label_get_text(s_row_labels[UI_ALARM_SUMMARY_ROW]), "N/A - unavailable") == 0);
}

/** @brief 详情切换后按当前数据归属关闭，电机详情的控件继续可用。 */
static void TestDetailsModule(void)
{
  s_framework_display.disabled_mask = 0U;
  ApplyDisplay();
  assert(Display_SetHmiPage(DISPLAY_HMI_PAGE_AIRCRAFT) == DISPLAY_OK);
  assert(Display_SetHmiPage(DISPLAY_HMI_PAGE_DETAILS) == DISPLAY_OK);
  s_details_kind = UI_DETAILS_MOTOR;
  s_layout_requested = 0U;
  s_framework_display.disabled_mask = 1UL << PX4LITE_MODULE_IMU;
  ApplyDisplay();
  assert(UiMotorInputPage() && !UiExperimentPageClosed() && !s_layout_requested);

  s_details_kind = UI_DETAILS_GNSS;
  assert(!UiExperimentPageClosed());
  s_framework_display.disabled_mask |= 1UL << PX4LITE_MODULE_GNSS;
  ApplyDisplay();
  assert(UiExperimentPageClosed() && s_layout_requested);
  BindPool();
  Display_LvglLayoutClosedModule();
  assert(s_button_used == 1U && s_buttons[0].argument == PX4LITE_MODULE_GNSS);
}

/** @brief 关闭命令不切页时，原来没有本地导航的两个模块仍可从自检进入。 */
static void TestClosedModuleShortcuts(void)
{
  assert(Display_LvglSetPage(DISPLAY_HMI_PAGE_SELF_CHECK) == DISPLAY_OK);
  s_framework_display.disabled_mask = 0U;
  ApplyDisplay();
  BindPool();
  Display_LvglLayoutSelfCheck();
  assert(s_button_used == 2U);

  s_layout_requested = 0U;
  s_framework_display.disabled_mask = (1UL << PX4LITE_MODULE_BARO) | (1UL << PX4LITE_MODULE_REMOTE_ID);
  ApplyDisplay();
  assert(s_layout_requested && s_current_lvgl_page == DISPLAY_HMI_PAGE_SELF_CHECK);
  BindPool();
  Display_LvglLayoutSelfCheck();
  assert(s_button_used == 4U);
  assert(s_buttons[0].action == UI_ACTION_PAGE && s_buttons[0].argument == DISPLAY_HMI_PAGE_BARO);
  assert(s_buttons[1].action == UI_ACTION_PAGE && s_buttons[1].argument == DISPLAY_HMI_PAGE_REMOTE_ID);
}

/** @brief 气压各量和 Remote ID 本机提交统计使用各自的值和可用条件。 */
static void TestBaroAndRemoteIdReadings(void)
{
  s_framework_display.disabled_mask = 0U;
  ApplyDisplay();
  assert(Display_LvglSetPage(DISPLAY_HMI_PAGE_BARO) == DISPLAY_OK);
  BindPool();
  s_last_tick_ms = 1000U;
  s_framework_status.last_valid_ms = 875U;
  (void)Display_LvglSetValue(DISPLAY_HMI_VAR_PRESSURE, 101325U);
  (void)Display_LvglSetValue(DISPLAY_HMI_VAR_TEMPERATURE, 250U);
  (void)Display_LvglSetValue(DISPLAY_HMI_VAR_HUMIDITY, 458U);
  Display_LvglLayoutExperimentModule(PX4LITE_MODULE_BARO);
  assert(s_value_slots[DISPLAY_HMI_VAR_HUMIDITY].label != NULL);
  assert(strstr(s_value_slots[DISPLAY_HMI_VAR_PRESSURE].text, "101325") != NULL);
  assert(strcmp(s_value_slots[DISPLAY_HMI_VAR_TEMPERATURE].text, "25.0C") == 0);
  assert(strcmp(s_value_slots[DISPLAY_HMI_VAR_HUMIDITY].text, "45.8%") == 0);
  Display_LvglUpdateExperimentModule(PX4LITE_MODULE_BARO);
  AssertLabelContains(s_page_info, "ONLINE");
  AssertLabelContains(s_row_labels[0], "Sample age 125 ms");
  (void)Display_LvglInvalidateValue(DISPLAY_HMI_VAR_PRESSURE);
  Display_LvglUpdateExperimentModule(PX4LITE_MODULE_BARO);
  AssertLabelContains(s_row_labels[0], "N/A");

  assert(Display_LvglSetPage(DISPLAY_HMI_PAGE_REMOTE_ID) == DISPLAY_OK);
  BindPool();
  s_framework_display.remote_id_submitted_count = 23U;
  s_framework_status.last_valid_ms = 920U;
  s_framework_status.error_count = 7U;
  s_framework_status.drop_count = 3U;
  ApplyDisplay();
  Display_LvglLayoutExperimentModule(PX4LITE_MODULE_REMOTE_ID);
  Display_LvglUpdateExperimentModule(PX4LITE_MODULE_REMOTE_ID);
  AssertLabelContains(s_page_info, "ONLINE");
  assert(s_row_labels[0] != NULL);
  assert(strcmp(lv_label_get_text(s_row_labels[0]), "23") == 0);
  AssertLabelContains(s_row_labels[1], "TX errors: 7");
  AssertLabelContains(s_row_labels[1], "DMA busy: 3");
  AssertLabelContains(s_row_labels[2], "Last submit");
  AssertLabelContains(s_row_labels[2], "80 ms");
  AssertLabelContains(s_row_labels[3], "DCDW-001");
  s_framework_display.remote_id_submitted_count = 0U;
  ApplyDisplay();
  Display_LvglUpdateExperimentModule(PX4LITE_MODULE_REMOTE_ID);
  AssertLabelContains(s_row_labels[2], "N/A");
}

/** @brief 航迹角的有效零值与定位、航迹各自过期分开。 */
static void TestGnssReadings(void)
{
  assert(Display_LvglSetPage(DISPLAY_HMI_PAGE_DATA) == DISPLAY_OK);
  BindPool();
  s_last_tick_ms = 1000U;
  s_framework_gnss.valid_mask = PX4LITE_GNSS_VALID_POSITION | PX4LITE_GNSS_VALID_VELOCITY;
  s_framework_gnss.last_rx_ms = 980U;
  s_framework_gnss.position_update_ms = 920U;
  s_framework_gnss.velocity_update_ms = 950U;
  s_framework_gnss.heading_deg100 = 0U;
  Display_LvglLayoutGnss();
  Display_LvglUpdateExperimentModule(PX4LITE_MODULE_GNSS);
  AssertLabelContains(s_page_info, "ONLINE");
  AssertLabelContains(s_row_labels[0], "Position age 80 ms");
  AssertLabelContains(s_row_labels[1], "Course 0.00 deg");

  s_last_tick_ms = 2450U;
  s_framework_gnss.last_rx_ms = s_last_tick_ms;
  Display_LvglUpdateExperimentModule(PX4LITE_MODULE_GNSS);
  AssertLabelContains(s_row_labels[0], "N/A");
  AssertLabelContains(s_row_labels[1], "Course 0.00 deg");
  s_last_tick_ms++;
  Display_LvglUpdateExperimentModule(PX4LITE_MODULE_GNSS);
  AssertLabelContains(s_row_labels[1], "N/A");
}

/** @brief 六轴读数随现有冻结开关暂停，恢复后超龄数据不得当作当前读数。 */
static void TestImuReadings(void)
{
  assert(Display_LvglSetPage(DISPLAY_HMI_PAGE_AIRCRAFT) == DISPLAY_OK);
  BindPool();
  s_last_tick_ms = 1000U;
  s_framework_imu.header.valid = 1U;
  s_framework_imu.header.sample_time_ms = 975U;
  s_framework_imu.accel_mg[0] = -125;
  s_framework_imu.accel_mg[2] = 998;
  s_framework_imu.gyro_mdps[0] = 250;
  s_framework_imu.gyro_mdps[2] = -800;
  s_framework_imu.sample_period_us = 10000U;
  Display_LvglLayoutAttitude();
  Display_LvglUpdateExperimentModule(PX4LITE_MODULE_IMU);
  AssertLabelContains(s_page_info, "ONLINE");
  AssertLabelContains(s_row_labels[0], "Accel mg");
  AssertLabelContains(s_row_labels[0], "-125");
  AssertLabelContains(s_row_labels[1], "Gyro mdps");
  AssertLabelContains(s_row_labels[1], "-800");
  AssertLabelContains(s_row_labels[2], "Sample age 25 ms");
  AssertLabelContains(s_row_labels[2], "Interval 10000 us");

  s_observation_frozen = 1U;
  s_last_tick_ms += PX4LITE_IMU_MAX_AGE_MS;
  Display_LvglUpdateExperimentModule(PX4LITE_MODULE_IMU);
  AssertLabelContains(s_row_labels[0], "-125");
  AssertLabelContains(s_row_labels[1], "-800");
  s_observation_frozen = 0U;
  Display_LvglUpdateExperimentModule(PX4LITE_MODULE_IMU);
  AssertLabelContains(s_row_labels[0], "N/A");
  AssertLabelContains(s_row_labels[1], "N/A");
}

/** @brief 默认远端版拒绝已裁剪页面，同时保留串口统计和 SD 记录页。 */
static void TestLocalPageSelection(void)
{
  assert(Display_LvglSetPage(DISPLAY_HMI_PAGE_SERIAL) == DISPLAY_OK);
  assert(Display_LvglSetPage(DISPLAY_HMI_PAGE_NMEA) == DISPLAY_ERROR);
  assert(Display_LvglSetPage(DISPLAY_HMI_PAGE_INSPECTOR) == DISPLAY_ERROR);
  assert(Display_LvglSetPage(DISPLAY_HMI_PAGE_OBSERVE) == DISPLAY_ERROR);
  assert(Display_LvglSetPage(DISPLAY_HMI_PAGE_EVENT_HISTORY) == DISPLAY_ERROR);
  assert(s_current_lvgl_page == DISPLAY_HMI_PAGE_SERIAL);
  assert(Display_LvglSetPage(DISPLAY_HMI_PAGE_RECORD) == DISPLAY_OK);
}

int main(void)
{
  TestRoutes();
  TestClosedPrimaryPages();
  TestModuleNavigationHighlight();
  TestSelectedLoraPeer();
  TestLoraControls();
  TestCurrentAlarmSummary();
  TestDetailsModule();
  TestClosedModuleShortcuts();
  TestBaroAndRemoteIdReadings();
  TestGnssReadings();
  TestImuReadings();
  TestLocalPageSelection();
  puts("module pages: five routes, N/A when closed, reopen, navigation highlight, live/stale/frozen readings, details, alarms, local selection PASS");
  return 0;
}
