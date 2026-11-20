/**
 * @file test_gnss_diagnostics_display.c
 * @brief 正常GNSS测量与双协议Frames页面回归，保留无诊断读取边界。
 * @details 文件名保留既有runner入口；使用真实App/LVGL页面实现和静态控件替身，
 *          不访问BSP、不创建任务、不读取诊断或发送控制命令。
 */
#include <assert.h>
#include <stdio.h>

typedef int BaseType_t;
typedef void *TaskHandle_t;
#include "../../Business/Src/app_data_api.c"
#include "../../Display/Src/display_lvgl.c"

#define TEST_OBJECTS (UI_LABEL_COUNT + UI_BUTTON_COUNT * 2U + 3U)
static lv_obj_t s_fake_objects[TEST_OBJECTS];
static const char *s_fake_text[TEST_OBJECTS];
static const lv_font_t *s_fake_font[TEST_OBJECTS];
static lv_coord_t s_fake_y[TEST_OBJECTS];
static Px4Lite_SensorGnss_t s_measurement;
static Px4Lite_Result_t s_measurement_result = PX4LITE_NOT_READY;
static Px4Lite_CaptureStatus_t s_capture_status;
static Px4Lite_Result_t s_capture_status_result=PX4LITE_OK;
static Px4Lite_CaptureSummary_t s_summary_rows[PX4LITE_CAPTURE_SUMMARIES];
static App_GnssCapturedFrame_t s_captured_gnss;
static App_CaptureRequest_t s_last_capture_request;
static unsigned s_capture_requests,s_gnss_frame_copies;
static unsigned s_measurement_copies, s_diagnostic_copies;
static uint32_t s_now_ms;
const lv_font_t display_lvgl_font_zh_16 = {0};
const lv_font_t display_lvgl_numeric_14 = {0};
const lv_font_t display_lvgl_hex_14 = {0};
const lv_font_t lv_font_montserrat_14 = {0};
const lv_font_t lv_font_montserrat_24 = {0};

/** @brief Framework测量替身仍经真实App执行逐域时效检查。 */
Px4Lite_Result_t Px4Lite_CopyGnss(Px4Lite_SensorGnss_t *out)
{ s_measurement_copies++; *out = s_measurement; return s_measurement_result; }
/** @brief 与App复制后的真实毫秒基准对应，不以RTC或本地时区续期GNSS事实。 */
uint32_t Px4Lite_PlatformGetMs(void) { return s_now_ms; }
/** @brief 诊断接口保留但正常页面不得再调用它。 */
Px4Lite_Result_t Px4Lite_GnssCopyDiagnostics(Px4Lite_GnssDiagnostics_t *out)
{ (void)out; s_diagnostic_copies++; assert(0 && "clean GNSS UI must not read diagnostics"); return PX4LITE_NOT_READY; }
/** @brief 捕获状态仅供NMEA页面测试，不重配任何端口。 */
Px4Lite_Result_t Px4Lite_CaptureCopyStatus(Px4Lite_CaptureStatus_t *out)
{ if(s_capture_status_result!=PX4LITE_OK){return s_capture_status_result;}*out = s_capture_status; return PX4LITE_OK; }
/** @brief 元数据替身覆盖NMEA/UBX汇总，Display不得解码原帧payload。 */
Px4Lite_Result_t Px4Lite_CaptureCopySummary(uint32_t session, uint8_t index, Px4Lite_CaptureSummary_t *out)
{
  if (session != s_capture_status.session_id || index >= s_capture_status.summary_count) { return PX4LITE_NOT_READY; }
  *out=s_summary_rows[index];
  return PX4LITE_OK;
}
/** @brief 实际App包装后的会话/token读取；不提供MAVLink或旧NMEA专用副本。 */
Px4Lite_Result_t Px4Lite_CaptureCopyGnss(uint32_t session,uint32_t token,Px4Lite_GnssCapturedFrame_t *out)
{
  s_gnss_frame_copies++;
  if (session!=s_capture_status.session_id || session!=s_captured_gnss.session_id || token!=s_captured_gnss.token) { return PX4LITE_NOT_READY; }
  *out=s_captured_gnss;return PX4LITE_OK;
}
/** @brief 请求替身只记录参数，执行结果必须由另一次状态副本确认。 */
Px4Lite_Result_t Px4Lite_CaptureRequest(const Px4Lite_CaptureRequest_t *request)
{ s_capture_requests++;s_last_capture_request=*request;return PX4LITE_OK; }
/** @brief NMEA测试不应走MAVLink阶段查询分支。 */
Px4Lite_Result_t Px4Lite_CaptureCopyStage(uint32_t session, uint32_t token, uint8_t *stage)
{ (void)session; (void)token; (void)stage; assert(0); return PX4LITE_NOT_READY; }
/** @brief NMEA页面无需MAVLink解码。 */
Px4Lite_Result_t Px4Lite_DecodeFrame(const Px4Lite_CaptureFrame_t *frame, uint8_t page, Px4Lite_DecodedPage_t *out)
{ (void)frame; (void)page; (void)out; assert(0); return PX4LITE_NOT_READY; }
/** @brief 既有捕获页面读取通信统计，仅提供零副本，不改变捕获语义。 */
void Px4Lite_GetCommDebugInfo(Px4Lite_CommDebugInfo_t *out) { memset(out, 0, sizeof(*out)); }

/** @brief 所有对象必须来自测试静态池，防止越界或运行时创建新对象。 */
static unsigned ObjectIndex(const lv_obj_t *obj)
{
  unsigned i;
  for (i = 0U; i < TEST_OBJECTS; ++i) { if (obj == &s_fake_objects[i]) { return i; } }
  assert(0 && "unexpected LVGL object"); return 0U;
}
void lv_label_set_text_static(lv_obj_t *obj, const char *text) { s_fake_text[ObjectIndex(obj)] = text; }
char *lv_label_get_text(const lv_obj_t *obj)
{ const char *text = s_fake_text[ObjectIndex(obj)]; return (char *)(text ? text : ""); }
void lv_obj_set_pos(lv_obj_t *obj, lv_coord_t x, lv_coord_t y) { (void)x; s_fake_y[ObjectIndex(obj)] = y; }
void lv_obj_set_size(lv_obj_t *obj, lv_coord_t w, lv_coord_t h) { (void)w; (void)h; (void)ObjectIndex(obj); }
void lv_obj_set_height(lv_obj_t *obj, lv_coord_t h) { (void)h; (void)ObjectIndex(obj); }
void lv_obj_set_style_text_font(lv_obj_t *obj, const lv_font_t *font, lv_style_selector_t selector)
{ (void)selector; s_fake_font[ObjectIndex(obj)] = font; }
void lv_obj_set_style_text_color(lv_obj_t *obj, lv_color_t color, lv_style_selector_t selector)
{ (void)color; (void)selector; (void)ObjectIndex(obj); }
void lv_obj_set_style_bg_color(lv_obj_t *obj, lv_color_t color, lv_style_selector_t selector)
{ (void)color; (void)selector; (void)ObjectIndex(obj); }
void lv_obj_clear_flag(lv_obj_t *obj, lv_obj_flag_t flag) { (void)flag; (void)ObjectIndex(obj); }
void lv_obj_clear_state(lv_obj_t *obj, lv_state_t state) { (void)state; (void)ObjectIndex(obj); }
lv_style_value_t lv_obj_get_style_prop(const lv_obj_t *obj, lv_part_t part, lv_style_prop_t property)
{ lv_style_value_t value; (void)part; assert(property == LV_STYLE_TEXT_FONT); value.ptr = s_fake_font[ObjectIndex(obj)]; return value; }
const lv_font_t *Display_LvglNumericFont(const lv_font_t *font) { return font; }

/** @brief 复用与生产相同的标签/按钮容量，不调用LVGL分配函数。 */
static void BindPool(void)
{
  unsigned i;
  memset(s_fake_objects, 0, sizeof(s_fake_objects)); memset(s_fake_text, 0, sizeof(s_fake_text));
  memset(s_fake_font, 0, sizeof(s_fake_font)); memset(s_row_labels, 0, sizeof(s_row_labels));
  memset(s_value_slots, 0, sizeof(s_value_slots)); memset(s_value_valid, 0, sizeof(s_value_valid));
  for (i = 0U; i < UI_LABEL_COUNT; ++i) { s_labels[i] = &s_fake_objects[i]; }
  for (i = 0U; i < UI_BUTTON_COUNT; ++i) {
    s_buttons[i].obj = &s_fake_objects[UI_LABEL_COUNT + i * 2U];
    s_buttons[i].label = &s_fake_objects[UI_LABEL_COUNT + i * 2U + 1U];
  }
  for (i = 0U; i < 3U; ++i) { s_panels[i] = &s_fake_objects[UI_LABEL_COUNT + UI_BUTTON_COUNT * 2U + i]; }
  s_label_used = s_button_used = 0U; s_page_info = NULL;
}

/** @brief 临时GNSS收发、配置和服务提示不应留在正常页面文本中。 */
static void AssertNoTemporaryText(const char *text)
{
  assert(strstr(text, "Local ") == NULL && strstr(text, "UART RX") == NULL);
  assert(strstr(text, "Svc ") == NULL && strstr(text, "Config ") == NULL);
  assert(strstr(text, "receive diagnostics") == NULL && strstr(text, "UBX status") == NULL);
  assert(strstr(text, "TX submit") == NULL && strstr(text, "HW(now)") == NULL);
}

/** @brief 主页面绑定八项测量和可读时效；可选 Frames 入口仍独立于实时读数。 */
static void TestNormalGnssLayout(void)
{
  static const Display_HmiVariableId_t ids[8] = {DISPLAY_HMI_VAR_GNSS_FIX, DISPLAY_HMI_VAR_GNSS_SAT_COUNT,
    DISPLAY_HMI_VAR_GNSS_HDOP, DISPLAY_HMI_VAR_GNSS_SPEED, DISPLAY_HMI_VAR_LATITUDE,
    DISPLAY_HMI_VAR_LONGITUDE, DISPLAY_HMI_VAR_ALTITUDE, DISPLAY_HMI_VAR_GNSS_TIME};
  unsigned i;
  BindPool(); Display_LvglLayoutGnss();
  assert(s_button_used == 2U);
  for (i = 0U; i < 8U; ++i) {
    assert(s_value_slots[ids[i]].label != NULL);
    assert(strcmp(s_value_slots[ids[i]].text, "N/A") == 0);
  }
  assert(s_page_info != NULL && s_row_labels[0] != NULL && s_row_labels[1] != NULL);
  assert(s_fake_y[ObjectIndex(s_value_slots[DISPLAY_HMI_VAR_LATITUDE].label)] ==
         s_fake_y[ObjectIndex(s_value_slots[DISPLAY_HMI_VAR_LONGITUDE].label)]);
  assert(s_buttons[0].action == UI_ACTION_PAGE && s_buttons[0].argument == DISPLAY_HMI_PAGE_NMEA);
  assert(strcmp(s_fake_text[ObjectIndex(s_buttons[0].label)],"Frames >")==0);
  assert(s_buttons[1].action == UI_ACTION_PAGE && s_buttons[1].argument == DISPLAY_HMI_PAGE_DETAILS);
  for (i = 0U; i < s_label_used; ++i) { AssertNoTemporaryText(s_fake_text[i]); }
  for (i = 0U; i < 8U; ++i) { s_value_valid[ids[i]] = 1U; }
  s_values[DISPLAY_HMI_VAR_GNSS_FIX] = 3U; s_values[DISPLAY_HMI_VAR_GNSS_SAT_COUNT] = 12U;
  s_values[DISPLAY_HMI_VAR_GNSS_HDOP] = 95U; s_values[DISPLAY_HMI_VAR_GNSS_SPEED] = 250U;
  s_values[DISPLAY_HMI_VAR_LATITUDE] = 225000000U; s_values[DISPLAY_HMI_VAR_LONGITUDE] = 1135000000U;
  s_values[DISPLAY_HMI_VAR_ALTITUDE] = 1250U; s_values[DISPLAY_HMI_VAR_GNSS_TIME] = 123519U;
  s_label_used = s_button_used = 0U; Display_LvglLayoutGnss();
  assert(s_button_used == 2U);
  assert(strcmp(s_value_slots[DISPLAY_HMI_VAR_GNSS_HDOP].text, "0.95") == 0);
  assert(strcmp(s_value_slots[DISPLAY_HMI_VAR_LATITUDE].text, "22.5000000") == 0);
  assert(strcmp(s_value_slots[DISPLAY_HMI_VAR_GNSS_TIME].text, "12:35:19") == 0);
  assert(s_diagnostic_copies == 0U && s_measurement_copies == 0U);
}

/** @brief 使用实际GNSS测量详情及收尾，未知行必须清为N/A。 */
static void DetailsAt(uint32_t now_ms)
{
  unsigned i;
  s_now_ms = s_last_tick_ms = now_ms; s_row_written = 0U;
  Display_LvglFormatGnssDetails(now_ms); UiRowsFinish(6U, "N/A");
  for (i = 0U; i < 6U; ++i) { AssertNoTemporaryText(s_row_text[i]); }
  assert(s_diagnostic_copies == 0U);
}

/** @brief 有/无定位均保留常规详情与独立有效位，过期不恢复旧精度和速度。 */
static void TestMeasurementDetails(void)
{
  unsigned i;
  memset(&s_measurement, 0, sizeof(s_measurement));
  s_measurement_result = PX4LITE_NOT_READY; DetailsAt(1000U);
  for (i = 0U; i < 6U; ++i) { assert(strcmp(s_row_text[i], "N/A") == 0); }
  s_measurement_result = PX4LITE_OK;
  s_measurement.last_rx_ms = s_measurement.position_update_ms = s_measurement.velocity_update_ms = 1000U;
  s_measurement.altitude_update_ms = s_measurement.time_update_ms = 1000U;
  s_measurement.status_update_ms = 1000U;
  s_measurement.valid_mask = PX4LITE_GNSS_VALID_NAV_STATUS | PX4LITE_GNSS_VALID_POSITION | PX4LITE_GNSS_VALID_VELOCITY | PX4LITE_GNSS_VALID_ALTITUDE | PX4LITE_GNSS_VALID_TIME;
  s_measurement.dimension_source = PX4LITE_GNSS_SOURCE_NMEA_GSA;
  s_measurement.dimension_update_ms = 1000U;
  s_measurement.fix_type = 1U; s_measurement.fix_dimension = 3U; s_measurement.hdop_x100 = 95U;
  s_measurement.satellites_used = 12U; s_measurement.utc_sec = 45296U;
  s_measurement.ground_speed_cms = 250U; s_measurement.heading_deg100 = 9000U;
  DetailsAt(1000U);
  assert(strcmp(s_row_text[0], "3D fix / used satellites 12 / age 0ms") == 0);
  assert(strcmp(s_row_text[1], "Position valid / age 0ms / HDOP 0.95") == 0);
  assert(strstr(s_row_text[2], "Ground speed 2.50m/s / course 90.00deg"));
  assert(strstr(s_row_text[3], "Altitude valid") && strstr(s_row_text[4], "UTC 12:34:56"));
  s_measurement.dimension_source = PX4LITE_GNSS_SOURCE_NONE; DetailsAt(1000U);
  assert(strcmp(s_row_text[0], "Position fix / used satellites 12 / age 0ms") == 0);
  s_measurement.dimension_source = PX4LITE_GNSS_SOURCE_NMEA_GSA;
  s_measurement.dimension_update_ms = 1000U - (PX4LITE_GNSS_MAX_AGE_MS + 1U);
  DetailsAt(1000U); assert(strcmp(s_row_text[0], "Position fix / used satellites 12 / age 0ms") == 0);
  s_measurement.hdop_x100 = 0U; DetailsAt(1000U); assert(strstr(s_row_text[1], "HDOP N/A"));
  s_measurement.valid_mask = PX4LITE_GNSS_VALID_NAV_STATUS | PX4LITE_GNSS_VALID_TIME; s_measurement.fix_type = 0U;
  s_measurement.fix_dimension = 0U;
  for (i = 0U; i < 3U; ++i) {
    static const uint8_t satellites[] = {0U, 2U, 12U};
    char expected[80]; s_measurement.satellites_used = satellites[i];
    DetailsAt(1000U); (void)snprintf(expected, sizeof(expected), "No fix / used satellites %u / age 0ms", (unsigned)satellites[i]);
    assert(strcmp(s_row_text[0], expected) == 0);
  }
  DetailsAt(1000U);
  assert(strstr(s_row_text[1], "Position N/A") && strcmp(s_row_text[2], "N/A") == 0);
  s_measurement.utc_sec = 0U; DetailsAt(1000U); assert(strstr(s_row_text[4], "UTC 00:00:00"));
  /* 新ACK只刷新链路时刻，旧状态及UTC不得因此继续显示。 */
  s_measurement.last_rx_ms = 2501U; DetailsAt(2501U);
  assert(strcmp(s_row_text[0], "Fix N/A / used satellites N/A") == 0 && strcmp(s_row_text[4], "UTC N/A") == 0);
  DetailsAt(2501U);
  assert(strstr(s_row_text[0], "Fix N/A") && strstr(s_row_text[1], "HDOP N/A"));
  assert(strcmp(s_row_text[2], "N/A") == 0 && strstr(s_row_text[4], "UTC N/A"));
}

/** @brief Auto兼容NMEA，保留ASCII/HEX；暂停标题仍明确真实RX与历史选帧不同。 */
static void TestNormalNmeaCapture(void)
{
  static const char sentence[] = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
  BindPool(); memset(&s_capture_status, 0, sizeof(s_capture_status));
  s_capture_status.source = PX4LITE_CAPTURE_NMEA; s_capture_status.running = 1U;
  s_capture_status.session_id = 7U; s_capture_status.count = 3U; s_capture_status.stored_count = 1U;
  s_capture_status.gnss_seen_mask=1U;s_capture_status.last_nmea_ms=1000U;
  memset(s_summary_rows,0,sizeof(s_summary_rows));s_summary_rows[0].source=PX4LITE_CAPTURE_GNSS;
  s_summary_rows[0].system_id=1U;s_summary_rows[0].msg_id=1U;s_summary_rows[0].count=3U;
  s_summary_rows[0].first_ms=100U;s_summary_rows[0].last_ms=1000U;s_summary_rows[0].latest_token=17U;
  s_capture_status.summary_count = 1U; s_current_lvgl_page = DISPLAY_HMI_PAGE_NMEA;
  s_now_ms=s_last_tick_ms = 1000U; s_inspector_mode = 0U; s_inspector_row = 0U;
  Display_LvglLayoutInspector();
  assert(strcmp(s_page_info_text, "Auto RX NMEA / Capturing / s7") == 0);
  assert(strstr(s_row_text[0], "Count 3 / stored 1") && strstr(s_row_text[2],"NMEA GGA"));
  assert(s_buttons[0].action == UI_ACTION_CAPTURE_SOURCE && s_buttons[1].action == UI_ACTION_CAPTURE_RUN);
  assert(s_buttons[2].action == UI_ACTION_CAPTURE_CLEAR && s_buttons[5].action == UI_ACTION_CAPTURE_SELECT);
  memset(&s_captured_gnss,0,sizeof(s_captured_gnss));s_captured_gnss.session_id=7U;s_captured_gnss.token=17U;
  s_captured_gnss.protocol=1U;s_captured_gnss.nmea_type=1U;s_captured_gnss.length=sizeof(sentence)-1U;
  s_captured_gnss.checksum_valid=1U;s_captured_gnss.time_ms=1000U;
  memcpy(s_captured_gnss.bytes,sentence,sizeof(sentence)-1U);
  assert(UiGnssCopySelection(7U,17U)==PX4LITE_OK);
  s_selected_session = 7U; s_selected_ready = 1U; s_inspector_mode = 1U; s_inspector_page = 0U;
  Display_LvglUpdateInspector();
  assert(strstr(s_row_text[0], "GGA") && strstr(s_row_text[0], "checksum OK"));
  assert(strstr(s_row_text[2], "$GPGGA") && strstr(s_row_text[4], "24 47 50"));
  assert(strstr(s_row_text[6],"Selected frame is history"));
  s_capture_status.filter = 2U; s_capture_status.running = 0U; Display_LvglUpdateInspector();
  assert(strcmp(s_page_info_text, "RMC RX NMEA / Paused / s7") == 0);
  s_capture_status.gnss_seen_mask=2U;s_capture_status.last_ubx_ms=1000U;Display_LvglUpdateInspector();
  assert(strcmp(s_page_info_text,"RMC RX UBX / Paused / s7")==0);
  assert(strstr(s_row_text[0],"Selected #17 NMEA GGA"));
  AssertNoTemporaryText(s_page_info_text); assert(s_diagnostic_copies == 0U);
}

/** @brief 完整392B UBX按64B分页，覆盖每个原字节；坏校验仍只显示元数据BAD。 */
static void TestUbxRawAndMessageNames(void)
{
  unsigned i,page,total=0U;static const char hex[]="0123456789ABCDEF";
  BindPool();memset(&s_capture_status,0,sizeof(s_capture_status));memset(&s_captured_gnss,0,sizeof(s_captured_gnss));
  s_capture_status.source=PX4LITE_CAPTURE_GNSS;s_capture_status.session_id=8U;s_capture_status.gnss_seen_mask=2U;
  s_capture_status.last_ubx_ms=2000U;s_capture_status.running=1U;s_now_ms=s_last_tick_ms=2000U;
  s_current_lvgl_page=DISPLAY_HMI_PAGE_NMEA;s_inspector_mode=0U;Display_LvglLayoutInspector();
  s_captured_gnss.session_id=8U;s_captured_gnss.token=18U;s_captured_gnss.protocol=2U;
  s_captured_gnss.class_id=1U;s_captured_gnss.message_id=7U;s_captured_gnss.length=392U;s_captured_gnss.time_ms=2000U;
  for(i=0U;i<392U;i++){s_captured_gnss.bytes[i]=(uint8_t)(i*7U);}
  assert(UiGnssCopySelection(8U,18U)==PX4LITE_OK);s_selected_session=8U;s_selected_ready=1U;s_inspector_mode=1U;
  for(page=0U;page<7U;page++){
    s_inspector_page=(uint8_t)page;Display_LvglUpdateInspector();
    assert(strstr(s_row_text[0],"UBX NAV-PVT") && strstr(s_row_text[0],"class 01 id 07") && strstr(s_row_text[0],"392B") && strstr(s_row_text[0],"checksum BAD"));
    for(i=2U;i<6U;i++){
      unsigned offset,j;if(total>=392U){assert(!s_row_text[i][0]);continue;}
      assert(sscanf(s_row_text[i],"%u:",&offset)==1 && offset==total);
      for(j=0U;j<16U && total<392U;j++,total++){
        uint8_t byte=s_captured_gnss.bytes[total];unsigned at=5U+j*3U;
        assert(s_row_text[i][at]==hex[byte>>4U] && s_row_text[i][at+1U]==hex[byte&15U]);
      }
      assert(s_fake_font[ObjectIndex(s_row_labels[i])]==&display_lvgl_hex_14);
    }
  }
  assert(total==392U && strstr(s_row_text[1],"bytes 384..391 / page 7/7"));
  s_captured_gnss.time_ms=2001U;assert(UiGnssCopySelection(8U,18U)==PX4LITE_OK);
  Display_LvglUpdateInspector();assert(strstr(s_row_text[1],"age0ms"));
  s_now_ms=s_last_tick_ms=10000U;Display_LvglUpdateInspector();assert(strstr(s_row_text[1],"age7999ms"));
  s_now_ms=s_last_tick_ms=2000U;
  assert(!strcmp(UiGnssFrameName(2U,1U,4U,0U),"UBX NAV-DOP"));
  assert(!strcmp(UiGnssFrameName(2U,1U,0x35U,0U),"UBX NAV-SAT"));
  assert(!strcmp(UiGnssFrameName(2U,10U,4U,0U),"UBX MON-VER"));
  assert(!strcmp(UiGnssFrameName(2U,5U,1U,0U),"UBX ACK-ACK"));
  assert(!strcmp(UiGnssFrameName(2U,5U,0U,0U),"UBX ACK-NAK"));
  assert(!strcmp(UiGnssFrameName(2U,1U,0x10007U,0U),"UBX Other"));
  s_capture_status.summary_count=2U;memset(s_summary_rows,0,sizeof(s_summary_rows));
  s_summary_rows[0].system_id=1U;s_summary_rows[0].msg_id=2U;s_summary_rows[0].count=1U;
  s_summary_rows[1].system_id=2U;s_summary_rows[1].component_id=1U;s_summary_rows[1].msg_id=0x35U;s_summary_rows[1].count=1U;
  s_inspector_mode=0U;Display_LvglUpdateInspector();
  assert(strstr(s_row_text[2],"NMEA RMC") && strstr(s_row_text[3],"UBX NAV-SAT"));
  assert(s_fake_font[ObjectIndex(s_row_labels[4])]==&display_lvgl_numeric_14);
  assert(s_diagnostic_copies==0U);
}

/** @brief 常驻RX独立于捕获运行/筛选，支持0ms/回绕；等待和陈旧状态不能冒充正在收数。 */
static void TestAutoProtocolAndSession(void)
{
  static const uint8_t order[11]={0U,5U,6U,1U,2U,3U,4U,7U,8U,9U,0U};
  unsigned i,before;BindPool();memset(&s_capture_status,0,sizeof(s_capture_status));
  s_capture_status.source=PX4LITE_CAPTURE_GNSS;s_capture_status.session_id=9U;
  s_current_lvgl_page=DISPLAY_HMI_PAGE_NMEA;s_inspector_mode=0U;s_now_ms=s_last_tick_ms=0U;
  Display_LvglLayoutInspector();assert(strstr(s_page_info_text,"Auto RX Waiting / Paused"));
  assert(strstr(s_row_text[2],"No stored frames") && strstr(s_row_text[6],"No valid NMEA/UBX"));
  s_capture_status.gnss_seen_mask=2U;s_capture_status.last_ubx_ms=0U;Display_LvglUpdateInspector();
  assert(strstr(s_page_info_text,"Auto RX UBX / Paused"));
  s_capture_status.gnss_seen_mask=3U;s_capture_status.last_nmea_ms=0U;Display_LvglUpdateInspector();assert(strstr(s_page_info_text,"RX Mixed"));
  s_now_ms=s_last_tick_ms=3001U;Display_LvglUpdateInspector();assert(strstr(s_page_info_text,"RX Stale"));
  s_capture_status.last_ubx_ms=3001U;Display_LvglUpdateInspector();assert(strstr(s_page_info_text,"RX UBX"));
  s_capture_status.last_ubx_ms=UINT32_MAX-100U;s_capture_status.gnss_seen_mask=2U;s_now_ms=s_last_tick_ms=50U;
  Display_LvglUpdateInspector();assert(strstr(s_page_info_text,"RX UBX"));
  assert(UiGnssAge(1000U,1001U)==0U && UiGnssAge(1000U,4000U)==0U);
  assert(UiGnssAge(6000U,1000U)==5000U && UiGnssAge(50U,UINT32_MAX-100U)==151U);
  assert(UiGnssAge(UINT32_MAX,0U)==0U);
  s_capture_status.last_ubx_ms=51U;Display_LvglUpdateInspector();assert(strstr(s_page_info_text,"RX UBX"));
  s_capture_status.summary_count=1U;memset(&s_summary_rows[0],0,sizeof(s_summary_rows[0]));
  s_summary_rows[0].system_id=2U;s_summary_rows[0].component_id=1U;s_summary_rows[0].msg_id=7U;
  s_summary_rows[0].count=2U;s_summary_rows[0].first_ms=49U;s_summary_rows[0].last_ms=51U;
  Display_LvglUpdateInspector();assert(strstr(s_row_text[2],"age0ms"));
  s_now_ms=s_last_tick_ms=10000U;s_summary_rows[0].first_ms=0U;s_summary_rows[0].last_ms=1000U;
  Display_LvglUpdateInspector();assert(strstr(s_row_text[2],"age9000ms") && strstr(s_page_info_text,"RX Stale"));
  s_capture_status.summary_count=0U;
  for(i=0U;i<10U;i++){
    s_capture_status.filter=order[i];before=s_capture_requests;UiGnssCycleFilter(&s_capture_status);
    assert(s_capture_requests==before+1U && s_last_capture_request.operation==PX4LITE_CAPTURE_SELECT);
    assert(s_last_capture_request.source==PX4LITE_CAPTURE_GNSS && s_last_capture_request.filter==order[i+1U]);
  }
  s_capture_status.pending=1U;before=s_capture_requests;UiGnssCycleFilter(&s_capture_status);assert(before==s_capture_requests);
  s_capture_status.pending=0U;s_selected_ready=1U;s_selected_session=9U;s_inspector_mode=1U;
  s_capture_status.session_id=10U;s_capture_status.running=0U;Display_LvglUpdateInspector();
  assert(!s_selected_ready && strstr(s_row_text[0],"No selected GNSS frame"));
  assert(UiGnssCopySelection(9U,18U)==PX4LITE_NOT_READY);
  s_capture_status_result=PX4LITE_NOT_READY;Display_LvglUpdateInspector();
  assert(!strcmp(s_page_info_text,"Capture status unavailable") && strstr(s_row_text[0],"no live RX claim"));
  s_capture_status_result=PX4LITE_OK;
}

/** @brief 正常页面闭环回归，不启动实际LVGL或任何硬件服务。 */
int main(void)
{
  TestNormalGnssLayout(); TestMeasurementDetails(); TestNormalNmeaCapture();
  TestUbxRawAndMessageNames();TestAutoProtocolAndSession();
  puts("GNSS Frames UI: PASS (normal measurements/NMEA+UBX complete raw/Auto RX/filters/session isolation)");
  return 0;
}
