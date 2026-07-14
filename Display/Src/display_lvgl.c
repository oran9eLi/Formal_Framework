/**
 * @file display_lvgl.c
 * @brief LVGL renderer backend for the Display facade.
 *
 * @details
 * LVGL is an implementation detail of the Display layer. The module consumes
 * values already prepared by Display_PrepareSnapshot() and never reads
 * Framework topics, BSP buffers, or driver private state directly.
 */

#include "display_lvgl.h"
#include "display_logo.h"

#include "app_data_api.h"
#include "display_lvgl_font_zh.h"
#include "display_pages.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "px4lite_config.h"
#include "px4lite_platform.h"
#include "px4lite_remote_telemetry.h"
#include "lvgl.h"
#include "px4lite_faults.h"

#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* 地平仪绘制参数：每度对应的像素、横滚/俯仰方向符号。
   若实测方向与显示相反，只需把对应 SIGN 改成 -1.0f。 */
#define HZ_PX_PER_DEG  2.6f
#define HZ_ROLL_SIGN   1.0f
#define HZ_PITCH_SIGN  1.0f
#define HZ_YAW_SIGN    1.0f
/* 偏航常量偏置（单位：度）。用于补偿 IMU 相对机头的安装转角，或把相对零位
   对到已知航向。注意：当前 yaw 为陀螺积分相对航向（开机置零、随时间漂移），
   该偏置只能修正固定安装偏差，无法消除漂移或得到真北——真北需磁力计/GNSS 航向。 */
#define HZ_YAW_OFFSET_DEG  0.0f

#define DISPLAY_LVGL_WIDTH           800U
#define DISPLAY_LVGL_HEIGHT          480U
#define DISPLAY_LVGL_HEADER_H        64U
#define DISPLAY_LVGL_BODY_Y          76U
#define DISPLAY_LVGL_BODY_H          330U
#define DISPLAY_LVGL_FOOTER_Y        424U
#define DISPLAY_LVGL_FOOTER_H        56U
#define DISPLAY_LVGL_CARD_RADIUS     6U
#define DISPLAY_LVGL_VALUE_TEXT_LEN  32U
#define DISPLAY_LVGL_STATUS_COUNT    11U
#define DISPLAY_LVGL_TAB_COUNT       6U
#define DISPLAY_LVGL_MOTOR_COUNT     4U
#define DISPLAY_LVGL_REMOTE_ROWS     16U   /* 通信连接页远端节点列表最大行数 */
#define DISPLAY_LVGL_REMOTE_LIST_REFRESH_MS 1000U /* 通信连接页节点列表周期重建间隔 */
#define DISPLAY_LVGL_LOG_ROWS        9U
/* 告警行容量：数据侧最多 PX4LITE_MODULE_COUNT + 5G占位/电机/主控，取 16 与 APP_DISPLAY_ALARM_MAX 对齐。
   超过可视高度时容器可竖向滚动(见 Display_LvglCreateAlarmTable/SummaryAlarmList)。 */
#define DISPLAY_LVGL_ALARM_ROWS      16U
#define DISPLAY_LVGL_ALARM_ROW_H     50
#define DISPLAY_LVGL_VALUE_LABEL_EXTRA_COUNT 2U

/*
 * 自检汇总告警列表的中文字体。窄卡片要放下"代码+模块+原因"三项，
 * 需更小字号；当前字库只有 16px 子集，先回退到 16px（两行紧凑布局下可显示）。
 * 重新生成更小字号字体（如 display_lvgl_font_zh_12）并在 display_lvgl_font_zh.h
 * 声明 extern 后，仅需把此宏改为指向它即可，无需改动布局代码。
 */
#ifndef DISPLAY_LVGL_FONT_ZH_SMALL
#define DISPLAY_LVGL_FONT_ZH_SMALL display_lvgl_font_zh_16
#endif

typedef struct {
  lv_obj_t *time_label;
  lv_obj_t *msg_label;
  char time_text[9];
} Display_LvglLogRow_t;

typedef struct {
  lv_obj_t *row;          /* 行容器(滚动体的子对象)；空行隐藏并停靠到 y=0 以收缩滚动范围。 */
  lv_obj_t *bar;
  lv_obj_t *code_label;
  lv_obj_t *module_label;
  lv_obj_t *reason_label;
  char code_text[8];
} Display_LvglAlarmRow_t;

typedef struct {
  lv_obj_t *label;
  char text[DISPLAY_LVGL_VALUE_TEXT_LEN];
} Display_LvglValueSlot_t;

typedef struct {
  Display_HmiVariableId_t id;
  const char *name;
} Display_LvglStatusItem_t;

typedef struct {
  Display_HmiPage_t page;
  const char *name;
} Display_LvglTabItem_t;

static const Display_LvglStatusItem_t s_status_items[DISPLAY_LVGL_STATUS_COUNT] = {
    {DISPLAY_HMI_VAR_SELF_CHECK_GNSS, "\xE5""\xAE""\x9A""\xE4""\xBD""\x8D""\xE6""\xA8""\xA1""\xE5""\x9D""\x97"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MPU6050, "\xE5""\xA7""\xBF""\xE6""\x80""\x81""\xE6""\xA8""\xA1""\xE5""\x9D""\x97"},
    {DISPLAY_HMI_VAR_SELF_CHECK_BME280, "\xE7""\x8E""\xAF""\xE5""\xA2""\x83""\xE6""\xA8""\xA1""\xE5""\x9D""\x97"},
    {DISPLAY_HMI_VAR_SELF_CHECK_LORA, "LoRa""\xE9""\x80""\x9A""\xE4""\xBF""\xA1"},
    {DISPLAY_HMI_VAR_SELF_CHECK_5GA, "5G""\xE9""\x80""\x9A""\xE4""\xBF""\xA1"},
    {DISPLAY_HMI_VAR_SELF_CHECK_REMOTEID, "RemoteID"},
    {DISPLAY_HMI_VAR_SELF_CHECK_SD, "\xE5""\xAD""\x98""\xE5""\x82""\xA8""\xE6""\xA8""\xA1""\xE5""\x9D""\x97"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR, "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE4""\xB8""\x80"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_2, "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE4""\xBA""\x8C"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_3, "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE4""\xB8""\x89"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_4, "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE5""\x9B""\x9B"},
};

static const Display_LvglTabItem_t s_tabs[DISPLAY_LVGL_TAB_COUNT] = {
    {DISPLAY_HMI_PAGE_SELF_CHECK, "\xE8""\x87""\xAA""\xE6""\xA3""\x80"},
    {DISPLAY_HMI_PAGE_FLIGHT, "\xE9""\xA3""\x9E""\xE8""\xA1""\x8C"},
    {DISPLAY_HMI_PAGE_AIRCRAFT, "\xE5""\xA7""\xBF""\xE6""\x80""\x81"},
    {DISPLAY_HMI_PAGE_DATA, "\xE5""\xAE""\x9A""\xE4""\xBD""\x8D"},
    {DISPLAY_HMI_PAGE_MOTOR, "\xE7""\x94""\xB5""\xE6""\x9C""\xBA"},
    {DISPLAY_HMI_PAGE_ALARM, "\xE5""\x91""\x8A""\xE8""\xAD""\xA6"},
};

static Display_LvglValueSlot_t s_value_slots[DISPLAY_HMI_VAR_COUNT];
static lv_obj_t *s_value_extra_labels[DISPLAY_HMI_VAR_COUNT][DISPLAY_LVGL_VALUE_LABEL_EXTRA_COUNT];
static uint32_t s_values[DISPLAY_HMI_VAR_COUNT];
static uint8_t s_value_valid[DISPLAY_HMI_VAR_COUNT];
static const lv_img_dsc_t s_logo_img_dsc = {
    .header = { .cf = LV_IMG_CF_TRUE_COLOR_CHROMA_KEYED, .always_zero = 0, .reserved = 0,
                .w = DISPLAY_HEADER_LOGO_WIDTH, .h = DISPLAY_HEADER_LOGO_HEIGHT },
    .data_size = DISPLAY_HEADER_LOGO_WIDTH * DISPLAY_HEADER_LOGO_HEIGHT * 2U,
    .data = display_logo_rgb565_data,
};

static lv_obj_t *s_screen;
static lv_obj_t *s_status_leds[DISPLAY_LVGL_STATUS_COUNT];
static lv_obj_t *s_motor_pwm_bars[DISPLAY_LVGL_MOTOR_COUNT];
static lv_obj_t *s_motor_pulse_labels[DISPLAY_LVGL_MOTOR_COUNT];
static char s_motor_pulse_text[DISPLAY_LVGL_MOTOR_COUNT][16];
static uint8_t s_motor_slider_dragging[DISPLAY_LVGL_MOTOR_COUNT];
static lv_obj_t *s_log_alarm_label;
static lv_obj_t *s_attitude_obj;
static int16_t s_attitude_roll_deg10;
static int16_t s_attitude_pitch_deg10;
static int16_t s_attitude_yaw_deg10;
static Display_LvglLogRow_t s_log_rows[DISPLAY_LVGL_LOG_ROWS];
static Display_LvglAlarmRow_t s_alarm_rows[DISPLAY_LVGL_ALARM_ROWS];
/* 告警行数据(packed = (source_id<<16)|fault_code)，由 Display_LvglSetAlarmRows 批量写入，
   取代原 ALARM_ROW1..5 五个 HMI 变量以支持最多 16 行滚动。 */
static uint32_t s_alarm_packed[DISPLAY_LVGL_ALARM_ROWS];
static uint16_t s_alarm_count;
static uint8_t s_log_visible_rows;
static Display_HmiPage_t s_current_lvgl_page = DISPLAY_HMI_PAGE_SELF_CHECK;
static Display_HmiPage_t s_requested_page    = DISPLAY_HMI_PAGE_SELF_CHECK;
static Display_HmiPage_t s_page_before_hidden = DISPLAY_HMI_PAGE_SELF_CHECK;
static uint8_t s_page_change_requested;
static uint8_t s_page_rebuild_requested;
static uint8_t s_lvgl_core_ready;
static uint8_t s_lvgl_display_ready;
static uint8_t s_control_update_active;
static uint32_t s_last_tick_ms;
static uint32_t s_next_remote_list_rebuild_ms;
static uint32_t s_remote_list_sig; /* 通信连接页节点列表内容签名(node_id+state)，仅内容变化才整页重建，避免每秒屏闪 */
static App_RemoteNodeView_t s_remote_node_views[DISPLAY_LVGL_REMOTE_ROWS];
static char s_remote_node_texts[DISPLAY_LVGL_REMOTE_ROWS][64];

/**
 * @brief 计算远端节点列表内容签名，只纳入 node_id 与在线状态。
 * @note  丢包率/接收计数持续微变不进签名，避免通信连接页每秒整页重建导致屏闪；
 *        仅当节点上下线或状态切换时签名变化，才触发一次重建。
 */
static uint32_t Display_LvglRemoteNodeSig(const App_RemoteNodeView_t *views, uint8_t count)
{
  uint32_t sig = 2166136261UL ^ (uint32_t)count;
  uint8_t i;

  for (i = 0U; i < count; i++) {
    sig = (sig * 16777619UL) ^ (uint32_t)views[i].node_id;
    sig = (sig * 16777619UL) ^ (uint32_t)views[i].state;
  }
  return sig;
}

/**
 * @brief 远端视图下的触摸活动保活：任何点按都刷新远端视图有效期。
 */
static void Display_LvglTouchActivity(void)
{
  if (App_GetRemoteDisplayMode() == PX4LITE_REMOTE_MODE_REMOTE) {
    (void)App_SetRemoteViewEnabled(1U, s_last_tick_ms);
  }
}

/**
 * @brief 延迟切回本机数据源，并在需要时重建当前页。
 */
static void Display_LvglRequestLocalView(Display_HmiPage_t target_page, uint8_t force_rebuild)
{
  (void)App_SetRemoteViewEnabled(0U, s_last_tick_ms);
  s_requested_page          = target_page;
  s_page_change_requested   = 1U;
  s_page_rebuild_requested |= force_rebuild;
}

/**
 * @brief Return the formal page title used by the LVGL header.
 */
static const char *Display_LvglPageTitle(Display_HmiPage_t page)
{
  switch (page) {
    case DISPLAY_HMI_PAGE_SELF_CHECK:
      return "\xE4""\xB8""\x8A""\xE7""\x94""\xB5""\xE8""\x87""\xAA""\xE6""\xA3""\x80";
    case DISPLAY_HMI_PAGE_FLIGHT:
      return "\xE9""\xA3""\x9E""\xE8""\xA1""\x8C""\xE6""\x95""\xB0""\xE6""\x8D""\xAE";
    case DISPLAY_HMI_PAGE_AIRCRAFT:
      return "\xE9""\xA3""\x9E""\xE6""\x9C""\xBA""\xE5""\xA7""\xBF""\xE6""\x80""\x81";
    case DISPLAY_HMI_PAGE_DATA:
      return "\xE5""\xAE""\x9A""\xE4""\xBD""\x8D""\xE6""\x95""\xB0""\xE6""\x8D""\xAE";
    case DISPLAY_HMI_PAGE_MOTOR:
      return "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE6""\x8E""\xA7""\xE5""\x88""\xB6";
    case DISPLAY_HMI_PAGE_ALARM:
      return "\xE5""\x91""\x8A""\xE8""\xAD""\xA6""\xE5""\x88""\x97""\xE8""\xA1""\xA8";
    case DISPLAY_HMI_PAGE_HIDDEN:
      return "\xE9""\x80""\x9A""\xE4""\xBF""\xA1""\xE8""\xBF""\x9E""\xE6""\x8E""\xA5";
    default:
      return "\xE6""\x98""\xBE""\xE7""\xA4""\xBA";
  }
}

/**
 * @brief Map status value to LVGL palette color.
 */
static lv_color_t Display_LvglStatusColor(uint32_t value)
{
  /* 状态灯只保留红黄绿三态：在线=绿，启动/降级=黄，其余(离线/故障/未就绪/未初始化)=红。
     原 value 0(未就绪)灰色已并入红色，不再出现第四种颜色。 */
  if (value == 2U) {
    return lv_palette_main(LV_PALETTE_GREEN);
  }
  if (value == 1U) {
    return lv_palette_main(LV_PALETTE_AMBER);
  }
  return lv_palette_main(LV_PALETTE_RED);
}

/**
 * @brief Get readable status text for self-check fields.
 */
static const char *Display_LvglStatusText(uint32_t value)
{
  /* 状态文字只保留 正常/警告/故障；原 value 0 的"未就绪"已并入"故障"(与红灯一致)。 */
  if (value == 2U) {
    return "\xE6""\xAD""\xA3""\xE5""\xB8""\xB8";
  }
  if (value == 1U) {
    return "\xE8""\xAD""\xA6""\xE5""\x91""\x8A";
  }
  return "\xE6""\x95""\x85""\xE9""\x9A""\x9C";
}

/**
 * @brief 把 GNSS 解算 fix 等级映射为定位状态文字。
 * @details
 * 本地来自 Px4Lite_GnssDisplayFixState(0/2/3/4)，远端来自 GPS_RAW 的 MAVLink
 * fix_type(0..8)。映射：0/1 无定位、2 2D定位、3 3D定位、4 差分(DGPS)、5/6 RTK。
 * 注：字库缺 差/浮/固 三字，差分/RTK 用 ASCII 词(DGPS/RTK-F/RTK-X)避免显示为空框；
 * 后续若要中文"差分/RTK浮动/RTK固定"，需按 lvgl-font-add-glyph 补字库再改这里。
 * ATGM336H 常见只有 0/2/3(无/2D/3D)，DGPS 偶发，RTK 基本不出现。
 */
static const char *Display_LvglGnssFixText(uint32_t value)
{
  switch (value) {
    case 0U:
    case 1U:
      return "\xE6""\x97""\xA0""\xE5""\xAE""\x9A""\xE4""\xBD""\x8D"; /* 无定位 */
    case 2U:
      return "2D\xE5""\xAE""\x9A""\xE4""\xBD""\x8D"; /* 2D定位 */
    case 4U:
      return "DGPS"; /* 差分(字库缺"差") */
    case 5U:
      return "RTK-F"; /* RTK 浮动 */
    case 6U:
      return "RTK-X"; /* RTK 固定 */
    case 3U:
    default:
      return "3D\xE5""\xAE""\x9A""\xE4""\xBD""\x8D"; /* 3D定位 */
  }
}

/**
 * @brief Clear active object pointers before rebuilding the current page.
 */
static void Display_LvglClearActiveObjects(void)
{
  uint16_t i;
  uint8_t j;

  for (i = 0U; i < DISPLAY_HMI_VAR_COUNT; i++) {
    s_value_slots[i].label = 0;
    for (j = 0U; j < DISPLAY_LVGL_VALUE_LABEL_EXTRA_COUNT; j++) {
      s_value_extra_labels[i][j] = 0;
    }
  }
  for (i = 0U; i < DISPLAY_LVGL_STATUS_COUNT; i++) {
    s_status_leds[i] = 0;
  }
  for (i = 0U; i < DISPLAY_LVGL_MOTOR_COUNT; i++) {
    s_motor_pwm_bars[i] = 0;
    s_motor_pulse_labels[i] = 0;
    s_motor_pulse_text[i][0] = '\0';
    s_motor_slider_dragging[i] = 0U;
  }
  for (i = 0U; i < DISPLAY_LVGL_LOG_ROWS; i++) {
    s_log_rows[i].time_label = 0;
    s_log_rows[i].msg_label  = 0;
  }
  for (i = 0U; i < DISPLAY_LVGL_ALARM_ROWS; i++) {
    s_alarm_rows[i].row          = 0;
    s_alarm_rows[i].bar          = 0;
    s_alarm_rows[i].code_label   = 0;
    s_alarm_rows[i].module_label = 0;
    s_alarm_rows[i].reason_label = 0;
  }
  s_log_alarm_label = 0;
  s_attitude_obj = 0;
  s_log_visible_rows = 0U;
}

/**
 * @brief Create one static label.
 */
static lv_obj_t *Display_LvglCreateLabel(lv_obj_t *parent, const char *text, lv_coord_t x, lv_coord_t y, const lv_font_t *font, lv_color_t color)
{
  lv_obj_t *label;

  label = lv_label_create(parent);
  lv_label_set_text_static(label, text);
  lv_obj_set_style_text_font(label, font, 0);
  lv_obj_set_style_text_color(label, color, 0);
  lv_obj_set_pos(label, x, y);
  return label;
}

static lv_obj_t *Display_LvglCreateClipLabel(lv_obj_t *parent, const char *text, lv_coord_t x, lv_coord_t y, lv_coord_t w, const lv_font_t *font, lv_color_t color)
{
  lv_obj_t *label;

  label = Display_LvglCreateLabel(parent, text, x, y, font, color);
  lv_obj_set_width(label, w);
  lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
  return label;
}

static lv_obj_t *Display_LvglCreateCenteredLabel(lv_obj_t *parent, const char *text, lv_coord_t x, lv_coord_t y, lv_coord_t w, const lv_font_t *font, lv_color_t color)
{
  lv_obj_t *label;

  label = Display_LvglCreateClipLabel(parent, text, x, y, w, font, color);
  lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
  return label;
}

/**
 * @brief Create one card panel with a title.
 */
static lv_obj_t *Display_LvglCreateCard(lv_obj_t *parent, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h, const char *title)
{
  lv_obj_t *card;

  card = lv_obj_create(parent);
  lv_obj_set_size(card, w, h);
  lv_obj_set_pos(card, x, y);
  lv_obj_set_style_radius(card, DISPLAY_LVGL_CARD_RADIUS, 0);
  lv_obj_set_style_bg_color(card, lv_color_hex(0x13202E), 0);
  lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(card, 1, 0);
  lv_obj_set_style_border_color(card, lv_color_hex(0x304357), 0);
  lv_obj_set_style_pad_all(card, 0, 0);
  lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

  (void)Display_LvglCreateCenteredLabel(card, title, 4, 10, (lv_coord_t)(w - 8), &display_lvgl_font_zh_16, lv_color_hex(0xDCE8F2));
  return card;
}

/**
 * @brief Draw a small status dot.
 */
static lv_obj_t *Display_LvglCreateStatusDot(lv_obj_t *parent, lv_coord_t x, lv_coord_t y)
{
  lv_obj_t *dot;

  dot = lv_obj_create(parent);
  lv_obj_set_size(dot, 14, 14);
  lv_obj_set_pos(dot, x, y);
  lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(dot, lv_palette_main(LV_PALETTE_GREY), 0);
  lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(dot, 0, 0);
  lv_obj_clear_flag(dot, LV_OBJ_FLAG_SCROLLABLE);
  return dot;
}

/**
 * @brief Format a signed fixed-point value with one decimal digit.
 */
static void Display_LvglFormatSignedFixed1(char *text, int32_t raw, const char *unit)
{
  uint32_t abs_value;
  const char *sign;

  if (raw < 0) {
    sign      = "-";
    abs_value = (uint32_t)(-raw);
  } else {
    sign      = "";
    abs_value = (uint32_t)raw;
  }
  (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%s%lu.%lu%s", sign, (unsigned long)(abs_value / 10U), (unsigned long)(abs_value % 10U), unit);
}

/**
 * @brief Format a signed coordinate in 1e-7 degrees.
 */
static void Display_LvglFormatCoordinate(char *text, int32_t raw)
{
  uint32_t abs_value;
  const char *sign;

  if (raw < 0) {
    sign      = "-";
    abs_value = (uint32_t)(-raw);
  } else {
    sign      = "";
    abs_value = (uint32_t)raw;
  }
  (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%s%lu.%07lu", sign, (unsigned long)(abs_value / 10000000U), (unsigned long)(abs_value % 10000000U));
}

/**
 * @brief Format one HMI value into its static text slot.
 */
static void Display_LvglFormatValue(Display_HmiVariableId_t id, uint32_t value)
{
  char *text;
  uint32_t hh;
  uint32_t mm;
  uint32_t ss;

  if (id >= DISPLAY_HMI_VAR_COUNT) {
    return;
  }

  text = s_value_slots[id].text;
  switch (id) {
    case DISPLAY_HMI_VAR_CLOCK_TIME:
    case DISPLAY_HMI_VAR_GNSS_TIME:
      hh = (value / 10000U) % 100U;
      mm = (value / 100U) % 100U;
      ss = value % 100U;
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%02lu:%02lu:%02lu", (unsigned long)hh, (unsigned long)mm, (unsigned long)ss);
      break;
    case DISPLAY_HMI_VAR_DATE:
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%04lu-%02lu-%02lu", (unsigned long)(value / 10000U), (unsigned long)((value / 100U) % 100U), (unsigned long)(value % 100U));
      break;
    case DISPLAY_HMI_VAR_UPTIME_MS:
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%lu""\xE7""\xA7""\x92", (unsigned long)(value / 1000U));
      break;
    case DISPLAY_HMI_VAR_VIEW_NODE_ID:
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "DCDW-%03lu", (unsigned long)value);
      break;
    case DISPLAY_HMI_VAR_FLIGHT_TIME_S:
      hh = value / 3600U;
      mm = (value / 60U) % 60U;
      ss = value % 60U;
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%02lu:%02lu:%02lu", (unsigned long)hh, (unsigned long)mm, (unsigned long)ss);
      break;
    case DISPLAY_HMI_VAR_BATTERY_VOLTAGE:
    case DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE:
      {
        uint32_t deci_v = (value + 5U) / 10U; /* 0.01V 单位四舍五入到 0.1V，仅保留一位小数 */
        (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%lu.%lu""\xE4""\xBC""\x8F", (unsigned long)(deci_v / 10U), (unsigned long)(deci_v % 10U));
      }
      break;
    case DISPLAY_HMI_VAR_BATTERY_CURRENT:
    case DISPLAY_HMI_VAR_MOTOR_BAT_CURRENT:
      /* 0.1A 单位，保留一位小数，单位"安"(U+5B89) */
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%lu.%lu""\xE5""\xAE""\x89", (unsigned long)(value / 10U), (unsigned long)(value % 10U));
      break;
    case DISPLAY_HMI_VAR_BATTERY_PERCENT:
    case DISPLAY_HMI_VAR_MOTOR_BAT_PERCENT:
    case DISPLAY_HMI_VAR_MOTOR_PWM_1:
    case DISPLAY_HMI_VAR_MOTOR_PWM_2:
    case DISPLAY_HMI_VAR_MOTOR_PWM_3:
    case DISPLAY_HMI_VAR_MOTOR_PWM_4:
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%lu%%", (unsigned long)value);
      break;
    case DISPLAY_HMI_VAR_GNSS_FIX:
      /* 定位状态显示解算 fix 等级(无定位/2D/3D/差分…)，与自检模块态文字区分开。 */
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%s", Display_LvglGnssFixText(value));
      break;
    case DISPLAY_HMI_VAR_SYSTEM_STATUS:
    case DISPLAY_HMI_VAR_SELF_CHECK_GNSS:
    case DISPLAY_HMI_VAR_SELF_CHECK_MPU6050:
    case DISPLAY_HMI_VAR_SELF_CHECK_BME280:
    case DISPLAY_HMI_VAR_SELF_CHECK_LORA:
    case DISPLAY_HMI_VAR_SELF_CHECK_5GA:
    case DISPLAY_HMI_VAR_SELF_CHECK_REMOTEID:
    case DISPLAY_HMI_VAR_SELF_CHECK_SD:
    case DISPLAY_HMI_VAR_SELF_CHECK_MOTOR:
    case DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_2:
    case DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_3:
    case DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_4:
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%s", Display_LvglStatusText(value));
      break;
    case DISPLAY_HMI_VAR_GNSS_HDOP:
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%lu.%02lu", (unsigned long)(value / 100U), (unsigned long)(value % 100U));
      break;
    case DISPLAY_HMI_VAR_LORA_LOSS_RATE:
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%lu.%lu%%", (unsigned long)(value / 10U), (unsigned long)(value % 10U));
      break;
    case DISPLAY_HMI_VAR_LATITUDE:
    case DISPLAY_HMI_VAR_LONGITUDE:
      Display_LvglFormatCoordinate(text, (int32_t)value);
      break;
    case DISPLAY_HMI_VAR_ALTITUDE:
      {
        int32_t raw = (int32_t)value;
        uint32_t abs_value;
        const char *sign;

        if (raw < 0) {
          sign      = "-";
          abs_value = (uint32_t)(-raw);
        } else {
          sign      = "";
          abs_value = (uint32_t)raw;
        }
        (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%s%lu.%03lu""\xE7""\xB1""\xB3", sign, (unsigned long)(abs_value / 1000U), (unsigned long)(abs_value % 1000U));
      }
      break;
    case DISPLAY_HMI_VAR_ROLL:
    case DISPLAY_HMI_VAR_PITCH:
    case DISPLAY_HMI_VAR_YAW:
      Display_LvglFormatSignedFixed1(text, (int16_t)value, "\xE5""\xBA""\xA6");
      break;
    case DISPLAY_HMI_VAR_TEMPERATURE:
      Display_LvglFormatSignedFixed1(text, (int16_t)value, "\xE5""\xBA""\xA6");
      break;
    case DISPLAY_HMI_VAR_HUMIDITY:
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%lu.%lu%%", (unsigned long)(value / 10U), (unsigned long)(value % 10U));
      break;
    case DISPLAY_HMI_VAR_PRESSURE:
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%lu""\xE5""\xB8""\x95", (unsigned long)value);
      break;
    case DISPLAY_HMI_VAR_GNSS_SPEED:
      /* 值为地速 cm/s，转 m/s 显示一位小数(此前落到 default 直接打印原始 cm/s 数字)。 */
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%lu.%lum/s", (unsigned long)(value / 100U), (unsigned long)((value / 10U) % 10U));
      break;
    case DISPLAY_HMI_VAR_SELF_CHECK_ERROR_CODE:
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%lu", (unsigned long)(value & 0xFFFFU));
      break;
    case DISPLAY_HMI_VAR_ALARM_ACTIVE_MASK:
    case DISPLAY_HMI_VAR_ALARM_ROW1_CODE:
    case DISPLAY_HMI_VAR_ALARM_ROW2_CODE:
    case DISPLAY_HMI_VAR_ALARM_ROW3_CODE:
    case DISPLAY_HMI_VAR_ALARM_ROW4_CODE:
    case DISPLAY_HMI_VAR_ALARM_ROW5_CODE:
      if (value == 0U) {
        (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "--");
      } else {
        (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%lu", (unsigned long)value);
      }
      break;
    default:
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%lu", (unsigned long)value);
      break;
  }
}

/**
 * @brief Create one value label and immediately populate cached data.
 */
static void Display_LvglCreateValueLabel(lv_obj_t *parent, Display_HmiVariableId_t id, lv_coord_t x, lv_coord_t y, lv_coord_t w, const lv_font_t *font)
{
  lv_obj_t *label;
  uint8_t i;

  if (id >= DISPLAY_HMI_VAR_COUNT) {
    return;
  }

  label = lv_label_create(parent);
  if (s_value_slots[id].label == 0) {
    s_value_slots[id].label = label;
  } else {
    for (i = 0U; i < DISPLAY_LVGL_VALUE_LABEL_EXTRA_COUNT; i++) {
      if (s_value_extra_labels[id][i] == 0) {
        s_value_extra_labels[id][i] = label;
        break;
      }
    }
  }
  if ((id == DISPLAY_HMI_VAR_VIEW_NODE_ID) ||
      (id == DISPLAY_HMI_VAR_DATE) ||
      (id == DISPLAY_HMI_VAR_CLOCK_TIME) ||
      (id == DISPLAY_HMI_VAR_GNSS_TIME) ||
      (id == DISPLAY_HMI_VAR_FLIGHT_TIME_S)) {
    lv_obj_set_style_text_font(label, (font != 0) ? font : &lv_font_montserrat_14, 0);
    /* DCDW-xxx 右对齐：文字贴右缘，使其与下方本地/远端按钮右缘对齐。 */
    if (id == DISPLAY_HMI_VAR_VIEW_NODE_ID) { lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_RIGHT, 0); }
  } else {
    lv_obj_set_style_text_font(label, &display_lvgl_font_zh_16, 0);
  }
  lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_width(label, w);
  lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
  lv_obj_set_pos(label, x, y);

  if (s_value_valid[id] != 0U) {
    Display_LvglFormatValue(id, s_values[id]);
  } else {
    (void)snprintf(s_value_slots[id].text, DISPLAY_LVGL_VALUE_TEXT_LEN, "--");
  }
  lv_label_set_text_static(label, s_value_slots[id].text);
}

/**
 * @brief Create one named value row.
 */
static void Display_LvglCreateValueRow(lv_obj_t *parent, Display_HmiVariableId_t id, const char *name, lv_coord_t x, lv_coord_t y, lv_coord_t value_x)
{
  (void)Display_LvglCreateLabel(parent, name, x, y, &display_lvgl_font_zh_16, lv_color_hex(0xA8B7C7));
  Display_LvglCreateValueLabel(parent, id, value_x, y - 1, 118, &lv_font_montserrat_16);
}

/**
 * @brief Update one active status LED.
 */
static void Display_LvglUpdateStatusLed(Display_HmiVariableId_t id, uint32_t value)
{
  uint16_t i;

  for (i = 0U; i < DISPLAY_LVGL_STATUS_COUNT; i++) {
    if ((s_status_items[i].id == id) && (s_status_leds[i] != 0)) {
      lv_obj_set_style_bg_color(s_status_leds[i], Display_LvglStatusColor(value), 0);
    }
  }
}

/**
 * @brief Clamp a percentage-like value to LVGL bar range.
 */
static int32_t Display_LvglClampPercent(uint32_t value)
{
  return (value > 100U) ? 100 : (int32_t)value;
}

static uint16_t Display_LvglMotorPulseUs(uint32_t percent)
{
  uint32_t range;
  uint32_t pulse;

  if (percent > 100U) {
    percent = 100U;
  }
  range = (uint32_t)PX4LITE_CONTROL_ESC_MAX_PULSE_US - (uint32_t)PX4LITE_CONTROL_ESC_MIN_PULSE_US;
  pulse = (uint32_t)PX4LITE_CONTROL_ESC_MIN_PULSE_US + ((range * percent) / 100U);
  return (uint16_t)pulse;
}

static void Display_LvglUpdateMotorPulseLabel(uint8_t motor_index, uint32_t pulse_us)
{
  if (motor_index >= DISPLAY_LVGL_MOTOR_COUNT) {
    return;
  }

  (void)snprintf(s_motor_pulse_text[motor_index], sizeof(s_motor_pulse_text[motor_index]), "%uus", (unsigned int)pulse_us);
  if (s_motor_pulse_labels[motor_index] != 0) {
    lv_label_set_text_static(s_motor_pulse_labels[motor_index], s_motor_pulse_text[motor_index]);
  }
}

static void Display_LvglFormatClock(char *text, uint32_t hhmmss)
{
  uint32_t hh = (hhmmss / 10000U) % 100U;
  uint32_t mm = (hhmmss / 100U) % 100U;
  uint32_t ss = hhmmss % 100U;

  (void)snprintf(text, 9U, "%02lu:%02lu:%02lu", (unsigned long)hh, (unsigned long)mm, (unsigned long)ss);
}

static const char *Display_LvglLogMessageText(Display_LogMsg_t msg)
{
  switch (msg) {
    case DISPLAY_LOGMSG_SYSTEM_START:
      return "\xE7""\xB3""\xBB""\xE7""\xBB""\x9F""\xE5""\x90""\xAF""\xE5""\x8A""\xA8";
    case DISPLAY_LOGMSG_SELFCHECK_OK:
      return "\xE8""\x87""\xAA""\xE6""\xA3""\x80""\xE9""\x80""\x9A""\xE8""\xBF""\x87";
    case DISPLAY_LOGMSG_SELFCHECK_PART:
      return "\xE8""\x87""\xAA""\xE6""\xA3""\x80""\xE9""\x83""\xA8""\xE5""\x88""\x86""\xE9""\x80""\x9A""\xE8""\xBF""\x87";
    case DISPLAY_LOGMSG_SELFCHECK_FAIL:
      return "\xE8""\x87""\xAA""\xE6""\xA3""\x80""\xE6""\x9C""\xAA""\xE9""\x80""\x9A""\xE8""\xBF""\x87";
    case DISPLAY_LOGMSG_GPS_OK:
      return "\xE5""\xAE""\x9A""\xE4""\xBD""\x8D""\xE6""\xAD""\xA3""\xE5""\xB8""\xB8";
    case DISPLAY_LOGMSG_GPS_NOSIG:
      return "\xE5""\xAE""\x9A""\xE4""\xBD""\x8D""\xE6""\x97""\xA0""\xE4""\xBF""\xA1""\xE5""\x8F""\xB7";
    case DISPLAY_LOGMSG_GPS_LOST:
      return "\xE5""\xAE""\x9A""\xE4""\xBD""\x8D""\xE6""\x96""\xAD""\xE5""\xBC""\x80";
    case DISPLAY_LOGMSG_ATT_OK:
      return "\xE5""\xA7""\xBF""\xE6""\x80""\x81""\xE6""\xAD""\xA3""\xE5""\xB8""\xB8";
    case DISPLAY_LOGMSG_ATT_LOST:
      return "\xE5""\xA7""\xBF""\xE6""\x80""\x81""\xE6""\x96""\xAD""\xE5""\xBC""\x80";
    case DISPLAY_LOGMSG_ENV_OK:
      return "\xE7""\x8E""\xAF""\xE5""\xA2""\x83""\xE6""\xAD""\xA3""\xE5""\xB8""\xB8";
    case DISPLAY_LOGMSG_ENV_LOST:
      return "\xE7""\x8E""\xAF""\xE5""\xA2""\x83""\xE6""\x96""\xAD""\xE5""\xBC""\x80";
    case DISPLAY_LOGMSG_COMM_OK:
      return "LoRa""\xE6""\xAD""\xA3""\xE5""\xB8""\xB8";
    case DISPLAY_LOGMSG_COMM_LOST:
      return "LoRa""\xE6""\x96""\xAD""\xE5""\xBC""\x80";
    case DISPLAY_LOGMSG_STORAGE_OK:
      return "\xE5""\xAD""\x98""\xE5""\x82""\xA8""\xE6""\xAD""\xA3""\xE5""\xB8""\xB8";
    case DISPLAY_LOGMSG_STORAGE_LOST:
      return "\xE5""\xAD""\x98""\xE5""\x82""\xA8""\xE6""\x96""\xAD""\xE5""\xBC""\x80";
    case DISPLAY_LOGMSG_MOTOR_OK:
      return "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE6""\xAD""\xA3""\xE5""\xB8""\xB8";
    case DISPLAY_LOGMSG_MOTOR_DISCONNECT:
      return "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE6""\x96""\xAD""\xE5""\xBC""\x80";
    case DISPLAY_LOGMSG_MOTOR_LOWPOWER:
      return "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE4""\xBE""\x9B""\xE7""\x94""\xB5""\xE4""\xB8""\x8D""\xE8""\xB6""\xB3";
    case DISPLAY_LOGMSG_MOTOR1_FAIL:
      return "\xE4""\xB8""\x80""\xE5""\x8F""\xB7""\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE6""\x95""\x85""\xE9""\x9A""\x9C";
    case DISPLAY_LOGMSG_MOTOR2_FAIL:
      return "\xE4""\xBA""\x8C""\xE5""\x8F""\xB7""\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE6""\x95""\x85""\xE9""\x9A""\x9C";
    case DISPLAY_LOGMSG_MOTOR3_FAIL:
      return "\xE4""\xB8""\x89""\xE5""\x8F""\xB7""\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE6""\x95""\x85""\xE9""\x9A""\x9C";
    case DISPLAY_LOGMSG_MOTOR4_FAIL:
      return "\xE5""\x9B""\x9B""\xE5""\x8F""\xB7""\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE6""\x95""\x85""\xE9""\x9A""\x9C";
    case DISPLAY_LOGMSG_MOTOR_ALL_FAIL:
      return "\xE5""\x85""\xA8""\xE9""\x83""\xA8""\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE6""\x95""\x85""\xE9""\x9A""\x9C";
    case DISPLAY_LOGMSG_MOTOR_DEAD:
      return "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE7""\x94""\xB5""\xE6""\xB1""\xA0""\xE6""\xB2""\xA1""\xE7""\x94""\xB5";
    case DISPLAY_LOGMSG_MOTOR_CHARGE:
      return "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE7""\x94""\xB5""\xE6""\xB1""\xA0""\xE9""\x9C""\x80""\xE5""\x85""\x85""\xE7""\x94""\xB5";
    case DISPLAY_LOGMSG_MAIN_CHARGE:
      return "\xE4""\xB8""\xBB""\xE6""\x8E""\xA7""\xE7""\x94""\xB5""\xE6""\xB1""\xA0""\xE9""\x9C""\x80""\xE5""\x85""\x85""\xE7""\x94""\xB5";
    case DISPLAY_LOGMSG_ALARM_ACTIVE:
      return "\xE6""\x9C""\x89""\xE5""\x91""\x8A""\xE8""\xAD""\xA6";
    case DISPLAY_LOGMSG_ALARM_NONE:
      return "\xE6""\x97""\xA0""\xE5""\x91""\x8A""\xE8""\xAD""\xA6";
    case DISPLAY_LOGMSG_REMOTEID_OK:
      return "RemoteID""\xE6""\xAD""\xA3""\xE5""\xB8""\xB8";
    case DISPLAY_LOGMSG_REMOTEID_LOST:
      return "RemoteID""\xE6""\x96""\xAD""\xE5""\xBC""\x80";
    case DISPLAY_LOGMSG_5G_OK:
      return "5G""\xE6""\xAD""\xA3""\xE5""\xB8""\xB8";
    case DISPLAY_LOGMSG_5G_LOST:
      return "5G""\xE6""\x96""\xAD""\xE5""\xBC""\x80";
    default:
      return "--";
  }
}

static void Display_LvglUpdateMessageLog(void)
{
  Display_MessageLogEntry_t entries[DISPLAY_LVGL_LOG_ROWS];
  Display_MessageLogEntry_t alarm_entry;
  uint8_t alarm_valid = 0U;
  uint16_t count;
  uint16_t visible_rows;
  uint16_t i;

  if ((s_log_alarm_label == 0) && (s_log_rows[0].msg_label == 0)) {
    return;
  }

  visible_rows = (s_log_visible_rows == 0U) ? DISPLAY_LVGL_LOG_ROWS : s_log_visible_rows;
  if (visible_rows > DISPLAY_LVGL_LOG_ROWS) {
    visible_rows = DISPLAY_LVGL_LOG_ROWS;
  }
  count = Display_PagesCopyLogMessages(entries, visible_rows, &alarm_entry, &alarm_valid);

  if (s_log_alarm_label != 0) {
    if (alarm_valid != 0U) {
      lv_label_set_text_static(s_log_alarm_label, Display_LvglLogMessageText(alarm_entry.msg));
      lv_obj_set_style_text_color(s_log_alarm_label, (alarm_entry.msg == DISPLAY_LOGMSG_ALARM_ACTIVE) ? lv_color_hex(0xF76D7E) : lv_color_hex(0xA8B7C7), 0);
    } else {
      lv_label_set_text_static(s_log_alarm_label, "--");
      lv_obj_set_style_text_color(s_log_alarm_label, lv_color_hex(0x7D91A6), 0);
    }
  }

  for (i = 0U; i < visible_rows; i++) {
    if (s_log_rows[i].time_label == 0) {
      continue;
    }
    if (i < count) {
      Display_LvglFormatClock(s_log_rows[i].time_text, entries[i].time_hhmmss);
      lv_label_set_text_static(s_log_rows[i].time_label, s_log_rows[i].time_text);
      lv_label_set_text_static(s_log_rows[i].msg_label, Display_LvglLogMessageText(entries[i].msg));
    } else {
      s_log_rows[i].time_text[0] = '\0';
      lv_label_set_text_static(s_log_rows[i].time_label, s_log_rows[i].time_text);
      lv_label_set_text_static(s_log_rows[i].msg_label, "");
    }
  }
}

static void Display_LvglFormatFaultCode(char *text, uint16_t code)
{
  (void)snprintf(text, 8U, "0x%04X", code);
}

static const char *Display_LvglAlarmModuleText(uint16_t source_id, uint16_t fault_code)
{
  if ((fault_code == (uint16_t)PX4LITE_FAULT_MOTOR_DISCONNECT) || (fault_code == (uint16_t)PX4LITE_FAULT_MOTOR_POWER_LOW) ||
      (fault_code == (uint16_t)PX4LITE_FAULT_MOTOR_DEAD) || (fault_code == (uint16_t)PX4LITE_FAULT_MOTOR_CHARGE)) {
    return "\xE7""\x94""\xB5""\xE6""\x9C""\xBA";
  }

  switch ((Px4Lite_ModuleId_t)source_id) {
    case PX4LITE_MODULE_GNSS:
      return "\xE5""\xAE""\x9A""\xE4""\xBD""\x8D";
    case PX4LITE_MODULE_IMU:
      return "\xE5""\xA7""\xBF""\xE6""\x80""\x81";
    case PX4LITE_MODULE_BARO:
      return "\xE7""\x8E""\xAF""\xE5""\xA2""\x83";
    case PX4LITE_MODULE_BATTERY:
      return "\xE7""\x94""\xB5""\xE6""\xBA""\x90";
    case PX4LITE_MODULE_LORA:
      return "\xE9""\x80""\x9A""\xE4""\xBF""\xA1";
    case PX4LITE_MODULE_5G:
      return "\xE9""\x80""\x9A""\xE4""\xBF""\xA1";
    case PX4LITE_MODULE_STORAGE:
      return "\xE5""\xAD""\x98""\xE5""\x82""\xA8";
    case PX4LITE_MODULE_REMOTE_ID:
      return "\xE9""\x80""\x9A""\xE4""\xBF""\xA1";
    case PX4LITE_MODULE_DISPLAY:
      return "\xE6""\x98""\xBE""\xE7""\xA4""\xBA";
    case PX4LITE_MODULE_CONTROL:
      return "\xE6""\x8E""\xA7""\xE5""\x88""\xB6";
    case PX4LITE_MODULE_ALARM:
      return "\xE5""\x91""\x8A""\xE8""\xAD""\xA6";
    case PX4LITE_MODULE_SYSTEM:
      return "\xE7""\xB3""\xBB""\xE7""\xBB""\x9F";
    case PX4LITE_MODULE_ESTIMATOR:
      return "\xE4""\xBC""\xB0""\xE8""\xAE""\xA1";
    case PX4LITE_MODULE_BUSINESS:
      return "\xE4""\xB8""\x9A""\xE5""\x8A""\xA1";
    default:
      return "\xE6""\x9C""\xAA""\xE7""\x9F""\xA5";
  }
}

static const char *Display_LvglAlarmReasonText(uint16_t code)
{
  switch ((Px4Lite_FaultCode_t)code) {
    case PX4LITE_FAULT_SYSTEM_SELF_CHECK:
      return "\xE8""\x87""\xAA""\xE6""\xA3""\x80""\xE5""\xA4""\xB1""\xE8""\xB4""\xA5";
    case PX4LITE_FAULT_SYSTEM_HEAP:
      return "\xE5""\xA0""\x86""\xE5""\xBC""\x82""\xE5""\xB8""\xB8";
    case PX4LITE_FAULT_SYSTEM_STACK:
      return "\xE6""\xA0""\x88""\xE5""\xBC""\x82""\xE5""\xB8""\xB8";
    case PX4LITE_FAULT_SYSTEM_TASK_LOST:
      return "\xE4""\xBB""\xBB""\xE5""\x8A""\xA1""\xE4""\xB8""\xA2""\xE5""\xA4""\xB1";
    case PX4LITE_FAULT_SENSOR_INIT:
      /* "传感器未初始化" → "传感器未就绪"。 */
      return "\xE4""\xBC""\xA0""\xE6""\x84""\x9F""\xE5""\x99""\xA8""\xE6""\x9C""\xAA""\xE5""\xB0""\xB1""\xE7""\xBB""\xAA";
    case PX4LITE_FAULT_SENSOR_OFFLINE:
      return "\xE4""\xBC""\xA0""\xE6""\x84""\x9F""\xE5""\x99""\xA8""\xE7""\xA6""\xBB""\xE7""\xBA""\xBF";
    case PX4LITE_FAULT_SENSOR_INVALID:
    case PX4LITE_FAULT_APP_INVALID_DATA:
      return "\xE6""\x95""\xB0""\xE6""\x8D""\xAE""\xE5""\xBC""\x82""\xE5""\xB8""\xB8";
    case PX4LITE_FAULT_SENSOR_NO_FIX:
      return "\xE6""\x97""\xA0""\xE5""\xAE""\x9A""\xE4""\xBD""\x8D";
    case PX4LITE_FAULT_SENSOR_TIMEOUT:
      return "\xE4""\xBC""\xA0""\xE6""\x84""\x9F""\xE5""\x99""\xA8""\xE8""\xB6""\x85""\xE6""\x97""\xB6";
    case PX4LITE_FAULT_COMM_OFFLINE:
      return "\xE9""\x80""\x9A""\xE4""\xBF""\xA1""\xE7""\xA6""\xBB""\xE7""\xBA""\xBF";
    case PX4LITE_FAULT_COMM_TIMEOUT:
      return "\xE9""\x80""\x9A""\xE4""\xBF""\xA1""\xE8""\xB6""\x85""\xE6""\x97""\xB6";
    case PX4LITE_FAULT_COMM_FRAME:
      return "\xE9""\x80""\x9A""\xE4""\xBF""\xA1""\xE5""\xB8""\xA7""\xE9""\x94""\x99""\xE8""\xAF""\xAF";
    case PX4LITE_FAULT_DISPLAY_OFFLINE:
      return "\xE6""\x98""\xBE""\xE7""\xA4""\xBA""\xE7""\xA6""\xBB""\xE7""\xBA""\xBF";
    case PX4LITE_FAULT_DISPLAY_REFRESH:
      return "\xE6""\x98""\xBE""\xE7""\xA4""\xBA""\xE5""\x88""\xB7""\xE6""\x96""\xB0";
    case PX4LITE_FAULT_STORAGE_NOT_READY:
      return "\xE5""\xAD""\x98""\xE5""\x82""\xA8""\xE6""\x9C""\xAA""\xE5""\xB0""\xB1""\xE7""\xBB""\xAA";
    case PX4LITE_FAULT_STORAGE_WRITE:
      return "\xE5""\x86""\x99""\xE5""\x85""\xA5""\xE5""\xA4""\xB1""\xE8""\xB4""\xA5";
    case PX4LITE_FAULT_STORAGE_FULL:
      return "\xE5""\xAD""\x98""\xE5""\x82""\xA8""\xE5""\xB7""\xB2""\xE6""\xBB""\xA1";
    case PX4LITE_FAULT_PROTOCOL_PARSE:
      return "\xE8""\xA7""\xA3""\xE6""\x9E""\x90""\xE9""\x94""\x99""\xE8""\xAF""\xAF";
    case PX4LITE_FAULT_PROTOCOL_CRC:
      return "\xE6""\xA0""\xA1""\xE9""\xAA""\x8C""\xE9""\x94""\x99""\xE8""\xAF""\xAF";
    case PX4LITE_FAULT_ESTIMATOR_INPUT:
      return "\xE4""\xBC""\xB0""\xE8""\xAE""\xA1""\xE8""\xBE""\x93""\xE5""\x85""\xA5""\xE5""\xBC""\x82""\xE5""\xB8""\xB8";
    case PX4LITE_FAULT_ESTIMATOR_DIVERGE:
      return "\xE4""\xBC""\xB0""\xE8""\xAE""\xA1""\xE5""\x8F""\x91""\xE6""\x95""\xA3";
    case PX4LITE_FAULT_MOTOR_DISCONNECT:
      return "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE6""\x96""\xAD""\xE5""\xBC""\x80";
    case PX4LITE_FAULT_MOTOR_POWER_LOW:
      return "\xE4""\xBE""\x9B""\xE7""\x94""\xB5""\xE4""\xB8""\x8D""\xE8""\xB6""\xB3";
    case PX4LITE_FAULT_MOTOR_DEAD:
      return "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE7""\x94""\xB5""\xE6""\xB1""\xA0""\xE6""\xB2""\xA1""\xE7""\x94""\xB5";
    case PX4LITE_FAULT_MOTOR_CHARGE:
      return "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE7""\x94""\xB5""\xE6""\xB1""\xA0""\xE9""\x9C""\x80""\xE5""\x85""\x85""\xE7""\x94""\xB5";
    case PX4LITE_FAULT_POWER_CHARGE:
      return "\xE4""\xB8""\xBB""\xE6""\x8E""\xA7""\xE7""\x94""\xB5""\xE6""\xB1""\xA0""\xE9""\x9C""\x80""\xE5""\x85""\x85""\xE7""\x94""\xB5";
    default:
      return "\xE6""\x9C""\xAA""\xE7""\x9F""\xA5""\xE5""\x91""\x8A""\xE8""\xAD""\xA6";
  }
}

/*
 * 刷新一行告警。value=(source_id<<16)|fault_code；fault_code==0 表示空行。
 * 空行(除"无记录"占位)隐藏并停靠到 y=0，使滚动范围收缩到实际告警条数。
 */
static void Display_LvglUpdateAlarmRow(uint8_t row, uint32_t value)
{
  uint16_t source_id;
  uint16_t fault_code;
  lv_coord_t home_y;

  if (row >= DISPLAY_LVGL_ALARM_ROWS) {
    return;
  }
  if (s_alarm_rows[row].code_label == 0) {
    return;
  }

  source_id  = (uint16_t)(value >> 16);
  fault_code = (uint16_t)value;
  home_y     = (lv_coord_t)(row * DISPLAY_LVGL_ALARM_ROW_H);

  if (fault_code == 0U) {
    /* 无任何告警时用第 0 行显示"无记录"占位；其余空行隐藏并停靠 y=0 收缩滚动范围。 */
    if ((row == 0U) && (s_alarm_count == 0U)) {
      if (s_alarm_rows[row].row != 0) {
        lv_obj_set_y(s_alarm_rows[row].row, home_y);
        lv_obj_clear_flag(s_alarm_rows[row].row, LV_OBJ_FLAG_HIDDEN);
      }
      (void)snprintf(s_alarm_rows[row].code_text, sizeof(s_alarm_rows[row].code_text), "--");
      lv_label_set_text_static(s_alarm_rows[row].code_label, s_alarm_rows[row].code_text);
      lv_label_set_text_static(s_alarm_rows[row].module_label, "");
      lv_label_set_text_static(s_alarm_rows[row].reason_label, "\xE6""\x97""\xA0""\xE8""\xAE""\xB0""\xE5""\xBD""\x95");
      if (s_alarm_rows[row].bar != 0) {
        lv_obj_set_style_bg_opa(s_alarm_rows[row].bar, LV_OPA_TRANSP, 0);
      }
    } else if (s_alarm_rows[row].row != 0) {
      lv_obj_set_y(s_alarm_rows[row].row, 0);
      lv_obj_add_flag(s_alarm_rows[row].row, LV_OBJ_FLAG_HIDDEN);
    }
    return;
  }

  if (s_alarm_rows[row].row != 0) {
    lv_obj_set_y(s_alarm_rows[row].row, home_y);
    lv_obj_clear_flag(s_alarm_rows[row].row, LV_OBJ_FLAG_HIDDEN);
  }
  Display_LvglFormatFaultCode(s_alarm_rows[row].code_text, fault_code);
  lv_label_set_text_static(s_alarm_rows[row].code_label, s_alarm_rows[row].code_text);
  lv_label_set_text_static(s_alarm_rows[row].module_label, Display_LvglAlarmModuleText(source_id, fault_code));
  /* 通信离线原因按模块区分：LoRa 用"通信离线"，RemoteID 用"RemoteID离线"，两者分开显示。 */
  if ((source_id == (uint16_t)PX4LITE_MODULE_LORA) && (fault_code == (uint16_t)PX4LITE_FAULT_COMM_OFFLINE)) {
    lv_label_set_text_static(s_alarm_rows[row].reason_label, "LoRa""\xE7""\xA6""\xBB""\xE7""\xBA""\xBF");
  } else if ((source_id == (uint16_t)PX4LITE_MODULE_5G) && (fault_code == (uint16_t)PX4LITE_FAULT_COMM_OFFLINE)) {
    lv_label_set_text_static(s_alarm_rows[row].reason_label, "5G""\xE9""\x80""\x9A""\xE4""\xBF""\xA1""\xE7""\xA6""\xBB""\xE7""\xBA""\xBF");
  } else if ((source_id == (uint16_t)PX4LITE_MODULE_REMOTE_ID) && (fault_code == (uint16_t)PX4LITE_FAULT_COMM_OFFLINE)) {
    lv_label_set_text_static(s_alarm_rows[row].reason_label, "RemoteID""\xE7""\xA6""\xBB""\xE7""\xBA""\xBF");
  } else {
    lv_label_set_text_static(s_alarm_rows[row].reason_label, Display_LvglAlarmReasonText(fault_code));
  }
  if (s_alarm_rows[row].bar != 0) {
    lv_obj_set_style_bg_opa(s_alarm_rows[row].bar, LV_OPA_COVER, 0);
  }
}

static void Display_LvglUpdateAlarmTable(void)
{
  uint8_t row;

  for (row = 0U; row < DISPLAY_LVGL_ALARM_ROWS; row++) {
    Display_LvglUpdateAlarmRow(row, (row < s_alarm_count) ? s_alarm_packed[row] : 0U);
  }
}

void Display_LvglSetAlarmRows(const uint32_t *packed, uint16_t count)
{
  uint16_t i;
  uint8_t changed = 0U;

  if (count > (uint16_t)DISPLAY_LVGL_ALARM_ROWS) { count = (uint16_t)DISPLAY_LVGL_ALARM_ROWS; }

  /* 仅在告警集合真正变化时才重排行：否则每个显示周期无条件重设 16 行的
     位置/隐藏/文字会触发 LVGL 重排，与用户滚动手势打架，表现为滑动时刷新不稳定。 */
  if (count != s_alarm_count) { changed = 1U; }
  for (i = 0U; i < (uint16_t)DISPLAY_LVGL_ALARM_ROWS; i++) {
    uint32_t v = ((packed != 0) && (i < count)) ? packed[i] : 0U;
    if (v != s_alarm_packed[i]) {
      s_alarm_packed[i] = v;
      changed = 1U;
    }
  }
  s_alarm_count = count;

  if (changed != 0U) {
    Display_LvglUpdateAlarmTable();
  }
}

static void Display_LvglMotorSliderEventCb(lv_event_t *event)
{
  lv_event_code_t code;
  lv_obj_t *slider;
  Display_HmiVariableId_t id;
  uint8_t motor_index;
  int32_t value;

  code   = lv_event_get_code(event);
  slider = lv_event_get_target(event);
  id     = (Display_HmiVariableId_t)(uintptr_t)lv_event_get_user_data(event);
  if ((id < DISPLAY_HMI_VAR_MOTOR_PWM_1) || (id > DISPLAY_HMI_VAR_MOTOR_PWM_4)) { return; }
  motor_index = (uint8_t)((uint16_t)id - (uint16_t)DISPLAY_HMI_VAR_MOTOR_PWM_1);

  if (code == LV_EVENT_PRESSED) {
    s_motor_slider_dragging[motor_index] = 1U;
    return;
  }
  if ((code == LV_EVENT_RELEASED) || (code == LV_EVENT_PRESS_LOST)) {
    s_motor_slider_dragging[motor_index] = 0U;
    return;
  }
  if ((code != LV_EVENT_VALUE_CHANGED) || (s_control_update_active != 0U)) { return; }

  value  = lv_slider_get_value(slider);
  if (value < 0) {
    value = 0;
  }
  if (value > 100) {
    value = 100;
  }

  if (Display_RequestMotorThrottle(id, (uint16_t)value) != DISPLAY_OK) {
    if ((id < DISPLAY_HMI_VAR_COUNT) && (s_value_valid[id] != 0U)) {
      s_control_update_active = 1U;
      lv_slider_set_value(slider, Display_LvglClampPercent(s_values[id]), LV_ANIM_OFF);
      s_control_update_active = 0U;
    }
    return;
  }

  if ((id < DISPLAY_HMI_VAR_COUNT) && (s_value_valid[id] != 0U) && (s_values[id] != (uint32_t)value)) {
    s_control_update_active = 1U;
    lv_slider_set_value(slider, Display_LvglClampPercent(s_values[id]), LV_ANIM_OFF);
    s_control_update_active = 0U;
  }
}

static void Display_LvglEstopEventCb(lv_event_t *event)
{
  if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
    return;
  }

  (void)Display_RequestMotorEmergencyStop();
}

static void Display_LvglAttitudeCalEventCb(lv_event_t *event)
{
  if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
    return;
  }

  (void)Display_RequestAttitudeLevelCalibration();
}

/**
 * @brief Apply one cached value to currently active LVGL objects.
 */
static void Display_LvglApplyValue(Display_HmiVariableId_t id, uint32_t value)
{
  uint8_t motor_index;
  uint8_t label_index;

  if (id >= DISPLAY_HMI_VAR_COUNT) {
    return;
  }

  Display_LvglUpdateStatusLed(id, value);

  if ((id >= DISPLAY_HMI_VAR_MOTOR_PWM_1) && (id <= DISPLAY_HMI_VAR_MOTOR_PWM_4)) {
    motor_index = (uint8_t)((uint16_t)id - (uint16_t)DISPLAY_HMI_VAR_MOTOR_PWM_1);
    /* 用户正在拖动该滑块时不回推控制层 duty：否则滞后的 duty(电机未解锁时甚至恒为 0)
       会每帧把滑点往回拽，表现为拖动卡顿或干脆"滑不动"。松手后 is_dragged 变 false，
       下一次 duty 变化会正常把滑块重新同步到实际油门。 */
    if ((motor_index < DISPLAY_LVGL_MOTOR_COUNT) && (s_motor_pwm_bars[motor_index] != 0) &&
        (s_motor_slider_dragging[motor_index] == 0U) &&
        (lv_slider_is_dragged(s_motor_pwm_bars[motor_index]) == false)) {
      s_control_update_active = 1U;
      lv_slider_set_value(s_motor_pwm_bars[motor_index], Display_LvglClampPercent(value), LV_ANIM_OFF);
      s_control_update_active = 0U;
    }
    Display_LvglUpdateMotorPulseLabel(motor_index, Display_LvglMotorPulseUs(value));
  }

  if (id == DISPLAY_HMI_VAR_MESSAGE_LOG) {
    Display_LvglUpdateMessageLog();
  }

  /* 告警行不再由 ALARM_ROW1..5 单值变量驱动，改由 Display_LvglSetAlarmRows 批量写入
     (支持最多 16 行滚动)；此处不再分发到 Display_LvglUpdateAlarmRow。 */

  if ((id == DISPLAY_HMI_VAR_ROLL) || (id == DISPLAY_HMI_VAR_PITCH) || (id == DISPLAY_HMI_VAR_YAW)) {
    if (id == DISPLAY_HMI_VAR_ROLL) {
      s_attitude_roll_deg10 = (int16_t)value;
    } else if (id == DISPLAY_HMI_VAR_PITCH) {
      s_attitude_pitch_deg10 = (int16_t)value;
    } else {
      s_attitude_yaw_deg10 = (int16_t)value;
    }
    if (s_attitude_obj != 0) {
      lv_obj_invalidate(s_attitude_obj);
    }
  }

  if (s_value_slots[id].label != 0) {
    Display_LvglFormatValue(id, value);
    lv_label_set_text_static(s_value_slots[id].label, s_value_slots[id].text);
    for (label_index = 0U; label_index < DISPLAY_LVGL_VALUE_LABEL_EXTRA_COUNT; label_index++) {
      if (s_value_extra_labels[id][label_index] != 0) {
        lv_label_set_text_static(s_value_extra_labels[id][label_index], s_value_slots[id].text);
      }
    }
  }
}

/**
 * @brief Re-apply all cached values after a page rebuild.
 */
static void Display_LvglApplyCachedValues(void)
{
  uint16_t i;

  for (i = 0U; i < DISPLAY_HMI_VAR_COUNT; i++) {
    if (s_value_valid[i] != 0U) {
      Display_LvglApplyValue((Display_HmiVariableId_t)i, s_values[i]);
    }
  }
}

/**
 * @brief LVGL tab click event. The actual page switch is deferred.
 */
static void Display_LvglTabEventCb(lv_event_t *event)
{
  if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
    return;
  }

  s_requested_page          = (Display_HmiPage_t)(uintptr_t)lv_event_get_user_data(event);
  s_page_change_requested  = 1U;
}

/**
 * @brief 通信连接页节点行点击：选中该远端节点并回到进入前的页面显示远端数据。
 */
static void Display_LvglRemoteNodeEventCb(lv_event_t *event)
{
  uint8_t node_id;

  if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
    return;
  }
  Display_LvglTouchActivity();

  node_id = (uint8_t)(uintptr_t)lv_event_get_user_data(event);
  if (App_SelectRemoteNode(node_id, s_last_tick_ms) != 0U) {
    (void)App_SetRemoteViewEnabled(1U, s_last_tick_ms);
    s_requested_page         = s_page_before_hidden;
    s_page_change_requested  = 1U;
    s_page_rebuild_requested = 0U;
  }
}

/**
 * @brief 本地/远端按钮：在常规页(本地)与隐藏的通信连接页(远端节点选择)之间切换。
 * @note  fj-lora 语义：远端页/远端模式下点按回本地；本地模式下点按进入节点选择页。
 */
static void Display_LvglLocalRemoteEventCb(lv_event_t *event)
{
  if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
    return;
  }
  Display_LvglTouchActivity();

  if (s_current_lvgl_page == DISPLAY_HMI_PAGE_HIDDEN) {
    Display_LvglRequestLocalView(s_page_before_hidden, 0U);
  } else if (App_GetRemoteDisplayMode() == PX4LITE_REMOTE_MODE_REMOTE) {
    Display_LvglRequestLocalView(s_current_lvgl_page, 1U);
  } else {
    (void)App_SetRemoteViewEnabled(1U, s_last_tick_ms);
    s_page_before_hidden     = s_current_lvgl_page;
    s_requested_page         = DISPLAY_HMI_PAGE_HIDDEN;
    s_page_change_requested  = 1U;
    s_page_rebuild_requested = 0U;
  }
}

/**
 * @brief Create the formal header.
 */
static void Display_LvglCreateHeader(lv_obj_t *parent, Display_HmiPage_t page)
{
  lv_obj_t *bar;

  bar = lv_obj_create(parent);
  lv_obj_set_size(bar, DISPLAY_LVGL_WIDTH, DISPLAY_LVGL_HEADER_H);
  lv_obj_set_pos(bar, 0, 0);
  lv_obj_set_style_radius(bar, 0, 0);
  lv_obj_set_style_bg_color(bar, lv_color_hex(0x0D1824), 0);
  lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(bar, 0, 0);
  lv_obj_set_style_pad_all(bar, 0, 0);
  lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

  /* Logo */
  {
    lv_obj_t *logo_img = lv_img_create(bar);
    lv_img_set_src(logo_img, &s_logo_img_dsc);
    lv_obj_set_pos(logo_img, 0, 0);
  }
  /* 公司名：第一行东创大为，第二行 CNS 飞控系统。
     CNS 为拉丁字母(点阵中文字库不含 ASCII，走 montserrat 回退)，与后面汉字基线不同，
     故拆成两个标签分别定位。★可微调：CNS 的 y(与汉字基线对齐) 和 汉字起始 x(与 CNS 右缘衔接)。 */
  (void)Display_LvglCreateLabel(bar, "\xE4""\xB8""\x9C""\xE5""\x88""\x9B""\xE5""\xA4""\xA7""\xE4""\xB8""\xBA", 68, 10, &display_lvgl_font_zh_16, lv_color_hex(0xFFFFFF));
  /* y=36：montserrat 基线在 y+15(18-3)，与汉字 16px 字模墨迹底(约 y+15~16)齐平；
     x=105：C+N+S advance 共 37px(12+13+12)，汉字紧随其后。 */
  (void)Display_LvglCreateLabel(bar, "CNS", 68, 36, &lv_font_montserrat_16, lv_color_hex(0xB0C8D8));
  (void)Display_LvglCreateLabel(bar, "\xE9""\xA3""\x9E""\xE6""\x8E""\xA7""\xE7""\xB3""\xBB""\xE7""\xBB""\x9F", 105, 36, &display_lvgl_font_zh_16, lv_color_hex(0xB0C8D8));
  /* 日期/时间/飞行时间竖排三行（表头 64px 内，行距 20px，标签 x=174 数值 x=208） */
  (void)Display_LvglCreateLabel(bar, "\xE6""\x97""\xA5""\xE6""\x9C""\x9F", 174, 4, &display_lvgl_font_zh_16, lv_color_hex(0x7D91A6));
  Display_LvglCreateValueLabel(bar, DISPLAY_HMI_VAR_DATE, 208, 3, 90, &lv_font_montserrat_14);
  (void)Display_LvglCreateLabel(bar, "\xE6""\x97""\xB6""\xE9""\x97""\xB4", 174, 24, &display_lvgl_font_zh_16, lv_color_hex(0x7D91A6));
  Display_LvglCreateValueLabel(bar, DISPLAY_HMI_VAR_CLOCK_TIME, 208, 23, 90, &lv_font_montserrat_14);
  (void)Display_LvglCreateLabel(bar, "\xE9""\xA3""\x9E""\xE8""\xA1""\x8C", 174, 44, &display_lvgl_font_zh_16, lv_color_hex(0x7D91A6));
  Display_LvglCreateValueLabel(bar, DISPLAY_HMI_VAR_FLIGHT_TIME_S, 208, 43, 90, &lv_font_montserrat_14);
  /* 中间标题 */
  (void)Display_LvglCreateCenteredLabel(bar, "\xE9""\xA3""\x9E""\xE6""\x8E""\xA7""\xE6""\x98""\xBE""\xE7""\xA4""\xBA""\xE7""\xB3""\xBB""\xE7""\xBB""\x9F", 298, 9, 268, &display_lvgl_font_zh_16, lv_color_hex(0xFFFFFF));
  (void)Display_LvglCreateCenteredLabel(bar, Display_LvglPageTitle(page), 298, 35, 268, &display_lvgl_font_zh_16, lv_color_hex(0x1DB7C9));
  /* 本机/当前查看对象身份 DCDW-xxx：本地模式=本机 sysid(UID派生，两台应不同)，远端模式=选中节点。
     右对齐(见 Display_LvglCreateValueLabel)，框右缘 566+100=666 与下方本地/远端按钮右缘(596+70)对齐。 */
  Display_LvglCreateValueLabel(bar, DISPLAY_HMI_VAR_VIEW_NODE_ID, 566, 1, 100, &lv_font_montserrat_14);
  /* 本地/远端按钮：放在"系统"左侧，点按切换本地常规页 / 远端通信连接页；
     标题随当前页显示"本地"或"远端"，按在远端页时高亮。 */
  {
    lv_obj_t *lr_btn;
    lv_obj_t *lr_label;
    uint8_t remote_selected = (uint8_t)(App_GetRemoteDisplayMode() == PX4LITE_REMOTE_MODE_REMOTE);

    lr_btn = lv_obj_create(bar);
    lv_obj_set_size(lr_btn, 70, 32);
    lv_obj_set_pos(lr_btn, 596, 16);
    lv_obj_set_style_radius(lr_btn, 4, 0);
    lv_obj_set_style_bg_color(lr_btn, remote_selected ? lv_color_hex(0x1DB7C9) : lv_color_hex(0x143747), 0);
    lv_obj_set_style_bg_opa(lr_btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(lr_btn, 1, 0);
    lv_obj_set_style_border_color(lr_btn, lv_color_hex(0x1DB7C9), 0);
    lv_obj_set_style_pad_all(lr_btn, 0, 0);
    lv_obj_add_flag(lr_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(lr_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(lr_btn, Display_LvglLocalRemoteEventCb, LV_EVENT_CLICKED, 0);
    lr_label = Display_LvglCreateLabel(lr_btn, remote_selected ? "\xE8""\xBF""\x9C""\xE7""\xAB""\xAF" : "\xE6""\x9C""\xAC""\xE5""\x9C""\xB0", 0, 0, &display_lvgl_font_zh_16, lv_color_hex(0xFFFFFF));
    lv_obj_center(lr_label);
  }

  /* 顶栏右上角：原"系统"状态模块替换为通信统计三行——发送计数 / 接收计数 / 丢包率。
     顶栏所有页面共用，故三项在每个页面都显示。丢包率直接复用
     DISPLAY_HMI_VAR_LORA_LOSS_RATE（由 link.loss_permille 提供，单位 0.1%），无需另算。 */
  (void)Display_LvglCreateLabel(bar, "\xE5""\x8F""\x91""\xE9""\x80""\x81", 672, 6, &display_lvgl_font_zh_16, lv_color_hex(0x7D91A6));
  Display_LvglCreateValueLabel(bar, DISPLAY_HMI_VAR_LORA_TX_COUNT, 710, 4, 82, &lv_font_montserrat_14);
  (void)Display_LvglCreateLabel(bar, "\xE6""\x8E""\xA5""\xE6""\x94""\xB6", 672, 26, &display_lvgl_font_zh_16, lv_color_hex(0x7D91A6));
  Display_LvglCreateValueLabel(bar, DISPLAY_HMI_VAR_LORA_RX_COUNT, 710, 24, 82, &lv_font_montserrat_14);
  (void)Display_LvglCreateLabel(bar, "\xE4""\xB8""\xA2""\xE5""\x8C""\x85", 672, 46, &display_lvgl_font_zh_16, lv_color_hex(0x7D91A6));
  Display_LvglCreateValueLabel(bar, DISPLAY_HMI_VAR_LORA_LOSS_RATE, 710, 44, 82, &lv_font_montserrat_14);
}

/**
 * @brief Create the bottom page tabs.
 */
static void Display_LvglCreateFooter(lv_obj_t *parent, Display_HmiPage_t page)
{
  lv_obj_t *footer;
  uint16_t i;
  lv_coord_t tab_w;

  footer = lv_obj_create(parent);
  lv_obj_set_size(footer, DISPLAY_LVGL_WIDTH, DISPLAY_LVGL_FOOTER_H);
  lv_obj_set_pos(footer, 0, DISPLAY_LVGL_FOOTER_Y);
  lv_obj_set_style_radius(footer, 0, 0);
  lv_obj_set_style_bg_color(footer, lv_color_hex(0x0D1824), 0);
  lv_obj_set_style_bg_opa(footer, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(footer, 0, 0);
  lv_obj_set_style_pad_all(footer, 0, 0);
  lv_obj_clear_flag(footer, LV_OBJ_FLAG_SCROLLABLE);

  tab_w = (lv_coord_t)(DISPLAY_LVGL_WIDTH / DISPLAY_LVGL_TAB_COUNT);
  for (i = 0U; i < DISPLAY_LVGL_TAB_COUNT; i++) {
    lv_obj_t *tab;
    lv_obj_t *label;
    uint8_t active;
    lv_coord_t x;
    lv_coord_t w;

    active = (uint8_t)(s_tabs[i].page == page);
    x      = (lv_coord_t)(i * tab_w);
    w      = (i == (DISPLAY_LVGL_TAB_COUNT - 1U)) ? (lv_coord_t)(DISPLAY_LVGL_WIDTH - x) : tab_w;

    tab = lv_obj_create(footer);
    lv_obj_set_size(tab, w, DISPLAY_LVGL_FOOTER_H);
    lv_obj_set_pos(tab, x, 0);
    lv_obj_set_style_radius(tab, 0, 0);
    lv_obj_set_style_bg_color(tab, active ? lv_color_hex(0x143747) : lv_color_hex(0x0D1824), 0);
    lv_obj_set_style_bg_opa(tab, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(tab, 0, 0);
    lv_obj_set_style_pad_all(tab, 0, 0);
    lv_obj_add_flag(tab, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(tab, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(tab, Display_LvglTabEventCb, LV_EVENT_CLICKED, (void *)(uintptr_t)s_tabs[i].page);

    if (active != 0U) {
      lv_obj_t *line = lv_obj_create(tab);
      lv_obj_set_size(line, w, 3);
      lv_obj_set_pos(line, 0, 0);
      lv_obj_set_style_radius(line, 0, 0);
      lv_obj_set_style_bg_color(line, lv_color_hex(0x1DB7C9), 0);
      lv_obj_set_style_bg_opa(line, LV_OPA_COVER, 0);
      lv_obj_set_style_border_width(line, 0, 0);
      lv_obj_clear_flag(line, LV_OBJ_FLAG_SCROLLABLE);
    }

    label = Display_LvglCreateLabel(tab, s_tabs[i].name, 0, 20, &display_lvgl_font_zh_16, active ? lv_color_hex(0xFFFFFF) : lv_color_hex(0x7D91A6));
    lv_obj_center(label);
  }
}

/**
 * @brief Create the common system status column.
 */
static void Display_LvglCreateSystemColumnAt(lv_obj_t *parent, lv_coord_t x, lv_coord_t w);

static void Display_LvglCreateSystemColumn(lv_obj_t *parent)
{
  Display_LvglCreateSystemColumnAt(parent, 8, 248);
}

static void Display_LvglCreateSystemColumnAt(lv_obj_t *parent, lv_coord_t x, lv_coord_t w)
{
  lv_obj_t *card;
  uint16_t i;

  card = Display_LvglCreateCard(parent, x, DISPLAY_LVGL_BODY_Y, w, DISPLAY_LVGL_BODY_H, "\xE7""\xB3""\xBB""\xE7""\xBB""\x9F""\xE7""\x8A""\xB6""\xE6""\x80""\x81");
  /* 起始 44 + 行距 28：含 RemoteID 共 10 项，末项底部约 44+9*28+16=312，容于卡片高 330。 */
  for (i = 0U; i < DISPLAY_LVGL_STATUS_COUNT; i++) {
    lv_coord_t y = (lv_coord_t)(42 + (i * 25U));
    s_status_leds[i] = Display_LvglCreateStatusDot(card, 18, y + 3);
    (void)Display_LvglCreateLabel(card, s_status_items[i].name, 42, y, &display_lvgl_font_zh_16, lv_color_hex(0xDCE8F2));
  }
}

static void Display_LvglCreateMessageLogPanelAt(lv_obj_t *parent, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h, uint8_t max_rows)
{
  lv_obj_t *card;
  lv_coord_t msg_x;
  lv_coord_t msg_w;
  lv_coord_t alarm_x;
  lv_coord_t row_y0;
  lv_coord_t row_step;
  uint16_t i;

  card = lv_obj_create(parent);
  lv_obj_set_size(card, w, h);
  lv_obj_set_pos(card, x, y);
  lv_obj_set_style_radius(card, DISPLAY_LVGL_CARD_RADIUS, 0);
  lv_obj_set_style_bg_color(card, lv_color_hex(0x13202E), 0);
  lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(card, 1, 0);
  lv_obj_set_style_border_color(card, lv_color_hex(0x304357), 0);
  lv_obj_set_style_pad_all(card, 0, 0);
  lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
  (void)Display_LvglCreateClipLabel(card, "\xE6""\xB6""\x88""\xE6""\x81""\xAF""\xE6""\x97""\xA5""\xE5""\xBF""\x97", 14, 10, (lv_coord_t)(w - 100), &display_lvgl_font_zh_16, lv_color_hex(0xDCE8F2));
  msg_x   = (w < 220) ? 66 : 84;
  msg_w   = (w > (msg_x + 14)) ? (lv_coord_t)(w - msg_x - 14) : 64;
  alarm_x = (w > 86) ? (lv_coord_t)(w - 82) : 88;
  row_y0  = (h < 120) ? 36 : ((h < 160) ? 42 : 48);
  row_step = (h < 120) ? 21 : ((h < 160) ? 24 : 29);

  if (max_rows > DISPLAY_LVGL_LOG_ROWS) {
    max_rows = DISPLAY_LVGL_LOG_ROWS;
  }
  s_log_visible_rows = max_rows;

  s_log_alarm_label = Display_LvglCreateClipLabel(card, "--", alarm_x, 10, 72, &display_lvgl_font_zh_16, lv_color_hex(0x7D91A6));
  lv_obj_set_style_text_align(s_log_alarm_label, LV_TEXT_ALIGN_RIGHT, 0);

  for (i = 0U; i < max_rows; i++) {
    lv_coord_t row_y = (lv_coord_t)(row_y0 + (i * row_step));
    s_log_rows[i].time_text[0] = '\0';
    s_log_rows[i].time_label = Display_LvglCreateClipLabel(card, s_log_rows[i].time_text, 12, row_y + 2, 62, &lv_font_montserrat_12, lv_color_hex(0x7D91A6));
    s_log_rows[i].msg_label  = Display_LvglCreateClipLabel(card, "", msg_x, row_y, msg_w, &display_lvgl_font_zh_16, lv_color_hex(0xDCE8F2));
  }

  Display_LvglUpdateMessageLog();
}

static void Display_LvglCreateMessageLogPanel(lv_obj_t *parent, lv_coord_t x, lv_coord_t w)
{
  Display_LvglCreateMessageLogPanelAt(parent, x, DISPLAY_LVGL_BODY_Y, w, DISPLAY_LVGL_BODY_H, DISPLAY_LVGL_LOG_ROWS);
}

/*
 * 让容器可竖向滚动并在右侧显示滚动条：内容(子对象)超过容器高度时自动出现滚动条，
 * 在容器(含其子对象)上任意处按住拖动即可上下滚动。
 */
static void Display_LvglMakeScrollable(lv_obj_t *obj)
{
  lv_obj_add_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scroll_dir(obj, LV_DIR_VER);
  lv_obj_set_scrollbar_mode(obj, LV_SCROLLBAR_MODE_AUTO);
  lv_obj_set_style_bg_color(obj, lv_color_hex(0x4A6076), LV_PART_SCROLLBAR);
  lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, LV_PART_SCROLLBAR);
  lv_obj_set_style_width(obj, 6, LV_PART_SCROLLBAR);
  lv_obj_set_style_radius(obj, 3, LV_PART_SCROLLBAR);
}

static void Display_LvglCreateAlarmTable(lv_obj_t *parent, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h)
{
  lv_obj_t *table;
  lv_obj_t *header;
  lv_obj_t *body;
  lv_coord_t code_col_w;
  lv_coord_t module_col_w;
  lv_coord_t reason_col_x;
  lv_coord_t row_y0;
  lv_coord_t row_h;
  uint16_t i;

  code_col_w  = 150;
  module_col_w = 170;
  reason_col_x = (lv_coord_t)(code_col_w + module_col_w);
  row_y0      = 40;
  row_h       = DISPLAY_LVGL_ALARM_ROW_H;

  table = lv_obj_create(parent);
  lv_obj_set_size(table, w, h);
  lv_obj_set_pos(table, x, y);
  lv_obj_set_style_radius(table, 0, 0);
  lv_obj_set_style_bg_color(table, lv_color_hex(0x13202E), 0);
  lv_obj_set_style_bg_opa(table, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(table, 1, 0);
  lv_obj_set_style_border_color(table, lv_color_hex(0x304357), 0);
  lv_obj_set_style_pad_all(table, 0, 0);
  lv_obj_clear_flag(table, LV_OBJ_FLAG_SCROLLABLE);

  header = lv_obj_create(table);
  lv_obj_set_size(header, w, row_y0);
  lv_obj_set_pos(header, 0, 0);
  lv_obj_set_style_radius(header, 0, 0);
  lv_obj_set_style_bg_color(header, lv_color_hex(0x1C3042), 0);
  lv_obj_set_style_bg_opa(header, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(header, 0, 0);
  lv_obj_set_style_pad_all(header, 0, 0);
  lv_obj_clear_flag(header, LV_OBJ_FLAG_SCROLLABLE);

  /* 表头列标题与下方各列内容左缘对齐：代码 x=30、模块 x=code_col_w+20、原因 x=reason_col_x+20。 */
  (void)Display_LvglCreateClipLabel(header, "\xE4""\xBB""\xA3""\xE7""\xA0""\x81", 30, 10, 60, &display_lvgl_font_zh_16, lv_color_hex(0xFFFFFF));
  (void)Display_LvglCreateClipLabel(header, "\xE6""\xA8""\xA1""\xE5""\x9D""\x97", (lv_coord_t)(code_col_w + 20), 10, 60, &display_lvgl_font_zh_16, lv_color_hex(0xFFFFFF));
  (void)Display_LvglCreateClipLabel(header, "\xE5""\x8E""\x9F""\xE5""\x9B""\xA0", (lv_coord_t)(reason_col_x + 20), 10, 60, &display_lvgl_font_zh_16, lv_color_hex(0xFFFFFF));

  /* 表头固定，行区放在可滚动 body 内(告警超过可视行数时右侧出滚动条)。 */
  body = lv_obj_create(table);
  lv_obj_set_size(body, w, (lv_coord_t)(h - row_y0));
  lv_obj_set_pos(body, 0, row_y0);
  lv_obj_set_style_radius(body, 0, 0);
  lv_obj_set_style_bg_opa(body, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(body, 0, 0);
  lv_obj_set_style_pad_all(body, 0, 0);
  Display_LvglMakeScrollable(body);

  for (i = 0U; i < DISPLAY_LVGL_ALARM_ROWS; i++) {
    lv_obj_t *row_bg = lv_obj_create(body);
    s_alarm_rows[i].row = row_bg;
    lv_obj_set_size(row_bg, w, row_h);
    lv_obj_set_pos(row_bg, 0, (lv_coord_t)(i * row_h));
    lv_obj_set_style_radius(row_bg, 0, 0);
    lv_obj_set_style_bg_color(row_bg, (i & 1U) ? lv_color_hex(0x172738) : lv_color_hex(0x101B27), 0);
    lv_obj_set_style_bg_opa(row_bg, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(row_bg, lv_color_hex(0x304357), 0);
    lv_obj_set_style_border_side(row_bg, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_width(row_bg, 1, 0);
    lv_obj_set_style_pad_all(row_bg, 0, 0);
    lv_obj_clear_flag(row_bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(row_bg, LV_OBJ_FLAG_HIDDEN);

    s_alarm_rows[i].bar = lv_obj_create(row_bg);
    lv_obj_set_size(s_alarm_rows[i].bar, 5, (lv_coord_t)(row_h - 1));
    lv_obj_set_pos(s_alarm_rows[i].bar, 1, 1);
    lv_obj_set_style_radius(s_alarm_rows[i].bar, 0, 0);
    lv_obj_set_style_bg_color(s_alarm_rows[i].bar, lv_color_hex(0xF76D7E), 0);
    lv_obj_set_style_bg_opa(s_alarm_rows[i].bar, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_alarm_rows[i].bar, 0, 0);
    lv_obj_clear_flag(s_alarm_rows[i].bar, LV_OBJ_FLAG_SCROLLABLE);

    s_alarm_rows[i].code_text[0] = '\0';
    s_alarm_rows[i].code_label   = Display_LvglCreateClipLabel(row_bg, s_alarm_rows[i].code_text, 30, 16, 110, &lv_font_montserrat_16, lv_color_hex(0xF76D7E));
    s_alarm_rows[i].module_label = Display_LvglCreateClipLabel(row_bg, "", (lv_coord_t)(code_col_w + 20), 15, 130, &display_lvgl_font_zh_16, lv_color_hex(0xDCE8F2));
    s_alarm_rows[i].reason_label = Display_LvglCreateClipLabel(row_bg, "", (lv_coord_t)(reason_col_x + 20), 15, (lv_coord_t)(w - reason_col_x - 30), &display_lvgl_font_zh_16, lv_color_hex(0xDCE8F2));
  }

  Display_LvglUpdateAlarmTable();
}

/*
 * 自检汇总告警列表：与告警页同一数据源（s_alarm_packed，由 Display_LvglSetAlarmRows 批量写入）。
 * 窄卡片内两行紧凑布局：第一行 代码 + 模块，第二行 原因。告警超过可视高度时 body 可竖向滚动，
 * 右侧出滚动条，框内任意处拖动即可上下滑动。
 */
static void Display_LvglCreateSummaryAlarmList(lv_obj_t *card)
{
  lv_obj_t *body;
  lv_coord_t row_h = DISPLAY_LVGL_ALARM_ROW_H;
  uint16_t i;

  body = lv_obj_create(card);
  lv_obj_set_size(body, 224, 280);
  lv_obj_set_pos(body, 2, 36);
  lv_obj_set_style_radius(body, 0, 0);
  lv_obj_set_style_bg_opa(body, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(body, 0, 0);
  lv_obj_set_style_pad_all(body, 0, 0);
  Display_LvglMakeScrollable(body);

  for (i = 0U; i < DISPLAY_LVGL_ALARM_ROWS; i++) {
    lv_obj_t *row_bg = lv_obj_create(body);
    s_alarm_rows[i].row = row_bg;
    lv_obj_set_size(row_bg, 220, row_h);
    lv_obj_set_pos(row_bg, 0, (lv_coord_t)(i * row_h));
    lv_obj_set_style_radius(row_bg, 0, 0);
    lv_obj_set_style_bg_opa(row_bg, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(row_bg, lv_color_hex(0x223547), 0);
    lv_obj_set_style_border_side(row_bg, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_width(row_bg, 1, 0);
    lv_obj_set_style_pad_all(row_bg, 0, 0);
    lv_obj_clear_flag(row_bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(row_bg, LV_OBJ_FLAG_HIDDEN);

    s_alarm_rows[i].bar = lv_obj_create(row_bg);
    lv_obj_set_size(s_alarm_rows[i].bar, 4, 40);
    lv_obj_set_pos(s_alarm_rows[i].bar, 4, 4);
    lv_obj_set_style_radius(s_alarm_rows[i].bar, 0, 0);
    lv_obj_set_style_bg_color(s_alarm_rows[i].bar, lv_color_hex(0xF76D7E), 0);
    lv_obj_set_style_bg_opa(s_alarm_rows[i].bar, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_alarm_rows[i].bar, 0, 0);
    lv_obj_clear_flag(s_alarm_rows[i].bar, LV_OBJ_FLAG_SCROLLABLE);

    s_alarm_rows[i].code_text[0] = '\0';
    s_alarm_rows[i].code_label   = Display_LvglCreateClipLabel(row_bg, s_alarm_rows[i].code_text, 14, 4, 78, &lv_font_montserrat_14, lv_color_hex(0xF76D7E));
    s_alarm_rows[i].module_label = Display_LvglCreateClipLabel(row_bg, "", 96, 4, 118, &DISPLAY_LVGL_FONT_ZH_SMALL, lv_color_hex(0xDCE8F2));
    s_alarm_rows[i].reason_label = Display_LvglCreateClipLabel(row_bg, "", 14, 26, 200, &DISPLAY_LVGL_FONT_ZH_SMALL, lv_color_hex(0xDCE8F2));
  }

  Display_LvglUpdateAlarmTable();
}

/**
 * @brief Create self-check page.
 */
static void Display_LvglCreateSelfCheckPage(lv_obj_t *parent)
{
  lv_obj_t *card;
  lv_obj_t *summary;
  uint16_t i;

  card = Display_LvglCreateCard(parent, 24, 82, 500, 320, "\xE4""\xB8""\x8A""\xE7""\x94""\xB5""\xE8""\x87""\xAA""\xE6""\xA3""\x80");
  /* 3 列网格：含 5G/RemoteID 共 11 项占 4 行；行距压到 66 让第 4 行容于卡片高 320。 */
  for (i = 0U; i < DISPLAY_LVGL_STATUS_COUNT; i++) {
    lv_coord_t col = (lv_coord_t)(i % 3U);
    lv_coord_t row = (lv_coord_t)(i / 3U);
    lv_coord_t x   = (lv_coord_t)(34 + (col * 154));
    lv_coord_t y   = (lv_coord_t)(52 + (row * 66));

    s_status_leds[i] = Display_LvglCreateStatusDot(card, x, y + 2);
    (void)Display_LvglCreateLabel(card, s_status_items[i].name, x + 22, y - 2, &display_lvgl_font_zh_16, lv_color_hex(0xDCE8F2));
    Display_LvglCreateValueLabel(card, s_status_items[i].id, x + 22, y + 22, 92, &lv_font_montserrat_14);
  }

  summary = Display_LvglCreateCard(parent, 548, 82, 228, 320, "\xE8""\x87""\xAA""\xE6""\xA3""\x80""\xE6""\xB1""\x87""\xE6""\x80""\xBB");
  Display_LvglCreateSummaryAlarmList(summary);
}

/**
 * @brief Create flight data page.
 */
static void Display_LvglCreateFlightPage(lv_obj_t *parent)
{
  lv_obj_t *card;

  Display_LvglCreateSystemColumn(parent);
  card = Display_LvglCreateCard(parent, 264, DISPLAY_LVGL_BODY_Y, 268, DISPLAY_LVGL_BODY_H, "\xE9""\xA3""\x9E""\xE8""\xA1""\x8C""\xE6""\x95""\xB0""\xE6""\x8D""\xAE");
  /* 飞行时间移至顶栏；每组电池按 电压/电流/电量 三行排列；行距 30px 容纳 9 行。 */
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_BATTERY_VOLTAGE, "\xE4""\xB8""\xBB""\xE6""\x8E""\xA7""\xE7""\x94""\xB5""\xE5""\x8E""\x8B", 22, 48, 154);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_BATTERY_CURRENT, "\xE4""\xB8""\xBB""\xE6""\x8E""\xA7""\xE7""\x94""\xB5""\xE6""\xB5""\x81", 22, 78, 154);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_BATTERY_PERCENT, "\xE4""\xB8""\xBB""\xE6""\x8E""\xA7""\xE7""\x94""\xB5""\xE9""\x87""\x8F", 22, 108, 154);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE, "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE7""\x94""\xB5""\xE5""\x8E""\x8B", 22, 138, 154);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_MOTOR_BAT_CURRENT, "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE7""\x94""\xB5""\xE6""\xB5""\x81", 22, 168, 154);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_MOTOR_BAT_PERCENT, "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE7""\x94""\xB5""\xE9""\x87""\x8F", 22, 198, 154);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_TEMPERATURE, "\xE6""\xB8""\xA9""\xE5""\xBA""\xA6", 22, 228, 154);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_HUMIDITY, "\xE6""\xB9""\xBF""\xE5""\xBA""\xA6", 22, 258, 154);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_PRESSURE, "\xE6""\xB0""\x94""\xE5""\x8E""\x8B", 22, 288, 154);
  Display_LvglCreateMessageLogPanel(parent, 540, 252);
}

/**
 * @brief 在姿态卡片预留容器内绘制地平仪（DRAW_POST 直绘，不占额外帧缓冲）。
 *
 * @details 内圈姿态盘：天/地随横滚旋转、随俯仰平移，叠加俯仰刻度梯与固定飞机符号；
 *          外圈指南针：随航向旋转（航向朝上），含 10°刻度、N/E/S/W 方位字母与
 *          顶部固定航向指针。横滚/俯仰/偏航取自 DISPLAY_HMI_VAR_ROLL / PITCH / YAW
 *          （int16，单位 0.1°）。方向相反时调整 HZ_ROLL_SIGN / HZ_PITCH_SIGN / HZ_YAW_SIGN。
 */
static void Display_LvglHorizonDrawCb(lv_event_t *e)
{
  lv_obj_t *obj = lv_event_get_target(e);
  lv_draw_ctx_t *dc = lv_event_get_draw_ctx(e);
  lv_area_t co;
  lv_area_t fill;
  lv_draw_rect_dsc_t rdsc;
  lv_draw_line_dsc_t ldsc;
  lv_draw_mask_radius_param_t mcirc;
  lv_draw_mask_line_param_t mline;
  lv_point_t pa;
  lv_point_t pb;
  lv_coord_t w;
  lv_coord_t h;
  lv_coord_t cx;
  lv_coord_t cy;
  lv_coord_t r_co;
  lv_coord_t r_ci;
  lv_coord_t r;
  float a;
  float ca;
  float sa;
  float pitch_deg;
  float pitch_px;
  float yaw_deg;
  float tx;
  float ty;
  float nx;
  float ny;
  float ox;
  float oy;
  int16_t id_c;
  int16_t id_l;
  uint16_t i;
  static const int8_t ladder[] = {-30, -25, -20, -15, -10, -5, 5, 10, 15, 20, 25, 30};

  if (dc == NULL) {
    return;
  }

  lv_obj_get_coords(obj, &co);
  w = lv_area_get_width(&co);
  h = lv_area_get_height(&co);
  cx = (lv_coord_t)(co.x1 + (w / 2));
  cy = (lv_coord_t)(co.y1 + (h / 2));
  r_co = (lv_coord_t)((LV_MIN(w, h) / 2) - 3);   /* 罗盘外半径 */
  r_ci = (lv_coord_t)(r_co - 22);                /* 罗盘内半径 = 姿态盘外缘 */
  r = (lv_coord_t)(r_ci - 3);                    /* 姿态盘（天/地）半径 */

  pitch_deg = ((float)s_attitude_pitch_deg10 / 10.0f) * HZ_PITCH_SIGN;
  yaw_deg = (((float)s_attitude_yaw_deg10 / 10.0f) * HZ_YAW_SIGN) + HZ_YAW_OFFSET_DEG;
  a = ((float)s_attitude_roll_deg10 / 10.0f) * HZ_ROLL_SIGN * (float)(M_PI / 180.0);
  ca = cosf(a);
  sa = sinf(a);
  tx = ca;            /* 地平线切向 */
  ty = sa;
  nx = sa;            /* 指向天空的法向（a=0 时为屏幕上方）*/
  ny = -ca;
  pitch_px = pitch_deg * HZ_PX_PER_DEG;
  ox = (float)cx - (nx * pitch_px);   /* 当前地平线中心，随俯仰平移 */
  oy = (float)cy - (ny * pitch_px);

  fill.x1 = (lv_coord_t)(cx - r);
  fill.y1 = (lv_coord_t)(cy - r);
  fill.x2 = (lv_coord_t)(cx + r);
  fill.y2 = (lv_coord_t)(cy + r);

  /* 圆形遮罩：动态内容全部裁进圆盘 */
  lv_draw_mask_radius_init(&mcirc, &fill, LV_RADIUS_CIRCLE, false);
  id_c = lv_draw_mask_add(&mcirc, NULL);

  /* 天空铺满圆盘 */
  lv_draw_rect_dsc_init(&rdsc);
  rdsc.bg_opa = LV_OPA_COVER;
  rdsc.bg_color = lv_color_hex(0x2E8BE6);
  lv_draw_rect(dc, &rdsc, &fill);

  /* 地面：沿地平线加直线遮罩（保留下半侧）后覆盖棕色 */
  {
    lv_coord_t p1x = (lv_coord_t)(ox - (tx * (float)(r + 4)));
    lv_coord_t p1y = (lv_coord_t)(oy - (ty * (float)(r + 4)));
    lv_coord_t p2x = (lv_coord_t)(ox + (tx * (float)(r + 4)));
    lv_coord_t p2y = (lv_coord_t)(oy + (ty * (float)(r + 4)));
    lv_draw_mask_line_points_init(&mline, p1x, p1y, p2x, p2y, LV_DRAW_MASK_LINE_SIDE_BOTTOM);
    id_l = lv_draw_mask_add(&mline, NULL);
    rdsc.bg_color = lv_color_hex(0x8A5A2B);
    lv_draw_rect(dc, &rdsc, &fill);
    lv_draw_mask_remove_id(id_l);
    lv_draw_mask_free_param(&mline);
  }

  /* 地平线（俯仰刻度梯 v=0），整条略粗 */
  lv_draw_line_dsc_init(&ldsc);
  ldsc.opa = LV_OPA_COVER;
  ldsc.color = lv_color_hex(0xFFFFFF);
  ldsc.round_start = 1;
  ldsc.round_end = 1;
  ldsc.width = 3;
  pa.x = (lv_coord_t)(ox - (tx * (float)r));
  pa.y = (lv_coord_t)(oy - (ty * (float)r));
  pb.x = (lv_coord_t)(ox + (tx * (float)r));
  pb.y = (lv_coord_t)(oy + (ty * (float)r));
  lv_draw_line(dc, &ldsc, &pa, &pb);

  /* 其余俯仰刻度线（两段，中间留出飞机符号缺口）*/
  for (i = 0U; i < (uint16_t)(sizeof(ladder) / sizeof(ladder[0])); i++) {
    float k = ((float)ladder[i] - pitch_deg) * HZ_PX_PER_DEG;
    float mx = (float)cx + (nx * k);
    float my = (float)cy + (ny * k);
    float half = ((ladder[i] % 10) == 0) ? 18.0f : 11.0f;
    float gap = 12.0f;

    ldsc.width = ((ladder[i] % 10) == 0) ? 2 : 1;
    pa.x = (lv_coord_t)(mx - (tx * half));
    pa.y = (lv_coord_t)(my - (ty * half));
    pb.x = (lv_coord_t)(mx - (tx * gap));
    pb.y = (lv_coord_t)(my - (ty * gap));
    lv_draw_line(dc, &ldsc, &pa, &pb);
    pa.x = (lv_coord_t)(mx + (tx * gap));
    pa.y = (lv_coord_t)(my + (ty * gap));
    pb.x = (lv_coord_t)(mx + (tx * half));
    pb.y = (lv_coord_t)(my + (ty * half));
    lv_draw_line(dc, &ldsc, &pa, &pb);
  }

  lv_draw_mask_remove_id(id_c);
  lv_draw_mask_free_param(&mcirc);

  /* ===== 固定层（不裁剪）===== */
  /* 姿态盘外缘细圈 */
  {
    lv_draw_arc_dsc_t adsc;
    lv_point_t ctr;
    ctr.x = cx;
    ctr.y = cy;
    lv_draw_arc_dsc_init(&adsc);
    adsc.opa = LV_OPA_COVER;
    adsc.color = lv_color_hex(0x33485C);
    adsc.width = 2;
    lv_draw_arc(dc, &adsc, &ctr, r_ci, 0, 360);
  }

  /* 外圈指南针：底环 + 刻度 + 方位字母，随航向旋转（航向朝上）。
     字母直立绘制（LVGL 无法旋转字形），随罗盘平移，常见于嵌入式罗盘。 */
  {
    lv_draw_arc_dsc_t bandsc;
    lv_draw_line_dsc_t tdsc;
    lv_draw_label_dsc_t txtdsc;
    lv_point_t ctr;
    static const int16_t card_deg[4] = {0, 90, 180, 270};
    static const char *const card_txt[4] = {"N", "E", "S", "W"};
    float yaw_r = yaw_deg * (float)(M_PI / 180.0);
    float r_mid = ((float)r_co + (float)r_ci) / 2.0f;
    int16_t d;
    uint16_t c;

    ctr.x = cx;
    ctr.y = cy;
    lv_draw_arc_dsc_init(&bandsc);
    bandsc.opa = LV_OPA_COVER;
    bandsc.color = lv_color_hex(0x101D29);
    bandsc.width = (lv_coord_t)(r_co - r_ci);
    lv_draw_arc(dc, &bandsc, &ctr, (lv_coord_t)r_mid, 0, 360);

    lv_draw_line_dsc_init(&tdsc);
    tdsc.opa = LV_OPA_COVER;
    tdsc.color = lv_color_hex(0xC8D6E2);
    for (d = 0; d < 360; d = (int16_t)(d + 10)) {
      float sang = ((float)d * (float)(M_PI / 180.0)) - yaw_r;
      float dxx = sinf(sang);
      float dyy = -cosf(sang);
      float tlen = ((d % 30) == 0) ? 9.0f : 5.0f;

      tdsc.width = ((d % 90) == 0) ? 2 : 1;
      pa.x = (lv_coord_t)((float)cx + (dxx * (float)r_co));
      pa.y = (lv_coord_t)((float)cy + (dyy * (float)r_co));
      pb.x = (lv_coord_t)((float)cx + (dxx * ((float)r_co - tlen)));
      pb.y = (lv_coord_t)((float)cy + (dyy * ((float)r_co - tlen)));
      lv_draw_line(dc, &tdsc, &pa, &pb);
    }

    lv_draw_label_dsc_init(&txtdsc);
    txtdsc.opa = LV_OPA_COVER;
    txtdsc.font = &lv_font_montserrat_14;
    txtdsc.align = LV_TEXT_ALIGN_CENTER;
    for (c = 0U; c < 4U; c++) {
      float sang = ((float)card_deg[c] * (float)(M_PI / 180.0)) - yaw_r;
      float dxx = sinf(sang);
      float dyy = -cosf(sang);
      lv_coord_t lx = (lv_coord_t)((float)cx + (dxx * r_mid));
      lv_coord_t ly = (lv_coord_t)((float)cy + (dyy * r_mid));
      lv_area_t la;

      txtdsc.color = (card_deg[c] == 0) ? lv_color_hex(0xF2C14E) : lv_color_hex(0xDCE8F2);
      la.x1 = (lv_coord_t)(lx - 8);
      la.y1 = (lv_coord_t)(ly - 9);
      la.x2 = (lv_coord_t)(lx + 8);
      la.y2 = (lv_coord_t)(ly + 9);
      lv_draw_label(dc, &txtdsc, &la, card_txt[c], NULL);
    }
  }

  /* 顶部固定航向指针：指向罗盘当前航向位（航向朝上） */
  {
    lv_draw_rect_dsc_t tdsc;
    lv_point_t tri[3];
    lv_draw_rect_dsc_init(&tdsc);
    tdsc.bg_opa = LV_OPA_COVER;
    tdsc.bg_color = lv_color_hex(0xF2C14E);
    tri[0].x = cx;
    tri[0].y = (lv_coord_t)(cy - r_ci + 1);
    tri[1].x = (lv_coord_t)(cx - 6);
    tri[1].y = (lv_coord_t)(cy - r_co + 2);
    tri[2].x = (lv_coord_t)(cx + 6);
    tri[2].y = (lv_coord_t)(cy - r_co + 2);
    lv_draw_polygon(dc, &tdsc, tri, 3);
  }

  /* 固定飞机符号：黄色双翼 + 翼端下折 + 中心点 */
  {
    lv_draw_rect_dsc_t ddsc;
    lv_area_t da;
    lv_draw_line_dsc_init(&ldsc);
    ldsc.opa = LV_OPA_COVER;
    ldsc.color = lv_color_hex(0xFFC400);
    ldsc.width = 4;
    ldsc.round_start = 1;
    ldsc.round_end = 1;
    pa.x = (lv_coord_t)(cx - 46);
    pa.y = cy;
    pb.x = (lv_coord_t)(cx - 14);
    pb.y = cy;
    lv_draw_line(dc, &ldsc, &pa, &pb);
    pa.x = (lv_coord_t)(cx - 14);
    pa.y = cy;
    pb.x = (lv_coord_t)(cx - 14);
    pb.y = (lv_coord_t)(cy + 8);
    lv_draw_line(dc, &ldsc, &pa, &pb);
    pa.x = (lv_coord_t)(cx + 14);
    pa.y = cy;
    pb.x = (lv_coord_t)(cx + 46);
    pb.y = cy;
    lv_draw_line(dc, &ldsc, &pa, &pb);
    pa.x = (lv_coord_t)(cx + 14);
    pa.y = cy;
    pb.x = (lv_coord_t)(cx + 14);
    pb.y = (lv_coord_t)(cy + 8);
    lv_draw_line(dc, &ldsc, &pa, &pb);
    lv_draw_rect_dsc_init(&ddsc);
    ddsc.bg_opa = LV_OPA_COVER;
    ddsc.bg_color = lv_color_hex(0xFFC400);
    ddsc.radius = LV_RADIUS_CIRCLE;
    da.x1 = (lv_coord_t)(cx - 3);
    da.y1 = (lv_coord_t)(cy - 3);
    da.x2 = (lv_coord_t)(cx + 3);
    da.y2 = (lv_coord_t)(cy + 3);
    lv_draw_rect(dc, &ddsc, &da);
  }
}

/**
 * @brief Create aircraft attitude page.
 */
static void Display_LvglCreateAircraftPage(lv_obj_t *parent)
{
  lv_obj_t *card;
  lv_obj_t *horizon;

  Display_LvglCreateSystemColumn(parent);
  card = Display_LvglCreateCard(parent, 264, DISPLAY_LVGL_BODY_Y, 268, DISPLAY_LVGL_BODY_H, "\xE9""\xA3""\x9E""\xE6""\x9C""\xBA""\xE5""\xA7""\xBF""\xE6""\x80""\x81");

  /* 删除发送/接收/心跳三行；上半部为姿态地平仪区域：卡片内 (8,36) 起，252x212。
     地平仪由 Display_LvglHorizonDrawCb 在 DRAW_POST 事件里直绘（约 200 直径圆盘）。 */
  horizon = lv_obj_create(card);
  lv_obj_set_size(horizon, 252, 212);
  lv_obj_set_pos(horizon, 8, 36);
  lv_obj_set_style_radius(horizon, 4, 0);
  lv_obj_set_style_bg_color(horizon, lv_color_hex(0x0C1622), 0);
  lv_obj_set_style_bg_opa(horizon, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(horizon, 1, 0);
  lv_obj_set_style_border_color(horizon, lv_color_hex(0x304357), 0);
  lv_obj_set_style_pad_all(horizon, 0, 0);
  lv_obj_clear_flag(horizon, LV_OBJ_FLAG_SCROLLABLE);
  s_attitude_obj = horizon;
  lv_obj_add_event_cb(horizon, Display_LvglHorizonDrawCb, LV_EVENT_DRAW_POST, NULL);

  /* 三态姿态数据下移到卡片底部三行，行距压缩到 24px 给地平仪让出高度；
     数据整体左移，右侧腾出"校准"按钮位置。 */
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_ROLL, "\xE6""\xA8""\xAA""\xE6""\xBB""\x9A", 16, 256, 96);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_PITCH, "\xE4""\xBF""\xAF""\xE4""\xBB""\xB0", 16, 280, 96);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_YAW, "\xE5""\x81""\x8F""\xE8""\x88""\xAA", 16, 304, 96);

  /* 校准按钮：点下把当前姿态记为水平零位，地平仪以当前姿势归零。 */
  {
    lv_obj_t *cal = lv_obj_create(card);
    lv_obj_t *cal_label;

    lv_obj_set_size(cal, 74, 64);
    lv_obj_set_pos(cal, 186, 258);
    lv_obj_set_style_radius(cal, 4, 0);
    lv_obj_set_style_bg_color(cal, lv_color_hex(0x1B3A4E), 0);
    lv_obj_set_style_bg_opa(cal, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(cal, 1, 0);
    lv_obj_set_style_border_color(cal, lv_color_hex(0x1DB7C9), 0);
    lv_obj_add_flag(cal, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(cal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(cal, Display_LvglAttitudeCalEventCb, LV_EVENT_CLICKED, 0);
    cal_label = Display_LvglCreateLabel(cal, "\xE6""\xA0""\xA1""\xE5""\x87""\x86", 0, 0, &display_lvgl_font_zh_16, lv_color_hex(0xFFFFFF));
    lv_obj_center(cal_label);
  }

  Display_LvglCreateMessageLogPanel(parent, 540, 252);
}

/**
 * @brief Create GNSS page.
 */
static void Display_LvglCreateGnssPage(lv_obj_t *parent)
{
  lv_obj_t *card;

  Display_LvglCreateSystemColumn(parent);
  card = Display_LvglCreateCard(parent, 264, DISPLAY_LVGL_BODY_Y, 268, DISPLAY_LVGL_BODY_H, "\xE5""\xAE""\x9A""\xE4""\xBD""\x8D""\xE6""\x95""\xB0""\xE6""\x8D""\xAE");
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_GNSS_FIX, "\xE5""\xAE""\x9A""\xE4""\xBD""\x8D""\xE7""\x8A""\xB6""\xE6""\x80""\x81", 22, 48, 122);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_GNSS_SAT_COUNT, "\xE5""\x8D""\xAB""\xE6""\x98""\x9F""\xE6""\x95""\xB0", 22, 88, 122);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_LATITUDE, "\xE7""\xBA""\xAC""\xE5""\xBA""\xA6", 22, 128, 122);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_LONGITUDE, "\xE7""\xBB""\x8F""\xE5""\xBA""\xA6", 22, 168, 122);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_ALTITUDE, "\xE9""\xAB""\x98""\xE5""\xBA""\xA6", 22, 208, 122);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_GNSS_HDOP, "\xE7""\xB2""\xBE""\xE5""\xBA""\xA6", 22, 248, 122);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_GNSS_SPEED, "\xE9""\x80""\x9F""\xE5""\xBA""\xA6", 22, 288, 122);
  Display_LvglCreateMessageLogPanel(parent, 540, 252);
}

/**
 * @brief Create motor control page.
 */
static void Display_LvglCreateMotorPage(lv_obj_t *parent)
{
  lv_obj_t *card;
  lv_obj_t *estop;
  lv_obj_t *estop_label;
  uint16_t i;
  static const char *motor_names[DISPLAY_LVGL_MOTOR_COUNT] = {"\xE4""\xB8""\x80""\xE5""\x8F""\xB7", "\xE4""\xBA""\x8C""\xE5""\x8F""\xB7", "\xE4""\xB8""\x89""\xE5""\x8F""\xB7", "\xE5""\x9B""\x9B""\xE5""\x8F""\xB7"};
  static const lv_coord_t track_x[DISPLAY_LVGL_MOTOR_COUNT] = {50, 135, 220, 305};

  /* 系统栏收窄 204->150，油门卡片左移，消息日志加宽到 230（原 176 太窄，文字被时间列挡住）。 */
  Display_LvglCreateSystemColumnAt(parent, 8, 150);
  card = Display_LvglCreateCard(parent, 166, DISPLAY_LVGL_BODY_Y, 388, DISPLAY_LVGL_BODY_H, "\xE6""\xB2""\xB9""\xE9""\x97""\xA8""\xE6""\x8E""\xA7""\xE5""\x88""\xB6");
  for (i = 0U; i < DISPLAY_LVGL_MOTOR_COUNT; i++) {
    lv_coord_t x = track_x[i];
    lv_coord_t label_x = (lv_coord_t)(x - 10);
    Display_HmiVariableId_t id = (Display_HmiVariableId_t)((uint16_t)DISPLAY_HMI_VAR_MOTOR_PWM_1 + i);
    uint32_t initial_pulse;

    (void)Display_LvglCreateLabel(card, motor_names[i], label_x, 34, &display_lvgl_font_zh_16, lv_color_hex(0xDCE8F2));
    s_motor_pwm_bars[i] = lv_slider_create(card);
    /* 顶端下移、缩短滑轨(底端不动)，使滑点滑到 100% 时不再压住上方"X号"名称。 */
    lv_obj_set_size(s_motor_pwm_bars[i], 28, 138);
    lv_obj_set_pos(s_motor_pwm_bars[i], (lv_coord_t)(x + 1), 80);
    lv_slider_set_range(s_motor_pwm_bars[i], 0, 100);
    lv_slider_set_value(s_motor_pwm_bars[i], 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_motor_pwm_bars[i], lv_color_hex(0x263748), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_motor_pwm_bars[i], lv_color_hex(0x1DB7C9), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s_motor_pwm_bars[i], lv_color_hex(0xFFFFFF), LV_PART_KNOB);
    lv_obj_set_style_width(s_motor_pwm_bars[i], 18, LV_PART_KNOB);
    lv_obj_set_style_height(s_motor_pwm_bars[i], 18, LV_PART_KNOB);
    lv_obj_add_event_cb(s_motor_pwm_bars[i], Display_LvglMotorSliderEventCb, LV_EVENT_ALL, (void *)(uintptr_t)id);
    /* 数值区避开滑块旋钮：左列 PWM，右列脉宽，列间保留触摸与视觉间距。 */
    (void)Display_LvglCreateClipLabel(card, "PWM", (lv_coord_t)(x - 25), 234, 30, &lv_font_montserrat_12, lv_color_hex(0x7D91A6));
    (void)Display_LvglCreateClipLabel(card, "us", (lv_coord_t)(x + 13), 234, 45, &lv_font_montserrat_12, lv_color_hex(0x7D91A6));
    Display_LvglCreateValueLabel(card, id, (lv_coord_t)(x - 25), 250, 30, &lv_font_montserrat_12);
    if (s_value_slots[id].label != 0) {
      lv_obj_set_style_text_font(s_value_slots[id].label, &lv_font_montserrat_12, 0);
      lv_obj_set_style_text_align(s_value_slots[id].label, LV_TEXT_ALIGN_RIGHT, 0);
    }
    s_motor_pulse_labels[i] = Display_LvglCreateClipLabel(card, s_motor_pulse_text[i], (lv_coord_t)(x + 13), 250, 45, &lv_font_montserrat_12, lv_color_hex(0xDCE8F2));
    initial_pulse = Display_LvglMotorPulseUs(((id < DISPLAY_HMI_VAR_COUNT) && (s_value_valid[id] != 0U)) ? s_values[id] : 0U);
    Display_LvglUpdateMotorPulseLabel((uint8_t)i, initial_pulse);
  }

  estop = lv_obj_create(card);
  lv_obj_set_size(estop, DISPLAY_MOTOR_ESTOP_W, DISPLAY_MOTOR_ESTOP_H);
  /* 急停按钮在卡片内水平居中，纵向沿用原位置；相对卡片定位，卡片左移后仍正确。 */
  lv_obj_set_pos(estop, (lv_coord_t)((388U - DISPLAY_MOTOR_ESTOP_W) / 2U), (lv_coord_t)(DISPLAY_MOTOR_ESTOP_Y - DISPLAY_LVGL_BODY_Y));
  lv_obj_set_style_radius(estop, 4, 0);
  lv_obj_set_style_bg_color(estop, lv_color_hex(0x4E1B25), 0);
  lv_obj_set_style_bg_opa(estop, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(estop, 1, 0);
  lv_obj_set_style_border_color(estop, lv_color_hex(0xE85D75), 0);
  lv_obj_add_flag(estop, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(estop, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(estop, Display_LvglEstopEventCb, LV_EVENT_CLICKED, 0);
  estop_label = Display_LvglCreateLabel(estop, "\xE6""\x80""\xA5""\xE5""\x81""\x9C", 0, 0, &display_lvgl_font_zh_16, lv_color_hex(0xFFFFFF));
  lv_obj_center(estop_label);

  Display_LvglCreateMessageLogPanel(parent, 562, 230);
}

/**
 * @brief Create alarm page.
 */
static void Display_LvglCreateAlarmPage(lv_obj_t *parent)
{
  (void)Display_LvglCreateLabel(parent, "\xE5""\x91""\x8A""\xE8""\xAD""\xA6""\xE6""\xB1""\x87""\xE6""\x80""\xBB", 52, 72, &display_lvgl_font_zh_16, lv_color_hex(0xF76D7E));
  Display_LvglCreateAlarmTable(parent, 40, 110, 720, 291);
}

/**
 * @brief Create hidden LoRa-link page placeholder.
 */
static void Display_LvglCreateHiddenPage(lv_obj_t *parent)
{
  lv_obj_t *card;
  lv_obj_t *back_btn;
  lv_obj_t *back_label;
  lv_obj_t *list;
  uint8_t count = 0U;
  uint8_t i;
  uint8_t selected_node = 0xFFU;

  card = Display_LvglCreateCard(parent, 70, 96, 660, 292, "\xE9""\x80""\x9A""\xE4""\xBF""\xA1""\xE8""\xBF""\x9E""\xE6""\x8E""\xA5");

  /* 返回本地按钮 */
  back_btn = lv_obj_create(card);
  lv_obj_set_size(back_btn, 120, 32);
  lv_obj_set_pos(back_btn, 32, 34);
  lv_obj_set_style_radius(back_btn, 4, 0);
  lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x25384A), 0);
  lv_obj_set_style_bg_opa(back_btn, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(back_btn, 1, 0);
  lv_obj_set_style_border_color(back_btn, lv_color_hex(0x1DB7C9), 0);
  lv_obj_set_style_pad_all(back_btn, 0, 0);
  lv_obj_add_flag(back_btn, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(back_btn, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(back_btn, Display_LvglLocalRemoteEventCb, LV_EVENT_CLICKED, 0);
  back_label = Display_LvglCreateLabel(back_btn, "\xE8""\xBF""\x94""\xE5""\x9B""\x9E""\xE6""\x9C""\xAC""\xE5""\x9C""\xB0", 0, 0, &display_lvgl_font_zh_16, lv_color_hex(0xFFFFFF));
  lv_obj_center(back_label);

  (void)Display_LvglCreateLabel(card, "\xE9""\x80""\x9A""\xE4""\xBF""\xA1""\xE8""\x8A""\x82""\xE7""\x82""\xB9""\xE9""\x80""\x89""\xE6""\x8B""\xA9", 32, 82, &display_lvgl_font_zh_16, lv_color_hex(0xDCE8F2));
  (void)App_GetSelectedRemoteNode(&selected_node);

  /* 远端节点列表：每行一个节点，含状态灯、身份、接收计数与丢包率 */
  list = lv_obj_create(card);
  lv_obj_set_size(list, 318, 188);
  lv_obj_set_pos(list, 306, 66);
  lv_obj_set_style_radius(list, 4, 0);
  lv_obj_set_style_bg_opa(list, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(list, 0, 0);
  lv_obj_set_style_pad_all(list, 0, 0);
  lv_obj_add_flag(list, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scroll_dir(list, LV_DIR_VER);
  lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_AUTO);

  if ((App_CopyRemoteNodeStatuses(s_remote_node_views, DISPLAY_LVGL_REMOTE_ROWS, &count, s_last_tick_ms) == PX4LITE_OK) && (count != 0U)) {
    for (i = 0U; (i < count) && (i < DISPLAY_LVGL_REMOTE_ROWS); i++) {
      lv_obj_t *btn;
      lv_obj_t *label;
      lv_obj_t *dot;
      lv_color_t dot_color;
      uint8_t selected = (uint8_t)(s_remote_node_views[i].node_id == selected_node);

      if ((s_remote_node_views[i].state == APP_REMOTE_NODE_ACTIVE) || (s_remote_node_views[i].state == APP_REMOTE_NODE_DISCOVERED)) {
        dot_color = lv_palette_main(LV_PALETTE_GREEN);
      } else {
        dot_color = lv_palette_main(LV_PALETTE_AMBER);
      }

      (void)snprintf(s_remote_node_texts[i], sizeof(s_remote_node_texts[i]), "DCDW-%03u  RX %lu  Loss %u.%u%%",
                     (unsigned int)s_remote_node_views[i].node_id,
                     (unsigned long)s_remote_node_views[i].rx_frame_count,
                     (unsigned int)(s_remote_node_views[i].rx_loss_rate_x10 / 10U),
                     (unsigned int)(s_remote_node_views[i].rx_loss_rate_x10 % 10U));
      btn = lv_obj_create(list);
      lv_obj_set_size(btn, 300, 30);
      lv_obj_set_pos(btn, 0, (lv_coord_t)(i * 36U));
      lv_obj_set_style_radius(btn, 4, 0);
      lv_obj_set_style_bg_color(btn, selected ? lv_color_hex(0x143747) : lv_color_hex(0x25384A), 0);
      lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
      lv_obj_set_style_border_width(btn, 1, 0);
      lv_obj_set_style_border_color(btn, selected ? lv_color_hex(0x1DB7C9) : lv_color_hex(0x2F4A60), 0);
      lv_obj_set_style_pad_all(btn, 0, 0);
      lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
      lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
      lv_obj_add_event_cb(btn, Display_LvglRemoteNodeEventCb, LV_EVENT_CLICKED, (void *)(uintptr_t)s_remote_node_views[i].node_id);
      dot = lv_obj_create(btn);
      lv_obj_set_size(dot, 12, 12);
      lv_obj_set_pos(dot, 10, 9);
      lv_obj_set_style_radius(dot, 6, 0);
      lv_obj_set_style_bg_color(dot, dot_color, 0);
      lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
      lv_obj_set_style_border_width(dot, 0, 0);
      lv_obj_set_style_pad_all(dot, 0, 0);
      lv_obj_clear_flag(dot, LV_OBJ_FLAG_SCROLLABLE);
      label = Display_LvglCreateLabel(btn, s_remote_node_texts[i], 30, 6, &lv_font_montserrat_14, lv_color_hex(0xFFFFFF));
      (void)label;
    }
  } else {
    (void)Display_LvglCreateLabel(list, "No remote heartbeat", 16, 30, &lv_font_montserrat_14, lv_color_hex(0x7D91A6));
  }

  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_LORA_TX_COUNT, "\xE5""\x8F""\x91""\xE9""\x80""\x81""\xE8""\xAE""\xA1""\xE6""\x95""\xB0", 32, 148, 180);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_LORA_RX_COUNT, "\xE6""\x8E""\xA5""\xE6""\x94""\xB6""\xE8""\xAE""\xA1""\xE6""\x95""\xB0", 32, 188, 180);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_LORA_LOSS_RATE, "\xE4""\xB8""\xA2""\xE5""\x8C""\x85""\xE7""\x8E""\x87", 32, 228, 180);

  /* 记录本次构建的节点列表签名；周期检查只在签名变化时才重建，静止时不重绘。 */
  s_remote_list_sig = Display_LvglRemoteNodeSig(s_remote_node_views, count);
}

/**
 * @brief Rebuild the active LVGL page.
 */
static Display_Result_t Display_LvglCreatePage(Display_HmiPage_t page)
{
  if (page == DISPLAY_HMI_PAGE_LOGO) {
    page = DISPLAY_HMI_PAGE_SELF_CHECK;
  }
  if (page >= DISPLAY_HMI_PAGE_COUNT) {
    return DISPLAY_ERROR;
  }

  if (s_screen == 0) {
    s_screen = lv_obj_create(0);
    lv_scr_load(s_screen);
  } else {
    lv_obj_clean(s_screen);
  }

  Display_LvglClearActiveObjects();
  s_current_lvgl_page = page;

  lv_obj_set_size(s_screen, DISPLAY_LVGL_WIDTH, DISPLAY_LVGL_HEIGHT);
  lv_obj_set_style_bg_color(s_screen, lv_color_hex(0x08111A), 0);
  lv_obj_set_style_bg_opa(s_screen, LV_OPA_COVER, 0);
  lv_obj_set_style_pad_all(s_screen, 0, 0);
  lv_obj_clear_flag(s_screen, LV_OBJ_FLAG_SCROLLABLE);

  Display_LvglCreateHeader(s_screen, page);

  switch (page) {
    case DISPLAY_HMI_PAGE_SELF_CHECK:
      Display_LvglCreateSelfCheckPage(s_screen);
      break;
    case DISPLAY_HMI_PAGE_FLIGHT:
      Display_LvglCreateFlightPage(s_screen);
      break;
    case DISPLAY_HMI_PAGE_AIRCRAFT:
      Display_LvglCreateAircraftPage(s_screen);
      break;
    case DISPLAY_HMI_PAGE_DATA:
      Display_LvglCreateGnssPage(s_screen);
      break;
    case DISPLAY_HMI_PAGE_MOTOR:
      Display_LvglCreateMotorPage(s_screen);
      break;
    case DISPLAY_HMI_PAGE_ALARM:
      Display_LvglCreateAlarmPage(s_screen);
      break;
    case DISPLAY_HMI_PAGE_HIDDEN:
      Display_LvglCreateHiddenPage(s_screen);
      break;
    default:
      return DISPLAY_ERROR;
  }

  if (page != DISPLAY_HMI_PAGE_HIDDEN) {
    Display_LvglCreateFooter(s_screen, page);
    s_next_remote_list_rebuild_ms = 0U;
  } else {
    s_next_remote_list_rebuild_ms = s_last_tick_ms + DISPLAY_LVGL_REMOTE_LIST_REFRESH_MS;
  }

  Display_LvglApplyCachedValues();
  return DISPLAY_OK;
}

/**
 * @brief Initialize LVGL and its Display-owned ports.
 */
Display_Result_t Display_LvglInit(uint32_t now_ms)
{
  if (s_lvgl_core_ready == 0U) {
    lv_init();
    s_lvgl_core_ready = 1U;
  }

  if (LvPortDisp_Init() == 0U) {
    s_lvgl_display_ready = 0U;
    return DISPLAY_ERROR;
  }

  s_lvgl_display_ready = 1U;
  (void)LvPortIndev_Init();

  if (Display_LvglCreatePage(DISPLAY_HMI_PAGE_SELF_CHECK) != DISPLAY_OK) {
    return DISPLAY_ERROR;
  }

  s_last_tick_ms = now_ms;
  return DISPLAY_OK;
}

/**
 * @brief Self-check LVGL display backend.
 */
Display_Result_t Display_LvglSelfCheck(uint16_t *error_code)
{
  if (error_code != 0) {
    *error_code = 0U;
  }

  if (s_lvgl_display_ready == 0U) {
    if (error_code != 0) {
      *error_code = 1U;
    }
    return DISPLAY_NOT_READY;
  }

  return DISPLAY_OK;
}

/**
 * @brief Request display controller recovery from the Display task path.
 */
void Display_LvglRequestRecover(void)
{
  s_lvgl_display_ready = 0U;
}

/**
 * @brief Check whether an immediate page rebuild is pending.
 */
uint8_t Display_LvglNeedsRefresh(void)
{
  if (s_lvgl_display_ready == 0U) {
    return 0U;
  }

  return ((s_page_change_requested != 0U) || (s_page_rebuild_requested != 0U)) ? 1U : 0U;
}

/**
 * @brief Service LVGL timers and pending dirty-area flushes.
 */
Display_Result_t Display_LvglRefreshStep(uint32_t now_ms, uint32_t budget_us)
{
  uint32_t elapsed_ms;

  (void)budget_us;
  if (s_lvgl_display_ready == 0U) {
    return DISPLAY_NOT_READY;
  }

  /* 远端视图过期(断链/超时)自动回本地，避免长期停留在过期远端页面。 */
  if ((App_GetRemoteDisplayMode() == PX4LITE_REMOTE_MODE_REMOTE) &&
      (s_current_lvgl_page != DISPLAY_HMI_PAGE_HIDDEN) &&
      (App_RemoteViewExpired(now_ms) != 0U)) {
    Display_LvglRequestLocalView(s_current_lvgl_page, 1U);
  }

  /* 通信连接页：仅当节点上下线/状态变化(签名变化)时才整页重建，静止时不重绘，消除屏闪。 */
  if ((s_current_lvgl_page == DISPLAY_HMI_PAGE_HIDDEN) &&
      (s_page_change_requested == 0U) &&
      (s_next_remote_list_rebuild_ms != 0U) &&
      ((int32_t)(now_ms - s_next_remote_list_rebuild_ms) >= 0)) {
    App_RemoteNodeView_t probe[DISPLAY_LVGL_REMOTE_ROWS];
    uint8_t probe_count = 0U;
    uint32_t sig;

    if (App_CopyRemoteNodeStatuses(probe, DISPLAY_LVGL_REMOTE_ROWS, &probe_count, now_ms) != PX4LITE_OK) {
      probe_count = 0U;
    }
    sig = Display_LvglRemoteNodeSig(probe, probe_count);
    if (sig != s_remote_list_sig) {
      s_requested_page         = DISPLAY_HMI_PAGE_HIDDEN;
      s_page_change_requested  = 1U;
      s_page_rebuild_requested = 1U;
    }
    s_next_remote_list_rebuild_ms = now_ms + DISPLAY_LVGL_REMOTE_LIST_REFRESH_MS;
  }

  if (s_page_change_requested != 0U) {
    s_page_change_requested = 0U;
    if ((s_requested_page != s_current_lvgl_page) || (s_page_rebuild_requested != 0U)) {
      s_page_rebuild_requested = 0U;
      (void)Display_SetHmiPage(s_requested_page);
    }
  }

  elapsed_ms = now_ms - s_last_tick_ms;
  if (elapsed_ms != 0U) {
    lv_tick_inc(elapsed_ms);
    s_last_tick_ms = now_ms;
  }

  lv_timer_handler();
  return DISPLAY_OK;
}

/**
 * @brief Update one LVGL value from the existing Display HMI cache path.
 */
Display_Result_t Display_LvglSetValue(Display_HmiVariableId_t id, uint32_t value)
{
  if (id >= DISPLAY_HMI_VAR_COUNT) {
    return DISPLAY_ERROR;
  }

  s_values[id]      = value;
  s_value_valid[id] = 1U;
  Display_LvglApplyValue(id, value);
  return DISPLAY_OK;
}

/**
 * @brief Switch the active formal LVGL page.
 */
Display_Result_t Display_LvglSetPage(Display_HmiPage_t page)
{
  if (s_lvgl_display_ready == 0U) {
    return DISPLAY_NOT_READY;
  }

  return Display_LvglCreatePage(page);
}
