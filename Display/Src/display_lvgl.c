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

#include "app_data_api.h"
#include "display_log.h"
#include "display_lvgl_font_zh.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "lvgl.h"
#include "px4lite_platform.h"

#include <stdio.h>

#define DISPLAY_LVGL_WIDTH           800U
#define DISPLAY_LVGL_HEIGHT          480U
#define DISPLAY_LVGL_HEADER_H        64U
#define DISPLAY_LVGL_BODY_Y          76U
#define DISPLAY_LVGL_BODY_H          330U
#define DISPLAY_LVGL_FOOTER_Y        424U
#define DISPLAY_LVGL_FOOTER_H        56U
#define DISPLAY_LVGL_CARD_RADIUS     6U
#define DISPLAY_LVGL_VALUE_TEXT_LEN  32U
#define DISPLAY_LVGL_STATUS_COUNT    9U
#define DISPLAY_LVGL_TAB_COUNT       6U
#define DISPLAY_LVGL_MOTOR_COUNT     4U
#define DISPLAY_LVGL_LOG_ROWS        9U
#define DISPLAY_LVGL_ALARM_ROWS      5U
#define DISPLAY_LVGL_REMOTE_ROWS     5U
#define DISPLAY_LVGL_LCD_PROBE_MS    1000U

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
    {DISPLAY_HMI_VAR_SELF_CHECK_LORA, "\xE9""\x80""\x9A""\xE4""\xBF""\xA1""\xE6""\xA8""\xA1""\xE5""\x9D""\x97"},
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
static uint32_t s_values[DISPLAY_HMI_VAR_COUNT];
static uint8_t s_value_valid[DISPLAY_HMI_VAR_COUNT];
static char s_remote_node_texts[DISPLAY_LVGL_REMOTE_ROWS][48];
static lv_obj_t *s_screen;
static lv_obj_t *s_status_leds[DISPLAY_LVGL_STATUS_COUNT];
static lv_obj_t *s_motor_pwm_bars[DISPLAY_LVGL_MOTOR_COUNT];
static lv_obj_t *s_log_alarm_label;
static Display_LvglLogRow_t s_log_rows[DISPLAY_LVGL_LOG_ROWS];
static Display_LvglAlarmRow_t s_alarm_rows[DISPLAY_LVGL_ALARM_ROWS];
static uint8_t s_log_visible_rows;
static Display_HmiPage_t s_current_lvgl_page = DISPLAY_HMI_PAGE_SELF_CHECK;
static Display_HmiPage_t s_requested_page    = DISPLAY_HMI_PAGE_SELF_CHECK;
static Display_HmiPage_t s_page_before_hidden = DISPLAY_HMI_PAGE_SELF_CHECK; /* 进入远端(隐藏)页前的页面，供本地/远端按钮返回 */
static uint8_t s_page_change_requested;
static uint8_t s_lvgl_core_ready;
static uint8_t s_lvgl_display_ready;
static uint8_t s_control_update_active;
static uint32_t s_last_tick_ms;
static uint32_t s_next_lcd_probe_ms;
static Display_DebugStats_t s_debug_stats;

static void Display_LvglTouchActivity(void)
{
  if (Display_GetDataSource() == DISPLAY_DATA_SOURCE_REMOTE) {
    (void)App_SetRemoteViewEnabled(1U, s_last_tick_ms);
  }
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
  if (value == 2U) {
    return lv_palette_main(LV_PALETTE_GREEN);
  }
  if (value == 1U) {
    return lv_palette_main(LV_PALETTE_AMBER);
  }
  if (value == 0U) {
    return lv_palette_main(LV_PALETTE_GREY);
  }
  return lv_palette_main(LV_PALETTE_RED);
}

/**
 * @brief Get readable status text for self-check fields.
 */
static const char *Display_LvglStatusText(uint32_t value)
{
  if (value == 2U) {
    return "\xE6""\xAD""\xA3""\xE5""\xB8""\xB8";
  }
  if (value == 1U) {
    return "\xE8""\xAD""\xA6""\xE5""\x91""\x8A";
  }
  if (value == 0U) {
    return "\xE6""\x9C""\xAA""\xE5""\xB0""\xB1""\xE7""\xBB""\xAA";
  }
  return "\xE6""\x95""\x85""\xE9""\x9A""\x9C";
}

/**
 * @brief Clear active object pointers before rebuilding the current page.
 */
static void Display_LvglClearActiveObjects(void)
{
  uint16_t i;

  for (i = 0U; i < DISPLAY_HMI_VAR_COUNT; i++) {
    s_value_slots[i].label = 0;
  }
  for (i = 0U; i < DISPLAY_LVGL_STATUS_COUNT; i++) {
    s_status_leds[i] = 0;
  }
  for (i = 0U; i < DISPLAY_LVGL_MOTOR_COUNT; i++) {
    s_motor_pwm_bars[i] = 0;
  }
  for (i = 0U; i < DISPLAY_LVGL_LOG_ROWS; i++) {
    s_log_rows[i].time_label = 0;
    s_log_rows[i].msg_label  = 0;
  }
  for (i = 0U; i < DISPLAY_LVGL_ALARM_ROWS; i++) {
    s_alarm_rows[i].bar          = 0;
    s_alarm_rows[i].code_label   = 0;
    s_alarm_rows[i].module_label = 0;
    s_alarm_rows[i].reason_label = 0;
  }
  s_log_alarm_label = 0;
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

  (void)Display_LvglCreateLabel(card, title, 22, 8, &display_lvgl_font_zh_16, lv_color_hex(0xDCE8F2));
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
    case DISPLAY_HMI_VAR_VIEW_NODE_ID:
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "ID:%lu", (unsigned long)value);
      break;
    case DISPLAY_HMI_VAR_UPTIME_MS:
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%lu""\xE7""\xA7""\x92", (unsigned long)(value / 1000U));
      break;
    case DISPLAY_HMI_VAR_FLIGHT_TIME_S:
      hh = value / 3600U;
      mm = (value / 60U) % 60U;
      ss = value % 60U;
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%02lu:%02lu:%02lu", (unsigned long)hh, (unsigned long)mm, (unsigned long)ss);
      break;
    case DISPLAY_HMI_VAR_BATTERY_VOLTAGE:
    case DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE:
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%lu.%02lu""\xE4""\xBC""\x8F", (unsigned long)(value / 100U), (unsigned long)(value % 100U));
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
    case DISPLAY_HMI_VAR_SYSTEM_STATUS:
    case DISPLAY_HMI_VAR_SELF_CHECK_GNSS:
    case DISPLAY_HMI_VAR_SELF_CHECK_MPU6050:
    case DISPLAY_HMI_VAR_SELF_CHECK_BME280:
    case DISPLAY_HMI_VAR_SELF_CHECK_LORA:
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

  if (id >= DISPLAY_HMI_VAR_COUNT) {
    return;
  }

  label = lv_label_create(parent);
  s_value_slots[id].label = label;
  (void)font;
  lv_obj_set_style_text_font(label, &display_lvgl_font_zh_16, 0);
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
      return "\xE9""\x80""\x9A""\xE4""\xBF""\xA1""\xE6""\xAD""\xA3""\xE5""\xB8""\xB8";
    case DISPLAY_LOGMSG_COMM_LOST:
      return "\xE9""\x80""\x9A""\xE4""\xBF""\xA1""\xE6""\x96""\xAD""\xE5""\xBC""\x80";
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
  count = Display_LogCopyMessages(entries, visible_rows, &alarm_entry, &alarm_valid);

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
  return App_GetModuleDisplayName(source_id, fault_code);
}

static const char *Display_LvglAlarmReasonText(uint16_t code)
{
  return App_GetFaultReasonText(code);
}

static void Display_LvglUpdateAlarmRow(uint8_t row, uint32_t value)
{
  uint16_t source_id;
  uint16_t fault_code;

  if (row >= DISPLAY_LVGL_ALARM_ROWS) {
    return;
  }
  if (s_alarm_rows[row].code_label == 0) {
    return;
  }

  source_id  = (uint16_t)(value >> 16);
  fault_code = (uint16_t)value;

  if (fault_code == 0U) {
    (void)snprintf(s_alarm_rows[row].code_text, sizeof(s_alarm_rows[row].code_text), "--");
    lv_label_set_text_static(s_alarm_rows[row].code_label, s_alarm_rows[row].code_text);
    lv_label_set_text_static(s_alarm_rows[row].module_label, "");
    lv_label_set_text_static(s_alarm_rows[row].reason_label, (row == 0U) ? "\xE6""\x97""\xA0""\xE8""\xAE""\xB0""\xE5""\xBD""\x95" : "");
    if (s_alarm_rows[row].bar != 0) {
      lv_obj_set_style_bg_opa(s_alarm_rows[row].bar, LV_OPA_TRANSP, 0);
    }
    return;
  }

  Display_LvglFormatFaultCode(s_alarm_rows[row].code_text, fault_code);
  lv_label_set_text_static(s_alarm_rows[row].code_label, s_alarm_rows[row].code_text);
  lv_label_set_text_static(s_alarm_rows[row].module_label, Display_LvglAlarmModuleText(source_id, fault_code));
  lv_label_set_text_static(s_alarm_rows[row].reason_label, Display_LvglAlarmReasonText(fault_code));
  if (s_alarm_rows[row].bar != 0) {
    lv_obj_set_style_bg_opa(s_alarm_rows[row].bar, LV_OPA_COVER, 0);
  }
}

static void Display_LvglUpdateAlarmTable(void)
{
  uint8_t row;

  for (row = 0U; row < DISPLAY_LVGL_ALARM_ROWS; row++) {
    Display_HmiVariableId_t id = (Display_HmiVariableId_t)((uint16_t)DISPLAY_HMI_VAR_ALARM_ROW1_CODE + row);
    if (s_value_valid[id] != 0U) {
      Display_LvglUpdateAlarmRow(row, s_values[id]);
    } else {
      Display_LvglUpdateAlarmRow(row, 0U);
    }
  }
}

static void Display_LvglMotorSliderEventCb(lv_event_t *event)
{
  lv_obj_t *slider;
  Display_HmiVariableId_t id;
  int32_t value;

  if ((lv_event_get_code(event) != LV_EVENT_VALUE_CHANGED) || (s_control_update_active != 0U)) {
    return;
  }
  Display_LvglTouchActivity();

  slider = lv_event_get_target(event);
  id     = (Display_HmiVariableId_t)(uintptr_t)lv_event_get_user_data(event);
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
  Display_LvglTouchActivity();

  (void)Display_RequestMotorEmergencyStop();
}

/**
 * @brief Apply one cached value to currently active LVGL objects.
 */
static void Display_LvglApplyValue(Display_HmiVariableId_t id, uint32_t value)
{
  uint8_t motor_index;

  if (id >= DISPLAY_HMI_VAR_COUNT) {
    return;
  }

  Display_LvglUpdateStatusLed(id, value);

  if ((id >= DISPLAY_HMI_VAR_MOTOR_PWM_1) && (id <= DISPLAY_HMI_VAR_MOTOR_PWM_4)) {
    motor_index = (uint8_t)((uint16_t)id - (uint16_t)DISPLAY_HMI_VAR_MOTOR_PWM_1);
    if ((motor_index < DISPLAY_LVGL_MOTOR_COUNT) && (s_motor_pwm_bars[motor_index] != 0)) {
      s_control_update_active = 1U;
      lv_slider_set_value(s_motor_pwm_bars[motor_index], Display_LvglClampPercent(value), LV_ANIM_OFF);
      s_control_update_active = 0U;
    }
  }

  if (id == DISPLAY_HMI_VAR_MESSAGE_LOG) {
    Display_LvglUpdateMessageLog();
  }

  if ((id >= DISPLAY_HMI_VAR_ALARM_ROW1_CODE) && (id <= DISPLAY_HMI_VAR_ALARM_ROW5_CODE)) {
    Display_LvglUpdateAlarmRow((uint8_t)((uint16_t)id - (uint16_t)DISPLAY_HMI_VAR_ALARM_ROW1_CODE), value);
  }

  if (s_value_slots[id].label != 0) {
    Display_LvglFormatValue(id, value);
    lv_label_set_text_static(s_value_slots[id].label, s_value_slots[id].text);
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
  Display_LvglTouchActivity();

  s_requested_page          = (Display_HmiPage_t)(uintptr_t)lv_event_get_user_data(event);
  s_page_change_requested  = 1U;
}

/**
 * @brief 本地/远端按钮：在常规页(本地)与隐藏的通信连接页(远端)之间切换。
 * @note  取代 LVGL 后端下已停用的 KEY0；与切页一样延迟到刷新步执行。
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
    (void)Display_SetDataSource(DISPLAY_DATA_SOURCE_REMOTE);
    s_requested_page        = s_page_before_hidden;
    s_page_change_requested = 1U;
  }
}

static void Display_LvglLocalRemoteEventCb(lv_event_t *event)
{
  if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
    return;
  }
  Display_LvglTouchActivity();

  if (s_current_lvgl_page == DISPLAY_HMI_PAGE_HIDDEN) {
    (void)App_SetRemoteViewEnabled(0U, s_last_tick_ms);
    (void)Display_SetDataSource(DISPLAY_DATA_SOURCE_LOCAL);
    s_requested_page = s_page_before_hidden;
  } else {
    s_page_before_hidden = s_current_lvgl_page;
    s_requested_page     = DISPLAY_HMI_PAGE_HIDDEN;
  }
  s_page_change_requested = 1U;
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

  (void)Display_LvglCreateLabel(bar, "\xE9""\xA3""\x9E""\xE6""\x8E""\xA7""\xE6""\x98""\xBE""\xE7""\xA4""\xBA""\xE7""\xB3""\xBB""\xE7""\xBB""\x9F", 344, 10, &display_lvgl_font_zh_16, lv_color_hex(0xFFFFFF));
  (void)Display_LvglCreateLabel(bar, Display_LvglPageTitle(page), 368, 35, &display_lvgl_font_zh_16, lv_color_hex(0x1DB7C9));
  (void)Display_LvglCreateLabel(bar, "\xE6""\x97""\xA5""\xE6""\x9C""\x9F", 18, 12, &display_lvgl_font_zh_16, lv_color_hex(0x7D91A6));
  Display_LvglCreateValueLabel(bar, DISPLAY_HMI_VAR_DATE, 58, 10, 116, &lv_font_montserrat_14);
  (void)Display_LvglCreateLabel(bar, "\xE6""\x97""\xB6""\xE9""\x97""\xB4", 18, 36, &display_lvgl_font_zh_16, lv_color_hex(0x7D91A6));
  Display_LvglCreateValueLabel(bar, DISPLAY_HMI_VAR_CLOCK_TIME, 58, 34, 116, &lv_font_montserrat_14);
  Display_LvglCreateValueLabel(bar, DISPLAY_HMI_VAR_VIEW_NODE_ID, 178, 10, 74, &lv_font_montserrat_14);
  /* 本地/远端按钮：放在"系统"左侧，点按切换本地常规页 / 远端通信连接页；
     标题随当前页显示"本地"或"远端"，按在远端页时高亮。 */
  {
    lv_obj_t *lr_btn;
    lv_obj_t *lr_label;
    uint8_t on_hidden = (uint8_t)(page == DISPLAY_HMI_PAGE_HIDDEN);

    lr_btn = lv_obj_create(bar);
    lv_obj_set_size(lr_btn, 70, 32);
    lv_obj_set_pos(lr_btn, 596, 16);
    lv_obj_set_style_radius(lr_btn, 4, 0);
    lv_obj_set_style_bg_color(lr_btn, on_hidden ? lv_color_hex(0x1DB7C9) : lv_color_hex(0x143747), 0);
    lv_obj_set_style_bg_opa(lr_btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(lr_btn, 1, 0);
    lv_obj_set_style_border_color(lr_btn, lv_color_hex(0x1DB7C9), 0);
    lv_obj_set_style_pad_all(lr_btn, 0, 0);
    lv_obj_add_flag(lr_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(lr_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(lr_btn, Display_LvglLocalRemoteEventCb, LV_EVENT_CLICKED, 0);
    lr_label = Display_LvglCreateLabel(lr_btn, on_hidden ? "\xE8""\xBF""\x9C""\xE7""\xAB""\xAF" : "\xE6""\x9C""\xAC""\xE5""\x9C""\xB0", 0, 0, &display_lvgl_font_zh_16, lv_color_hex(0xFFFFFF));
    lv_obj_center(lr_label);
  }

  (void)Display_LvglCreateLabel(bar, "\xE7""\xB3""\xBB""\xE7""\xBB""\x9F", 690, 12, &display_lvgl_font_zh_16, lv_color_hex(0x7D91A6));
  Display_LvglCreateValueLabel(bar, DISPLAY_HMI_VAR_SYSTEM_STATUS, 728, 10, 58, &lv_font_montserrat_14);
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
  for (i = 0U; i < DISPLAY_LVGL_STATUS_COUNT; i++) {
    lv_coord_t y = (lv_coord_t)(48 + (i * 30U));
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

  card = Display_LvglCreateCard(parent, x, y, w, h, "\xE6""\xB6""\x88""\xE6""\x81""\xAF""\xE6""\x97""\xA5""\xE5""\xBF""\x97");
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

static void Display_LvglCreateAlarmTable(lv_obj_t *parent, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h)
{
  lv_obj_t *table;
  lv_obj_t *header;
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
  row_h       = 50;

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

  (void)Display_LvglCreateClipLabel(header, "\xE4""\xBB""\xA3""\xE7""\xA0""\x81", 59, 10, 50, &display_lvgl_font_zh_16, lv_color_hex(0xFFFFFF));
  (void)Display_LvglCreateClipLabel(header, "\xE6""\xA8""\xA1""\xE5""\x9D""\x97", (lv_coord_t)(code_col_w + 69), 10, 52, &display_lvgl_font_zh_16, lv_color_hex(0xFFFFFF));
  (void)Display_LvglCreateClipLabel(header, "\xE5""\x8E""\x9F""\xE5""\x9B""\xA0", (lv_coord_t)(reason_col_x + 184), 10, 52, &display_lvgl_font_zh_16, lv_color_hex(0xFFFFFF));

  for (i = 0U; i < DISPLAY_LVGL_ALARM_ROWS; i++) {
    lv_coord_t row_y = (lv_coord_t)(row_y0 + (i * row_h));
    lv_obj_t *row_bg = lv_obj_create(table);
    lv_obj_set_size(row_bg, w, row_h);
    lv_obj_set_pos(row_bg, 0, row_y);
    lv_obj_set_style_radius(row_bg, 0, 0);
    lv_obj_set_style_bg_color(row_bg, (i & 1U) ? lv_color_hex(0x172738) : lv_color_hex(0x101B27), 0);
    lv_obj_set_style_bg_opa(row_bg, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(row_bg, 0, 0);
    lv_obj_set_style_pad_all(row_bg, 0, 0);
    lv_obj_clear_flag(row_bg, LV_OBJ_FLAG_SCROLLABLE);

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

  for (i = 1U; i < DISPLAY_LVGL_ALARM_ROWS; i++) {
    lv_obj_t *line = lv_obj_create(table);
    lv_obj_set_size(line, w, 1);
    lv_obj_set_pos(line, 0, (lv_coord_t)(row_y0 + (i * row_h)));
    lv_obj_set_style_radius(line, 0, 0);
    lv_obj_set_style_bg_color(line, lv_color_hex(0x304357), 0);
    lv_obj_set_style_bg_opa(line, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(line, 0, 0);
    lv_obj_clear_flag(line, LV_OBJ_FLAG_SCROLLABLE);
  }
  for (i = 0U; i < 2U; i++) {
    lv_obj_t *line = lv_obj_create(table);
    lv_coord_t line_x = (i == 0U) ? code_col_w : reason_col_x;
    lv_obj_set_size(line, 1, h);
    lv_obj_set_pos(line, line_x, 0);
    lv_obj_set_style_radius(line, 0, 0);
    lv_obj_set_style_bg_color(line, lv_color_hex(0x304357), 0);
    lv_obj_set_style_bg_opa(line, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(line, 0, 0);
    lv_obj_clear_flag(line, LV_OBJ_FLAG_SCROLLABLE);
  }

  Display_LvglUpdateAlarmTable();
}

/*
 * 自检汇总告警列表：与告警页同一数据源（DISPLAY_HMI_VAR_ALARM_ROW1..5），
 * 复用 s_alarm_rows[]，现有刷新机制（Display_LvglUpdateAlarmRow）自动保持同步。
 * 窄卡片内用两行紧凑布局：第一行 代码 + 模块，第二行 原因，完整显示三项。
 */
static void Display_LvglCreateSummaryAlarmList(lv_obj_t *card)
{
  uint16_t i;

  for (i = 0U; i < DISPLAY_LVGL_ALARM_ROWS; i++) {
    lv_coord_t row_y = (lv_coord_t)(44 + (i * 50));

    if (i != 0U) {
      lv_obj_t *line = lv_obj_create(card);
      lv_obj_set_size(line, 204, 1);
      lv_obj_set_pos(line, 12, (lv_coord_t)(row_y - 6));
      lv_obj_set_style_radius(line, 0, 0);
      lv_obj_set_style_bg_color(line, lv_color_hex(0x223547), 0);
      lv_obj_set_style_bg_opa(line, LV_OPA_COVER, 0);
      lv_obj_set_style_border_width(line, 0, 0);
      lv_obj_clear_flag(line, LV_OBJ_FLAG_SCROLLABLE);
    }

    s_alarm_rows[i].bar = lv_obj_create(card);
    lv_obj_set_size(s_alarm_rows[i].bar, 4, 40);
    lv_obj_set_pos(s_alarm_rows[i].bar, 4, (lv_coord_t)(row_y + 2));
    lv_obj_set_style_radius(s_alarm_rows[i].bar, 0, 0);
    lv_obj_set_style_bg_color(s_alarm_rows[i].bar, lv_color_hex(0xF76D7E), 0);
    lv_obj_set_style_bg_opa(s_alarm_rows[i].bar, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_alarm_rows[i].bar, 0, 0);
    lv_obj_clear_flag(s_alarm_rows[i].bar, LV_OBJ_FLAG_SCROLLABLE);

    s_alarm_rows[i].code_text[0] = '\0';
    s_alarm_rows[i].code_label   = Display_LvglCreateClipLabel(card, s_alarm_rows[i].code_text, 14, row_y, 78, &lv_font_montserrat_14, lv_color_hex(0xF76D7E));
    s_alarm_rows[i].module_label = Display_LvglCreateClipLabel(card, "", 96, row_y, 118, &DISPLAY_LVGL_FONT_ZH_SMALL, lv_color_hex(0xDCE8F2));
    s_alarm_rows[i].reason_label = Display_LvglCreateClipLabel(card, "", 14, (lv_coord_t)(row_y + 22), 200, &DISPLAY_LVGL_FONT_ZH_SMALL, lv_color_hex(0xDCE8F2));
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
  for (i = 0U; i < DISPLAY_LVGL_STATUS_COUNT; i++) {
    lv_coord_t col = (lv_coord_t)(i % 3U);
    lv_coord_t row = (lv_coord_t)(i / 3U);
    lv_coord_t x   = (lv_coord_t)(34 + (col * 154));
    lv_coord_t y   = (lv_coord_t)(58 + (row * 72));

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
  /* 删除电池/电机电量进度条，行距均匀重排；动力电压/电量改称电机电压/电量；湿度下新增气压。 */
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_BATTERY_VOLTAGE, "\xE4""\xB8""\xBB""\xE6""\x8E""\xA7""\xE7""\x94""\xB5""\xE5""\x8E""\x8B", 22, 48, 154);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_BATTERY_PERCENT, "\xE4""\xB8""\xBB""\xE6""\x8E""\xA7""\xE7""\x94""\xB5""\xE9""\x87""\x8F", 22, 82, 154);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE, "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE7""\x94""\xB5""\xE5""\x8E""\x8B", 22, 116, 154);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_MOTOR_BAT_PERCENT, "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE7""\x94""\xB5""\xE9""\x87""\x8F", 22, 150, 154);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_FLIGHT_TIME_S, "\xE9""\xA3""\x9E""\xE8""\xA1""\x8C""\xE6""\x97""\xB6""\xE9""\x97""\xB4", 22, 184, 154);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_TEMPERATURE, "\xE6""\xB8""\xA9""\xE5""\xBA""\xA6", 22, 218, 154);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_HUMIDITY, "\xE6""\xB9""\xBF""\xE5""\xBA""\xA6", 22, 252, 154);
  /* 气压：标签"气压"中的"气"(U+6C14)需重新生成字体后才显示，数值"<值>帕"可正常显示。 */
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_PRESSURE, "\xE6""\xB0""\x94""\xE5""\x8E""\x8B", 22, 286, 154);
  Display_LvglCreateMessageLogPanel(parent, 540, 252);
}

/**
 * @brief Create aircraft attitude page.
 */
static void Display_LvglCreateAircraftPage(lv_obj_t *parent)
{
  lv_obj_t *card;

  Display_LvglCreateSystemColumn(parent);
  card = Display_LvglCreateCard(parent, 264, DISPLAY_LVGL_BODY_Y, 268, DISPLAY_LVGL_BODY_H, "\xE9""\xA3""\x9E""\xE6""\x9C""\xBA""\xE5""\xA7""\xBF""\xE6""\x80""\x81");
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_ROLL, "\xE6""\xA8""\xAA""\xE6""\xBB""\x9A", 24, 58, 142);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_PITCH, "\xE4""\xBF""\xAF""\xE4""\xBB""\xB0", 24, 100, 142);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_YAW, "\xE5""\x81""\x8F""\xE8""\x88""\xAA", 24, 142, 142);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_LORA_TX_COUNT, "\xE5""\x8F""\x91""\xE9""\x80""\x81""\xE8""\xAE""\xA1""\xE6""\x95""\xB0", 24, 194, 142);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_LORA_RX_COUNT, "\xE6""\x8E""\xA5""\xE6""\x94""\xB6""\xE8""\xAE""\xA1""\xE6""\x95""\xB0", 24, 236, 142);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_LORA_LOSS_RATE, "\xE4""\xB8""\xA2""\xE5""\x8C""\x85""\xE7""\x8E""\x87", 24, 278, 142);
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

    (void)Display_LvglCreateLabel(card, motor_names[i], label_x, 40, &display_lvgl_font_zh_16, lv_color_hex(0xDCE8F2));
    s_motor_pwm_bars[i] = lv_slider_create(card);
    lv_obj_set_size(s_motor_pwm_bars[i], 28, 150);
    lv_obj_set_pos(s_motor_pwm_bars[i], (lv_coord_t)(x + 1), 68);
    lv_slider_set_range(s_motor_pwm_bars[i], 0, 100);
    lv_slider_set_value(s_motor_pwm_bars[i], 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_motor_pwm_bars[i], lv_color_hex(0x263748), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_motor_pwm_bars[i], lv_color_hex(0x1DB7C9), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s_motor_pwm_bars[i], lv_color_hex(0xFFFFFF), LV_PART_KNOB);
    lv_obj_set_style_width(s_motor_pwm_bars[i], 18, LV_PART_KNOB);
    lv_obj_set_style_height(s_motor_pwm_bars[i], 18, LV_PART_KNOB);
    lv_obj_add_event_cb(s_motor_pwm_bars[i], Display_LvglMotorSliderEventCb, LV_EVENT_VALUE_CHANGED, (void *)(uintptr_t)id);
    Display_LvglCreateValueLabel(card, id, (lv_coord_t)(x - 16), 238, 64, &lv_font_montserrat_14);
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
  App_RemoteNodeView_t nodes[DISPLAY_LVGL_REMOTE_ROWS];
  uint8_t count = 0U;
  uint8_t i;
  uint8_t selected_node = 0xFFU;

  card = Display_LvglCreateCard(parent, 70, 96, 660, 292, "\xE9""\x80""\x9A""\xE4""\xBF""\xA1""\xE8""\xBF""\x9E""\xE6""\x8E""\xA5");
  (void)Display_LvglCreateLabel(card, "\xE9""\x80""\x9A""\xE4""\xBF""\xA1""\xE8""\x8A""\x82""\xE7""\x82""\xB9""\xE9""\x80""\x89""\xE6""\x8B""\xA9""\xE5""\x90""\x8E""\xE7""\xBB""\xAD""\xE8""\xBF""\x81""\xE7""\xA7""\xBB", 32, 82, &display_lvgl_font_zh_16, lv_color_hex(0xDCE8F2));
  (void)App_GetSelectedRemoteNode(&selected_node);

  if ((App_CopyRemoteNodeStatuses(nodes, DISPLAY_LVGL_REMOTE_ROWS, &count, s_last_tick_ms) == PX4LITE_OK) && (count != 0U)) {
    for (i = 0U; (i < count) && (i < DISPLAY_LVGL_REMOTE_ROWS); i++) {
      lv_obj_t *btn;
      lv_obj_t *label;
      lv_obj_t *dot;
      lv_color_t dot_color;
      uint8_t selected = (uint8_t)(nodes[i].node_id == selected_node);

      if ((nodes[i].state == APP_REMOTE_NODE_ACTIVE) || (nodes[i].state == APP_REMOTE_NODE_DISCOVERED)) {
        dot_color = lv_palette_main(LV_PALETTE_GREEN);
      } else {
        dot_color = lv_palette_main(LV_PALETTE_AMBER);
      }

      (void)snprintf(s_remote_node_texts[i], sizeof(s_remote_node_texts[i]), "Node %u  RX %lu  Loss %u.%u%%",
                     (unsigned int)nodes[i].node_id,
                     (unsigned long)nodes[i].rx_frame_count,
                     (unsigned int)(nodes[i].rx_loss_rate_x10 / 10U),
                     (unsigned int)(nodes[i].rx_loss_rate_x10 % 10U));
      btn = lv_obj_create(card);
      lv_obj_set_size(btn, 300, 30);
      lv_obj_set_pos(btn, 320, (lv_coord_t)(70 + (i * 36U)));
      lv_obj_set_style_radius(btn, 4, 0);
      lv_obj_set_style_bg_color(btn, selected ? lv_color_hex(0x143747) : lv_color_hex(0x25384A), 0);
      lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
      lv_obj_set_style_border_width(btn, 1, 0);
      lv_obj_set_style_border_color(btn, selected ? lv_color_hex(0x1DB7C9) : lv_color_hex(0x2F4A60), 0);
      lv_obj_set_style_pad_all(btn, 0, 0);
      lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
      lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
      lv_obj_add_event_cb(btn, Display_LvglRemoteNodeEventCb, LV_EVENT_CLICKED, (void *)(uintptr_t)nodes[i].node_id);
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
    (void)Display_LvglCreateLabel(card, "No remote heartbeat", 340, 96, &lv_font_montserrat_14, lv_color_hex(0x7D91A6));
  }

  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_LORA_TX_COUNT, "\xE5""\x8F""\x91""\xE9""\x80""\x81""\xE8""\xAE""\xA1""\xE6""\x95""\xB0", 32, 148, 180);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_LORA_RX_COUNT, "\xE6""\x8E""\xA5""\xE6""\x94""\xB6""\xE8""\xAE""\xA1""\xE6""\x95""\xB0", 32, 188, 180);
  Display_LvglCreateValueRow(card, DISPLAY_HMI_VAR_LORA_LOSS_RATE, "\xE4""\xB8""\xA2""\xE5""\x8C""\x85""\xE7""\x8E""\x87", 32, 228, 180);
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
  }

  Display_LvglApplyCachedValues();
  s_debug_stats.page_rebuild_count++;
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
  s_next_lcd_probe_ms  = now_ms + DISPLAY_LVGL_LCD_PROBE_MS;
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
 * @brief LVGL needs the task to service timers regularly.
 */
uint8_t Display_LvglNeedsRefresh(void)
{
  return (s_lvgl_display_ready != 0U) ? 1U : 0U;
}

/**
 * @brief Service LVGL timers and pending dirty-area flushes.
 */
Display_Result_t Display_LvglRefreshStep(uint32_t now_ms, uint32_t budget_us)
{
  uint32_t elapsed_ms;
  uint32_t start_us;
  uint32_t elapsed_us;

  if (s_lvgl_display_ready == 0U) {
    return DISPLAY_NOT_READY;
  }

  if ((int32_t)(now_ms - s_next_lcd_probe_ms) >= 0) {
    if (LvPortDisp_IsReady() == 0U) {
      s_lvgl_display_ready = 0U;
      return DISPLAY_NOT_READY;
    }
    s_next_lcd_probe_ms = now_ms + DISPLAY_LVGL_LCD_PROBE_MS;
  }

  start_us = (budget_us != 0U) ? Px4Lite_PlatformGetUs() : 0U;

  if (s_page_change_requested != 0U) {
    s_page_change_requested = 0U;
    if (s_requested_page != s_current_lvgl_page) {
      (void)Display_SetHmiPage(s_requested_page);
      if (budget_us != 0U) {
        elapsed_us = Px4Lite_PlatformGetUs() - start_us;
        if (elapsed_us >= budget_us) {
          s_debug_stats.last_refresh_elapsed_us = elapsed_us;
          if (elapsed_us > s_debug_stats.max_refresh_elapsed_us) { s_debug_stats.max_refresh_elapsed_us = elapsed_us; }
          s_debug_stats.budget_busy_count++;
          return DISPLAY_NOT_READY;
        }
      }
    }
  }

  elapsed_ms = now_ms - s_last_tick_ms;
  if (elapsed_ms != 0U) {
    lv_tick_inc(elapsed_ms);
    s_last_tick_ms = now_ms;
  }

  if (budget_us != 0U) {
    elapsed_us = Px4Lite_PlatformGetUs() - start_us;
    if (elapsed_us >= budget_us) {
      s_debug_stats.last_refresh_elapsed_us = elapsed_us;
      if (elapsed_us > s_debug_stats.max_refresh_elapsed_us) { s_debug_stats.max_refresh_elapsed_us = elapsed_us; }
      s_debug_stats.budget_busy_count++;
      return DISPLAY_NOT_READY;
    }
  }

  lv_timer_handler();
  elapsed_us = (budget_us != 0U) ? (Px4Lite_PlatformGetUs() - start_us) : 0U;
  s_debug_stats.last_refresh_elapsed_us = elapsed_us;
  if (elapsed_us > s_debug_stats.max_refresh_elapsed_us) { s_debug_stats.max_refresh_elapsed_us = elapsed_us; }
  if ((budget_us != 0U) && (elapsed_us > budget_us)) {
    s_debug_stats.budget_busy_count++;
    return DISPLAY_NOT_READY;
  }
  return DISPLAY_OK;
}

/**
 * @brief 复制 LVGL 后端刷新预算诊断统计。
 *
 * @param[out] out 输出缓冲区，允许为 NULL。
 */
void Display_LvglGetDebugStats(Display_DebugStats_t *out)
{
  if (out == 0) { return; }
  *out = s_debug_stats;
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
