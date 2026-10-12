/**
 * @file display_lvgl.c
 * @brief Display 层固定容量 LVGL 工作区、两层本机导航及事件观察视图。
 * @details 数据经 display.h / app_data_api.h 输入；不读取 BSP、驱动私有数据或 topic。
 * 启动一次性创建控件和样式。切页只复用平铺控件，禁止清屏销毁/重建和隐式控制操作。
 */
#include "display_lvgl.h"
#include "app_data_api.h"
#include "display_lvgl_font_zh.h"
#include "display_pages.h"
#include "display_gt911.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "px4lite_config.h"
#include "px4lite_faults.h"
#include "px4lite_local_msglog.h"
#include "lvgl.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define HZ_PX_PER_DEG 2.6f
#define HZ_ROLL_SIGN 1.0f
#define HZ_PITCH_SIGN 1.0f
#define HZ_YAW_SIGN 1.0f
#define HZ_YAW_OFFSET_DEG 0.0f
#define DISPLAY_LVGL_VALUE_TEXT_LEN 32U
#define UI_LABEL_COUNT 48U
#define UI_BUTTON_COUNT 16U
#define UI_STATUS_COUNT 12U
#define UI_LOG_CAP 32U
#define UI_HISTORY_ROWS 6U
#define UI_ALARM_CAP 16U
#define UI_REMOTE_CAP 16U
#define UI_TEXT_ROWS 12U
#define UI_ROW_TEXT_LEN 128U
#define UI_COLOR_TEXT 0xDCE8F2U
#define UI_COLOR_MUTED 0x8BA0B4U
#define UI_COLOR_ACCENT 0x28BDCDU
#define UI_COLOR_WARN 0xF2C14EU
#define UI_COLOR_ERROR 0xF76D7EU

/** @brief HMI 数值绑定；静态文本寿命覆盖全部页面。 */
typedef struct {
  lv_obj_t *label; /**< 当前工作区标签，未显示为 NULL。 */
  char text[DISPLAY_LVGL_VALUE_TEXT_LEN]; /**< 带单位格式化值，字节数含终止符。 */
} Display_LvglValueSlot_t;
/** @brief 单一按钮回调的动作编号；切页不重绑 LVGL 事件描述符。 */
typedef enum {
  UI_ACTION_NONE, UI_ACTION_PAGE, UI_ACTION_BACK, UI_ACTION_FREEZE,
  UI_ACTION_HISTORY_OLDER, UI_ACTION_HISTORY_NEWER, UI_ACTION_HISTORY_FILTER,
  UI_ACTION_HISTORY_CLEAR, UI_ACTION_HISTORY_ALL, UI_ACTION_ALARM_NEXT,
  UI_ACTION_DIAG_NEXT, UI_ACTION_REMOTE_NEXT, UI_ACTION_REMOTE_SELECT,
  UI_ACTION_MOTOR_MODE, UI_ACTION_RAMP, UI_ACTION_ESTOP, UI_ACTION_DOWN,
  UI_ACTION_ATTITUDE
} UiAction_t;
/** @brief 固定按钮描述；参数仅由显示任务修改。 */
typedef struct {
  lv_obj_t *obj; /**< 点击对象。 */
  lv_obj_t *label; /**< 静态按钮标题。 */
  UiAction_t action; /**< 当前页面动作。 */
  uint16_t argument; /**< 页面、动作或节点编号。 */
} UiButton_t;
/** @brief 自检网格绑定实际模块状态，电机不推断机械健康。 */
typedef struct {
  Px4Lite_ModuleId_t id; /**< 模块编号。 */
  const char *name; /**< 简短对象名。 */
} UiModule_t;

static const UiModule_t s_modules[UI_STATUS_COUNT] = {
  {PX4LITE_MODULE_GNSS, "GNSS"}, {PX4LITE_MODULE_IMU, "IMU"},
  {PX4LITE_MODULE_BARO, "Baro"}, {PX4LITE_MODULE_BATTERY, "Power"},
  {PX4LITE_MODULE_LORA, "LoRa"}, {PX4LITE_MODULE_5G, "5G UART"},
  {PX4LITE_MODULE_STORAGE, "SD"}, {PX4LITE_MODULE_REMOTE_ID, "RID UART"},
  {PX4LITE_MODULE_DISPLAY, "Display"}, {PX4LITE_MODULE_CONTROL, "Motor PWM"},
  {PX4LITE_MODULE_ALARM, "Alarm"}, {PX4LITE_MODULE_SYSTEM, "System"}
};
static const char * const s_main_titles[6] = {"\xE8""\x87""\xAA""\xE6""\xA3""\x80""", "\xE9""\xA3""\x9E""\xE8""\xA1""\x8C""", "\xE5""\xA7""\xBF""\xE6""\x80""\x81""", "\xE5""\xAE""\x9A""\xE4""\xBD""\x8D""", "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""", "\xE5""\x91""\x8A""\xE8""\xAD""\xA6"""};
static const char * const s_module_names[PX4LITE_MODULE_COUNT] = {
  "GNSS", "IMU", "Baro", "Power", "LoRa", "5G UART", "SD", "RID UART",
  "Display", "Motor PWM", "Alarm", "System", "Estimator", "Business"
};
static Display_LvglValueSlot_t s_value_slots[DISPLAY_HMI_VAR_COUNT];
static uint32_t s_values[DISPLAY_HMI_VAR_COUNT];
static uint8_t s_value_valid[DISPLAY_HMI_VAR_COUNT];
static lv_obj_t *s_screen;
static lv_obj_t *s_labels[UI_LABEL_COUNT];
static UiButton_t s_buttons[UI_BUTTON_COUNT];
static lv_obj_t *s_dots[UI_STATUS_COUNT];
static lv_obj_t *s_panels[3];
static lv_obj_t *s_motor_pwm_bars[4];
static lv_obj_t *s_motor_pulse_labels[4];
static uint16_t s_motor_pulse_us[4];
static uint8_t s_motor_pulse_valid[4];
static char s_motor_pulse_text[4][16];
static uint8_t s_motor_slider_dragging[4];
static uint8_t s_motor_touch_value[4];
static uint8_t s_motor_touch_valid[4];
static uint8_t s_control_update_active;
static lv_obj_t *s_attitude_obj;
static int16_t s_attitude_roll_deg10, s_attitude_pitch_deg10, s_attitude_yaw_deg10;
static lv_obj_t *s_state_labels[UI_STATUS_COUNT];
static char s_state_text[UI_STATUS_COUNT][20];
static lv_obj_t *s_row_labels[UI_TEXT_ROWS];
static char s_row_text[UI_TEXT_ROWS][UI_ROW_TEXT_LEN];
static lv_obj_t *s_alarm_header;
static char s_alarm_header_text[24];
static lv_obj_t *s_page_info;
static char s_page_info_text[64];
static lv_obj_t *s_freeze_label;
static char s_runtime_text[96];
static lv_obj_t *s_runtime_label;
static lv_style_t s_base_style, s_label_style, s_button_style, s_panel_style;
static uint8_t s_label_used, s_button_used;
static uint8_t s_workspace_ready, s_lvgl_core_ready, s_lvgl_display_ready;
static uint8_t s_page_change_requested, s_layout_requested;
static uint8_t s_observation_frozen;
static uint8_t s_alarm_available;
static uint16_t s_alarm_count;
static uint32_t s_alarm_packed[UI_ALARM_CAP];
static App_AlarmSnapshot_t s_alarm_details;
static uint8_t s_alarm_offset, s_diag_offset, s_remote_offset;
static uint8_t s_remote_selected;
static uint8_t s_remote_selection_valid;
static uint8_t s_history_offset, s_history_filter;
static uint8_t s_history_cleared;
static uint32_t s_history_clear_version; /**< 清视图对应生产环追加版本，非 16 位事件序号。 */
static uint32_t s_log_source_version; /**< 与显示日志窗口同次复制的生产环版本。 */
static Display_MessageLogEntry_t s_log_entries[UI_LOG_CAP];
static uint16_t s_log_count;
static uint32_t s_log_version = 0xFFFFFFFFUL;
static App_RemoteNodeView_t s_remote_views[UI_REMOTE_CAP];
static uint8_t s_remote_count;
static Display_HmiPage_t s_current_lvgl_page = DISPLAY_HMI_PAGE_SELF_CHECK;
static Display_HmiPage_t s_requested_page = DISPLAY_HMI_PAGE_SELF_CHECK;
static Display_HmiPage_t s_primary_page = DISPLAY_HMI_PAGE_SELF_CHECK;
static uint32_t s_last_tick_ms, s_next_status_ms, s_next_remote_ms;

static void Display_LvglLayout(void);
static void Display_LvglUpdateLogs(void);
static void Display_LvglUpdateAlarms(void);
static void Display_LvglUpdateModules(void);
static void Display_LvglUpdateRemote(void);

/** @brief 将原 HMI 状态编码保留为可区分的短状态。 */
static const char *Display_LvglStatusText(uint32_t value)
{
  static const char * const names[] = {"N/A", "STARTING", "ONLINE", "OFFLINE", "DEGRADED", "FAILED", "DISABLED"};
  return (value < 7U) ? names[value] : "N/A";
}
/** @brief 格式化 GNSS 解算等级。 */
static const char *Display_LvglGnssFixText(uint32_t value)
{
  switch (value) {
    case 0U:
    case 1U:
      return "\xE6""\x97""\xA0""\xE5""\xAE""\x9A""\xE4""\xBD""\x8D"; /* 无定位 */
    case 2U:
      return "2D\xE5""\xAE""\x9A""\xE4""\xBD""\x8D"; /* 2D定位 */
    case 4U:
      return "DGPS"; /* 差分(字库缺"\xE5""\xB7""\xAE""") */
    case 5U:
      return "RTK-F"; /* RTK 浮动 */
    case 6U:
      return "RTK-X"; /* RTK 固定 */
    case 3U:
    default:
      return "3D\xE5""\xAE""\x9A""\xE4""\xBD""\x8D"; /* 3D定位 */
  }
}

/** @brief 格式化带符号十分位物理量。 */
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

/** @brief 格式化 1e-7 度坐标。 */
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

/** @brief 格式化姿态请求所属状态机的实际进度。 */
static void Display_LvglFormatAttitudePhase(char *text, uint32_t phase)
{
  uint32_t action = s_values[DISPLAY_HMI_VAR_ATTITUDE_ACTION];
  uint32_t reason = s_values[DISPLAY_HMI_VAR_ATTITUDE_REASON];
  uint32_t progress = s_values[DISPLAY_HMI_VAR_ATTITUDE_PROGRESS];

  if (phase == 255U) {
    (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "\xE8\xBF\x9C\xE7\xAB\xAF\xE5\x8F\xAA\xE8\xAF\xBB"); /* 远端只读 */
  } else if (phase == PX4LITE_ATTITUDE_REQUESTED) {
    (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "\xE8\xAF\xB7\xE6\xB1\x82\xE5\xB7\xB2\xE6\x8E\xA5\xE5\x8F\x97"); /* 请求已接受 */
  } else if (phase == PX4LITE_ATTITUDE_WAIT_STILL) {
    (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "\xE8\xAF\xB7\xE6\x94\xBE\xE7\xA8\xB3\xE8\xAE\xBE\xE5\xA4\x87"); /* 请放稳设备 */
  } else if (phase == PX4LITE_ATTITUDE_COLLECTING) {
    (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "\xE9\x87\x87\xE9\x9B\x86\xE4\xB8\xAD %lu%%", (unsigned long)progress); /* 采集中 */
  } else if (phase == PX4LITE_ATTITUDE_SUCCEEDED) {
    if (action == PX4LITE_ATTITUDE_ACTION_BIAS) {
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "\xE9\x9B\xB6\xE5\x81\x8F\xE6\xA0\xA1\xE5\x87\x86\xE6\x88\x90\xE5\x8A\x9F"); /* 零偏校准成功 */
    } else if (action == PX4LITE_ATTITUDE_ACTION_ZERO) {
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "\xE5\xB7\xB2\xE8\xAE\xBE\xE7\x9B\xB8\xE5\xAF\xB9\xE9\x9B\xB6\xE4\xBD\x8D"); /* 已设相对零位 */
    } else {
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "\xE5\xB7\xB2\xE6\x81\xA2\xE5\xA4\x8D\xE5\x8E\x9F\xE5\x9F\xBA\xE5\x87\x86"); /* 已恢复原基准 */
    }
  } else if (phase == PX4LITE_ATTITUDE_FAILED) {
    if (reason == PX4LITE_ATTITUDE_REASON_NO_DATA) {
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "\xE5\xA4\xB1\xE8\xB4\xA5: \xE6\x97\xA0\xE5\xA7\xBF\xE6\x80\x81\xE6\x95\xB0\xE6\x8D\xAE"); /* 失败：无姿态数据 */
    } else {
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "\xE5\xA4\xB1\xE8\xB4\xA5: \xE8\xAE\xBE\xE5\xA4\x87\xE6\x9C\xAA\xE6\x94\xBE\xE7\xA8\xB3"); /* 失败：设备未放稳 */
    }
  } else {
    (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%s",
                   (s_values[DISPLAY_HMI_VAR_ATTITUDE_REFERENCE] != 0U) ?
                   "\xE7\x9B\xB8\xE5\xAF\xB9\xE9\x9B\xB6\xE4\xBD\x8D\xE5\xB7\xB2\xE5\x90\xAF\xE7\x94\xA8" : /* 相对零位已启用 */
                   "\xE5\xBE\x85\xE6\x93\x8D\xE4\xBD\x9C"); /* 待操作 */
  }
}

/** @brief 格式化已有 HMI 值，保留原单位及控制语义。 */
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
  if (s_value_valid[id] == 0U) {
    const char *unavailable = "N/A";
    if ((id >= DISPLAY_HMI_VAR_MOTOR_TARGET_1) && (id <= DISPLAY_HMI_VAR_MOTOR_TARGET_4)) { unavailable = "Set N/A"; }
    if ((id >= DISPLAY_HMI_VAR_MOTOR_PWM_1) && (id <= DISPLAY_HMI_VAR_MOTOR_PWM_4)) { unavailable = "Out N/A"; }
    (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%s", unavailable);
    return;
  }
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
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%lus", (unsigned long)(value / 1000U));
      break;
    case DISPLAY_HMI_VAR_VIEW_NODE_ID:
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "Node %02lu", (unsigned long)value);
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
      /* 0.1A 单位，保留一位小数，单位安。 */
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%lu.%lu""\xE5""\xAE""\x89", (unsigned long)(value / 10U), (unsigned long)(value % 10U));
      break;
    case DISPLAY_HMI_VAR_BATTERY_PERCENT:
    case DISPLAY_HMI_VAR_MOTOR_BAT_PERCENT:
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%lu%%", (unsigned long)value);
      break;
    case DISPLAY_HMI_VAR_MOTOR_TARGET_1:
    case DISPLAY_HMI_VAR_MOTOR_TARGET_2:
    case DISPLAY_HMI_VAR_MOTOR_TARGET_3:
    case DISPLAY_HMI_VAR_MOTOR_TARGET_4:

        (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "\xE8\xAE\xBE%lu%%", (unsigned long)value); /* 设 */
      break;
    case DISPLAY_HMI_VAR_MOTOR_PWM_1:
    case DISPLAY_HMI_VAR_MOTOR_PWM_2:
    case DISPLAY_HMI_VAR_MOTOR_PWM_3:
    case DISPLAY_HMI_VAR_MOTOR_PWM_4:
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "\xE5\x87\xBA%lu%%", (unsigned long)value); /* 出 */
      break;
    case DISPLAY_HMI_VAR_MOTOR_CONTROL_MODE:
      (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "%s",
                     (value == PX4LITE_CONTROL_MODE_DIRECT) ? "\xE7\x9B\xB4\xE6\x8E\xA7" :
                     ((value == PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST) ? "\xE5\xA7\xBF\xE6\x80\x81\xE8\xBE\x85\xE5\x8A\xA9" :
                      "N/A")); /* 直控/姿态辅助/未知 */
      break;
    case DISPLAY_HMI_VAR_MOTOR_OPERATION_STATE:
      switch (value) {
        case PX4LITE_MOTOR_OP_STOPPED: (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "\xE5\x81\x9C\xE6\xAD\xA2"); break;
        case PX4LITE_MOTOR_OP_ESC_PREPARING: (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "ESC\xE5\x87\x86\xE5\xA4\x87\xE4\xB8\xAD"); break;
        case PX4LITE_MOTOR_OP_MANUAL_RUNNING: (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "\xE6\x89\x8B\xE5\x8A\xA8\xE8\xBF\x90\xE8\xA1\x8C"); break;
        case PX4LITE_MOTOR_OP_SYNC_RAMP_UP: (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "\xE5\x90\x8C\xE6\xAD\xA5\xE5\x8D\x87\xE9\x80\x9F"); break;
        case PX4LITE_MOTOR_OP_RAMP_DOWN: (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "\xE7\xBC\x93\xE9\x99\x8D\xE5\x81\x9C\xE6\x9C\xBA"); break;
        case PX4LITE_MOTOR_OP_INHIBITED: (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "\xE7\x94\xB5\xE6\xBA\x90\xE9\x97\xAD\xE9\x94\x81"); break;
        default: (void)snprintf(text, DISPLAY_LVGL_VALUE_TEXT_LEN, "N/A"); break;
      }
      break;
    case DISPLAY_HMI_VAR_ATTITUDE_PHASE:
      Display_LvglFormatAttitudePhase(text, value);
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

/** @brief 限制百分数到滑块范围。 */
static int32_t Display_LvglClampPercent(uint32_t value)
{
  return (value > 100U) ? 100 : (int32_t)value;
}

/** @brief 仅提交实际触摸变化，不回灌显示值；保留松手补偿。 */
static void Display_LvglMotorSliderEventCb(lv_event_t *event)
{
  lv_event_code_t code;
  lv_obj_t *slider;
  Display_HmiVariableId_t id;
  Display_HmiVariableId_t target_id;
  uint8_t motor_index;
  int32_t value;

  code   = lv_event_get_code(event);
  slider = lv_event_get_target(event);
  id     = (Display_HmiVariableId_t)(uintptr_t)lv_event_get_user_data(event);
  if ((id < DISPLAY_HMI_VAR_MOTOR_PWM_1) || (id > DISPLAY_HMI_VAR_MOTOR_PWM_4)) { return; }
  motor_index = (uint8_t)((uint16_t)id - (uint16_t)DISPLAY_HMI_VAR_MOTOR_PWM_1);
  target_id = (Display_HmiVariableId_t)((uint16_t)DISPLAY_HMI_VAR_MOTOR_TARGET_1 + motor_index);


  if (s_current_lvgl_page != DISPLAY_HMI_PAGE_MOTOR) { return; }
  if (s_value_valid[target_id] == 0U) {
    s_motor_slider_dragging[motor_index] = 0U;
    s_motor_touch_valid[motor_index] = 0U;
    return;
  }

  if (code == LV_EVENT_PRESSED) {
    s_motor_slider_dragging[motor_index]   = 1U;
    s_motor_touch_valid[motor_index]       = 0U;
    return;
  }
  if ((code == LV_EVENT_RELEASED) || (code == LV_EVENT_PRESS_LOST)) {
    s_motor_slider_dragging[motor_index] = 0U;
  } else if (code != LV_EVENT_VALUE_CHANGED) {
    return;
  }
  if (s_control_update_active != 0U) { return; }

  if (code == LV_EVENT_VALUE_CHANGED) {
    /* 旋钮被手指移动了：本次位置才是用户的新目标。 */
    value = lv_slider_get_value(slider);
    if (value < 0) { value = 0; }
    if (value > 100) { value = 100; }
    s_motor_touch_value[motor_index] = (uint8_t)value;
    s_motor_touch_valid[motor_index] = 1U;
  } else {
    /* 松手：本次按压期间手指没动过旋钮就什么都不提交，避免把显示值当成新目标。 */
    if (s_motor_touch_valid[motor_index] == 0U) { return; }
    value = (int32_t)s_motor_touch_value[motor_index];
  }

  if (Display_RequestMotorThrottle(id, (uint16_t)value) != DISPLAY_OK) {
    if ((target_id < DISPLAY_HMI_VAR_COUNT) && (s_value_valid[target_id] != 0U)) {
      s_control_update_active = 1U;
      lv_slider_set_value(slider, Display_LvglClampPercent(s_values[target_id]), LV_ANIM_OFF);
      s_control_update_active = 0U;
    }
    return;
  }

  /* 请求已进入所属控制服务，实际目标以随后发布的快照为准。 */
}

/** @brief 姿态仪直接绘制；只使用显示缓存，不启动采集或控制。 */
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

/** @brief 给固定对象预置所有会修改的局部属性；运行期只更新已有属性。 */
static void UiInitGeometry(lv_obj_t *obj)
{
  lv_obj_set_pos(obj, 0, 0);
  lv_obj_set_size(obj, 10, 10);
  lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
}

/** @brief 固定标签写入；静态缓冲相同时也允许显式重算尺寸。 */
static void UiText(lv_obj_t *obj, const char *text)
{
  if (obj != 0) { lv_label_set_text_static(obj, text); }
}

/** @brief 从启动池取标签；全部标签平铺于同一父对象，不重建父子关系。 */
static lv_obj_t *UiLabel(const char *text, lv_coord_t x, lv_coord_t y, lv_coord_t w,
                         const lv_font_t *font, uint32_t color)
{
  lv_obj_t *obj;
  if (s_label_used >= UI_LABEL_COUNT) { return 0; }
  obj = s_labels[s_label_used++];
  lv_obj_set_pos(obj, x, y);
  lv_obj_set_size(obj, w, 22);
  lv_obj_set_style_text_font(obj, font, 0);
  lv_obj_set_style_text_color(obj, lv_color_hex(color), 0);
  UiText(obj, text);
  lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);
  return obj;
}

/** @brief 绑定一个 HMI 数值，未知值可见为 N/A。 */
static void UiValue(Display_HmiVariableId_t id, lv_coord_t x, lv_coord_t y, lv_coord_t w)
{
  Display_LvglFormatValue(id, s_values[id]);
  s_value_slots[id].label = UiLabel(s_value_slots[id].text, x, y, w, &display_lvgl_font_zh_16, UI_COLOR_TEXT);
}

/** @brief 创建无副作用的数据行；数值与说明使用独立标签。 */
static void UiValueRow(Display_HmiVariableId_t id, const char *name, lv_coord_t x, lv_coord_t y, lv_coord_t gap)
{
  (void)UiLabel(name, x, y, gap - 4, &display_lvgl_font_zh_16, UI_COLOR_MUTED);
  UiValue(id, x + gap, y, 180);
}

/** @brief 重排共享背景，不创建卡片标题或新对象。 */
static void UiPanel(uint8_t index, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h)
{
  lv_obj_set_pos(s_panels[index], x, y);
  lv_obj_set_size(s_panels[index], w, h);
  lv_obj_clear_flag(s_panels[index], LV_OBJ_FLAG_HIDDEN);
}

/** @brief 从固定按钮池取一个动作；回调终生不变。 */
static UiButton_t *UiButton(const char *title, lv_coord_t x, lv_coord_t y, lv_coord_t w,
                            lv_coord_t h, UiAction_t action, uint16_t argument, uint32_t color)
{
  UiButton_t *button;
  if (s_button_used >= UI_BUTTON_COUNT) { return 0; }
  button = &s_buttons[s_button_used++];
  button->action = action;
  button->argument = argument;
  lv_obj_set_pos(button->obj, x, y);
  lv_obj_set_size(button->obj, w, h);
  lv_obj_set_style_bg_color(button->obj, lv_color_hex(color), 0);
  UiText(button->label, title);
  lv_obj_clear_flag(button->obj, LV_OBJ_FLAG_HIDDEN);
  return button;
}

/** @brief 返回真实模块状态文字，不把未知值作为故障或在线。 */
static const char *UiModuleState(Px4Lite_State_t state)
{
  switch (state) {
    case PX4LITE_STATE_STARTING: return "STARTING";
    case PX4LITE_STATE_ONLINE: return "ONLINE";
    case PX4LITE_STATE_DEGRADED: return "DEGRADED";
    case PX4LITE_STATE_OFFLINE: return "OFFLINE";
    case PX4LITE_STATE_FAILED: return "FAILED";
    case PX4LITE_STATE_DISABLED: return "DISABLED";
    default: return "N/A";
  }
}

/** @brief 严重度文字与颜色双通道表达，ERROR 包含更高严重度。 */
static const char *UiSeverity(uint8_t severity)
{
  return (severity >= PX4LITE_SEVERITY_ERROR) ? "ERROR" :
         ((severity == PX4LITE_SEVERITY_WARNING) ? "WARN" : "INFO");
}

/** @brief 将严重度映射到日志颜色。 */
static uint32_t UiSeverityColor(uint8_t severity)
{
  return (severity >= PX4LITE_SEVERITY_ERROR) ? UI_COLOR_ERROR :
         ((severity == PX4LITE_SEVERITY_WARNING) ? UI_COLOR_WARN : UI_COLOR_TEXT);
}

/** @brief 提交页面请求；二级页面归属最后一个主入口。 */
static void UiRequestPage(Display_HmiPage_t page)
{
  s_requested_page = page;
  s_page_change_requested = 1U;
}

/** @brief 固定按钮统一分发；导航、冻结和日志浏览不会产生控制请求。 */
static void UiButtonEvent(lv_event_t *event)
{
  UiButton_t *button = (UiButton_t *)lv_event_get_user_data(event);
  lv_event_code_t code = lv_event_get_code(event);
  if (button == 0) { return; }
  if (button->action == UI_ACTION_ESTOP) {
    if (code == LV_EVENT_PRESSED) { (void)Display_RequestMotorEmergencyStop(); }
    return;
  }
  if (code != LV_EVENT_CLICKED) { return; }
  switch (button->action) {
    case UI_ACTION_PAGE: UiRequestPage((Display_HmiPage_t)button->argument); break;
    case UI_ACTION_BACK: UiRequestPage(s_primary_page); break;
    case UI_ACTION_FREEZE:
      s_observation_frozen = (s_observation_frozen == 0U) ? 1U : 0U;
      UiText(button->label, s_observation_frozen ? "Resume" : "Freeze");
      UiText(s_freeze_label, s_observation_frozen ? "FROZEN" : "local");
      break;
    case UI_ACTION_HISTORY_OLDER:
      if ((uint16_t)s_history_offset + UI_HISTORY_ROWS < UI_LOG_CAP) { s_history_offset += UI_HISTORY_ROWS; }
      Display_LvglUpdateLogs(); break;
    case UI_ACTION_HISTORY_NEWER:
      s_history_offset = (s_history_offset >= UI_HISTORY_ROWS) ? (uint8_t)(s_history_offset - UI_HISTORY_ROWS) : 0U;
      Display_LvglUpdateLogs(); break;
    case UI_ACTION_HISTORY_FILTER:
      s_history_filter = (uint8_t)((s_history_filter + 1U) % 3U);
      s_history_offset = 0U;
      UiText(button->label, (s_history_filter == 0U) ? "All" : ((s_history_filter == 1U) ? "WARN+" : "ERROR"));
      Display_LvglUpdateLogs(); break;
    case UI_ACTION_HISTORY_CLEAR:
      s_log_count = Display_PagesCopyLogMessages(s_log_entries, UI_LOG_CAP, 0, 0);
      s_history_clear_version = s_log_source_version;
      s_history_cleared = 1U; s_history_offset = 0U; Display_LvglUpdateLogs(); break;
    case UI_ACTION_HISTORY_ALL:
      s_history_cleared = 0U; s_history_offset = 0U; Display_LvglUpdateLogs(); break;
    case UI_ACTION_ALARM_NEXT:
      s_alarm_offset = ((uint16_t)s_alarm_offset + 5U < s_alarm_count) ? (uint8_t)(s_alarm_offset + 5U) : 0U;
      Display_LvglUpdateAlarms(); break;
    case UI_ACTION_DIAG_NEXT:
      s_diag_offset = ((uint16_t)s_diag_offset + 6U < PX4LITE_MODULE_COUNT) ? (uint8_t)(s_diag_offset + 6U) : 0U;
      Display_LvglUpdateModules(); break;
    case UI_ACTION_REMOTE_NEXT:
      s_remote_offset = ((uint16_t)s_remote_offset + 4U < s_remote_count) ? (uint8_t)(s_remote_offset + 4U) : 0U;
      s_layout_requested = 1U; break;
    case UI_ACTION_REMOTE_SELECT:
      s_remote_selected = (uint8_t)button->argument;
      s_remote_selection_valid = 1U;
      Display_LvglUpdateRemote(); break;
    case UI_ACTION_MOTOR_MODE:
      if (s_value_valid[DISPLAY_HMI_VAR_MOTOR_CONTROL_MODE]) {
        (void)Display_RequestMotorControlMode((s_values[DISPLAY_HMI_VAR_MOTOR_CONTROL_MODE] == PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST) ? PX4LITE_CONTROL_MODE_DIRECT : PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST);
      }
      break;
    case UI_ACTION_RAMP: (void)Display_RequestMotorAutoTakeoff(); break;
    case UI_ACTION_DOWN: (void)Display_RequestMotorAutoLanding(); break;
    case UI_ACTION_ATTITUDE: (void)Display_RequestAttitudeAction((Px4Lite_AttitudeAction_t)button->argument); break;
    default: break;
  }
}

/** @brief 启动时预分配唯一对象池及所有样式属性；保持 40 KiB LVGL 堆配置。 */
static void Display_LvglInitWorkspace(void)
{
  uint16_t i;
  if (s_workspace_ready != 0U) { return; }
  lv_disp_set_theme(lv_disp_get_default(), 0);
  lv_style_init(&s_base_style);
  lv_style_set_pad_all(&s_base_style, 0);
  lv_style_set_border_width(&s_base_style, 0);
  lv_style_set_radius(&s_base_style, 0);
  lv_style_set_bg_opa(&s_base_style, LV_OPA_TRANSP);
  lv_style_set_anim_time(&s_base_style, 0);
  lv_style_init(&s_label_style);
  lv_style_set_text_font(&s_label_style, &display_lvgl_font_zh_16);
  lv_style_set_text_color(&s_label_style, lv_color_hex(UI_COLOR_TEXT));
  lv_style_init(&s_button_style);
  lv_style_set_radius(&s_button_style, 5);
  lv_style_set_bg_opa(&s_button_style, LV_OPA_COVER);
  lv_style_set_border_width(&s_button_style, 1);
  lv_style_set_border_color(&s_button_style, lv_color_hex(0x355269));
  lv_style_init(&s_panel_style);
  lv_style_set_bg_color(&s_panel_style, lv_color_hex(0x12202D));
  lv_style_set_bg_opa(&s_panel_style, LV_OPA_COVER);
  lv_style_set_radius(&s_panel_style, 6);
  s_screen = lv_obj_create(0);
  lv_obj_remove_style_all(s_screen);
  lv_obj_add_style(s_screen, &s_base_style, 0);
  lv_obj_set_size(s_screen, 800, 480);
  lv_obj_set_style_bg_color(s_screen, lv_color_hex(0x08111A), 0);
  lv_obj_set_style_bg_opa(s_screen, LV_OPA_COVER, 0);
  lv_obj_clear_flag(s_screen, LV_OBJ_FLAG_SCROLLABLE);
  for (i = 0U; i < 3U; i++) {
    s_panels[i] = lv_obj_create(s_screen);
    lv_obj_remove_style_all(s_panels[i]);
    lv_obj_add_style(s_panels[i], &s_base_style, 0);
    lv_obj_add_style(s_panels[i], &s_panel_style, 0);
    UiInitGeometry(s_panels[i]);
    lv_obj_clear_flag(s_panels[i], LV_OBJ_FLAG_CLICKABLE);
  }
  for (i = 0U; i < UI_LABEL_COUNT; i++) {
    s_labels[i] = lv_label_create(s_screen);
    lv_obj_remove_style_all(s_labels[i]);
    lv_obj_add_style(s_labels[i], &s_base_style, 0);
    lv_obj_add_style(s_labels[i], &s_label_style, 0);
    UiInitGeometry(s_labels[i]);
    lv_obj_set_style_text_font(s_labels[i], &display_lvgl_font_zh_16, 0);
    lv_obj_set_style_text_color(s_labels[i], lv_color_hex(UI_COLOR_TEXT), 0);
    lv_label_set_long_mode(s_labels[i], LV_LABEL_LONG_CLIP);
    lv_obj_clear_flag(s_labels[i], LV_OBJ_FLAG_CLICKABLE);
    UiText(s_labels[i], "");
  }
  for (i = 0U; i < UI_BUTTON_COUNT; i++) {
    UiButton_t *button = &s_buttons[i];
    button->obj = lv_obj_create(s_screen);
    lv_obj_remove_style_all(button->obj);
    lv_obj_add_style(button->obj, &s_base_style, 0);
    lv_obj_add_style(button->obj, &s_button_style, 0);
    UiInitGeometry(button->obj);
    lv_obj_set_style_bg_color(button->obj, lv_color_hex(0x173749), 0);
    lv_obj_add_flag(button->obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(button->obj, UiButtonEvent, LV_EVENT_ALL, button);
    button->label = lv_label_create(button->obj);
    lv_obj_remove_style_all(button->label);
    lv_obj_add_style(button->label, &s_label_style, 0);
    lv_label_set_long_mode(button->label, LV_LABEL_LONG_CLIP);
    UiText(button->label, "");
    lv_obj_clear_flag(button->label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_align(button->label, LV_ALIGN_CENTER, 0, 0);
  }
  for (i = 0U; i < UI_STATUS_COUNT; i++) {
    s_dots[i] = lv_obj_create(s_screen);
    lv_obj_remove_style_all(s_dots[i]);
    lv_obj_add_style(s_dots[i], &s_base_style, 0);
    UiInitGeometry(s_dots[i]);
    lv_obj_set_style_radius(s_dots[i], LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(s_dots[i], LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(s_dots[i], lv_color_hex(UI_COLOR_MUTED), 0);
    lv_obj_clear_flag(s_dots[i], LV_OBJ_FLAG_CLICKABLE);
  }
  for (i = 0U; i < 4U; i++) {
    s_motor_pwm_bars[i] = lv_slider_create(s_screen);
    lv_obj_remove_style_all(s_motor_pwm_bars[i]);
    lv_obj_add_style(s_motor_pwm_bars[i], &s_base_style, LV_PART_MAIN);
    UiInitGeometry(s_motor_pwm_bars[i]);
    lv_slider_set_range(s_motor_pwm_bars[i], 0, 100);
    lv_obj_set_style_bg_color(s_motor_pwm_bars[i], lv_color_hex(0x2B4052), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_motor_pwm_bars[i], LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_motor_pwm_bars[i], lv_color_hex(UI_COLOR_ACCENT), LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(s_motor_pwm_bars[i], LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s_motor_pwm_bars[i], lv_color_hex(0xFFFFFF), LV_PART_KNOB);
    lv_obj_set_style_bg_opa(s_motor_pwm_bars[i], LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_pad_all(s_motor_pwm_bars[i], 8, LV_PART_KNOB);
    lv_obj_set_style_radius(s_motor_pwm_bars[i], 6, LV_PART_KNOB);
    lv_obj_add_event_cb(s_motor_pwm_bars[i], Display_LvglMotorSliderEventCb, LV_EVENT_ALL,
                       (void *)(uintptr_t)(DISPLAY_HMI_VAR_MOTOR_PWM_1 + i));
  }
  s_attitude_obj = lv_obj_create(s_screen);
  lv_obj_remove_style_all(s_attitude_obj);
  lv_obj_add_style(s_attitude_obj, &s_base_style, 0);
  UiInitGeometry(s_attitude_obj);
  lv_obj_clear_flag(s_attitude_obj, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(s_attitude_obj, Display_LvglHorizonDrawCb, LV_EVENT_DRAW_POST, 0);
  lv_scr_load(s_screen);
  s_workspace_ready = 1U;
}

/** @brief 隐藏并解除上一页面所有数据绑定，不销毁对象或改变控制状态。 */
static void Display_LvglResetWorkspace(void)
{
  uint16_t i;
  for (i = 0U; i < UI_LABEL_COUNT; i++) { lv_obj_add_flag(s_labels[i], LV_OBJ_FLAG_HIDDEN); }
  for (i = 0U; i < UI_BUTTON_COUNT; i++) {
    s_buttons[i].action = UI_ACTION_NONE;
    lv_obj_add_flag(s_buttons[i].obj, LV_OBJ_FLAG_HIDDEN);
  }
  for (i = 0U; i < UI_STATUS_COUNT; i++) {
    lv_obj_add_flag(s_dots[i], LV_OBJ_FLAG_HIDDEN); s_state_labels[i] = 0;
  }
  for (i = 0U; i < 3U; i++) { lv_obj_add_flag(s_panels[i], LV_OBJ_FLAG_HIDDEN); }
  for (i = 0U; i < 4U; i++) {
    lv_obj_add_flag(s_motor_pwm_bars[i], LV_OBJ_FLAG_HIDDEN);
    s_motor_slider_dragging[i] = 0U; s_motor_touch_valid[i] = 0U;
    s_motor_pulse_labels[i] = 0;
  }
  lv_obj_add_flag(s_attitude_obj, LV_OBJ_FLAG_HIDDEN);
  for (i = 0U; i < DISPLAY_HMI_VAR_COUNT; i++) { s_value_slots[i].label = 0; }
  for (i = 0U; i < UI_TEXT_ROWS; i++) { s_row_labels[i] = 0; }
  s_alarm_header = 0; s_page_info = 0; s_runtime_label = 0; s_freeze_label = 0;
  s_label_used = 0U; s_button_used = 0U;
}

/** @brief 绘制公共页头和六个主入口，二级页面始终提供所属主入口返回。 */
static void Display_LvglLayoutChrome(void)
{
  uint8_t i;
  const char *title;
  uint8_t secondary = (s_current_lvgl_page >= DISPLAY_HMI_PAGE_HIDDEN) ? 1U : 0U;
  if (s_current_lvgl_page == DISPLAY_HMI_PAGE_HIDDEN) { title = "Comm"; }
  else if (s_current_lvgl_page == DISPLAY_HMI_PAGE_DIAGNOSTICS) { title = "Diagnostics"; }
  else if (s_current_lvgl_page == DISPLAY_HMI_PAGE_EVENT_HISTORY) { title = "Event history"; }
  else { title = s_main_titles[s_current_lvgl_page - DISPLAY_HMI_PAGE_SELF_CHECK]; }
  (void)UiLabel(title, 12, 9, 160, &display_lvgl_font_zh_16, UI_COLOR_TEXT);
  UiValue(DISPLAY_HMI_VAR_VIEW_NODE_ID, 180, 8, 110);
  (void)UiLabel("Uptime", 300, 8, 66, &lv_font_montserrat_14, UI_COLOR_MUTED);
  UiValue(DISPLAY_HMI_VAR_FLIGHT_TIME_S, 370, 7, 120);
  s_freeze_label = UiLabel((s_observation_frozen &&
    ((s_current_lvgl_page == DISPLAY_HMI_PAGE_FLIGHT) || (s_current_lvgl_page == DISPLAY_HMI_PAGE_AIRCRAFT) ||
     (s_current_lvgl_page == DISPLAY_HMI_PAGE_DATA))) ? "FROZEN" : "local", 180, 33, 115, &lv_font_montserrat_14, UI_COLOR_ACCENT);
  s_alarm_header = UiLabel("Alarms N/A", 300, 33, 185, &lv_font_montserrat_14, UI_COLOR_WARN);
  (void)UiButton("\xE5""\x91""\x8A""\xE8""\xAD""\xA6""", 500, 5, 76, 48, UI_ACTION_PAGE, DISPLAY_HMI_PAGE_ALARM, 0x4B2634);
  (void)UiButton("Comm", 584, 5, 82, 48, UI_ACTION_PAGE, DISPLAY_HMI_PAGE_HIDDEN, 0x173749);
  if ((s_current_lvgl_page == DISPLAY_HMI_PAGE_FLIGHT) || (s_current_lvgl_page == DISPLAY_HMI_PAGE_AIRCRAFT) ||
      (s_current_lvgl_page == DISPLAY_HMI_PAGE_DATA)) {
    (void)UiButton(s_observation_frozen ? "Resume" : "Freeze", 676, 5, 112, 48, UI_ACTION_FREEZE, 0, 0x173749);
  } else if (secondary != 0U) {
    (void)UiButton("\xE8""\xBF""\x94""\xE5""\x9B""\x9E""", 676, 5, 112, 48, UI_ACTION_BACK, 0, 0x173749);
  }
  for (i = 0U; i < 6U; i++) {
    Display_HmiPage_t page = (Display_HmiPage_t)(DISPLAY_HMI_PAGE_SELF_CHECK + i);
    (void)UiButton(s_main_titles[i], (lv_coord_t)(8 + i * 132), 430, 124, 44,
                   UI_ACTION_PAGE, (uint16_t)page, (page == s_primary_page) ? 0x186577 : 0x142A3B);
  }
}

/** @brief 主页面最近三条事件摘要，点击统一历史入口；日志始终保持运行。 */
static void Display_LvglLayoutSummary(void)
{
  uint8_t i;
  UiPanel(2U, 8, 342, 784, 82);
  (void)UiLabel("Event Log", 18, 352, 100, &lv_font_montserrat_14, UI_COLOR_ACCENT);
  (void)UiLabel("Uptime", 126, 344, 80, &lv_font_montserrat_12, UI_COLOR_MUTED);
  for (i = 0U; i < 3U; i++) {
    s_row_labels[9U + i] = UiLabel("", 126, (lv_coord_t)(360 + i * 20), 554, &lv_font_montserrat_12, UI_COLOR_TEXT);
  }
  (void)UiButton("History", 684, 358, 98, 52, UI_ACTION_PAGE, DISPLAY_HMI_PAGE_EVENT_HISTORY, 0x173749);
}

/** @brief 自检网格与活动告警摘要分别表达当前状态及未解除故障。 */
static void Display_LvglLayoutSelfCheck(void)
{
  uint8_t i;
  UiPanel(0U, 8, 62, 490, 272);
  UiPanel(1U, 506, 62, 286, 272);
  (void)UiLabel("Checks: GNSS IMU Baro LoRa SD", 18, 69, 470, &lv_font_montserrat_14, UI_COLOR_MUTED);
  for (i = 0U; i < UI_STATUS_COUNT; i++) {
    lv_coord_t x = (lv_coord_t)(22 + (i % 3U) * 158);
    lv_coord_t y = (lv_coord_t)(103 + (i / 3U) * 47);
    lv_obj_set_pos(s_dots[i], x, y + 3); lv_obj_set_size(s_dots[i], 12, 12);
    lv_obj_clear_flag(s_dots[i], LV_OBJ_FLAG_HIDDEN);
    (void)UiLabel(s_modules[i].name, x + 18, y, 133, &lv_font_montserrat_14, UI_COLOR_TEXT);
    s_state_labels[i] = UiLabel("N/A", x + 18, y + 19, 133, &lv_font_montserrat_12, UI_COLOR_MUTED);
  }
  (void)UiButton("Diagnostics", 320, 293, 160, 34, UI_ACTION_PAGE, DISPLAY_HMI_PAGE_DIAGNOSTICS, 0x173749);
  (void)UiLabel("Active Alarms", 520, 76, 250, &lv_font_montserrat_16, UI_COLOR_WARN);
  for (i = 0U; i < 4U; i++) {
    s_row_labels[i] = UiLabel("", 520, (lv_coord_t)(111 + i * 44), 260, &display_lvgl_font_zh_16, UI_COLOR_TEXT);
    lv_obj_set_height(s_row_labels[i], 40);
    lv_label_set_long_mode(s_row_labels[i], LV_LABEL_LONG_WRAP);
  }
  (void)UiButton("\xE5""\x91""\x8A""\xE8""\xAD""\xA6""", 670, 290, 106, 38, UI_ACTION_PAGE, DISPLAY_HMI_PAGE_ALARM, 0x4B2634);
}

/** @brief 电源与环境按两列展开。 */
static void Display_LvglLayoutFlight(void)
{
  UiPanel(0U, 8, 62, 784, 272);
  UiValueRow(DISPLAY_HMI_VAR_BATTERY_VOLTAGE, "\xE4""\xB8""\xBB""\xE6""\x8E""\xA7""\xE7""\x94""\xB5""\xE5""\x8E""\x8B""", 28, 86, 140);
  UiValueRow(DISPLAY_HMI_VAR_BATTERY_CURRENT, "\xE4""\xB8""\xBB""\xE6""\x8E""\xA7""\xE7""\x94""\xB5""\xE6""\xB5""\x81""", 28, 130, 140);
  UiValueRow(DISPLAY_HMI_VAR_BATTERY_PERCENT, "\xE4""\xB8""\xBB""\xE6""\x8E""\xA7""\xE7""\x94""\xB5""\xE9""\x87""\x8F""", 28, 174, 140);
  UiValueRow(DISPLAY_HMI_VAR_TEMPERATURE, "\xE6""\xB8""\xA9""\xE5""\xBA""\xA6""", 28, 218, 140);
  UiValueRow(DISPLAY_HMI_VAR_HUMIDITY, "\xE6""\xB9""\xBF""\xE5""\xBA""\xA6""", 28, 262, 140);
  UiValueRow(DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE, "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE7""\x94""\xB5""\xE5""\x8E""\x8B""", 414, 86, 140);
  UiValueRow(DISPLAY_HMI_VAR_MOTOR_BAT_CURRENT, "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE7""\x94""\xB5""\xE6""\xB5""\x81""", 414, 130, 140);
  UiValueRow(DISPLAY_HMI_VAR_MOTOR_BAT_PERCENT, "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE7""\x94""\xB5""\xE9""\x87""\x8F""", 414, 174, 140);
  UiValueRow(DISPLAY_HMI_VAR_PRESSURE, "\xE6""\xB0""\x94""\xE5""\x8E""\x8B""", 414, 218, 140);
}

/** @brief 姿态仪与操作各用独立区域，校准状态不随观察冻结。 */
static void Display_LvglLayoutAttitude(void)
{
  UiPanel(0U, 8, 62, 784, 272);
  lv_obj_set_pos(s_attitude_obj, 26, 64); lv_obj_set_size(s_attitude_obj, 258, 258);
  if (s_value_valid[DISPLAY_HMI_VAR_ROLL] && s_value_valid[DISPLAY_HMI_VAR_PITCH] && s_value_valid[DISPLAY_HMI_VAR_YAW]) {
    lv_obj_clear_flag(s_attitude_obj, LV_OBJ_FLAG_HIDDEN);
  }
  (void)UiLabel("local attitude", 312, 78, 200, &lv_font_montserrat_14, UI_COLOR_MUTED);
  UiValueRow(DISPLAY_HMI_VAR_ROLL, "\xE6""\xA8""\xAA""\xE6""\xBB""\x9A""", 314, 116, 118);
  UiValueRow(DISPLAY_HMI_VAR_PITCH, "\xE4""\xBF""\xAF""\xE4""\xBB""\xB0""", 314, 152, 118);
  UiValueRow(DISPLAY_HMI_VAR_YAW, "\xE7""\x9B""\xB8""\xE5""\xAF""\xB9""\xE5""\x81""\x8F""\xE8""\x88""\xAA""", 314, 188, 118);
  UiValue(DISPLAY_HMI_VAR_ATTITUDE_PHASE, 314, 234, 460);
  (void)UiButton("\xE9""\x9B""\xB6""\xE5""\x81""\x8F""\xE6""\xA0""\xA1""\xE5""\x87""\x86""", 310, 275, 145, 48, UI_ACTION_ATTITUDE, PX4LITE_ATTITUDE_ACTION_BIAS, 0x173749);
  (void)UiButton("\xE7""\x9B""\xB8""\xE5""\xAF""\xB9""\xE9""\x9B""\xB6""\xE4""\xBD""\x8D""", 470, 275, 145, 48, UI_ACTION_ATTITUDE, PX4LITE_ATTITUDE_ACTION_ZERO, 0x173749);
  (void)UiButton("\xE6""\x81""\xA2""\xE5""\xA4""\x8D""\xE5""\x9F""\xBA""\xE5""\x87""\x86""", 630, 275, 145, 48, UI_ACTION_ATTITUDE, PX4LITE_ATTITUDE_ACTION_RESTORE, 0x173749);
}

/** @brief 定位数据保留单位、解算等级与未知状态。 */
static void Display_LvglLayoutGnss(void)
{
  UiPanel(0U, 8, 62, 784, 272);
  UiValueRow(DISPLAY_HMI_VAR_GNSS_FIX, "\xE5""\xAE""\x9A""\xE4""\xBD""\x8D""\xE7""\x8A""\xB6""\xE6""\x80""\x81""", 28, 84, 132);
  UiValueRow(DISPLAY_HMI_VAR_GNSS_SAT_COUNT, "\xE5""\x8D""\xAB""\xE6""\x98""\x9F""\xE6""\x95""\xB0""", 28, 132, 132);
  UiValueRow(DISPLAY_HMI_VAR_GNSS_HDOP, "HDOP", 28, 180, 132);
  UiValueRow(DISPLAY_HMI_VAR_GNSS_SPEED, "\xE9""\x80""\x9F""\xE5""\xBA""\xA6""", 28, 228, 132);
  UiValueRow(DISPLAY_HMI_VAR_LATITUDE, "\xE7""\xBA""\xAC""\xE5""\xBA""\xA6""", 414, 84, 108);
  UiValueRow(DISPLAY_HMI_VAR_LONGITUDE, "\xE7""\xBB""\x8F""\xE5""\xBA""\xA6""", 414, 132, 108);
  UiValueRow(DISPLAY_HMI_VAR_ALTITUDE, "\xE9""\xAB""\x98""\xE5""\xBA""\xA6""", 414, 180, 108);
  UiValueRow(DISPLAY_HMI_VAR_GNSS_TIME, "\xE6""\x97""\xB6""\xE9""\x97""\xB4""", 414, 228, 108);
}

/** @brief 四路滑块宽度 30、高度 128 像素；实际输出和急停永远为本机实时值。 */
static void Display_LvglLayoutMotor(void)
{
  uint8_t i;
  UiButton_t *mode;
  static const char * const names[4] = {"\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE4""\xB8""\x80""", "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE4""\xBA""\x8C""", "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE4""\xB8""\x89""", "\xE7""\x94""\xB5""\xE6""\x9C""\xBA""\xE5""\x9B""\x9B"""};
  UiPanel(0U, 8, 62, 784, 272);
  mode = UiButton("", 548, 76, 228, 42, UI_ACTION_MOTOR_MODE, 0, 0x173749);
  s_value_slots[DISPLAY_HMI_VAR_MOTOR_CONTROL_MODE].label = mode->label;
  Display_LvglFormatValue(DISPLAY_HMI_VAR_MOTOR_CONTROL_MODE, s_values[DISPLAY_HMI_VAR_MOTOR_CONTROL_MODE]);
  UiText(mode->label, s_value_slots[DISPLAY_HMI_VAR_MOTOR_CONTROL_MODE].text);
  UiValue(DISPLAY_HMI_VAR_MOTOR_OPERATION_STATE, 548, 126, 228);
  for (i = 0U; i < 4U; i++) {
    lv_coord_t x = (lv_coord_t)(46 + i * 126);
    Display_HmiVariableId_t target = (Display_HmiVariableId_t)(DISPLAY_HMI_VAR_MOTOR_TARGET_1 + i);
    (void)UiLabel(names[i], x - 12, 75, 112, &display_lvgl_font_zh_16, UI_COLOR_TEXT);
    lv_obj_set_pos(s_motor_pwm_bars[i], x + 14, 108); lv_obj_set_size(s_motor_pwm_bars[i], 30, 128);
    if (s_value_valid[target]) {
      s_control_update_active = 1U;
      lv_slider_set_value(s_motor_pwm_bars[i], Display_LvglClampPercent(s_values[target]), LV_ANIM_OFF);
      s_control_update_active = 0U;
      lv_obj_clear_flag(s_motor_pwm_bars[i], LV_OBJ_FLAG_HIDDEN);
    }
    UiValue(target, x - 12, 252, 112);
    UiValue((Display_HmiVariableId_t)(DISPLAY_HMI_VAR_MOTOR_PWM_1 + i), x - 12, 278, 112);
    s_motor_pulse_labels[i] = UiLabel(s_motor_pulse_valid[i] ? s_motor_pulse_text[i] : "N/A", x - 12, 304, 112, &lv_font_montserrat_14, UI_COLOR_MUTED);
  }
  (void)UiButton("\xE5""\x90""\x8C""\xE6""\xAD""\xA5""\xE5""\x8D""\x87""\xE9""\x80""\x9F""", 548, 154, 228, 48, UI_ACTION_RAMP, 0, 0x173749);
  (void)UiButton("\xE6""\x9C""\xAC""\xE6""\x9C""\xBA""\xE6""\x80""\xA5""\xE5""\x81""\x9C""", 548, 211, 228, 56, UI_ACTION_ESTOP, 0, 0x782538);
  (void)UiButton("\xE7""\xBC""\x93""\xE9""\x99""\x8D""\xE5""\x81""\x9C""\xE6""\x9C""\xBA""", 548, 276, 228, 48, UI_ACTION_DOWN, 0, 0x214C36);
}

/** @brief 活动告警分页，全部 16 项可读；无数据与无活动告警独立表达。 */
static void Display_LvglLayoutAlarms(void)
{
  uint8_t i;
  UiPanel(0U, 8, 62, 784, 272);
  (void)UiLabel("Active Alarms", 24, 74, 208, &lv_font_montserrat_16, UI_COLOR_WARN);
  s_page_info = UiLabel("", 258, 74, 210, &lv_font_montserrat_14, UI_COLOR_MUTED);
  (void)UiButton("Next", 674, 68, 102, 38, UI_ACTION_ALARM_NEXT, 0, 0x173749);
  for (i = 0U; i < 5U; i++) {
    s_row_labels[i] = UiLabel("", 26, (lv_coord_t)(117 + i * 42), 748, &display_lvgl_font_zh_16, UI_COLOR_TEXT);
    lv_obj_set_height(s_row_labels[i], 40);
    lv_label_set_long_mode(s_row_labels[i], LV_LABEL_LONG_WRAP);
  }
}

/** @brief 模块状态、故障与运行诊断共用二级页面。 */
static void Display_LvglLayoutDiagnostics(void)
{
  uint8_t i;
  UiPanel(0U, 8, 62, 784, 362);
  s_runtime_label = UiLabel("", 24, 75, 630, &lv_font_montserrat_14, UI_COLOR_MUTED);
  (void)UiButton("Next", 680, 68, 98, 40, UI_ACTION_DIAG_NEXT, 0, 0x173749);
  s_page_info = UiLabel("", 24, 106, 744, &lv_font_montserrat_14, UI_COLOR_ACCENT);
  for (i = 0U; i < 6U; i++) {
    s_row_labels[i] = UiLabel("", 24, (lv_coord_t)(142 + i * 43), 752, &lv_font_montserrat_14, UI_COLOR_TEXT);
    lv_obj_set_height(s_row_labels[i], 40); lv_label_set_long_mode(s_row_labels[i], LV_LABEL_LONG_WRAP);
  }
}

/** @brief 固定 32 条事件快照的六行分页观察，不改变生产环。 */
static void Display_LvglLayoutHistory(void)
{
  uint8_t i;
  UiPanel(0U, 8, 62, 784, 362);
  s_page_info = UiLabel("", 24, 72, 742, &lv_font_montserrat_14, UI_COLOR_ACCENT);
  for (i = 0U; i < UI_HISTORY_ROWS; i++) {
    s_row_labels[i] = UiLabel("", 24, (lv_coord_t)(102 + i * 43), 750, &lv_font_montserrat_14, UI_COLOR_TEXT);
    lv_obj_set_height(s_row_labels[i], 40); lv_label_set_long_mode(s_row_labels[i], LV_LABEL_LONG_WRAP);
  }
  (void)UiButton("Newer", 22, 365, 126, 48, UI_ACTION_HISTORY_NEWER, 0, 0x173749);
  (void)UiButton("Older", 160, 365, 126, 48, UI_ACTION_HISTORY_OLDER, 0, 0x173749);
  (void)UiButton((s_history_filter == 0U) ? "All" : ((s_history_filter == 1U) ? "WARN+" : "ERROR"), 298, 365, 126, 48, UI_ACTION_HISTORY_FILTER, 0, 0x173749);
  (void)UiButton("Clear view", 436, 365, 166, 48, UI_ACTION_HISTORY_CLEAR, 0, 0x173749);
  (void)UiButton("Show all", 614, 365, 166, 48, UI_ACTION_HISTORY_ALL, 0, 0x173749);
}

/** @brief 通信列表保留真实 RX/估算丢帧，节点选择只保存在本页。 */
static void Display_LvglLayoutComm(void)
{
  uint8_t i;
  UiPanel(0U, 8, 62, 784, 272);
  UiValueRow(DISPLAY_HMI_VAR_LORA_TX_COUNT, "TX done", 22, 73, 86);
  UiValueRow(DISPLAY_HMI_VAR_LORA_RX_COUNT, "RX", 284, 73, 48);
  UiValueRow(DISPLAY_HMI_VAR_LORA_LOSS_RATE, "Loss est.", 526, 73, 100);
  s_page_info = UiLabel("Remote nodes", 24, 106, 622, &lv_font_montserrat_14, UI_COLOR_ACCENT);
  (void)UiButton("Next", 678, 101, 98, 36, UI_ACTION_REMOTE_NEXT, 0, 0x173749);
  for (i = 0U; i < 4U; i++) {
    uint8_t index = (uint8_t)(s_remote_offset + i);
    s_row_labels[i] = UiLabel("", 28, (lv_coord_t)(150 + i * 43), 596, &lv_font_montserrat_14, UI_COLOR_TEXT);
    lv_obj_set_height(s_row_labels[i], 40); lv_label_set_long_mode(s_row_labels[i], LV_LABEL_LONG_WRAP);
    if (index < s_remote_count) {
      (void)UiButton("Select", 660, (lv_coord_t)(144 + i * 43), 116, 38,
                     UI_ACTION_REMOTE_SELECT, s_remote_views[index].node_id, 0x173749);
    }
  }
}

/** @brief 将固定容量日志投影到当前视口，普通无新事件时不重排。 */
static void Display_LvglUpdateLogs(void)
{
  uint16_t i, n = 0U, begin, end, cleared_count = 0U;
  uint8_t indices[UI_LOG_CAP];
  uint8_t rows = (s_current_lvgl_page == DISPLAY_HMI_PAGE_EVENT_HISTORY) ? UI_HISTORY_ROWS : 3U;
  uint8_t base = (s_current_lvgl_page == DISPLAY_HMI_PAGE_EVENT_HISTORY) ? 0U : 9U;
  uint8_t history = (s_current_lvgl_page == DISPLAY_HMI_PAGE_EVENT_HISTORY) ? 1U : 0U;
  s_log_count = Display_PagesCopyLogMessages(s_log_entries, UI_LOG_CAP, 0, 0);
  s_log_version = Display_PagesGetLogVersion();
  if (s_history_cleared) {
    uint32_t appended = s_log_source_version - s_history_clear_version;
    /* 所有页面都推进清视图边界；整个环已覆盖后永久释放旧边界，不依赖序号或筛选。 */
    if (appended >= s_log_count) { s_history_cleared = 0U; }
    else { cleared_count = (uint16_t)(s_log_count - appended); }
  }
  for (i = 0U; i < s_log_count; i++) {
    uint8_t include = 1U;
    if (history && s_history_filter && s_log_entries[i].severity < s_history_filter) { include = 0U; }
    if (history && i < cleared_count) { include = 0U; }
    if (include) { indices[n++] = (uint8_t)i; }
  }
  if (history && s_history_offset >= n) { s_history_offset = n ? (uint8_t)(((n - 1U) / rows) * rows) : 0U; }
  end = history ? (uint16_t)(n - s_history_offset) : n;
  begin = (end > rows) ? (uint16_t)(end - rows) : 0U;
  for (i = 0U; i < rows; i++) {
    uint8_t slot = (uint8_t)(base + i);
    if (begin + i < end) {
      Display_MessageLogEntry_t *entry = &s_log_entries[indices[begin + i]];
      uint32_t t = entry->time_hhmmss;
      if (history) {
        (void)snprintf(s_row_text[slot], UI_ROW_TEXT_LEN, "%02lu:%02lu:%02lu  %-5s %s\n#%u  %s  0x%04X  active=%u",
          (unsigned long)(t / 10000U), (unsigned long)((t / 100U) % 100U), (unsigned long)(t % 100U),
          UiSeverity(entry->severity), Px4Lite_LogMessageText((uint16_t)entry->msg),
          (unsigned int)entry->sequence, (entry->source_id < PX4LITE_MODULE_COUNT) ? s_module_names[entry->source_id] : "system",
          (unsigned int)entry->fault_code, (unsigned int)entry->active);
      } else {
        (void)snprintf(s_row_text[slot], UI_ROW_TEXT_LEN, "%02lu:%02lu:%02lu  %-5s %s",
          (unsigned long)(t / 10000U), (unsigned long)((t / 100U) % 100U), (unsigned long)(t % 100U),
          UiSeverity(entry->severity), Px4Lite_LogMessageText((uint16_t)entry->msg));
      }
      if (s_row_labels[slot]) { lv_obj_set_style_text_color(s_row_labels[slot], lv_color_hex(UiSeverityColor(entry->severity)), 0); }
    } else {
      (void)snprintf(s_row_text[slot], UI_ROW_TEXT_LEN, "%s", (i == 0U) ? "No events in view" : "");
    }
    UiText(s_row_labels[slot], s_row_text[slot]);
  }
  if (history) {
    (void)snprintf(s_page_info_text, sizeof(s_page_info_text), "Uptime / local  %u-%u of %u  (ring %u/32)%s",
                   n ? (unsigned int)(begin + 1U) : 0U, (unsigned int)end, (unsigned int)n,
                   (unsigned int)s_log_count, s_history_cleared ? "  view cleared" : "");
    UiText(s_page_info, s_page_info_text);
  }
}

/** @brief 刷新活动告警摘要及分页视图；读失败不呈现无告警。 */
static void Display_LvglUpdateAlarms(void)
{
  uint8_t i, rows, offset;
  if (s_alarm_available) { (void)snprintf(s_alarm_header_text, sizeof(s_alarm_header_text), "Alarms %u", (unsigned int)s_alarm_count); }
  else { (void)snprintf(s_alarm_header_text, sizeof(s_alarm_header_text), "Alarms N/A"); }
  UiText(s_alarm_header, s_alarm_header_text);
  if ((s_current_lvgl_page != DISPLAY_HMI_PAGE_SELF_CHECK) && (s_current_lvgl_page != DISPLAY_HMI_PAGE_ALARM)) { return; }
  rows = (s_current_lvgl_page == DISPLAY_HMI_PAGE_SELF_CHECK) ? 4U : 5U;
  if (s_alarm_offset >= s_alarm_count) { s_alarm_offset = 0U; }
  offset = (s_current_lvgl_page == DISPLAY_HMI_PAGE_SELF_CHECK) ? 0U : s_alarm_offset;
  for (i = 0U; i < rows; i++) {
    uint8_t index = (uint8_t)(offset + i);
    if (!s_alarm_available) { (void)snprintf(s_row_text[i], UI_ROW_TEXT_LEN, "%s", i ? "" : "N/A - unavailable"); }
    else if (index < s_alarm_count) {
      uint16_t source = (uint16_t)(s_alarm_packed[index] >> 16);
      uint16_t fault = (uint16_t)s_alarm_packed[index];
      const App_AlarmRecord_t *detail = 0;
      uint16_t j;
      if (rows == 5U) {
        for (j = 0U; j < PX4LITE_MODULE_COUNT; j++) {
          const App_AlarmRecord_t *candidate = &s_alarm_details.records[j];
          if (candidate->active && candidate->source_id == source && candidate->fault_code == fault) { detail = candidate; break; }
        }
      }
      if ((rows == 5U) && detail) {
        uint32_t seconds = detail->raised_ms / 1000U;
        (void)snprintf(s_row_text[i], UI_ROW_TEXT_LEN, "0x%04X %s %s  %s\nUptime %02lu:%02lu:%02lu",
          (unsigned int)fault, (source < PX4LITE_MODULE_COUNT) ? s_module_names[source] : "System",
          UiSeverity(detail->severity), App_GetFaultReasonText(fault),
          (unsigned long)(seconds / 3600U), (unsigned long)((seconds / 60U) % 60U), (unsigned long)(seconds % 60U));
      } else {
        (void)snprintf(s_row_text[i], UI_ROW_TEXT_LEN, "0x%04X %s%s%s%s", (unsigned int)fault,
          (source < PX4LITE_MODULE_COUNT) ? s_module_names[source] : "System",
          (rows == 4U) ? "\n" : "  ", App_GetFaultReasonText(fault), (rows == 5U) ? "\nUptime N/A" : "");
      }
    } else { (void)snprintf(s_row_text[i], UI_ROW_TEXT_LEN, "%s", i ? "" : "No active alarms"); }
    UiText(s_row_labels[i], s_row_text[i]);
  }
  if (s_current_lvgl_page == DISPLAY_HMI_PAGE_ALARM) {
    (void)snprintf(s_page_info_text, sizeof(s_page_info_text), "%u-%u / %u",
      s_alarm_count ? (unsigned int)(offset + 1U) : 0U,
      (unsigned int)(((uint16_t)offset + rows < s_alarm_count) ? offset + rows : s_alarm_count), (unsigned int)s_alarm_count);
    UiText(s_page_info, s_page_info_text);
  }
}

/** @brief 200 ms 节奏读取 App 模块状态；仅活动诊断页格式化详情。 */
static void Display_LvglUpdateModules(void)
{
  uint8_t i;
  Px4Lite_ModuleStatus_t status;
  if (s_current_lvgl_page == DISPLAY_HMI_PAGE_SELF_CHECK) {
    for (i = 0U; i < UI_STATUS_COUNT; i++) {
      Px4Lite_Result_t result = App_GetModuleStatus(s_modules[i].id, &status);
      const char *state = (result == PX4LITE_OK) ? UiModuleState(status.state) : "N/A";
      if (strcmp(s_state_text[i], state) != 0) {
        uint32_t color = UI_COLOR_MUTED;
        if (result == PX4LITE_OK) {
          if (status.state == PX4LITE_STATE_ONLINE) { color = 0x59CC8A; }
          else if ((status.state == PX4LITE_STATE_STARTING) || (status.state == PX4LITE_STATE_DEGRADED)) { color = UI_COLOR_WARN; }
          else if ((status.state == PX4LITE_STATE_OFFLINE) || (status.state == PX4LITE_STATE_FAILED)) { color = UI_COLOR_ERROR; }
        }
        (void)snprintf(s_state_text[i], sizeof(s_state_text[i]), "%s", state);
        UiText(s_state_labels[i], s_state_text[i]);
        lv_obj_set_style_bg_color(s_dots[i], lv_color_hex(color), 0);
      }
    }
  } else if (s_current_lvgl_page == DISPLAY_HMI_PAGE_DIAGNOSTICS) {
    Display_RuntimeStatus_t runtime;
    Display_LvglGetStatus(&runtime);
    (void)snprintf(s_runtime_text, sizeof(s_runtime_text), "LCD %s  Touch %s  retries %lu  error %u",
      runtime.display_ready ? "ready" : "N/A", runtime.touch_ready ? "ready" : "N/A",
      (unsigned long)runtime.touch_recovery_count, (unsigned int)runtime.touch_last_error);
    UiText(s_runtime_label, s_runtime_text);
    (void)snprintf(s_page_info_text, sizeof(s_page_info_text), "Module / state / fault / age / errors / recoveries");
    UiText(s_page_info, s_page_info_text);
    for (i = 0U; i < 6U; i++) {
      uint8_t id = (uint8_t)(s_diag_offset + i);
      if (id < PX4LITE_MODULE_COUNT && App_GetModuleStatus((Px4Lite_ModuleId_t)id, &status) == PX4LITE_OK) {
        (void)snprintf(s_row_text[i], UI_ROW_TEXT_LEN, "%s  %s  0x%04X\nage %lus  errors %lu  recoveries %u",
          s_module_names[id], UiModuleState(status.state), (unsigned int)status.fault_code,
          (unsigned long)((s_last_tick_ms - status.state_since_ms) / 1000U), (unsigned long)status.error_count,
          (unsigned int)status.recovery_count);
      } else { (void)snprintf(s_row_text[i], UI_ROW_TEXT_LEN, "%s", (id < PX4LITE_MODULE_COUNT) ? "N/A" : ""); }
      UiText(s_row_labels[i], s_row_text[i]);
    }
  }
}

/** @brief 原位更新远端列表，只读身份和链路事实，不启用全局 remote 模式。 */
static void Display_LvglUpdateRemote(void)
{
  uint8_t i;
  if (s_current_lvgl_page != DISPLAY_HMI_PAGE_HIDDEN) { return; }
  if (s_remote_selection_valid) {
    (void)snprintf(s_page_info_text, sizeof(s_page_info_text), "remote Node %02u selected / link statistics", (unsigned int)s_remote_selected);
  } else { (void)snprintf(s_page_info_text, sizeof(s_page_info_text), "Remote nodes / select to inspect link statistics"); }
  UiText(s_page_info, s_page_info_text);
  for (i = 0U; i < 4U; i++) {
    uint8_t index = (uint8_t)(s_remote_offset + i);
    if (index < s_remote_count) {
      App_RemoteNodeView_t *node = &s_remote_views[index];
      const char *state = (node->state == APP_REMOTE_NODE_STALE) ? "STALE" :
                          ((node->state == APP_REMOTE_NODE_ACTIVE) ? "ACTIVE" : "HEARTBEAT");
      (void)snprintf(s_row_text[i], UI_ROW_TEXT_LEN, "%sNode %02u  %s  HB age %lus\nRX %lu  Loss est. %u.%u%%",
        (s_remote_selection_valid && node->node_id == s_remote_selected) ? "> " : "",
        (unsigned int)node->node_id, state, (unsigned long)((s_last_tick_ms - node->last_heartbeat_ms) / 1000U),
        (unsigned long)node->rx_frame_count, (unsigned int)(node->rx_loss_rate_x10 / 10U),
        (unsigned int)(node->rx_loss_rate_x10 % 10U));
    } else { (void)snprintf(s_row_text[i], UI_ROW_TEXT_LEN, "%s", (i == 0U) ? "No remote heartbeat" : ""); }
    UiText(s_row_labels[i], s_row_text[i]);
  }
}

/** @brief 重排唯一工作区，既不分配控件也不发控制命令。 */
static void Display_LvglLayout(void)
{
  uint16_t i;
  /* 外部按键也可切页。先取消旧触点，防止同一池对象改绑后把松手识别为新动作。 */
  lv_indev_t *indev = lv_indev_get_next(0);
  while (indev != 0) {
    lv_indev_reset(indev, 0);
    lv_indev_wait_release(indev);
    indev = lv_indev_get_next(indev);
  }
  Display_LvglResetWorkspace();
  /* WRAP 也使用静态文本，不启用滚动动画；下个页面恢复普通裁剪。 */
  for (i = 0U; i < UI_LABEL_COUNT; i++) { lv_label_set_long_mode(s_labels[i], LV_LABEL_LONG_CLIP); }
  Display_LvglLayoutChrome();
  switch (s_current_lvgl_page) {
    case DISPLAY_HMI_PAGE_SELF_CHECK: Display_LvglLayoutSelfCheck(); break;
    case DISPLAY_HMI_PAGE_FLIGHT: Display_LvglLayoutFlight(); break;
    case DISPLAY_HMI_PAGE_AIRCRAFT: Display_LvglLayoutAttitude(); break;
    case DISPLAY_HMI_PAGE_DATA: Display_LvglLayoutGnss(); break;
    case DISPLAY_HMI_PAGE_MOTOR: Display_LvglLayoutMotor(); break;
    case DISPLAY_HMI_PAGE_ALARM: Display_LvglLayoutAlarms(); break;
    case DISPLAY_HMI_PAGE_DIAGNOSTICS: Display_LvglLayoutDiagnostics(); break;
    case DISPLAY_HMI_PAGE_EVENT_HISTORY: Display_LvglLayoutHistory(); break;
    case DISPLAY_HMI_PAGE_HIDDEN: Display_LvglLayoutComm(); break;
    default: break;
  }
  if ((s_current_lvgl_page != DISPLAY_HMI_PAGE_DIAGNOSTICS) && (s_current_lvgl_page != DISPLAY_HMI_PAGE_EVENT_HISTORY)) { Display_LvglLayoutSummary(); }
  memset(s_state_text, 0, sizeof(s_state_text));
  Display_LvglUpdateAlarms();
  Display_LvglUpdateLogs();
  Display_LvglUpdateModules();
  Display_LvglUpdateRemote();
}

/** @brief 初始化硬件与静态工作区；显示恢复复用已有对象和事件历史。 */
Display_Result_t Display_LvglInit(uint32_t now_ms)
{
  if (!s_lvgl_core_ready) { lv_init(); s_lvgl_core_ready = 1U; }
  if (!LvPortDisp_Init()) { s_lvgl_display_ready = 0U; return DISPLAY_ERROR; }
  (void)LvPortIndev_Init();
  s_last_tick_ms = now_ms;
  Display_LvglInitWorkspace();
  s_lvgl_display_ready = 1U;
  Display_LvglLayout();
  return DISPLAY_OK;
}

/** @brief 检查 LCD 后端状态；触摸状态独立经运行状态接口提供。 */
Display_Result_t Display_LvglSelfCheck(uint16_t *error_code)
{
  if (error_code) { *error_code = s_lvgl_display_ready ? 0U : 1U; }
  return s_lvgl_display_ready ? DISPLAY_OK : DISPLAY_NOT_READY;
}

/** @brief 请求所属显示任务重试硬件，不销毁工作区或清除事件。 */
void Display_LvglRequestRecover(void) { s_lvgl_display_ready = 0U; }

/** @brief 页面切换/重排的即时服务提示。 */
uint8_t Display_LvglNeedsRefresh(void)
{
  return (uint8_t)(s_lvgl_display_ready && (s_page_change_requested || s_layout_requested));
}

/** @brief 服务触摸、页面请求与低频诊断；底层 LVGL flush 耗时仍需实板测量。 */
Display_Result_t Display_LvglRefreshStep(uint32_t now_ms, uint32_t budget_us)
{
  uint32_t elapsed_ms;
  (void)budget_us;
  if (!s_lvgl_display_ready) { return DISPLAY_NOT_READY; }
  Display_Gt911_Service(now_ms);
  elapsed_ms = now_ms - s_last_tick_ms;
  s_last_tick_ms = now_ms;
  if (elapsed_ms) { lv_tick_inc(elapsed_ms); }
  if (s_page_change_requested) {
    s_page_change_requested = 0U;
    if (s_requested_page != s_current_lvgl_page) { (void)Display_SetHmiPage(s_requested_page); }
  }
  if ((s_current_lvgl_page == DISPLAY_HMI_PAGE_HIDDEN) && ((int32_t)(now_ms - s_next_remote_ms) >= 0)) {
    uint8_t old_ids[UI_REMOTE_CAP], old_count = s_remote_count, i;
    for (i = 0U; i < old_count; i++) { old_ids[i] = s_remote_views[i].node_id; }
    if (App_CopyRemoteNodeStatuses(s_remote_views, UI_REMOTE_CAP, &s_remote_count, now_ms) != PX4LITE_OK) { s_remote_count = 0U; }
    if (old_count != s_remote_count) { s_layout_requested = 1U; }
    else { for (i = 0U; i < old_count; i++) { if (old_ids[i] != s_remote_views[i].node_id) { s_layout_requested = 1U; break; } } }
    if (s_remote_offset >= s_remote_count) { s_remote_offset = 0U; }
    s_next_remote_ms = now_ms + 1000U;
    Display_LvglUpdateRemote();
  }
  if (s_layout_requested) { s_layout_requested = 0U; Display_LvglLayout(); }
  if ((int32_t)(now_ms - s_next_status_ms) >= 0) { Display_LvglUpdateModules(); s_next_status_ms = now_ms + 200U; }
  if (s_log_version != Display_PagesGetLogVersion()) { Display_LvglUpdateLogs(); }
  lv_timer_handler();
  return DISPLAY_OK;
}

/** @brief 写入显示缓存；仅活动标签和姿态对象失效，不新增控件。 */
Display_Result_t Display_LvglSetValue(Display_HmiVariableId_t id, uint32_t value)
{
  uint8_t changed;
  uint8_t was_valid;
  if (id >= DISPLAY_HMI_VAR_COUNT) { return DISPLAY_ERROR; }
  was_valid = s_value_valid[id];
  changed = (uint8_t)(!s_value_valid[id] || s_values[id] != value);
  s_values[id] = value; s_value_valid[id] = 1U;
  if (!changed || !s_workspace_ready) { return DISPLAY_OK; }
  if (s_value_slots[id].label) { Display_LvglFormatValue(id, value); UiText(s_value_slots[id].label, s_value_slots[id].text); }
  if (id == DISPLAY_HMI_VAR_MESSAGE_LOG) { Display_LvglUpdateLogs(); }
  if ((id >= DISPLAY_HMI_VAR_MOTOR_TARGET_1) && (id <= DISPLAY_HMI_VAR_MOTOR_TARGET_4) && (s_current_lvgl_page == DISPLAY_HMI_PAGE_MOTOR)) {
    uint8_t index = (uint8_t)(id - DISPLAY_HMI_VAR_MOTOR_TARGET_1);
    lv_obj_clear_flag(s_motor_pwm_bars[index], LV_OBJ_FLAG_HIDDEN);
    if (!was_valid || (!s_motor_slider_dragging[index] && !lv_slider_is_dragged(s_motor_pwm_bars[index]))) {
      s_control_update_active = 1U; lv_slider_set_value(s_motor_pwm_bars[index], Display_LvglClampPercent(value), LV_ANIM_OFF); s_control_update_active = 0U;
    }
  }
  if ((id >= DISPLAY_HMI_VAR_ATTITUDE_ACTION) && (id <= DISPLAY_HMI_VAR_ATTITUDE_REFERENCE) && s_value_slots[DISPLAY_HMI_VAR_ATTITUDE_PHASE].label) {
    Display_LvglFormatValue(DISPLAY_HMI_VAR_ATTITUDE_PHASE, s_values[DISPLAY_HMI_VAR_ATTITUDE_PHASE]);
    UiText(s_value_slots[DISPLAY_HMI_VAR_ATTITUDE_PHASE].label, s_value_slots[DISPLAY_HMI_VAR_ATTITUDE_PHASE].text);
  }
  if ((id == DISPLAY_HMI_VAR_ROLL) || (id == DISPLAY_HMI_VAR_PITCH) || (id == DISPLAY_HMI_VAR_YAW)) {
    s_attitude_roll_deg10 = (int16_t)s_values[DISPLAY_HMI_VAR_ROLL];
    s_attitude_pitch_deg10 = (int16_t)s_values[DISPLAY_HMI_VAR_PITCH];
    s_attitude_yaw_deg10 = (int16_t)s_values[DISPLAY_HMI_VAR_YAW];
    if ((s_current_lvgl_page == DISPLAY_HMI_PAGE_AIRCRAFT) && s_value_valid[DISPLAY_HMI_VAR_ROLL] && s_value_valid[DISPLAY_HMI_VAR_PITCH] && s_value_valid[DISPLAY_HMI_VAR_YAW]) {
      lv_obj_clear_flag(s_attitude_obj, LV_OBJ_FLAG_HIDDEN); lv_obj_invalidate(s_attitude_obj);
    }
  }
  return DISPLAY_OK;
}

/** @brief 值无效时撤销旧值，不把未知数字变成有效零。 */
Display_Result_t Display_LvglInvalidateValue(Display_HmiVariableId_t id)
{
  if (id >= DISPLAY_HMI_VAR_COUNT) { return DISPLAY_ERROR; }
  if (s_value_valid[id] == 0U) { return DISPLAY_OK; }
  s_value_valid[id] = 0U;
  Display_LvglFormatValue(id, s_values[id]);
  UiText(s_value_slots[id].label, s_value_slots[id].text);
  if ((id >= DISPLAY_HMI_VAR_MOTOR_TARGET_1) && (id <= DISPLAY_HMI_VAR_MOTOR_TARGET_4)) {
    uint8_t index = (uint8_t)(id - DISPLAY_HMI_VAR_MOTOR_TARGET_1);
    s_motor_slider_dragging[index] = 0U; s_motor_touch_valid[index] = 0U;
    if (s_motor_pwm_bars[index]) { lv_obj_add_flag(s_motor_pwm_bars[index], LV_OBJ_FLAG_HIDDEN); }
  }
  if (((id == DISPLAY_HMI_VAR_ROLL) || (id == DISPLAY_HMI_VAR_PITCH) || (id == DISPLAY_HMI_VAR_YAW)) && s_attitude_obj) {
    lv_obj_add_flag(s_attitude_obj, LV_OBJ_FLAG_HIDDEN);
  }
  return DISPLAY_OK;
}

/** @brief 显示 Control 最终输出脉宽，不用百分数推算或改变目标。 */
Display_Result_t Display_LvglSetMotorPulseUs(uint8_t motor_index, uint16_t pulse_us)
{
  if (motor_index >= 4U) { return DISPLAY_ERROR; }
  if ((pulse_us < PX4LITE_CONTROL_ESC_MIN_PULSE_US) || (pulse_us > PX4LITE_CONTROL_ESC_MAX_PULSE_US)) {
    (void)Display_LvglInvalidateMotorPulseUs(motor_index);
    return DISPLAY_ERROR;
  }
  if (s_motor_pulse_valid[motor_index] && s_motor_pulse_us[motor_index] == pulse_us) { return DISPLAY_OK; }
  s_motor_pulse_us[motor_index] = pulse_us; s_motor_pulse_valid[motor_index] = 1U;
  (void)snprintf(s_motor_pulse_text[motor_index], sizeof(s_motor_pulse_text[motor_index]), "%uus", (unsigned int)pulse_us);
  UiText(s_motor_pulse_labels[motor_index], s_motor_pulse_text[motor_index]);
  return DISPLAY_OK;
}

/** @brief 撤销一路实际脉宽；无效时不显示旧值或 ESC 最小输出假值。 */
Display_Result_t Display_LvglInvalidateMotorPulseUs(uint8_t motor_index)
{
  if (motor_index >= 4U) { return DISPLAY_ERROR; }
  s_motor_pulse_valid[motor_index] = 0U;
  (void)snprintf(s_motor_pulse_text[motor_index], sizeof(s_motor_pulse_text[motor_index]), "N/A");
  UiText(s_motor_pulse_labels[motor_index], s_motor_pulse_text[motor_index]);
  return DISPLAY_OK;
}

/** @brief 切换两层导航；主入口始终使用本机数据。 */
Display_Result_t Display_LvglSetPage(Display_HmiPage_t page)
{
  if (!s_lvgl_display_ready) { return DISPLAY_NOT_READY; }
  if (page == DISPLAY_HMI_PAGE_LOGO) { page = DISPLAY_HMI_PAGE_SELF_CHECK; }
  if (page >= DISPLAY_HMI_PAGE_COUNT) { return DISPLAY_ERROR; }
  if (page <= DISPLAY_HMI_PAGE_ALARM) { s_primary_page = page; }
  s_current_lvgl_page = page;
  Display_LvglLayout();
  return DISPLAY_OK;
}

/** @brief 同一快照同时更新列表与详情；严重度和触发时刻变化也会刷新，不在 UI 重读告警。 */
void Display_LvglSetAlarmSnapshot(const App_AlarmSnapshot_t *alarm, const uint32_t *packed, uint16_t count)
{
  uint16_t i;
  uint8_t changed;
  uint8_t available = (alarm != 0) ? 1U : 0U;
  if (count > UI_ALARM_CAP) { count = UI_ALARM_CAP; }
  if (!packed || !available) { count = 0U; }
  changed = (uint8_t)((s_alarm_count != count) || (s_alarm_available != available));
  if (available) {
    for (i = 0U; i < PX4LITE_MODULE_COUNT; ++i) {
      const App_AlarmRecord_t *old_record = &s_alarm_details.records[i];
      const App_AlarmRecord_t *new_record = &alarm->records[i];
      if ((old_record->active != new_record->active) || (old_record->source_id != new_record->source_id) ||
          (old_record->fault_code != new_record->fault_code) || (old_record->severity != new_record->severity) ||
          (old_record->raised_ms != new_record->raised_ms)) { changed = 1U; }
    }
    s_alarm_details = *alarm;
  }
  for (i = 0U; i < UI_ALARM_CAP; i++) {
    uint32_t value = (i < count) ? packed[i] : 0U;
    if (s_alarm_packed[i] != value) { s_alarm_packed[i] = value; changed = 1U; }
  }
  s_alarm_count = count;
  s_alarm_available = available;
  if (changed && s_workspace_ready) { Display_LvglUpdateAlarms(); }
}

/** @brief 同显示任务按每份生产窗口推进边界；离开历史页期间整环覆盖也立即释放旧边界。 */
void Display_LvglSetLogSnapshotVersion(uint32_t source_version)
{
  s_log_source_version = source_version;
  if (s_history_cleared && ((uint32_t)(source_version - s_history_clear_version) >= UI_LOG_CAP)) {
    s_history_cleared = 0U;
  }
}

/** @brief 冻结仅适用于观察数据；Display 门面保持控制/告警/事件/时间实时。 */
uint8_t Display_LvglObservationFrozen(void) { return s_observation_frozen; }

/** @brief 只复制同一显示任务维护的 LCD/触摸状态，无硬件 I/O。 */
void Display_LvglGetStatus(Display_RuntimeStatus_t *out)
{
  Display_Gt911Status_t touch;
  if (!out) { return; }
  Display_Gt911_GetStatus(&touch);
  memset(out, 0, sizeof(*out));
  out->display_ready = s_lvgl_display_ready;
  out->touch_registered = LvPortIndev_IsRegistered();
  out->touch_ready = (uint8_t)(touch.ready && out->touch_registered);
  out->touch_recovering = touch.recovering;
  out->touch_last_error = touch.last_error;
  out->touch_recovery_count = touch.recovery_count;
}
