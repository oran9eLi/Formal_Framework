/**
 * @file display_pages.c
 * @brief Implement fixed page layouts and dynamic field drawing.
 */

#include "display_pages.h"
#include "display_gfx.h"
#include "display_text.h"
#include "display_logo.h"
#include "px4lite_faults.h"

#define DISPLAY_HEADER_HEIGHT          64U
#define DISPLAY_HEADER_DT_X            70U
#define DISPLAY_HEADER_DATE_Y         16U
#define DISPLAY_HEADER_TIME_Y         30U
#define DISPLAY_HEADER_POWER_CLEAR_X  560U
#define DISPLAY_HEADER_POWER_CLEAR_Y  24U
#define DISPLAY_HEADER_POWER_CLEAR_W  90U
#define DISPLAY_HEADER_POWER_CLEAR_H  18U
#define DISPLAY_HEADER_LOSS_X         660U
#define DISPLAY_HEADER_LOSS_Y         27U
#define DISPLAY_DASH_Y                 76U
#define DISPLAY_DASH_H                 330U
#define DISPLAY_DASH_STATUS_X          8U
#define DISPLAY_DASH_STATUS_W          248U
#define DISPLAY_DASH_GPS_X             264U
#define DISPLAY_DASH_GPS_W             268U
#define DISPLAY_DASH_LOG_X             540U
#define DISPLAY_DASH_LOG_W             252U
#define DISPLAY_DASH_ROW_Y             122U
#define DISPLAY_DASH_ROW_H             30U
#define DISPLAY_SELFCHECK_ERR_BODY_Y   153U
#define DISPLAY_SELFCHECK_ERR_BODY_H   238U
#define DISPLAY_SELFCHECK_ERR_ROW_Y0   170U
#define DISPLAY_SELFCHECK_ERR_ROW_H    22U
#define DISPLAY_SELFCHECK_ERR_MAX_ROWS 10U
#define DISPLAY_SELFCHECK_ERR_CODE_X   492U
#define DISPLAY_SELFCHECK_ERR_MODULE_X 578U
#define DISPLAY_SELFCHECK_ERR_REASON_X 684U
#define DISPLAY_MSGLOG_CAP             9U
#define DISPLAY_MSGLOG_BODY_Y          106U
#define DISPLAY_MSGLOG_BODY_H          298U
#define DISPLAY_MSGLOG_ROW_Y0          120U
#define DISPLAY_MSGLOG_ROW_H           30U
#define DISPLAY_MSGLOG_TIME_X          548U
#define DISPLAY_MSGLOG_TEXT_X          600U
#define DISPLAY_MSGLOG_ALARM_X         730U
#define DISPLAY_MSGLOG_ALARM_Y         84U
#define DISPLAY_MSGLOG_ALARM_W         58U
#define DISPLAY_MSGLOG_ALARM_H         18U
#define DISPLAY_ALARM_TABLE_X          40U
#define DISPLAY_ALARM_TABLE_Y          110U
#define DISPLAY_ALARM_TABLE_W          720U
#define DISPLAY_ALARM_TABLE_H          291U
#define DISPLAY_ALARM_CODE_X           190U
#define DISPLAY_ALARM_MODULE_X         360U
#define DISPLAY_ALARM_HEADER_H         40U
#define DISPLAY_ALARM_ROW_H            50U
#define DISPLAY_ALARM_ROW_COUNT        5U
#define DISPLAY_ALARM_BODY_Y           (DISPLAY_ALARM_TABLE_Y + DISPLAY_ALARM_HEADER_H)
#define DISPLAY_ALARM_TITLE_X          52U
#define DISPLAY_ALARM_TITLE_Y          72U
#define DISPLAY_ALARM_COLOR_HEADER     DISPLAY_THEME_PANEL_HI
#define DISPLAY_ALARM_COLOR_GRID       DISPLAY_GFX_COLOR_CARD_BORDER
#define DISPLAY_ALARM_COLOR_ROW_ALT    DISPLAY_THEME_PANEL_HI
#define DISPLAY_MOTOR_STATUS_X         8U
#define DISPLAY_MOTOR_STATUS_W         204U
#define DISPLAY_MOTOR_PANEL_X          220U
#define DISPLAY_MOTOR_PANEL_W          388U
#define DISPLAY_MOTOR_LOG_X            616U
#define DISPLAY_MOTOR_LOG_W            176U
#define DISPLAY_MOTOR_LABEL_Y          116U
#define DISPLAY_MOTOR_MSGLOG_TIME_X    624U
#define DISPLAY_MOTOR_MSGLOG_TEXT_X    676U
#define DISPLAY_MOTOR_MSGLOG_ALARM_X   734U
#define DISPLAY_MOTOR_MSGLOG_ALARM_W   54U
#define DISPLAY_MOTOR_SLIDER_MAX_VALUE 100U
#define DISPLAY_PAGES_CARD_RADIUS      8U

/*
 * 通过回调读取变量值，回调为空时使用默认值。
 */
static uint32_t Display_PagesReadValue(Display_PagesValueReader_t read_value, Display_HmiVariableId_t id, uint32_t default_value)
{
  if (read_value == 0) { return default_value; }

  return read_value(id, default_value);
}

/*
 * 将状态值映射为页面显示颜色值
 */
static uint16_t Display_PagesGetStatusColor(uint32_t value)
{
  if (value == 0U) { return DISPLAY_GFX_COLOR_GRAY; }

  if (value == 1U) { return DISPLAY_GFX_COLOR_YELLOW; }

  if (value == 2U) { return DISPLAY_GFX_COLOR_GREEN; }

  return DISPLAY_GFX_COLOR_RED;
}

/* 深色卡片外框：深色面板底 + 边框圆角。字段刷新各自清成面板底色，逻辑不变。 */
static void Display_PagesDrawCardFrame(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
  (void)Display_GfxFillRect(x, y, width, height, DISPLAY_THEME_PANEL);
  (void)Display_GfxDrawRoundRect(x, y, width, height, DISPLAY_PAGES_CARD_RADIUS, DISPLAY_THEME_BORDER);
}

/* 卡片标题行：左侧青色强调竖条 + 加粗浅色标题 + 分隔线，分隔线内缩避开圆角。 */
static void Display_PagesDrawCardTitle(uint16_t x, uint16_t y, uint16_t width, Display_TextLabel_t title_label)
{
  (void)Display_GfxFillRect((uint16_t)(x + 12U), (uint16_t)(y + 8U), 4U, 14U, DISPLAY_THEME_ACCENT);
  (void)Display_TextDrawLabelBold((uint16_t)(x + 22U), (uint16_t)(y + 5U), title_label, DISPLAY_THEME_TEXT);
  (void)Display_GfxDrawHLine((uint16_t)(x + 10U), (uint16_t)(y + 28U), (uint16_t)(width - 20U), DISPLAY_THEME_DIVIDER);
}

static void Display_PagesDrawPanelLabel(uint16_t x, uint16_t y, uint16_t width, uint16_t height, Display_TextLabel_t title_label)
{
  Display_PagesDrawCardFrame(x, y, width, height);
  Display_PagesDrawCardTitle(x, y, width, title_label);
}

static void Display_PagesDrawMotorLabel(uint16_t x, uint16_t y, char index)
{
  uint16_t motor_w;

  (void)Display_GfxDrawString(x, y, "-", DISPLAY_THEME_TEXT, 1U);
  (void)Display_TextDrawLabel((uint16_t)(x + 12U), y, DISPLAY_TXT_MOTOR, DISPLAY_THEME_TEXT);
  motor_w = Display_TextGetLabelWidth(DISPLAY_TXT_MOTOR);
  (void)Display_GfxDrawChar((uint16_t)(x + 16U + motor_w), y, index, DISPLAY_THEME_TEXT, 1U);
}

static void Display_PagesDrawSelfMotorLabel(uint16_t x, uint16_t y, uint16_t width, char index)
{
  uint16_t motor_w = Display_TextGetLabelWidth(DISPLAY_TXT_MOTOR);
  uint16_t text_w  = (uint16_t)(motor_w + 8U);
  uint16_t text_x  = (text_w < width) ? (uint16_t)(x + ((width - text_w) / 2U)) : x;

  (void)Display_TextDrawLabel(text_x, y, DISPLAY_TXT_MOTOR, DISPLAY_THEME_TEXT);
  (void)Display_GfxDrawChar((uint16_t)(text_x + motor_w + 2U), y, index, DISPLAY_THEME_TEXT, 1U);
}

/*
 * 根据页面 ID 获取标题文字标签。 */
/*
 * 绘制页面底部翻页按钮和页码状态点。
 */
/* 底部标签栏：六个可导航页面直达，当前页高亮青色 + 顶部强调条。 */
#define DISPLAY_PAGES_TAB_COUNT (uint16_t)(DISPLAY_HMI_PAGE_HIDDEN - DISPLAY_HMI_PAGE_SELF_CHECK)

static void Display_PagesDrawFooter(Display_HmiPage_t page)
{
  static const Display_TextLabel_t tab_labels[DISPLAY_PAGES_TAB_COUNT] = {DISPLAY_TEXT_SELF_CHECK, DISPLAY_TITLE_FLIGHT, DISPLAY_TITLE_AIRCRAFT, DISPLAY_TITLE_GNSS, DISPLAY_TEXT_MOTOR_PWM, DISPLAY_TEXT_ALARM_PAGE};
  uint16_t tab_w = (uint16_t)(DISPLAY_GFX_WIDTH / DISPLAY_PAGES_TAB_COUNT);
  uint16_t i;

  (void)Display_GfxFillRect(0U, DISPLAY_PAGES_FOOTER_Y, DISPLAY_GFX_WIDTH, DISPLAY_PAGES_FOOTER_HEIGHT, DISPLAY_THEME_PANEL);
  (void)Display_GfxDrawHLine(0U, DISPLAY_PAGES_FOOTER_Y, DISPLAY_GFX_WIDTH, DISPLAY_THEME_BORDER);

  for (i = 0U; i < DISPLAY_PAGES_TAB_COUNT; i++) {
    uint16_t tx      = (uint16_t)(i * tab_w);
    uint16_t page_i  = (uint16_t)((uint16_t)DISPLAY_HMI_PAGE_SELF_CHECK + i);
    uint8_t active   = (uint8_t)(page_i == (uint16_t)page);
    uint16_t label_w = Display_TextGetLabelWidth(tab_labels[i]);
    uint16_t lx      = (label_w < tab_w) ? (uint16_t)(tx + ((tab_w - label_w) / 2U)) : tx;
    uint16_t color   = active ? DISPLAY_THEME_ACCENT : DISPLAY_THEME_TEXT_MUTED;

    if (i != 0U) { (void)Display_GfxDrawVLine(tx, (uint16_t)(DISPLAY_PAGES_FOOTER_Y + 8U), (uint16_t)(DISPLAY_PAGES_FOOTER_HEIGHT - 16U), DISPLAY_THEME_DIVIDER); }
    if (active != 0U) { (void)Display_GfxFillRect(tx, DISPLAY_PAGES_FOOTER_Y, tab_w, 3U, DISPLAY_THEME_ACCENT); }
    (void)Display_TextDrawLabel(lx, (uint16_t)(DISPLAY_PAGES_FOOTER_Y + 22U), tab_labels[i], color);
  }
}

static void Display_PagesDrawDataStatusRow(uint16_t x, uint16_t y, Display_TextLabel_t label, char motor_index)
{
  (void)Display_GfxDrawStatusDot(x, (uint16_t)(y + 8U), 6U, DISPLAY_GFX_COLOR_GRAY, DISPLAY_GFX_COLOR_BLACK);

  if (motor_index != 0) {
    Display_PagesDrawMotorLabel((uint16_t)(x + 20U), y, motor_index);
  } else {
    (void)Display_TextDrawLabel((uint16_t)(x + 20U), y, label, DISPLAY_THEME_TEXT);
  }
}

static void Display_PagesDrawDataGpsRow(uint16_t label_x, uint16_t y, Display_TextLabel_t label)
{
  (void)Display_TextDrawLabel(label_x, y, label, DISPLAY_THEME_TEXT);
  (void)Display_GfxDrawHLine((uint16_t)(label_x - 8U), (uint16_t)(y + 24U), 236U, DISPLAY_THEME_DIVIDER);
}

static void Display_PagesDrawDataGpsLabelRow(uint16_t label_x, uint16_t y, Display_TextLabel_t label, const char *placeholder)
{
  (void)Display_TextDrawLabel(label_x, y, label, DISPLAY_THEME_TEXT);
  if (placeholder != 0) { (void)Display_GfxDrawString(390U, (uint16_t)(y - 2U), placeholder, DISPLAY_GFX_COLOR_GRAY, 2U); }
  (void)Display_GfxDrawHLine((uint16_t)(label_x - 8U), (uint16_t)(y + 24U), 236U, DISPLAY_THEME_DIVIDER);
}

/* 飞行数据页双电池行：主控/电机 + 电压/电池标签 + 下划线。 */
static void Display_PagesDrawBatteryRow(uint16_t label_x, uint16_t y, uint8_t is_motor, uint8_t is_battery)
{
  (void)Display_TextDrawBatteryLabel(label_x, y, is_motor, is_battery, DISPLAY_THEME_TEXT);
  (void)Display_GfxDrawHLine((uint16_t)(label_x - 8U), (uint16_t)(y + 24U), 236U, DISPLAY_THEME_DIVIDER);
}

/*
 * 绘制左侧系统栏面板和 9 个模块状态行。
 * 三个数据页(飞行数据/飞机情况/定位数据)共用同一系统栏。
 */
static void Display_PagesDrawSystemColumnAt(uint16_t x, uint16_t width)
{
  uint16_t y;
  uint16_t dot_x = (uint16_t)(x + 22U);

  Display_PagesDrawPanelLabel(x, DISPLAY_DASH_Y, width, DISPLAY_DASH_H, DISPLAY_TITLE_SYSTEM);

  y = DISPLAY_DASH_ROW_Y;
  Display_PagesDrawDataStatusRow(dot_x, y, DISPLAY_TEXT_SELF_GNSS, 0);
  y = (uint16_t)(y + DISPLAY_DASH_ROW_H);
  Display_PagesDrawDataStatusRow(dot_x, y, DISPLAY_TEXT_SELF_ATTITUDE, 0);
  y = (uint16_t)(y + DISPLAY_DASH_ROW_H);
  Display_PagesDrawDataStatusRow(dot_x, y, DISPLAY_TEXT_SELF_ENV, 0);
  y = (uint16_t)(y + DISPLAY_DASH_ROW_H);
  Display_PagesDrawDataStatusRow(dot_x, y, DISPLAY_TEXT_SELF_LORA, 0);
  y = (uint16_t)(y + DISPLAY_DASH_ROW_H);
  Display_PagesDrawDataStatusRow(dot_x, y, DISPLAY_TEXT_SELF_STORAGE, 0);
  y = (uint16_t)(y + DISPLAY_DASH_ROW_H);
  Display_PagesDrawDataStatusRow(dot_x, y, DISPLAY_TEXT_COUNT, '1');
  y = (uint16_t)(y + DISPLAY_DASH_ROW_H);
  Display_PagesDrawDataStatusRow(dot_x, y, DISPLAY_TEXT_COUNT, '2');
  y = (uint16_t)(y + DISPLAY_DASH_ROW_H);
  Display_PagesDrawDataStatusRow(dot_x, y, DISPLAY_TEXT_COUNT, '3');
  y = (uint16_t)(y + DISPLAY_DASH_ROW_H);
  Display_PagesDrawDataStatusRow(dot_x, y, DISPLAY_TEXT_COUNT, '4');
}

static void Display_PagesDrawSystemColumn(void)
{
  Display_PagesDrawSystemColumnAt(DISPLAY_DASH_STATUS_X, DISPLAY_DASH_STATUS_W);
}

/* ---- 消息日志环形缓冲 ---- */
typedef struct {
  Display_LogMsg_t msg;
  uint32_t time_hhmmss;
} Display_MsgLogEntry_t;

static Display_MsgLogEntry_t s_msglog[DISPLAY_MSGLOG_CAP];
static Display_MsgLogEntry_t s_msglog_alarm;
static uint16_t s_msglog_head;  /* 下一个写入位置 */
static uint16_t s_msglog_count; /* 已存普通消息条数，上限 CAP */
static uint8_t s_msglog_alarm_valid;
static uint32_t s_msglog_version; /* 每追加一条 +1，用于驱动重绘 */

static uint8_t Display_PagesIsAlarmLogMessage(Display_LogMsg_t msg)
{
  return ((msg == DISPLAY_LOGMSG_ALARM_ACTIVE) || (msg == DISPLAY_LOGMSG_ALARM_NONE)) ? 1U : 0U;
}

void Display_PagesPushLogMessage(Display_LogMsg_t msg, uint32_t time_hhmmss)
{
  if (msg >= DISPLAY_LOGMSG_COUNT) { return; }

  if (Display_PagesIsAlarmLogMessage(msg) != 0U) {
    s_msglog_alarm.msg   = msg;
    s_msglog_alarm_valid = 1U;
    s_msglog_version++;
    return;
  }

  s_msglog[s_msglog_head].msg         = msg;
  s_msglog[s_msglog_head].time_hhmmss = time_hhmmss;
  s_msglog_head                       = (uint16_t)((s_msglog_head + 1U) % DISPLAY_MSGLOG_CAP);
  if (s_msglog_count < DISPLAY_MSGLOG_CAP) { s_msglog_count++; }
  s_msglog_version++;
}

uint32_t Display_PagesGetLogVersion(void)
{
  return s_msglog_version;
}

uint16_t Display_PagesCopyLogMessages(Display_MessageLogEntry_t *entries, uint16_t max_count, Display_MessageLogEntry_t *alarm_entry, uint8_t *alarm_valid)
{
  uint16_t copy_count;
  uint16_t first;
  uint16_t i;

  if (alarm_valid != 0) {
    *alarm_valid = s_msglog_alarm_valid;
  }
  if ((alarm_entry != 0) && (s_msglog_alarm_valid != 0U)) {
    alarm_entry->msg         = s_msglog_alarm.msg;
    alarm_entry->time_hhmmss = s_msglog_alarm.time_hhmmss;
  }

  if ((entries == 0) || (max_count == 0U)) {
    return 0U;
  }

  copy_count = (s_msglog_count < max_count) ? s_msglog_count : max_count;
  first      = (uint16_t)(s_msglog_count - copy_count);

  for (i = 0U; i < copy_count; i++) {
    uint16_t idx = (uint16_t)((s_msglog_head + DISPLAY_MSGLOG_CAP - s_msglog_count + first + i) % DISPLAY_MSGLOG_CAP);
    entries[i].msg         = s_msglog[idx].msg;
    entries[i].time_hhmmss = s_msglog[idx].time_hhmmss;
  }

  return copy_count;
}

/* 把 HHMMSS 编码格式化为 "HH:MM:SS"。 */
static void Display_PagesFmtClock(char *buf, uint32_t hhmmss)
{
  uint32_t hh = (hhmmss / 10000U) % 100U;
  uint32_t mm = (hhmmss / 100U) % 100U;
  uint32_t ss = hhmmss % 100U;

  buf[0] = (char)('0' + ((hh / 10U) % 10U));
  buf[1] = (char)('0' + (hh % 10U));
  buf[2] = ':';
  buf[3] = (char)('0' + ((mm / 10U) % 10U));
  buf[4] = (char)('0' + (mm % 10U));
  buf[5] = ':';
  buf[6] = (char)('0' + ((ss / 10U) % 10U));
  buf[7] = (char)('0' + (ss % 10U));
  buf[8] = '\0';
}

static void Display_PagesDrawMessageLogAlarmStatusAt(uint16_t alarm_x, uint16_t alarm_y, uint16_t alarm_w, uint16_t alarm_h)
{
  uint16_t color;

  (void)Display_GfxFillRect(alarm_x, alarm_y, alarm_w, alarm_h, DISPLAY_THEME_PANEL);

  if (s_msglog_alarm_valid == 0U) { return; }

  color = (s_msglog_alarm.msg == DISPLAY_LOGMSG_ALARM_ACTIVE) ? DISPLAY_GFX_COLOR_RED : DISPLAY_THEME_TEXT;
  (void)Display_TextDrawLogMessage(alarm_x, alarm_y, s_msglog_alarm.msg, color);
}

/*
 * 绘制消息日志正文：普通消息从最旧到最新逐行显示。
 * 由 DISPLAY_HMI_VAR_MESSAGE_LOG 字段刷新驱动；缓冲为空时整片留白。
 */
static void Display_PagesDrawMessageLogBodyAt(uint16_t log_x, uint16_t log_w, uint16_t time_x, uint16_t text_x, uint16_t alarm_x, uint16_t alarm_w)
{
  uint16_t i;
  char tbuf[9];

  if (Display_GfxIsReady() == 0U) { return; }

  Display_PagesDrawMessageLogAlarmStatusAt(alarm_x, DISPLAY_MSGLOG_ALARM_Y, alarm_w, DISPLAY_MSGLOG_ALARM_H);

  (void)Display_GfxFillRect((uint16_t)(log_x + 1U), DISPLAY_MSGLOG_BODY_Y, (uint16_t)(log_w - 2U), DISPLAY_MSGLOG_BODY_H, DISPLAY_THEME_PANEL);

  for (i = 0U; i < s_msglog_count; i++) {
    uint16_t idx   = (uint16_t)((s_msglog_head + DISPLAY_MSGLOG_CAP - s_msglog_count + i) % DISPLAY_MSGLOG_CAP);
    uint16_t row_y = (uint16_t)(DISPLAY_MSGLOG_ROW_Y0 + (i * DISPLAY_MSGLOG_ROW_H));

    Display_PagesFmtClock(tbuf, s_msglog[idx].time_hhmmss);
    (void)Display_GfxDrawString(time_x, (uint16_t)(row_y + 4U), tbuf, DISPLAY_GFX_COLOR_GRAY, 1U);
    (void)Display_TextDrawLogMessage(text_x, row_y, s_msglog[idx].msg, DISPLAY_THEME_TEXT);
  }
}

static void Display_PagesDrawMessageLogBody(void)
{
  Display_PagesDrawMessageLogBodyAt(DISPLAY_DASH_LOG_X, DISPLAY_DASH_LOG_W, DISPLAY_MSGLOG_TIME_X, DISPLAY_MSGLOG_TEXT_X, DISPLAY_MSGLOG_ALARM_X, DISPLAY_MSGLOG_ALARM_W);
}

static void Display_PagesDrawMotorMessageLogBody(void)
{
  Display_PagesDrawMessageLogBodyAt(DISPLAY_MOTOR_LOG_X, DISPLAY_MOTOR_LOG_W, DISPLAY_MOTOR_MSGLOG_TIME_X, DISPLAY_MOTOR_MSGLOG_TEXT_X, DISPLAY_MOTOR_MSGLOG_ALARM_X, DISPLAY_MOTOR_MSGLOG_ALARM_W);
}

/*
 * 绘制右侧消息日志面板的固定骨架(外框+标题+分隔线)。
 * 正文由 Display_PagesDrawMessageLogBody 动态刷新。三个数据页共用。
 */
static void Display_PagesDrawMessageLogAt(uint16_t log_x, uint16_t log_w, uint16_t time_x, uint16_t text_x, uint16_t alarm_x, uint16_t alarm_w)
{
  Display_PagesDrawCardFrame(log_x, DISPLAY_DASH_Y, log_w, DISPLAY_DASH_H);
  Display_PagesDrawCardTitle(log_x, DISPLAY_DASH_Y, log_w, DISPLAY_TEXT_MESSAGE_LOG);
  Display_PagesDrawMessageLogAlarmStatusAt(alarm_x, DISPLAY_MSGLOG_ALARM_Y, alarm_w, DISPLAY_MSGLOG_ALARM_H);

  Display_PagesDrawMessageLogBodyAt(log_x, log_w, time_x, text_x, alarm_x, alarm_w);
}

static void Display_PagesDrawMessageLog(void)
{
  Display_PagesDrawMessageLogAt(DISPLAY_DASH_LOG_X, DISPLAY_DASH_LOG_W, DISPLAY_MSGLOG_TIME_X, DISPLAY_MSGLOG_TEXT_X, DISPLAY_MSGLOG_ALARM_X, DISPLAY_MSGLOG_ALARM_W);
}

static void Display_PagesDrawMotorMessageLog(void)
{
  Display_PagesDrawMessageLogAt(DISPLAY_MOTOR_LOG_X, DISPLAY_MOTOR_LOG_W, DISPLAY_MOTOR_MSGLOG_TIME_X, DISPLAY_MOTOR_MSGLOG_TEXT_X, DISPLAY_MOTOR_MSGLOG_ALARM_X, DISPLAY_MOTOR_MSGLOG_ALARM_W);
}

/*
 * 定位数据页：系统栏 + GNSS 定位面板 + 消息日志。
 */
static void Display_PagesDrawDataDashboardLayout(void)
{
  Display_PagesDrawSystemColumn();

  Display_PagesDrawPanelLabel(DISPLAY_DASH_GPS_X, DISPLAY_DASH_Y, DISPLAY_DASH_GPS_W, DISPLAY_DASH_H, DISPLAY_TITLE_GNSS);
  Display_PagesDrawDataGpsRow(290U, 122U, DISPLAY_TXT_STATUS);
  Display_PagesDrawDataGpsLabelRow(290U, 164U, DISPLAY_TXT_SAT_COUNT, 0);
  Display_PagesDrawDataGpsRow(290U, 206U, DISPLAY_TXT_LAT);
  Display_PagesDrawDataGpsRow(290U, 248U, DISPLAY_TXT_LON);
  Display_PagesDrawDataGpsRow(290U, 290U, DISPLAY_TXT_ALT);
  Display_PagesDrawDataGpsRow(290U, 332U, DISPLAY_TXT_HDOP);

  Display_PagesDrawMessageLog();
}

/*
 * 飞行数据页中间栏：电压/飞行时间/温度/湿度/气压。
 * 电压为主控电池电压(ADC1)，由飞机情况页迁移而来；日期/时间已移至标题栏。
 * 标签沿用系统内置中文位图，数值由各变量字段刷新绘制。
 */
static void Display_PagesDrawFlightMiddle(void)
{
  Display_PagesDrawPanelLabel(DISPLAY_DASH_GPS_X, DISPLAY_DASH_Y, DISPLAY_DASH_GPS_W, DISPLAY_DASH_H, DISPLAY_TITLE_FLIGHT);
  Display_PagesDrawBatteryRow(290U, 118U, 0U, 0U);
  Display_PagesDrawBatteryRow(290U, 154U, 0U, 1U);
  Display_PagesDrawBatteryRow(290U, 190U, 1U, 0U);
  Display_PagesDrawBatteryRow(290U, 226U, 1U, 1U);
  Display_PagesDrawDataGpsRow(290U, 262U, DISPLAY_TXT_UPTIME);
  Display_PagesDrawDataGpsRow(290U, 298U, DISPLAY_TXT_TEMP);
  Display_PagesDrawDataGpsRow(290U, 334U, DISPLAY_TXT_HUM);
  Display_PagesDrawDataGpsRow(290U, 370U, DISPLAY_TXT_PRESS);
}

static void Display_PagesDrawFlightLayout(void)
{
  Display_PagesDrawSystemColumn();
  Display_PagesDrawFlightMiddle();
  Display_PagesDrawMessageLog();
}

/*
 * 飞机情况页中间栏的一行普通标签 + 下划线。
 */
static void Display_PagesDrawAircraftLabelRow(uint16_t y, Display_TextLabel_t label)
{
  (void)Display_TextDrawLabel(280U, y, label, DISPLAY_THEME_TEXT);
  (void)Display_GfxDrawHLine(272U, (uint16_t)(y + 26U), 244U, DISPLAY_THEME_DIVIDER);
}

/*
 * 飞机情况页中间栏：横滚/俯仰/偏航/发送/接收。
 * 电压/电量行已删除（电压迁至飞行数据页）。
 */
static void Display_PagesDrawAircraftMiddle(void)
{
  Display_PagesDrawPanelLabel(DISPLAY_DASH_GPS_X, DISPLAY_DASH_Y, DISPLAY_DASH_GPS_W, DISPLAY_DASH_H, DISPLAY_TITLE_AIRCRAFT);
  Display_PagesDrawAircraftLabelRow(118U, DISPLAY_TXT_ROLL);  /* 横滚 */
  Display_PagesDrawAircraftLabelRow(150U, DISPLAY_TXT_PITCH); /* 俯仰 */
  Display_PagesDrawAircraftLabelRow(182U, DISPLAY_TXT_YAW);   /* 偏航 */
  Display_PagesDrawAircraftLabelRow(214U, DISPLAY_TXT_TX);    /* 发送计数 */
  Display_PagesDrawAircraftLabelRow(246U, DISPLAY_TXT_RX);    /* 接收计数 */
}

static void Display_PagesDrawAircraftLayout(void)
{
  Display_PagesDrawSystemColumn();
  Display_PagesDrawAircraftMiddle();
  Display_PagesDrawMessageLog();
}

static void Display_PagesDrawMotorSliderStatic(uint16_t track_x, char index)
{
  uint16_t cx = (uint16_t)(track_x + (DISPLAY_MOTOR_TRACK_W / 2U));
  uint16_t motor_w = Display_TextGetLabelWidth(DISPLAY_TXT_MOTOR);
  uint16_t total   = (uint16_t)(motor_w + 14U);
  uint16_t label_x = (cx > (total / 2U)) ? (uint16_t)(cx - (total / 2U)) : track_x;

  (void)Display_TextDrawLabel(label_x, DISPLAY_MOTOR_LABEL_Y, DISPLAY_TXT_MOTOR, DISPLAY_THEME_TEXT);
  (void)Display_GfxDrawChar((uint16_t)(label_x + motor_w + 2U), DISPLAY_MOTOR_LABEL_Y, index, DISPLAY_THEME_TEXT, 2U);
  (void)Display_GfxFillRect(track_x, DISPLAY_MOTOR_TRACK_TOP_Y, DISPLAY_MOTOR_TRACK_W, DISPLAY_MOTOR_TRACK_H, DISPLAY_THEME_PANEL_HI);
  (void)Display_GfxDrawRect((uint16_t)(track_x - 1U), (uint16_t)(DISPLAY_MOTOR_TRACK_TOP_Y - 1U), (uint16_t)(DISPLAY_MOTOR_TRACK_W + 2U), (uint16_t)(DISPLAY_MOTOR_TRACK_H + 2U), DISPLAY_THEME_BORDER);
}

static void Display_PagesDrawMotorLayout(void)
{
  uint16_t estop_text_x = (uint16_t)(DISPLAY_MOTOR_ESTOP_X + ((DISPLAY_MOTOR_ESTOP_W - 64U) / 2U));
  uint16_t estop_text_y = (uint16_t)(DISPLAY_MOTOR_ESTOP_Y + ((DISPLAY_MOTOR_ESTOP_H - 32U) / 2U));

  Display_PagesDrawSystemColumnAt(DISPLAY_MOTOR_STATUS_X, DISPLAY_MOTOR_STATUS_W);

  Display_PagesDrawCardFrame(DISPLAY_MOTOR_PANEL_X, DISPLAY_DASH_Y, DISPLAY_MOTOR_PANEL_W, DISPLAY_DASH_H);
  Display_PagesDrawCardTitle(DISPLAY_MOTOR_PANEL_X, DISPLAY_DASH_Y, DISPLAY_MOTOR_PANEL_W, DISPLAY_TEXT_MOTOR_PWM);

  Display_PagesDrawMotorSliderStatic(DISPLAY_MOTOR_TRACK1_X, '1');
  Display_PagesDrawMotorSliderStatic(DISPLAY_MOTOR_TRACK2_X, '2');
  Display_PagesDrawMotorSliderStatic(DISPLAY_MOTOR_TRACK3_X, '3');
  Display_PagesDrawMotorSliderStatic(DISPLAY_MOTOR_TRACK4_X, '4');

  (void)Display_GfxFillRoundRect(DISPLAY_MOTOR_ESTOP_X, DISPLAY_MOTOR_ESTOP_Y, DISPLAY_MOTOR_ESTOP_W, DISPLAY_MOTOR_ESTOP_H, 12U, DISPLAY_GFX_COLOR_RED);
  (void)Display_TextDrawEstop(estop_text_x, estop_text_y, DISPLAY_GFX_COLOR_WHITE);

  Display_PagesDrawMotorMessageLog();
}

/*
 * 绘制上电自检页面的固定布局。
 */
static void Display_PagesDrawSelfCheckLayout(void)
{
  const Display_TextLabel_t labels[] = {DISPLAY_TEXT_SELF_GNSS, DISPLAY_TEXT_SELF_ATTITUDE, DISPLAY_TEXT_SELF_ENV, DISPLAY_TEXT_SELF_LORA, DISPLAY_TEXT_SELF_STORAGE, DISPLAY_TEXT_COUNT, DISPLAY_TEXT_COUNT, DISPLAY_TEXT_COUNT, DISPLAY_TEXT_COUNT};
  const char motor_index[]           = {0, 0, 0, 0, 0, '1', '2', '3', '4'};
  static const uint16_t col_x[3]     = {8U, 164U, 320U};
  static const uint16_t row_y[3]     = {76U, 188U, 300U};
  uint16_t cell_w                    = 148U;
  uint16_t cell_h                    = 104U;
  uint16_t r;
  uint16_t c;
  uint16_t i;

  for (r = 0U; r < 3U; r++) {
    for (c = 0U; c < 3U; c++) {
      uint16_t label_w;
      uint16_t label_x;

      i = r * 3U + c;
      Display_PagesDrawCardFrame(col_x[c], row_y[r], cell_w, cell_h);
      if (motor_index[i] != 0) {
        Display_PagesDrawSelfMotorLabel(col_x[c], (uint16_t)(row_y[r] + 28U), cell_w, motor_index[i]);
      } else {
        label_w = Display_TextGetLabelWidth(labels[i]);
        label_x = (label_w < cell_w) ? (uint16_t)(col_x[c] + ((cell_w - label_w) / 2U)) : col_x[c];
        (void)Display_TextDrawLabel(label_x, (uint16_t)(row_y[r] + 28U), labels[i], DISPLAY_THEME_TEXT);
      }
    }
  }

  Display_PagesDrawCardFrame(480U, 82U, 312U, 310U);
  (void)Display_GfxFillRect(492U, 90U, 4U, 14U, DISPLAY_GFX_COLOR_BLUE);
  (void)Display_TextDrawLabelBold(502U, 94U, DISPLAY_TEXT_ERROR_CODE, DISPLAY_GFX_COLOR_BLUE);
  (void)Display_GfxDrawHLine(480U, 118U, 312U, DISPLAY_GFX_COLOR_CARD_DIVIDER);
  (void)Display_TextDrawLabel(492U, 136U, DISPLAY_TEXT_CODE, DISPLAY_THEME_TEXT);
  (void)Display_TextDrawLabel(578U, 136U, DISPLAY_TEXT_MODULE, DISPLAY_THEME_TEXT);
  (void)Display_TextDrawLabel(684U, 136U, DISPLAY_TEXT_REASON, DISPLAY_THEME_TEXT);
  (void)Display_GfxDrawHLine(480U, 152U, 312U, DISPLAY_GFX_COLOR_CARD_DIVIDER);
  (void)Display_TextDrawLabel(604U, 170U, DISPLAY_TEXT_FAILED_MODULES, DISPLAY_GFX_COLOR_GRAY);
}

/*
 * 根据运行期告警码获取告警原因文本。
 */
/* Alarm page fixed layout. */
static void Display_PagesDrawAlarmTitle(uint16_t x, uint16_t y)
{
  (void)Display_TextDrawLabelBold(x, y, DISPLAY_TEXT_ALARM_SUMMARY, DISPLAY_GFX_COLOR_RED);
}

static void Display_PagesDrawAlarmHeaderLabel(uint16_t x, uint16_t y, uint16_t width, uint16_t height, Display_TextLabel_t label)
{
  uint16_t label_w = Display_TextGetLabelWidth(label);
  uint16_t label_x = (label_w < width) ? (uint16_t)(x + ((width - label_w) / 2U)) : x;
  uint16_t label_y = (uint16_t)(y + ((height - 20U) / 2U));

  (void)Display_TextDrawLabel(label_x, label_y, label, DISPLAY_GFX_COLOR_WHITE);
}

static uint16_t Display_PagesGetAlarmRowFill(uint16_t row)
{
  return ((row & 0x01U) == 0U) ? DISPLAY_THEME_PANEL : DISPLAY_ALARM_COLOR_ROW_ALT;
}

static void Display_PagesDrawAlarmGridLines(void)
{
  uint16_t i;

  (void)Display_GfxDrawRect(DISPLAY_ALARM_TABLE_X, DISPLAY_ALARM_TABLE_Y, DISPLAY_ALARM_TABLE_W, DISPLAY_ALARM_TABLE_H, DISPLAY_ALARM_COLOR_GRID);
  (void)Display_GfxDrawVLine(DISPLAY_ALARM_CODE_X, DISPLAY_ALARM_TABLE_Y, DISPLAY_ALARM_TABLE_H, DISPLAY_ALARM_COLOR_GRID);
  (void)Display_GfxDrawVLine(DISPLAY_ALARM_MODULE_X, DISPLAY_ALARM_TABLE_Y, DISPLAY_ALARM_TABLE_H, DISPLAY_ALARM_COLOR_GRID);
  (void)Display_GfxDrawHLine(DISPLAY_ALARM_TABLE_X, DISPLAY_ALARM_BODY_Y, DISPLAY_ALARM_TABLE_W, DISPLAY_ALARM_COLOR_GRID);

  for (i = 1U; i <= DISPLAY_ALARM_ROW_COUNT; i++) { (void)Display_GfxDrawHLine(DISPLAY_ALARM_TABLE_X, (uint16_t)(DISPLAY_ALARM_BODY_Y + (i * DISPLAY_ALARM_ROW_H)), DISPLAY_ALARM_TABLE_W, DISPLAY_ALARM_COLOR_GRID); }
}

static void Display_PagesDrawAlarmTable(void)
{
  uint16_t row;

  (void)Display_GfxFillRect(DISPLAY_ALARM_TABLE_X, DISPLAY_ALARM_TABLE_Y, DISPLAY_ALARM_TABLE_W, DISPLAY_ALARM_TABLE_H, DISPLAY_THEME_PANEL);
  (void)Display_GfxFillRect(DISPLAY_ALARM_TABLE_X, DISPLAY_ALARM_TABLE_Y, DISPLAY_ALARM_TABLE_W, DISPLAY_ALARM_HEADER_H, DISPLAY_ALARM_COLOR_HEADER);

  for (row = 0U; row < DISPLAY_ALARM_ROW_COUNT; row++) { (void)Display_GfxFillRect(DISPLAY_ALARM_TABLE_X, (uint16_t)(DISPLAY_ALARM_BODY_Y + (row * DISPLAY_ALARM_ROW_H)), DISPLAY_ALARM_TABLE_W, DISPLAY_ALARM_ROW_H, Display_PagesGetAlarmRowFill(row)); }

  Display_PagesDrawAlarmHeaderLabel(DISPLAY_ALARM_TABLE_X, DISPLAY_ALARM_TABLE_Y, (uint16_t)(DISPLAY_ALARM_CODE_X - DISPLAY_ALARM_TABLE_X), DISPLAY_ALARM_HEADER_H, DISPLAY_TEXT_CODE);
  Display_PagesDrawAlarmHeaderLabel(DISPLAY_ALARM_CODE_X, DISPLAY_ALARM_TABLE_Y, (uint16_t)(DISPLAY_ALARM_MODULE_X - DISPLAY_ALARM_CODE_X), DISPLAY_ALARM_HEADER_H, DISPLAY_TEXT_MODULE);
  Display_PagesDrawAlarmHeaderLabel(DISPLAY_ALARM_MODULE_X, DISPLAY_ALARM_TABLE_Y, (uint16_t)((DISPLAY_ALARM_TABLE_X + DISPLAY_ALARM_TABLE_W) - DISPLAY_ALARM_MODULE_X), DISPLAY_ALARM_HEADER_H, DISPLAY_TEXT_REASON);
  Display_PagesDrawAlarmGridLines();
}

static void Display_PagesDrawAlarmStaticLayout(void)
{
  Display_PagesDrawAlarmTable();
  Display_PagesDrawAlarmTitle(DISPLAY_ALARM_TITLE_X, DISPLAY_ALARM_TITLE_Y);
}

/* Alarm code reason text. */
/* ===== 告警页中文标签字模(16x16 宋体, 由 字模.txt 生成, 多字逐行交错) ===== */
typedef struct { uint16_t width; uint16_t bpr; const uint8_t *data; } Display_AlarmCn_t;

static const uint8_t acn_mod_gnss[] = { /* 定位 */
  0x02U, 0x00U, 0x08U, 0x80U,
  0x01U, 0x00U, 0x08U, 0x40U,
  0x7FU, 0xFEU, 0x08U, 0x40U,
  0x40U, 0x02U, 0x10U, 0x00U,
  0x80U, 0x04U, 0x17U, 0xFCU,
  0x00U, 0x00U, 0x30U, 0x00U,
  0x3FU, 0xF8U, 0x30U, 0x08U,
  0x01U, 0x00U, 0x52U, 0x08U,
  0x01U, 0x00U, 0x92U, 0x08U,
  0x11U, 0x00U, 0x11U, 0x10U,
  0x11U, 0xF8U, 0x11U, 0x10U,
  0x11U, 0x00U, 0x11U, 0x10U,
  0x11U, 0x00U, 0x11U, 0x20U,
  0x29U, 0x00U, 0x10U, 0x20U,
  0x47U, 0xFEU, 0x1FU, 0xFEU,
  0x80U, 0x00U, 0x10U, 0x00U,
};
static const Display_AlarmCn_t s_acn_mod_gnss = {32U, 4U, acn_mod_gnss};

static const uint8_t acn_mod_imu[] = { /* 姿态 */
  0x40U, 0x80U, 0x01U, 0x00U,
  0x20U, 0x80U, 0x01U, 0x00U,
  0x09U, 0xFCU, 0x7FU, 0xFCU,
  0x12U, 0x04U, 0x01U, 0x00U,
  0x24U, 0x48U, 0x02U, 0x80U,
  0xE0U, 0x40U, 0x04U, 0x40U,
  0x20U, 0xA0U, 0x0AU, 0x20U,
  0x23U, 0x18U, 0x31U, 0x18U,
  0x2CU, 0x06U, 0xC0U, 0x06U,
  0x04U, 0x00U, 0x01U, 0x00U,
  0xFFU, 0xFEU, 0x08U, 0x88U,
  0x08U, 0x20U, 0x48U, 0x84U,
  0x1CU, 0x40U, 0x48U, 0x12U,
  0x03U, 0x80U, 0x48U, 0x12U,
  0x0CU, 0x70U, 0x87U, 0xF0U,
  0x70U, 0x08U, 0x00U, 0x00U,
};
static const Display_AlarmCn_t s_acn_mod_imu = {32U, 4U, acn_mod_imu};

static const uint8_t acn_mod_baro[] = { /* 环境 */
  0x00U, 0x00U, 0x20U, 0x80U,
  0x00U, 0x00U, 0x20U, 0x40U,
  0xFDU, 0xFEU, 0x23U, 0xF8U,
  0x10U, 0x10U, 0x21U, 0x10U,
  0x10U, 0x10U, 0xF8U, 0xA0U,
  0x10U, 0x20U, 0x27U, 0xFEU,
  0x10U, 0x20U, 0x20U, 0x00U,
  0x7CU, 0x68U, 0x23U, 0xF8U,
  0x10U, 0xA4U, 0x22U, 0x08U,
  0x11U, 0x22U, 0x23U, 0xF8U,
  0x12U, 0x22U, 0x22U, 0x08U,
  0x10U, 0x20U, 0x3BU, 0xF8U,
  0x1CU, 0x20U, 0xE1U, 0x20U,
  0xE0U, 0x20U, 0x41U, 0x22U,
  0x40U, 0x20U, 0x02U, 0x22U,
  0x00U, 0x20U, 0x0CU, 0x1EU,
};
static const Display_AlarmCn_t s_acn_mod_baro = {32U, 4U, acn_mod_baro};

static const uint8_t acn_mod_power[] = { /* 电源 */
  0x01U, 0x00U, 0x00U, 0x00U,
  0x01U, 0x00U, 0x27U, 0xFEU,
  0x01U, 0x00U, 0x14U, 0x20U,
  0x3FU, 0xF8U, 0x14U, 0x40U,
  0x21U, 0x08U, 0x85U, 0xFCU,
  0x21U, 0x08U, 0x45U, 0x04U,
  0x21U, 0x08U, 0x45U, 0xFCU,
  0x3FU, 0xF8U, 0x15U, 0x04U,
  0x21U, 0x08U, 0x15U, 0xFCU,
  0x21U, 0x08U, 0x25U, 0x24U,
  0x21U, 0x08U, 0xE4U, 0x20U,
  0x3FU, 0xF8U, 0x24U, 0xA8U,
  0x21U, 0x0AU, 0x29U, 0x24U,
  0x01U, 0x02U, 0x2AU, 0x22U,
  0x01U, 0x02U, 0x30U, 0xA0U,
  0x00U, 0xFEU, 0x00U, 0x40U,
};
static const Display_AlarmCn_t s_acn_mod_power = {32U, 4U, acn_mod_power};

static const uint8_t acn_mod_comm[] = { /* 通信 */
  0x00U, 0x00U, 0x08U, 0x40U,
  0x47U, 0xF8U, 0x08U, 0x20U,
  0x20U, 0x10U, 0x0BU, 0xFEU,
  0x21U, 0xA0U, 0x10U, 0x00U,
  0x00U, 0x40U, 0x10U, 0x00U,
  0x07U, 0xFCU, 0x31U, 0xFCU,
  0xE4U, 0x44U, 0x30U, 0x00U,
  0x24U, 0x44U, 0x50U, 0x00U,
  0x27U, 0xFCU, 0x91U, 0xFCU,
  0x24U, 0x44U, 0x10U, 0x00U,
  0x24U, 0x44U, 0x10U, 0x00U,
  0x27U, 0xFCU, 0x11U, 0xFCU,
  0x24U, 0x44U, 0x11U, 0x04U,
  0x24U, 0x54U, 0x11U, 0x04U,
  0x54U, 0x08U, 0x11U, 0xFCU,
  0x8FU, 0xFEU, 0x11U, 0x04U,
};
static const Display_AlarmCn_t s_acn_mod_comm = {32U, 4U, acn_mod_comm};

static const uint8_t acn_mod_storage[] = { /* 存储 */
  0x04U, 0x00U, 0x10U, 0x20U,
  0x04U, 0x00U, 0x10U, 0x20U,
  0xFFU, 0xFEU, 0x18U, 0xFAU,
  0x08U, 0x00U, 0x24U, 0x24U,
  0x08U, 0x00U, 0x24U, 0x28U,
  0x13U, 0xF8U, 0x61U, 0xFEU,
  0x10U, 0x10U, 0x60U, 0x20U,
  0x30U, 0x20U, 0xBCU, 0x40U,
  0x50U, 0x40U, 0x24U, 0xFCU,
  0x97U, 0xFEU, 0x25U, 0x44U,
  0x10U, 0x40U, 0x26U, 0x44U,
  0x10U, 0x40U, 0x24U, 0x7CU,
  0x10U, 0x40U, 0x25U, 0x44U,
  0x10U, 0x40U, 0x26U, 0x44U,
  0x11U, 0x40U, 0x24U, 0x7CU,
  0x10U, 0x80U, 0x20U, 0x44U,
};
static const Display_AlarmCn_t s_acn_mod_storage = {32U, 4U, acn_mod_storage};

static const uint8_t acn_mod_remote[] = { /* 远程 */
  0x00U, 0x00U, 0x08U, 0x00U,
  0x23U, 0xF8U, 0x1DU, 0xFCU,
  0x10U, 0x00U, 0xF1U, 0x04U,
  0x10U, 0x00U, 0x11U, 0x04U,
  0x00U, 0x00U, 0x11U, 0x04U,
  0x07U, 0xFCU, 0xFDU, 0xFCU,
  0xF1U, 0x20U, 0x10U, 0x00U,
  0x11U, 0x20U, 0x30U, 0x00U,
  0x11U, 0x20U, 0x39U, 0xFEU,
  0x11U, 0x20U, 0x54U, 0x20U,
  0x11U, 0x24U, 0x54U, 0x20U,
  0x12U, 0x24U, 0x91U, 0xFCU,
  0x12U, 0x24U, 0x10U, 0x20U,
  0x14U, 0x1CU, 0x10U, 0x20U,
  0x28U, 0x00U, 0x13U, 0xFEU,
  0x47U, 0xFEU, 0x10U, 0x00U,
};
static const Display_AlarmCn_t s_acn_mod_remote = {32U, 4U, acn_mod_remote};

static const uint8_t acn_mod_display[] = { /* 显示 */
  0x00U, 0x00U, 0x00U, 0x00U,
  0x1FU, 0xF0U, 0x3FU, 0xF8U,
  0x10U, 0x10U, 0x00U, 0x00U,
  0x10U, 0x10U, 0x00U, 0x00U,
  0x1FU, 0xF0U, 0x00U, 0x00U,
  0x10U, 0x10U, 0x00U, 0x00U,
  0x10U, 0x10U, 0xFFU, 0xFEU,
  0x1FU, 0xF0U, 0x01U, 0x00U,
  0x04U, 0x40U, 0x01U, 0x00U,
  0x44U, 0x44U, 0x11U, 0x10U,
  0x24U, 0x44U, 0x11U, 0x08U,
  0x14U, 0x48U, 0x21U, 0x04U,
  0x14U, 0x50U, 0x41U, 0x02U,
  0x04U, 0x40U, 0x81U, 0x02U,
  0xFFU, 0xFEU, 0x05U, 0x00U,
  0x00U, 0x00U, 0x02U, 0x00U,
};
static const Display_AlarmCn_t s_acn_mod_display = {32U, 4U, acn_mod_display};

static const uint8_t acn_mod_control[] = { /* 控制 */
  0x10U, 0x40U, 0x04U, 0x04U,
  0x10U, 0x20U, 0x24U, 0x04U,
  0x10U, 0x20U, 0x24U, 0x04U,
  0x13U, 0xFEU, 0x3FU, 0xA4U,
  0xFAU, 0x02U, 0x44U, 0x24U,
  0x14U, 0x94U, 0x04U, 0x24U,
  0x11U, 0x08U, 0xFFU, 0xE4U,
  0x1AU, 0x04U, 0x04U, 0x24U,
  0x30U, 0x00U, 0x04U, 0x24U,
  0xD1U, 0xFCU, 0x3FU, 0xA4U,
  0x10U, 0x20U, 0x24U, 0xA4U,
  0x10U, 0x20U, 0x24U, 0xA4U,
  0x10U, 0x20U, 0x26U, 0x84U,
  0x10U, 0x20U, 0x25U, 0x04U,
  0x57U, 0xFEU, 0x04U, 0x14U,
  0x20U, 0x00U, 0x04U, 0x08U,
};
static const Display_AlarmCn_t s_acn_mod_control = {32U, 4U, acn_mod_control};

static const uint8_t acn_mod_alarm[] = { /* 告警 */
  0x01U, 0x00U, 0x24U, 0x20U,
  0x11U, 0x00U, 0xFFU, 0x20U,
  0x11U, 0x00U, 0x24U, 0x7EU,
  0x1FU, 0xF8U, 0x7EU, 0xC4U,
  0x21U, 0x00U, 0x82U, 0x28U,
  0x41U, 0x00U, 0x7AU, 0x10U,
  0x01U, 0x00U, 0x4AU, 0x28U,
  0xFFU, 0xFEU, 0x7AU, 0xC6U,
  0x00U, 0x00U, 0x05U, 0x00U,
  0x00U, 0x00U, 0xFFU, 0xFEU,
  0x1FU, 0xF0U, 0x00U, 0x00U,
  0x10U, 0x10U, 0x3FU, 0xF8U,
  0x10U, 0x10U, 0x00U, 0x00U,
  0x10U, 0x10U, 0x3FU, 0xF8U,
  0x1FU, 0xF0U, 0x20U, 0x08U,
  0x10U, 0x10U, 0x3FU, 0xF8U,
};
static const Display_AlarmCn_t s_acn_mod_alarm = {32U, 4U, acn_mod_alarm};

static const uint8_t acn_mod_system[] = { /* 系统 */
  0x00U, 0xF8U, 0x10U, 0x40U,
  0x3FU, 0x00U, 0x10U, 0x20U,
  0x04U, 0x00U, 0x20U, 0x20U,
  0x08U, 0x20U, 0x23U, 0xFEU,
  0x10U, 0x40U, 0x48U, 0x40U,
  0x3FU, 0x80U, 0xF8U, 0x88U,
  0x01U, 0x00U, 0x11U, 0x04U,
  0x06U, 0x10U, 0x23U, 0xFEU,
  0x18U, 0x08U, 0x40U, 0x92U,
  0x7FU, 0xFCU, 0xF8U, 0x90U,
  0x01U, 0x04U, 0x40U, 0x90U,
  0x09U, 0x20U, 0x00U, 0x90U,
  0x11U, 0x10U, 0x19U, 0x12U,
  0x21U, 0x08U, 0xE1U, 0x12U,
  0x45U, 0x04U, 0x42U, 0x0EU,
  0x02U, 0x00U, 0x04U, 0x00U,
};
static const Display_AlarmCn_t s_acn_mod_system = {32U, 4U, acn_mod_system};

static const uint8_t acn_mod_est[] = { /* 估计 */
  0x08U, 0x40U, 0x00U, 0x40U,
  0x08U, 0x40U, 0x20U, 0x40U,
  0x08U, 0x40U, 0x10U, 0x40U,
  0x10U, 0x40U, 0x10U, 0x40U,
  0x17U, 0xFEU, 0x00U, 0x40U,
  0x30U, 0x40U, 0x00U, 0x40U,
  0x30U, 0x40U, 0xF7U, 0xFEU,
  0x50U, 0x40U, 0x10U, 0x40U,
  0x93U, 0xF8U, 0x10U, 0x40U,
  0x12U, 0x08U, 0x10U, 0x40U,
  0x12U, 0x08U, 0x10U, 0x40U,
  0x12U, 0x08U, 0x10U, 0x40U,
  0x12U, 0x08U, 0x14U, 0x40U,
  0x12U, 0x08U, 0x18U, 0x40U,
  0x13U, 0xF8U, 0x10U, 0x40U,
  0x12U, 0x08U, 0x00U, 0x40U,
};
static const Display_AlarmCn_t s_acn_mod_est = {32U, 4U, acn_mod_est};

static const uint8_t acn_mod_business[] = { /* 业务 */
  0x04U, 0x40U, 0x04U, 0x00U,
  0x04U, 0x40U, 0x04U, 0x00U,
  0x04U, 0x40U, 0x0FU, 0xF0U,
  0x04U, 0x40U, 0x18U, 0x20U,
  0x44U, 0x44U, 0x24U, 0xC0U,
  0x24U, 0x44U, 0x03U, 0x00U,
  0x24U, 0x48U, 0x0CU, 0xC0U,
  0x14U, 0x48U, 0x32U, 0x30U,
  0x14U, 0x50U, 0xC2U, 0x0EU,
  0x14U, 0x60U, 0x1FU, 0xF0U,
  0x04U, 0x40U, 0x02U, 0x10U,
  0x04U, 0x40U, 0x04U, 0x10U,
  0x04U, 0x40U, 0x04U, 0x10U,
  0x04U, 0x40U, 0x08U, 0x10U,
  0xFFU, 0xFEU, 0x10U, 0xA0U,
  0x00U, 0x00U, 0x20U, 0x40U,
};
static const Display_AlarmCn_t s_acn_mod_business = {32U, 4U, acn_mod_business};

static const uint8_t acn_mod_sensor[] = { /* 传感 */
  0x08U, 0x40U, 0x00U, 0x28U,
  0x08U, 0x40U, 0x00U, 0x24U,
  0x08U, 0x40U, 0x3FU, 0xFEU,
  0x13U, 0xF8U, 0x20U, 0x20U,
  0x10U, 0x40U, 0x2FU, 0xA4U,
  0x30U, 0x80U, 0x20U, 0x24U,
  0x37U, 0xFEU, 0x2FU, 0xA8U,
  0x50U, 0x80U, 0x28U, 0x98U,
  0x91U, 0x00U, 0x28U, 0x92U,
  0x13U, 0xF8U, 0x4FU, 0xAAU,
  0x10U, 0x08U, 0x40U, 0x46U,
  0x11U, 0x10U, 0x80U, 0x82U,
  0x10U, 0xA0U, 0x01U, 0x00U,
  0x10U, 0x40U, 0x48U, 0x84U,
  0x10U, 0x20U, 0x48U, 0x12U,
  0x10U, 0x20U, 0x87U, 0xF2U,
};
static const Display_AlarmCn_t s_acn_mod_sensor = {32U, 4U, acn_mod_sensor};

static const uint8_t acn_mod_proto[] = { /* 协议 */
  0x20U, 0x80U, 0x00U, 0x80U,
  0x20U, 0x80U, 0x20U, 0x48U,
  0x20U, 0x80U, 0x12U, 0x48U,
  0x20U, 0x80U, 0x12U, 0x08U,
  0xFBU, 0xF0U, 0x02U, 0x08U,
  0x20U, 0x90U, 0x01U, 0x10U,
  0x20U, 0x90U, 0xF1U, 0x10U,
  0x22U, 0x98U, 0x11U, 0x10U,
  0x22U, 0x94U, 0x10U, 0xA0U,
  0x24U, 0x92U, 0x10U, 0xA0U,
  0x28U, 0x92U, 0x10U, 0x40U,
  0x20U, 0x90U, 0x14U, 0x40U,
  0x21U, 0x10U, 0x18U, 0xA0U,
  0x21U, 0x10U, 0x11U, 0x10U,
  0x22U, 0x50U, 0x02U, 0x08U,
  0x24U, 0x20U, 0x0CU, 0x06U,
};
static const Display_AlarmCn_t s_acn_mod_proto = {32U, 4U, acn_mod_proto};

static const uint8_t acn_mod_app[] = { /* 应用 */
  0x01U, 0x00U, 0x00U, 0x00U,
  0x00U, 0x80U, 0x3FU, 0xF8U,
  0x3FU, 0xFEU, 0x21U, 0x08U,
  0x20U, 0x00U, 0x21U, 0x08U,
  0x20U, 0x00U, 0x21U, 0x08U,
  0x21U, 0x04U, 0x3FU, 0xF8U,
  0x28U, 0x84U, 0x21U, 0x08U,
  0x24U, 0x84U, 0x21U, 0x08U,
  0x24U, 0x48U, 0x21U, 0x08U,
  0x22U, 0x48U, 0x3FU, 0xF8U,
  0x22U, 0x10U, 0x21U, 0x08U,
  0x22U, 0x10U, 0x21U, 0x08U,
  0x40U, 0x20U, 0x21U, 0x08U,
  0x40U, 0x40U, 0x41U, 0x08U,
  0x9FU, 0xFEU, 0x41U, 0x28U,
  0x00U, 0x00U, 0x80U, 0x10U,
};
static const Display_AlarmCn_t s_acn_mod_app = {32U, 4U, acn_mod_app};

static const uint8_t acn_rsn_selfcheck[] = { /* 自检失败 */
  0x01U, 0x00U, 0x10U, 0x40U, 0x01U, 0x00U, 0x00U, 0x40U,
  0x02U, 0x00U, 0x10U, 0x40U, 0x11U, 0x00U, 0x7CU, 0x40U,
  0x04U, 0x00U, 0x10U, 0xA0U, 0x11U, 0x00U, 0x44U, 0x40U,
  0x1FU, 0xF0U, 0x10U, 0xA0U, 0x11U, 0x00U, 0x54U, 0x80U,
  0x10U, 0x10U, 0xFDU, 0x10U, 0x3FU, 0xF8U, 0x54U, 0xFEU,
  0x10U, 0x10U, 0x12U, 0x08U, 0x21U, 0x00U, 0x55U, 0x08U,
  0x10U, 0x10U, 0x35U, 0xF6U, 0x41U, 0x00U, 0x56U, 0x88U,
  0x1FU, 0xF0U, 0x38U, 0x00U, 0x01U, 0x00U, 0x54U, 0x88U,
  0x10U, 0x10U, 0x54U, 0x88U, 0xFFU, 0xFEU, 0x54U, 0x88U,
  0x10U, 0x10U, 0x50U, 0x48U, 0x02U, 0x80U, 0x54U, 0x50U,
  0x1FU, 0xF0U, 0x92U, 0x48U, 0x04U, 0x40U, 0x54U, 0x50U,
  0x10U, 0x10U, 0x11U, 0x50U, 0x04U, 0x40U, 0x10U, 0x20U,
  0x10U, 0x10U, 0x11U, 0x10U, 0x08U, 0x20U, 0x28U, 0x50U,
  0x10U, 0x10U, 0x10U, 0x20U, 0x10U, 0x10U, 0x24U, 0x88U,
  0x1FU, 0xF0U, 0x17U, 0xFEU, 0x20U, 0x08U, 0x45U, 0x04U,
  0x10U, 0x10U, 0x10U, 0x00U, 0xC0U, 0x06U, 0x82U, 0x02U,
};
static const Display_AlarmCn_t s_acn_rsn_selfcheck = {64U, 8U, acn_rsn_selfcheck};

static const uint8_t acn_rsn_heap[] = { /* 内存不足 */
  0x01U, 0x00U, 0x04U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
  0x01U, 0x00U, 0x04U, 0x00U, 0x7FU, 0xFCU, 0x1FU, 0xF0U,
  0x01U, 0x00U, 0xFFU, 0xFEU, 0x00U, 0x80U, 0x10U, 0x10U,
  0x7FU, 0xFCU, 0x08U, 0x00U, 0x00U, 0x80U, 0x10U, 0x10U,
  0x41U, 0x04U, 0x08U, 0x00U, 0x01U, 0x00U, 0x10U, 0x10U,
  0x41U, 0x04U, 0x13U, 0xF8U, 0x01U, 0x00U, 0x10U, 0x10U,
  0x41U, 0x04U, 0x10U, 0x10U, 0x03U, 0x40U, 0x1FU, 0xF0U,
  0x42U, 0x84U, 0x30U, 0x20U, 0x05U, 0x20U, 0x01U, 0x00U,
  0x42U, 0x44U, 0x50U, 0x40U, 0x09U, 0x10U, 0x01U, 0x00U,
  0x44U, 0x24U, 0x97U, 0xFEU, 0x11U, 0x08U, 0x11U, 0x00U,
  0x48U, 0x14U, 0x10U, 0x40U, 0x21U, 0x04U, 0x11U, 0xF8U,
  0x50U, 0x14U, 0x10U, 0x40U, 0x41U, 0x04U, 0x11U, 0x00U,
  0x40U, 0x04U, 0x10U, 0x40U, 0x81U, 0x00U, 0x29U, 0x00U,
  0x40U, 0x04U, 0x10U, 0x40U, 0x01U, 0x00U, 0x25U, 0x00U,
  0x40U, 0x14U, 0x11U, 0x40U, 0x01U, 0x00U, 0x43U, 0xFEU,
  0x40U, 0x08U, 0x10U, 0x80U, 0x01U, 0x00U, 0x80U, 0x00U,
};
static const Display_AlarmCn_t s_acn_rsn_heap = {64U, 8U, acn_rsn_heap};

static const uint8_t acn_rsn_stack[] = { /* 栈溢出 */
  0x10U, 0x50U, 0x02U, 0x08U, 0x01U, 0x00U,
  0x10U, 0x48U, 0x21U, 0x08U, 0x01U, 0x00U,
  0x10U, 0x40U, 0x11U, 0x10U, 0x21U, 0x08U,
  0x10U, 0x5CU, 0x10U, 0x00U, 0x21U, 0x08U,
  0xFDU, 0xE0U, 0x87U, 0xFCU, 0x21U, 0x08U,
  0x10U, 0x40U, 0x40U, 0x00U, 0x21U, 0x08U,
  0x30U, 0x5EU, 0x41U, 0x10U, 0x3FU, 0xF8U,
  0x39U, 0xE0U, 0x12U, 0x08U, 0x01U, 0x08U,
  0x54U, 0x44U, 0x14U, 0x04U, 0x01U, 0x00U,
  0x50U, 0x48U, 0x23U, 0xF8U, 0x01U, 0x00U,
  0x90U, 0x30U, 0xE2U, 0xA8U, 0x41U, 0x04U,
  0x10U, 0x22U, 0x22U, 0xA8U, 0x41U, 0x04U,
  0x10U, 0x52U, 0x22U, 0xA8U, 0x41U, 0x04U,
  0x10U, 0x8AU, 0x22U, 0xA8U, 0x41U, 0x04U,
  0x13U, 0x06U, 0x2FU, 0xFEU, 0x7FU, 0xFCU,
  0x10U, 0x02U, 0x00U, 0x00U, 0x00U, 0x04U,
};
static const Display_AlarmCn_t s_acn_rsn_stack = {48U, 6U, acn_rsn_stack};

static const uint8_t acn_rsn_tasklost[] = { /* 任务丢失 */
  0x08U, 0x10U, 0x04U, 0x00U, 0x00U, 0x10U, 0x01U, 0x00U,
  0x08U, 0x78U, 0x04U, 0x00U, 0x00U, 0xF8U, 0x11U, 0x00U,
  0x0BU, 0xC0U, 0x0FU, 0xF0U, 0x3FU, 0x00U, 0x11U, 0x00U,
  0x10U, 0x40U, 0x18U, 0x20U, 0x01U, 0x00U, 0x11U, 0x00U,
  0x10U, 0x40U, 0x24U, 0xC0U, 0x01U, 0x00U, 0x3FU, 0xF8U,
  0x30U, 0x40U, 0x03U, 0x00U, 0x3FU, 0xF8U, 0x21U, 0x00U,
  0x30U, 0x40U, 0x0CU, 0xC0U, 0x01U, 0x00U, 0x41U, 0x00U,
  0x5FU, 0xFEU, 0x32U, 0x30U, 0x01U, 0x00U, 0x01U, 0x00U,
  0x90U, 0x40U, 0xC2U, 0x0EU, 0x01U, 0x00U, 0xFFU, 0xFEU,
  0x10U, 0x40U, 0x1FU, 0xF0U, 0xFFU, 0xFEU, 0x02U, 0x80U,
  0x10U, 0x40U, 0x02U, 0x10U, 0x02U, 0x00U, 0x04U, 0x40U,
  0x10U, 0x40U, 0x04U, 0x10U, 0x04U, 0x00U, 0x04U, 0x40U,
  0x10U, 0x40U, 0x04U, 0x10U, 0x08U, 0x20U, 0x08U, 0x20U,
  0x10U, 0x40U, 0x08U, 0x10U, 0x10U, 0x10U, 0x10U, 0x10U,
  0x17U, 0xFCU, 0x10U, 0xA0U, 0x3FU, 0xF8U, 0x20U, 0x08U,
  0x10U, 0x00U, 0x20U, 0x40U, 0x10U, 0x08U, 0xC0U, 0x06U,
};
static const Display_AlarmCn_t s_acn_rsn_tasklost = {64U, 8U, acn_rsn_tasklost};

static const uint8_t acn_rsn_sensorinit[] = { /* 初始化失败 */
  0x20U, 0x00U, 0x10U, 0x20U, 0x08U, 0x80U, 0x01U, 0x00U, 0x00U, 0x40U,
  0x10U, 0x00U, 0x10U, 0x20U, 0x08U, 0x80U, 0x11U, 0x00U, 0x7CU, 0x40U,
  0x01U, 0xFCU, 0x10U, 0x20U, 0x08U, 0x84U, 0x11U, 0x00U, 0x44U, 0x40U,
  0xFCU, 0x44U, 0x10U, 0x40U, 0x10U, 0x88U, 0x11U, 0x00U, 0x54U, 0x80U,
  0x08U, 0x44U, 0xFCU, 0x48U, 0x10U, 0x90U, 0x3FU, 0xF8U, 0x54U, 0xFEU,
  0x10U, 0x44U, 0x24U, 0x84U, 0x30U, 0xA0U, 0x21U, 0x00U, 0x55U, 0x08U,
  0x10U, 0x44U, 0x25U, 0xFEU, 0x30U, 0xC0U, 0x41U, 0x00U, 0x56U, 0x88U,
  0x34U, 0x44U, 0x24U, 0x82U, 0x50U, 0x80U, 0x01U, 0x00U, 0x54U, 0x88U,
  0x58U, 0x44U, 0x24U, 0x00U, 0x91U, 0x80U, 0xFFU, 0xFEU, 0x54U, 0x88U,
  0x94U, 0x44U, 0x48U, 0xFCU, 0x12U, 0x80U, 0x02U, 0x80U, 0x54U, 0x50U,
  0x14U, 0x44U, 0x28U, 0x84U, 0x14U, 0x80U, 0x04U, 0x40U, 0x54U, 0x50U,
  0x10U, 0x84U, 0x10U, 0x84U, 0x10U, 0x82U, 0x04U, 0x40U, 0x10U, 0x20U,
  0x10U, 0x84U, 0x28U, 0x84U, 0x10U, 0x82U, 0x08U, 0x20U, 0x28U, 0x50U,
  0x11U, 0x04U, 0x44U, 0x84U, 0x10U, 0x82U, 0x10U, 0x10U, 0x24U, 0x88U,
  0x12U, 0x28U, 0x80U, 0xFCU, 0x10U, 0x7EU, 0x20U, 0x08U, 0x45U, 0x04U,
  0x14U, 0x10U, 0x00U, 0x84U, 0x10U, 0x00U, 0xC0U, 0x06U, 0x82U, 0x02U,
};
static const Display_AlarmCn_t s_acn_rsn_sensorinit = {80U, 10U, acn_rsn_sensorinit};

static const uint8_t acn_rsn_sensoroffline[] = { /* 传感离线 */
  0x08U, 0x40U, 0x00U, 0x28U, 0x02U, 0x00U, 0x10U, 0x50U,
  0x08U, 0x40U, 0x00U, 0x24U, 0x01U, 0x00U, 0x10U, 0x48U,
  0x08U, 0x40U, 0x3FU, 0xFEU, 0xFFU, 0xFEU, 0x20U, 0x40U,
  0x13U, 0xF8U, 0x20U, 0x20U, 0x00U, 0x00U, 0x24U, 0x5CU,
  0x10U, 0x40U, 0x2FU, 0xA4U, 0x14U, 0x50U, 0x45U, 0xE0U,
  0x30U, 0x80U, 0x20U, 0x24U, 0x13U, 0x90U, 0xF8U, 0x40U,
  0x37U, 0xFEU, 0x2FU, 0xA8U, 0x14U, 0x50U, 0x10U, 0x5EU,
  0x50U, 0x80U, 0x28U, 0x98U, 0x1FU, 0xF0U, 0x23U, 0xE0U,
  0x91U, 0x00U, 0x28U, 0x92U, 0x01U, 0x00U, 0x40U, 0x44U,
  0x13U, 0xF8U, 0x4FU, 0xAAU, 0x7FU, 0xFCU, 0xFCU, 0x48U,
  0x10U, 0x08U, 0x40U, 0x46U, 0x42U, 0x04U, 0x40U, 0x30U,
  0x11U, 0x10U, 0x80U, 0x82U, 0x44U, 0x44U, 0x00U, 0x22U,
  0x10U, 0xA0U, 0x01U, 0x00U, 0x4FU, 0xE4U, 0x1CU, 0x52U,
  0x10U, 0x40U, 0x48U, 0x84U, 0x44U, 0x24U, 0xE0U, 0x8AU,
  0x10U, 0x20U, 0x48U, 0x12U, 0x40U, 0x14U, 0x43U, 0x06U,
  0x10U, 0x20U, 0x87U, 0xF2U, 0x40U, 0x08U, 0x00U, 0x02U,
};
static const Display_AlarmCn_t s_acn_rsn_sensoroffline = {64U, 8U, acn_rsn_sensoroffline};

static const uint8_t acn_rsn_invalid[] = { /* 数据无效 */
  0x08U, 0x20U, 0x20U, 0x00U, 0x00U, 0x00U, 0x10U, 0x20U,
  0x49U, 0x20U, 0x23U, 0xFCU, 0x3FU, 0xF0U, 0x08U, 0x20U,
  0x2AU, 0x20U, 0x22U, 0x04U, 0x02U, 0x00U, 0x00U, 0x20U,
  0x08U, 0x3EU, 0x22U, 0x04U, 0x02U, 0x00U, 0xFFU, 0x3EU,
  0xFFU, 0x44U, 0xFBU, 0xFCU, 0x02U, 0x00U, 0x00U, 0x44U,
  0x2AU, 0x44U, 0x22U, 0x20U, 0x02U, 0x00U, 0x24U, 0x44U,
  0x49U, 0x44U, 0x22U, 0x20U, 0x7FU, 0xFCU, 0x42U, 0x44U,
  0x88U, 0xA4U, 0x2BU, 0xFEU, 0x04U, 0x80U, 0x81U, 0xA4U,
  0x10U, 0x28U, 0x32U, 0x20U, 0x04U, 0x80U, 0x24U, 0x28U,
  0xFEU, 0x28U, 0xE2U, 0x20U, 0x04U, 0x80U, 0x14U, 0x28U,
  0x22U, 0x10U, 0x22U, 0xFCU, 0x08U, 0x80U, 0x08U, 0x10U,
  0x42U, 0x10U, 0x22U, 0x84U, 0x08U, 0x80U, 0x14U, 0x10U,
  0x64U, 0x28U, 0x22U, 0x84U, 0x10U, 0x84U, 0x22U, 0x28U,
  0x18U, 0x28U, 0x24U, 0x84U, 0x20U, 0x84U, 0x42U, 0x48U,
  0x34U, 0x44U, 0xA4U, 0xFCU, 0x40U, 0x7CU, 0x80U, 0x84U,
  0xC2U, 0x82U, 0x48U, 0x84U, 0x80U, 0x00U, 0x01U, 0x02U,
};
static const Display_AlarmCn_t s_acn_rsn_invalid = {64U, 8U, acn_rsn_invalid};

static const uint8_t acn_rsn_nofix[] = { /* no signal */
  0x00U, 0x00U, 0x08U, 0x40U, 0x00U, 0x00U,
  0x3FU, 0xF0U, 0x08U, 0x20U, 0x1FU, 0xF0U,
  0x02U, 0x00U, 0x0BU, 0xFEU, 0x10U, 0x10U,
  0x02U, 0x00U, 0x10U, 0x00U, 0x10U, 0x10U,
  0x02U, 0x00U, 0x10U, 0x00U, 0x10U, 0x10U,
  0x02U, 0x00U, 0x31U, 0xFCU, 0x1FU, 0xF0U,
  0x7FU, 0xFCU, 0x30U, 0x00U, 0x00U, 0x00U,
  0x04U, 0x80U, 0x50U, 0x00U, 0xFFU, 0xFEU,
  0x04U, 0x80U, 0x91U, 0xFCU, 0x08U, 0x00U,
  0x04U, 0x80U, 0x10U, 0x00U, 0x10U, 0x00U,
  0x08U, 0x80U, 0x10U, 0x00U, 0x1FU, 0xF0U,
  0x08U, 0x80U, 0x11U, 0xFCU, 0x00U, 0x10U,
  0x10U, 0x84U, 0x11U, 0x04U, 0x00U, 0x10U,
  0x20U, 0x84U, 0x11U, 0x04U, 0x00U, 0x10U,
  0x40U, 0x7CU, 0x11U, 0xFCU, 0x00U, 0xA0U,
  0x80U, 0x00U, 0x11U, 0x04U, 0x00U, 0x40U,
};
static const Display_AlarmCn_t s_acn_rsn_nofix = {48U, 6U, acn_rsn_nofix};

static const uint8_t acn_rsn_sensortimeout[] = { /* 传感超时 */
  0x08U, 0x40U, 0x00U, 0x28U, 0x08U, 0x00U, 0x00U, 0x08U,
  0x08U, 0x40U, 0x00U, 0x24U, 0x09U, 0xFCU, 0x00U, 0x08U,
  0x08U, 0x40U, 0x3FU, 0xFEU, 0x08U, 0x44U, 0x7CU, 0x08U,
  0x13U, 0xF8U, 0x20U, 0x20U, 0x7EU, 0x44U, 0x44U, 0x08U,
  0x10U, 0x40U, 0x2FU, 0xA4U, 0x08U, 0x44U, 0x45U, 0xFEU,
  0x30U, 0x80U, 0x20U, 0x24U, 0x08U, 0x94U, 0x44U, 0x08U,
  0x37U, 0xFEU, 0x2FU, 0xA8U, 0xFFU, 0x08U, 0x44U, 0x08U,
  0x50U, 0x80U, 0x28U, 0x98U, 0x08U, 0xFCU, 0x7CU, 0x08U,
  0x91U, 0x00U, 0x28U, 0x92U, 0x28U, 0x84U, 0x44U, 0x88U,
  0x13U, 0xF8U, 0x4FU, 0xAAU, 0x28U, 0x84U, 0x44U, 0x48U,
  0x10U, 0x08U, 0x40U, 0x46U, 0x2EU, 0x84U, 0x44U, 0x48U,
  0x11U, 0x10U, 0x80U, 0x82U, 0x28U, 0xFCU, 0x44U, 0x08U,
  0x10U, 0xA0U, 0x01U, 0x00U, 0x28U, 0x00U, 0x7CU, 0x08U,
  0x10U, 0x40U, 0x48U, 0x84U, 0x58U, 0x00U, 0x44U, 0x08U,
  0x10U, 0x20U, 0x48U, 0x12U, 0x4FU, 0xFEU, 0x00U, 0x28U,
  0x10U, 0x20U, 0x87U, 0xF2U, 0x80U, 0x00U, 0x00U, 0x10U,
};
static const Display_AlarmCn_t s_acn_rsn_sensortimeout = {64U, 8U, acn_rsn_sensortimeout};

static const uint8_t acn_rsn_commoffline[] = { /* 通信离线 */
  0x00U, 0x00U, 0x08U, 0x40U, 0x02U, 0x00U, 0x10U, 0x50U,
  0x47U, 0xF8U, 0x08U, 0x20U, 0x01U, 0x00U, 0x10U, 0x48U,
  0x20U, 0x10U, 0x0BU, 0xFEU, 0xFFU, 0xFEU, 0x20U, 0x40U,
  0x21U, 0xA0U, 0x10U, 0x00U, 0x00U, 0x00U, 0x24U, 0x5CU,
  0x00U, 0x40U, 0x10U, 0x00U, 0x14U, 0x50U, 0x45U, 0xE0U,
  0x07U, 0xFCU, 0x31U, 0xFCU, 0x13U, 0x90U, 0xF8U, 0x40U,
  0xE4U, 0x44U, 0x30U, 0x00U, 0x14U, 0x50U, 0x10U, 0x5EU,
  0x24U, 0x44U, 0x50U, 0x00U, 0x1FU, 0xF0U, 0x23U, 0xE0U,
  0x27U, 0xFCU, 0x91U, 0xFCU, 0x01U, 0x00U, 0x40U, 0x44U,
  0x24U, 0x44U, 0x10U, 0x00U, 0x7FU, 0xFCU, 0xFCU, 0x48U,
  0x24U, 0x44U, 0x10U, 0x00U, 0x42U, 0x04U, 0x40U, 0x30U,
  0x27U, 0xFCU, 0x11U, 0xFCU, 0x44U, 0x44U, 0x00U, 0x22U,
  0x24U, 0x44U, 0x11U, 0x04U, 0x4FU, 0xE4U, 0x1CU, 0x52U,
  0x24U, 0x54U, 0x11U, 0x04U, 0x44U, 0x24U, 0xE0U, 0x8AU,
  0x54U, 0x08U, 0x11U, 0xFCU, 0x40U, 0x14U, 0x43U, 0x06U,
  0x8FU, 0xFEU, 0x11U, 0x04U, 0x40U, 0x08U, 0x00U, 0x02U,
};
static const Display_AlarmCn_t s_acn_rsn_commoffline = {64U, 8U, acn_rsn_commoffline};

static const uint8_t acn_rsn_commtimeout[] = { /* 通信超时 */
  0x00U, 0x00U, 0x08U, 0x40U, 0x08U, 0x00U, 0x00U, 0x08U,
  0x47U, 0xF8U, 0x08U, 0x20U, 0x09U, 0xFCU, 0x00U, 0x08U,
  0x20U, 0x10U, 0x0BU, 0xFEU, 0x08U, 0x44U, 0x7CU, 0x08U,
  0x21U, 0xA0U, 0x10U, 0x00U, 0x7EU, 0x44U, 0x44U, 0x08U,
  0x00U, 0x40U, 0x10U, 0x00U, 0x08U, 0x44U, 0x45U, 0xFEU,
  0x07U, 0xFCU, 0x31U, 0xFCU, 0x08U, 0x94U, 0x44U, 0x08U,
  0xE4U, 0x44U, 0x30U, 0x00U, 0xFFU, 0x08U, 0x44U, 0x08U,
  0x24U, 0x44U, 0x50U, 0x00U, 0x08U, 0xFCU, 0x7CU, 0x08U,
  0x27U, 0xFCU, 0x91U, 0xFCU, 0x28U, 0x84U, 0x44U, 0x88U,
  0x24U, 0x44U, 0x10U, 0x00U, 0x28U, 0x84U, 0x44U, 0x48U,
  0x24U, 0x44U, 0x10U, 0x00U, 0x2EU, 0x84U, 0x44U, 0x48U,
  0x27U, 0xFCU, 0x11U, 0xFCU, 0x28U, 0xFCU, 0x44U, 0x08U,
  0x24U, 0x44U, 0x11U, 0x04U, 0x28U, 0x00U, 0x7CU, 0x08U,
  0x24U, 0x54U, 0x11U, 0x04U, 0x58U, 0x00U, 0x44U, 0x08U,
  0x54U, 0x08U, 0x11U, 0xFCU, 0x4FU, 0xFEU, 0x00U, 0x28U,
  0x8FU, 0xFEU, 0x11U, 0x04U, 0x80U, 0x00U, 0x00U, 0x10U,
};
static const Display_AlarmCn_t s_acn_rsn_commtimeout = {64U, 8U, acn_rsn_commtimeout};

static const uint8_t acn_rsn_commframe[] = { /* 帧错误 */
  0x10U, 0x20U, 0x21U, 0x10U, 0x00U, 0x00U,
  0x10U, 0x20U, 0x21U, 0x10U, 0x43U, 0xF8U,
  0x10U, 0x3EU, 0x39U, 0x10U, 0x22U, 0x08U,
  0x7CU, 0x20U, 0x27U, 0xFCU, 0x22U, 0x08U,
  0x54U, 0x20U, 0x41U, 0x10U, 0x03U, 0xF8U,
  0x55U, 0xFCU, 0x79U, 0x10U, 0x00U, 0x00U,
  0x55U, 0x04U, 0xAFU, 0xFEU, 0xE0U, 0x00U,
  0x55U, 0x24U, 0x20U, 0x00U, 0x27U, 0xFCU,
  0x55U, 0x24U, 0xFBU, 0xF8U, 0x20U, 0x40U,
  0x55U, 0x24U, 0x22U, 0x08U, 0x20U, 0x40U,
  0x55U, 0x24U, 0x22U, 0x08U, 0x2FU, 0xFEU,
  0x5DU, 0x24U, 0x23U, 0xF8U, 0x20U, 0x40U,
  0x10U, 0x50U, 0x2AU, 0x08U, 0x28U, 0xA0U,
  0x10U, 0x48U, 0x32U, 0x08U, 0x31U, 0x10U,
  0x10U, 0x84U, 0x23U, 0xF8U, 0x22U, 0x08U,
  0x11U, 0x04U, 0x02U, 0x08U, 0x0CU, 0x06U,
};
static const Display_AlarmCn_t s_acn_rsn_commframe = {48U, 6U, acn_rsn_commframe};

static const uint8_t acn_rsn_dispoffline[] = { /* 显示离线 */
  0x00U, 0x00U, 0x00U, 0x00U, 0x02U, 0x00U, 0x10U, 0x50U,
  0x1FU, 0xF0U, 0x3FU, 0xF8U, 0x01U, 0x00U, 0x10U, 0x48U,
  0x10U, 0x10U, 0x00U, 0x00U, 0xFFU, 0xFEU, 0x20U, 0x40U,
  0x10U, 0x10U, 0x00U, 0x00U, 0x00U, 0x00U, 0x24U, 0x5CU,
  0x1FU, 0xF0U, 0x00U, 0x00U, 0x14U, 0x50U, 0x45U, 0xE0U,
  0x10U, 0x10U, 0x00U, 0x00U, 0x13U, 0x90U, 0xF8U, 0x40U,
  0x10U, 0x10U, 0xFFU, 0xFEU, 0x14U, 0x50U, 0x10U, 0x5EU,
  0x1FU, 0xF0U, 0x01U, 0x00U, 0x1FU, 0xF0U, 0x23U, 0xE0U,
  0x04U, 0x40U, 0x01U, 0x00U, 0x01U, 0x00U, 0x40U, 0x44U,
  0x44U, 0x44U, 0x11U, 0x10U, 0x7FU, 0xFCU, 0xFCU, 0x48U,
  0x24U, 0x44U, 0x11U, 0x08U, 0x42U, 0x04U, 0x40U, 0x30U,
  0x14U, 0x48U, 0x21U, 0x04U, 0x44U, 0x44U, 0x00U, 0x22U,
  0x14U, 0x50U, 0x41U, 0x02U, 0x4FU, 0xE4U, 0x1CU, 0x52U,
  0x04U, 0x40U, 0x81U, 0x02U, 0x44U, 0x24U, 0xE0U, 0x8AU,
  0xFFU, 0xFEU, 0x05U, 0x00U, 0x40U, 0x14U, 0x43U, 0x06U,
  0x00U, 0x00U, 0x02U, 0x00U, 0x40U, 0x08U, 0x00U, 0x02U,
};
static const Display_AlarmCn_t s_acn_rsn_dispoffline = {64U, 8U, acn_rsn_dispoffline};

static const uint8_t acn_rsn_disprefresh[] = { /* 刷新异常 */
  0x00U, 0x02U, 0x10U, 0x00U, 0x00U, 0x00U, 0x01U, 0x00U,
  0x3FU, 0xC2U, 0x08U, 0x04U, 0x3FU, 0xF0U, 0x11U, 0x10U,
  0x20U, 0x42U, 0x7FU, 0x78U, 0x20U, 0x10U, 0x09U, 0x20U,
  0x20U, 0x42U, 0x00U, 0x40U, 0x20U, 0x10U, 0x7FU, 0xFEU,
  0x3FU, 0xD2U, 0x22U, 0x40U, 0x3FU, 0xF0U, 0x40U, 0x02U,
  0x22U, 0x12U, 0x14U, 0x40U, 0x20U, 0x04U, 0x9FU, 0xF4U,
  0x22U, 0x12U, 0xFFU, 0x7EU, 0x20U, 0x04U, 0x10U, 0x10U,
  0x22U, 0x12U, 0x08U, 0x48U, 0x1FU, 0xFCU, 0x1FU, 0xF0U,
  0x3FU, 0xD2U, 0x08U, 0x48U, 0x00U, 0x00U, 0x01U, 0x00U,
  0x52U, 0x52U, 0x7FU, 0x48U, 0x08U, 0x20U, 0x3FU, 0xF8U,
  0x52U, 0x52U, 0x08U, 0x48U, 0x08U, 0x20U, 0x21U, 0x08U,
  0x52U, 0x52U, 0x2AU, 0x48U, 0xFFU, 0xFEU, 0x21U, 0x08U,
  0x93U, 0x42U, 0x49U, 0x48U, 0x08U, 0x20U, 0x21U, 0x28U,
  0x12U, 0x82U, 0x88U, 0x88U, 0x10U, 0x20U, 0x21U, 0x10U,
  0x02U, 0x0AU, 0x28U, 0x88U, 0x20U, 0x20U, 0x01U, 0x00U,
  0x02U, 0x04U, 0x11U, 0x08U, 0x40U, 0x20U, 0x01U, 0x00U,
};
static const Display_AlarmCn_t s_acn_rsn_disprefresh = {64U, 8U, acn_rsn_disprefresh};

static const uint8_t acn_rsn_storenotready[] = { /* 存储未就绪 */
  0x04U, 0x00U, 0x10U, 0x20U, 0x01U, 0x00U, 0x20U, 0x40U, 0x10U, 0x40U,
  0x04U, 0x00U, 0x10U, 0x20U, 0x01U, 0x00U, 0x10U, 0x50U, 0x10U, 0x44U,
  0xFFU, 0xFEU, 0x18U, 0xFAU, 0x01U, 0x00U, 0xFEU, 0x48U, 0x23U, 0xF4U,
  0x08U, 0x00U, 0x24U, 0x24U, 0x3FU, 0xF8U, 0x00U, 0x48U, 0x20U, 0x48U,
  0x08U, 0x00U, 0x24U, 0x28U, 0x01U, 0x00U, 0x00U, 0x40U, 0x48U, 0x50U,
  0x13U, 0xF8U, 0x61U, 0xFEU, 0x01U, 0x00U, 0x7DU, 0xFEU, 0xF7U, 0xFEU,
  0x10U, 0x10U, 0x60U, 0x20U, 0x01U, 0x00U, 0x44U, 0x50U, 0x10U, 0x40U,
  0x30U, 0x20U, 0xBCU, 0x40U, 0xFFU, 0xFEU, 0x44U, 0x50U, 0x20U, 0x80U,
  0x50U, 0x40U, 0x24U, 0xFCU, 0x03U, 0x80U, 0x44U, 0x50U, 0x41U, 0xF8U,
  0x97U, 0xFEU, 0x25U, 0x44U, 0x05U, 0x40U, 0x7CU, 0x50U, 0xFBU, 0x08U,
  0x10U, 0x40U, 0x26U, 0x44U, 0x09U, 0x20U, 0x10U, 0x90U, 0x45U, 0x08U,
  0x10U, 0x40U, 0x24U, 0x7CU, 0x11U, 0x10U, 0x54U, 0x90U, 0x01U, 0xF8U,
  0x10U, 0x40U, 0x25U, 0x44U, 0x21U, 0x08U, 0x92U, 0x92U, 0x19U, 0x08U,
  0x10U, 0x40U, 0x26U, 0x44U, 0xC1U, 0x06U, 0x11U, 0x12U, 0xE1U, 0x08U,
  0x11U, 0x40U, 0x24U, 0x7CU, 0x01U, 0x00U, 0x51U, 0x0EU, 0x41U, 0xF8U,
  0x10U, 0x80U, 0x20U, 0x44U, 0x01U, 0x00U, 0x22U, 0x00U, 0x01U, 0x08U,
};
static const Display_AlarmCn_t s_acn_rsn_storenotready = {80U, 10U, acn_rsn_storenotready};

static const uint8_t acn_rsn_storewrite[] = { /* 写入失败 */
  0x00U, 0x00U, 0x04U, 0x00U, 0x01U, 0x00U, 0x00U, 0x40U,
  0x7FU, 0xFEU, 0x02U, 0x00U, 0x11U, 0x00U, 0x7CU, 0x40U,
  0x40U, 0x02U, 0x01U, 0x00U, 0x11U, 0x00U, 0x44U, 0x40U,
  0x90U, 0x04U, 0x01U, 0x00U, 0x11U, 0x00U, 0x54U, 0x80U,
  0x10U, 0x00U, 0x01U, 0x00U, 0x3FU, 0xF8U, 0x54U, 0xFEU,
  0x1FU, 0xF8U, 0x02U, 0x80U, 0x21U, 0x00U, 0x55U, 0x08U,
  0x10U, 0x00U, 0x02U, 0x80U, 0x41U, 0x00U, 0x56U, 0x88U,
  0x20U, 0x00U, 0x02U, 0x80U, 0x01U, 0x00U, 0x54U, 0x88U,
  0x3FU, 0xF8U, 0x04U, 0x40U, 0xFFU, 0xFEU, 0x54U, 0x88U,
  0x00U, 0x08U, 0x04U, 0x40U, 0x02U, 0x80U, 0x54U, 0x50U,
  0x00U, 0x08U, 0x08U, 0x20U, 0x04U, 0x40U, 0x54U, 0x50U,
  0xFFU, 0xC8U, 0x08U, 0x20U, 0x04U, 0x40U, 0x10U, 0x20U,
  0x00U, 0x08U, 0x10U, 0x10U, 0x08U, 0x20U, 0x28U, 0x50U,
  0x00U, 0x08U, 0x20U, 0x10U, 0x10U, 0x10U, 0x24U, 0x88U,
  0x00U, 0x50U, 0x40U, 0x08U, 0x20U, 0x08U, 0x45U, 0x04U,
  0x00U, 0x20U, 0x80U, 0x06U, 0xC0U, 0x06U, 0x82U, 0x02U,
};
static const Display_AlarmCn_t s_acn_rsn_storewrite = {64U, 8U, acn_rsn_storewrite};

static const uint8_t acn_rsn_storefull[] = { /* 存储已满 */
  0x04U, 0x00U, 0x10U, 0x20U, 0x00U, 0x00U, 0x01U, 0x08U,
  0x04U, 0x00U, 0x10U, 0x20U, 0x3FU, 0xF0U, 0x21U, 0x08U,
  0xFFU, 0xFEU, 0x18U, 0xFAU, 0x00U, 0x10U, 0x17U, 0xFEU,
  0x08U, 0x00U, 0x24U, 0x24U, 0x00U, 0x10U, 0x11U, 0x08U,
  0x08U, 0x00U, 0x24U, 0x28U, 0x00U, 0x10U, 0x80U, 0x00U,
  0x13U, 0xF8U, 0x61U, 0xFEU, 0x20U, 0x10U, 0x47U, 0xFEU,
  0x10U, 0x10U, 0x60U, 0x20U, 0x20U, 0x10U, 0x40U, 0x90U,
  0x30U, 0x20U, 0xBCU, 0x40U, 0x3FU, 0xF0U, 0x10U, 0x90U,
  0x50U, 0x40U, 0x24U, 0xFCU, 0x20U, 0x00U, 0x17U, 0xFEU,
  0x97U, 0xFEU, 0x25U, 0x44U, 0x20U, 0x00U, 0x24U, 0x92U,
  0x10U, 0x40U, 0x26U, 0x44U, 0x20U, 0x00U, 0xE4U, 0x92U,
  0x10U, 0x40U, 0x24U, 0x7CU, 0x20U, 0x04U, 0x25U, 0x6AU,
  0x10U, 0x40U, 0x25U, 0x44U, 0x20U, 0x04U, 0x26U, 0x46U,
  0x10U, 0x40U, 0x26U, 0x44U, 0x20U, 0x04U, 0x24U, 0x02U,
  0x11U, 0x40U, 0x24U, 0x7CU, 0x1FU, 0xFCU, 0x24U, 0x0AU,
  0x10U, 0x80U, 0x20U, 0x44U, 0x00U, 0x00U, 0x04U, 0x04U,
};
static const Display_AlarmCn_t s_acn_rsn_storefull = {64U, 8U, acn_rsn_storefull};

static const uint8_t acn_rsn_protoparse[] = { /* 解析错误 */
  0x10U, 0x00U, 0x10U, 0x08U, 0x21U, 0x10U, 0x00U, 0x00U,
  0x10U, 0xFCU, 0x10U, 0x1CU, 0x21U, 0x10U, 0x43U, 0xF8U,
  0x3CU, 0x24U, 0x11U, 0xE0U, 0x39U, 0x10U, 0x22U, 0x08U,
  0x24U, 0x24U, 0x11U, 0x00U, 0x27U, 0xFCU, 0x22U, 0x08U,
  0x48U, 0x54U, 0xFDU, 0x00U, 0x41U, 0x10U, 0x03U, 0xF8U,
  0xBEU, 0x88U, 0x11U, 0x00U, 0x79U, 0x10U, 0x00U, 0x00U,
  0x2AU, 0x10U, 0x31U, 0xFEU, 0xAFU, 0xFEU, 0xE0U, 0x00U,
  0x2AU, 0x50U, 0x39U, 0x10U, 0x20U, 0x00U, 0x27U, 0xFCU,
  0x3EU, 0x7CU, 0x55U, 0x10U, 0xFBU, 0xF8U, 0x20U, 0x40U,
  0x2AU, 0x90U, 0x51U, 0x10U, 0x22U, 0x08U, 0x20U, 0x40U,
  0x2AU, 0x10U, 0x91U, 0x10U, 0x22U, 0x08U, 0x2FU, 0xFEU,
  0x3EU, 0xFEU, 0x11U, 0x10U, 0x23U, 0xF8U, 0x20U, 0x40U,
  0x2AU, 0x10U, 0x11U, 0x10U, 0x2AU, 0x08U, 0x28U, 0xA0U,
  0x4AU, 0x10U, 0x12U, 0x10U, 0x32U, 0x08U, 0x31U, 0x10U,
  0x42U, 0x10U, 0x12U, 0x10U, 0x23U, 0xF8U, 0x22U, 0x08U,
  0x86U, 0x10U, 0x14U, 0x10U, 0x02U, 0x08U, 0x0CU, 0x06U,
};
static const Display_AlarmCn_t s_acn_rsn_protoparse = {64U, 8U, acn_rsn_protoparse};

static const uint8_t acn_rsn_protocrc[] = { /* 校验错误 */
  0x10U, 0x40U, 0x00U, 0x20U, 0x21U, 0x10U, 0x00U, 0x00U,
  0x10U, 0x20U, 0xF8U, 0x20U, 0x21U, 0x10U, 0x43U, 0xF8U,
  0x10U, 0x20U, 0x08U, 0x50U, 0x39U, 0x10U, 0x22U, 0x08U,
  0x11U, 0xFEU, 0x48U, 0x50U, 0x27U, 0xFCU, 0x22U, 0x08U,
  0xFCU, 0x00U, 0x48U, 0x88U, 0x41U, 0x10U, 0x03U, 0xF8U,
  0x10U, 0x88U, 0x49U, 0x04U, 0x79U, 0x10U, 0x00U, 0x00U,
  0x31U, 0x04U, 0x4AU, 0xFAU, 0xAFU, 0xFEU, 0xE0U, 0x00U,
  0x3AU, 0x02U, 0x7CU, 0x00U, 0x20U, 0x00U, 0x27U, 0xFCU,
  0x54U, 0x88U, 0x04U, 0x44U, 0xFBU, 0xF8U, 0x20U, 0x40U,
  0x50U, 0x88U, 0x04U, 0x24U, 0x22U, 0x08U, 0x20U, 0x40U,
  0x90U, 0x50U, 0x1DU, 0x24U, 0x22U, 0x08U, 0x2FU, 0xFEU,
  0x10U, 0x50U, 0xE4U, 0xA8U, 0x23U, 0xF8U, 0x20U, 0x40U,
  0x10U, 0x20U, 0x44U, 0x88U, 0x2AU, 0x08U, 0x28U, 0xA0U,
  0x10U, 0x50U, 0x04U, 0x10U, 0x32U, 0x08U, 0x31U, 0x10U,
  0x10U, 0x88U, 0x2BU, 0xFEU, 0x23U, 0xF8U, 0x22U, 0x08U,
  0x13U, 0x06U, 0x10U, 0x00U, 0x02U, 0x08U, 0x0CU, 0x06U,
};
static const Display_AlarmCn_t s_acn_rsn_protocrc = {64U, 8U, acn_rsn_protocrc};

static const uint8_t acn_rsn_estinput[] = { /* 输入异常 */
  0x20U, 0x40U, 0x04U, 0x00U, 0x00U, 0x00U, 0x01U, 0x00U,
  0x20U, 0xA0U, 0x02U, 0x00U, 0x3FU, 0xF0U, 0x11U, 0x10U,
  0x21U, 0x10U, 0x01U, 0x00U, 0x20U, 0x10U, 0x09U, 0x20U,
  0xFAU, 0x08U, 0x01U, 0x00U, 0x20U, 0x10U, 0x7FU, 0xFEU,
  0x25U, 0xF6U, 0x01U, 0x00U, 0x3FU, 0xF0U, 0x40U, 0x02U,
  0x40U, 0x00U, 0x02U, 0x80U, 0x20U, 0x04U, 0x9FU, 0xF4U,
  0x53U, 0xC4U, 0x02U, 0x80U, 0x20U, 0x04U, 0x10U, 0x10U,
  0x92U, 0x54U, 0x02U, 0x80U, 0x1FU, 0xFCU, 0x1FU, 0xF0U,
  0xFAU, 0x54U, 0x04U, 0x40U, 0x00U, 0x00U, 0x01U, 0x00U,
  0x13U, 0xD4U, 0x04U, 0x40U, 0x08U, 0x20U, 0x3FU, 0xF8U,
  0x1AU, 0x54U, 0x08U, 0x20U, 0x08U, 0x20U, 0x21U, 0x08U,
  0xF2U, 0x54U, 0x08U, 0x20U, 0xFFU, 0xFEU, 0x21U, 0x08U,
  0x53U, 0xD4U, 0x10U, 0x10U, 0x08U, 0x20U, 0x21U, 0x28U,
  0x12U, 0x44U, 0x20U, 0x10U, 0x10U, 0x20U, 0x21U, 0x10U,
  0x12U, 0x54U, 0x40U, 0x08U, 0x20U, 0x20U, 0x01U, 0x00U,
  0x12U, 0xC8U, 0x80U, 0x06U, 0x40U, 0x20U, 0x01U, 0x00U,
};
static const Display_AlarmCn_t s_acn_rsn_estinput = {64U, 8U, acn_rsn_estinput};

static const uint8_t acn_rsn_estdiverge[] = { /* 估计发散 */
  0x08U, 0x40U, 0x00U, 0x40U, 0x01U, 0x00U, 0x24U, 0x20U,
  0x08U, 0x40U, 0x20U, 0x40U, 0x11U, 0x10U, 0x24U, 0x20U,
  0x08U, 0x40U, 0x10U, 0x40U, 0x11U, 0x08U, 0x7EU, 0x20U,
  0x10U, 0x40U, 0x10U, 0x40U, 0x22U, 0x00U, 0x24U, 0x3EU,
  0x17U, 0xFEU, 0x00U, 0x40U, 0x3FU, 0xFCU, 0x24U, 0x44U,
  0x30U, 0x40U, 0x00U, 0x40U, 0x02U, 0x00U, 0xFFU, 0x44U,
  0x30U, 0x40U, 0xF7U, 0xFEU, 0x04U, 0x00U, 0x00U, 0x44U,
  0x50U, 0x40U, 0x10U, 0x40U, 0x07U, 0xF8U, 0x7EU, 0xA4U,
  0x93U, 0xF8U, 0x10U, 0x40U, 0x0AU, 0x08U, 0x42U, 0x28U,
  0x12U, 0x08U, 0x10U, 0x40U, 0x09U, 0x08U, 0x7EU, 0x28U,
  0x12U, 0x08U, 0x10U, 0x40U, 0x11U, 0x10U, 0x42U, 0x10U,
  0x12U, 0x08U, 0x10U, 0x40U, 0x10U, 0xA0U, 0x7EU, 0x10U,
  0x12U, 0x08U, 0x14U, 0x40U, 0x20U, 0x40U, 0x42U, 0x28U,
  0x12U, 0x08U, 0x18U, 0x40U, 0x40U, 0xA0U, 0x42U, 0x28U,
  0x13U, 0xF8U, 0x10U, 0x40U, 0x03U, 0x18U, 0x4AU, 0x44U,
  0x12U, 0x08U, 0x00U, 0x40U, 0x1CU, 0x06U, 0x44U, 0x82U,
};
static const Display_AlarmCn_t s_acn_rsn_estdiverge = {64U, 8U, acn_rsn_estdiverge};

static const uint8_t acn_rsn_unknown[] = { /* 未知告警 */
  0x01U, 0x00U, 0x20U, 0x00U, 0x01U, 0x00U, 0x24U, 0x20U,
  0x01U, 0x00U, 0x20U, 0x00U, 0x11U, 0x00U, 0xFFU, 0x20U,
  0x01U, 0x00U, 0x20U, 0x7CU, 0x11U, 0x00U, 0x24U, 0x7EU,
  0x3FU, 0xF8U, 0x7EU, 0x44U, 0x1FU, 0xF8U, 0x7EU, 0xC4U,
  0x01U, 0x00U, 0x48U, 0x44U, 0x21U, 0x00U, 0x82U, 0x28U,
  0x01U, 0x00U, 0x88U, 0x44U, 0x41U, 0x00U, 0x7AU, 0x10U,
  0x01U, 0x00U, 0x08U, 0x44U, 0x01U, 0x00U, 0x4AU, 0x28U,
  0xFFU, 0xFEU, 0x08U, 0x44U, 0xFFU, 0xFEU, 0x7AU, 0xC6U,
  0x03U, 0x80U, 0xFFU, 0x44U, 0x00U, 0x00U, 0x05U, 0x00U,
  0x05U, 0x40U, 0x08U, 0x44U, 0x00U, 0x00U, 0xFFU, 0xFEU,
  0x09U, 0x20U, 0x14U, 0x44U, 0x1FU, 0xF0U, 0x00U, 0x00U,
  0x11U, 0x10U, 0x14U, 0x44U, 0x10U, 0x10U, 0x3FU, 0xF8U,
  0x21U, 0x08U, 0x22U, 0x7CU, 0x10U, 0x10U, 0x00U, 0x00U,
  0xC1U, 0x06U, 0x22U, 0x44U, 0x10U, 0x10U, 0x3FU, 0xF8U,
  0x01U, 0x00U, 0x42U, 0x00U, 0x1FU, 0xF0U, 0x20U, 0x08U,
  0x01U, 0x00U, 0x80U, 0x00U, 0x10U, 0x10U, 0x3FU, 0xF8U,
};
static const Display_AlarmCn_t s_acn_rsn_unknown = {64U, 8U, acn_rsn_unknown};

/* ----- 电机告警中文字模(电机 / 电机断开 / 供电不足) ----- */
static const uint8_t acn_mod_motor[] = { /* 电机 */
  0x01U, 0x00U, 0x10U, 0x00U,
  0x01U, 0x00U, 0x11U, 0xF0U,
  0x01U, 0x00U, 0x11U, 0x10U,
  0x3FU, 0xF8U, 0x11U, 0x10U,
  0x21U, 0x08U, 0xFDU, 0x10U,
  0x21U, 0x08U, 0x11U, 0x10U,
  0x21U, 0x08U, 0x31U, 0x10U,
  0x3FU, 0xF8U, 0x39U, 0x10U,
  0x21U, 0x08U, 0x55U, 0x10U,
  0x21U, 0x08U, 0x55U, 0x10U,
  0x21U, 0x08U, 0x91U, 0x10U,
  0x3FU, 0xF8U, 0x11U, 0x12U,
  0x21U, 0x0AU, 0x11U, 0x12U,
  0x01U, 0x02U, 0x12U, 0x12U,
  0x01U, 0x02U, 0x12U, 0x0EU,
  0x00U, 0xFEU, 0x14U, 0x00U,
};
static const Display_AlarmCn_t s_acn_mod_motor = {32U, 4U, acn_mod_motor};

static const uint8_t acn_rsn_motordisc[] = { /* 电机断开 */
  0x01U, 0x00U, 0x10U, 0x00U, 0x04U, 0x00U, 0x00U, 0x00U,
  0x01U, 0x00U, 0x11U, 0xF0U, 0x04U, 0x04U, 0x7FU, 0xFCU,
  0x01U, 0x00U, 0x11U, 0x10U, 0x55U, 0x78U, 0x08U, 0x20U,
  0x3FU, 0xF8U, 0x11U, 0x10U, 0x4EU, 0x40U, 0x08U, 0x20U,
  0x21U, 0x08U, 0xFDU, 0x10U, 0x44U, 0x40U, 0x08U, 0x20U,
  0x21U, 0x08U, 0x11U, 0x10U, 0x7FU, 0x40U, 0x08U, 0x20U,
  0x21U, 0x08U, 0x31U, 0x10U, 0x44U, 0x7EU, 0x08U, 0x20U,
  0x3FU, 0xF8U, 0x39U, 0x10U, 0x4EU, 0x48U, 0xFFU, 0xFEU,
  0x21U, 0x08U, 0x55U, 0x10U, 0x55U, 0x48U, 0x08U, 0x20U,
  0x21U, 0x08U, 0x55U, 0x10U, 0x65U, 0x48U, 0x08U, 0x20U,
  0x21U, 0x08U, 0x91U, 0x10U, 0x44U, 0x48U, 0x08U, 0x20U,
  0x3FU, 0xF8U, 0x11U, 0x12U, 0x44U, 0x48U, 0x08U, 0x20U,
  0x21U, 0x0AU, 0x11U, 0x12U, 0x40U, 0x48U, 0x10U, 0x20U,
  0x01U, 0x02U, 0x12U, 0x12U, 0x7FU, 0x88U, 0x10U, 0x20U,
  0x01U, 0x02U, 0x12U, 0x0EU, 0x00U, 0x88U, 0x20U, 0x20U,
  0x00U, 0xFEU, 0x14U, 0x00U, 0x01U, 0x08U, 0x40U, 0x20U,
};
static const Display_AlarmCn_t s_acn_rsn_motordisc = {64U, 8U, acn_rsn_motordisc};

static const uint8_t acn_rsn_powerlow[] = { /* 供电不足 */
  0x09U, 0x10U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
  0x09U, 0x10U, 0x01U, 0x00U, 0x7FU, 0xFCU, 0x1FU, 0xF0U,
  0x09U, 0x10U, 0x01U, 0x00U, 0x00U, 0x80U, 0x10U, 0x10U,
  0x11U, 0x10U, 0x3FU, 0xF8U, 0x00U, 0x80U, 0x10U, 0x10U,
  0x13U, 0xFCU, 0x21U, 0x08U, 0x01U, 0x00U, 0x10U, 0x10U,
  0x31U, 0x10U, 0x21U, 0x08U, 0x01U, 0x00U, 0x10U, 0x10U,
  0x31U, 0x10U, 0x21U, 0x08U, 0x03U, 0x40U, 0x1FU, 0xF0U,
  0x51U, 0x10U, 0x3FU, 0xF8U, 0x05U, 0x20U, 0x01U, 0x00U,
  0x91U, 0x10U, 0x21U, 0x08U, 0x09U, 0x10U, 0x01U, 0x00U,
  0x17U, 0xFEU, 0x21U, 0x08U, 0x11U, 0x08U, 0x11U, 0x00U,
  0x10U, 0x00U, 0x21U, 0x08U, 0x21U, 0x04U, 0x11U, 0xF8U,
  0x11U, 0x10U, 0x3FU, 0xF8U, 0x41U, 0x04U, 0x11U, 0x00U,
  0x11U, 0x08U, 0x21U, 0x0AU, 0x81U, 0x00U, 0x29U, 0x00U,
  0x12U, 0x08U, 0x01U, 0x02U, 0x01U, 0x00U, 0x25U, 0x00U,
  0x14U, 0x04U, 0x01U, 0x02U, 0x01U, 0x00U, 0x43U, 0xFEU,
  0x18U, 0x04U, 0x00U, 0xFEU, 0x01U, 0x00U, 0x80U, 0x00U,
};
static const Display_AlarmCn_t s_acn_rsn_powerlow = {64U, 8U, acn_rsn_powerlow};

static const char *Display_PagesGetAlarmModule(uint16_t source_id, uint16_t code)
{
  /* 电机无独立模块 ID，按故障码优先归类，不受 source_id 影响。 */
  if ((code == PX4LITE_FAULT_MOTOR_DISCONNECT) || (code == PX4LITE_FAULT_MOTOR_POWER_LOW) || (code == PX4LITE_FAULT_MOTOR_DEAD) || (code == PX4LITE_FAULT_MOTOR_CHARGE)) { return "MOTOR"; }

  switch ((Px4Lite_ModuleId_t)source_id) {
    case PX4LITE_MODULE_GNSS:
      return "GPS";
    case PX4LITE_MODULE_IMU:
      return "IMU";
    case PX4LITE_MODULE_BARO:
      return "ENV";
    case PX4LITE_MODULE_BATTERY:
      return "POWER";
    case PX4LITE_MODULE_LORA:
      return "LORA";
    case PX4LITE_MODULE_5G:
      return "5G";
    case PX4LITE_MODULE_STORAGE:
      return "SD";
    case PX4LITE_MODULE_REMOTE_ID:
      return "REMOTE ID";
    case PX4LITE_MODULE_DISPLAY:
      return "DISPLAY";
    case PX4LITE_MODULE_CONTROL:
      return "CONTROL";
    case PX4LITE_MODULE_ALARM:
      return "ALARM";
    case PX4LITE_MODULE_SYSTEM:
      return "SYSTEM";
    case PX4LITE_MODULE_ESTIMATOR:
      return "EST";
    case PX4LITE_MODULE_BUSINESS:
      return "BUSINESS";
    default:
      break;
  }

  switch (code) {
    case PX4LITE_FAULT_SYSTEM_SELF_CHECK:
    case PX4LITE_FAULT_SYSTEM_HEAP:
    case PX4LITE_FAULT_SYSTEM_STACK:
    case PX4LITE_FAULT_SYSTEM_TASK_LOST:
      return "SYSTEM";

    case PX4LITE_FAULT_SENSOR_INIT:
    case PX4LITE_FAULT_SENSOR_OFFLINE:
    case PX4LITE_FAULT_SENSOR_INVALID:
    case PX4LITE_FAULT_SENSOR_NO_FIX:
    case PX4LITE_FAULT_SENSOR_TIMEOUT:
      return "SENSOR";

    case PX4LITE_FAULT_COMM_OFFLINE:
    case PX4LITE_FAULT_COMM_TIMEOUT:
    case PX4LITE_FAULT_COMM_FRAME:
      return "COMM";

    case PX4LITE_FAULT_DISPLAY_OFFLINE:
    case PX4LITE_FAULT_DISPLAY_REFRESH:
      return "DISPLAY";

    case PX4LITE_FAULT_STORAGE_NOT_READY:
    case PX4LITE_FAULT_STORAGE_WRITE:
    case PX4LITE_FAULT_STORAGE_FULL:
      return "STORAGE";

    case PX4LITE_FAULT_PROTOCOL_PARSE:
    case PX4LITE_FAULT_PROTOCOL_CRC:
      return "PROTO";

    case PX4LITE_FAULT_APP_INVALID_DATA:
      return "APP";

    case PX4LITE_FAULT_ESTIMATOR_INPUT:
    case PX4LITE_FAULT_ESTIMATOR_DIVERGE:
      return "EST";

    case PX4LITE_FAULT_MOTOR_DISCONNECT:
    case PX4LITE_FAULT_MOTOR_POWER_LOW:
      return "MOTOR";

    default:
      break;
  }

  return "SYSTEM";
}

static const char *Display_PagesGetAlarmReason(uint32_t code)
{
  switch ((uint16_t)code) {
    case PX4LITE_FAULT_SYSTEM_SELF_CHECK:
      return "SELF CHECK";
    case PX4LITE_FAULT_SYSTEM_HEAP:
      return "HEAP FAILED";
    case PX4LITE_FAULT_SYSTEM_STACK:
      return "STACK OVERFLOW";
    case PX4LITE_FAULT_SYSTEM_TASK_LOST:
      return "TASK LOST";
    case PX4LITE_FAULT_SENSOR_INIT:
      return "SENSOR INIT";
    case PX4LITE_FAULT_SENSOR_OFFLINE:
      return "SENSOR OFFLINE";
    case PX4LITE_FAULT_SENSOR_INVALID:
      return "SENSOR INVALID";
    case PX4LITE_FAULT_SENSOR_NO_FIX:
      return "NO SIGNAL";
    case PX4LITE_FAULT_SENSOR_TIMEOUT:
      return "SENSOR TIMEOUT";
    case PX4LITE_FAULT_COMM_OFFLINE:
      return "COMM OFFLINE";
    case PX4LITE_FAULT_COMM_TIMEOUT:
      return "COMM TIMEOUT";
    case PX4LITE_FAULT_COMM_FRAME:
      return "COMM FRAME";
    case PX4LITE_FAULT_DISPLAY_OFFLINE:
      return "DISPLAY OFFLINE";
    case PX4LITE_FAULT_DISPLAY_REFRESH:
      return "DISPLAY REFRESH";
    case PX4LITE_FAULT_STORAGE_NOT_READY:
      return "STORAGE NOT READY";
    case PX4LITE_FAULT_STORAGE_WRITE:
      return "STORAGE WRITE";
    case PX4LITE_FAULT_STORAGE_FULL:
      return "STORAGE FULL";
    case PX4LITE_FAULT_PROTOCOL_PARSE:
      return "PROTO PARSE";
    case PX4LITE_FAULT_PROTOCOL_CRC:
      return "PROTO CRC";
    case PX4LITE_FAULT_APP_INVALID_DATA:
      return "APP INVALID";
    case PX4LITE_FAULT_ESTIMATOR_INPUT:
      return "EST INPUT";
    case PX4LITE_FAULT_ESTIMATOR_DIVERGE:
      return "EST DIVERGE";
    case PX4LITE_FAULT_MOTOR_DISCONNECT:
      return "MOTOR DISCONNECT";
    case PX4LITE_FAULT_MOTOR_POWER_LOW:
      return "POWER LOW";
    default:
      break;
  }

  return "UNKNOWN ALARM";
}

/* 告警页模块栏中文字模：无对应中文(如 5G)时返回 0，由调用方回退英文。 */
static const Display_AlarmCn_t *Display_PagesGetAlarmModuleCn(uint16_t source_id, uint16_t code)
{
  /* 电机无独立模块 ID，按故障码优先归类，不受 source_id 影响。 */
  if ((code == PX4LITE_FAULT_MOTOR_DISCONNECT) || (code == PX4LITE_FAULT_MOTOR_POWER_LOW) || (code == PX4LITE_FAULT_MOTOR_DEAD) || (code == PX4LITE_FAULT_MOTOR_CHARGE)) { return &s_acn_mod_motor; }

  switch ((Px4Lite_ModuleId_t)source_id) {
    case PX4LITE_MODULE_GNSS: return &s_acn_mod_gnss;
    case PX4LITE_MODULE_IMU: return &s_acn_mod_imu;
    case PX4LITE_MODULE_BARO: return &s_acn_mod_baro;
    case PX4LITE_MODULE_BATTERY: return &s_acn_mod_power;
    case PX4LITE_MODULE_LORA: return &s_acn_mod_comm;
    case PX4LITE_MODULE_5G: return 0;
    case PX4LITE_MODULE_STORAGE: return &s_acn_mod_storage;
    case PX4LITE_MODULE_REMOTE_ID: return &s_acn_mod_remote;
    case PX4LITE_MODULE_DISPLAY: return &s_acn_mod_display;
    case PX4LITE_MODULE_CONTROL: return &s_acn_mod_control;
    case PX4LITE_MODULE_ALARM: return &s_acn_mod_alarm;
    case PX4LITE_MODULE_SYSTEM: return &s_acn_mod_system;
    case PX4LITE_MODULE_ESTIMATOR: return &s_acn_mod_est;
    case PX4LITE_MODULE_BUSINESS: return &s_acn_mod_business;
    default: break;
  }

  switch (code) {
    case PX4LITE_FAULT_SYSTEM_SELF_CHECK:
    case PX4LITE_FAULT_SYSTEM_HEAP:
    case PX4LITE_FAULT_SYSTEM_STACK:
    case PX4LITE_FAULT_SYSTEM_TASK_LOST: return &s_acn_mod_system;
    case PX4LITE_FAULT_SENSOR_INIT:
    case PX4LITE_FAULT_SENSOR_OFFLINE:
    case PX4LITE_FAULT_SENSOR_INVALID:
    case PX4LITE_FAULT_SENSOR_NO_FIX:
    case PX4LITE_FAULT_SENSOR_TIMEOUT: return &s_acn_mod_sensor;
    case PX4LITE_FAULT_COMM_OFFLINE:
    case PX4LITE_FAULT_COMM_TIMEOUT:
    case PX4LITE_FAULT_COMM_FRAME: return &s_acn_mod_comm;
    case PX4LITE_FAULT_DISPLAY_OFFLINE:
    case PX4LITE_FAULT_DISPLAY_REFRESH: return &s_acn_mod_display;
    case PX4LITE_FAULT_STORAGE_NOT_READY:
    case PX4LITE_FAULT_STORAGE_WRITE:
    case PX4LITE_FAULT_STORAGE_FULL: return &s_acn_mod_storage;
    case PX4LITE_FAULT_PROTOCOL_PARSE:
    case PX4LITE_FAULT_PROTOCOL_CRC: return &s_acn_mod_proto;
    case PX4LITE_FAULT_APP_INVALID_DATA: return &s_acn_mod_app;
    case PX4LITE_FAULT_ESTIMATOR_INPUT:
    case PX4LITE_FAULT_ESTIMATOR_DIVERGE: return &s_acn_mod_est;
    case PX4LITE_FAULT_MOTOR_DISCONNECT:
    case PX4LITE_FAULT_MOTOR_POWER_LOW: return &s_acn_mod_motor;
    default: break;
  }

  return &s_acn_mod_system;
}

/* 告警页原因栏中文字模。 */
static const Display_AlarmCn_t *Display_PagesGetAlarmReasonCn(uint32_t code)
{
  switch ((uint16_t)code) {
    case PX4LITE_FAULT_SYSTEM_SELF_CHECK: return &s_acn_rsn_selfcheck;
    case PX4LITE_FAULT_SYSTEM_HEAP: return &s_acn_rsn_heap;
    case PX4LITE_FAULT_SYSTEM_STACK: return &s_acn_rsn_stack;
    case PX4LITE_FAULT_SYSTEM_TASK_LOST: return &s_acn_rsn_tasklost;
    case PX4LITE_FAULT_SENSOR_INIT: return &s_acn_rsn_sensorinit;
    case PX4LITE_FAULT_SENSOR_OFFLINE: return &s_acn_rsn_sensoroffline;
    case PX4LITE_FAULT_SENSOR_INVALID: return &s_acn_rsn_invalid;
    case PX4LITE_FAULT_SENSOR_NO_FIX: return &s_acn_rsn_nofix;
    case PX4LITE_FAULT_SENSOR_TIMEOUT: return &s_acn_rsn_sensortimeout;
    case PX4LITE_FAULT_COMM_OFFLINE: return &s_acn_rsn_commoffline;
    case PX4LITE_FAULT_COMM_TIMEOUT: return &s_acn_rsn_commtimeout;
    case PX4LITE_FAULT_COMM_FRAME: return &s_acn_rsn_commframe;
    case PX4LITE_FAULT_DISPLAY_OFFLINE: return &s_acn_rsn_dispoffline;
    case PX4LITE_FAULT_DISPLAY_REFRESH: return &s_acn_rsn_disprefresh;
    case PX4LITE_FAULT_STORAGE_NOT_READY: return &s_acn_rsn_storenotready;
    case PX4LITE_FAULT_STORAGE_WRITE: return &s_acn_rsn_storewrite;
    case PX4LITE_FAULT_STORAGE_FULL: return &s_acn_rsn_storefull;
    case PX4LITE_FAULT_PROTOCOL_PARSE: return &s_acn_rsn_protoparse;
    case PX4LITE_FAULT_PROTOCOL_CRC: return &s_acn_rsn_protocrc;
    case PX4LITE_FAULT_APP_INVALID_DATA: return &s_acn_rsn_invalid;
    case PX4LITE_FAULT_ESTIMATOR_INPUT: return &s_acn_rsn_estinput;
    case PX4LITE_FAULT_ESTIMATOR_DIVERGE: return &s_acn_rsn_estdiverge;
    case PX4LITE_FAULT_MOTOR_DISCONNECT: return &s_acn_rsn_motordisc;
    case PX4LITE_FAULT_MOTOR_POWER_LOW: return &s_acn_rsn_powerlow;
    /* 电机没电/需充电、主控需充电 用消息日志字模拼绘，不走宽位图，见 Display_PagesAlarmReasonLogMsg。 */
    case PX4LITE_FAULT_MOTOR_DEAD:
    case PX4LITE_FAULT_MOTOR_CHARGE:
    case PX4LITE_FAULT_POWER_CHARGE: return 0;
    default: break;
  }

  return &s_acn_rsn_unknown;
}

/* 电池类告警原因改用消息日志字模拼绘(复用 没/充/需/电/池 等单字)；
   返回 DISPLAY_LOGMSG_COUNT 表示该故障走原有中文宽位图/ASCII。 */
static Display_LogMsg_t Display_PagesAlarmReasonLogMsg(uint16_t code)
{
  if (code == (uint16_t)PX4LITE_FAULT_MOTOR_DEAD) { return DISPLAY_LOGMSG_MOTOR_DEAD; }
  if (code == (uint16_t)PX4LITE_FAULT_MOTOR_CHARGE) { return DISPLAY_LOGMSG_MOTOR_CHARGE; }
  if (code == (uint16_t)PX4LITE_FAULT_POWER_CHARGE) { return DISPLAY_LOGMSG_MAIN_CHARGE; }
  return DISPLAY_LOGMSG_COUNT;
}

/* ============================ LoRa 连接页（隐藏页） ============================ */
#define DISPLAY_LORA_LIST_X     8U
#define DISPLAY_LORA_LIST_Y     72U
#define DISPLAY_LORA_LIST_W     580U
#define DISPLAY_LORA_LIST_H     400U
#define DISPLAY_LORA_HEAD_Y     104U /* 标题行下分隔线 */
#define DISPLAY_LORA_ROW_Y0     108U
#define DISPLAY_LORA_ROW_H      52U
#define DISPLAY_LORA_ROW_RIGHT  552U /* 行内容右界(滚动列左侧) */
#define DISPLAY_LORA_DOT_CX     30U
#define DISPLAY_LORA_NAME_X     48U
#define DISPLAY_LORA_ID_X       250U
#define DISPLAY_LORA_ONLINE_X   430U
#define DISPLAY_LORA_SCROLL_X   556U
#define DISPLAY_LORA_SCROLL_W   28U
#define DISPLAY_LORA_UP_Y       108U
#define DISPLAY_LORA_DOWN_Y     424U
#define DISPLAY_LORA_SBTN_H     40U
#define DISPLAY_LORA_TRACK_X    562U
#define DISPLAY_LORA_TRACK_Y    152U
#define DISPLAY_LORA_TRACK_W    16U
#define DISPLAY_LORA_TRACK_H    268U
#define DISPLAY_LORA_INFO_X     596U
#define DISPLAY_LORA_INFO_Y     72U
#define DISPLAY_LORA_INFO_W     196U
#define DISPLAY_LORA_INFO_H     300U
#define DISPLAY_LORA_CBTN_X     596U
#define DISPLAY_LORA_CBTN_Y     388U
#define DISPLAY_LORA_CBTN_W     196U
#define DISPLAY_LORA_CBTN_H     64U
#define DISPLAY_LORA_SEL_FILL   0x0B6AU             /* 深青选中底色 */
#define DISPLAY_LORA_TRACK_BG   DISPLAY_THEME_PANEL_HI /* 滚动条轨道底色 */
#define DISPLAY_LORA_NO_SEL     0xFFFFU

static Display_LoraNode_t s_lora_nodes[DISPLAY_LORA_MAX_NODES];
static uint16_t s_lora_count;
static uint16_t s_lora_sel_id        = DISPLAY_LORA_NO_SEL; /* 当前选中节点 ID */
static uint16_t s_lora_page;                               /* 当前页，0 基 */
static uint8_t s_lora_connected;                           /* 是否已连接 */
static uint16_t s_lora_conn_id;                            /* 已连接的节点 ID */
static uint32_t s_lora_version;                            /* 内容版本号 */
static uint32_t s_lora_drawn_version = 0xFFFFFFFFU;        /* 上次已绘制版本 */
static Display_LoraConnectHandler_t s_lora_handler;

static uint16_t Display_LoraPageCount(void)
{
  if (s_lora_count == 0U) { return 1U; }
  return (uint16_t)((s_lora_count + DISPLAY_LORA_ROWS_PER_PAGE - 1U) / DISPLAY_LORA_ROWS_PER_PAGE);
}

/* 返回当前选中节点在列表中的下标，未选中或不在表内返回 -1。 */
static int32_t Display_LoraSelIndex(void)
{
  uint16_t i;
  if (s_lora_sel_id == DISPLAY_LORA_NO_SEL) { return -1; }
  for (i = 0U; i < s_lora_count; i++) {
    if (s_lora_nodes[i].node_id == s_lora_sel_id) { return (int32_t)i; }
  }
  return -1;
}

static char Display_LoraHexDigit(uint8_t v)
{
  v = (uint8_t)(v & 0x0FU);
  return (v < 10U) ? (char)('0' + v) : (char)('A' + (v - 10U));
}

/* 把节点 ID 格式化为 "0xXX"（>0xFF 时 4 位）。 */
static void Display_LoraFmtId(char *buf, uint16_t id)
{
  uint8_t p = 0U;
  buf[p++]  = '0';
  buf[p++]  = 'x';
  if (id > 0xFFU) {
    buf[p++] = Display_LoraHexDigit((uint8_t)(id >> 12));
    buf[p++] = Display_LoraHexDigit((uint8_t)(id >> 8));
  }
  buf[p++] = Display_LoraHexDigit((uint8_t)(id >> 4));
  buf[p++] = Display_LoraHexDigit((uint8_t)id);
  buf[p]   = '\0';
}

/* 把数值格式化为两位十进制（截断到 0..99）。 */
static void Display_LoraFmt2(char *buf, uint16_t v)
{
  if (v > 99U) { v = (uint16_t)(v % 100U); }
  buf[0] = (char)('0' + (v / 10U));
  buf[1] = (char)('0' + (v % 10U));
  buf[2] = '\0';
}

void Display_PagesSetLoraNodes(const Display_LoraNode_t *nodes, uint16_t count)
{
  uint16_t i;

  if (nodes == 0) { count = 0U; }
  if (count > DISPLAY_LORA_MAX_NODES) { count = DISPLAY_LORA_MAX_NODES; }
  for (i = 0U; i < count; i++) { s_lora_nodes[i] = nodes[i]; }
  s_lora_count = count;

  if (Display_LoraSelIndex() < 0) { s_lora_sel_id = (count > 0U) ? s_lora_nodes[0].node_id : DISPLAY_LORA_NO_SEL; }
  if (s_lora_page >= Display_LoraPageCount()) { s_lora_page = (uint16_t)(Display_LoraPageCount() - 1U); }
  s_lora_version++;
}

void Display_PagesSetLoraConnected(uint8_t connected, uint16_t node_id)
{
  s_lora_connected = (connected != 0U) ? 1U : 0U;
  s_lora_conn_id   = node_id;
  s_lora_version++;
}

void Display_PagesSetLoraConnectHandler(Display_LoraConnectHandler_t handler)
{
  s_lora_handler = handler;
}

uint32_t Display_PagesGetLoraVersion(void)
{
  return s_lora_version;
}

uint8_t Display_PagesLoraContentDirty(void)
{
  return (s_lora_version != s_lora_drawn_version) ? 1U : 0U;
}

/* 是否已连接到指定节点。 */
static uint8_t Display_LoraIsConnectedTo(uint16_t node_id)
{
  return ((s_lora_connected != 0U) && (s_lora_conn_id == node_id)) ? 1U : 0U;
}

/* 绘制一个三角箭头(up 非 0 向上，否则向下)。 */
static void Display_LoraDrawArrow(uint16_t cx, uint16_t cy, uint8_t up)
{
  uint16_t i;
  for (i = 0U; i < 7U; i++) {
    uint16_t w  = (uint16_t)(2U + (i * 2U));
    uint16_t yy = (up != 0U) ? (uint16_t)(cy - 6U + i) : (uint16_t)(cy + 6U - i);
    (void)Display_GfxFillRect((uint16_t)(cx - (w / 2U)), yy, w, 1U, DISPLAY_THEME_TEXT);
  }
}

static void Display_LoraDrawRow(uint16_t row, uint16_t idx)
{
  uint16_t top = (uint16_t)(DISPLAY_LORA_ROW_Y0 + (row * DISPLAY_LORA_ROW_H));
  uint16_t ty  = (uint16_t)(top + 18U);
  uint8_t sel  = (s_lora_nodes[idx].node_id == s_lora_sel_id) ? 1U : 0U;
  char buf[8];

  (void)Display_GfxFillRect(9U, top, (uint16_t)(DISPLAY_LORA_ROW_RIGHT - 9U), DISPLAY_LORA_ROW_H, sel ? DISPLAY_LORA_SEL_FILL : DISPLAY_THEME_PANEL);
  if (sel != 0U) { (void)Display_GfxFillRect(9U, top, 4U, DISPLAY_LORA_ROW_H, DISPLAY_GFX_COLOR_BLUE); }

  (void)Display_GfxDrawStatusDot(DISPLAY_LORA_DOT_CX, (uint16_t)(top + (DISPLAY_LORA_ROW_H / 2U)), 6U, DISPLAY_GFX_COLOR_GREEN, DISPLAY_GFX_COLOR_BLACK);
  (void)Display_TextDrawNodeLabel(DISPLAY_LORA_NAME_X, ty, DISPLAY_THEME_TEXT);
  Display_LoraFmt2(buf, s_lora_nodes[idx].label);
  (void)Display_GfxDrawString((uint16_t)(DISPLAY_LORA_NAME_X + 38U), (uint16_t)(ty + 1U), buf, DISPLAY_THEME_TEXT, 2U);
  Display_LoraFmtId(buf, s_lora_nodes[idx].node_id);
  (void)Display_GfxDrawString(DISPLAY_LORA_ID_X, (uint16_t)(ty + 1U), buf, DISPLAY_THEME_TEXT, 2U);
  (void)Display_TextDrawLabel(DISPLAY_LORA_ONLINE_X, ty, DISPLAY_TXT_ONLINE, DISPLAY_GFX_COLOR_GREEN);

  (void)Display_GfxDrawHLine(DISPLAY_LORA_LIST_X, (uint16_t)(top + DISPLAY_LORA_ROW_H), (uint16_t)(DISPLAY_LORA_ROW_RIGHT - DISPLAY_LORA_LIST_X), DISPLAY_GFX_COLOR_GRAY);
}

static void Display_LoraDrawInfo(void)
{
  int32_t sel = Display_LoraSelIndex();
  char buf[12];
  uint16_t x = (uint16_t)(DISPLAY_LORA_INFO_X + 14U);
  uint16_t status_w;

  (void)Display_GfxFillRect((uint16_t)(DISPLAY_LORA_INFO_X + 1U), 106U, (uint16_t)(DISPLAY_LORA_INFO_W - 2U), (uint16_t)((DISPLAY_LORA_INFO_Y + DISPLAY_LORA_INFO_H) - 107U), DISPLAY_THEME_PANEL);
  if (sel < 0) { return; }

  (void)Display_TextDrawNodeLabel(x, 120U, DISPLAY_THEME_TEXT);
  Display_LoraFmt2(buf, s_lora_nodes[sel].label);
  (void)Display_GfxDrawString((uint16_t)(x + 38U), 121U, buf, DISPLAY_THEME_TEXT, 2U);

  (void)Display_GfxDrawString(x, 162U, "ID", DISPLAY_GFX_COLOR_GRAY, 2U);
  Display_LoraFmtId(buf, s_lora_nodes[sel].node_id);
  (void)Display_GfxDrawString((uint16_t)(x + 42U), 162U, buf, DISPLAY_THEME_TEXT, 2U);

  (void)Display_TextDrawLabel(x, 202U, DISPLAY_TXT_STATUS, DISPLAY_THEME_TEXT);
  status_w = (uint16_t)(Display_TextGetLabelWidth(DISPLAY_TXT_STATUS) + 8U);
  if (Display_LoraIsConnectedTo(s_lora_nodes[sel].node_id) != 0U) {
    (void)Display_TextDrawConnectedLabel((uint16_t)(x + status_w), 202U, DISPLAY_GFX_COLOR_GREEN);
  } else {
    (void)Display_TextDrawWaitConnectLabel((uint16_t)(x + status_w), 202U, DISPLAY_GFX_COLOR_YELLOW);
  }

  (void)Display_TextDrawLastCommLabel(x, 246U, DISPLAY_GFX_COLOR_GRAY);
  Display_PagesFmtClock(buf, s_lora_nodes[sel].last_comm_hhmmss);
  (void)Display_GfxDrawString(x, 274U, buf, DISPLAY_THEME_TEXT, 2U);
}

static void Display_LoraDrawButton(void)
{
  int32_t sel   = Display_LoraSelIndex();
  uint8_t conn  = (sel >= 0) ? Display_LoraIsConnectedTo(s_lora_nodes[sel].node_id) : 0U;
  uint16_t fill = (sel < 0) ? DISPLAY_GFX_COLOR_GRAY : ((conn != 0U) ? DISPLAY_GFX_COLOR_RED : DISPLAY_GFX_COLOR_GREEN);
  uint16_t lx   = (uint16_t)(DISPLAY_LORA_CBTN_X + ((DISPLAY_LORA_CBTN_W - 32U) / 2U));
  uint16_t ly   = (uint16_t)(DISPLAY_LORA_CBTN_Y + ((DISPLAY_LORA_CBTN_H - 16U) / 2U));

  (void)Display_GfxFillRect(DISPLAY_LORA_CBTN_X, DISPLAY_LORA_CBTN_Y, DISPLAY_LORA_CBTN_W, DISPLAY_LORA_CBTN_H, fill);
  if (conn != 0U) {
    (void)Display_TextDrawDisconnectLabel(lx, ly, DISPLAY_GFX_COLOR_WHITE);
  } else {
    (void)Display_TextDrawConnectLabel(lx, ly, DISPLAY_GFX_COLOR_WHITE);
  }
}

void Display_PagesDrawLoraContent(void)
{
  uint16_t row;
  uint16_t page_top;
  uint16_t pages;
  uint16_t online_w;
  uint16_t thumb_h;
  uint16_t thumb_y;
  char buf[8];
  char pbuf[6];

  if (Display_GfxIsReady() == 0U) { return; }

  pages = Display_LoraPageCount();
  if (s_lora_page >= pages) { s_lora_page = (uint16_t)(pages - 1U); }
  page_top = (uint16_t)(s_lora_page * DISPLAY_LORA_ROWS_PER_PAGE);

  /* 标题行右侧：在线数 + 页码 */
  (void)Display_GfxFillRect(300U, (uint16_t)(DISPLAY_LORA_LIST_Y + 6U), 244U, 22U, DISPLAY_THEME_PANEL);
  (void)Display_TextDrawLabel(300U, (uint16_t)(DISPLAY_LORA_LIST_Y + 8U), DISPLAY_TXT_ONLINE, DISPLAY_GFX_COLOR_GREEN);
  online_w = (uint16_t)(Display_TextGetLabelWidth(DISPLAY_TXT_ONLINE) + 6U);
  Display_LoraFmt2(buf, s_lora_count);
  (void)Display_GfxDrawString((uint16_t)(300U + online_w), (uint16_t)(DISPLAY_LORA_LIST_Y + 9U), buf, DISPLAY_THEME_TEXT, 2U);
  pbuf[0] = 'P';
  pbuf[1] = ' ';
  pbuf[2] = (char)('0' + ((s_lora_page + 1U) % 10U));
  pbuf[3] = '/';
  pbuf[4] = (char)('0' + (pages % 10U));
  pbuf[5] = '\0';
  (void)Display_GfxDrawString(456U, (uint16_t)(DISPLAY_LORA_LIST_Y + 9U), pbuf, DISPLAY_GFX_COLOR_GRAY, 2U);

  /* 列表正文 */
  (void)Display_GfxFillRect(9U, DISPLAY_LORA_ROW_Y0, (uint16_t)(DISPLAY_LORA_ROW_RIGHT - 9U), (uint16_t)((DISPLAY_LORA_LIST_Y + DISPLAY_LORA_LIST_H) - DISPLAY_LORA_ROW_Y0 - 1U), DISPLAY_THEME_PANEL);
  /* 列表为空时正文留白；标题行已显示“在线 00”表明无在线节点 */
  for (row = 0U; row < DISPLAY_LORA_ROWS_PER_PAGE; row++) {
    uint16_t idx = (uint16_t)(page_top + row);
    if (idx >= s_lora_count) { break; }
    Display_LoraDrawRow(row, idx);
  }

  /* 滚动条滑块 */
  (void)Display_GfxFillRect(DISPLAY_LORA_TRACK_X, DISPLAY_LORA_TRACK_Y, DISPLAY_LORA_TRACK_W, DISPLAY_LORA_TRACK_H, DISPLAY_LORA_TRACK_BG);
  thumb_h = (uint16_t)(DISPLAY_LORA_TRACK_H / pages);
  if (thumb_h < 24U) { thumb_h = 24U; }
  thumb_y = (pages > 1U) ? (uint16_t)(DISPLAY_LORA_TRACK_Y + (((DISPLAY_LORA_TRACK_H - thumb_h) * s_lora_page) / (pages - 1U))) : DISPLAY_LORA_TRACK_Y;
  (void)Display_GfxFillRect(DISPLAY_LORA_TRACK_X, thumb_y, DISPLAY_LORA_TRACK_W, thumb_h, DISPLAY_GFX_COLOR_GRAY);

  Display_LoraDrawInfo();
  Display_LoraDrawButton();

  s_lora_drawn_version = s_lora_version;
}

static void Display_PagesDrawLoraLayout(void)
{
  /* 列表外框 + 标题 "LoRa 连接" */
  Display_PagesDrawCardFrame(DISPLAY_LORA_LIST_X, DISPLAY_LORA_LIST_Y, DISPLAY_LORA_LIST_W, DISPLAY_LORA_LIST_H);
  (void)Display_GfxFillRect((uint16_t)(DISPLAY_LORA_LIST_X + 12U), (uint16_t)(DISPLAY_LORA_LIST_Y + 11U), 4U, 14U, DISPLAY_GFX_COLOR_BLUE);
  (void)Display_GfxDrawString((uint16_t)(DISPLAY_LORA_LIST_X + 22U), (uint16_t)(DISPLAY_LORA_LIST_Y + 9U), "LoRa", DISPLAY_GFX_COLOR_BLUE, 2U);
  (void)Display_TextDrawConnectLabel((uint16_t)(DISPLAY_LORA_LIST_X + 80U), (uint16_t)(DISPLAY_LORA_LIST_Y + 8U), DISPLAY_GFX_COLOR_BLUE);
  (void)Display_GfxDrawHLine(DISPLAY_LORA_LIST_X, DISPLAY_LORA_HEAD_Y, DISPLAY_LORA_LIST_W, DISPLAY_GFX_COLOR_CARD_DIVIDER);

  /* 滚动列分隔线 + 上下翻页按钮 + 轨道 */
  (void)Display_GfxDrawVLine((uint16_t)(DISPLAY_LORA_SCROLL_X - 4U), DISPLAY_LORA_HEAD_Y, (uint16_t)((DISPLAY_LORA_LIST_Y + DISPLAY_LORA_LIST_H) - DISPLAY_LORA_HEAD_Y), DISPLAY_GFX_COLOR_GRAY);
  (void)Display_GfxDrawFrame(DISPLAY_LORA_SCROLL_X, DISPLAY_LORA_UP_Y, DISPLAY_LORA_SCROLL_W, DISPLAY_LORA_SBTN_H, DISPLAY_GFX_COLOR_GRAY, DISPLAY_LORA_TRACK_BG);
  Display_LoraDrawArrow((uint16_t)(DISPLAY_LORA_SCROLL_X + (DISPLAY_LORA_SCROLL_W / 2U)), (uint16_t)(DISPLAY_LORA_UP_Y + (DISPLAY_LORA_SBTN_H / 2U)), 1U);
  (void)Display_GfxDrawFrame(DISPLAY_LORA_SCROLL_X, DISPLAY_LORA_DOWN_Y, DISPLAY_LORA_SCROLL_W, DISPLAY_LORA_SBTN_H, DISPLAY_GFX_COLOR_GRAY, DISPLAY_LORA_TRACK_BG);
  Display_LoraDrawArrow((uint16_t)(DISPLAY_LORA_SCROLL_X + (DISPLAY_LORA_SCROLL_W / 2U)), (uint16_t)(DISPLAY_LORA_DOWN_Y + (DISPLAY_LORA_SBTN_H / 2U)), 0U);

  /* 右侧详情框 + 标题 "当前选择" */
  Display_PagesDrawCardFrame(DISPLAY_LORA_INFO_X, DISPLAY_LORA_INFO_Y, DISPLAY_LORA_INFO_W, DISPLAY_LORA_INFO_H);
  (void)Display_GfxFillRect((uint16_t)(DISPLAY_LORA_INFO_X + 12U), (uint16_t)(DISPLAY_LORA_INFO_Y + 11U), 4U, 14U, DISPLAY_GFX_COLOR_BLUE);
  (void)Display_TextDrawSelectTitle((uint16_t)(DISPLAY_LORA_INFO_X + 22U), (uint16_t)(DISPLAY_LORA_INFO_Y + 9U), DISPLAY_GFX_COLOR_BLUE);
  (void)Display_GfxDrawHLine(DISPLAY_LORA_INFO_X, (uint16_t)(DISPLAY_LORA_INFO_Y + 32U), DISPLAY_LORA_INFO_W, DISPLAY_GFX_COLOR_CARD_DIVIDER);

  Display_PagesDrawLoraContent();
}

Display_LoraTouchResult_t Display_PagesLoraHandleTouch(uint16_t x, uint16_t y)
{
  uint16_t pages = Display_LoraPageCount();

  /* 连接/断开按钮 */
  if ((x >= DISPLAY_LORA_CBTN_X) && (x < (DISPLAY_LORA_CBTN_X + DISPLAY_LORA_CBTN_W)) && (y >= DISPLAY_LORA_CBTN_Y) && (y < (DISPLAY_LORA_CBTN_Y + DISPLAY_LORA_CBTN_H))) {
    int32_t sel = Display_LoraSelIndex();
    uint16_t id;
    uint8_t want_connect;
    if (sel < 0) { return DISPLAY_LORA_TOUCH_NONE; }
    id           = s_lora_nodes[sel].node_id;
    want_connect = (Display_LoraIsConnectedTo(id) != 0U) ? 0U : 1U;
    if (want_connect != 0U) {
      s_lora_connected = 1U;
      s_lora_conn_id   = id;
    } else {
      s_lora_connected = 0U;
    }
    if (s_lora_handler != 0) { s_lora_handler(id, want_connect); }
    s_lora_version++;
    return DISPLAY_LORA_TOUCH_COMMAND;
  }

  /* 上翻页 */
  if ((x >= DISPLAY_LORA_SCROLL_X) && (x < (DISPLAY_LORA_SCROLL_X + DISPLAY_LORA_SCROLL_W)) && (y >= DISPLAY_LORA_UP_Y) && (y < (DISPLAY_LORA_UP_Y + DISPLAY_LORA_SBTN_H))) {
    if (s_lora_page > 0U) {
      s_lora_page--;
      s_lora_version++;
      return DISPLAY_LORA_TOUCH_REDRAW;
    }
    return DISPLAY_LORA_TOUCH_NONE;
  }

  /* 下翻页 */
  if ((x >= DISPLAY_LORA_SCROLL_X) && (x < (DISPLAY_LORA_SCROLL_X + DISPLAY_LORA_SCROLL_W)) && (y >= DISPLAY_LORA_DOWN_Y) && (y < (DISPLAY_LORA_DOWN_Y + DISPLAY_LORA_SBTN_H))) {
    if ((uint16_t)(s_lora_page + 1U) < pages) {
      s_lora_page++;
      s_lora_version++;
      return DISPLAY_LORA_TOUCH_REDRAW;
    }
    return DISPLAY_LORA_TOUCH_NONE;
  }

  /* 列表行选中 */
  if ((x >= DISPLAY_LORA_LIST_X) && (x < DISPLAY_LORA_ROW_RIGHT) && (y >= DISPLAY_LORA_ROW_Y0) && (y < (uint16_t)(DISPLAY_LORA_ROW_Y0 + (DISPLAY_LORA_ROWS_PER_PAGE * DISPLAY_LORA_ROW_H)))) {
    uint16_t row = (uint16_t)((y - DISPLAY_LORA_ROW_Y0) / DISPLAY_LORA_ROW_H);
    uint16_t idx = (uint16_t)((s_lora_page * DISPLAY_LORA_ROWS_PER_PAGE) + row);
    if ((idx < s_lora_count) && (s_lora_sel_id != s_lora_nodes[idx].node_id)) {
      s_lora_sel_id = s_lora_nodes[idx].node_id;
      s_lora_version++;
      return DISPLAY_LORA_TOUCH_REDRAW;
    }
    return DISPLAY_LORA_TOUCH_NONE;
  }

  return DISPLAY_LORA_TOUCH_NONE;
}

/*
 * 获取当前页面的上一页，首尾循环。
 */
Display_HmiPage_t Display_PagesGetPrevPage(Display_HmiPage_t page)
{
  if (page == DISPLAY_HMI_PAGE_SELF_CHECK) { return DISPLAY_HMI_PAGE_ALARM; }

  return (Display_HmiPage_t)((uint16_t)page - 1U);
}

/*
 * 获取当前页面的下一页，首尾循环。
 */
Display_HmiPage_t Display_PagesGetNextPage(Display_HmiPage_t page)
{
  /* 隐藏页不参与翻页循环，导航在自检页与告警页之间首尾相接 */
  if (((uint16_t)page + 1U) >= DISPLAY_HMI_PAGE_HIDDEN) { return DISPLAY_HMI_PAGE_SELF_CHECK; }

  return (Display_HmiPage_t)((uint16_t)page + 1U);
}

/*
 * 在页眉中绘制 LoRa 丢包率文本。
 */
static void Display_PagesDrawHeaderLossRate(uint16_t x, uint16_t y, uint32_t loss_rate)
{
  uint16_t rate;
  static char buf[10];
  uint8_t p;

  rate = (loss_rate > 1000U) ? 1000U : (uint16_t)loss_rate;
  p    = 0U;
  if (rate >= 1000U) {
    buf[p++] = '1';
    buf[p++] = '0';
    buf[p++] = '0';
  } else {
    if (rate >= 100U) { buf[p++] = (char)('0' + (rate / 100U) % 10U); }
    buf[p++] = (char)('0' + (rate / 10U) % 10U);
  }
  buf[p++] = '.';
  buf[p++] = (char)('0' + rate % 10U);
  buf[p++] = '%';
  buf[p]   = '\0';

  (void)Display_GfxDrawString(x, y, "L:", DISPLAY_GFX_COLOR_CYAN, 1U);
  (void)Display_GfxDrawString((uint16_t)(x + 14U), y, buf, DISPLAY_GFX_COLOR_CYAN, 1U);
}

/*
 * 绘制页面头部：Logo、居中公司标题、底部分隔线；
 * 左侧日期/时间与右侧电池、丢包率由 Display_PagesDrawHeaderDynamic 绘制。
 */
Display_Result_t Display_PagesDrawHeader(Display_HmiPage_t page, Display_PagesValueReader_t read_value)
{
  uint16_t title_width;
  uint16_t title_x;

  (void)page;

  if (Display_GfxIsReady() == 0U) { return DISPLAY_NOT_READY; }

  (void)Display_GfxFillRect(0U, 0U, DISPLAY_GFX_WIDTH, DISPLAY_HEADER_HEIGHT, DISPLAY_THEME_PANEL);
  (void)Display_PagesDrawHeaderLogo(0U, 0U);
  title_width = Display_TextGetCompanyTitleWidth();
  title_x     = (DISPLAY_GFX_WIDTH > title_width) ? (uint16_t)((DISPLAY_GFX_WIDTH - title_width) / 2U) : 0U;
  (void)Display_TextDrawCompanyTitle(title_x, 18U, DISPLAY_GFX_COLOR_WHITE);
  (void)Display_PagesDrawHeaderDynamic(read_value, 1U);
  (void)Display_GfxDrawHLine(0U, (uint16_t)(DISPLAY_HEADER_HEIGHT - 1U), DISPLAY_GFX_WIDTH, DISPLAY_GFX_COLOR_CYAN);

  return DISPLAY_OK;
}

/*
 * 绘制指定页面的固定背景、标签和导航区域。
 */
Display_Result_t Display_PagesDrawStatic(Display_HmiPage_t page, Display_PagesValueReader_t read_value)
{
  if (Display_GfxIsReady() == 0U) { return DISPLAY_NOT_READY; }

  (void)Display_GfxClear(DISPLAY_THEME_BG);

  if (page == DISPLAY_HMI_PAGE_SELF_CHECK) {
    Display_PagesDrawSelfCheckLayout();
  } else if (page == DISPLAY_HMI_PAGE_FLIGHT) {
    Display_PagesDrawFlightLayout();
  } else if (page == DISPLAY_HMI_PAGE_AIRCRAFT) {
    Display_PagesDrawAircraftLayout();
  } else if (page == DISPLAY_HMI_PAGE_DATA) {
    Display_PagesDrawDataDashboardLayout();
  } else if (page == DISPLAY_HMI_PAGE_MOTOR) {
    Display_PagesDrawMotorLayout();
  } else if (page == DISPLAY_HMI_PAGE_ALARM) {
    Display_PagesDrawAlarmStaticLayout();
  } else if (page == DISPLAY_HMI_PAGE_HIDDEN) {
    Display_PagesDrawLoraLayout(); /* LoRa 连接页：列表 + 详情 + 连接按钮，无底部翻页栏 */
  } else {
    return DISPLAY_ERROR;
  }

  (void)Display_PagesDrawHeader(page, read_value);
  if (page != DISPLAY_HMI_PAGE_HIDDEN) { Display_PagesDrawFooter(page); }

  return DISPLAY_OK;
}

/*
 * 绘制无符号 32 位整数量
 */
static uint16_t Display_PagesDrawU32Scaled(uint16_t x, uint16_t y, uint32_t value, uint16_t color, uint8_t scale)
{
  static char buf[11];
  uint8_t len;
  uint8_t i;

  if (value == 0U) {
    (void)Display_GfxDrawChar(x, y, '0', color, scale);
    return (uint16_t)(6U * scale);
  }

  len = 0U;
  while (value > 0U && len < 10U) {
    buf[len] = (char)('0' + (value % 10U));
    len++;
    value /= 10U;
  }

  for (i = len; i > 0U; i--) {
    (void)Display_GfxDrawChar(x, y, buf[i - 1U], color, scale);
    x = (uint16_t)(x + (6U * scale));
  }

  return (uint16_t)(len * 6U * scale);
}

/*
 * 绘制无符号 32 位整数，返回绘制像素宽度。
 */
static uint16_t Display_PagesDrawU32(uint16_t x, uint16_t y, uint32_t value, uint16_t color)
{
  return Display_PagesDrawU32Scaled(x, y, value, color, 2U);
}

static char Display_PagesHexDigit(uint8_t value)
{
  value = (uint8_t)(value & 0x0FU);
  return (value < 10U) ? (char)('0' + value) : (char)('A' + (value - 10U));
}

static void Display_PagesFormatFaultCode(char *buf, uint16_t code)
{
  buf[0] = '0';
  buf[1] = 'x';
  buf[2] = Display_PagesHexDigit((uint8_t)(code >> 12));
  buf[3] = Display_PagesHexDigit((uint8_t)(code >> 8));
  buf[4] = Display_PagesHexDigit((uint8_t)(code >> 4));
  buf[5] = Display_PagesHexDigit((uint8_t)code);
  buf[6] = '\0';
}

/*
 * 自检页错误码表当前的激活故障列表，每项为 (source_id << 16) | fault_code。
 * 由 display 层在每次加载告警快照时刷新；故障恢复后不再出现在列表中，
 * 对应行随之消失。绘制由 SELF_CHECK_ERROR_CODE 变量的集合指纹触发。
 */
static uint32_t s_selfcheck_faults[DISPLAY_SELFCHECK_ERR_MAX_ROWS];
static uint16_t s_selfcheck_fault_count;

void Display_PagesSetSelfCheckFaults(const uint32_t *faults, uint16_t count)
{
  uint16_t i;

  if ((faults == 0) || (count == 0U)) {
    s_selfcheck_fault_count = 0U;
    return;
  }

  if (count > DISPLAY_SELFCHECK_ERR_MAX_ROWS) { count = DISPLAY_SELFCHECK_ERR_MAX_ROWS; }

  for (i = 0U; i < count; i++) { s_selfcheck_faults[i] = faults[i]; }
  s_selfcheck_fault_count = count;
}

/*
 * 绘制自检页错误码表正文：逐条列出当前激活故障，有几条画几条；
 * 无故障时显示占位文本。传入的 value 仅用作集合指纹触发重绘，不参与内容。
 */
static void Display_PagesDrawSelfCheckErrorField(uint32_t value)
{
  char code_buf[7];
  uint16_t i;

  (void)value;

  (void)Display_GfxFillRect(481U, DISPLAY_SELFCHECK_ERR_BODY_Y, 310U, DISPLAY_SELFCHECK_ERR_BODY_H, DISPLAY_THEME_PANEL);
  (void)Display_GfxDrawHLine(480U, 152U, 312U, DISPLAY_GFX_COLOR_CARD_DIVIDER);

  if (s_selfcheck_fault_count == 0U) {
    (void)Display_TextDrawLabel(604U, DISPLAY_SELFCHECK_ERR_ROW_Y0, DISPLAY_TEXT_FAILED_MODULES, DISPLAY_GFX_COLOR_GRAY);
    return;
  }

  for (i = 0U; i < s_selfcheck_fault_count; i++) {
    uint16_t row_y      = (uint16_t)(DISPLAY_SELFCHECK_ERR_ROW_Y0 + (i * DISPLAY_SELFCHECK_ERR_ROW_H));
    uint16_t source_id  = (uint16_t)(s_selfcheck_faults[i] >> 16);
    uint16_t fault_code = (uint16_t)s_selfcheck_faults[i];

    Display_PagesFormatFaultCode(code_buf, fault_code);
    (void)Display_GfxDrawString(DISPLAY_SELFCHECK_ERR_CODE_X, row_y, code_buf, DISPLAY_GFX_COLOR_RED, 1U);
    (void)Display_GfxDrawString(DISPLAY_SELFCHECK_ERR_MODULE_X, row_y, Display_PagesGetAlarmModule(source_id, fault_code), DISPLAY_THEME_TEXT, 1U);
    (void)Display_GfxDrawString(DISPLAY_SELFCHECK_ERR_REASON_X, row_y, Display_PagesGetAlarmReason(fault_code), DISPLAY_THEME_TEXT, 1U);
  }
}

static void Display_PagesDrawAlarmSummaryField(const Display_HmiVariableConfig_t *variable, uint32_t value)
{
  (void)value;

  (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
  Display_PagesDrawAlarmTitle(variable->x, variable->y);
}

static uint16_t Display_PagesGetAlarmRowIndex(const Display_HmiVariableConfig_t *variable)
{
  if (variable->y <= DISPLAY_ALARM_BODY_Y) { return 0U; }

  return (uint16_t)((variable->y - DISPLAY_ALARM_BODY_Y) / DISPLAY_ALARM_ROW_H);
}

static uint8_t Display_PagesIsAlarmRow(Display_HmiVariableId_t id)
{
  return ((id == DISPLAY_HMI_VAR_ALARM_ROW1_CODE) || (id == DISPLAY_HMI_VAR_ALARM_ROW2_CODE) || (id == DISPLAY_HMI_VAR_ALARM_ROW3_CODE) || (id == DISPLAY_HMI_VAR_ALARM_ROW4_CODE) || (id == DISPLAY_HMI_VAR_ALARM_ROW5_CODE)) ? 1U : 0U;
}

static void Display_PagesDrawAlarmRow(const Display_HmiVariableConfig_t *variable, uint32_t value)
{
  char code_buf[7];
  uint16_t source_id  = (uint16_t)(value >> 16);
  uint16_t fault_code = (uint16_t)value;
  uint16_t row        = Display_PagesGetAlarmRowIndex(variable);
  uint16_t row_y      = (uint16_t)(DISPLAY_ALARM_BODY_Y + (row * DISPLAY_ALARM_ROW_H));

  (void)Display_GfxFillRect((uint16_t)(DISPLAY_ALARM_TABLE_X + 1U), (uint16_t)(row_y + 1U), (uint16_t)(DISPLAY_ALARM_TABLE_W - 2U), (uint16_t)(DISPLAY_ALARM_ROW_H - 1U), Display_PagesGetAlarmRowFill(row));
  (void)Display_GfxDrawHLine(DISPLAY_ALARM_TABLE_X, row_y, DISPLAY_ALARM_TABLE_W, DISPLAY_ALARM_COLOR_GRID);
  (void)Display_GfxDrawHLine(DISPLAY_ALARM_TABLE_X, (uint16_t)(row_y + DISPLAY_ALARM_ROW_H), DISPLAY_ALARM_TABLE_W, DISPLAY_ALARM_COLOR_GRID);
  (void)Display_GfxDrawVLine(DISPLAY_ALARM_TABLE_X, row_y, DISPLAY_ALARM_ROW_H, DISPLAY_ALARM_COLOR_GRID);
  (void)Display_GfxDrawVLine(DISPLAY_ALARM_CODE_X, row_y, DISPLAY_ALARM_ROW_H, DISPLAY_ALARM_COLOR_GRID);
  (void)Display_GfxDrawVLine(DISPLAY_ALARM_MODULE_X, row_y, DISPLAY_ALARM_ROW_H, DISPLAY_ALARM_COLOR_GRID);
  (void)Display_GfxDrawVLine((uint16_t)(DISPLAY_ALARM_TABLE_X + DISPLAY_ALARM_TABLE_W - 1U), row_y, DISPLAY_ALARM_ROW_H, DISPLAY_ALARM_COLOR_GRID);

  if (fault_code != 0U) {
    const Display_AlarmCn_t *module_cn = Display_PagesGetAlarmModuleCn(source_id, fault_code);
    const Display_AlarmCn_t *reason_cn = Display_PagesGetAlarmReasonCn(fault_code);

    /* 激活行左侧红色严重度色条，与列表卡片观感一致。 */
    (void)Display_GfxFillRect((uint16_t)(DISPLAY_ALARM_TABLE_X + 1U), (uint16_t)(row_y + 1U), 5U, (uint16_t)(DISPLAY_ALARM_ROW_H - 1U), DISPLAY_GFX_COLOR_RED);

    Display_PagesFormatFaultCode(code_buf, fault_code);
    (void)Display_GfxDrawString((uint16_t)(DISPLAY_ALARM_TABLE_X + 30U), (uint16_t)(row_y + 16U), code_buf, DISPLAY_GFX_COLOR_RED, 2U);

    if (module_cn != 0) {
      (void)Display_TextDrawRawBitmap((uint16_t)(DISPLAY_ALARM_CODE_X + 20U), (uint16_t)(row_y + 15U), module_cn->width, 16U, module_cn->bpr, module_cn->data, DISPLAY_THEME_TEXT);
    } else {
      (void)Display_GfxDrawString((uint16_t)(DISPLAY_ALARM_CODE_X + 20U), (uint16_t)(row_y + 16U), Display_PagesGetAlarmModule(source_id, fault_code), DISPLAY_THEME_TEXT, 2U);
    }

    {
      Display_LogMsg_t reason_msg = Display_PagesAlarmReasonLogMsg(fault_code);
      if (reason_msg != DISPLAY_LOGMSG_COUNT) {
        (void)Display_TextDrawLogMessage((uint16_t)(DISPLAY_ALARM_MODULE_X + 20U), (uint16_t)(row_y + 15U), reason_msg, DISPLAY_THEME_TEXT);
      } else if (reason_cn != 0) {
        (void)Display_TextDrawRawBitmap((uint16_t)(DISPLAY_ALARM_MODULE_X + 20U), (uint16_t)(row_y + 15U), reason_cn->width, 16U, reason_cn->bpr, reason_cn->data, DISPLAY_THEME_TEXT);
      } else {
        (void)Display_GfxDrawString((uint16_t)(DISPLAY_ALARM_MODULE_X + 20U), (uint16_t)(row_y + 16U), Display_PagesGetAlarmReason(fault_code), DISPLAY_THEME_TEXT, 2U);
      }
    }
  }
}

static void Display_PagesDrawWholePercent(uint16_t x, uint16_t y, uint16_t value, uint16_t color)
{
  uint16_t drawn_w;

  drawn_w = Display_PagesDrawU32Scaled(x, y, value, color, 1U);
  (void)Display_GfxDrawChar((uint16_t)(x + drawn_w), y, '%', color, 1U);
}

static void Display_PagesDrawTenthsPercent(uint16_t x, uint16_t y, uint16_t value, uint16_t color)
{
  uint16_t rate = (value > 1000U) ? 1000U : value;
  static char buf[10];
  uint8_t p = 0U;

  if (rate >= 1000U) {
    buf[p++] = '1';
    buf[p++] = '0';
    buf[p++] = '0';
  } else {
    if (rate >= 100U) { buf[p++] = (char)('0' + ((rate / 100U) % 10U)); }
    buf[p++] = (char)('0' + ((rate / 10U) % 10U));
  }
  buf[p++] = '.';
  buf[p++] = (char)('0' + (rate % 10U));
  buf[p++] = '%';
  buf[p]   = '\0';

  (void)Display_GfxDrawString(x, y, buf, color, 1U);
}

/* 将 YYYYMMDD 编码格式化为 "YYYY-MM-DD"。 */
static void Display_PagesFmtDate(char *buf, uint32_t ymd)
{
  uint32_t y = (ymd / 10000U) % 10000U;
  uint32_t m = (ymd / 100U) % 100U;
  uint32_t d = ymd % 100U;

  buf[0]  = (char)('0' + ((y / 1000U) % 10U));
  buf[1]  = (char)('0' + ((y / 100U) % 10U));
  buf[2]  = (char)('0' + ((y / 10U) % 10U));
  buf[3]  = (char)('0' + (y % 10U));
  buf[4]  = '-';
  buf[5]  = (char)('0' + ((m / 10U) % 10U));
  buf[6]  = (char)('0' + (m % 10U));
  buf[7]  = '-';
  buf[8]  = (char)('0' + ((d / 10U) % 10U));
  buf[9]  = (char)('0' + (d % 10U));
  buf[10] = '\0';
}

/*
 * 绘制页眉动态区：左侧日期/时间，右侧丢包率。
 * force 非 0 时强制全部重绘，否则仅重绘发生变化的部分以避免闪烁。
 */
Display_Result_t Display_PagesDrawHeaderDynamic(Display_PagesValueReader_t read_value, uint8_t force)
{
  static uint32_t s_last_date = 0xFFFFFFFFU;
  static uint32_t s_last_time = 0xFFFFFFFFU;
  static uint32_t s_last_loss = 0xFFFFFFFFU;
  uint32_t date_v;
  uint32_t time_v;
  uint32_t loss_v;
  char buf[12];

  if (Display_GfxIsReady() == 0U) { return DISPLAY_NOT_READY; }

  date_v = Display_PagesReadValue(read_value, DISPLAY_HMI_VAR_DATE, 0U);
  time_v = Display_PagesReadValue(read_value, DISPLAY_HMI_VAR_CLOCK_TIME, 0U);
  loss_v = Display_PagesReadValue(read_value, DISPLAY_HMI_VAR_LORA_LOSS_RATE, 0U);

  if ((force != 0U) || (date_v != s_last_date)) {
    Display_PagesFmtDate(buf, date_v);
    (void)Display_GfxFillRect(DISPLAY_HEADER_DT_X, DISPLAY_HEADER_DATE_Y, 64U, 8U, DISPLAY_THEME_PANEL);
    (void)Display_GfxDrawString(DISPLAY_HEADER_DT_X, DISPLAY_HEADER_DATE_Y, buf, DISPLAY_GFX_COLOR_WHITE, 1U);
    s_last_date = date_v;
  }

  if ((force != 0U) || (time_v != s_last_time)) {
    Display_PagesFmtClock(buf, time_v);
    (void)Display_GfxFillRect(DISPLAY_HEADER_DT_X, DISPLAY_HEADER_TIME_Y, 100U, 16U, DISPLAY_THEME_PANEL);
    (void)Display_GfxDrawString(DISPLAY_HEADER_DT_X, DISPLAY_HEADER_TIME_Y, buf, DISPLAY_GFX_COLOR_WHITE, 2U);
    s_last_time = time_v;
  }

  if (force != 0U) { (void)Display_GfxFillRect(DISPLAY_HEADER_POWER_CLEAR_X, DISPLAY_HEADER_POWER_CLEAR_Y, DISPLAY_HEADER_POWER_CLEAR_W, DISPLAY_HEADER_POWER_CLEAR_H, DISPLAY_THEME_PANEL); }

  if ((force != 0U) || (loss_v != s_last_loss)) {
    (void)Display_GfxFillRect(DISPLAY_HEADER_LOSS_X, DISPLAY_HEADER_LOSS_Y, 60U, 8U, DISPLAY_THEME_PANEL);
    Display_PagesDrawHeaderLossRate(DISPLAY_HEADER_LOSS_X, DISPLAY_HEADER_LOSS_Y, loss_v);
    s_last_loss = loss_v;
  }

  return DISPLAY_OK;
}

static uint16_t Display_PagesMotorLimitPercent(uint32_t value)
{
  return (value > DISPLAY_MOTOR_SLIDER_MAX_VALUE) ? DISPLAY_MOTOR_SLIDER_MAX_VALUE : (uint16_t)value;
}

static uint16_t Display_PagesMotorFilledHeight(const Display_HmiVariableConfig_t *variable, uint16_t percent)
{
  if ((variable == 0) || (variable->height == 0U)) { return 0U; }

  return (uint16_t)(((uint32_t)variable->height * percent) / DISPLAY_MOTOR_SLIDER_MAX_VALUE);
}

static uint16_t Display_PagesMotorHandleCy(const Display_HmiVariableConfig_t *variable, uint16_t percent)
{
  uint16_t fill_h;
  uint16_t cy;
  uint16_t lo;
  uint16_t hi;

  if ((variable == 0) || (variable->height == 0U)) { return 0U; }

  fill_h = Display_PagesMotorFilledHeight(variable, percent);
  cy     = (uint16_t)(variable->y + variable->height - fill_h);
  lo     = (uint16_t)(variable->y + DISPLAY_MOTOR_HANDLE_HALF_H);
  hi     = (uint16_t)(variable->y + variable->height - DISPLAY_MOTOR_HANDLE_HALF_H);
  if (cy < lo) { cy = lo; }
  if (cy > hi) { cy = hi; }

  return cy;
}

static uint16_t Display_PagesMinU16(uint16_t a, uint16_t b)
{
  return (a < b) ? a : b;
}

static uint16_t Display_PagesMaxU16(uint16_t a, uint16_t b)
{
  return (a > b) ? a : b;
}

static void Display_PagesDrawMotorTrackBand(const Display_HmiVariableConfig_t *variable, uint16_t y, uint16_t height, uint16_t percent)
{
  uint16_t clear_x;
  uint16_t clear_w;
  uint16_t fill_h;
  uint16_t fill_y;
  uint16_t y_end;
  uint16_t fill_end;

  if ((variable == 0) || (height == 0U)) { return; }

  clear_x = (variable->x > DISPLAY_MOTOR_HANDLE_HALF_W) ? (uint16_t)(variable->x - DISPLAY_MOTOR_HANDLE_HALF_W) : 0U;
  clear_w = (uint16_t)(variable->width + (DISPLAY_MOTOR_HANDLE_HALF_W * 2U));
  y_end    = (uint16_t)(y + height);
  fill_h   = Display_PagesMotorFilledHeight(variable, percent);
  fill_y   = (uint16_t)(variable->y + variable->height - fill_h);
  fill_end = (uint16_t)(variable->y + variable->height);

  (void)Display_GfxFillRect(clear_x, y, clear_w, height, DISPLAY_THEME_PANEL);
  if ((fill_h != 0U) && (y_end > fill_y) && (y < fill_end)) {
    uint16_t blue_y = (y > fill_y) ? y : fill_y;
    uint16_t blue_h = (uint16_t)(((y_end < fill_end) ? y_end : fill_end) - blue_y);
    if (blue_h != 0U) { (void)Display_GfxFillRect(variable->x, blue_y, variable->width, blue_h, DISPLAY_GFX_COLOR_BLUE); }
  }
}

static void Display_PagesDrawMotorPercent(const Display_HmiVariableConfig_t *variable, uint16_t percent)
{
  (void)Display_GfxFillRect((uint16_t)(variable->x - 16U), (uint16_t)(variable->y + variable->height + 14U), 64U, 18U, DISPLAY_THEME_PANEL);
  Display_PagesDrawWholePercent((uint16_t)(variable->x - 10U), (uint16_t)(variable->y + variable->height + 16U), percent, DISPLAY_THEME_TEXT);
}

Display_Result_t Display_PagesDrawMotorSliderField(const Display_HmiVariableConfig_t *variable, uint32_t old_value, uint32_t value, uint8_t full_redraw, uint8_t draw_percent)
{
  uint16_t old_fill_y;
  uint16_t fill_y;
  uint16_t old_percent;
  uint16_t percent;
  uint16_t old_handle_cy;
  uint16_t handle_cy;
  uint16_t cx;

  if ((variable == 0) || (Display_GfxIsReady() == 0U)) { return DISPLAY_ERROR; }

  old_percent   = Display_PagesMotorLimitPercent(old_value);
  percent       = Display_PagesMotorLimitPercent(value);
  old_handle_cy = Display_PagesMotorHandleCy(variable, old_percent);
  handle_cy     = Display_PagesMotorHandleCy(variable, percent);
  old_fill_y     = (uint16_t)(variable->y + variable->height - Display_PagesMotorFilledHeight(variable, old_percent));
  fill_y         = (uint16_t)(variable->y + variable->height - Display_PagesMotorFilledHeight(variable, percent));
  cx            = (uint16_t)(variable->x + (variable->width / 2U));

  if (full_redraw != 0U) {
    Display_PagesDrawMotorTrackBand(variable, variable->y, variable->height, percent);
    (void)Display_GfxDrawRect((uint16_t)(variable->x - 1U), (uint16_t)(variable->y - 1U), (uint16_t)(variable->width + 2U), (uint16_t)(variable->height + 2U), DISPLAY_THEME_PANEL);
  } else if (old_handle_cy != handle_cy) {
    uint16_t dirty_y   = Display_PagesMinU16(old_fill_y, fill_y);
    uint16_t dirty_end = Display_PagesMaxU16(old_fill_y, fill_y);
    uint16_t old_top   = (old_handle_cy > DISPLAY_MOTOR_HANDLE_HALF_H) ? (uint16_t)(old_handle_cy - DISPLAY_MOTOR_HANDLE_HALF_H) : variable->y;
    uint16_t new_top   = (handle_cy > DISPLAY_MOTOR_HANDLE_HALF_H) ? (uint16_t)(handle_cy - DISPLAY_MOTOR_HANDLE_HALF_H) : variable->y;
    uint16_t old_end   = (uint16_t)(old_handle_cy + DISPLAY_MOTOR_HANDLE_HALF_H);
    uint16_t new_end   = (uint16_t)(handle_cy + DISPLAY_MOTOR_HANDLE_HALF_H);

    dirty_y   = Display_PagesMinU16(dirty_y, Display_PagesMinU16(old_top, new_top));
    dirty_end = Display_PagesMaxU16(dirty_end, Display_PagesMaxU16(old_end, new_end));
    if (dirty_y < variable->y) { dirty_y = variable->y; }
    if (dirty_end > (uint16_t)(variable->y + variable->height)) { dirty_end = (uint16_t)(variable->y + variable->height); }
    if (dirty_end > dirty_y) { Display_PagesDrawMotorTrackBand(variable, dirty_y, (uint16_t)(dirty_end - dirty_y), percent); }
    (void)Display_GfxDrawRect((uint16_t)(variable->x - 1U), (uint16_t)(variable->y - 1U), (uint16_t)(variable->width + 2U), (uint16_t)(variable->height + 2U), DISPLAY_THEME_PANEL);
  }

  (void)Display_GfxFillRect((uint16_t)(cx - DISPLAY_MOTOR_HANDLE_HALF_W), (uint16_t)(handle_cy - DISPLAY_MOTOR_HANDLE_HALF_H), (uint16_t)(DISPLAY_MOTOR_HANDLE_HALF_W * 2U), (uint16_t)(DISPLAY_MOTOR_HANDLE_HALF_H * 2U), DISPLAY_GFX_COLOR_CYAN);
  (void)Display_GfxDrawRect((uint16_t)(cx - DISPLAY_MOTOR_HANDLE_HALF_W), (uint16_t)(handle_cy - DISPLAY_MOTOR_HANDLE_HALF_H), (uint16_t)(DISPLAY_MOTOR_HANDLE_HALF_W * 2U), (uint16_t)(DISPLAY_MOTOR_HANDLE_HALF_H * 2U), DISPLAY_THEME_TEXT);
  (void)Display_GfxDrawHLine((uint16_t)(cx - DISPLAY_MOTOR_HANDLE_HALF_W + 3U), handle_cy, (uint16_t)((DISPLAY_MOTOR_HANDLE_HALF_W * 2U) - 6U), DISPLAY_THEME_TEXT);
  if (draw_percent != 0U) { Display_PagesDrawMotorPercent(variable, percent); }
  return DISPLAY_OK;
}

/*
 * 绘制有符号 32 位整数，返回绘制像素宽度。
 */
static uint16_t Display_PagesDrawI32(uint16_t x, uint16_t y, int32_t value, uint16_t color)
{
  uint32_t uval;
  uint16_t w;

  if (value < 0) {
    (void)Display_GfxDrawChar(x, y, '-', color, 2U);
    x    = (uint16_t)(x + 12U);
    uval = (uint32_t)(-value);
    w    = (uint16_t)(12U + Display_PagesDrawU32Scaled(x, y, uval, color, 2U));
  } else {
    uval = (uint32_t)value;
    w    = Display_PagesDrawU32Scaled(x, y, uval, color, 2U);
  }

  return w;
}

#define DISPLAY_PAGES_DEG_STR "\xB0"
#define DISPLAY_PAGES_DEGC_STR                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         \
  "\xB0"                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               \
  "C"
#define DISPLAY_PAGES_UNIT_GAP 2U

static uint16_t Display_PagesStrLen(const char *s)
{
  uint16_t n = 0U;
  while (s[n] != '\0') { n++; }
  return n;
}

static uint16_t Display_PagesDrawUnitAfter(uint16_t x, uint16_t y, uint16_t drawn_w, const char *unit)
{
  uint16_t unit_w;

  if ((unit == 0) || (unit[0] == '\0') || (unit[0] == '-')) { return drawn_w; }

  unit_w = (uint16_t)(Display_PagesStrLen(unit) * 6U);
  (void)Display_GfxDrawString((uint16_t)(x + drawn_w + DISPLAY_PAGES_UNIT_GAP), y, unit, DISPLAY_GFX_COLOR_GRAY, 1U);
  return (uint16_t)(drawn_w + DISPLAY_PAGES_UNIT_GAP + unit_w);
}

/*
 * 按 Framework 生成的 GNSS 显示状态绘制信号/定位状态。
 */
static const uint8_t display_gnss_signal_lost[] = {
  0x08U, 0x40U, 0x00U, 0x00U, 0x04U, 0x00U, 0x00U, 0x00U,
  0x08U, 0x20U, 0x1FU, 0xF0U, 0x04U, 0x04U, 0x7FU, 0xFCU,
  0x0BU, 0xFEU, 0x10U, 0x10U, 0x55U, 0x78U, 0x08U, 0x20U,
  0x10U, 0x00U, 0x10U, 0x10U, 0x4EU, 0x40U, 0x08U, 0x20U,
  0x10U, 0x00U, 0x10U, 0x10U, 0x44U, 0x40U, 0x08U, 0x20U,
  0x31U, 0xFCU, 0x1FU, 0xF0U, 0x7FU, 0x40U, 0x08U, 0x20U,
  0x30U, 0x00U, 0x00U, 0x00U, 0x44U, 0x7EU, 0x08U, 0x20U,
  0x50U, 0x00U, 0xFFU, 0xFEU, 0x4EU, 0x48U, 0xFFU, 0xFEU,
  0x91U, 0xFCU, 0x08U, 0x00U, 0x55U, 0x48U, 0x08U, 0x20U,
  0x10U, 0x00U, 0x10U, 0x00U, 0x65U, 0x48U, 0x08U, 0x20U,
  0x10U, 0x00U, 0x1FU, 0xF0U, 0x44U, 0x48U, 0x08U, 0x20U,
  0x11U, 0xFCU, 0x00U, 0x10U, 0x44U, 0x48U, 0x08U, 0x20U,
  0x11U, 0x04U, 0x00U, 0x10U, 0x40U, 0x48U, 0x10U, 0x20U,
  0x11U, 0x04U, 0x00U, 0x10U, 0x7FU, 0x88U, 0x10U, 0x20U,
  0x11U, 0xFCU, 0x00U, 0xA0U, 0x00U, 0x88U, 0x20U, 0x20U,
  0x11U, 0x04U, 0x00U, 0x40U, 0x01U, 0x08U, 0x40U, 0x20U,
};

static const uint8_t display_gnss_signal_ok[] = {
  0x08U, 0x40U, 0x00U, 0x00U, 0x00U, 0x00U, 0x01U, 0x00U,
  0x08U, 0x20U, 0x1FU, 0xF0U, 0x7FU, 0xFCU, 0x11U, 0x10U,
  0x0BU, 0xFEU, 0x10U, 0x10U, 0x01U, 0x00U, 0x09U, 0x20U,
  0x10U, 0x00U, 0x10U, 0x10U, 0x01U, 0x00U, 0x7FU, 0xFEU,
  0x10U, 0x00U, 0x10U, 0x10U, 0x01U, 0x00U, 0x40U, 0x02U,
  0x31U, 0xFCU, 0x1FU, 0xF0U, 0x01U, 0x00U, 0x9FU, 0xF4U,
  0x30U, 0x00U, 0x00U, 0x00U, 0x11U, 0x00U, 0x10U, 0x10U,
  0x50U, 0x00U, 0xFFU, 0xFEU, 0x11U, 0xF8U, 0x1FU, 0xF0U,
  0x91U, 0xFCU, 0x08U, 0x00U, 0x11U, 0x00U, 0x01U, 0x00U,
  0x10U, 0x00U, 0x10U, 0x00U, 0x11U, 0x00U, 0x3FU, 0xF8U,
  0x10U, 0x00U, 0x1FU, 0xF0U, 0x11U, 0x00U, 0x21U, 0x08U,
  0x11U, 0xFCU, 0x00U, 0x10U, 0x11U, 0x00U, 0x21U, 0x08U,
  0x11U, 0x04U, 0x00U, 0x10U, 0x11U, 0x00U, 0x21U, 0x28U,
  0x11U, 0x04U, 0x00U, 0x10U, 0x11U, 0x00U, 0x21U, 0x10U,
  0x11U, 0xFCU, 0x00U, 0xA0U, 0xFFU, 0xFEU, 0x01U, 0x00U,
  0x11U, 0x04U, 0x00U, 0x40U, 0x00U, 0x00U, 0x01U, 0x00U,
};

static void Display_PagesDrawGnssFixText(const Display_HmiVariableConfig_t *variable, uint32_t value)
{
  const uint8_t *text_data;
  uint16_t color;

  if (value == 0U) {
    text_data = display_gnss_signal_lost;
    color     = DISPLAY_GFX_COLOR_RED;
  } else {
    text_data = display_gnss_signal_ok;
    color     = DISPLAY_GFX_COLOR_GREEN;
  }

  (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
  (void)Display_TextDrawRawBitmap((uint16_t)(variable->x + 2U), (uint16_t)(variable->y + 2U), 64U, 16U, 8U, text_data, color);
}

static void Display_PagesFmtUnsigned(char *buf, uint32_t whole, uint32_t frac, uint8_t decimals, uint32_t divisor)
{
  uint8_t pos = 0U;
  char tmp[11];
  uint8_t tlen = 0U;
  uint32_t rem = whole;
  uint8_t i;

  if (rem == 0U) {
    buf[pos++] = '0';
  } else {
    while (rem > 0U) {
      tmp[tlen++] = (char)('0' + (rem % 10U));
      rem /= 10U;
    }
    while (tlen > 0U) { buf[pos++] = tmp[--tlen]; }
  }

  if (decimals > 0U) {
    buf[pos++] = '.';
    for (i = 0U; i < decimals; i++) {
      frac *= 10U;
      buf[pos++] = (char)('0' + (frac / divisor));
      frac %= divisor;
    }
  }

  buf[pos] = '\0';
}

/*
 * 将时/分/秒格式化为 "HH:MM:SS"，小时超过两位时自动补一位。
 */
static void Display_PagesFmtHms(char *buf, uint32_t h, uint32_t m, uint32_t s)
{
  uint8_t p = 0U;

  if (h > 99U) { buf[p++] = (char)('0' + ((h / 100U) % 10U)); }
  buf[p++] = (char)('0' + ((h / 10U) % 10U));
  buf[p++] = (char)('0' + (h % 10U));
  buf[p++] = ':';
  buf[p++] = (char)('0' + ((m / 10U) % 10U));
  buf[p++] = (char)('0' + (m % 10U));
  buf[p++] = ':';
  buf[p++] = (char)('0' + ((s / 10U) % 10U));
  buf[p++] = (char)('0' + (s % 10U));
  buf[p]   = '\0';
}

/*
 * 按变量配置绘制单个动态字段。
 */
Display_Result_t Display_PagesDrawField(const Display_HmiVariableConfig_t *variable, uint32_t value)
{
  uint16_t dot_x;
  uint16_t dot_y;
  uint16_t limited_value;
  uint16_t vx;
  uint16_t vy;
  uint16_t drawn_w;
  uint16_t total_w;
  uint32_t whole;
  uint32_t frac;
  int32_t sval;
  int16_t s16;
  static char fmt_buf[16];

  if ((variable == 0) || (Display_GfxIsReady() == 0U)) { return DISPLAY_ERROR; }

  if ((variable->width == 0U) || (variable->height == 0U)) { return DISPLAY_OK; }

  if (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_ERROR_CODE) {
    Display_PagesDrawSelfCheckErrorField(value);
    return DISPLAY_OK;
  }

  if (variable->id == DISPLAY_HMI_VAR_MESSAGE_LOG) {
    if (variable->page == DISPLAY_HMI_PAGE_MOTOR) {
      Display_PagesDrawMotorMessageLogBody();
    } else {
      Display_PagesDrawMessageLogBody();
    }
    return DISPLAY_OK;
  }

  if (variable->id == DISPLAY_HMI_VAR_ALARM_ACTIVE_MASK) {
    Display_PagesDrawAlarmSummaryField(variable, value);
    return DISPLAY_OK;
  }

  if (Display_PagesIsAlarmRow(variable->id) != 0U) {
    Display_PagesDrawAlarmRow(variable, value);
    return DISPLAY_OK;
  }

  if ((variable->id == DISPLAY_HMI_VAR_BATTERY_PERCENT) || (variable->id == DISPLAY_HMI_VAR_MOTOR_BAT_PERCENT)) {
    limited_value = (value > 100U) ? 100U : (uint16_t)value;
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    drawn_w = Display_PagesDrawU32Scaled((uint16_t)(variable->x + 2U), (uint16_t)(variable->y + 2U), limited_value, DISPLAY_THEME_TEXT, 2U);
    (void)Display_GfxDrawChar((uint16_t)(variable->x + 2U + drawn_w), (uint16_t)(variable->y + 2U), '%', DISPLAY_THEME_TEXT, 2U);
    (void)Display_GfxDrawRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    return DISPLAY_OK;
  }

  if (((variable->id == DISPLAY_HMI_VAR_MOTOR_PWM_1) || (variable->id == DISPLAY_HMI_VAR_MOTOR_PWM_2) || (variable->id == DISPLAY_HMI_VAR_MOTOR_PWM_3) || (variable->id == DISPLAY_HMI_VAR_MOTOR_PWM_4)) && (variable->page == DISPLAY_HMI_PAGE_MOTOR)) { return Display_PagesDrawMotorSliderField(variable, value, value, 1U, 1U); }

  if ((variable->id == DISPLAY_HMI_VAR_MOTOR_PWM_1) || (variable->id == DISPLAY_HMI_VAR_MOTOR_PWM_2) || (variable->id == DISPLAY_HMI_VAR_MOTOR_PWM_3) || (variable->id == DISPLAY_HMI_VAR_MOTOR_PWM_4)) {
    limited_value = (value > 100U) ? 100U : (uint16_t)value;
    (void)Display_GfxDrawProgressBar(variable->x, variable->y, variable->width, variable->height, limited_value, 100U, DISPLAY_GFX_COLOR_BLUE, DISPLAY_GFX_COLOR_GRAY, DISPLAY_GFX_COLOR_BLACK);
    (void)Display_GfxFillRect((uint16_t)(variable->x + variable->width + 6U), variable->y, 34U, variable->height, DISPLAY_THEME_PANEL);
    Display_PagesDrawWholePercent((uint16_t)(variable->x + variable->width + 8U), (uint16_t)(variable->y + 5U), limited_value, DISPLAY_THEME_TEXT);
    return DISPLAY_OK;
  }

  if (variable->id == DISPLAY_HMI_VAR_LORA_LOSS_RATE) {
    limited_value = (value > 1000U) ? 1000U : (uint16_t)value;
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    Display_PagesDrawTenthsPercent((uint16_t)(variable->x + 2U), (uint16_t)(variable->y + 5U), limited_value, DISPLAY_THEME_TEXT);
    return DISPLAY_OK;
  }

  if ((variable->id == DISPLAY_HMI_VAR_GNSS_FIX) && (variable->page == DISPLAY_HMI_PAGE_DATA)) {
    Display_PagesDrawGnssFixText(variable, value);
    return DISPLAY_OK;
  }

  if (variable->id == DISPLAY_HMI_VAR_GNSS_HDOP) {
    whole = value / 100U;
    frac  = value % 100U;
    Display_PagesFmtUnsigned(fmt_buf, whole, frac, 2U, 100U);
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    (void)Display_GfxDrawString((uint16_t)(variable->x + 2U), (uint16_t)(variable->y + 2U), fmt_buf, DISPLAY_THEME_TEXT, 2U);
    (void)Display_GfxDrawRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    return DISPLAY_OK;
  }

  if ((variable->id == DISPLAY_HMI_VAR_SELF_CHECK_5GA) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_MPU6050) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_BME280) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_SD) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_MOTOR) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_2) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_3) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_4) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_POWER) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_BUZZER) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_KEY) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_DEBUG) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_GNSS) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_LORA) || (variable->id == DISPLAY_HMI_VAR_SYSTEM_STATUS) || (variable->id == DISPLAY_HMI_VAR_GNSS_FIX) || (variable->id == DISPLAY_HMI_VAR_LORA_STATUS) || (variable->id == DISPLAY_HMI_VAR_LORA_HEARTBEAT) ||
      (variable->id == DISPLAY_HMI_VAR_ALARM_CODE)) {
    dot_x = (uint16_t)(variable->x + 12U);
    dot_y = (uint16_t)(variable->y + (variable->height / 2U));
    (void)Display_GfxFillRect((uint16_t)(dot_x - 10U), (uint16_t)(dot_y - 10U), 20U, 20U, DISPLAY_THEME_PANEL);
    (void)Display_GfxDrawStatusDot(dot_x, dot_y, 8U, Display_PagesGetStatusColor(value), DISPLAY_GFX_COLOR_BLACK);
    return DISPLAY_OK;
  }

  vx = (uint16_t)(variable->x + 2U);
  vy = (uint16_t)(variable->y + 2U);

  /* 飞行时间(上电后运行)：秒 -> HH:MM:SS */
  if (variable->id == DISPLAY_HMI_VAR_FLIGHT_TIME_S) {
    Display_PagesFmtHms(fmt_buf, value / 3600U, (value / 60U) % 60U, value % 60U);
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    (void)Display_GfxDrawString(vx, vy, fmt_buf, DISPLAY_THEME_TEXT, 2U);
    (void)Display_GfxDrawRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    return DISPLAY_OK;
  }

  /* 本地显示时间：HHMMSS -> HH:MM:SS */
  if (variable->id == DISPLAY_HMI_VAR_CLOCK_TIME) {
    Display_PagesFmtHms(fmt_buf, (value / 10000U) % 100U, (value / 100U) % 100U, value % 100U);
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    (void)Display_GfxDrawString(vx, vy, fmt_buf, DISPLAY_THEME_TEXT, 2U);
    (void)Display_GfxDrawRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    return DISPLAY_OK;
  }

  /* 本地显示日期：YYYYMMDD -> YYYY-MM-DD（0 表示无有效日期）*/
  if (variable->id == DISPLAY_HMI_VAR_DATE) {
    uint32_t yyyy = value / 10000U;
    uint32_t mm   = (value / 100U) % 100U;
    uint32_t dd   = value % 100U;
    uint8_t p     = 0U;

    if (value == 0U) {
      for (p = 0U; p < 10U; p++) { fmt_buf[p] = '-'; }
    } else {
      fmt_buf[p++] = (char)('0' + ((yyyy / 1000U) % 10U));
      fmt_buf[p++] = (char)('0' + ((yyyy / 100U) % 10U));
      fmt_buf[p++] = (char)('0' + ((yyyy / 10U) % 10U));
      fmt_buf[p++] = (char)('0' + (yyyy % 10U));
      fmt_buf[p++] = '-';
      fmt_buf[p++] = (char)('0' + ((mm / 10U) % 10U));
      fmt_buf[p++] = (char)('0' + (mm % 10U));
      fmt_buf[p++] = '-';
      fmt_buf[p++] = (char)('0' + ((dd / 10U) % 10U));
      fmt_buf[p++] = (char)('0' + (dd % 10U));
    }
    fmt_buf[p] = '\0';
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    (void)Display_GfxDrawString(vx, vy, fmt_buf, DISPLAY_THEME_TEXT, 2U);
    (void)Display_GfxDrawRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    return DISPLAY_OK;
  }

  /* Variables with decimal formatting and unit strings */
  if ((variable->id == DISPLAY_HMI_VAR_BATTERY_VOLTAGE) || (variable->id == DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE)) {
    whole = value / 100U;
    frac  = value % 100U;
    Display_PagesFmtUnsigned(fmt_buf, whole, frac, 1U, 100U); /* 电压只显示一位小数 */
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    (void)Display_GfxDrawString(vx, vy, fmt_buf, DISPLAY_THEME_TEXT, 2U);
    drawn_w = (uint16_t)(Display_PagesStrLen(fmt_buf) * 12U);
    Display_PagesDrawUnitAfter(vx, vy, drawn_w, "V");
    (void)Display_GfxDrawRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    return DISPLAY_OK;
  }

  if (variable->id == DISPLAY_HMI_VAR_LATITUDE || variable->id == DISPLAY_HMI_VAR_LONGITUDE) {
    sval = (int32_t)value;
    /* 3 decimal places from 1e-7 degrees */
    {
      uint8_t pos = 0U;
      char tmp[11];
      uint8_t tlen  = 0U;
      uint32_t uval = (sval < 0) ? (uint32_t)(-sval) : (uint32_t)sval;
      uint32_t w    = uval / 10000000U;
      uint32_t f    = uval % 10000000U;
      uint8_t di;

      if (sval < 0) { fmt_buf[pos++] = '-'; }
      if (w == 0U) {
        fmt_buf[pos++] = '0';
      } else {
        uint32_t r = w;
        while (r > 0U) {
          tmp[tlen++] = (char)('0' + (r % 10U));
          r /= 10U;
        }
        while (tlen > 0U) { fmt_buf[pos++] = tmp[--tlen]; }
      }
      fmt_buf[pos++] = '.';
      for (di = 0U; di < 3U; di++) {
        f *= 10U;
        fmt_buf[pos++] = (char)('0' + (f / 10000000U));
        f %= 10000000U;
      }
      fmt_buf[pos] = '\0';
    }
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    (void)Display_GfxDrawString(vx, vy, fmt_buf, DISPLAY_THEME_TEXT, 2U);
    drawn_w = (uint16_t)(Display_PagesStrLen(fmt_buf) * 12U);
    Display_PagesDrawUnitAfter(vx, vy, drawn_w, DISPLAY_PAGES_DEG_STR);
    (void)Display_GfxDrawRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    return DISPLAY_OK;
  }

  if (variable->id == DISPLAY_HMI_VAR_ALTITUDE) {
    sval = (int32_t)value;
    /* Integer meters, no decimals */
    {
      uint8_t pos = 0U;
      char tmp[11];
      uint8_t tlen   = 0U;
      int32_t meters = sval / 1000;
      uint32_t uval;

      if (meters < 0) {
        fmt_buf[pos++] = '-';
        uval           = (uint32_t)(-meters);
      } else {
        uval = (uint32_t)meters;
      }

      if (uval == 0U) {
        fmt_buf[pos++] = '0';
      } else {
        while (uval > 0U) {
          tmp[tlen++] = (char)('0' + (uval % 10U));
          uval /= 10U;
        }
        while (tlen > 0U) { fmt_buf[pos++] = tmp[--tlen]; }
      }
      fmt_buf[pos] = '\0';
    }
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    (void)Display_GfxDrawString(vx, vy, fmt_buf, DISPLAY_THEME_TEXT, 2U);
    drawn_w = (uint16_t)(Display_PagesStrLen(fmt_buf) * 12U);
    Display_PagesDrawUnitAfter(vx, vy, drawn_w, "m");
    (void)Display_GfxDrawRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    return DISPLAY_OK;
  }

  if (variable->id == DISPLAY_HMI_VAR_ROLL || variable->id == DISPLAY_HMI_VAR_PITCH || variable->id == DISPLAY_HMI_VAR_YAW) {
    s16  = (int16_t)(value & 0xFFFFU);
    sval = s16;
    {
      uint8_t pos = 0U;
      char tmp[8];
      uint8_t tlen = 0U;
      uint32_t abs_val;
      uint32_t abs_frac;

      if (sval < 0) {
        fmt_buf[pos++] = '-';
        abs_val        = (uint32_t)(-sval);
      } else {
        abs_val = (uint32_t)sval;
      }

      abs_frac = abs_val % 10U;
      abs_val /= 10U;

      if (abs_val == 0U) {
        fmt_buf[pos++] = '0';
      } else {
        uint32_t rem = abs_val;
        while (rem > 0U) {
          tmp[tlen++] = (char)('0' + (rem % 10U));
          rem /= 10U;
        }
        while (tlen > 0U) { fmt_buf[pos++] = tmp[--tlen]; }
      }
      fmt_buf[pos++] = '.';
      fmt_buf[pos++] = (char)('0' + (abs_frac % 10U));
      fmt_buf[pos]   = '\0';
    }
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    (void)Display_GfxDrawString(vx, vy, fmt_buf, DISPLAY_THEME_TEXT, 2U);
    drawn_w = (uint16_t)(Display_PagesStrLen(fmt_buf) * 12U);
    Display_PagesDrawUnitAfter(vx, vy, drawn_w, DISPLAY_PAGES_DEG_STR);
    (void)Display_GfxDrawRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    return DISPLAY_OK;
  }

  if (variable->id == DISPLAY_HMI_VAR_TEMPERATURE) {
    s16  = (int16_t)(value & 0xFFFFU);
    sval = s16;
    {
      uint8_t pos = 0U;
      char tmp[8];
      uint8_t tlen = 0U;
      uint32_t abs_val;
      uint32_t abs_frac;

      if (sval < 0) {
        fmt_buf[pos++] = '-';
        abs_val        = (uint32_t)(-sval);
      } else {
        abs_val = (uint32_t)sval;
      }

      abs_frac = abs_val % 10U;
      abs_val /= 10U;

      if (abs_val == 0U) {
        fmt_buf[pos++] = '0';
      } else {
        uint32_t rem = abs_val;
        while (rem > 0U) {
          tmp[tlen++] = (char)('0' + (rem % 10U));
          rem /= 10U;
        }
        while (tlen > 0U) { fmt_buf[pos++] = tmp[--tlen]; }
      }
      fmt_buf[pos++] = '.';
      fmt_buf[pos++] = (char)('0' + (abs_frac % 10U));
      fmt_buf[pos]   = '\0';
    }
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    (void)Display_GfxDrawString(vx, vy, fmt_buf, DISPLAY_THEME_TEXT, 2U);
    drawn_w = (uint16_t)(Display_PagesStrLen(fmt_buf) * 12U);
    Display_PagesDrawUnitAfter(vx, vy, drawn_w, DISPLAY_PAGES_DEGC_STR);
    (void)Display_GfxDrawRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    return DISPLAY_OK;
  }

  if (variable->id == DISPLAY_HMI_VAR_HUMIDITY) {
    whole = value / 10U;
    frac  = value % 10U;
    Display_PagesFmtUnsigned(fmt_buf, whole, frac, 1U, 10U);
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    (void)Display_GfxDrawString(vx, vy, fmt_buf, DISPLAY_THEME_TEXT, 2U);
    drawn_w = (uint16_t)(Display_PagesStrLen(fmt_buf) * 12U);
    Display_PagesDrawUnitAfter(vx, vy, drawn_w, "%");
    (void)Display_GfxDrawRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    return DISPLAY_OK;
  }

  /* Generic signed integer with optional unit */
  if ((variable->data_type == DISPLAY_HMI_TYPE_I32) || (variable->data_type == DISPLAY_HMI_TYPE_I16)) {
    sval = (variable->data_type == DISPLAY_HMI_TYPE_I32) ? (int32_t)value : (int16_t)(value & 0xFFFFU);
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    drawn_w = Display_PagesDrawI32(vx, vy, sval, DISPLAY_THEME_TEXT);
    total_w = drawn_w;
    if (variable->unit[0] != '-' && variable->unit[0] != '\0') { total_w = Display_PagesDrawUnitAfter(vx, vy, drawn_w, variable->unit); }
    if ((total_w + 4U) < variable->width) { (void)Display_GfxFillRect((uint16_t)(variable->x + 2U + total_w), (uint16_t)(variable->y + 2U), (uint16_t)(variable->width - 4U - total_w), (uint16_t)(variable->height - 4U), DISPLAY_THEME_PANEL); }
  } else if ((variable->data_type == DISPLAY_HMI_TYPE_U32) || (variable->data_type == DISPLAY_HMI_TYPE_U16)) {
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
    drawn_w = Display_PagesDrawU32(vx, vy, value, DISPLAY_THEME_TEXT);
    total_w = drawn_w;
    if (variable->unit[0] != '-' && variable->unit[0] != '\0') { total_w = Display_PagesDrawUnitAfter(vx, vy, drawn_w, variable->unit); }
    if ((total_w + 4U) < variable->width) { (void)Display_GfxFillRect((uint16_t)(variable->x + 2U + total_w), (uint16_t)(variable->y + 2U), (uint16_t)(variable->width - 4U - total_w), (uint16_t)(variable->height - 4U), DISPLAY_THEME_PANEL); }
  }

  (void)Display_GfxDrawRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_THEME_PANEL);
  return DISPLAY_OK;
}
