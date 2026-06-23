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
#define DISPLAY_ALARM_COLOR_HEADER     DISPLAY_GFX_COLOR_DARK
#define DISPLAY_ALARM_COLOR_GRID       DISPLAY_GFX_COLOR_GRAY
#define DISPLAY_ALARM_COLOR_ROW_ALT    0xF7BEU
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

static void Display_PagesDrawPanelLabel(uint16_t x, uint16_t y, uint16_t width, uint16_t height, Display_TextLabel_t title_label)
{
  (void)Display_GfxDrawFrame(x, y, width, height, DISPLAY_GFX_COLOR_GRAY, DISPLAY_GFX_COLOR_WHITE);
  (void)Display_TextDrawLabel((uint16_t)(x + 10U), (uint16_t)(y + 4U), title_label, DISPLAY_GFX_COLOR_BLUE);
  (void)Display_GfxDrawHLine(x, (uint16_t)(y + 28U), width, DISPLAY_GFX_COLOR_GRAY);
}

static void Display_PagesDrawMotorLabel(uint16_t x, uint16_t y, char index)
{
  uint16_t motor_w;

  (void)Display_GfxDrawString(x, y, "-", DISPLAY_GFX_COLOR_DARK, 1U);
  (void)Display_TextDrawLabel((uint16_t)(x + 12U), y, DISPLAY_TXT_MOTOR, DISPLAY_GFX_COLOR_DARK);
  motor_w = Display_TextGetLabelWidth(DISPLAY_TXT_MOTOR);
  (void)Display_GfxDrawChar((uint16_t)(x + 16U + motor_w), y, index, DISPLAY_GFX_COLOR_DARK, 1U);
}

static void Display_PagesDrawSelfMotorLabel(uint16_t x, uint16_t y, uint16_t width, char index)
{
  uint16_t motor_w = Display_TextGetLabelWidth(DISPLAY_TXT_MOTOR);
  uint16_t text_w  = (uint16_t)(motor_w + 8U);
  uint16_t text_x  = (text_w < width) ? (uint16_t)(x + ((width - text_w) / 2U)) : x;

  (void)Display_TextDrawLabel(text_x, y, DISPLAY_TXT_MOTOR, DISPLAY_GFX_COLOR_DARK);
  (void)Display_GfxDrawChar((uint16_t)(text_x + motor_w + 2U), y, index, DISPLAY_GFX_COLOR_DARK, 1U);
}

/*
 * 根据页面 ID 获取标题文字标签。 */
/*
 * 绘制页面底部翻页按钮和页码状态点。
 */
#define DISPLAY_PAGES_DOT_SPACING 28U

static void Display_PagesDrawFooter(Display_HmiPage_t page)
{
  uint16_t i;
  uint16_t dot_x;
  uint16_t dot_count;
  uint16_t start_x;

  (void)Display_GfxFillRect(0U, DISPLAY_PAGES_FOOTER_Y, DISPLAY_GFX_WIDTH, DISPLAY_PAGES_FOOTER_HEIGHT, DISPLAY_GFX_COLOR_WHITE);
  (void)Display_GfxDrawHLine(0U, DISPLAY_PAGES_FOOTER_Y, DISPLAY_GFX_WIDTH, DISPLAY_GFX_COLOR_GRAY);
  (void)Display_GfxDrawFrame(8U, 426U, 250U, 48U, DISPLAY_GFX_COLOR_GRAY, DISPLAY_GFX_COLOR_WHITE);
  (void)Display_TextDrawLabel(96U, 442U, DISPLAY_TEXT_PREV, DISPLAY_GFX_COLOR_DARK);
  (void)Display_GfxDrawFrame(542U, 426U, 250U, 48U, DISPLAY_GFX_COLOR_GRAY, DISPLAY_GFX_COLOR_WHITE);
  (void)Display_TextDrawLabel(630U, 442U, DISPLAY_TEXT_NEXT, DISPLAY_GFX_COLOR_DARK);

  /* 页码圆点以屏幕中线为中心，随页面数量自动居中 */
  dot_count = (uint16_t)(DISPLAY_HMI_PAGE_COUNT - DISPLAY_HMI_PAGE_SELF_CHECK);
  start_x   = (uint16_t)((DISPLAY_GFX_WIDTH / 2U) - (((dot_count - 1U) * DISPLAY_PAGES_DOT_SPACING) / 2U));

  for (i = (uint16_t)DISPLAY_HMI_PAGE_SELF_CHECK; i < DISPLAY_HMI_PAGE_COUNT; i++) {
    dot_x = (uint16_t)(start_x + ((i - (uint16_t)DISPLAY_HMI_PAGE_SELF_CHECK) * DISPLAY_PAGES_DOT_SPACING));
    (void)Display_GfxDrawStatusDot(dot_x, 450U, 6U, (i == (uint16_t)page) ? DISPLAY_GFX_COLOR_BLUE : DISPLAY_GFX_COLOR_GRAY, DISPLAY_GFX_COLOR_BLACK);
  }
}

static void Display_PagesDrawDataStatusRow(uint16_t x, uint16_t y, Display_TextLabel_t label, char motor_index)
{
  (void)Display_GfxDrawStatusDot(x, (uint16_t)(y + 8U), 6U, DISPLAY_GFX_COLOR_GRAY, DISPLAY_GFX_COLOR_BLACK);

  if (motor_index != 0) {
    Display_PagesDrawMotorLabel((uint16_t)(x + 20U), y, motor_index);
  } else {
    (void)Display_TextDrawLabel((uint16_t)(x + 20U), y, label, DISPLAY_GFX_COLOR_DARK);
  }
}

static void Display_PagesDrawDataGpsRow(uint16_t label_x, uint16_t y, Display_TextLabel_t label)
{
  (void)Display_TextDrawLabel(label_x, y, label, DISPLAY_GFX_COLOR_DARK);
  (void)Display_GfxDrawHLine((uint16_t)(label_x - 8U), (uint16_t)(y + 24U), 236U, DISPLAY_GFX_COLOR_GRAY);
}

static void Display_PagesDrawDataGpsAsciiRow(uint16_t label_x, uint16_t y, const char *label, const char *placeholder)
{
  (void)Display_GfxDrawString(label_x, y, label, DISPLAY_GFX_COLOR_DARK, 1U);
  if (placeholder != 0) { (void)Display_GfxDrawString(390U, (uint16_t)(y - 2U), placeholder, DISPLAY_GFX_COLOR_GRAY, 2U); }
  (void)Display_GfxDrawHLine((uint16_t)(label_x - 8U), (uint16_t)(y + 24U), 236U, DISPLAY_GFX_COLOR_GRAY);
}

static void Display_PagesDrawDataGpsLabelRow(uint16_t label_x, uint16_t y, Display_TextLabel_t label, const char *placeholder)
{
  (void)Display_TextDrawLabel(label_x, y, label, DISPLAY_GFX_COLOR_DARK);
  if (placeholder != 0) { (void)Display_GfxDrawString(390U, (uint16_t)(y - 2U), placeholder, DISPLAY_GFX_COLOR_GRAY, 2U); }
  (void)Display_GfxDrawHLine((uint16_t)(label_x - 8U), (uint16_t)(y + 24U), 236U, DISPLAY_GFX_COLOR_GRAY);
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

  (void)Display_GfxFillRect(alarm_x, alarm_y, alarm_w, alarm_h, DISPLAY_GFX_COLOR_WHITE);

  if (s_msglog_alarm_valid == 0U) { return; }

  color = (s_msglog_alarm.msg == DISPLAY_LOGMSG_ALARM_ACTIVE) ? DISPLAY_GFX_COLOR_RED : DISPLAY_GFX_COLOR_DARK;
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

  (void)Display_GfxFillRect((uint16_t)(log_x + 1U), DISPLAY_MSGLOG_BODY_Y, (uint16_t)(log_w - 2U), DISPLAY_MSGLOG_BODY_H, DISPLAY_GFX_COLOR_WHITE);

  for (i = 0U; i < s_msglog_count; i++) {
    uint16_t idx   = (uint16_t)((s_msglog_head + DISPLAY_MSGLOG_CAP - s_msglog_count + i) % DISPLAY_MSGLOG_CAP);
    uint16_t row_y = (uint16_t)(DISPLAY_MSGLOG_ROW_Y0 + (i * DISPLAY_MSGLOG_ROW_H));

    Display_PagesFmtClock(tbuf, s_msglog[idx].time_hhmmss);
    (void)Display_GfxDrawString(time_x, (uint16_t)(row_y + 4U), tbuf, DISPLAY_GFX_COLOR_GRAY, 1U);
    (void)Display_TextDrawLogMessage(text_x, row_y, s_msglog[idx].msg, DISPLAY_GFX_COLOR_DARK);
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
  (void)Display_GfxDrawFrame(log_x, DISPLAY_DASH_Y, log_w, DISPLAY_DASH_H, DISPLAY_GFX_COLOR_GRAY, DISPLAY_GFX_COLOR_WHITE);
  (void)Display_TextDrawLabel((uint16_t)(log_x + 10U), (uint16_t)(DISPLAY_DASH_Y + 4U), DISPLAY_TEXT_MESSAGE_LOG, DISPLAY_GFX_COLOR_BLUE);
  (void)Display_GfxDrawHLine(log_x, (uint16_t)(DISPLAY_DASH_Y + 28U), log_w, DISPLAY_GFX_COLOR_GRAY);
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
  Display_PagesDrawDataGpsAsciiRow(290U, 332U, "HDOP", 0);

  Display_PagesDrawMessageLog();
}

/*
 * 飞行数据页中间栏：时间/飞行时间/电压/电量/温度/湿度/气压。
 * 标签沿用系统内置中文位图，数值由各变量字段刷新绘制。
 */
static void Display_PagesDrawFlightMiddle(void)
{
  Display_PagesDrawPanelLabel(DISPLAY_DASH_GPS_X, DISPLAY_DASH_Y, DISPLAY_DASH_GPS_W, DISPLAY_DASH_H, DISPLAY_TITLE_FLIGHT);
  Display_PagesDrawDataGpsRow(290U, 118U, DISPLAY_TXT_DATE);    /* 本地显示日期 */
  Display_PagesDrawDataGpsRow(290U, 154U, DISPLAY_TXT_UTC);     /* 本地显示时间 */
  Display_PagesDrawDataGpsRow(290U, 190U, DISPLAY_TXT_UPTIME);  /* 运行(飞行时间) */
  Display_PagesDrawDataGpsRow(290U, 226U, DISPLAY_TXT_BATTERY); /* 电压 */
  Display_PagesDrawDataGpsRow(290U, 262U, DISPLAY_TXT_BATPCT);  /* 电量 */
  Display_PagesDrawDataGpsRow(290U, 298U, DISPLAY_TXT_TEMP);    /* 温度 */
  Display_PagesDrawDataGpsRow(290U, 334U, DISPLAY_TXT_HUM);     /* 湿度 */
  Display_PagesDrawDataGpsRow(290U, 370U, DISPLAY_TXT_PRESS);   /* 气压 */
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
  (void)Display_TextDrawLabel(280U, y, label, DISPLAY_GFX_COLOR_DARK);
  (void)Display_GfxDrawHLine(272U, (uint16_t)(y + 26U), 244U, DISPLAY_GFX_COLOR_GRAY);
}

/*
 * 飞机情况页中间栏：横滚、俯仰、偏航、LoRa 发送/接收。
 */
static void Display_PagesDrawAircraftMiddle(void)
{
  Display_PagesDrawPanelLabel(DISPLAY_DASH_GPS_X, DISPLAY_DASH_Y, DISPLAY_DASH_GPS_W, DISPLAY_DASH_H, DISPLAY_TITLE_AIRCRAFT);
  Display_PagesDrawAircraftLabelRow(140U, DISPLAY_TXT_ROLL);  /* 横滚 */
  Display_PagesDrawAircraftLabelRow(190U, DISPLAY_TXT_PITCH); /* 俯仰 */
  Display_PagesDrawAircraftLabelRow(240U, DISPLAY_TXT_YAW);   /* 偏航 */
  Display_PagesDrawAircraftLabelRow(290U, DISPLAY_TXT_TX);    /* 发送计数 */
  Display_PagesDrawAircraftLabelRow(340U, DISPLAY_TXT_RX);    /* 接收计数 */
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

  (void)Display_TextDrawLabel(label_x, DISPLAY_MOTOR_LABEL_Y, DISPLAY_TXT_MOTOR, DISPLAY_GFX_COLOR_DARK);
  (void)Display_GfxDrawChar((uint16_t)(label_x + motor_w + 2U), DISPLAY_MOTOR_LABEL_Y, index, DISPLAY_GFX_COLOR_DARK, 2U);
  (void)Display_GfxFillRect(track_x, DISPLAY_MOTOR_TRACK_TOP_Y, DISPLAY_MOTOR_TRACK_W, DISPLAY_MOTOR_TRACK_H, DISPLAY_GFX_COLOR_WHITE);
  (void)Display_GfxDrawRect((uint16_t)(track_x - 1U), (uint16_t)(DISPLAY_MOTOR_TRACK_TOP_Y - 1U), (uint16_t)(DISPLAY_MOTOR_TRACK_W + 2U), (uint16_t)(DISPLAY_MOTOR_TRACK_H + 2U), DISPLAY_GFX_COLOR_GRAY);
}

static void Display_PagesDrawMotorLayout(void)
{
  uint16_t estop_text_x = (uint16_t)(DISPLAY_MOTOR_ESTOP_X + ((DISPLAY_MOTOR_ESTOP_W - 64U) / 2U));
  uint16_t estop_text_y = (uint16_t)(DISPLAY_MOTOR_ESTOP_Y + ((DISPLAY_MOTOR_ESTOP_H - 32U) / 2U));

  Display_PagesDrawSystemColumnAt(DISPLAY_MOTOR_STATUS_X, DISPLAY_MOTOR_STATUS_W);

  (void)Display_GfxDrawFrame(DISPLAY_MOTOR_PANEL_X, DISPLAY_DASH_Y, DISPLAY_MOTOR_PANEL_W, DISPLAY_DASH_H, DISPLAY_GFX_COLOR_GRAY, DISPLAY_GFX_COLOR_WHITE);
  (void)Display_TextDrawLabel((uint16_t)(DISPLAY_MOTOR_PANEL_X + 10U), (uint16_t)(DISPLAY_DASH_Y + 4U), DISPLAY_TEXT_MOTOR_PWM, DISPLAY_GFX_COLOR_BLUE);
  (void)Display_GfxDrawHLine(DISPLAY_MOTOR_PANEL_X, (uint16_t)(DISPLAY_DASH_Y + 28U), DISPLAY_MOTOR_PANEL_W, DISPLAY_GFX_COLOR_GRAY);

  Display_PagesDrawMotorSliderStatic(DISPLAY_MOTOR_TRACK1_X, '1');
  Display_PagesDrawMotorSliderStatic(DISPLAY_MOTOR_TRACK2_X, '2');
  Display_PagesDrawMotorSliderStatic(DISPLAY_MOTOR_TRACK3_X, '3');
  Display_PagesDrawMotorSliderStatic(DISPLAY_MOTOR_TRACK4_X, '4');

  (void)Display_GfxDrawFrame(DISPLAY_MOTOR_ESTOP_X, DISPLAY_MOTOR_ESTOP_Y, DISPLAY_MOTOR_ESTOP_W, DISPLAY_MOTOR_ESTOP_H, DISPLAY_GFX_COLOR_RED, DISPLAY_GFX_COLOR_RED);
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
      (void)Display_GfxDrawFrame(col_x[c], row_y[r], cell_w, cell_h, DISPLAY_GFX_COLOR_GRAY, DISPLAY_GFX_COLOR_WHITE);
      if (motor_index[i] != 0) {
        Display_PagesDrawSelfMotorLabel(col_x[c], (uint16_t)(row_y[r] + 28U), cell_w, motor_index[i]);
      } else {
        label_w = Display_TextGetLabelWidth(labels[i]);
        label_x = (label_w < cell_w) ? (uint16_t)(col_x[c] + ((cell_w - label_w) / 2U)) : col_x[c];
        (void)Display_TextDrawLabel(label_x, (uint16_t)(row_y[r] + 28U), labels[i], DISPLAY_GFX_COLOR_DARK);
      }
    }
  }

  (void)Display_GfxDrawFrame(480U, 82U, 312U, 310U, DISPLAY_GFX_COLOR_GRAY, DISPLAY_GFX_COLOR_WHITE);
  (void)Display_TextDrawLabel(496U, 94U, DISPLAY_TEXT_ERROR_CODE, DISPLAY_GFX_COLOR_DARK);
  (void)Display_GfxDrawHLine(480U, 118U, 312U, DISPLAY_GFX_COLOR_GRAY);
  (void)Display_TextDrawLabel(492U, 136U, DISPLAY_TEXT_CODE, DISPLAY_GFX_COLOR_DARK);
  (void)Display_TextDrawLabel(578U, 136U, DISPLAY_TEXT_MODULE, DISPLAY_GFX_COLOR_DARK);
  (void)Display_TextDrawLabel(684U, 136U, DISPLAY_TEXT_REASON, DISPLAY_GFX_COLOR_DARK);
  (void)Display_GfxDrawHLine(480U, 152U, 312U, DISPLAY_GFX_COLOR_GRAY);
  (void)Display_TextDrawLabel(604U, 170U, DISPLAY_TEXT_FAILED_MODULES, DISPLAY_GFX_COLOR_GRAY);
}

/*
 * 根据运行期告警码获取告警原因文本。
 */
/* Alarm page fixed layout. */
static void Display_PagesDrawAlarmTitle(uint16_t x, uint16_t y)
{
  (void)Display_TextDrawLabel(x, y, DISPLAY_TEXT_ALARM_SUMMARY, DISPLAY_GFX_COLOR_RED);
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
  return ((row & 0x01U) == 0U) ? DISPLAY_GFX_COLOR_WHITE : DISPLAY_ALARM_COLOR_ROW_ALT;
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

  (void)Display_GfxFillRect(DISPLAY_ALARM_TABLE_X, DISPLAY_ALARM_TABLE_Y, DISPLAY_ALARM_TABLE_W, DISPLAY_ALARM_TABLE_H, DISPLAY_GFX_COLOR_WHITE);
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
static const char *Display_PagesGetAlarmModule(uint16_t source_id, uint16_t code)
{
  switch ((Px4Lite_ModuleId_t)source_id) {
    case PX4LITE_MODULE_GNSS:
      return "GPS";
    case PX4LITE_MODULE_IMU:
      return "IMU";
    case PX4LITE_MODULE_BARO:
      return "BARO";
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
      return "NO FIX";
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
    default:
      break;
  }

  return "UNKNOWN ALARM";
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
  if (page == DISPLAY_HMI_PAGE_LOGO) { return DISPLAY_HMI_PAGE_SELF_CHECK; }

  if (((uint16_t)page + 1U) >= DISPLAY_HMI_PAGE_COUNT) { return DISPLAY_HMI_PAGE_SELF_CHECK; }

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
 * 绘制右上角告警图标，有告警时红闪。
 */
Display_Result_t Display_PagesDrawAlarmIcon(uint32_t now_ms, Display_PagesValueReader_t read_value, uint8_t force_draw)
{
  uint32_t alarm_code;
  static uint16_t s_last_color = 0U;
  static uint8_t s_last_valid  = 0U;
  uint16_t color;
  uint8_t flash_on;

  if (Display_GfxIsReady() == 0U) { return DISPLAY_NOT_READY; }

  alarm_code = Display_PagesReadValue(read_value, DISPLAY_HMI_VAR_ALARM_CODE, 0U);

  if (alarm_code != 0U) {
    flash_on = (uint8_t)((now_ms % 800U) < 400U);
    color    = flash_on ? DISPLAY_GFX_COLOR_RED : DISPLAY_GFX_COLOR_DARK;
  } else {
    color = DISPLAY_GFX_COLOR_DARK;
  }

  if ((force_draw == 0U) && (s_last_valid != 0U) && (s_last_color == color)) { return DISPLAY_OK; }

  (void)Display_GfxFillCircle(776U, 20U, 7U, color);
  (void)Display_GfxDrawCircle(776U, 20U, 7U, DISPLAY_GFX_COLOR_WHITE);
  s_last_color = color;
  s_last_valid = 1U;

  return DISPLAY_OK;
}

/*
 * 绘制页面头部标题、网络状态和链路质量。
 */
Display_Result_t Display_PagesDrawHeader(Display_HmiPage_t page, Display_PagesValueReader_t read_value)
{
  uint32_t network_status;
  uint32_t loss_rate;
  uint32_t remote_mode;
  uint16_t title_width;
  uint16_t title_x;

  (void)page;

  if (Display_GfxIsReady() == 0U) { return DISPLAY_NOT_READY; }

  network_status = Display_PagesReadValue(read_value, DISPLAY_HMI_VAR_LORA_STATUS, 0U);
  loss_rate      = Display_PagesReadValue(read_value, DISPLAY_HMI_VAR_LORA_LOSS_RATE, 0U);
  remote_mode    = Display_PagesReadValue(read_value, DISPLAY_HMI_VAR_REMOTE_MODE, 0U);

  (void)Display_GfxFillRect(0U, 0U, DISPLAY_GFX_WIDTH, DISPLAY_HEADER_HEIGHT, DISPLAY_GFX_COLOR_DARK);
  title_width = Display_TextGetCompanyTitleWidth();
  title_x     = (DISPLAY_GFX_WIDTH > title_width) ? (uint16_t)((DISPLAY_GFX_WIDTH - title_width) / 2U) : 0U;
  (void)Display_GfxDrawString(12U, 20U, (remote_mode != 0U) ? "REMOTE" : "LOCAL", DISPLAY_GFX_COLOR_WHITE, 1U);
  (void)Display_TextDrawCompanyTitle(title_x, 18U, DISPLAY_GFX_COLOR_WHITE);
  (void)Display_GfxDrawStatusDot(628U, 24U, 7U, Display_PagesGetStatusColor(network_status), DISPLAY_GFX_COLOR_WHITE);
  (void)Display_GfxDrawString(644U, 20U, "NET", DISPLAY_GFX_COLOR_WHITE, 1U);
  Display_PagesDrawHeaderLossRate(702U, 22U, loss_rate);
  (void)Display_GfxFillCircle(776U, 20U, 7U, DISPLAY_GFX_COLOR_DARK);
  (void)Display_GfxDrawCircle(776U, 20U, 7U, DISPLAY_GFX_COLOR_WHITE);
  (void)Display_GfxDrawHLine(0U, (uint16_t)(DISPLAY_HEADER_HEIGHT - 1U), DISPLAY_GFX_WIDTH, DISPLAY_GFX_COLOR_CYAN);

  return DISPLAY_OK;
}

/*
 * 绘制指定页面的固定背景、标签和导航区域。
 */
Display_Result_t Display_PagesDrawStatic(Display_HmiPage_t page, Display_PagesValueReader_t read_value)
{
  if (Display_GfxIsReady() == 0U) { return DISPLAY_NOT_READY; }

  if (page == DISPLAY_HMI_PAGE_LOGO) { return Display_PagesDrawLogoLayout(); }

  (void)Display_GfxClear(DISPLAY_GFX_COLOR_WHITE);

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
  } else {
    return DISPLAY_ERROR;
  }

  (void)Display_PagesDrawHeader(page, read_value);
  Display_PagesDrawFooter(page);

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

  (void)Display_GfxFillRect(481U, DISPLAY_SELFCHECK_ERR_BODY_Y, 310U, DISPLAY_SELFCHECK_ERR_BODY_H, DISPLAY_GFX_COLOR_WHITE);
  (void)Display_GfxDrawHLine(480U, 152U, 312U, DISPLAY_GFX_COLOR_GRAY);

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
    (void)Display_GfxDrawString(DISPLAY_SELFCHECK_ERR_MODULE_X, row_y, Display_PagesGetAlarmModule(source_id, fault_code), DISPLAY_GFX_COLOR_DARK, 1U);
    (void)Display_GfxDrawString(DISPLAY_SELFCHECK_ERR_REASON_X, row_y, Display_PagesGetAlarmReason(fault_code), DISPLAY_GFX_COLOR_DARK, 1U);
  }
}

static void Display_PagesDrawAlarmSummaryField(const Display_HmiVariableConfig_t *variable, uint32_t value)
{
  (void)value;

  (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_WHITE);
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
    Display_PagesFormatFaultCode(code_buf, fault_code);
    (void)Display_GfxDrawString((uint16_t)(DISPLAY_ALARM_TABLE_X + 30U), (uint16_t)(row_y + 16U), code_buf, DISPLAY_GFX_COLOR_RED, 2U);
    (void)Display_GfxDrawString((uint16_t)(DISPLAY_ALARM_CODE_X + 20U), (uint16_t)(row_y + 16U), Display_PagesGetAlarmModule(source_id, fault_code), DISPLAY_GFX_COLOR_DARK, 2U);
    (void)Display_GfxDrawString((uint16_t)(DISPLAY_ALARM_MODULE_X + 20U), (uint16_t)(row_y + 16U), Display_PagesGetAlarmReason(fault_code), DISPLAY_GFX_COLOR_DARK, 2U);
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

  (void)Display_GfxFillRect(clear_x, y, clear_w, height, DISPLAY_GFX_COLOR_WHITE);
  if ((fill_h != 0U) && (y_end > fill_y) && (y < fill_end)) {
    uint16_t blue_y = (y > fill_y) ? y : fill_y;
    uint16_t blue_h = (uint16_t)(((y_end < fill_end) ? y_end : fill_end) - blue_y);
    if (blue_h != 0U) { (void)Display_GfxFillRect(variable->x, blue_y, variable->width, blue_h, DISPLAY_GFX_COLOR_BLUE); }
  }
}

static void Display_PagesDrawMotorPercent(const Display_HmiVariableConfig_t *variable, uint16_t percent)
{
  (void)Display_GfxFillRect((uint16_t)(variable->x - 16U), (uint16_t)(variable->y + variable->height + 14U), 64U, 18U, DISPLAY_GFX_COLOR_WHITE);
  Display_PagesDrawWholePercent((uint16_t)(variable->x - 10U), (uint16_t)(variable->y + variable->height + 16U), percent, DISPLAY_GFX_COLOR_DARK);
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
    (void)Display_GfxDrawRect((uint16_t)(variable->x - 1U), (uint16_t)(variable->y - 1U), (uint16_t)(variable->width + 2U), (uint16_t)(variable->height + 2U), DISPLAY_GFX_COLOR_GRAY);
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
    (void)Display_GfxDrawRect((uint16_t)(variable->x - 1U), (uint16_t)(variable->y - 1U), (uint16_t)(variable->width + 2U), (uint16_t)(variable->height + 2U), DISPLAY_GFX_COLOR_GRAY);
  }

  (void)Display_GfxFillRect((uint16_t)(cx - DISPLAY_MOTOR_HANDLE_HALF_W), (uint16_t)(handle_cy - DISPLAY_MOTOR_HANDLE_HALF_H), (uint16_t)(DISPLAY_MOTOR_HANDLE_HALF_W * 2U), (uint16_t)(DISPLAY_MOTOR_HANDLE_HALF_H * 2U), DISPLAY_GFX_COLOR_CYAN);
  (void)Display_GfxDrawRect((uint16_t)(cx - DISPLAY_MOTOR_HANDLE_HALF_W), (uint16_t)(handle_cy - DISPLAY_MOTOR_HANDLE_HALF_H), (uint16_t)(DISPLAY_MOTOR_HANDLE_HALF_W * 2U), (uint16_t)(DISPLAY_MOTOR_HANDLE_HALF_H * 2U), DISPLAY_GFX_COLOR_DARK);
  (void)Display_GfxDrawHLine((uint16_t)(cx - DISPLAY_MOTOR_HANDLE_HALF_W + 3U), handle_cy, (uint16_t)((DISPLAY_MOTOR_HANDLE_HALF_W * 2U) - 6U), DISPLAY_GFX_COLOR_DARK);
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
static void Display_PagesDrawGnssFixText(const Display_HmiVariableConfig_t *variable, uint32_t value)
{
  const char *text;
  uint16_t color;

  if (value == 0U) {
    text  = "No Fix";
    color = DISPLAY_GFX_COLOR_RED;
  } else if (value == 2U) {
    text  = "2D Fix";
    color = DISPLAY_GFX_COLOR_YELLOW;
  } else if (value == 3U) {
    text  = "3D Fix";
    color = DISPLAY_GFX_COLOR_GREEN;
  } else if (value == 4U) {
    text  = "DGPS Fix";
    color = DISPLAY_GFX_COLOR_GREEN;
  } else {
    text  = "No Fix";
    color = DISPLAY_GFX_COLOR_RED;
  }

  (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_WHITE);
  (void)Display_GfxDrawString((uint16_t)(variable->x + 2U), (uint16_t)(variable->y + 2U), text, color, 2U);
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

  if ((variable->id == DISPLAY_HMI_VAR_BATTERY_PERCENT)) {
    limited_value = (value > 100U) ? 100U : (uint16_t)value;
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_WHITE);
    drawn_w = Display_PagesDrawU32Scaled((uint16_t)(variable->x + 2U), (uint16_t)(variable->y + 2U), limited_value, DISPLAY_GFX_COLOR_DARK, 2U);
    (void)Display_GfxDrawChar((uint16_t)(variable->x + 2U + drawn_w), (uint16_t)(variable->y + 2U), '%', DISPLAY_GFX_COLOR_DARK, 2U);
    (void)Display_GfxDrawRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_GRAY);
    return DISPLAY_OK;
  }

  if (((variable->id == DISPLAY_HMI_VAR_MOTOR_PWM_1) || (variable->id == DISPLAY_HMI_VAR_MOTOR_PWM_2) || (variable->id == DISPLAY_HMI_VAR_MOTOR_PWM_3) || (variable->id == DISPLAY_HMI_VAR_MOTOR_PWM_4)) && (variable->page == DISPLAY_HMI_PAGE_MOTOR)) { return Display_PagesDrawMotorSliderField(variable, value, value, 1U, 1U); }

  if ((variable->id == DISPLAY_HMI_VAR_MOTOR_PWM_1) || (variable->id == DISPLAY_HMI_VAR_MOTOR_PWM_2) || (variable->id == DISPLAY_HMI_VAR_MOTOR_PWM_3) || (variable->id == DISPLAY_HMI_VAR_MOTOR_PWM_4)) {
    limited_value = (value > 100U) ? 100U : (uint16_t)value;
    (void)Display_GfxDrawProgressBar(variable->x, variable->y, variable->width, variable->height, limited_value, 100U, DISPLAY_GFX_COLOR_BLUE, DISPLAY_GFX_COLOR_GRAY, DISPLAY_GFX_COLOR_BLACK);
    (void)Display_GfxFillRect((uint16_t)(variable->x + variable->width + 6U), variable->y, 34U, variable->height, DISPLAY_GFX_COLOR_WHITE);
    Display_PagesDrawWholePercent((uint16_t)(variable->x + variable->width + 8U), (uint16_t)(variable->y + 5U), limited_value, DISPLAY_GFX_COLOR_DARK);
    return DISPLAY_OK;
  }

  if (variable->id == DISPLAY_HMI_VAR_LORA_LOSS_RATE) {
    limited_value = (value > 1000U) ? 1000U : (uint16_t)value;
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_WHITE);
    Display_PagesDrawTenthsPercent((uint16_t)(variable->x + 2U), (uint16_t)(variable->y + 5U), limited_value, DISPLAY_GFX_COLOR_DARK);
    return DISPLAY_OK;
  }

  if ((variable->id == DISPLAY_HMI_VAR_GNSS_FIX) && (variable->page == DISPLAY_HMI_PAGE_DATA)) {
    Display_PagesDrawGnssFixText(variable, value);
    return DISPLAY_OK;
  }

  if ((variable->id == DISPLAY_HMI_VAR_LORA_STATUS) && (variable->y < DISPLAY_HEADER_HEIGHT)) {
    dot_x = (uint16_t)(variable->x + 12U);
    dot_y = (uint16_t)(variable->y + (variable->height / 2U));
    (void)Display_GfxFillCircle(dot_x, dot_y, 8U, DISPLAY_GFX_COLOR_DARK);
    (void)Display_GfxDrawStatusDot(dot_x, dot_y, 7U, Display_PagesGetStatusColor(value), DISPLAY_GFX_COLOR_WHITE);
    return DISPLAY_OK;
  }

  if (variable->id == DISPLAY_HMI_VAR_GNSS_HDOP) {
    whole = value / 100U;
    frac  = value % 100U;
    Display_PagesFmtUnsigned(fmt_buf, whole, frac, 2U, 100U);
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_WHITE);
    (void)Display_GfxDrawString((uint16_t)(variable->x + 2U), (uint16_t)(variable->y + 2U), fmt_buf, DISPLAY_GFX_COLOR_DARK, 2U);
    (void)Display_GfxDrawRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_GRAY);
    return DISPLAY_OK;
  }

  if ((variable->id == DISPLAY_HMI_VAR_SELF_CHECK_5GA) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_MPU6050) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_BME280) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_SD) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_MOTOR) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_2) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_3) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_4) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_POWER) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_BUZZER) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_KEY) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_DEBUG) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_GNSS) || (variable->id == DISPLAY_HMI_VAR_SELF_CHECK_LORA) || (variable->id == DISPLAY_HMI_VAR_SYSTEM_STATUS) || (variable->id == DISPLAY_HMI_VAR_GNSS_FIX) || (variable->id == DISPLAY_HMI_VAR_LORA_STATUS) || (variable->id == DISPLAY_HMI_VAR_LORA_HEARTBEAT) ||
      (variable->id == DISPLAY_HMI_VAR_ALARM_CODE)) {
    dot_x = (uint16_t)(variable->x + 12U);
    dot_y = (uint16_t)(variable->y + (variable->height / 2U));
    (void)Display_GfxFillRect((uint16_t)(dot_x - 10U), (uint16_t)(dot_y - 10U), 20U, 20U, DISPLAY_GFX_COLOR_WHITE);
    (void)Display_GfxDrawStatusDot(dot_x, dot_y, 8U, Display_PagesGetStatusColor(value), DISPLAY_GFX_COLOR_BLACK);
    return DISPLAY_OK;
  }

  vx = (uint16_t)(variable->x + 2U);
  vy = (uint16_t)(variable->y + 2U);

  /* 飞行时间(上电后运行)：秒 -> HH:MM:SS */
  if (variable->id == DISPLAY_HMI_VAR_FLIGHT_TIME_S) {
    Display_PagesFmtHms(fmt_buf, value / 3600U, (value / 60U) % 60U, value % 60U);
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_WHITE);
    (void)Display_GfxDrawString(vx, vy, fmt_buf, DISPLAY_GFX_COLOR_DARK, 2U);
    (void)Display_GfxDrawRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_GRAY);
    return DISPLAY_OK;
  }

  /* 本地显示时间：HHMMSS -> HH:MM:SS */
  if (variable->id == DISPLAY_HMI_VAR_CLOCK_TIME) {
    Display_PagesFmtHms(fmt_buf, (value / 10000U) % 100U, (value / 100U) % 100U, value % 100U);
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_WHITE);
    (void)Display_GfxDrawString(vx, vy, fmt_buf, DISPLAY_GFX_COLOR_DARK, 2U);
    (void)Display_GfxDrawRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_GRAY);
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
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_WHITE);
    (void)Display_GfxDrawString(vx, vy, fmt_buf, DISPLAY_GFX_COLOR_DARK, 2U);
    (void)Display_GfxDrawRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_GRAY);
    return DISPLAY_OK;
  }

  /* Variables with decimal formatting and unit strings */
  if (variable->id == DISPLAY_HMI_VAR_BATTERY_VOLTAGE) {
    whole = value / 100U;
    frac  = value % 100U;
    Display_PagesFmtUnsigned(fmt_buf, whole, frac, 2U, 100U);
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_WHITE);
    (void)Display_GfxDrawString(vx, vy, fmt_buf, DISPLAY_GFX_COLOR_DARK, 2U);
    drawn_w = (uint16_t)(Display_PagesStrLen(fmt_buf) * 12U);
    Display_PagesDrawUnitAfter(vx, vy, drawn_w, "V");
    (void)Display_GfxDrawRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_GRAY);
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
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_WHITE);
    (void)Display_GfxDrawString(vx, vy, fmt_buf, DISPLAY_GFX_COLOR_DARK, 2U);
    drawn_w = (uint16_t)(Display_PagesStrLen(fmt_buf) * 12U);
    Display_PagesDrawUnitAfter(vx, vy, drawn_w, DISPLAY_PAGES_DEG_STR);
    (void)Display_GfxDrawRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_GRAY);
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
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_WHITE);
    (void)Display_GfxDrawString(vx, vy, fmt_buf, DISPLAY_GFX_COLOR_DARK, 2U);
    drawn_w = (uint16_t)(Display_PagesStrLen(fmt_buf) * 12U);
    Display_PagesDrawUnitAfter(vx, vy, drawn_w, "m");
    (void)Display_GfxDrawRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_GRAY);
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
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_WHITE);
    (void)Display_GfxDrawString(vx, vy, fmt_buf, DISPLAY_GFX_COLOR_DARK, 2U);
    drawn_w = (uint16_t)(Display_PagesStrLen(fmt_buf) * 12U);
    Display_PagesDrawUnitAfter(vx, vy, drawn_w, DISPLAY_PAGES_DEG_STR);
    (void)Display_GfxDrawRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_GRAY);
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
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_WHITE);
    (void)Display_GfxDrawString(vx, vy, fmt_buf, DISPLAY_GFX_COLOR_DARK, 2U);
    drawn_w = (uint16_t)(Display_PagesStrLen(fmt_buf) * 12U);
    Display_PagesDrawUnitAfter(vx, vy, drawn_w, DISPLAY_PAGES_DEGC_STR);
    (void)Display_GfxDrawRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_GRAY);
    return DISPLAY_OK;
  }

  if (variable->id == DISPLAY_HMI_VAR_HUMIDITY) {
    whole = value / 10U;
    frac  = value % 10U;
    Display_PagesFmtUnsigned(fmt_buf, whole, frac, 1U, 10U);
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_WHITE);
    (void)Display_GfxDrawString(vx, vy, fmt_buf, DISPLAY_GFX_COLOR_DARK, 2U);
    drawn_w = (uint16_t)(Display_PagesStrLen(fmt_buf) * 12U);
    Display_PagesDrawUnitAfter(vx, vy, drawn_w, "%");
    (void)Display_GfxDrawRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_GRAY);
    return DISPLAY_OK;
  }

  /* Generic signed integer with optional unit */
  if ((variable->data_type == DISPLAY_HMI_TYPE_I32) || (variable->data_type == DISPLAY_HMI_TYPE_I16)) {
    sval = (variable->data_type == DISPLAY_HMI_TYPE_I32) ? (int32_t)value : (int16_t)(value & 0xFFFFU);
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_WHITE);
    drawn_w = Display_PagesDrawI32(vx, vy, sval, DISPLAY_GFX_COLOR_DARK);
    total_w = drawn_w;
    if (variable->unit[0] != '-' && variable->unit[0] != '\0') { total_w = Display_PagesDrawUnitAfter(vx, vy, drawn_w, variable->unit); }
    if ((total_w + 4U) < variable->width) { (void)Display_GfxFillRect((uint16_t)(variable->x + 2U + total_w), (uint16_t)(variable->y + 2U), (uint16_t)(variable->width - 4U - total_w), (uint16_t)(variable->height - 4U), DISPLAY_GFX_COLOR_WHITE); }
  } else if ((variable->data_type == DISPLAY_HMI_TYPE_U32) || (variable->data_type == DISPLAY_HMI_TYPE_U16)) {
    (void)Display_GfxFillRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_WHITE);
    drawn_w = Display_PagesDrawU32(vx, vy, value, DISPLAY_GFX_COLOR_DARK);
    total_w = drawn_w;
    if (variable->unit[0] != '-' && variable->unit[0] != '\0') { total_w = Display_PagesDrawUnitAfter(vx, vy, drawn_w, variable->unit); }
    if ((total_w + 4U) < variable->width) { (void)Display_GfxFillRect((uint16_t)(variable->x + 2U + total_w), (uint16_t)(variable->y + 2U), (uint16_t)(variable->width - 4U - total_w), (uint16_t)(variable->height - 4U), DISPLAY_GFX_COLOR_WHITE); }
  }

  (void)Display_GfxDrawRect(variable->x, variable->y, variable->width, variable->height, DISPLAY_GFX_COLOR_GRAY);
  return DISPLAY_OK;
}
