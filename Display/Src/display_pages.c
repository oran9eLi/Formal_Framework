/**
 * @file display_pages.c
 * @brief Data buffers shared by the Display service and the LVGL renderer.
 */

#include "display_pages.h"

#include <string.h>

#define DISPLAY_MSGLOG_CAP 9U

static Display_MessageLogEntry_t s_msglog[DISPLAY_MSGLOG_CAP];
static Display_MessageLogEntry_t s_msglog_alarm;
static uint16_t s_msglog_head;
static uint16_t s_msglog_count;
static uint8_t s_msglog_alarm_valid;
static uint32_t s_msglog_version;

static Display_LoraNode_t s_lora_nodes[DISPLAY_LORA_MAX_NODES];
static uint16_t s_lora_count;
static uint8_t s_lora_connected;
static uint16_t s_lora_connected_node_id;
static uint32_t s_lora_version;
static Display_LoraConnectHandler_t s_lora_handler;

static uint8_t Display_PagesIsAlarmLogMessage(Display_LogMsg_t msg)
{
  return ((msg == DISPLAY_LOGMSG_ALARM_ACTIVE) || (msg == DISPLAY_LOGMSG_ALARM_NONE)) ? 1U : 0U;
}

void Display_PagesPushLogMessage(Display_LogMsg_t msg, uint32_t time_hhmmss)
{
  if (msg >= DISPLAY_LOGMSG_COUNT) { return; }

  if (Display_PagesIsAlarmLogMessage(msg) != 0U) {
    s_msglog_alarm.msg        = msg;
    s_msglog_alarm.time_hhmmss = time_hhmmss;
    s_msglog_alarm_valid      = 1U;
    s_msglog_version++;
    return;
  }

  s_msglog[s_msglog_head].msg        = msg;
  s_msglog[s_msglog_head].time_hhmmss = time_hhmmss;
  s_msglog_head = (uint16_t)((s_msglog_head + 1U) % DISPLAY_MSGLOG_CAP);
  if (s_msglog_count < DISPLAY_MSGLOG_CAP) { s_msglog_count++; }
  s_msglog_version++;
}

void Display_PagesClearLogMessages(void)
{
  s_msglog_head        = 0U;
  s_msglog_count       = 0U;
  s_msglog_alarm_valid = 0U;
  memset(s_msglog, 0, sizeof(s_msglog));
  memset(&s_msglog_alarm, 0, sizeof(s_msglog_alarm));
  s_msglog_version++;
}

uint16_t Display_PagesCopyLogMessages(Display_MessageLogEntry_t *entries, uint16_t max_count, Display_MessageLogEntry_t *alarm_entry, uint8_t *alarm_valid)
{
  uint16_t copy_count;
  uint16_t first;
  uint16_t i;

  if (alarm_valid != 0) { *alarm_valid = s_msglog_alarm_valid; }
  if ((alarm_entry != 0) && (s_msglog_alarm_valid != 0U)) { *alarm_entry = s_msglog_alarm; }

  if ((entries == 0) || (max_count == 0U) || (s_msglog_count == 0U)) { return 0U; }

  copy_count = (s_msglog_count > max_count) ? max_count : s_msglog_count;
  first      = (uint16_t)(s_msglog_count - copy_count);
  for (i = 0U; i < copy_count; i++) {
    uint16_t idx = (uint16_t)((s_msglog_head + DISPLAY_MSGLOG_CAP - s_msglog_count + first + i) % DISPLAY_MSGLOG_CAP);
    entries[i] = s_msglog[idx];
  }

  return copy_count;
}

uint32_t Display_PagesGetLogVersion(void)
{
  return s_msglog_version;
}

void Display_PagesSetSelfCheckFaults(const uint32_t *faults, uint16_t count)
{
  (void)faults;
  (void)count;
}

void Display_PagesSetLoraNodes(const Display_LoraNode_t *nodes, uint16_t count)
{
  uint16_t copy_count;

  copy_count = (count > DISPLAY_LORA_MAX_NODES) ? DISPLAY_LORA_MAX_NODES : count;
  if ((nodes != 0) && (copy_count > 0U)) {
    memcpy(s_lora_nodes, nodes, (size_t)copy_count * sizeof(s_lora_nodes[0]));
  }
  s_lora_count = copy_count;
  s_lora_version++;
}

void Display_PagesSetLoraConnected(uint8_t connected, uint16_t node_id)
{
  s_lora_connected        = (connected != 0U) ? 1U : 0U;
  s_lora_connected_node_id = node_id;
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

Display_LoraTouchResult_t Display_PagesLoraHandleTouch(uint16_t x, uint16_t y)
{
  (void)x;
  (void)y;
  (void)s_lora_nodes;
  (void)s_lora_count;
  (void)s_lora_connected;
  (void)s_lora_connected_node_id;
  (void)s_lora_handler;
  return DISPLAY_LORA_TOUCH_NONE;
}

void Display_PagesDrawLoraContent(void)
{
}

uint8_t Display_PagesLoraContentDirty(void)
{
  return 0U;
}
