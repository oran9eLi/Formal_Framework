#include "display_log.h"

#define DISPLAY_LOG_CAP 9U

static Display_MessageLogEntry_t s_log_entries[DISPLAY_LOG_CAP];
static Display_MessageLogEntry_t s_alarm_entry;
static uint16_t s_log_head;
static uint16_t s_log_count;
static uint8_t s_alarm_valid;
static uint32_t s_log_version;

static uint8_t Display_LogIsAlarmMessage(Display_LogMsg_t msg)
{
  return ((msg == DISPLAY_LOGMSG_ALARM_ACTIVE) || (msg == DISPLAY_LOGMSG_ALARM_NONE)) ? 1U : 0U;
}

void Display_LogPushMessage(Display_LogMsg_t msg, uint32_t time_hhmmss)
{
  if (msg >= DISPLAY_LOGMSG_COUNT) {
    return;
  }

  if (Display_LogIsAlarmMessage(msg) != 0U) {
    s_alarm_entry.msg         = msg;
    s_alarm_entry.time_hhmmss = time_hhmmss;
    s_alarm_valid             = 1U;
    s_log_version++;
    return;
  }

  s_log_entries[s_log_head].msg         = msg;
  s_log_entries[s_log_head].time_hhmmss = time_hhmmss;
  s_log_head                            = (uint16_t)((s_log_head + 1U) % DISPLAY_LOG_CAP);
  if (s_log_count < DISPLAY_LOG_CAP) {
    s_log_count++;
  }
  s_log_version++;
}

uint32_t Display_LogGetVersion(void)
{
  return s_log_version;
}

uint16_t Display_LogCopyMessages(Display_MessageLogEntry_t *entries, uint16_t max_count, Display_MessageLogEntry_t *alarm_entry, uint8_t *alarm_valid)
{
  uint16_t copy_count;
  uint16_t first;
  uint16_t i;

  if (alarm_valid != 0) {
    *alarm_valid = s_alarm_valid;
  }
  if ((alarm_entry != 0) && (s_alarm_valid != 0U)) {
    *alarm_entry = s_alarm_entry;
  }

  if ((entries == 0) || (max_count == 0U)) {
    return 0U;
  }

  copy_count = (s_log_count < max_count) ? s_log_count : max_count;
  first      = (uint16_t)(s_log_count - copy_count);

  for (i = 0U; i < copy_count; i++) {
    uint16_t idx = (uint16_t)((s_log_head + DISPLAY_LOG_CAP - s_log_count + first + i) % DISPLAY_LOG_CAP);
    entries[i] = s_log_entries[idx];
  }

  return copy_count;
}
