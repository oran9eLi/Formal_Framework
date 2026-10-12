/**
 * @file display_pages.c
 * @brief Display 层固定容量事件视图；只在显示任务内使用。
 * @details 全量快照复制处理环覆盖和序号回绕，不把历史告警当活动告警。
 *          不读硬件，不修改业务历史，无动态分配。
 */

#include "display_pages.h"
#include "px4lite_local_msglog.h"

#include <string.h>

#define DISPLAY_MSGLOG_CAP APP_DISPLAY_LOG_CAP

static Display_MessageLogEntry_t s_msglog[DISPLAY_MSGLOG_CAP];
static Display_MessageLogEntry_t s_msglog_alarm;
static uint16_t s_msglog_head;
static uint16_t s_msglog_count;
static uint8_t s_msglog_alarm_valid;
static uint32_t s_msglog_version;
static uint32_t s_source_version;
static uint8_t s_source_valid;

/** @brief 按源版本复制完整窗口；不依赖序号普通大小关系或历史告警特殊行。 */
void Display_PagesSetLogSnapshot(const App_DisplayLogSnapshot_t *snapshot)
{
  uint16_t i;
  if (snapshot == 0) { return; }
  if ((s_source_valid != 0U) && (snapshot->version == s_source_version) &&
      (snapshot->count == s_msglog_count)) { return; }
  s_msglog_count = (snapshot->count > DISPLAY_MSGLOG_CAP) ? DISPLAY_MSGLOG_CAP : snapshot->count;
  for (i = 0U; i < s_msglog_count; i++) {
    const App_DisplayLogEntry_t *entry = &snapshot->entries[i];
    s_msglog[i].msg = (Display_LogMsg_t)entry->message_id;
    s_msglog[i].time_hhmmss = entry->time_hhmmss;
    s_msglog[i].sequence = entry->sequence;
    s_msglog[i].fault_code = entry->fault_code;
    s_msglog[i].severity = entry->severity;
    s_msglog[i].source_id = entry->source_id;
    s_msglog[i].active = entry->active;
  }
  s_msglog_head = (uint16_t)(s_msglog_count % DISPLAY_MSGLOG_CAP);
  s_msglog_alarm_valid = 0U;
  s_source_version = snapshot->version;
  s_source_valid = 1U;
  s_msglog_version++;
}

/** @brief 旧显示注入兼容函数；不修改生产日志，下一份快照会覆盖本视图。 */
void Display_PagesPushLogMessage(Display_LogMsg_t msg, uint32_t time_hhmmss)
{
  if ((uint32_t)msg >= DISPLAY_LOGMSG_COUNT) { return; }

  memset(&s_msglog[s_msglog_head], 0, sizeof(s_msglog[s_msglog_head]));
  s_msglog[s_msglog_head].msg        = msg;
  s_msglog[s_msglog_head].time_hhmmss = time_hhmmss;
  s_msglog[s_msglog_head].severity = Px4Lite_LogMessageSeverity((uint16_t)msg);
  s_msglog_head = (uint16_t)((s_msglog_head + 1U) % DISPLAY_MSGLOG_CAP);
  if (s_msglog_count < DISPLAY_MSGLOG_CAP) { s_msglog_count++; }
  s_source_valid = 0U;
  s_msglog_version++;
}

/** @brief 清空显示缓存；不清除底层事件、SD 或活动告警。 */
void Display_PagesClearLogMessages(void)
{
  s_msglog_head        = 0U;
  s_msglog_count       = 0U;
  s_msglog_alarm_valid = 0U;
  s_source_valid      = 0U;
  memset(s_msglog, 0, sizeof(s_msglog));
  memset(&s_msglog_alarm, 0, sizeof(s_msglog_alarm));
  s_msglog_version++;
}

/** @brief 按时间旧到新复制最近事件，保留所有严重度的历史。 */
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

/** @brief 返回只随内容变化的缓存修订号。 */
uint32_t Display_PagesGetLogVersion(void)
{
  return s_msglog_version;
}
