#ifndef DISPLAY_PAGES_H
#define DISPLAY_PAGES_H

#include "display.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  DISPLAY_LOGMSG_SYSTEM_START = 0,
  DISPLAY_LOGMSG_SELFCHECK_OK,
  DISPLAY_LOGMSG_SELFCHECK_PART,
  DISPLAY_LOGMSG_SELFCHECK_FAIL,
  DISPLAY_LOGMSG_GPS_OK,
  DISPLAY_LOGMSG_GPS_NOSIG,
  DISPLAY_LOGMSG_GPS_LOST,
  DISPLAY_LOGMSG_ATT_OK,
  DISPLAY_LOGMSG_ATT_LOST,
  DISPLAY_LOGMSG_ENV_OK,
  DISPLAY_LOGMSG_ENV_LOST,
  DISPLAY_LOGMSG_COMM_OK,
  DISPLAY_LOGMSG_COMM_LOST,
  DISPLAY_LOGMSG_STORAGE_OK,
  DISPLAY_LOGMSG_STORAGE_LOST,
  DISPLAY_LOGMSG_MOTOR_OK,
  DISPLAY_LOGMSG_MOTOR_DISCONNECT,
  DISPLAY_LOGMSG_MOTOR_LOWPOWER,
  DISPLAY_LOGMSG_MOTOR1_FAIL,
  DISPLAY_LOGMSG_MOTOR2_FAIL,
  DISPLAY_LOGMSG_MOTOR3_FAIL,
  DISPLAY_LOGMSG_MOTOR4_FAIL,
  DISPLAY_LOGMSG_MOTOR_ALL_FAIL,
  DISPLAY_LOGMSG_MOTOR_DEAD,
  DISPLAY_LOGMSG_MOTOR_CHARGE,
  DISPLAY_LOGMSG_MAIN_CHARGE,
  DISPLAY_LOGMSG_ALARM_ACTIVE,
  DISPLAY_LOGMSG_ALARM_NONE,
  DISPLAY_LOGMSG_COUNT
} Display_LogMsg_t;

typedef struct {
  Display_LogMsg_t msg;
  uint32_t time_hhmmss;
} Display_MessageLogEntry_t;

void Display_PagesPushLogMessage(Display_LogMsg_t msg, uint32_t time_hhmmss);
void Display_PagesClearLogMessages(void);
uint16_t Display_PagesCopyLogMessages(Display_MessageLogEntry_t *entries, uint16_t max_count, Display_MessageLogEntry_t *alarm_entry, uint8_t *alarm_valid);
uint32_t Display_PagesGetLogVersion(void);

#define DISPLAY_LORA_MAX_NODES     16U
#define DISPLAY_LORA_ROWS_PER_PAGE 7U

typedef struct {
  uint16_t node_id;
  uint16_t label;
  uint32_t last_comm_hhmmss;
} Display_LoraNode_t;

typedef void (*Display_LoraConnectHandler_t)(uint16_t node_id, uint8_t connect);

typedef enum {
  DISPLAY_LORA_TOUCH_NONE = 0,
  DISPLAY_LORA_TOUCH_REDRAW,
  DISPLAY_LORA_TOUCH_COMMAND
} Display_LoraTouchResult_t;

void Display_PagesSetSelfCheckFaults(const uint32_t *faults, uint16_t count);
void Display_PagesSetLoraNodes(const Display_LoraNode_t *nodes, uint16_t count);
void Display_PagesSetLoraConnected(uint8_t connected, uint16_t node_id);
void Display_PagesSetLoraConnectHandler(Display_LoraConnectHandler_t handler);
uint32_t Display_PagesGetLoraVersion(void);
Display_LoraTouchResult_t Display_PagesLoraHandleTouch(uint16_t x, uint16_t y);
void Display_PagesDrawLoraContent(void);
uint8_t Display_PagesLoraContentDirty(void);

#ifdef __cplusplus
}
#endif

#endif
