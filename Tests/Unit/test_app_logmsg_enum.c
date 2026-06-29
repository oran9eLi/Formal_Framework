/**
 * @file test_app_logmsg_enum.c
 * @brief 锁定 App_LogMessageId_t 取值(append-only 契约)，防重排/改号。
 */
#include <stdio.h>
#include "app_message_log.h"

static int g_fail;
static void Eq(const char *n, int a, int e)
{ if (a != e) { printf("FAIL %s actual=%d expected=%d\n", n, a, e); g_fail++; } }

int main(void)
{
  Eq("SYSTEM_START", APP_LOGMSG_SYSTEM_START, 0);
  Eq("SELFCHECK_OK", APP_LOGMSG_SELFCHECK_OK, 1);
  Eq("GPS_OK", APP_LOGMSG_GPS_OK, 4);
  Eq("COMM_LOST", APP_LOGMSG_COMM_LOST, 12);
  Eq("MOTOR_OK", APP_LOGMSG_MOTOR_OK, 15);
  Eq("MAIN_CHARGE", APP_LOGMSG_MAIN_CHARGE, 25);
  Eq("ALARM_ACTIVE", APP_LOGMSG_ALARM_ACTIVE, 26);
  Eq("ALARM_NONE", APP_LOGMSG_ALARM_NONE, 27);
  Eq("COUNT", APP_LOGMSG_COUNT, 28);
  if (g_fail == 0) { printf("PASS test_app_logmsg_enum\n"); return 0; }
  printf("FAIL test_app_logmsg_enum fails=%d\n", g_fail); return 1;
}
