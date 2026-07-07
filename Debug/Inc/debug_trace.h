/**
 * @file debug_trace.h
 * @brief Define compile-time debug trace macros and text constants.
 */

#ifndef DEBUG_TRACE_H
#define DEBUG_TRACE_H

#include "debug_console.h"

/***************DEBUG***************/
#define DBG_TEXT_LOW_ALTITUDE          "\xB5\xCD\xBF\xD5"
#define DBG_TEXT_DEBUG_MODE            "\xB5\xF7\xCA\xD4\xC4\xA3\xCA\xBD"
#define DBG_TEXT_UART_READY            "\xB5\xF7\xCA\xD4\xB4\xAE\xBF\xDA\xD2\xD1\xBE\xCD\xD0\xF7"
#define DBG_TEXT_UART_INIT_STATUS      "\xB4\xAE\xBF\xDA\xB3\xF5\xCA\xBC\xBB\xAF\xD7\xB4\xCC\xAC"
#define DBG_TEXT_SYSTEM_CLOCK          "\xCF\xB5\xCD\xB3\xCA\xB1\xD6\xD3"
#define DBG_TEXT_PROJECT               "\xB9\xA4\xB3\xCC"
#define DBG_TEXT_LOOP_SEND             "\xD1\xAD\xBB\xB7\xB7\xA2\xCB\xCD"
#define DBG_TEXT_BOOT_HOLD             "\xC6\xF4\xB6\xAF\xB1\xA3\xB3\xD6"
#define DBG_TEXT_INIT_START            "\xB3\xF5\xCA\xBC\xBB\xAF\xBF\xAA\xCA\xBC"
#define DBG_TEXT_INIT_DONE             "\xB3\xF5\xCA\xBC\xBB\xAF\xCD\xEA\xB3\xC9"
#define DBG_TEXT_APP_TASK_INIT_START   "\xD3\xA6\xD3\xC3\xC8\xCE\xCE\xF1\xB3\xF5\xCA\xBC\xBB\xAF\xBF\xAA\xCA\xBC"
#define DBG_TEXT_APP_TASK_INIT_RESULT  "\xD3\xA6\xD3\xC3\xC8\xCE\xCE\xF1\xB3\xF5\xCA\xBC\xBB\xAF\xBD\xE1\xB9\xFB"
#define DBG_TEXT_APP_TASK_INIT_FAILED  "\xD3\xA6\xD3\xC3\xC8\xCE\xCE\xF1\xB3\xF5\xCA\xBC\xBB\xAF\xCA\xA7\xB0\xDC"
#define DBG_TEXT_ENTER                 "\xBD\xF8\xC8\xEB"
#define DBG_TEXT_SCHEDULER             "\xB5\xF7\xB6\xC8\xC6\xF7"
#define DBG_TEXT_ABNORMAL_RETURN       "\xD2\xEC\xB3\xA3\xB7\xB5\xBB\xD8"
#define DBG_TEXT_POST_INIT_START       "\xB3\xF5\xCA\xBC\xBB\xAF\xBD\xD7\xB6\xCE\xBF\xAA\xCA\xBC"
#define DBG_TEXT_POST_INIT_DONE        "\xB3\xF5\xCA\xBC\xBB\xAF\xBD\xD7\xB6\xCE\xCD\xEA\xB3\xC9"
#define DBG_TEXT_RESULT                "\xBD\xE1\xB9\xFB"
#define DBG_TEXT_POST_SELF_CHECK_START "\xD7\xD4\xBC\xEC\xBD\xD7\xB6\xCE\xBF\xAA\xCA\xBC"
#define DBG_TEXT_POST_SELF_CHECK_DONE  "\xD7\xD4\xBC\xEC\xBD\xD7\xB6\xCE\xCD\xEA\xB3\xC9"
#define DBG_TEXT_TASKS_READY           "\xC8\xCE\xCE\xF1\xB4\xB4\xBD\xA8\xCD\xEA\xB3\xC9\xA3\xAC\xD7\xBC\xB1\xB8\xC6\xF4\xB6\xAF\xB5\xF7\xB6\xC8\xC6\xF7"
#define DBG_TEXT_STARTED_OK            "\xD2\xD1\xC6\xF4\xB6\xAF\xA3\xAC\xD7\xB4\xCC\xAC\x3D\xD5\xFD\xB3\xA3"
#define DBG_TEXT_SAMPLE                "\xB2\xC9\xD1\xF9"
#define DBG_TEXT_LATITUDE              "\xCE\xB3\xB6\xC8"
#define DBG_TEXT_LONGITUDE             "\xBE\xAD\xB6\xC8"
#define DBG_TEXT_ALTITUDE              "\xB8\xDF\xB6\xC8"
#define DBG_TEXT_FRAME                 "\xD6\xA1"
#define DBG_TEXT_SENSOR_READ           "\xB4\xAB\xB8\xD0\xC6\xF7\xB6\xC1\xC8\xA1"
#define DBG_TEXT_SAMPLE_COUNT          "\xD1\xF9\xB1\xBE"
#define DBG_TEXT_STATUS                "\xD7\xB4\xCC\xAC"

#if DEBUG_BOOT_LOG_ENABLE

#define DBG_TRACE_TASK_STARTED(name)           DBG_TASK_INFO((name), DBG_TEXT_STARTED_OK)
#define DBG_TRACE_POST_INIT_START()            DBG_PRINT("POST " DBG_TEXT_POST_INIT_START)
#define DBG_TRACE_POST_INIT_DONE(result)       DBG_PRINT("POST " DBG_TEXT_POST_INIT_DONE ": " DBG_TEXT_RESULT "=%d", (result))
#define DBG_TRACE_POST_SELF_CHECK_START()      DBG_PRINT("POST " DBG_TEXT_POST_SELF_CHECK_START)
#define DBG_TRACE_POST_SELF_CHECK_DONE(result) DBG_PRINT("POST " DBG_TEXT_POST_SELF_CHECK_DONE ": " DBG_TEXT_RESULT "=%d", (result))
#define DBG_TRACE_TASKS_READY()                DBG_PRINT(DBG_TEXT_TASKS_READY)

#else

#define DBG_TRACE_TASK_STARTED(name)
#define DBG_TRACE_POST_INIT_START()
#define DBG_TRACE_POST_INIT_DONE(result)
#define DBG_TRACE_POST_SELF_CHECK_START()
#define DBG_TRACE_POST_SELF_CHECK_DONE(result)
#define DBG_TRACE_TASKS_READY()

#endif
/*************DEBUG END*************/

#endif
