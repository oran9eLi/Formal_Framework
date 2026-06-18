/**
 * @file debug_console.c
 * @brief Implement the mutex-protected USART1 debug console.
 */

#include "debug_console.h"

#if DEBUG_CONSOLE_ENABLE

#include "bsp_uart.h"
#include "debug_trace.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <stdarg.h>
#include <stdio.h>

#define DEBUG_BANNER        "========================================"
#define DEBUG_BANNER_TITLE  " STM32F407 " DBG_TEXT_LOW_ALTITUDE " CNS " DBG_TEXT_DEBUG_MODE
#define DEBUG_STDOUT_BUFFER_SIZE 256U
#define DEBUG_LINE_BUFFER_SIZE 256U

extern uint32_t SystemCoreClock;

static SemaphoreHandle_t s_debug_console_mutex;
static char s_stdout_buffer[DEBUG_STDOUT_BUFFER_SIZE];
static uint16_t s_stdout_length = 0U;

/**
 * @brief Acquire the debug console mutex when the scheduler is running.
 */
static void DebugConsole_Lock(void)
{
  if ((s_debug_console_mutex != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)) {
    (void)xSemaphoreTake(s_debug_console_mutex, portMAX_DELAY);
  }
}

/**
 * @brief Release the debug console mutex when it was acquired.
 */
static void DebugConsole_Unlock(void)
{
  if ((s_debug_console_mutex != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)) {
    (void)xSemaphoreGive(s_debug_console_mutex);
  }
}

/**
 * @brief Send one byte sequence through the debug UART.
 */
static void DebugConsole_Write(const char *data, uint16_t length)
{
  if ((data == NULL) || (length == 0U)) {
    return;
  }

  (void)BSP_UART_Send((const uint8_t *)data, length, 1000U);
}

/**
 * @brief Serialize and send one byte sequence through the debug UART.
 */
static void DebugConsole_WriteLocked(const char *data, uint16_t length)
{
  DebugConsole_Lock();
  DebugConsole_Write(data, length);
  DebugConsole_Unlock();
}

/**
 * @brief Flush buffered standard output bytes to the debug UART.
 */
static void DebugConsole_FlushStdout(void)
{
  if (s_stdout_length == 0U) {
    return;
  }

  DebugConsole_Write(s_stdout_buffer, s_stdout_length);
  s_stdout_length = 0U;
}

/**
 * @brief Append one character to the standard output buffer and flush when needed.
 */
static void DebugConsole_BufferStdout(const char *data, uint16_t length)
{
  uint16_t index;

  if ((data == NULL) || (length == 0U)) {
    return;
  }

  for (index = 0U; index < length; index++) {
    if (s_stdout_length >= DEBUG_STDOUT_BUFFER_SIZE) {
      DebugConsole_FlushStdout();
    }

    s_stdout_buffer[s_stdout_length] = data[index];
    s_stdout_length++;

    if ((data[index] == '\n') || (s_stdout_length >= DEBUG_STDOUT_BUFFER_SIZE)) {
      DebugConsole_FlushStdout();
    }
  }
}

/**
 * @brief Initialize the debug UART and console synchronization resources.
 */
void DebugConsole_Init(void)
{
  BSP_Status_t uart_status;
  static const char raw_boot[] = "\r\n[RAW] " DBG_TEXT_UART_READY "\r\n";

  s_debug_console_mutex = xSemaphoreCreateMutex();
  uart_status = BSP_UART_Init();
  DebugConsole_Write(raw_boot, (uint16_t)(sizeof(raw_boot) - 1U));

  printf("\r\n%s\r\n", DEBUG_BANNER);
  printf("%s\r\n", DEBUG_BANNER_TITLE);
  printf(" " DBG_TEXT_UART_INIT_STATUS ": %d\r\n", (int)uart_status);
  printf(" " DBG_TEXT_SYSTEM_CLOCK ": %lu MHz\r\n", SystemCoreClock / 1000000U);
  printf("%s\r\n", DEBUG_BANNER);
}

/**
 * @brief Format and print a prefixed debug line.
 */
void DebugConsole_Printf(const char *prefix, const char *fmt, ...)
{
  char line[DEBUG_LINE_BUFFER_SIZE];
  int len;
  va_list args;

  len = snprintf(line, sizeof(line), "%s", (prefix != NULL) ? prefix : "");
  if ((len < 0) || ((size_t)len >= sizeof(line))) {
    return;
  }

  va_start(args, fmt);
  len += vsnprintf(&line[len], sizeof(line) - (size_t)len, fmt, args);
  va_end(args);

  if ((len < 0) || ((size_t)len >= (sizeof(line) - 2U))) {
    return;
  }

  line[len++] = '\r';
  line[len++] = '\n';
  DebugConsole_WriteLocked(line, (uint16_t)len);
}

/**
 * @brief Format and print a task-scoped diagnostic line.
 */
void DebugConsole_TaskInfo(const char *name, const char *fmt, ...)
{
  char line[DEBUG_LINE_BUFFER_SIZE];
  int len;
  va_list args;

  len = snprintf(line, sizeof(line), "[TASK %-8s] ", (name != NULL) ? name : "");
  if ((len < 0) || ((size_t)len >= sizeof(line))) {
    return;
  }

  va_start(args, fmt);
  len += vsnprintf(&line[len], sizeof(line) - (size_t)len, fmt, args);
  va_end(args);

  if ((len < 0) || ((size_t)len >= (sizeof(line) - 2U))) {
    return;
  }

  line[len++] = '\r';
  line[len++] = '\n';
  DebugConsole_WriteLocked(line, (uint16_t)len);
}

/**
 * @brief Print a byte buffer as hexadecimal diagnostic output.
 */
void DebugConsole_HexDump(const char *prefix, const uint8_t *data, uint16_t len)
{
  DebugConsole_Lock();
  printf("[%s] ", prefix);
  for (uint16_t i = 0; i < len; i++) {
    printf("%02X ", data[i]);
  }
  printf("\r\n");
  DebugConsole_Unlock();
}

/**
 * @brief Retarget one standard library output character to the debug console.
 */
int fputc(int ch, FILE *stream)
{
  char byte = (char)ch;

  (void)stream;

  DebugConsole_BufferStdout(&byte, 1U);

  return ch;
}

/**
 * @brief Retarget a standard library byte write to the debug console.
 */
int _write(int fd, const char *ptr, int len)
{
  (void)fd;
  if (len > 0) {
    DebugConsole_BufferStdout(ptr, (uint16_t)len);
  }
  return len;
}

#endif
