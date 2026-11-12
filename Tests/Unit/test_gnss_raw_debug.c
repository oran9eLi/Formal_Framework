/**
 * @file test_gnss_raw_debug.c
 * @brief 通过真实Adapter与Debug报告器验证原线帧HEX完整性、轮转和任务限速。
 * @details Driver仅提供完整原字节副本；不执行HAL或传感器采集，不伪造定位测量。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <setjmp.h>
#include "px4lite_gnss_diagnostics.h"
#if defined(SENSOR_GNSS_H) || defined(BSP_GNSS_H) || defined(STM32F4xx_HAL_H)
#error Framework raw contract must not expose driver or hardware headers
#endif

typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { int unused; } TIM_HandleTypeDef;
typedef struct { int unused; } DMA_HandleTypeDef;
typedef struct { uint32_t CNT,SR; } HostTimer;
#define UART_WORDLENGTH_8B 0U
#define UART_WORDLENGTH_9B 0x1000U
#define UART_PARITY_NONE 0U
#define UART_PARITY_EVEN 0x0400U
#define UART_PARITY_ODD 0x0200U
#define UART_STOPBITS_1 0U
#define UART_STOPBITS_2 0x2000U
#define UART_HWCONTROL_NONE 0U
#define UART_HWCONTROL_RTS 0x0100U
#define UART_HWCONTROL_CTS 0x0200U
#define UART_HWCONTROL_RTS_CTS 0x0300U
#define pdPASS 1
#define pdFAIL 0
#define tskIDLE_PRIORITY 0U
#define pdMS_TO_TICKS(ms) (ms)
static HostTimer s_host_timer;
static uint32_t s_clock;
#define TIM6 (&s_host_timer)
#define TIM_SR_UIF 1U
static uint32_t __get_PRIMASK(void) { return 0U; }
static void __disable_irq(void) { }
static void __enable_irq(void) { }
static uint32_t HAL_GetTick(void) { return s_clock; }
static uint32_t HAL_GetUIDw0(void) { return 0U; }
static uint32_t HAL_GetUIDw1(void) { return 0U; }
static uint32_t HAL_GetUIDw2(void) { return 0U; }
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *last,TickType_t ticks);
BaseType_t xTaskCreate(void (*fn)(void *),const char *name,uint16_t words,void *arg,UBaseType_t priority,TaskHandle_t *out);

#include "px4lite_config.h"
#if defined(TEST_GNSS_DISABLED) && TEST_GNSS_DISABLED
#undef PX4LITE_ENABLE_GNSS
#define PX4LITE_ENABLE_GNSS 0U
#endif
#include "debug_config.h"
/* GNSS专用夹具隔离三传感器/水位/电源报告；各自行为由对应独立测试覆盖。 */
#undef DEBUG_STACK_MONITOR_ENABLE
#define DEBUG_STACK_MONITOR_ENABLE 0U
#undef DEBUG_IMU_MONITOR_ENABLE
#define DEBUG_IMU_MONITOR_ENABLE 0U
#undef DEBUG_BARO_MONITOR_ENABLE
#define DEBUG_BARO_MONITOR_ENABLE 0U
#undef DEBUG_POWER_MONITOR_ENABLE
#define DEBUG_POWER_MONITOR_ENABLE 0U
#if defined(TEST_RAW_MONITOR_DISABLED) && TEST_RAW_MONITOR_DISABLED
#undef DEBUG_GNSS_MONITOR_ENABLE
#define DEBUG_GNSS_MONITOR_ENABLE 0U
#endif
#include "../../Framework/Src/px4lite_platform_f407.c"
#include "../../Debug/Src/debug_service.c"

static Gnss_RawFrameInfo_t s_info[3];
static uint8_t s_wire[3][392],s_ready[3];
static uint8_t *s_expected_destination;
static unsigned s_driver_calls,s_lines,s_task_creations;
static char s_output[128][256];
static jmp_buf s_task_exit;
static Gnss_Diagnostics_t s_link_driver_source;
static BSP_GNSS_TransportInfo_t s_link_transport_source;
static Gnss_Result_t s_link_driver_result=GNSS_RESULT_NO_DATA;
static BSP_Status_t s_link_transport_result=BSP_STATUS_ERROR;
static unsigned s_link_driver_copies,s_link_transport_copies;
static uint8_t s_link_follow_clock;

/** @brief 独立注入sensor已经发布的诊断，不执行Service或读取任何新硬件。 */
Gnss_Result_t Sensor_GNSS_CopyDiagnostics(Gnss_Diagnostics_t *out)
{ s_link_driver_copies++;*out=s_link_driver_source;return s_link_driver_result; }
/** @brief 提供BSP原始累计副本，与Driver事实独立；不推进DMA或消费接收队列。 */
BSP_Status_t BSP_GNSS_CopyTransportInfo(BSP_GNSS_TransportInfo_t *out)
{ s_link_transport_copies++;*out=s_link_transport_source;return s_link_transport_result; }

/** @brief Driver原帧副本替身；确认Adapter直接使用调用者目标，无大栈中转。 */
Gnss_Result_t Sensor_GNSS_CopyRawFrame(uint8_t slot,uint8_t *bytes,uint16_t cap,Gnss_RawFrameInfo_t *info)
{
  s_driver_calls++;assert(slot<3U && cap==392U && bytes==s_expected_destination);
  if(!s_ready[slot]){return GNSS_RESULT_NO_DATA;}
  memcpy(bytes,s_wire[slot],s_info[slot].length);*info=s_info[slot];return GNSS_RESULT_OK;
}
uint32_t BSP_Time_GetTickMs(void){return s_clock;}
/** @brief 无测量topic，仍须能输出原帧。 */
Px4Lite_Result_t Px4Lite_CopyGnss(Px4Lite_SensorGnss_t *out){(void)out;return PX4LITE_NOT_READY;}
Px4Lite_Result_t Px4Lite_GetModuleStatus(Px4Lite_ModuleId_t id,Px4Lite_ModuleStatus_t *out)
{(void)id;(void)out;return PX4LITE_NOT_READY;}
Px4Lite_Result_t Px4Lite_CopyTime(Px4Lite_TimeSnapshot_t *out){(void)out;return PX4LITE_NOT_READY;}
/** @brief 捕获每次DBG_PRINT的完整文本，模拟独立行写入。 */
void DebugConsole_Printf(const char *prefix,const char *fmt,...)
{
  va_list args;int count;assert(s_lines<128U);assert(strcmp(prefix,"[DBG] ")==0);
  va_start(args,fmt);count=vsnprintf(s_output[s_lines],sizeof(s_output[0]),fmt,args);va_end(args);
  assert(count>=0 && (unsigned)count<sizeof(s_output[0]));s_lines++;
}
TickType_t xTaskGetTickCount(void){return s_clock;}
/** @brief 运行真实100ms循环至2200ms，1100ms提供一个新owner样本。 */
void vTaskDelayUntil(TickType_t *last,TickType_t ticks)
{
  assert(ticks==100U);*last+=ticks;s_clock+=ticks;
  if(s_link_follow_clock) {
    s_link_driver_source.measured_ms=s_clock;s_link_driver_source.service_count++;
    s_link_driver_source.last_rx_ms=s_clock;s_link_driver_source.last_frame_ms=s_clock;
    s_link_driver_source.rx_bytes++;s_link_driver_source.valid_ubx++;s_link_transport_source.rx_bytes++;
  }
  if(s_clock==1100U){s_info[0].sequence++;}
  if(s_clock>2200U){longjmp(s_task_exit,1);}
}
BaseType_t xTaskCreate(void (*fn)(void *),const char *name,uint16_t words,void *arg,UBaseType_t priority,TaskHandle_t *out)
{(void)fn;(void)arg;assert(!strcmp(name,"debug") && words==384U && priority==1U);s_task_creations++;*out=(void *)1;return pdPASS;}
BaseType_t DebugTaskMonitor_Register(TaskHandle_t handle,const char *name,uint16_t words)
{assert(handle==(void *)1 && !strcmp(name,"debug") && words==384U);return pdPASS;}

/** @brief 重置测试源和Debug独占游标，不能视为生产清统计接口。 */
static void Reset(void)
{
  memset(s_info,0,sizeof(s_info));memset(s_ready,0,sizeof(s_ready));memset(s_wire,0,sizeof(s_wire));
  s_driver_calls=0U;s_lines=0U;s_clock=0U;s_task_creations=0U;s_expected_destination=NULL;
  memset(&s_link_driver_source,0,sizeof(s_link_driver_source));memset(&s_link_transport_source,0,sizeof(s_link_transport_source));
  s_link_driver_result=GNSS_RESULT_NO_DATA;s_link_transport_result=BSP_STATUS_ERROR;
  s_link_driver_copies=0U;s_link_transport_copies=0U;
  s_link_follow_clock=0U;
#if DEBUG_GNSS_MONITOR_ENABLE
  memset(s_gnss_raw_seen,0,sizeof(s_gnss_raw_seen));memset(s_gnss_raw_last_sequence,0,sizeof(s_gnss_raw_last_sequence));s_gnss_raw_next_slot=0U;
  memset(&s_gnss_link_diag,0,sizeof(s_gnss_link_diag));memset(&s_gnss_link_gate,0,sizeof(s_gnss_link_gate));
#endif
}
/** @brief 生成包含任意0/B5/62/换行等值的原字节，不重新计算或修正校验。 */
static void Fill(uint8_t slot,uint16_t length,uint32_t sequence,uint8_t valid)
{
  uint16_t i;s_ready[slot]=1U;s_info[slot].length=length;s_info[slot].sequence=sequence;
  s_info[slot].received_ms=UINT32_MAX;s_info[slot].protocol=2U;s_info[slot].class_id=1U;
  s_info[slot].message_id=slot==0U?7U:slot==1U?4U:0x99U;s_info[slot].checksum_valid=valid;
  for(i=0U;i<length;i++){s_wire[slot][i]=(uint8_t)(i*19U+slot);}
}
/** @brief Adapter逐字段映射、0序号、错误参数及禁用行为。 */
static void TestAdapter(void)
{
  static Px4Lite_GnssRawFrame_t out;
  Reset();assert(Px4Lite_GnssCopyRawFrame(0U,NULL)==PX4LITE_INVALID_PARAM);
  assert(Px4Lite_GnssCopyRawFrame(3U,&out)==PX4LITE_INVALID_PARAM && !s_driver_calls);
  s_expected_destination=out.bytes;Fill(2U,392U,0U,0U);
#if PX4LITE_ENABLE_GNSS
  assert(Px4Lite_GnssCopyRawFrame(0U,&out)==PX4LITE_NOT_READY);
  assert(Px4Lite_GnssCopyRawFrame(2U,&out)==PX4LITE_OK);
  assert(out.sequence==0U && out.received_ms==UINT32_MAX && out.length==392U);
  assert(out.protocol==2U && out.class_id==1U && out.message_id==0x99U && !out.checksum_valid);
  assert(!memcmp(out.bytes,s_wire[2],392U));
#else
  assert(Px4Lite_GnssCopyRawFrame(2U,&out)==PX4LITE_NOT_READY && !s_driver_calls);
#endif
}

#if DEBUG_GNSS_MONITOR_ENABLE && PX4LITE_ENABLE_GNSS
/** @brief 一次原帧输出必须逐字节完整，最多32字节/行且每行含同一序号和准确偏移。 */
static void ExpectHex(uint8_t slot,const char *name)
{
  unsigned line,total=0U,headers=0U;
  for(line=0U;line<s_lines;line++){
    if(strstr(s_output[line],"GNSS-RAW sampled ")){headers++;assert(strstr(s_output[line],name));assert(strstr(s_output[line],s_info[slot].checksum_valid?"CRC=OK":"CRC=BAD"));}
    if(!strncmp(s_output[line],"RAW seq=",8U)){
      unsigned long seq;unsigned offset;int at=0;unsigned count=0U;
      assert(sscanf(s_output[line],"RAW seq=%lu +%u %n",&seq,&offset,&at)==2);
      assert(seq==s_info[slot].sequence && offset==total);
      assert(strlen(s_output[line])+strlen("[DBG] ")<128U);
      while(s_output[line][at]){
        unsigned byte;int consumed=0;
        assert(sscanf(s_output[line]+at,"%2x%n",&byte,&consumed)==1 && consumed==2);
        assert(total<s_info[slot].length && byte==s_wire[slot][total]);total++;count++;at+=consumed;
        if(s_output[line][at]){assert(s_output[line][at]==' ');at++;}
      }
      assert(count>0U && count<=32U);
    }
  }
  assert(headers==1U && total==s_info[slot].length);
}
/** @brief 三槽轮转、不重复、0和回绕、badCRC及最大392字节。 */
static void TestRoundRobin(void)
{
  Reset();Fill(0U,100U,0U,1U);Fill(1U,26U,11U,1U);Fill(2U,392U,UINT32_MAX,0U);
  s_expected_destination=s_gnss_raw_frame.bytes;
  DebugService_ReportGnssRaw();ExpectHex(0U,"NAV-PVT");s_lines=0U;
  DebugService_ReportGnssRaw();ExpectHex(1U,"NAV-DOP");s_lines=0U;
  DebugService_ReportGnssRaw();ExpectHex(2U,"OTHER");s_lines=0U;
  DebugService_ReportGnssRaw();assert(!s_lines);
  s_info[2].sequence=0U;DebugService_ReportGnssRaw();ExpectHex(2U,"OTHER");s_lines=0U;
  DebugService_ReportGnssRaw();assert(!s_lines);
  s_info[0].sequence=1U;s_info[1].sequence=12U;
  DebugService_ReportGnssRaw();ExpectHex(0U,"NAV-PVT");s_lines=0U;
  DebugService_ReportGnssRaw();ExpectHex(1U,"NAV-DOP");
}
/** @brief 无定位topic也输出真实原帧，真实100ms任务循环每1s至多一个新原帧。 */
static void TestTaskRateAndNoMeasurement(void)
{
  unsigned i;volatile unsigned frames=0U;Reset();Fill(0U,100U,0U,1U);s_expected_destination=s_gnss_raw_frame.bytes;
  if(setjmp(s_task_exit)==0){DebugService_Task(NULL);}
  for(i=0U;i<s_lines;i++){if(strstr(s_output[i],"GNSS-RAW sampled ")){frames++;}}
  assert(frames==2U && s_clock==2300U);
}
#endif

/** @brief 同一文件覆盖monitor关闭和GNSS禁用构建，不要求新增任务。 */
int main(void)
{
  TestAdapter();Reset();assert(DebugService_CreateTask()==pdPASS);
#if DEBUG_PERIODIC_SERVICE_ENABLE
  assert(s_task_creations==1U);
#else
  assert(s_task_creations==0U);
#endif
#if DEBUG_GNSS_MONITOR_ENABLE && PX4LITE_ENABLE_GNSS
  TestRoundRobin();TestTaskRateAndNoMeasurement();
#endif
  puts("GNSS raw Adapter/Debug tests passed");return 0;
}
