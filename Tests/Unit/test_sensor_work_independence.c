/**
 * @file test_sensor_work_independence.c
 * @brief 真实sensor工作入口的三传感器失败隔离、静态暂存清零与发布副本寿命。
 * @details 外设Adapter注入结果；被测调度/发布分支使用生产实现，不模拟实际连接。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef int BaseType_t;
typedef void *TaskHandle_t;
#include "../../Framework/Src/px4lite_modules.c"

static uint32_t s_now;
static Px4Lite_Result_t s_results[3];
static uint8_t s_order[3],s_reads,s_published[3],s_partial;
static Px4Lite_SensorGnss_t s_copy_gnss;
static Px4Lite_SensorImu_t s_copy_imu;
static Px4Lite_SensorBaro_t s_copy_baro;

/** @brief 独立真实毫秒源，每次测试步跨越三模块正常采集许可。 */
uint32_t Px4Lite_PlatformGetMs(void) { return s_now; }
/** @brief 统计只读微秒源不引入额外硬件调用。 */
uint32_t Px4Lite_PlatformGetUs(void) { return s_now*1000U; }
/** @brief 隔离运行统计出口，不影响实际采集分支。 */
void Px4Lite_RuntimeReportSensorWork(uint32_t elapsed) { (void)elapsed; }
/** @brief 记录所有者固定读取顺序，单个失败不能跳过其它模块。 */
static void Read(unsigned id) { assert(s_reads<3U);s_order[s_reads++]=(uint8_t)id; }
/** @brief 只填当前来源实际给出的域，检出静态暂存残留。 */
Px4Lite_Result_t Px4Lite_GnssRead(Px4Lite_SensorGnss_t *out)
{
  Read(0U);out->last_rx_ms=s_now;out->status_update_ms=s_now;out->valid_mask=PX4LITE_GNSS_VALID_NAV_STATUS;
  if(!s_partial) { out->altitude_mm=12345; }
  return s_results[0];
}
/** @brief 注入新IMU或失败；部分帧不填角速度以检查清零。 */
Px4Lite_Result_t Px4Lite_ImuRead(Px4Lite_SensorImu_t *out)
{ Read(1U);out->header.sample_time_ms=s_now;if(!s_partial) { out->gyro_mdps[0]=12500; }return s_results[1]; }
/** @brief 气压服务仍获得原周期许可，不由GNSS结果控制。 */
Px4Lite_Result_t Px4Lite_BaroRead(Px4Lite_SensorBaro_t *out,uint8_t permit)
{ Read(2U);assert(permit);out->header.sample_time_ms=s_now;if(!s_partial) { out->pressure_pa=101325U; }return s_results[2]; }
/** @brief 电源路径未产生新帧，不属于本次三模块输入。 */
Px4Lite_Result_t Px4Lite_BatteryRead(Px4Lite_BatteryStatus_t *out) { (void)out;return PX4LITE_IDLE; }
/** @brief 第二组电源保留原调用，其结果不影响三模块读取。 */
Px4Lite_Result_t Px4Lite_Battery2Read(Px4Lite_BatteryStatus_t *out) { (void)out;return PX4LITE_IDLE; }
/** @brief 出口使用真实topic契约的同步值复制，返回后不能借用scratch指针。 */
void Px4Lite_PublishGnss(const Px4Lite_SensorGnss_t *out)
{ assert(out->last_rx_ms==s_now);s_copy_gnss=*out;s_published[0]++; }
/** @brief FIFO契约同样在返回前复制整份样本。 */
Px4Lite_Result_t Px4Lite_PushImu(const Px4Lite_SensorImu_t *out)
{ assert(out->header.sample_time_ms==s_now);s_copy_imu=*out;s_published[1]++;return PX4LITE_OK; }
/** @brief 气压topic复制，不保留本轮暂存地址。 */
void Px4Lite_PublishBaro(const Px4Lite_SensorBaro_t *out)
{ assert(out->header.sample_time_ms==s_now);s_copy_baro=*out;s_published[2]++; }
/** @brief 禁止把无新帧电源发布成成功样本。 */
void Px4Lite_PublishBattery(const Px4Lite_BatteryStatus_t *out) { (void)out;assert(0); }
/** @brief 禁止把无新帧第二组电源发布成成功样本。 */
void Px4Lite_PublishBattery2(const Px4Lite_BatteryStatus_t *out) { (void)out;assert(0); }

/** @brief 每轮重置观察器；不重置被测静态scratch，覆盖连续复用。 */
static void Step(unsigned mask)
{
  unsigned id;
  s_now+=1000U;s_reads=0U;
  for(id=0U;id<3U;id++) { s_results[id]=(mask&(1U<<id))?PX4LITE_IO_ERROR:PX4LITE_OK;s_published[id]=0U; }
  Px4Lite_SensorWorkRun(s_now);
  assert(s_reads==3U && s_order[0]==0U && s_order[1]==1U && s_order[2]==2U);
  for(id=0U;id<3U;id++) { assert(s_published[id]==((mask&(1U<<id))?0U:1U)); }
}
/** @brief 所有I/O组合都读取三模块，失败不会修改此前已经发布的副本。 */
int main(void)
{
  unsigned mask;
  s_now=100U;assert(Px4Lite_ModulesInit()==PX4LITE_OK);
  for(mask=0U;mask<8U;mask++) {
    Px4Lite_SensorGnss_t gnss;
    Px4Lite_SensorImu_t imu;
    Px4Lite_SensorBaro_t baro;
    s_partial=0U;Step(0U);gnss=s_copy_gnss;imu=s_copy_imu;baro=s_copy_baro;
    assert(gnss.altitude_mm==12345 && imu.gyro_mdps[0]==12500 && baro.pressure_pa==101325U);
    s_partial=1U;Step(mask);
    if(mask&1U) { assert(memcmp(&gnss,&s_copy_gnss,sizeof(gnss))==0); }
    else { assert(s_copy_gnss.altitude_mm==0 && s_copy_gnss.last_rx_ms==s_now); }
    if(mask&2U) { assert(memcmp(&imu,&s_copy_imu,sizeof(imu))==0); }
    else { assert(s_copy_imu.gyro_mdps[0]==0 && s_copy_imu.header.sample_time_ms==s_now); }
    if(mask&4U) { assert(memcmp(&baro,&s_copy_baro,sizeof(baro))==0); }
    else { assert(s_copy_baro.pressure_pa==0U && s_copy_baro.header.sample_time_ms==s_now); }
  }
  puts("Sensor work: all8 failure combinations / independent reads / scratch clear / published copy lifetime PASS");
  return 0;
}
