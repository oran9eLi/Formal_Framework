/**
 * @file app_experiment.c
 * @brief Business非消费型两路采样、统计和局部GNSS漂移。
 * @details 唯一写者biz_acq；App API读取生产副本，不写控制或Navigation。
 */
#include "app_experiment.h"
#include "app_data_api.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>
static App_ObservationPoint_t s_points[APP_OBSERVATION_POINTS];
static App_ObservationStatus_t s_state={0U,0U,1U,0U,PX4LITE_OK,{{0}},0U,{0U,1U},0U,0U,0U,0U,0U,0U,{0U,0U},0.0f,0.0f};
static App_ObservationRequest_t s_request;
static App_ObservationStatus_t s_public;
/** @brief 源判新与局部角度状态，均仅采样任务使用。 */
typedef struct { uint32_t time_ms; /**< 上次源ms。 */uint32_t seq; /**< 上次源序号。 */float angle; /**< 上次展开角，deg。 */uint8_t seen; /**< 曾接受测量。 */uint8_t gap; /**< 前槽缺测。 */uint8_t reserved[2]; /**< 对齐。 */ } ObservationSource_t;
static ObservationSource_t s_source[2];
static uint32_t s_due_ms,s_reference_ms;
static int32_t s_ref_lat,s_ref_lon;
static uint16_t s_head;
static const uint16_t s_intervals[3]={200U,500U,1000U};
static const App_ObservationFieldInfo_t s_fields[APP_OBSERVATION_FIELDS]={
 {"Accel X","mg",0,0},{"Accel Y","mg",0,0},{"Accel Z","mg",0,0},
 {"Gyro X","deg/s",0,0},{"Gyro Y","deg/s",0,0},{"Gyro Z","deg/s",0,0},{"IMU temp","C",0,0},
 {"Roll","deg",1,1},{"Pitch","deg",1,1},{"Yaw","deg",1,1},{"Roll rate","deg/s",1,0},{"Pitch rate","deg/s",1,0},{"Yaw rate","deg/s",1,0},
 {"Pressure","Pa",2,0},{"Baro temp","C",2,0},
 {"Power1 voltage","V",3,0},{"Power1 current","A",3,0},{"Power1 power","W",3,0},
 {"Power2 voltage","V",3,0},{"Power2 current","A",3,0},{"Power2 power","W",3,0},
 {"GNSS altitude","m",4,0},{"Ground speed","m/s",4,0},{"Ground course","deg",4,1},
 {"M1 target","%",5,0},{"M1 output","%",5,0},{"M1 pulse","us",5,0},
 {"M2 target","%",5,0},{"M2 output","%",5,0},{"M2 pulse","us",5,0},
 {"M3 target","%",5,0},{"M3 output","%",5,0},{"M3 pulse","us",5,0},
 {"M4 target","%",5,0},{"M4 output","%",5,0},{"M4 pulse","us",5,0},
 {"Drift North","m",6,0},{"Drift East","m",6,0}
};
/** @brief 有界字段元数据。 */
const App_ObservationFieldInfo_t *App_ObservationFieldInfo(uint8_t field) { return field<APP_OBSERVATION_FIELDS?&s_fields[field]:0; }
/** @brief 清结果并换代数，不改变配置或正在处理的请求身份。 */
static void ObservationReset(void)
{
 s_state.session_id++;if(!s_state.session_id) { s_state.session_id=1U; }s_state.ring_version++;
 s_state.point_count=0U;s_head=0U;s_state.frozen=0U;
 memset(s_state.stats,0,sizeof(s_state.stats));memset(s_source,0,sizeof(s_source));
}
/** @brief 请求只复制；固定槽满拒绝，不覆盖未执行动作。 */
Px4Lite_Result_t App_RequestObservation(const App_ObservationRequest_t *r)
{
 if(!r||r->operation>APP_OBSERVE_SET_REF||r->field[0]>=APP_OBSERVATION_FIELDS||r->field[1]>=APP_OBSERVATION_FIELDS||r->interval>2U) { return PX4LITE_INVALID_PARAM; }
 taskENTER_CRITICAL();if(s_state.pending) { taskEXIT_CRITICAL();return PX4LITE_BUSY; }
 s_request=*r;s_state.request_id++;s_state.pending=1U;s_public.request_id=s_state.request_id;s_public.pending=1U;taskEXIT_CRITICAL();return PX4LITE_OK;
}
/** @brief 新鲜定位的局部平面近似，经度跨界归一化；无效不保留旧有效值。 */
static void ObservationDrift(uint32_t now)
{
 App_GnssDetails_t g;int64_t dlon;double north,east;
 s_state.drift_state=0U;
 if(!s_state.reference_valid) { return; }
 (void)App_CopyGnssDetails(&g,now);
 if(!(g.valid_mask&PX4LITE_GNSS_VALID_POSITION)) { s_state.drift_state=2U;return; }
 dlon=(int64_t)g.longitude_e7-s_ref_lon;
 if(dlon>1800000000LL) { dlon-=3600000000LL; }else if(dlon< -1800000000LL) { dlon+=3600000000LL; }
 north=((double)g.latitude_e7-s_ref_lat)*0.011131949079;
 east=(double)dlon*0.011131949079*cos((double)s_ref_lat*0.000000001745329252);
 if(north*north+east*east>1000000.0) { s_state.drift_state=3U;return; }
 s_state.north_m=(float)north;s_state.east_m=(float)east;s_state.drift_state=1U;s_reference_ms=g.position_update_ms;
}
/** @brief 从App读取单字段及真正测量身份；零值可有效，过期/无效返回0。 */
static uint8_t ObservationSample(uint8_t f,uint32_t now,float *v,uint32_t *ms,uint32_t *seq)
{
 union { App_ImuSnapshot_t imu;App_NavigationSnapshot_t nav;App_EnvironmentSnapshot_t env;App_GnssDetails_t gnss;App_MotorSnapshot_t motor; } data;
 *seq=0U;
 if(f<7U) { if(App_CopyImuLatest(&data.imu,now)!=PX4LITE_OK) { return 0U; }*ms=data.imu.header.sample_time_ms;*seq=data.imu.header.sequence;
  *v=f<3U?(float)data.imu.accel_mg[f]:(f<6U?(float)data.imu.gyro_mdps[f-3U]/1000.0f:(float)data.imu.temperature_cdeg/100.0f);return 1U; }
 if(f<13U) { (void)App_CopyNavigation(&data.nav,now);if(!(data.nav.valid_mask&PX4LITE_NAV_VALID_ATTITUDE)||(uint32_t)(now-data.nav.attitude_measurement_ms)>200U) { return 0U; }*ms=data.nav.attitude_measurement_ms;
  switch(f) {case 7:*v=data.nav.roll_deg100/100.0f;break;case 8:*v=data.nav.pitch_deg100/100.0f;break;case 9:*v=data.nav.yaw_deg100/100.0f;break;case 10:*v=data.nav.roll_rate_dps100/100.0f;break;case 11:*v=data.nav.pitch_rate_dps100/100.0f;break;default:*v=data.nav.yaw_rate_dps100/100.0f;break;}return 1U; }
 if(f<21U) { uint32_t bits; (void)App_CopyEnvironment(&data.env,now);
  if(f<15U) { bits=APP_ENV_VALID_PRESSURE_TEMPERATURE;*ms=data.env.environment_measurement_ms;*v=f==13U?data.env.pressure_pa:data.env.temperature_c; }
  else { uint8_t p=(uint8_t)(f>=18U);uint8_t kind=(uint8_t)((f-15U)%3U);float voltage=(float)(p?data.env.voltage2_mv:data.env.voltage_mv)/1000.0f;float current=(float)(p?data.env.current2_ma:data.env.current_ma)/1000.0f;
   bits=kind==0U?(p?APP_ENV_VALID_VOLTAGE2:APP_ENV_VALID_VOLTAGE1):(kind==1U?(p?APP_ENV_VALID_CURRENT2:APP_ENV_VALID_CURRENT1):(p?(APP_ENV_VALID_VOLTAGE2|APP_ENV_VALID_CURRENT2):(APP_ENV_VALID_VOLTAGE1|APP_ENV_VALID_CURRENT1)));
   *ms=p?data.env.power2_measurement_ms:data.env.power1_measurement_ms;*v=kind==0U?voltage:(kind==1U?current:voltage*current); }
  return (uint8_t)((data.env.valid_mask&bits)==bits); }
 if(f<24U) { uint32_t bit=f==21U?PX4LITE_GNSS_VALID_ALTITUDE:PX4LITE_GNSS_VALID_VELOCITY;(void)App_CopyGnssDetails(&data.gnss,now);
  *ms=f==21U?data.gnss.altitude_update_ms:data.gnss.velocity_update_ms;*v=f==21U?data.gnss.altitude_mm/1000.0f:(f==22U?data.gnss.ground_speed_cms/100.0f:data.gnss.heading_deg100/100.0f);return (uint8_t)((data.gnss.valid_mask&bit)!=0U); }
 if(f<36U) { uint8_t index=(uint8_t)((f-24U)/3U),kind=(uint8_t)((f-24U)%3U);if(App_CopyMotor(&data.motor,now)!=PX4LITE_OK) { return 0U; }*ms=data.motor.header.sample_time_ms;*seq=data.motor.header.sequence;*v=kind==0U?data.motor.base_percent[index]:(kind==1U?data.motor.duty_percent[index]:data.motor.pulse_us[index]);return 1U; }
 *ms=s_reference_ms;*v=f==36U?s_state.north_m:s_state.east_m;return (uint8_t)(s_state.drift_state==1U);
}
/** @brief 唯一采样任务处理动作，读取/计算放临界区外，发布只做小结构复制。 */
void App_ObservationWorkRun(uint32_t now)
{
 App_ObservationRequest_t r;uint8_t pending;
 taskENTER_CRITICAL();pending=s_state.pending;r=s_request;taskEXIT_CRITICAL();
 /* 本函数为唯一写者；读者复制时禁止任务切换，写者每次执行不会与读者并行。 */
 if(pending) {
  Px4Lite_Result_t result=PX4LITE_OK;
  switch(r.operation) {
   case APP_OBSERVE_EDIT:if(s_state.running) { result=PX4LITE_BUSY; }else { s_state.field[0]=r.field[0];s_state.field[1]=r.field[1];s_state.interval=r.interval;ObservationReset(); }break;
   case APP_OBSERVE_START:if(s_state.running) { result=PX4LITE_BUSY; }else { ObservationReset();s_state.running=1U;s_due_ms=now; }break;
   case APP_OBSERVE_STOP:s_state.running=0U;break;
   case APP_OBSERVE_RESET:if(s_state.running) { result=PX4LITE_BUSY; }else { ObservationReset();s_state.reference_valid=0U;s_state.drift_state=0U; }break;
   case APP_OBSERVE_FREEZE:s_state.frozen=1U;break;
   case APP_OBSERVE_RESUME:s_state.frozen=0U;break;
   case APP_OBSERVE_SET_REF:{ App_GnssDetails_t g;(void)App_CopyGnssDetails(&g,now);if(g.valid_mask&PX4LITE_GNSS_VALID_POSITION) { s_ref_lat=g.latitude_e7;s_ref_lon=g.longitude_e7;s_state.reference_valid=1U;memset(s_source,0,sizeof(s_source));memset(s_state.stats,0,sizeof(s_state.stats)); }else { result=PX4LITE_NOT_READY; } }break;
   default:result=PX4LITE_INVALID_PARAM;break;
  }
  taskENTER_CRITICAL();s_state.result=result;s_state.completed_request_id=s_state.request_id;s_state.pending=0U;taskEXIT_CRITICAL();
 }
 ObservationDrift(now);
 if(s_state.running && (int32_t)(now-s_due_ms)>=0) {
  App_ObservationPoint_t point;uint8_t i;
  memset(&point,0,sizeof(point));point.time_ms=now;
  for(i=0U;i<2U;i++) {
   float value=0.0f;uint32_t ms=0U,seq=0U;ObservationSource_t *src=&s_source[i];App_ObservationStats_t *st=&s_state.stats[i];
   uint8_t valid=ObservationSample(s_state.field[i],now,&value,&ms,&seq);
   if(valid && (!src->seen||src->time_ms!=ms||src->seq!=seq) && value==value && value<3.4e38f && value> -3.4e38f) {
    uint8_t angle=s_fields[s_state.field[i]].angle;
    if(!src->seen||src->gap) { point.segment_mask|=(uint8_t)(1U<<i);if(angle) { st->segment_count=0U; } }
    else if(angle) { float delta=value-src->angle;while(delta>180.0f) { delta-=360.0f; }while(delta< -180.0f) { delta+=360.0f; }value=src->angle+delta; }
    src->time_ms=ms;src->seq=seq;src->seen=1U;src->gap=0U;src->angle=value;
    st->count++;st->last_ms=ms;
    if(!st->segment_count) { st->minimum=value;st->maximum=value;st->mean=value; }else { if(value<st->minimum) { st->minimum=value; }if(value>st->maximum) { st->maximum=value; }st->mean+=(value-st->mean)/(float)(st->segment_count+1U); }
    st->segment_count++;point.value[i]=value;point.valid_mask|=(uint8_t)(1U<<i);
   }else { st->missing++;src->gap=1U; }
  }
  taskENTER_CRITICAL();s_points[s_head]=point;s_head=(uint16_t)((s_head+1U)%APP_OBSERVATION_POINTS);if(s_state.point_count<APP_OBSERVATION_POINTS) { s_state.point_count++; }s_state.ring_version++;taskEXIT_CRITICAL();
  /* 500ms请求在200ms调度上交替600/400；保存真实时间，不虚构准时点。 */
  s_due_ms+=s_intervals[s_state.interval];if((int32_t)(now-s_due_ms)>=0) { s_due_ms=now+s_intervals[s_state.interval]; }
 }
 taskENTER_CRITICAL();s_public=s_state;taskEXIT_CRITICAL();
}
/** @brief 固定状态副本，后续发布使用单写者完整快照。 */
Px4Lite_Result_t App_CopyObservation(App_ObservationStatus_t *out,uint32_t now)
{ (void)now;if(!out) { return PX4LITE_INVALID_PARAM; }taskENTER_CRITICAL();*out=s_public;taskEXIT_CRITICAL();return PX4LITE_OK; }
/** @brief 单点复制；调用者检查前后ring_version以避免移动环混帧。 */
Px4Lite_Result_t App_CopyObservationPoint(uint32_t session,uint16_t index,App_ObservationPoint_t *out)
{
 uint16_t slot;if(!out) { return PX4LITE_INVALID_PARAM; }taskENTER_CRITICAL();
 if(session!=s_state.session_id||index>=s_state.point_count) { taskEXIT_CRITICAL();return PX4LITE_NOT_READY; }
 slot=(uint16_t)((s_head+APP_OBSERVATION_POINTS-s_state.point_count+index)%APP_OBSERVATION_POINTS);*out=s_points[slot];taskEXIT_CRITICAL();return PX4LITE_OK;
}
