/**
 * @file app_experiment.h
 * @brief Business共享两路观察实验，不参与控制、协议或存储。
 * @details biz_acq唯一采样者，读取只经app_data_api；固定64点，无运行期分配。
 */
#ifndef APP_EXPERIMENT_H
#define APP_EXPERIMENT_H
#include "px4lite_types.h"
#define APP_OBSERVATION_POINTS 64U /**< 两路共享的固定采样槽数。 */
#define APP_OBSERVATION_FIELDS 38U /**< 有界已有测量字段数量。 */
/** @brief 学生手动操作；EDIT必须Stopped，Freeze不停止采样。 */
typedef enum { APP_OBSERVE_EDIT=0,APP_OBSERVE_START,APP_OBSERVE_STOP,APP_OBSERVE_RESET,
 APP_OBSERVE_FREEZE,APP_OBSERVE_RESUME,APP_OBSERVE_SET_REF } App_ObservationOperation_t;
/** @brief 非阻塞单槽请求，字段编号查FieldInfo，间隔编号0/1/2。 */
typedef struct {
 uint8_t operation; /**< ObservationOperation。 */
 uint8_t field[2]; /**< 两路有界字段编号0..37。 */
 uint8_t interval; /**< 0=200/1=500/2=1000 ms。 */
} App_ObservationRequest_t;
/** @brief 一次真实轮询槽；缺测/重复留断点，不补旧缓存。 */
typedef struct {
 uint32_t time_ms; /**< biz_acq实际采样槽时间，ms；源测量时刻另在统计last_ms。 */
 float value[2]; /**< 字段原单位，角度为本段局部展开值。 */
 uint8_t valid_mask; /**< bit0/1代表各路本槽有新鲜新测量。 */
 uint8_t segment_mask; /**< bit0/1代表新段起点，不连接上一点。 */
 uint16_t reserved; /**< 对齐；结构16 B。 */
} App_ObservationPoint_t;
/** @brief 每路有效测量统计；角度缺测后重建段内统计。 */
typedef struct {
 uint32_t count; /**< 会话有效点总数，不计重复。 */
 uint32_t segment_count; /**< 当前统计段有效点数。 */
 uint32_t missing; /**< 缺测或重复槽数。 */
 uint32_t last_ms; /**< 最近接受源测量时间，ms。 */
 float minimum; /**< 当前统计段最小值，单位见FieldInfo。 */
 float maximum; /**< 当前统计段最大值。 */
 float mean; /**< 当前统计段增量均值。 */
} App_ObservationStats_t;
/** @brief 一致性快照；逐点复制前后核对ring_version。 */
typedef struct {
 uint32_t request_id; /**< 递增请求身份。 */
 uint32_t completed_request_id; /**< 已处理请求身份。 */
 uint32_t session_id; /**< Start/Reset代数。 */
 uint32_t ring_version; /**< 每次写点或清环递增。 */
 Px4Lite_Result_t result; /**< 最后动作结果。 */
 App_ObservationStats_t stats[2]; /**< 两路统计。 */
 uint16_t point_count; /**< 当前环内槽数0..64。 */
 uint8_t field[2]; /**< 字段编号。 */
 uint8_t interval; /**< 间隔编号。 */
 uint8_t running; /**< 1采样，0Stopped。 */
 uint8_t frozen; /**< 1仅固定绘图，采样和统计继续。 */
 uint8_t pending; /**< 1请求待处理。 */
 uint8_t reference_valid; /**< 1已Set ref。 */
 uint8_t drift_state; /**< 0无参考/1有效/2定位无效/3超1000 m。 */
 uint8_t reserved[2]; /**< 对齐。 */
 float north_m; /**< 当前局部北向位移，m，drift_state=1时有效。 */
 float east_m; /**< 当前局部东向位移，m。 */
} App_ObservationStatus_t;
/** @brief ROM字段描述，不包含可写参数或通用编辑器。 */
typedef struct { const char *name; /**< 英文名。 */ const char *unit; /**< 物理单位。 */ uint8_t group; /**< 0IMU/1Att/2Baro/3Power/4GNSS/5Motor/6Drift。 */ uint8_t angle; /**< 1度数需局部展开。 */ } App_ObservationFieldInfo_t;
/** @brief 排队动作；OK不代表已执行。 */
Px4Lite_Result_t App_RequestObservation(const App_ObservationRequest_t *request);
/** @brief biz_acq唯一调用，间隔到期采一次，不追赶补点。 */
void App_ObservationWorkRun(uint32_t now_ms);
/** @brief 复制状态，now_ms供统一消费者接口，Age由last_ms计算。 */
Px4Lite_Result_t App_CopyObservation(App_ObservationStatus_t *out,uint32_t now_ms);
/** @brief 按当前会话最旧起始索引复制一个点。 */
Px4Lite_Result_t App_CopyObservationPoint(uint32_t session_id,uint16_t index,App_ObservationPoint_t *out);
/** @brief 查询固定ROM字段。 */
const App_ObservationFieldInfo_t *App_ObservationFieldInfo(uint8_t field);
#endif
