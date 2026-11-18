/**
 * @file display_hud_present.h
 * @brief HUD完整后备帧的RGB565无损编码与集中提交。
 * @details Display层，只允许显示任务调用；不读取传感器、不动态分配。
 * 普通Flush保持旧路径；显式Begin/Capture/Commit批次才使用离屏缓存。
 */
#ifndef DISPLAY_HUD_PRESENT_H
#define DISPLAY_HUD_PRESENT_H
#include <stdint.h>

/** @brief 兼容原回退开关；0不分配后备帧，Begin失败且普通Flush不变。 */
#ifndef DISPLAY_HUD_DIFF_ENABLE
#define DISPLAY_HUD_DIFF_ENABLE 1U
#endif

/** @brief 条带捕获结果；ERROR表示整帧撤销，不允许部分提交。 */
typedef enum {
  DISPLAY_HUD_CAPTURE_ERROR = 0, /**< 参数、顺序、容量或LCD状态错误。 */
  DISPLAY_HUD_CAPTURE_MORE = 1,  /**< 已离屏保存，仍需后续完整行。 */
  DISPLAY_HUD_CAPTURE_READY = 2  /**< 258行完整，可CommitFrame。 */
} Display_HudCaptureResult_t;

/** @brief 最近后备帧状态，只描述显示提交事实，不表示传感器状态。 */
typedef enum {
  DISPLAY_HUD_FRAME_IDLE = 0,      /**< 未开始帧。 */
  DISPLAY_HUD_FRAME_BUILDING = 1,  /**< 正在捕获。 */
  DISPLAY_HUD_FRAME_READY = 2,     /**< 完整待提交。 */
  DISPLAY_HUD_FRAME_COMMITTED = 3, /**< 整帧同步提交成功。 */
  DISPLAY_HUD_FRAME_ABORTED = 4,   /**< 上层或身份变化撤销。 */
  DISPLAY_HUD_FRAME_OVERFLOW = 5,  /**< 编码超过24KB，整帧撤销。 */
  DISPLAY_HUD_FRAME_ERROR = 6      /**< 参数、顺序、流或LCD状态错误。 */
} Display_HudFrameState_t;

/** @brief 显示任务独占的有界帧统计；计数为自启动累计次数。 */
typedef struct {
  uint32_t started_frames;        /**< Begin成功次数，帧。 */
  uint32_t captured_frames;       /**< 完整258行离屏捕获次数，帧。 */
  uint32_t committed_frames;      /**< 集中提交成功次数，帧。 */
  uint32_t aborted_frames;        /**< 已开始帧撤销次数，帧。 */
  uint32_t overflow_frames;       /**< 编码容量不足次数，帧。 */
  uint32_t error_frames;          /**< 参数/顺序/LCD/编码异常次数，帧。 */
  uint32_t last_encoded_bytes;    /**< 最近帧成功或撤销前编码量，B，0..24576。 */
  uint32_t maximum_encoded_bytes; /**< 编码峰值，B，0..24576。 */
  uint16_t captured_rows;         /**< 当前完整捕获行数，0..258；撤销/提交后0。 */
  uint8_t state;                  /**< 最近状态，Display_HudFrameState_t。 */
} Display_HudPresentStats_t;

/**
 * @brief 配置HUD身份；身份变化撤销在途帧，不写屏。
 * @param active 1启用身份，0禁用。
 * @param valid 1有效配色、0灰盘；仅作身份，不变换颜色。
 * @param x HUD左上角X，px。
 * @param y HUD左上角Y，px。
 * @param width HUD宽，px；当前仅258有效。
 * @param height HUD高，px；当前仅258有效。
 */
void Display_HudPresent_Configure(uint8_t active,uint8_t valid,uint16_t x,uint16_t y,
                                uint16_t width,uint16_t height);

/** @brief 撤销帧并禁用配置身份，保留累计统计；不触碰LVGL前40KB。 */
void Display_HudPresent_Reset(void);

/** @brief 一次调用原SSD1963批量提交，不参与后备帧，不保留输入指针。 */
void Display_HudPresent_Flush(uint16_t x,uint16_t y,uint16_t width,uint16_t height,
                            const uint16_t *pixels);

/**
 * @brief 开始258x258后备帧，成功1、失败0；会先撤销已有帧。
 * @details 仅验证尺寸/屏内边界/LCD就绪；不依赖Configure已启用。
 * 起始Capture必须从首行开始；本函数不写GRAM。
 */
uint8_t Display_HudPresent_BeginFrame(uint16_t x,uint16_t y,uint16_t width,uint16_t height);

/** @brief 捕获中或完整待提交返回1，其他状态返回0。 */
uint8_t Display_HudPresent_IsBuilding(void);

/**
 * @brief 无损保存连续完整扫描行，期间绝不写GRAM。
 * @param x 必须等于Begin区域X，px。
 * @param y 必须等于下一待捕获行屏幕Y，px。
 * @param width 必须258px，每行完整覆盖HUD矩形。
 * @param height 条带完整行数，1..剩余行数，允许16行条带。
 * @param pixels 至少width*height项RGB565，返回前已复制，不跨调用保留。
 * @return MORE/READY成功；ERROR撤销整帧，不能部分提交。
 */
Display_HudCaptureResult_t Display_HudPresent_Capture(uint16_t x,uint16_t y,uint16_t width,
                                                    uint16_t height,const uint16_t *pixels);

/**
 * @brief 完整预检后集中解码同步提交258行，成功1、失败0。
 * @details 不等待RTOS节拍、不关中断；静态单行scratch仅显示任务访问。
 * 未完成/撤销/坏流在写屏前拒绝，提交中LCD失效则停止并作废。
 */
uint8_t Display_HudPresent_CommitFrame(void);

/** @brief 撤销在途帧，不写屏；重复中止无重复累计副作用。 */
void Display_HudPresent_AbortFrame(void);

/** @brief 同任务复制私有统计；dest为空无操作，不新增UI或打印。 */
void Display_HudPresent_GetStats(Display_HudPresentStats_t *dest);
#endif
