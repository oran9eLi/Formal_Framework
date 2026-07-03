/**
 * @file debug_selftest.h
 * @brief 声明 Debug 模块的分项自测入口。
 *
 * @details
 * 自测入口只允许由 DebugTask 调用，用于板级 bring-up 和逐项故障确认。生产业务、显示和
 * 通信路径不得依赖本文件中的接口，关闭 Debug 开关后不得改变系统正式数据流。
 */

#ifndef DEBUG_SELFTEST_H
#define DEBUG_SELFTEST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 执行一次已启用的 Debug 分项自测打印。
 *
 * @param[in] now_ms 当前系统毫秒时间，用于远端快照新鲜度判断。
 *
 * @note 本函数只复制现有快照并打印，不执行外设重新初始化、阻塞协议交互或页面渲染。
 */
void DebugSelfTest_Run(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
