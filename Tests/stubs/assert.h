/**
 * @file assert.h
 * @brief 仅供 host 测试使用的断言出口，避免 Windows CRT 弹窗阻塞自动运行。
 * @details 保留表达式检查、失败位置与非零退出码；不进入 Keil 固件工程。
 *          Windows 失败写 stderr 后退出，不调用会触发错误窗口的 CRT abort。
 */
#if !defined(_WIN32)
#include_next <assert.h>
#else

#ifndef HOST_TEST_ASSERT_HELPER_H
#define HOST_TEST_ASSERT_HELPER_H
#include <stdio.h>
#include <stdlib.h>

/**
 * @brief 记录断言失败并终止当前测试进程，由运行器统计为失败。
 * @param[in] expression 失败表达式文本。
 * @param[in] file 源文件路径。
 * @param[in] line 源文件行号，从 1 开始。
 */
static inline void HostTest_AssertFailed(const char *expression, const char *file, unsigned int line)
{
  fprintf(stderr, "Assertion failed: %s, file %s, line %u\n", expression, file, line);
  fflush(stderr);
  exit(EXIT_FAILURE);
}
#endif

/* 与标准 assert 一致，每次包含时根据 NDEBUG 重新定义，表达式仅求值一次。 */
#undef assert
#ifdef NDEBUG
#define assert(expression) ((void)0)
#else
#define assert(expression) ((expression) ? (void)0 : HostTest_AssertFailed(#expression, __FILE__, __LINE__))
#endif

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L) && !defined(__cplusplus)
#undef static_assert
#define static_assert _Static_assert
#endif
#endif
