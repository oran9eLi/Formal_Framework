# 本机(host)编译运行 Tests/Unit

这些测试是普通 x86 可执行程序（`int main()` + `printf`），跟 Keil/ARM 固件编译完全分开，
用 MinGW-w64 GCC 在本机直接编译运行，不需要连板子。

## 环境

已经通过 `winget install BrechtSanders.WinLibs.POSIX.UCRT` 装好，`gcc` 在：
```
%LOCALAPPDATA%\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin
```
把这个目录加进 PATH 就能直接用 `gcc`。

## 编译一个测试

每个 `test_*.c` 都是自包含的：只 `#include` 需要的 Framework/Business 头文件，并且
自己给不需要真实实现的外部函数写桩(stub)。所以编译时只需要把该测试**实际调用到**的
那几个真实 `.c` 源文件一起传给 gcc，其余全部由测试文件自己的桩函数满足链接。

Framework 里少数几个文件（`px4lite_local_msglog.c`、`px4lite_remote_telemetry.c`）
会 `#include "FreeRTOS.h"`，本机没有真实 RTOS，所以要把 `Tests/Stub` 加在
`-I Framework/Inc` **前面**，用这里的空壳头文件顶替，把临界区宏变成空操作。

示例（在仓库根目录执行）：

```bash
gcc -std=c99 -w \
  -I Tests/Stub -I Framework/Inc -I Business/Inc -I Third_Party/mavlink \
  Tests/Unit/test_local_msglog.c \
  Framework/Src/px4lite_local_msglog.c \
  -o test_local_msglog.exe
./test_local_msglog.exe
```

涉及 MAVLink 收发的测试（`test_mavlink_rx_*`、`test_mavlink_tx_*`）通常还需要链接：
`Framework/Src/px4lite_mavlink_rx.c` 或 `px4lite_mavlink_tx.c`、
`Framework/Src/px4lite_remote_telemetry.c`、`Framework/Src/px4lite_identity.c`。

涉及 Display/Business 数据模型的测试（`test_app_display_*`、`test_app_message_log.c`）
通常还需要：`Business/Src/app_data_api.c`（体量较大，依赖面广，测试文件里已经把它
用到的所有 Framework Copy* 函数都桩好了）、`Framework/Src/px4lite_local_msglog.c`、
`Framework/Src/px4lite_remote_telemetry.c`、`Framework/Src/px4lite_identity.c`，
`test_app_message_log.c` 额外还要 `Business/Src/app_message_log.c`。

`-w` 是为了屏蔽 Third_Party/mavlink 生成代码里大量无害的
`-Waddress-of-packed-member` 警告，不影响正确性判断。

## 没有统一的一键跑全部脚本

这个仓库目前没有 CMakeLists/Makefile 把 Tests/Unit 串起来跑；每个测试的确切链接文件
需要看它 `#include` 了哪些头、调用了哪些函数来决定，无法无脑把所有 `Framework/Src/*.c`
一起扔给编译器（会撞硬件相关文件的编译错误，以及重复定义 stub 冲突）。逐个编译运行是
目前唯一可靠的方式。
