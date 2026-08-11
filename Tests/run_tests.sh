#!/usr/bin/env bash
# PC 端单元测试运行器（Git Bash + mingw-w64 gcc，无需 make）。
#
# 用途：在开发机上编译并运行 Tests/Unit 下的纯逻辑单元测试，不依赖 STM32 硬件。
#   BSP 层由测试文件内的桩函数替换，只验证换算、状态机、映射等纯软件逻辑。
#   正式固件构建仍以 Keil MDK-ARM（ARMCC5）为唯一入口，本脚本不产出固件。
#
# 用法（在 Tests 目录下）：  bash run_tests.sh
#
# gcc 来源：mingw-w64（winget 包 BrechtSanders.WinLibs.POSIX.UCRT）。
#   优先用 PATH 中的 gcc；找不到则在 winget 安装目录里自动查找。
set -u
cd "$(dirname "$0")"

# ---- 定位 gcc ----
CC="$(command -v gcc 2>/dev/null || true)"
if [ -z "$CC" ]; then
  CC="$(find "$LOCALAPPDATA/Microsoft/WinGet" -name gcc.exe 2>/dev/null | head -1)"
fi
if [ -z "$CC" ]; then
  echo "找不到 gcc。请先安装 mingw-w64：winget install --id BrechtSanders.WinLibs.POSIX.UCRT -e" >&2
  exit 2
fi
echo "使用编译器：$("$CC" --version | head -1)"

# ---- 头文件搜索路径（stubs 必须在最前，用空 HAL 顶替真实 HAL）----
INC="-Istubs -I../Core/Inc -I../Debug/Inc -I../Bsp/Inc -I../Sensor/Inc \
     -I../Framework/Inc -I../Business/Inc -I../Display/Inc -I../Storage/Inc"
CFLAGS="-std=c11 -Wall -Wextra -O0 -g"

OUT="build"
mkdir -p "$OUT"

# ---- 测试名 -> 源文件组合 ----
# test_power 需一并编译被测实现 sensor_power.c；另两个各自 #include 了被测 .c 或只用 static inline 头。
build_and_run() {
  local name="$1"; shift
  echo "----- $name -----"
  if ! "$CC" $CFLAGS $INC "$@" -o "$OUT/$name.exe" 2>"$OUT/$name.build.log"; then
    echo "  [编译失败] 见 $OUT/$name.build.log"; cat "$OUT/$name.build.log"; return 1
  fi
  if "./$OUT/$name.exe"; then return 0; else echo "  [运行失败] $name"; return 1; fi
}

fail=0
build_and_run test_power   Unit/test_power_adc_calibration.c ../Sensor/Src/sensor_power.c || fail=1
build_and_run test_control Unit/test_control_motor_logic.c || fail=1
build_and_run test_imu     Unit/test_platform_imu_axis_mapping.c || fail=1

echo "======================================"
if [ "$fail" -eq 0 ]; then echo "全部通过"; else echo "存在失败"; fi
exit "$fail"
