# PC 端单元测试构建脚本（host gcc / mingw-w64）。
#
# 用途：在开发机上编译并运行 Tests/Unit 下的纯逻辑单元测试，不依赖 STM32 硬件。
#   这些测试用桩函数替换 BSP 层，只验证换算、状态机、映射等纯软件逻辑。
#   工程的正式固件构建仍以 Keil MDK-ARM（ARMCC5）为唯一入口，本 Makefile 不产出固件。
#
# 用法（在 Tests 目录下）：
#   make            编译并运行全部测试
#   make clean      清理生成的可执行文件
#
# gcc 来源：mingw-w64（winget 包 BrechtSanders.WinLibs.POSIX.UCRT）。
#   若 gcc 已在 PATH，直接用；否则自动在 winget 安装目录里查找。

# ---- 定位 gcc ----
CC ?= $(shell command -v gcc 2>/dev/null)
ifeq ($(CC),)
CC := $(shell find "$(LOCALAPPDATA)/Microsoft/WinGet" -name gcc.exe 2>/dev/null | head -1)
endif

# ---- 头文件搜索路径 ----
# stubs 必须在最前：用空的 stm32f4xx_hal.h 顶替真实 HAL，使 bsp_config.h 可在 PC 上包含。
INC := -Istubs \
       -I../Core/Inc -I../Debug/Inc -I../Bsp/Inc -I../Sensor/Inc \
       -I../Framework/Inc -I../Business/Inc -I../Display/Inc -I../Storage/Inc

CFLAGS := -std=c11 -Wall -Wextra -O0 -g $(INC)

OUTDIR := build

# ---- 各测试的源文件组合 ----
# test_power：被测实现 sensor_power.c 需要一并编译；BSP 由测试文件内桩替换。
# 另两个测试各自 #include 了被测 .c，或只依赖 static inline 头，无需额外源文件。
POWER_SRC   := Unit/test_power_adc_calibration.c ../Sensor/Src/sensor_power.c
CONTROL_SRC := Unit/test_control_motor_logic.c
IMU_SRC     := Unit/test_platform_imu_axis_mapping.c

TESTS := power control imu

.PHONY: all run clean
all: run

$(OUTDIR):
	@mkdir -p $(OUTDIR)

$(OUTDIR)/test_power.exe: $(POWER_SRC) | $(OUTDIR)
	$(CC) $(CFLAGS) $^ -o $@

$(OUTDIR)/test_control.exe: $(CONTROL_SRC) | $(OUTDIR)
	$(CC) $(CFLAGS) $^ -o $@

$(OUTDIR)/test_imu.exe: $(IMU_SRC) | $(OUTDIR)
	$(CC) $(CFLAGS) $^ -o $@

run: $(OUTDIR)/test_power.exe $(OUTDIR)/test_control.exe $(OUTDIR)/test_imu.exe
	@echo "===== 运行单元测试 ====="
	@fail=0; \
	for t in $(TESTS); do \
	  echo "----- test_$$t -----"; \
	  ./$(OUTDIR)/test_$$t.exe || fail=1; \
	done; \
	if [ $$fail -eq 0 ]; then echo "===== 全部通过 ====="; else echo "===== 存在失败 ====="; exit 1; fi

clean:
	@rm -rf $(OUTDIR)
