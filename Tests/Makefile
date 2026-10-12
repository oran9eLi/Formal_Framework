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
.DEFAULT_GOAL := all
CC ?= $(shell command -v gcc 2>/dev/null)
ifeq ($(CC),)
CC := $(shell find "$(LOCALAPPDATA)/Microsoft/WinGet" -name gcc.exe 2>/dev/null | head -1)
endif

# ---- 头文件搜索路径 ----
# stubs 必须在最前：用空的 stm32f4xx_hal.h 顶替真实 HAL，使 bsp_config.h 可在 PC 上包含。
# 同时使用 host assert 出口：Windows 失败写 stderr 并非零退出，不弹出 CRT 错误窗口。
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

TESTS := power control imu display command alarm attitude attitude_ops estimator reference storage rpi_h1_echo rpi_baud_switch pwm_fatal rtc_startup heartbeat_policy watchdog_startup
TESTS += lora_parser rpi_parser
TESTS += display_touch_health
TESTS += attitude_roll_wrap adc_current_sampling adc_current_probe bme280_progress baro_schedule sensor_debug_ownership sensor_debug_off
TESTS += m2_sd_recovery m2_touch_recovery m2_remoteid_fields
TESTS += display_field_validity display_lvgl_invalidation
TESTS += remote_freshness remote_navigation remote_slots gnss_measurement gnss_dma gnss_tx_freshness gnss_time_freshness estimator_gnss_freshness
TESTS += power_current_valid

$(OUTDIR)/test_power_current_valid.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program Unit/test_power_current_valid.c -o $@

$(OUTDIR)/test_remote_freshness.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program Unit/test_remote_freshness.c -o $@

$(OUTDIR)/test_remote_navigation.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program -I../Third_Party/mavlink -Wno-address-of-packed-member -Wno-type-limits Unit/test_remote_navigation.c -o $@

$(OUTDIR)/test_remote_slots.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program Unit/test_remote_slots.c -o $@

$(OUTDIR)/test_gnss_measurement.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program Unit/test_gnss_measurement.c -o $@

$(OUTDIR)/test_gnss_dma.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program Unit/test_gnss_dma.c -o $@

$(OUTDIR)/test_gnss_tx_freshness.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program -I../Third_Party/mavlink -Wno-address-of-packed-member -Wno-type-limits Unit/test_gnss_tx_freshness.c -o $@

$(OUTDIR)/test_gnss_time_freshness.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program Unit/test_gnss_time_freshness.c -o $@

$(OUTDIR)/test_estimator_gnss_freshness.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program Unit/test_estimator_gnss_freshness.c -o $@

$(OUTDIR)/test_display_field_validity.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program Unit/test_display_field_validity.c -o $@

$(OUTDIR)/test_display_lvgl_invalidation.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program -I../Third_Party/LVGL -I../Third_Party/LVGL/lvgl Unit/test_display_lvgl_invalidation.c -o $@

$(OUTDIR)/test_m2_sd_recovery.exe: FORCE | $(OUTDIR)
	$(CC) -Istubs -Im2_comms_stubs $(CFLAGS) -O1 -fwhole-program -I../Third_Party/mavlink -Wno-address-of-packed-member Unit/test_m2_sd_recovery.c -o $@

$(OUTDIR)/test_m2_touch_recovery.exe: FORCE | $(OUTDIR)
	$(CC) -Istubs -Im2_comms_stubs $(CFLAGS) -O1 -fwhole-program -I../Third_Party/mavlink -Wno-address-of-packed-member Unit/test_m2_touch_recovery.c -o $@

$(OUTDIR)/test_m2_remoteid_fields.exe: FORCE | $(OUTDIR)
	$(CC) -Istubs -Im2_comms_stubs $(CFLAGS) -O1 -fwhole-program -I../Third_Party/mavlink -Wno-address-of-packed-member Unit/test_m2_remoteid_fields.c -o $@

$(OUTDIR)/test_attitude_roll_wrap.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) Unit/test_attitude_roll_wrap.c ../Framework/Src/px4lite_attitude.c -o $@

$(OUTDIR)/test_adc_current_sampling.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) Unit/test_adc_current_sampling.c -o $@

$(OUTDIR)/test_adc_current_probe.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -DBSP_ADC_CURRENT_FLOAT_CHECK_ENABLE=1 Unit/test_adc_current_sampling.c -o $@

$(OUTDIR)/test_bme280_progress.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -IUnit/runtime_stubs Unit/test_bme280_progress.c -o $@

$(OUTDIR)/test_baro_schedule.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program Unit/test_baro_schedule.c -o $@

$(OUTDIR)/test_sensor_debug_ownership.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program Unit/test_sensor_debug_ownership.c -o $@

$(OUTDIR)/test_sensor_debug_off.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program -DTEST_POWER_MONITOR=0 Unit/test_sensor_debug_ownership.c -o $@

$(OUTDIR)/test_display_touch_health.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program Unit/test_display_touch_health.c -o $@

$(OUTDIR)/test_lora_parser.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program -I../Third_Party/mavlink -Wno-address-of-packed-member Unit/test_lora_parser.c -o $@

$(OUTDIR)/test_rpi_parser.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program -I../Third_Party/mavlink -Wno-address-of-packed-member -Wno-type-limits Unit/test_rpi_parser.c -o $@

$(OUTDIR)/test_watchdog_startup.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) Unit/test_watchdog_startup.c -o $@

$(OUTDIR)/test_heartbeat_policy.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) Unit/test_heartbeat_policy.c -o $@

$(OUTDIR)/test_rtc_startup.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program Unit/test_rtc_startup.c -o $@

$(OUTDIR)/test_pwm_fatal.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program Unit/test_pwm_fatal.c -o $@

.PHONY: all run clean
all: run

$(OUTDIR):
	@mkdir -p $(OUTDIR)

$(OUTDIR)/test_power.exe: $(POWER_SRC) | $(OUTDIR)
	$(CC) $(CFLAGS) $(POWER_SRC) -o $@

$(OUTDIR)/test_control.exe: $(CONTROL_SRC) | $(OUTDIR)
	$(CC) $(CFLAGS) $(CONTROL_SRC) -o $@

$(OUTDIR)/test_imu.exe: $(IMU_SRC) | $(OUTDIR)
	$(CC) $(CFLAGS) $(IMU_SRC) -o $@

# 测试直接包含实现文件，源码或头文件变化后每次重新构建，避免运行旧二进制。
.PHONY: FORCE
FORCE:

$(OUTDIR)/test_power.exe $(OUTDIR)/test_control.exe $(OUTDIR)/test_imu.exe: FORCE

$(OUTDIR)/test_display.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program Unit/test_display_remote_control.c -o $@

$(OUTDIR)/test_command.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program -I../Third_Party/mavlink -Wno-address-of-packed-member Unit/test_command_motor.c -o $@

$(OUTDIR)/test_alarm.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program -I../Third_Party/mavlink -Wno-address-of-packed-member -Wno-type-limits Unit/test_mavlink_alarm_table.c -o $@

$(OUTDIR)/test_attitude.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) Unit/test_attitude_measurement.c ../Framework/Src/px4lite_attitude.c -o $@

$(OUTDIR)/test_attitude_ops.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) Unit/test_attitude_operations.c ../Framework/Src/px4lite_attitude.c -o $@

$(OUTDIR)/test_estimator.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program Unit/test_estimator_attitude_requests.c -o $@

$(OUTDIR)/test_reference.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program Unit/test_display_attitude_reference.c -o $@

$(OUTDIR)/test_storage.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program Unit/test_storage_freshness.c -o $@

$(OUTDIR)/test_rpi_h1_echo.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program -I../Third_Party/mavlink -Wno-address-of-packed-member Unit/test_rpi_h1_echo.c -o $@

$(OUTDIR)/test_rpi_baud_switch.exe: FORCE | $(OUTDIR)
	$(CC) $(CFLAGS) -O1 -fwhole-program -I../Third_Party/mavlink -Wno-address-of-packed-member Unit/test_rpi_baud_switch.c -o $@

run: $(addprefix $(OUTDIR)/test_,$(addsuffix .exe,$(TESTS)))
	@echo "===== 运行单元测试 ====="
	@fail=0; \
	for t in $(TESTS); do \
	  echo "----- test_$$t -----"; \
	  ./$(OUTDIR)/test_$$t.exe || fail=1; \
	done; \
	if [ $$fail -eq 0 ]; then echo "===== 全部通过 ====="; else echo "===== 存在失败 ====="; exit 1; fi

clean:
	@rm -rf $(OUTDIR)
