# 在 Windows 原生 PowerShell 中构建/运行 host 逻辑测试；不生成 STM32 固件。
param([string[]]$Names = @(), [string]$LogDirectory = 'build/host', [string]$Compiler = '')
$ErrorActionPreference = 'Stop'
$taskOriginalPath = $env:PATH
Push-Location $PSScriptRoot
try {
    if ([string]::IsNullOrWhiteSpace($Compiler)) {
        $taskCompilerCommand = Get-Command gcc -ErrorAction SilentlyContinue
        $taskCompilerCandidates = @()
        if ($taskCompilerCommand) { $taskCompilerCandidates += $taskCompilerCommand.Source }
        $taskCompilerCandidates += @('C:/msys64/mingw64/bin/gcc.exe', 'D:/mingw64/bin/gcc.exe')
        $Compiler = $taskCompilerCandidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
    }
    if ([string]::IsNullOrWhiteSpace($Compiler)) { throw 'Host GCC not found; supply -Compiler with its absolute path.' }
    $compiler = (Resolve-Path -LiteralPath $Compiler).Path
    # GCC 子程序依赖同目录 DLL；只调整本次测试进程，结束时恢复。
    $env:PATH = (Split-Path -Parent $compiler) + ';' + $taskOriginalPath
    # stubs 优先：host assert 失败写 stderr 并非零退出，不触发 Windows CRT 错误窗口。
    $includes = @('-I../Core/Inc', '-I../Debug/Inc', '-I../Bsp/Inc', '-I../Sensor/Inc', '-I../Framework/Inc', '-I../Business/Inc', '-I../Display/Inc', '-I../Storage/Inc')
    $common = @('-std=c11', '-Wall', '-Wextra', '-O0', '-g', '-Istubs')
    $whole = @('-O1', '-fwhole-program')
    $mav = $whole + @('-I../Third_Party/mavlink', '-Wno-address-of-packed-member')
    # 专项GPS/供电编码夹具隔离BMI；默认gnss_sat_stream保留5路真实共存验证。
    $mavNoBmi = $mav + @('-DPX4LITE_ENABLE_RPI_BMI088_UPLINK=0')
    $cases = [ordered]@{
        power = @('Unit/test_power_adc_calibration.c', '../Sensor/Src/sensor_power.c')
        startup_rgb = @('-Werror', 'Unit/test_startup_rgb.c', '../Framework/Src/px4lite_startup_tone.c')
        startup_rgb_off = @('-Werror', '-DPX4LITE_ENABLE_STARTUP_RGB=0', 'Unit/test_startup_rgb.c', '../Framework/Src/px4lite_startup_tone.c')
        startup_tone_off = @('-Werror', '-DPX4LITE_ENABLE_STARTUP_TONE=0', 'Unit/test_startup_rgb.c', '../Framework/Src/px4lite_startup_tone.c')
        rpi_feedback = @('-std=c99', '-Werror', 'Unit/test_rpi_feedback.c')
        rpi_feedback_off = @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_RPI_COMMAND_FEEDBACK=0', 'Unit/test_rpi_feedback.c')
        rpi_feedback_rgb_off = @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_STARTUP_RGB=0', 'Unit/test_rpi_feedback.c')
        rpi_feedback_tone_off = @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_STARTUP_TONE=0', 'Unit/test_rpi_feedback.c')
        rpi_feedback_tone_rgb_off = @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_STARTUP_TONE=0', '-DPX4LITE_ENABLE_STARTUP_RGB=0', 'Unit/test_rpi_feedback.c')
        rpi_feedback_all_off = @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_STARTUP_TONE=0', '-DPX4LITE_ENABLE_RPI_COMMAND_FEEDBACK=0', 'Unit/test_rpi_feedback.c')
        rpi_feedback_rx = $mav + @('-std=c99', '-Werror', '-Wno-type-limits', 'Unit/test_rpi_feedback_rx.c')
        rpi_feedback_rx_off = $mav + @('-std=c99', '-Werror', '-Wno-type-limits', '-DPX4LITE_ENABLE_RPI_COMMAND_FEEDBACK=0', 'Unit/test_rpi_feedback_rx.c')
        rpi_feedback_rx_rpioff = $mav + @('-std=c99', '-Werror', '-Wno-type-limits', '-DTEST_RPI_DISABLED=1', 'Unit/test_rpi_feedback_rx.c')
        rgb_platform = $mav + @('-Werror', '-Wno-type-limits', 'Unit/test_rgb_platform.c')
        control = @('-DPX4LITE_CONTROL_POWER_GATE_ENABLE=1', 'Unit/test_control_motor_logic.c', '../Framework/Src/px4lite_local_msglog.c')
        imu = @('Unit/test_platform_imu_axis_mapping.c')
        bmi088_spi_bus = @('-Werror', 'Unit/test_bmi088_spi_bus.c')
        bmi088_driver = @('-Werror', 'Unit/test_bmi088_driver.c')
        bmi088_uplink = $mav + @('-Werror', 'Unit/test_bmi088_uplink.c')
        bmi088_uplink_off = $mavNoBmi + @('-Werror', 'Unit/test_bmi088_uplink.c')
        bmi088_uplink_mpu = $mav + @('-Werror', '-DPX4LITE_IMU_DRIVER=0', 'Unit/test_bmi088_uplink.c')
        bmi088_platform = $mav + @('-Werror', '-Wno-type-limits', 'Unit/test_bmi088_platform.c')
        bmi088_platform_spi = $mav + @('-Werror', '-Wno-type-limits', '-DPX4LITE_IMU_BMI088_BUS=2', 'Unit/test_bmi088_platform.c')
        bmi088_platform_uplink_off = $mavNoBmi + @('-Werror', '-Wno-type-limits', 'Unit/test_bmi088_platform.c')
        mpu6050_platform = $mav + @('-Werror', '-Wno-type-limits', '-DPX4LITE_IMU_DRIVER=0', 'Unit/test_bmi088_platform.c')
        bmi088_axes = @('-Werror', 'Unit/test_bmi088_axis_mapping.c')
        bmi088_axes_rotated = @('-Werror', '-DTEST_BMI088_ROTATED=1', '-DPX4LITE_BMI088_AXIS_X=2', '-DPX4LITE_BMI088_AXIS_Y=-1', '-DPX4LITE_BMI088_AXIS_Z=3', 'Unit/test_bmi088_axis_mapping.c')
        display = $whole + @('Unit/test_display_remote_control.c')
        hud_refresh = $whole + @('-Werror', '-I../Third_Party/LVGL', 'Unit/test_hud_refresh.c')
        hud_present = @('-Werror', '-I../Third_Party/LVGL', 'Unit/test_hud_present.c')
        hud_present_off = @('-Werror', '-I../Third_Party/LVGL', '-DDISPLAY_HUD_DIFF_ENABLE=0', 'Unit/test_hud_present.c')
        hud_frame_port = $whole + @('-Werror', '-I../Third_Party/LVGL', 'Unit/test_hud_frame_port.c')
        command = $mav + @('-DPX4LITE_CONTROL_POWER_GATE_ENABLE=1', 'Unit/test_command_motor.c')
        motor_manual_lock = $mav + @('-Werror', 'Unit/test_motor_manual_lock.c')
        motor_manual_lock_gate = $mav + @('-Werror', '-DPX4LITE_CONTROL_POWER_GATE_ENABLE=1', 'Unit/test_motor_manual_lock.c')
        display_motor_lock = $whole + @('-Werror', '-I../Third_Party/LVGL', 'Unit/test_display_motor_lock.c')
        alarm = $mav + @('-Wno-type-limits', 'Unit/test_mavlink_alarm_table.c')
        attitude = @('Unit/test_attitude_measurement.c', '../Framework/Src/px4lite_attitude.c')
        attitude_ops = @('Unit/test_attitude_operations.c', '../Framework/Src/px4lite_attitude.c')
        estimator = $whole + @('Unit/test_estimator_attitude_requests.c')
        reference = $whole + @('Unit/test_display_attitude_reference.c')
        storage = $whole + @('-DPX4LITE_ENABLE_BATTERY1=1', 'Unit/test_storage_freshness.c')
        single_supply_adc = @('Unit/test_single_supply_adc.c')
        power2_conversion_audit = @('-Werror', 'Unit/test_power2_conversion_audit.c')
        power_adc_reference = @('-Werror', 'Unit/test_power_adc_reference.c')
        power_diagnostics_mapping = $mav + @('-Werror', '-Wno-type-limits', 'Unit/test_power_diagnostics_mapping.c')
        power_diagnostics_mapping_off = $mav + @('-Werror', '-Wno-type-limits', '-DTEST_BATTERY2_DISABLED=1', 'Unit/test_power_diagnostics_mapping.c')
        single_supply_csv = $whole + @('Unit/test_single_supply_csv.c')
        single_supply_service = $whole + @('Unit/test_single_supply_service.c')
        single_supply_service_dual = $whole + @('-DPX4LITE_ENABLE_BATTERY1=1', 'Unit/test_single_supply_service.c')
        single_supply_service_only1 = $whole + @('-DPX4LITE_ENABLE_BATTERY1=1', '-DPX4LITE_ENABLE_BATTERY2=0', 'Unit/test_single_supply_service.c')
        single_supply_service_alloff = $whole + @('-DPX4LITE_ENABLE_BATTERY1=0', '-DPX4LITE_ENABLE_BATTERY2=0', 'Unit/test_single_supply_service.c')
        single_supply_mavlink = $mavNoBmi + @('Unit/test_single_supply_mavlink.c')
        single_supply_mavlink_dual = $mavNoBmi + @('-DPX4LITE_ENABLE_BATTERY1=1', 'Unit/test_single_supply_mavlink.c')
        single_supply_mavlink_only1 = $mavNoBmi + @('-DPX4LITE_ENABLE_BATTERY1=1', '-DPX4LITE_ENABLE_BATTERY2=0', 'Unit/test_single_supply_mavlink.c')
        single_supply_mavlink_alloff = $mavNoBmi + @('-DPX4LITE_ENABLE_BATTERY1=0', '-DPX4LITE_ENABLE_BATTERY2=0', 'Unit/test_single_supply_mavlink.c')
        rpi_h1_echo = $mav + @('Unit/test_rpi_h1_echo.c')
        rpi_baud_switch = $mav + @('Unit/test_rpi_baud_switch.c')
        pwm_fatal = $whole + @('Unit/test_pwm_fatal.c')
        rtc_startup = $whole + @('Unit/test_rtc_startup.c')
        heartbeat_policy = @('Unit/test_heartbeat_policy.c')
        watchdog_startup = @('Unit/test_watchdog_startup.c')
        lora_parser = $mav + @('Unit/test_lora_parser.c')
        rpi_parser = $mav + @('-Wno-type-limits', 'Unit/test_rpi_parser.c')
        display_touch_health = $whole + @('Unit/test_display_touch_health.c')
        attitude_roll_wrap = @('Unit/test_attitude_roll_wrap.c', '../Framework/Src/px4lite_attitude.c')
        adc_current_sampling = @('-DBSP_ENABLE_POWER1=1', 'Unit/test_adc_current_sampling.c')
        adc_current_probe = @('-DBSP_ENABLE_POWER1=1', '-DBSP_ADC_CURRENT_FLOAT_CHECK_ENABLE=1', 'Unit/test_adc_current_sampling.c')
        bme280_progress = @('-IUnit/runtime_stubs', 'Unit/test_bme280_progress.c')
        baro_schedule = $whole + @('Unit/test_baro_schedule.c')
        sensor_debug_ownership = $whole + @('Unit/test_sensor_debug_ownership.c')
        sensor_work_independence = $whole + @('-Werror', 'Unit/test_sensor_work_independence.c')
        three_sensor_debug = $whole + @('-Werror', 'Unit/test_three_sensor_debug.c')
        three_sensor_debug_off = $whole + @('-Werror', '-DTEST_THREE_SENSOR_DIAG_DISABLED=1', 'Unit/test_three_sensor_debug.c')
        sensor_debug_off = $whole + @('-DTEST_POWER_MONITOR=0', 'Unit/test_sensor_debug_ownership.c')
        sensor_debug_dual = $whole + @('-DPX4LITE_ENABLE_BATTERY1=1', 'Unit/test_sensor_debug_ownership.c')
        m2_sd_recovery = $mav + @('-Im2_comms_stubs', 'Unit/test_m2_sd_recovery.c')
        m2_touch_recovery = $mav + @('-Im2_comms_stubs', 'Unit/test_m2_touch_recovery.c')
        m2_remoteid_fields = $mav + @('-Im2_comms_stubs', 'Unit/test_m2_remoteid_fields.c')
        display_field_validity = $whole + @('Unit/test_display_field_validity.c')
        display_lvgl_invalidation = $whole + @('-I../Third_Party/LVGL', '-I../Third_Party/LVGL/lvgl', 'Unit/test_display_lvgl_invalidation.c')
        remote_freshness = $whole + @('Unit/test_remote_freshness.c')
        remote_navigation = $mav + @('-Wno-type-limits', 'Unit/test_remote_navigation.c')
        remote_slots = $whole + @('Unit/test_remote_slots.c')
        gnss_measurement = $whole + @('Unit/test_gnss_measurement.c')
        ubx_codec = @('-Werror', 'Unit/test_ubx_codec.c', '../Sensor/Src/sensor_ubx.c')
        gnss_bringup = @('-Werror', 'Unit/test_gnss_bringup.c')
        gnss_frame_timeout = @('-Werror', 'Unit/test_gnss_frame_timeout.c')
        gnss_tx_isolation = @('-Werror', 'Unit/test_gnss_tx_isolation.c')
        gnss_dropout_integration = $mav + @('-Werror', '-Wno-type-limits', 'Unit/test_gnss_dropout_integration.c')
        gnss_hardware_reconnect = @('-Werror', 'Unit/test_gnss_hardware_reconnect.c')
        gnss_rx_health = $whole + @('-Werror', 'Unit/test_gnss_rx_health.c')
        gnss_silence_recovery = @('-Werror', 'Unit/test_gnss_silence_recovery.c')
        gnss_satellites = @('-Werror', 'Unit/test_gnss_satellites.c')
        gnss_sat_stream = $mav + @('-Werror', 'Unit/test_gnss_sat_stream.c')
        gnss_satellite_mapping = $mav + @('-Werror', '-Wno-type-limits', 'Unit/test_gnss_satellite_mapping.c')
        gnss_satellite_mapping_off = $mav + @('-Werror', '-Wno-type-limits', '-DTEST_GNSS_DISABLED=1', 'Unit/test_gnss_satellite_mapping.c')
        p25_gnss_transport = @('-Werror', 'Unit/test_p25_gnss_transport.c')
        gnss_diagnostics_mapping = $mav + @('-Werror', '-Wno-type-limits', 'Unit/test_gnss_diagnostics_mapping.c')
        gnss_diagnostics_mapping_off = $mav + @('-Werror', '-Wno-type-limits', '-DTEST_GNSS_DISABLED=1', 'Unit/test_gnss_diagnostics_mapping.c')
        gnss_clean_display = $whole + @('-Werror', '-I../Third_Party/LVGL', 'Unit/test_gnss_diagnostics_display.c')
        gnss_status_domain = $mav + @('-Werror', '-Wno-type-limits', 'Unit/test_gnss_status_domain.c')
        gnss_status_domain_off = $mav + @('-Werror', '-Wno-type-limits', '-DTEST_GNSS_DISABLED=1', 'Unit/test_gnss_status_domain.c')
        display_gnss_status_utc = $whole + @('-Werror', 'Unit/test_display_gnss_status_utc.c')
        gnss_raw_capture = @('-Werror', 'Unit/test_gnss_raw_capture.c')
        gnss_protocol_capture = $whole + @('-Werror', 'Unit/test_gnss_protocol_capture.c')
        gnss_unified_facts = $whole + @('-Werror', 'Unit/test_gnss_unified_facts.c')
        gnss_unified_adapter = $mav + @('-Werror', '-Wno-type-limits', '-DTEST_GNSS_UNIFIED_ADAPTER=1', 'Unit/test_gnss_unified_facts.c')
        gnss_unified_uplink = $mavNoBmi + @('-Werror', '-DPX4LITE_RPI_GNSS_PARSED_TX_ENABLE=1', '-DPX4LITE_ENABLE_RPI_GNSS_UPLINK=1', '-DPX4LITE_ENABLE_RPI_GNSS_RAW_UPLINK=0', 'Unit/test_gnss_unified_uplink.c')
        gnss_raw_uplink = $mavNoBmi + @('-Werror', 'Unit/test_gnss_raw_uplink.c')
        gnss_raw_uplink_compat = $mavNoBmi + @('-Werror', '-DPX4LITE_RPI_GNSS_PARSED_TX_ENABLE=1', '-DPX4LITE_ENABLE_RPI_GNSS_UPLINK=1', '-DPX4LITE_RPI_GNSS_RAW_STATS_ENABLE=1', 'Unit/test_gnss_raw_uplink.c')
        gnss_raw_only_policy_guard = $mavNoBmi + @('-Werror', '-DPX4LITE_RPI_GNSS_PARSED_TX_ENABLE=0', '-DPX4LITE_ENABLE_RPI_GNSS_UPLINK=1', 'Unit/test_gnss_raw_uplink.c')
        gnss_raw_uplink_off = $mavNoBmi + @('-Werror', '-DPX4LITE_ENABLE_RPI_GNSS_RAW_UPLINK=0', 'Unit/test_gnss_raw_uplink.c')
        gnss_raw_forward_hooks = $mav + @('-Werror', '-Wno-type-limits', 'Unit/test_gnss_raw_forward_hooks.c')
        gnss_raw_debug = $mav + @('-Werror', '-Wno-type-limits', 'Unit/test_gnss_raw_debug.c')
        gnss_raw_debug_off = $mav + @('-Werror', '-Wno-type-limits', '-DTEST_RAW_MONITOR_DISABLED=1', 'Unit/test_gnss_raw_debug.c')
        gnss_raw_debug_gnss_off = $mav + @('-Werror', '-Wno-type-limits', '-DTEST_GNSS_DISABLED=1', 'Unit/test_gnss_raw_debug.c')
        gnss_link_debug = $mav + @('-Werror', '-Wno-type-limits', 'Unit/test_gnss_link_debug.c')
        gnss_link_debug_off = $mav + @('-Werror', '-Wno-type-limits', '-DTEST_RAW_MONITOR_DISABLED=1', 'Unit/test_gnss_link_debug.c')
        gnss_link_debug_gnss_off = $mav + @('-Werror', '-Wno-type-limits', '-DTEST_GNSS_DISABLED=1', 'Unit/test_gnss_link_debug.c')
        gnss_dma = $whole + @('Unit/test_gnss_dma.c')
        gnss_tx_freshness = $mavNoBmi + @('-Wno-type-limits', 'Unit/test_gnss_tx_freshness.c')
        gnss_time_freshness = $whole + @('Unit/test_gnss_time_freshness.c')
        estimator_gnss_freshness = $whole + @('Unit/test_estimator_gnss_freshness.c')
        power_current_valid = $whole + @('Unit/test_power_current_valid.c')
    }
    if ($Names.Count -eq 0) { $Names = @($cases.Keys) }
    New-Item -ItemType Directory -Force -Path $LogDirectory | Out-Null
    $failed = 0
    foreach ($name in $Names) {
        if (!$cases.Contains($name)) { throw "Unknown test: $name" }
        $binary = Join-Path $LogDirectory "test_$name.exe"
        $buildLog = Join-Path $LogDirectory "$name.build.log"
        $runLog = Join-Path $LogDirectory "$name.run.log"
        & $compiler @common @($cases[$name]) @includes -o $binary *> $buildLog
        if ($LASTEXITCODE -ne 0) { Get-Content $buildLog; $failed++; continue }
        # LogDirectory可为绝对路径；先解析实际产物，避免将磁盘路径重复拼到Tests目录。
        $taskBinaryPath = (Resolve-Path -LiteralPath $binary).Path
        & $taskBinaryPath *> $runLog
        $runResult = $LASTEXITCODE
        Get-Content $runLog
        if ($runResult -ne 0) { $failed++ }
    }
    Write-Output "Groups=$($Names.Count) Failed=$failed"
    if ($failed -ne 0) { exit 1 }
} finally { $env:PATH = $taskOriginalPath; Pop-Location }
