# 在 Windows 原生 PowerShell 中构建/运行 host 逻辑测试；不生成 STM32 固件。
param([string[]]$Names = @(), [string]$LogDirectory = 'build/host')
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    $compiler = 'C:/msys64/mingw64/bin/gcc.exe'
    # stubs 优先：host assert 失败写 stderr 并非零退出，不触发 Windows CRT 错误窗口。
    $includes = @('-I../Core/Inc', '-I../Debug/Inc', '-I../Bsp/Inc', '-I../Sensor/Inc', '-I../Framework/Inc', '-I../Business/Inc', '-I../Display/Inc', '-I../Storage/Inc')
    $common = @('-std=c11', '-Wall', '-Wextra', '-O0', '-g', '-Istubs')
    $whole = @('-O1', '-fwhole-program')
    $mav = $whole + @('-I../Third_Party/mavlink', '-Wno-address-of-packed-member')
    $cases = [ordered]@{
        power = @('Unit/test_power_adc_calibration.c', '../Sensor/Src/sensor_power.c')
        control = @('Unit/test_control_motor_logic.c')
        imu = @('Unit/test_platform_imu_axis_mapping.c')
        display = $whole + @('Unit/test_display_remote_control.c')
        command = $mav + @('Unit/test_command_motor.c')
        alarm = $mav + @('-Wno-type-limits', 'Unit/test_mavlink_alarm_table.c')
        attitude = @('Unit/test_attitude_measurement.c', '../Framework/Src/px4lite_attitude.c')
        attitude_ops = @('Unit/test_attitude_operations.c', '../Framework/Src/px4lite_attitude.c')
        estimator = $whole + @('Unit/test_estimator_attitude_requests.c')
        reference = $whole + @('Unit/test_display_attitude_reference.c')
        storage = $whole + @('Unit/test_storage_freshness.c')
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
        adc_current_sampling = @('Unit/test_adc_current_sampling.c')
        adc_current_probe = @('-DBSP_ADC_CURRENT_FLOAT_CHECK_ENABLE=1', 'Unit/test_adc_current_sampling.c')
        bme280_progress = @('-IUnit/runtime_stubs', 'Unit/test_bme280_progress.c')
        baro_schedule = $whole + @('Unit/test_baro_schedule.c')
        sensor_debug_ownership = $whole + @('Unit/test_sensor_debug_ownership.c')
        sensor_debug_off = $whole + @('-DTEST_POWER_MONITOR=0', 'Unit/test_sensor_debug_ownership.c')
        m2_sd_recovery = $mav + @('-Im2_comms_stubs', 'Unit/test_m2_sd_recovery.c')
        m2_touch_recovery = $mav + @('-Im2_comms_stubs', 'Unit/test_m2_touch_recovery.c')
        m2_remoteid_fields = $mav + @('-Im2_comms_stubs', 'Unit/test_m2_remoteid_fields.c')
        display_field_validity = $whole + @('Unit/test_display_field_validity.c')
        display_lvgl_invalidation = $whole + @('-I../Third_Party/LVGL', '-I../Third_Party/LVGL/lvgl', 'Unit/test_display_lvgl_invalidation.c')
        remote_freshness = $whole + @('Unit/test_remote_freshness.c')
        remote_navigation = $mav + @('-Wno-type-limits', 'Unit/test_remote_navigation.c')
        remote_slots = $whole + @('Unit/test_remote_slots.c')
        gnss_measurement = $whole + @('Unit/test_gnss_measurement.c')
        gnss_dma = $whole + @('Unit/test_gnss_dma.c')
        gnss_tx_freshness = $mav + @('-Wno-type-limits', 'Unit/test_gnss_tx_freshness.c')
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
        & (Join-Path (Get-Location) $binary) *> $runLog
        $runResult = $LASTEXITCODE
        Get-Content $runLog
        if ($runResult -ne 0) { $failed++ }
    }
    Write-Output "Groups=$($Names.Count) Failed=$failed"
    if ($failed -ne 0) { exit 1 }
} finally { Pop-Location }
