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
    # 旧本地自动结果策略在显式回退模式验证；默认Pi通知由pi_result_notify专项验证。
    $mavLocalFeedback = $mav + @('-DPX4LITE_RPI_RESULT_SOURCE_PI=0')
    # 专项GPS/供电编码夹具隔离BMI；默认gnss_sat_stream保留5路真实共存验证。
    $mavNoBmi = $mav + @('-DPX4LITE_ENABLE_RPI_BMI088_UPLINK=0')
    $cases = [ordered]@{
        cns_control_v01 = @('-std=c99', '-Werror', 'Unit/test_cns_control_v01.c')
        cns_control_v01_off = @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_CNS_CONTROL=0', 'Unit/test_cns_control_v01.c')
        cns_tone_v01 = @('-std=c99', '-Werror', 'Unit/test_cns_tone_v01.c')
        cns_v01_integration = @('-std=c99', '-Werror', 'Unit/test_cns_v01_integration.c')
        cns_protocol_v01 = $mav + @('-std=c99', '-Werror', '-Wno-type-limits', 'Unit/test_cns_protocol_v01.c')
        power = @('Unit/test_power_adc_calibration.c', '../Sensor/Src/sensor_power.c')
        power_voltage_discharge_direct = @('-std=c99', '-Werror', '-DBSP_POWER2_USE_EXPANSION_ROUTE=0', 'Unit/test_power_voltage_discharge.c')
        power_voltage_discharge_direct_off = @('-std=c99', '-Werror', '-DBSP_POWER2_USE_EXPANSION_ROUTE=0', '-DBSP_ADC2_DISCHARGE_ENABLE=0', 'Unit/test_power_voltage_discharge.c')
        power_input_detection_integration_direct = @('-std=c99', '-Werror', '-DBSP_POWER2_USE_EXPANSION_ROUTE=0', 'Unit/test_power_input_detection_integration.c')
        single_supply_adc_direct = @('-std=c99', '-Werror', '-DBSP_POWER2_USE_EXPANSION_ROUTE=0', '-DBSP_ADC2_DISCHARGE_ENABLE=0', 'Unit/test_single_supply_adc.c')
        power_voltage_discharge = @('-std=c99', '-Werror', 'Unit/test_power_voltage_discharge.c')
        power_voltage_discharge_off = @('-std=c99', '-Werror', '-DBSP_ADC2_DISCHARGE_ENABLE=0', 'Unit/test_power_voltage_discharge.c')
        power_voltage_discharge_power2_off = @('-std=c99', '-Werror', '-DBSP_ENABLE_POWER2=0', 'Unit/test_power_voltage_discharge.c')
        power2_input_validity = @('-std=c99', '-Werror', 'Unit/test_power2_input_validity.c')
        power2_input_mapping = $mav + @('-std=c99', '-Werror', '-Wno-type-limits', 'Unit/test_power2_input_mapping.c')
        power2_input_mapping_off = $mav + @('-std=c99', '-Werror', '-Wno-type-limits', '-DTEST_BATTERY2_DISABLED=1', 'Unit/test_power2_input_mapping.c')
        power2_input_owner = $whole + @('-std=c99', '-Werror', 'Unit/test_power2_input_owner.c')
        power2_input_owner_dual = $whole + @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_BATTERY1=1', 'Unit/test_power2_input_owner.c')
        power2_input_owner_only1 = $whole + @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_BATTERY1=1', '-DPX4LITE_ENABLE_BATTERY2=0', 'Unit/test_power2_input_owner.c')
        power2_input_owner_alloff = $whole + @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_BATTERY2=0', 'Unit/test_power2_input_owner.c')
        power_input_detection_integration = @('-std=c99', '-Werror', 'Unit/test_power_input_detection_integration.c')
        motor_music = @('-std=c99', '-Werror', 'Unit/test_motor_music.c')
        motor_music_off = @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_MOTOR_MUSIC=0', 'Unit/test_motor_music.c')
        motor_music_rgb_off = @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_STARTUP_RGB=0', 'Unit/test_motor_music.c')
        motor_music_startup_off = @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_STARTUP_TONE=0', 'Unit/test_motor_music.c')
        motor_music_only = @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_STARTUP_TONE=0', '-DPX4LITE_ENABLE_RPI_COMMAND_FEEDBACK=0', '-DPX4LITE_ENABLE_MODULE_CONNECTION_FEEDBACK=0', 'Unit/test_motor_music.c')
        motor_music_only_rgb_off = @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_STARTUP_TONE=0', '-DPX4LITE_ENABLE_RPI_COMMAND_FEEDBACK=0', '-DPX4LITE_ENABLE_MODULE_CONNECTION_FEEDBACK=0', '-DPX4LITE_ENABLE_STARTUP_RGB=0', 'Unit/test_motor_music.c')
        motor_music_all_off = @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_MOTOR_MUSIC=0', '-DPX4LITE_ENABLE_STARTUP_TONE=0', '-DPX4LITE_ENABLE_RPI_COMMAND_FEEDBACK=0', '-DPX4LITE_ENABLE_MODULE_CONNECTION_FEEDBACK=0', '-DPX4LITE_ENABLE_STARTUP_RGB=0', 'Unit/test_motor_music.c')
        motor_music_score_playback = @('-std=c99', '-Werror', '-DTEST_MOTOR_MUSIC_REAL_SCORE=1', 'Unit/test_motor_music.c')
        motor_music_integration = $mav + @('-std=c99', '-Werror', '-Wno-type-limits', 'Unit/test_motor_music_integration.c')
        motor_music_integration_off = $mav + @('-std=c99', '-Werror', '-Wno-type-limits', '-DPX4LITE_ENABLE_MOTOR_MUSIC=0', 'Unit/test_motor_music_integration.c')
        motor_music_integration_only = $mav + @('-std=c99', '-Werror', '-Wno-type-limits', '-DPX4LITE_ENABLE_RPI_COMMAND_FEEDBACK=0', '-DPX4LITE_ENABLE_MODULE_CONNECTION_FEEDBACK=0', 'Unit/test_motor_music_integration.c')
        motor_music_integration_no_startup_rgb = $mav + @('-std=c99', '-Werror', '-Wno-type-limits', '-DPX4LITE_ENABLE_STARTUP_TONE=0', '-DPX4LITE_ENABLE_STARTUP_RGB=0', 'Unit/test_motor_music_integration.c')
        motor_music_integration_gate = $mav + @('-std=c99', '-Werror', '-Wno-type-limits', '-DPX4LITE_CONTROL_POWER_GATE_ENABLE=1', 'Unit/test_motor_music_integration.c')
        startup_rgb = @('-Werror', 'Unit/test_startup_rgb.c', '../Framework/Src/px4lite_startup_tone.c')
        startup_rgb_off = @('-Werror', '-DPX4LITE_ENABLE_STARTUP_RGB=0', 'Unit/test_startup_rgb.c', '../Framework/Src/px4lite_startup_tone.c')
        startup_tone_off = @('-Werror', '-DPX4LITE_ENABLE_STARTUP_TONE=0', 'Unit/test_startup_rgb.c', '../Framework/Src/px4lite_startup_tone.c')
        rpi_feedback = @('-std=c99', '-Werror', 'Unit/test_rpi_feedback.c')
        rpi_feedback_off = @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_RPI_COMMAND_FEEDBACK=0', 'Unit/test_rpi_feedback.c')
        rpi_feedback_rgb_off = @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_STARTUP_RGB=0', 'Unit/test_rpi_feedback.c')
        rpi_feedback_tone_off = @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_STARTUP_TONE=0', 'Unit/test_rpi_feedback.c')
        rpi_feedback_tone_rgb_off = @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_STARTUP_TONE=0', '-DPX4LITE_ENABLE_STARTUP_RGB=0', 'Unit/test_rpi_feedback.c')
        rpi_feedback_all_off = @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_STARTUP_TONE=0', '-DPX4LITE_ENABLE_RPI_COMMAND_FEEDBACK=0', '-DPX4LITE_ENABLE_MODULE_CONNECTION_FEEDBACK=0', 'Unit/test_rpi_feedback.c')
        module_feedback_off = @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_MODULE_CONNECTION_FEEDBACK=0', 'Unit/test_rpi_feedback.c')
        module_feedback_only = @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_RPI_COMMAND_FEEDBACK=0', 'Unit/test_rpi_feedback.c')
        module_feedback_both_off = @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_RPI_COMMAND_FEEDBACK=0', '-DPX4LITE_ENABLE_MODULE_CONNECTION_FEEDBACK=0', 'Unit/test_rpi_feedback.c')
        module_feedback_only_no_startup = @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_RPI_COMMAND_FEEDBACK=0', '-DPX4LITE_ENABLE_STARTUP_TONE=0', 'Unit/test_rpi_feedback.c')
        module_connection = $whole + @('-std=c99', '-Werror', 'Unit/test_module_connection_feedback.c')
        module_connection_off = $whole + @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_MODULE_CONNECTION_FEEDBACK=0', 'Unit/test_module_connection_feedback.c')
        module_connection_sources_off = $whole + @('-std=c99', '-Werror', '-DTEST_CONNECTION_MODULES_DISABLED=1', 'Unit/test_module_connection_feedback.c')
        module_connection_integration = $mav + @('-std=c99', '-Werror', '-Wno-type-limits', 'Unit/test_module_connection_feedback_integration.c')
        rpi_feedback_rx = $mavLocalFeedback + @('-std=c99', '-Werror', '-Wno-type-limits', 'Unit/test_rpi_feedback_rx.c')
        rpi_feedback_rx_off = $mavLocalFeedback + @('-std=c99', '-Werror', '-Wno-type-limits', '-DPX4LITE_ENABLE_RPI_COMMAND_FEEDBACK=0', 'Unit/test_rpi_feedback_rx.c')
        rpi_feedback_rx_rpioff = $mavLocalFeedback + @('-std=c99', '-Werror', '-Wno-type-limits', '-DTEST_RPI_DISABLED=1', 'Unit/test_rpi_feedback_rx.c')
        rpi_result_tx = $mavLocalFeedback + @('-std=c99', '-Werror', '-Wno-type-limits', 'Unit/test_rpi_result_feedback_tx.c')
        rpi_result_tx_off = $mavLocalFeedback + @('-std=c99', '-Werror', '-Wno-type-limits', '-DPX4LITE_ENABLE_RPI_COMMAND_FEEDBACK=0', 'Unit/test_rpi_result_feedback_tx.c')
        rpi_result_control = $mavLocalFeedback + @('-std=c99', '-Werror', 'Unit/test_rpi_result_feedback_control.c')
        rpi_result_control_off = $mavLocalFeedback + @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_RPI_COMMAND_FEEDBACK=0', 'Unit/test_rpi_result_feedback_control.c')
        rpi_result_control_gate = $mavLocalFeedback + @('-std=c99', '-Werror', '-DPX4LITE_CONTROL_POWER_GATE_ENABLE=1', 'Unit/test_rpi_result_feedback_control.c')
        pi_result_notify_integration = $mav + @('-std=c99', '-Werror', '-Wno-type-limits', 'Unit/test_pi_result_notify_integration.c')
        pi_result_notify_integration_local = $mavLocalFeedback + @('-std=c99', '-Werror', '-Wno-type-limits', 'Unit/test_pi_result_notify_integration.c')
        pi_result_notify_integration_off = $mav + @('-std=c99', '-Werror', '-Wno-type-limits', '-DPX4LITE_ENABLE_RPI_COMMAND_FEEDBACK=0', 'Unit/test_pi_result_notify_integration.c')
        pi_result_notify = $mav + @('-std=c99', '-Werror', '-Wno-type-limits', 'Unit/test_pi_result_notify.c')
        pi_result_notify_local = $mavLocalFeedback + @('-std=c99', '-Werror', '-Wno-type-limits', 'Unit/test_pi_result_notify.c')
        pi_result_notify_off = $mav + @('-std=c99', '-Werror', '-Wno-type-limits', '-DPX4LITE_ENABLE_RPI_COMMAND_FEEDBACK=0', 'Unit/test_pi_result_notify.c')
        pi_result_notify_rpioff = $mav + @('-std=c99', '-Werror', '-Wno-type-limits', '-DTEST_RPI_DISABLED=1', 'Unit/test_pi_result_notify.c')
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
        # 以下旧夹具只隔离静态比例/参考，新增默认弱泄放与恢复由独立BSP/Driver集成验证。
        single_supply_adc = @('-DBSP_ADC2_DISCHARGE_ENABLE=0', 'Unit/test_single_supply_adc.c')
        power2_conversion_audit = @('-Werror', '-DBSP_ADC2_DISCHARGE_ENABLE=0', 'Unit/test_power2_conversion_audit.c')
        power_adc_reference = @('-Werror', '-DBSP_ADC2_DISCHARGE_ENABLE=0', 'Unit/test_power_adc_reference.c')
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
        adc_current_expansion = @('-std=c99', '-Werror', 'Unit/test_adc_current_sampling.c')
        adc_current_expansion_probe = @('-std=c99', '-Werror', '-DBSP_ADC_CURRENT_FLOAT_CHECK_ENABLE=1', 'Unit/test_adc_current_sampling.c')
        adc_current_sampling = @('-DBSP_POWER2_USE_EXPANSION_ROUTE=0', '-DBSP_ENABLE_POWER1=1', 'Unit/test_adc_current_sampling.c')
        adc_current_probe = @('-DBSP_POWER2_USE_EXPANSION_ROUTE=0', '-DBSP_ENABLE_POWER1=1', '-DBSP_ADC_CURRENT_FLOAT_CHECK_ENABLE=1', 'Unit/test_adc_current_sampling.c')
        bme280_progress = @('-IUnit/runtime_stubs', 'Unit/test_bme280_progress.c')
        baro_schedule = $whole + @('Unit/test_baro_schedule.c')
        sensor_debug_ownership = $whole + @('Unit/test_sensor_debug_ownership.c')
        sensor_work_independence = $whole + @('-Werror', 'Unit/test_sensor_work_independence.c')
        three_sensor_debug = $whole + @('-Werror', 'Unit/test_three_sensor_debug.c')
        three_sensor_debug_off = $whole + @('-Werror', '-DTEST_THREE_SENSOR_DIAG_DISABLED=1', 'Unit/test_three_sensor_debug.c')
        sensor_debug_off = $whole + @('-DTEST_POWER_MONITOR=0', 'Unit/test_sensor_debug_ownership.c')
        sensor_debug_dual = $whole + @('-DPX4LITE_ENABLE_BATTERY1=1', 'Unit/test_sensor_debug_ownership.c')
        m2_sd_recovery = $mav + @('-Im2_comms_stubs', 'Unit/test_m2_sd_recovery.c')
        sd_spi_media = @('-std=c99', '-Werror', '-I../Third_Party/FatFs', 'Unit/test_sd_spi_media.c')
        m2_touch_recovery = $mav + @('-Im2_comms_stubs', 'Unit/test_m2_touch_recovery.c')
        m2_remoteid_fields = $mav + @('-Im2_comms_stubs', 'Unit/test_m2_remoteid_fields.c')
        display_field_validity = $whole + @('Unit/test_display_field_validity.c')
        display_supply_details = $whole + @('-std=c99', '-Werror', '-I../Third_Party/LVGL', '-I../Third_Party/LVGL/lvgl', 'Unit/test_display_supply_details.c')
        display_supply_details_only1 = $whole + @('-std=c99', '-Werror', '-DPX4LITE_ENABLE_BATTERY1=1', '-DPX4LITE_ENABLE_BATTERY2=0', '-I../Third_Party/LVGL', '-I../Third_Party/LVGL/lvgl', 'Unit/test_display_supply_details.c')
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
        gnss_recovery_liveness = $mav + @('-std=c99', '-Werror', '-Wno-type-limits', 'Unit/test_gnss_recovery_liveness.c')
        gnss_soft_recovery = @('-std=c99', '-Werror', 'Unit/test_gnss_soft_recovery.c')
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
    # 旧收包声光专项验证已保留的旧入口；严格V0.1端点/七参数由cns_protocol_v01独立验证。
    foreach ($legacyRx in @('rpi_feedback_rx','rpi_feedback_rx_off','rpi_feedback_rx_rpioff')) {
        $cases[$legacyRx] += '-DPX4LITE_ENABLE_CNS_CONTROL=0'
    }
    if ($Names.Count -eq 0) { $Names = @($cases.Keys) }
    New-Item -ItemType Directory -Force -Path $LogDirectory | Out-Null
    # 主机专项只给缺席的CNS边界加桩；真实实现或显式强桩不会被替换。
    function Read-TestCnsSources([string]$sourcePath, $seen) {
        $fullPath = [IO.Path]::GetFullPath($sourcePath)
        if (!$seen.Add($fullPath)) { return '' }
        $body = Get-Content -LiteralPath $fullPath -Raw -Encoding UTF8
        $combined = $body
        foreach ($included in [regex]::Matches($body, '#include\s+"([^"\r\n]+\.c)"')) {
            $child = Join-Path (Split-Path -Parent $fullPath) $included.Groups[1].Value
            if (Test-Path -LiteralPath $child) { $combined += "`n" + (Read-TestCnsSources $child $seen) }
        }
        return $combined
    }
    $cnsBoundaries = @('Px4Lite_ControlCnsRequest','Px4Lite_ControlCnsAckSent','Px4Lite_ControlCnsArmed',
        'Px4Lite_StartupToneCnsStart','Px4Lite_StartupToneCnsCancel','Px4Lite_StartupToneCnsResult',
        'Px4Lite_MavlinkCnsAckAvailable','Px4Lite_MavlinkQueueCnsAck','Px4Lite_CommandHandleRpiLongV01')
    $failed = 0
    foreach ($name in $Names) {
        if (!$cases.Contains($name)) { throw "Unknown test: $name" }
        $binary = Join-Path $LogDirectory "test_$name.exe"
        $buildLog = Join-Path $LogDirectory "$name.build.log"
        $runLog = Join-Path $LogDirectory "$name.run.log"
        $testCnsSources = ''
        $testCnsSeen = [Collections.Generic.HashSet[string]]::new([StringComparer]::OrdinalIgnoreCase)
        foreach ($source in @($cases[$name] | Where-Object { $_ -match '\.c$' })) {
            $testCnsSources += "`n" + (Read-TestCnsSources (Join-Path $PSScriptRoot $source) $testCnsSeen)
        }
        $testCnsFlags = @()
        foreach ($boundary in $cnsBoundaries) {
            if ($testCnsSources -notmatch ('(?m)^\s*(?:Px4Lite_Result_t|void|uint8_t)\s+' + $boundary + '\s*\([^;{}]*\)\s*\{')) {
                $testCnsFlags += "-DHOST_CNS_STUB_$boundary=1"
            }
        }
        & $compiler @common @($cases[$name]) @includes @testCnsFlags 'stubs/cns_boundary_stubs.c' -o $binary *> $buildLog
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
