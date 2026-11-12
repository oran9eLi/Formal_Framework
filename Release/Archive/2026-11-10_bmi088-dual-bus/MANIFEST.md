# BMI088双接口开发候选

日期：2026-11-10；证据V1-E58，任务V1-IMU-T01。当前源码默认BMI088 I²C，原MPU6050保留选择。**软件验证通过，模块尚未接线，实板待验证。** 不更新Latest、不烧录、不做GitHub发布。

| 固件 | 使用条件 | HEX SHA256 |
|---|---|---|
| [cns-f407-bmi088-i2c-20261110.hex](cns-f407-bmi088-i2c-20261110.hex) | 默认BMI088 I²C，PB6/PB7；IMU周期Debug默认0 | `b9a839868a42470709f752906bf48a5210967613a1a5bf4f57b3b51b33211b09` |
| [cns-f407-bmi088-spi-debug-20261110.hex](cns-f407-bmi088-spi-debug-20261110.hex) | BMI088 SPI2；IMU周期监视及缓存快照自检开启，仅用于接板诊断 | `347d532bffff8bd0f275adc4c64dd5512ed7a16fb1dbdc7d801a3c474b620e19` |
| [cns-f407-mpu6050-compatible-20261110.hex](cns-f407-mpu6050-compatible-20261110.hex) | 选择原MPU6050/I²C及旧轴向；不是BMI088固件 | `6a090bb307c8bccb9625bbda1026d7f143c20852ab85a7e51c9279099fe08baa` |

三配置均使用唯一Keil工程、ARMCC5.06/C99执行Rebuild All，0错误0警告；日志耗时依次默认I²C26s、SPI Debug26s、MPU27s。实际1039项生产输入各轮前后不变，结束后恢复默认项目；默认源文件、源码ZIP与最后I²C构建一致。两种覆盖配置的宏见`build_matrix.json`，不把它们称为默认源码直接构建。

35组选定host最终通过；首次5个旧夹具失败及修复经过见[开发日志](../../../Development_Guide/Change_History/2026-11-10_BMI088双接口驱动接入.md)。2项非法轴映射配置正确拒绝，不计入35组。未运行全库历史测试或实板验收。

默认主SRAM124392B、余6680B，较E55增加208B；Code367064/RO111852/RW2016/ZI122376B，压缩后ROM479044B。未增加任务、堆分配、DMA或中断；原堆预算与长稳问题保留。

| 文件 | 用途 |
|---|---|
| `production_source.zip`、`production_inputs.json` | 默认1039项生产输入及逐文件SHA256 |
| `formal_framework.axf`、`formal_framework.map` | 同次默认I²C调试程序与资源报告 |
| `build_matrix.json`、`verification/` | 三配置实际构建、覆盖宏、时间、产物SHA与日志 |
| `host_summary.json`、`host_logs.zip` | 最终35组选定回归及对应日志SHA |
| `host_tests_source.zip` | 同次Tests/Tools文本源码、测试入口及参考工具 |
| `configuration_checks.json` | 重复轴和反射坐标系被拒绝的检查 |
| `changed_production.json`、`final_integrity.json`、`SHA256SUMS.txt` | 15项生产变更、输入/ZIP一致性及归档校验 |

接线、模式及验收步骤见[BMI088接入与接线](../../../Development_Guide/BMI088接入与接线.md)。量程±12g/±1000°/s，100Hz；温度约1.28s更新，不声称DataSync或MEMS激励自检已实现。SPI备用脚须确认实物未被PHY等占用，模块供电不能仅凭名称推定。

GNSS、电压电流标定和电机人工锁/门禁设置保留。下一步由实板核对模式与双ID、安装轴向、重校准、传感器共存及掉线恢复；本候选不关闭M5/M6、资源或电流计问题。
