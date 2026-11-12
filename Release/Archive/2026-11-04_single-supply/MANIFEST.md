# 2026-11-04 单路主回路供电监测候选

状态：本地开发候选，软件验证完成，实板待验证。任务 `V1-PWR-T01`、证据 `V1-E34`；不构成V1.0稳定版或既有M5/M6验收通过。

| 项目 | 记录 |
|---|---|
| 功能 | 默认停用PA5电压1与PC0电流1，只保留PA4电压2、PC1电流2；界面统一Voltage / Current / Power；不判断电池/外接电源 |
| 默认配置 | BSP_ENABLE_POWER1/2=0/1，PX4LITE_ENABLE_BATTERY1/2=0/1；ADC1内部VREFINT保留 |
| 本地源码身份 | 当前目录没有.git，不能声明当前提交或分支；以production_inputs.json内1027项路径、大小和SHA-256识别 |
| 工程 / Target | MDK-ARM/formal_framework.uvprojx / Target 1 / Rebuild All |
| 实际工具链 | Keil MDK5.39 / ARMCC5.06 update7 build960 / C99；本机STM32F4xx_DFP3.1.1，工程仍记录2.17.0，HAL/CMSIS来自工程目录 |
| 工程元数据差异 | uvprojx记录ARMCC u5 build528，本轮实际使用u7；不宣称复现历史u5/DFP2.17.0构建环境 |
| 最终默认构建 | 2026-11-04 16:25:37—16:25:51 UTC+8；Keil日志11 s；退出码0，0 Error(s), 0 Warning(s) |
| 四配置验证 | 仅1、双路、全关、仅2依次Rebuild All，全部0错误0警告；最后恢复并构建默认仅2 |
| 主机回归 | GCC15.2.0，20组全部通过；覆盖采样、健康、恢复、协议、CSV、Control、显示和Debug |
| 生产输入一致性 | 每次构建前后1027项SHA一致，最终仅2清单与当前恢复后的配置一致；production_source.zip逐项核对一致 |
| Code / RO / RW / ZI | 340240 / 110872 / 1956 / 109988 B |
| 主SRAM静态使用 / 余量 | 111944 / 19128 B；包含固定堆池，不代表运行堆或栈水位 |
| Flash实际装载 | 451240 B，压缩装载口径 |
| HEX检查 | 全部记录校验通过、无重叠、连续范围[0x08000000, 0x0806E2A8)，不写sector11配置区 |
| HEX SHA-256 | 42f14ec12d6fd7f6ec7683fd27800ac2021a7dd7e43bacdddacba0887699ba2c |
| 输入清单SHA-256 | ac4b49637790bc328195bd5d3c960024424e548a33a46e5b1e2c60afca02ca3c |
| 源码ZIP SHA-256 | a47c967ee9ee0f124765b2eed0848e80c85d6d1dbcd1068803c9feac1ed72391 |

可烧录文件为[formal_framework.hex](formal_framework.hex)。本目录AXF、MAP、HTM、rebuild.log均取自最后默认仅2的同一次构建。`SHA256SUMS.txt`记录固件、源码和证据文件哈希；`keil_matrix.json`与`verification/`保留其他三配置的验证记录，其他配置不作为本候选下载固件。

`production_source.zip`范围是Core/Bsp/Sensor/Framework/Business/Display/Debug/Storage/Third_Party目录全部C/H/S及唯一uvprojx，包含部分未编入文件，不是整个项目文档或硬件资料备份。`host_tests_source.zip`保存Tests中的测试源码、脚本与桩，运行命令及20个测试名称见`verification/host_command.txt`；`host_final_summary.log`及`verification/*.build.log`、`*.run.log`保存本轮结果。

第二组仍使用Battery2 topic和MAVLink BATTERY_STATUS id=1，不能仅监听id=0。SYS_STATUS第一组字段未知，SOC与充电状态未知；CSV保持schema2、49列，停用第一组为Missing，未知百分比为空列。原Battery2电机保护阈值保留。详见[技术说明](../../../Development_Guide/Change_History/2026-11-04_单路主回路供电监测与统一显示.md)。

尚未烧录、核验主回路布线/ADC电平和标定、实屏、断流恢复、保护波形、满负载或24小时长稳；既有GT911触摸故障仍未定位。独立静态复核未发现本轮新增P1/P2，但不替代实板证据。未更新Latest、合入、推送或建立新标签。
