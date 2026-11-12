# PC9 RGB与启动旋律联动候选

日期：2026-11-10。任务V1-RGB-T01，证据V1-E60。状态：软件验证完成，实板待验证。继承E58默认BMI088 I²C，PC9彩灯按现有17音符旋律轮换七色，静音/结束请求黑帧，只播放一次；峰值通道48/255，无新页面。

- [可烧录HEX](cns-f407-rgb-startup-rhythm-20261110.hex)
- [开发日志及实板检查](../../../Development_Guide/Change_History/2026-11-10_PC9彩灯与蜂鸣器旋律节拍联动.md)
- [生产源码](production_source.zip)、[测试源码](host_tests_source.zip)、[14组测试](host_summary.json)、[双配置构建](build_matrix.json)
- [本次开发记录快照](development_records.zip)，含日志、索引、短设计、发布入口和执行台账；实际源码身份以生产输入清单为准。
- [输入清单](production_inputs.json)、[13项生产变更](changed_production.json)、[最终身份和内存](final_integrity.json)、[SHA256清单](SHA256SUMS.txt)

HEX SHA256：`95ebe3867b3e9adcce3075bf064994cb4170e393173fe6be88e22b5631536f76`。

唯一工程MDK-ARM/formal_framework.uvprojx，Target 1，ARMCC5.06 update7 build960/C99；2026-11-10 11:18:29—11:19:11（UTC+8）关闭RGB/默认RGB两配置Rebuild All均退出0、0 Error(s), 0 Warning(s)，日志20/19s。最终默认1041项生产输入前后不变并与源码ZIP一致。首次执行环境导致两配置在编译器启动前中止，留存在verification/initial_environment_abort，不计编译通过。

14组主机测试全部通过；默认Code370044、RO111872、RW2044、ZI122828B，主SRAM124872B余6200B（+480B），Flash压缩装载482048B（+3004B）。DMA254B位于0x20008bb0主SRAM；TIM8_CH4/DMA2 Stream7 Channel7独立于TIM9蜂鸣器及TIM3/TIM4电机。

未烧录或实测。后半旋律静音7.8125ms短于10ms周期，可能被跨过，灯跟随实际软件音调状态，不能保证每个音间都有可见黑隙。单向器件无在位反馈；灯色、响度、码宽、供电/公共地、sensor时序/栈和共存长稳待验。fatal只停波形，灯可能保留末次锁存颜色。既有堆预算/电流计及M5/M6状态保持，旧候选与Latest保留，未自动烧录、提交或推送。
