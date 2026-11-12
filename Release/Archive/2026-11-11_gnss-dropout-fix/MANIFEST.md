# GNSS异常半帧与收发故障隔离修复候选

日期：2026-11-11。V1-GNSS-R02 / V1-E68。基于E66，按E67审查结果修复两处掉线风险；软件验证完成，实板待验证。

- [可烧录HEX](cns-f407-gnss-dropout-fix-20261111.hex)
- [开发日志与未完成项](../../../Development_Guide/Change_History/2026-11-11_GNSS半帧期限与收发故障隔离修复.md)
- [生产源码](production_source.zip)、[测试源码](host_tests_source.zip)、[开发记录快照](development_records.zip)
- [本次Keil结果](build_result.json)、[39组最终host](host_summary.json)、[失败/修正及最终证据](verification.zip)
- [1047项输入](production_inputs.json)、[仅5项GNSS生产变化](changed_production.json)、[内存与固件完整性](final_integrity.json)、[SHA256清单](SHA256SUMS.txt)

HEX SHA256：`10d42da663e4497311f60b89c738e18cde8332eb20313e5e5545aa7986c4c866`。

UBX候选由首B5消费起绝对500ms到期，仅丢解析片段并报告实际RAW拒绝；完整原帧保真、未过期帧二进制隔离及界面保持。TX卡住优先只中止发送，RX DMA/队列保持；真实接收错误仍走原恢复。Adapter有新有效RX序号时独立发布，旧数据不续命；Health原2秒离线规则与固定115200保持。

Keil唯一工程、Target 1、ARMCC5.06/C99默认生产配置Rebuild All 0错误0警告、13s、退出0；39组最终host通过。1047项输入前后与源码ZIP一致，5项GNSS改动，其他1042项生产输入与E66一致。主SRAM126016B余5056B（+8B），ROM491236B（+432B），GNSS DMA在0x20009094/1024B主SRAM。

HUD完整帧、单路电压电流、电机锁、BMI088、RGB以及GNSS/BMI原始/解析MAVLink协议和速率保留。未烧录或实测掉线根因/接线/供电/噪声/长稳，不把软件通过等同现场通过；V1-GNSS-R02待验证，既有资源/M5/M6等保持。旧归档及Latest不覆盖。
