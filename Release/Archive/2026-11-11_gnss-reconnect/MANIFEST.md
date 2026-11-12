# GNSS接收重建校验与静默重连候选

日期：2026-11-11。V1-GNSS-R02 / V1-E70。用户确认E68已烧录仍不稳定，本次继续修复接收重建与静默自愈；软件通过，实板待验证。

- [可烧录HEX](cns-f407-gnss-reconnect-20261111.hex)
- [开发日志及排查说明](../../../Development_Guide/Change_History/2026-11-11_GNSS重建校验与静默重连跟进.md)
- [生产源码](production_source.zip)、[测试源码](host_tests_source.zip)、[开发记录](development_records.zip)
- [最终Keil](build_result.json)、[45组host](host_summary.json)、[修复前复现/失败与最终证据](verification.zip)
- [1047输入](production_inputs.json)、[4项生产变更](changed_production.json)、[内存/固件完整性](final_integrity.json)、[SHA256清单](SHA256SUMS.txt)

HEX SHA256：`caf6619f9085ad99cd360362a49122ec1c22402c1df80973797734496d54e5b1`。

修复UART/DMA重开中断后已故障却返回成功的路径，补齐接收健康检查；原始字节连续静默6秒后请求owner重建，失败仍2秒退避。原固定115200、500ms半帧期限、2秒Health离线及协议查询限速保持。异常时新增GNSS-LINK/GNSS-IO两行USART1 Debug输出，115200/8N1，健康正常增长不重复打印。

最终run2 Keil唯一工程Target 1/ARMCC5.06/C99 Rebuild All 0错误0警告、13s；45组host通过，含真实BSP/Driver故障后再次收到GGA。首个审批超时及run1普通环境编译器未识别均留证据，不算成功构建。1047输入前后与源码ZIP一致，仅4项生产变更，其余1043项与E68相同。

主SRAM126184B余4888B（+168B），ROM492368B（+1132B）；GNSS DMA0x200090A4/1024B仍在主SRAM，CCM与其他功能保持。未烧录/现场根因未确认，不能仅凭软件通过认定接线正常或实板稳定。

烧录后若再次掉线，请保留连续数秒的`GNSS-LINK`与`GNSS-IO`，用于区分服务停滞、无字节、坏帧和重建失败；具体字段见日志。旧归档/Latest不覆盖，电流计及M5/M6/资源等既有待验项保持。
