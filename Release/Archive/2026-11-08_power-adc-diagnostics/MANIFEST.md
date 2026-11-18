# 主回路ADC参考校验与串口诊断候选

日期2026-11-08；R58 / V1-E54，V1-PWR-T02继续进行中。父候选为2026-11-06_gnss-parsed-raw-restored。软件验证完成，未烧录或完成电压/电流实板标定。

固件：[cns-f407-power-adc-diag-20261108.hex](cns-f407-power-adc-diag-20261108.hex)，SHA256：`e491359e7b3052808dbecdbb0b112c8f2d9e6b4dc5c19baa2a3e7696fc701f87`。

修复非法工厂参考字、原码和VDDA计算结果缺校验；异常回退3300mV并明确标记/降级，合法参考仍实测换算。保留10.1779分压、电压增益1/偏移0和原电流系数。ADC2缓存同次原码/参考，现有sensor所有者发布，Debug每秒输出POWER、POWER-ADC、POWER-REF；关闭Debug不改变采集，无新任务或运行期分配。GPS双流、电机人工锁与电压门禁默认0均保留。

22组选定主机回归通过，无编译警告；实际Keil Rebuild All 0错误0警告、18s。1034生产输入构建前后、当前目录及production_source.zip逐项一致；HEX/AXF/MAP/日志为同次构建。主SRAM静态122520B，余8552B；既有堆预算79.22%超75%目标仍未关闭。

烧录后USART1 Debug设115200、8N1，保留主回路约12V与PA4约1.18V工况，提供同一seq的POWER-ADC和POWER-REF行及邻近POWER行。两条详细行的参考属于电压该次采集，PC1电流零点/灵敏度须独立标定。字段与status见[2026-11-08_主回路ADC参考校验与诊断.md](2026-11-08_主回路ADC参考校验与诊断.md)。

本候选仅完成软件检查，不认定现场唯一根因或测量准确性通过。未更新Latest、修改旧归档、提交或推送。
