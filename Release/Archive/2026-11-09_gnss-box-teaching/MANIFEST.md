# 箱体GNSS教学功能候选

2026-11-09，R59 / V1-E55，V1-GNSS-T11待验证。父候选为主回路ADC诊断E54，既有电压/电流系数、第二组采样、人工锁和门禁默认0保留。

固件[cns-f407-gnss-box-teaching-20261109.hex](cns-f407-gnss-box-teaching-20261109.hex)，SHA256：`68900b56135e67e1a28cd4ee8300fd834999d923637352c243eb687fb8238c22`。

箱子操作入口：定位 → Details > → View >，Nav/Time/Sat循环。Time显示GNSS UTC、系统UTC、本地时间、来源、同步状态和同步年龄。Sat最多64项、每页5颗，Next sats循环页，Hold只定格当前已显示页，Live或翻页恢复；采集和自动上报继续。

UBX四步确认配置申请PVT/DOP/SAT与1000ms导航周期，仅模块RAM。NMEA接收、旧解析出口与RAW8020保持，不新增上位机命令或消息类型。逐星表当前支持UBX NAV-SAT，未声称完成NMEA/GSV逐星聚合。

33组最终选定host回归通过；Keil Rebuild All 0错误0警告、13s。1034项生产输入构建前后、当前源码与production_source.zip逐项一致，仅8项生产文件变更。主SRAM静态124184B、余6888B，比父版增加1664B；没有新增任务或运行期分配，既有79.22%堆预算超目标问题保留。

真实RunRpi负载模拟9s：8轮1Hz PVT100B+DOP26B+64星SAT784B，24/24完整原帧、192片原字节一致，队满/中止0，最大390ms；typed组、ACK/H1、旧遥测均继续。这只覆盖该模拟配置，不代表实板或任意NMEA负载下无损。

测试边界：GNSS原帧debug夹具已隔离独立POWER监视开关后重跑通过；旧reference测试仍期待已取消的全局远端显示选源，未计通过且未改生产迎合。细节见host_summary.json。手册两份均7页、按模板宋体制作并逐页检查；只使用箱体和纸笔，不要求电脑直连、额外SD、外接开关或上位机操作。

本候选未烧录。NAV-SAT实际输出、触摸交互、定位校时、SD保存、栈水位和长稳均待实板，M5/M6不关闭。Latest和旧归档不覆盖，无提交/推送/tag声明。
