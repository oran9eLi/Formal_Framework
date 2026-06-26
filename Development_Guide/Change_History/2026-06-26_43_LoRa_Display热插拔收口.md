# 2026-06-26 LoRa/Display 热插拔收口

## 背景

此前 GNSS、IMU、BME280、电池 ADC 和 SD 卡已有完整或较完整的热插拔恢复链路；LoRa E22 有恢复入口但本机模块拔掉和对端超时容易混在一起；LCD/GT911 有初始化和恢复入口，但运行中失联后的探测与重试不完整。

## 本次变更

1. LoRa E22 的 AUX 引脚改为下拉输入，模块拔掉后不再依赖悬空电平。
2. LoRa Driver 增加 AUX 最近就绪时间和本机不可用判断：半双工发送窗口内 AUX 拉低不判离线，AUX 长期不可用才认为本机模块不可用。
3. Framework 通信状态收口为三态语义：本机 E22 离线为 OFFLINE/红灯；对端合法 MAVLink RX 超时为 DEGRADED/黄灯；最近收到对端合法帧才 ONLINE/绿灯。
4. LoRa 通用 recovery 开启，恢复回调只设置重初始化请求，实际执行仍由 comm service 完成。
5. SSD1963 增加周期 PID 探测，探测失败时清除 Display ready，随后走 Business/Health/Recovery 链路重新初始化。
6. GT911 输入设备注册不再依赖开机探测成功；触摸芯片缺失或读失败后，由 LVGL 输入回调在 DisplayTask 中每 1 s 限频重探测，插回后自动恢复触摸。

## 边界结论

- LoRa Driver 只记录本机硬件事实、合法 MAVLink RX 事实和通信统计，不解释业务数据。
- 对端收不到数据不是本机 LoRa 拔掉；对端新鲜度只影响链路降级颜色和远端节点状态。
- LCD/GT911 热插拔只在 Display 层内部处理，页面层和 Business 层不得直接访问 FSMC、I2C 或触摸驱动私有状态。
- GT911 触摸不可用不应阻断 LCD 页面刷新；LCD 控制器不可用才推动 Display 模块离线和恢复。

## 验证

已使用 `D:\Keil_v5\UV4\UV4.exe` 执行 Keil Rebuild All，工程入口为 `MDK-ARM/formal_framework.uvprojx`。

```text
Program Size: Code=243484 RO-data=80412 RW-data=1076 ZI-data=113732
".\Objects\formal_framework.axf" - 0 Error(s), 0 Warning(s).
Build Time Elapsed:  00:00:36
```

台架验证仍需覆盖：运行中拔掉 E22、插回 E22、只断对端数据、运行中断开 LCD、插回 LCD、断开/插回 GT911 触摸。
