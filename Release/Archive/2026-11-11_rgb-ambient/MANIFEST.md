# 启动音后RGB慢速常亮渐变候选

日期：2026-11-11。V1-RGB-T01 / V1-E64，状态：软件验证通过，实板待验证。基于E63默认BMI088 I²C及双流/HUD/GPS当前源码，播放时继续按节拍七彩闪灯，响完后持续常亮缓慢变色；每色渐变2秒、七色14秒，通道最高48/255，不重播音乐。

- [可烧录HEX](cns-f407-rgb-ambient-20261111.hex)
- [详细开发日志及未完成项](../../../Development_Guide/Change_History/2026-11-11_启动音结束后RGB慢速常亮渐变.md)
- [生产源码](production_source.zip)、[测试源码](host_tests_source.zip)、[开发记录快照](development_records.zip)
- [本次双配置构建](build_matrix.json)、[4组最终测试](host_summary.json)、[初次测试与修正日志](host_attempts.zip)
- [输入身份](production_inputs.json)、[3项生产变更](changed_production.json)、[内存与固件校验](final_integrity.json)、[SHA256清单](SHA256SUMS.txt)

HEX SHA256：`ee8e81d6c290820aa86f03eff23d667b12c694bfccd5405cd242ace3802f933d`。

2026-11-11 08:54:03—08:54:37（UTC+8），唯一Keil工程/Target 1/ARMCC5.06/C99关闭RGB及默认RGB两配置Rebuild All均0 Error(s), 0 Warning(s)，日志13/12s，退出0。1043输入前后不变且与源码ZIP一致。4组最终-Werror测试通过；初次一项新增夹具断言失败，补齐末尾静音步骤后通过，未修改生产行为来迁就测试。

主SRAM125296B余5776B（+16B）；Flash实际485968B（+344B）。DMA254B在0x20008d3c主SRAM。硬件输出链未改；ERROR仍黑，恢复按当前相位，不积压旧帧。

未烧录或实测灯色、速度、平滑及长稳；模拟49.7天回绕不代表实板长稳。旧候选与Latest保留，既有M5/M6/资源/电流计等待验项不关闭。播放期间极短静音仍受10ms量化；fatal只停波形，灯可能保留末次锁存颜色。
