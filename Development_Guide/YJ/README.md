# YJ 迁移资料说明

本目录收录自 `origin/NW3` 分支快照中的开发资料，用于保留 MPU6050、BME280、Power ADC 和 SD-CSV 等模块的迁移思路、调试记录和设计参考。

这些文档属于分支资料归档，不替代仓库根目录下 `README.md`、`AGENTS.md` 和 `Development_Guide/07_移植进度与功能清单.md` 的事实口径。判断模块是否完成时，仍以当前代码、当前 Keil 构建结果和 `Development_Guide/07_移植进度与功能清单.md` 为准。

从 `NW3` 手工合入当前工程时，已保留以下有效内容：

- MPU6050 机体系轴向映射思路。
- Power ADC 实测分压系数和电量百分比稳定策略。
- 模块迁移记录、开发日志和技术说明。

以下内容不直接从 `NW3` 快照合入：

- `.clang-format`、`.gitignore`、`AGENTS.md`、`README.md` 等仓库规则文件。
- RTC/GNSS 时间持久化相关文件的删除。
- `build/` 下的本地构建产物。
- 未按当前中文 Doxygen 注释规范整理的旧格式源码。
