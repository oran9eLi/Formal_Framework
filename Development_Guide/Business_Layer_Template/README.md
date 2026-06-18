# 业务层正式接入记录

业务层代码已经从本目录迁入正式工程：

- `Business/Inc/business_*.h`
- `Business/Src/business_*.c`
- `MDK-ARM/formal_framework.uvprojx`

当前正式启用：

1. 非阻塞 EventBus，单个订阅者队列满不影响其他订阅者。
2. topic 级和 subscriber 级投递统计。
3. `biz_system` 固定周期任务，启动日志只产生一次。
4. `biz_acq` 固定周期业务分发任务，只复制 topic，不读取硬件。
5. `biz_display` 单任务多速率服务，触摸优先、刷新分步执行。
6. Display 单开关 `BUSINESS_ENABLE_DISPLAY=1U`。

当前 Display 真实驱动尚未实现，因此
`Business/Src/business_display_placeholder.c` 返回
`BUSINESS_SERVICE_NOT_READY`。后续只替换这个适配文件，不修改任务流程。

本目录不再保存重复源码，避免正式代码和示例代码形成两个版本。
