# 固件发布目录

- `Latest/`：始终保存最近一次全量编译通过的固件和构建记录。
- `Archive/`：需要长期保留的版本，按日期和版本号建立子目录。

只有满足以下条件才允许更新 `Latest/`：

1. Keil ARMCC5 全量重建完成。
2. 编译结果为 `0 Error / 0 Warning`。
3. `hex`、`axf`、`map` 和构建日志来自同一次构建。
4. `MANIFEST.md` 已更新。

开发过程中的中间产物仍保留在 `MDK-ARM/Objects` 和
`MDK-ARM/Listings`，不得当作发布版本分发。
