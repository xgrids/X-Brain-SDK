# Release Notes 索引

本目录是 `lixel_sdk` 对外发布记录。版本号以公开头 `XG_SDK_VERSION_*` 与 `xg_sdk_version()` 为准，Git 史实见各篇「标识」栏。仓库**没有 annotated tag**；下列 SHA 来自 `develop` 历史。

当前集成请读：

- 用法：[README.md](../../README.md)
- 当前版完整接口说明书：[api_reference_v2.md](../api_reference_v2.md)

## 版本索引

| 版本 | 日期 | Git SHA | CMake `PROJECT_VERSION` | 头 / `xg_sdk_version()` | 相对上一版 ABI |
|------|------|---------|-------------------------|-------------------------|----------------|
| 0.1.0 | 2026-08-19 | `f56d825` | 0.1.0 | 0.1.0 | 基线（仅录制控制，本文档不展开） |
| [0.2.0](v0.2.0.md) | 2026-08-21 | `f0c277a` | 0.2.0 | 0.2.0 | 仅追加订阅符号 |
| [0.2.1](v0.2.1.md) | 2026-08-24 | `2b8acf5` | 0.2.1 | 0.2.1 | 无接口变更 |
| [0.3.1](v0.3.1.md) | 2026-08-27 | `93845de` | **0.3.0**（不一致） | **0.3.1** | 仅追加符号，无删除 |
| [1.0.0](v1.0.0.md) | 2026-09-01 | — | 1.0.0 | 1.0.0 | 符号集不变；**对外行为变更**：`Options.port`→modeling_app 端口（默认 7448）、只连一个端点、位姿/点云切设备(IMU)系 |

**不存在独立交付的 v0.3.0。** 头文件把 IMU / 图像 / 标定 / 设备状态标成「v0.3.0 新增」，把初值 / 路径 / 错误码订阅标成「v0.3.1 新增」；本仓库第一次可检出这些接口的提交就是 0.3.1。

## 兼容性矩阵（公开 C ABI）

`SOVERSION` 在 `0.x` 全程为 `0`，`1.0.0` 起为 `1`（CMake `SOVERSION ${PROJECT_VERSION_MAJOR}`）。`0.x` 不承诺 ABI 冻结；**`1.0.0` 起进入 ABI 冻结与兼容承诺**（公开符号只追加不删改）。升级必须同步头文件与 `liblixel_sdk.so`。

| 符号族 | 0.1.0 | 0.2.0 | 0.2.1 | 0.3.1 | 1.0.0 |
|--------|:-----:|:-----:|:-----:|:-----:|:-----:|
| 连接 / 录制 / 辅助 | Y | Y | Y | Y | Y |
| `subscribePose` / `subscribeCloud` | — | Y | Y | Y | Y |
| `subscribeImu` / `subscribeRawCloud` / `subscribeImage` | — | — | — | Y | Y |
| `getCameraCalib` / `getImuCalib` / `getLidarCalib` / `getDeviceInfo` | — | — | — | Y | Y |
| `setLocInitPose` / `getDataPath` / `subscribeErrCode` | — | — | — | Y | Y |

> **1.0.0 符号集与 0.3.1 完全一致**（无新增/删除符号），但含**运行期行为变更**（`Options.port` 语义、连接端点数、位姿/点云坐标系），详见 [v1.0.0.md](v1.0.0.md)。

## 共同约束（各版均适用）

- 同一 `xg_device_t` / `Device` **不可重入**；多线程须由调用方加锁。
- 启停只下发命令，结果必须轮询 `queryRecordStatus`。
- 本版本**无身份认证**，同网段任意主机可控制录制。
- 回调在 SDK 内部线程触发；指针型载荷仅在回调执行期间有效。
