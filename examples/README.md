# 示例程序（examples）

示例按功能分为 6 类子目录。每个 demo 都是独立的 `main`，通常第一个参数为设备 IP。

**预编译产物的位置与分类一一对应：** `examples/<分类>/<名>.cpp` 的可执行文件是 `bin/<arch>/<分类>/<名>`（对外仓），例如 `examples/subscribe/pose_demo.cpp` 对应 `bin/x86_64/subscribe/pose_demo`。下文命令只写程序名，按上述路径补全即可。

| 子目录 | demo | 说明 |
|---|---|---|
| `connection/` | `device_info_demo` | 连接设备、查询设备状态、设备时间（含与本机时钟的偏差区间）与标定信息 |
| `record/` | `record_demo` | 启动/停止数据录制 |
| `subscribe/` | `subscribe_demo` | 订阅全量数据流（位姿、IMU、去畸变点云、三路图像）；自带 `setLocInitPose`，单独运行位姿和去畸变点云就有数据 |
| | `undist_cloud_demo` | 订阅设备(IMU)系 Locate 去畸变点云（`subscribeDeviceUndistCloud`）并存 PCD；自带 `setLocInitPose` |
| | `raw_cloud_demo` | 订阅设备(IMU)系原始雷达全量点云（`subscribeDeviceRawCloud`，仅 RoboSense，未去畸变，带逐点时间偏移）并可存 PCD |
| | `imu_demo` | 订阅 IMU 数据 |
| | `image_demo` | 订阅相机图像（帧率以设备 `jpeg.fps` 配置为准，本机实测 10.01 Hz） |
| | `depth_demo` | 订阅中间相机 AI 深度图，工作线程做 Jet 伪彩色并存 PPM。设备端默认配置（`lixel_t1xb.camera_center.ai.fps: 0`）下不发布 `depth/center`，须设备端配置为大于 0 且 AI 模型就绪后才有数据 |
| | `pose_demo` | 订阅融合位姿；自带 `setLocInitPose`，可选存 CSV |
| `relocation_map/` | `map_upload_demo` | **上传重定位地图到设备、激活、列出设备现有地图** |
| | `reloc_demo` | 重定位：注入初始位姿后订阅位姿与点云观察。参数顺序 `<设备IP> [X] [Y] [Z] [注入前等待秒] [订阅时长秒]`，位置单位 m、世界/地图系，默认 `X=0 Y=0 Z=0`（地图原点，请按实际位置传入） |
| `network/` | `network_demo` | **修改设备对外网口的 IP / 网段，限时确认，超时自动回滚** |
| `comprehensive/` | `all_features_demo` | 全功能串联演示 |

## 命令行参数一览与先决条件

`[ ]` 为可选。位置 `X/Y/Z` 单位 m、世界/地图系，默认 `0 0 0`（地图原点），朝向固定为单位四元数，请按设备在地图上的实际位置传入。`-h` / `--help` 在 `pose_demo`、`undist_cloud_demo`、`subscribe_demo` 中打印用法并以 0 退出。

| demo | 命令行 | 说明 |
|---|---|---|
| `device_info_demo` | `<设备IP>` | 只读，不录制；同时打印录制状态，可作为其他示例起跑前的就绪探针 |
| `record_demo` | `<设备IP> [录制秒=10]` | 录制启停与状态演变；无信号处理，勿用 Ctrl+C 打断 |
| `imu_demo` | `<设备IP> [持续秒=60]` | 不依赖 Locate |
| `image_demo` | `<设备IP> [持续秒=60] [保存目录]` | 需设备 `jpeg.fps > 0`；保存目录不存在时自动创建 |
| `raw_cloud_demo` | `<设备IP> [持续秒=60] [保存目录]` | 仅 RoboSense；不依赖 Locate；保存目录不存在时自动创建 |
| `depth_demo` | `<设备IP> [持续秒=60] [保存目录\|-] [近端米 远端米]` | 需设备 `ai.fps > 0` 且 AI 模型加载成功 |
| `pose_demo` | `<设备IP> [持续秒=60] [输出CSV\|-] [X=0] [Y=0] [Z=0]` | 先订阅再 `setLocInitPose`；`-` 表示不存 CSV |
| `undist_cloud_demo` | `<设备IP> [持续秒=60] [保存目录\|-] [X=0] [Y=0] [Z=0]` | 先订阅再 `setLocInitPose`；`-` 表示不存 PCD |
| `subscribe_demo` | `<设备IP> [输出目录\|-] [X=0] [Y=0] [Z=0]` | 无持续时间参数，Ctrl+C 退出；先订阅再 `setLocInitPose` |
| `reloc_demo` | `<设备IP> [X=0] [Y=0] [Z=0] [注入前等待秒=5] [订阅时长秒=60]` | 先注入再订阅，不判定重定位是否成功 |
| `all_features_demo` | `<设备IP> [订阅秒=15] [X=0] [Y=0] [Z=0]` | 默认 0 0 0 在本机地图上重定位可能失败（位姿 0 帧属预期），想要位姿数据请传设备实际所在附近的坐标；`setLocInitPose` 在订阅之后、等数据之前 |
| `map_upload_demo` | `<设备IP> <本机地图目录> <mapId>` | 会改设备有效地图 |
| `network_demo` | 见下文「修改设备 IP / 网段」 | 会改设备 eth1 网络 |

**Locate 时序（pose / undist_cloud / subscribe / all_features / reloc 共同遵循）：** 设备端 Locate 在录制启动时完成初始化，`setLocInitPose` 才喂入初值并启动算法，之后才有位姿（`device_locate`）和去畸变点云（`device_undist_cloud`）。所以这些示例的流程都是：连接 → `startRecord` → 等 Recording → 订阅 → `setLocInitPose` → 采集 → `stopRecord`。注入失败（Rejected / Timeout）会打印原因和建议；结束时位姿或去畸变点云为 0 帧会提示"Locate 未启动或重定位未成功"。

**会录制的示例的共同副作用：** 每次运行在设备上生成一个录制工程（本机实测 87~103 MB/次）；`stopRecord` 之后设备应用退出并由 systemd 重启，10~20 s 内不可用，下一次运行前先用 `device_info_demo` 确认连得上且录制状态为 idle。重启期间下发的 `startRecord` 会被排队。回调都在 Zenoh RX 线程触发，不得阻塞。

## 地图管理（relocation_map）

`map_upload_demo` 覆盖地图的完整生命周期：

```bash
# 用法：<设备IP> <本机地图目录> <mapId>
./map_upload_demo 192.168.123.103 /home/user/maps/site_A site_A
```

流程：连接 → 上传地图（带进度回调，内部走分片 + NACK 重传）→ 激活（setActiveMap）→ 列出设备现有地图（listMaps，标注激活项与逐文件大小）。

本机地图目录结构要求（缺一不可）：

```
<地图目录>/
├── location_map.las
└── loc_database/
    ├── camera_param.proto
    ├── fpfh_feature.proto
    ├── keyframes.proto
    └── vlad.proto
```

对应的地图管理 API（见 `include/xg/robot_sdk.h`）：`uploadMap` / `setActiveMap` / `listMaps` / `deleteMap`。

## 修改设备 IP / 网段（network）

`network_demo` 修改 eth1 的 IP / 网段，并走完「限时确认」流程，窗口内没确认设备自动回滚：

```bash
# <设备连接IP> <新IP> <前缀长度> [网关|keep|none] [确认窗口秒] [--confirm-via <确认用的设备地址>]
./network_demo 192.168.123.103 192.168.123.150 24                              # 经 eth1，同网段改 IP
./network_demo 192.168.55.1 192.168.123.150 24 --confirm-via 192.168.55.1     # 经 USB 网卡改 eth1
```

- 省略 `--confirm-via` 时经新 IP 确认，本机须能到达新网段（换网段时先给网卡加辅助地址）。
- 指定 USB 等其他网卡上的设备地址时，只证明设备在线，不证明新 IP 可达。
- 经 USB 确认时，确认可能早于设备应用新配置到达而被拒绝，demo 会在窗口内自动重试。
- 新网段不能与设备其他网口（含 USB 网卡所在网段）重叠。
- 退出码：0 已确认；1 参数或连接失败；2 设备未修改；3 未确认；4 连接地址都连不上。
