# lixel_sdk 快速启动

**适用版本：** 1.0.0　　**面向：** 首次拿到设备、想尽快跑通数据的集成开发者

本文指导你从零跑通 XGRIDS 设备的数据订阅与录制控制，提供两条路径：

- **路径 A — C/C++**：直接链接 `liblixel_sdk.so` 调 API（SDK 的第一形态）。
- **路径 B — ROS2**：用自带的 ROS2 桥接节点，把设备数据转成标准 ROS2 Topic，`rviz2` 直接可视化。

逐字段接口契约见 [api_reference_v2.md](api_reference_v2.md)；坐标系/TF 概念见 [coordinate_frames_and_tf.md](coordinate_frames_and_tf.md)；完整集成与排障见 [user_guide.md](user_guide.md)。

---

## 1. 环境与前提

| 项 | 要求 |
|----|------|
| 操作系统 | Linux x86_64 / aarch64 |
| glibc | x86_64 ≥ 2.34；aarch64 ≥ 2.30（`ldd --version` 查看） |
| 网络 | 主机与设备**同一交换机直连**，设备 `modeling_app` 端口（默认 **7448**）可达 |
| 交付物 | `include/`（头文件）、`lib/liblixel_sdk.so`、`examples/`、`ros2/` |
| ROS2（仅路径 B） | Humble（Ubuntu 22.04）或 Foxy（Ubuntu 20.04），需 Eigen3 |

**端口说明：** SDK **只连 `modeling_app` 一个端口**。数据面（位姿/点云/IMU/图像/错误码）与命令面（录制启停、状态/标定/初值/路径查询的 `robot_cmd`）都在 modeling_app 的同一 zenoh session。

先做连通性自检（`open()` 超时最常见的原因就是这一步没过）：

```bash
ping <设备IP>                 # 网络与网段
nc -z <设备IP> 7448 && echo OK # modeling_app 端口可达
```

> 若设备端改过 modeling_app 端口，`Options.port` / `device_port` 直接填其实际端口；旧固件 modeling_app 落在随机端口 X 时，直接填 X（可在设备上 `ss -tlnp | grep lixel_app` 查到）。

---

## 2. 核心概念（读数据前必须知道的三件事）

1. **坐标系 = 设备（IMU）系**：SDK 输出的位姿/点云，原点在设备 IMU 点，遵循右手系（x 前 / y 左 / z 上）。载体（机器狗/车/无人机）的安装变换由你自己补一层 TF。设备只管"设备系"。
2. **两种点云不是一回事**：`subscribeCloud` 是重定位 NDT 配准点云（~10Hz，设备系）；`subscribeRawCloud` 是去畸变的单帧雷达扫描（~10Hz，未经建图配准）。要每帧扫描用后者。
3. **录制启停只下发、不等待**：`startRecord()` 返回成功仅表示命令被接受；进入 `Recording` 需轮询确认；而 `Recording` 也 **≠ 定位就绪**（重定位可能还要 1~2 分钟）。

---

## 路径 A — C/C++

### A.1 编译

```bash
g++ -std=c++17 my_app.cpp \
    -I<sdk>/include \
    -L<sdk>/lib -llixel_sdk \
    -Wl,-rpath,<sdk>/lib \
    -o my_app
```

`-rpath` 已内嵌库路径；或运行前 `export LD_LIBRARY_PATH=<sdk>/lib`。

### A.2 最小示例：连接 → 订阅位姿 → 断开

```cpp
#include "xg/robot_sdk.h"
#include <cstdio>
#include <thread>
#include <chrono>

int main() {
    xg::Options opts;
    opts.deviceIp = "192.168.123.103";   // 改成你的设备 IP
    // opts.port = 0;                     // 0 = 默认 7448（modeling_app）
    // opts.connectTimeoutMs = 0;         // 0 = 默认 5000ms

    xg::Error err = xg::Error::Ok;
    auto dev = xg::Device::open(opts, &err);
    if (!dev) {
        std::printf("连接失败: %s\n", xg::toString(err));
        return 1;
    }

    dev->subscribePose([](const xg::Pose& p) {   // 回调在 SDK 内部线程触发，勿阻塞
        std::printf("t=%.3f pos=(%.3f, %.3f, %.3f)\n",
                    p.timestampSec, p.posX, p.posY, p.posZ);
    });

    std::this_thread::sleep_for(std::chrono::seconds(10));
    return 0;   // dev 析构自动取消订阅并断开，无需手动 close
}
```

- `Device` 由 `unique_ptr` 持有，析构自动收尾；禁止拷贝/移动，**同一实例不可重入**（多线程自行加锁）。

### A.3 录制启停（只下发 + 轮询确认）

```cpp
if (dev->startRecord() != xg::Error::Ok) {
    // Timeout 时命令可能已送达，禁止直接重发，先 queryRecordStatus 再决定
}
xg::RecordStatus st;
while (true) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(XG_RECOMMENDED_POLL_INTERVAL_MS));  // 500ms
    if (dev->queryRecordStatus(&st) != xg::Error::Ok) continue;
    if (st.state == xg::RecordState::Recording) break;
    if (st.state == xg::RecordState::Error)     break;   // 处理故障
}
```

完整状态机见 `examples/record_demo.cpp`。

### A.4 可订阅的数据

所有订阅同构：`subscribeXxx(回调)` / `unsubscribeXxx()`，每类占一个订阅槽，重复订阅返回 `Rejected`。

| 数据 | C++ 接口 | 频率 | 坐标系 | 结构体 |
|------|----------|------|--------|--------|
| 位姿 | `subscribePose` | ~100 Hz | 设备(IMU)系 | `Pose` |
| reloc 点云 | `subscribeCloud` | ~10 Hz | 设备(IMU)系 | `CloudFrame` |
| 原始点云 | `subscribeRawCloud` | ~20 Hz | 雷达系（去畸变单帧） | `CloudFrame` |
| 原始 IMU | `subscribeImu` | ~200 Hz | IMU 系 | `Imu` |
| 相机图像 | `subscribeImage(CameraId, cb)` | 取决于设备配置 | 相机光学系 | `ImageFrame` |
| 错误码 | `subscribeErrCode` | ~10 Hz 批量 | — | `ErrCodeFrame` |

```cpp
dev->subscribeCloud([](const xg::CloudFrame& f) {
    // f.points 指向 SDK 内部缓冲，仅本次回调有效！留用须拷贝：
    // std::vector<xg_point_t> saved(f.points, f.points + f.count);
    std::printf("cloud: %zu points\n", f.count);
});
dev->subscribeImu([](const xg::Device::Imu& imu) {          // 纯标量，可按值保存
    std::printf("acc=(%.2f,%.2f,%.2f)\n", imu.accX, imu.accY, imu.accZ);
});
dev->subscribeImage(xg::Device::CameraId::Left,
    [](const xg::Device::ImageFrame& f) {                   // jpegData 仅回调内有效
        std::printf("jpeg: %zu bytes\n", f.jpegSize);
    });
```

### A.5 回调内存三条铁律

回调在 **SDK 内部线程**触发：

1. **不要阻塞**（会拖住整个会话线程，影响所有订阅）；重活拷贝后转交自己的线程。
2. **变长指针只在回调期内有效**（`CloudFrame::points`、`ImageFrame::jpegData` 指向复用缓冲，下一帧覆盖）；留用必须回调内深拷贝。
3. **不要 `free`/`delete` 回调里的任何指针**（不是你分配的）。

按值类型（`Pose`/`Imu`）可整体保存；`ErrCodeFrame::entries`（C++）是已拷出的 `vector`，跨回调持有仍需再拷。

---

## 路径 B — ROS2

用自带桥接节点把数据转成标准 ROS2 Topic，适合已有 ROS2 工作流、想直接在 `rviz2` 看的用户。

### B.1 编译

```bash
mkdir -p ~/ros2_ws/src
ln -sf <sdk>/ros2 ~/ros2_ws/src/lixel_sdk_ros2

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select lixel_sdk_ros2
```

### B.2 运行

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=<sdk>/lib:$LD_LIBRARY_PATH

ros2 launch lixel_sdk_ros2 sdk_bridge.launch.py device_ip:=192.168.123.103
```

节点启动后自动连接设备并启动录制，进入录制状态后开始发布数据；`Ctrl+C` 退出时自动停录并断开。

### B.3 发布的 Topic

| Topic | 类型 | 频率 | 说明 |
|-------|------|------|------|
| `/lixel/odom` | nav_msgs/Odometry | ~100Hz | 设备(IMU)系位姿，`map → base_link` |
| `/lixel/tf` | tf2_msgs/TFMessage | ~100Hz | `map → base_link` 动态变换 |
| `/lixel/cloud_points` | sensor_msgs/PointCloud2 | ~10Hz | 设备(IMU)系 reloc 点云 |
| `/lixel/raw_cloud` | sensor_msgs/PointCloud2 | ~20Hz | 去畸变原始点云 |
| `/lixel/imu` | sensor_msgs/Imu | ~200Hz | 原始 IMU |
| `/lixel/camera/{left,center,right}/compressed` | sensor_msgs/CompressedImage | 取决于设备 | 三路相机 JPEG |
| `/lixel/diagnostics` | diagnostic_msgs/DiagnosticArray | ~10Hz | 设备错误码 |

```bash
ros2 topic list             # 查看话题
ros2 topic hz /lixel/odom   # 确认位姿在发
```

### B.4 rviz2 可视化

```bash
rviz2
# Fixed Frame 设为 map
# 添加 TF / Odometry(/lixel/odom) / PointCloud2(/lixel/cloud_points)
```

### B.5 TF 结构与参数

TF 仅两层：`map ──(动态,~100Hz)──► base_link`。`base_link` 即设备 IMU 基准，桥接不做外参变换。若设备装在载体上，需要载体中心位姿时，由你在 `base_link` 之下再发一层静态 TF（设备 → 载体）。

常用参数（改 `config/default_params.yaml` 或 `--ros-args -p key:=value`）：`device_ip`、`device_port`（默认 7448）、`connect_timeout_ms`、`map_frame`、`base_frame`。详见 `ros2/README.md`。

---

## 3. 错误处理速览

所有接口以返回值报错，**不抛异常**：

| 返回码 | 典型原因 | 建议动作 |
|--------|----------|----------|
| `Ok` | 成功（启停时仅表示命令已接受） | 启停后轮询确认 |
| `InvalidArg` | 空指针、IP 空/超长 | 修正参数 |
| `Timeout` | 未在超时内收到应答 | 启停**勿直接重发**，先 `queryRecordStatus` |
| `Rejected` | 设备拒绝，或订阅槽已占 | 查状态，或先 `unsubscribe` |
| `Protocol` | 应答无法解析 | 核对固件/SDK 版本 |
| `Internal` | SDK 内部（如 declare 失败） | 检查连接与日志 |

`xg::toString(err)` 返回可读字符串。完整错误模型见 [api_reference_v2.md](api_reference_v2.md) §0.1。

---

## 4. 联调与排障

自带示例（编译后在 `build-<arch>/`）：

```bash
./record_demo <设备IP> [录制秒数]   # 录制启停 + 状态演变
./subscribe_demo <设备IP>           # 数据订阅
./all_features_demo <设备IP>        # 全功能演示
```

| 现象 | 原因 | 处理 |
|------|------|------|
| `open()` 超时 | 网络/端口不通 | `ping` + `nc -z <ip> 7448` |
| 连上但所有数据 0 帧 | 设备未在录制 | 先录制并等待进入 `Recording` |
| 位姿正常但点云为 0 | 重定位未运行 | 需加载先验地图并重定位成功 |
| 位姿/点云坐标"不对" | 按 Body 系理解了 | 数据是**设备(IMU)系**，载体变换自己补 |
| `startRecord` 返回 Timeout | 命令可能已送达 | 不要重试，先 `queryRecordStatus` |

---

## 5. 版本与兼容

- `xg_sdk_version()` = `"1.0.0"`。
- 位姿/点云为设备(IMU)系；`Options.port` 直接指定 modeling_app 端口（默认 7448），SDK 只连一个端点。
- `Pose.isLocal` 为**保留字段**（恒为 0），勿使用。
