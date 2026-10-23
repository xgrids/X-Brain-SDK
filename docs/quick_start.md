# lixel_sdk 快速启动

**适用版本：** 1.1.0　　**面向：** 首次拿到设备、想尽快跑通数据的集成开发者

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
| ROS2（仅路径 B） | Humble（Ubuntu 22.04），另需 CycloneDDS、cv_bridge、python3-opencv，见 `ros2/README.md` |

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
2. **去畸变点云来自 Locate**：1.1 起 `subscribeDeviceUndistCloud`（key `device_undist_cloud`）输出 Locate 去畸变点云（设备（IMU）系），建图阶段（Locate 未输出）无数据；`subscribeDeviceRelocCloud` 已在 1.1 删除。点数取决于 Locate 输入（未验证）。回调运行在 Zenoh RX 线程，回调内禁止同步文件 IO、`join`、`startLocate`。
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
| Locate 去畸变点云 | `subscribeDeviceUndistCloud` | 随 Locate 输出（频率未验证） | 设备(IMU)系 | `CloudFrame` |
| 原始 IMU | `subscribeImu` | ~200 Hz | IMU 系 | `Imu` |
| 相机图像 | `subscribeImage(CameraId, cb)` | 取决于设备配置 | 相机光学系 | `ImageFrame` |
| 错误码 | `subscribeErrCode` | ~10 Hz 批量 | — | `ErrCodeFrame` |

```cpp
dev->subscribeDeviceUndistCloud([](const xg::CloudFrame& f) {
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
cd <sdk>/ros2        # ros2/ 与 include/、lib/<arch>/ 平级
source /opt/ros/humble/setup.bash
colcon build --packages-select lixel_ros
```

### B.2 运行

先用 SDK 完成设备侧流程（初始化、设置初始位姿、`startRecord()`，见路径 A 或 `examples/comprehensive`）。桥接节点只订阅数据，不启停录制，`Ctrl+C` 只停 ROS 节点：

```bash
bash <sdk>/ros2/start_lixel_ros.sh                                  # 参数来自 config/default_params.yaml
bash <sdk>/ros2/start_lixel_ros.sh device_ip:=192.168.123.103       # 只覆盖设备 IP
```

### B.3 发布的 Topic

| Topic | 类型 | 说明 |
|-------|------|------|
| `/odom` | nav_msgs/Odometry | `map → base_link`，~100Hz |
| `/tf` | tf2_msgs/TFMessage | `map → imu_link`，~100Hz；`/tf_static` 发 `imu_link → base_link`（外参） |
| `/lixel/device_undist_cloud` | sensor_msgs/PointCloud2 | 设备(IMU)系 Locate 去畸变点云，随 Locate 输出 |
| `/lixel/imu` | sensor_msgs/Imu | 原始 IMU，~200Hz |
| `/lixel/camera/{left,center,right}/compressed` | sensor_msgs/CompressedImage | 三路相机 JPEG，帧率以设备配置为准 |
| `/lixel/diagnostics` | diagnostic_msgs/DiagnosticArray | 设备错误码 |

```bash
ros2 topic list             # 查看话题
ros2 topic hz /odom         # 确认位姿在发
```

### B.4 rviz2 可视化

```bash
rviz2
# Fixed Frame 设为 map
# 添加 TF / Odometry(/odom) / PointCloud2(/lixel/device_undist_cloud)
```

### B.5 TF 结构与参数

TF：`map ──(动态,~100Hz)──► imu_link ──(静态,外参)──► base_link`。`base_link` 是 ROS 直出的 ODOM 坐标系，默认原点在扫描仪原点，手持时 X 朝前；装到机器人上只改平移外参。全部参数（设备连接、坐标系、外参、话题名）在 `config/default_params.yaml`，换算方法与坐标系定义见 [`ros2/README.md`](../ros2/README.md)。

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

自带示例的预编译产物在 `bin/<arch>/<分类>/`（以下以 x86_64 为例，分类说明见 `examples/README.md`）：

```bash
./bin/x86_64/record/record_demo <设备IP> [录制秒数]          # 录制启停 + 状态演变
./bin/x86_64/subscribe/subscribe_demo <设备IP> [输出目录|-] [X] [Y] [Z]   # 数据订阅，自带 setLocInitPose，X/Y/Z 为地图系初值(m)，默认 0 0 0
./bin/x86_64/comprehensive/all_features_demo <设备IP> [订阅秒=15] [X=0] [Y=0] [Z=0]   # 全功能演示，自带 setLocInitPose，X/Y/Z 为地图系初值(m)，默认 0 0 0；默认值在本机地图上可能重定位失败，请传设备实际所在附近的坐标
```

| 现象 | 原因 | 处理 |
|------|------|------|
| `open()` 超时 | 网络/端口不通 | `ping` + `nc -z <ip> 7448` |
| 连上但所有数据 0 帧 | 设备未在录制 | 先录制并等待进入 `Recording` |
| 位姿正常但点云为 0 | 重定位未运行 | 需加载先验地图并重定位成功 |
| 位姿/点云坐标"不对" | 按 Body 系理解了 | 数据是**设备(IMU)系**，载体变换自己补 |
| 图像、IMU、点云的时间戳互相对不上，或与本机时间差很多 | 各路取自不同的时钟（设备系统时钟、MCU、雷达） | 见 [api_reference_v2.md §0.8](api_reference_v2.md#08-时间戳时基与换算)；图像、深度图可用 `getDeviceTime` 换到本机时钟 |
| `getDeviceTime` 返回 `Protocol` | 设备固件不支持时间查询（应答里没有 `time`） | 升级设备固件；`getDeviceInfo` 等其他接口不受影响 |
| `startRecord` 返回 Timeout | 命令可能已送达 | 不要重试，先 `queryRecordStatus` |

---

## 5. 版本与兼容

- `xg_sdk_version()` = `"1.1.0"`。
- 动态库 SONAME 为 `liblixel_sdk.so.1.1`（`<主>.<次>`两位）。补丁版本不改 SONAME，次版本改 SONAME 须重新链接；1.0.0 的 SONAME 是 `liblixel_sdk.so.1`，用 1.0.0 编译的程序须重新编译，详见 [releases/v1.1.0.md](releases/v1.1.0.md)。
- 位姿/点云为设备(IMU)系；`Options.port` 直接指定 modeling_app 端口（默认 7448），SDK 只连一个端点。
- `Pose.isLocal` 为**保留字段**（恒为 0），勿使用。
