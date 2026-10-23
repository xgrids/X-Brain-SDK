# XGRIDS Robot SDK 集成指南

本文档面向集成方开发人员，提供 SDK 使用的完整参考，包括坐标系概念、数据订阅、ROS2 桥接、测试验证与排障。

基础接口说明（连接、录制启停、编译链接）请参阅根目录 [README.md](../README.md)。完整 C/C++ 契约见 [api_reference_v2.md](api_reference_v2.md)，当前版本的 Release Note 见 [releases/v1.1.0.md](releases/v1.1.0.md)。

尚未提供的能力请勿据本文推测。

---

## 1. 核心概念

### 1.1 坐标系

SDK 对外统一输出**设备（IMU）系**的位姿与点云。涉及的坐标系：

| 坐标系 | 简称 | 定义 | 特征 |
|--------|------|------|------|
| 世界坐标系 | W 系 / odom | 设备首次建图时的起始位置为原点 | 全局固定不动 |
| 设备（IMU）坐标系 | I 系 / sensor_link | 原点在 IMU 芯片安装点，轴向由 IMU 硬件决定，为整机基准 | 随设备运动 |

> SDK 位姿/点云为设备（IMU）系。设备只输出"以设备为中心、无歧义"的坐标；**载体（狗/车/机器人本体）的安装变换由集成方自行负责**（如在 ROS 侧再挂一层 `base_link → sensor_link` 的 TF）。坐标系详见 [coordinate_frames_and_tf.md](coordinate_frames_and_tf.md)。

坐标系间的变换关系：

```
世界系 (odom)
 │
 ├── R_W_I, t_W_I（SDK 输出的位姿）
 │
 └── 设备(IMU)系 (sensor_link)
      │
      └── 载体系 (base_link)  ← 由集成方按自己的安装外参挂接，SDK 不提供
```

### 1.2 位姿数据

SDK 通过 `subscribePose()` 回调输出的位姿数据（~100Hz）：

| 字段 | 坐标系 | 单位 | 说明 |
|------|--------|------|------|
| posX/Y/Z | W 系 | 米 | 设备（IMU）安装点在世界系下的位置 |
| quatW/X/Y/Z | W←I | — | 设备（IMU）相对世界系的姿态四元数（R_W_I） |
| velX/Y/Z | I 系 | m/s | 设备（IMU）系下的线速度 |
| gyroX/Y/Z | I 系 | rad/s | 设备（IMU）系下的角速度 |
| errorCode | — | — | 定位故障码，0=正常 |
| isLocal | — | — | true=无图定位，false=有图定位 |

四元数 `q = (w, x, y, z)` 满足 `w² + x² + y² + z² = 1`，表示一个三维旋转。`(1, 0, 0, 0)` 表示无旋转。

### 1.3 点云数据

`subscribeDeviceUndistCloud()` 回调输出的是设备（IMU）系 Locate 去畸变点云（1.1 起；建图阶段 Locate 未输出时无数据）：

| 字段 | 单位 | 说明 |
|------|------|------|
| x, y, z | 米 | 点在设备（IMU）系下的坐标 |
| intensity | — | 反射强度 |

注意：`subscribeDeviceRelocCloud()` 已在 1.1 删除。回调运行在 Zenoh RX 线程，回调内禁止同步文件 IO、`join`、`startLocate`。

需要雷达全量点、并自行做运动去畸变时，订阅 `subscribeDeviceRawCloud()`（RoboSense 设备，~10Hz）：每帧是一次完整扫描的全部点，未降采样、未去畸变，设备（IMU）系；每个点带 `time_offset_us`（相对帧时间的微秒偏移）和 `ring`。坐标精度 1 mm。接口细节见 `api_reference_v2.md` 的 §3.3.1 与 §7.2.1，示例见 `examples/subscribe/raw_cloud_demo.cpp`。

### 1.4 IMU 数据

`subscribeImu()` 回调输出原始 IMU 数据（~200Hz）：

| 字段 | 单位 | 说明 |
|------|------|------|
| accX/Y/Z | m/s² | 加速度 |
| gyroX/Y/Z | rad/s | 角速度 |

### 1.5 相机图像

`subscribeImage()` 按相机 ID 订阅 JPEG 图像：

| CameraId | 相机 | 说明 |
|----------|------|------|
| Left (0) | 左相机 | 鱼眼，~10fps |
| Center (1) | 中相机 | 鱼眼，~10fps |
| Right (2) | 右相机 | 针孔，~10fps |

注意：图像发布依赖设备端录制状态和固件版本。

### 1.5.1 深度图

`subscribeDepth()` 订阅中间相机的 AI 深度图（key `depth/center`，度量深度，单位 m，行优先 float）。**设备端默认配置下不发布此 key：** T1XB 配置文件 `modeling_app_t1xb.yaml` 中 `lixel_t1xb.camera_center.ai.fps` 默认为 0，须设备端把该项配置为大于 0 且 AI 模型就绪后才有数据；无数据时订阅仍返回 `Ok`，SDK 无法区分原因。接口定义以 [`robot_sdk_c.h`](../include/xg/robot_sdk_c.h) 的 `xg_subscribe_depth` 为准，示例见 `examples/subscribe/depth_demo.cpp`。

### 1.6 时间戳与时钟

各路数据的 `timestamp_sec` 都是 Unix epoch 秒，但**取自不同的时钟**：图像取设备系统时钟，深度图沿用图像时间戳（推断）；IMU 随平台取 MCU 时钟或雷达时钟；位姿与点云的时基未在设备代码中确认。同一台设备上它们之间不保证对齐，也不保证等于设备系统时间。

- 想知道设备系统时钟和本机差多少：调用 `getDeviceTime()`，得到偏差区间（宽度略大于往返时延）。该偏差只对图像及沿用其时间戳的深度图成立。示例见 `examples/subscribe/image_demo.cpp`（把图像时间换到本机时钟，估算收到时相对设备时间戳的延迟，区间没有计入设备端固定补偿和取帧口径）和 `examples/connection/device_info_demo.cpp`。
- 要把 IMU、位姿、点云与本机或其他传感器严格对齐：须在设备侧做时间同步，`getDeviceTime()` 不能替代。
- 每路数据取自哪个时钟、换算公式与使用限制，唯一详写处是 [api_reference_v2.md §0.8](api_reference_v2.md#08-时间戳时基与换算)。
- `getDeviceTime()` 是只读查询，SDK 不提供设置设备时间的能力；旧固件不支持时返回 `Protocol`。

---

## 2. 数据订阅使用模式

### 2.1 订阅-回调模型

所有数据订阅共享相同模式：

```cpp
// 注册回调，SDK 内部线程触发
device->subscribePose([](const xg::Pose& pose) {
    // 在此处理数据，须尽快返回
    // 如需耗时处理，拷贝数据到自有队列后立即返回
});

// 取消订阅
device->unsubscribePose();
```

关键约束：
- 回调在 SDK 内部线程触发，**不可阻塞**
- 回调中的指针（如点云 `frame.points`）仅在回调执行期间有效
- 同一设备句柄每种数据类型最多一个活跃订阅
- 不订阅时不消耗网络带宽
- `Device` 析构时自动取消所有订阅

### 2.2 完整订阅示例

```cpp
#include "xg/robot_sdk.h"
#include <cstdio>
#include <thread>
#include <chrono>

int main() {
    xg::Options opts;
    opts.deviceIp = "192.168.55.1";

    auto device = xg::Device::open(opts);
    if (!device) return 1;

    // 启动录制（数据流依赖录制状态）
    device->startRecord();

    // 等待进入录制状态
    xg::RecordStatus st;
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (device->queryRecordStatus(&st) == xg::Error::Ok &&
            st.state == xg::RecordState::Recording) break;
    }

    // 订阅位姿
    device->subscribePose([](const xg::Pose& p) {
        std::printf("pos=(%.3f,%.3f,%.3f)\n", p.posX, p.posY, p.posZ);
    });

    // 订阅设备(IMU)系 Locate 去畸变点云
    device->subscribeDeviceUndistCloud([](const xg::CloudFrame& f) {
        std::printf("cloud: %zu points\n", f.count);
    });

    std::this_thread::sleep_for(std::chrono::seconds(30));

    device->stopRecord();
    return 0;
}
```

---

## 3. 标定参数查询

### 3.1 相机标定

```cpp
xg::Device::CameraCalib cam;
if (device->getCameraCalib(xg::Device::CameraId::Left, &cam) == xg::Error::Ok) {
    printf("fx=%.2f fy=%.2f cx=%.2f cy=%.2f\n", cam.fx, cam.fy, cam.cx, cam.cy);
    printf("dist=[%.4f, %.4f, %.4f, %.4f]\n",
           cam.distortion[0], cam.distortion[1],
           cam.distortion[2], cam.distortion[3]);
    printf("model=%s\n", cam.model.c_str());  // "kb4" 或 "pinhole"
}
```

支持查询 camera_id 0~3，分别对应 Left/Center/Right 和第四颗相机（部分型号）。

### 3.2 IMU 标定

```cpp
xg::Device::ImuCalib imu;
if (device->getImuCalib(&imu) == xg::Error::Ok) {
    printf("accelBias: [%.6f, %.6f, %.6f]\n",
           imu.accelBias[0], imu.accelBias[1], imu.accelBias[2]);
    printf("gyroBias: [%.6f, %.6f, %.6f]\n",
           imu.gyroBias[0], imu.gyroBias[1], imu.gyroBias[2]);
}
```

### 3.3 雷达标定

```cpp
xg::Device::LidarCalib lidar;
if (device->getLidarCalib(&lidar) == xg::Error::Ok) {
    // T_imu_lidar: Lidar→IMU 的 4x4 变换矩阵（行优先）
    // T_cam0_lidar: Lidar→Cam0 的 4x4 变换矩阵
}
```

---

## 4. ROS2 桥接

SDK 提供 `lixel_ros` 包（源码在 `ros2/`），将 SDK 数据桥接为标准 ROS2 Topic。**话题表、坐标系、外参、参数文件的权威说明只在 [`ros2/README.md`](../ros2/README.md)**，本节只给最短路径。

### 4.1 编译

```bash
# 前提：ROS2 Humble 已安装；ros2/ 与 include/、lib/<arch>/ 平级（发布包默认如此）
cd <sdk_path>/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select lixel_ros
```

### 4.2 运行

先用 SDK 完成设备侧流程（初始化、设置初始位姿、`startRecord()`），桥接节点只订阅数据，不启停录制：

```bash
bash <sdk_path>/ros2/start_lixel_ros.sh                     # 参数全部来自 config/default_params.yaml
bash <sdk_path>/ros2/start_lixel_ros.sh device_ip:=192.168.123.103   # 只覆盖设备 IP
```

### 4.3 发布的 Topic 与 TF

| Topic | 消息类型 | 说明 |
|-------|---------|------|
| `/odom` | nav_msgs/Odometry | `map → base_link`（已叠加外参，`base_link` 为 ROS 直出的 ODOM 坐标系） |
| `/tf` | tf2_msgs/TFMessage | 动态 TF `map → imu_link` |
| `/tf_static` | tf2_msgs/TFMessage | 静态 TF `imu_link → base_link`（外参，启动时发一次） |
| `/lixel/device_undist_cloud` | sensor_msgs/PointCloud2 | 设备(IMU)系 Locate 去畸变点云，frame `imu_link` |
| `/lixel/imu` | sensor_msgs/Imu | 原始 IMU |
| `/lixel/camera/*/compressed` | sensor_msgs/CompressedImage | 相机图像，帧率以设备配置为准 |

频率、frame 定义与外参换算见 [`ros2/README.md`](../ros2/README.md)。

### 4.6 RViz2 可视化

```bash
rviz2
# Fixed Frame: map
# 添加 TF / Odometry (/odom) / PointCloud2 (/lixel/device_undist_cloud)
```

---

## 5. 测试验证

### 5.1 网络检查

```bash
ping <设备IP>
nc -zv <设备IP> 7448    # modeling_app（SDK 唯一需要的端口）
```

### 5.2 SDK 直接测试

```bash
# 录制控制测试（预编译产物在 bin/<arch>/<分类>/，以下以 x86_64 为例）
./bin/x86_64/record/record_demo <设备IP> 10

# 数据订阅测试
./bin/x86_64/subscribe/subscribe_demo <设备IP> [输出目录|-] [X] [Y] [Z]   # 自带 setLocInitPose，X/Y/Z 为地图系初值(m)，默认 0 0 0

# 全功能测试
./bin/x86_64/comprehensive/all_features_demo <设备IP> [订阅秒=15] [X=0] [Y=0] [Z=0]   # 自带 setLocInitPose，X/Y/Z 为地图系初值(m)，默认 0 0 0；默认值在本机地图上可能重定位失败，请传设备实际所在附近的坐标
```

### 5.3 设备端验证（SSH 到设备）

```bash
# 查看 zenoh 配置和工具
cat /opt/xgrids/robot/app/install/etc/zenoh_robot.json5

# 查询录制状态
echo '{"mode":"peer","connect":{"endpoints":["tcp://127.0.0.1:7448"]}}' > /tmp/z.json
ZENOH_ROBOT_CONFIG=/tmp/z.json \
  /opt/xgrids/robot/app/install/bin/zenoh get robot_cmd \
  '{"method":"checkRecordStatus"}'

# 订阅位姿（验证设备端是否在发数据）
ZENOH_ROBOT_CONFIG=/tmp/z.json \
  timeout 5 /opt/xgrids/robot/app/install/bin/zenoh sub body_locate
```

### 5.4 ROS2 验证

```bash
ros2 topic list
ros2 topic hz /odom         # 预期 ~100Hz
ros2 topic echo /odom       # 查看数据
ros2 run tf2_tools view_frames  # 查看 TF 树
```

---

## 6. 排障

| 现象 | 原因 | 解决 |
|------|------|------|
| `open()` 超时 | 网络不通或端口不可达 | `ping` + `nc -zv <ip> 7448` |
| 连接成功但位姿/点云为 0 帧 | 设备不在录制状态 | 先 `startRecord()` 并等待进入 Recording |
| 位姿正常但点云为 0 | 重定位未运行 | 设备需加载先验地图 |
| 所有订阅数据为 0（含位姿） | 设备端 Publisher 未启动 | 确认 7448 端口在监听（`ss -tlnp \| grep 7448`）、modeling_app 已运行 |
| 图像订阅为 0 | JPEG 发布未启用 | 确认设备端固件版本支持图像 zenoh 发布 |
| `startRecord` 返回 `Timeout` | 命令可能已送达 | 不要重试，先 `queryRecordStatus()` 确认 |
| 状态一直 `Error: device not ready` | 设备服务未初始化完成 | 等待数十秒后重试 |
| ROS2 topic 无数据 | Bridge 未连接或设备未录制 | 检查 bridge 终端日志 |
| `liblixel_sdk.so not found` | 库路径未设 | `export LD_LIBRARY_PATH=.../lib:$LD_LIBRARY_PATH` |

### 新设备端口变化的诊断

部分新设备（新版固件）的进程架构可能与老设备不同。确认方法：

```bash
# SSH 到设备
ss -tlnp | grep 7448
# 预期：7448 在监听（modeling_app）。SDK 只需这一个端口

# 查看进程
ps aux | grep modeling
systemctl list-units --type=service | grep xgrids
```

---

## 7. 性能参考

| 数据类型 | 单帧大小 | 频率 | 带宽 |
|---------|---------|------|------|
| 位姿 | ~200B | 100Hz | ~20 KB/s |
| 去畸变点云 | ~1-3MB | 10Hz | ~10-30 MB/s |
| IMU | ~50B | 200Hz | ~10 KB/s |
| 相机 JPEG | ~50-200KB | 10Hz/路 | ~1-2 MB/s/路 |

---

## 8. 接口版本变更记录

当前版本（1.1.0）的接口变更见 [releases/v1.1.0.md](releases/v1.1.0.md)；只保留最新版本的发布说明，历史版本见 git 历史。
