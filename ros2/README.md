# lixel_sdk_ros2 — ROS2 Bridge

将 XGRIDS 设备的位姿和点云数据桥接为标准 ROS2 Topic。

## 依赖

- ROS2 Humble（Ubuntu 22.04）或 Foxy（Ubuntu 20.04）
- lixel_sdk 核心库（`lib/<arch>/liblixel_sdk.so`，`<arch>` 为 x86_64 或 aarch64）
- Eigen3

## 编译

```bash
# 创建 workspace 并链接本目录
mkdir -p ~/ros2_ws/src
ln -sf <本目录绝对路径> ~/ros2_ws/src/lixel_sdk_ros2

# 编译
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select lixel_sdk_ros2
```

## 运行

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=<SDK lib 目录>:$LD_LIBRARY_PATH

# 方式 1: 直接运行
ros2 run lixel_sdk_ros2 sdk_bridge_node --ros-args -p device_ip:=192.168.55.1

# 方式 2: launch 文件
ros2 launch lixel_sdk_ros2 sdk_bridge.launch.py device_ip:=192.168.55.1
```

## 发布的 Topic

| Topic | 消息类型 | 频率 | 说明 |
|-------|---------|------|------|
| `/lixel/odom` | nav_msgs/Odometry | ~100Hz | 设备（IMU）系位姿，`map → base_link`，无外参变换 |
| `/lixel/tf` | tf2_msgs/TFMessage | ~100Hz | `map → base_link` 动态变换 |
| `/lixel/cloud_points` | sensor_msgs/PointCloud2 | ~1Hz | 设备（IMU）系 reloc 点云（XYZI, float32），frame=base_link |
| `/lixel/imu` | sensor_msgs/Imu | ~200Hz | 原始 IMU（加速度 + 角速度），frame=base_link |
| `/lixel/raw_cloud` | sensor_msgs/PointCloud2 | ~10Hz | 去畸变原始点云，frame=base_link |
| `/lixel/camera/left/compressed` | sensor_msgs/CompressedImage | ~2fps | 左相机 JPEG |
| `/lixel/camera/center/compressed` | sensor_msgs/CompressedImage | ~2fps | 中相机 JPEG |
| `/lixel/camera/right/compressed` | sensor_msgs/CompressedImage | ~2fps | 右相机 JPEG |
| `/lixel/diagnostics` | diagnostic_msgs/DiagnosticArray | ~10Hz | 设备错误码 |

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| device_ip | "192.168.123.103" | 设备 IP |
| device_port | 7448 | modeling_app 端口（1.0.0 起 SDK 只连此单端点） |
| connect_timeout_ms | 5000 | 连接超时 (ms) |
| map_frame | "map" | 世界/地图坐标系（SLAM/重定位全局位姿，参见 REP-105） |
| base_frame | "base_link" | 设备基座坐标系，**等于设备 IMU 系** |

参数可通过 `config/default_params.yaml` 或命令行 `--ros-args -p key:=value` 传入。

> SDK 订阅 `device_locate` / `reloc_device_cloud`，位姿与点云本身即设备（IMU）系，桥接不做外参变换。

## 行为说明

- 启动时自动连接设备并启动录制
- 等待设备进入录制状态后开始订阅数据
- Ctrl+C 退出时自动停止录制并断开连接
- 位姿与点云直接以设备（IMU）系发布，`base_link` 即设备 IMU 基准，**不做任何外参变换**

## 坐标系

TF 结构极简，仅两层：

```
map ──(动态, ~100Hz)──► base_link
 └ 世界/地图系            └ 设备(IMU)基准
```

- 设备端 SDK 输出的 `device_locate` / `reloc_device_cloud` 本身即设备（IMU）系位姿/点云，桥接直接落到 `base_link`。
- 若设备安装在载体（机器狗/车等）上，需要载体中心位姿时，由集成方在 `base_link` 之下自行发布一层静态 TF（设备 → 载体）。这与竞品做法一致，也符合"设备只输出设备系、载体变换归集成方"的约定。
- 坐标轴遵循 ROS REP-103（右手系，x 前 / y 左 / z 上）；frame 语义遵循 REP-105（`map` 全局无漂移、`base_link` 固连基座）。详见坐标系规范文档。

## 前提条件

- 设备与上位机处于同一网段且 7448 端口可达
- 设备在录制状态下才发布位姿数据（bridge node 会自动启动录制）
- 点云数据需要设备在有图模式下且重定位成功后才会发布
