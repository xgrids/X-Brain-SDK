# XGRIDS Robot SDK 集成指南

本文档面向集成方开发人员，提供 SDK 使用的完整参考，包括坐标系概念、数据订阅、ROS2 桥接、测试验证与排障。

基础接口说明（连接、录制启停、编译链接）请参阅根目录 [README.md](../README.md)。完整 C/C++ 契约见 [api_reference_v2.md](api_reference_v2.md)，各版 Release Note 见 [releases/](releases/README.md)。

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

`subscribeCloud()` 回调输出的是设备（IMU）系点云（~10Hz），来自重定位算法：

| 字段 | 单位 | 说明 |
|------|------|------|
| x, y, z | 米 | 点在设备（IMU）系下的坐标 |
| intensity | — | 反射强度 |

注意：该点云不是每帧 LiDAR 完整扫描，而是重定位算法配准处理后的结果，频率约 10Hz。每帧去畸变原始扫描需通过 `subscribeRawCloud()`（~10Hz）获取。

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

    // 订阅点云
    device->subscribeCloud([](const xg::CloudFrame& f) {
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

SDK 提供 `lixel_sdk_ros2` 包，将 SDK 数据桥接为标准 ROS2 Topic。

### 4.1 编译

```bash
# 前提：ROS2 Humble 已安装，SDK 已编译
mkdir -p ~/ros2_ws/src
ln -sf <sdk_path>/ros2 ~/ros2_ws/src/lixel_sdk_ros2

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select lixel_sdk_ros2
source install/setup.bash
```

### 4.2 运行

```bash
# 设置 SDK 库路径
export LD_LIBRARY_PATH=<sdk_path>/build-x86_64:$LD_LIBRARY_PATH

# 运行
ros2 run lixel_sdk_ros2 sdk_bridge_node --ros-args -p device_ip:=192.168.55.1

# 或使用 launch
ros2 launch lixel_sdk_ros2 sdk_bridge.launch.py device_ip:=192.168.55.1
```

### 4.3 发布的 Topic

| Topic | 消息类型 | 频率 | 说明 |
|-------|---------|------|------|
| `/odom` | nav_msgs/Odometry | ~100Hz | 位姿（含外参变换） |
| `/tf` | tf2_msgs/TFMessage | ~100Hz | 动态 TF (odom → sensor_link) |
| `/tf_static` | tf2_msgs/TFMessage | 启动一次 | 静态 TF (sensor_link → base_link) |
| `/cloud_points` | sensor_msgs/PointCloud2 | ~10Hz | 点云 |
| `/lixel/camera/*/compressed` | sensor_msgs/CompressedImage | ~2fps | 相机图像 |

### 4.4 TF 树

```
odom
 └── sensor_link  (动态，~100Hz)
      └── base_link  (静态，外参)
```

### 4.5 外参配置

编辑 `ros2/config/default_params.yaml`：

```yaml
sdk_bridge_node:
  ros__parameters:
    device_ip: "192.168.55.1"
    static_tf:
      translation: [-0.29242062, 0.0, -0.41988808]       # sensor→base 位移 (m)
      rotation_quat: [0.0, -0.17395643, 0.0, 0.98475335] # [x, y, z, w]
    use_omega_cross_r: true  # 速度变换含杆臂补偿
```

外参含义：`translation` 是 base_link 原点在 sensor_link 系中的位移，`rotation_quat` 是 sensor→base 的旋转。

### 4.6 RViz2 可视化

```bash
rviz2
# Fixed Frame: odom
# 添加 TF / Odometry (/odom) / PointCloud2 (/cloud_points)
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
# 录制控制测试
./record_demo <设备IP> 10

# 数据订阅测试
./subscribe_demo <设备IP>

# 全功能测试
./all_features_demo <设备IP>
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

部分新设备（如 V3.3.x 固件）的进程架构可能与老设备不同。确认方法：

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
| 重定位点云 | ~100KB | 10Hz | ~1 MB/s |
| 原始点云 | ~1-3MB | 10Hz | ~10-30 MB/s |
| IMU | ~50B | 200Hz | ~10 KB/s |
| 相机 JPEG | ~50-200KB | 10Hz/路 | ~1-2 MB/s/路 |

---

## 8. 接口版本变更记录

| 版本 | 新增接口 |
|------|---------|
| v0.1.0 | 录制启停、状态查询 |
| v0.2.0 | 位姿订阅、重定位点云订阅 |
| v0.3.1 | IMU 订阅、原始点云订阅、相机图像订阅、标定参数查询、设备信息查询、初始位姿设置 |
