# lixel_sdk 1.1.0 接口说明书

**文档等级：** 对外 ICD（接口控制文件）  
**适用版本：** `XG_SDK_VERSION_*` = 1.1.0，`xg_sdk_version()` = `"1.1.0"`  
**ABI 真源：** [`include/xg/robot_sdk_c.h`](../include/xg/robot_sdk_c.h)  
**C++ 封装：** [`include/xg/robot_sdk.h`](../include/xg/robot_sdk.h)（header-only，全部 inline）

集成入门见 [README.md](../README.md)。各版差异见 [releases/](releases/)。

> **本文组织方式：** 全文分两大板块——**第一板块「数据结构」（第 1~4 章）** 只描述结构体长什么样（源码 + 成员表 + 字节布局），不含调用语义；**第二板块「接口」（第 5~12 章）** 描述每个接口的作用、参数、返回值，以及回调指针/变量的内存与生命周期。第 0 章通则是两板块共同的地基。附录（第 13~15 章）为速查表与修订记录。

---

## 0. 通则

### 0.1 错误模型

所有会失败的接口以返回值报错，**不抛异常**。C++ `xg::Error` 与 C `xg_err_t` 数值一一对应，不得改已发布数值，只允许追加。

| C | C++ | 值 | 含义与集成方动作 |
|---|-----|----|------------------|
| `XG_OK` | `Error::Ok` | 0 | 成功。启停时只表示**命令已接受**，不表示状态已到达 |
| `XG_ERR_INVALID_ARG` | `InvalidArg` | -1 | 空指针、`struct_size` 不匹配、IP 空/过长、相机枚举越界。修正参数后重试 |
| `XG_ERR_NOT_CONNECTED` | `NotConnected` | -2 | 未连接（当前多数路径在 open 失败时不会得到已分配句柄） |
| `XG_ERR_TIMEOUT` | `Timeout` | -3 | 未在超时内收到设备应答。**命令是否送达未知。禁止对启停直接重发**，先 `queryRecordStatus` |
| `XG_ERR_REJECTED` | `Rejected` | -4 | 设备拒绝，或订阅槽已被占。查询状态或先 unsubscribe |
| `XG_ERR_DEVICE_ERROR` | `DeviceError` | -5 | 设备故障态（头文件保留；录制路径更多通过 `RecordState::Error` 表达） |
| `XG_ERR_PROTOCOL` | `Protocol` | -6 | 应答无法解析，通常为固件/SDK 协议不匹配 |
| `XG_ERR_INTERNAL` | `Internal` | -7 | SDK 内部（如 declare_subscriber 失败） |

`xg_err_str` / `xg::toString(Error)` 返回静态 UTF-8 字符串，不必释放。

### 0.2 `struct_size`

C 侧由**调用方分配**的结构（`xg_options_t`、`xg_record_status_t`、各 `*_calib_t`、`xg_device_info_t`、`xg_device_time_t`、`xg_init_pose_t`、`xg_data_path_t`）必须先置 `struct_size = sizeof(...)`，否则 `InvalidArg`。

由 **SDK 填写**的回调结构（`xg_pose_t`、`xg_cloud_frame_t`、`xg_raw_cloud_frame_t`、`xg_imu_t`、`xg_image_frame_t`、`xg_err_code_frame_t`）的 `struct_size` 由 SDK 写入，调用方只读。

当前实现要求 **精确相等**，不是「大于等于即可」。C++ 封装已代填。

### 0.3 线程与寿命

- 同一 `xg_device_t` / `Device` **不可重入**。多线程必须由调用方加锁。不同实例互不影响。
- 订阅回调在 **SDK 内部线程**（Zenoh 会话线程）触发，不在调用 `subscribe*` 的线程。
- 回调内禁止阻塞、禁止再调同一句柄的 SDK 接口。
- 回调里的指针（`points`、`jpeg_data`、C 的 `entries`）**返回后不可访问**。需要持久化必须在回调内拷贝。
- `Device` 禁止拷贝与移动。析构时取消全部订阅并 `xg_device_close`。

### 0.4 连接拓扑

`Options.port`（默认 7448）**直接是 modeling_app 的端口**。SDK 只连 modeling_app 一个端点：数据面（`device_locate`/`raw_imu`/`device_undist_cloud`/`device_raw_cloud`/`camera/*`/`depth/*`/`err_code`）与命令面（`robot_cmd` Get）都在 modeling_app 的同一 session。详见 README「网络要求」。

### 0.5 C++ 与 C 对照约定

| C++ | C |
|-----|---|
| `Device::open` → `unique_ptr`，失败 `nullptr` | `xg_device_open`，失败 `*out_dev = NULL` |
| `Error` 枚举 | `xg_err_t` |
| `std::string` / `std::function` | 仅存在于 header-only 侧，不跨 `.so` |
| 回调里 POD 已拷贝标量；点云/JPEG 指针仍指向 SDK 缓冲 | 同左 |

### 0.6 命名规范来源（为何 C 与 C++ 字段名不同）

同一份数据在 C 头与 C++ 头里字段名不同（如 `timestamp_sec` vs `timestampSec`），这不是笔误，而是遵循 scanner_app 编码规范（`coding-style` 第 2.3、2.4 节）对两种语言的**不同强制要求**：

- **C 结构体成员**：下划线命名（snake_case），如 `xg_pose_t::pos_x`。
- **C++ struct 公有成员**：小驼峰（camelCase），如 `Pose::posX`。
- **C 函数**用 snake_case（`xg_record_start`），**C++ 方法**用 camelCase（`Device::startRecord`）。

因此 C++ 封装无法直接 `using Pose = xg_pose_t;` 复用 C 结构体（那会把 snake_case 泄漏进 C++ API，违反规范），必须造一份 camelCase 的镜像并逐字段搬运。这也是 C↔C++ 存在「平行结构体」的根因之一。

### 0.7 内存布局与所有权模型

所有对外结构体按**内存所有权**分两类，混淆二者是集成方最易犯的错误。

#### 0.7.1 调用方分配型（同步查询 / 入参）

`xg_options_t`、`xg_record_status_t`、`xg_camera_calib_t`、`xg_imu_calib_t`、`xg_lidar_calib_t`、`xg_device_info_t`、`xg_init_pose_t`、`xg_data_path_t`。

- **内存由调用方分配**（栈上或堆上均可），生命周期由调用方掌控。
- 调用前必须置 `struct_size = sizeof(...)`，否则返回 `InvalidArg`（当前实现要求**精确相等**，不是「≥ 即可」）。
- 这些结构体**内部没有 SDK 拥有的指针**，字符串是定长内嵌数组（如 `char message[256]`、`char path[512]`），SDK 只写内容不接管内存。
- **调用方无需、也不允许对这些结构体或其字段调用 `free` / `delete`**——里面没有 SDK 分配的堆内存，随你自己的变量销毁即可。
- C++ 封装（`RecordStatus`、`CameraCalib` 等）已代填 `struct_size` 并把定长字段转成 `std::string`，由 C++ 对象自行管理。

#### 0.7.2 SDK 填充型（订阅回调）

`xg_pose_t`、`xg_imu_t`、`xg_cloud_frame_t`、`xg_image_frame_t`、`xg_err_code_frame_t`。

- 结构体本身与其内部指针（`points` / `jpeg_data` / `entries`）**全部由 SDK 拥有**，`struct_size` 由 SDK 写入，调用方只读。
- 变长数据（点数组、JPEG 码流、错误码数组）指向 **SDK 内部的 `thread_local` 复用缓冲区**（见 `subscription.cpp`）：
  - 指针**仅在本次回调执行期间有效**；
  - 回调返回后，**下一帧会覆盖同一块缓冲**（不是释放，是复用改写）；
  - 因此跨回调持有该指针 = 悬垂 / 被改写，属未定义行为。
- **调用方内存管理铁律**：
  1. 回调内**绝不保存裸指针**。要留数据必须在回调内**深拷贝**：定长标量直接按值存；变长数据用 `memcpy`（点云 / JPEG）或 `std::vector::assign`（错误码）拷到自己的缓冲。
  2. **不要对回调里的任何指针调用 `free` / `delete`**——不是你分配的。
  3. 回调**不可阻塞**（会拖住 SDK 的 Zenoh 会话线程，影响所有订阅）。要做重活，拷贝后转交自己的线程。
- C++ 封装的差异：
  - `Pose`、`Imu` 是纯标量，桥接时**已整体按值拷贝**给你，回调参数本身可安全按值再存一份；
  - `CloudFrame`、`ImageFrame` 的 `points` / `jpegData` **仍是指向 SDK 缓冲的裸指针**，规则同 C，须回调内拷贝；
  - `ErrCodeFrame::entries` 是**已拷贝好的 `std::vector`**（桥接层已从 C 缓冲拷出），但该 vector 随回调参数对象销毁，**跨回调持有仍需再拷一份**。

### 0.8 时间戳时基与换算

本节是时间戳语义的唯一详写处，其余章节只链接到这里。

**文档性质与依据：** 设备端取时钟的来源依据固件源码静态阅读得出，**未上板实测**。标「推断」的是由调用关系得出、没有直接看到赋值的结论，标「未查到」的是读不到实现（预编译算法库、相机驱动库、MCU 固件）。

**1. 数值换算（SDK 侧，确定）**

SDK 不对任何时间戳加减偏移，只做量纲换算：

| 来源 | 线上类型 | SDK 换算 | 得到 |
|------|----------|----------|------|
| 位姿、点云、原始点云、IMU（protobuf `header.stamp`） | int64，微秒 | `÷ 1e6` | `timestamp_sec`（double，秒） |
| 图像、深度图（数据体前 8 字节，小端） | uint64，微秒 | `× 1e-6` | `timestamp_sec`（double，秒） |
| 错误码条目 | uint64，微秒 | 不换算 | `entries[i].timestamp`（微秒） |

所有数值都按「自 1970-01-01 00:00:00 UTC 起的 Unix epoch」解读，与 `xg_device_time_t.device_epoch_us`（int64，微秒）**同量纲**。还原为线上微秒用 `llround(timestamp_sec × 1e6)`。epoch 秒值在 double 里的分辨率约 0.24 µs；在 1.7e15 到 1.8e15 µs 区间随机取 200 万个整数，`÷1e6` 与 `×1e-6` 两种换算再用 `llround` 还原，与原值均零差异。

**2. 同量纲不等于同一个时钟（设备侧，按平台而异）**

设备端对每路数据**不做发布层偏移**，但各路 epoch 值取自不同的时钟：

| 数据 | SDK 字段 | 时钟来源 | 能否直接与 `device_epoch_us` 相减 |
|------|----------|----------|------------------------------------|
| 图像 `camera/*` | `ImageFrame.timestampSec` | 设备系统时钟（`CLOCK_REALTIME`）：取帧时把驱动缓冲时间映射为 epoch，再叠加设备配置的固定补偿（当前配置值在 0 到 9.21 ms 之间，随平台不同）。取帧口径（帧起 / 帧止 / 曝光中点）未查到 | 可以，误差为固定补偿加口径不确定 |
| 深度图 `depth/center` | `DepthFrame.timestampSec` | 沿用中间相机图像的时间戳（推断） | 同图像 |
| 错误码 `err_code` | `entries[i].timestamp` | **同一条消息内混合**：建模 / 系统类取设备系统时钟；SLAM 类取算法状态时间（传感器时基）；定位库类时基未查到 | 无法逐条区分，不可用于对时 |
| IMU `raw_imu` | `Imu.timestampSec` | 随平台：使用 MCU 串口 IMU 的平台取 **MCU 时钟**（原样使用 MCU 打戳）；使用雷达内置 IMU 的平台取**雷达时钟再减固定 9.5 ms** | 不保证 |
| 位姿 `device_locate` | `Pose.timestampSec` | 定位算法输入的 IMU / 点云时基，算法库内部赋值未查到（推断同上一行） | 不保证 |
| 去畸变点云 `device_undist_cloud` | `CloudFrame.timestampSec` | Locate 输出帧时基（`UndistortCloudType.timestamp`，时基未验证）；对应帧首、帧尾还是中间未查到 | 不保证 |
| 原始点云 `device_raw_cloud` | `RawCloudFrame.timestampSec` | **雷达自身时钟**（RoboSense 数据包头里的 UTC 时间），帧内最早点 | 不保证 |

设备端唯一的粗对齐保证：RoboSense 雷达帧在进入处理前会与系统时间比较，雷达帧时间不在「系统时间之前 0 到 10 s」内的帧被丢弃，这个比较发生在雷达回调入口，所以进入处理的雷达帧，其帧时间在该时刻的设备系统时间之前 0 到 10 s 内；帧到达 SDK 时还要再加上设备端处理、队列和网络的时延，上界不再是 10 s。**这不是对时精度。**

MCU 平台上，MCU 时钟由设备在启动、每次开始扫描时用系统时钟授时一次，授时之间 MCU 自走。雷达平台上雷达时钟与系统时钟是否由 PTP 对齐，在设备端代码里未查到配置，也没有对外暴露的同步状态位。

**3. 把设备时钟换到本机时钟**

设备采样 `device_epoch_us` 的时刻必然落在本机 `[host_send_epoch_us, host_recv_epoch_us]` 之内，所以

```
offset = 设备系统时钟 - 本机时钟
device_epoch_us - host_recv_epoch_us  <=  offset  <=  device_epoch_us - host_send_epoch_us
```

区间宽度等于 `host_recv_epoch_us - host_send_epoch_us`，比 `rtt_us` 多出几次时钟读取的开销（微秒量级），**不依赖去程与回程对称**。`DeviceTime` 提供 `offsetLowerUs()` / `offsetUpperUs()` / `offsetMidUs()`，中点只是粗略估计。

对取自**设备系统时钟**的数据（图像、深度图），本机时刻为：

```
t_host_us = timestamp_us - offset        // offset 取区间内的值，不确定度 = 区间宽度
```

对 IMU、位姿、点云等**不保证取自设备系统时钟**的数据，`offset` 对它们不成立，SDK 与设备都不提供它们相对设备系统时钟的偏差。需要把这些数据与本机或其他传感器严格对齐时，须在设备侧做时间同步（MCU 的 PPS 或雷达 PTP），本接口不能替代。

**4. 使用限制**

- 本机 `system_clock` 在查询期间被调整（NTP 跳变等）时，区间无意义。
- 设备系统时钟在运行中会被调整：手机 / App 设置时间、MCU RTC 回写、配置了 PPS 的设备在开始扫描时启动 chrony 校准。调整之后的 `device_epoch_us` 与之前收到的图像时间戳不在同一个连续时间轴上，**每次要换算前重新查询**，并建议在开始录制之后再查。
- SDK 只提供只读查询，不提供设置设备时间的能力。

---

# 第一板块 · 数据结构

本板块只描述 SDK 对外结构体的**数据形态**：源码（C 与 C++）、成员含义、字节布局。**如何获取这些数据、指针能活多久，见第二板块「接口」。**

## 1. 数据结构总览

| 结构体（C / C++） | 类别 | 关联接口 | 详见 |
|-------------------|------|----------|------|
| `xg_err_t` / `Error` | 枚举 | 所有接口返回值 | [2.1](#21-error--xg_err_t) |
| `xg_record_state_t` / `RecordState` | 枚举 | `queryRecordStatus` | [2.2](#22-recordstate--xg_record_state_t) |
| `xg_record_status_t` / `RecordStatus` | 调用方分配型 | `queryRecordStatus` | [2.3](#23-recordstatus--xg_record_status_t) |
| `xg_options_t` / `Options` | 调用方分配型 | `open` | [2.4](#24-options--xg_options_t) |
| `xg_pose_t` / `Pose` | SDK 填充型 | `subscribePose` | [3.1](#31-位姿-pose) |
| `xg_point_t` | SDK 填充型（点元素） | `subscribeDeviceUndistCloud` | [3.2](#32-单点-point) |
| `xg_cloud_frame_t` / `CloudFrame` | SDK 填充型 | `subscribeDeviceUndistCloud` | [3.3](#33-点云帧-cloudframe) |
| `xg_raw_point_t` / `xg_raw_cloud_frame_t` / `RawCloudFrame` | SDK 填充型（点元素 + 帧） | `subscribeDeviceRawCloud` | [3.3.1](#331-原始点云-rawcloudframe) |
| `xg_imu_t` / `Imu` | SDK 填充型 | `subscribeImu` | [3.4](#34-惯导-imu) |
| `xg_camera_id_t` / `CameraId` | 枚举 | `subscribeImage` | [3.5](#35-相机标识-cameraid) |
| `xg_image_frame_t` / `ImageFrame` | SDK 填充型 | `subscribeImage` | [3.6](#36-图像帧-imageframe) |
| `xg_err_code_entry_t` / `ErrCodeEntry` | SDK 填充型（条目） | `subscribeErrCode` | [3.7](#37-错误码-errcodeframe) |
| `xg_err_code_frame_t` / `ErrCodeFrame` | SDK 填充型 | `subscribeErrCode` | [3.7](#37-错误码-errcodeframe) |
| `xg_camera_calib_t` / `CameraCalib` | 调用方分配型 | `getCameraCalib` | [4.1](#41-相机标定-cameracalib) |
| `xg_imu_calib_t` / `ImuCalib` | 调用方分配型 | `getImuCalib` | [4.2](#42-imu-标定-imucalib) |
| `xg_lidar_calib_t` / `LidarCalib` | 调用方分配型 | `getLidarCalib` | [4.3](#43-雷达标定-lidarcalib) |
| `xg_device_info_t` / `DeviceInfo` | 调用方分配型 | `getDeviceInfo` | [4.4](#44-设备状态-deviceinfo) |
| `xg_device_time_t` / `DeviceTime` | 调用方分配型 | `getDeviceTime` | [4.4.1](#441-设备时间-devicetime) |
| `xg_init_pose_t` / `InitPose` | 调用方分配型（入参） | `setLocInitPose` | [4.5](#45-定位初值-initpose) |
| `xg_data_path_t` | 调用方分配型 | `getDataPath` | [4.6](#46-工程路径-xg_data_path_t) |

两类内存所有权的总则见 [0.7](#07-内存布局与所有权模型)；C/C++ 命名差异见 [0.6](#06-命名规范来源为何-c-与-c-字段名不同)。

---

## 2. 基础与状态类型

### 2.1 `Error` / `xg_err_t`

**C（`robot_sdk_c.h`）**
```c
typedef enum {
    XG_OK = 0,                 /**< 成功 */
    XG_ERR_INVALID_ARG = -1,   /**< 参数非法（空指针、struct_size 不匹配等） */
    XG_ERR_NOT_CONNECTED = -2, /**< 未连接到设备 */
    XG_ERR_TIMEOUT = -3,       /**< 超时。启停录制时表示未在限定时间内到达目标状态 */
    XG_ERR_REJECTED = -4,      /**< 设备拒绝执行，当前状态不允许该操作 */
    XG_ERR_DEVICE_ERROR = -5,  /**< 设备处于故障状态，详见 xg_record_status_t */
    XG_ERR_PROTOCOL = -6,      /**< 设备应答无法解析，通常为版本不匹配 */
    XG_ERR_INTERNAL = -7,      /**< SDK 内部错误 */
} xg_err_t;
```

**C++（`robot_sdk.h`）**
```cpp
enum class Error {
    Ok = XG_OK,                         /**< 成功 */
    InvalidArg = XG_ERR_INVALID_ARG,    /**< 参数非法 */
    NotConnected = XG_ERR_NOT_CONNECTED,/**< 未连接到设备 */
    Timeout = XG_ERR_TIMEOUT,           /**< 超时 */
    Rejected = XG_ERR_REJECTED,         /**< 设备拒绝执行，当前状态不允许该操作 */
    DeviceError = XG_ERR_DEVICE_ERROR,  /**< 设备处于故障状态 */
    Protocol = XG_ERR_PROTOCOL,         /**< 设备应答无法解析 */
    Internal = XG_ERR_INTERNAL,         /**< SDK 内部错误 */
};
```

**成员说明**：数值锁定，一经发布不得变更，只允许追加。各值的详细含义与集成方动作见 [0.1 错误模型](#01-错误模型)。`xg_err_str` / `xg::toString(Error)` 返回静态 UTF-8 字符串。

### 2.2 `RecordState` / `xg_record_state_t`

**C（`robot_sdk_c.h`）**
```c
typedef enum {
    XG_REC_IDLE = 0,      /**< 待机，可以开始录制 */
    XG_REC_STARTING = 1,  /**< 启动中，尚未开始出数据 */
    XG_REC_RECORDING = 2, /**< 录制中 */
    XG_REC_STOPPING = 3,  /**< 停止中 */
    XG_REC_ERROR = 4,     /**< 故障，详见 message 字段 */
} xg_record_state_t;
```

**C++（`robot_sdk.h`）**
```cpp
enum class RecordState {
    Idle = XG_REC_IDLE,           /**< 待机，可以开始录制 */
    Starting = XG_REC_STARTING,   /**< 启动中，尚未开始出数据 */
    Recording = XG_REC_RECORDING, /**< 录制中 */
    Stopping = XG_REC_STOPPING,   /**< 停止中 */
    Error = XG_REC_ERROR,         /**< 故障，详见 message */
};
```

**成员说明**：设备内部 `ModelingState` 远多于此，SDK 归并为五态。业务只判断 `state` 与 `warning`，不要解析 `rawCode`。

| C | C++ | 值 | 含义 |
|---|-----|----|------|
| `XG_REC_IDLE` | `Idle` | 0 | 待机，可开录；或上次录制已停 |
| `XG_REC_STARTING` | `Starting` | 1 | 启动中，尚未出数据，继续等 |
| `XG_REC_RECORDING` | `Recording` | 2 | 录制中 |
| `XG_REC_STOPPING` | `Stopping` | 3 | 停止中（落盘） |
| `XG_REC_ERROR` | `Error` | 4 | 故障，见 `message`，应中止等待 |

`xg_record_state_str` / `toString(RecordState)`：静态字符串。

### 2.3 `RecordStatus` / `xg_record_status_t`

**C（`robot_sdk_c.h`）**
```c
typedef struct {
    size_t struct_size; /**< 必须由调用方填写为 sizeof(xg_record_status_t) */

    xg_record_state_t state; /**< 归并后的对外状态 */

    /**
     * 告警标志。为 1 时表示存在需要关注的异常，但不影响当前 state 所
     * 允许的操作。典型情形：
     *   - state 为 XG_REC_RECORDING：建图过程异常、已退化为仅录包模式，
     *     录制仍在继续，可正常停止；
     *   - state 为 XG_REC_IDLE：上一次工程的后处理存在异常，设备本身可用。
     * 与 XG_REC_ERROR 的区别是：告警不阻塞操作，故障状态则阻塞。
     */
    uint8_t warning;

    uint8_t reserved[3]; /**< 对齐填充，保留 */

    /**
     * 设备原始状态码，仅供排障与日志记录。
     * 该值反映设备内部状态机，不保证跨版本稳定。
     */
    int32_t raw_code;

    char message[XG_MESSAGE_MAX]; /**< 人可读说明，UTF-8，始终以 '\0' 结尾 */
} xg_record_status_t;
```

**C++（`robot_sdk.h`）**
```cpp
struct RecordStatus {
    RecordState state{RecordState::Idle}; /**< 归并后的对外状态 */
    bool warning{false};                  /**< 存在需关注的异常，但不阻塞当前操作 */
    std::int32_t rawCode{0};              /**< 设备原始状态码，仅供排障 */
    std::string message{""};              /**< 人可读说明，UTF-8 */
};
```

**成员说明**

| C 字段 | C++ | 类型 | 说明 |
|--------|-----|------|------|
| `struct_size` | （封装填写） | `size_t` | 调用前必须 = `sizeof` |
| `state` | `state` | 枚举 | 归并态 |
| `warning` | `warning` | `uint8_t` / `bool` | 1/true = 有告警但不阻塞当前 `state` 允许的操作 |
| `reserved[3]` | — | | 对齐，勿用 |
| `raw_code` | `rawCode` | `int32_t` | 设备原码，仅日志，跨版本不稳 |
| `message[256]` | `message` | UTF-8 | 始终以 `'\0'` 结尾 |

`warning` 典型：`Recording` + 建图异常（仅录包，仍可停）；`Idle` + 上次工程后处理异常（设备仍可用）。`Error` 态阻塞开录。

> 内存所有权属「调用方分配型」（[0.7.1](#071-调用方分配型同步查询--入参)）：无堆指针，`message` 为定长内嵌数组，无需 `free`/`delete`。查询语义见 [§6.3 queryRecordStatus](#63-queryrecordstatus--xg_record_query)。

### 2.4 `Options` / `xg_options_t`

**C（`robot_sdk_c.h`）**
```c
typedef struct {
    size_t struct_size; /**< 必须由调用方填写为 sizeof(xg_options_t) */

    char device_ip[XG_IP_MAX]; /**< 设备 IP，必填。当前要求与设备同一交换机直连 */

    uint16_t port;     /**< 设备端口，0 表示使用 XG_DEFAULT_PORT */
    uint16_t reserved; /**< 对齐填充，保留 */

    uint32_t connect_timeout_ms; /**< 连接超时，0 表示使用默认值 */
} xg_options_t;
```

**C++（`robot_sdk.h`）**
```cpp
struct Options {
    std::string deviceIp{""};        /**< 设备 IP，必填。当前要求与设备同一交换机直连 */
    std::uint16_t port{0};           /**< modeling_app 端口，0 表示使用默认端口 7448 */
    std::uint32_t connectTimeoutMs{0}; /**< 连接超时（毫秒），0 表示使用默认值 */
};
```

**成员说明**

| C | C++ | 约束 |
|---|-----|------|
| `device_ip[64]` | `deviceIp` | 必填；空或 `size >= 64` → `InvalidArg`（C++ 不截断） |
| `port` | `port` | modeling_app 端口；0 = 默认 7448。旧设备 modeling_app 随机端口时填其实际端口 |
| `connect_timeout_ms` | `connectTimeoutMs` | 0 = 5000 ms |
| `reserved` | — | 对齐 |

> 内存所有权属「调用方分配型」（[0.7.1](#071-调用方分配型同步查询--入参)）。连接语义见 [§5.1 open](#51-open--xg_device_open)。

---

## 3. 订阅数据结构

本章结构体均属「SDK 填充型」（[0.7.2](#072-sdk-填充型订阅回调)），由订阅回调填充。**订阅怎么调、回调指针能活多久、如何持久化，见 [§7 订阅接口](#7-订阅接口)。** 本章只描述数据形态与坐标系语义。

> **时间戳：** 本章各结构的 `timestamp_sec` 都是 Unix epoch 秒，但**各路取自不同的时钟**（设备系统时钟、MCU 时钟、雷达时钟），换算关系与能否与 `getDeviceTime` 对比见 [§0.8](#08-时间戳时基与换算)，本章不重复。

### 3.1 位姿 Pose

> **坐标系约定（重要）：** SDK 对外输出的位姿以**设备（IMU）系**为基准（R_W_I，原点在设备 IMU 点）。设备是唯一基准，**载体（狗/车等）的安装变换由集成方自行计算**（与竞品一致）。设备内部另有一套 Body/载体系位姿仅供设备导航使用，不经 SDK。坐标系详见 [coordinate_frames_and_tf.md](coordinate_frames_and_tf.md)。
>
> **数据面 key：** **`device_locate`（IMU 系）**。下表字段语义为设备（IMU）系。

**C（`robot_sdk_c.h`）**
```c
typedef struct {
    size_t struct_size;   /**< 必须由 SDK 填写为 sizeof(xg_pose_t) */
    double timestamp_sec; /**< 时间戳（秒） */

    float pos_x;  /**< 设备(IMU)点在世界系下的 X 位置 (m) */
    float pos_y;  /**< 设备(IMU)点在世界系下的 Y 位置 (m) */
    float pos_z;  /**< 设备(IMU)点在世界系下的 Z 位置 (m) */

    float quat_w; /**< 四元数实部 (W←I, R_W_I) */
    float quat_x; /**< 四元数 X (W←I) */
    float quat_y; /**< 四元数 Y (W←I) */
    float quat_z; /**< 四元数 Z (W←I) */

    float vel_x;  /**< 设备(IMU)系线速度 X (m/s) */
    float vel_y;  /**< 设备(IMU)系线速度 Y (m/s) */
    float vel_z;  /**< 设备(IMU)系线速度 Z (m/s) */

    float gyro_x; /**< 设备(IMU)系角速度 X (rad/s) */
    float gyro_y; /**< 设备(IMU)系角速度 Y (rad/s) */
    float gyro_z; /**< 设备(IMU)系角速度 Z (rad/s) */

    uint32_t error_code; /**< 定位故障码，0 = 正常 */
    uint8_t is_local;    /**< 保留字段：1.0 暂不提供，恒为 0，勿使用 */
    uint8_t reserved[3]; /**< 对齐填充 */
} xg_pose_t;
```

**C++（`robot_sdk.h`）**
```cpp
struct Pose {
    double timestampSec{0.0};
    float posX{0}, posY{0}, posZ{0};
    float quatW{1}, quatX{0}, quatY{0}, quatZ{0};
    float velX{0}, velY{0}, velZ{0};
    float gyroX{0}, gyroY{0}, gyroZ{0};
    std::uint32_t errorCode{0};
    bool isLocal{false};
};
```

**成员说明**（坐标系目标 = 设备/IMU 系：**位置** = 设备 IMU 点在世界系坐标；**姿态** = R_W_I（世界←IMU）；**速度/角速度** = IMU（设备）系）

| C 字段 | C++ 字段 | 类型 | 单位 | 含义 |
|--------|----------|------|------|------|
| `timestamp_sec` | `timestampSec` | double | s | 时间戳（proto stamp 微秒 / 1e6） |
| `pos_x/y/z` | `posX/Y/Z` | float | m | 设备 IMU 点的世界坐标 |
| `quat_w/x/y/z` | `quatW/X/Y/Z` | float | — | 单位四元数（W←I），`(1,0,0,0)` = 无旋转 |
| `vel_x/y/z` | `velX/Y/Z` | float | m/s | 设备（IMU）系线速度 |
| `gyro_x/y/z` | `gyroX/Y/Z` | float | rad/s | 设备（IMU）系角速度 |
| `error_code` | `errorCode` | uint32 | — | 定位故障码，0 = 正常 |
| `is_local` | `isLocal` | uint8 / bool | — | **保留字段：1.0 暂不提供，恒为 0，勿使用**（proto 对齐后启用） |

> C++ 版无 `struct_size`（SDK 内部代填）、`is_local` 表示为 `bool isLocal`、四元数默认 `(1,0,0,0)`。设备侧还发送协方差 `cov`、时延 `delay_time`，本 ABI **不暴露**。

**字节布局**（64 位 `size_t`，小端；跨平台以实际 `sizeof`/`offsetof` 为准，勿硬编码）

| 偏移 | 字段 | 类型 | 字节 |
|------|------|------|------|
| 0 | `struct_size` | size_t | 8 |
| 8 | `timestamp_sec` | double | 8 |
| 16 | `pos_x/y/z` | float×3 | 12 |
| 28 | `quat_w/x/y/z` | float×4 | 16 |
| 44 | `vel_x/y/z` | float×3 | 12 |
| 56 | `gyro_x/y/z` | float×3 | 12 |
| 68 | `error_code` | uint32 | 4 |
| 72 | `is_local` | uint8 | 1 |
| 73 | `reserved[3]` | uint8×3 | 3 |

> 由 `subscribePose` 填充。回调指针寿命与持久化 → 见 [§7.1 位姿订阅](#71-位姿订阅)。

### 3.2 单点 Point

**C（`robot_sdk_c.h`）**
```c
/** 单个点的 POD 表示 */
typedef struct {
    float x;         /**< X 坐标 (m)，坐标系随订阅接口而定 */
    float y;         /**< Y 坐标 (m) */
    float z;         /**< Z 坐标 (m) */
    float intensity; /**< 反射强度 */
} xg_point_t;
```

**C++（`robot_sdk.h`）**：`xg_point_t` 是纯 POD，C++ 侧**直接复用**，不造镜像。

**成员说明**

| C 字段 | C++ 字段 | 类型 | 单位 | 含义 |
|--------|----------|------|------|------|
| `x, y, z` | （同） | float | m | 点坐标（坐标系随订阅接口而定） |
| `intensity` | （同） | float | — | 反射强度（proto `w` / scaler） |

**字节布局**：16 字节，`x/y/z/intensity` 各 float，偏移 0/4/8/12，**无 padding**。数组内存与 ROS `PointCloud2` 的 XYZI（float32×4）逐字节一致，可直接 `memcpy`（`sdk_bridge_node.cpp` 即如此转 PointCloud2）。

### 3.3 点云帧 CloudFrame

> **坐标系约定（重要）：** 点云与位姿一样，SDK 对外以**设备（IMU）系**为基准，桥接与 SDK 均不做外参变换。
>
> - 点云只剩一路（见下）。
> - Locate 去畸变点云 `device_undist_cloud`（1.1 起）：设备端 Locate 输出 `imu_cloud`，设备（IMU）系，点数取决于 Locate 输入点云（当前为驱动层 1/4 降采样；全量输入属 locate-full-cloud-feed，RFC 状态：提案，未实现；未验证）；建图阶段 Locate 未输出时无数据；回调运行在 Zenoh RX 线程，回调内禁止同步文件 IO、`join`、`startLocate`；回调单次耗时上限待定。
> - `device_reloc_cloud` 与 `subscribeDeviceRelocCloud` 已在 1.1 删除，不双发。
>
> 坐标系详见 [coordinate_frames_and_tf.md](coordinate_frames_and_tf.md)。

**C（`robot_sdk_c.h`）**
```c
/** 点云帧数据 */
typedef struct {
    size_t struct_size;       /**< 必须由 SDK 填写为 sizeof(xg_cloud_frame_t) */
    const xg_point_t* points; /**< 点数组首地址，回调返回后不可再访问 */
    size_t count;             /**< 点数量 */
    double timestamp_sec;     /**< 时间戳（秒） */
    uint32_t seq;             /**< 帧序号，自订阅起单调递增 */
} xg_cloud_frame_t;
```

**C++（`robot_sdk.h`）**
```cpp
struct CloudFrame {
    const xg_point_t* points{nullptr}; /**< 点数组，回调期间有效 */
    std::size_t count{0};              /**< 点数量 */
    double timestampSec{0.0};          /**< 时间戳（秒） */
    std::uint32_t seq{0};              /**< 帧序号 */
};
```

> `CloudFrame` 镜像省去 `struct_size`。只有 `subscribeDeviceUndistCloud`（Locate 去畸变点云，见 [§7.2 点云订阅](#72-点云订阅)）使用此结构体；原始雷达点云使用 [`RawCloudFrame`](#331-原始点云-rawcloudframe)。重定位点云订阅已在 1.1 删除。

**成员说明**

| C 字段 | C++ 字段 | 类型 | 含义 |
|--------|----------|------|------|
| `struct_size` | （SDK 填写） | size_t | SDK 写入，调用方只读 |
| `points` | `points` | `const xg_point_t*` | 指向 `count` 个 `xg_point_t` 的连续数组 |
| `count` | `count` | size_t | 点数量 |
| `timestamp_sec` | `timestampSec` | double | 时间戳（秒） |
| `seq` | `seq` | uint32 | **SDK 本地**序号，自订阅起递增，非设备 seq；不能用其跳变发现丢帧，点云丢帧只能用相邻帧 `timestamp_sec` 的间隔判断 |

**布局示意**

```
frame->points ─► [ point[0] | point[1] | ... | point[count-1] ]
                   └16B┘      └16B┘            每个 = {x,y,z,intensity} float×4
```

> 由 `subscribeDeviceUndistCloud` 填充。`points` 指针寿命与拷贝方式 → 见 [§7.2 点云订阅](#72-点云订阅)。

### 3.3.1 原始点云 RawCloudFrame

> **与 §3.3 去畸变点云的区别：** 本路是 RoboSense 一次完整扫描的**全部点**，**未降采样、未运动去畸变**；点坐标为设备（IMU）系，只做了静态外参刚体变换（`extrinsic_imu_lidar`，Lidar→IMU，见 `getLidarCalib`）。每个点带 `time_offset_us`，集成方用它配合自己的位姿做去畸变。点类型和帧类型都**不同于** §3.2 / §3.3，不能混用。

**C（`robot_sdk_c.h`）**
```c
/** 原始雷达点。固定 24 字节布局，无 struct_size，永不改动 */
typedef struct {
    float x, y, z;          /**< 设备(IMU)系，单位 m，线路上四舍五入到 1 mm */
    float intensity;        /**< 雷达反射强度，无单位 */
    int32_t time_offset_us;  /**< 相对 timestamp_sec 的偏移 (us)，可为负 */
    uint16_t ring;          /**< 雷达线束号，超过 65535 时饱和为 65535 */
    uint16_t reserved;      /**< 保留，集成方须忽略其值 */
} xg_raw_point_t;

typedef struct {
    size_t struct_size;            /**< 由 SDK 填写为 sizeof(xg_raw_cloud_frame_t) */
    const xg_raw_point_t* points;  /**< 回调返回后不可再访问 */
    size_t count;
    double timestamp_sec;          /**< 帧内最早点的采样时刻（雷达自身时钟的 Unix epoch 秒，
                                        不保证等于设备系统时钟，见 §0.8） */
    uint32_t seq;                  /**< 设备端雷达 frame_id，不是 SDK 本地计数 */
} xg_raw_cloud_frame_t;
```

**C++（`robot_sdk.h`）**
```cpp
struct RawCloudFrame {
    const xg_raw_point_t* points{nullptr}; /**< 点数组，回调期间有效 */
    std::size_t count{0};
    double timestampSec{0.0};
    std::uint32_t seq{0};                  /**< 设备端雷达 frame_id */
};
```

**字节布局（`xg_raw_point_t`，24 字节）**

| 偏移 | 字段 | 类型 |
|------|------|------|
| 0 / 4 / 8 | `x` / `y` / `z` | float |
| 12 | `intensity` | float |
| 16 | `time_offset_us` | int32 |
| 20 | `ring` | uint16 |
| 22 | `reserved` | uint16 |

**语义要点**

- 点的采样时刻 = `timestamp_sec + time_offset_us × 1e-6`。
- `seq` 是设备端雷达帧号，可用其跳变发现设备端丢帧（设备端拥塞时 DROP）。与 `CloudFrame.seq`（SDK 本地计数）语义不同。
- 坐标在设备端四舍五入到毫米后传输，最大误差 0.5 mm。
- `reserved` 为保留位，集成方须忽略其值；当前实现写 0，预留给逐点标志。
- `timestamp_sec` 是帧内最早点的采样时刻，帧内各点的 `time_offset_us` 不小于 0 是预期（驱动按首点取帧时间戳，`ts_first_point`，所以最小偏移按设计为 0，未上板实测），契约允许为负。
- `timestamp_sec` 取自**雷达自身时钟**（RoboSense 数据包头里的 UTC 时间），不保证等于设备系统时钟，不能直接与 `getDeviceTime` 的 `device_epoch_us` 相减；换算关系见 [§0.8](#08-时间戳时基与换算)。
- 不过滤设备配置里被屏蔽的线束。

### 3.4 惯导 Imu

**C（`robot_sdk_c.h`）**
```c
typedef struct {
    size_t struct_size;    /**< 必须由 SDK 填写为 sizeof(xg_imu_t) */
    double timestamp_sec;  /**< 时间戳（秒） */
    float acc_x;           /**< 加速度 X (m/s^2) */
    float acc_y;           /**< 加速度 Y (m/s^2) */
    float acc_z;           /**< 加速度 Z (m/s^2) */
    float gyr_x;           /**< 角速度 X (rad/s) */
    float gyr_y;           /**< 角速度 Y (rad/s) */
    float gyr_z;           /**< 角速度 Z (rad/s) */
} xg_imu_t;
```

**C++（`robot_sdk.h`）**
```cpp
struct Imu {
    double timestampSec{0.0};
    float accX{0}, accY{0}, accZ{0};
    float gyrX{0}, gyrY{0}, gyrZ{0};
};
```

**成员说明**

| C 字段 | C++ 字段 | 类型 | 单位 | 含义 |
|--------|----------|------|------|------|
| `timestamp_sec` | `timestampSec` | double | s | 时间戳 |
| `acc_x/y/z` | `accX/Y/Z` | float | m/s² | 三轴加速度 |
| `gyr_x/y/z` | `gyrX/Y/Z` | float | rad/s | 三轴角速度 |

> 纯标量、无堆指针。由 `subscribeImu` 填充，寿命说明见 [§7.3 IMU 订阅](#73-imu-订阅)。

### 3.5 相机标识 CameraId

**C（`robot_sdk_c.h`）**
```c
/** 相机索引 */
typedef enum {
    XG_CAMERA_LEFT = 0,    /**< 左相机 */
    XG_CAMERA_CENTER = 1,  /**< 中相机 */
    XG_CAMERA_RIGHT = 2,   /**< 右相机 */
} xg_camera_id_t;
```

**C++（`robot_sdk.h`）**
```cpp
enum class CameraId {
    Left = XG_CAMERA_LEFT,
    Center = XG_CAMERA_CENTER,
    Right = XG_CAMERA_RIGHT,
};
```

**成员说明**：`Left`→`camera/left`、`Center`→`camera/center`、`Right`→`camera/right`（Zenoh key 映射）。⚠️ 此 `CameraId`（0/1/2）与 `getCameraCalib(0..3)` 的标定索引**不是同一套映射**。

### 3.6 图像帧 ImageFrame

**C（`robot_sdk_c.h`）**
```c
/** 图像帧数据 */
typedef struct {
    size_t struct_size;          /**< 必须由 SDK 填写为 sizeof(xg_image_frame_t) */
    xg_camera_id_t camera_id;   /**< 相机标识 */
    double timestamp_sec;        /**< 时间戳（秒） */
    const uint8_t* jpeg_data;   /**< JPEG 裸数据指针，回调返回后不可再访问 */
    size_t jpeg_size;            /**< JPEG 数据大小（字节） */
    uint32_t seq;                /**< 帧序号，自订阅起单调递增 */
} xg_image_frame_t;
```

**C++（`robot_sdk.h`）**
```cpp
struct ImageFrame {
    CameraId cameraId{CameraId::Left};
    double timestampSec{0.0};
    const uint8_t* jpegData{nullptr}; /**< JPEG 裸数据，回调期间有效 */
    std::size_t jpegSize{0};
    std::uint32_t seq{0};
};
```

**成员说明**

| C 字段 | C++ 字段 | 类型 | 含义 |
|--------|----------|------|------|
| `struct_size` | （SDK 填写） | size_t | SDK 写入，调用方只读 |
| `camera_id` | `cameraId` | 枚举 | 相机标识（见 [3.5](#35-相机标识-cameraid) key 映射） |
| `timestamp_sec` | `timestampSec` | double | 时间戳，由线上头 `timestamp_us * 1e-6` 得来 |
| `jpeg_data` | `jpegData` | `const uint8_t*` | JPEG 码流首地址，**回调后失效** |
| `jpeg_size` | `jpegSize` | size_t | JPEG 字节数 |
| `seq` | `seq` | uint32 | SDK 本地递增序号 |

**线上载荷格式**（Zenoh `camera/*`）：设备端 `ZenohNode::publishImage` 打包为 **16 字节头 + JPEG 裸流**，SDK 已替你解析头部，回调里的 `jpegData/jpegSize` 就是纯 JPEG。头部布局（小端）：

| 偏移 | 字段 | 类型 | 字节 | 说明 |
|------|------|------|------|------|
| 0 | `timestamp_us` | uint64 | 8 | 微秒时间戳 |
| 8 | `jpeg_size` | uint32 | 4 | JPEG 字节数 |
| 12 | `reserved` | — | 4 | 保留（当前全 0；未来可能承载格式标志） |
| 16 | JPEG 码流 | uint8[] | `jpeg_size` | 完整 JPEG（含 SOI/EOI） |

```
payload: [ 8B ts_us | 4B jpeg_size | 4B reserved ][ ...... JPEG ...... ]
         └────────── 16B 头（SDK 已解析） ──────┘ └ jpegData/jpegSize ┘
```

> **当前格式恒为 JPEG**；将来若新增 H.264 子码流订阅，会通过新接口或头部 `reserved` 内的格式字段区分（见 modeling_app 侧 `docs/camera_stream_pipeline.md`），届时集成方需按格式分别解码。由 `subscribeImage` 填充，`jpegData` 寿命见 [§7.4 图像订阅](#74-图像订阅)。

### 3.7 错误码 ErrCodeFrame

**C（`robot_sdk_c.h`）**
```c
/** 单个错误码条目 */
typedef struct {
    uint32_t code;        /**< 错误码 */
    uint64_t timestamp;   /**< 时间戳（微秒） */
} xg_err_code_entry_t;

/** 错误码帧数据 */
typedef struct {
    size_t struct_size;                  /**< 必须由 SDK 填写为 sizeof(xg_err_code_frame_t) */
    const xg_err_code_entry_t* entries; /**< 错误码数组首地址，回调返回后不可再访问 */
    size_t count;                        /**< 错误码数量 */
} xg_err_code_frame_t;
```

**C++（`robot_sdk.h`）**
```cpp
struct ErrCodeEntry {
    std::uint32_t code{0};
    std::uint64_t timestamp{0}; /**< 微秒 */
};

struct ErrCodeFrame {
    std::vector<ErrCodeEntry> entries;
};
```

> C++ 侧 `entries` 是**桥接层已拷出的 `std::vector`**，不是裸指针 —— 这是它与点云/图像最大的差异。

**成员说明**

| C 字段 | C++ 字段 | 类型 | 含义 |
|--------|----------|------|------|
| `entries[i].code` | `entries[i].code` | uint32 | 模块错误码 |
| `entries[i].timestamp` | `entries[i].timestamp` | uint64 | 时间戳（**微秒**） |
| `entries` | `entries` | 指针 / `std::vector` | 错误码数组；C 为裸指针回调后失效，C++ 为已拷 vector |
| `count` | （`entries.size()`） | size_t | 条目数（C 侧字段；C++ 用 vector 大小） |

**字节布局**（`xg_err_code_entry_t`，注意对齐 padding）

| 偏移 | 字段 | 类型 | 字节 |
|------|------|------|------|
| 0 | `code` | uint32 | 4 |
| 4 | （padding） | — | 4 |
| 8 | `timestamp` | uint64 | 8 |

因 `uint64` 需 8 字节对齐，`code` 之后有 **4 字节填充**，`sizeof(xg_err_code_entry_t) == 16`（非 12）。跨语言/跨平台解析勿假设紧凑排布，以 `offsetof` 为准。

> 由 `subscribeErrCode` 填充。C 侧裸指针 vs C++ 侧 vector 的寿命差异见 [§7.5 错误码订阅](#75-错误码订阅)。

---

## 4. 同步查询与入参数据结构

本章结构体均属「调用方分配型」（[0.7.1](#071-调用方分配型同步查询--入参)）：内存由调用方分配，无 SDK 拥有的堆指针，字符串为定长内嵌数组，**无需 `free`/`delete`**；C 侧调用前须置 `struct_size`，C++ 封装代填。获取语义见第二板块对应接口。

### 4.1 相机标定 CameraCalib

**C（`robot_sdk_c.h`）**
```c
/** 相机标定参数 */
typedef struct {
    size_t struct_size;    /**< 必须由调用方填写为 sizeof(xg_camera_calib_t) */
    uint32_t width;       /**< 图像宽度 */
    uint32_t height;      /**< 图像高度 */
    float fx;             /**< 焦距 X（像素） */
    float fy;             /**< 焦距 Y（像素） */
    float cx;             /**< 主点 X */
    float cy;             /**< 主点 Y */
    float distortion[4];  /**< 畸变系数（kb4: k1,k2,k3,k4; pinhole: k1,k2,p1,p2） */
    char model[32];       /**< 标定模型（"kb4" 或 "pinhole"） */
    float pose[16];       /**< 相对 cam0 的 4x4 变换矩阵（行优先） */
    uint8_t calibrated;   /**< 是否已标定 */
    uint8_t reserved[3];
} xg_camera_calib_t;
```

**C++（`robot_sdk.h`）**
```cpp
/** 相机标定参数 */
struct CameraCalib {
    uint32_t width{0}, height{0};
    float fx{0}, fy{0}, cx{0}, cy{0};
    float distortion[4]{0};
    std::string model;
    float pose[16]{0}; /**< 相对 cam0 的 4x4 变换 */
    bool calibrated{false};
};
```

> C++ 版无 `struct_size`（封装代填）、`model` 为 `std::string`、`calibrated` 为 `bool`。

**成员说明**

| C 字段 | C++ 字段 | 类型 | 含义 |
|--------|----------|------|------|
| `struct_size` | （封装填写） | size_t | 调用前必须 = `sizeof` |
| `width, height` | `width, height` | uint32 | 图像尺寸（像素） |
| `fx, fy` | `fx, fy` | float | 焦距（像素） |
| `cx, cy` | `cx, cy` | float | 主点 |
| `distortion[4]` | `distortion[4]` | float×4 | kb4: k1–k4；pinhole: k1,k2,p1,p2 |
| `model` | `model` | `char[32]` / `std::string` | `"kb4"` / `"pinhole"` 等 |
| `pose[16]` | `pose[16]` | float×16 | 相对 cam0 的 4×4 变换，**行优先** |
| `calibrated` | `calibrated` | uint8 / bool | 是否已标定 |

> 获取语义见 [§8.1 getCameraCalib](#81-getcameracalib--xg_get_camera_calib)。

### 4.2 IMU 标定 ImuCalib

**C（`robot_sdk_c.h`）**
```c
/** IMU 标定参数 */
typedef struct {
    size_t struct_size;        /**< 必须由调用方填写为 sizeof(xg_imu_calib_t) */
    float accel_bias[3];       /**< 加速度计零偏 Ba[3] */
    float gyro_bias[3];        /**< 陀螺仪零偏 Bg[3] */
    float accel_transform[9];  /**< 加速度计变换矩阵 Ta[3x3]（行优先） */
    float accel_scale[9];      /**< 加速度计尺度矩阵 Ka[3x3]（行优先） */
    float gyro_transform[9];   /**< 陀螺仪变换矩阵 Tg[3x3]（行优先） */
    float gyro_scale[9];       /**< 陀螺仪尺度矩阵 Kg[3x3]（行优先） */
    /**
     * T_imu_lidar：Lidar→IMU 的 4x4 齐次变换矩阵（行优先）。
     * 命名约定从右往左读：extrinsic_imu_lidar 表示将 Lidar 坐标系下的点
     * 变换到 IMU 坐标系。数据来源为设备标定文件 extrinsic_imu_lidar.yaml。
     */
    float extrinsic_imu_lidar[16];
    uint8_t calibrated;        /**< 是否已标定 */
    uint8_t reserved[3];
} xg_imu_calib_t;
```

**C++（`robot_sdk.h`）**
```cpp
/** IMU 标定参数 */
struct ImuCalib {
    float accelBias[3]{0};
    float gyroBias[3]{0};
    float accelTransform[9]{0};
    float accelScale[9]{0};
    float gyroTransform[9]{0};
    float gyroScale[9]{0};
    /**
     * T_imu_lidar：Lidar→IMU 的 4x4 齐次变换矩阵（行优先）。
     * 从右往左读：将 Lidar 坐标系下的点变换到 IMU 坐标系。
     */
    float extrinsicImuLidar[16]{0};
    bool calibrated{false};
};
```

**成员说明**

| C 字段 | C++ 字段 | 类型 | 含义 |
|--------|----------|------|------|
| `struct_size` | （封装填写） | size_t | 调用前必须 = `sizeof` |
| `accel_bias[3]` | `accelBias[3]` | float×3 | 加速度计零偏 Ba |
| `gyro_bias[3]` | `gyroBias[3]` | float×3 | 陀螺仪零偏 Bg |
| `accel_transform[9]` | `accelTransform[9]` | float×9 | 加速度计变换 Ta（3×3，行优先） |
| `accel_scale[9]` | `accelScale[9]` | float×9 | 加速度计尺度 Ka（3×3，行优先） |
| `gyro_transform[9]` | `gyroTransform[9]` | float×9 | 陀螺仪变换 Tg（3×3，行优先） |
| `gyro_scale[9]` | `gyroScale[9]` | float×9 | 陀螺仪尺度 Kg（3×3，行优先） |
| `extrinsic_imu_lidar[16]` | `extrinsicImuLidar[16]` | float×16 | T_imu_lidar：Lidar 点 → IMU 系，4×4 行优先 |
| `calibrated` | `calibrated` | uint8 / bool | 是否已标定 |

> 获取语义见 [§8.2 getImuCalib](#82-getimucalib--xg_get_imu_calib)。

### 4.3 雷达标定 LidarCalib

**C（`robot_sdk_c.h`）**
```c
/** 雷达标定参数 */
typedef struct {
    size_t struct_size;             /**< 必须由调用方填写为 sizeof(xg_lidar_calib_t) */
    /**
     * T_imu_lidar：Lidar→IMU 的 4x4 齐次变换矩阵（行优先）。
     * 命名约定从右往左读：extrinsic_imu_lidar 表示将 Lidar 坐标系下的点
     * 变换到 IMU 坐标系。数据来源为设备标定文件 extrinsic_imu_lidar.yaml。
     */
    float extrinsic_imu_lidar[16];
    /**
     * T_cam0_lidar：Lidar→Cam0 的 4x4 齐次变换矩阵（行优先）。
     * 命名约定从右往左读：extrinsic_camera_lidar 表示将 Lidar 坐标系下的点
     * 变换到 Cam0 坐标系。数据来源为设备标定文件 extrinsic_camera_lidar.yaml。
     */
    float extrinsic_camera_lidar[16];
    char lidar_type[64];           /**< 雷达型号 */
    uint8_t calibrated;            /**< 是否已标定 */
    uint8_t reserved[3];
} xg_lidar_calib_t;
```

**C++（`robot_sdk.h`）**
```cpp
/** 雷达标定参数 */
struct LidarCalib {
    /**
     * T_imu_lidar：Lidar→IMU 的 4x4 齐次变换矩阵（行优先）。
     * 从右往左读：将 Lidar 坐标系下的点变换到 IMU 坐标系。
     */
    float extrinsicImuLidar[16]{0};
    /**
     * T_cam0_lidar：Lidar→Cam0 的 4x4 齐次变换矩阵（行优先）。
     * 从右往左读：将 Lidar 坐标系下的点变换到 Cam0 坐标系。
     */
    float extrinsicCameraLidar[16]{0};
    std::string lidarType;
    bool calibrated{false};
};
```

**成员说明**

| C 字段 | C++ 字段 | 类型 | 含义 |
|--------|----------|------|------|
| `struct_size` | （封装填写） | size_t | 调用前必须 = `sizeof` |
| `extrinsic_imu_lidar[16]` | `extrinsicImuLidar[16]` | float×16 | T_imu_lidar：Lidar 点 → IMU 系，4×4 行优先 |
| `extrinsic_camera_lidar[16]` | `extrinsicCameraLidar[16]` | float×16 | T_cam0_lidar：Lidar 点 → Cam0 系，4×4 行优先 |
| `lidar_type` | `lidarType` | `char[64]` / `std::string` | 雷达型号 |
| `calibrated` | `calibrated` | uint8 / bool | 是否已标定 |

> 获取语义见 [§8.3 getLidarCalib](#83-getlidarcalib--xg_get_lidar_calib)。

### 4.4 设备状态 DeviceInfo

**C（`robot_sdk_c.h`）**
```c
/** 设备状态 */
typedef struct {
    size_t struct_size;     /**< 必须由调用方填写为 sizeof(xg_device_info_t) */
    float battery_level;   /**< 电池电量（百分比） */
    float battery_temp;    /**< 电池温度（摄氏度） */
    float battery_voltage; /**< 电池电压（V） */
    int64_t disk_total_kb; /**< 存储总量（KB） */
    int64_t disk_used_kb;  /**< 存储已用（KB） */
    int64_t mem_total_kb;  /**< 内存总量（KB） */
    int64_t mem_used_kb;   /**< 内存已用（KB） */
    uint8_t rtk_state;     /**< RTK 状态 */
    uint8_t satellite_num; /**< 卫星数 */
    int32_t modeling_state;/**< 建图状态码（内部状态，仅供参考） */
} xg_device_info_t;
```

**C++（`robot_sdk.h`）**
```cpp
/** 设备状态 */
struct DeviceInfo {
    float batteryLevel{0};       /**< 电池电量（%） */
    float batteryTemp{0};        /**< 电池温度（℃） */
    float batteryVoltage{0};     /**< 电池电压（V） */
    std::int64_t diskTotalKb{0}; /**< 存储总量（KB） */
    std::int64_t diskUsedKb{0};  /**< 存储已用（KB） */
    std::int64_t memTotalKb{0};  /**< 内存总量（KB） */
    std::int64_t memUsedKb{0};   /**< 内存已用（KB） */
    std::uint8_t rtkState{0};    /**< RTK 状态 */
    std::uint8_t satelliteNum{0};/**< 卫星数 */
    std::int32_t modelingState{0}; /**< 建图状态码 */
};
```

**成员说明**

| C 字段 | C++ 字段 | 类型 | 单位 | 含义 |
|--------|----------|------|------|------|
| `struct_size` | （封装填写） | size_t | — | 调用前必须 = `sizeof` |
| `battery_level` | `batteryLevel` | float | % | 电池电量 |
| `battery_temp` | `batteryTemp` | float | ℃ | 电池温度 |
| `battery_voltage` | `batteryVoltage` | float | V | 电池电压 |
| `disk_total_kb` / `disk_used_kb` | `diskTotalKb` / `diskUsedKb` | int64 | KB | 存储总量 / 已用 |
| `mem_total_kb` / `mem_used_kb` | `memTotalKb` / `memUsedKb` | int64 | KB | 内存总量 / 已用 |
| `rtk_state` / `satellite_num` | `rtkState` / `satelliteNum` | uint8 | — | 设备原值，无本 SDK 枚举文档 |
| `modeling_state` | `modelingState` | int32 | — | 内部建图码，仅参考；**不是 `RecordState`** |

> 不映射设备 JSON 的 `type/cycle`、`cpu_loads`、`navsat_status`。`modelingState` 不要当 `RecordState` 用；录制状态请用 `queryRecordStatus`。获取语义见 [§9.1 getDeviceInfo](#91-getdeviceinfo--xg_get_device_info)。

### 4.4.1 设备时间 DeviceTime

> 设备系统时钟的只读快照加本机往返时刻。**换算关系、哪些订阅数据取自设备系统时钟，见 [§0.8](#08-时间戳时基与换算)。** 获取语义见 [§9.2 getDeviceTime](#92-getdevicetime--xg_get_device_time)。

**C（`robot_sdk_c.h`）**
```c
typedef struct {
    size_t struct_size;         /**< 必须由调用方填写为 sizeof(xg_device_time_t) */
    int64_t device_epoch_us;    /**< 设备系统时钟（CLOCK_REALTIME），Unix epoch 微秒 */
    int32_t utc_offset_s;       /**< 设备本地时区相对 UTC 的偏移（秒，含夏令时），仅用于显示 */
    int32_t reserved;           /**< 保留，SDK 写 0，集成方须忽略 */
    int64_t host_send_epoch_us; /**< 本机发出查询时的 system_clock，Unix epoch 微秒 */
    int64_t host_recv_epoch_us; /**< 本机收到应答时的 system_clock，Unix epoch 微秒 */
    int64_t rtt_us;             /**< 本机 steady_clock 测得的往返时延（微秒） */
} xg_device_time_t;
```

**C++（`robot_sdk.h`）**
```cpp
struct DeviceTime {
    std::int64_t deviceEpochUs{0};
    std::int32_t utcOffsetSec{0};
    std::int64_t hostSendEpochUs{0};
    std::int64_t hostRecvEpochUs{0};
    std::int64_t rttUs{0};
    std::int64_t offsetLowerUs() const;  // deviceEpochUs - hostRecvEpochUs
    std::int64_t offsetUpperUs() const;  // deviceEpochUs - hostSendEpochUs
    std::int64_t offsetMidUs() const;    // 区间中点，仅粗略估计
};
```

**成员说明**

| C 字段 | C++ 字段 | 类型 | 单位 | 含义 |
|--------|----------|------|------|------|
| `struct_size` | （封装填写） | size_t | — | 调用前必须 = `sizeof` |
| `device_epoch_us` | `deviceEpochUs` | int64 | µs | 设备系统时钟，自 1970-01-01 00:00:00 UTC 起；设备在状态查询完成后、序列化应答之前采样 |
| `utc_offset_s` | `utcOffsetSec` | int32 | s | 设备本地时区相对 UTC 的偏移，含夏令时；范围 -86400 到 86400，超出视为应答非法；`device_epoch_us` 同理，大于 0 且不晚于 3000-01-01 00:00:00 UTC |
| `reserved` | （无） | int32 | — | 保留，写 0 |
| `host_send_epoch_us` | `hostSendEpochUs` | int64 | µs | 本机 `system_clock`，查询发出前取 |
| `host_recv_epoch_us` | `hostRecvEpochUs` | int64 | µs | 本机 `system_clock`，应答收到后取 |
| `rtt_us` | `rttUs` | int64 | µs | 本机 `steady_clock` 测得，不受 `system_clock` 调整影响 |

字节布局（共 48 字节，无填充）：`struct_size` 偏移 0（8 字节）；`device_epoch_us` 偏移 8；`utc_offset_s` 偏移 16；`reserved` 偏移 20；`host_send_epoch_us` 偏移 24；`host_recv_epoch_us` 偏移 32；`rtt_us` 偏移 40。

### 4.5 定位初值 InitPose

> 这是重定位**先验**（入参），不是查询结果。写入语义见 [§10.1 setLocInitPose](#101-setlocinitpose--xg_set_loc_init_pose)。

**C（`robot_sdk_c.h`）**
```c
/** 定位初始位姿参数 */
typedef struct {
    size_t struct_size;   /**< 必须由调用方填写为 sizeof(xg_init_pose_t) */
    double timestamp_sec; /**< 时间戳（秒） */
    float pos_x;          /**< 位置 X (m) */
    float pos_y;          /**< 位置 Y (m) */
    float pos_z;          /**< 位置 Z (m) */
    float quat_x;         /**< 四元数 X */
    float quat_y;         /**< 四元数 Y */
    float quat_z;         /**< 四元数 Z */
    float quat_w;         /**< 四元数 W */
} xg_init_pose_t;
```

**C++（`robot_sdk.h`）**
```cpp
/** 定位初始位姿参数 */
struct InitPose {
    double timestampSec{0.0};
    float posX{0}, posY{0}, posZ{0};
    float quatX{0}, quatY{0}, quatZ{0}, quatW{1};
};
```

> C++ 版无 `struct_size`（封装代填），`quatW` 默认 1（无旋转）。

**成员说明**

| C 字段 | C++ 字段 | 类型 | 单位 | 含义 |
|--------|----------|------|------|------|
| `struct_size` | （封装填写） | size_t | — | 调用前必须 = `sizeof` |
| `timestamp_sec` | `timestampSec` | double | s | 设备要求存在该字段，可为 0 |
| `pos_x/y/z` | `posX/Y/Z` | float | m | **世界 / 地图系**位置 |
| `quat_x/y/z/w` | `quatX/Y/Z/W` | float | — | 朝向；模长必须 > 0.1 |

### 4.6 工程路径 xg_data_path_t

**C（`robot_sdk_c.h`）**
```c
/** 数据路径最大长度 */
#define XG_PATH_MAX 512

/** 工程数据路径查询结果 */
typedef struct {
    size_t struct_size;       /**< 必须由调用方填写为 sizeof(xg_data_path_t) */
    char path[XG_PATH_MAX];  /**< 设备端工程数据目录路径，UTF-8 */
} xg_data_path_t;
```

**C++（`robot_sdk.h`）**：C++ 侧**无独立镜像结构体**，直接以 `std::string*` 出参：
```cpp
Error getDataPath(std::string* path);
```

> 这是唯一 C++ 侧不造平行结构体的同步类型——因只有单个字符串字段，封装直接返回 `std::string`。

**成员说明**

| C 字段 | C++ | 类型 | 含义 |
|--------|-----|------|------|
| `struct_size` | （封装填写） | size_t | 调用前必须 = `sizeof` |
| `path` | 返回值 `*path` | `char[512]` / `std::string` | 设备端工程数据目录，UTF-8，可能为空串 |

> 获取语义见 [§11.1 getDataPath](#111-getdatapath--xg_get_data_path)。

---

# 第二板块 · 接口

本板块描述每个接口的**作用、参数、返回值、副作用**，以及订阅回调中指针/变量的**内存与生命周期**。接口涉及的数据结构详见第一板块。

## 5. 连接接口

### 5.1 `open` / `xg_device_open`

**接口**

| | 签名 |
|---|------|
| C++ | `static std::unique_ptr<Device> Device::open(const Options& opts, Error* error = nullptr)` |
| C | `xg_err_t xg_device_open(const xg_options_t* opts, xg_device_t** out_dev)` |

**作用：** 建立到 modeling_app（默认 7448）的 Zenoh peer，并用 `checkRecordStatus` 做真实查询探测。
**前置：** 无。
**入参：** [`Options` / `xg_options_t`](#24-options--xg_options_t)（须 `struct_size`）。
**出参：** C++ `unique_ptr<Device>`；C `*out_dev`。失败时 C++ 为 `nullptr` 且可写 `error`；C 为 `NULL`。

**返回值**

| 返回 | 条件 |
|------|------|
| `Ok` | 探测收到可解析应答 |
| `InvalidArg` | 空指针、`struct_size` 错、IP 非法 |
| `Timeout` | 超时内 modeling_app 无 replier（端口不通/服务未起/IP 错） |
| 其他 | 探测应答协议错误等 |

**副作用：** 分配句柄与 session。未成功则不泄漏句柄。
**线程：** 调用线程阻塞至探测结束（默认最长约 `connectTimeoutMs`）。

### 5.2 `~Device` / `xg_device_close`

**接口**

| | 签名 |
|---|------|
| C++ | `Device::~Device()`（析构自动调用） |
| C | `void xg_device_close(xg_device_t* dev)` |

**作用：** 取消全部订阅，关闭 session。`dev == NULL` 为空操作。关闭后指针不得再用。

### 5.3 `isConnected` / `xg_device_is_connected`

**接口**

| | 签名 |
|---|------|
| C++ | `bool Device::isConnected() const` |
| C | `int xg_device_is_connected(const xg_device_t* dev)` |

**作用：** 探测与设备的连接是否正常。返回 `true`/1 或 `false`/0。`dev == NULL` → 0。
**注意：** 实现上可能再发一次 Get（最长约 3 s），**不要在热路径高频调用**。

---

## 6. 录制控制接口

**共同前置：** 句柄已 open。
**共同副作用：** start/stop 向设备派发事件，**不等待**状态机到达目标。

### 6.1 `startRecord` / `xg_record_start`

**接口**

| | 签名 |
|---|------|
| C++ | `Error Device::startRecord()` |
| C | `xg_err_t xg_record_start(xg_device_t* dev)` |

**作用：** 下发开始录制命令。**只下发不等待**，须轮询 `queryRecordStatus` 确认。

**返回值**

| 返回 | 含义 | 动作 |
|------|------|------|
| `Ok` | 已接受，或本就在录（幂等） | 轮询直到 `Recording` 或 `Error` |
| `Timeout` | 无应答 | **先 query**，再决定是否重发 |
| `Rejected` | 明确拒绝且非幂等 | query 原因 |
| `Protocol` | 应答坏 | 核对版本 |
| `InvalidArg` | `dev == NULL` | — |

轮询建议间隔 ≥ `XG_RECOMMENDED_POLL_INTERVAL_MS`（500）。`Idle`/`Starting` 继续等；`Error` 立即停。

### 6.2 `stopRecord` / `xg_record_stop`

**接口**

| | 签名 |
|---|------|
| C++ | `Error Device::stopRecord()` |
| C | `xg_err_t xg_record_stop(xg_device_t* dev)` |

**作用：** 下发停止录制命令。与 start 对称，目标态 `Idle`。`warning == true` 时仍可停。落盘耗时随数据量变化。返回值语义同 [6.1](#61-startrecord--xg_record_start)。

### 6.3 `queryRecordStatus` / `xg_record_query`

**接口**

| | 签名 |
|---|------|
| C++ | `Error Device::queryRecordStatus(RecordStatus* status)` |
| C | `xg_err_t xg_record_query(xg_device_t* dev, xg_record_status_t* out_status)` |

**作用：** 查询当前录制状态，**不改变设备状态**，可安全轮询。出参 [`RecordStatus`](#23-recordstatus--xg_record_status_t) 不可空，C 须填 `struct_size`。

**返回值**

| 返回 | 含义 |
|------|------|
| `Ok` | `status` 已填 |
| `InvalidArg` | 空或 `struct_size` 错 |
| `Timeout` | 无应答；**不代表录制失败**，可重试 |
| `Protocol` | 无 `data.recordStatus` 或非数字 |

---

## 7. 订阅接口

**共同约定：**

- 所有订阅须在 `open` 成功后调用；出数据通常要求设备已在录制且定位在跑，否则可能长时间 0 帧。
- 每类数据同一句柄只占**一个订阅槽**，重复订阅返回 `Rejected`（图像按相机分三个独立槽）。
- 回调在 **SDK 内部线程**（Zenoh 会话线程）触发，**不可阻塞**、不可在回调内再调同一句柄的 SDK 接口。
- 取消订阅**幂等**：无活跃订阅时也返回 `Ok`；返回后保证不再触发对应回调。
- 内存所有权总则见 [0.7](#07-内存布局与所有权模型)。
- **深度图 `depth/center`（`subscribeDepth` / `xg_subscribe_depth`，本说明书尚无完整条目，接口定义以头文件为准）：** 设备端默认配置下不发布此 key（T1XB 配置文件 `modeling_app_t1xb.yaml` 中 `lixel_t1xb.camera_center.ai.fps` 默认为 0），须设备端把该项配置为大于 0 且 AI 模型就绪后才有数据；无数据时订阅仍返回 `Ok`，SDK 无法区分原因。

### 7.1 位姿订阅

**接口**

| | C++ | C |
|---|-----|---|
| 订阅 | `Error subscribePose(std::function<void(const Pose&)>)` | `xg_err_t xg_subscribe_pose(xg_device_t*, xg_pose_cb_t, void*)` |
| 取消 | `Error unsubscribePose()` | `xg_err_t xg_unsubscribe_pose(xg_device_t*)` |

**作用：** 订阅设备位姿流。数据结构见 [§3.1 Pose](#31-位姿-pose)。
**频率 / 数据面 key：** 约 100 Hz；`device_locate`（IMU 系，1.0.0 起；见 3.1）。

**返回值**

| 返回 | 条件 |
|------|------|
| `Ok` | 订阅成功 |
| `InvalidArg` | `dev` / `cb` 空 |
| `Rejected` | 已有位姿订阅（须先 `unsubscribePose`） |
| `Internal` | Zenoh declare 失败 |

**内存与生命周期**

- 属「SDK 填充型」（[0.7.2](#072-sdk-填充型订阅回调)），但 `Pose` **纯标量、无堆指针**。
- 回调参数**可整体按值保存**（C++ 桥接层已把 C 结构体逐字段拷进 `Pose`），直接 `Pose saved = pose;` 即安全。无需任何 `free`/`delete`。

### 7.2 点云订阅

**接口**：本节只有一路点云，Locate 去畸变点云，使用结构体 [`CloudFrame`](#33-点云帧-cloudframe)（原始雷达点云见 [§7.2.1](#721-原始雷达点云订阅)）：

| 来源 | C++ | C | key | 说明 |
|------|-----|---|-----|------|
| Locate 去畸变点云 | `subscribeDeviceUndistCloud(cb)` / `unsubscribeDeviceUndistCloud()` | `xg_subscribe_device_undist_cloud` / `xg_unsubscribe_device_undist_cloud` | `device_undist_cloud` | 随 Locate 输出（频率未验证）；Locate 去畸变点云，设备（IMU）系；建图阶段无数据；回调在 Zenoh RX 线程，禁止同步文件 IO、`join`、`startLocate` |

> **1.1.0 破坏性变更：** 旧接口 `xg_subscribe_cloud` / `xg_subscribe_raw_cloud`（C++ `subscribeCloud` / `subscribeRawCloud`）及旧 key `reloc_device_cloud` / `raw_pointcloud` 已删除，不保留兼容。SDK 1.1.0 必须配套发布新 key 的设备固件；1.1.0 SDK 连接旧固件时订阅返回 `Ok` 但收不到点云，1.0.0 SDK 连接新固件同样收不到点云。

**作用：** 订阅 Locate 去畸变点云流。该订阅槽与 [§7.2.1](#721-原始雷达点云订阅) 的原始点云订阅槽独立，互不影响。

**返回值（C）：** `XG_OK`；`XG_ERR_INVALID_ARG`（`dev` 或 `cb` 为空）；`XG_ERR_REJECTED`（本句柄该路点云已有订阅，已有订阅不受影响）；`XG_ERR_INTERNAL`（Zenoh 订阅声明失败）。设备端暂无该 key 数据时订阅同样返回 `XG_OK`，只是暂无回调。C++ 封装始终传入非空桥接回调，因此 C++ 不会返回 `InvalidArg`。

**失败后状态：** C 接口返回非 `XG_OK` 时不登记订阅、不保存回调，修正后可重试。

**取消：** 幂等，无活跃订阅时也返回 `Ok`；`dev` 为空返回 `InvalidArg`；返回后保证不再触发该回调。禁止在该订阅自己的回调内取消。

**线程：** 回调在 SDK 内部线程（Zenoh 会话线程）触发；回调内禁止阻塞、禁止再调同一句柄的 SDK 接口。

**内存与生命周期**

- 属「SDK 填充型」（[0.7.2](#072-sdk-填充型订阅回调)）。`points` 指向 SDK 内部 `thread_local std::vector<xg_point_t>` 的 `data()`。
- **指针仅回调期内有效；下一帧点云回调会 `resize` 覆盖同一块缓冲。**
- 要留整帧，回调内深拷贝：`memcpy(dst, frame->points, frame->count * sizeof(xg_point_t))`（即 `count * 16` 字节）。
- **禁止**对 `points` 调用 `free`/`delete`。

### 7.2.1 原始雷达点云订阅

**接口**

| | C++ | C |
|---|-----|---|
| 订阅 | `Error subscribeDeviceRawCloud(std::function<void(const RawCloudFrame&)>)` | `xg_err_t xg_subscribe_device_raw_cloud(xg_device_t*, xg_raw_cloud_cb_t, void*)` |
| 取消 | `Error unsubscribeDeviceRawCloud()` | `xg_err_t xg_unsubscribe_device_raw_cloud(xg_device_t*)` |

**作用 / key / 频率：** 订阅 RoboSense 的全量、未去畸变点云，设备（IMU）系。key `device_raw_cloud`，~10 Hz。数据结构见 [§3.3.1](#331-原始点云-rawcloudframe)。独立订阅槽，与 §7.2 的 Locate 去畸变点云订阅互不影响。

**返回值（C）：** `XG_OK`；`XG_ERR_INVALID_ARG`（`dev` 或 `cb` 为空）；`XG_ERR_REJECTED`（本句柄已有原始点云订阅，已有订阅不受影响）；`XG_ERR_INTERNAL`（Zenoh 订阅声明失败或内存不足）。
**返回值（C++）：** `Ok`；`InvalidArg`（回调为空）；`Rejected`（已订阅，C++ 封装不覆盖原回调）；`Internal`。

**设备端发布条件：** 设备端只在存在订阅者（Zenoh 匹配状态）且处于扫描状态时才构造和发布全量帧；订阅者状态无法判断时退回常开发布。订阅后到收到首帧的延迟，由匹配状态传播和下一帧到达共同决定，未测量。

**订阅返回 `Ok` 但收不到帧的情况（SDK 无法区分）：** 设备雷达不是 RoboSense；设备端 `extrinsic_imu_lidar.yaml` 缺失或不是合法刚体变换（设备端丢弃该帧）；设备尚未进入录制/扫描状态。设备端拥塞时按 DROP 丢帧，表现为 `seq` 跳变。

**失败后状态：** C 接口返回非 `XG_OK` 时不登记订阅、不保存回调，修正后可重试。

**取消：** 幂等，无活跃订阅时也返回 `Ok`；`dev` 为空返回 `InvalidArg`；返回后保证不再触发该回调。禁止在该订阅自己的回调内取消或关闭句柄。

**线程：** 回调在 Zenoh 会话接收线程触发，不在调用方线程；回调内禁止阻塞、禁止再调同一句柄的 SDK 接口。阻塞会拖住本进程全部订阅。单帧点数多，回调内请只做拷贝或入队。

**内存与生命周期**

- 属「SDK 填充型」。`points` 指向 SDK 内部 `thread_local std::vector<xg_raw_point_t>` 的 `data()`，**仅回调期内有效，下一帧覆盖同一块缓冲**。
- 要留整帧，回调内深拷贝：`memcpy(dst, frame->points, frame->count * sizeof(xg_raw_point_t))`（即 `count * 24` 字节）。
- **禁止**对 `points` 调用 `free`/`delete`。

### 7.3 IMU 订阅

**接口**

| | C++ | C |
|---|-----|---|
| 订阅 | `Error subscribeImu(std::function<void(const Imu&)>)` | `xg_err_t xg_subscribe_imu(xg_device_t*, xg_imu_cb_t, void*)` |
| 取消 | `Error unsubscribeImu()` | `xg_err_t xg_unsubscribe_imu(xg_device_t*)` |

**作用：** 订阅原始 IMU 流，未经 SLAM 处理。数据结构见 [§3.4 Imu](#34-惯导-imu)。
**频率 / 数据面 key：** 约 200 Hz；`raw_imu`。

**返回值：** `Ok`；`InvalidArg`；`Rejected`（已有 IMU 订阅）；`Internal`。

**内存与生命周期**

- 属「SDK 填充型」，但**纯标量、无堆指针**，规则同 `Pose`：回调参数可整体按值保存，无需 `free`/`delete`。

### 7.4 图像订阅

**接口**

| | C++ | C |
|---|-----|---|
| 订阅 | `Error subscribeImage(CameraId, std::function<void(const ImageFrame&)>)` | `xg_err_t xg_subscribe_image(xg_device_t*, xg_camera_id_t, xg_image_cb_t, void*)` |
| 取消 | `Error unsubscribeImage(CameraId)` | `xg_err_t xg_unsubscribe_image(xg_device_t*, xg_camera_id_t)` |

**作用：** 订阅相机 JPEG 图像流。数据结构见 [§3.6 ImageFrame](#36-图像帧-imageframe)。
**订阅槽：** **三路相机各占独立订阅槽**（`Left`/`Center`/`Right`）。
**频率：** 取决于设备端 JPEG 编码配置（设备 yaml `jpeg.fps`，SDK 头注释历史值约 2 fps，以设备配置为准；板端实测某配置 `jpeg.fps=10` 时为 10.01 Hz）。
**Zenoh key 映射：** `Left`→`camera/left`、`Center`→`camera/center`、`Right`→`camera/right`。

**返回值：** `Ok`；`InvalidArg`（空 / `cameraId` 越界）；`Rejected`（**该相机**已有订阅）；`Internal`。

⚠️ 此 `CameraId`（0/1/2）与 `getCameraCalib(0..3)` 的标定索引**不是同一套映射**。

**内存与生命周期**

- 属「SDK 填充型」（[0.7.2](#072-sdk-填充型订阅回调)）。`jpegData` 指向 SDK 内部缓冲（Zenoh sample payload + 16），**回调返回即失效**。要存图/解码留用，须回调内 `memcpy(dst, frame.jpegData, frame.jpegSize)`。**禁止** `free`/`delete`。
- 线上载荷格式（16 字节头 + JPEG 裸流）已由 SDK 解析，回调里的 `jpegData/jpegSize` 就是纯 JPEG，详见 [§3.6](#36-图像帧-imageframe)。

### 7.5 错误码订阅

**接口**

| | C++ | C |
|---|-----|---|
| 订阅 | `Error subscribeErrCode(std::function<void(const ErrCodeFrame&)>)` | `xg_err_t xg_subscribe_err_code(xg_device_t*, xg_err_code_cb_t, void*)` |
| 取消 | `Error unsubscribeErrCode()` | `xg_err_t xg_unsubscribe_err_code(xg_device_t*)` |

**作用：** 订阅设备各模块（定位、建图、SLAM 等）错误码。数据结构见 [§3.7 ErrCodeFrame](#37-错误码-errcodeframe)。
**频率 / 数据面 key：** 设备批量推送，量级 ~10 Hz。空数组不回调。`err_code`。

**返回值：** `Ok`；`InvalidArg`；`Rejected`（已有订阅）；`Internal`。

**内存与生命周期**

- **C 侧**：`entries` 指向 SDK 内部 `thread_local std::vector<xg_err_code_entry_t>` 的 `data()`，**回调后被下一帧覆盖**；要留用须 `memcpy(dst, frame->entries, frame->count * sizeof(xg_err_code_entry_t))`。**禁止** `free`/`delete`。
- **C++ 侧**：`ErrCodeFrame.entries` 是已拷出的 `std::vector`，回调内可随意读；但它随回调参数对象销毁，**跨回调持有仍需再拷一份**（如 `auto saved = frame.entries;`）。

---

## 8. 标定查询接口

三者都对设备打同一 `getSensorCalib`，再抽字段。C 出参必须 `struct_size`。可在未录制时调用（读设备文件）。结构体字段见 [4.1](#41-相机标定-cameracalib)~[4.3](#43-雷达标定-lidarcalib)。

### 8.1 `getCameraCalib` / `xg_get_camera_calib`

**接口**

| | 签名 |
|---|------|
| C++ | `Error Device::getCameraCalib(int cameraIndex, CameraCalib* out)` |
| C | `xg_err_t xg_get_camera_calib(xg_device_t* dev, int camera_id, xg_camera_calib_t* out)` |

**作用：** 查询指定相机标定参数。`cameraIndex` / `camera_id`：**0–3**，标定文件 `camera_N`，**不是** [`CameraId`](#35-相机标识-cameraid)。数据结构见 [§4.1](#41-相机标定-cameracalib)。

**返回值**

| 返回 | 条件 |
|------|------|
| `Ok` | 找到对应 index |
| `InvalidArg` | 空、index 非 0–3、`struct_size` 错 |
| `Timeout` | 无应答 |
| `Protocol` | JSON 坏或找不到该 index |

### 8.2 `getImuCalib` / `xg_get_imu_calib`

**接口**

| | 签名 |
|---|------|
| C++ | `Error Device::getImuCalib(ImuCalib* out)` |
| C | `xg_err_t xg_get_imu_calib(xg_device_t* dev, xg_imu_calib_t* out)` |

**作用：** 查询 IMU 标定参数。数据结构见 [§4.2](#42-imu-标定-imucalib)。
**返回值：** `Ok` / `InvalidArg` / `Timeout` / `Protocol`。缺节点时对应数组保持 0。

### 8.3 `getLidarCalib` / `xg_get_lidar_calib`

**接口**

| | 签名 |
|---|------|
| C++ | `Error Device::getLidarCalib(LidarCalib* out)` |
| C | `xg_err_t xg_get_lidar_calib(xg_device_t* dev, xg_lidar_calib_t* out)` |

**作用：** 查询雷达标定参数。数据结构见 [§4.3](#43-雷达标定-lidarcalib)。
**返回值：** 同 [8.2](#82-getimucalib--xg_get_imu_calib)。

---

## 9. 设备状态接口

### 9.1 `getDeviceInfo` / `xg_get_device_info`

**接口**

| | 签名 |
|---|------|
| C++ | `Error Device::getDeviceInfo(DeviceInfo* out)` |
| C | `xg_err_t xg_get_device_info(xg_device_t* dev, xg_device_info_t* out)` |

**作用：** 查询设备电量/存储/内存/RTK/建图状态等。JSON method 实为 `getDeviceStatus`。数据结构见 [§4.4](#44-设备状态-deviceinfo)。

**返回值**

| 返回 | 条件 |
|------|------|
| `Ok` | 解析到 `data` 对象（设备 `code=-1` 时 SDK **仍可能 Ok** 且字段为 0，以实现为准） |
| `InvalidArg` | 空 / `struct_size` |
| `Timeout` | 无应答 |
| `Protocol` | 非对象或无 `data` |

`modelingState` 不是五态，不要当 `RecordState` 用；录制请用 `queryRecordStatus`。

### 9.2 `getDeviceTime` / `xg_get_device_time`

**接口**

| | 签名 |
|---|------|
| C++ | `Error Device::getDeviceTime(DeviceTime* out)` |
| C | `xg_err_t xg_get_device_time(xg_device_t* dev, xg_device_time_t* out)` |

**作用：** 只读查询设备系统时钟，并给出它与本机时钟的偏差区间。数据结构见 [§4.4.1](#441-设备时间-devicetime)，**与各订阅时间戳的换算关系见 [§0.8](#08-时间戳时基与换算)**。

**线路复用：** 发送与 [§9.1](#91-getdeviceinfo--xg_get_device_info) 相同的 `getDeviceStatus` 命令，设备在应答 `data` 里追加 `time` 对象：

```json
{ "method": "getDeviceStatus",
  "data": { "code": 0, "...": "原有字段不变",
            "time": { "epoch_us": 1791795901300521, "utc_offset_s": 28800 } } }
```

**兼容性**

| 组合 | 行为 |
|------|------|
| 新 SDK + 新固件 | 正常 |
| 新 SDK + 旧固件 | 应答里没有 `time`，返回 `Protocol` |
| 旧 SDK（1.0.0）+ 新固件 | 旧 SDK 只读自己认识的键，多出的 `time` 被忽略，`getDeviceInfo` 不受影响 |

**返回值**

| 返回 | 条件 |
|------|------|
| `Ok` | 应答含合法 `data.time`。设备状态查询失败（`data.code != 0`）时设备仍会带回 `time`，此时同样返回 `Ok` |
| `InvalidArg` | `dev` 或 `out` 为空；`struct_size` 不匹配 |
| `Timeout` | 无应答（查询超时 3 s，加 0.5 s 余量，调用最长阻塞 3.5 s） |
| `Protocol` | 应答非 JSON、无 `data`、无 `time`（旧固件）、`epoch_us` 或 `utc_offset_s` 缺失、不是整数或超出 int64 范围、`epoch_us` 小于等于 0 或晚于 3000-01-01 00:00:00 UTC、`utc_offset_s` 超出 ±86400 |
| `Internal` | SDK 内部异常 |

**线程：** 同步阻塞调用线程，最长 3.5 s；同一句柄不可重入（见 [§0.3](#03-线程与寿命)）。

**内存与寿命：** `out` 由调用方分配并持有，SDK 不保留指针。

**失败后状态：** 除 `struct_size` 不匹配外，返回非 `Ok` 时 `out` 除 `struct_size` 外全部置 0（含 `dev` 为空），不得使用；`struct_size` 不匹配时无法确定 `out` 的大小，`out` 保持原样。C++ 的 `getDeviceTime` 失败时一律把 `DeviceTime` 清零。设备端无任何状态变化。

**禁止用法**

- 不要把 `device_epoch_us` 当作 MCU 时钟或雷达时钟的读数，也不要用它换算 IMU、位姿、点云的时间戳，原因见 [§0.8](#08-时间戳时基与换算)。
- 本接口只读，SDK 不提供设置设备时间的能力。

**精度：** 偏差只能得到区间 `[device_epoch_us - host_recv_epoch_us, device_epoch_us - host_send_epoch_us]`，宽度等于 `rtt_us`。设备在状态查询完成后才采样时间，所以真实偏差更靠近区间下界（状态查询越慢越明显）；中点仅作粗略估计。区间宽度见 [§0.8](#08-时间戳时基与换算)；往返时延的实际分布未在真机上测量。

---

## 10. 定位初值接口

### 10.1 `setLocInitPose` / `xg_set_loc_init_pose`

**接口**

| | 签名 |
|---|------|
| C++ | `Error Device::setLocInitPose(const InitPose& pose)` |
| C | `xg_err_t xg_set_loc_init_pose(xg_device_t* dev, const xg_init_pose_t* pose)` |

**作用：** 向已加载地图的定位库注入世界系位姿先验，并启动设备端 Locate 算法。设备端时序为：录制启动时初始化（`startLocate`，即 `locateInit`）-> 本接口喂入初值（`setInitPose`）并启动算法（`startLocateAlgo`，即 `locateStart`）。这是启动 Locate 的必经步骤：调用前设备不发布 `device_locate`（位姿）和 `device_undist_cloud`。
**前置：** 设备已 Recording（此时初始化已完成）。未初始化时设备应答 `code=-1`、`reason=start locate failed`，SDK 返回 `Rejected`，初值不会被缓存。同一录制内重复调用：设备回 `code=0` 并跳过重复启动，新初值是否触发重新定位由定位库决定，未测。
**入参：** [`InitPose` / `xg_init_pose_t`](#45-定位初值-initpose)；C 须 `struct_size`。不发送 `origin`（设备侧写死 0）。

**返回值**

| 返回 | 条件 |
|------|------|
| `Ok` | 应答 `data.code == 0`（**已收下 JSON**） |
| `InvalidArg` | 空 / `struct_size` |
| `Timeout` | 无应答 |
| `Rejected` | `code != 0`（缺字段、四元数模长 ≤ 0.1、`locate not found`、`start locate failed` 等） |
| `Protocol` | 非 JSON 或无 `data` |

`Ok` **不是**重定位完成（只表示初值已收下、算法已启动），也**没有**阶段查询。

---

## 11. 工程路径接口

### 11.1 `getDataPath` / `xg_get_data_path`

**接口**

| | 签名 |
|---|------|
| C++ | `Error Device::getDataPath(std::string* path)` |
| C | `xg_err_t xg_get_data_path(xg_device_t* dev, xg_data_path_t* out)` |

**作用：** 查询当前工程的录制数据存储路径。JSON method 实为 `checkDataPackagePath`。录制中为当前工程；未录制可能为空或上一工程。数据结构见 [§4.6](#46-工程路径-xg_data_path_t)。

**返回值**

| 返回 | 条件 |
|------|------|
| `Ok` | 有 `data`（`path` 可为空串） |
| `InvalidArg` | 空 / `struct_size` / C++ `path == nullptr` |
| `Timeout` | 无应答 |
| `Protocol` | 解析失败 |

---

## 12. 辅助接口

| 接口 | 返回 | 说明 |
|------|------|------|
| `xg_err_str` / `toString(Error)` | `const char*` | 静态，勿 free；未知码 `"unknown error"` |
| `xg_record_state_str` / `toString(RecordState)` | `const char*` | 静态 |
| `xg_sdk_version` / `sdkVersion` | `"1.1.0"` | C++ 为 `std::string` 拷贝 |

**版本宏（`robot_sdk_c.h`）：** `XG_SDK_VERSION_MAJOR` / `MINOR` / `PATCH` 自 0.1.0 起提供；以下两个宏 1.1.0 新增，均由这三个宏派生。

| 宏 | 类型 | 说明 |
|----|------|------|
| `XG_SDK_VERSION_STRING` | 字符串字面量，如 `"1.1.0"` | 编译期头文件版本；与运行时 `xg_sdk_version()` 不同即为头文件与库不配套 |
| `XG_SDK_VERSION_NUMBER` | 整数常量表达式，`MAJOR*10000 + MINOR*100 + PATCH`，如 1.1.0 为 `10100` | 可用于 `#if`，按头文件版本条件编译（如 `#if XG_SDK_VERSION_NUMBER >= 10100`）。只反映编译期头文件，**禁止**用它判断运行时库版本 |

内部辅助宏 `XG_SDK_VERSION_STR_` / `XG_SDK_VERSION_STR_IMPL_` 不属于公开接口，禁止直接使用。

---

# 附录

## 13. 订阅能力一览

各订阅的数据结构见第 3 章，接口与生命周期见第 7 章。

| 数据 | 结构体 | 订阅接口 | C++ 接口 | 引入 | 频率（量级） | 指针寿命 |
|------|--------|----------|----------|------|--------------|----------|
| Body 位姿 | [3.1](#31-位姿-pose) | [7.1](#71-位姿订阅) | `subscribePose` | 0.2.0 | ~100 Hz | 结构体按值；无堆指针 |
| Locate 去畸变点云 | [3.3](#33-点云帧-cloudframe) | [7.2](#72-点云订阅) | `subscribeDeviceUndistCloud` | 1.1.0（新增；0.3.1 的 `subscribeRawCloud` 已删除，语义不同：旧接口是 SLAM 去畸变单帧、1/4 降采样，见 [v1.1.0.md](releases/v1.1.0.md) Changed 第 3 条） | 随 Locate 输出（未验证） | `points` 仅回调内 |
| 原始雷达点云 | [3.3.1](#331-原始点云-rawcloudframe) | [7.2.1](#721-原始雷达点云订阅) | `subscribeDeviceRawCloud` | 1.1.0 | ~10 Hz，仅 RoboSense | `points` 仅回调内 |
| 原始 IMU | [3.4](#34-惯导-imu) | [7.3](#73-imu-订阅) | `subscribeImu` | 0.3.1* | ~200 Hz | 按值 |
| JPEG 图像 | [3.6](#36-图像帧-imageframe) | [7.4](#74-图像订阅) | `subscribeImage` | 0.3.1* | 帧率以设备配置为准（实测 10 Hz） | `jpegData` 仅回调内 |
| 错误码 | [3.7](#37-错误码-errcodeframe) | [7.5](#75-错误码订阅) | `subscribeErrCode` | 0.3.1 | ~10 Hz | C++ 已拷贝 vector |

`*` 头注释写 v0.3.0，仓库随 0.3.1 首次出现。

---

## 14. 未发布能力（禁止当 API 用）

- 重定位阶段、NDT `score`/`overlap`、`RelocOutType.pose`
- `is_local` 的真实有图/无图（当前恒 0）
- 地图上传 / 列表 / 切换
- 身份认证

---

## 15. 修订

| 日期 | 说明 |
|------|------|
| 2026-09-03 | 1.0.0 接口说明书：数据结构板块 + 接口板块 + 附录。位姿/reloc 点云为设备（IMU）系（`device_locate` / `reloc_device_cloud`）；`is_local` 为保留字段（恒 0）。 |
| 2026-09-29 | 点云接口改名（破坏性）：`subscribeCloud` → `subscribeDeviceRelocCloud`（key `device_reloc_cloud`），`subscribeRawCloud` → `subscribeDeviceUndistCloud`（key `device_undist_cloud`）；修正原表述“去畸变原始帧，未经 SLAM / 雷达原始系”：该路是 SLAM 去畸变单帧、设备（IMU）系、1/4 降采样。新增版本宏 `XG_SDK_VERSION_STRING` / `XG_SDK_VERSION_NUMBER`。 |
| 2026-10-09 | 新增 `subscribeDeviceRawCloud`（key `device_raw_cloud`）：RoboSense 全量、未降采样、未去畸变点云，设备（IMU）系，逐点 `time_offset_us` / `ring`；新类型 `xg_raw_point_t`（24 字节）/ `xg_raw_cloud_frame_t`。纯增量，随 1.1.0 发布。 |
| 2026-10-12 | 文档版本对齐 1.1.0（标题、适用版本、`xg_sdk_version` 返回值）。新增发布说明 [releases/v1.1.0.md](releases/v1.1.0.md)。本说明书尚无 `xg_subscribe_depth` / `xg_unsubscribe_depth`、`xg_set_device_network` / `xg_confirm_device_network`、地图管理 `xg_upload_map` / `xg_set_active_map` / `xg_list_maps` / `xg_delete_map` 的条目，这些接口的契约以 [`robot_sdk_c.h`](../include/xg/robot_sdk_c.h) 为准。 |
| 2026-10-12 | 新增只读设备时间查询 `getDeviceTime` / `xg_get_device_time`（[§9.2](#92-getdevicetime--xg_get_device_time)）与数据结构 `xg_device_time_t`（[§4.4.1](#441-设备时间-devicetime)），新增 [§0.8 时间戳时基与换算](#08-时间戳时基与换算)。更正原始点云时间戳「设备时钟」的说法：实际取自雷达自身时钟，不保证等于设备系统时钟。纯增量，随 1.1.0 发布；需要设备固件在 `getDeviceStatus` 应答里带 `time`。 |
| 2026-10-14 | `device_undist_cloud` 语义变更（locate-full-cloud-feed）：由 SLAM 单帧去畸变变为 Locate 去畸变点云（设备（IMU）系）；key 由 `device_undistort_cloud` 改为 `device_undist_cloud`；删除 `subscribeDeviceRelocCloud` / `xg_subscribe_device_reloc_cloud`（key `device_reloc_cloud`）。回调耗时未验证。 |
| 2026-10-22 | 补充 §7 开头列表中的深度图数据可用性（默认配置 `ai.fps: 0` 时设备不发布 `depth/center`）；清理已删除重定位点云的残留表述（§3.3、§7.2、§7.2.1、§13）；§3.3 `seq` 说明补充点云丢帧判断方式。纯文档。 |

