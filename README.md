# X-Brain SDK

用于远程控制 XGRIDS 设备录制并订阅实时数据的 C++ SDK。

- 语言：C++11 及以上（C++17 推荐）
- 版本：1.0.0

本仓库直接包含双架构预编译动态库，clone 后按目标机架构选用即可，无需另行下载：

| 架构 | 动态库目录 | 示例程序目录 | 运行要求 |
|---|---|---|---|
| x86_64 | `lib/x86_64/` | `bin/x86_64/` | glibc ≥ 2.34 |
| aarch64 | `lib/aarch64/` | `bin/aarch64/` | glibc ≥ 2.30 |

头文件（`include/`）与接口在两架构间完全一致，只是动态库的二进制架构不同。部署前请先确认目标机的 glibc 版本满足要求：

```bash
ldd --version | head -1
uname -m
```

glibc 版本不足时的表现是程序启动即失败（提示找不到 `GLIBC_2.xx`），不是运行期出错。

## 快速开始

```cpp
#include "xg/robot_sdk.h"
#include <iostream>
#include <thread>

int main() {
    xg::Options opts;
    opts.deviceIp = "192.168.123.103";

    xg::Error err = xg::Error::Ok;
    auto device = xg::Device::open(opts, &err);
    if (!device) {
        std::cerr << "连接失败: " << xg::toString(err) << std::endl;
        return 1;
    }

    // 下发命令, 立即返回。返回 Ok 只表示设备已接受, 不代表录制已开始
    if (device->startRecord() != xg::Error::Ok) {
        std::cerr << "命令下发失败" << std::endl;
        return 1;
    }

    // 轮询确认录制是否真正开始
    xg::RecordStatus status;
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (device->queryRecordStatus(&status) != xg::Error::Ok) {
            continue;   // 单次查询失败可重试
        }
        if (status.state == xg::RecordState::Recording) {
            break;      // 录制已开始
        }
        if (status.state == xg::RecordState::Error) {
            std::cerr << "启动失败: " << status.message << std::endl;
            return 1;
        }
        // Idle / Starting 都表示仍在准备, 继续等待
    }

    device->stopRecord();
    return 0;
}
```

**启停接口不等待状态变化**，确认结果需要自行轮询。原因见下方[为什么不代为等待](#为什么不代为等待)。

编译（`<arch>` 取 `x86_64` 或 `aarch64`，与目标机架构一致）：

```bash
g++ -std=c++11 main.cpp -Iinclude -Llib/<arch> -llixel_sdk \
    -Wl,-rpath,'$ORIGIN/lib/<arch>' -o myapp
```

`-Wl,-rpath` 让程序在运行时按相对路径找到 `liblixel_sdk.so`，不加则需要设置 `LD_LIBRARY_PATH`。用绝对路径也可以：

```bash
g++ -std=c++11 main.cpp -Iinclude -Llib/x86_64 -llixel_sdk \
    -Wl,-rpath,/opt/x-brain-sdk/lib/x86_64 -o myapp
```

完整示例见 `examples/record/record_demo.cpp`，预编译产物可直接运行：

```bash
./bin/x86_64/record_demo 192.168.123.103 30
```

## 数据订阅

SDK 支持订阅设备的实时数据流：位姿、点云、IMU、图像、错误码。SDK 内部完成 Protobuf 反序列化，集成方通过回调获取 POD 结构体，无需链接 protobuf。

### 订阅能力汇总

| 数据类型 | SDK 接口 | 频率 | 坐标系 |
|----------|----------|------|--------|
| 位姿 | `subscribePose` | ~100Hz | 设备（IMU）系 |
| 重定位点云 | `subscribeCloud` | ~10Hz | 设备（IMU）系 |
| 原始 IMU | `subscribeImu` | ~200Hz | 设备（IMU）系 |
| 原始点云 | `subscribeRawCloud` | ~10Hz | 雷达去畸变帧 |
| 图像（左/中/右） | `subscribeImage` | ~2fps | — |
| 错误码 | `subscribeErrCode` | ~10Hz | — |

> 位姿与重定位点云均以**设备（IMU）系**为基准，载体安装变换由集成方自行叠加，详见 [docs/coordinate_frames_and_tf.md](docs/coordinate_frames_and_tf.md)。重定位点云是 NDT 配准产物（~10Hz），非原始雷达扫描。

### 数据类型

**IMU 数据**：

```cpp
struct Imu {
    double timestampSec;     // 时间戳（秒）
    float accX, accY, accZ;  // 加速度 (m/s^2)
    float gyrX, gyrY, gyrZ;  // 角速度 (rad/s)
};
```

**位姿 / 点云**：位姿见 `xg::Pose`；点云复用 `xg::CloudFrame`。原始点云为去畸变后的完整雷达帧。

### 快速示例

```cpp
#include "xg/robot_sdk.h"
#include <chrono>
#include <cstdio>
#include <thread>

int main() {
    xg::Options opts;
    opts.deviceIp = "192.168.123.103";

    auto device = xg::Device::open(opts);
    if (!device) {
        return 1;
    }

    device->subscribePose([](const xg::Pose& pose) {
        std::printf("pose t=%.3f pos=(%.3f, %.3f, %.3f)\n",
                    pose.timestampSec, pose.posX, pose.posY, pose.posZ);
    });

    device->subscribeImu([](const xg::Device::Imu& imu) {
        std::printf("imu t=%.6f acc=(%.3f, %.3f, %.3f)\n",
                    imu.timestampSec, imu.accX, imu.accY, imu.accZ);
    });

    std::this_thread::sleep_for(std::chrono::seconds(30));
    return 0;
}
```

订阅示例程序：

```bash
./bin/x86_64/subscribe_demo 192.168.123.103 60    # 订阅 60 秒
```

完整示例源码见 `examples/subscribe/`。

### 使用注意

- **回调在 SDK 内部线程触发**，实现须尽快返回，不可阻塞。若需耗时处理，应将数据拷贝到自有队列后立即返回。
- **IMU 回调频率很高（~200Hz）**，原始点云数据量较大（数万点/帧），如需持久化建议异步写入。
- 同一设备句柄每种数据类型最多一个活跃订阅，重复订阅返回 `Error::Rejected`。
- 不订阅时不消耗网络带宽。
- `Device` 析构时自动取消所有活跃订阅。
- 数据订阅与录制控制接口互不影响，可同时使用。
- Protobuf 依赖已静态并入 SDK 库内部，集成方无需安装 protobuf。

## 交付内容

```
include/xg/robot_sdk.h        C++ 接口（推荐使用，header-only）
include/xg/robot_sdk_c.h      C 接口（ABI 真源）
lib/x86_64/  lib/aarch64/     动态库（按架构分目录，已内含全部第三方依赖）
bin/x86_64/  bin/aarch64/     预编译示例程序（按架构分目录）
examples/                     示例源码（connection/record/subscribe/relocation_map/comprehensive）
ros2/                         ROS2 Bridge 源码（colcon build 即用）
docs/                         用户文档（API 参考、数据类型、坐标系、发布说明等）
README.md
```

头文件（`include/`）、示例源码（`examples/`）、ROS2 桥接（`ros2/`）两架构通用；仅 `lib/` 与 `bin/` 按架构分目录，按目标机架构选用。`liblixel_sdk.so` 已包含全部第三方依赖，除系统库（glibc、libstdc++）外无需额外部署。

`bin/<arch>/` 下的预编译示例已内嵌 rpath，**保持仓库 `bin/<arch>` 与 `lib/<arch>` 的相对目录结构不变，即可直接运行**（如 `./bin/x86_64/record_demo`），无需设置 `LD_LIBRARY_PATH`。

系统库依赖只有 glibc 与 libstdc++（`pthread`、`dl`、`m`、`rt` 均属其中），常见 Linux 发行版均自带。可自行核对：

```bash
readelf -d lib/x86_64/liblixel_sdk.so | grep NEEDED
```

## 网络要求

设备与调用方需处于**同一交换机、同一网段**，本版本不支持跨网段或经公网访问。

```
设备          192.168.123.103/24
调用方主机    192.168.123.x/24     任选，避开设备地址与 192.168.123.100
```

SDK 只通过 TCP 直连设备上的**一个端口**，该端口必须放通：

| 端口 | 设备端进程 | 本 SDK 的用途 |
|---|---|---|
| 7448 | modeling_app | 数据订阅、录制控制与状态查询（SDK 全部能力都走它） |

SDK 需要的数据面（位姿/点云/IMU/图像/错误码）与命令面（录制控制、状态查询）都由 modeling_app 在同一个 zenoh session 内提供，SDK **只连接 modeling_app 的 7448** 即可拿到全部数据。SDK 不使用组播或任何自动发现机制，因此交换机或防火墙拦截组播不影响使用，但该 TCP 端口若被拦则无法连接。

`Device::open()` 会实际访问一次设备作为连通性探测，返回成功即代表链路已通、设备端服务已就绪，调用方不需要额外等待或重试。同网段实测 `open()` 耗时约 10 毫秒。

连接失败时的排查顺序：

```bash
ping 192.168.123.103        # 网线与网段
nc -z 192.168.123.103 7448  # modeling_app 端口，SDK 唯一需要的端口
```

## 接口

### 连接

```cpp
xg::Options opts;
opts.deviceIp = "192.168.123.103";  // 必填
opts.port = 0;                      // 0 表示默认 7448（modeling_app），见下方说明
opts.connectTimeoutMs = 0;          // 0 表示默认 5000 毫秒

xg::Error err;
auto device = xg::Device::open(opts, &err);   // 失败返回 nullptr
```

`Device` 析构时自动断开连接。禁止拷贝与移动，避免句柄被重复释放。

一个 `Device` 对应一台设备，本版本不支持同时连接多台设备。

**关于 `opts.port`：直接填 modeling_app 的端口，默认 7448。** 保持 `0` 即用默认 7448，绝大多数设备无需改动。仅当设备端调整过 modeling_app 的端口时，才填其实际端口。

### 开始录制

```cpp
xg::Error startRecord();
```

**只下发命令，不等待录制真正开始。** 返回 `Error::Ok` 表示设备已接受命令，接下来必须轮询 `queryRecordStatus()`，等到状态变为 `Recording` 才能认为录制已开始。

设备已在录制时重复调用返回 `Error::Ok`（幂等）。

### 停止录制

```cpp
xg::Error stopRecord();
```

语义与 `startRecord` 对称，同样需要轮询确认，等到状态变为 `Idle`。停止过程涉及数据落盘，耗时与数据量相关。

设备本就未在录制时返回 `Error::Ok`（幂等）。

### 查询状态

```cpp
xg::Error queryRecordStatus(RecordStatus* status);
```

不改变设备状态，可安全轮询。这是确认启停结果的唯一途径。**轮询间隔不要小于 500 毫秒**，可用 `XG_RECOMMENDED_POLL_INTERVAL_MS`。

### 为什么不代为等待

设备从收到命令到真正开始录制，需要初始化定位与建图，耗时与**地图规模、存储状态**等因素相关，实测从数秒到数十秒不等，没有一个对所有场景都成立的上限。

如果 SDK 内部按固定超时等待，超时值定短了就会在设备正常启动的过程中返回失败 —— 制造出一个并不存在的故障，比不等待更糟。所以等多久、如何向操作者反馈进度，交给调用方按自身业务判断。

### 轮询范式

`examples/record/record_demo.cpp` 里的 `waitForState()` 可以直接拿去用，要点有四条：

```cpp
bool waitForState(xg::Device* device, xg::RecordState target,
                  long maxWaitMs, xg::RecordStatus* status) {
    const auto t0 = std::chrono::steady_clock::now();
    while (elapsedMs(t0) < maxWaitMs) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(XG_RECOMMENDED_POLL_INTERVAL_MS));

        if (device->queryRecordStatus(status) != xg::Error::Ok) {
            continue;   // 要点 1: 查询失败不等于操作失败, 继续重试
        }
        if (status->state == target) {
            return true;
        }
        if (status->state == xg::RecordState::Error) {
            return false;   // 要点 2: 故障态立即退出, 不必等满
        }
        // 要点 3: Idle 与 Starting 都表示仍在准备, 继续等
    }
    return false;   // 要点 4: 等待上限到了不代表失败, 见下文
}
```

1. **单次查询失败不代表操作失败。** 可能只是一次通信抖动，应该重试而不是判定录制失败。
2. **状态为 `Error` 时立即退出。** 设备已明确失败，继续等待没有意义。
3. **`Idle` 和 `Starting` 都要继续等。** 命令刚下发时状态可能还是 `Idle`，不要据此判定失败。
4. **等待上限到了也不代表失败。** 命令已经下发，设备可能仍在准备。此时应继续观察或提示操作者，**不要重复下发** `startRecord()`。

`maxWaitMs` 该给多大？实测参考值（空载无地图）：

| 操作 | 状态变化 | 从下发到确认 |
|---|---|---|
| `startRecord()` | `Idle → Starting → Recording` | 约 8 秒 |
| `stopRecord()` | `Recording → Stopping → Idle` | 约 5.5 秒 |

**这只是参考值，不要当成上限写死。** 启动耗时随地图规模增长（设备需加载地图、做配置校验后才真正开始出数据），大地图场景会明显更久。建议取实测值的数倍作为上限，并在等待期间向操作者显示进度，而不是卡住界面。

## 状态

```cpp
enum class RecordState {
    Idle,       // 待机，可以开始录制
    Starting,   // 启动中，尚未开始出数据
    Recording,  // 录制中
    Stopping,   // 停止中
    Error,      // 故障，见 message
};
```

```cpp
struct RecordStatus {
    RecordState state;
    bool        warning;   // 存在需关注的异常，但不阻塞当前操作
    int32_t     rawCode;   // 设备原始状态码，仅供排障
    std::string message;   // 人可读说明
};
```

### warning 与 Error 的区别

**`warning` 不阻塞操作，`Error` 阻塞操作。** 这是判断该继续还是该中止的依据。

`warning` 为真时不需要中断作业，但建议记入日志：

| 场景 | state | 含义 |
|---|---|---|
| 建图过程异常 | `Recording` | 录制仍在继续，已退化为仅录包模式，可正常停止 |
| 上次工程后处理异常 | `Idle` | 设备本身可用，不影响本次作业 |

`state` 为 `Error` 时设备处于故障状态（存储、相机、雷达、内存等），需要先处理故障才能录制，具体原因见 `message`。

### rawCode

设备内部原始状态码，**仅用于排障和日志记录**。该值反映设备内部状态机，不保证跨版本稳定，请勿作为业务判断依据 —— 业务逻辑一律基于 `state` 与 `warning`。

## 错误码

```cpp
enum class Error {
    Ok,            // 成功
    InvalidArg,    // 参数非法
    NotConnected,  // 未连接到设备
    Timeout,       // 超时
    Rejected,      // 设备拒绝，当前状态不允许该操作
    DeviceError,   // 设备处于故障状态，详见 RecordStatus
    Protocol,      // 设备应答无法解析，通常为版本不匹配
    Internal,      // SDK 内部错误
};
```

`xg::toString(err)` 可取得可读说明。

### Timeout 的正确处理方式

`startRecord` / `stopRecord` 返回 `Timeout` 表示**没收到设备应答**，此时命令是否已送达设备无从判断。

不要直接重试，否则可能重复下发。应先查询确认：

```cpp
if (device->startRecord() == xg::Error::Timeout) {
    // 命令可能已送达, 先确认实际状态再决定是否重发
    xg::RecordStatus status;
    if (device->queryRecordStatus(&status) == xg::Error::Ok) {
        if (status.state != xg::RecordState::Idle) {
            // 已在准备或已开始, 转入正常轮询即可
        }
    }
}
```

`Rejected` 表示设备明确拒绝了命令，此时应查询状态判断原因。注意「已在录制时启动」和「未录制时停止」都按幂等处理返回 `Ok`，不会得到 `Rejected`。

## 线程安全

**同一个 `Device` 实例的方法不可并发调用**，多线程使用时请由调用方加锁。不同实例之间互不影响。

## C 接口

需要 C 接口或自行封装到其他语言时，直接使用 `xg/robot_sdk_c.h`。C++ 接口是它的 header-only 封装，两者能力完全一致。

```c
#include "xg/robot_sdk_c.h"

xg_options_t opts;
memset(&opts, 0, sizeof(opts));
opts.struct_size = sizeof(opts);          /* 必填 */
snprintf(opts.device_ip, sizeof(opts.device_ip), "192.168.123.103");

xg_device_t* dev = NULL;
if (xg_device_open(&opts, &dev) != XG_OK) {
    return 1;
}

if (xg_record_start(dev) != XG_OK) {      /* 只下发, 不等待 */
    xg_device_close(dev);
    return 1;
}

/* 轮询确认 */
xg_record_status_t st;
for (;;) {
    usleep(XG_RECOMMENDED_POLL_INTERVAL_MS * 1000);
    memset(&st, 0, sizeof(st));
    st.struct_size = sizeof(st);          /* 每次调用前必填 */
    if (xg_record_query(dev, &st) != XG_OK) {
        continue;
    }
    if (st.state == XG_REC_RECORDING) {
        break;
    }
    if (st.state == XG_REC_ERROR) {
        break;
    }
}

xg_device_close(dev);
```

**两个结构体都必须先填 `struct_size`**，否则接口返回 `XG_ERR_INVALID_ARG`。这个字段用于结构体在末尾追加字段时保持二进制兼容，C++ 封装已代为处理。

## 设计说明

**C++ 接口是 header-only 的。** `robot_sdk.h` 中所有函数均为内联，由调用方自行编译，不产生需要链接的符号。这样做是为了规避 C++ ABI 不兼容问题 —— 编译器版本、libstdc++ 版本、`_GLIBCXX_USE_CXX11_ABI` 取值若与 SDK 构建环境不一致，`std::string` 等类型跨越库边界会导致链接失败或运行期崩溃。SDK 库本身只导出 C 接口，故无此风险。

**动态库只导出 `xg_` 前缀符号。** 第三方依赖已静态并入并隐藏，调用方工程即便自带同名库的其他版本也不会冲突。可自行核对：

```bash
nm -D --defined-only lib/x86_64/liblixel_sdk.so | grep -v ' xg_'
```

**只提供动态库。** 动态库配合链接脚本能把内部符号彻底隐藏，静态库做不到这一点 —— 静态库的符号在调用方链接时全部可见，需要额外做部分链接与符号局部化处理，复杂度与收益不成比例。

## 安全须知

**本版本不含身份认证，设备控制接口对同网段完全开放。** 同一交换机上的任何主机都能连接设备并控制录制。

因此：

- 设备**不得**接入不可信网络（公共网络、访客网络、直连互联网）
- 建议设备与调用方主机使用独立交换机或专用 VLAN，与办公网隔离

## 兼容性

升级 SDK 时请同步更新头文件与库文件，两者版本必须一致，可用 `xg::sdkVersion()` 核对。

## 常见问题

**连接失败（`Timeout`）**

按顺序检查：网线是否连通、主机是否与设备同网段、设备端服务是否已启动。用 `ping` 和 `nc -z <ip> 7448` 可以逐层定位。

**`startRecord` 返回 `Ok`，但状态一直不变成 `Recording`**

设备启动录制的耗时与地图规模等因素相关，可能需要数十秒。轮询过程中状态为 `Idle` 或 `Starting` 都属正常，继续等待即可。若变成 `Error`，`message` 会指明原因（存储挂载失败、存储空间不足、相机异常、雷达数据异常、内存不足等），需要先排除故障。

**不要因为等得久就重复调用 `startRecord()`。** 命令已经下发，重复下发不会加快启动。

**`startRecord` 返回 `Timeout`**

这是通信层面没收到应答，与设备启动快慢无关。先 `queryRecordStatus()` 确认命令是否已生效，再决定是否重发。

**`state` 一直是 `Error`、`message` 为 `device not ready, state unknown`**

设备端服务尚未完成初始化，等待数十秒后重试。若持续如此，需要检查设备端配置。
