/******************************************************************************
This file is part of XGRIDS Robot SDK.
Project: lixel_sdk
Author : XGRIDS Embedded Software Team
*******************************************************************************
History:
1. create this file. (DATE: 20261009)
******************************************************************************/

/**
 * @file   raw_cloud_demo.cpp
 * @brief  XGRIDS Robot SDK 示例：设备(IMU)系原始雷达点云订阅
 *
 * 订阅 subscribeDeviceRawCloud（Zenoh key: device_raw_cloud）：
 * RoboSense 一次完整扫描的全部点，未降采样、未运动去畸变，点坐标为设备(IMU)系
 * （仅做了静态外参变换）。每个点带 ring 和相对帧时间的偏移 time_offset_us，
 * 集成方据此配合自己的位姿做去畸变。不依赖 Locate，不需要注入初值，单独运行
 * 就有数据。
 *
 * 前置条件：
 *   - 设备已开机、应用已起来、状态为 Idle（先用 device_info_demo 确认）。
 *   - 仅 RoboSense 雷达发布该 key。getLidarCalib().lidarType 在本机为空（设备缺
 *     lidar.yaml），不能用它判断雷达型号；本机雷达实为 RoboSense（设备日志
 *     RoboSenseClient）。收不到帧时请看设备日志。
 *
 * 调用顺序与每步为什么：
 *   1. Device::open 连接。
 *   2. startRecord + 轮询到 Recording：雷达数据流依赖录制状态。
 *   3. subscribeDeviceRawCloud，每帧打印点数、逐点时间偏移范围和前 3 个点。
 *   4. 采集 [持续秒数]，退出时 unsubscribeDeviceRawCloud + stopRecord。
 *
 * 运行：
 *   ./raw_cloud_demo <设备IP> [持续秒数=60] [保存目录]
 *   ./raw_cloud_demo 192.168.123.103 30
 *   ./raw_cloud_demo 192.168.123.103 30 /tmp/raw_clouds
 *   保存目录不存在时自动创建一级目录。
 *
 * 预期输出与实测数字（raw_cloud_demo 30；数字随固件版本变化，仅供参考）：
 *   - 30 s 收 300 帧，10.000 Hz，间隔 99.9~100.1 ms，设备 seq 连续；
 *   - 每帧平均 80.6k 点（范围 73.2k~81.6k）；time_offset_us 每帧最小 0、
 *     最大约 99.9~100.1 ms；ring 0~95；
 *   - 帧时间戳取雷达时钟，不是设备系统时钟，不要直接用 getDeviceTime 的偏差换算；
 *     时基见 docs/api_reference_v2.md §0.8。
 *   - 点云围绕传感器原点分布（实测距原点 0.5 m 内的点占 9.79%）。
 *
 * 失败现象与处理：
 *   - 0 帧：非 RoboSense 雷达，或没进入 Recording，或雷达数据异常；看设备日志。
 *   - 从雷达待机起跑时设备日志可能有 1 条"lidar data abnormal time"，是丢弃一帧
 *     时间异常的雷达帧，属正常。
 *   - 保存 PCD 单帧约 7.7 万点的 ASCII 写盘，会明显拖慢接收线程，仅用于调试。
 *   - 启动耗时：从雷达待机起约 17.5~18.6 s，新进程约 7.5~8.5 s。
 *
 * 副作用：
 *   - 会 startRecord，每次运行在设备上生成一个录制工程（本机实测 87~103 MB/次）。
 *   - 退出时 stopRecord；stopRecord 之后设备应用会退出并由 systemd 重启，约
 *     10~20 s 内设备不可用。下一次运行请先用 device_info_demo 确认连得上且状态
 *     为 Idle；重启期间下发的 startRecord 会被排队，收尾的 stopRecord 在非录制态
 *     是空操作。
 *
 * 线程模型：回调在 Zenoh RX 线程触发，回调内不得阻塞。单帧点数多，回调内只做
 * 统计和可选保存；正式集成请把帧拷贝入队，由工作线程处理。
 */

#include "xg/robot_sdk.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sys/stat.h>
#include <thread>

namespace {
std::atomic<bool> g_running{true};
std::atomic<uint32_t> g_count{0};
std::string g_saveDir;

void signalHandler(int) { g_running.store(false); }

/** 保存为 ASCII PCD，字段含 ring 和 time_offset_us（单位 us，相对帧时间） */
void savePcd(const xg::RawCloudFrame& frame, uint32_t n) {
    if (g_saveDir.empty()) {
        return;
    }
    char path[512];
    std::snprintf(path, sizeof(path), "%s/raw_cloud_%06u.pcd",
                  g_saveDir.c_str(), n);
    FILE* fp = std::fopen(path, "w");
    if (!fp) {
        return;
    }
    std::fprintf(fp, "# .PCD v0.7\n");
    std::fprintf(fp, "VERSION 0.7\n");
    std::fprintf(fp, "FIELDS x y z intensity ring time_offset_us\n");
    std::fprintf(fp, "SIZE 4 4 4 4 2 4\n");
    std::fprintf(fp, "TYPE F F F F U I\n");
    std::fprintf(fp, "COUNT 1 1 1 1 1 1\n");
    std::fprintf(fp, "WIDTH %zu\n", frame.count);
    std::fprintf(fp, "HEIGHT 1\n");
    std::fprintf(fp, "VIEWPOINT 0 0 0 1 0 0 0\n");
    std::fprintf(fp, "POINTS %zu\n", frame.count);
    std::fprintf(fp, "DATA ascii\n");
    for (size_t i = 0; i < frame.count; ++i) {
        const xg_raw_point_t& p = frame.points[i];
        std::fprintf(fp, "%.6f %.6f %.6f %.1f %u %d\n", p.x, p.y, p.z,
                     p.intensity, static_cast<unsigned>(p.ring),
                     static_cast<int>(p.time_offset_us));
    }
    std::fclose(fp);
    std::printf("  -> 已保存 %s (%zu 点)\n", path, frame.count);
}
}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("用法: %s <设备IP> [持续秒数] [保存目录]\n", argv[0]);
        std::printf("  保存目录非空时，每帧保存为 ASCII PCD 文件\n");
        return 1;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const std::string deviceIp = argv[1];
    int duration = (argc >= 3) ? std::atoi(argv[2]) : 60;
    if (argc >= 4) {
        g_saveDir = argv[3];
    }
    if (!g_saveDir.empty()) {
        mkdir(g_saveDir.c_str(), 0755);  // 已存在时返回失败, 无需处理
    }

    // 连接设备
    xg::Options opts;
    opts.deviceIp = deviceIp;
    xg::Error err = xg::Error::Ok;
    auto device = xg::Device::open(opts, &err);
    if (!device) {
        std::printf("连接失败: %s\n", xg::toString(err));
        return 1;
    }
    std::printf("已连接 %s (SDK %s)\n\n", deviceIp.c_str(),
                xg::sdkVersion().c_str());

    // 启动录制（传感器数据流依赖录制状态）
    std::printf("启动录制...\n");
    err = device->startRecord();
    if (err != xg::Error::Ok) {
        std::printf("启动录制失败: %s\n", xg::toString(err));
        if (err == xg::Error::Timeout) {
            // 超时不代表命令没送达：设备可能已经开始录制。补发一次停止，
            // 避免设备被留在录制状态（停止在非录制态是幂等的）
            device->stopRecord();
        }
        return 1;
    }
    xg::RecordStatus recStatus;
    int waitTicks = 0;
    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (device->queryRecordStatus(&recStatus) != xg::Error::Ok) {
            continue;
        }
        // 每 2 秒打印一次状态；设备拒绝开始录制（例如处于 USB 存储模式）时
        // 状态会一直是 Idle，没有这行输出会像卡死
        if (++waitTicks % 4 == 0) {
            std::printf("  等待录制开始... 当前状态 state=%d (0=Idle,1=Starting,2=Recording)\n",
                        static_cast<int>(recStatus.state));
        }
        if (recStatus.state == xg::RecordState::Recording) {
            std::printf("设备已进入录制状态\n\n");
            break;
        }
        if (recStatus.state == xg::RecordState::Error) {
            std::printf("录制失败: %s\n", recStatus.message.c_str());
            device->stopRecord();
            return 1;
        }
    }
    // 等待录制期间收到 Ctrl+C：startRecord 已下发，必须停掉，否则设备会一直录
    if (!g_running.load()) {
        device->stopRecord();
        return 0;
    }

    // 订阅设备(IMU)系原始雷达点云
    auto rc = device->subscribeDeviceRawCloud([](const xg::RawCloudFrame& frame) {
        uint32_t n = g_count.fetch_add(1) + 1;
        // 统计帧内逐点时间偏移范围，便于确认逐点时间可用于去畸变
        int32_t minOff = 0;
        int32_t maxOff = 0;
        if (frame.count > 0) {
            minOff = maxOff = frame.points[0].time_offset_us;
        }
        for (size_t i = 1; i < frame.count; ++i) {
            const int32_t off = frame.points[i].time_offset_us;
            if (off < minOff) { minOff = off; }
            if (off > maxOff) { maxOff = off; }
        }
        std::printf("[RawCloud %3u] t=%.6f seq=%u points=%zu offset=[%d, %d] us\n",
                    n, frame.timestampSec, frame.seq, frame.count,
                    static_cast<int>(minOff), static_cast<int>(maxOff));
        // 打印前 3 个点
        for (size_t i = 0; i < frame.count && i < 3; ++i) {
            const xg_raw_point_t& p = frame.points[i];
            std::printf("    [%zu] (%.3f, %.3f, %.3f) i=%.1f ring=%u dt=%d us\n",
                        i, p.x, p.y, p.z, p.intensity,
                        static_cast<unsigned>(p.ring),
                        static_cast<int>(p.time_offset_us));
        }
        savePcd(frame, n);
    });
    if (rc != xg::Error::Ok) {
        std::printf("原始点云订阅失败: %s\n", xg::toString(rc));
        device->stopRecord();
        return 1;
    }
    std::printf("原始点云订阅成功 (~10Hz, 仅 RoboSense 设备有数据)\n");
    std::printf("持续 %d 秒, Ctrl+C 提前退出\n", duration);
    if (!g_saveDir.empty()) {
        std::printf("保存到: %s\n", g_saveDir.c_str());
    }
    std::printf("\n");

    auto start = std::chrono::steady_clock::now();
    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= duration) {
            break;
        }
    }

    device->unsubscribeDeviceRawCloud();
    device->stopRecord();
    std::printf("\n总计接收 %u 帧原始点云\n", g_count.load());
    return 0;
}
