/******************************************************************************
This file is part of XGRIDS Robot SDK.
Project: lixel_sdk
Author : XGRIDS Embedded Software Team
*******************************************************************************
History:
1. create this file. (DATE: 20260826)
******************************************************************************/

/**
 * @file   imu_demo.cpp
 * @brief  XGRIDS Robot SDK 示例：IMU 数据订阅
 *
 * 订阅原始 IMU 数据（Zenoh key: raw_imu，~200Hz），持续打印加速度和角速度。
 * IMU 不依赖 Locate，不需要注入初值，单独运行就有数据。
 *
 * 前置条件：设备已开机、应用已起来、状态为 Idle（先用 device_info_demo 确认）。
 *
 * 调用顺序与每步为什么：
 *   1. Device::open 连接。
 *   2. startRecord + 轮询到 Recording：IMU 数据流依赖录制状态，Recording 之前不发布。
 *   3. subscribeImu，每 200 帧（约 1 s）打印一行。
 *   4. 采集 [持续秒数]，退出时 unsubscribeImu + stopRecord。
 *
 * 运行：
 *   ./imu_demo <设备IP> [持续秒数=60]
 *   ./imu_demo 192.168.123.103 30
 *
 * 预期输出与实测数字（imu_demo 20；数字随固件版本变化，仅供参考）：
 *   - 20 s 收 3999 帧，199.9 Hz（采样行局部频率 197~200 Hz）；
 *   - |acc| 均值约 9.79 m/s^2（静止时约等于重力），|gyr| 均值约 0.025 rad/s；
 *   - 启动耗时：从雷达待机起约 17.5~18.6 s，新进程约 7.5~8.5 s。
 *
 * 失败现象与处理：
 *   - 收到 0 帧：先确认确实进入了 Recording（"设备已进入录制状态"），再查设备网络。
 *   - startRecord 超时：本示例会补发 stopRecord，避免设备被留在录制态。
 *
 * 副作用：
 *   - 会 startRecord，每次运行在设备上生成一个录制工程（本机实测 87~103 MB/次）。
 *   - 退出时 stopRecord；stopRecord 之后设备应用会退出并由 systemd 重启，约
 *     10~20 s 内设备不可用。下一次运行请先用 device_info_demo 确认连得上且状态
 *     为 Idle；重启期间下发的 startRecord 会被排队，收尾的 stopRecord 在非录制态
 *     是空操作。
 *
 * 线程模型：回调在 Zenoh RX 线程触发，回调内不得阻塞（本示例只计数和打印）。
 */

#include "xg/robot_sdk.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <thread>

namespace {
std::atomic<bool> g_running{true};
std::atomic<uint64_t> g_count{0};

void signalHandler(int) { g_running.store(false); }
}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("用法: %s <设备IP> [持续秒数, 默认60]\n", argv[0]);
        return 1;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const std::string deviceIp = argv[1];
    int duration = (argc >= 3) ? std::atoi(argv[2]) : 60;

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

    // 启动录制
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
    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (device->queryRecordStatus(&recStatus) != xg::Error::Ok) {
            continue;
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
    // startRecord 已下发, 提前退出前必须停掉, 否则设备会一直录
    if (!g_running.load()) {
        device->stopRecord();
        return 0;
    }

    // 订阅 IMU
    auto rc = device->subscribeImu([](const xg::Device::Imu& imu) {
        uint64_t n = g_count.fetch_add(1) + 1;
        // 每 200 帧打印一次（约 1Hz）
        if (n % 200 == 1) {
            std::printf("[IMU %6lu] t=%.6f  acc=(%8.4f, %8.4f, %8.4f)  "
                        "gyr=(%7.4f, %7.4f, %7.4f)\n",
                        (unsigned long)n, imu.timestampSec,
                        imu.accX, imu.accY, imu.accZ,
                        imu.gyrX, imu.gyrY, imu.gyrZ);
        }
    });
    if (rc != xg::Error::Ok) {
        std::printf("IMU 订阅失败: %s\n", xg::toString(rc));
        device->stopRecord();
        return 1;
    }
    std::printf("IMU 订阅成功 (~200Hz, 每秒打印1次)\n");
    std::printf("持续 %d 秒, Ctrl+C 提前退出\n\n", duration);

    auto start = std::chrono::steady_clock::now();
    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= duration) {
            break;
        }
    }

    device->unsubscribeImu();
    // 与其他示例一致: 演示结束时停止录制, 否则设备会一直录
    device->stopRecord();
    std::printf("\n总计接收 %lu 帧 IMU 数据\n", (unsigned long)g_count.load());
    return 0;
}
