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
 * 连接设备后订阅原始 IMU 数据（~200Hz），持续打印加速度和角速度。
 *
 * 运行：
 *   ./imu_demo <设备IP> [持续秒数]
 *   ./imu_demo 192.168.123.103 30
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
            return 1;
        }
    }
    if (!g_running.load()) { return 0; }

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
    std::printf("\n总计接收 %lu 帧 IMU 数据\n", (unsigned long)g_count.load());
    return 0;
}
