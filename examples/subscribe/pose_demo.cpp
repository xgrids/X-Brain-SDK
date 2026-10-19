/******************************************************************************
This file is part of XGRIDS Robot SDK.
Project: lixel_sdk
Author : XGRIDS Embedded Software Team
*******************************************************************************
History:
1. create this file. (DATE: 20260826)
******************************************************************************/

/**
 * @file   pose_demo.cpp
 * @brief  XGRIDS Robot SDK 示例：位姿数据订阅
 *
 * 连接设备后订阅 Body 系位姿（~100Hz），持续打印位置、姿态和速度。
 * 可选将位姿轨迹保存为 CSV 文件。
 *
 * 运行：
 *   ./pose_demo <设备IP> [持续秒数] [输出CSV路径]
 *   ./pose_demo 192.168.123.103 30
 *   ./pose_demo 192.168.123.103 60 /tmp/trajectory.csv
 */

#include "xg/robot_sdk.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>
#include <thread>

namespace {
std::atomic<bool> g_running{true};
std::atomic<uint64_t> g_count{0};
FILE* g_csvFile{nullptr};
std::mutex g_csvMutex;

void signalHandler(int) { g_running.store(false); }
}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("用法: %s <设备IP> [持续秒数] [输出CSV]\n", argv[0]);
        return 1;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const std::string deviceIp = argv[1];
    int duration = (argc >= 3) ? std::atoi(argv[2]) : 60;
    if (argc >= 4) {
        g_csvFile = std::fopen(argv[3], "w");
        if (g_csvFile) {
            std::fprintf(g_csvFile,
                "timestamp,pos_x,pos_y,pos_z,quat_w,quat_x,quat_y,quat_z,"
                "vel_x,vel_y,vel_z,gyro_x,gyro_y,gyro_z,error_code,is_local\n");
        }
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

// 订阅位姿
    auto rc = device->subscribePose([](const xg::Pose& p) {
        uint64_t n = g_count.fetch_add(1) + 1;
        // 每 100 帧打印一次（约 1Hz）
        if (n % 100 == 1) {
            std::printf("[Pose %5lu] t=%.3f  pos=(%7.3f, %7.3f, %7.3f)  "
                        "vel=(%6.3f, %6.3f, %6.3f)  err=%u %s\n",
                        (unsigned long)n, p.timestampSec,
                        p.posX, p.posY, p.posZ,
                        p.velX, p.velY, p.velZ,
                        p.errorCode, p.isLocal ? "[local]" : "");
        }
        // 写 CSV
        if (g_csvFile) {
            std::lock_guard<std::mutex> lock(g_csvMutex);
            std::fprintf(g_csvFile,
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%d\n",
                p.timestampSec, p.posX, p.posY, p.posZ,
                p.quatW, p.quatX, p.quatY, p.quatZ,
                p.velX, p.velY, p.velZ,
                p.gyroX, p.gyroY, p.gyroZ,
                p.errorCode, p.isLocal ? 1 : 0);
        }
    });
    if (rc != xg::Error::Ok) {
        std::printf("位姿订阅失败: %s\n", xg::toString(rc));
        return 1;
    }
    std::printf("位姿订阅成功 (~100Hz, 每秒打印1次)\n");
    std::printf("持续 %d 秒, Ctrl+C 提前退出\n", duration);
    if (g_csvFile) {
        std::printf("轨迹保存到: %s\n", argv[3]);
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

    device->unsubscribePose();
    device->stopRecord();
    if (g_csvFile) {
        std::fclose(g_csvFile);
    }
    std::printf("\n总计接收 %lu 帧位姿数据\n", (unsigned long)g_count.load());
    return 0;
}
