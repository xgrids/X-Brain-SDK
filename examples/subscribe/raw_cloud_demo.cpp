/******************************************************************************
This file is part of XGRIDS Robot SDK.
Project: lixel_sdk
Author : XGRIDS Embedded Software Team
*******************************************************************************
History:
1. create this file. (DATE: 20260826)
******************************************************************************/

/**
 * @file   raw_cloud_demo.cpp
 * @brief  XGRIDS Robot SDK 示例：原始点云数据订阅
 *
 * 连接设备后订阅原始点云数据（去畸变后、未经 SLAM 处理的雷达帧），
 * 每帧打印点数和前几个点坐标。可选保存为 PCD 文件。
 *
 * 运行：
 *   ./raw_cloud_demo <设备IP> [持续秒数] [保存目录]
 *   ./raw_cloud_demo 192.168.123.103 30
 *   ./raw_cloud_demo 192.168.123.103 30 /tmp/clouds
 */

#include "xg/robot_sdk.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

namespace {
std::atomic<bool> g_running{true};
std::atomic<uint32_t> g_count{0};
std::string g_saveDir;

void signalHandler(int) { g_running.store(false); }

void savePcd(const xg::CloudFrame& frame, uint32_t seq) {
    if (g_saveDir.empty()) {
        return;
    }
    char path[512];
    std::snprintf(path, sizeof(path), "%s/cloud_%06u.pcd",
                  g_saveDir.c_str(), seq);
    FILE* fp = std::fopen(path, "w");
    if (!fp) {
        return;
    }
    // 最简 ASCII PCD 格式
    std::fprintf(fp, "# .PCD v0.7\n");
    std::fprintf(fp, "VERSION 0.7\n");
    std::fprintf(fp, "FIELDS x y z intensity\n");
    std::fprintf(fp, "SIZE 4 4 4 4\n");
    std::fprintf(fp, "TYPE F F F F\n");
    std::fprintf(fp, "COUNT 1 1 1 1\n");
    std::fprintf(fp, "WIDTH %zu\n", frame.count);
    std::fprintf(fp, "HEIGHT 1\n");
    std::fprintf(fp, "VIEWPOINT 0 0 0 1 0 0 0\n");
    std::fprintf(fp, "POINTS %zu\n", frame.count);
    std::fprintf(fp, "DATA ascii\n");
    for (size_t i = 0; i < frame.count; ++i) {
        std::fprintf(fp, "%.6f %.6f %.6f %.1f\n",
                     frame.points[i].x, frame.points[i].y,
                     frame.points[i].z, frame.points[i].intensity);
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

// 订阅原始点云
    auto rc = device->subscribeRawCloud([](const xg::CloudFrame& frame) {
        uint32_t n = g_count.fetch_add(1) + 1;
        std::printf("[RawCloud %3u] t=%.3f  points=%zu\n",
                    n, frame.timestampSec, frame.count);
        // 打印前 5 个点
        for (size_t i = 0; i < frame.count && i < 5; ++i) {
            std::printf("    [%zu] (%.3f, %.3f, %.3f) i=%.1f\n",
                        i, frame.points[i].x, frame.points[i].y,
                        frame.points[i].z, frame.points[i].intensity);
        }
        if (frame.count > 5) {
            std::printf("    ... (省略 %zu 点)\n", frame.count - 5);
        }
        savePcd(frame, n);
    });
    if (rc != xg::Error::Ok) {
        std::printf("原始点云订阅失败: %s\n", xg::toString(rc));
        return 1;
    }
    std::printf("原始点云订阅成功 (频率取决于雷达帧率)\n");
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

    device->unsubscribeRawCloud();
    device->stopRecord();
    std::printf("\n总计接收 %u 帧原始点云\n", g_count.load());
    return 0;
}
