/******************************************************************************
This file is part of XGRIDS Robot SDK.
Project: lixel_sdk
Author : XGRIDS Embedded Software Team
*******************************************************************************
History:
1. create this file. (DATE: 20260826)
******************************************************************************/

/**
 * @file   image_demo.cpp
 * @brief  XGRIDS Robot SDK 示例：相机图像订阅
 *
 * 连接设备后订阅三路相机 JPEG 图像，打印帧信息，可选保存 JPEG 文件。
 *
 * 运行：
 *   ./image_demo <设备IP> [持续秒数] [保存目录]
 *   ./image_demo 192.168.123.103 30
 *   ./image_demo 192.168.123.103 60 /tmp/images
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
std::atomic<uint32_t> g_count[3] = {{0}, {0}, {0}};
std::string g_saveDir;
const char* g_cameraNames[] = {"left", "center", "right"};

void signalHandler(int) { g_running.store(false); }

void saveJpeg(int idx, uint32_t seq, const uint8_t* data, size_t size) {
    if (g_saveDir.empty()) {
        return;
    }
    char path[512];
    std::snprintf(path, sizeof(path), "%s/%s_%06u.jpg",
                  g_saveDir.c_str(), g_cameraNames[idx], seq);
    FILE* fp = std::fopen(path, "wb");
    if (fp) {
        std::fwrite(data, 1, size, fp);
        std::fclose(fp);
    }
}
}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("用法: %s <设备IP> [持续秒数] [保存目录]\n", argv[0]);
        std::printf("  保存目录非空时，每帧保存为 JPEG 文件\n");
        std::printf("\n注意: 需要设备端启用 JPEG 输出 (jpeg.fps > 0)\n");
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

    // 启动录制（相机流依赖录制状态）
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
            std::printf("录制启动失败: %s\n", recStatus.message.c_str());
            return 1;
        }
    }
    if (!g_running.load()) { return 0; }

    // 订阅三路相机
    using CamId = xg::Device::CameraId;
    CamId cameras[] = {CamId::Left, CamId::Center, CamId::Right};

    for (int i = 0; i < 3; ++i) {
        auto rc = device->subscribeImage(cameras[i],
            [i](const xg::Device::ImageFrame& frame) {
                uint32_t n = g_count[i].fetch_add(1) + 1;
                std::printf("[%s %3u] t=%.3f  jpeg=%zu bytes\n",
                            g_cameraNames[i], n,
                            frame.timestampSec, frame.jpegSize);
                saveJpeg(i, n, frame.jpegData, frame.jpegSize);
            });
        if (rc == xg::Error::Ok) {
            std::printf("  %s 订阅成功\n", g_cameraNames[i]);
        } else {
            std::printf("  %s 订阅失败: %s\n", g_cameraNames[i], xg::toString(rc));
        }
    }

    std::printf("\n持续 %d 秒, Ctrl+C 提前退出\n", duration);
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

    // 取消订阅
    for (int i = 0; i < 3; ++i) {
        device->unsubscribeImage(cameras[i]);
    }
    device->stopRecord();

    std::printf("\n总计: left=%u  center=%u  right=%u\n",
                g_count[0].load(), g_count[1].load(), g_count[2].load());
    return 0;
}
