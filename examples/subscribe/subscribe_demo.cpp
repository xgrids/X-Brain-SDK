/******************************************************************************
This file is part of XGRIDS Robot SDK.
Project: lixel_sdk
Author : XGRIDS Embedded Software Team
*******************************************************************************
History:
1. create this file. (DATE: 20260815)
2. add auto-record and continuous print. (DATE: 20260816)
3. add image subscription and file save. (DATE: 20260826)
******************************************************************************/

/**
 * @file   subscribe_demo.cpp
 * @brief  XGRIDS Robot SDK 示例：自动录制 + 全量数据订阅 + 文件保存
 *
 * 流程：连接设备 → 启动录制 → 等待进入录制状态 → 订阅所有数据 →
 * 持续打印并保存到磁盘 → Ctrl+C 退出 → 停止录制。
 *
 * 保存内容：
 *   <输出目录>/pose.csv           位姿轨迹
 *   <输出目录>/cloud_NNNNNN.pcd   点云帧
 *   <输出目录>/left_NNNNNN.jpg    左相机图像
 *   <输出目录>/center_NNNNNN.jpg  中相机图像
 *   <输出目录>/right_NNNNNN.jpg   右相机图像
 *
 * 运行：
 *   ./subscribe_demo <设备IP> [输出目录]
 *   ./subscribe_demo 192.168.123.103
 *   ./subscribe_demo 192.168.123.103 /tmp/sdk_data
 */

#include "xg/robot_sdk.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <sys/stat.h>
#include <thread>

namespace {

std::atomic<bool> g_running{true};
std::atomic<uint32_t> g_poseCount{0};
std::atomic<uint32_t> g_cloudCount{0};
std::atomic<uint32_t> g_imuCount{0};
std::atomic<uint32_t> g_rawCloudCount{0};
std::atomic<uint32_t> g_imageCount[3] = {{0}, {0}, {0}};

std::string g_outputDir;
FILE* g_poseCsv{nullptr};
std::mutex g_csvMutex;

const char* g_cameraNames[] = {"left", "center", "right"};

void signalHandler(int) {
    g_running.store(false);
}

void mkdirp(const std::string& path) {
    mkdir(path.c_str(), 0755);
}

void savePcd(const xg::CloudFrame& frame, uint32_t seq, const char* prefix) {
    if (g_outputDir.empty()) {
        return;
    }
    char path[512];
    std::snprintf(path, sizeof(path), "%s/%s_%06u.pcd",
                  g_outputDir.c_str(), prefix, seq);
    FILE* fp = std::fopen(path, "w");
    if (!fp) {
        return;
    }
    std::fprintf(fp, "# .PCD v0.7\nVERSION 0.7\n");
    std::fprintf(fp, "FIELDS x y z intensity\n");
    std::fprintf(fp, "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n");
    std::fprintf(fp, "WIDTH %zu\nHEIGHT 1\n", frame.count);
    std::fprintf(fp, "VIEWPOINT 0 0 0 1 0 0 0\n");
    std::fprintf(fp, "POINTS %zu\nDATA ascii\n", frame.count);
    for (size_t i = 0; i < frame.count; ++i) {
        std::fprintf(fp, "%.6f %.6f %.6f %.1f\n",
                     frame.points[i].x, frame.points[i].y,
                     frame.points[i].z, frame.points[i].intensity);
    }
    std::fclose(fp);
}

void saveJpeg(int camIdx, uint32_t seq, const uint8_t* data, size_t size) {
    if (g_outputDir.empty()) {
        return;
    }
    char path[512];
    std::snprintf(path, sizeof(path), "%s/%s_%06u.jpg",
                  g_outputDir.c_str(), g_cameraNames[camIdx], seq);
    FILE* fp = std::fopen(path, "wb");
    if (fp) {
        std::fwrite(data, 1, size, fp);
        std::fclose(fp);
    }
}

void printUsage(const char* prog) {
    std::printf("XGRIDS Robot SDK 全量数据订阅示例 (SDK %s)\n\n",
                xg::sdkVersion().c_str());
    std::printf("用法:\n");
    std::printf("  %s <设备IP> [输出目录]\n\n", prog);
    std::printf("流程: 连接 → 启动录制 → 订阅全部数据 → 持续打印+保存 → Ctrl+C 退出\n\n");
    std::printf("保存内容:\n");
    std::printf("  <输出目录>/pose.csv           位姿轨迹\n");
    std::printf("  <输出目录>/cloud_NNNNNN.pcd   重定位点云\n");
    std::printf("  <输出目录>/raw_NNNNNN.pcd     原始点云\n");
    std::printf("  <输出目录>/left_NNNNNN.jpg    左相机图像\n");
    std::printf("  <输出目录>/center_NNNNNN.jpg  中相机图像\n");
    std::printf("  <输出目录>/right_NNNNNN.jpg   右相机图像\n");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const std::string deviceIp = argv[1];
    if (argc >= 3) {
        g_outputDir = argv[2];
        mkdirp(g_outputDir);
    }

    // ---- 1. 连接设备 ----
    std::printf("[1/4] 连接设备 %s ...\n", deviceIp.c_str());
    xg::Options opts;
    opts.deviceIp = deviceIp;

    xg::Error err = xg::Error::Ok;
    auto device = xg::Device::open(opts, &err);
    if (!device) {
        std::printf("连接失败: %s\n", xg::toString(err));
        return 1;
    }
    std::printf("      已连接\n\n");

    // ---- 2. 启动录制 ----
    std::printf("[2/4] 启动录制...\n");
    err = device->startRecord();
    if (err != xg::Error::Ok) {
        std::printf("      启动录制失败: %s\n", xg::toString(err));
        return 1;
    }
    std::printf("      命令已下发, 等待设备进入录制状态...\n");

    xg::RecordStatus status;
    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (device->queryRecordStatus(&status) != xg::Error::Ok) {
            continue;
        }
        if (status.state == xg::RecordState::Recording) {
            std::printf("      设备已进入录制状态\n\n");
            break;
        }
        if (status.state == xg::RecordState::Error) {
            std::printf("      录制启动失败: %s\n", status.message.c_str());
            return 1;
        }
        std::printf("      状态: %s, 继续等待...\n",
                    xg::toString(status.state));
    }
    if (!g_running.load()) {
        device->stopRecord();
        return 0;
    }

    // ---- 3. 订阅全部数据 ----
    std::printf("[3/4] 订阅数据流...\n");

    // 位姿 CSV
    if (!g_outputDir.empty()) {
        char csvPath[512];
        std::snprintf(csvPath, sizeof(csvPath), "%s/pose.csv", g_outputDir.c_str());
        g_poseCsv = std::fopen(csvPath, "w");
        if (g_poseCsv) {
            std::fprintf(g_poseCsv,
                "timestamp,pos_x,pos_y,pos_z,quat_w,quat_x,quat_y,quat_z,"
                "vel_x,vel_y,vel_z,gyro_x,gyro_y,gyro_z,error_code,is_local\n");
        }
    }

    // 位姿
    device->subscribePose([](const xg::Pose& p) {
        uint32_t n = g_poseCount.fetch_add(1) + 1;
        if (n % 100 == 1) {
            std::printf("  [Pose %5u] t=%.3f pos=(%7.3f,%7.3f,%7.3f) err=%u\n",
                        n, p.timestampSec, p.posX, p.posY, p.posZ, p.errorCode);
        }
        if (g_poseCsv) {
            std::lock_guard<std::mutex> lock(g_csvMutex);
            std::fprintf(g_poseCsv,
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%d\n",
                p.timestampSec, p.posX, p.posY, p.posZ,
                p.quatW, p.quatX, p.quatY, p.quatZ,
                p.velX, p.velY, p.velZ,
                p.gyroX, p.gyroY, p.gyroZ,
                p.errorCode, p.isLocal ? 1 : 0);
        }
    });
    std::printf("      位姿 ✓ (~100Hz)\n");

    // 重定位点云
    device->subscribeCloud([](const xg::CloudFrame& frame) {
        uint32_t n = g_cloudCount.fetch_add(1) + 1;
        std::printf("  [Cloud %3u] t=%.3f points=%zu\n",
                    n, frame.timestampSec, frame.count);
        savePcd(frame, n, "cloud");
    });
    std::printf("      重定位点云 ✓ (~1Hz)\n");

    // IMU
    device->subscribeImu([](const xg::Device::Imu& imu) {
        uint32_t n = g_imuCount.fetch_add(1) + 1;
        if (n % 200 == 1) {
            std::printf("  [IMU %5u] t=%.6f acc=(%7.3f,%7.3f,%7.3f)\n",
                        n, imu.timestampSec, imu.accX, imu.accY, imu.accZ);
        }
    });
    std::printf("      IMU ✓ (~200Hz)\n");

    // 原始点云
    device->subscribeRawCloud([](const xg::CloudFrame& frame) {
        uint32_t n = g_rawCloudCount.fetch_add(1) + 1;
        std::printf("  [RawCloud %3u] t=%.3f points=%zu\n",
                    n, frame.timestampSec, frame.count);
        savePcd(frame, n, "raw");
    });
    std::printf("      原始点云 ✓ (~10Hz)\n");

    // 三路图像
    using CamId = xg::Device::CameraId;
    CamId cameras[] = {CamId::Left, CamId::Center, CamId::Right};
    for (int i = 0; i < 3; ++i) {
        device->subscribeImage(cameras[i],
            [i](const xg::Device::ImageFrame& frame) {
                uint32_t n = g_imageCount[i].fetch_add(1) + 1;
                std::printf("  [%s %3u] t=%.3f jpeg=%zu bytes\n",
                            g_cameraNames[i], n,
                            frame.timestampSec, frame.jpegSize);
                saveJpeg(i, n, frame.jpegData, frame.jpegSize);
            });
        std::printf("      %s 图像 ✓ (~2fps)\n", g_cameraNames[i]);
    }

    if (!g_outputDir.empty()) {
        std::printf("\n      数据保存到: %s\n", g_outputDir.c_str());
    }
    std::printf("\n");

    // ---- 4. 持续接收 ----
    std::printf("[4/4] 接收数据中... (Ctrl+C 退出)\n");
    std::printf("──────────────────────────────────────────────────────\n\n");

    auto lastStats = std::chrono::steady_clock::now();
    uint32_t lastPose = 0, lastImu = 0;

    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            now - lastStats).count();
        if (elapsed >= 10) {
            uint32_t curPose = g_poseCount.load();
            uint32_t curImu = g_imuCount.load();
            std::printf("\n  --- [统计] 位姿:%u(%.0fHz) IMU:%u(%.0fHz) "
                        "点云:%u 原始点云:%u 图像:L%u/C%u/R%u ---\n\n",
                        curPose, (double)(curPose - lastPose) / elapsed,
                        curImu, (double)(curImu - lastImu) / elapsed,
                        g_cloudCount.load(), g_rawCloudCount.load(),
                        g_imageCount[0].load(), g_imageCount[1].load(),
                        g_imageCount[2].load());
            lastPose = curPose;
            lastImu = curImu;
            lastStats = now;
        }
    }

    // ---- 退出清理 ----
    std::printf("\n──────────────────────────────────────────────────────\n");
    std::printf("收到退出信号, 清理中...\n");

    device->unsubscribePose();
    device->unsubscribeCloud();
    device->unsubscribeImu();
    device->unsubscribeRawCloud();
    for (int i = 0; i < 3; ++i) {
        device->unsubscribeImage(cameras[i]);
    }

    if (g_poseCsv) {
        std::fclose(g_poseCsv);
    }

    device->stopRecord();
    std::printf("  已停止录制\n");

    std::printf("\n总计: 位姿=%u IMU=%u 点云=%u 原始点云=%u 图像=L%u/C%u/R%u\n",
                g_poseCount.load(), g_imuCount.load(),
                g_cloudCount.load(), g_rawCloudCount.load(),
                g_imageCount[0].load(), g_imageCount[1].load(),
                g_imageCount[2].load());
    if (!g_outputDir.empty()) {
        std::printf("数据已保存到: %s\n", g_outputDir.c_str());
    }
    std::printf("完成\n");
    return 0;
}
