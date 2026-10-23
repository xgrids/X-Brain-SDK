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
 * @brief  XGRIDS Robot SDK 示例：自动录制 + 全量数据订阅 + 文件保存（可单独运行出数据）
 *
 * 同时订阅位姿、IMU、Locate 去畸变点云和三路图像，持续打印并保存到磁盘，
 * Ctrl+C 退出后停止录制。位姿和去畸变点云要 Locate 在跑才有数据，而 Locate 只在
 * setLocInitPose 之后才启动，所以本示例在订阅之后自带初值注入。
 *
 * 前置条件：
 *   - 设备已开机、应用已起来、状态为 Idle（先用 device_info_demo 确认连得上）。
 *   - 设备上有可用的重定位地图，[X] [Y] [Z] 与设备当前实际位置大致相符；
 *     否则重定位可能不收敛，位姿和去畸变点云为 0 帧，IMU 和图像不受影响。
 *   - 图像要设备 jpeg.fps > 0（以设备配置为准）。
 *
 * 调用顺序与每步为什么：
 *   1. Device::open 连接。
 *   2. startRecord + 轮询到 Recording：IMU、图像等数据流依赖录制状态，Locate 在录制
 *      启动时完成初始化（进入 Recording 前已完成）。
 *   3. 订阅全部数据流：必须在注入之前订阅，否则设备一启动 Locate 就发布的前几帧会丢。
 *   4. setLocInitPose(X,Y,Z, 单位四元数)：设备做 setInitPose + startLocateAlgo，
 *      一条命令同时给初值并启动 Locate；之后才有位姿和去畸变点云。
 *   5. 持续接收，每 10 s 打印一行统计；Ctrl+C 后取消订阅并 stopRecord。
 *
 * 保存内容：
 *   <输出目录>/pose.csv           位姿轨迹
 *   <输出目录>/undist_NNNNNN.pcd   设备(IMU)系 Locate 去畸变点云
 *   <输出目录>/left_NNNNNN.jpg    左相机图像
 *   <输出目录>/center_NNNNNN.jpg  中相机图像
 *   <输出目录>/right_NNNNNN.jpg   右相机图像
 *
 * 运行（本示例没有持续时间参数，Ctrl+C 退出）：
 *   ./subscribe_demo <设备IP> [输出目录|-] [X=0] [Y=0] [Z=0]
 *   ./subscribe_demo 192.168.123.103
 *   ./subscribe_demo 192.168.123.103 /tmp/sdk_data 1 1 0.45
 *   ./subscribe_demo 192.168.123.103 - 1 1 0.45     # 不保存，只改初值
 *   X/Y/Z 单位 m，世界/地图系，默认 0 0 0（地图原点），与 reloc_demo 一致；
 *   朝向固定为单位四元数 (w=1,x=0,y=0,z=0)。-h / --help 打印用法。
 *
 * 预期输出与实测数字（数字随固件版本变化，仅供参考）：
 *   - IMU 约 199 Hz；三路图像各约 10 Hz（实测 10.007 Hz，帧率以设备
 *     jpeg.fps 配置为准，旧注释的默认 2 fps 已过时）；
 *   - 注入 (1,1,0.45) 后 5.6~6.9 s 重定位成功（本示例实测 5.64 s），随后位姿约
 *     100 Hz，去畸变点云约 9.7 Hz、每帧 19.0k 点（范围 17.1k~19.3k）。
 *   - 注入前的纯录制实测：位姿 0 帧、去畸变点云 0 帧，IMU 6162 帧、图像 L305/C309/R307
 *     （40 s）。
 *
 * 失败现象与处理：
 *   - "注入失败: Rejected"：设备拒绝，常见为定位尚未初始化、无地图或四元数非法；
 *     确认已 Recording、设备上有地图后重跑。
 *   - "注入失败: Timeout"：命令可能已送达，不要立刻重复注入，先看统计行。
 *   - 结束时位姿或去畸变点云 0 帧：Locate 未启动或重定位未成功（见 reloc_demo）。
 *   - 同时订阅 IMU、三路图像并在回调里写文件时，设备端曾记录到 IMU 发布耗时超阈值
 *     （原因未判定）；主机侧 IMU 未见丢帧。只要看数据，不保存目录即可降低负载。
 *   - 启动耗时：从雷达待机起约 17.5~18.6 s，新进程约 7.5~8.5 s。
 *
 * 副作用：
 *   - 会 startRecord，每次运行在设备上生成一个录制工程（本机实测 87~103 MB/次）。
 *   - 退出时 stopRecord；stopRecord 之后设备应用会退出并由 systemd 重启，约
 *     10~20 s 内设备不可用。下一次运行请先用 device_info_demo 确认连得上且状态
 *     为 Idle；重启期间下发的 startRecord 会被排队，收尾的 stopRecord 在非录制态
 *     是空操作。
 *
 * 线程模型：回调在 Zenoh RX 线程触发，回调内不得阻塞。本示例为演示直接在回调里
 * 写 PCD/JPEG，会拖慢接收线程，正式集成请把数据拷贝入队，由工作线程落盘。
 */

#include "xg/robot_sdk.h"

#include <atomic>
#include <chrono>
#include <cmath>
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
std::atomic<uint32_t> g_imuCount{0};
std::atomic<uint32_t> g_undistCloudCount{0};
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

/** 严格解析浮点参数：整串必须是有限数，否则返回 false */
bool parseFloatArg(const char* text, float* out) {
    char* end = nullptr;
    const float v = std::strtof(text, &end);
    if (end == text || *end != '\0' || !std::isfinite(v)) {
        return false;
    }
    *out = v;
    return true;
}

/**
 * @brief 注入定位初值（单位四元数），失败时打印原因和处理建议。
 * @return 设备已收下初值返回 true
 * @note   Ok 只表示设备收下了初值并启动 Locate，不代表重定位完成。
 */
bool injectInitPose(xg::Device* device, float x, float y, float z) {
    xg::Device::InitPose init;
    init.posX = x;
    init.posY = y;
    init.posZ = z;
    init.quatW = 1.0f;  // 单位四元数, 模长 = 1 (> 0.1 约束)
    init.quatX = 0.0f;
    init.quatY = 0.0f;
    init.quatZ = 0.0f;
    const xg::Error err = device->setLocInitPose(init);
    if (err == xg::Error::Ok) {
        std::printf("      设备已接受初值 (仅表示已收下, 不代表重定位完成)\n");
        return true;
    }
    std::printf("      注入失败: %s\n", xg::toString(err));
    if (err == xg::Error::Rejected) {
        std::printf("      原因: 设备拒绝 (定位未初始化 / 无地图 / 四元数非法)。\n"
                    "      建议: 确认设备已 Recording 且设备上有可用地图后重跑。\n");
    } else if (err == xg::Error::Timeout) {
        std::printf("      原因: 超时, 命令可能已送达。\n"
                    "      建议: 不要立刻重复注入, 先观察统计行里位姿是否增长。\n");
    } else {
        std::printf("      建议: 检查与设备的连接后重跑。\n");
    }
    return false;
}

void printUsage(const char* prog) {
    std::printf("XGRIDS Robot SDK 全量数据订阅示例 (SDK %s)\n\n",
                xg::sdkVersion().c_str());
    std::printf("用法:\n");
    std::printf("  %s <设备IP> [输出目录|-] [X=0] [Y=0] [Z=0]\n", prog);
    std::printf("  X/Y/Z: 定位初值位置, 单位 m, 世界/地图系, 默认 0 0 0\n\n");
    std::printf("流程: 连接 → 启动录制 → 订阅全部数据 → 注入定位初值 → "
                "持续打印+保存 → Ctrl+C 退出\n\n");
    std::printf("保存内容:\n");
    std::printf("  <输出目录>/pose.csv           位姿轨迹\n");
    std::printf("  <输出目录>/undist_NNNNNN.pcd   去畸变点云(设备系)\n");
    std::printf("  <输出目录>/left_NNNNNN.jpg    左相机图像\n");
    std::printf("  <输出目录>/center_NNNNNN.jpg  中相机图像\n");
    std::printf("  <输出目录>/right_NNNNNN.jpg   右相机图像\n");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc >= 2 && (std::strcmp(argv[1], "-h") == 0 ||
                      std::strcmp(argv[1], "--help") == 0)) {
        printUsage(argv[0]);
        return 0;
    }
    if (argc < 2 || argc > 6) {
        printUsage(argv[0]);
        return 1;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const std::string deviceIp = argv[1];
    if (argc >= 3 && std::strcmp(argv[2], "-") != 0) {
        g_outputDir = argv[2];
    }
    float posX = 0.0f, posY = 0.0f, posZ = 0.0f;
    if ((argc >= 4 && !parseFloatArg(argv[3], &posX)) ||
        (argc >= 5 && !parseFloatArg(argv[4], &posY)) ||
        (argc >= 6 && !parseFloatArg(argv[5], &posZ))) {
        std::printf("参数错误: X/Y/Z 必须是数字 (单位 m)\n");
        printUsage(argv[0]);
        return 1;
    }
    if (!g_outputDir.empty()) {
        mkdirp(g_outputDir);
    }

    // ---- 1. 连接设备 ----
    std::printf("[1/5] 连接设备 %s ...\n", deviceIp.c_str());
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
    std::printf("[2/5] 启动录制...\n");
    err = device->startRecord();
    if (err != xg::Error::Ok) {
        std::printf("      启动录制失败: %s\n", xg::toString(err));
        if (err == xg::Error::Timeout) {
            // 超时不代表命令没送达：设备可能已经开始录制。补发一次停止，
            // 避免设备被留在录制状态（停止在非录制态是幂等的）
            device->stopRecord();
        }
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
            device->stopRecord();
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
    std::printf("[3/5] 订阅数据流...\n");

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


    // IMU
    device->subscribeImu([](const xg::Device::Imu& imu) {
        uint32_t n = g_imuCount.fetch_add(1) + 1;
        if (n % 200 == 1) {
            std::printf("  [IMU %5u] t=%.6f acc=(%7.3f,%7.3f,%7.3f)\n",
                        n, imu.timestampSec, imu.accX, imu.accY, imu.accZ);
        }
    });
    std::printf("      IMU ✓ (~200Hz)\n");

    // 设备(IMU)系 Locate 去畸变点云（Locate 启动前无数据, 注入见第 4 步）
    device->subscribeDeviceUndistCloud([](const xg::CloudFrame& frame) {
        uint32_t n = g_undistCloudCount.fetch_add(1) + 1;
        std::printf("  [UndistCloud %3u] t=%.3f points=%zu\n",
                    n, frame.timestampSec, frame.count);
        savePcd(frame, n, "undist");
    });
    std::printf("      去畸变点云 ✓ (~10Hz)\n");

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
        std::printf("      %s 图像 ✓ (帧率以设备配置为准)\n", g_cameraNames[i]);
    }

    if (!g_outputDir.empty()) {
        std::printf("\n      数据保存到: %s\n", g_outputDir.c_str());
    }
    std::printf("\n");

    // ---- 4. 注入定位初值 ----
    // 订阅已就绪, 现在注入才不丢首帧; 失败不退出, IMU 和图像仍可正常观察
    std::printf("[4/5] 注入定位初值 pos=(%.3f, %.3f, %.3f) "
                "quat=(w=1, x=0, y=0, z=0)...\n", posX, posY, posZ);
    injectInitPose(device.get(), posX, posY, posZ);
    std::printf("\n");

    // ---- 5. 持续接收 ----
    std::printf("[5/5] 接收数据中... (Ctrl+C 退出)\n");
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
                        "去畸变点云:%u 图像:L%u/C%u/R%u ---\n\n",
                        curPose, (double)(curPose - lastPose) / elapsed,
                        curImu, (double)(curImu - lastImu) / elapsed,
                        g_undistCloudCount.load(),
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
    device->unsubscribeImu();
    device->unsubscribeDeviceUndistCloud();
    for (int i = 0; i < 3; ++i) {
        device->unsubscribeImage(cameras[i]);
    }

    if (g_poseCsv) {
        std::fclose(g_poseCsv);
    }

    device->stopRecord();
    std::printf("  已停止录制\n");

    std::printf("\n总计: 位姿=%u IMU=%u 去畸变点云=%u 图像=L%u/C%u/R%u\n",
                g_poseCount.load(), g_imuCount.load(),
                g_undistCloudCount.load(),
                g_imageCount[0].load(), g_imageCount[1].load(),
                g_imageCount[2].load());
    if (!g_outputDir.empty()) {
        std::printf("数据已保存到: %s\n", g_outputDir.c_str());
    }
    if (g_poseCount.load() == 0 || g_undistCloudCount.load() == 0) {
        std::printf("提示: 位姿或去畸变点云有 0 帧。原因: Locate 未启动或重定位未成功 "
                    "(注入失败 / 初值与实际位置相差过大 / 无地图), 见 reloc_demo。\n");
    }
    std::printf("完成\n");
    return 0;
}
