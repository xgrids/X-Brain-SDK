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
 * @brief  XGRIDS Robot SDK 示例：位姿数据订阅（可单独运行出数据）
 *
 * 订阅设备（IMU）系位姿（~100Hz），持续打印位置、姿态和速度，可选保存为 CSV。
 * 设备端 Locate 只在 setLocInitPose 之后才启动，所以本示例自带初值注入，
 * 不需要先跑别的示例。
 *
 * 前置条件：
 *   - 设备已开机、应用已起来、状态为 Idle（先用 device_info_demo 确认连得上）。
 *   - 设备上有可用的重定位地图，且 [X] [Y] [Z] 与设备当前实际位置大致相符；
 *     位置不对时重定位可能不收敛，位姿能出数据但落点不是你要的位置。
 *
 * 调用顺序与每步为什么：
 *   1. Device::open 连接。
 *   2. startRecord + 轮询到 Recording：Locate 在录制启动时完成初始化
 *      （startLocate，进入 Recording 前已完成），不 Recording 就没有可注入的对象。
 *   3. subscribePose：必须在注入之前订阅，否则设备一启动算法就发布的前几帧会丢。
 *   4. setLocInitPose(X,Y,Z, 单位四元数)：设备做 setInitPose + startLocateAlgo，
 *      一条命令同时给初值并启动 Locate。之后才有 device_locate。
 *   5. 采集 [持续秒数]，退出时 unsubscribePose + stopRecord。
 *
 * 运行：
 *   ./pose_demo <设备IP> [持续秒数=60] [输出CSV路径|-] [X=0] [Y=0] [Z=0]
 *   ./pose_demo 192.168.123.103 30
 *   ./pose_demo 192.168.123.103 60 /tmp/trajectory.csv 1 1 0.45
 *   ./pose_demo 192.168.123.103 60 - 1 1 0.45        # 不存 CSV，只改初值
 *   X/Y/Z 单位 m，世界/地图系，默认 0 0 0（地图原点），与 reloc_demo 一致；
 *   朝向固定为单位四元数 (w=1,x=0,y=0,z=0)。-h / --help 打印用法。
 *
 * 预期输出与实测数字（注入 (1,1,0.45)；数字随固件版本变化，仅供参考）：
 *   - 注入后 5.6~6.9 s 重定位成功（实测 5.64/6.02/6.26/6.85 s），随后约 100 Hz
 *     出位姿（实测 100.00 Hz），err=0；
 *     本示例每 100 帧（约 1 s）打印一行。
 *   - 本机实测位姿稳定在约 (1.05, 4.01, 0.43) m，与注入值 Y 向相差约 3.0 m：
 *     那是设备在地图上匹配到的位置，不是接口缺数据。
 *   - 位姿时间戳时基见 docs/api_reference_v2.md §0.8，不要直接和图像时间戳相减。
 *
 * 失败现象与处理：
 *   - "注入失败: Rejected"：设备拒绝，常见为定位尚未初始化、无地图或四元数非法；
 *     确认已 Recording、设备上有地图后重跑。
 *   - "注入失败: Timeout"：命令可能已送达，不要立刻重复注入，先看是否有位姿输出。
 *   - 结束时"位姿 0 帧"：Locate 未启动或重定位未成功（见 reloc_demo，
 *     它会等待并观察收敛）。
 *   - startRecord 超时：本示例会补发 stopRecord 避免设备被留在录制态。
 *   - 启动耗时：从雷达待机起约 17.5~18.6 s，新进程约 7.5~8.5 s。
 *
 * 持续秒数语义：[持续秒数] 从注入返回后开始计时，包含重定位等待（实测 5.6~6.9 s）。
 * 例如 15 秒窗口实测只得到 8.74 s 的位姿（875 帧）；想要 N 秒位姿数据，
 * 请把持续秒数设为 N + 7。
 *
 * 副作用：
 *   - 会 startRecord，每次运行在设备上生成一个录制工程（本机实测 87~103 MB/次）。
 *   - 退出时 stopRecord；stopRecord 之后设备应用会退出并由 systemd 重启，约
 *     10~20 s 内设备不可用。下一次运行请先用 device_info_demo 确认连得上且状态
 *     为 Idle；重启期间下发的 startRecord 会被排队，收尾的 stopRecord 在非录制态
 *     是空操作。
 *
 * 线程模型：回调在 Zenoh RX 线程触发，回调内不得阻塞（不做耗时 IO、不等长锁）。
 * 本示例回调里只打印和写 CSV（小量写入），长期运行或高负载场景请改为入队。
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
#include <thread>

namespace {
std::atomic<bool> g_running{true};
std::atomic<uint64_t> g_count{0};
FILE* g_csvFile{nullptr};
std::mutex g_csvMutex;

void signalHandler(int) { g_running.store(false); }

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
                    "      建议: 不要立刻重复注入, 先观察是否有位姿输出。\n");
    } else {
        std::printf("      建议: 检查与设备的连接后重跑。\n");
    }
    return false;
}

void printUsage(const char* prog) {
    std::printf("用法: %s <设备IP> [持续秒数=60] [输出CSV|-] [X=0] [Y=0] [Z=0]\n",
                prog);
    std::printf("  X/Y/Z: 定位初值位置, 单位 m, 世界/地图系, 默认 0 0 0\n");
    std::printf("  输出CSV 填 - 表示不保存\n");
}
}  // namespace

int main(int argc, char** argv) {
    if (argc >= 2 && (std::strcmp(argv[1], "-h") == 0 ||
                      std::strcmp(argv[1], "--help") == 0)) {
        printUsage(argv[0]);
        return 0;
    }
    if (argc < 2 || argc > 7) {
        printUsage(argv[0]);
        return 1;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const std::string deviceIp = argv[1];
    int duration = (argc >= 3) ? std::atoi(argv[2]) : 60;
    const bool saveCsv = (argc >= 4 && std::strcmp(argv[3], "-") != 0);
    float posX = 0.0f, posY = 0.0f, posZ = 0.0f;
    if ((argc >= 5 && !parseFloatArg(argv[4], &posX)) ||
        (argc >= 6 && !parseFloatArg(argv[5], &posY)) ||
        (argc >= 7 && !parseFloatArg(argv[6], &posZ))) {
        std::printf("参数错误: X/Y/Z 必须是数字 (单位 m)\n");
        printUsage(argv[0]);
        return 1;
    }
    if (saveCsv) {
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

    // 先订阅, 再注入: 设备一启动 Locate 就会发布位姿, 先订阅才不丢首帧
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
        device->stopRecord();
        return 1;
    }
    std::printf("位姿订阅成功 (~100Hz, 每秒打印1次)\n");

    // 注入定位初值, 同时启动设备端 Locate; 失败不退出, 仍采集以便观察设备状态
    std::printf("注入定位初值 pos=(%.3f, %.3f, %.3f) quat=(w=1, x=0, y=0, z=0)...\n",
                posX, posY, posZ);
    injectInitPose(device.get(), posX, posY, posZ);

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
    if (g_count.load() == 0) {
        std::printf("提示: 位姿 0 帧。原因: Locate 未启动或重定位未成功 "
                    "(注入失败 / 初值与实际位置相差过大 / 无地图), 见 reloc_demo。\n");
    }
    return 0;
}
