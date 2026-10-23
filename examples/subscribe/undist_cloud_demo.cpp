/******************************************************************************
This file is part of XGRIDS Robot SDK.
Project: lixel_sdk
Author : XGRIDS Embedded Software Team
*******************************************************************************
History:
1. create this file. (DATE: 20260826)
2. rename raw_cloud_demo -> undist_cloud_demo, use device_undist_cloud. (DATE: 20260929)
******************************************************************************/

/**
 * @file   undist_cloud_demo.cpp
 * @brief  XGRIDS Robot SDK 示例：设备(IMU)系 Locate 去畸变点云订阅（可单独运行出数据）
 *
 * 订阅 subscribeDeviceUndistCloud（Zenoh key: device_undist_cloud）：设备端 Locate
 * 输出的去畸变点云（imu_cloud），点坐标为设备(IMU)系。每帧打印点数和前几个点坐标，
 * 可选保存为 ASCII PCD。Locate 只在 setLocInitPose 之后才启动，所以本示例自带
 * 初值注入，不需要先跑别的示例。
 *
 * 前置条件：
 *   - 设备已开机、应用已起来、状态为 Idle（先用 device_info_demo 确认连得上）。
 *   - 设备上有可用的重定位地图，[X] [Y] [Z] 与设备当前实际位置大致相符，否则
 *     重定位可能不收敛，点云不会出。
 *
 * 调用顺序与每步为什么：
 *   1. Device::open 连接。
 *   2. startRecord + 轮询到 Recording：Locate 在录制启动时完成初始化，
 *      进入 Recording 前已完成。
 *   3. subscribeDeviceUndistCloud：先订阅，再注入，避免丢首帧。
 *   4. setLocInitPose(X,Y,Z, 单位四元数)：设备做 setInitPose + startLocateAlgo，
 *      一条命令同时给初值并启动 Locate；之后才发布 device_undist_cloud。
 *   5. 采集 [持续秒数]，退出时 unsubscribeDeviceUndistCloud + stopRecord。
 *
 * 运行：
 *   ./undist_cloud_demo <设备IP> [持续秒数=60] [保存目录|-] [X=0] [Y=0] [Z=0]
 *   ./undist_cloud_demo 192.168.123.103 30
 *   ./undist_cloud_demo 192.168.123.103 30 /tmp/clouds 1 1 0.45
 *   ./undist_cloud_demo 192.168.123.103 30 - 1 1 0.45   # 不保存，只改初值
 *   X/Y/Z 单位 m，世界/地图系，默认 0 0 0（地图原点），与 reloc_demo 一致；
 *   朝向固定为单位四元数 (w=1,x=0,y=0,z=0)。保存目录不存在时自动创建一级目录。
 *   -h / --help 打印用法。
 *
 * 预期输出与实测数字（注入 (1,1,0.45)；数字随固件版本变化，仅供参考）：
 *   - 重定位成功后约 9.7 Hz（实测 9.71 Hz，间隔 100 ms）；重定位成功用时
 *     5.6~6.9 s（实测 6.85 s），首帧点云的到达时刻本轮未单独测量；
 *   - 每帧 19.0k 点（均值 19057，范围 17.1k~19.3k），约为雷达全量
 *     点云（raw_cloud_demo，均值 80.6k）的 23.6%；该比例与发布说明里
 *     "Locate 输入为 1/4 降采样"量级相符但不精确，差异原因未验证。
 *   - 该帧点坐标为设备(IMU)系，不是世界系。
 *
 * 持续秒数语义：[持续秒数] 从注入返回后开始计时，包含重定位等待（实测 5.6~6.9 s）。
 * 例如 15 秒窗口实测得到 136 帧（帧时间戳跨度 13.9 s）；想要 N 秒点云数据，
 * 请把持续秒数设为 N + 7。
 *
 * 失败现象与处理：
 *   - "注入失败: Rejected"：设备拒绝，常见为定位尚未初始化、无地图或四元数非法；
 *     确认已 Recording、设备上有地图后重跑。
 *   - "注入失败: Timeout"：命令可能已送达，不要立刻重复注入，先看是否有点云。
 *   - 结束时"点云 0 帧"：Locate 未启动或重定位未成功（见 reloc_demo）。
 *   - 保存 PCD 在回调里同步写文件，单帧约 19k 点，会拖慢 Zenoh RX 线程，
 *     仅用于调试；正式集成请把帧拷贝入队，由工作线程落盘。
 *   - 启动耗时：从雷达待机起约 17.5~18.6 s，新进程约 7.5~8.5 s。
 *
 * 副作用：
 *   - 会 startRecord，每次运行在设备上生成一个录制工程（本机实测 87~103 MB/次）。
 *   - 退出时 stopRecord；stopRecord 之后设备应用会退出并由 systemd 重启，约
 *     10~20 s 内设备不可用。下一次运行请先用 device_info_demo 确认连得上且状态
 *     为 Idle；重启期间下发的 startRecord 会被排队，收尾的 stopRecord 在非录制态
 *     是空操作。
 *
 * 线程模型：回调在 Zenoh RX 线程触发，回调内不得阻塞（不做耗时 IO、不等长锁），
 * 重活请入队交给工作线程。
 */

#include "xg/robot_sdk.h"

#include <atomic>
#include <chrono>
#include <cmath>
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
                    "      建议: 不要立刻重复注入, 先观察是否有点云输出。\n");
    } else {
        std::printf("      建议: 检查与设备的连接后重跑。\n");
    }
    return false;
}

void printUsage(const char* prog) {
    std::printf("用法: %s <设备IP> [持续秒数=60] [保存目录|-] [X=0] [Y=0] [Z=0]\n",
                prog);
    std::printf("  保存目录非空且不为 - 时，每帧保存为 ASCII PCD 文件\n");
    std::printf("  X/Y/Z: 定位初值位置, 单位 m, 世界/地图系, 默认 0 0 0\n");
}

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
    if (argc >= 4 && std::strcmp(argv[3], "-") != 0) {
        g_saveDir = argv[3];
    }
    float posX = 0.0f, posY = 0.0f, posZ = 0.0f;
    if ((argc >= 5 && !parseFloatArg(argv[4], &posX)) ||
        (argc >= 6 && !parseFloatArg(argv[5], &posY)) ||
        (argc >= 7 && !parseFloatArg(argv[6], &posZ))) {
        std::printf("参数错误: X/Y/Z 必须是数字 (单位 m)\n");
        printUsage(argv[0]);
        return 1;
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

    
    // 启动录制（传感器数据流和 Locate 初始化都依赖录制状态；Locate 去畸变点云
    // 还需 setLocInitPose 启动 Locate, 见下方订阅之后的注入步骤）
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

    // 先订阅, 再注入: 设备一启动 Locate 就会发布点云, 先订阅才不丢首帧
    auto rc = device->subscribeDeviceUndistCloud([](const xg::CloudFrame& frame) {
        uint32_t n = g_count.fetch_add(1) + 1;
        std::printf("[UndistCloud %3u] t=%.3f  points=%zu\n",
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
        std::printf("去畸变点云订阅失败: %s\n", xg::toString(rc));
        device->stopRecord();
        return 1;
    }
    std::printf("去畸变点云订阅成功 (~10Hz)\n");

    // 注入定位初值, 同时启动设备端 Locate; 失败不退出, 仍采集以便观察设备状态
    std::printf("注入定位初值 pos=(%.3f, %.3f, %.3f) quat=(w=1, x=0, y=0, z=0)...\n",
                posX, posY, posZ);
    injectInitPose(device.get(), posX, posY, posZ);

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

    device->unsubscribeDeviceUndistCloud();
    device->stopRecord();
    std::printf("\n总计接收 %u 帧去畸变点云\n", g_count.load());
    if (g_count.load() == 0) {
        std::printf("提示: 去畸变点云 0 帧。原因: Locate 未启动或重定位未成功 "
                    "(注入失败 / 初值与实际位置相差过大 / 无地图), 见 reloc_demo。\n");
    }
    return 0;
}
