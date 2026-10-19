/******************************************************************************
This file is part of XGRIDS Robot SDK.
Project: lixel_sdk
Author : XGRIDS Embedded Software Team
*******************************************************************************
History:
1. create this file. (DATE: 20260831)
******************************************************************************/

/**
 * @file   reloc_demo.cpp
 * @brief  XGRIDS Robot SDK 示例：重定位初值注入 + 数据订阅
 *
 * 演示在已有地图上做重定位的完整时序：
 *   连接设备 -> 启动录制 -> 等待进入 Recording -> 延时等待定位启动
 *   -> setLocInitPose 注入世界系初始位姿 -> 订阅位姿与点云供观察。
 *
 * 关于「重定位是否成功」的说明（务必阅读）：
 *   本版 SDK（0.3.1）**没有**重定位结果查询接口，Pose.isLocal 在 0.3.1
 *   反序列化恒为 0、不可信。因此本示例**不判定重定位成功与否**，只负责
 *   注入初值并把位姿/点云打印出来，是否收敛由使用者**观察点云与位姿轨迹**
 *   人工判断（例如位置是否收敛到初值附近并稳定）。
 *
 * setLocInitPose 的前置条件：
 *   须在设备已 Recording 且定位已启动后调用，否则设备可能丢弃初值仍返回
 *   成功。四元数模长须 > 0.1。默认注入朝向为单位四元数 (w=1,x=0,y=0,z=0)。
 *
 * 运行：
 *   ./reloc_demo <设备IP> [Y=4.5] [X=0] [Z=0] [注入前等待秒=5] [订阅时长秒=60]
 *   ./reloc_demo 192.168.123.103                 # 注入 pos=(0, 4.5, 0)
 *   ./reloc_demo 192.168.123.103 4.5 0 0 5 120   # 全参数
 */

#include "xg/robot_sdk.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

namespace {
std::atomic<bool> g_running{true};
std::atomic<uint64_t> g_poseCount{0};
std::atomic<uint64_t> g_cloudCount{0};

void signalHandler(int) { g_running.store(false); }

/**
 * @brief 秒级节流器：每满 1 秒放行一次，用于把高频回调降到每秒打印一行。
 * @note  回调在 SDK 内部线程触发，这里用原子时间戳做无锁节流；偶发竞态
 *        只影响是否多打印一行，不影响正确性，对观察用途足够。
 */
class SecondThrottle {
public:
    bool tick() {
        auto now = std::chrono::steady_clock::now();
        auto last = m_last.load(std::memory_order_relaxed);
        if (last != std::chrono::steady_clock::time_point{} &&
            now - last < std::chrono::seconds(1)) {
            return false;
        }
        m_last.store(now, std::memory_order_relaxed);
        return true;
    }

private:
    std::atomic<std::chrono::steady_clock::time_point> m_last{
        std::chrono::steady_clock::time_point{}};
};

/** 带超时的状态等待，返回是否到达目标态 */
bool waitForState(xg::Device* device, xg::RecordState target, long maxWaitMs) {
    const auto t0 = std::chrono::steady_clock::now();
    xg::RecordStatus status;
    while (g_running.load()) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now() - t0)
                           .count();
        if (elapsed >= maxWaitMs) {
            return false;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(XG_RECOMMENDED_POLL_INTERVAL_MS));
        if (device->queryRecordStatus(&status) != xg::Error::Ok) {
            continue;  // 单次查询失败可重试, 不代表操作失败
        }
        if (status.state == target) {
            return true;
        }
        if (status.state == xg::RecordState::Error) {
            std::printf("设备进入故障态: %s\n", status.message.c_str());
            return false;
        }
    }
    return false;
}
}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf(
            "用法: %s <设备IP> [Y=4.5] [X=0] [Z=0] [注入前等待秒=5] "
            "[订阅时长秒=60]\n",
            argv[0]);
        return 1;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const std::string deviceIp = argv[1];
    // 按用户需求: Y 为主要调整量, 默认 (X=0, Y=4.5, Z=0)
    float posY = (argc >= 3) ? std::atof(argv[2]) : 4.5f;
    float posX = (argc >= 4) ? std::atof(argv[3]) : 0.0f;
    float posZ = (argc >= 5) ? std::atof(argv[4]) : 0.0f;
    int preInjectWaitSec = (argc >= 6) ? std::atoi(argv[5]) : 5;
    int duration = (argc >= 7) ? std::atoi(argv[6]) : 60;

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

    // 1. 启动录制（传感器数据流与定位依赖录制状态）
    std::printf("[1/4] 启动录制...\n");
    if (device->startRecord() != xg::Error::Ok) {
        std::printf("启动录制命令下发失败\n");
        return 1;
    }
    // 启动耗时随地图规模变化, 这里给较宽裕的上限
    if (!waitForState(device.get(), xg::RecordState::Recording, 60000)) {
        std::printf("未能在超时内进入 Recording, 退出\n");
        return 1;
    }
    std::printf("      设备已进入 Recording\n\n");
    if (!g_running.load()) {
        return 0;
    }

    // 2. 等待定位启动（Recording != 定位就绪, 需给定位算法一点时间起来）
    std::printf("[2/4] 等待定位启动 %d 秒...\n\n", preInjectWaitSec);
    for (int i = 0; i < preInjectWaitSec && g_running.load(); ++i) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (!g_running.load()) {
        device->stopRecord();
        return 0;
    }

    // 3. 注入重定位初值（世界/地图系）
    std::printf("[3/4] 注入定位初值 pos=(%.3f, %.3f, %.3f) "
                "quat=(w=1, x=0, y=0, z=0)...\n",
                posX, posY, posZ);
    xg::Device::InitPose init;
    init.posX = posX;
    init.posY = posY;
    init.posZ = posZ;
    init.quatW = 1.0f;  // 单位四元数, 模长 = 1 (> 0.1 约束)
    init.quatX = 0.0f;
    init.quatY = 0.0f;
    init.quatZ = 0.0f;
    err = device->setLocInitPose(init);
    if (err != xg::Error::Ok) {
        std::printf("      注入失败: %s\n", xg::toString(err));
        std::printf("      (常见原因: 定位未启动 / 四元数非法 / 无地图)\n");
        // 注入失败不直接退出, 仍订阅数据便于观察设备当前状态
    } else {
        std::printf("      设备已接受初值 (仅表示已收下, 不代表重定位完成)\n");
    }
    std::printf("\n");

    // 4. 订阅位姿与点云, 供人工观察是否收敛到初值附近
    std::printf("[4/4] 订阅位姿(~100Hz)与点云(~1Hz), 观察重定位收敛情况\n");
    std::printf("      位姿每秒打印1次, 点云每帧打印;"
                " 关注 pose 是否收敛到 (%.2f, %.2f, %.2f) 附近\n",
                posX, posY, posZ);
    std::printf("      持续 %d 秒, Ctrl+C 提前退出\n\n", duration);

    static SecondThrottle poseThrottle;
    auto poseRc = device->subscribePose([](const xg::Pose& p) {
        uint64_t n = g_poseCount.fetch_add(1) + 1;
        if (poseThrottle.tick()) {  // 每秒打印一组
            std::printf("[pose  #%-6lu] t=%.3f pos=(%7.3f, %7.3f, %7.3f) "
                        "quat=(%.3f, %.3f, %.3f, %.3f) vel=(%6.3f, %6.3f, "
                        "%6.3f) err=%u\n",
                        (unsigned long)n, p.timestampSec, p.posX, p.posY,
                        p.posZ, p.quatW, p.quatX, p.quatY, p.quatZ, p.velX,
                        p.velY, p.velZ, p.errorCode);
        }
    });
    if (poseRc != xg::Error::Ok) {
        std::printf("位姿订阅失败: %s\n", xg::toString(poseRc));
    }

    static SecondThrottle cloudThrottle;
    auto cloudRc = device->subscribeCloud([](const xg::CloudFrame& f) {
        uint64_t n = g_cloudCount.fetch_add(1) + 1;
        if (cloudThrottle.tick()) {  // 每秒打印一组
            std::printf("[cloud #%-6lu] t=%.3f points=%zu seq=%u\n",
                        (unsigned long)n, f.timestampSec, f.count, f.seq);
        }
    });
    if (cloudRc != xg::Error::Ok) {
        std::printf("点云订阅失败: %s\n", xg::toString(cloudRc));
    }

    auto start = std::chrono::steady_clock::now();
    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                           std::chrono::steady_clock::now() - start)
                           .count();
        if (elapsed >= duration) {
            break;
        }
    }

    device->unsubscribePose();
    device->unsubscribeCloud();
    device->stopRecord();
    std::printf("\n总计接收 位姿 %lu 帧, 点云 %lu 帧\n",
                (unsigned long)g_poseCount.load(),
                (unsigned long)g_cloudCount.load());
    return 0;
}
