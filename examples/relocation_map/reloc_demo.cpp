/******************************************************************************
This file is part of XGRIDS Robot SDK.
Project: lixel_sdk
Author : XGRIDS Embedded Software Team
******************************************************************************/

/**
 * @file   reloc_demo.cpp
 * @brief  XGRIDS Robot SDK 示例：重定位初值注入 + 数据订阅
 *
 * 演示在已有地图上做重定位的完整时序：
 *   连接设备 -> 启动录制 -> 等待进入 Recording -> 注入前等待（只留余量）
 *   -> setLocInitPose 注入世界系初始位姿 -> 订阅位姿与点云供观察。
 *
 * 前置条件：
 *   - 设备已开机、应用已起来、状态为 Idle（先用 device_info_demo 确认连得上）。
 *   - 设备上有可用的重定位地图（map_upload_demo 可上传并激活）；[X] [Y] [Z] 与设备
 *     当前实际位置大致相符，相差过大重定位可能不收敛。
 *
 * 调用顺序与每步为什么：
 *   1. startRecord + 轮询到 Recording（上限 60 s）：Locate 在录制启动时完成初始化，
 *      进入 Recording 时已完成；没有 Recording 就没有可注入的对象。
 *   2. 注入前等待 [注入前等待秒]：Recording 时初始化已完成，这里只是留余量，
 *      Locate 算法并不是在这一步启动的。
 *   3. setLocInitPose：设备做 setInitPose + startLocateAlgo，一条命令同时给初值
 *      并启动 Locate；之后才发布位姿与去畸变点云。未初始化时设备拒绝（Rejected），
 *      四元数模长须 > 0.1，本示例固定注入单位四元数 (w=1,x=0,y=0,z=0)。
 *   4. 订阅位姿与点云并打印。订阅在注入返回之后才建立，注入前的帧不会统计
 *      （纯录制下注入前本来就没有位姿和 Locate 点云）。想不丢首帧的写法见
 *      pose_demo / undist_cloud_demo（先订阅再注入）。
 *
 * 关于「重定位是否成功」的说明（务必阅读）：
 *   当前 SDK **没有**重定位结果查询接口；Pose.isLocal 为保留字段、恒为 0、
 *   不可信（见 xg_pose_t.is_local）。因此本示例**不判定重定位成功与否**，只负责
 *   注入初值并把位姿/点云打印出来，是否收敛由使用者**观察点云与位姿轨迹**
 *   人工判断（例如位置是否收敛到初值附近并稳定）。
 *
 * 运行：
 *   ./reloc_demo <设备IP> [X=0] [Y=0] [Z=0] [注入前等待秒=5] [订阅时长秒=60]
 *   ./reloc_demo 192.168.123.103                 # 注入 pos=(0, 0, 0)，即地图原点
 *   ./reloc_demo 192.168.123.103 1.5 -2 0 5 120  # 全参数: pos=(1.5, -2, 0)
 *   X/Y/Z 单位 m，世界/地图系，默认 0 0 0。
 *
 * 预期输出与实测数字（参数 1 1 0.45 10 90；数字随固件版本变化，仅供参考）：
 *   - 注入后 5.6~6.9 s 重定位成功（设备日志 relocation success!；本示例实测
 *     6.02 s，同期其他示例实测 5.64/6.26/6.85 s）；
 *   - 位姿约 100 Hz（实测 8397 帧，99.89 Hz），err=0；点云约 9.7 Hz（实测 867 帧，
 *     9.74 Hz），每帧 19.0k 点（范围 17.2k~19.3k）；位姿和点云每秒各打印一行；
 *   - 位姿稳定在约 (1.05, 4.01, 0.43) m，与注入值 Y 向相差约 3.0 m：那是设备在
 *     地图上匹配到的位置，不是接口缺数据。
 *
 * 失败现象与处理：
 *   - "注入失败: Rejected"：定位未初始化 / 无地图 / 四元数非法。本示例不退出，
 *     继续订阅便于观察，但位姿和点云会是 0 帧；确认已 Recording、设备上有地图后重跑。
 *   - "注入失败: Timeout"：命令可能已送达，不要立刻重复注入。
 *   - 启动耗时：从雷达待机起约 17.5~18.6 s，新进程约 7.5~8.5 s，60 s 上限足够。
 *   - 位姿 / 点云长期 0 帧：Locate 未启动或重定位未成功。
 *
 * 副作用：
 *   - 会 startRecord，每次运行在设备上生成一个录制工程（本机实测 87~103 MB/次）。
 *   - 退出时 stopRecord；stopRecord 之后设备应用会退出并由 systemd 重启，约
 *     10~20 s 内设备不可用。下一次运行请先用 device_info_demo 确认连得上且状态
 *     为 Idle；重启期间下发的 startRecord 会被排队，收尾的 stopRecord 在非录制态
 *     是空操作。
 *
 * 线程模型：回调在 Zenoh RX 线程触发，回调内不得阻塞；本示例回调只做计数和
 * 秒级节流打印。
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
            "用法: %s <设备IP> [X=0] [Y=0] [Z=0] [注入前等待秒=5] "
            "[订阅时长秒=60]\n",
            argv[0]);
        return 1;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const std::string deviceIp = argv[1];
    // 位置参数按 X、Y、Z 的自然顺序, 与 InitPose / xg_init_pose_t 的字段顺序一致。
    // 默认 (X=0, Y=0, Z=0)，即地图原点，单位 m, 世界/地图系。
    float posX = (argc >= 3) ? std::atof(argv[2]) : 0.0f;
    float posY = (argc >= 4) ? std::atof(argv[3]) : 0.0f;
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
    err = device->startRecord();
    if (err != xg::Error::Ok) {
        std::printf("启动录制命令下发失败: %s\n", xg::toString(err));
        if (err == xg::Error::Timeout) {
            // 超时不代表命令没送达：设备可能已经开始录制。补发一次停止，
            // 避免设备被留在录制状态（停止在非录制态是幂等的）
            device->stopRecord();
        }
        return 1;
    }
    // 启动耗时随地图规模变化, 这里给较宽裕的上限
    if (!waitForState(device.get(), xg::RecordState::Recording, 60000)) {
        // startRecord 已下发, 设备可能仍在启动中, 退出前必须停掉, 否则会一直录
        std::printf("未能在超时内进入 Recording, 退出\n");
        device->stopRecord();
        return 1;
    }
    std::printf("      设备已进入 Recording\n\n");
    if (!g_running.load()) {
        device->stopRecord();
        return 0;
    }

    // 2. 注入前等待（Recording 时 Locate 初始化已完成, 这里只留余量;
    //    Locate 算法由下一步 setLocInitPose 启动）
    std::printf("[2/4] 注入前等待 %d 秒...\n\n", preInjectWaitSec);
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
        std::printf("      (常见原因: 定位未初始化 / 四元数非法 / 无地图)\n");
        // 注入失败不直接退出, 仍订阅数据便于观察设备当前状态
    } else {
        std::printf("      设备已接受初值 (仅表示已收下, 不代表重定位完成)\n");
    }
    std::printf("\n");

    // 4. 订阅位姿与点云, 供人工观察是否收敛到初值附近
    std::printf("[4/4] 订阅位姿(~100Hz)与设备系 Locate 去畸变点云, 观察重定位收敛情况\n");
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
    auto cloudRc = device->subscribeDeviceUndistCloud([](const xg::CloudFrame& f) {
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
    device->unsubscribeDeviceUndistCloud();
    device->stopRecord();
    std::printf("\n总计接收 位姿 %lu 帧, 点云 %lu 帧\n",
                (unsigned long)g_poseCount.load(),
                (unsigned long)g_cloudCount.load());
    return 0;
}
