/******************************************************************************
This file is part of XGRIDS Robot SDK.
Project: lixel_sdk
Author : XGRIDS Embedded Software Team
*******************************************************************************
History:
1. create this file. (DATE: 20260804)
******************************************************************************/

/**
 * @file   record_demo.cpp
 * @brief  XGRIDS Robot SDK 示例：录制启停与状态监视
 *
 * 本示例全程不停轮询设备状态并逐条打印，可直观看到下发命令后设备状态的
 * 演变过程。启停接口只负责触发，确认结果是调用方的职责。
 *
 * @note **状态变为 Recording 不等于设备已完全就绪。**
 *   设备进入录制状态后，定位算法还要完成重定位才会输出定位数据，这一步
 *   实测可能耗时 1~2 分钟，也可能失败。若业务依赖定位数据（如导航），
 *   需要在 Recording 之外另行确认定位数据已就绪 —— 本 SDK 当前版本不提供
 *   该判据，仅反映录制状态本身。
 *
 * 编译（集成方工程中）：
 *   g++ -std=c++11 record_demo.cpp -I<sdk>/include \
 *       -L<sdk>/lib -llixel_sdk -Wl,-rpath,<sdk>/lib -o record_demo
 *
 * 运行：
 *   ./record_demo <设备IP> [录制时长(秒), 默认 10]
 *   ./record_demo 192.168.123.103 30
 */

#include "xg/robot_sdk.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

namespace {

using Clock = std::chrono::steady_clock;

Clock::time_point g_start;

/**
 * @brief 取程序启动至今的秒数
 */
double now() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() -
                                                                 g_start)
               .count() /
           1000.0;
}

/**
 * @brief 打印一条事件标记，便于在轮询流中定位命令下发时刻
 */
void event(const char* text) {
    std::printf("[%6.1fs] >>> %s\n", now(), text);
    std::fflush(stdout);
}

/**
 * @brief  查询一次并打印结果
 * @param  device 设备实例
 * @param  prev   上一次的状态，用于标注变化；首次传 nullptr
 * @param  out    输出本次状态
 * @return 查询成功返回 true
 * @note   查询失败照样打印，不静默跳过 —— 通信抖动本身也是需要看到的信息。
 */
bool pollOnce(xg::Device* device, const xg::RecordState* prev,
              xg::RecordStatus* out) {
    auto err = device->queryRecordStatus(out);
    if (err != xg::Error::Ok) {
        std::printf("[%6.1fs] 查询失败: %s\n", now(), xg::toString(err));
        std::fflush(stdout);
        return false;
    }

    const bool changed = (prev == nullptr) || (*prev != out->state);
    std::printf("[%6.1fs] %-10s raw=%-10d%s%s\n", now(),
                xg::toString(out->state), out->rawCode,
                out->warning ? " [WARNING]" : "",
                changed ? "   <-- 状态变化" : "");
    if (changed && out->warning) {
        std::printf("           %s\n", out->message.c_str());
    }
    std::fflush(stdout);
    return true;
}

/**
 * @brief 打印用法
 */
void printUsage(const char* prog) {
    std::printf("XGRIDS Robot SDK 示例 (SDK 版本 %s)\n\n",
                xg::sdkVersion().c_str());
    std::printf("用法:\n");
    std::printf("  %s <设备IP> [录制时长(秒), 默认 10]\n\n", prog);
    std::printf("示例:\n");
    std::printf("  %s 192.168.123.103\n", prog);
    std::printf("  %s 192.168.123.103 30\n", prog);
}

/** 下发命令前先观察一段基线状态的时长（秒） */
constexpr double kBaselineSec = 1.5;
/** 等待进入录制状态的观察上限（秒）。仅本示例的取值，SDK 不做限定。 */
constexpr double kStartWatchSec = 180.0;
/** 等待退出录制状态的观察上限（秒） */
constexpr double kStopWatchSec = 120.0;

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    const std::string deviceIp = argv[1];
    const double recordSeconds = (argc > 2) ? std::atof(argv[2]) : 10.0;

    g_start = Clock::now();

    // ---- 连接设备 ----
    xg::Options opts;
    opts.deviceIp = deviceIp;

    event(("连接 " + deviceIp).c_str());
    xg::Error err = xg::Error::Ok;
    auto device = xg::Device::open(opts, &err);
    if (!device) {
        std::printf("连接失败: %s\n", xg::toString(err));
        std::printf("请检查: 网线是否连通、是否与设备同一网段、设备端服务是否已启动\n");
        return 1;
    }
    event("已连接, 开始持续查询状态");

    xg::RecordStatus st;
    xg::RecordState prev = xg::RecordState::Error;
    bool hasPrev = false;

    // 单一轮询循环贯穿全程，命令在相应时机插入下发。
    // 这样每一次状态变化都会被打印出来，不会因为「等待某个状态」而漏掉中间过程。
    enum class Phase {
        Baseline,     // 下发前先看一段基线
        WaitingStart, // 已下发启动, 等待进入 Recording
        Recording,    // 录制中, 按设定时长持续观察
        WaitingStop,  // 已下发停止, 等待回到 Idle
        Done,
    };
    Phase phase = Phase::Baseline;
    double phaseBegin = now();
    int exitCode = 0;

    while (phase != Phase::Done) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(XG_RECOMMENDED_POLL_INTERVAL_MS));

        const bool ok = pollOnce(device.get(), hasPrev ? &prev : nullptr, &st);
        if (ok) {
            prev = st.state;
            hasPrev = true;
        }
        // 查询失败不改变阶段判断：一次通信抖动不代表设备状态有变化
        if (!ok) {
            continue;
        }

        switch (phase) {
            case Phase::Baseline:
                if (now() - phaseBegin < kBaselineSec) {
                    break;
                }
                // 设备已在录制则直接进入停止流程，保证示例从干净状态收尾
                if (st.state == xg::RecordState::Recording) {
                    event("设备已在录制, 先下发 stopRecord()");
                    if (device->stopRecord() != xg::Error::Ok) {
                        event("停止命令被拒绝");
                        return 1;
                    }
                    phase = Phase::WaitingStop;
                    phaseBegin = now();
                    break;
                }
                if (st.state == xg::RecordState::Error) {
                    std::printf("设备处于故障状态: %s\n", st.message.c_str());
                    std::printf("需先排除故障再启动录制\n");
                    return 1;
                }
                event("下发 startRecord() —— 只触发, 不等待");
                err = device->startRecord();
                if (err != xg::Error::Ok) {
                    std::printf("命令下发失败: %s\n", xg::toString(err));
                    if (err == xg::Error::Timeout) {
                        // 超时不代表命令未送达, 不要直接重试
                        std::printf("命令可能已送达, 继续观察状态即可, 勿重复下发\n");
                    }
                    return 1;
                }
                event("命令已被接受, 等待状态变为 recording");
                phase = Phase::WaitingStart;
                phaseBegin = now();
                break;

            case Phase::WaitingStart:
                if (st.state == xg::RecordState::Recording) {
                    std::printf("[%6.1fs] === 录制已开始, 从下发到确认 %.1f 秒 ===\n",
                                now(), now() - phaseBegin);
                    std::printf("           注意: 定位数据可能尚未就绪, 若业务依赖\n"
                                "           定位(如导航), 需另行确认\n");
                    phase = Phase::Recording;
                    phaseBegin = now();
                    break;
                }
                if (st.state == xg::RecordState::Error) {
                    std::printf("[%6.1fs] === 启动失败: %s ===\n", now(),
                                st.message.c_str());
                    exitCode = 1;
                    phase = Phase::Done;
                    break;
                }
                // Idle / Starting 均表示仍在准备, 继续等待
                if (now() - phaseBegin > kStartWatchSec) {
                    std::printf("[%6.1fs] === 观察 %.0f 秒仍未进入录制 ===\n",
                                now(), kStartWatchSec);
                    std::printf("           命令已下发, 设备可能仍在准备, 勿重复下发\n");
                    exitCode = 1;
                    phase = Phase::Done;
                }
                break;

            case Phase::Recording:
                if (st.state == xg::RecordState::Error) {
                    std::printf("[%6.1fs] === 录制中转入故障: %s ===\n", now(),
                                st.message.c_str());
                    exitCode = 1;
                    phase = Phase::Done;
                    break;
                }
                if (now() - phaseBegin >= recordSeconds) {
                    event("下发 stopRecord() —— 只触发, 不等待");
                    err = device->stopRecord();
                    if (err != xg::Error::Ok) {
                        std::printf("命令下发失败: %s\n", xg::toString(err));
                        return 1;
                    }
                    event("命令已被接受, 等待状态变为 idle");
                    phase = Phase::WaitingStop;
                    phaseBegin = now();
                }
                break;

            case Phase::WaitingStop:
                if (st.state == xg::RecordState::Idle) {
                    std::printf("[%6.1fs] === 录制已停止, 从下发到确认 %.1f 秒 ===\n",
                                now(), now() - phaseBegin);
                    phase = Phase::Done;
                    break;
                }
                if (now() - phaseBegin > kStopWatchSec) {
                    std::printf("[%6.1fs] === 观察 %.0f 秒仍未回到待机 ===\n", now(),
                                kStopWatchSec);
                    exitCode = 1;
                    phase = Phase::Done;
                }
                break;

            case Phase::Done:
                break;
        }
    }

    std::printf("\n总耗时 %.1f 秒\n", now());
    return exitCode;
}
