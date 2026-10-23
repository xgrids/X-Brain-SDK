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
 * 同时演示时间戳换算：图像时间戳取自设备系统时钟，录制开始后先用 getDeviceTime
 * 取设备时钟与本机时钟的偏差区间，再把每帧的设备时间换到本机时钟，估算本程序
 * 收到该帧时相对设备时间戳已经过去多久。换算关系与适用范围见
 * docs/api_reference_v2.md §0.8：这个换算只对取自设备系统时钟的数据（图像及沿用其
 * 时间戳的深度图）成立，IMU / 位姿 / 点云不适用。区间只来自往返时延，没有计入设备端
 * 叠加在图像时间戳上的固定补偿（当前配置 0 到 9.21 ms）和取帧口径（未查到），
 * 所以它是「相对设备时间戳的延迟」，不是严格的拍摄到收到延迟。
 * 设备固件不支持时间查询时，退化为只打印设备时间戳。
 *
 * 前置条件：
 *   - 设备已开机、应用已起来、状态为 Idle（先用 device_info_demo 确认）。
 *   - 设备端启用 JPEG 输出（jpeg.fps > 0）。图像帧率以设备配置为准：本机实测
 *     10.01 Hz；旧注释里的默认约 2 fps 已过时，不要拿它做判断。
 *
 * 调用顺序与每步为什么：
 *   1. Device::open 连接。
 *   2. startRecord + 轮询到 Recording：相机流依赖录制状态。
 *   3. getDeviceTime：必须在 Recording 之后查。配置了 PPS 的设备开始扫描时会校准
 *      系统时钟，之前查到的偏差会失效。
 *   4. subscribeImage × 3（Left/Center/Right），每帧打印，可选存 JPEG。
 *   5. 采集 [持续秒数]，退出时 unsubscribeImage × 3 + stopRecord。
 *
 * 运行：
 *   ./image_demo <设备IP> [持续秒数=60] [保存目录]
 *   ./image_demo 192.168.123.103 30
 *   ./image_demo 192.168.123.103 60 /tmp/images
 *   保存目录不存在时自动创建一级目录。
 *
 * 预期输出与实测数字（image_demo 20 ./img；数字随固件版本变化，仅供参考）：
 *   - 三路各约 10.01 Hz（left 199 / center 200 / right 201 帧），间隔 99~100 ms，
 *     时间戳严格递增；
 *   - JPEG 均值：left 30123 B、center 899781 B、right 25149 B（约 30 KB / 900 KB / 25 KB）；
 *     尺寸 center 4000x3000，left/right 640x480；
 *   - "相对设备时间戳的延迟"区间下界 54~131 ms，上界 56~133 ms。
 *
 * 失败现象与处理：
 *   - 三路都是 0 帧：设备 jpeg.fps 为 0，或没进入 Recording。
 *   - "无法查询设备时间"：设备固件不支持，示例退化为只打印设备时间戳。
 *   - 每帧打印并写文件会拖慢接收线程，仅用于调试。
 *   - startRecord 超时：本示例会补发 stopRecord。
 *
 * 副作用：
 *   - 会 startRecord，每次运行在设备上生成一个录制工程（本机实测 87~103 MB/次）。
 *   - 退出时 stopRecord；stopRecord 之后设备应用会退出并由 systemd 重启，约
 *     10~20 s 内设备不可用。下一次运行请先用 device_info_demo 确认连得上且状态
 *     为 Idle；重启期间下发的 startRecord 会被排队，收尾的 stopRecord 在非录制态
 *     是空操作。
 *
 * 线程模型：回调在 Zenoh RX 线程触发，回调内不得阻塞。本示例为演示在回调里写
 * JPEG，正式集成请把数据拷贝入队，由工作线程落盘。
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

    // 启动录制（相机流依赖录制状态）
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
            std::printf("录制启动失败: %s\n", recStatus.message.c_str());
            device->stopRecord();
            return 1;
        }
    }
    // startRecord 已下发, 提前退出前必须停掉, 否则设备会一直录
    if (!g_running.load()) {
        device->stopRecord();
        return 0;
    }

    // 取设备时钟与本机时钟的偏差区间。必须在录制开始之后查询：配置了 PPS 的设备
    // 在开始扫描时会校准系统时钟，之前查到的偏差会失效。
    xg::Device::DeviceTime devTime;
    const bool haveOffset =
        (device->getDeviceTime(&devTime) == xg::Error::Ok);
    const long long offsetLoUs = haveOffset ? devTime.offsetLowerUs() : 0;
    const long long offsetHiUs = haveOffset ? devTime.offsetUpperUs() : 0;
    if (haveOffset) {
        std::printf("设备时钟 - 本机时钟 ∈ [%+.3f, %+.3f] s (区间宽度 = 往返时延 %.1f ms)\n\n",
                    offsetLoUs * 1e-6, offsetHiUs * 1e-6, devTime.rttUs * 1e-3);
    } else {
        std::printf("无法查询设备时间（设备固件不支持，或应答无法解析），只打印设备时间戳\n\n");
    }

    // 订阅三路相机
    using CamId = xg::Device::CameraId;
    CamId cameras[] = {CamId::Left, CamId::Center, CamId::Right};

    for (int i = 0; i < 3; ++i) {
        auto rc = device->subscribeImage(cameras[i],
            [i, haveOffset, offsetLoUs, offsetHiUs](
                const xg::Device::ImageFrame& frame) {
                uint32_t n = g_count[i].fetch_add(1) + 1;
                if (haveOffset) {
                    // t_host = timestamp - offset，offset 落在 [lo, hi]；
                    // 帧延迟 = 本机现在 - t_host，随之落在 [now-ts+lo, now-ts+hi]
                    const long long tsUs =
                        std::llround(frame.timestampSec * 1e6);
                    const long long nowUs =
                        std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
                    std::printf("[%s %3u] t=%.3f  jpeg=%zu bytes  相对设备时间戳的延迟 ∈ [%.0f, %.0f] ms\n",
                                g_cameraNames[i], n, frame.timestampSec,
                                frame.jpegSize,
                                (nowUs - tsUs + offsetLoUs) * 1e-3,
                                (nowUs - tsUs + offsetHiUs) * 1e-3);
                } else {
                    std::printf("[%s %3u] t=%.3f  jpeg=%zu bytes\n",
                                g_cameraNames[i], n,
                                frame.timestampSec, frame.jpegSize);
                }
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
