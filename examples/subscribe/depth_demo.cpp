/******************************************************************************
This file is part of XGRIDS Robot SDK.
Project: lixel_sdk
Author : XGRIDS Embedded Software Team
*******************************************************************************
History:
1. create this file. (DATE: 20260920)
2. colorize depth on a worker thread, save Jet pseudo-color PPM. (DATE: 20260929)
******************************************************************************/

/**
 * @file   depth_demo.cpp
 * @brief  XGRIDS Robot SDK 示例：深度图订阅 + 工作线程伪彩色化
 *         （AI 推理产出的度量深度，单位米，仅中间相机有效）
 *
 * 线程分工：
 *   - SDK 回调线程：只把本帧深度拷进"最新帧槽位"并唤醒工作线程，立即返回。
 *     原因：depthData 仅在回调期间有效，且回调须尽快返回、不可阻塞
 *     （见 robot_sdk_c.h 中 xg_depth_cb_t 的说明）。
 *     工作线程来不及处理时，新帧直接覆盖槽位里的旧帧并计入丢帧数，不排队。
 *   - 工作线程：统计归一化范围 → Jet 伪彩色（近红远蓝，无效像素为黑色）→ 可选落盘。
 *
 * 归一化范围：
 *   - 默认：每帧取有效像素（深度 > 0 且为有限值）的 2% / 98% 分位数，
 *     抗离群点，但帧与帧之间颜色会轻微跳动；
 *   - 传入 [近端米] [远端米] 时使用固定范围，帧间颜色稳定；
 *     超出范围的像素按端点颜色饱和显示。
 *
 * 落盘（保存目录非空且不为 "-" 时；文件序号为 SDK 收到的帧序号，丢帧处会跳号）：
 *   - depth_<seq>.bin：自定义二进制 [4B width][4B height][width*height*4B float, 米]
 *   - depth_<seq>.ppm：8-bit RGB 伪彩色图（PPM P6），eog / feh / GIMP 可直接打开
 *
 * 运行：
 *   ./depth_demo <设备IP> [持续秒数=60] [保存目录|-] [近端米 远端米]
 *   ./depth_demo 192.168.123.103 30
 *   ./depth_demo 192.168.123.103 60 /tmp/depths
 *   ./depth_demo 192.168.123.103 60 /tmp/depths 0.3 8.0
 *   ./depth_demo 192.168.123.103 60 - 0.3 8.0      # 不落盘，只打印上色统计
 *   保存目录需已存在；不存在时写文件失败，请先 mkdir。
 *
 * 前置条件：
 *   - 设备已开机、应用已起来、状态为 Idle（先用 device_info_demo 确认）。
 *   - 设备 ai.fps > 0 且 PromptDA 模型加载成功，否则不发布 depth/center。设备配置
 *     文件源码名是 modeling_app_t1xb.yaml（键 lixel_t1xb.camera_center.ai.fps），
 *     安装到设备后叫 modeling_app.yaml；出厂默认 0 不发布，本机已被改为 10。
 *     无数据时订阅仍返回 Ok，本程序在持续时长内收到 0 帧。
 *
 * 调用顺序与每步为什么：
 *   1. Device::open 连接。
 *   2. startRecord + 轮询到 Recording：AI 深度依赖录制中的传感器数据。
 *   3. 先起工作线程，再 subscribeDepth(Center)：保证第一帧到达时已有消费者。
 *   4. 采集 [持续秒数]；退出时先 unsubscribeDepth（停数据源），再让工作线程处理完
 *      最后一帧并退出，最后 stopRecord。
 *
 * 预期输出与实测数字（ai.fps=10，depth_demo 20；数字随固件版本变化，仅供参考）：
 *   - 20 s 收 118 帧，5.88 Hz（低于配置的 10：间隔 100 ms×36、200 ms×80、300 ms×1），
 *     384x288，有效像素 100.0%，上色 118 帧、丢帧 0；
 *   - 默认分位范围：2% 分位 0.75~0.76 m，98% 分位约 10.40 m；
 *   - 深度图时间戳沿用图像时间戳（设备系统时钟），偏差换算见 image_demo。
 *
 * 失败现象与处理：
 *   - 0 帧：ai.fps 为 0、模型没加载，或没进入 Recording；看设备日志里
 *     PromptdaDepthTask::load 是否成功。
 *   - 起跑头几秒设备日志偶有 1 对 "no matched cloud / zero sparse depth"，
 *     是点云与图像未配对的首帧，之后不再出现。
 *   - 工作线程来不及处理时，新帧覆盖旧帧并计入 dropped。
 *   - 启动耗时：从雷达待机起约 17.5~18.6 s，新进程约 7.5~8.5 s。
 *
 * 副作用：
 *   - 会 startRecord，每次运行在设备上生成一个录制工程（本机实测 87~103 MB/次）。
 *   - 退出时 stopRecord；stopRecord 之后设备应用会退出并由 systemd 重启，约
 *     10~20 s 内设备不可用。下一次运行请先用 device_info_demo 确认连得上且状态
 *     为 Idle；重启期间下发的 startRecord 会被排队，收尾的 stopRecord 在非录制态
 *     是空操作。
 *
 * 线程模型：回调在 Zenoh RX 线程触发，不得阻塞。本示例回调只拷贝深度并唤醒工作线程，
 * 上色和落盘都在工作线程。
 */

#include "xg/robot_sdk.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {
std::atomic<bool> g_running{true};
std::atomic<uint32_t> g_recvCount{0}; /**< SDK 回调收到的帧数 */
std::atomic<uint32_t> g_doneCount{0}; /**< 工作线程上色完成的帧数 */
std::atomic<uint32_t> g_dropCount{0}; /**< 未被工作线程取走、被新帧覆盖的帧数 */
std::string g_saveDir;

/** 分位数归一化的下分位 / 上分位 */
constexpr double kLowPercentile = 0.02;
constexpr double kHighPercentile = 0.98;
/** 归一化范围的最小跨度（米），防止整帧深度相同时除零 */
constexpr float kMinSpanM = 1e-3f;

/** 深度归一化范围（米） */
struct DepthRange {
    float nearM{0.0f};
    float farM{0.0f};
};

/** 命令行指定的固定范围；main 在启动工作线程前写入，之后只读 */
bool g_useFixedRange = false;
DepthRange g_fixedRange;

/** 一帧深度的自持有拷贝（在回调期间从 SDK 指针拷出） */
struct DepthBuffer {
    uint32_t seq{0};
    double timestampSec{0.0};
    uint32_t width{0};
    uint32_t height{0};
    std::vector<float> data; /**< 行优先，单位米 */
};

// 回调线程 → 工作线程的单槽交接区：只保留最新一帧
std::mutex g_slotMutex;
std::condition_variable g_slotCv;
DepthBuffer g_slot;
bool g_slotFull = false;
bool g_workerStop = false;

void signalHandler(int) { g_running.store(false); }

/** 有效深度：有限值且大于 0（SDK 约定未命中区域为 0） */
bool isValidDepth(float d) { return std::isfinite(d) && d > 0.0f; }

/** Jet 色表：下标 0 为深蓝，255 为深红（与 OpenCV COLORMAP_JET 走向一致） */
using JetLut = std::array<std::array<uint8_t, 3>, 256>;

const JetLut& jetLut() {
    static const JetLut lut = [] {
        JetLut table{};
        for (int i = 0; i < 256; ++i) {
            const float v = static_cast<float>(i) / 255.0f;
            // 分段线性 Jet：每个通道是以 center 为峰、宽度为 1.5 的梯形
            auto channel = [v](float center) {
                float c = 1.5f - std::fabs(4.0f * v - center);
                c = std::min(1.0f, std::max(0.0f, c));
                return static_cast<uint8_t>(std::lround(c * 255.0f));
            };
            table[i] = {channel(3.0f), channel(2.0f), channel(1.0f)};  // R, G, B
        }
        return table;
    }();
    return lut;
}

/**
 * @brief  按分位数求本帧归一化范围
 * @param  depth   本帧深度（米）
 * @param  scratch 复用的临时缓冲，避免每帧重新分配
 * @param  out     输出范围；有效像素数为 0 时不修改
 * @return 有效像素数
 */
size_t percentileRange(const std::vector<float>& depth,
                       std::vector<float>* scratch, DepthRange* out) {
    scratch->clear();
    for (float d : depth) {
        if (isValidDepth(d)) {
            scratch->push_back(d);
        }
    }
    const size_t n = scratch->size();
    if (n == 0) {
        return 0;
    }
    const size_t lo =
        static_cast<size_t>(kLowPercentile * static_cast<double>(n - 1));
    const size_t hi =
        static_cast<size_t>(kHighPercentile * static_cast<double>(n - 1));
    std::nth_element(scratch->begin(), scratch->begin() + lo, scratch->end());
    out->nearM = (*scratch)[lo];
    out->farM = out->nearM;
    if (hi > lo) {
        // 第一次 nth_element 之后 [lo+1, end) 的元素都不小于 lo 处的值，
        // 只需在该子区间内再定位 hi。
        std::nth_element(scratch->begin() + lo + 1, scratch->begin() + hi,
                         scratch->end());
        out->farM = (*scratch)[hi];
    }
    return n;
}

/**
 * @brief  深度转 Jet 伪彩色 RGB：近端红、远端蓝，无效像素黑色
 * @param  depth 本帧深度（米）
 * @param  range 归一化范围，要求 farM > nearM
 * @param  rgb   输出，大小为像素数 * 3，复用调用方缓冲
 */
void colorizeJet(const std::vector<float>& depth, const DepthRange& range,
                 std::vector<uint8_t>* rgb) {
    const JetLut& lut = jetLut();
    rgb->resize(depth.size() * 3);
    const float invSpan = 1.0f / (range.farM - range.nearM);
    uint8_t* px = rgb->data();
    for (float d : depth) {
        if (!isValidDepth(d)) {
            px[0] = 0;
            px[1] = 0;
            px[2] = 0;
        } else {
            // t: 0 = 近端，1 = 远端；超出范围饱和到端点
            float t = (d - range.nearM) * invSpan;
            t = std::min(1.0f, std::max(0.0f, t));
            // 近红远蓝：近端取色表高端（红），远端取低端（蓝）
            const auto& c =
                lut[static_cast<size_t>(std::lround((1.0f - t) * 255.0f))];
            px[0] = c[0];
            px[1] = c[1];
            px[2] = c[2];
        }
        px += 3;
    }
}

// 落盘为自定义二进制格式: [4B width][4B height][width*height*4B float, 米]。
void saveBin(const DepthBuffer& buf) {
    char path[512];
    std::snprintf(path, sizeof(path), "%s/depth_%06u.bin", g_saveDir.c_str(),
                  buf.seq);
    FILE* fp = std::fopen(path, "wb");
    if (!fp) {
        std::printf("Warning: failed to open %s!\n", path);
        return;
    }
    std::fwrite(&buf.width, sizeof(buf.width), 1, fp);
    std::fwrite(&buf.height, sizeof(buf.height), 1, fp);
    std::fwrite(buf.data.data(), sizeof(float), buf.data.size(), fp);
    std::fclose(fp);
}

// 落盘为 8-bit RGB PPM (P6)，零依赖（不需要 OpenCV），看图工具可直接打开。
void savePpm(const DepthBuffer& buf, const std::vector<uint8_t>& rgb) {
    char path[512];
    std::snprintf(path, sizeof(path), "%s/depth_%06u.ppm", g_saveDir.c_str(),
                  buf.seq);
    FILE* fp = std::fopen(path, "wb");
    if (!fp) {
        std::printf("Warning: failed to open %s!\n", path);
        return;
    }
    // PPM P6 头: magic, width height, maxval；像素按行优先 RGB 排列
    std::fprintf(fp, "P6\n%u %u\n255\n", buf.width, buf.height);
    std::fwrite(rgb.data(), 1, rgb.size(), fp);
    std::fclose(fp);
}

/**
 * @brief  SDK 深度回调（SDK 内部线程）：只拷贝 + 唤醒，立即返回
 * @note   frame.depthData 仅在本回调期间有效，必须在这里拷出。
 */
void onDepthFrame(const xg::Device::DepthFrame& frame) {
    const uint32_t seq = g_recvCount.fetch_add(1) + 1;
    const size_t total = static_cast<size_t>(frame.width) * frame.height;
    if (frame.depthData == nullptr || total == 0 ||
        frame.depthSize < total * sizeof(float)) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(g_slotMutex);
        if (g_slotFull) {
            g_dropCount.fetch_add(1);  // 上一帧还没被取走，直接覆盖
        }
        g_slot.seq = seq;
        g_slot.timestampSec = frame.timestampSec;
        g_slot.width = frame.width;
        g_slot.height = frame.height;
        // 稳态下槽位缓冲与工作线程缓冲尺寸相同，assign 复用已有容量，不重新分配
        g_slot.data.assign(frame.depthData, frame.depthData + total);
        g_slotFull = true;
    }
    g_slotCv.notify_one();
}

/** 工作线程：取最新帧 → 求范围 → Jet 上色 → 可选落盘 */
void colorizeWorker() {
    DepthBuffer buf;
    std::vector<float> scratch;
    std::vector<uint8_t> rgb;
    for (;;) {
        {
            std::unique_lock<std::mutex> lock(g_slotMutex);
            g_slotCv.wait(lock, [] { return g_slotFull || g_workerStop; });
            if (!g_slotFull) {
                break;  // 已请求停止且槽位已空
            }
            // 交换而非拷贝：锁内只做 O(1) 操作，回调下次 assign 复用旧缓冲容量
            std::swap(buf, g_slot);
            g_slotFull = false;
        }

        const auto t0 = std::chrono::steady_clock::now();
        DepthRange range;
        size_t valid = 0;
        if (g_useFixedRange) {
            range = g_fixedRange;
            valid = static_cast<size_t>(
                std::count_if(buf.data.begin(), buf.data.end(), isValidDepth));
        } else {
            valid = percentileRange(buf.data, &scratch, &range);
        }
        if (range.farM - range.nearM < kMinSpanM) {
            range.farM = range.nearM + kMinSpanM;
        }
        colorizeJet(buf.data, range, &rgb);
        const double costMs = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - t0)
                                  .count();

        const double validPct = 100.0 * static_cast<double>(valid) /
                                static_cast<double>(buf.data.size());
        if (valid == 0) {
            std::printf("[Depth %4u] t=%.3f  %ux%u  no valid depth, "
                        "colorize=%.1f ms.\n",
                        buf.seq, buf.timestampSec, buf.width, buf.height,
                        costMs);
        } else {
            std::printf("[Depth %4u] t=%.3f  %ux%u  valid=%.1f%%  "
                        "range=[%.2f, %.2f] m  colorize=%.1f ms.\n",
                        buf.seq, buf.timestampSec, buf.width, buf.height,
                        validPct, range.nearM, range.farM, costMs);
        }
        if (!g_saveDir.empty()) {
            saveBin(buf);
            savePpm(buf, rgb);
        }
        g_doneCount.fetch_add(1);
    }
}

/** 请求工作线程退出：处理完槽位里剩下的一帧后返回 */
void stopWorker(std::thread* worker) {
    {
        std::lock_guard<std::mutex> lock(g_slotMutex);
        g_workerStop = true;
    }
    g_slotCv.notify_one();
    if (worker->joinable()) {
        worker->join();
    }
}

/** 解析米制浮点参数，整串必须是合法有限数 */
bool parseMeters(const char* s, float* out) {
    char* end = nullptr;
    const double v = std::strtod(s, &end);
    if (end == s || *end != '\0' || !std::isfinite(v)) {
        return false;
    }
    *out = static_cast<float>(v);
    return true;
}

void printUsage(const char* prog) {
    std::printf("Usage: %s <device_ip> [duration_sec] [save_dir|-] "
                "[near_m far_m]\n",
                prog);
    std::printf("  save_dir      save depth_<seq>.bin (raw float, meters) and "
                "depth_<seq>.ppm (Jet color); \"-\" = do not save.\n");
    std::printf("  near_m far_m  fixed colorize range in meters; default is "
                "per-frame 2%%/98%% percentile.\n");
    std::printf("  Color: near = red, far = blue, invalid (0) = black.\n");
    std::printf("\nNote: the device does not publish depth/center by default "
                "(lixel_t1xb.camera_center.ai.fps is 0).\n"
                "      It must be set to a value > 0 on the device and the AI "
                "model must be ready;\n"
                "      without data the subscription still returns Ok.\n");
}
}  // namespace

int main(int argc, char** argv) {
    // 参数个数只允许 2..4 或 6（近端/远端必须成对给出）
    if (argc < 2 || argc == 5 || argc > 6) {
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
    if (argc == 6) {
        if (!parseMeters(argv[4], &g_fixedRange.nearM) ||
            !parseMeters(argv[5], &g_fixedRange.farM) ||
            g_fixedRange.nearM < 0.0f ||
            g_fixedRange.farM <= g_fixedRange.nearM) {
            std::printf("Invalid range: require 0 <= near_m < far_m!\n");
            return 1;
        }
        g_useFixedRange = true;
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

    // 先起工作线程再订阅，保证第一帧到达时已有消费者
    std::thread worker(colorizeWorker);

    // 订阅中间相机深度图（仅中间相机有效）
    auto rc = device->subscribeDepth(xg::Device::CameraId::Center,
                                     onDepthFrame);
    if (rc != xg::Error::Ok) {
        std::printf("深度图订阅失败: %s\n", xg::toString(rc));
        stopWorker(&worker);
        device->stopRecord();
        return 1;
    }
    std::printf("深度图订阅成功 (中间相机)\n");
    std::printf("持续 %d 秒, Ctrl+C 提前退出\n", duration);
    if (g_useFixedRange) {
        std::printf("Colorize range: fixed [%.2f, %.2f] m.\n",
                    g_fixedRange.nearM, g_fixedRange.farM);
    } else {
        std::printf("Colorize range: per-frame %.0f%%/%.0f%% percentile.\n",
                    kLowPercentile * 100.0, kHighPercentile * 100.0);
    }
    if (!g_saveDir.empty()) {
        std::printf("Saving to: %s (.bin raw float + .ppm Jet color).\n",
                    g_saveDir.c_str());
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

    // 先停数据源，再让工作线程处理完槽位里最后一帧后退出
    device->unsubscribeDepth(xg::Device::CameraId::Center);
    stopWorker(&worker);
    device->stopRecord();

    std::printf("\nTotal: received %u, colorized %u, dropped %u "
                "(worker busy).\n",
                g_recvCount.load(), g_doneCount.load(), g_dropCount.load());
    return 0;
}
