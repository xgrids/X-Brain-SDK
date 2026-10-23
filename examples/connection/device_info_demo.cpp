/******************************************************************************
This file is part of XGRIDS Robot SDK.
Project: lixel_sdk
Author : XGRIDS Embedded Software Team
******************************************************************************/

/**
 * @file   device_info_demo.cpp
 * @brief  XGRIDS Robot SDK 示例：连接设备并只读查询状态、录制状态、时间与标定
 *
 * 全程只读：不 startRecord、不订阅、不改设备任何配置。也是其他示例的"设备就绪"
 * 探针：能连上并打印出"录制状态: idle"，才可以起会 startRecord 的示例。
 *
 * 前置条件：设备已开机，主机能到达设备 IP（USB 网卡默认 192.168.55.1）。
 *
 * 调用顺序与每步为什么：
 *   1. Device::open 连接，失败直接退出码 1。
 *   2. getDeviceInfo：电池、存储、内存、RTK。
 *   3. queryRecordStatus：确认设备是否待机（Idle）。stopRecord 之后设备应用
 *      会退出并由 systemd 重启，约 10~20 s 内连不上或查询失败，等它恢复到 Idle 再起
 *      下一个示例。
 *   4. getDeviceTime：设备时钟与本机时钟的偏差区间。
 *   5. getCameraCalib × 4、getImuCalib、getLidarCalib：标定信息。
 *
 * 运行：
 *   ./device_info_demo <设备IP>
 *   ./device_info_demo 192.168.55.1
 *
 * 预期输出与实测数字（数字随固件版本变化，仅供参考）：
 *   - 第一行 "已连接 <IP> (SDK 1.1.0)"；整个运行约 0.1 s。
 *   - 时钟偏差形如 "设备时钟 - 本机时钟 ∈ [+133.989, +133.989] s，区间宽度 = 往返
 *     时延 0.893 ms"：区间下界、上界由往返时延给出，真实偏差落在区间内，区间宽度
 *     等于往返时延（本机实测 0.9~1.6 ms）；偏差本身取决于两台机器的时钟，
 *     本机实测设备比主机快约 134 s，且随时间缓慢漂移（约 1 ms/min）。
 *     该偏差只适用于取自设备系统时钟的数据（图像、深度图），IMU / 位姿 / 点云
 *     不适用，见 docs/api_reference_v2.md §0.8。
 *   - 雷达标定 "type=" 在本机为空（设备缺 /mnt/system/factory-data/config/lidar.yaml），
 *     calibrated=1；相机 camera_3 在本机为 0x0 且 calibrated=0。
 *
 * 失败现象与处理：
 *   - "连接失败": 检查网线 / 网段 / 设备应用是否已起来（刚 stopRecord 后需等 10~20 s）。
 *   - 单项 "查询失败" 只打印，不影响退出码；退出码 0 只表示连接成功。
 *   - 设备时间 "无法获取": 应答无法解析，或设备固件不支持时间查询。
 *
 * 副作用：无录制副作用（不 startRecord / stopRecord）。副作用仅有一点：getLidarCalib
 * 每次调用会让设备日志多出 6 行 lidar.yaml 不存在的 <E> 记录（本机缺该文件），
 * 属于设备端噪声，不是故障。
 *
 * 线程模型：同步调用，无回调。
 */

#include "xg/robot_sdk.h"
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("用法: %s <设备IP>\n", argv[0]);
        return 1;
    }
    xg::Options opts;
    opts.deviceIp = argv[1];
    xg::Error err = xg::Error::Ok;
    auto device = xg::Device::open(opts, &err);
    if (!device) {
        std::printf("连接失败: %s\n", xg::toString(err));
        return 1;
    }
    std::printf("已连接 %s (SDK %s)\n\n", argv[1], xg::sdkVersion().c_str());

    // 设备状态
    std::printf("=== 设备状态 ===\n");
    xg::Device::DeviceInfo info;
    if (device->getDeviceInfo(&info) == xg::Error::Ok) {
        std::printf("  电池: %.1f%% (%.1f℃, %.2fV)\n",
                    info.batteryLevel, info.batteryTemp, info.batteryVoltage);
        std::printf("  存储: %lld / %lld MB\n",
                    (long long)(info.diskUsedKb / 1024),
                    (long long)(info.diskTotalKb / 1024));
        std::printf("  内存: %lld / %lld MB\n",
                    (long long)(info.memUsedKb / 1024),
                    (long long)(info.memTotalKb / 1024));
        std::printf("  RTK: state=%d, sat=%d\n", info.rtkState, info.satelliteNum);
    } else {
        std::printf("  查询失败\n");
    }

    // 录制状态（只读）：判断设备是否待机, 供其他示例起跑前确认
    std::printf("\n=== 录制状态 ===\n");
    xg::RecordStatus recStatus;
    const xg::Error recRc = device->queryRecordStatus(&recStatus);
    if (recRc == xg::Error::Ok) {
        std::printf("  录制状态: %s (raw=%d)%s\n", xg::toString(recStatus.state),
                    recStatus.rawCode, recStatus.warning ? " [WARNING]" : "");
    } else {
        std::printf("  查询失败: %s\n", xg::toString(recRc));
    }

    // 设备时间（只读）。设备应答的是设备系统时钟；偏差只能得到一个区间，
    // 区间宽度等于往返时延。该偏差只适用于取自设备系统时钟的数据（图像、深度图），
    // IMU / 位姿 / 点云取自 MCU 或雷达时钟，见 docs/api_reference_v2.md §0.8。
    std::printf("\n=== 设备时间 ===\n");
    xg::Device::DeviceTime devTime;
    const xg::Error timeRc = device->getDeviceTime(&devTime);
    if (timeRc == xg::Error::Ok) {
        const std::int64_t localUs =
            devTime.deviceEpochUs +
            static_cast<std::int64_t>(devTime.utcOffsetSec) * 1000000;
        const std::time_t localSec = static_cast<std::time_t>(localUs / 1000000);
        std::tm tmLocal;
        gmtime_r(&localSec, &tmLocal);  // 已加时区偏移，按 UTC 展开即为设备本地时间
        const int offAbsMin =
            (devTime.utcOffsetSec < 0 ? -devTime.utcOffsetSec
                                      : devTime.utcOffsetSec) / 60;
        std::printf("  设备本地时间: %04d-%02d-%02d %02d:%02d:%02d.%06d (UTC%c%d:%02d)\n",
                    tmLocal.tm_year + 1900, tmLocal.tm_mon + 1, tmLocal.tm_mday,
                    tmLocal.tm_hour, tmLocal.tm_min, tmLocal.tm_sec,
                    static_cast<int>(localUs % 1000000),
                    devTime.utcOffsetSec < 0 ? '-' : '+', offAbsMin / 60,
                    offAbsMin % 60);
        std::printf("  device_epoch_us = %lld\n",
                    static_cast<long long>(devTime.deviceEpochUs));
        std::printf("  设备时钟 - 本机时钟 ∈ [%+.3f, %+.3f] s，区间宽度 = 往返时延 %.3f ms\n",
                    devTime.offsetLowerUs() * 1e-6, devTime.offsetUpperUs() * 1e-6,
                    devTime.rttUs * 1e-3);
    } else if (timeRc == xg::Error::Protocol) {
        std::printf("  无法获取：应答无法解析，或设备固件不支持时间查询\n");
    } else {
        std::printf("  查询失败: %s\n", xg::toString(timeRc));
    }

    // 相机标定
    std::printf("\n=== 相机标定 ===\n");
    for (int i = 0; i < 4; ++i) {
        xg::Device::CameraCalib cam;
        if (device->getCameraCalib(i, &cam) == xg::Error::Ok) {
            std::printf("  [camera_%d] %ux%u model=%s calibrated=%d\n",
                        i, cam.width, cam.height,
                        cam.model.c_str(), cam.calibrated);
            std::printf("    intrinsic: fx=%.1f fy=%.1f cx=%.1f cy=%.1f\n",
                        cam.fx, cam.fy, cam.cx, cam.cy);
            std::printf("    distortion: [%.6f, %.6f, %.6f, %.6f]\n",
                        cam.distortion[0], cam.distortion[1],
                        cam.distortion[2], cam.distortion[3]);
        } else {
            std::printf("  [camera_%d] 查询失败\n", i);
        }
    }

    // IMU 标定
    std::printf("\n=== IMU 标定 ===\n");
    xg::Device::ImuCalib imu;
    if (device->getImuCalib(&imu) == xg::Error::Ok) {
        std::printf("  calibrated=%d\n", imu.calibrated);
        std::printf("  Bg: [%.6f, %.6f, %.6f]\n",
                    imu.gyroBias[0], imu.gyroBias[1], imu.gyroBias[2]);
        std::printf("  Ba: [%.6f, %.6f, %.6f]\n",
                    imu.accelBias[0], imu.accelBias[1], imu.accelBias[2]);
        std::printf("  extrinsic_imu_lidar (row 0): [%.4f, %.4f, %.4f, %.6f]\n",
                    imu.extrinsicImuLidar[0], imu.extrinsicImuLidar[1],
                    imu.extrinsicImuLidar[2], imu.extrinsicImuLidar[3]);
    } else {
        std::printf("  查询失败\n");
    }

    // 雷达标定
    std::printf("\n=== 雷达标定 ===\n");
    xg::Device::LidarCalib lidar;
    if (device->getLidarCalib(&lidar) == xg::Error::Ok) {
        std::printf("  type=%s calibrated=%d\n",
                    lidar.lidarType.c_str(), lidar.calibrated);
        std::printf("  extrinsic_imu_lidar (row 0): [%.4f, %.4f, %.4f, %.6f]\n",
                    lidar.extrinsicImuLidar[0], lidar.extrinsicImuLidar[1],
                    lidar.extrinsicImuLidar[2], lidar.extrinsicImuLidar[3]);
        std::printf("  extrinsic_camera_lidar (row 0): [%.4f, %.4f, %.4f, %.6f]\n",
                    lidar.extrinsicCameraLidar[0], lidar.extrinsicCameraLidar[1],
                    lidar.extrinsicCameraLidar[2], lidar.extrinsicCameraLidar[3]);
    } else {
        std::printf("  查询失败\n");
    }

    return 0;
}
