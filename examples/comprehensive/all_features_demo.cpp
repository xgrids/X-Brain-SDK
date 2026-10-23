/**
 * @file   all_features_demo.cpp
 * @brief  综合功能演示：逐一验证 SDK 全部暴露能力
 *
 * 用法：./all_features_demo <设备IP> [订阅秒=15] [X=0] [Y=0] [Z=0]
 *   ./all_features_demo 192.168.123.103
 *   ./all_features_demo 192.168.123.103 15 1 1 0.45
 *   X/Y/Z 单位 m，世界/地图系，默认 0 0 0（地图原点），与 pose_demo 一致；
 *   朝向固定为单位四元数 (w=1,x=0,y=0,z=0)。-h / --help 打印用法。
 *
 * 流程：
 *   1. 连接设备
 *   2. 查询设备状态（getDeviceInfo）与设备时间（getDeviceTime，只读）
 *   3. 查询相机标定（getCameraCalib × 4）
 *   4. 查询 IMU 标定（getImuCalib）
 *   5. 查询雷达标定（getLidarCalib）
 *   6. 查询工程数据路径（getDataPath）
 *   7. 查询录制状态（queryRecordStatus）
 *   8. 启动录制（startRecord + 轮询确认）
 *   9. 订阅全部数据流（Pose/IMU/DeviceUndistCloud/Image/ErrCode）
 *  10. 设置定位初始位姿（setLocInitPose，同时启动 Locate）
 *  11. 等待 N 秒，统计各数据流帧数
 *  12. 取消全部订阅
 *  13. 停止录制（stopRecord + 轮询确认）
 *  14. 查询工程数据路径（录制后）
 *  15. 断开连接
 *
 * 每个步骤打印结果，方便在真机上快速确认所有功能是否正常。
 *
 * 前置条件：
 *   - 设备已开机、应用已起来、状态为 Idle（先用 device_info_demo 确认连得上）。
 *   - 设备上有可用的重定位地图，且 [X] [Y] [Z] 与设备当前实际位置大致相符。
 *     默认 0 0 0 在本机地图上重定位可能失败（位姿 0 帧属预期）；想要位姿数据，
 *     请传设备实际所在附近的坐标（本机实测 1 1 0.45 可成功）。
 *
 * 调用顺序与每步为什么：
 *   - 第 8 步 startRecord 到 Recording：数据流依赖录制状态，Locate 在录制启动时完成初始化。
 *   - 第 9 步先订阅、第 10 步再 setLocInitPose：设备做 setInitPose + startLocateAlgo，
 *     Locate 启动后才发布位姿和去畸变点云，先订阅才不丢首帧。
 *   - 第 11 步等数据：此时位姿与去畸变点云计数才有意义。
 *   - 第 13 步 stopRecord 并等回 Idle。
 *
 * 预期输出与实测数字（注入有效位置 1 1 0.45 时；数字随固件版本变化，仅供参考）：
 *   - IMU 约 199.9 Hz；三路图像各约 10 Hz（实测 10 Hz，以设备配置为准）；
 *   - 重定位成功后位姿约 100 Hz，去畸变点云约 9.7 Hz（每帧 19.0k 点，
 *     范围 17.1k~19.3k），注入后 5.6~6.9 s 重定位成功（实测 5.64/6.02/6.26/6.85 s）；
 *   - 注入 0 0 0 时本机重定位失败：位姿 0 帧，去畸变点云仅有失败前后到达的少量缓存帧。
 *   - errCode 只统计帧数，是否有帧取决于设备端发布，本次板端验证未覆盖该项。
 *   - 设备时钟偏差为区间，宽度等于往返时延（本机实测 0.9~1.6 ms）。
 *   - 雷达标定 type 在本机为空（设备缺 lidar.yaml），calibrated=1。
 *   - 从雷达待机起 startRecord 到 Recording 约 17.5~18.6 s，新进程约 7.5~8.5 s。
 *
 * 失败现象与处理：
 *   - 第 8 步未进入 Recording：后续订阅无数据，先查设备状态再重跑。
 *   - 第 10 步 Rejected：定位未初始化 / 无地图 / 四元数非法；Timeout：命令可能已送达，
 *     不要立刻重复注入。
 *   - 位姿或去畸变点云 0 帧：Locate 未启动或重定位未成功（见 reloc_demo）。
 *
 * 副作用：
 *   - 会 startRecord，每次运行在设备上生成一个录制工程（本机实测 87~103 MB/次）。
 *   - 第 13 步 stopRecord 之后设备应用会退出并由 systemd 重启，约 10~20 s 内设备
 *     不可用。下一次运行请先用 device_info_demo 确认连得上且状态为 Idle；
 *     重启期间下发的 startRecord 会被排队，收尾的 stopRecord 在非录制态是空操作。
 *   - 本示例只做查询、startRecord / stopRecord、订阅和 setLocInitPose，
 *     不改网络、不上传地图。
 *
 * 线程模型：回调在 Zenoh RX 线程触发，回调内不得阻塞；本示例回调只做计数和限频打印。
 */

#include "xg/robot_sdk.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

namespace {

/**
 * @brief 秒级节流器：每满 1 秒放行一次，把高频回调降到每秒打印一行。
 * @note  回调在 SDK 内部线程触发，用原子时间戳做无锁节流；偶发竞态只影响
 *        是否多打印一行，不影响正确性，对观察用途足够。
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

// 简易计数器
struct DataCounters {
    std::atomic<int> pose{0};
    std::atomic<int> imu{0};
    std::atomic<int> undistCloud{0};
    std::atomic<int> imageLeft{0};
    std::atomic<int> imageCenter{0};
    std::atomic<int> imageRight{0};
    std::atomic<int> errCode{0};
};

bool waitForState(xg::Device* dev, xg::RecordState target, long maxWaitMs) {
    xg::RecordStatus st;
    const auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - t0)
               .count() < maxWaitMs) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(XG_RECOMMENDED_POLL_INTERVAL_MS));
        if (dev->queryRecordStatus(&st) != xg::Error::Ok) {
            continue;
        }
        if (st.state == target) {
            return true;
        }
        if (st.state == xg::RecordState::Error) {
            std::printf("    设备故障: %s\n", st.message.c_str());
            return false;
        }
    }
    return false;
}

void printSeparator(const char* title) {
    std::printf("\n========== %s ==========\n", title);
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

void printUsage(const char* prog) {
    std::printf("用法: %s <设备IP> [订阅秒=15] [X=0] [Y=0] [Z=0]\n", prog);
    std::printf("  X/Y/Z: 定位初值位置, 单位 m, 世界/地图系, 默认 0 0 0\n");
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
    const std::string deviceIp = argv[1];
    const int subscribeSec = (argc >= 3) ? std::atoi(argv[2]) : 15;
    float posX = 0.0f, posY = 0.0f, posZ = 0.0f;
    if ((argc >= 4 && !parseFloatArg(argv[3], &posX)) ||
        (argc >= 5 && !parseFloatArg(argv[4], &posY)) ||
        (argc >= 6 && !parseFloatArg(argv[5], &posZ))) {
        std::printf("参数错误: X/Y/Z 必须是数字 (单位 m)\n");
        printUsage(argv[0]);
        return 1;
    }

    std::printf("=== XGRIDS Robot SDK 全功能演示 (v%s) ===\n",
                xg::sdkVersion().c_str());
    std::printf("设备: %s, 订阅时长: %d 秒, 初值 pos=(%.3f, %.3f, %.3f)\n\n",
                deviceIp.c_str(), subscribeSec, posX, posY, posZ);

    // ================================================================
    // 1. 连接设备
    // ================================================================
    printSeparator("1. 连接设备");

    xg::Options opts;
    opts.deviceIp = deviceIp;

    xg::Error err;
    auto dev = xg::Device::open(opts, &err);
    if (!dev) {
        std::printf("  连接失败: %s\n", xg::toString(err));
        return 1;
    }
    std::printf("  连接成功, isConnected=%d\n", dev->isConnected());

    // ================================================================
    // 2. 查询设备状态
    // ================================================================
    printSeparator("2. 设备状态与设备时间 (getDeviceInfo / getDeviceTime)");
    {
        xg::Device::DeviceInfo info;
        auto rc = dev->getDeviceInfo(&info);
        std::printf("  result: %s\n", xg::toString(static_cast<xg::Error>(rc)));
        if (rc == xg::Error::Ok) {
            std::printf("  电池: %.1f%% / %.1f°C / %.2fV\n",
                        info.batteryLevel, info.batteryTemp, info.batteryVoltage);
            std::printf("  磁盘: %lld / %lld KB (%.1f%%)\n",
                        (long long)info.diskUsedKb, (long long)info.diskTotalKb,
                        info.diskTotalKb > 0 ? 100.0 * info.diskUsedKb / info.diskTotalKb : 0.0);
            std::printf("  内存: %lld / %lld KB (%.1f%%)\n",
                        (long long)info.memUsedKb, (long long)info.memTotalKb,
                        info.memTotalKb > 0 ? 100.0 * info.memUsedKb / info.memTotalKb : 0.0);
            std::printf("  RTK: state=%d, 卫星数=%d\n", info.rtkState, info.satelliteNum);
            std::printf("  建图状态码: %d\n", info.modelingState);
        }
        // 设备时间（只读）。偏差只能得到区间，宽度等于往返时延；该区间只适用于取自
        // 设备系统时钟的数据（图像、深度图），IMU / 位姿 / 点云见 api_reference_v2.md §0.8。
        xg::Device::DeviceTime devTime;
        auto trc = dev->getDeviceTime(&devTime);
        std::printf("  设备时间 result: %s\n",
                    xg::toString(static_cast<xg::Error>(trc)));
        if (trc == xg::Error::Ok) {
            std::printf("  device_epoch_us = %lld (UTC%+.2f h)\n",
                        (long long)devTime.deviceEpochUs,
                        devTime.utcOffsetSec / 3600.0);
            std::printf("  设备时钟 - 本机时钟 ∈ [%+.3f, %+.3f] s, 区间宽度 = 往返时延 %.3f ms\n",
                        devTime.offsetLowerUs() * 1e-6,
                        devTime.offsetUpperUs() * 1e-6, devTime.rttUs * 1e-3);
        } else if (trc == xg::Error::Protocol) {
            std::printf("  (设备固件不支持时间查询, 或应答无法解析)\n");
        }
    }

    // ================================================================
    // 3. 查询相机标定
    // ================================================================
    printSeparator("3. 相机标定 (getCameraCalib)");
    {
        for (int i = 0; i < 4; ++i) {
            xg::Device::CameraCalib cam;
            auto rc = dev->getCameraCalib(i, &cam);
            if (rc == xg::Error::Ok) {
                std::printf("  camera_%d: %ux%u, model=%s, calibrated=%d\n",
                            i, cam.width, cam.height,
                            cam.model.c_str(), cam.calibrated);
                std::printf("         fx=%.2f fy=%.2f cx=%.2f cy=%.2f\n",
                            cam.fx, cam.fy, cam.cx, cam.cy);
                std::printf("         dist=[%.4f, %.4f, %.4f, %.4f]\n",
                            cam.distortion[0], cam.distortion[1],
                            cam.distortion[2], cam.distortion[3]);
            } else {
                std::printf("  camera_%d: 查询失败 (%s)\n", i, xg::toString(rc));
            }
        }
    }

    // ================================================================
    // 4. 查询 IMU 标定
    // ================================================================
    printSeparator("4. IMU 标定 (getImuCalib)");
    {
        xg::Device::ImuCalib imu;
        auto rc = dev->getImuCalib(&imu);
        if (rc == xg::Error::Ok) {
            std::printf("  calibrated=%d\n", imu.calibrated);
            std::printf("  Ba: [%.6f, %.6f, %.6f]\n",
                        imu.accelBias[0], imu.accelBias[1], imu.accelBias[2]);
            std::printf("  Bg: [%.6f, %.6f, %.6f]\n",
                        imu.gyroBias[0], imu.gyroBias[1], imu.gyroBias[2]);
            std::printf("  T_imu_lidar (Lidar->IMU) row0: [%.4f, %.4f, %.4f, %.6f]\n",
                        imu.extrinsicImuLidar[0], imu.extrinsicImuLidar[1],
                        imu.extrinsicImuLidar[2], imu.extrinsicImuLidar[3]);
        } else {
            std::printf("  查询失败: %s\n", xg::toString(rc));
        }
    }

    // ================================================================
    // 5. 查询雷达标定
    // ================================================================
    printSeparator("5. 雷达标定 (getLidarCalib)");
    {
        xg::Device::LidarCalib lidar;
        auto rc = dev->getLidarCalib(&lidar);
        if (rc == xg::Error::Ok) {
            std::printf("  type=%s, calibrated=%d\n",
                        lidar.lidarType.c_str(), lidar.calibrated);
            std::printf("  T_imu_lidar (Lidar->IMU) row0: [%.4f, %.4f, %.4f, %.6f]\n",
                        lidar.extrinsicImuLidar[0], lidar.extrinsicImuLidar[1],
                        lidar.extrinsicImuLidar[2], lidar.extrinsicImuLidar[3]);
            std::printf("  T_cam0_lidar (Lidar->Cam0) row0: [%.4f, %.4f, %.4f, %.6f]\n",
                        lidar.extrinsicCameraLidar[0], lidar.extrinsicCameraLidar[1],
                        lidar.extrinsicCameraLidar[2], lidar.extrinsicCameraLidar[3]);
        } else {
            std::printf("  查询失败: %s\n", xg::toString(rc));
        }
    }

    // ================================================================
    // 6. 查询工程数据路径（录制前）
    // ================================================================
    printSeparator("6. 工程数据路径 (getDataPath) - 录制前");
    {
        std::string path;
        auto rc = dev->getDataPath(&path);
        std::printf("  result: %s\n", xg::toString(rc));
        std::printf("  path: \"%s\"\n", path.c_str());
    }

    // ================================================================
    // 7. 查询录制状态
    // ================================================================
    printSeparator("7. 录制状态 (queryRecordStatus)");
    {
        xg::RecordStatus st;
        auto rc = dev->queryRecordStatus(&st);
        std::printf("  result: %s\n", xg::toString(rc));
        if (rc == xg::Error::Ok) {
            std::printf("  state=%s, warning=%d, rawCode=%d\n",
                        xg::toString(st.state), st.warning, st.rawCode);
            std::printf("  message=\"%s\"\n", st.message.c_str());
        }
    }

    // ================================================================
    // 8. 启动录制
    // ================================================================
    printSeparator("8. 启动录制 (startRecord + 轮询)");
    {
        auto rc = dev->startRecord();
        std::printf("  startRecord: %s\n", xg::toString(rc));
        if (rc == xg::Error::Ok) {
            std::printf("  轮询等待...\n");
            if (waitForState(dev.get(), xg::RecordState::Recording, 120000)) {
                std::printf("  已进入录制状态.\n");
            } else {
                std::printf("  WARNING: 未能进入录制, 后续订阅可能无数据.\n");
            }
        }
    }

    // ================================================================
    // 9. 订阅全部数据流
    // ================================================================
    printSeparator("9. 订阅全部数据");
    DataCounters cnt;

    auto sub = [](const char* name, xg::Error rc) {
        std::printf("  %s: %s\n", name, xg::toString(rc));
    };

    // 每类数据一个秒级节流器，订阅期间每秒打印一组实际内容
    static SecondThrottle thrPose, thrImu, thrUndistCloud;
    static SecondThrottle thrImgL, thrImgC, thrImgR, thrErr;

    sub("subscribePose", dev->subscribePose([&cnt](const xg::Pose& p) {
        cnt.pose.fetch_add(1);
        if (thrPose.tick()) {
            std::printf("    [pose ] t=%.3f pos=(%.3f, %.3f, %.3f) "
                        "quat=(%.3f, %.3f, %.3f, %.3f) vel=(%.3f, %.3f, %.3f) "
                        "err=%u\n",
                        p.timestampSec, p.posX, p.posY, p.posZ, p.quatW,
                        p.quatX, p.quatY, p.quatZ, p.velX, p.velY, p.velZ,
                        p.errorCode);
        }
    }));


    sub("subscribeImu", dev->subscribeImu([&cnt](const xg::Device::Imu& imu) {
        cnt.imu.fetch_add(1);
        if (thrImu.tick()) {
            std::printf("    [imu  ] t=%.6f acc=(%.3f, %.3f, %.3f) "
                        "gyr=(%.3f, %.3f, %.3f)\n",
                        imu.timestampSec, imu.accX, imu.accY, imu.accZ,
                        imu.gyrX, imu.gyrY, imu.gyrZ);
        }
    }));

    sub("subscribeDeviceUndistCloud",
        dev->subscribeDeviceUndistCloud([&cnt](const xg::CloudFrame& f) {
        cnt.undistCloud.fetch_add(1);
        if (thrUndistCloud.tick()) {
            std::printf("    [undist cloud] t=%.3f points=%zu seq=%u\n",
                        f.timestampSec, f.count, f.seq);
        }
    }));

    sub("subscribeImage(Left)", dev->subscribeImage(xg::Device::CameraId::Left,
        [&cnt](const xg::Device::ImageFrame& f) {
            cnt.imageLeft.fetch_add(1);
            if (thrImgL.tick()) {
                std::printf("    [img L] t=%.3f jpeg=%zu bytes seq=%u\n",
                            f.timestampSec, f.jpegSize, f.seq);
            }
        }));

    sub("subscribeImage(Center)", dev->subscribeImage(xg::Device::CameraId::Center,
        [&cnt](const xg::Device::ImageFrame& f) {
            cnt.imageCenter.fetch_add(1);
            if (thrImgC.tick()) {
                std::printf("    [img C] t=%.3f jpeg=%zu bytes seq=%u\n",
                            f.timestampSec, f.jpegSize, f.seq);
            }
        }));

    sub("subscribeImage(Right)", dev->subscribeImage(xg::Device::CameraId::Right,
        [&cnt](const xg::Device::ImageFrame& f) {
            cnt.imageRight.fetch_add(1);
            if (thrImgR.tick()) {
                std::printf("    [img R] t=%.3f jpeg=%zu bytes seq=%u\n",
                            f.timestampSec, f.jpegSize, f.seq);
            }
        }));

    sub("subscribeErrCode", dev->subscribeErrCode(
        [&cnt](const xg::Device::ErrCodeFrame& f) {
            cnt.errCode.fetch_add(1);
            if (thrErr.tick()) {
                std::printf("    [err  ] entries=%zu", f.entries.size());
                if (!f.entries.empty()) {
                    std::printf(" first_code=%u", f.entries.front().code);
                }
                std::printf("\n");
            }
        }));

    // ================================================================
    // 10. 设置定位初始位姿
    // ================================================================
    printSeparator("10. 设置定位初始位姿 (setLocInitPose)");
    // setLocInitPose 同时启动设备端 Locate, 之后才有位姿和 Locate 去畸变点云。
    // 放在订阅之后、等数据之前: 先订阅才不丢首帧, 第 11 步的位姿 / 去畸变点云
    // 计数才有意义。注入命令行给定的 X/Y/Z (默认地图原点 0 0 0, 单位 m,
    // 世界/地图系) + 单位四元数; 应传设备当前在地图上的位置附近。
    {
        xg::Device::InitPose pose;
        pose.timestampSec = 0.0;
        pose.posX = posX;
        pose.posY = posY;
        pose.posZ = posZ;
        pose.quatW = 1.0f;
        pose.quatX = 0.0f;
        pose.quatY = 0.0f;
        pose.quatZ = 0.0f;
        auto rc = dev->setLocInitPose(pose);
        std::printf("  result: %s\n", xg::toString(rc));
        if (rc == xg::Error::Ok) {
            std::printf("  设备已接受初值 (仅表示已收下, 不代表重定位完成).\n");
        } else if (rc == xg::Error::Rejected) {
            std::printf("  设备拒绝 (定位未初始化 / 无地图 / 四元数非法), "
                        "位姿与去畸变点云将为 0 帧.\n");
        } else if (rc == xg::Error::Timeout) {
            std::printf("  超时, 命令可能已送达, 以第 11 步的计数为准.\n");
        }
    }

    // ================================================================
    // 11. 等待数据
    // ================================================================
    printSeparator("11. 等待数据");
    std::printf("  收集 %d 秒 (每类数据每秒打印一组)...\n", subscribeSec);
    std::this_thread::sleep_for(std::chrono::seconds(subscribeSec));

    std::printf("\n  --- 数据统计 ---\n");
    std::printf("  pose:         %d 帧 (%.1f Hz)\n",
                cnt.pose.load(), (float)cnt.pose.load() / subscribeSec);
    std::printf("  imu:            %d 帧 (%.1f Hz)\n",
                cnt.imu.load(), (float)cnt.imu.load() / subscribeSec);
    std::printf("  undistCloud: %d 帧 (%.1f Hz)\n", cnt.undistCloud.load(),
                (float)cnt.undistCloud.load() / subscribeSec);
    std::printf("  imageLeft:    %d 帧\n", cnt.imageLeft.load());
    std::printf("  imageCenter:  %d 帧\n", cnt.imageCenter.load());
    std::printf("  imageRight:   %d 帧\n", cnt.imageRight.load());
    std::printf("  errCode:      %d 帧\n", cnt.errCode.load());
    if (cnt.pose.load() == 0 || cnt.undistCloud.load() == 0) {
        std::printf("  提示: 位姿或去畸变点云 0 帧。原因: Locate 未启动或重定位未成功 "
                    "(注入失败 / 初值与实际位置相差过大 / 无地图), 见 reloc_demo.\n");
    }

    // ================================================================
    // 12. 取消全部订阅
    // ================================================================
    printSeparator("12. 取消全部订阅");
    dev->unsubscribePose();
    dev->unsubscribeImu();
    dev->unsubscribeDeviceUndistCloud();
    dev->unsubscribeImage(xg::Device::CameraId::Left);
    dev->unsubscribeImage(xg::Device::CameraId::Center);
    dev->unsubscribeImage(xg::Device::CameraId::Right);
    dev->unsubscribeErrCode();
    std::printf("  全部订阅已取消.\n");

    // ================================================================
    // 13. 停止录制
    // ================================================================
    printSeparator("13. 停止录制 (stopRecord + 轮询)");
    {
        auto rc = dev->stopRecord();
        std::printf("  stopRecord: %s\n", xg::toString(rc));
        if (rc == xg::Error::Ok) {
            std::printf("  轮询等待...\n");
            if (waitForState(dev.get(), xg::RecordState::Idle, 60000)) {
                std::printf("  已停止录制.\n");
            } else {
                std::printf("  WARNING: 停止超时.\n");
            }
        }
    }

    // ================================================================
    // 14. 查询工程数据路径（录制后）
    // ================================================================
    printSeparator("14. 工程数据路径 (getDataPath) - 录制后");
    {
        std::string path;
        auto rc = dev->getDataPath(&path);
        std::printf("  result: %s\n", xg::toString(rc));
        std::printf("  path: \"%s\"\n", path.c_str());
    }

    // ================================================================
    // 15. 断开
    // ================================================================
    printSeparator("15. 断开连接");
    dev.reset();
    std::printf("  已断开.\n");

    std::printf("\n=== 全功能演示完成 ===\n");
    return 0;
}
