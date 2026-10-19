/**
 * @file   all_features_demo.cpp
 * @brief  综合功能演示：逐一验证 SDK 全部暴露能力
 *
 * 用法：./all_features_demo <设备IP> [订阅秒数, 默认15]
 *
 * 流程：
 *   1. 连接设备
 *   2. 查询设备状态（getDeviceInfo）
 *   3. 查询相机标定（getCameraCalib × 4）
 *   4. 查询 IMU 标定（getImuCalib）
 *   5. 查询雷达标定（getLidarCalib）
 *   6. 查询工程数据路径（getDataPath）
 *   7. 查询录制状态（queryRecordStatus）
 *   8. 启动录制（startRecord + 轮询确认）
 *   9. 订阅全部数据流（Pose/Cloud/IMU/RawCloud/Image/ErrCode）
 *  10. 等待 N 秒，统计各数据流帧数
 *  11. 设置定位初始位姿（setLocInitPose）
 *  12. 取消全部订阅
 *  13. 停止录制（stopRecord + 轮询确认）
 *  14. 查询工程数据路径（录制后）
 *  15. 断开连接
 *
 * 每个步骤打印结果，方便在真机上快速确认所有功能是否正常。
 */

#include "xg/robot_sdk.h"

#include <atomic>
#include <chrono>
#include <cstdio>
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
    std::atomic<int> cloud{0};
    std::atomic<int> imu{0};
    std::atomic<int> rawCloud{0};
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

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("用法: %s <设备IP> [订阅秒数, 默认15]\n", argv[0]);
        return 1;
    }
    const std::string deviceIp = argv[1];
    const int subscribeSec = (argc >= 3) ? std::atoi(argv[2]) : 15;

    std::printf("=== XGRIDS Robot SDK 全功能演示 (v%s) ===\n",
                xg::sdkVersion().c_str());
    std::printf("设备: %s, 订阅时长: %d 秒\n\n", deviceIp.c_str(), subscribeSec);

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
    printSeparator("2. 设备状态 (getDeviceInfo)");
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
    static SecondThrottle thrPose, thrCloud, thrImu, thrRawCloud;
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

    sub("subscribeCloud", dev->subscribeCloud([&cnt](const xg::CloudFrame& f) {
        cnt.cloud.fetch_add(1);
        if (thrCloud.tick()) {
            std::printf("    [cloud] t=%.3f points=%zu seq=%u\n",
                        f.timestampSec, f.count, f.seq);
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

    sub("subscribeRawCloud", dev->subscribeRawCloud([&cnt](const xg::CloudFrame& f) {
        cnt.rawCloud.fetch_add(1);
        if (thrRawCloud.tick()) {
            std::printf("    [rawcloud] t=%.3f points=%zu seq=%u\n",
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
    // 10. 等待数据
    // ================================================================
    printSeparator("10. 等待数据");
    std::printf("  收集 %d 秒 (每类数据每秒打印一组)...\n", subscribeSec);
    std::this_thread::sleep_for(std::chrono::seconds(subscribeSec));

    std::printf("\n  --- 数据统计 ---\n");
    std::printf("  pose:         %d 帧 (%.1f Hz)\n",
                cnt.pose.load(), (float)cnt.pose.load() / subscribeSec);
    std::printf("  cloud:        %d 帧\n", cnt.cloud.load());
    std::printf("  imu:          %d 帧 (%.1f Hz)\n",
                cnt.imu.load(), (float)cnt.imu.load() / subscribeSec);
    std::printf("  rawCloud:     %d 帧 (%.1f Hz)\n",
                cnt.rawCloud.load(), (float)cnt.rawCloud.load() / subscribeSec);
    std::printf("  imageLeft:    %d 帧\n", cnt.imageLeft.load());
    std::printf("  imageCenter:  %d 帧\n", cnt.imageCenter.load());
    std::printf("  imageRight:   %d 帧\n", cnt.imageRight.load());
    std::printf("  errCode:      %d 帧\n", cnt.errCode.load());

    // ================================================================
    // 11. 设置定位初始位姿
    // ================================================================
    printSeparator("11. 设置定位初始位姿 (setLocInitPose)");
    {
        xg::Device::InitPose pose;
        pose.timestampSec = 0.0;
        pose.posX = 0.0f;
        pose.posY = 0.0f;
        pose.posZ = 0.0f;
        pose.quatW = 1.0f;
        pose.quatX = 0.0f;
        pose.quatY = 0.0f;
        pose.quatZ = 0.0f;
        auto rc = dev->setLocInitPose(pose);
        std::printf("  result: %s\n", xg::toString(rc));
    }

    // ================================================================
    // 12. 取消全部订阅
    // ================================================================
    printSeparator("12. 取消全部订阅");
    dev->unsubscribePose();
    dev->unsubscribeCloud();
    dev->unsubscribeImu();
    dev->unsubscribeRawCloud();
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
