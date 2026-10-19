#include "xg/robot_sdk.h"
#include <cstdio>
#include <cstring>
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
