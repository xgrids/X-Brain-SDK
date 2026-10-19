/******************************************************************************
This file is part of XGRIDS Robot SDK.
Project: lixel_sdk_ros2
Author : XGRIDS Embedded Software Team
*******************************************************************************
History:
1. create this file. (DATE: 20250715)
******************************************************************************/

/**
 * @file   sdk_bridge_node.cpp
 * @brief  ROS2 Bridge Node: 将 lixel_sdk 数据订阅桥接为标准 ROS2 Topic
 *
 * 功能：
 *   - 通过 lixel_sdk 连接设备并订阅位姿/点云/IMU/图像/错误码数据
 *   - 将位姿数据转换为 nav_msgs/Odometry 发布到 /lixel/odom（map → base_link）
 *   - 同时发布 tf2_msgs/TFMessage 到 /lixel/tf（map → base_link 动态 TF）
 *   - base_link 即设备（IMU）系，SDK 订阅 device_locate/reloc_device_cloud，
 *     位姿/点云本身即设备系，桥接不做外参变换；载体变换由集成方自行叠加
 *   - 将点云数据转换为 sensor_msgs/PointCloud2 发布到 /lixel/cloud_points
 *   - 将 IMU 数据转换为 sensor_msgs/Imu 发布到 /lixel/imu
 *   - 将原始点云转换为 sensor_msgs/PointCloud2 发布到 /lixel/raw_cloud
 *   - 将 JPEG 图像转换为 sensor_msgs/CompressedImage 发布到 /lixel/camera/xxx
 *   - 将错误码转换为 diagnostic_msgs/DiagnosticArray 发布到 /lixel/diagnostics
 */

#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include "xg/robot_sdk.h"

class SdkBridgeNode : public rclcpp::Node {
public:
    SdkBridgeNode() : Node("sdk_bridge_node") {
        // 声明参数
        declare_parameter("device_ip", "192.168.123.103");
        declare_parameter("device_port", 7448);
        declare_parameter("connect_timeout_ms", 5000);
        // 世界系默认 map：设备输出为 SLAM/重定位全局位姿（含重定位，绑地图），
        // 语义上是 map 而非 odom（参见 coordinate_frames.md 与 ROS REP-105）。
        declare_parameter("map_frame", "map");
        // base_link 直接等于设备（IMU）系：SDK 订阅 device_locate/reloc_device_cloud，
        // 拿到的位姿/点云本身就是设备 IMU 基准，桥接不再做任何外参变换。
        declare_parameter("base_frame", "base_link");
        declare_parameter("odom_topic", "/lixel/odom");
        declare_parameter("tf_topic", "/lixel/tf");
        declare_parameter("cloud_topic", "/lixel/cloud_points");
        declare_parameter("imu_topic", "/lixel/imu");
        declare_parameter("raw_cloud_topic", "/lixel/raw_cloud");
        declare_parameter("image_left_topic", "/lixel/camera/left/compressed");
        declare_parameter("image_center_topic", "/lixel/camera/center/compressed");
        declare_parameter("image_right_topic", "/lixel/camera/right/compressed");
        declare_parameter("diagnostics_topic", "/lixel/diagnostics");

        // 读取参数
        std::string deviceIp = get_parameter("device_ip").as_string();
        int devicePort = get_parameter("device_port").as_int();
        int connectTimeoutMs = get_parameter("connect_timeout_ms").as_int();
        mapFrame_ = get_parameter("map_frame").as_string();
        baseFrame_ = get_parameter("base_frame").as_string();

        // 创建 Publisher
        std::string odomTopic = get_parameter("odom_topic").as_string();
        std::string tfTopic = get_parameter("tf_topic").as_string();
        std::string cloudTopic = get_parameter("cloud_topic").as_string();
        std::string imuTopic = get_parameter("imu_topic").as_string();
        std::string rawCloudTopic = get_parameter("raw_cloud_topic").as_string();
        std::string imageLeftTopic = get_parameter("image_left_topic").as_string();
        std::string imageCenterTopic = get_parameter("image_center_topic").as_string();
        std::string imageRightTopic = get_parameter("image_right_topic").as_string();
        std::string diagTopic = get_parameter("diagnostics_topic").as_string();

        odomPub_ = create_publisher<nav_msgs::msg::Odometry>(odomTopic, 10);
        tfPub_ = create_publisher<tf2_msgs::msg::TFMessage>(tfTopic, 10);
        cloudPub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloudTopic, 10);
        imuPub_ = create_publisher<sensor_msgs::msg::Imu>(imuTopic, 50);
        rawCloudPub_ = create_publisher<sensor_msgs::msg::PointCloud2>(rawCloudTopic, 10);
        imageLeftPub_ = create_publisher<sensor_msgs::msg::CompressedImage>(imageLeftTopic, 5);
        imageCenterPub_ = create_publisher<sensor_msgs::msg::CompressedImage>(imageCenterTopic, 5);
        imageRightPub_ = create_publisher<sensor_msgs::msg::CompressedImage>(imageRightTopic, 5);
        diagPub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagTopic, 10);

        // 连接设备
        RCLCPP_INFO(get_logger(), "Connecting to device %s:%d ...",
                    deviceIp.c_str(), devicePort);

        xg::Options opts;
        opts.deviceIp = deviceIp;
        opts.port = static_cast<uint16_t>(devicePort);
        opts.connectTimeoutMs = static_cast<uint32_t>(connectTimeoutMs);

        xg::Error err = xg::Error::Ok;
        device_ = xg::Device::open(opts, &err);
        if (!device_) {
            RCLCPP_FATAL(get_logger(), "Failed to connect: %s",
                         xg::toString(err));
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(get_logger(), "Connected to device.");

        // 自动启动录制
        RCLCPP_INFO(get_logger(), "Starting record...");
        auto startErr = device_->startRecord();
        if (startErr != xg::Error::Ok) {
            RCLCPP_WARN(get_logger(), "Start record failed: %s (device may already be recording)",
                        xg::toString(startErr));
        }

        // 等待进入录制状态
        RCLCPP_INFO(get_logger(), "Waiting for recording state...");
        xg::RecordStatus recStatus;
        for (int i = 0; i < 60; ++i) {  // 最多等 30 秒
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            if (device_->queryRecordStatus(&recStatus) != xg::Error::Ok) {
                continue;
            }
            if (recStatus.state == xg::RecordState::Recording) {
                RCLCPP_INFO(get_logger(), "Device is now recording.");
                break;
            }
            if (recStatus.state == xg::RecordState::Error) {
                RCLCPP_ERROR(get_logger(), "Device error: %s", recStatus.message.c_str());
                break;
            }
            if (i % 4 == 0) {
                RCLCPP_INFO(get_logger(), "  state: %s, waiting...", xg::toString(recStatus.state));
            }
        }

        // 订阅位姿
        auto poseErr = device_->subscribePose(
            [this](const xg::Pose& pose) { onPose(pose); });
        if (poseErr != xg::Error::Ok) {
            RCLCPP_ERROR(get_logger(), "Failed to subscribe pose: %s",
                         xg::toString(poseErr));
        } else {
            RCLCPP_INFO(get_logger(), "Pose subscription active (~100Hz).");
        }

        // 订阅点云
        auto cloudErr = device_->subscribeCloud(
            [this](const xg::CloudFrame& frame) { onCloud(frame); });
        if (cloudErr != xg::Error::Ok) {
            RCLCPP_ERROR(get_logger(), "Failed to subscribe cloud: %s",
                         xg::toString(cloudErr));
        } else {
            RCLCPP_INFO(get_logger(), "Cloud subscription active (~1Hz).");
        }

        // 订阅 IMU
        auto imuErr = device_->subscribeImu(
            [this](const xg::Device::Imu& imu) { onImu(imu); });
        if (imuErr != xg::Error::Ok) {
            RCLCPP_ERROR(get_logger(), "Failed to subscribe IMU: %s",
                         xg::toString(imuErr));
        } else {
            RCLCPP_INFO(get_logger(), "IMU subscription active (~200Hz).");
        }

        // 订阅原始点云
        auto rawCloudErr = device_->subscribeRawCloud(
            [this](const xg::CloudFrame& frame) { onRawCloud(frame); });
        if (rawCloudErr != xg::Error::Ok) {
            RCLCPP_ERROR(get_logger(), "Failed to subscribe raw cloud: %s",
                         xg::toString(rawCloudErr));
        } else {
            RCLCPP_INFO(get_logger(), "Raw cloud subscription active (~10Hz).");
        }

        // 订阅图像（三路相机）
        auto imgLeftErr = device_->subscribeImage(xg::Device::CameraId::Left,
            [this](const xg::Device::ImageFrame& f) { onImage(f); });
        if (imgLeftErr != xg::Error::Ok) {
            RCLCPP_WARN(get_logger(), "Failed to subscribe left camera.");
        }
        auto imgCenterErr = device_->subscribeImage(xg::Device::CameraId::Center,
            [this](const xg::Device::ImageFrame& f) { onImage(f); });
        if (imgCenterErr != xg::Error::Ok) {
            RCLCPP_WARN(get_logger(), "Failed to subscribe center camera.");
        }
        auto imgRightErr = device_->subscribeImage(xg::Device::CameraId::Right,
            [this](const xg::Device::ImageFrame& f) { onImage(f); });
        if (imgRightErr != xg::Error::Ok) {
            RCLCPP_WARN(get_logger(), "Failed to subscribe right camera.");
        }
        RCLCPP_INFO(get_logger(), "Image subscriptions active (~2fps).");

        // 订阅错误码
        auto errCodeErr = device_->subscribeErrCode(
            [this](const xg::Device::ErrCodeFrame& frame) { onErrCode(frame); });
        if (errCodeErr != xg::Error::Ok) {
            RCLCPP_WARN(get_logger(), "Failed to subscribe error codes.");
        } else {
            RCLCPP_INFO(get_logger(), "Error code subscription active (~10Hz).");
        }

        RCLCPP_INFO(get_logger(), "SDK Bridge Node ready (v0.3.1).");
    }

    ~SdkBridgeNode() override {
        if (device_) {
            device_->stopRecord();
            RCLCPP_INFO(get_logger(), "Record stopped.");
        }
        device_.reset();
        RCLCPP_INFO(get_logger(), "Device disconnected.");
    }

private:
    // ==================== 位姿回调 ====================
    // 设备（IMU）系位姿直接作为 map → base_link，不做任何外参变换。
    // base_link 即设备 IMU 基准；载体（狗/车等）到设备的变换由集成方自行叠加。
    void onPose(const xg::Pose& pose) {
        auto odom = nav_msgs::msg::Odometry();
        fillTimestamp(odom.header.stamp, pose.timestampSec);
        odom.header.frame_id = mapFrame_;
        odom.child_frame_id = baseFrame_;

        odom.pose.pose.position.x = pose.posX;
        odom.pose.pose.position.y = pose.posY;
        odom.pose.pose.position.z = pose.posZ;
        odom.pose.pose.orientation.w = pose.quatW;
        odom.pose.pose.orientation.x = pose.quatX;
        odom.pose.pose.orientation.y = pose.quatY;
        odom.pose.pose.orientation.z = pose.quatZ;
        odom.twist.twist.linear.x = pose.velX;
        odom.twist.twist.linear.y = pose.velY;
        odom.twist.twist.linear.z = pose.velZ;
        odom.twist.twist.angular.x = pose.gyroX;
        odom.twist.twist.angular.y = pose.gyroY;
        odom.twist.twist.angular.z = pose.gyroZ;

        // 发布 /odom（map → base_link）
        odomPub_->publish(odom);

        // 发布动态 /tf（map → base_link），值与 /odom 一致
        auto tfMsg = tf2_msgs::msg::TFMessage();
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = odom.header.stamp;
        tf.header.frame_id = mapFrame_;
        tf.child_frame_id = baseFrame_;
        tf.transform.translation.x = odom.pose.pose.position.x;
        tf.transform.translation.y = odom.pose.pose.position.y;
        tf.transform.translation.z = odom.pose.pose.position.z;
        tf.transform.rotation = odom.pose.pose.orientation;
        tfMsg.transforms.push_back(tf);
        tfPub_->publish(tfMsg);
    }

    // ==================== 点云回调 ====================
    // reloc 点云来自 device_locate 对应的 reloc_device_cloud（设备/IMU 系），
    // 直接标 base_link，无需外参变换。
    void onCloud(const xg::CloudFrame& frame) {
        auto msg = sensor_msgs::msg::PointCloud2();
        fillTimestamp(msg.header.stamp, frame.timestampSec);
        msg.header.frame_id = baseFrame_;

        // PointCloud2 格式描述
        msg.height = 1;
        msg.width = static_cast<uint32_t>(frame.count);
        msg.is_dense = true;
        msg.is_bigendian = false;
        msg.point_step = 16;  // 4 floats × 4 bytes
        msg.row_step = msg.point_step * msg.width;

        // 字段定义: x, y, z, intensity
        sensor_msgs::msg::PointField field;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;

        field.name = "x";
        field.offset = 0;
        msg.fields.push_back(field);
        field.name = "y";
        field.offset = 4;
        msg.fields.push_back(field);
        field.name = "z";
        field.offset = 8;
        msg.fields.push_back(field);
        field.name = "intensity";
        field.offset = 12;
        msg.fields.push_back(field);

        // 数据拷贝（xg_point_t 内存布局与 PointCloud2 XYZI 一致）
        msg.data.resize(msg.row_step);
        if (frame.count > 0 && frame.points != nullptr) {
            std::memcpy(msg.data.data(), frame.points,
                        frame.count * sizeof(xg_point_t));
        }

        // Task 14.2: 发布 /cloud_points
        cloudPub_->publish(msg);
    }

    // ==================== IMU 回调 ====================
    void onImu(const xg::Device::Imu& imu) {
        auto msg = sensor_msgs::msg::Imu();
        fillTimestamp(msg.header.stamp, imu.timestampSec);
        msg.header.frame_id = baseFrame_;

        msg.linear_acceleration.x = imu.accX;
        msg.linear_acceleration.y = imu.accY;
        msg.linear_acceleration.z = imu.accZ;
        msg.angular_velocity.x = imu.gyrX;
        msg.angular_velocity.y = imu.gyrY;
        msg.angular_velocity.z = imu.gyrZ;

        // 无姿态估计，设为 -1 表示未知
        msg.orientation_covariance[0] = -1.0;

        imuPub_->publish(msg);
    }

    // ==================== 原始点云回调 ====================
    void onRawCloud(const xg::CloudFrame& frame) {
        auto msg = sensor_msgs::msg::PointCloud2();
        fillTimestamp(msg.header.stamp, frame.timestampSec);
        msg.header.frame_id = baseFrame_;

        msg.height = 1;
        msg.width = static_cast<uint32_t>(frame.count);
        msg.is_dense = true;
        msg.is_bigendian = false;
        msg.point_step = 16;
        msg.row_step = msg.point_step * msg.width;

        sensor_msgs::msg::PointField field;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;

        field.name = "x";
        field.offset = 0;
        msg.fields.push_back(field);
        field.name = "y";
        field.offset = 4;
        msg.fields.push_back(field);
        field.name = "z";
        field.offset = 8;
        msg.fields.push_back(field);
        field.name = "intensity";
        field.offset = 12;
        msg.fields.push_back(field);

        msg.data.resize(msg.row_step);
        if (frame.count > 0 && frame.points != nullptr) {
            std::memcpy(msg.data.data(), frame.points,
                        frame.count * sizeof(xg_point_t));
        }

        rawCloudPub_->publish(msg);
    }

    // ==================== 图像回调 ====================
    void onImage(const xg::Device::ImageFrame& frame) {
        auto msg = sensor_msgs::msg::CompressedImage();
        fillTimestamp(msg.header.stamp, frame.timestampSec);
        msg.header.frame_id = baseFrame_;
        msg.format = "jpeg";

        if (frame.jpegData != nullptr && frame.jpegSize > 0) {
            msg.data.assign(frame.jpegData, frame.jpegData + frame.jpegSize);
        }

        switch (frame.cameraId) {
            case xg::Device::CameraId::Left:
                imageLeftPub_->publish(msg);
                break;
            case xg::Device::CameraId::Center:
                imageCenterPub_->publish(msg);
                break;
            case xg::Device::CameraId::Right:
                imageRightPub_->publish(msg);
                break;
        }
    }

    // ==================== 错误码回调 ====================
    void onErrCode(const xg::Device::ErrCodeFrame& frame) {
        if (frame.entries.empty()) {
            return;
        }

        auto msg = diagnostic_msgs::msg::DiagnosticArray();
        msg.header.stamp = now();

        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "lixel_device";
        status.hardware_id = "modeling_app";

        // 检查是否有严重错误（高 4 位 = 0x3）
        bool hasError = false;
        for (const auto& entry : frame.entries) {
            if ((entry.code >> 28) == 0x3) {
                hasError = true;
                break;
            }
        }
        status.level = hasError
            ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
            : diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "error codes from device";

        for (const auto& entry : frame.entries) {
            diagnostic_msgs::msg::KeyValue kv;
            kv.key = "code_" + std::to_string(entry.code);
            kv.value = std::to_string(entry.timestamp);
            status.values.push_back(kv);
        }

        msg.status.push_back(status);
        diagPub_->publish(msg);
    }

    // ==================== 工具函数 ====================
    static void fillTimestamp(builtin_interfaces::msg::Time& stamp,
                              double sec) {
        int64_t nsec = static_cast<int64_t>(sec * 1e9);
        stamp.sec = static_cast<int32_t>(nsec / 1000000000LL);
        stamp.nanosec = static_cast<uint32_t>(nsec % 1000000000LL);
    }

private:
    // SDK 设备句柄
    std::unique_ptr<xg::Device> device_;

    // ROS2 Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tfPub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rawCloudPub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr imageLeftPub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr imageCenterPub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr imageRightPub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagPub_;

    // 坐标系参数：map → base_link 两层，base_link 即设备（IMU）系
    std::string mapFrame_;
    std::string baseFrame_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SdkBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
