/******************************************************************************
This file is part of XGRIDS Robot SDK.
Project: lixel_ros
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
 *   - 将设备位姿发布为 map → imu_link 的里程计与动态 TF（SDK 测的是设备/IMU 位姿）
 *   - 发布 imu_link → base_link 的静态 TF（参数即发布方向，不做取逆）
 *   - 将 IMU 数据转换为 sensor_msgs/Imu 发布到 /lixel/imu
 *   - 将设备系 Locate 去畸变点云（key device_undist_cloud）转换为
 *     sensor_msgs/PointCloud2 发布到 /lixel/device_undist_cloud
 *   - 将 JPEG 图像转换为 sensor_msgs/CompressedImage 发布到 /lixel/camera/xxx
 *   - 将错误码转换为 diagnostic_msgs/DiagnosticArray 发布到 /lixel/diagnostics
 */

#include <cmath>
#include <cstring>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>


#include "xg/robot_sdk.h"

class SdkBridgeNode : public rclcpp::Node {
public:
    SdkBridgeNode() : Node("sdk_bridge_node") {
        // 声明参数
        declare_parameter("device_ip", "192.168.123.103");
        // 默认值与 config/default_params.yaml 一致（以 yaml 为准）。SDK 1.0.0 的
        // modeling_app 在 7448，7447 是 0.3.x 的旧端口，见 yaml 注释。
        declare_parameter("device_port", 7448);
        declare_parameter("connect_timeout_ms", 5000);
        declare_parameter("odom_frame", "map");
        declare_parameter("base_frame", "base_link");
        declare_parameter("sensor_frame", "sensor_link");
        declare_parameter("lidar_frame", "imu_link");
        // imu_link -> base_link 的静态外参（IMU 坐标系 -> ODOM 坐标系），就是本节点
        // 原样发布的那条 TF。参数方向与发布方向一致，节点不做任何取逆或取反，
        // 含义是"base_link 在 imu_link 中的位姿"。
        //
        // 以 config/default_params.yaml 为准，这里的默认值只在没加载 yaml 时
        // 生效，与 yaml 保持一致：
        //   旋转：固定的轴系重定义，手持扫描仪时 base X 朝前。
        //         base X = IMU -z,  base Y = IMU +x,  base Z = IMU -y
        //   平移：0，ODOM 原点就在扫描仪原点（IMU 点）。平移按 IMU 轴向填，
        //         换算方法见 yaml 注释和 README。
        declare_parameter("lidar_to_base_x", 0.0);
        declare_parameter("lidar_to_base_y", 0.0);
        declare_parameter("lidar_to_base_z", 0.0);
        declare_parameter("lidar_to_base_qx", 0.5);
        declare_parameter("lidar_to_base_qy", 0.5);
        declare_parameter("lidar_to_base_qz", -0.5);
        declare_parameter("lidar_to_base_qw", 0.5);
        declare_parameter("odom_topic", "/odom");
        declare_parameter("tf_topic", "/tf");
        declare_parameter("imu_topic", "/lixel/imu");
        declare_parameter("device_undist_cloud_topic",
                          "/lixel/device_undist_cloud");
        declare_parameter("image_left_topic", "/lixel/camera/left/compressed");
        declare_parameter("image_center_topic", "/lixel/camera/center/compressed");
        declare_parameter("image_right_topic", "/lixel/camera/right/compressed");
        declare_parameter("diagnostics_topic", "/lixel/diagnostics");

        // 读取参数
        std::string deviceIp = get_parameter("device_ip").as_string();
        int devicePort = get_parameter("device_port").as_int();
        int connectTimeoutMs = get_parameter("connect_timeout_ms").as_int();
        odomFrame_ = get_parameter("odom_frame").as_string();
        baseFrame_ = get_parameter("base_frame").as_string();
        sensorFrame_ = get_parameter("sensor_frame").as_string();
        imuFrame_ = get_parameter("lidar_frame").as_string();
        const double i2bX = get_parameter("lidar_to_base_x").as_double();
        const double i2bY = get_parameter("lidar_to_base_y").as_double();
        const double i2bZ = get_parameter("lidar_to_base_z").as_double();
        const double i2bQx = get_parameter("lidar_to_base_qx").as_double();
        const double i2bQy = get_parameter("lidar_to_base_qy").as_double();
        const double i2bQz = get_parameter("lidar_to_base_qz").as_double();
        const double i2bQw = get_parameter("lidar_to_base_qw").as_double();
        // 缓存 imu_link -> base_link，onPose 要用它把设备位姿复合成
        // base_link 位姿。构造 tf2::Quaternion 的参数顺序是 (x, y, z, w)。
        tfImuBase_ = tf2::Transform(
            tf2::Quaternion(i2bQx, i2bQy, i2bQz, i2bQw),
            tf2::Vector3(i2bX, i2bY, i2bZ));
        // 创建 Publisher
        std::string odomTopic = get_parameter("odom_topic").as_string();
        std::string tfTopic = get_parameter("tf_topic").as_string();
        std::string imuTopic = get_parameter("imu_topic").as_string();
        std::string undistCloudTopic =
            get_parameter("device_undist_cloud_topic").as_string();
        std::string imageLeftTopic = get_parameter("image_left_topic").as_string();
        std::string imageCenterTopic = get_parameter("image_center_topic").as_string();
        std::string imageRightTopic = get_parameter("image_right_topic").as_string();
        std::string diagTopic = get_parameter("diagnostics_topic").as_string();

        odomPub_ = create_publisher<nav_msgs::msg::Odometry>(odomTopic, 10);
        tfPub_ = create_publisher<tf2_msgs::msg::TFMessage>(tfTopic, 10);
        imuPub_ = create_publisher<sensor_msgs::msg::Imu>(imuTopic, 50);
        undistCloudPub_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(undistCloudTopic, 10);
        imageLeftPub_ = create_publisher<sensor_msgs::msg::CompressedImage>(imageLeftTopic, 5);
        imageCenterPub_ = create_publisher<sensor_msgs::msg::CompressedImage>(imageCenterTopic, 5);
        imageRightPub_ = create_publisher<sensor_msgs::msg::CompressedImage>(imageRightTopic, 5);
        diagPub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagTopic, 10);

        // imu_link -> base_link：参数即发布内容，方向一致，不做任何换算。
        // 用 StaticTransformBroadcaster 而不是自己发 /tf_static，是为了拿到正确的
        // transient_local QoS —— 后启动的节点才能收到这条只发一次的变换。
        staticTfBroadcaster_ =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        geometry_msgs::msg::TransformStamped mount;
        mount.header.stamp = now();
        mount.header.frame_id = imuFrame_;
        mount.child_frame_id = baseFrame_;
        mount.transform.translation.x = i2bX;
        mount.transform.translation.y = i2bY;
        mount.transform.translation.z = i2bZ;
        mount.transform.rotation.x = i2bQx;
        mount.transform.rotation.y = i2bQy;
        mount.transform.rotation.z = i2bQz;
        mount.transform.rotation.w = i2bQw;

        // 非单位四元数会让下游 TF 静默算错，宁可启动时就报出来。
        const double qn = std::sqrt(i2bQx * i2bQx + i2bQy * i2bQy +
                                    i2bQz * i2bQz + i2bQw * i2bQw);
        if (!std::isfinite(qn) || std::abs(qn - 1.0) > 1e-3) {
            RCLCPP_ERROR(get_logger(),
                         "lidar_to_base 四元数模长 %.6f 偏离 1，静态 TF 会失真："
                         "(%.6f, %.6f, %.6f, %.6f)",
                         qn, i2bQx, i2bQy, i2bQz, i2bQw);
        }

        staticTfBroadcaster_->sendTransform(mount);
        RCLCPP_INFO(get_logger(),
                    "Static TF %s -> %s: t=(%.6f, %.6f, %.6f) "
                    "q=(%.6f, %.6f, %.6f, %.6f) |q|=%.6f",
                    imuFrame_.c_str(), baseFrame_.c_str(),
                    i2bX, i2bY, i2bZ, i2bQx, i2bQy, i2bQz, i2bQw, qn);

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
            // 不在构造函数里 shutdown: 由 main 通过 connected() 判断后退出,
            // 否则 shutdown 之后 main 仍会 spin, 进程 abort
            RCLCPP_FATAL(get_logger(), "Failed to connect: %s",
                         xg::toString(err));
            return;
        }
        RCLCPP_INFO(get_logger(), "Connected to device.");

        // 订阅位姿
        auto poseErr = device_->subscribePose(
            [this](const xg::Pose& pose) { onPose(pose); });
        if (poseErr != xg::Error::Ok) {
            RCLCPP_ERROR(get_logger(), "Failed to subscribe pose: %s",
                         xg::toString(poseErr));
        } else {
            RCLCPP_INFO(get_logger(), "Pose subscription active (~100Hz).");
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

        // 订阅设备(IMU)系 Locate 去畸变点云（建图阶段该 key 无数据）
        auto undistCloudErr = device_->subscribeDeviceUndistCloud(
            [this](const xg::CloudFrame& frame) { onDeviceUndistCloud(frame); });
        if (undistCloudErr != xg::Error::Ok) {
            RCLCPP_ERROR(get_logger(),
                         "Failed to subscribe device undist cloud: %s!",
                         xg::toString(undistCloudErr));
        } else {
            RCLCPP_INFO(get_logger(),
                        "Device undist cloud subscription active (~10Hz).");
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
        RCLCPP_INFO(get_logger(), "Image subscriptions active (rate set by device jpeg.fps).");

        // 订阅错误码
        auto errCodeErr = device_->subscribeErrCode(
            [this](const xg::Device::ErrCodeFrame& frame) { onErrCode(frame); });
        if (errCodeErr != xg::Error::Ok) {
            RCLCPP_WARN(get_logger(), "Failed to subscribe error codes.");
        } else {
            RCLCPP_INFO(get_logger(), "Error code subscription active (~10Hz).");
        }

        RCLCPP_INFO(get_logger(), "SDK Bridge Node ready (SDK %s).",
                    xg::sdkVersion().c_str());
    }

    ~SdkBridgeNode() override {
        device_.reset();
        RCLCPP_INFO(get_logger(), "Device disconnected.");
    }

    /// 是否已连上设备（连接失败时构造函数提前返回, 节点未完成初始化）
    bool connected() const { return device_ != nullptr; }

private:
    // ==================== 位姿回调 ====================
    void onPose(const xg::Pose& pose) {
        const auto stamp = this->now();

        // SDK 报的是设备（IMU/雷达）位姿，即 map -> imu_link。
        const tf2::Transform tMapImu(
            tf2::Quaternion(pose.quatX, pose.quatY, pose.quatZ, pose.quatW),
            tf2::Vector3(pose.posX, pose.posY, pose.posZ));

        // /odom 声明 map -> base_link，就必须给出 base_link 的位姿。
        // 只改 child_frame_id 而照发设备位姿，等于给雷达位姿贴上 base_link
        // 的标签，下游会按错误的原点做规划。
        const tf2::Transform tMapBase = tMapImu * tfImuBase_;

        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = stamp;
        odom.header.frame_id = odomFrame_;
        odom.child_frame_id = baseFrame_;

        const tf2::Vector3& pBase = tMapBase.getOrigin();
        odom.pose.pose.position.x = pBase.x();
        odom.pose.pose.position.y = pBase.y();
        odom.pose.pose.position.z = pBase.z();
        const tf2::Quaternion qBase = tMapBase.getRotation();
        odom.pose.pose.orientation.x = qBase.x();
        odom.pose.pose.orientation.y = qBase.y();
        odom.pose.pose.orientation.z = qBase.z();
        odom.pose.pose.orientation.w = qBase.w();

        // twist 按 ROS 约定在 child_frame_id（base_link）系表达。设备给的
        // vel/gyro 在设备系，描述的是雷达那一点；base_link 是同一刚体上的另一
        // 点，线速度必须补 ω × r 这一项 —— 如 A2 安装外参平移 0.344 m，1 rad/s 偏航就会
        // 带来 0.34 m/s 的误差，不是可以忽略的量级。
        const tf2::Vector3 vImu(pose.velX, pose.velY, pose.velZ);
        const tf2::Vector3 wImu(pose.gyroX, pose.gyroY, pose.gyroZ);
        const tf2::Matrix3x3 rBaseImu(tfImuBase_.getRotation().inverse());
        const tf2::Vector3 vBase =
            rBaseImu * (vImu + wImu.cross(tfImuBase_.getOrigin()));
        const tf2::Vector3 wBase = rBaseImu * wImu;
        odom.twist.twist.linear.x = vBase.x();
        odom.twist.twist.linear.y = vBase.y();
        odom.twist.twist.linear.z = vBase.z();
        odom.twist.twist.angular.x = wBase.x();
        odom.twist.twist.angular.y = wBase.y();
        odom.twist.twist.angular.z = wBase.z();
        odomPub_->publish(odom);

        // 动态 TF 仍发 map -> imu_link，用 SDK 原值；与静态 TF
        // imu_link -> base_link 串起来就是完整的 map -> base_link。
        // 这样安装外参只在静态 TF 一处生效，不会被写进两个地方各算一遍。
        auto tfMsg = tf2_msgs::msg::TFMessage();
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = odomFrame_;
        tf.child_frame_id = imuFrame_;
        tf.transform.translation.x = pose.posX;
        tf.transform.translation.y = pose.posY;
        tf.transform.translation.z = pose.posZ;
        tf.transform.rotation.w = pose.quatW;
        tf.transform.rotation.x = pose.quatX;
        tf.transform.rotation.y = pose.quatY;
        tf.transform.rotation.z = pose.quatZ;
        tfMsg.transforms.push_back(tf);
        tfPub_->publish(tfMsg);
    }
    // ==================== 点云回调 ====================
    // ==================== IMU 回调 ====================
    void onImu(const xg::Device::Imu& imu) {
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->now();
        msg.header.frame_id = sensorFrame_;

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

    // ==================== 去畸变点云回调 ====================
    // 来自 device_undist_cloud（Locate 去畸变点云），坐标系是设备（IMU）系，
    // 标 imu_link（lidar_frame 参数），经静态 TF 与 base_link 相连。
    void onDeviceUndistCloud(const xg::CloudFrame& frame) {
        auto msg = sensor_msgs::msg::PointCloud2();
        msg.header.stamp = this->now();
        msg.header.frame_id = imuFrame_;

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

        undistCloudPub_->publish(msg);
    }

    // ==================== 图像回调 ====================
    void onImage(const xg::Device::ImageFrame& frame) {
        auto msg = sensor_msgs::msg::CompressedImage();
        msg.header.stamp = this->now();
        msg.header.frame_id = sensorFrame_;
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

    // 所有话题统一用 ROS 时钟（this->now()），不再使用设备时间戳。
    // 实测设备墙钟与机器人系统时间相差约 1016 天（设备 2023-11-29，
    // 机器人 2026-09-10），两者混用会让下游按消息时间查 TF 时差出数年。
    // 原先把设备 timestampSec 转成 ROS Time 的 fillTimestamp 已无调用者，
    // 一并移除，避免留下死代码被误用。

private:
    // SDK 设备句柄
    std::unique_ptr<xg::Device> device_;

    // ROS2 Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tfPub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTfBroadcaster_;

    // imu_link -> base_link 的安装外参，构造时从参数缓存，onPose 每帧复用
    tf2::Transform tfImuBase_{tf2::Transform::getIdentity()};
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr undistCloudPub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr imageLeftPub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr imageCenterPub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr imageRightPub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagPub_;

    // 坐标系参数
    std::string odomFrame_;
    std::string baseFrame_;
    std::string sensorFrame_;
    std::string imuFrame_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SdkBridgeNode>();
    if (!node->connected()) {
        // 连接失败: 已在构造函数里打印原因, 直接退出, 不进入 spin
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
