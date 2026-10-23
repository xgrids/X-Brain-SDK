/******************************************************************************
This file is part of XGRIDS Robot SDK.
Project: xg_robot_sdk
Author : XGRIDS Embedded Software Team
*******************************************************************************
History:
1. create this file. (DATE: 20260804)
******************************************************************************/

/**
 * @file   robot_sdk.h
 * @brief  XGRIDS 设备录制控制 SDK —— C++ 封装接口
 *
 * 本头文件是 header-only 的薄封装，全部实现均为内联，由集成方自行编译。
 * 这样做的目的是规避 C++ ABI 不兼容问题：集成方的编译器版本、
 * libstdc++ 版本、_GLIBCXX_USE_CXX11_ABI 取值若与 SDK 构建环境不一致，
 * std::string / std::unique_ptr 等类型跨越动态库边界会导致链接失败或
 * 运行期崩溃。本封装只调用 robot_sdk_c.h 中的 C ABI，故无此风险。
 *
 * @note 设计约束（修改本文件前务必阅读）：
 *   1. 只允许使用 C++ 标准库与 robot_sdk_c.h。不得引入 appkit/basetype.h
 *      等项目内部头文件，集成方环境中不存在这些头文件。
 *   2. 同上，不得使用 DECL_CLASSMETA / DECL_PROPERTY / DECL_UNCOPYABLE
 *      等项目宏，相关语义在本文件中手工展开。
 *   3. 所有函数必须是 inline，本文件不允许产生需要单独编译的符号。
 *   4. 不抛异常，一律以返回值传递错误。
 *
 * @note 线程安全性：同一 Device 实例的方法调用不可重入，多线程使用时
 *   须由调用方加锁；不同实例之间互不影响。
 *
 * @note 启停接口只下发命令、不等待状态变化，结果须经 queryRecordStatus()
 *   轮询确认。设备启动录制的耗时与地图规模等因素相关，无法预设固定上限，
 *   故不由 SDK 代为等待 —— 由调用方按自身业务需要决定等多久、如何反馈进度。
 *
 * @code 用法
 *     xg::Options opts;
 *     opts.deviceIp = "192.168.123.103";
 *
 *     auto device = xg::Device::open(opts);
 *     if (!device) {
 *         return -1;   // 连接失败
 *     }
 *
 *     // 下发命令, 立即返回
 *     if (device->startRecord() != xg::Error::Ok) {
 *         return -1;
 *     }
 *
 *     // 轮询确认是否真正开始
 *     xg::RecordStatus status;
 *     while (true) {
 *         std::this_thread::sleep_for(std::chrono::milliseconds(500));
 *         if (device->queryRecordStatus(&status) != xg::Error::Ok) {
 *             continue;   // 查询失败可重试, 不代表录制失败
 *         }
 *         if (status.state == xg::RecordState::Recording) {
 *             break;      // 录制已开始
 *         }
 *         if (status.state == xg::RecordState::Error) {
 *             std::cerr << "启动失败: " << status.message << std::endl;
 *             return -1;
 *         }
 *         // Idle / Starting 均表示仍在准备, 继续等待
 *     }
 *
 *     device->stopRecord();
 * @endcode
 */

#pragma once

#include "xg/robot_sdk_c.h"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace xg {

/**
 * @brief 接口返回值
 * @note  与 xg_err_t 一一对应。
 */
enum class Error {
    Ok = XG_OK,                         /**< 成功 */
    InvalidArg = XG_ERR_INVALID_ARG,    /**< 参数非法 */
    NotConnected = XG_ERR_NOT_CONNECTED,/**< 未连接到设备 */
    Timeout = XG_ERR_TIMEOUT,           /**< 超时 */
    Rejected = XG_ERR_REJECTED,         /**< 设备拒绝执行，当前状态不允许该操作 */
    DeviceError = XG_ERR_DEVICE_ERROR,  /**< 设备处于故障状态 */
    Protocol = XG_ERR_PROTOCOL,         /**< 设备应答无法解析 */
    Internal = XG_ERR_INTERNAL,         /**< SDK 内部错误 */
};

/**
 * @brief 录制状态
 * @note  与 xg_record_state_t 一一对应。
 */
enum class RecordState {
    Idle = XG_REC_IDLE,           /**< 待机，可以开始录制 */
    Starting = XG_REC_STARTING,   /**< 启动中，尚未开始出数据 */
    Recording = XG_REC_RECORDING, /**< 录制中 */
    Stopping = XG_REC_STOPPING,   /**< 停止中 */
    Error = XG_REC_ERROR,         /**< 故障，详见 message */
};

/**
 * @brief 录制状态详情
 */
struct RecordStatus {
    RecordState state{RecordState::Idle}; /**< 归并后的对外状态 */

    /**
     * 告警标志。为 true 时表示存在需要关注的异常，但不影响当前 state 所
     * 允许的操作。典型情形：
     *   - state 为 Recording：建图过程异常、已退化为仅录包模式，
     *     录制仍在继续，可正常停止；
     *   - state 为 Idle：上一次工程的后处理存在异常，设备本身可用。
     * 与 RecordState::Error 的区别是：告警不阻塞操作，故障状态则阻塞。
     */
    bool warning{false};

    /**
     * 设备原始状态码，仅供排障与日志记录。
     * 该值反映设备内部状态机，不保证跨版本稳定，不应作为业务判断依据。
     */
    std::int32_t rawCode{0};

    std::string message{""}; /**< 人可读说明，UTF-8 */
};

/**
 * @brief 连接参数
 */
struct Options {
    /** 设备 IP，必填。当前要求与设备同一交换机直连 */
    std::string deviceIp{""};

    /** modeling_app 端口，0 表示使用默认端口 7448 */
    std::uint16_t port{0};

    /** 连接超时（毫秒），0 表示使用默认值 */
    std::uint32_t connectTimeoutMs{0};
};

/**
 * @brief  错误码转可读字符串
 * @param  err 错误码
 * @return 静态字符串
 */
inline const char* toString(Error err) {
    return xg_err_str(static_cast<xg_err_t>(err));
}

/**
 * @brief  录制状态转可读字符串
 * @param  state 录制状态
 * @return 静态字符串
 */
inline const char* toString(RecordState state) {
    return xg_record_state_str(static_cast<xg_record_state_t>(state));
}

/**
 * @brief  取 SDK 版本号（运行时库的版本）
 * @return "MAJOR.MINOR.PATCH" 形式的字符串，详见 xg_sdk_version() 与
 *         XG_SDK_VERSION_STRING
 */
inline std::string sdkVersion() {
    return std::string(xg_sdk_version());
}

/** 点云帧（C++ 视角） */
struct CloudFrame {
    const xg_point_t* points{nullptr}; /**< 点数组，回调期间有效 */
    std::size_t count{0};              /**< 点数量 */
    double timestampSec{0.0};          /**< 时间戳（秒） */
    std::uint32_t seq{0};              /**< SDK 本地帧序号（自订阅起递增），不是设备帧号 */
};

/** 原始点云帧（C++ 视角，设备(IMU)系，未去畸变，未降采样） */
struct RawCloudFrame {
    const xg_raw_point_t* points{nullptr}; /**< 点数组，回调期间有效 */
    std::size_t count{0};                  /**< 点数量 */
    double timestampSec{0.0};              /**< 帧内最早点的采样时刻（雷达自身时钟的 Unix epoch 秒，不保证等于设备系统时钟） */
    std::uint32_t seq{0};                  /**< 设备端雷达 frame_id */
};

/** 位姿（C++ 视角） */
struct Pose {
    double timestampSec{0.0};
    float posX{0}, posY{0}, posZ{0};
    float quatW{1}, quatX{0}, quatY{0}, quatZ{0};
    float velX{0}, velY{0}, velZ{0};
    float gyroX{0}, gyroY{0}, gyroZ{0};
    std::uint32_t errorCode{0};
    /** 保留字段：1.0 暂不提供，恒为 false，勿使用（proto 对齐后启用） */
    bool isLocal{false};
};

/**
 * @brief 设备连接
 * @note  通过 open() 创建，析构时自动断开连接并释放资源。
 *        禁止拷贝与移动，避免句柄被重复释放。
 */
class Device {
public:
    /**
     * @brief  连接设备
     * @param  opts  连接参数，deviceIp 不可为空
     * @param  error 输出错误码，可为空
     * @return 成功返回实例；失败返回 nullptr，原因见 error
     */
    static std::unique_ptr<Device> open(const Options& opts,
                                        Error* error = nullptr) {
        xg_options_t cOpts;
        std::memset(&cOpts, 0, sizeof(cOpts));
        cOpts.struct_size = sizeof(cOpts);
        cOpts.port = opts.port;
        cOpts.connect_timeout_ms = opts.connectTimeoutMs;

        // IP 超长视为参数错误，而非静默截断后连到错误的地址
        if (opts.deviceIp.empty() || opts.deviceIp.size() >= XG_IP_MAX) {
            if (error != nullptr) {
                *error = Error::InvalidArg;
            }
            return nullptr;
        }
        std::memcpy(cOpts.device_ip, opts.deviceIp.c_str(),
                    opts.deviceIp.size() + 1);

        xg_device_t* raw = nullptr;
        auto rc = xg_device_open(&cOpts, &raw);
        if (rc != XG_OK || raw == nullptr) {
            if (error != nullptr) {
                *error = static_cast<Error>(rc);
            }
            return nullptr;
        }
        if (error != nullptr) {
            *error = Error::Ok;
        }
        return std::unique_ptr<Device>(new Device(raw));
    }

    ~Device() {
        xg_unsubscribe_pose(m_handle);
        xg_unsubscribe_imu(m_handle);
        xg_unsubscribe_device_undist_cloud(m_handle);
        xg_unsubscribe_image(m_handle, XG_CAMERA_LEFT);
        xg_unsubscribe_image(m_handle, XG_CAMERA_CENTER);
        xg_unsubscribe_image(m_handle, XG_CAMERA_RIGHT);
        xg_unsubscribe_err_code(m_handle);
        xg_device_close(m_handle);
        m_handle = nullptr;
    }

    Device(const Device&) = delete;
    Device& operator=(const Device&) = delete;
    Device(Device&&) = delete;
    Device& operator=(Device&&) = delete;

    /**
     * @brief  连接是否正常
     * @return 已连接返回 true
     */
    bool isConnected() const {
        return xg_device_is_connected(m_handle) != 0;
    }

    /**
     * @brief  下发开始录制命令
     * @return Error::Ok 设备已接受命令，或设备本就在录制中（幂等）
     *
     * @note   **本接口不等待录制真正开始。** 返回 Error::Ok 仅表示设备已接受
     *         命令。设备需初始化定位与建图后才进入录制状态，耗时与地图规模等
     *         因素相关，从数秒到数十秒不等，无法预设固定上限。结果须由调用方
     *         轮询 queryRecordStatus() 确认，等到 RecordState::Recording
     *         方可认为录制已开始。
     *
     * @note   返回 Error::Timeout 时命令可能已送达设备。不要直接重试，
     *         应先 queryRecordStatus() 确认，否则会重复下发。
     */
    Error startRecord() {
        return static_cast<Error>(xg_record_start(m_handle));
    }

    /**
     * @brief  下发停止录制命令
     * @return Error::Ok 设备已接受命令，或设备本就未在录制（幂等）
     * @note   同样不等待状态变化，需轮询 queryRecordStatus() 直至
     *         RecordState::Idle。停止涉及数据落盘，耗时与数据量相关。
     * @note   处于告警状态（warning 为 true）时同样可以正常停止。
     */
    Error stopRecord() {
        return static_cast<Error>(xg_record_stop(m_handle));
    }

    /**
     * @brief  查询当前录制状态
     * @param  status 输出录制状态，不可为空
     * @return Error::Ok 表示查询成功
     * @note   本接口不改变设备状态，可安全轮询。这是确认启停结果的唯一途径，
     *         建议间隔不小于 XG_RECOMMENDED_POLL_INTERVAL_MS。
     */
    Error queryRecordStatus(RecordStatus* status) {
        if (status == nullptr) {
            return Error::InvalidArg;
        }
        xg_record_status_t cStatus;
        initStatus(&cStatus);
        auto rc = xg_record_query(m_handle, &cStatus);
        fromCStatus(cStatus, status);
        return static_cast<Error>(rc);
    }


    /**
     * @brief  订阅位姿数据
     * @param  callback 回调，在 SDK 内部线程触发，须尽快返回
     * @return Error::Ok 成功；Error::Rejected 已有活跃订阅
     */
    Error subscribePose(std::function<void(const Pose&)> callback) {
        m_poseCb = std::move(callback);
        return static_cast<Error>(
            xg_subscribe_pose(m_handle, &Device::poseCbBridge, this));
    }

    /** 取消位姿订阅 */
    Error unsubscribePose() {
        auto rc = static_cast<Error>(xg_unsubscribe_pose(m_handle));
        m_poseCb = nullptr;
        return rc;
    }

    /** IMU 数据（C++ 视角） */
    struct Imu {
        double timestampSec{0.0};
        float accX{0}, accY{0}, accZ{0};
        float gyrX{0}, gyrY{0}, gyrZ{0};
    };

    /**
     * @brief  订阅 IMU 数据
     * @param  callback 回调，在 SDK 内部线程触发，须尽快返回
     * @return Error::Ok 成功；Error::Rejected 已有活跃订阅
     */
    Error subscribeImu(std::function<void(const Imu&)> callback) {
        m_imuCb = std::move(callback);
        return static_cast<Error>(
            xg_subscribe_imu(m_handle, &Device::imuCbBridge, this));
    }

    /** 取消 IMU 订阅 */
    Error unsubscribeImu() {
        auto rc = static_cast<Error>(xg_unsubscribe_imu(m_handle));
        m_imuCb = nullptr;
        return rc;
    }

    /**
     * @brief  订阅设备(IMU)系 Locate 去畸变点云（key: device_undist_cloud）
     * @param  callback 回调，在 SDK 内部线程触发，须尽快返回
     * @return Error::Ok 成功；Error::Rejected 已有活跃订阅；
     *         Error::Internal 订阅声明失败
     * @note   数据语义、线程与内存规则见 xg_subscribe_device_undist_cloud。
     *         CloudFrame::points 仅回调期间有效。CloudFrame::seq 是 SDK 本地计数，
     *         点云丢帧用相邻帧 timestampSec 的间隔判断。
     */
    Error subscribeDeviceUndistCloud(
        std::function<void(const CloudFrame&)> callback) {
        m_deviceUndistCloudCb = std::move(callback);
        return static_cast<Error>(xg_subscribe_device_undist_cloud(
            m_handle, &Device::deviceUndistCloudCbBridge, this));
    }

    /** 取消设备(IMU)系 Locate 去畸变点云订阅（幂等） */
    Error unsubscribeDeviceUndistCloud() {
        auto rc =
            static_cast<Error>(xg_unsubscribe_device_undist_cloud(m_handle));
        m_deviceUndistCloudCb = nullptr;
        return rc;
    }

    /**
     * @brief  订阅设备(IMU)系原始雷达点云（key: device_raw_cloud）
     * @param  callback 回调，在 Zenoh 会话接收线程触发，须尽快返回
     * @return Error::Ok 成功；Error::InvalidArg 回调为空；
     *         Error::Rejected 已有活跃订阅；Error::Internal 订阅声明失败
     * @note   数据语义（RoboSense 全量点、未去畸变、IMU 系、逐点时间偏移）、
     *         无数据的情况（含设备未处于扫描状态）、
     *         线程与内存规则见 xg_subscribe_device_raw_cloud。
     *         RawCloudFrame::points 仅回调期间有效。
     */
    Error subscribeDeviceRawCloud(
        std::function<void(const RawCloudFrame&)> callback) {
        if (!callback) {
            return Error::InvalidArg;
        }
        // 已订阅时不覆盖回调：接收线程可能正在读取旧回调
        if (m_deviceRawCloudCb) {
            return Error::Rejected;
        }
        m_deviceRawCloudCb = std::move(callback);
        auto rc = static_cast<Error>(xg_subscribe_device_raw_cloud(
            m_handle, &Device::deviceRawCloudCbBridge, this));
        if (rc != Error::Ok) {
            m_deviceRawCloudCb = nullptr;
        }
        return rc;
    }

    /** 取消设备(IMU)系原始点云订阅（幂等） */
    Error unsubscribeDeviceRawCloud() {
        auto rc = static_cast<Error>(xg_unsubscribe_device_raw_cloud(m_handle));
        m_deviceRawCloudCb = nullptr;
        return rc;
    }

    /** 相机标识 */
    enum class CameraId {
        Left = XG_CAMERA_LEFT,
        Center = XG_CAMERA_CENTER,
        Right = XG_CAMERA_RIGHT,
    };

    /** 图像帧（C++ 视角） */
    struct ImageFrame {
        CameraId cameraId{CameraId::Left};
        double timestampSec{0.0};
        const uint8_t* jpegData{nullptr}; /**< JPEG 裸数据，回调期间有效 */
        std::size_t jpegSize{0};
        std::uint32_t seq{0};
    };

    /**
     * @brief  订阅相机图像
     * @param  cameraId 相机标识
     * @param  callback 回调，在 SDK 内部线程触发，须尽快返回
     * @return Error::Ok 成功；Error::Rejected 该相机已有活跃订阅
     * @note   回调频率取决于设备端 JPEG 编码配置（帧率以设备配置为准，实测 10 Hz）。
     */
    Error subscribeImage(CameraId cameraId,
                         std::function<void(const ImageFrame&)> callback) {
        int idx = static_cast<int>(cameraId);
        if (idx < 0 || idx > 2) {
            return Error::InvalidArg;
        }
        m_imageCb[idx] = std::move(callback);
        return static_cast<Error>(
            xg_subscribe_image(m_handle,
                               static_cast<xg_camera_id_t>(cameraId),
                               &Device::imageCbBridge, this));
    }

    /** 取消相机图像订阅 */
    Error unsubscribeImage(CameraId cameraId) {
        int idx = static_cast<int>(cameraId);
        if (idx < 0 || idx > 2) {
            return Error::InvalidArg;
        }
        auto rc = static_cast<Error>(
            xg_unsubscribe_image(m_handle,
                                 static_cast<xg_camera_id_t>(cameraId)));
        m_imageCb[idx] = nullptr;
        return rc;
    }

    /** 深度图帧（C++ 视角，AI 推理产出的度量深度，单位米） */
    struct DepthFrame {
        CameraId cameraId{CameraId::Left};
        double timestampSec{0.0};
        uint32_t width{0};
        uint32_t height{0};
        const float* depthData{nullptr}; /**< 行优先深度数组(米), 回调期间有效 */
        std::size_t depthSize{0};        /**< depthData 字节数 */
        std::uint32_t seq{0};
    };

    /**
     * @brief  订阅相机深度图（AI 推理产出，仅中间相机有效）
     * @param  cameraId 相机标识
     * @param  callback 回调，在 SDK 内部线程触发，须尽快返回
     * @return Error::Ok 成功；Error::Rejected 该相机已有活跃订阅
     * @note   回调中 depthData 指针仅在回调执行期间有效。
     *         设备端默认配置 lixel_t1xb.camera_center.ai.fps 为 0，此时设备不发布
     *         depth/center，订阅仍返回 Ok；详见 xg_subscribe_depth。
     */
    Error subscribeDepth(CameraId cameraId,
                        std::function<void(const DepthFrame&)> callback) {
        int idx = static_cast<int>(cameraId);
        if (idx < 0 || idx > 2) {
            return Error::InvalidArg;
        }
        m_depthCb[idx] = std::move(callback);
        return static_cast<Error>(
            xg_subscribe_depth(m_handle, static_cast<xg_camera_id_t>(cameraId),
                               &Device::depthCbBridge, this));
    }

    /** 取消相机深度图订阅 */
    Error unsubscribeDepth(CameraId cameraId) {
        int idx = static_cast<int>(cameraId);
        if (idx < 0 || idx > 2) {
            return Error::InvalidArg;
        }
        auto rc = static_cast<Error>(xg_unsubscribe_depth(
            m_handle, static_cast<xg_camera_id_t>(cameraId)));
        m_depthCb[idx] = nullptr;
        return rc;
    }

    // ================================================================

    // ================================================================
    // 传感器标定参数查询（按传感器拆分）
    // ================================================================

    /** 相机标定参数 */
    struct CameraCalib {
        uint32_t width{0}, height{0};
        float fx{0}, fy{0}, cx{0}, cy{0};
        float distortion[4]{0};
        std::string model;
        float pose[16]{0}; /**< 相对 cam0 的 4x4 变换 */
        bool calibrated{false};
    };

    /**
     * @brief  查询指定相机的标定参数
     * @param  cameraIndex 相机原始索引（0~3），对应设备标定文件中的 camera_N
     * @param  out 输出标定参数
     * @return Error::Ok 成功
     * @note   此索引为设备标定文件中的原始编号，与 subscribeImage 的 CameraId
     *         不是同一套映射。具体哪个索引对应哪个物理相机取决于设备型号。
     */
    Error getCameraCalib(int cameraIndex, CameraCalib* out) {
        if (out == nullptr) { return Error::InvalidArg; }
        xg_camera_calib_t cOut;
        std::memset(&cOut, 0, sizeof(cOut));
        cOut.struct_size = sizeof(cOut);
        auto rc = static_cast<Error>(xg_get_camera_calib(
            m_handle, cameraIndex, &cOut));
        if (rc != Error::Ok) { return rc; }
        out->width = cOut.width; out->height = cOut.height;
        out->fx = cOut.fx; out->fy = cOut.fy;
        out->cx = cOut.cx; out->cy = cOut.cy;
        std::memcpy(out->distortion, cOut.distortion, sizeof(out->distortion));
        out->model = cOut.model;
        std::memcpy(out->pose, cOut.pose, sizeof(out->pose));
        out->calibrated = (cOut.calibrated != 0);
        return Error::Ok;
    }

    /** IMU 标定参数 */
    struct ImuCalib {
        float accelBias[3]{0};
        float gyroBias[3]{0};
        float accelTransform[9]{0};
        float accelScale[9]{0};
        float gyroTransform[9]{0};
        float gyroScale[9]{0};
        /**
         * T_imu_lidar：Lidar→IMU 的 4x4 齐次变换矩阵（行优先）。
         * 从右往左读：将 Lidar 坐标系下的点变换到 IMU 坐标系。
         */
        float extrinsicImuLidar[16]{0};
        bool calibrated{false};
    };

    /** 查询 IMU 标定参数 */
    Error getImuCalib(ImuCalib* out) {
        if (out == nullptr) { return Error::InvalidArg; }
        xg_imu_calib_t cOut;
        std::memset(&cOut, 0, sizeof(cOut));
        cOut.struct_size = sizeof(cOut);
        auto rc = static_cast<Error>(xg_get_imu_calib(m_handle, &cOut));
        if (rc != Error::Ok) { return rc; }
        std::memcpy(out->accelBias, cOut.accel_bias, sizeof(out->accelBias));
        std::memcpy(out->gyroBias, cOut.gyro_bias, sizeof(out->gyroBias));
        std::memcpy(out->accelTransform, cOut.accel_transform, sizeof(out->accelTransform));
        std::memcpy(out->accelScale, cOut.accel_scale, sizeof(out->accelScale));
        std::memcpy(out->gyroTransform, cOut.gyro_transform, sizeof(out->gyroTransform));
        std::memcpy(out->gyroScale, cOut.gyro_scale, sizeof(out->gyroScale));
        std::memcpy(out->extrinsicImuLidar, cOut.extrinsic_imu_lidar, sizeof(out->extrinsicImuLidar));
        out->calibrated = (cOut.calibrated != 0);
        return Error::Ok;
    }

    /** 雷达标定参数 */
    struct LidarCalib {
        /**
         * T_imu_lidar：Lidar→IMU 的 4x4 齐次变换矩阵（行优先）。
         * 从右往左读：将 Lidar 坐标系下的点变换到 IMU 坐标系。
         */
        float extrinsicImuLidar[16]{0};
        /**
         * T_cam0_lidar：Lidar→Cam0 的 4x4 齐次变换矩阵（行优先）。
         * 从右往左读：将 Lidar 坐标系下的点变换到 Cam0 坐标系。
         */
        float extrinsicCameraLidar[16]{0};
        std::string lidarType;
        bool calibrated{false};
    };

    /** 查询雷达标定参数 */
    Error getLidarCalib(LidarCalib* out) {
        if (out == nullptr) { return Error::InvalidArg; }
        xg_lidar_calib_t cOut;
        std::memset(&cOut, 0, sizeof(cOut));
        cOut.struct_size = sizeof(cOut);
        auto rc = static_cast<Error>(xg_get_lidar_calib(m_handle, &cOut));
        if (rc != Error::Ok) { return rc; }
        std::memcpy(out->extrinsicImuLidar, cOut.extrinsic_imu_lidar, sizeof(out->extrinsicImuLidar));
        std::memcpy(out->extrinsicCameraLidar, cOut.extrinsic_camera_lidar, sizeof(out->extrinsicCameraLidar));
        out->lidarType = cOut.lidar_type;
        out->calibrated = (cOut.calibrated != 0);
        return Error::Ok;
    }


    // ================================================================
    // 设备状态查询
    // ================================================================

    /** 设备状态 */
    struct DeviceInfo {
        float batteryLevel{0};    /**< 电池电量（%） */
        float batteryTemp{0};     /**< 电池温度（℃） */
        float batteryVoltage{0};  /**< 电池电压（V） */
        std::int64_t diskTotalKb{0}; /**< 存储总量（KB） */
        std::int64_t diskUsedKb{0};  /**< 存储已用（KB） */
        std::int64_t memTotalKb{0};  /**< 内存总量（KB） */
        std::int64_t memUsedKb{0};   /**< 内存已用（KB） */
        std::uint8_t rtkState{0};    /**< RTK 状态 */
        std::uint8_t satelliteNum{0};/**< 卫星数 */
        std::int32_t modelingState{0}; /**< 建图状态码 */
    };

    /**
     * @brief  查询设备状态
     * @param  out 输出设备状态，不可为空
     * @return Error::Ok 成功
     */
    Error getDeviceInfo(DeviceInfo* out) {
        if (out == nullptr) { return Error::InvalidArg; }
        xg_device_info_t cOut;
        std::memset(&cOut, 0, sizeof(cOut));
        cOut.struct_size = sizeof(cOut);
        auto rc = static_cast<Error>(xg_get_device_info(m_handle, &cOut));
        if (rc != Error::Ok) { return rc; }
        out->batteryLevel = cOut.battery_level;
        out->batteryTemp = cOut.battery_temp;
        out->batteryVoltage = cOut.battery_voltage;
        out->diskTotalKb = cOut.disk_total_kb;
        out->diskUsedKb = cOut.disk_used_kb;
        out->memTotalKb = cOut.mem_total_kb;
        out->memUsedKb = cOut.mem_used_kb;
        out->rtkState = cOut.rtk_state;
        out->satelliteNum = cOut.satellite_num;
        out->modelingState = cOut.modeling_state;
        return Error::Ok;
    }

    // ================================================================
    // 设备时间查询（只读）
    // ================================================================
    /**
     * 设备系统时钟快照及本机往返时刻。各字段含义、量纲与偏差区间见
     * xg_device_time_t 与 api_reference_v2.md §0.8。
     */
    struct DeviceTime {
        std::int64_t deviceEpochUs{0};   /**< 设备系统时钟，Unix epoch 微秒 */
        std::int32_t utcOffsetSec{0};    /**< 设备本地时区相对 UTC 的偏移（秒，含夏令时），仅用于显示 */
        std::int64_t hostSendEpochUs{0}; /**< 本机发出查询时的 system_clock，Unix epoch 微秒 */
        std::int64_t hostRecvEpochUs{0}; /**< 本机收到应答时的 system_clock，Unix epoch 微秒 */
        std::int64_t rttUs{0};           /**< 本机 steady_clock 测得的往返时延（微秒） */

        /** 偏差下界：offset = 设备时钟 - 本机时钟 >= 该值 */
        std::int64_t offsetLowerUs() const { return deviceEpochUs - hostRecvEpochUs; }
        /** 偏差上界：offset = 设备时钟 - 本机时钟 <= 该值 */
        std::int64_t offsetUpperUs() const { return deviceEpochUs - hostSendEpochUs; }
        /** 区间中点，仅作粗略估计；真实偏差只保证落在 [下界, 上界] 内 */
        std::int64_t offsetMidUs() const { return (offsetLowerUs() + offsetUpperUs()) / 2; }
    };

    /**
     * @brief  查询设备系统时间（只读）
     * @param  out 输出，不可为空
     * @return Error::Ok 成功；Error::InvalidArg out 为空；Error::Timeout 未应答；
     *         Error::Protocol 应答无法解析，或设备固件不支持本查询
     * @note   语义、偏差区间与禁止用法见 xg_get_device_time。
     */
    Error getDeviceTime(DeviceTime* out) {
        if (out == nullptr) { return Error::InvalidArg; }
        xg_device_time_t cOut;
        std::memset(&cOut, 0, sizeof(cOut));
        cOut.struct_size = sizeof(cOut);
        auto rc = static_cast<Error>(xg_get_device_time(m_handle, &cOut));
        if (rc != Error::Ok) {
            *out = DeviceTime();  // 失败时清零，与 C 接口的承诺一致
            return rc;
        }
        out->deviceEpochUs = cOut.device_epoch_us;
        out->utcOffsetSec = cOut.utc_offset_s;
        out->hostSendEpochUs = cOut.host_send_epoch_us;
        out->hostRecvEpochUs = cOut.host_recv_epoch_us;
        out->rttUs = cOut.rtt_us;
        return Error::Ok;
    }

    // ================================================================
    // 定位初始位姿设置
    // ================================================================

    /** 定位初始位姿参数 */
    struct InitPose {
        double timestampSec{0.0};
        float posX{0}, posY{0}, posZ{0};
        float quatX{0}, quatY{0}, quatZ{0}, quatW{1};
    };

    /**
     * @brief  设置定位初始位姿，并启动设备端 Locate 算法
     * @param  pose 初始位姿参数
     * @return Error::Ok 设备已收下初值且 Locate 算法已启动；
     *         Error::Rejected 设备拒绝（如四元数模长不合法、定位尚未初始化）
     * @note   设备端 Locate 的时序是：录制启动时完成初始化 -> 本接口喂入初值并启动算法。
     *         设备进入 Recording 时初始化已完成。本接口是启动 Locate 的必经步骤：
     *         调用前设备不发布位姿（subscribePose）和 Locate 去畸变点云
     *         （subscribeDeviceUndistCloud）。Ok 不代表重定位完成。
     */
    Error setLocInitPose(const InitPose& pose) {
        xg_init_pose_t cPose;
        std::memset(&cPose, 0, sizeof(cPose));
        cPose.struct_size = sizeof(cPose);
        cPose.timestamp_sec = pose.timestampSec;
        cPose.pos_x = pose.posX;
        cPose.pos_y = pose.posY;
        cPose.pos_z = pose.posZ;
        cPose.quat_x = pose.quatX;
        cPose.quat_y = pose.quatY;
        cPose.quat_z = pose.quatZ;
        cPose.quat_w = pose.quatW;
        return static_cast<Error>(xg_set_loc_init_pose(m_handle, &cPose));
    }

    // ================================================================
    // 工程数据路径查询
    // ================================================================

    /**
     * @brief  查询当前工程的录制数据存储路径
     * @param  path 输出路径字符串
     * @return Error::Ok 成功
     */
    Error getDataPath(std::string* path) {
        if (path == nullptr) { return Error::InvalidArg; }
        xg_data_path_t cOut;
        std::memset(&cOut, 0, sizeof(cOut));
        cOut.struct_size = sizeof(cOut);
        auto rc = static_cast<Error>(xg_get_data_path(m_handle, &cOut));
        if (rc != Error::Ok) { return rc; }
        *path = cOut.path;
        return Error::Ok;
    }

    // ================================================================
    // 设备网络配置
    // ================================================================

    /**
     * @brief  修改设备对外网口的 IP 与网段，需随后 confirmNetwork，否则自动回滚
     * @param  gateway         nullptr 保持现有网关，"" 删除默认路由，其他为新网关
     * @param  confirmTimeoutS 确认窗口（秒），0 为默认值，否则范围 [10, 120]
     * @return Error::Ok 设备已接受（不代表已生效）；其余见 xg_set_device_network
     * @note   成功后只应销毁本 Device，确认请使用重新打开的 Device。
     */
    Error setNetwork(const std::string& ip, int prefixLen,
                     const char* gateway = nullptr, int confirmTimeoutS = 0) {
        return static_cast<Error>(xg_set_device_network(
            m_handle, ip.c_str(), prefixLen, gateway, confirmTimeoutS));
    }

    /** 确认网络配置变更，取消自动回滚（幂等） */
    Error confirmNetwork() {
        return static_cast<Error>(xg_confirm_device_network(m_handle));
    }

    // ================================================================
    // 错误码订阅
    // ================================================================

    /** 单个错误码条目 */
    struct ErrCodeEntry {
        std::uint32_t code{0};
        std::uint64_t timestamp{0}; /**< 微秒 */
    };

    /** 错误码帧 */
    struct ErrCodeFrame {
        std::vector<ErrCodeEntry> entries;
    };

    /**
     * @brief  订阅错误码数据
     * @param  callback 回调，在 SDK 内部线程触发，须尽快返回
     * @return Error::Ok 成功；Error::Rejected 已有活跃订阅
     * @note   设备以 ~10Hz 批量推送错误码（含定位、建图、SLAM 等模块）。
     */
    Error subscribeErrCode(std::function<void(const ErrCodeFrame&)> callback) {
        m_errCodeCb = std::move(callback);
        return static_cast<Error>(
            xg_subscribe_err_code(m_handle, &Device::errCodeCbBridge, this));
    }

    /** 取消错误码订阅 */
    Error unsubscribeErrCode() {
        auto rc = static_cast<Error>(xg_unsubscribe_err_code(m_handle));
        m_errCodeCb = nullptr;
        return rc;
    }

    // ================================================================
    // 重定位地图管理
    // ================================================================

    /** 地图上传选项（C++ 视角） */
    struct MapUploadOptions {
        std::string localMapDir;   /**< 本机地图数据根目录 */
        std::string mapId;         /**< 地图标识 */
        std::uint32_t chunkSize{0}; /**< 0 = SDK 默认（512KB） */
        bool useGzip{false};       /**< 默认不压缩 */
    };

    /** 单个地图文件条目 */
    struct MapFileEntry {
        std::string path;   /**< 相对 <mapId>/ 的路径 */
        std::int64_t size{0};
        std::int64_t mtime{0};
    };

    /** 单个地图信息 */
    struct MapInfo {
        std::string mapId;
        bool isActive{false};
        std::int64_t totalSize{0};
        std::vector<MapFileEntry> files;
    };

    /**
     * @brief  上传一整套重定位地图（阻塞至完成或失败）
     * @param  opts       上传选项
     * @param  progressCb 进度回调 [0,1]，可空
     * @return Error::Ok 成功就绪；其余见 xg_upload_map 返回码
     */
    Error uploadMap(const MapUploadOptions& opts,
                    std::function<void(float)> progressCb = nullptr) {
        m_mapProgressCb = std::move(progressCb);
        xg_map_upload_opts_t cOpts;
        std::memset(&cOpts, 0, sizeof(cOpts));
        cOpts.struct_size = sizeof(cOpts);
        cOpts.local_map_dir = opts.localMapDir.c_str();
        cOpts.map_id = opts.mapId.c_str();
        cOpts.chunk_size = opts.chunkSize;
        cOpts.use_gzip = opts.useGzip ? 1 : 0;
        return static_cast<Error>(
            xg_upload_map(m_handle, &cOpts, &Device::mapProgressBridge, this));
    }

    /** 激活指定地图用于重定位 */
    Error setActiveMap(const std::string& mapId) {
        return static_cast<Error>(xg_set_active_map(m_handle, mapId.c_str()));
    }

    /**
     * @brief  查询设备上已有的地图列表（含文件层次清单）
     * @param  out 输出地图列表
     * @return Error::Ok 成功
     */
    Error listMaps(std::vector<MapInfo>* out) {
        if (out == nullptr) { return Error::InvalidArg; }
        xg_map_list_t cList;
        std::memset(&cList, 0, sizeof(cList));
        cList.struct_size = sizeof(cList);
        auto rc = static_cast<Error>(xg_list_maps(m_handle, &cList));
        if (rc != Error::Ok) { return rc; }
        out->clear();
        out->reserve(cList.count);
        for (size_t i = 0; i < cList.count; ++i) {
            const xg_map_info_t& ci = cList.maps[i];
            MapInfo info;
            info.mapId = ci.map_id;
            info.isActive = (ci.is_active != 0);
            info.totalSize = ci.total_size;
            info.files.reserve(ci.file_count);
            for (size_t j = 0; j < ci.file_count; ++j) {
                MapFileEntry e;
                e.path = ci.files[j].path;
                e.size = ci.files[j].size;
                e.mtime = ci.files[j].mtime;
                info.files.push_back(std::move(e));
            }
            out->push_back(std::move(info));
        }
        return Error::Ok;
    }

    /** 删除设备上指定地图 */
    Error deleteMap(const std::string& mapId) {
        return static_cast<Error>(xg_delete_map(m_handle, mapId.c_str()));
    }

private:
    explicit Device(xg_device_t* handle) : m_handle(handle) {}

    static void mapProgressBridge(float progress, void* ud) {
        auto* self = static_cast<Device*>(ud);
        if (self->m_mapProgressCb) {
            self->m_mapProgressCb(progress);
        }
    }

    static void initStatus(xg_record_status_t* cStatus) {
        std::memset(cStatus, 0, sizeof(*cStatus));
        cStatus->struct_size = sizeof(*cStatus);
    }

    /**
     * @brief 将 C 结构体转换为 C++ 结构体
     * @note  out 为空时直接返回，便于调用方不关心状态详情时省略该参数。
     */
    static void fromCStatus(const xg_record_status_t& cStatus,
                            RecordStatus* out) {
        if (out == nullptr) {
            return;
        }
        out->state = static_cast<RecordState>(cStatus.state);
        out->warning = (cStatus.warning != 0);
        out->rawCode = cStatus.raw_code;

        // C 侧保证以 '\0' 结尾，此处仍限长，避免实现侧缺陷导致越界读取。
        // 不用 strnlen，它属 POSIX 扩展，未必在所有集成方环境中可见。
        std::size_t len = 0;
        while (len < sizeof(cStatus.message) && cStatus.message[len] != '\0') {
            ++len;
        }
        out->message.assign(cStatus.message, len);
    }


    static void poseCbBridge(const xg_pose_t* pose, void* ud) {
        auto* self = static_cast<Device*>(ud);
        if (self->m_poseCb && pose != nullptr) {
            Pose p;
            p.timestampSec = pose->timestamp_sec;
            p.posX = pose->pos_x;
            p.posY = pose->pos_y;
            p.posZ = pose->pos_z;
            p.quatW = pose->quat_w;
            p.quatX = pose->quat_x;
            p.quatY = pose->quat_y;
            p.quatZ = pose->quat_z;
            p.velX = pose->vel_x;
            p.velY = pose->vel_y;
            p.velZ = pose->vel_z;
            p.gyroX = pose->gyro_x;
            p.gyroY = pose->gyro_y;
            p.gyroZ = pose->gyro_z;
            p.errorCode = pose->error_code;
            p.isLocal = (pose->is_local != 0);
            self->m_poseCb(p);
        }
    }

    static void imuCbBridge(const xg_imu_t* imu, void* ud) {
        auto* self = static_cast<Device*>(ud);
        if (self->m_imuCb && imu != nullptr) {
            Imu i;
            i.timestampSec = imu->timestamp_sec;
            i.accX = imu->acc_x;
            i.accY = imu->acc_y;
            i.accZ = imu->acc_z;
            i.gyrX = imu->gyr_x;
            i.gyrY = imu->gyr_y;
            i.gyrZ = imu->gyr_z;
            self->m_imuCb(i);
        }
    }

    static void deviceUndistCloudCbBridge(const xg_cloud_frame_t* frame,
                                             void* ud) {
        auto* self = static_cast<Device*>(ud);
        if (self->m_deviceUndistCloudCb && frame != nullptr) {
            CloudFrame cf;
            cf.points = frame->points;
            cf.count = frame->count;
            cf.timestampSec = frame->timestamp_sec;
            cf.seq = frame->seq;
            self->m_deviceUndistCloudCb(cf);
        }
    }

    static void deviceRawCloudCbBridge(const xg_raw_cloud_frame_t* frame,
                                       void* ud) {
        auto* self = static_cast<Device*>(ud);
        if (self->m_deviceRawCloudCb && frame != nullptr) {
            RawCloudFrame rf;
            rf.points = frame->points;
            rf.count = frame->count;
            rf.timestampSec = frame->timestamp_sec;
            rf.seq = frame->seq;
            self->m_deviceRawCloudCb(rf);
        }
    }

    static void imageCbBridge(const xg_image_frame_t* frame, void* ud) {
        auto* self = static_cast<Device*>(ud);
        if (frame == nullptr) {
            return;
        }
        int idx = static_cast<int>(frame->camera_id);
        if (idx < 0 || idx > 2 || !self->m_imageCb[idx]) {
            return;
        }
        ImageFrame img;
        img.cameraId = static_cast<CameraId>(frame->camera_id);
        img.timestampSec = frame->timestamp_sec;
        img.jpegData = frame->jpeg_data;
        img.jpegSize = frame->jpeg_size;
        img.seq = frame->seq;
        self->m_imageCb[idx](img);
    }

    static void depthCbBridge(const xg_depth_frame_t* frame, void* ud) {
        auto* self = static_cast<Device*>(ud);
        if (frame == nullptr) {
            return;
        }
        int idx = static_cast<int>(frame->camera_id);
        if (idx < 0 || idx > 2 || !self->m_depthCb[idx]) {
            return;
        }
        DepthFrame df;
        df.cameraId = static_cast<CameraId>(frame->camera_id);
        df.timestampSec = frame->timestamp_sec;
        df.width = frame->width;
        df.height = frame->height;
        df.depthData = frame->depth_data;
        df.depthSize = frame->depth_size;
        df.seq = frame->seq;
        self->m_depthCb[idx](df);
    }

    static void errCodeCbBridge(const xg_err_code_frame_t* frame, void* ud) {
        auto* self = static_cast<Device*>(ud);
        if (!self->m_errCodeCb || frame == nullptr || frame->entries == nullptr) {
            return;
        }
        ErrCodeFrame ef;
        ef.entries.reserve(frame->count);
        for (std::size_t i = 0; i < frame->count; ++i) {
            ErrCodeEntry entry;
            entry.code = frame->entries[i].code;
            entry.timestamp = frame->entries[i].timestamp;
            ef.entries.push_back(entry);
        }
        self->m_errCodeCb(ef);
    }

private:
    xg_device_t* m_handle{nullptr};
    std::function<void(const Pose&)> m_poseCb;
    std::function<void(const Imu&)> m_imuCb;
    std::function<void(const CloudFrame&)> m_deviceUndistCloudCb;
    std::function<void(const RawCloudFrame&)> m_deviceRawCloudCb;
    std::function<void(const ImageFrame&)> m_imageCb[3];
    std::function<void(const DepthFrame&)> m_depthCb[3];
    std::function<void(const ErrCodeFrame&)> m_errCodeCb;
    std::function<void(float)> m_mapProgressCb;
};

}  // namespace xg
