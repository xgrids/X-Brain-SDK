/******************************************************************************
This file is part of XGRIDS Robot SDK.
Project: xg_robot_sdk
Author : XGRIDS Embedded Software Team
*******************************************************************************
History:
1. create this file. (DATE: 20260804)
******************************************************************************/

/**
 * @file   robot_sdk_c.h
 * @brief  XGRIDS 设备录制控制 SDK —— C ABI 接口
 *
 * 本头文件是 SDK 的稳定 ABI 边界，供集成方直接使用，或经 robot_sdk.h
 * 提供的 C++ 封装间接使用。
 *
 * @note 设计约束（修改本文件前务必阅读）：
 *   1. 只使用 C 标准类型。不得引入 appkit/basetype.h 等项目内部头文件，
 *      集成方环境中不存在这些头文件。
 *   2. 不得出现 zenoh、protobuf、ModelingState 等实现细节。传输方式与
 *      内部协议均可能变更，本文件是对外契约，须与之解耦。
 *   3. 不得使用 DECL_CLASSMETA / DECL_PROPERTY 等项目宏，同第 1 条。
 *   4. 结构体一律带 struct_size 首字段，由调用方填写，用于后续扩展时
 *      保持二进制兼容。新增字段只允许追加在结构体末尾。
 *   5. 所有接口以返回值传递错误，不抛异常。
 *
 * @note 线程安全性：同一 xg_device_t 句柄的接口调用不可重入，
 *   多线程使用时须由调用方加锁；不同句柄之间互不影响。
 */

#ifndef __XG_ROBOT_SDK_C_H__
#define __XG_ROBOT_SDK_C_H__

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * 符号可见性
 *===========================================================================*/

/**
 * 接口导出标记。
 *
 * SDK 自身以 -fvisibility=hidden 编译，隐藏全部符号，只有本宏标记的接口
 * 对外可见，从而避免静态链入的第三方库符号泄漏到集成方，
 * 引发难以定位的符号冲突。
 *
 * 集成方编译时 XG_SDK_BUILDING 未定义，本宏展开为空，不产生任何影响。
 */
#if defined(XG_SDK_BUILDING) && defined(__GNUC__)
#define XG_API __attribute__((visibility("default")))
#else
#define XG_API
#endif

/*=============================================================================
 * 版本
 *
 * 版本号的唯一来源是下面三个宏。CMake 工程版本、SOVERSION（取
 * XG_SDK_VERSION_MAJOR.XG_SDK_VERSION_MINOR，两位）、xg_sdk_version() 返回值、
 * ROS2 包版本、对外 README 都从这里派生，升级版本只改这三行。
 * 动态库 SONAME = liblixel_sdk.so.<主>.<次>：补丁版本只修 Bug，SONAME 不变；
 * 次版本允许小范围增删接口，SONAME 随之变化，集成方须重新链接；
 * 主版本只在大重构时提升。
 * 注意：三行须保持 "#define XG_SDK_VERSION_<部件> <十进制数>" 的格式，
 *       cmake/LixelSdkVersion.cmake 与 ros2/CMakeLists.txt 按此格式解析。
 *===========================================================================*/

#define XG_SDK_VERSION_MAJOR 1
#define XG_SDK_VERSION_MINOR 1
#define XG_SDK_VERSION_PATCH 0

#if XG_SDK_VERSION_MINOR > 99 || XG_SDK_VERSION_PATCH > 99
#error "XG_SDK_VERSION_MINOR/PATCH must be <= 99 to fit XG_SDK_VERSION_NUMBER"
#endif

/**
 * 整数形式版本号：MAJOR * 10000 + MINOR * 100 + PATCH，如 1.1.0 为 10100。
 * 供集成方按编译期头文件版本做条件编译，例如：
 *   #if XG_SDK_VERSION_NUMBER >= 10100
 *       xg_subscribe_depth(...);
 *   #endif
 * 只反映编译时头文件的版本；运行时库的版本用 xg_sdk_version() 核对。
 */
#define XG_SDK_VERSION_NUMBER                                         \
    (XG_SDK_VERSION_MAJOR * 10000 + XG_SDK_VERSION_MINOR * 100 + \
     XG_SDK_VERSION_PATCH)

/* 内部辅助：把宏展开后的数值转成字符串字面量，集成方勿直接使用 */
#define XG_SDK_VERSION_STR_IMPL_(x) #x
#define XG_SDK_VERSION_STR_(x) XG_SDK_VERSION_STR_IMPL_(x)

/** 字符串形式版本号，如 "1.1.0"；与同版本库的 xg_sdk_version() 返回值相同 */
#define XG_SDK_VERSION_STRING                       \
    XG_SDK_VERSION_STR_(XG_SDK_VERSION_MAJOR) "." \
    XG_SDK_VERSION_STR_(XG_SDK_VERSION_MINOR) "." \
    XG_SDK_VERSION_STR_(XG_SDK_VERSION_PATCH)

/*=============================================================================
 * 常量
 *===========================================================================*/

/** message 字段缓冲区长度（含结尾 '\0'） */
#define XG_MESSAGE_MAX 256

/** device_ip 字段缓冲区长度（含结尾 '\0'） */
#define XG_IP_MAX 64

/** modeling_app 默认端口（SDK 数据面与命令面都在 modeling_app 的同一 session） */
#define XG_DEFAULT_PORT 7448

/** 建立连接的默认超时（毫秒） */
#define XG_DEFAULT_CONNECT_TIMEOUT_MS 5000u

/**
 * 轮询设备状态的推荐间隔（毫秒）。
 *
 * xg_record_start() / xg_record_stop() 只下发命令、不等待状态变化，
 * 需由调用方轮询 xg_record_query() 确认结果。间隔不宜小于此值，
 * 过于频繁的查询会占用设备侧处理资源。
 */
#define XG_RECOMMENDED_POLL_INTERVAL_MS 500u

/*=============================================================================
 * 错误码
 *===========================================================================*/

/**
 * @brief 接口返回值
 * @note  数值一经发布不得变更，只允许追加新枚举项。
 */
typedef enum {
    XG_OK = 0,                 /**< 成功 */
    XG_ERR_INVALID_ARG = -1,   /**< 参数非法（空指针、struct_size 不匹配等） */
    XG_ERR_NOT_CONNECTED = -2, /**< 未连接到设备 */
    XG_ERR_TIMEOUT = -3,       /**< 超时。启停录制时表示未在限定时间内到达目标状态 */
    XG_ERR_REJECTED = -4,      /**< 设备拒绝执行，当前状态不允许该操作 */
    XG_ERR_DEVICE_ERROR = -5,  /**< 设备处于故障状态，详见 xg_record_status_t */
    XG_ERR_PROTOCOL = -6,      /**< 设备应答无法解析，通常为版本不匹配 */
    XG_ERR_INTERNAL = -7,      /**< SDK 内部错误 */
} xg_err_t;

/*=============================================================================
 * 录制状态
 *===========================================================================*/

/**
 * @brief 录制状态
 * @note  设备内部状态数量远多于此处，SDK 已归并为以下五种对外状态。
 *        需要原始状态码时读取 xg_record_status_t::raw_code，但该字段仅供
 *        排障参考，不保证跨版本稳定，不应作为业务判断依据。
 */
typedef enum {
    XG_REC_IDLE = 0,      /**< 待机，可以开始录制 */
    XG_REC_STARTING = 1,  /**< 启动中，尚未开始出数据 */
    XG_REC_RECORDING = 2, /**< 录制中 */
    XG_REC_STOPPING = 3,  /**< 停止中 */
    XG_REC_ERROR = 4,     /**< 故障，详见 message 字段 */
} xg_record_state_t;

/**
 * @brief 录制状态详情
 * @note  由调用方分配，调用前须置 struct_size = sizeof(xg_record_status_t)。
 */
typedef struct {
    size_t struct_size; /**< 必须由调用方填写为 sizeof(xg_record_status_t) */

    xg_record_state_t state; /**< 归并后的对外状态 */

    /**
     * 告警标志。为 1 时表示存在需要关注的异常，但不影响当前 state 所
     * 允许的操作。典型情形：
     *   - state 为 XG_REC_RECORDING：建图过程异常、已退化为仅录包模式，
     *     录制仍在继续，可正常停止；
     *   - state 为 XG_REC_IDLE：上一次工程的后处理存在异常，设备本身可用。
     * 与 XG_REC_ERROR 的区别是：告警不阻塞操作，故障状态则阻塞。
     */
    uint8_t warning;

    uint8_t reserved[3]; /**< 对齐填充，保留 */

    /**
     * 设备原始状态码，仅供排障与日志记录。
     * 该值反映设备内部状态机，不保证跨版本稳定。
     */
    int32_t raw_code;

    char message[XG_MESSAGE_MAX]; /**< 人可读说明，UTF-8，始终以 '\0' 结尾 */
} xg_record_status_t;

/*=============================================================================
 * 连接参数
 *===========================================================================*/

/**
 * @brief 连接参数
 * @note  由调用方分配，调用前须置 struct_size = sizeof(xg_options_t)。
 *        数值型字段填 0 表示采用默认值。
 */
typedef struct {
    size_t struct_size; /**< 必须由调用方填写为 sizeof(xg_options_t) */

    char device_ip[XG_IP_MAX]; /**< 设备 IP，必填。当前要求与设备同一交换机直连 */

    uint16_t port;     /**< modeling_app 端口，0 表示使用 XG_DEFAULT_PORT（即 7448） */
    uint16_t reserved; /**< 对齐填充，保留 */

    uint32_t connect_timeout_ms; /**< 连接超时，0 表示使用默认值 */
} xg_options_t;

/*=============================================================================
 * 设备句柄
 *===========================================================================*/

/**
 * @brief 设备句柄（不透明类型）
 * @note  内部结构不对外暴露，集成方只能持有指针。
 */
typedef struct xg_device xg_device_t;

/**
 * @brief  连接设备并创建句柄
 * @param  opts    连接参数，不可为空
 * @param  out_dev 输出句柄，不可为空。成功时由 SDK 分配，须以
 *                 xg_device_close() 释放；失败时置为 NULL
 * @return XG_OK 成功；XG_ERR_TIMEOUT 未能在超时内连上设备
 * @note   一个句柄对应一台设备。当前版本不支持一个句柄连接多台设备。
 */
XG_API xg_err_t xg_device_open(const xg_options_t* opts, xg_device_t** out_dev);

/**
 * @brief 关闭句柄并释放资源
 * @param dev 设备句柄，传入 NULL 时不做任何事
 * @note  关闭后该指针不得再使用。
 */
XG_API void xg_device_close(xg_device_t* dev);

/**
 * @brief  查询与设备的连接是否正常
 * @param  dev 设备句柄
 * @return 1 已连接；0 未连接或 dev 为空
 */
XG_API int xg_device_is_connected(const xg_device_t* dev);

/*=============================================================================
 * 录制控制
 *===========================================================================*/

/**
 * @brief  下发开始录制命令
 * @param  dev 设备句柄
 * @return XG_OK 设备已接受命令，或设备本就在录制中（幂等）；
 *         XG_ERR_REJECTED 设备拒绝执行；
 *         XG_ERR_TIMEOUT 未收到设备应答（通信层面，命令是否送达未知）；
 *         XG_ERR_PROTOCOL 设备应答无法解析
 *
 * @note   **本接口不等待录制真正开始。** 返回 XG_OK 仅表示设备已接受命令。
 *         设备需要初始化定位与建图后才会进入录制状态，耗时与地图规模等
 *         因素相关，从数秒到数十秒不等，无法预设固定上限。因此结果须由
 *         调用方轮询 xg_record_query() 确认，等到状态变为 XG_REC_RECORDING
 *         方可认为录制已开始。
 *
 * @note   轮询期间的状态含义：
 *           XG_REC_STARTING  正在初始化，继续等待
 *           XG_REC_RECORDING 已开始录制，可以进行后续作业
 *           XG_REC_ERROR     启动失败，原因见 message，不必继续等待
 *           XG_REC_IDLE      命令尚未生效，继续等待
 *
 * @note   返回 XG_ERR_TIMEOUT 时命令可能已经送达设备。**不要直接重试**，
 *         应先调用 xg_record_query() 确认实际状态，否则会重复下发。
 */
XG_API xg_err_t xg_record_start(xg_device_t* dev);

/**
 * @brief  下发停止录制命令
 * @param  dev 设备句柄
 * @return XG_OK 设备已接受命令，或设备本就未在录制（幂等）；
 *         XG_ERR_REJECTED 设备拒绝执行；
 *         XG_ERR_TIMEOUT 未收到设备应答；
 *         XG_ERR_PROTOCOL 设备应答无法解析
 *
 * @note   与 xg_record_start() 同样不等待状态变化，需轮询
 *         xg_record_query() 直至状态变为 XG_REC_IDLE。
 *         停止过程涉及数据落盘，耗时同样与数据量相关。
 *
 * @note   处于告警状态（warning 为 1）时同样可以正常停止。
 */
XG_API xg_err_t xg_record_stop(xg_device_t* dev);

/**
 * @brief  查询当前录制状态
 * @param  dev        设备句柄
 * @param  out_status 输出录制状态，不可为空，须先填好 struct_size
 * @return XG_OK 成功；XG_ERR_TIMEOUT 设备未在超时内应答
 * @note   本接口不改变设备状态，可安全轮询。这是确认启停结果的唯一途径，
 *         建议间隔不小于 XG_RECOMMENDED_POLL_INTERVAL_MS。
 */
XG_API xg_err_t xg_record_query(xg_device_t* dev, xg_record_status_t* out_status);

/*=============================================================================
 * 辅助接口
 *===========================================================================*/

/**
 * @brief  错误码转可读字符串
 * @param  err 错误码
 * @return 静态字符串，不需释放；未知错误码返回 "unknown error"
 */
XG_API const char* xg_err_str(xg_err_t err);

/**
 * @brief  录制状态转可读字符串
 * @param  state 录制状态
 * @return 静态字符串，不需释放
 */
XG_API const char* xg_record_state_str(xg_record_state_t state);

/**
 * @brief  取 SDK 版本号（运行时库的版本）
 * @return "MAJOR.MINOR.PATCH" 形式的静态字符串，不需释放；值等于构建该库时
 *         头文件的 XG_SDK_VERSION_STRING。与集成方编译时的
 *         XG_SDK_VERSION_STRING 不同，说明头文件与库版本不匹配。
 */
XG_API const char* xg_sdk_version(void);

/*=============================================================================
 * 数据订阅类型
 *===========================================================================*/

/** 单个点的 POD 表示（设备/IMU 系） */
typedef struct {
    float x;         /**< 设备(IMU)系 X 坐标 (m) */
    float y;         /**< 设备(IMU)系 Y 坐标 (m) */
    float z;         /**< 设备(IMU)系 Z 坐标 (m) */
    float intensity; /**< 反射强度 */
} xg_point_t;

/** 点云帧数据 */
typedef struct {
    size_t struct_size;        /**< 必须由 SDK 填写为 sizeof(xg_cloud_frame_t) */
    const xg_point_t* points; /**< 点数组首地址，回调返回后不可再访问 */
    size_t count;              /**< 点数量 */
    double timestamp_sec;      /**< 时间戳（秒） */
    uint32_t seq;              /**< 帧序号，自订阅起单调递增 */
} xg_cloud_frame_t;

/**
 * @brief 点云回调函数类型
 * @param frame     点云帧数据，仅在回调执行期间有效
 * @param user_data 用户在订阅时传入的上下文指针
 * @note  回调在 SDK 内部线程触发，实现须尽快返回，不可阻塞。
 */
typedef void (*xg_cloud_cb_t)(const xg_cloud_frame_t* frame, void* user_data);

/**
 * 原始雷达点（设备(IMU)系，未去畸变，未降采样）。
 * 固定 24 字节布局，无 struct_size，发布后永不改动；需要新字段时另起新类型。
 */
typedef struct {
    float x;                /**< 设备(IMU)系 X 坐标 (m)，线路上四舍五入到 1 mm */
    float y;                /**< 设备(IMU)系 Y 坐标 (m)，线路上四舍五入到 1 mm */
    float z;                /**< 设备(IMU)系 Z 坐标 (m)，线路上四舍五入到 1 mm */
    float intensity;        /**< 雷达反射强度，无单位 */
    int32_t time_offset_us; /**< 该点相对 frame.timestamp_sec 的偏移 (us)，可为负 */
    uint16_t ring;          /**< 雷达线束号，超过 65535 时饱和为 65535 */
    uint16_t reserved;      /**< 保留，集成方须忽略其值（当前实现写 0，预留给逐点标志） */
} xg_raw_point_t;

/** 原始点云帧数据（xg_subscribe_device_raw_cloud 的回调参数） */
typedef struct {
    size_t struct_size;            /**< 由 SDK 填写为 sizeof(xg_raw_cloud_frame_t) */
    const xg_raw_point_t* points;  /**< 点数组首地址，回调返回后不可再访问 */
    size_t count;                  /**< 点数量 */
    double timestamp_sec;          /**< 帧内最早点的采样时刻（雷达自身时钟的 Unix epoch 秒，
                                        不保证等于设备系统时钟，见 api_reference_v2.md §0.8） */
    uint32_t seq;                  /**< 设备端雷达 frame_id，不是 SDK 本地计数 */
} xg_raw_cloud_frame_t;

/**
 * @brief 原始点云回调函数类型
 * @param frame     原始点云帧，仅在回调执行期间有效
 * @param user_data 用户在订阅时传入的上下文指针
 * @note  回调在 Zenoh 会话接收线程触发，实现须尽快返回，不可阻塞。
 */
typedef void (*xg_raw_cloud_cb_t)(const xg_raw_cloud_frame_t* frame,
                                  void* user_data);

/** 位姿数据（设备/IMU 系，R_W_I；坐标系见 coordinate_frames.md） */
typedef struct {
    size_t struct_size;   /**< 必须由 SDK 填写为 sizeof(xg_pose_t) */
    double timestamp_sec; /**< 时间戳（秒） */

    float pos_x;  /**< 设备(IMU)点在世界系下的 X 位置 (m) */
    float pos_y;  /**< 设备(IMU)点在世界系下的 Y 位置 (m) */
    float pos_z;  /**< 设备(IMU)点在世界系下的 Z 位置 (m) */

    float quat_w; /**< 四元数实部 (W←I, R_W_I) */
    float quat_x; /**< 四元数 X (W←I) */
    float quat_y; /**< 四元数 Y (W←I) */
    float quat_z; /**< 四元数 Z (W←I) */

    float vel_x;  /**< 设备(IMU)系线速度 X (m/s) */
    float vel_y;  /**< 设备(IMU)系线速度 Y (m/s) */
    float vel_z;  /**< 设备(IMU)系线速度 Z (m/s) */

    float gyro_x; /**< 设备(IMU)系角速度 X (rad/s) */
    float gyro_y; /**< 设备(IMU)系角速度 Y (rad/s) */
    float gyro_z; /**< 设备(IMU)系角速度 Z (rad/s) */

    uint32_t error_code; /**< 定位故障码，0 = 正常 */
    uint8_t is_local;    /**< 保留字段：1.0 暂不提供，恒为 0，勿使用（proto 对齐后启用） */
    uint8_t reserved[3]; /**< 对齐填充 */
} xg_pose_t;

/**
 * @brief 位姿回调函数类型
 * @param pose      位姿数据，仅在回调执行期间有效
 * @param user_data 用户在订阅时传入的上下文指针
 * @note  回调在 SDK 内部线程触发，实现须尽快返回，不可阻塞。
 */
typedef void (*xg_pose_cb_t)(const xg_pose_t* pose, void* user_data);

/*=============================================================================
 * 数据订阅接口
 *===========================================================================*/


/**
 * @brief  订阅位姿数据
 * @param  dev       设备句柄
 * @param  cb        回调函数，不可为空
 * @param  user_data 透传给回调的用户指针，可为空
 * @return XG_OK 订阅成功；XG_ERR_INVALID_ARG 参数非法；
 *         XG_ERR_REJECTED 已有活跃的位姿订阅（须先取消）
 * @note   回调频率约 100Hz，在 SDK 内部线程触发。
 *         回调中传递的 pose 指针仅在回调执行期间有效。
 */
XG_API xg_err_t xg_subscribe_pose(xg_device_t* dev,
                                  xg_pose_cb_t cb, void* user_data);

/**
 * @brief  取消位姿订阅
 * @param  dev 设备句柄
 * @return XG_OK 取消成功（幂等）；XG_ERR_INVALID_ARG dev 为空
 * @note   返回后保证不再触发位姿回调。
 */
XG_API xg_err_t xg_unsubscribe_pose(xg_device_t* dev);

/*=============================================================================
 * IMU 数据订阅
 *===========================================================================*/

/** IMU 数据 */
typedef struct {
    size_t struct_size;    /**< 必须由 SDK 填写为 sizeof(xg_imu_t) */
    double timestamp_sec;  /**< 时间戳（秒） */
    float acc_x;           /**< 加速度 X (m/s^2) */
    float acc_y;           /**< 加速度 Y (m/s^2) */
    float acc_z;           /**< 加速度 Z (m/s^2) */
    float gyr_x;           /**< 角速度 X (rad/s) */
    float gyr_y;           /**< 角速度 Y (rad/s) */
    float gyr_z;           /**< 角速度 Z (rad/s) */
} xg_imu_t;

/**
 * @brief IMU 回调函数类型
 * @param imu       IMU 数据，仅在回调执行期间有效
 * @param user_data 用户在订阅时传入的上下文指针
 */
typedef void (*xg_imu_cb_t)(const xg_imu_t* imu, void* user_data);

/**
 * @brief  订阅 IMU 数据
 * @param  dev       设备句柄
 * @param  cb        回调函数，不可为空
 * @param  user_data 透传给回调的用户指针，可为空
 * @return XG_OK 订阅成功；XG_ERR_REJECTED 已有活跃订阅
 * @note   回调频率约 200Hz，在 SDK 内部线程触发。
 */
XG_API xg_err_t xg_subscribe_imu(xg_device_t* dev,
                                 xg_imu_cb_t cb, void* user_data);

/**
 * @brief  取消 IMU 订阅
 * @param  dev 设备句柄
 * @return XG_OK 取消成功（幂等）
 */
XG_API xg_err_t xg_unsubscribe_imu(xg_device_t* dev);

/**
 * @brief  订阅设备(IMU)系 Locate 去畸变点云（Zenoh key: device_undist_cloud）
 * @param  dev       设备句柄
 * @param  cb        回调函数，不可为空
 * @param  user_data 透传给回调的用户指针，可为空
 * @return XG_OK 订阅成功（设备端暂无该 key 数据时同样返回 XG_OK，只是暂无回调）；
 *         XG_ERR_INVALID_ARG dev 或 cb 为空；
 *         XG_ERR_REJECTED 本句柄已有去畸变点云订阅（须先取消）；
 *         XG_ERR_INTERNAL Zenoh 订阅声明失败
 * @note   数据：设备端 Locate 输出的去畸变点云（imu_cloud），点坐标为设备(IMU)系，
 *         时间戳为该帧 Locate 输出时间戳（秒）。当前 Locate 输入为驱动层 1/4 降采样点云；全量输入属 locate-full-cloud-feed（RFC 状态：提案，未实现）。
 *         频率随 Locate 输出（量级 10Hz，未实测）。
 * @note   语义变更（1.1）：旧语义为 SLAM 去畸变单帧、上游 1/4 降采样。
 *         新语义下内容来源为 Locate 输出，点数取决于 Locate 输入（未验证）。
 *         重定位点云订阅 xg_subscribe_device_reloc_cloud 已在 1.1 删除，
 *         本 key 是 Locate 点云的唯一入口。
 * @note   无数据的情况（订阅仍返回 XG_OK，SDK 无法区分）：建图阶段
 *         Locate 未输出时本 key 无数据。
 * @note   丢帧判断：frame.seq 是 SDK 本地订阅计数（自订阅起递增），不是设备
 *         帧号，不能用其跳变发现丢帧；点云丢帧只能用相邻帧 timestamp_sec 的
 *         间隔判断。
 * @note   线程：回调在 Zenoh 会话接收线程触发，不在调用方线程；回调内禁止
 *         同步文件 IO、join、startLocate 等阻塞操作，禁止再调同一句柄的 SDK 接口。
 *         阻塞会拖住本进程全部订阅。回调单次耗时上限：待定（ms，未确定）。
 *         单帧点数多，回调内请只做拷贝或入队。
 * @note   内存：frame 及 frame->points 由 SDK 持有，仅回调执行期间有效，
 *         下一帧覆盖同一缓冲；需保留须在回调内深拷贝，禁止 free/delete。
 * @note   失败后状态：返回非 XG_OK 时不登记订阅、不保存 cb，可修正后重试；
 *         XG_ERR_REJECTED 不影响已有订阅。
 */
XG_API xg_err_t xg_subscribe_device_undist_cloud(xg_device_t* dev,
                                                    xg_cloud_cb_t cb,
                                                    void* user_data);

/**
 * @brief  取消设备(IMU)系 SLAM 去畸变点云订阅
 * @param  dev 设备句柄
 * @return XG_OK 取消成功（幂等，无活跃订阅时也返回成功）；
 *         XG_ERR_INVALID_ARG dev 为空
 * @note   返回后保证不再触发该回调。禁止在该订阅自己的回调内调用。
 */
XG_API xg_err_t xg_unsubscribe_device_undist_cloud(xg_device_t* dev);

/**
 * @brief  订阅设备(IMU)系原始雷达点云（Zenoh key: device_raw_cloud）
 * @param  dev       设备句柄
 * @param  cb        回调函数，不可为空
 * @param  user_data 透传给回调的用户指针，可为空
 * @return XG_OK 订阅成功（设备端暂无该 key 数据时同样返回 XG_OK，只是暂无回调）；
 *         XG_ERR_INVALID_ARG dev 或 cb 为空；
 *         XG_ERR_REJECTED 本句柄已有原始点云订阅（须先取消）；
 *         XG_ERR_INTERNAL Zenoh 订阅声明失败
 * @note   数据：RoboSense 一次完整扫描的全部点，~10Hz。不做降采样，不做运动
 *         去畸变；只做静态外参刚体变换，把点从雷达系变换到设备(IMU)系
 *         （外参为 extrinsic_imu_lidar，Lidar→IMU，见 xg_lidar_calib_t）。
 *         集成方用每点的 time_offset_us 配合自己的位姿自行去畸变。
 *         不过滤被屏蔽线束。与 device_undist_cloud
 *         语义不同，订阅槽互相独立。
 * @note   无数据的情况（订阅仍返回 XG_OK，SDK 无法区分）：设备雷达不是
 *         RoboSense；设备端外参文件不可用（该帧被丢弃）；设备未处于扫描状态；
 *         设备端拥塞丢帧（可用 frame.seq 的跳变发现，本路 seq 取设备雷达
 *         frame_id）。
 * @note   线路精度：坐标在设备端四舍五入到 1 mm（最大误差 0.5 mm）。
 * @note   线程：回调在 Zenoh 会话接收线程触发，不在调用方线程；
 *         回调内禁止阻塞、禁止再调同一句柄的 SDK 接口。阻塞会拖住本进程
 *         全部订阅。
 * @note   内存：frame 及 frame->points 由 SDK 持有（线程本地复用缓冲），
 *         仅回调执行期间有效；需保留须在回调内深拷贝，禁止 free/delete。
 *         单帧点数多，回调内请只做拷贝或入队。
 * @note   失败后状态：返回非 XG_OK 时不登记订阅、不保存 cb，可修正后重试；
 *         XG_ERR_REJECTED 不影响已有订阅。
 * @note   禁止用法：保存 frame->points 指针到回调之外；在回调内取消订阅或
 *         关闭句柄。
 */
XG_API xg_err_t xg_subscribe_device_raw_cloud(xg_device_t* dev,
                                              xg_raw_cloud_cb_t cb,
                                              void* user_data);

/**
 * @brief  取消设备(IMU)系原始点云订阅
 * @param  dev 设备句柄
 * @return XG_OK 取消成功（幂等，无活跃订阅时也返回成功）；
 *         XG_ERR_INVALID_ARG dev 为空
 * @note   返回后保证不再触发该回调。禁止在该订阅自己的回调内调用。
 */
XG_API xg_err_t xg_unsubscribe_device_raw_cloud(xg_device_t* dev);

/*=============================================================================
 * 图像数据订阅
 *===========================================================================*/

/** 相机索引 */
typedef enum {
    XG_CAMERA_LEFT = 0,    /**< 左相机 */
    XG_CAMERA_CENTER = 1,  /**< 中相机 */
    XG_CAMERA_RIGHT = 2,   /**< 右相机 */
} xg_camera_id_t;

/** 图像帧数据 */
typedef struct {
    size_t struct_size;          /**< 必须由 SDK 填写为 sizeof(xg_image_frame_t) */
    xg_camera_id_t camera_id;   /**< 相机标识 */
    double timestamp_sec;        /**< 时间戳（秒） */
    const uint8_t* jpeg_data;   /**< JPEG 裸数据指针，回调返回后不可再访问 */
    size_t jpeg_size;            /**< JPEG 数据大小（字节） */
    uint32_t seq;                /**< 帧序号，自订阅起单调递增 */
} xg_image_frame_t;

/**
 * @brief 图像回调函数类型
 * @param frame     图像帧数据，仅在回调执行期间有效
 * @param user_data 用户在订阅时传入的上下文指针
 * @note  回调在 SDK 内部线程触发，实现须尽快返回，不可阻塞。
 */
typedef void (*xg_image_cb_t)(const xg_image_frame_t* frame, void* user_data);

/**
 * @brief  订阅图像数据
 * @param  dev        设备句柄
 * @param  camera_id  相机标识（XG_CAMERA_LEFT / CENTER / RIGHT）
 * @param  cb         回调函数，不可为空
 * @param  user_data  透传给回调的用户指针，可为空
 * @return XG_OK 订阅成功；XG_ERR_INVALID_ARG 参数非法；
 *         XG_ERR_REJECTED 该相机已有活跃订阅（须先取消）
 * @note   回调频率取决于设备端 JPEG 编码配置（帧率以设备配置为准，实测 10 Hz）。
 *         回调中 jpeg_data 指针仅在回调执行期间有效。
 */
XG_API xg_err_t xg_subscribe_image(xg_device_t* dev, xg_camera_id_t camera_id,
                                   xg_image_cb_t cb, void* user_data);

/**
 * @brief  取消图像订阅
 * @param  dev       设备句柄
 * @param  camera_id 相机标识
 * @return XG_OK 取消成功（幂等）
 */
XG_API xg_err_t xg_unsubscribe_image(xg_device_t* dev, xg_camera_id_t camera_id);

/*=============================================================================
 * 深度图数据订阅
 *===========================================================================*/

/** 深度图帧数据(AI 推理产出的度量深度, 单位米) */
typedef struct {
    size_t struct_size; /**< 必须由 SDK 填写为 sizeof(xg_depth_frame_t) */
    xg_camera_id_t camera_id; /**< 相机标识, 当前仅 XG_CAMERA_CENTER 有效 */
    double timestamp_sec;     /**< 时间戳（秒） */
    uint32_t width;      /**< 深度图宽度（像素） */
    uint32_t height;     /**< 深度图高度（像素） */
    const float* depth_data; /**< 度量深度裸数据指针(行优先, 单位米, 未命中
                                  区域为 0), 回调返回后不可再访问 */
    size_t depth_size;   /**< depth_data 字节数, 等于 width*height*sizeof(float) */
    uint32_t seq;        /**< 帧序号，自订阅起单调递增 */
} xg_depth_frame_t;

/**
 * @brief 深度图回调函数类型
 * @param frame     深度图帧数据，仅在回调执行期间有效
 * @param user_data 用户在订阅时传入的上下文指针
 * @note  回调在 SDK 内部线程触发，实现须尽快返回，不可阻塞。
 */
typedef void (*xg_depth_cb_t)(const xg_depth_frame_t* frame, void* user_data);

/**
 * @brief  订阅深度图数据(AI 推理产出, 目前仅中间相机支持)
 * @param  dev        设备句柄
 * @param  camera_id  相机标识（当前仅 XG_CAMERA_CENTER 有数据）
 * @param  cb         回调函数，不可为空
 * @param  user_data  透传给回调的用户指针，可为空
 * @return XG_OK 订阅成功；XG_ERR_INVALID_ARG 参数非法；
 *         XG_ERR_REJECTED 该相机已有活跃订阅（须先取消）
 * @note   设备未开启 AI 推理功能时不会有数据推送，订阅本身仍返回 XG_OK。
 *         设备端默认配置 lixel_t1xb.camera_center.ai.fps 为 0，此时设备不发布
 *         depth/center；须设备端把该项配置为大于 0 且 AI 模型加载成功后才有数据。
 *         回调中 depth_data 指针仅在回调执行期间有效。
 */
XG_API xg_err_t xg_subscribe_depth(xg_device_t* dev, xg_camera_id_t camera_id,
                                   xg_depth_cb_t cb, void* user_data);

/**
 * @brief  取消深度图订阅
 * @param  dev       设备句柄
 * @param  camera_id 相机标识
 * @return XG_OK 取消成功（幂等）
 */
XG_API xg_err_t xg_unsubscribe_depth(xg_device_t* dev,
                                     xg_camera_id_t camera_id);


/*=============================================================================
 * 传感器标定参数查询
 *===========================================================================*/

/** 相机标定参数 */
typedef struct {
    size_t struct_size;    /**< 必须由调用方填写为 sizeof(xg_camera_calib_t) */
    uint32_t width;       /**< 图像宽度 */
    uint32_t height;      /**< 图像高度 */
    float fx;             /**< 焦距 X（像素） */
    float fy;             /**< 焦距 Y（像素） */
    float cx;             /**< 主点 X */
    float cy;             /**< 主点 Y */
    float distortion[4];  /**< 畸变系数（kb4: k1,k2,k3,k4; pinhole: k1,k2,p1,p2） */
    char model[32];       /**< 标定模型（"kb4" 或 "pinhole"） */
    float pose[16];       /**< 相对 cam0 的 4x4 变换矩阵（行优先） */
    uint8_t calibrated;   /**< 是否已标定 */
    uint8_t reserved[3];
} xg_camera_calib_t;

/**
 * @brief  查询指定相机的标定参数
 * @param  dev        设备句柄
 * @param  camera_id  相机原始索引（0~3），对应设备标定文件中的 camera_N
 * @param  out        输出标定参数，须先填好 struct_size
 * @return XG_OK 成功；XG_ERR_TIMEOUT 设备未应答
 * @note   此索引为设备标定文件中的原始编号，与 xg_subscribe_image 的
 *         xg_camera_id_t 不是同一套映射。具体哪个索引对应哪个物理相机
 *         取决于设备型号。
 */
XG_API xg_err_t xg_get_camera_calib(xg_device_t* dev, int camera_id,
                                    xg_camera_calib_t* out);

/** IMU 标定参数 */
typedef struct {
    size_t struct_size;        /**< 必须由调用方填写为 sizeof(xg_imu_calib_t) */
    float accel_bias[3];       /**< 加速度计零偏 Ba[3] */
    float gyro_bias[3];        /**< 陀螺仪零偏 Bg[3] */
    float accel_transform[9];  /**< 加速度计变换矩阵 Ta[3x3]（行优先） */
    float accel_scale[9];      /**< 加速度计尺度矩阵 Ka[3x3]（行优先） */
    float gyro_transform[9];   /**< 陀螺仪变换矩阵 Tg[3x3]（行优先） */
    float gyro_scale[9];       /**< 陀螺仪尺度矩阵 Kg[3x3]（行优先） */
    /**
     * T_imu_lidar：Lidar→IMU 的 4x4 齐次变换矩阵（行优先）。
     * 命名约定从右往左读：extrinsic_imu_lidar 表示将 Lidar 坐标系下的点
     * 变换到 IMU 坐标系。数据来源为设备标定文件 extrinsic_imu_lidar.yaml。
     */
    float extrinsic_imu_lidar[16];
    uint8_t calibrated;        /**< 是否已标定 */
    uint8_t reserved[3];
} xg_imu_calib_t;

/**
 * @brief  查询 IMU 标定参数
 * @param  dev    设备句柄
 * @param  out    输出 IMU 标定参数，须先填好 struct_size
 * @return XG_OK 成功；XG_ERR_TIMEOUT 设备未应答
 */
XG_API xg_err_t xg_get_imu_calib(xg_device_t* dev, xg_imu_calib_t* out);

/** 雷达标定参数 */
typedef struct {
    size_t struct_size;             /**< 必须由调用方填写为 sizeof(xg_lidar_calib_t) */
    /**
     * T_imu_lidar：Lidar→IMU 的 4x4 齐次变换矩阵（行优先）。
     * 命名约定从右往左读：extrinsic_imu_lidar 表示将 Lidar 坐标系下的点
     * 变换到 IMU 坐标系。数据来源为设备标定文件 extrinsic_imu_lidar.yaml。
     */
    float extrinsic_imu_lidar[16];
    /**
     * T_cam0_lidar：Lidar→Cam0 的 4x4 齐次变换矩阵（行优先）。
     * 命名约定从右往左读：extrinsic_camera_lidar 表示将 Lidar 坐标系下的点
     * 变换到 Cam0 坐标系。数据来源为设备标定文件 extrinsic_camera_lidar.yaml。
     */
    float extrinsic_camera_lidar[16];
    char lidar_type[64];           /**< 雷达型号 */
    uint8_t calibrated;            /**< 是否已标定 */
    uint8_t reserved[3];
} xg_lidar_calib_t;

/**
 * @brief  查询雷达标定参数
 * @param  dev    设备句柄
 * @param  out    输出雷达标定参数，须先填好 struct_size
 * @return XG_OK 成功；XG_ERR_TIMEOUT 设备未应答
 */
XG_API xg_err_t xg_get_lidar_calib(xg_device_t* dev, xg_lidar_calib_t* out);

/*=============================================================================
 * 设备状态查询
 *===========================================================================*/

/** 设备状态 */
typedef struct {
    size_t struct_size;     /**< 必须由调用方填写为 sizeof(xg_device_info_t) */
    float battery_level;   /**< 电池电量（百分比） */
    float battery_temp;    /**< 电池温度（摄氏度） */
    float battery_voltage; /**< 电池电压（V） */
    int64_t disk_total_kb; /**< 存储总量（KB） */
    int64_t disk_used_kb;  /**< 存储已用（KB） */
    int64_t mem_total_kb;  /**< 内存总量（KB） */
    int64_t mem_used_kb;   /**< 内存已用（KB） */
    uint8_t rtk_state;     /**< RTK 状态 */
    uint8_t satellite_num; /**< 卫星数 */
    int32_t modeling_state;/**< 建图状态码（内部状态，仅供参考） */
} xg_device_info_t;

/**
 * @brief  查询设备状态
 * @param  dev    设备句柄
 * @param  out    输出设备状态，不可为空，须先填好 struct_size
 * @return XG_OK 成功；XG_ERR_TIMEOUT 设备未应答
 */
XG_API xg_err_t xg_get_device_info(xg_device_t* dev, xg_device_info_t* out);

/*=============================================================================
 * 设备时间查询（只读）
 *===========================================================================*/
/**
 * 设备系统时钟快照及本机往返时刻。
 *
 * 量纲：所有 *_epoch_us 均为自 1970-01-01 00:00:00 UTC 起的微秒（Unix epoch），
 * 与各订阅数据的 timestamp_sec 同量纲：timestamp_sec * 1e6 可直接与
 * device_epoch_us 相减。但「同量纲」不等于「同一个时钟」：哪些订阅数据取自
 * 设备系统时钟、哪些取自 MCU / 雷达时钟，见 api_reference_v2.md §0.8，
 * 只有取自设备系统时钟的数据才能直接与 device_epoch_us 比较。
 */
typedef struct {
    size_t struct_size;        /**< 必须由调用方填写为 sizeof(xg_device_time_t) */
    int64_t device_epoch_us;   /**< 设备系统时钟（CLOCK_REALTIME），Unix epoch 微秒。
                                    设备在状态查询完成后、序列化应答之前采样 */
    int32_t utc_offset_s;      /**< 设备本地时区相对 UTC 的偏移（秒，含夏令时）。
                                    只用于显示本地时间，与 epoch 值无关 */
    int32_t reserved;          /**< 保留，SDK 写 0，集成方须忽略 */
    int64_t host_send_epoch_us;/**< 本机发出查询时的 system_clock（Unix epoch 微秒） */
    int64_t host_recv_epoch_us;/**< 本机收到应答时的 system_clock（Unix epoch 微秒） */
    int64_t rtt_us;            /**< 本机用 steady_clock 测得的往返时延（微秒） */
} xg_device_time_t;

/**
 * @brief  查询设备系统时间（只读，不修改设备时间）
 * @param  dev 设备句柄
 * @param  out 输出，不可为空，须先填好 struct_size
 * @return XG_OK 成功；
 *         XG_ERR_INVALID_ARG dev 或 out 为空，或 struct_size 不匹配；
 *         XG_ERR_TIMEOUT 未收到应答；
 *         XG_ERR_PROTOCOL 应答无法解析，或设备固件不支持本查询（应答里没有
 *         time 字段，旧固件即是如此），或 epoch_us / utc_offset_s 缺失、
 *         不是 int64 范围内的整数、超出合法范围
 * @note   与设备时钟的偏差只能得到一个区间：设备采样时刻位于本机
 *         [host_send_epoch_us, host_recv_epoch_us] 之内，因此
 *         offset = 设备时钟 - 本机时钟 满足
 *         device_epoch_us - host_recv_epoch_us <= offset
 *                                              <= device_epoch_us - host_send_epoch_us
 *         区间宽度等于 host_recv_epoch_us - host_send_epoch_us，比 rtt_us 多出
 *         几次时钟读取的开销（微秒量级），不依赖去程与回程对称。本机
 *         system_clock 在查询期间被调整（NTP 跳变等）时，区间无意义。
 * @note   设备应答的是 getDeviceStatus，设备状态查询失败时仍会带回时间，
 *         本接口此时返回 XG_OK。
 * @note   线程：同步阻塞调用线程，最长 3.5 s（查询超时 3 s 加 0.5 s 余量）；
 *         同一句柄不可重入。
 * @note   失败后状态：除 struct_size 不匹配外，返回非 XG_OK 时 out 除 struct_size
 *         外全部置 0，不得使用；struct_size 不匹配时无法确定 out 的大小，out 保持
 *         原样（C++ 封装的 getDeviceTime 失败时一律把 DeviceTime 清零）。
 * @note   禁止用法：把 device_epoch_us 当作 MCU 或雷达时钟读数使用；用本接口
 *         修改设备时间（本接口只读，SDK 不提供设置设备时间的能力）。
 */
XG_API xg_err_t xg_get_device_time(xg_device_t* dev, xg_device_time_t* out);

/*=============================================================================
 * 定位初始位姿设置
 *===========================================================================*/

/** 定位初始位姿参数 */
typedef struct {
    size_t struct_size;   /**< 必须由调用方填写为 sizeof(xg_init_pose_t) */
    double timestamp_sec; /**< 时间戳（秒） */
    float pos_x;          /**< 位置 X (m) */
    float pos_y;          /**< 位置 Y (m) */
    float pos_z;          /**< 位置 Z (m) */
    float quat_x;         /**< 四元数 X */
    float quat_y;         /**< 四元数 Y */
    float quat_z;         /**< 四元数 Z */
    float quat_w;         /**< 四元数 W */
} xg_init_pose_t;

/**
 * @brief  设置定位初始位姿，并启动设备端 Locate 算法
 * @param  dev   设备句柄
 * @param  pose  初始位姿参数，不可为空，须先填好 struct_size
 * @return XG_OK 设备已收下初值且 Locate 算法已启动；XG_ERR_TIMEOUT 设备未应答；
 *         XG_ERR_REJECTED 设备拒绝（如四元数模长不合法、定位尚未初始化）
 * @note   设备端 Locate 的时序是：录制启动时完成初始化 -> 本接口喂入初值并启动算法。
 *         设备进入 Recording 时初始化已完成。本接口是启动 Locate 的必经步骤：
 *         调用前设备不发布位姿（xg_subscribe_pose）和 Locate 去畸变点云
 *         （xg_subscribe_device_undist_cloud）。XG_OK 不代表重定位完成。
 *         四元数模长须大于 0.1。
 */
XG_API xg_err_t xg_set_loc_init_pose(xg_device_t* dev,
                                      const xg_init_pose_t* pose);

/*=============================================================================
 * 工程数据路径查询
 *===========================================================================*/

/** 数据路径最大长度 */
#define XG_PATH_MAX 512

/** 工程数据路径查询结果 */
typedef struct {
    size_t struct_size;       /**< 必须由调用方填写为 sizeof(xg_data_path_t) */
    char path[XG_PATH_MAX];  /**< 设备端工程数据目录路径，UTF-8 */
} xg_data_path_t;

/**
 * @brief  查询当前工程的录制数据存储路径
 * @param  dev    设备句柄
 * @param  out    输出路径，不可为空，须先填好 struct_size
 * @return XG_OK 成功；XG_ERR_TIMEOUT 设备未应答
 * @note   录制期间返回当前活跃工程路径；未录制时返回上一次工程路径或空串。
 */
XG_API xg_err_t xg_get_data_path(xg_device_t* dev, xg_data_path_t* out);

/*=============================================================================
 * 设备网络配置（修改设备对外网口的 IP 与网段）
 *
 * 采用「限时确认」流程，超时未确认则设备自动回滚：
 *   1. 在当前能到达设备的地址上调用 xg_set_device_network()；
 *   2. 设备应答后应用新配置，经被修改网口建立的连接随之中断；
 *   3. 关闭旧句柄，用此刻能到达设备的地址重新 xg_device_open()：新 IP
 *      （本机须能到达新网段，SDK 不修改主机网络），或经 USB 等其他网卡
 *      可达的设备地址；
 *   4. 在 confirm_timeout_s 内调用 xg_confirm_device_network()。
 * 经新 IP 确认可证明新地址可达；经其他网卡确认只证明设备在线。
 *===========================================================================*/

/** confirm_timeout_s 的取值范围（秒），传 0 取默认值 */
#define XG_NETWORK_CONFIRM_TIMEOUT_MIN_S 10
#define XG_NETWORK_CONFIRM_TIMEOUT_MAX_S 120
#define XG_NETWORK_CONFIRM_TIMEOUT_DEFAULT_S 60

/** prefix_len 的取值范围 */
#define XG_NETWORK_PREFIX_MIN 8
#define XG_NETWORK_PREFIX_MAX 30

/**
 * @brief  修改设备对外网口的 IP 与网段（需随后确认，否则自动回滚）
 * @param  dev               设备句柄
 * @param  ip                新 IPv4 地址，点分十进制
 * @param  prefix_len        前缀长度，范围 [XG_NETWORK_PREFIX_MIN, XG_NETWORK_PREFIX_MAX]
 * @param  gateway           NULL 保持现有网关（不在新网段内则设备拒绝）；
 *                           "" 删除默认路由；其他为新网关，须在新网段内且不等于 ip
 * @param  confirm_timeout_s 确认窗口（秒），0 为默认值，否则范围
 *                           [XG_NETWORK_CONFIRM_TIMEOUT_MIN_S, XG_NETWORK_CONFIRM_TIMEOUT_MAX_S]，
 *                           从设备完成地址切换起算
 * @return XG_OK 设备已接受，不代表已生效；XG_ERR_INVALID_ARG 参数非法（发送前
 *         本地校验，设备配置不变）；XG_ERR_TIMEOUT 结果未知，请用新旧地址分别
 *         探测；XG_ERR_REJECTED 设备拒绝，配置不变（扫描中、已有变更待确认、
 *         网段与其他网口重叠等，原因见设备日志）；XG_ERR_PROTOCOL 应答无法
 *         解析或固件不支持本命令
 * @note   同步阻塞，同一句柄不可重入。只作用于设备对外网口。
 * @note   XG_OK 之后只允许对 dev 调用 xg_device_close()：经被修改网口连接时
 *         连接会中断、订阅全部停止；确认请使用重新打开的句柄。
 * @note   对外网口没插网线时，设备无法校验新地址是否生效，只依据 netplan 的
 *         应用结果，插上网线后新地址才会出现。
 * @note   与设备当前配置完全相同时直接视为已确认，连接不中断。
 * @note   确认窗口内设备重启则不再回滚，新配置保留。
 * @warning 确认窗口内禁止再次调用本函数。
 */
XG_API xg_err_t xg_set_device_network(xg_device_t* dev, const char* ip,
                                       int prefix_len, const char* gateway,
                                       int confirm_timeout_s);

/**
 * @brief  确认设备网络配置变更，取消自动回滚
 * @param  dev 设备句柄，须为 xg_set_device_network 成功后重新打开的句柄
 * @return XG_OK 已确认（幂等，重复调用仍返回 XG_OK）；XG_ERR_INVALID_ARG dev 为空；
 *         XG_ERR_TIMEOUT 结果未知，可直接重试；XG_ERR_REJECTED 设备没有可确认的
 *         变更（窗口已过期并回滚、应用失败、从未发起，或设备还没应用完新配置）；
 *         XG_ERR_PROTOCOL 应答无法解析或固件不支持本命令
 * @note   设备在应答 xg_set_device_network 后约 1 秒才应用新配置。经 USB 等不受
 *         影响的连接确认时，确认可能早于应用到达而被拒绝，请在确认窗口内重试。
 * @note   「已确认」「已回滚」的记录只在设备应用进程内存里，应用重启后一律
 *         返回 XG_ERR_REJECTED。
 */
XG_API xg_err_t xg_confirm_device_network(xg_device_t* dev);

/*=============================================================================
 * 错误码订阅
 *===========================================================================*/

/** 单个错误码条目 */
typedef struct {
    uint32_t code;        /**< 错误码 */
    uint64_t timestamp;   /**< 时间戳（微秒） */
} xg_err_code_entry_t;

/** 错误码帧数据 */
typedef struct {
    size_t struct_size;                  /**< 必须由 SDK 填写为 sizeof(xg_err_code_frame_t) */
    const xg_err_code_entry_t* entries; /**< 错误码数组首地址，回调返回后不可再访问 */
    size_t count;                        /**< 错误码数量 */
} xg_err_code_frame_t;

/**
 * @brief 错误码回调函数类型
 * @param frame     错误码帧数据，仅在回调执行期间有效
 * @param user_data 用户在订阅时传入的上下文指针
 * @note  回调在 SDK 内部线程触发，实现须尽快返回，不可阻塞。
 */
typedef void (*xg_err_code_cb_t)(const xg_err_code_frame_t* frame,
                                 void* user_data);

/**
 * @brief  订阅错误码数据
 * @param  dev       设备句柄
 * @param  cb        回调函数，不可为空
 * @param  user_data 透传给回调的用户指针，可为空
 * @return XG_OK 订阅成功；XG_ERR_REJECTED 已有活跃订阅
 * @note   设备以 ~10Hz 批量推送错误码（含定位、建图、SLAM 等模块）。
 *         回调在 SDK 内部线程触发。
 */
XG_API xg_err_t xg_subscribe_err_code(xg_device_t* dev,
                                       xg_err_code_cb_t cb, void* user_data);

/**
 * @brief  取消错误码订阅
 * @param  dev 设备句柄
 * @return XG_OK 取消成功（幂等）
 */
XG_API xg_err_t xg_unsubscribe_err_code(xg_device_t* dev);

/*=============================================================================
 * 重定位地图管理
 *
 * 用于把上位机本机的一整套全局初始化重定位地图上传到设备，供设备端
 * 做全局初始化重定位。集成方只提供本机目录 + mapId，设备端真实存放
 * 路径不暴露。
 *===========================================================================*/

/** mapId 最大长度（含结尾 '\0'） */
#define XG_MAP_ID_MAX 128

/** 地图上传选项（调用方分配，须填 struct_size） */
typedef struct {
    size_t struct_size;       /**< 必须由调用方填写为 sizeof(xg_map_upload_opts_t) */
    const char* local_map_dir; /**< in，非空：本机地图数据根目录（含 location_map.las 与 loc_database/） */
    const char* map_id;        /**< in，非空：地图标识，落盘为 /userdata/app_data/maps/<map_id>/；仅允许 [A-Za-z0-9_-] */
    uint32_t chunk_size;       /**< 分片字节数，0 表示用 SDK 默认（512*1024） */
    uint8_t use_gzip;          /**< 是否 gzip 打包，0=否（默认建议） */
    uint8_t reserved[3];       /**< 对齐填充，保留 */
} xg_map_upload_opts_t;

/**
 * @brief 上传进度回调
 * @param progress  进度 [0.0, 1.0]
 * @param user_data 用户上下文
 */
typedef void (*xg_map_progress_cb_t)(float progress, void* user_data);

/**
 * @brief  上传一整套重定位地图到设备（阻塞至完成或失败，进度经回调上报）
 * @param  dev       设备句柄
 * @param  opts      上传选项，不可为空，须填 struct_size
 * @param  on_progress 进度回调，可为空
 * @param  user_data 透传给进度回调的上下文，可为空
 * @return XG_OK 成功就绪；XG_ERR_INVALID_ARG 参数非法；XG_ERR_NOT_CONNECTED 未连接；
 *         XG_ERR_TIMEOUT 超时；XG_ERR_REJECTED 设备拒绝（磁盘不足/mapId 冲突/校验失败）；
 *         XG_ERR_INTERNAL 本地打包或传输失败
 * @note   上传后需调 xg_set_active_map 激活，再调 xg_set_loc_init_pose 给初值（同时启动 Locate）。
 */
XG_API xg_err_t xg_upload_map(xg_device_t* dev,
                              const xg_map_upload_opts_t* opts,
                              xg_map_progress_cb_t on_progress,
                              void* user_data);

/**
 * @brief  激活指定地图用于重定位（把设备定位路径切到该地图目录）
 * @param  dev    设备句柄
 * @param  map_id 地图标识，不可为空
 * @return XG_OK 成功；XG_ERR_INVALID_ARG 参数非法；XG_ERR_REJECTED 地图不存在；
 *         XG_ERR_TIMEOUT 设备未应答
 */
XG_API xg_err_t xg_set_active_map(xg_device_t* dev, const char* map_id);

/** 单个地图文件条目（SDK 填充，回调/出参缓冲仅本次调用有效） */
typedef struct {
    const char* path;  /**< 相对 <mapId>/ 的路径，如 "loc_database/vlad.proto" */
    int64_t size;      /**< 文件字节数 */
    int64_t mtime;     /**< 修改时间（Unix 秒） */
} xg_map_file_entry_t;

/** 单个地图信息 */
typedef struct {
    char map_id[XG_MAP_ID_MAX];        /**< 地图标识 */
    uint8_t is_active;                 /**< 是否为当前激活地图 */
    uint8_t reserved[7];               /**< 对齐填充 */
    int64_t total_size;                /**< 该地图所有文件总字节数 */
    const xg_map_file_entry_t* files;  /**< 文件条目数组，仅本次调用有效 */
    size_t file_count;                 /**< 文件条目数 */
} xg_map_info_t;

/** 地图列表（SDK 填充，须填 struct_size） */
typedef struct {
    size_t struct_size;          /**< 必须由调用方填写为 sizeof(xg_map_list_t) */
    const xg_map_info_t* maps;   /**< 地图信息数组，回调返回后/下次调用后不可再访问 */
    size_t count;                /**< 地图数量 */
} xg_map_list_t;

/**
 * @brief  查询设备上已有的地图列表（含每个地图的文件层次清单）
 * @param  dev  设备句柄
 * @param  out  输出地图列表，不可为空，须填 struct_size
 * @return XG_OK 成功；XG_ERR_INVALID_ARG 参数非法；XG_ERR_TIMEOUT 设备未应答
 * @note   out->maps 及其内部 files/path 指针指向 SDK 内部缓冲，仅本次调用有效，
 *         下次调用同接口会覆盖。需持久化请在返回后立即深拷贝。
 */
XG_API xg_err_t xg_list_maps(xg_device_t* dev, xg_map_list_t* out);

/**
 * @brief  删除设备上指定地图（回收空间）
 * @param  dev    设备句柄
 * @param  map_id 地图标识，不可为空
 * @return XG_OK 成功（幂等，不存在也返回成功）；XG_ERR_INVALID_ARG 参数非法；
 *         XG_ERR_REJECTED 删除激活中的地图被拒绝；XG_ERR_TIMEOUT 设备未应答
 */
XG_API xg_err_t xg_delete_map(xg_device_t* dev, const char* map_id);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* __XG_ROBOT_SDK_C_H__ */
