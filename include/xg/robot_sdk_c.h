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
 *===========================================================================*/

#define XG_SDK_VERSION_MAJOR 1
#define XG_SDK_VERSION_MINOR 0
#define XG_SDK_VERSION_PATCH 0

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
 * @brief  取 SDK 版本号
 * @return 形如 "1.0.0" 的静态字符串，不需释放
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
 * @brief  订阅点云数据
 * @param  dev       设备句柄
 * @param  cb        回调函数，不可为空
 * @param  user_data 透传给回调的用户指针，可为空
 * @return XG_OK 订阅成功；XG_ERR_INVALID_ARG 参数非法；
 *         XG_ERR_REJECTED 已有活跃的点云订阅（须先取消）
 * @note   同一句柄同时只允许一个点云订阅。若需多个消费者，由集成方在
 *         回调中自行分发。
 * @note   回调在 SDK 内部线程（Zenoh 会话线程）触发，不在调用方线程。
 *         回调中传递的 frame 指针仅在回调执行期间有效。
 */
XG_API xg_err_t xg_subscribe_cloud(xg_device_t* dev,
                                   xg_cloud_cb_t cb, void* user_data);

/**
 * @brief  取消点云订阅
 * @param  dev 设备句柄
 * @return XG_OK 取消成功（幂等，无活跃订阅时也返回成功）；
 *         XG_ERR_INVALID_ARG dev 为空
 * @note   返回后保证不再触发点云回调。
 */
XG_API xg_err_t xg_unsubscribe_cloud(xg_device_t* dev);

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
 * @brief  订阅原始点云数据
 * @param  dev       设备句柄
 * @param  cb        回调函数，不可为空
 * @param  user_data 透传给回调的用户指针，可为空
 * @return XG_OK 订阅成功；XG_ERR_REJECTED 已有活跃订阅
 * @note   回调在 SDK 内部线程触发，频率取决于雷达帧率（约 10Hz）。
 * @note   该点云为 SLAM **去畸变后的单帧扫描**（已做运动补偿），但**未经建图
 *         配准/拼接**，处于传感器帧坐标系；与 xg_subscribe_cloud 的 reloc
 *         配准点云（NDT 产物、设备/IMU 系）语义不同。
 */
XG_API xg_err_t xg_subscribe_raw_cloud(xg_device_t* dev,
                                       xg_cloud_cb_t cb, void* user_data);

/**
 * @brief  取消原始点云订阅
 * @param  dev 设备句柄
 * @return XG_OK 取消成功（幂等）
 */
XG_API xg_err_t xg_unsubscribe_raw_cloud(xg_device_t* dev);

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
 * @note   回调频率取决于设备端 JPEG 编码配置（默认 2fps）。
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
 * @brief  设置定位初始位姿（用于重定位）
 * @param  dev   设备句柄
 * @param  pose  初始位姿参数，不可为空，须先填好 struct_size
 * @return XG_OK 设备已接受；XG_ERR_TIMEOUT 设备未应答；
 *         XG_ERR_REJECTED 设备拒绝（如四元数模长不合法）
 * @note   当设备需要在已有地图上重新定位时，可通过本接口提供初始位姿估计，
 *         帮助定位算法加速收敛。四元数须满足模长 > 0.1 的约束。
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
 * @note   上传后需调 xg_set_active_map 激活，再（可选）xg_set_loc_init_pose 给初值。
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
