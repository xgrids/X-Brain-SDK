cmake_minimum_required(VERSION 3.14)
project(lixel_sdk_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# lixel_sdk: 核心 SDK
# 支持两种目录结构:
#   开发环境: ros2/ 在仓库内, SDK 头文件在 ../include, 库在 ../build-<arch>
#   交付包:   ros2/ 与 include/ lib/ 平级, 库在 ../lib/<arch>
set(LIXEL_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." CACHE PATH "lixel_sdk 根目录")
set(LIXEL_SDK_INCLUDE "${LIXEL_SDK_DIR}/include")

# 归一化架构名：x86_64/amd64 -> x86_64，aarch64/arm64 -> aarch64
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(LIXEL_SDK_ARCH "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
    set(LIXEL_SDK_ARCH "x86_64")
else()
    message(FATAL_ERROR "不支持的架构: ${CMAKE_SYSTEM_PROCESSOR}（仅 x86_64 / aarch64）")
endif()

# 自动检测库路径: 优先开发环境 build-<arch>，否则交付包 lib/<arch>
if(EXISTS "${LIXEL_SDK_DIR}/build-${LIXEL_SDK_ARCH}/liblixel_sdk.so")
    set(LIXEL_SDK_LIB_DIR "${LIXEL_SDK_DIR}/build-${LIXEL_SDK_ARCH}")
elseif(EXISTS "${LIXEL_SDK_DIR}/lib/${LIXEL_SDK_ARCH}/liblixel_sdk.so")
    set(LIXEL_SDK_LIB_DIR "${LIXEL_SDK_DIR}/lib/${LIXEL_SDK_ARCH}")
else()
    message(FATAL_ERROR
        "未找到 liblixel_sdk.so\n"
        "  检查路径: ${LIXEL_SDK_DIR}/build-${LIXEL_SDK_ARCH}/ 或 ${LIXEL_SDK_DIR}/lib/${LIXEL_SDK_ARCH}/\n"
        "  开发环境请先执行 ./build.sh 编译核心 SDK")
endif()

if(NOT EXISTS "${LIXEL_SDK_INCLUDE}/xg/robot_sdk_c.h")
    message(FATAL_ERROR
        "未找到 lixel_sdk 头文件: ${LIXEL_SDK_INCLUDE}/xg/robot_sdk_c.h\n"
        "请确认 LIXEL_SDK_DIR 正确指向 lixel_sdk 根目录")
endif()

add_executable(sdk_bridge_node
    src/sdk_bridge_node.cpp
)

target_include_directories(sdk_bridge_node PRIVATE
    ${LIXEL_SDK_INCLUDE}
    ${EIGEN3_INCLUDE_DIRS}
)

# 链接 lixel_sdk 动态库
target_link_directories(sdk_bridge_node PRIVATE ${LIXEL_SDK_LIB_DIR})
target_link_libraries(sdk_bridge_node lixel_sdk)

ament_target_dependencies(sdk_bridge_node
    rclcpp
    nav_msgs
    sensor_msgs
    geometry_msgs
    tf2_msgs
    tf2_ros
    diagnostic_msgs
)

install(TARGETS sdk_bridge_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
