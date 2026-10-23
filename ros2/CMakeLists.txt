cmake_minimum_required(VERSION 3.14)
project(lixel_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_msgs REQUIRED)

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

# 版本一致性: package.xml 的 <version> 必须等于 SDK 头文件版本
# (唯一来源 XG_SDK_VERSION_MAJOR/MINOR/PATCH)。本目录随交付包独立构建,
# 交付包里没有 lixel_sdk 的 cmake/ 目录, 所以这里自带解析, 规则与
# cmake/LixelSdkVersion.cmake 相同: 每个宏恰好一行 "#define XG_SDK_VERSION_<部件> <十进制数>"。
set(_xg_header "${LIXEL_SDK_INCLUDE}/xg/robot_sdk_c.h")
set(_xg_package_xml "${CMAKE_CURRENT_SOURCE_DIR}/package.xml")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${_xg_header}" "${_xg_package_xml}")

file(STRINGS "${_xg_header}" _xg_ver_lines
     REGEX "^[ \t]*#[ \t]*define[ \t]+XG_SDK_VERSION_(MAJOR|MINOR|PATCH)([ \t\r]|$)")
set(_xg_ver_parts "")
foreach(_xg_part MAJOR MINOR PATCH)
    set(_xg_count 0)
    set(_xg_value "")
    foreach(_xg_line IN LISTS _xg_ver_lines)
        if(_xg_line MATCHES "define[ \t]+XG_SDK_VERSION_${_xg_part}([ \t\r]|$)")
            math(EXPR _xg_count "${_xg_count} + 1")
            if(_xg_line MATCHES
               "^#define XG_SDK_VERSION_${_xg_part}[ \t]+(0|[1-9][0-9]*)[ \t\r]*$")
                set(_xg_value "${CMAKE_MATCH_1}")
            endif()
        endif()
    endforeach()
    if(NOT _xg_count EQUAL 1 OR _xg_value STREQUAL "")
        message(FATAL_ERROR
            "无法从 ${_xg_header} 解析 XG_SDK_VERSION_${_xg_part}: "
            "须恰好有一行 '#define XG_SDK_VERSION_${_xg_part} <十进制数>', 实际找到 ${_xg_count} 行")
    endif()
    list(APPEND _xg_ver_parts "${_xg_value}")
endforeach()
string(JOIN "." LIXEL_SDK_VERSION ${_xg_ver_parts})

file(READ "${_xg_package_xml}" _xg_package_content)
if(NOT _xg_package_content MATCHES "<version>([^<]*)</version>")
    message(FATAL_ERROR "${_xg_package_xml} 中没有 <version> 元素")
endif()
set(_xg_package_version "${CMAKE_MATCH_1}")
if(NOT _xg_package_version STREQUAL LIXEL_SDK_VERSION)
    message(FATAL_ERROR
        "ROS2 包版本与 SDK 头文件版本不一致:\n"
        "  ${_xg_package_xml}: <version>${_xg_package_version}</version>\n"
        "  ${_xg_header}: ${LIXEL_SDK_VERSION}\n"
        "请把 package.xml 的 <version> 改为 ${LIXEL_SDK_VERSION}, "
        "或确认 LIXEL_SDK_DIR 指向配套版本的 SDK")
endif()
message(STATUS "lixel_sdk 版本: ${LIXEL_SDK_VERSION} (与 package.xml 一致)")

add_executable(sdk_bridge_node
    src/sdk_bridge_node.cpp
)

target_include_directories(sdk_bridge_node PRIVATE
    ${LIXEL_SDK_INCLUDE}
)

# 链接 lixel_sdk 动态库
target_link_directories(sdk_bridge_node PRIVATE ${LIXEL_SDK_LIB_DIR})
target_link_libraries(sdk_bridge_node lixel_sdk)

ament_target_dependencies(sdk_bridge_node
    rclcpp
    nav_msgs
    sensor_msgs
    geometry_msgs
    tf2
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

# install(PROGRAMS) 会带上可执行位，launch 通过 libexec 找它时需要这一点。
install(PROGRAMS
    scripts/jpeg_decoder_node.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
