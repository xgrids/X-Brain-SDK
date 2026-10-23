#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# SDK 动态库目录：交付包布局（本目录与 lib/<架构>/ 平级）优先，
# 其次是把库拷到 <工作区>/lixel_sdk/lib 的布局。
case "$(uname -m)" in
    aarch64|arm64) SDK_ARCH=aarch64 ;;
    *)             SDK_ARCH=x86_64 ;;
esac
if [[ -d "${SCRIPT_DIR}/../lib/${SDK_ARCH}" ]]; then
    SDK_LIB_DIR="$(cd -- "${SCRIPT_DIR}/../lib/${SDK_ARCH}" && pwd)"
elif [[ -d "${SCRIPT_DIR}/../lixel_sdk/lib" ]]; then
    SDK_LIB_DIR="$(cd -- "${SCRIPT_DIR}/../lixel_sdk/lib" && pwd)"
else
    echo "未找到 SDK 动态库目录（lib/${SDK_ARCH} 或 ../lixel_sdk/lib）!" >&2
    exit 1
fi
# colcon 安装目录：环境变量 LIXEL_WS 优先；否则依次找本目录（在 ros2/ 内 colcon build）、
# 上一级（工作区布局）。
WORKSPACE_ROOT=""
for cand in "${LIXEL_WS:-}" "${SCRIPT_DIR}" "${SCRIPT_DIR}/.."; do
    if [[ -n "${cand}" && -f "${cand}/install/setup.bash" ]]; then
        WORKSPACE_ROOT="$(cd -- "${cand}" && pwd)"
        break
    fi
done
if [[ -z "${WORKSPACE_ROOT}" ]]; then
    echo "未找到 install/setup.bash，请先 colcon build（见 README），或设置 LIXEL_WS!" >&2
    exit 1
fi
source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# CycloneDDS 配置：环境变量里给了就用给的；否则只在 A2 机器狗的配置文件存在时用它。
# CYCLONEDDS_URI 指向不存在的文件时节点直接建不起来（rmw handle is invalid），
# 所以别的机器上不能无条件设置。
A2_CYCLONEDDS_XML=/home/unitree/slam_config/cyclonedds_go2_B2_ws/cyclonedds.xml
if [[ -z "${CYCLONEDDS_URI:-}" && -f "${A2_CYCLONEDDS_XML}" ]]; then
    export CYCLONEDDS_URI="${A2_CYCLONEDDS_XML}"
fi
export LD_LIBRARY_PATH="${SDK_LIB_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
# 参数全部从参数文件读（默认 install/lixel_ros/share/lixel_ros/config/default_params.yaml）。
# 脚本后面的参数原样传给 launch，例如换一份参数文件：
#   start_lixel_ros.sh params_file:=/path/to/my_params.yaml
exec ros2 launch lixel_ros sdk_bridge.launch.py "$@"
