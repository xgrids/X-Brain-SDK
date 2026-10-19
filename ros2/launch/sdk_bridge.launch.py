from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('lixel_sdk_ros2')
    default_params = os.path.join(pkg_dir, 'config', 'default_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'device_ip',
            default_value='192.168.123.103',
            description='设备 IP 地址'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='参数文件路径'
        ),
        Node(
            package='lixel_sdk_ros2',
            executable='sdk_bridge_node',
            name='sdk_bridge_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'device_ip': LaunchConfiguration('device_ip')}
            ],
        ),
    ])
