from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def _bridge_node(context):
    # 所有参数（设备连接、坐标系、外参、话题名）都从参数文件读。
    # 以前这里总是用 launch 参数 device_ip 覆盖一次，参数文件里的 device_ip
    # 写了也不生效；现在只有命令行显式给了 device_ip:=... 才覆盖。
    parameters = [LaunchConfiguration('params_file').perform(context)]
    device_ip = LaunchConfiguration('device_ip').perform(context).strip()
    if device_ip:
        parameters.append({'device_ip': device_ip})
    return [
        Node(
            package='lixel_ros',
            executable='sdk_bridge_node',
            name='sdk_bridge_node',
            output='screen',
            parameters=parameters,
        ),
    ]


def generate_launch_description():
    pkg_dir = get_package_share_directory('lixel_ros')
    default_params = os.path.join(pkg_dir, 'config', 'default_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='参数文件路径（设备连接、坐标系、外参、话题名都在这里）'
        ),
        DeclareLaunchArgument(
            'device_ip',
            default_value='',
            description='可选：覆盖参数文件里的 device_ip；留空则用参数文件的值'
        ),
        OpaqueFunction(function=_bridge_node),
        # 相机发的是 CompressedImage(jpeg)。RViz 的 Image 显示和多数工具只认
        # sensor_msgs/Image，所以额外解码出一路 image_raw；压缩流保持不变，
        # 需要省带宽的下游可以继续订阅 compressed。
        #
        # 这里没有用 image_transport republish：本机只注册了
        # image_transport/raw_sub，缺 compressed_image_transport 插件，而
        # apt 当前处于损坏状态（无关的 nginx 依赖 libssl1.1）装不上。
        # cv_bridge/cv2 已就绪，所以在本包内解码。
        #
        # 帧率由设备端 JPEG 配置(jpeg.fps)决定（实测 10 Hz），不是本层的限制。
        Node(
            package='lixel_ros',
            executable='jpeg_decoder_node.py',
            name='lixel_jpeg_decoder',
            output='screen',
        ),
    ])
