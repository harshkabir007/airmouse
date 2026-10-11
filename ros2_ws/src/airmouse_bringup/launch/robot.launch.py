"""Real drone (Raspberry Pi 5): Livox Mid-360 + USB camera + AirMouse core + MAVROS.

  ros2 launch airmouse_bringup robot.launch.py
  ros2 launch airmouse_bringup robot.launch.py fcu_url:=/dev/ttyACM0:115200 gcs_url:=udp-b://@14550

Web GCS: http://<pi-ip>:8080   Mission Planner: UDP 14550 (via gcs_url)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('airmouse_bringup')
    lc = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyAMA0:921600'),
        DeclareLaunchArgument('gcs_url', default_value='udp-b://@14550'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('livox_config', default_value=os.path.join(share, 'config', 'livox', 'MID360_config.json')),

        Node(package='livox_ros_driver2', executable='livox_ros_driver2_node', name='livox_lidar_publisher',
             output='screen',
             parameters=[{
                 'xfer_format': 1,          # Livox CustomMsg (what FAST-LIO expects for lidar_type 1)
                 'multi_topic': 0,
                 'data_src': 0,
                 'publish_freq': 10.0,
                 'output_data_type': 0,
                 'frame_id': 'livox_frame',
                 'lvx_file_path': '',
                 'user_config_path': lc('livox_config'),
                 'cmdline_input_bd_code': 'livox0000000001',
             }]),

        Node(package='v4l2_camera', executable='v4l2_camera_node', name='camera', output='screen',
             parameters=[{'video_device': lc('video_device'), 'image_size': [640, 480],
                          'output_encoding': 'rgb8', 'camera_frame_id': 'camera_link'}],
             remappings=[('image_raw', '/camera/image_raw'), ('camera_info', '/camera/camera_info')]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'core.launch.py')),
            launch_arguments={
                'use_sim_time': 'false',
                'fast_lio_config': os.path.join(share, 'config', 'fast_lio', 'mid360.yaml'),
                'fcu_url': lc('fcu_url'),
                'gcs_url': lc('gcs_url'),
            }.items()),
    ])
