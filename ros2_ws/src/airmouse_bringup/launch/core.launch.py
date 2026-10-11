"""AirMouse core stack, shared by simulation and the real drone.

FAST-LIO2 -> pose_bridge -> (mapper, commander, survivor_detector, gcs_server) + MAVROS.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    use_sim = arg('use_sim_time').lower() == 'true'
    params = [arg('params_file')]
    params += [p for p in (arg('extra_params'), arg('arena_params')) if p]
    params.append({'use_sim_time': use_sim})

    def airmouse(executable):
        return Node(package='airmouse', executable=executable, name=executable,
                    parameters=params, output='screen', emulate_tty=True)

    actions = [
        Node(package='fast_lio', executable='fastlio_mapping', name='fast_lio',
             parameters=[arg('fast_lio_config'), {'use_sim_time': use_sim}], output='screen'),
        airmouse('pose_bridge'),
        airmouse('mapper'),
        airmouse('commander'),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(get_package_share_directory('mavros'), 'launch', 'apm.launch')),
            launch_arguments={'fcu_url': arg('fcu_url'), 'gcs_url': arg('gcs_url'),
                              'tgt_system': '1', 'tgt_component': '1'}.items()),
    ]
    if arg('detector').lower() == 'true':
        actions.append(airmouse('survivor_detector'))
    if arg('gcs').lower() == 'true':
        actions.append(airmouse('gcs_server'))
    return actions


def generate_launch_description():
    share = get_package_share_directory('airmouse_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'airmouse.yaml')),
        DeclareLaunchArgument('extra_params', default_value='', description='YAML applied after params_file'),
        DeclareLaunchArgument('arena_params', default_value='', description='YAML with arena.* overrides'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('fast_lio_config', default_value=os.path.join(share, 'config', 'fast_lio', 'mid360.yaml')),
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyAMA0:921600'),
        DeclareLaunchArgument('gcs_url', default_value=''),
        DeclareLaunchArgument('detector', default_value='true'),
        DeclareLaunchArgument('gcs', default_value='true'),
        OpaqueFunction(function=_nodes),
    ])
