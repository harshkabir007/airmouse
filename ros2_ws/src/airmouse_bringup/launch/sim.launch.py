"""Full software-in-the-loop simulation.

Gazebo Harmonic (maze + drone) + ArduPilot SITL + ros_gz bridge + AirMouse core.

  ros2 launch airmouse_bringup sim.launch.py                 # default maze_01
  ros2 launch airmouse_bringup sim.launch.py maze:=/path/to/layout.txt headless:=true
  ros2 launch airmouse_bringup sim.launch.py maze_scale:=1.0     # exact 1 m corridors

Then open http://localhost:8080 and press START.
Mission Planner (optional): connect TCP 127.0.0.1 port 5762.
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from airmouse_sim import drone_model, maze_gen


def _setup(context):
    maze = LaunchConfiguration('maze').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    home = LaunchConfiguration('sitl_home').perform(context)
    scale = float(LaunchConfiguration('maze_scale').perform(context))
    sim_share = get_package_share_directory('airmouse_sim')
    bringup_share = get_package_share_directory('airmouse_bringup')
    out = os.path.join(tempfile.gettempdir(), 'airmouse')
    os.makedirs(out, exist_ok=True)

    # Drone model (iris_with_ardupilot + our sensors) and Gazebo resource path
    res_paths = [p for p in os.environ.get('GZ_SIM_RESOURCE_PATH', '').split(':') if p]
    models_dir = os.path.join(out, 'models')
    drone_model.build(models_dir, res_paths)
    gz_path = ':'.join([models_dir] + res_paths)

    # World + arena grid from the ASCII layout
    layout = maze if os.path.isfile(maze) else os.path.join(sim_share, 'mazes', f'{maze}.txt')
    name = os.path.splitext(os.path.basename(layout))[0]
    with open(layout) as f:
        world_sdf, arena, survivors = maze_gen.generate(f.read(), name=name, scale=scale)
    world = os.path.join(out, f'{name}.sdf')
    with open(world, 'w') as f:
        f.write(world_sdf)
    arena_file = os.path.join(out, f'{name}_arena.yaml')
    with open(arena_file, 'w') as f:
        f.write(maze_gen.arena_yaml(arena))
    print(f'[airmouse] world {world}, arena {arena}')
    for s in survivors:
        print(f"[airmouse] ground truth survivor {s['id']}: x={s['x']} y={s['y']}")

    # ArduPilot SITL (JSON backend talks to the ardupilot_gazebo plugin)
    ap = os.environ.get('ARDUPILOT_HOME', '/opt/ardupilot')
    defaults = ','.join([
        os.path.join(ap, 'Tools', 'autotest', 'default_params', 'copter.parm'),
        os.path.join(ap, 'Tools', 'autotest', 'default_params', 'gazebo-iris.parm'),
        os.path.join(bringup_share, 'config', 'ardupilot', 'sitl.parm'),
    ])
    sitl_dir = os.path.join(out, 'sitl')
    os.makedirs(sitl_dir, exist_ok=True)
    sitl = ExecuteProcess(
        name='arducopter_sitl', cwd=sitl_dir, output='screen',
        cmd=[os.path.join(ap, 'build', 'sitl', 'bin', 'arducopter'), '--model', 'JSON', '--speedup', '1',
             '--slave', '0', '--defaults', defaults, '--sim-address=127.0.0.1', '-I0', '-w', '--home', home])

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch',
                                                   'gz_sim.launch.py')),
        launch_arguments={'gz_args': ('-s ' if headless else '') + f'-r -v 2 {world}',
                          'on_exit_shutdown': 'true'}.items())

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge', output='screen',
                  parameters=[{'config_file': os.path.join(sim_share, 'config', 'gz_bridge.yaml'),
                               'use_sim_time': True}])
    lidar_filter = Node(package='airmouse_sim', executable='lidar_filter', name='lidar_filter', output='screen',
                        parameters=[{'use_sim_time': True}])

    core =IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'core.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'extra_params': os.path.join(bringup_share, 'config', 'sim.yaml'),
            'arena_params': arena_file,
            'fast_lio_config': os.path.join(bringup_share, 'config', 'fast_lio', 'sim.yaml'),
            'fcu_url': 'tcp://127.0.0.1:5760',
            'gcs_url': '',
        }.items())

    return [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_path), gz, bridge, lidar_filter, sitl,
            TimerAction(period=8.0, actions=[core])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('maze', default_value='maze_01', description='layout name or path to .txt'),
        DeclareLaunchArgument('headless', default_value='false', description='true = no Gazebo GUI'),
        DeclareLaunchArgument('sitl_home', default_value='12.9716,77.5946,900,0'),
        # The SITL iris (0.38 m prop-tip radius) is ~1.5x the real airframe. Scaling the maze
        # by 1.3 keeps the vehicle/corridor ratio of a ~0.29 m-radius drone in 1 m corridors.
        # maze_scale:=1.0 gives the exact competition dimensions.
        DeclareLaunchArgument('maze_scale', default_value='1.3'),
        OpaqueFunction(function=_setup),
    ])
