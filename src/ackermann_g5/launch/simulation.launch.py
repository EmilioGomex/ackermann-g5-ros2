import csv
import math
from pathlib import Path
from shlex import quote

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    share = Path(get_package_share_directory('ackermann_g5'))
    gz_share = Path(get_package_share_directory('ros_gz_sim'))
    slam = LaunchConfiguration('slam').perform(context).lower() == 'true'
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    gazebo_args = f'-r -v 3 {quote(str(share / "worlds" / "arena.sdf"))}'
    if not gui:
        gazebo_args = f'-s {gazebo_args}'
        if slam:
            gazebo_args += ' --headless-rendering'

    description = xacro.process_file(
        str(share / 'urdf' / 'ackermann_g5.urdf.xacro'),
        mappings={'lidar': str(slam).lower(), 'mesh_dir': str(share / 'meshes')},
    ).toxml()
    with (share / 'config' / 'umbrella_path.csv').open(newline='') as file:
        rows = csv.reader(file)
        x, y = map(float, next(rows))
        next_x, next_y = map(float, next(rows))
    yaw = math.atan2(next_y - y, next_x - x)

    bridges = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/g5/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/g5/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/g5/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/g5/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        '/g5/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
    ]
    if slam:
        bridges.append('/g5/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan')

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gz_share / 'launch' / 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gazebo_args}.items(),
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': description, 'use_sim_time': True}],
        ),
        Node(
            package='ros_gz_sim', executable='create',
            arguments=['-world', 'g5_arena', '-topic', '/robot_description',
                       '-name', 'ackermann_g5', '-z', '0.003',
                       '-x', str(x), '-y', str(y), '-Y', str(yaw)],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            arguments=bridges, remappings=[('/g5/tf', '/tf')], output='screen',
        ),
        Node(
            package='ackermann_g5', executable='follow_path.py',
            condition=IfCondition(LaunchConfiguration('controller')),
            parameters=[{
                'use_sim_time': True,
                'start_delay': ParameterValue(LaunchConfiguration('start_delay'), value_type=float),
            }],
            output='screen',
        ),
    ]
    if slam:
        actions.append(Node(
            package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', parameters=[str(share / 'config' / 'slam.yaml')],
            output='screen',
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('slam', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('controller', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_delay', default_value='5.0'),
        OpaqueFunction(function=setup),
    ])
