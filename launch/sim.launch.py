"""Gazebo Harmonic world + rover with lidar, depth camera, IMU, GPS + ROS 2 bridge + RViz."""
import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

START_X, START_Y = -8.0, 0.0  # centre of zone A


def generate_launch_description():
    pkg = get_package_share_directory('gps_nav_demo')

    # Robot: xacro -> URDF string
    robot_description = xacro.process_file(
        os.path.join(pkg, 'urdf', 'rover.urdf.xacro')).toxml()

    # World: xacro -> plain SDF file that Gazebo can open
    world_sdf = os.path.join(tempfile.gettempdir(), 'gps_nav_world.sdf')
    with open(world_sdf, 'w') as f:
        f.write(xacro.process_file(
            os.path.join(pkg, 'worlds', 'nav_world.sdf.xacro')).toprettyxml(indent='  '))

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v 2 {world_sdf}',
                          'on_exit_shutdown': 'true'}.items())

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen')

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'rover',
                   '-x', str(START_X), '-y', str(START_Y), '-z', '0.02'])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': os.path.join(pkg, 'config', 'bridge.yaml'),
                     'use_sim_time': True}])

    rviz_cfg = os.path.join(pkg, 'rviz', 'sim.rviz')
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', rviz_cfg] if os.path.exists(rviz_cfg) else [],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        # Qt GUIs (Gazebo, RViz) are unreliable on native Wayland -> force XWayland
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        gz_sim, rsp, spawn, bridge, rviz,
    ])
