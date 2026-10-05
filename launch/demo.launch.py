"""ONE command: Gazebo + rover + localization (EKF + GPS) + Nav2 + GPS waypoint mission.

    ros2 launch gps_nav_demo demo.launch.py            # full autonomous run A -> B
    ros2 launch gps_nav_demo demo.launch.py record:=true   # + rosbag
"""
import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('gps_nav_demo')
    loc_yaml = os.path.join(pkg, 'config', 'localization.yaml')
    sim_time = {'use_sim_time': True}

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')))

    ekf_local = Node(
        package='robot_localization', executable='ekf_node', name='ekf_local', output='screen',
        parameters=[loc_yaml, sim_time],
        remappings=[('odometry/filtered', '/odometry/local')])

    ekf_global = Node(
        package='robot_localization', executable='ekf_node', name='ekf_global', output='screen',
        parameters=[loc_yaml, sim_time],
        remappings=[('odometry/filtered', '/odometry/global')])

    navsat = Node(
        package='robot_localization', executable='navsat_transform_node', name='navsat_transform',
        output='screen', parameters=[loc_yaml, sim_time],
        remappings=[('imu', '/imu'), ('gps/fix', '/gps/fix'),
                    ('odometry/filtered', '/odometry/global'),
                    ('odometry/gps', '/odometry/gps'), ('gps/filtered', '/gps/filtered')])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'true', 'autostart': 'true',
                          'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml')}.items())

    mission = Node(
        package='gps_nav_demo', executable='gps_waypoint_follower.py', output='screen',
        parameters=[{'waypoints_file': os.path.join(pkg, 'config', 'waypoints.yaml')}, sim_time])

    bag_dir = os.path.expanduser('~/gps_nav_bags')
    os.makedirs(bag_dir, exist_ok=True)
    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o',
             os.path.join(bag_dir, f'run_{time.strftime("%Y%m%d_%H%M%S")}'),
             '/clock', '/tf', '/tf_static', '/odom', '/odometry/local', '/odometry/global',
             '/odometry/gps', '/gps/fix', '/imu', '/scan', '/cmd_vel', '/plan',
             '/waypoint_markers', '/robot_description'],
        output='screen', condition=IfCondition(LaunchConfiguration('record')))

    return LaunchDescription([
        DeclareLaunchArgument('record', default_value='false'),
        sim, ekf_local, ekf_global, navsat,
        TimerAction(period=5.0, actions=[nav2]),       # let the EKF snap to GPS first
        TimerAction(period=12.0, actions=[mission, bag]),
    ])
