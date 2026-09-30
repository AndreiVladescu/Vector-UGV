"""GNSS localization and the GPS waypoint follower (config in vector_nav/config/localization.yaml).

Included by robot.launch.py and sim_gazebo.launch.py with nav:=true. Needs gnss/fix, imu
(with an absolute heading) and odom/legs from gait_node. Send a mission with:
  ros2 run vector_nav waypoints.py send 44.43510,26.10310 44.43520,26.10290
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('vector_nav'), 'config', 'localization.yaml')
    params = [config, {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]
    declination = {'magnetic_declination_radians':
                   ParameterValue(LaunchConfiguration('declination'), value_type=float)}

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cmd_topic', default_value='cmd_vel',
                              description='cmd_vel/nav when the link manager runs (links:=true)'),
        DeclareLaunchArgument('declination', default_value='0.105',
                              description='rad, east positive; 0 in Gazebo, whose IMU gives true heading'),

        Node(package='robot_localization', executable='ekf_node', name='ekf_odom', parameters=params,
             remappings=[('odometry/filtered', 'odometry/local')]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_map', parameters=params,
             remappings=[('odometry/filtered', 'odometry/global')]),
        Node(package='robot_localization', executable='navsat_transform_node', name='navsat_transform',
             parameters=params + [declination],
             remappings=[('gps/fix', 'gnss/fix'), ('gps/filtered', 'gnss/filtered'),
                         ('odometry/filtered', 'odometry/global')]),
        Node(package='vector_nav', executable='waypoints.py', name='waypoint_follower', parameters=params,
             remappings=[('cmd_vel', LaunchConfiguration('cmd_topic'))]),
        Node(package='vector_nav', executable='obstacles.py', name='obstacles', parameters=params),
    ])
