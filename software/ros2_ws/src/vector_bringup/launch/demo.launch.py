"""Everything the operator page shows, without hardware: mock legs and the gait, the link
manager and the page, and the carrier's IO MCU simulated on a pseudo-terminal (sim_io.py)
behind the real io_bridge, plus imu_node on simulated chips, and the nav stack (EKFs,
navsat_transform, the waypoint follower), so missions and the walk home on a lost link run
too. Walking from the page moves the GNSS position, the heading, the lidar's view of a yard
and the battery current.

  ros2 launch vector_bringup demo.launch.py [rviz:=true] [gnss_scale:=10]
  then http://localhost:8080 (or http://<this machine>:8080 from a phone on the same network)

Break things while it runs:
  ros2 param set /sim_io radio false      # also lidar, gnss, lora
  ros2 param set /sim_io vbat 13.3        # low battery
  ros2 param set /sim_io gnss_scale 20    # walk across the map faster
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

LINK = '/tmp/vector-io-sim'


def generate_launch_description():
    bringup = get_package_share_directory('vector_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('gnss_scale', default_value='1.0', description='GNSS distance per metre walked'),
        DeclareLaunchArgument('nav', default_value='true', description='EKFs, navsat_transform, waypoint follower'),
        DeclareLaunchArgument('can_interface', default_value='vcan0',
                              description="for the page's calibration wizard: sim_legs there answers it"),
        Node(package='vector_io', executable='sim_io.py', name='sim_io', output='screen',
             parameters=[{'link': LINK,
                          'gnss_scale': ParameterValue(LaunchConfiguration('gnss_scale'), value_type=float)}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'robot.launch.py')),
            launch_arguments={'hardware': 'mock', 'rviz': LaunchConfiguration('rviz'), 'links': 'true',
                              'nav': LaunchConfiguration('nav'), 'can_interface': LaunchConfiguration('can_interface'),
                              'io_port': LINK, 'elrs_port': 'io', 'imu': 'true', 'imu_bus': 'sim'}.items()),
    ])
