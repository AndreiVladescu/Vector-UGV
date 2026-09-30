"""Robot bring-up without Gazebo: ros2_control, gait node, optional rviz.

  ros2 launch vector_bringup robot.launch.py                       # mock joints (kinematic sim)
  ros2 launch vector_bringup robot.launch.py hardware:=can          # leg nodes on can0
  ros2 launch vector_bringup robot.launch.py hardware:=can can_interface:=vcan0   # sim_legs
  gnss_port:=/dev/ttyAMA0 [gnss_baud:=115200]   NMEA GNSS on a UART: gnss/fix, gnss/vel
  nav:=true                                     EKFs, navsat_transform, waypoint follower
  links:=true [elrs_port:=/dev/ttyAMA2] [lte_at_port:=/dev/ttyUSB2]
      the link manager owns cmd_vel (radio > cmd_vel/teleop > cmd_vel/nav) and sits the robot
      down when every link is gone; teleop then goes to cmd_vel/teleop

Drive it with: ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(__file__))
from common import geometry_params  # noqa: E402


def generate_launch_description():
    desc = get_package_share_directory('vector_description')
    bringup = get_package_share_directory('vector_bringup')
    gait = get_package_share_directory('vector_gait')

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(desc, 'urdf', 'vector.urdf.xacro'),
                 ' hardware:=', LaunchConfiguration('hardware'),
                 ' can_interface:=', LaunchConfiguration('can_interface')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('hardware', default_value='mock', description='mock or can'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('nav', default_value='false'),
        DeclareLaunchArgument('gnss_port', default_value='', description='serial port of an NMEA receiver'),
        DeclareLaunchArgument('gnss_baud', default_value='115200'),
        DeclareLaunchArgument('links', default_value='false'),
        DeclareLaunchArgument('elrs_port', default_value='', description='UART of the ExpressLRS receiver'),
        DeclareLaunchArgument('lte_at_port', default_value='', description='AT port of the LTE modem'),
        DeclareLaunchArgument('page_key', default_value='', description='?key= the operator page asks for'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[os.path.join(bringup, 'config', 'controllers.yaml')],
            remappings=[('~/robot_description', '/robot_description')]),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', 'leg_controller', 'leg_power']),

        Node(
            package='vector_gait',
            executable='gait_node',
            parameters=[
                os.path.join(gait, 'config', 'gait.yaml'),
                geometry_params(os.path.join(desc, 'config', 'legs.yaml')),
                # with nav, odom -> base_link comes from the EKF instead
                {'publish_odom_tf': ParameterValue(PythonExpression(["'", LaunchConfiguration('nav'), "' != 'true'"]), value_type=bool)}]),

        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='gnss',
            parameters=[{'port': LaunchConfiguration('gnss_port'),
                         'baud': ParameterValue(LaunchConfiguration('gnss_baud'), value_type=int),
                         'frame_id': 'gnss'}],
            remappings=[('fix', 'gnss/fix'), ('vel', 'gnss/vel'), ('heading', 'gnss/heading'),
                        ('time_reference', 'gnss/time')],
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('gnss_port'), "' != ''"]))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'nav.launch.py')),
            launch_arguments={'cmd_topic': PythonExpression(
                ["'cmd_vel/nav' if '", LaunchConfiguration('links'), "' == 'true' else 'cmd_vel'"])}.items(),
            condition=IfCondition(LaunchConfiguration('nav'))),

        Node(
            package='vector_link',
            executable='link_manager.py',
            parameters=[{'lte_at_port': LaunchConfiguration('lte_at_port')}],
            condition=IfCondition(LaunchConfiguration('links'))),
        # obstacles around the robot, for the follower and the operator page's parking HUD
        Node(
            package='vector_nav',
            executable='obstacles.py',
            name='obstacles',
            parameters=[os.path.join(get_package_share_directory('vector_nav'), 'config', 'localization.yaml')],
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('nav'), "' == 'true' or '", LaunchConfiguration('links'), "' == 'true'"]))),

        # the operator page, http://<robot>:8080
        Node(
            package='vector_link',
            executable='operator_page.py',
            parameters=[{'token': LaunchConfiguration('page_key')}],
            condition=IfCondition(LaunchConfiguration('links'))),
        Node(
            package='vector_link',
            executable='elrs.py',
            parameters=[{'port': LaunchConfiguration('elrs_port')}],
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('elrs_port'), "' != ''"]))),

        # faults, supplies and ToF from the legs: /diagnostics and legs/<leg>/tof
        Node(
            package='vector_hw',
            executable='leg_monitor',
            parameters=[{'can_interface': LaunchConfiguration('can_interface')}],
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('hardware'), "' == 'can'"]))),

        # VL53L8CX on the nose board: nose/tof/points and nose/tof/depth
        Node(
            package='vector_tof',
            executable='tof_front',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('hardware'), "' == 'can'"]))),

        Node(
            package='vector_hw',
            executable='power_monitor',
            parameters=[{'can_interface': LaunchConfiguration('can_interface')}],
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('hardware'), "' == 'can'"]))),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(bringup, 'rviz', 'sim.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
