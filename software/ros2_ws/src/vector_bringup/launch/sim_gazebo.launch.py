"""Physics simulation in Gazebo Harmonic, same controllers and gait node as the robot.

  ros2 launch vector_bringup sim_gazebo.launch.py            # with the Gazebo window
  ros2 launch vector_bringup sim_gazebo.launch.py gui:=false # headless
  world:=flat|slope|rough, level:=true for IMU leveling
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(__file__))
from common import geometry_params  # noqa: E402


def gazebo(context):
    world = os.path.join(get_package_share_directory('vector_bringup'), 'worlds',
                         LaunchConfiguration('world').perform(context) + '.sdf')
    flags = '-r -v 1 ' + ('' if LaunchConfiguration('gui').perform(context) == 'true' else '-s ')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': flags + world, 'on_exit_shutdown': 'true'}.items())]


def generate_launch_description():
    desc = get_package_share_directory('vector_description')
    bringup = get_package_share_directory('vector_bringup')
    gait = get_package_share_directory('vector_gait')
    controllers = os.path.join(bringup, 'config', 'controllers.yaml')
    sim_time = {'use_sim_time': True}

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(desc, 'urdf', 'vector.urdf.xacro'),
                 ' hardware:=gazebo controllers:=', controllers]),
        value_type=str)
    legs = geometry_params(os.path.join(desc, 'config', 'legs.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('world', default_value='flat'),
        DeclareLaunchArgument('level', default_value='false'),

        OpaqueFunction(function=gazebo),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}, sim_time]),

        # Spawn a little above standing height; the joints start in the standing pose.
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'vector',
                       '-z', str(legs['body_height'] + 0.01)]),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/model/vector/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/model/vector/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            ],
            remappings=[('/model/vector/pose', '/tf'),
                        ('/model/vector/odometry', '/ground_truth')]),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', 'leg_controller'],
            parameters=[sim_time]),

        Node(
            package='vector_gait',
            executable='gait_node',
            parameters=[os.path.join(gait, 'config', 'gait.yaml'), legs, sim_time,
                        {'publish_odom_tf': False,
                         'level': ParameterValue(LaunchConfiguration('level'), value_type=bool)}]),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(bringup, 'rviz', 'sim.rviz')],
            parameters=[sim_time],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
