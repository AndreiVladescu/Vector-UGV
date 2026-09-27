"""Physics simulation in Gazebo Harmonic, same controllers and gait node as the robot.

  ros2 launch vector_bringup sim_gazebo.launch.py            # with the Gazebo window
  ros2 launch vector_bringup sim_gazebo.launch.py gui:=false # headless
  world:=flat|slope|rough, level:=true for IMU leveling, touchdown:=true for contact-aware steps
"""
import os
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(__file__))
from common import geometry_params  # noqa: E402


# `gz sim` starts the server (and GUI) as child processes and ignores a SIGINT sent to itself
# alone, which is what ros2 launch does on Ctrl+C. A leftover server keeps its controller
# manager alive and the next launch's spawner talks to that one instead, so the new robot
# never gets controllers. Run gz in its own process group and signal the whole group.
GZ_WRAPPER = (
    'setsid gz sim "$@" & pid=$!; '
    'trap \'kill -INT -$pid 2>/dev/null; (sleep 5; kill -KILL -$pid 2>/dev/null) & wait $pid\' INT TERM HUP; '
    'wait $pid')


def gazebo(context):
    actions = []
    running = subprocess.run(['pgrep', '-f', 'gz sim .*vector_bringup/worlds/'],
                             capture_output=True, text=True).stdout.split()
    if running:
        actions.append(LogInfo(msg=(
            'WARNING: an older Gazebo for this robot is still running (pid ' + ' '.join(running) +
            '). Stop it first: pkill -f "gz sim"')))

    world = os.path.join(get_package_share_directory('vector_bringup'), 'worlds',
                         LaunchConfiguration('world').perform(context) + '.sdf')
    args = ['-r', '-v', '1'] + ([] if LaunchConfiguration('gui').perform(context) == 'true' else ['-s'])
    gz = ExecuteProcess(cmd=['bash', '-c', GZ_WRAPPER, 'gz'] + args + [world],
                        name='gazebo', output='screen')
    actions.append(gz)
    # closing the Gazebo window ends the whole launch
    actions.append(RegisterEventHandler(OnProcessExit(target_action=gz, on_exit=[EmitEvent(event=Shutdown())])))
    return actions


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
        DeclareLaunchArgument('touchdown', default_value='false'),

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
                         'level': ParameterValue(LaunchConfiguration('level'), value_type=bool),
                         'touchdown': ParameterValue(LaunchConfiguration('touchdown'), value_type=bool)}]),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(bringup, 'rviz', 'sim.rviz')],
            parameters=[sim_time],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
