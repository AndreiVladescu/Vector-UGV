"""View the full Fusion 360 model (vector_cad.urdf.xacro) in rviz, with sliders for the joints.

  ros2 launch vector_bringup view_cad.launch.py

The model is exported with fusion360descriptor; its joint names are still the Fusion ones
(Revolute7, Revolute8_0, ...), not the ones vector.urdf.xacro and vector_gait use.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc = get_package_share_directory('vector_description')
    bringup = get_package_share_directory('vector_bringup')

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(desc, 'urdf', 'vector_cad.urdf.xacro')]),
        value_type=str)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(bringup, 'rviz', 'cad.rviz')]),
    ])
