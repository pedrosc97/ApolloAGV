"""Publish the AGV model: robot_state_publisher only (something else must publish joint_states).

    model:=apollo_agv    the full model (default)
    model:=mock_apollo   the basic stand-in used in simulation
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    urdf_dir = os.path.join(get_package_share_directory('apollo_agv_description'), 'urdf')
    xacro_file = [urdf_dir, os.sep, LaunchConfiguration('model'), '.urdf.xacro']
    robot_description = ParameterValue(Command(['xacro ', *xacro_file]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='apollo_agv', description='apollo_agv or mock_apollo'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
    ])
