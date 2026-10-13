"""On the Raspberry Pi. For now only the model and the follower start: there are no hardware drivers yet
(motors, ring stepper, ultrasonics, RealSense), so nothing moves. Drivers get added here as they are written."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    description = get_package_share_directory('apollo_agv_description')
    bringup = get_package_share_directory('apollo_agv_bringup')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(description, 'launch', 'description.launch.py'))),
        Node(package='apollo_agv_follow', executable='follower', output='screen',
             parameters=[os.path.join(bringup, 'config', 'follow.yaml')]),
    ])
