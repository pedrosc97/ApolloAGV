"""Whole stack against the 2D simulator: model + simulator + follower (+ Foxglove bridge).

    ros2 launch apollo_agv_bringup sim2d.launch.py                 # then open ws://localhost:8765 in Foxglove
    ros2 launch apollo_agv_bringup sim2d.launch.py foxglove:=false follow:=false   # simulator only, drive it yourself
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description = get_package_share_directory('apollo_agv_description')
    bringup = get_package_share_directory('apollo_agv_bringup')
    with open(os.path.join(description, 'config', 'geometry.yaml')) as f:
        g = yaml.safe_load(f)
    geometry = {
        'wheel_radius': g['wheel']['radius'], 'wheel_separation': g['wheel']['separation'],
        'base_length': g['base']['length'], 'base_width': g['base']['width'],
        'ultrasonic_fov': g['ultrasonic']['fov'], 'ultrasonic_min_range': g['ultrasonic']['min_range'],
        'ultrasonic_max_range': g['ultrasonic']['max_range'],
    }
    return LaunchDescription([
        DeclareLaunchArgument('follow', default_value='true', description='start the follow controller'),
        DeclareLaunchArgument('foxglove', default_value='true', description='start the Foxglove bridge on port 8765'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(description, 'launch', 'description.launch.py'))),
        Node(package='apollo_agv_sim', executable='sim2d', output='screen',
             parameters=[os.path.join(bringup, 'config', 'sim2d.yaml'), geometry]),
        Node(package='apollo_agv_follow', executable='follower', output='screen',
             parameters=[os.path.join(bringup, 'config', 'follow.yaml')],
             condition=IfCondition(LaunchConfiguration('follow'))),
        Node(package='foxglove_bridge', executable='foxglove_bridge', output='log',
             parameters=[{'port': 8765, 'address': '0.0.0.0'}],
             condition=IfCondition(LaunchConfiguration('foxglove'))),
    ])
