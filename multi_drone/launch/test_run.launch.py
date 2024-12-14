# !/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    pkg_multi_drone = get_package_share_directory('multi_drone')
        
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_multi_drone, 'launch', 'test_gazebo.launch.py'])
        ),
    )

    # Запуск multi_drones с задержкой
    multi_drone_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_multi_drone, 'launch', 'test_multi_drone_run.launch.py'])
                ),
            )
        ]
    )
    
    return LaunchDescription([
        gazebo_launch,
        multi_drone_launch,
    ])

if __name__ == "__main__":
    generate_launch_description()
