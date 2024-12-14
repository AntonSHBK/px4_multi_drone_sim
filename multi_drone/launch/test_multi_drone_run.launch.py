#!/usr/bin/env python

import os
import logging

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

from multi_drone.utils.methods import load_yaml_params
from multi_drone.scripts.runner import launch_robot


def generate_launch_description():    
    pkg_multi_drone = get_package_share_directory('multi_drone')
    params_file = os.path.join(pkg_multi_drone, 'config', 'test_params.yaml')    
    robots_config = load_yaml_params(params_file)

    launch_descriptions = []

    microxrce_agent = ExecuteProcess(
        cmd=[
            'MicroXRCEAgent', 'udp4', '-p', '8888'
        ],
        output='log',
        name="microxrce_agent"
    )
    launch_descriptions.append(microxrce_agent)

    for robot in robots_config['robots']:
        
        logging.info(f"Запускаем дрон {robot['drone_id']} с типом {robot['drone_type']}")

        launch_descriptions.extend(
            launch_robot(
                drone_id=robot['drone_id'],
                drone_type=robot['drone_type'],
                spawn_position=robot['position'],
                px4_dir="/workspace/src/PX4-Autopilot/",
                # terminal="gnome-terminal",
                terminal='bash',
                package_name="multi_drone",
                executables_script=robot['executables_script'],
                additional_params={}
            )
        )

    return LaunchDescription(launch_descriptions)

if __name__ == "__main__":
    generate_launch_description()