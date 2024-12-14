#!/usr/bin/env python3

from launch import LaunchDescription

from multi_drone.scripts.gazebo_server import run_simulation_gazebo

def generate_launch_description() -> LaunchDescription:
    
    # Определение пути к файлу параметров
    # pkg_share = FindPackageShare('custom_models').find('custom_models')

    world_file = 'walls'
        
    gazebo_server = run_simulation_gazebo(
        world=world_file,
        # custom_model_store=pkg_share, # если есть своя директория с моделями мира
        custom_model_store_other=[],
        headless=False,
        gz_ip=None,
        gz_partition=None
    )

    return LaunchDescription([
        gazebo_server,
    ])

if __name__ == "__main__":
    generate_launch_description()
